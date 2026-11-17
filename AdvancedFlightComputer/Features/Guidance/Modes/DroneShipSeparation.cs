#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;

// Separating a drone-ship booster where it will land on its ship, from the live state, not where the plan said.
//
// An optimal return's landing point is frozen from the plan made on the ground: a ship placed there before launch is where the booster is sent. But the booster separated on the plan's reserve cue, after minutes of real flight, and the climb never flies the plan exactly: a heavy's core separated 26 km short of its point (2026-11-17). So while a drone-ship booster is the next to separate, its impact is predicted from the stack's live state, ten times a second, with its own aero (KsaBoosterGeometry), and it separates when that impact reaches the ship, or at the reserve cue if that comes first. Leaving early, it carries more than its reserve, which is landing margin; leaving at the reserve, whatever miss is left goes to its small boostback.
//
// Only an optimal return. A booster returning to a site flies a boostback to it, and separates on its reserve as the plan sized it.
public static partial class GuidanceWindow
{
    /// <summary>How often the live impact is predicted while a drone-ship booster is next to separate, s of sim time. Near burnout the impact sweeps downrange at kilometres a second, so it is checked often and led by the rate it moves at.</summary>
    private const double DroneShipCueIntervalS = 0.1;

    /// <summary>How far ahead the crossing is led, s: one check and the frame the staging takes to land.</summary>
    private const double DroneShipCueLeadS = 0.15;

    /// <summary>
    /// The drone-ship booster next to separate, when the reserve is armed for it (RefreshReserve), and its sweep: its own aero, measured as the vehicle it becomes. Swept once per booster, when it is first armed.
    /// </summary>
    private static void PrepareDroneShipAero(Vehicle vehicle, ReturnableStage booster)
    {
        if (booster == null || !booster.OptimalReturn)
            return;
        if (_s.DroneShipAeroRoot == booster.RootId && _s.DroneShipAero != null)
            return;
        _s.DroneShipAeroRoot = booster.RootId;
        _s.DroneShipAero = null;
        _s.DroneShipLastD = double.NaN;
        _s.DroneShipRate = double.NaN;

        Part root = null;
        ReadOnlySpan<Part> parts = vehicle.Parts.Parts;
        for (int i = 0; i < parts.Length && root == null; i++)
            if (parts[i].InstanceId == booster.RootId)
                root = parts[i];
        IParentBody parent = vehicle.Orbit?.Parent;
        if (root == null || parent == null)
            return;
        if (KsaAeroSweep.TryBuildBooster(vehicle, root, PartsLeavingWith(vehicle, root, booster.SequenceIndex), parent, SimNow(),
                                         out KsaAeroSweep.Result aero, out string error))
            _s.DroneShipAero = aero;
        else
            GuidanceLog.Info(vehicle, $"drone ship: '{booster.Name}' could not be swept ({error}); it separates on its reserve.");
    }

    /// <summary>
    /// True when the drone-ship booster next to separate should separate now: its impact, predicted from the stack's live state, has reached its ship's point along the ground track, or will within the lead. False for anything that is not an armed optimal return with a frozen point, on the pad, or with no impact predicted yet.
    /// </summary>
    private static bool ShouldStageForDroneShip(Vehicle vehicle)
    {
        if (!_s.Running || !_s.ReserveArmed || _s.ReserveStaged || _s.DroneShipAero?.Table == null)
            return false;
        ReturnableStage booster = NextTargetedBooster();
        if (booster == null || !booster.OptimalReturn || booster.RootId != _s.DroneShipAeroRoot
            || !_s.OptimalReturnSites.TryGetValue(booster.RootId, out double2 ship))
            return false;

        // Not on the pad, as for the reserve's own cue.
        if (_s.Phase == AscentPhase.Vertical)
            return false;
        Orbit orbit = vehicle.Orbit;
        if (orbit?.Parent is not IParentBody parent)
            return false;
        double3 r = orbit.StateVectors.PositionCci, v = orbit.StateVectors.VelocityCci;
        if (r.Length() - parent.MeanRadius < TraceMinAltitude)
            return false;

        double now = SimNow();
        if (now - _s.DroneShipCueAt < DroneShipCueIntervalS)
            return false;
        _s.DroneShipCueAt = now;

        // The booster as it will coast: its own drag area against its share of the dry mass and the reserve it keeps.
        int onRow = 0;
        foreach (ReturnableStage st in _s.ReturnableStages)
            if (st.SequenceIndex == booster.SequenceIndex)
                onRow++;
        double mass = (_s.ReserveBoosterDryKg + _s.ReserveKg) / Math.Max(onRow, 1);
        KsaAeroSweep.Result aero = _s.DroneShipAero;
        if (!(mass > 0.0) || aero.Atmosphere == null)
            return false;
        var sys = new DragCoastSystem
        {
            Mu = parent.Mu,
            OmegaZ = parent.GetAngularVelocity(),
            MeanRadius = parent.MeanRadius,
            AreaOverMass = aero.ReferenceArea / mass,
            Alpha = GlideNominalAlpha(aero),
            Table = aero.Table,
            LiftTable = aero.Lift,
            Atmosphere = aero.Atmosphere,
        };
        var opt = ImpactOptions.Default(parent.MeanRadius);
        opt.MaxTime = ImpactHorizonMinutes * 60.0;
        opt.PathStride = 0;
        Span<Dual> x0 = stackalloc Dual[ImpactPredictor.N];
        x0[0] = r.X; x0[1] = r.Y; x0[2] = r.Z; x0[3] = v.X; x0[4] = v.Y; x0[5] = v.Z;
        _s.DroneShipScratch ??= new Dual[ImpactPredictor.ScratchLength];
        ImpactPrediction pred = ImpactPredictor.Predict(in sys, x0, in opt, _s.DroneShipScratch, Span<double>.Empty);
        if (!pred.Hit)
        {
            // Still climbing too fast to come down inside the horizon, or not yet arcing over: nothing to compare.
            _s.DroneShipLastD = double.NaN;
            _s.DroneShipRate = double.NaN;
            return false;
        }

        // Both on the ground, in the frame the prediction's ground point is in: the inertial frame of now.
        double3 hit = new double3(pred.Fx.V, pred.Fy.V, pred.Fz.V);
        double3 site = (SiteDirCcfAt(ship.X, ship.Y) * parent.MeanRadius).Transform(parent.GetCcf2Cci());
        // Along the ground track: the stack's velocity over the turning ground, flattened to the horizontal.
        double3 up = double3.Normalize(r);
        double3 vg = v - double3.Cross(new double3(0, 0, parent.GetAngularVelocity()), r);
        double3 along = vg - double3.Dot(vg, up) * up;
        if (!(along.Length() > 1e-6))
            return false;
        along = double3.Normalize(along);
        // How far the ship still lies beyond the impact, m: positive while the impact has yet to reach it.
        double d = double3.Dot(site - hit, along);
        double rate = double.IsFinite(_s.DroneShipLastD) && now > _s.DroneShipLastT ? (d - _s.DroneShipLastD) / (now - _s.DroneShipLastT) : 0.0;
        _s.DroneShipLastD = d;
        _s.DroneShipLastT = now;
        // Smoothed for the countdown (SeparationCountdown); the cue's own lead keeps the raw rate.
        if (rate != 0.0)
            _s.DroneShipRate = double.IsFinite(_s.DroneShipRate) ? _s.DroneShipRate + 0.3 * (rate - _s.DroneShipRate) : rate;
        _s.DroneShipMissM = (site - hit).Length();

        bool reached = d <= 0.0 || (rate < 0.0 && d + rate * DroneShipCueLeadS <= 0.0);
        if (reached)
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"drone ship: '{booster.Name}' separates with its impact at the ship ({d / 1000.0:F2} km along, {_s.DroneShipMissM / 1000.0:F2} km off, moving {-rate / 1000.0:F2} km/s), {_s.ReserveKg:F0} kg reserve planned."));
        return reached;
    }
}

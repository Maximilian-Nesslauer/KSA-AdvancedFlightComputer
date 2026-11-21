#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Threading;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;
using Brutal.Numerics;
using KSA;

/// <summary>
/// ORBITAL ENTRY GUIDANCE: from the air's first touch to a hand-over at about Mach 2, belly first at the pitch-trim angle of attack, banking to fly the planner's vertical lift over drag.
///
///  PLAN. The planner (Scvx/OrbitalEntry/EntryPlanner) chooses vertical L/D against energy to reach the target's range inside the heating, dynamic-pressure and load limits, with the least heat load. It runs on its own thread and is re-solved every OrbitalEntryResolveS from the craft's state, warm from the plan before.
///  TRACK. The tracker (EntryTracker) flies the plan with drag and sink-rate feedback, turns vertical L/D into bank from the measured L/D, reverses the bank's side on a crossrange deadband, and may overbank past the plan's limit for a while to stop a sink.
///  ATTITUDE. The angle of attack is the pitch-trim one at the current Mach, from a belly-first sweep of the craft taken at engage (KsaAeroSweep, bellyFirst). The nose goes at that angle above the flight path towards the lift, the belly axis faces the air, and the bank turns both about the velocity.
///  END. At the hand-over energy the craft holds its entry attitude, wings level, until the player takes it; how it flies out of the plan (a belly-flop landing or an aircraft-like approach) is for later.
/// </summary>
public static partial class GuidanceWindow
{
    public enum OrbitalEntryPhase { Idle, Sweeping, Entry, Holding }

    internal static bool OrbitalEntryLive => _s.OrbitalEntry.Phase is OrbitalEntryPhase.Sweeping or OrbitalEntryPhase.Entry or OrbitalEntryPhase.Holding;

    private static bool OrbitalEntryLiveFor(VehicleAutopilotState s)
        => s.OrbitalEntry.Phase is OrbitalEntryPhase.Sweeping or OrbitalEntryPhase.Entry or OrbitalEntryPhase.Holding;

    private static readonly EntryPlanner _entryPlanner = new();

    /// <summary>The config the planner and tracker fly with, from the craft's settings.</summary>
    private static EntryConfig OrbitalEntryConfig() => new()
    {
        HeatingRateMax = _s.OrbitalEntryHeatMaxKWm2 * 1000.0,
        DynamicPressureMax = _s.OrbitalEntryQMaxKPa * 1000.0,
        LoadFactorMax = _s.OrbitalEntryGMax,
        BankMinDeg = _s.OrbitalEntryBankMinDeg,
        BankMaxDeg = _s.OrbitalEntryBankMaxDeg,
        OverbankDeg = Math.Max(_s.OrbitalEntryOverbankDeg, _s.OrbitalEntryBankMaxDeg),
        OverbankMaxS = _s.OrbitalEntryOverbankS,
        RollRateDegS = Math.Max(_s.OrbitalEntryRollRateDegS, 0.5),
        StartDrag = _s.OrbitalEntryStartDragMs2,
        HandoverAltitude = _s.OrbitalEntryHandoverAltKm * 1000.0,
        HandoverMach = _s.OrbitalEntryHandoverMach,
    };

    /// <summary>Take the craft for orbital entry. Draw code only sets OrbitalEntry.EngagePending; this runs from the step, ahead of the claim.</summary>
    private static bool EngageOrbitalEntry(Vehicle vehicle, double now)
    {
        var st = _s.OrbitalEntry;
        if (!vehicle.IsControllable)
            return RefuseOrbitalEntry(vehicle, vehicle.NoControlTooltip.ToString());
        IParentBody parent = vehicle.Orbit?.Parent;
        if (parent?.GetPhysicalAtmosphere() == null)
            return RefuseOrbitalEntry(vehicle, "NO ATMOSPHERE.");
        if (vehicle.ComputeLiveThrust() > 0.0)
            return RefuseOrbitalEntry(vehicle, "AN ENGINE IS BURNING.");
        if (!KsaAeroSweep.TryStart(vehicle, parent, now, out KsaAeroSweep.Job job, out string error, bellyFirst: true))
            return RefuseOrbitalEntry(vehicle, "AERO SWEEP FAILED: " + error.ToUpperInvariant() + ".");

        if (IsStockDeorbitPhase(_s.LandingPhase))
            StopGuidance(vehicle, StopTarget.Landing);
        ClaimVehicle(GuidanceMode.OrbitalEntry, vehicle);
        _s.Engage = true;
        st.Reset();
        st.AeroJob = job;
        st.Tracker = new EntryTracker(OrbitalEntryConfig());
        st.LastStep = now;
        st.Phase = OrbitalEntryPhase.Sweeping;
        st.SetStatus("MEASURING THE CRAFT BELLY FIRST.");
        GuidanceLog.Info(vehicle, "Orbital entry guidance engaged; belly-first aero sweep started.");
        return true;
    }

    private static bool RefuseOrbitalEntry(Vehicle vehicle, string reason)
    {
        string status = "ORBITAL ENTRY REFUSED: " + reason;
        if (_s.OrbitalEntry.Status != status)
            GuidanceLog.Info(vehicle, status);
        _s.OrbitalEntry.SetStatus(status, warn: true);
        return false;
    }

    /// <summary>Stop orbital entry: its solver and attitude let go, the engine left as it is.</summary>
    private static void StopOrbitalEntry(Vehicle vehicle, string why)
    {
        var st = _s.OrbitalEntry;
        if (_s.OwnAttitudeFlying && _s.OwnAttitudeVehicle is { } flown)
            StopOwnAttitude(flown);
        st.Reset();
        st.Phase = OrbitalEntryPhase.Idle;
        st.SetStatus(why);
        _s.HasCommand = false;
        _s.ReleaseWithoutEngineCut = true;
        if (vehicle != null)
            GuidanceLog.Info(vehicle, "Orbital entry guidance stopped: " + why);
    }

    /// <summary>The release path's reset: no log, no status change.</summary>
    private static void ResetOrbitalEntry()
    {
        _s.OrbitalEntry.Reset();
        _s.OrbitalEntry.Phase = OrbitalEntryPhase.Idle;
        _s.OrbitalEntry.EngagePending = false;
    }

    /// <summary>One step of orbital entry guidance, from ApplyAutopilot.</summary>
    private static void StepOrbitalEntry(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        var st = _s.OrbitalEntry;
        if (!OrbitalEntryLive)
            return;
        if (st.StopPending)
        {
            StopOrbitalEntry(vehicle, "ORBITAL ENTRY STOPPED.");
            return;
        }
        double now = SimNow();
        double dt = double.IsNaN(st.LastStep) ? 0.0 : Math.Clamp(now - st.LastStep, 0.0, 1.0);
        st.LastStep = now;

        if (st.Phase == OrbitalEntryPhase.Sweeping)
        {
            _s.HasCommand = false;
            if (st.AeroJob == null || !st.AeroJob.TryTake(out KsaAeroSweep.Result aero, out string error))
                return;
            st.AeroJob = null;
            if (aero?.Belly == null || aero.Atmosphere == null)
            {
                StopOrbitalEntry(vehicle, "ORBITAL ENTRY REFUSED: NO BELLY-FIRST AERO (" + (error.Length > 0 ? error.ToUpperInvariant() : "NO AIR") + ").");
                return;
            }
            st.Aero = aero;
            if (!BuildOrbitalEntryTrim(st, out string why))
            {
                StopOrbitalEntry(vehicle, "ORBITAL ENTRY REFUSED: " + why);
                return;
            }
            st.Worker = new Ksa3DofWorker();
            st.Phase = OrbitalEntryPhase.Entry;
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"Orbital entry: belly axis {st.BellyAxisName}, {st.Trim.TrimmedRows}/{st.Trim.Mach.Length} Mach rows trimmed, alpha {st.Trim.AlphaAt(20):F0} deg at Mach 20, {st.Trim.AlphaAt(5):F0} at Mach 5, L/D {st.Trim.LiftToDragAt(20):F2} / {st.Trim.LiftToDragAt(5):F2}."));
        }

        // ---- where the craft is ----
        double3 r = orbit.StateVectors.PositionCci;
        double3 vRel = orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), r);
        double rMag = r.Length(), vMag = vRel.Length();
        if (vMag < 1.0)
            return;
        double3 rHat = r / rMag, vHat = vRel / vMag;
        double R0 = parent.MeanRadius;
        double alt = rMag - R0;
        double gamma = Math.Asin(Math.Clamp(double3.Dot(vHat, rHat), -1.0, 1.0));
        double3 siteHat = double3.Normalize(SiteDirCciAt(parent, 0));
        double rangeRad = Math.Acos(Math.Clamp(double3.Dot(rHat, siteHat), -1.0, 1.0));
        double3 leftHat = double3.Cross(r, vRel).NormalizeOrZero();
        double crossRad = Math.Asin(Math.Clamp(double3.Dot(leftHat, siteHat), -1.0, 1.0));

        var atm = st.Aero.Atmosphere;
        double mach = atm.Mach(vMag);
        st.SpeedMs = vMag; st.AltitudeM = alt; st.RangeM = rangeRad * R0; st.CrossrangeM = crossRad * R0;
        st.HdotMs = vMag * Math.Sin(gamma); st.Mach = mach;

        // ---- the air the craft is in: the game's own aero force on it now ----
        double drag = 0.0, ld = st.Trim.LiftToDragAt(mach);
        if (KsaAeroSweep.TryLiveAero(vehicle, out double3 forceBody, out _, dt))
        {
            double3 f = forceBody.Transform(KsaFrameBridge.BodyToCci(vehicle));
            double m = Math.Max(vehicle.TotalMass, 1.0);
            drag = Math.Max(-double3.Dot(f, vHat) / m, 0.0);
            double liftAccel = (f + drag * m * vHat).Length() / m;
            if (drag > 0.3)
                ld = liftAccel / drag;
        }
        st.DragMs2 = drag; st.LiftToDrag = ld;

        EntryModel nominal = OrbitalEntryModel(vehicle, parent, st, 1.0, 1.0);
        double modelDrag = nominal.DragAccel(alt, vMag);
        if (drag > 0.3 && modelDrag > 0.1)
        {
            double k = Math.Clamp(dt / 5.0, 0.0, 1.0);
            st.DensityScale += k * (Math.Clamp(drag / modelDrag, 0.5, 2.0) - st.DensityScale);
            double ldModel = st.Trim.LiftToDragAt(mach);
            if (Math.Abs(ldModel) > 0.05)
                st.LiftToDragScale += k * (Math.Clamp(ld / ldModel, 0.5, 1.5) - st.LiftToDragScale);
        }

        EntryConfig cfg = OrbitalEntryConfig();
        double e = EntryDynamics.Energy(rMag / R0, vMag / nominal.SpeedUnit);
        st.Energy = e;
        double ef = EntryPlanner.FinalEnergy(nominal, cfg);

        // ---- plans: take what the worker published, ask for the next ----
        if (Interlocked.Exchange(ref st.Pending, null) is { } res)
        {
            st.Solves++;
            if (res.Plan != null && (res.Plan.Converged || st.Plan == null))
            {
                st.Plan = res.Plan;
                st.LastSolveMs = res.Plan.SolveMs;
                if (!res.Plan.Converged)
                    st.LastRefusal = res.Why;
            }
            else
            {
                st.Refusals++;
                st.LastRefusal = res.Why.Length > 0 ? res.Why : "no plan";
            }
        }
        if (st.Phase == OrbitalEntryPhase.Entry && e < ef && now >= st.NextSolveAt && st.Worker != null && !st.Worker.Busy)
            QueueOrbitalEntrySolve(vehicle, parent, st, cfg, nominal, e, rMag, vMag, gamma, rangeRad, drag, now);

        // ---- hand-over ----
        if (st.Phase == OrbitalEntryPhase.Entry && e >= ef)
        {
            st.Phase = OrbitalEntryPhase.Holding;
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"Orbital entry hand-over: Mach {mach:F2} at {alt / 1000:F1} km, {st.RangeM / 1000:F1} km to go, {st.CrossrangeM / 1000:F1} km across."));
        }

        // ---- bank ----
        double bankDeg;
        if (st.Phase == OrbitalEntryPhase.Holding)
        {
            // Wings level, still at the trim angle: the hand-over keeps the entry attitude.
            double step = cfg.RollRateDegS * dt;
            double b = st.Tracker.BankDeg;
            bankDeg = Math.Abs(b) <= step ? 0.0 : b - Math.Sign(b) * step;
            st.Tracker.Reset();
            st.SetStatus(FormattableString.Invariant($"HAND-OVER: MACH {mach:F1} AT {alt / 1000:F1} KM, {st.RangeM / 1000:F0} KM TO GO. HOLDING ENTRY ATTITUDE."));
        }
        else
        {
            int side = st.Tracker.Side;
            var outp = st.Tracker.Update(new EntryTracker.Input(dt, e, drag, st.HdotMs, ld, atm.ScaleHeightAt(Math.Max(alt, 0.0)),
                                                                crossRad, vMag / nominal.SpeedUnit, st.Plan));
            if (side != 0 && outp.Side != side) st.Reversals++;
            st.Last = outp;
            bankDeg = outp.BankDeg;
            string status = outp.Status;
            if (st.Plan == null)
                status = double.IsNaN(st.PredictedStartS) ? "PLANNING THE ENTRY." : FormattableString.Invariant($"PLANNING THE ENTRY. AIR IN {st.PredictedStartS:F0} s.");
            else if (!st.Plan.Converged)
                status += " (PLAN NOT CONVERGED: " + st.LastRefusal.ToUpperInvariant() + ")";
            st.SetStatus(status, warn: outp.Overbanked || outp.LiftUpLimit || outp.LiftDownLimit || (st.Plan != null && !st.Plan.Converged));
        }

        // ---- attitude ----
        double alpha = st.Trim.AlphaAt(mach) * Math.PI / 180.0;
        st.AlphaDeg = alpha * 180.0 / Math.PI;
        double3 liftUp = (rHat - double3.Dot(rHat, vHat) * vHat).NormalizeOrZero();
        double3 right = double3.Cross(vHat, liftUp);
        double sigma = bankDeg * Math.PI / 180.0;
        double3 lift = Math.Cos(sigma) * liftUp + Math.Sin(sigma) * right;
        double3 nose = Math.Cos(alpha) * vHat + Math.Sin(alpha) * lift;
        double3 belly = Math.Sin(alpha) * vHat - Math.Cos(alpha) * lift;
        st.RollRefCci = st.BellyIndex switch
        {
            0 => double3.Cross(-belly, nose),     // belly is body -Z: body z = -belly, y = z x x
            1 => double3.Cross(belly, nose),      // +Z
            2 => -belly,                          // -Y
            _ => belly,                           // +Y
        };
        _s.CommandDir = nose;
        _s.CommandRate = double3.Zero;
        _s.HasCommand = true;

        // ---- the TRAJ window's history ----
        if (now >= st.NextTraceAt)
        {
            st.NextTraceAt = now + 1.0;
            st.Trace.Add(new OrbitalEntryState.TrajSample(vMag, st.RangeM, drag, st.Last.DragRefMs2, bankDeg, alt));
            if (st.Trace.Count > 4000)
                st.Trace.RemoveRange(0, 1000);
        }
    }

    private static void QueueOrbitalEntrySolve(Vehicle vehicle, IParentBody parent, OrbitalEntryState st, EntryConfig cfg, EntryModel nominal,
                                               double e, double rMag, double vMag, double gamma, double rangeRad, double drag, double now)
    {
        EntryModel model = nominal.WithScales(st.DensityScale, st.LiftToDragScale);
        double R0 = model.BodyRadius;
        double e0;
        double[] x0;
        if (drag >= cfg.StartDrag)
        {
            e0 = e;
            x0 = [rMag / R0, gamma, rangeRad];
            st.PredictedStartS = double.NaN;
            st.NextSolveAt = now + Math.Max(_s.OrbitalEntryResolveS, 0.25);
        }
        else
        {
            var start = EntryPlanner.PredictStart(model, cfg, rMag, vMag, gamma, rangeRad);
            if (start == null)
            {
                st.SetStatus("NOT ON AN ENTRY TRAJECTORY: THE CRAFT DOES NOT MEET THE AIR.", warn: true);
                st.NextSolveAt = now + 10.0;
                return;
            }
            e0 = start.Value.E;
            x0 = start.Value.X;
            st.PredictedStartS = start.Value.SecondsAhead;
            st.NextSolveAt = now + 10.0;
        }
        EntryPlan warm = st.Plan;
        st.Worker.TryRun(st, ct =>
        {
            st.Pending = _entryPlanner.Solve(model, cfg, e0, x0, warm);
        });
    }

    /// <summary>
    /// The planner's model of this craft: the sampled atmosphere, the trim schedule's drag and L/D on the sweep's reference area, the craft's mass now. Immutable and thread-safe, so the worker can fly it.
    /// </summary>
    private static EntryModel OrbitalEntryModel(Vehicle vehicle, IParentBody parent, OrbitalEntryState st, double densityScale, double ldScale)
    {
        var atm = st.Aero.Atmosphere;
        EntryTrim trim = st.Trim;
        return new EntryModel
        {
            BodyRadius = parent.MeanRadius,
            Mu = parent.Mu,
            Density = h => atm.Density(h),
            SpeedOfSound = _ => atm.SpeedOfSound,
            AtmosphereTop = atm.TopAltitude,
            DragCoefficient = trim.CdAt,
            LiftToDrag = trim.LiftToDragAt,
            Mass = Math.Max(vehicle.TotalMass, 1.0),
            ReferenceArea = st.Aero.ReferenceArea,
            NoseRadius = Math.Max(_s.OrbitalEntryNoseRadiusM, 0.05),
            DensityScale = densityScale,
            LiftToDragScale = ldScale,
        };
    }

    /// <summary>The angle-of-attack schedule from the belly-first sweep: the chosen belly axis, or the one of the four with the most trimmed Mach rows and then the best lift.</summary>
    private static bool BuildOrbitalEntryTrim(OrbitalEntryState st, out string why)
    {
        why = "";
        var sweeps = st.Aero.Belly;
        int choice = (int)Math.Round(_s.OrbitalEntryBellyAxis);
        EntryTrim Make(KsaAeroSweep.BellySweep b) => _s.OrbitalEntryTrimAoa
            ? EntryTrim.Build(b.MachGrid, b.AlphaDeg, b.Cd, b.Cl, b.Cm, _s.OrbitalEntryAoaMinDeg, _s.OrbitalEntryAoaMaxDeg, _s.OrbitalEntryAoaDeg)
            : EntryTrim.Fixed(b.MachGrid, b.AlphaDeg, b.Cd, b.Cl, _s.OrbitalEntryAoaDeg);
        int best = -1;
        EntryTrim bestTrim = null;
        double bestScore = double.NegativeInfinity;
        for (int i = 0; i < sweeps.Length; i++)
        {
            if (choice >= 1 && choice <= 4 && i != choice - 1)
                continue;
            EntryTrim t = Make(sweeps[i]);
            double meanLd = 0;
            for (int j = 0; j < t.Mach.Length; j++) meanLd += t.LiftToDrag[j];
            meanLd /= Math.Max(t.Mach.Length, 1);
            double score = t.TrimmedRows * 10.0 + meanLd;
            if (score > bestScore) { bestScore = score; best = i; bestTrim = t; }
        }
        if (bestTrim == null)
        {
            why = "NO BELLY AXIS TO FLY.";
            return false;
        }
        if (bestTrim.LiftToDragAt(10) <= 0.02)
        {
            why = FormattableString.Invariant($"NO LIFT BELLY FIRST ON {sweeps[best].AxisName} (L/D {bestTrim.LiftToDragAt(10):F2} AT MACH 10).");
            return false;
        }
        st.Trim = bestTrim;
        st.BellyIndex = best;
        st.BellyAxisName = sweeps[best].AxisName;
        return true;
    }
}

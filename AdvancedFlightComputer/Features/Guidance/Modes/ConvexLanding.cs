#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Threading.Tasks;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

// The 3-DOF glide and landing burn, flown from inside the boostback machine (Scvx/ThreeDof/Guidance3Dof.cs is the guidance itself).
//
// ENGAGE MID-GLIDE. The PID glide flies from boostback cutoff. Once its predicted time to impact falls under ConvexEngageTgoS, the cold glide-and-burn solve starts on a worker thread, and the PID keeps flying until it has a plan; from then the plan's attitude is what the glide flies. Above that the air is too thin to plan anything worth having, and the horizon would force coarse nodes through the entry.
//
// IGNITION is the plan's. The guidance commits it a few seconds early and swaps to the burn-only problem from the state predicted at ignition (see Guidance3Dof), so nothing changes structure at the moment the engine lights. When the published plan's ignition time arrives the machine moves to LandingBurn and flies the plan's attitude and throttle.
//
// FALLBACKS. Before the burn: a cold solve that fails, or a run of refused re-solves, hands the attitude back to the PID glide and starts a fresh cold solve. After ignition the glide cannot catch it, so a plan that runs out, or a long run of refusals, hands the burn to 6-DOF, which flies in air on its own model.
//
// HANDOVERS. At the aim point - ConvexAimHeightM over the site, sinking ConvexAimSinkMs - terminal hover lands it. Optionally, below ConvexSixDofSpeedMs, 6-DOF takes the burn instead, for its rotational model of the final flare.
//
// TODO (offline harness, --3dof-mpc): two cases still fail. A 2 km cross-range engage does not converge its cold solve with resized scales, and a vehicle with 30 % less lift than its table misses by 20-30 m. The 6-DOF handover is not pre-warmed: 6-DOF starts its cold solve when the speed threshold is crossed.
public static partial class GuidanceWindow
{
    /// <summary>Re-solve cadence, sim seconds.</summary>
    private const double ConvexUpdateS = 0.1;

    /// <summary>Refused re-solves in a row before the glide falls back to the PID and starts again cold, and before a lit burn goes to 6-DOF.</summary>
    private const int ConvexGlideRefusalLimit = 20;
    private const int ConvexBurnRefusalLimit = 10;

    /// <summary>Wait between cold-solve attempts after one fails, s.</summary>
    private const double ConvexColdRetryS = 2.0;

    /// <summary>Attitude rate the plans may ask of the vehicle, rad/s.</summary>
    private const double ConvexRateMax = 10.0 * Math.PI / 180.0;

    private static bool ConvexBusy => _s.ConvexWorker != null && _s.ConvexWorker.Busy;

    /// <summary>
    /// One step of the 3-DOF guidance while the boostback machine glides or burns. Returns the thrust-axis direction to fly, CCI, or false when the PID glide should fly this step instead.
    /// </summary>
    private static bool StepConvexLanding(Vehicle vehicle, Orbit orbit, IParentBody parent, double now, out double3 wantCci)
    {
        wantCci = default;
        if (!_s.ConvexLanding)
            return false;
        if (_s.ConvexWorker?.LastError is { Length: > 0 } fault)
        {
            GuidanceLog.Info(vehicle, "3-DOF solve faulted: " + fault);
            _s.ConvexWorker.LastError = "";
            DropConvex("3-DOF solve faulted - PID glide, retrying cold.", now);
        }

        KsaAeroSweep.Result aero = _s.Aero;
        if (aero?.Table == null || aero.Atmosphere == null)
        {
            _s.ConvexStatus = "No aero surrogate yet.";
            return false;
        }

        double3 siteCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        KsaFrameBridge.SiteFrame frame = KsaFrameBridge.BuildSiteFrame(siteCci);
        double[] x14 = KsaFrameBridge.ToModelState(vehicle, frame);
        double[] x = [x14[0], x14[1], x14[2], x14[3], x14[4], x14[5], vehicle.TotalMass];
        double3 attLocal = frame.VecToLocal(ThrustAxisCci(vehicle)).NormalizeOrZero();
        double[] att = [attLocal.X, attLocal.Y, attLocal.Z];
        bool burning = _s.BoostbackPhase == BoostbackPhase.LandingBurn;

        // What the model is missing, learned from what the vehicle feels: aerodynamics with the engine off, thrust with it on.
        _s.ConvexNominal ??= BuildConvexModel(vehicle, parent, frame, aero);
        _s.ConvexEstimator ??= new AeroScaleEstimator();
        if (_s.ConvexNominal != null)
            MeasureConvex(x, att, burning ? _s.ConvexThrottle : 0.0, now);

        Guidance3Dof g = _s.ConvexGuidance;
        if (g == null)
        {
            // The gate: close enough to impact for the air to be worth planning in.
            if (!burning && double.IsFinite(_s.GlideTgo) && _s.GlideTgo <= _s.ConvexEngageTgoS
                && now - _s.ConvexColdTime >= ConvexColdRetryS && _s.ConvexNominal != null)
                StartConvexCold(vehicle, x, now);
            else if (!burning)
                _s.ConvexStatus = double.IsFinite(_s.GlideTgo)
                    ? $"PID glide; 3-DOF engages at {_s.ConvexEngageTgoS:F0} s to impact ({_s.GlideTgo:F0} s now)."
                    : "PID glide; waiting for an impact prediction.";
            return false;
        }

        if (!ConvexBusy)
        {
            // Results are read only while the worker is idle: the guidance is the worker's while it solves.
            if (g.Phase == Phase3Dof.Idle || (g.Phase == Phase3Dof.Glide && g.ConsecutiveRefusals >= ConvexGlideRefusalLimit))
            {
                if (burning)
                {
                    ConvexToSixDof(vehicle, now, "3-DOF lost its plan in the burn - 6-DOF takes over.");
                    return false;
                }
                DropConvex(g.Phase == Phase3Dof.Idle
                    ? "3-DOF " + g.Error + " - PID glide, retrying cold."
                    : $"3-DOF refused {g.ConsecutiveRefusals} re-solves - PID glide, retrying cold.", now);
                return false;
            }
            if (burning && g.ConsecutiveRefusals >= ConvexBurnRefusalLimit)
            {
                ConvexToSixDof(vehicle, now, $"3-DOF refused {g.ConsecutiveRefusals} re-solves in the burn - 6-DOF takes over.");
                return false;
            }
            if (g.Phase != Phase3Dof.Converging && now - _s.ConvexLastUpdate >= ConvexUpdateS)
            {
                _s.ConvexLastUpdate = now;
                KsaPointMassModel model = _s.ConvexEstimator.Apply(_s.ConvexNominal);
                double[] x0 = x, a0 = att;
                _s.ConvexWorker.TryRun(() =>
                {
                    g.Model = model;
                    g.Update(x0, a0, now);
                });
            }
        }

        Plan3Dof plan = g.Published;
        if (plan == null)
        {
            _s.ConvexStatus = "PID glide while the 3-DOF cold solve runs.";
            return false;
        }

        // Past the end with the engine lit and nothing newer: the plan is no longer guidance.
        if (burning && now > plan.EndTime + 0.5)
        {
            ConvexToSixDof(vehicle, now, "3-DOF plan ran out in the burn - 6-DOF takes over.");
            return false;
        }

        double height = x[2];
        double speed = Math.Sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
        if (burning)
        {
            if (_s.ConvexSixDofHandover && speed < _s.ConvexSixDofSpeedMs)
            {
                ConvexToSixDof(vehicle, now, $"3-DOF hands the burn to 6-DOF below {_s.ConvexSixDofSpeedMs:F0} m/s.");
                return false;
            }
            if (height <= _s.ConvexAimHeightM + 1.0 || now >= plan.EndTime - 0.2)
            {
                if (TerminalHoverAvailable(vehicle, out _))
                {
                    GuidanceLog.Info(vehicle, $"3-DOF burn reached the aim point at {height:F0} m, {speed:F1} m/s - terminal hover lands it.");
                    ShutDownConvex();
                    StartTerminalHover(vehicle);
                    _s.LandingStatus = "3-DOF handoff to terminal hover.";
                    return false;
                }
            }
        }

        Span<double> xs = stackalloc double[PointMass3Dof.NX];
        Span<double> us = stackalloc double[PointMass3Dof.NU];
        plan.Sample(now, xs, us);
        double3 b = new(us[0], us[1], us[2]);
        if (b.Length() < 0.5)
            return false;
        wantCci = frame.VecToCci(b.NormalizeOrZero());

        if (!burning && now >= plan.IgnitionTime)
        {
            GuidanceLog.Info(vehicle, $"3-DOF ignition at {height:F0} m, {speed:F0} m/s.");
            EnterBoostbackPhase(BoostbackPhase.LandingBurn, now);
            burning = true;
        }

        if (burning)
        {
            // The plan asks for a throttle fraction of its own nozzle law; invert the game's real thrust curve for the newtons that law gives.
            KsaPointMassModel model = g.Model;
            double altitude = model.Altitude(x);
            double pa = KsaEnginePerf.AmbientPressureAt(parent, altitude);
            double demand = model.Thrust(Math.Max(us[3], 0.0), altitude);
            double throttle = KsaEnginePerf.ThrottleForThrust(vehicle, demand, pa);
            _s.ConvexThrottle = Math.Clamp(throttle >= 0.0 ? throttle : us[3], 0.0, 1.0);
        }

        _s.ConvexStatus = $"3-DOF {plan.Phase.ToString().ToLowerInvariant()}: "
            + (burning ? $"burn {plan.EndTime - now:F1} s left, throttle {_s.ConvexThrottle:P0}"
                       : $"ignition in {plan.IgnitionTime - now:F1} s")
            + $"; drag x{_s.ConvexEstimator.DragScale:F2} lift x{_s.ConvexEstimator.LiftScale:F2} thrust x{_s.ConvexEstimator.ThrustScale:F2}"
            + (g.Error.Length > 0 && !ConvexBusy ? $"; {g.Error}" : "");
        return true;
    }

    private static void StartConvexCold(Vehicle vehicle, double[] x, double now)
    {
        _s.ConvexColdTime = now;
        Scvx3DofConfig glide = ConvexConfig(vehicle, nodes: 30, glideIntervals: 12);
        Scvx3DofConfig burn = ConvexConfig(vehicle, nodes: 20, glideIntervals: 0);
        var g = new Guidance3Dof(glide, burn, _s.ConvexEstimator.Apply(_s.ConvexNominal));
        _s.ConvexGuidance = g;
        _s.ConvexWorker ??= new Ksa3DofWorker();
        double[] xf = [0, 0, _s.ConvexAimHeightM, 0, 0, -_s.ConvexAimSinkMs];
        _s.ConvexStatus = "3-DOF cold solve started.";
        GuidanceLog.Info(vehicle, $"3-DOF cold solve at {x[2] / 1000:F1} km, {_s.GlideTgo:F0} s to impact.");
        _s.ConvexWorker.TryRun(() =>
        {
            if (!g.BeginCold(x, xf, now, new Seed3Dof.Options()))
                return;
            while (g.Phase == Phase3Dof.Converging)
                g.StepCold(10);
        });
    }

    private static Scvx3DofConfig ConvexConfig(Vehicle vehicle, int nodes, int glideIntervals)
    {
        var min = new double[PointMass3Dof.NX];
        var max = new double[PointMass3Dof.NX];
        Array.Fill(min, double.NegativeInfinity);
        Array.Fill(max, double.PositiveInfinity);
        min[PointMass3Dof.IR + 2] = 0.0;
        min[PointMass3Dof.IM] = Math.Max(vehicle.TotalMass - vehicle.PropellantMass, 1.0);
        return new Scvx3DofConfig
        {
            Nodes = nodes,
            GlideIntervals = glideIntervals,
            ThrottleFloor = Math.Clamp(vehicle.GetMinThrottle(), 0.01, 0.95),
            AlphaMaxDeg = Math.Clamp(_s.GlideMaxAoaDeg, 1.0, 45.0),
            StateMin = min,
            StateMax = max,
            TerminalAttitude = [0, 0, 1],
            AttitudeRateMax = ConvexRateMax,
            AttitudeAnchor = glideIntervals == 0,
            PathSlackWeight = 1e3,
            TerminalMissWeight = 1e3,
            TerminalSpeedWeight = 1e4,
            GlideSigmaMax = 400,
            BurnSigmaMin = 1,
            BurnSigmaMax = 120,
            ProximalWeight = 0.0,
        };
    }

    /// <summary>
    /// The planner's model of this vehicle over this body, in the site frame: central gravity and the frame's rotation from the body, the aerodynamics from the boostback's sweep, and the lit engines' thrust law measured from the game at vacuum and at sea level.
    /// </summary>
    private static KsaPointMassModel BuildConvexModel(Vehicle vehicle, IParentBody parent, KsaFrameBridge.SiteFrame frame,
                                                      KsaAeroSweep.Result aero)
    {
        (double vacuum, double flow) = KsaEnginePerf.UncappedAtPressure(vehicle, 0.0);
        if (!(vacuum > 0.0) || !(flow > 0.0))
        {
            _s.ConvexStatus = "No active, supplied engine to plan a landing burn with.";
            return null;
        }
        double p0 = aero.Atmosphere.SeaLevelPressure;
        double seaLevel = KsaEnginePerf.UncappedAtPressure(vehicle, p0).thrust;
        double exitArea = p0 > 0.0 ? Math.Max((vacuum - seaLevel) / p0, 0.0) : 0.0;

        double3 centre = frame.PosToLocal(double3.Zero);
        double3 omega = frame.VecToLocal(parent.GetAngularVelocityCci());
        AeroTable lift = aero.LiftTable != null
            ? KsaPointMassModel.LiftSlopeTable(aero.MachGrid, aero.AlphaGridDeg, aero.LiftTable)
            : null;
        return new KsaPointMassModel
        {
            Mu = parent.Mu,
            MeanRadius = parent.MeanRadius,
            CentreX = centre.X, CentreY = centre.Y, CentreZ = centre.Z,
            OmegaX = omega.X, OmegaY = omega.Y, OmegaZ = omega.Z,
            Atmosphere = aero.Atmosphere,
            Drag = aero.Table,
            LiftSlope = lift,
            ReferenceArea = aero.ReferenceArea,
            VacuumThrust = vacuum,
            ExitArea = exitArea,
            MassFlow = flow,
        };
    }

    /// <summary>
    /// Feed the estimator the non-gravitational acceleration, differenced from the site-frame velocity over the re-solve interval less the model's gravity and frame terms.
    /// </summary>
    private static void MeasureConvex(double[] x, double[] att, double throttle, double now)
    {
        double3 v = new(x[3], x[4], x[5]);
        double dt = now - _s.ConvexAccelTime;
        if (!double.IsFinite(_s.ConvexAccelTime) || dt <= 0.0 || dt > 1.0)
        {
            _s.ConvexAccelTime = now;
            _s.ConvexAccelVelocity = v;
            return;
        }
        if (dt < ConvexUpdateS)
            return;

        double3 a = (v - _s.ConvexAccelVelocity) / dt;
        Span<double> f = stackalloc double[PointMass3Dof.NX];
        KsaPointMassModel bare = _s.ConvexNominal with { Drag = null, LiftSlope = null };
        PointMass3Dof.Eval(bare, x, [att[0], att[1], att[2], 0.0], f);
        double[] measured = [a.X - f[3], a.Y - f[4], a.Z - f[5]];
        _s.ConvexEstimator.Update(_s.ConvexNominal, x, att, throttle, measured, dt);
        _s.ConvexAccelTime = now;
        _s.ConvexAccelVelocity = v;
    }

    /// <summary>Back to the PID glide, and a fresh cold solve after the retry wait.</summary>
    private static void DropConvex(string why, double now)
    {
        _s.ConvexStatus = why;
        _s.ConvexColdTime = now;
        if (!ConvexBusy)
            _s.ConvexGuidance = null;
    }

    private static void ConvexToSixDof(Vehicle vehicle, double now, string why)
    {
        GuidanceLog.Info(vehicle, why);
        ShutDownConvex();
        _s.ConvexStatus = why;
        Engage6Dof(vehicle);
    }

    /// <summary>Forget the guidance and the model; the worker finishes whatever it is on and is left for the next engage.</summary>
    private static void ShutDownConvex()
    {
        _s.ConvexGuidance = null;
        _s.ConvexNominal = null;
        _s.ConvexEstimator = null;
        _s.ConvexAccelTime = double.NaN;
        _s.ConvexThrottle = 0.0;
    }

    /// <summary>Reset at boostback start, so a new flight does not inherit the last one's plan or corrections.</summary>
    private static void ResetConvex()
    {
        ShutDownConvex();
        _s.ConvexLastUpdate = _s.ConvexColdTime = double.NegativeInfinity;
        _s.ConvexStatus = "";
    }
}

/// <summary>
/// One 3-DOF solve at a time, off the sim thread. The guidance belongs to the job while it runs; the sim thread reads only the immutable plan it publishes, and calls nothing on the guidance until the job is done.
/// </summary>
public sealed class Ksa3DofWorker
{
    private Task _task;

    public bool Busy => _task is { IsCompleted: false };

    /// <summary>The last job's exception, for the log; cleared by the reader.</summary>
    public string LastError = "";

    public bool TryRun(Action job)
    {
        if (Busy)
            return false;
        _task = Task.Run(() =>
        {
            try { job(); }
            catch (Exception e) { LastError = e.ToString(); }
        });
        return true;
    }
}

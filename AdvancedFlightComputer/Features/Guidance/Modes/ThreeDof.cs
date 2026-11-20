#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

// The 3-DOF glide-and-burn landing, a guidance mode beside 6-DOF (Scvx/ThreeDof/Guidance3Dof.cs is the guidance; Adapters/Scvx/ThreeDof the game side; Ui/Gauges/ThreeDofGauge.cs its page).
//
// ENGAGE MID-GLIDE. A boostback glide hands the craft over once its predicted time to impact falls under ThreeDofEngageTgoS, or the page engages it directly. The cold glide-and-burn solve then runs on a worker thread while this mode flies the boostback's PID glide from the same glide state, so the hand-over does not move the command; the first plan takes over from there. Above that point the air is too thin to plan anything worth having, and the horizon would force coarse nodes through the entry.
//
// IGNITION is the plan's. The guidance commits it a few seconds early and swaps to the burn-only problem from the state predicted at ignition (see Guidance3Dof), so nothing changes structure at the moment the engine lights.
//
// THE COMMAND IS SMOOTH BY CONSTRUCTION, which the first flights showed it has to be. Four things keep it so:
//  - each re-solve anchors its opening attitude on the attitude last COMMANDED, a cone a few degrees wide, so consecutive plans start where the command already is rather than where the lagging vehicle happens to point;
//  - the plan's attitude and throttle reach the vehicle through a first-order low-pass, ThreeDofSmoothTau, as G-FOLD's command does. A new plan arrives about every tenth of a second; the filter is continuous whatever that rate. (A blend from the old plan to the new over a fixed window was tried first and was worse than nothing: each new plan cut the last blend off a third of the way in and restarted from the OLD plan's attitude, so the command stepped back at every re-solve.)
//  - the turn-rate feed-forward is the newest plan's own rate of turn, never a difference of successive commands, which would turn every re-solve's small step into a rate spike;
//  - while ignition is locked, the burn plan's opening attitude lies a few seconds ahead, and the command turns towards it at the rate limit the ignition prediction assumed, rather than jumping to it: that jump was a 15-degree swing three seconds before ignition.
//
// THE BURN IS TRACKED, not just played back. The plan is re-solved from the measured state ten times a second, but between re-solves, and through a run of refused ones, it is open loop, and a min-fuel burn rides its throttle ceiling with nothing in hand. So the plan's throttle stops short of full - ThreeDofGlideCeiling before ignition is committed, ThreeDofBurnCeiling after - and BurnTracker3Dof flies it with PD feedback on its position and velocity: the correction along the axis as throttle, up to the engine's real maximum, and across it as a tilt of at most ThreeDofTrackTiltDeg, since the axis is also the attitude the aerodynamics were planned around.
//
// Before ignition a failed solve returns to the PID glide and retries. After ignition the entry tracker holds the burn until a fresh touchdown plan can take it.
//
// THE BURN ENDS AT ZERO/ZERO - the legs on the pad, at rest (the CoM VehicleHeightM above the site) - and the engine is cut at contact, unless, in the burn's last ThreeDofSixDofTimeS (5 s, on by default), 6-DOF has taken the burn first, for its rotational model of the final flare.
//
// BY TIME, NOT SPEED. The hand-over was below 100 m/s, which a heavy booster reaches a few seconds from the pad but a light one, falling at a terminal speed of ~50 m/s, reaches at ignition: a 20 t booster lit 2.3 km up was handed a 40 s burn (headless, 2026-10-08). 6-DOF has no engine-off coast and no drag model - only a constant bias - so over a burn that long it rides its thrust floor, tilts 25-30 degrees to shed what it cannot throttle away, wanders hundreds of metres, and loses the plan as drag falls off. Its last seconds are what it is for, whatever the speed. There is no hover hand-off: a craft that cannot throttle below its weight could not be handed one, and one that stops short is cut and dropped instead (see the burn's stop-short check).
// Touchdown guidance solves on standby from ignition. Entry keeps flying its tracked plan until the standby has a fresh plan that matches the craft's attitude and motion.
//
public static partial class GuidanceWindow
{
    public enum ThreeDofPhase { Idle, Glide, Burn, Done }

    /// <summary>True while the 3-DOF flies the craft.</summary>
    internal static bool ThreeDofLive => _s.ThreeDofPhase is ThreeDofPhase.Glide or ThreeDofPhase.Burn;

    /// <summary>Re-solve cadence, sim seconds.</summary>
    private const double ThreeDofUpdateS = 0.1;

    /// <summary>How far ahead the plan's own rate of turn is read, s.</summary>
    private const double ThreeDofRateProbeS = 0.05;

    /// <summary>Fastest the command may turn, as a guard over the plan's own rate limit, rad/s.</summary>
    private const double ThreeDofSlewMax = 30.0 * Math.PI / 180.0;

    /// <summary>Consecutive refusals before restarting the glide or requesting a touchdown handover during the burn.</summary>
    private const int ThreeDofGlideRefusalLimit = 20;
    private const int ThreeDofBurnRefusalLimit = 10;

    /// <summary>Wait between cold-solve attempts after one fails, s.</summary>
    private const double ThreeDofColdRetryS = 2.0;

    internal static bool CanRestartEntryGlide(double ignitionInS, double lockLeadS)
        => ignitionInS > lockLeadS + 2.0 * ThreeDofColdRetryS;

    private static bool CanRestartEntryGlide(Guidance3Dof guidance, double now)
        => guidance?.Published is not { } plan || CanRestartEntryGlide(plan.IgnitionTime - now, guidance.IgnitionLockLeadS);

    private static bool ThreeDofBusy => _s.ThreeDofWorker != null && _s.ThreeDofWorker.Busy;

    /// <summary>
    /// Take the craft for the 3-DOF landing. From the boostback glide, or from the page. The glide state carries over: until the first plan this mode flies the same PID glide.
    /// </summary>
    private static bool Engage3Dof(Vehicle vehicle, double now)
    {
        _s.SixDofCoastPending = false;
        _s.ThreeDofGlideHandoff = null;
        _s.SixDofCoastScaleState = null;
        if (!vehicle.IsControllable)
            return RefuseEntry(vehicle, vehicle.NoControlTooltip.ToString());
        IParentBody parent = vehicle.Orbit?.Parent;
        if (parent?.GetPhysicalAtmosphere() == null)
            return RefuseEntry(vehicle, "NO ATMOSPHERE.");
        if (EntryWouldInterruptPoweredMode(vehicle))
            return RefuseEntry(vehicle, "ANOTHER MODE FLIES A LIT ENGINE.");
        var thrust = KsaEnginePerf.GetThrottleControlStatus(vehicle);
        if (thrust == KsaEnginePerf.ThrustStatus.UnsupportedEngine)
            return RefuseEntry(vehicle, "THROTTLE CONTROL REQUIRES ACTIVE LIQUID ENGINES.");
        if (thrust != KsaEnginePerf.ThrustStatus.Available)
            return RefuseEntry(vehicle, "NO ACTIVE, SUPPLIED ENGINE FOR LANDING.");

        EnsureBoostbackAero(vehicle, parent);
        KsaAeroSweep.Result aero = _s.Aero;
        if (aero?.Table == null || aero.Atmosphere == null
            || _s.AeroStale(LiveBoxExtents(vehicle), KsaAeroSweep.LiveMeshHash(vehicle), parent))
            return RefuseEntry(vehicle, "AERO MODEL UNAVAILABLE.");
        double3 site = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        KsaFrameBridge.SiteFrame engageFrame = KsaFrameBridge.BuildSiteFrame(site);
        KsaPointMassModel model = Ksa3DofSetup.BuildModel(vehicle, parent, engageFrame, aero, out string refusal, ReturnRollRefLocal(vehicle, parent, engageFrame));
        if (model == null)
            return RefuseEntry(vehicle, refusal);

        if (IsStockDeorbitPhase(_s.LandingPhase))
            StopGuidance(vehicle, StopTarget.Landing);
        ClaimVehicle(GuidanceMode.ThreeDof, vehicle);
        _s.Engage = true;
        Reset3Dof();
        _s.ThreeDofNominal = model;
        // The per-step glide log, under the same switch as touchdown guidance's (SixDofLogging).
        if (_s.EntryLogging && EntryLog.Start(_s, vehicle.Id, (vehicle.Orbit?.Parent as Astronomical)?.Id ?? ""))
            GuidanceLog.Info(vehicle, "Entry guidance log: " + EntryLog.Path);
        _s.ThreeDofPhase = ThreeDofPhase.Glide;
        _s.ThreeDofLastStep = now;
        // A fresh flown path for the entry page's plot, which touchdown guidance's plot carries on after the hand-over.
        ResetGfoldTrace();
        _s.DiagEntry("Entry guidance engaged; PID glide while the cold solve runs.");
        GuidanceLog.Info(vehicle, $"Entry guidance engaged, {_s.GlideTgo:F0} s to impact.");
        string noLift = AeroLiftWarning();
        if (noLift.Length > 0)
        {
            GuidanceLog.Info(vehicle, "Entry guidance: " + noLift);
            _s.ThreeDofStatus = "NO LIFT. BALLISTIC FALL. BURN LANDS.";
        }
        return true;
    }

    private static bool EntryWouldInterruptPoweredMode(Vehicle vehicle)
    {
        if (vehicle.FlightComputer.BurnMode == FlightComputerBurnMode.Auto && vehicle.ComputeLiveThrust() > 0.0)
            return true;
        bool poweredGuidance = _s.Active || _s.Running || LandingMachineLive || _s.LandingPhase == LandingPhase.Burn
            || BoostbackLive && _s.BoostbackEngineOn;
        return poweredGuidance && (ManualInputs(vehicle).EngineOn || vehicle.ComputeLiveThrust() > 0.0);
    }

    private static bool RefuseEntry(Vehicle vehicle, string reason)
    {
        string status = "ENTRY REFUSED: " + reason;
        if (_s.ThreeDofStatus != status)
            GuidanceLog.Info(vehicle, status);
        _s.WarnEntry(status);
        return false;
    }

    /// <summary>Stop the 3-DOF. In the air the engine is left as it is; a cut there drops the craft.</summary>
    private static void Disengage3Dof(string why)
    {
        bool burning = _s.ThreeDofPhase == ThreeDofPhase.Burn;
        // AFC's own attitude controller flew the glide: its RCS and surfaces stop with entry guidance.
        if (_s.OwnAttitudeFlying && _s.OwnAttitudeVehicle is { } flown)
            StopOwnAttitude(flown);
        StopSixDofStandby();
        // A coast nothing will fly any more: the next touchdown guidance plans without one unless entry guidance asks again.
        if (!_s.Active && !_s.EngagePending)
        {
            _s.SixDofCoastPending = false;
            _s.ThreeDofGlideHandoff = null;
        }
        Reset3Dof();
        _s.ThreeDofPhase = ThreeDofPhase.Done;
        if (burning)
            _s.ReleaseWithoutEngineCut = true;
        else
            _s.LandingCutPending = true;
        _s.ThreeDofStatus = why;
    }

    /// <summary>Cancel the solve and drop the entry model and command history.</summary>
    private static void Reset3Dof()
    {
        _s.CancelEntrySolve();
        _s.ThreeDofGuidance = null;
        _s.ThreeDofNominal = null;
        _s.ThreeDofEstimator = null;
        _s.ThreeDofAccelTime = _s.ThreeDofLogTime = double.NaN;
        _s.ThreeDofLastUpdate = _s.ThreeDofColdTime = double.NegativeInfinity;
        _s.ThreeDofPlan = null;
        _s.ThreeDofCommandLocal = default;
        _s.ThreeDofPlanCommandCci = default;
        _s.ThreeDofTrimCci = default;
        EntryLog.Stop(_s);
        _s.ThreeDofSolveText = "";
        _s.ThreeDofRefusing = false;
        _s.ThreeDofEngineOn = false;
        _s.ThreeDofThrottle = _s.ThreeDofPlanThrottle = 0.0;
        _s.ThreeDofSixDofWaiting = false;
        _s.ThreeDofBurnSolveFault = false;
        _s.ThreeDofTouchdownHandoff = default;
        _s.ThreeDofHandoffRefusal = "";
        _s.SixDofStandbyPredicted = false;
        _s.ThreeDofHandoverDueAt = double.NaN;
    }

    /// <summary>One step of the 3-DOF landing, run for this vehicle from ApplyAutopilot whether or not it is the one on screen.</summary>
    private static void Step3Dof(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        double now = SimNow();
        // A pending engage is taken in ApplyAutopilot, ahead of the control claim.
        if (!ThreeDofLive)
        {
            // Whatever ended entry guidance, its log ends with it.
            EntryLog.Stop(_s);
            return;
        }

        double dt = Math.Clamp(now - _s.ThreeDofLastStep, 0.0, 1.0);
        _s.ThreeDofLastStep = now;

        if (HasTouchedDown(vehicle))
        {
            GuidanceLog.Info(vehicle, "Entry guidance: touchdown, engine cut.");
            StopSixDofStandby();
            Reset3Dof();
            _s.ThreeDofPhase = ThreeDofPhase.Done;
            _s.LandingCutPending = true;
            _s.ThreeDofStatus = "TOUCHDOWN. ENGINE CUT.";
            return;
        }

        bool burning = _s.ThreeDofPhase == ThreeDofPhase.Burn;
        if (_s.ThreeDofWorker?.TakeError(_s.ThreeDofGuidance) is { Length: > 0 } fault)
        {
            GuidanceLog.Info(vehicle, "Entry guidance solve faulted: " + fault);
            if (burning)
                _s.ThreeDofBurnSolveFault = true;
            else if (CanRestartEntryGlide(_s.ThreeDofGuidance, now))
                Drop3Dof(vehicle, "ENTRY SOLVE FAULT. PID GLIDE, COLD RETRY.", now);
        }

        EnsureBoostbackAero(vehicle, parent);
        KsaAeroSweep.Result aero = _s.Aero;
        // The game's moment limiter, measured through the glide so touchdown guidance's first plan has it (BodyAero6Dof.MomentLimiterPerPa).
        UpdateMomentLimiter(vehicle, now);
        double3 siteCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        KsaFrameBridge.SiteFrame frame = KsaFrameBridge.BuildSiteFrame(siteCci);
        double[] x14 = KsaFrameBridge.ToModelState(vehicle, frame);
        double[] x = [x14[0], x14[1], x14[2], x14[3], x14[4], x14[5], vehicle.TotalMass];
        RecordGfoldTrace(Math.Sqrt(x14[0] * x14[0] + x14[1] * x14[1]), x14[2]);
        double3 attLocal = frame.VecToLocal(KsaFrameBridge.ThrustAxisCci(vehicle)).NormalizeOrZero();
        double[] att = [attLocal.X, attLocal.Y, attLocal.Z];

        if (aero?.Table != null && aero.Atmosphere != null && _s.ThreeDofNominal == null)
        {
            _s.ThreeDofNominal = Ksa3DofSetup.BuildModel(vehicle, parent, frame, aero, out string error, ReturnRollRefLocal(vehicle, parent, frame));
            if (_s.ThreeDofNominal == null)
                _s.DiagEntry(error);
        }
        _s.ThreeDofEstimator ??= new AeroScaleEstimator();
        if (_s.ThreeDofNominal != null)
        {
            Measure3Dof(x, att, burning ? _s.ThreeDofThrottle : 0.0, now);
            _s.ThreeDofBiasAccel = _s.ThreeDofEstimator.BiasAcceleration(_s.ThreeDofNominal, x);
        }

        if (burning)
        {
            // Entry guidance lit the engine itself: a standby planning a coast carries on with the coast pinned at zero, and the burn is handed over as before.
            if (_s.SixDofCoastPending)
            {
                _s.SixDofCoastPending = false;
                _s.ThreeDofGlideHandoff = null;
                if (_s.SixDofStandby && _s.Guidance != null)
                    _s.Guidance.EngineLit = true;
            }
            if (_s.SixDofStandbyPredicted)
            {
                // The engine has lit: the standby carries on from the craft as it is. The bias estimator starts afresh, and a standby that could not be set up on the forecast may be tried again at once.
                _s.SixDofStandbyPredicted = false;
                _s.PrevV = null;
                if (!_s.SixDofStandby)
                    _s.SixDofStandbyRetryAt = now;
            }
            StepEntryTouchdownHandoff(vehicle, parent, siteCci, x14, now);
        }
        else
            StepEntryTouchdownPrepare(vehicle, parent, siteCci, x, x14, now);

        // Fly the plan if there is one; otherwise the PID glide until there is.
        if (!Plan3DofStep(vehicle, parent, frame, x, att, now, dt, out double3 wantLocal, out double3 rateLocal, out bool smooth))
        {
            if (!ThreeDofLive)
                return;   // a handover claimed the craft on this step
            if (_s.ThreeDofPhase == ThreeDofPhase.Burn)
            {
                // Lit with no plan for this step: hold the command and the throttle rather than steer on a law the burn does not use.
                CommandThreeDof(frame, frame.VecToLocal(_s.ThreeDofPlanCommandCci.Length() > 0.5 ? _s.ThreeDofPlanCommandCci : _s.CommandDir), default, dt, smooth: false);
                TrimThreeDof(vehicle, orbit, parent, dt, integrate: false);
                EntryLogRow(vehicle, parent, frame, x, att, now, onPlan: false);
                return;
            }
            // The PID glide has its own trim (GlideDirection), so entry guidance's fades out under it.
            double3 glide = GlideDirection(vehicle, orbit, parent, now, dt);
            double3 glideRate = BoostbackTargetRate(glide, dt);
            CommandThreeDof(frame, frame.VecToLocal(glide), frame.VecToLocal(glideRate), dt, smooth: false);
            TrimThreeDof(vehicle, orbit, parent, dt, integrate: false);
            _s.ThreeDofEngineOn = false;
            _s.ThreeDofThrottle = 0.0;
            EntryLogRow(vehicle, parent, frame, x, att, now, onPlan: false);
            return;
        }
        CommandThreeDof(frame, wantLocal, rateLocal, dt, smooth);
        // Trimmed only in the glide on a plan, and not while the command turns to the burn's opening attitude after the lock, where the error is the turn's own and winding it in would carry it into the burn.
        TrimThreeDof(vehicle, orbit, parent, dt, integrate: _s.ThreeDofPhase == ThreeDofPhase.Glide);
        Log3Dof(vehicle, x, att, now);
        EntryLogRow(vehicle, parent, frame, x, att, now, onPlan: true);
    }

    /// <summary>The held roll's reference in the site frame, for the model (KsaPointMassModel.RollRef), or zero when the roll is not held.</summary>
    private static double3 ReturnRollRefLocal(Vehicle vehicle, IParentBody parent, KsaFrameBridge.SiteFrame frame)
        => _s.BoosterRollHold ? frame.VecToLocal(ReturnRollRefCci(vehicle, parent)).NormalizeOrZero() : default;

    /// <summary>Largest trim, degrees: a correction for a steady offset, not a second steering law.</summary>
    private const double ThreeDofTrimMaxDeg = 5.0;

    /// <summary>Time constant the trim fades out with where it is not wound in, s: through the turn to the burn's opening attitude after the lock, in the burn, where the tracker and the gimbals hold the attitude, and under the PID glide.</summary>
    private const double ThreeDofTrimFadeS = 1.0;

    /// <summary>Below this airspeed the airflow direction the trim is measured across means little, m/s.</summary>
    private const double ThreeDofTrimMinSpeedMs = 30.0;

    /// <summary>
    /// THE ATTITUDE TRIM, every step: the tilt of the thrust axis off the airflow as asked (the command before the trim) against the tilt flown, the difference wound into a small rotation that is added to the command the flight computer points. The boostback glide's trim, in vector form (GlideTrackGain, GlideTrimMaxErrorDeg).
    ///  It stays out of what the plan sees. ThreeDofCommandLocal, which each re-solve anchors its opening attitude on, and ThreeDofPlanCommandCci, which the low-pass steps from, are the command before the trim, so a plan never starts from the trim and the trim never compounds through the filter. Only CommandDir carries it.
    /// </summary>
    private static void TrimThreeDof(Vehicle vehicle, Orbit orbit, IParentBody parent, double dt, bool integrate)
    {
        double3 asked = _s.ThreeDofPlanCommandCci;
        if (asked.Length() < 0.5)
            return;
        double3 trim = _s.ThreeDofTrimCci;
        double3 r = orbit.StateVectors.PositionCci;
        double3 vAir = orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), r);
        double speed = vAir.Length();
        // Off under AFC's own attitude controller (ThreeDofOwnAttitude): it holds the command against the air itself, where the trim wound out the stock flight computer's standing error.
        if (!_s.ThreeDofTrim || OwnsEntryAttitude(true))
            trim = default;
        else if (integrate && speed > ThreeDofTrimMinSpeedMs)
        {
            double3 retro = -vAir / speed;
            double3 flown = KsaFrameBridge.ThrustAxisCci(vehicle).NormalizeOrZero();
            double3 err = (asked - double3.Dot(asked, retro) * retro) - (flown - double3.Dot(flown, retro) * retro);
            // Only a steady offset: an error larger than this is a manoeuvre still in progress.
            if (err.Length() <= GlideTrimMaxErrorDeg * Math.PI / 180.0)
                trim += GlideTrackGain * dt * err;
            trim -= double3.Dot(trim, retro) * retro;
            double max = ThreeDofTrimMaxDeg * Math.PI / 180.0;
            if (trim.Length() > max)
                trim *= max / trim.Length();
        }
        else
            trim *= Math.Exp(-Math.Max(dt, 0.0) / ThreeDofTrimFadeS);
        _s.ThreeDofTrimCci = trim;
        double3 sent = (asked + trim).NormalizeOrZero();
        if (sent.Length() > 0.5)
            _s.CommandDir = sent;
    }

    /// <summary>Sim seconds between 3-DOF lines in the game log.</summary>
    private const double ThreeDofLogIntervalS = 2.0;

    /// <summary>
    /// What the 3-DOF is flying, every ThreeDofLogIntervalS, so a landing that misbehaves can be read back: where it is, what the plan says, how far the vehicle's attitude is from the command, and how the solver and the model corrections are doing.
    /// </summary>
    private static void Log3Dof(Vehicle vehicle, double[] x, double[] att, double now)
    {
        if (double.IsFinite(_s.ThreeDofLogTime) && now - _s.ThreeDofLogTime < ThreeDofLogIntervalS)
            return;
        _s.ThreeDofLogTime = now;
        Guidance3Dof g = _s.ThreeDofGuidance;
        Plan3Dof plan = _s.ThreeDofPlan;
        double speed = Math.Sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
        double3 cmd = _s.ThreeDofCommandLocal;
        double off = Math.Acos(Math.Clamp(cmd.X * att[0] + cmd.Y * att[1] + cmd.Z * att[2], -1.0, 1.0)) * 180.0 / Math.PI;
        AeroScaleEstimator est = _s.ThreeDofEstimator;
        GuidanceLog.Info(vehicle, string.Format(System.Globalization.CultureInfo.InvariantCulture,
            "Entry guidance {0}: alt {1:F2} km, {2:F0} m/s, {3:F0} m from the site; {4}; flown {5:F1} deg off the command (trim {11:F1} deg); "
            + "{6}; drag x{7:F2} cross-flow bias {8:F2} m/s^2 thrust x{9:F2}{10}",
            ThreeDofPhaseName(_s.ThreeDofPhase), x[2] / 1000.0, speed, Math.Sqrt(x[0] * x[0] + x[1] * x[1]),
            plan == null ? "no plan"
                : _s.ThreeDofPhase == ThreeDofPhase.Burn
                    ? FormattableString.Invariant($"burn {plan.EndTime - now:F1} s left, throttle {_s.ThreeDofThrottle:P0} (plan {_s.ThreeDofPlanThrottle:P0}), {TrackingErrorText(plan, x, now)}")
                : FormattableString.Invariant($"ignition in {plan.IgnitionTime - now:F1} s, burn {plan.SigmaBurn:F1} s"),
            off,
            g == null ? "NO SOLVER" : ThreeDofBusy ? "SOLVING" : FormattableString.Invariant($"LAST SOLVE {g.LastSolveMs:F0} ms, {g.ConsecutiveRefusals} REFUSED{RefusalReason(g)}"),
            est?.DragScale ?? 1.0, est != null && _s.ThreeDofNominal != null ? est.BiasAcceleration(_s.ThreeDofNominal, x) : 0.0, est?.ThrustScale ?? 1.0,
            _s.ThreeDofPhase == ThreeDofPhase.Burn ? "; TOUCHDOWN STANDBY " + SixDofStandbyText(now) : "",
            _s.ThreeDofTrimCci.Length() * 180.0 / Math.PI));
    }

    /// <summary>
    /// One row of the per-step glide log (EntryLog), at its own cadence. Angles are taken against the airflow in the site frame: the thrust axis flies at alpha off retrograde, engine first. The error between the attitude flown and the one asked for (before the trim) is split three ways - along the asked angle of attack, across it, and along the command's own motion - because a craft that is unstable engine first errs past the command, and an attitude loop that lags errs behind its motion.
    /// </summary>
    private static void EntryLogRow(Vehicle vehicle, IParentBody parent, KsaFrameBridge.SiteFrame frame, double[] x, double[] att, double now, bool onPlan)
    {
        if (!EntryLog.Due(_s, now))
            return;
        try
        {
            const double Deg = 180.0 / Math.PI;
            var v = new double3(x[3], x[4], x[5]);
            double speed = v.Length();
            double3 retro = speed > 1e-6 ? -v / speed : default;
            double3 asked = _s.ThreeDofCommandLocal.NormalizeOrZero();
            double3 sent = frame.VecToLocal(_s.CommandDir).NormalizeOrZero();
            var flown = new double3(att[0], att[1], att[2]);

            double3 askedAcross = asked - double3.Dot(asked, retro) * retro;
            double3 flownAcross = flown - double3.Dot(flown, retro) * retro;
            // The direction of the asked angle of attack; at alpha near zero, the flown one's.
            double3 alphaDir = askedAcross.Length() > 1e-3 ? askedAcross.NormalizeOrZero() : flownAcross.NormalizeOrZero();
            double3 sideDir = double3.Cross(retro, alphaDir).NormalizeOrZero();
            double3 errAcross = flownAcross - askedAcross;

            // The command's motion, from its rate: d(asked)/dt = w x asked, in the site frame.
            double3 rateLocal = frame.VecToLocal(_s.CommandRate);
            double3 motion = double3.Cross(rateLocal, asked);
            double3 motionDir = motion.Length() > 1e-9 ? motion.NormalizeOrZero() : default;

            double3 trimLocal = frame.VecToLocal(_s.ThreeDofTrimCci);
            double rho = vehicle.PhysicsEnvironment.AtmosphericDensity;
            double q = 0.5 * rho * speed * speed;

            KsaAeroSweep.Result aero = _s.Aero;
            double mach = aero?.Atmosphere?.Mach(speed) ?? double.NaN;
            double alphaFlown = AngleBetween(flown, retro);
            double cm = aero?.Moment != null && double.IsFinite(mach) ? aero.MomentAt(Math.Clamp(mach, aero.Moment.MachMin, aero.Moment.MachMax), alphaFlown) : double.NaN;
            double moment = aero != null ? q * aero.ReferenceArea * aero.ReferenceLength * cm : double.NaN;
            // Transverse inertia: the mean of the two largest principal-axis values of the body tensor's diagonal; the smallest is the long axis.
            var inertia = vehicle.TotalMassPropsBody.Inertia;
            double ixx = inertia.XX, iyy = inertia.YY, izz = inertia.ZZ;
            double inertiaT = 0.5 * (ixx + iyy + izz - Math.Min(ixx, Math.Min(iyy, izz)));
            double aeroAccel = inertiaT > 0.0 ? moment / inertiaT : double.NaN;

            FlightComputer fc = vehicle.FlightComputer;
            float3 err = fc.ErrorAngles;

            // THE FORCES, measured against modelled. The measured non-gravitational acceleration (Measure3Dof) split along the airflow (+ drag), along the lift the model predicts for the thrust axis flown, and across both; beside it the model's drag and lift for the thrust axis the plan points, and for the body axis the aero tables are measured about. A lift residual that the body axis's prediction closes says the plan flies one axis while the air sees the other.
            double3 bodyNose = frame.VecToLocal(KsaFrameBridge.BodyNoseCci(vehicle)).NormalizeOrZero();
            double axisOffset = AngleBetween(flown, bodyNose);
            double3 measured = _s.ThreeDofMeasuredAccel;
            double3 predThrust = default, predBody = default;
            if (_s.ThreeDofNominal != null && speed > 1.0)
            {
                double[] pt = AdvancedFlightComputer.Guidance.Scvx.ThreeDof.AeroScaleEstimator.PredictedAero(_s.ThreeDofNominal, x, att);
                double[] pb = AdvancedFlightComputer.Guidance.Scvx.ThreeDof.AeroScaleEstimator.PredictedAero(_s.ThreeDofNominal, x, [bodyNose.X, bodyNose.Y, bodyNose.Z]);
                predThrust = new double3(pt[0], pt[1], pt[2]);
                predBody = new double3(pb[0], pb[1], pb[2]);
            }
            // retro is -v_hat, so drag acts along it. The lift direction is the thrust axis's predicted lift, or the engine end's tilt where that is nothing.
            double3 liftPred = predThrust - double3.Dot(predThrust, retro) * retro;
            double3 liftDir = liftPred.Length() > 1e-6 ? liftPred.NormalizeOrZero() : (-flownAcross).NormalizeOrZero();
            double3 liftSide = double3.Cross(retro, liftDir).NormalizeOrZero();
            // The game's own aerodynamic force on the craft as it is, over its mass, site frame.
            bool liveOk = KsaAeroSweep.TryLiveAero(vehicle, out double3 liveForceBody, out double3 liveTorque);
            double3 liveAccel = liveOk && vehicle.TotalMass > 0.0
                ? frame.VecToLocal(liveForceBody.Transform(KsaFrameBridge.BodyToCci(vehicle))) / vehicle.TotalMass
                : default;
            // Roll: the body's +y across the body axis, against the lift direction across it. Its zero is arbitrary; it says whether the craft turns about its axis through the glide.
            double3 bodyY = frame.VecToLocal(double3.UnitY.Transform(KsaFrameBridge.BodyToCci(vehicle)));
            double3 yAcross = (bodyY - double3.Dot(bodyY, bodyNose) * bodyNose).NormalizeOrZero();
            double3 liftAcross = (liftDir - double3.Dot(liftDir, bodyNose) * bodyNose).NormalizeOrZero();
            double rollDeg = yAcross.Length() > 0.5 && liftAcross.Length() > 0.5
                ? Math.Atan2(double3.Dot(double3.Cross(liftAcross, yAcross), bodyNose), double3.Dot(liftAcross, yAcross)) * Deg
                : double.NaN;
            // The held roll against the one flown: the body's +y now against the held wing direction's part square to the axis, about the body axis. Near zero means the body-axis table is read in the axes the craft really has.
            double rollHoldErr = double.NaN;
            if (_s.BoosterRollHold && _s.ReturnRollRefCcf.Length() > 0.5)
            {
                double3 d = frame.VecToLocal(ReturnRollRefCci(vehicle, parent)).NormalizeOrZero();
                double3 wantY = (d - double3.Dot(d, bodyNose) * bodyNose).NormalizeOrZero();
                if (wantY.Length() > 0.5 && yAcross.Length() > 0.5)
                    rollHoldErr = Math.Atan2(double3.Dot(double3.Cross(wantY, yAcross), bodyNose), double3.Dot(wantY, yAcross));
            }
            // THE ROLL BALANCE about the thrust axis (EntryLog.EntryRow.RollNetNm): measured, air, RCS fired, and what is left.
            double rollRate = double.NaN, rollAccel = double.NaN, rollIzz = double.NaN, rollAir = double.NaN, rollRcs = double.NaN, rcsPos = double.NaN, rcsNeg = double.NaN;
            // All three axes: [0] pitch (model x), [1] yaw (model y), [2] roll (model z).
            var axNet = new double[] { double.NaN, double.NaN, double.NaN };
            var axInt = new double[] { double.NaN, double.NaN, double.NaN };
            var axAir = new double[] { double.NaN, double.NaN, double.NaN };
            var axAirDt = new double[] { double.NaN, double.NaN, double.NaN };
            var axRcs = new double[] { double.NaN, double.NaN, double.NaN };
            double physDt = double.NaN;
            {
                KsaFrameBridge.BodyAxes(vehicle, out double3 pitchAxis, out double3 yawAxis, out double3 rollAxis);
                double3[] axes = [pitchAxis, yawAxis, rollAxis];
                Ksa6DofSetup.Inertia(vehicle, out double monIxx, out double monIyy, out rollIzz);
                double[] monInertia = [monIxx, monIyy, rollIzz];
                double3 wBody = vehicle.KinematicStates.AngularVelocityPhys.Transform(vehicle.KinematicStates.Body2Phys.Inverse());
                rollRate = double3.Dot(wBody, rollAxis);
                double stepS = now - _s.RollMonitorPrevT;
                bool stepOk = stepS > 1e-4 && stepS < 1.0;
                if (stepOk && double.IsFinite(_s.RollMonitorPrevRate))
                    rollAccel = (rollRate - _s.RollMonitorPrevRate) / stepS;
                if (stepOk && _s.RollMonitorPrevW is { } prevW)
                {
                    double3 dw = (wBody - prevW) / stepS;
                    for (int a = 0; a < 3; a++)
                        axNet[a] = monInertia[a] * double3.Dot(dw, axes[a]);
                }
                _s.RollMonitorPrevRate = rollRate;
                _s.RollMonitorPrevW = wBody;
                _s.RollMonitorPrevT = now;
                // The integrator's own: the angular acceleration it applied on its last substep, and that substep.
                KinematicMeasurements meas = vehicle.KinematicMeasurements;
                physDt = meas.DeltaTime;
                for (int a = 0; a < 3; a++)
                    axInt[a] = monInertia[a] * double3.Dot(meas.AngularAccelerationBody, axes[a]);
                double3 liveTorqueDt = default;
                bool liveDtOk = physDt > 0.0 && KsaAeroSweep.TryLiveAero(vehicle, out _, out liveTorqueDt, physDt);
                for (int a = 0; a < 3; a++)
                {
                    if (liveOk)
                        axAir[a] = double3.Dot(liveTorque, axes[a]);
                    if (liveDtOk)
                        axAirDt[a] = double3.Dot(liveTorqueDt, axes[a]);
                    if (AdvancedFlightComputer.Features.RcsTorque.KsaRcsTorqueControl.FiredTorqueAlong(vehicle, axes[a], stepOk ? stepS : 0.02, out double fired, out double pos, out double neg))
                    {
                        axRcs[a] = fired;
                        if (a == 2)
                        {
                            rcsPos = pos;
                            rcsNeg = neg;
                        }
                    }
                }
                rollAir = axAir[2];
                rollRcs = axRcs[2];
            }
            // The moment, per body axis: the game's against the body-axis tables' for the air the craft meets now, about its centre of mass now.
            double3 predMoment = new double3(double.NaN, double.NaN, double.NaN);
            if (aero != null)
            {
                double3 vCci = frame.VecToCci(v);
                double3 airBody = vCci.Transform(doubleQuat.Inverse(KsaFrameBridge.BodyToCci(vehicle)));
                float3 comNow = vehicle.CenterOfMassAsmbF;
                if (!KsaAeroSweep.PredictBodyMoment(aero, airBody, rho, new double3(comNow.X, comNow.Y, comNow.Z), out predMoment))
                    predMoment = new double3(double.NaN, double.NaN, double.NaN);
            }
            float3 rcs = fc.RcsTorqueAuthority;
            float3 tvc = fc.TvcTorqueAuthorityMax;
            // The RCS as fired, under either controller, and AFC's own attitude controller's last step when it flew.
            double rcsFlow = double.NaN;
            int rcsFiring = 0;
            if (AdvancedFlightComputer.Features.RcsTorque.KsaRcsTorqueControl.FiredFlow(vehicle, physDt > 0.0 && physDt < 1.0 ? physDt : 0.02, out double firedFlow, out int firedCount))
            {
                rcsFlow = firedFlow;
                rcsFiring = firedCount;
            }
            bool ownAtt = _s.OwnAttitudeFlying;
            AttitudeController.Report ac = ownAtt ? AttitudeController.Last(vehicle) : default;
            double rcsTransverse = Math.Min(rcs.Y, rcs.Z);
            Guidance3Dof g = _s.ThreeDofGuidance;

            EntryLog.Row(_s, new EntryLog.EntryRow
            {
                T = now,
                Phase = ThreeDofPhaseName(_s.ThreeDofPhase),
                OnPlan = onPlan,
                Alt = x[2], Speed = speed, Mach = mach, Q = q,
                AlphaAsked = AngleBetween(asked, retro) * Deg,
                AlphaSent = AngleBetween(sent, retro) * Deg,
                AlphaFlown = alphaFlown * Deg,
                OffAsked = AngleBetween(flown, asked) * Deg,
                PastDeg = double3.Dot(errAcross, alphaDir) * Deg,
                SideDeg = double3.Dot(errAcross, sideDir) * Deg,
                LagDeg = motionDir.Length() > 0.5 ? double3.Dot(flown - asked, motionDir) * Deg : 0.0,
                CmdTurnDegS = rateLocal.Length() * Deg,
                BodyRateDegS = vehicle.KinematicStates.AngularVelocityPhys.Length() * Deg,
                TrimDeg = trimLocal.Length() * Deg,
                TrimPastDeg = double3.Dot(trimLocal, alphaDir) * Deg,
                FcErrX = err.X * Deg, FcErrY = err.Y * Deg, FcErrZ = err.Z * Deg,
                FcDeadband = fc.AngleDeadband * Deg,
                RcsAuthX = rcs.X, RcsAuthY = rcs.Y, RcsAuthZ = rcs.Z,
                TvcAuthY = tvc.Y, TvcAuthZ = tvc.Z,
                CmFlown = cm, AeroMomentNm = moment, InertiaT = inertiaT, AeroAccel = aeroAccel,
                AeroOverRcs = rcsTransverse > 0.0 ? Math.Abs(aeroAccel) / rcsTransverse : double.NaN,
                Throttle = _s.ThreeDofThrottle,
                DragScale = _s.ThreeDofEstimator?.DragScale ?? 1.0,
                BiasAccel = _s.ThreeDofEstimator != null && _s.ThreeDofNominal != null ? _s.ThreeDofEstimator.BiasAcceleration(_s.ThreeDofNominal, x) : double.NaN,
                LiveOk = liveOk,
                LiveDrag = double3.Dot(liveAccel, retro),
                LiveLift = double3.Dot(liveAccel, liftDir),
                LiveSide = double3.Dot(liveAccel, liftSide),
                LiveMomentNm = liveTorque.Length(),
                BodyAero = _s.ThreeDofNominal?.BodyAero != null,
                RollHoldErrDeg = rollHoldErr * Deg,
                RollRateDegS = rollRate * Deg,
                RollAccelDegS2 = rollAccel * Deg,
                RollIzz = rollIzz,
                RollNetNm = rollIzz * rollAccel,
                RollAirNm = rollAir,
                RollRcsNm = rollRcs,
                RollResidNm = rollIzz * rollAccel - rollAir - rollRcs,
                RcsRollPosNm = rcsPos,
                RcsRollNegNm = rcsNeg,
                PhysDtS = physDt,
                PitchNetNm = axNet[0], PitchIntNm = axInt[0], PitchAirNm = axAir[0], PitchAirDtNm = axAirDt[0], PitchRcsNm = axRcs[0],
                YawNetNm = axNet[1], YawIntNm = axInt[1], YawAirNm = axAir[1], YawAirDtNm = axAirDt[1], YawRcsNm = axRcs[1],
                RollIntNm = axInt[2], RollAirDtNm = axAirDt[2],
                LiveMx = liveOk ? liveTorque.X : double.NaN,
                LiveMy = liveOk ? liveTorque.Y : double.NaN,
                LiveMz = liveOk ? liveTorque.Z : double.NaN,
                PredMx = predMoment.X, PredMy = predMoment.Y, PredMz = predMoment.Z,
                SolveWaitMs = _s.ThreeDofWorker?.LastWaitMs ?? double.NaN,
                SolveRunMs = _s.ThreeDofWorker?.LastRunMs ?? double.NaN,
                RcsFlowKgS = rcsFlow,
                RcsFiring = rcsFiring,
                OwnAtt = ownAtt,
                AcErrDeg = ownAtt ? ac.ErrorDeg : double.NaN,
                AcRateErrDegS = ownAtt ? ac.RateErrorDegS : double.NaN,
                AcDeadband = ac.InDeadband,
                AcAirKnown = ac.AirKnown,
                AcDemand = ownAtt ? ac.DemandNm : new double3(double.NaN, double.NaN, double.NaN),
                AcAir = ownAtt && ac.AirKnown ? ac.AirNm : new double3(double.NaN, double.NaN, double.NaN),
                AcSurf = ownAtt ? ac.SurfacesNm : new double3(double.NaN, double.NaN, double.NaN),
                AcRcsAsk = ownAtt ? ac.RcsAskedNm : new double3(double.NaN, double.NaN, double.NaN),
                AcRcs = ownAtt ? ac.RcsNm : new double3(double.NaN, double.NaN, double.NaN),
                AcRcsFlowKgS = ownAtt ? ac.RcsFlowKgS : double.NaN,
                // Guidance results are the worker's while it solves; -1 then.
                Refusals = g == null ? 0 : ThreeDofBusy ? -1 : g.ConsecutiveRefusals,
                AxisOffsetDeg = axisOffset * Deg,
                AlphaBody = AngleBetween(bodyNose, retro) * Deg,
                Tnm = fc.ActiveControlFrame.IsTnm(),
                RollDeg = rollDeg,
                MeasDrag = double3.Dot(measured, retro),
                MeasLift = double3.Dot(measured, liftDir),
                MeasSide = double3.Dot(measured, liftSide),
                PredDrag = double3.Dot(predThrust, retro),
                PredLift = double3.Dot(predThrust, liftDir),
                PredLiftBody = double3.Dot(predBody, liftDir),
                PredSideBody = double3.Dot(predBody, liftSide),
                Solver = g == null ? "NO SOLVER" : ThreeDofBusy ? "SOLVING" : _s.ThreeDofSolveText.Trim(),
            });
        }
        catch (Exception e)
        {
            // A broken log must never break a flight.
            GuidanceLog.Debug(vehicle, "entry log row failed: " + e.Message);
        }
    }

    /// <summary>
    /// The plan's part of the step: the cold solve, the re-solve cadence, the fallbacks and handovers, and the command from the plan. False when there is no plan to fly this step.
    /// </summary>
    private static bool Plan3DofStep(Vehicle vehicle, IParentBody parent, KsaFrameBridge.SiteFrame frame, double[] x, double[] att,
                                     double now, double dt, out double3 wantLocal, out double3 rateLocal, out bool smooth)
    {
        wantLocal = rateLocal = default;
        smooth = false;
        bool burning = _s.ThreeDofPhase == ThreeDofPhase.Burn;
        if (_s.ThreeDofNominal == null)
        {
            if (_s.ThreeDofStatus.Length == 0)
                _s.DiagEntry("Waiting for the aero surrogate.");
            return false;
        }

        Guidance3Dof g = _s.ThreeDofGuidance;
        if (g == null && !burning)
        {
            if (!burning && now - _s.ThreeDofColdTime >= ThreeDofColdRetryS)
                StartThreeDofCold(vehicle, x, now);
            return false;
        }

        // The command a new plan should open on: the one last sent, or the vehicle's own attitude before the first.
        double3 anchor = _s.ThreeDofCommandLocal.Length() > 0.5 ? _s.ThreeDofCommandLocal : new double3(att[0], att[1], att[2]);
        // And the turn it is making, w x b, which the turn-acceleration limit could continue from. NOT PASSED for now: offline (--3dof-mpc --turn-accel 3), starting each plan from the command's turn rate had 38-100 re-solves a case refused against 4-34 without it, the command's rate carrying the burn tracker's corrections. The locked burn still starts from the turn its ignition prediction ends on.
        double3 anchorTurn = _s.ThreeDofCommandLocal.Length() > 0.5 ? double3.Cross(frame.VecToLocal(_s.CommandRate), anchor) : default;
        const bool turnKnown = false;

        if (g != null && !ThreeDofBusy)
        {
            _s.ThreeDofSolveText = $"{g.LastSolveMs,8:F0} ms, {g.LastIterations} IT, {g.ConsecutiveRefusals} REFUSED";
            _s.ThreeDofRefusing = g.ConsecutiveRefusals > 0;
            // Results are read only while the worker is idle: the guidance is the worker's while it solves.
            // Allow time for one retry after a failed cold solve before the ignition lock. Keep the last accepted plan inside that window.
            if (!burning && (g.Phase == Phase3Dof.Idle
                || (g.Phase == Phase3Dof.Glide && g.ConsecutiveRefusals >= ThreeDofGlideRefusalLimit
                    && CanRestartEntryGlide(g, now))))
            {
                Drop3Dof(vehicle, g.Phase == Phase3Dof.Idle
                    ? "ENTRY " + g.Error + ". PID GLIDE, COLD RETRY."
                    : $"ENTRY REFUSED {g.ConsecutiveRefusals} RE-SOLVES{RefusalReason(g)}. PID GLIDE, COLD RETRY.", now);
                return false;
            }
            if (_s.ThreeDofTouchdownHandoff.Status == null
                && g.Phase is not (Phase3Dof.Idle or Phase3Dof.Converging)
                && now - _s.ThreeDofLastUpdate >= ThreeDofUpdateS)
            {
                KsaPointMassModel model = _s.ThreeDofEstimator.Apply(_s.ThreeDofNominal);
                double[] x0 = x, a0 = [anchor.X, anchor.Y, anchor.Z];
                double[] r0 = turnKnown ? [anchorTurn.X, anchorTurn.Y, anchorTurn.Z] : null;
                if (_s.ThreeDofWorker.TryRun(g, _ =>
                {
                    g.Model = model;
                    g.Update(x0, a0, now, r0);
                }))
                    _s.ThreeDofLastUpdate = now;
            }
        }

        Plan3Dof plan = g?.Published ?? (burning ? _s.ThreeDofPlan : null);
        if (plan == null)
        {
            _s.DiagEntry("PID glide while the entry guidance cold solve runs.");
            return false;
        }
        if (!ReferenceEquals(plan, _s.ThreeDofPlan))
        {
            if (_s.ThreeDofPlan == null)
                GuidanceLog.Info(vehicle, $"Entry guidance first plan at {x[2] / 1000:F1} km: ignition in {plan.IgnitionTime - now:F1} s, burn {plan.SigmaBurn:F1} s.");
            _s.ThreeDofPlan = plan;
            _s.ThreeDofBurnSolveFault = false;
            if (_s.ThreeDofTouchdownHandoff.Warning)
                _s.ThreeDofTouchdownHandoff = default;
        }

        double height = x[2];
        double speed = Math.Sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
        if (burning)
        {
            // STOPPED SHORT, OR CLIMBING. The plan ends at zero/zero, but a craft whose minimum thrust is above its weight climbs from wherever it actually stops, and nothing with the engine lit brings it back: the engine is cut and it drops the rest of the way. 6-DOF is no better placed, since it keeps the engine lit too.
            //  THE SAME RULE AS TOUCHDOWN GUIDANCE'S (NoPlanNearGround): cut on any climb at all, whatever the model says - a landing burn never plans one - and once it stops descending if the minimum throttle comes within a margin of the weight, judged on the thrust the engine has been MEASURED to give (the estimator's thrust scale). Entry kept the older rule, a model TWR above 1.0 and no climb test: on 2026-11-18 the model put that TWR at 1.00 while the engine gave 10 % more, and the booster climbed from 10 m to 65 m, rising 13.5 m/s, before it was cut.
            double minTwr = TerminalMinThrottleTwr(vehicle, vehicle.Orbit, parent.Mu) * (_s.ThreeDofEstimator?.ThrustScale ?? 1.0);
            bool cannotStop = x[5] >= 0.0 && minTwr > SixDofHoverMaxMinTwr;
            bool climbing = x[5] > SixDofClimbCutMs;
            if (cannotStop || climbing)
            {
                {
                    string why = FormattableString.Invariant($"ENTRY {(cannotStop ? "STOPPED SHORT" : "CLIMBING")} {height - _s.VehicleHeightM:F1} m ABOVE PAD, RISING {x[5]:F1} m/s, ")
                               + (cannotStop
                                   ? FormattableString.Invariant($"CANNOT THROTTLE BELOW WEIGHT (TWR {minTwr:F2} AT MIN). ENGINE CUT, DROP TO PAD.")
                                   : "ENGINE CUT, DROP TO PAD.");
                    GuidanceLog.Info(vehicle, why);
                    StopSixDofStandby();
                    Reset3Dof();
                    _s.ThreeDofPhase = ThreeDofPhase.Done;
                    _s.LandingCutPending = true;
                    _s.WarnEntry(why);
                    return false;
                }
            }
        }

        // The newest plan's attitude and throttle now, and its own rate of turn. Ignition locked and the engine still off, the plan the ignition was locked on, flown open loop to ignition (Guidance3Dof.LockPlan): its glide runs on into its burn's opening attitude, as the ignition prediction assumes, and nothing turns the craft that a plan did not ask for. The scripted turn towards the burn plan's opening attitude this replaced swung the 2026-11-18 booster across retrograde at 10 deg/s, three seconds before ignition, towards an attitude two seconds stale.
        Plan3Dof attitudePlan = plan.Phase == Phase3Dof.Locked && now < plan.IgnitionTime && g?.LockPlan is { } locked ? locked : plan;
        if (!SamplePlan(attitudePlan, now, out double3 b0, out double throttle) || !SamplePlan(attitudePlan, now + ThreeDofRateProbeS, out double3 b1, out _))
            return false;
        wantLocal = b0;
        rateLocal = double3.Cross(b0, b1) / ThreeDofRateProbeS;
        smooth = true;

        if (!burning && now >= plan.IgnitionTime)
        {
            GuidanceLog.Info(vehicle, $"Entry guidance ignition at {height:F0} m, {speed:F0} m/s.");
            _s.ThreeDofPhase = ThreeDofPhase.Burn;
            burning = true;
        }

        if (burning)
        {
            // The plan flown by the tracker: its thrust plus feedback on its state, the direction kept within a few degrees of its axis. See BurnTracker3Dof.
            var tracker = new BurnTracker3Dof
            {
                Kp = _s.ThreeDofTrackKp,
                Kd = _s.ThreeDofTrackKd,
                MaxTiltRad = Math.Clamp(_s.ThreeDofTrackTiltDeg, 0.0, 30.0) * Math.PI / 180.0,
            };
            double thrust = tracker.Command(plan, now, x, out double[] axis);
            wantLocal = new double3(axis[0], axis[1], axis[2]);
            _s.ThreeDofPlanThrottle = throttle;

            // The tracker's newtons by the game's real thrust curve. That curve is the one the nominal model was measured from, and the thrust scale is what the vehicle has been measured to deliver against it, so the curve is asked for the newtons over the scale: asked for the newtons themselves, it would take the deficit off a second time.
            KsaPointMassModel model = plan.Model ?? _s.ThreeDofNominal;
            double altitude = model.Altitude(x);
            double pa = KsaEnginePerf.AmbientPressureAt(parent, altitude);
            double scale = _s.ThreeDofEstimator?.ThrustScale ?? 1.0;
            double inverted = KsaEnginePerf.ThrottleForThrust(vehicle, thrust / scale, pa);
            double target = Math.Clamp(inverted >= 0.0 ? inverted : model.ThrottleFor(thrust, altitude), 0.0, 1.0);
            // Low-passed as the attitude is, except on the step the engine lights, which opens on the plan's own throttle rather than ramping up from zero.
            _s.ThreeDofThrottle = _s.ThreeDofEngineOn
                ? _s.ThreeDofThrottle + SmoothGain(dt) * (target - _s.ThreeDofThrottle)
                : target;
            _s.ThreeDofEngineOn = true;
        }
        else
        {
            _s.ThreeDofThrottle = _s.ThreeDofPlanThrottle = 0.0;
            _s.ThreeDofEngineOn = false;
        }

        // Qualitative only; the figures are the page's own rows, so this line keeps its length frame to frame.
        _s.DiagEntry(burning ? "Burning on the plan." : "Gliding on the plan.");
        return true;
    }

    /// <summary>How far the vehicle is off the plan's state now, for the log: what the burn tracker is correcting.</summary>
    private static string TrackingErrorText(Plan3Dof plan, double[] x, double now)
    {
        Span<double> xs = stackalloc double[PointMass3Dof.NX];
        Span<double> us = stackalloc double[PointMass3Dof.NU];
        plan.Sample(now, xs, us);
        double dr = new double3(xs[0] - x[0], xs[1] - x[1], xs[2] - x[2]).Length();
        double dv = new double3(xs[3] - x[3], xs[4] - x[4], xs[5] - x[5]).Length();
        return FormattableString.Invariant($"{dr:F1} m, {dv:F1} m/s OFF PLAN");
    }

    /// <summary>Why the last re-solve was refused, as " (...)" for a log line, or nothing when none was. Read only while the worker is idle, as every other guidance result is.</summary>
    private static string RefusalReason(Guidance3Dof g)
        => g.ConsecutiveRefusals > 0 && g.Error.Length > 0 ? $" ({g.Error})" : "";

    /// <summary>The landing burn's thrust acceleration limit, m/s^2, or 0 for none; shared by 3-DOF and 6-DOF.</summary>
    private static double LandingAccelLimitMs2 => _s.LandingAccelLimitG > 0.0 ? _s.LandingAccelLimitG * StandardGravity : 0.0;

    /// <summary>A plan's body axis and throttle at a time.</summary>
    private static bool SamplePlan(Plan3Dof plan, double t, out double3 b, out double throttle)
    {
        Span<double> xs = stackalloc double[PointMass3Dof.NX];
        Span<double> us = stackalloc double[PointMass3Dof.NU];
        plan.Sample(t, xs, us);
        b = new double3(us[0], us[1], us[2]).NormalizeOrZero();
        throttle = us[3];
        return b.Length() > 0.5;
    }

    /// <summary>The first-order low-pass's step gain over dt, for <see cref="VehicleAutopilotState.ThreeDofSmoothTau"/>.</summary>
    private static double SmoothGain(double dt)
        => _s.ThreeDofSmoothTau <= 1e-3 ? 1.0 : 1.0 - Math.Exp(-Math.Max(dt, 0.0) / _s.ThreeDofSmoothTau);

    /// <summary>
    /// Send a site-frame attitude and turn rate to the flight computer: low-passed towards the plan when <paramref name="smooth"/>, as G-FOLD's command is, and slewed no faster than the guard rate either way.
    /// </summary>
    private static void CommandThreeDof(KsaFrameBridge.SiteFrame frame, double3 wantLocal, double3 rateLocal, double dt, bool smooth)
    {
        double3 want = frame.VecToCci(wantLocal).NormalizeOrZero();
        if (want.Length() < 0.5)
        {
            _s.CommandRate = default;
            return;
        }
        // Stepped from the command before the trim (TrimThreeDof), or, on the first step, from whatever was flying the craft.
        double3 prev = _s.ThreeDofPlanCommandCci.Length() > 0.5 ? _s.ThreeDofPlanCommandCci : _s.CommandDir;
        if (smooth && prev.Length() > 0.5)
            want = (prev + SmoothGain(dt) * (want - prev)).NormalizeOrZero();
        double3 rate = frame.VecToCci(rateLocal);
        double maxRad = ThreeDofSlewMax * dt;
        bool clamped = false;
        double3 slewed = prev.Length() > 0.5 ? SlewToward(prev, want, maxRad, out clamped) : want;
        if (clamped && dt > 1e-9)
        {
            double3 axis = double3.Cross(prev, slewed);
            rate = axis.Length() > 1e-12 ? double3.Normalize(axis) * (maxRad / dt) : default;
        }
        _s.ThreeDofPlanCommandCci = slewed;
        _s.CommandDir = slewed;
        _s.CommandRate = rate;
        _s.HasCommand = _s.CommandDir.Length() > 0.5;
        _s.ThreeDofCommandLocal = frame.VecToLocal(slewed);
    }

    private static void StartThreeDofCold(Vehicle vehicle, double[] x, double now)
    {
        // Anchored tight on the last command, so consecutive plans start where the command is; see the file summary.
        var settings = new Guidance3Dof.Settings
        {
            AnchorSeconds = 0.3, AnchorFloor = 1.0 * Math.PI / 180.0,
            GlideThrottleCeiling = Math.Clamp(_s.ThreeDofGlideCeiling, 0.1, 1.0),
            BurnThrottleCeiling = Math.Clamp(_s.ThreeDofBurnCeiling, 0.1, 1.0),
        };
        // THE ACCELERATION LIMIT HOLDS THE WHOLE BURN, and the glide plan times the ignition under it with headroom to spare (Settings.AccelHeadroom). Timed to it but not held to it, as from 8d26ebd, the burn re-solves and 6-DOF after them were free to coast on minimum thrust and brake at the end: the 2026-10-09 7 t booster was handed over at 182 m and 88 m/s, a 3 g burn's height, fell on its floor to 27 m at 48 m/s and planned up to 13.7 g for the last second - a burn too short for any node count the solver could keep up with. The limit had been taken off after a burn stopped 15 m up and 32 m off with it binding (2026-10-08, 18:36), but that plan weighted terminal speed 100x above the miss, which alone makes stopping above the pad the cheap answer; the same commit put the miss above the speed. The limit is soft, so a burn that needs more to stop still gets it.
        // Both plan from above the engines' minimum throttle, so the tracker has thrust to take off as well as to add; see LandingPlanFloorMargin.
        // Both turn no faster than the vehicle can change its turn rate; see ThreeDofTurnAccelDegS2.
        double turn = _s.ThreeDofTurnAccelDegS2 > 0.0 ? _s.ThreeDofTurnAccelDegS2 * Math.PI / 180.0
            : _s.ThreeDofTurnAccelDegS2 < 0.0 ? 0.0
            : Ksa3DofSetup.TurnAccelLimit(vehicle);
        _s.ThreeDofTurnAccelInUse = turn * 180.0 / Math.PI;
        _s.ThreeDofRateInUse = Ksa3DofSetup.PlanRateMax(vehicle);
        double coastTurn = _s.ThreeDofTurnAccelDegS2 == 0.0 ? Ksa3DofSetup.CoastTurnAccelLimit(vehicle) : turn;
        bool fixedCoast = _s.ThreeDofTurnAccelDegS2 == 0.0 && coastTurn == 0.0;
        Scvx3DofConfig glide = Ksa3DofSetup.Config(vehicle, nodes: 30, glideIntervals: 12, _s.GlideMaxAoaDeg,
            _s.VehicleHeightM, settings.GlideAccelMax(LandingAccelLimitMs2), _s.LandingPlanFloorMargin, turn,
            fixedGlideAttitude: fixedCoast, burnTiltPerSDeg: _s.ThreeDofBurnTiltPerSDeg);
        Scvx3DofConfig burn = Ksa3DofSetup.Config(vehicle, nodes: 20, glideIntervals: 0, _s.GlideMaxAoaDeg,
                                                   _s.VehicleHeightM, LandingAccelLimitMs2, _s.LandingPlanFloorMargin, turn,
                                                   burnTiltPerSDeg: _s.ThreeDofBurnTiltPerSDeg);
        var g = new Guidance3Dof(glide, burn, _s.ThreeDofEstimator.Apply(_s.ThreeDofNominal), settings);
        _s.ThreeDofWorker ??= new Ksa3DofWorker();
        // Zero/zero: the legs on the pad, at rest. The plan flies the CoM, VehicleHeightM above the legs.
        double[] xf = [0, 0, _s.VehicleHeightM, 0, 0, 0];
        _s.DiagEntry("PID glide while the entry guidance cold solve runs.");
        GuidanceLog.Info(vehicle, $"Entry guidance cold solve at {x[2] / 1000:F1} km.");
        // Seed the glide the boostback planned: the nominal angle, lifting up the trajectory. Positive C_L lifts towards the engine end's tilt, so the engine end tilts up and the thrust axis down; the other way round for an airframe that lifts the other way.
        double nominal = GlideNominalAlpha(_s.Aero);
        var seed = new Seed3Dof.Options
        {
            GlideTilt = _s.GlideLiftSign != 0 ? nominal : 0.0,
            GlideTiltDirection = _s.GlideLiftSign > 0 ? [0, 0, -1] : [0, 0, 1],
        };
        if (!_s.ThreeDofWorker.TryRun(g, stop =>
        {
            if (stop.IsCancellationRequested || !g.BeginCold(x, xf, now, seed))
                return;
            while (!stop.IsCancellationRequested && g.Phase == Phase3Dof.Converging)
                g.StepCold(2);
        }))
            return;
        _s.ThreeDofColdTime = now;
        _s.ThreeDofGuidance = g;
    }

    /// <summary>
    /// Feed the estimator the non-gravitational acceleration, differenced from the site-frame velocity over the re-solve interval less the model's gravity and frame terms.
    /// </summary>
    private static void Measure3Dof(double[] x, double[] att, double throttle, double now)
    {
        double3 v = new(x[3], x[4], x[5]);
        double dt = now - _s.ThreeDofAccelTime;
        if (!double.IsFinite(_s.ThreeDofAccelTime) || dt <= 0.0 || dt > 1.0)
        {
            _s.ThreeDofAccelTime = now;
            _s.ThreeDofAccelVelocity = v;
            return;
        }
        if (dt < ThreeDofUpdateS)
            return;

        double3 a = (v - _s.ThreeDofAccelVelocity) / dt;
        Span<double> f = stackalloc double[PointMass3Dof.NX];
        KsaPointMassModel bare = _s.ThreeDofNominal with { Drag = null, LiftSlope = null, CrossBiasX = 0, CrossBiasY = 0, CrossBiasZ = 0 };
        PointMass3Dof.Eval(bare, x, [att[0], att[1], att[2], 0.0], f);
        double[] measured = [a.X - f[3], a.Y - f[4], a.Z - f[5]];
        _s.ThreeDofMeasuredAccel = new double3(measured[0], measured[1], measured[2]);
        _s.ThreeDofEstimator.Update(_s.ThreeDofNominal, x, att, throttle, measured, dt);
        _s.ThreeDofAccelTime = now;
        _s.ThreeDofAccelVelocity = v;
    }

    /// <summary>Back to the PID glide, and a fresh cold solve after the retry wait.</summary>
    private static void Drop3Dof(Vehicle vehicle, string why, double now)
    {
        GuidanceLog.Info(vehicle, why);
        _s.DiagEntry(why);
        _s.ThreeDofColdTime = now;
        if (!ThreeDofBusy)
        {
            _s.CancelEntrySolve();
            _s.ThreeDofGuidance = null;
            _s.ThreeDofPlan = null;
        }
    }

    /// <summary>
    /// TOUCHDOWN GUIDANCE'S HEAD START. From the ignition lock to ignition, its standby solves the burn it will be handed: from the state entry guidance predicts at the committed ignition - the craft carried along the locked plan, at that plan's attitude there - and at the ignition time, re-solved as the prediction closes in. When the engine lights, the plan is converged for the state the craft is in, and touchdown guidance can take the burn from its start.
    ///  WHY. The hand-over is ThreeDofSixDofTimeS (5 s) before the pad and the standby used to start at ignition. A landing burn ran 6.6-7.1 s, so the standby had two seconds and took the burn at 250 m and 104 m/s with a converged plan. KSA 5901's aerodynamics gave the boosters about 60 % more drag: they arrive at 103 m/s where they came at 125, the burn is 4.8-5.0 s, ignition and the hand-over time coincide, and the standby solved from cold through the hardest braking and took over 1-1.6 s in, at half the height (2026-11-18).
    ///  The standby is given the forecast as its state and the ignition time as its clock, so nothing in it knows the difference; only the bias estimator and the drift restart, which read the craft's motion, wait for the real thing (SixDofStandbyPredicted).
    /// </summary>
    /// <summary>How often the standby's forecast is renewed while the craft glides, s.</summary>
    private const double SixDofStandbyGlideForecastS = 0.1;

    private static void StepEntryTouchdownPrepare(Vehicle vehicle, IParentBody parent, double3 siteCci,
                                                  double[] x, double[] x14, double now)
    {
        Guidance3Dof g = _s.ThreeDofGuidance;
        Plan3Dof plan = g?.Published;
        Plan3Dof lockPlan = g?.LockPlan;
        if (!_s.ThreeDofSixDofHandover || !_s.ThreeDofTouchdownPrepare || plan == null
            || now >= plan.IgnitionTime || _s.ThreeDofNominal == null)
            return;
        // In the lock window the craft flies the lock plan open loop, so the forecast follows it. EARLIER, in the glide, once ignition is ThreeDofTouchdownPrepareS away, the forecast follows the glide plan: the standby's cold solve then has seconds of glide to converge in rather than the 2.4 s of the lock, and re-solves warm on a forecast that sharpens as ignition nears. On 2026-11-20 a cold solve with RCS took 4-7 s offline, longer than the lock window.
        bool locked = plan.Phase == Phase3Dof.Locked && lockPlan != null;
        bool gliding = plan.Phase == Phase3Dof.Glide && plan.IgnitionTime - now <= _s.ThreeDofTouchdownPrepareS;
        if (!locked && !gliding)
            return;
        double ignition = plan.IgnitionTime;
        // TOUCHDOWN GUIDANCE OWNS IGNITION (SixDofOwnsIgnition): its standby plans the coast and the burn from the craft as it is, every step, and is handed the craft in the glide. The forecast below then only scales its problem.
        bool owns = _s.SixDofOwnsIgnition;
        bool forecast = !gliding || now - _s.ThreeDofPrepareForecastAt >= SixDofStandbyGlideForecastS || _s.SixDofCoastScaleState == null;
        if (!forecast)
        {
            if (owns && _s.SixDofCoastPending)
                StepEntryTouchdownOwn(vehicle, parent, siteCci, plan, x14, now);
            return;
        }
        // Forecast at the standby's own cadence while gliding; every step in the lock, as before.
        _s.ThreeDofPrepareForecastAt = now;
        Plan3Dof flown = locked ? lockPlan : plan;
        KsaPointMassModel model = plan.Model ?? flown.Model ?? _s.ThreeDofNominal;
        double[] predicted = g.PredictAlong(model, x, flown, now, ignition - now, out double[] opening, out _);

        // The 14-state at ignition: the predicted position and velocity, the attitude turned the shortest way from the thrust axis now onto the plan's there, the body rates and the mass as they are.
        double[] xi = (double[])x14.Clone();
        for (int i = 0; i < 6; i++)
            xi[i] = predicted[i];
        double qw = x14[6], qx = x14[7], qy = x14[8], qz = x14[9];
        var axis = new double3(2.0 * (qx * qz + qw * qy), 2.0 * (qy * qz - qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy));
        var target = new double3(opening[0], opening[1], opening[2]);
        double3 turn = double3.Cross(axis, target);
        double sin = turn.Length(), cos = double3.Dot(axis, target);
        if (sin > 1e-9)
        {
            double half = 0.5 * Math.Atan2(sin, cos);
            double3 n = turn / sin * Math.Sin(half);
            double dw = Math.Cos(half);
            // q_at_ignition = q_turn (x) q_now, both taking body axes to the site frame.
            xi[6] = dw * qw - n.X * qx - n.Y * qy - n.Z * qz;
            xi[7] = dw * qx + n.X * qw + n.Y * qz - n.Z * qy;
            xi[8] = dw * qy - n.X * qz + n.Y * qw + n.Z * qx;
            xi[9] = dw * qz + n.X * qy - n.Y * qx + n.Z * qw;
        }

        if (owns)
        {
            _s.SixDofCoastScaleState = xi;
            if (!_s.SixDofCoastPending)
            {
                GuidanceLog.Info(vehicle, FormattableString.Invariant($"Touchdown guidance preparing {ignition - now:F1} s before ignition: planning the coast and the burn from {x14[2]:F0} m, scaled on the state predicted at ignition, {xi[2]:F0} m at {Math.Sqrt(xi[3] * xi[3] + xi[4] * xi[4] + xi[5] * xi[5]):F0} m/s."));
                StopSixDofStandby();
                _s.SixDofStandbyRetryAt = now;
                _s.SixDofStandbyPredicted = false;
                _s.SixDofCoastPending = true;
            }
            StepEntryTouchdownOwn(vehicle, parent, siteCci, plan, x14, now);
            return;
        }

        if (!_s.SixDofStandbyPredicted)
            GuidanceLog.Info(vehicle, FormattableString.Invariant($"Touchdown guidance preparing {ignition - now:F1} s before ignition: planning from {xi[2]:F0} m at {Math.Sqrt(xi[3] * xi[3] + xi[4] * xi[4] + xi[5] * xi[5]):F0} m/s, the state predicted there."));
        _s.SixDofStandbyPredicted = true;
        StepSixDofStandby(vehicle, parent, siteCci, xi, ignition, Math.Max(plan.SigmaBurn, SixDofStandbyMinSeedS));
    }

    /// <summary>
    /// Entry guidance's plan as a path for touchdown guidance's cold seed (ColdSeed6Dof.BuildAlongPath), fixed at <paramref name="now"/>: the coast from now to entry's ignition along its glide, the burn along its burn, with its commanded body axis and its thrust by its own model's law. Null while entry has no glide or lock plan with an ignition still ahead. The plan is immutable, so the worker can sample it.
    /// </summary>
    internal static AdvancedFlightComputer.Guidance.Scvx.SixDof.SeedPath EntrySeedPath(double now)
    {
        Guidance3Dof g = _s.ThreeDofGuidance;
        Plan3Dof plan = g?.Published;
        if (plan?.Phase == Phase3Dof.Locked)
            plan = g.LockPlan;
        else if (plan?.Phase != Phase3Dof.Glide)
            plan = null;
        KsaPointMassModel model = plan?.Model ?? _s.ThreeDofNominal;
        if (plan == null || model == null || !(plan.IgnitionTime > now) || !(plan.SigmaBurn > 0.0))
            return null;
        double ignition = plan.IgnitionTime, coast = ignition - now, burn = plan.SigmaBurn;
        return (int phase, double f, Span<double> rv, Span<double> axis, out double thrustN) =>
        {
            Span<double> xs = stackalloc double[PointMass3Dof.NX];
            Span<double> us = stackalloc double[PointMass3Dof.NU];
            plan.Sample(phase == 0 ? now + f * coast : ignition + f * burn, xs, us);
            xs[..6].CopyTo(rv);
            double norm = Math.Sqrt(us[0] * us[0] + us[1] * us[1] + us[2] * us[2]);
            thrustN = phase == 0 ? 0.0 : model.Thrust(us[3], model.Altitude(xs));
            if (!(norm > 0.5))
                return false;
            for (int i = 0; i < 3; i++)
                axis[i] = us[i] / norm;
            return true;
        };
    }

    /// <summary>
    /// One step of the standby that takes the craft before ignition (SixDofOwnsIgnition): re-solved from the measured state with a coast to entry guidance's ignition, and the hand-over asked for once it is due - ThreeDofTakeoverBeforeIgnitionS before entry's ignition or ThreeDofTakeoverBeforeImpactS before its touchdown, whichever is earlier - and the standby is ready. The claim is the next step's (CheckPendingEntryTouchdownHandoff).
    /// </summary>
    private static void StepEntryTouchdownOwn(Vehicle vehicle, IParentBody parent, double3 siteCci, Plan3Dof plan, double[] x14, double now)
    {
        double ignition = plan.IgnitionTime;
        _s.SixDofCoastSeedS = Math.Max(ignition - now, 0.0);
        StepSixDofStandby(vehicle, parent, siteCci, x14, now, Math.Max(plan.SigmaBurn, SixDofStandbyMinSeedS));
        double due = Math.Min(plan.EndTime - _s.ThreeDofTakeoverBeforeImpactS, ignition - _s.ThreeDofTakeoverBeforeIgnitionS);
        if (now < due || _s.ThreeDofGlideHandoff != null || !SixDofStandbyReady(now))
            return;
        _s.ThreeDofGlideHandoff = FormattableString.Invariant($"HANDOVER IN THE GLIDE: TOUCHDOWN, {ignition - now:F1} s BEFORE ENTRY'S IGNITION, {plan.EndTime - now:F1} s FROM PAD, {x14[2]:F0} m.");
    }

    private static void StepEntryTouchdownHandoff(Vehicle vehicle, IParentBody parent, double3 siteCci,
                                                  double[] x, double now)
    {
        Plan3Dof plan = _s.ThreeDofGuidance?.Published ?? _s.ThreeDofPlan;
        double burnLeft = plan?.EndTime - now ?? double.NaN;
        string reason = _s.ThreeDofBurnSolveFault ? "ENTRY SOLVE FAULT IN BURN" : null;
        if (reason == null && !ThreeDofBusy && _s.ThreeDofGuidance is { } guidance)
        {
            if (guidance.Phase == Phase3Dof.Idle)
                reason = "ENTRY PLAN LOST IN BURN";
            else if (guidance.ConsecutiveRefusals >= ThreeDofBurnRefusalLimit)
                reason = $"ENTRY REFUSED {guidance.ConsecutiveRefusals} RE-SOLVES IN BURN{RefusalReason(guidance)}";
        }
        if (reason == null && burnLeft < -0.5)
            reason = "ENTRY PLAN RAN OUT IN BURN";

        // THE HAND-OVER TIME IS SET WHEN THE ENGINE LIGHTS: ThreeDofSixDofTimeS before the end of the burn as planned then, and not pushed back by a burn that grows afterwards. Entry guidance's re-solves stretch the burn once it runs (5.1 s at the lock, 5.5 s left 1.7 s after ignition on 2026-11-18), and judged on the time left each step, the hand-over waited 2.3 s into a 5 s burn with touchdown guidance's plan ready all along.
        if (!double.IsFinite(_s.ThreeDofHandoverDueAt) && double.IsFinite(burnLeft))
            _s.ThreeDofHandoverDueAt = now + Math.Max(burnLeft - _s.ThreeDofSixDofTimeS, 0.0);
        bool due = burnLeft < _s.ThreeDofSixDofTimeS || now >= _s.ThreeDofHandoverDueAt;
        bool handoff = reason != null || (_s.ThreeDofSixDofHandover && due);
        if (handoff && !Can6DofTake(vehicle, out string refusal))
        {
            StopSixDofStandby();
            KeepEntryBurn(vehicle, refusal);
            return;
        }
        _s.ThreeDofHandoffRefusal = "";
        double seed = double.IsFinite(burnLeft) ? Math.Max(burnLeft, SixDofStandbyMinSeedS) : _s.SixDofSigmaSeed;
        StepSixDofStandby(vehicle, parent, siteCci, x, now, seed, holdForEntryHandoff: handoff);
        if (!handoff)
            return;

        if (!SixDofStandbyReady(now, SixDofStandbyMaxPlanAgeS, x))
        {
            if (!_s.ThreeDofSixDofWaiting)
            {
                _s.ThreeDofSixDofWaiting = true;
                GuidanceLog.Info(vehicle, $"{reason ?? "Entry guidance approaching touchdown"}: keeping the burn while touchdown guidance is {SixDofStandbyText(now)}.");
            }
            return;
        }

        string status = reason != null
            ? reason + ". HANDOVER: TOUCHDOWN."
            : FormattableString.Invariant($"HANDOVER: TOUCHDOWN, {burnLeft:F1} s FROM PAD, {x[2]:F0} m.");
        _s.ThreeDofTouchdownHandoff = (status, reason != null);
    }

    private static void CheckPendingEntryTouchdownHandoff(Vehicle vehicle, IParentBody parent)
    {
        // IN THE GLIDE (SixDofOwnsIgnition): the engine is off and stays off until touchdown guidance's plan lights it.
        string glide = _s.ThreeDofGlideHandoff;
        _s.ThreeDofGlideHandoff = null;
        if (glide != null && _s.ThreeDofPhase == ThreeDofPhase.Glide && _s.SixDofCoastPending && !HasTouchedDown(vehicle))
        {
            if (!Can6DofTake(vehicle, out string why) || !StandbyEnginesStillMatch(vehicle))
            {
                GuidanceLog.Info(vehicle, "Touchdown guidance cannot take the glide: " + (why ?? "ENGINE CONFIGURATION CHANGED.") + " Entry guidance keeps it.");
                StopSixDofStandby();
                _s.SixDofCoastPending = false;
                return;
            }
            if (!SixDofStandbyReady(SimNow()))
                return;
            GuidanceLog.Info(vehicle, glide);
            Engage6Dof(vehicle, fromBrakingBurn: true, tookBurn: false);
            _s.ThreeDofStatus = glide;
            return;
        }

        var (status, warning) = _s.ThreeDofTouchdownHandoff;
        _s.ThreeDofTouchdownHandoff = default;
        if (status == null || _s.ThreeDofPhase != ThreeDofPhase.Burn || HasTouchedDown(vehicle))
            return;
        // A warning can take the burn only once the entry worker has finished without publishing a replacement.
        if (warning && (ThreeDofBusy || (_s.ThreeDofGuidance?.Published is { } recovered
            && !ReferenceEquals(recovered, _s.ThreeDofPlan))))
            return;
        if (!Can6DofTake(vehicle, out string refusal))
        {
            StopSixDofStandby();
            KeepEntryBurn(vehicle, refusal);
            return;
        }
        if (!StandbyEnginesStillMatch(vehicle))
        {
            StopSixDofStandby();
            KeepEntryBurn(vehicle, "ENGINE CONFIGURATION CHANGED.");
            return;
        }

        double3 siteCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        double[] x = KsaFrameBridge.ToModelState(vehicle, KsaFrameBridge.BuildSiteFrame(siteCci));
        if (!SixDofStandbyReady(SimNow(), SixDofStandbyMaxPlanAgeS, x))
            return;
        if (x[5] >= 0.0 && TerminalMinThrottleTwr(vehicle, vehicle.Orbit, parent.Mu) > 1.0)
            return;

        ref ManualControlInputs carried = ref ManualInputs(vehicle);
        if (!carried.EngineOn || !(carried.EngineThrottle > 0.0f))
            return;
        double throttle = carried.EngineThrottle;
        GuidanceLog.Info(vehicle, status);
        Engage6Dof(vehicle, fromBrakingBurn: true, tookBurn: true);
        _s.SixDofEntryBlendStart = SimNow();
        _s.SixDofEntryBlendThrottle = throttle;
        _s.ThreeDofStatus = status;
        if (warning)
            _s.WarnEntry(status);
    }

    private static void KeepEntryBurn(Vehicle vehicle, string reason)
    {
        string status = "KEEPING ENTRY BURN. TOUCHDOWN UNAVAILABLE: " + reason;
        if (!_s.ThreeDofSixDofWaiting)
            GuidanceLog.Info(vehicle, status);
        _s.ThreeDofSixDofWaiting = true;
        _s.ThreeDofHandoffRefusal = status;
        _s.WarnEntry(status);
    }

    internal static string ThreeDofPhaseName(ThreeDofPhase p) => p switch
    {
        ThreeDofPhase.Idle => "IDLE",
        ThreeDofPhase.Glide => _s.ThreeDofPlan != null ? "GLIDE ON PLAN" : "GLIDE, NO PLAN",
        ThreeDofPhase.Burn => "LANDING BURN",
        ThreeDofPhase.Done => "ENDED",
        _ => "?",
    };
}

#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using KSA;
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
// FALLBACKS. Before the burn: a cold solve that fails, or a run of refused re-solves, flies the PID glide again and starts a fresh cold solve. After ignition the glide cannot catch it, so a plan that runs out, or a long run of refusals, hands the burn to 6-DOF, which flies in air on its own model.
//
// THE BURN ENDS AT ZERO/ZERO - the legs on the pad, at rest (the CoM VehicleHeightM above the site) - and the engine is cut at contact, unless, in the burn's last ThreeDofSixDofTimeS (5 s, on by default), 6-DOF has taken the burn first, for its rotational model of the final flare.
//
// BY TIME, NOT SPEED. The hand-over was below 100 m/s, which a heavy booster reaches a few seconds from the pad but a light one, falling at a terminal speed of ~50 m/s, reaches at ignition: a 20 t booster lit 2.3 km up was handed a 40 s burn (headless, 2026-10-08). 6-DOF has no engine-off coast and no drag model - only a constant bias - so over a burn that long it rides its thrust floor, tilts 25-30 degrees to shed what it cannot throttle away, wanders hundreds of metres, and loses the plan as drag falls off. Its last seconds are what it is for, whatever the speed. There is no hover hand-off: a craft that cannot throttle below its weight could not be handed one, and one that stops short is cut and dropped instead (see the burn's stop-short check).
// 6-DOF is ON STANDBY from ignition (SixDofStandby.cs): its guidance cold-solves and then re-solves on its own thread from the measured state while this mode flies, so it takes the burn on a warm plan. In the last seconds it takes over only once that plan is converged and fresh; until then this mode keeps the burn, to touchdown if need be. The fallbacks above hand over whatever state the standby is in, which at worst is a cold solve already under way, seeded with this burn's time to go.
//
// TODO (offline harness, --3dof-mpc, which flies the 3-DOF to touchdown with no 6-DOF hand-over): at zero/zero the nominal cases touch down at 0.4-0.7 m/s, but a 30 % lift deficit, an 8 % weak engine and a second's late ignition touch down at 3-4.5 m/s. In each, every burn re-solve from a few hundred metres down is refused on the mass channel (1-5x its tolerance), as it was against the old 20 m aim point, and the tracker flies a plan up to 8 s stale to the ground. In flight ThreeDofBurnRefusalLimit hands those burns to 6-DOF first.
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

    /// <summary>Refused re-solves in a row before the glide falls back to the PID and starts again cold, and before a lit burn goes to 6-DOF.</summary>
    private const int ThreeDofGlideRefusalLimit = 20;
    private const int ThreeDofBurnRefusalLimit = 10;

    /// <summary>Wait between cold-solve attempts after one fails, s.</summary>
    private const double ThreeDofColdRetryS = 2.0;

    private static bool ThreeDofBusy => _s.ThreeDofWorker != null && _s.ThreeDofWorker.Busy;

    /// <summary>
    /// Take the craft for the 3-DOF landing. From the boostback glide, or from the page. The glide state carries over: until the first plan this mode flies the same PID glide.
    /// </summary>
    private static void Engage3Dof(Vehicle vehicle, double now)
    {
        ClaimVehicle(GuidanceMode.ThreeDof, vehicle);
        _s.Engage = true;
        Reset3Dof();
        _s.ThreeDofPhase = ThreeDofPhase.Glide;
        _s.ThreeDofLastStep = now;
        // A fresh flown path for the entry page's plot, which touchdown guidance's plot carries on after the hand-over.
        ResetGfoldTrace();
        _s.ThreeDofStatus = "Entry guidance engaged; PID glide while the cold solve runs.";
        GuidanceLog.Info(vehicle, $"Entry guidance engaged, {_s.GlideTgo:F0} s to impact.");
        string noLift = AeroLiftWarning();
        if (noLift.Length > 0)
        {
            GuidanceLog.Info(vehicle, "Entry guidance: " + noLift);
            _s.ThreeDofStatus = "Engaged without lift - ballistic fall, the burn lands it.";
        }
    }

    /// <summary>Stop the 3-DOF. In the air the engine is left as it is; a cut there drops the craft.</summary>
    private static void Disengage3Dof(string why)
    {
        bool burning = _s.ThreeDofPhase == ThreeDofPhase.Burn;
        StopSixDofStandby();
        Reset3Dof();
        _s.ThreeDofPhase = ThreeDofPhase.Done;
        if (burning)
            _s.ReleaseWithoutEngineCut = true;
        else
            _s.LandingCutPending = true;
        _s.ThreeDofStatus = why;
    }

    /// <summary>Forget the guidance, the model and the command history; the worker finishes whatever it is on and is kept for the next engage.</summary>
    private static void Reset3Dof()
    {
        _s.ThreeDofGuidance = null;
        _s.ThreeDofNominal = null;
        _s.ThreeDofEstimator = null;
        _s.ThreeDofAccelTime = _s.ThreeDofLogTime = double.NaN;
        _s.ThreeDofLastUpdate = _s.ThreeDofColdTime = double.NegativeInfinity;
        _s.ThreeDofPlan = null;
        _s.ThreeDofCommandLocal = default;
        _s.ThreeDofSolveText = "";
        _s.ThreeDofRefusing = false;
        _s.ThreeDofEngineOn = false;
        _s.ThreeDofThrottle = _s.ThreeDofPlanThrottle = 0.0;
        _s.ThreeDofSixDofWaiting = false;
    }

    /// <summary>One step of the 3-DOF landing, run for this vehicle from ApplyAutopilot whether or not it is the one on screen.</summary>
    private static void Step3Dof(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        double now = SimNow();
        if (_s.ThreeDofEngagePending)
        {
            _s.ThreeDofEngagePending = false;
            Engage3Dof(vehicle, now);
        }
        if (!ThreeDofLive)
            return;

        double dt = Math.Clamp(now - _s.ThreeDofLastStep, 0.0, 1.0);
        _s.ThreeDofLastStep = now;

        if (HasTouchedDown(vehicle))
        {
            GuidanceLog.Info(vehicle, "Entry guidance: touchdown, engine cut.");
            StopSixDofStandby();
            Reset3Dof();
            _s.ThreeDofPhase = ThreeDofPhase.Done;
            _s.LandingCutPending = true;
            _s.ThreeDofStatus = "Touchdown - engine cut.";
            return;
        }

        if (_s.ThreeDofWorker?.LastError is { Length: > 0 } fault)
        {
            GuidanceLog.Info(vehicle, "Entry guidance solve faulted: " + fault);
            _s.ThreeDofWorker.LastError = "";
            Drop3Dof(vehicle, "Entry guidance solve faulted - PID glide, retrying cold.", now);
        }

        EnsureBoostbackAero(vehicle, parent);
        KsaAeroSweep.Result aero = _s.Aero;
        double3 siteCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        KsaFrameBridge.SiteFrame frame = KsaFrameBridge.BuildSiteFrame(siteCci);
        double[] x14 = KsaFrameBridge.ToModelState(vehicle, frame);
        double[] x = [x14[0], x14[1], x14[2], x14[3], x14[4], x14[5], vehicle.TotalMass];
        RecordGfoldTrace(Math.Sqrt(x14[0] * x14[0] + x14[1] * x14[1]), x14[2]);
        double3 attLocal = frame.VecToLocal(ThrustAxisCci(vehicle)).NormalizeOrZero();
        double[] att = [attLocal.X, attLocal.Y, attLocal.Z];
        bool burning = _s.ThreeDofPhase == ThreeDofPhase.Burn;

        if (aero?.Table != null && aero.Atmosphere != null && _s.ThreeDofNominal == null)
        {
            _s.ThreeDofNominal = Ksa3DofSetup.BuildModel(vehicle, parent, frame, aero, out string error);
            if (_s.ThreeDofNominal == null)
                _s.ThreeDofStatus = error;
        }
        _s.ThreeDofEstimator ??= new AeroScaleEstimator();
        if (_s.ThreeDofNominal != null)
            Measure3Dof(x, att, burning ? _s.ThreeDofThrottle : 0.0, now);

        // 6-DOF on standby through the burn, on its own thread, so the hand-over finds it with a warm plan; ahead of the plan step, which decides the hand-over on what the standby has. Seeded with this burn's time to go.
        if (burning && _s.ThreeDofPlan != null)
            StepSixDofStandby(vehicle, parent, siteCci, x14, now, Math.Max(_s.ThreeDofPlan.EndTime - now, SixDofStandbyMinSeedS));

        // Fly the plan if there is one; otherwise the PID glide until there is.
        if (!Plan3DofStep(vehicle, parent, frame, x, att, now, dt, out double3 wantLocal, out double3 rateLocal, out bool smooth))
        {
            if (!ThreeDofLive)
                return;   // a handover claimed the craft on this step
            if (_s.ThreeDofPhase == ThreeDofPhase.Burn)
            {
                // Lit with no plan for this step: hold the command and the throttle rather than steer on a law the burn does not use.
                CommandThreeDof(frame, frame.VecToLocal(_s.CommandDir), default, dt, smooth: false);
                return;
            }
            double3 glide = GlideDirection(vehicle, orbit, parent, now, dt);
            double3 glideRate = BoostbackTargetRate(glide, dt);
            CommandThreeDof(frame, frame.VecToLocal(glide), frame.VecToLocal(glideRate), dt, smooth: false);
            _s.ThreeDofEngineOn = false;
            _s.ThreeDofThrottle = 0.0;
            return;
        }
        CommandThreeDof(frame, wantLocal, rateLocal, dt, smooth);
        Log3Dof(vehicle, x, att, now);
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
            "Entry guidance {0}: alt {1:F2} km, {2:F0} m/s, {3:F0} m from the site; {4}; flown {5:F1} deg off the command; "
            + "{6}; drag x{7:F2} lift x{8:F2} thrust x{9:F2}{10}",
            ThreeDofPhaseName(_s.ThreeDofPhase), x[2] / 1000.0, speed, Math.Sqrt(x[0] * x[0] + x[1] * x[1]),
            plan == null ? "no plan"
                : _s.ThreeDofPhase == ThreeDofPhase.Burn
                    ? $"burn {plan.EndTime - now:F1} s left, throttle {_s.ThreeDofThrottle:P0} (plan {_s.ThreeDofPlanThrottle:P0}), {TrackingErrorText(plan, x, now)}"
                : $"ignition in {plan.IgnitionTime - now:F1} s, burn {plan.SigmaBurn:F1} s",
            off,
            g == null ? "no solver" : ThreeDofBusy ? "solving" : $"last solve {g.LastSolveMs:F0} ms, {g.ConsecutiveRefusals} refused{RefusalReason(g)}",
            est?.DragScale ?? 1.0, est?.LiftScale ?? 1.0, est?.ThrustScale ?? 1.0,
            _s.ThreeDofPhase == ThreeDofPhase.Burn ? "; touchdown guidance standby " + SixDofStandbyText(now) : ""));
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
                _s.ThreeDofStatus = "Waiting for the aero surrogate.";
            return false;
        }

        Guidance3Dof g = _s.ThreeDofGuidance;
        if (g == null)
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

        if (!ThreeDofBusy)
        {
            _s.ThreeDofSolveText = $"{g.LastSolveMs,8:F0} ms, {g.LastIterations} it, {g.ConsecutiveRefusals} refused";
            _s.ThreeDofRefusing = g.ConsecutiveRefusals > 0;
            // Results are read only while the worker is idle: the guidance is the worker's while it solves.
            if (g.Phase == Phase3Dof.Idle || (g.Phase == Phase3Dof.Glide && g.ConsecutiveRefusals >= ThreeDofGlideRefusalLimit))
            {
                if (burning)
                {
                    ThreeDofToSixDof(vehicle, "Entry guidance lost its plan in the burn - touchdown guidance takes over.");
                    return false;
                }
                Drop3Dof(vehicle, g.Phase == Phase3Dof.Idle
                    ? "Entry guidance " + g.Error + " - PID glide, retrying cold."
                    : $"Entry guidance refused {g.ConsecutiveRefusals} re-solves{RefusalReason(g)} - PID glide, retrying cold.", now);
                return false;
            }
            if (burning && g.ConsecutiveRefusals >= ThreeDofBurnRefusalLimit)
            {
                ThreeDofToSixDof(vehicle, $"Entry guidance refused {g.ConsecutiveRefusals} re-solves in the burn{RefusalReason(g)} - touchdown guidance takes over.");
                return false;
            }
            if (g.Phase != Phase3Dof.Converging && now - _s.ThreeDofLastUpdate >= ThreeDofUpdateS)
            {
                _s.ThreeDofLastUpdate = now;
                KsaPointMassModel model = _s.ThreeDofEstimator.Apply(_s.ThreeDofNominal);
                double[] x0 = x, a0 = [anchor.X, anchor.Y, anchor.Z];
                double[] r0 = turnKnown ? [anchorTurn.X, anchorTurn.Y, anchorTurn.Z] : null;
                _s.ThreeDofWorker.TryRun(() =>
                {
                    g.Model = model;
                    g.Update(x0, a0, now, r0);
                });
            }
        }

        Plan3Dof plan = g.Published;
        if (plan == null)
        {
            _s.ThreeDofStatus = "PID glide while the entry guidance cold solve runs.";
            return false;
        }
        if (!ReferenceEquals(plan, _s.ThreeDofPlan))
        {
            if (_s.ThreeDofPlan == null)
                GuidanceLog.Info(vehicle, $"Entry guidance first plan at {x[2] / 1000:F1} km: ignition in {plan.IgnitionTime - now:F1} s, burn {plan.SigmaBurn:F1} s.");
            _s.ThreeDofPlan = plan;
        }

        // Past the end with the engine lit and nothing newer: the plan is no longer guidance.
        if (burning && now > plan.EndTime + 0.5)
        {
            ThreeDofToSixDof(vehicle, "Entry guidance plan ran out in the burn - touchdown guidance takes over.");
            return false;
        }

        double height = x[2];
        double speed = Math.Sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
        if (burning)
        {
            // STOPPED SHORT. The plan ends at zero/zero, but a craft whose minimum thrust is above its weight climbs from wherever it actually stops, and nothing with the engine lit brings it back: the engine is cut and it drops the rest of the way. 6-DOF is no better placed, since it keeps the engine lit too.
            if (x[5] >= 0.0)
            {
                double minTwr = TerminalMinThrottleTwr(vehicle, vehicle.Orbit, parent.Mu);
                if (minTwr > 1.0)
                {
                    string why = $"Entry guidance stopped short {height - _s.VehicleHeightM:F1} m above the pad, rising at {x[5]:F1} m/s, "
                               + $"and cannot throttle below its weight ({minTwr:F2} at minimum) - engine cut, dropping to the pad.";
                    GuidanceLog.Info(vehicle, why);
                    StopSixDofStandby();
                    Reset3Dof();
                    _s.ThreeDofPhase = ThreeDofPhase.Done;
                    _s.LandingCutPending = true;
                    _s.ThreeDofStatus = why;
                    return false;
                }
            }
            // Only to a standby with a warm plan: a 6-DOF that is still converging would leave the craft on the 3-DOF's last command until it is, which this mode can fly better.
            double burnLeft = plan.EndTime - now;
            if (_s.ThreeDofSixDofHandover && burnLeft < _s.ThreeDofSixDofTimeS)
            {
                if (SixDofStandbyReady(now))
                {
                    ThreeDofToSixDof(vehicle, $"Entry guidance hands the burn to touchdown guidance {burnLeft:F1} s from the pad, at {height:F0} m and {speed:F0} m/s.");
                    return false;
                }
                if (!_s.ThreeDofSixDofWaiting)
                {
                    _s.ThreeDofSixDofWaiting = true;
                    GuidanceLog.Info(vehicle, $"Entry guidance {burnLeft:F1} s from the pad at {height:F0} m, touchdown guidance not ready ({SixDofStandbyText(now)}) - entry guidance keeps the burn until it is.");
                }
            }
        }

        // The newest plan's attitude and throttle now, and its own rate of turn.
        if (!SamplePlan(plan, now, out double3 b0, out double throttle) || !SamplePlan(plan, now + ThreeDofRateProbeS, out double3 b1, out _))
            return false;
        if (plan.Phase == Phase3Dof.Locked && now < plan.IgnitionTime)
        {
            // Ignition locked: the burn's opening attitude is ahead of us. Turn towards it as the ignition prediction assumed, so the burn opens where the vehicle actually is: speeding up and slowing down at the turn-acceleration limit, as fast as it can still stop on the opening attitude, capped at the rate limit (Guidance3Dof.PredictIgnition). Turned at the rate limit from a standing start, the 2026-10-09 booster was 14 deg off its command by ignition.
            double3 from = _s.ThreeDofCommandLocal.Length() > 0.5 ? _s.ThreeDofCommandLocal : anchor;
            double accel = _s.ThreeDofTurnAccelInUse * Math.PI / 180.0;
            double turnSpeed = Ksa3DofSetup.RateMax;
            if (accel > 0.0)
            {
                double angle = Math.Acos(Math.Clamp(double3.Dot(from.NormalizeOrZero(), b0.NormalizeOrZero()), -1.0, 1.0));
                double want = Math.Min(Ksa3DofSetup.RateMax, Math.Sqrt(2.0 * accel * angle));
                double was = _s.ThreeDofLockTurn;
                turnSpeed = was < want ? Math.Min(want, was + accel * dt) : Math.Max(want, was - accel * dt);
            }
            _s.ThreeDofLockTurn = turnSpeed;
            double3 next = SlewToward(from, b0, turnSpeed * dt, out bool turning);
            wantLocal = next;
            rateLocal = turning && dt > 1e-9 ? double3.Cross(from, next).NormalizeOrZero() * turnSpeed : default;
            smooth = false;
        }
        else
        {
            _s.ThreeDofLockTurn = 0.0;
            wantLocal = b0;
            rateLocal = double3.Cross(b0, b1) / ThreeDofRateProbeS;
            smooth = true;
        }

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
            KsaPointMassModel model = g.Model;
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
        _s.ThreeDofStatus = burning ? "Burning on the plan." : "Gliding on the plan.";
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
        return $"{dr:F1} m and {dv:F1} m/s off the plan";
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
        if (smooth && _s.CommandDir.Length() > 0.5)
            want = (_s.CommandDir + SmoothGain(dt) * (want - _s.CommandDir)).NormalizeOrZero();
        double3 rate = frame.VecToCci(rateLocal);
        double maxRad = ThreeDofSlewMax * dt;
        bool clamped = false;
        double3 slewed = _s.CommandDir.Length() > 0.5 ? SlewToward(_s.CommandDir, want, maxRad, out clamped) : want;
        if (clamped && dt > 1e-9)
        {
            double3 axis = double3.Cross(_s.CommandDir, slewed);
            rate = axis.Length() > 1e-12 ? double3.Normalize(axis) * (maxRad / dt) : default;
        }
        _s.CommandDir = slewed;
        _s.CommandRate = rate;
        _s.HasCommand = _s.CommandDir.Length() > 0.5;
        _s.ThreeDofCommandLocal = frame.VecToLocal(slewed);
    }

    private static void StartThreeDofCold(Vehicle vehicle, double[] x, double now)
    {
        _s.ThreeDofColdTime = now;
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
        Scvx3DofConfig glide = Ksa3DofSetup.Config(vehicle, nodes: 30, glideIntervals: 12, _s.GlideMaxAoaDeg,
                                                    _s.VehicleHeightM, settings.GlideAccelMax(LandingAccelLimitMs2), _s.LandingPlanFloorMargin, turn);
        Scvx3DofConfig burn = Ksa3DofSetup.Config(vehicle, nodes: 20, glideIntervals: 0, _s.GlideMaxAoaDeg,
                                                   _s.VehicleHeightM, LandingAccelLimitMs2, _s.LandingPlanFloorMargin, turn);
        var g = new Guidance3Dof(glide, burn, _s.ThreeDofEstimator.Apply(_s.ThreeDofNominal), settings);
        _s.ThreeDofGuidance = g;
        _s.ThreeDofWorker ??= new Ksa3DofWorker();
        // Zero/zero: the legs on the pad, at rest. The plan flies the CoM, VehicleHeightM above the legs.
        double[] xf = [0, 0, _s.VehicleHeightM, 0, 0, 0];
        _s.ThreeDofStatus = "PID glide while the entry guidance cold solve runs.";
        GuidanceLog.Info(vehicle, $"Entry guidance cold solve at {x[2] / 1000:F1} km.");
        // Seed the glide the boostback planned: the nominal angle, lifting up the trajectory. Positive C_L lifts towards the engine end's tilt, so the engine end tilts up and the thrust axis down; the other way round for an airframe that lifts the other way.
        double nominal = GlideNominalAlpha(_s.Aero);
        var seed = new Seed3Dof.Options
        {
            GlideTilt = _s.GlideLiftSign != 0 ? nominal : 0.0,
            GlideTiltDirection = _s.GlideLiftSign > 0 ? [0, 0, -1] : [0, 0, 1],
        };
        _s.ThreeDofWorker.TryRun(() =>
        {
            if (!g.BeginCold(x, xf, now, seed))
                return;
            while (g.Phase == Phase3Dof.Converging)
                g.StepCold(10);
        });
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
        KsaPointMassModel bare = _s.ThreeDofNominal with { Drag = null, LiftSlope = null };
        PointMass3Dof.Eval(bare, x, [att[0], att[1], att[2], 0.0], f);
        double[] measured = [a.X - f[3], a.Y - f[4], a.Z - f[5]];
        _s.ThreeDofEstimator.Update(_s.ThreeDofNominal, x, att, throttle, measured, dt);
        _s.ThreeDofAccelTime = now;
        _s.ThreeDofAccelVelocity = v;
    }

    /// <summary>Back to the PID glide, and a fresh cold solve after the retry wait.</summary>
    private static void Drop3Dof(Vehicle vehicle, string why, double now)
    {
        GuidanceLog.Info(vehicle, why);
        _s.ThreeDofStatus = why;
        _s.ThreeDofColdTime = now;
        if (!ThreeDofBusy)
        {
            _s.ThreeDofGuidance = null;
            _s.ThreeDofPlan = null;
        }
    }

    /// <summary>Hand the burn to 6-DOF: the standby if there is one (promoted on 6-DOF's next step), else a cold engage seeded with the burn's time to go.</summary>
    private static void ThreeDofToSixDof(Vehicle vehicle, string why)
    {
        GuidanceLog.Info(vehicle, why);
        double seed = _s.ThreeDofPlan != null
            ? Math.Max(_s.ThreeDofPlan.EndTime - SimNow(), SixDofStandbyMinSeedS)
            : double.NaN;
        Reset3Dof();
        _s.ThreeDofStatus = why;
        Engage6Dof(vehicle, sigmaSeed: seed, tookBurn: true);
    }

    internal static string ThreeDofPhaseName(ThreeDofPhase p) => p switch
    {
        ThreeDofPhase.Idle => "idle",
        ThreeDofPhase.Glide => _s.ThreeDofPlan != null ? "glide on the plan" : "PID glide, planning",
        ThreeDofPhase.Burn => "landing burn",
        ThreeDofPhase.Done => "ended",
        _ => "?",
    };
}

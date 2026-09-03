using System;
using Brutal.Numerics;
using KSA;
using Navbox.Flight;
using Navbox.Numerics;

// THE BOOSTBACK STATE MACHINE: separate, turn round, burn back, and set up for entry.
//
// Four phases, flown in order, each ending on a condition rather than a timer where
// one exists:
//
//   Separation     2 s at the vehicle's LOWEST throttle. Not a manoeuvre - it settles
//                  propellant and pushes the booster clear of whatever it just let go
//                  of, while the attitude is simply held where separation left it.
//   Rotation       slew the commanded thrust axis onto the boostback dV, at a limited
//                  rate, with the engine STILL LIT at that same minimum throttle - the
//                  gimbals are what turn a booster this size and they need thrust to
//                  deflect. The flight computer flies it: we publish both the attitude
//                  AND the rate the attitude is turning at (see KsaAttitudeRate), so
//                  the FC tracks the slew instead of nulling a sequence of stationary
//                  targets.
//   Boostback      full throttle along the PLAN, re-solved every two seconds. The
//                  plan is a shot: an integrated powered arc whose pitch, yaw and
//                  duration were optimised for propellant (see BoostbackShooter), and
//                  what is flown is its steering law evaluated at the elapsed time.
//                  At tgo <= 5 s the plan FREEZES and the rest is open loop - see
//                  BoostbackPlanFreezeS.
//   EntryOrient    slew to surface retrograde and HOLD there. Terminal: nothing ends
//                  it but an abort or another mode taking the vehicle.
//
// TWO GUIDANCE LAWS RUN HERE, AND THEY DO DIFFERENT JOBS.
//
// ImpactSteering.Correction answers "what is the smallest impulse, applied NOW, that
// puts the predicted impact on the site". It is cheap (about 3.6 ms) and it is the
// right question to ask ABOUT A BURN THAT HAS NOT STARTED: is there anything worth
// lighting an engine for, and is there anything left to correct. That is all it is used
// for here - see the two magnitudes in StepBoostback.
//
// It is the wrong thing to FLY, and measurably so. A boostback burn is not an impulse:
// it lasts tens of seconds and the vehicle falls through all of them. Pretending
// otherwise makes the cheapest-looking correction point at the ground - 33 degrees
// below the horizon on the reference arc, which is the documented failure of
// instantaneous impact-point guidance. BoostbackShooter integrates the powered arc
// instead and optimises the burn as a whole; on that same arc it comes out 25 degrees
// ABOVE the horizon for 18% less propellant, and below the horizon the burn is not
// merely dear but impossible for the vehicle. So the plan flies the burn.
//
// AND THE PLAN IS RE-SOLVED RATHER THAN PLANNED ONCE, on a receding horizon: every
// solve plans a burn starting NOW, from the live state, and the vehicle flies the head
// of the freshest one. What that absorbs is everything the shot does not model - thrust
// that differs from the engine table, the attitude lagging the command, drag that is
// not quite the surrogate. Two seconds is the cadence because the plan answers a slow
// question ("what shape of burn is cheapest from here") whose answer barely moves in
// two seconds, which is also what makes the warm re-solve cheap enough to run at all.
//
// THE BURN ATTITUDE AND THE PREDICTION'S ASSUMPTION AGREE, which is load-bearing and
// not a coincidence worth leaving unstated. DragCoastSystem holds alpha = 0 for the
// whole coast — engine into the wind — and both flown attitudes land there: the
// boostback dV is very nearly anti-parallel to the velocity, so thrusting along it
// points body +x aft, and the entry phase commands exactly that direction outright. The
// only stretch of the flight where the vehicle is NOT near alpha 0 is the rotation, and
// nothing is being predicted through it. If the burn ever became a large plane change,
// that assumption would stop holding and the prediction would need the flown alpha.
//
// AND WHY THE PLAN STOPS BEING RE-SOLVED AT THE END. Both laws degrade the same way as
// the miss goes to zero: the answer is the ratio of two small numbers, and the last few
// seconds of a burn are exactly where the prediction is noisiest (the terrain height
// under a moving impact point is low-passed, not exact). Re-solving there makes the
// vehicle chase its own numerical noise at the moment precision matters most.
//
// What is frozen is the PLAN, not a direction, and that is strictly better than the
// lock it replaces: the linear tangent law goes on being evaluated, so the command
// keeps turning through cutoff exactly as the optimised burn intended, instead of
// holding whatever vector it happened to have at the freeze.
public static partial class PoweredGuidanceWindow
{
    // Public because VehicleAutopilotState holds a vehicle's phase - every craft runs
    // this machine on its own, exactly like AscentPhase and LandingPhase.
    public enum BoostbackPhase { Idle, Separation, Rotation, Boostback, EntryOrient, Done }

    /// <summary>
    /// How often the BURN PLAN is re-solved, seconds of sim time.
    ///
    /// Two orders of magnitude slower than the impulsive correction beside it, and
    /// that asymmetry is the whole reason both exist. A shot is an integrated powered
    /// arc inside a two-variable Newton inside a pattern search, where a Jacobian is
    /// three seeded sweeps. But it is also answering a slower QUESTION: the correction
    /// tracks a miss that shrinks by kilometres a second, while the plan answers "what
    /// shape of burn is cheapest from here", and that shape barely moves in two seconds
    /// - which is precisely what makes the warm re-solve cheap enough to run at all.
    ///
    /// IT RUNS ON THE SIM THREAD, so the cost is one frame in every hundred-odd taking
    /// a warm solve longer. Measured by --shoot at 2.3 ms in Release, which is what the
    /// packaged mod is built as, against 24.6 ms in Debug - a tenfold gap that is worth
    /// knowing about before reading a profile. Either is affordable at this cadence,
    /// and it is deliberately not hidden: BoostbackPlanSolveMs is on the tab, and if it
    /// reads in the hundreds on some vehicle this wants moving onto the worker thread
    /// the 6-DOF solver already uses rather than being left to stutter.
    /// </summary>
    private const double BoostbackPlanIntervalS = 2.0;

    /// <summary>
    /// Burn time remaining at which the plan stops being re-solved, s.
    ///
    /// The same argument as the steering lock it replaces: the last seconds are where
    /// the prediction is noisiest - the terrain height under a moving impact point is
    /// low-passed, not exact - and where a changed answer buys least. What is frozen is
    /// different, though, and better: the old lock froze a DIRECTION and flew a fixed
    /// vector, whereas freezing the plan keeps evaluating the linear tangent law, so
    /// the command goes on turning through cutoff exactly as the optimised burn
    /// intended.
    ///
    /// AND IT IS NOT FREE. --shoot flies the loop against an engine 3% down on thrust:
    /// open loop the burn misses by 6.91 km, re-solved every two seconds by 875 m, and
    /// re-solved right through to cutoff by 240 m. So five seconds of open loop costs
    /// about 600 m of that residue, because a thrust error persisting through the
    /// window is by definition unabsorbed.
    ///
    /// It is still the right trade, but the measurement does not show that, and should
    /// not be read as though it did: the check has a perfect prediction, so it prices
    /// what the freeze COSTS without reproducing what it BUYS. If the terrain filter
    /// were ever made exact, this number would deserve revisiting.
    /// </summary>
    private const double BoostbackPlanFreezeS = 5.0;

    /// <summary>
    /// Coast step sizes for the in-flight solve, s, against the overlay's 1 and 8.
    ///
    /// The coast dominates a shot's cost, and it only has to place a target to tens of
    /// metres for the inner Newton to converge on it - measured at four times cheaper
    /// for no change in the burn that comes out. The overlay keeps the fine steps
    /// because it is DRAWING the trajectory, where a coarse step is visible.
    /// </summary>
    private const double BoostbackPlanCoastStepAir = 3.0;
    private const double BoostbackPlanCoastStepVac = 16.0;

    /// <summary>
    /// How often the correction is re-solved during the boostback, ms.
    ///
    /// Faster than the overlay's 250 ms because this one is flown rather than drawn:
    /// the miss shrinks by kilometres a second under a full-throttle burn, and a
    /// quarter-second-old answer is a quarter second of steering in the wrong place. A
    /// Jacobian is three seeded sweeps, about 3.6 ms, so 100 ms is ~4% of one core.
    /// </summary>
    private const long BoostbackSteerIntervalMs = 100;

    /// <summary>
    /// <summary>Alignment, in degrees, at which the rotation is called complete. Both
    /// the COMMAND and the vehicle have to be inside it - the command reaching the
    /// target only means the slew has finished issuing, not that anything turned.</summary>
    private const double BoostbackAlignDeg = 2.0;

    /// <summary>
    /// Below this much correction there is nothing worth lighting an engine for, m/s.
    /// A booster whose ballistic impact is already on the site skips straight to the
    /// entry attitude rather than burning a metre per second and calling it guidance.
    /// </summary>
    private const double BoostbackMinDvMs = 5.0;

    /// <summary>
    /// Time constant for the low-pass on the TARGET's own turning rate, s.
    ///
    /// The rate is obtained by differencing the target direction between steps, and the
    /// boostback target only moves when a new solve lands - so the raw difference is a
    /// train of spikes at the solve rate with zeros between them. Published as a
    /// feedforward that would be a shudder. The entry target moves continuously and
    /// needs no smoothing, but one filter for both is simpler than two paths.
    /// </summary>
    private const double BoostbackRateTau = 0.5;

    /// <summary>
    /// Burn-duration backstop as a multiple of the burn time predicted at ignition.
    ///
    /// The closed loop has no guarantee of convergence: if the correction grows instead
    /// of shrinking - a bad aero table, a site the vehicle cannot reach, a prediction
    /// that stops hitting the ground - nothing else in the loop ever says stop, and the
    /// booster burns to depletion pointed at whatever the last solve produced. Three
    /// times the initial estimate is far outside anything the real thing needs (the
    /// measured convergence is 21.5 km -> 1.35 km -> 5 m in three re-linearisations)
    /// and comfortably inside "the tanks are empty".
    /// </summary>
    private const double BoostbackBurnLimitFactor = 3.0;

    // ------------------------------------------------------------------ commit

    /// <summary>
    /// EXECUTE on the Boostback tab: start the machine at separation, from whatever
    /// attitude and state the vehicle is in right now.
    ///
    /// The aero sweep is FORCED here rather than left to the staleness check. The
    /// check compares bounding boxes, and a booster that has just separated has a new
    /// one - but this is pressed within seconds of that separation, sometimes on the
    /// frame the part tree is still settling, and a table fitted to the stack that
    /// included the upper stage would give the prediction several times the drag area
    /// the booster actually has.
    /// </summary>
    private static void ExecuteBoostback(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        ResampleBoostbackAero(vehicle, parent);
        if (_s.Aero?.Table == null)
        {
            _s.BoostbackStatus = _s.AeroError.Length > 0
                ? "No aero surrogate: " + _s.AeroError
                : "No aero surrogate - cannot predict an impact point.";
            return;
        }

        // Boostback owns the vehicle now. Ascent and the landing machine both drive the
        // same flight-computer command path, and two of them writing it would fight.
        _s.Running = false;
        _s.LaunchArmed = false;
        _s.LandingPhase = LandingPhase.Idle;

        _s.BoostbackPhase = BoostbackPhase.Separation;
        _s.BoostbackPhaseStart = SimNow();
        _s.BoostbackLastStep = SimNow();
        _s.BoostbackLocked = false;
        _s.BoostbackHasPlan = false;
        _s.BoostbackPlanFrozen = false;
        _s.BoostbackPlanTime = double.NegativeInfinity;
        _s.BoostbackPlanAttemptTime = double.NegativeInfinity;
        _s.BoostbackPlanError = "";
        _s.BoostbackAccumDv = 0.0;
        _s.BoostbackLockDv = 0.0;
        _s.BoostbackDvGo = 0.0;
        _s.BoostbackTgo = double.NaN;
        _s.BoostbackBurnLimit = double.PositiveInfinity;
        _s.BoostbackWantRate = default;
        _s.BoostbackPrevWantValid = false;
        _s.BoostbackThrottle = 0.0;
        _s.BoostbackEngineOn = false;

        // The attitude separation left us in, latched so the hold is a fixed inertial
        // direction rather than a fresh reading of a vehicle that is drifting. It is
        // also what the rotation slews FROM, so the two phases join continuously.
        _s.BoostbackHoldDir = ThrustAxisCci(vehicle);
        _s.CommandDir = _s.BoostbackHoldDir;
        _s.CommandRate = default;
        _s.HasCommand = _s.CommandDir.Length() > 0.5;
        _s.BoostbackStatus = "";
    }

    /// <summary>ABORT: stop steering, cut the engine, leave the vehicle where it is.</summary>
    private static void AbortBoostback()
    {
        _s.BoostbackPhase = BoostbackPhase.Done;
        _s.BoostbackHasPlan = false;
        _s.BoostbackPlanFrozen = false;
        _s.BoostbackLocked = false;
        _s.BoostbackThrottle = 0.0;
        _s.BoostbackEngineOn = false;
        _s.LandingCutPending = true;   // the one-shot engine cut in ApplyAutopilot
        _s.HasCommand = false;
        _s.BoostbackStatus = "Aborted.";
    }

    /// <summary>True while the machine is flying the vehicle.</summary>
    private static bool BoostbackLive =>
        _s.BoostbackPhase != BoostbackPhase.Idle && _s.BoostbackPhase != BoostbackPhase.Done;

    // ------------------------------------------------------------------ the step

    /// <summary>
    /// One step of the machine, run for this vehicle from ApplyAutopilot (the
    /// PrepareWorker prefix) whether or not it is the one on screen.
    ///
    /// THE PREDICTION AND THE CORRECTION ARE DRIVEN FROM HERE, not from the overlay
    /// that also uses them. The overlay runs once per frame for the focused craft only,
    /// so a booster flying itself home unwatched would have had its guidance stop the
    /// moment the camera left it - which is the same trap the ascent and landing flows
    /// were pulled out of. Both are throttled on their own wall-clock ticks, so the two
    /// callers share one answer rather than paying for it twice.
    /// </summary>
    private static void StepBoostback(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        if (!BoostbackLive)
            return;

        double now = SimNow();
        // Sim time, so a warp step is the long interval it really is; clamped because
        // the first step after EXECUTE (and any warp jump) would otherwise make the
        // slew limit and the dV integration meaningless in one direction or the other.
        double dt = Math.Clamp(now - _s.BoostbackLastStep, 0.0, 1.0);
        _s.BoostbackLastStep = now;

        EnsureBoostbackAero(vehicle, parent);
        UpdateImpactPrediction(vehicle, orbit, parent, force: false);
        UpdateSteering(vehicle, orbit, parent, BoostbackSteerIntervalMs);

        double3 r = orbit.StateVectors.PositionCci;
        double altAsl = r.Length() - parent.MeanRadius;

        // --- the impulsive correction, and what it is still for ---------------
        //
        // NEITHER OF THESE IS FLOWN. Both come from the impulsive linearisation, which
        // is wrong about a burn lasting a minute in exactly the way that matters - it
        // points at the ground. The direction and the duration come from the plan
        // below. What survives is the one question the linearisation answers well:
        //
        //   missDv       the targeting correction alone: is there any targeting work
        //                LEFT. That is what starts the burn and what ends it early.
        //                Shaping is deliberately NOT in it - folding it in would hold
        //                the number up after the miss was nulled and the burn would
        //                never stop on its own. See VehicleAutopilotState.SteerShape.
        //   dvMag        the whole impulsive command including shaping. A size estimate
        //                only, and the cold guess the first shot starts from.
        double3 dvVec = _s.HasSteer ? _s.SteerCommand : default;
        double dvMag = dvVec.Length();
        double missDv = _s.HasSteer ? _s.SteerDv.Length() : 0.0;

        if (_s.HasSteer)
            _s.BoostbackDvGo = dvMag;
        // else: HOLD the last figure rather than zeroing it. A solve that fails - a
        // prediction that stops reaching the ground inside the horizon, a staging frame
        // with no mass - means "no news", and reporting no news as "no dV left" would
        // put a zero on the readout mid-burn and make the burn-time estimate agree
        // with it.

        _s.BoostbackTgo = BoostbackBurnTime(vehicle, parent, altAsl, _s.BoostbackDvGo);

        // SENSED dV through the open-loop tail, integrated from the thrust the lit
        // engines are actually producing at this altitude. Not a cutoff - the plan
        // clock is - but it is the one measurement that says whether the engine model
        // the plan was built on matches the engine, which is the first thing that would
        // explain a burn ending off target. Compared against BoostbackLockDv on the tab.
        if (_s.BoostbackPlanFrozen)
        {
            double paNow = KsaEnginePerf.AmbientPressureAt(parent, altAsl);
            double thrustNow = KsaEnginePerf.ActiveThrustCapability(vehicle, paNow);
            double massNow = vehicle.TotalMass;
            if (thrustNow > 0.0 && massNow > 0.0)
                _s.BoostbackAccumDv += thrustNow / massNow * dt;
        }

        // --- the plan ---------------------------------------------------------
        //
        // Only in the two phases that fly it. Solving during separation would spend a
        // solve planning a burn from a state the settling thrust is still changing, and
        // the entry phase has no burn to plan.
        //
        // The freeze is checked BEFORE the re-solve so the last plan is never replaced
        // inside the window it is supposed to be flown out over.
        if (_s.BoostbackPhase == BoostbackPhase.Rotation
            || _s.BoostbackPhase == BoostbackPhase.Boostback)
        {
            if (!_s.BoostbackPlanFrozen
                && _s.BoostbackPhase == BoostbackPhase.Boostback
                && _s.BoostbackHasPlan
                && BoostbackPlanTgo(now) <= BoostbackPlanFreezeS)
            {
                _s.BoostbackPlanFrozen = true;
                _s.BoostbackLocked = true;
                _s.BoostbackLockDv = BoostbackPlanDvGo(vehicle, parent, altAsl,
                                                       BoostbackPlanTgo(now));
                _s.BoostbackAccumDv = 0.0;
            }

            if (!_s.BoostbackPlanFrozen)
                UpdateBoostbackPlan(vehicle, orbit, parent, now, force: false);
        }

        double planTgo = BoostbackPlanTgo(now);
        double3 planDir = BoostbackPlanDirection(now);

        // With a plan in hand it is the plan clock that says how much burn is left, not
        // the rocket equation over an impulsive dV. The two disagree by whatever the
        // impulsive model gets wrong about a finite burn, which is the entire reason
        // the shooter is here.
        if (_s.BoostbackHasPlan)
        {
            _s.BoostbackTgo = planTgo;
            _s.BoostbackDvGo = BoostbackPlanDvGo(vehicle, parent, altAsl, planTgo);
        }

        // --- transitions ------------------------------------------------------
        // Written as a cascade over successive steps, like the ascent's: each one only
        // decides whether to leave the phase it is in, and the command below is built
        // from whatever phase that leaves us in.
        switch (_s.BoostbackPhase)
        {
            case BoostbackPhase.Separation:
                if (now - _s.BoostbackPhaseStart >= Math.Max(_s.BoostbackSeparationS, 0.0))
                    EnterBoostbackPhase(BoostbackPhase.Rotation, now);
                break;

            case BoostbackPhase.Rotation:
            {
                if (!_s.HasSteer)
                {
                    _s.BoostbackStatus = "No steering solution - holding attitude.";
                    break;
                }
                if (missDv < BoostbackMinDvMs)
                {
                    // Already on the site. Nothing to burn, so go straight to the entry
                    // attitude rather than lighting an engine to prove a point. Judged
                    // on the TARGETING correction: shaping is optional, and a burn is
                    // not worth lighting for it alone.
                    _s.BoostbackStatus = $"Correction is only {missDv:F1} m/s - skipping the burn.";
                    EnterBoostbackPhase(BoostbackPhase.EntryOrient, now);
                    break;
                }
                _s.BoostbackStatus = "";

                if (!_s.BoostbackHasPlan)
                {
                    _s.BoostbackStatus = _s.BoostbackPlanError.Length > 0
                        ? "No burn plan: " + _s.BoostbackPlanError
                        : "Solving the burn plan...";
                    break;
                }

                // BOTH the command and the vehicle. The command reaching the target
                // only says the slew has finished issuing; the flight computer is still
                // some way behind it, and lighting the engine there points the thrust
                // at the last of the turn instead of at the target.
                double cmdErr = AngleBetween(_s.CommandDir, planDir) * 180.0 / Math.PI;
                double fcErr = vehicle.FlightComputer.ErrorAngles.Length() * 180.0 / Math.PI;
                if (cmdErr <= BoostbackAlignDeg && fcErr <= BoostbackAlignDeg)
                {
                    EnterBoostbackPhase(BoostbackPhase.Boostback, now);

                    // A FRESH PLAN AT THE INSTANT THE ENGINE LIGHTS, off the interval.
                    // The plan in hand is up to two seconds old and its duration is
                    // measured from when it was solved, so flying it from here would
                    // cut the burn short by however long it has been sitting. Ignition
                    // is precisely the moment worth paying an off-cadence solve for.
                    UpdateBoostbackPlan(vehicle, orbit, parent, now, force: true);
                    planTgo = BoostbackPlanTgo(now);
                    planDir = BoostbackPlanDirection(now);

                    // The backstop, sized off the burn the PLAN says we are about to
                    // fly - which is the burn we are about to fly.
                    _s.BoostbackBurnLimit = now + (double.IsFinite(planTgo)
                        ? Math.Max(planTgo * BoostbackBurnLimitFactor, 30.0)
                        : 120.0);
                }
                break;
            }

            case BoostbackPhase.Boostback:
            {
                // The plan runs out. This is the primary cutoff and it is a CLOCK, not
                // a threshold on a shrinking vector: the shot answered "burn for this
                // long" and the burn is over when it has.
                bool done = _s.BoostbackHasPlan && planTgo <= 0.0;
                bool overrun = now >= _s.BoostbackBurnLimit;

                // Propellant, but not for the first second. The engine has only just
                // been commanded on at this point and the master switch does not reach
                // it until ApplyAutopilot runs at the foot of this same step, so the
                // very frames where the burn begins are also the frames most likely to
                // report nothing lit and nothing fed - and cutting on that would end
                // the burn before it started.
                bool dry = now - _s.BoostbackPhaseStart > 1.0
                        && !vehicle.IsAnyEnginePropellantAvailable();

                // The independent cutoff: the ballistic impact is already ON the site,
                // so whatever the plan clock still says, continuing would take it back
                // off. Targeting only, not the commanded total - shaping would hold
                // this number up after the miss was nulled. Disabled inside the freeze
                // window, where the whole point is that nothing new is listened to.
                bool nulled = !_s.BoostbackPlanFrozen && _s.HasSteer
                           && missDv < BoostbackMinDvMs;

                if (done || nulled || dry || overrun)
                {
                    _s.BoostbackStatus = dry ? "Cutoff - propellant exhausted."
                        : overrun ? "Cutoff - burn exceeded its time limit; check the solution."
                        : $"Cutoff - miss {_s.SteerMissM / 1000.0:F1} km.";
                    EnterBoostbackPhase(BoostbackPhase.EntryOrient, now);
                }
                break;
            }
        }

        // --- where to point, and how fast that point is moving ----------------
        //
        // Both, every step. The flight computer tracks a target's RATE as well as its
        // angle (see KsaAttitudeRate), and handing it only the angle declares a moving
        // target stationary - the difference between tracking and chasing, and the
        // whole reason the rotation and the entry slew read as manoeuvres rather than
        // as a series of settling steps.
        double3 want;
        switch (_s.BoostbackPhase)
        {
            case BoostbackPhase.Separation:
                want = _s.BoostbackHoldDir;
                break;

            case BoostbackPhase.Rotation:
                // Where the optimised burn STARTS - the plan is re-solved through the
                // turn, so this tracks a burn that begins whenever the turn finishes.
                // Not where the impulsive correction points, which on the reference arc
                // is most of sixty degrees away and would leave the vehicle having to
                // turn again as soon as the engine lit.
                want = planDir.Length() > 0.5 ? planDir : _s.BoostbackHoldDir;
                break;

            case BoostbackPhase.Boostback:
                // The steering law, evaluated at the time elapsed since the plan was
                // solved. Frozen or not, this GOES ON TURNING - which is the difference
                // between flying the linear tangent law out and holding the direction
                // it happened to have at the freeze.
                want = planDir.Length() > 0.5 ? planDir : _s.CommandDir;
                break;

            case BoostbackPhase.EntryOrient:
                want = SurfaceRetrogradeCci(orbit, parent);
                break;

            default:
                want = _s.CommandDir;
                break;
        }

        if (want.Length() < 0.5)
        {
            // Nothing usable to point at: hold, rather than commanding a zero vector
            // that CommandAttitude would normalise into whatever the last frame's
            // rounding produced.
            _s.CommandRate = default;
            ApplyBoostbackActuation(vehicle, now);
            return;
        }
        want = double3.Normalize(want);

        double3 wantRate = BoostbackTargetRate(want, dt);

        // The command SLEWS toward the target rather than jumping to it, and the slew
        // is the manoeuvre in the rotation and entry phases rather than a guard on one:
        // BoostbackSlewDegS is the rate the booster actually turns at. When the slew
        // binds, the published rate is the SLEW's, not the target's - a feedforward the
        // command is not following would have the FC drive toward one rate while the
        // target it can see moved at another.
        double maxRad = _s.BoostbackSlewDegS * Math.PI / 180.0 * dt;
        double3 slewed = SlewToward(_s.CommandDir, want, maxRad, out bool clamped);
        if (clamped && dt > 1e-9)
        {
            double3 axis = double3.Cross(_s.CommandDir, slewed);
            wantRate = axis.Length() > 1e-12
                ? double3.Normalize(axis) * (maxRad / dt)
                : default;
        }

        _s.CommandDir = slewed;
        _s.CommandRate = wantRate;
        _s.HasCommand = _s.CommandDir.Length() > 0.5;

        ApplyBoostbackActuation(vehicle, now);
    }

    // ------------------------------------------------------------------ the plan

    /// <summary>
    /// Re-solve the burn plan, throttled to <see cref="BoostbackPlanIntervalS"/>.
    ///
    /// RECEDING HORIZON. Every solve plans a burn that starts NOW, from the live state,
    /// and the vehicle flies the head of the freshest one. So the plan is never flown
    /// to completion while the loop is closed - it is replaced two seconds in, by a
    /// shorter plan built on where the burn has actually got to. That is what absorbs
    /// everything the shot does not model: thrust that differs from the engine table,
    /// the attitude lagging the command, drag that is not quite the surrogate's.
    ///
    /// WARM WHENEVER POSSIBLE. Two seconds on, the answer has barely moved, so the
    /// previous plan seeds the search and the pattern step starts at two degrees rather
    /// than eight. That, plus BurnNodes and the coarse coast, is most of the
    /// difference between the first version of this at ~100 ms and the 2.3 ms one that
    /// ships, and it is why the cold solve at ignition is allowed to be the slow one.
    /// </summary>
    private static void UpdateBoostbackPlan(Vehicle vehicle, Orbit orbit,
                                            IParentBody parent, double now, bool force)
    {
        if (!force && now - _s.BoostbackPlanAttemptTime < BoostbackPlanIntervalS)
            return;
        _s.BoostbackPlanAttemptTime = now;

        KsaAeroSweep.Result aero = _s.Aero;
        double mass = vehicle.TotalMass;
        if (aero?.Table == null || !(mass > 0.0))
        {
            _s.BoostbackPlanError = "no aero surrogate";
            return;
        }

        double altAsl = orbit.StateVectors.PositionCci.Length() - parent.MeanRadius;
        double pa = KsaEnginePerf.AmbientPressureAt(parent, altAsl);
        (double thrust, double massFlow) = KsaEnginePerf.AtPressure(vehicle, pa);
        if (!(thrust > 0.0) || !(massFlow > 0.0))
        {
            _s.BoostbackPlanError = "no engine model";
            return;
        }

        var sys = new PoweredBurnSystem
        {
            Mu = parent.Mu,
            OmegaZ = parent.GetAngularVelocity(),
            MeanRadius = parent.MeanRadius,
            ReferenceArea = aero.ReferenceArea,
            Thrust = thrust,
            MassFlow = massFlow,
            Table = aero.Table,
            Atmosphere = aero.Atmosphere,
        };

        double3 r0 = orbit.StateVectors.PositionCci;
        double3 v0 = orbit.StateVectors.VelocityCci;
        Span<double> x0 = stackalloc double[6] { r0.X, r0.Y, r0.Z, v0.X, v0.Y, v0.Z };

        // The site in the frame the shot reports its impact in - the co-rotating one,
        // which is the current body-fixed frame turned back into CCI axes. Exactly the
        // conversion UpdateSteering makes, for the same reason: the site is body-fixed,
        // and getting this backwards aims the whole burn off by the body's rotation.
        double3 siteCcf = SiteDirCcf() * (parent.MeanRadius + SiteTerrainHeight(parent));
        double3 siteF = siteCcf.Transform(parent.GetCcf2Cci());
        Span<double> target = stackalloc double[3] { siteF.X, siteF.Y, siteF.Z };

        var opt = ImpactOptions.Default(parent.MeanRadius);
        opt.MaxTime = ImpactHorizonMinutes * 60.0;
        opt.PathStride = 0;
        opt.StepAir = BoostbackPlanCoastStepAir;
        opt.StepVacuum = BoostbackPlanCoastStepVac;
        if (_s.ImpactTerrainValid)
            opt.TargetRadius = parent.MeanRadius + _s.ImpactTerrainH;

        // The burn cannot outlast the tanks. Given to the solve as a BOUND rather than
        // checked afterwards, so a site the vehicle cannot reach comes back as "not
        // enough propellant" instead of as a converged plan that runs dry mid-flight.
        double usable = vehicle.PropellantMass;
        double maxBurn = usable > 0.0 ? usable / massFlow : double.PositiveInfinity;

        _s.BoostbackPlanScratch ??= new Dual[BoostbackShooter.ScratchLength];

        // Warm from the last plan; cold, guess a burn as long as the impulsive
        // correction says it needs, which is the only estimate available before any
        // shot has been flown.
        bool warm = _s.BoostbackHasPlan;
        var guess = warm
            ? _s.BoostbackPlan
            : new BurnParameters
            {
                PitchDeg = Math.Max(_s.BoostbackPitchDeg, 10.0),
                Duration = double.IsFinite(_s.BoostbackTgo)
                    ? Math.Clamp(_s.BoostbackTgo, 5.0, 300.0) : 30.0,
            };

        long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
        BoostbackShooter.SolveResult sol = BoostbackShooter.Solve(
            in sys, x0, mass, target, in guess, in opt, _s.BoostbackPlanScratch,
            // The user knob, entering as a hard BOUND on the optimisation rather than
            // as the null-space nudge ShapeFlightPathAngle buys it with. Same knob, but
            // here it constrains the burn that is actually flown - and if the
            // unconstrained optimum already clears it, which it usually does (the
            // measured optimum is +25 deg), it never binds and costs nothing.
            minPitchDeg: _s.BoostbackPitchDeg,
            maxDuration: maxBurn,
            // Rates left at zero. A warm re-solve every two seconds re-datums the whole
            // law against the live state, which is the same correction a turn rate
            // would make and a more honest one; searching them as well would triple the
            // shots per solve to refine something about to be thrown away.
            searchRates: false,
            maxSweeps: warm ? 4 : 10,
            initialPitchStepDeg: warm ? 2.0 : 8.0);
        _s.BoostbackPlanSolveMs =
            (System.Diagnostics.Stopwatch.GetTimestamp() - t0) * 1000.0
            / System.Diagnostics.Stopwatch.Frequency;

        if (!sol.Converged)
        {
            // The PREVIOUS plan is kept and goes on being flown. A burn already under
            // way is better served by a two-second-old plan than by none, the usual
            // failure is the coast momentarily not reaching the ground, and the next
            // attempt is two seconds off. What must not happen is silence, so the
            // reason goes on the tab either way.
            _s.BoostbackPlanError = sol.Shot.OutOfPropellant ? "not enough propellant"
                : sol.Shot.FlewIntoGround ? "the burn flies into the ground"
                : sol.Shot.Infeasible ? "no burn reaches the site"
                : "the plan did not converge";
            return;
        }

        _s.BoostbackPlanError = "";
        _s.BoostbackPlan = sol.Parameters;
        _s.BoostbackPlanTime = now;
        _s.BoostbackPlanFrame = BoostbackShooter.Frame.FromState(x0);
        _s.BoostbackPlanMissM = sol.MissM;
        _s.BoostbackPlanPropellantKg = sol.PropellantKg;
        _s.BoostbackHasPlan = true;
    }

    /// <summary>
    /// Where the plan says to point right now: the steering law evaluated at the time
    /// elapsed since the solve. Zero when there is no plan.
    /// </summary>
    private static double3 BoostbackPlanDirection(double now)
    {
        if (!_s.BoostbackHasPlan)
            return default;
        BoostbackShooter.SteeringAt(in _s.BoostbackPlanFrame, in _s.BoostbackPlan,
                                    Math.Max(now - _s.BoostbackPlanTime, 0.0),
                                    out double dx, out double dy, out double dz);
        return new double3(dx, dy, dz);
    }

    /// <summary>Seconds of burn the plan has left; NaN when there is no plan.</summary>
    private static double BoostbackPlanTgo(double now)
        => _s.BoostbackHasPlan
            ? Math.Max(_s.BoostbackPlan.Duration - (now - _s.BoostbackPlanTime), 0.0)
            : double.NaN;

    /// <summary>
    /// The dV the plan has left to impart, m/s - the rocket equation over its remaining
    /// burn time, at the current ambient thrust.
    ///
    /// Derived from the plan rather than from the impulsive correction, because with
    /// the shot flying the burn it is the plan duration that decides when the engine
    /// stops. A readout, not a cutoff: nothing is tested against it.
    /// </summary>
    private static double BoostbackPlanDvGo(Vehicle vehicle, IParentBody parent,
                                            double altAsl, double tgo)
    {
        if (!(tgo > 0.0))
            return 0.0;

        double pa = KsaEnginePerf.AmbientPressureAt(parent, altAsl);
        (double thrust, double massFlow) = KsaEnginePerf.AtPressure(vehicle, pa);
        double mass = vehicle.TotalMass;
        if (!(thrust > 0.0) || !(massFlow > 0.0) || !(mass > 0.0))
            return double.NaN;

        double burned = Math.Min(massFlow * tgo, mass * 0.99);
        return thrust / massFlow * Math.Log(mass / (mass - burned));
    }

    /// <summary>
    /// What the engine should be doing in the current phase. Recorded on the state and
    /// applied in ApplyAutopilot, which is where writes to _manualControlInputs reach
    /// the sim - the same split every other mode here uses.
    /// </summary>
    private static void ApplyBoostbackActuation(Vehicle vehicle, double now)
    {
        switch (_s.BoostbackPhase)
        {
            case BoostbackPhase.Separation:
                _s.BoostbackThrottle = SettleThrottle(vehicle);
                _s.BoostbackEngineOn = true;
                break;

            case BoostbackPhase.Rotation:
                // LIT THROUGH THE TURN, AND THAT IS THE POINT. A booster's RCS is
                // sized for attitude hold, not for throwing a half-empty first stage
                // through 180 degrees; the gimbals are what turn it, and a gimbal
                // produces torque in proportion to the thrust it is deflecting. Shut
                // the engine down for the flip and the only authority left is the one
                // that cannot do it.
                //
                // At the same minimum throttle the settling burn uses, which is the
                // trade: gimbal torque scales with it, and so does the dV the sweeping
                // thrust axis lays down. That dV is NOT a targeting error - the
                // correction is re-derived from the live state at 10 Hz and the burn
                // does not start until the turn is finished, so what the flip does to
                // the trajectory is simply part of the state the first solve sees. It
                // costs propellant, not accuracy.
                _s.BoostbackThrottle = SettleThrottle(vehicle);
                _s.BoostbackEngineOn = true;
                break;

            case BoostbackPhase.Boostback:
                _s.BoostbackThrottle = 1.0;
                _s.BoostbackEngineOn = true;
                break;

            default:
                _s.BoostbackThrottle = 0.0;
                _s.BoostbackEngineOn = false;
                break;
        }
    }

    private static void EnterBoostbackPhase(BoostbackPhase phase, double now)
    {
        _s.BoostbackPhase = phase;
        _s.BoostbackPhaseStart = now;
        // The target changes discontinuously across a phase boundary (the dV direction
        // to surface retrograde is most of a half turn), so the differencing filter has
        // to be re-seeded or the first step of the new phase publishes that whole jump
        // as a turning rate.
        _s.BoostbackPrevWantValid = false;
        _s.BoostbackWantRate = default;

        // The plan belongs to the burn. Past it there is nothing to fly and nothing to
        // warm-start from, and leaving it set would put a stale burn on the tab beside
        // a vehicle that is coasting.
        if (phase != BoostbackPhase.Rotation && phase != BoostbackPhase.Boostback)
        {
            _s.BoostbackHasPlan = false;
            _s.BoostbackPlanFrozen = false;
            _s.BoostbackLocked = false;
            _s.BoostbackPlanTime = double.NegativeInfinity;
            _s.BoostbackPlanAttemptTime = double.NegativeInfinity;
        }
    }

    // ------------------------------------------------------------------ helpers

    /// <summary>Token throttle for a vehicle whose minimum is not usable. See
    /// <see cref="SettleThrottle"/>.</summary>
    private const double BoostbackSettleFallback = 0.05;

    /// <summary>
    /// The throttle the settling burn commands: the lowest this vehicle will honour,
    /// from KSA's own aggregate (PartTree.EngineThrottleMin, the minimum of
    /// MinimumThrottle over every engine).
    ///
    /// DELIBERATELY NOT Ksa6DofSetup.VehicleThrottleFloor, which reports 1.0 for
    /// anything outside (0, 1] — including an engine whose MinimumThrottle is zero.
    /// That is the right conservative direction for a descent solver sizing its
    /// authority, and exactly the wrong one here, where it would read "settle the
    /// propellant at full thrust" and lay down a large unplanned dV two seconds before
    /// the correction is solved for. Erring small costs nothing in the other
    /// direction: a vehicle that genuinely cannot throttle clamps this up to its own
    /// minimum anyway.
    /// </summary>
    private static double SettleThrottle(Vehicle vehicle)
    {
        double floor = vehicle?.Parts?.EngineThrottleMin ?? 0.0;
        return floor > 0.0 && floor < 1.0 ? floor : BoostbackSettleFallback;
    }

    /// <summary>
    /// The angular velocity of the TARGET direction, rad/s in CCI, low-passed.
    ///
    /// For a unit direction u, the rotation that carries it is omega = u x u_dot, and
    /// u_dot is differenced between steps. The filter is not cosmetic: the boostback
    /// target only moves when a solve lands, so the raw difference is a spike train at
    /// the solve rate. See BoostbackRateTau.
    /// </summary>
    private static double3 BoostbackTargetRate(double3 want, double dt)
    {
        double3 raw = default;
        if (_s.BoostbackPrevWantValid && dt > 1e-6)
        {
            double3 udot = (want - _s.BoostbackPrevWant) / dt;
            raw = double3.Cross(want, udot);
        }
        _s.BoostbackPrevWant = want;
        _s.BoostbackPrevWantValid = true;

        double a = Blend(BoostbackRateTau, dt);
        _s.BoostbackWantRate += (raw - _s.BoostbackWantRate) * a;
        return _s.BoostbackWantRate;
    }

    /// <summary>
    /// Burn time for a given dV at full throttle, from the rocket equation:
    /// t = (m / mdot) * (1 - exp(-dv / ve)), with ve = thrust / mdot.
    ///
    /// AT THE CURRENT AMBIENT PRESSURE, not in vacuum. A booster's sea-level thrust is
    /// typically ~80% of its vacuum figure, so a vacuum ve and mdot overstate how fast
    /// the dV is flown off - which would trip the steering lock early and cut the burn
    /// short by the same ratio. NaN when the vehicle has nothing to burn with, which
    /// the caller treats as "no estimate" rather than as zero.
    /// </summary>
    private static double BoostbackBurnTime(Vehicle vehicle, IParentBody parent,
                                            double altAsl, double dv)
    {
        if (!(dv > 0.0))
            return 0.0;

        double pa = KsaEnginePerf.AmbientPressureAt(parent, altAsl);
        (double thrust, double massFlow) = KsaEnginePerf.AtPressure(vehicle, pa);
        double mass = vehicle.TotalMass;
        if (!(thrust > 0.0) || !(massFlow > 0.0) || !(mass > 0.0))
            return double.NaN;

        double ve = thrust / massFlow;
        return mass * (1.0 - Math.Exp(-dv / ve)) / massFlow;
    }

    /// <summary>
    /// Surface retrograde in CCI: the direction the THRUST AXIS points to fly the
    /// vehicle engine-first into the relative wind.
    ///
    /// Surface-relative, not inertial, because KSA's atmosphere co-rotates rigidly with
    /// the body - the game subtracts omega x r itself before computing drag, so an
    /// inertial retrograde is wrong by a full equatorial rotation speed near the ground
    /// and by the cosine of that higher up.
    ///
    /// The sign is the one that puts the vehicle at alpha = 0 in the surrogate's
    /// retrograde-first convention: KSA's body +x is the nose and thrust acts along it
    /// (BurnTarget.ComputeBurnBody2Cci points body x at the steering vector), so
    /// commanding -v_air puts the nose aft, the engine into the airflow, and any burn
    /// from that attitude decelerates.
    /// </summary>
    private static double3 SurfaceRetrogradeCci(Orbit orbit, IParentBody parent)
    {
        double3 r = orbit.StateVectors.PositionCci;
        double3 vAir = orbit.StateVectors.VelocityCci
                     - double3.Cross(parent.GetAngularVelocityCci(), r);
        return -vAir.NormalizeOrZero();
    }

    /// <summary>The vehicle's thrust axis in CCI - KSA's body +x.</summary>
    private static double3 ThrustAxisCci(Vehicle vehicle)
        => new double3(1, 0, 0).Transform(KsaFrameBridge.BodyToCci(vehicle));

    /// <summary>
    /// Make sure this vehicle has an aero surrogate, resampling only when its bounding
    /// box says the old one describes a different stack.
    ///
    /// Shared by the tab (which needs it to draw) and the guidance step (which needs it
    /// to predict), so a boostback flown with the panel shut is fitted to the same
    /// table a watched one is. KSA only recomputes AerodynamicCdABody on a part-tree
    /// modification, so the box is exactly as stale as the table is.
    /// </summary>
    private static void EnsureBoostbackAero(Vehicle vehicle, IParentBody parent)
    {
        if (_s.AeroStale(LiveBoxExtents(vehicle)))
            ResampleBoostbackAero(vehicle, parent);
    }

    internal static string BoostbackPhaseName(BoostbackPhase p) => p switch
    {
        BoostbackPhase.Idle => "idle",
        BoostbackPhase.Separation => "separation (settling)",
        BoostbackPhase.Rotation => "rotating to burn attitude",
        BoostbackPhase.Boostback => "boostback burn",
        BoostbackPhase.EntryOrient => "entry attitude (surface retrograde)",
        BoostbackPhase.Done => "ended",
        _ => "?",
    };
}

#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using AdvancedFlightComputer.Guidance.Conic;
using AdvancedFlightComputer.Guidance.Gfold;
using KSA;

// Convex (G-FOLD) powered descent from the high gate to the surface, flown as committed-trajectory tracking: solve a min-fuel plan, fly it by time index (feed-forward the planned thrust + PD feedback on the planned state), re-solve on a cadence.
public static partial class GuidanceWindow
{
    // Orbital landings remove horizontal speed at an elevated gate before the solver targets the surface.
    internal enum GfoldApproach { Direct, BrakeAtGate, VerticalFinal }

    // The surface descent may start early inside this band above the gate, once the craft is slow, near the site and upright.
    private const double VerticalGateBandM = 100;
    private const double VerticalGateSpeedMs = 5;
    private const double VerticalGateMinDistanceM = 100;
    private const double VerticalGateDistanceFraction = 0.2;
    private const double VerticalFinalPointingDeg = 15;
    private const double VerticalFinalGlideSlopeDeg = 30;

    private static bool GfoldBrakingToGate => _s.GfoldApproach == GfoldApproach.BrakeAtGate;
    private static bool GfoldVerticalFinal => _s.GfoldApproach == GfoldApproach.VerticalFinal;
    private static double GfoldSolverTargetAltM => _s.VehicleHeightM + (GfoldBrakingToGate ? _s.LandingVerticalGateM : 0);
    // The gate is passed sinking at the gate descent rate, because a craft whose minimum thrust exceeds its weight cannot stop and hold there.
    private static double GfoldSolverArrivalRateMs => GfoldBrakingToGate ? _s.DescentRate : 0;
    private static double GfoldActivePointingDeg => GfoldVerticalFinal
        ? Math.Min(VerticalFinalPointingDeg, _s.GfoldPointingDeg) : _s.GfoldPointingDeg;
    private static double GfoldActiveGlideSlopeDeg => GfoldVerticalFinal
        ? Math.Max(VerticalFinalGlideSlopeDeg, _s.GfoldGlideSlopeDeg) : _s.GfoldGlideSlopeDeg;
    private static bool GfoldReadyForHover => !GfoldBrakingToGate
        && _s.GfoldAltM <= _s.GfoldHoverHandoffAltM && !_s.GfoldHoverRefused;

    private static bool AtVerticalApproachGate(double altitude, double horizontalSpeed, double distance) =>
        GfoldBrakingToGate && double.IsFinite(altitude + horizontalSpeed + distance)
        && altitude <= _s.LandingVerticalGateM + VerticalGateBandM && horizontalSpeed <= VerticalGateSpeedMs
        && distance <= Math.Max(VerticalGateMinDistanceM, _s.LandingVerticalGateM * VerticalGateDistanceFraction);

    private static bool TryBeginVerticalDescent(double altitude, double horizontalSpeed, double distance, double tilt, double now)
    {
        if (!AtVerticalApproachGate(altitude, horizontalSpeed, distance)
            || !double.IsFinite(tilt) || tilt > VerticalFinalPointingDeg) return false;
        ChangeGfoldApproach(GfoldApproach.VerticalFinal, now);
        _s.LandingStatus = "Horizontal braking complete. Descending with the engines down.";
        return true;
    }

    // A craft that passes below the gate outside its limits, or outlives the plan to it, cannot reach the gate any more.
    // It lands on the site directly, which also allows the hover handoff again.
    private static bool TryAbandonVerticalGate(double altitude, double now)
    {
        bool missed = altitude <= _s.LandingVerticalGateM || (_s.GfoldPlan != null && now > _s.GfoldArrivalTime);
        if (!GfoldBrakingToGate || !missed) return false;
        ChangeGfoldApproach(GfoldApproach.Direct, now);
        _s.WarnLanding("The vertical approach gate was missed. G-FOLD lands on the site directly.");
        return true;
    }

    // A new target makes the previous flight time a wrong guess, so the next solve searches the full range.
    private static void ChangeGfoldApproach(GfoldApproach approach, double now)
    {
        _s.GfoldApproach = approach;
        _s.GfoldForceSearch = true;
        _s.GfoldArrivalTime = now + SearchTfMax;
        _s.GfoldLastSolveTime = double.NegativeInfinity;
        _s.GfoldSearchRetryTime = double.NegativeInfinity;
        _s.GfoldTrackInit = false;
    }

    // The length across the X-up site frame's horizontal plane.
    private static double HorizontalLength(double3 local) => Math.Sqrt(local.Y * local.Y + local.Z * local.Z);

    // Keep the checked plan through its final seconds. A missed arrival needs a fresh solve.
    private static bool GfoldInTerminalWindow(double now) => _s.GfoldPlan != null
        && _s.GfoldArrivalTime >= now && _s.GfoldArrivalTime - now <= GfoldMinTf;

    private const double GfoldMinTf = 4.0;

    // A planned coast is flown as a real coast. The engine cannot deliver less than its minimum throttle, so a demand under this fraction of full thrust switches it off, and it lights again only above the re-light fraction, so a command that sits near the threshold does not toggle the engine every step.
    private const double GfoldCoastFraction = 0.02;
    private const double GfoldRelightFraction = 0.06;

    // Ignition after a coast waits until the thrust axis points along the command, because an engine lit during the slew throws its thrust sideways. The wait is bounded, because a craft that cannot align in time is still better off braking.
    private const double GfoldIgnitionAlignDeg = 15.0;
    private const double GfoldIgnitionAlignLimitS = 3.0;

    // Flight-time search bounds.
    // SearchTfMax is the cold-start ceiling; the two bracket factors are the window searched around the previous solution's remaining time when one is available (see SolveGfoldPlan).
    // Wide enough to absorb a plan that has drifted, narrow enough to cut most of the coarse scan.
    private const double SearchTfMax = 120.0;
    private const double SearchBracketLo = 0.5;
    private const double SearchBracketHi = 2.0;

    // How long the flight-time searches rest after a plan was refused for propellant. A craft that is short now is still short a quarter second later, and a search costs tens of solves on the sim thread.
    private const double GfoldFuelSearchRetryS = 2.0;

    // Shown while the plan flies to touchdown after a refused handoff, and kept through the re-solves that clear the status on success. The reason is in the guidance log.
    private const string GfoldHoverRefusedStatus = "The hover cannot take over, so G-FOLD flies its plan to the ground.";

    // Committed-trajectory tracking: the solved descent plan is flown by time index (feed-forward the planned thrust at the current time + light PD feedback on the reference state) and re-solved on a cadence, rather than applying node 0.
    // This follows the plan's coast (throttle down) and brake arcs instead of freezing the first node.
    // Command smoothing: the throttle and thrust direction are low-pass filtered toward the freshly-computed command with this time constant, so per-frame feedback noise and re-solve steps don't reach the engine/gimbal as chatter.

    private static void StepGfoldDescent(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                         double bodyRadius, double now)
    {
        if (!PrepareLandingEngines(vehicle, parent, now, requireAirless: true))
            return;

        double3 siteCci = SiteDirCciAt(parent, 0) * (bodyRadius + SiteTerrainHeight(parent));
        var frame = KsaGfold.BuildFrame(siteCci);
        double3 v = orbit.StateVectors.VelocityCci;

        // The flown state is the CoM - the vehicle-height allowance lives in the solver TARGET (see GfoldSolverTargetAltM), so attitude changes don't perturb the reference state. _s.GfoldAltM is height above touchdown: zero when the CoM sits _s.VehicleHeightM over the pad, i.e. legs on the ground.
        double3 r = orbit.StateVectors.PositionCci;
        double3 vSrf = v - double3.Cross(parent.GetAngularVelocityCci(), r);
        _s.GfoldAltM = double3.Dot(r - siteCci, frame.Ex) - _s.VehicleHeightM;
        _s.GfoldSpeedMs = vSrf.Length();
        double3 gfLocal = frame.PointToLocal(r);   // X-up frame
        double horizontalDistance = HorizontalLength(gfLocal);
        RecordGfoldTrace(horizontalDistance, gfLocal.X);
        double3 localVelocity = frame.VecToLocal(vSrf);
        double horizontalSpeed = HorizontalLength(localVelocity);
        double tilt = AngleBetween(KsaFrameBridge.ThrustAxisCci(vehicle), frame.Ex) * 180 / Math.PI;
        if (TryStartTerminalBurn(vehicle, _s.GfoldAltM, horizontalSpeed, horizontalDistance, now))
        {
            StepTerminalBurn(vehicle, orbit, parent, now);
            return;
        }
        if (TryBeginVerticalDescent(_s.GfoldAltM, horizontalSpeed, horizontalDistance, tilt, now))
            GuidanceLog.Info(vehicle, $"vertical descent starts: {ApproachState()}.");
        else if (TryAbandonVerticalGate(_s.GfoldAltM, now))
            GuidanceLog.Info(vehicle, $"vertical approach gate missed: {ApproachState()}.");
        if (GuidanceLog.Enabled && now - _s.LastGuidanceLogTime >= GuidanceLogIntervalS)
        {
            _s.LastGuidanceLogTime = now;
            GuidanceLog.Debug(vehicle, $"G-FOLD approach: {ApproachState()}"
                + $", target height {GfoldSolverTargetAltM - _s.VehicleHeightM:F1} m, approach {_s.GfoldApproach}.");
        }

        // Hand off to the terminal hover controller for the last stretch: G-FOLD brings the vehicle down to the handoff height (slow and near-vertical), and the hover flies the final touchdown.
        // A hover that refuses the craft is asked once, and the plan, which ends at the surface anyway, is flown to the touchdown StepLanding detects.
        if (GfoldReadyForHover)
        {
            StartTerminalHover(vehicle);
            if (_s.LandingPhase == LandingPhase.TerminalHover)
            {
                _s.LandingStatus = $"The hover takes over from G-FOLD at {_s.GfoldAltM:F0} m.";
                return;
            }
            _s.GfoldHoverRefused = true;
            GuidanceLog.Info(vehicle, $"G-FOLD flies its plan to touchdown from {_s.GfoldAltM:F0} m: {_s.LandingStatus}");
            _s.WarnLanding(GfoldHoverRefusedStatus);
        }

        // Inside the last GfoldMinTf seconds before the planned arrival, the distance still to fly is too small for any valid flight time: tf >= TfMin (the search floor) overshoots it, so a re-solve goes degenerate and reports the target unreachable right before the handoff.
        // Freeze the committed plan there - it already terminates at the target - and just fly it down.
        if (!GfoldInTerminalWindow(now) &&
            (_s.GfoldPlan == null || now - _s.GfoldLastSolveTime >= _s.GfoldIntervalS))
            SolveGfoldPlan(vehicle, parent, frame, siteCci, r, now);

        // Fly the committed plan by time index every frame (feed-forward + PD).
        // Track in the LIVE site frame (rebuilt this step), not the solve-time frame: the site is body-fixed, so its CCI position rotates with the body, and the live frame carries the plan around with it so we keep aiming at the real pad.
        if (_s.GfoldPlan != null)
            TrackGfoldPlan(vehicle, parent, frame, r, vSrf, vehicle.TotalMass, now);

        string ApproachState() => $"altitude {_s.GfoldAltM:F1} m, horizontal speed {horizontalSpeed:F2} m/s"
            + $", sink {-localVelocity.X:F2} m/s, distance {horizontalDistance:F1} m, thrust-axis tilt {tilt:F1} deg";
    }

    // Solve a fresh descent plan from the current state and commit it.
    // A min-fuel search (so it coasts/brakes optimally - "throttles down") to the site; if the site is unreachable in the remaining time the search floats the touchdown to the closest point, so this degrades gracefully instead of going infeasible.
    private static void SolveGfoldPlan(Vehicle vehicle, IParentBody parent,
                                       KsaGfold.Frame frame, double3 siteCci, double3 comPos, double now)
    {
#if DEBUG
        using var _perf = new AdvancedFlightComputer.Core.PerfTracker.Scope("Guidance.SolveGfoldPlan");
#endif
        GfoldParams p = KsaGfold.BuildParams(
            vehicle, parent, frame, siteCci, comPos, GfoldActiveGlideSlopeDeg, GfoldActivePointingDeg, _s.GfoldVMaxMs,
            GfoldSolverTargetAltM, GfoldSolverArrivalRateMs, _s.GfoldThrottleMin, _s.GfoldThrottleMax,
            EffectiveAccelLimitG(vehicle) * StandardGravity, out string refusal);
        if (p == null)
        {
            EndLandingBadly(vehicle, "G-FOLD cannot plan for this craft", refusal);
            _s.GfoldLastSolveTime = now;
            _s.GfoldPlan = null;
            _s.GfoldThrottle = 0.0;
            _s.HasCommand = false;
            _s.LandingPhase = LandingPhase.Done;
            _s.LandingCutPending = true;
            return;
        }

        // Mark the attempt now (not only on success) so a failing solve retries on the normal cadence rather than every frame while we keep flying the last plan.
        _s.GfoldLastSolveTime = now;

        // GfoldPlanner.SolveTimeLimitS is process-wide, which is safe ONLY because the solve below is synchronous and on this thread: each vehicle sets it immediately before its own call and the call has returned before any other vehicle is serviced.
        // Move the solve to a worker and this becomes a race - see GfoldSolveMs for why that move is worth making anyway.
        GfoldPlanner.SolveTimeLimitS = _s.GfoldSolveTimeLimitS > 0 ? _s.GfoldSolveTimeLimitS : null;
        var solveClock = System.Diagnostics.Stopwatch.StartNew();

        try
        {
            // On the cadence, prefer to keep the arrival time set at handoff (solve the remaining time); only re-run the full search when that fails or we have no plan yet.
            GfoldTrajectory traj = null;
            if (!_s.GfoldForceSearch && _s.GfoldPlan != null && _s.GfoldArrivalTime - now > GfoldMinTf)
            {
                double remaining = _s.GfoldArrivalTime - now;
                GfoldTrajectory t = GfoldPlanner.SolveMinFuel(
                    p, remaining, _s.GfoldNodes, [GfoldSolverTargetAltM, 0.0, 0.0],
                    options: GfoldOptions.Descent with { SlewReg = _s.GfoldSlewReg });
                if (t.Status is ConicStatus.Optimal or ConicStatus.OptimalInaccurate && FitsFuel(t, p))
                    traj = t;
            }
            // While the searches rest after a propellant refusal, the committed plan keeps flying and the single solve above still runs on every cadence, so a plan that fits again is taken at once. A retarget ends the rest, so its first search runs straight away, and a refused retarget rests like any other search. A craft with no plan yet always searches.
            if (traj == null && _s.GfoldPlan != null && now < _s.GfoldSearchRetryTime)
            {
                FailGfold(vehicle, FormattableString.Invariant($"G-FOLD is short of propellant, searching again in {_s.GfoldSearchRetryTime - now:F1} s"));
                return;
            }
            if (traj == null)
            {
                // Warm-start the SEARCH from the last solution.
                // The solver itself can't be warm started - ECOS is an interior-point method and the previous optimum sits on the boundary, the worst possible starting iterate - but the flight-time search around it can be.
                //
                // The committed plan's remaining time IS the previous solution carried forward: _s.GfoldArrivalTime was set to now + tf* when that search ran, so (_s.GfoldArrivalTime - now) is tf* minus the time since.
                // That's the best available estimate of the new optimum, so bracket it instead of rescanning the full range.
                // Searching [4, 120] from cold spends 8 coarse points plus ~10 golden-section steps, at up to two SOCP solves each - 35-40 solves.
                // A tight bracket collapses the coarse scan onto the plausible range.
                //
                // Both bounds move, not just the upper one: with a 20 s remaining flight the old floor of 4 s was as wasteful as the old 120 s ceiling, just at the other end. _s.GfoldForceSearch means the previous solution is no longer a valid guess - a retarget replaces _s.GfoldArrivalTime with a placeholder far in the future purely to escape the terminal freeze, so bracketing around it would spend a narrow search on a fabricated centre and then fall back anyway.
                // Go straight to the full range.
                double tfLo = GfoldMinTf, tfHi = SearchTfMax;
                bool bracketed = !_s.GfoldForceSearch
                    && _s.GfoldPlan != null && _s.GfoldArrivalTime - now > GfoldMinTf;
                if (bracketed)
                {
                    double expected = _s.GfoldArrivalTime - now;
                    tfLo = Math.Max(expected * SearchBracketLo, GfoldMinTf);
                    tfHi = Math.Min(expected * SearchBracketHi, SearchTfMax);
                    bracketed = tfHi > tfLo;
                }

                GfoldPlanner.SearchResult best = bracketed
                    ? GfoldPlanner.SearchMinFuel(
                        p, _s.GfoldNodes, tfLo: tfLo, tfHi: tfHi,
                        options: GfoldOptions.Descent with { SlewReg = _s.GfoldSlewReg })
                    : null;

                // A bracket can only lose solutions that lie outside it, so the full range is still tried before declaring the site unreachable.
                // Costs the old price only when the cheap window genuinely found nothing - which is also exactly when the vehicle's situation has changed enough that the previous solution was a bad guess.
                // A plan the tanks cannot supply is no better than none, so the full range is also searched for a cheaper one.
                // The bracket's plan is kept when that finds nothing, so the refusal below can say how much propellant it was short.
                if (best == null || !FitsFuel(best.Trajectory, p))
                    best = GfoldPlanner.SearchMinFuel(
                        p, _s.GfoldNodes, tfLo: GfoldMinTf, tfHi: SearchTfMax,
                        options: GfoldOptions.Descent with { SlewReg = _s.GfoldSlewReg }) ?? best;

                if (best == null)
                {
                    FailGfold(vehicle, FormattableString.Invariant($"G-FOLD unreachable: alt {_s.GfoldAltM:F0} m, {_s.GfoldSpeedMs:F0} m/s, ") +
                              FormattableString.Invariant($"TWR {p.ThrustMax / (vehicle.TotalMass * p.GravityMag):F1}, fuel {p.FuelMass:F0} kg")
                              + (double.IsFinite(p.AccelMax) ? FormattableString.Invariant($", thrust acceleration limit {p.AccelMax / p.GravityMag:F1} local g") : ""));
                    return;
                }
                if (!FitsFuel(best.Trajectory, p))
                {
                    _s.GfoldSearchRetryTime = now + GfoldFuelSearchRetryS;
                    FailGfold(vehicle, FormattableString.Invariant($"G-FOLD needs {best.FuelUsed:F0} kg of propellant, {p.FuelMass:F0} kg aboard"));
                    return;
                }
                traj = best.Trajectory;
                _s.GfoldArrivalTime = now + best.TimeOfFlight;
            }

            _s.GfoldSolveMs = solveClock.Elapsed.TotalMilliseconds;
            _s.GfoldStatus = traj.Status;
            _s.GfoldPlan = traj;
            _s.GfoldPlanStart = now;
            _s.GfoldThrustMax = p.ThrustMax;
            _s.GfoldFailStreak = 0;
            _s.GfoldForceSearch = false;
            _s.GfoldSearchRetryTime = double.NegativeInfinity;
            _s.LandingStatus = _s.GfoldHoverRefused ? GfoldHoverRefusedStatus
                : GfoldBrakingToGate ? $"Braking to the vertical approach at {_s.LandingVerticalGateM:F0} m."
                : GfoldVerticalFinal ? "Vertical final descent." : "";
            if (_s.GfoldHoverRefused)
                _s.WarnLanding(GfoldHoverRefusedStatus);
        }
        catch (Exception e)
        {
            _s.GfoldSolveMs = solveClock.Elapsed.TotalMilliseconds;
            FailGfold(vehicle, "G-FOLD: " + e.Message);
        }
    }

    // Fly the committed plan: feed-forward the planned thrust at the current time plus PD feedback on the planned state, expressed in the given (live) site frame so the plan stays locked to the body-fixed, rotating landing pad.
    private static void TrackGfoldPlan(Vehicle vehicle, IParentBody parent, KsaGfold.Frame f,
                                       double3 r, double3 vSrf, double mass, double now)
    {
#if DEBUG
        using var _perf = new AdvancedFlightComputer.Core.PerfTracker.Scope("Guidance.TrackGfoldPlan");
#endif
        GfoldTrajectory plan = _s.GfoldPlan;
        int n = plan.Nodes;
        double elapsed = now - _s.GfoldPlanStart;

        // Reference STATE at the current time: interpolate from node 0.
        // The plan starts at the current state, so tracking error is ~0 just after a solve; reading it one node ahead (as the feed-forward does) would feed a whole step of expected motion ~v*dt back as phantom error and tilt the command past the pointing cone.
        double sf = Math.Clamp(elapsed / plan.Dt, 0.0, n - 1);
        int s0 = Math.Clamp((int)Math.Floor(sf), 0, n - 2);
        double sfrac = Math.Clamp(sf - s0, 0.0, 1.0);
        double3 refPos = Lerp(Node(plan.Position, s0), Node(plan.Position, s0 + 1), sfrac);
        double3 refVel = Lerp(Node(plan.Velocity, s0), Node(plan.Velocity, s0 + 1), sfrac);

        // Feed-forward THRUST: skip node 0 (its control is the unconstrained artifact), so the first step uses node 1's real, cone-respecting thrust.
        double tf = Math.Max(elapsed / plan.Dt, 1.0);
        int t0 = Math.Clamp((int)Math.Floor(tf), 1, n - 2);
        double tfrac = Math.Clamp(tf - t0, 0.0, 1.0);
        double3 ff = Lerp(Node(plan.AccelCmd, t0), Node(plan.AccelCmd, t0 + 1), tfrac);
        // The final burn sample is valid only until arrival. A failed re-plan must not repeat it indefinitely.
        if (elapsed >= plan.TimeOfFlight)
            ff = new double3(parent.Mu / r.LengthSquared(), 0, 0);

        double3 curPos = f.PointToLocal(r);
        double3 curVel = f.VecToLocal(vSrf);
        double3 fb = _s.GfoldKp * (refPos - curPos) + _s.GfoldKd * (refVel - curVel);
        double3 cmd = ff + fb; // local thrust acceleration

        double pressure = KsaEnginePerf.AmbientPressureAt(parent, r.Length() - parent.MeanRadius);
        double demand = cmd.Length() * mass;

        // The plan may coast, and the engine cannot follow a demand under its minimum throttle, so the coast is flown with the engine off. The decision is made on the unsmoothed demand as a fraction of full thrust with hysteresis, because a demand between the minimum and zero would otherwise be clamped up to the minimum and the craft would brake far harder than the plan. The smoothed demand cannot decide it, because the smoothing restarts from an engine that is off.
        double fullThrust = KsaEnginePerf.ThrustAtThrottle(vehicle, 1.0, pressure);
        double fraction = fullThrust > 0.0 && double.IsFinite(fullThrust) ? demand / fullThrust : 0.0;
        bool wantsBurn = fraction > GfoldRelightFraction;
        if (fraction < GfoldCoastFraction)
            _s.GfoldEngineOn = false;

        // Direction. While the engine burns, the command is the plan's feed-forward plus the tracking feedback, clamped to the pointing cone of local up, because the feedback can tilt past the cone the plan respects. While the engine is off the feedback has nothing to act on and its direction is noise, so the craft is aligned in advance to the first burn the plan still holds, and to local up when the plan holds none.
        double3 dirLocal = ClampToCone(_s.GfoldEngineOn ? cmd : NextBurnDirection(plan, t0, mass, fullThrust), GfoldActivePointingDeg);
        double3 targetDir = double3.Normalize(f.VecToCci(dirLocal));
        if (!double.IsFinite(targetDir.X) || !double.IsFinite(targetDir.Y) || !double.IsFinite(targetDir.Z))
            targetDir = _s.CommandDir.Length() > 0.5 ? _s.CommandDir : f.Ex;

        // First-order low-pass toward the fresh command, so feedback noise and re-solve steps don't reach the engine/gimbal as chatter.
        double dt = Math.Clamp(now - _s.GfoldLastTrackTime, 0.0, 0.25);
        _s.GfoldLastTrackTime = now;
        double a = (!_s.GfoldTrackInit || _s.GfoldSmoothTau <= 1e-3)
            ? 1.0
            : 1.0 - Math.Exp(-dt / _s.GfoldSmoothTau);
        _s.GfoldTrackInit = true;

        double3 blended = _s.CommandDir.Length() > 0.5 ? _s.CommandDir + a * (targetDir - _s.CommandDir) : targetDir;
        if (blended.Length() > 1e-6)
            _s.CommandDir = double3.Normalize(blended);

        // Ignition after a coast waits for the attitude, because the flight computer lags the command and an engine lit during the slew throws its thrust sideways.
        if (wantsBurn && !_s.GfoldEngineOn)
        {
            if (double.IsNaN(_s.GfoldRelightRequestTime))
                _s.GfoldRelightRequestTime = now;
            double alignErrorDeg = AngleBetween(KsaFrameBridge.ThrustAxisCci(vehicle), _s.CommandDir) * 180.0 / Math.PI;
            if (alignErrorDeg <= GfoldIgnitionAlignDeg || now - _s.GfoldRelightRequestTime >= GfoldIgnitionAlignLimitS)
                _s.GfoldEngineOn = true;
        }
        if (!wantsBurn || _s.GfoldEngineOn)
            _s.GfoldRelightRequestTime = double.NaN;

        double previousThrust = _s.GfoldThrottle > 0.0
            ? KsaEnginePerf.ThrustAtThrottle(vehicle, _s.GfoldThrottle, pressure) : 0.0;
        double smoothedDemand = demand > 0.0 ? previousThrust + a * (demand - previousThrust) : demand;
        KsaEnginePerf.ThrustCommand command = KsaEnginePerf.CommandForThrust(vehicle, smoothedDemand, pressure);
        if (_s.GfoldEngineOn)
        {
            _s.GfoldThrottle = command.Throttle;
            _s.GfoldThrustStatus = command.Message;
        }
        else
        {
            _s.GfoldThrottle = 0.0;
            bool engineUsable = command.Status == KsaEnginePerf.ThrustStatus.Available
                || command.Status == KsaEnginePerf.ThrustStatus.BelowMinimum
                || command.Status == KsaEnginePerf.ThrustStatus.Off;
            _s.GfoldThrustStatus = !engineUsable ? command.Message
                : wantsBurn ? "Aligning for ignition." : "Planned coast, engine off.";
        }
        _s.HasCommand = true;
    }

    // The thrust direction of the first plan node from the given one whose thrust exceeds the re-light fraction, in the plan's local frame, or local up when the rest of the plan coasts.
    private static double3 NextBurnDirection(GfoldTrajectory plan, int from, double mass, double fullThrust)
    {
        if (fullThrust > 0.0 && double.IsFinite(fullThrust))
            for (int k = Math.Max(from, 1); k < plan.Nodes; k++)
            {
                double3 accel = Node(plan.AccelCmd, k);
                double len = accel.Length();
                if (len * mass > GfoldRelightFraction * fullThrust)
                    return accel * (1.0 / len);
            }
        return new double3(1, 0, 0);
    }

    // Unit thrust direction of v, clamped to within maxAngleDeg of local up (+X).
    // If v tilts past the cone, it's pushed onto the cone surface keeping azimuth.
    private static double3 ClampToCone(double3 v, double maxAngleDeg)
    {
        double len = v.Length();
        if (len < 1e-9) return new double3(1, 0, 0);
        double3 d = v * (1.0 / len);
        double cosMax = Math.Cos(Math.Clamp(maxAngleDeg, 0.0, 90.0) * Math.PI / 180.0);
        if (d.X >= cosMax) return d;                       // already within the cone
        double sinMax = Math.Sqrt(Math.Max(1.0 - cosMax * cosMax, 0.0));
        var horiz = new double3(0, d.Y, d.Z);
        double h = horiz.Length();
        double3 hUnit = h > 1e-9 ? horiz * (1.0 / h) : new double3(0, 1, 0);
        return new double3(cosMax, sinMax * hUnit.Y, sinMax * hUnit.Z);
    }

    // The formulation carries no fuel budget, so a usable conic status can still describe a burn the tanks cannot supply. FuelMass counts every tank on the craft, so this can refuse a plan but cannot prove that the landing engines reach that propellant.
    private static bool FitsFuel(GfoldTrajectory plan, GfoldParams p) => plan.FuelUsed <= p.FuelMass;

    // A failed solve holds the last command briefly; a short run of failures gives the vehicle back rather than flying a stale (often sideways) command in.
    private static void FailGfold(Vehicle vehicle, string message)
    {
        _s.GfoldFailStreak++;
        // A failed re-solve is not fatal once we hold a feasible plan: keep flying the last committed trajectory (the solver usually only chokes on the degenerate last few metres, where the existing plan lands fine) and just tell the user.
        // Only give up when there's nothing to fly - no plan was ever found.
        if (_s.GfoldPlan != null)
        {
            _s.DiagLanding($"G-FOLD re-solve failed ({_s.GfoldFailStreak}) - flying last trajectory. {message}");
            return;
        }
        _s.DiagLanding(message);
        if (_s.GfoldFailStreak > 3)
        {
            if (_s.LandingPhase != LandingPhase.Done)
                GuidanceLog.Info(vehicle, $"G-FOLD gave up after {_s.GfoldFailStreak} failed solves: {message}");
            _s.LandingPhase = LandingPhase.Done;
            _s.LandingCutPending = true;
            EndLandingBadly(vehicle, "G-FOLD found no trajectory", "G-FOLD found no trajectory. You fly.");
        }
    }
}

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
        _s.LandingStatus = "HORIZONTAL BRAKING COMPLETE. DESCENT WITH ENGINES DOWN.";
        return true;
    }

    // A craft that passes below the gate outside its limits, or outlives the plan to it, cannot reach the gate any more.
    // It lands on the site directly, which also allows the hover handoff again.
    private static bool TryAbandonVerticalGate(double altitude, double now)
    {
        bool missed = altitude <= _s.LandingVerticalGateM || (_s.GfoldPlan != null && now > _s.GfoldArrivalTime);
        if (!GfoldBrakingToGate || !missed) return false;
        ChangeGfoldApproach(GfoldApproach.Direct, now);
        _s.WarnLanding("VERTICAL APPROACH GATE MISSED. G-FOLD LANDS ON SITE.");
        return true;
    }

    // A new target makes the previous flight time a wrong guess, so the next solve searches the full range.
    private static void ChangeGfoldApproach(GfoldApproach approach, double now)
    {
        _s.CancelGfoldSolve();
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
    private const double GfoldRequestMaxAgeS = 1.25;

    // A planned coast is flown as a real coast. The engine cannot deliver less than its minimum throttle, so a demand under this fraction of full thrust switches it off, and it lights again only above the re-light fraction, so a command that sits near the threshold does not toggle the engine every step.
    private const double GfoldCoastFraction = 0.02;
    private const double GfoldRelightFraction = 0.06;

    // Ignition after a coast waits until the thrust axis points along the command, because an engine lit during the slew throws its thrust sideways. The wait is bounded, because a craft that cannot align in time is still better off braking.
    private const double GfoldIgnitionAlignDeg = 15.0;
    private const double GfoldIgnitionAlignLimitS = 3.0;

    // Flight-time search bounds.
    // SearchTfMax caps the flight-time search from a new target or from the remaining time of the previous plan.
    private const double SearchTfMax = 120.0;

    // Flight-time searches rest after a propellant refusal, while the fixed-arrival solve keeps its normal cadence.
    private const double GfoldFuelSearchRetryS = 2.0;

    // Shown while the plan flies to touchdown after a refused handoff, and kept through the re-solves that clear the status on success. The reason is in the guidance log.
    private const string GfoldHoverRefusedStatus = "NO HOVER HANDOVER. G-FOLD TO TOUCHDOWN.";

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
                _s.LandingStatus = $"HANDOVER: HOVER AT {_s.GfoldAltM:F0} m";
                return;
            }
            _s.GfoldHoverRefused = true;
            GuidanceLog.Info(vehicle, $"G-FOLD flies its plan to touchdown from {_s.GfoldAltM:F0} m: {_s.LandingStatus}");
            _s.WarnLanding(GfoldHoverRefusedStatus);
        }

        // Inside the last GfoldMinTf seconds before the planned arrival, the distance still to fly is too small for any valid flight time: tf >= TfMin (the search floor) overshoots it, so a re-solve goes degenerate and reports the target unreachable right before the handoff.
        // Freeze the committed plan there - it already terminates at the target - and just fly it down.
        if (GfoldInTerminalWindow(now))
            _s.CancelGfoldSolve();
        else
        {
            PublishGfoldPlan(vehicle, now, gfLocal, localVelocity);
            if (_s.GfoldJob is { } job && now - job.SimTime > GfoldRequestMaxAgeS)
            {
                _s.CancelGfoldSolve();
                FailGfold(vehicle, "G-FOLD SOLVE EXPIRED WHILE THE CRAFT WAS MOVING.");
            }
            if (_s.LandingPhase == LandingPhase.GfoldDescent && _s.GfoldJob == null
                && now - _s.GfoldLastSolveTime >= _s.GfoldIntervalS)
            {
                _s.GfoldJobSiteCcf = siteCci.Transform(parent.GetCci2Ccf());
                SubmitGfoldPlan(vehicle, parent, frame, siteCci, r, now);
            }
        }

        // Fly the committed plan by time index every frame (feed-forward + PD).
        // Track in the LIVE site frame (rebuilt this step), not the solve-time frame: the site is body-fixed, so its CCI position rotates with the body, and the live frame carries the plan around with it so we keep aiming at the real pad.
        if (_s.GfoldPlan != null)
        {
            var planFrame = _s.GfoldPlanSiteCcf.LengthSquared() > 0.0
                ? KsaGfold.BuildFrame(_s.GfoldPlanSiteCcf.Transform(parent.GetCcf2Cci()))
                : frame;
            TrackGfoldPlan(vehicle, parent, planFrame, r, vSrf, vehicle.TotalMass, now);
        }

        string ApproachState() => $"ALT {_s.GfoldAltM:F1} m, HORIZONTAL SPEED {horizontalSpeed:F2} m/s"
            + $", SINK {-localVelocity.X:F2} m/s, DISTANCE {horizontalDistance:F1} m, THRUST-AXIS TILT {tilt:F1} deg";
    }

    // Capture the current game state on the simulation thread. The solve uses only these numeric inputs.
    private static void SubmitGfoldPlan(Vehicle vehicle, IParentBody parent,
                                       KsaGfold.Frame frame, double3 siteCci, double3 comPos, double now)
    {
#if DEBUG
        using var _perf = new AdvancedFlightComputer.Core.PerfTracker.Scope("Guidance.SubmitGfoldPlan");
#endif
        GfoldParams p = KsaGfold.BuildParams(
            vehicle, parent, frame, siteCci, comPos, GfoldActiveGlideSlopeDeg, GfoldActivePointingDeg, _s.GfoldVMaxMs,
            GfoldSolverTargetAltM, GfoldSolverArrivalRateMs, _s.GfoldThrottleMin, _s.GfoldThrottleMax,
            EffectiveAccelLimitG(vehicle) * StandardGravity, out string refusal);
        if (p == null)
        {
            EndLandingBadly(vehicle, "G-FOLD: NO SOLUTION", refusal);
            _s.GfoldLastSolveTime = now;
            _s.GfoldPlan = null;
            _s.GfoldThrottle = 0.0;
            _s.HasCommand = false;
            _s.LandingPhase = LandingPhase.Done;
            _s.LandingCutPending = true;
            return;
        }

        _s.GfoldLastSolveTime = now;
        double arrival = !_s.GfoldForceSearch && _s.GfoldPlan != null ? _s.GfoldArrivalTime : double.NaN;
        var options = GfoldOptions.Descent with { SlewReg = _s.GfoldSlewReg, SolveTimeLimitS = _s.GfoldSolveTimeLimitS };
        _s.GfoldJob = new GfoldSolveJob(new GfoldSolveJob.Request(p, _s.GfoldNodes, now, arrival,
            GfoldMinTf, SearchTfMax, _s.GfoldPlan != null && now < _s.GfoldSearchRetryTime, options));
    }

    private static void PublishGfoldPlan(Vehicle vehicle, double now, double3 position, double3 velocity)
    {
        GfoldSolveJob job = _s.GfoldJob;
        if (job == null || !job.TryTakeResult(out GfoldSolveJob.Result result))
            return;
        _s.CancelGfoldSolve();
        _s.GfoldSolveMs = result.SolveMs;

        GfoldParams p = job.Parameters;
        switch (result.Status)
        {
            case GfoldSolveJob.Outcome.Resting:
                FailGfold(vehicle, FormattableString.Invariant($"G-FOLD SHORT OF PROPELLANT. SEARCH AGAIN IN {_s.GfoldSearchRetryTime - now:F1} s"));
                return;
            case GfoldSolveJob.Outcome.Unreachable:
                FailGfold(vehicle, FormattableString.Invariant($"G-FOLD UNREACHABLE: ALT {_s.GfoldAltM:F0} m, {_s.GfoldSpeedMs:F0} m/s, ")
                    + FormattableString.Invariant($"TWR {p.ThrustMax / (vehicle.TotalMass * p.GravityMag):F1}, FUEL {p.FuelMass:F0} kg")
                    + (double.IsFinite(p.AccelMax) ? FormattableString.Invariant($", THRUST ACCELERATION LIMIT {p.AccelMax / p.GravityMag:F1} local g") : ""));
                return;
            case GfoldSolveJob.Outcome.SearchTimedOut:
                FailGfold(vehicle, "G-FOLD FLIGHT-TIME SEARCH REACHED ITS TIME LIMIT.");
                return;
            case GfoldSolveJob.Outcome.FuelShort:
                _s.GfoldSearchRetryTime = now + GfoldFuelSearchRetryS;
                FailGfold(vehicle, FormattableString.Invariant($"G-FOLD NEEDS {result.Trajectory.FuelUsed:F0} kg PROPELLANT, {p.FuelMass:F0} kg ABOARD"));
                return;
            case GfoldSolveJob.Outcome.Fault:
                FailGfold(vehicle, "G-FOLD: " + result.Error);
                return;
        }

        // The trajectory starts at the measured state in the request, so advance through it by the time spent solving.
        GfoldTrajectory candidate = result.Trajectory;
        double elapsed = now - job.SimTime;
        double horizon = result.Arrival - job.SimTime;
        double maxAge = Math.Clamp(horizon * 0.2, 0.5, GfoldRequestMaxAgeS);
        if (elapsed < 0 || elapsed > maxAge || !MatchesGfoldStart(candidate, elapsed, position, velocity))
        {
            FailGfold(vehicle, "G-FOLD PLAN NO LONGER MATCHES THE DESCENT STATE.");
            return;
        }

        _s.GfoldStatus = candidate.Status;
        _s.GfoldPlan = candidate;
        _s.GfoldPlanSiteCcf = _s.GfoldJobSiteCcf;
        _s.GfoldPlanStart = job.SimTime;
        _s.GfoldArrivalTime = result.Arrival;
        _s.GfoldThrustMax = p.ThrustMax;
        _s.GfoldFailStreak = 0;
        _s.GfoldForceSearch = false;
        _s.GfoldSearchRetryTime = double.NegativeInfinity;
        _s.LandingStatus = _s.GfoldHoverRefused ? GfoldHoverRefusedStatus
            : GfoldBrakingToGate ? $"BRAKING TO VERTICAL APPROACH AT {_s.LandingVerticalGateM:F0} m"
            : GfoldVerticalFinal ? "VERTICAL FINAL DESCENT" : "";
        if (_s.GfoldHoverRefused)
            _s.WarnLanding(GfoldHoverRefusedStatus);
    }

    private static bool MatchesGfoldStart(GfoldTrajectory plan, double elapsed, double3 position, double3 velocity)
    {
        double sample = Math.Clamp(elapsed / plan.Dt, 0.0, plan.Nodes - 1);
        int node = Math.Min((int)sample, plan.Nodes - 2);
        double fraction = sample - node;
        double3 expectedPosition = Lerp(Node(plan.Position, node), Node(plan.Position, node + 1), fraction);
        double3 expectedVelocity = Lerp(Node(plan.Velocity, node), Node(plan.Velocity, node + 1), fraction);
        double positionTolerance = Math.Clamp(expectedVelocity.Length() * 0.1, 3.0, 15.0);
        double velocityTolerance = Math.Clamp(expectedVelocity.Length() * 0.08, 2.0, 8.0);
        return (expectedPosition - position).Length() <= positionTolerance
            && (expectedVelocity - velocity).Length() <= velocityTolerance;
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
                : wantsBurn ? "ALIGNING FOR IGNITION" : "PLANNED COAST, ENGINE OFF";
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
            EndLandingBadly(vehicle, "G-FOLD: NO TRAJECTORY", "G-FOLD: NO TRAJECTORY. MANUAL.");
        }
    }
}

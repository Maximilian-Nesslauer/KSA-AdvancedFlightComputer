#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;

// 6-DOF ON STANDBY through the 3-DOF's landing burn, so the hand-over swaps to a plan that is already converged and warm. The braking burn of DEORBIT + LAND keeps its standby the same way from its hand-over time (StepBrakingHandover).
//
// Before this the hand-over started a cold 6-DOF solve with the engine lit, 170-270 m up at 90 m/s. In every flight that took over a second, and until it published nothing flew the craft: the engine sat at the 3-DOF's last throttle and the flight computer on its last attitude, and the craft lost 80-100 m of the height it had left before 6-DOF steered at all.
//
// So from the first burn step the 6-DOF guidance is built and cold-solved on its own solve thread, and from then on re-solved back to back from the measured state - but nothing it plans reaches the vehicle, which stays the 3-DOF's. It runs on the same fields 6-DOF flies on (Guidance, Worker, Converging and the bookkeeping), so the swap is Active = true and nothing more (PromoteSixDofStandby): the next 6-DOF step carries on from the plan the standby last published.
//
// THE RE-SOLVE IS Ksa6DofGuidance.Track, NOT THE FLYING LOOP'S Update. Update assumes the craft follows the plan, and this one follows the 3-DOF's: each cycle it was refused, its work thrown away and the next attempt started from the same ageing plan, so the first standby flown published nothing for 1.8 s after converging. Track carries on from its own latest trajectory whether or not it passed the gate, so it keeps pace with the craft, and it runs whenever the worker is free rather than on the flying cadence.
//
// THE BURN-TIME SEED IS THE 3-DOF'S TIME TO GO, not the 20 s default. The problem's sigma bounds, conditioning and node count are all sized on it (Ksa6DofSetup, ColdNodesFor), and a seed several times the real burn is much of why the cold engage took so many iterations.
public static partial class GuidanceWindow
{
    /// <summary>Oldest a standby plan may be and still be handed the burn, s: five re-solves at the default cadence.</summary>
    private const double SixDofStandbyMaxPlanAgeS = 0.5;

    /// <summary>Cold iterations before the standby gives up on a cold solve and starts again from the current state.</summary>
    private const int SixDofStandbyColdLimit = 40;

    /// <summary>Wait before setting the standby up again after it could not be, s.</summary>
    private const double SixDofStandbyRetryS = 1.0;

    /// <summary>Shortest burn-time seed the standby and the hand-over give 6-DOF, s. Below it the sigma bounds pinch the solver.</summary>
    private const double SixDofStandbyMinSeedS = 2.0;

    /// <summary>
    /// Seconds between game-log lines while the standby is not publishing, saying why. The first flight's standby went stale for 1.8 s and the log could not say why; this is what will.
    /// </summary>
    private const double SixDofStandbyStaleLogS = 1.0;


    /// <summary>Iterations per standby re-solve: few, because they run back to back and each one starts where the last left off.</summary>
    private const int SixDofStandbyTrackIterations = 2;

    /// <summary>How long the standby may go without publishing before it starts again from the current state, s.</summary>
    private const double SixDofStandbyStaleRestartS = 3.0;

    /// <summary>The time one subproblem may take for the guidance that starts as the standby under the braking burn, ms; see Ksa6DofGuidance.LongBudgetMs.</summary>
    private const double BrakingStandbyBudgetMs = 1000.0;

    private const double BrakingHandoffAngleRad = 5.0 * Math.PI / 180.0;

    /// <summary>
    /// THE TURN-AWAY: the latest a hand-over to touchdown guidance from a mode that flies the craft's attitude itself (bellyflop, entry's glide) may come, so touchdown has the craft before its plan asks for an attitude the other mode does not fly: TurnAwayLeadS before the first time touchdown's standby plan is more than TurnAwayDeg from the attitude flown now (Ksa6DofGuidance.TurnAwayTime); +infinity with no plan. Any craft: the plan says when it turns. A Starship's plans, with its whole-sphere aero, began turning it for the flip within their first seconds, on the air and the flaps, while bellyflop held it belly first until 5 s before ignition; re-solved from the craft bellyflop held, they drifted off and were refused, and the standby never took it (harness, 2026-11-24).
    /// </summary>
    internal static double SixDofTurnAwayDue(double[] state, double now)
        => _s.Guidance?.Published is { } plan
            ? Ksa6DofGuidance.TurnAwayTime(plan, state, now, TurnAwayDeg * Math.PI / 180.0) - TurnAwayLeadS
            : double.PositiveInfinity;

    // 15 deg since 2026-11-27 (was 5), the user's call, to leave the craft with bellyflop longer: at 5 the standby plan's first turn handed a Starship over at 1680-1950 m, 31-36 s before ignition (14:25, 16:37), where the other landings that day were handed over at the latest, about 5 s before it.
    private const double TurnAwayDeg = 15.0;
    private const double TurnAwayLeadS = 1.0;
    private const double BrakingHandoffRateRadS = 3.0 * Math.PI / 180.0;

    /// <summary>True when the standby has a converged plan fresh enough to be handed the burn.</summary>
    private static bool SixDofStandbyReady(double now, double maxPlanAgeS = SixDofStandbyMaxPlanAgeS,
                                          double[] brakingState = null)
        => _s.SixDofStandby && !_s.Converging && _s.Guidance?.Published is { } plan
           && now - plan.SolveTime <= maxPlanAgeS
           && now - plan.SolveTime < plan.Duration
           && (brakingState == null || (GuidanceIdle && Ksa6DofGuidance.MatchesHandoffAttitude(
               plan, brakingState, now, BrakingHandoffAngleRad, BrakingHandoffRateRadS)
               && _s.Guidance.MeasureDrift(brakingState, now) <= SixDofRestartDriftLimit(brakingState[2])));

    /// <summary>The burn-time floor for restarts and node rebuilds: the configured seed, or the entry seed when this engagement started from a shorter burn (the 3-DOF hand-over). Restarting a 3 s burn on a 20 s seed would pin sigma at a minimum longer than the burn.</summary>
    /// <summary>The progressive glideslope's knots for the log: each height, its half-angle from vertical, and the offset it allows there.</summary>
    private static string FunnelText(Scvx6DofConfig cfg)
    {
        var sb = new System.Text.StringBuilder();
        for (int i = 0; i < cfg.GlideFunnelHeightsM.Length; i++)
            sb.Append(i > 0 ? ", " : "").Append(FormattableString.Invariant(
                $"{cfg.GlideFunnelHalfAngleDeg[i]:F0} deg from vertical at {cfg.GlideFunnelHeightsM[i]:F0} m over the target ({cfg.GlideSlopeRadiusAt(cfg.GlideFunnelHeightsM[i]):F0} m of offset)"));
        return sb.ToString();
    }

    private static double SixDofSeedFloor => Math.Min(_s.SixDofSigmaSeed, _s.SixDofEntrySigmaSeed);

    /// <summary>
    /// Build the 6-DOF guidance for the craft as it is now and start its cold solve on its own worker thread, commanding nothing. False, with the reason in the log, if 6-DOF cannot plan for it.
    /// </summary>
    private static bool StartSixDofStandby(Vehicle vehicle, IParentBody parent, double3 siteCci, double[] x, double now,
                                           double sigmaSeed, int nodes = 0)
    {
        StopSixDofStandby();
        _s.SixDofStandbyRetryAt = now + SixDofStandbyRetryS;
        if (nodes <= 0)
            nodes = ScvxUsesClarabel
                ? (_s.SixDofCoastPending ? TwoGridCoastNodes : TwoGridBurnNodes)
                : MaxNodes;

        // The standby runs the spread cold solve; the two options that replace it plan synchronously on engage instead.
        if (_s.SixDofFixedTime || _s.SixDofGfoldSeed)
        {
            string setting = _s.SixDofFixedTime ? "FIXED BURN TIME" : "G-FOLD SEED";
            LogSixDofStandbyRefusal(vehicle, $"DISABLED BY {setting}.");
            return false;
        }

        _s.SixDofEntrySigmaSeed = sigmaSeed;
        if (!TryConfigure6Dof(vehicle, parent, siteCci, x, out double[] xf, out int engageNodes,
                              out var cfg, out var dyn, out string error, nodes))
        {
            LogSixDofStandbyRefusal(vehicle, "Touchdown guidance standby cannot plan: " + error);
            return false;
        }

        // Capture all live vehicle and state inputs on the main thread before handing ownership to the worker.
        bool braking = _s.LandingPhase == LandingPhase.Burn;
        double leadS = Ksa6DofSetup.LateralTorqueLead(vehicle);
        SeedPath seedPath = _s.SixDofCoastPending ? EntrySeedPath(now) ?? BellyflopSeedPath(now) : null;
        double coastSeed = _s.SixDofCoastPending ? _s.SixDofCoastSeedS : double.NaN;
        double[] xfCopy = (double[])xf.Clone();

        var worker = new Ksa6DofSolveWorker();
        _s.Guidance = null;
        _s.Worker = worker;
        _s.SixDofStandby = true;
        _s.SixDofStandbyEngines = CaptureStandbyEngines(vehicle);
        _s.SixDofStandbyStart = now;
        _s.SixDofStandbyRefusal = "";
        _s.SixDofStandbyRefusals = 0;
        _s.Converging = true;
        _s.ColdFrames = 0;
        _s.SixDofLastPublishedPlan = null;
        ResetNodeLadderForBackend(ScvxUsesClarabel);
        _s.RefusalRun = 0;
        _s.PrevV = null;
        _s.Bias = default;
        _s.LastReplan = now;
        _s.LastMass = vehicle.TotalMass;
        _s.DiagError("standby: converging...");

        string what = $"BURN-TIME SEED {sigmaSeed:F1} s, {engageNodes} NODES"
                      + (cfg.HasCoast ? FormattableString.Invariant($", COAST {_s.SixDofCoastSeedS:F1} s TO IGNITION OVER {cfg.CoastIntervals} INTERVALS") : "");
        if (_s.SixDofLogging)
        {
            SixDofLog.Start(_s, vehicle.ToString(), parent.ToString());
            SixDofLog.Event(_s, now, $"STANDBY at alt {x[2]:F0} m: cold solve on the worker, {what}");
        }
        GuidanceLog.Info(vehicle, $"Touchdown guidance standby at {x[2]:F0} m: cold solve on its own thread, {what}.");
        double rad = 180.0 / Math.PI;
        GuidanceLog.Info(vehicle, FormattableString.Invariant(
            $"Touchdown guidance thrust moment {dyn.ThrustMomentX:F3} {dyn.ThrustMomentY:F3} {dyn.ThrustMomentZ:F3} m (N m per N): gimbal cone centred on a {Math.Atan(cfg.GimbalTrim) * rad:F2} deg trim, {cfg.GimbalMaxDeg:F2} deg around it."));
        if (cfg.GlideSlopeProgressive)
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"Touchdown guidance glide funnel: {FunnelText(cfg)}; outer cone {cfg.GlideSlopeDeg:F0} deg above the horizon."));
        if (cfg.SideSpeedEnabled)
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"Touchdown guidance sideways speed: at most {cfg.SideSpeedPerM:F2} m/s per metre of height over the target plus {cfg.SideSpeedFloorMs:F1} m/s ({cfg.SideSpeedPerM * 10.0 + cfg.SideSpeedFloorMs:F1} m/s at 10 m)."));
        GuidanceLog.Debug(vehicle, FormattableString.Invariant($"Touchdown guidance model: air moment limiter {_s.MomentLimiterPerPa * 1e3:F3} per kPa (divides by {_s.MomentLimiterDivisor:F2} now), Tmax {cfg.Tmax:F0} N, floor {cfg.ThrottleFloor:F3}, gimbal {cfg.GimbalMaxDeg:F2} deg, roll torque {cfg.TauRollMax:F0} N m{(cfg.TerminalRollFree ? " (roll left free)" : "")}, engine arm {dyn.LArm:F2} m, inertia {dyn.Ixx:F0} {dyn.Iyy:F0} {dyn.Izz:F0} kg m2, Isp {dyn.Isp:F0} s, tilt limit {cfg.TiltMaxDeg:F0} deg, scales {cfg.XScale[0]:F0} m {cfg.XScale[3]:F0} m/s, plan ends {_s.SixDofTargetComAltM:F2} m over the site of which {_s.SiteEarlyContactM:F2} m for the ground under the footprint, which rises {(_s.SiteGround?.Within(_s.SiteGround.ReachM) ?? 0.0):F2} m within {(_s.SiteGround?.ReachM ?? 0.0):F0} m of the site."));

        // Dispatch after all cfg/dyn log reads complete so only the worker accesses them during cold init.
        worker.TryDispatchInitCold((sx, st) =>
        {
            var g = new Ksa6DofGuidance(cfg, dyn, leadS)
            {
                FixedTime = false,
                ColdIterationIntervalS = 0.0,
                BrakingSeed = braking,
                LongBudgetMs = braking ? BrakingStandbyBudgetMs : 0.0,
                ColdSeedPath = seedPath,
            };
            g.BeginCold(sx, xfCopy, sigmaSeed, coastSeed, st);
            return g;
        }, x, now);
        return true;
    }

    // One line per distinct reason, so a craft 6-DOF cannot take does not log it every second of the burn.
    private static void LogSixDofStandbyRefusal(Vehicle vehicle, string why)
    {
        if (why != _s.SixDofStandbyRefusal)
            GuidanceLog.Info(vehicle, why);
        _s.SixDofStandbyRefusal = why;
    }

    /// <summary>Drop the standby: its worker abandons whatever it is solving, and the guidance goes with it.</summary>
    private static void StopSixDofStandby()
    {
        if (!_s.SixDofStandby)
            return;
        _s.SixDofStandby = false;
        _s.SixDofStandbyEngines = null;
        _s.Converging = false;
        // Reference first, as Disengage6Dof does: a dispose that throws must still leave no live worker behind.
        Ksa6DofSolveWorker worker = _s.Worker;
        Ksa6DofGuidance guidance = _s.Guidance;
        _s.Worker = null;
        _s.Guidance = null;
        bool guidanceOwned = worker == null;
        try
        {
            if (worker != null)
            {
                guidanceOwned = worker.TryStopWhenIdle();
                if (!guidanceOwned)
                    worker.Dispose();
            }
        }
        catch { /* stopping must always succeed */ }
        if (guidanceOwned)
            try { guidance?.Dispose(); } catch { /* stopping must always succeed */ }
        if (ReferenceEquals(SixDofLog.Owner, _s))
            ReportLogStop(SixDofLog.Stop(_s));
    }

    /// <summary>
    /// One standby step through the 3-DOF's burn: collect what the worker finished, then dispatch the next cold iteration or warm re-solve from the measured state. Commands nothing. A fault costs the standby, never the 3-DOF flying the craft.
    /// </summary>
    private static void StepSixDofStandby(Vehicle vehicle, IParentBody parent, double3 siteCci, double[] x, double now,
                                          double sigmaSeed, bool holdForEntryHandoff = false)
    {
        try
        {
            StepSixDofStandbyCore(vehicle, parent, siteCci, x, now, sigmaSeed, holdForEntryHandoff);
        }
        catch (Exception e)
        {
            LogSixDofFault(vehicle, e);
            GuidanceLog.Info(vehicle, "Touchdown guidance standby faulted, retrying: " + e.Message);
            StopSixDofStandby();
            _s.SixDofStandbyRetryAt = now + SixDofStandbyRetryS;
        }
    }

    private static void StepSixDofStandbyCore(Vehicle vehicle, IParentBody parent, double3 siteCci, double[] x, double now,
                                              double sigmaSeed, bool holdForEntryHandoff)
    {
        if (!StandbyEnginesStillMatch(vehicle))
            StopSixDofStandby();
        if (!_s.SixDofStandby)
        {
            if (now >= _s.SixDofStandbyRetryAt)
                StartSixDofStandby(vehicle, parent, siteCci, x, now, sigmaSeed);
            return;
        }

        // Kept current for the staging check 6-DOF runs from its first step once it flies.
        _s.LastMass = vehicle.TotalMass;

        Ksa6DofSolveResult result = null;
        if (_s.Worker.TryCollect(out result))
        {
            if (result.Job == Ksa6DofJob.InitCold)
            {
                if (result.Faulted || !result.Ok || result.Produced == null)
                {
                    GuidanceLog.Info(vehicle, "Touchdown guidance standby init faulted: " + result.Error + " - retrying.");
                    StopSixDofStandby();
                    _s.SixDofStandbyRetryAt = now + SixDofStandbyRetryS;
                    return;
                }
                _s.Guidance = result.Produced;
                if (_s.SixDofLogging && _s.Guidance.HasCoast)
                {
                    SixDofLog.Event(_s, now, _s.Guidance.SeededAlongPath ? "STANDBY COAST SEEDED ALONG ENTRY'S PLAN" : "STANDBY FREE-COAST SEED");
                    SixDofLog.PlanSnapshot(_s, now, _s.Guidance.WorkingPlan(now), "seed");
                }
                result = null;
            }
            else if (result.Job == Ksa6DofJob.Rebuild)
            {
                // A node step reseeded on the worker (BeginRebuild).
                FinishNodeRebuild(result, now);
                result = null;
            }
        }
        if (_s.Guidance == null)
            return;
        if (result != null
            && result.Matches(_s.Converging ? Ksa6DofJob.StepCold : Ksa6DofJob.Track, _s.Guidance))
        {
            if (result.Faulted)
            {
                GuidanceLog.Info(vehicle, "Touchdown guidance standby solve faulted: " + result.Error + " - starting again.");
                StartSixDofStandby(vehicle, parent, siteCci, x, now, sigmaSeed);
                return;
            }
            if (result.Job == Ksa6DofJob.StepCold)
            {
                _s.ColdFrames++;
                // Every cold iteration in the events log: a standby that stalls says how (2026-11-20: ten restarts in a glide, and nothing to read them from).
                ScvxIteration it = _s.Guidance.LastIteration;
                // A coast standby's first iterates in the plan log, so a stall can be read node by node (2026-11-20: a roll-rate error at the first burn interval that the offline seed does not show).
                if (_s.Guidance.HasCoast && _s.Guidance.LastIterations <= 3)
                    SixDofLog.PlanSnapshot(_s, now, _s.Guidance.WorkingPlan(now), "iterate");
                SixDofLog.Event(_s, now, FormattableString.Invariant(
                    $"STANDBY COLD {_s.Guidance.LastIterations} at {x[2]:F0} m: {(it.Solved ? it.Accepted ? "accepted" : "rejected" : "FAILED " + _s.Guidance.LastSubproblemFailure)}, rho {it.Rho:G3}, trust {it.TrustRegion:G3}, burn {it.Sigma:F2} s, coast {_s.Guidance.WorkingCoast:F2} s, SCS {it.SolverIterations}, worst {Scvx6DofSolver.ChannelName(it.DefectChannel)} at {it.DefectNode}, gate {_s.Guidance.LastGatedRatio:G3}x on {_s.Guidance.LastGatedDefectChannelName} at {_s.Guidance.LastGatedDefectNode}{(result.Ok ? "" : ": " + result.Error)}"));
                if (result.Ok)
                {
                    _s.Converging = false;
                    _s.Error = "";
                    _s.LastReplan = now;
                    string what = $"{_s.Guidance.LastIterations} ITERATIONS IN {now - _s.SixDofStandbyStart:F1} s, "
                                + $"SIGMA {_s.Guidance.Sigma:F1} s, {_s.Guidance.Nodes} NODES, DEFECT {_s.Guidance.DefectText}";
                    GuidanceLog.Info(vehicle, $"Touchdown guidance standby converged at {x[2]:F0} m: {what}.");
                    SixDofLog.Event(_s, now, "STANDBY COLD SOLVE CONVERGED: " + what);
                    SixDofLog.PlanSnapshot(_s, now, _s.Guidance.Published);
                }
                else if (_s.Guidance.NeedsMoreNodes || _s.ColdFrames >= SixDofStandbyColdLimit)
                {
                    // The 3-DOF still has the craft, so starting again from here costs nothing but time.
                    int nodes = _s.Guidance.NeedsMoreNodes ? FinerNodeRung(_s.Guidance.Nodes) : 0;
                    GuidanceLog.Info(vehicle, (_s.Guidance.NeedsMoreNodes
                            ? $"Touchdown guidance standby cold solve stalled at {_s.Guidance.DefectText} with {_s.Guidance.Nodes} nodes"
                            : $"Touchdown guidance standby cold solve not converged in {_s.ColdFrames} iterations")
                        + $" - starting again at {x[2]:F0} m" + (nodes > 0 ? $" with {nodes} nodes." : "."));
                    StartSixDofStandby(vehicle, parent, siteCci, x, now, sigmaSeed, nodes);
                    return;
                }
                else
                {
                    _s.DiagError($"standby: converging... {_s.Guidance.LastIterations} iterations, defect {_s.Guidance.DefectText}");
                }
            }
            else if (result.Ok)
            {
                _s.DidSolve = true;
                OnSolveSucceeded(now, x);
                _s.SixDofStandbyRefusals = 0;
            }
            else
            {
                // Counted apart from RefusalRun, which drives the flying loop's back-off and restart: a refused tracking attempt still moved the solve on, and is not the failure those remedies are for.
                _s.SixDofStandbyRefusals++;
                _s.DiagError("standby: re-solve refused: " + result.Error);
                SixDofLog.Event(_s, now, "STANDBY RE-SOLVE REFUSED: " + result.Error);
            }
        }

        Ksa6DofPlan published = _s.Guidance.Published;
        if (!_s.Converging && published != null && !ReferenceEquals(published, _s.SixDofLastPublishedPlan))
            NotePublishedPlan(published, now);

        // The worker owns the guidance while it solves.
        if (!GuidanceIdle)
            return;

        if (_s.Converging)
        {
            _s.Worker.TryDispatchStepCold(_s.Guidance, x, now);
            return;
        }

        // Leave a checked publication in place through the first command after handover.
        if ((_s.LandingPhase == LandingPhase.Burn && SixDofStandbyReady(now, BrakingStandbyMaxPlanAgeS, x))
            || (holdForEntryHandoff && SixDofStandbyReady(now, SixDofStandbyMaxPlanAgeS, x)))
            return;

        // Nothing published for too long, or the craft far off the last plan: start again from here.
        double age = published != null ? now - published.SolveTime : double.PositiveInfinity;
        if (age > SixDofStandbyMaxPlanAgeS && now - _s.SixDofStandbyLogTime >= SixDofStandbyStaleLogS)
        {
            _s.SixDofStandbyLogTime = now;
            GuidanceLog.Info(vehicle, $"Touchdown guidance standby not publishing at {x[2]:F0} m: plan {age:F1} s old, {_s.SixDofStandbyRefusals} refused, "
                + $"last solve {_s.Worker.LastSolveMs:F0} ms at {_s.Guidance.Nodes} nodes; {_s.Error}");
        }
        double drift = _s.Guidance.MeasureDrift(x, now);
        double driftLimit = SixDofRestartDriftLimit(x[2]);
        if (age > SixDofStandbyStaleRestartS || drift > driftLimit)
        {
            GuidanceLog.Info(vehicle, $"Touchdown guidance standby restarting at {x[2]:F0} m: "
                + (age > SixDofStandbyStaleRestartS
                    ? $"nothing published for {age:F1} s, {_s.SixDofStandbyRefusals} re-solves refused ({_s.Error})."
                    : $"{drift:F0} m off its plan."));
            StartSixDofStandby(vehicle, parent, siteCci, x, now, sigmaSeed);
            return;
        }

        // The thrust the flying mode is delivering, for the bias estimator, recorded as 6-DOF records its own so the estimate runs on unbroken across the swap.
        double ambientPa = KsaEnginePerf.AmbientPressureAt(parent, siteCci.Length() - parent.MeanRadius + x[2]);
        _s.CapabilityN = KsaEnginePerf.ActiveThrustCapability(vehicle, ambientPa);
        _s.LastThrottle = _s.LandingPhase == LandingPhase.Burn ? _s.Upfg.Throttle : _s.ThreeDofThrottle;
        // Off the engines' curve, as Step6DofCore reads it: throttle x full thrust overstates low throttle by tens of kN.
        // Not on a predicted state (SixDofStandbyPredicted): its velocity is a forecast that stands still, and differencing it would read as an acceleration the model lacks.
        if (!_s.SixDofStandbyPredicted)
        {
            double standbyThrust = _s.LastThrottle > 0.0 ? KsaEnginePerf.ThrustAtThrottle(vehicle, _s.LastThrottle, ambientPa) : 0.0;
            UpdateAccelBias(x, now, standbyThrust, ThrustDirModel(vehicle, standbyThrust), default, FlownFinDeg(vehicle));
        }

        // The node ladder too, so the guidance handed over is already at the count 6-DOF would fly. Only off a fresh plan: a rebuild reseeds from the published plan, on the worker (BeginRebuild).
        if (_s.SixDofNodeGates && age <= SixDofStandbyMaxPlanAgeS && !_s.SixDofCoastPending)
            StepNodeGates(vehicle, parent, siteCci, x, now);

        Ksa6DofSetup.Inertia(vehicle, out double ixx, out double iyy, out double izz);
        UpdateMomentLimiter(vehicle, now);
        _s.Guidance.Inputs = _s.Guidance.Inputs.WithInertia(ixx, iyy, izz)
            .WithArm(Ksa6DofSetup.EngineArm(vehicle))
            .WithTipGeometry(TipRadiusM(_s), _s.VehicleHeightM)
            .WithMomentLimiterExact(_s.MomentLimiterPerPa, _s.MomentLimiterStiffnessPerPa, _s.MomentLimiterDampingPerPaMs)
            .WithMomentBias(_s.SixDofStandbyPredicted ? _s.MomentBiasPerPa : UpdateMomentBias(vehicle, x, now));
        if (!_s.SixDofStandbyPredicted)
            LogAeroCheck(vehicle, x, now, "standby");
        // Back to back, not on the flying cadence: the craft is not following this plan, so the solve has to keep moving to keep up.
        _s.Worker.TryDispatchTrack(_s.Guidance, x, now, SixDofStandbyTrackIterations);
    }

    private static EngineController[] CaptureStandbyEngines(Vehicle vehicle)
    {
        var engines = new List<EngineController>();
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
            if (StandbyEngineSupplied(vehicle, engine))
                engines.Add(engine);
        return engines.ToArray();
    }

    private static bool StandbyEnginesStillMatch(Vehicle vehicle)
    {
        if (!_s.SixDofStandby)
            return true;
        EngineController[] expected = _s.SixDofStandbyEngines;
        if (expected == null)
            return false;
        int index = 0;
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
        {
            if (!StandbyEngineSupplied(vehicle, engine))
                continue;
            if (index >= expected.Length || !ReferenceEquals(expected[index++], engine))
                return false;
        }
        return index == expected.Length;
    }

    private static bool StandbyEngineSupplied(Vehicle vehicle, EngineController engine)
    {
        if (!engine.IsActive)
            return false;
        return ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
            .TryGetFrom(vehicle.Parts.States, out var states)
            && states.States[engine.StatesIdx].IsPropellantAvailable;
    }

    /// <summary>The next node rung finer than <paramref name="nodes"/>, or the finest.</summary>
    private static int FinerNodeRung(int nodes)
    {
        if (ScvxUsesClarabel)
            return nodes < TwoGridBurnNodes ? TwoGridBurnNodes
                : nodes < TwoGridCoastNodes ? TwoGridCoastNodes
                : MaxNodes;
        for (int i = NodeRungs.Length - 1; i >= 0; i--)
            if (NodeRungs[i] > nodes)
                return NodeRungs[i];
        return MaxNodes;
    }

    /// <summary>
    /// Hand the standby the craft: it is 6-DOF from this step on, flying the plan it has or, if its cold solve is still running, carrying on with that exactly as a cold engage would.
    /// </summary>
    private static void PromoteSixDofStandby(Vehicle vehicle, double now, double[] x)
    {
        LogLandingEngineState(vehicle, now, "touchdown handover");
        bool converging = _s.Converging;
        Ksa6DofPlan plan = _s.Guidance.Published;
        _s.SixDofStandby = false;
        _s.Active = true;
        _s.Guidance.ArmIgnitionCap(plan != null && plan.HasCoast ? plan.IgnitionTime : double.NaN, plan != null ? plan.SolveTime + plan.Duration : double.NaN);
        // The standby's sideways bias stays behind: the flying mode steers through stock TVC, whose gimbal angles the estimate cannot see (no Lsq pass, KsaGimbalControl.Slot.ThrustDirKnown), so their sideways thrust was learnt as a missing force; touchdown learns its own from here, through the gimbals it measures.
        if (!KeepStandbyBias && KsaGimbalControl.Diagnostics(vehicle) is { ThrustDirKnown: true })
            _s.Bias = new Brutal.Numerics.double3(0.0, 0.0, _s.Bias.Z);
        _s.RefusalRun = 0;
        _s.TouchdownArmed = false;
        _s.SixDofSeenDescending = _s.SixDofTookBurn;
        _s.SixDofToContact = false;
        _s.SixDofRestartOpenLoop = false;
        _s.SixDofFromBrakingBurn = false;
        _s.DiagError(converging ? "converging..." : "");

        string what = converging || plan == null
            ? $"COLD SOLVE RUNNING ({_s.Guidance.LastIterations} ITERATIONS)"
            : FormattableString.Invariant($"WARM PLAN {now - plan.SolveTime:F2} s OLD: {plan.Duration:F1} s TO TARGET{(plan.HasCoast ? FormattableString.Invariant($", IGNITION IN {plan.IgnitionTime - now:F1} s") : "")}, {_s.Guidance.Nodes} NODES");
        GuidanceLog.Info(vehicle, $"Touchdown guidance takes the burn from standby at {x[2]:F0} m with {what}.");
        SixDofLog.Event(_s, now, "ENGAGED FROM STANDBY: " + what);
    }

    /// <summary>What the standby is doing, for the 3-DOF's log line and page.</summary>
    internal static string SixDofStandbyText(double now)
    {
        if (!_s.SixDofStandby)
            return _s.SixDofStandbyRefusal.Length > 0 ? _s.SixDofStandbyRefusal : "NOT RUNNING";
        if (_s.Converging)
            return "converging";
        Ksa6DofPlan plan = _s.Guidance?.Published;
        if (plan == null)
            return "NO PLAN";
        return SixDofStandbyReady(now)
            ? FormattableString.Invariant($"READY. PLAN {now - plan.SolveTime:F1} s OLD, {plan.Duration:F1} s TO GO")
            : FormattableString.Invariant($"CATCHING UP. PLAN {now - plan.SolveTime:F1} s OLD, {_s.SixDofStandbyRefusals} REFUSED");
    }
}

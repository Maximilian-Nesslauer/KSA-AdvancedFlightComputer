#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using KSA;

// 6-DOF ON STANDBY through the 3-DOF's landing burn, so the hand-over swaps to a plan that is already converged and warm.
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

    /// <summary>True when the standby has a converged plan fresh enough to be handed the burn.</summary>
    private static bool SixDofStandbyReady(double now)
        => _s.SixDofStandby && !_s.Converging && _s.Guidance?.Published is { } plan
           && now - plan.SolveTime <= SixDofStandbyMaxPlanAgeS;

    /// <summary>The burn-time floor for restarts and node rebuilds: the configured seed, or the entry seed when this engagement started from a shorter burn (the 3-DOF hand-over). Restarting a 3 s burn on a 20 s seed would pin sigma at a minimum longer than the burn.</summary>
    private static double SixDofSeedFloor => Math.Min(_s.SixDofSigmaSeed, _s.SixDofEntrySigmaSeed);

    /// <summary>
    /// Build the 6-DOF guidance for the craft as it is now and start its cold solve on its own worker thread, commanding nothing. False, with the reason in the log, if 6-DOF cannot plan for it.
    /// </summary>
    private static bool StartSixDofStandby(Vehicle vehicle, IParentBody parent, double3 siteCci, double[] x, double now,
                                           double sigmaSeed, int nodes = 0)
    {
        StopSixDofStandby();
        _s.SixDofStandbyRetryAt = now + SixDofStandbyRetryS;
        // AT THE FINEST RUNG, not the count the engage would pick from the burn time. Mid-burn the craft is fast and the spacing that suits a descent is too coarse: the first standby flown stalled at 25, 30 and 40 nodes and only converged at 50, 3.6 s after ignition. Off the sim thread the extra nodes cost nothing that matters, and the ladder steps them down once there is a plan.
        if (nodes <= 0)
            nodes = MaxNodes;

        // The standby runs the spread cold solve; the two options that replace it plan synchronously on engage instead.
        if (_s.SixDofFixedTime || _s.SixDofGfoldSeed)
        {
            LogSixDofStandbyRefusal(vehicle, "6-DOF standby is off: it needs the spread cold solve, and fixed burn time or the G-FOLD seed is on.");
            return false;
        }

        _s.SixDofEntrySigmaSeed = sigmaSeed;
        if (!TryConfigure6Dof(vehicle, parent, siteCci, x, out double[] xf, out int engageNodes,
                              out var cfg, out var dyn, out string error, nodes))
        {
            LogSixDofStandbyRefusal(vehicle, "6-DOF standby cannot plan: " + error);
            return false;
        }

        // No pacing: the solve is off the sim thread, so there is no frame to protect.
        var guidance = new Ksa6DofGuidance(cfg, dyn) { FixedTime = false, ColdIterationIntervalS = 0.0 };
        guidance.BeginCold(x, xf, sigmaSeed);
        _s.Guidance = guidance;
        _s.Worker = new Ksa6DofSolveWorker();
        _s.SixDofStandby = true;
        _s.SixDofStandbyStart = now;
        _s.SixDofStandbyRefusal = "";
        _s.SixDofStandbyRefusals = 0;
        _s.Converging = true;
        _s.ColdFrames = 0;
        _s.SixDofLastPlanTime = now;
        _s.SixDofLastPublishedPlan = null;
        _s.GateIndex = -1;
        _s.RefusalRun = 0;
        _s.RungFloor = int.MaxValue;
        _s.RungFloorSpeed = 0.0;
        _s.BackedOffTo = -1;
        _s.PrevV = null;
        _s.Bias = default;
        _s.LastReplan = now;
        _s.LastMass = vehicle.TotalMass;
        _s.Error = "standby: converging...";

        string what = $"burn-time seed {sigmaSeed:F1} s, {engageNodes} nodes";
        if (_s.SixDofLogging)
        {
            SixDofLog.Start(_s, vehicle.ToString(), parent.ToString());
            SixDofLog.Event(_s, now, $"STANDBY at alt {x[2]:F0} m: cold solve on the worker, {what}");
        }
        GuidanceLog.Info(vehicle, $"6-DOF standby at {x[2]:F0} m: cold solve on its own thread, {what}.");
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
        _s.Converging = false;
        // Reference first, as Disengage6Dof does: a dispose that throws must still leave no live worker behind.
        Ksa6DofSolveWorker worker = _s.Worker;
        _s.Worker = null;
        _s.Guidance = null;
        try { worker?.Dispose(); } catch { /* stopping must always succeed */ }
        if (ReferenceEquals(SixDofLog.Owner, _s))
            ReportLogStop(SixDofLog.Stop(_s));
    }

    /// <summary>
    /// One standby step through the 3-DOF's burn: collect what the worker finished, then dispatch the next cold iteration or warm re-solve from the measured state. Commands nothing. A fault costs the standby, never the 3-DOF flying the craft.
    /// </summary>
    private static void StepSixDofStandby(Vehicle vehicle, IParentBody parent, double3 siteCci, double[] x, double now,
                                          double sigmaSeed)
    {
        try
        {
            StepSixDofStandbyCore(vehicle, parent, siteCci, x, now, sigmaSeed);
        }
        catch (Exception e)
        {
            LogSixDofFault(vehicle, e);
            GuidanceLog.Info(vehicle, "6-DOF standby faulted, retrying: " + e.Message);
            StopSixDofStandby();
            _s.SixDofStandbyRetryAt = now + SixDofStandbyRetryS;
        }
    }

    private static void StepSixDofStandbyCore(Vehicle vehicle, IParentBody parent, double3 siteCci, double[] x, double now,
                                              double sigmaSeed)
    {
        if (!_s.SixDofStandby)
        {
            if (now >= _s.SixDofStandbyRetryAt)
                StartSixDofStandby(vehicle, parent, siteCci, x, now, sigmaSeed);
            return;
        }

        // Kept current for the staging check 6-DOF runs from its first step once it flies.
        _s.LastMass = vehicle.TotalMass;

        if (_s.Worker.TryCollect(out Ksa6DofSolveResult result)
            && result.Matches(_s.Converging ? Ksa6DofJob.StepCold : Ksa6DofJob.Track, _s.Guidance))
        {
            if (result.Faulted)
            {
                GuidanceLog.Info(vehicle, "6-DOF standby solve faulted: " + result.Error + " - starting again.");
                StartSixDofStandby(vehicle, parent, siteCci, x, now, sigmaSeed);
                return;
            }
            if (result.Job == Ksa6DofJob.StepCold)
            {
                _s.ColdFrames++;
                if (result.Ok)
                {
                    _s.Converging = false;
                    _s.Error = "";
                    _s.LastReplan = now;
                    string what = $"{_s.Guidance.LastIterations} iterations in {now - _s.SixDofStandbyStart:F1} s, "
                                + $"sigma {_s.Guidance.Sigma:F1} s, {_s.Guidance.Nodes} nodes, defect {_s.Guidance.LastDefectM:F2} m";
                    GuidanceLog.Info(vehicle, $"6-DOF standby converged at {x[2]:F0} m: {what}.");
                    SixDofLog.Event(_s, now, "STANDBY COLD SOLVE CONVERGED: " + what);
                    SixDofLog.PlanSnapshot(_s, now, _s.Guidance.Published);
                }
                else if (_s.Guidance.NeedsMoreNodes || _s.ColdFrames >= SixDofStandbyColdLimit)
                {
                    // The 3-DOF still has the craft, so starting again from here costs nothing but time.
                    int nodes = _s.Guidance.NeedsMoreNodes ? FinerNodeRung(_s.Guidance.Nodes) : 0;
                    GuidanceLog.Info(vehicle, (_s.Guidance.NeedsMoreNodes
                            ? $"6-DOF standby cold solve stalled at {_s.Guidance.LastDefectM:F1} m with {_s.Guidance.Nodes} nodes"
                            : $"6-DOF standby cold solve not converged in {_s.ColdFrames} iterations")
                        + $" - starting again at {x[2]:F0} m" + (nodes > 0 ? $" with {nodes} nodes." : "."));
                    StartSixDofStandby(vehicle, parent, siteCci, x, now, sigmaSeed, nodes);
                    return;
                }
                else
                {
                    _s.Error = $"standby: converging... {_s.Guidance.LastIterations} iterations, defect {_s.Guidance.LastDefectM:F1} m";
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
                _s.Error = "standby: re-solve refused: " + result.Error;
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

        // Nothing published for too long, or the craft far off the last plan: start again from here.
        double age = published != null ? now - published.SolveTime : double.PositiveInfinity;
        if (age > SixDofStandbyMaxPlanAgeS && now - _s.SixDofStandbyLogTime >= SixDofStandbyStaleLogS)
        {
            _s.SixDofStandbyLogTime = now;
            GuidanceLog.Info(vehicle, $"6-DOF standby not publishing at {x[2]:F0} m: plan {age:F1} s old, {_s.SixDofStandbyRefusals} refused, "
                + $"last solve {_s.Worker.LastSolveMs:F0} ms at {_s.Guidance.Nodes} nodes; {_s.Error}");
        }
        double drift = _s.Guidance.MeasureDrift(x, now);
        double driftLimit = Math.Max(ColdRestartDriftM, 0.05 * Math.Max(x[2], 1.0));
        if (age > SixDofStandbyStaleRestartS || drift > driftLimit)
        {
            GuidanceLog.Info(vehicle, $"6-DOF standby restarting at {x[2]:F0} m: "
                + (age > SixDofStandbyStaleRestartS
                    ? $"nothing published for {age:F1} s, {_s.SixDofStandbyRefusals} re-solves refused ({_s.Error})."
                    : $"{drift:F0} m off its plan."));
            StartSixDofStandby(vehicle, parent, siteCci, x, now, sigmaSeed);
            return;
        }

        // The thrust the 3-DOF is delivering, for the bias estimator, recorded as 6-DOF records its own so the estimate runs on unbroken across the swap.
        double ambientPa = KsaEnginePerf.AmbientPressureAt(parent, siteCci.Length() - parent.MeanRadius + x[2]);
        _s.CapabilityN = KsaEnginePerf.ActiveThrustCapability(vehicle, ambientPa);
        _s.LastThrottle = _s.ThreeDofThrottle;
        UpdateAccelBias(x, now, _s.LastThrottle * _s.CapabilityN);

        // The node ladder too, so the guidance handed over is already at the count 6-DOF would fly. Only off a fresh plan: a rebuild reseeds from the published plan, synchronously.
        if (_s.SixDofNodeGates && age <= SixDofStandbyMaxPlanAgeS)
            StepNodeGates(vehicle, parent, siteCci, x, now);

        Ksa6DofSetup.Inertia(vehicle, out double ixx, out double iyy, out double izz);
        _s.Guidance.Inputs = _s.Guidance.Inputs.WithInertia(ixx, iyy, izz);
        // Back to back, not on the flying cadence: the craft is not following this plan, so the solve has to keep moving to keep up.
        _s.Worker.TryDispatchTrack(_s.Guidance, x, now, SixDofStandbyTrackIterations);
    }

    /// <summary>The next node rung finer than <paramref name="nodes"/>, or the finest.</summary>
    private static int FinerNodeRung(int nodes)
    {
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
        bool converging = _s.Converging;
        Ksa6DofPlan plan = _s.Guidance.Published;
        _s.SixDofStandby = false;
        _s.Active = true;
        _s.RefusalRun = 0;
        _s.TouchdownArmed = false;
        _s.SixDofHoverRefused = false;
        _s.SixDofFromBrakingBurn = false;
        _s.Error = converging ? "converging..." : "";

        string what = converging || plan == null
            ? $"its cold solve still running ({_s.Guidance.LastIterations} iterations so far)"
            : $"a warm plan {now - plan.SolveTime:F2} s old: {plan.Sigma:F1} s to the target, {_s.Guidance.Nodes} nodes";
        GuidanceLog.Info(vehicle, $"6-DOF takes the burn from standby at {x[2]:F0} m with {what}.");
        SixDofLog.Event(_s, now, "ENGAGED FROM STANDBY: " + what);
    }

    /// <summary>What the standby is doing, for the 3-DOF's log line and page.</summary>
    internal static string SixDofStandbyText(double now)
    {
        if (!_s.SixDofStandby)
            return _s.SixDofStandbyRefusal.Length > 0 ? "unavailable" : "not running";
        if (_s.Converging)
            return "converging";
        Ksa6DofPlan plan = _s.Guidance?.Published;
        if (plan == null)
            return "no plan";
        return SixDofStandbyReady(now)
            ? $"ready, plan {now - plan.SolveTime:F1} s old, {plan.Sigma:F1} s to go"
            : $"catching up, plan {now - plan.SolveTime:F1} s old, {_s.SixDofStandbyRefusals} refused";
    }
}

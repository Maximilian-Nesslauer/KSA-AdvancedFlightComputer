using System.Collections.Generic;
using System.Diagnostics;
using AutoStage.Core;
using Brutal.Logging;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AutoStage;

/// <summary>
/// Triggers staging once per frame, around the vehicle-solver apply.
///
///   Monitoring -> active engines lose propellant -> stage
///   Monitoring -> the next row would shed only spent engines -> stage
///   AwaitingIgnition -> both delays elapsed -> AwaitingPropagation
///   AwaitingPropagation -> new engines fueled -> Monitoring, else cascade stage
///
/// The second trigger drops burnt-out boosters while the core keeps firing; the
/// all-dry edge never comes for a stack that still has thrust. AwaitingPropagation
/// exists because IsPropellantAvailable only flips a tick after activation, and
/// BurnMode is forced to Auto across that window so the worker cannot abort the burn.
///
/// State belongs to the controlled vehicle, a pending staging does not: it ticks
/// against the vehicle it was committed on, whatever is controlled.
/// </summary>
static class StagingDetector
{
    enum State { Monitoring, AwaitingIgnition, AwaitingPropagation }

    private static State _state = State.Monitoring;
    private static int _propagationFrames;
    private static double _spentJettisonSince = double.NaN;
    // Last reason reported by ReportSpentJettison, so it only speaks on change.
    // A non-null sentinel, because null is the armed state: starting there would
    // suppress the first real arm line.
    private static string? _spentJettisonBlocker = "";
    private static FlightComputerBurnMode _triggeredMode;
    private static PendingStaging? _pendingStaging;
    private static Vehicle? _currentVehicle;

    // Taken by Sample before the solver results reach the universe and consumed
    // by Evaluate afterwards. The vehicle doubles as the validity flag, so one
    // sample can never be evaluated twice.
    private static Vehicle? _sampledVehicle;
    private static FlightComputerBurnMode _sampledBurnMode;
    private static bool _sampledHadPropellant;

    // 1 frame for worker thread to process new engines, +1 margin.
    private const int PropagationFrames = 2;

    // This trigger is level-triggered, not edge-triggered, so it needs a dwell.
    // Sim time, not frames: Evaluate also runs over frozen state while paused,
    // where a frame counter would arm and fire.
    private const double SpentJettisonDwellSeconds = 0.25;

    /// <summary>
    /// Both values are edges the applied worker results destroy. Sampled here
    /// rather than carried forward from last frame: ApplyInputEvents drains in
    /// between, so an engine the player shut down would look like a burnout.
    /// </summary>
    internal static void Sample()
    {
        Vehicle? vehicle = Program.ControlledVehicle;
        _sampledVehicle = vehicle;

        if (vehicle == null || !Mod.AutoStageEnabled)
        {
            _sampledBurnMode = FlightComputerBurnMode.Manual;
            _sampledHadPropellant = false;
            return;
        }

        _sampledBurnMode = vehicle.FlightComputer.BurnMode;
        _sampledHadPropellant = StagingHelpers.HasActiveEngineWithPropellant(vehicle);
    }

    internal static void Evaluate()
    {
#if DEBUG
        long perfStart = DebugConfig.Performance ? Stopwatch.GetTimestamp() : 0;
#endif
        // First, and whatever is controlled or armed: the row is already marked
        // activated, and stock skips activated rows forever, so a module dropped
        // here can never fire again. Returns the vehicle it finished on, which
        // the focus check below can no longer read off the cleared slot.
        Vehicle? justCompletedOn = TickPendingStaging(Universe.GetElapsedSeconds());

        Vehicle? vehicle = _sampledVehicle;
        _sampledVehicle = null;
        if (vehicle == null)
        {
            // Absolute sim time, and the sim runs on with nothing controlled, so
            // a dwell left armed would be expired on the first frame back.
            _spentJettisonSince = double.NaN;
            _spentJettisonBlocker = "";
            return;
        }

        if (!Mod.AutoStageEnabled)
        {
            // Dropped rather than frozen: sim time keeps running while the mod
            // is off, so a dwell left armed here would already be expired on
            // the frame it is switched back on.
            _propagationFrames = 0;
            _spentJettisonSince = double.NaN;
            _spentJettisonBlocker = "";
            if (_state != State.AwaitingIgnition)
                _state = State.Monitoring;
            return;
        }

        if (_currentVehicle != vehicle)
        {
            _propagationFrames = 0;
            _spentJettisonSince = double.NaN;
            _spentJettisonBlocker = "";
            _currentVehicle = vehicle;
            // Resume rather than reset, so returning mid-delay does not start a
            // second staging on top of the one still executing.
            if (_pendingStaging != null && _pendingStaging.Vehicle == vehicle)
            {
                _state = State.AwaitingIgnition;
            }
            else if (justCompletedOn == vehicle)
            {
                _state = State.AwaitingPropagation;
            }
            else
            {
                _state = State.Monitoring;
            }
        }

        FlightComputer fc = vehicle.FlightComputer;
        IReadOnlySet<Part>? jettison = _state == State.Monitoring && Config.DropSpentStages
            ? JettisonAnalysis.GetPendingJettison(vehicle)
            : null;

        // Only the spent-jettison trigger needs the full tally. Without a
        // pending jettison, stay on the early-exiting scan: for a quenched
        // solid motor the per-core answer runs a fixed-point pressure solve.
        StagingHelpers.EngineSurvey survey = default;
        bool hasPropellant;
        if (jettison != null)
        {
            survey = StagingHelpers.SurveyActiveEngines(vehicle, jettison);
            hasPropellant = survey.AnyFueled;
        }
        else
        {
            hasPropellant = StagingHelpers.HasActiveEngineWithPropellant(vehicle);
        }

        switch (_state)
        {
            case State.Monitoring:
                if (_sampledHadPropellant && !hasPropellant
                    && !IsBurnComplete(fc)
                    && StagingHelpers.HasNextEngineSequence(vehicle))
                {
                    _spentJettisonSince = double.NaN;
                    ExecuteStaging(vehicle, fc, _sampledBurnMode);
                }
                else if (Config.DropSpentStages)
                {
                    TickSpentJettison(vehicle, fc, _sampledBurnMode, in survey, jettison);
                }
                else
                {
                    // Switched off: say nothing rather than report it as the
                    // analysis having declined, which is a different answer.
                    _spentJettisonSince = double.NaN;
                }
                break;

            case State.AwaitingIgnition:
                MaintainBurnMode(fc);
                break;

            case State.AwaitingPropagation:
                _propagationFrames++;
                MaintainBurnMode(fc);

                if (hasPropellant)
                {
                    _state = State.Monitoring;
                }
                else if (_propagationFrames >= PropagationFrames)
                {
                    if (!IsBurnComplete(fc)
                        && StagingHelpers.HasNextEngineSequence(vehicle))
                        ExecuteStaging(vehicle, fc, _triggeredMode);
                    else
                        _state = State.Monitoring;
                }
                break;
        }

#if DEBUG
        if (DebugConfig.Performance)
            PerfTracker.Record("StagingDetector.Evaluate",
                Stopwatch.GetTimestamp() - perfStart);
#endif
    }

    /// <summary>
    /// Stages while the vehicle is still under thrust, when the next sequence
    /// would shed nothing but burnt-out engines. Requires thrust to remain
    /// afterwards, so a stack that is simply running out falls to the all-dry
    /// trigger instead and keeps its cascade behaviour.
    /// </summary>
    private static void TickSpentJettison(Vehicle vehicle, FlightComputer fc,
        FlightComputerBurnMode mode, in StagingHelpers.EngineSurvey survey,
        IReadOnlySet<Part>? jettison)
    {
        // SpentInside is only ever counted for parts inside a pending jettison,
        // so a non-zero count already implies there is one.
        string? blocker =
            jettison == null ? "no pending jettison to judge"
            : survey.SpentInside == 0 ? "nothing spent among the parts that would be shed"
            : survey.InactiveInside > 0 ? $"{survey.InactiveInside} engine(s) to be shed have not run"
            : survey.FueledInside > 0 ? $"{survey.FueledInside} engine(s) to be shed still have propellant"
            : survey.BrokenInside > 0 ? $"{survey.BrokenInside} engine(s) to be shed have an unresolved motor stack"
            : !survey.ThrustingOutside ? "nothing staying aboard is thrusting"
            : IsBurnComplete(fc) ? "the planned burn is already complete"
            : null;

        if (blocker != null)
        {
            ReportSpentJettison(vehicle, blocker, in survey, measured: jettison != null);
            _spentJettisonSince = double.NaN;
            return;
        }

        double now = Universe.GetElapsedSeconds();
        if (double.IsNaN(_spentJettisonSince))
        {
            ReportSpentJettison(vehicle, null, in survey, measured: true);
            _spentJettisonSince = now;
            return;
        }
        if (now - _spentJettisonSince < SpentJettisonDwellSeconds)
            return;

        // Last gate, deliberately after the dwell: it walks the jettisoned
        // parts' tanks, and running it here costs a scan a few times per stage
        // instead of one every frame the boosters read spent.
        _spentJettisonSince = double.NaN;
        if (JettisonAnalysis.CarriesOffUsablePropellant(vehicle, jettison!, out string? carried))
        {
            ReportSpentJettison(vehicle, carried, in survey, measured: true);
            return;
        }

        if (DebugConfig.AutoStage)
        {
            DefaultCategory.Log.Debug(
                $"[AutoStage] Shedding spent stage on '{vehicle.Id}': {survey.SpentInside} " +
                $"spent engine(s) jettisoned, {survey.FueledOutside} fueled engine(s) staying.");
            // Named once per drop, so a wrong jettison can be read straight off
            // the log instead of reconstructed from counters.
            foreach (Part part in jettison!)
            {
                // SubtreeModules: everything on a shed part leaves with it.
                Span<EngineController> engines = part.SubtreeModules.Get<EngineController>();
                for (int i = 0; i < engines.Length; i++)
                    DefaultCategory.Log.Debug(
                        $"[AutoStage]   shedding engine on '{part.DisplayName}' "
                        + $"(active={engines[i].IsActive}, seq={engines[i].Sequence}).");
            }
        }

        ExecuteStaging(vehicle, fc, mode);
    }

    /// <summary>
    /// Once per change, because the condition is evaluated every frame and
    /// silence makes "rode along" and "correctly refused" look alike. When
    /// <paramref name="measured"/> is false the survey was skipped, so its zero
    /// counters mean "not counted", not "nothing running".
    /// </summary>
    private static void ReportSpentJettison(Vehicle vehicle, string? blocker,
        in StagingHelpers.EngineSurvey survey, bool measured)
    {
        if (!DebugConfig.AutoStage || blocker == _spentJettisonBlocker)
            return;
        _spentJettisonBlocker = blocker;

        // Only read when a line is actually emitted, so the scan stays off the
        // per-frame path.
        int next = vehicle.Parts.SequenceList.GetNextSequenceNumber();
        string detail = measured
            ? $"next sequence {next}, inside: {survey.SpentInside} spent / "
              + $"{survey.FueledInside} fueled / {survey.BrokenInside} broken / "
              + $"{survey.InactiveInside} not run, "
              + $"outside: {survey.FueledOutside} fueled, thrusting={survey.ThrustingOutside}"
            : $"next sequence {next}, engines not surveyed";

        DefaultCategory.Log.Debug(blocker == null
            ? $"[AutoStage] Spent-stage drop armed on '{vehicle.Id}' ({detail})."
            : $"[AutoStage] Spent-stage drop held on '{vehicle.Id}': {blocker} ({detail}).");
    }

    /// <summary>
    /// Fires whichever pending parts have reached their deadline, on the
    /// vehicle the staging was committed against rather than on whatever is
    /// controlled now.
    /// </summary>
    private static Vehicle? TickPendingStaging(double now)
    {
        PendingStaging? p = _pendingStaging;
        if (p == null)
            return null;

        if (p.Vehicle.IsDisposed)
        {
            _pendingStaging = null;
            return null;
        }

        if (p.DecouplersPending && now >= p.DecouplerDeadline)
        {
            StagingExecution.ActivatePendingModules(p.Vehicle, p.DecouplerModules!, "decoupler");
            p.ClearDecouplers();
        }

        if (p.EnginesPending && now >= p.EngineDeadline)
        {
            StagingExecution.ActivatePendingModules(p.Vehicle, p.EngineModules!, "engine");
            p.ClearEngines();
        }

        if (p.AnyPending)
            return null;

        _pendingStaging = null;
        // Only the monitored vehicle advances the state machine here; for any
        // other the caller picks the wait up from the return value.
        if (_state == State.AwaitingIgnition && _currentVehicle == p.Vehicle)
        {
            _state = State.AwaitingPropagation;
            _propagationFrames = 0;
        }
        return p.Vehicle;
    }

    /// <summary>
    /// Fires every part still pending, ignoring its deadline. Used when a
    /// second staging is about to take the single pending slot: dropping the
    /// first would leave its already-activated sequence unfired for good, and
    /// firing it early only shortens a delay that exists for looks.
    /// </summary>
    private static void FlushPendingStaging()
    {
        PendingStaging? p = _pendingStaging;
        if (p == null)
            return;

        if (DebugConfig.IgnitionDelay)
            DefaultCategory.Log.Debug(
                $"[AutoStage] Firing the pending staging on '{p.Vehicle.Id}' early, "
                + "a second staging needs the slot.");

        TickPendingStaging(double.PositiveInfinity);
    }

    private static void ExecuteStaging(Vehicle vehicle, FlightComputer fc,
        FlightComputerBurnMode originalBurnMode)
    {
        if (DebugConfig.AutoStage)
        {
            string dvInfo = fc.Burn != null
                ? $"dV remaining = {fc.Burn.DeltaVToGoCci.Length():F1} m/s"
                : "no burn planned";
            DefaultCategory.Log.Debug(
                $"[AutoStage] Staging ({originalBurnMode} mode): {dvInfo}");
        }

        _triggeredMode = originalBurnMode;

        // There is one pending slot. A staging on another vehicle can still own
        // it after a focus change, so empty it before it is overwritten.
        FlushPendingStaging();

        PendingStaging? pending = StagingExecution.ActivateNextSequenceSplit(vehicle);

        if (originalBurnMode == FlightComputerBurnMode.Auto && fc.Burn != null)
            fc.BurnMode = FlightComputerBurnMode.Auto;

        if (pending != null)
        {
            _pendingStaging = pending;
            _state = State.AwaitingIgnition;

            if (pending.DecouplersPending && pending.DecouplerDelay > 0.0)
                TimedAlert.Create(
                    $"Decouple in {pending.DecouplerDelay:F1}s",
                    Color.Yellow, pending.DecouplerDelay);
            if (pending.EnginesPending && pending.EngineDelay > 0.0)
                TimedAlert.Create(
                    $"Ignition in {pending.EngineDelay:F1}s",
                    Color.Yellow, pending.EngineDelay);

            DefaultCategory.Log.Info(
                $"[AutoStage] Staging delay on '{vehicle.Id}': " +
                $"decouplers={pending.DecouplerDelay:F1}s " +
                $"({pending.DecouplerModules?.Count ?? 0}), " +
                $"engines={pending.EngineDelay:F1}s " +
                $"({pending.EngineModules?.Count ?? 0})");
        }
        else
        {
            _state = State.AwaitingPropagation;
            _propagationFrames = 0;
        }
    }

    /// <summary>
    /// FlightComputer.ComputeControl drops BurnMode to Manual after two denied
    /// ignitions, which is what a freshly staged engine looks like until its
    /// propellant state propagates. Manual wins within a worker pass, this wins
    /// at the transition.
    /// </summary>
    private static void MaintainBurnMode(FlightComputer fc)
    {
        if (_triggeredMode == FlightComputerBurnMode.Auto
            && fc.Burn != null
            && !IsBurnComplete(fc)
            && fc.BurnMode == FlightComputerBurnMode.Manual)
        {
            fc.BurnMode = FlightComputerBurnMode.Auto;
        }
    }

    /// <summary>
    /// True once what is left to go no longer points along the target. The
    /// zero-target guard is load-bearing: Burn outlives its BurnPlan entry and
    /// is saved, so a vehicle can load with a zero DeltaVTargetCci, which passes
    /// the overshoot test forever and would disable both triggers for the flight.
    /// </summary>
    private static bool IsBurnComplete(FlightComputer fc)
    {
        BurnTarget? burn = fc.Burn;
        if (burn == null || burn.DeltaVTargetCci.IsNearlyZero())
            return false;
        return float3.Dot(burn.DeltaVToGoCci, burn.DeltaVTargetCci) <= 0f;
    }

    /// <summary>
    /// Modules left held at unload are dead for the flight, because their row is
    /// already activated. Firing early only shortens a cosmetic delay.
    /// </summary>
    internal static void FlushPendingForUnload()
    {
        PendingStaging? p = _pendingStaging;
        if (p == null || p.Vehicle.IsDisposed)
            return;

        DefaultCategory.Log.Info(
            $"[AutoStage] Unloading with a staging still pending on '{p.Vehicle.Id}': "
            + $"firing {p.DecouplerModules?.Count ?? 0} decoupler(s) and "
            + $"{p.EngineModules?.Count ?? 0} engine(s) now, because their sequence "
            + "is already marked activated and nothing would fire them later.");

        TickPendingStaging(double.PositiveInfinity);
    }

    /// <summary>
    /// Drops the references a disposed vehicle would otherwise keep alive. A
    /// pending staging on it is abandoned rather than fired: its parts are
    /// going away with it.
    /// </summary>
    internal static void ForgetVehicle(Vehicle vehicle)
    {
        if (_currentVehicle == vehicle)
        {
            _currentVehicle = null;
            _state = State.Monitoring;
            _propagationFrames = 0;
            _spentJettisonSince = double.NaN;
            _spentJettisonBlocker = "";
        }
        if (_sampledVehicle == vehicle)
            _sampledVehicle = null;
        if (_pendingStaging?.Vehicle == vehicle)
            _pendingStaging = null;
    }

    internal static void Reset()
    {
        _state = State.Monitoring;
        _propagationFrames = 0;
        _spentJettisonSince = double.NaN;
        _spentJettisonBlocker = "";
        _triggeredMode = FlightComputerBurnMode.Manual;
        _pendingStaging = null;
        _currentVehicle = null;
        _sampledVehicle = null;
        _sampledBurnMode = FlightComputerBurnMode.Manual;
        _sampledHadPropellant = false;
        JettisonAnalysis.Reset();
    }
}

/// <summary>
/// The only point in the apply still guaranteed to be on the main thread and to
/// run once per frame: UpdateFromTaskResultsUnsynchronized is parcelled out per
/// bubble, and its synchronized counterpart is AggressiveInlining. Prefix
/// samples ahead of every bubble's results, postfix decides once all are in.
/// </summary>
[HarmonyPatch(typeof(Universe), nameof(Universe.ApplyVehicleSolvers))]
static class Patch_Universe_ApplyVehicleSolvers
{
    static void Prefix() => StagingDetector.Sample();

    static void Postfix() => StagingDetector.Evaluate();
}

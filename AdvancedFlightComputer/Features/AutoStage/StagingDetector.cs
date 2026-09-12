using System.Diagnostics;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

// Triggers staging once per frame per armed vehicle, around the vehicle-solver apply.
//
//   Monitoring -> the active engines lose propellant -> stage
//   Monitoring -> the next row would shed only spent engines -> stage
//   Monitoring -> a caller requested one row -> stage
//   AwaitingIgnition -> both delays elapsed -> AwaitingPropagation
//   AwaitingPropagation -> new engines fueled -> Monitoring, else cascade stage
//
// AwaitingPropagation exists because a freshly activated engine reports propellant only a tick
// after activation, and BurnMode is held at Auto across that window so the worker cannot abort
// the burn. Each vehicle runs its own machine, so an armed craft keeps staging when it is not the
// controlled one.
internal static class StagingDetector
{
    private static readonly Dictionary<Vehicle, StagingState> _states = new();
    // Snapshot for the per-frame walks, because a staging can dispose a vehicle mid-walk.
    private static readonly List<KeyValuePair<Vehicle, StagingState>> _walk = new();

    // One frame for the worker to process the new engines, plus one margin.
    private const int PropagationFrames = 2;

    // Level-triggered, so it needs a dwell. Sim time, because Evaluate also runs while paused.
    private const double SpentJettisonDwellSeconds = 0.25;

    internal static bool IsArmed(Vehicle vehicle)
        => _states.TryGetValue(vehicle, out StagingState? state) && state.Active;

    internal static void Arm(Vehicle vehicle, bool active)
    {
        StagingState state = StateOf(vehicle);
        if (state.Active == active)
            return;
        state.Active = active;
        if (!active)
        {
            // Dropped rather than frozen: sim time runs on while the switch is off, so a dwell left
            // armed would already be expired on the frame it is switched back on.
            state.PropagationFrames = 0;
            state.ResetDwell();
            if (state.State != StagingState.Phase.AwaitingIgnition)
                state.State = StagingState.Phase.Monitoring;
        }
        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug($"[AFC] AutoStage {(active ? "armed" : "disarmed")} on '{vehicle.Id}'.");
    }

    // Asks for one row on the next evaluation, for a caller with a cue of its own such as a planned
    // stage boundary or a cold ignition. Ignored while a staging is already in flight on the vehicle.
    internal static void RequestStaging(Vehicle vehicle) => StateOf(vehicle).Requested = true;

    internal static bool HasState(Vehicle vehicle) => _states.ContainsKey(vehicle);

    internal static StagingState StateOf(Vehicle vehicle)
    {
        if (!_states.TryGetValue(vehicle, out StagingState? state))
        {
            state = new StagingState();
            _states[vehicle] = state;
        }
        return state;
    }

    // Both values are edges the applied worker results destroy, and ApplyInputEvents drains in
    // between, so an engine the player shut down would otherwise look like a burnout.
    internal static void Sample()
    {
        foreach (KeyValuePair<Vehicle, StagingState> entry in _states)
        {
            StagingState state = entry.Value;
            Vehicle vehicle = entry.Key;
            state.Sampled = state.Active && !vehicle.IsDisposed;
            if (!state.Sampled)
                continue;
            state.SampledBurnMode = vehicle.FlightComputer.BurnMode;
            state.SampledHadPropellant = StagingHelpers.HasActiveEngineWithPropellant(vehicle);
        }
    }

    internal static void Evaluate()
    {
#if DEBUG
        long perfStart = DebugConfig.Performance ? Stopwatch.GetTimestamp() : 0;
#endif
        _walk.Clear();
        _walk.AddRange(_states);
        double now = Universe.GetElapsedSeconds();
        foreach (KeyValuePair<Vehicle, StagingState> entry in _walk)
        {
            Vehicle vehicle = entry.Key;
            StagingState state = entry.Value;
            if (vehicle.IsDisposed)
            {
                _states.Remove(vehicle);
                continue;
            }

            // First, armed or not: the row is already marked activated and stock skips activated
            // rows forever, so a module dropped here could never fire again.
            TickPendingStaging(vehicle, state, now);

            bool sampled = state.Sampled;
            state.Sampled = false;
            if (!sampled || !state.Active)
                continue;

            EvaluateVehicle(vehicle, state, now);
        }
#if DEBUG
        if (DebugConfig.Performance)
            PerfTracker.Record("StagingDetector.Evaluate", Stopwatch.GetTimestamp() - perfStart);
#endif
    }

    private static void EvaluateVehicle(Vehicle vehicle, StagingState state, double now)
    {
        FlightComputer fc = vehicle.FlightComputer;
        IReadOnlySet<Part>? jettison = state.State == StagingState.Phase.Monitoring && StagingConfig.DropSpentStages
            ? JettisonAnalysis.GetPendingJettison(vehicle, state)
            : null;

        // Only the spent-jettison trigger needs the full tally; for a quenched solid motor the
        // per-core answer runs a fixed-point pressure solve.
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

        switch (state.State)
        {
            case StagingState.Phase.Monitoring:
                if (state.Requested)
                {
                    state.Requested = false;
                    state.SpentJettisonSince = double.NaN;
                    ExecuteStaging(vehicle, state, fc, state.SampledBurnMode, "requested");
                }
                else if (state.SampledHadPropellant && !hasPropellant
                    && !IsBurnComplete(fc)
                    && StagingHelpers.HasNextEngineSequence(vehicle))
                {
                    state.SpentJettisonSince = double.NaN;
                    ExecuteStaging(vehicle, state, fc, state.SampledBurnMode, "burnout");
                }
                else if (StagingConfig.DropSpentStages)
                {
                    TickSpentJettison(vehicle, state, fc, now, in survey, jettison);
                }
                else
                {
                    state.SpentJettisonSince = double.NaN;
                }
                break;

            case StagingState.Phase.AwaitingIgnition:
                state.Requested = false;
                MaintainBurnMode(state, fc);
                break;

            case StagingState.Phase.AwaitingPropagation:
                state.Requested = false;
                state.PropagationFrames++;
                MaintainBurnMode(state, fc);

                if (hasPropellant)
                {
                    state.State = StagingState.Phase.Monitoring;
                }
                else if (state.PropagationFrames >= PropagationFrames)
                {
                    if (!IsBurnComplete(fc) && StagingHelpers.HasNextEngineSequence(vehicle))
                        ExecuteStaging(vehicle, state, fc, state.TriggeredMode, "cascade");
                    else
                        state.State = StagingState.Phase.Monitoring;
                }
                break;
        }
    }

    // Stages while the vehicle is still under thrust, when the next sequence would shed nothing
    // but burnt-out engines. Requires thrust to remain afterwards, so a stack that is simply
    // running out falls to the all-dry trigger and keeps its cascade behaviour.
    private static void TickSpentJettison(Vehicle vehicle, StagingState state, FlightComputer fc, double now,
        in StagingHelpers.EngineSurvey survey, IReadOnlySet<Part>? jettison)
    {
        StagingState.SpentDropBlocker blocker =
            jettison == null ? StagingState.SpentDropBlocker.NoJettison
            : survey.SpentInside == 0 ? StagingState.SpentDropBlocker.NothingSpent
            : survey.InactiveInside > 0 ? StagingState.SpentDropBlocker.InactiveInside
            : survey.FueledInside > 0 ? StagingState.SpentDropBlocker.FueledInside
            : survey.BrokenInside > 0 ? StagingState.SpentDropBlocker.BrokenInside
            : !survey.ThrustingOutside ? StagingState.SpentDropBlocker.NothingThrusting
            : IsBurnComplete(fc) ? StagingState.SpentDropBlocker.BurnComplete
            : StagingState.SpentDropBlocker.None;

        if (blocker != StagingState.SpentDropBlocker.None)
        {
            ReportSpentJettison(vehicle, state, blocker, in survey, measured: jettison != null);
            state.SpentJettisonSince = double.NaN;
            return;
        }

        if (double.IsNaN(state.SpentJettisonSince))
        {
            ReportSpentJettison(vehicle, state, StagingState.SpentDropBlocker.None, in survey, measured: true);
            state.SpentJettisonSince = now;
            return;
        }
        if (now - state.SpentJettisonSince < SpentJettisonDwellSeconds)
            return;

        // Last gate, after the dwell, because it walks the jettisoned parts' tanks.
        state.SpentJettisonSince = double.NaN;
        if (JettisonAnalysis.CarriesOffUsablePropellant(vehicle, jettison!, out string? carried))
        {
            ReportSpentJettison(vehicle, state, StagingState.SpentDropBlocker.CarriesPropellant, in survey, measured: true, carried);
            return;
        }

        if (DebugConfig.AutoStage)
        {
            DefaultCategory.Log.Debug(
                $"[AFC] AutoStage shedding a spent stage on '{vehicle.Id}': {survey.SpentInside} spent engine(s) jettisoned, {survey.FueledOutside} fueled engine(s) staying.");
            foreach (Part part in jettison!)
            {
                Span<EngineController> engines = part.SubtreeModules.Get<EngineController>();
                for (int i = 0; i < engines.Length; i++)
                    DefaultCategory.Log.Debug(
                        $"[AFC]   shedding an engine on '{part.DisplayName}' (active={engines[i].IsActive}, seq={engines[i].Sequence}).");
            }
        }

        ExecuteStaging(vehicle, state, fc, state.SampledBurnMode, "spent drop");
    }

    // Once per change, because silence makes "rode along" and "correctly refused" look alike.
    private static void ReportSpentJettison(Vehicle vehicle, StagingState state, StagingState.SpentDropBlocker blocker,
        in StagingHelpers.EngineSurvey survey, bool measured, string? carried = null)
    {
        if (!DebugConfig.AutoStage || blocker == state.SpentJettisonBlockerReported)
            return;
        state.SpentJettisonBlockerReported = blocker;

        int next = vehicle.Parts.SequenceList.GetNextSequenceNumber();
        string detail = measured
            ? $"next sequence {next}, inside: {survey.SpentInside} spent / {survey.FueledInside} fueled / " +
              $"{survey.BrokenInside} broken / {survey.InactiveInside} not run, " +
              $"outside: {survey.FueledOutside} fueled, thrusting={survey.ThrustingOutside}"
            : $"next sequence {next}, engines not surveyed";
        string reason = blocker switch
        {
            StagingState.SpentDropBlocker.NoJettison => "no pending jettison to judge",
            StagingState.SpentDropBlocker.NothingSpent => "nothing spent among the parts that would be shed",
            StagingState.SpentDropBlocker.InactiveInside => $"{survey.InactiveInside} engine(s) to be shed have not run",
            StagingState.SpentDropBlocker.FueledInside => $"{survey.FueledInside} engine(s) to be shed still have propellant",
            StagingState.SpentDropBlocker.BrokenInside => $"{survey.BrokenInside} engine(s) to be shed have an unresolved motor stack",
            StagingState.SpentDropBlocker.NothingThrusting => "nothing staying aboard is thrusting",
            StagingState.SpentDropBlocker.BurnComplete => "the planned burn is already complete",
            StagingState.SpentDropBlocker.CarriesPropellant => carried ?? "the shed parts still feed the vehicle",
            _ => "",
        };

        DefaultCategory.Log.Debug(blocker == StagingState.SpentDropBlocker.None
            ? $"[AFC] Spent-stage drop armed on '{vehicle.Id}' ({detail})."
            : $"[AFC] Spent-stage drop held on '{vehicle.Id}': {reason} ({detail}).");
    }

    // Fires whichever pending parts have reached their deadline.
    private static void TickPendingStaging(Vehicle vehicle, StagingState state, double now)
    {
        PendingStaging? p = state.Pending;
        if (p == null)
            return;

        if (p.DecouplersPending && now >= p.DecouplerDeadline)
        {
            StagingExecution.ActivatePendingModules(vehicle, p.DecouplerModules!, "decoupler");
            p.ClearDecouplers();
        }

        if (p.EnginesPending && now >= p.EngineDeadline)
        {
            StagingExecution.ActivatePendingModules(vehicle, p.EngineModules!, "engine");
            p.ClearEngines();
        }

        if (p.AnyPending)
            return;

        state.Pending = null;
        if (state.State == StagingState.Phase.AwaitingIgnition)
        {
            state.State = StagingState.Phase.AwaitingPropagation;
            state.PropagationFrames = 0;
        }
    }

    // A held staging cannot be dropped: its already-activated row would stay unfired for good,
    // and firing it early only shortens a delay that exists for looks.
    private static void FlushPendingStaging(Vehicle vehicle, StagingState state)
    {
        if (state.Pending == null)
            return;
        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug($"[AFC] AutoStage fires the pending staging on '{vehicle.Id}' early, a second staging follows.");
        TickPendingStaging(vehicle, state, double.PositiveInfinity);
    }

    private static void ExecuteStaging(Vehicle vehicle, StagingState state, FlightComputer fc,
        FlightComputerBurnMode originalBurnMode, string trigger)
    {
        if (JettisonAnalysis.WouldSeparateLastControl(vehicle))
        {
            LogHelper.WarnOnce("autostage-control-loss:" + vehicle.Id,
                $"[AFC] AutoStage held on '{vehicle.Id}': the next sequence would separate the last control module. Stage it by hand if that is wanted.");
            TimedAlert.Create("AutoStage held: the next sequence would separate the control module", Color.Yellow, 4.0);
            state.State = StagingState.Phase.Monitoring;
            return;
        }

        if (DebugConfig.AutoStage)
        {
            string dvInfo = fc.Burn != null
                ? $"dV remaining = {fc.Burn.DeltaVToGoCci.Length():F1} m/s"
                : "no burn planned";
            DefaultCategory.Log.Debug($"[AFC] AutoStage staging '{vehicle.Id}' on {trigger} ({originalBurnMode} mode): {dvInfo}");
        }

        state.TriggeredMode = originalBurnMode;
        FlushPendingStaging(vehicle, state);

        PendingStaging? pending = StagingExecution.ActivateNextSequenceSplit(vehicle);

        if (originalBurnMode == FlightComputerBurnMode.Auto && fc.Burn != null)
            fc.BurnMode = FlightComputerBurnMode.Auto;

        if (pending != null)
        {
            state.Pending = pending;
            state.State = StagingState.Phase.AwaitingIgnition;

            if (pending.DecouplersPending && pending.DecouplerDelay > 0.0)
                TimedAlert.Create($"Decouple in {pending.DecouplerDelay:F1}s", Color.Yellow, pending.DecouplerDelay);
            if (pending.EnginesPending && pending.EngineDelay > 0.0)
                TimedAlert.Create($"Ignition in {pending.EngineDelay:F1}s", Color.Yellow, pending.EngineDelay);

            DefaultCategory.Log.Info(
                $"[AFC] AutoStage staging delay on '{vehicle.Id}': decouplers={pending.DecouplerDelay:F1}s ({pending.DecouplerModules?.Count ?? 0}), " +
                $"engines={pending.EngineDelay:F1}s ({pending.EngineModules?.Count ?? 0})");
        }
        else
        {
            state.State = StagingState.Phase.AwaitingPropagation;
            state.PropagationFrames = 0;
        }
    }

    // FlightComputer.ComputeControl drops BurnMode to Manual after two denied ignitions, which is
    // what a freshly staged engine looks like until its propellant state propagates.
    private static void MaintainBurnMode(StagingState state, FlightComputer fc)
    {
        if (state.TriggeredMode == FlightComputerBurnMode.Auto
            && fc.Burn != null
            && !IsBurnComplete(fc)
            && fc.BurnMode == FlightComputerBurnMode.Manual)
        {
            fc.BurnMode = FlightComputerBurnMode.Auto;
        }
    }

    // True once what is left to go no longer points along the target. Burn outlives its BurnPlan
    // entry and is saved, so a vehicle can load with a zero DeltaVTargetCci, which would pass the
    // overshoot test forever and disable both triggers for the flight.
    private static bool IsBurnComplete(FlightComputer fc)
    {
        BurnTarget? burn = fc.Burn;
        if (burn == null || burn.DeltaVTargetCci.IsNearlyZero())
            return false;
        return float3.Dot(burn.DeltaVToGoCci, burn.DeltaVTargetCci) <= 0f;
    }

    // Modules still held at unload are dead for the flight, because their row is already activated.
    internal static void FlushPendingForUnload()
    {
        foreach (KeyValuePair<Vehicle, StagingState> entry in _states)
        {
            PendingStaging? p = entry.Value.Pending;
            if (p == null || entry.Key.IsDisposed)
                continue;

            DefaultCategory.Log.Info(
                $"[AFC] Unloading with a staging still pending on '{entry.Key.Id}': firing {p.DecouplerModules?.Count ?? 0} decoupler(s) and " +
                $"{p.EngineModules?.Count ?? 0} engine(s) now, because their sequence is already marked activated.");

            TickPendingStaging(entry.Key, entry.Value, double.PositiveInfinity);
        }
    }

    // A pending staging on a disposed vehicle is abandoned rather than fired: its parts are going away with it.
    internal static void ForgetVehicle(Vehicle vehicle) => _states.Remove(vehicle);

    internal static void Reset()
    {
        _states.Clear();
        _walk.Clear();
    }
}

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
//   Monitoring -> the player stages during an Auto burn -> AwaitingActivation
//   AwaitingIgnition -> both delays elapsed -> AwaitingPropagation
//   AwaitingActivation -> the input drain switched the player's row on -> AwaitingPropagation
//   AwaitingPropagation -> new engines reported and fueled -> Monitoring, else cascade stage when armed
//
// AwaitingPropagation exists because a freshly activated engine reports propellant only after the first Rocket.UpdateRockets pass over it, and FlightComputer.ComputeControl runs before that pass in every worker frame.
// Until then, and through the delays while the spent engines are still attached, every active engine looks dry, so FlightComputer.ComputeControl drops a running Auto burn to Manual and raises AutoBurnStoppedOutOfPropellant.
// FlightComputer.RaisePendingAlerts would then end the burn later in the same frame, stop the warp, and remove the burn when another one follows.
// A staging in flight therefore takes that stop back on every frame for the burn target it staged under, see KeepAutoBurn.
// A player staging meets the same stop one frame later than AutoStage's own, because SequenceList.ActivateNextSequence runs inside InputEvents.ApplyInputEvents after that frame's activations were drained, so its activations land in the next frame's drain.
// AwaitingActivation holds the burn until they have landed. It runs for every vehicle with a running Auto burn, armed or not, and only an armed vehicle stages further on its own.
// Each vehicle runs its own machine, so an armed craft keeps staging when it is not the controlled one.
internal static class StagingDetector
{
    private static readonly Dictionary<Vehicle, StagingState> _states = new();
    // Snapshot for the walks, because the helpers a walk calls can insert a state through StateOf.
    private static readonly List<KeyValuePair<Vehicle, StagingState>> _walk = new();

    // One frame for the worker to process the new engines, plus one margin.
    private const int PropagationFrames = 2;

    // A player's activations land in the input drain of the next frame, so this only bounds one that never lands.
    internal const int ActivationFrames = 3;

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
            // Dropped rather than frozen: sim time runs on while the switch is off, so a dwell left armed would already be expired on the frame it is switched back on.
            state.PropagationFrames = 0;
            state.ResetDwell();
            state.HeldForControl = false;
            state.HeldBurn = null;
            state.ActivationEngines = null;
            if (state.State != StagingState.Phase.AwaitingIgnition)
                state.State = StagingState.Phase.Monitoring;
        }
        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug($"[AFC] AutoStage {(active ? "armed" : "disarmed")} on '{vehicle.Id}'.");
    }

    internal enum StagingRequest
    {
        // The machine evaluates the row next, and still holds it when it would separate the last control module.
        Queued,
        // The vehicle is disarmed, so the row waits until it is armed.
        Deferred,
        // A staging is still in flight and its evaluation would drop the request, so nothing is kept.
        Busy,
    }

    // Asks for one row, for a caller with a cue of its own such as a planned stage boundary or a cold ignition.
    // The answer says whether the row is on its way, so a caller that records something for the separation records it only for a request that was kept.
    internal static StagingRequest RequestStaging(Vehicle vehicle)
    {
        StagingState state = StateOf(vehicle);
        if (state.State != StagingState.Phase.Monitoring)
            return StagingRequest.Busy;
        state.Requested = true;
        return state.Active ? StagingRequest.Queued : StagingRequest.Deferred;
    }

    internal static bool HasState(Vehicle vehicle) => _states.ContainsKey(vehicle);

    internal static int ActivationsOf(Vehicle vehicle)
        => _states.TryGetValue(vehicle, out StagingState? state) ? state.Activations : 0;

    internal static bool IsHeldForControl(Vehicle vehicle)
        => _states.TryGetValue(vehicle, out StagingState? state) && state.HeldForControl;

    internal static StagingState StateOf(Vehicle vehicle)
    {
        if (!_states.TryGetValue(vehicle, out StagingState? state))
        {
            state = new StagingState();
            _states[vehicle] = state;
        }
        return state;
    }

    // Both values are edges the applied worker results destroy, and ApplyInputEvents drains in between, so an engine the player shut down would otherwise look like a burnout.
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

            // First, armed or not: the row is already marked activated and stock skips activated rows forever, so a module dropped here could never fire again.
            TickPendingStaging(vehicle, state, now);

            bool sampled = state.Sampled;
            state.Sampled = false;
            // An unarmed vehicle only finishes a staging in flight, which a player staging during an Auto burn can start.
            if (state.Active ? !sampled : state.State == StagingState.Phase.Monitoring)
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

        // Only the spent-jettison trigger needs the full tally; for a quenched solid motor the per-core answer runs a fixed-point pressure solve.
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
                    ExecuteStaging(vehicle, state, fc, RunningAutoBurn(state, fc), "requested");
                }
                // The stock stop counts as a burnout edge too, because the worker can see the engines dry before this frame's sample did.
                else if (!hasPropellant
                    && (state.SampledHadPropellant || fc.AutoBurnStoppedOutOfPropellant)
                    && !IsBurnComplete(fc)
                    && StagingHelpers.HasNextEngineSequence(vehicle))
                {
                    state.SpentJettisonSince = double.NaN;
                    ExecuteStaging(vehicle, state, fc, RunningAutoBurn(state, fc), "burnout");
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
                KeepAutoBurn(vehicle, state, fc);
                break;

            case StagingState.Phase.AwaitingActivation:
                state.Requested = false;
                if (ActivationLanded(vehicle, state.ActivationEngines))
                {
                    state.ActivationEngines = null;
                    BeginPropagation(state, now);
                    EvaluatePropagation(vehicle, state, fc, hasPropellant, now);
                }
                else if (++state.ActivationFrames > ActivationFrames)
                {
                    // The drain did not apply the row, so there is nothing to wait for and a stop stands.
                    state.ActivationEngines = null;
                    state.HeldBurn = null;
                    state.State = StagingState.Phase.Monitoring;
                }
                else
                {
                    KeepAutoBurn(vehicle, state, fc);
                }
                break;

            case StagingState.Phase.AwaitingPropagation:
                EvaluatePropagation(vehicle, state, fc, hasPropellant, now);
                break;
        }
    }

    private static void BeginPropagation(StagingState state, double now)
    {
        state.State = StagingState.Phase.AwaitingPropagation;
        state.PropagationFrames = 0;
        state.PropagationCountedAt = now;
    }

    private static void EvaluatePropagation(Vehicle vehicle, StagingState state, FlightComputer fc, bool hasPropellant, double now)
    {
        state.Requested = false;
        // A paused frame runs no worker step, so it neither counts towards the wait nor lets the new engines report.
        if (now > state.PropagationCountedAt)
        {
            state.PropagationFrames++;
            state.PropagationCountedAt = now;
        }
        bool waited = state.PropagationFrames >= PropagationFrames;

        if (hasPropellant && (waited || !StagingHelpers.HasUnreportedActiveEngine(vehicle)))
        {
            // This frame's stop came from the engine that has only just been fed.
            KeepAutoBurn(vehicle, state, fc);
            state.HeldBurn = null;
            state.State = StagingState.Phase.Monitoring;
        }
        else if (!hasPropellant && waited)
        {
            if (state.Active && !IsBurnComplete(fc) && StagingHelpers.HasNextEngineSequence(vehicle))
            {
                ExecuteStaging(vehicle, state, fc, state.HeldBurn, "cascade");
            }
            else
            {
                // Nothing left to stage, or the vehicle is not armed to stage it, so the stop stands and stock ends the burn.
                state.HeldBurn = null;
                state.State = StagingState.Phase.Monitoring;
            }
        }
        else
        {
            KeepAutoBurn(vehicle, state, fc);
        }
    }

    // Landed once every engine reads active or has left the vehicle with a decoupled part.
    private static bool ActivationLanded(Vehicle vehicle, List<EngineController>? engines)
    {
        if (engines == null)
            return true;
        foreach (EngineController engine in engines)
        {
            if (!engine.IsActive && engine.Parent.FullPart.Tree == vehicle.Parts)
                return false;
        }
        return true;
    }

    // SequenceList.ActivateNextSequence switched these engines on while an Auto burn ran. AutoStage's own staging does not call it, so this is the player's staging key.
    // The activation only queues, so the burn is held from here on, through the frame the activation lands in and the frame the stale stop of the new engines arrives in.
    internal static void OnPlayerStaged(Vehicle vehicle, List<EngineController> engines)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (vehicle.IsDisposed || fc.BurnMode != FlightComputerBurnMode.Auto || fc.Burn == null)
            return;
        StagingState state = StateOf(vehicle);
        // A staging waiting out its delays already holds the burn on every frame, and its propagation follows.
        if (state.State == StagingState.Phase.AwaitingIgnition)
            return;
        state.HeldBurn = fc.Burn;
        state.ActivationEngines = engines;
        state.ActivationFrames = 0;
        state.State = StagingState.Phase.AwaitingActivation;
        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug(
                $"[AFC] AutoStage holds the Auto burn on '{vehicle.Id}' across a player staging that lights {engines.Count} engine(s).");
    }

    // Stages while the vehicle is still under thrust, when the next sequence would shed nothing but burnt-out engines.
    // Requires thrust to remain afterwards, so a stack that is simply running out falls to the all-dry trigger and keeps its cascade behaviour.
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

        ExecuteStaging(vehicle, state, fc, RunningAutoBurn(state, fc), "spent drop");
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
            BeginPropagation(state, Universe.GetElapsedSeconds());
    }

    // A held staging cannot be dropped: its already-activated row would stay unfired for good, and firing it early only shortens a delay that exists for looks.
    private static void FlushPendingStaging(Vehicle vehicle, StagingState state)
    {
        if (state.Pending == null)
            return;
        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug($"[AFC] AutoStage fires the pending staging on '{vehicle.Id}' early, a second staging follows.");
        TickPendingStaging(vehicle, state, double.PositiveInfinity);
    }

    // heldBurn is the Auto burn target the staging keeps alive, null when no Auto burn was running.
    private static void ExecuteStaging(Vehicle vehicle, StagingState state, FlightComputer fc,
        BurnTarget? heldBurn, string trigger)
    {
        if (JettisonAnalysis.WouldSeparateLastControl(vehicle))
        {
            // Once per row: the triggers keep asking every frame while the row stands.
            if (!state.HeldForControl)
            {
                state.HeldForControl = true;
                DefaultCategory.Log.Warning(
                    $"[AFC] AutoStage held on '{vehicle.Id}': the next sequence would separate the last control module. Stage it by hand if that is wanted.");
                TimedAlert.Create("AutoStage held: the next sequence would separate the control module", Color.Yellow, 4.0);
            }
            // The stop stands, so stock ends the burn it cannot continue.
            state.HeldBurn = null;
            state.State = StagingState.Phase.Monitoring;
            return;
        }
        state.HeldForControl = false;

        if (DebugConfig.AutoStage)
        {
            string dvInfo = fc.Burn != null
                ? $"dV remaining = {fc.Burn.DeltaVToGoCci.Length():F1} m/s"
                : "no burn planned";
            string mode = heldBurn != null ? "Auto burn" : "no Auto burn";
            DefaultCategory.Log.Debug($"[AFC] AutoStage staging '{vehicle.Id}' on {trigger} ({mode}): {dvInfo}");
        }

        state.HeldBurn = heldBurn;
        FlushPendingStaging(vehicle, state);

        PendingStaging? pending = StagingExecution.ActivateNextSequenceSplit(vehicle);
        state.Activations++;

        KeepAutoBurn(vehicle, state, fc);

        if (pending != null)
        {
            state.Pending = pending;
            state.ActivationEngines = null;
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
            state.ActivationEngines = null;
            BeginPropagation(state, Universe.GetElapsedSeconds());
        }
    }

    // The Auto burn target running when the trigger fired, or null.
    // The mode is sampled before the worker results are applied, because a stop in this frame's results has already dropped it to Manual.
    private static BurnTarget? RunningAutoBurn(StagingState state, FlightComputer fc)
        => state.SampledBurnMode == FlightComputerBurnMode.Auto || fc.AutoBurnStoppedOutOfPropellant ? fc.Burn : null;

    // FlightComputer.ComputeControl drops an Auto burn to Manual when every active engine reports no propellant, raising AutoBurnStoppedOutOfPropellant, and after two denied ignitions.
    // Both are what a staging looks like until the new engines are fed, so while the held target is still the loaded one the stop is taken back before FlightComputer.RaisePendingAlerts reads it, and Auto is given back although the engines still read dry.
    // The hold ends when stock completes the burn or loads another one, which leaves the burn to stock.
    private static void KeepAutoBurn(Vehicle vehicle, StagingState state, FlightComputer fc)
    {
        BurnTarget? held = state.HeldBurn;
        if (held == null)
            return;
        if (!ReferenceEquals(fc.Burn, held) || IsBurnComplete(fc))
        {
            state.HeldBurn = null;
            return;
        }
        bool takesBack = fc.BurnMode == FlightComputerBurnMode.Manual || fc.AutoBurnStoppedOutOfPropellant;
        if (fc.BurnMode == FlightComputerBurnMode.Manual && !StockBurnMode.GiveBackAuto(vehicle, held, takesStopBack: true))
        {
            // A vehicle without control cannot fly the burn, so stock ends it.
            state.HeldBurn = null;
            return;
        }
        fc.AutoBurnStoppedOutOfPropellant = false;
        // The stop came with a denied ignition. FlightComputer.ComputeControl drops an Auto burn to Manual without any flag when the next ignition is denied too,
        // so the denial is cleared with the stop. A next dry tick then raises the stop again, and when the hold lets it stand, FlightComputer.RaisePendingAlerts ends the burn with its alert.
        if (takesBack)
            held.LastIgnitionDenied = false;
    }

    // True in the frame stock reports the burn complete, and after it for a burn that stock completes by the delta-V reversal, because what is left to go then no longer points along the target.
    // A fixed-duration burn reads complete only in that frame, because FlightComputer.RaisePendingAlerts clears the flag and the reversal does not apply to it.
    // Burn outlives its BurnPlan entry and is saved, so a vehicle can load with a zero DeltaVTargetCci, which would pass the overshoot test forever and disable both triggers for the flight.
    private static bool IsBurnComplete(FlightComputer fc)
    {
        BurnTarget? burn = fc.Burn;
        if (burn == null)
            return false;
        if (fc.AutoBurnCompleted)
            return true;
        if (burn.DeltaVTargetCci.IsNearlyZero())
            return false;
        return float3.Dot(burn.DeltaVToGoCci, burn.DeltaVTargetCci) <= 0f;
    }

    // Modules still held at unload are dead for the flight, because their row is already activated.
    internal static void FlushPendingForUnload()
    {
        _walk.Clear();
        _walk.AddRange(_states);
        foreach (KeyValuePair<Vehicle, StagingState> entry in _walk)
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

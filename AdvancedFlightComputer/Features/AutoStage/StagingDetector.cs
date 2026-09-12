using System.Diagnostics;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

// Triggers staging once per frame, around the vehicle-solver apply.
//
//   Monitoring -> the active engines lose propellant -> stage
//   Monitoring -> the next row would shed only spent engines -> stage
//   AwaitingIgnition -> both delays elapsed -> AwaitingPropagation
//   AwaitingPropagation -> new engines fueled -> Monitoring, else cascade stage
//
// AwaitingPropagation exists because a freshly activated engine reports propellant only a tick
// after activation, and BurnMode is held at Auto across that window so the worker cannot abort
// the burn.
//
// The state machine follows the controlled vehicle. A pending staging does not: it ticks against
// the vehicle it was committed on, whatever is controlled.
internal static class StagingDetector
{
    private enum State { Monitoring, AwaitingIgnition, AwaitingPropagation }

    // The player's switch, the AUTOSTAGE gauge button. Not persisted, like the stock RCS toggle.
    internal static bool Active { get; set; }

    private static State _state = State.Monitoring;
    private static int _propagationFrames;
    private enum SpentDropBlocker
    {
        None,
        Unreported,
        NoJettison,
        NothingSpent,
        InactiveInside,
        FueledInside,
        BrokenInside,
        NothingThrusting,
        BurnComplete,
        CarriesPropellant,
    }

    private static double _spentJettisonSince = double.NaN;
    // Unreported rather than None, because None is the armed state and would suppress the first arm line.
    private static SpentDropBlocker _spentJettisonBlocker = SpentDropBlocker.Unreported;
    private static FlightComputerBurnMode _triggeredMode;
    private static PendingStaging? _pendingStaging;
    private static Vehicle? _currentVehicle;

    // Taken by Sample before the solver results reach the universe and consumed by Evaluate
    // afterwards. The vehicle doubles as the validity flag, so one sample is never evaluated twice.
    private static Vehicle? _sampledVehicle;
    private static FlightComputerBurnMode _sampledBurnMode;
    private static bool _sampledHadPropellant;

    // One frame for the worker to process the new engines, plus one margin.
    private const int PropagationFrames = 2;

    // Level-triggered, so it needs a dwell. Sim time, because Evaluate also runs while paused.
    private const double SpentJettisonDwellSeconds = 0.25;

    // Both values are edges the applied worker results destroy, and ApplyInputEvents drains in
    // between, so an engine the player shut down would otherwise look like a burnout.
    internal static void Sample()
    {
        Vehicle? vehicle = Program.ControlledVehicle;
        _sampledVehicle = vehicle;

        if (vehicle == null || !Active)
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
        // First, whatever is controlled or armed: the row is already marked activated and stock
        // skips activated rows forever, so a module dropped here could never fire again.
        Vehicle? justCompletedOn = TickPendingStaging(Universe.GetElapsedSeconds());

        Vehicle? vehicle = _sampledVehicle;
        _sampledVehicle = null;
        if (vehicle == null)
        {
            ResetDwell();
            return;
        }

        if (!Active)
        {
            // Dropped rather than frozen: sim time runs on while the switch is off, so a dwell left
            // armed would already be expired on the frame it is switched back on.
            _propagationFrames = 0;
            ResetDwell();
            if (_state != State.AwaitingIgnition)
                _state = State.Monitoring;
            return;
        }

        if (_currentVehicle != vehicle)
        {
            _propagationFrames = 0;
            ResetDwell();
            _currentVehicle = vehicle;
            // Resume rather than reset, so returning mid-delay does not start a second staging.
            if (_pendingStaging != null && _pendingStaging.Vehicle == vehicle)
                _state = State.AwaitingIgnition;
            else if (justCompletedOn == vehicle)
                _state = State.AwaitingPropagation;
            else
                _state = State.Monitoring;
        }

        FlightComputer fc = vehicle.FlightComputer;
        IReadOnlySet<Part>? jettison = _state == State.Monitoring && StagingConfig.DropSpentStages
            ? JettisonAnalysis.GetPendingJettison(vehicle)
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
                else if (StagingConfig.DropSpentStages)
                {
                    TickSpentJettison(vehicle, fc, _sampledBurnMode, in survey, jettison);
                }
                else
                {
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
                    if (!IsBurnComplete(fc) && StagingHelpers.HasNextEngineSequence(vehicle))
                        ExecuteStaging(vehicle, fc, _triggeredMode);
                    else
                        _state = State.Monitoring;
                }
                break;
        }

#if DEBUG
        if (DebugConfig.Performance)
            PerfTracker.Record("StagingDetector.Evaluate", Stopwatch.GetTimestamp() - perfStart);
#endif
    }

    private static void ResetDwell()
    {
        _spentJettisonSince = double.NaN;
        _spentJettisonBlocker = SpentDropBlocker.Unreported;
    }

    // Stages while the vehicle is still under thrust, when the next sequence would shed nothing
    // but burnt-out engines. Requires thrust to remain afterwards, so a stack that is simply
    // running out falls to the all-dry trigger and keeps its cascade behaviour.
    private static void TickSpentJettison(Vehicle vehicle, FlightComputer fc,
        FlightComputerBurnMode mode, in StagingHelpers.EngineSurvey survey, IReadOnlySet<Part>? jettison)
    {
        SpentDropBlocker blocker =
            jettison == null ? SpentDropBlocker.NoJettison
            : survey.SpentInside == 0 ? SpentDropBlocker.NothingSpent
            : survey.InactiveInside > 0 ? SpentDropBlocker.InactiveInside
            : survey.FueledInside > 0 ? SpentDropBlocker.FueledInside
            : survey.BrokenInside > 0 ? SpentDropBlocker.BrokenInside
            : !survey.ThrustingOutside ? SpentDropBlocker.NothingThrusting
            : IsBurnComplete(fc) ? SpentDropBlocker.BurnComplete
            : SpentDropBlocker.None;

        if (blocker != SpentDropBlocker.None)
        {
            ReportSpentJettison(vehicle, blocker, in survey, measured: jettison != null);
            _spentJettisonSince = double.NaN;
            return;
        }

        double now = Universe.GetElapsedSeconds();
        if (double.IsNaN(_spentJettisonSince))
        {
            ReportSpentJettison(vehicle, SpentDropBlocker.None, in survey, measured: true);
            _spentJettisonSince = now;
            return;
        }
        if (now - _spentJettisonSince < SpentJettisonDwellSeconds)
            return;

        // Last gate, after the dwell, because it walks the jettisoned parts' tanks.
        _spentJettisonSince = double.NaN;
        if (JettisonAnalysis.CarriesOffUsablePropellant(vehicle, jettison!, out string? carried))
        {
            ReportSpentJettison(vehicle, SpentDropBlocker.CarriesPropellant, in survey, measured: true, carried);
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

        ExecuteStaging(vehicle, fc, mode);
    }

    // Once per change, because silence makes "rode along" and "correctly refused" look alike.
    private static void ReportSpentJettison(Vehicle vehicle, SpentDropBlocker blocker,
        in StagingHelpers.EngineSurvey survey, bool measured, string? carried = null)
    {
        if (!DebugConfig.AutoStage || blocker == _spentJettisonBlocker)
            return;
        _spentJettisonBlocker = blocker;

        int next = vehicle.Parts.SequenceList.GetNextSequenceNumber();
        string detail = measured
            ? $"next sequence {next}, inside: {survey.SpentInside} spent / {survey.FueledInside} fueled / " +
              $"{survey.BrokenInside} broken / {survey.InactiveInside} not run, " +
              $"outside: {survey.FueledOutside} fueled, thrusting={survey.ThrustingOutside}"
            : $"next sequence {next}, engines not surveyed";
        string reason = blocker switch
        {
            SpentDropBlocker.NoJettison => "no pending jettison to judge",
            SpentDropBlocker.NothingSpent => "nothing spent among the parts that would be shed",
            SpentDropBlocker.InactiveInside => $"{survey.InactiveInside} engine(s) to be shed have not run",
            SpentDropBlocker.FueledInside => $"{survey.FueledInside} engine(s) to be shed still have propellant",
            SpentDropBlocker.BrokenInside => $"{survey.BrokenInside} engine(s) to be shed have an unresolved motor stack",
            SpentDropBlocker.NothingThrusting => "nothing staying aboard is thrusting",
            SpentDropBlocker.BurnComplete => "the planned burn is already complete",
            SpentDropBlocker.CarriesPropellant => carried ?? "the shed parts still feed the vehicle",
            _ => "",
        };

        DefaultCategory.Log.Debug(blocker == SpentDropBlocker.None
            ? $"[AFC] Spent-stage drop armed on '{vehicle.Id}' ({detail})."
            : $"[AFC] Spent-stage drop held on '{vehicle.Id}': {reason} ({detail}).");
    }

    // Fires whichever pending parts have reached their deadline, on the vehicle the staging was
    // committed against. Returns that vehicle once nothing is pending on it any more.
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
        if (_state == State.AwaitingIgnition && _currentVehicle == p.Vehicle)
        {
            _state = State.AwaitingPropagation;
            _propagationFrames = 0;
        }
        return p.Vehicle;
    }

    // There is one pending slot. Dropping a held staging would leave its already-activated row
    // unfired for good, and firing it early only shortens a delay that exists for looks.
    private static void FlushPendingStaging()
    {
        if (_pendingStaging == null)
            return;
        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug(
                $"[AFC] AutoStage fires the pending staging on '{_pendingStaging.Vehicle.Id}' early, a second staging needs the slot.");
        TickPendingStaging(double.PositiveInfinity);
    }

    private static void ExecuteStaging(Vehicle vehicle, FlightComputer fc, FlightComputerBurnMode originalBurnMode)
    {
        if (DebugConfig.AutoStage)
        {
            string dvInfo = fc.Burn != null
                ? $"dV remaining = {fc.Burn.DeltaVToGoCci.Length():F1} m/s"
                : "no burn planned";
            DefaultCategory.Log.Debug($"[AFC] AutoStage staging '{vehicle.Id}' ({originalBurnMode} mode): {dvInfo}");
        }

        _triggeredMode = originalBurnMode;
        FlushPendingStaging();

        PendingStaging? pending = StagingExecution.ActivateNextSequenceSplit(vehicle);

        if (originalBurnMode == FlightComputerBurnMode.Auto && fc.Burn != null)
            fc.BurnMode = FlightComputerBurnMode.Auto;

        if (pending != null)
        {
            _pendingStaging = pending;
            _state = State.AwaitingIgnition;

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
            _state = State.AwaitingPropagation;
            _propagationFrames = 0;
        }
    }

    // FlightComputer.ComputeControl drops BurnMode to Manual after two denied ignitions, which is
    // what a freshly staged engine looks like until its propellant state propagates.
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
        PendingStaging? p = _pendingStaging;
        if (p == null || p.Vehicle.IsDisposed)
            return;

        DefaultCategory.Log.Info(
            $"[AFC] Unloading with a staging still pending on '{p.Vehicle.Id}': firing {p.DecouplerModules?.Count ?? 0} decoupler(s) and " +
            $"{p.EngineModules?.Count ?? 0} engine(s) now, because their sequence is already marked activated.");

        TickPendingStaging(double.PositiveInfinity);
    }

    // A pending staging on a disposed vehicle is abandoned rather than fired: its parts are going away with it.
    internal static void ForgetVehicle(Vehicle vehicle)
    {
        if (_currentVehicle == vehicle)
        {
            _currentVehicle = null;
            _state = State.Monitoring;
            _propagationFrames = 0;
            ResetDwell();
        }
        if (_sampledVehicle == vehicle)
            _sampledVehicle = null;
        if (_pendingStaging?.Vehicle == vehicle)
            _pendingStaging = null;
    }

    internal static void Reset()
    {
        Active = false;
        _state = State.Monitoring;
        _propagationFrames = 0;
        ResetDwell();
        _triggeredMode = FlightComputerBurnMode.Manual;
        _pendingStaging = null;
        _currentVehicle = null;
        _sampledVehicle = null;
        _sampledBurnMode = FlightComputerBurnMode.Manual;
        _sampledHadPropellant = false;
        JettisonAnalysis.Reset();
    }
}

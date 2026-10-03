using AdvancedFlightComputer.Core;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

internal static class StagingHelpers
{
    // One frame's tally.
    // The outside counters only count active engines, because they answer "is the vehicle still under thrust".
    // The inside counters count every engine a pending jettison would carry away, active or not, because they answer "is it safe to let these go".
    internal struct EngineSurvey
    {
        public int FueledOutside;
        public bool ThrustingOutside;

        public int FueledInside;
        public int SpentInside;
        public int BrokenInside;
        public int InactiveInside;

        public bool AnyFueled;
    }

    public static bool HasActiveEngineWithPropellant(Vehicle vehicle)
    {
        // The staging hooks run in Universe.ApplyVehicleSolvers, before Program.PrepareFrame flushes dirty part trees, and IsFueled reads SolidMotor.Stack.
        vehicle.Parts.EnsureDerived(DerivedData.SolidMotorStacks);
        ReadOnlySpan<MoleState> moleStates = vehicle.Parts.Moles.States;
        ReadOnlySpan<RocketCoreState> coreStates = vehicle.Parts.RocketCores.States;
        Span<EngineController> engines = vehicle.Parts.Modules.Get<EngineController>();
        for (int i = 0; i < engines.Length; i++)
        {
            if (engines[i].IsActive && VehiclePropellant.IsFueled(engines[i], moleStates, coreStates, out _, out _))
                return true;
        }
        return false;
    }

    // Pass the parts a pending jettison would shed to learn whether it drops only spent engines, or null to only count thrust.
    public static EngineSurvey SurveyActiveEngines(Vehicle vehicle, IReadOnlySet<Part>? jettisonSet)
    {
        vehicle.Parts.EnsureDerived(DerivedData.SolidMotorStacks);
        EngineSurvey survey = default;
        ReadOnlySpan<MoleState> moleStates = vehicle.Parts.Moles.States;
        ReadOnlySpan<RocketCoreState> coreStates = vehicle.Parts.RocketCores.States;
        Span<EngineController> engines = vehicle.Parts.Modules.Get<EngineController>();
        for (int i = 0; i < engines.Length; i++)
        {
            EngineController engine = engines[i];
            bool inside = jettisonSet != null && jettisonSet.Contains(engine.Parent.FullPart);

            // Staging only queues the activation, so a booster staged this frame is still inactive and full.
            // An engine that has not run has not proven it is spent, and is counted apart so it can never license a drop.
            if (!engine.IsActive)
            {
                if (inside)
                    survey.InactiveInside++;
                continue;
            }

            bool fueled = VehiclePropellant.IsFueled(engine, moleStates, coreStates, out bool burning, out bool broken);
            if (fueled)
                survey.AnyFueled = true;

            if (inside)
            {
                if (broken)
                    survey.BrokenInside++;
                else if (fueled)
                    survey.FueledInside++;
                else
                    survey.SpentInside++;
            }
            else if (fueled)
            {
                survey.FueledOutside++;
                survey.ThrustingOutside |= burning;
            }
        }
        return survey;
    }

    // Rocket.UpdateRockets sets EngineControllerState.WasActive on its first pass over a newly activated engine, and until then the committed IsPropellantAvailable is the false of an inactive engine.
    // FlightComputer.ComputeControl runs before that pass in every worker frame, so while this holds, its next tick can still find every active engine dry.
    public static bool HasUnreportedActiveEngine(Vehicle vehicle)
    {
        if (!ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
                .TryGetFrom(vehicle.Parts.States, out var engines))
            return false;
        foreach (var engine in engines.ModulesAndStates)
        {
            if (engine.Module.IsActive && !engine.State.WasActive)
                return true;
        }
        return false;
    }

    // The engines SequenceList.ActivateNextSequence is about to switch on, read before it marks the row activated.
    // Stock picks the first unactivated row with parts and walks each part's modules in that row through Part.ActivateSubtreeInStage.
    public static List<EngineController>? EnginesLitByNextRow(SequenceList sequenceList)
    {
        foreach (Sequence sequence in sequenceList.Sequences)
        {
            if (sequence.Activated || sequence.Parts.IsEmpty)
                continue;
            List<EngineController>? engines = null;
            ReadOnlySpan<Part> parts = sequence.Parts;
            for (int i = 0; i < parts.Length; i++)
            {
                foreach (ISequenced module in parts[i].InSequence(sequence.Number))
                {
                    if (module is EngineController { IsActive: false } engine)
                        (engines ??= new List<EngineController>()).Add(engine);
                }
            }
            return engines;
        }
        return null;
    }

    // Bumped on every sequence activation and cache reset, for every vehicle, so the per-vehicle answers below refresh.
    // Deliberately never reset: every consumer starts at -1.
    private static int _sequenceGeneration;

    public static int SequenceGeneration => _sequenceGeneration;

    // Interlocked, because SequencePerformanceList.Recompute can reset sequence caches on a worker thread, in the editor and in flight.
    public static void InvalidateSequenceCache() => Interlocked.Increment(ref _sequenceGeneration);

    // Queried per frame by the gauge button, but it only changes on sequence activation.
    public static bool HasNextEngineSequence(Vehicle vehicle)
    {
        StagingState state = StagingDetector.StateOf(vehicle);
        if (state.NextEngineGeneration != _sequenceGeneration)
        {
            state.NextEngineGeneration = _sequenceGeneration;
            state.NextEngineSequence = ComputeHasNextEngineSequence(vehicle);
        }
        return state.NextEngineSequence;
    }

    private static bool ComputeHasNextEngineSequence(Vehicle vehicle)
    {
        foreach (Sequence sequence in vehicle.Parts.SequenceList.Sequences)
        {
            if (!sequence.Activated && SequencedModules.LightsEngine(sequence))
                return true;
        }
        return false;
    }
}

using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

internal static class StagingHelpers
{
    // One frame's tally. The outside counters only count active engines, because they answer "is
    // the vehicle still under thrust". The inside counters count every engine a pending jettison
    // would carry away, active or not, because they answer "is it safe to let these go".
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
        ReadOnlySpan<MoleState> moleStates = vehicle.Parts.Moles.States;
        ReadOnlySpan<RocketCoreState> coreStates = vehicle.Parts.RocketCores.States;
        Span<EngineController> engines = vehicle.Parts.Modules.Get<EngineController>();
        for (int i = 0; i < engines.Length; i++)
        {
            if (engines[i].IsActive && IsFueled(engines[i], moleStates, coreStates, out _, out _))
                return true;
        }
        return false;
    }

    // Pass the parts a pending jettison would shed to learn whether it drops only spent engines,
    // or null to only count thrust.
    public static EngineSurvey SurveyActiveEngines(Vehicle vehicle, IReadOnlySet<Part>? jettisonSet)
    {
        EngineSurvey survey = default;
        ReadOnlySpan<MoleState> moleStates = vehicle.Parts.Moles.States;
        ReadOnlySpan<RocketCoreState> coreStates = vehicle.Parts.RocketCores.States;
        Span<EngineController> engines = vehicle.Parts.Modules.Get<EngineController>();
        for (int i = 0; i < engines.Length; i++)
        {
            EngineController engine = engines[i];
            bool inside = jettisonSet != null && jettisonSet.Contains(engine.Parent.FullPart);

            // Staging only queues the activation, so a booster staged this frame is still inactive
            // and full. An engine that has not run has not proven it is spent, and is counted
            // apart so it can never license a drop.
            if (!engine.IsActive)
            {
                if (inside)
                    survey.InactiveInside++;
                continue;
            }

            bool fueled = IsFueled(engine, moleStates, coreStates, out bool burning, out bool broken);
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

    private static bool IsFueled(EngineController engine,
        ReadOnlySpan<MoleState> moleStates, ReadOnlySpan<RocketCoreState> coreStates,
        out bool burning, out bool broken)
    {
        bool fueled = false;
        burning = false;
        broken = false;
        foreach (RocketCore core in engine.Cores)
        {
            // Mirrors Rocket.UpdateRockets: a core burns above zero throttle. A lit solid motor
            // counts its remaining grain as propellant, while a quenched one falls back to the
            // equilibrium-pressure check and reads as spent.
            bool isBurning = coreStates[core.StatesIdx].Throttle > 0f;
            burning |= isBurning;
            fueled |= core.ComputePropellantAvailable(moleStates, isBurning);

            // A motor whose stack did not resolve reports no propellant for the life of the vehicle.
            broken |= core is SolidMotor { Stack.IsValid: false };
        }
        return fueled;
    }

    // Bumped on every sequence activation and cache reset, so the per-vehicle answers below refresh.
    private static int _sequenceGeneration;

    public static int SequenceGeneration => _sequenceGeneration;

    public static void InvalidateSequenceCache() => _sequenceGeneration++;

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

using System;
using System.Collections.Generic;
using KSA;

namespace AutoStage;

static class StagingHelpers
{
    /// <summary>
    /// Tally of the vehicle's engines for one frame.
    ///
    /// The "outside" numbers only count active engines, because they answer
    /// "is the vehicle still under thrust". The "inside" numbers count every
    /// engine on a part a pending jettison would carry away, active or not,
    /// because they answer "is it safe to let these go".
    /// </summary>
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
            if (!engines[i].IsActive) continue;
            if (IsFueled(engines[i], moleStates, coreStates, out _, out _))
                return true;
        }
        return false;
    }

    /// <summary>
    /// One pass over the engines. Pass the parts a pending jettison would shed
    /// as <paramref name="jettisonSet"/> to find out whether that jettison
    /// would drop only spent engines; pass null to just count thrust.
    /// </summary>
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

            if (!engine.IsActive)
            {
                // An engine that is not running has not proven it is spent, and
                // the propellant answer cannot prove it either: staging only
                // queues the activation, so a booster that was just staged is
                // still IsActive == false on the frame its sequence fires.
                // Counted apart from "spent" so it can never license a drop,
                // and answered before IsFueled because this is the state that
                // holds the drop indefinitely, and for a quenched solid motor
                // the propellant answer runs a fixed-point pressure solve.
                if (inside) survey.InactiveInside++;
                continue;
            }

            bool fueled = IsFueled(engine, moleStates, coreStates, out bool burning, out bool broken);

            if (fueled)
                survey.AnyFueled = true;

            if (inside)
            {
                if (broken) survey.BrokenInside++;
                else if (fueled) survey.FueledInside++;
                else survey.SpentInside++;
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
            // isBurning mirrors Rocket.UpdateRockets: a core burns when its
            // throttle is above zero. A lit SolidMotor then counts remaining
            // grain as propellant, while a quenched motor falls back to the
            // equilibrium-pressure check and reads as spent, so its unburnable
            // grain sliver does not keep a burnt booster flagged active.
            bool isBurning = coreStates[core.StatesIdx].Throttle > 0f;
            burning |= isBurning;
            fueled |= core.ComputePropellantAvailable(moleStates, isBurning);

            // A motor whose stack did not resolve (several causes, among them a
            // grain shared with another motor and a nozzle that could not be
            // sized) reports no propellant for the life of the vehicle. Reported
            // separately so the staging triggers can tell a broken booster from
            // a burnt-out one without distorting the thrust answer.
            broken |= core is SolidMotor { Stack.IsValid: false };
        }
        return fueled;
    }

    // HasNextEngineSequence is queried per frame by the gauge, but only
    // changes on sequence activation.
    private static Vehicle? _cachedVehicle;
    private static bool _cachedHasNextEngineSequence;
    private static int _cachedGeneration = -1;
    private static int _sequenceGeneration;

    public static int SequenceGeneration => _sequenceGeneration;

    public static void InvalidateSequenceCache() => _sequenceGeneration++;

    public static bool HasNextEngineSequence(Vehicle vehicle)
    {
        if (_cachedVehicle == vehicle && _cachedGeneration == _sequenceGeneration)
            return _cachedHasNextEngineSequence;

        _cachedVehicle = vehicle;
        _cachedGeneration = _sequenceGeneration;
        _cachedHasNextEngineSequence = ComputeHasNextEngineSequence(vehicle);
        return _cachedHasNextEngineSequence;
    }

    // Per module: a part present only for its decoupler must not make the row
    // count as an engine sequence.
    private static bool ComputeHasNextEngineSequence(Vehicle vehicle)
    {
        foreach (Sequence sequence in vehicle.Parts.SequenceList.Sequences)
        {
            if (sequence.Activated) continue;
            if (SequencedModules.LightsEngine(sequence)) return true;
        }
        return false;
    }

    internal static void ForgetVehicle(Vehicle vehicle)
    {
        if (_cachedVehicle != vehicle)
            return;
        _cachedVehicle = null;
        _cachedGeneration = -1;
        _cachedHasNextEngineSequence = false;
    }

    internal static void Reset()
    {
        _cachedVehicle = null;
        _cachedGeneration = -1;
        _sequenceGeneration = 0;
        _cachedHasNextEngineSequence = false;
    }
}

using System;
using System.Runtime.CompilerServices;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// The full-throttle phases that stock propagates a planned burn of a craft with, known before the burn exists.
/// PhysicsBubble.UpdateStagedBurn fills FlightComputer.StagedBurn in two ways. While FlightComputer.ActiveEnginePerformanceMax has no thrust, it fills on every update with SequencePerformanceList.FillStagedBurn(fromActive: false), which starts at the first sequence that is not activated. While it has thrust, it fills only on a replan of the burn plan, with fromActive: true after RecomputeForFlight, which starts at the active sequence.
/// A craft without burns gets no such replan, because BurnPlan.UpdateBurnPlan clears FlightPlansOutOfDate when there is no burn to plan. So before a burn exists, FlightComputer.GetFullThrottlePhases can still give the list of the other branch, the stages after the active one, or the masses of an earlier replan. The replan that BurnPlan.AddBurnFromFlightComputerOnly asks for then propagates the new burn from the active sequence with the current masses, which is another trajectory.
/// This class gives that second list at any time. When FlightComputer.ActiveEnginePerformanceMax has thrust, which is the condition stock branches on, it runs the stock drain simulation on a SequencePerformanceList of its own, so the result of the game worker is not changed, and fills from the active sequence. Otherwise the stock list is already the one a new burn gets, and it is returned as it is.
/// </summary>
internal static class PlannedBurnPhases
{
    // FlightComputer.SetStagedBurn keeps at most 16 phases.
    private const int MaxPhases = 16;
    // The phases follow the mass of the craft, which changes on every frame of a burn. The list is kept until the mass has moved by this fraction, so a preview that compares the phases is not built again on every frame while the engines fire. The fraction changes the length of a burn by the same fraction, which is far below what a preview shows.
    private const double MassDriftFraction = 0.002;

    private sealed class VehicleCache
    {
        public readonly FiniteBurnPhase[] Phases = new FiniteBurnPhase[MaxPhases];
        public readonly FiniteBurnPhase[] Single = new FiniteBurnPhase[1];
        public int Count;
        public PartTree? Tree;
        public SequencePerformanceList? Performance;
        public int Structure;
        public double Mass;
    }

    private static readonly ConditionalWeakTable<Vehicle, VehicleCache> _caches = new();

    /// <summary>How often the drain simulation ran, for a check that an unchanged craft costs none.</summary>
    internal static int Recomputes { get; private set; }

    // The span covers a per-vehicle buffer, so read it on the main thread and do not keep it across calls for that craft.
    public static ReadOnlySpan<FiniteBurnPhase> For(Vehicle vehicle)
    {
        FlightComputer flightComputer = vehicle.FlightComputer;
        PartTree? tree = vehicle.Parts;
        if (tree == null)
            return flightComputer.GetFullThrottlePhases();
        VehicleCache cache = _caches.GetOrCreateValue(vehicle);
        try
        {
            if (!BurnsFromActiveSequence(tree, flightComputer, out int structure))
                return flightComputer.GetFullThrottlePhases();

            double mass = vehicle.TotalMass;
            if (!ReferenceEquals(tree, cache.Tree) || structure != cache.Structure
                || !(Math.Abs(mass - cache.Mass) <= MassDriftFraction * cache.Mass))
            {
                if (cache.Performance == null || !ReferenceEquals(tree, cache.Tree))
                    cache.Performance = new SequencePerformanceList(tree);
                cache.Tree = tree;
                cache.Structure = structure;
                cache.Mass = mass;
                cache.Count = 0;
                cache.Performance.RecomputeForFlight(flightComputer.AmbientPressure);
                cache.Count = cache.Performance.FillStagedBurn(fromActive: true, cache.Phases);
                Recomputes++;
            }
            // No sequence has the number of the active one, for example when the player switched an engine on in its part window and never staged. Stock then has no staged phases after the replan, and BurnPlan.CalculateNewFlightPlansFromFlightComputerOnly plans with one phase of the running engines that never ends.
            if (cache.Count == 0 && flightComputer.ActiveEnginePerformanceMax.Thrust > 0f)
            {
                cache.Single[0] = new FiniteBurnPhase(flightComputer.ActiveEnginePerformanceMax.Thrust,
                    flightComputer.ActiveEnginePerformanceMax.MassFlowRate, flightComputer.TotalMassPropsBody.Mass, double.PositiveInfinity);
                return cache.Single;
            }
        }
        catch (Exception ex)
        {
            cache.Count = 0;
            LogHelper.WarnOnce("afc-planned-burn-phases:" + ex.GetType().Name,
                $"[AFC] The burn phases of the active sequence could not be computed, the stock phases are used: {ex}");
        }
        return cache.Count > 0 ? cache.Phases.AsSpan(0, cache.Count) : flightComputer.GetFullThrottlePhases();
    }

    public static void Reset() => _caches.Clear();

    // The cached list holds the part tree of the craft, so a disposed craft must leave it.
    public static void OnVehicleDisposed(Vehicle vehicle) => _caches.Remove(vehicle);

    // True when stock would fill a new burn from the active sequence, which PhysicsBubble.UpdateStagedBurn decides by the thrust in FlightComputer.ActiveEnginePerformanceMax alone.
    // The hash covers what SequencePerformanceList.Recompute reads besides the propellant masses, so a change of the staging or of an engine computes the phases again at once.
    private static bool BurnsFromActiveSequence(PartTree tree, FlightComputer flightComputer, out int structure)
    {
        tree.EnsureDerived(DerivedData.SolidMotorStacks | DerivedData.Sequences);
        ReadOnlySpan<MoleState> moleStates = tree.Moles.States;
        ReadOnlySpan<RocketState> rocketStates = RocketStates.Of(tree);
        var hash = new HashCode();
        bool burns = flightComputer.ActiveEnginePerformanceMax.Thrust > 0f;

        Span<EngineController> engines = tree.Modules.Get<EngineController>();
        hash.Add(engines.Length);
        for (int i = 0; i < engines.Length; i++)
        {
            EngineController engine = engines[i];
            bool fueled = VehiclePropellant.IsFueled(engine, moleStates, rocketStates, out _, out _);
            hash.Add(engine.IsActive);
            hash.Add(fueled);
            hash.Add(engine.Sequence);
        }

        Span<Decoupler> decouplers = tree.Modules.Get<Decoupler>();
        hash.Add(decouplers.Length);
        for (int i = 0; i < decouplers.Length; i++)
        {
            hash.Add(decouplers[i].IsEnabled);
            hash.Add(decouplers[i].Sequence);
        }

        Span<Tank> tanks = tree.Tanks.Modules;
        hash.Add(tanks.Length);
        for (int i = 0; i < tanks.Length; i++)
            hash.Add(tanks[i].PropellantUseEnabled);

        ReadOnlySpan<Sequence> sequences = tree.SequenceList.Sequences;
        hash.Add(sequences.Length);
        for (int i = 0; i < sequences.Length; i++)
        {
            hash.Add(sequences[i].Number);
            hash.Add((int)sequences[i].Environment);
        }
        hash.Add(tree.SequenceList.ActiveSequence);
        hash.Add(tree.Parts.Length);
        structure = hash.ToHashCode();
        return burns;
    }
}

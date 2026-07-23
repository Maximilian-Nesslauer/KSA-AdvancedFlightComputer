using System;
using System.Collections.Generic;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>Per-sequence rocket-equation inputs in firing order
/// (lowest Sequence.Number first). Sequences (activation units),
/// not Stages (jettison groups), are the right index for dV math.</summary>
internal readonly record struct SequenceInfo(
    int Number,
    double StartMassKg,
    double FuelMassKg,
    double MassFlowKgPerSec,
    double ExhaustVelocityMs,
    double BurnTimeSec);

/// <summary>
/// Per-sequence Tsiolkovsky snapshot. Vacuum thrust (multi-pass burns
/// fire at periapsis / AN / DN). Fuel routing follows each engine
/// core's ConsumptionOrder (the player-selected FlowRule) and counts
/// only propellant-enabled tanks; decoupler subtrees are subtracted at
/// sequence boundaries so StartMass reflects what's still attached.
///
/// Computed in-house rather than read from the game's SequencePerformanceList
/// so the numbers stay vacuum-consistent for the Splitter regardless of a
/// sequence's atmospheric toggle, and depend only on stable per-part inputs.
/// SequenceBurnStateTest validates them against the game's own values.
/// </summary>
internal sealed class SequenceBurnState
{
    private const double MinMassFlowKgPerSec = 1e-6;
    private const double MinDryMassKg = 1.0;

    public IReadOnlyList<SequenceInfo> Sequences { get; }
    public bool HasUsableEngines { get; }

    private SequenceBurnState(IReadOnlyList<SequenceInfo> sequences, bool hasUsableEngines)
    {
        Sequences = sequences;
        HasUsableEngines = hasUsableEngines;
    }

    public static SequenceBurnState Empty { get; } = new(Array.Empty<SequenceInfo>(), false);

    public static SequenceBurnState Analyze(Vehicle vehicle)
    {
        if (vehicle?.Parts == null)
            return Empty;

        ReadOnlySpan<Sequence> sequences = vehicle.Parts.SequenceList.Sequences;
        ReadOnlySpan<MoleState> moleStates = vehicle.Parts.Moles.States;

        // SequenceList sorts by Number ascending = firing order.
        var jettisonedPartIds = new HashSet<uint>();
        var fuelClaimedTankIds = new HashSet<ulong>();
        var enginesScratch = new List<EngineController>();

        double currentMassKg = vehicle.TotalMass;
        var result = new List<SequenceInfo>(sequences.Length);
        bool anyUsable = false;

        for (int si = 0; si < sequences.Length; si++)
        {
            Sequence sequence = sequences[si];
            if (sequence.Parts.IsEmpty)
                continue;

            currentMassKg -= ComputeJettisonedMass(
                sequence, moleStates, jettisonedPartIds, fuelClaimedTankIds);

            // Activated sequences honour engine.IsActive (pilot may have
            // killed some); not-yet-activated sequences co-ignite all.
            CollectEngines(sequence, jettisonedPartIds, sequence.Activated, enginesScratch);
            if (enginesScratch.Count == 0)
                continue;

            double thrust = 0.0;
            double mDot = 0.0;
            foreach (EngineController engine in enginesScratch)
            {
                thrust += engine.VacuumData.ThrustMax.Length();
                mDot += engine.VacuumData.MassFlowRateMax;
            }
            if (mDot < MinMassFlowKgPerSec)
                continue;

            double vExhaust = thrust / mDot;

            double fuelMass = ComputeSequenceFuel(
                enginesScratch, fuelClaimedTankIds, moleStates);

            // Keep end mass strictly positive for the Tsiolkovsky log.
            double burnableFuel = fuelMass;
            double maxBurnable = currentMassKg - MinDryMassKg;
            if (burnableFuel > maxBurnable)
                burnableFuel = Math.Max(0.0, maxBurnable);

            double startMass = currentMassKg;
            double endMass = currentMassKg - burnableFuel;
            double burnTime = burnableFuel / mDot;

            result.Add(new SequenceInfo(
                Number: sequence.Number,
                StartMassKg: startMass,
                FuelMassKg: burnableFuel,
                MassFlowKgPerSec: mDot,
                ExhaustVelocityMs: vExhaust,
                BurnTimeSec: burnTime));

            if (burnableFuel > 0.0)
                anyUsable = true;

            currentMassKg = endMass;
        }

        return new SequenceBurnState(result, anyUsable);
    }

    private static void CollectEngines(
        Sequence sequence,
        HashSet<uint> jettisonedPartIds,
        bool sequenceActivated,
        List<EngineController> outEngines)
    {
        outEngines.Clear();
        ReadOnlySpan<Part> parts = sequence.Parts;
        for (int pi = 0; pi < parts.Length; pi++)
        {
            Part part = parts[pi];
            if (jettisonedPartIds.Contains(part.InstanceId))
                continue;

            Span<EngineController> engines = part.Modules.Get<EngineController>();
            for (int ei = 0; ei < engines.Length; ei++)
            {
                EngineController engine = engines[ei];
                if (sequenceActivated && !engine.IsActive)
                    continue;
                outEngines.Add(engine);
            }
        }
    }

    private static double ComputeSequenceFuel(
        List<EngineController> engines,
        HashSet<ulong> fuelClaimedTankIds,
        ReadOnlySpan<MoleState> moleStates)
    {
        double total = 0.0;
        foreach (EngineController engine in engines)
        {
            foreach (RocketCore core in engine.Cores)
            {
                ResourceManager rm = core.ResourceManager;
                // ConsumptionOrder resolves the engine's player-selected
                // FlowRule to the matching drain-level array, so a cross-stage
                // rule pulls from the tanks the engine actually drains.
                Tank[][]? levels = rm?.ConsumptionOrder;
                if (levels == null)
                    continue;

                for (int i = 0; i < levels.Length; i++)
                {
                    Tank[] tanks = levels[i];
                    if (tanks == null) continue;
                    for (int j = 0; j < tanks.Length; j++)
                    {
                        Tank tank = tanks[j];
                        if (tank == null) continue;
                        // A propellant-disabled tank feeds no engine. Skip
                        // it unclaimed so its mass stays as dead weight in
                        // the running total and a later decoupler subtree
                        // still accounts for it at jettison.
                        if (!tank.PropellantUseEnabled)
                            continue;
                        // First-claim-wins across the whole walk.
                        if (!fuelClaimedTankIds.Add(tank.InstanceId))
                            continue;
                        total += tank.ComputeSubstanceMass(moleStates);
                    }
                }
            }
        }
        return total;
    }

    private static double ComputeJettisonedMass(
        Sequence sequence,
        ReadOnlySpan<MoleState> moleStates,
        HashSet<uint> jettisonedPartIds,
        HashSet<ulong> fuelClaimedTankIds)
    {
        double total = 0.0;
        ReadOnlySpan<Part> parts = sequence.Parts;
        for (int pi = 0; pi < parts.Length; pi++)
        {
            Part part = parts[pi];
            if (!part.Modules.HasAny<Decoupler>()) continue;
            total += CollectSubtreeMass(part, moleStates, jettisonedPartIds, fuelClaimedTankIds);
        }
        return total;
    }

    private static double CollectSubtreeMass(
        Part part,
        ReadOnlySpan<MoleState> moleStates,
        HashSet<uint> jettisonedPartIds,
        HashSet<ulong> fuelClaimedTankIds)
    {
        if (!jettisonedPartIds.Add(part.InstanceId))
            return 0.0;

        double mass = ComputePartMass(part, moleStates, fuelClaimedTankIds);
        List<Part> children = part.TreeChildren;
        for (int i = 0; i < children.Count; i++)
            mass += CollectSubtreeMass(children[i], moleStates, jettisonedPartIds, fuelClaimedTankIds);
        return mass;
    }

    private static double ComputePartMass(
        Part part,
        ReadOnlySpan<MoleState> moleStates,
        HashSet<ulong> fuelClaimedTankIds)
    {
        double mass = SumComponentMass(part.Modules, moleStates, fuelClaimedTankIds);
        ReadOnlySpan<Part> subParts = part.SubParts;
        for (int i = 0; i < subParts.Length; i++)
            mass += SumComponentMass(subParts[i].Modules, moleStates, fuelClaimedTankIds);
        return mass;
    }

    private static double SumComponentMass(
        ModuleList components,
        ReadOnlySpan<MoleState> moleStates,
        HashSet<ulong> fuelClaimedTankIds)
    {
        double mass = SumInertMass(components);
        Span<Tank> tanks = components.Get<Tank>();
        for (int i = 0; i < tanks.Length; i++)
        {
            // First-claim-wins. Claiming a tank in a jettisoned subtree
            // also blocks later sequences' engine networks from pulling
            // fuel from it (it leaves with the booster).
            if (fuelClaimedTankIds.Add(tanks[i].InstanceId))
                mass += tanks[i].ComputeSubstanceMass(moleStates);
        }
        return mass;
    }

    private static double SumInertMass(ModuleList modules)
    {
        double mass = 0.0;
        Span<InertMass> inerts = modules.Get<InertMass>();
        for (int i = 0; i < inerts.Length; i++)
            mass += inerts[i].MassPropertiesAsmb.Props.Mass;
        return mass;
    }
}

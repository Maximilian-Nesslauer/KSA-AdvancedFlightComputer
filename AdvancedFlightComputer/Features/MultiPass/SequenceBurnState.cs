using System;
using System.Collections.Generic;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

internal readonly record struct SequenceInfo(
    int Number,
    double StartMassKg,
    double FuelMassKg,
    double MassFlowKgPerSec,
    double ExhaustVelocityMs,
    double BurnTimeSec);
// A private SequencePerformanceList avoids changing the game worker's shared result.
// RecomputeForFlight(0f) uses vacuum for the active sequence and the selected environment for the rest.
// Flow is held at its ignition value, although later cutoff phases can reduce the real mass flow.
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

        var performance = new SequencePerformanceList(vehicle.Parts);
        performance.RecomputeForFlight(0f);
        // PerformanceSequences and SequenceList.Sequences have matching indices.
        ReadOnlySpan<Sequence> sequences = vehicle.Parts.SequenceList.Sequences;
        ReadOnlySpan<SequencePerformance> perf = performance.PerformanceSequences;
        int count = Math.Min(sequences.Length, perf.Length);

        var result = new List<SequenceInfo>(count);
        bool anyUsable = false;

        for (int i = 0; i < count; i++)
        {
            ref readonly SequencePerformance p = ref perf[i];
            if (!(p.MassFlowRate >= MinMassFlowKgPerSec) || !(p.Thrust > 0f))
                continue;

            double vExhaust = p.Thrust / p.MassFlowRate;
            // SequencePerformanceList.Recompute omits inert mass from subparts when calculating WetMass.
            double startMass = p.WetMass + SubPartInertMassKg(p.AttachedParts);
            double burnableFuel = p.BurnedFuelMass;
            // Keep dry mass positive for the rocket equation logarithm.
            double maxBurnable = startMass - MinDryMassKg;
            if (burnableFuel > maxBurnable)
                burnableFuel = Math.Max(0.0, maxBurnable);

            result.Add(new SequenceInfo(
                Number: sequences[i].Number,
                StartMassKg: startMass,
                FuelMassKg: burnableFuel,
                MassFlowKgPerSec: p.MassFlowRate,
                ExhaustVelocityMs: vExhaust,
                BurnTimeSec: burnableFuel / p.MassFlowRate));

            if (burnableFuel > 0.0)
                anyUsable = true;
        }

        return new SequenceBurnState(result, anyUsable);
    }

    private static double SubPartInertMassKg(HashSet<Part>? attachedParts)
    {
        if (attachedParts == null)
            return 0.0;
        double mass = 0.0;
        foreach (Part part in attachedParts)
        {
            ReadOnlySpan<Part> subParts = part.SubParts;
            for (int i = 0; i < subParts.Length; i++)
                mass += subParts[i].InertMass?.MassPropertiesAsmb.Props.Mass ?? 0f;
        }
        return mass;
    }
}

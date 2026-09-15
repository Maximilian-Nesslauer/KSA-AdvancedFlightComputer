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
// Flow is the first phase's value, although later cutoff phases can reduce the real mass flow.
// Exhaust velocity comes from Isp. For a sequence that is not active the game takes Thrust from the ignition thrust but MassFlowRate from the paced first phase, so their ratio is not an exhaust velocity once a solid motor burns.
internal sealed class SequenceBurnState
{
    private const double G0 = 9.80665;
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
            if (!(p.MassFlowRate >= MinMassFlowKgPerSec) || !(p.Thrust > 0f) || !(p.Isp > 0f))
                continue;

            double vExhaust = p.Isp * G0;
            double startMass = p.WetMass;
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
}

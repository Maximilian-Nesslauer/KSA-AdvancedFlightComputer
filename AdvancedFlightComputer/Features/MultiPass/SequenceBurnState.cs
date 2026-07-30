using System;
using System.Collections.Generic;
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
/// Per-sequence Tsiolkovsky snapshot for the Splitter, adapted from a private
/// <see cref="SequencePerformanceList"/> run. The game's staged model is
/// non-trivial - jettison subtrees resolve through the decoupler's connector
/// side, engines from earlier sequences keep burning while still attached, and
/// a sequence's drain stops when its cutoff group (engines a later sequence
/// jettisons) is spent - so the numbers come from the game's own computation
/// rather than a re-derivation that has to chase that model across updates.
///
/// Deliberate consequences of adopting the stock model, so they read as
/// choices rather than oversights:
/// <list type="bullet">
/// <item><c>RecomputeForFlight(0f)</c> pins only the ACTIVE sequence to
/// vacuum; a non-active sequence a player toggled Atmospheric evaluates at
/// sea level, matching the stock sequence display. For an orbital split that
/// under-states such a sequence's dV (sea-level Isp), which errs on the
/// conservative side. The preview cache hashes the toggle so a change is not
/// served stale.</item>
/// <item>Stock applies no <c>engine.IsActive</c> filter outside the live
/// active-sequence override, so a shut-down engine still contributes its
/// design thrust to planned sequences (the old in-house walk excluded
/// it).</item>
/// <item>While the active sequence is actually firing, stock replaces its
/// design numbers with the live throttled nozzle sums. The two preview caches
/// freeze during Auto burns, so the Splitter rarely sees those transients;
/// <see cref="MultiPassPreviewCache.GetSequenceState"/> has no such freeze,
/// so the burn-time and savings readouts can show throttle-inflated numbers
/// mid-burn.</item>
/// </list>
///
/// The instance is private to the call so the game's shared, worker-thread-
/// recomputed list is never touched. Safe because every piece of recompute
/// scratch in <see cref="SequencePerformanceList"/> (and the drain helpers it
/// calls) is instance state; re-verify that on game update, since scratch
/// moving to a static would make this silently racy.
///
/// A sequence with cutoff phases really has a falling mass flow; this snapshot
/// carries the ignition-time flow for the whole sequence, the same first-order
/// simplification the game's own Isp readout makes.
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

        var performance = new SequencePerformanceList(vehicle.Parts);
        performance.RecomputeForFlight(0f);

        // PerformanceSequences is index-aligned with SequenceList.Sequences.
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
            // Stock's WetMass sums InertMass over top-level parts only, while
            // its fuel sum does include sub-part tanks, so the reported start
            // mass runs light by the sub-part inert mass (engines ship as
            // sub-parts). Left uncorrected, ln(m0 / (m0 - fuel)) overstates dV
            // capacity and the fuel-short warning can stay silent. Repair it
            // from the sequence's own attached set.
            double startMass = p.WetMass + SubPartInertMassKg(p.AttachedParts);

            // Keep end mass strictly positive for the Tsiolkovsky log.
            double burnableFuel = p.BurnedFuelMass;
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

    /// <summary>Inert mass of the sub-parts hanging off the still-attached
    /// top-level parts, the share stock's WetMass omits. Mirrors stock's own
    /// per-part accessor (<c>part.InertMass</c>) so this adds exactly the
    /// missing terms and nothing else.</summary>
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

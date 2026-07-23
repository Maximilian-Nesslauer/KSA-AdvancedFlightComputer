#if DEBUG
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// DEBUG-only diagnostic that logs AFC's per-sequence <see cref="SequenceBurnState"/>
/// numbers next to stock's <see cref="SequencePerformanceList"/> output for the same
/// vehicle. Stock's side is computed on a private, mod-owned instance, so this never
/// reads or writes the game's shared performance list; it only logs, and it is
/// compiled out of Release builds.
/// </summary>
internal static class SequenceBurnStateCrossCheck
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    // Force vacuum so stock's per-sequence numbers are comparable to AFC's
    // VacuumData-based ones (the active flight sequence would otherwise use
    // live altitude pressure). A per-sequence Environment=Atmospheric toggle
    // still overrides this to sea level in stock, which is a divergence the
    // cross-check is meant to surface.
    private const float VacuumPressurePa = 0f;

    // Sub-part inert mass below this (kg) is treated as zero when listing.
    private const double MinListedSubMassKg = 0.001;

    private static readonly HashSet<string> _breakdownLogged = new HashSet<string>();

    public static void Log(Vehicle vehicle, SequenceBurnState afcState)
    {
        if (!DebugConfig.MultiPass) return;
        if (vehicle == null) return;

        PartTree parts = vehicle.Parts;
        if (parts == null || afcState.Sequences.Count == 0) return;

        try
        {
            // A private instance, never parts.PerformanceSequences: the game
            // recomputes its shared list from a worker thread (VehicleUpdateTask
            // on JobSystems.VehicleSolvers), so mutating that instance here would
            // race its scratch buffers and clobber the span the stock staging UI
            // reads. Our own instance only reads the PartTree and writes itself.
            var stock = new SequencePerformanceList(parts);
            stock.RecomputeForFlight(VacuumPressurePa);

            // PerformanceSequences is index-aligned with SequenceList.Sequences,
            // so map by Sequence.Number to align with AFC's firing-order list.
            ReadOnlySpan<Sequence> seqs = parts.SequenceList.Sequences;
            ReadOnlySpan<SequencePerformance> perf = stock.PerformanceSequences;
            int count = Math.Min(seqs.Length, perf.Length);
            var byNumber = new Dictionary<int, SequencePerformance>(count);
            for (int i = 0; i < count; i++)
                byNumber[seqs[i].Number] = perf[i];

            var sb = new StringBuilder();
            sb.AppendFormat(Inv,
                "[AFC] SequenceBurnState vs stock PerformanceSequences (vehicle='{0}', vacuum). afc/stock (d%):",
                vehicle.Id);

            double afcTotalDv = 0.0;
            foreach (SequenceInfo s in afcState.Sequences)
            {
                double afcDv = TsiolkovskyDv(s.StartMassKg, s.FuelMassKg, s.ExhaustVelocityMs);
                afcTotalDv += afcDv;

                if (!byNumber.TryGetValue(s.Number, out SequencePerformance p))
                {
                    sb.AppendFormat(Inv, "\n  seq {0}: no stock sequence with this number", s.Number);
                    continue;
                }

                double stockVe = p.MassFlowRate > 0f ? p.Thrust / p.MassFlowRate : 0.0;
                sb.AppendFormat(Inv, "\n  seq {0}:", s.Number);
                AppendMetric(sb, "start", s.StartMassKg, p.WetMass);
                AppendMetric(sb, "fuel", s.FuelMassKg, p.BurnedFuelMass);
                AppendMetric(sb, "mDot", s.MassFlowKgPerSec, p.MassFlowRate);
                AppendMetric(sb, "Ve", s.ExhaustVelocityMs, stockVe);
                AppendMetric(sb, "dV", afcDv, p.DeltaV);
            }

            sb.Append('\n');
            AppendMetric(sb, "total dV", afcTotalDv, stock.TotalDeltaV);

            DefaultCategory.Log.Debug(sb.ToString());

            LogInertMassBreakdown(vehicle, parts);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("seqburnstate-crosscheck",
                string.Format(Inv, "[AFC] SequenceBurnState cross-check failed (vehicle='{0}'): {1}",
                    vehicle.Id, ex));
        }
    }

    // Stock's staging WetMass counts only each top-level part's own InertMass;
    // Vehicle.TotalMass counts the tree-wide list, sub-parts included. This
    // dumps, once per vehicle, where the sub-part inert mass (the staging-mass
    // gap the cross-check reports) actually sits, grouped by sub-part template.
    private static void LogInertMassBreakdown(Vehicle vehicle, PartTree parts)
    {
        string vid = vehicle.Id ?? "?";
        if (!_breakdownLogged.Add(vid)) return;

        double totalOwn = 0.0;
        double totalSub = 0.0;
        var subByType = new Dictionary<string, (int count, double mass)>();

        var sb = new StringBuilder();
        sb.AppendFormat(Inv,
            "[AFC] InertMass breakdown for '{0}' (top-level own vs sub-part; the sub-part total is the staging-mass gap):",
            vid);

        ReadOnlySpan<Part> topParts = parts.Parts;
        for (int i = 0; i < topParts.Length; i++)
        {
            Part part = topParts[i];
            double own = part.InertMass?.MassPropertiesAsmb.Props.Mass ?? 0.0;
            totalOwn += own;

            double sub = 0.0;
            ReadOnlySpan<Part> subParts = part.SubParts;
            for (int j = 0; j < subParts.Length; j++)
            {
                Part sp = subParts[j];
                double m = sp.InertMass?.MassPropertiesAsmb.Props.Mass ?? 0.0;
                if (m <= 0.0) continue;
                sub += m;
                string key = string.IsNullOrEmpty(sp.Id) ? (sp.DisplayName ?? "?") : sp.Id;
                subByType.TryGetValue(key, out var agg);
                subByType[key] = (agg.count + 1, agg.mass + m);
            }
            totalSub += sub;

            if (sub > MinListedSubMassKg)
                sb.AppendFormat(Inv, "\n  {0}#{1}: own {2:F2}kg, sub-parts {3:F2}kg",
                    part.DisplayName, part.InstanceId, own, sub);
        }

        sb.AppendFormat(Inv,
            "\n  TOTALS: top-level own {0:F2}kg, sub-part {1:F2}kg, TotalMass {2:F2}kg",
            totalOwn, totalSub, vehicle.TotalMass);

        var ranked = new List<(string key, int count, double mass)>();
        foreach (var kv in subByType)
            ranked.Add((kv.Key, kv.Value.count, kv.Value.mass));
        ranked.Sort((a, b) => b.mass.CompareTo(a.mass));

        sb.Append("\n  heaviest sub-part templates:");
        int shown = Math.Min(ranked.Count, 25);
        for (int i = 0; i < shown; i++)
            sb.AppendFormat(Inv, "\n    {0} x{1} = {2:F2}kg", ranked[i].key, ranked[i].count, ranked[i].mass);
        if (ranked.Count > shown)
            sb.AppendFormat(Inv, "\n    ... and {0} more template(s)", ranked.Count - shown);

        DefaultCategory.Log.Debug(sb.ToString());
    }

    private static double TsiolkovskyDv(double startMass, double fuelMass, double vExhaust)
    {
        double endMass = startMass - fuelMass;
        if (startMass <= 0.0 || endMass <= 0.0 || vExhaust <= 0.0)
            return 0.0;
        return vExhaust * Math.Log(startMass / endMass);
    }

    private static void AppendMetric(StringBuilder sb, string name, double afc, double stock) =>
        sb.AppendFormat(Inv, "  {0} {1:F1}/{2:F1} ({3}%)", name, afc, stock, Pct(afc, stock));

    private static string Pct(double afc, double stock)
    {
        double d = Math.Abs(stock) > 1e-9 ? (afc - stock) / stock * 100.0 : 0.0;
        return d.ToString("F1", Inv);
    }
}
#endif

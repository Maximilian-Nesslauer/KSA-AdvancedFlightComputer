using System.Collections.Generic;
using System.Globalization;
using System.Text;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Diagnostic logging helpers for the multi-pass state machine. All
/// methods short-circuit when <see cref="DebugConfig.MultiPass"/> is
/// off, so call sites do not need to gate themselves.
/// </summary>
internal static class MultiPassDebug
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static void LogExec(string context, MultiPassExecution exec)
    {
        if (!DebugConfig.MultiPass) return;
        DefaultCategory.Log.Debug(string.Format(Inv,
            "[AFC] {0}: save='{1}' vehicle='{2}' kind='{3}' mode={4} " +
            "passIndex={5}/{6} burn=(t={7} dv={8} ms={9}) await={10}/{11} fails={12}",
            context,
            exec.SaveId,
            exec.VehicleId,
            exec.Intent.Kind,
            exec.Mode,
            exec.PassIndex,
            exec.PassCountTotal,
            exec.CurrentBurnTimeSec.HasValue ? exec.CurrentBurnTimeSec.Value.ToString("F1", Inv) : "-",
            exec.CurrentBurnDvMagnitudeMs.HasValue ? exec.CurrentBurnDvMagnitudeMs.Value.ToString("F2", Inv) : "-",
            exec.CurrentBurn != null ? "yes" : "no",
            exec.AwaitingMaterialization ? "yes" : "no",
            exec.AwaitingMaterializationTicks,
            exec.ConsecutiveScheduleFailures));
    }

    /// <summary>Dumps all burns currently in <paramref name="plan"/>
    /// with their fingerprints (Time + DvMagnitude). Used to diagnose
    /// "registry says X but BurnPlan disagrees" states.</summary>
    public static void LogBurnPlan(string context, BurnPlan plan)
    {
        if (!DebugConfig.MultiPass) return;
        int n = plan.BurnCount;
        var sb = new StringBuilder();
        sb.AppendFormat(Inv, "[AFC] {0}: BurnPlan has {1} burn(s)", context, n);
        for (int i = 0; i < n; i++)
        {
            if (!plan.TryGetBurn(i, out Burn? b) || b == null) continue;
            int patchCount = b.FlightPlan?.Patches?.Count ?? 0;
            sb.AppendFormat(Inv,
                "\n  [{0}] t={1:F1}s dv={2:F2}m/s gizmoActive={3} patches={4}",
                i, b.Time.Seconds(), b.DeltaVVlf.Length(),
                b.IsGizmoActive ? "yes" : "no",
                patchCount);
        }
        DefaultCategory.Log.Debug(sb.ToString());
    }

    public static void LogRegistry(string context, IReadOnlyDictionary<(string, string), MultiPassExecution> snapshot)
    {
        if (!DebugConfig.MultiPass) return;
        var sb = new StringBuilder();
        sb.AppendFormat(Inv, "[AFC] {0}: registry has {1} entry(ies)", context, snapshot.Count);
        foreach (var kv in snapshot)
        {
            sb.AppendFormat(Inv,
                "\n  save='{0}' vehicle='{1}' kind='{2}' passIndex={3}/{4} t={5} dv={6}",
                kv.Key.Item1, kv.Key.Item2,
                kv.Value.Intent.Kind,
                kv.Value.PassIndex,
                kv.Value.PassCountTotal,
                kv.Value.CurrentBurnTimeSec.HasValue ? kv.Value.CurrentBurnTimeSec.Value.ToString("F1", Inv) : "-",
                kv.Value.CurrentBurnDvMagnitudeMs.HasValue ? kv.Value.CurrentBurnDvMagnitudeMs.Value.ToString("F2", Inv) : "-");
        }
        DefaultCategory.Log.Debug(sb.ToString());
    }
}

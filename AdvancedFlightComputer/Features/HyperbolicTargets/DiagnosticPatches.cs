#if DEBUG
using System;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.HyperbolicTargets;

/// <summary>
/// Logs transfer data when a porkchop entry is selected. Debug-only:
/// the patch isn't even installed in Release, so the runtime check on
/// DebugConfig.HyperbolicTargets is a per-feature mute toggle, not a
/// build-mode toggle.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), "SetSelectedTransfer", new Type[0])]
internal static class Patch_DiagnosticLog
{
    static void Postfix(bool __result)
    {
        if (!__result || !DebugConfig.HyperbolicTargets) return;

        try
        {
            var entry = GameReflection.TransferPlanner_selectedEntry!.GetValue(null)
                as OrbitalTransfers.PorkChopEntry;
            if (entry?.FlightPlan == null) return;

            var data = entry.TransferData;
            var patches = entry.FlightPlan.Patches;
            double dvMag = data.TransferDvVlf.Length();

            DefaultCategory.Log.Info(
                $"[AFC] Transfer: dV={dvMag:F1} m/s, {patches.Count} patches, " +
                $"transit={data.Transit.Seconds():F0}s, " +
                $"closest={data.ClosestApproachDistance:E2}m");

            for (int i = 0; i < patches.Count; i++)
            {
                var p = patches[i];
                var o = p.Orbit;
                double startTaDeg = p.StartTrueAnomaly.Value() * (180.0 / Math.PI);
                double endTaDeg = p.EndTrueAnomaly.Value() * (180.0 / Math.PI);

                DefaultCategory.Log.Debug(
                    $"[AFC]   [{i}] e={o.Eccentricity:F4} parent={p.PrimaryBody?.Id ?? "?"} " +
                    $"{p.StartTransition}->{p.EndTransition} " +
                    $"TA={startTaDeg:F1}..{endTaDeg:F1}deg");

                if (o.Eccentricity >= 1.0)
                {
                    // Asymptote half-angle: for a hyperbolic orbit with e>1,
                    // the trajectory approaches +/-acos(-1/e) from the focal axis.
                    double asymDeg = Math.Acos(-1.0 / o.Eccentricity) * (180.0 / Math.PI);
                    DefaultCategory.Log.Debug(
                        $"[AFC]       asymptote=+/-{asymDeg:F1}deg " +
                        $"points={(!o.IsMissingPoints() ? "ok" : "MISSING")}");
                }
            }
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] DiagnosticLog: {ex.Message}");
        }
    }
}
#endif

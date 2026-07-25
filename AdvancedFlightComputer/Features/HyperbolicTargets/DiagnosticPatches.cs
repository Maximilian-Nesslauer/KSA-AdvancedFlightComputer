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
            OrbitalTransfers.PorkChopEntry? entry = StockPlanner.SelectedEntry;
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
                else
                {
                    LogApoapsisSensitivity(o);
                }
            }
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] DiagnosticLog: {ex}");
        }
    }

    /// <summary>How far the apoapsis moves per 1 m/s of periapsis-velocity error,
    /// d(Ap)/dv = 4 a^2 v_p / mu (from a = 1/(2/r - v^2/mu), Ap = 2a - r_p).
    ///
    /// For a near-escape departure ellipse this runs into thousands of km per m/s,
    /// so a sub-percent finite-burn shortfall on a multi-km/s injection drops the
    /// apoapsis far short of the target and the encounter is lost. The plan is
    /// impulsive, the burn is not, which is what the number makes visible.</summary>
    private static void LogApoapsisSensitivity(Orbit o)
    {
        double mu = o.Mu;
        double rp = o.Periapsis;
        double a = o.SemiMajorAxis;
        if (!(mu > 0.0) || !(rp > 0.0) || !(a > 0.0)) return;

        double term = 2.0 / rp - 1.0 / a;
        if (!(term > 0.0)) return;
        double vp = Math.Sqrt(mu * term);
        double dApDv = 4.0 * a * a * vp / mu;

        DefaultCategory.Log.Debug(
            $"[AFC]       v_p={vp:F1}m/s Ap={o.Apoapsis:E3}m " +
            $"dAp/dv={dApDv / 1000.0:F0}km per m/s " +
            $"(1% of a 3km/s injection = {30.0 * dApDv / 1000.0:E2}km of apoapsis)");
    }
}
#endif

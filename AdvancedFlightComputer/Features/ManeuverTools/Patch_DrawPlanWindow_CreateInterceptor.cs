using System;
using System.Collections.Generic;
using System.Reflection.Emit;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.Logging;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// Second IL transpiler on <see cref="TransferPlanner.DrawPlanWindow"/>:
/// replaces the <see cref="Burn.Create"/> call inside the stock "Create"
/// button block with our wrapper, so clicking stock's Create button
/// either fires a single burn (N==1) or starts a multi-pass execution
/// (N>1) depending on the inline UI state.
///
/// Co-exists with <see cref="Patch_DrawPlanWindow_HohmannMultiPass"/>:
/// that one injects DrawInline before DrawCorrectionTransfer (different
/// IL site), this one replaces a Burn.Create call (also different site).
///
/// DrawPlanWindow has exactly one Burn.Create call site (line ~443 in
/// KSA 2026.5.7.4397). The other Burn.Create at line ~904 lives in the
/// separate DrawCorrectionTransfer method body and is NOT touched by
/// this transpiler.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow))]
internal static class Patch_DrawPlanWindow_CreateInterceptor
{
    private static readonly Type[] BurnCreateSig = new[]
    {
        typeof(OrbitPointCce), typeof(double), typeof(double3),
        typeof(PatchedConic), typeof(Vehicle),
    };

    /// <summary>Returns true if the stock <see cref="Burn.Create"/>
    /// overload we anchor on exists. Mod.cs uses this to decide whether
    /// to apply the transpiler.</summary>
    public static bool IsAnchorPresent =>
        AccessTools.Method(typeof(Burn), nameof(Burn.Create), BurnCreateSig) != null;

    [HarmonyTranspiler]
    static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        var anchor = AccessTools.Method(typeof(Burn), nameof(Burn.Create), BurnCreateSig);
        var replacement = AccessTools.Method(typeof(HohmannCreateInterceptor),
            nameof(HohmannCreateInterceptor.CreateMaybeMultiPass));

        if (anchor == null)
        {
            DefaultCategory.Log.Warning(
                "[AFC] HohmannCreateInterceptor transpiler: stock Burn.Create " +
                "overload not found; leaving DrawPlanWindow unmodified.");
            foreach (var ins in instructions) yield return ins;
            yield break;
        }

        int callsToAnchor = 0;
        bool replaced = false;
        foreach (var ins in instructions)
        {
            if (ins.Calls(anchor))
            {
                callsToAnchor++;
                // Replace only the first occurrence (Create button site).
                // Same Type[] signature on our replacement so the IL stack
                // effects match; labels carry over from the original call.
                if (!replaced)
                {
                    var newIns = new CodeInstruction(OpCodes.Call, replacement);
                    newIns.labels.AddRange(ins.labels);
                    yield return newIns;
                    replaced = true;
                    continue;
                }
            }
            yield return ins;
        }

        if (replaced)
        {
            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(
                    $"[AFC] HohmannCreateInterceptor transpiler: replaced Burn.Create " +
                    $"in DrawPlanWindow ({callsToAnchor} call site(s) found).");
        }
        else
            DefaultCategory.Log.Warning(
                "[AFC] HohmannCreateInterceptor transpiler: no Burn.Create call " +
                "found in DrawPlanWindow; create-button interception inactive.");
    }
}

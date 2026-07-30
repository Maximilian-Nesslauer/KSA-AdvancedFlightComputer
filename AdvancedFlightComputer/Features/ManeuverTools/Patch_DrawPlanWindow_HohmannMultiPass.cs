using System.Collections.Generic;
using System.Reflection.Emit;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// IL transpiler that injects <see cref="HohmannMultiPassUI.DrawInline"/>
/// into stock's <see cref="TransferPlanner.DrawPlanWindow"/> right before
/// its call to <c>DrawCorrectionTransfer</c>. Result: our multi-pass
/// section appears inside the stock Transfer Planning window between the
/// Create button and the correction-burn / porkchop UI.
///
/// Anchor choice: <c>DrawCorrectionTransfer</c> is a private static
/// method called exactly once from <c>DrawPlanWindow</c> in the
/// Hohmann-success path. That makes it a stable, unique anchor that
/// doesn't depend on string literals or argument signatures.
///
/// Fail-soft: if the anchor isn't found (KSA refactored / removed
/// DrawCorrectionTransfer), the transpiler returns the original IL
/// unchanged and logs a warning. The multi-pass section then doesn't
/// render but stock Hohmann still works normally. <see cref="IsAnchorPresent"/>
/// is checked at load time so we can also gate <see cref="HohmannMultiPassUI.Enabled"/>
/// on the anchor being patchable.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow))]
internal static class Patch_DrawPlanWindow_HohmannMultiPass
{
    /// <summary>Returns true if the stock method we need to anchor on
    /// exists in this KSA build. Mod.cs uses this to decide whether to
    /// apply the transpiler at all.</summary>
    public static bool IsAnchorPresent =>
        AccessTools.Method(typeof(TransferPlanner), "DrawCorrectionTransfer",
            System.Type.EmptyTypes) != null;

    [HarmonyTranspiler]
    static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        var anchor = AccessTools.Method(typeof(TransferPlanner),
            "DrawCorrectionTransfer", System.Type.EmptyTypes);
        var injectTarget = AccessTools.Method(typeof(HohmannMultiPassUI),
            nameof(HohmannMultiPassUI.DrawInline));

        if (anchor == null || injectTarget == null)
        {
            DefaultCategory.Log.Warning(
                "[AFC] HohmannMultiPass transpiler: anchor or inject target missing " +
                "(anchor=" + (anchor != null ? "ok" : "MISSING") + ", " +
                "inject=" + (injectTarget != null ? "ok" : "MISSING") +
                "); leaving DrawPlanWindow unmodified.");
            foreach (var ins in instructions)
                yield return ins;
            yield break;
        }

        int totalIns = 0;
        int callsToAnchor = 0;
        bool injected = false;
        foreach (var ins in instructions)
        {
            totalIns++;
            // Anchor on the first invocation of DrawCorrectionTransfer.
            // The invariant is that DrawPlanWindow calls it exactly once;
            // guarded with `injected` so a hypothetical future second site
            // doesn't cause double draws.
            if (ins.Calls(anchor))
            {
                callsToAnchor++;
                if (!injected)
                {
                    // Move labels from the anchor instruction to our
                    // injected call. Without this, branch instructions
                    // (e.g. the br at the end of the if-else block
                    // preceding DrawCorrectionTransfer) target the
                    // original call directly, skipping our injection.
                    var injectIns = new CodeInstruction(OpCodes.Call, injectTarget);
                    injectIns.labels.AddRange(ins.labels);
                    ins.labels.Clear();
                    yield return injectIns;
                    injected = true;
                }
            }
            yield return ins;
        }

        if (injected)
        {
            // Once per load: Harmony re-runs this transpiler whenever
            // another patch lands on or leaves DrawPlanWindow, and each
            // re-run would repeat the success line.
            if (DebugConfig.MultiPass)
                LogHelper.DebugOnce("transpiler-hohmann-multipass",
                    $"[AFC] HohmannMultiPass transpiler: injected DrawInline before " +
                    $"DrawCorrectionTransfer ({totalIns} IL instructions scanned, " +
                    $"{callsToAnchor} anchor call(s) found).");
        }
        else
            DefaultCategory.Log.Warning(
                $"[AFC] HohmannMultiPass transpiler: no call to DrawCorrectionTransfer " +
                $"found in DrawPlanWindow IL ({totalIns} IL instructions scanned); " +
                "multi-pass section not injected.");
    }
}

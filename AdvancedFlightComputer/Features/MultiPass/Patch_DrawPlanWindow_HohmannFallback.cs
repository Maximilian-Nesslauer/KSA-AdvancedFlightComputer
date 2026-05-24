using System.Collections.Generic;
using System.Reflection.Emit;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Second IL transpiler on <see cref="TransferPlanner.DrawPlanWindow"/>
/// that injects <see cref="HohmannMultiPassUI.DrawInline"/> right
/// before the outermost <see cref="ImGui.End"/> call. This is the
/// fallback render point that fires regardless of stock's
/// <c>_transferCalculated</c> state.
///
/// The first injection (<see cref="ManeuverTools.Patch_DrawPlanWindow_HohmannMultiPass"/>)
/// lives inside the <c>if (_transferCalculated &amp;&amp; _transferInfo != null)</c>
/// block and only fires when stock has a calculated porkchop. After
/// the user closes the window with F4 (stock's <c>ShowPlanWindow.set(false)</c>
/// clears <c>_selectedEntry</c> and <c>_transferCalculated</c>) and
/// reopens, the first injection is skipped, leaving the user with no
/// way to view active multi-pass status or to cancel without first
/// running Calculate again. This second injection guarantees a render
/// path even in that state.
///
/// <see cref="HohmannMultiPassUI.DrawInline"/> is per-frame idempotent
/// via <see cref="ImGui.GetFrameCount"/>, so in the normal flow where
/// the first injection has already rendered mid-window, the call from
/// this second injection short-circuits and there is no duplicate UI.
///
/// Anchor stability: <see cref="TransferPlanner.DrawPlanWindow"/>
/// contains exactly one direct <c>ImGui.End()</c> call (matching the
/// outer <c>ImGui.Begin("Transfer Planning")</c>). Helper methods
/// invoked from inside (<c>DrawCorrectionTransfer</c>, etc.) live in
/// separate method bodies, so their End calls do not appear in this
/// transpiler's IL stream. If stock later adds another End in this
/// method (e.g., a sub-window), we anchor on the FIRST one, which by
/// stock's current structure is the outer one.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow))]
internal static class Patch_DrawPlanWindow_HohmannFallback
{
    /// <summary>Returns true if the <see cref="ImGui.End"/> overload
    /// we anchor on exists. Mod.cs uses this to decide whether to
    /// apply the transpiler.</summary>
    public static bool IsAnchorPresent =>
        AccessTools.Method(typeof(ImGui), nameof(ImGui.End), System.Type.EmptyTypes) != null;

    [HarmonyTranspiler]
    static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        var anchor = AccessTools.Method(typeof(ImGui), nameof(ImGui.End), System.Type.EmptyTypes);
        var injectTarget = AccessTools.Method(typeof(HohmannMultiPassUI),
            nameof(HohmannMultiPassUI.DrawInline));

        if (anchor == null || injectTarget == null)
        {
            DefaultCategory.Log.Warning(
                "[AFC] HohmannFallback transpiler: anchor or inject target missing " +
                "(anchor=" + (anchor != null ? "ok" : "MISSING") + ", " +
                "inject=" + (injectTarget != null ? "ok" : "MISSING") +
                "); leaving DrawPlanWindow unmodified.");
            foreach (var ins in instructions) yield return ins;
            yield break;
        }

        int totalIns = 0;
        bool injected = false;
        foreach (var ins in instructions)
        {
            totalIns++;
            // Inject before the first ImGui.End() call. By stock's
            // current layout this is the outermost call inside the
            // "Transfer Planning" ImGui.Begin block, after all the
            // window content has been drawn.
            if (!injected && ins.Calls(anchor))
            {
                var injectIns = new CodeInstruction(OpCodes.Call, injectTarget);
                injectIns.labels.AddRange(ins.labels);
                ins.labels.Clear();
                yield return injectIns;
                injected = true;
            }
            yield return ins;
        }

        if (injected)
        {
            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(
                    "[AFC] HohmannFallback transpiler: injected DrawInline before " +
                    "ImGui.End (" + totalIns + " IL instructions scanned).");
        }
        else
            DefaultCategory.Log.Warning(
                "[AFC] HohmannFallback transpiler: no ImGui.End() call found in " +
                "DrawPlanWindow IL (" + totalIns + " IL instructions scanned); " +
                "fallback render inactive (multi-pass status hidden after F4 close+reopen " +
                "until user clicks Calculate).");
    }
}

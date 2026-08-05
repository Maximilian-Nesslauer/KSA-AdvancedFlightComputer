using System.Collections.Generic;
using System.Reflection.Emit;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// IL transpiler on <see cref="TransferPlanner.DrawPlanWindow"/>
/// that injects <see cref="HohmannMultiPassUI.DrawInline"/> right
/// before the <see cref="ConsoleStyle.PopWidgetStyle"/> call that closes
/// the window body. This is the fallback render point that fires
/// regardless of stock's <c>_transferCalculated</c> state.
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
/// Anchor choice: the injected UI has to land inside the body child
/// window and while the console widget style is still pushed, which is
/// exactly the span <c>PopWidgetStyle</c> terminates. Anchoring on the
/// window's own <c>EndWindow</c> instead would draw into the raw window
/// after <c>EndBody</c> has already closed the child.
/// <see cref="TransferPlanner.DrawPlanWindow"/> contains exactly one
/// <c>PopWidgetStyle</c> call; helpers invoked from inside
/// (<c>DrawCorrectionTransfer</c>, etc.) live in separate method bodies,
/// so their calls do not appear in this transpiler's IL stream. Should
/// stock ever add a second one, we anchor on the FIRST, which by stock's
/// current structure is the one closing the main body.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow))]
internal static class Patch_DrawPlanWindow_HohmannFallback
{
    /// <summary>Returns true if the <see cref="ConsoleStyle.PopWidgetStyle"/>
    /// overload we anchor on exists. Mod.cs uses this to decide whether to
    /// apply the transpiler.</summary>
    public static bool IsAnchorPresent => Anchor != null;

    private static System.Reflection.MethodInfo? Anchor =>
        AccessTools.Method(typeof(ConsoleStyle), nameof(ConsoleStyle.PopWidgetStyle),
            System.Type.EmptyTypes);

    [HarmonyTranspiler]
    static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        var anchor = Anchor;
        var injectTarget = AccessTools.Method(typeof(HohmannMultiPassUI),
            nameof(HohmannMultiPassUI.DrawInline));

        // Deduped like the success line: Harmony re-runs every transpiler on
        // DrawPlanWindow whenever another patch lands on or leaves it, and five
        // AFC patches target that method, so an undeduped failure line prints
        // once per re-run.
        if (anchor == null || injectTarget == null)
        {
            LogHelper.WarnOnce("transpiler-hohmann-fallback-missing",
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
            // Inject before the first PopWidgetStyle call: by stock's current
            // layout that is the end of the plan window's body content, after
            // everything stock draws and while the widget style still applies.
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
            // Once per load: Harmony re-runs this transpiler whenever
            // another patch lands on or leaves DrawPlanWindow, and each
            // re-run would repeat the success line.
            if (DebugConfig.MultiPass)
                LogHelper.DebugOnce("transpiler-hohmann-fallback",
                    "[AFC] HohmannFallback transpiler: injected DrawInline before " +
                    "ConsoleStyle.PopWidgetStyle (" + totalIns + " IL instructions scanned).");
        }
        else
            LogHelper.WarnOnce("transpiler-hohmann-fallback-noanchor",
                "[AFC] HohmannFallback transpiler: no ConsoleStyle.PopWidgetStyle() call found in " +
                "DrawPlanWindow IL; fallback render inactive (multi-pass status hidden " +
                "after F4 close+reopen until user clicks Calculate).");
    }
}

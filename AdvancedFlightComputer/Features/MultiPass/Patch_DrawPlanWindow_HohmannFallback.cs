using System.Collections.Generic;
using System.Reflection.Emit;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// Keep the fallback inside the styled window body after stock clears the calculated transfer block. A shared frame claim prevents duplicate controls.
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow))]
internal static class Patch_DrawPlanWindow_HohmannFallback
{
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

        // Harmony reapplies transpilers when patches change. Report the warning only once per load.
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

        bool injected = false;
        foreach (var ins in instructions)
        {
            // The first PopWidgetStyle call ends the styled body, so insert the controls immediately before it.
            if (!injected && ins.Calls(anchor))
            {
                var injectIns = new CodeInstruction(OpCodes.Call, injectTarget);
                injectIns.labels.AddRange(ins.labels);
                ins.labels.Clear();
                // Move exception begin markers before the inserted call. Keep end markers on the original anchor.
                foreach (ExceptionBlock block in ins.blocks)
                    if (block.blockType != ExceptionBlockType.EndExceptionBlock)
                        injectIns.blocks.Add(block);
                ins.blocks.RemoveAll(block => block.blockType != ExceptionBlockType.EndExceptionBlock);
                yield return injectIns;
                injected = true;
            }
            yield return ins;
        }

        if (injected)
        {
            // Report success only once because Harmony can apply this transpiler again.
            if (DebugConfig.MultiPass)
                LogHelper.DebugOnce("transpiler-hohmann-fallback",
                    "[AFC] HohmannFallback transpiler: injected DrawInline before " +
                    "ConsoleStyle.PopWidgetStyle (first successful patch application).");
        }
        else
            LogHelper.WarnOnce("transpiler-hohmann-fallback-noanchor",
                "[AFC] HohmannFallback transpiler: no ConsoleStyle.PopWidgetStyle() call found in " +
                "DrawPlanWindow IL; fallback render inactive (multi-pass status hidden " +
                "after F4 close+reopen until user clicks Calculate).");
    }
}

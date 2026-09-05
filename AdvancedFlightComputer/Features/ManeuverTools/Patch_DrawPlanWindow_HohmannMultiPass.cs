using System.Collections.Generic;
using System.Reflection.Emit;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

// DrawCorrectionTransfer provides the single insertion point in the stock UI for a calculated transfer.
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow), new[] { typeof(IGameViewport) })]
internal static class Patch_DrawPlanWindow_HohmannMultiPass
{
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
            LogHelper.WarnOnce("transpiler-hohmann-multipass-missing",
                "[AFC] HohmannMultiPass transpiler: anchor or inject target missing " +
                "(anchor=" + (anchor != null ? "ok" : "MISSING") + ", " +
                "inject=" + (injectTarget != null ? "ok" : "MISSING") +
                "); leaving DrawPlanWindow unmodified.");
            foreach (var ins in instructions)
                yield return ins;
            yield break;
        }

        int totalIns = 0;
        bool injected = false;
        foreach (var ins in instructions)
        {
            totalIns++;
            if (!injected && ins.Calls(anchor))
            {
                var injectIns = new CodeInstruction(OpCodes.Call, injectTarget);
                TranspilerInsertion.MoveEntryMarkers(ins, injectIns);
                yield return injectIns;
                injected = true;
            }
            yield return ins;
        }

        if (!injected)
            LogHelper.WarnOnce("transpiler-hohmann-multipass-no-call",
                $"[AFC] HohmannMultiPass transpiler: no call to DrawCorrectionTransfer " +
                $"found in DrawPlanWindow IL ({totalIns} IL instructions scanned); " +
                "multi-pass section not injected.");
        else if (DebugConfig.MultiPass)
            LogHelper.DebugOnce("transpiler-hohmann-multipass",
                "[AFC] HohmannMultiPass transpiler: injected DrawInline before DrawCorrectionTransfer.");
    }
}

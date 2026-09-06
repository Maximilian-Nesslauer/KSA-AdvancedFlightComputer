using System.Collections.Generic;
using AdvancedFlightComputer.Features.MultiPass;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow), new[] { typeof(IGameViewport) })]
internal static class Patch_DrawPlanWindow_HohmannMultiPass
{
    public static bool IsAnchorPresent =>
        AccessTools.Method(typeof(TransferPlanner), "DrawCorrectionTransfer",
            System.Type.EmptyTypes) != null;

    [HarmonyPrepare]
    static bool Prepare() => false;

    static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
        => PlanWindowPatchPipeline.InjectCalculatedControls(instructions);
}

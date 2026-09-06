using System.Collections.Generic;
using Brutal.ImGuiApi;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow), new[] { typeof(IGameViewport) })]
internal static class Patch_DrawPlanWindow_HohmannFallback
{
    public static bool IsAnchorPresent => Anchor != null;

    private static System.Reflection.MethodInfo? Anchor =>
        AccessTools.Method(typeof(ConsoleStyle), nameof(ConsoleStyle.PopWidgetStyle),
            System.Type.EmptyTypes);

    [HarmonyPrepare]
    static bool Prepare() => false;

    static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
        => AdvancedFlightComputer.Features.ManeuverTools.PlanWindowPatchPipeline
            .InjectFallbackControls(instructions);
}

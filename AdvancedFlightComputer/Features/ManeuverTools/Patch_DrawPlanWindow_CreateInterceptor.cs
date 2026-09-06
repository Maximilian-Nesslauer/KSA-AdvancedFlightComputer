using System.Collections.Generic;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow), new[] { typeof(IGameViewport) })]
internal static class Patch_DrawPlanWindow_CreateInterceptor
{
    private static readonly System.Type[] BurnCreateSig =
    {
        typeof(OrbitPointCce), typeof(double), typeof(Brutal.Numerics.double3),
        typeof(PatchedConic), typeof(Vehicle),
    };

    public static bool IsAnchorPresent =>
        AccessTools.Method(typeof(Burn), nameof(Burn.Create), BurnCreateSig) != null;


    [HarmonyPrepare]
    static bool Prepare() => false;

    static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
        => PlanWindowPatchPipeline.InterceptCreate(instructions);
}

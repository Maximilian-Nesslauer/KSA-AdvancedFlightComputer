using System;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow), new[] { typeof(IGameViewport) })]
internal static class Patch_TransferPlanner_DrawPlanWindow_HohmannMarkers
{
    [HarmonyPrepare]
    static bool Prepare() => false;

    static void Postfix(IGameViewport inViewport)
        => Draw(inViewport);

    internal static void Draw(IGameViewport inViewport)
    {
        try
        {
            if (!HohmannMultiPassUI.ShouldRenderOverlay(out Vehicle? source)) return;
            HohmannMultiPassUI.RenderMarkers(inViewport, source!);
        }
        catch (Exception ex)
        {
            // Deduped: runs per frame.
            LogHelper.WarnOnce("hohmann-markers:" + ex.GetType().Name,
                $"[AFC] Hohmann DrawPlanWindow marker postfix: {ex}");
        }
    }
}

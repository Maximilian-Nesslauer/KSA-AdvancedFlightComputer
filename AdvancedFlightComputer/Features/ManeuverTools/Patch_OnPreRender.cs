using System;
using AdvancedFlightComputer.Core;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.OnPreRender), new[] { typeof(IViewport) })]
internal static class Patch_OnPreRender
{
    static void Postfix(IViewport inViewport)
    {
        try
        {
            Patch_DrawPlanWindow.TickWindowState();
            string? typeKey = StockPlanner.TransferTypeKey;
            if (typeKey == null || !ManeuverTools.IsHandledType(typeKey))
                return;

            Patch_DrawPlanWindow.RenderOrbitPreview(inViewport);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("maneuvertools-onprerender:" + ex.GetType().Name,
                $"[AFC] ManeuverTools OnPreRender: {ex}");
        }
    }
}

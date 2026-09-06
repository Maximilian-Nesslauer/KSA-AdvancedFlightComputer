using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.MultiPass;
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
            if (typeKey != null && ManeuverTools.IsHandledType(typeKey))
                Patch_DrawPlanWindow.RenderOrbitPreview(inViewport);
            else
                RenderHohmannOverlay(inViewport);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("maneuvertools-onprerender:" + ex.GetType().Name,
                $"[AFC] ManeuverTools OnPreRender: {ex}");
        }
    }

    internal static void RenderHohmannOverlay(IViewport inViewport)
    {
        try
        {
            if (HohmannMultiPassUI.ShouldRenderOverlay(out Vehicle? source))
            {
                HohmannMultiPassUI.RenderOrbits(inViewport, source!);
                return;
            }

            if (HohmannFlybyUI.ShouldRenderPreview(out Vehicle? flybySource))
                HohmannFlybyUI.RenderPreview(inViewport, flybySource!);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("hohmann-onprerender:" + ex.GetType().Name,
                $"[AFC] Hohmann OnPreRender postfix: {ex}");
        }
    }
}

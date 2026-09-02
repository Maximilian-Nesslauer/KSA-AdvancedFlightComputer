using System;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// Postfix on TransferPlanner.OnPreRender to render the visual orbit preview
/// in the 3D view when one of our plan types is active.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.OnPreRender))]
internal static class Patch_OnPreRender
{
    static void Postfix(IViewport inViewport)
    {
        try
        {
            string? typeKey = StockPlanner.TransferTypeKey;
            if (typeKey == null || !ManeuverTools.IsHandledType(typeKey))
                return;

            Patch_DrawPlanWindow.RenderOrbitPreview(inViewport);
        }
        catch (Exception ex)
        {
            // Deduped: runs per frame per viewport.
            LogHelper.WarnOnce("maneuvertools-onprerender:" + ex.GetType().Name,
                $"[AFC] ManeuverTools OnPreRender: {ex}");
        }
    }
}

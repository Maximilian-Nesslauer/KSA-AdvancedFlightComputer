using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using KSA;

namespace AdvancedFlightComputer.Features.PlanWindow;

internal static class PlanWindowMarkers
{
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

using System;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Postfix on <see cref="TransferPlanner.DrawPlanWindow"/> that draws
/// per-pass markers (Ap, Pe, AN, DN, SOI transitions, closest approach)
/// for the cached multi-pass preview. Anchored on the public top-level
/// method, not on stock's private <c>DrawSelectedTransferUi</c>, so the
/// markers keep rendering after F4 close + reopen (where
/// <c>_transferCalculated</c> is false and stock's private helper is
/// not called). Gates explicitly on the user's "Preview Selected
/// Transfer" checkbox so the overlay is opt-in just like the orbit
/// lines and stock's own selected-transfer markers.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow))]
internal static class Patch_TransferPlanner_DrawPlanWindow_HohmannMarkers
{
    static void Postfix(IGameViewport inViewport)
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

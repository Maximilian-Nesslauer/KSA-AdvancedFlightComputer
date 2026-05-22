using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.ImGuiApi;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Postfix on private <see cref="TransferPlanner"/>.DrawSelectedTransferUi
/// that draws per-pass markers (Ap, Pe, AN, DN, SOI transitions, closest
/// approach) for the cached multi-pass preview. Anchored on the private
/// method so stock's own gating (<c>_displaySelectedTransfer</c> +
/// <c>_selectedEntry != null</c>) is inherited - the postfix only fires
/// when stock would also render the single-burn marker overlay, which
/// keeps both layers under one user toggle.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), "DrawSelectedTransferUi",
    new[] { typeof(Viewport) })]
internal static class Patch_TransferPlanner_DrawSelectedTransferUi_Hohmann
{
    /// <summary>True if the private DrawSelectedTransferUi(Viewport)
    /// overload exists on this KSA build. Mod.cs gates patch application
    /// on this so a future stock rename / signature change disables the
    /// overlay cleanly rather than throwing at load time.</summary>
    public static bool IsAnchorPresent =>
        AccessTools.Method(typeof(TransferPlanner), "DrawSelectedTransferUi",
            new[] { typeof(Viewport) }) != null;

    static void Postfix(Viewport inViewport)
    {
        try
        {
            if (!HohmannMultiPassUI.HasMultiPassPreview) return;

            // Transfer-type guard: stock only calls DrawSelectedTransferUi
            // from inside DrawPlanWindow, which AFC's Patch_DrawPlanWindow
            // skips for handled (non-Hohmann) types - so today the path
            // only ever runs for Hohmann. The explicit check defends
            // against future stock types that we do not claim.
            var transferType = (TransferType)GameReflection.TransferPlanner_transferType!
                .GetValue(null)!;
            if (transferType.GetKey() != ManeuverTools.ManeuverTools.KeyStockHohmann) return;

            var sourceBody = (TransferObject)GameReflection.TransferPlanner_sourceBody!
                .GetValue(null)!;
            if (sourceBody.Body is not Vehicle source) return;

            HohmannMultiPassUI.RenderMarkers(inViewport, source);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] Hohmann DrawSelectedTransferUi postfix: {ex}");
        }
    }
}

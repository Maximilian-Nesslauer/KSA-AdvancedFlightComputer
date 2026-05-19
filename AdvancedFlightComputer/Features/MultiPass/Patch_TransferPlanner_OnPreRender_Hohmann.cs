using System;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Postfix on <see cref="TransferPlanner.OnPreRender"/> that draws the
/// multi-pass orbit overlay when a Hohmann multi-pass is armed (N&gt;1
/// with a successful preview) or currently executing on the source
/// vehicle. Gates on stock's <c>_displaySelectedTransfer</c> checkbox
/// so the user controls both stock's single-burn and our multi-pass
/// overlay with the same toggle.
///
/// Separate class from <see cref="ManeuverTools.Patch_OnPreRender"/>
/// because that one renders for AFC-handled transfer types (apse /
/// inclination / circularize) where AFC owns the plan window; this
/// patch runs for stock-owned Hohmann where the window is stock's.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.OnPreRender))]
internal static class Patch_TransferPlanner_OnPreRender_Hohmann
{
    static void Postfix(Viewport inViewport)
    {
        try
        {
            if (!ShouldRender(out Vehicle? source)) return;
            HohmannMultiPassUI.RenderOrbits(inViewport, source!);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] Hohmann OnPreRender postfix: {ex}");
        }
    }

    /// <summary>Mirrors stock's OnPreRender gate (ShowPlanWindow,
    /// _displaySelectedTransfer, _transferCalculated) and additionally
    /// requires the current transfer type to be Hohmann + an armed
    /// Hohmann preview. The transfer-type check guards against future
    /// stock transfer types we haven't claimed: PassCompletionPatch
    /// pins _transferCalculated=true while a Hohmann exec is active
    /// (scoped to its own vehicle), so the earlier gate alone cannot
    /// be relied on to filter out unrelated transfer types the user
    /// may have selected.</summary>
    private static bool ShouldRender(out Vehicle? source)
    {
        source = null;
        if (!HohmannMultiPassUI.HasMultiPassPreview) return false;

        if (!(bool)(GameReflection.TransferPlanner_showPlanWindow?.GetValue(null) ?? false))
            return false;
        if (!(bool)(GameReflection.TransferPlanner_displaySelectedTransfer?.GetValue(null) ?? false))
            return false;
        if (!(bool)(GameReflection.TransferPlanner_transferCalculated?.GetValue(null) ?? false))
            return false;

        var transferType = (TransferType)GameReflection.TransferPlanner_transferType!
            .GetValue(null)!;
        if (transferType.GetKey() != "Hohmann") return false;

        var sourceBody = (TransferObject)GameReflection.TransferPlanner_sourceBody!
            .GetValue(null)!;
        source = sourceBody.Body as Vehicle;
        return source != null;
    }
}

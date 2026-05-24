using System;
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
            if (!HohmannMultiPassUI.ShouldRenderOverlay(out Vehicle? source)) return;
            HohmannMultiPassUI.RenderOrbits(inViewport, source!);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] Hohmann OnPreRender postfix: {ex}");
        }
    }
}

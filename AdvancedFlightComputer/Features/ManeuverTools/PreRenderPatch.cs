using System;
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
    static void Postfix(Viewport inViewport)
    {
        try
        {
            Patch_DrawPlanWindow.RenderOrbitPreview(inViewport);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] ManeuverTools OnPreRender: {ex.Message}");
        }
    }
}

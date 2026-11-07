using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.Ui.Console.Pages;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console;

/// <summary>
/// The world lines of the console pages: the orbit after an ORBIT tool's burn, and the selected
/// transfer, its passes and its flyby on TRANSFER. CelestialSystem.OnPreRender calls
/// TransferPlanner.OnPreRender every frame without a window check, so this postfix runs at the same
/// frame point without a patch on any planner member. Each part checks its own MAP switch.
/// </summary>
[HarmonyPatch(typeof(CelestialSystem), nameof(CelestialSystem.OnPreRender), new[] { typeof(IViewport) })]
internal static class Patch_CelestialSystem_OnPreRender
{
    internal static bool Applied;

    internal const string UnavailableTip = "SHOW IN WORLD is not available in this session, because the world line patch did not apply. The log names the reason.";

    static void Postfix(IViewport inViewport)
    {
        try
        {
            if (OrbitToolsPanel.PreviewVisible)
                OrbitToolsPanel.RenderOrbitPreview(inViewport);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("orbit-onprerender:" + ex.GetType().Name,
                $"[AFC] ORBIT world lines: {ex}");
        }

        try
        {
            TransferPage.RenderLines(inViewport);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("transfer-onprerender:" + ex.GetType().Name,
                $"[AFC] TRANSFER world lines: {ex}");
        }
    }
}

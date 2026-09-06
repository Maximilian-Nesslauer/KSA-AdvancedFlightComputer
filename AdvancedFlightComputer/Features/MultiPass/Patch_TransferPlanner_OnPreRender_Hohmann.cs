using System;
using Brutal.ImGuiApi;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.OnPreRender), new[] { typeof(IViewport) })]
internal static class Patch_TransferPlanner_OnPreRender_Hohmann
{
    [HarmonyPrepare]
    static bool Prepare() => false;

    static void Postfix(IViewport inViewport)
        => AdvancedFlightComputer.Features.ManeuverTools.Patch_OnPreRender
            .RenderHohmannOverlay(inViewport);
}

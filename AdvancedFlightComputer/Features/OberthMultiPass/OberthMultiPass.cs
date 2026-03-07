using System;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.OberthMultiPass;

static class OberthMultiPass
{
    public static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(Patch_OberthPreviewRender)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_BurnPlanAddLineInstances)).Patch();

        if (DebugConfig.OberthMultiPass)
            DefaultCategory.Log.Debug("[AFC] OberthMultiPass: patches applied.");
    }
}

[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.OnPreRender))]
static class Patch_OberthPreviewRender
{
    static void Postfix(Viewport inViewport)
    {
        try
        {
            MultiPassRenderer.RenderPreviewLines(inViewport);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] OberthMultiPass OnPreRender: {ex.Message}");
        }
    }
}

/// <summary>
/// Skips the game's BurnPlan orbit line rendering when a multi-pass split is
/// active. The game's AddLineInstances clips intermediate burns via
/// startTa/nextBurnTa which breaks for multi-pass apse burns at the same TA.
/// </summary>
[HarmonyPatch(typeof(BurnPlan), nameof(BurnPlan.AddLineInstances))]
static class Patch_BurnPlanAddLineInstances
{
    static bool Prefix()
    {
        return !MultiPassState.HasActiveSplit;
    }
}

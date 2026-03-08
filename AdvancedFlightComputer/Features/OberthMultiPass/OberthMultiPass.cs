using System;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.OberthMultiPass;

static class OberthMultiPass
{
    public static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(Patch_OberthPreviewRender)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_BurnPlanAddLineInstances)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_FlightPlanDrawUi)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_BurnUpdateGizmos)).Patch();

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

/// <summary>
/// Suppresses stock FlightPlan.DrawUi (Ap/Pe, closest approach, AN/DN, SOI
/// markers) for multi-pass split burns. MultiPassRenderer.RenderPreviewMarkers
/// renders correctly decluttered markers instead.
/// Non-split burns are unaffected so their markers still render normally.
/// Note: Burn.DrawUi (gizmo icon) is a separate call in BurnPlan.DrawUi and
/// is intentionally not suppressed here.
/// </summary>
[HarmonyPatch(typeof(FlightPlan), "DrawUi")]
static class Patch_FlightPlanDrawUi
{
    static bool Prefix(FlightPlan __instance)
    {
        if (!MultiPassState.HasActiveSplit || MultiPassState.PassBurns == null)
            return true;
        foreach (Burn b in MultiPassState.PassBurns)
        {
            if (b.FlightPlan == __instance)
                return false;
        }
        return true;
    }
}

/// <summary>
/// Hides gizmo spheres and drag handles for non-selected multi-pass burns.
/// All passes at the same true anomaly overlap, making it easy to grab the
/// wrong one. Only the radio-selected pass renders its gizmo. Non-multi-pass
/// burns are unaffected.
/// </summary>
[HarmonyPatch(typeof(Burn), nameof(Burn.UpdateGizmos))]
static class Patch_BurnUpdateGizmos
{
    static bool Prefix(Burn __instance)
    {
        if (!MultiPassState.HasActiveSplit || MultiPassState.PassBurns == null)
            return true;

        int idx = MultiPassState.PassBurns.IndexOf(__instance);
        if (idx < 0)
            return true;

        return idx == MultiPassState.SelectedPassIndex;
    }
}

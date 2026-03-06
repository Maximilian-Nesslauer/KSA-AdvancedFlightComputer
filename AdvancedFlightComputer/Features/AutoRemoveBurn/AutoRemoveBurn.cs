using System;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoRemoveBurn;

/// <summary>
/// Independent feature that auto-removes completed burns from the BurnPlan.
/// Works for ALL burn types (not just multi-pass).
/// Detects Auto->Manual BurnMode transition + dot product reversal to confirm
/// the burn executed successfully (not just ran out of propellant).
/// </summary>
static class AutoRemoveBurn
{
    public static bool Enabled = true;

    public static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(Patch_AutoRemoveBurn)).Patch();

        if (DebugConfig.AutoRemoveBurn)
            DefaultCategory.Log.Debug("[AFC] AutoRemoveBurn: patch applied.");
    }
}

[HarmonyPatch(typeof(Vehicle), "UpdateFromTaskResults")]
static class Patch_AutoRemoveBurn
{
    static void Prefix(Vehicle __instance, out FlightComputerBurnMode __state)
    {
        if (!AutoRemoveBurn.Enabled || __instance.FlightComputer == null)
        {
            __state = FlightComputerBurnMode.Manual;
            return;
        }
        __state = __instance.FlightComputer.BurnMode;
    }

    static void Postfix(Vehicle __instance, FlightComputerBurnMode __state)
    {
        try
        {
            if (!AutoRemoveBurn.Enabled)
                return;

            var fc = __instance.FlightComputer;
            if (fc == null)
                return;

            // Only trigger on Auto->Manual transition
            if (__state != FlightComputerBurnMode.Auto)
                return;
            if (fc.BurnMode != FlightComputerBurnMode.Manual)
                return;

            // Confirm completion via dot product reversal.
            // If the burn ran out of propellant, DeltaVToGoCci still points the
            // same way as DeltaVTargetCci (dot product > 0). Reversal (dot <= 0)
            // means the target dV was reached and the vehicle overshot slightly.
            if (fc.Burn == null || float3.Dot(fc.Burn.DeltaVToGoCci, fc.Burn.DeltaVTargetCci) > 0f)
                return;

            if (!fc.BurnPlan.HasActiveBurns)
                return;

            if (DebugConfig.AutoRemoveBurn)
                DefaultCategory.Log.Debug("[AFC] AutoRemoveBurn: burn completed, removing from plan.");

            fc.RemoveBurnAt(0);

            // TODO Phase 2b: MultiPassPlanner.HandlePassCompletion()
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] AutoRemoveBurn Postfix: {ex.Message}");
        }
    }
}

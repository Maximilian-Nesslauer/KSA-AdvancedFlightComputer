using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.OberthMultiPass;
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
            if (__instance != Program.ControlledVehicle)
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

            if (MultiPassState.HasActiveSplit && MultiPassState.PassBurns != null)
            {
                // Verify the removed burn was actually one of our pass burns. Pass burns are
                // sorted by time, so if PassBurns[0] is no longer in the plan, it was the one
                // completed. If it is still present, a non-multi-pass burn was removed instead
                // (e.g., the user had a manually created burn scheduled before our passes).
                bool firstPassBurnRemoved = !fc.BurnPlan.TryGetBurn(MultiPassState.PassBurns[0]);
                if (!firstPassBurnRemoved)
                {
                    if (DebugConfig.AutoRemoveBurn)
                        DefaultCategory.Log.Debug(
                            "[AFC] AutoRemoveBurn: removed burn was not a multi-pass burn, skipping correction.");
                }
                else if (fc.BurnPlan.HasActiveBurns)
                {
                    MultiPassPlanner.HandlePassCompletion(__instance);
                    fc.BurnMode = FlightComputerBurnMode.Auto;
                }
                else
                {
                    MultiPassState.ValidateState();
                    if (DebugConfig.AutoRemoveBurn)
                        DefaultCategory.Log.Debug(
                            "[AFC] AutoRemoveBurn: last multi-pass burn done, state reset.");
                }
            }
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] AutoRemoveBurn Postfix: {ex.Message}");
        }
    }
}

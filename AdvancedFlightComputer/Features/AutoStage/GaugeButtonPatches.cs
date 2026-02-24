using System;
using System.Reflection;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

/// <summary>
/// Intercepts Vehicle.ToggleEnum() for our custom AfcAutoStage enum.
/// The original method uses cascading type checks that silently ignore
/// unrecognized enum types. Our prefix catches AfcAutoStage and toggles
/// AutoStage.Enabled before the original can discard it.
/// </summary>
[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.ToggleEnum))]
static class Patch_Vehicle_ToggleEnum
{
    static bool Prefix(Enum? enumValue)
    {
        if (enumValue is not AfcAutoStage) return true;

        AutoStage.Enabled = !AutoStage.Enabled;

        if (Mod.DebugMode)
        {
            DefaultCategory.Log.Debug($"[AFC] AutoStage.Enabled = {AutoStage.Enabled}");
        }
        return false;
    }
}

/// <summary>
/// Reports the active state of our gauge button so it renders lit/unlit.
/// Vehicle.IsSet&lt;T&gt; returns false for unrecognized enum types,
/// so without this patch our button would never light up when enabled.
///
/// Targets the closed instantiation IsSet&lt;Enum&gt; because
/// GaugeButtonFlightComputer.PackData() calls IsSet with _enumValue
/// typed as System.Enum.
/// </summary>
[HarmonyPatch]
static class Patch_Vehicle_IsSet
{
    static MethodBase TargetMethod()
    {
        var open = AutoStage.FindGenericMethod("IsSet")
            ?? throw new InvalidOperationException(
                "[AFC] Vehicle.IsSet<T> not found - cannot patch gauge button state.");

        return open.MakeGenericMethod(typeof(Enum));
    }

    static bool Prefix(Enum value, ref bool __result)
    {
        if (value is not AfcAutoStage) return true;

        __result = AutoStage.Enabled;
        return false;
    }
}

/// <summary>
/// Disables (grays out) the AUTOSTAGE button when no burn is planned,
/// matching the behavior of the AUTO/WARP/DELETE burn buttons.
/// Vehicle.IsFlightComputerDisabled&lt;T&gt; returns false for unrecognized
/// enum types (never disabled), so without this patch the button stays
/// active even when there is nothing to auto-stage for.
/// </summary>
[HarmonyPatch]
static class Patch_Vehicle_IsFlightComputerDisabled
{
    static MethodBase TargetMethod()
    {
        var open = AutoStage.FindGenericMethod("IsFlightComputerDisabled")
            ?? throw new InvalidOperationException(
                "[AFC] Vehicle.IsFlightComputerDisabled<T> not found.");

        return open.MakeGenericMethod(typeof(Enum));
    }

    static bool Prefix(Vehicle __instance, Enum value, ref bool __result)
    {
        if (value is not AfcAutoStage) return true;

        __result = __instance.FlightComputer.Burn == null
                || __instance.FlightComputer.Burn.BurnDuration <= 0f;
        return false;
    }
}

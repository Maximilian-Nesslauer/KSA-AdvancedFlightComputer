using System;
using System.Collections.Generic;
using System.Reflection;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features;

/// <summary>
/// Custom enum for the AUTOSTAGE gauge button binding.
/// Injected into GaugeButtonFlightComputer._enumLookup so the XML attribute
/// Action="AfcAutoStage" Value="Enabled" resolves at load time.
/// </summary>
public enum AfcAutoStage { Enabled }

/// <summary>
/// Adds an AUTOSTAGE toggle button to the BurnControl gauge panel.
/// When enabled, automatically activates the next stage if engines
/// run out of propellant during an auto-burn.
/// </summary>
static class AutoStage
{
    /// <summary>Whether auto-staging is currently enabled by the user.</summary>
    public static bool Enabled;

    private static readonly FieldInfo? f_enumLookup =
        AccessTools.Field(typeof(GaugeButtonFlightComputer), "_enumLookup");

    /// <summary>
    /// Injects AfcAutoStage into the gauge button enum lookup dictionary.
    /// Called from [StarMapImmediateLoad] to ensure the enum is available
    /// when GaugeButtonFlightComputer.OnDataLoad() processes our XML patch.
    /// Idempotent - safe to call multiple times.
    /// </summary>
    public static bool InjectEnumLookup()
    {
        if (f_enumLookup == null)
        {
            DefaultCategory.Log.Error(
                "[AFC] GaugeButtonFlightComputer._enumLookup field not found - game version changed?");
            return false;
        }

        if (f_enumLookup.GetValue(null) is not Dictionary<string, Type> dict)
        {
            DefaultCategory.Log.Error("[AFC] _enumLookup is null or unexpected type.");
            return false;
        }

        dict["AfcAutoStage"] = typeof(AfcAutoStage);

        if (Mod.DebugMode)
        {
            DefaultCategory.Log.Debug(
                $"[AFC] Injected AfcAutoStage into _enumLookup ({dict.Count} entries total).");
        }
        return true;
    }

    /// <summary>
    /// Finds an open generic method definition on Vehicle by name.
    /// Used to create closed instantiations (e.g. IsSet&lt;Enum&gt;) for Harmony patching.
    /// </summary>
    public static MethodInfo? FindGenericMethod(string name)
    {
        foreach (var method in typeof(Vehicle).GetMethods())
        {
            if (method.Name == name && method.IsGenericMethodDefinition)
                return method;
        }
        return null;
    }
}

#region Gauge Button Patches

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

#endregion

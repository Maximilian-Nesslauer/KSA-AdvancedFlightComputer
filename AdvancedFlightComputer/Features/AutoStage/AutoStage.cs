using System;
using System.Collections.Generic;
using System.Reflection;
using Brutal.Logging;
using Brutal.Numerics;
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

#region Auto-Staging Logic

/// <summary>
/// Detects propellant depletion during auto-burns and triggers staging.
///
/// FlightComputer.UpdateBurnTarget (background thread) checks whether any
/// active engine has propellant. When none do, it sets BurnMode = Manual,
/// aborting the burn. We detect that Auto-to-Manual transition when the
/// background job results are applied back on the main thread, and call
/// ActivateNextStage to continue the burn with the next stage's engines.
///
/// Hook point: Vehicle.UpdateFromTaskResults runs on the main thread after
/// the VehicleUpdateTask job completes. This is safe because:
/// - ActivateNextStage modifies the part tree (Decoupler.Decouple creates
///   new vehicles, adds to parent.Children) and therefore must be on main thread
/// - Cannot use UpdatePerFrameData: it runs inside UpdatePerFrameDataTree
///   which iterates parent.Children . autostaging would modify that collection
///   mid-iteration (InvalidOperationException)
/// - Cannot use UpdateBurnTarget: it runs on the background worker thread
///
/// Prefix captures the BurnMode before the FC copy overwrites it, giving
/// us a clean transition detection without stale static state.
///
/// After staging, a grace period maintains BurnMode = Auto while
/// Rocket.UpdateRockets propagates IsPropellantAvailable to the newly
/// activated engines. Without this, the next frame's background job would
/// see engines without propellant and set BurnMode = Manual again.
///
/// Why 3 frames (1 needed, 2 margin):
///   Frame N:   Staging happens here. PrepareWorker snapshots the new engine
///              states (IsActive=true, IsPropellantAvailable=false).
///   N -> N+1 job: UpdateBurnTarget reads IsPropellantAvailable=false, sets
///              BurnMode=Manual. Then UpdateModules/Rocket.UpdateRockets
///              checks propellant and sets IsPropellantAvailable=true in the
///              state buffer. (ComputeControl runs before UpdateModules.)
///   Frame N+1: ApplyResults writes BurnMode=Manual AND the updated engine
///              states (IsPropellantAvailable=true) back to the Vehicle.
///              Grace forces BurnMode=Auto. PrepareWorker now snapshots
///              BurnMode=Auto + IsPropellantAvailable=true.
///   N+1 -> N+2 job: Propellant check passes. Burn continues normally.
/// </summary>
[HarmonyPatch(typeof(Vehicle), "UpdateFromTaskResults")]
static class Patch_AutoStageExecution
{
    private static int _graceFrames;

    internal static void Reset()
    {
        _graceFrames = 0;
    }

    static void Prefix(Vehicle __instance, out FlightComputerBurnMode __state)
    {
        __state = __instance.FlightComputer.BurnMode;
    }

    static void Postfix(Vehicle __instance, FlightComputerBurnMode __state)
    {
        if (__instance != Program.ControlledVehicle) return;

        if (!AutoStage.Enabled)
        {
            _graceFrames = 0;
            return;
        }

        FlightComputer fc = __instance.FlightComputer;

        if (fc.Burn == null)
        {
            _graceFrames = 0;
            return;
        }

        bool burnIncomplete = float3.Dot(fc.Burn.DeltaVToGoCci, fc.Burn.DeltaVTargetCci) > 0f;

        if (_graceFrames > 0)
        {
            _graceFrames--;
            if (burnIncomplete && fc.BurnMode == FlightComputerBurnMode.Manual)
                fc.BurnMode = FlightComputerBurnMode.Auto;
            return;
        }

        // __state = BurnMode before the background job's FC copy was applied.
        // fc.BurnMode = BurnMode from the background job (after UpdateBurnTarget).
        // Auto -> Manual with remaining dV means propellant depletion, not completion.
        if (__state == FlightComputerBurnMode.Auto
            && fc.BurnMode == FlightComputerBurnMode.Manual
            && burnIncomplete)
        {
            bool hasNextEngineStage = false;
            foreach (Stage stage in __instance.Parts.StageList.Stages)
            {
                if (!stage.Activated && stage.ContainsEngine)
                {
                    hasNextEngineStage = true;
                    break;
                }
            }

            if (hasNextEngineStage)
            {
                if (Mod.DebugMode)
                {
                    DefaultCategory.Log.Debug(
                        $"[AFC] Auto-staging: dV remaining = {fc.Burn.DeltaVToGoCci.Length():F1} m/s");
                }

                __instance.Parts.StageList.ActivateNextStage(__instance);
                fc.BurnMode = FlightComputerBurnMode.Auto;
                _graceFrames = 3; // see class doc comment for timing analysis
            }
        }
    }
}

#endregion

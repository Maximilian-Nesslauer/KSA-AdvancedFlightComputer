using System;
using System.Collections.Generic;
using System.Reflection;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

/// <summary>
/// Custom enum for the AUTOSTAGE gauge button binding.
/// Injected into GaugeButtonFlightComputer._enumLookup so the XML attribute
/// Action="AfcAutoStage" Value="Enabled" resolves at load time.
/// </summary>
public enum AfcAutoStage { Enabled }

/// <summary>
/// Adds an AUTOSTAGE toggle button to the BurnControl gauge panel.
/// When enabled, automatically activates the next stage whenever active
/// engines lose propellant - during auto-burns, manual burns, or
/// unpowered ascent without a maneuver node.
/// </summary>
static class AutoStage
{
    /// <summary>Whether auto-staging is currently enabled by the user.</summary>
    public static bool Enabled;

    /// <summary>
    /// Checks whether any active engine on the vehicle has propellant available.
    /// Uses ResourceManager.ResourceAvailable - the same check the game performs
    /// in Rocket.UpdateRockets to set IsPropellantAvailable on engine states.
    /// </summary>
    public static bool HasActiveEngineWithPropellant(Vehicle vehicle)
    {
        ReadOnlySpan<MoleState> moleStates = vehicle.Parts.Moles.States;
        foreach (Stage stage in vehicle.Parts.StageList.Stages)
        {
            ReadOnlySpan<Part> parts = stage.Parts;
            for (int i = 0; i < parts.Length; i++)
            {
                Span<EngineController> engines = parts[i].Modules.Get<EngineController>();
                for (int j = 0; j < engines.Length; j++)
                {
                    if (!engines[j].IsActive) continue;
                    foreach (RocketCore core in engines[j].Cores)
                        if (core.ResourceManager != null
                            && core.ResourceManager.ResourceAvailable(moleStates))
                            return true;
                }
            }
        }
        return false;
    }

    /// <summary>
    /// Checks whether the vehicle has an unactivated stage containing engines.
    /// </summary>
    public static bool HasNextEngineStage(Vehicle vehicle)
    {
        foreach (Stage stage in vehicle.Parts.StageList.Stages)
            if (!stage.Activated && stage.ContainsEngine)
                return true;
        return false;
    }

    /// <summary>
    /// Injects AfcAutoStage into the gauge button enum lookup dictionary.
    /// Called from [StarMapImmediateLoad] to ensure the enum is available
    /// when GaugeButtonFlightComputer.OnDataLoad() processes our XML patch.
    /// Idempotent - safe to call multiple times.
    /// </summary>
    public static bool InjectEnumLookup()
    {
        if (GameReflection.GaugeButton_enumLookup == null)
        {
            DefaultCategory.Log.Error(
                "[AFC] GaugeButtonFlightComputer._enumLookup field not found - game version changed?");
            return false;
        }

        if (GameReflection.GaugeButton_enumLookup.GetValue(null) is not Dictionary<string, Type> dict)
        {
            DefaultCategory.Log.Error("[AFC] _enumLookup is null or unexpected type.");
            return false;
        }

        dict["AfcAutoStage"] = typeof(AfcAutoStage);

        if (DebugConfig.AutoStage)
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

    /// <summary>
    /// Applies all AutoStage Harmony patches. Called from Mod.cs
    /// after GameReflection.ValidateAutoStage() passes.
    /// </summary>
    public static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(Patch_Vehicle_ToggleEnum)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_Vehicle_IsSet)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_Vehicle_IsFlightComputerDisabled)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_AutoStageExecution)).Patch();

        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug("[AFC] AutoStage: all patches applied.");
    }
}

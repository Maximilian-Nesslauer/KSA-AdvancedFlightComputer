using System;
using System.Reflection;
using AutoStage.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AutoStage;

// Type marker for GaugeButtonFlightComputer._enumLookup. Value is never read.
public enum AutoStageToggle { Enabled }

[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.ToggleEnum))]
static class Patch_ToggleEnum
{
    static bool Prefix(Enum? enumValue)
    {
        if (enumValue is not AutoStageToggle) return true;

        Mod.AutoStageEnabled = !Mod.AutoStageEnabled;

        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug($"[AutoStage] Enabled = {Mod.AutoStageEnabled}");

        return false;
    }
}

// KittenEva overrides this and forwards anything that is not a KittenEvaAction
// to base, so the patch still answers for an EVA kitten.
[HarmonyPatch]
static class Patch_IsSet
{
    static MethodBase TargetMethod() => GameReflection.Vehicle_IsSet_Enum!;

    static bool Prefix(Enum value, ref bool __result)
    {
        if (value is not AutoStageToggle) return true;

        __result = Mod.AutoStageEnabled;
        return false;
    }
}

// Unlike IsSet, KittenEva's override answers "disabled" for everything that is
// not a KittenEvaAction instead of calling base, so this never runs for an EVA
// kitten. That is the wanted answer anyway, and the AUTOSTAGE button rides on a
// canvas an EVA kitten does not draw.
[HarmonyPatch]
static class Patch_IsFlightComputerDisabled
{
    static MethodBase TargetMethod() => GameReflection.Vehicle_IsFlightComputerDisabled_Enum!;

    static bool Prefix(Vehicle __instance, Enum value, ref bool __result)
    {
        if (value is not AutoStageToggle) return true;

        __result = !StagingHelpers.HasNextEngineSequence(__instance);
        return false;
    }
}

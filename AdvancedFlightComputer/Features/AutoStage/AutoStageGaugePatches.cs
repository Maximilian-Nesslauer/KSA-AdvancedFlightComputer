using System.Reflection;
using AdvancedFlightComputer.Core;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

internal static class AutoStageGaugePatches
{
    [HarmonyPatch(typeof(Vehicle), nameof(Vehicle.ToggleEnum), new[] { typeof(Enum) })]
    internal static class TogglePatch
    {
        static bool Prefix(Vehicle __instance, Enum? enumValue)
        {
            if (enumValue is not AfcAutoStageToggle)
                return true;
            StagingDetector.Arm(__instance, !StagingDetector.IsArmed(__instance));
            return false;
        }
    }

    // KittenEva's override forwards anything that is not a KittenEvaAction to base, so this still
    // answers for an EVA kitten.
    [HarmonyPatch]
    internal static class IsSetPatch
    {
        static MethodBase TargetMethod() => GameReflection.Vehicle_IsSet_Enum!;

        static bool Prefix(Vehicle __instance, Enum value, ref bool __result)
        {
            if (value is not AfcAutoStageToggle)
                return true;
            __result = StagingDetector.IsArmed(__instance);
            return false;
        }
    }

    // KittenEva's override answers "disabled" for everything but its own actions without calling
    // base, which is the wanted answer: an EVA kitten draws no engine panel.
    [HarmonyPatch]
    internal static class IsDisabledPatch
    {
        static MethodBase TargetMethod() => GameReflection.Vehicle_IsFlightComputerDisabled_Enum!;

        // Stays enabled while armed, so the player can always switch it off, and while a pure
        // jettison row is pending, which is what the spent-stage drop stages. The jettison
        // rebuild runs from the draw pass while solvers are queued; that is safe because the
        // worker path snapshots PartTree.Parts itself and never reads the lazy Sequence.Parts cache.
        static bool Prefix(Vehicle __instance, Enum value, ref bool __result)
        {
            if (value is not AfcAutoStageToggle)
                return true;
            __result = !StagingDetector.IsArmed(__instance)
                       && !StagingHelpers.HasNextEngineSequence(__instance)
                       && !(StagingConfig.DropSpentStages
                            && JettisonAnalysis.GetPendingJettison(__instance, StagingDetector.StateOf(__instance)) != null);
            return false;
        }
    }
}

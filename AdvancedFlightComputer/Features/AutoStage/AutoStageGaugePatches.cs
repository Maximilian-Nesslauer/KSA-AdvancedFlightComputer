using System.Reflection;
using AdvancedFlightComputer.Core;
using Brutal.Numerics;
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

    // KittenEva's override forwards anything that is not a KittenEvaAction to base, so this still answers for an EVA kitten.
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

    // KittenEva's override answers "disabled" for everything but its own actions without calling base, which is the wanted answer: an EVA kitten draws no engine panel.
    [HarmonyPatch]
    internal static class IsDisabledPatch
    {
        static MethodBase TargetMethod() => GameReflection.Vehicle_IsFlightComputerDisabled_Enum!;

        static bool Prefix(Vehicle __instance, Enum value, ref bool __result)
        {
            if (value is not AfcAutoStageToggle)
                return true;
            __result = IsToggleDisabled(__instance);
            return false;
        }
    }

    // The gauge button reaches IsSet<T> and IsFlightComputerDisabled<T> through generic calls that do not always run the patched bodies above, and stock then answers "never lit".
    // So the button itself is answered as well, the same way RcsGaugePatches answers the Auto burn button.
    [HarmonyPatch(typeof(GaugeButtonFlightComputer), nameof(GaugeButtonFlightComputer.IsDisabled), new Type[0])]
    internal static class ButtonIsDisabledPatch
    {
        static void Postfix(GaugeButtonFlightComputer __instance, ref bool __result)
        {
            if (IsAutoStageButton(__instance) && Program.ControlledVehicle is { } vehicle && vehicle is not KittenEva)
                __result = IsToggleDisabled(vehicle);
        }
    }

    [HarmonyPatch(typeof(GaugeButtonFlightComputer), nameof(GaugeButtonFlightComputer.PackData), new Type[0])]
    internal static class ButtonPackDataPatch
    {
        // Bit 0 is clicked, bit 1 lit, bit 2 disabled and bit 3 enabled, as stock packs them. Stock already folds its own disabled answer into bit 0.
        static void Postfix(GaugeButtonFlightComputer __instance, ref uint2 __result)
        {
            if (!IsAutoStageButton(__instance) || Program.ControlledVehicle is not { } vehicle || vehicle is KittenEva)
                return;
            bool disabled = IsToggleDisabled(vehicle);
            uint bits = disabled ? 0b0100u : (__result.X & 0b0001u) | 0b1000u;
            if (StagingDetector.IsArmed(vehicle))
                bits |= 0b0010u;
            __result = new uint2(bits, __result.Y);
        }
    }

    private static bool IsAutoStageButton(GaugeButtonFlightComputer button) =>
        GameReflection.GaugeButtonFlightComputer_enumValueRef?.Invoke(button) is AfcAutoStageToggle;

    // Stays enabled while armed, so the player can always switch it off, and while a pure jettison row is pending, which is what the spent-stage drop stages.
    // The jettison rebuild runs from the draw pass while solvers are queued; that is safe because the worker path snapshots PartTree.Parts itself and never reads the lazy Sequence.Parts cache.
    private static bool IsToggleDisabled(Vehicle vehicle) =>
        !StagingDetector.IsArmed(vehicle)
        && !StagingHelpers.HasNextEngineSequence(vehicle)
        && !(StagingConfig.DropSpentStages
             && JettisonAnalysis.GetPendingJettison(vehicle, StagingDetector.StateOf(vehicle)) != null);
}

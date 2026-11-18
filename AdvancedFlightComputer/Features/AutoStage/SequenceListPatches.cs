using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

internal static class SequenceListPatches
{
    // The staging key and kFS use this stock path. AutoStage fires its modules separately.
    // The row's engines are read before stock marks the row activated, so a row that lights engines during an Auto burn can hold that burn, see StagingDetector.OnPlayerStaged.
    [HarmonyPatch(typeof(SequenceList), nameof(SequenceList.ActivateNextSequence), new[] { typeof(Vehicle) })]
    internal static class ActivateNextSequencePatch
    {
        static void Prefix(SequenceList __instance, Vehicle vehicle, out PlayerStaging __state)
        {
            __state = new PlayerStaging(__instance.ActiveSequence,
                vehicle?.FlightComputer.BurnMode == FlightComputerBurnMode.Auto
                    ? StagingHelpers.EnginesLitByNextRow(__instance)
                    : null);
            if (vehicle == null || StagingHooks.NextRowToFire(__instance) is not Sequence next
                || next.Number == __state.ActiveSequence)
                return;

            ReadOnlySpan<Part> parts = next.Parts;
            for (int i = parts.Length - 1; i >= 0; i--)
                foreach (ISequenced module in parts[i].InSequence(next.Number))
                    if (module is Decoupler decoupler)
                        StagingHooks.RaiseDecouplerActivating(vehicle, decoupler);
        }

        // Stock activates a row only when that changes ActiveSequence.
        static void Postfix(SequenceList __instance, Vehicle vehicle, PlayerStaging __state)
        {
            StagingHelpers.InvalidateSequenceCache();
            if (__state.Engines != null && vehicle != null && __instance.ActiveSequence != __state.ActiveSequence)
                StagingDetector.OnPlayerStaged(vehicle, __state.Engines);
        }
    }

    internal readonly record struct PlayerStaging(int ActiveSequence, List<EngineController>? Engines);

    // The staging window's drag-drop still works in flight and runs through Part.SetSequence, which activates nothing, so ResetCaches is the one place that sees every one of those edits.
    [HarmonyPatch(typeof(SequenceList), nameof(SequenceList.ResetCaches), new Type[0])]
    internal static class ResetCachesPatch
    {
        static void Postfix() => StagingHelpers.InvalidateSequenceCache();
    }
}

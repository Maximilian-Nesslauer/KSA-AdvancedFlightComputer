using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

internal static class SequenceListPatches
{
    [HarmonyPatch(typeof(SequenceList), nameof(SequenceList.ActivateNextSequence), new[] { typeof(Vehicle) })]
    internal static class ActivateNextSequencePatch
    {
        static void Postfix() => StagingHelpers.InvalidateSequenceCache();
    }

    // The staging window's drag-drop still works in flight and runs through Part.SetSequence, which
    // activates nothing, so ResetCaches is the one place that sees every one of those edits.
    [HarmonyPatch(typeof(SequenceList), nameof(SequenceList.ResetCaches), new Type[0])]
    internal static class ResetCachesPatch
    {
        static void Postfix() => StagingHelpers.InvalidateSequenceCache();
    }
}

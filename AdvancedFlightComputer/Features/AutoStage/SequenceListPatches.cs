using HarmonyLib;
using KSA;

namespace AutoStage;

[HarmonyPatch(typeof(SequenceList), nameof(SequenceList.ActivateNextSequence))]
static class Patch_SequenceList_ActivateNextSequence
{
    static void Postfix() => StagingHelpers.InvalidateSequenceCache();
}

/// <summary>
/// The staging window is shared between the editor and flight, and its
/// drag-drop is only partly flight-gated: a player can still reorder sequences
/// and move parts between them mid-flight. Those edits run through
/// Part.SetSequence, which activates nothing and leaves the part count alone,
/// so ResetCaches is the one place that sees every one of them.
/// </summary>
[HarmonyPatch(typeof(SequenceList), nameof(SequenceList.ResetCaches))]
static class Patch_SequenceList_ResetCaches
{
    static void Postfix() => StagingHelpers.InvalidateSequenceCache();
}

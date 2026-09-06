using HarmonyLib;

namespace AdvancedFlightComputer.Core;

internal static class TranspilerInsertion
{
    internal static void MoveEntryMarkers(CodeInstruction anchor, CodeInstruction first)
    {
        first.labels.AddRange(anchor.labels);
        anchor.labels.Clear();

        // Begin markers apply before an instruction, while end markers apply after it. Keep end markers on the anchor so the inserted call stays inside its exception region.
        foreach (ExceptionBlock block in anchor.blocks)
        {
            if (block.blockType != ExceptionBlockType.EndExceptionBlock)
                first.blocks.Add(block);
        }
        anchor.blocks.RemoveAll(block => block.blockType != ExceptionBlockType.EndExceptionBlock);
    }
}

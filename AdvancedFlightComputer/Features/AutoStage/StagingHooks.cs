using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

// Tells other features that a staging row is about to fire, whatever fired it: AutoStage's own triggers (a guidance request, a burnout, a spent drop) or the player's staging key.
// Raised before the row's modules activate, while what it will separate is still attached and can be recorded.
internal static class StagingHooks
{
    internal static event Action<Vehicle, Sequence>? SequenceActivating;

    internal static void RaiseSequenceActivating(Vehicle vehicle, Sequence sequence)
    {
        Action<Vehicle, Sequence>? handlers = SequenceActivating;
        if (handlers == null || vehicle == null || sequence == null)
            return;
        // A listener's fault must never cost the staging itself.
        try
        {
            handlers(vehicle, sequence);
        }
        catch (Exception e)
        {
            DefaultCategory.Log.Error($"[AFC] A staging listener failed on '{vehicle.Id}', sequence {sequence.Number}: {e}");
        }
    }

    // The row stock's ActivateNextSequence would fire next: the first one not yet activated that has parts, as its own walk finds it.
    internal static Sequence? NextRowToFire(SequenceList seqList)
    {
        ReadOnlySpan<Sequence> sequences = seqList.Sequences;
        for (int i = 0; i < sequences.Length; i++)
        {
            if (!sequences[i].Activated && !sequences[i].Parts.IsEmpty)
                return sequences[i];
        }
        return null;
    }
}

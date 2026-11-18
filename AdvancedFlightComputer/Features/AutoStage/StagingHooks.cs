using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

// Announces a decoupler being activated while its connection still identifies the departing subtree.
internal static class StagingHooks
{
    internal static event Action<Vehicle, Decoupler>? DecouplerActivating;

    // Match Decoupler.SetIsActive so planning and activation agree on what can separate.
    internal static bool CanActivate(Decoupler decoupler)
        => decoupler.IsEnabled && !decoupler.IsActive
           && decoupler.Connector?.Connection?.OtherPart(decoupler.Parent.FullPart) != null;

    internal static void RaiseDecouplerActivating(Vehicle vehicle, Decoupler decoupler)
    {
        Action<Vehicle, Decoupler>? handlers = DecouplerActivating;
        if (handlers == null || !CanActivate(decoupler))
            return;
        // A listener's fault must never cost the staging itself.
        try
        {
            handlers(vehicle, decoupler);
        }
        catch (Exception e)
        {
            DefaultCategory.Log.Error($"[AFC] A staging listener failed on '{vehicle.Id}', decoupler '{decoupler.TemplateId}': {e}");
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

using AdvancedFlightComputer.Core;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

// Which parts the next row would throw overboard, without activating it. Mirrors Vehicle.Split: a
// decoupler sheds the subtree below the child side of its connection. Only the shape is cached,
// per vehicle, because only the shape is fixed between tree and sequence edits.
internal static class JettisonAnalysis
{
    // Scratch for the control-loss guard, which is not cached.
    private static readonly HashSet<Part> _guardSet = new();

    // Null when the row is not a pure jettison: it lights an engine, or sheds nothing predictable.
    public static IReadOnlySet<Part>? GetPendingJettison(Vehicle vehicle, StagingState state)
    {
        int generation = StagingHelpers.SequenceGeneration;
        // The part count catches tree changes the sequence generation misses, such as a decouple from a part menu.
        int partCount = vehicle.Parts.Count;
        int sequenceNumber = vehicle.Parts.SequenceList.GetNextSequenceNumber();

        if (state.JettisonGeneration != generation
            || state.JettisonPartCount != partCount
            || state.JettisonSequenceNumber != sequenceNumber)
        {
            // The key is written before the rebuild, so a throw mid-rebuild leaves the set
            // unusable until the inputs change instead of throwing again every frame.
            state.JettisonGeneration = generation;
            state.JettisonPartCount = partCount;
            state.JettisonSequenceNumber = sequenceNumber;
            state.JettisonValid = false;
            state.JettisonValid = Rebuild(vehicle, state.JettisonSet);
        }

        return state.JettisonValid ? state.JettisonSet : null;
    }

    private static bool Rebuild(Vehicle vehicle, HashSet<Part> jettisonSet)
    {
        jettisonSet.Clear();

        SequenceList seqList = vehicle.Parts.SequenceList;
        int number = seqList.GetNextSequenceNumber();
        if (number < 0)
            return Decline(vehicle, "nothing left to stage");

        Sequence? target = FindSequence(seqList, number);
        if (target == null)
            return Decline(vehicle, $"sequence {number} is not in the list");

        bool anyDecoupler = false;
        ReadOnlySpan<Part> parts = target.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            foreach (ISequenced module in parts[i].InSequence(number))
            {
                if (module is EngineController)
                    return Decline(vehicle, $"sequence {number} lights an engine");

                if (module is not Decoupler decoupler)
                    continue;

                switch (GetJettisonedRoot(decoupler, out Part? root))
                {
                    case Separation.Predicted:
                        anyDecoupler = true;
                        AddSubtree(root!, jettisonSet);
                        break;
                    case Separation.None:
                        break;
                    case Separation.Unpredictable:
                        return Decline(vehicle, $"sequence {number} has a decoupler whose separation cannot be predicted");
                }
            }
        }

        if (!anyDecoupler)
            return Decline(vehicle, $"sequence {number} separates nothing");

        if (DebugConfig.AutoStage)
        {
            int engines = 0;
            foreach (Part part in jettisonSet)
                engines += part.SubtreeModules.Get<EngineController>().Length;
            DefaultCategory.Log.Debug(
                $"[AFC] Spent-stage jettison armed on '{vehicle.Id}': sequence {number} would shed {jettisonSet.Count} part(s) carrying {engines} engine(s).");
        }

        return true;
    }

    // True when the next row that would fire separates every control module from the vehicle.
    // Staging past the last control module is a legitimate thing to want, not an automatic one.
    public static bool WouldSeparateLastControl(Vehicle vehicle)
    {
        Sequence? next = null;
        foreach (Sequence sequence in vehicle.Parts.SequenceList.Sequences)
        {
            if (!sequence.Activated && !sequence.Parts.IsEmpty)
            {
                next = sequence;
                break;
            }
        }
        if (next == null)
            return false;

        _guardSet.Clear();
        ReadOnlySpan<Part> parts = next.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            foreach (ISequenced module in parts[i].InSequence(next.Number))
            {
                if (module is Decoupler decoupler && GetJettisonedRoot(decoupler, out Part? root) == Separation.Predicted)
                    AddSubtree(root!, _guardSet);
            }
        }
        if (_guardSet.Count == 0)
            return false;

        Span<Control> controls = vehicle.Parts.Modules.Get<Control>();
        if (controls.Length == 0)
            return false;
        for (int i = 0; i < controls.Length; i++)
        {
            if (!_guardSet.Contains(controls[i].Parent.FullPart))
                return false;
        }
        return true;
    }

    private static Sequence? FindSequence(SequenceList seqList, int number)
    {
        foreach (Sequence sequence in seqList.Sequences)
        {
            if (sequence.Number == number)
                return sequence;
        }
        return null;
    }

    private static bool Decline(Vehicle vehicle, string reason)
    {
        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug($"[AFC] No spent-stage jettison on '{vehicle.Id}': {reason}.");
        return false;
    }

    // The crossfeed case: a booster whose own engine is spent may still feed the core. Live, not
    // cached, so the caller runs it as the last gate before staging.
    public static bool CarriesOffUsablePropellant(Vehicle vehicle, IReadOnlySet<Part> jettison, out string? reason)
    {
        reason = null;
        ReadOnlySpan<MoleState> moleStates = vehicle.Parts.Moles.States;

        foreach (Part part in jettison)
        {
            // SubtreeModules, because a tank usually sits on a sub-part of the tree part.
            Span<Tank> tanks = part.SubtreeModules.Get<Tank>();
            for (int i = 0; i < tanks.Length; i++)
            {
                Tank tank = tanks[i];
                if (tank.ComputeSubstanceMass(moleStates) <= 0f)
                    continue;

                // AvailableConsumers is narrower than what a FurtherestToNearest flow rule drains,
                // which is fine on stock parts, whose decoupler joints carry no BulkFluid capability.
                foreach ((ResourceManager manager, int _) in tank.AvailableConsumers)
                {
                    if (manager.Consumer is not Combustor consumer)
                        continue;
                    if (jettison.Contains(consumer.Parent.FullPart))
                        continue;

                    reason = $"a tank on '{part.DisplayName}' still feeds '{consumer.Parent.FullPart.DisplayName}', which stays aboard";
                    return true;
                }
            }
        }

        // A player-drawn fuel link is plumbing the resource graph does not necessarily model.
        ReadOnlySpan<FuelLink> links = vehicle.Parts.FuelLinks.Links;
        for (int i = 0; i < links.Length; i++)
        {
            FuelLink link = links[i];
            if (!link.Enabled)
                continue;
            if (jettison.Contains(link.PartA.FullPart) == jettison.Contains(link.PartB.FullPart))
                continue;

            reason = $"an enabled fuel link joins '{link.PartA.FullPart.DisplayName}' and '{link.PartB.FullPart.DisplayName}' across the separation";
            return true;
        }

        return false;
    }

    private enum Separation { Predicted, None, Unpredictable }

    private static Separation GetJettisonedRoot(Decoupler decoupler, out Part? root)
    {
        root = null;

        // Mirrors the guard in Decoupler.SetIsActive. IsActive never flips, so a spent decoupler
        // is recognised by its connector having lost the connection.
        if (!decoupler.IsEnabled)
            return Separation.None;

        Part.Connection? connection = decoupler.Connector.Connection;
        if (connection == null)
            return Separation.None;

        // Raw parts, not FullPart: Vehicle.Split tests these same two.
        Part near = decoupler.Connector.ConnectionPart;
        Part far = connection.OtherPart(near);

        // Vehicle.Split keeps whichever endpoint is not a tree child of the other. When neither or
        // both are, its pick depends on connector order, so no side is predicted.
        bool nearIsParent = near.TreeChildren.Contains(far);
        bool farIsParent = far.TreeChildren.Contains(near);
        if (nearIsParent == farIsParent)
            return Separation.Unpredictable;

        root = nearIsParent ? far : near;
        return Separation.Predicted;
    }

    private static void AddSubtree(Part root, HashSet<Part> into)
    {
        into.Add(root);
        PartTreeChildrenIterator iterator = new PartTreeChildrenIterator(root);
        while (true)
        {
            Part? node = iterator.GetNextNode();
            if (node == null)
                break;
            into.Add(node);
        }
    }
}

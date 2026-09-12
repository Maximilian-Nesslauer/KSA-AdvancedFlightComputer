using System;
using System.Collections.Generic;
using AutoStage.Core;
using Brutal.Logging;
using KSA;

namespace AutoStage;

/// <summary>
/// Which parts the next row would throw overboard, without activating it.
/// Mirrors Vehicle.Split: a decoupler sheds the subtree below the child side of
/// its connection. Only the shape is cached, since only the shape is fixed
/// between tree and sequence edits; whether shedding it is safe moves every
/// frame and is answered live.
/// </summary>
static class JettisonAnalysis
{
    private static readonly HashSet<Part> _jettisonSet = new();

    private static Vehicle? _cachedVehicle;
    private static int _cachedGeneration = -1;
    private static int _cachedPartCount = -1;
    private static int _cachedSequenceNumber = -1;
    private static bool _cachedValid;

    /// <summary>
    /// Null when the row is not a pure jettison: it lights an engine, or sheds
    /// nothing predictable. The set is a buffer the next rebuild reuses.
    /// </summary>
    public static IReadOnlySet<Part>? GetPendingJettison(Vehicle vehicle)
    {
        int generation = StagingHelpers.SequenceGeneration;
        // Part count catches tree changes the sequence generation misses, such
        // as a decouple the player triggered from a part menu.
        int partCount = vehicle.Parts.Count;
        // Which sequence is pending is the analysis' actual subject, so it is
        // read rather than inferred from an invalidation hook. Patching every
        // path that can activate a sequence is a claim about the whole game;
        // this is a claim about one short loop over the sequence list.
        int sequenceNumber = vehicle.Parts.SequenceList.GetNextSequenceNumber();

        if (_cachedVehicle != vehicle
            || _cachedGeneration != generation
            || _cachedPartCount != partCount
            || _cachedSequenceNumber != sequenceNumber)
        {
            // Invalidate before rebuilding, so a throw mid-rebuild leaves a
            // half-built set marked unusable instead of armed.
            _cachedValid = false;
            _cachedValid = Rebuild(vehicle);
            _cachedVehicle = vehicle;
            _cachedGeneration = generation;
            _cachedPartCount = partCount;
            _cachedSequenceNumber = sequenceNumber;
        }

        return _cachedValid ? _jettisonSet : null;
    }

    private static bool Rebuild(Vehicle vehicle)
    {
        _jettisonSet.Clear();

        SequenceList seqList = vehicle.Parts.SequenceList;
        int number = seqList.GetNextSequenceNumber();
        if (number < 0)
            return Decline("nothing left to stage");

        Sequence? target = FindSequence(seqList, number);
        if (target == null)
            return Decline($"sequence {number} is not in the list");

        bool anyDecoupler = false;
        ReadOnlySpan<Part> parts = target.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            // Only what this row fires: a part is listed here for any of its
            // modules, so judging by the part would see a later row's motor.
            foreach (ISequenced module in parts[i].InSequence(number))
            {
                // Lighting the next stage is a staging decision, not shedding
                // dead weight.
                if (module is EngineController)
                    return Decline($"sequence {number} lights an engine");

                if (module is not Decoupler decoupler)
                    continue;

                switch (GetJettisonedRoot(decoupler, out Part? root))
                {
                    case Separation.Predicted:
                        anyDecoupler = true;
                        AddSubtree(root!);
                        break;
                    case Separation.None:
                        break;
                    case Separation.Unpredictable:
                        // One ambiguous decoupler makes the whole verdict wrong,
                        // because the set would be missing whatever it takes.
                        return Decline($"sequence {number} has a decoupler whose "
                                       + "separation cannot be predicted");
                }
            }
        }

        if (!anyDecoupler)
            return Decline($"sequence {number} separates nothing");

        if (DebugConfig.AutoStage)
        {
            int engines = 0;
            foreach (Part part in _jettisonSet)
                engines += part.SubtreeModules.Get<EngineController>().Length;
            DefaultCategory.Log.Debug(
                $"[AutoStage] Spent-stage jettison armed: sequence {number} would shed "
                + $"{_jettisonSet.Count} part(s) carrying {engines} engine(s).");
        }

        return true;
    }

    private static Sequence? FindSequence(SequenceList seqList, int number)
    {
        foreach (Sequence sequence in seqList.Sequences)
        {
            if (sequence.Number == number) return sequence;
        }
        return null;
    }

    // Declining is the normal case, so it is logged only under the debug flag,
    // where "the boosters rode along" would otherwise look identical to a
    // correct refusal.
    private static bool Decline(string reason)
    {
        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug($"[AutoStage] No spent-stage jettison: {reason}.");
        return false;
    }

    /// <summary>
    /// The crossfeed case: a booster whose own engine is spent may still feed the
    /// core. Live, not cached, so the caller runs it as the last gate before
    /// staging. Reports the reason so the caller can log it once per transition.
    /// </summary>
    public static bool CarriesOffUsablePropellant(Vehicle vehicle, IReadOnlySet<Part> jettison,
        out string? reason)
    {
        reason = null;
        ReadOnlySpan<MoleState> moleStates = vehicle.Parts.Moles.States;

        foreach (Part part in jettison)
        {
            // SubtreeModules, not Modules: the set holds tree parts, and a tank
            // usually sits on one of their sub-parts.
            Span<Tank> tanks = part.SubtreeModules.Get<Tank>();
            for (int i = 0; i < tanks.Length; i++)
            {
                Tank tank = tanks[i];
                if (tank.ComputeSubstanceMass(moleStates) <= 0f) continue;

                // AvailableConsumers is narrower than what a FurtherestToNearest
                // flow rule actually drains, so this can miss a cross-stage feed.
                // Not on stock parts: their decoupler joints carry no BulkFluid
                // capability, so no fluid graph crosses a separation.
                foreach ((ResourceManager manager, int _) in tank.AvailableConsumers)
                {
                    if (manager.Consumer is not Combustor consumer) continue;
                    if (jettison.Contains(consumer.Parent.FullPart)) continue;

                    reason = $"a tank on '{part.DisplayName}' still feeds "
                             + $"'{consumer.Parent.FullPart.DisplayName}', which stays aboard";
                    return true;
                }
            }
        }

        // A player-drawn fuel link is plumbing the resource graph does not
        // necessarily model, so a link across the cut blocks on its own.
        ReadOnlySpan<FuelLink> links = vehicle.Parts.FuelLinks.Links;
        for (int i = 0; i < links.Length; i++)
        {
            FuelLink link = links[i];
            if (!link.Enabled) continue;
            if (jettison.Contains(link.PartA.FullPart)
                == jettison.Contains(link.PartB.FullPart)) continue;

            reason = $"an enabled fuel link joins '{link.PartA.FullPart.DisplayName}' and "
                     + $"'{link.PartB.FullPart.DisplayName}' across the separation";
            return true;
        }

        return false;
    }

    private enum Separation { Predicted, None, Unpredictable }

    private static Separation GetJettisonedRoot(Decoupler decoupler, out Part? root)
    {
        root = null;

        // Mirrors the guard in Decoupler.SetIsActive, so the prediction matches
        // what staging really sheds. IsActive never flips, so a spent decoupler
        // is recognised by its connector having lost the connection.
        if (!decoupler.IsEnabled)
            return Separation.None;

        Part.Connection? connection = decoupler.Connector.Connection;
        if (connection == null)
            return Separation.None;

        // Raw, not FullPart: Vehicle.Split tests these same two, so normalizing
        // would predict a side where stock's pick depends on connector order.
        Part near = decoupler.Connector.ConnectionPart;
        Part far = connection.OtherPart(near);

        // Vehicle.Split keeps whichever endpoint is not a tree child of the
        // other, which only names a side when exactly one of them is the tree
        // parent. On any other connection its pick depends on connector order,
        // so say so rather than guess at what would leave.
        bool nearIsParent = near.TreeChildren.Contains(far);
        bool farIsParent = far.TreeChildren.Contains(near);
        if (nearIsParent == farIsParent)
            return Separation.Unpredictable;

        root = nearIsParent ? far : near;
        return Separation.Predicted;
    }

    private static void AddSubtree(Part root)
    {
        _jettisonSet.Add(root);
        PartTreeChildrenIterator iterator = new PartTreeChildrenIterator(root);
        while (true)
        {
            Part? node = iterator.GetNextNode();
            if (node == null) break;
            _jettisonSet.Add(node);
        }
    }

    internal static void ForgetVehicle(Vehicle vehicle)
    {
        if (_cachedVehicle == vehicle)
            Reset();
    }

    internal static void Reset()
    {
        _jettisonSet.Clear();
        _cachedVehicle = null;
        _cachedGeneration = -1;
        _cachedPartCount = -1;
        _cachedSequenceNumber = -1;
        _cachedValid = false;
    }
}

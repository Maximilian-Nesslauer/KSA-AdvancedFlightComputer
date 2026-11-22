using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

// A stock aeroshroud has no release module of its own, so it leaves on the radial decoupler it sits on.
// A row that holds only such mounts is released by pressure and clearance, apart from engine staging.
// A row that also holds anything else stays with the stock sequence.
internal static class FairingJettison
{
    private static readonly EditorTag Structural = new("Structural");
    private static readonly HashSet<string> ShroudIds = new(StringComparer.Ordinal)
    {
        "CoreFairingA_Prefab_AeroshroudSmallV1A", "CoreFairingA_Prefab_AeroshroudSmallV2A",
        "CoreFairingA_Prefab_AeroshroudMediumV1A", "CoreFairingA_Prefab_AeroshroudMediumV2A",
        "CoreFairingA_Prefab_AeroshroudLargeV1A", "CoreFairingA_Prefab_AeroshroudLargeV2A",
        "CoreFairingA_Prefab_AeroshroudLargeV3A",
    };

    internal sealed record Row(Sequence Sequence, (Decoupler Mount, Part Shroud)[] Leaves, bool Mixed)
    {
        public int Number => Sequence.Number;
        public double MassKg => Leaves.Sum(leaf => leaf.Shroud.ComputeSubtreeInertMass());
    }

    internal static bool IsStockShroud(PartTemplate template)
        => ShroudIds.Contains(template.Id) && template.HasEditorTag(Structural)
            && template.HasEditorTag(EditorTag.Radial) && !template.HasEditorTag(EditorTag.Interstage)
            && !template.HasEditorTag(EditorTag.Booster);

    private static bool HasOnlyJoint(Part part, Part.Connection joint)
    {
        foreach (Part.Connection connection in part.Connections)
            if (connection != joint)
                return false;
        foreach (Part child in part.SubParts)
            if (!HasOnlyJoint(child, joint))
                return false;
        return true;
    }

    internal static Row? Inspect(Vehicle vehicle, Sequence sequence)
    {
        var leaves = new List<(Decoupler, Part)>();
        bool containsShroud = false, other = false;
        foreach (Part part in sequence.Parts)
        {
            foreach (ISequenced module in part.InSequence(sequence.Number))
            {
                if (module is Decoupler { Connector.Connection: null })
                    continue;
                if (module is not Decoupler mount || !StagingHooks.CanActivate(mount))
                {
                    other = true;
                    continue;
                }
                Part? root = JettisonAnalysis.PredictedJettisonRoot(mount);
                if (root == null)
                {
                    Part? peer = mount.Connector.Connection?.OtherPart(mount.Parent.FullPart);
                    containsShroud |= peer != null && IsStockShroud(peer.Template);
                    other = true;
                    continue;
                }
                if (!ContainsShroud(root))
                {
                    other = true;
                    continue;
                }
                containsShroud = true;
                // Only a stock radial mount with a single inert shroud branch leaves the retained structure intact.
                Part owner = mount.Parent.FullPart;
                string mountId = owner.Template.Id;
                if (!IsStockShroud(root.Template) || root.TreeChildren.Count != 0 || root.TreeParent != owner
                    || mountId is not ("CoreStructuralA_Prefab_RadialDecouplerSmallA" or "CoreStructuralA_Prefab_RadialDecouplerLargeA")
                    || !ReferenceEquals(owner.Tree, vehicle.Parts)
                    || root.SubtreeModules.Get<EngineController>().Length != 0
                    || root.SubtreeModules.Get<Tank>().Length != 0
                    || root.SubtreeModules.Get<Control>().Length != 0
                    || !HasOnlyJoint(root, mount.Connector.Connection!)
                    || leaves.Any(leaf => leaf.Item2 == root))
                {
                    other = true;
                    continue;
                }
                leaves.Add((mount, root));
            }
        }
        return containsShroud ? new Row(sequence, leaves.ToArray(), other) : null;
    }

    private static bool ContainsShroud(Part root)
    {
        if (IsStockShroud(root.Template))
            return true;
        var children = new PartTreeChildrenIterator(root);
        while (children.GetNextNode() is { } child)
            if (IsStockShroud(child.Template))
                return true;
        return false;
    }

    internal static Row[] Rows(Vehicle vehicle, StagingState state)
    {
        int generation = StagingHelpers.SequenceGeneration;
        int parts = vehicle.Parts.Count;
        if (state.FairingRows != null && state.FairingRowsGeneration == generation && state.FairingRowsPartCount == parts)
            return state.FairingRows;
        var rows = new List<Row>();
        // Activated rows count too. SequenceList.SetActiveSequence marks every row up to the active one,
        // so a fairing row that engine staging passed reads as activated while its shrouds are still attached.
        foreach (Sequence sequence in vehicle.Parts.SequenceList.Sequences)
            if (Inspect(vehicle, sequence) is { } row)
                rows.Add(row);
        state.FairingRowsGeneration = generation;
        state.FairingRowsPartCount = parts;
        return state.FairingRows = rows.ToArray();
    }

    internal static bool IsDeferred(Row[] rows, Sequence sequence)
    {
        foreach (Row row in rows)
            if (!row.Mixed && ReferenceEquals(row.Sequence, sequence))
                return true;
        return false;
    }

    // The row stock's SequenceList.ActivateNextSequence would fire, with the deferred fairing rows passed over.
    internal static Sequence? NextRowOutside(SequenceList list, Row[] deferred)
    {
        foreach (Sequence sequence in list.Sequences)
            if (!sequence.Activated && !sequence.Parts.IsEmpty && !IsDeferred(deferred, sequence))
                return sequence;
        return null;
    }

    internal static Sequence? NextAutomaticSequence(Vehicle vehicle, StagingState state)
        => NextRowOutside(vehicle.Parts.SequenceList, Rows(vehicle, state));

    internal static bool Observe(Vehicle vehicle, StagingState state, double now)
    {
        var orbit = vehicle.Orbit;
        var parent = orbit?.Parent;
        if (parent != state.FairingBody || vehicle.Situation.HasAnyContact())
        {
            state.FairingBody = parent;
            state.FairingGate.Reset();
        }
        if (parent == null || !vehicle.HasLaunched || vehicle.Situation.HasAnyContact())
            return false;
        var r = orbit!.StateVectors.PositionCci;
        var v = orbit.StateVectors.VelocityCci;
        PhysicsEnvironment env = vehicle.PhysicsEnvironment;
        if (!env.InPhysicsRadius || !ReferenceEquals(env.ClosestParent, parent) || !(env.TerrainRadius > 0.0))
            return false;
        bool air = parent.GetPhysicalAtmosphere() is { Height: > 0.0 };
        double speed = vehicle.GetSurfaceSpeed();
        double q = 0.5 * env.AtmosphericDensity * speed * speed;
        // The whole vehicle clears the terrain below it, so the bounding sphere comes off the centre's height.
        double clearance = r.Length() - env.TerrainRadius - vehicle.BoundingSphereRadiusBody;
        state.FairingGateQ = q;
        state.FairingGateClearance = clearance;
        double floor = BodyDefaults.DefaultOf(GuidanceSettingSpecs.FairingAltitudeKm, parent.Id);
        double ceiling = BodyDefaults.DefaultOf(GuidanceSettingSpecs.FairingQKpa, parent.Id);
        if (VehicleAutopilotState.TryGet(vehicle, out var settings))
        {
            floor = settings.FairingAltitudeKm;
            ceiling = settings.FairingQKpa;
        }
        return state.FairingGate.Sample(now, q, clearance, double3.Dot(r, v) > 0.0, air,
            Math.Clamp(floor, 0.0, 250.0) * 1000.0, Math.Clamp(ceiling, 0.0, 50.0) * 1000.0);
    }

    internal static void TryRelease(Vehicle vehicle, StagingState state, double now)
    {
        if (state.FairingPending != null)
            return;
        foreach (Row cached in Rows(vehicle, state))
        {
            if (cached.Mixed || cached.Leaves.Any(leaf => state.FairingFailed.Contains(leaf.Mount)))
                continue;
            // The cache follows the sequence generation, so the joints are read again at the point of the write.
            if (!vehicle.Parts.SequenceList.Contains(cached.Sequence)
                || Inspect(vehicle, cached.Sequence) is not { Mixed: false } row)
                continue;
            var drop = row.Leaves.Select(leaf => leaf.Shroud).ToHashSet();
            if (StageControl.Evaluate(vehicle, drop, exclude: true) != VehicleControl.Controllable)
                continue;

            state.FairingPending = row;
            state.FairingIssuedAt = now;
            // Decoupler.SetIsActive queues the decouple for the next input drain, as stock staging does.
            foreach (var leaf in row.Leaves)
                leaf.Mount.SetIsActive(vehicle, true);
            return;
        }
    }

    internal static void AwaitRelease(Vehicle vehicle, StagingState state, double now)
    {
        if (state.FairingPending is not { } row || now <= state.FairingIssuedAt)
            return;
        bool allSeparated = true;
        foreach (var leaf in row.Leaves)
        {
            bool separated = leaf.Shroud.Tree != vehicle.Parts && leaf.Mount.Connector.Connection == null;
            allSeparated &= separated;
            // A spent decoupler keeps its part in the row, and stock fires the first unactivated row that has parts.
            // A disabled one leaves the row (Part.ComputeIsSequenceable), so the empty row no longer stands in front of engine staging.
            if (separated && leaf.Mount.Parent.FullPart.Tree == vehicle.Parts)
                leaf.Mount.SetIsEnabled(false);
        }
        if (!allSeparated && now - state.FairingIssuedAt < 1.0)
            return;
        state.FairingPending = null;
        StagingHelpers.InvalidateSequenceCache();
        if (!allSeparated)
        {
            foreach (var leaf in row.Leaves)
                state.FairingFailed.Add(leaf.Mount);
            DefaultCategory.Log.Warning($"[AFC] Fairing row {row.Number} on '{vehicle.Id}' did not finish separating. Check its joints.");
            return;
        }
        if (VehicleAutopilotState.TryGet(vehicle, out var guidance))
            guidance.StageModelDirty = true;
        state.FairingLastRelease = FormattableString.Invariant($"ROW {row.Number} RELEASED {row.MassKg:F1} kg");
        DefaultCategory.Log.Info(FormattableString.Invariant(
            $"[AFC] Fairing row {row.Number} on '{vehicle.Id}' released {row.MassKg:F1} kg at t={now:F2} s. Gate at t={state.FairingIssuedAt:F2} s, q={state.FairingGateQ:F0} Pa, clearance={state.FairingGateClearance:F0} m."));
    }
}

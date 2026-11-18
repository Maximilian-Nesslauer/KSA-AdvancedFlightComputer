using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

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

    internal sealed record Row(int Number, (Decoupler Mount, Part Shroud)[] Leaves, bool Mixed)
    {
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
                // A stock shroud has no release module of its own. Only a stock radial mount
                // with a single inert shroud branch can leave the retained structure intact.
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
        return containsShroud ? new Row(sequence.Number, leaves.ToArray(), other) : null;
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
        // Stock saves can mark a row passed when a later engine row was activated.
        // The connected shroud remains a candidate until the actual joint opens.
        foreach (Sequence sequence in vehicle.Parts.SequenceList.Sequences)
            if (Inspect(vehicle, sequence) is { } row)
                rows.Add(row);
        state.FairingRowsGeneration = generation;
        state.FairingRowsPartCount = parts;
        return state.FairingRows = rows.ToArray();
    }

    internal static Sequence? NextAutomaticSequence(Vehicle vehicle, StagingState state)
    {
        Row[] fairings = Rows(vehicle, state);
        foreach (Sequence sequence in vehicle.Parts.SequenceList.Sequences)
            if (!sequence.Activated && !sequence.Parts.IsEmpty && !fairings.Any(row => !row.Mixed && row.Number == sequence.Number))
                return sequence;
        return null;
    }

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
        double altitude = r.Length() - env.TerrainRadius - vehicle.BoundingSphereRadiusBody;
        state.FairingGateQ = q;
        state.FairingGateClearance = altitude;
        double floor = BodyDefaults.DefaultOf(GuidanceSettingSpecs.FairingAltitudeKm, parent.Id);
        double ceiling = BodyDefaults.DefaultOf(GuidanceSettingSpecs.FairingQKpa, parent.Id);
        if (VehicleAutopilotState.TryGet(vehicle, out var settings))
        {
            floor = settings.FairingAltitudeKm;
            ceiling = settings.FairingQKpa;
        }
        return state.FairingGate.Sample(now, q, altitude, double3.Dot(r, v) > 0.0, air,
            Math.Clamp(floor, 0.0, 250.0) * 1000.0, Math.Clamp(ceiling, 0.0, 50.0) * 1000.0);
    }

    internal static void TryRelease(Vehicle vehicle, StagingState state, double now)
    {
        foreach (Row cached in Rows(vehicle, state))
        {
            if (cached.Mixed || cached.Leaves.Any(leaf => state.FairingFailed.Contains(leaf.Mount)))
                continue;
            // Re-read the live connections and row membership at the point of the write.
            Sequence? live = null;
            foreach (Sequence sequence in vehicle.Parts.SequenceList.Sequences)
                if (sequence.Number == cached.Number)
                    live = sequence;
            if (live == null || Inspect(vehicle, live) is not { Mixed: false } row)
                continue;
            var drop = row.Leaves.Select(leaf => leaf.Shroud).ToHashSet();
            if (StageControl.Evaluate(vehicle, drop, exclude: true) != VehicleControl.Controllable)
                continue;

            state.FairingPending = row;
            state.FairingIssuedAt = now;
            foreach (var leaf in row.Leaves)
                leaf.Mount.SetIsActive(vehicle, true);
            return;
        }
    }

    internal static bool AwaitRelease(Vehicle vehicle, StagingState state, double now)
    {
        if (state.FairingPending is not { } row)
            return false;
        if (now <= state.FairingIssuedAt)
            return true;
        bool allSeparated = true;
        foreach (var leaf in row.Leaves)
        {
            bool separated = leaf.Shroud.Tree != vehicle.Parts && leaf.Mount.Connector.Connection == null;
            allSeparated &= separated;
            // A disconnected decoupler remains sequenceable until disabled. Retire only a
            // spent mount confirmed by stock, so the now-empty row no longer blocks staging.
            if (separated && leaf.Mount.Parent.FullPart.Tree == vehicle.Parts)
                leaf.Mount.SetIsEnabled(false);
        }
        if (!allSeparated && now - state.FairingIssuedAt < 1.0)
            return true;
        state.FairingPending = null;
        StagingHelpers.InvalidateSequenceCache();
        if (!allSeparated)
        {
            foreach (var leaf in row.Leaves)
                state.FairingFailed.Add(leaf.Mount);
            DefaultCategory.Log.Warning($"[AFC] Fairing row {row.Number} on '{vehicle.Id}' did not finish separating. Check its joints.");
            return false;
        }
        if (VehicleAutopilotState.TryGet(vehicle, out var guidance))
            guidance.StageModelDirty = true;
        state.FairingLastRelease = FormattableString.Invariant($"ROW {row.Number} RELEASED {row.MassKg:F1} kg");
        DefaultCategory.Log.Info(FormattableString.Invariant(
            $"[AFC] Fairing row {row.Number} on '{vehicle.Id}' released {row.MassKg:F1} kg at t={now:F2} s. Gate at t={state.FairingIssuedAt:F2} s, q={state.FairingGateQ:F0} Pa, clearance={state.FairingGateClearance:F0} m."));
        return false;
    }
}

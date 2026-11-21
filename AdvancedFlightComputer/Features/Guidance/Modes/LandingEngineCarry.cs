using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

// A returnable stage's landing choice goes with the stage when it separates, however it separates: AFC's booster hand-over, the stock staging key, or a decoupler fired by hand.
// The stack keeps a record per returnable stage with a choice, bound to the live engine controllers (they keep their identity through a split).
// The craft that the split makes takes the record on its first step and keeps the choice as its own (LandingEngineGroups.CraftScope), with addresses rebuilt from its new root.
public static partial class GuidanceWindow
{
    private sealed class StageEngineCarry
    {
        internal required Part StageRoot;
        internal Vehicle From = null!;
        internal LandingEngineGroups Groups = LandingEngineGroups.Empty;
        internal Dictionary<string, EngineController?> Bindings = new(StringComparer.Ordinal);
        // Set when the stage has left the stack: a record is kept this long for its craft to take it.
        internal double ExpiresAt = double.PositiveInfinity;
    }

    private const double StageEngineCarryWindowS = 60.0;
    private static readonly List<StageEngineCarry> _stageEngineCarries = new();

    /// <summary>Records the landing choice of every returnable stage of the stack, so the choice can follow the stage at separation. Rides the returnable-stage refresh, which runs again on any change of choice (StageModelDirty).</summary>
    private static void RememberStageEngineChoices(Vehicle vehicle)
    {
        PartTree? tree = vehicle.Parts;
        double now = SimNow();
        _stageEngineCarries.RemoveAll(record => now > record.ExpiresAt || record.From.IsDisposed && double.IsPositiveInfinity(record.ExpiresAt));
        foreach (StageEngineCarry record in _stageEngineCarries)
        {
            // A stage that has left starts its window here, the first refresh after the split.
            if (ReferenceEquals(record.From, vehicle) && !ReferenceEquals(record.StageRoot.Tree, tree) && double.IsPositiveInfinity(record.ExpiresAt))
                record.ExpiresAt = now + StageEngineCarryWindowS;
        }
        if (tree == null)
            return;

        foreach (ReturnableStage stage in _s.ReturnableStages)
        {
            Part? root = FindPart(vehicle, stage.RootId);
            if (root == null || stage.EngineGroupScope.Length == 0)
                continue;
            StageEngineCarry? existing = _stageEngineCarries.Find(record => ReferenceEquals(record.StageRoot, root));
            LandingEngineGroups groups = _s.GetEngineGroups(stage.EngineGroupScope);
            if (groups.Chosen == null)
            {
                if (existing != null)
                    _stageEngineCarries.Remove(existing);
                continue;
            }
            existing ??= new StageEngineCarry { StageRoot = root };
            if (!_stageEngineCarries.Contains(existing))
                _stageEngineCarries.Add(existing);
            existing.From = vehicle;
            existing.Groups = groups;
            existing.Bindings = LandingEngineGroups.Controllers(vehicle, root, PartsLeavingWith(vehicle, root, stage.SequenceIndex));
            existing.ExpiresAt = double.PositiveInfinity;
        }
    }

    /// <summary>
    /// Gives a craft that has just separated the landing choice its stage had on the stack.
    /// With <paramref name="createState"/>, a craft without guidance state gets one for the choice, as a booster hand-over gives one; otherwise only a craft that already has state takes it.
    /// True when a record was taken.
    /// </summary>
    private static bool TakeStageEngineChoice(Vehicle vehicle, bool createState)
    {
        if (_stageEngineCarries.Count == 0)
            return false;
        PartTree? tree = vehicle?.Parts;
        if (vehicle == null || tree == null)
            return false;
        // While the stage is on the stack its root is in the stack's tree, so any other craft holding that root has split off with it, whichever side kept the old tree object.
        StageEngineCarry? mine = _stageEngineCarries.Find(record => ReferenceEquals(record.StageRoot.Tree, tree)
            && !ReferenceEquals(record.From, vehicle));
        if (mine == null)
            return false;
        _stageEngineCarries.Remove(mine);

        if (createState)
        {
            _s = VehicleAutopilotState.For(vehicle);
            BodySettings.Follow(_s, vehicle);
        }
        // A booster hand-over has already given the same choice from the decoupler's own split.
        if (_s.GetEngineGroups(LandingEngineGroups.CraftScope).Chosen == null)
        {
            _s.EngineGroups[LandingEngineGroups.CraftScope] = mine.Groups.Rebase(mine.Bindings, tree);
            _s.StageModelDirty = true;
            GuidanceLog.Info(vehicle, $"took landing choice '{mine.Groups.Chosen?.Name}' from the stage it separated from.");
        }
        return true;
    }
}

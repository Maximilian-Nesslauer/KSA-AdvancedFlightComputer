using System.Collections.Generic;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Object identity is the oracle when LookupCollection.Deregister moves a target index.
public sealed class TargetSelectionTest : AfcTest
{
    private const double SourceAltitudeM = 400_000.0;
    private const double FillerAltitudeM = 600_000.0;
    private const double TargetAltitudeM = 800_000.0;

    // Only Id, Parent and lookup membership matter, so the shipped Rocket is sufficient and needs no local save.
    private const string SpawnFrom = "Rocket";

    public override string Name => "afc-target-identity";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        VehicleSave? save = DefaultVehicleSaves.FindSave(SpawnFrom);
        if (save?.VehicleSaveData.RootPartInstance == null)
        {
            t.Fail("default vehicle", $"'{SpawnFrom}' is not shipped");
            return;
        }
        PartInstance design = save.VehicleSaveData.RootPartInstance;

        Vehicle? source = null;
        Vehicle? filler = null;
        Vehicle? target = null;
        try
        {
            // Spawn inside the cleanup scope. Removing the filler moves the later target into its lookup slot.
            source = Spawn(t, home, design, "TargetIdentity_Source", SourceAltitudeM);
            filler = Spawn(t, home, design, "TargetIdentity_Filler", FillerAltitudeM);
            target = Spawn(t, home, design, "TargetIdentity_Target", TargetAltitudeM);
            string targetId = target.Id;
            string? parentId = home.Id;

            // Refresh the frame list after changing registered vehicles.
            Program.RefreshVehiclesInFrame();

            List<TransferObject> list = new();
            TargetSelection.BuildList(source, list);
            t.Check("list offers the target", Contains(list, targetId));
            t.Check("list excludes the source", !Contains(list, source.Id));

            string? selectedId = targetId;
            t.Check("reconcile keeps a resolvable selection",
                TargetSelection.Reconcile(list, ref selectedId) != null && selectedId == targetId);
            t.Check("resolves to the spawned target",
                ReferenceEquals(TargetSelection.Resolve(selectedId, parentId), target));

            int indexBefore = target.LookupIndex;
            VehicleSpawner.Despawn(filler);
            filler = null;
            Program.RefreshVehiclesInFrame();
            int indexAfter = target.LookupIndex;
            t.Info($"target LookupIndex {indexBefore} -> {indexAfter} " +
                   "after an unrelated vehicle was deregistered.");

            t.Check("survives an unrelated deregistration",
                ReferenceEquals(TargetSelection.Resolve(targetId, parentId), target));

            TargetSelection.BuildList(source, list);
            selectedId = targetId;
            t.Check("reconcile still finds the target after the deregistration",
                TargetSelection.Reconcile(list, ref selectedId) != null && selectedId == targetId);

            // Reject different parents because Orbit.GetRelativeInclination does not check CCI frames.
            t.Check("refuses a target under a different parent",
                TargetSelection.Resolve(targetId, "not-the-source-parent") == null);
            t.Check("refuses an unknown id",
                TargetSelection.Resolve("TargetIdentity_NoSuchVehicle", parentId) == null);
            t.Check("refuses a null id", TargetSelection.Resolve(null, parentId) == null);

            VehicleSpawner.Despawn(target);
            target = null;
            Program.RefreshVehiclesInFrame();
            t.Check("drops a deregistered target instead of re-pointing",
                TargetSelection.Resolve(targetId, parentId) == null);

            // Clear an unavailable target so readouts and Create cannot use it.
            List<TransferObject> empty = new();
            selectedId = targetId;
            t.Check("empty list clears the selection",
                TargetSelection.Reconcile(empty, ref selectedId) == null && selectedId == null);
        }
        finally
        {
            // A constructor failure can register a partial vehicle before a local reference is assigned.
            foreach (Vehicle? spawned in new[] { source, filler, target })
            {
                if (spawned != null) VehicleSpawner.Despawn(spawned);
            }
            Program.RefreshVehiclesInFrame();
        }
    }

    private static bool Contains(List<TransferObject> list, string id)
    {
        for (int i = 0; i < list.Count; i++)
        {
            if (list[i].GetKey() == id) return true;
        }
        return false;
    }

    private static Vehicle Spawn(
        TestContext t, IParentBody home, PartInstance design, string id, double altitudeM)
        => VehicleFixtures.SpawnDesign(
            t.System, home, design, id,
            OrbitFixtures.CircularAt(home, altitudeM, Universe.GetElapsedTime()));
}

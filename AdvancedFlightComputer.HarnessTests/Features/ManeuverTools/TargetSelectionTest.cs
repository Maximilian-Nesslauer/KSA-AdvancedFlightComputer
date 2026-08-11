using System.Collections.Generic;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates that a Match Inclination target selection keeps naming the same body while vehicles
// come and go. The hazard is the game's, not the mod's arithmetic: a TransferObject stores only a
// LookupIndex, and LookupCollection.Deregister swap-removes (moving the last entry into the freed
// slot and rewriting its LookupIndex), so an index held from one frame to the next either resolves
// to a different body or, once it is past the end, makes CelestialSystem.GetIndex throw. The mod
// therefore holds an id and re-resolves it, which is what TargetSelection.Resolve does and what
// this test pins down.
//
// The oracle is object identity against the Vehicle instances the test spawned itself, never a
// re-derivation of the lookup.
public sealed class TargetSelectionTest : AfcTest
{
    private const double SourceAltitudeM = 400_000.0;
    private const double FillerAltitudeM = 600_000.0;
    private const double TargetAltitudeM = 800_000.0;

    // Any shipped vehicle will do: the selection logic reads Id, Parent and the lookup, never parts
    // or performance. "Rocket" ships in Content/Core/defaultvehicles, so this needs no
    // machine-specific save.
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
            // Spawned inside the try because Astronomical's constructor calls
            // CelestialSystem.Register before the Vehicle constructor's own validation can throw,
            // so a failed spawn leaves the earlier ones registered. HarnessRunner catches per test
            // and carries on, and HeadlessHarness's own spawn test copies the first live Vehicle in
            // the system, so a leak would reach later tests.
            //
            // Spawn order decides what the swap-remove below moves: the deregistered filler is not
            // the last entry, so LookupCollection.Deregister moves the target into the filler's slot
            // and rewrites the target's LookupIndex. That is exactly the case a held index gets
            // wrong, and the case an id has to survive.
            source = Spawn(t, home, design, "TargetIdentity_Source", SourceAltitudeM);
            filler = Spawn(t, home, design, "TargetIdentity_Filler", FillerAltitudeM);
            target = Spawn(t, home, design, "TargetIdentity_Target", TargetAltitudeM);
            string targetId = target.Id;
            string? parentId = home.Id;

            // BuildList reads Program.VehiclesInFrame, which only the frame loop and
            // Universe.DeserializeSave refresh, so refresh it here after spawning.
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

            // The regression guard: an unrelated deregistration must not re-point the selection.
            t.Check("survives an unrelated deregistration",
                ReferenceEquals(TargetSelection.Resolve(targetId, parentId), target));

            TargetSelection.BuildList(source, list);
            selectedId = targetId;
            t.Check("reconcile still finds the target after the deregistration",
                TargetSelection.Reconcile(list, ref selectedId) != null && selectedId == targetId);

            // A target in a different SOI must not be planned against: Orbit.GetRelativeInclination
            // compares orbit normals without a parent check, so the numbers would silently be taken
            // across two different CCI frames.
            t.Check("refuses a target under a different parent",
                TargetSelection.Resolve(targetId, "not-the-source-parent") == null);
            t.Check("refuses an unknown id",
                TargetSelection.Resolve("TargetIdentity_NoSuchVehicle", parentId) == null);
            t.Check("refuses a null id", TargetSelection.Resolve(null, parentId) == null);

            // Deregistering the target itself must drop the selection, not slide it onto whichever
            // body the swap-remove moved into the freed slot.
            VehicleSpawner.Despawn(target);
            target = null;
            Program.RefreshVehiclesInFrame();
            t.Check("drops a deregistered target instead of re-pointing",
                TargetSelection.Resolve(targetId, parentId) == null);

            // An empty list has to clear the stored id: a selection kept past it would leave the
            // relative inclination, the AN/DN times and an enabled Create button live for a body
            // the UI has just reported as unavailable.
            List<TransferObject> empty = new();
            selectedId = targetId;
            t.Check("empty list clears the selection",
                TargetSelection.Reconcile(empty, ref selectedId) == null && selectedId == null);
        }
        finally
        {
            // The two nulled above are already gone; the rest cover a throw part-way through.
            // A throw inside a Vehicle constructor is not recoverable here, since no local holds
            // the half-built vehicle that its base constructor already registered.
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

using Brutal.Numerics;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests.Fixtures;

// Spawning from a shipped default save or a design this suite builds itself; VehicleSpawner covers
// player saves and live-vehicle copies. Despawn from a finally, or the vehicle keeps ticking into
// later tests.
//
// Each spawn deserializes its own part tree: Vehicle.CreateVehicle(..., Part root, ...) does
// Parts = root.Tree, so reusing one design without deserializing would alias the tree across both.
//
// Each spawn seats crew in every free seat, as VehicleSpawner does: Vehicle.ComputeControl gives a crew
// pod without able crew no control, and the first step then hands Attitude and Burn back to Manual.
public static class VehicleFixtures
{
    // No staged state. A test that reads engines, sequences or fuel flow wants SpawnFromSaveData.
    public static Vehicle SpawnDesign(
        CelestialSystem system, IParentBody parent, PartInstance design, string id, Orbit orbit)
    {
        EnsurePartsLoaded(design, id);
        EnsureIdAvailable(system, id);
        PartTree tree = PartTree.Deserialize(design);
        Vehicle? vehicle = null;
        try
        {
            vehicle = Vehicle.CreateVehicle(
                system, doubleQuat.Identity, double3.Zero, parent, id, tree.Root, orbit);
            parent.Children.Add(vehicle);
            VehicleSpawner.SeatRandomCrew(vehicle);
            return vehicle;
        }
        catch
        {
            RollBackSpawn(system, id, vehicle, tree);
            throw;
        }
    }

    // Adds the staged-state restore VehicleTemplate.CreateInto performs, so engine state and
    // per-stage performance match the save.
    public static Vehicle SpawnFromSaveData(
        CelestialSystem system, IParentBody parent, VehicleSaveData data, string id, Orbit orbit)
    {
        PartInstance design = data.RootPartInstance
            ?? throw new InvalidOperationException($"vehicle '{data.Id}' has no root part instance.");
        EnsurePartsLoaded(design, data.Id);
        EnsureIdAvailable(system, id);
        PartTree tree = PartTree.Deserialize(design);
        Vehicle? vehicle = null;
        try
        {
            vehicle = Vehicle.CreateVehicle(
                system, doubleQuat.Identity, double3.Zero, parent, id, tree.Root, orbit);
            // Derived data is built lazily, so build it now as the next frame flush would. SetActiveSequence
            // needs the sequence list, and a test may refill or read solid motor stacks before any step.
            vehicle.Parts.EnsureDerived(DerivedData.All);
            vehicle.Parts.SequenceList.SetActiveSequence(data.ActiveSequence);
            vehicle.Parts.SequenceList.ApplyEnvironments(data.SequenceEnvironments);
            vehicle.Parts.FuelLinks.ApplySaveData(data.FuelLinks, design);
            parent.Children.Add(vehicle);
            VehicleSpawner.SeatRandomCrew(vehicle);
            return vehicle;
        }
        catch
        {
            RollBackSpawn(system, id, vehicle, tree);
            throw;
        }
    }

    // PartTree.Deserialize fails with a bare NullReferenceException on a part id the game no longer
    // loads, so the check runs first, as CelestialSystem.DeserializeSave does with the same call.
    //
    // A save made before a part lost connectors names connector indices the part no longer has, and
    // PartTree.Deserialize indexes past the end on them, so they are dropped first, as
    // CelestialSystem.DeserializeSave and UncompressedVehicleSave.Load do.
    private static void EnsurePartsLoaded(PartInstance design, string name)
    {
        string? missingParts = design.FindMissingParts();
        if (missingParts != null)
            throw new InvalidOperationException(
                $"vehicle '{name}' uses parts that are missing or failed to load ({missingParts}). " +
                "Save it again in the game, or build it again from the current parts.");
        int dropped = design.DropStaleConnectors();
        if (dropped > 0)
            HarnessLog.Line($"[afc] vehicle '{name}': dropped {dropped} connector(s) its parts no longer have.");
    }

    private static void EnsureIdAvailable(CelestialSystem system, string id)
    {
        if (system.All.Get(id) != null)
            throw new InvalidOperationException($"cannot spawn vehicle '{id}' because the id is already registered.");
    }

    private static void RollBackSpawn(
        CelestialSystem system, string id, Vehicle? vehicle, PartTree tree)
    {
        if (vehicle != null)
        {
            if (ReferenceEquals(system.All.Get(id), vehicle))
                VehicleSpawner.Despawn(vehicle);
            return;
        }

        Astronomical? registered = system.All.Get(id);
        if (registered != null)
            system.All.Deregister(registered);
        tree.Dispose();
    }
}

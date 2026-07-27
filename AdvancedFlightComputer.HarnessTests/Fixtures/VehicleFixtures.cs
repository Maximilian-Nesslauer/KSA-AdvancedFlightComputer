using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.HarnessTests.Fixtures;

// Spawning from a shipped default save or a design this suite builds itself; VehicleSpawner covers
// player saves and live-vehicle copies. Despawn from a finally, or the vehicle keeps ticking into
// later tests.
//
// Each spawn deserializes its own part tree: Vehicle.CreateVehicle(..., Part root, ...) does
// Parts = root.Tree, so reusing one design without deserializing would alias the tree across both.
public static class VehicleFixtures
{
    // No staged state. A test that reads engines, sequences or fuel flow wants SpawnFromSaveData.
    public static Vehicle SpawnDesign(
        CelestialSystem system, IParentBody parent, PartInstance design, string id, Orbit orbit)
    {
        PartTree tree = PartTree.Deserialize(design);
        Vehicle vehicle = Vehicle.CreateVehicle(
            system, doubleQuat.Identity, double3.Zero, parent, id, tree.Root, orbit);
        parent.Children.Add(vehicle);
        return vehicle;
    }

    // Adds the staged-state restore VehicleTemplate.CreateInto performs, so engine state and
    // per-stage performance match the save.
    public static Vehicle SpawnFromSaveData(
        CelestialSystem system, IParentBody parent, VehicleSaveData data, string id, Orbit orbit)
    {
        PartInstance design = data.RootPartInstance
            ?? throw new InvalidOperationException($"vehicle '{data.Id}' has no root part instance.");
        PartTree tree = PartTree.Deserialize(design);
        Vehicle vehicle = Vehicle.CreateVehicle(
            system, doubleQuat.Identity, double3.Zero, parent, id, tree.Root, orbit);
        vehicle.Parts.SequenceList.SetActiveSequence(data.ActiveSequence);
        vehicle.Parts.SequenceList.ApplyEnvironments(data.SequenceEnvironments);
        vehicle.Parts.FuelLinks.ApplySaveData(data.FuelLinks, design);
        parent.Children.Add(vehicle);
        return vehicle;
    }
}

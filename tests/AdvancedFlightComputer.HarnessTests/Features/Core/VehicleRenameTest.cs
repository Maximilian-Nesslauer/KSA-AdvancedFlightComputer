using System.Linq;
using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Use CelestialSystem.Rename so the system index follows the vehicle's new ID.
public sealed class VehicleRenameTest : AfcTest
{
    public override string Name => "afc-vehicle-rename";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        VehicleSave? save = DefaultVehicleSaves.FindSave("Rocket");
        if (save?.VehicleSaveData.RootPartInstance == null)
        {
            t.Fail("default vehicle", "Rocket is not available");
            return;
        }

        Harmony harmony = new("com.maxi.afc.harnesstests.rename");
        string firstId = "Rename_" + Guid.NewGuid().ToString("N");
        string secondId = "Rename_" + Guid.NewGuid().ToString("N");
        string renamedId = "Rename_" + Guid.NewGuid().ToString("N");
        Vehicle? vehicle = null;
        Vehicle? other = null;
        try
        {
            SharedVehicleHooks.ApplyPatches(harmony);
            vehicle = VehicleFixtures.SpawnDesign(t.System, home,
                save.VehicleSaveData.RootPartInstance, firstId,
                OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime()));
            other = VehicleFixtures.SpawnDesign(t.System, home,
                save.VehicleSaveData.RootPartInstance, secondId,
                OrbitFixtures.CircularAt(home, 600_000, Universe.GetElapsedTime()));

            CheckBinding(t, harmony.Id);
            CheckRenameMovesState(t, vehicle, renamedId);
            CheckClaimIsReleasableAgain(t, vehicle);
            CheckRefusedRenameMovesNothing(t, vehicle, secondId, renamedId);
        }
        finally
        {
            // Release only this fixture's claims, because the claim table is shared.
            if (vehicle != null)
                VehicleControlOwnership.ReleaseAll(vehicle);
            if (other != null)
                VehicleControlOwnership.ReleaseAll(other);
            if (vehicle != null)
                VehicleSpawner.Despawn(vehicle);
            if (other != null)
                VehicleSpawner.Despawn(other);
            foreach (string id in new[] { firstId, secondId, renamedId })
            {
                MultiPassRegistry.Remove(id);
                RcsExecRegistry.Remove(id);
                PassCompletionPatch.OnRegistryRemovedExternally(id);
            }
            harmony.UnpatchAll(harmony.Id);
        }
    }

    private static void CheckBinding(TestContext t, string owner)
    {
        Patches? patches = Harmony.GetPatchInfo(GameReflection.Vehicle_SetName!);
        t.Check("one prefix and one postfix on SetName", patches != null
            && patches.Prefixes.Count(p => p.owner == owner) == 1
            && patches.Postfixes.Count(p => p.owner == owner) == 1);
    }

    private static void CheckRenameMovesState(TestContext t, Vehicle vehicle, string renamedId)
    {
        string oldId = vehicle.Id;
        RcsExecution rcs = Arm(vehicle);
        MultiPassExecution pass = ArmPass(vehicle);
        RecordBurnMode(oldId, FlightComputerBurnMode.Auto);
        VehicleControlOwnership.TryClaim(vehicle, ControlClaimant.RcsTranslation, out _);
        if (!t.Check("the craft is armed and claimed under its first id",
                RcsExecRegistry.TryGet(oldId, out _) && MultiPassRegistry.Has(oldId)
                && VehicleControlOwnership.ClaimedId(vehicle) == oldId))
            return;

        if (!t.Check("the system accepts the new name", t.System.Rename(vehicle, renamedId)))
            return;
        t.Check("the vehicle carries the new id", vehicle.Id == renamedId);
        t.Check("the system index follows the rename",
            t.System.All.TryGet(renamedId, out Astronomical? found) && ReferenceEquals(found, vehicle)
            && !t.System.All.TryGet(oldId, out _));

        t.Check("the RCS execution follows the rename",
            RcsExecRegistry.TryGet(renamedId, out RcsExecution? movedRcs)
            && ReferenceEquals(movedRcs, rcs) && !RcsExecRegistry.TryGet(oldId, out _));
        t.Check("the RCS execution stores the new id, so a save writes it",
            rcs.VehicleId == renamedId);

        t.Check("the pass sequence follows the rename",
            MultiPassRegistry.TryGet(renamedId, out MultiPassExecution? movedPass)
            && ReferenceEquals(movedPass, pass) && !MultiPassRegistry.Has(oldId));
        t.Check("the pass sequence stores the new id, so a save writes it",
            pass.VehicleId == renamedId);

        // Completion detection needs the previous mode under the new ID.
        t.Check("the recorded burn mode follows the rename",
            RecordedBurnMode(renamedId) == FlightComputerBurnMode.Auto
            && RecordedBurnMode(oldId) == null);
    }

    // An idle claim is releasable only when its recorded ID matches the vehicle's ID.
    private static void CheckClaimIsReleasableAgain(TestContext t, Vehicle vehicle)
    {
        if (!t.Check("the claim survives the rename",
                VehicleControlOwnership.Holds(vehicle, ControlClaimant.RcsTranslation)
                && VehicleControlOwnership.ClaimedId(vehicle) == vehicle.Id))
            return;

        if (RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec))
            exec.ClearActive();
        RcsExecutor.ReconcileClaim(vehicle);
        t.Check("an idle execution gives the craft back after the rename",
            VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.None);
    }

    private static void CheckRefusedRenameMovesNothing(
        TestContext t, Vehicle vehicle, string takenId, string currentId)
    {
        t.Check("the system refuses a name another vehicle holds",
            !t.System.Rename(vehicle, takenId));

        // Rename refuses before it calls SetName, so this covers the postfix's own result guard.
        t.Check("SetName reports a taken name as refused", !vehicle.SetName(takenId));

        t.Check("a refused rename keeps the id and its state",
            vehicle.Id == currentId
            && RcsExecRegistry.TryGet(currentId, out _) && MultiPassRegistry.Has(currentId)
            && !RcsExecRegistry.TryGet(takenId, out _) && !MultiPassRegistry.Has(takenId));
    }

    private static RcsExecution Arm(Vehicle vehicle)
    {
        RcsExecution exec = RcsExecRegistry.GetOrCreate(vehicle.Id);
        exec.ActiveBurnTimeSec = 100.0;
        exec.ActiveBurnDvMs = 20.0;
        exec.ForcedRcsOn = true;
        exec.AlignCommanded = true;
        exec.ControlTaken = true;
        return exec;
    }

    private static MultiPassExecution ArmPass(Vehicle vehicle)
    {
        MultiPassExecution exec = new()
        {
            SaveId = SaveLoadObserver.CurrentSaveId,
            VehicleId = vehicle.Id,
            Mode = SplitMode.EqualBurnTime,
            PassCountTotal = 2,
            Intent = new ApseIntent
            {
                IsSetApoapsis = true,
                TargetRadiusMeters = 7_000_000,
                ParentId = vehicle.Parent.Id,
            },
        };
        MultiPassRegistry.Add(exec);
        return exec;
    }

    // Seed history directly to avoid advancing the synthetic pass sequence.
    private static void RecordBurnMode(string vehicleId, FlightComputerBurnMode mode)
        => BurnModeHistory()[vehicleId] = mode;

    private static FlightComputerBurnMode? RecordedBurnMode(string vehicleId)
        => BurnModeHistory().TryGetValue(vehicleId, out FlightComputerBurnMode mode) ? mode : null;

    private static Dictionary<string, FlightComputerBurnMode> BurnModeHistory()
        => (Dictionary<string, FlightComputerBurnMode>)typeof(PassCompletionPatch)
            .GetField("_lastBurnMode", BindingFlags.NonPublic | BindingFlags.Static)!
            .GetValue(null)!;
}

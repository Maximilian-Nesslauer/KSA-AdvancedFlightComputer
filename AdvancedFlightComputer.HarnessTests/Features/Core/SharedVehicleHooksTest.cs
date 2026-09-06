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

public sealed class SharedVehicleHooksTest : AfcTest
{
    private static readonly List<string> Calls = new();

    public override string Name => "afc-shared-vehicle-hooks";

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

        Harmony harmony = new("com.maxi.afc.harnesstests.shared-hooks");
        bool oldMultiPass = SharedVehicleHooks.MultiPassEnabled;
        bool oldRcs = SharedVehicleHooks.RcsEnabled;
        string vehicleId = "SharedHooks_" + Guid.NewGuid().ToString("N");
        Vehicle? vehicle = null;
        try
        {
            SharedVehicleHooks.ApplyPatches(harmony);
            PatchRecorder(harmony, typeof(PassCompletionPatch), nameof(RecordMultiPass));
            PatchRecorder(harmony, typeof(RcsDriverPatch), nameof(RecordRcs));
            vehicle = VehicleFixtures.SpawnDesign(t.System, home,
                save.VehicleSaveData.RootPartInstance, vehicleId,
                OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime()));

            CheckTickOrder(t, vehicle);
            CheckDisposal(t, vehicle);
            CheckBinding(t, harmony.Id, GameReflection.Universe_ApplyVehicleSolvers!);
            CheckBinding(t, harmony.Id, GameReflection.Vehicle_Dispose!);
            SharedVehicleHooks.Reset();
            t.Check("reset disables both drivers", !SharedVehicleHooks.MultiPassEnabled && !SharedVehicleHooks.RcsEnabled);
        }
        finally
        {
            if (vehicle != null)
                VehicleSpawner.Despawn(vehicle);
            MultiPassRegistry.Remove(vehicleId);
            RcsExecRegistry.Remove(vehicleId);
            PassCompletionPatch.OnRegistryRemovedExternally(vehicleId);
            harmony.UnpatchAll(harmony.Id);
            SharedVehicleHooks.MultiPassEnabled = oldMultiPass;
            SharedVehicleHooks.RcsEnabled = oldRcs;
            Calls.Clear();
        }
    }

    private static void CheckTickOrder(TestContext t, Vehicle vehicle)
    {
        SharedVehicleHooks.MultiPassEnabled = true;
        SharedVehicleHooks.RcsEnabled = true;
        Calls.Clear();
        SharedVehicleHooks.TickVehicles([vehicle]);
        t.Check("MultiPass runs before RCS exactly once", Calls.SequenceEqual(new[] { "MultiPass", "RCS" }));

        SharedVehicleHooks.MultiPassEnabled = false;
        Calls.Clear();
        SharedVehicleHooks.TickVehicles([vehicle]);
        t.Check("failed MultiPass block cannot tick", Calls.SequenceEqual(new[] { "RCS" }));

        SharedVehicleHooks.MultiPassEnabled = true;
        SharedVehicleHooks.RcsEnabled = false;
        Calls.Clear();
        SharedVehicleHooks.TickVehicles([vehicle]);
        t.Check("failed RCS block cannot tick", Calls.SequenceEqual(new[] { "MultiPass" }));

        SharedVehicleHooks.Reset();
        Calls.Clear();
        SharedVehicleHooks.TickVehicles([vehicle]);
        t.Check("disabled blocks cannot tick", Calls.Count == 0);
    }

    private static void CheckDisposal(TestContext t, Vehicle vehicle)
    {
        SharedVehicleHooks.MultiPassEnabled = true;
        SharedVehicleHooks.RcsEnabled = true;
        MultiPassRegistry.Add(new MultiPassExecution
        {
            SaveId = SaveLoadObserver.CurrentSaveId,
            VehicleId = vehicle.Id,
            Mode = SplitMode.EqualBurnTime,
            PassCountTotal = 2,
            Intent = new ApseIntent { IsSetApoapsis = true, TargetRadiusMeters = 7_000_000, ParentId = vehicle.Parent.Id },
        });
        RcsExecRegistry.GetOrCreate(vehicle.Id);
        t.Check("both registry entries exist before disposal", MultiPassRegistry.Has(vehicle.Id)
            && RcsExecRegistry.TryGet(vehicle.Id, out _));

        SharedVehicleHooks.Reset();
        vehicle.Dispose(false);
        t.Check("disposal removes entries even when both drivers are disabled", !MultiPassRegistry.Has(vehicle.Id)
            && !RcsExecRegistry.TryGet(vehicle.Id, out _));
        SharedVehicleHooks.MultiPassEnabled = true;
        SharedVehicleHooks.RcsEnabled = true;
        Calls.Clear();
        SharedVehicleHooks.TickVehicles([vehicle]);
        t.Check("disposed vehicles do not tick", Calls.Count == 0);
    }

    private static void CheckBinding(TestContext t, string owner, MethodBase target)
    {
        Patches? patches = Harmony.GetPatchInfo(target);
        t.Check($"one shared postfix on {target.Name}", patches != null
            && patches.Postfixes.Count(p => p.owner == owner) == 1
            && !patches.Prefixes.Any(p => p.owner == owner));
    }

    private static void PatchRecorder(Harmony harmony, Type driver, string recorder)
        => harmony.Patch(AccessTools.Method(driver, "TickVehicle", [typeof(Vehicle)]),
            prefix: new HarmonyMethod(typeof(SharedVehicleHooksTest), recorder));

    private static bool RecordMultiPass()
    {
        Calls.Add("MultiPass");
        return false;
    }

    private static bool RecordRcs()
    {
        Calls.Add("RCS");
        return false;
    }
}

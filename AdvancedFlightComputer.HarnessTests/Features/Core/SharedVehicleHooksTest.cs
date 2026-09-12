using System.IO;
using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class SharedVehicleHooksTest : AfcTest
{
    private static readonly List<string> Calls = new();

    // Step past the burn time without waiting long enough for the plan to remove the expired burn.
    private const double StallBurnLeadSec = 3.0;
    private const int StallStepCount = 32;

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
        bool oldAutoStage = SharedVehicleHooks.AutoStageEnabled;
        string vehicleId = "SharedHooks_" + Guid.NewGuid().ToString("N");
        Vehicle? vehicle = null;
        try
        {
            SharedVehicleHooks.ApplyPatches(harmony);
            SharedVehicleHooks.Reset();
            vehicle = VehicleFixtures.SpawnDesign(t.System, home,
                save.VehicleSaveData.RootPartInstance, vehicleId,
                OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime()));

            CheckRcsCompletionHandover(t, vehicle);
            CheckRcsStallHint(t, vehicle);
            PatchRecorder(harmony, typeof(StagingDetector), nameof(StagingDetector.Evaluate), [], nameof(RecordAutoStage));
            PatchRecorder(harmony, typeof(PassCompletionPatch), "TickVehicle", [typeof(Vehicle)], nameof(RecordMultiPass));
            PatchRecorder(harmony, typeof(RcsDriverPatch), "TickVehicle", [typeof(Vehicle)], nameof(RecordRcs));
            CheckTickOrder(t, vehicle);
            CheckDisposal(t, vehicle);
            CheckBinding(t, harmony.Id, GameReflection.Universe_ApplyVehicleSolvers!, expectPrefix: true);
            CheckBinding(t, harmony.Id, GameReflection.Vehicle_Dispose!, expectPrefix: false);
            SharedVehicleHooks.Reset();
            t.Check("reset disables every driver", !SharedVehicleHooks.MultiPassEnabled
                && !SharedVehicleHooks.RcsEnabled && !SharedVehicleHooks.AutoStageEnabled);
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
            SharedVehicleHooks.AutoStageEnabled = oldAutoStage;
            Calls.Clear();
        }
    }

    private static void CheckTickOrder(TestContext t, Vehicle vehicle)
    {
        SharedVehicleHooks.AutoStageEnabled = true;
        SharedVehicleHooks.MultiPassEnabled = true;
        SharedVehicleHooks.RcsEnabled = true;
        Calls.Clear();
        SharedVehicleHooks.TickVehicles([vehicle]);
        t.Check("AutoStage, MultiPass, then RCS, each exactly once",
            Calls.SequenceEqual(new[] { "AutoStage", "MultiPass", "RCS" }));

        SharedVehicleHooks.MultiPassEnabled = false;
        Calls.Clear();
        SharedVehicleHooks.TickVehicles([vehicle]);
        t.Check("failed MultiPass block cannot tick", Calls.SequenceEqual(new[] { "AutoStage", "RCS" }));

        SharedVehicleHooks.MultiPassEnabled = true;
        SharedVehicleHooks.RcsEnabled = false;
        Calls.Clear();
        SharedVehicleHooks.TickVehicles([vehicle]);
        t.Check("failed RCS block cannot tick", Calls.SequenceEqual(new[] { "AutoStage", "MultiPass" }));

        SharedVehicleHooks.RcsEnabled = true;
        SharedVehicleHooks.AutoStageEnabled = false;
        Calls.Clear();
        SharedVehicleHooks.TickVehicles([vehicle]);
        t.Check("failed AutoStage block cannot tick", Calls.SequenceEqual(new[] { "MultiPass", "RCS" }));

        SharedVehicleHooks.Reset();
        Calls.Clear();
        SharedVehicleHooks.TickVehicles([vehicle]);
        t.Check("disabled blocks cannot tick", Calls.Count == 0);
    }

    private static void CheckDisposal(TestContext t, Vehicle vehicle)
    {
        SharedVehicleHooks.MultiPassEnabled = true;
        SharedVehicleHooks.RcsEnabled = true;
        // The state cache holds the vehicle and its part graph.
        MultiPassPreviewCache.GetSequenceState(vehicle);
        FieldInfo cachedState = StaticField(typeof(MultiPassPreviewCache), "_cachedState");
        t.Check("sequence-state cache holds the vehicle before disposal", cachedState.GetValue(null) != null);
        StagingHelpers.HasNextEngineSequence(vehicle);
        FieldInfo stagingCache = StaticField(typeof(StagingHelpers), "_cachedVehicle");
        t.Check("staging sequence cache holds the vehicle before disposal", ReferenceEquals(stagingCache.GetValue(null), vehicle));
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
        t.Check("disposal drops the sequence-state cache of that vehicle", cachedState.GetValue(null) == null);
        t.Check("disposal drops the staging sequence cache of that vehicle", stagingCache.GetValue(null) == null);
        SharedVehicleHooks.AutoStageEnabled = true;
        SharedVehicleHooks.MultiPassEnabled = true;
        SharedVehicleHooks.RcsEnabled = true;
        Calls.Clear();
        SharedVehicleHooks.TickVehicles([vehicle]);
        // The staging detector ticks per frame, not per vehicle, so it is the one call expected here.
        t.Check("disposed vehicles do not tick", Calls.SequenceEqual(new[] { "AutoStage" }));
    }

    private static void CheckRcsCompletionHandover(TestContext t, Vehicle vehicle)
    {
        SimDriver driver = t.Session.CreateDriver();
        driver.Step(0.05, 2);
        RcsFlightSupport.BurnSetup? setup = RcsFlightSupport.AddBurn(
            vehicle, driver, double3.UnitX, 0.5, 60.0);
        if (setup == null)
        {
            t.Fail("RCS MultiPass handover setup", "could not add a burn");
            RcsFlightSupport.CleanupBurns(vehicle.FlightComputer);
            return;
        }

        Burn completedBurn = setup.Burn;
        var exec = new MultiPassExecution
        {
            SaveId = SaveLoadObserver.CurrentSaveId,
            VehicleId = vehicle.Id,
            Mode = SplitMode.EqualDv,
            PassCountTotal = 2,
            Intent = new NextBurnIntent(),
        };
        exec.AssignCurrentBurn(completedBurn);
        exec.AwaitingMaterialization = false;
        MultiPassRegistry.Add(exec);
        RcsBurnCompletions.Completed += PassCompletionPatch.OnRcsBurnCompleted;

        try
        {
            RcsBurnCompletions.Raise(vehicle, completedBurn);
            if (!t.Check("RCS completion advances MultiPass",
                    exec.PassIndex == 1 && exec.CurrentBurn == null && exec.ReengageAutoOnNextBurn
                    && !vehicle.FlightComputer.BurnPlan.TryGetBurn(completedBurn)))
                return;

            PassCompletionPatch.TickVehicle(vehicle);
            if (!t.Check("RCS completion creates next pass",
                    exec.CurrentBurn != null && exec.AwaitingMaterialization))
                return;
            driver.Step(0.05);
            PassCompletionPatch.TickVehicle(vehicle);
            if (!t.Check("next pass re-engages execution",
                    !exec.AwaitingMaterialization && !exec.ReengageAutoOnNextBurn
                    && vehicle.FlightComputer.BurnMode == FlightComputerBurnMode.Auto))
                return;

            Burn removedBySubscriber = exec.CurrentBurn!;
            vehicle.FlightComputer.RemoveBurn(removedBySubscriber);
            RcsBurnCompletions.Raise(vehicle, removedBySubscriber);
            t.Check("removed RCS burn completes final pass",
                exec.PassIndex == 2 && exec.CurrentBurn == null);
            PassCompletionPatch.TickVehicle(vehicle);
            t.Check("RCS completion ends MultiPass", !MultiPassRegistry.Has(vehicle.Id));
        }
        finally
        {
            RcsBurnCompletions.Completed -= PassCompletionPatch.OnRcsBurnCompleted;
            MultiPassRegistry.Remove(vehicle.Id);
            PassCompletionPatch.OnRegistryRemovedExternally(vehicle.Id);
            RcsFlightSupport.CleanupBurns(vehicle.FlightComputer);
        }
    }

    // RcsExecutor.Cancel clears the active execution but leaves the burn in the plan.
    private static void CheckRcsStallHint(TestContext t, Vehicle vehicle)
    {
        SimDriver driver = t.Session.CreateDriver();
        driver.Step(0.05, 2);
        RcsFlightSupport.BurnSetup? setup = RcsFlightSupport.AddBurn(
            vehicle, driver, double3.UnitX, 0.5, StallBurnLeadSec);
        if (setup == null)
        {
            t.Fail("RCS stall hint setup", "could not add a burn");
            RcsFlightSupport.CleanupBurns(vehicle.FlightComputer);
            return;
        }

        var exec = new MultiPassExecution
        {
            SaveId = SaveLoadObserver.CurrentSaveId,
            VehicleId = vehicle.Id,
            Mode = SplitMode.EqualDv,
            PassCountTotal = 2,
            Intent = new NextBurnIntent(),
        };
        exec.AssignCurrentBurn(setup.Burn);
        exec.AwaitingMaterialization = false;
        MultiPassRegistry.Add(exec);

        RcsExecution rcs = RcsExecRegistry.GetOrCreate(vehicle.Id);
        rcs.ActiveBurnTimeSec = setup.Burn.Time.Seconds();
        rcs.ActiveBurnDvMs = setup.Burn.DeltaVVlf.Length();

        try
        {
            // Keep automatic ticks off while the simulation steps.
            SharedVehicleHooks.RcsEnabled = true;
            PassCompletionPatch.TickVehicle(vehicle);
            SharedVehicleHooks.RcsEnabled = false;
            if (!t.Check("an RCS execution arms the pass",
                    exec.PassArmed && !exec.StallHintShown
                    && vehicle.FlightComputer.BurnMode == FlightComputerBurnMode.Manual))
                return;

            rcs.ClearActive();
            driver.Step(0.1, StallStepCount);
            PassCompletionPatch.TickVehicle(vehicle);
            t.Check("a cancelled RCS pass reports the stall", exec.StallHintShown);
            t.Check("the stalled execution is preserved", MultiPassRegistry.Has(vehicle.Id)
                && exec.CurrentBurn != null && exec.PassIndex == 0);
        }
        finally
        {
            SharedVehicleHooks.RcsEnabled = false;
            MultiPassRegistry.Remove(vehicle.Id);
            PassCompletionPatch.OnRegistryRemovedExternally(vehicle.Id);
            RcsExecRegistry.Remove(vehicle.Id);
            RcsFlightSupport.CleanupBurns(vehicle.FlightComputer);
        }
    }

    private static FieldInfo StaticField(Type type, string name)
        => type.GetField(name, BindingFlags.Static | BindingFlags.NonPublic)
           ?? throw new MissingFieldException(type.FullName, name);

    private static void CheckBinding(TestContext t, string owner, MethodBase target, bool expectPrefix)
    {
        Patches? patches = Harmony.GetPatchInfo(target);
        int prefixes = patches?.Prefixes.Count(p => p.owner == owner) ?? 0;
        t.Check($"one shared postfix and {(expectPrefix ? "one prefix" : "no prefix")} on {target.Name}",
            patches != null && patches.Postfixes.Count(p => p.owner == owner) == 1
            && prefixes == (expectPrefix ? 1 : 0));
    }

    private static void PatchRecorder(Harmony harmony, Type driver, string method, Type[] parameters, string recorder)
        => harmony.Patch(AccessTools.Method(driver, method, parameters),
            prefix: new HarmonyMethod(typeof(SharedVehicleHooksTest), recorder));

    private static bool RecordAutoStage()
    {
        Calls.Add("AutoStage");
        return false;
    }

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

    private sealed class NextBurnIntent : IManeuverIntent
    {
        public string Kind => "test-next-burn";
        public string TypeKey => "test-next-burn";

        public OrbitManeuvers.ManeuverResult? ComputeManeuver(Vehicle vehicle) => null;

        public bool IsSatisfied(Vehicle vehicle) => false;

        public PassPlanResult RecomputePass(
            Vehicle vehicle, int passIndex, int passCountTotal, SplitMode mode)
        {
            UniverseTime burnTime = Universe.GetElapsedTime() + 60.0;
            if (vehicle.FlightPlan.TryFindPatch(burnTime) == null)
                return PassPlanResult.Failure("no flight plan patch");
            return PassPlanResult.Success(new PassPreview(
                burnTime, new double3(0.5, 0.0, 0.0), 0.0, vehicle.FlightPlan));
        }

        public void WriteToToml(TextWriter writer)
        {
        }
    }
}

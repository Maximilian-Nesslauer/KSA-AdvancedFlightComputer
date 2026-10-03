using System.IO;
using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoRemove;
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
    private static int _autoRemoveCalls;
    private static bool _autoRemoveFoundBurn;

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
        bool oldAutoRemove = SharedVehicleHooks.AutoRemoveEnabled;
        string vehicleId = "SharedHooks_" + Guid.NewGuid().ToString("N");
        Vehicle? vehicle = null;
        try
        {
            SharedVehicleHooks.ApplyPatches(harmony);
            SharedVehicleHooks.Reset();
            vehicle = VehicleFixtures.SpawnDesign(t.System, home,
                save.VehicleSaveData.RootPartInstance, vehicleId,
                OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime()));

            CheckRcsPassHandover(t, vehicle);
            CheckRcsStallHint(t, vehicle);
            CheckStallHintWaitsForStockEnd(t, vehicle);
            CheckStockCompletionHandover(t, vehicle);
            CheckStockPropellantStop(t, vehicle);
            // Empties the vehicle's tanks, so it runs after every check that burns.
            CheckDryPropellantStop(t, vehicle);
            PatchRecorder(harmony, typeof(StagingDetector), nameof(StagingDetector.Evaluate), [], nameof(RecordAutoStage));
            PatchRecorder(harmony, typeof(PassCompletionPatch), "TickVehicle", [typeof(Vehicle)], nameof(RecordMultiPass));
            PatchRecorder(harmony, typeof(RcsDriverPatch), "TickVehicle", [typeof(Vehicle)], nameof(RecordRcs));
            PatchRecorder(harmony, typeof(PassCompletionPatch), nameof(PassCompletionPatch.OnStockBurnEnded),
                [typeof(Vehicle), typeof(Burn), typeof(bool), typeof(bool)], nameof(RecordMultiPass));
            PatchRecorder(harmony, typeof(FinishedBurnRemover), nameof(FinishedBurnRemover.OnBurnCompleted),
                [typeof(Vehicle), typeof(Burn)], nameof(RecordAutoRemove));
            CheckTickOrder(t, vehicle);
            CheckStockBurnEndOrder(t, vehicle);
            CheckDisposal(t, vehicle);
            CheckBinding(t, harmony.Id, GameReflection.Universe_ApplyVehicleSolvers!, expectPrefix: true);
            CheckBinding(t, harmony.Id, GameReflection.FlightComputer_RaisePendingAlerts!, expectPrefix: true);
            CheckBinding(t, harmony.Id, GameReflection.Vehicle_Dispose!, expectPrefix: false);
            SharedVehicleHooks.Reset();
            t.Check("reset disables every driver", !SharedVehicleHooks.MultiPassEnabled
                && !SharedVehicleHooks.RcsEnabled && !SharedVehicleHooks.AutoStageEnabled
                && !SharedVehicleHooks.AutoRemoveEnabled);
        }
        finally
        {
            if (vehicle != null)
                VehicleSpawner.Despawn(vehicle);
            MultiPassRegistry.Remove(vehicleId);
            RcsExecRegistry.Remove(vehicleId);
            harmony.UnpatchAll(harmony.Id);
            SharedVehicleHooks.MultiPassEnabled = oldMultiPass;
            SharedVehicleHooks.RcsEnabled = oldRcs;
            SharedVehicleHooks.AutoStageEnabled = oldAutoStage;
            SharedVehicleHooks.AutoRemoveEnabled = oldAutoRemove;
            Calls.Clear();
        }
    }

    private static void CheckTickOrder(TestContext t, Vehicle vehicle)
    {
        SharedVehicleHooks.AutoStageEnabled = true;
        SharedVehicleHooks.MultiPassEnabled = true;
        SharedVehicleHooks.RcsEnabled = true;
        SharedVehicleHooks.AutoRemoveEnabled = true;
        Calls.Clear();
        SharedVehicleHooks.TickVehicles([vehicle]);
        t.Check("AutoStage, MultiPass, then RCS, each exactly once, and AutoRemove has no tick",
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

    // The end of a stock Auto burn reaches MultiPass before AutoRemove, and a propellant stop never reaches AutoRemove.
    private static void CheckStockBurnEndOrder(TestContext t, Vehicle vehicle)
    {
        UniverseTime now = Universe.GetElapsedTime();
        PatchedConic patch = new PatchedConic(now, UniverseTime.EndOfTime, PatchTransition.Burn,
            PatchTransition.Final, Orbit.CreateFrom(vehicle.Orbit), vehicle.ParentPatchIdHash);
        Burn burn = Burn.Create(OrbitPointCce.Zero, (now + 3600.0).Seconds(), new double3(1.0, 0.0, 0.0), patch, vehicle);
        try
        {
            SharedVehicleHooks.MultiPassEnabled = true;
            SharedVehicleHooks.AutoRemoveEnabled = true;
            Calls.Clear();
            SharedVehicleHooks.OnStockBurnEnded(vehicle, new StockBurnEnd(burn, completed: true));
            t.Check("a completed stock burn reaches MultiPass, then AutoRemove",
                Calls.SequenceEqual(new[] { "MultiPass", "AutoRemove" }));

            Calls.Clear();
            SharedVehicleHooks.OnStockBurnEnded(vehicle, new StockBurnEnd(burn, completed: false));
            t.Check("a propellant stop reaches MultiPass only", Calls.SequenceEqual(new[] { "MultiPass" }));

            SharedVehicleHooks.MultiPassEnabled = false;
            Calls.Clear();
            SharedVehicleHooks.OnStockBurnEnded(vehicle, new StockBurnEnd(burn, completed: true));
            t.Check("failed MultiPass block does not see the stock burn end", Calls.SequenceEqual(new[] { "AutoRemove" }));

            SharedVehicleHooks.Reset();
            Calls.Clear();
            SharedVehicleHooks.OnStockBurnEnded(vehicle, new StockBurnEnd(burn, completed: true));
            t.Check("disabled blocks do not see the stock burn end", Calls.Count == 0);
        }
        finally
        {
            SharedVehicleHooks.Reset();
            Calls.Clear();
            burn.Dispose();
        }
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
        t.Check("staging state exists for the vehicle before disposal", StagingDetector.HasState(vehicle));
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
        t.Check("disposal drops the staging state of that vehicle", !StagingDetector.HasState(vehicle));
        SharedVehicleHooks.AutoStageEnabled = true;
        SharedVehicleHooks.MultiPassEnabled = true;
        SharedVehicleHooks.RcsEnabled = true;
        SharedVehicleHooks.AutoRemoveEnabled = true;
        Calls.Clear();
        SharedVehicleHooks.TickVehicles([vehicle]);
        // The staging detector ticks per frame, not per vehicle, so that call is expected.
        t.Check("disposed vehicles do not tick", Calls.SequenceEqual(new[] { "AutoStage" }));
    }

    // An RCS pass ends through the stock completion flag and FlightComputer.RaisePendingAlerts, the
    // same dispatch an engine pass takes. With a player burn planned after the first pass stock removes
    // that pass itself and MultiPass only advances. The final pass is the last burn, so stock keeps it
    // and MultiPass removes it, with AutoRemove on as well. A probe on AutoRemove shows that the pass is
    // already gone when AutoRemove runs, so the removal is MultiPass's own.
    private static void CheckRcsPassHandover(TestContext t, Vehicle vehicle)
    {
        SimDriver driver = t.Session.CreateDriver();
        driver.Step(0.05, 2);
        FlightComputer fc = vehicle.FlightComputer;
        Burn pass = AddPlanBurn(vehicle, 60.0);
        Burn playerBurn = AddPlanBurn(vehicle, 7200.0);
        var exec = new MultiPassExecution
        {
            SaveId = SaveLoadObserver.CurrentSaveId,
            VehicleId = vehicle.Id,
            Mode = SplitMode.EqualDv,
            PassCountTotal = 2,
            Intent = new NextBurnIntent(),
        };
        exec.AssignCurrentBurn(pass);
        exec.AwaitingMaterialization = false;
        MultiPassRegistry.Add(exec);
        Harmony probe = new("com.maxi.afc.harnesstests.sharedhooks.rcs-pass");

        try
        {
            fc.BurnMode = FlightComputerBurnMode.Manual;
            RcsExecution rcs = RcsFlightSupport.SeedFlyingExecution(vehicle, pass);
            RcsExecutor.Complete(vehicle, fc, rcs, 0f);
            if (!t.Check("an RCS pass raises the stock completion flag",
                    fc.AutoBurnCompleted && !fc.AutoBurnStoppedOutOfPropellant && !rcs.IsActive))
                return;
            RaisePendingAlertsWith(vehicle, autoRemove: false);
            if (!t.Check("an RCS pass advances MultiPass through the stock end, removed once by stock",
                    !fc.AutoBurnCompleted
                    && exec.PassIndex == 1 && exec.CurrentBurn == null && exec.ReengageAutoOnNextBurn
                    && !fc.BurnPlan.TryGetBurn(pass) && fc.BurnPlan.TryGetBurn(playerBurn)
                    && fc.BurnPlan.BurnCount == 1,
                    $"passIndex={exec.PassIndex} burns={fc.BurnPlan.BurnCount}"))
                return;

            PassCompletionPatch.TickVehicle(vehicle);
            driver.Step(0.05);
            PassCompletionPatch.TickVehicle(vehicle);
            Burn? next = exec.CurrentBurn;
            if (!t.Check("the next pass is planned ahead of the player burn",
                    next != null && !exec.AwaitingMaterialization
                    && ReferenceEquals(fc.BurnPlan.FindFirstExecutableBurn(), next)
                    && fc.BurnPlan.TryGetBurn(playerBurn) && fc.BurnPlan.BurnCount == 2,
                    $"burns={fc.BurnPlan.BurnCount}"))
                return;

            // The RCS executor flies in Manual, so the Auto the re-engage armed is not part of this end.
            fc.BurnMode = FlightComputerBurnMode.Manual;
            fc.RemoveBurn(playerBurn);
            rcs = RcsFlightSupport.SeedFlyingExecution(vehicle, next!);
            RcsExecutor.Complete(vehicle, fc, rcs, 0f);
            _autoRemoveCalls = 0;
            _autoRemoveFoundBurn = false;
            probe.Patch(AccessTools.Method(typeof(FinishedBurnRemover), nameof(FinishedBurnRemover.OnBurnCompleted)),
                prefix: new HarmonyMethod(typeof(SharedVehicleHooksTest), nameof(ProbeAutoRemove)));
            RaisePendingAlertsWith(vehicle, autoRemove: true);
            probe.UnpatchAll(probe.Id);
            PassCompletionPatch.TickVehicle(vehicle);
            t.Check("the final RCS pass, kept by stock, is removed once by MultiPass and ends MultiPass",
                !fc.AutoBurnCompleted && !MultiPassRegistry.Has(vehicle.Id)
                && fc.BurnPlan.BurnCount == 0 && fc.Burn == null
                && _autoRemoveCalls == 1 && !_autoRemoveFoundBurn,
                $"burns={fc.BurnPlan.BurnCount} autoRemoveCalls={_autoRemoveCalls} " +
                $"autoRemoveFoundBurn={_autoRemoveFoundBurn}");
        }
        finally
        {
            probe.UnpatchAll(probe.Id);
            MultiPassRegistry.Remove(vehicle.Id);
            RcsExecRegistry.Remove(vehicle.Id);
            fc.BurnMode = FlightComputerBurnMode.Manual;
            RcsFlightSupport.CleanupBurns(fc);
        }
    }

    // Stock FlightComputer.EndAutoBurn removes a finished pass itself when another burn follows it,
    // so MultiPass must advance without removing a burn of its own, and a later player burn survives
    // every pass. The worker raises the completion in a step, and the test calls RaisePendingAlerts before
    // the next step would, with MultiPass enabled only for that call so the per-frame tick stays manual.
    private static void CheckStockCompletionHandover(TestContext t, Vehicle vehicle)
    {
        SimDriver driver = t.Session.CreateDriver();
        driver.Step(0.05, 2);
        FlightComputer fc = vehicle.FlightComputer;
        Burn pass = AddPlanBurn(vehicle, 60.0);
        Burn playerBurn = AddPlanBurn(vehicle, 7200.0);
        var exec = new MultiPassExecution
        {
            SaveId = SaveLoadObserver.CurrentSaveId,
            VehicleId = vehicle.Id,
            Mode = SplitMode.EqualDv,
            PassCountTotal = 2,
            Intent = new NextBurnIntent(),
        };
        exec.AssignCurrentBurn(pass);
        exec.AwaitingMaterialization = false;
        MultiPassRegistry.Add(exec);

        try
        {
            if (!t.Check("stock completion setup loads the pass", CompleteLoadedBurnInAuto(vehicle, driver)))
                return;
            RaisePendingAlertsWithMultiPass(vehicle);
            if (!t.Check("stock completion advances MultiPass without a second removal",
                    exec.PassIndex == 1 && exec.CurrentBurn == null && exec.ReengageAutoOnNextBurn
                    && !fc.BurnPlan.TryGetBurn(pass) && fc.BurnPlan.TryGetBurn(playerBurn)
                    && fc.BurnPlan.BurnCount == 1,
                    $"passIndex={exec.PassIndex} burns={fc.BurnPlan.BurnCount}"))
                return;

            PassCompletionPatch.TickVehicle(vehicle);
            driver.Step(0.05);
            PassCompletionPatch.TickVehicle(vehicle);
            Burn? next = exec.CurrentBurn;
            if (!t.Check("the next pass is planned ahead of the player burn and armed",
                    next != null && !exec.AwaitingMaterialization
                    && ReferenceEquals(fc.BurnPlan.FindFirstExecutableBurn(), next)
                    && fc.BurnMode == FlightComputerBurnMode.Auto && exec.PassArmed
                    && fc.BurnPlan.TryGetBurn(playerBurn) && fc.BurnPlan.BurnCount == 2,
                    $"burns={fc.BurnPlan.BurnCount} mode={fc.BurnMode}"))
                return;

            if (!t.Check("final pass completes in stock Auto", CompleteLoadedBurnInAuto(vehicle, driver)))
                return;
            RaisePendingAlertsWithMultiPass(vehicle);
            PassCompletionPatch.TickVehicle(vehicle);
            t.Check("the final pass ends MultiPass and the player burn is loaded",
                !MultiPassRegistry.Has(vehicle.Id) && fc.BurnPlan.BurnCount == 1
                && fc.BurnPlan.TryGetBurn(playerBurn) && !fc.BurnPlan.TryGetBurn(next!),
                $"burns={fc.BurnPlan.BurnCount}");
        }
        finally
        {
            MultiPassRegistry.Remove(vehicle.Id);
            fc.BurnMode = FlightComputerBurnMode.Manual;
            RcsFlightSupport.CleanupBurns(fc);
        }
    }

    // A propellant stop with another burn planned makes stock remove the running pass. MultiPass
    // plans the same pass again from the current orbit and leaves it in Manual for the player.
    // The flag is set by hand, because a dry engine is not needed to exercise the stock handler.
    private static void CheckStockPropellantStop(TestContext t, Vehicle vehicle)
    {
        SimDriver driver = t.Session.CreateDriver();
        driver.Step(0.05, 2);
        FlightComputer fc = vehicle.FlightComputer;
        Burn pass = AddPlanBurn(vehicle, 60.0);
        Burn playerBurn = AddPlanBurn(vehicle, 7200.0);
        var exec = new MultiPassExecution
        {
            SaveId = SaveLoadObserver.CurrentSaveId,
            VehicleId = vehicle.Id,
            Mode = SplitMode.EqualDv,
            PassCountTotal = 2,
            Intent = new NextBurnIntent(),
        };
        exec.AssignCurrentBurn(pass);
        exec.AwaitingMaterialization = false;
        MultiPassRegistry.Add(exec);

        try
        {
            fc.BurnMode = FlightComputerBurnMode.Manual;
            fc.AutoBurnStoppedOutOfPropellant = true;
            RaisePendingAlertsWithMultiPass(vehicle);
            if (!t.Check("a propellant stop keeps the pass index and drops the removed pass",
                    exec.PassIndex == 0 && exec.CurrentBurn == null && !exec.ReengageAutoOnNextBurn
                    && !fc.BurnPlan.TryGetBurn(pass) && fc.BurnPlan.TryGetBurn(playerBurn),
                    $"passIndex={exec.PassIndex} burns={fc.BurnPlan.BurnCount}"))
                return;

            PassCompletionPatch.TickVehicle(vehicle);
            driver.Step(0.05);
            PassCompletionPatch.TickVehicle(vehicle);
            t.Check("the stopped pass is planned again and left in Manual",
                MultiPassRegistry.Has(vehicle.Id) && exec.PassIndex == 0 && exec.CurrentBurn != null
                && !exec.AwaitingMaterialization && fc.BurnMode == FlightComputerBurnMode.Manual
                && fc.BurnPlan.TryGetBurn(playerBurn),
                $"mode={fc.BurnMode} burns={fc.BurnPlan.BurnCount}");
        }
        finally
        {
            MultiPassRegistry.Remove(vehicle.Id);
            fc.BurnMode = FlightComputerBurnMode.Manual;
            RcsFlightSupport.CleanupBurns(fc);
        }
    }

    // A pass stock removed on a propellant stop is planned again only while something aboard can still thrust.
    // With every tank and grain segment empty the execution ends on the stop, and the burn stock kept stays planned.
    // A dry pass that stock kept, the last planned burn, still waits in Manual, so the player can refuel and engage it again.
    // OnStockBurnEnded also cancels the execution when it throws, so the kept case, which must stay registered, tells the intended path from that one.
    private static void CheckDryPropellantStop(TestContext t, Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.CleanupBurns(fc);
        Burn pass = AddPlanBurn(vehicle, 60.0);
        Burn playerBurn = AddPlanBurn(vehicle, 7200.0);
        var exec = new MultiPassExecution
        {
            SaveId = SaveLoadObserver.CurrentSaveId,
            VehicleId = vehicle.Id,
            Mode = SplitMode.EqualDv,
            PassCountTotal = 2,
            Intent = new NextBurnIntent(),
        };
        exec.AssignCurrentBurn(pass);
        exec.AwaitingMaterialization = false;
        MultiPassRegistry.Add(exec);

        try
        {
            foreach (Tank tank in vehicle.Parts.Modules.Get<Tank>())
                tank.DepleteAll(vehicle.Parts.Moles);
            foreach (SolidGrainSegment segment in vehicle.Parts.Modules.Get<SolidGrainSegment>())
                segment.DepleteAll(vehicle.Parts.Moles);
            if (!t.Check("dry stop: no engine or RCS thruster can draw propellant",
                    !VehiclePropellant.AnyUsable(vehicle, includeRcs: false)
                    && !VehiclePropellant.AnyUsable(vehicle, includeRcs: true)))
                return;

            fc.BurnMode = FlightComputerBurnMode.Manual;
            fc.AutoBurnStoppedOutOfPropellant = true;
            RaisePendingAlertsWithMultiPass(vehicle);
            t.Check("dry stop: the execution ends instead of planning the pass again",
                !MultiPassRegistry.Has(vehicle.Id) && !fc.BurnPlan.TryGetBurn(pass) && fc.BurnPlan.TryGetBurn(playerBurn),
                $"registered={MultiPassRegistry.Has(vehicle.Id)} burns={fc.BurnPlan.BurnCount}");

            RcsFlightSupport.CleanupBurns(fc);
            Burn kept = AddPlanBurn(vehicle, 60.0);
            exec.AssignCurrentBurn(kept);
            exec.AwaitingMaterialization = false;
            exec.StallHintShown = false;
            MultiPassRegistry.Add(exec);
            fc.BurnMode = FlightComputerBurnMode.Manual;
            fc.AutoBurnStoppedOutOfPropellant = true;
            RaisePendingAlertsWithMultiPass(vehicle);
            t.Check("dry stop: a pass stock kept waits in Manual with the execution",
                MultiPassRegistry.Has(vehicle.Id) && exec.StallHintShown && fc.BurnPlan.TryGetBurn(kept)
                && ReferenceEquals(exec.CurrentBurn, kept),
                $"registered={MultiPassRegistry.Has(vehicle.Id)} hint={exec.StallHintShown} burns={fc.BurnPlan.BurnCount}");
        }
        finally
        {
            MultiPassRegistry.Remove(vehicle.Id);
            fc.BurnMode = FlightComputerBurnMode.Manual;
            RcsFlightSupport.CleanupBurns(fc);
        }
    }

    // The driver joins the workers before Step returns, so the plan can be changed directly here.
    private static Burn AddPlanBurn(Vehicle vehicle, double leadSeconds)
    {
        UniverseTime now = Universe.GetElapsedTime();
        PatchedConic patch = new PatchedConic(now, UniverseTime.EndOfTime, PatchTransition.Burn,
            PatchTransition.Final, Orbit.CreateFrom(vehicle.Orbit), vehicle.ParentPatchIdHash);
        Burn burn = Burn.Create(OrbitPointCce.Zero, (now + leadSeconds).Seconds(), new double3(0.5, 0.0, 0.0), patch, vehicle);
        vehicle.FlightComputer.AddBurn(burn);
        return burn;
    }

    // Arms Auto on the loaded burn, then overshoots its target so the worker raises AutoBurnCompleted.
    private static bool CompleteLoadedBurnInAuto(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (fc.Burn == null)
            return false;
        fc.Burn.DeltaVAccumCci = float3.Zero;
        fc.BurnMode = FlightComputerBurnMode.Auto;
        driver.Step(0.05);
        if (fc.BurnMode != FlightComputerBurnMode.Auto || fc.Burn == null)
            return false;
        fc.Burn.DeltaVAccumCci = fc.Burn.DeltaVTargetCci * 1.01f;
        driver.Step(0.05);
        return fc.AutoBurnCompleted;
    }

    private static void RaisePendingAlertsWithMultiPass(Vehicle vehicle)
        => RaisePendingAlertsWith(vehicle, autoRemove: false);

    private static void RaisePendingAlertsWith(Vehicle vehicle, bool autoRemove)
    {
        SharedVehicleHooks.MultiPassEnabled = true;
        SharedVehicleHooks.AutoRemoveEnabled = autoRemove;
        bool autoRemoveSwitch = AutoRemoveConfig.Enabled;
        AutoRemoveConfig.Enabled = true;
        try
        {
            vehicle.FlightComputer.RaisePendingAlerts(vehicle);
        }
        finally
        {
            SharedVehicleHooks.MultiPassEnabled = false;
            SharedVehicleHooks.AutoRemoveEnabled = false;
            AutoRemoveConfig.Enabled = autoRemoveSwitch;
        }
    }

    // In the frame a pass ends, the ApplyVehicleSolvers postfix ticks MultiPass with the pass in Manual and still planned,
    // because FlightComputer.RaisePendingAlerts acts on the worker's flag only later in that frame. The flags are set by hand
    // past the impulse time, which is where a finite burn completes, so only the flag gate keeps the stall hint back.
    private static void CheckStallHintWaitsForStockEnd(TestContext t, Vehicle vehicle)
    {
        SimDriver driver = t.Session.CreateDriver();
        driver.Step(0.05, 2);
        FlightComputer fc = vehicle.FlightComputer;
        Burn pass = AddPlanBurn(vehicle, StallBurnLeadSec);
        var exec = new MultiPassExecution
        {
            SaveId = SaveLoadObserver.CurrentSaveId,
            VehicleId = vehicle.Id,
            Mode = SplitMode.EqualDv,
            PassCountTotal = 2,
            Intent = new NextBurnIntent(),
        };
        exec.AssignCurrentBurn(pass);
        exec.AwaitingMaterialization = false;
        exec.PassArmed = true;
        MultiPassRegistry.Add(exec);

        try
        {
            fc.BurnMode = FlightComputerBurnMode.Manual;
            driver.Step(0.1, StallStepCount);
            if (!t.Check("the pass is planned in Manual past its impulse time",
                    fc.BurnPlan.TryGetBurn(pass) && Universe.GetElapsedTime() >= pass.ImpulseTime))
                return;

            fc.AutoBurnCompleted = true;
            PassCompletionPatch.TickVehicle(vehicle);
            fc.AutoBurnCompleted = false;
            fc.AutoBurnStoppedOutOfPropellant = true;
            PassCompletionPatch.TickVehicle(vehicle);
            fc.AutoBurnStoppedOutOfPropellant = false;
            t.Check("a pending stock completion or propellant stop shows no stall hint", !exec.StallHintShown);

            PassCompletionPatch.TickVehicle(vehicle);
            t.Check("without a pending stock end the same pass reports the stall", exec.StallHintShown);
        }
        finally
        {
            MultiPassRegistry.Remove(vehicle.Id);
            RcsFlightSupport.CleanupBurns(fc);
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

    // Records whether the burn AutoRemove is handed is still planned when it runs.
    private static void ProbeAutoRemove(Vehicle vehicle, Burn burn)
    {
        _autoRemoveCalls++;
        _autoRemoveFoundBurn |= vehicle.FlightComputer.BurnPlan.TryGetBurn(burn);
    }

    private static bool RecordAutoRemove()
    {
        Calls.Add("AutoRemove");
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

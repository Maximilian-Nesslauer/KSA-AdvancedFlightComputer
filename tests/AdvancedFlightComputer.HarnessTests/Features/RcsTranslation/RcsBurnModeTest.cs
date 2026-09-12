using System.Reflection;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using AdvancedFlightComputer.Core;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Starts with Auto already set because the normal intercepted Auto click does not write that value.
public sealed class RcsBurnModeTest : AfcTest
{
    private static bool _failAttitude;

    public override string Name => "afc-rcs-burn-mode";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(RcsTestVehicles.Candidates);
        if (saves.Count == 0)
        {
            t.Skip("no RCS test vehicle save present.");
            return;
        }

        using RcsTestPatches.Scope patches = RcsTestPatches.Apply();
        AReleaseGivesTheModeBack(t, home, saves[0]);
        AUserStopKeepsManual(t, home, saves[0]);
        ACompletedBurnKeepsManual(t, home, saves[0]);
        AFailedCleanupHoldsTheModeUntilItSucceeds(t, home, saves[0]);
        AStopDuringPendingCleanupKeepsManual(t, home, saves[0]);
        TheCaptureSurvivesSaveAndLoad(t);
    }

    private static void AReleaseGivesTheModeBack(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessBurnModeRelease", (vehicle, driver) =>
        {
            FlightComputer fc = vehicle.FlightComputer;
            if (!Engage(t, vehicle, driver, out RcsExecution? exec))
                return;
            t.Check("taking the burn records the stock arming and holds Manual",
                exec.ForcedBurnManual && fc.BurnMode == FlightComputerBurnMode.Manual);

            RcsExecutor.Cancel(vehicle, exec, "no usable translation");
            t.Check("a release AFC decided on gives the mode back",
                fc.BurnMode == FlightComputerBurnMode.Auto && !exec.ForcedBurnManual);
        });
    }

    private static void AUserStopKeepsManual(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessBurnModeStop", (vehicle, driver) =>
        {
            FlightComputer fc = vehicle.FlightComputer;
            if (!Engage(t, vehicle, driver, out RcsExecution? exec))
                return;

            // The production handler for the gauge button and the burn-mode hotkeys.
            vehicle.SetEnum(FlightComputerBurnMode.Auto);
            t.Check("a stop the player asked for ends the burn", !exec.IsActive);
            t.Check("a stop the player asked for does not arm stock Auto",
                fc.BurnMode == FlightComputerBurnMode.Manual && !exec.ForcedBurnManual);
        });
    }

    private static void ACompletedBurnKeepsManual(TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessBurnModeComplete", (vehicle, driver) =>
        {
            FlightComputer fc = vehicle.FlightComputer;
            if (!Engage(t, vehicle, driver, out RcsExecution? exec))
                return;

            // Call completion directly to check mode restoration without flying the burn.
            typeof(RcsExecutor).GetMethod("Complete", BindingFlags.NonPublic | BindingFlags.Static)!
                .Invoke(null, [vehicle, fc, exec, 0f]);
            t.Check("a completed burn stays in Manual, the way stock leaves one",
                fc.BurnMode == FlightComputerBurnMode.Manual && !exec.ForcedBurnManual);
        });
    }

    private static void AFailedCleanupHoldsTheModeUntilItSucceeds(
        TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessBurnModeFault", (vehicle, driver) =>
        {
            FlightComputer fc = vehicle.FlightComputer;
            if (!Engage(t, vehicle, driver, out RcsExecution? exec))
                return;

            Harmony harmony = new("com.maxi.afc.harnesstests.burnmode");
            harmony.Patch(AccessTools.Method(typeof(FlightComputer), nameof(FlightComputer.SetNullRot)),
                prefix: new HarmonyMethod(typeof(RcsBurnModeTest), nameof(RefuseAttitudeRelease)));
            try
            {
                // The injected failure needs a commanded tracker for cleanup to attempt its release.
                exec.AlignCommanded = true;
                _failAttitude = true;
                RcsExecutor.RequestCancel(exec, "no usable translation");
                RcsDriverPatch.TickVehicle(vehicle);
                t.Check("a failed cleanup keeps the mode in Manual",
                    fc.BurnMode == FlightComputerBurnMode.Manual
                    && exec.ForcedBurnManual && exec.CleanupPending);
            }
            finally
            {
                _failAttitude = false;
                harmony.UnpatchAll(harmony.Id);
            }

            RcsDriverPatch.TickVehicle(vehicle);
            t.Check("the retry gives the mode back once the rest of the cleanup succeeds",
                fc.BurnMode == FlightComputerBurnMode.Auto
                && !exec.ForcedBurnManual && !exec.CleanupPending);
        });
    }

    // A burn-mode request while cleanup is pending is suppressed and only queues the retry, so the
    // stop the player asked for has to be recorded before that return.
    private static void AStopDuringPendingCleanupKeepsManual(
        TestContext t, IParentBody home, string save)
    {
        Fly(t, home, save, "HarnessBurnModePending", (vehicle, driver) =>
        {
            FlightComputer fc = vehicle.FlightComputer;
            if (!Engage(t, vehicle, driver, out RcsExecution? exec))
                return;

            Harmony harmony = new("com.maxi.afc.harnesstests.burnmode.pending");
            harmony.Patch(AccessTools.Method(typeof(FlightComputer), nameof(FlightComputer.SetNullRot)),
                prefix: new HarmonyMethod(typeof(RcsBurnModeTest), nameof(RefuseAttitudeRelease)));
            try
            {
                exec.AlignCommanded = true;
                _failAttitude = true;
                RcsExecutor.RequestCancel(exec, "no usable translation");
                RcsDriverPatch.TickVehicle(vehicle);
                if (!t.Check("the cleanup is pending for this case", exec.CleanupPending))
                    return;

                vehicle.SetEnum(FlightComputerBurnMode.Manual);
                t.Check("a request during pending cleanup drops the saved Auto",
                    !exec.ForcedBurnManual);
            }
            finally
            {
                _failAttitude = false;
                harmony.UnpatchAll(harmony.Id);
            }

            RcsDriverPatch.TickVehicle(vehicle);
            t.Check("the retry after that request leaves the craft in Manual",
                fc.BurnMode == FlightComputerBurnMode.Manual && !exec.CleanupPending);
        });
    }

    private static void TheCaptureSurvivesSaveAndLoad(TestContext t)
    {
        RcsExecution exec = new() { SaveId = "burn-mode-save", VehicleId = "burn-mode-craft" };
        exec.GetOrCreateOptions(100.0, 20.0).Mode = RcsExecutionMode.Rcs;
        exec.ActiveBurnTimeSec = 100.0;
        exec.ActiveBurnDvMs = 20.0;
        exec.ResolvedStrategy = RcsAttitudeStrategy.Hold;
        exec.ResolvedAxis = -1;
        exec.ForcedBurnManual = true;

        using StringWriter writer = new();
        RcsExecRegistry.WriteToml(writer, [exec]);
        string written = writer.ToString();
        Dictionary<(string SaveId, string VehicleId), RcsExecution> entries = new();
        if (!t.Check("the saved capture loads",
                RcsExecRegistry.ParseLines(written.Split('\n'), "test", entries)))
            return;
        t.Check("the capture survives a save and load",
            entries[("burn-mode-save", "burn-mode-craft")].ForcedBurnManual);

        // Records written before this field must keep loading, so the key is optional on read.
        Dictionary<(string SaveId, string VehicleId), RcsExecution> older = new();
        string[] withoutKey = written.Split('\n')
            .Where(line => !line.StartsWith("forced_burn_manual", StringComparison.Ordinal)).ToArray();
        if (!t.Check("a record written without the key still loads",
                RcsExecRegistry.ParseLines(withoutKey, "test", older)))
            return;
        t.Check("a record without the key claims no stock arming",
            !older[("burn-mode-save", "burn-mode-craft")].ForcedBurnManual);
    }

    // Set Auto before activation to exercise capture of an existing mode.
    private static bool Engage(
        TestContext t, Vehicle vehicle, SimDriver driver, out RcsExecution exec)
    {
        exec = null!;
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.CleanupBurns(fc);
        fc.BurnMode = FlightComputerBurnMode.Manual;
        TestSupport.SetManualControlInputs(vehicle, 0f, engineOn: false);
        driver.Step(0.05, 40);
        if (!RcsCapability.Probe(vehicle).HasAnyTranslation)
        {
            t.Skip("save has no active RCS translation capability.");
            return false;
        }
        if (RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitX, 1.0, 30.0) == null)
        {
            t.Fail("burn mode setup", "no patch or loaded burn target");
            return false;
        }

        fc.BurnMode = FlightComputerBurnMode.Auto;
        RcsExecutor.Activate(vehicle);
        if (!RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? started) || !started.IsActive)
        {
            t.Fail("burn mode setup", "the RCS burn did not engage");
            return false;
        }
        exec = started;
        return true;
    }

    private static void Fly(
        TestContext t, IParentBody home, string save, string spawnName, Action<Vehicle, SimDriver> body)
        => RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, spawnName, (vehicle, driver) =>
        {
            try
            {
                body(vehicle, driver);
            }
            finally
            {
                VehicleControlOwnership.ReleaseAll(vehicle);
                vehicle.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
            }
        });

    private static void RefuseAttitudeRelease()
    {
        if (_failAttitude)
            throw new InvalidOperationException("Injected attitude cleanup fault");
    }
}

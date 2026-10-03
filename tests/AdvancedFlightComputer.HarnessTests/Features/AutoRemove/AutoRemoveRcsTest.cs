using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoRemove;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// An RCS burn ends through the stock flags, so stock FlightComputer.RaisePendingAlerts ends it the
// way it ends an engine auto-burn, and AutoRemove sees it through the same postfix. The execution is
// seeded on a real planned burn and ended through RcsExecutor.Complete and
// RcsExecutor.StopOutOfPropellant, so the test does not depend on the thruster layout of the copied
// vehicle. RaisePendingAlerts is then called directly, as the next driver step would call it.
//
// Scenarios, each from a fresh plan:
//   last-burn:     the only burn completes, stock stops the warp and keeps the burn, and the
//                  feature removes it.
//   three-burns:   the first of three completes, stock removes it and loads the second in Manual,
//                  and the feature removes nothing more.
//   disabled:      the switch is off, the burn stays; switching back on does not remove it
//                  retroactively.
//   uncontrolled:  completed on a vehicle that is not controlled, the burn is removed.
//   propellant:    the only burn stops without RCS propellant, stock keeps it and so does the
//                  feature.
//   cancel:        a cancelled execution raises no flag, the burn stays.
public sealed class AutoRemoveRcsTest : AfcTest
{
    private const double StepDt = 1.0;
    private const double SpawnAltitudeOffsetM = 700_000.0;
    private const double BurnLeadSeconds = 3600.0;
    private const double BurnDvMps = 5.0;
    private const int SettleSteps = 5;
    private const double WarpSpeed = 4.0;

    public override string Name => "afc-autoremove-rcs";

    protected override void Execute(TestContext t)
    {
        Vehicle? source = AutoRemoveBurnTest.FirstVehicle(t.System);
        if (source == null)
        {
            t.Skip("the loaded system has no vehicle to copy");
            return;
        }

        Vehicle? originalControlled = Program.ControlledVehicle;
        double originalSpeed = Universe.GetSimulationSpeed();
        Vehicle? vehicle = null;
        using AutoRemoveTestPatches.Scope patches = AutoRemoveTestPatches.Apply();
        try
        {
            // Inside the try, because the constructor registers with the system before the spawn finishes.
            UniverseTime now = Universe.GetElapsedTime();
            IParentBody parent = source.Orbit.Parent;
            Orbit orbit = VehicleSpawner.CircularCci(parent, source.Orbit.SemiMajorAxis + SpawnAltitudeOffsetM, now);
            vehicle = VehicleSpawner.SpawnCopy(source, parent, "AfcAutoRemoveRcs", orbit);
            Program.ControlledVehicle = vehicle;
            SimDriver driver = t.Session.CreateDriver();
            driver.Step(StepDt, SettleSteps);
            Cleanup(vehicle);

            ScenarioLastBurnRemoved(t, vehicle, driver);
            ScenarioThreeBurnsKeepsTheRest(t, vehicle, driver);
            ScenarioDisabledKeeps(t, vehicle, driver);
            ScenarioUncontrolledRemoves(t, vehicle, driver);
            ScenarioPropellantStopKeeps(t, vehicle, driver);
            ScenarioCancelKeeps(t, vehicle, driver);
        }
        finally
        {
            Universe.SetSimulationSpeed(originalSpeed, alert: false);
            Program.ControlledVehicle = originalControlled;
            if (vehicle != null)
            {
                RcsExecRegistry.Remove(vehicle.Id);
                VehicleSpawner.Despawn(vehicle);
            }
        }
    }

    private static void ScenarioLastBurnRemoved(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (BeginScenario(t, "last-burn", vehicle, driver, out Burn? burn))
        {
            RcsExecution exec = RcsFlightSupport.SeedFlyingExecution(vehicle, burn!);
            RcsExecutor.Complete(vehicle, fc, exec, 0f);
            t.Check("last-burn: the executor raised the completion flag", fc.AutoBurnCompleted && !exec.IsActive);

            Universe.SetSimulationSpeed(WarpSpeed, alert: false);
            fc.RaisePendingAlerts(vehicle);
            t.Check("last-burn: stock dropped the warp to real time", Universe.GetSimulationSpeed() == 1.0,
                $"speed={Universe.GetSimulationSpeed()}");
            t.Check("last-burn: burn removed from the plan", !fc.BurnPlan.HasActiveBurns && fc.Burn == null);
            t.Check("last-burn: stock consumed the flag", !fc.AutoBurnCompleted);
        }
        Cleanup(vehicle);
    }

    // FlightComputer.EndAutoBurn removes the finished first burn itself whenever another follows it,
    // so a second removal by the feature would lose the burn stock just loaded.
    private static void ScenarioThreeBurnsKeepsTheRest(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (BeginScenario(t, "three-burns", vehicle, driver, out Burn? first))
        {
            Burn second = QueueBurn(vehicle, driver, BurnLeadSeconds * 2.0);
            Burn third = QueueBurn(vehicle, driver, BurnLeadSeconds * 3.0);
            if (t.Check("three-burns: three burns planned", fc.BurnPlan.BurnCount == 3))
            {
                RcsExecution exec = RcsFlightSupport.SeedFlyingExecution(vehicle, first!);
                RcsExecutor.Complete(vehicle, fc, exec, 0f);
                fc.RaisePendingAlerts(vehicle);
                t.Check("three-burns: only the finished burn left the plan",
                    fc.BurnPlan.BurnCount == 2
                    && ReferenceEquals(BurnAt(fc, 0), second) && ReferenceEquals(BurnAt(fc, 1), third),
                    $"burns={fc.BurnPlan.BurnCount}");
                t.Check("three-burns: the second burn is loaded in Manual",
                    StockBurnIdentity.IsLoaded(fc.Burn, second) && fc.BurnMode == FlightComputerBurnMode.Manual);
            }
        }
        Cleanup(vehicle);
    }

    private static void ScenarioDisabledKeeps(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        AutoRemoveConfig.Enabled = false;
        if (BeginScenario(t, "disabled", vehicle, driver, out Burn? burn))
        {
            RcsExecution exec = RcsFlightSupport.SeedFlyingExecution(vehicle, burn!);
            RcsExecutor.Complete(vehicle, fc, exec, 0f);
            fc.RaisePendingAlerts(vehicle);
            t.Check("disabled: burn kept while the switch is off", fc.BurnPlan.HasActiveBurns);

            // Stock has consumed the completion, so switching on later must not act on it.
            AutoRemoveConfig.Enabled = true;
            driver.Step(StepDt);
            t.Check("disabled: no retroactive removal after switching on", fc.BurnPlan.HasActiveBurns);
        }
        AutoRemoveConfig.Enabled = true;
        Cleanup(vehicle);
    }

    // Stock ends the burns of every vehicle, so the last one is removed on a vehicle that is not controlled too.
    private static void ScenarioUncontrolledRemoves(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        Program.ControlledVehicle = null;
        if (BeginScenario(t, "uncontrolled", vehicle, driver, out Burn? burn))
        {
            RcsExecution exec = RcsFlightSupport.SeedFlyingExecution(vehicle, burn!);
            RcsExecutor.Complete(vehicle, fc, exec, 0f);
            fc.RaisePendingAlerts(vehicle);
            t.Check("uncontrolled: burn removed on a vehicle that is not controlled", !fc.BurnPlan.HasActiveBurns);
        }
        Program.ControlledVehicle = vehicle;
        Cleanup(vehicle);
    }

    // A stop is not a completion, so the last burn stays planned for a restart after a refill.
    private static void ScenarioPropellantStopKeeps(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (BeginScenario(t, "propellant", vehicle, driver, out Burn? burn))
        {
            RcsExecution exec = RcsFlightSupport.SeedFlyingExecution(vehicle, burn!);
            RcsExecutor.StopOutOfPropellant(vehicle, fc, exec);
            t.Check("propellant: the executor raised the propellant flag",
                fc.AutoBurnStoppedOutOfPropellant && !fc.AutoBurnCompleted && !exec.IsActive);
            fc.RaisePendingAlerts(vehicle);
            t.Check("propellant: the stopped last burn stays in the plan",
                fc.BurnPlan.BurnCount == 1 && ReferenceEquals(BurnAt(fc, 0), burn)
                && !fc.AutoBurnStoppedOutOfPropellant);
        }
        Cleanup(vehicle);
    }

    private static void ScenarioCancelKeeps(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (BeginScenario(t, "cancel", vehicle, driver, out Burn? burn))
        {
            RcsExecution exec = RcsFlightSupport.SeedFlyingExecution(vehicle, burn!);
            RcsExecutor.Cancel(vehicle, exec, "test cancel");
            t.Check("cancel: no stock flag raised",
                !exec.IsActive && !fc.AutoBurnCompleted && !fc.AutoBurnStoppedOutOfPropellant);
            fc.RaisePendingAlerts(vehicle);
            t.Check("cancel: burn kept in the plan", fc.BurnPlan.BurnCount == 1);
        }
        Cleanup(vehicle);
    }

    // One fresh future burn in the plan, through the same input-event path the burn UI uses.
    private static bool BeginScenario(TestContext t, string scenario, Vehicle vehicle, SimDriver driver, out Burn? burn)
    {
        FlightComputer fc = vehicle.FlightComputer;
        fc.BurnMode = FlightComputerBurnMode.Manual;
        burn = QueueBurn(vehicle, driver, BurnLeadSeconds);
        return t.Check($"{scenario}: burn added and loaded",
            fc.BurnPlan.BurnCount == 1 && StockBurnIdentity.IsLoaded(fc.Burn, burn));
    }

    // Burns sort by time, so leadSeconds also decides which one the flight computer loads.
    private static Burn QueueBurn(Vehicle vehicle, SimDriver driver, double leadSeconds)
    {
        UniverseTime now = Universe.GetElapsedTime();
        PatchedConic patch = new PatchedConic(now, UniverseTime.EndOfTime, PatchTransition.Burn,
            PatchTransition.Final, Orbit.CreateFrom(vehicle.Orbit), vehicle.ParentPatchIdHash);
        Burn burn = Burn.Create(OrbitPointCce.Zero, (now + leadSeconds).Seconds(),
            new double3(BurnDvMps, 0.0, 0.0), patch, vehicle);
        InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
        {
            FlightComputer = vehicle.FlightComputer,
            Burn = burn,
            AddBurn = true,
        });
        driver.Step(StepDt);
        return burn;
    }

    private static Burn? BurnAt(FlightComputer fc, int index)
        => fc.BurnPlan.TryGetBurn(index, out Burn? burn) ? burn : null;

    private static void Cleanup(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        RcsExecRegistry.Remove(vehicle.Id);
        while (fc.BurnPlan.HasActiveBurns)
            fc.RemoveBurnAt(0);
    }
}

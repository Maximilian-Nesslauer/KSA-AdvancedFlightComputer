using AdvancedFlightComputer.Features.AutoRemove;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The RCS completion path: delivery through the real RcsBurnCompletions event, which proves the
// subscription the feature makes, and the removal policy on the handler itself.
//
// Scenarios, each from one fresh future burn in the plan:
//   raised-event: the feature's subscription receives the event, the burn is removed.
//   disabled:     the switch is off, the burn stays.
//   uncontrolled: completion for a vehicle that is not controlled, the burn stays.
//   stale-burn:   completion for a burn already removed, no effect and no throw.
public sealed class AutoRemoveRcsTest : AfcTest
{
    private const double StepDt = 1.0;
    private const double SpawnAltitudeOffsetM = 700_000.0;
    private const double BurnLeadSeconds = 3600.0;
    private const double BurnDvMps = 5.0;
    private const int SettleSteps = 5;

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

            ScenarioRaisedEventRemoves(t, vehicle, driver);
            ScenarioDisabledKeeps(t, vehicle, driver);
            ScenarioUncontrolledKeeps(t, vehicle, driver);
            ScenarioStaleBurnIsIgnored(t, vehicle, driver);
        }
        finally
        {
            Program.ControlledVehicle = originalControlled;
            if (vehicle != null)
                VehicleSpawner.Despawn(vehicle);
        }
    }

    private static void ScenarioRaisedEventRemoves(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (AddBurn(t, "raised-event", vehicle, driver, out Burn? burn))
        {
            RcsBurnCompletions.Raise(vehicle, burn!);
            t.Check("raised-event: burn removed from the plan", !fc.BurnPlan.HasActiveBurns);
        }
        CleanupBurns(fc);
    }

    private static void ScenarioDisabledKeeps(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        AutoRemoveConfig.Enabled = false;
        if (AddBurn(t, "disabled", vehicle, driver, out Burn? burn))
        {
            FinishedBurnRemover.OnRcsBurnCompleted(vehicle, burn!);
            t.Check("disabled: burn kept while the switch is off", fc.BurnPlan.HasActiveBurns);
        }
        AutoRemoveConfig.Enabled = true;
        CleanupBurns(fc);
    }

    private static void ScenarioUncontrolledKeeps(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        bool added = AddBurn(t, "uncontrolled", vehicle, driver, out Burn? burn);
        Program.ControlledVehicle = null;
        if (added)
        {
            FinishedBurnRemover.OnRcsBurnCompleted(vehicle, burn!);
            t.Check("uncontrolled: burn kept on a vehicle that is not controlled", fc.BurnPlan.HasActiveBurns);
        }
        Program.ControlledVehicle = vehicle;
        CleanupBurns(fc);
    }

    private static void ScenarioStaleBurnIsIgnored(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (AddBurn(t, "stale-burn", vehicle, driver, out Burn? burn))
        {
            CleanupBurns(fc);
            FinishedBurnRemover.OnRcsBurnCompleted(vehicle, burn!);
            t.Check("stale-burn: no burn resurrected or thrown for a removed burn", !fc.BurnPlan.HasActiveBurns);
        }
        CleanupBurns(fc);
    }

    private static bool AddBurn(TestContext t, string scenario, Vehicle vehicle, SimDriver driver, out Burn? burn)
    {
        FlightComputer fc = vehicle.FlightComputer;
        fc.BurnMode = FlightComputerBurnMode.Manual;
        UniverseTime now = Universe.GetElapsedTime();
        PatchedConic patch = new PatchedConic(now, UniverseTime.EndOfTime, PatchTransition.Burn,
            PatchTransition.Final, Orbit.CreateFrom(vehicle.Orbit), vehicle.ParentPatchIdHash);
        burn = Burn.Create(OrbitPointCce.Zero, (now + BurnLeadSeconds).Seconds(),
            new double3(BurnDvMps, 0.0, 0.0), patch, vehicle);
        InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
        {
            FlightComputer = fc,
            Burn = burn,
            AddBurn = true,
        });
        driver.Step(StepDt);
        return t.Check($"{scenario}: burn added to the plan", fc.BurnPlan.HasActiveBurns);
    }

    private static void CleanupBurns(FlightComputer fc)
    {
        while (fc.BurnPlan.HasActiveBurns)
            fc.RemoveBurnAt(0);
    }
}

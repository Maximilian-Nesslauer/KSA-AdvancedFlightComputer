using AdvancedFlightComputer.Features.AutoRemove;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// A real Burn goes in through the game's input queue, the real FlightComputer flips Auto to Manual
// inside the vehicle solver, and the shared tick reacts to that transition. Only the completion
// signal is fabricated: writing DeltaVAccumCci past DeltaVTargetCci reproduces the delta-V reversal
// a finished burn measures, so the test is deterministic and independent of engine content.
//
// The burn sits far in the future on purpose: in Auto the flight computer waits for IgnitionTime
// commanding a zero burn duration, so no engine fires, no propellant is spent, and Auto survives
// from one step to the next.
//
// Scenarios, each from a fresh burn and a cleared previous-mode sample:
//   completed:    Auto armed, completion injected, the burn is removed from the plan.
//   engines-off:  Auto armed, every engine deactivated, stock keeps Auto for a burn still waiting
//                 for ignition, so there is no transition and nothing is removed.
//   disabled:     the switch is off, completion injected, the burn stays; switching back on does
//                 not remove it retroactively.
//   manual:       completion injected in Manual mode, the burn stays.
//   zero-dv:      a zero-delta-V node inserted ahead of the armed burn drops the flight computer
//                 out of Auto by itself, both burns stay.
//   uncontrolled: completed on a vehicle that is not controlled, the burn stays.
public sealed class AutoRemoveBurnTest : AfcTest
{
    private const double StepDt = 1.0;
    // Keeps the copy clear of the source vehicle's bubble.
    private const double SpawnAltitudeOffsetM = 500_000.0;
    private const double BurnLeadSeconds = 3600.0;
    private const double BurnDvMps = 100.0;
    private const int SettleSteps = 5;
    // Activation drains next step, propellant state one later.
    private const int MaxEngineFeedSteps = 10;
    // Margin over the two steps a denied-ignition fallback needs.
    private const int AutoHoldSteps = 6;

    public override string Name => "afc-autoremove-burns";

    protected override void Execute(TestContext t)
    {
        Vehicle? source = FirstVehicle(t.System);
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
            vehicle = VehicleSpawner.SpawnCopy(source, parent, "AfcAutoRemoveBurns", orbit);
            Program.ControlledVehicle = vehicle;
            SimDriver driver = t.Session.CreateDriver();
            driver.Step(StepDt, SettleSteps);

            ScenarioCompletedRemoves(t, vehicle, driver);
            ScenarioEnginesOffKeeps(t, vehicle, driver);
            ScenarioDisabledKeeps(t, vehicle, driver);
            ScenarioManualKeeps(t, vehicle, driver);
            ScenarioZeroDeltaVKeeps(t, vehicle, driver);
            ScenarioUncontrolledKeeps(t, vehicle, driver);
        }
        finally
        {
            Program.ControlledVehicle = originalControlled;
            if (vehicle != null)
                VehicleSpawner.Despawn(vehicle);
        }
    }

    internal static Vehicle? FirstVehicle(CelestialSystem system)
    {
        for (int i = 0; i < system.Count; i++)
        {
            if (system.GetIndex(i) is Vehicle v)
                return v;
        }
        return null;
    }

    private static void ScenarioCompletedRemoves(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (BeginScenario(t, "completed", vehicle, driver) && Arm(t, "completed", vehicle, driver)
            && t.Check("completed: burn still in plan while in progress", fc.BurnPlan.HasActiveBurns))
        {
            InjectCompletion(fc);
            driver.Step(StepDt);
            t.Check("completed: burn removed from the plan", !fc.BurnPlan.HasActiveBurns);
            t.Check("completed: no burn target loaded", fc.Burn == null);
        }
        CleanupBurn(fc);
    }

    // Stock only leaves Auto through the denied-ignition latch, which needs an ignition attempt,
    // so a burn still counting down keeps Auto however long it sits without an engine. The
    // held-Auto check pins stock; the burn-kept check is the feature verdict.
    private static void ScenarioEnginesOffKeeps(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (!EnsureEnginesFed(t, vehicle, driver))
        {
            t.Skip("engines-off: no engine on the copied vehicle ever became active and fed");
            return;
        }

        if (BeginScenario(t, "engines-off", vehicle, driver) && Arm(t, "engines-off", vehicle, driver))
        {
            SetAllEngines(vehicle, active: false);
            t.Check("engines-off: Auto held with no engine to ignite", StepsHoldBurnModeAuto(t, "engines-off", vehicle, driver));
            t.Check("engines-off: burn kept in the plan", fc.BurnPlan.HasActiveBurns);
            // Setup for the later scenarios rather than a verdict, but a failure still fails the run.
            t.Check("engines-off: engines re-fed for the following scenarios", EnsureEnginesFed(t, vehicle, driver));
        }
        CleanupBurn(fc);
    }

    private static void ScenarioDisabledKeeps(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        AutoRemoveConfig.Enabled = false;
        if (BeginScenario(t, "disabled", vehicle, driver) && Arm(t, "disabled", vehicle, driver))
        {
            InjectCompletion(fc);
            driver.Step(StepDt);
            t.Check("disabled: burn kept while the switch is off", fc.BurnPlan.HasActiveBurns);

            // The transition is already consumed, so switching on later must not act on it.
            AutoRemoveConfig.Enabled = true;
            driver.Step(StepDt);
            t.Check("disabled: no retroactive removal after switching on", fc.BurnPlan.HasActiveBurns);
        }
        AutoRemoveConfig.Enabled = true;
        CleanupBurn(fc);
    }

    private static void ScenarioManualKeeps(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (BeginScenario(t, "manual", vehicle, driver))
        {
            InjectCompletion(fc);
            driver.Step(StepDt);
            t.Check("manual: manual burn kept despite the delta-V reversal", fc.BurnPlan.HasActiveBurns);
        }
        CleanupBurn(fc);
    }

    // A node with no delta-V reads as already reversed, and inserting one ahead of the running
    // burn makes stock FlightComputer.AddBurn unload the loaded burn, which flips Auto to Manual.
    private static void ScenarioZeroDeltaVKeeps(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (BeginScenario(t, "zero-dv", vehicle, driver) && Arm(t, "zero-dv", vehicle, driver))
        {
            QueueBurn(vehicle, driver, BurnLeadSeconds * 0.5, double3.Zero);
            t.Check("zero-dv: flight computer dropped out of Auto on the insert", fc.BurnMode == FlightComputerBurnMode.Manual);
            t.Check("zero-dv: both burns kept in the plan", fc.BurnPlan.BurnCount == 2);
        }
        CleanupBurn(fc);
    }

    private static void ScenarioUncontrolledKeeps(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        Program.ControlledVehicle = null;
        if (BeginScenario(t, "uncontrolled", vehicle, driver) && Arm(t, "uncontrolled", vehicle, driver))
        {
            InjectCompletion(fc);
            driver.Step(StepDt);
            t.Check("uncontrolled: burn kept on a vehicle that is not controlled", fc.BurnPlan.HasActiveBurns);
        }
        Program.ControlledVehicle = vehicle;
        CleanupBurn(fc);
    }

    // Previous-mode sample cleared and one fresh future burn in the plan, through the same input-event path the burn UI uses.
    private static bool BeginScenario(TestContext t, string scenario, Vehicle vehicle, SimDriver driver)
    {
        FinishedBurnRemover.Reset();
        FlightComputer fc = vehicle.FlightComputer;
        fc.BurnMode = FlightComputerBurnMode.Manual;
        QueueBurn(vehicle, driver, BurnLeadSeconds, new double3(BurnDvMps, 0.0, 0.0));
        return t.Check($"{scenario}: burn added and burn target loaded", fc.BurnPlan.HasActiveBurns && fc.Burn != null);
    }

    // Burns sort by time, so leadSeconds also decides which one the flight computer loads.
    private static void QueueBurn(Vehicle vehicle, SimDriver driver, double leadSeconds, double3 deltaVVlf)
    {
        UniverseTime now = Universe.GetElapsedTime();
        PatchedConic patch = new PatchedConic(now, UniverseTime.EndOfTime, PatchTransition.Burn,
            PatchTransition.Final, Orbit.CreateFrom(vehicle.Orbit), vehicle.ParentPatchIdHash);
        Burn burn = Burn.Create(OrbitPointCce.Zero, (now + leadSeconds).Seconds(), deltaVVlf, patch, vehicle);
        InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
        {
            FlightComputer = vehicle.FlightComputer,
            Burn = burn,
            AddBurn = true,
        });
        driver.Step(StepDt);
    }

    // Auto has to survive a full step, because the tick can only observe a transition out of a
    // mode it saw recorded. Zeroing DeltaVAccumCci rules out the reversal.
    private static bool Arm(TestContext t, string scenario, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        fc.Burn!.DeltaVAccumCci = float3.Zero;
        fc.BurnMode = FlightComputerBurnMode.Auto;
        driver.Step(StepDt);
        return t.Check($"{scenario}: Auto mode held for a full step", fc.BurnMode == FlightComputerBurnMode.Auto);
    }

    // Overshooting DeltaVTargetCci makes DeltaVToGoCci point backwards, the reversal FlightComputer.UpdateBurnTarget flips on.
    private static void InjectCompletion(FlightComputer fc)
    {
        BurnTarget burn = fc.Burn!;
        burn.DeltaVAccumCci = burn.DeltaVTargetCci * 1.01f;
    }

    private static void CleanupBurn(FlightComputer fc)
    {
        while (fc.BurnPlan.HasActiveBurns)
            fc.RemoveBurnAt(0);
    }

    private static void SetAllEngines(Vehicle vehicle, bool active)
    {
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
            engine.SetIsActive(vehicle, active);
    }

    private static bool EnsureEnginesFed(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        if (vehicle.Parts.Modules.Get<EngineController>().Length == 0)
            return false;
        SetAllEngines(vehicle, active: true);
        for (int i = 0; i < MaxEngineFeedSteps; i++)
        {
            driver.Step(StepDt);
            if (TestSupport.AnyActiveEngineFed(vehicle))
                return true;
        }
        t.Info($"no active engine reported propellant after {MaxEngineFeedSteps} steps");
        return false;
    }

    // Several steps, because stock arms BurnTarget.LastIgnitionDenied on a denied ignition and
    // only leaves Auto if the next step is denied too.
    private static bool StepsHoldBurnModeAuto(TestContext t, string scenario, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        for (int i = 0; i < AutoHoldSteps; i++)
        {
            driver.Step(StepDt);
            if (fc.BurnMode != FlightComputerBurnMode.Auto)
            {
                t.Info($"{scenario}: left Auto for {fc.BurnMode} on step {i + 1} of {AutoHoldSteps}");
                return false;
            }
        }
        return true;
    }
}

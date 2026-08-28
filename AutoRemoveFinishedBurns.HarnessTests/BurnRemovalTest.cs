using AutoRemoveFinishedBurns.Core;
using AutoRemoveFinishedBurns.Features;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AutoRemoveFinishedBurns.HarnessTests;

// Exercises the mod's burn-removal behavior against the live simulation: a real Burn is added
// through the game's input queue, the real FlightComputer flips Auto -> Manual inside the vehicle
// solver, and the mod's Harmony postfix (applied here, since the headless run does not load the mod
// through StarMap) reacts to that transition. Only the completion signal is fabricated: writing
// DeltaVAccumCci past DeltaVTargetCci reproduces the delta-V reversal a finished burn measures,
// which keeps the test deterministic and independent of engine content, while everything the mod
// actually reads (BurnMode transition, dot product, burn plan) is produced by game code.
//
// The burn is placed far in the future on purpose: in Auto mode the flight computer waits for
// IgnitionTime commanding a zero burn duration, so no engine fires, the vehicle coasts, no
// propellant is spent, and Auto survives from one step to the next. The spawned copy's engines are
// activated up front so the engines-off scenario has something to take away.
//
// Scenarios (each starts from a fresh burn and a cleared previous-mode table):
//   1. completed:    Auto armed, completion injected -> the burn is removed from the plan.
//   2. engines-off:  Auto armed, every engine deactivated -> stock keeps Auto for a burn that is
//                    still waiting for ignition -> no transition, so the mod removes nothing.
//   3. disabled:     Config.Enabled = false, completed -> the burn stays; re-enabling afterwards
//                     does not retroactively remove it.
//   4. manual:       completion injected while in Manual mode -> the burn stays.
//   5. zero-dv:      a zero-delta-V node inserted ahead of the armed burn drops the flight computer
//                    out of Auto by itself -> both burns stay.
//   6. uncontrolled: completed on a vehicle that is not Program.ControlledVehicle -> the burn stays.
public sealed class BurnRemovalTest : IHarnessTest
{
    private const string HarmonyId = "com.maxi.autoremovefinishedburns.harnesstests";
    private const string Prefix = "[arfb-burn-removal]";

    private const double StepDt = 1.0;
    private const double SpawnAltitudeOffsetM = 500_000.0; // keep the copy clear of the source vehicle's bubble
    private const double BurnLeadSeconds = 3600.0;
    private const double BurnDvMps = 100.0;
    private const int SettleSteps = 5;                     // update task assignment + flight plan before the first burn
    private const int MaxEngineFeedSteps = 10;             // activation drains next step, propellant state one later
    private const int AutoHoldSteps = 6;                   // margin over the 2 steps a denied-ignition fallback needs

    public string Name => "arfb-burn-removal";

    public int Run(HeadlessSession session)
    {
        if (!GameReflection.ValidateDetection())
        {
            HarnessLog.Line($"{Prefix} FAIL: detection reflection targets missing, cannot apply the patch.");
            return 1;
        }

        CelestialSystem system = session.System;
        Vehicle? source = null;
        for (int i = 0; i < system.Count; i++)
        {
            if (system.GetIndex(i) is Vehicle v)
            {
                source = v;
                break;
            }
        }
        if (source == null)
        {
            HarnessLog.Line($"{Prefix} SKIP: the loaded system has no vehicle to copy.");
            return 0;
        }

        UniverseTime now = Universe.GetElapsedTime();
        IParentBody parent = source.Orbit.Parent;
        Orbit orbit = VehicleSpawner.CircularCci(parent, source.Orbit.SemiMajorAxis + SpawnAltitudeOffsetM, now);
        Vehicle vehicle = VehicleSpawner.SpawnCopy(source, parent, "ArfbTestVehicle", orbit);

        Harmony harmony = new Harmony(HarmonyId);
        bool originalEnabled = Config.Enabled;
        Vehicle? originalControlled = Program.ControlledVehicle;
        bool ok;
        try
        {
            harmony.CreateClassProcessor(typeof(BurnRemovalPatch)).Patch();
            Config.Enabled = true;
            BurnRemovalPatch.Reset();
            Program.ControlledVehicle = vehicle;

            SimDriver driver = session.CreateDriver();
            driver.Step(StepDt, SettleSteps);

            ok = ScenarioCompletedRemoves(vehicle, driver);
            ok &= ScenarioEnginesOffKeeps(vehicle, driver);
            ok &= ScenarioDisabledKeeps(vehicle, driver);
            ok &= ScenarioManualKeeps(vehicle, driver);
            ok &= ScenarioZeroDeltaVKeeps(vehicle, driver);
            ok &= ScenarioUncontrolledKeeps(vehicle, driver);
        }
        finally
        {
            harmony.UnpatchAll(HarmonyId);
            BurnRemovalPatch.Reset();
            Config.Enabled = originalEnabled;
            Program.ControlledVehicle = originalControlled;
            VehicleSpawner.Despawn(vehicle);
        }

        HarnessLog.Line($"{Prefix} {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private static bool ScenarioCompletedRemoves(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        bool ok = BeginScenario("completed", vehicle, driver);
        ok = ok && Arm("completed", vehicle, driver);
        ok = ok && Check("completed", "burn still in plan while in progress", fc.BurnPlan.HasActiveBurns);

        if (ok)
        {
            InjectCompletion(fc);
            driver.Step(StepDt);
            ok &= Check("completed", "burn removed from the plan", !fc.BurnPlan.HasActiveBurns);
            ok &= Check("completed", "no burn target loaded", fc.Burn == null);
        }
        CleanupBurn(fc);
        return ok;
    }

    // A vehicle that cannot burn must not lose its plan. Stock only leaves Auto here through the
    // denied-ignition latch, which needs the flight computer to actually attempt ignition, so a burn
    // still counting down to its ignition time keeps Auto however long it sits without an engine.
    // That leaves the mod with no Auto -> Manual transition to react to, which is the behavior under
    // test: it must not read "cannot burn" as "burn finished".
    //
    // The held-Auto check pins stock, not the mod, so a failure there reports that the game changed
    // its mind about dropping Auto rather than that the mod regressed. The burn-kept check below is
    // the mod verdict. This is the only scenario that needs engines, and it needs them only as
    // something to deactivate.
    private static bool ScenarioEnginesOffKeeps(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (!EnsureEnginesFed(vehicle, driver))
        {
            HarnessLog.Line($"{Prefix} engines-off: SKIP, no engine on the copied vehicle ever became " +
                            "active and fed, so there is nothing to take away.");
            return true;
        }

        bool ok = BeginScenario("engines-off", vehicle, driver);
        ok = ok && Arm("engines-off", vehicle, driver);

        if (ok)
        {
            SetAllEngines(vehicle, active: false);
            ok &= Check("engines-off", "Auto held with no engine to ignite",
                StepsHoldBurnModeAuto("engines-off", vehicle, driver));
            ok &= Check("engines-off", "burn kept in the plan", fc.BurnPlan.HasActiveBurns);

            // Putting the engines back is setup for the later scenarios rather than a verdict on the
            // behavior above, but leaving them off would run the rest of the suite on state it never
            // asked for, so a failure here still fails the run.
            bool refed = EnsureEnginesFed(vehicle, driver);
            if (!refed)
                HarnessLog.Line($"{Prefix} engines-off: could not re-feed the engines for the following scenarios.");
            ok &= refed;
        }
        CleanupBurn(fc);
        return ok;
    }

    private static bool ScenarioDisabledKeeps(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        Config.Enabled = false;
        bool ok = BeginScenario("disabled", vehicle, driver);
        ok = ok && Arm("disabled", vehicle, driver);

        if (ok)
        {
            InjectCompletion(fc);
            driver.Step(StepDt);
            ok &= Check("disabled", "burn kept while the mod is disabled", fc.BurnPlan.HasActiveBurns);

            // The Auto -> Manual transition is already consumed, so re-enabling later must not
            // remove a burn whose completion it never observed while enabled.
            Config.Enabled = true;
            driver.Step(StepDt);
            ok &= Check("disabled", "no retroactive removal after re-enabling", fc.BurnPlan.HasActiveBurns);
        }
        Config.Enabled = true;
        CleanupBurn(fc);
        return ok;
    }

    private static bool ScenarioManualKeeps(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        bool ok = BeginScenario("manual", vehicle, driver);

        if (ok)
        {
            InjectCompletion(fc);
            driver.Step(StepDt);
            ok &= Check("manual", "manual burn kept despite delta-V reversal", fc.BurnPlan.HasActiveBurns);
        }
        CleanupBurn(fc);
        return ok;
    }

    // A node with no delta-V reads as "already reversed" (DeltaVToGoCci and DeltaVTargetCci are both
    // zero, so their dot product is 0), and inserting one ahead of the running burn makes stock
    // FlightComputer.AddBurn unload the loaded burn, which flips Auto -> Manual. Both halves of the
    // completion signal therefore appear without any burn having finished.
    private static bool ScenarioZeroDeltaVKeeps(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        bool ok = BeginScenario("zero-dv", vehicle, driver);
        ok = ok && Arm("zero-dv", vehicle, driver);

        if (ok)
        {
            QueueBurn(vehicle, driver, BurnLeadSeconds * 0.5, double3.Zero);
            ok &= Check("zero-dv", "flight computer dropped out of Auto on the insert",
                fc.BurnMode == FlightComputerBurnMode.Manual);
            ok &= Check("zero-dv", "both burns kept in the plan", fc.BurnPlan.BurnCount == 2);
        }
        CleanupBurn(fc);
        return ok;
    }

    private static bool ScenarioUncontrolledKeeps(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        Program.ControlledVehicle = null;
        bool ok = BeginScenario("uncontrolled", vehicle, driver);

        ok = ok && Arm("uncontrolled", vehicle, driver);

        if (ok)
        {
            InjectCompletion(fc);
            driver.Step(StepDt);
            ok &= Check("uncontrolled", "burn kept on a vehicle that is not controlled", fc.BurnPlan.HasActiveBurns);
        }
        Program.ControlledVehicle = vehicle;
        CleanupBurn(fc);
        return ok;
    }

    // Every scenario starts identically: previous-mode tracking cleared and one fresh future burn
    // in the plan, added through the same input-event path the game's burn UI uses.
    private static bool BeginScenario(string scenario, Vehicle vehicle, SimDriver driver)
    {
        BurnRemovalPatch.Reset();
        FlightComputer fc = vehicle.FlightComputer;
        fc.BurnMode = FlightComputerBurnMode.Manual;
        QueueBurn(vehicle, driver, BurnLeadSeconds, new double3(BurnDvMps, 0.0, 0.0));
        return Check(scenario, "burn added and burn target loaded",
            fc.BurnPlan.HasActiveBurns && fc.Burn != null);
    }

    // Adds one burn through the same input-event path the game's burn UI uses, then steps so the
    // event drains. Burns sort by time, so leadSeconds also decides which one the flight computer
    // loads.
    private static void QueueBurn(Vehicle vehicle, SimDriver driver, double leadSeconds, double3 deltaVVlf)
    {
        UniverseTime now = Universe.GetElapsedTime();
        PatchedConic patch = new PatchedConic(now, UniverseTime.EndOfTime, PatchTransition.Burn,
            PatchTransition.Final, Orbit.CreateFrom(vehicle.Orbit), vehicle.ParentPatchIdHash);
        Burn burn = Burn.Create(OrbitPointCce.Zero, (now + leadSeconds).Seconds(),
            deltaVVlf, patch, vehicle);
        InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
        {
            FlightComputer = vehicle.FlightComputer,
            Burn = burn,
            AddBurn = true,
        });
        driver.Step(StepDt);
    }

    // Puts the flight computer into Auto and proves it survived a full step (the postfix can only
    // observe a transition out of a mode it saw recorded). Engine state does not decide this: the burn
    // sits far enough out that ignition is never attempted, and zeroing DeltaVAccumCci rules out the
    // delta-V reversal, which together are every way stock leaves Auto on its own.
    private static bool Arm(string scenario, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        fc.Burn!.DeltaVAccumCci = float3.Zero;
        fc.BurnMode = FlightComputerBurnMode.Auto;
        driver.Step(StepDt);
        return Check(scenario, "Auto mode held for a full step", fc.BurnMode == FlightComputerBurnMode.Auto);
    }

    // Reproduces the completion measurement: overshooting DeltaVTargetCci makes DeltaVToGoCci point
    // backwards, the same dot-product reversal FlightComputer.UpdateBurnTarget flips Auto -> Manual on.
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

    // Activates every engine and waits for the propellant state to come up (the activation drains
    // on the next step and per-core availability bootstraps one step later).
    private static bool EnsureEnginesFed(Vehicle vehicle, SimDriver driver)
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
        HarnessLog.Line($"{Prefix} no active engine reported propellant after {MaxEngineFeedSteps} steps.");
        return false;
    }

    // True while the flight computer stays in Auto for the whole window. Several steps rather than one,
    // because the fallback this guards against is latched: stock arms BurnTarget.LastIgnitionDenied on
    // a denied ignition and only leaves Auto if the next step is denied too, so a single step cannot
    // tell "never falls back" from "has not fallen back yet".
    private static bool StepsHoldBurnModeAuto(string scenario, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        for (int i = 0; i < AutoHoldSteps; i++)
        {
            driver.Step(StepDt);
            if (fc.BurnMode != FlightComputerBurnMode.Auto)
            {
                HarnessLog.Line($"{Prefix} {scenario}: left Auto for {fc.BurnMode} on step {i + 1} of {AutoHoldSteps}.");
                return false;
            }
        }
        return true;
    }

    private static bool Check(string scenario, string what, bool pass)
    {
        HarnessLog.Line($"{Prefix} {scenario}: {what} => {TestSupport.Verdict(pass)}");
        return pass;
    }
}

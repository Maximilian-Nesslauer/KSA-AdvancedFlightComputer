using AutoRemoveFinishedBurns.Core;
using AutoRemoveFinishedBurns.Features;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AutoRemoveFinishedBurns.HarnessTests;

// Exercises the AdvancedFlightComputer RCS interop: the reflection binding
// against the deployed AFC assembly, delivery through AFC's actual event
// (invoked via its backing delegate, proving the full subscribe -> raise ->
// remove chain), and the removal policy on the handler itself (enabled,
// controlled vehicle, burn still in plan). AFC not deployed skips the test:
// the binding is a soft dependency by design.
//
// Scenarios (each starts from one fresh future burn in the plan):
//   1. bound-event: handler subscribed via TryEnable, AFC's event delegate
//      invoked -> the burn is removed from the plan.
//   2. disabled:    Config.Enabled = false, handler called -> the burn stays.
//   3. uncontrolled: completion for a vehicle that is not controlled -> stays.
//   4. stale-burn:  completion for a burn already removed -> no effect, no throw.
public sealed class RcsInteropTest : IHarnessTest
{
    private const string Prefix = "[arfb-rcs-interop]";
    private const string CompletionsTypeName =
        "AdvancedFlightComputer.Features.RcsTranslation.RcsBurnCompletions";

    private const double StepDt = 1.0;
    private const double SpawnAltitudeOffsetM = 700_000.0;
    private const double BurnLeadSeconds = 3600.0;
    private const double BurnDvMps = 5.0;
    private const int SettleSteps = 5;

    public string Name => "arfb-rcs-interop";

    public int Run(HeadlessSession session)
    {
        if (AccessTools.TypeByName(CompletionsTypeName) == null)
        {
            HarnessLog.Line($"{Prefix} SKIP: AdvancedFlightComputer not deployed; nothing to bind.");
            return 0;
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
        Orbit orbit = VehicleSpawner.CircularCci(
            parent, source.Orbit.SemiMajorAxis + SpawnAltitudeOffsetM, now);
        Vehicle vehicle = VehicleSpawner.SpawnCopy(source, parent, "ArfbRcsInteropVehicle", orbit);

        bool originalEnabled = Config.Enabled;
        Vehicle? originalControlled = Program.ControlledVehicle;
        bool interopWasActive = AfcRcsInterop.Active;
        bool ok;
        try
        {
            Config.Enabled = true;
            Program.ControlledVehicle = vehicle;
            SimDriver driver = session.CreateDriver();
            driver.Step(StepDt, SettleSteps);

            ok = ScenarioBoundEventRemoves(vehicle, driver);
            ok &= ScenarioDisabledKeeps(vehicle, driver);
            ok &= ScenarioUncontrolledKeeps(vehicle, driver);
            ok &= ScenarioStaleBurnIsIgnored(vehicle, driver);
        }
        finally
        {
            // State-preserving teardown: only drop the subscription when
            // this test created it, so a session where the mod itself
            // enabled the interop keeps it.
            if (!interopWasActive)
                AfcRcsInterop.Disable();
            Config.Enabled = originalEnabled;
            Program.ControlledVehicle = originalControlled;
            VehicleSpawner.Despawn(vehicle);
        }

        HarnessLog.Line($"{Prefix} {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private bool ScenarioBoundEventRemoves(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        Burn? burn = null;
        bool ok = Check("bound-event", "TryEnable bound the AFC event", AfcRcsInterop.TryEnable());
        ok = ok && AddBurn("bound-event", vehicle, driver, out burn);
        if (ok)
        {
            // Raise through AFC's own event delegate (the backing field), so
            // the assertion covers the real subscription, not just the
            // handler in isolation.
            Type type = AccessTools.TypeByName(CompletionsTypeName)!;
            Delegate? evt = AccessTools.Field(type, "Completed")?.GetValue(null) as Delegate;
            ok &= Check("bound-event", "AFC event has a subscriber", evt != null);
            evt?.DynamicInvoke(vehicle, burn);
            ok &= Check("bound-event", "burn removed from the plan", !fc.BurnPlan.HasActiveBurns);
        }
        CleanupBurns(fc);
        return ok;
    }

    private bool ScenarioDisabledKeeps(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        Config.Enabled = false;
        bool ok = AddBurn("disabled", vehicle, driver, out Burn? burn);
        if (ok)
        {
            AfcRcsInterop.OnRcsBurnCompleted(vehicle, burn!);
            ok &= Check("disabled", "burn kept while the mod is disabled", fc.BurnPlan.HasActiveBurns);
        }
        Config.Enabled = true;
        CleanupBurns(fc);
        return ok;
    }

    private bool ScenarioUncontrolledKeeps(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        bool ok = AddBurn("uncontrolled", vehicle, driver, out Burn? burn);
        Program.ControlledVehicle = null;
        if (ok)
        {
            AfcRcsInterop.OnRcsBurnCompleted(vehicle, burn!);
            ok &= Check("uncontrolled", "burn kept on a vehicle that is not controlled",
                fc.BurnPlan.HasActiveBurns);
        }
        Program.ControlledVehicle = vehicle;
        CleanupBurns(fc);
        return ok;
    }

    private bool ScenarioStaleBurnIsIgnored(Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        bool ok = AddBurn("stale-burn", vehicle, driver, out Burn? burn);
        if (ok)
        {
            CleanupBurns(fc);
            AfcRcsInterop.OnRcsBurnCompleted(vehicle, burn!);
            ok &= Check("stale-burn", "no burn resurrected or thrown for a removed burn",
                !fc.BurnPlan.HasActiveBurns);
        }
        CleanupBurns(fc);
        return ok;
    }

    // One fresh future burn through the same input-event path the game's
    // burn UI uses (mirrors BurnRemovalTest.BeginScenario).
    private bool AddBurn(string scenario, Vehicle vehicle, SimDriver driver, out Burn? burn)
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
        return Check(scenario, "burn added to the plan", fc.BurnPlan.HasActiveBurns);
    }

    private static void CleanupBurns(FlightComputer fc)
    {
        while (fc.BurnPlan.HasActiveBurns)
            fc.RemoveBurnAt(0);
    }

    private bool Check(string scenario, string label, bool condition)
    {
        HarnessLog.Line($"{Prefix} {scenario}: {label} => {TestSupport.Verdict(condition)}");
        return condition;
    }
}

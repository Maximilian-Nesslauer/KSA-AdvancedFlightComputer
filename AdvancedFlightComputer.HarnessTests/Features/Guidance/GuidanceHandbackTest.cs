using System.Reflection;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Calls release paths directly because guidance's vehicle driver is disabled.
// Fault injection makes cleanup failures deterministic.
public sealed class GuidanceHandbackTest : AfcTest
{
    public override string Name => "afc-guidance-handback";

    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> Inputs =
        AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

    private static Vehicle? _failing;

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(RcsTestVehicles.Candidates);
        if (saves.Count == 0)
        {
            t.Skip("no test vehicle save present.");
            return;
        }

        // Run the ownership cases before spawning, since they need no vehicle.
        OwnedFieldsAreRestored(t);
        PlayerChangesArePreserved(t);
        ReleaseDoesNotFollowAReplacementComputer(t);

        // Both global operations below reach every craft in the shared session, so they only
        // run when no other craft holds guidance state.
        bool isolated = VehicleAutopilotState.Snapshot().Length == 0;
        if (!isolated)
            t.Skip("another craft holds guidance state, so the global release cases do not run.");

        Vehicle? previousFocus = Program.ControlledVehicle;
        bool previousModActive = GuidanceWindow.ModActive;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        Vehicle? spawned = null;
        Vehicle? spawnedOther = null;
        try
        {
            Orbit orbit = OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed);
            Vehicle vehicle;
            try
            {
                vehicle = VehicleSpawner.SpawnFromSave(saves[0], t.System, home, "HarnessGuidanceRelease", orbit);
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"'{saves[0]}': {e.Message}");
                return;
            }
            spawned = vehicle;
            Vehicle other = VehicleSpawner.SpawnFromSave(
                saves[0], t.System, home, "HarnessGuidanceOther",
                OrbitFixtures.CircularAt(home, 600_000.0, driver.Elapsed));
            spawnedOther = other;

            DisabledCraftIsUntouched(t, vehicle, driver, isolated);
            AcquiredCraftStopsWithoutChangingItsPlan(t, vehicle, other, driver);
            if (isolated)
                FailedReleaseRemainsPending(t, vehicle, other);
            FailedRateCleanupDoesNotBlockAttitudeRelease(t, vehicle);
            AHandoverReleaseKeepsTheEngineLit(t, vehicle, driver);
        }
        finally
        {
            // Only this test's craft are cleared. The harness shares one session, so a global
            // release would take state another test left behind.
            _failing = null;
            GuidanceWindow.ModActive = previousModActive;
            if (spawned != null)
                VehicleAutopilotState.Remove(spawned);
            if (spawnedOther != null)
                VehicleAutopilotState.Remove(spawnedOther);
            Program.ControlledVehicle = previousFocus;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    // A plan with a burn in it, so clearing the same BurnPlan object cannot read as
    // preserving it.
    private sealed record PlannedBurn(Burn Burn, double3 Dv);

    private static PlannedBurn? SeedBurn(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.CleanupBurns(fc);
        RcsFlightSupport.BurnSetup? setup = RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitX, 1.0, 60.0);
        if (setup == null)
        {
            t.Skip("no patch or loaded burn target, so the plan contents are not checked.");
            return null;
        }
        return new PlannedBurn(setup.Burn, setup.Burn.DeltaVVlf);
    }

    private static void CheckPlanContents(TestContext t, FlightComputer computer, PlannedBurn? planned)
    {
        if (planned == null)
            return;
        t.Check("the planned burn survives unchanged",
            computer.BurnPlan.BurnCount == 1
            && ReferenceEquals(computer.BurnPlan.FirstBurn, planned.Burn)
            && planned.Burn.DeltaVVlf.Equals(planned.Dv));
    }

    private static void WriteGuidance(AttitudeOwnership ownership, FlightComputer computer)
    {
        ownership.BeginWrite(computer);
        computer.CustomAttitudeTarget = new double3(1, 2, 3);
        computer.AttitudeFrame = VehicleReferenceFrame.EclBody;
        computer.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
        computer.RollMode = FlightComputerRollMode.Up;
        ownership.EndWrite(computer, writesRoll: true);
    }

    private static void OwnedFieldsAreRestored(TestContext t)
    {
        var computer = new FlightComputer { AttitudeFrame = VehicleReferenceFrame.EnuBody };
        object plan = computer.BurnPlan;
        var ownership = new AttitudeOwnership();
        WriteGuidance(ownership, computer);
        computer.AngleDeadband = 0.75f;

        // Releasing twice must leave the same result.
        ownership.Release(computer);
        ownership.Release(computer);
        t.Check("attitude mode restored", computer.AttitudeMode == FlightComputerAttitudeMode.Manual);
        t.Check("attitude frame restored", computer.AttitudeFrame == VehicleReferenceFrame.EnuBody);
        t.Check("custom target restored", computer.CustomAttitudeTarget.Equals(default(double3)));
        t.Check("a field guidance never wrote is left alone", computer.AngleDeadband == 0.75f);
        t.Check("the burn plan survives the release", ReferenceEquals(computer.BurnPlan, plan));
    }

    private static void PlayerChangesArePreserved(TestContext t)
    {
        var computer = new FlightComputer();
        var ownership = new AttitudeOwnership();
        WriteGuidance(ownership, computer);
        computer.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.None;
        computer.RollMode = FlightComputerRollMode.Down;
        ownership.Release(computer);
        t.Check("a player tracking change survives", computer.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.None);
        t.Check("guidance angles do not become a body rate",
            computer.CustomAttitudeTarget.Equals(default(double3)));
        t.Check("a player roll change survives", computer.RollMode == FlightComputerRollMode.Down);
        t.Check("the player's tracking mode is left alone",
            computer.AttitudeMode == FlightComputerAttitudeMode.Auto);
    }

    private static void ReleaseDoesNotFollowAReplacementComputer(TestContext t)
    {
        var ownership = new AttitudeOwnership();
        WriteGuidance(ownership, new FlightComputer());
        var replacement = new FlightComputer { CustomAttitudeTarget = new double3(4, 5, 6) };
        ownership.Release(replacement);
        t.Check("release writes nothing into a replacement flight computer",
            replacement.CustomAttitudeTarget.Equals(new double3(4, 5, 6)));
    }

    private static void DisabledCraftIsUntouched(TestContext t, Vehicle vehicle, SimDriver driver, bool isolated)
    {
        FlightComputer computer = vehicle.FlightComputer;
        computer.CustomAttitudeTarget = new double3(0.2, 0.3, 0.4);
        TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: true);
        object plan = computer.BurnPlan;
        PlannedBurn? burn = SeedBurn(t, vehicle, driver);
        Program.ControlledVehicle = vehicle;

        var fields = typeof(FlightComputer).GetFields(BindingFlags.Public | BindingFlags.Instance)
            .ToDictionary(field => field, field => field.GetValue(computer));

        // Set the flag directly to test the disabled path without queuing cleanup.
        GuidanceWindow.ModActive = false;
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check("a disabled panel creates no state", !VehicleAutopilotState.TryGet(vehicle, out _));

        // Opening a panel creates state without granting control.
        VehicleAutopilotState panelState = VehicleAutopilotState.For(vehicle);
        panelState.AutoLaunch = true;
        if (isolated)
            GuidanceWindow.QueueAllReleases();
        else
            panelState.FcResetPending = true;
        GuidanceWindow.ApplyAutopilot(vehicle);
        GuidanceWindow.ApplyAutopilot(vehicle);

        t.Check("the launch preference survives", panelState.AutoLaunch);
        t.Check("the flight computer is not replaced", ReferenceEquals(vehicle.FlightComputer, computer));
        t.Check("the burn plan is not replaced", ReferenceEquals(computer.BurnPlan, plan));
        CheckPlanContents(t, computer, burn);
        bool changed = false;
        foreach (var field in fields)
            if (!Equals(field.Key.GetValue(computer), field.Value))
            {
                t.Fail("flight computer field", $"{field.Key.Name} changed on an untouched craft");
                changed = true;
            }
        t.Check("no public flight computer field changed", !changed);
        t.Check("the stored engine command survives",
            Inputs(vehicle).EngineOn && Inputs(vehicle).EngineThrottle == 0.63f);

        GuidanceWindow.ModActive = true;
        VehicleAutopilotState.Remove(vehicle);
    }

    private static void AcquiredCraftStopsWithoutChangingItsPlan(
        TestContext t, Vehicle vehicle, Vehicle other, SimDriver driver)
    {
        FlightComputer computer = vehicle.FlightComputer;
        PlannedBurn? burn = SeedBurn(t, vehicle, driver);
        TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: true);
        TestSupport.SetManualControlInputs(other, 0.63f, engineOn: true);
        // Engage=false requests release of a previously running mode.
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        state.Engage = false;
        state.ControlAcquired = true;
        state.Running = true;
        VehicleAutopilotState otherState = VehicleAutopilotState.For(other);
        otherState.Engage = false;
        otherState.ControlAcquired = true;
        otherState.Running = true;
        object plan = computer.BurnPlan;

        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check("the release finishes",
            !state.ControlAcquired && !state.Running && !state.FcResetPending);
        t.Check("the stored engine command is cleared", !Inputs(vehicle).EngineOn);
        t.Check("the player's throttle setting survives", Inputs(vehicle).EngineThrottle == 0.63f);
        t.Check("the burn plan is not replaced", ReferenceEquals(computer.BurnPlan, plan));
        CheckPlanContents(t, computer, burn);
        t.Check("another craft is untouched",
            otherState.ControlAcquired && otherState.Running && Inputs(other).EngineOn);

        ManualControlInputs released = Inputs(vehicle);
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check("a second release changes nothing", Inputs(vehicle).Equals(released));

        VehicleAutopilotState.Remove(vehicle);
        VehicleAutopilotState.Remove(other);
    }

    private static void RejectShutdown(Vehicle __instance)
    {
        if (ReferenceEquals(__instance, _failing))
            throw new InvalidOperationException("Injected shutdown failure");
    }

    private static void FailedReleaseRemainsPending(TestContext t, Vehicle vehicle, Vehicle other)
    {
        TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: true);
        TestSupport.SetManualControlInputs(other, 0.63f, engineOn: true);
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        state.Engage = false;
        state.ControlAcquired = true;
        VehicleAutopilotState otherState = VehicleAutopilotState.For(other);
        otherState.Engage = false;
        otherState.ControlAcquired = true;

        var harmony = new Harmony("com.maxi.afc.harnesstests.guidance.shutdown");
        harmony.Patch(AccessTools.Method(typeof(Vehicle), nameof(Vehicle.SetEnum), new[] { typeof(Enum) }),
            prefix: new HarmonyMethod(typeof(GuidanceHandbackTest), nameof(RejectShutdown)));
        try
        {
            _failing = vehicle;
            GuidanceWindow.ReleaseAllVehicles();
            t.Check("a failed release is not marked complete", state.ControlAcquired && state.FcResetPending);
            t.Check("the failure is visible", state.ReleaseError.Contains("Injected shutdown failure"));
            t.Check("one failure does not stop another craft's cleanup", !Inputs(other).EngineOn);
            t.Check("a released craft loses its state", !VehicleAutopilotState.TryGet(other, out _));

            // Terminal cleanup discards state because no later step can retry.
            // The rate-cleanup case tests retries on a live vehicle.
            t.Check("a terminal path does not keep a craft it cannot retry",
                !VehicleAutopilotState.TryGet(vehicle, out _));
        }
        finally
        {
            _failing = null;
            harmony.UnpatchAll(harmony.Id);
            VehicleAutopilotState.Remove(vehicle);
            VehicleAutopilotState.Remove(other);
        }
    }

    private static void RejectRateClear(Vehicle __0)
    {
        if (ReferenceEquals(__0, _failing))
            throw new InvalidOperationException("Injected rate cleanup failure");
    }

    // Tests the release flag directly, without running the 6-DOF handover.
    private static void AHandoverReleaseKeepsTheEngineLit(
        TestContext t, Vehicle vehicle, SimDriver driver)
    {
        SeedBurn(t, vehicle, driver);
        TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: true);
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        state.Engage = false;
        state.ControlAcquired = true;
        state.Running = true;
        state.ReleaseWithoutEngineCut = true;

        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check("a handover release finishes",
            !state.ControlAcquired && !state.Running && !state.FcResetPending);
        t.Check("a handover release preserves the engine-on input", Inputs(vehicle).EngineOn);
        t.Check("the finished release restores the default stop",
            !state.ReleaseWithoutEngineCut);

        // A later ordinary stop must not inherit the no-cut request.
        TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: true);
        state.Engage = false;
        state.ControlAcquired = true;
        state.Running = true;
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check("an ordinary stop still cuts the engine", !Inputs(vehicle).EngineOn);

        VehicleAutopilotState.Remove(vehicle);
    }

    private static void FailedRateCleanupDoesNotBlockAttitudeRelease(TestContext t, Vehicle vehicle)
    {
        FlightComputer computer = vehicle.FlightComputer;
        computer.AttitudeFrame = VehicleReferenceFrame.EnuBody;
        computer.AttitudeMode = FlightComputerAttitudeMode.Manual;
        computer.CustomAttitudeTarget = default;
        computer.RollMode = FlightComputerRollMode.Decoupled;
        TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: true);

        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        state.Engage = false;
        state.ControlAcquired = true;
        WriteGuidance(state.AttitudeOwnership, computer);

        var harmony = new Harmony("com.maxi.afc.harnesstests.guidance.rate");
        harmony.Patch(AccessTools.Method(typeof(KsaAttitudeRate), nameof(KsaAttitudeRate.Clear)),
            prefix: new HarmonyMethod(typeof(GuidanceHandbackTest), nameof(RejectRateClear)));
        try
        {
            _failing = vehicle;
            GuidanceWindow.ApplyAutopilot(vehicle);
            t.Check("a failed rate cleanup keeps its retry", state.FcResetPending && state.ControlAcquired);
            t.Check("the rate failure is visible",
                state.ReleaseError.Contains("Injected rate cleanup failure"));
            t.Check("the engine still stops", !Inputs(vehicle).EngineOn);
            t.Check("the attitude is restored independently",
                computer.AttitudeMode == FlightComputerAttitudeMode.Manual
                && computer.AttitudeFrame == VehicleReferenceFrame.EnuBody
                && computer.CustomAttitudeTarget.Equals(default(double3)));

            _failing = null;
            GuidanceWindow.ApplyAutopilot(vehicle);
            t.Check("the retry completes", !state.FcResetPending && !state.ControlAcquired);
        }
        finally
        {
            _failing = null;
            harmony.UnpatchAll(harmony.Id);
            VehicleAutopilotState.Remove(vehicle);
        }
    }
}

using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Installs the production guidance hooks on a test-scoped Harmony id and steps the universe, so the
// PrepareWorker prefix, the sink dispatch and the worker postfix run the way the game runs them.
// The ascent step itself is replaced, because the driver plumbing is under test, not the flight.
public sealed class GuidanceDriverTest : AfcTest
{
    public override string Name => "afc-guidance-driver";

    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> Inputs =
        AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(
            AccessTools.Field(typeof(GuidanceWindow), "_s"));

    private static readonly double3 CommandRate = new(0.01, 0.0, 0.0);
    private const double RateTolerance = 1e-9;

    private static Vehicle? _held;
    private static Vehicle? _faulting;

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
        if (VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state.");
            return;
        }

        Vehicle? previousFocus = Program.ControlledVehicle;
        VehicleAutopilotState previousAmbient = AmbientState();
        bool previousModActive = GuidanceWindow.ModActive;
        bool previousEnabled = SharedVehicleHooks.GuidanceEnabled;
        bool previousOffRails = PhysicsBubble._forceOffRails;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.driver");
        try
        {
            VehicleCommandSink.ApplyPatches(harmony);
            GuidanceFeature.ApplyDriverPatches(harmony);
            harmony.Patch(AccessTools.Method(typeof(GuidanceWindow), "StepAscent"),
                prefix: new HarmonyMethod(typeof(GuidanceDriverTest), nameof(ReplaceTheAscentStep)));
            SharedVehicleHooks.GuidanceEnabled = true;
            GuidanceWindow.ModActive = true;

            Orbit orbit = OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed);
            Vehicle vehicle;
            try
            {
                vehicle = VehicleSpawner.SpawnFromSave(saves[0], t.System, home, "HarnessGuidanceDriver", orbit);
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"'{saves[0]}': {e.Message}");
                return;
            }
            _held = vehicle;

            // The step services the focused craft, and the sink only runs for a craft in full physics.
            // The settling steps give the spawn a flight plan, which the burn fixture needs.
            Program.ControlledVehicle = vehicle;
            PhysicsBubble._forceOffRails = true;
            driver.Step(0.05, 40);

            TheDriverFliesARunningMode(t, vehicle, driver);
            StockAutoIsHeldAndGivenBack(t, vehicle, driver);
            ArmingAutoTakesTheEngineBack(t, vehicle, driver);
            AReplacedBurnKeepsManual(t, vehicle, driver);
            ANoCutReleaseKeepsManual(t, vehicle, driver);
            TheGimbalWriterRunsThroughTheSink(t, vehicle, driver);
            SwitchingGuidanceOffReleasesTheCraft(t, vehicle, driver);
            AFailedStepReleasesTheCraft(t, vehicle, driver);
        }
        finally
        {
            _held = null;
            _faulting = null;
            PhysicsBubble._forceOffRails = previousOffRails;
            harmony.UnpatchAll(harmony.Id);
            // Releases this test's craft before it despawns. No other craft holds state, see above.
            GuidanceFeature.DisableDriver();
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.ModActive = previousModActive;
            Program.ControlledVehicle = previousFocus;
            AmbientState() = previousAmbient;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void TheDriverFliesARunningMode(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        FlightComputerAttitudeTrackTarget targetBefore = fc.AttitudeTrackTarget;
        VehicleReferenceFrame frameBefore = fc.AttitudeFrame;
        double3 ratesBefore = fc.AttitudeTarget.RatesCci;
        VehicleAutopilotState state = Running(vehicle);
        try
        {
            driver.Step(0.05, 3);
            t.Check("the prefix takes the claim for a running mode",
                VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.Guidance && state.ControlAcquired);
            t.Check("the attitude command reaches the flight computer",
                fc.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Custom
                && fc.AttitudeFrame == VehicleReferenceFrame.EclBody
                && fc.AttitudeMode == FlightComputerAttitudeMode.Auto);
            t.Check("the engine command survives stock's own input pass",
                Inputs(vehicle).EngineOn && Inputs(vehicle).EngineThrottle == 1f,
                $"on={Inputs(vehicle).EngineOn} throttle={Inputs(vehicle).EngineThrottle}");

            // EclBody is inertial, so the worker's target rate is the published rate alone.
            double3 rates = fc.AttitudeTarget.RatesCci;
            t.Check("the commanded turning rate reaches the worker",
                (rates - CommandRate).Length() < RateTolerance, $"rates={rates}");

            state.Engage = false;
            driver.Step(0.05, 1);
            t.Check("ending the mode releases the claim in the same step",
                VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.None && !state.ControlAcquired);
            t.Check("the release cuts the engine", !Inputs(vehicle).EngineOn);
            t.Check("the release hands the attitude back",
                fc.AttitudeTrackTarget == targetBefore && fc.AttitudeFrame == frameBefore,
                $"target={fc.AttitudeTrackTarget} frame={fc.AttitudeFrame}");

            driver.Step(0.05, 1);
            t.Check("the cleared rate no longer reaches the worker",
                (fc.AttitudeTarget.RatesCci - ratesBefore).Length() < RateTolerance,
                $"rates={fc.AttitudeTarget.RatesCci} before={ratesBefore}");
        }
        finally
        {
            VehicleAutopilotState.Remove(vehicle);
        }
    }

    // Vehicle.PrepareWorker clears the engine switch on every step while the burn mode is Auto.
    private static void StockAutoIsHeldAndGivenBack(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        VehicleAutopilotState? state = HoldFromAuto(t, vehicle, driver, "hold and give back");
        if (state == null)
            return;
        try
        {
            state.Engage = false;
            driver.Step(0.05, 1);
            t.Check("a release that cuts the engine gives Auto back",
                fc.BurnMode == FlightComputerBurnMode.Auto && !state.ForcedBurnManual);
        }
        finally
        {
            EndHold(vehicle);
        }
    }

    // Stock never writes Auto by itself, so an Auto that appears during a hold is the player or
    // another mod arming the stock autopilot, which owns the engine from then on.
    private static void ArmingAutoTakesTheEngineBack(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        VehicleAutopilotState? state = HoldFromAuto(t, vehicle, driver, "auto takeover");
        if (state == null)
            return;
        try
        {
            fc.BurnMode = FlightComputerBurnMode.Auto;
            driver.Step(0.05, 1);
            t.Check("arming stock Auto during a hold stops guidance",
                VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.None
                && !state.ControlAcquired && !state.Running);
            t.Check("the engine takeover leaves Auto armed",
                fc.BurnMode == FlightComputerBurnMode.Auto && !state.ForcedBurnManual);
            t.Check("the engine takeover says why", state.Status.Contains("engine"), state.Status);
            // Guidance made no cut. Stock's own Auto handling switched the engine off before ignition.
            t.Check("stock owns the engine after the takeover", !Inputs(vehicle).EngineOn);
        }
        finally
        {
            EndHold(vehicle);
        }
    }

    // Stock writes Manual itself when the loaded burn goes away, so a release must not read that
    // Manual as its own and arm Auto on whatever burn is loaded next.
    private static void AReplacedBurnKeepsManual(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        VehicleAutopilotState? state = HoldFromAuto(t, vehicle, driver, "replaced burn");
        if (state == null)
            return;
        try
        {
            RcsFlightSupport.CleanupBurns(fc);
            state.Engage = false;
            driver.Step(0.05, 1);
            t.Check("a release after the burn changed hands leaves Manual",
                fc.BurnMode == FlightComputerBurnMode.Manual && !state.ForcedBurnManual
                && VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.None);
        }
        finally
        {
            EndHold(vehicle);
        }
    }

    // A restored Auto would have stock switch the engine off on the same step, which is the cut
    // the no-cut release exists to avoid.
    private static void ANoCutReleaseKeepsManual(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        FlightComputer fc = vehicle.FlightComputer;
        VehicleAutopilotState? state = HoldFromAuto(t, vehicle, driver, "no-cut release");
        if (state == null)
            return;
        try
        {
            vehicle.SetEnum(FlightComputerAttitudeTrackTarget.Prograde);
            driver.Step(0.05, 1);
            t.Check("an attitude takeover during a hold stops guidance without a cut",
                VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.None && Inputs(vehicle).EngineOn);
            t.Check("a no-cut release leaves the burn mode in Manual",
                fc.BurnMode == FlightComputerBurnMode.Manual && !state.ForcedBurnManual);
        }
        finally
        {
            EndHold(vehicle);
        }
    }

    // A hold that started from stock Auto. A burn far from ignition keeps stock from dropping Auto
    // by itself, and the first step's acquisition replaces it with Manual.
    private static VehicleAutopilotState? HoldFromAuto(
        TestContext t, Vehicle vehicle, SimDriver driver, string label)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitX, 1.0, 600.0) == null)
        {
            t.Fail($"{label} setup", "no patch or loaded burn target");
            return null;
        }
        VehicleAutopilotState state = Running(vehicle);
        fc.BurnMode = FlightComputerBurnMode.Auto;
        driver.Step(0.05, 2);
        if (!t.Check($"{label}: acquisition holds stock's burn mode in Manual",
                fc.BurnMode == FlightComputerBurnMode.Manual && state.ForcedBurnManual
                && Inputs(vehicle).EngineOn))
        {
            EndHold(vehicle);
            return null;
        }
        return state;
    }

    private static void EndHold(Vehicle vehicle)
    {
        vehicle.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
        RcsFlightSupport.CleanupBurns(vehicle.FlightComputer);
        VehicleControlOwnership.ReleaseAll(vehicle);
        VehicleAutopilotState.Remove(vehicle);
    }

    private static void TheGimbalWriterRunsThroughTheSink(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        if (vehicle.FlightComputer.VehicleConfig.Gimbals.Count == 0)
        {
            t.Skip("the save has no gimbals, so the gimbal writer has nothing to drive.");
            return;
        }
        if (!t.Check("the fixture holds the claim",
                VehicleControlOwnership.TryClaim(vehicle, ControlClaimant.Guidance, out _)))
            return;
        try
        {
            KsaGimbalControl.SetDirect(vehicle, 0.25f, 0f);
            driver.Step(0.05, 2);
            KsaGimbalControl.Slot? slot = KsaGimbalControl.Diagnostics(vehicle);
            if (!t.Check("the gimbal override runs on the worker through the sink",
                    slot != null && slot.AppliedCount > 0, $"applied={slot?.AppliedCount}"))
                return;

            SharedVehicleHooks.GuidanceEnabled = false;
            slot!.AppliedCount = 0;
            driver.Step(0.05, 2);
            t.Check("the sink gates the writer on the feature flag", slot.AppliedCount == 0);
        }
        finally
        {
            SharedVehicleHooks.GuidanceEnabled = true;
            KsaGimbalControl.Disengage(vehicle);
            VehicleControlOwnership.Release(vehicle, ControlClaimant.Guidance);
        }
    }

    private static void SwitchingGuidanceOffReleasesTheCraft(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        VehicleAutopilotState state = Running(vehicle);
        try
        {
            driver.Step(0.05, 2);
            if (!t.Check("the mode runs before the switch", state.ControlAcquired && Inputs(vehicle).EngineOn))
                return;

            GuidanceWindow.SetModActive(false);
            driver.Step(0.05, 1);
            t.Check("switching guidance off releases the craft on its next step",
                VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.None
                && !state.ControlAcquired && !state.FcResetPending);
            t.Check("the switched-off release cuts the engine", !Inputs(vehicle).EngineOn);
        }
        finally
        {
            GuidanceWindow.SetModActive(true);
            VehicleAutopilotState.Remove(vehicle);
        }
    }

    private static void AFailedStepReleasesTheCraft(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        VehicleAutopilotState state = Running(vehicle);
        TestSupport.SetManualControlInputs(vehicle, 0.5f, engineOn: true);
        _faulting = vehicle;
        try
        {
            driver.Step(0.05, 1);
            t.Check("a failed step releases the craft",
                VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.None
                && !state.ControlAcquired && !state.Running);
            t.Check("a failed step reports why",
                state.Error.StartsWith("Guidance stopped", StringComparison.Ordinal), state.Error);
            t.Check("a failed step cuts the engine", !Inputs(vehicle).EngineOn);
        }
        finally
        {
            _faulting = null;
            VehicleAutopilotState.Remove(vehicle);
        }
    }

    // A running ascent with a command the replaced step leaves alone, so what reaches the craft is
    // the driver's own work.
    private static VehicleAutopilotState Running(Vehicle vehicle)
    {
        TestSupport.SetManualControlInputs(vehicle, 0f, engineOn: false);
        vehicle.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        state.Engage = true;
        state.Running = true;
        state.AutoStage = true;
        state.HasCommand = true;
        state.CommandDir = double3.Normalize(vehicle.Orbit.StateVectors.PositionCci);
        state.CommandRate = CommandRate;
        return state;
    }

    // A Harmony prefix that returns false skips the original, so the fixture's ascent step is held
    // and every other craft's runs as usual.
    private static bool ReplaceTheAscentStep(Vehicle vehicle)
    {
        if (ReferenceEquals(vehicle, _faulting))
            throw new InvalidOperationException("Injected guidance step fault");
        return !ReferenceEquals(vehicle, _held);
    }
}

using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// A craft that guidance flies loses control the way the game takes it away: its crew leaves the pod and every probe core is switched off, so Vehicle.UpdateControl gives it no control on the next step and hands Attitude and Burn back to Manual.
// Guidance must then stop with the engine cut and the game's reason on the status line, keep the attitude in Manual, and refuse a mode engaged on the dead craft without touching its engine.
// ApplyAutopilot is called directly after each step, as in afc-guidance-player-takeover, so each case controls one step.
public sealed class GuidanceControlLossTest : AfcTest
{
    public override string Name => "afc-guidance-control-loss";

    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> Inputs =
        AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(AccessTools.Field(typeof(GuidanceWindow), "_s"));

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
        if (VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state.");
            return;
        }

        VehicleAutopilotState previousAmbient = AmbientState();
        bool previousModActive = GuidanceWindow.ModActive;
        Vehicle? vehicle = null;
        try
        {
            GuidanceWindow.SetModActive(true);
            SimDriver driver = t.Session.CreateDriver();
            vehicle = VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance,
                "AfcGuidanceControlLoss", OrbitFixtures.CircularAt(home, 500_000, driver.Elapsed));
            driver.Step(0.02, 2);
            if (!t.Check("the crewed fixture is controllable", vehicle.IsControllable, GuidanceWindow.ControlLostReason(vehicle, "CONTROL")))
                return;

            VehicleAutopilotState state = Engaged(vehicle);
            vehicle.EndAllCrewMissions();
            int switchedOff = SwitchOffProbeCores(vehicle);
            driver.Step(0.02, 1);
            if (vehicle.IsControllable)
            {
                t.Skip($"the Rocket stays controllable without crew and with {switchedOff} probe core(s) switched off.");
                return;
            }
            t.Info($"control lost: {GuidanceWindow.ControlLostReason(vehicle, "CONTROL")} ({switchedOff} probe core(s) switched off)");

            GuidanceWindow.ApplyAutopilot(vehicle);
            t.Check("guidance stops", !state.ControlAcquired && !state.Running,
                $"acquired {state.ControlAcquired}, running {state.Running}");
            t.Check("guidance lets go of the claim", VehicleControlOwnership.HolderOf(vehicle) != ControlClaimant.Guidance);
            t.Check("the engine is cut", !Inputs(vehicle).EngineOn);
            t.Check("the reason is visible", state.Status.Contains("OUT OF CONTROL"), state.Status);
            t.Check("the attitude stays out of Auto", vehicle.FlightComputer.AttitudeMode != FlightComputerAttitudeMode.Auto,
                vehicle.FlightComputer.AttitudeMode.ToString());
            VehicleAutopilotState.Remove(vehicle);

            // A mode engaged on the dead craft: guidance never held it, so it refuses and leaves the engine as it is.
            TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: true);
            VehicleAutopilotState refused = VehicleAutopilotState.For(vehicle);
            refused.Engage = true;
            refused.Running = true;
            GuidanceWindow.ApplyAutopilot(vehicle);
            t.Check("a new mode on the dead craft is refused", !refused.Running && !refused.ControlAcquired
                && VehicleControlOwnership.HolderOf(vehicle) != ControlClaimant.Guidance);
            t.Check("the refusal is visible", refused.Status.Contains("REFUSED"), refused.Status);
            t.Check("the refusal leaves the engine as it was", Inputs(vehicle).EngineOn);
        }
        finally
        {
            if (vehicle != null)
            {
                VehicleControlOwnership.ReleaseAll(vehicle);
                VehicleAutopilotState.Remove(vehicle);
                VehicleSpawner.Despawn(vehicle);
            }
            GuidanceWindow.SetModActive(previousModActive);
            AmbientState() = previousAmbient;
        }
    }

    // A running ascent that holds the claim, the engine lit and the attitude, the same fixture the takeover test flies.
    private static VehicleAutopilotState Engaged(Vehicle vehicle)
    {
        TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: true);
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        state.Engage = true;
        state.Running = true;
        state.ControlAcquired = true;
        VehicleControlOwnership.TryClaim(vehicle, ControlClaimant.Guidance, out _);

        FlightComputer computer = vehicle.FlightComputer;
        state.AttitudeOwnership.BeginWrite(computer);
        computer.CustomAttitudeTarget = new double3(1, 2, 3);
        computer.AttitudeFrame = VehicleReferenceFrame.EclBody;
        computer.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
        computer.AttitudeMode = FlightComputerAttitudeMode.Auto;
        state.AttitudeOwnership.EndWrite(computer, writesRoll: false);
        return state;
    }

    // The power switch the game reads for a probe core outside every habitat zone (Vehicle.FindProbeConsumer).
    private static int SwitchOffProbeCores(Vehicle vehicle)
    {
        int count = 0;
        foreach (Control control in vehicle.Parts.Modules.Get<Control>())
        {
            foreach (PowerConsumer consumer in control.Parent.Modules.Get<PowerConsumer>())
            {
                if (!consumer.PowerSwitch)
                    continue;
                consumer.Mode = PowerSwitchMode.Off;
                count++;
                break;
            }
        }
        return count;
    }
}

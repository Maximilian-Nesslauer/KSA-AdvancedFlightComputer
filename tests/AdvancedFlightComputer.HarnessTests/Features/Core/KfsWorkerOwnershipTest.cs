using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;
using KSA.Kfs;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class KfsWorkerOwnershipTest : AfcTest
{
    public override string Name => "afc-kfs-worker-ownership";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        var save = DefaultVehicleSaves.FindSave("Rocket");
        if (save?.VehicleSaveData.RootPartInstance == null)
        {
            t.Fail("fixture", "Rocket is not available");
            return;
        }
        bool kfs = KfsPrototype.Enabled;
        bool modActive = GuidanceWindow.ModActive;
        Vehicle? vehicle = null;
        try
        {
            GuidanceWindow.SetModActive(true);
            var driver = t.Session.CreateDriver();
            vehicle = VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance,
                "AfcKfsWorker", OrbitFixtures.CircularAt(home, 500_000, driver.Elapsed));
            driver.Step(0.02, 2);
            vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
            driver.Step(0.02, 3);
            if (!t.Check("the worker fixture has active gimbals", vehicle.FlightComputer.VehicleConfig.Gimbals.Count > 0))
                return;

            // The main-thread tick has already published these commands when the workbench queues its start.
            KsaGimbalControl.SetDirect(vehicle, 0.81f, -0.73f);
            KsaAttitudeRate.Set(vehicle, new double3(0.12, 0.23, 0.34));
            var rcs = new RcsWorkerCommand { Active = true, Owner = vehicle };
            RcsCommandChannel.Publish(vehicle.FlightComputer.BurnPlan, rcs);
            KfsPrototype.Enabled = true;
            var host = KfsFixtures.QueuePilot(vehicle);
            t.Check("the queued start is invisible to the main-thread gate", !VehicleControlOwnership.KfsOwns(vehicle));
            var dispatch = KfsFixtures.DispatchPilot(host);
            t.Check("dispatch drains a real queued pilot start", dispatch.UnderControl && dispatch.Pilot
                && host.Computer.UnderKfsControl && host.Computer.Image != null);

            var worker = new FlightComputer(vehicle.FlightComputer);
            worker.Burn = new BurnTarget
            {
                BurnDuration = 9.25f,
                IgnitionTime = Universe.GetElapsedTime() + 15.0,
                DeltaVTargetCci = float3.UnitX,
            };
            UniverseTime ignition = worker.Burn.IgnitionTime;
            using var updates = new ModuleStateUpdaters(vehicle.Parts.States);
            var engines = ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>.InitializeHotPathUpdater(updates);
            var gimbals = ModuleStateful<GimbalController, GimbalControllerState, EmptyStruct, EmptyStruct>.InitializeHotPathUpdater(updates);
            var thrusters = ModuleStateful<ThrusterController, ThrusterControllerState, ThrusterControllerGlobalState, EmptyStruct>.InitializeHotPathUpdater(updates);
            var output = new FlightComputerOutput(gimbals, thrusters, engines);
            t.Check("engine commands are present for the overwrite check", engines.GetReadOnlyStates().Length > 0);
            foreach (var engine in vehicle.Parts.Modules.Get<EngineController>())
            {
                ref var command = ref engines.GetNewState(engine);
                command.CommandThrottle = 0.61f;
                command.CommandBurnTime = 7.25;
            }
            foreach (var gimbal in worker.VehicleConfig.Gimbals)
            {
                ref var command = ref gimbals.GetNewState(gimbal);
                command.CommandY = 0.31f;
                command.CommandZ = -0.22f;
                command.NaturalFrequencyLimit = 3.5f;
            }
            output.EngineThrottle = 0.61f;
            output.EngineBurnDuration = 7.25;
            output.NextWakeupDeltaTime = 0.7;
            output.NextWakeupSource = PhysicsStepLimit.KfsTick;
            worker.AttitudeTarget.RatesCci = new double3(0.7, 0.8, 0.9);
            UniverseTime lastThrust = worker.LastThrustTime;
            var receipt = new VehicleCommandSink.Receipt();
            FlightComputerNavigation nav = default;
            RcsComputeControlPatch.Command(worker, in nav, ref output, ref receipt);
            KsaGimbalControl.OnComputeControl(worker, ref output, ref receipt);
            KsaAttitudeRate.OnUpdateAttitudeTarget(worker);

            t.Check("kFS engine commands survive stale RCS payloads", engines.GetReadOnlyStates().ToArray().All(
                state => state.CommandThrottle == 0.61f && state.CommandBurnTime == 7.25));
            t.Check("kFS gimbal commands and limits survive", worker.VehicleConfig.Gimbals.All(gimbal =>
                gimbals.GetReadOnlyState(gimbal).CommandY == 0.31f && gimbals.GetReadOnlyState(gimbal).CommandZ == -0.22f
                && gimbals.GetReadOnlyState(gimbal).NaturalFrequencyLimit == 3.5f));
            t.Check("kFS attitude rate survives stale feedforward", worker.AttitudeTarget.RatesCci.Equals(new double3(0.7, 0.8, 0.9)));
            t.Check("no AFC consumption, wake or timing write occurs", !rcs.WasConsumed && !receipt.Commanded
                && double.IsPositiveInfinity(receipt.WakeupSec) && output.NextWakeupDeltaTime == 0.7
                && output.NextWakeupSource == PhysicsStepLimit.KfsTick && output.EngineThrottle == 0.61f
                && output.EngineBurnDuration == 7.25 && worker.LastThrustTime == lastThrust
                && worker.Burn.IgnitionTime == ignition && worker.Burn.BurnDuration == 9.25f);
        }
        finally
        {
            if (vehicle != null)
            {
                vehicle.KfsHost?.Computer.Stop();
                vehicle.KfsHost = null;
                KsaGimbalControl.Disengage(vehicle);
                KsaAttitudeRate.Clear(vehicle);
                RcsCommandChannel.Clear(vehicle.FlightComputer.BurnPlan);
                VehicleSpawner.Despawn(vehicle);
            }
            KfsPrototype.Enabled = kfs;
            GuidanceWindow.SetModActive(modActive);
        }
    }
}

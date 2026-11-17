using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;
using KSA.Kfs;
using KSA.Kfs.Host;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class GuidancePendingControlTest : AfcTest
{
    public override string Name => "afc-guidance-pending-control";

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> Ambient =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(AccessTools.Field(typeof(GuidanceWindow), "_s"));
    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> Inputs =
        AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

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
        VehicleAutopilotState previous = Ambient();
        bool active = GuidanceWindow.ModActive;
        bool kfsEnabled = KfsPrototype.Enabled;
        Vehicle? vehicle = null;
        try
        {
            GuidanceWindow.SetModActive(true);
            var driver = t.Session.CreateDriver();
            vehicle = VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance,
                "AfcPendingControl", OrbitFixtures.CircularAt(home, 500_000, driver.Elapsed));
            driver.Step(0.02, 2);
            vehicle.EndAllCrewMissions();
            foreach (Control control in vehicle.Parts.Modules.Get<Control>())
                foreach (PowerConsumer consumer in control.Parent.Modules.Get<PowerConsumer>())
                    consumer.Mode = PowerSwitchMode.Off;
            driver.Step(0.02);
            if (!t.Check("the fixture lost control", !vehicle.IsControllable))
                return;

            CheckLaunchRefusals(t, vehicle, home);
            CheckPending(t, vehicle);
            CheckRcs(t, vehicle, loaded: false);
            CheckRcs(t, vehicle, loaded: true);

            VehicleAutopilotState stopped = State(vehicle);
            stopped.ConvexLaunchPending = true;
            stopped.AscentPlanRequested = true;
            stopped.ThreeDofEngagePending = true;
            stopped.BoostbackEngagePending = true;
            GuidanceWindow.ApplyAutopilot(vehicle);
            VehicleSpawner.SeatRandomCrew(vehicle);
            driver.Step(0.02, 2);
            t.Check("crew restores control", vehicle.IsControllable);
            GuidanceWindow.ApplyAutopilot(vehicle);
            RcsExecutor.Tick(vehicle);
            t.Check("recovery restarts no pending mode", !stopped.Running && !stopped.LaunchArmed
                && !stopped.ConvexLaunchPending && !stopped.AscentPlanRequested && stopped.AscentPlanJob == null
                && !stopped.ThreeDofEngagePending && !stopped.BoostbackEngagePending
                && stopped.ThreeDofPhase == GuidanceWindow.ThreeDofPhase.Idle && stopped.BoostbackPhase == GuidanceWindow.BoostbackPhase.Idle);
            t.Check("recovery restarts no RCS execution", !RcsExecutor.IsActive(vehicle)
                && !RcsCommandChannel.TryGet(vehicle.FlightComputer.BurnPlan, out _));
            CheckKfs(t, vehicle, home);
        }
        finally
        {
            KfsPrototype.Enabled = kfsEnabled;
            if (vehicle != null)
            {
                vehicle.KfsHost = null;
                RcsExecRegistry.Remove(vehicle.Id);
                RcsCommandChannel.Clear(vehicle.FlightComputer.BurnPlan);
                VehicleControlOwnership.ReleaseAll(vehicle);
                VehicleAutopilotState.Remove(vehicle);
                VehicleSpawner.Despawn(vehicle);
            }
            GuidanceWindow.SetModActive(active);
            Ambient() = previous;
        }
    }

    private static VehicleAutopilotState State(Vehicle vehicle)
    {
        VehicleAutopilotState.Remove(vehicle);
        var state = VehicleAutopilotState.For(vehicle);
        Ambient() = state;
        return state;
    }

    private static void CheckLaunchRefusals(TestContext t, Vehicle vehicle, IParentBody home)
    {
        foreach (string entry in new[] { "ExecuteAscent", "LaunchBackupAscent", "StartGuidance", "LaunchAscent" })
        {
            foreach (bool lit in new[] { false, true })
            {
                VehicleAutopilotState state = State(vehicle);
                TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: lit);
                object[] args = entry == "LaunchAscent"
                    ? new object[] { vehicle, vehicle.Orbit, home, true }
                    : new object[] { vehicle, vehicle.Orbit, home };
                AccessTools.Method(typeof(GuidanceWindow), entry).Invoke(null, args);
                t.Check($"{entry} refuses before launch, engine {lit}", !state.Running && !state.LaunchArmed
                    && !state.ConvexLaunchPending && !state.AscentPlanRequested && state.AscentPlanJob == null
                    && state.Status.Contains("OUT OF CONTROL"), state.Status);
                t.Check($"{entry} preserves the manual engine, engine {lit}", Inputs(vehicle).EngineOn == lit
                    && Inputs(vehicle).EngineThrottle == 0.63f);
            }
        }
    }

    private static void CheckPending(TestContext t, Vehicle vehicle)
    {
        foreach (string pending in new[] { "ConvexLaunchPending", "ThreeDofEngagePending", "BoostbackEngagePending", "LaunchArmed" })
        {
            VehicleAutopilotState state = State(vehicle);
            AccessTools.Field(typeof(VehicleAutopilotState), pending).SetValue(state, true);
            state.Engage = true;
            state.AutoStage = true;
            using var job = EmptyJob();
            if (state.ConvexLaunchPending)
            {
                state.AscentPlanRequested = true;
                state.AscentPlanJob = job;
            }
            TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: true);
            t.Check($"{pending} counts as releasable", GuidanceWindow.HoldsReleasableState(state));
            GuidanceWindow.ApplyAutopilot(vehicle);
            t.Check($"{pending} is cancelled before dispatch", !(bool)AccessTools.Field(typeof(VehicleAutopilotState), pending).GetValue(state)!
                && !state.Running && !state.Active && !state.ControlAcquired);
            t.Check($"{pending} keeps the unclaimed engine", Inputs(vehicle).EngineOn);
            if (pending == "ConvexLaunchPending")
                t.Check("the executable launch job is detached and cancelled", state.AscentPlanJob == null
                    && !state.AscentPlanRequested && Cancelled(job));
        }

        VehicleAutopilotState readOnly = State(vehicle);
        using var preview = EmptyJob();
        readOnly.AscentPlanJob = preview;
        readOnly.AscentPlanRequested = true;
        readOnly.ThreeDofEngagePending = true;
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check("release preserves a read-only plan job", ReferenceEquals(readOnly.AscentPlanJob, preview)
            && readOnly.AscentPlanRequested && !Cancelled(preview));
        readOnly.AscentPlanJob = null;
        readOnly.AscentPlanRequested = false;

        VehicleAutopilotState acquired = State(vehicle);
        acquired.ControlAcquired = true;
        acquired.ConvexLaunchPending = true;
        acquired.AscentPlanRequested = true;
        TestSupport.SetManualControlInputs(vehicle, 1.0f, engineOn: true);
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check("a late refusal cuts the acquired engine", !Inputs(vehicle).EngineOn
            && !acquired.ControlAcquired && !acquired.ConvexLaunchPending);
    }

    private static AscentPlanJob EmptyJob() => new((_, _, _) => throw new InvalidOperationException("No solve required."),
        1, 1, 1000, (_, _, _, _, _, _) => throw new InvalidOperationException("No plan required."));

    private static bool Cancelled(AscentPlanJob job)
        => ((CancellationTokenSource)AccessTools.Field(typeof(AscentPlanJob), "_cts").GetValue(job)!).IsCancellationRequested;

    private static void CheckRcs(TestContext t, Vehicle vehicle, bool loaded)
    {
        FlightComputer fc = vehicle.FlightComputer;
        var exec = RcsExecRegistry.GetOrCreate(vehicle.Id);
        exec.ActiveBurnTimeSec = 10;
        exec.ActiveBurnDvMs = 1;
        exec.AlignCommanded = true;
        exec.ForcedAttitudeAuto = true;
        exec.ForcedBurnManual = true;
        exec.ReconciledAfterLoad = !loaded;
        fc.AttitudeMode = FlightComputerAttitudeMode.Manual;
        fc.BurnMode = FlightComputerBurnMode.Manual;
        fc.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Custom;
        fc.CustomAttitudeTarget = new double3(1, 2, 3);
        RcsCommandChannel.Publish(fc.BurnPlan, new RcsWorkerCommand { Active = true, Owner = vehicle });
        RcsExecutor.Tick(vehicle);
        t.Check($"RCS cancels before reconciliation, loaded {loaded}", !exec.IsActive
            && !RcsCommandChannel.TryGet(fc.BurnPlan, out _));
        t.Check($"RCS restores no Auto attitude or burn, loaded {loaded}", fc.AttitudeMode == FlightComputerAttitudeMode.Manual
            && fc.BurnMode == FlightComputerBurnMode.Manual && fc.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Custom
            && fc.CustomAttitudeTarget.Equals(new double3(1, 2, 3)));
    }

    private static void CheckKfs(TestContext t, Vehicle vehicle, IParentBody home)
    {
        KfsPrototype.Enabled = true;
        t.Check("no host means no kFS ownership", !VehicleControlOwnership.KfsOwns(vehicle));
        var host = KfsVehicleHost.For(vehicle);
        t.Check("an idle host owns nothing", !VehicleControlOwnership.KfsOwns(vehicle));
        // Halted retains ownership, even when a program no longer runs ticks.
        AccessTools.PropertySetter(typeof(HostComputer), "State").Invoke(host.Computer, new object[] { ComputerState.Halted });
        t.Check("a halted host still owns the craft", VehicleControlOwnership.KfsOwns(vehicle));
        KfsPrototype.Enabled = false;
        t.Check("the prototype switch gates ownership", !VehicleControlOwnership.KfsOwns(vehicle));
        KfsPrototype.Enabled = true;

        var state = State(vehicle);
        state.ControlAcquired = true;
        state.ConvexLaunchPending = true;
        state.AscentPlanRequested = true;
        state.ThreeDofEngagePending = true;
        state.BoostbackEngagePending = true;
        state.RcsHeldOff = true;
        var fc = vehicle.FlightComputer;
        state.AttitudeOwnership.BeginWrite(fc);
        fc.CustomAttitudeTarget = new double3(3, 2, 1);
        fc.AttitudeMode = FlightComputerAttitudeMode.Auto;
        state.AttitudeOwnership.EndWrite(fc, writesRoll: true);
        fc.RCSMode = FlightComputerRCSMode.Disabled;
        TestSupport.SetManualControlInputs(vehicle, 0.72f, engineOn: true);
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check("kFS cancels pending autonomy", !state.ControlAcquired && !state.ConvexLaunchPending
            && !state.AscentPlanRequested && !state.ThreeDofEngagePending && !state.BoostbackEngagePending);
        t.Check("kFS keeps engine and attitude commands", Inputs(vehicle).EngineOn && Inputs(vehicle).EngineThrottle == 0.72f
            && fc.AttitudeMode == FlightComputerAttitudeMode.Auto && fc.CustomAttitudeTarget.Equals(new double3(3, 2, 1))
            && fc.RCSMode == FlightComputerRCSMode.Disabled);
        AccessTools.Method(typeof(GuidanceWindow), "ExecuteAscent").Invoke(null, new object[] { vehicle, vehicle.Orbit, home });
        t.Check("kFS refuses a fresh launch", !state.ConvexLaunchPending && !state.Running && state.Status.Contains("KFS"));
        CheckRcs(t, vehicle, loaded: true);
        AccessTools.PropertySetter(typeof(HostComputer), "State").Invoke(host.Computer, new object[] { ComputerState.Idle });
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check("kFS release restarts nothing", !state.Running && !state.ConvexLaunchPending && !RcsExecutor.IsActive(vehicle));
    }
}

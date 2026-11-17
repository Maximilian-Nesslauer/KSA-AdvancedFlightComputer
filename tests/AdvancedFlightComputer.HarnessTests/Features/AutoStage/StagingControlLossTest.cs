using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;
using KSA.Kfs;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class StagingControlLossTest : AfcTest
{
    public override string Name => "afc-autostage-control-loss";

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
        bool enabled = KfsPrototype.Enabled;
        using var patches = AutoStageTestPatches.Apply();
        try
        {
            foreach (bool kfs in new[] { false, true })
            {
                var driver = t.Session.CreateDriver();
                Vehicle vehicle = VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance,
                    kfs ? "AfcStageKfsLoss" : "AfcStageControlLoss", OrbitFixtures.CircularAt(home, 500_000, driver.Elapsed));
                try
                {
                    driver.Step(0.02, 2);
                    CheckDelayedRow(t, vehicle, driver, kfs);
                }
                finally
                {
                    vehicle.KfsHost?.Computer.Stop();
                    vehicle.KfsHost = null;
                    StagingDetector.ForgetVehicle(vehicle);
                    VehicleSpawner.Despawn(vehicle);
                }
            }
        }
        finally
        {
            KfsPrototype.Enabled = enabled;
        }
    }

    private static void CheckDelayedRow(TestContext t, Vehicle vehicle, SimDriver driver, bool kfs)
    {
        StagingConfig.EngineDelays.Clear();
        foreach (var engine in vehicle.Parts.Modules.Get<EngineController>())
            StagingConfig.EngineDelays[SequencedModules.DelayKey(engine)] = 1.0;
        int? ignitionRow = null;
        foreach (Sequence row in vehicle.Parts.SequenceList.Sequences)
        {
            var modules = new List<string>();
            foreach (Part part in row.Parts)
                foreach (ISequenced module in part.InSequence(row.Number))
                {
                    modules.Add($"{module.GetType().Name} active={module.IsActive}");
                    if (ignitionRow == null && module is EngineController { IsActive: false })
                        ignitionRow = row.Number;
                }
            t.Info($"row {row.Number}, activated={row.Activated}: {string.Join(", ", modules)}");
        }
        if (!t.Check("the fixture has an inactive engine row", ignitionRow.HasValue))
            return;
        // The saved first stage is already active, so the pending ignition uses an inactive engine row.
        vehicle.Parts.SequenceList.SetActiveSequence(ignitionRow!.Value - 1);
        StagingDetector.Arm(vehicle, true);
        StagingDetector.RequestStaging(vehicle);
        StagingDetector.Sample();
        StagingDetector.Evaluate();
        var state = StagingDetector.StateOf(vehicle);
        PendingStaging? pending = state.Pending;
        if (!t.Check($"a real row waits for ignition, kFS {kfs}", pending?.EngineModules is { Count: > 0 },
            $"control={vehicle.IsControllable}, armed={state.Active}, phase={state.State}, requested={state.Requested}, activations={state.Activations}, active row={vehicle.Parts.SequenceList.ActiveSequence}"))
            return;
        var engines = pending!.EngineModules!.ToArray();
        int activations = state.Activations;
        int sequence = vehicle.Parts.SequenceList.ActiveSequence;
        if (kfs)
        {
            KfsPrototype.Enabled = true;
            var host = KfsFixtures.QueuePilot(vehicle);
            t.Check("queued start does not own control yet", !VehicleControlOwnership.KfsOwns(vehicle));
            var dispatch = KfsFixtures.DispatchPilot(host);
            t.Check("queued start acquires pilot control", dispatch.UnderControl && dispatch.Pilot);
        }
        else
        {
            KfsFixtures.LoseControl(vehicle, driver);
            if (!t.Check("the crew loss removes control", !vehicle.IsControllable))
                return;
        }
        TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: true);
        StagingDetector.Evaluate();
        t.Check($"loss cancels delayed and requested work, kFS {kfs}", Clean(state));
        t.Check($"the activated row is not undone, kFS {kfs}", vehicle.Parts.SequenceList.ActiveSequence == sequence);
        foreach (var engine in engines)
            t.Check($"the delayed engine has not fired, kFS {kfs}", !engine.IsActive);

        // Cross both deadlines through the real simulation, then recover without issuing another request.
        driver.Step(0.05, 30);
        StagingDetector.FlushPendingForUnload();
        t.Check($"deadline and unload fire no cancelled module, kFS {kfs}", engines.All(engine => !engine.IsActive)
            && state.Activations == activations);

        state.Requested = true;
        state.HeldForControl = true;
        state.ActivationEngines = new();
        state.Sampled = true;
        state.State = StagingState.Phase.AwaitingActivation;
        StagingDetector.Arm(vehicle, true);
        t.Check($"arming while unavailable clears stale state, kFS {kfs}", Clean(state));
        t.Check($"a new request is refused, kFS {kfs}", StagingDetector.RequestStaging(vehicle) == StagingDetector.StagingRequest.Unavailable);
        t.Check($"the split entry point refuses, kFS {kfs}", StagingExecution.ActivateNextSequenceSplit(vehicle) == null
            && vehicle.Parts.SequenceList.ActiveSequence == sequence);
        StagingExecution.ActivatePendingModules(vehicle, engines.ToList(), "engine");
        t.Check($"direct delayed activation refuses, kFS {kfs}", engines.All(engine => !engine.IsActive));
        StagingDetector.OnPlayerStaged(vehicle, new());
        t.Check($"player staging installs no AFC hold, kFS {kfs}", Clean(state));

        var inputs = AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");
        t.Check($"cancellation leaves the manual engine command alone, kFS {kfs}", inputs(vehicle).EngineOn
            && inputs(vehicle).EngineThrottle == 0.63f);
        vehicle.KfsHost?.Computer.Stop();
        vehicle.KfsHost = null;
        if (!kfs)
            VehicleSpawner.SeatRandomCrew(vehicle);
        driver.Step(0.02, 2);
        t.Check($"control is available again, kFS {kfs}", vehicle.IsControllable);
        StagingDetector.Sample();
        StagingDetector.Evaluate();
        t.Check($"recovery resumes no staging, kFS {kfs}", Clean(state) && state.Activations == activations
            && engines.All(engine => !engine.IsActive));
    }

    private static bool Clean(StagingState state) => !state.Active && !state.Requested && state.Pending == null
        && state.HeldBurn == null && !state.HeldForControl && state.ActivationEngines == null
        && state.State == StagingState.Phase.Monitoring && !state.Sampled && double.IsNaN(state.SpentJettisonSince);
}

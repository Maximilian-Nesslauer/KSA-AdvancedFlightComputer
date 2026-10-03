using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The driver's exception path on a live vehicle. A faulted tick has to hand back the controls
// the executor took, keep whatever it could not hand back visible and retryable, leave the burn
// node alone, and never raise the stock flag that tells FlightComputer.RaisePendingAlerts the
// burn ended. The two ends that do raise it, a completion and a propellant stop, are checked
// against the stock handler too.
//
// The faults are injected with Harmony because these paths cannot be made to fail from the
// outside. Each fault is armed around one driver step and disarmed in a finally, so the live
// simulation never runs with a throwing game method.
public sealed class RcsDriverFaultTest : AfcTest
{
    public override string Name => "afc-rcs-driver-fault";

    private static bool _failTick;
    private static bool _failAttitude;
    private static bool _failRcs;
    private static bool _failTelemetry;
    private static bool _completeInTick;
    private static int _attitudeCalls;
    private static int _rcsCalls;

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(RcsTestVehicles.Candidates);
        if (saves.Count == 0)
        {
            t.Skip("no RCS test vehicle save present.");
            return;
        }

        // Every case here drives the executor's state machine, which does not depend on the
        // thruster layout, so one save carries the whole set.
        using RcsTestPatches.Scope patches = RcsTestPatches.Apply();
        using FaultScope faults = new();
        RcsFlightSupport.RunOnSave(t, home, saves[0], 500_000.0, "HarnessRcsFault",
            (vehicle, driver) =>
            {
                using RcsFlightSupport.CompletionWatcher completions = new(vehicle);
                Vehicle? other = SpawnSecond(t, home, saves[0], driver);
                PublishedFaultStopsAndPreservesOtherVehicle(t, vehicle, other, driver, completions);
                AttitudeFailureRestoresRcsAndRetries(t, vehicle, driver, completions);
                RcsFailureDoesNotRepeatAttitudeRelease(t, vehicle, driver, completions);
                RetriesAreBoundedAndUserCanRetry(t, vehicle, driver, completions);
                TelemetryFailureCannotPreventShutdown(t, vehicle, driver, completions);
                FaultSurvivesSaveAndLoad(t, vehicle, driver, completions);
                UnownedControlsStayUnchanged(t, vehicle, driver);
                CompletionCleanupFailureDoesNotNotify(t, vehicle, driver, completions);
                NormalCompletionRaisesStockFlagAfterTeardown(t, vehicle, driver, completions);
                PropellantStopEndsLikeAnEngineBurn(t, vehicle, driver, completions);
                DryThrustersStopAtIgnition(t, vehicle, driver, completions);
                InactiveThrustersCancelWithoutStockFlag(t, vehicle, driver, completions);
            });
    }

    private sealed record Craft(Vehicle Vehicle, FlightComputer Computer, RcsExecution Exec, Burn Burn, double3 Dv);

    // Puts one vehicle into the state a running RCS burn leaves behind: both control flags
    // owned, a published worker command, and one burn with its options.
    private static Craft? Arm(TestContext t, Vehicle vehicle, SimDriver driver, string label)
    {
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.CleanupBurns(fc);
        RcsFlightSupport.BurnSetup? setup = RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitX, 1.0, 60.0);
        if (setup == null)
        {
            t.Fail(label, "no patch or loaded burn target");
            return null;
        }
        double timeSec = (driver.Elapsed + 60.0).Seconds();
        fc.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Custom;
        fc.RCSMode = FlightComputerRCSMode.Enabled;

        RcsExecution exec = RcsExecRegistry.GetOrCreate(vehicle.Id);
        exec.ActiveBurn = setup.Burn;
        exec.ActiveBurnTimeSec = timeSec;
        exec.ActiveBurnDvMs = 1.0;
        exec.ResolvedStrategy = RcsAttitudeStrategy.Align;
        exec.ResolvedAxis = 0;
        exec.AlignCommanded = true;
        exec.ForcedRcsOn = true;
        exec.ControlTaken = true;
        exec.ReconciledAfterLoad = true;
        exec.GetOrCreateOptions(timeSec, 1.0).Mode = RcsExecutionMode.Rcs;
        RcsWorkerCommand command = new() { Active = true };
        exec.LastPublishedCommand = command;
        RcsCommandChannel.Publish(fc.BurnPlan, command);
        return new Craft(vehicle, fc, exec, setup.Burn, setup.Burn.DeltaVVlf);
    }

    // A case starts from an empty registry, so an earlier fault cannot carry into the next one.
    private static Craft? Fresh(TestContext t, Vehicle vehicle, SimDriver driver,
        RcsFlightSupport.CompletionWatcher? completions, string label)
    {
        RcsExecRegistry.Reset();
        RcsCommandChannel.Reset();
        completions?.Reset();
        return Arm(t, vehicle, driver, label);
    }

    private static Vehicle? SpawnSecond(TestContext t, IParentBody home, string save, SimDriver driver)
    {
        // RunOnSave despawns every vehicle it did not find, so this one is torn down with it.
        Orbit orbit = OrbitFixtures.CircularAt(home, 600_000.0, driver.Elapsed);
        try
        {
            return VehicleSpawner.SpawnFromSave(save, t.System, home, "HarnessRcsFaultOther", orbit);
        }
        catch (InvalidOperationException e)
        {
            t.Skip($"second vehicle: {e.Message}");
            return null;
        }
    }

    private static void Fault(TestContext t, Craft craft, string label)
    {
        _failTick = true;
        try { RcsDriverPatch.TickVehicle(craft.Vehicle); }
        finally { _failTick = false; }
        t.Check($"{label}: faulted execution is inactive", !craft.Exec.IsActive && craft.Exec.Faulted);
        t.Check($"{label}: fault drops published execution state",
            !craft.Exec.ControlTaken && craft.Exec.LastPublishedCommand == null
            && !RcsCommandChannel.TryGet(craft.Computer.BurnPlan, out _));
    }

    private static void CheckPlan(TestContext t, Craft craft,
        RcsFlightSupport.CompletionWatcher completions, string label)
    {
        t.Check($"{label}: burn node survives the fault",
            craft.Computer.BurnPlan.BurnCount == 1
            && ReferenceEquals(craft.Computer.BurnPlan.FirstBurn, craft.Burn)
            && craft.Burn.DeltaVVlf.Equals(craft.Dv));
        t.Check($"{label}: burn options survive the fault", craft.Exec.Options.Count == 1);
        t.Check($"{label}: no stock burn end raised", !completions.AnyEnd);
    }

    private static void PublishedFaultStopsAndPreservesOtherVehicle(TestContext t, Vehicle vehicle,
        Vehicle? other, SimDriver driver, RcsFlightSupport.CompletionWatcher completions)
    {
        const string label = "published fault";
        if (Fresh(t, vehicle, driver, completions, label) is not { } craft)
            return;
        Craft? second = other == null ? null : Arm(t, other, driver, label);
        Fault(t, craft, label);
        t.Check($"{label}: RCS given back",
            !craft.Exec.CleanupPending && craft.Computer.RCSMode == FlightComputerRCSMode.Disabled);
        t.Check($"{label}: attitude tracking released",
            craft.Computer.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.None);
        if (second != null)
            t.Check($"{label}: another vehicle is untouched",
                second.Exec.IsActive && second.Exec.AlignCommanded && second.Exec.ForcedRcsOn
                && second.Computer.RCSMode == FlightComputerRCSMode.Enabled
                && second.Computer.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Custom
                && RcsCommandChannel.TryGet(second.Computer.BurnPlan, out _));
        CheckPlan(t, craft, completions, label);
    }

    private static void AttitudeFailureRestoresRcsAndRetries(TestContext t, Vehicle vehicle,
        SimDriver driver, RcsFlightSupport.CompletionWatcher completions)
    {
        const string label = "attitude cleanup failure";
        if (Fresh(t, vehicle, driver, completions, label) is not { } craft)
            return;
        _failAttitude = true;
        try { Fault(t, craft, label); }
        finally { _failAttitude = false; }
        t.Check($"{label}: attitude ownership stays pending",
            craft.Exec.CleanupPending && craft.Exec.AlignCommanded && !craft.Exec.ForcedRcsOn);
        t.Check($"{label}: RCS is restored independently",
            craft.Computer.RCSMode == FlightComputerRCSMode.Disabled);
        t.Check($"{label}: the failure is visible", craft.Exec.CleanupError != null);
        RcsDriverPatch.TickVehicle(craft.Vehicle);
        t.Check($"{label}: the next driver step finishes cleanup",
            !craft.Exec.CleanupPending && craft.Exec.CleanupError == null);
        CheckPlan(t, craft, completions, label);
    }

    private static void RcsFailureDoesNotRepeatAttitudeRelease(TestContext t, Vehicle vehicle,
        SimDriver driver, RcsFlightSupport.CompletionWatcher completions)
    {
        const string label = "RCS cleanup failure";
        if (Fresh(t, vehicle, driver, completions, label) is not { } craft)
            return;
        _failRcs = true;
        try { Fault(t, craft, label); }
        finally { _failRcs = false; }
        t.Check($"{label}: RCS ownership stays pending",
            !craft.Exec.AlignCommanded && craft.Exec.ForcedRcsOn);
        int calls = _attitudeCalls;
        craft.Computer.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Custom;
        RcsDriverPatch.TickVehicle(craft.Vehicle);
        t.Check($"{label}: the retry keeps a player attitude change",
            _attitudeCalls == calls
            && craft.Computer.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Custom);
        t.Check($"{label}: the retry finishes", !craft.Exec.CleanupPending);
        CheckPlan(t, craft, completions, label);
    }

    private static void RetriesAreBoundedAndUserCanRetry(TestContext t, Vehicle vehicle,
        SimDriver driver, RcsFlightSupport.CompletionWatcher completions)
    {
        const string label = "bounded retries";
        if (Fresh(t, vehicle, driver, completions, label) is not { } craft)
            return;
        int before = _attitudeCalls;
        _failAttitude = true;
        try
        {
            Fault(t, craft, label);
            for (int i = 0; i < 8; i++)
                RcsDriverPatch.TickVehicle(craft.Vehicle);
            t.Check($"{label}: automatic retries stop at the limit",
                _attitudeCalls - before == RcsExecutor.MaxCleanupAttempts);
            t.Check($"{label}: the exhausted retry keeps its ownership",
                craft.Exec.CleanupPending && craft.Exec.AlignCommanded);
            RcsExecutor.Activate(craft.Vehicle);
            t.Check($"{label}: activation cannot bypass pending cleanup",
                !craft.Exec.IsActive && craft.Exec.CleanupPending);
        }
        finally { _failAttitude = false; }
        RcsExecutor.RequestCancel(craft.Exec, "retry fault cleanup");
        t.Check($"{label}: the queued retry waits for the driver step", craft.Exec.AlignCommanded);
        RcsDriverPatch.TickVehicle(craft.Vehicle);
        t.Check($"{label}: the queued retry finishes cleanup",
            !craft.Exec.CleanupPending && !craft.Exec.IsActive);
        CheckPlan(t, craft, completions, label);
    }

    private static void TelemetryFailureCannotPreventShutdown(TestContext t, Vehicle vehicle,
        SimDriver driver, RcsFlightSupport.CompletionWatcher completions)
    {
        const string label = "fuel telemetry failure";
        if (Fresh(t, vehicle, driver, completions, label) is not { } craft)
            return;
        craft.Exec.CancelRequestReason = "user request";
        _failTelemetry = true;
        try { RcsDriverPatch.TickVehicle(craft.Vehicle); }
        finally { _failTelemetry = false; }
        t.Check($"{label}: shutdown still completes",
            craft.Exec.Faulted && !craft.Exec.IsActive && !craft.Exec.CleanupPending
            && !RcsCommandChannel.TryGet(craft.Computer.BurnPlan, out _));
        CheckPlan(t, craft, completions, label);
    }

    private static void FaultSurvivesSaveAndLoad(TestContext t, Vehicle vehicle,
        SimDriver driver, RcsFlightSupport.CompletionWatcher completions)
    {
        const string label = "fault across save and load";
        if (Fresh(t, vehicle, driver, completions, label) is not { } craft)
            return;
        craft.Exec.SaveId = "test-save";
        _failAttitude = true;
        try { Fault(t, craft, label); }
        finally { _failAttitude = false; }

        using StringWriter writer = new();
        RcsExecRegistry.WriteToml(writer, [craft.Exec]);
        Dictionary<(string SaveId, string VehicleId), RcsExecution> entries = new();
        if (!t.Check($"{label}: the saved state loads",
                RcsExecRegistry.ParseLines(writer.ToString().Split('\n'), "test", entries)))
            return;
        RcsExecution loaded = entries[("test-save", vehicle.Id)];
        t.Check($"{label}: the fault and its pending ownership survive",
            loaded.Faulted && !loaded.IsActive && loaded.CleanupPending && loaded.AlignCommanded
            && !loaded.ForcedRcsOn);
        t.Check($"{label}: a saved fault cannot resume as a burn",
            writer.ToString().Contains("active = false"));

        // A loaded execution reconciles its burn on the next driver step. A faulted one has to
        // clean up instead, so it must not reattach the burn it was running.
        craft.Exec.ReconciledAfterLoad = false;
        RcsDriverPatch.TickVehicle(craft.Vehicle);
        t.Check($"{label}: a loaded fault cleans up instead of reattaching",
            !craft.Exec.CleanupPending && !craft.Exec.IsActive && !craft.Exec.ReconciledAfterLoad);
        CheckPlan(t, craft, completions, label);
    }

    private static void UnownedControlsStayUnchanged(TestContext t, Vehicle vehicle, SimDriver driver)
    {
        const string label = "unowned controls";
        if (Fresh(t, vehicle, driver, null, label) is not { } craft)
            return;
        craft.Exec.AlignCommanded = false;
        craft.Exec.ForcedRcsOn = false;
        int before = _attitudeCalls + _rcsCalls;
        Fault(t, craft, label);
        t.Check($"{label}: a fault changes nothing it does not own",
            _attitudeCalls + _rcsCalls == before
            && craft.Computer.RCSMode == FlightComputerRCSMode.Enabled
            && craft.Computer.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Custom);
    }

    private static void CompletionCleanupFailureDoesNotNotify(TestContext t, Vehicle vehicle,
        SimDriver driver, RcsFlightSupport.CompletionWatcher completions)
    {
        const string label = "failed completion";
        if (Fresh(t, vehicle, driver, completions, label) is not { } craft)
            return;
        _completeInTick = true;
        _failRcs = true;
        try { RcsDriverPatch.TickVehicle(craft.Vehicle); }
        finally
        {
            _completeInTick = false;
            _failRcs = false;
        }
        t.Check($"{label}: a failed completion enters terminal cleanup",
            craft.Exec.Faulted && !craft.Exec.IsActive && craft.Exec.CleanupPending);
        RcsDriverPatch.TickVehicle(craft.Vehicle);
        t.Check($"{label}: the retry recovers", !craft.Exec.CleanupPending);
        CheckPlan(t, craft, completions, label);
    }

    // The stock flag is raised only after the release, so FlightComputer.RaisePendingAlerts finds the
    // controls handed back and the burn still loaded. With one burn planned stock keeps it, and the
    // ended execution does not start again on the next driver step.
    private static void NormalCompletionRaisesStockFlagAfterTeardown(TestContext t, Vehicle vehicle,
        SimDriver driver, RcsFlightSupport.CompletionWatcher completions)
    {
        const string label = "successful completion";
        if (Fresh(t, vehicle, driver, completions, label) is not { } craft)
            return;
        _completeInTick = true;
        try { RcsDriverPatch.TickVehicle(craft.Vehicle); }
        finally { _completeInTick = false; }
        t.Check($"{label}: the completion flag is raised after teardown",
            completions.CompletedOn(craft.Burn) && !craft.Computer.AutoBurnStoppedOutOfPropellant
            && !craft.Exec.IsActive && !craft.Exec.AlignCommanded && !craft.Exec.ForcedRcsOn
            && !RcsCommandChannel.TryGet(craft.Computer.BurnPlan, out _)
            && craft.Exec.Options.Count == 0);

        craft.Computer.RaisePendingAlerts(craft.Vehicle);
        t.Check($"{label}: stock clears the flag and keeps the only burn",
            !completions.AnyEnd && craft.Computer.BurnPlan.BurnCount == 1
            && ReferenceEquals(craft.Computer.BurnPlan.FirstBurn, craft.Burn)
            && craft.Computer.BurnMode == FlightComputerBurnMode.Manual);
        RcsDriverPatch.TickVehicle(craft.Vehicle);
        t.Check($"{label}: the ended execution stays ended",
            !craft.Exec.IsActive && !completions.AnyEnd
            && craft.Computer.BurnMode == FlightComputerBurnMode.Manual);
    }

    // A propellant stop hands the controls back like a completion, keeps the burn options for a
    // later restart, and raises the stock propellant flag. With another burn planned, stock removes
    // the stopped burn and loads the next one in Manual, and the driver leaves that one alone.
    private static void PropellantStopEndsLikeAnEngineBurn(TestContext t, Vehicle vehicle,
        SimDriver driver, RcsFlightSupport.CompletionWatcher completions)
    {
        const string label = "propellant stop";
        if (Fresh(t, vehicle, driver, completions, label) is not { } craft)
            return;
        RcsFlightSupport.BurnSetup? next = RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitX, 1.0, 600.0);
        if (!t.Check($"{label}: a second burn is planned after the running one",
                next != null && craft.Computer.BurnPlan.BurnCount == 2
                && ReferenceEquals(craft.Computer.BurnPlan.FindFirstExecutableBurn(), craft.Burn)))
            return;

        RcsExecutor.StopOutOfPropellant(craft.Vehicle, craft.Computer, craft.Exec);
        t.Check($"{label}: the propellant flag is raised after teardown",
            completions.StoppedOutOfPropellantOn(craft.Burn) && !craft.Computer.AutoBurnCompleted
            && !craft.Exec.IsActive && !craft.Exec.AlignCommanded && !craft.Exec.ForcedRcsOn
            && craft.Computer.RCSMode == FlightComputerRCSMode.Disabled
            && !RcsCommandChannel.TryGet(craft.Computer.BurnPlan, out _)
            && craft.Exec.Options.Count == 1);

        craft.Computer.RaisePendingAlerts(craft.Vehicle);
        t.Check($"{label}: stock removes the stopped burn once and loads the next in Manual",
            !completions.AnyEnd && craft.Computer.BurnPlan.BurnCount == 1
            && ReferenceEquals(craft.Computer.BurnPlan.FirstBurn, next!.Burn)
            && StockBurnIdentity.IsLoaded(craft.Computer.Burn, next.Burn)
            && craft.Computer.BurnMode == FlightComputerBurnMode.Manual);
        RcsDriverPatch.TickVehicle(craft.Vehicle);
        t.Check($"{label}: the driver does not engage the next burn",
            !craft.Exec.IsActive && craft.Computer.BurnMode == FlightComputerBurnMode.Manual
            && !RcsCommandChannel.TryGet(craft.Computer.BurnPlan, out _));
    }

    // The driver decides between the stock propellant stop and a cancel from the live thruster
    // states. With every active thruster dry, the burn waits until its ignition time like a stock
    // Auto burn and is then stopped through the stock flag with its options kept.
    private static void DryThrustersStopAtIgnition(TestContext t, Vehicle vehicle,
        SimDriver driver, RcsFlightSupport.CompletionWatcher completions)
    {
        const string label = "dry thrusters";
        if (Fresh(t, vehicle, driver, completions, label) is not { } craft)
            return;
        BurnTarget? bt = craft.Computer.Burn;
        double nowSec = Universe.GetElapsedTime().Seconds();
        if (!t.Check($"{label}: the burn is loaded before its ignition time",
                bt != null && nowSec < bt.IgnitionTime.Seconds()))
            return;

        bool[] supplied = SetThrusterPropellant(vehicle, null);
        try
        {
            if (!t.Check($"{label}: the vehicle has active thrusters", supplied.Length > 0
                    && RcsCapability.ActiveThrustersOutOfPropellant(vehicle)))
                return;
            craft.Exec.CapabilityProbedAtSec = double.NegativeInfinity;
            RcsDriverPatch.TickVehicle(vehicle);
            t.Check($"{label}: before ignition the burn waits",
                craft.Exec.IsActive && !completions.AnyEnd && craft.Exec.Options.Count == 1);

            bt!.IgnitionTime = Universe.GetElapsedTime();
            RcsDriverPatch.TickVehicle(vehicle);
            t.Check($"{label}: at ignition the stock propellant stop is raised",
                !craft.Exec.IsActive && !craft.Exec.Faulted
                && completions.StoppedOutOfPropellantOn(craft.Burn)
                && !craft.Computer.AutoBurnCompleted
                && craft.Computer.RCSMode == FlightComputerRCSMode.Disabled
                && craft.Exec.Options.Count == 1);
        }
        finally
        {
            SetThrusterPropellant(vehicle, supplied);
            completions.Reset();
        }
    }

    // Thrusters that are switched off are not out of propellant. Stock raises no stop when no
    // engine is active, so the driver cancels with its own alert and raises no stock flag.
    private static void InactiveThrustersCancelWithoutStockFlag(TestContext t, Vehicle vehicle,
        SimDriver driver, RcsFlightSupport.CompletionWatcher completions)
    {
        const string label = "inactive thrusters";
        if (Fresh(t, vehicle, driver, completions, label) is not { } craft)
            return;
        List<ThrusterController> switchedOff = SetActiveThrusters(vehicle, false, null);
        try
        {
            if (!t.Check($"{label}: the vehicle had active thrusters", switchedOff.Count > 0
                    && !RcsCapability.ActiveThrustersOutOfPropellant(vehicle)))
                return;
            RcsCancelLogPatch.LastReason = null;
            craft.Exec.CapabilityProbedAtSec = double.NegativeInfinity;
            RcsDriverPatch.TickVehicle(vehicle);
            t.Check($"{label}: the burn is cancelled without a stock flag",
                !craft.Exec.IsActive && !craft.Exec.Faulted && !completions.AnyEnd
                && RcsCancelLogPatch.LastReason == "no usable translation");
        }
        finally
        {
            SetActiveThrusters(vehicle, true, switchedOff);
        }
        CheckPlan(t, craft, completions, label);
    }

    // Writes IsPropellantAvailable on every active thruster, false when restore is null and the
    // saved values otherwise, and returns the values it found, in enumeration order.
    private static bool[] SetThrusterPropellant(Vehicle vehicle, bool[]? restore)
    {
        List<bool> found = new();
        if (!ThrusterController.TryGetFrom(vehicle.Parts.States, out var stateList))
            return [];
        var enumerator = new ModuleStateful<ThrusterController, ThrusterControllerState, ThrusterControllerGlobalState, EmptyStruct>
            .StateList.ModuleAndStateEnumerator(stateList);
        while (enumerator.MoveNext())
        {
            var current = enumerator.Current;
            if (!current.Module.IsActive)
                continue;
            ref ThrusterControllerState state = ref Unsafe.AsRef(in current.State);
            found.Add(state.IsPropellantAvailable);
            state.IsPropellantAvailable = restore != null && found.Count <= restore.Length
                && restore[found.Count - 1];
        }
        return found.ToArray();
    }

    // Switches thrusters through the stock input path that the part window uses. Off acts on
    // every active thruster and returns them, on acts only on the given list.
    private static List<ThrusterController> SetActiveThrusters(Vehicle vehicle, bool active,
        List<ThrusterController>? only)
    {
        List<ThrusterController> changed = new();
        if (ThrusterController.TryGetFrom(vehicle.Parts.States, out var stateList))
        {
            List<ThrusterController> candidates = only ?? new List<ThrusterController>(stateList.Modules.ToArray());
            foreach (ThrusterController thruster in candidates)
            {
                if (thruster.IsActive == active)
                    continue;
                thruster.SetIsActive(vehicle, active);
                changed.Add(thruster);
            }
        }
        InputEvents.ApplyInputEvents();
        return changed;
    }

    // The four injection points. Each one only throws while its flag is armed, so the patches
    // are inert for the rest of the run.
    private sealed class FaultScope : IDisposable
    {
        private readonly Harmony _harmony;

        internal FaultScope()
        {
            _harmony = new Harmony("com.maxi.afc.harnesstests.rcs.fault");
            _harmony.Patch(AccessTools.Method(typeof(RcsExecutor), nameof(RcsExecutor.Tick)),
                prefix: new HarmonyMethod(typeof(RcsDriverFaultTest), nameof(TickPrefix)));
            _harmony.Patch(AccessTools.Method(typeof(FlightComputer), nameof(FlightComputer.SetNullRot)),
                prefix: new HarmonyMethod(typeof(RcsDriverFaultTest), nameof(AttitudePrefix)));
            _harmony.Patch(AccessTools.Method(typeof(RcsExecutor), "RestoreRcsMode"),
                prefix: new HarmonyMethod(typeof(RcsDriverFaultTest), nameof(RcsPrefix)));
            _harmony.Patch(AccessTools.Method(typeof(RcsExecutor), "ComputeFuelSummary"),
                prefix: new HarmonyMethod(typeof(RcsDriverFaultTest), nameof(TelemetryPrefix)));
        }

        public void Dispose()
        {
            _failTick = false;
            _failAttitude = false;
            _failRcs = false;
            _failTelemetry = false;
            _completeInTick = false;
            _harmony.UnpatchAll(_harmony.Id);
        }
    }

    private static bool TickPrefix(Vehicle __0)
    {
        if (_failTick)
            throw new InvalidOperationException("Injected driver fault");
        if (!_completeInTick)
            return true;
        RcsExecRegistry.TryGet(__0.Id, out RcsExecution? exec);
        RcsExecutor.Complete(__0, __0.FlightComputer, exec!, 0f);
        return false;
    }

    private static void AttitudePrefix()
    {
        _attitudeCalls++;
        if (_failAttitude)
            throw new InvalidOperationException("Injected attitude cleanup fault");
    }

    private static void RcsPrefix()
    {
        _rcsCalls++;
        if (_failRcs)
            throw new InvalidOperationException("Injected RCS cleanup fault");
    }

    private static void TelemetryPrefix()
    {
        if (_failTelemetry)
            throw new InvalidOperationException("Injected fuel telemetry fault");
    }
}

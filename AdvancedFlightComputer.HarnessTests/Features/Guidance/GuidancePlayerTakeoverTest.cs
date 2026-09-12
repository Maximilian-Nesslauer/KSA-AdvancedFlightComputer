using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Check takeover, a pending cut, and a failed release through ApplyAutopilot.
// The calls are direct instead of through the vehicle driver, so each case controls one step.
public sealed class GuidancePlayerTakeoverTest : AfcTest
{
    public override string Name => "afc-guidance-player-takeover";

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
        if (VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state.");
            return;
        }

        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        // ApplyAutopilot binds this static to the fixture state, and removing the entry cannot
        // clear that reference, so the test puts back what it found.
        VehicleAutopilotState previousAmbient = Ambient();
        Vehicle? fixture = null;
        try
        {
            Orbit orbit = OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed);
            Vehicle vehicle;
            try
            {
                vehicle = VehicleSpawner.SpawnFromSave(
                    saves[0], t.System, home, "HarnessGuidanceTakeover", orbit);
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"'{saves[0]}': {e.Message}");
                return;
            }
            fixture = vehicle;

            ATakeoverStopsTheModeAndLeavesTheEngine(t, vehicle);
            AnAutomaticCutSurvivesATakeover(t, vehicle);
            AnAbortKeepsItsShutdownThroughATakeover(t, vehicle);
            AFailedReleaseKeepsTheNoCutRequest(t, vehicle);
            AnAbortBetweenRetriesStillCuts(t, vehicle);
            AHandoverKeepsTheModeItStarted(t, vehicle);
        }
        finally
        {
            if (fixture != null)
                VehicleAutopilotState.Remove(fixture);
            Ambient() = previousAmbient;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void ATakeoverStopsTheModeAndLeavesTheEngine(TestContext t, Vehicle vehicle)
    {
        FlightComputer computer = vehicle.FlightComputer;
        VehicleAutopilotState state = Engaged(vehicle);

        TakeTheAttitude(vehicle);
        GuidanceWindow.ApplyAutopilot(vehicle);

        t.Check("the mode stops", !state.ControlAcquired && !state.Running);
        t.Check("the engine command is left alone", Inputs(vehicle).EngineOn);
        t.Check("the throttle setting is left alone", Inputs(vehicle).EngineThrottle == 0.63f);
        t.Check("the target the player selected survives",
            computer.AttitudeTrackTarget == FlightComputerAttitudeTrackTarget.Prograde);
        t.Check("the reason is visible", state.Status.Contains("attitude"));
        t.Check("the finished release restores the default stop", !state.ReleaseWithoutEngineCut);
    }

    private static void AFailedReleaseKeepsTheNoCutRequest(TestContext t, Vehicle vehicle)
    {
        VehicleAutopilotState state = Engaged(vehicle);
        TakeTheAttitude(vehicle);

        var harmony = new Harmony("com.maxi.afc.harnesstests.guidance.takeover");
        harmony.Patch(AccessTools.Method(typeof(KsaAttitudeRate), nameof(KsaAttitudeRate.Clear)),
            prefix: new HarmonyMethod(typeof(GuidancePlayerTakeoverTest), nameof(RejectRateClear)));
        try
        {
            _failing = vehicle;
            GuidanceWindow.ApplyAutopilot(vehicle);
            t.Check("a failed release keeps its retry", state.FcResetPending && state.ControlAcquired);
            t.Check("a failed release still does not cut the engine", Inputs(vehicle).EngineOn);
            t.Check("the no-cut request waits for the retry", state.ReleaseWithoutEngineCut);

            _failing = null;
            GuidanceWindow.ApplyAutopilot(vehicle);
            t.Check("the retry finishes", !state.ControlAcquired && !state.FcResetPending);
            t.Check("the retry does not cut the engine either", Inputs(vehicle).EngineOn);
            t.Check("the retry still says why guidance stopped", state.Status.Contains("attitude"));
        }
        finally
        {
            _failing = null;
            harmony.UnpatchAll(harmony.Id);
            VehicleAutopilotState.Remove(vehicle);
        }
    }

    // The production handover, driven through the dispatch that decides whether a 6-DOF step was a
    // stop. The step itself is replaced, because reaching the handoff altitude needs a full solve,
    // but the transition it performs is the production pair of calls.
    private static Vehicle? _handingOver;
    private static bool _handoverDone;

    // The save's engines are not lit, and hover refuses without throttle control. The dispatch
    // under test does not depend on that capability, so the probe is answered for the fixture.
    private static bool ForceThrottleControl(Vehicle vehicle, ref bool __result)
    {
        if (!ReferenceEquals(vehicle, _handingOver))
            return true;
        __result = true;
        return false;
    }

    private static bool HandOverToTerminalHover(Vehicle vehicle)
    {
        if (!ReferenceEquals(vehicle, _handingOver) || _handoverDone)
            return true;
        _handoverDone = true;
        AccessTools.Method(typeof(GuidanceWindow), "Disengage6Dof")
            ?.Invoke(null, new object[] { vehicle, false });
        AccessTools.Method(typeof(GuidanceWindow), "StartTerminalHover")
            ?.Invoke(null, new object[] { vehicle });
        return false;
    }

    private static void AHandoverKeepsTheModeItStarted(TestContext t, Vehicle vehicle)
    {
        VehicleAutopilotState state = Engaged(vehicle);
        state.Active = true;
        if (!t.Check("the fixture holds the claim",
                VehicleControlOwnership.TryClaim(vehicle, ControlClaimant.Guidance, out _)))
            return;

        var harmony = new Harmony("com.maxi.afc.harnesstests.guidance.handover");
        harmony.Patch(AccessTools.Method(typeof(GuidanceWindow), "Step6Dof"),
            prefix: new HarmonyMethod(typeof(GuidancePlayerTakeoverTest), nameof(HandOverToTerminalHover)));
        harmony.Patch(AccessTools.Method(typeof(KsaEnginePerf), nameof(KsaEnginePerf.SupportsThrottleControl)),
            prefix: new HarmonyMethod(typeof(GuidancePlayerTakeoverTest), nameof(ForceThrottleControl)));
        try
        {
            _handoverDone = false;
            _handingOver = vehicle;
            GuidanceWindow.ApplyAutopilot(vehicle);

            if (!t.Check("the handover started terminal hover",
                    state.LandingPhase == GuidanceWindow.LandingPhase.TerminalHover,
                    state.LandingStatus))
                return;
            t.Check("6-DOF is no longer active", !state.Active && !state.EngagePending);
            t.Check("the handover keeps the engine lit", Inputs(vehicle).EngineOn);
            t.Check("the mode that started keeps the claim",
                VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.Guidance
                && state.ControlAcquired);
        }
        finally
        {
            _handingOver = null;
            _handoverDone = false;
            harmony.UnpatchAll(harmony.Id);
            VehicleControlOwnership.ReleaseAll(vehicle);
            VehicleAutopilotState.Remove(vehicle);
        }
    }

    // The no-cut request outlives a failed cleanup, so an abort between two retries has to take
    // the engine channel back from it.
    private static void AnAbortBetweenRetriesStillCuts(TestContext t, Vehicle vehicle)
    {
        VehicleAutopilotState state = Engaged(vehicle);
        TakeTheAttitude(vehicle);

        var harmony = new Harmony("com.maxi.afc.harnesstests.guidance.takeover.abort");
        harmony.Patch(AccessTools.Method(typeof(KsaAttitudeRate), nameof(KsaAttitudeRate.Clear)),
            prefix: new HarmonyMethod(typeof(GuidancePlayerTakeoverTest), nameof(RejectRateClear)));
        try
        {
            _failing = vehicle;
            GuidanceWindow.ApplyAutopilot(vehicle);
            if (!t.Check("the takeover cleanup failed and kept its no-cut request",
                    state.FcResetPending && state.ReleaseWithoutEngineCut && Inputs(vehicle).EngineOn))
                return;

            Ambient() = state;
            AccessTools.Method(typeof(GuidanceWindow), "AbortLanding")?.Invoke(null, Array.Empty<object>());
            _failing = null;
            GuidanceWindow.ApplyAutopilot(vehicle);

            t.Check("the abort takes the engine channel back", !Inputs(vehicle).EngineOn);
            t.Check("the retry finishes", !state.ControlAcquired && !state.FcResetPending);
            t.Check("the abort leaves no cut pending", !state.LandingCutPending);
        }
        finally
        {
            _failing = null;
            harmony.UnpatchAll(harmony.Id);
            VehicleAutopilotState.Remove(vehicle);
        }
    }

    // The production abort paths, called the way the panel calls them, so the test proves that
    // they record the shutdown intent and not only that the release honours a flag.
    private static void AnAbortKeepsItsShutdownThroughATakeover(TestContext t, Vehicle vehicle)
    {
        AbortPathKeepsItsShutdown(t, vehicle, "ascent abort", "AbortAscent", withVehicle: false);
        AbortPathKeepsItsShutdown(t, vehicle, "landing abort", "AbortLanding", withVehicle: false);
        AbortPathKeepsItsShutdown(t, vehicle, "boostback abort", "AbortBoostback", withVehicle: false);
        AbortPathKeepsItsShutdown(t, vehicle, "6-DOF disengage", "Disengage6Dof", withVehicle: true);
    }

    private static void AbortPathKeepsItsShutdown(
        TestContext t, Vehicle vehicle, string label, string method, bool withVehicle)
    {
        VehicleAutopilotState state = Engaged(vehicle);
        Ambient() = state;
        object[] arguments = withVehicle ? new object[] { vehicle, true } : Array.Empty<object>();
        AccessTools.Method(typeof(GuidanceWindow), method)?.Invoke(null, arguments);
        if (!t.Check($"{label} records its shutdown",
                state.LandingCutPending || state.ShutdownRequested))
        {
            VehicleAutopilotState.Remove(vehicle);
            return;
        }

        TakeTheAttitude(vehicle);
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check($"{label} still shuts the engine down through a takeover", !Inputs(vehicle).EngineOn);
        t.Check($"{label} leaves no cut pending", !state.LandingCutPending);
        t.Check($"{label} still reports the takeover", state.Status.Contains("attitude"));
        VehicleAutopilotState.Remove(vehicle);
    }

    // A cut a mode queued on its own, which is the shape touchdown and a failed landing solve
    // leave behind. It has no explicit stop, and it still has to reach the engine.
    private static void AnAutomaticCutSurvivesATakeover(TestContext t, Vehicle vehicle)
    {
        VehicleAutopilotState state = Engaged(vehicle);
        state.LandingCutPending = true;

        TakeTheAttitude(vehicle);
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check("an automatic cut still reaches the engine", !Inputs(vehicle).EngineOn);
        t.Check("an automatic cut leaves nothing pending", !state.LandingCutPending);
        VehicleAutopilotState.Remove(vehicle);
    }

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(
            AccessTools.Field(typeof(GuidanceWindow), "_s"));

    private static ref VehicleAutopilotState Ambient() => ref AmbientState();

    private static void RejectRateClear(Vehicle __0)
    {
        if (ReferenceEquals(__0, _failing))
            throw new InvalidOperationException("Injected rate cleanup failure");
    }

    // A running mode that only a takeover can end, so the other two release reasons stay false.
    private static VehicleAutopilotState Engaged(Vehicle vehicle)
    {
        TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: true);
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        state.Engage = true;
        state.Running = true;
        state.ControlAcquired = true;

        FlightComputer computer = vehicle.FlightComputer;
        state.AttitudeOwnership.BeginWrite(computer);
        computer.CustomAttitudeTarget = new double3(1, 2, 3);
        computer.AttitudeFrame = VehicleReferenceFrame.EclBody;
        computer.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
        state.AttitudeOwnership.EndWrite(computer, writesRoll: false);
        return state;
    }

    // The player steering through the gauge, which is the path Vehicle.SetEnum serves.
    private static void TakeTheAttitude(Vehicle vehicle)
        => vehicle.SetEnum(FlightComputerAttitudeTrackTarget.Prograde);
}

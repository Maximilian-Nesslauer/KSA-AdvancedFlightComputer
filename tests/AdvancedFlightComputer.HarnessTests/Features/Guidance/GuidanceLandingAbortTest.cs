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

// The production abort and hover paths, called the way the panel calls them, released through one
// ApplyAutopilot step. The engine figures the hover reads are answered for the fixture, because the
// save's engines are not lit.
public sealed class GuidanceLandingAbortTest : AfcTest
{
    public override string Name => "afc-guidance-landing-abort";

    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> Inputs =
        AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(
            AccessTools.Field(typeof(GuidanceWindow), "_s"));

    private static ref VehicleAutopilotState Ambient() => ref AmbientState();

    private static Vehicle? _fixture;
    private static double _thrustToWeight;
    private static int _hoverStarts;

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
        VehicleAutopilotState previousAmbient = Ambient();
        Vehicle? fixture = null;
        try
        {
            Orbit orbit = OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed);
            Vehicle vehicle;
            try
            {
                vehicle = VehicleSpawner.SpawnFromSave(
                    saves[0], t.System, home, "HarnessGuidanceLandingAbort", orbit);
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"'{saves[0]}': {e.Message}");
                return;
            }
            fixture = vehicle;

            AnAbortInTheAirLeavesTheEngine(t, vehicle, GuidanceWindow.LandingPhase.TerminalHover);
            AnAbortInTheAirLeavesTheEngine(t, vehicle, GuidanceWindow.LandingPhase.GfoldDescent);
            AnAbortInTheAirLeavesTheEngine(t, vehicle, GuidanceWindow.LandingPhase.TerminalCoast);
            AnAbortInTheAirLeavesTheEngine(t, vehicle, GuidanceWindow.LandingPhase.TerminalBrake);
            AnAbortOfTheBurnCutsTheEngine(t, vehicle, GuidanceWindow.LandingPhase.Burn);
            AnAbortOfTheBurnCutsTheEngine(t, vehicle, GuidanceWindow.LandingPhase.TransferPlanning);
            AnAbortOfTheBurnCutsTheEngine(t, vehicle, GuidanceWindow.LandingPhase.TransferCoast);
            AnUnownedAbortLeavesTheEngine(t, vehicle, GuidanceWindow.LandingPhase.DeorbitPlanning);
            AnUnownedAbortLeavesTheEngine(t, vehicle, GuidanceWindow.LandingPhase.DeorbitCoast);
            TheHoverRefusesAnEngineThatOutThrustsTheWeight(t, vehicle);
            TheDescentFliesOnWhenTheHoverRefuses(t, vehicle);
        }
        finally
        {
            _fixture = null;
            if (fixture != null)
            {
                VehicleControlOwnership.ReleaseAll(fixture);
                VehicleAutopilotState.Remove(fixture);
            }
            Ambient() = previousAmbient;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void AnAbortInTheAirLeavesTheEngine(
        TestContext t, Vehicle vehicle, GuidanceWindow.LandingPhase phase)
    {
        VehicleAutopilotState state = Landing(vehicle, phase);
        Ambient() = state;
        Method("AbortLanding").Invoke(null, Array.Empty<object>());
        t.Check($"{phase}: the abort ends the landing", state.LandingPhase == GuidanceWindow.LandingPhase.Done);
        t.Check($"{phase}: the abort asks for no cut", !state.LandingCutPending && state.ReleaseWithoutEngineCut);

        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check($"{phase}: the release gives the craft back",
            !state.ControlAcquired && VehicleControlOwnership.HolderOf(vehicle) != ControlClaimant.Guidance);
        t.Check($"{phase}: the engine command is left alone", Inputs(vehicle).EngineOn);
        t.Check($"{phase}: the throttle setting is left alone", Inputs(vehicle).EngineThrottle == 0.63f);
        t.Check($"{phase}: the status says so", state.LandingStatus.Contains("engine is left"), state.LandingStatus);
        t.Check($"{phase}: the finished release restores the default stop", !state.ReleaseWithoutEngineCut);
        VehicleAutopilotState.Remove(vehicle);
    }

    private static void AnAbortOfTheBurnCutsTheEngine(TestContext t, Vehicle vehicle, GuidanceWindow.LandingPhase phase)
    {
        VehicleAutopilotState state = Landing(vehicle, phase);
        Ambient() = state;
        Method("AbortLanding").Invoke(null, Array.Empty<object>());
        t.Check("a burn abort records its cut", state.LandingCutPending && !state.ReleaseWithoutEngineCut);

        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check("a burn abort gives the craft back", !state.ControlAcquired);
        t.Check("a burn abort shuts the engine down", !Inputs(vehicle).EngineOn);
        t.Check("a burn abort leaves no cut pending", !state.LandingCutPending);
        VehicleAutopilotState.Remove(vehicle);
    }

    private static void AnUnownedAbortLeavesTheEngine(TestContext t, Vehicle vehicle, GuidanceWindow.LandingPhase phase)
    {
        TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: true);
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        state.Engage = true;
        state.AutoStage = true;
        state.LandingPhase = phase;
        Ambient() = state;
        Method("AbortLanding").Invoke(null, Array.Empty<object>());
        GuidanceWindow.ApplyAutopilot(vehicle);
        t.Check($"{phase}: abort leaves an unowned engine command", Inputs(vehicle).EngineOn && Inputs(vehicle).EngineThrottle == 0.63f);
        t.Check($"{phase}: abort takes no control", !state.ControlAcquired && VehicleControlOwnership.HolderOf(vehicle) != ControlClaimant.Guidance);
        VehicleAutopilotState.Remove(vehicle);
    }

    private static void TheHoverRefusesAnEngineThatOutThrustsTheWeight(TestContext t, Vehicle vehicle)
    {
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        Ambient() = state;
        var harmony = new Harmony("com.maxi.afc.harnesstests.guidance.landingabort.hover");
        PatchEngineAnswers(harmony);
        try
        {
            _fixture = vehicle;
            _thrustToWeight = 1.5;
            Method("StartTerminalHover").Invoke(null, new object[] { vehicle });
            t.Check("an engine that out-thrusts the weight at minimum throttle is refused",
                state.LandingPhase == GuidanceWindow.LandingPhase.Idle, state.LandingStatus);
            t.Check("the refusal says why", state.LandingStatus.Contains("minimum throttle"), state.LandingStatus);
            t.Check("a refused hover claims nothing",
                !state.ControlAcquired && VehicleControlOwnership.HolderOf(vehicle) != ControlClaimant.Guidance);

            _thrustToWeight = 0.5;
            Method("StartTerminalHover").Invoke(null, new object[] { vehicle });
            t.Check("an engine that can descend engages the hover",
                state.LandingPhase == GuidanceWindow.LandingPhase.TerminalHover, state.LandingStatus);
        }
        finally
        {
            _fixture = null;
            harmony.UnpatchAll(harmony.Id);
            VehicleAutopilotState.Remove(vehicle);
        }
    }

    // The fixture orbits far above any pad and its body has an atmosphere, so the handoff height is
    // raised to reach it, the engine check is answered and the solve is skipped. The step under test
    // is the handoff decision, not the descent.
    private static void TheDescentFliesOnWhenTheHoverRefuses(TestContext t, Vehicle vehicle)
    {
        VehicleAutopilotState state = Landing(vehicle, GuidanceWindow.LandingPhase.GfoldDescent);
        state.GfoldHoverHandoffAltM = 1e9;
        Ambient() = state;
        Method("ResetGfoldTrace").Invoke(null, Array.Empty<object>());

        var harmony = new Harmony("com.maxi.afc.harnesstests.guidance.landingabort.handoff");
        PatchEngineAnswers(harmony);
        harmony.Patch(Method("PrepareLandingEngines"),
            prefix: new HarmonyMethod(typeof(GuidanceLandingAbortTest), nameof(ForceEnginesReady)));
        harmony.Patch(Method("SolveGfoldPlan"),
            prefix: new HarmonyMethod(typeof(GuidanceLandingAbortTest), nameof(SkipSolve)));
        harmony.Patch(Method("StartTerminalHover"),
            postfix: new HarmonyMethod(typeof(GuidanceLandingAbortTest), nameof(CountHoverStart)));
        try
        {
            _fixture = vehicle;
            _thrustToWeight = 1.5;
            _hoverStarts = 0;
            Orbit orbit = vehicle.Orbit;
            IParentBody parent = orbit.Parent;
            object[] args = { vehicle, orbit, parent, parent.MeanRadius, 0.0 };

            Method("StepGfoldDescent").Invoke(null, args);
            t.Check("the handoff asked the hover once", _hoverStarts == 1);
            t.Check("a refused handoff keeps the descent",
                state.LandingPhase == GuidanceWindow.LandingPhase.GfoldDescent, state.LandingStatus);
            t.Check("the refusal is remembered", state.GfoldHoverRefused);
            t.Check("the status says the plan flies on", state.LandingStatus.Contains("flies its plan"), state.LandingStatus);
            t.Check("no cut is queued", !state.LandingCutPending);

            Method("StepGfoldDescent").Invoke(null, args);
            t.Check("the hover is not asked again", _hoverStarts == 1);
            t.Check("the descent keeps its phase on the next step",
                state.LandingPhase == GuidanceWindow.LandingPhase.GfoldDescent);
        }
        finally
        {
            _fixture = null;
            harmony.UnpatchAll(harmony.Id);
            VehicleControlOwnership.ReleaseAll(vehicle);
            VehicleAutopilotState.Remove(vehicle);
        }
    }

    private static void PatchEngineAnswers(Harmony harmony)
    {
        harmony.Patch(AccessTools.Method(typeof(KsaEnginePerf), nameof(KsaEnginePerf.SupportsThrottleControl)),
            prefix: new HarmonyMethod(typeof(GuidanceLandingAbortTest), nameof(ForceThrottleControl)));
        harmony.Patch(AccessTools.Method(typeof(KsaEnginePerf), nameof(KsaEnginePerf.ThrustAtThrottle)),
            prefix: new HarmonyMethod(typeof(GuidanceLandingAbortTest), nameof(AnswerThrust)));
    }

    private static bool ForceThrottleControl(Vehicle vehicle, ref bool __result)
    {
        if (!ReferenceEquals(vehicle, _fixture))
            return true;
        __result = true;
        return false;
    }

    // The stubbed engine delivers the chosen multiple of the craft's weight at every throttle.
    private static bool AnswerThrust(Vehicle vehicle, ref double __result)
    {
        if (!ReferenceEquals(vehicle, _fixture))
            return true;
        Orbit orbit = vehicle.Orbit;
        double r = orbit.StateVectors.PositionCci.Length();
        __result = _thrustToWeight * vehicle.TotalMass * orbit.Parent.Mu / (r * r);
        return false;
    }

    private static bool ForceEnginesReady(Vehicle vehicle, ref bool __result)
    {
        if (!ReferenceEquals(vehicle, _fixture))
            return true;
        __result = true;
        return false;
    }

    private static bool SkipSolve(Vehicle vehicle) => !ReferenceEquals(vehicle, _fixture);

    private static void CountHoverStart(Vehicle vehicle)
    {
        if (ReferenceEquals(vehicle, _fixture))
            _hoverStarts++;
    }

    // A landing in the given phase that only an abort can end.
    private static VehicleAutopilotState Landing(Vehicle vehicle, GuidanceWindow.LandingPhase phase)
    {
        TestSupport.SetManualControlInputs(vehicle, 0.63f, engineOn: true);
        vehicle.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        state.Engage = true;
        state.AutoStage = true;
        state.LandingPhase = phase;
        state.ControlAcquired = true;
        VehicleControlOwnership.TryClaim(vehicle, ControlClaimant.Guidance, out _);

        FlightComputer computer = vehicle.FlightComputer;
        state.AttitudeOwnership.BeginWrite(computer);
        computer.CustomAttitudeTarget = new double3(1, 2, 3);
        computer.AttitudeFrame = VehicleReferenceFrame.EclBody;
        computer.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
        state.AttitudeOwnership.EndWrite(computer, writesRoll: false);
        return state;
    }

    private static System.Reflection.MethodInfo Method(string name)
        => AccessTools.Method(typeof(GuidanceWindow), name)
           ?? throw new MissingMethodException(nameof(GuidanceWindow), name);
}

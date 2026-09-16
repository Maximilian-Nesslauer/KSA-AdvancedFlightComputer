using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Steps a deorbit burn at its handoff through the real ApplyAutopilot on a spawned craft, once for
// each descent solver. The UPFG solve and the staging cue are stubbed for the fixture, so the burn
// reads a converged solution inside the handoff gate, and the 6-DOF dispatch is counted instead of
// solved. The step under test is the handoff, not either descent.
public sealed class GuidanceLandingHandoffTest : AfcTest
{
    public override string Name => "afc-guidance-landing-handoff";

    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;

    // What the burn wrote on its last step, and what UPFG commands on the handoff step. Both are
    // exact in float, so the engine input can be compared for equality.
    private const float BurnThrottle = 0.625f;
    private const double HandoffThrottle = 0.75;

    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> Inputs =
        AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

    private static Vehicle? _fixture;
    private static int _sixDofSteps;

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

        FieldInfo ambient = typeof(GuidanceWindow).GetField("_s", PrivateStatic)!;
        object? previousAmbient = ambient.GetValue(null);
        Vehicle? previousFocus = Program.ControlledVehicle;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        var harmony = new Harmony("com.maxi.afc.harnesstests.guidance.handoff");
        Vehicle? craft = null;
        try
        {
            try
            {
                craft = VehicleSpawner.SpawnFromSave(saves[0], t.System, home, "HarnessGuidanceHandoff",
                    OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed));
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"'{saves[0]}': {e.Message}");
                return;
            }
            _fixture = craft;
            Program.ControlledVehicle = craft;

            harmony.Patch(Method("BuildUpfgVehicle"), prefix: Prefix(nameof(NotForFixture)));
            harmony.Patch(Method("AutoSequence"), prefix: Prefix(nameof(NotForFixture)));
            harmony.Patch(Method("Step6Dof"), prefix: Prefix(nameof(CountSixDofEngage)));

            TheBurnHandsOverToSixDof(t, craft);
            TheBurnHandsOverToGfold(t, craft);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            _fixture = null;
            _sixDofSteps = 0;
            if (craft != null)
            {
                StagingDetector.Arm(craft, false);
                VehicleControlOwnership.ReleaseAll(craft);
                VehicleAutopilotState.Remove(craft);
            }
            Program.ControlledVehicle = previousFocus;
            ambient.SetValue(null, previousAmbient);
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    // The burn queues the engage and lets the next step's 6-DOF dispatch run it, so the step that
    // queues it must neither release the craft nor cut the engine the burn left lit.
    private static void TheBurnHandsOverToSixDof(TestContext t, Vehicle craft)
    {
        VehicleAutopilotState state = BurnAtHandoff(craft, sixDof: true);
        _sixDofSteps = 0;

        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("6-DOF: the handoff ends the burn",
            state.LandingPhase == GuidanceWindow.LandingPhase.Idle, state.LandingStatus);
        t.Check("6-DOF: the engage is still queued when the step ends", state.EngagePending);
        t.Check("6-DOF: the handoff keeps the craft",
            state.ControlAcquired && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.Guidance);
        t.Check("6-DOF: the engine keeps the burn's command",
            Inputs(craft).EngineOn && Inputs(craft).EngineThrottle == BurnThrottle,
            $"engine on {Inputs(craft).EngineOn}, throttle {Inputs(craft).EngineThrottle}");
        t.Check("6-DOF: the attitude is given back for the allocator", !state.WasEngaged);
        t.Check("6-DOF: AutoStage stays armed through the handoff",
            StagingDetector.IsArmed(craft) && state.ArmedStaging);

        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("6-DOF: the next step dispatches the queued engage", _sixDofSteps == 1, $"{_sixDofSteps} dispatches");
        t.Check("6-DOF: the engaged descent keeps the craft",
            state.Active && state.ControlAcquired
            && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.Guidance);
        t.Check("6-DOF: the engine is still lit when the engage runs", Inputs(craft).EngineOn);
        t.Check("6-DOF: AutoStage is still armed when the engage runs",
            StagingDetector.IsArmed(craft) && state.ArmedStaging);

        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);
    }

    // The handoff step already writes the descent's engine command, before G-FOLD has a plan, so it
    // must carry the burn's throttle rather than switch the engine off for a step.
    private static void TheBurnHandsOverToGfold(TestContext t, Vehicle craft)
    {
        VehicleAutopilotState state = BurnAtHandoff(craft, sixDof: false);

        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("G-FOLD: the handoff starts the descent",
            state.LandingPhase == GuidanceWindow.LandingPhase.GfoldDescent, state.LandingStatus);
        t.Check("G-FOLD: the handoff keeps the craft",
            state.ControlAcquired && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.Guidance);
        t.Check("G-FOLD: the engine stays lit through the handoff step",
            Inputs(craft).EngineOn && Inputs(craft).EngineThrottle == (float)HandoffThrottle,
            $"engine on {Inputs(craft).EngineOn}, throttle {Inputs(craft).EngineThrottle}");
        t.Check("G-FOLD: the descent starts without a plan or a tracker history",
            state.GfoldPlan == null && !state.GfoldTrackInit);
        t.Check("G-FOLD: AutoStage stays armed through the handoff",
            StagingDetector.IsArmed(craft) && state.ArmedStaging);

        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);
    }

    // A deorbit burn on the step it reaches the handoff: guidance owns and steers the craft, the
    // engine is lit at the throttle the burn wrote last, guidance armed AutoStage for the flight, and
    // UPFG reads converged inside the gate.
    private static VehicleAutopilotState BurnAtHandoff(Vehicle craft, bool sixDof)
    {
        TestSupport.SetManualControlInputs(craft, BurnThrottle, engineOn: true);
        craft.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
        StagingDetector.Arm(craft, true);

        VehicleAutopilotState state = VehicleAutopilotState.For(craft);
        state.StagingArmChecked = true;
        state.ArmedStaging = true;
        state.Engage = true;
        state.AutoStage = true;
        state.UseSixDofLanding = sixDof;
        state.LandingPhase = GuidanceWindow.LandingPhase.Burn;
        state.ControlAcquired = true;
        VehicleControlOwnership.TryClaim(craft, ControlClaimant.Guidance, out _);

        FlightComputer computer = craft.FlightComputer;
        state.AttitudeOwnership.BeginWrite(computer);
        computer.CustomAttitudeTarget = new double3(1, 2, 3);
        computer.AttitudeFrame = VehicleReferenceFrame.EclBody;
        computer.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
        state.AttitudeOwnership.EndWrite(computer, writesRoll: false);
        state.CommandDir = -craft.Orbit.StateVectors.VelocityCci.Normalized();
        state.HasCommand = true;
        state.WasEngaged = true;

        SetSolution(state.Upfg, tgo: state.GfoldHandoffTgo - 1.0, throttle: HandoffThrottle);
        return state;
    }

    // UPFG sets these only from a solve, and the stubbed model means no solve runs.
    private static void SetSolution(UpfgGuidance upfg, double tgo, double throttle)
    {
        AccessTools.PropertySetter(typeof(UpfgGuidance), nameof(UpfgGuidance.Converged))
            .Invoke(upfg, new object[] { true });
        AccessTools.PropertySetter(typeof(UpfgGuidance), nameof(UpfgGuidance.Throttle))
            .Invoke(upfg, new object[] { throttle });
        object solution = AccessTools.Field(typeof(UpfgGuidance), "_prev").GetValue(upfg)!;
        AccessTools.Field(solution.GetType(), "Tgo").SetValue(solution, tgo);
    }

    private static MethodInfo Method(string name) =>
        typeof(GuidanceWindow).GetMethod(name, PrivateStatic)
        ?? throw new MissingMethodException(nameof(GuidanceWindow), name);

    private static HarmonyMethod Prefix(string name) => new(typeof(GuidanceLandingHandoffTest), name);

    private static bool NotForFixture(Vehicle vehicle) => !ReferenceEquals(vehicle, _fixture);

    // Stands in for an engage that took: the dispatch is counted and the request consumed, so the
    // step reads a 6-DOF that is flying without building a problem or starting a solve.
    private static bool CountSixDofEngage(Vehicle vehicle)
    {
        if (!ReferenceEquals(vehicle, _fixture))
            return true;
        _sixDofSteps++;
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        if (state.EngagePending)
        {
            state.EngagePending = false;
            state.Active = true;
        }
        return false;
    }
}

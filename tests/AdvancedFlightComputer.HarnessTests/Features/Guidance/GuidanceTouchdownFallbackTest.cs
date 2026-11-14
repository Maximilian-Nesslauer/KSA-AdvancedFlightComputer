using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// What a landing does with a craft that touchdown guidance has no plan for, on a spawned craft that
// falls toward the ground. Touchdown guidance is the only descent solver the player gets, so G-FOLD
// takes such a craft when it can fly it, and the landing says no when it cannot. Three parts:
//  - a queued touchdown engage whose 6-DOF configuration is refused, stepped through the real
//    ApplyAutopilot, for a lit burn that entry guidance hands over and for LAND NOW with the engine off;
//  - a fault of touchdown guidance in the air, through the rule that keeps the craft;
//  - DEORBIT + LAND, which decides the solver or refuses before anything burns.
// Whether 6-DOF can configure, whether G-FOLD can fly the craft and whether its engines can be
// throttled are stubbed, so the result does not depend on the gimbals and engines of the fixture.
// No descent is flown.
public sealed class GuidanceTouchdownFallbackTest : AfcTest
{
    public override string Name => "afc-guidance-touchdown-fallback";

    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;

    // Exact in float, so the engine input can be compared for equality.
    private const float BurnThrottle = 0.625f;

    private const string Refusal = "no gimballed engine - touchdown guidance needs thrust vectoring";
    private const string GfoldRefusal = "G-FOLD needs a gimballed engine to hold the thrust direction.";

    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> Inputs =
        AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

    private static Vehicle? _fixture;
    private static bool _gfoldCanFly;
    private static bool _sixDofCanTake;

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
        var harmony = new Harmony("com.maxi.afc.harnesstests.guidance.touchdownfallback");
        Vehicle? craft = null;
        try
        {
            // 500 km up with the horizontal speed cut by 1 km/s: bound, the periapsis under the
            // surface and no radial speed, which the console reads as a fall.
            Orbit circular = OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed);
            StateVectors sv = circular.GetStateVectorsAt(driver.Elapsed);
            Orbit falling = OrbitFixtures.ApplyImpulse(circular, -sv.VelocityCci.Normalized() * 1000.0, driver.Elapsed);
            try
            {
                craft = VehicleSpawner.SpawnFromSave(saves[0], t.System, home, "HarnessTouchdownFallback", falling);
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"'{saves[0]}': {e.Message}");
                return;
            }
            _fixture = craft;
            Program.ControlledVehicle = craft;

            Type enginePerf = typeof(GuidanceWindow).Assembly.GetType("AdvancedFlightComputer.Features.Guidance.KsaEnginePerf", true)!;
            harmony.Patch(Method("BuildUpfgVehicle"), prefix: Prefix(nameof(NotForFixture)));
            harmony.Patch(Method("AutoSequence"), prefix: Prefix(nameof(NotForFixture)));
            harmony.Patch(Method("TryConfigure6Dof"), prefix: Prefix(nameof(RefuseConfigure)));
            harmony.Patch(Method("Can6DofTake"), prefix: Prefix(nameof(AnswerCanSixDofTake)));
            harmony.Patch(Method("GfoldCanFly"), prefix: Prefix(nameof(AnswerGfoldCanFly)));
            // The fixture's engines are not activated, so the check that they can be throttled is answered.
            harmony.Patch(AccessTools.Method(enginePerf, "SupportsThrottleControl"), prefix: Prefix(nameof(FixtureHasEngines)));

            TheEntryBurnGoesToGfoldLit(t, craft);
            LandNowGoesToGfoldUnlit(t, craft);
            ARefusedBurnKeepsItsEngine(t, craft);
            ARefusedLandNowTakesNothing(t, craft);
            AFaultInTheAirKeepsTheCraft(t, craft, ambient);
            DeorbitDecidesBeforeTheBurn(t, craft, ambient);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            _fixture = null;
            if (craft != null)
            {
                VehicleControlOwnership.ReleaseAll(craft);
                VehicleAutopilotState.Remove(craft);
            }
            Program.ControlledVehicle = previousFocus;
            ambient.SetValue(null, previousAmbient);
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    // Entry guidance hands its lit burn to touchdown guidance as a queued engage. A refused engage
    // must not release the craft with the engine cut in the air.
    private static void TheEntryBurnGoesToGfoldLit(TestContext t, Vehicle craft)
    {
        VehicleAutopilotState state = QueuedEngage(craft, burning: true);
        _gfoldCanFly = true;

        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("burn: the refused engage starts the G-FOLD descent",
            state.LandingPhase == GuidanceWindow.LandingPhase.GfoldDescent, state.LandingStatus);
        t.Check("burn: no 6-DOF stays engaged or queued", !state.Active && !state.EngagePending);
        t.Check("burn: the craft stays held",
            state.ControlAcquired && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.Guidance);
        t.Check("burn: no engine cut is queued", !state.LandingCutPending);
        t.Check("burn: the engine stays lit at the burn's throttle",
            Inputs(craft).EngineOn && Inputs(craft).EngineThrottle == BurnThrottle,
            $"engine on {Inputs(craft).EngineOn}, throttle {Inputs(craft).EngineThrottle}");
        t.Check("burn: G-FOLD starts from the lit engine and its throttle",
            state.GfoldEngineOn && state.GfoldThrottle == BurnThrottle, $"on {state.GfoldEngineOn}, throttle {state.GfoldThrottle}");
        t.Check("burn: the solver switch follows the descent that flies", !state.UseSixDofLanding);
        t.Check("burn: the status names the refusal", state.LandingStatus.Contains(Refusal), state.LandingStatus);

        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);
    }

    // TOUCHDOWN from LAND NOW with the engine off: G-FOLD takes the craft with the engine off too.
    private static void LandNowGoesToGfoldUnlit(TestContext t, Vehicle craft)
    {
        VehicleAutopilotState state = QueuedEngage(craft, burning: false);
        _gfoldCanFly = true;

        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("land now: the refused engage starts the G-FOLD descent",
            state.LandingPhase == GuidanceWindow.LandingPhase.GfoldDescent, state.LandingStatus);
        t.Check("land now: the craft is claimed for the descent",
            VehicleControlOwnership.HolderOf(craft) == ControlClaimant.Guidance);
        t.Check("land now: G-FOLD starts with the engine off", !state.GfoldEngineOn && !Inputs(craft).EngineOn);

        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);
    }

    // A lit burn handed to a touchdown guidance that refuses the craft, which G-FOLD cannot fly either.
    // Nothing can fly it, so the craft goes back to the player with the engine as it is.
    private static void ARefusedBurnKeepsItsEngine(TestContext t, Vehicle craft)
    {
        VehicleAutopilotState state = QueuedEngage(craft, burning: true);
        _gfoldCanFly = false;

        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("burn, no fallback: no descent starts",
            state.LandingPhase == GuidanceWindow.LandingPhase.Idle && !state.Active && !state.EngagePending, state.LandingStatus);
        t.Check("burn, no fallback: the craft is released",
            !state.ControlAcquired && VehicleControlOwnership.HolderOf(craft) != ControlClaimant.Guidance);
        t.Check("burn, no fallback: the engine is not cut",
            Inputs(craft).EngineOn && Inputs(craft).EngineThrottle == BurnThrottle,
            $"engine on {Inputs(craft).EngineOn}, throttle {Inputs(craft).EngineThrottle}");
        t.Check("burn, no fallback: the status names the refusal and the engine",
            state.Error.Contains(Refusal) && state.Error.Contains("ENGINE UNCHANGED"), state.Error);

        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);
    }

    // TOUCHDOWN from LAND NOW for a craft that neither solver can fly is a refusal that takes nothing.
    private static void ARefusedLandNowTakesNothing(TestContext t, Vehicle craft)
    {
        VehicleAutopilotState state = QueuedEngage(craft, burning: false);
        _gfoldCanFly = false;

        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("land now, no fallback: no descent starts",
            state.LandingPhase == GuidanceWindow.LandingPhase.Idle && !state.Active && !state.EngagePending, state.LandingStatus);
        t.Check("land now, no fallback: the craft is not held",
            !state.ControlAcquired && VehicleControlOwnership.HolderOf(craft) != ControlClaimant.Guidance);
        t.Check("land now, no fallback: the engine stays off", !Inputs(craft).EngineOn);
        t.Check("land now, no fallback: the status names the refusal", state.Error.Contains(Refusal), state.Error);

        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);
    }

    // A fault of touchdown guidance while the craft is in the air. G-FOLD takes the craft when it can
    // fly it. Otherwise touchdown guidance engages again and flies its no-plan law, and a second
    // fault at once gives the craft back with the engine as it is. No path cuts the engine.
    private static void AFaultInTheAirKeepsTheCraft(TestContext t, Vehicle craft, FieldInfo ambient)
    {
        MethodInfo restart = Method("RestartAfterFault");

        VehicleAutopilotState state = FlyingTouchdown(craft, ambient);
        _gfoldCanFly = true;
        bool kept = (bool)restart.Invoke(null, new object[] { craft, "TOUCHDOWN STOPPED: SOLVER FAULT", "WARM SOLVE FAILED: test" })!;
        t.Check("fault, G-FOLD can fly: G-FOLD takes the craft",
            kept && state.LandingPhase == GuidanceWindow.LandingPhase.GfoldDescent && !state.Active, state.LandingStatus);
        t.Check("fault, G-FOLD can fly: the engine stays lit and no cut is queued",
            state.GfoldEngineOn && state.GfoldThrottle == BurnThrottle && !state.LandingCutPending);
        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);

        // A descent that touchdown guidance lit itself, so a refused restart must still read the engine as lit.
        state = FlyingTouchdown(craft, ambient);
        state.SixDofTookBurn = false;
        _gfoldCanFly = false;
        kept = (bool)restart.Invoke(null, new object[] { craft, "TOUCHDOWN STOPPED: SOLVER FAULT", "WARM SOLVE FAILED: test" })!;
        t.Check("fault, no fallback: touchdown guidance engages again",
            kept && state.EngagePending && !state.Active && state.SixDofNoPlanLaw, $"pending {state.EngagePending}, law {state.SixDofNoPlanLaw}");
        t.Check("fault, no fallback: the new engage counts the lit engine as a burn it took", state.SixDofTookBurn);
        t.Check("fault, no fallback: no engine cut is queued and the engine input stays",
            !state.LandingCutPending && Inputs(craft).EngineOn && Inputs(craft).EngineThrottle == BurnThrottle);

        state.EngagePending = false;
        state.Active = true;
        kept = (bool)restart.Invoke(null, new object[] { craft, "TOUCHDOWN STOPPED: SOLVER FAULT", "WARM SOLVE FAILED: test" })!;
        t.Check("second fault at once: guidance ends without a cut",
            kept && !state.Active && !state.EngagePending && state.ReleaseWithoutEngineCut && !state.LandingCutPending);
        t.Check("second fault at once: the status says the engine is unchanged", state.Error.Contains("ENGINE UNCHANGED"), state.Error);
        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);
    }

    // DEORBIT + LAND decides the solver before the deorbit burn: touchdown guidance, G-FOLD for a
    // craft it cannot plan for, and a refusal with both reasons when G-FOLD cannot fly it either.
    private static void DeorbitDecidesBeforeTheBurn(TestContext t, Vehicle craft, FieldInfo ambient)
    {
        MethodInfo execute = Method("ExecuteLanding");

        VehicleAutopilotState state = ReadyToCommit(craft, ambient);
        _sixDofCanTake = false;
        _gfoldCanFly = false;
        execute.Invoke(null, new object[] { craft });
        t.Check("deorbit, no solver: the landing is refused before it plans",
            state.LandingPhase == GuidanceWindow.LandingPhase.Idle && VehicleControlOwnership.HolderOf(craft) != ControlClaimant.Guidance, state.LandingStatus);
        t.Check("deorbit, no solver: the refusal names both reasons",
            state.LandingStatus.StartsWith("NO LANDING GUIDANCE CAN FLY THIS CRAFT") && state.LandingStatus.Contains(Refusal) && state.LandingStatus.Contains(GfoldRefusal.TrimEnd('.')),
            state.LandingStatus);
        VehicleAutopilotState.Remove(craft);

        state = ReadyToCommit(craft, ambient);
        _gfoldCanFly = true;
        execute.Invoke(null, new object[] { craft });
        t.Check("deorbit, G-FOLD can fly: the landing plans with G-FOLD",
            state.LandingPhase == GuidanceWindow.LandingPhase.DeorbitPlanning && !state.UseSixDofLanding, state.LandingStatus);
        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);

        state = ReadyToCommit(craft, ambient);
        state.UseSixDofLanding = false;
        _sixDofCanTake = true;
        _gfoldCanFly = false;
        execute.Invoke(null, new object[] { craft });
        t.Check("deorbit, touchdown can plan: the landing plans with touchdown guidance",
            state.LandingPhase == GuidanceWindow.LandingPhase.DeorbitPlanning && state.UseSixDofLanding, state.LandingStatus);
        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);
    }

    // The state Engage6Dof leaves: the request queued for the next step. A burn handed over holds the
    // craft with the engine lit and the attitude written, as entry guidance left them.
    private static VehicleAutopilotState QueuedEngage(Vehicle craft, bool burning)
    {
        TestSupport.SetManualControlInputs(craft, burning ? BurnThrottle : 0f, engineOn: burning);
        craft.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;

        VehicleAutopilotState state = VehicleAutopilotState.For(craft);
        state.StagingArmChecked = true;
        state.Engage = true;
        state.AutoStage = true;
        state.UseSixDofLanding = true;
        state.EngagePending = true;
        state.SixDofTookBurn = burning;
        if (burning)
        {
            state.ControlAcquired = true;
            VehicleControlOwnership.TryClaim(craft, ControlClaimant.Guidance, out _);
            FlightComputer computer = craft.FlightComputer;
            state.AttitudeOwnership.BeginWrite(computer);
            computer.CustomAttitudeTarget = new double3(1, 2, 3);
            computer.AttitudeFrame = VehicleReferenceFrame.EclBody;
            computer.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
            state.AttitudeOwnership.EndWrite(computer, writesRoll: false);
            state.WasEngaged = true;
        }
        return state;
    }

    // Touchdown guidance flying a lit burn, as the ambient state of the guidance step.
    private static VehicleAutopilotState FlyingTouchdown(Vehicle craft, FieldInfo ambient)
    {
        TestSupport.SetManualControlInputs(craft, BurnThrottle, engineOn: true);
        craft.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
        VehicleAutopilotState state = VehicleAutopilotState.For(craft);
        state.StagingArmChecked = true;
        state.Engage = true;
        state.AutoStage = true;
        state.Active = true;
        state.SixDofTookBurn = true;
        state.ControlAcquired = true;
        VehicleControlOwnership.TryClaim(craft, ControlClaimant.Guidance, out _);
        ambient.SetValue(null, state);
        return state;
    }

    // A craft with both control switches on and nothing flying, as the ambient state of a commit.
    private static VehicleAutopilotState ReadyToCommit(Vehicle craft, FieldInfo ambient)
    {
        TestSupport.SetManualControlInputs(craft, 0f, engineOn: false);
        VehicleAutopilotState state = VehicleAutopilotState.For(craft);
        state.StagingArmChecked = true;
        state.Engage = true;
        state.AutoStage = true;
        ambient.SetValue(null, state);
        return state;
    }

    private static MethodInfo Method(string name) =>
        typeof(GuidanceWindow).GetMethod(name, PrivateStatic)
        ?? throw new MissingMethodException(nameof(GuidanceWindow), name);

    private static HarmonyMethod Prefix(string name) => new(typeof(GuidanceTouchdownFallbackTest), name);

    private static bool NotForFixture(Vehicle vehicle) => !ReferenceEquals(vehicle, _fixture);

    private static bool RefuseConfigure(Vehicle vehicle, ref string error, ref bool __result)
    {
        if (!ReferenceEquals(vehicle, _fixture))
            return true;
        error = Refusal;
        __result = false;
        return false;
    }

    private static bool AnswerCanSixDofTake(Vehicle vehicle, ref string error, ref bool __result)
    {
        if (!ReferenceEquals(vehicle, _fixture))
            return true;
        error = _sixDofCanTake ? "" : Refusal;
        __result = _sixDofCanTake;
        return false;
    }

    private static bool AnswerGfoldCanFly(Vehicle vehicle, ref string why, ref bool __result)
    {
        if (!ReferenceEquals(vehicle, _fixture))
            return true;
        why = _gfoldCanFly ? "" : GfoldRefusal;
        __result = _gfoldCanFly;
        return false;
    }

    private static bool FixtureHasEngines(Vehicle vehicle, ref bool __result)
    {
        if (!ReferenceEquals(vehicle, _fixture))
            return true;
        __result = true;
        return false;
    }
}

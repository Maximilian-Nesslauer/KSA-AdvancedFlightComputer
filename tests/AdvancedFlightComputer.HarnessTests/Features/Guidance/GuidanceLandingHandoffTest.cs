using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Steps a deorbit burn at its handoff through the real ApplyAutopilot on a spawned craft, once for
// each descent solver, once for a craft 6-DOF cannot plan for, and once for a standby that has no
// plan in the last seconds of the burn. The UPFG solve and the staging cue are stubbed for the
// fixture, so the burn reads a converged solution inside the handoff gate; whether 6-DOF can take
// the craft, whether its standby has a plan and whether G-FOLD can fly the craft are set per case
// rather than read off the fixture's hardware; and the 6-DOF dispatch is counted instead of solved.
// The step under test is the handoff, not either descent.
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

    // What the stubbed check says when 6-DOF cannot take the craft.
    private const string Refusal = "no gimballed engine - touchdown guidance needs thrust vectoring";

    private static Vehicle? _fixture;
    private static int _sixDofSteps;
    private static bool _sixDofCanTake;
    private static bool _standbyReady;
    private static bool _gfoldCanFly;
    private static bool _contact;

    protected override void Execute(TestContext t)
    {
        CheckStandbyAttitude(t);
        t.Check("entry retains a published ignition plan seven seconds before the burn",
            !GuidanceWindow.CanRestartEntryGlide(7.0, 4.0));
        t.Check("entry can restart when there is time for a cold retry before ignition lock",
            GuidanceWindow.CanRestartEntryGlide(13.0, 4.0));
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
            // The fixture writes the burn's engine command without an active engine, so the engine wait is stubbed with the staging it depends on.
            harmony.Patch(Method("PrepareLandingEngines"), prefix: Prefix(nameof(EnginesReadyForFixture)));
            harmony.Patch(Method("Step6Dof"), prefix: Prefix(nameof(CountSixDofEngage)));
            harmony.Patch(Method("Can6DofTake"), prefix: Prefix(nameof(AnswerCanSixDofTake)));
            // The standby would build and solve a 6-DOF problem for the fixture, so it is started, stepped and asked by stubs.
            harmony.Patch(Method("StartSixDofStandby"), prefix: Prefix(nameof(StartStandbyForFixture)));
            harmony.Patch(Method("StepSixDofStandby"), prefix: Prefix(nameof(NotForFixture)));
            harmony.Patch(Method("SixDofStandbyReady"), prefix: Prefix(nameof(AnswerStandbyReady)));
            harmony.Patch(Method("GfoldCanFly"), prefix: Prefix(nameof(AnswerGfoldCanFly)));
            harmony.Patch(Method("HasTouchedDown"), prefix: Prefix(nameof(AnswerContact)));

            TheBurnHandsOverToSixDof(t, craft);
            TheBurnHandsOverToGfold(t, craft);
            ACraftSixDofRefusesGoesToGfold(t, craft);
            AStandbyWithNoPlanGoesToGfold(t, craft);
            ContactCancelsHandoff(t, craft);
            EntryFaultWaitsForAReadyPlan(t, craft, home, ambient);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            _fixture = null;
            _sixDofSteps = 0;
            _sixDofCanTake = false;
            _standbyReady = false;
            _gfoldCanFly = false;
            _contact = false;
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

    private static bool AnswerContact(Vehicle vehicle, ref bool __result)
    {
        if (!ReferenceEquals(vehicle, _fixture))
            return true;
        __result = _contact;
        return false;
    }

    private static void ContactCancelsHandoff(TestContext t, Vehicle craft)
    {
        VehicleAutopilotState state = BurnAtHandoff(craft, sixDof: true);
        _sixDofSteps = 0;
        _sixDofCanTake = true;
        _standbyReady = true;
        GuidanceWindow.ApplyAutopilot(craft);
        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("the airborne vehicle queued a standby handoff", state.EngagePending && state.LandingTouchdownArmed);

        _contact = true;
        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("ground contact cancels the pending handoff and cuts the engine",
            !state.EngagePending && _sixDofSteps == 0 && !Inputs(craft).EngineOn);
        _contact = false;
        state.SixDofStandby = false;
        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);
    }

    private static void EntryFaultWaitsForAReadyPlan(TestContext t, Vehicle craft, IParentBody home, FieldInfo ambient)
    {
        VehicleAutopilotState state = VehicleAutopilotState.For(craft);
        ambient.SetValue(null, state);
        state.ThreeDofPhase = GuidanceWindow.ThreeDofPhase.Burn;
        state.ThreeDofEngineOn = true;
        state.ThreeDofThrottle = BurnThrottle;
        state.SixDofStandby = true;
        state.ThreeDofBurnSolveFault = true;
        Inputs(craft).EngineOn = true;
        Inputs(craft).EngineThrottle = BurnThrottle;
        object[] args = [craft, home, double3.UnitZ * home.MeanRadius, new double[Dynamics6Dof.NX],
            Universe.GetElapsedSeconds()];
        MethodInfo step = Method("StepEntryTouchdownHandoff");
        MethodInfo take = Method("CheckPendingEntryTouchdownHandoff");

        _standbyReady = false;
        step.Invoke(null, args);
        t.Check("an entry solve fault keeps the lit burn while touchdown guidance is cold",
            state.ThreeDofTouchdownHandoff.Status == null && !state.EngagePending
            && state.ThreeDofPhase == GuidanceWindow.ThreeDofPhase.Burn
            && state.ThreeDofEngineOn && state.ThreeDofThrottle == BurnThrottle);

        _standbyReady = true;
        step.Invoke(null, args);
        t.Check("a retained fault requests handover after the standby becomes ready",
            state.ThreeDofTouchdownHandoff.Status != null && !state.EngagePending
            && state.ThreeDofPhase == GuidanceWindow.ThreeDofPhase.Burn);
        _standbyReady = false;
        take.Invoke(null, [craft, home]);
        t.Check("a changed standby leaves the entry burn in control on the command step",
            state.ThreeDofTouchdownHandoff.Status == null && !state.EngagePending
            && state.ThreeDofPhase == GuidanceWindow.ThreeDofPhase.Burn && Inputs(craft).EngineOn);

        _standbyReady = true;
        step.Invoke(null, args);
        _contact = true;
        take.Invoke(null, [craft, home]);
        t.Check("contact cancels the entry handover before touchdown guidance can command",
            state.ThreeDofTouchdownHandoff.Status == null && !state.EngagePending);

        _contact = false;
        step.Invoke(null, args);
        var model = new KsaPointMassModel();
        var guidance = new Guidance3Dof(new Scvx3DofConfig { Nodes = 3, GlideIntervals = 1 },
            new Scvx3DofConfig { Nodes = 3, GlideIntervals = 0, AttitudeAnchor = true }, model);
        double now = Universe.GetElapsedSeconds();
        var recovered = new Plan3Dof([], [], 3, 0, 0.0, 20.0, now, now, Phase3Dof.Burn, 0.0);
        AccessTools.Field(typeof(Guidance3Dof), "_published").SetValue(guidance, recovered);
        state.ThreeDofGuidance = guidance;
        take.Invoke(null, [craft, home]);
        t.Check("a plan published during a pending fault handoff keeps entry in control",
            state.ThreeDofTouchdownHandoff.Status == null && !state.EngagePending
            && state.ThreeDofPhase == GuidanceWindow.ThreeDofPhase.Burn);
        state.ThreeDofGuidance = null;

        step.Invoke(null, args);
        take.Invoke(null, [craft, home]);
        t.Check("a ready standby takes the entry burn without cutting its engine",
            state.EngagePending && state.SixDofStandby
            && state.ThreeDofPhase == GuidanceWindow.ThreeDofPhase.Idle
            && Inputs(craft).EngineOn && Inputs(craft).EngineThrottle == BurnThrottle);
        _standbyReady = false;
        state.SixDofStandby = false;
        VehicleAutopilotState.Remove(craft);
    }

    private static void CheckStandbyAttitude(TestContext t)
    {
        int stride = Dynamics6Dof.NX;
        int q = Dynamics6Dof.IQ;
        int w = Dynamics6Dof.IW;
        var nodes = new double[2 * stride];
        nodes[q] = 1.0;
        nodes[stride + q] = -Math.Cos(Math.PI / 18.0);
        nodes[stride + q + 1] = -Math.Sin(Math.PI / 18.0);
        nodes[stride + w] = 0.2;
        var plan = new Ksa6DofPlan(nodes, [], 2.0, 10.0, 2);
        var live = new double[stride];
        live[q] = Math.Cos(Math.PI / 36.0);
        live[q + 1] = Math.Sin(Math.PI / 36.0);
        live[w] = 0.1;
        bool CanTake(double time) => Ksa6DofGuidance.MatchesHandoffAttitude(
            plan, live, time, Math.PI / 36.0, Math.PI / 60.0);
        t.Check("a standby plan can take a craft matching its time-indexed turn", CanTake(11.0));
        live[w] = 0.0;
        t.Check("a standby plan waits for a craft that has not started the planned turn", !CanTake(11.0));
        live[w] = 0.1;
        live[q] = 1.0;
        live[q + 1] = 0.0;
        t.Check("matching rate alone cannot hand over to a plan already turned away", !CanTake(11.0));
        t.Check("a plan that reached its end cannot take the burn", !CanTake(12.0));
    }

    // Inside the handoff time the burn keeps the craft while touchdown guidance plans on standby. With
    // a plan the burn queues the engage and lets the next step's 6-DOF dispatch run it, so the step
    // that queues it must neither release the craft nor cut the engine the burn left lit.
    private static void TheBurnHandsOverToSixDof(TestContext t, Vehicle craft)
    {
        VehicleAutopilotState state = BurnAtHandoff(craft, sixDof: true);
        _sixDofSteps = 0;
        _sixDofCanTake = true;
        _standbyReady = false;

        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("6-DOF: the burn starts the standby and keeps flying",
            state.SixDofStandby && state.LandingPhase == GuidanceWindow.LandingPhase.Burn && !state.EngagePending, state.LandingStatus);
        t.Check("6-DOF: the burn still commands the engine while the standby plans",
            Inputs(craft).EngineOn && Inputs(craft).EngineThrottle == (float)HandoffThrottle,
            $"engine on {Inputs(craft).EngineOn}, throttle {Inputs(craft).EngineThrottle}");
        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("6-DOF: a standby with no plan leaves the burn flying",
            state.SixDofStandby && state.LandingPhase == GuidanceWindow.LandingPhase.Burn && !state.EngagePending && _sixDofSteps == 0);

        _standbyReady = true;
        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("6-DOF: the pending handoff leaves the braking burn in control",
            state.LandingPhase == GuidanceWindow.LandingPhase.Burn && state.EngagePending, state.LandingStatus);
        t.Check("6-DOF: the standby is kept for the engage to promote", state.SixDofStandby);
        t.Check("6-DOF: the handoff keeps the craft",
            state.ControlAcquired && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.Guidance);
        t.Check("6-DOF: the engine keeps the burn's command",
            Inputs(craft).EngineOn && Inputs(craft).EngineThrottle == (float)HandoffThrottle,
            $"engine on {Inputs(craft).EngineOn}, throttle {Inputs(craft).EngineThrottle}");
        t.Check("6-DOF: the previous attitude stays held while no new plan steers",
            state.WasEngaged && state.AttitudeOwnership.IsCurrent(craft.FlightComputer));
        t.Check("6-DOF: AutoStage stays armed through the handoff",
            StagingDetector.IsArmed(craft) && state.ArmedStaging);

        _standbyReady = false;
        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("6-DOF: a plan that no longer matches leaves the burn flying",
            state.LandingPhase == GuidanceWindow.LandingPhase.Burn && !state.EngagePending && _sixDofSteps == 0);
        t.Check("6-DOF: refusing a pending handoff keeps the braking engine lit", Inputs(craft).EngineOn);
        _standbyReady = true;
        GuidanceWindow.ApplyAutopilot(craft);
        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("6-DOF: the next step dispatches the queued engage", _sixDofSteps == 1, $"{_sixDofSteps} dispatches");
        t.Check("6-DOF: the accepted handoff ends the braking mode",
            state.LandingPhase == GuidanceWindow.LandingPhase.Idle, state.LandingStatus);
        t.Check("6-DOF: the engaged descent keeps the craft",
            state.Active && state.ControlAcquired
            && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.Guidance);
        t.Check("6-DOF: the engine is still lit when the engage runs", Inputs(craft).EngineOn);
        t.Check("6-DOF: AutoStage is still armed when the engage runs",
            StagingDetector.IsArmed(craft) && state.ArmedStaging);

        state.SixDofStandby = false;
        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);
    }

    // The standby has no plan when the burn reaches its last seconds. G-FOLD takes the craft when it
    // can fly it, with the engine lit, and a cold touchdown engage takes it when G-FOLD cannot.
    private static void AStandbyWithNoPlanGoesToGfold(TestContext t, Vehicle craft)
    {
        foreach (bool gfold in new[] { true, false })
        {
            VehicleAutopilotState state = BurnAtHandoff(craft, sixDof: true, tgo: 3.0);
            _sixDofSteps = 0;
            _sixDofCanTake = true;
            _standbyReady = false;
            _gfoldCanFly = gfold;
            string label = gfold ? "no plan, G-FOLD can fly" : "no plan, G-FOLD cannot fly";

            GuidanceWindow.ApplyAutopilot(craft);
            t.Check($"{label}: the standby gets its first step", state.SixDofStandby && state.LandingPhase == GuidanceWindow.LandingPhase.Burn);
            GuidanceWindow.ApplyAutopilot(craft);
            if (gfold)
            {
                t.Check($"{label}: the burn hands over to G-FOLD",
                    state.LandingPhase == GuidanceWindow.LandingPhase.GfoldDescent && !state.SixDofStandby && !state.EngagePending, state.LandingStatus);
                t.Check($"{label}: the status says why", state.LandingStatus.Contains("no plan before the end of the braking burn"), state.LandingStatus);
            }
            else
            {
                t.Check($"{label}: the burn queues a cold touchdown engage", state.EngagePending && state.LandingPhase == GuidanceWindow.LandingPhase.Idle, state.LandingStatus);
            }
            t.Check($"{label}: the craft stays held with the engine lit",
                state.ControlAcquired && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.Guidance
                && Inputs(craft).EngineOn && !state.LandingCutPending);

            state.SixDofStandby = false;
            VehicleControlOwnership.ReleaseAll(craft);
            VehicleAutopilotState.Remove(craft);
        }
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

    // 6-DOF is the default, but a craft it cannot plan for would be refused on the next step with the
    // engine lit and nothing steering. The handoff gives it to G-FOLD instead, and the craft's solver
    // choice follows, so the panel shows and aborts the descent that is flying.
    private static void ACraftSixDofRefusesGoesToGfold(TestContext t, Vehicle craft)
    {
        VehicleAutopilotState state = BurnAtHandoff(craft, sixDof: true);
        _sixDofSteps = 0;
        _sixDofCanTake = false;
        _gfoldCanFly = true;

        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("refused: the handoff starts the G-FOLD descent",
            state.LandingPhase == GuidanceWindow.LandingPhase.GfoldDescent, state.LandingStatus);
        t.Check("refused: no 6-DOF engage is queued or run", !state.EngagePending && !state.Active && _sixDofSteps == 0,
            $"pending {state.EngagePending}, active {state.Active}, {_sixDofSteps} dispatches");
        t.Check("refused: the craft now lands with G-FOLD", !state.UseSixDofLanding);
        t.Check("refused: the status says why", state.LandingStatus.Contains(Refusal), state.LandingStatus);
        t.Check("refused: the handoff keeps the craft",
            state.ControlAcquired && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.Guidance);
        t.Check("refused: the engine stays lit through the handoff step",
            Inputs(craft).EngineOn && Inputs(craft).EngineThrottle == (float)HandoffThrottle,
            $"engine on {Inputs(craft).EngineOn}, throttle {Inputs(craft).EngineThrottle}");

        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);

        // G-FOLD cannot fly the craft either, so no G-FOLD descent starts. The burn queues the touchdown engage, whose refusal gives the craft back with the engine as it is.
        state = BurnAtHandoff(craft, sixDof: true);
        _gfoldCanFly = false;
        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("refused, G-FOLD cannot fly: no G-FOLD descent starts",
            state.LandingPhase == GuidanceWindow.LandingPhase.Idle && state.EngagePending && state.UseSixDofLanding, state.LandingStatus);
        t.Check("refused, G-FOLD cannot fly: the engine stays lit at its throttle and no cut is queued",
            Inputs(craft).EngineOn && Inputs(craft).EngineThrottle == BurnThrottle && !state.LandingCutPending && state.SixDofTookBurn,
            $"engine on {Inputs(craft).EngineOn}, throttle {Inputs(craft).EngineThrottle}");

        VehicleControlOwnership.ReleaseAll(craft);
        VehicleAutopilotState.Remove(craft);
    }

    // A deorbit burn on the step it reaches the handoff: guidance owns and steers the craft, the
    // engine is lit at the throttle the burn wrote last, guidance armed AutoStage for the flight, and
    // UPFG reads converged inside the gate.
    private static VehicleAutopilotState BurnAtHandoff(Vehicle craft, bool sixDof, double tgo = double.NaN)
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

        SetSolution(state.Upfg, tgo: double.IsNaN(tgo) ? state.GfoldHandoffTgo - 1.0 : tgo, throttle: HandoffThrottle);
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

    private static bool EnginesReadyForFixture(Vehicle vehicle, ref bool __result)
    {
        if (!ReferenceEquals(vehicle, _fixture))
            return true;
        __result = true;
        return false;
    }

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

    private static bool StartStandbyForFixture(Vehicle vehicle, ref bool __result)
    {
        if (!ReferenceEquals(vehicle, _fixture))
            return true;
        VehicleAutopilotState.For(vehicle).SixDofStandby = true;
        __result = true;
        return false;
    }

    private static bool AnswerStandbyReady(ref bool __result)
    {
        if (_fixture == null)
            return true;
        __result = _standbyReady;
        return false;
    }

    private static bool AnswerGfoldCanFly(Vehicle vehicle, ref string why, ref bool __result)
    {
        if (!ReferenceEquals(vehicle, _fixture))
            return true;
        why = _gfoldCanFly ? "" : "G-FOLD is limited to airless bodies.";
        __result = _gfoldCanFly;
        return false;
    }

    // Stands in for the handoff's check that 6-DOF can plan for the craft, which would otherwise
    // depend on the fixture's gimbals and throttle floor.
    private static bool AnswerCanSixDofTake(Vehicle vehicle, ref string error, ref bool __result)
    {
        if (!ReferenceEquals(vehicle, _fixture))
            return true;
        error = _sixDofCanTake ? "" : Refusal;
        __result = _sixDofCanTake;
        return false;
    }
}

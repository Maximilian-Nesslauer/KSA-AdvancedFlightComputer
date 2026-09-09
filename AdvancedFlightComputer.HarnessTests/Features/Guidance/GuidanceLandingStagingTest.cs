using System.Reflection;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Guidance.Gfold;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Uses a controlled clock and synthetic sequence activation to test landing waits.
// Most cases stop at engine readiness, before transforms, solving, or actuator writes.
// A spawned-vehicle case exercises ownership through ApplyAutopilot.
public sealed class GuidanceLandingStagingTest : AfcTest
{
    public override string Name => "afc-guidance-landing-staging";

    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;

    private static readonly HashSet<Vehicle> Synthetic = [];
    private static VehicleAutopilotState? _endModeOnStep;
    private static bool _modeEndedInStep;
    private static double _now;
    private static int _activations;
    private static bool _freezeSequence;
    private static bool _resumed;

    protected override void Execute(TestContext t)
    {
        FieldInfo ambient = typeof(GuidanceWindow).GetField("_s", PrivateStatic)!;
        object? previousAmbient = ambient.GetValue(null);
        var harmony = new Harmony("com.maxi.afc.harnesstests.guidance.staging");
        try
        {
            harmony.Patch(Method("SimNow"), prefix: Prefix(nameof(Clock)));
            harmony.Patch(Method("ShouldStageForReserve"), prefix: Prefix(nameof(False)));
            harmony.Patch(Method("ShouldDropSpentEngines"), prefix: Prefix(nameof(False)));
            harmony.Patch(Method("WouldLoseControl"), prefix: Prefix(nameof(False)));
            harmony.Patch(Method("BuildUpfgVehicle"), prefix: Prefix(nameof(Skip)));
            harmony.Patch(AccessTools.Method(typeof(Vehicle), nameof(Vehicle.UpdateAfterPartTreeModification)),
                prefix: Prefix(nameof(SkipSyntheticConfiguration)));
            harmony.Patch(AccessTools.Method(typeof(SequenceList), nameof(SequenceList.ActivateNextSequence)),
                prefix: Prefix(nameof(Activate)));
            harmony.Patch(Method("PrepareLandingEngines"), postfix: Prefix(nameof(StopAfterReady)));

            foreach (GuidanceWindow.LandingPhase phase in new[]
                { GuidanceWindow.LandingPhase.GfoldDescent, GuidanceWindow.LandingPhase.TerminalHover })
            {
                DecoupleThenIgnite(t, phase);
                NoProgress(t, phase);
                LastSequenceWithoutEngine(t, phase);
                TotalWaitBound(t, phase);
                IncompatibleEngine(t, phase);
                AutoStageDisabled(t, phase);
            }
            AtmosphereRefusal(t);
            RestartAfterAbort(t);
            WaitStateEndsWithOwnership(t);
            OwnershipSurvivesTheWaitAndEndsWithIt(t, harmony);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            _freezeSequence = false;
            _endModeOnStep = null;
            _modeEndedInStep = false;
            Synthetic.Clear();
            ambient.SetValue(null, previousAmbient);
        }
    }

    private static MethodInfo Method(string name) =>
        typeof(GuidanceWindow).GetMethod(name, PrivateStatic)
        ?? throw new MissingMethodException(nameof(GuidanceWindow), name);

    private static HarmonyMethod Prefix(string name) => new(typeof(GuidanceLandingStagingTest), name);

    private static bool Clock(ref double __result)
    {
        __result = _now;
        return false;
    }

    private static bool False(ref bool __result)
    {
        __result = false;
        return false;
    }

    private static bool Skip() => false;

    // A built craft has no part tree to configure. A spawned one must configure normally,
    // because its constructor runs this while this patch is installed.
    private static bool SkipSyntheticConfiguration(Vehicle __instance) => !Synthetic.Contains(__instance);

    // Ends the mode from inside the step, which is where a timeout ends it, so the ownership
    // release after mode processing is the one under test.
    private static bool EndModeDuringStep()
    {
        if (_endModeOnStep != null)
        {
            _endModeOnStep.LandingPhase = GuidanceWindow.LandingPhase.Done;
            _modeEndedInStep = true;
        }
        return false;
    }

    private static bool Activate(SequenceList __instance)
    {
        _activations++;
        if (!_freezeSequence)
            foreach (Sequence sequence in __instance.Sequences)
                if (!sequence.Activated)
                {
                    sequence.Activated = true;
                    break;
                }
        return false;
    }

    // Stop at readiness because the synthetic vehicle cannot run the rest of the mode.
    private static void StopAfterReady(bool __result)
    {
        if (__result)
            throw new ReadyException();
    }

    private sealed class ReadyException : Exception;

    private static void Step(Fixture fixture, double time, IParentBody? parent = null)
    {
        _now = time;
        _resumed = false;
        typeof(GuidanceWindow).GetField("_s", PrivateStatic)!.SetValue(null, fixture.State);
        object?[] args = fixture.Phase == GuidanceWindow.LandingPhase.GfoldDescent
            ? [fixture.Vehicle, null, parent, 1.0, time]
            : [fixture.Vehicle, null, parent, 1.0, 1.0, time];
        try
        {
            Method(fixture.Phase == GuidanceWindow.LandingPhase.GfoldDescent
                ? "StepGfoldDescent" : "StepTerminalHover").Invoke(null, args);
        }
        catch (TargetInvocationException error) when (error.InnerException is ReadyException)
        {
            _resumed = true;
        }
    }

    private static void DecoupleThenIgnite(TestContext t, GuidanceWindow.LandingPhase phase)
    {
        var fixture = new Fixture(phase, 2);
        Step(fixture, 100);
        CheckWaiting(t, fixture, $"{phase} decouple");
        t.Check($"{phase}: a decouple sequence is requested", _activations == 1);
        Step(fixture, 100.5);
        t.Check($"{phase}: the sequence cooldown is respected", _activations == 1);
        Step(fixture, 101);
        t.Check($"{phase}: the ignition sequence follows the cooldown", _activations == 2);
        Step(fixture, 101.1);
        t.Check($"{phase}: staging can end before the supplied state arrives", !fixture.State.StagingActive);

        fixture.SupplyEngine();
        Step(fixture, 101.2);
        t.Check($"{phase}: the mode resumes once the engine is supplied",
            _resumed && fixture.State.LandingPhase == phase);
        t.Check($"{phase}: the resume does not restore a stale command",
            fixture.State.GfoldPlan == null && !fixture.State.HasCommand);
        t.Check($"{phase}: the wait state is reset after recovery",
            double.IsNaN(fixture.State.LandingEngineWaitStart) && fixture.State.LandingStatus == "");
    }

    private static void NoProgress(TestContext t, GuidanceWindow.LandingPhase phase)
    {
        var fixture = new Fixture(phase, 2);
        _freezeSequence = true;
        try
        {
            Step(fixture, 100);
            Step(fixture, 101);
            Step(fixture, 102);
            CheckWaiting(t, fixture, $"{phase} no progress");
            int before = _activations;
            Step(fixture, 103);
            t.Check($"{phase}: the expiry step asks for no further staging", _activations == before);
        }
        finally
        {
            _freezeSequence = false;
        }
        t.Check($"{phase}: repeated requests without sequence progress time out",
            fixture.State.LandingPhase == GuidanceWindow.LandingPhase.Done);
        t.Check($"{phase}: the timeout is visible", fixture.State.LandingStatus.Contains("timed out"));
    }

    private static void LastSequenceWithoutEngine(TestContext t, GuidanceWindow.LandingPhase phase)
    {
        var fixture = new Fixture(phase, 1);
        Step(fixture, 100);
        CheckWaiting(t, fixture, $"{phase} last sequence");
        Step(fixture, 101);
        Step(fixture, 102);
        t.Check($"{phase}: the settling window after the last sequence is bounded",
            fixture.State.LandingPhase == GuidanceWindow.LandingPhase.Done);
        t.Check($"{phase}: a last sequence without thrust leaves the engine off",
            fixture.State.LandingCutPending && fixture.State.GfoldThrottle == 0);
    }

    private static void TotalWaitBound(TestContext t, GuidanceWindow.LandingPhase phase)
    {
        var fixture = new Fixture(phase, 15);
        bool held = true;
        for (int second = 0; second < 10; second++)
        {
            Step(fixture, 100 + second);
            held &= fixture.State.LandingPhase == phase;
        }
        t.Check($"{phase}: sequence progress keeps the wait alive", held);
        int activationsBefore = _activations;
        Step(fixture, 110);
        t.Check($"{phase}: the expiry step asks for no further staging", _activations == activationsBefore);
        t.Check($"{phase}: the total wait is bounded even while sequences keep firing",
            fixture.State.LandingPhase == GuidanceWindow.LandingPhase.Done);
    }

    private static void IncompatibleEngine(TestContext t, GuidanceWindow.LandingPhase phase)
    {
        var fixture = new Fixture(phase, 2, solid: true);
        fixture.SupplyEngine();
        Step(fixture, 100);
        t.Check($"{phase}: an unsupported core is refused before any staging request",
            fixture.State.LandingPhase == GuidanceWindow.LandingPhase.Done && _activations == 0);
        t.Check($"{phase}: the reason is visible", fixture.State.LandingStatus.Contains("shutdown"));
    }

    private static void AutoStageDisabled(TestContext t, GuidanceWindow.LandingPhase phase)
    {
        var fixture = new Fixture(phase, 2);
        fixture.State.AutoStage = false;
        Step(fixture, 100);
        t.Check($"{phase}: manual staging does not inherit the automatic wait",
            fixture.State.LandingPhase == GuidanceWindow.LandingPhase.Done && _activations == 0);
    }

    private static void AtmosphereRefusal(TestContext t)
    {
        var fixture = new Fixture(GuidanceWindow.LandingPhase.GfoldDescent, 2);
        var body = DispatchProxy.Create<IParentBody, AtmosphericBody>();
        Step(fixture, 100, body);
        t.Check("an atmospheric body is refused before any staging request",
            fixture.State.LandingPhase == GuidanceWindow.LandingPhase.Done && _activations == 0);
        t.Check("the atmosphere reason is visible", fixture.State.LandingStatus.Contains("airless"));
    }

    private static void CheckWaiting(TestContext t, Fixture fixture, string label)
    {
        t.Check($"{label}: the mode waits instead of ending",
            !_resumed && fixture.State.LandingPhase == fixture.Phase);
        t.Check($"{label}: waiting commands the engine off",
            fixture.State.GfoldThrottle == 0 && !fixture.State.GfoldEngineOn && fixture.State.LandingCutPending);
        t.Check($"{label}: waiting drops the stale plan and steering",
            fixture.State.GfoldPlan == null && !fixture.State.HasCommand && !fixture.State.GfoldTrackInit);
        t.Check($"{label}: waiting resets the hover integral",
            !fixture.State.TermInit && fixture.State.TermPidUp.I == 0);
        t.Check($"{label}: the wait is visible", fixture.State.LandingStatus.Contains("Waiting"));
    }

    private static void RestartAfterAbort(TestContext t)
    {
        var fixture = new Fixture(GuidanceWindow.LandingPhase.GfoldDescent, 2);
        Step(fixture, 100);
        CheckWaiting(t, fixture, "restart");

        Method("AbortLanding").Invoke(null, null);
        t.Check("an abort ends the previous wait", WaitReset(fixture));

        _now = 200;
        Method("StartGfoldNow").Invoke(null, [fixture.Vehicle]);
        t.Check("a restart begins without the previous deadline", WaitReset(fixture));
        Step(fixture, 200);
        CheckWaiting(t, fixture, "restarted");
        t.Check("the restart gets its own deadline", fixture.State.LandingEngineWaitStart == 200);

        fixture.SupplyEngine();
        Step(fixture, 200.1);
        t.Check("the restarted landing resumes when the engine arrives", _resumed);
    }

    private static void WaitStateEndsWithOwnership(TestContext t)
    {
        foreach (string method in new[] { "ResetFlightComputer", "ClaimVehicle", "HandBackVehicle", "ExecuteLanding" })
        {
            var fixture = new Fixture(GuidanceWindow.LandingPhase.GfoldDescent, 2);
            Step(fixture, 100);
            if (!t.Check($"{method}: the fixture starts with an active wait",
                    double.IsFinite(fixture.State.LandingEngineWaitStart)))
                continue;

            MethodInfo target = Method(method);
            object?[]? args = method switch
            {
                "ClaimVehicle" => [Enum.Parse(target.GetParameters()[0].ParameterType, "Ascent"), fixture.Vehicle],
                "HandBackVehicle" => [fixture.Vehicle],
                "ExecuteLanding" => [fixture.Vehicle, null, null, 1.0, 1.0],
                _ => null,
            };
            target.Invoke(null, args);
            t.Check($"{method} clears every wait field", WaitReset(fixture));
        }
    }

    // Waiting retains ownership, but completion must release it before the next frame's input.
    private static void OwnershipSurvivesTheWaitAndEndsWithIt(TestContext t, Harmony harmony)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(RcsTestVehicles.Candidates);
        if (saves.Count == 0)
        {
            t.Skip("no test vehicle save present, so the ownership sequence is not checked.");
            return;
        }
        if (VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state, so the ownership sequence is not checked.");
            return;
        }

        Vehicle? previousFocus = Program.ControlledVehicle;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        Vehicle? craft = null;
        try
        {
            try
            {
                craft = VehicleSpawner.SpawnFromSave(saves[0], t.System, home, "HarnessGuidanceOwnership",
                    OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed));
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"'{saves[0]}': {e.Message}");
                return;
            }

            // Stub the mode to isolate ApplyAutopilot's ownership transitions.
            harmony.Patch(Method("StepGfoldDescent"), prefix: Prefix(nameof(EndModeDuringStep)));
            Program.ControlledVehicle = craft;
            VehicleAutopilotState state = VehicleAutopilotState.For(craft);
            state.Engage = true;
            state.LandingPhase = GuidanceWindow.LandingPhase.GfoldDescent;
            state.ControlAcquired = true;
            state.HasCommand = true;
            state.CommandDir = craft.Orbit.StateVectors.PositionCci.Normalized();

            GuidanceWindow.ApplyAutopilot(craft);
            t.Check("a commanded step owns the craft", state.ControlAcquired && state.WasEngaged);

            // What a staging wait does to the state.
            state.HasCommand = false;
            GuidanceWindow.ApplyAutopilot(craft);
            t.Check("a waiting step keeps the craft", state.ControlAcquired);
            t.Check("a waiting step gives the attitude back", !state.WasEngaged);

            // The timeout ends the mode inside the step, so the entry guard cannot be the one
            // that releases, and the release after mode processing is what this asserts.
            _endModeOnStep = state;
            _modeEndedInStep = false;
            GuidanceWindow.ApplyAutopilot(craft);
            t.Check("the mode ended inside the step", _modeEndedInStep);

            // The release clears the phase as it clears the rest, so ownership is the signal.
            t.Check("the step that ends the mode also ends the ownership", !state.ControlAcquired);
        }
        finally
        {
            if (craft != null)
                VehicleAutopilotState.Remove(craft);
            Program.ControlledVehicle = previousFocus;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static bool WaitReset(Fixture fixture) =>
        double.IsNaN(fixture.State.LandingEngineWaitStart)
        && fixture.State.LandingEngineWaitProgressTime == 0
        && fixture.State.LandingEngineWaitNextSequence == null;

    private sealed class Fixture
    {
        internal readonly Vehicle Vehicle = Empty<Vehicle>();
        internal readonly VehicleAutopilotState State = new();
        internal readonly GuidanceWindow.LandingPhase Phase;
        private readonly EngineController _engine = Empty<EngineController>();
        private readonly RocketCore _core;

        internal Fixture(GuidanceWindow.LandingPhase phase, int sequenceCount, bool solid = false)
        {
            Phase = phase;
            _activations = 0;
            _freezeSequence = false;

            Synthetic.Add(Vehicle);
            PartTree tree = Empty<PartTree>();
            tree.States = new ModuleStateList();
            Vehicle.Parts = tree;
            tree.SequenceList = Empty<SequenceList>();
            var sequences = Enumerable.Range(0, sequenceCount).Select(_ => Empty<Sequence>()).ToList();
            typeof(SequenceList).GetField("_sequences", BindingFlags.Instance | BindingFlags.NonPublic)!
                .SetValue(tree.SequenceList, sequences);

            _core = solid ? Empty<SolidMotor>() : Empty<Combustor>();
            _engine.Cores = [_core];
            tree.States.AddNew<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>(_engine);
            tree.States.AddNew<RocketCore, RocketCoreState, RocketCoreGlobalState, EmptyStruct>(_core);

            State.Engage = State.AutoStage = true;
            State.LandingPhase = phase;
            State.GfoldPlan = Empty<GfoldTrajectory>();
            State.GfoldThrottle = 0.8;
            State.GfoldEngineOn = State.HasCommand = State.GfoldTrackInit = State.TermInit = true;
            State.TermPidUp.I = 5;
        }

        internal void SupplyEngine()
        {
            typeof(EngineController).GetProperty(nameof(EngineController.IsActive))!.SetValue(_engine, true);
            EngineController.TryGetFrom(Vehicle.Parts.States, out var engineStates);
            Unsafe.AsRef(in engineStates.States[_engine.StatesIdx]).IsPropellantAvailable = true;
            engineStates.GetMutableGlobalStateForInitialization().IsAnyActive = true;
            engineStates.GetMutableGlobalStateForInitialization().IsAnyPropellantAvailable = true;
            RocketCore.TryGetFrom(Vehicle.Parts.States, out var coreStates);
            Unsafe.AsRef(in coreStates.States[_core.StatesIdx]).IsPropellantAvailable = true;
        }
    }

    // Constructors are skipped, so finalizers must not access incomplete state.
    private static T Empty<T>() where T : class
    {
        var value = (T)RuntimeHelpers.GetUninitializedObject(typeof(T));
        GC.SuppressFinalize(value);
        return value;
    }
}

public class AtmosphericBody : DispatchProxy
{
    protected override object Invoke(MethodInfo? targetMethod, object?[]? args)
        => targetMethod?.Name == nameof(IParentBody.GetAtmosphereReference)
            ? new AtmosphereReference()
            : throw new NotSupportedException(targetMethod?.Name);
}

using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;
using System.Reflection;

namespace AdvancedFlightComputer.HarnessTests;

// While a staging keeps an Auto burn alive, FlightComputer.ComputeControl sees every active engine
// dry, drops the burn to Manual and raises FlightComputer.AutoBurnStoppedOutOfPropellant, which
// FlightComputer.RaisePendingAlerts would turn into the end of the burn and, with another burn
// planned, its removal. The detector's evaluation runs before that and takes the stop back for the
// target it staged under, and only for that target.
//
// The stop is set by hand on a burn that waits far in the future, so no engine content is needed:
//   held:      the staged-under target is loaded, the stop is cleared and Auto is given back, and
//              stock then leaves both burns in the plan.
//   completed: stock completed the burn, the hold ends and the stop stands for stock.
//   reloaded:  another burn was loaded ahead of it, the hold ends and the stop stands for stock.
//   give-back: StockBurnMode.GiveBackAuto leaves a started burn in Manual while every active engine reads
//              dry, unless the caller takes the stop back itself as the staging hold does.
//   player:    a player staging during an Auto burn, armed and not armed. The row's activation lands in
//              the next frame's input drain, and the stale stop of the new engines is taken back until
//              the worker has reported them. The new engines are reported fueled for the first worker tick.
//   give-up:   a player staging whose activation never lands lets the stop stand after the frame bound.
//   next row:  the engines the staging patch hands over are the ones SequenceList.ActivateNextSequence lights.
//   binding:   the staging patch and the new-engine patch bind to their game methods.
//   burnout:   the burnout trigger itself. The sample sees the burn in Auto and the stop arrives with no
//              engine fed, so the evaluation stages, holds the running target and takes the stop back,
//              and on the next frame the phase the staging left takes it back again.
public sealed class AutoStageAutoBurnHoldTest : AfcTest
{
    private const double BurnLeadSeconds = 3600.0;

    public override string Name => "afc-autostage-auto-burn-hold";

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

        Vehicle? vehicle = null;
        try
        {
            vehicle = VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance,
                "AfcStagingHold_" + Guid.NewGuid().ToString("N"),
                OrbitFixtures.CircularAt(home, 600_000, Universe.GetElapsedTime()));
            SimDriver driver = t.Session.CreateDriver();
            driver.Step(0.05, 2);
            StagingDetector.Arm(vehicle, true);

            CheckHeld(t, vehicle);
            CheckCompleted(t, vehicle);
            CheckReloaded(t, vehicle);
            CheckGiveBackGuard(t, vehicle);
            CheckPlayerStaging(t, vehicle, armed: true);
            CheckPlayerStaging(t, vehicle, armed: false);
            CheckActivationGiveUp(t, vehicle);
            CheckNextRowEngines(t, vehicle);
            CheckPatchBinding(t);
            CheckBurnoutTrigger(t, vehicle);
        }
        finally
        {
            if (vehicle != null)
            {
                StagingDetector.ForgetVehicle(vehicle);
                vehicle.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
                RcsFlightSupport.CleanupBurns(vehicle.FlightComputer);
                VehicleSpawner.Despawn(vehicle);
            }
        }
    }

    private static void CheckHeld(TestContext t, Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        AddPlanBurn(vehicle, BurnLeadSeconds);
        Burn later = AddPlanBurn(vehicle, BurnLeadSeconds * 2.0);
        BurnTarget? held = fc.Burn;
        if (!t.Check("held: a burn target is loaded", held != null && fc.BurnPlan.BurnCount == 2))
            return;

        StagingState state = HoldDuringIgnitionDelay(vehicle, held!);
        StopOutOfPropellant(fc);
        Evaluate();
        t.Check("held: the stop is taken back before stock reads it", !fc.AutoBurnStoppedOutOfPropellant);
        t.Check("held: Auto is given back on the same target",
            fc.BurnMode == FlightComputerBurnMode.Auto && ReferenceEquals(fc.Burn, held)
            && ReferenceEquals(state.HeldBurn, held));

        fc.RaisePendingAlerts(vehicle);
        t.Check("held: stock leaves both burns in the plan",
            fc.BurnPlan.BurnCount == 2 && fc.BurnPlan.TryGetBurn(later) && ReferenceEquals(fc.Burn, held));
        Release(vehicle, state);
    }

    private static void CheckCompleted(TestContext t, Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (!t.Check("completed: a burn target is loaded", fc.Burn != null))
            return;

        StagingState state = HoldDuringIgnitionDelay(vehicle, fc.Burn!);
        StopOutOfPropellant(fc);
        fc.AutoBurnCompleted = true;
        Evaluate();
        t.Check("completed: the hold ends and the stop stands",
            state.HeldBurn == null && fc.AutoBurnStoppedOutOfPropellant
            && fc.BurnMode == FlightComputerBurnMode.Manual);
        Release(vehicle, state);
    }

    private static void CheckReloaded(TestContext t, Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (!t.Check("reloaded: a burn target is loaded", fc.Burn != null))
            return;

        StagingState state = HoldDuringIgnitionDelay(vehicle, fc.Burn!);
        // FlightComputer.AddBurn loads a burn that becomes the first executable one, which replaces the target.
        AddPlanBurn(vehicle, BurnLeadSeconds * 0.5);
        StopOutOfPropellant(fc);
        Evaluate();
        t.Check("reloaded: the hold ends and the stop stands",
            state.HeldBurn == null && fc.AutoBurnStoppedOutOfPropellant
            && fc.BurnMode == FlightComputerBurnMode.Manual);
        Release(vehicle, state);
    }

    // FlightComputer.ComputeControl stops a started Auto burn when an engine is active and none reports propellant, so Auto given back there would stop on the next worker step.
    private static void CheckGiveBackGuard(TestContext t, Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.CleanupBurns(fc);
        AddPlanBurn(vehicle, BurnLeadSeconds);
        AddPlanBurn(vehicle, BurnLeadSeconds * 2.0);
        BurnTarget? target = fc.Burn;
        List<EngineController> lit = vehicle.Parts.Modules.Get<EngineController>().ToArray().Where(e => e.IsActive).ToList();
        if (!t.Check("give-back: a burn target is loaded and an engine is lit", target != null && lit.Count > 0))
            return;

        List<bool> saved = lit.ConvertAll(engine => CommittedState(vehicle, engine).IsPropellantAvailable);
        float? savedThrottle = target!.Throttle;
        try
        {
            // A throttle on the target is how stock marks a burn as started.
            target.Throttle = 1f;
            fc.BurnMode = FlightComputerBurnMode.Manual;
            lit.ForEach(engine => SetReported(vehicle, engine, false));
            t.Check("give-back: a started burn with every active engine dry stays in Manual",
                !StockBurnMode.GiveBackAuto(vehicle, target) && fc.BurnMode == FlightComputerBurnMode.Manual
                && StockBurnMode.WouldStopAtOnce(vehicle, target));
            t.Check("give-back: the staging hold, which takes the stop back itself, still gets Auto",
                StockBurnMode.GiveBackAuto(vehicle, target, takesStopBack: true) && fc.BurnMode == FlightComputerBurnMode.Auto);

            fc.BurnMode = FlightComputerBurnMode.Manual;
            SetReported(vehicle, lit[0], true);
            t.Check("give-back: a fed engine gets Auto back",
                StockBurnMode.GiveBackAuto(vehicle, target) && fc.BurnMode == FlightComputerBurnMode.Auto);
        }
        finally
        {
            for (int i = 0; i < lit.Count; i++)
                SetReported(vehicle, lit[i], saved[i]);
            target.Throttle = savedThrottle;
            fc.BurnMode = FlightComputerBurnMode.Manual;
            RcsFlightSupport.CleanupBurns(fc);
        }
    }

    // SequenceList.ActivateNextSequence enters the hold for the engines of its row, which only land in the next frame's input drain.
    // One fueled engine of a later sequence stands in for that row here, switched on through the same queue, so no part separates.
    // No step runs, so the worker never reports the engine and the hold stays in its propagation wait.
    private static void CheckPlayerStaging(TestContext t, Vehicle vehicle, bool armed)
    {
        string label = armed ? "player staging, armed" : "player staging, not armed";
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.CleanupBurns(fc);
        AddPlanBurn(vehicle, BurnLeadSeconds);
        Burn later = AddPlanBurn(vehicle, BurnLeadSeconds * 2.0);
        BurnTarget? running = fc.Burn;
        EngineController? engine = FindUnlitFueledEngine(vehicle);
        if (!t.Check(label + ": a burn target is loaded and an unlit fueled engine exists",
                running != null && engine != null && fc.BurnPlan.BurnCount == 2))
            return;

        StagingDetector.Arm(vehicle, armed);
        StagingState state = StagingDetector.StateOf(vehicle);
        running!.DeltaVAccumCci = float3.Zero;
        fc.BurnMode = FlightComputerBurnMode.Auto;
        try
        {
            StagingDetector.OnPlayerStaged(vehicle, [engine!]);
            t.Check(label + ": the staging holds the running target",
                state.State == StagingState.Phase.AwaitingActivation && ReferenceEquals(state.HeldBurn, running));

            Evaluate();
            t.Check(label + ": the hold waits while the activation is queued",
                state.State == StagingState.Phase.AwaitingActivation && fc.BurnMode == FlightComputerBurnMode.Auto,
                $"phase {state.State}");

            engine!.SetIsActive(vehicle, true);
            InputEvents.ApplyInputEvents();
            t.Check(label + ": the first worker tick sees the new engine fueled while it is still unreported",
                NewEngineSeedPatch.Seed(vehicle) == 1 && CommittedState(vehicle, engine).IsPropellantAvailable
                && !CommittedState(vehicle, engine).WasActive && StagingHelpers.HasUnreportedActiveEngine(vehicle));

            StopOutOfPropellant(fc);
            Evaluate();
            t.Check(label + ": the stop is taken back and Auto is given back on the same target",
                !fc.AutoBurnStoppedOutOfPropellant && fc.BurnMode == FlightComputerBurnMode.Auto
                && ReferenceEquals(fc.Burn, running), $"phase {state.State}");
            t.Check(label + ": the hold stays until the worker reports the engine",
                state.State == StagingState.Phase.AwaitingPropagation && ReferenceEquals(state.HeldBurn, running),
                $"phase {state.State}");
            fc.RaisePendingAlerts(vehicle);
            t.Check(label + ": stock leaves both burns in the plan",
                fc.BurnPlan.BurnCount == 2 && fc.BurnPlan.TryGetBurn(later) && ReferenceEquals(fc.Burn, running));

            // Rocket.UpdateRockets sets WasActive on its first pass over the engine, which is what the worker reports.
            SetWasActive(vehicle, engine, true);
            Evaluate();
            t.Check(label + ": once the worker reports the engine the hold ends with Auto on the same target",
                state.State == StagingState.Phase.Monitoring && state.HeldBurn == null
                && fc.BurnMode == FlightComputerBurnMode.Auto && ReferenceEquals(fc.Burn, running),
                $"phase {state.State}");
        }
        finally
        {
            if (engine != null)
                SetWasActive(vehicle, engine, false);
            if (engine is { IsActive: true })
            {
                engine.SetIsActive(vehicle, false);
                InputEvents.ApplyInputEvents();
            }
            if (engine != null)
                SetReported(vehicle, engine, false);
            Release(vehicle, state);
            StagingDetector.Arm(vehicle, true);
            RcsFlightSupport.CleanupBurns(fc);
        }
    }

    // The row's activation never lands, so the hold waits out its frame bound and then lets the stop stand for stock.
    private static void CheckActivationGiveUp(TestContext t, Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.CleanupBurns(fc);
        AddPlanBurn(vehicle, BurnLeadSeconds);
        AddPlanBurn(vehicle, BurnLeadSeconds * 2.0);
        BurnTarget? running = fc.Burn;
        EngineController? engine = FindUnlitFueledEngine(vehicle);
        if (!t.Check("give-up: a burn target is loaded and an unlit fueled engine exists", running != null && engine != null))
            return;

        StagingState state = StagingDetector.StateOf(vehicle);
        running!.DeltaVAccumCci = float3.Zero;
        fc.BurnMode = FlightComputerBurnMode.Auto;
        try
        {
            StagingDetector.OnPlayerStaged(vehicle, [engine!]);
            for (int i = 0; i < StagingDetector.ActivationFrames; i++)
                Evaluate();
            t.Check("give-up: the hold waits for the activation up to its frame bound",
                state.State == StagingState.Phase.AwaitingActivation && ReferenceEquals(state.HeldBurn, running),
                $"phase {state.State}");

            StopOutOfPropellant(fc);
            Evaluate();
            t.Check("give-up: past the bound the hold ends and the stop stands",
                state.State == StagingState.Phase.Monitoring && state.HeldBurn == null
                && fc.AutoBurnStoppedOutOfPropellant && fc.BurnMode == FlightComputerBurnMode.Manual,
                $"phase {state.State}");
        }
        finally
        {
            Release(vehicle, state);
            RcsFlightSupport.CleanupBurns(fc);
        }
    }

    // SequenceList.ActivateNextSequence lights the first unactivated row with parts, through Part.ActivateSubtreeInStage on each of its parts.
    private static void CheckNextRowEngines(TestContext t, Vehicle vehicle)
    {
        SequenceList list = vehicle.Parts.SequenceList;
        var expected = new HashSet<EngineController>();
        int row = -1;
        foreach (Sequence sequence in list.Sequences)
        {
            if (sequence.Activated || sequence.Parts.IsEmpty)
                continue;
            row = sequence.Number;
            foreach (Part part in sequence.Parts)
            {
                foreach (ISequenced module in part.GetSubtreeSequencedModules())
                {
                    if (module.Sequence == sequence.Number && module is EngineController { IsActive: false } engine)
                        expected.Add(engine);
                }
            }
            break;
        }
        List<EngineController>? found = StagingHelpers.EnginesLitByNextRow(list);
        t.Check("next row: the staging patch hands over the engines stock lights",
            (found?.Count ?? 0) == expected.Count && (found == null || found.All(expected.Contains)),
            $"row {row}, {expected.Count} engine(s) expected, {found?.Count ?? 0} found");
    }

    // Bound on an instance of their own, which also resolves the parameter names the patches inject.
    private static void CheckPatchBinding(TestContext t)
    {
        const string id = "com.maxi.afc.harnesstests.auto-burn-hold";
        var harmony = new Harmony(id);
        try
        {
            harmony.CreateClassProcessor(typeof(SequenceListPatches.ActivateNextSequencePatch)).Patch();
            harmony.CreateClassProcessor(typeof(NewEngineSeedPatch)).Patch();
            MethodInfo activate = AccessTools.Method(typeof(SequenceList), nameof(SequenceList.ActivateNextSequence), [typeof(Vehicle)])
                ?? throw new MissingMethodException(typeof(SequenceList).FullName, nameof(SequenceList.ActivateNextSequence));
            MethodInfo prepare = AccessTools.Method(typeof(Vehicle), nameof(Vehicle.PrepareWorker), [typeof(SimStep)])
                ?? throw new MissingMethodException(typeof(Vehicle).FullName, nameof(Vehicle.PrepareWorker));
            Patches? staging = Harmony.GetPatchInfo(activate);
            Patches? seed = Harmony.GetPatchInfo(prepare);
            t.Check("binding: ActivateNextSequence has the staging prefix and postfix",
                staging != null && staging.Prefixes.Count(p => p.owner == id) == 1 && staging.Postfixes.Count(p => p.owner == id) == 1);
            t.Check("binding: PrepareWorker has the new-engine prefix",
                seed != null && seed.Prefixes.Count(p => p.owner == id) == 1);
        }
        catch (Exception e)
        {
            t.Fail("binding", e.Message);
        }
        finally
        {
            harmony.UnpatchAll(id);
        }
    }

    private static EngineController? FindUnlitFueledEngine(Vehicle vehicle)
    {
        vehicle.Parts.EnsureDerived(DerivedData.SolidMotorStacks);
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
        {
            if (!engine.IsActive && VehiclePropellant.IsFueled(engine, vehicle.Parts.Moles.States,
                    vehicle.Parts.RocketCores.States, out _, out _))
                return engine;
        }
        return null;
    }

    private static EngineControllerState CommittedState(Vehicle vehicle, EngineController engine)
        => EngineStates(vehicle).GetState(engine);

    private static void SetReported(Vehicle vehicle, EngineController engine, bool available)
        => EngineStates(vehicle).GetModuleAndAllMutableStatesForInitialization(engine).State.IsPropellantAvailable = available;

    private static void SetWasActive(Vehicle vehicle, EngineController engine, bool wasActive)
        => EngineStates(vehicle).GetModuleAndAllMutableStatesForInitialization(engine).State.WasActive = wasActive;

    private static ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>.StateList EngineStates(Vehicle vehicle)
        => ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
            .TryGetFrom(vehicle.Parts.States, out var states)
            ? states
            : throw new InvalidOperationException("the vehicle has no engine states");

    // No step runs, so the activation the staging queued has not landed and no engine is fed on the second frame either.
    // Depending on the staging delays the machine waits for the ignition or for the propagation, and both hold the burn.
    private static void CheckBurnoutTrigger(TestContext t, Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        RcsFlightSupport.CleanupBurns(fc);
        ShutDownLitEngines(vehicle);
        AddPlanBurn(vehicle, BurnLeadSeconds);
        Burn later = AddPlanBurn(vehicle, BurnLeadSeconds * 2.0);
        BurnTarget? running = fc.Burn;
        if (!t.Check("burnout: a burn target is loaded, no engine is fed and an engine sequence follows",
                running != null && fc.BurnPlan.BurnCount == 2
                && !StagingHelpers.HasActiveEngineWithPropellant(vehicle)
                && StagingHelpers.HasNextEngineSequence(vehicle),
                $"target {running != null}, burns {fc.BurnPlan.BurnCount}, "
                + $"fed engine {StagingHelpers.HasActiveEngineWithPropellant(vehicle)}, "
                + $"next engine sequence {StagingHelpers.HasNextEngineSequence(vehicle)}"))
            return;

        StagingState state = StagingDetector.StateOf(vehicle);
        int activations = StagingDetector.ActivationsOf(vehicle);
        running!.DeltaVAccumCci = float3.Zero;
        fc.BurnMode = FlightComputerBurnMode.Auto;
        try
        {
            StagingDetector.Sample();
            StopOutOfPropellant(fc);
            StagingDetector.Evaluate();
            t.Check("burnout: the stop stages the next sequence",
                StagingDetector.ActivationsOf(vehicle) == activations + 1,
                $"activations {StagingDetector.ActivationsOf(vehicle) - activations}, held for control {state.HeldForControl}");
            t.Check("burnout: the staging holds the running target", ReferenceEquals(state.HeldBurn, running));
            t.Check("burnout: the stop is taken back and Auto is given back on that target",
                !fc.AutoBurnStoppedOutOfPropellant && fc.BurnMode == FlightComputerBurnMode.Auto
                && ReferenceEquals(fc.Burn, running));
            fc.RaisePendingAlerts(vehicle);
            t.Check("burnout: stock leaves both burns in the plan",
                fc.BurnPlan.BurnCount == 2 && fc.BurnPlan.TryGetBurn(later) && ReferenceEquals(fc.Burn, running));

            StagingDetector.Sample();
            StopOutOfPropellant(fc);
            StagingDetector.Evaluate();
            t.Check("burnout: the next frame takes the stop back again",
                !fc.AutoBurnStoppedOutOfPropellant && fc.BurnMode == FlightComputerBurnMode.Auto
                && ReferenceEquals(fc.Burn, running) && ReferenceEquals(state.HeldBurn, running),
                $"phase {state.State}");
            fc.RaisePendingAlerts(vehicle);
            t.Check("burnout: stock still leaves both burns in the plan",
                fc.BurnPlan.BurnCount == 2 && fc.BurnPlan.TryGetBurn(later));
        }
        finally
        {
            // Lands the queued activation while the vehicle still exists.
            InputEvents.ApplyInputEvents();
            state.Pending = null;
            Release(vehicle, state);
        }
    }

    // The spawned design starts with an engine lit and fed, while the burnout trigger needs every active engine dry.
    // EngineController.SetIsActive only queues the change, so the input buffer is applied here.
    private static void ShutDownLitEngines(Vehicle vehicle)
    {
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
        {
            if (engine.IsActive)
                engine.SetIsActive(vehicle, false);
        }
        InputEvents.ApplyInputEvents();
    }

    // A staging whose engine delay is still running, with no modules left to fire, held for the given target.
    private static StagingState HoldDuringIgnitionDelay(Vehicle vehicle, BurnTarget target)
    {
        StagingState state = StagingDetector.StateOf(vehicle);
        state.State = StagingState.Phase.AwaitingIgnition;
        state.Pending = null;
        state.HeldBurn = target;
        target.DeltaVAccumCci = float3.Zero;
        return state;
    }

    // What the worker leaves on the main flight computer when every active engine is dry.
    private static void StopOutOfPropellant(FlightComputer fc)
    {
        fc.BurnMode = FlightComputerBurnMode.Manual;
        fc.AutoBurnStoppedOutOfPropellant = true;
    }

    // The sample comes before the worker results in a frame and the evaluation after them.
    private static void Evaluate()
    {
        StagingDetector.Sample();
        StagingDetector.Evaluate();
    }

    // No step runs between the checks, so stock never reads the flags a check set, and the next check would start with them.
    private static void Release(Vehicle vehicle, StagingState state)
    {
        state.State = StagingState.Phase.Monitoring;
        state.HeldBurn = null;
        state.ActivationEngines = null;
        vehicle.FlightComputer.AutoBurnCompleted = false;
        vehicle.FlightComputer.AutoBurnStoppedOutOfPropellant = false;
        vehicle.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
    }

    private static Burn AddPlanBurn(Vehicle vehicle, double leadSeconds)
    {
        UniverseTime now = Universe.GetElapsedTime();
        PatchedConic patch = new PatchedConic(now, UniverseTime.EndOfTime, PatchTransition.Burn,
            PatchTransition.Final, Orbit.CreateFrom(vehicle.Orbit), vehicle.ParentPatchIdHash);
        Burn burn = Burn.Create(OrbitPointCce.Zero, (now + leadSeconds).Seconds(), new double3(50.0, 0.0, 0.0), patch, vehicle);
        vehicle.FlightComputer.AddBurn(burn);
        return burn;
    }
}

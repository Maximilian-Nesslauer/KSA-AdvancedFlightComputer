using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class GuidanceDeorbitLifecycleTest : AfcTest
{
    public override string Name => "afc-guidance-deorbit-lifecycle";
    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> Inputs =
        AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

    protected override void Execute(TestContext t)
    {
        if (VehicleAutopilotState.Snapshot().Length != 0)
        { t.Skip("another craft holds guidance state."); return; }
        using DeorbitFixture? fixture = DeorbitFixture.Open(t);
        if (fixture == null) return;
        using DeorbitFixture? other = DeorbitFixture.Open(t);
        if (other == null) return;
        Vehicle craft = fixture.Vehicle;
        VehicleAutopilotState state = VehicleAutopilotState.For(craft);
        VehicleAutopilotState otherState = VehicleAutopilotState.For(other.Vehicle);
        otherState.BrakingAltitudeKm = 91;
        otherState.LandingStatus = "Other vehicle.";
        var ambient = AccessTools.Field(typeof(GuidanceWindow), "_s");
        object? previous = ambient.GetValue(null);
        try
        {
            CheckAngleSnapshot(t, state, ambient);
            CheckSequenceTopology(t, fixture, state, ambient);
            CheckPriorModeRelease(t, fixture, state, ambient);
            CheckControlSwitches(t, fixture, state);
            CheckPlanningWait(t, fixture, state);
            CheckActiveBurnWait(t, fixture, state);
            CheckDirectCoast(t, fixture, state);
            Seed(fixture, state);
            VehicleControlOwnership.TryClaim(craft, ControlClaimant.RcsTranslation, out _);
            TestSupport.SetManualControlInputs(craft, 0.63f, engineOn: true);
            GuidanceWindow.ApplyAutopilot(craft);
            t.Check("a denied claim creates no node", state.DeorbitNode == null && !state.DeorbitNodeClaimed);
            t.Check("a denied claim keeps the engine input", Inputs(craft).EngineOn && Inputs(craft).EngineThrottle == 0.63f);
            VehicleControlOwnership.Release(craft, ControlClaimant.RcsTranslation);

            if (!Arm(t, fixture, state)) return;
            Burn node = state.DeorbitNode!;
            BurnTarget target = craft.FlightComputer.Burn!;
            StateVectors departure = node.Patch.Orbit.GetStateVectorsAt(node.Time);
            double3 stockDeltaV = node.DeltaVVlf.Transform(departure.GetVlf2ParentCci().OrIdentity());
            t.CheckAbs("stock reads the planned CCI correction from the VLF node",
                (stockDeltaV - state.DeorbitPlan!.DeltaV).Length(), 0, 1e-6);
            GuidanceWindow.ApplyAutopilot(craft);
            t.Check("AFC leaves stock Auto in control", craft.FlightComputer.BurnMode == FlightComputerBurnMode.Auto
                && ReferenceEquals(craft.FlightComputer.Burn, target) && !state.ControlAcquired && !state.HasCommand);
            double warpTime = (double)AccessTools.Method(typeof(GuidanceWindow), "StockDeorbitWarpTime")
                .Invoke(null, new object[] { craft.FlightComputer, target })!;
            t.Check("stock-node warp stops before alignment preparation",
                warpTime <= target.IgnitionTime.Seconds() - 30 - 10
                && (!float.IsFinite(craft.FlightComputer.ConservativeFlipTime)
                    || warpTime <= target.IgnitionTime.Seconds() - 2 * craft.FlightComputer.ConservativeFlipTime - 10));
            state.DeorbitWarpRequested = true;

            craft.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
            TestSupport.SetManualControlInputs(craft, 0.63f, engineOn: true);
            GuidanceWindow.ApplyAutopilot(craft);
            t.Check("a manual cancellation stops landing without a braking handoff",
                state.LandingPhase == GuidanceWindow.LandingPhase.Done && state.DeorbitPlanner == null
                && !state.DeorbitWarpRequested && double.IsNaN(state.DeorbitWarpTime));
            t.Check("a manual cancellation keeps the player's input and node", Inputs(craft).EngineOn
                && Inputs(craft).EngineThrottle == 0.63f && ReferenceEquals(craft.FlightComputer.BurnPlan.FindFirstExecutableBurn(), node)
                && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.None);
            craft.FlightComputer.RemoveBurn(node);

            Seed(fixture, state);
            if (!Arm(t, fixture, state)) return;
            node = state.DeorbitNode!;
            ambient.SetValue(null, state);
            AccessTools.Method(typeof(GuidanceWindow), "AbortLanding").Invoke(null, Array.Empty<object>());
            GuidanceWindow.ApplyAutopilot(craft);
            t.Check("abort stops stock Auto and removes the owned node", craft.FlightComputer.BurnMode == FlightComputerBurnMode.Manual
                && !Inputs(craft).EngineOn && !craft.FlightComputer.BurnPlan.TryGetBurn(node)
                && state.DeorbitNode == null && state.DeorbitPlan == null
                && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.None);

            Seed(fixture, state);
            if (!Arm(t, fixture, state)) return;
            node = state.DeorbitNode!;
            craft.FlightComputer.RemoveBurn(node);
            GuidanceWindow.ApplyAutopilot(craft);
            t.Check("a deleted node with positive residual is not completion",
                state.LandingPhase == GuidanceWindow.LandingPhase.Done && !state.ControlAcquired);

            CheckDifferentTarget(t, fixture, state);
            CheckLateIgnition(t, fixture, state);
            CheckLongMinimumPreview(t, fixture, state);
            CheckContact(t, fixture, state);
            CheckCompletionAfterRemoval(t, fixture, state);
            state.FcResetPending = true;
            GuidanceWindow.ApplyAutopilot(craft);
            t.Check("reset releases the AFC braking claim", !state.ControlAcquired && state.DeorbitPlan == null
                && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.None);

            Seed(fixture, state);
            if (!Arm(t, fixture, state)) return;
            state.FcResetPending = true;
            GuidanceWindow.ApplyAutopilot(craft);
            t.Check("reset releases stock Auto and its node", state.DeorbitNode == null && !state.DeorbitNodeClaimed
                && craft.FlightComputer.BurnMode == FlightComputerBurnMode.Manual && !Inputs(craft).EngineOn
                && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.None);
            t.Check("the second vehicle keeps its state", otherState.BrakingAltitudeKm == 91
                && otherState.LandingStatus == "Other vehicle." && otherState.LandingPhase == GuidanceWindow.LandingPhase.Idle);
        }
        finally
        {
            ambient.SetValue(null, state);
            AccessTools.Method(typeof(GuidanceWindow), "HandBackVehicle").Invoke(null, new object[] { craft });
            GuidanceWindow.ReleaseDisposedVehicle(craft);
            ambient.SetValue(null, previous);
        }
    }

    private static void CheckSequenceTopology(TestContext t, DeorbitFixture fixture, VehicleAutopilotState state,
        System.Reflection.FieldInfo ambient)
    {
        Vehicle craft = fixture.Vehicle;
        SequenceList? list = craft.Parts.SequenceList;
        if (list == null || list.Count < 2)
        { t.Skip("the fixture has fewer than two staging sequences."); return; }
        Seed(fixture, state);
        ambient.SetValue(null, state);
        Sequence first = list.Sequences[0];
        Sequence second = list.Sequences[1];
        int capturedSignature = state.DeorbitEngineSignature;
        try
        {
            list.MoveSequence(first, null);
            int currentSignature = (int)AccessTools.Method(typeof(GuidanceWindow), "DeorbitEngineSignature")
                .Invoke(null, new object[] { craft })!;
            GuidanceWindow.ApplyAutopilot(craft);
            t.Check("a sequence edit invalidates the captured braking model before node creation",
                currentSignature != capturedSignature && state.LandingPhase == GuidanceWindow.LandingPhase.DeorbitPlanning
                && state.DeorbitNode == null && !state.DeorbitNodeClaimed, state.LandingStatus);
        }
        finally { list.MoveSequence(first, second); }
    }

    private static void CheckAngleSnapshot(TestContext t, VehicleAutopilotState state, System.Reflection.FieldInfo ambient)
    {
        ambient.SetValue(null, state);
        var settings = AccessTools.Method(typeof(GuidanceWindow), "CurrentDeorbitSettings");
        state.DeorbitArrivalAngleEnabled = true;
        state.DeorbitArrivalDescentDeg = 5;
        var captured = (DeorbitSettings)settings.Invoke(null, null)!;
        state.DeorbitArrivalDescentDeg = 6;
        var changed = (DeorbitSettings)settings.Invoke(null, null)!;
        t.Check("the angle is captured and a change invalidates the request", captured.ArrivalDescentDeg == 5 && changed != captured);
        state.DeorbitArrivalAngleEnabled = false;
        state.DeorbitArrivalDescentDeg = double.NaN;
        var automatic = (DeorbitSettings)settings.Invoke(null, null)!;
        t.Check("automatic ignores the inactive angle value", automatic.ArrivalDescentDeg == null && automatic.IsValid);
        state.DeorbitArrivalDescentDeg = 5;
        state.DirectBrakingPlan = new(1000, default, default, 1000, "");
        state.LandingPhase = GuidanceWindow.LandingPhase.Prep;
        var targetLocked = AccessTools.Property(typeof(GuidanceWindow), "DeorbitTargetLocked");
        t.Check("a committed direct approach locks its angle", (bool)targetLocked.GetValue(null)!);
        state.LandingPhase = GuidanceWindow.LandingPhase.Coast;
        t.Check("a waiting direct coast can still replan its angle", !(bool)targetLocked.GetValue(null)!);
        state.DirectBrakingPlan = null;
        state.LandingPhase = GuidanceWindow.LandingPhase.Idle;
    }

    private static void CheckPriorModeRelease(TestContext t, DeorbitFixture fixture, VehicleAutopilotState state,
        System.Reflection.FieldInfo ambient)
    {
        Vehicle craft = fixture.Vehicle;
        Seed(fixture, state);
        state.Running = true;
        state.ControlAcquired = true;
        VehicleControlOwnership.TryClaim(craft, ControlClaimant.Guidance, out _);
        TestSupport.SetManualControlInputs(craft, 0.63f, engineOn: true);
        ambient.SetValue(null, state);
        AccessTools.Method(typeof(GuidanceWindow), "ExecuteLanding").Invoke(null, new object[] { craft });
        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("landing releases a previous powered mode before waiting for idle engines",
            state.LandingPhase == GuidanceWindow.LandingPhase.DeorbitPlanning
            && !state.ControlAcquired && !Inputs(craft).EngineOn
            && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.None);
    }

    private static void CheckControlSwitches(TestContext t, DeorbitFixture fixture, VehicleAutopilotState state)
    {
        foreach (bool disableEngage in new[] { true, false })
        {
            Seed(fixture, state);
            state.LandingPhase = GuidanceWindow.LandingPhase.Coast;
            if (disableEngage) state.Engage = false; else state.AutoStage = false;
            TestSupport.SetManualControlInputs(fixture.Vehicle, 0.63f, engineOn: false);
            GuidanceWindow.ApplyAutopilot(fixture.Vehicle);
            t.Check($"direct coast stops when {(disableEngage ? "engagement" : "automatic engines")} is disabled",
                state.LandingPhase == GuidanceWindow.LandingPhase.Done && !state.ControlAcquired
                && !Inputs(fixture.Vehicle).EngineOn);
        }
        state.Engage = state.AutoStage = true;
    }

    internal static void Seed(DeorbitFixture fixture, VehicleAutopilotState state, double deltaV = 10)
    {
        Vehicle craft = fixture.Vehicle;
        DeorbitRequest saved = fixture.Request;
        double now = Universe.GetElapsedTime().Seconds();
        state.Engage = state.AutoStage = true;
        state.LandingCutPending = false;
        state.StageModel = saved.Model;
        DeorbitFixture.SetSettings(state, saved.Settings);
        state.DeorbitRequest = new DeorbitRequest(craft.Orbit, now, craft.TotalMass, craft.BoundingSphereRadiusBody,
            saved.Settings, saved.Model, saved.Engines, saved.MinimumPulse, saved.ControlStep);
        state.DeorbitEngineSignature = (int)AccessTools.Method(typeof(GuidanceWindow), "DeorbitEngineSignature")
            .Invoke(null, new object[] { craft })!;
        double lead = float.IsFinite(craft.FlightComputer.ConservativeFlipTime)
            ? Math.Max(600, 2 * craft.FlightComputer.ConservativeFlipTime + 60) : 600;
        double departureTime = now + lead;
        double arrivalTime = departureTime + 3600;
        StateVectors source = craft.Orbit.GetStateVectorsAt(new UniverseTime(departureTime));
        double3 dv = -source.VelocityCci.Normalized() * deltaV;
        Orbit transfer = Orbit.CreateFromStateCci(saved.Parent, source.StateTime, source.PositionCci,
            source.VelocityCci + dv, craft.Orbit.OrbitLineColor);
        StateVectors arrival = transfer.GetStateVectorsAt(new UniverseTime(arrivalTime));
        state.DeorbitPlan = new(departureTime, arrivalTime, source.PositionCci, source.VelocityCci + dv, dv,
            arrival.PositionCci, arrival.VelocityCci, 1000, 1000, craft.TotalMass);
        state.BurnStartTime = arrivalTime;
        state.LandingPhase = GuidanceWindow.LandingPhase.DeorbitCoast;
    }

    internal static bool Arm(TestContext t, DeorbitFixture fixture, VehicleAutopilotState state)
    {
        GuidanceWindow.ApplyAutopilot(fixture.Vehicle);
        for (int i = 0; i < 4 && state.LandingPhase == GuidanceWindow.LandingPhase.DeorbitNodePending; i++)
        {
            t.Session.CreateDriver().Step(0.01);
            GuidanceWindow.ApplyAutopilot(fixture.Vehicle);
        }
        return t.Check("the deorbit node is loaded and stock Auto is armed",
            state.LandingPhase == GuidanceWindow.LandingPhase.DeorbitBurn && state.DeorbitNode != null
            && ReferenceEquals(fixture.Vehicle.FlightComputer.BurnPlan.FindFirstExecutableBurn(), state.DeorbitNode)
            && fixture.Vehicle.FlightComputer.BurnMode == FlightComputerBurnMode.Auto
            && !state.ControlAcquired && VehicleControlOwnership.HolderOf(fixture.Vehicle) == ControlClaimant.Guidance,
            $"{state.LandingStatus} {StockTiming(fixture.Vehicle, state)}");
    }

    private static string StockTiming(Vehicle craft, VehicleAutopilotState state)
    {
        FlightComputer fc = craft.FlightComputer;
        BurnTarget? target = state.DeorbitTarget ?? fc.Burn;
        double now = Universe.GetElapsedTime().Seconds();
        return target == null ? "No stock target."
            : $"Ignition in {target.IgnitionTime.Seconds() - now:F1} s, duration {target.BurnDuration:F2} s, flip time {fc.ConservativeFlipTime:F1} s"
            + $", planned throttle {fc.PlannedBurnThrottle:F3}, engine minimum {fc.ActiveEnginePerformanceMax.MinThrottle:F3}"
            + $", node in {(state.DeorbitPlan?.DepartureTime ?? double.NaN) - now:F1} s, arrival in {(state.DeorbitPlan?.ArrivalTime ?? double.NaN) - now:F1} s.";
    }

    private static void CheckLateIgnition(TestContext t, DeorbitFixture fixture, VehicleAutopilotState state)
    {
        Seed(fixture, state);
        TestSupport.SetManualControlInputs(fixture.Vehicle, 0.63f, engineOn: false);
        GuidanceWindow.ApplyAutopilot(fixture.Vehicle);
        if (!t.Check("the timing preview waits in Manual", state.LandingPhase == GuidanceWindow.LandingPhase.DeorbitNodePending
            && fixture.Vehicle.FlightComputer.BurnMode == FlightComputerBurnMode.Manual)) return;
        Burn node = state.DeorbitNode!;
        BurnTarget target = state.DeorbitTarget!;
        target.BurnDuration = 300;
        target.IgnitionTime = new UniverseTime(Universe.GetElapsedTime().Seconds() - 10);
        GuidanceWindow.ApplyAutopilot(fixture.Vehicle);
        t.Check("late ignition cannot arm Auto and keeps the node", state.LandingPhase == GuidanceWindow.LandingPhase.Done
            && fixture.Vehicle.FlightComputer.BurnMode == FlightComputerBurnMode.Manual
            && ReferenceEquals(fixture.Vehicle.FlightComputer.BurnPlan.FindFirstExecutableBurn(), node)
            && !Inputs(fixture.Vehicle).EngineOn && Inputs(fixture.Vehicle).EngineThrottle == 0.63f
            && VehicleControlOwnership.HolderOf(fixture.Vehicle) == ControlClaimant.None, state.LandingStatus);
        fixture.Vehicle.FlightComputer.RemoveBurn(node);
    }

    private static void CheckContact(TestContext t, DeorbitFixture fixture, VehicleAutopilotState state)
    {
        Seed(fixture, state);
        if (!Arm(t, fixture, state)) return;
        var properties = AccessTools.FieldRefAccess<Vehicle, VehicleProperties>("_props");
        Situation previous = properties(fixture.Vehicle).Situation;
        try
        {
            properties(fixture.Vehicle).Situation = Situation.Landed;
            GuidanceWindow.ApplyAutopilot(fixture.Vehicle);
            t.Check("contact stops the stock burn before its residual completes", state.LandingPhase == GuidanceWindow.LandingPhase.Done
                && fixture.Vehicle.FlightComputer.BurnMode == FlightComputerBurnMode.Manual && !Inputs(fixture.Vehicle).EngineOn
                && state.DeorbitNode == null && VehicleControlOwnership.HolderOf(fixture.Vehicle) == ControlClaimant.None,
                state.LandingStatus);
        }
        finally { properties(fixture.Vehicle).Situation = previous; }
    }

    private static void CheckLongMinimumPreview(TestContext t, DeorbitFixture fixture, VehicleAutopilotState state)
    {
        Seed(fixture, state, deltaV: 424.74);
        TestSupport.SetManualControlInputs(fixture.Vehicle, 0.63f, engineOn: false);
        if (!Arm(t, fixture, state)) return;
        t.Check("a long minimum-throttle preview can arm stock Auto when the burn window fits",
            state.DeorbitTarget!.BurnDuration > 0.01 * (state.DeorbitPlan!.ArrivalTime - state.DeorbitPlan.DepartureTime),
            $"Minimum-throttle duration {state.DeorbitTarget.BurnDuration:F1} s");
        state.FcResetPending = true;
        GuidanceWindow.ApplyAutopilot(fixture.Vehicle);
    }

    private static void CheckActiveBurnWait(TestContext t, DeorbitFixture fixture, VehicleAutopilotState state)
    {
        Seed(fixture, state);
        Vehicle craft = fixture.Vehicle;
        double time = state.DeorbitPlan!.DepartureTime;
        PatchedConic? patch = craft.FlightPlan.TryFindPatch(new UniverseTime(time));
        if (!t.Check("the active-burn fixture has a stock flight-plan patch", patch != null)) return;
        Burn node = Burn.Create(patch!.Orbit.GetPointAt(new UniverseTime(time)), time, new double3(-10, 0, 0), patch, craft);
        craft.FlightComputer.AddBurn(node);
        craft.FlightComputer.BurnMode = FlightComputerBurnMode.Auto;
        BurnTarget target = craft.FlightComputer.Burn!;
        state.LandingPhase = GuidanceWindow.LandingPhase.DeorbitPlanning;
        state.DeorbitPlanner = new DeorbitPlanner(state.DeorbitRequest!);
        TestSupport.SetManualControlInputs(craft, 0.63f, engineOn: false);
        try
        {
            for (int i = 0; i < 300; i++) GuidanceWindow.ApplyAutopilot(craft);
            t.Check("planning does not capture or restart while another stock burn is active",
                state.LandingPhase == GuidanceWindow.LandingPhase.DeorbitPlanning
                && state.DeorbitPlanner == null && state.DeorbitRequest == null);
            t.Check("waiting preserves the active stock burn and manual inputs",
                ReferenceEquals(craft.FlightComputer.Burn, target) && craft.FlightComputer.BurnMode == FlightComputerBurnMode.Auto
                && Inputs(craft).EngineThrottle == 0.63f && !Inputs(craft).EngineOn && !state.ControlAcquired);
            craft.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
            TestSupport.SetManualControlInputs(craft, 0.63f, engineOn: true);
            GuidanceWindow.ApplyAutopilot(craft);
            t.Check("planning also waits for a manual burn", state.DeorbitPlanner == null && state.DeorbitRequest == null
                && Inputs(craft).EngineOn);
        }
        finally
        {
            craft.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
            craft.FlightComputer.RemoveBurn(node);
            TestSupport.SetManualControlInputs(craft, 0.63f, engineOn: false);
        }
    }

    private static void CheckDifferentTarget(TestContext t, DeorbitFixture fixture, VehicleAutopilotState state)
    {
        Seed(fixture, state);
        if (!Arm(t, fixture, state)) return;
        Vehicle craft = fixture.Vehicle;
        Burn own = state.DeorbitNode!;
        Burn other = Burn.Create(OrbitPointCce.Zero, own.Time.Seconds() - 30, new double3(2, 0, 0), own.Patch, craft);
        craft.FlightComputer.AddBurn(other);
        BurnTarget replacement = craft.FlightComputer.Burn!;
        StockBurnMode.GiveBackAuto(craft, replacement);
        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("a different target keeps its stock Auto and measured state",
            craft.FlightComputer.BurnMode == FlightComputerBurnMode.Auto && ReferenceEquals(craft.FlightComputer.Burn, replacement)
            && state.LandingPhase == GuidanceWindow.LandingPhase.Done && !state.DeorbitNodeClaimed);
        craft.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
        craft.FlightComputer.RemoveBurn(other);
        craft.FlightComputer.RemoveBurn(own);
    }

    private static void CheckCompletionAfterRemoval(TestContext t, DeorbitFixture fixture, VehicleAutopilotState state)
    {
        Seed(fixture, state);
        if (!Arm(t, fixture, state)) return;
        Vehicle craft = fixture.Vehicle;
        Burn node = state.DeorbitNode!;
        BurnTarget target = craft.FlightComputer.Burn!;
        // These stock observations isolate lifecycle handling from the separate engine-flight test.
        target.DeltaVAccumCci = target.DeltaVTargetCci * 1.01f;
        craft.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
        craft.FlightComputer.RemoveBurn(node);
        GuidanceWindow.ApplyAutopilot(craft);
        t.Check("confirmed completion survives stock node removal and starts AFC braking preparation",
            state.LandingPhase == GuidanceWindow.LandingPhase.TransferPlanning && state.ControlAcquired
            && state.DeorbitNode == null && state.DeorbitPlan != null && !Inputs(craft).EngineOn
            && VehicleControlOwnership.HolderOf(craft) == ControlClaimant.Guidance, state.LandingStatus);
    }

    private static void CheckPlanningWait(TestContext t, DeorbitFixture fixture, VehicleAutopilotState state)
    {
        double previousSpeed = Universe.SimulationSpeed;
        try
        {
            state.DeorbitRequest = fixture.Request;
            state.DeorbitPlanner = new DeorbitPlanner(fixture.Request);
            state.LandingPhase = GuidanceWindow.LandingPhase.DeorbitPlanning;
            TestSupport.SetManualControlInputs(fixture.Vehicle, 0.63f, engineOn: false);
            Universe.SetSimulationSpeed(100, alert: false);
            GuidanceWindow.ApplyAutopilot(fixture.Vehicle);
            t.Check("planning waits at high warp without a captured search", state.LandingPhase == GuidanceWindow.LandingPhase.DeorbitPlanning
                && state.DeorbitRequest == null && state.DeorbitPlanner == null && state.LandingStatus.Contains("1x"));
            t.Check("planning at high warp keeps the engine input and takes no control",
                !Inputs(fixture.Vehicle).EngineOn && Inputs(fixture.Vehicle).EngineThrottle == 0.63f && !state.ControlAcquired);
        }
        finally { Universe.SetSimulationSpeed(previousSpeed, alert: false); }
    }

    private static void CheckDirectCoast(TestContext t, DeorbitFixture fixture, VehicleAutopilotState state)
    {
        double now = Universe.GetElapsedTime().Seconds();
        DeorbitRequest saved = fixture.Request;
        state.Engage = state.AutoStage = true;
        DeorbitFixture.SetSettings(state, saved.Settings);
        var request = new DeorbitRequest(fixture.Vehicle.Orbit, now, fixture.Vehicle.TotalMass * 1.01, saved.VehicleRadius,
            saved.Settings, saved.Model, saved.Engines, saved.MinimumPulse, saved.ControlStep);
        state.DeorbitRequest = request;
        state.DirectBrakingPlan = new(now + 600, default, default, 1000, "");
        state.BurnStartTime = now + 600;
        state.LandingPhase = GuidanceWindow.LandingPhase.Coast;
        GuidanceWindow.ApplyAutopilot(fixture.Vehicle);
        t.Check("mass drift in direct coast keeps the accepted plan", state.LandingPhase == GuidanceWindow.LandingPhase.Coast
            && ReferenceEquals(state.DeorbitRequest, request) && state.BurnStartTime == now + 600 && state.DeorbitPlanner == null);
    }
}

public sealed class GuidanceStockDeorbitTest : AfcTest
{
    public override string Name => "afc-guidance-stock-deorbit";

    protected override void Execute(TestContext t)
    {
        if (VehicleAutopilotState.Snapshot().Length != 0)
        { t.Skip("another craft holds guidance state."); return; }
        using DeorbitFixture? fixture = DeorbitFixture.Open(t, "High Luna Orbit");
        if (fixture == null) return;
        Vehicle craft = fixture.Vehicle;
        VehicleAutopilotState state = VehicleAutopilotState.For(craft);
        var ambient = AccessTools.Field(typeof(GuidanceWindow), "_s");
        object? previous = ambient.GetValue(null);
        bool rails = PhysicsBubble._forceOffRails;
        try
        {
            GuidanceDeorbitLifecycleTest.Seed(fixture, state, deltaV: 20);
            if (!GuidanceDeorbitLifecycleTest.Arm(t, fixture, state)) return;
            BurnTarget target = craft.FlightComputer.Burn!;
            double mass = craft.TotalMass;
            PhysicsBubble._forceOffRails = true;
            SimDriver driver = t.Session.CreateDriver();
            double deadline = state.DeorbitPlan!.DepartureTime + 60;
            for (int i = 0; i < 120000 && Universe.GetElapsedTime().Seconds() < deadline
                && state.LandingPhase == GuidanceWindow.LandingPhase.DeorbitBurn; i++)
            {
                GuidanceWindow.ApplyAutopilot(craft);
                driver.Step(0.02);
            }
            t.Check("the real stock controller completes the small node and returns to AFC",
                state.LandingPhase == GuidanceWindow.LandingPhase.TransferPlanning
                    || state.LandingPhase == GuidanceWindow.LandingPhase.TransferCoast, state.LandingStatus);
            t.Check("the game measured the requested impulse", target.DeltaVAccumCci.Length() > 19
                && float3.Dot(target.DeltaVToGoCci, target.DeltaVTargetCci) <= 0);
            t.Check("the stock burn consumed propellant", craft.TotalMass < mass);
            t.Check("AFC owns the braking handoff in Manual", state.ControlAcquired
                && craft.FlightComputer.BurnMode == FlightComputerBurnMode.Manual);
        }
        finally
        {
            ambient.SetValue(null, state);
            AccessTools.Method(typeof(GuidanceWindow), "HandBackVehicle").Invoke(null, new object[] { craft });
            PhysicsBubble._forceOffRails = rails;
            ambient.SetValue(null, previous);
        }
    }
}

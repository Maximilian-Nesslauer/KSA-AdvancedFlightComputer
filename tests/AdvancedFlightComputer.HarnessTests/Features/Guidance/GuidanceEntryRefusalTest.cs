using System.Reflection;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class GuidanceEntryRefusalTest : AfcTest
{
    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;
    public override string Name => "afc-guidance-entry-refusal";

    protected override void Execute(TestContext t)
    {
        CheckCancellation(t);
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        if (t.System.All.Get("Luna") is not IParentBody moon)
        {
            t.Skip("Luna is unavailable");
            return;
        }
        FieldInfo ambient = typeof(GuidanceWindow).GetField("_s", PrivateStatic)!;
        object? prior = ambient.GetValue(null);
        Vehicle airless = Spawn(moon, "HarnessEntryAirless");
        Vehicle atmosphere = Spawn(home, "HarnessEntryNoEngine");
        try
        {
            var engage = Method("Engage3Dof");
            double now = Universe.GetElapsedSeconds();
            var airlessState = VehicleAutopilotState.For(airless);
            ambient.SetValue(null, airlessState);
            bool entered = (bool)engage.Invoke(null, [airless, now])!;
            t.Check("entry refuses without an atmosphere",
                !entered && airlessState.ThreeDofStatus.Contains("NO ATMOSPHERE", StringComparison.Ordinal));

            var state = VehicleAutopilotState.For(atmosphere);
            ambient.SetValue(null, state);
            entered = (bool)engage.Invoke(null, [atmosphere, now])!;
            t.Check("entry refuses without a supplied engine",
                !entered && state.ThreeDofStatus.Contains("NO ACTIVE, SUPPLIED ENGINE", StringComparison.Ordinal));

            FieldInfo manualField = typeof(Vehicle).GetField("_manualControlInputs", BindingFlags.NonPublic | BindingFlags.Instance)!;
            var manual = (ManualControlInputs)manualField.GetValue(atmosphere)!;
            manual.EngineOn = true;
            manual.EngineThrottle = 0.65f;
            manualField.SetValue(atmosphere, manual);
            state.Active = true;
            entered = (bool)engage.Invoke(null, [atmosphere, now])!;
            t.Check("entry leaves the powered touchdown mode in control",
                !entered && state.Active && state.ThreeDofStatus.Contains("ANOTHER MODE", StringComparison.Ordinal));

            state.Active = false;
            state.LandingPhase = GuidanceWindow.LandingPhase.DeorbitBurn;
            atmosphere.FlightComputer.BurnMode = FlightComputerBurnMode.Auto;
            manual.EngineOn = false;
            manualField.SetValue(atmosphere, manual);
            var harmony = new Harmony("com.maxi.afc.harness.entry-powered-request");
            try
            {
                harmony.Patch(AccessTools.DeclaredMethod(typeof(Vehicle), nameof(Vehicle.ComputeLiveThrust)),
                    prefix: new HarmonyMethod(typeof(GuidanceEntryRefusalTest), nameof(ReportLitEngine)));
                Method("RequestEntryGuidance").Invoke(null, [atmosphere]);
                t.Check("the console request leaves a lit stock deorbit in Auto",
                    state.LandingPhase == GuidanceWindow.LandingPhase.DeorbitBurn && !state.ThreeDofEngagePending
                    && atmosphere.FlightComputer.BurnMode == FlightComputerBurnMode.Auto
                    && state.ThreeDofStatus.Contains("ANOTHER MODE", StringComparison.Ordinal));
            }
            finally
            {
                harmony.UnpatchAll(harmony.Id);
            }
            atmosphere.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
            state.LandingPhase = GuidanceWindow.LandingPhase.Idle;
            state.ThreeDofPhase = GuidanceWindow.ThreeDofPhase.Burn;
            state.ThreeDofEngineOn = true;
            state.ThreeDofThrottle = 0.65;
            state.ThreeDofBurnSolveFault = true;
            Method("StepEntryTouchdownHandoff").Invoke(null, [atmosphere, home, new double3(), new double[14], now]);
            t.Check("a refused touchdown handover leaves the entry burn lit",
                state.ThreeDofPhase == GuidanceWindow.ThreeDofPhase.Burn && state.ThreeDofEngineOn
                && state.ThreeDofThrottle == 0.65 && !state.LandingCutPending
                && state.ThreeDofStatus.Contains("TOUCHDOWN UNAVAILABLE", StringComparison.Ordinal),
                state.ThreeDofStatus);
            state.DiagEntry("Burning on the plan.");
            t.Check("entry tracking diagnostics preserve the refused handover warning",
                state.ThreeDofStatus.Contains("TOUCHDOWN UNAVAILABLE", StringComparison.Ordinal));

            FlightComputer fc = atmosphere.FlightComputer;
            fc.RcsTorqueAuthority = float3.Zero;
            fc.TvcTorqueAuthority = new float3(0.0f, 2.0f, 3.0f);
            t.CheckAbs("entry uses the weaker gimbal angular acceleration when RCS gives none",
                Ksa3DofSetup.TurnAccelLimit(atmosphere), 1.5, 1e-6);
        }
        finally
        {
            ambient.SetValue(null, prior);
            VehicleAutopilotState.Remove(airless);
            VehicleAutopilotState.Remove(atmosphere);
            VehicleSpawner.Despawn(atmosphere);
            VehicleSpawner.Despawn(airless);
        }

        Vehicle Spawn(IParentBody body, string id)
        {
            var design = new PartInstance { InstanceOf = "CoreCommandC_Prefab_ProbeCoreSmallA", LocalInstanceId = 1 };
            return VehicleFixtures.SpawnDesign(t.System, body, design, id, OrbitFixtures.CircularAt(body, 400_000.0, Universe.GetElapsedTime()));
        }
    }

    private static MethodInfo Method(string name)
        => typeof(GuidanceWindow).GetMethod(name, PrivateStatic) ?? throw new MissingMethodException(nameof(GuidanceWindow), name);

    private static bool ReportLitEngine(ref float __result)
    {
        __result = 100.0f;
        return false;
    }

    private static void CheckCancellation(TestContext t)
    {
        object oldGuidance = new(), newGuidance = new();
        using var oldWorker = new Ksa3DofWorker();
        using var nextWorker = new Ksa3DofWorker();
        using var began = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        using var newFinished = new ManualResetEventSlim();
        bool observedCancellation = false;
        t.Check("the old solve starts", oldWorker.TryRun(oldGuidance, stop =>
        {
            began.Set();
            release.Wait(TimeSpan.FromSeconds(3));
            observedCancellation = stop.IsCancellationRequested;
            throw new InvalidOperationException("old cancelled entry task");
        }) && began.Wait(TimeSpan.FromSeconds(2)));
        var state = new VehicleAutopilotState { ThreeDofWorker = oldWorker };
        state.CancelEntrySolve();
        t.Check("a new solve can finish while the old one unwinds",
            nextWorker.TryRun(newGuidance, _ => newFinished.Set()) && newFinished.Wait(TimeSpan.FromSeconds(2)));
        release.Set();
        t.Check("the old job sees cancellation and its fault cannot affect the new guidance",
            SpinWait.SpinUntil(() => !oldWorker.Busy, TimeSpan.FromSeconds(2)) && observedCancellation
            && oldWorker.TakeError(newGuidance).Length == 0 && nextWorker.TakeError(newGuidance).Length == 0);
    }
}

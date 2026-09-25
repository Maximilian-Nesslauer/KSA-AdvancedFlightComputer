using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// What EXECUTE does on the Ascent tab now that the convex profile is the ascent and the deg/s gravity turn only its
// backup. Nothing is flown: each case commits, checks what was committed to, and aborts.
//
// Without a plan, EXECUTE calculates one and launches on it once it converges - asked for the script's 35 kPa, which a KSA stack cannot hold at full throttle, so the first attempt does not converge and the panel is told the retry is looser. With a plan that fits, it launches at
// once. A plan solved to other limits does not fit, so EXECUTE calculates again, and ABORT cancels that launch. A plan
// that cannot be calculated launches nothing: the panel says why, and its backup button flies the vertical rise and
// gravity turn.
//
// The pad is the convex flight test's: a spawn a few hundred metres up, co-rotating and climbing.
public sealed class GuidanceConvexExecuteTest : AfcTest
{
    public override string Name => "afc-guidance-convex-execute";

    private static readonly string[] Saves = { "Test Vehicle 1", "Test Vehicle 2" };

    private const double SiteLatDeg = 28.5;
    private const double PadHeightM = 300.0;
    private const double SpawnClimbMs = 15.0;
    private const double TargetAltKm = 200.0;
    private const double TargetIncDeg = 35.0;
    private const double PlanTimeoutS = 180.0;

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(
            AccessTools.Field(typeof(GuidanceWindow), "_s"));

    private static readonly MethodInfo ExecuteAscent = AccessTools.Method(typeof(GuidanceWindow), "ExecuteAscent");
    private static readonly MethodInfo AbortAscent = AccessTools.Method(typeof(GuidanceWindow), "AbortAscent");
    private static readonly MethodInfo LaunchBackupAscent = AccessTools.Method(typeof(GuidanceWindow), "LaunchBackupAscent");
    private static readonly MethodInfo LanOverhead = AccessTools.Method(typeof(GuidanceWindow), "LanOverhead");

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        if (home is not Celestial body)
        {
            t.Skip("the home body is not a celestial with terrain.");
            return;
        }
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(Saves);
        if (saves.Count == 0)
        {
            t.Skip($"none of '{string.Join("', '", Saves)}' is in the game's Vehicles folder.");
            return;
        }
        if (VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state.");
            return;
        }

        Vehicle? previousFocus = Program.ControlledVehicle;
        VehicleAutopilotState previousAmbient = AmbientState();
        bool previousModActive = GuidanceWindow.ModActive;
        bool previousEnabled = SharedVehicleHooks.GuidanceEnabled;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        using AutoStageTestPatches.Scope staging = AutoStageTestPatches.Apply();
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.convex-execute");
        try
        {
            VehicleCommandSink.ApplyPatches(harmony);
            GuidanceFeature.ApplyDriverPatches(harmony);
            SharedVehicleHooks.GuidanceEnabled = true;
            GuidanceWindow.SetModActive(true);

            double lat = SiteLatDeg * Math.PI / 180.0;
            var dirCcf = new double3(Math.Cos(lat), 0.0, Math.Sin(lat));
            double terrain = body.GetTerrainHeightFromDirCcf(dirCcf);
            if (!double.IsFinite(terrain) || terrain < 0.0)
                terrain = 0.0;
            double3 dirCci = double3.Normalize(dirCcf.Transform(home.GetCcf2Cci()));
            double3 r = dirCci * (home.MeanRadius + terrain + PadHeightM);
            double3 v = double3.Cross(new double3(0, 0, home.GetAngularVelocity()), r) + SpawnClimbMs * dirCci;
            Orbit pad = Orbit.CreateFromStateCci(home, Universe.GetElapsedTime(), r, v, VehicleSpawner.OrbitLineColor);
            Vehicle vehicle;
            try
            {
                vehicle = VehicleSpawner.SpawnFromSave(saves[0], t.System, home, "HarnessConvexExecute", pad);
            }
            catch (InvalidOperationException e)
            {
                t.Fail("spawn", e.Message);
                return;
            }
            t.Info($"'{saves[0]}' spawned, {vehicle.TotalMass / 1000.0:F1} t");
            Program.ControlledVehicle = vehicle;
            PhysicsBubble._forceOffRails = true;
            driver.Step(0.05, 2);

            Run(t, vehicle, home, driver);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            GuidanceFeature.DisableDriver();
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.SetModActive(previousModActive);
            AmbientState() = previousAmbient;
            AutoStageFlightSupport.CleanupAfterFlight(t.System, preexisting);
            Program.ControlledVehicle = previousFocus;
        }
    }

    private static void Run(TestContext t, Vehicle vehicle, IParentBody home, SimDriver driver)
    {
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        state.PeKm = TargetAltKm;
        state.ApKm = TargetAltKm;
        state.IncDeg = TargetIncDeg;
        state.LanDeg = (double)LanOverhead.Invoke(null, new object[] { vehicle.Orbit.StateVectors.PositionCci, TargetIncDeg, home })!;
        state.LanSeeded = true;
        state.Engage = true;
        state.AutoStage = false;
        state.TargetId = "";
        state.FlyConvexAscent = true;
        state.ConvexThrottleMinPct = 99.0;
        state.ConvexQMaxKpa = 35.0;
        state.AscentPlan = null;
        AmbientState() = state;
        object[] args = { vehicle, vehicle.Orbit, home };

        // No plan: EXECUTE asks for one and waits.
        ExecuteAscent.Invoke(null, args);
        if (!t.Check("EXECUTE without a plan calculates one first",
                state.ConvexLaunchPending && !state.Running && state.AscentPlanRequested,
                $"pending {state.ConvexLaunchPending}, running {state.Running}, requested {state.AscentPlanRequested}"))
            return;
        driver.Step(0.02);
        AscentPlanJob? job = state.AscentPlanJob;
        if (!t.Check("the sim step starts the solve", job != null, $"status '{state.AscentPlanStatus}'"))
            return;
        // The sim is not stepped while it solves, so the vehicle is still on its pad when the plan comes back.
        var clock = System.Diagnostics.Stopwatch.StartNew();
        string retry = "";
        while (!job!.IsDone && clock.Elapsed.TotalSeconds < PlanTimeoutS)
        {
            if (retry.Length == 0)
                retry = job.Retry;
            Thread.Sleep(50);
        }
        if (!t.Check($"the solve finishes within {PlanTimeoutS:F0} s", job.IsDone, job.Progress))
        {
            job.Cancel();
            return;
        }
        driver.Step(0.02);
        if (!t.Check("the plan converged", state.AscentPlan?.Usable == true, $"status '{state.AscentPlanStatus}', failure '{state.ConvexLaunchFailure}'"))
            return;
        t.Check("while it ran, the retry said the first attempt did not converge and what was loosened",
            retry.StartsWith("Did not converge", StringComparison.Ordinal) && retry.Contains("looser constraints", StringComparison.Ordinal), $"'{retry}'");
        t.Check("and the plan's status says so too", state.AscentPlanStatus.Contains("did not converge", StringComparison.Ordinal)
            && state.AscentPlanStatus.Contains("looser constraints", StringComparison.Ordinal), $"'{state.AscentPlanStatus}'");
        t.Check("and EXECUTE launches on it without being pressed again",
            state.Running && state.Phase == GuidanceWindow.AscentPhase.Profile && ReferenceEquals(state.FlyingPlan, state.AscentPlan)
            && !state.ConvexLaunchPending,
            $"running {state.Running}, phase {state.Phase}, pending {state.ConvexLaunchPending}");
        AbortAscent.Invoke(null, null);
        t.Check("ABORT stops it", !state.Running, $"running {state.Running}");

        // A plan that fits: EXECUTE launches at once, asking for nothing.
        ExecuteAscent.Invoke(null, args);
        t.Check("EXECUTE with a plan that fits launches at once",
            state.Running && state.Phase == GuidanceWindow.AscentPhase.Profile && !state.ConvexLaunchPending
            && !state.AscentPlanRequested && state.AscentPlanJob == null,
            $"running {state.Running}, phase {state.Phase}, pending {state.ConvexLaunchPending}, requested {state.AscentPlanRequested}");
        AbortAscent.Invoke(null, null);

        // Other limits: the plan no longer fits, so EXECUTE calculates again - and ABORT cancels that launch before it starts.
        double qAlpha = state.ConvexQAlphaMax;
        state.ConvexQAlphaMax = qAlpha - 500.0;
        ExecuteAscent.Invoke(null, args);
        t.Check("a plan solved to other limits is calculated again", state.ConvexLaunchPending && !state.Running,
            $"pending {state.ConvexLaunchPending}, running {state.Running}");
        AbortAscent.Invoke(null, null);
        t.Check("ABORT cancels a launch waiting on its plan", !state.ConvexLaunchPending && !state.AscentPlanRequested && !state.Running,
            $"pending {state.ConvexLaunchPending}, requested {state.AscentPlanRequested}, running {state.Running}");
        driver.Step(0.02);
        t.Check("and no solve starts after it", state.AscentPlanJob == null, $"job {(state.AscentPlanJob == null ? "none" : "running")}");
        state.ConvexQAlphaMax = qAlpha;

        // A plan that cannot be calculated: nothing launches, the panel says why, and the backup is the player's call.
        state.PeKm = -100.0;
        ExecuteAscent.Invoke(null, args);
        driver.Step(0.02);
        t.Check("a plan that cannot be calculated launches nothing",
            !state.Running && !state.ConvexLaunchPending && state.ConvexLaunchFailure.Length > 0,
            $"running {state.Running}, pending {state.ConvexLaunchPending}, failure '{state.ConvexLaunchFailure}'");
        LaunchBackupAscent.Invoke(null, args);
        t.Check("the backup flies the vertical rise and gravity turn",
            state.Running && state.Phase == GuidanceWindow.AscentPhase.Vertical && state.FlyingPlan == null && state.BackupAscent
            && state.ConvexLaunchFailure.Length == 0,
            $"running {state.Running}, phase {state.Phase}, backup {state.BackupAscent}, failure '{state.ConvexLaunchFailure}'");
        AbortAscent.Invoke(null, null);
        t.Check("ABORT clears the backup", !state.Running && !state.BackupAscent, $"running {state.Running}, backup {state.BackupAscent}");
    }
}

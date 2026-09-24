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

// The convex ascent end to end, on a real vehicle: plan it through the production planner, fly the plan open
// loop, hand over to UPFG, and reach orbit.
//
// The harness cannot spawn a craft on the ground, so the pad is a spawn a few hundred metres above the terrain,
// co-rotating and climbing - which is where launch3dof.py starts its own vehicle, 100 m up and rising at 10 m/s.
//
// What it proves: Calculate builds the problem from the live part tree on the sim step and the worker converges
// it; EXECUTE flies the profile in place of the gravity turn, from lift-off to the hand-over altitude, with the
// vehicle's altitude following the plan's; UPFG takes over above it; and the orbit the game
// reports after cutoff is the one asked for. The planned mass to orbit is reported against the flown one.
public sealed class GuidanceConvexAscentTest : AfcTest
{
    public override string Name => "afc-guidance-convex-ascent";

    // A liquid-fuelled stack first: the plan models every stage at constant thrust, as the script does, and a solid motor's thrust curve is not that, so a solid first stage flies measurably off its plan (see GuidanceWindow's convex ascent notes).
    private static readonly string[] DefaultSaves = { "Test Vehicle 1", "Test Vehicle 2" };

    private const double SiteLatDeg = 28.5;
    private const double SiteLonDeg = 0.0;
    private const double PadHeightM = 300.0;
    private const double SpawnClimbMs = 15.0;
    private const double TargetAltKm = 200.0;
    private const double TargetIncDeg = 35.0;
    private const double HandoverAltKm = 80.0;
    private const double GLimitG = 4.0;

    private const double PlanTimeoutS = 180.0;
    private const double StepDt = 0.25;
    private const double TerminalStepDt = 0.02;
    private const double MaxFlightSeconds = 1500.0;
    private const double SampleIntervalS = 5.0;

    // How far the flown climb may sit off the plan's, at the same air speed, before the hand-over, km. Loose: the plan's drag is the stack's all the way up, and the profile absorbs a heavier or weaker vehicle by flying at the speed it has, not by matching altitude.
    private const double ProfileAltitudeTolKm = 15.0;
    private const double PeriapsisTolKm = 10.0;
    private const double ApoapsisTolKm = 25.0;
    private const double InclinationTolDeg = 0.5;

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(
            AccessTools.Field(typeof(GuidanceWindow), "_s"));

    private static readonly MethodInfo StartGuidance =
        AccessTools.Method(typeof(GuidanceWindow), "StartGuidance");

    private static readonly MethodInfo LanOverhead =
        AccessTools.Method(typeof(GuidanceWindow), "LanOverhead");

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        if (home is not Celestial body)
        {
            t.Skip("the home body is not a celestial with terrain.");
            return;
        }
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(DefaultSaves);
        if (saves.Count == 0)
        {
            t.Skip($"none of '{string.Join("', '", DefaultSaves)}' is in the game's Vehicles folder.");
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
        bool previousDropSpent = StagingConfig.DropSpentStages;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        using AutoStageTestPatches.Scope staging = AutoStageTestPatches.Apply();
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.convex-ascent");
        try
        {
            VehicleCommandSink.ApplyPatches(harmony);
            GuidanceFeature.ApplyDriverPatches(harmony);
            SharedVehicleHooks.GuidanceEnabled = true;
            GuidanceWindow.SetModActive(true);

            // The pad: over the site, a few hundred metres above its terrain, turning with the ground and climbing.
            double lat = SiteLatDeg * Math.PI / 180.0, lon = SiteLonDeg * Math.PI / 180.0;
            var dirCcf = new double3(Math.Cos(lat) * Math.Cos(lon), Math.Cos(lat) * Math.Sin(lon), Math.Sin(lat));
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
                vehicle = VehicleSpawner.SpawnFromSave(saves[0], t.System, home, "HarnessConvexAscent", pad);
            }
            catch (InvalidOperationException e)
            {
                t.Fail("spawn", e.Message);
                return;
            }
            t.Info($"'{saves[0]}' spawned {terrain + PadHeightM:F0} m above the mean radius at lat {SiteLatDeg:F1}, lon {SiteLonDeg:F1} "
                 + $"(terrain {terrain:F0} m), {vehicle.TotalMass / 1000.0:F1} t");

            Program.ControlledVehicle = vehicle;
            PhysicsBubble._forceOffRails = true;
            StagingConfig.EngineDelays.Clear();
            StagingConfig.DecouplerDelays.Clear();
            StagingConfig.DropSpentStages = true;
            driver.Step(0.05, 6);

            Fly(t, vehicle, home, driver, preexisting);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            GuidanceFeature.DisableDriver();
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.SetModActive(previousModActive);
            StagingConfig.DropSpentStages = previousDropSpent;
            AmbientState() = previousAmbient;
            AutoStageFlightSupport.CleanupAfterFlight(t.System, preexisting);
            Program.ControlledVehicle = previousFocus;
        }
    }

    private static void Fly(TestContext t, Vehicle vehicle, IParentBody home, SimDriver driver, HashSet<string> preexisting)
    {
        VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
        double3 r0 = vehicle.Orbit.StateVectors.PositionCci;
        state.PeKm = TargetAltKm;
        state.ApKm = TargetAltKm;
        state.IncDeg = TargetIncDeg;
        state.LanDeg = (double)LanOverhead.Invoke(null, new object[] { r0, TargetIncDeg, home })!;
        state.LanSeeded = true;
        state.ArgPeFixed = false;
        state.Engage = true;
        state.AutoStage = true;
        // The g-limit throttles only UPFG's closed loop (the profile flies full throttle, as planned): a stage that ends at ten g against a quarter-second step would otherwise cut off tens of m/s off.
        state.GLimitEnabled = true;
        state.GLimitG = GLimitG;
        state.ReserveArmed = false;
        state.TargetId = "";
        state.FlyConvexAscent = Environment.GetEnvironmentVariable("AFC_CONVEX_BASELINE") != "1";
        state.ConvexHandoverAltKm = HandoverAltKm;
        AmbientState() = state;

        // Calculate, the way the button asks: the sim step builds the problem and starts the worker. The sim is not
        // stepped while it solves, so the vehicle is still on its pad when the plan comes back.
        state.AscentPlanRequested = true;
        driver.Step(0.02);
        AscentPlanJob? job = state.AscentPlanJob;
        if (!t.Check("Calculate starts a solve", job != null, $"status '{state.AscentPlanStatus}'"))
            return;
        var clock = System.Diagnostics.Stopwatch.StartNew();
        while (!job!.IsDone && clock.Elapsed.TotalSeconds < PlanTimeoutS)
            Thread.Sleep(50);
        if (!t.Check($"the solve finishes within {PlanTimeoutS:F0} s", job.IsDone, job.Progress))
        {
            job.Cancel();
            return;
        }
        driver.Step(0.02);
        AscentPlan? plan = state.AscentPlan;
        foreach (string note in job.Notes)
            t.Info("attempt: " + note);
        if (!t.Check("the plan converged", plan?.Usable == true, $"status '{state.AscentPlanStatus}'"))
        {
            if (plan?.Solution != null)
                foreach (AscentIteration it in plan.Solution.Trace.TakeLast(8))
                    t.Info($"  iter {it.Index}: {it.Status} rho {it.Rho:F2} tr {it.TrustRegion:F3} pred {it.Predicted:E1} "
                         + $"defect {it.DefectNorm:E1} path {it.PathViolation:E1} term {it.TerminalViolation:E1}");
            return;
        }
        AscentSolution sol = plan!.Solution;
        // The script's 35 kPa is a Saturn V's: a KSA stack lifting off at three or five g cannot climb under it at full throttle, and the planner re-plans to a limit it can hold.
        t.Info($"max q planned to {plan.QMaxKpa:F0} kPa (asked {plan.QMaxRequestedKpa:F0}{(plan.QRelaxed ? ", out of reach" : "")})");
        t.Info($"plan: {sol.Message} in {sol.Iterations} iterations ({sol.Accepted} accepted), {plan.WallSeconds:F1} s wall, seed {sol.SeedSeconds:F1} s, kick {sol.KickDeg:F3} deg; "
             + $"{plan.StagesPlanned} of {plan.StagesAvailable} stages, burns {string.Join("/", sol.BurnTime.Select(b => b.ToString("F1")))} s, "
             + $"{sol.FinalMass / 1000.0:F2} t to orbit, max q {sol.DynamicPressure.Max() / 1000.0:F1} kPa, max q-alpha {sol.QAlpha.Max():F0} Pa rad, "
             + $"insertion miss {sol.TerminalResidual[0]:F0} m / {sol.TerminalResidual[1]:F2} m/s");
        ConvexAscentProfile profile = plan.Profile;
        for (int k = 0; k < profile.Count; k += Math.Max(1, profile.Count / 12))
            t.Info($"  profile: {profile.Speed[k],7:F0} m/s  pitch {profile.PitchRad[k] * 180.0 / Math.PI,5:F1} deg  "
                 + $"alt {profile.Altitude[k] / 1000.0,6:F1} km  t {profile.Time[k],5:F0} s");

        // Lit the way the other flight tests light it, then EXECUTE.
        if (!AutoStageFlightSupport.Arm(t, vehicle))
            return;
        vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
        driver.Step(1.0);
        StartGuidance.Invoke(null, new object[] { vehicle, vehicle.Orbit, home });
        if (!t.Check("EXECUTE flies the convex profile", state.Running && (state.Phase == GuidanceWindow.AscentPhase.Profile || !state.FlyConvexAscent),
                $"running {state.Running}, phase {state.Phase}, status '{state.Status}'"))
            return;

        double time = 1.0, nextSample = 0.0;
        double handoverAltKm = double.NaN, handoverTime = double.NaN, handoverSpeed = double.NaN;
        double worstDevKm = 0.0, worstDevSpeed = double.NaN;
        bool upfgConverged = false;
        while (time < MaxFlightSeconds && state.Running)
        {
            // Fine steps through the terminal count: the cutoff lands on a step, and at four g a quarter second is 10 m/s, which is 35 km of apoapsis.
            double dt = state.Phase == GuidanceWindow.AscentPhase.Terminal ? TerminalStepDt : StepDt;
            driver.Step(dt);
            time += dt;
            // A spent stage falls back to the ground minutes later, and the game's ground-impact effect throws in the headless runtime. The flight under test does not need them, so they go as soon as they separate.
            DespawnJettisoned(t.System, preexisting, vehicle);
            double3 r = vehicle.Orbit.StateVectors.PositionCci;
            double3 v = vehicle.Orbit.StateVectors.VelocityCci;
            double altKm = (r.Length() - home.MeanRadius) / 1000.0;
            double air = (v - double3.Cross(new double3(0, 0, home.GetAngularVelocity()), r)).Length();
            upfgConverged |= state.Upfg.Converged;

            if (state.Phase == GuidanceWindow.AscentPhase.Profile)
            {
                double planKm = profile.AltitudeAt(state.ConvexPlanTime) / 1000.0;
                if (Math.Abs(altKm - planKm) > Math.Abs(worstDevKm))
                {
                    worstDevKm = altKm - planKm;
                    worstDevSpeed = air;
                }
            }
            else if (double.IsNaN(handoverAltKm))
            {
                handoverAltKm = altKm;
                handoverTime = time;
                handoverSpeed = air;
                t.Info($"hand-over to {state.Phase} at t={time:F1} s, {altKm:F1} km, air speed {air:F0} m/s, plan time {state.ConvexPlanTime:F1} s "
                     + $"(plan there {profile.AltitudeAt(state.ConvexPlanTime) / 1000.0:F1} km, {profile.SpeedAt(state.ConvexPlanTime):F0} m/s)");
            }

            if (time >= nextSample)
            {
                nextSample += SampleIntervalS;
                double3 up = double3.Normalize(r);
                double pitch = 90.0 - Math.Acos(Math.Clamp(double3.Dot(up, double3.Normalize(state.CommandDir)), -1.0, 1.0)) * 180.0 / Math.PI;
                profile.Attitude(state.ConvexPlanTime, out double planPitch, out _);
                t.Info($"t={time,6:F1}s {state.Phase,-10} alt {altKm,6:F1} km  air {air,6:F0} m/s  plan t {state.ConvexPlanTime,6:F1} s alt {profile.AltitudeAt(state.ConvexPlanTime) / 1000.0,6:F1} km {profile.SpeedAt(state.ConvexPlanTime),6:F0} m/s  "
                     + $"cmd pitch {pitch,5:F1} (plan {planPitch * 180.0 / Math.PI,5:F1})  tgo {state.Upfg.Tgo,6:F1}s  mass {vehicle.TotalMass / 1000.0,7:F1} t  '{state.Status}'");
            }
        }

        t.Check($"the profile hands over above {HandoverAltKm:F0} km",
            double.IsFinite(handoverAltKm) && handoverAltKm >= HandoverAltKm - 0.5,
            $"at {handoverAltKm:F1} km, t={handoverTime:F1} s, {handoverSpeed:F0} m/s");
        t.CheckAbs("the profile climbs as planned, km off the plan's altitude where the flight is on the plan",
            worstDevKm, 0.0, ProfileAltitudeTolKm);
        t.Info($"worst altitude deviation {worstDevKm:F2} km at {worstDevSpeed:F0} m/s air speed");
        t.Check("UPFG converged", upfgConverged);
        if (!t.Check("the ascent cuts off and releases guidance",
                !state.Running && state.Status.StartsWith("Ascent complete", StringComparison.Ordinal),
                $"running={state.Running} after {time:F0} s, status '{state.Status}', error '{state.GuidanceError}'"))
            return;
        driver.Step(StepDt, 8);

        Orbit flown = vehicle.Orbit;
        double radius = home.MeanRadius;
        t.Info($"flown {(flown.Periapsis - radius) / 1000.0:F1} x {(flown.Apoapsis - radius) / 1000.0:F1} km, inc {flown.Inclination * 180.0 / Math.PI:F3} deg; "
             + $"{vehicle.TotalMass / 1000.0:F2} t in orbit against {sol.FinalMass / 1000.0:F2} t planned");
        t.CheckAbs("flown periapsis altitude, km", (flown.Periapsis - radius) / 1000.0, TargetAltKm, PeriapsisTolKm);
        t.CheckAbs("flown apoapsis altitude, km", (flown.Apoapsis - radius) / 1000.0, TargetAltKm, ApoapsisTolKm);
        t.CheckAbs("flown inclination, deg", flown.Inclination * 180.0 / Math.PI, TargetIncDeg, InclinationTolDeg);
    }

    private static void DespawnJettisoned(CelestialSystem system, HashSet<string> preexisting, Vehicle flying)
    {
        List<Vehicle>? spent = null;
        for (int i = 0; i < system.Count; i++)
            if (system.GetIndex(i) is Vehicle v && !ReferenceEquals(v, flying) && !preexisting.Contains(v.Id))
                (spent ??= new()).Add(v);
        if (spent != null)
            foreach (Vehicle v in spent)
                VehicleSpawner.Despawn(v);
    }
}

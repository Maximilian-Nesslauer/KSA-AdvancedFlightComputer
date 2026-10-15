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
// vehicle's altitude following the plan's; the command turns onto UPFG's steering smoothly over the hand-over blend
// rather than slewing onto it; and the orbit the game reports after cutoff is the one asked for. The planned mass
// to orbit is reported against the flown one.
//
// Twice: at the script's 99 % throttle floor, where the plan is a full-throttle one, and at a 40 % floor, where the
// plan throttles the liquid stages and the profile has to fly that throttle as well as the attitude. And once on a
// stack that burns solid motors, whose thrust the plan now follows along each motor's own curve (#73).
public abstract class ConvexAscentFlightTest : AfcTest
{
    /// <summary>The plan's throttle floor, percent.</summary>
    protected abstract double ThrottleFloorPct { get; }

    /// <summary>The saves to fly, first found first.</summary>
    protected virtual string[] Saves => DefaultSaves;

    /// <summary>Skip a save that burns no solids.</summary>
    protected virtual bool RequiresSolids => false;

    /// <summary>Hold the climb to the plan's altitude all the way to the hand-over. Off where a known drift is tracked elsewhere; the deviation is still logged.</summary>
    protected virtual bool ChecksClimbTracking => true;

    /// <summary>How far each staging may land off the plan's altitude, km, or NaN for no check.</summary>
    protected virtual double StagingAltitudeTolKm => double.NaN;

    /// <summary>Log every UPFG solve from UpfgSolveLogLeadS before the hand-over to the end of the blend watch: its stage model, steering and the command.</summary>
    protected virtual bool LogsUpfgSolvesAroundHandover => false;

    private const double UpfgSolveLogLeadS = 10.0;

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

    // How far the flown climb may sit off the plan's altitude, at the flight's place on the plan, before the hand-over, km. On 2stage_new it stays within 4.5 km. Kept at twice that: the plan's drag is the whole stack's all the way up. Solids fly their own thrust curves in the plan since #73; the solids test reports how far each staging lands off the plan's.
    private const double ProfileAltitudeTolKm = 10.0;
    // The orbit is UPFG's, not the plan's: the same bounds as its own ascent test (see GuidanceAscentArgPeTest) and wide enough for its cutoff at four g on this step. On 2stage_new it inserted 10 km low at periapsis and 23 km high at apoapsis.
    private const double PeriapsisTolKm = 25.0;
    private const double ApoapsisTolKm = 30.0;
    private const double InclinationTolDeg = 0.25;

    // How fast the command may turn through the hand-over blend, deg/s. The slew limit it replaces is 5 deg/s, which is what a snap onto UPFG's steering reads as; blended, a ten-degree difference turns at a peak of 1 deg/s on top of UPFG's own rate.
    // A step at which UPFG's own steering turns faster than the limit is not counted: the command would turn as fast with no blend at all. The solve after a separation works on a new stage list and on the coast the staging took, and its steering can step by a degree within one guidance cycle; the slew limit holds that, and it is UPFG's rate, not the blend's.
    private const double BlendRateLimitDegS = 2.5;
    private const double BlendWatchS = 20.0;

    // A throttled plan is one that goes below this anywhere.
    private const double ThrottledBelow = 0.9;
    // GuidanceWindow.ConvexThrottle commands full throttle for a planned fraction at or above this.
    private const double FullThrottlePlan = 0.995;
    // How far the lowest flown command may sit above the lowest planned fraction. The command is a throttle setting and the plan a thrust fraction, which the engine's throttle curve maps a little apart.
    private const double ThrottleFollowTol = 0.03;

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(
            AccessTools.Field(typeof(GuidanceWindow), "_s"));

    private static readonly MethodInfo StartGuidance =
        AccessTools.Method(typeof(GuidanceWindow), "StartGuidance");

    private static readonly MethodInfo LanOverhead =
        AccessTools.Method(typeof(GuidanceWindow), "LanOverhead");

    private static readonly MethodInfo ConvexThrottle =
        AccessTools.Method(typeof(GuidanceWindow), "ConvexThrottle");

    // The flown vessel, and the stack that disposed it, so a flight that stops because the vessel is gone names who removed it.
    private static Vehicle? _flown;
    private static string? _flownDisposedBy;

    private static void RecordFlownDispose(Vehicle __instance)
    {
        if (ReferenceEquals(__instance, _flown) && _flownDisposedBy == null)
            _flownDisposedBy = Environment.StackTrace;
    }

    private static void RecordFlownDestruction(Vehicle vehicle, VehicleDestructionEvent destructionEvent)
    {
        if (ReferenceEquals(vehicle, _flown))
            _flownDestroyedFor = $"{destructionEvent.Cause}, peak {destructionEvent.PeakGLoad:F2} g, "
                               + $"peak q {destructionEvent.PeakDynamicPressure:F0} Pa, structural limit {vehicle.FlightComputer.MaxGLoad:F2} g";
    }

    private static string? _flownDestroyedFor;

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
                 + $"(terrain {terrain:F0} m), {vehicle.TotalMass / 1000.0:F1} t, id '{vehicle.Id}', hash {vehicle.Hash.Code}");

            Program.ControlledVehicle = vehicle;
            PhysicsBubble._forceOffRails = true;
            StagingConfig.EngineDelays.Clear();
            StagingConfig.DecouplerDelays.Clear();
            StagingConfig.DropSpentStages = true;
            driver.Step(0.05, 6);

            _flown = vehicle;
            _flownDisposedBy = null;
            _flownDestroyedFor = null;
            harmony.Patch(AccessTools.Method(typeof(Vehicle), nameof(Vehicle.Dispose), new[] { typeof(bool) }),
                prefix: new HarmonyMethod(typeof(ConvexAscentFlightTest), nameof(RecordFlownDispose)));
            harmony.Patch(AccessTools.Method(typeof(Universe), nameof(Universe.DestroyVehicleFromEvent)),
                prefix: new HarmonyMethod(typeof(ConvexAscentFlightTest), nameof(RecordFlownDestruction)));
            Fly(t, vehicle, home, driver, preexisting);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            _flown = null;
            GuidanceFeature.DisableDriver();
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.SetModActive(previousModActive);
            StagingConfig.DropSpentStages = previousDropSpent;
            AmbientState() = previousAmbient;
            AutoStageFlightSupport.CleanupAfterFlight(t.System, preexisting);
            Program.ControlledVehicle = previousFocus;
        }
    }

    private void Fly(TestContext t, Vehicle vehicle, IParentBody home, SimDriver driver, HashSet<string> preexisting)
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
        // The g-limit holds the plan's throttle and UPFG's closed loop alike: a stage that ends at ten g against a quarter-second step would otherwise cut off tens of m/s off.
        state.GLimitEnabled = true;
        state.GLimitG = GLimitG;
        state.ReserveArmed = false;
        state.TargetId = "";
        state.FlyConvexAscent = true;
        state.ConvexHandoverAltKm = HandoverAltKm;
        state.ConvexThrottleMinPct = ThrottleFloorPct;
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
        for (int i = 0; i < plan.StageLines.Count; i++)
            t.Info($"stage {i + 1}: {plan.StageLines[i]}");
        foreach (string note in plan.StageNotes)
            t.Info("stages: " + note);
        if (RequiresSolids && plan.SolidStages == 0)
        {
            t.Skip("the save burns no solids in the stages planned.");
            return;
        }
        // The panel's 140 kPa is four times the script's Saturn V 35 kPa; a stack that still cannot climb under it at full throttle is retried at a limit it can hold.
        t.Info($"max q planned to {plan.QMaxKpa:F0} kPa (asked {plan.QMaxRequestedKpa:F0}{(plan.QRelaxed ? ", out of reach" : "")}), "
             + $"throttle floor {plan.ThrottleMinPct:F0} %, lowest planned throttle {100.0 * plan.Profile.MinThrottle:F0} %");
        bool throttled = ThrottleFloorPct < 99.0;
        if (throttled)
            t.Check($"the plan throttles below {ThrottledBelow:P0} somewhere", plan.Profile.MinThrottle < ThrottledBelow,
                $"lowest {plan.Profile.MinThrottle:P0}");
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
        if (!t.Check("EXECUTE flies the convex profile", state.Running && state.Phase == GuidanceWindow.AscentPhase.Profile,
                $"running {state.Running}, phase {state.Phase}, status '{state.Status}'"))
            return;

        double time = 1.0, nextSample = 0.0;
        double handoverAltKm = double.NaN, handoverTime = double.NaN, handoverSpeed = double.NaN;
        double worstDevKm = 0.0, worstDevSpeed = double.NaN;
        bool upfgConverged = false, blended = false;
        double lowestCommand = 1.0, lowestPlanned = 1.0, blendRate = 0.0, blendRateTime = double.NaN;
        double terminalTime = double.NaN, skippedUpfgRate = 0.0;
        int skippedUpfgSteps = 0;
        double3 lastSteer = default;
        double3 lastCommand = state.CommandDir;
        int lastStage = state.ConvexStage;
        int lastActivations = StagingDetector.ActivationsOf(vehicle);
        double worstStagingAltKm = 0.0;
        double3 lastSampleR = double3.Zero;
        bool frozenReported = false, cutoffLogged = false;
        double3 lastR = vehicle.Orbit.StateVectors.PositionCci, lastV = vehicle.Orbit.StateVectors.VelocityCci;
        double nextThrustLog = 0.0;
        // The solves before the hand-over are only known to be before it once it happens, so they wait here and are logged then.
        var solvesBeforeHandover = new Queue<(double Time, string Line)>();
        double lastSolveTime = state.LastSolveTime;
        while (time < MaxFlightSeconds && state.Running)
        {
            // Fine steps through the terminal count: the cutoff lands on a step, and at four g a quarter second is 10 m/s, which is 35 km of apoapsis.
            double dt = state.Phase == GuidanceWindow.AscentPhase.Terminal ? TerminalStepDt : StepDt;
            driver.Step(dt);
            time += dt;
            // A spent stage falls back to the ground minutes later, and the game's ground-impact effect throws in the headless runtime. The flight under test does not need them, so they go as soon as they separate.
            int activations = StagingDetector.ActivationsOf(vehicle);
            if (activations != lastActivations)
            {
                t.Info($"AutoStage staged ({activations - lastActivations}) at t={time:F2} s, {vehicle.TotalMass / 1000.0:F2} t, "
                     + $"phase {StagingDetector.StateOf(vehicle).State}, guidance {state.Phase}, "
                     + $"active sequence {vehicle.Parts.SequenceList.ActiveSequence} of {vehicle.Parts.SequenceList.Sequences.Length}, "
                     + $"{vehicle.Parts.Count} parts, delays {StagingConfig.EngineDelays.Count}/{StagingConfig.DecouplerDelays.Count}");
                lastActivations = activations;
            }
            DespawnJettisoned(t.System, preexisting, vehicle);
            double3 r = vehicle.Orbit.StateVectors.PositionCci;
            double3 v = vehicle.Orbit.StateVectors.VelocityCci;
            double altKm = (r.Length() - home.MeanRadius) / 1000.0;
            double air = (v - double3.Cross(new double3(0, 0, home.GetAngularVelocity()), r)).Length();
            upfgConverged |= state.Upfg.Converged;

            // How fast the command turns through the hand-over blend.
            // UPFG's steering law where the command reads it this step (see Ascent.UpdatePhase), and how far it turned since the last one.
            double3 steerNow = state.Upfg.SteeringAt(Math.Clamp(Universe.GetElapsedSeconds() - state.LastSolveTime, 0.0, 1.0), out _);
            double steerRate = lastSteer.Length() > 0.5 && steerNow.Length() > 0.5 ? AngleDeg(lastSteer, steerNow) / dt : 0.0;
            lastSteer = steerNow;
            if (double.IsFinite(handoverTime) && time - handoverTime <= BlendWatchS && steerRate > BlendRateLimitDegS)
            {
                skippedUpfgSteps++;
                skippedUpfgRate = Math.Max(skippedUpfgRate, steerRate);
            }
            else if (double.IsFinite(handoverTime) && time - handoverTime <= BlendWatchS
                && lastCommand.Length() > 0.5 && state.CommandDir.Length() > 0.5)
            {
                double turn = Math.Acos(Math.Clamp(double3.Dot(double3.Normalize(lastCommand), double3.Normalize(state.CommandDir)), -1.0, 1.0));
                double rate = turn * 180.0 / Math.PI / dt;
                if (rate > blendRate)
                {
                    blendRate = rate;
                    blendRateTime = time;
                }
                // Each fast step of the blend, split into pitch and heading, so a peak can be told apart from the blend's own turn.
                if (LogsUpfgSolvesAroundHandover && rate > 1.5)
                {
                    double3 upNow = double3.Normalize(r);
                    t.Info($"blend step t={time:F2}s {rate:F2} deg/s: pitch {PitchDeg(upNow, lastCommand):F2} -> {PitchDeg(upNow, state.CommandDir):F2}, "
                         + $"heading {HeadingDeg(upNow, lastCommand):F2} -> {HeadingDeg(upNow, state.CommandDir):F2}, steer pitch {PitchDeg(upNow, state.Upfg.Steering):F2}, "
                         + $"plan stage {state.ConvexStage + 1} t {state.ConvexPlanTime:F2} s, mass {vehicle.TotalMass / 1000.0:F2} t, status '{state.Status}'");
                }
            }
            lastCommand = state.CommandDir;

            // The thrust the vehicle actually delivers over the last 20 s of the burn, from its change of velocity less point-mass gravity, against the command: an attitude or thrust-axis error the terminal phase cannot steer out shows here.
            if (LogsUpfgSolvesAroundHandover && state.Running && !state.CutoffDone && time >= nextThrustLog
                && (state.Phase == GuidanceWindow.AscentPhase.Terminal || (state.Phase == GuidanceWindow.AscentPhase.ClosedLoop && state.Upfg.Tgo < 20.0)))
            {
                nextThrustLog = time + 1.0;
                double3 mid = 0.5 * (r + lastR);
                double3 accel = (v - lastV) * (1.0 / dt) + mid * (home.Mu / Math.Pow(mid.Length(), 3));
                double3 upMid = double3.Normalize(mid);
                t.Info($"thrust t={time:F2}s {state.Phase}: delivered pitch {PitchDeg(upMid, accel):F2} heading {HeadingDeg(upMid, accel):F2} at {accel.Length() / 9.80665:F2} g, "
                     + $"command pitch {PitchDeg(upMid, state.CommandDir):F2} heading {HeadingDeg(upMid, state.CommandDir):F2}, "
                     + $"steer pitch {PitchDeg(upMid, state.Upfg.Steering):F2}, off command {AngleDeg(accel, state.CommandDir):F2} deg, tgo {state.Upfg.Tgo:F2} s");
            }
            lastR = r;
            lastV = v;

            // The cutoff: when the terminal phase latched the steering law and what the burn left, against the circular speed at the radius reached.
            if (LogsUpfgSolvesAroundHandover && state.Phase == GuidanceWindow.AscentPhase.Terminal && double.IsNaN(terminalTime))
            {
                terminalTime = time;
                t.Info($"terminal at t={time:F2} s: tgo {state.Upfg.Tgo:F2} s, "
                     + $"pitch at the latch {PitchDeg(double3.Normalize(r), state.FrozenDir):F2}, alt {altKm:F2} km, mass {vehicle.TotalMass / 1000.0:F2} t");
            }
            if (LogsUpfgSolvesAroundHandover && double.IsFinite(terminalTime) && (state.CutoffDone || !state.Running) && !cutoffLogged)
            {
                cutoffLogged = true;
                double3 upNow = double3.Normalize(r);
                double radial = double3.Dot(v, upNow);
                double horizontal = (v - radial * upNow).Length();
                double circular = Math.Sqrt(home.Mu / r.Length());
                t.Info($"cut off by t={time:F2} s: alt {altKm:F3} km, radial {radial:F2} m/s, horizontal {horizontal:F2} m/s against circular {circular:F2} m/s, "
                     + $"thrust accel {KsaEnginePerf.ActiveThrustCapability(vehicle, 0.0) / vehicle.TotalMass / 9.80665:F2} g at full, mass {vehicle.TotalMass / 1000.0:F2} t");
            }

            // Each staging against the plan's own: where the profile moved on to the next planned stage, against where the plan has it.
            if (state.Phase == GuidanceWindow.AscentPhase.Profile && state.ConvexStage != lastStage)
            {
                lastStage = state.ConvexStage;
                int first = Array.IndexOf(sol.NodeStage, lastStage);
                if (first > 0)
                {
                    var pr = new double3(sol.Position[first * 3], sol.Position[first * 3 + 1], sol.Position[first * 3 + 2]);
                    var pv = new double3(sol.Velocity[first * 3], sol.Velocity[first * 3 + 1], sol.Velocity[first * 3 + 2]);
                    double planAlt = (pr.Length() - home.MeanRadius) / 1000.0;
                    double planAir = (pv - double3.Cross(new double3(0, 0, home.GetAngularVelocity()), pr)).Length();
                    if (Math.Abs(altKm - planAlt) > Math.Abs(worstStagingAltKm))
                        worstStagingAltKm = altKm - planAlt;
                    t.Info($"into plan stage {lastStage + 1} at t={time:F1} s, {altKm:F1} km, air {air:F0} m/s, {vehicle.TotalMass / 1000.0:F1} t; "
                         + $"the plan at t={sol.Time[first]:F1} s, {planAlt:F1} km, {planAir:F0} m/s, {sol.Mass[first] / 1000.0:F1} t");
                }
            }

            if (state.Phase == GuidanceWindow.AscentPhase.Profile)
            {
                lowestCommand = Math.Min(lowestCommand, (float)ConvexThrottle.Invoke(null, new object[] { vehicle, home })!);
                lowestPlanned = Math.Min(lowestPlanned, profile.Throttle(state.ConvexPlanTime));
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
                blended = double.IsFinite(state.ConvexBlendStart);
                foreach ((double _, string line) in solvesBeforeHandover)
                    t.Info(line);
                solvesBeforeHandover.Clear();
                t.Info($"hand-over to {state.Phase} at t={time:F1} s, {altKm:F1} km, air speed {air:F0} m/s, plan time {state.ConvexPlanTime:F1} s "
                     + $"(plan there {profile.AltitudeAt(state.ConvexPlanTime) / 1000.0:F1} km, {profile.SpeedAt(state.ConvexPlanTime):F0} m/s)");
            }

            // UPFG's own view of the vehicle at each solve through the hand-over: the stage model it plans on (thrust and burn time per stage), the steering it gives and the command flown, against the live thrust.
            if (LogsUpfgSolvesAroundHandover && state.LastSolveTime != lastSolveTime)
            {
                lastSolveTime = state.LastSolveTime;
                bool afterHandover = double.IsFinite(handoverTime);
                if (!afterHandover || time - handoverTime <= BlendWatchS)
                {
                    double3 up = double3.Normalize(r);
                    profile.Attitude(state.ConvexPlanTime, out double planPitchRad, out _);
                    double liveThrustKn = KsaEnginePerf.ActiveThrustCapability(vehicle, KsaEnginePerf.AmbientPressureAt(home, altKm * 1000.0)) / 1000.0;
                    string line = $"UPFG solve t={time:F2}s {state.Phase} tgo {state.Upfg.Tgo:F1} s vgo {state.Upfg.VgoMag:F0} m/s converged {state.Upfg.Converged}  "
                                + $"steer pitch {PitchDeg(up, state.Upfg.Steering):F1} cmd pitch {PitchDeg(up, state.CommandDir):F1} plan pitch {planPitchRad * 180.0 / Math.PI:F1}  "
                                + $"alt {altKm:F1} km mass {vehicle.TotalMass / 1000.0:F2} t live thrust {liveThrustKn:F0} kN  model {GuidanceLog.DescribeStages(state.UpfgVehicle)}";
                    if (afterHandover)
                        t.Info(line);
                    else
                    {
                        solvesBeforeHandover.Enqueue((time, line));
                        while (time - solvesBeforeHandover.Peek().Time > UpfgSolveLogLeadS)
                            solvesBeforeHandover.Dequeue();
                    }
                }
            }

            if (time >= nextSample)
            {
                nextSample += SampleIntervalS;
                // A vessel that no longer moves between two samples is not simulated any more, which the asserts below only show as a missed cutoff.
                if (!frozenReported && r == lastSampleR)
                {
                    frozenReported = true;
                    bool inFrame = false;
                    foreach (Vehicle framed in Program.VehiclesInFrame)
                        inFrame |= ReferenceEquals(framed, vehicle);
                    t.Info($"the vessel has not moved since the last sample at t={time:F1} s: phase {state.Phase}, "
                         + $"autostage {StagingDetector.StateOf(vehicle).State}, burn mode {vehicle.FlightComputer.BurnMode}, "
                         + $"situation {vehicle.Situation}, controlled {ReferenceEquals(Program.ControlledVehicle, vehicle)}, "
                         + $"disposed {vehicle.IsDisposed}, in frame {inFrame}, mass {vehicle.TotalMass / 1000.0:F1} t");
                    if (_flownDestroyedFor != null)
                        t.Info($"the game destroyed the vessel: {_flownDestroyedFor}");
                    if (_flownDisposedBy != null)
                        t.Info($"the vessel was disposed by:{Environment.NewLine}{_flownDisposedBy}");
                }
                lastSampleR = r;
                double3 up = double3.Normalize(r);
                double pitch = 90.0 - Math.Acos(Math.Clamp(double3.Dot(up, double3.Normalize(state.CommandDir)), -1.0, 1.0)) * 180.0 / Math.PI;
                profile.Attitude(state.ConvexPlanTime, out double planPitch, out _);
                // Full thrust against the plan's at the same point, so a model that over- or under-states the engines shows up here rather than only as drift.
                double pressure = KsaEnginePerf.AmbientPressureAt(home, altKm * 1000.0);
                double thrustKn = KsaEnginePerf.ActiveThrustCapability(vehicle, pressure) / 1000.0;
                double planThrustKn = PlanThrustAt(sol, profile, state.ConvexPlanTime) / 1000.0;
                t.Info($"t={time,6:F1}s {state.Phase,-10} alt {altKm,6:F1} km  air {air,6:F0} m/s  plan stage {state.ConvexStage + 1} t {state.ConvexPlanTime,6:F1} s alt {profile.AltitudeAt(state.ConvexPlanTime) / 1000.0,6:F1} km {profile.SpeedAt(state.ConvexPlanTime),6:F0} m/s  "
                     + $"cmd pitch {pitch,5:F1} (plan {planPitch * 180.0 / Math.PI,5:F1})  plan throttle {100.0 * profile.Throttle(state.ConvexPlanTime),3:F0} %  "
                     + $"thrust {thrustKn,6:F0} kN (plan {planThrustKn,6:F0})  tgo {state.Upfg.Tgo,6:F1}s  mass {vehicle.TotalMass / 1000.0,7:F1} t  '{state.Status}'");
            }
        }

        t.Check($"the profile hands over above {HandoverAltKm:F0} km",
            double.IsFinite(handoverAltKm) && handoverAltKm >= HandoverAltKm - 0.5,
            $"at {handoverAltKm:F1} km, t={handoverTime:F1} s, {handoverSpeed:F0} m/s");
        if (ChecksClimbTracking)
            t.CheckAbs("the profile climbs as planned, km off the plan's altitude where the flight is on the plan",
                worstDevKm, 0.0, ProfileAltitudeTolKm);
        else
            t.Info($"climb tracking not checked here (#81): {worstDevKm:F2} km off the plan's altitude at worst, against {ProfileAltitudeTolKm:F0} km elsewhere");
        if (!double.IsNaN(StagingAltitudeTolKm))
            t.CheckAbs("each staging lands where the plan stages, km off the plan's altitude", worstStagingAltKm, 0.0, StagingAltitudeTolKm);
        t.Info($"worst altitude deviation {worstDevKm:F2} km at {worstDevSpeed:F0} m/s air speed; worst at a staging {worstStagingAltKm:F2} km");
        t.Check("the hand-over blends onto UPFG's steering", blended);
        t.Check($"the command turns under {BlendRateLimitDegS:F1} deg/s through the blend", blendRate <= BlendRateLimitDegS,
            $"peak {blendRate:F2} deg/s at t={blendRateTime:F2} s in the {BlendWatchS:F0} s after the hand-over"
            + (skippedUpfgSteps > 0 ? $", not counting {skippedUpfgSteps} step(s) where UPFG's own steering turned at up to {skippedUpfgRate:F2} deg/s" : ""));
        if (throttled && lowestPlanned < FullThrottlePlan)
            t.Check($"the profile throttles the engines as the plan does, within {ThrottleFollowTol:P0}",
                lowestCommand < FullThrottlePlan && lowestCommand <= lowestPlanned + ThrottleFollowTol,
                $"lowest command {lowestCommand:P1}, lowest planned {lowestPlanned:P1} before the hand-over");
        else if (throttled)
            t.Info($"the plan keeps full throttle until the hand-over (lowest planned {lowestPlanned:P1}), so the profile throttle is not checked");
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

    // Degrees between two directions, NaN for a zero one.
    private static double AngleDeg(double3 a, double3 b)
        => a.Length() < 1e-9 || b.Length() < 1e-9 ? double.NaN
         : Math.Acos(Math.Clamp(double3.Dot(double3.Normalize(a), double3.Normalize(b)), -1.0, 1.0)) * 180.0 / Math.PI;

    // Degrees from north toward east of the horizontal part, NaN for a vertical or zero direction.
    private static double HeadingDeg(double3 up, double3 dir)
    {
        double3 h = dir - double3.Dot(dir, up) * up;
        double3 east = double3.Cross(new double3(0, 0, 1), up);
        if (h.Length() < 1e-9 || east.Length() < 1e-9)
            return double.NaN;
        east = double3.Normalize(east);
        double3 north = double3.Cross(up, east);
        return Math.Atan2(double3.Dot(h, east), double3.Dot(h, north)) * 180.0 / Math.PI;
    }

    // Degrees above the local horizontal, NaN for a zero direction.
    private static double PitchDeg(double3 up, double3 dir)
        => dir.Length() < 1e-9 ? double.NaN
         : 90.0 - Math.Acos(Math.Clamp(double3.Dot(up, double3.Normalize(dir)), -1.0, 1.0)) * 180.0 / Math.PI;

    // The plan's full-throttle thrust at a plan time, liquids and solids, from the node nearest in time.
    private static double PlanThrustAt(AscentSolution sol, ConvexAscentProfile profile, double planTime)
    {
        int best = 0;
        for (int k = 1; k < sol.Nodes; k++)
            if (Math.Abs(sol.Time[k] - planTime) <= Math.Abs(sol.Time[best] - planTime)) best = k;
        return sol.FullThrust.Length > best ? sol.FullThrust[best] : double.NaN;
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

// The script's throttle floor: a full-throttle plan.
public sealed class GuidanceConvexAscentTest : ConvexAscentFlightTest
{
    public override string Name => "afc-guidance-convex-ascent";
    protected override double ThrottleFloorPct => 99.0;
}

// A 40 % floor: the plan throttles the liquid stages through the thick air, and the profile flies that throttle.
public sealed class GuidanceConvexAscentThrottledTest : ConvexAscentFlightTest
{
    public override string Name => "afc-guidance-convex-ascent-throttled";
    protected override double ThrottleFloorPct => 40.0;
}

// A stack with solid motors: Test Vehicle 2's progressive boosters. The plan follows each motor's burn as the game's own model steps it, so the climb stays with the plan through the boosters' burn rather than drifting off it the way a mean thrust did (#73). At a 40 % floor, so a core lit with the boosters is the planner's to throttle.
public sealed class GuidanceConvexAscentSolidsTest : ConvexAscentFlightTest
{
    public override string Name => "afc-guidance-convex-ascent-solids";
    protected override double ThrottleFloorPct => 40.0;
    protected override string[] Saves => new[] { "Test Vehicle 2" };
    protected override bool RequiresSolids => true;

    // Through a solid stage's tail-off the speed-indexed profile falls further behind the boosters' clock-driven thrust (#81), so the climb check waits on that. What the solid model itself decides is where the boosters separate: on 2stage they burn out 1.3 s after the plan's burnout, with the plan's mass to 0.2 t, 3.5 km below the plan's altitude.
    protected override bool ChecksClimbTracking => false;
    protected override double StagingAltitudeTolKm => 5.0;

    // The hand-over falls inside the solid stage, so the log shows whether UPFG's stage model of the boosters matches their real burn.
    protected override bool LogsUpfgSolvesAroundHandover => true;
}

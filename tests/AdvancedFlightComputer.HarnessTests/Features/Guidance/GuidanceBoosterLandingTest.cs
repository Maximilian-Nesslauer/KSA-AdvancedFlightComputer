using System.Collections.Concurrent;
using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Opt-in measurement: 2stage_controlled's booster flown from the glide to touchdown by entry guidance (3-DOF) and touchdown guidance (6-DOF), the way the 2026-10-08 flights flew it, with the 6-DOF log on.
//
// THE START is where entry guidance took the glide over in those flights: about 5 km over the site, 900 m short of it, sinking at ~130 m/s, upright and engine first. The stack is spawned there, its booster separated, the upper stage removed, and the booster's tanks set to a landing load - the save's booster is full, and a boostback is not what is under test.
//
// PACED TO REAL TIME. Both guidances solve on worker threads against the wall clock while the step reads the sim clock; stepped as fast as the machine can go, a 50 ms solve would be seconds of flight, and the plans would be stale in a way no flight sees.
//
// A line every half second of flight - height of the legs over the site, range, speeds, throttle, what flies the booster - and the 6-DOF's own CSVs in navbox-logs, named in the last lines.
public sealed class GuidanceBoosterLandingTest : AfcTest
{
    public override string Name => "afc-guidance-booster-landing";
    public override bool OptIn => true;

    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;
    private const string Save = "2stage_controlled";
    private const double StepS = 0.02;
    private const double SampleS = 0.5;
    private const double TimeoutS = 150.0;

    // Overridable from the environment, so a case can be re-flown without a rebuild.
    private static double Env(string name, double fallback)
        => double.TryParse(Environment.GetEnvironmentVariable(name), NumberStyles.Float, CultureInfo.InvariantCulture, out double v) ? v : fallback;

    // The harness does not capture the game log, so guidance's INFO lines are queued here - from whichever thread logs them - and printed between steps.
    private static readonly ConcurrentQueue<string> GuidanceLines = new();

    private static void QueueGuidanceLine(Vehicle vehicle, string message) => GuidanceLines.Enqueue(message);

    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> Inputs =
        AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        if (VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state.");
            return;
        }

        double heightM = Env("AFC_LANDING_HEIGHT_M", 5000.0);
        double rangeM = Env("AFC_LANDING_RANGE_M", 900.0);
        double sinkMs = Env("AFC_LANDING_SINK_MS", 130.0);
        double groundSpeedMs = Env("AFC_LANDING_GROUND_SPEED_MS", 20.0);
        double propellantKg = Env("AFC_LANDING_PROPELLANT_KG", 5000.0);
        double pace = Math.Max(Env("AFC_LANDING_PACE", 1.0), 0.05);

        FieldInfo ambient = typeof(GuidanceWindow).GetField("_s", PrivateStatic)!;
        object? previousAmbient = ambient.GetValue(null);
        Vehicle? previousFocus = Program.ControlledVehicle;
        bool previousModActive = GuidanceWindow.ModActive;
        bool previousEnabled = SharedVehicleHooks.GuidanceEnabled;
        // The production hooks, on a test-scoped id: the PrepareWorker prefix steps guidance, the command sink applies its gimbal and engine commands.
        // The player's aerodynamics (KSAero), when it is deployed: without it the booster flies on stock aero, which is a different vehicle. AFC_LANDING_KSAERO=0 flies stock.
        using KsaeroScope aero = Env("AFC_LANDING_KSAERO", 1.0) > 0.0 ? KsaeroScope.Apply() : null!;
        t.Info(aero?.Status ?? "stock aerodynamics (AFC_LANDING_KSAERO=0)");
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.boosterlanding");
        VehicleCommandSink.ApplyPatches(harmony);
        GuidanceFeature.ApplyDriverPatches(harmony);
        harmony.Patch(AccessTools.Method(typeof(GuidanceLog), nameof(GuidanceLog.Info)),
            prefix: new HarmonyMethod(typeof(GuidanceBoosterLandingTest), nameof(QueueGuidanceLine)));
        GuidanceLines.Clear();
        SharedVehicleHooks.GuidanceEnabled = true;
        GuidanceWindow.SetModActive(true);
        // The bodies turn: the harness freezes them by default, which leaves the site still while a booster launched with the surface's velocity runs away from it at ~460 m/s.
        SimDriver driver = t.Session.CreateDriver();
        driver.StepOrbits = true;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool railsBefore = PhysicsBubble._forceOffRails;
        Vehicle? booster = null;
        try
        {
            // A step first: until the orbit solvers have run once, the body's rotation is not the universe clock's, and the site read here would be an hour of spin away from the one guidance reads in flight.
            driver.Step(StepS, 2);
            // The site, as guidance computes it, from a state with the default site.
            ambient.SetValue(null, new VehicleAutopilotState());
            double3 up = (double3)Method("SiteDirCciAt").Invoke(null, [home, 0.0])!;
            double terrain = (double)Method("SiteTerrainHeight").Invoke(null, [home])!;
            double3 siteCci = up * (home.MeanRadius + terrain);
            double3 omega = home.GetAngularVelocityCci();
            double3 east = double3.Cross(omega, up);
            east = east.Length() > 1e-12 ? double3.Normalize(east) : double3.Normalize(double3.Cross(double3.UnitZ, up));

            // Uprange to the east, closing west on the site, sinking.
            double3 r0 = siteCci + up * heightM + east * rangeM;
            double3 vSurface = -up * sinkMs - east * groundSpeedMs;
            double3 v0 = vSurface + double3.Cross(omega, r0);
            Orbit orbit = Orbit.CreateFromStateCci(home, driver.Elapsed, r0, v0, VehicleSpawner.OrbitLineColor);

            // Upright: the thrust axis, body +X, along the local vertical.
            double3 x = double3.UnitX;
            double3 axis = double3.Cross(x, up);
            double angle = Math.Acos(Math.Clamp(double3.Dot(x, up), -1.0, 1.0));
            doubleQuat body2Cci = axis.Length() > 1e-12 ? doubleQuat.CreateFromAxisAngle(double3.Normalize(axis), angle) : doubleQuat.Identity;
            doubleQuat body2Cce = doubleQuat.Concatenate(body2Cci, home.GetCci2Cce());

            Vehicle stack;
            try
            {
                stack = Spawn(Save, t.System, home, "HarnessBoosterLanding", orbit, body2Cce);
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"{Save}: {e.Message}");
                return;
            }
            PhysicsBubble._forceOffRails = true;
            driver.Step(StepS, 5);

            booster = SeparateBooster(t, stack, driver, preexisting);
            if (booster == null)
                return;
            SetPropellant(booster, propellantKg, out double loadedKg, out double capacityKg);
            // The mass properties are the vehicle's, recomputed from the tanks only when told.
            booster.Parts.RecomputeTotalFillFraction();
            booster.UpdateAfterPartTreeModification();
            Program.ControlledVehicle = booster;
            driver.Step(StepS, 2);
            t.Info($"booster '{booster.Id}': {booster.TotalMass:F0} kg with {loadedKg:F0} of {capacityKg:F0} kg propellant, " +
                   $"starting {heightM:F0} m over the site, {rangeM:F0} m short, sinking {sinkMs:F0} m/s, closing at {groundSpeedMs:F0} m/s");

            VehicleAutopilotState state = VehicleAutopilotState.For(booster);
            state.SixDofLogging = true;
            state.SixDofTrack = Env("AFC_LANDING_SIXDOF_TRACK", 1.0) > 0.0;
            state.LandingPlanFloorMargin = Env("AFC_LANDING_FLOOR_MARGIN", state.LandingPlanFloorMargin);
            state.AutoStage = true;
            // AFC_LANDING_MODE=6dof engages touchdown guidance directly, as its EXECUTE does, for a start low enough to be a hand-over; anything else engages entry guidance.
            if (string.Equals(Environment.GetEnvironmentVariable("AFC_LANDING_MODE"), "6dof", StringComparison.OrdinalIgnoreCase))
            {
                // Its cold solve straight through on the worker rather than spread over frames: a hand-over finds the standby's plan ready, and a spread solve from this low falls for seconds first.
                state.SixDofSpreadCold = Env("AFC_LANDING_SPREAD", 0.0) > 0.0;
                object? before = ambient.GetValue(null);
                ambient.SetValue(null, state);
                typeof(GuidanceWindow).GetMethod("Engage6Dof", PrivateStatic, [typeof(Vehicle), typeof(bool), typeof(double), typeof(bool)])!
                    .Invoke(null, [booster, false, double.NaN, false]);
                ambient.SetValue(null, before);
                t.Info("touchdown guidance engaged directly");
            }
            else
            {
                state.UseThreeDofLanding = true;
                state.ThreeDofEngagePending = true;
            }
            state.LandingAccelLimitG = Env("AFC_LANDING_ACCEL_G", state.LandingAccelLimitG);
            t.Info($"burn limited to {state.LandingAccelLimitG:F1} g, hand-over with {state.ThreeDofSixDofTimeS:F1} s of burn left");
            // What the planners believe about the engines, against which the measured flow below is read.
            (double vacThrust, double vacFlow) = KsaEnginePerf.UncappedAtPressure(booster, 0.0);
            double seaThrust = KsaEnginePerf.UncappedAtPressure(booster, 101325.0).thrust;
            double minThrottle = booster.GetMinThrottle();
            t.Info($"engines: vacuum {vacThrust / 1e3:F0} kN at {vacFlow:F1} kg/s, sea level {seaThrust / 1e3:F0} kN; min throttle {minThrottle:P0}, " +
                   $"there {KsaEnginePerf.AtThrottle(booster, minThrottle, 101325.0).thrust / 1e3:F0} kN at {KsaEnginePerf.AtThrottle(booster, minThrottle, 101325.0).massFlow:F1} kg/s; " +
                   $"structural cap {KsaEnginePerf.StructuralThrottleCap(booster, 101325.0):P0}, limit {KsaEnginePerf.StructuralAccelerationLimit(booster):F1} m/s2");
            // The gimbals' torque authority as touchdown guidance plans it: the allocator's capability per newton of the thrust it was evaluated at.
            double planningThrust = Ksa6DofSetup.PlanningThrust(booster), lateral = Ksa6DofSetup.LateralTorqueLimit(booster), roll = Ksa6DofSetup.RollTorqueLimit(booster), arm = Ksa6DofSetup.EngineArm(booster);
            t.Info($"gimbals: capability at {planningThrust / 1e3:F0} kN planning thrust - lateral {lateral / 1e3:F0} kN m, roll {roll / 1e3:F0} kN m, arm {arm:F1} m; " +
                   $"effective gimbal {Math.Atan(lateral / (arm * planningThrust)) * 180 / Math.PI:F2} deg (was {Math.Atan(lateral / (arm * seaThrust)) * 180 / Math.PI:F2} deg against sea-level full thrust)");
            Ksa6DofSetup.Inertia(booster, out double ixx, out double iyy, out double izz);
            t.Info($"inertia as touchdown guidance models it: {ixx:E3} {iyy:E3} {izz:E3} kg m2, mass {booster.TotalMass:F0} kg");
            // The entry guidance's engine table against the game it was sampled from, between its breakpoints, and its inverse.
            EngineThrustTable? table = Ksa3DofSetup.EngineTable(booster, 101325.0);
            if (table == null)
                t.Info("engine table: none (the game returned nothing usable)");
            else
            {
                var line = new System.Text.StringBuilder("engine table vs game:");
                foreach (double pa in new[] { 101325.0, 60000.0 })
                    foreach (double u in new[] { 0.1, 0.14, 0.25, 0.5, 0.77, 1.0 })
                    {
                        (double game, double gameFlow) = KsaEnginePerf.UncappedAtThrottle(booster, u, pa);
                        double model = table.Thrust(u, pa);
                        line.Append($"  {pa / 1000:F0} kPa {u:F2}: {model / 1e3:F1}/{game / 1e3:F1} kN {table.MassFlow(u):F1}/{gameFlow:F1} kg/s inv {table.ThrottleFor(game, pa):F4}");
                    }
                t.Info(line.ToString());
            }
            if (Env("AFC_LANDING_ENGINE_TABLE", 0.0) > 0.0)
                foreach (double pa in new[] { 0.0, 50000.0, 80000.0, 101325.0 })
                {
                    var row = new System.Text.StringBuilder($"  {pa / 1000,5:F0} kPa:");
                    foreach (double u in new[] { 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0 })
                    {
                        (double thrust, double massFlow) = KsaEnginePerf.AtThrottle(booster, u, pa);
                        row.Append($"  {u:F2}: {thrust / 1e3,6:F0} kN {massFlow,6:F1}");
                    }
                    t.Info(row.ToString());
                }

            Fly(t, booster, state, driver, home, ambient, pace);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            GuidanceFeature.DisableDriver();
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.SetModActive(previousModActive);
            Program.ControlledVehicle = previousFocus;
            if (booster != null)
                VehicleAutopilotState.Remove(booster);
            ambient.SetValue(null, previousAmbient);
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void Fly(TestContext t, Vehicle booster, VehicleAutopilotState state, SimDriver driver, IParentBody home,
                            FieldInfo ambient, double pace)
    {
        var wall = Stopwatch.StartNew();
        double sim = 0.0, nextSample = 0.0, sampleMass = double.NaN;
        bool engaged = false, airborne = false, touched = false;
        double touchSink = double.NaN, touchSideways = double.NaN, touchMiss = double.NaN, endAt = double.PositiveInfinity;
        string lastMode = "", runName = "";
        double nextAeroProbe = 0.0, probeT = double.NaN;
        bool couplingProbed = false;
        double3 probeV = default;
        Guidance3Dof? lastEntry = null, dumpedEntry = null;
        Plan3Dof? dumpedPlan = null;
        double nextPlanDump = 0.0;
        double nextTurnProbe = 0.0, nextTurnPrint = 0.0, turnT = double.NaN;
        double maxGlideRate = 0.0, maxGlideAccel = 0.0, maxBurnRate = 0.0, maxBurnAccel = 0.0;
        double3 turnW = default;
        int dumps = 0;
        double lastLegs = double.NaN, lastVz = double.NaN, lastVh = double.NaN, lastRange = double.NaN;

        while (sim < TimeoutS && sim < endAt)
        {
            try
            {
                driver.Step(StepS);
            }
            catch (NullReferenceException) when (lastLegs < 50.0)
            {
                // KSA's ground-impact effect has no renderer to spawn into headless and throws: the booster hit the ground hard enough to break.
                t.Info($"  t {sim,6:F2}  IMPACT at {-lastVz:F2} m/s down, {lastVh:F2} m/s sideways, {lastRange:F1} m from the site");
                touched = true;
                touchSink = -lastVz;
                touchSideways = lastVh;
                touchMiss = lastRange;
                break;
            }
            sim += StepS;
            // Real time: the solves run against the wall clock on their own threads.
            double ahead = sim / pace - wall.Elapsed.TotalSeconds;
            if (ahead > 0.002)
                Thread.Sleep(TimeSpan.FromSeconds(ahead));

            // The site turns with the body, so it is read again every step, from the booster's own state.
            object? stepAmbient = ambient.GetValue(null);
            ambient.SetValue(null, state);
            double3 up = (double3)Method("SiteDirCciAt").Invoke(null, [home, 0.0])!;
            double3 siteCci = up * (home.MeanRadius + (double)Method("SiteTerrainHeight").Invoke(null, [home])!);
            ambient.SetValue(null, stepAmbient);
            double3 r = booster.Orbit.StateVectors.PositionCci;
            double3 vSurface = booster.Orbit.StateVectors.VelocityCci - double3.Cross(home.GetAngularVelocityCci(), r);
            double3 offset = r - siteCci;
            double legs = double3.Dot(offset, up) - state.VehicleHeightM;
            double range = (offset - up * double3.Dot(offset, up)).Length();
            double vz = double3.Dot(vSurface, up);
            double vh = (vSurface - up * vz).Length();
            (lastLegs, lastVz, lastVh, lastRange) = (legs, vz, vh, range);
            bool contact = booster.Situation.HasAnyContact();
            if (!contact) airborne = true;

            bool threeDof = state.ThreeDofPhase is GuidanceWindow.ThreeDofPhase.Glide or GuidanceWindow.ThreeDofPhase.Burn;
            bool landing = state.LandingPhase is not (GuidanceWindow.LandingPhase.Idle or GuidanceWindow.LandingPhase.Done);
            if (threeDof || state.Active || landing) engaged = true;
            if (SixDofLog.Enabled && SixDofLog.RunName.Length > 0) runName = SixDofLog.RunName;

            string mode = threeDof ? $"entry {state.ThreeDofPhase}"
                : state.Active ? (state.Converging ? "touchdown converging" : "touchdown")
                : landing ? $"landing {state.LandingPhase}"
                : engaged ? "released" : "waiting";
            if (state.ThreeDofGuidance != null)
                lastEntry = state.ThreeDofGuidance;
            // The entry guidance's burn plan, node by node, about once a second from shortly before ignition to the hand-over.
            if (threeDof && state.ThreeDofGuidance?.Published is Plan3Dof plan3 && !ReferenceEquals(plan3, dumpedPlan)
                && plan3.SigmaGlide < 6.0 && sim >= nextPlanDump)
            {
                dumpedPlan = plan3;
                nextPlanDump = sim + 1.0;
                DumpBurnPlan(t, sim, plan3);
            }
            // HOW FAST THE BOOSTER TURNS while the entry guidance flies it: the error to its command, the lateral turn rate and acceleration it achieves, and what KSA's flight computer says it has - its rate limit, RCS and gimbal angular authority (rad/s^2 on the control axes, shown in deg/s^2).
            if (threeDof && sim >= nextTurnProbe)
            {
                nextTurnProbe = sim + 0.1;
                double[] xt = KsaFrameBridge.ToModelState(booster, KsaFrameBridge.BuildSiteFrame(siteCci));
                double3 w = new(xt[10], xt[11], 0.0);
                double qw = xt[6], qx = xt[7], qy = xt[8], qz = xt[9];
                double3 b = new(2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx), 1 - 2 * (qx * qx + qy * qy));
                double3 cmd = state.ThreeDofCommandLocal;
                double err = cmd.Length() > 0.5 ? Math.Acos(Math.Clamp(double3.Dot(b, double3.Normalize(cmd)), -1.0, 1.0)) * 180.0 / Math.PI : double.NaN;
                double rate = w.Length() * 180.0 / Math.PI;
                double accel = double.IsFinite(turnT) ? (w - turnW).Length() / (sim - turnT) * 180.0 / Math.PI : double.NaN;
                (turnW, turnT) = (w, sim);
                bool burning = state.ThreeDofPhase == GuidanceWindow.ThreeDofPhase.Burn;
                if (double.IsFinite(accel))
                {
                    if (burning) { maxBurnRate = Math.Max(maxBurnRate, rate); maxBurnAccel = Math.Max(maxBurnAccel, accel); }
                    else { maxGlideRate = Math.Max(maxGlideRate, rate); maxGlideAccel = Math.Max(maxGlideAccel, accel); }
                }
                if (sim >= nextTurnPrint)
                {
                    nextTurnPrint = sim + 0.5;
                    FlightComputer fc = booster.FlightComputer;
                    const double D = 180.0 / Math.PI;
                    t.Info($"  turn t {sim,6:F2} {state.ThreeDofPhase,-5} z {xt[2],6:F0}  off command {err,5:F1} deg  rate {rate,5:F2} deg/s  accel {accel,6:F2} deg/s2  " +
                           $"| KSA rate limit {fc.RateLimit * D:F1} deg/s, RCS {fc.RcsTorqueAuthority.Y * D:F2}/{fc.RcsTorqueAuthority.Z * D:F2}, gimbal {fc.TvcTorqueAuthority.Y * D:F2}/{fc.TvcTorqueAuthority.Z * D:F2} deg/s2, RCS mode {fc.RCSMode}, engine {(Inputs(booster).EngineOn ? "on" : "off")}");
                }
            }
            if (dumps < 2 && lastEntry != null && !ReferenceEquals(lastEntry, dumpedEntry) && state.ThreeDofStatus.Contains("gave up"))
            {
                dumpedEntry = lastEntry;
                dumps++;
                DumpColdTrace(t, lastEntry);
            }
            // Touchdown guidance's aero model at the measured state and at its plan's state now, against what the booster feels: the measured acceleration less thrust along its axis and gravity.
            if (state.Active && state.Guidance != null && state.Guidance.Published is Ksa6DofPlan plan6 && sim >= nextAeroProbe)
            {
                nextAeroProbe = sim + 0.25;
                double[] xm = KsaFrameBridge.ToModelState(booster, KsaFrameBridge.BuildSiteFrame(siteCci));
                double dtNode = plan6.Sigma / (plan6.Nodes - 1);
                int kp = Math.Clamp((int)(state.Guidance.PlanElapsed / dtNode), 0, plan6.Nodes - 1);
                double[] xp = plan6.X.AsSpan(kp * 14, 14).ToArray();
                double3 am = state.Guidance.ModelAeroAccel(xm), ap = state.Guidance.ModelAeroAccel(xp);
                double3 vm = new(xm[3], xm[4], xm[5]);
                double3 meas = double.IsFinite(probeT) ? (vm - probeV) / (sim - probeT) : double3.Zero;
                double qw = xm[6], qx = xm[7], qy = xm[8], qz = xm[9];
                double3 bm = new(2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx), 1 - 2 * (qx * qx + qy * qy));
                // Off the engines' curve at this pressure, as the bias estimate reads it: throttle x full thrust overstates low throttle by tens of kN.
                double thrustAcc = Inputs(booster).EngineOn && state.LastThrottle > 0.0
                    ? KsaEnginePerf.ThrustAtThrottle(booster, state.LastThrottle,
                          KsaEnginePerf.AmbientPressureAt(booster.Orbit.Parent, siteCci.Length() - booster.Orbit.Parent.MeanRadius + xm[2])) / booster.TotalMass
                    : 0.0;
                double3 feltAero = meas - bm * thrustAcc + new double3(0, 0, 9.81);
                t.Info($"  aero probe z {xm[2]:F0} v ({xm[3]:F1},{xm[4]:F1},{xm[5]:F1}) axis ({bm.X:F3},{bm.Y:F3}) | model@meas ({am.X:F2},{am.Y:F2},{am.Z:F2}) " +
                       $"| plan node {kp} z {xp[2]:F0} v ({xp[3]:F1},{xp[4]:F1},{xp[5]:F1}) model@plan ({ap.X:F2},{ap.Y:F2},{ap.Z:F2}) | felt ({feltAero.X:F2},{feltAero.Y:F2},{feltAero.Z:F2})");
                // The gimbals' torque authority as the allocator sees it now, per newton of the thrust through them, against what the plan allows.
                KsaGimbalControl.Slot slot = KsaGimbalControl.Diagnostics(booster);
                if (slot != null)
                {
                    double through = 0.0;
                    foreach (double th in slot.Thrusts) through += th;
                    TvcAllocationResult la = slot.LastAllocation;
                    // The allocator's own capability in the model's axes, alone and shared: does half the pitch with half the roll fit?
                    if (!couplingProbed && through > 0.0 && slot.Gimbals.Length > 0)
                    {
                        couplingProbed = true;
                        KsaFrameBridge.BodyAxes(booster, out double3 mx, out double3 my, out double3 mz);
                        int ng = slot.Thrusts.Length;
                        GimbalController[] gimbals = slot.Gimbals[..ng];
                        double[] thrusts = slot.Thrusts[..ng];
                        var cmds = new double[2 * ng];
                        double Cap(double3 dir)
                        {
                            TvcAllocationResult r = KsaTvcAllocator.Solve(gimbals, thrusts, booster.CenterOfMassAsmbF, dir * 1e9, cmds);
                            return double3.Dot(r.AchievedTorque, double3.Normalize(dir));
                        }
                        double capX = Cap(mx), capY = Cap(my), capZ = Cap(mz);
                        TvcAllocationResult half = KsaTvcAllocator.Solve(gimbals, thrusts, booster.CenterOfMassAsmbF, mx * (0.5 * capX) + mz * (0.5 * capZ), cmds);
                        TvcAllocationResult most = KsaTvcAllocator.Solve(gimbals, thrusts, booster.CenterOfMassAsmbF, mx * (0.7 * capX) + mz * (0.7 * capZ), cmds);
                        TvcAllocationResult diag = KsaTvcAllocator.Solve(gimbals, thrusts, booster.CenterOfMassAsmbF, (mx + my) * (0.75 * capX), cmds);
                        t.Info($"    coupling at {through / 1e3:F0} kN: model-axis capability x {capX / 1e3:F0} y {capY / 1e3:F0} roll {capZ / 1e3:F0} kN m " +
                               $"({capX / through:F3}/{capY / through:F3}/{capZ / through:F3} m per N); half pitch + half roll sat {half.SaturationScale:F2}; " +
                               $"0.7 + 0.7 sat {most.SaturationScale:F2}; pitch+yaw at 0.75 each (1.06 radial) sat {diag.SaturationScale:F2}");
                    }
                    t.Info($"    gimbals: {through / 1e3:F0} kN through {slot.Thrusts.Length} gimbal(s), max torque KSA ({la.MaxTorque.X / 1e3:F0},{la.MaxTorque.Y / 1e3:F0},{la.MaxTorque.Z / 1e3:F0}) kN m " +
                           $"= {Math.Min(la.MaxTorque.Y, la.MaxTorque.Z) / Math.Max(through, 1.0):F3} m per N; plan allows {state.Guidance.LateralTorquePerNewton:F3} m per N; sat {la.SaturationScale:F2}");
                }
                probeV = vm;
                probeT = sim;
            }
            ManualControlInputs inputs = Inputs(booster);
            while (GuidanceLines.TryDequeue(out string? line))
                t.Info($"  t {sim,6:F2}  | {line}");
            if (mode != lastMode)
            {
                t.Info($"  t {sim,6:F2}  -> {mode}  ({(state.Active ? state.Error : threeDof ? state.ThreeDofStatus : state.LandingStatus)})");
                lastMode = mode;
            }
            if (sim >= nextSample)
            {
                nextSample += SampleS;
                double flow = double.IsFinite(sampleMass) ? (sampleMass - booster.TotalMass) / SampleS : 0.0;
                sampleMass = booster.TotalMass;
                t.Info($"  t {sim,6:F2}  legs {legs,8:F1} m  range {range,6:F1} m  vz {vz,7:F2}  vh {vh,6:F2} m/s  " +
                       $"throttle {(inputs.EngineOn ? inputs.EngineThrottle : 0f),5:P0}  mass {booster.TotalMass,7:F0} ({flow,5:F1} kg/s)  box {2 * booster.BoundingBoxHalfExtentsAsmb.X:F1}x{2 * booster.BoundingBoxHalfExtentsAsmb.Y:F1}x{2 * booster.BoundingBoxHalfExtentsAsmb.Z:F1}  {mode}" +
                       (threeDof ? $"  [{state.ThreeDofStatus}] [{state.ThreeDofSolveText}]" : ""));
            }

            if (airborne && contact && !touched)
            {
                touched = true;
                touchSink = -vz;
                touchSideways = vh;
                touchMiss = range;
                endAt = sim + 1.0;
                t.Info($"  t {sim,6:F2}  CONTACT at {touchSink:F2} m/s down, {touchSideways:F2} m/s sideways, {touchMiss:F1} m from the site");
            }
            // Guidance let go in the air: a few seconds more to see where it goes. The hand-over from entry to touchdown guidance passes through a frame of neither, so a release that ends puts the clock back.
            if (mode == "released" && !touched && double.IsPositiveInfinity(endAt))
                endAt = sim + 5.0;
            else if (mode != "released" && !touched)
                endAt = double.PositiveInfinity;
            if (legs < -50.0)
                break;
        }

        t.Info($"flown {sim:F1} s of sim in {wall.Elapsed.TotalSeconds:F1} s");
        t.Info($"entry guidance turning, peak achieved: glide {maxGlideRate:F2} deg/s and {maxGlideAccel:F2} deg/s2, burn {maxBurnRate:F2} deg/s and {maxBurnAccel:F2} deg/s2");
        if (runName.Length > 0)
            t.Info($"6-DOF log: {Path.Combine(SixDofLog.Directory, runName)}-cycle.csv / -plan.csv / -events.log");
        else
            t.Info("6-DOF log: none written");

        t.Check("the booster touched down", touched);
        if (touched)
        {
            t.Check("touchdown under 3 m/s down", touchSink < 3.0, $"{touchSink:F2} m/s");
            t.Check("touchdown under 1 m/s sideways", touchSideways < 1.0, $"{touchSideways:F2} m/s");
            t.Check("touchdown within 10 m of the site", touchMiss < 10.0, $"{touchMiss:F1} m");
        }
    }

    // One entry-guidance plan's burn: per node the time from the plan's start, height, sideways and vertical speed, the thrust axis's tilt off vertical resolved along and across the sideways motion (positive along = leaning with it), the turn rate and turn acceleration to the next node, and the throttle.
    private static void DumpBurnPlan(TestContext t, double sim, Plan3Dof p)
    {
        int nx = PointMass3Dof.NX, nu = PointMass3Dof.NU, g = p.GlideIntervals, n = p.Nodes;
        double dtBurn = p.SigmaBurn / Math.Max(n - 1 - g, 1);
        t.Info($"  t {sim,6:F2}  entry plan {p.Phase}: glide {p.SigmaGlide:F2} s, burn {p.SigmaBurn:F2} s over {n - 1 - g} intervals of {dtBurn:F3} s, defect {p.GatedDefect:G3}");
        double prevRate = double.NaN;
        double3 prevB = default;
        for (int k = g; k < n; k++)
        {
            double tk = p.SigmaGlide + (k - g) * dtBurn;
            double z = p.X[k * nx + PointMass3Dof.IR + 2];
            double vx = p.X[k * nx + PointMass3Dof.IV], vy = p.X[k * nx + PointMass3Dof.IV + 1], vzk = p.X[k * nx + PointMass3Dof.IV + 2];
            double3 b = new(p.U[k * nu + PointMass3Dof.IB], p.U[k * nu + PointMass3Dof.IB + 1], p.U[k * nu + PointMass3Dof.IB + 2]);
            double bl = b.Length();
            if (bl > 0.0) b /= bl;
            double vh = Math.Sqrt(vx * vx + vy * vy);
            double ux = vh > 0.05 ? vx / vh : 1.0, uy = vh > 0.05 ? vy / vh : 0.0;
            double along = Math.Asin(Math.Clamp(b.X * ux + b.Y * uy, -1.0, 1.0)) * 180.0 / Math.PI;
            double across = Math.Asin(Math.Clamp(-b.X * uy + b.Y * ux, -1.0, 1.0)) * 180.0 / Math.PI;
            string rate = "", accel = "";
            if (k > g)
            {
                double ang = Math.Acos(Math.Clamp(double3.Dot(prevB, b), -1.0, 1.0)) * 180.0 / Math.PI;
                double r = ang / dtBurn;
                rate = $"{r,6:F1}";
                if (!double.IsNaN(prevRate))
                    accel = $"{(r - prevRate) / dtBurn,7:F1}";
                prevRate = r;
            }
            prevB = b;
            t.Info($"    node {k,2} t {tk,5:F2}  z {z,6:F1}  vh {vh,5:F1} vz {vzk,6:F1}  tilt along {along,5:F1} across {across,5:F1} deg  " +
                   $"rate {rate,6} deg/s accel {accel,7} deg/s2  throttle {p.U[k * nu + PointMass3Dof.ITH]:F3}");
        }
    }

    // The cold solve's SCvx trace, when it gives up: every tenth iteration and the last five.
    private static void DumpColdTrace(TestContext t, Guidance3Dof entry)
    {
        var solver = (Scvx3DofSolver)typeof(Guidance3Dof).GetField("_two", BindingFlags.NonPublic | BindingFlags.Instance)!.GetValue(entry)!;
        IReadOnlyList<Scvx3DofIteration> trace = solver.Trace;
        t.Info($"  cold trace: {trace.Count} iterations, last failure '{solver.LastFailureReason}', trust {solver.TrustRegion:G3}, " +
               $"sigma glide {solver.SigmaGlide:F1} s burn {solver.SigmaBurn:F1} s, nodes {solver.Config.Nodes}, glide intervals {solver.Config.GlideIntervals}");
        for (int i = 0; i < trace.Count; i++)
        {
            if (i % 10 != 0 && i < trace.Count - 5)
                continue;
            Scvx3DofIteration it = trace[i];
            t.Info($"    it {it.Index,3}  solved {it.Solved,-5} acc {it.Accepted,-5} rho {it.Rho,9:G3}  trust {it.TrustRegion,8:G3}  " +
                   $"sg {it.SigmaGlide,6:F1} sb {it.SigmaBurn,5:F1}  step {it.Step,8:G3}  defect {it.DefectNorm,8:G3} " +
                   $"({Scvx3DofSolver.ChannelName(it.DefectChannel)} @ {it.DefectNode})  cost {it.Cost,10:G4}  admm {it.SolverIterations}");
        }
    }

    // VehicleSpawner.SpawnFromSave with an attitude: it spawns at identity, which would put the booster on its side.
    private static Vehicle Spawn(string saveId, CelestialSystem system, IParentBody parent, string id, Orbit orbit, doubleQuat body2Cce)
    {
        VehicleSave? save = null;
        foreach (VehicleSave candidate in VehicleSaves.AsSpan())
            if (string.Equals(candidate.Id, saveId, StringComparison.Ordinal))
            {
                save = candidate;
                break;
            }
        if (save == null)
            throw new InvalidOperationException($"vehicle save '{saveId}' not found in the game's Vehicles folder.");
        PartInstance design = save.VehicleSaveData.RootPartInstance
            ?? throw new InvalidOperationException($"vehicle save '{saveId}' has no root part instance.");
        PartTree tree = PartTree.Deserialize(design);
        Vehicle vehicle = Vehicle.CreateVehicle(system, body2Cce, double3.Zero, parent, id, tree.Root, orbit);
        vehicle.Parts.SequenceList.SetActiveSequence(save.VehicleSaveData.ActiveSequence);
        vehicle.Parts.SequenceList.ApplyEnvironments(save.VehicleSaveData.SequenceEnvironments);
        vehicle.Parts.FuelLinks.ApplySaveData(save.VehicleSaveData.FuelLinks, design);
        parent.Children.Add(vehicle);
        RestoreTanks(vehicle.Parts, design);
        VehicleSpawner.SeatRandomCrew(vehicle);
        return vehicle;
    }

    // Applying the fuel links re-plans every tank's mixture from the engines it can feed - and the resource graph is not built yet, so no tank has an engine to feed and every one is emptied. The save's contents go back in, and the tanks are pinned to them so the re-plan at separation leaves them be.
    private static readonly MethodInfo RecreateResourceManagers =
        typeof(PartTree).GetMethod("RecreateResourceManagers", BindingFlags.NonPublic | BindingFlags.Instance)
        ?? throw new MissingMethodException(nameof(PartTree), "RecreateResourceManagers");

    private static void RestoreTanks(PartTree tree, PartInstance design)
    {
        var parts = new PartTreeIterator(tree.Root);
        var instances = new PartInstanceTreeIterator(design);
        while (parts.GetNextNode() is Part part && instances.GetNextNode() is PartInstance instance)
            foreach (ModuleBase.SaveDataBase component in instance.Components)
                if (component is Tank.SaveData)
                    part.Modules.ApplySaveData(component, tree.States);
        foreach (Tank tank in tree.Tanks.Modules)
            tank.IsManuallyAssigned = true;
        RecreateResourceManagers.Invoke(tree, null);
    }

    // Fires the stack's rows - ignition first, which lights the booster's engines - until one separates something, keeps the separated craft with the most engines as the booster, and removes the rest.
    private static Vehicle? SeparateBooster(TestContext t, Vehicle stack, SimDriver driver, HashSet<string> preexisting)
    {
        for (int row = 0; row < 6; row++)
        {
            stack.Parts.SequenceList.ActivateNextSequence(stack);
            stack.UpdateAfterPartTreeModification();
            driver.Step(StepS, 5);

            List<Vehicle> separated = new();
            foreach (Astronomical body in t.System.All.AsSpan())
                if (body is Vehicle v && v is not KittenEva && !preexisting.Contains(v.Id) && !ReferenceEquals(v, stack))
                    separated.Add(v);
            if (separated.Count == 0)
                continue;

            // The booster is whichever side kept the lit engines and the bigger tanks - which side keeps the original craft depends on where the save's root part is.
            List<Vehicle> crafts = new(separated) { stack };
            foreach (Vehicle v in crafts)
            {
                Propellant(v, out double kg, out double cap, out int tanks);
                t.Info($"row {row + 1}: '{v.Id}' {v.TotalMass:F0} kg, {v.Parts.Modules.Get<EngineController>().Length} engine(s), " +
                       $"{ActiveEngines(v)} active, {tanks} tank(s), {kg:F0} of {cap:F0} kg propellant");
            }
            Vehicle? booster = crafts.Where(v => ActiveEngines(v) > 0)
                .OrderByDescending(v => { Propellant(v, out _, out double cap, out _); return cap; })
                .FirstOrDefault();
            if (booster == null)
            {
                t.Fail("separation", "neither side of the separation has a lit engine");
                return null;
            }
            foreach (Vehicle other in crafts)
                if (!ReferenceEquals(other, booster))
                    VehicleSpawner.Despawn(other);
            t.Info($"row {row + 1} separated the booster '{booster.Id}'");
            return booster;
        }
        t.Fail("separation", "no row of the stack separated anything");
        return null;
    }

    private static int ActiveEngines(Vehicle vehicle)
    {
        int active = 0;
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
            if (engine.IsActive) active++;
        return active;
    }

    private static void Propellant(Vehicle vehicle, out double kg, out double capacityKg, out int tanks)
    {
        kg = capacityKg = 0.0;
        tanks = 0;
        ReadOnlySpan<MoleState> states = vehicle.Parts.Moles.States;
        foreach (Tank tank in vehicle.Parts.Tanks.Modules)
        {
            tanks++;
            kg += tank.ComputeSubstanceMass(states);
            foreach (Mole mole in tank.Moles)
                capacityKg += mole.GetStoredMass(mole.ContainerVolume);
        }
    }

    // Every tank to the same fraction of its capacity, so the load and its mixture come out as the target.
    private static void SetPropellant(Vehicle vehicle, double targetKg, out double loadedKg, out double capacityKg)
    {
        capacityKg = 0.0;
        foreach (Tank tank in vehicle.Parts.Tanks.Modules)
            foreach (Mole mole in tank.Moles)
                capacityKg += mole.GetStoredMass(mole.ContainerVolume);
        double fraction = capacityKg > 0.0 ? Math.Clamp(targetKg / capacityKg, 0.0, 1.0) : 0.0;
        loadedKg = 0.0;
        foreach (Tank tank in vehicle.Parts.Tanks.Modules)
            foreach (Mole mole in tank.Moles)
            {
                var state = vehicle.Parts.Moles.GetModuleAndAllMutableStatesForInitialization(mole);
                state.State.Mass = (float)(fraction * mole.GetStoredMass(mole.ContainerVolume));
                loadedKg += state.State.Mass;
            }
    }

    private static MethodInfo Method(string name) =>
        typeof(GuidanceWindow).GetMethod(name, PrivateStatic)
        ?? throw new MissingMethodException(nameof(GuidanceWindow), name);
}

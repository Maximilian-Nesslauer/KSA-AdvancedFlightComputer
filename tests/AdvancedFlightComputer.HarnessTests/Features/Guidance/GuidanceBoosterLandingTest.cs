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
            t.Info($"burn timed for {state.LandingAccelLimitG:F1} g, hand-over with {state.ThreeDofSixDofTimeS:F1} s of burn left");
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
        Guidance3Dof? lastEntry = null, dumpedEntry = null;
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
            if (dumps < 2 && lastEntry != null && !ReferenceEquals(lastEntry, dumpedEntry) && state.ThreeDofStatus.Contains("gave up"))
            {
                dumpedEntry = lastEntry;
                dumps++;
                DumpColdTrace(t, lastEntry);
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
                       $"throttle {(inputs.EngineOn ? inputs.EngineThrottle : 0f),5:P0}  mass {booster.TotalMass,7:F0} ({flow,5:F1} kg/s)  {mode}" +
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

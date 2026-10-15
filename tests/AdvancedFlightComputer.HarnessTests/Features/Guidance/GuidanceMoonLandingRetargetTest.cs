using System.Collections.Concurrent;
using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Opt-in measurement: the 2026-10-13 lunar landing as it was flown - the moon_des save's hopper, EXECUTE pressed, the deorbit plan, the coast, UPFG's braking burn and its hand-over to touchdown guidance (6-DOF) - with the target moved a set time after the hand-over.
//
// FOR COMPARING BUILDS. It touches only what main already had (ExecuteLanding, the guidance state's switches, site, Active, Converging and Error), so the same DLL flies main and every commit since.
//
// Paced: AFC_ML_FAST_PACE times real time until touchdown guidance engages, real time after it, since its solves run against the wall clock. AFC_RT_SHIFT_M (north, 0 for none) and AFC_RT_AT_S (seconds after the hand-over) move the target.
public sealed class GuidanceMoonLandingRetargetTest : AfcTest
{
    public override string Name => "afc-guidance-moon-landing-retarget";
    public override bool OptIn => true;

    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;
    private const string SaveName = "moon_des";
    private const double StepS = 0.02;

    private static double Env(string name, double fallback)
        => double.TryParse(Environment.GetEnvironmentVariable(name), NumberStyles.Float, CultureInfo.InvariantCulture, out double v) ? v : fallback;

    private static readonly ConcurrentQueue<string> GuidanceLines = new();
    private static void QueueGuidanceLine(Vehicle vehicle, string message) => GuidanceLines.Enqueue(message);

    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> Inputs =
        AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

    protected override void Execute(TestContext t)
    {
        string path = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            "My Games", "Kitten Space Agency", "saves", SaveName, "universe.xml");
        if (!File.Exists(path) || t.System.All.Get("Luna") is not Celestial lunaCelestial || lunaCelestial is not IParentBody luna)
        {
            t.Skip($"{SaveName} and the Luna body are required.");
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
        bool previousModActive = GuidanceWindow.ModActive;
        bool previousEnabled = SharedVehicleHooks.GuidanceEnabled;
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.moonlanding");
        VehicleCommandSink.ApplyPatches(harmony);
        GuidanceFeature.ApplyDriverPatches(harmony);
        harmony.Patch(AccessTools.Method(typeof(GuidanceLog), nameof(GuidanceLog.Info)),
            prefix: new HarmonyMethod(typeof(GuidanceMoonLandingRetargetTest), nameof(QueueGuidanceLine)));
        GuidanceLines.Clear();
        SharedVehicleHooks.GuidanceEnabled = true;
        GuidanceWindow.SetModActive(true);
        SimDriver driver = t.Session.CreateDriver();
        driver.StepOrbits = true;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool railsBefore = PhysicsBubble._forceOffRails;
        Vehicle? craft = null;
        try
        {
            driver.Step(StepS, 2);
            using var reader = new StreamReader(path);
            var save = (UniverseData)GameSaves.UniverseSerializer.Deserialize(reader)!;
            save.OnDataLoad(KSA.Mod.Empty);
            VehicleData data = save.CelestialSystems.SelectMany(system => system.Vehicles).Single(vehicle => vehicle.Id == "hopper");
            // Moved to now: the save's own epoch is not the harness's, and its state is where the flight began.
            Orbit live = Orbit.CreateFromStateCci(luna, Universe.GetElapsedTime(), data.AnalyticState.PositionCci, data.AnalyticState.VelocityCci, new byte4(255, 255, 255, 255));
            craft = VehicleFixtures.SpawnFromSaveData(t.System, luna, data, "HarnessMoonLanding", live);
            PhysicsBubble._forceOffRails = true;
            Program.ControlledVehicle = craft;
            driver.Step(StepS, 2);
            t.Info($"hopper '{craft.Id}': {craft.TotalMass:F0} kg, {(craft.Orbit.StateVectors.PositionCci.Length() - luna.MeanRadius) / 1000:F1} km up");

            VehicleAutopilotState state = VehicleAutopilotState.For(craft);
            state.SixDofLogging = true;
            state.Engage = true;
            state.AutoStage = true;
            state.UseSixDofLanding = true;
            ambient.SetValue(null, state);
            Method("ExecuteLanding").Invoke(null, [craft]);
            ambient.SetValue(null, previousAmbient);
            t.Info($"EXECUTE: {state.LandingStatus}");

            Fly(t, craft, state, driver, luna, ambient);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            GuidanceFeature.DisableDriver();
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.SetModActive(previousModActive);
            Program.ControlledVehicle = previousFocus;
            if (craft != null)
                VehicleAutopilotState.Remove(craft);
            ambient.SetValue(null, previousAmbient);
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void Fly(TestContext t, Vehicle craft, VehicleAutopilotState state, SimDriver driver, IParentBody luna, FieldInfo ambient)
    {
        double fastPace = Math.Max(Env("AFC_ML_FAST_PACE", 20.0), 1.0);
        double shiftM = Env("AFC_RT_SHIFT_M", 250.0);
        double retargetAfterS = Env("AFC_RT_AT_S", 13.0);
        double timeoutS = Env("AFC_ML_TIMEOUT_S", 1500.0);

        var wall = Stopwatch.StartNew();
        double sim = 0.0, wallBase = 0.0, simBase = 0.0, pace = fastPace, nextSample = 0.0;
        double engagedAt = double.NaN, firstPlanAt = double.NaN, convergingAfterMove = 0.0;
        bool moved = false, touched = false, airborne = false;
        double lastAlt = double.NaN, lastVz = double.NaN, lastVh = double.NaN, lastRange = double.NaN;
        double touchSink = double.NaN, touchSideways = double.NaN, touchMiss = double.NaN;
        while (sim < timeoutS)
        {
            try
            {
                driver.Step(StepS);
            }
            catch (NullReferenceException) when (lastAlt < 50.0)
            {
                t.Info($"  t {sim,7:F2}  IMPACT at {-lastVz:F2} m/s down, {lastVh:F2} m/s sideways, {lastRange:F1} m from the site");
                (touched, touchSink, touchSideways, touchMiss) = (true, -lastVz, lastVh, lastRange);
                break;
            }
            sim += StepS;
            // Real time once touchdown guidance flies: its solves run against the wall clock.
            // Engagement noted on its own, so a run paced at 1x from the start still records it.
            if (state.Active && double.IsNaN(engagedAt))
                engagedAt = sim;
            if (state.Active && pace != 1.0)
                (pace, simBase, wallBase) = (1.0, sim, wall.Elapsed.TotalSeconds);
            double ahead = (sim - simBase) / pace - (wall.Elapsed.TotalSeconds - wallBase);
            if (ahead > 0.002)
                Thread.Sleep(TimeSpan.FromSeconds(ahead));

            if (!moved && shiftM != 0.0 && double.IsFinite(engagedAt) && sim - engagedAt >= retargetAfterS)
            {
                moved = true;
                state.SiteLatDeg += shiftM / luna.MeanRadius * 180.0 / Math.PI;
                t.Info($"  t {sim,7:F2}  TARGET MOVED {shiftM:F0} m north, {sim - engagedAt:F1} s after the hand-over");
            }

            object? stepAmbient = ambient.GetValue(null);
            ambient.SetValue(null, state);
            double3 up = (double3)Method("SiteDirCciAt").Invoke(null, [luna, 0.0])!;
            double3 siteCci = up * (luna.MeanRadius + (double)Method("SiteTerrainHeight").Invoke(null, [luna])!);
            ambient.SetValue(null, stepAmbient);
            double3 r = craft.Orbit.StateVectors.PositionCci;
            double3 vSurface = craft.Orbit.StateVectors.VelocityCci - double3.Cross(luna.GetAngularVelocityCci(), r);
            double3 offset = r - siteCci;
            double alt = double3.Dot(offset, up);
            double range = (offset - up * alt).Length();
            double vz = double3.Dot(vSurface, up);
            double vh = (vSurface - up * vz).Length();
            (lastAlt, lastVz, lastVh, lastRange) = (alt, vz, vh, range);
            bool contact = craft.Situation.HasAnyContact();
            if (!contact) airborne = true;

            if (state.Active && !state.Converging && double.IsNaN(firstPlanAt))
                firstPlanAt = sim;
            if (moved && state.Active && state.Converging)
                convergingAfterMove += StepS;

            while (GuidanceLines.TryDequeue(out string? line))
                if (!line.Contains("UPFG Burn") && !line.Contains("UPFG Prep"))
                    t.Info($"  t {sim,7:F2}  | {line}");

            if (sim >= nextSample)
            {
                nextSample += state.Active ? 0.5 : 10.0;
                ManualControlInputs inputs = Inputs(craft);
                string mode = state.Active ? state.Converging ? "6-DOF converging" : "6-DOF flying a plan" : $"landing {state.LandingPhase}";
                t.Info($"  t {sim,7:F2}  alt {alt,8:F1} m  range {range,8:F1} m  vz {vz,7:F2}  vh {vh,7:F2} m/s  " +
                       $"throttle {(inputs.EngineOn ? inputs.EngineThrottle : 0f),4:P0}  {mode}  [{Trim(state.Active ? state.Error : state.LandingStatus)}]");
            }

            if (airborne && contact && alt < 50.0)
            {
                (touched, touchSink, touchSideways, touchMiss) = (true, -vz, vh, range);
                t.Info($"  t {sim,7:F2}  TOUCHDOWN at {-vz:F2} m/s down, {vh:F2} m/s sideways, {range:F1} m from the site");
                break;
            }
            if (double.IsFinite(engagedAt) && !state.Active && state.LandingPhase.ToString() is "Idle" or "Done")
            {
                t.Info($"  t {sim,7:F2}  guidance let go at {alt:F0} m: {Trim(state.Error)}");
                break;
            }
        }

        t.Info($"touchdown guidance engaged {(double.IsNaN(engagedAt) ? "never" : $"at {engagedAt:F1} s")}, first plan " +
               $"{(double.IsNaN(firstPlanAt) ? "never" : $"{firstPlanAt - engagedAt:F1} s after")}, {convergingAfterMove:F1} s converging after the move, flown {sim:F1} s");
        t.Check("touchdown guidance engaged", !double.IsNaN(engagedAt));
        t.Check("touchdown guidance found a first plan", !double.IsNaN(firstPlanAt));
        t.Check("the hopper touched down", touched);
        if (touched)
        {
            t.Check($"touchdown under 3 m/s down: {touchSink:F2} m/s", touchSink < 3.0);
            t.Check($"touchdown under 1 m/s sideways: {touchSideways:F2} m/s", touchSideways < 1.0);
            t.Check($"touchdown within 10 m of the site: {touchMiss:F1} m", touchMiss < 10.0);
        }
    }

    private static string Trim(string s) => s.Length > 110 ? s[..110] : s;

    private static MethodInfo Method(string name) =>
        typeof(GuidanceWindow).GetMethod(name, PrivateStatic)
        ?? throw new MissingMethodException(nameof(GuidanceWindow), name);
}

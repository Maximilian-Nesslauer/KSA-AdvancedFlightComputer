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

// Opt-in flight: the player's belly-flop craft 'flopper' flown by ORBITAL ENTRY from the middle of an entry to 100 m over the site, bellyflop taking over on the way, on KSAero's aerodynamics (its hinged flaps are KSAero parts).
//
// RUN WITH KSAero's CONTENT: run-headless.ps1 -ContentMods KSAero -Tests afc-guidance-flopper-entry -Vehicle ''. The minimal manifest loads no mod's parts, and KsaeroScope loads only KSAero's code, so without the flag the flopper's flaps have no part template and the test skips.
//
// THE START is the 2026-11-23 11:36 flight at Mach 12: 48 km up, 4.2 km/s over the ground, 1.3 deg down, 720 km short of the site on the great circle through it heading east, belly first at 40 deg from nose-first, the craft's belly axis (+Z) to the air. AFC_FLOPPER_* override it.
//
// PACED: the entry and bellyflop solve on their own threads against the wall clock, every 2 s and 1 s of flight, so the flight runs at AFC_FLOPPER_PACE times real time (2 by default, as a player flies it under light warp).
//
// THE END is 100 m over the site's ground: there is no landing burn yet (bellyflop's flip is touchdown guidance's, not built), and the craft would hit at its terminal speed. The checks: the entry engages, bellyflop takes over between Mach 5.5 and 4 (HAND-OVER MACH 5), and the craft is within AFC_FLOPPER_MISS_M (1000 m) of the site there. The entry's CSV is in navbox-logs, named in the log lines.
public sealed class GuidanceFlopperEntryTest : AfcTest
{
    public override string Name => "afc-guidance-flopper-entry";
    public override bool OptIn => true;

    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;
    private const string VehicleSave = "flopper";
    private const double StepS = 0.02;
    private const double SampleS = 10.0;
    private const double TimeoutS = 2400.0;
    private const double EngageWaitS = 60.0;
    private const double EndHeightM = 100.0;

    private static double Env(string name, double fallback)
        => double.TryParse(Environment.GetEnvironmentVariable(name), NumberStyles.Float, CultureInfo.InvariantCulture, out double v) ? v : fallback;

    private static readonly ConcurrentQueue<string> GuidanceLines = new();

    private static void QueueGuidanceLine(Vehicle vehicle, string message) => GuidanceLines.Enqueue(message);

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        if (VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state.");
            return;
        }
        if (!(home.GetAtmosphereRadius() > home.MeanRadius))
        {
            t.Skip("the home body has no atmosphere");
            return;
        }

        double altitudeM = Env("AFC_FLOPPER_ALT_M", 48_000.0);
        double speedMs = Env("AFC_FLOPPER_SPEED_MS", 4200.0);
        double gammaDeg = Env("AFC_FLOPPER_GAMMA_DEG", -1.3);
        double rangeM = Env("AFC_FLOPPER_RANGE_M", 720_000.0);
        double crossM = Env("AFC_FLOPPER_CROSS_M", 0.0);
        double aoaDeg = Env("AFC_FLOPPER_AOA_DEG", 40.0);
        double pace = Math.Max(Env("AFC_FLOPPER_PACE", 2.0), 0.05);
        double missLimitM = Env("AFC_FLOPPER_MISS_M", 1000.0);
        // AFC_FLOPPER_HEAT_KWM2 sets MAX HEATING (0 off), to see what the limit does to the trajectory; unset keeps the default.
        double heatKwm2 = Env("AFC_FLOPPER_HEAT_KWM2", double.NaN);

        FieldInfo ambient = typeof(GuidanceWindow).GetField("_s", PrivateStatic)!;
        object? previousAmbient = ambient.GetValue(null);
        Vehicle? previousFocus = Program.ControlledVehicle;
        bool previousModActive = GuidanceWindow.ModActive;
        bool previousEnabled = SharedVehicleHooks.GuidanceEnabled;
        using KsaeroScope aero = KsaeroScope.Apply();
        t.Info(aero.Status);
        if (!aero.Active)
        {
            t.Skip("the flopper's flaps are KSAero parts: " + aero.Status);
            return;
        }
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.flopperentry");
        VehicleCommandSink.ApplyPatches(harmony);
        GuidanceFeature.ApplyDriverPatches(harmony);
        harmony.Patch(AccessTools.Method(typeof(GuidanceLog), nameof(GuidanceLog.Info)),
            prefix: new HarmonyMethod(typeof(GuidanceFlopperEntryTest), nameof(QueueGuidanceLine)));
        GuidanceLines.Clear();
        SharedVehicleHooks.GuidanceEnabled = true;
        GuidanceWindow.SetModActive(true);
        // The bodies turn, as in flight: the site moves with the ground and the craft is given the air's velocity relative to it.
        SimDriver driver = t.Session.CreateDriver();
        driver.StepOrbits = true;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool railsBefore = PhysicsBubble._forceOffRails;
        Vehicle? craft = null;
        try
        {
            driver.Step(StepS, 2);
            ambient.SetValue(null, new VehicleAutopilotState());
            double3 siteUp = (double3)Method("SiteDirCciAt").Invoke(null, [home, 0.0])!;
            double3 omega = home.GetAngularVelocityCci();
            double3 east = double3.Cross(omega, siteUp);
            east = east.Length() > 1e-12 ? double3.Normalize(east) : double3.Normalize(double3.Cross(double3.UnitZ, siteUp));
            double3 north = double3.Cross(siteUp, east);

            // On the great circle through the site heading east, rangeM short of it, crossM to its north.
            double back = -rangeM / home.MeanRadius;
            double3 up0 = siteUp * Math.Cos(back) + east * Math.Sin(back);
            double3 east0 = double3.Normalize(double3.Cross(north, up0));
            double side = crossM / home.MeanRadius;
            up0 = double3.Normalize(up0 * Math.Cos(side) + north * Math.Sin(side));
            east0 = double3.Normalize(east0 - up0 * double3.Dot(east0, up0));
            double3 r0 = up0 * (home.MeanRadius + altitudeM);
            double gamma = gammaDeg * Math.PI / 180.0;
            double3 vHat = east0 * Math.Cos(gamma) + up0 * Math.Sin(gamma);
            double3 v0 = vHat * speedMs + double3.Cross(omega, r0);
            Orbit orbit = Orbit.CreateFromStateCci(home, driver.Elapsed, r0, v0, VehicleSpawner.OrbitLineColor);

            // Belly first at aoaDeg from nose-first: the nose that far above the airflow towards the lift, the belly (+Z) to the air.
            double alpha = aoaDeg * Math.PI / 180.0;
            double3 lift = double3.Normalize(up0 - vHat * double3.Dot(up0, vHat));
            double3 nose = double3.Normalize(vHat * Math.Cos(alpha) + lift * Math.Sin(alpha));
            double3 belly = double3.Normalize(vHat * Math.Sin(alpha) - lift * Math.Cos(alpha));
            double3 span = double3.Cross(belly, nose);
            doubleQuat body2Cci = doubleQuat.CreateFromRotationMatrix(new double4x4(
                nose.X, nose.Y, nose.Z, 0.0,
                span.X, span.Y, span.Z, 0.0,
                belly.X, belly.Y, belly.Z, 0.0,
                0.0, 0.0, 0.0, 1.0));
            doubleQuat body2Cce = doubleQuat.Concatenate(body2Cci, home.GetCci2Cce());

            try
            {
                craft = GuidanceBoosterLandingTest.Spawn(VehicleSave, t.System, home, "HarnessFlopperEntry", orbit, body2Cce, freshDesign: false);
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"{VehicleSave}: {e.Message}");
                return;
            }
            catch (NullReferenceException e) when (e.Message.Contains("PartTemplate", StringComparison.Ordinal))
            {
                t.Skip($"{VehicleSave}: {e.Message}. KSAero's parts are not loaded: run with -ContentMods KSAero.");
                return;
            }
            PhysicsBubble._forceOffRails = true;
            craft.Parts.RecomputeTotalFillFraction();
            craft.UpdateAfterPartTreeModification();
            Program.ControlledVehicle = craft;
            driver.Step(StepS, 2);
            t.Info($"'{craft.Id}': {craft.TotalMass:F0} kg, from {altitudeM / 1000:F1} km at {speedMs:F0} m/s, {gammaDeg:F1} deg, " +
                   $"{rangeM / 1000:F0} km short of the site and {crossM / 1000:F1} km to its north, {aoaDeg:F0} deg from nose-first; pace {pace:F1}x");

            VehicleAutopilotState state = VehicleAutopilotState.For(craft);
            state.EntryLogging = true;
            if (double.IsFinite(heatKwm2))
                state.OrbitalEntryHeatMaxKWm2 = heatKwm2;
            state.AutoStage = false;
            // As the console's ORBITAL ENTRY does (RequestOrbitalEntry): the next step engages it from the craft's state.
            state.OrbitalEntry.StopPending = false;
            state.OrbitalEntry.EngagePending = true;
            t.Info($"heating limit {state.OrbitalEntryHeatMaxKWm2:F0} kW/m2 on a {state.OrbitalEntryNoseRadiusM:F2} m nose, q {state.OrbitalEntryQMaxKPa:F0} kPa, load {state.OrbitalEntryGMax:F1} g");
            t.Info($"hand-over Mach {state.OrbitalEntryHandoverMach:F1} at {state.OrbitalEntryHandoverAltKm:F0} km, bellyflop solving from Mach {state.OrbitalEntryBellyflopArmMach:F0}, " +
                   $"pitch {state.OrbitalEntryBellyflopAngleMaxDeg:F0} down / {state.OrbitalEntryBellyflopPitchUpDeg:F0} up, bank {state.OrbitalEntryBellyflopBankMaxDeg:F0} deg");

            Fly(t, craft, state, driver, home, ambient, pace, missLimitM);
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

    private static void Fly(TestContext t, Vehicle craft, VehicleAutopilotState state, SimDriver driver, IParentBody home,
                            FieldInfo ambient, double pace, double missLimitM)
    {
        var wall = Stopwatch.StartNew();
        double sim = 0.0, nextSample = 0.0;
        double engagedAt = double.NaN, takeOverMach = double.NaN, takeOverAlt = double.NaN, takeOverRange = double.NaN;
        double peakLoadG = 0.0, peakLoadBellyflopG = 0.0, peakHeat = 0.0, peakHeatAlt = double.NaN, peakHeatMach = double.NaN, peakQ = 0.0, leastHeat = double.NaN;
        double endMiss = double.NaN, endSpeed = double.NaN;
        GuidanceWindow.OrbitalEntryPhase lastPhase = GuidanceWindow.OrbitalEntryPhase.Idle;
        bool ended = false;

        while (sim < TimeoutS)
        {
            try
            {
                driver.Step(StepS);
            }
            catch (NullReferenceException)
            {
                // KSA's ground-impact effect has no renderer to spawn into headless: the craft hit the ground before the end height.
                t.Info($"  t {sim,6:F1}  IMPACT before {EndHeightM:F0} m over the site");
                break;
            }
            sim += StepS;
            double ahead = sim / pace - wall.Elapsed.TotalSeconds;
            if (ahead > 0.002)
                Thread.Sleep(TimeSpan.FromSeconds(ahead));
            while (GuidanceLines.TryDequeue(out string? line))
                t.Info($"  t {sim,6:F1}  | {line}");

            var st = state.OrbitalEntry;
            // The site turns with the body: read it from the craft's own state.
            object? stepAmbient = ambient.GetValue(null);
            ambient.SetValue(null, state);
            double3 up = (double3)Method("SiteDirCciAt").Invoke(null, [home, 0.0])!;
            double3 siteCci = up * (home.MeanRadius + (double)Method("SiteTerrainHeight").Invoke(null, [home])!);
            ambient.SetValue(null, stepAmbient);
            double3 r = craft.Orbit.StateVectors.PositionCci;
            double3 offset = r - siteCci;
            // Over the site's ground, as the craft's own altitude: the height along the site's vertical is far below it a long way off.
            double height = r.Length() - siteCci.Length();
            double ground = (offset - up * double3.Dot(offset, up)).Length();
            double3 vSurface = craft.Orbit.StateVectors.VelocityCci - double3.Cross(home.GetAngularVelocityCci(), r);

            // The planner's own estimate (EntryModel.HeatingRate, Sutton-Graves) in the air the sweep measured.
            if (st.Aero?.Atmosphere is { } atm && st.SpeedMs > 100.0)
            {
                double rho = atm.Density(st.AltitudeM);
                double heat = 1.7415e-4 * Math.Sqrt(rho / Math.Max(state.OrbitalEntryNoseRadiusM, 0.05)) * Math.Pow(st.SpeedMs, 3);
                if (heat > peakHeat)
                    (peakHeat, peakHeatAlt, peakHeatMach) = (heat, st.AltitudeM, st.Mach);
                // The least peak from here on falls as the entry goes; the first, from the top, is the craft's.
                if (double.IsFinite(st.HeatFloorKWm2) && !(st.HeatFloorKWm2 <= leastHeat))
                    leastHeat = st.HeatFloorKWm2;
                peakQ = Math.Max(peakQ, 0.5 * rho * st.SpeedMs * st.SpeedMs);
            }
            if (st.AeroAccelKnown)
            {
                double g = st.AeroAccelCci.Length() / 9.80665;
                peakLoadG = Math.Max(peakLoadG, g);
                if (st.Phase == GuidanceWindow.OrbitalEntryPhase.Bellyflop)
                    peakLoadBellyflopG = Math.Max(peakLoadBellyflopG, g);
            }
            if (st.Phase != lastPhase)
            {
                t.Info($"  t {sim,6:F1}  phase {lastPhase} -> {st.Phase} at Mach {st.Mach:F2}, {height / 1000:F1} km up, {ground / 1000:F1} km from the site");
                if (st.Phase == GuidanceWindow.OrbitalEntryPhase.Entry && double.IsNaN(engagedAt))
                    engagedAt = sim;
                if (st.Phase == GuidanceWindow.OrbitalEntryPhase.Bellyflop && double.IsNaN(takeOverMach))
                    (takeOverMach, takeOverAlt, takeOverRange) = (st.Mach, height, ground);
                lastPhase = st.Phase;
            }
            if (sim >= nextSample)
            {
                nextSample += SampleS;
                var bf = st.Bellyflop;
                t.Info($"  t {sim,6:F1}  {st.Phase,-9} Mach {st.Mach,5:F2} {height / 1000,6:F1} km up {ground / 1000,7:F1} km off ({st.CrossrangeM / 1000:F1} across)  " +
                       $"drag {st.DragMs2,5:F1} m/s2" +
                       (st.Phase == GuidanceWindow.OrbitalEntryPhase.Bellyflop
                           ? $"  pitch {bf.PitchDeg,5:F1} bank {bf.BankDeg,5:F1} yaw {bf.YawDeg,6:F1}  plan miss {bf.Plan?.MissM ?? double.NaN:F0} m  force x{bf.ForceScale:F2}"
                           : $"  bank {st.Last.BankDeg,6:F1}  aim fall {bf.TerminalRangeM / 1000:F1} km") +
                       (st.StatusWarn ? "  ! " + st.Status : ""));
            }
            if (double.IsNaN(engagedAt) && sim > EngageWaitS)
            {
                t.Fail("engage", $"orbital entry did not engage in {EngageWaitS:F0} s: {st.Status}");
                return;
            }
            if (!double.IsNaN(engagedAt) && st.Phase == GuidanceWindow.OrbitalEntryPhase.Idle)
            {
                t.Fail("flight", $"orbital entry ended at {sim:F0} s: {st.Status}");
                return;
            }
            if (height <= EndHeightM)
            {
                (endMiss, endSpeed, ended) = (ground, vSurface.Length(), true);
                break;
            }
        }
        while (GuidanceLines.TryDequeue(out string? line))
            t.Info($"  | {line}");

        t.Info($"peak aero load {peakLoadG:F2} g ({peakLoadBellyflopG:F2} g under bellyflop), dynamic pressure {peakQ / 1000:F1} kPa; " +
               $"heating {peakHeat / 1000:F0} kW/m2 at most, at Mach {peakHeatMach:F1} and {peakHeatAlt / 1000:F1} km; {sim:F0} s flown, {wall.Elapsed.TotalSeconds:F0} s real");
        t.Check("orbital entry engages", !double.IsNaN(engagedAt), state.OrbitalEntry.Status);
        // With a heating limit set, the flown peak must stay within 5 % of it (the game's density against the plan's, and the tracker's give).
        double heatLimit = state.OrbitalEntryHeatMaxKWm2;
        if (heatLimit > 0)
            t.Check($"the heating peak holds the {heatLimit:F0} kW/m2 limit", peakHeat / 1000 <= 1.05 * heatLimit,
                FormattableString.Invariant($"{peakHeat / 1000:F0} kW/m2 flown, {peakHeat / 1000 / heatLimit * 100:F0} % of the limit; the least this craft can fly {leastHeat:F0}"));
        // Bellyflop takes over at its first converged plan from BELLYFLOP FROM MACH; HAND-OVER MACH is the latest.
        double armMach = state.OrbitalEntryBellyflopArmMach;
        t.Check($"bellyflop takes over between Mach {armMach:F1} and 4", takeOverMach >= 4.0 && takeOverMach <= armMach + 0.2,
            double.IsNaN(takeOverMach) ? "it never took over" : FormattableString.Invariant($"Mach {takeOverMach:F2}, {takeOverAlt / 1000:F1} km up, {takeOverRange / 1000:F1} km from the site"));
        if (t.Check($"the flight reaches {EndHeightM:F0} m over the site", ended, $"stopped at {sim:F0} s"))
            t.Check($"within {missLimitM:F0} m of the site at {EndHeightM:F0} m up", endMiss <= missLimitM,
                FormattableString.Invariant($"{endMiss:F0} m off at {endSpeed:F0} m/s"));
    }

    private static MethodInfo Method(string name) =>
        typeof(GuidanceWindow).GetMethod(name, PrivateStatic)
        ?? throw new MissingMethodException(nameof(GuidanceWindow), name);
}

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

// Opt-in flight: A PLAYER'S GAME SAVE FLOWN TO THE GROUND AS THEY FLY IT. The save named by AFC_SCENARIO_SAVE (bellyflop, launchpad_entry, moon_des, ...) is loaded through the game's own load path (Universe.DeserializeSave: every craft rebuilt as saved, attitude and rates included, the sim clock restored), the craft's AFC settings are restored from its afc-craft-settings.toml, the craft the camera follows (or AFC_SCENARIO_CRAFT) is taken, and its landing is engaged as the player does: ORBITAL ENTRY for a craft falling through air (bellyflop or glide, then touchdown), EXECUTE for an airless one (moon_des). AFC_SCENARIO_MODE=entry|execute overrides.
//
// RUN WITH THE CONTENT THE SAVE USES: run-headless.ps1 -Tests afc-guidance-save-landing -Vehicle '' -ContentMods 'KSAero,KSAStarship' -EnvVars 'AFC_SCENARIO_SAVE=bellyflop'. The save is read from the harness profile's own saves folder.
//
// PACED AT REAL TIME (AFC_SCENARIO_PACE, 1) once touchdown guidance flies or stands by, and while bellyflop or the entry solves: they all solve against the wall clock. AFC_SCENARIO_FAST_PACE (20) times real time before that on an airless descent (the moon's deorbit coast).
//
// THE MEASURE is the campaign's pass bar (2026-11-27): at the first contact, under 2 m/s down, under 1 m/s sideways, within 3 m of the site, tilted under 3 deg, the engine not cut above 1 m and no climb in the last 30 m; for a craft that lands on a skirt (AFC_SCENARIO_STANDS=1, the Starship), only that it is still standing - under 5 deg - 8 s after contact. Every number is logged in one SCENARIO RESULT line for scripts to read.
public sealed class GuidanceSaveLandingTest : AfcTest
{
    public override string Name => "afc-guidance-save-landing";
    public override bool OptIn => true;

    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;
    /// <summary>The sim step, s: AFC_SCENARIO_STEP_S, 0.015 by default: the game steps once a frame, and its 2026-11-27 17:29 Starship touchdown ran at a median 15.5 ms (7-33).</summary>
    private static readonly double StepS = Math.Clamp(Env("AFC_SCENARIO_STEP_S", 0.015), 0.002, 0.1);
    private const double SettleS = 8.0;

    private static double Env(string name, double fallback)
        => double.TryParse(Environment.GetEnvironmentVariable(name), NumberStyles.Float, CultureInfo.InvariantCulture, out double v) ? v : fallback;

    private static string EnvText(string name, string fallback)
        => Environment.GetEnvironmentVariable(name) is { Length: > 0 } v ? v : fallback;

    private static readonly ConcurrentQueue<string> GuidanceLines = new();
    private static void QueueGuidanceLine(Vehicle vehicle, string message) => GuidanceLines.Enqueue(message);

    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> Inputs =
        AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

    protected override void Execute(TestContext t)
    {
        string saveName = EnvText("AFC_SCENARIO_SAVE", "bellyflop");
        string saveDir = Path.Combine(GameSaves.SaveFolderPath, saveName);
        string path = Path.Combine(saveDir, "universe.xml");
        if (!File.Exists(path))
        {
            t.Skip($"save '{saveName}' not found at {path}.");
            return;
        }
        if (VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state.");
            return;
        }

        FieldInfo ambient = typeof(GuidanceWindow).GetField("_s", PrivateStatic)!;
        object? previousAmbient = ambient.GetValue(null);
        bool previousModActive = GuidanceWindow.ModActive;
        bool previousEnabled = SharedVehicleHooks.GuidanceEnabled;
        using KsaeroScope aero = KsaeroScope.Apply();
        t.Info(aero.Status);
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.savelanding");
        VehicleCommandSink.ApplyPatches(harmony);
        GuidanceFeature.ApplyDriverPatches(harmony);
        harmony.Patch(AccessTools.Method(typeof(GuidanceLog), nameof(GuidanceLog.Info)),
            prefix: new HarmonyMethod(typeof(GuidanceSaveLandingTest), nameof(QueueGuidanceLine)));
        GuidanceLines.Clear();
        SharedVehicleHooks.GuidanceEnabled = true;
        GuidanceWindow.SetModActive(true);
        bool railsBefore = PhysicsBubble._forceOffRails;
        Vehicle? craft = null;
        try
        {
            string xml = File.ReadAllText(path);
            UniverseData data;
            using (var reader = new StringReader(xml))
                data = (UniverseData)GameSaves.UniverseSerializer.Deserialize(reader)!;
            data.OnDataLoad(KSA.Mod.Empty);
            string craftId = EnvText("AFC_SCENARIO_CRAFT", System.Text.RegularExpressions.Regex.Match(xml, "<Following Id=\"([^\"]+)\"").Groups[1].Value);
            Universe.DeserializeSave(data);
            CelestialSystem system = Universe.CurrentSystem ?? t.System;
            if (string.IsNullOrEmpty(craftId) || !system.All.TryGet(craftId, out Astronomical? found) || found is not Vehicle v)
            {
                t.Fail("load", $"craft '{craftId}' is not in save '{saveName}' after the load.");
                return;
            }
            craft = v;
            int restored = CraftSettings.RestoreFrom(new DirectoryInfo(saveDir), system);
            // THE OTHER CRAFT of the save are removed (AFC_SCENARIO_KEEP_OTHERS=1 keeps them): the bellyflop save's second Starship stands about 35 m from the site, and a run on 2026-11-27 (18:54) struck it 33 m up on a divert - a collision guidance cannot see, which says nothing about the touchdown.
            if (Env("AFC_SCENARIO_KEEP_OTHERS", 0.0) == 0.0)
            {
                var others = new List<Vehicle>();
                for (int i = 0; i < system.Count; i++)
                    if (system.GetIndex(i) is Vehicle other && !ReferenceEquals(other, craft) && !other.IsDisposed)
                        others.Add(other);
                foreach (Vehicle other in others)
                {
                    t.Info($"removed other craft '{other.Id}', {(other.Orbit.StateVectors.PositionCci - craft.Orbit.StateVectors.PositionCci).Length():F0} m from this one");
                    VehicleSpawner.Despawn(other);
                }
            }
            PhysicsBubble._forceOffRails = true;
            Program.ControlledVehicle = craft;
            IParentBody body = craft.Orbit.Parent;
            bool airless = !(body.GetAtmosphereRadius() > body.MeanRadius);
            string mode = EnvText("AFC_SCENARIO_MODE", airless ? "execute" : craft.Id.Contains("starship", StringComparison.OrdinalIgnoreCase) ? "entry" : "retro");
            SimDriver driver = t.Session.CreateDriver();
            driver.StepOrbits = true;
            driver.Step(StepS, 2);
            t.Info($"save '{saveName}': craft '{craft.Id}' over {((Astronomical)body).Id}, {craft.TotalMass:F0} kg, {(craft.Orbit.StateVectors.PositionCci.Length() - body.MeanRadius):F0} m above the mean radius; AFC settings restored for {restored} craft; mode {mode}");

            VehicleAutopilotState state = VehicleAutopilotState.For(craft);
            if (Environment.GetEnvironmentVariable("AFC_SCVX_BACKEND") is { Length: > 0 } backend)
                state.ScvxClarabel = !string.Equals(backend, "scs", StringComparison.OrdinalIgnoreCase);
            state.SixDofLogging = true;
            // Every published plan into plan.csv (the game's own logging writes one a second), for following the plans re-solve by re-solve.
            if (typeof(GuidanceWindow).Assembly.GetType("AdvancedFlightComputer.Features.Guidance.SixDofLog")?.GetField("PlanSnapshotInterval", BindingFlags.NonPublic | BindingFlags.Static) is { } snap)
                snap.SetValue(null, Env("AFC_PLAN_SNAPSHOT_S", 0.0));
            state.EntryLogging = true;
            foreach (string pair in EnvText("AFC_SCENARIO_SET", "").Split(';', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
            {
                string[] kv = pair.Split('=', 2);
                if (kv.Length == 2)
                {
                    int applied = CraftSettings.Apply(state, new Dictionary<string, string> { [kv[0].Trim()] = kv[1].Trim() });
                    t.Info($"setting {kv[0].Trim()} = {kv[1].Trim()}: {(applied == 1 ? "applied" : "NOT TAKEN")}");
                }
            }
            MoveSite(t, craft, state, body, ambient);
            t.Info($"SCvx backend {(state.ScvxClarabel ? "Clarabel" : "SCS")}; site {state.SiteLatDeg:F5} {state.SiteLonDeg:F5}; vehicle height {state.VehicleHeightM:F2} m");

            if (mode == "execute")
            {
                state.Engage = true;
                state.AutoStage = true;
                state.UseSixDofLanding = true;
                ambient.SetValue(null, state);
                Method("ExecuteLanding").Invoke(null, [craft]);
                ambient.SetValue(null, previousAmbient);
                t.Info($"EXECUTE: {state.LandingStatus}");
            }
            else if (mode == "retro")
            {
                // LAND > LAND NOW > GLIDE with RETRO BURN: entry guidance (the 3-DOF glide and landing burn), then touchdown guidance.
                state.Engage = true;
                state.AutoStage = true;
                state.UseThreeDofLanding = true;
                state.UseSixDofLanding = true;
                ambient.SetValue(null, state);
                Method("RequestEntryGuidance").Invoke(null, [craft]);
                ambient.SetValue(null, previousAmbient);
                t.Info("RETRO BURN (entry guidance) requested");
            }
            else
            {
                state.OrbitalEntryBellyflop = true;
                state.OrbitalEntryFlip = true;
                state.OrbitalEntry.StopPending = false;
                state.OrbitalEntry.EngagePending = true;
                state.OrbitalEntry.TrajWindowClosed = false;
                t.Info("ORBITAL ENTRY requested");
            }

            Fly(t, craft, state, driver, body, ambient, airless);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            GuidanceFeature.DisableDriver();
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.SetModActive(previousModActive);
            if (craft != null)
                VehicleAutopilotState.Remove(craft);
            ambient.SetValue(null, previousAmbient);
            PhysicsBubble._forceOffRails = railsBefore;
        }
    }

    private static void Fly(TestContext t, Vehicle craft, VehicleAutopilotState state, SimDriver driver, IParentBody body, FieldInfo ambient, bool airless)
    {
        double pace = Math.Max(Env("AFC_SCENARIO_PACE", 1.0), 0.05);
        double fastPace = airless ? Math.Max(Env("AFC_SCENARIO_FAST_PACE", 20.0), 1.0) : pace;
        double timeoutS = Env("AFC_SCENARIO_TIMEOUT_S", 2400.0);
        bool stands = Env("AFC_SCENARIO_STANDS", craft.Id.Contains("starship", StringComparison.OrdinalIgnoreCase) ? 1.0 : 0.0) != 0.0;
        int partsAtStart = craft.Parts.Parts.Length;

        var wall = Stopwatch.StartNew();
        double sim = 0.0, wallBase = 0.0, simBase = 0.0, curPace = fastPace, nextSample = 0.0;
        double touchdownAt = double.NaN, touchdownAlt = double.NaN, contactAt = double.NaN;
        double cutAt = double.NaN, cutHeight = double.NaN, maxClimbLow = 0.0, lastHeight = double.NaN;
        double cSink = double.NaN, cSide = double.NaN, cMiss = double.NaN, cTilt = double.NaN, cRate = double.NaN;
        double maxTiltAfter = 0.0, endTilt = double.NaN;
        bool engineWasOn = false, impact = false;
        double3 lastNose = default;
        double lastRate = 0.0, nextAfter = 0.0;
        string endReason = "timeout";
        GuidanceWindow.OrbitalEntryPhase lastPhase = GuidanceWindow.OrbitalEntryPhase.Idle;
        // BELLYFLOP AND HAND-OVER METRICS: the flop's angles step by step, the hand-over state, the ignition, the propellant.
        var flop = new List<(double T, double Alt, double Pitch, double Bank, double Yaw)>();
        double massFlopStart = double.NaN, massHandover = double.NaN, massContact = double.NaN;
        double hPitch = double.NaN, hBank = double.NaN, hYaw = double.NaN, hVh = double.NaN, hVz = double.NaN, hRange = double.NaN, hTilt = double.NaN;
        double ignAt = double.NaN, ignAlt = double.NaN, ignTilt = double.NaN, ignRange = double.NaN, ignVh = double.NaN;
        double maxVhBurn = 0.0, vhAt100 = double.NaN, rangeAt100 = double.NaN;

        while (sim < timeoutS)
        {
            try
            {
                driver.Step(StepS);
            }
            catch (NullReferenceException)
            {
                // KSA's impact effect has no renderer to spawn into headless.
                impact = true;
                endReason = "impact (exception)";
                break;
            }
            if (craft.IsDisposed)
            {
                impact = true;
                endReason = "craft destroyed";
                break;
            }
            sim += StepS;
            var st = state.OrbitalEntry;
            bool touchdownFlies = state.Active || state.EngagePending;
            bool realTime = touchdownFlies || state.SixDofStandby || st.Phase != GuidanceWindow.OrbitalEntryPhase.Idle || !airless;
            double want = realTime ? pace : fastPace;
            if (want != curPace)
                (curPace, simBase, wallBase) = (want, sim, wall.Elapsed.TotalSeconds);
            double ahead = (sim - simBase) / curPace - (wall.Elapsed.TotalSeconds - wallBase);
            if (ahead > 0.002)
                Thread.Sleep(TimeSpan.FromSeconds(ahead));
            while (GuidanceLines.TryDequeue(out string? line))
                if (!line.Contains("UPFG Burn") && !line.Contains("UPFG Prep"))
                    t.Info($"  t {sim,7:F2}  | {line}");

            object? stepAmbient = ambient.GetValue(null);
            ambient.SetValue(null, state);
            double3 up = (double3)Method("SiteDirCciAt").Invoke(null, [body, 0.0])!;
            double3 siteCci = up * (body.MeanRadius + (double)Method("SiteTerrainHeight").Invoke(null, [body])!);
            ambient.SetValue(null, stepAmbient);
            double3 r = craft.Orbit.StateVectors.PositionCci;
            double3 vSurface = craft.Orbit.StateVectors.VelocityCci - double3.Cross(body.GetAngularVelocityCci(), r);
            double3 offset = r - siteCci;
            double alt = double3.Dot(offset, up);
            double range = (offset - up * alt).Length();
            double vz = double3.Dot(vSurface, up);
            double vh = (vSurface - up * vz).Length();
            double height = alt - state.VehicleHeightM;
            double3 nose = double3.UnitX.Transform(KsaFrameBridge.BodyToCci(craft));
            double tilt = Math.Acos(Math.Clamp(double3.Dot(nose, up), -1.0, 1.0)) * 180.0 / Math.PI;
            double rate = sim > StepS ? Math.Acos(Math.Clamp(double3.Dot(nose, lastNose), -1.0, 1.0)) * 180.0 / Math.PI / StepS : 0.0;
            lastNose = nose;
            if (double.IsNaN(contactAt) && !craft.Situation.HasAnyContact())
                lastRate = rate;
            bool contact = craft.Situation.HasAnyContact();
            ManualControlInputs inputs = Inputs(craft);
            bool engineOn = inputs.EngineOn && inputs.EngineThrottle > 0.0f;
            if (st.Phase == GuidanceWindow.OrbitalEntryPhase.Bellyflop)
            {
                if (double.IsNaN(massFlopStart))
                    massFlopStart = craft.TotalMass;
                var bfs = st.Bellyflop;
                flop.Add((sim, alt, bfs.PitchDeg, bfs.BankDeg, bfs.YawDeg));
            }
            if (touchdownFlies && double.IsNaN(massHandover))
            {
                massHandover = craft.TotalMass;
                (hVh, hVz, hRange, hTilt) = (vh, vz, range, tilt);
                if (flop.Count > 0)
                    (hPitch, hBank, hYaw) = (flop[^1].Pitch, flop[^1].Bank, flop[^1].Yaw);
            }
            if (touchdownFlies && engineOn && double.IsNaN(ignAt))
                (ignAt, ignAlt, ignTilt, ignRange, ignVh) = (sim, alt, tilt, range, vh);
            if (!double.IsNaN(ignAt) && double.IsNaN(contactAt))
            {
                maxVhBurn = Math.Max(maxVhBurn, vh);
                if (double.IsNaN(vhAt100) && height < 100.0)
                    (vhAt100, rangeAt100) = (vh, range);
            }

            if (st.Phase != lastPhase)
            {
                t.Info($"  t {sim,7:F2}  phase {lastPhase} -> {st.Phase} at {alt:F0} m, {range:F0} m from the site, {vSurface.Length():F0} m/s");
                lastPhase = st.Phase;
            }
            if (touchdownFlies && double.IsNaN(touchdownAt))
            {
                (touchdownAt, touchdownAlt) = (sim, alt);
                t.Info($"  t {sim,7:F2}  TOUCHDOWN TAKES OVER at {alt:F0} m, {range:F0} m from the site, {vSurface.Length():F0} m/s");
            }
            if (double.IsNaN(contactAt))
            {
                if (touchdownFlies && engineOn)
                    engineWasOn = true;
                if (engineWasOn && !engineOn && double.IsNaN(cutAt))
                    (cutAt, cutHeight) = (sim, height);
                if (engineOn && !double.IsNaN(cutAt))
                    (cutAt, cutHeight) = (double.NaN, double.NaN);
                if (touchdownFlies && range < 300.0 && height < 30.0 && double.IsFinite(lastHeight) && vz > 0.0)
                    maxClimbLow = Math.Max(maxClimbLow, vz);
                lastHeight = height;
            }

            if (sim >= nextSample)
            {
                nextSample = sim + (touchdownFlies || state.SixDofStandby ? 0.5 : 5.0);
                string what = touchdownFlies ? state.Converging ? "6-DOF converging" : "6-DOF flying" : st.Phase != GuidanceWindow.OrbitalEntryPhase.Idle ? st.Phase.ToString() : state.ThreeDofPhase.ToString() != "Idle" ? $"3-DOF {state.ThreeDofPhase}" : $"landing {state.LandingPhase}";
                t.Info($"  t {sim,7:F2}  alt {alt,8:F1} m  range {range,7:F1} m  vz {vz,7:F2}  vh {vh,6:F2} m/s  tilt {tilt,5:F1} deg  " +
                       $"throttle {(engineOn ? inputs.EngineThrottle : 0f),4:P0}  {what}  [{Trim(touchdownFlies ? state.Error : st.Status)}]");
            }

            if (contact && double.IsNaN(contactAt) && sim > 1.0)
            {
                (contactAt, cSink, cSide, cMiss, cTilt, cRate) = (sim, -vz, vh, range, tilt, lastRate);
                massContact = craft.TotalMass;
                t.Info($"  t {sim,7:F2}  CONTACT at {-vz:F2} m/s down, {vh:F2} m/s sideways, {range:F2} m from the site, tilt {tilt:F2} deg, turning {lastRate:F1} deg/s the step before");
            }
            if (!double.IsNaN(contactAt))
            {
                // After contact, every 0.25 s: what the craft does once guidance lets go - a lander that touched at 0.3 deg and toppled over the next 8 s had to be pushed (2026-11-27).
                if (sim >= nextAfter)
                {
                    nextAfter = sim + 0.25;
                    var fc = craft.FlightComputer;
                    double3 w = craft.BodyRates;
                    t.Info($"  t {sim,7:F2}  after +{sim - contactAt,4:F2} s  tilt {tilt,5:F1} deg  rates {w.X,6:F3} {w.Y,6:F3} {w.Z,6:F3} rad/s  vz {vz,6:F2} vh {vh,5:F2}  contact {(contact ? 1 : 0)}  engine {(engineOn ? inputs.EngineThrottle : 0f),4:P0}  " +
                           $"fc {fc.AttitudeMode}/{fc.AttitudeTrackTarget} rcs {fc.RCSMode}  owner {(VehicleControlOwnership.Holds(craft, ControlClaimant.Guidance) ? "AFC" : "stock")}");
                }
                maxTiltAfter = Math.Max(maxTiltAfter, tilt);
                endTilt = tilt;
                if (sim - contactAt >= SettleS)
                {
                    endReason = "settled";
                    break;
                }
            }
            else if (r.Length() - siteCci.Length() < -5.0 && range < 5000.0)
            {
                endReason = "below the site with no contact";
                break;
            }
            if (double.IsNaN(touchdownAt) && sim > 60.0 && st.Phase == GuidanceWindow.OrbitalEntryPhase.Idle && !state.SixDofStandby && state.ThreeDofPhase.ToString() == "Idle" && !state.ThreeDofEngagePending && state.LandingPhase.ToString() is "Idle" or "Done" && !airless)
            {
                endReason = "nothing flies: " + Trim(st.Status);
                break;
            }
        }
        while (GuidanceLines.TryDequeue(out string? line))
            t.Info($"  | {line}");

        int partsAtEnd = craft.IsDisposed ? 0 : craft.Parts.Parts.Length;
        bool contacted = !double.IsNaN(contactAt);
        bool standing = contacted && !impact && endTilt < 5.0;
        string F(double x) => double.IsFinite(x) ? x.ToString("F2", CultureInfo.InvariantCulture) : "nan";
        t.Info($"SCENARIO RESULT save={EnvText("AFC_SCENARIO_SAVE", "bellyflop")} craft={craft.Id} end={endReason.Replace(' ', '_')} handover_alt={F(touchdownAlt)} contact={(contacted ? 1 : 0)} " +
               $"sink={F(cSink)} side={F(cSide)} miss={F(cMiss)} tilt={F(cTilt)} rate={F(cRate)} cut_height={F(cutHeight)} climb_low={F(maxClimbLow)} " +
               $"max_tilt_after={F(maxTiltAfter)} end_tilt={F(endTilt)} standing={(standing ? 1 : 0)} parts={partsAtStart}->{partsAtEnd} sim={F(sim)} wall={F(wall.Elapsed.TotalSeconds)}");

        t.Info("BELLYFLOP RESULT " + FlopSummary(flop, touchdownAt) + FormattableString.Invariant(
            $" hand_pitch={F(hPitch)} hand_bank={F(hBank)} hand_yaw={F(hYaw)} hand_vh={F(hVh)} hand_vz={F(hVz)} hand_range={F(hRange)} hand_tilt={F(hTilt)}") +
            FormattableString.Invariant($" coast_s={F(ignAt - touchdownAt)} ign_alt={F(ignAlt)} ign_tilt={F(ignTilt)} ign_range={F(ignRange)} ign_vh={F(ignVh)} burn_s={F(contactAt - ignAt)} max_vh_burn={F(maxVhBurn)} vh_at100={F(vhAt100)} range_at100={F(rangeAt100)}") +
            FormattableString.Invariant($" flop_kg={F(massFlopStart - massHandover)} touchdown_kg={F(massHandover - massContact)}"));
        t.Check("touchdown guidance took over", !double.IsNaN(touchdownAt));
        if (!t.Check("the craft came down on the ground", contacted, endReason))
            return;
        t.Check($"still standing {SettleS:F0} s after contact (under 5 deg)", standing, FormattableString.Invariant($"{endTilt:F1} deg, at most {maxTiltAfter:F1}"));
        if (stands)
            return;
        t.Check($"under 2 m/s down: {cSink:F2}", cSink < 2.0);
        t.Check($"under 1 m/s sideways: {cSide:F2}", cSide < 1.0);
        t.Check($"within 3 m of the site: {cMiss:F2}", cMiss < 3.0);
        t.Check($"tilted under 3 deg at contact: {cTilt:F2}", cTilt < 3.0);
        t.Check($"engine not cut above 1 m: {(double.IsNaN(cutHeight) ? "lit to contact" : cutHeight.ToString("F2", CultureInfo.InvariantCulture))}", !(cutHeight > 1.0));
        t.Check($"no climb in the last 30 m: {maxClimbLow:F2} m/s up at most", maxClimbLow < 0.3);
    }

    /// <summary>The flop's angles in one line: over the whole flop and its last 20 s before the hand-over, the largest and the mean magnitude of bank and yaw, the pitch's mean, the bank's reversals past 3 deg; the time and altitude it flew.</summary>
    private static string FlopSummary(List<(double T, double Alt, double Pitch, double Bank, double Yaw)> flop, double handoverAt)
    {
        string F(double x) => double.IsFinite(x) ? x.ToString("F2", CultureInfo.InvariantCulture) : "nan";
        if (flop.Count == 0)
            return "flop_s=0";
        double end = double.IsFinite(handoverAt) ? handoverAt : flop[^1].T;
        (double Max, double Mean) Stat(Func<(double T, double Alt, double Pitch, double Bank, double Yaw), double> f, double from)
        {
            double max = 0.0, sum = 0.0;
            int n = 0;
            foreach (var r in flop)
            {
                if (r.T < from) continue;
                double v = f(r);
                if (!double.IsFinite(v)) continue;
                max = Math.Max(max, Math.Abs(v));
                sum += Math.Abs(v);
                n++;
            }
            return (max, n > 0 ? sum / n : double.NaN);
        }
        var bank = Stat(r => r.Bank, double.NegativeInfinity);
        var yaw = Stat(r => r.Yaw, double.NegativeInfinity);
        var bank20 = Stat(r => r.Bank, end - 20.0);
        var yaw20 = Stat(r => r.Yaw, end - 20.0);
        double pitchSum = 0.0;
        int pn = 0;
        foreach (var r in flop)
            if (double.IsFinite(r.Pitch)) { pitchSum += r.Pitch; pn++; }
        int reversals = 0, sign = 0;
        foreach (var r in flop)
        {
            int sNow = r.Bank > 3.0 ? 1 : r.Bank < -3.0 ? -1 : 0;
            if (sNow != 0 && sign != 0 && sNow != sign) reversals++;
            if (sNow != 0) sign = sNow;
        }
        return FormattableString.Invariant(
            $"flop_s={F(flop[^1].T - flop[0].T)} flop_from={F(flop[0].Alt)} pitch_mean={F(pn > 0 ? pitchSum / pn : double.NaN)} bank_max={F(bank.Max)} bank_mean={F(bank.Mean)} bank_last20={F(bank20.Mean)} bank_rev={reversals} yaw_max={F(yaw.Max)} yaw_mean={F(yaw.Mean)} yaw_last20={F(yaw20.Mean)}");
    }

    /// <summary>
    /// AFC_SCENARIO_SITE_DOWN_M / AFC_SCENARIO_SITE_CROSS_M move the landing site that far along and across the craft's bearing to it (+ cross to the left), so one save flies a dispersed approach: crossrange for bellyflop to take out. The site's latitude and longitude come from a few least-squares steps on the site's own direction (SiteDirCciAt).
    /// </summary>
    private static void MoveSite(TestContext t, Vehicle craft, VehicleAutopilotState state, IParentBody body, FieldInfo ambient)
    {
        double down = Env("AFC_SCENARIO_SITE_DOWN_M", 0.0), cross = Env("AFC_SCENARIO_SITE_CROSS_M", 0.0);
        if (down == 0.0 && cross == 0.0)
            return;
        object? before = ambient.GetValue(null);
        ambient.SetValue(null, state);
        try
        {
            double radius = body.MeanRadius;
            double3 Site() => (double3)Method("SiteDirCciAt").Invoke(null, [body, 0.0])!;
            double3 up = Site();
            double3 r = craft.Orbit.StateVectors.PositionCci;
            double3 toSite = up * radius - r;
            double3 bearing = (toSite - up * double3.Dot(toSite, up)).NormalizeOrZero();
            double3 left = double3.Cross(up, bearing);
            double3 want = (up * radius + bearing * down + left * cross).NormalizeOrZero();
            double lat0 = state.SiteLatDeg, lon0 = state.SiteLonDeg;
            for (int it = 0; it < 3; it++)
            {
                double3 s0 = Site();
                double3 e = want - s0;
                const double h = 1e-4;
                state.SiteLatDeg += h;
                double3 dLat = (Site() - s0) / h;
                state.SiteLatDeg -= h;
                state.SiteLonDeg += h;
                double3 dLon = (Site() - s0) / h;
                state.SiteLonDeg -= h;
                double a11 = double3.Dot(dLat, dLat), a12 = double3.Dot(dLat, dLon), a22 = double3.Dot(dLon, dLon);
                double b1 = double3.Dot(dLat, e), b2 = double3.Dot(dLon, e), det = a11 * a22 - a12 * a12;
                if (!(Math.Abs(det) > 1e-30)) break;
                state.SiteLatDeg += (a22 * b1 - a12 * b2) / det;
                state.SiteLonDeg += (a11 * b2 - a12 * b1) / det;
            }
            double moved = (Site() - up).Length() * radius;
            t.Info(FormattableString.Invariant($"site moved {down:F0} m down the bearing and {cross:F0} m across it: {lat0:F5} {lon0:F5} -> {state.SiteLatDeg:F5} {state.SiteLonDeg:F5}, {moved:F0} m"));
        }
        finally
        {
            ambient.SetValue(null, before);
        }
    }

    private static string Trim(string s) => s.Length > 110 ? s[..110] : s;

    private static MethodInfo Method(string name) =>
        typeof(GuidanceWindow).GetMethod(name, PrivateStatic)
        ?? throw new MissingMethodException(nameof(GuidanceWindow), name);
}

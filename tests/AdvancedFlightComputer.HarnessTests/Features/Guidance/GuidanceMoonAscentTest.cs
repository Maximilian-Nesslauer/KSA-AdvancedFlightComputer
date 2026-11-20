using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// A solved climb from the Moon to a 30 km orbit, issue #167: the ascent never handed over to UPFG there, because UPFG FROM
// was 80 km and the orbit is reached far under it. Two flights of the moon_des hopper from the Apollo 11 site:
//  - with UPFG FROM at Luna's own default, which the craft takes from the body it is at (BodySettings), 10 km;
//  - with UPFG FROM set to 80 km by the player, which the ascent holds under 75 % of the target periapsis, 22.5 km.
// Each must hand over to UPFG at that altitude and reach an orbit.
public sealed class GuidanceMoonAscentTest : AfcTest
{
    public override string Name => "afc-guidance-moon-ascent";

    private const string SaveName = "moon_des";
    private const double SiteLatDeg = 0.67408, SiteLonDeg = 23.47297;
    private const double PadHeightM = 300.0, SpawnClimbMs = 10.0;
    private const double TargetKm = 30.0, TargetIncDeg = 2.0;
    private const double StepS = 0.1, TerminalStepS = 0.02, MaxFlightS = 900.0, PlanTimeoutS = 180.0;
    // Where the hand-over may land above the altitude it is set to: the one guidance cycle UPFG may still need to converge, climbing.
    private const double HandoverLateKm = 1.5;
    private const double OrbitTolKm = 10.0;

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(AccessTools.Field(typeof(GuidanceWindow), "_s"));
    private static readonly MethodInfo StartGuidance = AccessTools.Method(typeof(GuidanceWindow), "StartGuidance");
    private static readonly MethodInfo LanOverhead = AccessTools.Method(typeof(GuidanceWindow), "LanOverhead");

    protected override void Execute(TestContext t)
    {
        string path = Path.Combine(GameSaves.SaveFolderPath, SaveName, "universe.xml");
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
        BodyDefaults.Reset();

        VehicleAutopilotState previousAmbient = AmbientState();
        Vehicle? previousFocus = Program.ControlledVehicle;
        bool previousModActive = GuidanceWindow.ModActive;
        bool previousEnabled = SharedVehicleHooks.GuidanceEnabled;
        bool railsBefore = PhysicsBubble._forceOffRails;
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.moonascent");
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        try
        {
            VehicleCommandSink.ApplyPatches(harmony);
            GuidanceFeature.ApplyDriverPatches(harmony);
            SharedVehicleHooks.GuidanceEnabled = true;
            GuidanceWindow.SetModActive(true);
            driver.StepOrbits = true;
            driver.Step(0.02, 2);

            VehicleData data;
            using (var reader = new StreamReader(path))
            {
                var save = (UniverseData)GameSaves.UniverseSerializer.Deserialize(reader)!;
                save.OnDataLoad(KSA.Mod.Empty);
                data = save.CelestialSystems.SelectMany(system => system.Vehicles).Single(vehicle => vehicle.Id == "hopper");
            }

            double lunaDefault = BodyDefaults.DefaultOf(GuidanceSettingSpecs.ConvexHandoverAltKm, "Luna");
            Flight(t, "Luna default", null, lunaDefault, data, lunaCelestial, luna, driver, preexisting);
            Flight(t, "UPFG FROM 80 km", 80.0, AscentHandover.PeShare * TargetKm, data, lunaCelestial, luna, driver, preexisting);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            GuidanceFeature.DisableDriver();
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.SetModActive(previousModActive);
            Program.ControlledVehicle = previousFocus;
            AmbientState() = previousAmbient;
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void Flight(TestContext t, string name, double? upfgFromKm, double expectKm, VehicleData data,
                               Celestial body, IParentBody luna, SimDriver driver, HashSet<string> preexisting)
    {
        t.Info($"--- {name}: expect the hand-over at {expectKm:F1} km");
        double lat = SiteLatDeg * Math.PI / 180.0, lon = SiteLonDeg * Math.PI / 180.0;
        var dirCcf = new double3(Math.Cos(lat) * Math.Cos(lon), Math.Cos(lat) * Math.Sin(lon), Math.Sin(lat));
        double terrain = body.GetTerrainHeightFromDirCcf(dirCcf);
        if (!double.IsFinite(terrain))
            terrain = 0.0;
        double3 dirCci = double3.Normalize(dirCcf.Transform(luna.GetCcf2Cci()));
        double3 r = dirCci * (luna.MeanRadius + terrain + PadHeightM);
        double3 v = double3.Cross(new double3(0, 0, luna.GetAngularVelocity()), r) + SpawnClimbMs * dirCci;
        Orbit pad = Orbit.CreateFromStateCci(luna, Universe.GetElapsedTime(), r, v, VehicleSpawner.OrbitLineColor);
        Vehicle craft = VehicleFixtures.SpawnFromSaveData(t.System, luna, data, "HarnessMoonAscent", pad);
        try
        {
            PhysicsBubble._forceOffRails = true;
            Program.ControlledVehicle = craft;
            driver.Step(0.02, 2);
            t.Info($"hopper '{craft.Id}': {craft.TotalMass:F0} kg, {terrain + PadHeightM:F0} m above the mean radius");

            VehicleAutopilotState state = VehicleAutopilotState.For(craft);
            BodySettings.Follow(state, craft);
            t.Check($"{name}: the craft follows Luna's defaults", state.DefaultsBody == "Luna" && state.ConvexHandoverAltKm == BodyDefaults.DefaultOf(GuidanceSettingSpecs.ConvexHandoverAltKm, "Luna"),
                $"body '{state.DefaultsBody}', UPFG FROM {state.ConvexHandoverAltKm} km");
            if (upfgFromKm is double set)
                state.ConvexHandoverAltKm = set;
            state.PeKm = TargetKm;
            state.ApKm = TargetKm;
            state.IncDeg = TargetIncDeg;
            state.LanDeg = (double)LanOverhead.Invoke(null, new object[] { craft.Orbit.StateVectors.PositionCci, TargetIncDeg, luna })!;
            state.LanSeeded = true;
            state.ArgPeFixed = false;
            state.TargetId = "";
            state.Engage = true;
            state.AutoStage = true;
            state.ReserveArmed = false;
            state.FlyConvexAscent = true;
            t.CheckAbs($"{name}: the hand-over altitude", state.HandoverAltKm, expectKm, 1e-9);
            AmbientState() = state;

            AscentPlanJob? job = AscentPlanRequests.Start(driver, state);
            if (!t.Check($"{name}: Calculate starts a solve", job != null, $"status '{state.AscentPlanStatus}'"))
                return;
            var clock = System.Diagnostics.Stopwatch.StartNew();
            while (!job!.IsDone && clock.Elapsed.TotalSeconds < PlanTimeoutS)
                Thread.Sleep(50);
            driver.Step(0.02);
            if (!t.Check($"{name}: the plan converged", state.AscentPlan?.Usable == true, $"status '{state.AscentPlanStatus}', {job.Progress}"))
            {
                job.Cancel();
                return;
            }
            t.Info($"plan: {state.AscentPlan!.Solution.Message}, {state.AscentPlan.Solution.FinalMass:F0} kg to orbit");

            StartGuidance.Invoke(null, new object[] { craft, craft.Orbit, luna });
            if (!t.Check($"{name}: EXECUTE flies the solved climb", state.Running && state.Phase == GuidanceWindow.AscentPhase.Profile,
                    $"running {state.Running}, phase {state.Phase}, status '{state.Status}'"))
                return;

            double time = 0.0, handoverKm = double.NaN, topKm = 0.0, nextLog = 0.0;
            GuidanceWindow.AscentPhase last = state.Phase;
            while (time < MaxFlightS && state.Running && !craft.IsDisposed)
            {
                double dt = state.Phase == GuidanceWindow.AscentPhase.Terminal ? TerminalStepS : StepS;
                try
                {
                    driver.Step(dt);
                }
                catch (NullReferenceException) when (time < 60.0)
                {
                    // The game's ground-impact effect throws headless.
                    t.Fail($"{name}: the climb leaves the ground", $"hit the ground at t {time:F1} s");
                    return;
                }
                time += dt;
                double3 rNow = craft.Orbit.StateVectors.PositionCci;
                double altKm = (rNow.Length() - luna.MeanRadius) / 1000.0;
                double climbMs = double3.Dot(craft.Orbit.StateVectors.VelocityCci, double3.Normalize(rNow));
                topKm = Math.Max(topKm, altKm);
                if (state.Phase != last)
                {
                    t.Info($"  t {time,6:F1} s  {last} -> {state.Phase} at {altKm:F2} km, UPFG converged {state.Upfg.Converged}");
                    if (last == GuidanceWindow.AscentPhase.Profile && double.IsNaN(handoverKm))
                        handoverKm = altKm;
                    last = state.Phase;
                }
                if (time >= nextLog)
                {
                    t.Info($"  t {time,6:F1} s  {altKm,6:F2} km  climb {climbMs,6:F1} m/s  {state.Phase}  {craft.TotalMass:F0} kg");
                    nextLog += time < 10.0 ? 1.0 : 30.0;
                }
            }

            t.Check($"{name}: the climb hands over to UPFG", double.IsFinite(handoverKm), $"phase {state.Phase}, highest {topKm:F1} km, status '{state.Status}'");
            if (double.IsFinite(handoverKm))
                t.Check($"{name}: the hand-over is at {expectKm:F1} km", handoverKm >= expectKm - 0.05 && handoverKm <= expectKm + HandoverLateKm, $"at {handoverKm:F2} km");
            double peKm = (craft.Orbit.Periapsis - luna.MeanRadius) / 1000.0, apKm = (craft.Orbit.Apoapsis - luna.MeanRadius) / 1000.0;
            t.Info($"{name}: ended after {time:F0} s, phase {state.Phase}, status '{state.Status}', orbit {peKm:F1} x {apKm:F1} km");
            t.Check($"{name}: the ascent reaches its orbit", state.CutoffDone && Math.Abs(peKm - TargetKm) <= OrbitTolKm && Math.Abs(apKm - TargetKm) <= OrbitTolKm,
                $"cutoff {state.CutoffDone}, {peKm:F1} x {apKm:F1} km");
        }
        finally
        {
            VehicleAutopilotState.Remove(craft);
            if (!craft.IsDisposed)
                VehicleSpawner.Despawn(craft);
        }
    }
}

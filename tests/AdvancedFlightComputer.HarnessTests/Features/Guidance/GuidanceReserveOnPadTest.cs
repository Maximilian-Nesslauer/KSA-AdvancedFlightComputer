using System.Text.Json;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The booster reserve on the pad, before anything has lit.
//
// The convex ascent is planned on the pad, before EXECUTE launches, and keeps the reserve out of the first
// liquid load only if the reserve is armed by then. The reserve arms when the next staging row drops every
// burning engine (NextSeparationDropsAllEngines), and on the pad the next row is usually the ignition, which
// drops nothing: so it stayed unarmed until the engines lit, the plan was solved for the whole first stage, and
// the reserve then staged the booster early in the climb, leaving the vehicle below the plan all the way to the
// hand-over (2stage_controlled, 2026-11-04: 30 deg planned pitch at 80 km against UPFG's 51).
//
// There is no reserve to set: a landing target on the booster arms it, and the convex plan sizes it from nothing (#84).
// On 2stage_controlled, whose first row lights the first stage's engines and whose second drops them with the
// booster: with no target the reserve stays off; with one it arms on the pad, the convex problem brings the booster
// home starting from no reserve, the plan keeps what the booster needs, and the armed reserve is the plan's until
// the target moves. Skipped where that save is not installed.
public sealed class GuidanceReserveOnPadTest : AfcTest
{
    public override string Name => "afc-guidance-reserve-on-pad";

    private const string Save = "2stage_controlled";
    private const double SiteLatDeg = 28.5;
    private const double SiteLonDeg = 0.0;
    private const double PadHeightM = 300.0;
    private const double SpawnClimbMs = 15.0;

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(AccessTools.Field(typeof(GuidanceWindow), "_s"));

    private static readonly System.Reflection.MethodInfo RefreshStageModel =
        AccessTools.Method(typeof(GuidanceWindow), "RefreshStageModel");

    private static readonly System.Reflection.MethodInfo LanOverhead =
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
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.reserve-on-pad");
        VehicleAutopilotState? state = null;
        try
        {
            VehicleCommandSink.ApplyPatches(harmony);
            GuidanceFeature.ApplyDriverPatches(harmony);
            SharedVehicleHooks.GuidanceEnabled = true;
            GuidanceWindow.SetModActive(true);
            // The bodies turn, and a step first: until the orbit solvers have run once the body's rotation is not the universe clock's, and the booster's site, read through it, would be an hour of spin from the pad.
            driver.StepOrbits = true;
            driver.Step(0.02, 2);

            // The pad, as the convex ascent test makes it: over the site, a few hundred metres up, turning with the ground.
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
                vehicle = VehicleSpawner.SpawnFromSave(Save, t.System, home, "HarnessReserveOnPad", pad);
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"{Save}: {e.Message}");
                return;
            }
            Program.ControlledVehicle = vehicle;
            PhysicsBubble._forceOffRails = true;
            driver.Step(0.05, 6);

            state = VehicleAutopilotState.For(vehicle);
            state.PeKm = 200.0;
            state.ApKm = 200.0;
            state.IncDeg = 51.6;
            state.LanDeg = (double)LanOverhead.Invoke(null, new object[] { vehicle.Orbit.StateVectors.PositionCci, 51.6, home })!;
            state.LanSeeded = true;
            state.TargetId = "";
            state.FlyConvexAscent = true;
            // At the 80 % default this spawned-in-the-air climb stalls with the linear model promising 309 kg it never delivers (the plan is 10.39 t against 10.42 t at 90 %), over the 0.1 % a stall may leave: a throttle question, not the reserve's, so the floor is held where this test was written.
            state.ConvexThrottleMinPct = 90.0;
            AmbientState() = state;

            bool unlit = true;
            foreach (Sequence s in vehicle.Parts.SequenceList.Sequences)
                if (s.Activated)
                    unlit = false;
            t.Check("on the pad, no staging row has fired", unlit);

            RefreshStageModel.Invoke(null, new object[] { vehicle });
            t.Info($"stage model: {GuidanceLog.DescribeStages(state.StageModel)}");
            t.Check("with no booster target the reserve is off", !state.ReserveArmed && state.ReserveKg == 0.0, $"note '{state.ReserveNote}'");
            // The service module leaves its upper stage in orbit: a command pod, but going orbital speed, so not listed.
            t.Check("only the booster can be targeted, not the stage that separates in orbit", state.ReturnableStages.Count == 1, $"{state.ReturnableStages.Count} listed");
            GuidanceWindow.ReturnableStage? booster = state.ReturnableStages.Find(st => st.IsNext);
            if (!t.Check("the booster is the next returnable stage", booster != null))
                return;

            // The booster comes home to the pad it lifts off from: Set target on its row.
            state.StageTargets[booster!.RootId] = new double2(SiteLatDeg, SiteLonDeg);
            state.StageModelDirty = true;
            RefreshStageModel.Invoke(null, new object[] { vehicle });
            t.Info($"reserve {(state.ReserveArmed ? "armed" : "not armed")}: {state.ReserveKg:F0} kg, booster {state.ReserveBoosterDryKg:F0} kg dry, note '{state.ReserveNote}'");
            t.Check("a booster target arms the reserve on the pad", state.ReserveArmed, $"note '{state.ReserveNote}'");
            t.Check("keeping nothing until the plan sizes it", state.ReserveKg == 0.0, $"{state.ReserveKg:F0} kg");

            // The convex problem the sim step builds from it: the first stage's whole load, and the booster's reserve starting from none.
            UpfgStage first = state.StageModel.Stages[0];
            double load = first.MassTotal - first.MassDry;
            string dump = Path.Combine(Path.GetTempPath(), "afc-convex-ascent", "HarnessReserveOnPad.json");
            if (File.Exists(dump))
                File.Delete(dump);
            state.AscentPlanRequested = true;
            driver.Step(0.02);
            AscentPlanJob? job = state.AscentPlanJob;
            if (!t.Check("Calculate wrote the problem", File.Exists(dump), $"status '{state.AscentPlanStatus}'"))
                return;
            using JsonDocument doc = JsonDocument.Parse(File.ReadAllText(dump));
            double planned = doc.RootElement.GetProperty("Stages")[0].GetProperty("PropellantMass").GetDouble();
            t.Info($"first stage: {load:F0} kg of propellant, {planned:F0} kg in the problem");
            t.Check("the problem starts from the first stage's whole load", Math.Abs(load - planned) < 0.02 * load, $"{planned:F0} kg against {load:F0} kg");

            // THE PLAN SIZES IT (#84): the problem carries the booster, and the plan's reserve is what it needs from where the plan separates it - which the armed reserve then takes.
            bool hasRecovery = doc.RootElement.TryGetProperty("Recovery", out JsonElement rec) && rec.ValueKind == JsonValueKind.Object;
            if (!t.Check("the problem brings the booster home", hasRecovery))
            {
                job?.Cancel();
                return;
            }
            t.Check("its reserve starting from none", rec.GetProperty("InitialReserveKg").GetDouble() == 0.0, $"{rec.GetProperty("InitialReserveKg").GetDouble():F0} kg");
            double dry = rec.GetProperty("BoosterDryKg").GetDouble();
            t.Check("at the booster's dry mass", Math.Abs(dry - state.ReserveBoosterDryKg) < 0.02 * state.ReserveBoosterDryKg,
                $"{dry:F0} kg against {state.ReserveBoosterDryKg:F0} kg");
            var clock = System.Diagnostics.Stopwatch.StartNew();
            while (job != null && !job.IsDone && clock.Elapsed.TotalSeconds < 240.0)
                Thread.Sleep(50);
            if (!t.Check("the solve finishes", job?.IsDone == true, job?.Progress ?? "no job"))
            {
                job?.Cancel();
                return;
            }
            driver.Step(0.02);
            AscentPlan? plan = state.AscentPlan;
            if (!t.Check("the plan is usable", plan?.Usable == true, $"status '{state.AscentPlanStatus}'"))
                return;
            AscentSolution sol = plan!.Solution;
            t.Info($"plan: {sol.Message}, {sol.FinalMass / 1000.0:F2} t to orbit; booster keeps {sol.RecoveryReserveKg:F0} kg against a need of {sol.RecoveryRequiredKg:F0} kg "
                 + $"({sol.BoostbackDv:F0} m/s boostback, {sol.LandingDv:F0} m/s landing), separates at T+{sol.SeparationTime:F0} s");
            t.Check("the booster's need is met", double.IsFinite(sol.RecoveryReserveKg) && sol.RecoveryReserveKg >= sol.RecoveryRequiredKg - 50.0,
                $"{sol.RecoveryReserveKg:F0} kg against {sol.RecoveryRequiredKg:F0} kg");
            // The refresh is gated to 4 Hz on the wall clock; the plan landing does not mark it, so mark it here, as a staging would.
            state.StageModelDirty = true;
            RefreshStageModel.Invoke(null, new object[] { vehicle });
            t.Check("the armed reserve is the plan's", state.ReserveArmed && Math.Abs(state.ReserveKg - sol.RecoveryReserveKg) < 1.0,
                $"{state.ReserveKg:F0} kg against {sol.RecoveryReserveKg:F0} kg");

            // A target moved after planning: that plan was for somewhere else, so the reserve keeps nothing until it is planned again.
            state.StageTargets[booster.RootId] = new double2(SiteLatDeg + 1.0, SiteLonDeg);
            state.StageModelDirty = true;
            RefreshStageModel.Invoke(null, new object[] { vehicle });
            t.Check("a moved target drops the plan's reserve", state.ReserveArmed && state.ReserveKg == 0.0, $"{state.ReserveKg:F0} kg, note '{state.ReserveNote}'");
        }
        finally
        {
            state?.AscentPlanJob?.Cancel();
            harmony.UnpatchAll(harmony.Id);
            GuidanceFeature.DisableDriver();
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.SetModActive(previousModActive);
            AmbientState() = previousAmbient;
            AutoStageFlightSupport.CleanupAfterFlight(t.System, preexisting);
            Program.ControlledVehicle = previousFocus;
        }
    }
}

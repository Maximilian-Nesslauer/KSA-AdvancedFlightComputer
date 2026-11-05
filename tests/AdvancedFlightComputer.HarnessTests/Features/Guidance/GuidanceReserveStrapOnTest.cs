using System.Text.Json;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The booster reserve for a core under strap-ons (#84).
//
// On boosters_controlled the strap-on solids separate first and carry no command pod; the core behind them is the
// booster that flies home. Its target used to be ignored: the reserve only looked at the NEXT returnable stage, and
// the strap-ons' separation ahead of it meant the core was never next on the pad, so the plan never sized its reserve
// (2026-11-05: "booster reserve off (no booster target set)" with a target on the core).
//
// With a target on the core: the reserve waits, unarmed, for the strap-ons; the convex problem still brings the core
// home, in the stage that drops its liquid load whole; and the plan keeps what the core needs. Skipped where that save
// is not installed.
public sealed class GuidanceReserveStrapOnTest : AfcTest
{
    public override string Name => "afc-guidance-reserve-strap-on";

    private const string Save = "boosters_controlled";
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
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.reserve-strap-on");
        VehicleAutopilotState? state = null;
        try
        {
            VehicleCommandSink.ApplyPatches(harmony);
            GuidanceFeature.ApplyDriverPatches(harmony);
            SharedVehicleHooks.GuidanceEnabled = true;
            GuidanceWindow.SetModActive(true);
            // The bodies turn, and a step first, as in afc-guidance-reserve-on-pad.
            driver.StepOrbits = true;
            driver.Step(0.02, 2);

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
                vehicle = VehicleSpawner.SpawnFromSave(Save, t.System, home, "HarnessReserveStrapOn", pad);
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
            AmbientState() = state;

            RefreshStageModel.Invoke(null, new object[] { vehicle });
            t.Info($"stage model: {GuidanceLog.DescribeStages(state.StageModel)}");
            foreach (GuidanceWindow.ReturnableStage st in state.ReturnableStages)
                t.Info($"returnable: '{st.Name}', sequence {st.SequenceIndex}{(st.IsNext ? ", next" : "")}");
            GuidanceWindow.ReturnableStage? core = state.ReturnableStages.Count > 0 ? state.ReturnableStages[0] : null;
            if (!t.Check("the core can fly home", core != null, $"{state.ReturnableStages.Count} listed"))
                return;
            if (core!.IsNext)
            {
                t.Skip($"{Save}: nothing separates ahead of '{core.Name}', so this is not the strap-on case.");
                return;
            }

            // Set target on the core, though the strap-ons go first.
            state.StageTargets[core.RootId] = new double2(SiteLatDeg, SiteLonDeg);
            state.StageModelDirty = true;
            RefreshStageModel.Invoke(null, new object[] { vehicle });
            t.Info($"reserve {(state.ReserveArmed ? "armed" : "not armed")}: {state.ReserveKg:F0} kg, note '{state.ReserveNote}'");
            t.Check("the core's target is the booster target", ReferenceEquals(GuidanceWindow.BoostbackTarget(), core));
            t.Check("the reserve waits for the strap-ons", !state.ReserveArmed && state.ReserveKg == 0.0, $"note '{state.ReserveNote}'");

            string dump = Path.Combine(Path.GetTempPath(), "afc-convex-ascent", "HarnessReserveStrapOn.json");
            if (File.Exists(dump))
                File.Delete(dump);
            state.AscentPlanRequested = true;
            driver.Step(0.02);
            AscentPlanJob? job = state.AscentPlanJob;
            if (!t.Check("Calculate wrote the problem", File.Exists(dump), $"status '{state.AscentPlanStatus}'"))
                return;
            using JsonDocument doc = JsonDocument.Parse(File.ReadAllText(dump));
            bool hasRecovery = doc.RootElement.TryGetProperty("Recovery", out JsonElement rec) && rec.ValueKind == JsonValueKind.Object;
            if (!t.Check("the problem brings the core home", hasRecovery))
            {
                job?.Cancel();
                return;
            }
            int stage = rec.GetProperty("Stage").GetInt32();
            JsonElement stages = doc.RootElement.GetProperty("Stages");
            t.Info($"reserve kept in stage {stage + 1} of {stages.GetArrayLength()}, {rec.GetProperty("BoosterDryKg").GetDouble():F0} kg dry, from {rec.GetProperty("InitialReserveKg").GetDouble():F0} kg");
            t.Check("in the core's stage, after the strap-ons", stage >= 1, $"stage {stage + 1}");
            t.Check("its reserve starting from none", rec.GetProperty("InitialReserveKg").GetDouble() == 0.0);

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
            t.Info($"plan: {sol.Message}, {sol.FinalMass / 1000.0:F2} t to orbit; core keeps {sol.RecoveryReserveKg:F0} kg against a need of {sol.RecoveryRequiredKg:F0} kg "
                 + $"({sol.BoostbackDv:F0} m/s boostback, {sol.LandingDv:F0} m/s landing), separates at T+{sol.SeparationTime:F0} s");
            t.Check("the core's need is met", double.IsFinite(sol.RecoveryReserveKg) && sol.RecoveryReserveKg >= sol.RecoveryRequiredKg - 50.0,
                $"{sol.RecoveryReserveKg:F0} kg against {sol.RecoveryRequiredKg:F0} kg");
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

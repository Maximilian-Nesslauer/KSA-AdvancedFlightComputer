using System.Diagnostics;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class GuidanceDeorbitRefusalTest : AfcTest
{
    public override string Name => "afc-guidance-deorbit-refusal";

    protected override void Execute(TestContext t)
    {
        using DeorbitFixture? fixture = DeorbitFixture.Open(t);
        if (fixture == null) return;
        DeorbitRequest request = fixture.Request;
        CheckArrivalAngles(t, request.Settings);
        DirectBrakingPlan? direct = DeorbitPlanner.FindDirect(request);
        if (!t.Check("the saved orbit has an approaching braking point", direct != null)) return;
        StateVectors ignition = request.Source.GetStateVectorsAt(new UniverseTime(direct!.IgnitionTime));
        t.Check("the high approach is refused", direct.Refusal.Length > 0, direct.Refusal);
        t.CheckAbs("ignition position comes from stock propagation", (direct.Position - ignition.PositionCci).Length(), 0, 0.01);
        t.CheckAbs("ignition velocity comes from stock propagation", (direct.Velocity - ignition.VelocityCci).Length(), 0, 0.001);
        t.Check("the refusal is for the high orbit", ignition.PositionCci.Length() - request.Parent.MeanRadius > 200_000);
        t.Check("a planning refusal takes no control", VehicleControlOwnership.HolderOf(fixture.Vehicle) != ControlClaimant.Guidance);
        t.Check("collinear Lambert geometry is refused", !DeorbitPlanner.TryLambert(request.Parent.Mu,
            ignition.PositionCci, ignition.PositionCci * 0.9, 1000, request.Source.GetOrbitNormalCci(), out _, out _));
        t.Check("a negative flight time is refused", !DeorbitPlanner.TryLambert(request.Parent.Mu,
            ignition.PositionCci, request.SiteAt(request.Epoch) * request.BrakingRadius, -1,
            request.Source.GetOrbitNormalCci(), out _, out _));
        CheckLambertDirections(t, request);
        CheckFlownArc(t, request);
        CheckLowApproach(t, request);
        CheckRuntimeRefusal(t, fixture);
    }

    private static void CheckLambertDirections(TestContext t, DeorbitRequest request)
    {
        double radius = request.Source.StateVectors.PositionCci.Length();
        double3 normal = new double3(1, 2, 3).Normalized();
        double3 start = double3.Cross(normal, new double3(0, 0, 1)).Normalized() * radius;
        double3 end = GuidanceWindow.RotateAbout(start, normal, Math.PI / 2);
        double flight = Math.PI * Math.Sqrt(radius * radius * radius / request.Parent.Mu);
        bool prograde = DeorbitPlanner.TryLambert(request.Parent.Mu, start, end, flight, normal,
            out double3 progradeVelocity, out _);
        bool retrograde = DeorbitPlanner.TryLambert(request.Parent.Mu, start, end, flight, -normal,
            out double3 retrogradeVelocity, out _);
        t.Check("Lambert supports both directions in the same inclined plane",
            prograde && retrograde
            && double3.Dot(double3.Cross(start, progradeVelocity), normal) > 0
            && double3.Dot(double3.Cross(start, retrogradeVelocity), normal) < 0);
    }

    private static void CheckArrivalAngles(TestContext t, DeorbitSettings automatic)
    {
        var selected = automatic with { ArrivalDescentDeg = 5 };
        t.Check("a selected angle accepts its one-degree tolerance", Matches(selected, 4) && Matches(selected, 5) && Matches(selected, 6));
        t.Check("a selected angle rejects shallower and steeper arrivals", !Matches(selected, 3.9) && !Matches(selected, 6.1));
        t.Check("a zero-degree target has the full stated tolerance", Matches(automatic with { ArrivalDescentDeg = 0 }, -1)
            && Matches(automatic with { ArrivalDescentDeg = 0 }, 1) && !Matches(automatic with { ArrivalDescentDeg = 0 }, -1.1));
        t.Check("automatic retains the original shallow range", Matches(automatic, 10) && Matches(automatic, -0.4)
            && !Matches(automatic, 10.1) && !Matches(automatic, -0.6));
        t.Check("a selected steep angle can exceed the automatic limit", Matches(automatic with { ArrivalDescentDeg = 20 }, 20));
        t.Check("invalid angles are refused", !(automatic with { ArrivalDescentDeg = double.NaN }).IsValid
            && !(automatic with { ArrivalDescentDeg = double.PositiveInfinity }).IsValid
            && !(automatic with { ArrivalDescentDeg = -1 }).IsValid
            && !(automatic with { ArrivalDescentDeg = 31 }).IsValid);
        t.Check("an arrival without velocity is refused", !DeorbitPlanner.MatchesArrivalAngle(selected, new double3(1, 0, 0), default));

        static bool Matches(DeorbitSettings settings, double downDeg)
        {
            double radians = downDeg * Math.PI / 180;
            return DeorbitPlanner.MatchesArrivalAngle(settings, new double3(1, 0, 0),
                new double3(-Math.Sin(radians), Math.Cos(radians), 0));
        }
    }

    private static void CheckFlownArc(TestContext t, DeorbitRequest request)
    {
        IParentBody parent = request.Parent;
        Orbit ellipse = Orbit.CreateFromStateCci(parent, new UniverseTime(request.Epoch),
            new double3(parent.MeanRadius + 500_000, 0, 0), new double3(0, 1400, 0), request.Source.OrbitLineColor);
        double periapsis = ellipse.TimeAtPeriapsis.Seconds();
        while (periapsis < request.Epoch) periapsis += ellipse.Period;
        StateVectors peri = ellipse.GetStateVectorsAt(new UniverseTime(periapsis));
        if (!t.Check("the negative fixture intersects the body", peri.PositionCci.Length() < parent.MeanRadius)) return;
        var crossing = new DeorbitArcCheck(request, ellipse, request.Epoch, request.Epoch + ellipse.Period);
        while (crossing.Advance()) { }
        t.Check("a surface crossing is refused before a clear endpoint", crossing.Refusal.Length > 0, crossing.Refusal);
        var shortArc = new DeorbitArcCheck(request, ellipse, request.Epoch, request.Epoch + 10);
        while (shortArc.Advance()) { }
        t.Check("a later surface crossing does not reject a clear flown interval", shortArc.Refusal.Length == 0, shortArc.Refusal);
    }

    private static void CheckLowApproach(TestContext t, DeorbitRequest request)
    {
        double periapsisTime = request.Epoch + 120;
        Orbit low = OrbitFixtures.EllipticalAt(request.Parent, 11000, 215000, new UniverseTime(periapsisTime));
        double3 site = low.GetStateVectorsAt(new UniverseTime(periapsisTime + 40)).PositionCci.Normalized()
            .Transform(request.Parent.GetCci2Ccf(new UniverseTime(periapsisTime)));
        double3 lla = request.Parent.GetLlaFromCcf(site * request.Parent.MeanRadius);
        var settings = request.Settings with { Latitude = lla.X, Longitude = lla.Y };
        var approach = new DeorbitRequest(low, request.Epoch, request.Mass, request.VehicleRadius,
            settings, request.Model, request.Engines, request.MinimumPulse, request.ControlStep);
        using var planner = new DeorbitPlanner(approach);
        var elapsed = Stopwatch.StartNew();
        while (!planner.Complete && elapsed.Elapsed.TotalSeconds < 30) planner.Step(3);
        t.Check("the full low-periapsis plan keeps direct braking after its terrain check",
            planner.Complete && planner.Direct is { Refusal.Length: 0 } && planner.Plan == null, planner.Status);
    }

    private static void CheckRuntimeRefusal(TestContext t, DeorbitFixture fixture)
    {
        Vehicle craft = fixture.Vehicle;
        DeorbitRequest saved = fixture.Request;
        double now = Universe.GetElapsedTime().Seconds();
        double3 site = saved.SiteAt(saved.Epoch).Transform(saved.Parent.GetCci2Ccf(new UniverseTime(now)));
        double3 lla = saved.Parent.GetLlaFromCcf(site * saved.Parent.MeanRadius);
        var settings = saved.Settings with { Latitude = lla.X, Longitude = lla.Y };
        var request = new DeorbitRequest(craft.Orbit, now, craft.TotalMass, craft.BoundingSphereRadiusBody,
            settings, saved.Model, Array.Empty<DeorbitEngine>(), saved.MinimumPulse, saved.ControlStep);
        VehicleAutopilotState state = VehicleAutopilotState.For(craft);
        DeorbitFixture.SetSettings(state, settings);
        state.Engage = state.AutoStage = true;
        state.DeorbitRequest = request;
        state.DeorbitPlanner = new DeorbitPlanner(request);
        state.DeorbitEngineSignature = (int)AccessTools.Method(typeof(GuidanceWindow), "DeorbitEngineSignature").Invoke(null, new object[] { craft })!;
        state.LandingPhase = GuidanceWindow.LandingPhase.DeorbitPlanning;
        var inputs = AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");
        var ambient = AccessTools.Field(typeof(GuidanceWindow), "_s");
        object? previous = ambient.GetValue(null);
        VehicleControlOwnership.TryClaim(craft, ControlClaimant.RcsTranslation, out _);
        TestSupport.SetManualControlInputs(craft, 0.63f, engineOn: false);
        try
        {
            var timer = Stopwatch.StartNew();
            while (state.LandingPhase == GuidanceWindow.LandingPhase.DeorbitPlanning && timer.Elapsed.TotalSeconds < 30)
                GuidanceWindow.ApplyAutopilot(craft);
            t.Check("an infeasible runtime plan stops before acquisition", state.LandingPhase == GuidanceWindow.LandingPhase.Done && !state.ControlAcquired,
                state.LandingStatus);
            t.Check("refusal keeps the existing controller", VehicleControlOwnership.HolderOf(craft) == ControlClaimant.RcsTranslation);
            t.Check("refusal keeps the existing engine input", !inputs(craft).EngineOn && inputs(craft).EngineThrottle == 0.63f);
            t.Check("refusal clears the plan and countdown state", state.DeorbitRequest == null
                && state.DeorbitPlanner == null && state.DeorbitPlan == null && state.DirectBrakingPlan == null
                && double.IsNaN(state.DeorbitNodeTime) && state.DeorbitEngineSignature == 0 && state.BurnStartTime == 0);
        }
        finally
        {
            state.DeorbitPlanner?.Dispose();
            VehicleAutopilotState.Remove(craft);
            VehicleControlOwnership.ReleaseAll(craft);
            ambient.SetValue(null, previous);
        }
    }
}

public class GuidanceDeorbitPlanTest : AfcTest
{
    public override string Name => "afc-guidance-deorbit-plan";
    protected virtual string SaveName => "Low Luna Orbit";
    private protected virtual DeorbitRequest CreateRequest(DeorbitFixture fixture) => fixture.Request;

    protected override void Execute(TestContext t)
    {
        using DeorbitFixture? fixture = DeorbitFixture.Open(t, SaveName);
        if (fixture == null) return;
        DeorbitRequest request = CreateRequest(fixture);
        using var planner = new DeorbitPlanner(request);
        var elapsed = Stopwatch.StartNew();
        double longestStep = 0;
        int steps = 0;
        while (!planner.Complete && elapsed.Elapsed.TotalSeconds < 120)
        {
            long start = Stopwatch.GetTimestamp();
            planner.Step(3);
            longestStep = Math.Max(longestStep, Stopwatch.GetElapsedTime(start).TotalMilliseconds);
            steps++;
        }
        t.Info($"Planning used {steps} steps and {planner.Evaluations} evaluations in {elapsed.Elapsed.TotalSeconds:F2} s, longest step {longestStep:F2} ms.");
        if (!t.Check("planning completes within the test budget", planner.Complete, planner.Status)) return;
        if (!t.Check("the saved high orbit has a two-stage plan", planner.Plan != null, planner.Status)) return;
        DeorbitPlan plan = planner.Plan!;
        t.Check("the deorbit node is within the next source orbit", plan.DepartureTime > request.Epoch
            && plan.DepartureTime <= request.Epoch + request.Source.Period);
        t.Check("the search does not require a large Lambert scan", planner.Evaluations < 2048, $"{planner.Evaluations} evaluations");
        t.Check("the constraint phase checks at most eight candidates", planner.CandidatesChecked <= DeorbitPlanner.CandidateLimit);
        t.Check("only one departure and flight refinement pass is used", planner.RefinementSamples <= 260);
        t.Info($"Departure in {plan.DepartureTime - request.Epoch:F1} s, node delta-v {plan.DeltaV.Length():F2} m/s.");
        Orbit transfer = Orbit.CreateFromStateCci(request.Parent, new UniverseTime(plan.DepartureTime),
            plan.DeparturePosition, plan.DepartureVelocity, request.Source.OrbitLineColor);
        StateVectors arrival = transfer.GetStateVectorsAt(new UniverseTime(plan.ArrivalTime));
        t.CheckAbs("stock propagation reaches the Lambert endpoint", (arrival.PositionCci - plan.ArrivalPosition).Length(), 0, 100);
        t.CheckAbs("stock propagation reaches the Lambert capture velocity", (arrival.VelocityCci - plan.ArrivalVelocity).Length(), 0, 0.1);
        t.Check("the propagated arrival meets the selected angle policy",
            DeorbitPlanner.MatchesArrivalAngle(request.Settings, arrival.PositionCci, arrival.VelocityCci));
        t.CheckAbs("stock propagation reaches the braking altitude", arrival.PositionCci.Length() - request.Parent.MeanRadius - request.SiteHeight,
            request.Settings.BrakingAltitude, 100);

        double3 siteAtArrival = request.SiteCcf.Transform(request.Parent.GetCcf2Cci(new UniverseTime(plan.ArrivalTime)));
        double3 normal = transfer.GetOrbitNormalCci();
        double angle = Math.Atan2(double3.Dot(double3.Cross(arrival.PositionCci.Normalized(), siteAtArrival), normal),
            double3.Dot(arrival.PositionCci.Normalized(), siteAtArrival));
        double uprange = angle * request.Parent.MeanRadius - request.Settings.GateUprange;
        t.CheckAbs("the stock endpoint is uprange of the rotating site", uprange,
            plan.BrakingDistance * request.Settings.DownrangeFactor, 100);
        t.Check("the transfer keeps the source direction", double3.Dot(normal, request.Source.GetOrbitNormalCci()) > 0);


        double minimumClearance = double.PositiveInfinity;
        for (double time = plan.DepartureTime; ; time = Math.Min(plan.ArrivalTime, time + 1))
        {
            StateVectors sample = transfer.GetStateVectorsAt(new UniverseTime(time));
            double3 direction = sample.PositionCci.Normalized().Transform(request.Parent.GetCci2Ccf(new UniverseTime(time)));
            double terrain = ((Celestial)request.Parent).GetTerrainHeightFromDirCcf(direction);
            minimumClearance = Math.Min(minimumClearance,
                sample.PositionCci.Length() - request.Parent.MeanRadius - terrain - request.VehicleRadius);
            if (time == plan.ArrivalTime) break;
        }
        t.Check("the stock flown arc clears the terrain", minimumClearance >= DeorbitPlanner.ClearanceMargin, $"{minimumClearance:F1} m");

        CheckBrakingAfterNode(t, request, plan, transfer);
    }

    private static void CheckBrakingAfterNode(TestContext t, DeorbitRequest request, DeorbitPlan plan, Orbit transfer)
    {
        double cutoffTime = plan.DepartureTime + 10;
        StateVectors cutoff = transfer.GetStateVectorsAt(new UniverseTime(cutoffTime));
        Orbit measured = Orbit.CreateFromStateCci(request.Parent, cutoff.StateTime,
            cutoff.PositionCci, cutoff.VelocityCci, transfer.OrbitLineColor);
        var actual = new DeorbitRequest(measured, cutoffTime, plan.CutoffMass, request.VehicleRadius,
            request.Settings, request.Model, request.Engines, request.MinimumPulse, request.ControlStep);
        using var braking = new DeorbitPlanner(actual, brakingOnly: true, arrivalHint: plan);
        var timer = Stopwatch.StartNew();
        while (!braking.Complete && timer.Elapsed.TotalSeconds < 30) braking.Step(3);
        if (!t.Check("the measured transfer orbit has a braking approach",
            braking.Complete && braking.Direct is { Refusal.Length: 0 }, braking.Status)) return;
        t.Check("the braking refresh cannot create another deorbit node", braking.Plan == null && braking.Evaluations == 0);
        DirectBrakingPlan direct = braking.Direct!;
        StateVectors ignition = measured.GetStateVectorsAt(new UniverseTime(direct.IgnitionTime));
        t.CheckAbs("the refreshed ignition position uses stock propagation", (direct.Position - ignition.PositionCci).Length(), 0, 0.01);
        t.Check("the refreshed braking point stays before the rotating site",
            DeorbitPlanner.SignedDistance(ignition.PositionCci, actual.GateAt(direct.IgnitionTime, measured.GetOrbitNormalCci()),
                measured.GetOrbitNormalCci(), request.Parent.MeanRadius) > 0);
    }
}
public sealed class GuidanceHighDeorbitPlanTest : GuidanceDeorbitPlanTest
{
    public override string Name => "afc-guidance-high-deorbit-plan";
    protected override string SaveName => "High Luna Orbit";
}

public sealed class GuidanceArrivalAnglePlanTest : GuidanceDeorbitPlanTest
{
    public override string Name => "afc-guidance-arrival-angle-plan";
    protected override string SaveName => "High Luna Orbit";

    private protected override DeorbitRequest CreateRequest(DeorbitFixture fixture)
    {
        DeorbitRequest saved = fixture.Request;
        return new DeorbitRequest(saved.Source, saved.Epoch, saved.Mass, saved.VehicleRadius,
            saved.Settings with { ArrivalDescentDeg = 5 }, saved.Model, saved.Engines, saved.MinimumPulse, saved.ControlStep);
    }
}

public sealed class GuidanceFlightLogDeorbitPlanTest : GuidanceDeorbitPlanTest
{
    public override string Name => "afc-guidance-flight-log-deorbit-plan";
    protected override string SaveName => "High Luna Orbit";

    private protected override DeorbitRequest CreateRequest(DeorbitFixture fixture)
    {
        DeorbitRequest saved = fixture.Request;
        const double epoch = 761245.276;
        Orbit source = Orbit.CreateFromStateCci(saved.Parent, new UniverseTime(epoch),
            new double3(-199271.4982, -7825923.2760, -900136.0748),
            new double3(788.7153, -20.0126, -1.5156), saved.Source.OrbitLineColor);
        return new DeorbitRequest(source, epoch, 223256.8, saved.VehicleRadius,
            saved.Settings with { Latitude = 5.737093, Longitude = -53.547194 }, saved.Model,
            saved.Engines, saved.MinimumPulse, saved.ControlStep);
    }
}

// A separate saved part tree supplies the engine model without replacing the loaded universe.
internal sealed class DeorbitFixture : IDisposable
{
    internal Vehicle Vehicle { get; }
    internal DeorbitRequest Request { get; }

    private DeorbitFixture(Vehicle vehicle, DeorbitRequest request) { Vehicle = vehicle; Request = request; }

    internal static void SetSettings(VehicleAutopilotState state, DeorbitSettings settings)
    {
        state.SiteLatDeg = settings.Latitude;
        state.SiteLonDeg = settings.Longitude;
        state.AimAltKm = settings.GateAltitude / 1000;
        state.GateUprangeKm = settings.GateUprange / 1000;
        state.BrakingAltitudeKm = settings.BrakingAltitude / 1000;
        state.DescentRate = settings.SinkRate;
        state.DownrangeFactor = settings.DownrangeFactor;
        state.DeorbitArrivalAngleEnabled = settings.ArrivalDescentDeg.HasValue;
        state.DeorbitArrivalDescentDeg = settings.ArrivalDescentDeg ?? 5;
    }

    internal static DeorbitFixture? Open(TestContext t, string saveName = "Low Luna Orbit")
    {
        string path = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            "My Games", "Kitten Space Agency", "saves", saveName, "universe.xml");
        if (!File.Exists(path) || t.System.All.Get("Luna") is not Celestial parent)
        {
            t.Skip($"{saveName} and the Luna body are required.");
            return null;
        }
        using var reader = new StreamReader(path);
        var save = (UniverseData)GameSaves.UniverseSerializer.Deserialize(reader)!;
        save.OnDataLoad(KSA.Mod.Empty);
        VehicleData data = save.CelestialSystems.SelectMany(system => system.Vehicles).Single(vehicle => vehicle.Id == "Rocket");
        if (!t.Check("the saved craft orbits Luna", data.ParentBody.Id == "Luna")) return null;
        double epoch = ((UniverseTime)data.AnalyticState.LastUpdateTime).Seconds();
        double3 position = data.AnalyticState.PositionCci;
        double3 velocity = data.AnalyticState.VelocityCci;
        bool high = saveName == "High Luna Orbit";
        double3 expectedPosition = high ? new(-222135.9202, -7825310.1338, -900088.3444) : new(1793295.7327, -766187.8995, -85961.7164);
        double3 expectedVelocity = high ? new(788.6541, -22.2868, -1.7772) : new(626.3123, 1446.6422, 166.5749);
        if (!t.Check("the saved orbit matches the regression fixture",
            (position - expectedPosition).Length() < 1 && (velocity - expectedVelocity).Length() < 0.01)) return null;
        Orbit source = Orbit.CreateFromStateCci(parent, new UniverseTime(epoch), position, velocity, new byte4(255, 255, 255, 255));
        Orbit live = Orbit.CreateFromStateCci(parent, Universe.GetElapsedTime(), position, velocity, source.OrbitLineColor);
        Vehicle craft = VehicleFixtures.SpawnFromSaveData(t.System, parent, data, "HarnessDeorbit" + Guid.NewGuid().ToString("N"), live);
        bool rails = PhysicsBubble._forceOffRails;
        try
        {
            TestSupport.SetManualControlInputs(craft, 0, engineOn: false);
            craft.FlightComputer.BurnMode = FlightComputerBurnMode.Manual;
            PhysicsBubble._forceOffRails = true;
            t.Session.CreateDriver().Step(0.01, 2);
            craft.Parts.PerformanceSequences.RecomputeForFlight(0);
            UpfgVehicle model = KsaVehicleAdapter.Build(craft, 0);
            var engines = new List<DeorbitEngine>();
            double floor = Math.Max(craft.GetMinThrottle(), 0.001);
            for (int i = 0; i <= 16; i++)
            {
                double throttle = floor + (1 - floor) * i / 16;
                (double thrust, double flow) = KsaEnginePerf.AtThrottle(craft, throttle, 0);
                if (thrust > 0 && flow > 0) engines.Add(new(throttle, thrust, flow));
            }
            double pulse = KsaEnginePerf.MinimumPulse(craft);
            if (!t.Check("the saved stage has supplied liquid engines and a braking model",
                KsaEnginePerf.SupportsThrottleControl(craft) && engines.Count > 0 && model.Stages.Count > 0))
            { VehicleSpawner.Despawn(craft); return null; }
            var settings = new DeorbitSettings(0.253, 46.300, 11000, 515, 0, 20, 1.2);
            return new(craft, new DeorbitRequest(source, epoch, craft.TotalMass, craft.BoundingSphereRadiusBody,
                settings, model, engines.ToArray(), pulse, high ? 1.0 / 30 : 1.0 / 60));
        }
        catch
        {
            VehicleSpawner.Despawn(craft);
            throw;
        }
        finally { PhysicsBubble._forceOffRails = rails; }
    }

    public void Dispose()
    {
        VehicleControlOwnership.ReleaseAll(Vehicle);
        VehicleAutopilotState.Remove(Vehicle);
        VehicleSpawner.Despawn(Vehicle);
    }
}

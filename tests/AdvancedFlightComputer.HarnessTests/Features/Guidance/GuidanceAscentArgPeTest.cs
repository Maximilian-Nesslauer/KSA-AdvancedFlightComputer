using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The ascent's insertion target, against orbits the game itself propagates.
//
// Geometry, with no save: around inclined eccentric orbits, one of them retrograde, the insertion UpfgTarget
// aims at under each propagated position has that position's radius, speed and flight-path angle, and the
// argument of periapsis it reads off the state vectors is the game's own element. Then the free insertion at
// periapsis and placed past it, the insertion where the orbit climbs through a floor above periapsis, the
// per-solve rate limit, the floor holding the side it entered from, and the near-circular fallback to free.
//
// Chase orbit: against a spawned eccentric target, launch-to-target plans a co-elliptic orbit - the target's
// eccentricity and argument of periapsis on a semi-major axis the offset below its own - and copies the
// argument of periapsis only while that is switched on.
//
// Insertion search: on the flight's own stage model and start state, the free insertion the search settles on
// costs no more than the cheapest one found by pricing every degree, beyond the search's own threshold.
//
// Flight: the real UPFG ascent with the production hooks, from an inclined circular orbit to an eccentric
// orbit in the same plane, with the argument of periapsis set so the burn has to insert InsertionAnomalyDeg
// past periapsis rather than at it. The orbit the game reports after cutoff must have that argument of
// periapsis. The burn is a long one under the staging test's g-limit: a short burn that has to climb is where
// this UPFG diverges whatever the target, which would say nothing about the argument of periapsis. For the same
// reason the insertion stays well inside the band a burn from orbit can be flown in at all: in closed-loop
// simulation of this flight a free insertion pinned 18 deg past the natural cutoff already diverged, so asking
// the fixed target for that would test UPFG's reach rather than the argument of periapsis.
public sealed class GuidanceAscentArgPeTest : AfcTest
{
    public override string Name => "afc-guidance-ascent-argpe";

    private static readonly string[] DefaultSaves = { "Test Vehicle 2", "Test Vehicle 1" };

    private const int GeometrySamples = 16;
    private const double GeometryRelTol = 1e-7;
    private const double GeometryAngleTolDeg = 1e-4;
    private const double PlacedInsertionDeg = 20.0;

    private const double ChaseOffsetKm = 20.0;

    // Enough to converge a solver from the preview onto one instant, so the search and the brute force read the same cost curve.
    private const int SettleSolves = 300;
    // The goal moves at most MaxGoalStepDeg a search, so this reaches the far end of the search's range from periapsis.
    private const int SearchCycles = 12;
    // What the parabola's reading of the curve may leave on top of the search's own threshold, m/s.
    private const double SearchSettleSlackMs = 3.0;

    // Inclined, so the node and with it the game's argument of periapsis are well defined for the flown orbit.
    private const double SpawnInclinationDeg = 28.5;
    private const double StepDt = 0.25;
    private const double MaxFlightSeconds = 900.0;
    private const double SampleIntervalS = 5.0;
    private const int PreviewIterations = 60;
    private const double TargetPeriapsisRiseKm = 100.0;
    private const double TargetApoapsisKm = 200_000.0;
    // The apoapsis stays this far inside the home body's sphere of influence.
    private const double SoiFraction = 0.4;
    private const double GLimitG = 2.0;
    private const double InsertionAnomalyDeg = 10.0;
    private const double ArgPeTolDeg = 3.0;
    private const double PeriapsisTolKm = 25.0;
    // Relative, because an apoapsis this high moves by kilometres per centimetre per second at cutoff.
    private const double ApoapsisRelTol = 0.10;
    private const double InclinationTolDeg = 0.25;

    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(
            AccessTools.Field(typeof(GuidanceWindow), "_s"));

    private static readonly MethodInfo StartGuidance =
        AccessTools.Method(typeof(GuidanceWindow), "StartGuidance");

    private static readonly MethodInfo RefreshStageModel =
        AccessTools.Method(typeof(GuidanceWindow), "RefreshStageModel");

    private static readonly MethodInfo BuildUpfgVehicle =
        AccessTools.Method(typeof(GuidanceWindow), "BuildUpfgVehicle");

    private static readonly MethodInfo ApplyGLimit =
        AccessTools.Method(typeof(GuidanceWindow), "ApplyGLimit");

    private static readonly MethodInfo TryChaseOrbit =
        AccessTools.Method(typeof(GuidanceWindow), "TryChaseOrbit");

    private static readonly MethodInfo ApplyChaseOrbit =
        AccessTools.Method(typeof(GuidanceWindow), "ApplyChaseOrbit");

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        CheckGeometry(t, home);
        CheckAimShaping(t, home);

        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(DefaultSaves);
        if (saves.Count == 0)
        {
            t.Skip($"neither '{DefaultSaves[0]}' nor '{DefaultSaves[1]}' is in the game's Vehicles folder, so the chase orbit, the insertion search and the flight are not covered.");
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
        using AutoStageTestPatches.Scope staging = AutoStageTestPatches.Apply();
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.ascent-argpe");
        try
        {
            VehicleCommandSink.ApplyPatches(harmony);
            GuidanceFeature.ApplyDriverPatches(harmony);
            SharedVehicleHooks.GuidanceEnabled = true;
            GuidanceWindow.SetModActive(true);

            Vehicle vehicle;
            try
            {
                Orbit spawn = OrbitFixtures.InclinedEllipticalAt(home, AutoStageFlightSupport.SpawnAltitudeM,
                    AutoStageFlightSupport.SpawnAltitudeM, Rad(SpawnInclinationDeg), Universe.GetElapsedTime());
                vehicle = VehicleSpawner.SpawnFromSave(saves[0], t.System, home, "HarnessGuidanceAscentArgPe", spawn);
            }
            catch (InvalidOperationException e)
            {
                t.Fail("spawn", e.Message);
                return;
            }

            Program.ControlledVehicle = vehicle;
            PhysicsBubble._forceOffRails = true;
            StagingConfig.EngineDelays.Clear();
            StagingConfig.DecouplerDelays.Clear();
            driver.Step(0.05, 40);

            VehicleAutopilotState state = VehicleAutopilotState.For(vehicle);
            AmbientState() = state;
            CheckChaseOrbit(t, home, vehicle, state, saves[0]);
            Fly(t, vehicle, state, home, driver);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            GuidanceFeature.DisableDriver();
            SharedVehicleHooks.GuidanceEnabled = previousEnabled;
            GuidanceWindow.SetModActive(previousModActive);
            AmbientState() = previousAmbient;
            AutoStageFlightSupport.CleanupAfterFlight(t.System, preexisting);
            Program.ControlledVehicle = previousFocus;
        }
    }

    private static void CheckGeometry(TestContext t, IParentBody home)
    {
        UniverseTime now = Universe.GetElapsedTime();
        var cases = new (double PeKm, double ApKm, double IncDeg, double ArgPeDeg)[]
        {
            (200.0, 1_000.0, 51.6, 30.0),
            (300.0, 20_000.0, 28.5, 270.0),
            (400.0, 3_000.0, 97.0, 135.0),
        };

        foreach ((double peKm, double apKm, double incDeg, double argPeDeg) in cases)
        {
            string tag = $"{peKm:F0} x {apKm:F0} km at {incDeg:F1} deg, arg. Pe {argPeDeg:F0} deg";
            Orbit orbit = OrbitFixtures.InclinedEllipticalAt(home, peKm * 1000.0, apKm * 1000.0, Rad(incDeg), now, Rad(argPeDeg));

            // The fixture builds the orbit from state vectors, so the game's elements say what those vectors
            // mean. A copied argument of periapsis is only the target's if UpfgTarget reads it the same way.
            t.CheckAbs($"{tag}: the game's argument of periapsis is the one the fixture set, deg",
                AngleDiffDeg(Deg(orbit.ArgumentOfPeriapsis), argPeDeg), 0.0, GeometryAngleTolDeg);

            double lanDeg = Deg(orbit.LongitudeOfAscendingNode);
            UpfgTarget target = UpfgTarget.FromOrbit(peKm, apKm, incDeg, lanDeg, home.MeanRadius, home.Mu, argPeDeg);
            t.Check($"{tag}: the target holds the argument of periapsis", target.ArgPeFixed, $"eccentricity {target.Ecc:F5}");
            double3 hHat = double3.Normalize(double3.Cross(orbit.StateVectors.PositionCci, orbit.StateVectors.VelocityCci));
            t.CheckAbs($"{tag}: the target plane is the orbit's, deg off its angular momentum",
                Deg(Math.Acos(Math.Clamp(double3.Dot(target.Normal, hHat), -1.0, 1.0))), 0.0, GeometryAngleTolDeg);

            double worstRadius = 0.0, worstSpeed = 0.0, worstFpaDeg = 0.0, worstArgPeDeg = 0.0;
            for (int k = 0; k < GeometrySamples; k++)
            {
                StateVectors sv = orbit.GetStateVectorsAt(now + new UniverseTime(orbit.Period * k / GeometrySamples));
                double3 r = sv.PositionCci;
                double3 v = sv.VelocityCci;
                UpfgTarget.Insertion aim = target.InsertionToward(r, default, double.PositiveInfinity);
                double fpa = Math.Asin(Math.Clamp(double3.Dot(double3.Normalize(r), double3.Normalize(v)), -1.0, 1.0));
                worstRadius = Math.Max(worstRadius, Math.Abs(aim.Radius / r.Length() - 1.0));
                worstSpeed = Math.Max(worstSpeed, Math.Abs(aim.Speed / v.Length() - 1.0));
                worstFpaDeg = Math.Max(worstFpaDeg, Math.Abs(Deg(aim.Fpa - fpa)));
                double argPe = UpfgTarget.ArgumentOfPeriapsisOf(r, v, home.Mu, orbit.LongitudeOfAscendingNode);
                worstArgPeDeg = Math.Max(worstArgPeDeg, Math.Abs(AngleDiffDeg(Deg(argPe), Deg(orbit.ArgumentOfPeriapsis))));
            }
            t.CheckAbs($"{tag}: aimed radius against the propagated one, worst relative error round the orbit", worstRadius, 0.0, GeometryRelTol);
            t.CheckAbs($"{tag}: aimed speed against the propagated one, worst relative error", worstSpeed, 0.0, GeometryRelTol);
            t.CheckAbs($"{tag}: aimed flight-path angle against the propagated one, worst error deg", worstFpaDeg, 0.0, GeometryAngleTolDeg);
            t.CheckAbs($"{tag}: argument of periapsis read off the state vectors against the game's, worst error deg", worstArgPeDeg, 0.0, GeometryAngleTolDeg);

            StateVectors atPe = orbit.GetStateVectorsAt(orbit.TimeAtPeriapsis);
            double h = double3.Cross(atPe.PositionCci, atPe.VelocityCci).Length();
            UpfgTarget free = UpfgTarget.FromOrbit(peKm, apKm, incDeg, lanDeg, home.MeanRadius, home.Mu);
            UpfgTarget.Insertion freeAim = free.InsertionToward(orbit.StateVectors.PositionCci, default, double.PositiveInfinity);
            t.Check($"{tag}: a free target inserts at periapsis, level, whatever the direction",
                !free.ArgPeFixed && !freeAim.FloorLimited
                && Math.Abs(freeAim.Radius / atPe.PositionCci.Length() - 1.0) <= GeometryRelTol
                && Math.Abs(freeAim.Speed / atPe.VelocityCci.Length() - 1.0) <= GeometryRelTol
                && Math.Abs(freeAim.Fpa) < 1e-12,
                $"radius {freeAim.Radius:F1} m against {atPe.PositionCci.Length():F1}, speed {freeAim.Speed:F4} m/s against {atPe.VelocityCci.Length():F4}, FPA {Deg(freeAim.Fpa):E2} deg");

            // Placed past periapsis, a free insertion is the orbit's own state there: on its radius, with its angular momentum and its energy.
            UpfgTarget placed = UpfgTarget.FromOrbit(peKm, apKm, incDeg, lanDeg, home.MeanRadius, home.Mu, double.NaN, 0.0, PlacedInsertionDeg);
            UpfgTarget.Insertion placedAim = placed.InsertionToward(orbit.StateVectors.PositionCci, default, double.PositiveInfinity);
            UpfgTarget.Insertion clonedAim = target.WithFreeInsertion(Rad(PlacedInsertionDeg))
                .InsertionToward(orbit.StateVectors.PositionCci, default, double.PositiveInfinity);
            double semiLatus = orbit.SemiMajorAxis * (1.0 - orbit.Eccentricity * orbit.Eccentricity);
            double placedRadius = semiLatus / (1.0 + orbit.Eccentricity * Math.Cos(Rad(PlacedInsertionDeg)));
            t.Check($"{tag}: a free insertion placed {PlacedInsertionDeg:F0} deg past periapsis is the orbit's own state there",
                !placedAim.FloorLimited && placedAim.Fpa > 0.0
                && Math.Abs(Deg(placedAim.TrueAnomaly) - PlacedInsertionDeg) < 1e-9
                && Math.Abs(placedAim.Radius / placedRadius - 1.0) <= GeometryRelTol
                && Math.Abs(placedAim.Radius * placedAim.Speed * Math.Cos(placedAim.Fpa) / h - 1.0) <= GeometryRelTol
                && Math.Abs(placedAim.Speed * placedAim.Speed / (home.Mu * (2.0 / placedRadius - 1.0 / orbit.SemiMajorAxis)) - 1.0) <= GeometryRelTol,
                Describe(placedAim));
            t.Check($"{tag}: the same insertion made from the fixed target frees its argument of periapsis",
                clonedAim.Radius == placedAim.Radius && clonedAim.Speed == placedAim.Speed && clonedAim.Fpa == placedAim.Fpa
                && !target.WithFreeInsertion(Rad(PlacedInsertionDeg)).ArgPeFixed && target.ArgPeFixed,
                $"{Describe(clonedAim)} against {Describe(placedAim)}");

            double floor = orbit.Periapsis + 0.25 * (orbit.Apoapsis - orbit.Periapsis);
            UpfgTarget floored = UpfgTarget.FromOrbit(peKm, apKm, incDeg, lanDeg, home.MeanRadius, home.Mu, double.NaN, floor);
            UpfgTarget.Insertion floorAim = floored.InsertionToward(atPe.PositionCci, default, double.PositiveInfinity);
            double visViva = home.Mu * (2.0 / floor - 1.0 / orbit.SemiMajorAxis);
            t.Check($"{tag}: a periapsis under the floor inserts where the orbit climbs through it",
                floorAim.FloorLimited && floorAim.Fpa > 0.0
                && Math.Abs(floorAim.Radius / floor - 1.0) <= GeometryRelTol
                && Math.Abs(floorAim.Radius * floorAim.Speed * Math.Cos(floorAim.Fpa) / h - 1.0) <= GeometryRelTol
                && Math.Abs(floorAim.Speed * floorAim.Speed / visViva - 1.0) <= GeometryRelTol,
                $"{Describe(floorAim)}; floor {floor:F1} m, r v cos(FPA) {floorAim.Radius * floorAim.Speed * Math.Cos(floorAim.Fpa):E8} against h {h:E8}");
        }
    }

    // The aim's own rules: the rate limit, the side the floor holds, and the near-circular fallback.
    private static void CheckAimShaping(TestContext t, IParentBody home)
    {
        const double peKm = 200.0, apKm = 1_000.0, incDeg = 51.6, lanDeg = 40.0, argPeDeg = 30.0;
        UpfgTarget target = UpfgTarget.FromOrbit(peKm, apKm, incDeg, lanDeg, home.MeanRadius, home.Mu, argPeDeg);

        UpfgTarget.Insertion AimAt(UpfgTarget on, double nuDeg, UpfgTarget.Insertion previous, double maxStep)
            => on.InsertionToward(UpfgTarget.PeriapsisDirection(on.Inclination, on.Lan, on.ArgPe + Rad(nuDeg)), previous, maxStep);

        UpfgTarget.Insertion start = AimAt(target, 0.0, default, Rad(5.0));
        UpfgTarget.Insertion stepped = AimAt(target, 40.0, start, Rad(5.0));
        t.CheckAbs("toward a cutoff 40 deg round, one solve moves the aim 5 deg", Deg(stepped.TrueAnomaly), 5.0, 1e-9);

        UpfgTarget.Insertion nearAp = AimAt(target, -178.0, default, Rad(5.0));
        UpfgTarget.Insertion wrapped = AimAt(target, 179.0, nearAp, Rad(5.0));
        t.CheckAbs("the aim takes the short way across apoapsis, deg", AngleDiffDeg(Deg(wrapped.TrueAnomaly), 179.0), 0.0, 1e-9);

        // A floor the ellipse climbs through 20 deg either side of periapsis.
        double floor = target.RadiusAt(Rad(20.0));
        UpfgTarget floored = UpfgTarget.FromOrbit(peKm, apKm, incDeg, lanDeg, home.MeanRadius, home.Mu, argPeDeg, floor);
        UpfgTarget.Insertion first = AimAt(floored, -3.0, default, double.PositiveInfinity);
        UpfgTarget.Insertion held = AimAt(floored, -3.0, first, double.PositiveInfinity);
        UpfgTarget.Insertion outside = AimAt(floored, -35.0, held, double.PositiveInfinity);
        UpfgTarget.Insertion heldDown = AimAt(floored, -3.0, outside, double.PositiveInfinity);
        t.Check("a cutoff over periapsis under the floor inserts at the floor, climbing",
            first.FloorLimited && Math.Abs(Deg(first.TrueAnomaly) - 20.0) < 1e-6 && first.Fpa > 0.0, Describe(first));
        t.Check("the floor holds the climbing side while the cutoff stays over periapsis",
            held.FloorLimited && Math.Abs(Deg(held.TrueAnomaly) - 20.0) < 1e-6, Describe(held));
        t.Check("a cutoff clear of the floor inserts where it is, descending",
            !outside.FloorLimited && Math.Abs(Deg(outside.TrueAnomaly) + 35.0) < 1e-6 && outside.Fpa < 0.0, Describe(outside));
        t.Check("back over periapsis from the descending side, the floor holds the descending crossing",
            heldDown.FloorLimited && Math.Abs(Deg(heldDown.TrueAnomaly) + 20.0) < 1e-6 && heldDown.Fpa < 0.0, Describe(heldDown));

        UpfgTarget round = UpfgTarget.FromOrbit(300.0, 300.2, incDeg, lanDeg, home.MeanRadius, home.Mu, argPeDeg);
        UpfgTarget.Insertion roundAim = round.InsertionToward(
            UpfgTarget.PeriapsisDirection(round.Inclination, round.Lan, Rad(90.0)), default, double.PositiveInfinity);
        t.Check("a near-circular target falls back to a free argument of periapsis",
            !round.ArgPeFixed && roundAim.TrueAnomaly == 0.0 && Math.Abs(roundAim.Radius / round.Pe - 1.0) < 1e-12,
            $"eccentricity {round.Ecc:E2}; {Describe(roundAim)}");
    }

    private static void CheckChaseOrbit(TestContext t, IParentBody home, Vehicle chaser, VehicleAutopilotState state, string save)
    {
        Orbit orbit = OrbitFixtures.InclinedEllipticalAt(home, 600_000.0, 6_000_000.0, Rad(51.6), Universe.GetElapsedTime(), Rad(210.0));
        Vehicle target;
        try
        {
            target = VehicleSpawner.SpawnFromSave(save, t.System, home, "HarnessGuidanceArgPeChaseTarget", orbit);
        }
        catch (InvalidOperationException e)
        {
            t.Fail("spawn the chase target", e.Message);
            return;
        }

        try
        {
            state.TargetId = target.Id;
            state.ChaseOffsetKm = ChaseOffsetKm;
            state.MatchTargetArgPe = true;

            object?[] args = { chaser, chaser.Orbit, home, home.MeanRadius, null };
            var status = (GuidanceWindow.ChaseStatus)TryChaseOrbit.Invoke(null, args)!;
            if (!t.Check("launch-to-target plans a chase orbit for the spawned target",
                    status is GuidanceWindow.ChaseStatus.Ok or GuidanceWindow.ChaseStatus.PlaneUnreachable, $"status {status}"))
                return;
            var plan = (GuidanceWindow.ChasePlan)args[4]!;

            Orbit flown = target.Orbit;
            double chasePe = plan.PeKm * 1000.0 + home.MeanRadius;
            double chaseAp = plan.ApKm * 1000.0 + home.MeanRadius;
            t.Info($"target {(flown.Periapsis - home.MeanRadius) / 1000.0:F1} x {(flown.Apoapsis - home.MeanRadius) / 1000.0:F1} km, "
                 + $"e {flown.Eccentricity:F6}, arg. Pe {Deg(flown.ArgumentOfPeriapsis):F4} deg; "
                 + $"chase {plan.PeKm:F1} x {plan.ApKm:F1} km, arg. Pe {plan.ArgPeDeg:F4} deg");
            t.CheckAbs("the chase orbit has the target's eccentricity",
                (chaseAp - chasePe) / (chaseAp + chasePe), flown.Eccentricity, 1e-9);
            t.CheckAbs("the chase orbit's semi-major axis is the offset below the target's, m",
                (chasePe + chaseAp) / 2.0, flown.SemiMajorAxis - ChaseOffsetKm * 1000.0, 1.0);
            t.CheckAbs("the chase orbit has the target's argument of periapsis, deg",
                AngleDiffDeg(plan.ArgPeDeg, Deg(flown.ArgumentOfPeriapsis)), 0.0, GeometryAngleTolDeg);
            t.CheckAbs("the chase orbit has the target's inclination, deg", plan.IncDeg, Deg(flown.Inclination), GeometryAngleTolDeg);
            t.CheckAbs("the chase orbit has the target's LAN, deg",
                AngleDiffDeg(plan.LanDeg, Deg(flown.LongitudeOfAscendingNode)), 0.0, GeometryAngleTolDeg);

            ApplyChaseOrbit.Invoke(null, new object[] { plan });
            t.Check("with the copy on, the chase fixes the target's argument of periapsis",
                state.ArgPeFixed && state.ArgPeDeg == plan.ArgPeDeg, $"fixed={state.ArgPeFixed}, arg. Pe {state.ArgPeDeg:F4} deg");
            state.MatchTargetArgPe = false;
            ApplyChaseOrbit.Invoke(null, new object[] { plan });
            t.Check("with the copy off, the chase leaves the argument of periapsis free", !state.ArgPeFixed);
        }
        finally
        {
            state.TargetId = "";
            state.MatchTargetArgPe = true;
            state.ArgPeFixed = false;
            VehicleSpawner.Despawn(target);
        }
    }

    // The search against brute force, on the stage model and state the flight starts from. Every degree of free
    // insertion the search may use is priced the way the search prices a probe - a solver re-solved from that
    // instant until it has converged (UpfgInsertionSearch.TryPrice) - and the search is then stepped cycle after
    // cycle from that state, the live solver re-settled on each insertion it moves to. Where it settles has to cost
    // no more than the cheapest degree, beyond the threshold the search itself refuses to chase. The state stands
    // still throughout, which is harder on the search than flight: nothing moves on between searches, so any bias
    // its own solution puts into the costs compounds cycle after cycle instead of being flown out.
    private static void CheckInsertionSearch(TestContext t, UpfgGuidance preview, UpfgTarget free,
                                             double3 r, double3 v, double mass, double mu, UpfgVehicle model)
    {
        if (preview.Tgo < UpfgInsertionSearch.FreezeTgoS)
        {
            t.Skip($"the burn is {preview.Tgo:F0} s, too short for the insertion search to run.");
            return;
        }

        UpfgGuidance settled = preview.Clone();
        for (int i = 0; i < SettleSolves; i++)
            settled.Step(r, v, mass, mu, free, model);

        int unpriced = 0;
        double Price(double nu)
        {
            if (UpfgInsertionSearch.TryPrice(settled, free, nu, r, v, mass, mu, model, out double cost, out _, out _))
                return cost;
            unpriced++;
            return double.NaN;
        }

        // Periapsis itself can have no converged solution from here - UPFG diverges on it from some burns - so it is priced like any other degree, not required.
        double atPeriapsis = Price(0.0);
        double bestDeg = double.NaN, bestCost = double.PositiveInfinity;
        for (int deg = 0; deg <= (int)UpfgInsertionSearch.MaxNuDeg; deg++)
        {
            double cost = deg == 0 ? atPeriapsis : Price(Rad(deg));
            if (cost < bestCost)
            {
                bestCost = cost;
                bestDeg = deg;
            }
        }
        if (!t.Check("some insertion the search may use prices from the flight's start state", double.IsFinite(bestCost),
                $"none of 0..{UpfgInsertionSearch.MaxNuDeg:F0} deg converged within {UpfgInsertionSearch.MaxProbeSolves} solves"))
            return;

        var search = new UpfgInsertionSearch();
        UpfgGuidance live = settled.Clone();
        for (int cycle = 0; cycle < SearchCycles; cycle++)
        {
            UpfgTarget flown = free.WithFreeInsertion(search.Nu);
            for (int i = 0; i < SettleSolves; i++)
                live.Step(r, v, mass, mu, flown, model);
            search.Step(cycle * UpfgInsertionSearch.SearchIntervalS, UpfgInsertionSearch.SearchIntervalS,
                true, true, live, flown, r, v, mass, mu, model);
        }

        double goalDeg = Deg(search.GoalNu);
        double goalCost = Price(search.GoalNu);
        t.Info($"insertion search: brute force cheapest at {bestDeg:F0} deg past periapsis, "
             + (double.IsFinite(atPeriapsis) ? $"{atPeriapsis - bestCost:F1} m/s under periapsis" : $"{bestCost:F1} m/s, periapsis itself not converging")
             + $"{(unpriced > 0 ? $" ({unpriced} insertions did not converge and were left out)" : "")}; "
             + $"search settled at {goalDeg:F2} deg, {goalCost:F1} m/s by the same pricing, "
             + $"reporting a saving of {search.SavingMs:F1} m/s after {search.LastSearchSolves} solves in its last search");
        t.Check("the insertion search settles on the climbing side, inside its range",
            goalDeg >= 0.0 && goalDeg <= UpfgInsertionSearch.MaxNuDeg + 1e-9, $"{goalDeg:F3} deg");
        t.CheckAbs("the insertion the search settles on, m/s above the brute-force cheapest (floored at zero)",
            double.IsFinite(goalCost) ? Math.Max(goalCost - bestCost, 0.0) : double.PositiveInfinity,
            0.0, UpfgInsertionSearch.MinSavingMs + SearchSettleSlackMs);
        // Priced the same way, the saving the search reports is the one its goal has: the old fixed-count probes claimed 10 km/s here. Only where periapsis prices, because that is what the saving is measured from.
        if (double.IsFinite(atPeriapsis) && double.IsFinite(search.SavingMs))
            t.CheckAbs("the saving the search reports against the brute-force price of its goal, m/s",
                search.SavingMs, atPeriapsis - goalCost, UpfgInsertionSearch.MinSavingMs + SearchSettleSlackMs);
    }

    private static void Fly(TestContext t, Vehicle vehicle, VehicleAutopilotState state, IParentBody home, SimDriver driver)
    {
        // The spawn orbit's own plane, from its state vectors, so the burn reshapes the orbit and turns its periapsis without a plane change.
        double3 r0 = vehicle.Orbit.StateVectors.PositionCci;
        double3 v0 = vehicle.Orbit.StateVectors.VelocityCci;
        double3 hHat = double3.Normalize(double3.Cross(r0, v0));
        double incDeg = Deg(Math.Acos(Math.Clamp(hHat.Z, -1.0, 1.0)));
        double lanDeg = WrapDeg(Deg(Math.Atan2(hHat.X, -hHat.Y)));
        double altKm = (r0.Length() - home.MeanRadius) / 1000.0;
        double peKm = altKm + TargetPeriapsisRiseKm;
        double apKm = TargetApoapsisKm;
        if (double.IsFinite(home.SphereOfInfluence) && home.SphereOfInfluence > 0.0)
            apKm = Math.Min(apKm, SoiFraction * home.SphereOfInfluence / 1000.0);

        state.PeKm = peKm;
        state.ApKm = apKm;
        state.IncDeg = incDeg;
        state.LanDeg = lanDeg;
        state.LanSeeded = true;
        state.ArgPeFixed = false;
        state.Engage = true;
        state.AutoStage = true;
        state.GLimitEnabled = true;
        state.GLimitG = GLimitG;
        state.ReserveArmed = false;
        state.TargetId = "";
        AmbientState() = state;

        // Lit the way the staging test lights it: the save may spawn with an engine already flagged active,
        // which would hide guidance's own cold-ignition cue.
        if (!AutoStageFlightSupport.Arm(t, vehicle))
            return;
        vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
        driver.Step(1.0);

        // Where a free burn would cut off, from UPFG itself: a scratch solver run to convergence on the stage model
        // the ascent will fly, g-limit and all, the way the landing predictor measures its burn. An idle craft with
        // no panel open is not given a stage model, so it is built here.
        RefreshStageModel.Invoke(null, new object[] { vehicle });
        var model = (UpfgVehicle?)BuildUpfgVehicle.Invoke(null, new object[] { vehicle });
        if (model == null || model.Stages.Count == 0)
        {
            t.Fail("preview", "no stage model to plan the burn with after ignition");
            return;
        }
        ApplyGLimit.Invoke(null, new object[] { model, GLimitG });
        UpfgTarget free = UpfgTarget.FromOrbit(peKm, apKm, incDeg, lanDeg, home.MeanRadius, home.Mu);
        var preview = new UpfgGuidance();
        double3 r = vehicle.Orbit.StateVectors.PositionCci;
        double3 v = vehicle.Orbit.StateVectors.VelocityCci;
        for (int i = 0; i < PreviewIterations; i++)
            preview.Step(r, v, vehicle.TotalMass, home.Mu, free, model);
        double freeCutoffDeg = Deg(free.ArgumentOfLatitude(preview.Rd));
        if (!t.Check("the preview solve converges on a cutoff",
                preview.Converged && double.IsFinite(freeCutoffDeg) && preview.Tgo > 0.0,
                $"converged {preview.Converged}, tgo {preview.Tgo:F1} s, stage model {GuidanceLog.DescribeStages(model)}"))
            return;

        CheckInsertionSearch(t, preview, free, r, v, vehicle.TotalMass, home.Mu, model);

        double argPeDeg = WrapDeg(freeCutoffDeg - InsertionAnomalyDeg);
        state.ArgPeFixed = true;
        state.ArgPeDeg = argPeDeg;
        t.Info($"preview: tgo {preview.Tgo:F1} s, vgo {preview.VgoMag:F0} m/s, a free burn cuts off at argument of latitude {freeCutoffDeg:F2} deg; "
             + $"arg. Pe fixed at {argPeDeg:F2} deg, {InsertionAnomalyDeg:F0} deg behind it; target {peKm:F1} x {apKm:F0} km under {GLimitG:F1} g");

        StartGuidance.Invoke(null, new object[] { vehicle, vehicle.Orbit, home });
        if (!t.Check("the ascent starts running", state.Running, $"status '{state.Status}'"))
            return;

        double time = 0.0, nextSample = 0.0;
        bool converged = false;
        UpfgTarget.Insertion lastAim = default;
        bool held = false;
        while (time < MaxFlightSeconds && state.Running)
        {
            driver.Step(StepDt);
            time += StepDt;
            converged |= state.Upfg.Converged;
            held |= state.Upfg.AimHeld;
            if (state.Upfg.Aim.Valid)
                lastAim = state.Upfg.Aim;
            if (time >= nextSample)
            {
                nextSample += SampleIntervalS;
                double altNowKm = (vehicle.Orbit.StateVectors.PositionCci.Length() - home.MeanRadius) / 1000.0;
                t.Info($"t={time,6:F1}s phase={state.Phase} tgo={state.Upfg.Tgo,6:F1}s vgo={state.Upfg.VgoMag,6:F0}m/s converged={state.Upfg.Converged} "
                     + $"aim={Deg(lastAim.TrueAnomaly),6:F2}deg{(state.Upfg.AimHeld ? " held" : "")} from Pe at {(lastAim.Radius - home.MeanRadius) / 1000.0,7:F1}km FPA={Deg(lastAim.Fpa),5:F2}deg "
                     + $"alt={altNowKm,7:F1}km mass={vehicle.TotalMass / 1000.0,6:F1}t status='{state.Status}'");
            }
        }

        if (!t.Check("the ascent cuts off and releases guidance",
                !state.Running && state.Status.StartsWith("Ascent complete", StringComparison.Ordinal),
                $"running={state.Running} after {time:F0} s, status '{state.Status}', error '{state.GuidanceError}'"))
            return;
        driver.Step(StepDt, 8);

        Orbit flownOrbit = vehicle.Orbit;
        double radius = home.MeanRadius;
        t.Info($"flown {(flownOrbit.Periapsis - radius) / 1000.0:F1} x {(flownOrbit.Apoapsis - radius) / 1000.0:F1} km, inc {Deg(flownOrbit.Inclination):F3} deg, "
             + $"LAN {Deg(flownOrbit.LongitudeOfAscendingNode):F3} deg, arg. Pe {Deg(flownOrbit.ArgumentOfPeriapsis):F3} deg; target {peKm:F1} x {apKm:F1} km, "
             + $"inc {incDeg:F3} deg, LAN {lanDeg:F3} deg, arg. Pe {argPeDeg:F3} deg; last aim {Describe(lastAim)}");
        t.Check("UPFG converged on the fixed target", converged);
        // The end of the burn flies the held insertion, not the one under the cutoff, or the aim can run away from the vehicle (see UpfgGuidance.AimHoldTgoS).
        t.Check($"the aim was held for the last {UpfgGuidance.AimHoldTgoS:F0} s of the burn", held);
        // Otherwise the flight proves nothing about steering: a burn that inserted at periapsis would pass on a free target.
        t.Check("the burn aimed well off periapsis, as the fixed argument of periapsis asks",
            lastAim.Valid && Math.Abs(Deg(lastAim.TrueAnomaly)) > InsertionAnomalyDeg / 2.0, Describe(lastAim));
        t.CheckAbs("flown argument of periapsis against the target's, deg",
            AngleDiffDeg(Deg(flownOrbit.ArgumentOfPeriapsis), argPeDeg), 0.0, ArgPeTolDeg);
        t.CheckAbs("flown periapsis altitude, km", (flownOrbit.Periapsis - radius) / 1000.0, peKm, PeriapsisTolKm);
        t.CheckRel("flown apoapsis altitude, km", (flownOrbit.Apoapsis - radius) / 1000.0, apKm, ApoapsisRelTol);
        t.CheckAbs("flown inclination, deg", Deg(flownOrbit.Inclination), incDeg, InclinationTolDeg);
    }

    private static string Describe(UpfgTarget.Insertion aim) =>
        $"true anomaly {Deg(aim.TrueAnomaly):F4} deg (raw {Deg(aim.RawTrueAnomaly):F4}), radius {aim.Radius:F1} m, "
        + $"speed {aim.Speed:F2} m/s, FPA {Deg(aim.Fpa):F4} deg, floor-limited {aim.FloorLimited}, valid {aim.Valid}";

    private static double Rad(double degrees) => degrees * Math.PI / 180.0;

    private static double Deg(double radians) => radians * 180.0 / Math.PI;

    private static double WrapDeg(double degrees)
    {
        degrees %= 360.0;
        return degrees < 0.0 ? degrees + 360.0 : degrees;
    }

    // a - b folded into [-180, 180).
    private static double AngleDiffDeg(double a, double b)
    {
        double d = (a - b) % 360.0;
        if (d >= 180.0)
            d -= 360.0;
        else if (d < -180.0)
            d += 360.0;
        return d;
    }
}

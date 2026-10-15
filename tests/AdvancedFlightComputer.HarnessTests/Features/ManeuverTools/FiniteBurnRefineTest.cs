using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.PlanWindow;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Flies each refined maneuver with the stock FiniteBurnPredictor and checks the orbit after cutoff against the goal.
// The engine is a low thrust one, so every burn lasts a large part of the orbit and the impulsive burn flown as is misses clearly.
public sealed class FiniteBurnRefineTest : AfcTest
{
    private const double StartMassKg = 20_000.0;
    private const double ThrustN = 6_000.0;
    private const double IspSec = 330.0;
    private const double StandardGravity = 9.80665;

    // Test orbits, in meters above the home body's mean radius.
    private const double LowAltitudeM = 400_000.0;
    private const double MidAltitudeM = 600_000.0;
    private const double HighAltitudeM = 2_400_000.0;
    private const double SetApoapsisAltitudeM = 3_000_000.0;
    private const double SetPeriapsisAltitudeM = 1_500_000.0;
    private const double PlaneApoapsisAltitudeM = 1_500_000.0;

    private const double StartInclinationRad = 5.0 * Math.PI / 180.0;
    private const double SetInclinationTargetRad = 8.0 * Math.PI / 180.0;
    private const double MatchTargetInclinationRad = 8.0 * Math.PI / 180.0;

    private const double EccentricityTol = 1e-3;
    // The apsis refinement stops at 0.1 m, so this leaves room for the conic conversion only.
    private const double ApsisAbsTolM = 10.0;
    private const double PlanAgreementTol = 1e-6;

    public override string Name => "afc-finite-burn-refine";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        UniverseTime now = Universe.GetElapsedTime();
        FiniteBurnPhase[] phases =
        {
            new(ThrustN, ThrustN / (StandardGravity * IspSec), StartMassKg, double.PositiveInfinity),
        };

        CheckGoalMapping(t);
        CheckCircularize(t, home, phases, now);
        CheckSetApsis(t, home, phases, now, setApoapsis: true);
        CheckSetApsis(t, home, phases, now, setApoapsis: false);
        CheckSetInclination(t, home, phases, now);
        CheckMatchInclination(t, home, phases, now);
        CheckWithoutThrust(t, home, now);
    }

    private static void CheckGoalMapping(TestContext t)
    {
        t.Check("circularize uses the circular goal",
            Patch_DrawPlanWindow.FiniteBurnGoalFor(ManeuverTools.KeyStockCircularizeApoapsis).Kind
                == FiniteBurnGoalKind.Circular
            && Patch_DrawPlanWindow.FiniteBurnGoalFor(ManeuverTools.KeyStockCircularizePeriapsis).Kind
                == FiniteBurnGoalKind.Circular);
        t.Check("set apsis uses the apsis goal",
            Patch_DrawPlanWindow.FiniteBurnGoalFor(ManeuverTools.KeySetApoapsis).Kind == FiniteBurnGoalKind.Apsis
            && Patch_DrawPlanWindow.FiniteBurnGoalFor(ManeuverTools.KeySetPeriapsis).Kind == FiniteBurnGoalKind.Apsis);
        t.Check("set inclination uses the plane goal",
            Patch_DrawPlanWindow.FiniteBurnGoalFor(ManeuverTools.KeySetInclination).Kind == FiniteBurnGoalKind.Plane);
    }

    private static void CheckCircularize(TestContext t, IParentBody home, FiniteBurnPhase[] phases, UniverseTime now)
    {
        const string label = "circularize at Ap";
        Orbit orbit = OrbitFixtures.EllipticalAt(home, LowAltitudeM, HighAltitudeM, now);
        OrbitManeuvers.ManeuverResult? impulsive = OrbitManeuvers.ComputeCircularize(orbit, useApoapsis: true, now);
        if (!ManeuverAssertions.RequireResult(t, label, impulsive))
            return;

        OrbitManeuvers.ManeuverResult refined = OrbitManeuvers.RefineForFiniteBurn(
            orbit, impulsive!.Value, phases, FiniteBurnGoal.Circular);
        Orbit flownImpulsive = Fly(orbit, impulsive.Value, phases, out _);
        Orbit flownRefined = Fly(orbit, refined, phases, out FiniteBurnResult arc);
        LogBurn(t, label, impulsive.Value, refined, arc);

        bool shape = ManeuverAssertions.ResultShapeHolds(t, label, orbit, refined, now);
        t.Check($"{label}: refined burn reaches a circular orbit",
            shape && flownRefined.Eccentricity < EccentricityTol,
            $"ecc {flownRefined.Eccentricity:E3}, impulsive burn flown as is {flownImpulsive.Eccentricity:E3}");
        t.Check($"{label}: refinement improves on the impulsive burn",
            flownRefined.Eccentricity < flownImpulsive.Eccentricity);

        CheckPreviewPlan(t, label, orbit, refined, phases, flownRefined);
    }

    private static void CheckSetApsis(
        TestContext t, IParentBody home, FiniteBurnPhase[] phases, UniverseTime now, bool setApoapsis)
    {
        string label = setApoapsis ? "set apoapsis" : "set periapsis";
        Orbit orbit = setApoapsis
            ? OrbitFixtures.EllipticalAt(home, LowAltitudeM, MidAltitudeM, now)
            : OrbitFixtures.EllipticalAt(home, LowAltitudeM, SetApoapsisAltitudeM, now);
        double targetAltitude = setApoapsis ? SetApoapsisAltitudeM : SetPeriapsisAltitudeM;
        double targetRadius = home.MeanRadius + targetAltitude;
        OrbitManeuvers.ManeuverResult? impulsive = setApoapsis
            ? OrbitManeuvers.ComputeSetApoapsis(orbit, targetAltitude, home.MeanRadius, now)
            : OrbitManeuvers.ComputeSetPeriapsis(orbit, targetAltitude, home.MeanRadius, now);
        if (!ManeuverAssertions.RequireResult(t, label, impulsive))
            return;

        OrbitManeuvers.ManeuverResult refined = OrbitManeuvers.RefineForFiniteBurn(
            orbit, impulsive!.Value, phases, FiniteBurnGoal.Apsis);
        Orbit flownImpulsive = Fly(orbit, impulsive.Value, phases, out _);
        Orbit flownRefined = Fly(orbit, refined, phases, out FiniteBurnResult arc);
        LogBurn(t, label, impulsive.Value, refined, arc);

        double reached = setApoapsis ? flownRefined.Apoapsis : flownRefined.Periapsis;
        double impulsiveReached = setApoapsis ? flownImpulsive.Apoapsis : flownImpulsive.Periapsis;
        bool shape = ManeuverAssertions.ResultShapeHolds(t, label, orbit, refined, now);
        t.Check($"{label}: refined burn reaches the target radius",
            shape && Approx.Mixed(reached, targetRadius, ApsisAbsTolM, 0.0),
            $"reached {reached:F1}m, target {targetRadius:F1}m, impulsive burn flown as is {impulsiveReached:F1}m");
        t.Check($"{label}: refinement keeps the burn direction",
            double3.Cross(refined.DvVlf, impulsive.Value.DvVlf).Length()
                <= ManeuverAssertions.TransformRelTol * refined.DvVlf.Length() * impulsive.Value.DvVlf.Length() + 1e-9
            && double3.Dot(refined.DvVlf, impulsive.Value.DvVlf) > 0.0);
    }

    private static void CheckSetInclination(TestContext t, IParentBody home, FiniteBurnPhase[] phases, UniverseTime now)
    {
        const string label = "set inclination at DN";
        Orbit orbit = OrbitFixtures.InclinedEllipticalAt(
            home, LowAltitudeM, PlaneApoapsisAltitudeM, StartInclinationRad, now);
        OrbitManeuvers.ManeuverResult? impulsive = OrbitManeuvers.ComputeSetInclination(
            orbit, SetInclinationTargetRad, useDescendingNode: true, now,
            OrbitManeuvers.InclinationReference.Ecliptic);
        if (!ManeuverAssertions.RequireResult(t, label, impulsive))
            return;

        OrbitManeuvers.ManeuverResult refined = OrbitManeuvers.RefineForFiniteBurn(
            orbit, impulsive!.Value, phases, FiniteBurnGoal.Plane(null));
        Orbit flownImpulsive = Fly(orbit, impulsive.Value, phases, out _);
        Orbit flownRefined = Fly(orbit, refined, phases, out FiniteBurnResult arc);
        LogBurn(t, label, impulsive.Value, refined, arc);

        double error = Math.Abs(OrbitManeuvers.GetInclinationAgainst(
            flownRefined, OrbitManeuvers.InclinationReference.Ecliptic) - SetInclinationTargetRad);
        double impulsiveError = Math.Abs(OrbitManeuvers.GetInclinationAgainst(
            flownImpulsive, OrbitManeuvers.InclinationReference.Ecliptic) - SetInclinationTargetRad);
        bool shape = ManeuverAssertions.ResultShapeHolds(t, label, orbit, refined, now);
        t.Check($"{label}: refined burn reaches the target inclination",
            shape && error < ManeuverAssertions.IncTolRad,
            $"error {OrbitFixtures.Deg(error):F4}deg, impulsive burn flown as is {OrbitFixtures.Deg(impulsiveError):F4}deg");
    }

    private static void CheckMatchInclination(TestContext t, IParentBody home, FiniteBurnPhase[] phases, UniverseTime now)
    {
        const string label = "match inclination at DN";
        Orbit orbit = OrbitFixtures.InclinedEllipticalAt(
            home, LowAltitudeM, PlaneApoapsisAltitudeM, StartInclinationRad, now);
        Orbit target = OrbitFixtures.InclinedEllipticalAt(
            home, MidAltitudeM, MidAltitudeM, MatchTargetInclinationRad, now, 1.0);
        // The fixture puts the descending node at now, so search from just after it: the refined burn starts before its node, and that node is one orbit later.
        OrbitManeuvers.ManeuverResult? impulsive = OrbitManeuvers.ComputeMatchInclination(
            orbit, target, useDescendingNode: true, now + 1.0);
        if (!ManeuverAssertions.RequireResult(t, label, impulsive))
            return;

        OrbitManeuvers.ManeuverResult? atNow = OrbitManeuvers.ComputeMatchInclination(
            orbit, target, useDescendingNode: true, now);
        if (atNow is OrbitManeuvers.ManeuverResult nodeNow)
        {
            OrbitManeuvers.ManeuverResult kept = OrbitManeuvers.RefineForFiniteBurn(
                orbit, nodeNow, phases, FiniteBurnGoal.Plane(target), now);
            t.Check($"{label}: a node too close to now keeps the impulsive burn",
                kept.BurnTime == nodeNow.BurnTime && kept.DvVlf == nodeNow.DvVlf && kept.BurnTime >= now,
                $"burn {kept.BurnTime.Seconds():F1}s, node {nodeNow.BurnTime.Seconds():F1}s, now {now.Seconds():F1}s");
        }

        OrbitManeuvers.ManeuverResult refined = OrbitManeuvers.RefineForFiniteBurn(
            orbit, impulsive!.Value, phases, FiniteBurnGoal.Plane(target), now);
        Orbit flownImpulsive = Fly(orbit, impulsive.Value, phases, out _);
        Orbit flownRefined = Fly(orbit, refined, phases, out FiniteBurnResult arc);
        LogBurn(t, label, impulsive.Value, refined, arc);

        double relInc = flownRefined.GetRelativeInclination(target).Value();
        double impulsiveRelInc = flownImpulsive.GetRelativeInclination(target).Value();
        bool shape = ManeuverAssertions.ResultShapeHolds(t, label, orbit, refined, now);
        t.Check($"{label}: refined burn reaches the target plane",
            shape && relInc < ManeuverAssertions.IncTolRad,
            $"relInc {OrbitFixtures.Deg(relInc):F4}deg, impulsive burn flown as is {OrbitFixtures.Deg(impulsiveRelInc):F4}deg");
    }

    private static void CheckWithoutThrust(TestContext t, IParentBody home, UniverseTime now)
    {
        Orbit orbit = OrbitFixtures.EllipticalAt(home, LowAltitudeM, HighAltitudeM, now);
        OrbitManeuvers.ManeuverResult? impulsive = OrbitManeuvers.ComputeCircularize(orbit, useApoapsis: true, now);
        if (!ManeuverAssertions.RequireResult(t, "no thrust", impulsive))
            return;

        FiniteBurnPhase[] noThrust = { new(0.0, 0.0, StartMassKg, double.PositiveInfinity) };
        FiniteBurnGoal[] goals =
        {
            FiniteBurnGoal.SameOrbit, FiniteBurnGoal.Circular, FiniteBurnGoal.Plane(null),
            FiniteBurnGoal.Transfer(null), FiniteBurnGoal.Apsis, FiniteBurnGoal.Period,
        };
        bool unchanged = true;
        foreach (FiniteBurnGoal goal in goals)
        {
            foreach (FiniteBurnPhase[] phases in new[] { Array.Empty<FiniteBurnPhase>(), noThrust })
            {
                var (time, dv) = FiniteBurns.Refine(
                    orbit, impulsive!.Value.BurnTime, impulsive.Value.DvVlf, phases, goal);
                unchanged &= time == impulsive.Value.BurnTime && dv == impulsive.Value.DvVlf;
            }
        }
        t.Check("no thrust keeps the impulsive burn", unchanged);

        FlightPlan before = FlightPlan.CreateUninitialized(default);
        before.SetFirstOrbit(orbit);
        FlightPlan plan = FlightPlan.CreateUninitialized(default);
        PatchedConic? piece = FiniteBurns.TryAddFiniteBurn(
            plan, before.Patches[0], impulsive!.Value.BurnTime, impulsive.Value.DvVlf, noThrust, out _);
        t.Check("no thrust adds no finite burn to a preview", piece == null && plan.Patches.Count == 0);
    }

    // The preview plan is built the way AFC builds its own previews, so its conic after cutoff must be the orbit the refined burn flies to.
    private static void CheckPreviewPlan(
        TestContext t, string label, Orbit orbit, in OrbitManeuvers.ManeuverResult refined,
        FiniteBurnPhase[] phases, Orbit flownRefined)
    {
        FlightPlan before = FlightPlan.CreateUninitialized(default);
        before.SetFirstOrbit(orbit);
        FlightPlan plan = FlightPlan.CreateUninitialized(default);
        PatchedConic? piece = FiniteBurns.TryAddFiniteBurn(
            plan, before.Patches[0], refined.BurnTime, refined.DvVlf, phases, out bool impacted);
        if (piece == null || piece.BurnTrajectory is not FiniteBurnResult arc)
        {
            t.Fail($"{label}: preview has a finite burn", "no burn piece was added");
            return;
        }

        t.Check($"{label}: preview burn is centered on the impulse time",
            !impacted && arc.Ignition < refined.BurnTime && arc.Cutoff > refined.BurnTime,
            $"ignition {arc.Ignition.Seconds():F1}s impulse {refined.BurnTime.Seconds():F1}s cutoff {arc.Cutoff.Seconds():F1}s");
        t.Check($"{label}: preview orbit after cutoff matches the flown burn",
            Approx.Mixed(piece.Orbit.Eccentricity, flownRefined.Eccentricity, PlanAgreementTol, 0.0)
            && Approx.Mixed(piece.Orbit.SemiMajorAxis, flownRefined.SemiMajorAxis, 1.0, PlanAgreementTol),
            $"preview ecc {piece.Orbit.Eccentricity:E3} SMA {piece.Orbit.SemiMajorAxis:F1}m, " +
            $"flown ecc {flownRefined.Eccentricity:E3} SMA {flownRefined.SemiMajorAxis:F1}m");
    }

    private static Orbit Fly(
        Orbit orbit, in OrbitManeuvers.ManeuverResult maneuver, FiniteBurnPhase[] phases, out FiniteBurnResult arc)
    {
        arc = FiniteBurnPredictor.Predict(orbit, maneuver.BurnTime, maneuver.DvVlf, phases);
        return Orbit.CreateFromStateCci(
            orbit.Parent, arc.Cutoff, arc.PositionCci, arc.VelocityCci, VehicleSpawner.OrbitLineColor);
    }

    private static void LogBurn(
        TestContext t, string label, in OrbitManeuvers.ManeuverResult impulsive,
        in OrbitManeuvers.ManeuverResult refined, in FiniteBurnResult arc)
        => t.Info($"{label}: burn {(arc.Cutoff - arc.Ignition).Seconds():F0}s, " +
                  $"dv {impulsive.DvCci.Length():F2} -> {refined.DvCci.Length():F2}m/s, " +
                  $"time shift {(refined.BurnTime - impulsive.BurnTime).Seconds():F2}s");
}

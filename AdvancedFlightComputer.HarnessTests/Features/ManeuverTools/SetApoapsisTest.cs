using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates OrbitManeuvers.ComputeSetApoapsis: a single burn at periapsis must move the apoapsis
// to the requested altitude and leave the periapsis where it was, and impossible requests (target
// at or below the periapsis, unbound orbit) must yield no maneuver.
public sealed class SetApoapsisTest : AfcTest
{
    // Test orbit and targets, in meters above the home body's mean radius.
    private const double PeriapsisAltitudeM = 300_000.0;
    private const double ApoapsisAltitudeM = 2_000_000.0;
    private const double RaiseTargetAltitudeM = 5_000_000.0;
    private const double LowerTargetAltitudeM = 1_000_000.0;

    public override string Name => "afc-set-apoapsis";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        UniverseTime now = Universe.GetElapsedTime();
        Orbit orbit = OrbitFixtures.EllipticalAt(home, PeriapsisAltitudeM, ApoapsisAltitudeM, now);

        CheckReaches(t, orbit, home, RaiseTargetAltitudeM, "raise Ap", now);
        CheckReaches(t, orbit, home, LowerTargetAltitudeM, "lower Ap", now);
        ManeuverAssertions.CheckNone(t, "target below Pe",
            OrbitManeuvers.ComputeSetApoapsis(orbit, PeriapsisAltitudeM / 2.0, home.MeanRadius, now));
        ManeuverAssertions.CheckNone(t, "hyperbolic orbit",
            OrbitManeuvers.ComputeSetApoapsis(
                OrbitFixtures.HyperbolicAt(home, PeriapsisAltitudeM, now),
                RaiseTargetAltitudeM, home.MeanRadius, now));
    }

    private static void CheckReaches(
        TestContext t, Orbit orbit, IParentBody home, double targetAltitudeM, string label, UniverseTime now)
    {
        OrbitManeuvers.ManeuverResult? result =
            OrbitManeuvers.ComputeSetApoapsis(orbit, targetAltitudeM, home.MeanRadius, now);
        if (!ManeuverAssertions.RequireResult(t, label, result))
            return;

        bool ok = ManeuverAssertions.ResultShapeHolds(t, label, orbit, result!.Value, now);
        Orbit after = ManeuverAssertions.Apply(orbit, result.Value);
        double targetRadius = home.MeanRadius + targetAltitudeM;
        ok &= Approx.Rel(after.Apoapsis, targetRadius, ManeuverAssertions.RelTol);
        ok &= Approx.Rel(after.Periapsis, orbit.Periapsis, ManeuverAssertions.RelTol);
        t.Check(label, ok,
            $"dv={result.Value.DvCci.Length():F2}m/s Ap={after.Apoapsis:E6} (target {targetRadius:E6}) " +
            $"Pe={after.Periapsis:E6} (was {orbit.Periapsis:E6})");
    }
}

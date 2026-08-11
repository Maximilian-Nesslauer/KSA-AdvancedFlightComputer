using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates OrbitManeuvers.ComputeSetPeriapsis: a single burn at apoapsis must move the periapsis
// to the requested altitude and leave the apoapsis where it was, and impossible requests (target
// at or above the apoapsis, unbound orbit) must yield no maneuver.
public sealed class SetPeriapsisTest : AfcTest
{
    // Test orbit and targets, in meters above the home body's mean radius.
    private const double PeriapsisAltitudeM = 300_000.0;
    private const double ApoapsisAltitudeM = 2_000_000.0;
    private const double RaiseTargetAltitudeM = 800_000.0;
    private const double LowerTargetAltitudeM = 150_000.0;

    public override string Name => "afc-set-periapsis";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        UniverseTime now = Universe.GetElapsedTime();
        Orbit orbit = OrbitFixtures.EllipticalAt(home, PeriapsisAltitudeM, ApoapsisAltitudeM, now);

        CheckReaches(t, orbit, home, RaiseTargetAltitudeM, "raise Pe", now);
        CheckReaches(t, orbit, home, LowerTargetAltitudeM, "lower Pe", now);
        // Clearly above the apoapsis: exact equality is a floating-point knife edge on
        // Orbit.Apoapsis, and the plan window blocks targets within 1 km of the opposite apse
        // anyway.
        ManeuverAssertions.CheckNone(t, "target above Ap",
            OrbitManeuvers.ComputeSetPeriapsis(orbit, ApoapsisAltitudeM + 100_000.0, home.MeanRadius, now));
        ManeuverAssertions.CheckNone(t, "hyperbolic orbit",
            OrbitManeuvers.ComputeSetPeriapsis(
                OrbitFixtures.HyperbolicAt(home, PeriapsisAltitudeM, now),
                RaiseTargetAltitudeM, home.MeanRadius, now));
    }

    private static void CheckReaches(
        TestContext t, Orbit orbit, IParentBody home, double targetAltitudeM, string label, UniverseTime now)
    {
        OrbitManeuvers.ManeuverResult? result =
            OrbitManeuvers.ComputeSetPeriapsis(orbit, targetAltitudeM, home.MeanRadius, now);
        if (!ManeuverAssertions.RequireResult(t, label, result))
            return;

        bool ok = ManeuverAssertions.ResultShapeHolds(t, label, orbit, result!.Value, now);
        Orbit after = ManeuverAssertions.Apply(orbit, result.Value);
        double targetRadius = home.MeanRadius + targetAltitudeM;
        ok &= Approx.Rel(after.Periapsis, targetRadius, ManeuverAssertions.RelTol);
        ok &= Approx.Rel(after.Apoapsis, orbit.Apoapsis, ManeuverAssertions.RelTol);
        t.Check(label, ok,
            $"dv={result.Value.DvCci.Length():F2}m/s Pe={after.Periapsis:E6} (target {targetRadius:E6}) " +
            $"Ap={after.Apoapsis:E6} (was {orbit.Apoapsis:E6})");
    }
}

using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates OrbitManeuvers.ComputeCircularize at both apses: the applied burn must produce a
// near-circular orbit at the burn radius, and an already-circular or unbound orbit must yield no
// maneuver (the tool's "nothing to do" contract).
public sealed class CircularizeTest : AfcTest
{
    // Test orbit, in meters above the home body's mean radius.
    private const double PeriapsisAltitudeM = 400_000.0;
    private const double ApoapsisAltitudeM = 2_400_000.0;
    private const double CircularAltitudeM = 500_000.0;

    private const double EccentricityTol = 1e-3;

    public override string Name => "afc-circularize";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        UniverseTime now = Universe.GetElapsedTime();
        Orbit orbit = OrbitFixtures.EllipticalAt(home, PeriapsisAltitudeM, ApoapsisAltitudeM, now);

        CheckCircularizes(t, orbit, useApoapsis: true, orbit.Apoapsis, "at Ap", now);
        CheckCircularizes(t, orbit, useApoapsis: false, orbit.Periapsis, "at Pe", now);
        ManeuverAssertions.CheckNone(t, "already circular",
            OrbitManeuvers.ComputeCircularize(
                OrbitFixtures.CircularAt(home, CircularAltitudeM, now), true, now));
        ManeuverAssertions.CheckNone(t, "hyperbolic orbit",
            OrbitManeuvers.ComputeCircularize(
                OrbitFixtures.HyperbolicAt(home, PeriapsisAltitudeM, now), true, now));
    }

    private static void CheckCircularizes(
        TestContext t, Orbit orbit, bool useApoapsis, double burnRadius, string label, UniverseTime now)
    {
        OrbitManeuvers.ManeuverResult? result = OrbitManeuvers.ComputeCircularize(orbit, useApoapsis, now);
        if (!ManeuverAssertions.RequireResult(t, label, result))
            return;

        bool ok = ManeuverAssertions.ResultShapeHolds(t, label, orbit, result!.Value, now);
        Orbit after = ManeuverAssertions.Apply(orbit, result.Value);
        ok &= after.Eccentricity < EccentricityTol;
        ok &= Approx.Rel(after.SemiMajorAxis, burnRadius, ManeuverAssertions.RelTol);
        t.Check(label, ok,
            $"dv={result.Value.DvCci.Length():F2}m/s ecc={after.Eccentricity:F5} " +
            $"SMA={after.SemiMajorAxis:E6} (target {burnRadius:E6})");
    }
}

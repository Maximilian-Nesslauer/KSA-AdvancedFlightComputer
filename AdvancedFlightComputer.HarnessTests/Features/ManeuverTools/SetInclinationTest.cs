using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates OrbitManeuvers.ComputeSetInclination: a node burn must set the inclination against the
// chosen reference plane while keeping the orbital speed and the node line, the coplanar special
// case must fall back to the CCI +X node convention, and already-satisfied or unbound inputs must
// yield no maneuver. The ecliptic checks read stock Orbit.Inclination (the classical element
// against the CCI Z plane) as an oracle independent of the reference-normal path under test.
public sealed class SetInclinationTest : AfcTest
{
    private const double StartInclinationRad = 5.0 * Math.PI / 180.0;
    private const double RaiseTargetRad = 20.0 * Math.PI / 180.0;
    private const double LowerTargetRad = 2.0 * Math.PI / 180.0;
    private const double EquatorialTargetRad = 15.0 * Math.PI / 180.0;
    private const double CoplanarTargetRad = 10.0 * Math.PI / 180.0;

    // With the default builder geometry the node sits exactly on the periapsis; this second orbit
    // rotates the periapsis off the node line so a node/apse mix-up in the code under test cannot
    // slip through.
    private const double OffApseArgumentOfPeriapsisRad = 35.0 * Math.PI / 180.0;

    // Test orbit, in meters above the home body's mean radius.
    private const double PeriapsisAltitudeM = 500_000.0;
    private const double ApoapsisAltitudeM = 1_500_000.0;

    private const double HalfFraction = 0.5;
    // A half rotation splits the inclination change exactly (both plane normals are perpendicular
    // to the node line), and the burn state comes from a closed-form node solve, so only
    // floating-point error remains; 1e-6 leaves generous headroom over that.
    private const double FractionRelTol = 1e-6;
    private const double NodeLineDotMin = 0.9999;

    public override string Name => "afc-set-inclination";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        UniverseTime now = Universe.GetElapsedTime();
        Orbit orbit = OrbitFixtures.InclinedEllipticalAt(
            home, PeriapsisAltitudeM, ApoapsisAltitudeM, StartInclinationRad, now);
        Orbit offApse = OrbitFixtures.InclinedEllipticalAt(
            home, PeriapsisAltitudeM, ApoapsisAltitudeM, StartInclinationRad, now,
            OffApseArgumentOfPeriapsisRad);

        CheckEcliptic(t, orbit, RaiseTargetRad, useDescendingNode: false, "raise (ecliptic, AN)", now);
        CheckEcliptic(t, orbit, RaiseTargetRad, useDescendingNode: true, "raise (ecliptic, DN)", now);
        CheckEcliptic(t, orbit, LowerTargetRad, useDescendingNode: false, "lower (ecliptic, AN)", now);
        CheckEcliptic(t, offApse, RaiseTargetRad, useDescendingNode: false,
            "raise (ecliptic, AN, off-apse Pe)", now);
        CheckHalfFraction(t, orbit, now);
        CheckEquatorial(t, orbit, home, now);
        CheckCoplanarStart(t, home, now);
        ManeuverAssertions.CheckNone(t, "already at target",
            OrbitManeuvers.ComputeSetInclination(
                orbit,
                OrbitManeuvers.GetInclinationAgainst(orbit, OrbitManeuvers.InclinationReference.Ecliptic),
                false, now, OrbitManeuvers.InclinationReference.Ecliptic));
        ManeuverAssertions.CheckNone(t, "hyperbolic orbit",
            OrbitManeuvers.ComputeSetInclination(
                OrbitFixtures.HyperbolicAt(home, PeriapsisAltitudeM, now),
                RaiseTargetRad, false, now, OrbitManeuvers.InclinationReference.Ecliptic));
    }

    private static void CheckEcliptic(
        TestContext t, Orbit orbit, double targetRad, bool useDescendingNode, string label, UniverseTime now)
    {
        OrbitManeuvers.ManeuverResult? result = OrbitManeuvers.ComputeSetInclination(
            orbit, targetRad, useDescendingNode, now, OrbitManeuvers.InclinationReference.Ecliptic);
        if (!ManeuverAssertions.RequireResult(t, label, result))
            return;

        bool ok = ManeuverAssertions.ResultShapeHolds(t, label, orbit, result!.Value, now);
        StateVectors sv = orbit.GetStateVectorsAt(result.Value.BurnTime);
        ok &= Approx.Rel(
            (sv.VelocityCci + result.Value.DvCci).Length(), sv.VelocityCci.Length(),
            ManeuverAssertions.TransformRelTol);

        Orbit after = ManeuverAssertions.Apply(orbit, result.Value);
        ok &= Math.Abs(after.Inclination - targetRad) < ManeuverAssertions.IncTolRad;
        ok &= NodeLinePreserved(orbit, after);
        t.Check(label, ok,
            $"dv={result.Value.DvCci.Length():F2}m/s inc={OrbitFixtures.Deg(after.Inclination):F3}deg " +
            $"(target {OrbitFixtures.Deg(targetRad):F3}deg)");
    }

    // The plane change must tilt the orbit about its own node line, not move the node: the line
    // where the plane crosses the ecliptic has to stay put (up to sign, which flips with the
    // rotation direction).
    private static bool NodeLinePreserved(Orbit before, Orbit after)
    {
        double3 lineBefore = double3.Cross(double3.UnitZ, before.GetOrbitNormalCci()).NormalizeOrZero();
        double3 lineAfter = double3.Cross(double3.UnitZ, after.GetOrbitNormalCci()).NormalizeOrZero();
        return Math.Abs(double3.Dot(lineBefore, lineAfter)) > NodeLineDotMin;
    }

    private static void CheckHalfFraction(TestContext t, Orbit orbit, UniverseTime now)
    {
        OrbitManeuvers.ManeuverResult? result = OrbitManeuvers.ComputeSetInclination(
            orbit, RaiseTargetRad, false, now, OrbitManeuvers.InclinationReference.Ecliptic, HalfFraction);
        if (!ManeuverAssertions.RequireResult(t, "half fraction", result))
            return;

        bool ok = ManeuverAssertions.ResultShapeHolds(t, "half fraction", orbit, result!.Value, now);
        Orbit after = ManeuverAssertions.Apply(orbit, result.Value);
        double expected = StartInclinationRad + (RaiseTargetRad - StartInclinationRad) * HalfFraction;
        ok &= Approx.Rel(after.Inclination, expected, FractionRelTol);
        t.Check("half fraction", ok,
            $"inc={OrbitFixtures.Deg(after.Inclination):F3}deg " +
            $"(expect {OrbitFixtures.Deg(expected):F3}deg)");
    }

    private static void CheckEquatorial(TestContext t, Orbit orbit, IParentBody home, UniverseTime now)
    {
        OrbitManeuvers.ManeuverResult? result = OrbitManeuvers.ComputeSetInclination(
            orbit, EquatorialTargetRad, false, now, OrbitManeuvers.InclinationReference.Equatorial);
        if (!ManeuverAssertions.RequireResult(t, "equatorial", result))
            return;

        bool ok = ManeuverAssertions.ResultShapeHolds(t, "equatorial", orbit, result!.Value, now);
        Orbit after = ManeuverAssertions.Apply(orbit, result.Value);
        // Measure against the equator directly from the body's rotation axis (the CCE Z axis in
        // CCI, via IParentBody.GetCce2Cci), not through the GetInclinationAgainst path under test.
        double3 equatorNormal = double3.UnitZ.Transform(home.GetCce2Cci());
        double incAfter = MathEx.SafeAcos(double3.Dot(equatorNormal, after.GetOrbitNormalCci()));
        double obliquity = MathEx.SafeAcos(double3.Dot(equatorNormal, double3.UnitZ));
        ok &= Math.Abs(incAfter - EquatorialTargetRad) < ManeuverAssertions.IncTolRad;
        t.Check("equatorial", ok,
            $"dv={result.Value.DvCci.Length():F2}m/s inc={OrbitFixtures.Deg(incAfter):F3}deg " +
            $"(target {OrbitFixtures.Deg(EquatorialTargetRad):F3}deg, " +
            $"obliquity {OrbitFixtures.Deg(obliquity):F2}deg)");
    }

    private static void CheckCoplanarStart(TestContext t, IParentBody home, UniverseTime now)
    {
        // An orbit in the ecliptic plane has no defined node against the ecliptic; the tool picks
        // CCI +X by convention. The set must still work and must put the node line on X.
        Orbit planar = OrbitFixtures.EllipticalAt(home, PeriapsisAltitudeM, ApoapsisAltitudeM, now);
        OrbitManeuvers.ManeuverResult? result = OrbitManeuvers.ComputeSetInclination(
            planar, CoplanarTargetRad, false, now, OrbitManeuvers.InclinationReference.Ecliptic);
        if (!ManeuverAssertions.RequireResult(t, "coplanar start", result))
            return;

        bool ok = ManeuverAssertions.ResultShapeHolds(t, "coplanar start", planar, result!.Value, now);
        Orbit after = ManeuverAssertions.Apply(planar, result.Value);
        ok &= Math.Abs(after.Inclination - CoplanarTargetRad) < ManeuverAssertions.IncTolRad;
        double3 nodeLine = double3.Cross(double3.UnitZ, after.GetOrbitNormalCci()).NormalizeOrZero();
        bool nodeOk = Math.Abs(double3.Dot(nodeLine, double3.UnitX)) > NodeLineDotMin;
        ok &= nodeOk;
        t.Check("coplanar start", ok,
            $"inc={OrbitFixtures.Deg(after.Inclination):F3}deg " +
            $"(target {OrbitFixtures.Deg(CoplanarTargetRad):F3}deg) nodeLineOnX={nodeOk}");
    }
}

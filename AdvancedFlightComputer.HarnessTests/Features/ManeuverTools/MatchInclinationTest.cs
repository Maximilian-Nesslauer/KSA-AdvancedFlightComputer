using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates OrbitManeuvers.ComputeMatchInclination: a node burn must rotate the orbit into the
// target's plane (relative inclination to zero) without changing the orbital speed, a partial
// fraction must shrink the relative inclination proportionally, and coplanar or unbound inputs
// must yield no maneuver.
public sealed class MatchInclinationTest : AfcTest
{
    private const double VehicleInclinationRad = 10.0 * Math.PI / 180.0;
    private const double TargetInclinationRad = 30.0 * Math.PI / 180.0;

    // With the default builder geometry the mutual node sits exactly on the vehicle's periapsis;
    // this second vehicle orbit rotates the periapsis off the node line so a node/apse mix-up in
    // the code under test cannot slip through.
    private const double OffApseArgumentOfPeriapsisRad = 35.0 * Math.PI / 180.0;

    // Test orbits, in meters above the home body's mean radius. The target is circular so its
    // plane is the only thing that matters.
    private const double VehiclePeriapsisAltitudeM = 400_000.0;
    private const double VehicleApoapsisAltitudeM = 1_200_000.0;
    private const double TargetAltitudeM = 800_000.0;
    private const double CoplanarAltitudeM = 900_000.0;

    private const double HalfFraction = 0.5;
    // A half rotation halves the relative inclination exactly when the burn sits on the mutual
    // node line, and the burn state comes from a closed-form node solve, so only floating-point
    // error remains; 1e-6 leaves generous headroom over that.
    private const double FractionRelTol = 1e-6;

    public override string Name => "afc-match-inclination";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        UniverseTime now = Universe.GetElapsedTime();
        Orbit vehicle = OrbitFixtures.InclinedEllipticalAt(
            home, VehiclePeriapsisAltitudeM, VehicleApoapsisAltitudeM, VehicleInclinationRad, now);
        Orbit vehicleOffApse = OrbitFixtures.InclinedEllipticalAt(
            home, VehiclePeriapsisAltitudeM, VehicleApoapsisAltitudeM, VehicleInclinationRad, now,
            OffApseArgumentOfPeriapsisRad);
        Orbit target = OrbitFixtures.InclinedEllipticalAt(
            home, TargetAltitudeM, TargetAltitudeM, TargetInclinationRad, now);
        double relInc0 = vehicle.GetRelativeInclination(target).Value();
        t.Info($"initial relative inclination {OrbitFixtures.Deg(relInc0):F3}deg");

        CheckMatches(t, vehicle, target, useDescendingNode: false, "full match at AN", now);
        CheckMatches(t, vehicle, target, useDescendingNode: true, "full match at DN", now);
        CheckMatches(t, vehicleOffApse, target, useDescendingNode: false,
            "full match at AN (off-apse Pe)", now);
        CheckHalves(t, vehicle, target, relInc0, now);
        ManeuverAssertions.CheckNone(t, "coplanar orbits",
            OrbitManeuvers.ComputeMatchInclination(vehicle,
                OrbitFixtures.InclinedEllipticalAt(
                    home, CoplanarAltitudeM, CoplanarAltitudeM, VehicleInclinationRad, now),
                false, now));
        ManeuverAssertions.CheckNone(t, "hyperbolic vehicle orbit",
            OrbitManeuvers.ComputeMatchInclination(
                OrbitFixtures.HyperbolicAt(home, VehiclePeriapsisAltitudeM, now), target, false, now));
    }

    private static void CheckMatches(
        TestContext t, Orbit vehicle, Orbit target, bool useDescendingNode, string label, UniverseTime now)
    {
        OrbitManeuvers.ManeuverResult? result =
            OrbitManeuvers.ComputeMatchInclination(vehicle, target, useDescendingNode, now);
        if (!ManeuverAssertions.RequireResult(t, label, result))
            return;

        bool ok = ManeuverAssertions.ResultShapeHolds(t, label, vehicle, result!.Value, now);
        StateVectors sv = vehicle.GetStateVectorsAt(result.Value.BurnTime);
        double speedBefore = sv.VelocityCci.Length();
        double speedAfter = (sv.VelocityCci + result.Value.DvCci).Length();
        ok &= Approx.Rel(speedAfter, speedBefore, ManeuverAssertions.TransformRelTol);

        Orbit after = ManeuverAssertions.Apply(vehicle, result.Value);
        double relIncAfter = after.GetRelativeInclination(target).Value();
        ok &= relIncAfter < ManeuverAssertions.IncTolRad;
        t.Check(label, ok,
            $"dv={result.Value.DvCci.Length():F2}m/s relInc after={relIncAfter:E3}rad " +
            $"speed {speedBefore:F2}->{speedAfter:F2}m/s");
    }

    private static void CheckHalves(TestContext t, Orbit vehicle, Orbit target, double relInc0, UniverseTime now)
    {
        OrbitManeuvers.ManeuverResult? result =
            OrbitManeuvers.ComputeMatchInclination(vehicle, target, false, now, HalfFraction);
        if (!ManeuverAssertions.RequireResult(t, "half fraction", result))
            return;

        bool ok = ManeuverAssertions.ResultShapeHolds(t, "half fraction", vehicle, result!.Value, now);
        Orbit after = ManeuverAssertions.Apply(vehicle, result.Value);
        double relIncAfter = after.GetRelativeInclination(target).Value();
        ok &= Approx.Rel(relIncAfter, relInc0 * HalfFraction, FractionRelTol);
        t.Check("half fraction", ok,
            $"relInc {relInc0:E3} -> {relIncAfter:E3}rad (expect {relInc0 * HalfFraction:E3})");
    }
}

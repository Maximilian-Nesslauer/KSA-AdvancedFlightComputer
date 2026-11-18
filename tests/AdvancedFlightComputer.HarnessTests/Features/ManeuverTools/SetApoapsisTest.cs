using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

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

        double radius = home.MeanRadius + PeriapsisAltitudeM;
        double speed = Math.Sqrt(2.0 * home.Mu / radius * (1.0 - 1e-7));
        Orbit nearParabolic = Orbit.CreateFromStateCci(home, now, new double3(radius, 0, 0),
            new double3(-0.1, Math.Sqrt(0.99), 0) * speed, VehicleSpawner.OrbitLineColor);
        t.Check("KSA classifies a slightly subunit eccentricity as unbound",
            nearParabolic is Orbit.Parabolic && nearParabolic.Eccentricity < 1.0 && !nearParabolic.IsBound(),
            $"type {nearParabolic.GetType().Name}, eccentricity {nearParabolic.Eccentricity:R}");
        ManeuverAssertions.CheckNone(t, "near-parabolic orbit",
            OrbitManeuvers.ComputeSetApoapsis(nearParabolic, RaiseTargetAltitudeM, home.MeanRadius, now));
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

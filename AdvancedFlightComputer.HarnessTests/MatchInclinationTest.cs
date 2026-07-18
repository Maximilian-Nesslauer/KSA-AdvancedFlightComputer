using AdvancedFlightComputer.Features.ManeuverTools;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates OrbitManeuvers.ComputeMatchInclination: a node burn must rotate the orbit into the
// target's plane (relative inclination to zero) without changing the orbital speed, a partial
// fraction must shrink the relative inclination proportionally, and coplanar or unbound inputs
// must yield no maneuver.
public sealed class MatchInclinationTest : IHarnessTest
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

    public string Name => "afc-match-inclination";

    public int Run(HeadlessSession session)
    {
        if (!ManeuverTestSupport.RequireHome(Name, session, out IParentBody home))
            return 1;

        SimTime now = Universe.GetElapsedSimTime();
        Orbit vehicle = ManeuverTestSupport.InclinedEllipticalCci(
            home, home.MeanRadius + VehiclePeriapsisAltitudeM, home.MeanRadius + VehicleApoapsisAltitudeM,
            VehicleInclinationRad, now);
        Orbit vehicleOffApse = ManeuverTestSupport.InclinedEllipticalCci(
            home, home.MeanRadius + VehiclePeriapsisAltitudeM, home.MeanRadius + VehicleApoapsisAltitudeM,
            VehicleInclinationRad, now, OffApseArgumentOfPeriapsisRad);
        Orbit target = ManeuverTestSupport.InclinedEllipticalCci(
            home, home.MeanRadius + TargetAltitudeM, home.MeanRadius + TargetAltitudeM,
            TargetInclinationRad, now);
        double relInc0 = vehicle.GetRelativeInclination(target).Value();
        HarnessLog.Line($"[{Name}] initial relative inclination {ManeuverTestSupport.Deg(relInc0):F3}deg");

        bool ok = true;
        ok &= CheckMatches(vehicle, target, useDescendingNode: false, "full match at AN", now);
        ok &= CheckMatches(vehicle, target, useDescendingNode: true, "full match at DN", now);
        ok &= CheckMatches(vehicleOffApse, target, useDescendingNode: false, "full match at AN (off-apse Pe)", now);
        ok &= CheckHalves(vehicle, target, relInc0, now);
        ok &= ManeuverTestSupport.CheckNull(Name, "coplanar orbits",
            OrbitManeuvers.ComputeMatchInclination(vehicle,
                ManeuverTestSupport.InclinedEllipticalCci(
                    home, home.MeanRadius + CoplanarAltitudeM, home.MeanRadius + CoplanarAltitudeM,
                    VehicleInclinationRad, now),
                false, now));
        ok &= ManeuverTestSupport.CheckNull(Name, "hyperbolic vehicle orbit",
            OrbitManeuvers.ComputeMatchInclination(
                ManeuverTestSupport.HyperbolicCci(home, home.MeanRadius + VehiclePeriapsisAltitudeM, now),
                target, false, now));

        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private bool CheckMatches(Orbit vehicle, Orbit target, bool useDescendingNode, string label, SimTime now)
    {
        OrbitManeuvers.ManeuverResult? result =
            OrbitManeuvers.ComputeMatchInclination(vehicle, target, useDescendingNode, now);
        if (!ManeuverTestSupport.RequireResult(Name, label, result))
            return false;

        bool ok = ManeuverTestSupport.CheckResultShape(Name, label, vehicle, result!.Value, now);
        StateVectors sv = vehicle.GetStateVectorsAt(result.Value.BurnTime);
        double speedBefore = sv.VelocityCci.Length();
        double speedAfter = (sv.VelocityCci + result.Value.DvCci).Length();
        ok &= ManeuverTestSupport.NearRel(speedAfter, speedBefore, ManeuverTestSupport.TransformRelTol);

        Orbit after = ManeuverTestSupport.Apply(vehicle, result.Value);
        double relIncAfter = after.GetRelativeInclination(target).Value();
        ok &= relIncAfter < ManeuverTestSupport.IncTolRad;
        HarnessLog.Line($"[{Name}] TEST {label}: dv={result.Value.DvCci.Length():F2}m/s " +
                        $"relInc after={relIncAfter:E3}rad speed {speedBefore:F2}->{speedAfter:F2}m/s " +
                        $"=> {TestSupport.Verdict(ok)}");
        return ok;
    }

    private bool CheckHalves(Orbit vehicle, Orbit target, double relInc0, SimTime now)
    {
        OrbitManeuvers.ManeuverResult? result =
            OrbitManeuvers.ComputeMatchInclination(vehicle, target, false, now, HalfFraction);
        if (!ManeuverTestSupport.RequireResult(Name, "half fraction", result))
            return false;

        bool ok = ManeuverTestSupport.CheckResultShape(Name, "half fraction", vehicle, result!.Value, now);
        Orbit after = ManeuverTestSupport.Apply(vehicle, result.Value);
        double relIncAfter = after.GetRelativeInclination(target).Value();
        ok &= ManeuverTestSupport.NearRel(relIncAfter, relInc0 * HalfFraction, FractionRelTol);
        HarnessLog.Line($"[{Name}] TEST half fraction: relInc {relInc0:E3} -> {relIncAfter:E3}rad " +
                        $"(expect {relInc0 * HalfFraction:E3}) => {TestSupport.Verdict(ok)}");
        return ok;
    }
}

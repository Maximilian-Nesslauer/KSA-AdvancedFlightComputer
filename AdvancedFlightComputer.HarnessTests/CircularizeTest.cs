using AdvancedFlightComputer.Features.ManeuverTools;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates OrbitManeuvers.ComputeCircularize at both apses: the applied burn must produce a
// near-circular orbit at the burn radius, and an already-circular or unbound orbit must yield no
// maneuver (the tool's "nothing to do" contract).
public sealed class CircularizeTest : IHarnessTest
{
    // Test orbit, in meters above the home body's mean radius.
    private const double PeriapsisAltitudeM = 400_000.0;
    private const double ApoapsisAltitudeM = 2_400_000.0;
    private const double CircularAltitudeM = 500_000.0;

    private const double EccentricityTol = 1e-3;

    public string Name => "afc-circularize";

    public int Run(HeadlessSession session)
    {
        if (!ManeuverTestSupport.RequireHome(Name, session, out IParentBody home))
            return 1;

        SimTime now = Universe.GetElapsedSimTime();
        Orbit orbit = VehicleSpawner.EllipticalCci(
            home, home.MeanRadius + PeriapsisAltitudeM, home.MeanRadius + ApoapsisAltitudeM, now);

        bool ok = true;
        ok &= CheckCircularizes(orbit, useApoapsis: true, orbit.Apoapsis, "at Ap", now);
        ok &= CheckCircularizes(orbit, useApoapsis: false, orbit.Periapsis, "at Pe", now);
        ok &= ManeuverTestSupport.CheckNull(Name, "already circular",
            OrbitManeuvers.ComputeCircularize(
                VehicleSpawner.CircularCci(home, home.MeanRadius + CircularAltitudeM, now), true, now));
        ok &= ManeuverTestSupport.CheckNull(Name, "hyperbolic orbit",
            OrbitManeuvers.ComputeCircularize(
                ManeuverTestSupport.HyperbolicCci(home, home.MeanRadius + PeriapsisAltitudeM, now), true, now));

        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private bool CheckCircularizes(Orbit orbit, bool useApoapsis, double burnRadius, string label, SimTime now)
    {
        OrbitManeuvers.ManeuverResult? result = OrbitManeuvers.ComputeCircularize(orbit, useApoapsis, now);
        if (!ManeuverTestSupport.RequireResult(Name, label, result))
            return false;

        bool ok = ManeuverTestSupport.CheckResultShape(Name, label, orbit, result!.Value, now);
        Orbit after = ManeuverTestSupport.Apply(orbit, result.Value);
        ok &= after.Eccentricity < EccentricityTol;
        ok &= ManeuverTestSupport.NearRel(after.SemiMajorAxis, burnRadius);
        HarnessLog.Line($"[{Name}] TEST {label}: dv={result.Value.DvCci.Length():F2}m/s " +
                        $"ecc={after.Eccentricity:F5} SMA={after.SemiMajorAxis:E6} " +
                        $"(target {burnRadius:E6}) => {TestSupport.Verdict(ok)}");
        return ok;
    }
}

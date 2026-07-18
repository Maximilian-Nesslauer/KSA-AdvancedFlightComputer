using AdvancedFlightComputer.Features.ManeuverTools;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates OrbitManeuvers.ComputeSetPeriapsis: a single burn at apoapsis must move the periapsis
// to the requested altitude and leave the apoapsis where it was, and impossible requests (target
// at or above the apoapsis, unbound orbit) must yield no maneuver.
public sealed class SetPeriapsisTest : IHarnessTest
{
    // Test orbit and targets, in meters above the home body's mean radius.
    private const double PeriapsisAltitudeM = 300_000.0;
    private const double ApoapsisAltitudeM = 2_000_000.0;
    private const double RaiseTargetAltitudeM = 800_000.0;
    private const double LowerTargetAltitudeM = 150_000.0;

    public string Name => "afc-set-periapsis";

    public int Run(HeadlessSession session)
    {
        if (!ManeuverTestSupport.RequireHome(Name, session, out IParentBody home))
            return 1;

        SimTime now = Universe.GetElapsedSimTime();
        Orbit orbit = VehicleSpawner.EllipticalCci(
            home, home.MeanRadius + PeriapsisAltitudeM, home.MeanRadius + ApoapsisAltitudeM, now);

        bool ok = true;
        ok &= CheckReaches(orbit, home, RaiseTargetAltitudeM, "raise Pe", now);
        ok &= CheckReaches(orbit, home, LowerTargetAltitudeM, "lower Pe", now);
        // Clearly above the apoapsis: exact equality is a floating-point knife edge on
        // Orbit.Apoapsis, and the plan window blocks targets within 1 km of the opposite apse
        // anyway.
        ok &= ManeuverTestSupport.CheckNull(Name, "target above Ap",
            OrbitManeuvers.ComputeSetPeriapsis(orbit, ApoapsisAltitudeM + 100_000.0, home.MeanRadius, now));
        ok &= ManeuverTestSupport.CheckNull(Name, "hyperbolic orbit",
            OrbitManeuvers.ComputeSetPeriapsis(
                ManeuverTestSupport.HyperbolicCci(home, home.MeanRadius + PeriapsisAltitudeM, now),
                RaiseTargetAltitudeM, home.MeanRadius, now));

        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private bool CheckReaches(Orbit orbit, IParentBody home, double targetAltitudeM, string label, SimTime now)
    {
        OrbitManeuvers.ManeuverResult? result =
            OrbitManeuvers.ComputeSetPeriapsis(orbit, targetAltitudeM, home.MeanRadius, now);
        if (!ManeuverTestSupport.RequireResult(Name, label, result))
            return false;

        bool ok = ManeuverTestSupport.CheckResultShape(Name, label, orbit, result!.Value, now);
        Orbit after = ManeuverTestSupport.Apply(orbit, result.Value);
        double targetRadius = home.MeanRadius + targetAltitudeM;
        ok &= ManeuverTestSupport.NearRel(after.Periapsis, targetRadius);
        ok &= ManeuverTestSupport.NearRel(after.Apoapsis, orbit.Apoapsis);
        HarnessLog.Line($"[{Name}] TEST {label}: dv={result.Value.DvCci.Length():F2}m/s " +
                        $"Pe={after.Periapsis:E6} (target {targetRadius:E6}) " +
                        $"Ap={after.Apoapsis:E6} (was {orbit.Apoapsis:E6}) => {TestSupport.Verdict(ok)}");
        return ok;
    }
}

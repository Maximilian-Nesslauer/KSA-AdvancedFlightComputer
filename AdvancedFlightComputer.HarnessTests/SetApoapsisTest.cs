using AdvancedFlightComputer.Features.ManeuverTools;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates OrbitManeuvers.ComputeSetApoapsis: a single burn at periapsis must move the apoapsis
// to the requested altitude and leave the periapsis where it was, and impossible requests (target
// at or below the periapsis, unbound orbit) must yield no maneuver.
public sealed class SetApoapsisTest : IHarnessTest
{
    // Test orbit and targets, in meters above the home body's mean radius.
    private const double PeriapsisAltitudeM = 300_000.0;
    private const double ApoapsisAltitudeM = 2_000_000.0;
    private const double RaiseTargetAltitudeM = 5_000_000.0;
    private const double LowerTargetAltitudeM = 1_000_000.0;

    public string Name => "afc-set-apoapsis";

    public int Run(HeadlessSession session)
    {
        if (!ManeuverTestSupport.RequireHome(Name, session, out IParentBody home))
            return 1;

        SimTime now = Universe.GetElapsedSimTime();
        Orbit orbit = VehicleSpawner.EllipticalCci(
            home, home.MeanRadius + PeriapsisAltitudeM, home.MeanRadius + ApoapsisAltitudeM, now);

        bool ok = true;
        ok &= CheckReaches(orbit, home, RaiseTargetAltitudeM, "raise Ap", now);
        ok &= CheckReaches(orbit, home, LowerTargetAltitudeM, "lower Ap", now);
        ok &= ManeuverTestSupport.CheckNull(Name, "target below Pe",
            OrbitManeuvers.ComputeSetApoapsis(orbit, PeriapsisAltitudeM / 2.0, home.MeanRadius, now));
        ok &= ManeuverTestSupport.CheckNull(Name, "hyperbolic orbit",
            OrbitManeuvers.ComputeSetApoapsis(
                ManeuverTestSupport.HyperbolicCci(home, home.MeanRadius + PeriapsisAltitudeM, now),
                RaiseTargetAltitudeM, home.MeanRadius, now));

        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private bool CheckReaches(Orbit orbit, IParentBody home, double targetAltitudeM, string label, SimTime now)
    {
        OrbitManeuvers.ManeuverResult? result =
            OrbitManeuvers.ComputeSetApoapsis(orbit, targetAltitudeM, home.MeanRadius, now);
        if (!ManeuverTestSupport.RequireResult(Name, label, result))
            return false;

        bool ok = ManeuverTestSupport.CheckResultShape(Name, label, orbit, result!.Value, now);
        Orbit after = ManeuverTestSupport.Apply(orbit, result.Value);
        double targetRadius = home.MeanRadius + targetAltitudeM;
        ok &= ManeuverTestSupport.NearRel(after.Apoapsis, targetRadius);
        ok &= ManeuverTestSupport.NearRel(after.Periapsis, orbit.Periapsis);
        HarnessLog.Line($"[{Name}] TEST {label}: dv={result.Value.DvCci.Length():F2}m/s " +
                        $"Ap={after.Apoapsis:E6} (target {targetRadius:E6}) " +
                        $"Pe={after.Periapsis:E6} (was {orbit.Periapsis:E6}) => {TestSupport.Verdict(ok)}");
        return ok;
    }
}

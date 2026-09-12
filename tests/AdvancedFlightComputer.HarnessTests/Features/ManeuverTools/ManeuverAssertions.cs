using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

internal static class ManeuverAssertions
{
    // This relative tolerance applies to the radii and semimajor axes reached after a maneuver.
    public const double RelTol = 1e-3;

    // Absolute tolerance in radians for reached inclinations (about 0.06 deg, the same bar the maneuver computations use to report "nothing to do").
    public const double IncTolRad = 1e-3;

    // Allow for numerical rounding when a frame conversion is reversed or a plane change preserves speed.
    public const double TransformRelTol = 1e-9;

    public static bool ResultShapeHolds(
        TestContext t, string label, Orbit orbit, in OrbitManeuvers.ManeuverResult maneuver, UniverseTime now)
    {
        doubleQuat vlf2Cci = orbit.GetStateVectorsAt(maneuver.BurnTime).GetVlf2ParentCci().OrIdentity();
        double roundTrip = (maneuver.DvVlf.Transform(vlf2Cci) - maneuver.DvCci).Length();
        double scale = Math.Max(1.0, maneuver.DvCci.Length());
        bool vlfOk = roundTrip / scale < TransformRelTol;
        bool timeOk = maneuver.BurnTime.Seconds() >= now.Seconds();
        // Report transform and time failures separately from the orbital tolerance checks.
        if (!vlfOk)
            t.Info($"{label}: SHAPE VIOLATION: DvVlf->CCI round-trip error {roundTrip:E3}m/s.");
        if (!timeOk)
            t.Info($"{label}: SHAPE VIOLATION: burn time {maneuver.BurnTime.Seconds():F1}s " +
                   $"is before now {now.Seconds():F1}s.");
        return vlfOk && timeOk;
    }

    public static bool CheckNone(TestContext t, string label, OrbitManeuvers.ManeuverResult? result)
        => t.Check(label, result == null,
            result == null
                ? "expect no maneuver"
                : $"expect no maneuver, got dv={result.Value.DvCci.Length():F3}m/s " +
                  $"at t={result.Value.BurnTime.Seconds():F1}s");

    public static bool RequireResult(TestContext t, string label, OrbitManeuvers.ManeuverResult? result)
        => result != null || t.Fail(label, "no maneuver computed");

    public static Orbit Apply(Orbit orbit, in OrbitManeuvers.ManeuverResult maneuver)
        => OrbitFixtures.ApplyImpulse(orbit, maneuver.DvCci, maneuver.BurnTime);
}

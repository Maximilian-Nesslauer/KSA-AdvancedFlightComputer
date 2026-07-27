using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// What every ManeuverResult has to satisfy whichever quick-tool produced it, plus the tolerances the
// maneuver tests share.
internal static class ManeuverAssertions
{
    // Relative tolerance for radii and semi-major axes reached after an applied maneuver.
    public const double RelTol = 1e-3;

    // Absolute tolerance in radians for reached inclinations (about 0.06 deg, the same bar the
    // maneuver computations use to report "nothing to do").
    public const double IncTolRad = 1e-3;

    // Pure floating-point headroom for exact invariants (frame-transform round trips, the speed a
    // plane-change rotation preserves).
    public const double TransformRelTol = 1e-9;

    // The burn is not in the past, and DvVlf is DvCci in the VLF frame at the burn point (the
    // transform the mod uses to fill the stock transfer UI, via StateVectors.GetVlf2ParentCci).
    public static bool ResultShapeHolds(
        TestContext t, string label, Orbit orbit, in OrbitManeuvers.ManeuverResult maneuver, SimTime now)
    {
        doubleQuat vlf2Cci = orbit.GetStateVectorsAt(maneuver.BurnTime).GetVlf2ParentCci().OrIdentity();
        double roundTrip = (maneuver.DvVlf.Transform(vlf2Cci) - maneuver.DvCci).Length();
        double scale = Math.Max(1.0, maneuver.DvCci.Length());
        bool vlfOk = roundTrip / scale < TransformRelTol;
        bool timeOk = maneuver.BurnTime.Seconds() >= now.Seconds();
        // When only the shape breaks, every number on the subcase's FAIL line is inside tolerance,
        // so this is the only line that says why.
        if (!vlfOk)
            t.Info($"{label}: SHAPE VIOLATION: DvVlf->CCI round-trip error {roundTrip:E3}m/s.");
        if (!timeOk)
            t.Info($"{label}: SHAPE VIOLATION: burn time {maneuver.BurnTime.Seconds():F1}s " +
                   $"is before now {now.Seconds():F1}s.");
        return vlfOk && timeOk;
    }

    // An input that is invalid or already satisfied, so the tool must return nothing.
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

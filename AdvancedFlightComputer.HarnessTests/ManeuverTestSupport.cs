using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.Numerics;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Shared pieces for the OrbitManeuvers tests: orbit builders, impulsive application of a computed
// maneuver, and the invariants every ManeuverResult must satisfy regardless of tool. The oracle is
// always the game's own orbit code (Orbit.GetStateVectorsAt / Orbit.CreateFromStateCci), never a
// re-derivation of the math under test.
internal static class ManeuverTestSupport
{
    // Relative tolerance for radii/SMA reached after an applied maneuver.
    public const double RelTol = 1e-3;

    // Absolute tolerance in radians for reached inclinations (about 0.06 deg, the same bar the
    // maneuver computations use to report "nothing to do").
    public const double IncTolRad = 1e-3;

    // Pure floating-point headroom for exact invariants (frame-transform round trips, the speed a
    // plane-change rotation preserves).
    public const double TransformRelTol = 1e-9;

    public static bool NearRel(double actual, double expected, double relTol = RelTol)
        => Math.Abs(actual - expected) <= relTol * Math.Abs(expected);

    public static double Deg(double radians) => radians * (180.0 / Math.PI);

    // An elliptical orbit inclined against the ecliptic with its ascending node on CCI +X. The
    // argument of periapsis places the periapsis within the plane, measured from the node; the
    // default 0 puts the periapsis on the node itself. Equal radii give the circular case.
    public static Orbit InclinedEllipticalCci(
        IParentBody parent, double periapsisRadius, double apoapsisRadius, double inclinationRad,
        SimTime time, double argumentOfPeriapsisRad = 0.0)
    {
        double a = (periapsisRadius + apoapsisRadius) / 2.0;
        double vPe = Math.Sqrt(parent.Mu * (2.0 / periapsisRadius - 1.0 / a));
        double3 inPlaneX = double3.UnitX;
        double3 inPlaneY = new double3(0.0, Math.Cos(inclinationRad), Math.Sin(inclinationRad));
        double cosArg = Math.Cos(argumentOfPeriapsisRad);
        double sinArg = Math.Sin(argumentOfPeriapsisRad);
        double3 position = (inPlaneX * cosArg + inPlaneY * sinArg) * periapsisRadius;
        double3 velocity = (inPlaneX * -sinArg + inPlaneY * cosArg) * vPe;
        return Orbit.CreateFromStateCci(parent, time, position, velocity, VehicleSpawner.OrbitLineColor);
    }

    // An unbound planar orbit: 1.2x escape speed at periapsis.
    public static Orbit HyperbolicCci(IParentBody parent, double periapsisRadius, SimTime time)
    {
        double v = 1.2 * Math.Sqrt(2.0 * parent.Mu / periapsisRadius);
        return Orbit.CreateFromStateCci(
            parent, time, new double3(periapsisRadius, 0.0, 0.0), new double3(0.0, v, 0.0),
            VehicleSpawner.OrbitLineColor);
    }

    // Applies the maneuver's impulse to the orbit state at the burn time and returns the post-burn
    // orbit the assertions read.
    public static Orbit Apply(Orbit orbit, in OrbitManeuvers.ManeuverResult maneuver)
    {
        StateVectors sv = orbit.GetStateVectorsAt(maneuver.BurnTime);
        return Orbit.CreateFromStateCci(
            orbit.Parent, maneuver.BurnTime, sv.PositionCci, sv.VelocityCci + maneuver.DvCci,
            VehicleSpawner.OrbitLineColor);
    }

    // Invariants every ManeuverResult must satisfy: the burn is not scheduled in the past, and
    // DvVlf is DvCci expressed in the VLF frame at the burn point (the transform the mod uses to
    // fill the stock transfer UI, via StateVectors.GetVlf2ParentCci).
    public static bool CheckResultShape(
        string test, string label, Orbit orbit, in OrbitManeuvers.ManeuverResult maneuver, SimTime now)
    {
        doubleQuat vlf2Cci = orbit.GetStateVectorsAt(maneuver.BurnTime).GetVlf2ParentCci().OrIdentity();
        double roundTrip = (maneuver.DvVlf.Transform(vlf2Cci) - maneuver.DvCci).Length();
        double scale = Math.Max(1.0, maneuver.DvCci.Length());
        bool vlfOk = roundTrip / scale < TransformRelTol;
        bool timeOk = maneuver.BurnTime.Seconds() >= now.Seconds();
        if (!vlfOk)
            HarnessLog.Line($"[{test}] FAIL ({label}): DvVlf->CCI round-trip error {roundTrip:E3}m/s.");
        if (!timeOk)
            HarnessLog.Line($"[{test}] FAIL ({label}): burn time {maneuver.BurnTime.Seconds():F1}s " +
                            $"is before now {now.Seconds():F1}s.");
        return vlfOk && timeOk;
    }

    // A subcheck whose input must produce no maneuver (invalid or already satisfied).
    public static bool CheckNull(string test, string label, OrbitManeuvers.ManeuverResult? result)
    {
        if (result == null)
        {
            HarnessLog.Line($"[{test}] TEST {label}: expect no maneuver => PASS");
            return true;
        }
        HarnessLog.Line($"[{test}] TEST {label}: expect no maneuver, got dv={result.Value.DvCci.Length():F3}m/s " +
                        $"at t={result.Value.BurnTime.Seconds():F1}s => FAIL");
        return false;
    }

    // The inverse of CheckNull: a subcheck whose input must produce a maneuver to assert on.
    public static bool RequireResult(string test, string label, OrbitManeuvers.ManeuverResult? result)
    {
        if (result != null)
            return true;
        HarnessLog.Line($"[{test}] TEST {label}: no maneuver computed => FAIL");
        return false;
    }

    public static bool RequireHome(string test, HeadlessSession session, out IParentBody home)
    {
        if (session.System.HomeBody is IParentBody h)
        {
            home = h;
            return true;
        }
        home = null!;
        HarnessLog.Line($"[{test}] FAIL: the loaded system has no home body.");
        return false;
    }
}

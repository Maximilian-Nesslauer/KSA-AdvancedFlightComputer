using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.Numerics;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates OrbitManeuvers.ComputeSetInclination: a node burn must set the inclination against the
// chosen reference plane while keeping the orbital speed and the node line, the coplanar special
// case must fall back to the CCI +X node convention, and already-satisfied or unbound inputs must
// yield no maneuver. The ecliptic checks read stock Orbit.Inclination (the classical element
// against the CCI Z plane) as an oracle independent of the reference-normal path under test.
public sealed class SetInclinationTest : IHarnessTest
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

    public string Name => "afc-set-inclination";

    public int Run(HeadlessSession session)
    {
        if (!ManeuverTestSupport.RequireHome(Name, session, out IParentBody home))
            return 1;

        SimTime now = Universe.GetElapsedSimTime();
        Orbit orbit = ManeuverTestSupport.InclinedEllipticalCci(
            home, home.MeanRadius + PeriapsisAltitudeM, home.MeanRadius + ApoapsisAltitudeM,
            StartInclinationRad, now);
        Orbit offApse = ManeuverTestSupport.InclinedEllipticalCci(
            home, home.MeanRadius + PeriapsisAltitudeM, home.MeanRadius + ApoapsisAltitudeM,
            StartInclinationRad, now, OffApseArgumentOfPeriapsisRad);

        bool ok = true;
        ok &= CheckEcliptic(orbit, RaiseTargetRad, useDescendingNode: false, "raise (ecliptic, AN)", now);
        ok &= CheckEcliptic(orbit, RaiseTargetRad, useDescendingNode: true, "raise (ecliptic, DN)", now);
        ok &= CheckEcliptic(orbit, LowerTargetRad, useDescendingNode: false, "lower (ecliptic, AN)", now);
        ok &= CheckEcliptic(offApse, RaiseTargetRad, useDescendingNode: false, "raise (ecliptic, AN, off-apse Pe)", now);
        ok &= CheckHalfFraction(orbit, now);
        ok &= CheckEquatorial(orbit, home, now);
        ok &= CheckCoplanarStart(home, now);
        ok &= ManeuverTestSupport.CheckNull(Name, "already at target",
            OrbitManeuvers.ComputeSetInclination(
                orbit,
                OrbitManeuvers.GetInclinationAgainst(orbit, OrbitManeuvers.InclinationReference.Ecliptic),
                false, now, OrbitManeuvers.InclinationReference.Ecliptic));
        ok &= ManeuverTestSupport.CheckNull(Name, "hyperbolic orbit",
            OrbitManeuvers.ComputeSetInclination(
                ManeuverTestSupport.HyperbolicCci(home, home.MeanRadius + PeriapsisAltitudeM, now),
                RaiseTargetRad, false, now, OrbitManeuvers.InclinationReference.Ecliptic));

        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private bool CheckEcliptic(Orbit orbit, double targetRad, bool useDescendingNode, string label, SimTime now)
    {
        OrbitManeuvers.ManeuverResult? result = OrbitManeuvers.ComputeSetInclination(
            orbit, targetRad, useDescendingNode, now, OrbitManeuvers.InclinationReference.Ecliptic);
        if (!ManeuverTestSupport.RequireResult(Name, label, result))
            return false;

        bool ok = ManeuverTestSupport.CheckResultShape(Name, label, orbit, result!.Value, now);
        StateVectors sv = orbit.GetStateVectorsAt(result.Value.BurnTime);
        ok &= ManeuverTestSupport.NearRel(
            (sv.VelocityCci + result.Value.DvCci).Length(), sv.VelocityCci.Length(),
            ManeuverTestSupport.TransformRelTol);

        Orbit after = ManeuverTestSupport.Apply(orbit, result.Value);
        ok &= Math.Abs(after.Inclination - targetRad) < ManeuverTestSupport.IncTolRad;
        ok &= NodeLinePreserved(orbit, after);
        HarnessLog.Line($"[{Name}] TEST {label}: dv={result.Value.DvCci.Length():F2}m/s " +
                        $"inc={ManeuverTestSupport.Deg(after.Inclination):F3}deg " +
                        $"(target {ManeuverTestSupport.Deg(targetRad):F3}deg) => {TestSupport.Verdict(ok)}");
        return ok;
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

    private bool CheckHalfFraction(Orbit orbit, SimTime now)
    {
        OrbitManeuvers.ManeuverResult? result = OrbitManeuvers.ComputeSetInclination(
            orbit, RaiseTargetRad, false, now, OrbitManeuvers.InclinationReference.Ecliptic, HalfFraction);
        if (!ManeuverTestSupport.RequireResult(Name, "half fraction", result))
            return false;

        bool ok = ManeuverTestSupport.CheckResultShape(Name, "half fraction", orbit, result!.Value, now);
        Orbit after = ManeuverTestSupport.Apply(orbit, result.Value);
        double expected = StartInclinationRad + (RaiseTargetRad - StartInclinationRad) * HalfFraction;
        ok &= ManeuverTestSupport.NearRel(after.Inclination, expected, FractionRelTol);
        HarnessLog.Line($"[{Name}] TEST half fraction: inc={ManeuverTestSupport.Deg(after.Inclination):F3}deg " +
                        $"(expect {ManeuverTestSupport.Deg(expected):F3}deg) => {TestSupport.Verdict(ok)}");
        return ok;
    }

    private bool CheckEquatorial(Orbit orbit, IParentBody home, SimTime now)
    {
        OrbitManeuvers.ManeuverResult? result = OrbitManeuvers.ComputeSetInclination(
            orbit, EquatorialTargetRad, false, now, OrbitManeuvers.InclinationReference.Equatorial);
        if (!ManeuverTestSupport.RequireResult(Name, "equatorial", result))
            return false;

        bool ok = ManeuverTestSupport.CheckResultShape(Name, "equatorial", orbit, result!.Value, now);
        Orbit after = ManeuverTestSupport.Apply(orbit, result.Value);
        // Measure against the equator directly from the body's rotation axis (the CCE Z axis in
        // CCI, via IParentBody.GetCce2Cci), not through the GetInclinationAgainst path under test.
        double3 equatorNormal = double3.UnitZ.Transform(home.GetCce2Cci());
        double incAfter = MathEx.SafeAcos(double3.Dot(equatorNormal, after.GetOrbitNormalCci()));
        double obliquity = MathEx.SafeAcos(double3.Dot(equatorNormal, double3.UnitZ));
        ok &= Math.Abs(incAfter - EquatorialTargetRad) < ManeuverTestSupport.IncTolRad;
        HarnessLog.Line($"[{Name}] TEST equatorial: dv={result.Value.DvCci.Length():F2}m/s " +
                        $"inc={ManeuverTestSupport.Deg(incAfter):F3}deg " +
                        $"(target {ManeuverTestSupport.Deg(EquatorialTargetRad):F3}deg, " +
                        $"obliquity {ManeuverTestSupport.Deg(obliquity):F2}deg) => {TestSupport.Verdict(ok)}");
        return ok;
    }

    private bool CheckCoplanarStart(IParentBody home, SimTime now)
    {
        // An orbit in the ecliptic plane has no defined node against the ecliptic; the tool picks
        // CCI +X by convention. The set must still work and must put the node line on X.
        Orbit planar = VehicleSpawner.EllipticalCci(
            home, home.MeanRadius + PeriapsisAltitudeM, home.MeanRadius + ApoapsisAltitudeM, now);
        OrbitManeuvers.ManeuverResult? result = OrbitManeuvers.ComputeSetInclination(
            planar, CoplanarTargetRad, false, now, OrbitManeuvers.InclinationReference.Ecliptic);
        if (!ManeuverTestSupport.RequireResult(Name, "coplanar start", result))
            return false;

        bool ok = ManeuverTestSupport.CheckResultShape(Name, "coplanar start", planar, result!.Value, now);
        Orbit after = ManeuverTestSupport.Apply(planar, result.Value);
        ok &= Math.Abs(after.Inclination - CoplanarTargetRad) < ManeuverTestSupport.IncTolRad;
        double3 nodeLine = double3.Cross(double3.UnitZ, after.GetOrbitNormalCci()).NormalizeOrZero();
        bool nodeOk = Math.Abs(double3.Dot(nodeLine, double3.UnitX)) > NodeLineDotMin;
        ok &= nodeOk;
        HarnessLog.Line($"[{Name}] TEST coplanar start: inc={ManeuverTestSupport.Deg(after.Inclination):F3}deg " +
                        $"(target {ManeuverTestSupport.Deg(CoplanarTargetRad):F3}deg) nodeLineOnX={nodeOk} " +
                        $"=> {TestSupport.Verdict(ok)}");
        return ok;
    }
}

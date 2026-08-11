using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests.Fixtures;

// Test orbits, built through the game's own orbit code rather than a re-derivation. Altitudes are
// meters above the parent's mean radius; VehicleSpawner carries the radius-based builders.
public static class OrbitFixtures
{
    public static Orbit CircularAt(IParentBody parent, double altitudeM, UniverseTime time)
        => VehicleSpawner.CircularCci(parent, parent.MeanRadius + altitudeM, time);

    public static Orbit EllipticalAt(
        IParentBody parent, double periapsisAltitudeM, double apoapsisAltitudeM, UniverseTime time)
        => VehicleSpawner.EllipticalCci(
            parent, parent.MeanRadius + periapsisAltitudeM, parent.MeanRadius + apoapsisAltitudeM, time);

    // Ascending node on CCI +X. The argument of periapsis is measured from that node, so the default
    // 0 puts the periapsis on the node itself. Equal altitudes give the circular case.
    public static Orbit InclinedEllipticalAt(
        IParentBody parent, double periapsisAltitudeM, double apoapsisAltitudeM, double inclinationRad,
        UniverseTime time, double argumentOfPeriapsisRad = 0.0)
    {
        double periapsisRadius = parent.MeanRadius + periapsisAltitudeM;
        double apoapsisRadius = parent.MeanRadius + apoapsisAltitudeM;
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

    // Unbound and planar: 1.2x escape speed at periapsis.
    public static Orbit HyperbolicAt(IParentBody parent, double periapsisAltitudeM, UniverseTime time)
    {
        double periapsisRadius = parent.MeanRadius + periapsisAltitudeM;
        double v = 1.2 * Math.Sqrt(2.0 * parent.Mu / periapsisRadius);
        return Orbit.CreateFromStateCci(
            parent, time, new double3(periapsisRadius, 0.0, 0.0), new double3(0.0, v, 0.0),
            VehicleSpawner.OrbitLineColor);
    }

    public static Orbit ApplyImpulse(Orbit orbit, double3 dvCci, UniverseTime at)
    {
        StateVectors sv = orbit.GetStateVectorsAt(at);
        return Orbit.CreateFromStateCci(
            orbit.Parent, at, sv.PositionCci, sv.VelocityCci + dvCci, VehicleSpawner.OrbitLineColor);
    }

    public static double Deg(double radians) => radians * (180.0 / Math.PI);
}

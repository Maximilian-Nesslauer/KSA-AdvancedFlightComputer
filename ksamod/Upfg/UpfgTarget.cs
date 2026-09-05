using System;
using Brutal.Numerics;

namespace PoweredGuidance.Upfg;

// The desired insertion orbit, expressed the way UPFG needs it: a target radius,
// speed, flight-path angle and orbital-plane normal in the inertial (CCI) frame.
//
// Ported explicitly from navbox's UPFGTarget.Set (the orbital-ascent case). Inputs
// are taken from the UI in km / degrees; the body radius and gravitational parameter
// come from the live KSA celestial body rather than hard-coded Earth constants.
public sealed class UpfgTarget
{
    public double Radius;     // insertion radius (m, from body centre)
    public double Velocity;   // insertion speed (m/s)
    public double Fpa;        // flight-path angle at insertion (rad)
    public double3 Normal;    // unit orbital-plane normal in CCI
    public double Inclination;// rad
    public double Lan;        // rad
    public double Pe;         // periapsis radius (m)
    public double Ap;         // apoapsis radius (m)
    public double Ecc;
    public double3 Rdes;      // desired cutoff/landing position, CCI (modes 2/3)
    public double DescentRate; // desired downward speed at Rdes, m/s (mode 3)

    // peKm / apKm are altitudes above the surface; incDeg / lanDeg define the plane.
    public static UpfgTarget FromOrbit(double peKm, double apKm, double incDeg, double lanDeg,
                                       double bodyRadius, double mu)
    {
        var t = new UpfgTarget();

        double pe = peKm * 1000.0 + bodyRadius;
        double ap = apKm * 1000.0 + bodyRadius;
        if (ap < pe) (ap, pe) = (pe, ap);

        t.Pe = pe;
        t.Ap = ap;
        t.Ecc = (ap - pe) / (ap + pe);

        // Insert at periapsis (flight-path angle ~0 there).
        t.Radius = pe;

        double sma = (pe + ap) / 2.0;
        double vpe = Math.Sqrt(mu * (2.0 / pe - 1.0 / sma));
        t.Velocity = Math.Sqrt(mu * (2.0 / t.Radius - 1.0 / sma));
        double srm = pe * vpe;                                  // specific angular momentum at pe
        t.Fpa = Math.Acos(Math.Clamp(srm / (t.Velocity * t.Radius), -1.0, 1.0));

        t.Inclination = DegToRad(incDeg);
        t.Lan = DegToRad(lanDeg);
        t.Normal = OrbitNormal(t.Inclination, t.Lan);

        return t;
    }

    // navbox Utils.CalcOrbitNormal: plane normal from inclination and longitude of
    // ascending node, with Z as the celestial polar axis (matches CCI).
    public static double3 OrbitNormal(double inc, double lan)
    {
        return new double3(
            Math.Sin(inc) * Math.Sin(lan),
            -Math.Sin(inc) * Math.Cos(lan),
            Math.Cos(inc));
    }

    public static double DegToRad(double d) => d * Math.PI / 180.0;
    public static double RadToDeg(double r) => r * 180.0 / Math.PI;
}

using System;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using Brutal.Numerics;

namespace AdvancedFlightComputer.Features.MissionPlanner;

/// <summary>
/// The plane geometry of a launch to meet a moon, for an idealised Hohmann transfer: the injection burn is made at the point of the parking orbit opposite where the moon will be at arrival, so the transfer's apoapsis is the moon.
/// The parking orbit's plane therefore has to contain the moon's position at arrival, n . r_moon = 0.
/// That is one equation in the plane's two angles, so one of them is free: fix the inclination and two nodes solve it.
///  Everything is in the parent body's CCI frame, Z along its spin axis, with the plane normal convention of <see cref="UpfgTarget.OrbitNormal"/>: n = (sin i sin LAN, -sin i cos LAN, cos i).
/// </summary>
internal static class LunarTransferGeometry
{
    /// <summary>Time of flight of the Hohmann half-ellipse between radii <paramref name="r1"/> and <paramref name="r2"/>, s.</summary>
    public static double HohmannTime(double mu, double r1, double r2)
    {
        double a = 0.5 * (r1 + r2);
        return Math.PI * Math.Sqrt(a * a * a / mu);
    }

    /// <summary>Declination of a unit direction, rad.</summary>
    public static double Declination(double3 u) => Math.Asin(Math.Clamp(u.Z, -1.0, 1.0));

    /// <summary>
    /// The nodes of the two planes with inclination <paramref name="inc"/> that contain the unit direction <paramref name="u"/>, rad in [0, 2pi).
    /// n . u = u.x sin i sin W - u.y sin i cos W + u.z cos i = 0 is A sin W + B cos W = C with A = u.x sin i, B = -u.y sin i and C = -u.z cos i.
    /// The left side is one sinusoid, R sin(W + phi), with R the length of (A, B) and phi its angle, so W = asin(C / R) - phi or pi - asin(C / R) - phi.
    /// |C| &lt;= R says the plane has to be inclined at least as far as u's declination, and false means it is not.
    ///  The two planes differ in which half of the orbit meets u: the northbound one reaches it heading north, within 90 degrees of the ascending node, and the southbound one heading south.
    /// At the least inclination that reaches u they are the same plane.
    /// </summary>
    public static bool TryNodesForInclination(double3 u, double inc, out double lanNorthbound, out double lanSouthbound)
    {
        lanNorthbound = lanSouthbound = double.NaN;
        double sinI = Math.Sin(inc), cosI = Math.Cos(inc);
        double a = u.X * sinI, b = -u.Y * sinI, c = -u.Z * cosI;
        double r = Math.Sqrt(a * a + b * b);

        // An equatorial plane, or u along the pole: every node or none. Every node only when the plane contains u already, and then the node is put under u.
        if (r < 1e-12)
        {
            if (Math.Abs(c) > 1e-9)
                return false;
            lanNorthbound = lanSouthbound = UpfgTarget.WrapTwoPi(Math.Atan2(u.Y, u.X));
            return true;
        }

        double s = c / r;
        if (Math.Abs(s) > 1.0 + 1e-12)
            return false;
        double root = Math.Asin(Math.Clamp(s, -1.0, 1.0));
        double phi = Math.Atan2(b, a);
        double first = UpfgTarget.WrapTwoPi(root - phi);
        double second = UpfgTarget.WrapTwoPi(Math.PI - root - phi);

        // u . node is the cosine of u's argument of latitude, positive on the northbound half.
        bool firstNorthbound = double3.Dot(u, UpfgTarget.NodeDirection(first))
                               >= double3.Dot(u, UpfgTarget.NodeDirection(second));
        lanNorthbound = firstNorthbound ? first : second;
        lanSouthbound = firstNorthbound ? second : first;
        return true;
    }
}

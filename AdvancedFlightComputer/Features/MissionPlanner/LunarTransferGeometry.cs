using System;

namespace AdvancedFlightComputer.Features.MissionPlanner;

/// <summary>
/// The geometry of a transfer to a moon, for an idealised Hohmann transfer.
/// </summary>
internal static class LunarTransferGeometry
{
    /// <summary>Time of flight of the Hohmann half-ellipse between radii <paramref name="r1"/> and <paramref name="r2"/>, s.</summary>
    public static double HohmannTime(double mu, double r1, double r2)
    {
        double a = 0.5 * (r1 + r2);
        return Math.PI * Math.Sqrt(a * a * a / mu);
    }
}

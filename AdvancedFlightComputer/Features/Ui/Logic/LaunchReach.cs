using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>
/// What a launch from a site can reach without a plane change, and what a target plane costs beyond that. A launch
/// from latitude lat flies only planes with |lat| &lt;= inc &lt;= 180 - |lat|, because the site must lie in the plane.
/// A plane out of that band needs a plane change, which an ascent guidance flies as a dog-leg or a turn after
/// insertion. Its price is taken as one impulsive turn of the velocity at the apoapsis of the target orbit,
/// 2 v sin(di / 2), which is the cheapest single-burn plane change on that orbit. Equatorial frame, degrees.
/// </summary>
internal static class LaunchReach
{
    /// <summary>A plane change below this is noise of the plane seeding and is not reported.</summary>
    public const double PlaneChangeShownDeg = 0.5;

    /// <summary>The lowest inclination a direct launch from this latitude reaches: the site lies at the top of the plane.</summary>
    public static double LowestDirectIncDeg(double siteLatDeg) => Math.Min(Math.Abs(siteLatDeg), 90.0);

    /// <summary>The highest (retrograde) inclination a direct launch from this latitude reaches.</summary>
    public static double HighestDirectIncDeg(double siteLatDeg) => 180.0 - LowestDirectIncDeg(siteLatDeg);

    /// <summary>The plane change no launch time avoids, for an inclination whose node is free to choose.</summary>
    public static double LeastPlaneChangeDeg(double siteLatDeg, double incDeg)
    {
        double low = LowestDirectIncDeg(siteLatDeg);
        return Math.Max(0.0, Math.Max(low - incDeg, incDeg - (180.0 - low)));
    }

    /// <summary>
    /// The nearest inclination a direct launch reaches, rounded to a tenth of a degree into the band so that the
    /// rounded value is still reachable.
    /// </summary>
    public static double NearestDirectIncDeg(double siteLatDeg, double incDeg)
    {
        double low = Math.Ceiling(LowestDirectIncDeg(siteLatDeg) * 10.0) / 10.0;
        double high = Math.Floor(HighestDirectIncDeg(siteLatDeg) * 10.0) / 10.0;
        if (low > high)
            return 90.0;
        return Math.Clamp(incDeg, low, high);
    }

    /// <summary>
    /// The plane change a launch from the site direction (sx, sy, sz) needs into the plane with normal (nx, ny, nz):
    /// the angle between the normal and the nearest plane that holds the site, asin(|n . s|). Neither vector has to
    /// be a unit vector.
    /// </summary>
    public static double PlaneChangeDeg(double nx, double ny, double nz, double sx, double sy, double sz)
    {
        double n = Math.Sqrt(nx * nx + ny * ny + nz * nz);
        double s = Math.Sqrt(sx * sx + sy * sy + sz * sz);
        if (n <= 0.0 || s <= 0.0)
            return 0.0;
        double dot = Math.Abs(nx * sx + ny * sy + nz * sz) / (n * s);
        return Math.Asin(Math.Min(dot, 1.0)) * 180.0 / Math.PI;
    }

    /// <summary>The delta-v of a plane change of <paramref name="deg"/> at the apoapsis of the orbit with these radii, m/s.</summary>
    public static double PlaneChangeMs(double mu, double periapsisRadiusM, double apoapsisRadiusM, double deg)
    {
        if (!(mu > 0.0) || !(periapsisRadiusM > 0.0) || !(deg > 0.0))
            return 0.0;
        double ra = Math.Max(apoapsisRadiusM, periapsisRadiusM);
        double sma = 0.5 * (periapsisRadiusM + ra);
        double va = Math.Sqrt(mu * (2.0 / ra - 1.0 / sma));
        return 2.0 * va * Math.Sin(0.5 * deg * Math.PI / 180.0);
    }

    /// <summary>
    /// The speed relative to the ground that an insertion at orbital speed <paramref name="orbitSpeedMs"/> needs for a
    /// direct launch into <paramref name="incDeg"/>, with the site carried east at <paramref name="equatorSpeedMs"/>
    /// cos(lat). The insertion is taken over the site, which is the usual first-order estimate of the launch azimuth
    /// cost. NaN when the inclination is out of direct reach.
    /// </summary>
    public static double GroundRelativeSpeedMs(double siteLatDeg, double incDeg, double orbitSpeedMs, double equatorSpeedMs)
    {
        double lat = siteLatDeg * Math.PI / 180.0, inc = incDeg * Math.PI / 180.0;
        double cosLat = Math.Cos(lat);
        if (cosLat < 1e-9)
            return double.NaN;
        double sinAz = Math.Cos(inc) / cosLat;
        if (Math.Abs(sinAz) > 1.0 + 1e-12)
            return double.NaN;
        sinAz = Math.Clamp(sinAz, -1.0, 1.0);
        double east = orbitSpeedMs * sinAz - equatorSpeedMs * cosLat;
        double north = orbitSpeedMs * Math.Sqrt(1.0 - sinAz * sinAz);
        return Math.Sqrt(east * east + north * north);
    }

    /// <summary>What this inclination costs over a launch due east, which reaches the lowest direct inclination, m/s. NaN when out of direct reach.</summary>
    public static double AzimuthCostMs(double siteLatDeg, double incDeg, double orbitSpeedMs, double equatorSpeedMs)
        => GroundRelativeSpeedMs(siteLatDeg, incDeg, orbitSpeedMs, equatorSpeedMs)
         - GroundRelativeSpeedMs(siteLatDeg, LowestDirectIncDeg(siteLatDeg), orbitSpeedMs, equatorSpeedMs);
}

/// <summary>The facts of the REACH row of the launch form. A row shows only when <see cref="Shows"/>.</summary>
internal readonly record struct ReachFacts(double SiteLatDeg, double IncDeg, double PlaneChangeDeg, double LeastPlaneChangeDeg, double PlaneChangeMs, bool HasTarget)
{
    /// <summary>A plane change the launch would fly is worth a row.</summary>
    public bool Shows => PlaneChangeDeg >= LaunchReach.PlaneChangeShownDeg;

    /// <summary>The site cannot reach the plane at any launch time, so the inclination itself is the cause, not the time.</summary>
    public bool OutOfReach => LeastPlaneChangeDeg >= LaunchReach.PlaneChangeShownDeg;

    /// <summary>The inclination the fix button offers, or NaN when there is none to offer (a target sets the plane).</summary>
    public double OfferIncDeg => OutOfReach && !HasTarget ? LaunchReach.NearestDirectIncDeg(SiteLatDeg, IncDeg) : double.NaN;
}

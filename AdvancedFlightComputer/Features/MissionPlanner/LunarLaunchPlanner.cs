using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MissionPlanner;

/// <summary>
/// How the parking orbit's plane is chosen. With <see cref="Inclination"/> or <see cref="Node"/> the player sets that angle and the other follows from the moon's position at arrival.
/// <see cref="InPlane"/> sets neither: the plane is the moon's own orbit plane, so the injection is an in-plane manoeuvre with the moon's orbit and the transfer meets the moon along its path. A site gets into that plane only if its latitude is inside the moon's declination range.
/// </summary>
internal enum PlaneControl { Inclination, Node, InPlane }

/// <summary>
/// What the player chose. <see cref="IncDeg"/> is read with <see cref="PlaneControl.Inclination"/>, and <see cref="Southbound"/> picks which of that inclination's two planes. <see cref="LanDeg"/> is read with <see cref="PlaneControl.Node"/>. <see cref="PlaneControl.InPlane"/> reads neither.
/// </summary>
internal readonly record struct LunarLaunchInputs(
    double ArrivalTime,
    PlaneControl Control,
    double IncDeg,
    double LanDeg,
    bool Southbound,
    double ParkingPeKm,
    double ParkingApKm);

/// <summary>A plan to meet a moon: the transfer, the plane, and when the site passes under it. Times are sim seconds, angles degrees.</summary>
internal sealed class LunarLaunchPlan
{
    /// <summary>Why there is no plane to launch into, empty when there is one.</summary>
    public string Problem = "";

    public double Now;
    public double ArrivalTime;
    public double EarliestArrival;
    public double TransferTime;
    public double InjectionTime;
    public double MoonDistance;
    public double MoonDeclinationDeg;
    public double MoonTravelDeg;
    public double SiteLatDeg;
    public double ParkingPeriod;

    /// <summary>
    /// The moon's own orbit plane around the home body, where it is at arrival, and the furthest it gets north or south of the equator, which is that plane's inclination.
    /// Its declination swings between plus and minus that once an orbit. The plane itself stays put: KSA moves the moon on a fixed ellipse and does not precess the home body's axis.
    /// </summary>
    public double MoonOrbitIncDeg;
    public double MoonOrbitLanDeg;
    public double MoonMaxDeclinationDeg;

    /// <summary>
    /// The least inclination of a plane that holds the moon at arrival and passes over the site: whichever is further from the equator, the site or the moon at arrival. The moon's half of that changes with the arrival, as its declination swings through the month, so a plane just above the site's latitude (the most easterly launch) serves only the arrivals when the moon is inside it.
    /// </summary>
    public double LeastInclinationDeg;
    public bool LeastInclinationIsSite;

    /// <summary>Between the plane and the moon's orbit plane: zero in-plane, otherwise the angle the transfer crosses the moon's path at.</summary>
    public double TiltToMoonOrbitDeg = double.NaN;

    public bool HasPlane;
    public double IncDeg = double.NaN;
    public double LanDeg = double.NaN;

    /// <summary>The two planes of the set inclination, with <see cref="PlaneControl.Inclination"/>; NaN otherwise, or when neither reaches the moon.</summary>
    public double LanNorthboundDeg = double.NaN;
    public double LanSouthboundDeg = double.NaN;
    public double WaitNorthbound = double.NaN;
    public double WaitSouthbound = double.NaN;

    /// <summary>The site passes under the plane. False when the plane's inclination is below the site's latitude.</summary>
    public bool SiteReachesPlane;

    /// <summary>To ignition at the next window, the ascent's own lead included, and whether it is the descending crossing (a south-easterly launch).</summary>
    public double WaitSec = double.NaN;
    public bool Descending;
    public double AzimuthDeg = double.NaN;

    /// <summary>From orbit insertion, estimated, to the injection burn. Negative when the injection comes first.</summary>
    public double CoastSec = double.NaN;

    public bool Retrograde => HasPlane && IncDeg > 90.0;
}

/// <summary>
/// Plans a launch from the home body to meet one of its moons at a chosen arrival, for an idealised Hohmann transfer from the parking orbit: the plane has to contain the moon's position at arrival (see <see cref="LunarTransferGeometry"/>), or in-plane is the moon's own orbit plane, which always does, and the injection burn is the transfer time before the arrival.
/// The launch windows are the ascent's own (<see cref="GuidanceWindow.TryLaunchWindow"/>), so what is quoted here is what EXECUTE arms for once the plane is sent.
/// </summary>
internal static class LunarLaunchPlanner
{
    /// <summary>From ignition to orbit insertion, roughly, for the parking-coast estimate. The ascent is a few minutes either side of it, which the coast of a parking orbit or more does not notice.</summary>
    internal const double InsertionAfterIgnitionS = 600.0;

    /// <summary>The moon's position at <paramref name="time"/>, in its parent's CCI frame.</summary>
    public static double3 MoonAt(Celestial moon, double time)
        => moon.Orbit.GetStateVectorsAt(new UniverseTime(time)).PositionCci;

    /// <summary>
    /// The moon's declination at <paramref name="time"/>, deg: its angle north or south of the parent's equator in the non-rotating equatorial (CCI) frame.
    /// It moves only as the moon goes round its orbit, never with the parent's spin.
    /// </summary>
    public static double DeclinationAt(Celestial moon, double time)
    {
        double3 r = MoonAt(moon, time);
        return UpfgTarget.RadToDeg(LunarTransferGeometry.Declination(r / r.Length()));
    }

    /// <summary>The northern and southern extremes of the moon's monthly swing in declination, the next of each after a time.</summary>
    internal readonly record struct DeclinationPeaks(double NorthTime, double NorthDeg, double SouthTime, double SouthDeg);

    /// <summary>
    /// The next northern and southern peaks of the moon's declination after <paramref name="from"/>: sampled over one orbit, then refined by golden-section search between the samples either side, to well under a minute.
    /// </summary>
    public static DeclinationPeaks NextDeclinationPeaks(Celestial moon, double from)
    {
        const int Samples = 96;
        double step = moon.Orbit.Period / Samples;
        int north = 0, south = 0;
        double high = double.MinValue, low = double.MaxValue;
        for (int i = 0; i <= Samples; i++)
        {
            double dec = DeclinationAt(moon, from + i * step);
            if (dec > high) { high = dec; north = i; }
            if (dec < low) { low = dec; south = i; }
        }
        double northTime = Extreme(moon, from + Math.Max(north - 1, 0) * step, from + (north + 1) * step, 1.0);
        double southTime = Extreme(moon, from + Math.Max(south - 1, 0) * step, from + (south + 1) * step, -1.0);
        return new DeclinationPeaks(northTime, DeclinationAt(moon, northTime), southTime, DeclinationAt(moon, southTime));
    }

    // Golden-section search for the time in [a, b] where sign * declination is greatest.
    private static double Extreme(Celestial moon, double a, double b, double sign)
    {
        const double Ratio = 0.6180339887498949;
        double c = b - Ratio * (b - a), d = a + Ratio * (b - a);
        double fc = sign * DeclinationAt(moon, c), fd = sign * DeclinationAt(moon, d);
        for (int i = 0; i < 40; i++)
        {
            if (fc > fd) { b = d; d = c; fd = fc; c = b - Ratio * (b - a); fc = sign * DeclinationAt(moon, c); }
            else { a = c; c = d; fc = fd; d = a + Ratio * (b - a); fd = sign * DeclinationAt(moon, d); }
        }
        return 0.5 * (a + b);
    }

    /// <summary>
    /// The soonest arrival: a transfer injected now. The transfer time depends on how far away the moon is at arrival, so it is iterated, and settles within a few passes because that distance changes slowly.
    /// </summary>
    public static double EarliestArrival(Celestial moon, double mu, double parkingRadius, double now)
    {
        double arrival = now + LunarTransferGeometry.HohmannTime(mu, parkingRadius, moon.Orbit.SemiMajorAxis);
        for (int i = 0; i < 4; i++)
            arrival = now + LunarTransferGeometry.HohmannTime(mu, parkingRadius, MoonAt(moon, arrival).Length());
        return arrival;
    }

    public static LunarLaunchPlan Solve(Vehicle vehicle, Celestial moon, double now, in LunarLaunchInputs inputs)
    {
        IParentBody home = moon.Orbit.Parent;
        double mu = home.Mu;
        double parkingRadius = home.MeanRadius + inputs.ParkingPeKm * 1000.0;
        double parkingSma = home.MeanRadius + 0.5 * (inputs.ParkingPeKm + inputs.ParkingApKm) * 1000.0;
        var plan = new LunarLaunchPlan
        {
            Now = now,
            ArrivalTime = inputs.ArrivalTime,
            ParkingPeriod = 2.0 * Math.PI * Math.Sqrt(parkingSma * parkingSma * parkingSma / mu),
            EarliestArrival = EarliestArrival(moon, mu, parkingRadius, now),
        };

        StateVectors moonThen = moon.Orbit.GetStateVectorsAt(new UniverseTime(inputs.ArrivalTime));
        double3 moonAtArrival = moonThen.PositionCci;
        plan.MoonDistance = moonAtArrival.Length();
        double3 u = moonAtArrival / plan.MoonDistance;
        plan.MoonDeclinationDeg = UpfgTarget.RadToDeg(LunarTransferGeometry.Declination(u));
        plan.TransferTime = LunarTransferGeometry.HohmannTime(mu, parkingRadius, plan.MoonDistance);
        plan.InjectionTime = inputs.ArrivalTime - plan.TransferTime;

        // How far the moon moves while the transfer flies: the injection aims at where it will be, not where it is.
        double3 moonAtInjection = MoonAt(moon, plan.InjectionTime);
        plan.MoonTravelDeg = UpfgTarget.RadToDeg(Math.Acos(Math.Clamp(
            double3.Dot(u, moonAtInjection / moonAtInjection.Length()), -1.0, 1.0)));

        double3 site = vehicle.Orbit.StateVectors.PositionCci;
        double omega = home.GetAngularVelocity();
        plan.SiteLatDeg = UpfgTarget.RadToDeg(Math.Asin(Math.Clamp(site.Z / site.Length(), -1.0, 1.0)));

        // The moon's orbit plane where the transfer meets it. KSA keeps it fixed, but the arrival is the plane that matters if that ever changes.
        double3 moonNormal = double3.Normalize(double3.Cross(moonAtArrival, moonThen.VelocityCci));
        double moonInc = Math.Acos(Math.Clamp(moonNormal.Z, -1.0, 1.0));
        plan.MoonOrbitIncDeg = UpfgTarget.RadToDeg(moonInc);
        plan.MoonOrbitLanDeg = moonNormal.X == 0.0 && moonNormal.Y == 0.0 ? 0.0
            : UpfgTarget.RadToDeg(UpfgTarget.WrapTwoPi(Math.Atan2(moonNormal.X, -moonNormal.Y)));
        plan.MoonMaxDeclinationDeg = UpfgTarget.RadToDeg(Math.Min(moonInc, Math.PI - moonInc));
        plan.LeastInclinationIsSite = Math.Abs(plan.SiteLatDeg) >= Math.Abs(plan.MoonDeclinationDeg);
        plan.LeastInclinationDeg = Math.Max(Math.Abs(plan.SiteLatDeg), Math.Abs(plan.MoonDeclinationDeg));

        if (inputs.Control == PlaneControl.InPlane)
        {
            plan.IncDeg = plan.MoonOrbitIncDeg;
            plan.LanDeg = plan.MoonOrbitLanDeg;
        }
        else if (inputs.Control == PlaneControl.Inclination)
        {
            double inc = UpfgTarget.DegToRad(inputs.IncDeg);
            if (!LunarTransferGeometry.TryNodesForInclination(u, inc, out double north, out double south))
            {
                plan.Problem = $"No plane inclined {inputs.IncDeg:F2} deg reaches {moon.Id} at arrival, "
                    + $"at declination {plan.MoonDeclinationDeg:F2} deg. Raise the inclination past that, or arrive when {moon.Id} is nearer the equator.";
                return plan;
            }
            plan.LanNorthboundDeg = UpfgTarget.RadToDeg(north);
            plan.LanSouthboundDeg = UpfgTarget.RadToDeg(south);
            plan.WaitNorthbound = WaitFor(site, omega, inc, north);
            plan.WaitSouthbound = WaitFor(site, omega, inc, south);
            plan.IncDeg = inputs.IncDeg;
            plan.LanDeg = inputs.Southbound ? plan.LanSouthboundDeg : plan.LanNorthboundDeg;
        }
        else
        {
            double lan = UpfgTarget.WrapTwoPi(UpfgTarget.DegToRad(inputs.LanDeg));
            double inc = LunarTransferGeometry.InclinationForNode(u, lan);
            if (double.IsNaN(inc))
            {
                plan.Problem = $"{moon.Id} is on this node's line at arrival, so every inclination reaches it. Move the node or the arrival a little.";
                return plan;
            }
            plan.IncDeg = UpfgTarget.RadToDeg(inc);
            plan.LanDeg = UpfgTarget.RadToDeg(lan);
        }
        plan.HasPlane = true;

        double incRad = UpfgTarget.DegToRad(plan.IncDeg);
        double lanRad = UpfgTarget.DegToRad(plan.LanDeg);
        plan.TiltToMoonOrbitDeg = UpfgTarget.RadToDeg(Math.Acos(Math.Clamp(
            double3.Dot(UpfgTarget.OrbitNormal(incRad, lanRad), moonNormal), -1.0, 1.0)));

        // In-plane, this is the check that the launch latitude is inside the moon's declination range: the site passes under the moon's plane only then.
        plan.SiteReachesPlane = GuidanceWindow.TryLaunchWindow(site, omega, incRad, lanRad,
            out plan.WaitSec, out plan.Descending, out _);
        if (!plan.SiteReachesPlane)
        {
            plan.Problem = inputs.Control == PlaneControl.InPlane
                ? $"The site, at latitude {Math.Abs(plan.SiteLatDeg):F2} deg, is outside {moon.Id}'s declination range of +/-{plan.MoonMaxDeclinationDeg:F2} deg, "
                  + $"so it never passes under {moon.Id}'s orbit plane. Set the inclination or the LAN instead, for a plane through {moon.Id} at arrival."
                : $"The site, at latitude {Math.Abs(plan.SiteLatDeg):F2} deg, never passes under a plane inclined {plan.IncDeg:F2} deg.";
            return plan;
        }

        // The heading the plane crosses the site's latitude on: north-east at the ascending crossing, south-east at the descending one (for a prograde plane).
        double lat = UpfgTarget.DegToRad(plan.SiteLatDeg);
        double azimuth = Math.Asin(Math.Clamp(Math.Cos(incRad) / Math.Max(Math.Cos(lat), 1e-9), -1.0, 1.0));
        plan.AzimuthDeg = UpfgTarget.RadToDeg(plan.Descending ? Math.PI - azimuth : azimuth);

        if (double.IsFinite(plan.WaitSec))
            plan.CoastSec = plan.InjectionTime - (now + plan.WaitSec + InsertionAfterIgnitionS);
        return plan;
    }

    /// <summary>
    /// The plane a launch right now has to fly to meet the moon at <paramref name="arrivalTime"/>: through the site, and through the moon's position then.
    /// The site is taken where it will be a lead later, the plane the ascent's own window aims for (see <see cref="GuidanceWindow.LanLeadSeconds"/>), so the window it quotes for this plane is now.
    /// False when the site and the moon line up through the body's centre.
    /// </summary>
    public static bool TryLaunchNowPlane(Vehicle vehicle, Celestial moon, double arrivalTime, out double incDeg, out double lanDeg)
    {
        incDeg = lanDeg = double.NaN;
        IParentBody home = moon.Orbit.Parent;
        double3 site = LunarTransferGeometry.TurnedBy(vehicle.Orbit.StateVectors.PositionCci,
            home.GetAngularVelocity(), GuidanceWindow.LanLeadSeconds);
        if (!LunarTransferGeometry.TryPlaneThrough(site, MoonAt(moon, arrivalTime), out double inc, out double lan))
            return false;
        incDeg = UpfgTarget.RadToDeg(inc);
        lanDeg = UpfgTarget.RadToDeg(lan);
        return true;
    }

    private static double WaitFor(double3 site, double omega, double inc, double lan)
        => GuidanceWindow.TryLaunchWindow(site, omega, inc, lan, out double wait, out _, out _) ? wait : double.NaN;
}

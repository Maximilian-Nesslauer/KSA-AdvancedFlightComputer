using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MissionPlanner;

/// <summary>Which of the plane's two angles the player sets. The other follows from the moon's position at arrival.</summary>
internal enum PlaneControl { Inclination, Node }

/// <summary>
/// What the player chose. <see cref="IncDeg"/> is read with <see cref="PlaneControl.Inclination"/>, and <see cref="Southbound"/> picks which of that inclination's two planes. <see cref="LanDeg"/> is read with <see cref="PlaneControl.Node"/>.
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
/// Plans a launch from the home body to meet one of its moons at a chosen arrival, for an idealised Hohmann transfer from the parking orbit: the plane has to contain the moon's position at arrival (see <see cref="LunarTransferGeometry"/>), and the injection burn is the transfer time before the arrival.
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

        double3 moonAtArrival = MoonAt(moon, inputs.ArrivalTime);
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

        if (inputs.Control == PlaneControl.Inclination)
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
        plan.SiteReachesPlane = GuidanceWindow.TryLaunchWindow(site, omega, incRad, UpfgTarget.DegToRad(plan.LanDeg),
            out plan.WaitSec, out plan.Descending, out _);
        if (!plan.SiteReachesPlane)
        {
            plan.Problem = $"The site, at latitude {Math.Abs(plan.SiteLatDeg):F2} deg, never passes under a plane inclined {plan.IncDeg:F2} deg.";
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

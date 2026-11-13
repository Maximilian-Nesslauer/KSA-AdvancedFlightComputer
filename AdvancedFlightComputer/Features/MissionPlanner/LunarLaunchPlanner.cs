using System;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MissionPlanner;

/// <summary>
/// How the parking orbit's plane is chosen. With <see cref="Inclination"/> that angle is set and the node follows from the moon's position at arrival.
/// <see cref="InPlane"/> sets neither: the plane is the moon's own orbit plane, so the injection is an in-plane manoeuvre with the moon's orbit and the transfer meets the moon along its path. A site gets into that plane only if its latitude is inside the moon's declination range.
/// </summary>
internal enum PlaneControl { Inclination, InPlane }

/// <summary>
/// The request. <see cref="IncDeg"/> is read with <see cref="PlaneControl.Inclination"/>, and <see cref="Southbound"/> picks which of that inclination's two planes. <see cref="PlaneControl.InPlane"/> reads neither.
/// </summary>
internal readonly record struct LunarLaunchInputs(
    double ArrivalTime,
    PlaneControl Control,
    double IncDeg,
    bool Southbound,
    double ParkingPeKm,
    double ParkingApKm);

/// <summary>A plan to meet a moon: the transfer, the plane, and when the site passes under it. Times are sim seconds, angles degrees.</summary>
internal sealed class LunarLaunchPlan
{
    /// <summary>Why there is no plane to launch into, empty when there is one.</summary>
    public string Problem = "";

    public double ArrivalTime;
    public double TransferTime;
    public double InjectionTime;
    public double MoonDistance;
    public double MoonDeclinationDeg;
    public double SiteLatDeg;

    /// <summary>
    /// The moon's own orbit plane around the home body, where it is at arrival, and the furthest it gets north or south of the equator, which is that plane's inclination.
    /// Its declination swings between plus and minus that once an orbit. The plane itself stays put: KSA moves the moon on a fixed ellipse and does not precess the home body's axis.
    /// </summary>
    public double MoonOrbitIncDeg;
    public double MoonOrbitLanDeg;
    public double MoonMaxDeclinationDeg;

    /// <summary>Between the plane and the moon's orbit plane: zero in-plane, otherwise the angle the transfer crosses the moon's path at.</summary>
    public double TiltToMoonOrbitDeg = double.NaN;

    public bool HasPlane;
    public double IncDeg = double.NaN;
    public double LanDeg = double.NaN;

    /// <summary>The site passes under the plane. False when the plane's inclination is below the site's latitude.</summary>
    public bool SiteReachesPlane;

    /// <summary>To ignition at the next window, the ascent's own lead included, and whether it is the descending crossing (a south-easterly launch).</summary>
    public double WaitSec = double.NaN;
    public bool Descending;

    /// <summary>From orbit insertion, estimated, to the injection burn. Negative when the injection comes first.</summary>
    public double CoastSec = double.NaN;
}

/// <summary>
/// Plans a launch from the home body to meet one of its moons at a chosen arrival, for an idealised Hohmann transfer from the parking orbit: the plane has to contain the moon's position at arrival (see <see cref="LunarTransferGeometry"/>), or in-plane is the moon's own orbit plane, which always does, and the injection burn is the transfer time before the arrival.
/// The launch windows are the ascent's own (<see cref="GuidanceWindow.TryLaunchWindow"/>), so what is quoted here is what the launch arms for once the plane is its target.
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

    /// <summary>
    /// The soonest arrival the plan can make from the ground: its plane's next launch window, the climb to orbit, one parking orbit to line up the injection, then the transfer.
    /// <see cref="EarliestArrival"/> injects this instant, which no launch reaches.
    /// The plane has to hold the moon at arrival, so it moves as the arrival does, and its window with it. This therefore steps forward from the earliest arrival by what the parking coast falls short, until it no longer does.
    /// An arrival whose plane misses the moon or the site is stepped over an hour at a time.
    /// NaN when nothing within two of the moon's orbits works, as for an inclination the site never passes under.
    /// </summary>
    public static double SoonestArrival(Vehicle vehicle, Celestial moon, double now, in LunarLaunchInputs inputs)
    {
        IParentBody home = moon.Orbit.Parent;
        double sma = home.MeanRadius + 0.5 * (inputs.ParkingPeKm + inputs.ParkingApKm) * 1000.0;
        double margin = 2.0 * Math.PI * Math.Sqrt(sma * sma * sma / home.Mu);
        double arrival = EarliestArrival(moon, home.Mu, home.MeanRadius + inputs.ParkingPeKm * 1000.0, now);
        double limit = arrival + 2.0 * moon.Orbit.Period;
        double shortArrival = double.NaN;
        for (int i = 0; i < 200 && arrival <= limit; i++)
        {
            double shortfall = Shortfall(vehicle, moon, now, inputs, arrival, margin);
            if (double.IsNaN(shortfall))
            {
                arrival += 3600.0;
                continue;
            }
            if (shortfall <= 0.0)
            {
                // The steps overshoot a little. Bisect back to where the coast first covers the margin, from the last arrival that fell short.
                if (double.IsNaN(shortArrival))
                    return arrival;
                double good = arrival, bad = shortArrival;
                for (int k = 0; k < 24 && good - bad > 1.0; k++)
                {
                    double mid = 0.5 * (good + bad);
                    if (Shortfall(vehicle, moon, now, inputs, mid, margin) <= 0.0)
                        good = mid;
                    else
                        bad = mid;
                }
                return good;
            }
            shortArrival = arrival;
            // A little over, so the last steps do not creep up on the margin.
            arrival += Math.Max(1.02 * shortfall, 30.0);
        }
        return double.NaN;
    }

    // What the parking coast lacks of the margin at this arrival, s: NaN when the plane misses the moon or the site, or there is no window.
    private static double Shortfall(Vehicle vehicle, Celestial moon, double now, in LunarLaunchInputs inputs, double arrival, double margin)
    {
        LunarLaunchPlan plan = Solve(vehicle, moon, now, inputs with { ArrivalTime = arrival });
        return plan.HasPlane && plan.SiteReachesPlane && double.IsFinite(plan.CoastSec) ? margin - plan.CoastSec : double.NaN;
    }

    public static LunarLaunchPlan Solve(Vehicle vehicle, Celestial moon, double now, in LunarLaunchInputs inputs)
    {
        IParentBody home = moon.Orbit.Parent;
        double mu = home.Mu;
        double parkingRadius = home.MeanRadius + inputs.ParkingPeKm * 1000.0;
        var plan = new LunarLaunchPlan { ArrivalTime = inputs.ArrivalTime };

        StateVectors moonThen = moon.Orbit.GetStateVectorsAt(new UniverseTime(inputs.ArrivalTime));
        double3 moonAtArrival = moonThen.PositionCci;
        plan.MoonDistance = moonAtArrival.Length();
        double3 u = moonAtArrival / plan.MoonDistance;
        plan.MoonDeclinationDeg = UpfgTarget.RadToDeg(LunarTransferGeometry.Declination(u));
        plan.TransferTime = LunarTransferGeometry.HohmannTime(mu, parkingRadius, plan.MoonDistance);
        plan.InjectionTime = inputs.ArrivalTime - plan.TransferTime;

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

        if (inputs.Control == PlaneControl.InPlane)
        {
            plan.IncDeg = plan.MoonOrbitIncDeg;
            plan.LanDeg = plan.MoonOrbitLanDeg;
        }
        else
        {
            double inc = UpfgTarget.DegToRad(inputs.IncDeg);
            if (!LunarTransferGeometry.TryNodesForInclination(u, inc, out double north, out double south))
            {
                plan.Problem = $"No plane inclined {inputs.IncDeg:F2} deg reaches {moon.Id} at arrival, "
                    + $"at declination {plan.MoonDeclinationDeg:F2} deg.";
                return plan;
            }
            plan.IncDeg = inputs.IncDeg;
            plan.LanDeg = UpfgTarget.RadToDeg(inputs.Southbound ? south : north);
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
                  + $"so it never passes under {moon.Id}'s orbit plane."
                : $"The site, at latitude {Math.Abs(plan.SiteLatDeg):F2} deg, never passes under a plane inclined {plan.IncDeg:F2} deg.";
            return plan;
        }

        if (double.IsFinite(plan.WaitSec))
            plan.CoastSec = plan.InjectionTime - (now + plan.WaitSec + InsertionAfterIgnitionS);
        return plan;
    }
}

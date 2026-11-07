using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>A point in a picture plane, metres or any unit the picture chooses.</summary>
internal readonly record struct P2(double X, double Y);

/// <summary>
/// The pure geometry of the console pictures: the drawable true anomaly range of a conic, the projection of
/// a 3D point onto a picture plane, a uniform fit of a world box into a pixel box, and the broken
/// distance axis of the approach picture.
/// </summary>
internal static class SketchMath
{
    public static int NearestTime(ReadOnlySpan<double> times, double time)
    {
        if (!double.IsFinite(time))
            return -1;
        int best = -1;
        double bestGap = double.MaxValue;
        for (int i = 0; i < times.Length; i++)
        {
            double gap = Math.Abs(times[i] - time);
            if (gap < bestGap)
            {
                bestGap = gap;
                best = i;
            }
        }
        return best;
    }

    /// <summary>The true anomaly range a conic can be drawn over: the whole ellipse, or a hyperbola up to a margin inside its asymptotes.</summary>
    public static (double From, double To) DrawableAnomaly(double e)
    {
        if (e < 1.0)
            return (-Math.PI, Math.PI);
        double asymptote = Math.Acos(-1.0 / e);
        double limit = asymptote * 0.92;
        return (-limit, limit);
    }

    /// <summary>The projection of (x, y, z) onto a plane spanned by the unit vectors u and v.</summary>
    public static P2 Project(double x, double y, double z, double ux, double uy, double uz, double vx, double vy, double vz)
        => new(x * ux + y * uy + z * uz, x * vx + y * vy + z * vz);

    /// <summary>
    /// A uniform scale and an offset that fit the world box into the pixel box with a margin, keeping
    /// the aspect, so a circle stays round. Y grows downward on screen, so a world Y maps upward.
    /// </summary>
    public static (double Scale, double OffsetX, double OffsetY) Fit(
        double minX, double maxX, double minY, double maxY, double pixX, double pixY, double pixW, double pixH, double margin)
    {
        double w = Math.Max(maxX - minX, 1e-9);
        double h = Math.Max(maxY - minY, 1e-9);
        double usableW = Math.Max(pixW - 2.0 * margin, 1.0);
        double usableH = Math.Max(pixH - 2.0 * margin, 1.0);
        double scale = Math.Min(usableW / w, usableH / h);
        double cx = (minX + maxX) * 0.5;
        double cy = (minY + maxY) * 0.5;
        double offsetX = pixX + pixW * 0.5 - cx * scale;
        double offsetY = pixY + pixH * 0.5 + cy * scale;
        return (scale, offsetX, offsetY);
    }

    /// <summary>The screen point of a world point under a <see cref="Fit"/>.</summary>
    public static P2 ToScreen(P2 world, (double Scale, double OffsetX, double OffsetY) fit)
        => new(fit.OffsetX + world.X * fit.Scale, fit.OffsetY - world.Y * fit.Scale);

    /// <summary>
    /// The broken distance axis of the approach picture. The last <paramref name="nearSpan"/> metres
    /// to the site fill the right <paramref name="nearShare"/> of the width linearly, and everything
    /// farther out, up to <paramref name="farSpan"/>, is squeezed logarithmically into the left part, so
    /// the gate and the touchdown are readable while a craft hundreds of kilometres out still fits.
    /// Returns the share of the width from the left edge, 0 to 1. The site is at 1.
    /// </summary>
    public static double BrokenAxis(double distance, double nearSpan, double farSpan, double nearShare)
    {
        double d = Math.Max(0.0, distance);
        if (d <= nearSpan || farSpan <= nearSpan)
            return 1.0 - nearShare * Math.Min(d, nearSpan) / Math.Max(nearSpan, 1e-9);
        double t = Math.Log(Math.Min(d, farSpan) / nearSpan) / Math.Log(farSpan / nearSpan);
        return (1.0 - nearShare) * (1.0 - Math.Clamp(t, 0.0, 1.0));
    }

    public const double SketchBodyShare = 0.42;

    public const double SketchAirShare = 0.05;

    public static double SchematicRadius(double radius, double bodyRadius, double topRadius, double bodyShare = SketchBodyShare, double airRadius = 0.0)
    {
        double top = Math.Max(topRadius - bodyRadius, 1.0);
        double altitude = Math.Clamp(radius - bodyRadius, 0.0, top);
        double air = airRadius - bodyRadius;
        if (!(air > 0.0) || air >= top)
            return bodyShare + (1.0 - bodyShare) * Math.Sqrt(altitude / top);
        double bandTop = bodyShare + SketchAirShare;
        return altitude <= air
            ? bodyShare + SketchAirShare * altitude / air
            : bandTop + (1.0 - bandTop) * Math.Sqrt((altitude - air) / (top - air));
    }

    public static double ConicShare(double peShare, double apShare, double trueAnomaly)
    {
        double e = (apShare - peShare) / Math.Max(apShare + peShare, 1e-9);
        return peShare * (1.0 + e) / (1.0 + e * Math.Cos(trueAnomaly));
    }

    public static bool ApsisDotsShow(double eccentricity) => eccentricity >= 0.01;

    public static bool LineStandsClear(double radius, ReadOnlySpan<double> others, double gap)
    {
        foreach (double other in others)
        {
            if (Math.Abs(other - radius) < gap)
                return false;
        }
        return true;
    }

    public static double PlaneLineReach(double wanted, double halfHeight, double angleDeg)
    {
        double sine = Math.Abs(Math.Sin(angleDeg * Math.PI / 180.0));
        return sine * wanted > halfHeight ? halfHeight / sine : wanted;
    }

    public static bool TrackIsClimb(double firstKm, double lastKm, double airTopKm, double noAirLimitKm = 30.0)
        => double.IsFinite(firstKm) && double.IsFinite(lastKm) && firstKm < (airTopKm > 0.0 ? airTopKm : noAirLimitKm) && lastKm > firstKm;

    public static bool AtLimit(double percent, double limitPercent) => double.IsFinite(percent) && double.IsFinite(limitPercent) && percent >= limitPercent - 0.5;

    public static bool PeInAir(double peHeight, double airTop) => airTop > 0.0 && peHeight >= 0.0 && peHeight < airTop;

    public static PassTone PassToneOf(double minKm, double insideKm, double farKm)
        => minKm <= insideKm ? PassTone.Inside : minKm >= farKm ? PassTone.Far : PassTone.Plain;

    public static double RootShare(double value, double top)
        => top > 0.0 ? Math.Sqrt(Math.Clamp(value / top, 0.0, 1.0)) : 0.0;

    public static (double X, double Y, bool OffScale) DialPoint(double east, double north, double scale)
    {
        double s = Math.Max(scale, 1e-9);
        double x = east / s, y = north / s;
        double r = Math.Sqrt(x * x + y * y);
        return r > 1.0 ? (x / r, y / r, true) : (x, y, false);
    }

    public static (double Share, bool OffScale) BarPoint(double value, double scale)
    {
        double share = value / Math.Max(scale, 1e-9);
        return Math.Abs(share) > 1.0 ? (Math.Sign(share), true) : (share, false);
    }

    /// <summary>
    /// The top of the altitude axis of the launch profile, in km: 15 percent over the highest of the plan, the flown track, the
    /// target periapsis, the top of the atmosphere and 10 km. The apoapsis does not count, because the climb inserts near the
    /// periapsis, so a high apoapsis would squeeze the curve into the bottom of the picture; the picture marks it at its top edge
    /// instead. A value that is not finite counts as 0.
    /// </summary>
    public static double ProfileTopKm(double planTopKm, double flownTopKm, double peKm, double atmoKm)
    {
        static double F(double v) => double.IsFinite(v) ? v : 0.0;
        return 1.15 * Math.Max(Math.Max(Math.Max(F(planTopKm), F(flownTopKm)), Math.Max(F(peKm), F(atmoKm))), 10.0);
    }
}

internal enum PassTone { Inside, Plain, Far }

/// <summary>One stage of a craft for the PAID BY estimate: its mass when it starts, the propellant it can burn, and its exhaust velocity.</summary>
internal readonly record struct StageFuel(int Number, double StartMassKg, double FuelKg, double ExhaustVelocityMs);

/// <summary>Which stage pays a burn, and about how much propellant it takes.</summary>
internal readonly record struct BurnPay(int FirstStage, int LastStage, double PropellantKg, double ShortMs);

/// <summary>
/// The rocket equation applied to a list of burns in order: each burn takes what is left in the
/// current stage and spills into the next one when that is empty. Every result is an estimate, and
/// the console marks it with "~".
/// </summary>
internal static class BurnBudget
{
    public static double MassForDv(double m0, double dv, double ve)
        => m0 > 0.0 && ve > 0.0 && dv > 0.0 ? m0 * (1.0 - Math.Exp(-dv / ve)) : 0.0;

    public static double DvFromMass(double m0, double m1, double ve)
        => m0 > 0.0 && m1 > 0.0 && ve > 0.0 && m0 >= m1 ? ve * Math.Log(m0 / m1) : 0.0;

    /// <summary>The delta-v all stages hold together, m/s.</summary>
    public static double OnBoard(StageFuel[] stages)
    {
        double sum = 0.0;
        foreach (StageFuel s in stages)
            sum += DvFromMass(s.StartMassKg, s.StartMassKg - s.FuelKg, s.ExhaustVelocityMs);
        return sum;
    }

    public static bool IsShort(double onBoardMs, double totalDv) => totalDv > onBoardMs * 1.001;

    /// <summary>Pays the burns in order. A burn no stage can finish reports how much of it is short.</summary>
    public static BurnPay[] Pay(StageFuel[] stages, double[] burnsDv)
    {
        var result = new BurnPay[burnsDv.Length];
        int stage = 0;
        double fuelLeft = stages.Length > 0 ? stages[0].FuelKg : 0.0;
        double mass = stages.Length > 0 ? stages[0].StartMassKg : 0.0;
        for (int i = 0; i < burnsDv.Length; i++)
        {
            double need = Math.Max(0.0, burnsDv[i]);
            int first = -1, last = -1;
            double kg = 0.0;
            while (need > 1e-6 && stage < stages.Length)
            {
                if (fuelLeft <= 1e-6)
                {
                    stage++;
                    if (stage < stages.Length)
                    {
                        fuelLeft = stages[stage].FuelKg;
                        mass = stages[stage].StartMassKg;
                    }
                    continue;
                }
                double ve = stages[stage].ExhaustVelocityMs;
                double want = MassForDv(mass, need, ve);
                if (first < 0)
                    first = stage;
                last = stage;
                if (want <= fuelLeft)
                {
                    kg += want;
                    fuelLeft -= want;
                    mass -= want;
                    need = 0.0;
                }
                else
                {
                    need -= DvFromMass(mass, mass - fuelLeft, ve);
                    kg += fuelLeft;
                    mass -= fuelLeft;
                    fuelLeft = 0.0;
                }
            }
            result[i] = new BurnPay(first, last, kg, need > 1e-6 ? need : 0.0);
        }
        return result;
    }

    public static StageFuel[] After(StageFuel[] stages, double[] burnsDv)
    {
        var left = (StageFuel[])stages.Clone();
        int stage = 0;
        foreach (double burn in burnsDv)
        {
            double need = Math.Max(0.0, burn);
            while (need > 1e-6 && stage < left.Length)
            {
                StageFuel s = left[stage];
                if (s.FuelKg <= 1e-6)
                {
                    stage++;
                    continue;
                }
                double want = MassForDv(s.StartMassKg, need, s.ExhaustVelocityMs);
                double take = Math.Min(want, s.FuelKg);
                need = want <= s.FuelKg ? 0.0 : need - DvFromMass(s.StartMassKg, s.StartMassKg - s.FuelKg, s.ExhaustVelocityMs);
                left[stage] = s with { StartMassKg = s.StartMassKg - take, FuelKg = s.FuelKg - take };
            }
        }
        return Array.FindAll(left, s => s.FuelKg > 1e-6);
    }

    public static string Payer(StageFuel[] stages, in BurnPay pay)
        => pay.FirstStage == pay.LastStage
            ? "Seq " + stages[pay.FirstStage].Number.ToString(Fmt.Inv)
            : "Seq " + stages[pay.FirstStage].Number.ToString(Fmt.Inv) + "-" + stages[pay.LastStage].Number.ToString(Fmt.Inv);
}

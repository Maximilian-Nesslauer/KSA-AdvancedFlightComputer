using AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// The ground slope of a site, the stance of a craft on its feet, the slope on which it goes over after an upright touchdown, and the search for flatter ground, on made terrain with known answers.
/// </summary>
internal static class SiteSlopeCheck
{
    private const double Deg = Math.PI / 180.0;

    internal static int Run()
    {
        System.Globalization.CultureInfo.CurrentCulture = System.Globalization.CultureInfo.InvariantCulture;
        int failures = 0;
        void Check(string name, bool ok, string detail)
        {
            Console.WriteLine($"  {(ok ? "ok  " : "FAIL")} {name}: {detail}");
            if (!ok) failures++;
        }

        Console.WriteLine("GROUND SLOPE");
        foreach ((double slope, double towards) in new[] { (0.0, 0.0), (10.0, 30.0), (14.0, 200.0), (35.0, 300.0) })
        {
            double grad = Math.Tan(slope * Deg);
            double fit = SiteSlope.SlopeRad(Rings(4.5, (e, n) => 3.0 + grad * (e * Math.Sin(towards * Deg) + n * Math.Cos(towards * Deg))), out double de, out double dn) / Deg;
            Check($"a plane of {slope:F0} deg", Math.Abs(fit - slope) < 1e-9, $"{fit:F6} deg");
            if (slope > 0.0)
            {
                double downhill = Math.Atan2(de, dn) / Deg;
                double expected = (towards + 180.0) % 360.0;
                Check($"its downhill direction, {expected:F0} deg from north", Math.Abs(((downhill - expected) % 360.0 + 540.0) % 360.0 - 180.0) < 1e-6, $"{(downhill + 360.0) % 360.0:F3} deg");
            }
        }
        double bumpy = SiteSlope.SlopeRad(Rings(4.5, (e, n) => 0.18 * e + 0.05 * Math.Sin(3.0 * e) * Math.Cos(2.0 * n)), out _, out _) / Deg;
        Check("small bumps on a 10 deg plane leave its slope", Math.Abs(bumpy - Math.Atan(0.18) / Deg) < 0.5, $"{bumpy:F2} deg against {Math.Atan(0.18) / Deg:F2}");
        Check("two samples give no slope", double.IsNaN(SiteSlope.SlopeRad([new(0, 0, 0), new(1, 0, 1)], out _, out _)), "NaN");
        Check("samples on a line give no slope", double.IsNaN(SiteSlope.SlopeRad([new(0, 0, 0), new(1, 0, 1), new(2, 0, 2), new(3, 0, 3)], out _, out _)), "NaN");

        Console.WriteLine("STANCE");
        SiteSlope.SupportPoint[] four = Feet(4, 4.0, 6.0, 0.0);
        SiteSlope.Stance square = SiteSlope.StanceOf(four, 0.1);
        Check("four feet on a circle of 4 m", square.Known && square.Corners == 4 && Math.Abs(square.InnerM - 4.0 * Math.Cos(Math.PI / 4)) < 1e-9 && Math.Abs(square.OuterM - 4.0) < 1e-9,
            $"{square.Corners} corners, inner {square.InnerM:F4} m, outer {square.OuterM:F4} m, height {square.HeightM:F2} m");
        SiteSlope.Stance three = SiteSlope.StanceOf(Feet(3, 4.0, 6.0, 0.0), 0.1);
        Check("three feet on the same circle stand narrower", Math.Abs(three.InnerM - 2.0) < 1e-9, $"inner {three.InnerM:F4} m");
        SiteSlope.Stance withBell = SiteSlope.StanceOf([.. four, new(0.5, 0.0, 5.8), new(-0.5, 0.0, 5.8)], 0.1);
        Check("a bell 0.2 m above the feet does not carry", withBell.Corners == 4 && withBell.InnerM == square.InnerM, $"{withBell.Corners} corners");
        SiteSlope.Stance onBell = SiteSlope.StanceOf([.. Feet(4, 4.0, 5.75, 0.0), .. Feet(8, 1.0, 5.95, 0.0)], 0.1);
        Check("a bell under the feet carries alone", Math.Abs(onBell.HeightM - 5.95) < 1e-9 && Math.Abs(onBell.OuterM - 1.0) < 1e-9, $"height {onBell.HeightM:F2} m, outer {onBell.OuterM:F2} m");
        SiteSlope.Stance aside = SiteSlope.StanceOf([new(2, 1, 5), new(3, 1, 5), new(2.5, 2, 5)], 0.1);
        Check("feet that leave the axis outside give no inner radius", aside.InnerM == 0.0, $"inner {aside.InnerM:F3} m");
        Check("one foot is no stance", !SiteSlope.StanceOf([new(0, 0, 5)], 0.1).Known, "not known");

        Console.WriteLine("TIP-OVER LIMIT");
        // A craft like the logged one that went over on 10 deg: feet on a circle of 4 m, centre of mass 6 m up, k^2 6.6 m^2, on the Moon.
        double tip = SiteSlope.TipLimitRad(in square, 6.6, 1.0, 1.62) / Deg;
        double stat = Math.Atan(square.InnerM / square.HeightM) / Deg;
        double stand = SiteSlope.StandLimitRad(in square) / Deg;
        Check("the limit is under the static limit on the inner radius", tip > 0.0 && tip < stat && stat < stand, $"tip {tip:F2}, static {stat:F2}, stand {stand:F2} deg");
        Check("the fall onto the downhill feet, not the contact speed, sets it", Math.Abs(SiteSlope.TipLimitRad(in square, 6.6, 0.0, 1.62) / Deg - tip) < 0.5,
            $"{SiteSlope.TipLimitRad(in square, 6.6, 0.0, 1.62) / Deg:F2} deg at rest, {tip:F2} at 1 m/s");
        Check("a harder contact lowers it", SiteSlope.TipLimitRad(in square, 6.6, 3.0, 1.62) / Deg < tip, $"{SiteSlope.TipLimitRad(in square, 6.6, 3.0, 1.62) / Deg:F2} deg at 3 m/s");
        SiteSlope.Stance wide = square with { InnerM = square.InnerM * 1.5 }, tall = square with { HeightM = square.HeightM * 1.5 };
        Check("wider feet raise it", SiteSlope.TipLimitRad(in wide, 6.6, 1.0, 1.62) / Deg > tip, $"{SiteSlope.TipLimitRad(in wide, 6.6, 1.0, 1.62) / Deg:F2} deg");
        Check("a higher centre of mass lowers it", SiteSlope.TipLimitRad(in tall, 6.6, 1.0, 1.62) / Deg < tip, $"{SiteSlope.TipLimitRad(in tall, 6.6, 1.0, 1.62) / Deg:F2} deg");
        // At the limit the turn after the second contact lifts the centre of mass exactly over the downhill feet.
        double a = tip * Deg, r = square.InnerM, h = square.HeightM, k2 = 6.6, g = 1.62, v = 1.0;
        double pivot = k2 + r * r + h * h, keep = (k2 + h * h - r * r) / pivot;
        double energy = keep * keep * (0.5 * v * v * r * r / pivot + g * (h * (1 - Math.Cos(a)) + r * Math.Sin(a)));
        double lift = g * (Math.Sqrt(r * r + h * h) - (r * Math.Sin(a) + h * Math.Cos(a)));
        Check("at the limit the energy just lifts the centre of mass over the feet", Math.Abs(energy - lift) < 1e-6 * lift, $"{energy:F6} against {lift:F6} m^2/s^2");
        Check("no stance gives no limit", SiteSlope.TipLimitRad(default, 6.6, 1.0, 1.62) == 0.0 && SiteSlope.StandLimitRad(default) == 0.0, "0");

        Console.WriteLine("FLAT SEARCH");
        // Ground that rises 0.2 m per metre to the north everywhere, except a level disc of 20 m around a point 120 m east and 90 m north, and a level disc of the same size 250 m west.
        double SlopeAt(double e, double n)
        {
            if (Math.Sqrt((e - 120) * (e - 120) + (n - 90) * (n - 90)) < 20.0 || Math.Sqrt((e + 250) * (e + 250) + n * n) < 20.0)
                return 0.2 * Deg;
            return Math.Atan(0.2);
        }
        var search = new SiteSlope.FlatSearch(300.0, 10.0);
        Check("the grid covers the disc of the radius", Math.Abs(search.Count - Math.PI * 30 * 30) < 0.02 * Math.PI * 30 * 30, $"{search.Count} centres");
        int calls = 0;
        while (!search.Done)
        {
            search.Step(SlopeAt, 100);
            calls++;
        }
        Check("the search ends", search.Evaluated == search.Count, $"{calls} steps of 100");
        bool found = search.TryBest(0.5 * Deg, out double be, out double bn, out double bs);
        Check("it finds the nearer of two level discs", found && Math.Sqrt((be - 120) * (be - 120) + (bn - 90) * (bn - 90)) < 20.0,
            $"{be:F0} m east, {bn:F0} m north, {bs / Deg:F2} deg");
        Check("the answer is the candidate nearest the site in the disc", found && Math.Sqrt(be * be + bn * bn) <= Math.Sqrt(120 * 120 + 90 * 90) - 10.0,
            $"{Math.Sqrt(be * be + bn * bn):F0} m from the site");
        var none = new SiteSlope.FlatSearch(50.0, 10.0);
        Check("no measured candidate gives no answer", !none.TryBest(0.5 * Deg, out _, out _, out _), "false");
        none.Step((e, n) => double.NaN, 1000);
        Check("ground with no slope anywhere gives no answer", !none.TryBest(0.5 * Deg, out _, out _, out _), "false");
        var even = new SiteSlope.FlatSearch(50.0, 10.0);
        even.Step((e, n) => 3.0 * Deg, 1000);
        Check("on even ground the site itself is the answer", even.TryBest(0.5 * Deg, out double ee, out double en, out _) && ee == 0.0 && en == 0.0, $"{ee:F0}, {en:F0}");

        Console.WriteLine(failures == 0 ? "PASS" : $"FAIL ({failures})");
        return failures == 0 ? 0 : 1;
    }

    private static SiteSlope.GroundSample[] Rings(double reach, Func<double, double, double> height)
    {
        var samples = new List<SiteSlope.GroundSample> { new(0.0, 0.0, height(0.0, 0.0)) };
        for (double r = 0.5; r <= reach + 1e-9; r += 0.5)
        {
            for (int i = 0; i < 16; i++)
            {
                double a = i * Math.PI / 8.0;
                double e = r * Math.Cos(a), n = r * Math.Sin(a);
                samples.Add(new SiteSlope.GroundSample(e, n, height(e, n)));
            }
        }
        return samples.ToArray();
    }

    private static SiteSlope.SupportPoint[] Feet(int count, double radius, double depth, double turnDeg)
    {
        var feet = new SiteSlope.SupportPoint[count];
        for (int i = 0; i < count; i++)
        {
            double a = (turnDeg + 360.0 * i / count) * Deg;
            feet[i] = new SiteSlope.SupportPoint(radius * Math.Cos(a), radius * Math.Sin(a), depth);
        }
        return feet;
    }
}

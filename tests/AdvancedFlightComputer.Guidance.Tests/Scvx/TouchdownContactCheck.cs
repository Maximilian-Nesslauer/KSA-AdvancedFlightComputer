using AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// The first contact of an upright craft on ground that is not level and a burn time pinned at its lower bound, against flights that were logged.
/// The numbers are from two landings on a site of the Moon whose ground rises 0.18 m per metre: a craft that cannot hover (minimum thrust 2.35 times its weight) and a craft that can (1.03 times).
/// </summary>
internal static class TouchdownContactCheck
{
    private const double StepM = 0.5;

    internal static int Run()
    {
        System.Globalization.CultureInfo.CurrentCulture = System.Globalization.CultureInfo.InvariantCulture;
        int failures = 0;
        void Check(string name, bool ok, string detail)
        {
            Console.WriteLine($"  {(ok ? "ok  " : "FAIL")} {name}: {detail}");
            if (!ok) failures++;
        }

        Console.WriteLine("FIRST CONTACT ON GROUND THAT IS NOT LEVEL");

        // A bell on the axis that is the lowest point and four feet 0.2 m above its rim, 3.5 m from the axis.
        TouchdownContact.FootPoint[] craft =
        [
            new(1.2, 5.95), new(3.5, 5.75), new(3.5, 5.75), new(3.5, 5.75), new(3.5, 5.75), new(1.9, 2.0),
        ];

        double level = TouchdownContact.EarlyContactM(craft, Slope(0.0, 8.0));
        Check("level ground", level == 0.0, $"{level:F3} m");

        // On the slope a foot meets the ground when the centre of mass is 5.75 + 0.18 * 3.5 = 6.38 m up, and the bell at 5.95 + 0.18 * 1.2 = 6.17 m.
        double slope = TouchdownContact.EarlyContactM(craft, Slope(0.18, 8.0));
        Check("slope 0.18, the feet come first", Math.Abs(slope - (5.75 + 0.18 * 3.5 - 5.95)) < 1e-9, $"{slope:F3} m early");

        double gentle = TouchdownContact.EarlyContactM(craft, Slope(0.05, 8.0));
        Check("slope 0.05, the bell comes first", Math.Abs(gentle - 0.05 * 1.5) < 1e-9, $"{gentle:F3} m early, the bell takes the ring outside its rim");

        var hollow = new TouchdownContact.GroundRise(StepM, [-0.4, -0.3, -0.2, -0.1]);
        Check("a site in a hollow lower than its rim counts as level", TouchdownContact.EarlyContactM(craft, hollow) == 0.0,
            $"{TouchdownContact.EarlyContactM(craft, hollow):F3} m");

        double beyond = TouchdownContact.EarlyContactM(craft, Slope(0.18, 2.0));
        Check("a foot outside the sampled rings takes the last ring", Math.Abs(beyond - Math.Max(5.95 + 0.18 * 1.5, 5.75 + 0.18 * 2.0) + 5.95) < 1e-9, $"{beyond:F3} m early");

        Check("no points", TouchdownContact.EarlyContactM([], Slope(0.18, 8.0)) == 0.0, "0 m");

        Console.WriteLine();
        Console.WriteLine("  the logged landings: plan end, first contact, speed one frame before it");
        // The craft that cannot hover: the plan ended 5.67 m over the site (the height from the first solve, 48 s before), the centre of mass was 5.955 m over the lowest point by then, and the first contact came at 6.51 m. Minimum thrust 0.149 * 952331 N at 25018 kg under 1.62 m/s2, and the plan braked at 188 to 201 kN on its last two intervals.
        double early = 6.51 - 5.67;
        double floorBraking = 0.149 * 952331 / 25018 - 1.62;
        double planBraking = 201000.0 / 25018 - 1.62;
        double low = TouchdownContact.SpeedAtEarlyContact(early, floorBraking);
        double high = TouchdownContact.SpeedAtEarlyContact(early, planBraking);
        Console.WriteLine($"  no hover: {early:F2} m early ({5.955 - 5.67:F2} m from the old height, {6.51 - 5.955:F2} m from the slope), braking {floorBraking:F2} to {planBraking:F2} m/s2 -> {low:F1} to {high:F1} m/s, flown 3.1 to 3.9 m/s");
        Check("the early contact explains the hard touchdown of the craft that cannot hover", low > 2.0 && low < 3.9 && high > 3.0, $"{low:F2} to {high:F2} m/s");

        // The craft that can hover ended its plans 10.95 m over the site and met the ground at 12.25 to 12.4 m, 1.3 to 1.45 m early, at 1.6 to 1.9 m/s: its last plan sank at a near constant 1.5 to 1.9 m/s through that height, so the early contact cost it little.
        double hoverBraking = 319000.0 / 57400 - 1.62;
        Console.WriteLine($"  hover:    1.30 m early at up to {hoverBraking:F2} m/s2 could be {TouchdownContact.SpeedAtEarlyContact(1.30, hoverBraking):F1} m/s, flown 1.6 to 1.9 m/s because the plan held a slow sink");

        double fixedEarly = 6.51 - (5.955 + TouchdownContact.EarlyContactM(
            [new(1.2, 5.955), new(3.5, 5.955), new(3.5, 5.955)], Slope(0.18, 8.0)));
        Check("with the height of now and the slope under the feet the plan ends at or above the first contact", fixedEarly <= 0.0,
            $"{-fixedEarly:F2} m above it, a drop of {Math.Sqrt(2 * 1.62 * Math.Max(-fixedEarly, 0)):F1} m/s after the cut");

        Console.WriteLine();
        Console.WriteLine("  a burn time pinned at its lower bound, as logged 19.5 m up at 12.5 m/s down with the plan end at 7.0 m");
        // The last plan that passed the gate in that flight had the burn time at the bound of 3.0 s. A craft that brakes at the plan floor or more for 3.0 s and ends at rest covers at least a T^2 / 2.
        double least = 0.5 * floorBraking * 3.0 * 3.0;
        double toGo = 19.5 - 7.0;
        Check("no burn of 3.0 s at the plan floor or above ends at rest on that height", least > toGo, $"it needs {least:F1} m, {toGo:F1} m are left");
        double timeLeft = 2.0 * toGo / 12.5;
        Check("the burn that does end there is shorter than the bound and above the floor", timeLeft < 3.0 && 12.5 / timeLeft > floorBraking,
            $"{timeLeft:F1} s at {12.5 / timeLeft:F2} m/s2");
        // In that flight the craft was still 3.5 m/s down at the plan end.

        Console.WriteLine();
        Console.WriteLine(failures == 0 ? "PASS" : $"FAIL ({failures})");
        return failures == 0 ? 0 : 1;
    }

    private static TouchdownContact.GroundRise Slope(double risePerM, double reachM)
    {
        var rings = new double[(int)Math.Round(reachM / StepM)];
        for (int i = 0; i < rings.Length; i++)
            rings[i] = risePerM * (i + 1) * StepM;
        return new TouchdownContact.GroundRise(StepM, rings);
    }
}

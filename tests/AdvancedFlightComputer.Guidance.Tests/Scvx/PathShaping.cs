using System.Globalization;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// --gs-far deg: the touchdown checks' outer glideslope cone, deg above the horizon, in place of the one each check sets (most set none). --gs-funnel: the progressive glideslope as flight sets it (Ksa6DofSetup.GlideFunnelKnots), half-angles from vertical --gs-funnel-deg (default "20,30,60") at one craft length (--gs-len, 40 m for the checks' boosters), 300 m and 1000 m; --gs-weight its price (10). --gs-side the sideways-speed cone, 1/s (0, off).
/// </summary>
internal static class PathShaping
{
    static string? Text(string name)
    {
        string[] args = Environment.GetCommandLineArgs();
        int at = Array.IndexOf(args, name);
        return at >= 0 && at + 1 < args.Length ? args[at + 1] : null;
    }

    static double Arg(string name, double fallback)
        => Text(name) is { } t ? double.Parse(t, CultureInfo.InvariantCulture) : fallback;

    static bool On => Environment.GetCommandLineArgs().Contains("--gs-funnel");

    internal static double FarDeg(double own) => Arg("--gs-far", own);
    internal static double FunnelWeight => Arg("--gs-weight", 10.0);
    internal static double SideSpeedPerM => Arg("--gs-side", 0.0);

    internal static double[]? FunnelHeightsM => On ? [Arg("--gs-len", 40.0), 300.0, 1000.0] : null;

    internal static double[]? FunnelDeg => On
        ? (Text("--gs-funnel-deg") ?? "20,30,60").Split(',').Select(v => double.Parse(v, CultureInfo.InvariantCulture)).ToArray()
        : null;

    /// <summary>--glide-funnel: the progressive glideslope's radius is continuous and its slope is its run, it is the plain cone when off, it holds the half-angles at the knots, and no tangent lies outside it (convex in height).</summary>
    internal static int Check()
    {
        int failures = 0;
        void Expect(string what, bool ok) { Console.WriteLine((ok ? "  ok   " : "  FAIL ") + what); if (!ok) failures++; }
        double[] h = [40.0, 300.0, 1000.0], a = [20.0, 30.0, 60.0];
        double plain = Scvx6DofConfig.Radius(10.0, null, null, 100.0);
        Expect($"off, it is the plain cone: {plain:F2} m at 100 m", Math.Abs(plain - 100.0 / Math.Tan(10.0 * Math.PI / 180.0)) < 1e-9);
        Expect("below one craft length it is the 20 deg cone", Math.Abs(Scvx6DofConfig.Radius(10.0, h, a, 30.0) - 30.0 * Math.Tan(20.0 * Math.PI / 180.0)) < 1e-9);
        Expect("at 300 m it allows 300 tan 30 deg, at 1000 m 1000 tan 60 deg (the angle seen from the target)",
            Math.Abs(Scvx6DofConfig.Radius(10.0, h, a, 300.0) - 300.0 * Math.Tan(30.0 * Math.PI / 180.0)) < 1e-9
            && Math.Abs(Scvx6DofConfig.Radius(10.0, h, a, 1000.0) - 1000.0 * Math.Tan(60.0 * Math.PI / 180.0)) < 1e-9);
        Console.WriteLine(FormattableString.Invariant($"  offsets allowed: {Scvx6DofConfig.Radius(10.0, h, a, 40.0):F0} m at 40 m, {Scvx6DofConfig.Radius(10.0, h, a, 100.0):F0} m at 100 m, {Scvx6DofConfig.Radius(10.0, h, a, 300.0):F0} m at 300 m, {Scvx6DofConfig.Radius(10.0, h, a, 1000.0):F0} m at 1000 m, {Scvx6DofConfig.Radius(10.0, h, a, 2000.0):F0} m at 2000 m"));
        double worstSlope = 0.0, worstTangent = double.NegativeInfinity;
        for (double z = 0.0; z < 2500.0; z += 2.5)
        {
            double r0 = Scvx6DofConfig.Radius(10.0, h, a, z), r1 = Scvx6DofConfig.Radius(10.0, h, a, z + 2.5);
            if (!h.Any(k => k > z && k < z + 2.5))
                worstSlope = Math.Max(worstSlope, Math.Abs((r1 - r0) / 2.5 - Scvx6DofConfig.RunPerM(10.0, h, a, z + 1.25)));
            for (double href = 0.0; href < 2500.0; href += 37.5)
                worstTangent = Math.Max(worstTangent, Scvx6DofConfig.Radius(10.0, h, a, href) + Scvx6DofConfig.RunPerM(10.0, h, a, href) * (z - href) - r0);
        }
        Expect($"its slope is its run away from the heights (worst {worstSlope:E1})", worstSlope < 1e-3);
        Expect($"no tangent lies outside it (worst {worstTangent:F4} m)", worstTangent < 1e-6);
        Console.WriteLine(failures == 0 ? "PASS - the progressive glideslope" : "FAIL - see above");
        return failures == 0 ? 0 : 1;
    }
}

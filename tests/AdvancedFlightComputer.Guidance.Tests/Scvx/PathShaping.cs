using System.Globalization;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// --gs-far deg --gs-near deg [--gs-near-h m] [--gs-far-h m]: the touchdown checks' glideslope, plain or progressive (Scvx6DofConfig.GlideSlopeNearDeg), in place of the one each check sets. Without them each check keeps its own. The heights default to the checks' boosters: about their length (40 m), and 150 m.
/// </summary>
internal static class PathShaping
{
    static double Arg(string name, double fallback)
    {
        string[] args = Environment.GetCommandLineArgs();
        int at = Array.IndexOf(args, name);
        return at >= 0 && at + 1 < args.Length ? double.Parse(args[at + 1], CultureInfo.InvariantCulture) : fallback;
    }

    internal static double FarDeg(double own) => Arg("--gs-far", own);
    internal static double NearDeg => Arg("--gs-near", 0.0);
    internal static double NearHeightM => Arg("--gs-near-h", 40.0);
    internal static double FarHeightM => Arg("--gs-far-h", 150.0);
    internal static double FunnelWeight => Arg("--gs-weight", 10.0);

    /// <summary>--glide-funnel: the progressive glideslope's radius is continuous, matches the plain cone when off, and its tangent never cuts inside the funnel (it is convex in height, so the tangent bounds it from below).</summary>
    internal static int Check()
    {
        int failures = 0;
        void Expect(string what, bool ok) { Console.WriteLine((ok ? "  ok   " : "  FAIL ") + what); if (!ok) failures++; }
        double plain = Scvx6DofConfig.Radius(10.0, 0.0, 40.0, 150.0, 100.0);
        Expect($"off, it is the plain cone: {plain:F2} m at 100 m", Math.Abs(plain - 100.0 / Math.Tan(10.0 * Math.PI / 180.0)) < 1e-9);
        double near = Scvx6DofConfig.Radius(10.0, 60.0, 40.0, 150.0, 20.0);
        Expect($"below the near height it is the steep cone: {near:F2} m at 20 m", Math.Abs(near - 20.0 / Math.Tan(60.0 * Math.PI / 180.0)) < 1e-9);
        double worstJump = 0.0, worstTangent = double.NegativeInfinity;
        for (double h = 0.0; h < 400.0; h += 0.5)
        {
            double r0 = Scvx6DofConfig.Radius(10.0, 60.0, 40.0, 150.0, h), r1 = Scvx6DofConfig.Radius(10.0, 60.0, 40.0, 150.0, h + 0.5);
            worstJump = Math.Max(worstJump, Math.Abs(r1 - r0) / 0.5 - Scvx6DofConfig.RunPerM(10.0, 60.0, 40.0, 150.0, h + 0.25));
            for (double href = 0.0; href < 400.0; href += 7.5)
            {
                double tangent = Scvx6DofConfig.Radius(10.0, 60.0, 40.0, 150.0, href) + Scvx6DofConfig.RunPerM(10.0, 60.0, 40.0, 150.0, href) * (h - href);
                worstTangent = Math.Max(worstTangent, tangent - r0);
            }
        }
        Expect($"its slope is the run per metre (worst {worstJump:E1})", worstJump < 0.05);
        Expect($"no tangent lies outside the funnel (worst {worstTangent:F3} m)", worstTangent < 1e-6);
        Console.WriteLine(failures == 0 ? "PASS - the progressive glideslope" : "FAIL - see above");
        return failures == 0 ? 0 : 1;
    }
}

using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;
using AdvancedFlightComputer.Guidance.Numerics;

/// <summary>
/// Touchdown guidance's air tables blended over every direction (BodyAero6Dof.Wrench): the engine-first tables, the flip band, the belly-first tables and the whole sphere, each read inside its own edges and faded out over them. With all four the same constant the blend must give that constant everywhere (the shares sum to one, nothing missing, nothing counted twice); with four different constants it must give each set's value well inside its own region and change smoothly between them along a flip from engine first through belly first to nose first, in the flip's plane and 20 deg out of it.
/// </summary>
internal static class SixDofTableBlendCheck
{
    private const double Deg = Math.PI / 180.0;

    // Belly side -Z (KsaAeroSweep.BellyAxes[0]): the whole sphere's frame leads engine first with its longitude towards the belly, the belly-first frame leads with the belly.
    private static readonly BodyAeroFrame SphereFrame = new(-1, 0, 0, 0, 0, -1, 0, -1, 0);
    private static readonly BodyAeroFrame BellyFrame = BodyAeroFrame.Belly(0, -1);

    private static double[] Grid(double from, double to, double step)
    {
        int n = (int)Math.Round((to - from) / step) + 1;
        var g = new double[n];
        for (int i = 0; i < n; i++) g[i] = from + i * step;
        return g;
    }

    private static BodyAeroTable Constant(double c, double[] a, double[] b, BodyAeroFrame frame, bool sphere)
    {
        double[] mach = AeroTable.DefaultMachBreakpoints;
        var v = new double[mach.Length * a.Length * b.Length * 3];
        for (int i = 0; i < v.Length; i += 3) v[i] = c;
        return new BodyAeroTable(mach, a, b, v, frame, sphere);
    }

    private static BodyAero6Dof Model(double engine, double band, double belly, double sphere)
    {
        double[] fine = [-20, -16, -12, -8, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 8, 12, 16, 20];
        double[] bellyA = [-20, -10, 0, 10, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80];
        double[] bellyB = [-45, -32, -20, -12, -6, -3, 0, 3, 6, 12, 20, 32, 45];
        return new BodyAero6Dof
        {
            Atmosphere = ThreeDofScenario.Model().Atmosphere!,
            Force = Constant(engine, fine, fine, BodyAeroFrame.TailFirst, false),
            BandForce = Constant(band, Grid(-15, 100, 5), Grid(-45, 45, 7.5), SphereFrame, true),
            BellyForce = Constant(belly, bellyA, bellyB, BellyFrame, false),
            SphereForce = Constant(sphere, Grid(-210, 210, 15), Grid(-90, 90, 15), SphereFrame, true),
            ReferenceArea = 1.0,
        };
    }

    /// <summary>The blended axial coefficient for an air direction u (the craft's velocity through the air, body axes): the site-frame x force at unit q S, the model's axes the table's.</summary>
    private static double Cx(BodyAero6Dof m, double ux, double uy, double uz, double ones)
    {
        const double speed = 50.0;
        m.Wrench(new Dual(1000.0), new Dual(speed * ux), new Dual(speed * uy), new Dual(speed * uz),
                 1, 0, 0, 0, 1, 0, 0, 0, 1, out Dual fx, out _, out _, out _, out _, out _, out _);
        return fx.V / ones;
    }

    private static double Ones()
    {
        var m = Model(1, 1, 1, 1);
        const double speed = 50.0;
        m.Wrench(new Dual(1000.0), new Dual(-speed), new Dual(0.0), new Dual(0.0),
                 1, 0, 0, 0, 1, 0, 0, 0, 1, out Dual fx, out _, out _, out _, out _, out _, out _);
        return fx.V;
    }

    internal static int Run(bool verbose)
    {
        Console.WriteLine("TOUCHDOWN'S AIR TABLES BLENDED OVER EVERY DIRECTION");
        int bad = 0;
        double ones = Ones();

        // 1. The same constant in all four: one everywhere.
        var same = Model(1, 1, 1, 1);
        double worst = 0.0;
        for (double lat = -85; lat <= 85; lat += 5)
            for (double lon = -180; lon < 180; lon += 2.5)
            {
                double cl = Math.Cos(lat * Deg);
                // A direction about the sphere's frame: lead engine first (-x), towards the belly (-z), latitude towards -y.
                double l = cl * Math.Cos(lon * Deg), a = cl * Math.Sin(lon * Deg), b = Math.Sin(lat * Deg);
                double ux = -l, uy = -b, uz = -a;
                worst = Math.Max(worst, Math.Abs(Cx(same, ux, uy, uz, ones) - 1.0));
            }
        bool partition = worst < 1e-9;
        if (!partition) bad++;
        Console.WriteLine(FormattableString.Invariant($"  {(partition ? "ok  " : "FAIL")} the shares sum to one over the whole sphere: worst {worst:E1}"));

        // 2. Four constants: each set's value inside its own region, smooth between, along the flip and 20 deg out of its plane.
        var mixed = Model(1, 2, 3, 4);
        foreach (double outDeg in new[] { 0.0, 20.0 })
        {
            double prev = double.NaN, jump = 0.0;
            var at = new SortedDictionary<int, double>();
            for (double th = 0; th <= 180.0 + 1e-9; th += 0.25)
            {
                double co = Math.Cos(outDeg * Deg);
                double ux = -Math.Cos(th * Deg) * co, uz = -Math.Sin(th * Deg) * co, uy = -Math.Sin(outDeg * Deg);
                double c = Cx(mixed, ux, uy, uz, ones);
                if (double.IsFinite(prev)) jump = Math.Max(jump, Math.Abs(c - prev));
                prev = c;
                int k = (int)Math.Round(th);
                if (Math.Abs(th - k) < 1e-9 && k % 15 == 0) at[k] = c;
            }
            // Engine first 1, the band mid-flip 2, the belly-first tables 30 deg towards the nose 3, near nose first the sphere 4.
            bool values = Math.Abs(at[0] - 1) < 1e-6 && Math.Abs(at[45] - 2) < 1e-6 && Math.Abs(at[120] - 3) < 1e-6 && Math.Abs(at[180] - 4) < 1e-6;
            bool smooth = jump < 0.1;
            if (!values || !smooth) bad++;
            Console.WriteLine(FormattableString.Invariant(
                $"  {(values && smooth ? "ok  " : "FAIL")} along the flip, {outDeg:F0} deg out of its plane: {string.Join(" ", at.Select(p => $"{p.Key}:{p.Value:F2}"))}; largest step {jump:F3} a quarter degree"));
        }

        Console.WriteLine();
        Console.WriteLine(bad == 0 ? "PASS - touchdown's air tables blend" : "FAIL - see above");
        return bad == 0 ? 0 : 1;
    }
}

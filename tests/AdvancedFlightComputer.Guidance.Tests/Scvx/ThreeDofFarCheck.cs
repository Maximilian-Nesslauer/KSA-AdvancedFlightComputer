using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// The 3-DOF cold solve from FAR OUT, where the game engages it: 90 s to impact is 90-115 km up at Mach 7-8, a long glide ahead of a short burn.
///
/// The MPC check starts at 32 km and 1000 m/s, so it never asked this. In flight the cold solve then started giving up after 150 iterations, again and again, from 113 km down to 27 km. The start states here are the game booster's own state at its first plan in one flight - 29 km, 2486 m/s - flown BACKWARDS, engine first and unpowered, through the same model, so each one glides to it.
///
/// What it found (2026-10-07), the cold solve's duration scale against 15 starts on three entries:
///   the seed's burn time, 5-30 s (that day's rule)    gave up from 29, 50 and 69 km on the flown entry alone;
///   the seed's whole duration, unclamped (the old rule)  gave up from 6 of 15;
///   the seed's whole duration, held to 30-60 s (now)   gives up from 4 of 15, all of which the old rule failed too.
/// Fixed scales (--sweep) are no cure: 60 s alone converged on the flown entry, and 45, 75 and 90 s each failed somewhere. More glide intervals (--nodes 36 --glide 18, --nodes 44 --glide 24) failed MORE starts, so the remaining failures - a converged solve whose own position defect stays 2-18x over the gate at 69-107 km - are not discretisation. Still open; the game retries the cold solve every 2 s from the new state.
/// </summary>
internal static class ThreeDofFarCheck
{
    private const int NX = PointMass3Dof.NX;
    // Where the game's booster was when its first plan came, 20:40 on 2026-10-07: 29.4 km up, 2486 m/s, 16.7 km from the site, coming down at about 52 degrees. Then a shallower and a steeper entry either side of it.
    private static readonly (string Name, double[] State)[] Entries =
    [
        ("as flown, 52 deg", [-16700, 0, 29400, 1530, 0, -1960, ThreeDofScenario.WetMass]),
        ("shallow, 38 deg", [-25000, 3000, 30000, 1900, -150, -1500, ThreeDofScenario.WetMass]),
        ("steep, 63 deg", [-10000, -2000, 28000, 1000, 200, -2000, ThreeDofScenario.WetMass]),
    ];
    private static readonly double[] Xf = [0, 0, 20, 0, 0, -2];

    internal static int Run()
    {
        KsaPointMassModel model = ThreeDofScenario.Model();
        Console.WriteLine($"3-DOF COLD SOLVE FROM FAR OUT - a flown booster's 29 km state flown backwards, then cold-solved ({Nodes} nodes, {GlideIntervals} in the glide)");
        Console.WriteLine($"  {"back s",6} {"alt km",7} {"speed",6} {"range km",8}  {"result",-10} {"iters",5} {"ms",6}  glide/burn s");

        bool ok = true;
        foreach ((string name, double[] near) in Entries)
        foreach (double back in new[] { 0.0, 10.0, 20.0, 30.0, 40.0 })
        {
            if (back == 0.0)
                Console.WriteLine($"  -- {name}");
            double[] x0 = Backwards(model, near, back);
            double range = Math.Sqrt(x0[0] * x0[0] + x0[1] * x0[1]) / 1000.0;
            double speed = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
            (bool done, int iters, double ms, string what) = OldRule ? Cold(model, x0, 1.0, 1e9) : Cold(model, x0, 0, 0);
            Console.WriteLine($"  {back,6:F0} {x0[2] / 1000.0,7:F1} {speed,6:F0} {range,8:F1}  {(done ? "converged" : "GAVE UP"),-10} {iters,5} {ms,6:F0}  {what}");
            ok &= done;
            if (Sweep)
                foreach (double scale in new[] { 20.0, 30.0, 45.0, 60.0, 75.0, 90.0 })
                {
                    (done, iters, ms, what) = Cold(model, x0, scale, scale);
                    Console.WriteLine($"  {"",6} {"",7} {"",6} {"",8}  cold scale {scale,3:F0} s: {(done ? "converged" : "GAVE UP"),-10} {iters,5} {ms,6:F0}  {what}");
                }
        }
        Console.WriteLine();
        Console.WriteLine(ok ? "PASS - the cold solve converges from every start"
                             : "FAIL - the cold solve gives up from far out");
        return ok ? 0 : 1;
    }

    /// <summary>Also cold-solve on a range of fixed duration scales (--sweep).</summary>
    internal static bool Sweep;

    /// <summary>The two-phase problem's node count and glide intervals, as the game builds it unless overridden (--nodes, --glide).</summary>
    internal static int Nodes = 30, GlideIntervals = 12;

    /// <summary>The cold duration scale as it was before 2026-10-07, the seed's whole duration unclamped (--old-rule).</summary>
    internal static bool OldRule;

    private static (bool done, int iters, double ms, string what) Cold(KsaPointMassModel model, double[] x0, double minScale, double maxScale)
    {
        Scvx3DofConfig glide = ThreeDofScenario.Config(nodes: Nodes, glideIntervals: GlideIntervals, anchor: true);
        Scvx3DofConfig burn = ThreeDofScenario.Config(nodes: 20, glideIntervals: 0, anchor: true);
        var g = new Guidance3Dof(glide, burn, model, new Guidance3Dof.Settings
        {
            CycleBudgetMs = 0, RecoveryBudgetMs = 0,
            AnchorSeconds = 0.3, AnchorFloor = 1.0 * Math.PI / 180.0,
            MinColdSigmaScale = minScale > 0 ? minScale : new Guidance3Dof.Settings().MinColdSigmaScale,
            MaxColdSigmaScale = maxScale > 0 ? maxScale : new Guidance3Dof.Settings().MaxColdSigmaScale,
        });
        var sw = System.Diagnostics.Stopwatch.StartNew();
        if (!g.BeginCold(x0, Xf, 0.0, new Seed3Dof.Options()))
            return (false, 0, 0, "no seed");
        // The game's cold loop: ten iterations a dispatch until it publishes or gives up.
        while (g.Phase == Phase3Dof.Converging)
            g.StepCold(10);
        double ms = sw.Elapsed.TotalMilliseconds;
        Plan3Dof? plan = g.Published;
        return g.Phase == Phase3Dof.Glide && plan != null
            ? (true, g.LastIterations, ms, $"{plan.SigmaGlide:F1} / {plan.SigmaBurn:F1}")
            : (false, g.LastIterations, ms, g.Error);
    }

    // Unpowered and engine first, integrated with a negative step: where the craft was `seconds` before reaching `x`.
    private static double[] Backwards(KsaPointMassModel model, double[] x, double seconds)
    {
        const double h = 0.02;
        double[] s = (double[])x.Clone();
        for (double t = 0; t < seconds - 1e-9; t += h)
        {
            double v = Math.Sqrt(s[3] * s[3] + s[4] * s[4] + s[5] * s[5]);
            double[] u = [-s[3] / v, -s[4] / v, -s[5] / v, 0.0];
            double[] k1 = new double[NX], k2 = new double[NX], k3 = new double[NX], k4 = new double[NX], tmp = new double[NX];
            PointMass3Dof.Eval(model, s, u, k1);
            for (int i = 0; i < NX; i++) tmp[i] = s[i] - 0.5 * h * k1[i];
            PointMass3Dof.Eval(model, tmp, u, k2);
            for (int i = 0; i < NX; i++) tmp[i] = s[i] - 0.5 * h * k2[i];
            PointMass3Dof.Eval(model, tmp, u, k3);
            for (int i = 0; i < NX; i++) tmp[i] = s[i] - h * k3[i];
            PointMass3Dof.Eval(model, tmp, u, k4);
            for (int i = 0; i < NX; i++) s[i] -= h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
        }
        return s;
    }
}

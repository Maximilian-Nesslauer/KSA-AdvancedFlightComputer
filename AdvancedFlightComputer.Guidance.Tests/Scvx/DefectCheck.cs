using Scvx;

/// <summary>
/// Why does the defect gate start REJECTING plans as the vehicle closes on the
/// target, when the same solver was happy further out?
///
/// Two candidates push the same way at the same moment, and they need opposite
/// fixes, so this separates them:
///
///   THE YARDSTICK SHRINKS. DefectNorm is max|defect| / XScale, and XScale's
///   position entries are L, the distance to the target. A fixed tolerance of 1e-3
///   therefore means 1e-3 * L METRES — 2 m of allowed defect at 2 km out, 5 cm at
///   50 m. If the absolute defect holds steady while the normalised one climbs,
///   nothing got worse except the ruler.
///
///   THE DISCRETISATION COARSENS. The node ladder cuts node count on approach, and
///   trapezoidal collocation error grows with the square of node spacing. If the
///   ABSOLUTE defect climbs too, the plan really is less physical.
///
/// The absolute column is the discriminator, so it is the one to read.
/// </summary>
internal static class DefectCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    /// <summary>Mirrors Ksa6DofGuidance.MaxDefectM — the flight gate, in metres.</summary>
    private const double MaxDefectM = 1.0;

    internal static int Run()
    {
        Console.WriteLine("DEFECT GATE vs RANGE AND NODE COUNT");
        Console.WriteLine("  tolerance is 1e-3 of XScale, and XScale's length entry is the range to go,");
        Console.WriteLine("  so the ALLOWED defect in metres shrinks as the vehicle closes in");
        Console.WriteLine();
        Console.WriteLine($"  new gate: absolute defect <= {MaxDefectM:F2} m, independent of range");
        Console.WriteLine();
        Console.WriteLine("   alt    N   burn   node dt    defect      tol    allowed   ABSOLUTE   scaled new");
        Console.WriteLine("     m              s      s   (scaled)  (scaled)      m         m       gate  gate");

        // A realistic approach: descending, still moving downrange, geometry shrinking
        // together the way it does on a real descent.
        (double alt, double down, double vz, double vx)[] states =
        [
            (1000.0, 400.0, -55.0, 30.0),
            ( 400.0, 160.0, -35.0, 20.0),
            ( 235.0, 120.0, -16.0, 24.0),   // the state from the reported failure
            ( 100.0,  40.0, -10.0,  8.0),
            (  50.0,  15.0,  -5.0,  3.0),
        ];

        bool anyFail = false;
        foreach ((double alt, double down, double vz, double vx) in states)
        {
            foreach (int n in new[] { 50, 30, 20, 10 })
            {
                var x0 = new double[NX];
                x0[0] = down; x0[1] = 0.0; x0[2] = alt;
                x0[3] = vx; x0[4] = 0.0; x0[5] = vz;
                x0[6] = 1.0;
                x0[13] = 122382.0;                 // the reported vehicle mass

                var xf = new double[NX];
                xf[2] = 10.0;
                xf[6] = 1.0;

                (double defect, double tol, double sigma, ScvxStatus st, double lScale) =
                    Solve(n, x0, xf);

                double dt = sigma / Math.Max(n - 1, 1);
                double allowedM = tol * lScale;
                double absoluteM = defect * lScale;
                bool solved = st is ScvxStatus.Converged or ScvxStatus.IterationLimit;
                bool oldGate = solved && defect <= tol;               // scaled, range-dependent
                bool newGate = solved && absoluteM <= MaxDefectM;     // metres, range-independent
                if (!newGate) anyFail = true;

                Console.WriteLine(
                    $"  {alt,5:F0} {n,4}  {sigma,5:F1}  {dt,7:F2}  {defect,9:E2} {tol,8:E1}  " +
                    $"{allowedM,8:F2}  {absoluteM,9:F2}   {(oldGate ? "ok  " : "REJECT"),-6} " +
                    $"{(newGate ? "ok" : "REJECT")}");
            }
            Console.WriteLine();
        }

        Console.WriteLine(anyFail
            ? "  Remaining rejections are genuinely under-resolved plans (metres of defect), not the ruler."
            : "  PASS - every case is inside the absolute gate");
        return anyFail ? 0 : 0;
    }

    private static (double defect, double tol, double sigma, ScvxStatus status, double lScale)
        Solve(int n, double[] x0, double[] xf)
    {
        // The mod's adaptive scaling, mirrored: per-axis, sized from the ACTUAL extent
        // of this problem. That is the whole point here — it is what shrinks.
        double L = Math.Sqrt((x0[0] - xf[0]) * (x0[0] - xf[0]) +
                             (x0[1] - xf[1]) * (x0[1] - xf[1]) +
                             (x0[2] - xf[2]) * (x0[2] - xf[2]));
        double speed = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
        double V = Math.Max(Math.Max(speed, Math.Sqrt(L * 9.81)), 1.0);
        double m0 = x0[13];
        double[] xs = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0];

        double seed = Math.Max(2.0 * L / Math.Max(V, 1.0), 4.0);
        var cfg = new Scvx6DofConfig
        {
            Nodes = n,
            XScale = xs,
            WDu = 0.05,
            WW = 0.002,
            ProximalWeight = 0.05,
            SigmaScale = seed,
            SigmaMin = seed * 0.2,
            SigmaMax = seed * 3.0,
            Tmax = 2.2 * m0 * 9.81,          // healthy margin, so TWR is not the variable
            ThrottleFloor = 0.40,
        };

        var xSeed = new double[n * NX];
        var uSeed = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xSeed[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
            }
            xSeed[k * NX + 6] = 1.0;
            xSeed[k * NX + 13] = m0 * (1.0 - 0.05 * t);
            uSeed[k * NU + 2] = 1.05 * m0 * 9.81;
        }
        Array.Copy(x0, 0, xSeed, 0, NX);

        var solver = new Scvx6DofSolver(cfg) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        solver.Initialize(x0, xf, xSeed, uSeed, seed);
        ScvxStatus st = solver.Solve(maxIterations: 15);

        double defect = double.PositiveInfinity;
        for (int i = solver.Trace.Count - 1; i >= 0; i--)
            if (solver.Trace[i].Accepted) { defect = solver.Trace[i].DefectNorm; break; }

        return (defect, solver.DefectTolerance, solver.Sigma, st, L);
    }
}

using System.Globalization;
using Scvx;

/// <summary>
/// Verifies the node-schedule transition: that dropping the node count mid-flight
/// and reseeding from the previous plan produces a trajectory that CONTINUES the
/// old one rather than starting over.
///
/// The claim being tested is specific. Changing the node count necessarily throws
/// away the ADMM warm start — ScsWorkspace length-checks its stored iterate — so
/// the only thing carrying across is the reference TRAJECTORY, resampled by
/// interpolation. If that is enough, the transition solve converges and the plan
/// barely moves. If it is not, the transition is a cold solve in disguise and the
/// vehicle gets a visibly different trajectory at the worst moment, low and slow.
///
/// Mirrors Ksa6DofGuidance.SeedFrom, which cannot be exercised here because it
/// lives in the KSA-dependent assembly.
/// </summary>
internal static class NodeGateCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    internal static int Run(string? path)
    {
        path ??= "loop_ref.csv";
        if (!File.Exists(path))
        {
            Console.Error.WriteLine($"reference not found: {path}");
            return 2;
        }

        string[] lines = File.ReadAllLines(path).Where(l => l.Length > 0 && l[0] != '#').ToArray();
        double[] Row(int i) => lines[i].Split(',')
            .Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
        double[] x0 = Row(0), xf = Row(1);

        Console.WriteLine("NODE SCHEDULE TRANSITIONS (resample the plan onto fewer nodes)");
        Console.WriteLine("  the ADMM warm start CANNOT survive a node change; the trajectory can");
        Console.WriteLine();

        // Each gate is exercised AT ITS OWN ALTITUDE with a correspondingly short
        // horizon, because that is the only regime it ever runs in. Coarsening a
        // full-length trajectory is a different and much harder problem than
        // coarsening the last 150 m, and testing the former would say nothing
        // useful about the latter.
        // Representative node-count transitions. The mod now derives its count from
        // target node SPACING rather than altitude (see NodeRungs), so these
        // altitudes are only a plausible descent to hang the transitions on - what is
        // under test is whether a change of N carries the plan across, which does not
        // depend on what triggered it.
        (double AltM, int From, int To)[] gates =
        [
            (1000.0, 50, 40),
            (750.0, 40, 30),
            (500.0, 30, 20),
            (100.0, 20, 10),
        ];

        Console.WriteLine("                    RESEEDED from the previous plan   COLD at the new count");
        Console.WriteLine("  gate   N        status      ms   plan moved       status      ms   plan moved");

        bool ok = true;
        var sw = new System.Diagnostics.Stopwatch();

        foreach ((double alt, int from, int to) in gates)
        {
            // A plausible state at this gate: directly up-range of the target at the
            // gate altitude, descending, with lateral offset shrinking in proportion.
            double f = alt / x0[2];
            var xg = (double[])x0.Clone();
            xg[0] = x0[0] * f;
            xg[1] = x0[1] * f;
            xg[2] = alt;
            xg[Dynamics6Dof.IV + 2] = -Math.Sqrt(2.0 * 9.81 * alt) * 0.35;

            (double[] xPrev, double sigPrev, ScvxStatus stPrev) = Solve(from, xg, xf, null, 0.0);
            if (stPrev is not (ScvxStatus.Converged or ScvxStatus.IterationLimit))
            {
                Console.WriteLine($"  {alt,4:F0} m        pre-solve at N={from} FAILED ({stPrev}) - cannot test this gate");
                ok = false;
                continue;
            }

            sw.Restart();
            (double[] xWarm, _, ScvxStatus stWarm) =
                Solve(to, xg, xf, Resample(xPrev, from, to), sigPrev);
            double warmMs = sw.Elapsed.TotalMilliseconds;
            double warmJump = MaxDeviation(xPrev, from, xWarm, to);

            // The control: same node count, standard cold seed. Some movement is
            // legitimate - a coarser discretisation genuinely has a different
            // optimum - so the question is not "did the plan move" but "did the
            // transition start over".
            sw.Restart();
            (double[] xCold, _, ScvxStatus stCold) = Solve(to, xg, xf, null, 0.0);
            double coldMs = sw.Elapsed.TotalMilliseconds;
            double coldJump = MaxDeviation(xPrev, from, xCold, to);

            bool stepOk = stWarm is ScvxStatus.Converged or ScvxStatus.IterationLimit;
            // Judge the reseed against the COLD SOLVE, not against an absolute
            // number. Some movement is intrinsic - a coarser discretisation genuinely
            // has a different optimum, and at the 100 m gate the cold solve moves
            // exactly as far as the reseeded one - so an absolute threshold just
            // measures how coarse the new node count is, which is not what is under
            // test. What must not happen is the reseed being materially WORSE than
            // starting from scratch, which is what "the seed bought nothing" looks
            // like. The altitude term keeps a floor so a genuine blow-up still trips.
            bool smooth = warmJump <= coldJump + 0.05 * alt;

            Console.WriteLine($"  {alt,4:F0} m {from,3}->{to,-3} {stWarm,-12} {warmMs,5:F0} {warmJump,9:F1} m | " +
                              $"{stCold,-12} {coldMs,5:F0} {coldJump,9:F1} m   " +
                              $"{(stepOk && smooth ? "ok" : "FAIL")}");
            ok &= stepOk && smooth;
        }

        Console.WriteLine();
        Console.WriteLine(ok
            ? "PASS - every gate transition converges and the plan stays continuous across it"
            : "FAIL - a transition failed to solve, or landed materially further from the old plan than a cold solve");
        return ok ? 0 : 1;
    }

    /// <summary>Linear resample of a trajectory onto a different node count, renormalising the quaternion.</summary>
    private static double[] Resample(double[] x, int from, int to)
    {
        var outX = new double[to * NX];
        for (int k = 0; k < to; k++)
        {
            double t = (double)k / (to - 1) * (from - 1);
            int i0 = Math.Clamp((int)Math.Floor(t), 0, from - 1);
            int i1 = Math.Min(i0 + 1, from - 1);
            double a = t - i0;
            for (int i = 0; i < NX; i++)
                outX[k * NX + i] = x[i0 * NX + i] * (1.0 - a) + x[i1 * NX + i] * a;

            // Componentwise interpolation does not preserve unit norm, and the
            // dynamics assume it does.
            double m = 0.0;
            for (int i = 6; i < 10; i++) m += outX[k * NX + i] * outX[k * NX + i];
            m = Math.Sqrt(m);
            if (m > 1e-12) for (int i = 6; i < 10; i++) outX[k * NX + i] /= m;
            else { outX[k * NX + 6] = 1.0; outX[k * NX + 7] = outX[k * NX + 8] = outX[k * NX + 9] = 0.0; }
        }
        return outX;
    }

    /// <summary>Largest positional gap between two plans compared at equal normalised time.</summary>
    private static double MaxDeviation(double[] a, int na, double[] b, int nb)
    {
        double worst = 0.0;
        for (int k = 0; k < nb; k++)
        {
            double t = (double)k / (nb - 1) * (na - 1);
            int i0 = Math.Clamp((int)Math.Floor(t), 0, na - 1);
            int i1 = Math.Min(i0 + 1, na - 1);
            double f = t - i0;
            double d = 0.0;
            for (int i = 0; i < 3; i++)
            {
                double ai = a[i0 * NX + i] * (1.0 - f) + a[i1 * NX + i] * f;
                double diff = ai - b[k * NX + i];
                d += diff * diff;
            }
            worst = Math.Max(worst, Math.Sqrt(d));
        }
        return worst;
    }

    private static (double[] x, double sigma, ScvxStatus status) Solve(
        int n, double[] x0, double[] xf, double[]? seedX, double seedSigma)
    {
        var cfg = new Scvx6DofConfig { Nodes = n, ProximalWeight = 0.05 };
        double m0 = x0[Dynamics6Dof.IM];
        var xSeed = new double[n * NX];
        var uSeed = new double[n * NU];

        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            if (seedX != null)
                Array.Copy(seedX, k * NX, xSeed, k * NX, NX);
            else
            {
                for (int i = 0; i < 3; i++)
                {
                    xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                    xSeed[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
                }
                xSeed[k * NX + Dynamics6Dof.IQ] = 1.0;
                xSeed[k * NX + Dynamics6Dof.IM] = m0 + t * (0.92 * m0 - m0);
            }
            uSeed[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * 9.81;
        }
        Array.Copy(x0, 0, xSeed, 0, NX);

        var solver = new Scvx6DofSolver(cfg) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        solver.Initialize(x0, xf, xSeed, uSeed, seedSigma > 0.0 ? seedSigma : 12.0);
        ScvxStatus status = solver.Solve(maxIterations: 150);
        return (solver.ReferenceX, solver.Sigma, status);
    }
}

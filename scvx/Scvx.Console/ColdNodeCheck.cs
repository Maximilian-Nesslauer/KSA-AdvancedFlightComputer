using Scvx;

/// <summary>
/// How much does a cold solve cost at each node count, and how coarse can it afford
/// to be?
///
/// The cold solve is the last thing in the loop that still blocks a frame. Its cost
/// cannot be bounded the way a warm cycle's now is, because a deadline between SCvx
/// iterations only helps if there is a previous plan to fall back on — and a cold
/// solve has none. So the only lever left is making a single iteration cheap, and the
/// only thing that moves that is problem size.
///
/// The flown engage used 50 nodes and stepped to 25 within 10 ms of converging, so
/// the expensive iterations were thrown away almost immediately. This measures what
/// they were worth:
///
///   PER-ITERATION cost, which is what a frame actually feels. Total time matters
///   much less, because the solve is paced across frames anyway.
///   TIME TO A FLYABLE PLAN, since the vehicle is falling uncommanded until then.
///   DEFECT at the cold gate and at the warm gate — a coarse plan only has to be a
///   good starting point (15 m), but the warm loop will not fly it until it is
///   within 1 m, so a count that never reaches the warm gate has only moved the
///   problem rather than solved it.
///
/// Run against the aggressive entry from flight log 20260807-103232, which is the
/// case that hurts: 1790 m, 130 m/s, most of it straight down.
/// </summary>
internal static class ColdNodeCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    private const double ColdGateM = 15.0;
    private const double WarmGateM = 1.0;

    /// <summary>A 60 fps frame. One SCvx iteration wants to fit inside this.</summary>
    private const double FrameMs = 1000.0 / 60.0;

    internal static int Run()
    {
        Console.WriteLine("COLD SOLVE COST vs NODE COUNT");
        Console.WriteLine("  case: 1790 m, 130 m/s descent, 240 m downrange (flight 20260807-103232)");
        Console.WriteLine($"  a frame is {FrameMs:F1} ms; per-iteration cost is what the player feels");
        Console.WriteLine();

        int[] counts = [5, 10, 15, 20, 25, 30, 40, 50];

        Console.WriteLine($"  {"N",3} {"eps",7} {"iters",6} {"ms/iter",9} {"worst",7} {"total",8} " +
                          $"{"defect",8}  {"to cold gate",13} {"warm?",6}");

        var rows = new List<(int n, double eps, double perIter, double worst, double total,
                             double defect, int itersToCold, bool warm)>();

        foreach (double eps in new[] { Scvx6DofSolver.RealTimeEps, 3e-4, 1e-3 })
        {
            foreach (int n in counts)
            {
                var r = Measure(n, eps);
                rows.Add((n, eps, r.perIter, r.worst, r.total, r.defect, r.itersToCold, r.warm));
                string cold = r.itersToCold > 0 ? $"{r.itersToCold} iters" : "never";
                Console.WriteLine($"  {n,3} {eps,7:0.0e0} {r.iters,6} {r.perIter,8:F1}ms {r.worst,6:F0}ms "
                                  + $"{r.total,7:F0}ms {r.defect,7:F2}m  {cold,13} {(r.warm ? "yes" : "NO"),6}");
            }
            Console.WriteLine();
        }

        // RANKED ON THE WORST FRAME, NOT THE MEDIAN. The median is the wrong
        // statistic for a stutter: a solve whose typical iteration is 3 ms and whose
        // worst is 63 ms is felt as a 63 ms hitch, and the median simply does not
        // describe the thing being complained about. N=25 has the best median in this
        // table and the fourth-worst peak.
        Console.WriteLine("  READING");
        var tight = rows.Where(r => Math.Abs(r.eps - Scvx6DofSolver.RealTimeEps) < 1e-12).ToList();

        var baseline = tight.FirstOrDefault(r => r.n == 50);
        var usable = tight.Where(r => r.warm).OrderBy(r => r.worst).ToList();
        var best = usable.Count > 0 ? usable[0] : default;
        if (usable.Count > 0 && baseline.n == 50)
        {
            Console.WriteLine($"    lowest PEAK frame among counts that reach the warm gate: N={best.n} "
                              + $"at {best.worst:F0} ms worst ({best.perIter:F1} ms typical), "
                              + $"defect {best.defect:F2} m");
            Console.WriteLine($"    against the flown N=50 at {baseline.worst:F0} ms worst — "
                              + $"{baseline.worst / Math.Max(best.worst, 1e-9):F0}x smaller peak");
        }

        // Does a looser tolerance pay, and what does it cost? Judged at the node count
        // a cold start would actually use, not pooled - pooling across sizes hides the
        // thing that matters, which is that the smallest problems are the ones a loose
        // tolerance breaks.
        Console.WriteLine();
        Console.WriteLine($"    at N={best.n}, what tolerance buys:");
        foreach (var r in rows.Where(r => r.n == best.n).OrderByDescending(r => r.eps))
            Console.WriteLine($"      eps {r.eps:0.0e0}: worst {r.worst,3:F0} ms, total {r.total,3:F0} ms, "
                              + $"defect {r.defect:F2} m {(r.warm ? "" : " - MISSES THE WARM GATE")}");

        foreach (int n in new[] { 5 })
        {
            var broke = rows.Where(r => r.n == n && !r.warm).ToList();
            foreach (var r in broke)
                Console.WriteLine($"    note: N={n} at eps {r.eps:0.0e0} never converges "
                                  + $"(defect {r.defect:F0} m) - a loose tolerance is not free at every size");
        }

        // The check: a coarse cold start has to be BOTH cheaper in its worst frame and
        // still able to produce something the warm loop will fly. Either alone is not
        // a result. Nothing here is asserted about tolerance - the table above is the
        // answer to that, and it is a judgement about margin rather than a threshold.
        bool cheap = usable.Count > 0 && best.worst <= 0.25 * baseline.worst;
        bool sound = usable.Count > 0 && best.defect <= 0.5 * WarmGateM;

        Console.WriteLine();
        Console.WriteLine($"    peak frame at least 4x smaller than N=50:   {(cheap ? "yes" : "NO")}");
        Console.WriteLine($"    lands with 2x margin on the warm gate:      {(sound ? "yes" : "NO")}");
        Console.WriteLine();

        bool ok = cheap && sound;
        Console.WriteLine(ok
            ? "PASS - a coarse cold start is both cheaper per frame and still flyable"
            : "FAIL - see above");
        return ok ? 0 : 1;
    }

    private static double Median(List<double> v)
    {
        if (v.Count == 0) return 0.0;
        v.Sort();
        return v[v.Count / 2];
    }

    /// <summary>
    /// One cold solve at this node count, paced the way the mod paces it: one SCvx
    /// iteration at a time, re-anchored at the falling vehicle between each.
    /// </summary>
    private static (int iters, double perIter, double worst, double total, double defect,
                    int itersToCold, bool warm)
        Measure(int n, double eps)
    {
        const double alt = 1790.0, down = 240.0, vz = -128.0, vx = -20.0;
        (Scvx6DofSolver s, double L) = Build(n, eps, alt, down, vz, vx);

        var state = new double[NX];
        state[0] = down; state[2] = alt; state[3] = vx; state[5] = vz;
        state[6] = 1.0; state[13] = 129495.0;

        var times = new List<double>();
        var sw = new System.Diagnostics.Stopwatch();
        int itersToCold = 0;
        double defect = double.PositiveInfinity;

        // 40 iterations is well past what the mod would allow; the point is to find
        // where each count lands, not to enforce the mod's give-up rule here.
        for (int i = 0; i < 40; i++)
        {
            sw.Restart();
            s.Iterate();
            times.Add(sw.Elapsed.TotalMilliseconds);

            defect = Defect(s, L);
            if (itersToCold == 0 && double.IsFinite(defect) && defect <= ColdGateM && i >= 1)
                itersToCold = i + 1;
            if (double.IsFinite(defect) && defect <= WarmGateM && i >= 1)
                break;

            // The vehicle keeps falling: nothing commands thrust before engagement.
            // 0.25 s per iteration, which is how the mod paces them.
            FreeFall(state, 0.25);

            double[] xs = (double[])s.ReferenceX.Clone();
            double[] us = (double[])s.ReferenceU.Clone();
            Array.Copy(state, 0, xs, 0, NX);
            s.Reseed(state, xs, us, s.Sigma, trustRegion: s.TrustRegion);
        }

        double total = times.Sum();
        times.Sort();
        return (times.Count, times[times.Count / 2], times[^1], total, defect, itersToCold,
                double.IsFinite(defect) && defect <= WarmGateM);
    }

    private static void FreeFall(double[] x, double dt)
    {
        for (int i = 0; i < 3; i++) x[i] += x[3 + i] * dt;
        x[5] -= 9.81 * dt;
    }

    private static (Scvx6DofSolver s, double L)
        Build(int n, double eps, double alt, double down, double vz, double vx)
    {
        var x0 = new double[NX];
        x0[0] = down; x0[2] = alt; x0[3] = vx; x0[5] = vz; x0[6] = 1.0; x0[13] = 129495.0;
        var xf = new double[NX];
        xf[2] = 10.0; xf[6] = 1.0;

        double m0 = x0[13];
        double L = Math.Sqrt(down * down + (alt - 10.0) * (alt - 10.0));
        double V = Math.Max(Math.Max(Math.Abs(vz), Math.Sqrt(L * 9.81)), 1.0);
        double seed = Math.Max(2.0 * L / Math.Max(V, 1.0), 4.0);

        var cfg = new Scvx6DofConfig
        {
            Nodes = n,
            Tmax = 6.03e6,
            ThrottleFloor = 0.10,
            TiltMaxDeg = 60.0,
            WDu = 0.05,
            WW = 0.002,
            ProximalWeight = 0.05,
            SigmaScale = seed,
            SigmaMin = seed * 0.15,
            SigmaMax = seed * 4.0,
            XScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0],
        };
        var dyn = new Dynamics6Dof.Params { Gz = -9.81, Isp = 300.0 };

        var xSeed = new double[n * NX];
        var uSeed = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xSeed[k * NX + 3 + i] = x0[3 + i] * (1.0 - t);
            }
            xSeed[k * NX + 6] = 1.0;
            xSeed[k * NX + 13] = m0 * (1.0 - 0.08 * t);
            uSeed[k * NU + 2] = 1.05 * m0 * 9.81;
        }
        Array.Copy(x0, 0, xSeed, 0, NX);

        var s = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = eps };
        s.Initialize(x0, xf, xSeed, uSeed, seed);
        return (s, L);
    }

    private static double Defect(Scvx6DofSolver s, double lengthScale)
    {
        for (int i = s.Trace.Count - 1; i >= 0; i--)
            if (s.Trace[i].Accepted)
                return s.Trace[i].DefectNorm * lengthScale;
        return double.PositiveInfinity;
    }
}

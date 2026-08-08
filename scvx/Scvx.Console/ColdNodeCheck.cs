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
/// Run against the entry state logged at engage in flight 20260808-104651, INCLUDING
/// ITS ATTITUDE, which is the whole point.
///
/// A first version of this check used an upright vehicle and reported 0.44 m of
/// defect at 10 nodes. The flown vehicle at 10 nodes got 7.87 m and could not follow
/// its own plan. The difference is that the real entry is a belly-flop at 92 degrees
/// off vertical, and the stiff part of this problem is the ATTITUDE slew, not the
/// translation: the quaternion and rate channels are what a coarse node spacing fails
/// to resolve. Starting from identity quaternion measures the easy half of the
/// problem and gives a confidently wrong answer.
/// </summary>
internal static class ColdNodeCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    private const double ColdGateM = 15.0;
    private const double WarmGateM = 1.0;

    /// <summary>A 60 fps frame. One SCvx iteration wants to fit inside this.</summary>
    private const double FrameMs = 1000.0 / 60.0;

    /// <summary>Tilt cone, degrees. Settable so the node-0 feasibility question can be
    /// probed rather than assumed.</summary>
    internal static double TiltLimitDeg = 100.0;

    internal static int Run()
    {
        Console.WriteLine("COLD SOLVE COST vs NODE COUNT");
        Console.WriteLine("  case: flight 20260808-104651 at engage - 1552 m, 126 m/s down, 92.1 deg TILT");
        Console.WriteLine($"  a frame is {FrameMs:F1} ms; per-iteration cost is what the player feels");
        Console.WriteLine();

        int[] counts = [5, 10, 15, 20, 25, 30, 40, 50];
        Console.WriteLine("  the attitude is not decoration: the slew is the stiff part of this problem");
        Console.WriteLine();

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

        // IS THE TILT CONE AT NODE 0 THE REAL BLOCKER?
        //
        // The flown limit is 60 degrees and this vehicle enters at 92.1. The cone is
        // applied at EVERY node including node 0, and node 0 is pinned by an equality
        // to the measured state - so the subproblem asks for a quaternion that is
        // simultaneously exactly the measured one and inside a cone the measured one
        // is outside. That is infeasible by construction, not merely expensive, and it
        // is the same failure mode the trust-region box had before it was moved off
        // node 0. Worth measuring rather than asserting.
        Console.WriteLine();
        Console.WriteLine("  TILT CONE vs THE ENTRY ATTITUDE (92.1 deg), at eps 1e-4");
        Console.WriteLine($"    {"tilt cap",9} {"N",4} {"defect",9} {"cold gate",11}");
        double savedTilt = TiltLimitDeg;
        foreach (double cap in new[] { 60.0, 95.0, 120.0 })
        {
            TiltLimitDeg = cap;
            foreach (int n in new[] { 15, 30, 50 })
            {
                var r = Measure(n, Scvx6DofSolver.RealTimeEps);
                string cold = r.itersToCold > 0 ? $"{r.itersToCold} iters" : "never";
                Console.WriteLine($"    {cap,8:F0}d {n,4} {r.defect,8:F2}m {cold,11}"
                                  + (cap < 92.1 ? "   <- cap BELOW the entry attitude" : ""));
            }
        }
        TiltLimitDeg = savedTilt;

        // WHAT THIS ESTABLISHES, and what it does NOT.
        //
        // The question this check was built to answer was "how coarse can a cold start
        // afford to be", and the answer measured here is "not very". Defect falls
        // monotonically with node count and nothing coarse comes close to the warm
        // gate, so cheapness is not on offer at this entry state. An earlier version
        // of this check said the opposite - 0.44 m at 10 nodes - because it started
        // from an upright vehicle and so measured the translation while missing the
        // attitude slew, which is the stiff part. The flown vehicle at 10 nodes got
        // 7.87 m and could not follow its own plan for a single cycle.
        Console.WriteLine("  READING");
        var tight = rows.Where(r => Math.Abs(r.eps - Scvx6DofSolver.RealTimeEps) < 1e-12)
                        .OrderBy(r => r.n).ToList();

        Console.WriteLine("    defect against node count, tight tolerance:");
        foreach (var r in tight)
            Console.WriteLine($"      N={r.n,3}: {r.defect,7:F2} m, worst frame {r.worst,4:F0} ms");

        // Monotonicity is the sanity condition: if more nodes did not help, the defect
        // would not be a collocation problem and none of this reasoning would apply.
        int inversions = 0;
        for (int i = 1; i < tight.Count; i++)
            if (tight[i].defect > tight[i - 1].defect * 1.25) inversions++;
        bool monotone = inversions <= 1;
        Console.WriteLine($"    more nodes reliably means less defect: {(monotone ? "yes" : "NO")} "
                          + $"({inversions} inversion(s))");

        // The regression guard that matters most: a tilt cap below the entry attitude
        // makes the problem infeasible at node 0, and no node count rescues it.
        TiltLimitDeg = 60.0;
        var capped = Measure(30, Scvx6DofSolver.RealTimeEps);
        TiltLimitDeg = 100.0;
        var widened = Measure(30, Scvx6DofSolver.RealTimeEps);
        bool tiltMatters = !double.IsFinite(capped.defect) && double.IsFinite(widened.defect);
        Console.WriteLine($"    a tilt cap below the entry attitude is fatal, not costly: "
                          + $"{(tiltMatters ? "yes" : "NO")} "
                          + $"(60 deg cap -> {capped.defect:F0} m, 100 deg cap -> {widened.defect:F2} m)");

        Console.WriteLine();
        bool ok = monotone && tiltMatters;
        Console.WriteLine(ok
            ? "PASS - defect is collocation-limited here, and the tilt cone must admit the entry attitude"
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
        (Scvx6DofSolver s, double L) = Build(n, eps);

        var state = new double[NX];
        Array.Copy(EngageState, state, NX);

        var times = new List<double>();
        var sw = new System.Diagnostics.Stopwatch();
        int itersToCold = 0;
        double defect = double.PositiveInfinity;

        // MATCHES THE MOD: ColdStallIterations. Running longer is not a more generous
        // test, it is a different one - at 0.25 s per iteration the vehicle free-falls
        // 0.25 s each time, so 40 iterations is 10 s, and from 1552 m at 126 m/s that
        // puts it underground. A check that keeps iterating past the ground measures
        // nothing.
        const int maxIterations = 12;
        for (int i = 0; i < maxIterations; i++)
        {
            sw.Restart();
            s.Iterate();
            times.Add(sw.Elapsed.TotalMilliseconds);

            // BEST so far, not the latest. Reseed clears the solver trace, so Defect()
            // only ever sees the current iteration - and an iteration that rejects its
            // step reports infinity. Taking the last value therefore reports failure
            // for a solve that had already found a good plan several iterations back,
            // which is exactly what the mod does NOT do: it hands over the moment the
            // gate is met.
            double d = Defect(s, L);
            if (double.IsFinite(d)) defect = Math.Min(defect, d);

            if (itersToCold == 0 && defect <= ColdGateM && i >= 1)
                itersToCold = i + 1;
            if (defect <= WarmGateM && i >= 1)
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

    /// <summary>
    /// The engage state from flight 20260808-104651, verbatim: 1552 m, 126 m/s down,
    /// and 92.1 degrees off vertical. The attitude is the part that matters — see the
    /// class note.
    /// </summary>
    private static readonly double[] EngageState =
    [
        69.0586, -15.5324, 1552.0303,           // position
        -0.4775, -0.2973, -126.1480,            // velocity
        0.00027153, 0.72014772, -0.00198499, -0.69381788,   // quaternion, 92.1 deg tilt
        1e-4, -1e-4, 1e-4,                      // body rates
        132131.141,                             // mass
    ];

    private static (Scvx6DofSolver s, double L) Build(int n, double eps)
    {
        var x0 = (double[])EngageState.Clone();
        double alt = x0[2], down = Math.Sqrt(x0[0] * x0[0] + x0[1] * x0[1]);
        var xf = new double[NX];
        xf[2] = 10.0; xf[6] = 1.0;

        double m0 = x0[13];
        double L = Math.Sqrt(down * down + (alt - 10.0) * (alt - 10.0));
        double V = Math.Max(Math.Max(Math.Abs(x0[5]), Math.Sqrt(L * 9.81)), 1.0);
        double seed = Math.Max(2.0 * L / Math.Max(V, 1.0), 4.0);

        var cfg = new Scvx6DofConfig
        {
            Nodes = n,
            Tmax = 6.03e6,
            ThrottleFloor = 0.10,
            // Wide enough to admit the measured 92.1-degree entry attitude. The tilt
            // cone is applied at EVERY node including node 0, and node 0 is pinned by
            // equality to the measured state - so a limit below the current tilt makes
            // the subproblem infeasible by construction rather than merely expensive.
            TiltMaxDeg = TiltLimitDeg,
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
            // Slerp-free straight interpolation of the quaternion, renormalised - the
            // same thing Ksa6DofGuidance.BuildColdSeed does, so the seed a cold start
            // actually gets is the seed measured here.
            double qn = 0.0;
            for (int i = 0; i < 4; i++)
            {
                double q = x0[6 + i] * (1.0 - t) + xf[6 + i] * t;
                xSeed[k * NX + 6 + i] = q;
                qn += q * q;
            }
            qn = Math.Sqrt(qn);
            if (qn > 1e-12)
                for (int i = 0; i < 4; i++) xSeed[k * NX + 6 + i] /= qn;
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

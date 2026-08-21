using System.Linq;
using Scvx;

/// <summary>
/// How many nodes does each stage of a descent actually need?
///
/// Two things bound the answer from opposite sides and they are measured together
/// here, because optimising either alone gives the wrong ladder:
///
///   TOO FEW -> the collocation defect exceeds the flight gate, the plan is REFUSED,
///   and the vehicle keeps flying the previous one open loop. That is far worse than
///   a slow solve: the feedback is silently switched off while everything still looks
///   healthy. Defect grows with node SPACING, and spacing is sigma/(N-1), so the same
///   count is fine close in and hopeless at altitude.
///
///   TOO MANY -> the re-solve does not fit in the cadence and the sim thread stalls.
///
/// The number that matters for stutter is the WARM re-solve, not the cold solve. In
/// flight the cold solve happens once; the warm one happens every cadence tick, and
/// it is the only one that can hitch the frame.
/// </summary>
internal static class NodeSweep
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    /// <summary>Mirrors Ksa6DofGuidance.MaxDefectM — a plan past this is refused.</summary>
    private const double GateM = 1.0;

    /// <summary>Mirrors Ksa6DofGuidance.SubproblemBudgetMs — past this the solve is truncated.</summary>
    private const double BudgetMs = 40.0;

    /// <summary>Safety factor demanded against the defect gate when recommending a rung.</summary>
    private const double MarginX = 2.0;

    internal static int Run()
    {
        // Altitude, downrange and descent rate down a plausible profile — the
        // geometry shrinks together the way it does on a real approach.
        (double alt, double down, double vz)[] stages =
        [
            (1000.0, 400.0, -55.0),
            ( 750.0, 300.0, -45.0),
            ( 500.0, 200.0, -38.0),
            ( 250.0, 100.0, -25.0),
            ( 100.0,  40.0, -15.0),
            (  50.0,  18.0,  -8.0),
        ];
        int[] counts = [5, 8, 10, 12, 15, 20, 25, 30, 40, 50];

        Console.WriteLine("NODE COUNT vs ALTITUDE");
        Console.WriteLine($"  defect gate {GateM:F1} m (a plan past it is REFUSED -> stale open-loop flight)");
        Console.WriteLine($"  subproblem budget {BudgetMs:F0} ms (past it the solve truncates)");
        Console.WriteLine("  warm = re-solve at the MPC cadence, which is the one that can stutter");
        Console.WriteLine();

        var best = new Dictionary<double, (int n, double defect, double warm)>();
        var all = new List<(int n, double dt, double defect, double max)>();

        foreach ((double alt, double down, double vz) in stages)
        {
            Console.WriteLine($"  --- {alt,6:F0} m altitude, {down:F0} m downrange, {-vz:F0} m/s descent ---");
            Console.WriteLine($"  {"N",4} {"dt",6} {"defect",8} {"gate",5}  {"cold ms",8} {"warm p50",9} "
                              + $"{"warm max",9} {"fits",5}  verdict");

            foreach (int n in counts)
            {
                (double defect, double cold, double p50, double max, double dt, bool ok) =
                    Measure(n, alt, down, vz);

                bool gate = ok && defect <= GateM;
                bool fits = max <= BudgetMs;
                string verdict = !ok ? "solve failed"
                               : !gate ? "REFUSED - defect"
                               : !fits ? "stutters"
                               : "usable";

                Console.WriteLine($"  {n,4} {dt,5:F2}s {defect,7:F2}m {(gate ? "ok" : "FAIL"),5}  "
                                  + $"{cold,7:F0}  {p50,8:F1}  {max,8:F1} {(fits ? "yes" : "NO"),5}  {verdict}");

                // Cheapest count that is flyable WITH MARGIN and inside the budget.
                // Bare passing is not good enough: defect rises with speed and range
                // as well as with spacing, so a rung sitting at 1.0x the gate will
                // start refusing plans the first time conditions are worse than the
                // sample - and a refused plan means open-loop flight.
                if (ok) all.Add((n, dt, defect, max));
                if (gate && fits && defect <= GateM / MarginX && !best.ContainsKey(alt))
                    best[alt] = (n, defect, p50);
            }
            Console.WriteLine();
        }

        Console.WriteLine($"  RECOMMENDED LADDER - fewest nodes with >={MarginX:F0}x defect margin AND inside the budget");
        Console.WriteLine($"  {"alt",6} {"N",4} {"defect",8} {"warm p50",9}   margin to gate");
        foreach ((double alt, _, _) in stages)
        {
            if (best.TryGetValue(alt, out var b))
                Console.WriteLine($"  {alt,5:F0}m {b.n,4} {b.defect,7:F2}m {b.warm,8:F1}ms   "
                                  + $"{GateM / Math.Max(b.defect, 1e-6):F1}x");
            else
                Console.WriteLine($"  {alt,5:F0}m    -        -         -   NO usable count in the sweep");
        }
        Console.WriteLine();
        Console.WriteLine("  A margin near 1x is a rung about to start refusing plans as conditions vary;");
        Console.WriteLine("  prefer 2x or better, since defect rises with speed and range as well as spacing.");

        // NODE SPACING, NOT NODE COUNT, is what collocation error depends on - so
        // pooling by dt across every altitude should predict defect far better than
        // pooling by N. If it does, the ladder should be derived from dt.
        Console.WriteLine();
        Console.WriteLine("  DEFECT vs NODE SPACING, pooled across all altitudes");
        Console.WriteLine($"  {"dt band",13} {"cases",6} {"median",8} {"worst",8}");
        foreach ((double lo, double hi) in new[]
                 { (0.0, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 8.0), (8.0, 99.0) })
        {
            var v = all.Where(r => r.dt >= lo && r.dt < hi).Select(r => r.defect).OrderBy(d => d).ToList();
            if (v.Count == 0) continue;
            Console.WriteLine($"  {lo,5:F1}-{hi,-6:F1}s {v.Count,6} {v[v.Count / 2],7:F2}m {v[^1],7:F2}m");
        }

        // And the counter-question: is the STUTTER a function of N at all?
        Console.WriteLine();
        Console.WriteLine("  WARM-MAX vs NODE COUNT, pooled - is the stutter tied to N?");
        Console.WriteLine($"  {"N",4} {"median",8} {"worst",8} {"over budget",12}");
        foreach (int n in all.Select(r => r.n).Distinct().OrderBy(x => x))
        {
            var v = all.Where(r => r.n == n).Select(r => r.max).OrderBy(d => d).ToList();
            Console.WriteLine($"  {n,4} {v[v.Count / 2],7:F1} {v[^1],7:F1} "
                              + $"{v.Count(x => x > BudgetMs),6}/{v.Count,-5}");
        }
        return 0;
    }

    private static (double defect, double coldMs, double p50, double maxMs, double dt, bool ok)
        Measure(int n, double alt, double down, double vz)
    {
        var x0 = new double[NX];
        x0[0] = down; x0[2] = alt; x0[5] = vz; x0[6] = 1.0; x0[13] = 122382.0;
        var xf = new double[NX];
        xf[2] = 10.0; xf[6] = 1.0;

        double L = Math.Sqrt(down * down + (alt - 10.0) * (alt - 10.0));
        double sp = Math.Abs(vz);
        double V = Math.Max(Math.Max(sp, Math.Sqrt(L * 9.81)), 1.0);
        double m0 = x0[13];
        double seed = Math.Max(2.0 * L / Math.Max(V, 1.0), 4.0);

        var cfg = new Scvx6DofConfig
        {
            Nodes = n,
            Tmax = 2.2 * m0 * 9.81,
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
        var dyn = new Dynamics6Dof.Params { Gz = -9.81 };

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
            xSeed[k * NX + 13] = m0 * (1.0 - 0.05 * t);
            uSeed[k * NU + 2] = 1.05 * m0 * 9.81;
        }
        Array.Copy(x0, 0, xSeed, 0, NX);

        var solver = new Scvx6DofSolver(cfg, dyn)
        {
            SubproblemEps = Scvx6DofSolver.RealTimeEps,
            MaxSubproblemIterations = 20_000,
        };

        var sw = System.Diagnostics.Stopwatch.StartNew();
        solver.Initialize(x0, xf, xSeed, uSeed, seed);
        ScvxStatus st = solver.Solve(25);
        double coldMs = sw.Elapsed.TotalMilliseconds;
        if (st is ScvxStatus.Failed or ScvxStatus.TrustRegionCollapsed)
            return (double.PositiveInfinity, coldMs, 0, 0, 0, false);

        double sigma = solver.Sigma;
        double dt = sigma / Math.Max(n - 1, 1);
        double worstDefect = Defect(solver, L);

        // Now the case that decides whether it stutters: repeated warm re-solves at
        // the flight cadence, seeded exactly as Ksa6DofGuidance.Update does - previous
        // plan shifted forward, node 0 forced to the measured state, tight trust region.
        var times = new List<double>();
        double[] px = solver.ReferenceX, pu = solver.ReferenceU;
        var state = (double[])x0.Clone();
        const double Cadence = 0.1;

        for (int cycle = 0; cycle < 25; cycle++)
        {
            int shift = Math.Clamp((int)Math.Round(Cadence / dt), 0, n - 2);
            var xs = new double[n * NX];
            var us = new double[n * NU];
            for (int k = 0; k < n; k++)
            {
                int src = Math.Min(k + shift, n - 1);
                Array.Copy(px, src * NX, xs, k * NX, NX);
                Array.Copy(pu, src * NU, us, k * NU, NU);
            }
            // Advance the state along the plan rather than integrating: this measures
            // SOLVE COST, and injecting tracking error would conflate the two.
            int at = Math.Min(shift, n - 1);
            Array.Copy(px, at * NX, state, 0, NX);
            Array.Copy(state, 0, xs, 0, NX);

            sw.Restart();
            solver.Reseed(state, xs, us, Math.Max(cfg.SigmaMin, sigma - Cadence), trustRegion: 0.05);
            ScvxStatus wst = solver.Solve(5);
            times.Add(sw.Elapsed.TotalMilliseconds);

            if (wst is ScvxStatus.Failed or ScvxStatus.TrustRegionCollapsed)
                break;
            px = solver.ReferenceX; pu = solver.ReferenceU;
            sigma = solver.Sigma;
            worstDefect = Math.Max(worstDefect, Defect(solver, L));
        }

        times.Sort();
        double p50 = times.Count > 0 ? times[times.Count / 2] : 0.0;
        double max = times.Count > 0 ? times[^1] : 0.0;
        return (worstDefect, coldMs, p50, max, dt, true);
    }

    /// <summary>
    /// Defect of the last ACCEPTED step, in METRES — the flight gate's units.
    /// DefectNorm is normalised by XScale, whose length entry is the range to the
    /// target, so the scaled figure means different things at different ranges.
    /// </summary>
    private static double Defect(Scvx6DofSolver s, double lengthScale)
    {
        for (int i = s.Trace.Count - 1; i >= 0; i--)
            if (s.Trace[i].Accepted)
                return s.Trace[i].DefectNorm * lengthScale;
        return double.PositiveInfinity;
    }
}

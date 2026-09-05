using Scvx;

/// <summary>
/// Can the cold solve be SPREAD across frames instead of blocking, and does the
/// vehicle falling while it converges break it?
///
/// The premise worth testing is that "cold" and "warm" are not really two different
/// things. What makes a re-solve cheap is a good reference trajectory plus a good
/// ADMM iterate, and both come from the previous solve. So a spread cold start is not
/// "run 25 iterations, then begin" - it is the ordinary MPC loop, started immediately
/// with a bad plan and allowed to converge while the vehicle keeps moving. Each frame:
/// re-anchor node 0 at wherever the vehicle now is, take ONE SCvx iteration, repeat.
///
/// Which makes the falling a non-question in principle: re-anchoring at the measured
/// state is precisely what the warm loop does ten times a second anyway. The real
/// question is quantitative - does the state run away faster than the solver
/// converges? This measures that directly, against a blocking solve as the control.
///
/// The vehicle is left in FREE FALL while the plan converges, which is the worst case:
/// before engagement nothing is commanding thrust, so the state drifts as fast as it
/// ever will.
/// </summary>
internal static class SpreadCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    /// <summary>A 60 fps frame — the budget one iteration has to fit inside.</summary>
    private const double FrameDt = 1.0 / 60.0;

    /// <summary>Mirrors Ksa6DofGuidance.ColdMaxDefectM: the plan becomes flyable here.</summary>
    private const double ColdGateM = 15.0;

    internal static int Run()
    {
        (string name, double alt, double down, double vz)[] cases =
        [
            ("high and fast", 1971.0, 235.0, -167.0),
            ("mid approach",   800.0, 300.0,  -60.0),
            ("close in",       250.0, 100.0,  -25.0),
        ];

        Console.WriteLine("SPREADING THE COLD SOLVE ACROSS FRAMES");
        Console.WriteLine($"  one SCvx iteration per {FrameDt * 1000:F0} ms frame, re-anchored at the");
        Console.WriteLine("  falling vehicle each time; free fall throughout, which is the worst case");
        Console.WriteLine($"  flyable = defect within the cold gate ({ColdGateM:F0} m)");
        Console.WriteLine();

        bool ok = true;
        foreach ((string name, double alt, double down, double vz) in cases)
        {
            Console.WriteLine($"  --- {name}: {alt:F0} m, {down:F0} m downrange, {-vz:F0} m/s ---");

            (double blockMs, int blockIters, double blockDefect) = Blocking(alt, down, vz);
            Console.WriteLine($"    blocking : {blockIters,2} iters in ONE frame, {blockMs,6:F0} ms, "
                              + $"defect {blockDefect:F2} m");

            (int frames, double worstFrameMs, double medFrameMs, double defect,
             double fell, bool converged) = Spread(alt, down, vz);

            Console.WriteLine($"    spread   : {frames,2} frames, worst frame {worstFrameMs,5:F0} ms, "
                              + $"median {medFrameMs,5:F0} ms, defect {defect:F2} m");
            Console.WriteLine($"               vehicle fell {fell,6:F0} m while converging "
                              + $"({frames * FrameDt:F2} s)");

            bool smooth = worstFrameMs <= 3.0 * blockMs / Math.Max(blockIters, 1) + 20.0;
            if (!converged)
            {
                Console.WriteLine("               FAIL - never reached a flyable plan");
                ok = false;
            }
            else
            {
                Console.WriteLine($"               worst frame is {blockMs / Math.Max(worstFrameMs, 1e-6):F0}x "
                                  + $"smaller than the blocking solve");
                if (!smooth)
                {
                    Console.WriteLine("               FAIL - a single frame still costs too much");
                    ok = false;
                }
            }
            Console.WriteLine();
        }

        Console.WriteLine(ok
            ? "PASS - the cold solve spreads, and the vehicle falling does not prevent convergence"
            : "FAIL - see above");
        return ok ? 0 : 1;
    }

    /// <summary>The control: everything in one call, as the mod does today.</summary>
    private static (double ms, int iters, double defect) Blocking(double alt, double down, double vz)
    {
        (Scvx6DofSolver s, double[] x0, double[] xf, double L) = Build(alt, down, vz);
        var sw = System.Diagnostics.Stopwatch.StartNew();
        s.Solve(25);
        return (sw.Elapsed.TotalMilliseconds, s.IterationCount, Defect(s, L));
    }

    /// <summary>
    /// One iteration per frame, re-anchoring at the falling vehicle each time — i.e.
    /// the warm loop, started before there is anything worth flying.
    /// </summary>
    private static (int frames, double worstMs, double medMs, double defect, double fell, bool converged)
        Spread(double alt, double down, double vz)
    {
        (Scvx6DofSolver s, double[] x0, double[] xf, double L) = Build(alt, down, vz);
        var state = (double[])x0.Clone();
        double alt0 = state[2];
        var times = new List<double>();
        var sw = new System.Diagnostics.Stopwatch();

        for (int frame = 0; frame < 120; frame++)
        {
            sw.Restart();
            s.Iterate();
            times.Add(sw.Elapsed.TotalMilliseconds);

            double d = Defect(s, L);
            if (double.IsFinite(d) && d <= ColdGateM && frame >= 2)
            {
                times.Sort();
                return (frame + 1, times[^1], times[times.Count / 2], d, alt0 - state[2], true);
            }

            // The vehicle keeps falling while we iterate. Free fall: nothing is
            // commanding thrust before engagement.
            FreeFall(state, FrameDt);

            // Re-anchor: node 0 is wherever the vehicle now is, seeded from the plan
            // we have so far. Exactly what Update does every cadence tick.
            double[] xs = (double[])s.ReferenceX.Clone();
            double[] us = (double[])s.ReferenceU.Clone();
            Array.Copy(state, 0, xs, 0, NX);
            s.Reseed(state, xs, us, s.Sigma, trustRegion: s.TrustRegion);
        }

        times.Sort();
        return (120, times[^1], times[times.Count / 2], Defect(s, L), alt0 - state[2], false);
    }

    private static void FreeFall(double[] x, double dt)
    {
        for (int i = 0; i < 3; i++) x[i] += x[3 + i] * dt;
        x[5] -= 9.81 * dt;
    }

    private static (Scvx6DofSolver s, double[] x0, double[] xf, double L)
        Build(double alt, double down, double vz)
    {
        var x0 = new double[NX];
        x0[0] = down; x0[2] = alt; x0[5] = vz; x0[6] = 1.0; x0[13] = 129495.0;
        var xf = new double[NX];
        xf[2] = 10.0; xf[6] = 1.0;

        const int n = 20;
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

        var s = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        s.Initialize(x0, xf, xSeed, uSeed, seed);
        return (s, x0, xf, L);
    }

    private static double Defect(Scvx6DofSolver s, double lengthScale)
    {
        for (int i = s.Trace.Count - 1; i >= 0; i--)
            if (s.Trace[i].Accepted)
                return s.Trace[i].DefectNorm * lengthScale;
        return double.PositiveInfinity;
    }
}

using Scvx;

/// <summary>
/// Does softening the terminal position turn "no plan" into "imperfect plan"?
///
/// A hard terminal constraint demands arrival exactly at the target, exactly at rest.
/// When that is not achievable the problem is INFEASIBLE and the solver returns
/// nothing — but a booster on the way down cannot decline to land, so "no plan" is
/// not a safe answer. It just means continuing to fly an older plan that is getting
/// worse every cycle.
///
/// Thrust only buys deceleration ABOVE hover, so stopping from speed v needs
/// v^2 / (2*(TWR-1)*g) of altitude. Below that the target is unreachable by any
/// guidance law, which makes this case routine rather than exotic. The question is
/// only whether the solver says so usefully.
/// </summary>
internal static class SoftTerminalCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    internal static int Run()
    {
        Console.WriteLine("SOFT TERMINAL POSITION");
        Console.WriteLine("  hard: arrive exactly, or not at all");
        Console.WriteLine("  soft: a penalised miss, so an unreachable target still yields a plan");
        Console.WriteLine();
        Console.WriteLine($"  {"case",-26} {"terminal",-6} {"status",-16} {"defect",8} {"miss",8} {"touchdown",9}");

        // TWR 1.25 from 300 m at 50 m/s needs 510 m to stop: unreachable. TWR 2.2 from
        // the same state is comfortable, and is the control - softening must not
        // change an answer that was already achievable.
        (string name, double twr, double alt, double vz, double down)[] cases =
        [
            ("reachable",              2.20, 300.0, -50.0,  80.0),
            ("cannot stop in time",    1.25, 300.0, -50.0,  80.0),
            ("cannot stop, far out",   1.25, 300.0, -50.0, 300.0),
        ];

        bool ok = true;
        foreach ((string name, double twr, double alt, double vz, double down) in cases)
        {
            var hard = Solve(twr, alt, vz, down, 0.0);
            var soft = Solve(twr, alt, vz, down, 1e3);
            foreach ((string label, var r) in new[] { ("hard", hard), ("soft", soft) })
                Console.WriteLine($"  {name,-26} {label,-6} {r.status,-16} {r.defect,7:F2}m {r.miss,7:F1}m "
                                  + $"{r.touchdown,6:F1} m/s");

            bool reachable = name == "reachable";
            if (reachable)
            {
                // Softening must be INERT when the target is achievable: an exact
                // penalty means the slack stays at zero and the answer is unchanged.
                bool unchanged = soft.usable && soft.miss < 1.0;
                Console.WriteLine($"      -> softening inert when reachable: {(unchanged ? "yes" : "NO")}"
                                  + $" (miss {soft.miss:F2} m)");
                ok &= unchanged;
            }
            else
            {
                bool rescued = soft.usable && soft.defect <= 15.0;
                Console.WriteLine($"      -> hard {(hard.usable ? "solved" : "gave no usable plan")}, "
                                  + $"soft {(rescued ? $"lands {soft.miss:F0} m off" : "also failed")}");
                ok &= rescued;
            }
            Console.WriteLine();
        }

        Console.WriteLine(ok
            ? "PASS - softening is inert when reachable, and available when not"
            : "FAIL - see above");
        Console.WriteLine();
        Console.WriteLine("  Note on what this does NOT show. The cases above are not strictly");
        Console.WriteLine("  infeasible: the DYNAMICS are already soft (virtual control), so the");
        Console.WriteLine("  solver can always 'reach' a target by paying defect instead. It does,");
        Console.WriteLine("  which is why the hard and soft answers here barely differ. Terminal");
        Console.WriteLine("  softening is the right structure for the no-plan case and is provably");
        Console.WriteLine("  inert otherwise, but a case that genuinely exercises it is still");
        Console.WriteLine("  outstanding.");
        return ok ? 0 : 1;
    }

    private static double _lastTouchdownSpeed;

    private static (string status, double defect, double miss, bool usable, double touchdown)
        Solve(double twr, double alt, double vz, double down, double missWeight)
    {
        var x0 = new double[NX];
        x0[0] = down; x0[2] = alt; x0[5] = vz; x0[6] = 1.0; x0[13] = 120000.0;
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
            Tmax = twr * m0 * 9.81,
            ThrottleFloor = 0.10,
            TiltMaxDeg = 60.0,
            WDu = 0.05,
            WW = 0.002,
            ProximalWeight = 0.05,
            TerminalMissWeight = missWeight,
            TerminalSpeedWeight = missWeight > 0.0 ? 1e4 : 0.0,
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
        ScvxStatus st = s.Solve(40);

        double defect = double.PositiveInfinity;
        for (int i = s.Trace.Count - 1; i >= 0; i--)
            if (s.Trace[i].Accepted) { defect = s.Trace[i].DefectNorm * L; break; }

        double[] x = s.ReferenceX;
        double miss = Math.Sqrt(
            Math.Pow(x[(n - 1) * NX + 0] - xf[0], 2) +
            Math.Pow(x[(n - 1) * NX + 1] - xf[1], 2) +
            Math.Pow(x[(n - 1) * NX + 2] - xf[2], 2));
        double vend = Math.Sqrt(
            Math.Pow(x[(n - 1) * NX + 3], 2) + Math.Pow(x[(n - 1) * NX + 4], 2) +
            Math.Pow(x[(n - 1) * NX + 5], 2));
        _lastTouchdownSpeed = vend;
        bool usable = st is ScvxStatus.Converged or ScvxStatus.IterationLimit
                      && double.IsFinite(defect);
        return (st.ToString(), defect, miss, usable, vend);
    }
}

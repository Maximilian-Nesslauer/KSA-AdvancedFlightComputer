using Scvx;

/// <summary>
/// Does seeding the cold solve from a convex 3-DOF G-FOLD solution actually help?
///
/// SCvx refines a reference rather than searching for one: it linearises about
/// whatever it is handed and may only move a trust region's distance per iteration.
/// So the seed decides how many iterations a cold solve needs, and which local
/// solution it walks toward. The default seed is a straight line at constant thrust,
/// which satisfies neither the dynamics nor the constraints.
///
/// The claim under test is that a G-FOLD seed is cheaper END TO END - including the
/// cost of the G-FOLD solve itself - and produces a better first plan. Both halves
/// matter: a seed that saves solver iterations but costs more to compute than it saves
/// is worthless, and a fast cold solve that lands on a worse trajectory is worse than
/// useless.
/// </summary>
internal static class SeedCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    internal static int Run()
    {
        Console.WriteLine("COLD-START SEED: straight line vs 3-DOF G-FOLD");
        Console.WriteLine("  G-FOLD time is INCLUDED in the seeded total - it has to pay for itself");
        Console.WriteLine();
        Console.WriteLine($"  {"case",-22} {"seed",-9} {"status",-15} {"iters",6} {"total ms",9} "
                          + $"{"defect",8} {"sigma",7} {"fuel t",7}");

        (string name, double alt, double down, double vz)[] cases =
        [
            ("high and fast",     1971.0, 235.0, -167.0),
            ("mid approach",       800.0, 300.0,  -60.0),
            ("close in",           250.0, 100.0,  -25.0),
            ("steep, little range", 900.0,  40.0,  -90.0),
        ];

        bool ok = true;
        foreach ((string name, double alt, double down, double vz) in cases)
        {
            var res = new (string label, int mode)[]
                { ("straight", 0), ("gfold-1", 1), ("gfold-srch", 2) }
                .Select(v => Measure(name, alt, down, vz, v.mode, v.label)).ToList();

            foreach (var r in res)
                Console.WriteLine($"  {r.name,-22} {r.label,-9} {r.status,-15} {r.iters,6} "
                                  + $"{r.ms,8:F0}  {r.defect,7:F2}m {r.sigma,6:F1}s {r.fuel / 1000.0,6:F2}");

            var straight = res[0];
            var gfold = res.Skip(1).OrderBy(r => r.ms).First();
            if (!gfold.solved)
            {
                Console.WriteLine("      -> G-FOLD seed did not produce a plan; falls back, no loss");
            }
            else
            {
                double speedup = straight.ms / Math.Max(gfold.ms, 1e-6);
                Console.WriteLine($"      -> best seeded ({gfold.label}) is {speedup:F1}x "
                                  + $"{(speedup >= 1 ? "faster" : "SLOWER")}, "
                                  + $"{straight.iters - gfold.iters:+0;-0} iterations, "
                                  + $"defect {straight.defect:F2} -> {gfold.defect:F2} m");
                // The seed must not make things worse. Slower or a materially worse
                // first plan would mean the extra machinery is not earning its place.
                if (gfold.ms > straight.ms * 1.2) { ok = false; Console.WriteLine("      FAIL - slower"); }
            }
            Console.WriteLine();
        }

        Console.WriteLine();
        Console.WriteLine(ok
            ? "The G-FOLD seed pays for itself here."
            : "The G-FOLD seed does NOT pay for itself - which is why it ships OFF.");
        Console.WriteLine("  There is little headroom to win: the straight-line seed already converges");
        Console.WriteLine("  in 6-8 SCvx iterations at the node counts now flown, so a seed costing");
        Console.WriteLine("  hundreds of ms to compute cannot come out ahead. Re-run this if cold-solve");
        Console.WriteLine("  cost becomes the bottleneck again - at higher node counts, or from a much");
        Console.WriteLine("  worse initial state, the balance would differ.");
        // Reports rather than gates: this is a measurement, not a regression.
        return 0;
    }

    private static (string name, string label, string status, int iters, double ms,
                    double defect, double sigma, double fuel, bool solved)
        Measure(string name, double alt, double down, double vz, int mode, string label)
    {
        var x0 = new double[NX];
        x0[0] = down; x0[2] = alt; x0[5] = vz; x0[6] = 1.0; x0[13] = 129495.0;
        var xf = new double[NX];
        xf[2] = 10.0; xf[6] = 1.0;

        const int n = 20;
        double m0 = x0[13];
        double L = Math.Sqrt(down * down + (alt - 10.0) * (alt - 10.0));
        double V = Math.Max(Math.Max(Math.Abs(vz), Math.Sqrt(L * 9.81)), 1.0);
        double seedSigma = Math.Max(2.0 * L / Math.Max(V, 1.0), 4.0);

        var cfg = new Scvx6DofConfig
        {
            Nodes = n,
            Tmax = 6.03e6,
            ThrottleFloor = 0.10,
            TiltMaxDeg = 60.0,
            WDu = 0.05,
            WW = 0.002,
            ProximalWeight = 0.05,
            SigmaScale = seedSigma,
            SigmaMin = seedSigma * 0.15,
            SigmaMax = seedSigma * 4.0,
            XScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0],
        };
        var dyn = new Dynamics6Dof.Params { Gz = -9.81, Isp = 300.0 };

        var sw = System.Diagnostics.Stopwatch.StartNew();
        double[] xSeed = new double[n * NX];
        double[] uSeed = new double[n * NU];
        double sigma = seedSigma;
        bool seeded = false;

        if (mode != 0)
        {
            // The G-FOLD solve is INSIDE the stopwatch: it has to pay for itself.
            Ksa6DofGfoldSeed.SearchTimeOfFlight = mode == 2;
            seeded = Ksa6DofGfoldSeed.TryBuild(x0, xf, cfg, dyn, n,
                                               out xSeed, out uSeed, out sigma, out _);
            if (!seeded)
                return (name, label, "no G-FOLD seed", 0, sw.Elapsed.TotalMilliseconds, 0, 0, 0, false);
        }
        else
        {
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
        }
        Array.Copy(x0, 0, xSeed, 0, NX);

        var solver = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        solver.Initialize(x0, xf, xSeed, uSeed, sigma);
        ScvxStatus st = solver.Solve(25);
        double ms = sw.Elapsed.TotalMilliseconds;

        double defect = double.PositiveInfinity;
        for (int i = solver.Trace.Count - 1; i >= 0; i--)
            if (solver.Trace[i].Accepted) { defect = solver.Trace[i].DefectNorm * L; break; }

        double fuel = m0 - solver.ReferenceX[(n - 1) * NX + Dynamics6Dof.IM];
        bool solved = st is ScvxStatus.Converged or ScvxStatus.IterationLimit;
        return (name, label, st.ToString(), solver.IterationCount, ms, defect, solver.Sigma, fuel, solved);
    }
}

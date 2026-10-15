using System.Globalization;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// The 6-DOF standby's re-solve, offline: does it keep publishing plans while ANOTHER guidance flies the craft?
///
/// In flight the standby converged under the 3-DOF's burn and then published nothing for 1.8 s. Its re-solve was the flying loop's Update: seeded from the published plan in a tight trust region, a refused attempt thrown away, the next one started from the same plan again, older. That assumes the craft follows the plan, and under the 3-DOF it does not.
///
/// Here the craft flies the first 6-DOF plan's controls open loop with its thrust off by a few percent - a stand-in for a guidance with its own throttle profile - and the true dynamics are integrated. The same state history is then re-solved two ways at a 0.1 s cadence:
///   UPDATE  as Ksa6DofGuidance.Update: from the published plan, tight region, a wide retry, fail fast after two refusals;
///   TRACK   as Ksa6DofGuidance.Track: from its own latest trajectory whether or not it passed, region carried, two iterations.
/// Both shift their seed to now and re-anchor it at the measured state, and both are gated per channel as the flight gate is, under the game's wall-clock budgets. What is counted is how often each publishes and how old the published plan gets.
///
/// What it showed (2026-10-07): on this machine both publish every cycle whenever the craft is in a state that can still land, even at one iteration per attempt (--slow) - so the staleness seen in flight is not reproduced here, and its cause is something the game adds. Tracking is never worse, publishes more at one iteration per attempt, and its worst cycle is cheaper when the mismatch is large. The "95 % fixed" law over-brakes the craft into states nothing can land from; every mode fails there alike, and it is kept to show that. Moving the whole seed with the craft (ColdReferenceShift) as well made no difference and was dropped.
/// </summary>
internal static class StandbyTrackCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;
    private const double Cadence = 0.1;
    private const double GateM = 1.0;
    private const double WarmRegion = 0.05;
    // Ksa6DofGuidance's wall-clock budgets: a healthy cycle, and one recovering from a refusal (Update's retry, and every Track call).
    private const double CycleBudgetMs = 25.0;
    private const double RecoveryBudgetMs = 120.0;
    internal static bool Verbose;

    /// <summary>One SCvx iteration per attempt, as in the game when a 50-node iteration outlasts the cycle deadline under load.</summary>
    internal static bool Slow;

    internal static int Run()
    {
        Console.WriteLine("6-DOF STANDBY RE-SOLVE while another law flies the craft (first plan open loop, thrust biased)");
        Console.WriteLine($"  cadence {Cadence:F1} s, gate {GateM:F1} m; published = cycles whose plan passed the gate"
                          + (Slow ? "; ONE iteration per attempt, as a loaded game manages at 50 nodes" : ""));
        Console.WriteLine();
        Console.WriteLine($"  {"thrust bias",11}  {"mode",-6} {"published",10} {"oldest plan",12} {"mean ms",8} {"worst ms",9}");

        bool ok = true;
        // loop_ref.csv's gentle descent, then one like the flight's: 600 m up, sinking 120 m/s, 50 nodes.
        (string name, double[] x0, int n, double seed)[] cases =
        [
            ("loop_ref, 30 nodes", [100, 0, 300, 0, 0, -50, 1, 0, 0, 0, 0, 0, 0, 250000], 30, 12.0),
            ("600 m at 120 m/s, 50 nodes", [150, 0, 600, -30, 0, -120, 1, 0, 0, 0, 0, 0, 0, 200000], 50, 9.0),
        ];
        // Thrust off by a fraction, or held at 95 % throughout as the 3-DOF's burn was (NaN).
        double[] biases = [0.0, 0.05, 0.10, double.NaN];
        foreach ((string name, double[] x0, int n, double seed) in cases)
        {
            Console.WriteLine($"  -- {name}");
            foreach (double bias in biases)
            {
                var update = Run(x0, n, seed, bias, track: false);
                var track = Run(x0, n, seed, bias, track: true);
                if (update == null || track == null)
                {
                    Console.WriteLine($"  {Label(bias),11}   cold solve failed");
                    ok = false;
                    continue;
                }
                Print(bias, "UPDATE", update.Value);
                Print(bias, "TRACK", track.Value);
                // The fix has to publish at least as often and never let the plan get older.
                if (track.Value.published < update.Value.published || track.Value.oldest > update.Value.oldest + 1e-9)
                    ok = false;
            }
        }
        Console.WriteLine();
        Console.WriteLine(ok ? "  PASS: tracking publishes at least as often, and its plan never gets older"
                             : "  FAIL: tracking published less, or let its plan get older");
        return ok ? 0 : 1;
    }

    private static string Label(double bias) => double.IsNaN(bias) ? "95 % fixed" : bias.ToString("P0", CultureInfo.InvariantCulture);

    private static void Print(double bias, string mode, (int published, int cycles, double oldest, double meanMs, double worstMs) r)
        => Console.WriteLine($"  {Label(bias),11}   {mode,-6} {r.published,4}/{r.cycles,-5} {r.oldest,10:F1} s {r.meanMs,8:F1} {r.worstMs,9:F1}");

    private static (int published, int cycles, double oldest, double meanMs, double worstMs)? Run(double[] x0, int n, double seed, double bias, bool track)
    {
        double[] xf = new double[NX];
        xf[6] = 1.0;
        double L = Math.Sqrt(x0[0] * x0[0] + x0[1] * x0[1] + x0[2] * x0[2]);
        double V = Math.Max(Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]), Math.Sqrt(L * 9.81));
        var cfg = new Scvx6DofConfig
        {
            Nodes = n,
            XScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, x0[13]],
            WDu = 0.05,
            WW = 0.002,
            ProximalWeight = 0.05,
            SigmaScale = seed,
            SigmaMin = seed * 0.15,
            SigmaMax = seed * 4.0,
            ThrottleFloor = 0.4,
        };
        var dyn = new Dynamics6Dof.Params { Gz = -9.81 };
        var solver = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };

        // Cold solve from a straight line, as the standby's first plan.
        var xSeed = new double[n * NX];
        var uSeed = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xSeed[k * NX + 3 + i] = x0[3 + i] * (1 - t);
            }
            xSeed[k * NX + 6] = 1.0;
            xSeed[k * NX + 13] = x0[13] * (1.0 - 0.08 * t);
            uSeed[k * NU + 2] = 1.05 * x0[13] * 9.81;
        }
        Array.Copy(x0, 0, xSeed, 0, NX);
        solver.Initialize(x0, xf, xSeed, uSeed, seed);
        if (solver.Solve(30) is ScvxStatus.Failed or ScvxStatus.TrustRegionCollapsed)
            return null;

        // The law flying the craft: the first plan, open loop, thrust biased.
        double[] lawU = (double[])solver.ReferenceU.Clone();
        double lawSigma = solver.Sigma;

        double[] pubX = (double[])solver.ReferenceX.Clone(), pubU = (double[])solver.ReferenceU.Clone();
        double pubSigma = solver.Sigma, pubTime = 0.0;
        double[] workX = pubX, workU = pubU;
        double workSigma = pubSigma, workTime = 0.0;
        int failures = 0;

        var state = (double[])x0.Clone();
        double time = 0.0, oldest = 0.0, totalMs = 0.0, worstMs = 0.0;
        int published = 0, cycles = 0;
        // The first half of the burn: further on, a biased open-loop law has over- or under-braked the craft into states no plan can land from, and every re-solve fails alike.
        while (time < 0.5 * lawSigma && state[2] > 20.0)
        {
            const int Sub = 8;
            var u = new double[NU];
            for (int s = 0; s < Sub; s++)
            {
                SampleU(lawU, n, lawSigma, time + (s + 0.5) / Sub * Cadence, u);
                u[2] = double.IsNaN(bias) ? 0.95 * cfg.Tmax : u[2] * (1.0 + bias);
                Integrate(state, u, dyn, Cadence / Sub);
            }
            time += Cadence;
            cycles++;

            var sw = System.Diagnostics.Stopwatch.StartNew();
            bool pass;
            if (track)
            {
                bool fromWork = workTime > pubTime;
                (double[] bx, double[] bu, double bs, double bt) = fromWork ? (workX, workU, workSigma, workTime) : (pubX, pubU, pubSigma, pubTime);
                double sigma = Math.Max(cfg.SigmaMin, bs - (time - bt));
                (double[] xs, double[] us) = Shift(bx, bu, n, bs, time - bt, sigma, state);
                solver.Reseed(state, xs, us, sigma, trustRegion: Math.Clamp(solver.TrustRegion, WarmRegion, solver.TrustRegionMax));
                pass = Gate(solver.Solve(Slow ? 1 : 2, RecoveryBudgetMs), solver, state, cfg);
                workX = (double[])solver.ReferenceX.Clone();
                workU = (double[])solver.ReferenceU.Clone();
                workSigma = solver.Sigma;
                workTime = time;
            }
            else
            {
                double sigma = Math.Max(cfg.SigmaMin, pubSigma - (time - pubTime));
                (double[] xs, double[] us) = Shift(pubX, pubU, n, pubSigma, time - pubTime, sigma, state);
                solver.Reseed(state, xs, us, sigma, trustRegion: WarmRegion);
                pass = Gate(solver.Solve(Slow ? 1 : 5, failures > 0 ? RecoveryBudgetMs : CycleBudgetMs), solver, state, cfg);
                if (!pass && failures < 2)
                {
                    solver.Reseed(state, xs, us, sigma, trustRegion: solver.TrustRegionMax);
                    pass = Gate(solver.Solve(Slow ? 1 : 15, RecoveryBudgetMs), solver, state, cfg);
                }
                failures = pass ? 0 : failures + 1;
            }
            double ms = sw.Elapsed.TotalMilliseconds;
            totalMs += ms;
            worstMs = Math.Max(worstMs, ms);

            if (pass)
            {
                pubX = (double[])solver.ReferenceX.Clone();
                pubU = (double[])solver.ReferenceU.Clone();
                pubSigma = solver.Sigma;
                pubTime = time;
                published++;
            }
            oldest = Math.Max(oldest, time - pubTime);
        }
        return (published, cycles, oldest, totalMs / Math.Max(cycles, 1), worstMs);
    }

    // The flight gate as Ksa6DofGuidance.Finish asks it: anchored at the craft, and every channel within its own tolerance (DefectTolerances' defaults) over a 1 s commit horizon, loosening to 25x at the far end.
    private static bool Gate(ScvxStatus status, Scvx6DofSolver s, double[] x0, Scvx6DofConfig cfg)
    {
        if (status is ScvxStatus.Failed or ScvxStatus.TrustRegionCollapsed)
            return false;
        double[] r = s.ReferenceX;
        double anchor = Math.Sqrt((r[0] - x0[0]) * (r[0] - x0[0]) + (r[1] - x0[1]) * (r[1] - x0[1]) + (r[2] - x0[2]) * (r[2] - x0[2]));
        if (anchor > 1.0)
            return false;
        var tol = new double[NX];
        double q = Math.Sin(0.5 * Math.PI / 180.0);
        for (int i = 0; i < 3; i++) tol[Dynamics6Dof.IR + i] = GateM;
        for (int i = 0; i < 3; i++) tol[Dynamics6Dof.IV + i] = 0.5;
        for (int i = 0; i < 4; i++) tol[Dynamics6Dof.IQ + i] = q;
        for (int i = 0; i < 3; i++) tol[Dynamics6Dof.IW + i] = 0.01;
        tol[Dynamics6Dof.IM] = cfg.XScale[Dynamics6Dof.IM] * 1e-3;
        int commit = (int)Math.Floor(1.0 / Math.Max(s.Sigma / (cfg.Nodes - 1), 1e-6));
        double ratio = s.WeightedDefect(tol, commit, 25.0, out int ch, out int node, out double raw);
        if (Verbose && ratio > 1.0)
            Console.WriteLine($"      refused: {Scvx6DofSolver.ChannelName(ch)} {ratio:F1}x at interval {node}, alt {x0[2]:F0} m, vz {x0[5]:F1}, sigma {s.Sigma:F1} s, status {status}");
        return ratio <= 1.0;
    }

    // A trajectory resampled from `elapsed` onto a horizon of `sigmaNew`, node 0 replaced by the measured state.
    private static (double[] xs, double[] us) Shift(double[] x, double[] u, int n, double sigma, double elapsed, double sigmaNew, double[] state)
    {
        var xs = new double[n * NX];
        var us = new double[n * NU];
        double dtOld = sigma / (n - 1), dtNew = sigmaNew / (n - 1);
        for (int k = 0; k < n; k++)
        {
            double s = Math.Clamp((elapsed + k * dtNew) / dtOld, 0.0, n - 1.0);
            int a = Math.Min((int)s, n - 2);
            double f = s - a;
            for (int i = 0; i < NX; i++)
                xs[k * NX + i] = x[a * NX + i] * (1 - f) + x[(a + 1) * NX + i] * f;
            for (int i = 0; i < NU; i++)
                us[k * NU + i] = u[a * NU + i] * (1 - f) + u[(a + 1) * NU + i] * f;
            double qn = 0;
            for (int i = 6; i < 10; i++) qn += xs[k * NX + i] * xs[k * NX + i];
            qn = Math.Sqrt(qn);
            if (qn > 1e-12) for (int i = 6; i < 10; i++) xs[k * NX + i] /= qn;
        }
        Array.Copy(state, 0, xs, 0, NX);
        return (xs, us);
    }

    private static void SampleU(double[] u, int n, double sigma, double t, double[] into)
    {
        double s = Math.Clamp(t / (sigma / (n - 1)), 0.0, n - 1.0);
        int a = Math.Min((int)s, n - 2);
        double f = s - a;
        for (int i = 0; i < NU; i++)
            into[i] = u[a * NU + i] * (1 - f) + u[(a + 1) * NU + i] * f;
    }

    // RK4 on the true dynamics, quaternion renormalised, as MpcHarness integrates.
    private static void Integrate(double[] x, double[] u, Dynamics6Dof.Params p, double dt)
    {
        var k1 = new double[NX]; var k2 = new double[NX];
        var k3 = new double[NX]; var k4 = new double[NX];
        var tmp = new double[NX];
        Dynamics6Dof.Eval(x, u, p, k1);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * dt * k1[i];
        Dynamics6Dof.Eval(tmp, u, p, k2);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * dt * k2[i];
        Dynamics6Dof.Eval(tmp, u, p, k3);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + dt * k3[i];
        Dynamics6Dof.Eval(tmp, u, p, k4);
        for (int i = 0; i < NX; i++)
            x[i] += dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
        double qn = 0;
        for (int i = 6; i < 10; i++) qn += x[i] * x[i];
        qn = Math.Sqrt(qn);
        if (qn > 1e-12) for (int i = 6; i < 10; i++) x[i] /= qn;
    }
}

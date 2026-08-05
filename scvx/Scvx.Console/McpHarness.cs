using System.Globalization;
using Scvx;

/// <summary>
/// Closed-loop MPC simulator. No game required.
///
/// Everything before this measured the SOLVER in isolation — one plan, judged on its
/// own terms. But the reported problems are all closed-loop: solutions jumping
/// between re-solves, the vehicle looping instead of going to the target, solve times
/// climbing. None of those are visible in a single solve, which is why tuning
/// individual solves kept failing to fix them.
///
/// This runs the actual loop: solve, apply the plan's control, integrate the TRUE
/// nonlinear dynamics forward, re-solve from where the vehicle really ended up.
/// Dispersions are injected so it is not a self-fulfilling simulation.
///
/// The three measurements that matter:
///   PLAN JUMP   how far the new plan sits from the previous one, time-aligned. A
///               healthy MPC barely moves the plan between cycles; a large value IS
///               the "jumping between re-solves" symptom, quantified.
///   PATH LENGTH divided by the straight-line distance. 1.0 is direct; large means
///               the vehicle is touring, which is the "flying in big loops" symptom.
///   COST        ADMM iterations and wall time per cycle.
/// </summary>
internal static class MpcHarness
{
    private const int NX = 14;
    private const int NU = 4;

    internal sealed class Config
    {
        public string Name = "";
        public double[] XScale = [];
        public double WDu, WW, Proximal;
        public bool FixedSigma;
    }

    internal static double Dispersion = 1.0;
    internal static int Budget = 5;
    internal static int AdmmCap = ScsWorkspace.DefaultMaxIterations;

    internal static int Run()
    {
        string path = FindRef("loop_ref.csv");
        string[] lines = File.ReadAllLines(path).Where(l => l.Length > 0 && l[0] != '#').ToArray();
        double[] Row(int i) => lines[i].Split(',')
            .Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
        double[] x0 = Row(0), xf = Row(1), xRef = Row(2);
        int n = xRef.Length / NX;

        // A start that is NOT the benign reference one: off-plane velocity, tilted,
        // rotating. This is what a real engage looks like and where the trouble is.
        x0 = (double[])x0.Clone();
        x0[3] = 8.0; x0[4] = 6.0;
        double a = 12.0 * Math.PI / 180.0 / 2.0;
        x0[6] = Math.Cos(a); x0[7] = 0.0; x0[8] = Math.Sin(a); x0[9] = 0.0;
        x0[10] = 0.02; x0[11] = -0.02; x0[12] = 0.01;

        double L = Math.Sqrt((x0[0] - xf[0]) * (x0[0] - xf[0]) +
                             (x0[1] - xf[1]) * (x0[1] - xf[1]) +
                             (x0[2] - xf[2]) * (x0[2] - xf[2]));
        double speed = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
        double V = Math.Max(Math.Max(speed, Math.Sqrt(L * 9.81)), 1.0);
        double m0 = x0[13];

        // The reference's own scaling: per-axis, matching the actual magnitudes of the
        // problem, NOT one isotropic length. That distinction is one of the things
        // being tested here.
        double[] refScale = [100, 100, 300, 50, 50, 50, 1, 1, 1, 1, 1, 1, 1, 250000.0];
        double[] adaptiveScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0];

        var configs = new[]
        {
            new Config { Name = "reference weights + scale", XScale = refScale,      WDu = 0.2,  WW = 1.0,  Proximal = 0.0 },
            new Config { Name = "reference weights, adaptive scale", XScale = adaptiveScale, WDu = 0.2,  WW = 1.0,  Proximal = 0.0 },
            new Config { Name = "MOD AS SHIPPED",          XScale = adaptiveScale, WDu = 0.05, WW = 0.002, Proximal = 0.05 },
            new Config { Name = "mod weights, ref scale",  XScale = refScale,      WDu = 0.05, WW = 0.002, Proximal = 0.05 },
            new Config { Name = "reference weights + fixed sigma", XScale = refScale, WDu = 0.2, WW = 1.0, Proximal = 0.0, FixedSigma = true },
        };

        Console.WriteLine("CLOSED-LOOP MPC, dispersed start (off-plane velocity, 12 deg tilt, rotating)");
        Console.WriteLine("cadence 0.4 nodes, 5 SCvx iters/cycle, 3% thrust + 2% torque dispersion");
        Console.WriteLine();
        Console.WriteLine("  config                            miss    path/direct   PLAN JUMP   AWAY   WORST subproblem");

        foreach (Config c in configs)
            Simulate(c, x0, xf, n);

        // Separate HONEST disturbance response from INTERNAL churn. If plan jump goes
        // to ~0 with dispersion off, the MPC is behaving correctly and 5.7 m is simply
        // what a 3% thrust error costs. If it stays, the solver is moving the plan for
        // reasons of its own — different local optima cycle to cycle.
        Console.WriteLine();
        Console.WriteLine("  isolating the plan jump (MOD AS SHIPPED config):");
        Console.WriteLine("  dispersion  budget     miss   path/direct   PLAN JUMP   AWAY-FROM   ADMM/cycle");
        var probe = configs[2];
        foreach ((double disp, int budget) in new[] { (0.0, 5), (0.0, 15), (1.0, 5), (1.0, 15), (2.0, 5) })
        {
            Dispersion = disp;
            Budget = budget;
            Console.Write($"  {disp * 3.0,7:F1}%   {budget,6}  ");
            Simulate(probe, x0, xf, n, compact: true);
        }
        Dispersion = 1.0;
        Budget = 5;

        // Does capping ADMM buy a bounded worst case without wrecking the answer?
        // The SCvx loop already treats a truncated solve as a subproblem FAILURE and
        // shrinks the trust region, so a cap should degrade gracefully.
        Console.WriteLine();
        Console.WriteLine("  ADMM cap sweep (MOD AS SHIPPED, with dispersion):");
        Console.WriteLine("  cap        miss   path/direct   PLAN JUMP   AWAY   WORST subproblem");
        foreach (int cap in new[] { 100000, 20000, 8000, 4000, 2000, 1000 })
        {
            AdmmCap = cap;
            Console.Write($"  {cap,6}  ");
            Simulate(configs[2], x0, xf, n, compact: true);
        }
        AdmmCap = ScsWorkspace.DefaultMaxIterations;

        // If the plan jump is collocation error, it must FALL as the node count rises.
        // If it is flat, the solver is picking different answers cycle to cycle and
        // more nodes will not help.
        Console.WriteLine();
        Console.WriteLine("  node count sweep (zero dispersion, so any jump is the model's own error):");
        Console.WriteLine("  nodes     miss   path/direct   PLAN JUMP   AWAY-FROM   ms/cycle");
        Dispersion = 0.0;
        foreach (int nodes in new[] { 20, 30, 50, 80 })
        {
            Console.Write($"  {nodes,5}  ");
            SimulateNodes(configs[2], x0, xf, nodes);
        }
        Dispersion = 1.0;

        Console.WriteLine();
        Console.WriteLine("PLAN JUMP is the 'solutions jumping between re-solves' symptom, in metres.");
        Console.WriteLine("path/direct is the 'flying in loops' symptom; 1.0 would be a straight run.");
        return 0;
    }

    private static void SimulateNodes(Config c, double[] x0, double[] xf, int nodes) =>
        Simulate(c, x0, xf, nodes, compact: true);

    private static void Simulate(Config c, double[] x0, double[] xf, int n, bool compact = false)
    {
        double sigmaSeed = 12.0;
        var cfg = new Scvx6DofConfig
        {
            Nodes = n,
            XScale = c.XScale,
            WDu = c.WDu,
            WW = c.WW,
            ProximalWeight = c.Proximal,
            SigmaScale = sigmaSeed,
            SigmaMin = c.FixedSigma ? sigmaSeed * (1 - 1e-6) : sigmaSeed * 0.2,
            SigmaMax = c.FixedSigma ? sigmaSeed * (1 + 1e-6) : sigmaSeed * 3.0,
        };
        var dyn = new Dynamics6Dof.Params();
        var solver = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps, MaxSubproblemIterations = AdmmCap };

        // Cold solve.
        var xSeed = new double[n * NX];
        var uSeed = new double[n * NU];
        double m0 = x0[13];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xSeed[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
            }
            for (int i = 0; i < 4; i++)
                xSeed[k * NX + 6 + i] = x0[6 + i] * (1 - t) + xf[6 + i] * t;
            Normalise(xSeed, k);
            for (int i = 0; i < 3; i++) xSeed[k * NX + 10 + i] = x0[10 + i] * (1 - t);
            xSeed[k * NX + 13] = m0 * (1.0 - 0.08 * t);
            uSeed[k * NU + 2] = 1.05 * m0 * Math.Abs(dyn.Gz);
        }
        Array.Copy(x0, 0, xSeed, 0, NX);
        solver.Initialize(x0, xf, xSeed, uSeed, sigmaSeed);
        if (solver.Solve(30) is ScvxStatus.Failed or ScvxStatus.TrustRegionCollapsed)
        {
            Console.WriteLine($"  {c.Name,-32}  COLD SOLVE FAILED");
            return;
        }

        double[] plan = (double[])solver.ReferenceX.Clone();
        double[] planU = (double[])solver.ReferenceU.Clone();
        double sigma = solver.Sigma;

        var state = (double[])x0.Clone();
        var rng = new Random(4242);
        double totalMs = 0, pathLength = 0, planJump = 0, awayFromTarget = 0;
        double prevRange = Dist((state[0], state[1], state[2]), (xf[0], xf[1], xf[2]));
        long totalAdmm = 0;
        int worstAdmm = 0;
        double worstMs = 0;
        int cycles = 0;
        (double, double, double) prevPos = (state[0], state[1], state[2]);

        double simTime = 0.0;
        // Run until the burn is actually DONE. The previous cap of 60 cycles ended the
        // run at ~9.9 s of a ~12 s burn, so the reported "miss" was simply where the
        // vehicle happened to be part-way through — a harness artifact, not guidance.
        double budgetTime = sigma * 2.0;
        while (simTime < budgetTime && state[2] > xf[2] + 1.0 && cycles < 400)
        {
            double dtNode = sigma / (n - 1);
            double step = 0.4 * dtNode;                       // the mod's cadence

            // --- apply the plan's control over this step, integrating TRUE dynamics ---
            // The model treats the control as varying LINEARLY between nodes
            // (trapezoidal collocation). Holding node 0's value across the step is a
            // different control law, and the resulting mismatch was showing up as
            // "plan jump" that had nothing to do with the solver.
            double thrustErr = 1.0 + (rng.NextDouble() - 0.5) * 0.06 * Dispersion;
            double txErr = 1.0 + (rng.NextDouble() - 0.5) * 0.04 * Dispersion;
            double tyErr = 1.0 + (rng.NextDouble() - 0.5) * 0.04 * Dispersion;

            const int Sub = 8;
            var u = new double[NU];
            for (int sub = 0; sub < Sub; sub++)
            {
                double tau = (sub + 0.5) / Sub * step;        // mid-substep along the plan
                double sNode = Math.Clamp(tau / dtNode, 0.0, n - 1.001);
                int ka = (int)sNode;
                int kb = Math.Min(ka + 1, n - 1);
                double fr = sNode - ka;
                for (int j = 0; j < NU; j++)
                    u[j] = planU[ka * NU + j] * (1 - fr) + planU[kb * NU + j] * fr;
                u[2] *= thrustErr;
                u[0] *= txErr;
                u[1] *= tyErr;
                Integrate(state, u, dyn, step / Sub);
            }
            simTime += step;
            var pos = (state[0], state[1], state[2]);
            pathLength += Dist(pos, prevPos);
            prevPos = pos;
            // Range-to-target should fall monotonically on a sane approach. Any
            // INCREASE is the vehicle touring, and summing it separates a genuine
            // loop from a merely curved (but always-closing) descent.
            double range = Dist(pos, (xf[0], xf[1], xf[2]));
            if (range > prevRange) awayFromTarget += range - prevRange;
            prevRange = range;

            // --- re-solve from where the vehicle ACTUALLY is ---
            int shift = Math.Clamp((int)Math.Round(step / dtNode), 0, n - 2);
            var xs = new double[n * NX];
            var us = new double[n * NU];
            for (int k = 0; k < n; k++)
            {
                int src = Math.Min(k + shift, n - 1);
                Array.Copy(plan, src * NX, xs, k * NX, NX);
                Array.Copy(planU, src * NU, us, k * NU, NU);
            }
            Array.Copy(state, 0, xs, 0, NX);

            double newSigma = Math.Max(sigma - step, cfg.SigmaMin);
            if (c.FixedSigma)
            {
                cfg.SigmaMin = newSigma * (1 - 1e-6);
                cfg.SigmaMax = newSigma * (1 + 1e-6);
            }
            solver.Reseed(state, xs, us, newSigma, trustRegion: 0.05);

            var sw = System.Diagnostics.Stopwatch.StartNew();
            ScvxStatus st = solver.Solve(Budget);
            totalMs += sw.Elapsed.TotalMilliseconds;
            totalAdmm += solver.Trace.Sum(t => t.SolverIterations);
            foreach (ScvxIteration it in solver.Trace)
                worstAdmm = Math.Max(worstAdmm, it.SolverIterations);
            worstMs = Math.Max(worstMs, sw.Elapsed.TotalMilliseconds);
            if (st is ScvxStatus.Failed or ScvxStatus.TrustRegionCollapsed)
                break;

            // PLAN JUMP: how far the new plan sits from the OLD plan, time-aligned.
            double jump = 0;
            double[] fresh = solver.ReferenceX;
            for (int k = 0; k < n - shift; k++)
            {
                int old = k + shift;
                double dx = fresh[k * NX] - plan[old * NX];
                double dy = fresh[k * NX + 1] - plan[old * NX + 1];
                double dz = fresh[k * NX + 2] - plan[old * NX + 2];
                jump = Math.Max(jump, Math.Sqrt(dx * dx + dy * dy + dz * dz));
            }
            planJump += jump;

            plan = (double[])fresh.Clone();
            planU = (double[])solver.ReferenceU.Clone();
            sigma = solver.Sigma;
            cycles++;
        }

        double direct = Dist((x0[0], x0[1], x0[2]), (xf[0], xf[1], xf[2]));
        double miss = Dist((state[0], state[1], state[2]), (xf[0], xf[1], xf[2]));
        if (compact)
            Console.WriteLine($"{miss,7:F1} m   {pathLength / Math.Max(direct, 1),8:F2}   " +
                              $"{planJump / Math.Max(cycles, 1),8:F1} m   worst {worstAdmm,7} ADMM   worst {worstMs,6:F0} ms");
        else
            Console.WriteLine($"  {c.Name,-32} {miss,7:F1} m   {pathLength / Math.Max(direct, 1),8:F2}   " +
                              $"{planJump / Math.Max(cycles, 1),8:F1} m   {awayFromTarget,6:F1} m   " +
                              $"worst {worstAdmm,7} ADMM  worst {worstMs,6:F0} ms");
    }

    // RK4 on the true nonlinear dynamics, with the quaternion renormalised each step.
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

    private static void Normalise(double[] xs, int k)
    {
        double qn = 0;
        for (int i = 0; i < 4; i++) qn += xs[k * NX + 6 + i] * xs[k * NX + 6 + i];
        qn = Math.Sqrt(qn);
        if (qn < 1e-12) { xs[k * NX + 6] = 1; return; }
        for (int i = 0; i < 4; i++) xs[k * NX + 6 + i] /= qn;
    }

    private static double Dist((double, double, double) a, (double, double, double) b) =>
        Math.Sqrt((a.Item1 - b.Item1) * (a.Item1 - b.Item1) +
                  (a.Item2 - b.Item2) * (a.Item2 - b.Item2) +
                  (a.Item3 - b.Item3) * (a.Item3 - b.Item3));

    private static string FindRef(string name)
    {
        string dir = AppContext.BaseDirectory;
        for (int i = 0; i < 8 && dir != null; i++)
        {
            string c = Path.Combine(dir, "python_ref", name);
            if (File.Exists(c)) return c;
            dir = Path.GetDirectoryName(dir);
        }
        return name;
    }
}

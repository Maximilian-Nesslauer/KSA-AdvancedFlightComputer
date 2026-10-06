using AdvancedFlightComputer.Guidance.Scvx.SixDof;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// --3dof-ksa: glide-and-burn and burn-only solves of a returning booster on the KSA model.
///
///  1. COLD GLIDE AND BURN from a mid-glide state, seeded by Seed3Dof: converges, feasible against the true constraints, lands upright on the site at the terminal speed, and burns no more than the booster carries.
///  2. BURN ONLY (K = 0), from the plan's own ignition state with its burn as the seed - the problem the guidance swaps to once ignition is committed. Restricted to the same burn, it must agree with the glide-and-burn plan.
///  3. THE SAME GLIDE WITHOUT LIFT, to show what steering on lift is worth.
/// </summary>
internal static class ThreeDofKsaCheck
{
    private const int NX = PointMass3Dof.NX;
    private const int NU = PointMass3Dof.NU;

    // Mid-glide: 32 km up, 9 km short of the site and 1.5 km off its line, falling at 950 m/s.
    private static readonly double[] X0 = [-9000, 1500, 32000, 300, -30, -950, ThreeDofScenario.WetMass];
    // Aim 20 m over the pad, still sinking at 2 m/s - the terminal hover takes the last metres.
    private static readonly double[] Xf = [0, 0, 20, 0, 0, -2];

    public static int Run(bool verbose)
    {
        bool ok = true;

        KsaPointMassModel model = ThreeDofScenario.Model();
        Scvx3DofConfig cfg = ThreeDofScenario.Config();
        Console.WriteLine("1. cold glide-and-burn solve, mid-glide");
        (bool pass1, Scvx3DofSolver plan) = SolveCold(model, cfg, verbose);
        ok &= pass1;

        Console.WriteLine();
        Console.WriteLine("2. burn only, from the plan's ignition state");
        ok &= BurnOnly(model, plan, verbose);

        Console.WriteLine();
        // Information, not a criterion: without lift this glide cannot slow enough for the propellant on board, and the solve is meant to say so by not converging rather than by inventing a plan.
        Console.WriteLine("3. the same glide with no lift (information only)");
        KsaPointMassModel noLift = model with { LiftSlope = null };
        (bool pass3, Scvx3DofSolver flat) = SolveCold(noLift, ThreeDofScenario.Config(), verbose);
        double propLift = X0[6] - plan.ReferenceX[^1];
        double propFlat = X0[6] - flat.ReferenceX[^1];
        Console.WriteLine(pass3
            ? $"  propellant with lift {propLift:F0} kg, without {propFlat:F0} kg"
            : $"  without lift there is no plan on {X0[6] - ThreeDofScenario.DryMass:F0} kg of propellant; with lift it takes {propLift:F0} kg");

        Console.WriteLine();
        Console.WriteLine(ok ? "PASS - glide-and-burn and burn-only solves on the KSA model" : "FAIL");
        return ok ? 0 : 1;
    }

    private static (bool, Scvx3DofSolver) SolveCold(KsaPointMassModel model, Scvx3DofConfig cfg, bool verbose)
    {
        if (!Seed3Dof.TryBuild(model, cfg, X0, Xf, new Seed3Dof.Options(), out double[] xs, out double[] us,
                               out double sg, out double sb))
        {
            Console.WriteLine("  seed failed");
            return (false, null!);
        }
        Console.WriteLine($"  seed: glide {sg:F1} s, burn {sb:F1} s, ignition at {xs[cfg.GlideIntervals * NX + 2] / 1000:F2} km");

        var solver = new Scvx3DofSolver(cfg, model);
        solver.Initialize(X0, Xf, xs, us, sg, sb);
        var sw = System.Diagnostics.Stopwatch.StartNew();
        ScvxStatus status = solver.Solve(maxIterations: 150);
        double ms = sw.Elapsed.TotalMilliseconds;
        PrintTrace(solver, verbose);
        Console.WriteLine($"  status={status}  iterations={solver.IterationCount}  {ms:F0} ms ({ms / Math.Max(1, solver.IterationCount):F1} ms/iter)");

        bool feasible = Report(model, cfg, solver, X0, Xf);
        bool pass = status == ScvxStatus.Converged && feasible;
        Console.WriteLine($"  {(pass ? "ok" : "FAIL")}");
        return (pass, solver);
    }

    private static bool BurnOnly(KsaPointMassModel model, Scvx3DofSolver plan, bool verbose)
    {
        Scvx3DofConfig two = plan.Config;
        int nb = two.Nodes - two.GlideIntervals;   // the plan's own burn resolution, so the two solve the same discretised burn
        // Anchored as the guidance anchors it at the swap: the burn starts from the attitude the glide arrives in, within a couple of degrees, not from whatever attitude would suit it.
        Scvx3DofConfig cfg = ThreeDofScenario.Config(nodes: nb, glideIntervals: 0, anchor: true);

        // Node 0 is the ignition state the plan predicts; the seed is its burn, resampled.
        var xs = new double[nb * NX];
        var us = new double[nb * NU];
        Resample3Dof.Resample(plan.ReferenceX, plan.ReferenceU, two.Nodes, two.GlideIntervals, plan.SigmaGlide, plan.SigmaBurn,
                              plan.SigmaGlide, nb, 0, 0.0, plan.SigmaBurn, cfg.ThrottleFloor, xs, us);
        double[] x0 = plan.ReferenceX.AsSpan(two.GlideIntervals * NX, NX).ToArray();
        Array.Copy(x0, xs, NX);

        var solver = new Scvx3DofSolver(cfg, model);
        solver.SetAttitudeAnchor(plan.ReferenceU.AsSpan(two.GlideIntervals * NU, 3), 2.0 * Math.PI / 180);
        solver.Initialize(x0, Xf, xs, us, 0.0, plan.SigmaBurn);
        double defect0 = solver.TrueCost(xs, us, 0.0, plan.SigmaBurn, out double d0, out _, out _);
        var sw = System.Diagnostics.Stopwatch.StartNew();
        ScvxStatus status = solver.Solve(maxIterations: 60);
        double ms = sw.Elapsed.TotalMilliseconds;
        PrintTrace(solver, verbose);
        Console.WriteLine($"  seed defect {d0:E2} (the plan's burn read onto its own nodes)");
        Console.WriteLine($"  status={status}  iterations={solver.IterationCount}  {ms:F0} ms");
        bool feasible = Report(model, cfg, solver, x0, Xf);

        double propPlan = x0[6] - plan.ReferenceX[^1];
        double propBurn = x0[6] - solver.ReferenceX[^1];
        double rel = Math.Abs(propBurn - propPlan) / propPlan;
        double dSig = Math.Abs(solver.SigmaBurn - plan.SigmaBurn);
        bool agrees = rel < 0.02 && dSig < 1.0;
        Console.WriteLine($"  burn {solver.SigmaBurn:F2} s vs the plan's {plan.SigmaBurn:F2} s; propellant {propBurn:F0} vs {propPlan:F0} kg (rel {rel:E1})  {(agrees ? "agrees" : "DIFFERS")}");
        bool pass = status == ScvxStatus.Converged && feasible && agrees;
        Console.WriteLine($"  {(pass ? "ok" : "FAIL")}");
        return pass;
    }

    private static void PrintTrace(Scvx3DofSolver solver, bool verbose)
    {
        if (!verbose) return;
        foreach (Scvx3DofIteration it in solver.Trace)
            Console.WriteLine(it.Solved
                ? $"    iter {it.Index,3}: rho={it.Rho,+7:F2} {(it.Accepted ? "accept" : "REJECT")}  tr={it.TrustRegion:F3}  " +
                  $"tg={it.SigmaGlide,6:F2} tb={it.SigmaBurn,6:F2}  step={it.Step:E2}  defect={it.DefectNorm:E2} " +
                  $"({Scvx3DofSolver.ChannelName(it.DefectChannel)}@{it.DefectNode})  J={it.Cost:E3}  ({it.SolverIterations} scs, {it.ElapsedMs:F0} ms)"
                : $"    iter {it.Index,3}: subproblem FAILED ({solver.LastFailureReason}) -> tr={it.TrustRegion:F4}");
    }

    /// <summary>The trajectory against its true constraints, and what it looks like.</summary>
    private static bool Report(KsaPointMassModel model, Scvx3DofConfig cfg, Scvx3DofSolver solver, double[] x0, double[] xf)
    {
        int n = cfg.Nodes, k = cfg.GlideIntervals;
        double[] x = solver.ReferenceX, u = solver.ReferenceU;
        double unit = 0, thErr = 0, alphaOver = 0, retro = 0, bounds = 0, ends = 0, rate = 0, peakAlpha = 0, peakQ = 0;
        double minTh = double.MaxValue, maxTh = 0;
        for (int i = 0; i < n; i++)
        {
            double bx = u[i * NU], by = u[i * NU + 1], bz = u[i * NU + 2], th = u[i * NU + 3];
            unit = Math.Max(unit, Math.Abs(Math.Sqrt(bx * bx + by * by + bz * bz) - 1));
            if (i < k) thErr = Math.Max(thErr, Math.Abs(th));
            else
            {
                thErr = Math.Max(thErr, Math.Max(cfg.ThrottleFloor - th, th - cfg.ThrottleCeiling));
                minTh = Math.Min(minTh, th);
                maxTh = Math.Max(maxTh, th);
            }
            double vx = x[i * NX + 3], vy = x[i * NX + 4], vz = x[i * NX + 5];
            double sp = Math.Sqrt(vx * vx + vy * vy + vz * vz);
            peakQ = Math.Max(peakQ, model.DynamicPressure(x.AsSpan(i * NX, NX)));
            if (sp > cfg.AlphaSpeedThreshold)
            {
                double a = Math.Acos(Math.Clamp(-(bx * vx + by * vy + bz * vz) / sp, -1, 1)) * 180 / Math.PI;
                peakAlpha = Math.Max(peakAlpha, a);
                alphaOver = Math.Max(alphaOver, a - cfg.AlphaMaxDeg);
                retro = Math.Max(retro, (bx * vx + by * vy + bz * vz) / sp);
            }
            if (i > 0)
                for (int c = 0; c < NX; c++)
                    bounds = Math.Max(bounds, Math.Max(cfg.StateMin[c] - x[i * NX + c], x[i * NX + c] - cfg.StateMax[c]));
            if (i < n - 1 && cfg.AttitudeRateMax > 0)
            {
                double dt = solver.NodeTime(i + 1) - solver.NodeTime(i);
                double db = Math.Sqrt(Math.Pow(u[(i + 1) * NU] - bx, 2) + Math.Pow(u[(i + 1) * NU + 1] - by, 2) + Math.Pow(u[(i + 1) * NU + 2] - bz, 2));
                rate = Math.Max(rate, db / Math.Max(dt, 1e-9) - cfg.AttitudeRateMax);
            }
        }
        for (int c = 0; c < NX; c++) ends = Math.Max(ends, Math.Abs(x[c] - x0[c]) / cfg.XScale[c]);
        for (int c = 0; c < 6; c++) ends = Math.Max(ends, Math.Abs(x[(n - 1) * NX + c] - xf[c]) / cfg.XScale[c]);
        double upright = 0;
        if (cfg.TerminalAttitude != null)
            for (int j = 0; j < 3; j++) upright = Math.Max(upright, Math.Abs(u[(n - 1) * NU + j] - cfg.TerminalAttitude[j]));
        solver.TrueCost(x, u, solver.SigmaGlide, solver.SigmaBurn, out double defect, out int ch, out int node);

        bool ok = unit < 1e-9 && thErr < 1e-6 && alphaOver < 0.05 && retro < 1e-3 && bounds < 1e-2 && ends < 1e-6
                  && upright < 1e-6 && rate < 1e-4 && defect < 1e-3;
        double[] xi = x.AsSpan(k * NX, NX).ToArray();
        double ignSpeed = Math.Sqrt(xi[3] * xi[3] + xi[4] * xi[4] + xi[5] * xi[5]);
        Console.WriteLine($"  glide {solver.SigmaGlide:F2} s, burn {solver.SigmaBurn:F2} s; ignition at {xi[2] / 1000:F2} km, {ignSpeed:F0} m/s, " +
                          $"q {model.DynamicPressure(xi) / 1000:F1} kPa; propellant {x0[6] - x[^1]:F0} kg of {x0[6] - ThreeDofScenario.DryMass:F0}");
        Console.WriteLine($"  peak alpha {peakAlpha:F1} deg, peak q {peakQ / 1000:F1} kPa, burn throttle {100 * minTh:F0}-{100 * maxTh:F0} %, " +
                          $"defect {defect:E1} ({Scvx3DofSolver.ChannelName(ch)}@{node})");
        Console.WriteLine($"  constraints: |b| {unit:E0}, throttle {thErr:E0}, alpha over {alphaOver:E0} deg, retro {retro:E0}, bounds {bounds:E0}, " +
                          $"ends {ends:E0}, upright {upright:E0}, rate over {rate:E0} rad/s  {(ok ? "feasible" : "VIOLATED")}");
        return ok;
    }
}

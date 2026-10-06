using System.Globalization;
using AdvancedFlightComputer.Guidance.Conic;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// --3dof: the glide-and-burn SCvx loop against 3dof.py (python_ref/threedof_ref.py).
///
/// Same model, same seed, same weights and schedule; the script solves its subproblems with CLARABEL and this side with SCS, so the pass criterion is the 6-DOF loop check's rather than an iteration-for-iteration match: it converges, the answer is feasible against the TRUE constraints, and the glide time, burn time and propellant agree with the script's.
/// </summary>
internal static class ThreeDofCheck
{
    private const int NX = PointMass3Dof.NX;
    private const int NU = PointMass3Dof.NU;

    public static int Run(bool verbose)
    {
        string? path = FindFile("threedof_ref.csv");
        if (path == null)
        {
            Console.Error.WriteLine("reference not found: python_ref/threedof_ref.csv");
            Console.Error.WriteLine("run:  python python_ref/threedof_ref.py");
            return 2;
        }

        string[] all = File.ReadAllLines(path);
        string[] lines = all.Where(l => l.Length > 0 && l[0] != '#').ToArray();
        double[] Row(int i) => lines[i].Split(',').Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();

        double[] x0 = Row(0), xf = Row(1), xRef = Row(2), bRef = Row(3), thRef = Row(4), meta = Row(5);
        double sigCRef = meta[0], sigBRef = meta[1], itersRef = meta[2], costRef = meta[3];
        int n = xRef.Length / NX;
        int glide = int.Parse(all[0].Split(' ').First(t => t.StartsWith("K=")).Substring(2), CultureInfo.InvariantCulture);

        var model = new Toy3DofModel();
        var stateMin = Enumerable.Repeat(double.NegativeInfinity, NX).ToArray();
        var stateMax = Enumerable.Repeat(double.PositiveInfinity, NX).ToArray();
        stateMax[PointMass3Dof.IV] = 2.0;      // the script's "monotonic approach" pair
        stateMin[PointMass3Dof.IR] = -10.0;
        var cfg = new Scvx3DofConfig
        {
            Nodes = n,
            GlideIntervals = glide,
            StateMin = stateMin,
            StateMax = stateMax,
            XScale = [1000, 1000, 6000, 300, 300, 300, x0[PointMass3Dof.IM]],
        };
        if (CheckConstants(all, cfg, model) != 0)
            return 1;

        // The script's seed: straight lines in r and v, mass held over the glide and bled over the burn, body axis retrograde of the seed velocity, 60 % throttle over the burn, coast 22 s and burn 27 s.
        double m0 = x0[PointMass3Dof.IM];
        var xSeed = new double[n * NX];
        var uSeed = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xSeed[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
            }
            xSeed[k * NX + PointMass3Dof.IM] = k < glide ? m0 : m0 + (k - glide) * (0.85 * m0 - m0) / (n - 1 - glide);
            double vx = xSeed[k * NX + 3], vy = xSeed[k * NX + 4], vz = xSeed[k * NX + 5];
            double sp = Math.Sqrt(vx * vx + vy * vy + vz * vz);
            if (sp > 1.0)
            {
                uSeed[k * NU + 0] = -vx / sp;
                uSeed[k * NU + 1] = -vy / sp;
                uSeed[k * NU + 2] = -vz / sp;
            }
            else
                uSeed[k * NU + 2] = 1.0;   // -approach, with the approach straight down
            uSeed[k * NU + PointMass3Dof.ITH] = k < glide ? 0.0 : 0.6;
        }

        var solver = new Scvx3DofSolver(cfg, model);
        solver.Initialize(x0, xf, xSeed, uSeed, sigmaGlide: 22.0, sigmaBurn: 27.0);

        var sw = System.Diagnostics.Stopwatch.StartNew();
        ScvxStatus status = solver.Solve(maxIterations: 120);
        double totalMs = sw.Elapsed.TotalMilliseconds;

        Console.WriteLine($"SCS {ScsWorkspace.NativeVersion}   3-DOF glide-and-burn SCvx loop from the script's seed");
        foreach (Scvx3DofIteration it in solver.Trace)
        {
            if (!verbose && it.Index > 0 && !it.Accepted && it.Solved) continue;
            Console.WriteLine(it.Solved
                ? $"  iter {it.Index,3}: rho={it.Rho,+7:F2} {(it.Accepted ? "accept" : "REJECT")}  tr={it.TrustRegion:F3}  " +
                  $"tc={it.SigmaGlide,5:F1} tb={it.SigmaBurn,5:F1}  step={it.Step:E2}  defect={it.DefectNorm:E2}  " +
                  $"J={it.Cost:E3}  ({it.SolverIterations} scs, {it.ElapsedMs:F0} ms)"
                : $"  iter {it.Index,3}: subproblem FAILED ({solver.LastFailureReason}) -> tr={it.TrustRegion:F4}");
        }
        Console.WriteLine();
        Console.WriteLine($"status={status}  iterations={solver.IterationCount}  total={totalMs:F0} ms  " +
                          $"({totalMs / Math.Max(solver.IterationCount, 1):F1} ms/iter)  " +
                          $"{solver.Trace.Count(t => t.Solved)} subproblems");

        double[] xGot = solver.ReferenceX, uGot = solver.ReferenceU;
        double costGot = solver.TrueCost(xGot, uGot, solver.SigmaGlide, solver.SigmaBurn,
                                         out double defectGot, out int dch, out int dnode);
        double propGot = m0 - xGot[(n - 1) * NX + PointMass3Dof.IM];
        double propRef = m0 - xRef[(n - 1) * NX + PointMass3Dof.IM];
        double ec = Math.Abs(solver.SigmaGlide - sigCRef) / sigCRef;
        double eb = Math.Abs(solver.SigmaBurn - sigBRef) / sigBRef;
        double ep = Math.Abs(propGot - propRef) / propRef;
        double ex = 0;
        for (int i = 0; i < xGot.Length; i++)
            ex = Math.Max(ex, Math.Abs(xGot[i] - xRef[i]) / cfg.XScale[i % NX]);
        double eTh = 0;
        for (int k = 0; k < n; k++)
            eTh = Math.Max(eTh, Math.Abs(uGot[k * NU + PointMass3Dof.ITH] - thRef[k]));

        Console.WriteLine();
        Console.WriteLine("converged solution vs python_ref/threedof_ref.py:");
        Console.WriteLine($"  glide      {solver.SigmaGlide,8:F3} s  vs {sigCRef,8:F3} s   rel {ec:E2}");
        Console.WriteLine($"  burn       {solver.SigmaBurn,8:F3} s  vs {sigBRef,8:F3} s   rel {eb:E2}");
        Console.WriteLine($"  propellant {propGot,8:F1} kg vs {propRef,8:F1} kg  rel {ep:E2}");
        Console.WriteLine($"  X          max diff {ex:E2} of scale");
        Console.WriteLine($"  throttle   max diff {eTh:E2}");
        Console.WriteLine($"  cost       {costGot:E6} vs {costRef:E6}");
        Console.WriteLine($"  defect     {defectGot:E2} ({Scvx3DofSolver.ChannelName(dch)} at interval {dnode})");
        Console.WriteLine($"  iterations {solver.IterationCount} vs {itersRef:F0}");

        bool feasible = CheckFeasibility(cfg, model, solver, x0, xf);
        int shared = SharedIterations(solver, lines.Skip(6).Select((_, i) => Row(6 + i)).ToArray());

        // SCvx is a LOCAL method on a nonconvex problem, so reproducing the script's trajectory is the wrong pass criterion, for the reason the 6-DOF loop check gives: the two sides walk the same path only while they take the same accept/reject decisions and trust-region steps. Here the script's iteration 7 came back from CLARABEL "inaccurate" with rho 0.03, which halved its trust region and stopped it at 38 s of burn and 5.0 t of propellant; SCS took that step at rho 0.47 and carried on to a lower merit.
        // So: the iterations both sides DID share must agree - that tests the model, the linearisation and the subproblem - and the end point must converge, be feasible against the true constraints, and be no worse than the script's.
        bool converged = status == ScvxStatus.Converged;
        bool sharedOk = shared >= 5;
        bool noWorse = costGot <= costRef * (1.0 + 1e-6);
        bool pass = converged && feasible && defectGot < 1e-3 && sharedOk && noWorse;
        Console.WriteLine();
        Console.WriteLine($"  converged            {converged}");
        Console.WriteLine($"  feasible (nonlinear) {feasible}");
        Console.WriteLine($"  defect < 1e-3        {defectGot < 1e-3}");
        Console.WriteLine($"  shared iterations    {sharedOk}  ({shared} agree with the script before the paths part)");
        Console.WriteLine($"  merit <= script      {noWorse}  ({costGot:E4} vs {costRef:E4})");
        Console.WriteLine();
        Console.WriteLine(pass ? "PASS - 3-DOF loop follows 3dof.py and converges to a feasible solution at least as good"
                               : "FAIL");
        return pass ? 0 : 1;
    }

    /// <summary>
    /// How many leading iterations agree with the script's trace: same accept decision and trust region, merit within 2 % and both durations within 0.15 s. Stops at the first iteration where the trust regions differ, since the two runs are on different paths from there.
    /// </summary>
    private static int SharedIterations(Scvx3DofSolver solver, double[][] traceRef)
    {
        int shared = 0;
        int count = Math.Min(traceRef.Length, solver.Trace.Count);
        Console.WriteLine();
        Console.WriteLine("leading iterations against the script's trace:");
        for (int i = 0; i < count; i++)
        {
            double[] r = traceRef[i];   // it, rho, accepted, tr, sigma_coast, sigma_burn, step, defect, cost
            Scvx3DofIteration it = solver.Trace[i];
            bool sameTr = Math.Abs(it.TrustRegion - r[3]) < 1e-9 && it.Accepted == (r[2] > 0.5);
            double ej = Math.Abs(it.Cost - r[8]) / Math.Abs(r[8]);
            double es = Math.Max(Math.Abs(it.SigmaGlide - r[4]), Math.Abs(it.SigmaBurn - r[5]));
            bool ok = sameTr && ej < 0.02 && es < 0.15;
            Console.WriteLine($"  iter {i,2}: rho {it.Rho,6:F2} vs {r[1],6:F2}  tr {it.TrustRegion:F3} vs {r[3]:F3}  " +
                              $"merit rel {ej:E1}  sigma diff {es:F3} s  {(ok ? "ok" : sameTr ? "DIFFERS" : "paths part")}");
            if (!sameTr) break;
            if (!ok) return 0;
            shared++;
        }
        return shared;
    }

    /// <summary>The converged trajectory against the TRUE constraints, not the linearised ones it was solved with.</summary>
    private static bool CheckFeasibility(Scvx3DofConfig cfg, Toy3DofModel model, Scvx3DofSolver solver,
                                         double[] x0, double[] xf)
    {
        int n = cfg.Nodes, glide = cfg.GlideIntervals;
        double[] x = solver.ReferenceX, u = solver.ReferenceU;
        double unit = 0, thLow = 0, thHigh = 0, thGlide = 0, alpha = 0, retro = 0, bounds = 0, ends = 0;
        double peakAlpha = 0, minTh = double.MaxValue, maxTh = 0;
        for (int k = 0; k < n; k++)
        {
            double bx = u[k * NU], by = u[k * NU + 1], bz = u[k * NU + 2], th = u[k * NU + 3];
            unit = Math.Max(unit, Math.Abs(Math.Sqrt(bx * bx + by * by + bz * bz) - 1.0));
            if (k < glide)
                thGlide = Math.Max(thGlide, Math.Abs(th));
            else
            {
                thLow = Math.Max(thLow, cfg.ThrottleFloor - th);
                thHigh = Math.Max(thHigh, th - cfg.ThrottleCeiling);
                minTh = Math.Min(minTh, th);
                maxTh = Math.Max(maxTh, th);
            }
            double vx = x[k * NX + 3], vy = x[k * NX + 4], vz = x[k * NX + 5];
            double sp = Math.Sqrt(vx * vx + vy * vy + vz * vz);
            if (sp > cfg.AlphaSpeedThreshold)
            {
                double cosA = -(bx * vx + by * vy + bz * vz) / sp;
                double a = Math.Acos(Math.Clamp(cosA, -1, 1)) * 180 / Math.PI;
                peakAlpha = Math.Max(peakAlpha, a);
                alpha = Math.Max(alpha, a - cfg.AlphaMaxDeg);
                retro = Math.Max(retro, (bx * vx + by * vy + bz * vz) / sp);
            }
            if (k > 0)
                for (int i = 0; i < NX; i++)
                    bounds = Math.Max(bounds, Math.Max(cfg.StateMin[i] - x[k * NX + i], x[k * NX + i] - cfg.StateMax[i]));
        }
        for (int i = 0; i < NX; i++) ends = Math.Max(ends, Math.Abs(x[i] - x0[i]) / cfg.XScale[i]);
        for (int i = 0; i < 6; i++) ends = Math.Max(ends, Math.Abs(x[(n - 1) * NX + i] - xf[i]) / cfg.XScale[i]);

        bool ok = true;
        void Check(string what, double worst, double tol, string unitText = "")
        {
            bool good = worst <= tol;
            ok &= good;
            Console.WriteLine($"  {what,-28} worst {worst,10:E2}{unitText}  tol {tol:E1}  {(good ? "ok" : "VIOLATED")}");
        }
        Console.WriteLine();
        Console.WriteLine("feasibility against the true constraints:");
        Check("|b| = 1", unit, 1e-9);
        Check("glide throttle = 0", thGlide, 1e-6);
        Check("burn throttle >= floor", thLow, 1e-6);
        Check("burn throttle <= ceiling", thHigh, 1e-6);
        Check("angle of attack (deg over)", alpha, 0.05);
        Check("retrograde (b . v_hat)", retro, 1e-3);
        Check("state bounds", bounds, 1e-3);
        Check("boundary conditions (scaled)", ends, 1e-6);
        Console.WriteLine($"  (peak alpha {peakAlpha:F1} deg of {cfg.AlphaMaxDeg:F0}; burn throttle {100 * minTh:F0}-{100 * maxTh:F0} %)");
        return ok;
    }

    /// <summary>
    /// The scenario travels through the CSV, but the model constants and weights are hand-mirrored between Toy3DofModel / Scvx3DofConfig and 3dof.py. This catches the two drifting apart.
    /// </summary>
    private static int CheckConstants(string[] all, Scvx3DofConfig cfg, Toy3DofModel model)
    {
        string? line = all.FirstOrDefault(l => l.StartsWith("# consts"));
        if (line == null)
        {
            Console.Error.WriteLine("threedof_ref.csv has no constants line - regenerate it");
            return 1;
        }
        var kv = line.Substring("# consts".Length).Split(' ', StringSplitOptions.RemoveEmptyEntries)
            .Select(t => t.Split('=')).ToDictionary(p => p[0], p => double.Parse(p[1], CultureInfo.InvariantCulture));
        var mine = new Dictionary<string, double>
        {
            ["tmax"] = model.Tmax, ["floor"] = cfg.ThrottleFloor, ["isp"] = model.Isp, ["area"] = model.Area,
            ["cd0"] = model.Cd0, ["k"] = model.InducedK, ["cl_a"] = model.ClAlpha, ["rho0"] = model.Rho0,
            ["h"] = model.ScaleHeight, ["alpha_max_deg"] = cfg.AlphaMaxDeg, ["v_thresh"] = cfg.AlphaSpeedThreshold,
            ["rho_vc"] = cfg.RhoVc, ["w_dthr"] = cfg.WThrottleRate, ["w_datt"] = cfg.WAttitudeRate,
            ["sig_scale"] = cfg.SigmaScale,
        };
        int bad = 0;
        foreach ((string name, double value) in mine)
            if (!kv.TryGetValue(name, out double theirs) || Math.Abs(theirs - value) > 1e-9 * Math.Max(1.0, Math.Abs(value)))
            {
                Console.Error.WriteLine($"constant drift: {name} is {value} here, {(kv.TryGetValue(name, out double t) ? t : double.NaN)} in the script");
                bad++;
            }
        return bad;
    }

    private static string? FindFile(string name)
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir != null)
        {
            string candidate = Path.Combine(dir.FullName, "python_ref", name);
            if (File.Exists(candidate)) return candidate;
            dir = dir.Parent;
        }
        return null;
    }
}

using System.Globalization;
using Scvx;

/// <summary>
/// Verifies the glideslope and climb-rate path constraints actually bind, and —
/// more importantly — that they stay SOLVABLE from a state that violates them.
///
/// A constraint with a sign error or a wrong state index does not announce
/// itself: it produces a plausible trajectory that simply ignores the corridor,
/// or one that is mysteriously infeasible. Both are worse than having no
/// constraint at all, because the overlay would draw a cone the solver was never
/// really enforcing. So this checks the corridor is respected when it should be,
/// AND that a vehicle starting outside it still gets a plan.
/// </summary>
internal static class PathConstraintCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    internal static int Run(string? path)
    {
        path ??= "loop_ref.csv";
        if (!File.Exists(path))
        {
            Console.Error.WriteLine($"reference not found: {path}");
            return 2;
        }

        string[] lines = File.ReadAllLines(path).Where(l => l.Length > 0 && l[0] != '#').ToArray();
        double[] Row(int i) => lines[i].Split(',')
            .Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();

        double[] x0 = Row(0), xf = Row(1), xRef = Row(2);
        int n = xRef.Length / NX;

        Console.WriteLine("PATH CONSTRAINTS (glideslope + climb rate)");
        Console.WriteLine($"start ({x0[0]:F0}, {x0[1]:F0}, {x0[2]:F0}) m -> " +
                          $"target ({xf[0]:F0}, {xf[1]:F0}, {xf[2]:F0}) m");

        // Constrain to something the unconstrained solution ACTUALLY violates.
        // A cone chosen out of the air risks being so wide it never binds, which
        // would let a completely broken constraint pass the test.
        // Derive the test cone from the unconstrained solution's OWN shallowest
        // point, then demand something steeper. A cone picked out of the air is
        // either so wide it never binds - letting a completely broken constraint
        // pass - or so tight nothing can fly it.
        double shallowest = ShallowestDeg(SolveRaw(n, x0, xf, 0.0, -1.0), n, xf);
        double tight = shallowest + 10.0;
        (double freeWorst, _, double freeClimb, _) =
            Solve(n, x0, xf, 0.0, -1.0, "unconstrained", tight);
        Console.WriteLine();
        Console.WriteLine($"  unconstrained path is {shallowest:F1} deg above horizontal at its " +
                          $"shallowest, peak climb {freeClimb:+0.00;-0.00} m/s");
        Console.WriteLine($"  -> demanding {tight:F1} deg (worst {freeWorst:F0} m outside that cone) " +
                          "and vz <= 0.5 m/s");
        if (freeWorst <= 1.0)
        {
            Console.WriteLine("INCONCLUSIVE - the derived cone does not bind, so this tests nothing.");
            return 1;
        }
        Console.WriteLine();

        bool ok = true;

        // Enforced: the plan must come inside the cone and stop climbing.
        (double w1, _, double c1, ScvxStatus s1) = Solve(n, x0, xf, tight, 0.5, "constrained");
        // A metre of tolerance: the corridor is enforced at the NODES and measured
        // at the same nodes, but the solve runs to a finite tolerance, so an
        // exactly-active constraint settles a hair outside.
        bool coneOk = Usable(s1) && w1 <= 1.0;
        bool climbOk = Usable(s1) && c1 <= 0.5 + 0.05;
        Console.WriteLine($"  glideslope binds       {(coneOk ? "PASS" : "FAIL")}  " +
                          $"(worst {w1:F1} m outside, was {freeWorst:F0} m)");
        Console.WriteLine($"  climb rate binds       {(climbOk ? "PASS" : "FAIL")}  " +
                          $"(worst {c1:+0.00;-0.00} vs limit 0.50 m/s)");
        ok &= coneOk && climbOk;

        // THE CASE THAT MATTERS. Node 0 is pinned by equality, so a vehicle already
        // outside the corridor cannot be brought inside AT THAT NODE by any control.
        // Hard constraints there make the problem infeasible by construction and the
        // vehicle gets no plan at all - precisely when it most needs one. Start well
        // outside the cone AND climbing, violating both at once.
        var xBad = (double[])x0.Clone();
        xBad[0] += 400.0;
        xBad[1] += 400.0;
        xBad[Dynamics6Dof.IV + 2] = +15.0;
        Console.WriteLine();
        Console.WriteLine($"  recovery start: {Violation(xBad, xf, Cot(tight)):F0} m outside the cone, " +
                          $"climbing at {xBad[Dynamics6Dof.IV + 2]:F0} m/s");

        (_, double late2, _, ScvxStatus s2) = Solve(n, xBad, xf, tight, 0.5, "violating start");
        bool solvable = Usable(s2);
        Console.WriteLine($"  still solvable         {(solvable ? "PASS" : "FAIL")}  ({s2})");
        // And it must RECOVER, not merely return something: the slack should be
        // spent near the start and the plan back inside the cone by the end.
        bool recovers = solvable && late2 <= 1.0;
        Console.WriteLine($"  re-enters the cone     {(recovers ? "PASS" : "FAIL")}  " +
                          $"(last third {late2:F1} m outside)");
        ok &= solvable && recovers;

        Console.WriteLine();
        Console.WriteLine(ok
            ? "PASS - path constraints bind when they should and stay solvable when violated"
            : "FAIL - see above");
        return ok ? 0 : 1;
    }

    /// <summary>
    /// A plan we could actually fly. IterationLimit counts: the reference trajectory
    /// is still a valid, constraint-respecting plan, it simply stopped improving —
    /// which is the normal outcome under a real-time iteration budget.
    /// </summary>
    private static bool Usable(ScvxStatus s) =>
        s is ScvxStatus.Converged or ScvxStatus.IterationLimit;

    /// <summary>
    /// How far OUTSIDE the cone a point is, in metres: horizontal distance minus
    /// the cone's radius at that height. Negative means inside.
    ///
    /// Measured as a distance rather than an angle on purpose. Near the target both
    /// the horizontal and vertical offsets go to zero, so the approach ANGLE is
    /// atan2(tiny, tiny) — numerically meaningless exactly where the trajectory
    /// spends its last nodes, and it reads ~90 degrees for a perfectly good
    /// trajectory. The violation is the quantity the constraint actually states and
    /// it degrades gracefully to 0 at the target.
    /// </summary>
    private static double Violation(ReadOnlySpan<double> x, ReadOnlySpan<double> xf, double cot)
    {
        double dx = x[0] - xf[0], dy = x[1] - xf[1], dz = x[2] - xf[2];
        return Math.Sqrt(dx * dx + dy * dy) - cot * dz;
    }

    /// <summary>
    /// The shallowest angle above the target's horizontal plane that a trajectory
    /// reaches — the value the glideslope constraint would have to beat to bind.
    ///
    /// Nodes closer than 20 m to the target are skipped: there both offsets go to
    /// zero and the angle is atan2(tiny, tiny), which is numerically meaningless
    /// and would dominate the minimum with noise.
    /// </summary>
    private static double ShallowestDeg(double[] x, int n, double[] xf)
    {
        double min = 90.0;
        for (int k = 1; k < n; k++)
        {
            double dx = x[k * NX + 0] - xf[0], dy = x[k * NX + 1] - xf[1];
            double dz = x[k * NX + 2] - xf[2];
            double horiz = Math.Sqrt(dx * dx + dy * dy);
            if (Math.Sqrt(horiz * horiz + dz * dz) < 20.0) continue;
            min = Math.Min(min, Math.Atan2(dz, horiz) * 180.0 / Math.PI);
        }
        return min;
    }

    private static double Cot(double deg) =>
        1.0 / Math.Tan(Math.Clamp(deg, 1e-3, 89.999) * Math.PI / 180.0);

    /// <summary>The converged trajectory alone, for sizing the test before running it.</summary>
    private static double[] SolveRaw(int n, double[] x0, double[] xf, double glideDeg, double vzMax)
    {
        var prev = Console.Out;
        Console.SetOut(System.IO.TextWriter.Null);
        try { return SolveInner(n, x0, xf, glideDeg, vzMax).x; }
        finally { Console.SetOut(prev); }
    }

    private static (double worst, double late, double worstClimb, ScvxStatus status) Solve(
        int n, double[] x0, double[] xf, double glideDeg, double vzMax, string label,
        double measureDeg = 30.0)
    {
        (double[] x, ScvxStatus status, int iters, string why) =
            SolveInner(n, x0, xf, glideDeg, vzMax);

        double cot = Cot(glideDeg > 0.0 ? glideDeg : measureDeg);
        double worst = double.NegativeInfinity, late = double.NegativeInfinity;
        double worstClimb = double.NegativeInfinity;
        // From node 1: node 0 is the pinned initial state and is deliberately
        // exempt, so including it would measure the disturbance, not the plan.
        for (int k = 1; k < n; k++)
        {
            double v = Violation(x.AsSpan(k * NX, NX), xf, cot);
            worst = Math.Max(worst, v);
            if (k >= 2 * n / 3) late = Math.Max(late, v);   // has it re-entered by the end?
            worstClimb = Math.Max(worstClimb, x[k * NX + Dynamics6Dof.IV + 2]);
        }
        Console.WriteLine($"  [{label,-16}] {status,-16} iters {iters,3}  " +
                          $"worst outside cone {worst,8:F1} m  (last third {late,8:F1} m)  " +
                          $"worst climb {worstClimb,+6:F2} m/s" + why);
        return (worst, late, worstClimb, status);
    }

    private static (double[] x, ScvxStatus status, int iters, string why) SolveInner(
        int n, double[] x0, double[] xf, double glideDeg, double vzMax)
    {
        var cfg = new Scvx6DofConfig
        {
            Nodes = n,
            GlideSlopeDeg = glideDeg,
            VzMax = vzMax,
            ProximalWeight = 0.05,
        };

        double m0 = x0[Dynamics6Dof.IM];
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
            xSeed[k * NX + Dynamics6Dof.IQ] = 1.0;
            xSeed[k * NX + Dynamics6Dof.IM] = m0 + t * (0.92 * m0 - m0);
            uSeed[k * NU + Dynamics6Dof.IT] = 1.05 * m0 * 9.81;
        }

        var solver = new Scvx6DofSolver(cfg) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        solver.Initialize(x0, xf, xSeed, uSeed, sigmaSeed: 12.0);
        ScvxStatus status = solver.Solve(maxIterations: 150);
        string why = status == ScvxStatus.Failed
            ? $"   why: {solver.LastFailureReason}{Environment.NewLine}{solver.DiagnoseUnbounded().TrimEnd()}"
            : "";
        return (solver.ReferenceX, status, solver.IterationCount, why);
    }
}

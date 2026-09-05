using System.Runtime.InteropServices;

namespace Gfold;

/// <summary>
/// Solves a <see cref="ConicProblem"/> with Clarabel.
///
/// WHY A THIRD BACKEND. ECOS is interior-point and fast on this problem but GPLv3,
/// which forces the whole work to GPLv3. SCS is MIT but first-order, and G-FOLD is the
/// shape ADMM is worst at — small, banded, cold-started, on a frame budget — which
/// cost 2.4x per solve and 5x per search when it was measured against ECOS. Clarabel is
/// Apache-2.0 AND interior-point: the licence of the one and the algorithm class of the
/// other. If it performs like ECOS, it is the answer to the whole migration.
///
/// PROBLEM FORM IS SCS'S, NOT ECOS'S. Clarabel takes
///
///     minimize    (1/2) x'Px + q'x
///     subject to  Ax + s = b,  s in K
///
/// which is the single stacked matrix with a leading zero cone — exactly what
/// <see cref="SparseCcs.VStack"/> builds. That stacking was originally written for the
/// SCS binding, which took the same form; it outlived it. P is null here: the G-FOLD
/// objective is linear.
///
/// Cold start, no persistent state: every G-FOLD
/// call is a new plan, and a static function with no fields is safe to call from the
/// mod's solver thread without a per-vehicle instance.
///
/// clarabel_c.dll has to be built first (gfold/build-clarabel.ps1, needs a Rust
/// toolchain). Every layout in ClarabelNative is a reading of the vendored headers, and
/// struct layout is the part of a P/Invoke binding that fails silently, so verify with
/// Gfold.Console --clarabel-layout (pure reflection, needs no DLL) and
/// --clarabel-smoke (a problem whose answer is known by hand).
/// </summary>
public static class ClarabelSolver
{
    /// <summary>
    /// Iteration ceiling. An interior-point method's iteration count is bounded in
    /// practice — ECOS solves this problem in 11 to 20 — so unlike ADMM's cap this is
    /// a guard against pathology, not a budget that shapes the answer.
    /// </summary>
    public const int DefaultMaxIterations = 200;

    /// <summary>
    /// Convergence tolerance, applied to Clarabel's absolute and relative gap and to
    /// its feasibility tolerance.
    ///
    /// Left tight on purpose. A first-order method's tolerance has to be measured and
    /// loosened because ADMM's cost scales like 1/eps; an IPM's scales like log(1/eps),
    /// so accuracy is nearly free here and the tolerance stops being a tuning knob.
    /// Clarabel's own defaults are 1e-8, which is the same order as ECOS's.
    /// </summary>
    public const double DefaultEps = 1e-8;

    /// <summary>Timing and convergence detail from the last solve, for the A/B harness.</summary>
    public sealed record ClarabelSolveInfo(
        ClarabelExit Status, int Iterations, double ObjectiveValue,
        double ResPrimal, double ResDual, double SolverSolveTimeS, double TotalMs);

    /// <summary>
    /// The vendored Clarabel version, for the harness banner.
    /// A CONSTANT, not a query: Clarabel's C API exposes no version
    /// entry point (unlike scs_version), so this tracks gfold/clarabel/Clarabel.rs's
    /// Cargo.toml by hand and must be bumped when that is updated.
    /// </summary>
    public const string NativeVersion = "0.11.1";

    /// <summary>Prints the marshalled struct layouts, to be diffed against the C headers.</summary>
    public static string DumpLayouts() => ClarabelNative.DumpLayouts();

    /// <summary>
    /// What clarabel_DefaultSettings_f64_default() actually hands back. The layout dump
    /// proves .NET and the C header AGREE about offsets; this proves the values arrive
    /// intact, which is a different question — a by-value struct return is marshalled,
    /// and the two failures look identical from the outside.
    /// </summary>
    public static string DumpDefaultSettings()
    {
        ClarabelNative.ClarabelDefaultSettings d =
            ClarabelNative.clarabel_DefaultSettings_f64_default();
        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"max_iter={d.MaxIter}  time_limit={d.TimeLimit}  verbose={d.Verbose}");
        sb.AppendLine($"max_step_fraction={d.MaxStepFraction}  tol_gap_abs={d.TolGapAbs}  tol_feas={d.TolFeas}");
        sb.AppendLine($"equilibrate_enable={d.EquilibrateEnable}  equilibrate_max_iter={d.EquilibrateMaxIter}");
        sb.AppendLine($"max_threads={d.MaxThreads}  direct_kkt_solver={d.DirectKktSolver}  direct_solve_method={d.DirectSolveMethod}");
        sb.AppendLine($"static_reg_enable={d.StaticRegularizationEnable}  static_reg_const={d.StaticRegularizationConstant}");
        sb.AppendLine($"iter_refine_enable={d.IterativeRefinementEnable}  iter_refine_max_iter={d.IterativeRefinementMaxIter}");
        sb.AppendLine($"presolve_enable={d.PresolveEnable}");
        return sb.ToString();
    }

    public static ConicResult Solve(ConicProblem problem, bool verbose = false,
                                    int maxIterations = DefaultMaxIterations,
                                    double eps = DefaultEps,
                                    double timeLimitS = 0.0)
        => Solve(problem, out _, verbose, maxIterations, eps, timeLimitS);

    /// <param name="timeLimitS">
    /// Wall-clock ceiling, seconds, or 0 for none. Clarabel reports hitting it as
    /// MaxTime, a first-class status rather than something to be recovered from a
    /// status string — which is what the first-order backend it replaced forced.
    /// </param>
    public static ConicResult Solve(ConicProblem problem, out ClarabelSolveInfo info,
                                    bool verbose = false,
                                    int maxIterations = DefaultMaxIterations,
                                    double eps = DefaultEps,
                                    double timeLimitS = 0.0)
    {
        int n = problem.Vars;
        int p = problem.EqualityRows;
        int mCone = problem.ConeRows;
        int m = p + mCone;

        if (problem.C.Length != n)
            throw new ArgumentException($"c has length {problem.C.Length}, expected n={n}");
        if (problem.H.Length != mCone)
            throw new ArgumentException($"h has length {problem.H.Length}, expected m={mCone}");
        int coneSum = problem.PositiveOrthantDim + problem.SocDims.Sum();
        if (coneSum != mCone)
            throw new ArgumentException($"cone dims sum to {coneSum}, expected m={mCone}");
        if (problem.A != null && problem.A.Cols != n)
            throw new ArgumentException("A and G column counts differ");
        if (problem.A != null && problem.B?.Length != p)
            throw new ArgumentException($"b has length {problem.B?.Length}, expected p={p}");

        // Same stacking as the SCS path: equalities on top as a zero cone, then the
        // orthant, then the second-order cones, matching the cone list built below.
        SparseCcs stacked = problem.A != null
            ? SparseCcs.VStack(problem.A, problem.G)
            : problem.G;
        var rhs = new double[m];
        if (problem.A != null)
            Array.Copy(problem.B!, rhs, p);
        Array.Copy(problem.H, 0, rhs, p, mCone);

        (double[] pr, int[] jc, int[] ir) = stacked.Build();

        // WIDEN THE INDICES. Clarabel's CSC arrays are uintptr_t; SparseCcs speaks the
        // 32-bit ints ECOS and SCS both want. Handing the int arrays over directly
        // would have the native side read them at twice the stride.
        var colPtr = new nuint[jc.Length];
        for (int i = 0; i < jc.Length; i++) colPtr[i] = (nuint)jc[i];
        var rowVal = new nuint[ir.Length];
        for (int i = 0; i < ir.Length; i++) rowVal[i] = (nuint)ir[i];

        // The cone list, in the row order of the stacked matrix.
        var cones = new List<ClarabelNative.ClarabelSupportedCone>();
        if (p > 0)
            cones.Add(ClarabelNative.ClarabelSupportedCone.Of(ClarabelNative.ConeTag.Zero, p));
        if (problem.PositiveOrthantDim > 0)
            cones.Add(ClarabelNative.ClarabelSupportedCone.Of(
                ClarabelNative.ConeTag.Nonnegative, problem.PositiveOrthantDim));
        foreach (int q in problem.SocDims)
            cones.Add(ClarabelNative.ClarabelSupportedCone.Of(
                ClarabelNative.ConeTag.SecondOrder, q));
        ClarabelNative.ClarabelSupportedCone[] coneArray = cones.ToArray();

        var pins = new List<GCHandle>();
        IntPtr Pin(Array array)
        {
            var handle = GCHandle.Alloc(array, GCHandleType.Pinned);
            pins.Add(handle);
            return handle.AddrOfPinnedObject();
        }

        long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
        try
        {
            var aMat = new ClarabelNative.ClarabelCscMatrix
            {
                M = (nuint)m, N = (nuint)n,
                ColPtr = Pin(colPtr), RowVal = Pin(rowVal), NzVal = Pin(pr),
            };

            // P is the zero matrix: a linear objective. Clarabel still wants a
            // well-formed n x n CSC, so hand it one with no entries — an all-zero
            // column pointer array of length n+1 and null index/value pointers, which
            // is the representation CscMatrix.h documents for a zero matrix.
            var pMat = new ClarabelNative.ClarabelCscMatrix
            {
                M = (nuint)n, N = (nuint)n,
                ColPtr = Pin(new nuint[n + 1]), RowVal = IntPtr.Zero, NzVal = IntPtr.Zero,
            };

            ClarabelNative.ClarabelDefaultSettings settings =
                ClarabelNative.clarabel_DefaultSettings_f64_default();
            settings.Verbose = verbose;
            settings.MaxIter = (uint)Math.Max(1, maxIterations);
            // INFINITY, NOT ZERO, FOR "NO LIMIT". SCS guards its time limit with
            // `if (stgs->time_limit_secs)`, so 0 there means "unset"; Clarabel takes
            // the number literally and its own default is f64::INFINITY. Passing SCS's
            // 0 made every solve exit immediately with MaxTime after 0 iterations —
            // while still returning the correct answer, because it had already been
            // computed. The same parameter name, opposite meanings for the same value.
            settings.TimeLimit = timeLimitS > 0 ? timeLimitS : double.PositiveInfinity;
            settings.TolGapAbs = eps;
            settings.TolGapRel = eps;
            settings.TolFeas = eps;

            IntPtr solver = ClarabelNative.clarabel_DefaultSolver_f64_new(
                ref pMat, Pin(problem.C), ref aMat, Pin(rhs),
                (nuint)coneArray.Length, Pin(coneArray), ref settings);
            if (solver == IntPtr.Zero)
            {
                info = new ClarabelSolveInfo(ClarabelExit.Unsolved, 0,
                                             double.NaN, double.NaN, double.NaN, 0, Ms(t0));
                return new ConicResult(ConicStatus.SetupFailed, [], double.NaN, 0);
            }

            try
            {
                ClarabelNative.clarabel_DefaultSolver_f64_solve(solver);
                ClarabelNative.ClarabelDefaultSolution sol =
                    ClarabelNative.clarabel_DefaultSolver_f64_solution(solver);

                // x is owned by the solver and dies with it — copy before the free.
                var x = new double[n];
                if (sol.X != IntPtr.Zero && (int)sol.XLength >= n)
                    Marshal.Copy(sol.X, x, 0, n);

                info = new ClarabelSolveInfo(sol.Status, (int)sol.Iterations, sol.ObjVal,
                                             sol.RPrim, sol.RDual, sol.SolveTime, Ms(t0));
                return new ConicResult(MapStatus(sol.Status), x, sol.ObjVal, (int)sol.Iterations);
            }
            finally
            {
                ClarabelNative.clarabel_DefaultSolver_f64_free(solver);
            }
        }
        finally
        {
            foreach (GCHandle handle in pins)
                handle.Free();
        }
    }

    /// <summary>
    /// Clarabel's status onto the shared one.
    ///
    /// "Almost" is Clarabel's reduced-tolerance outcome — it converged, just to the
    /// looser of its two tolerance sets — which is the same meaning ECOS gives its
    /// inaccurate exits, so it maps to OptimalInaccurate and stays usable.
    /// MaxIterations, MaxTime and InsufficientProgress all mean the iterate is not a
    /// solution and must NOT be flown, so they land outside GfoldPlanner.IsUsable.
    /// </summary>
    private static ConicStatus MapStatus(ClarabelExit s) => s switch
    {
        ClarabelExit.Solved => ConicStatus.Optimal,
        ClarabelExit.AlmostSolved => ConicStatus.OptimalInaccurate,
        ClarabelExit.PrimalInfeasible => ConicStatus.PrimalInfeasible,
        ClarabelExit.AlmostPrimalInfeasible => ConicStatus.PrimalInfeasibleInaccurate,
        ClarabelExit.DualInfeasible => ConicStatus.DualInfeasible,
        ClarabelExit.AlmostDualInfeasible => ConicStatus.DualInfeasibleInaccurate,
        ClarabelExit.MaxIterations => ConicStatus.MaxIterations,
        ClarabelExit.MaxTime => ConicStatus.MaxIterations,
        ClarabelExit.InsufficientProgress => ConicStatus.Numerics,
        ClarabelExit.NumericalError => ConicStatus.Numerics,
        ClarabelExit.CallbackTerminated => ConicStatus.Interrupted,
        _ => ConicStatus.Fatal,
    };

    private static double Ms(long since) =>
        (System.Diagnostics.Stopwatch.GetTimestamp() - since) * 1000.0
        / System.Diagnostics.Stopwatch.Frequency;
}

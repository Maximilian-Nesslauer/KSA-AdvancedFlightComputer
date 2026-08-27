using System.Runtime.InteropServices;

namespace Gfold;

/// <summary>
/// Solves a <see cref="ConicProblem"/> with SCS.
///
/// COLD START, ALWAYS, AND NO PERSISTENT STATE. Scvx.Core's ScsWorkspace is an object
/// precisely because an SCvx iteration is a small perturbation of the last one and the
/// previous iterate is worth keeping. G-FOLD is the opposite: every call is a new plan
/// from a new vehicle state, and inside SearchMinFuel the successive solves are
/// different times of flight probed by a golden section — neighbouring in tf, but with
/// no reference trajectory relating their solutions, and evaluated in an order the
/// caller's cache decides rather than one that means anything. A warm start there would
/// make each solve depend on which tf happened to be tried before it, which is how a
/// search becomes irreproducible. So this is a static function with no fields, which
/// also makes it safe to call from the mod's solver thread without a per-vehicle
/// instance — the mistake the UPFG port had to be rescued from.
///
/// scs_init deep-copies everything handed to it (unlike ECOS_setup, which retains raw
/// pointers into the caller's arrays for the workspace's life), so the pins here only
/// have to survive the init call. They are held to the end of the solve anyway, which
/// costs nothing and is one less lifetime to reason about.
/// </summary>
public static class ScsSolver
{
    /// <summary>
    /// Iteration ceiling. ADMM needs headroom an interior-point method never does, and
    /// a low cap does not fail loudly — it returns a half-converged iterate flagged
    /// only as "solved (inaccurate)". <see cref="ScsSolveInfo.HitIterationLimit"/> is
    /// what makes that visible; this number exists so hitting it means something is
    /// wrong rather than that the budget was mean.
    /// </summary>
    public const int DefaultMaxIterations = 100_000;

    /// <summary>
    /// Convergence tolerance, absolute and relative. MEASURED, not chosen — see
    /// Gfold.Console --ab, whose sweep on the Mars reference case (N=120) gives:
    ///
    ///     eps     P4 solve    fuel error vs ECOS      full tf search
    ///     1e-4      33 ms          2.41 kg            1.7 s, tf +0.24 s, fuel +4.03 kg
    ///     1e-5      78 ms          0.05 kg            7.8 s, tf -0.06 s, fuel +0.06 kg
    ///     1e-6     358 ms          0.01 kg           50.5 s, tf -0.06 s, fuel -0.14 kg
    ///     1e-7    2784 ms          0.00 kg           69.0 s, tf +17.3 s, fuel +41.5 kg
    ///
    /// (ECOS, for scale: 23 ms per solve, 0.7 s for the whole search.)
    ///
    /// TIGHTER IS NOT BETTER, AND PAST A POINT IT IS MUCH WORSE. The 1e-7 row is not
    /// noise: individual solves still converge there, but enough of them exhaust the
    /// iteration budget inside a search that they come back as MaxIterations, the
    /// search discards those times of flight as infeasible, and it brackets the
    /// minimum somewhere else entirely — 17 seconds of flight time and 41 kg of fuel
    /// away from the right answer. A first-order solver degrades by returning a WORSE
    /// DECISION, not a looser number, and that is the failure mode to design against.
    ///
    /// 1e-5 is the knee: fuel agrees with ECOS to 0.06 kg out of ~316 (0.02%) and the
    /// chosen time of flight to 0.06 s, which is well inside the search's own 0.25 s
    /// resolution. Everything tighter buys accuracy the search cannot use and pays for
    /// it in the only currency that matters here.
    /// </summary>
    public const double DefaultEps = 1e-5;

    public static string NativeVersion => ScsNative.Version();

    /// <summary>Timing and convergence detail from the last solve, for the A/B harness.</summary>
    public sealed record ScsSolveInfo(
        ScsExit Exit, string StatusText, int Iterations, bool HitIterationLimit,
        double PrimalObjective, double DualObjective, double Gap,
        double ResPri, double ResDual, double InitMs, double SolveMs);

    public static ConicResult Solve(ConicProblem problem, bool verbose = false,
                                    int maxIterations = DefaultMaxIterations,
                                    double eps = DefaultEps)
        => Solve(problem, out _, verbose, maxIterations, eps);

    public static ConicResult Solve(ConicProblem problem, out ScsSolveInfo info,
                                    bool verbose = false,
                                    int maxIterations = DefaultMaxIterations,
                                    double eps = DefaultEps)
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

        // ECOS's split form -> SCS's single matrix. The equalities go on TOP, because
        // SCS's cone ordering is fixed: zero cone first, then the positive orthant,
        // then the second-order cones, and the rows of A must appear in that order.
        // A zero cone pins s = 0 over those rows, so "A x + s = b with s in {0}" is
        // the equality constraint verbatim — no sign or scaling change anywhere.
        SparseCcs stacked = problem.A != null
            ? SparseCcs.VStack(problem.A, problem.G)
            : problem.G;
        var rhs = new double[m];
        if (problem.A != null)
            Array.Copy(problem.B!, rhs, p);
        Array.Copy(problem.H, 0, rhs, p, mCone);

        (double[] pr, int[] jc, int[] ir) = stacked.Build();

        var pins = new List<GCHandle>();
        IntPtr Pin(Array array)
        {
            var handle = GCHandle.Alloc(array, GCHandleType.Pinned);
            pins.Add(handle);
            return handle.AddrOfPinnedObject();
        }

        var x = new double[n];
        var y = new double[m];
        var s = new double[m];

        try
        {
            var aMat = new ScsNative.ScsMatrix
            {
                X = Pin(pr), I = Pin(ir), P = Pin(jc), M = m, N = n,
            };
            var data = new ScsNative.ScsData
            {
                M = m, N = n,
                A = Pin(new[] { aMat }),
                P = IntPtr.Zero,              // linear objective: no quadratic term
                B = Pin(rhs), C = Pin(problem.C),
            };
            var cone = new ScsNative.ScsCone
            {
                Z = p,
                L = problem.PositiveOrthantDim,
                Q = problem.SocDims.Length > 0 ? Pin(problem.SocDims) : IntPtr.Zero,
                Qsize = problem.SocDims.Length,
            };

            var settings = new ScsNative.ScsSettings();
            ScsNative.scs_set_default_settings(ref settings);
            settings.Verbose = verbose ? 1 : 0;
            settings.MaxIters = maxIterations;
            settings.EpsAbs = eps;
            settings.EpsRel = eps;
            settings.WarmStart = 0;

            long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
            IntPtr work = ScsNative.scs_init(ref data, ref cone, ref settings);
            double initMs = Ms(t0);
            if (work == IntPtr.Zero)
            {
                info = new ScsSolveInfo(ScsExit.Failed, "scs_init failed", 0, false,
                                        double.NaN, double.NaN, double.NaN,
                                        double.NaN, double.NaN, initMs, 0);
                return new ConicResult(ConicStatus.SetupFailed, [], double.NaN, 0);
            }

            try
            {
                var sol = new ScsNative.ScsSolution { X = Pin(x), Y = Pin(y), S = Pin(s) };
                // Status/LinSysSolver are ByValArray-marshaled fixed-size buffers: the
                // managed arrays must already exist at the declared SizeConst before
                // the call, or the marshaler has nothing to copy the native bytes into.
                var raw = new ScsNative.ScsInfo
                {
                    Status = new byte[128], LinSysSolver = new byte[128],
                };

                long t1 = System.Diagnostics.Stopwatch.GetTimestamp();
                int exitCode = ScsNative.scs_solve(work, ref sol, ref raw, 0);
                double solveMs = Ms(t1);

                var exit = Enum.IsDefined(typeof(ScsExit), exitCode)
                    ? (ScsExit)exitCode : ScsExit.Failed;
                bool truncated = raw.Iter >= maxIterations;
                info = new ScsSolveInfo(
                    exit,
                    System.Text.Encoding.ASCII.GetString(raw.Status).TrimEnd('\0').Trim(),
                    raw.Iter, truncated, raw.Pobj, raw.Dobj, raw.Gap,
                    raw.ResPri, raw.ResDual, initMs, solveMs);

                return new ConicResult(MapStatus(exit, truncated), x, raw.Pobj, raw.Iter);
            }
            finally
            {
                ScsNative.scs_finish(work);
            }
        }
        finally
        {
            foreach (GCHandle handle in pins)
                handle.Free();
        }
    }

    /// <summary>
    /// SCS's exit code onto the shared status.
    ///
    /// The one judgement call is SolvedInaccurate. SCS reports it both for a solution
    /// that converged loosely and for one that merely ran out of iterations, and those
    /// are not the same thing: a truncated ADMM iterate can violate constraints by
    /// orders of magnitude and still be returned. Truncation is therefore reported as
    /// MaxIterations — which GfoldPlanner.IsUsable rejects — while a genuinely loose
    /// convergence maps to OptimalInaccurate, which it accepts, exactly as ECOS's own
    /// inaccurate exit does.
    /// </summary>
    private static ConicStatus MapStatus(ScsExit exit, bool hitIterationLimit) => exit switch
    {
        ScsExit.Solved => ConicStatus.Optimal,
        ScsExit.SolvedInaccurate => hitIterationLimit
            ? ConicStatus.MaxIterations
            : ConicStatus.OptimalInaccurate,
        ScsExit.Infeasible => ConicStatus.PrimalInfeasible,
        ScsExit.InfeasibleInaccurate => ConicStatus.PrimalInfeasibleInaccurate,
        ScsExit.Unbounded => ConicStatus.DualInfeasible,
        ScsExit.UnboundedInaccurate => ConicStatus.DualInfeasibleInaccurate,
        ScsExit.Interrupted => ConicStatus.Interrupted,
        ScsExit.Indeterminate => ConicStatus.Numerics,
        ScsExit.Unfinished => ConicStatus.MaxIterations,
        _ => ConicStatus.Fatal,
    };

    private static double Ms(long since) =>
        (System.Diagnostics.Stopwatch.GetTimestamp() - since) * 1000.0
        / System.Diagnostics.Stopwatch.Frequency;
}

using System.Runtime.InteropServices;

namespace AdvancedFlightComputer.Guidance.Conic;

/// <summary>
/// Solves the SCvx subproblem's cone-QP with Clarabel, as a drop-in for <see cref="ScsWorkspace"/>: same
/// problem data, same call, same status type.
///
/// NO ITERATE WARM START. Clarabel is interior-point: every solve starts from its own central point, and a
/// previous solution is no help to it. So <c>warmStart</c> is accepted and ignored. What carries across
/// solves instead is the SOLVER: one native Clarabel solver per workspace, built on the first solve, then
/// refreshed in place with update_P/A/q/b and solved again. That keeps its symbolic factorisation (the AMD
/// ordering and elimination tree of the KKT matrix) and its equilibration, and skips the setup a fresh
/// solver pays. It is safe because a subproblem's CcsAssembler freezes its sparsity pattern on the first
/// assembly and only rewrites values after that, so the pattern a solver was built with stays the pattern.
///
/// Settings for real-time use, from Maxi's measurements (the Clarabel-for-SCS review):
///   * presolve and zero-dropping OFF, which in-place updates require (a solver with either refuses an
///     update, and the C wrapper turns that refusal into a panic inside the DLL);
///   * iterative refinement OFF: Clarabel polishes each KKT solve far below what an SCvx subproblem asks
///     for. A NumericalError or InsufficientProgress with it off turns it back on for this workspace from
///     then on, rebuilds, and solves once more;
///   * the primary gap and feasibility tolerances clamp the subproblem's epsAbs/epsRel at
///     <see cref="MaxPrimaryTolerance"/> (5e-6). Clarabel's default reduced ("almost") tolerances
///     (5e-5 gap, 1e-4 feasibility) stay unchanged so unrefined solves keep a 10x margin between
///     primary convergence and reduced-accuracy acceptance.
///
/// The solver is rebuilt when the pattern, cone list or tolerance changes; nothing else.
///
/// The equilibration a persistent solver keeps is the one it computed on its FIRST problem. Clarabel
/// applies those row and column scalings to every update rather than recomputing them. The subproblems
/// already equilibrate their own rows and scale their columns to physical units before the solve, so
/// Clarabel's own scaling is a refinement on a problem that is well scaled to start with.
/// </summary>
public sealed class ClarabelWorkspace : IScvxConicWorkspace, IDisposable
{
    /// <summary>Interior-point iteration ceiling: a guard against pathology, not a budget. See ClarabelSolver.DefaultMaxIterations.</summary>
    public const int IpmIterationCeiling = ClarabelSolver.DefaultMaxIterations;

    /// <summary>
    /// Upper bound on Clarabel's primary gap and feasibility tolerances. SCvx callers pass 1e-4 or 1e-3
    /// for first-order ADMM speed, but an interior-point method reaches 5e-6 at virtually identical
    /// iteration count while staying 10x tighter than Clarabel's default reduced tolerances (5e-5 / 1e-4).
    /// </summary>
    public const double MaxPrimaryTolerance = 5e-6;

    /// <summary>Primary tolerance applied to Clarabel for a requested (epsAbs, epsRel) pair.</summary>
    public static double EffectiveTolerance(double epsAbs, double epsRel) =>
        Math.Min(Math.Max(epsAbs, epsRel), MaxPrimaryTolerance);

    /// <summary>Iterative refinement on every new solver. Off is Maxi's setting; on is Clarabel's default.</summary>
    public static bool IterativeRefinementDefault = false;

    /// <summary>Equilibration iterations, or null to keep Clarabel's default.</summary>
    public static uint? EquilibrateMaxIter;

    /// <summary>Keep the native solver between solves and update it in place. False builds a fresh one every solve (for A/B checks).</summary>
    public static bool Persistent = true;

    private IntPtr _solver;
    private bool _iterativeRefinement = IterativeRefinementDefault;

    // What the live solver was built from: the frozen CCS arrays (stable for an assembler's life), the cones, the tolerance and the cap.
    private int[]? _aColPtr, _aRowIdx, _pColPtr, _pRowIdx;
    private int _m, _n, _zeroDim, _posDim, _maxIter;
    private int[] _socDims = [];
    private double _eps;

    private bool _hasSolution;

    public double[] X { get; private set; } = [];
    public double PrimalObjective { get; private set; }
    public int Iterations { get; private set; }
    public string StatusText { get; private set; } = "";
    public bool HitIterationLimit { get; private set; }
    public ClarabelExit NativeStatus { get; private set; }

    /// <summary>Whether the last solve rebuilt the native solver (setup paid) rather than updating it.</summary>
    public bool LastRebuilt { get; private set; }
    public double LastSetupMs { get; private set; }
    public double LastSolveMs { get; private set; }

    /// <summary>
    /// True once a usable solution is stored. Clarabel does not start from it, but the SCvx loops read this
    /// as "the last subproblem solve was good", so it keeps ScsWorkspace's meaning.
    /// </summary>
    public bool HasWarmStart => _hasSolution;

    public void ResetWarmStart() => _hasSolution = false;

    public int EffectiveIterationCap(int requested) => Math.Clamp(requested, 1, IpmIterationCeiling);

    // Shared statistics kept in a bounded ring buffer so long sessions do not grow unbounded.
    private static readonly object StatsGate = new();
    private const int MaxSolveSamples = 1024;
    private static int _solveSampleNext;
    public static double TotalSetupMs, TotalSolveMs;
    public static long TotalCalls, TotalRebuilds, TotalIpmIterations, TotalRefinementFallbacks;
    public static readonly List<double> SolveMsSamples = [];

    public static void ResetTimers()
    {
        lock (StatsGate)
        {
            TotalSetupMs = TotalSolveMs = 0;
            TotalCalls = TotalRebuilds = TotalIpmIterations = TotalRefinementFallbacks = 0;
            SolveMsSamples.Clear();
            _solveSampleNext = 0;
        }
    }

    public static double SolveMsPercentile(double q)
    {
        double[] sorted;
        lock (StatsGate)
        {
            if (SolveMsSamples.Count == 0) return 0.0;
            sorted = SolveMsSamples.ToArray();
        }
        Array.Sort(sorted);
        int i = (int)Math.Round(q * (sorted.Length - 1));
        return sorted[Math.Clamp(i, 0, sorted.Length - 1)];
    }

    private static double Ms(long since) =>
        (System.Diagnostics.Stopwatch.GetTimestamp() - since) * 1000.0
        / System.Diagnostics.Stopwatch.Frequency;

    public ScsStatus Solve(CcsAssembler A, double[] b, double[] c, CcsAssembler? P,
                           int zeroDim, int posDim, int[] socDims,
                           bool warmStart, bool verbose = false,
                           int maxIterations = ScsWorkspace.DefaultMaxIterations,
                           double epsAbs = ScsWorkspace.DefaultEps, double epsRel = ScsWorkspace.DefaultEps)
    {
        int n = A.Cols, m = A.Rows;
        if (c.Length != n) throw new ArgumentException($"c is {c.Length}, expected n={n}");
        if (b.Length != m) throw new ArgumentException($"b is {b.Length}, expected m={m}");
        int coneSum = zeroDim + posDim + socDims.Sum();
        if (coneSum != m) throw new ArgumentException($"cone dims sum to {coneSum}, expected m={m}");

        // Clamp primary gap and feasibility tolerances at MaxPrimaryTolerance so loose ADMM settings
        // do not stop an unrefined interior-point solve early; ReducedTol* stays at Clarabel's defaults.
        double eps = EffectiveTolerance(epsAbs, epsRel);
        int cap = EffectiveIterationCap(maxIterations);
        bool hasP = P is { NonZeros: > 0 };

        long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
        bool rebuilt = false;
        if (!Persistent || _solver == IntPtr.Zero || verbose
            || !SameStructure(A, hasP ? P : null, m, n, zeroDim, posDim, socDims, eps, cap))
        {
            Rebuild(A, b, c, hasP ? P : null, m, n, zeroDim, posDim, socDims, eps, cap, verbose);
            rebuilt = true;
        }
        else
        {
            Update(A, b, c, hasP ? P : null, n, m);
        }
        LastSetupMs = Ms(t0);

        long t1 = System.Diagnostics.Stopwatch.GetTimestamp();
        ClarabelNative.clarabel_DefaultSolver_f64_solve(_solver);
        ClarabelNative.ClarabelDefaultSolution sol = ClarabelNative.clarabel_DefaultSolver_f64_solution(_solver);

        // Recover once when an in-place update fails or when an unrefined solve hits numerical trouble:
        // an in-place update reuses the first solve's Ruiz equilibration, so rebuilding refreshes scaling
        // even when iterative refinement is already enabled.
        bool badExit = sol.Status is not (ClarabelExit.Solved or ClarabelExit.AlmostSolved
            or ClarabelExit.PrimalInfeasible or ClarabelExit.AlmostPrimalInfeasible
            or ClarabelExit.DualInfeasible or ClarabelExit.AlmostDualInfeasible);
        bool numericTrouble = sol.Status is ClarabelExit.NumericalError or ClarabelExit.InsufficientProgress;
        if (badExit && (!rebuilt || (!_iterativeRefinement && numericTrouble)))
        {
            if (!_iterativeRefinement && numericTrouble)
            {
                _iterativeRefinement = true;
                lock (StatsGate) TotalRefinementFallbacks++;
            }
            Rebuild(A, b, c, hasP ? P : null, m, n, zeroDim, posDim, socDims, eps, cap, verbose);
            rebuilt = true;
            ClarabelNative.clarabel_DefaultSolver_f64_solve(_solver);
            sol = ClarabelNative.clarabel_DefaultSolver_f64_solution(_solver);
        }
        LastSolveMs = Ms(t1);
        LastRebuilt = rebuilt;

        var x = new double[n];
        if (sol.X != IntPtr.Zero && (int)sol.XLength >= n)
            Marshal.Copy(sol.X, x, 0, n);
        X = x;
        PrimalObjective = sol.ObjVal;
        Iterations = (int)sol.Iterations;
        NativeStatus = sol.Status;
        HitIterationLimit = sol.Status is ClarabelExit.MaxIterations or ClarabelExit.MaxTime;
        StatusText = sol.Status.ToString();

        ScsStatus status = Map(sol.Status);
        _hasSolution = status.IsUsable() && !HitIterationLimit;

        if (!Persistent)
            Free();

        lock (StatsGate)
        {
            TotalSetupMs += LastSetupMs;
            TotalSolveMs += LastSolveMs;
            TotalCalls++;
            if (rebuilt) TotalRebuilds++;
            TotalIpmIterations += Iterations;
            double totalMs = LastSetupMs + LastSolveMs;
            if (SolveMsSamples.Count < MaxSolveSamples)
                SolveMsSamples.Add(totalMs);
            else
            {
                SolveMsSamples[_solveSampleNext] = totalMs;
                _solveSampleNext = (_solveSampleNext + 1) % MaxSolveSamples;
            }
        }
        return status;
    }

    /// <summary>
    /// Clarabel's exit onto SCS's, keeping ScsWorkspace's conventions: an iteration or time cap reads as
    /// SolvedInaccurate WITH HitIterationLimit set, which the SCvx loops refuse as a step, exactly as they
    /// refuse a truncated ADMM run. AlmostSolved met Clarabel's reduced tolerances and stays usable, as
    /// SCS's inaccurate-but-converged exit does.
    /// </summary>
    private static ScsStatus Map(ClarabelExit s) => s switch
    {
        ClarabelExit.Solved => ScsStatus.Solved,
        ClarabelExit.AlmostSolved => ScsStatus.SolvedInaccurate,
        ClarabelExit.MaxIterations or ClarabelExit.MaxTime => ScsStatus.SolvedInaccurate,
        ClarabelExit.PrimalInfeasible => ScsStatus.Infeasible,
        ClarabelExit.AlmostPrimalInfeasible => ScsStatus.InfeasibleInaccurate,
        ClarabelExit.DualInfeasible => ScsStatus.Unbounded,
        ClarabelExit.AlmostDualInfeasible => ScsStatus.UnboundedInaccurate,
        ClarabelExit.CallbackTerminated => ScsStatus.Interrupted,
        _ => ScsStatus.Failed,
    };

    private bool SameStructure(CcsAssembler A, CcsAssembler? P, int m, int n, int zeroDim, int posDim,
                               int[] socDims, double eps, int cap)
    {
        if (m != _m || n != _n || zeroDim != _zeroDim || posDim != _posDim || eps != _eps || cap != _maxIter)
            return false;
        if (!ReferenceEquals(A.ColumnPointers, _aColPtr) || !ReferenceEquals(A.RowIndices, _aRowIdx))
            return false;
        if (!ReferenceEquals(P?.ColumnPointers, _pColPtr) || !ReferenceEquals(P?.RowIndices, _pRowIdx))
            return false;
        return socDims.AsSpan().SequenceEqual(_socDims);
    }

    private void Update(CcsAssembler A, double[] b, double[] c, CcsAssembler? P, int n, int m)
    {
        var pins = new List<GCHandle>(4);
        IntPtr Pin(Array array)
        {
            GCHandle h = GCHandle.Alloc(array, GCHandleType.Pinned);
            pins.Add(h);
            return h.AddrOfPinnedObject();
        }
        try
        {
            if (P != null)
                ClarabelNative.clarabel_DefaultSolver_f64_update_P(_solver, Pin(P.Values), (nuint)P.NonZeros);
            if (A.NonZeros > 0)
                ClarabelNative.clarabel_DefaultSolver_f64_update_A(_solver, Pin(A.Values), (nuint)A.NonZeros);
            ClarabelNative.clarabel_DefaultSolver_f64_update_q(_solver, Pin(c), (nuint)n);
            ClarabelNative.clarabel_DefaultSolver_f64_update_b(_solver, Pin(b), (nuint)m);
        }
        finally
        {
            foreach (GCHandle h in pins) h.Free();
        }
    }

    private void Rebuild(CcsAssembler A, double[] b, double[] c, CcsAssembler? P, int m, int n,
                         int zeroDim, int posDim, int[] socDims, double eps, int cap, bool verbose)
    {
        Free();

        var cones = new List<ClarabelNative.ClarabelSupportedCone>(socDims.Length + 2);
        if (zeroDim > 0) cones.Add(ClarabelNative.ClarabelSupportedCone.Of(ClarabelNative.ConeTag.Zero, zeroDim));
        if (posDim > 0) cones.Add(ClarabelNative.ClarabelSupportedCone.Of(ClarabelNative.ConeTag.Nonnegative, posDim));
        foreach (int q in socDims)
            cones.Add(ClarabelNative.ClarabelSupportedCone.Of(ClarabelNative.ConeTag.SecondOrder, q));
        ClarabelNative.ClarabelSupportedCone[] coneArray = cones.ToArray();

        var pins = new List<GCHandle>();
        IntPtr Pin(Array array)
        {
            GCHandle h = GCHandle.Alloc(array, GCHandleType.Pinned);
            pins.Add(h);
            return h.AddrOfPinnedObject();
        }
        static nuint[] Widen(int[] v)
        {
            var w = new nuint[v.Length];
            for (int i = 0; i < v.Length; i++) w[i] = (nuint)v[i];
            return w;
        }

        try
        {
            // Clarabel's CSC indices are uintptr_t (see ClarabelNative). The arrays are copied by the solver, so they only stay pinned for this call.
            var aMat = new ClarabelNative.ClarabelCscMatrix
            {
                M = (nuint)m, N = (nuint)n,
                ColPtr = Pin(Widen(A.ColumnPointers)),
                RowVal = A.NonZeros > 0 ? Pin(Widen(A.RowIndices)) : IntPtr.Zero,
                NzVal = A.NonZeros > 0 ? Pin(A.Values) : IntPtr.Zero,
            };
            var pMat = new ClarabelNative.ClarabelCscMatrix
            {
                M = (nuint)n, N = (nuint)n, ColPtr = Pin(new nuint[n + 1]),
                RowVal = IntPtr.Zero, NzVal = IntPtr.Zero,
            };
            if (P != null)
            {
                // Upper triangle only: Clarabel keeps the upper triangle of what it is given, so a lower entry would be dropped and the update length would no longer match.
                int[] jc = P.ColumnPointers, ir = P.RowIndices;
                for (int col = 0; col < n; col++)
                    for (int k = jc[col]; k < jc[col + 1]; k++)
                        if (ir[k] > col)
                            throw new ArgumentException($"P has an entry below the diagonal at ({ir[k]},{col})");
                pMat.ColPtr = Pin(Widen(jc));
                pMat.RowVal = Pin(Widen(ir));
                pMat.NzVal = Pin(P.Values);
            }

            ClarabelNative.ClarabelDefaultSettings s = ClarabelNative.clarabel_DefaultSettings_f64_default();
            s.Verbose = verbose;
            s.MaxIter = (uint)cap;
            s.TimeLimit = double.PositiveInfinity;   // see ClarabelSolver: 0 means "stop now" to Clarabel
            s.TolGapAbs = eps;
            s.TolGapRel = eps;
            s.TolFeas = eps;
            // Keep Clarabel's default ReducedTol* (5e-5 gap, 1e-4 feasibility) for AlmostSolved fallback.
            s.PresolveEnable = false;
            s.InputSparseDropzeros = false;
            s.IterativeRefinementEnable = _iterativeRefinement;
            if (EquilibrateMaxIter is uint eq) s.EquilibrateMaxIter = eq;

            _solver = ClarabelNative.clarabel_DefaultSolver_f64_new(
                ref pMat, Pin(c), ref aMat, Pin(b),
                (nuint)coneArray.Length, Pin(coneArray), ref s);
            if (_solver == IntPtr.Zero)
                throw new InvalidOperationException("clarabel_DefaultSolver_f64_new failed");
        }
        finally
        {
            foreach (GCHandle h in pins) h.Free();
        }

        _aColPtr = A.ColumnPointers; _aRowIdx = A.RowIndices;
        _pColPtr = P?.ColumnPointers; _pRowIdx = P?.RowIndices;
        _m = m; _n = n; _zeroDim = zeroDim; _posDim = posDim;
        _socDims = (int[])socDims.Clone();
        _eps = eps; _maxIter = cap;
    }

    private void Free()
    {
        if (_solver != IntPtr.Zero)
        {
            ClarabelNative.clarabel_DefaultSolver_f64_free(_solver);
            _solver = IntPtr.Zero;
        }
    }

    public void Dispose()
    {
        Free();
        GC.SuppressFinalize(this);
    }

    ~ClarabelWorkspace() => Free();
}

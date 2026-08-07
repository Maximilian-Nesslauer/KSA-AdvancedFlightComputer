using System.Runtime.InteropServices;

namespace Scvx;

/// <summary>
/// Solves one SCS cone-QP and carries the solution forward for the next call's
/// warm start.
///
/// Unlike EcosWorkspace this holds NO persistent native handle between solves.
/// scs_init's own documentation says it "maintains deep copies" of everything
/// handed to it, so — unlike ECOS, which keeps raw pointers into the caller's
/// arrays for the workspace's whole life and requires them pinned throughout —
/// every array here only needs to stay pinned for the duration of the scs_init
/// call itself. That also means there is nothing to gain from keeping a
/// long-lived ScsWork around: scs_update only refreshes b and c (see
/// ScsNative.cs), and our A/P values change every SCvx iteration regardless, so
/// a fresh scs_init is required every solve either way. The actual warm start
/// SCS offers is at the ITERATE level — the previous x/y/s fed back in via
/// scs_solve's warm_start flag — which this class provides by holding onto the
/// last solution and reusing it as the next call's starting point.
/// </summary>
public sealed class ScsWorkspace
{
    /// <summary>
    /// Iteration budget and tolerance for OFFLINE / validation solves.
    ///
    /// These are NOT SCS's own defaults (100k / 1e-4) and emphatically not
    /// interior-point-style defaults. ADMM needs iteration headroom the way an
    /// IPM never does, so a low cap silently degrades the answer rather than
    /// failing loudly — which is what the large budget exists to prevent.
    ///
    /// The TOLERANCE, however, is deliberately conservative and should NOT be
    /// used for flight. Measured across the whole SCvx loop at N=30, eps
    /// 1e-5 / 1e-6 / 1e-7 all converge in 17 iterations to the SAME answer to
    /// five significant figures (merit 9.7619e-2, defect 9.5e-6, peak tilt
    /// 6.1 deg, burn 24.2 s) — but cost 1.7 s / 3.1 s / 5.5 s respectively, and
    /// in receding horizon 33 ms / 334 ms / 1266 ms per cycle. ADMM's tail is
    /// almost the entire bill and it buys nothing here. Set
    /// <c>Scvx6DofSolver.SubproblemEps</c> to 1e-5 for real-time use.
    ///
    /// Caveat that motivated the conservative default: an under-solved
    /// subproblem is dangerous in a DIFFERENT way (see
    /// <see cref="HitIterationLimit"/>) — the guard there, not a tight
    /// tolerance, is what actually keeps the loop honest.
    /// </summary>
    public const int DefaultMaxIterations = 100_000;
    public const double DefaultEps = 1e-7;

    /// <summary>
    /// Anderson acceleration memory, or null to leave SCS's own default (10).
    ///
    /// This only does anything because aa.c is now compiled with USE_LAPACK — see
    /// native_src/blas_shim.c. Before that the entire accelerator was a no-op and
    /// this setting had no effect whatsoever.
    /// </summary>
    public static int? AccelerationLookback;

    private double[]? _prevX, _prevY, _prevS;

    public double[] X { get; private set; } = [];
    public double[] Y { get; private set; } = [];
    public double[] S { get; private set; } = [];
    public double PrimalObjective { get; private set; }
    public int Iterations { get; private set; }
    public string StatusText { get; private set; } = "";

    /// <summary>
    /// True if the solve stopped because it ran out of iterations rather than
    /// because it converged.
    ///
    /// SCS reports this as SolvedInaccurate ("solved (inaccurate - reached
    /// max_iters)"), which <see cref="ScsStatusEx.IsUsable"/> counts as usable —
    /// and for a one-off solve it broadly is. It is NOT usable as an SCvx step:
    /// a truncated ADMM iterate can violate the trust region by orders of
    /// magnitude while still being returned, and feeding that to the ratio test
    /// produces a wildly wrong rho and an accepted garbage step.
    /// </summary>
    public bool HitIterationLimit { get; private set; }

    /// <summary>
    /// Wall-clock split of the last Solve call, in milliseconds.
    ///
    /// This exists because "the solve is slow" is ambiguous: scs_init rebuilds the
    /// ENTIRE factorisation every call (deep copy, Ruiz equilibration, AMD ordering,
    /// symbolic + numeric LDL) while scs_solve runs the ADMM sweeps. Optimising the
    /// wrong one is wasted work, so measure the split before touching either.
    /// </summary>
    public double LastInitMs { get; private set; }
    public double LastSolveMs { get; private set; }
    public double LastFinishMs { get; private set; }

    /// <summary>Totals since the last <see cref="ResetTimers"/>, for sweeps.</summary>
    public static double TotalInitMs, TotalSolveMs, TotalFinishMs;
    public static long TotalCalls, TotalAdmmIterations;

    /// <summary>
    /// Per-solve wall clock, for percentiles. The MEAN is the wrong statistic for a
    /// real-time loop: a hitch is what the player sees, and a mean of 80 ms could be
    /// almost every solve at 5 ms plus a handful at two seconds. Those two
    /// distributions need completely different fixes, so measure the tail directly.
    /// </summary>
    public static readonly List<double> SolveMsSamples = [];

    public static void ResetTimers()
    {
        TotalInitMs = TotalSolveMs = TotalFinishMs = 0;
        TotalCalls = TotalAdmmIterations = 0;
        SolveMsSamples.Clear();
    }

    /// <summary>Percentile of the recorded per-solve times, 0..1. Empty -> 0.</summary>
    public static double SolveMsPercentile(double q)
    {
        if (SolveMsSamples.Count == 0) return 0.0;
        var sorted = SolveMsSamples.ToArray();
        Array.Sort(sorted);
        int i = (int)Math.Round(q * (sorted.Length - 1));
        return sorted[Math.Clamp(i, 0, sorted.Length - 1)];
    }

    private static double Ms(long since) =>
        (System.Diagnostics.Stopwatch.GetTimestamp() - since) * 1000.0
        / System.Diagnostics.Stopwatch.Frequency;

    public static string NativeVersion => ScsNative.Version();

    /// <summary>Debug aid: .NET's actual computed struct layout for the P/Invoke types.</summary>
    public static string DumpNativeStructLayouts() => ScsNative.DumpLayouts();

    /// <summary>
    /// True once a usable solution has been stored and not since discarded.
    /// The SCvx loop must key its warm start off THIS, not off its own iteration
    /// counter: a receding-horizon Reseed resets that counter while the solver
    /// iterate is still perfectly good, so counting iterations threw the warm
    /// start away on exactly the cycles that needed it.
    /// </summary>
    public bool HasWarmStart => _prevX != null;

    /// <summary>Discard the carried solution, e.g. after a large jump the previous iterate can't help with.</summary>
    public void ResetWarmStart()
    {
        _prevX = _prevY = _prevS = null;
    }

    /// <summary>
    /// Solves  minimize (1/2) x'Px + c'x  subject to  Ax + s = b,  s in K,
    /// where K = zero-cone(zeroDim) x positive-orthant(posDim) x SOC(socDims...),
    /// rows of A/b given in exactly that order.
    ///
    /// A and P are CcsAssemblers already filled with this call's values (P upper
    /// triangular, may be empty if the objective is purely linear).
    /// </summary>
    public ScsStatus Solve(CcsAssembler A, double[] b, double[] c, CcsAssembler? P,
                           int zeroDim, int posDim, int[] socDims,
                           bool warmStart, bool verbose = false,
                           int maxIterations = DefaultMaxIterations,
                           double epsAbs = DefaultEps, double epsRel = DefaultEps,
                           bool keepTruncatedIterate = false)
    {
        LastInitMs = LastSolveMs = LastFinishMs = 0;
        int n = A.Cols, m = A.Rows;
        if (c.Length != n) throw new ArgumentException($"c is {c.Length}, expected n={n}");
        if (b.Length != m) throw new ArgumentException($"b is {b.Length}, expected m={m}");
        int coneSum = zeroDim + posDim + socDims.Sum();
        if (coneSum != m) throw new ArgumentException($"cone dims sum to {coneSum}, expected m={m}");

        var pins = new List<GCHandle>();
        IntPtr Pin(Array array)
        {
            var handle = GCHandle.Alloc(array, GCHandleType.Pinned);
            pins.Add(handle);
            return handle.AddrOfPinnedObject();
        }

        double[] xBuf = new double[n];
        double[] yBuf = new double[m];
        double[] sBuf = new double[m];
        bool useWarmStart = warmStart && _prevX is { Length: var lx } && lx == n
                                       && _prevY is { Length: var ly } && ly == m
                                       && _prevS is { Length: var ls } && ls == m;
        if (useWarmStart)
        {
            Array.Copy(_prevX!, xBuf, n);
            Array.Copy(_prevY!, yBuf, m);
            Array.Copy(_prevS!, sBuf, m);
        }

        try
        {
            var aMat = new ScsNative.ScsMatrix
            {
                X = Pin(A.Values), I = Pin(A.RowIndices), P = Pin(A.ColumnPointers),
                M = m, N = n,
            };
            IntPtr aMatPtr = Pin(new[] { aMat });

            IntPtr pMatPtr = IntPtr.Zero;
            if (P is { NonZeros: > 0 })
            {
                var pMat = new ScsNative.ScsMatrix
                {
                    X = Pin(P.Values), I = Pin(P.RowIndices), P = Pin(P.ColumnPointers),
                    M = n, N = n,
                };
                pMatPtr = Pin(new[] { pMat });
            }

            var data = new ScsNative.ScsData
            {
                M = m, N = n, A = aMatPtr, P = pMatPtr,
                B = Pin(b), C = Pin(c),
            };
            var cone = new ScsNative.ScsCone
            {
                Z = zeroDim, L = posDim,
                Q = socDims.Length > 0 ? Pin(socDims) : IntPtr.Zero, Qsize = socDims.Length,
            };
            var settings = new ScsNative.ScsSettings();
            ScsNative.scs_set_default_settings(ref settings);
            settings.Verbose = verbose ? 1 : 0;
            settings.MaxIters = maxIterations;
            settings.EpsAbs = epsAbs;
            settings.EpsRel = epsRel;
            settings.WarmStart = useWarmStart ? 1 : 0;
            if (AccelerationLookback is int look) settings.AccelerationLookback = look;

            long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
            IntPtr work = ScsNative.scs_init(ref data, ref cone, ref settings);
            LastInitMs = Ms(t0);
            if (work == IntPtr.Zero)
                throw new InvalidOperationException("scs_init failed");

            try
            {
                var sol = new ScsNative.ScsSolution
                {
                    X = Pin(xBuf), Y = Pin(yBuf), S = Pin(sBuf),
                };
                // Status/LinSysSolver are ByValArray-marshaled fixed-size buffers:
                // the managed arrays must already be allocated at the declared
                // SizeConst before the call, or the marshaler has nothing to copy
                // the native bytes into.
                var info = new ScsNative.ScsInfo { Status = new byte[128], LinSysSolver = new byte[128] };
                long t1 = System.Diagnostics.Stopwatch.GetTimestamp();
                int exit = ScsNative.scs_solve(work, ref sol, ref info, useWarmStart ? 1 : 0);
                LastSolveMs = Ms(t1);

                X = xBuf; Y = yBuf; S = sBuf;
                PrimalObjective = info.Pobj;
                Iterations = info.Iter;
                HitIterationLimit = info.Iter >= maxIterations;
                StatusText = System.Text.Encoding.ASCII.GetString(info.Status)
                    .TrimEnd('\0');

                var status = Enum.IsDefined(typeof(ScsStatus), exit)
                    ? (ScsStatus)exit : ScsStatus.Failed;

                // Only carry a USABLE solution forward. On infeasible/unbounded/
                // failed exits SCS leaves a certificate (or nothing meaningful) in
                // x/y/s, not a primal point — seeding the next ADMM run with that
                // would poison every subsequent solve, and silently, since a bad
                // warm start degrades convergence rather than erroring. The SCvx
                // loop retries after shrinking the trust region, so this is a live
                // path, not a theoretical one.
                // ...and NOT a TRUNCATED one either. SolvedInaccurate passes
                // IsUsable(), so before this check a solve that merely ran out of
                // ADMM iterations was stored and became the next solve's starting
                // point — seeding the next run from a half-converged iterate, which
                // makes IT more likely to truncate too. That is the mechanism behind
                // long solves arriving in BURSTS rather than singly: one truncation
                // poisons the warm start and the next few inherit it.
                // keepTruncatedIterate inverts the rule below ON PURPOSE, and only
                // for a caller deliberately RESUMING this same subproblem in slices.
                // There a truncated iterate is not a contaminated answer to a
                // different question - it is the exact ADMM state this run left off
                // at, and discarding it would restart the solve every slice and never
                // converge.
                if (status.IsUsable() && (!HitIterationLimit || keepTruncatedIterate))
                {
                    _prevX = xBuf; _prevY = yBuf; _prevS = sBuf;
                }

                return status;
            }
            finally
            {
                long t2 = System.Diagnostics.Stopwatch.GetTimestamp();
                ScsNative.scs_finish(work);
                LastFinishMs = Ms(t2);
                TotalInitMs += LastInitMs;
                TotalSolveMs += LastSolveMs;
                TotalFinishMs += LastFinishMs;
                TotalCalls++;
                TotalAdmmIterations += Iterations;
                SolveMsSamples.Add(LastInitMs + LastSolveMs + LastFinishMs);
            }
        }
        finally
        {
            foreach (GCHandle h in pins)
                h.Free();
        }
    }
}

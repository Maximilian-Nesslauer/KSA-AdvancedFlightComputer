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
    /// Iteration budget and tolerance that this problem actually needs.
    ///
    /// These are NOT SCS's own defaults (100k / 1e-4) and emphatically not
    /// interior-point-style defaults. Measured on the reference subproblem:
    /// 5000 iters at 1e-6 returned "solved (inaccurate — reached max_iters)"
    /// with the objective ~5% out; 1e-7 with a large budget converges properly
    /// in ~5400 iterations cold, ~750 warm. ADMM needs iteration headroom the
    /// way an IPM never does, so a low cap silently degrades the answer rather
    /// than failing loudly — which is exactly the trap these constants exist to
    /// keep the SCvx loop out of.
    /// </summary>
    public const int DefaultMaxIterations = 100_000;
    public const double DefaultEps = 1e-7;

    private double[]? _prevX, _prevY, _prevS;

    public double[] X { get; private set; } = [];
    public double[] Y { get; private set; } = [];
    public double[] S { get; private set; } = [];
    public double PrimalObjective { get; private set; }
    public int Iterations { get; private set; }
    public string StatusText { get; private set; } = "";

    public static string NativeVersion => ScsNative.Version();

    /// <summary>Debug aid: .NET's actual computed struct layout for the P/Invoke types.</summary>
    public static string DumpNativeStructLayouts() => ScsNative.DumpLayouts();

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
                           double epsAbs = DefaultEps, double epsRel = DefaultEps)
    {
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

            IntPtr work = ScsNative.scs_init(ref data, ref cone, ref settings);
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
                int exit = ScsNative.scs_solve(work, ref sol, ref info, useWarmStart ? 1 : 0);

                X = xBuf; Y = yBuf; S = sBuf;
                PrimalObjective = info.Pobj;
                Iterations = info.Iter;
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
                if (status.IsUsable())
                {
                    _prevX = xBuf; _prevY = yBuf; _prevS = sBuf;
                }

                return status;
            }
            finally
            {
                ScsNative.scs_finish(work);
            }
        }
        finally
        {
            foreach (GCHandle h in pins)
                h.Free();
        }
    }
}

using System.Runtime.InteropServices;

namespace Scvx;

/// <summary>
/// A long-lived ECOS workspace for a problem whose sparsity pattern is fixed and
/// whose numbers change every solve — the SCvx subproblem.
///
///   minimize    c'x
///   subject to  A x = b
///               G x + s = h,   s in K = R+^l x SOC(q0) x SOC(q1) x ...
///
/// Setup (allocation, AMD ordering, symbolic LDL factorisation) happens once in
/// the constructor. Each subsequent <see cref="Solve"/> hands ECOS the same
/// pinned arrays, refilled with fresh values, via ECOS_updateData — which
/// rebuilds only the KKT numerical entries.
///
/// Every array handed to ECOS stays pinned for the object's lifetime: ECOS keeps
/// raw pointers into all of them from setup until cleanup, and equilibration
/// writes back through those pointers.
/// </summary>
public sealed class EcosWorkspace : IDisposable
{
    private readonly CcsAssembler _g;
    private readonly CcsAssembler? _a;
    private readonly double[] _c, _h, _b;
    private readonly int[] _socDims;

    private readonly List<GCHandle> _pins = [];
    private IntPtr _work;
    private bool _firstSolve = true;

    public int Variables { get; }
    public double[] X { get; }
    public double PrimalCost { get; private set; }
    public int Iterations { get; private set; }

    public static string NativeVersion => EcosNative.Version();

    /// <summary>
    /// Takes ownership of the assemblers and vectors: they are pinned and reused
    /// for the object's life. Refill them through the same objects, then call
    /// <see cref="Solve"/>.
    /// </summary>
    public EcosWorkspace(CcsAssembler g, double[] h, CcsAssembler? a, double[] b,
                         double[] c, int positiveOrthantDim, int[] socDims)
    {
        _g = g;
        _a = a;
        _c = c;
        _h = h;
        _b = b;
        _socDims = socDims;

        int n = g.Cols, m = g.Rows, p = a?.Rows ?? 0;
        Variables = n;
        X = new double[n];

        if (c.Length != n) throw new ArgumentException($"c is {c.Length}, expected n={n}");
        if (h.Length != m) throw new ArgumentException($"h is {h.Length}, expected m={m}");
        if (a != null)
        {
            if (a.Cols != n) throw new ArgumentException("A and G column counts differ");
            if (b.Length != p) throw new ArgumentException($"b is {b.Length}, expected p={p}");
        }
        int coneSum = positiveOrthantDim + socDims.Sum();
        if (coneSum != m) throw new ArgumentException($"cone dims sum to {coneSum}, expected m={m}");

        _work = EcosNative.ECOS_setup(
            n, m, p,
            positiveOrthantDim, socDims.Length, Pin(socDims), 0,
            Pin(g.Values), Pin(g.ColumnPointers), Pin(g.RowIndices),
            a == null ? IntPtr.Zero : Pin(a.Values),
            a == null ? IntPtr.Zero : Pin(a.ColumnPointers),
            a == null ? IntPtr.Zero : Pin(a.RowIndices),
            Pin(c), Pin(h), a == null ? IntPtr.Zero : Pin(b));

        if (_work == IntPtr.Zero)
        {
            Unpin();
            throw new InvalidOperationException("ECOS_setup failed");
        }
    }

    public void SetTolerances(double feasTol, double absTol, double relTol) =>
        EcosNative.ecsh_set_tols(_work, feasTol, absTol, relTol);

    public void SetMaxIterations(int maxIterations) =>
        EcosNative.ecsh_set_maxit(_work, maxIterations);

    /// <summary>
    /// Solves with whatever is currently in the assemblers and vectors.
    ///
    /// The first call uses the data ECOS already equilibrated during setup; every
    /// later call pushes the refilled values through ECOS_updateData first. Refill
    /// ALL of G, A, c, h and b before each call — updateData re-equilibrates from
    /// scratch and assumes the arrays hold raw, unscaled values.
    /// </summary>
    public EcosStatus Solve(bool verbose = false, int maxIterations = 100, int refinement = 30)
    {
        ObjectDisposedException.ThrowIf(_work == IntPtr.Zero, this);

        if (!_firstSolve)
        {
            EcosNative.ECOS_updateData(
                _work,
                PinnedAddress(_g.Values),
                _a == null ? IntPtr.Zero : PinnedAddress(_a.Values),
                PinnedAddress(_c), PinnedAddress(_h),
                _a == null ? IntPtr.Zero : PinnedAddress(_b));
        }
        _firstSolve = false;

        EcosNative.ecsh_set_verbose(_work, verbose ? 1 : 0);
        EcosNative.ecsh_set_maxit(_work, maxIterations);
        // Our hand-assembled problems are less well scaled than a CVXPY
        // canonicalisation; the extra KKT refinement prevents "unreliable search
        // direction" breakdowns at negligible cost.
        EcosNative.ecsh_set_nitref(_work, refinement);

        int exit = EcosNative.ECOS_solve(_work);
        Marshal.Copy(EcosNative.ecsh_x(_work), X, 0, Variables);
        PrimalCost = EcosNative.ecsh_pcost(_work);
        Iterations = EcosNative.ecsh_iter(_work);
        return Enum.IsDefined(typeof(EcosStatus), exit) ? (EcosStatus)exit : EcosStatus.Fatal;
    }

    private IntPtr Pin(Array array)
    {
        var handle = GCHandle.Alloc(array, GCHandleType.Pinned);
        _pins.Add(handle);
        return handle.AddrOfPinnedObject();
    }

    private IntPtr PinnedAddress(Array array)
    {
        foreach (GCHandle h in _pins)
            if (ReferenceEquals(h.Target, array))
                return h.AddrOfPinnedObject();
        throw new InvalidOperationException("array was not pinned at setup");
    }

    private void Unpin()
    {
        foreach (GCHandle handle in _pins)
            if (handle.IsAllocated)
                handle.Free();
        _pins.Clear();
    }

    public void Dispose()
    {
        if (_work != IntPtr.Zero)
        {
            EcosNative.ECOS_cleanup(_work, 0);
            _work = IntPtr.Zero;
        }
        Unpin();
    }
}

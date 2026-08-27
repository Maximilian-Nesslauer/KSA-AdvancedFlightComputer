using System.Runtime.InteropServices;

namespace Gfold;

public static class EcosSolver
{
    public static string NativeVersion => EcosNative.Version();

    // Solves the problem. The input arrays are treated as consumed: ECOS
    // equilibrates (scales) them in place during setup.
    public static ConicResult Solve(ConicProblem problem, bool verbose = false, int maxIterations = 100)
    {
        (double[] gpr, int[] gjc, int[] gir) = problem.G.Build();
        int n = problem.G.Cols;
        int m = problem.G.Rows;
        int p = problem.A?.Rows ?? 0;

        if (problem.C.Length != n)
            throw new ArgumentException($"c has length {problem.C.Length}, expected n={n}");
        if (problem.H.Length != m)
            throw new ArgumentException($"h has length {problem.H.Length}, expected m={m}");
        int coneSum = problem.PositiveOrthantDim + problem.SocDims.Sum();
        if (coneSum != m)
            throw new ArgumentException($"cone dims sum to {coneSum}, expected m={m}");

        double[]? apr = null;
        int[]? ajc = null, air = null;
        if (problem.A != null)
        {
            if (problem.A.Cols != n)
                throw new ArgumentException("A and G column counts differ");
            if (problem.B?.Length != p)
                throw new ArgumentException($"b has length {problem.B?.Length}, expected p={p}");
            (apr, ajc, air) = problem.A.Build();
        }

        // ECOS retains pointers into all of these arrays from setup until
        // cleanup (and equilibration writes through them), so pin everything
        // for the whole scope.
        var pins = new List<GCHandle>();
        IntPtr Pin(Array? array)
        {
            if (array == null)
                return IntPtr.Zero;
            var handle = GCHandle.Alloc(array, GCHandleType.Pinned);
            pins.Add(handle);
            return handle.AddrOfPinnedObject();
        }

        try
        {
            IntPtr work = EcosNative.ECOS_setup(
                n, m, p,
                problem.PositiveOrthantDim, problem.SocDims.Length, Pin(problem.SocDims), 0,
                Pin(gpr), Pin(gjc), Pin(gir),
                Pin(apr), Pin(ajc), Pin(air),
                Pin(problem.C), Pin(problem.H), Pin(problem.B));
            if (work == IntPtr.Zero)
                return new ConicResult(ConicStatus.SetupFailed, [], double.NaN, 0);

            try
            {
                EcosNative.ecsh_set_verbose(work, verbose ? 1 : 0);
                EcosNative.ecsh_set_maxit(work, maxIterations);
                // Generous iterative refinement: our hand-assembled problems are
                // less well scaled than CVXPY canonicalizations, and the extra
                // KKT refinement steps prevent "unreliable search direction"
                // breakdowns at negligible cost.
                EcosNative.ecsh_set_nitref(work, 30);

                int exit = EcosNative.ECOS_solve(work);

                var x = new double[n];
                Marshal.Copy(EcosNative.ecsh_x(work), x, 0, n);
                return new ConicResult(
                    Enum.IsDefined(typeof(ConicStatus), exit) ? (ConicStatus)exit : ConicStatus.Fatal,
                    x, EcosNative.ecsh_pcost(work), EcosNative.ecsh_iter(work));
            }
            finally
            {
                EcosNative.ECOS_cleanup(work, 0);
            }
        }
        finally
        {
            foreach (GCHandle handle in pins)
                handle.Free();
        }
    }
}

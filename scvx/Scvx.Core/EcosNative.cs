using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Scvx;

/// <summary>
/// P/Invoke surface for the vendored ecos.dll. Deliberately a separate binding
/// from Gfold.Core's: that one solves single one-shot problems and tears the
/// workspace down each time, whereas SCvx needs the workspace kept alive across
/// iterations so ECOS_updateData can reuse the symbolic factorisation.
///
/// Duplicating a dozen DllImports is the cost of Scvx.Core staying free of
/// project references. NativeLibrary.SetDllImportResolver is per-assembly, so
/// both bindings can register their own resolver without conflicting.
/// </summary>
internal static partial class EcosNative
{
    private const string Lib = "ecos";

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern IntPtr ECOS_setup(
        int n, int m, int p, int l, int ncones, IntPtr q, int nex,
        IntPtr gpr, IntPtr gjc, IntPtr gir,
        IntPtr apr, IntPtr ajc, IntPtr air,
        IntPtr c, IntPtr h, IntPtr b);

    /// <summary>
    /// Swaps in fresh numerical data for G, A, c, h and b, re-equilibrates, and
    /// refreshes the KKT entries — keeping the sparsity pattern, the ordering and
    /// the symbolic factorisation from setup.
    ///
    /// The data passed must be UNEQUILIBRATED. ECOS only un-scales the previous
    /// data when the new arrays live elsewhere in memory; passing the same arrays
    /// back (which is the point — they were pinned at setup) skips that step and
    /// equilibrates whatever is now in them. So overwrite them with fresh raw
    /// values before every call, never with values ECOS has already scaled.
    /// </summary>
    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void ECOS_updateData(
        IntPtr work, IntPtr gpr, IntPtr apr, IntPtr c, IntPtr h, IntPtr b);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ECOS_solve(IntPtr work);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void ECOS_cleanup(IntPtr work, int keepvars);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern IntPtr ECOS_ver();

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern IntPtr ecsh_x(IntPtr work);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern double ecsh_pcost(IntPtr work);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ecsh_iter(IntPtr work);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void ecsh_set_verbose(IntPtr work, int verbose);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void ecsh_set_maxit(IntPtr work, int maxit);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void ecsh_set_nitref(IntPtr work, int nitref);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void ecsh_set_tols(IntPtr work, double feastol, double abstol, double reltol);

    internal static string Version() => Marshal.PtrToStringAnsi(ECOS_ver()) ?? "?";
}

public enum EcosStatus
{
    Optimal = 0,
    PrimalInfeasible = 1,
    DualInfeasible = 2,
    OptimalInaccurate = 10,
    PrimalInfeasibleInaccurate = 11,
    DualInfeasibleInaccurate = 12,
    MaxIterations = -1,
    Numerics = -2,
    OutsideCone = -3,
    Interrupted = -4,
    SetupFailed = -5,
    Fatal = -7,
}

public static class EcosStatusEx
{
    public static bool IsUsable(this EcosStatus s) =>
        s is EcosStatus.Optimal or EcosStatus.OptimalInaccurate;
}

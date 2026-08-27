using System.Runtime.InteropServices;

namespace Gfold;

/// <summary>
/// P/Invoke surface for the vendored scs.dll (scvx/scs, built by scvx/build-scs.ps1).
///
/// Deliberately a SEPARATE copy of Scvx.Core's binding of the same name rather than a
/// shared project. Gfold.Core has no dependency on Scvx.Core and should not acquire
/// one for two hundred lines of struct layout — the 6-DOF solver is a large piece of
/// machinery that happens to use the same native library, not a library this one sits
/// on. If a third caller ever appears, extract then.
///
/// Built with DLONG and SFLOAT both undefined, so scs_int is int32 and scs_float is
/// double and these layouts need no width translation. Field order mirrors
/// scvx/scs/include/scs.h exactly; C# sequential layout matches a C struct as long as
/// no manual padding is needed, and none is here — every field is a pointer or a
/// machine-word-or-smaller scalar with natural alignment.
///
/// Note the DLONG trap documented in build-scs.ps1: scs_types.h tests `#ifdef DLONG`,
/// so building with -DDLONG=0 still DEFINES it and silently switches scs_int to 64-bit,
/// which corrupts every struct here without erroring.
/// </summary>
internal static partial class ScsNative
{
    private const string Lib = "scs";


    [StructLayout(LayoutKind.Sequential)]
    internal struct ScsMatrix
    {
        public IntPtr X;   // scs_float*, values
        public IntPtr I;   // scs_int*, row indices
        public IntPtr P;   // scs_int*, column pointers, length n+1
        public int M;
        public int N;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ScsData
    {
        public int M;
        public int N;
        public IntPtr A;   // ScsMatrix*
        public IntPtr P;   // ScsMatrix*, or Zero for a linear objective
        public IntPtr B;   // scs_float*, length m
        public IntPtr C;   // scs_float*, length n
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ScsCone
    {
        public int Z;          // zero cone (equalities), FIRST in row order
        public int L;          // positive orthant
        public IntPtr Bu;      // box cone upper, unused (Zero)
        public IntPtr Bl;      // box cone lower, unused (Zero)
        public int Bsize;
        public IntPtr Q;       // scs_int*, SOC dimensions
        public int Qsize;
        public IntPtr S;       // SDP, unused (Zero)
        public int Ssize;
        public IntPtr Cs;      // complex SDP, unused (Zero)
        public int Cssize;
        public int Ep;         // primal exponential cone count
        public int Ed;         // dual exponential cone count
        public IntPtr P_;      // power cone params, unused (Zero) — field named P in scs.h
        public int Psize;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ScsSettings
    {
        public int Normalize;
        public double Scale;
        public int AdaptiveScale;
        public double RhoX;
        public int MaxIters;
        public double EpsAbs;
        public double EpsRel;
        public double EpsInfeas;
        public double Alpha;
        public double TimeLimitSecs;
        public int Verbose;
        public int WarmStart;
        public int AccelerationLookback;
        public int AccelerationInterval;
        public IntPtr WriteDataFilename;   // const char*, NULL
        public IntPtr LogCsvFilename;      // const char*, NULL
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ScsSolution
    {
        public IntPtr X;   // scs_float*, length n — primal
        public IntPtr Y;   // scs_float*, length m — dual
        public IntPtr S;   // scs_float*, length m — slack
    }

    // ScsInfo carries two fixed 128-byte char buffers (status, lin_sys_solver)
    // followed by scs_int/scs_float fields. Marshal the strings by size rather than by
    // pointer: scs_solve fills them in place inside our struct.
    [StructLayout(LayoutKind.Sequential)]
    internal struct ScsInfo
    {
        public int Iter;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 128)]
        public byte[] Status;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 128)]
        public byte[] LinSysSolver;
        public int StatusVal;
        public int ScaleUpdates;
        public double Pobj;
        public double Dobj;
        public double ResPri;
        public double ResDual;
        public double Gap;
        public double ResInfeas;
        public double ResUnbddA;
        public double ResUnbddP;
        public double SetupTime;
        public double SolveTime;
        public double Scale;
        public double CompSlack;
        public int RejectedAccelSteps;
        public int AcceptedAccelSteps;
        public double LinSysTime;
        public double ConeTime;
        public double AccelTime;
    }

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern IntPtr scs_init(ref ScsData d, ref ScsCone k, ref ScsSettings stgs);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int scs_solve(IntPtr work, ref ScsSolution sol, ref ScsInfo info, int warmStart);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void scs_finish(IntPtr work);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void scs_set_default_settings(ref ScsSettings stgs);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern IntPtr scs_version();

    internal static string Version() => Marshal.PtrToStringAnsi(scs_version()) ?? "?";
}

/// <summary>SCS's own exit codes, as returned by scs_solve.</summary>
public enum ScsExit
{
    InfeasibleInaccurate = -7,
    UnboundedInaccurate = -6,
    Interrupted = -5,
    Failed = -4,
    Indeterminate = -3,
    Infeasible = -2,
    Unbounded = -1,
    Unfinished = 0,
    Solved = 1,
    SolvedInaccurate = 2,
}

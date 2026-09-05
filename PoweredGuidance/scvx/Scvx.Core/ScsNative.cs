using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Scvx;

/// <summary>
/// P/Invoke surface for the vendored scs.dll (scvx/scs, built by
/// scvx/build-scs.ps1). Built with DLONG undefined and SFLOAT undefined, so
/// scs_int is int32 and scs_float is double — matches ScsNative's layout
/// one-to-one with no width translation needed.
///
/// Struct layouts below mirror scvx/scs/include/scs.h field order and types
/// exactly; C# struct layout is sequential by default, which matches a C
/// struct's field order as long as no manual padding is required (none of
/// these need it — every field here is either a pointer or a scs_int/scs_float,
/// both machine-word-sized or smaller with natural alignment).
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
        public IntPtr P;   // ScsMatrix*, or Zero if P = 0
        public IntPtr B;   // scs_float*, length m
        public IntPtr C;   // scs_float*, length n
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ScsCone
    {
        public int Z;          // zero cone (equalities)
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
    // followed by scs_int/scs_float fields. Marshal the strings by size rather
    // than by pointer since scs_solve fills them in place inside our struct.
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
    internal static extern int scs_update(IntPtr work, IntPtr b, IntPtr c);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int scs_solve(IntPtr work, ref ScsSolution sol, ref ScsInfo info, int warmStart);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void scs_finish(IntPtr work);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void scs_set_default_settings(ref ScsSettings stgs);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern IntPtr scs_version();

    internal static string Version() => Marshal.PtrToStringAnsi(scs_version()) ?? "?";

    /// <summary>
    /// Dumps .NET's actual computed size/offsets for the marshaled structs, to
    /// check against the C struct's real layout by hand. A P/Invoke struct with
    /// the right field ORDER can still have the wrong OFFSETS if an alignment
    /// assumption is wrong somewhere — this makes that checkable instead of
    /// argued about.
    /// </summary>
    internal static string DumpLayouts()
    {
        var sb = new System.Text.StringBuilder();
        void Dump<T>(string name) where T : struct
        {
            sb.AppendLine($"{name}: size={Marshal.SizeOf<T>()}");
            foreach (var f in typeof(T).GetFields())
                sb.AppendLine($"  {f.Name,-24} offset={Marshal.OffsetOf<T>(f.Name)}");
        }
        Dump<ScsMatrix>("ScsMatrix");
        Dump<ScsData>("ScsData");
        Dump<ScsCone>("ScsCone");
        Dump<ScsSettings>("ScsSettings");
        return sb.ToString();
    }
}

public enum ScsStatus
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

public static class ScsStatusEx
{
    public static bool IsUsable(this ScsStatus s) =>
        s is ScsStatus.Solved or ScsStatus.SolvedInaccurate;
}

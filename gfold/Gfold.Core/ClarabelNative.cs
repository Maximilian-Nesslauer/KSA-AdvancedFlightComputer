using System.Runtime.InteropServices;

namespace Gfold;

/// <summary>
/// P/Invoke surface for clarabel_c.dll (gfold/clarabel, built by gfold/build-clarabel.ps1).
///
/// Clarabel is an INTERIOR-POINT conic solver under Apache-2.0 — the same algorithm
/// class as ECOS, with a licence that can be redistributed under MIT. That pairing is
/// the entire reason it is here: SCS is permissively licensed but first-order, and the
/// measurements in ScsSolver.DefaultEps show what that costs on a problem this shape.
///
/// LAYOUTS ARE TAKEN FROM THE VENDORED HEADERS, NOT FROM DOCUMENTATION. Every struct
/// below mirrors gfold/clarabel/include/c/*.h, and those in turn match the Rust side's
/// #[repr(C)] definitions in Clarabel.rs/src/solver/implementations/default/ffi/ —
/// which is the actual ABI. Two traps are specific to this binding:
///
///   * INDICES ARE uintptr_t, i.e. 64-BIT. ECOS and SCS both use 32-bit ints for CSC
///     column pointers and row indices; Clarabel does not. SparseCcs.Build() hands back
///     int[], so ClarabelSolver widens them. Passing the int arrays straight through
///     would be read as garbage at twice the stride — the same class of silent
///     corruption as the DLONG trap documented in scvx/build-scs.ps1.
///
///   * bool IS ONE BYTE. Rust's #[repr(C)] bool and C's stdbool are both a single byte,
///     while .NET marshals bool as a 4-byte BOOL by default. Every bool here therefore
///     carries [MarshalAs(UnmanagedType.U1)]; without it, every field after the first
///     bool in ClarabelDefaultSettings is read from the wrong offset.
///
/// Three entry points return a struct BY VALUE (settings default, solution, info). On
/// x64 that uses the hidden-return-pointer convention, which .NET's marshaller handles
/// for blittable structs — but it is another reason the layouts have to be exactly
/// right. <see cref="DumpLayouts"/> prints what .NET actually computed, so a mismatch
/// can be checked against the header rather than argued about.
/// </summary>
internal static partial class ClarabelNative
{
    private const string Lib = "clarabel_c";

    /// <summary>
    /// CSC matrix, mirroring ClarabelCscMatrix_f64. Note m/n and both index arrays are
    /// uintptr_t — see the class remarks.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    internal struct ClarabelCscMatrix
    {
        public nuint M;          // rows
        public nuint N;          // columns
        public IntPtr ColPtr;    // const uintptr_t*, length n+1
        public IntPtr RowVal;    // const uintptr_t*, length colptr[n]
        public IntPtr NzVal;     // const double*,    length colptr[n]
    }

    /// <summary>Cone tags, from ClarabelSupportedConeT_Tag.</summary>
    internal enum ConeTag
    {
        Zero = 0,
        Nonnegative = 1,
        SecondOrder = 2,
        Exponential = 3,
        Power = 4,
        GenPower = 5,
    }

    /// <summary>
    /// One cone in the constraint stack, mirroring ClarabelSupportedConeT_f64: a tag
    /// followed by an anonymous union.
    ///
    /// The union's largest member is the generalised-power case — a pointer plus two
    /// uintptr_t, 24 bytes — so the whole struct is 32 bytes on x64 (4-byte tag, 4
    /// bytes of padding to reach 8-byte alignment, then 24). Only the first member is
    /// ever used here: zero, nonnegative and second-order cones each carry a single
    /// dimension. The two trailing fields exist ONLY to reserve the union's full width;
    /// declaring the struct as tag + one dimension would be 16 bytes and every cone
    /// after the first would be read from the wrong offset.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    internal struct ClarabelSupportedCone
    {
        public ConeTag Tag;
        public nuint Dim;        // zero_cone_t / nonnegative_cone_t / second_order_cone_t
        private readonly nuint _unionPad1;
        private readonly nuint _unionPad2;

        internal static ClarabelSupportedCone Of(ConeTag tag, int dim) =>
            new() { Tag = tag, Dim = (nuint)dim };
    }

    /// <summary>
    /// Mirrors ClarabelDefaultSolution_f64. The x/z/s pointers are owned by the solver
    /// and are only valid until it is freed, so the caller copies out before that.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    internal struct ClarabelDefaultSolution
    {
        public IntPtr X;
        public nuint XLength;
        public IntPtr Z;
        public nuint ZLength;
        public IntPtr S;
        public nuint SLength;
        public ClarabelExit Status;
        public double ObjVal;
        public double ObjValDual;
        public double SolveTime;
        public uint Iterations;
        public double RPrim;
        public double RDual;
    }

    /// <summary>
    /// Direct linear solver choice, from DirectSolveMethodsFFI in Clarabel.rs. AUTO is
    /// the default and is 0; QDLDL is 1. (The values matter only for reading the
    /// setting back — nothing here sets it.)
    /// </summary>
    internal enum DirectSolveMethod
    {
        Auto = 0,
        Qdldl = 1,
    }

    /// <summary>
    /// Mirrors ClarabelDefaultSettings_f64, field for field and in order, as declared in
    /// include/c/DefaultSettings.h and Clarabel.rs's DefaultSettingsFFI.
    ///
    /// Never construct one of these from scratch — call
    /// <see cref="clarabel_DefaultSettings_f64_default"/> and modify what you need. The
    /// regularisation, equilibration and iterative-refinement blocks all have tuned
    /// defaults, and a zeroed struct disables them silently rather than erroring.
    ///
    /// The SDP and Pardiso tails in the header are behind FEATURE_SDP and
    /// FEATURE_PARDISO_ANY, neither of which the vendored build enables, so they are
    /// absent here too. If build-clarabel.ps1 ever passes --features sdp, this struct
    /// grows four fields and MUST be updated with it.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    internal struct ClarabelDefaultSettings
    {
        public uint MaxIter;
        public double TimeLimit;
        [MarshalAs(UnmanagedType.U1)] public bool Verbose;
        public double MaxStepFraction;

        public double TolGapAbs;
        public double TolGapRel;
        public double TolFeas;
        public double TolInfeasAbs;
        public double TolInfeasRel;
        public double TolKtratio;

        public double ReducedTolGapAbs;
        public double ReducedTolGapRel;
        public double ReducedTolFeas;
        public double ReducedTolInfeasAbs;
        public double ReducedTolInfeasRel;
        public double ReducedTolKtratio;

        [MarshalAs(UnmanagedType.U1)] public bool EquilibrateEnable;
        public uint EquilibrateMaxIter;
        public double EquilibrateMinScaling;
        public double EquilibrateMaxScaling;

        public double LinesearchBacktrackStep;
        public double MinSwitchStepLength;
        public double MinTerminateStepLength;

        public uint MaxThreads;
        [MarshalAs(UnmanagedType.U1)] public bool DirectKktSolver;
        public DirectSolveMethod DirectSolveMethod;

        [MarshalAs(UnmanagedType.U1)] public bool StaticRegularizationEnable;
        public double StaticRegularizationConstant;
        public double StaticRegularizationProportional;

        [MarshalAs(UnmanagedType.U1)] public bool DynamicRegularizationEnable;
        public double DynamicRegularizationEps;
        public double DynamicRegularizationDelta;

        [MarshalAs(UnmanagedType.U1)] public bool IterativeRefinementEnable;
        public double IterativeRefinementReltol;
        public double IterativeRefinementAbstol;
        public uint IterativeRefinementMaxIter;
        public double IterativeRefinementStopRatio;

        [MarshalAs(UnmanagedType.U1)] public bool PresolveEnable;
    }

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern ClarabelDefaultSettings clarabel_DefaultSettings_f64_default();

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern IntPtr clarabel_DefaultSolver_f64_new(
        ref ClarabelCscMatrix p, IntPtr q,
        ref ClarabelCscMatrix a, IntPtr b,
        nuint nCones, IntPtr cones,
        ref ClarabelDefaultSettings settings);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void clarabel_DefaultSolver_f64_solve(IntPtr solver);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void clarabel_DefaultSolver_f64_free(IntPtr solver);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern ClarabelDefaultSolution clarabel_DefaultSolver_f64_solution(IntPtr solver);

    /// <summary>
    /// What .NET actually computed for these layouts, to be checked by hand against
    /// gfold/clarabel/include/c/*.h. Same idea as Scvx.Core's ScsNative.DumpLayouts:
    /// a struct with the right field ORDER can still have wrong OFFSETS, and this is
    /// the difference between checking that and arguing about it.
    ///
    /// VERIFIED on x64 against the C headers by hand: CscMatrix 40, SupportedCone 32,
    /// Solution 104, Settings 280. The load-bearing offsets, all confirmed:
    ///
    ///   SupportedCone.Dim            8    tag is 4 bytes + 4 padding, not 8
    ///   Solution.Status             48    after six 8-byte fields
    ///   Solution.Iterations         80    u32 between two doubles
    ///   Settings.TimeLimit           8    f64 after a u32, so 4 bytes of padding
    ///   Settings.EquilibrateMaxIter 132   THE bool check: 128 is a one-byte bool, so
    ///                                     the u32 lands at 132. Drop the U1 attribute
    ///                                     and .NET makes that bool 4 bytes, pushing
    ///                                     this to 136 and every later field with it.
    ///   Settings.PresolveEnable     272   last field; struct pads to 280
    ///
    /// This ran before clarabel_c.dll existed — Marshal.SizeOf is reflection over the
    /// managed declaration and loads nothing native — which is the only reason the
    /// layouts could be checked at all while the binding was still unrunnable.</summary>
    internal static string DumpLayouts()
    {
        var sb = new System.Text.StringBuilder();
        void Dump<T>(string name) where T : struct
        {
            sb.AppendLine($"{name}: size={Marshal.SizeOf<T>()}");
            foreach (var f in typeof(T).GetFields())
                sb.AppendLine($"  {f.Name,-34} offset={Marshal.OffsetOf<T>(f.Name)}");
        }
        Dump<ClarabelCscMatrix>("ClarabelCscMatrix");
        Dump<ClarabelSupportedCone>("ClarabelSupportedCone");
        Dump<ClarabelDefaultSolution>("ClarabelDefaultSolution");
        Dump<ClarabelDefaultSettings>("ClarabelDefaultSettings");
        return sb.ToString();
    }
}

/// <summary>
/// Clarabel's own solver status, as returned in ClarabelDefaultSolution.status.
/// Public for the same reason ScsExit is: the A/B harness reports the backend's native
/// outcome, not just the normalised one, because "almost solved" and "max time" are
/// different diagnoses that both map to the same shared status.
/// </summary>
public enum ClarabelExit
{
    Unsolved = 0,
    Solved = 1,
    PrimalInfeasible = 2,
    DualInfeasible = 3,
    AlmostSolved = 4,
    AlmostPrimalInfeasible = 5,
    AlmostDualInfeasible = 6,
    MaxIterations = 7,
    MaxTime = 8,
    NumericalError = 9,
    InsufficientProgress = 10,
    CallbackTerminated = 11,
}

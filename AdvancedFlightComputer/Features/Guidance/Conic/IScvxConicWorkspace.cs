namespace AdvancedFlightComputer.Guidance.Conic;

/// <summary>
/// The conic backend behind the 6-DOF and 3-DOF SCvx subproblems: SCS (ADMM) or Clarabel (interior point).
///
/// Both take the problem in the same form, minimize (1/2) x'Px + c'x subject to Ax + s = b, s in K, with K the
/// zero cone, then the orthant, then second-order cones, rows of A and b in that order and P upper triangular.
/// So a subproblem assembles one problem and hands it to either, unchanged. The surface is ScsWorkspace's, so
/// the SCvx loops above it keep their control flow: ScsStatus results, HitIterationLimit for a truncated solve
/// (never an SCvx step), HasWarmStart for whether the last solve left a usable point.
/// </summary>
public interface IScvxConicWorkspace
{
    double[] X { get; }
    double PrimalObjective { get; }
    int Iterations { get; }
    string StatusText { get; }
    bool HitIterationLimit { get; }
    bool HasWarmStart { get; }
    void ResetWarmStart();

    /// <summary>
    /// The iteration cap a request turns into. SCS takes the requested ADMM budget as it is. Clarabel's
    /// iterations are interior-point steps, ten to forty to converge, so an ADMM budget of thousands means
    /// nothing to it and it caps at its own ceiling. The SCvx loops compare these to decide whether an
    /// escalated re-run can do anything a first run could not.
    /// </summary>
    int EffectiveIterationCap(int requested);

    ScsStatus Solve(CcsAssembler A, double[] b, double[] c, CcsAssembler? P,
                    int zeroDim, int posDim, int[] socDims,
                    bool warmStart, bool verbose = false,
                    int maxIterations = ScsWorkspace.DefaultMaxIterations,
                    double epsAbs = ScsWorkspace.DefaultEps, double epsRel = ScsWorkspace.DefaultEps);
}

/// <summary>Which conic backend the SCvx subproblems solve with.</summary>
public enum ScvxConicBackend
{
    Scs,
    Clarabel,
}

/// <summary>
/// The process-wide backend choice for SCvx subproblems, read at every subproblem solve.
///
/// One switch for every craft, because it is an A/B switch for testing the solver, not a craft setting: the
/// focused craft's setting writes it each frame (GuidanceWindow.ApplyAutopilot), and a booster guiding beside
/// it follows. A change takes effect at the next subproblem solve; the backend switched to starts with no
/// carried state (SCS: no warm-start iterate; Clarabel: a fresh factorisation).
/// </summary>
public static class ScvxConic
{
    private static volatile ScvxConicBackend _backend = ScvxConicBackend.Clarabel;

    public static ScvxConicBackend Backend
    {
        get => _backend;
        set => _backend = value;
    }
}

/// <summary>
/// Holds one workspace per backend and hands back the one <see cref="ScvxConic.Backend"/> selects, so a
/// subproblem keeps its warm-start iterate or factorisation per backend across a switch and back.
/// </summary>
public sealed class ScvxConicWorkspaces
{
    private ScsWorkspace? _scs;
    private ClarabelWorkspace? _clarabel;

    /// <summary>The workspace of the last solve, for reading its solution; the selected one before any solve.</summary>
    public IScvxConicWorkspace Last { get; private set; }

    public ScvxConicWorkspaces()
    {
        Last = Current;
    }

    public IScvxConicWorkspace Current => ScvxConic.Backend == ScvxConicBackend.Clarabel
        ? _clarabel ??= new ClarabelWorkspace()
        : _scs ??= new ScsWorkspace();

    /// <summary>The selected workspace, which becomes <see cref="Last"/>.</summary>
    public IScvxConicWorkspace ForSolve() => Last = Current;

    public void ResetWarmStart()
    {
        _scs?.ResetWarmStart();
        _clarabel?.ResetWarmStart();
    }
}

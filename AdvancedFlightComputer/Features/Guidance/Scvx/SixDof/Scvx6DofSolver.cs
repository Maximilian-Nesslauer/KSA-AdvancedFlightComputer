using AdvancedFlightComputer.Guidance.Conic;

namespace AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>Outcome of one SCvx iteration, for tracing and for the caller's loop control.</summary>
public readonly record struct ScvxIteration(
    int Index,
    bool Solved,          // did the convex subproblem solve at all
    bool Accepted,        // did the ratio test accept the step
    double Rho,           // actual / predicted cost reduction
    double TrustRegion,   // radius AFTER this iteration's update
    double Sigma,
    double Step,          // max(dX, dSigma), normalised - the convergence measure
    double DefectNorm,    // max |true nonlinear defect| / Xscale
    double Cost,          // true merit at the reference after this iteration
    int SolverIterations,
    double ElapsedMs,
    // WHERE that worst defect is.
    // The magnitude alone cannot be read: DefectNorm is a max over all 14 state channels, each divided by its OWN scale, and the position scale is metres while the quaternion and body-rate scales are 1.
    // Callers that multiply by the length scale to report "metres" are therefore only reporting a distance when the worst channel happens to be a position - otherwise the units are a rate error times a length. -1 when no defect was evaluated.
    int DefectChannel,    // state index 0..13, or -1
    int DefectNode);      // interval k (between node k and k+1), or -1

public enum ScvxStatus
{
    /// <summary>Step and defect both under tolerance - a genuine fixed point.</summary>
    Converged,
    /// <summary>Ran out of the caller's iteration budget. The reference is still usable.</summary>
    IterationLimit,
    /// <summary>Trust region collapsed to TR_MIN: the subproblem keeps failing or keeps being rejected.</summary>
    TrustRegionCollapsed,
    /// <summary>No usable reference at all (the very first subproblem never solved).</summary>
    Failed,
}

/// <summary>
/// The SCvx outer loop: repeatedly linearise the nonlinear dynamics about a
/// reference trajectory, solve the resulting convex subproblem inside a trust
/// region, and accept or reject the step on a merit-function ratio test.
///
/// Port of the loop in 6dof.py. The pieces that matter and are easy to get
/// subtly wrong:
///
/// - The MERIT function is evaluated with the TRUE nonlinear dynamics
///   (<see cref="TrueCost"/>), not the linearised ones. Its defect term is the
///   real integration residual, which is what makes the ratio test meaningful:
///   the subproblem's own virtual control Wv only measures how much the
///   LINEARISED dynamics had to be violated.
/// - PREDICTED reduction uses the subproblem's cost with Wv as the defect;
///   ACTUAL reduction uses the true nonlinear defect. rho = actual / predicted.
///   Both must include the same constant terms or the ratio is meaningless -
///   hence both go through the same fuel/smoothing helpers here.
/// - On accept the quaternion is REPROJECTED to unit norm. The subproblem only
///   enforces the tangent-plane linearisation qbar.q = 1, which is exact only at
///   the fixpoint; without reprojection the reference drifts off the sphere and
///   the linearisation degrades.
///
/// Designed for receding-horizon use as well as batch: <see cref="Iterate"/> runs
/// exactly one iteration so a guidance loop can spend a fixed budget per cycle
/// and carry the reference forward, rather than blocking until convergence.
/// </summary>
public sealed class Scvx6DofSolver
{
    private const int NX = Dynamics6Dof.NX;
    /// <summary>The control count, the base controls and the craft's fins (Scvx6DofConfig.NU).</summary>
    private readonly int NU;

    private readonly Scvx6DofConfig _cfg;
    private readonly Dynamics6Dof.Params _dyn;
    private readonly Scvx6DofSubproblemScs _sub;
    private readonly int _n;
    private readonly double _dtau;

    // The coast before ignition (Scvx6DofConfig.CoastIntervals): its interval count, the dynamics with the engine off, each phase's normalised step, and the ignition node's engine-off linearisation, which ends the last coast interval.
    private readonly int _nCoast;
    private readonly Dynamics6Dof.Params _dynCoast;
    private readonly double _dtauC, _dtauB;
    private readonly double[] _aJ = new double[NX * NX], _bJ, _f0J = new double[NX];
    private double _sigCBar;
    private readonly double[] _xs, _us;

    /// <summary>The scale the defect and the virtual control are PRICED on in the merit; the convergence test reads them on _xs. See Scvx6DofConfig.VirtualControlScale and SetVirtualControlScale.</summary>
    private readonly double[] _vcs;
    private bool _initialised;

    // Trust-region schedule. Names match 6dof.py's constants.
    public double TrustRegion { get; private set; } = 0.1;
    public double TrustRegionMin { get; init; } = 1e-3;
    public double TrustRegionMax { get; init; } = 0.1;
    public double RhoAccept { get; init; } = 0.0;    // RHO0: accept above this
    public double RhoShrink { get; init; } = 0.25;   // RHO1: shrink below this
    public double RhoGrow { get; init; } = 0.7;      // RHO2: grow above this (if the step used the radius)
    public double Shrink { get; init; } = 0.5;
    public double Grow { get; init; } = 1.5;
    public double StepTolerance { get; init; } = 8e-3;

    /// <summary>
    /// Convex-subproblem tolerance. Separate from the SCvx tolerances because
    /// the two answer different questions: this is how precisely each step is
    /// solved, StepTolerance/DefectTolerance are when the OUTER loop has
    /// converged. First-order methods have slow tail convergence, so demanding
    /// too little error here costs iterations superlinearly.
    /// </summary>
    /// <summary>
    /// Tolerance each convex subproblem is solved to - distinct from the OUTER
    /// SCvx convergence test. Defaults to the conservative offline value; flight
    /// code should pass <see cref="RealTimeEps"/>.
    /// </summary>
    public double SubproblemEps { get; set; } = ScsWorkspace.DefaultEps;

    /// <summary>
    /// The tolerance to use in flight. Measured at N=30: 1e-5, 1e-6 and 1e-7 all
    /// converge in the SAME 17 iterations to the SAME answer to five significant
    /// figures (merit 9.7619e-2, defect 9.5e-6, peak tilt 6.1 deg, burn 24.2 s),
    /// but cost 1.7 s / 3.1 s / 5.5 s cold and 33 ms / 334 ms / 1266 ms per
    /// receding-horizon cycle. ADMM's slow tail is almost the entire bill here and
    /// buys nothing, so the tighter defaults are for offline validation only.
    ///
    /// This is safe ONLY because <see cref="ScsWorkspace.HitIterationLimit"/>
    /// rejects truncated solves outright - it is that guard, not a tight tolerance,
    /// that keeps a bad subproblem out of the ratio test.
    /// </summary>
    /// <summary>
    /// Subproblem tolerance for FLIGHT. Not SCS's default and not the validation
    /// tolerance (<see cref="ScsWorkspace.DefaultEps"/>, 1e-7) - offline solves keep
    /// that.
    ///
    /// Measured in closed loop at N=80 with dispersion, eps 1e-5 / 1e-4 / 1e-3 all
    /// produce the SAME trajectory (miss 4.1-4.2 m, path 1.25-1.26 x direct, plan
    /// jump 1.0 m) but cost wildly different amounts of time:
    ///
    ///     eps     p50     p90     p99     max     ADMM/solve
    ///     1e-5   36.5   251.1   390.7     964            491
    ///     1e-4   12.7    25.6   828.0    1909            264
    ///     1e-3    9.0    12.7   807.7    1975            169
    ///
    /// 1e-4 is the pick. The p50/p90 collapse is the whole story: at 1e-5 roughly
    /// HALF of all solves overran the 40 ms subproblem budget and got truncated, at
    /// 1e-4 under a tenth do. The p99/max going UP is real but does not reach the
    /// vehicle - those solves hit the wall-clock budget and are cut off either way.
    /// 1e-3 buys little more and leaves less margin on the iterate SCvx's ratio test
    /// depends on.
    /// </summary>
    public const double RealTimeEps = 1e-4;

    /// <summary>
    /// The flight tolerance for a plan with RCS torque. 1e-3 was turned down above because, with the gimbals alone, it bought little over 1e-4. With the RCS in the plan it buys a great deal: the warm re-solves of a 103 m/s hand-over took SCS 825 iterations at 1e-3 against 7,800 at 3e-4, and with the RCS controls on the axis's whole torque (Scvx6DofConfig.ResolvedUScale) 100-775 against 1,700-3,400 at 1e-4, landing within 1-2 m either way (offline --6dof-rcs, 2026-11-20). The nonlinear defect gate still judges every plan.
    /// </summary>
    public const double RealTimeEpsWithRcs = 1e-3;

    /// <summary>Node counts up to this keep <see cref="RealTimeEps"/> even with RCS in the plan.</summary>
    public const int TightEpsMaxNodes = 10;

    /// <summary>
    /// The flight tolerance for these dynamics at <paramref name="nodes"/> nodes: <see cref="RealTimeEpsWithRcs"/> with RCS torque in the plan above <see cref="TightEpsMaxNodes"/> nodes, <see cref="RealTimeEps"/> otherwise.
    ///  The last rungs keep the tight tolerance: at 1e-3 a 5-node plan could not get its attitude under the gate at the last interval (1.4-2.1x, every iteration accepted), every re-solve near the ground was refused, and the booster flew a stale plan to a stop 1.6 m short (2026-11-20). At 5-10 nodes an SCS iteration costs 13-30 microseconds, so the extra iterations cost milliseconds.
    /// </summary>
    public static double RealTimeEpsFor(Dynamics6Dof.Params dyn, int nodes)
        => dyn.Rcs != null && nodes > TightEpsMaxNodes ? RealTimeEpsWithRcs : RealTimeEps;

    /// <summary>
    /// Hard cap on ADMM iterations per subproblem. THE REAL-TIME PATH MUST HAVE A
    /// BOUNDED WORST CASE - a mean is not a guarantee.
    ///
    /// ScsWorkspace.DefaultMaxIterations is 100,000, which is an OFFLINE VALIDATION
    /// number: it exists so a hard problem is solved properly rather than silently
    /// truncated. On a game thread it is a licence to stall for seconds. Measured in
    /// closed loop, an UNCAPPED worst-case subproblem ran 31,800 ADMM iterations /
    /// 1.7 s while the mean was ~400 - and that is on a synthetic problem. In game
    /// that presents as a freeze, and then the process is killed.
    ///
    /// Capping is SAFE BY CONSTRUCTION here, and that is not a lucky accident: SCS
    /// reports a truncated solve as SolvedInaccurate, ScsWorkspace.HitIterationLimit
    /// detects it, and the SCvx loop already treats it as a subproblem FAILURE and
    /// shrinks the trust region rather than accepting the iterate. So a cap converts
    /// "stall for two seconds" into "fail fast and take a smaller step".
    /// </summary>
    /// SETTABLE, because the cold solve and the receding-horizon re-solve need
    /// different budgets: measured, a cap of 1000 FAILS the cold solve outright while
    /// 2000 handles every warm re-solve with room to spare. The cold solve happens
    /// during a coast where a two-second stall is acceptable; a re-solve happens in
    /// the loop where it is not.
    public int MaxSubproblemIterations { get; set; } = ScsWorkspace.DefaultMaxIterations;

    /// <summary>
    /// Budget for ONE retry when a subproblem merely ran out of ADMM iterations.
    ///
    /// Hitting the iteration cap and being INFEASIBLE are different failures and
    /// used to share a response. Infeasible means the linearisation is being asked
    /// to do too much, and shrinking the trust region is right. Truncation means the
    /// SOLVER ran out of budget - shrinking does nothing for that and actively makes
    /// it WORSE, because a tighter trust region leaves more constraints active and a
    /// more degenerate problem, which ADMM finds harder. Measured in flight: cap,
    /// shrink, harder, cap... seven iterations, ZERO accepted, 2026 ms burnt, and
    /// the plan refused for an infinite defect because no step was ever taken.
    ///
    /// The matrices are already assembled at that point, so the retry costs only the
    /// extra ADMM iterations - no re-linearisation and no re-assembly.
    /// </summary>
    public int EscalatedSubproblemIterations { get; set; } = ScsWorkspace.DefaultMaxIterations;

    /// <summary>Subproblems that needed the escalated budget. Occasional is fine; frequent means the base cap is too low.</summary>
    public int Escalations { get; private set; }

    /// <summary>
    /// Structural explanation for an Unbounded subproblem: which variable the
    /// objective rewards moving with nothing to stop it. See
    /// Scvx6DofSubproblemScs.DiagnoseUnbounded.
    /// </summary>
    public string DiagnoseUnbounded() => _sub.DiagnoseUnbounded();

    /// <summary>
    /// Feed each subproblem the previous solve's ADMM iterate. The problem
    /// changes between SCvx iterations, but only within the trust region, so the
    /// last point is a good start. This is the ONLY warm start SCS offers -
    /// scs_update refreshes just b and c, never A or P, and our Jacobian block
    /// lives in A, so the workspace itself must be rebuilt every iteration.
    /// </summary>
    public bool WarmStart { get; init; } = true;
    public double DefectTolerance { get; init; } = 1e-3;

    // Reference trajectory - the SCvx state that persists across iterations and, in receding-horizon use, across guidance cycles.
    private double[] _xbar = [], _ubar = [];
    private double _sigBar;
    private double _jRef;
    private double[] _x0 = [], _xf = [];

    // Linearisation buffers, allocated once.
    private readonly double[] _a, _b, _f0;

    private readonly List<ScvxIteration> _trace = [];
    public IReadOnlyList<ScvxIteration> Trace => _trace;

    public double[] ReferenceX => _xbar;
    public double[] ReferenceU => _ubar;
    public double Sigma => _sigBar;

    /// <summary>The coast's duration, s: the time to ignition. 0 without a coast.</summary>
    public double CoastSigma => _sigCBar;

    /// <summary>Intervals in the coast (Scvx6DofConfig.CoastIntervals).</summary>
    public int CoastIntervals => _nCoast;

    /// <summary>The dynamics interval k is flown with: the engine off in the coast.</summary>
    private Dynamics6Dof.Params ParamsOf(int k) => k < _nCoast ? _dynCoast : _dyn;

    /// <summary>Interval k's duration at these phase durations, s.</summary>
    private double IntervalDt(int k, double sigma, double coastSigma) => k < _nCoast ? _dtauC * coastSigma : _dtauB * sigma;

    /// <summary>The time of node k from node 0, s, at the reference's durations: the coast's nodes spread over its duration, the burn's over the burn's.</summary>
    public double NodeTime(int k) => NodeTime(k, _n, _nCoast, _sigBar, _sigCBar);

    /// <summary>The time of node k of an n-node plan with a coast of coastIntervals over coastSigma s and a burn of sigma s.</summary>
    public static double NodeTime(int k, int n, int coastIntervals, double sigma, double coastSigma)
        => k <= coastIntervals
            ? (coastIntervals > 0 ? coastSigma * k / coastIntervals : 0.0)
            : coastSigma + sigma * (k - coastIntervals) / (n - 1 - coastIntervals);
    public double Cost => _jRef;
    public int IterationCount { get; private set; }

    /// <summary>Why the most recent subproblem failed - SCS status plus its own text.</summary>
    public string LastFailureReason { get; private set; } = "";

    public Scvx6DofSolver(Scvx6DofConfig cfg, Dynamics6Dof.Params? dyn = null)
    {
        _cfg = cfg;
        NU = cfg.NU;
        _bJ = new double[NX * NU];
        _dyn = dyn ?? new Dynamics6Dof.Params();
        if (_dyn.FinCount != cfg.FinCount)
            throw new ArgumentException($"The plan's fin limits are for {cfg.FinCount} control surfaces; the dynamics have {_dyn.FinCount}.");
        _n = cfg.Nodes;
        _dtau = 1.0 / (_n - 1);
        _nCoast = Math.Max(cfg.CoastIntervals, 0);
        _dynCoast = _dyn.WithEngineOff();
        _dtauC = _nCoast > 0 ? 1.0 / _nCoast : 0.0;
        _dtauB = 1.0 / (_n - 1 - _nCoast);
        _xs = cfg.XScale;
        _vcs = new double[NX];
        for (int i = 0; i < NX; i++) _vcs[i] = cfg.VirtualControlScaleOf(i);
        _us = cfg.ResolvedUScale;
        _sub = new Scvx6DofSubproblemScs(cfg);

        _a = new double[_n * NX * NX];
        _b = new double[_n * NX * NU];
        _f0 = new double[_n * NX];
    }

    /// <summary>
    /// Seed the loop. x0 is the full 14-component initial state; xf the 13
    /// pinned terminal components (mass is free). xSeed/uSeed are the initial
    /// reference trajectory - a straight line is fine cold; in receding-horizon
    /// use, pass the previous cycle's solution shifted forward.
    /// </summary>
    public void Initialize(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf,
                           double[] xSeed, double[] uSeed, double sigmaSeed,
                           double? trustRegion = null, double coastSigmaSeed = 0.0)
    {
        if (xSeed.Length != _n * NX) throw new ArgumentException($"xSeed must be {_n * NX} long");
        if (uSeed.Length != _n * NU) throw new ArgumentException($"uSeed must be {_n * NU} long");

        _x0 = x0.ToArray();
        _xf = xf.ToArray();
        _xbar = (double[])xSeed.Clone();
        _ubar = (double[])uSeed.Clone();
        ClampRcsSeed(_ubar);
        AlignTerminalBranch();
        _sigBar = sigmaSeed;
        _sigCBar = CoastSeed(coastSigmaSeed);
        TrustRegion = trustRegion ?? TrustRegionMax;
        IterationCount = 0;
        _trace.Clear();
        _sub.ResetWarmStart();

        (_jRef, _) = TrueCost(_xbar, _ubar, _sigBar);
        _initialised = true;
    }

    /// <summary>
    /// Change the price on virtual control between iterations: the scale each channel's jump is measured on (Scvx6DofConfig.VirtualControlScale, null for XScale). The subproblem reads it at its next assembly, and the reference's merit is recomputed here under it, so the next ratio test compares like with like - a price changed under a reference whose cost was taken at the old one would read as a gain or a loss the step never made.
    /// </summary>
    public void SetVirtualControlScale(double[]? scale)
    {
        _cfg.VirtualControlScale = scale;
        for (int i = 0; i < NX; i++) _vcs[i] = _cfg.VirtualControlScaleOf(i);
        if (_initialised)
            (_jRef, _) = TrueCost(_xbar, _ubar, _sigBar);
    }

    /// <summary>
    /// Advance to a new initial state, keeping the current reference trajectory
    /// (shifted by the caller) AND the ADMM warm start - the receding-horizon
    /// entry point, as opposed to <see cref="Initialize"/> which starts cold.
    ///
    /// The distinction matters: Initialize throws away the solver's iterate,
    /// which is the whole reason SCS was chosen. Here the previous cycle's
    /// solution is still an excellent starting point, because one control
    /// interval of vehicle motion barely changes the problem.
    /// </summary>
    public void Reseed(ReadOnlySpan<double> x0, double[] xShifted, double[] uShifted,
                       double sigma, double? trustRegion = null, double coastSigma = 0.0)
    {
        _x0 = x0.ToArray();
        _xbar = (double[])xShifted.Clone();
        _ubar = (double[])uShifted.Clone();
        ClampRcsSeed(_ubar);
        NormaliseQuaternions(_xbar);
        AlignTerminalBranch();
        _sigBar = sigma;
        _sigCBar = CoastSeed(coastSigma);
        if (trustRegion.HasValue) TrustRegion = trustRegion.Value;
        _trace.Clear();
        IterationCount = 0;
        (_jRef, _) = TrueCost(_xbar, _ubar, _sigBar);
        _initialised = true;
    }

    /// <summary>
    /// Run iterations until convergence, the budget runs out, or the trust
    /// region collapses. Safe to call repeatedly - it continues from the current
    /// reference, which is what a receding-horizon caller wants.
    /// </summary>
    /// <param name="deadlineMs">
    /// Wall-clock ceiling for the WHOLE call, or 0 for none.
    ///
    /// Bounding the SUBPROBLEM is not the same as bounding the CYCLE, and only the
    /// second one is what a caller on a frame thread cares about. A per-subproblem
    /// budget of 40 ms with maxIterations of 5 authorises 200 ms of work, and with
    /// the escalated retry, a full second. Measured in flight: p50 108 ms, p99 432 ms
    /// per cycle against a 100 ms cadence - the loop was asking for more time than
    /// existed, every cycle, and the sim thread paid for it.
    ///
    /// Checked BETWEEN iterations rather than inside one, because an SCvx iteration
    /// is indivisible: what this bounds is how many more are started, and the answer
    /// is still a complete, self-consistent trajectory - just refined less. Since a
    /// cycle is judged on the defect of its ACCEPTED step, stopping early costs
    /// convergence margin, not validity.
    /// </param>
    public ScvxStatus Solve(int maxIterations, double deadlineMs = 0.0)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        for (int i = 0; i < maxIterations; i++)
        {
            ScvxIteration it = Iterate();
            if (!it.Solved && TrustRegion <= TrustRegionMin * 1.001)
                return _trace.Any(t => t.Accepted) ? ScvxStatus.TrustRegionCollapsed : ScvxStatus.Failed;
            if (it.Accepted && it.Step < StepTolerance && it.DefectNorm < DefectTolerance)
                return ScvxStatus.Converged;
            // At least one iteration always runs: a cycle that returns without taking a step hands back the stale plan, which is worse than being late once.
            if (deadlineMs > 0.0 && sw.Elapsed.TotalMilliseconds >= deadlineMs)
            {
                TimedOut = true;
                return ScvxStatus.IterationLimit;
            }
        }
        return ScvxStatus.IterationLimit;
    }

    /// <summary>True if the last Solve stopped on its wall-clock deadline rather than
    /// on convergence or the iteration count. Diagnostic only.</summary>
    public bool TimedOut { get; private set; }

    /// <summary>
    /// Name of a state channel, for reading a defect report. The units matter as much
    /// as the name: position is metres, velocity m/s, quaternion dimensionless, body
    /// rate rad/s, mass kg - and the defect is that channel's error divided by that
    /// channel's scale, so two equal DefectNorms on different channels mean entirely
    /// different physical things.
    /// </summary>
    public static string ChannelName(int i) => i switch
    {
        0 => "rx", 1 => "ry", 2 => "rz",
        3 => "vx", 4 => "vy", 5 => "vz",
        6 => "qw", 7 => "qx", 8 => "qy", 9 => "qz",
        10 => "wx", 11 => "wy", 12 => "wz",
        13 => "mass",
        _ => "-",
    };

    /// <summary>Which family a channel belongs to: what the defect is ABOUT.</summary>
    public static string ChannelGroup(int i) => i switch
    {
        >= 0 and < 3 => "position",
        >= 3 and < 6 => "velocity",
        >= 6 and < 10 => "attitude",
        >= 10 and < 13 => "rate",
        13 => "mass",
        _ => "none",
    };

    /// <summary>One SCvx iteration: linearise, solve, ratio-test, update.</summary>
    public ScvxIteration Iterate()
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        int index = IterationCount++;

        // 1. Linearise the true dynamics about the current reference.
        for (int k = 0; k < _n; k++)
            Dynamics6Dof.Jacobian(
                _xbar.AsSpan(k * NX, NX), _ubar.AsSpan(k * NU, NU), ParamsOf(k),
                _f0.AsSpan(k * NX, NX),
                _a.AsSpan(k * NX * NX, NX * NX),
                _b.AsSpan(k * NX * NU, NX * NU));
        // The ignition node once more with the engine off, for the far end of the last coast interval.
        if (_nCoast > 0)
            Dynamics6Dof.Jacobian(_xbar.AsSpan(_nCoast * NX, NX), _ubar.AsSpan(_nCoast * NU, NU), _dynCoast, _f0J, _aJ, _bJ);

        // 2. Solve the convex subproblem inside the current trust region. Warm-started from the previous solve's iterate: the problem changes between iterations, but only within the trust region, so the last point is a good ADMM start. ScsWorkspace refuses to carry forward an unusable solution, so a failed solve cannot poison this.
        _sub.Assemble(_x0, _xf, _xbar, _ubar, _sigBar, TrustRegion, _a, _b, _f0, _sigCBar, _aJ, _bJ, _f0J);
        ScsStatus st = _sub.Run(warmStart: WarmStart && _sub.HasWarmStart,
                                maxIterations: MaxSubproblemIterations,
                                epsAbs: SubproblemEps, epsRel: SubproblemEps);

        // A truncated solve counts as a failure, not a step.
        // SCS returns SolvedInaccurate when it runs out of iterations, and that iterate can sit far outside the trust region - accepting it produced a step 100x the radius and a rho of +335 before this guard existed.
        // Truncation is a BUDGET problem, not a trajectory problem: retry the same already-assembled subproblem with more iterations rather than shrinking the trust region, which would only make it harder.
        // Warm-started from the last GOOD iterate, since truncated ones are no longer carried forward.
        if (_sub.HitIterationLimit && _sub.EffectiveIterationCap(EscalatedSubproblemIterations) > _sub.EffectiveIterationCap(MaxSubproblemIterations))
        {
            Escalations++;
            st = _sub.Run(warmStart: WarmStart && _sub.HasWarmStart,
                          maxIterations: EscalatedSubproblemIterations,
                          epsAbs: SubproblemEps, epsRel: SubproblemEps);
        }

        if (!st.IsUsable() || _sub.HitIterationLimit)
        {
            // Subproblem failure is not fatal - shrink and retry from the same reference.
            // A trust region that keeps collapsing is the signal that something is actually wrong.
            TrustRegion = Math.Max(TrustRegionMin, TrustRegion * Shrink);
            LastFailureReason = $"{st} \"{_sub.StatusText}\"" +
                                (_sub.HitIterationLimit ? " [truncated at iteration cap]" : "");
            var failed = new ScvxIteration(index, false, false, double.NaN, TrustRegion,
                _sigBar, 0, double.NaN, _jRef, _sub.Iterations, sw.Elapsed.TotalMilliseconds,
                -1, -1);
            _trace.Add(failed);
            return failed;
        }

        double[] x = _sub.SolutionX, u = _sub.SolutionU, wv = _sub.SolutionWv;
        double sigma = _sub.SolutionSigma;
        double coastSigma = _nCoast > 0 ? _sub.SolutionCoastSigma : 0.0;

        // A solution outside its own time bounds is not a step. SCS's "inaccurate" answers are taken as usable, and one came back with a burn and coast summing to -359 s (2026-11-20 12:36, a coast standby): the next re-anchor threw on it. Refused like a truncated solve, so the trust region shrinks and the reference stays.
        double slack = 1e-3 * _cfg.SigmaScale;
        double coastLo = _cfg.CoastSigmaFixed ?? _cfg.CoastSigmaMin, coastHi = _cfg.CoastSigmaFixed ?? _cfg.CoastSigmaMax;
        if (!double.IsFinite(sigma) || sigma < _cfg.SigmaMin - slack || sigma > _cfg.SigmaMax + slack
            || (_nCoast > 0 && (!double.IsFinite(coastSigma) || coastSigma < coastLo - slack || coastSigma > coastHi + slack)))
        {
            TrustRegion = Math.Max(TrustRegionMin, TrustRegion * Shrink);
            LastFailureReason = $"{st} \"{_sub.StatusText}\" with the burn {sigma:G4} s and the coast {coastSigma:G4} s outside their bounds";
            var outside = new ScvxIteration(index, false, false, double.NaN, TrustRegion,
                _sigBar, 0, double.NaN, _jRef, _sub.Iterations, sw.Elapsed.TotalMilliseconds, -1, -1);
            _trace.Add(outside);
            return outside;
        }
        sigma = Math.Clamp(sigma, _cfg.SigmaMin, _cfg.SigmaMax);
        if (_nCoast > 0)
            coastSigma = Math.Clamp(coastSigma, coastLo, coastHi);

        // 3. Ratio test. Predicted reduction uses the subproblem's own defect proxy (the virtual control); actual reduction uses the true nonlinear defect at the new point.
        //  THE SOFT LIMITS ARE PRICED THE SAME WAY IN BOTH (2026-11-21, #247): the acceleration, turn-rate, glideslope, climb-rate and terminal-miss slacks, each at what the subproblem's solution pays for it.
        //  Those rows are exact in the state, so they carry no model error, and only the dynamics should move rho. Until then the turn rate was in the actual merit only and the glideslope, climb-rate and terminal slacks in neither:
        //  offline the ratio test then accepted steps that raised the whole merit (444 of 1134 on the roll-free landing) on a predicted reduction that was often negative.
        //  NOT THE TRUE VIOLATIONS. SCS meets those rows only to its tolerance: a terminal slack priced 0.11 sat beside a true miss of 0.7 m priced 293, and with that in the actual merit the ratio test refused most steps and the trust region shrank on nearly every one.
        double convexSlack = _sub.AccelerationSlackCost + _sub.RateSlackCost + _sub.PathSlackCost;
        double jLin = Fuel(x) + Smoothing(x, u) + _cfg.RhoVc * SumSquaresScaled(wv, _vcs) + convexSlack;
        (double jTrue, double defectNorm) = TrueCost(x, u, sigma, coastSigma,
                                                     out int defectChannel, out int defectNode);
        jTrue += convexSlack - SoftLimitCost(x, u);

        double predicted = _jRef - jLin;
        double actual = _jRef - jTrue;
        double rho = Math.Abs(predicted) > 1e-9
            ? actual / predicted
            : (actual >= 0 ? 1.0 : -1.0);

        double dX = MaxNormalisedDiff(x, _xbar, _xs, NX);
        double dU = MaxNormalisedDiff(u, _ubar, _us, NU);
        double dSigma = Math.Max(Math.Abs(sigma - _sigBar), Math.Abs(coastSigma - _sigCBar)) / _cfg.SigmaScale;
        double used = Math.Max(Math.Max(dX, dU), dSigma);

        // 4. Accept or reject.
        bool accepted = rho > RhoAccept;
        double step = 0.0;
        if (accepted)
        {
            _xbar = x;
            _ubar = u;
            // Reproject onto the unit sphere: the subproblem only enforces the tangent plane at the reference, which is exact only at the fixpoint.
            // Skipping this lets |q| drift and quietly degrades every subsequent linearisation.
            NormaliseQuaternions(_xbar);
            _sigBar = sigma;
            _sigCBar = coastSigma;
            _jRef = jTrue;
            step = Math.Max(dX, dSigma);   // matches the reference: dU is not part of the measure
        }

        // 5. Trust-region update. Grow only if the step actually used most of the radius - otherwise the radius is not what is limiting progress.
        if (rho < RhoShrink)
            TrustRegion = Math.Max(TrustRegionMin, TrustRegion * Shrink);
        else if (rho >= RhoGrow && used >= 0.8 * TrustRegion)
            TrustRegion = Math.Min(TrustRegionMax, TrustRegion * Grow);

        var result = new ScvxIteration(index, true, accepted, rho, TrustRegion, _sigBar,
            step, defectNorm, _jRef, _sub.Iterations, sw.Elapsed.TotalMilliseconds,
            defectChannel, defectNode);
        _trace.Add(result);
        return result;
    }

    // ------------------------------------------------------------ merit terms

    /// <summary>
    /// Merit function at a candidate point, using the TRUE nonlinear dynamics.
    /// Returns the cost and the largest normalised integration defect (the
    /// convergence measure - a converged SCvx solution must be dynamically
    /// feasible, not merely optimal for its own linearisation).
    /// </summary>
    public (double Cost, double DefectNorm) TrueCost(double[] x, double[] u, double sigma)
        => TrueCost(x, u, sigma, _sigCBar, out _, out _);

    public (double Cost, double DefectNorm) TrueCost(double[] x, double[] u, double sigma,
                                                     out int worstChannel, out int worstNode)
        => TrueCost(x, u, sigma, _sigCBar, out worstChannel, out worstNode);

    /// <summary>
    /// As above, and also reports WHERE the worst defect is - which state channel and
    /// which interval. Without that the magnitude is close to unreadable, because the
    /// max runs over channels whose scales differ by orders of magnitude and whose
    /// units differ entirely.
    /// </summary>
    public (double Cost, double DefectNorm) TrueCost(double[] x, double[] u, double sigma, double coastSigma,
                                                     out int worstChannel, out int worstNode)
    {
        Span<double> fk = stackalloc double[NX];
        Span<double> fk1 = stackalloc double[NX];
        double sumSq = 0, worst = 0;
        worstChannel = -1;
        worstNode = -1;

        for (int k = 0; k < _n - 1; k++)
        {
            // Each interval with its phase's dynamics and step: the ignition node is evaluated with the engine off as the coast's end and lit as the burn's start.
            Dynamics6Dof.Params pk = ParamsOf(k);
            if (k == 0 || k == _nCoast)
                Dynamics6Dof.Eval(x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU), pk, fk);
            Dynamics6Dof.Eval(x.AsSpan((k + 1) * NX, NX), u.AsSpan((k + 1) * NU, NU), pk, fk1);
            double half = 0.5 * IntervalDt(k, sigma, coastSigma);
            for (int i = 0; i < NX; i++)
            {
                double d = x[(k + 1) * NX + i] - x[k * NX + i] - half * (fk[i] + fk1[i]);
                double priced = d / _vcs[i];
                sumSq += priced * priced;
                double scaled = d / _xs[i];
                if (Math.Abs(scaled) > worst)
                {
                    worst = Math.Abs(scaled);
                    worstChannel = i;
                    worstNode = k;
                }
            }
            fk1.CopyTo(fk);
        }

        return (Fuel(x) + Smoothing(x, u) + _cfg.RhoVc * sumSq + SoftLimitCost(x, u), worst);
    }

    /// <summary>
    /// Every soft limit's price on a trajectory: acceleration, turn rate, glideslope, climb rate and terminal miss, each as its L1 slack would pay it. A seed's reference is priced with this (Initialize, Reseed); a solved step's slacks are taken from the subproblem instead (Iterate).
    /// </summary>
    private double SoftLimitCost(double[] x, double[] u)
        => AccelerationViolationCost(x, u) + RateViolationCost(x) + PathViolationCost(x);

    /// <summary>The glideslope, climb-rate and terminal-miss limits' price on the trajectory, as the subproblem's slacks pay it at the optimum: each L1 slack is the violation itself, because those rows are exact in the state (AssembleCone, AssembleOrthants, AssembleEqualities).</summary>
    private double PathViolationCost(double[] x)
    {
        double gs = 0.0, vz = 0.0, tmPos = 0.0, tmVel = 0.0;
        if (_cfg.GlideSlopeEnabled)
        {
            double cot = _cfg.CotGlideSlope;
            double entry = Scvx6DofSubproblemScs.GlideSlopeEntryAllowance(_cfg, _x0, _xf);
            for (int node = 1; node < _n; node++)
            {
                double dx = x[node * NX + Dynamics6Dof.IR] - _xf[Dynamics6Dof.IR];
                double dy = x[node * NX + Dynamics6Dof.IR + 1] - _xf[Dynamics6Dof.IR + 1];
                double height = x[node * NX + Dynamics6Dof.IR + 2] - _xf[Dynamics6Dof.IR + 2];
                double over = Math.Sqrt(dx * dx + dy * dy) - (cot * height + Scvx6DofSubproblemScs.GlideSlopeAllowanceAt(entry, node, _n));
                gs += _cfg.GlideSlopeWeight * Math.Max(0.0, over) / _xs[Dynamics6Dof.IR];
            }
        }
        if (_cfg.VzLimitEnabled)
            for (int node = 1; node < _n; node++)
                vz += _cfg.VzWeight * Math.Max(0.0, x[node * NX + Dynamics6Dof.IV + 2] - _cfg.VzMax) / _xs[Dynamics6Dof.IV];
        int last = (_n - 1) * NX;
        for (int a = 0; a < 3; a++)
        {
            if (_cfg.TerminalMissWeight > 0.0)
                tmPos += _cfg.TerminalMissWeight * Math.Abs(x[last + Dynamics6Dof.IR + a] - _xf[Dynamics6Dof.IR + a]) / _cfg.TerminalMissPriceScale;
            if (_cfg.TerminalSpeedWeight > 0.0)
                tmVel += _cfg.TerminalSpeedWeight * Math.Abs(x[last + Dynamics6Dof.IV + a] - _xf[Dynamics6Dof.IV + a]) / _cfg.TerminalSpeedPriceScale;
        }
        return gs + vz + tmPos + tmVel;
    }

    /// <summary>A coast duration for the reference: none without a coast, the pinned one when locked (Scvx6DofConfig.CoastSigmaFixed), otherwise the seed inside the bounds.</summary>
    private double CoastSeed(double seed)
    {
        if (_nCoast == 0)
            return 0.0;
        if (_cfg.CoastSigmaFixed is double fixedSigma)
            return fixedSigma;
        return Math.Clamp(double.IsFinite(seed) ? seed : 0.0, _cfg.CoastSigmaMin, _cfg.CoastSigmaMax);
    }

    /// <summary>A seeded RCS control is put inside its bounds (Scvx6DofConfig.RcsBounds). A plan resampled from one built with more RCS, or with RCS where there is none now, would otherwise ask the first subproblem for a step its trust region cannot take.</summary>
    private void ClampRcsSeed(double[] u)
    {
        for (int k = 0; k < _n; k++)
        {
            for (int a = 0; a < Dynamics6Dof.NRCS; a++)
            {
                int i = k * NU + Dynamics6Dof.IRCS + a;
                (double pos, double neg) = _cfg.ControlTorqueBounds(a, u[k * NU + Dynamics6Dof.IT]);
                u[i] = Math.Clamp(double.IsFinite(u[i]) ? u[i] : 0.0, -neg, pos);
            }
            // Each fin inside its range; a slot without a fin at zero.
            for (int i = 0; i < _cfg.FinCount; i++)
            {
                int j = k * NU + Dynamics6Dof.IFIN + i;
                (double lo, double hi) = _cfg.FinBounds(i);
                u[j] = Math.Clamp(double.IsFinite(u[j]) ? u[j] : 0.0, lo, hi);
            }
            // The coast's nodes have the engine off.
            if (k < _nCoast)
            {
                u[k * NU + Dynamics6Dof.ITDX] = 0.0;
                u[k * NU + Dynamics6Dof.ITDY] = 0.0;
                u[k * NU + Dynamics6Dof.IT] = 0.0;
                if (_cfg.RollTorqueFollowsThrust)
                    u[k * NU + Dynamics6Dof.ITAU] = 0.0;
            }
            // TorqueDemand pins the gimbals' own controls at zero.
            if (_cfg.TorqueDemand)
            {
                u[k * NU + Dynamics6Dof.ITDX] = 0.0;
                u[k * NU + Dynamics6Dof.ITDY] = 0.0;
                u[k * NU + Dynamics6Dof.ITAU] = 0.0;
            }
        }
    }

    /// <summary>The turn-rate limit's price on the trajectory, as the subproblem's slacks pay it (Scvx6DofConfig.RateMaxRadS).</summary>
    private double RateViolationCost(double[] x)
    {
        if (!_cfg.RateLimitEnabled)
            return 0.0;
        double violation = 0.0;
        for (int k = 1; k < _n; k++)
        {
            double wx = x[k * NX + Dynamics6Dof.IW], wy = x[k * NX + Dynamics6Dof.IW + 1], wz = x[k * NX + Dynamics6Dof.IW + 2];
            double roll = _cfg.RateLimitIncludesRoll ? wz * wz : 0.0;
            violation += Math.Max(0.0, Math.Sqrt(wx * wx + wy * wy + roll) - _cfg.RateMaxRadS);
        }
        return _cfg.RateWeight * violation / _cfg.XScale[Dynamics6Dof.IW];
    }

    private double AccelerationViolationCost(double[] x, double[] u)
    {
        if (!_cfg.AccelLimitEnabled)
            return 0.0;

        double violation = 0.0;
        for (int k = 0; k < _n; k++)
        {
            double thrust = u[k * NU + Dynamics6Dof.IT];
            double mass = x[k * NX + Dynamics6Dof.IM];
            violation += Math.Max(0.0, thrust - _cfg.AccelMax * mass);
        }
        return _cfg.AccelWeight * violation / _us[Dynamics6Dof.IT];
    }

    /// <summary>
    /// The true nonlinear defect of the CURRENT REFERENCE, reduced to ONE
    /// DIMENSIONLESS NUMBER: the worst "how many times over its own tolerance" any
    /// interval and channel reaches. 1.0 means exactly at tolerance.
    ///
    /// Two things are wrong with the flat DefectNorm that this replaces for gating,
    /// and they are independent.
    ///
    /// ONE - EVERY CHANNEL WAS MEASURED WITH A POSITION RULER. DefectNorm divides each
    /// channel by its own XScale and then the caller multiplied by the POSITION scale
    /// to "put it in metres". That is a real distance only when the worst channel is a
    /// position. XScale's position entry is the RANGE TO THE TARGET, so on flight
    /// 20260820-150038 (range ~2200 m) a 1 m gate meant a body-rate tolerance of
    /// 1/2200 rad/s - 0.026 degrees per second - tight purely because the vehicle
    /// happened to be two kilometres out, and looser as it closed in. Of the refusals
    /// on that flight that survived horizon weighting, 41 were rate and 11 attitude;
    /// none were position. The gate was almost never refusing what it was written to
    /// refuse. Here each channel is divided by a tolerance in ITS OWN UNITS - metres,
    /// m/s, rad/s, quaternion components, kg - so the answer means the same thing
    /// whatever the worst channel turns out to be, and does not drift with range.
    ///
    /// TWO - EVERY INTERVAL COUNTED EQUALLY. An MPC loop flies the first interval or
    /// two and re-solves; on that same flight, sigma was 18.8 s at a 0.10 s cadence,
    /// so the far end of a plan is re-solved of the order of a hundred times before
    /// the vehicle arrives. And the far end is exactly where the defect lives, for a
    /// reason that is not the plan being wrong: the collocation grid is uniform in
    /// time but the trajectory is not - the busiest interval carried 5-50x the
    /// velocity change of interval 0 across the same dt, because the terminal flare
    /// drives velocity, attitude and rate to their terminal values at once. So each
    /// interval is also divided by an ALLOWANCE that grows with how far ahead it is.
    ///
    /// THE HORIZON PART IS SELF-TIGHTENING, which is what makes relaxing a safety gate
    /// defensible. An interval eighteen seconds out today is one second out in
    /// seventeen seconds, and is then inside the commit window and judged at full
    /// strength. Nothing unphysical is ever flown; it merely stops being refused long
    /// before it can matter.
    ///
    /// Deliberately NOT used for the solver's own convergence test, which still runs
    /// on the honest full-horizon DefectNorm - this asks what is safe to FLY, not
    /// whether SCvx has converged.
    /// </summary>
    /// <param name="tol">
    /// Per-channel tolerance in SI units, NX long. Must be strictly positive: a zero
    /// entry would make its channel infinitely intolerant and refuse everything.
    /// </param>
    /// <param name="commitIntervals">Intervals judged at full strength, from 0.</param>
    /// <param name="farSlack">Allowance multiplier at the last interval. 1 disables.</param>
    /// <param name="worstRaw">The offending defect in its OWN units, for the report.</param>
    public double WeightedDefect(ReadOnlySpan<double> tol, int commitIntervals, double farSlack,
                                 out int worstChannel, out int worstNode, out double worstRaw)
        => WeightedDefect(_xbar, tol, commitIntervals, farSlack, out worstChannel, out worstNode, out worstRaw);

    /// <summary>Check a candidate state against the current reference controls and burn duration.</summary>
    public double WeightedDefect(ReadOnlySpan<double> state, ReadOnlySpan<double> tol, int commitIntervals,
                                 double farSlack, out int worstChannel, out int worstNode, out double worstRaw)
    {
        worstChannel = -1;
        worstNode = -1;
        worstRaw = double.NaN;
        if (state.Length < _n * NX || _ubar.Length < _n * NU || tol.Length < NX)
            return double.PositiveInfinity;

        int lastInterval = _n - 2;
        int commit = Math.Clamp(commitIntervals, 0, Math.Max(lastInterval, 0));
        double span = Math.Max(lastInterval - commit, 1);
        double slack = Math.Max(farSlack, 1.0);

        Span<double> fk = stackalloc double[NX];
        Span<double> fk1 = stackalloc double[NX];
        double worst = 0;

        for (int k = 0; k < _n - 1; k++)
        {
            Dynamics6Dof.Params pk = ParamsOf(k);
            if (k == 0 || k == _nCoast)
                Dynamics6Dof.Eval(state.Slice(k * NX, NX), _ubar.AsSpan(k * NU, NU), pk, fk);
            Dynamics6Dof.Eval(state.Slice((k + 1) * NX, NX), _ubar.AsSpan((k + 1) * NU, NU), pk, fk1);
            double half = 0.5 * IntervalDt(k, _sigBar, _sigCBar);

            // Allowance is 1 across the commit window, then rises LINEARLY to slack at the last interval.
            // Linear in the ALLOWANCE rather than in its reciprocal, because the allowance is the half with a physical reading: "this interval may carry N times its tolerance".
            double frac = k <= commit ? 0.0 : (k - commit) / span;
            double allowance = 1.0 + frac * (slack - 1.0);

            for (int i = 0; i < NX; i++)
            {
                double d = state[(k + 1) * NX + i] - state[k * NX + i] - half * (fk[i] + fk1[i]);
                double t = tol[i];
                if (!(t > 0.0))
                    continue;
                double ratio = Math.Abs(d) / (t * allowance);
                if (!double.IsFinite(ratio))
                    ratio = double.PositiveInfinity;
                if (ratio > worst)
                {
                    worst = ratio;
                    worstChannel = i;
                    worstNode = k;
                    worstRaw = d;
                }
            }
            fk1.CopyTo(fk);
        }
        return worst;
    }

    /// <summary>Copy the reference with mass integrated from measured mass and the planned thrust.</summary>
    public double[]? CopyReferenceWithMassBalance()
    {
        if (!_initialised)
            return null;
        double mass = _x0[Dynamics6Dof.IM];
        if (!(_dtauB * _sigBar > 0.0) || !double.IsFinite(_sigBar) || !(mass > 0.0) || !double.IsFinite(mass))
            return null;

        double[] state = (double[])_xbar.Clone();
        state[Dynamics6Dof.IM] = mass;
        for (int k = 0; k < _n - 1; k++)
        {
            Dynamics6Dof.Params pk = ParamsOf(k);
            double flow = Dynamics6Dof.MassFlow(pk, _ubar.AsSpan(k * NU, NU));
            double nextFlow = Dynamics6Dof.MassFlow(pk, _ubar.AsSpan((k + 1) * NU, NU));
            if (!(flow >= 0.0) || !(nextFlow >= 0.0))
                return null;
            mass -= 0.5 * IntervalDt(k, _sigBar, _sigCBar) * (flow + nextFlow);
            if (!(mass > 0.0))
                return null;
            state[(k + 1) * NX + Dynamics6Dof.IM] = mass;
        }
        return state;
    }

    private double Fuel(double[] x)
    {
        double mInit = _x0[Dynamics6Dof.IM];
        return (mInit - x[(_n - 1) * NX + Dynamics6Dof.IM]) / mInit;
    }

    private double Smoothing(double[] x, double[] u)
    {
        double du = 0;
        for (int k = 0; k < _n - 1; k++)
            for (int j = 0; j < NU; j++)
            {
                if (Scvx6DofSubproblemScs.IsIgnitionStep(_nCoast, k, j))
                    continue;
                double d = (u[(k + 1) * NU + j] - u[k * NU + j]) / _us[j];
                du += d * d;
            }
        double ww = 0, coastWw = 0;
        for (int k = 0; k < _n; k++)
        {
            // The coast's own rate price (Scvx6DofConfig.CoastRateWeight), as the subproblem prices it.
            double coast = Scvx6DofSubproblemScs.CoastRateWeightAt(_cfg, _nCoast, k);
            for (int i = 0; i < 3; i++)
            {
                double w = x[k * NX + Dynamics6Dof.IW + i];
                ww += w * w;
                coastWw += coast * w * w;
            }
        }
        double rcs = 0;
        if (_cfg.RcsWeight > 0.0)
            for (int k = 0; k < _n; k++)
                for (int j = 0; j < Dynamics6Dof.NRCS; j++)
                {
                    double r = u[k * NU + Dynamics6Dof.IRCS + j] / _us[Dynamics6Dof.IRCS + j];
                    rcs += r * r;
                }
        double fin = 0;
        if (_cfg.FinWeight > 0.0)
            for (int k = 0; k < _n; k++)
                for (int i = 0; i < _cfg.FinCount; i++)
                {
                    double r = u[k * NU + Dynamics6Dof.IFIN + i] / _us[Dynamics6Dof.IFIN + i];
                    fin += r * r;
                }
        return _cfg.WDu * du + _cfg.WW * ww + coastWw + _cfg.RcsWeight * rcs + _cfg.FinWeight * fin;
    }

    private static double SumSquaresScaled(double[] v, double[] scale)
    {
        int stride = scale.Length;
        double s = 0;
        for (int i = 0; i < v.Length; i++)
        {
            double d = v[i] / scale[i % stride];
            s += d * d;
        }
        return s;
    }

    private static double MaxNormalisedDiff(double[] a, double[] b, double[] scale, int stride)
    {
        double worst = 0;
        for (int i = 0; i < a.Length; i++)
            worst = Math.Max(worst, Math.Abs((a[i] - b[i]) / scale[i % stride]));
        return worst;
    }

    /// <summary>
    /// Renormalise every node's quaternion, and keep the trajectory on ONE branch of
    /// the double cover.
    ///
    /// The subproblem knows nothing about q and -q being the same rotation - it sees
    /// four ordinary variables - so a solution is free to come back with a node on the
    /// opposite branch from its neighbour. Nothing is wrong with such a trajectory
    /// physically, but the next linearisation is taken about it, and the collocation
    /// defect is arithmetic on the components: a branch change between adjacent nodes
    /// reads as a jump of up to 2 in a channel whose scale is 1.
    ///
    /// Node 0 is left alone - it is pinned to the measured state by the initial-state
    /// equality, so it is the reference the rest should agree with rather than the
    /// other way round. On a trajectory that is already consistent this is a no-op,
    /// which is why it does not move the Python reference comparison.
    /// </summary>
    private void NormaliseQuaternions(double[] x)
    {
        for (int k = 0; k < _n; k++)
        {
            int q = k * NX + Dynamics6Dof.IQ;
            double norm = Math.Sqrt(x[q] * x[q] + x[q + 1] * x[q + 1]
                                  + x[q + 2] * x[q + 2] + x[q + 3] * x[q + 3]);
            if (norm < 1e-12) continue;
            for (int i = 0; i < 4; i++) x[q + i] /= norm;

            if (k == 0) continue;
            int p = (k - 1) * NX + Dynamics6Dof.IQ;
            double dot = x[q] * x[p] + x[q + 1] * x[p + 1]
                       + x[q + 2] * x[p + 2] + x[q + 3] * x[p + 3];
            if (dot < 0.0)
                for (int i = 0; i < 4; i++) x[q + i] = -x[q + i];
        }
    }

    /// <summary>
    /// Puts the terminal attitude on the quaternion branch of the last node of the reference. q and -q are the same rotation, but the terminal equality is arithmetic on the components. A reference that follows a sign change of the measured attitude ends on the other branch, where the pinned end state is 2 away in a channel whose scale is 1, and no step inside the trust region reaches it, so every subproblem is infeasible.
    /// </summary>
    private void AlignTerminalBranch()
    {
        int q = (_n - 1) * NX + Dynamics6Dof.IQ, f = Dynamics6Dof.IQ;
        double dot = _xbar[q] * _xf[f] + _xbar[q + 1] * _xf[f + 1]
                   + _xbar[q + 2] * _xf[f + 2] + _xbar[q + 3] * _xf[f + 3];
        if (dot < 0.0)
            for (int i = 0; i < 4; i++) _xf[f + i] = -_xf[f + i];
    }
}

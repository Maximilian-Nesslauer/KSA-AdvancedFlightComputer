namespace Scvx;

/// <summary>Outcome of one SCvx iteration, for tracing and for the caller's loop control.</summary>
public readonly record struct ScvxIteration(
    int Index,
    bool Solved,          // did the convex subproblem solve at all
    bool Accepted,        // did the ratio test accept the step
    double Rho,           // actual / predicted cost reduction
    double TrustRegion,   // radius AFTER this iteration's update
    double Sigma,
    double Step,          // max(dX, dSigma), normalised — the convergence measure
    double DefectNorm,    // max |true nonlinear defect| / Xscale
    double Cost,          // true merit at the reference after this iteration
    int SolverIterations,
    double ElapsedMs);

public enum ScvxStatus
{
    /// <summary>Step and defect both under tolerance — a genuine fixed point.</summary>
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
///   Both must include the same constant terms or the ratio is meaningless —
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
    private const int NU = Dynamics6Dof.NU;

    private readonly Scvx6DofConfig _cfg;
    private readonly Dynamics6Dof.Params _dyn;
    private readonly Scvx6DofSubproblemScs _sub;
    private readonly int _n;
    private readonly double _dtau;
    private readonly double[] _xs, _us;

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
    /// Tolerance each convex subproblem is solved to — distinct from the OUTER
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
    /// rejects truncated solves outright — it is that guard, not a tight tolerance,
    /// that keeps a bad subproblem out of the ratio test.
    /// </summary>
    /// <summary>
    /// Subproblem tolerance for FLIGHT. Not SCS's default and not the validation
    /// tolerance (<see cref="ScsWorkspace.DefaultEps"/>, 1e-7) — offline solves keep
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
    /// vehicle — those solves hit the wall-clock budget and are cut off either way.
    /// 1e-3 buys little more and leaves less margin on the iterate SCvx's ratio test
    /// depends on.
    /// </summary>
    public const double RealTimeEps = 1e-4;

    /// <summary>
    /// Hard cap on ADMM iterations per subproblem. THE REAL-TIME PATH MUST HAVE A
    /// BOUNDED WORST CASE — a mean is not a guarantee.
    ///
    /// ScsWorkspace.DefaultMaxIterations is 100,000, which is an OFFLINE VALIDATION
    /// number: it exists so a hard problem is solved properly rather than silently
    /// truncated. On a game thread it is a licence to stall for seconds. Measured in
    /// closed loop, an UNCAPPED worst-case subproblem ran 31,800 ADMM iterations /
    /// 1.7 s while the mean was ~400 — and that is on a synthetic problem. In game
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
    /// last point is a good start. This is the ONLY warm start SCS offers —
    /// scs_update refreshes just b and c, never A or P, and our Jacobian block
    /// lives in A, so the workspace itself must be rebuilt every iteration.
    /// </summary>
    public bool WarmStart { get; init; } = true;
    public double DefectTolerance { get; init; } = 1e-3;

    // Reference trajectory — the SCvx state that persists across iterations and,
    // in receding-horizon use, across guidance cycles.
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
    public double Cost => _jRef;
    public int IterationCount { get; private set; }

    /// <summary>Why the most recent subproblem failed — SCS status plus its own text.</summary>
    public string LastFailureReason { get; private set; } = "";

    public Scvx6DofSolver(Scvx6DofConfig cfg, Dynamics6Dof.Params? dyn = null)
    {
        _cfg = cfg;
        _dyn = dyn ?? new Dynamics6Dof.Params();
        _n = cfg.Nodes;
        _dtau = 1.0 / (_n - 1);
        _xs = cfg.XScale;
        _us = cfg.ResolvedUScale;
        _sub = new Scvx6DofSubproblemScs(cfg);

        _a = new double[_n * NX * NX];
        _b = new double[_n * NX * NU];
        _f0 = new double[_n * NX];
    }

    /// <summary>
    /// Seed the loop. x0 is the full 14-component initial state; xf the 13
    /// pinned terminal components (mass is free). xSeed/uSeed are the initial
    /// reference trajectory — a straight line is fine cold; in receding-horizon
    /// use, pass the previous cycle's solution shifted forward.
    /// </summary>
    public void Initialize(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf,
                           double[] xSeed, double[] uSeed, double sigmaSeed,
                           double? trustRegion = null)
    {
        if (xSeed.Length != _n * NX) throw new ArgumentException($"xSeed must be {_n * NX} long");
        if (uSeed.Length != _n * NU) throw new ArgumentException($"uSeed must be {_n * NU} long");

        _x0 = x0.ToArray();
        _xf = xf.ToArray();
        _xbar = (double[])xSeed.Clone();
        _ubar = (double[])uSeed.Clone();
        _sigBar = sigmaSeed;
        TrustRegion = trustRegion ?? TrustRegionMax;
        IterationCount = 0;
        _trace.Clear();
        _sub.ResetWarmStart();

        (_jRef, _) = TrueCost(_xbar, _ubar, _sigBar);
    }

    /// <summary>
    /// Advance to a new initial state, keeping the current reference trajectory
    /// (shifted by the caller) AND the ADMM warm start — the receding-horizon
    /// entry point, as opposed to <see cref="Initialize"/> which starts cold.
    ///
    /// The distinction matters: Initialize throws away the solver's iterate,
    /// which is the whole reason SCS was chosen. Here the previous cycle's
    /// solution is still an excellent starting point, because one control
    /// interval of vehicle motion barely changes the problem.
    /// </summary>
    public void Reseed(ReadOnlySpan<double> x0, double[] xShifted, double[] uShifted,
                       double sigma, double? trustRegion = null)
    {
        _x0 = x0.ToArray();
        _xbar = (double[])xShifted.Clone();
        _ubar = (double[])uShifted.Clone();
        NormaliseQuaternions(_xbar);
        _sigBar = sigma;
        if (trustRegion.HasValue) TrustRegion = trustRegion.Value;
        _trace.Clear();
        IterationCount = 0;
        (_jRef, _) = TrueCost(_xbar, _ubar, _sigBar);
    }

    /// <summary>
    /// Run iterations until convergence, the budget runs out, or the trust
    /// region collapses. Safe to call repeatedly — it continues from the current
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
            // At least one iteration always runs: a cycle that returns without taking
            // a step hands back the stale plan, which is worse than being late once.
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

    /// <summary>One SCvx iteration: linearise, solve, ratio-test, update.</summary>
    public ScvxIteration Iterate()
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        int index = IterationCount++;

        // 1. Linearise the true dynamics about the current reference.
        for (int k = 0; k < _n; k++)
            Dynamics6Dof.Jacobian(
                _xbar.AsSpan(k * NX, NX), _ubar.AsSpan(k * NU, NU), _dyn,
                _f0.AsSpan(k * NX, NX),
                _a.AsSpan(k * NX * NX, NX * NX),
                _b.AsSpan(k * NX * NU, NX * NU));

        // 2. Solve the convex subproblem inside the current trust region.
        //    Warm-started from the previous solve's iterate: the problem changes
        //    between iterations, but only within the trust region, so the last
        //    point is a good ADMM start. ScsWorkspace refuses to carry forward an
        //    unusable solution, so a failed solve cannot poison this.
        _sub.Assemble(_x0, _xf, _xbar, _ubar, _sigBar, TrustRegion, _a, _b, _f0);
        ScsStatus st = _sub.Run(warmStart: WarmStart && _sub.HasWarmStart,
                                maxIterations: MaxSubproblemIterations,
                                epsAbs: SubproblemEps, epsRel: SubproblemEps);

        // A truncated solve counts as a failure, not a step. SCS returns
        // SolvedInaccurate when it runs out of iterations, and that iterate can
        // sit far outside the trust region — accepting it produced a step 100x
        // the radius and a rho of +335 before this guard existed.
        // Truncation is a BUDGET problem, not a trajectory problem: retry the same
        // already-assembled subproblem with more iterations rather than shrinking the
        // trust region, which would only make it harder. Warm-started from the last
        // GOOD iterate, since truncated ones are no longer carried forward.
        if (_sub.HitIterationLimit && EscalatedSubproblemIterations > MaxSubproblemIterations)
        {
            Escalations++;
            st = _sub.Run(warmStart: WarmStart && _sub.HasWarmStart,
                          maxIterations: EscalatedSubproblemIterations,
                          epsAbs: SubproblemEps, epsRel: SubproblemEps);
        }

        if (!st.IsUsable() || _sub.HitIterationLimit)
        {
            // Subproblem failure is not fatal — shrink and retry from the same
            // reference. A trust region that keeps collapsing is the signal that
            // something is actually wrong.
            TrustRegion = Math.Max(TrustRegionMin, TrustRegion * Shrink);
            LastFailureReason = $"{st} \"{_sub.StatusText}\"" +
                                (_sub.HitIterationLimit ? " [truncated at iteration cap]" : "");
            var failed = new ScvxIteration(index, false, false, double.NaN, TrustRegion,
                _sigBar, 0, double.NaN, _jRef, _sub.Iterations, sw.Elapsed.TotalMilliseconds);
            _trace.Add(failed);
            return failed;
        }

        double[] x = _sub.SolutionX, u = _sub.SolutionU, wv = _sub.SolutionWv;
        double sigma = _sub.SolutionSigma;

        // 3. Ratio test. Predicted reduction uses the subproblem's own defect
        //    proxy (the virtual control); actual reduction uses the true
        //    nonlinear defect at the new point.
        double jLin = Fuel(x) + Smoothing(x, u) + _cfg.RhoVc * SumSquaresScaled(wv, _xs);
        (double jTrue, double defectNorm) = TrueCost(x, u, sigma);

        double predicted = _jRef - jLin;
        double actual = _jRef - jTrue;
        double rho = Math.Abs(predicted) > 1e-9
            ? actual / predicted
            : (actual >= 0 ? 1.0 : -1.0);

        double dX = MaxNormalisedDiff(x, _xbar, _xs, NX);
        double dU = MaxNormalisedDiff(u, _ubar, _us, NU);
        double dSigma = Math.Abs(sigma - _sigBar) / _cfg.SigmaScale;
        double used = Math.Max(Math.Max(dX, dU), dSigma);

        // 4. Accept or reject.
        bool accepted = rho > RhoAccept;
        double step = 0.0;
        if (accepted)
        {
            _xbar = x;
            _ubar = u;
            // Reproject onto the unit sphere: the subproblem only enforces the
            // tangent plane at the reference, which is exact only at the
            // fixpoint. Skipping this lets |q| drift and quietly degrades every
            // subsequent linearisation.
            NormaliseQuaternions(_xbar);
            _sigBar = sigma;
            _jRef = jTrue;
            step = Math.Max(dX, dSigma);   // matches the reference: dU is not part of the measure
        }

        // 5. Trust-region update. Grow only if the step actually used most of the
        //    radius — otherwise the radius is not what is limiting progress.
        if (rho < RhoShrink)
            TrustRegion = Math.Max(TrustRegionMin, TrustRegion * Shrink);
        else if (rho >= RhoGrow && used >= 0.8 * TrustRegion)
            TrustRegion = Math.Min(TrustRegionMax, TrustRegion * Grow);

        var result = new ScvxIteration(index, true, accepted, rho, TrustRegion, _sigBar,
            step, defectNorm, _jRef, _sub.Iterations, sw.Elapsed.TotalMilliseconds);
        _trace.Add(result);
        return result;
    }

    // ------------------------------------------------------------ merit terms

    /// <summary>
    /// Merit function at a candidate point, using the TRUE nonlinear dynamics.
    /// Returns the cost and the largest normalised integration defect (the
    /// convergence measure — a converged SCvx solution must be dynamically
    /// feasible, not merely optimal for its own linearisation).
    /// </summary>
    public (double Cost, double DefectNorm) TrueCost(double[] x, double[] u, double sigma)
    {
        Span<double> fk = stackalloc double[NX];
        Span<double> fk1 = stackalloc double[NX];
        double sumSq = 0, worst = 0;
        double half = 0.5 * _dtau * sigma;

        Dynamics6Dof.Eval(x.AsSpan(0, NX), u.AsSpan(0, NU), _dyn, fk);
        for (int k = 0; k < _n - 1; k++)
        {
            Dynamics6Dof.Eval(x.AsSpan((k + 1) * NX, NX), u.AsSpan((k + 1) * NU, NU), _dyn, fk1);
            for (int i = 0; i < NX; i++)
            {
                double d = x[(k + 1) * NX + i] - x[k * NX + i] - half * (fk[i] + fk1[i]);
                double scaled = d / _xs[i];
                sumSq += scaled * scaled;
                worst = Math.Max(worst, Math.Abs(scaled));
            }
            fk1.CopyTo(fk);
        }

        return (Fuel(x) + Smoothing(x, u) + _cfg.RhoVc * sumSq, worst);
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
                double d = (u[(k + 1) * NU + j] - u[k * NU + j]) / _us[j];
                du += d * d;
            }
        double ww = 0;
        for (int k = 0; k < _n; k++)
            for (int i = 0; i < 3; i++)
            {
                double w = x[k * NX + Dynamics6Dof.IW + i];
                ww += w * w;
            }
        return _cfg.WDu * du + _cfg.WW * ww;
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

    private void NormaliseQuaternions(double[] x)
    {
        for (int k = 0; k < _n; k++)
        {
            int q = k * NX + Dynamics6Dof.IQ;
            double norm = Math.Sqrt(x[q] * x[q] + x[q + 1] * x[q + 1]
                                  + x[q + 2] * x[q + 2] + x[q + 3] * x[q + 3]);
            if (norm < 1e-12) continue;
            for (int i = 0; i < 4; i++) x[q + i] /= norm;
        }
    }
}

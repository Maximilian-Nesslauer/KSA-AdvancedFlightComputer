using AdvancedFlightComputer.Guidance.Conic;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;

namespace AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>Outcome of one SCvx iteration of the glide-and-burn problem.</summary>
public readonly record struct Scvx3DofIteration(
    int Index,
    bool Solved,
    bool Accepted,
    double Rho,
    double TrustRegion,     // radius AFTER this iteration's update
    double SigmaGlide,
    double SigmaBurn,
    double Step,            // max(dX, dSigma), normalised - the convergence measure
    double DefectNorm,      // max |true nonlinear defect| / Xscale
    double Cost,
    int SolverIterations,
    double ElapsedMs,
    int DefectChannel,      // state index 0..6, or -1
    int DefectNode);        // interval k, or -1

/// <summary>
/// The SCvx loop for the glide-and-burn problem: 3dof.py's loop, on the same machinery as <see cref="Scvx6DofSolver"/>.
///
/// What is different from the 6-DOF loop is all in the phases. There are two durations, sigma_glide (which IS the time to ignition) and sigma_burn, each with its own pseudo-time step; the dynamics are linearised twice, once with the engine off over the glide and once with it on over the burn, and node K carries both. The ratio test, the trust-region schedule, the accept-and-reproject step and the convergence test are the 6-DOF's, which are the script's.
///
/// The unit attitude is re-projected on accept, as the 6-DOF quaternion is: the subproblem only enforces the plane tangent to the sphere at the reference, and without the projection the reference drifts off the sphere until the tangent plane and a tight trust region disagree.
/// </summary>
public sealed class Scvx3DofSolver
{
    private const int NX = PointMass3Dof.NX;
    private const int NU = PointMass3Dof.NU;

    private readonly Scvx3DofConfig _cfg;
    private IPointMassModel _model;
    private readonly Scvx3DofSubproblemScs _sub;
    private readonly Linearisation3Dof _lin;
    private readonly int _n, _k;
    private readonly double _dtauG, _dtauB;
    private double[] _xs => _cfg.XScale;
    private readonly double[] _alphaRadius;

    // Trust-region schedule: 3dof.py's constants, which are the 6-DOF's.
    public double TrustRegion { get; private set; } = 0.1;
    public double TrustRegionMin { get; init; } = 1e-3;
    public double TrustRegionMax { get; init; } = 0.1;
    public double RhoAccept { get; init; } = 0.0;
    public double RhoShrink { get; init; } = 0.25;
    public double RhoGrow { get; init; } = 0.7;
    public double Shrink { get; init; } = 0.5;
    public double Grow { get; init; } = 1.5;
    public double StepTolerance { get; init; } = 8e-3;
    public double DefectTolerance { get; init; } = 1e-3;

    /// <summary>Tolerance each subproblem is solved to. Offline default; flight passes <see cref="Scvx6DofSolver.RealTimeEps"/>.</summary>
    public double SubproblemEps { get; set; } = ScsWorkspace.DefaultEps;

    /// <summary>ADMM cap per subproblem. A truncated solve counts as a failure, never as a step. See Scvx6DofSolver.MaxSubproblemIterations.</summary>
    public int MaxSubproblemIterations { get; set; } = ScsWorkspace.DefaultMaxIterations;

    /// <summary>Budget for one retry of a subproblem that only ran out of ADMM iterations. See Scvx6DofSolver.EscalatedSubproblemIterations.</summary>
    public int EscalatedSubproblemIterations { get; set; } = ScsWorkspace.DefaultMaxIterations;

    public bool WarmStart { get; init; } = true;

    private double[] _xbar = [], _ubar = [];
    private double _sigGBar, _sigBBar, _jRef;
    private double[] _x0 = [], _xf = [];
    private readonly double[] _anchor = new double[3];
    private double _anchorCos = -1.0;

    private readonly List<Scvx3DofIteration> _trace = [];
    public IReadOnlyList<Scvx3DofIteration> Trace => _trace;

    public double[] ReferenceX => _xbar;
    public double[] ReferenceU => _ubar;
    public double SigmaGlide => _sigGBar;
    public double SigmaBurn => _sigBBar;
    public double Cost => _jRef;
    public int IterationCount { get; private set; }
    public int Nodes => _n;
    public int GlideIntervals => _k;
    public Scvx3DofConfig Config => _cfg;
    /// <summary>
    /// The model the next iteration linearises. Settable between cycles so the guidance can hand in corrected aerodynamics without rebuilding the solver; the merit baseline is recomputed on the next <see cref="Reseed"/> or <see cref="Initialize"/>, so change it only before one of those.
    /// </summary>
    public IPointMassModel Model { get => _model; set => _model = value ?? throw new ArgumentNullException(nameof(value)); }
    public string LastFailureReason { get; private set; } = "";
    public bool TimedOut { get; private set; }

    public Scvx3DofSolver(Scvx3DofConfig cfg, IPointMassModel model)
    {
        _cfg = cfg;
        _model = model;
        _n = cfg.Nodes;
        _k = cfg.GlideIntervals;
        _dtauG = _k > 0 ? 1.0 / _k : 0.0;
        _dtauB = 1.0 / cfg.BurnIntervals;
        _sub = new Scvx3DofSubproblemScs(cfg);
        _lin = new Linearisation3Dof(_n);
        _alphaRadius = new double[_n];
    }

    /// <summary>
    /// Seed the loop cold. x0 is the full 7-component initial state, xf the terminal position and velocity (6); the seed is any reference - a straight line is enough for the script's case.
    /// </summary>
    public void Initialize(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf, double[] xSeed, double[] uSeed,
                           double sigmaGlide, double sigmaBurn, double? trustRegion = null)
    {
        if (xSeed.Length != _n * NX) throw new ArgumentException($"xSeed must be {_n * NX} long");
        if (uSeed.Length != _n * NU) throw new ArgumentException($"uSeed must be {_n * NU} long");
        if (xf.Length != 6) throw new ArgumentException("xf is terminal position and velocity, 6 long");

        _x0 = x0.ToArray();
        _xf = xf.ToArray();
        _xbar = (double[])xSeed.Clone();
        _ubar = (double[])uSeed.Clone();
        NormaliseAttitudes(_ubar);
        _sigGBar = _k > 0 ? sigmaGlide : 0.0;
        _sigBBar = sigmaBurn;
        TrustRegion = trustRegion ?? TrustRegionMax;
        IterationCount = 0;
        _trace.Clear();
        _sub.ResetWarmStart();
        _jRef = TrueCost(_xbar, _ubar, _sigGBar, _sigBBar, out _, out _, out _);
    }

    /// <summary>
    /// Advance to a new initial state, keeping a (shifted) reference AND the ADMM warm start - the receding-horizon entry point.
    /// </summary>
    public void Reseed(ReadOnlySpan<double> x0, double[] xShifted, double[] uShifted,
                       double sigmaGlide, double sigmaBurn, double? trustRegion = null)
    {
        _x0 = x0.ToArray();
        _xbar = (double[])xShifted.Clone();
        _ubar = (double[])uShifted.Clone();
        NormaliseAttitudes(_ubar);
        _sigGBar = _k > 0 ? sigmaGlide : 0.0;
        _sigBBar = sigmaBurn;
        if (trustRegion.HasValue) TrustRegion = trustRegion.Value;
        _trace.Clear();
        IterationCount = 0;
        _jRef = TrueCost(_xbar, _ubar, _sigGBar, _sigBBar, out _, out _, out _);
    }

    /// <summary>Change the terminal target without reseeding (the site is fixed in the frame, but the aim altitude may move).</summary>
    public void SetTerminal(ReadOnlySpan<double> xf)
    {
        if (xf.Length != 6) throw new ArgumentException("xf is terminal position and velocity, 6 long");
        _xf = xf.ToArray();
    }

    /// <summary>
    /// Hold node 0's body axis within <paramref name="maxAngleRad"/> of <paramref name="direction"/>. Needs <see cref="Scvx3DofConfig.AttitudeAnchor"/>; an angle of pi or more releases it.
    /// </summary>
    public void SetAttitudeAnchor(ReadOnlySpan<double> direction, double maxAngleRad)
    {
        double len = Math.Sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
        if (!(len > 0.0) || !(maxAngleRad < Math.PI))
        {
            Array.Clear(_anchor);
            _anchorCos = -1.0;
            return;
        }
        for (int j = 0; j < 3; j++) _anchor[j] = direction[j] / len;
        _anchorCos = Math.Cos(Math.Max(maxAngleRad, 0.0));
    }

    /// <summary>
    /// Iterate until convergence, the iteration budget, the deadline, or a collapsed trust region. At least one iteration always runs. See Scvx6DofSolver.Solve.
    /// </summary>
    public ScvxStatus Solve(int maxIterations, double deadlineMs = 0.0)
    {
        TimedOut = false;
        var sw = System.Diagnostics.Stopwatch.StartNew();
        for (int i = 0; i < maxIterations; i++)
        {
            Scvx3DofIteration it = Iterate();
            if (!it.Solved && TrustRegion <= TrustRegionMin * 1.001)
                return _trace.Any(t => t.Accepted) ? ScvxStatus.TrustRegionCollapsed : ScvxStatus.Failed;
            if (it.Accepted && it.Step < StepTolerance && it.DefectNorm < DefectTolerance)
                return ScvxStatus.Converged;
            if (deadlineMs > 0.0 && sw.Elapsed.TotalMilliseconds >= deadlineMs)
            {
                TimedOut = true;
                return ScvxStatus.IterationLimit;
            }
        }
        return ScvxStatus.IterationLimit;
    }

    public static string ChannelName(int i) => i switch
    {
        0 => "rx", 1 => "ry", 2 => "rz",
        3 => "vx", 4 => "vy", 5 => "vz",
        6 => "mass",
        _ => "-",
    };

    /// <summary>One SCvx iteration: linearise, solve, ratio-test, update.</summary>
    public Scvx3DofIteration Iterate()
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        int index = IterationCount++;

        Linearise();

        _sub.Assemble(_x0, _xf, _xbar, _ubar, _sigGBar, _sigBBar, TrustRegion, _lin, _alphaRadius,
                      _anchor, _anchorCos);
        ScsStatus st = _sub.Run(WarmStart && _sub.HasWarmStart, MaxSubproblemIterations, SubproblemEps, SubproblemEps);
        if (_sub.HitIterationLimit && EscalatedSubproblemIterations > MaxSubproblemIterations)
            st = _sub.Run(WarmStart && _sub.HasWarmStart, EscalatedSubproblemIterations, SubproblemEps, SubproblemEps);

        if (!st.IsUsable() || _sub.HitIterationLimit)
        {
            TrustRegion = Math.Max(TrustRegionMin, TrustRegion * Shrink);
            LastFailureReason = $"{st} \"{_sub.StatusText}\"" +
                                (_sub.HitIterationLimit ? " [truncated at iteration cap]" : "");
            var failed = new Scvx3DofIteration(index, false, false, double.NaN, TrustRegion,
                _sigGBar, _sigBBar, 0, double.NaN, _jRef, _sub.Iterations, sw.Elapsed.TotalMilliseconds, -1, -1);
            _trace.Add(failed);
            return failed;
        }

        double[] z = _sub.ReadSolution();
        double[] x = _sub.SliceX(z), u = _sub.SliceU(z), wv = _sub.SliceWv(z);
        double sigG = _sub.SigmaGlideOf(z), sigB = _sub.SigmaBurnOf(z);

        // Predicted reduction prices the defect by the virtual control, actual by the true nonlinear defect; both carry the same fuel, smoothing and slack terms or the ratio means nothing.
        double jLin = Fuel(x) + Smoothing(u) + _sub.TerminalSlackCost(z) + _sub.PathSlackCost(z)
                    + _cfg.RhoVc * SumSquaresScaled(wv);
        double jTrue = TrueCost(x, u, sigG, sigB, out double defectNorm, out int defectChannel, out int defectNode);

        double predicted = _jRef - jLin;
        double actual = _jRef - jTrue;
        double rho = Math.Abs(predicted) > 1e-9 ? actual / predicted : (actual >= 0 ? 1.0 : -1.0);

        double dX = MaxNormalisedDiff(x, _xbar, NX, _xs);
        double dU = MaxNormalisedDiff(u, _ubar, NU, null);
        double dSigma = Math.Max(Math.Abs(sigG - _sigGBar), Math.Abs(sigB - _sigBBar)) / _cfg.SigmaScale;
        double used = Math.Max(Math.Max(dX, dU), dSigma);

        bool accepted = rho > RhoAccept;
        double step = 0.0;
        if (accepted)
        {
            _xbar = x;
            _ubar = u;
            NormaliseAttitudes(_ubar);
            _sigGBar = sigG;
            _sigBBar = sigB;
            _jRef = jTrue;
            step = Math.Max(dX, dSigma);
        }

        if (rho < RhoShrink)
            TrustRegion = Math.Max(TrustRegionMin, TrustRegion * Shrink);
        else if (rho >= RhoGrow && used >= 0.8 * TrustRegion)
            TrustRegion = Math.Min(TrustRegionMax, TrustRegion * Grow);

        var result = new Scvx3DofIteration(index, true, accepted, rho, TrustRegion, _sigGBar, _sigBBar,
            step, defectNorm, _jRef, _sub.Iterations, sw.Elapsed.TotalMilliseconds, defectChannel, defectNode);
        _trace.Add(result);
        return result;
    }

    /// <summary>
    /// Both phases' Jacobians about the reference, and the projection's. The glide's are taken with the engine off and their throttle column cleared, since the glide's throttle is pinned to zero and is not a control of its dynamics.
    /// </summary>
    private void Linearise()
    {
        Span<double> uOff = stackalloc double[NU];
        for (int k = 0; k <= _k; k++)
        {
            _ubar.AsSpan(k * NU, NU).CopyTo(uOff);
            uOff[PointMass3Dof.ITH] = 0.0;
            Span<double> B = _lin.BGlide.AsSpan(k * NX * NU, NX * NU);
            PointMass3Dof.Jacobian(_model, _xbar.AsSpan(k * NX, NX), uOff,
                _lin.F0Glide.AsSpan(k * NX, NX), _lin.AGlide.AsSpan(k * NX * NX, NX * NX), B);
            for (int r = 0; r < NX; r++)
                B[r * NU + PointMass3Dof.ITH] = 0.0;
        }
        for (int k = _k; k < _n; k++)
            PointMass3Dof.Jacobian(_model, _xbar.AsSpan(k * NX, NX), _ubar.AsSpan(k * NU, NU),
                _lin.F0Burn.AsSpan(k * NX, NX), _lin.ABurn.AsSpan(k * NX * NX, NX * NX),
                _lin.BBurn.AsSpan(k * NX * NU, NX * NU));

        double sinMax = _cfg.SinAlphaMax;
        for (int k = 0; k < _n; k++)
        {
            ReadOnlySpan<double> xk = _xbar.AsSpan(k * NX, NX);
            double vx = xk[PointMass3Dof.IV], vy = xk[PointMass3Dof.IV + 1], vz = xk[PointMass3Dof.IV + 2];
            double speed = Math.Sqrt(vx * vx + vy * vy + vz * vz);
            if (!(speed > _cfg.AlphaSpeedThreshold))
            {
                _alphaRadius[k] = 0.0;
                continue;
            }
            double radius = sinMax;
            if (_cfg.QAlphaMax > 0.0)
            {
                double q = _model.DynamicPressure(xk);
                if (q > 0.0) radius = Math.Min(radius, _cfg.QAlphaMax / q);
            }
            _alphaRadius[k] = Math.Max(radius, 1e-6);
            PointMass3Dof.ProjectionJacobian(xk, _ubar.AsSpan(k * NU, NU),
                _lin.P0.AsSpan(k * 3, 3), _lin.DpDv.AsSpan(k * 9, 9), _lin.DpDb.AsSpan(k * 9, 9));
        }
    }

    // ------------------------------------------------------------ merit terms

    /// <summary>
    /// Merit at a candidate, with the TRUE nonlinear dynamics: glide intervals with the engine off, burn intervals with it on. Returns the cost and the worst normalised defect.
    /// </summary>
    public double TrueCost(double[] x, double[] u, double sigG, double sigB,
                           out double defectNorm, out int worstChannel, out int worstNode)
    {
        Span<double> fk = stackalloc double[NX];
        Span<double> fk1 = stackalloc double[NX];
        double sumSq = 0, worst = 0;
        worstChannel = -1;
        worstNode = -1;

        for (int k = 0; k < _n - 1; k++)
        {
            bool glide = k < _k;
            double half = 0.5 * (glide ? _dtauG * sigG : _dtauB * sigB);
            EvalNode(x, u, k, glide, fk);
            EvalNode(x, u, k + 1, glide, fk1);
            for (int i = 0; i < NX; i++)
            {
                double d = x[(k + 1) * NX + i] - x[k * NX + i] - half * (fk[i] + fk1[i]);
                double scaled = d / _xs[i];
                sumSq += scaled * scaled;
                if (Math.Abs(scaled) > worst)
                {
                    worst = Math.Abs(scaled);
                    worstChannel = i;
                    worstNode = k;
                }
            }
        }

        defectNorm = worst;
        return Fuel(x) + Smoothing(u) + TerminalMissCost(x) + PathViolationCost(x, u, sigG, sigB) + _cfg.RhoVc * sumSq;
    }

    /// <summary>
    /// The current reference's defect as one dimensionless number: the worst "times over its own tolerance" any interval and channel reaches, with an allowance that grows linearly beyond the commit window to <paramref name="farSlack"/> at the last interval. See Scvx6DofSolver.WeightedDefect, which this mirrors.
    /// </summary>
    public double WeightedDefect(ReadOnlySpan<double> tol, int commitIntervals, double farSlack,
                                 out int worstChannel, out int worstNode, out double worstRaw)
    {
        worstChannel = -1;
        worstNode = -1;
        worstRaw = double.NaN;
        if (_xbar.Length < _n * NX || tol.Length < NX)
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
            bool glide = k < _k;
            double half = 0.5 * (glide ? _dtauG * _sigGBar : _dtauB * _sigBBar);
            EvalNode(_xbar, _ubar, k, glide, fk);
            EvalNode(_xbar, _ubar, k + 1, glide, fk1);
            double frac = k <= commit ? 0.0 : (k - commit) / span;
            double allowance = 1.0 + frac * (slack - 1.0);
            for (int i = 0; i < NX; i++)
            {
                double d = _xbar[(k + 1) * NX + i] - _xbar[k * NX + i] - half * (fk[i] + fk1[i]);
                double t = tol[i];
                if (!(t > 0.0)) continue;
                double ratio = Math.Abs(d) / (t * allowance);
                if (ratio > worst)
                {
                    worst = ratio;
                    worstChannel = i;
                    worstNode = k;
                    worstRaw = d;
                }
            }
        }
        return worst;
    }

    /// <summary>Node time from the start of the plan, seconds: glide nodes spaced over sigma_glide, burn nodes over sigma_burn.</summary>
    public double NodeTime(int k) => NodeTime(k, _k, _n, _sigGBar, _sigBBar);

    public static double NodeTime(int k, int glideIntervals, int nodes, double sigG, double sigB)
        => k <= glideIntervals
            ? (glideIntervals > 0 ? sigG * k / glideIntervals : 0.0)
            : sigG + sigB * (k - glideIntervals) / (nodes - 1 - glideIntervals);

    private void EvalNode(double[] x, double[] u, int k, bool engineOff, Span<double> f)
    {
        if (!engineOff)
        {
            PointMass3Dof.Eval(_model, x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU), f);
            return;
        }
        Span<double> uOff = stackalloc double[NU];
        u.AsSpan(k * NU, NU).CopyTo(uOff);
        uOff[PointMass3Dof.ITH] = 0.0;
        PointMass3Dof.Eval(_model, x.AsSpan(k * NX, NX), uOff, f);
    }

    private double Fuel(double[] x)
    {
        double mInit = _x0[PointMass3Dof.IM];
        return (mInit - x[(_n - 1) * NX + PointMass3Dof.IM]) / mInit;
    }

    private double Smoothing(double[] u)
    {
        double th = 0;
        for (int k = _k; k < _n - 1; k++)
        {
            double d = u[(k + 1) * NU + PointMass3Dof.ITH] - u[k * NU + PointMass3Dof.ITH];
            th += d * d;
        }
        double att = 0;
        for (int k = 0; k < _n - 1; k++)
            for (int j = 0; j < 3; j++)
            {
                double d = u[(k + 1) * NU + PointMass3Dof.IB + j] - u[k * NU + PointMass3Dof.IB + j];
                att += d * d;
            }
        return _cfg.WThrottleRate * th + _cfg.WAttitudeRate * att;
    }

    /// <summary>
    /// The soft path constraints' penalty measured on the trajectory with the TRUE geometry - the actual angle of attack, the actual airflow direction - where the subproblem's slacks price the linearised one. Zero when the path constraints are hard. The angle-of-attack radius is the candidate's own, from its own speed and dynamic pressure.
    /// </summary>
    private double PathViolationCost(double[] x, double[] u, double sigG, double sigB)
    {
        if (!_cfg.SoftPath) return 0.0;
        double cost = 0.0;
        double sinMax = _cfg.SinAlphaMax;
        for (int k = 0; k < _n; k++)
        {
            ReadOnlySpan<double> xk = x.AsSpan(k * NX, NX);
            double vx = xk[PointMass3Dof.IV], vy = xk[PointMass3Dof.IV + 1], vz = xk[PointMass3Dof.IV + 2];
            double bx = u[k * NU], by = u[k * NU + 1], bz = u[k * NU + 2];
            double speed = Math.Sqrt(vx * vx + vy * vy + vz * vz);
            if (speed > _cfg.AlphaSpeedThreshold)
            {
                double radius = sinMax;
                if (_cfg.QAlphaMax > 0.0)
                {
                    double q = _model.DynamicPressure(xk);
                    if (q > 0.0) radius = Math.Min(radius, _cfg.QAlphaMax / q);
                }
                double vm = Math.Sqrt(speed * speed + PointMass3Dof.VEps2);
                double bv = (bx * vx + by * vy + bz * vz) / vm;
                double px = bx - bv * vx / vm, py = by - bv * vy / vm, pz = bz - bv * vz / vm;
                cost += Math.Max(0.0, Math.Sqrt(px * px + py * py + pz * pz) - radius);
                if (_cfg.Retrograde)
                    cost += Math.Max(0.0, (bx * vx + by * vy + bz * vz) / speed);
            }
            if (k > 0)
                for (int i = 0; i < NX; i++)
                {
                    cost += Math.Max(0.0, (_cfg.StateMin[i] - xk[i]) / _xs[i]);
                    cost += Math.Max(0.0, (xk[i] - _cfg.StateMax[i]) / _xs[i]);
                }
            if (k < _n - 1 && _cfg.AttitudeRateMax > 0.0)
            {
                double dt = k < _k ? sigG * _dtauG : sigB * _dtauB;
                double dx = u[(k + 1) * NU] - bx, dy = u[(k + 1) * NU + 1] - by, dz = u[(k + 1) * NU + 2] - bz;
                cost += Math.Max(0.0, Math.Sqrt(dx * dx + dy * dy + dz * dz) - _cfg.AttitudeRateMax * dt);
            }
        }
        if (_cfg.AttitudeAnchor && _anchorCos > -1.0)
            cost += Math.Max(0.0, _anchorCos - (_anchor[0] * u[0] + _anchor[1] * u[1] + _anchor[2] * u[2]));
        return _cfg.PathSlackWeight * cost;
    }

    /// <summary>The terminal-miss penalty measured on the trajectory, which is what the subproblem's slacks equal at its solution.</summary>
    private double TerminalMissCost(double[] x)
    {
        double cost = 0;
        int last = (_n - 1) * NX;
        if (_cfg.TerminalMissWeight > 0.0)
            for (int i = 0; i < 3; i++)
                cost += _cfg.TerminalMissWeight * Math.Abs(x[last + i] - _xf[i]) / _xs[i];
        if (_cfg.TerminalSpeedWeight > 0.0)
            for (int i = 3; i < 6; i++)
                cost += _cfg.TerminalSpeedWeight * Math.Abs(x[last + i] - _xf[i]) / _xs[i];
        return cost;
    }

    private double SumSquaresScaled(double[] wv)
    {
        double s = 0;
        for (int i = 0; i < wv.Length; i++)
        {
            double d = wv[i] / _xs[i % NX];
            s += d * d;
        }
        return s;
    }

    private static double MaxNormalisedDiff(double[] a, double[] b, int stride, double[]? scale)
    {
        double worst = 0;
        for (int i = 0; i < a.Length; i++)
            worst = Math.Max(worst, Math.Abs(a[i] - b[i]) / (scale?[i % stride] ?? 1.0));
        return worst;
    }

    /// <summary>Re-project every node's body axis onto the unit sphere.</summary>
    private void NormaliseAttitudes(double[] u)
    {
        for (int k = 0; k < _n; k++)
        {
            int o = k * NU + PointMass3Dof.IB;
            double len = Math.Sqrt(u[o] * u[o] + u[o + 1] * u[o + 1] + u[o + 2] * u[o + 2]);
            if (len < 1e-12) continue;
            for (int j = 0; j < 3; j++) u[o + j] /= len;
        }
    }
}

using AdvancedFlightComputer.Guidance.Conic;

namespace AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// The SCvx subproblem in SCS's native form:
///     minimize    (1/2) x'Px + c'x
///     subject to  Ax + s = b,  s in K = zero(z) x nonneg(l) x SOC(q...)
///
/// This formulation uses a quadratic objective directly. The three
/// sum_squares penalties go into P, so the problem has no epigraph variables or
/// epigraph cones.
///
/// Formulated for a solver that takes a quadratic objective directly. The
/// This keeps the problem small, with n = N*NX + N*NU + (N-1)*NX + 1, and
/// every SOC is a genuine conic constraint. The constraints are N gimbal cones,
/// N tilt cones, and the glideslope cones when enabled.
///
/// SCS stacks equality, inequality and SOC rows into ONE matrix A (its zero
/// cone is rows [0,z), positive orthant is [z,z+l), SOC blocks follow in q
/// order) rather than ECOS's separate (A,b)/(G,h) pair, so the equality and
/// cone assembly below write into a single CcsAssembler at different row
/// offsets instead of two.
///
/// P convention, confirmed from scs_matrix.c's accum_by_p: supply the literal
/// upper triangle (row &lt;= col) of the true symmetric P, i.e. standard
/// `objective = (1/2) x'Px + c'x` with no extra doubling beyond the normal QP
/// expansion (a term w*z^2 contributes P_ii = 2w; a combined cross term w*zi*zj
/// contributes P_ij = w, i&lt;j).
/// </summary>
public sealed class Scvx6DofSubproblemScs : IDisposable
{
    private const int NX = Dynamics6Dof.NX;
    /// <summary>The control count, the base controls and the craft's fins (Scvx6DofConfig.NU).</summary>
    private readonly int NU;

    private readonly Scvx6DofConfig _cfg;
    private readonly int _n;
    private readonly double _dtau;
    private readonly double[] _xs, _us;

    private readonly int _oX, _oU, _oW, _iSig, _oGs, _oVz, _oTm, _oAc, _oRr, _iSigC, _oGf, _oSv, _oTip, _nVars;

    // The coast before ignition (Scvx6DofConfig.CoastIntervals): its interval count, and each phase's share of normalised time per interval.
    private readonly int _nCoast;
    private readonly double _dtauC, _dtauB;
    private readonly int _nGs, _nVz, _nTm, _nTmPos, _nTmVel, _nTmAtt, _nTmRate, _nAc, _nRr, _nGf, _nSv, _nAnchor, _nTip;
    private readonly int _nEq, _lDim, _nCone;   // _nCone = total rows = nEq + lDim + sum(soc)
    private readonly int[] _socDims;

    private readonly CcsAssembler _A;   // combined: [equalities; positive orthant; SOC]
    private readonly CcsAssembler _P;   // upper triangular, n x n
    private readonly double[] _b, _c;
    private readonly double[] _colScale;

    // The conic backend (ScvxConic.Backend): SCS or Clarabel, one workspace each, the same problem data for both.
    // Real-time 6-DOF warm updates use 3 Ruiz equilibration iterations on Clarabel.
    private readonly ScvxConicWorkspaces _wss = new(clarabelEquilibrateMaxIter: 3);
    /// <summary>The workspace of the last solve, whose solution and statistics the properties below read.</summary>
    private IScvxConicWorkspace _ws => _wss.Last;
    private double _sigCBar;
    private double[]? _aJ, _bJ, _f0J;
    private bool _frozen;
    private int _row;   // cone-row cursor
    private readonly int _nSlew;
    private readonly int _nPin;   // TorqueDemand: two rows per node for each of the gimbals' lateral thrusts and roll torque, pinning them at zero

    public int VariableCount => _nVars;
    public int EqualityCount => _nEq;
    public int RowCount => _nCone;
    public int Iterations => _ws.Iterations;
    public double PrimalObjective => _ws.PrimalObjective;
    public string StatusText => _ws.StatusText;
    public bool HitIterationLimit => _ws.HitIterationLimit;
    public bool HasWarmStart => _wss.Current.HasWarmStart;

    /// <summary>The iteration cap the last solve backend turns a requested budget into (IScvxConicWorkspace.EffectiveIterationCap).</summary>
    public int EffectiveIterationCap(int requested) => _ws.EffectiveIterationCap(requested);

    public void Dispose() => _wss.Dispose();

    public int IX(int node, int i) => _oX + node * NX + i;
    public int IU(int node, int j) => _oU + node * NU + j;
    public int IW(int interval, int i) => _oW + interval * NX + i;
    public int ISigma => _iSig;

    /// <summary>Slack index for path node <paramref name="k"/>, which is TRAJECTORY node k+1.</summary>
    public int IGs(int k) => _oGs + k;
    public int IGf(int k) => _oGf + k;
    public int ISv(int k) => _oSv + k;
    public int IVz(int k) => _oVz + k;

    public Scvx6DofSubproblemScs(Scvx6DofConfig cfg)
    {
        _cfg = cfg;
        NU = cfg.NU;
        _n = cfg.Nodes;
        _dtau = 1.0 / (_n - 1);
        _xs = cfg.XScale;
        _us = cfg.ResolvedUScale;
        _nCoast = Math.Max(cfg.CoastIntervals, 0);
        if (_nCoast > _n - 2)
            throw new ArgumentException($"a coast of {_nCoast} intervals leaves no burn in {_n} nodes");
        _dtauC = _nCoast > 0 ? 1.0 / _nCoast : 0.0;
        _dtauB = 1.0 / (_n - 1 - _nCoast);

        // Path constraints apply from node 1 onward, NEVER at node 0 - see Scvx6DofConfig.GlideSlopeWeight for why that is a correctness requirement rather than a tidiness choice.
        // Zero when disabled, so a config without them assembles exactly the problem it did before.
        _nGs = cfg.GlideSlopeEnabled ? _n - 1 : 0;
        _nVz = cfg.VzLimitEnabled ? _n - 1 : 0;

        _oX = 0;
        _oU = _oX + _n * NX;
        _oW = _oU + _n * NU;
        _iSig = _oW + (_n - 1) * NX;
        _oGs = _iSig + 1;             // glideslope slacks, one per constrained node
        _oVz = _oGs + _nGs;           // climb-rate slacks, likewise
        // Terminal miss: a POSITIVE and a NEGATIVE slack per position axis, so the miss can go either way while both parts stay non-negative and the penalty stays linear.
        // Six variables in total, or none when the terminal is hard.
        _oTm = _oVz + _nVz;
        // Six per softened block: a positive and a negative slack per axis, so the miss may go either way while both stay non-negative and the penalty linear.
        _nTmPos = cfg.TerminalMissWeight > 0.0 ? 6 : 0;
        _nTmVel = cfg.TerminalSpeedWeight > 0.0 ? 6 : 0;
        // The end attitude (qx, qy) and pitch and yaw rate (wx, wy), soft when priced (Scvx6DofConfig.TerminalAttitudeWeight): a positive and a negative slack each, after the speed's.
        _nTmAtt = cfg.TerminalAttitudeWeight > 0.0 ? 4 : 0;
        _nTmRate = cfg.TerminalRateWeight > 0.0 ? 4 : 0;
        _nTm = _nTmPos + _nTmVel + _nTmAtt + _nTmRate;
        // The acceleration limit's slacks, one per node and LAST, so every earlier block keeps its offset.
        _nAc = cfg.AccelLimitEnabled ? _n : 0;
        _oAc = _oTm + _nTm;
        // The turn-rate limit's slacks, one per node from node 1, LAST again (Scvx6DofConfig.RateMaxRadS).
        _nRr = cfg.RateLimitEnabled ? _n - 1 : 0;
        _oRr = _oAc + _nAc;
        // The coast's duration, LAST of all (Scvx6DofConfig.CoastIntervals); -1 without a coast.
        _iSigC = _nCoast > 0 ? _oRr + _nRr : -1;
        _nVars = _oRr + _nRr + (_nCoast > 0 ? 1 : 0);
        // THE PROGRESSIVE GLIDESLOPE'S OWN SLACK, one per glideslope node, after everything else so every earlier block keeps its offset (Scvx6DofConfig.GlideFunnelWeight): the funnel's narrowing beyond the plain cone is relaxed by this cheap slack, at most the gap between the two, and only past that by the glideslope's own.
        _nGf = cfg.GlideSlopeProgressive ? _nGs : 0;
        _oGf = _nVars;
        _nVars += _nGf;
        // THE SIDEWAYS-SPEED CONE's slacks, one per node from node 1, after the funnel's (Scvx6DofConfig.SideSpeedPerM).
        _nSv = cfg.SideSpeedEnabled ? _n - 1 : 0;
        // The first gimbal command's anchor (Scvx6DofConfig.FirstGimbalAnchor): four rows always, so the pattern never changes between fills; loose when there is no anchor. None in TorqueDemand, whose lateral thrust is pinned at zero.
        _nAnchor = cfg.TorqueDemand ? 0 : 4;
        _oSv = _nVars;
        _nVars += _nSv;
        // THE TIP-OVER LINE (Scvx6DofConfig.TipLimit): |v|, |w| and |q_xy| at the last node and the line's slack, after everything else.
        _nTip = cfg.TipLimit ? 4 : 0;
        _oTip = _nVars;
        _nVars += _nTip;

        // With the terminal roll free, three terminal rows go (qw, qz, wz) and the last node gets a tangent-plane row.
        _nEq = NX + (NX - 1) + (_n - 1) * NX + (_n - 2) - (cfg.TerminalRollFree ? 2 : 0);
        // FIVE orthant rows per node, not six.
        // The sixth was the LINEARISED tilt constraint; it is now an exact second-order cone in the block below, so the row is gone rather than merely changed.
        // See AssembleCone's tilt block. + _nVz climb-rate rows, + _nGs + _nVz slack non-negativity rows + _nTm non-negativity rows for the terminal-miss slacks
        // + four rows per interval for the gimbal's slew rate, the last of the orthant rows.
        _nSlew = cfg.GimbalSlewPerS > 0.0 ? 4 * (_n - 1) : 0;
        // + two rows per node for each RCS control's bounds, 0 <= u <= its authority, the last orthant rows of all.
        _nPin = cfg.TorqueDemand ? 6 * _n : 0;
        // + two rows per node for each fin's range, and two per interval for its rate, after the RCS's.
        _lDim = _n * 5 + 2 * _n * NX + 2 * _n * NU + 4 + _nVz + _nGs + _nVz + _nTm + 2 * _nAc + _nSlew
                + 2 * _n * Dynamics6Dof.NRCS + 2 * _n * _cfg.FinCount + 2 * (_n - 1) * _cfg.FinCount
                + _nPin + _nRr + (_nCoast > 0 ? 4 : 0) + 2 * _nGf + _nSv + _nAnchor + (_nTip > 0 ? 2 : 0);
        _socDims = new int[_n + _nGs + _n + _nRr + _nSv + (_nTip > 0 ? 3 : 0)];
        for (int k = 0; k < _n; k++) _socDims[k] = 3;             // gimbal cone per node
        for (int k = 0; k < _nGs; k++) _socDims[_n + k] = 3;      // glideslope cone per node
        // Tilt cones LAST, so adding them leaves every pre-existing block at the row offset it already had - anything reading a cone index keeps its meaning.
        for (int k = 0; k < _n; k++) _socDims[_n + _nGs + k] = 3; // tilt cone per node
        // ...and the turn-rate cones after them, for the same reason.
        for (int k = 0; k < _nRr; k++) _socDims[_n + _nGs + _n + k] = 1 + cfg.RateLimitComponents; // |w| (or its pitch and yaw) per node from node 1
        // ...and the sideways-speed cones after those, for the same reason.
        for (int k = 0; k < _nSv; k++) _socDims[_n + _nGs + _n + _nRr + k] = 3;
        for (int k = 0; k < (_nTip > 0 ? 3 : 0); k++) _socDims[_n + _nGs + _n + _nRr + _nSv + k] = 3;
        _nCone = _nEq + _lDim + _socDims.Sum();

        _A = new CcsAssembler(_nCone, _nVars);
        _P = new CcsAssembler(_nVars, _nVars);
        _b = new double[_nCone];
        _c = new double[_nVars];

        // Same physical-unit column scaling as the ECOS port, and for the same reason: raw SI is not solvable, confirmed by three-way validation there. x = diag(scale) x~; linear coefficients pick up one factor of scale (the column's), quadratic (P) coefficients pick up two (row AND column), since x'Px = (S x~)'P(S x~) = x~'(S'PS)x~.
        _colScale = new double[_nVars];
        for (int k = 0; k < _n; k++)
        {
            for (int i = 0; i < NX; i++) _colScale[IX(k, i)] = _xs[i];
            for (int j = 0; j < NU; j++) _colScale[IU(k, j)] = _us[j];
        }
        for (int k = 0; k < _n - 1; k++)
            for (int i = 0; i < NX; i++) _colScale[IW(k, i)] = _xs[i];
        _colScale[_iSig] = cfg.SigmaScale;
        // Slacks carry the units of what they relax, so they take the same scale as the quantity being violated: metres of horizontal miss, m/s of climb.
        for (int k = 0; k < _nGs; k++) _colScale[IGs(k)] = _xs[Dynamics6Dof.IR];
        for (int k = 0; k < _nGf; k++) _colScale[IGf(k)] = _xs[Dynamics6Dof.IR];
        for (int k = 0; k < _nSv; k++) _colScale[ISv(k)] = _xs[Dynamics6Dof.IV];
        if (_nTip > 0)
        {
            _colScale[_oTip] = _xs[Dynamics6Dof.IV];
            _colScale[_oTip + 1] = _xs[Dynamics6Dof.IW];
            _colScale[_oTip + 2] = _xs[Dynamics6Dof.IQ + 1];
            _colScale[_oTip + 3] = 0.1;
        }
        for (int k = 0; k < _nVz; k++) _colScale[IVz(k)] = _xs[Dynamics6Dof.IV];
        for (int i = 0; i < _nTmPos; i++) _colScale[_oTm + i] = _xs[Dynamics6Dof.IR];
        for (int i = 0; i < _nTmVel; i++) _colScale[_oTm + _nTmPos + i] = _xs[Dynamics6Dof.IV];
        for (int i = 0; i < _nTmAtt; i++) _colScale[_oTm + _nTmPos + _nTmVel + i] = _xs[Dynamics6Dof.IQ + 1];
        for (int i = 0; i < _nTmRate; i++) _colScale[_oTm + _nTmPos + _nTmVel + _nTmAtt + i] = _xs[Dynamics6Dof.IW];
        for (int k = 0; k < _nAc; k++) _colScale[_oAc + k] = _us[Dynamics6Dof.IT];
        for (int k = 0; k < _nRr; k++) _colScale[_oRr + k] = _xs[Dynamics6Dof.IW];
        if (_iSigC >= 0) _colScale[_iSigC] = cfg.SigmaScale;
    }

    /// <summary>The coast's duration variable, or -1 without a coast.</summary>
    public int ISigmaCoast => _iSigC;

    /// <summary>Whether interval k (node k to k+1) is in the coast.</summary>
    private bool CoastInterval(int k) => k < _nCoast;

    /// <summary>Half the normalised time step of interval k, and which duration variable scales it.</summary>
    private (double Half, int Sigma) Step(int k) => CoastInterval(k) ? (0.5 * _dtauC, _iSigC) : (0.5 * _dtauB, _iSig);

    private void AddA(int row, int col, double value) => _A.Add(row, col, value * _colScale[col]);
    private void AddP(int row, int col, double value) =>
        _P.Add(row, col, value * _colScale[row] * _colScale[col]);

    /// <summary>
    /// Fill (first call) or refill (later calls) from a reference trajectory and
    /// linearisation. Unlike the ECOS port there is no persistent native
    /// workspace to reuse across calls - scs_init deep-copies its input and
    /// scs_update only refreshes b/c, never A or P - so every solve pays a fresh
    /// scs_init. What DOES carry across calls is the solution itself: ScsWorkspace
    /// keeps the previous x/y/s and feeds it back as the ADMM starting iterate
    /// when warmStart is requested.
    /// </summary>
    public void Assemble(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf,
                         double[] xbar, double[] ubar, double sigBar, double tr,
                         double[] A, double[] B, double[] f0,
                         double sigCBar = 0.0, double[]? aJ = null, double[]? bJ = null, double[]? f0J = null)
    {
        if (_nCoast > 0 && (aJ == null || bJ == null || f0J == null))
            throw new ArgumentException("a coast needs the ignition node's engine-off linearisation");
        _sigCBar = sigCBar;
        _aJ = aJ; _bJ = bJ; _f0J = f0J;
        if (_frozen) { _A.BeginRefill(); _P.BeginRefill(); }
        Array.Clear(_b);
        Array.Clear(_c);

        AssembleObjective(x0[Dynamics6Dof.IM], xbar, sigBar);
        AssembleEqualities(x0, xf, xbar, ubar, sigBar, A, B, f0);
        AssembleCone(x0, xf, xbar, ubar, sigBar, tr);

        if (!_frozen) { _A.Freeze(); _P.Freeze(); _frozen = true; }
        else { _A.EndRefill(); _P.EndRefill(); }

        // Same row equilibration as the ECOS port, and for the same reason: column scaling to physical units is not enough.
        // Here the symptom was different - not a numerical breakdown but ADMM refusing to converge (100k iterations, still 5% off the reference) - but first-order methods are, if anything, MORE sensitive to row-scale disparity than an interior-point method, so the same fix applies.
        // SCS's own `normalize` does its own equilibration internally, but evidently doesn't fully absorb a problem this poorly scaled to start with.
        EquilibrateRows();
    }

    private void EquilibrateRows()
    {
        var rowMax = new double[_nCone];
        int[] jc = _A.ColumnPointers, ir = _A.RowIndices;
        double[] pr = _A.Values;
        for (int col = 0; col < _A.Cols; col++)
            for (int k = jc[col]; k < jc[col + 1]; k++)
            {
                double a = Math.Abs(pr[k]);
                if (a > rowMax[ir[k]]) rowMax[ir[k]] = a;
            }

        // Every row of a second-order cone must share one scale - scaling them independently would deform the cone rather than just rescale it.
        // The gimbal block sits at the tail of the combined matrix, after the equality and positive-orthant rows.
        int off = _nEq + _lDim;
        foreach (int d in _socDims)
        {
            double blockMax = 0;
            for (int i = off; i < off + d; i++) blockMax = Math.Max(blockMax, rowMax[i]);
            for (int i = off; i < off + d; i++) rowMax[i] = blockMax;
            off += d;
        }

        // Bound the row norms before inverting, exactly as SCS's own Ruiz equilibration does (apply_limit in scs_matrix.c, "need to bound to 1 for rows of all zeros, otherwise blows up").
        //
        // HISTORY, because this looks like defensive padding and was not.
        // The tilt constraint used to be an orthant row linearising R22 >= cos(tilt_max) about the reference quaternion, with gradient (-4*qx, -4*qy) - EXACTLY ZERO for a perfectly vertical booster and tiny for a nearly-vertical one, which is the whole flight - against a constant term of O(0.13).
        // Dividing that row by a ~1e-3 norm amplified the right-hand side by a thousand and wrecked the conditioning of an otherwise well-scaled problem.
        // Unbounded, it made every SCvx iteration past the first fail to converge in 100k ADMM iterations at any trust-region size - and shrinking the trust region did not help, which is what gave it away.
        //
        // That row no longer exists: the tilt limit is an exact SOC now (see AssembleCone), whose radius row is genuinely empty but takes its block's shared scale from the two quaternion rows beside it, so it never reaches the clamp.
        // The clamp stays because SCS applies the same one for the same reason, and a future row that vanishes should degrade rather than explode.
        // Only the LOWER bound is applied.
        // SCS pairs it with an upper clamp of 1e4, but that is inside its own Ruiz iteration on already-scaled data; imposing it on raw rows here is actively harmful - the mass trust-region rows legitimately have norm ~2.5e5 (Xscale for mass), and clamping leaves them 25x unnormalised, which broke the FIRST iteration that had previously converged.
        // Large rows are exactly the ones equilibration exists to fix; only vanishing ones need protecting from inversion.
        const double MinRowNorm = 1e-4;
        var inv = new double[_nCone];
        for (int i = 0; i < _nCone; i++)
            inv[i] = rowMax[i] < MinRowNorm ? 1.0 : 1.0 / rowMax[i];

        for (int col = 0; col < _A.Cols; col++)
            for (int k = jc[col]; k < jc[col + 1]; k++)
                pr[k] *= inv[ir[k]];
        for (int i = 0; i < _nCone; i++)
            _b[i] *= inv[i];
    }

    public ScsStatus Run(bool warmStart, bool verbose = false,
                         int maxIterations = ScsWorkspace.DefaultMaxIterations,
                         double epsAbs = ScsWorkspace.DefaultEps,
                         double epsRel = ScsWorkspace.DefaultEps)
    {
        ThrowIfNotFinite();
        return _wss.ForSolve().Solve(_A, _b, _c, _P, _nEq, _lDim, _socDims, warmStart, verbose,
                         maxIterations, epsAbs, epsRel);
    }

    /// <summary>
    /// Refuse to hand a non-finite problem to SCS.
    ///
    /// SCS is native and does no input validation worth relying on: a NaN or Inf
    /// anywhere in A, P, b or c gets consumed silently and comes back as nonsense, a
    /// NULL workspace with no readable diagnostic, or - as seen in flight - takes the
    /// whole PROCESS down. A managed exception naming the offending entry is
    /// enormously more useful than a game crash, and the check is O(nnz) against a
    /// solve that runs thousands of ADMM iterations, so it costs nothing measurable.
    ///
    /// Non-finite values here always mean bad INPUT DATA rather than a solver fault:
    /// a zero or negative inertia, a degenerate scale, a NaN in the linearisation
    /// because the reference trajectory contains one. Naming which array and index is
    /// what makes that traceable.
    /// </summary>
    private void ThrowIfNotFinite()
    {
        static int FirstBad(double[] v)
        {
            for (int i = 0; i < v.Length; i++)
                if (!double.IsFinite(v[i]))
                    return i;
            return -1;
        }

        int bad;
        if ((bad = FirstBad(_A.Values)) >= 0)
            throw new InvalidOperationException($"constraint matrix A has non-finite value at nz {bad} ({_A.Values[bad]})");
        if (_P.NonZeros > 0 && (bad = FirstBad(_P.Values)) >= 0)
            throw new InvalidOperationException($"objective matrix P has non-finite value at nz {bad} ({_P.Values[bad]})");
        if ((bad = FirstBad(_b)) >= 0)
            throw new InvalidOperationException($"constraint vector b has non-finite value at row {bad} ({_b[bad]})");
        if ((bad = FirstBad(_c)) >= 0)
            throw new InvalidOperationException($"objective vector c has non-finite value at column {bad} ({_c[bad]})");
    }

    public void ResetWarmStart() => _wss.ResetWarmStart();

    // ---------------------------------------------------------------- solution

    public double[] Solution
    {
        get
        {
            if (_ws.X.Length != _nVars)
                throw new InvalidOperationException(
                    "no solution yet - call Assemble then Run before reading the solution");
            var x = new double[_nVars];
            for (int i = 0; i < _nVars; i++) x[i] = _ws.X[i] * _colScale[i];
            return x;
        }
    }

    public double[] SolutionX => Slice(_oX, _n * NX);
    public double[] SolutionU => Slice(_oU, _n * NU);
    public double[] SolutionWv => Slice(_oW, (_n - 1) * NX);
    public double SolutionSigma => Solution[_iSig];
    public double SolutionCoastSigma => _iSigC >= 0 ? Solution[_iSigC] : 0.0;
    private double[] Slice(int offset, int count) => Solution[offset..(offset + count)];

    internal double AccelerationSlackCost => SlackCost(_oAc, _nAc);

    /// <summary>The turn-rate slacks' cost in the last solution, as the objective pays it (Scvx6DofConfig.RateWeight).</summary>
    internal double RateSlackCost => SlackCost(_oRr, _nRr);

    /// <summary>The glideslope, climb-rate and terminal-miss slacks' cost in the last solution, as the objective pays it.</summary>
    internal double PathSlackCost => SlackCost(_oGs, _nGs) + SlackCost(_oGf, _nGf) + SlackCost(_oSv, _nSv) + SlackCost(_oVz, _nVz) + SlackCost(_oTm, _nTm);

    private double SlackCost(int offset, int count)
    {
        double cost = 0.0;
        for (int k = 0; k < count; k++)
            cost += _c[offset + k] * _ws.X[offset + k];
        return cost;
    }

    /// <summary>How far outside the glideslope a craft handed over outside it starts, m: entry_0 (AssembleCone's glideslope block).</summary>
    internal static double GlideSlopeEntryAllowance(Scvx6DofConfig cfg, ReadOnlySpan<double> x0, ReadOnlySpan<double> xf)
    {
        double entryX = x0[Dynamics6Dof.IR] - xf[Dynamics6Dof.IR];
        double entryY = x0[Dynamics6Dof.IR + 1] - xf[Dynamics6Dof.IR + 1];
        double entryHeight = x0[Dynamics6Dof.IR + 2] - xf[Dynamics6Dof.IR + 2];
        return Math.Max(0.0, Math.Sqrt(entryX * entryX + entryY * entryY) - cfg.GlideSlopeRadiusAt(entryHeight));
    }

    /// <summary>The glideslope's widening at trajectory node <paramref name="node"/> of <paramref name="n"/>, m: entry_0 ((1-s)^2 + s (1-s)^4), s the node's fraction of the horizon.</summary>
    internal static double GlideSlopeAllowanceAt(double entryAllowance, int node, int n)
    {
        double progress = (double)node / (n - 1);
        double remaining = 1.0 - progress;
        double remainingSquared = remaining * remaining;
        return entryAllowance * (remainingSquared + progress * remainingSquared * remainingSquared);
    }

    // ---------------------------------------------------------------- objective

    private void AssembleObjective(double mInit, double[] xbar, double sigBar)
    {
        _c[IX(_n - 1, Dynamics6Dof.IM)] = -1.0 / mInit * _colScale[IX(_n - 1, Dynamics6Dof.IM)];

        // WDu * sum_k,j ((U[k+1,j]-U[k,j])/Uscale[j])^2
        for (int k = 0; k < _n - 1; k++)
            for (int j = 0; j < NU; j++)
            {
                // The engine's controls step at ignition, so their smoothing skips that interval.
                if (IsIgnitionStep(k, j))
                    continue;
                double w = _cfg.WDu / (_us[j] * _us[j]);
                AddP(IU(k, j), IU(k, j), 2.0 * w);
                AddP(IU(k + 1, j), IU(k + 1, j), 2.0 * w);
                AddP(IU(k, j), IU(k + 1, j), -2.0 * w);   // row < col: IU(k,j) < IU(k+1,j)
            }

        // RcsWeight * sum_k,j (u_rcs[k,j] / authority_j)^2 - pure diagonal (Scvx6DofConfig.RcsWeight).
        if (_cfg.RcsWeight > 0.0)
            for (int k = 0; k < _n; k++)
                for (int j = 0; j < Dynamics6Dof.NRCS; j++)
                {
                    int col = IU(k, Dynamics6Dof.IRCS + j);
                    double s = _us[Dynamics6Dof.IRCS + j];
                    AddP(col, col, 2.0 * _cfg.RcsWeight / (s * s));
                }

        // FinWeight * sum_k,i (d[k,i] / range_i)^2 - pure diagonal (Scvx6DofConfig.FinWeight).
        if (_cfg.FinWeight > 0.0)
            for (int k = 0; k < _n; k++)
                for (int i = 0; i < _cfg.FinCount; i++)
                {
                    int col = IU(k, Dynamics6Dof.IFIN + i);
                    double s = _us[Dynamics6Dof.IFIN + i];
                    AddP(col, col, 2.0 * _cfg.FinWeight / (s * s));
                }

        // WW * sum_k ||w[k]||^2 - pure diagonal, no coupling between nodes
        for (int k = 0; k < _n; k++)
            for (int i = 0; i < 3; i++)
                AddP(IX(k, Dynamics6Dof.IW + i), IX(k, Dynamics6Dof.IW + i), 2.0 * (_cfg.WW + CoastRateWeightAt(_cfg, _nCoast, k)));

        // Proximal term: ProximalWeight * sum_k ||(X[k]-Xbar[k])/Xscale||^2.
        // Expanding gives a diagonal P contribution 2*w/xs^2 and a LINEAR term -2*w*xbar/xs^2; the constant xbar'xbar is dropped as it cannot change the argmin.
        // Conditions P exactly as WW did but centred on the reference, so it biases nothing and is zero at convergence.
        if (_cfg.ProximalWeight > 0.0)
            for (int k = 0; k < _n; k++)
                for (int i = 0; i < NX; i++)
                {
                    double w = _cfg.ProximalWeight / (_xs[i] * _xs[i]);
                    int col = IX(k, i);
                    AddP(col, col, 2.0 * w);
                    _c[col] += -2.0 * w * xbar[k * NX + i] * _colScale[col];
                }

        // RhoVc * sum_k ||Wv[k]/VirtualControlScale||^2 - pure diagonal. The scale is XScale unless the config prices a channel on its own; see Scvx6DofConfig.VirtualControlScale.
        //  Per second (Scvx6DofConfig.VirtualControlRefIntervalS): each interval's price times its length factor, at the reference burn time, as Scvx6DofSolver's merit prices it.
        for (int k = 0; k < _n - 1; k++)
            for (int i = 0; i < NX; i++)
            {
                double vs = _cfg.VirtualControlScaleOf(i, k);
                double w = _cfg.RhoVc / (vs * vs) * _cfg.VirtualControlIntervalWeight(CoastInterval(k) ? 0.0 : _dtauB * sigBar);
                AddP(IW(k, i), IW(k, i), 2.0 * w);
            }

        // Path-constraint slacks: LINEAR (L1) penalties, not quadratic.
        // An L1 penalty is EXACT - above a finite weight the solution is identical to the hard-constrained one, so the slack sits at zero whenever the corridor is reachable and only opens when the alternative is having no plan at all.
        // A quadratic penalty would instead always trade a little violation for a little objective, quietly flying just outside the cone forever.
        //
        // The penalty applies to the NORMALISED slack (violation / XScale), which is what makes the weight dimensionless and comparable with the rest of the objective - the fuel term is -m_final/m_init, so everything here is order
        // 1. Penalising the RAW slack instead puts a coefficient of weight * XScale = 1e6 next to terms of order 1e-2, and SCS does not merely solve that slowly: it returns "unbounded", because a cost that lopsided makes the dual infeasible to working precision. Dividing by the same scale the column already carries keeps the two in step even if that scale changes.
        for (int k = 0; k < _nGs; k++)
            _c[IGs(k)] += _cfg.GlideSlopeWeight * _colScale[IGs(k)] / _xs[Dynamics6Dof.IR];
        for (int k = 0; k < _nGf; k++)
            _c[IGf(k)] += _cfg.GlideFunnelWeight * _colScale[IGf(k)] / Scvx6DofConfig.GlideFunnelPriceScaleM;
        for (int k = 0; k < _nSv; k++)
            _c[ISv(k)] += _cfg.SideSpeedWeight * _colScale[ISv(k)] / Scvx6DofConfig.SideSpeedPriceScaleMs;
        // The tip-over line's slack, a degree of it at TipWeight (Scvx6DofConfig.TipLimit).
        if (_nTip > 0)
            _c[_oTip + 3] += _cfg.TipWeight * _colScale[_oTip + 3] / (Math.PI / 180.0);
        // THE LAST SECONDS: tilt and pitch-yaw rate, squared, at each node's share (Scvx6DofConfig.TerminalPhaseShare) - pure diagonal. Every node gets its entries, zero outside the window: the pattern is frozen after the first fill, and the window moves as the plan shortens.
        if (_cfg.TerminalPhaseS > 0.0)
            for (int k = 0; k < _n; k++)
            {
                double share = Math.Max(_cfg.TerminalPhaseShare(k, _n, _nCoast, sigBar, _sigCBar), 0.0);
                double wq = share * _cfg.TerminalPhaseTiltWeight / (Scvx6DofConfig.PhaseTiltScaleQuat * Scvx6DofConfig.PhaseTiltScaleQuat);
                double ww = share * _cfg.TerminalPhaseRateWeight / (Scvx6DofConfig.PhaseRateScaleRadS * Scvx6DofConfig.PhaseRateScaleRadS);
                for (int a = 1; a <= 2; a++)
                    AddP(IX(k, Dynamics6Dof.IQ + a), IX(k, Dynamics6Dof.IQ + a), 2.0 * wq);
                for (int a = 0; a < 2; a++)
                    AddP(IX(k, Dynamics6Dof.IW + a), IX(k, Dynamics6Dof.IW + a), 2.0 * ww);
            }
        for (int k = 0; k < _nVz; k++)
            _c[IVz(k)] += _cfg.VzWeight * _colScale[IVz(k)] / _xs[Dynamics6Dof.IV];
        // The height axis (slacks 4 and 5) at TerminalHeightMissFactor times the miss across.
        for (int i = 0; i < _nTmPos; i++)
            _c[_oTm + i] += _cfg.TerminalMissWeight * (i >= 4 ? _cfg.TerminalHeightMissFactor : 1.0) * _colScale[_oTm + i] / _cfg.TerminalMissPriceScale;
        for (int i = 0; i < _nTmAtt; i++)
            _c[_oTm + _nTmPos + _nTmVel + i] += _cfg.TerminalAttitudeWeight * _colScale[_oTm + _nTmPos + _nTmVel + i] / Scvx6DofConfig.TerminalAttitudePriceQuat;
        for (int i = 0; i < _nTmRate; i++)
            _c[_oTm + _nTmPos + _nTmVel + _nTmAtt + i] += _cfg.TerminalRateWeight * _colScale[_oTm + _nTmPos + _nTmVel + _nTmAtt + i] / Scvx6DofConfig.TerminalRatePriceRadS;
        for (int i = 0; i < _nTmVel; i++)
            _c[_oTm + _nTmPos + i] +=
                _cfg.TerminalSpeedWeight * (i < 4 ? _cfg.TerminalSideSpeedFactor : 1.0) * _colScale[_oTm + _nTmPos + i] / _cfg.TerminalSpeedPriceScale;
        for (int k = 0; k < _nAc; k++)
            _c[_oAc + k] += _cfg.AccelWeight * _colScale[_oAc + k] / _us[Dynamics6Dof.IT];
        for (int k = 0; k < _nRr; k++)
            _c[_oRr + k] += _cfg.RateWeight * _colScale[_oRr + k] / _xs[Dynamics6Dof.IW];
    }

    /// <summary>Whether control j's change over interval k is the ignition's step: the engine's controls (not the RCS) over the last coast interval.</summary>
    internal bool IsIgnitionStep(int k, int j) => IsIgnitionStep(_nCoast, k, j);

    /// <summary>The coast's rate price at node k (Scvx6DofConfig.CoastRateWeight): on the coast's nodes after the measured one and before ignition, 0 elsewhere.</summary>
    internal static double CoastRateWeightAt(Scvx6DofConfig cfg, int coastIntervals, int k)
        => cfg.CoastRateWeight > 0.0 && k >= 1 && k < coastIntervals ? cfg.CoastRateWeight : 0.0;

    internal static bool IsIgnitionStep(int coastIntervals, int k, int j)
        => coastIntervals > 0 && k == coastIntervals - 1 && j < Dynamics6Dof.IRCS;

    // -------------------------------------------------------------- equalities

    // Identical construction to the deleted ECOS port's AssembleEqualities - same trapezoidal, time-dilated, virtual-controlled dynamics, same interior-only quaternion tangent plane (nodes 0 and N-1 are pinned outright, so including them there is a linearly dependent row; that redundancy was diagnosed against ECOS but is a property of the constraint set, not the solver, so it applies here too).
    private void AssembleEqualities(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf,
                                    double[] xbar, double[] ubar, double sigBar,
                                    double[] A, double[] B, double[] f0)
    {
        int row = 0;

        for (int i = 0; i < NX; i++)
        {
            AddA(row, IX(0, i), 1.0);
            _b[row++] = x0[i];
        }
        for (int i = 0; i < NX - 1; i++)
        {
            // A craft with no roll torque ends upright with no pitch or yaw rate, at whatever heading and roll rate it has by then.
            if (_cfg.TerminalRollFree && i is Dynamics6Dof.IQ or Dynamics6Dof.IQ + 3 or Dynamics6Dof.IW + 2)
                continue;
            AddA(row, IX(_n - 1, i), 1.0);
            // Terminal POSITION may miss, if softened: X[n-1] - (s+ - s-) = xf, with both slacks non-negative and penalised.
            // Velocity and attitude stay hard
            // - arriving at rest and upright is the part that must not be traded.
            if (_nTmPos > 0 && i >= Dynamics6Dof.IR && i < Dynamics6Dof.IR + 3)
            {
                int axis = i - Dynamics6Dof.IR;
                AddA(row, _oTm + 2 * axis, -1.0);
                AddA(row, _oTm + 2 * axis + 1, 1.0);
            }
            else if (_nTmVel > 0 && i >= Dynamics6Dof.IV && i < Dynamics6Dof.IV + 3)
            {
                int axis = i - Dynamics6Dof.IV;
                AddA(row, _oTm + _nTmPos + 2 * axis, -1.0);
                AddA(row, _oTm + _nTmPos + 2 * axis + 1, 1.0);
            }
            // The end's tilt and pitch and yaw rate may miss too, when priced (Scvx6DofConfig.TerminalAttitudeWeight).
            else if (_nTmAtt > 0 && (i == Dynamics6Dof.IQ + 1 || i == Dynamics6Dof.IQ + 2))
            {
                int axis = i - Dynamics6Dof.IQ - 1;
                AddA(row, _oTm + _nTmPos + _nTmVel + 2 * axis, -1.0);
                AddA(row, _oTm + _nTmPos + _nTmVel + 2 * axis + 1, 1.0);
            }
            else if (_nTmRate > 0 && (i == Dynamics6Dof.IW || i == Dynamics6Dof.IW + 1))
            {
                int axis = i - Dynamics6Dof.IW;
                AddA(row, _oTm + _nTmPos + _nTmVel + _nTmAtt + 2 * axis, -1.0);
                AddA(row, _oTm + _nTmPos + _nTmVel + _nTmAtt + 2 * axis + 1, 1.0);
            }
            _b[row++] = xf[i];
        }

        for (int k = 0; k < _n - 1; k++)
        {
            // Each phase has its own time step and duration. The last coast interval ends at the ignition node with the engine still off, so its far end takes that node's engine-off linearisation (aJ, bJ, f0J) rather than the burn's.
            (double half, int sigCol) = Step(k);
            double sig = CoastInterval(k) ? _sigCBar : sigBar;
            bool toIgnition = _nCoast > 0 && k == _nCoast - 1;
            double[] a1 = toIgnition ? _aJ! : A, b1 = toIgnition ? _bJ! : B, f1 = toIgnition ? _f0J! : f0;
            int aK = k * NX * NX, aK1 = toIgnition ? 0 : (k + 1) * NX * NX;
            int bK = k * NX * NU, bK1 = toIgnition ? 0 : (k + 1) * NX * NU;
            int fK1 = toIgnition ? 0 : (k + 1) * NX;
            for (int i = 0; i < NX; i++)
            {
                int r = row + i;
                for (int j = 0; j < NX; j++)
                {
                    double v = -half * sig * A[aK + i * NX + j];
                    if (i == j) v -= 1.0;
                    AddA(r, IX(k, j), v);
                }
                for (int j = 0; j < NX; j++)
                {
                    double v = -half * sig * a1[aK1 + i * NX + j];
                    if (i == j) v += 1.0;
                    AddA(r, IX(k + 1, j), v);
                }
                for (int j = 0; j < NU; j++)
                    AddA(r, IU(k, j), -half * sig * B[bK + i * NU + j]);
                for (int j = 0; j < NU; j++)
                    AddA(r, IU(k + 1, j), -half * sig * b1[bK1 + i * NU + j]);
                AddA(r, sigCol, -half * (f0[k * NX + i] + f1[fK1 + i]));
                AddA(r, IW(k, i), -1.0);

                double rhs = 0;
                for (int j = 0; j < NX; j++)
                    rhs += A[aK + i * NX + j] * xbar[k * NX + j]
                         + a1[aK1 + i * NX + j] * xbar[(k + 1) * NX + j];
                for (int j = 0; j < NU; j++)
                    rhs += B[bK + i * NU + j] * ubar[k * NU + j]
                         + b1[bK1 + i * NU + j] * ubar[(k + 1) * NU + j];
                _b[r] = -half * sig * rhs;
            }
            row += NX;
        }

        // The last node is pinned outright unless its roll is free. Then two of its quaternion components are variables, and it takes the row as well.
        for (int k = 1; k < (_cfg.TerminalRollFree ? _n : _n - 1); k++)
        {
            for (int i = 0; i < 4; i++)
                AddA(row, IX(k, Dynamics6Dof.IQ + i), xbar[k * NX + Dynamics6Dof.IQ + i]);
            _b[row++] = 1.0;
        }

        if (row != _nEq)
            throw new InvalidOperationException($"equality rows: emitted {row}, expected {_nEq}");
    }

    // ------------------------------------------------------------------- cone

    // Same positive-orthant rows as the ECOS port's AssembleCone, minus the three epigraph cones (now handled by P instead), written starting at row offset _nEq since SCS stacks equalities and cone rows into one matrix.
    private void AssembleCone(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf, double[] xbar, double[] ubar,
                              double sigBar, double tr)
    {
        _row = _nEq;
        ref int row = ref _row;

        for (int k = 0; k < _n; k++)
        {
            // In the coast the engine is off: no thrust, and so (by the gimbal cone, and the roll rows when the roll follows the thrust) no gimbal or engine roll either.
            bool coast = k < _nCoast;
            AddA(row, IU(k, Dynamics6Dof.IT), -1.0);
            _b[row++] = coast ? 0.0 : -_cfg.Tmin;
            AddA(row, IU(k, Dynamics6Dof.IT), 1.0);
            _b[row++] = coast ? 0.0 : _cfg.Tmax;
            // |tau_roll| <= TauRollMax, or <= (TauRollMax / Tmax) T when the roll comes from the gimbals; the thrust column is always present so the pattern does not depend on the mode.
            double rollPerNewton = _cfg.RollTorqueFollowsThrust && _cfg.Tmax > 0.0 ? _cfg.TauRollMax / _cfg.Tmax : 0.0;
            AddA(row, IU(k, Dynamics6Dof.ITAU), 1.0);
            AddA(row, IU(k, Dynamics6Dof.IT), -rollPerNewton);
            _b[row++] = rollPerNewton > 0.0 ? 0.0 : _cfg.TauRollMax;
            AddA(row, IU(k, Dynamics6Dof.ITAU), -1.0);
            AddA(row, IU(k, Dynamics6Dof.IT), -rollPerNewton);
            _b[row++] = rollPerNewton > 0.0 ? 0.0 : _cfg.TauRollMax;

            AddA(row, IX(k, 2), -1.0);
            _b[row++] = -_cfg.GroundFloor;
        }

        for (int k = 0; k < _n; k++)
            for (int i = 0; i < NX; i++)
            {
                double lim = tr * _xs[i], @ref = xbar[k * NX + i];
                AddA(row, IX(k, i), 1.0);
                _b[row++] = lim + @ref;
                AddA(row, IX(k, i), -1.0);
                _b[row++] = lim - @ref;
            }

        for (int k = 0; k < _n; k++)
            for (int j = 0; j < NU; j++)
            {
                double lim = tr * _us[j], @ref = ubar[k * NU + j];
                AddA(row, IU(k, j), 1.0);
                _b[row++] = lim + @ref;
                AddA(row, IU(k, j), -1.0);
                _b[row++] = lim - @ref;
            }

        // CLIMB RATE, from node 1 onward: v_z - d_k <= VzMax, d_k >= 0.
        //
        // Node 0 is excluded on purpose.
        // It is pinned by an equality to the measured state, so constraining it constrains a value the solver cannot change - and a vehicle that happens to be moving upward at that instant (a gust, a wobble, the pitch-over after ignition) would make the whole problem infeasible rather than merely expensive.
        for (int k = 0; k < _nVz; k++)
        {
            AddA(row, IX(k + 1, Dynamics6Dof.IV + 2), 1.0);
            AddA(row, IVz(k), -1.0);
            _b[row++] = _cfg.VzMax;
        }

        // Slack non-negativity for both path constraints: -slack <= 0.
        for (int k = 0; k < _nGs; k++)
        {
            AddA(row, IGs(k), -1.0);
            _b[row++] = 0.0;
        }
        for (int k = 0; k < _nVz; k++)
        {
            AddA(row, IVz(k), -1.0);
            _b[row++] = 0.0;
        }
        for (int i = 0; i < _nTm; i++)
        {
            AddA(row, _oTm + i, -1.0);
            _b[row++] = 0.0;
        }

        // ACCELERATION LIMIT at every node: T_k - AccelMax m_k - s_k <= 0, s_k >= 0. Node 0 included - its thrust is free even though its mass is pinned, and the slack keeps it feasible whatever the mass.
        for (int k = 0; k < _nAc; k++)
        {
            AddA(row, IU(k, Dynamics6Dof.IT), 1.0);
            AddA(row, IX(k, Dynamics6Dof.IM), -_cfg.AccelMax);
            AddA(row, _oAc + k, -1.0);
            _b[row++] = 0.0;
            AddA(row, _oAc + k, -1.0);
            _b[row++] = 0.0;
        }

        double sigLim = tr * _cfg.SigmaScale;
        AddA(row, _iSig, 1.0);
        _b[row++] = sigLim + sigBar;
        AddA(row, _iSig, -1.0);
        _b[row++] = sigLim - sigBar;
        AddA(row, _iSig, -1.0);
        _b[row++] = -_cfg.SigmaMin;
        AddA(row, _iSig, 1.0);
        _b[row++] = _cfg.SigmaMax;

        // The coast's duration: inside its trust region and its bounds, or pinned (Scvx6DofConfig.CoastSigmaFixed).
        if (_iSigC >= 0)
        {
            double lo = _cfg.CoastSigmaFixed ?? _cfg.CoastSigmaMin, hi = _cfg.CoastSigmaFixed ?? _cfg.CoastSigmaMax;
            // A pinned duration is reached whatever the trust region, so a reference a little off it is not made infeasible.
            double lim = _cfg.CoastSigmaFixed.HasValue ? Math.Max(sigLim, Math.Abs(_sigCBar - lo)) : sigLim;
            AddA(row, _iSigC, 1.0);
            _b[row++] = lim + _sigCBar;
            AddA(row, _iSigC, -1.0);
            _b[row++] = lim - _sigCBar;
            AddA(row, _iSigC, -1.0);
            _b[row++] = -lo;
            AddA(row, _iSigC, 1.0);
            _b[row++] = hi;
        }

        // THE GIMBAL'S SLEW RATE (Scvx6DofConfig.GimbalSlewPerS), on the gimbal's ANGLE: |td_{k+1}/T_{k+1} - td_k/T_k| <= rate * dt * tan(gimbal) on each lateral axis, linearised in (td, T) about the reference and scaled by the mean reference thrust so the row is in newtons like the rest. Exact in sigma: dt = sigma * dtau. Coast intervals keep the lateral-thrust form below, whose limit is zero there.
        // In TorqueDemand the same limit falls on the pitch and yaw torque: the gimbals' share slews at their rate, |tau_{k+1} - tau_k| <= rate * dt * DemandTorquePerN * (T_k + T_{k+1}) / 2 + what the RCS can jump, from one bound to the other.
        if (_nSlew > 0)
        {
            double perN = _cfg.TorqueDemand ? _cfg.DemandTorquePerN : _cfg.TanGimbal;
            for (int k = 0; k < _n - 1; k++)
                for (int axis = 0; axis < 2; axis++)
                {
                    // In the coast there is no thrust and so no gimbal to slew; at ignition the nozzle may already be where the burn wants it, so the step onto the burn is free.
                    bool coast = CoastInterval(k);
                    double coeff = coast ? 0.0 : 0.5 * _cfg.GimbalSlewPerS * _dtauB * perN;
                    double reach = coeff * sigBar;
                    double tBarSum = coast ? 0.0 : ubar[k * NU + Dynamics6Dof.IT] + ubar[(k + 1) * NU + Dynamics6Dof.IT];
                    double free = _nCoast > 0 && k == _nCoast - 1 ? 2.0 * perN * _cfg.Tmax : 0.0;
                    int j = _cfg.TorqueDemand ? Dynamics6Dof.IRCS + axis : axis == 0 ? Dynamics6Dof.ITDX : Dynamics6Dof.ITDY;
                    (double pos, double neg) = _cfg.TorqueDemand ? _cfg.RcsBounds(axis) : (0.0, 0.0);
                    if (!_cfg.TorqueDemand && !coast)
                    {
                        double floorT = Math.Max(_cfg.Tmin, 1e-3 * _cfg.Tmax);
                        double tb0 = Math.Max(ubar[k * NU + Dynamics6Dof.IT], floorT), tb1 = Math.Max(ubar[(k + 1) * NU + Dynamics6Dof.IT], floorT);
                        double a0 = ubar[k * NU + j] / tb0, a1 = ubar[(k + 1) * NU + j] / tb1;
                        double scale = 0.5 * (tb0 + tb1);
                        double perSig = 2.0 * coeff * scale;
                        for (int sign = 1; sign >= -1; sign -= 2)
                        {
                            AddA(row, IU(k + 1, j), sign * scale / tb1);
                            AddA(row, IU(k + 1, Dynamics6Dof.IT), -sign * scale * a1 / tb1);
                            AddA(row, IU(k, j), -sign * scale / tb0);
                            AddA(row, IU(k, Dynamics6Dof.IT), sign * scale * a0 / tb0);
                            AddA(row, _iSig, -perSig);
                            _b[row++] = -sign * scale * (a1 - a0);
                        }
                        continue;
                    }
                    double rhs = pos + neg + free - coeff * sigBar * tBarSum;
                    for (int sign = 1; sign >= -1; sign -= 2)
                    {
                        AddA(row, IU(k + 1, j), sign);
                        AddA(row, IU(k, j), -sign);
                        AddA(row, IU(k, Dynamics6Dof.IT), -reach);
                        AddA(row, IU(k + 1, Dynamics6Dof.IT), -reach);
                        if (!coast) AddA(row, _iSig, -coeff * tBarSum);
                        _b[row++] = rhs;
                    }
                }
        }

        // RCS TORQUE, each axis's signed control between what the thrusters give the negative way and the positive way (Scvx6DofConfig.RcsBounds). A craft without RCS gets the same rows with zero bounds, so the pattern does not depend on the craft and the controls stay at zero.
        for (int k = 0; k < _n; k++)
            for (int a = 0; a < Dynamics6Dof.NRCS; a++)
            {
                int col = IU(k, Dynamics6Dof.IRCS + a);
                (double pos, double neg) = _cfg.RcsBounds(a);
                // In TorqueDemand the gimbals' share grows with the thrust (Scvx6DofConfig.ControlTorqueBounds): -tau - perN T <= neg, tau - perN T <= pos.
                double perN = !_cfg.TorqueDemand ? 0.0 : a < 2 ? _cfg.DemandTorquePerN : _cfg.DemandRollPerN;
                AddA(row, col, -1.0);
                if (perN > 0.0) AddA(row, IU(k, Dynamics6Dof.IT), -perN);
                _b[row++] = neg;
                AddA(row, col, 1.0);
                if (perN > 0.0) AddA(row, IU(k, Dynamics6Dof.IT), -perN);
                _b[row++] = pos;
            }

        // THE CONTROL SURFACES (Scvx6DofConfig.FinBounds, FinRate): each fin between its stops at every node, min <= d <= max, and its change from one node to the next within its rate times the interval, in the coast as in the burn - a fin works in the air whether the engine burns or not. A slot without a fin gets the same rows with zero bounds and stays at zero.
        for (int k = 0; k < _n; k++)
            for (int i = 0; i < _cfg.FinCount; i++)
            {
                int col = IU(k, Dynamics6Dof.IFIN + i);
                (double lo, double hi) = _cfg.FinBounds(i);
                AddA(row, col, -1.0);
                _b[row++] = -lo;
                AddA(row, col, 1.0);
                _b[row++] = hi;
            }
        for (int k = 0; k < _n - 1; k++)
        {
            bool coast = CoastInterval(k);
            double dtau = coast ? _dtauC : _dtauB;
            int sigCol = coast ? _iSigC : _iSig;
            for (int i = 0; i < _cfg.FinCount; i++)
            {
                (double lo, double hi) = _cfg.FinBounds(i);
                double rate = _cfg.FinRate(i);
                for (int sign = 1; sign >= -1; sign -= 2)
                {
                    AddA(row, IU(k + 1, Dynamics6Dof.IFIN + i), sign);
                    AddA(row, IU(k, Dynamics6Dof.IFIN + i), -sign);
                    if (rate > 0.0)
                    {
                        // Relax fin-rate across a pinned-zero coast while keeping the sigma slot for frozen CCS sparsity.
                        bool zeroCoast = coast && (_cfg.CoastSigmaFixed is <= 0.0 || _cfg.CoastSigmaMax <= 0.0);
                        AddA(row, sigCol, zeroCoast ? 0.0 : -rate * dtau);
                        _b[row++] = zeroCoast ? hi - lo : 0.0;
                    }
                    else
                    {
                        _b[row++] = hi - lo;
                    }
                }
            }
        }

        // The turn-rate slacks are non-negative.
        for (int k = 0; k < _nRr; k++)
        {
            AddA(row, _oRr + k, -1.0);
            _b[row++] = 0.0;
        }

        // TorqueDemand: the gimbals' lateral thrust and roll torque at zero; the control torque carries it all.
        if (_nPin > 0)
            for (int k = 0; k < _n; k++)
                foreach (int j in new[] { Dynamics6Dof.ITDX, Dynamics6Dof.ITDY, Dynamics6Dof.ITAU })
                {
                    AddA(row, IU(k, j), 1.0);
                    _b[row++] = 0.0;
                    AddA(row, IU(k, j), -1.0);
                    _b[row++] = 0.0;
                }

        // THE FUNNEL'S SLACK: 0 <= f_k <= the gap between the plain cone and the funnel at the node's reference height, max(cot h_ref - R(h_ref), 0). A constant per iteration, so the row stays linear and never infeasible, even for a node below the target.
        for (int k = 0; k < _nGf; k++)
        {
            int node = k + 1;
            double hRef = xbar[node * NX + Dynamics6Dof.IR + 2] - xf[Dynamics6Dof.IR + 2];
            AddA(row, IGf(k), -1.0);
            _b[row++] = 0.0;
            AddA(row, IGf(k), 1.0);
            _b[row++] = Math.Max(_cfg.CotGlideSlope * hRef - _cfg.GlideSlopeRadiusAt(hRef), 0.0);
        }

        // THE FIRST GIMBAL COMMAND, near the one being flown: |td_0 - anchor| <= band on each lateral axis (Scvx6DofConfig.FirstGimbalAnchor). Without an anchor the band is four times the largest thrust, which the gimbal cone already bounds well inside.
        if (_nAnchor > 0)
        {
            double[]? anchor = _cfg.FirstGimbalAnchor;
            bool anchored = anchor is { Length: 3 } && double.IsFinite(anchor[0]) && double.IsFinite(anchor[1]) && anchor[2] > 0.0;
            double refX = anchored ? anchor![0] : 0.0, refY = anchored ? anchor![1] : 0.0;
            double band = anchored ? anchor![2] : 4.0 * _cfg.Tmax;
            AddA(row, IU(0, Dynamics6Dof.ITDX), 1.0);
            _b[row++] = refX + band;
            AddA(row, IU(0, Dynamics6Dof.ITDX), -1.0);
            _b[row++] = band - refX;
            AddA(row, IU(0, Dynamics6Dof.ITDY), 1.0);
            _b[row++] = refY + band;
            AddA(row, IU(0, Dynamics6Dof.ITDY), -1.0);
            _b[row++] = band - refY;
        }

        // The sideways-speed slacks are non-negative.
        for (int k = 0; k < _nSv; k++)
        {
            AddA(row, ISv(k), -1.0);
            _b[row++] = 0.0;
        }

        // THE TIP-OVER LINE (Scvx6DofConfig.TipLimit): (m H |v| + I |w|) / K + 2 |q_xy| - s <= alpha at the reference end mass, and s >= 0. Loose (a radian's worth of margin over everything) until the flight has set the numbers.
        if (_nTip > 0)
        {
            double mEnd = xbar[(_n - 1) * NX + Dynamics6Dof.IM];
            bool live = _cfg.TipLine(mEnd, out double k, out double alpha);
            AddA(row, _oTip, live ? mEnd * _cfg.TipHeightM / k : 0.0);
            AddA(row, _oTip + 1, live ? _cfg.TipInertiaKgM2 / k : 0.0);
            AddA(row, _oTip + 2, 2.0);
            AddA(row, _oTip + 3, -1.0);
            _b[row++] = live ? alpha : 10.0;
            AddA(row, _oTip + 3, -1.0);
            _b[row++] = 0.0;
        }

        // In TorqueDemand the same cone bounds the pitch and yaw torque as the mixer can give it together: ||(tau_x, tau_y)|| <= DemandTorquePerN T + the largest disc inside the RCS's box. The gimbals' share is a disc (they tilt one nozzle), so the box rows alone would promise up to 1.4 times the gimbals' torque at its corners.
        for (int k = 0; k < _n; k++)
        {
            if (_cfg.TorqueDemand)
            {
                double disc = double.PositiveInfinity;
                for (int a = 0; a < 2; a++)
                {
                    (double pos, double neg) = _cfg.RcsBounds(a);
                    disc = Math.Min(disc, Math.Min(pos, neg));
                }
                AddA(row, IU(k, Dynamics6Dof.IT), -_cfg.DemandTorquePerN);
                _b[row++] = double.IsFinite(disc) ? disc : 0.0;
                AddA(row, IU(k, Dynamics6Dof.IRCS), -1.0);
                _b[row++] = 0;
                AddA(row, IU(k, Dynamics6Dof.IRCS + 1), -1.0);
                _b[row++] = 0;
                continue;
            }
            // Centred on the deflection that holds the engines' own moment (Scvx6DofConfig.GimbalTrimX/Y): s = (tan T, tdx - x T, tdy - y T).
            AddA(row, IU(k, Dynamics6Dof.IT), -_cfg.TanGimbal);
            _b[row++] = 0;
            AddA(row, IU(k, Dynamics6Dof.ITDX), -1.0);
            // Only a trim that is there: the config is fixed for the solver's life, so the pattern still never changes between fills, and a balanced craft's cone stays exactly the matrix it was (explicit zeros moved SCS onto another path and lost the roll-free landing at -0.092/0.060/0.086 rad/s).
            if (_cfg.GimbalTrimX != 0.0) AddA(row, IU(k, Dynamics6Dof.IT), _cfg.GimbalTrimX);
            _b[row++] = 0;
            AddA(row, IU(k, Dynamics6Dof.ITDY), -1.0);
            if (_cfg.GimbalTrimY != 0.0) AddA(row, IU(k, Dynamics6Dof.IT), _cfg.GimbalTrimY);
            _b[row++] = 0;
        }

        // GLIDESLOPE, from node 1 onward, as a second-order cone:
        //     ||r_xy[k] - target_xy||  <=  cot(angle) * (r_z[k] - target_z) + entry_k + g_k
        //
        // SCS reads a SOC block as s = b - Ax with s[0] >= ||s[1:]||, so the three rows carry the cone's height and its two horizontal offsets.
        // Excluded at node 0 for the same reason as the climb rate, and slackened by g_k for a sharper one: alone, either constraint is survivable, but a vehicle that is both outside the cone and too low can only get back inside by CLIMBING - which the climb-rate row forbids.
        // Hard versions of the two together can trap the vehicle in a region with no feasible exit at all.
        // Soft versions cannot, and the L1 penalty keeps the slack at exactly zero whenever the corridor is actually reachable.
        // THE PROGRESSIVE GLIDESLOPE (Scvx6DofConfig.GlideFunnelHeightsM) at each node is its tangent at the reference's height h_ref there: radius R(h_ref) + R'(h_ref) (h - h_ref), an affine bound like the plain cone's cot x h, which it is when the glideslope is not progressive.
        // ENTRY ALLOWANCE. A vehicle handed over outside the cone starts entry_0 metres outside it, measured from x0 on every solve.
        // entry_k = entry_0 * ((1-s)^2 + s*(1-s)^4) with s the node's fraction of the horizon: the full violation at the entry, zero at the target, and a quartic term that only widens the early nodes, so the corridor closes on the target without asking the first interval to jump inside.
        double entryAllowance = GlideSlopeEntryAllowance(_cfg, x0, xf);
        for (int k = 0; k < _nGs; k++)
        {
            int node = k + 1;
            double hRef = xbar[node * NX + Dynamics6Dof.IR + 2] - xf[Dynamics6Dof.IR + 2];
            double run = _cfg.GlideSlopeRunPerMAt(hRef);
            AddA(row, IX(node, Dynamics6Dof.IR + 2), -run);
            AddA(row, IGs(k), -1.0);
            if (_nGf > 0)
                AddA(row, IGf(k), -1.0);
            double allowance = GlideSlopeAllowanceAt(entryAllowance, node, _n);
            _b[row++] = -run * xf[Dynamics6Dof.IR + 2] + (_cfg.GlideSlopeRadiusAt(hRef) - run * hRef) + allowance;

            AddA(row, IX(node, Dynamics6Dof.IR + 0), -1.0);
            _b[row++] = -xf[Dynamics6Dof.IR + 0];

            AddA(row, IX(node, Dynamics6Dof.IR + 1), -1.0);
            _b[row++] = -xf[Dynamics6Dof.IR + 1];
        }

        // TILT, as an EXACT second-order cone: ||(qx[k], qy[k])|| <= sin(tiltMax/2).
        //
        // WHAT THIS REPLACED.
        // The tilt limit is tilt <= tiltMax with tilt = acos(R22) and R22 = 1 - 2(qx^2 + qy^2), and it used to be enforced by LINEARISING R22 >= cos(tiltMax) about the reference quaternion:
        //
        //     r22  = 1 - 2*(qx^2 + qy^2)                 evaluated at the reference
        //     dqx  = -4*qx,  dqy = -4*qy                 its gradient
        //     row  : -dqx*qx[k] - dqy*qy[k] <= -(cos(tiltMax) - r22 + dqx*qx + dqy*qy)
        //
        // one ORTHANT row per node, rebuilt every SCvx iteration from xbar.
        //
        // WHY THAT FAILED, AND ONLY WHEN VERTICAL.
        // The gradient (-4*qx, -4*qy) is EXACTLY ZERO for a perfectly vertical vehicle and tiny for a nearly vertical one - so the row degenerated to 0 <= 1 - cos(tiltMax), which is trivially true and carries no information about qx or qy at all.
        // To first order the subproblem could not see the tilt limit.
        //
        // Worse than merely blind: R22 is CONCAVE in (qx, qy), so its linearisation OVER-estimates it, and the linearised feasible set is a RELAXATION.
        // The subproblem would tilt to the trust-region bound believing it free, the nonlinear defect check would reject the step, and the next iteration - linearising about a now-tilted reference with a large gradient - would haul it back.
        // That oscillation is why a descent that was already vertical converged far worse than a bellyflop, and why nudging the vehicle a few degrees off vertical fixed it: a non-zero qx, qy restores the gradient.
        // No amount of proximal weight can help, because the fault is in the FEASIBLE SET, not in the conditioning of the objective.
        //
        // WHY THIS FORM IS EXACT.
        // Squaring out the same inequality:
        // 1 - 2(qx^2 + qy^2) >= cos(tiltMax) <=> qx^2 + qy^2 <= (1 - cos(tiltMax))/2 = sin^2(tiltMax/2) which is a norm ball of CONSTANT radius on (qx, qy) - already convex, so it needs no linearisation, has no reference dependence, and is identical at every iteration including one seeded dead vertical.
        //
        // It relies on ||q|| = 1, but so did the expression it replaces: R22 is only
        // 1 - 2(qx^2 + qy^2) for a unit quaternion. The unit-norm equality is itself linearised (see the qbar.q = 1 rows in AssembleEqualities), so this cone is exact to precisely the degree the orthant row already was.
        //
        // SOC rows are s = b - Ax with s[0] >= ||s[1:]||, so s[0] is the constant radius - a row with NO matrix entries at all - and s[1], s[2] are qx, qy.
        // The all-zero row is harmless: EquilibrateRows gives every row of a SOC block the block's shared scale, and rows 1 and 2 carry the quaternion scale, so nothing is divided by a vanishing norm.
        double tiltRadius = _cfg.SinHalfTilt;
        for (int k = 0; k < _n; k++)
        {
            _b[row++] = tiltRadius;

            AddA(row, IX(k, Dynamics6Dof.IQ + 1), -1.0);
            _b[row++] = 0.0;

            AddA(row, IX(k, Dynamics6Dof.IQ + 2), -1.0);
            _b[row++] = 0.0;
        }

        // TURN RATE, from node 1 on: ||w_k|| <= RateMaxRadS + r_k (Scvx6DofConfig.RateMaxRadS). Node 0 is the measured rate, which the plan cannot change.
        for (int k = 0; k < _nRr; k++)
        {
            AddA(row, _oRr + k, -1.0);
            _b[row++] = _cfg.RateMaxRadS;
            for (int i = 0; i < _cfg.RateLimitComponents; i++)
            {
                AddA(row, IX(k + 1, Dynamics6Dof.IW + i), -1.0);
                _b[row++] = 0.0;
            }
        }

        // SIDEWAYS SPEED, from node 1 on: ||v_xy[k] - vf_xy|| <= SideSpeedPerM (r_z[k] - target_z) + SideSpeedFloorMs + s_k (Scvx6DofConfig.SideSpeedPerM). Exactly convex: a second-order cone in the state, no linearisation. Node 0 is the measured velocity.
        for (int k = 0; k < _nSv; k++)
        {
            int node = k + 1;
            AddA(row, IX(node, Dynamics6Dof.IR + 2), -_cfg.SideSpeedPerM);
            AddA(row, ISv(k), -1.0);
            _b[row++] = -_cfg.SideSpeedPerM * xf[Dynamics6Dof.IR + 2] + _cfg.SideSpeedFloorMs;

            AddA(row, IX(node, Dynamics6Dof.IV + 0), -1.0);
            _b[row++] = -xf[Dynamics6Dof.IV + 0];

            AddA(row, IX(node, Dynamics6Dof.IV + 1), -1.0);
            _b[row++] = -xf[Dynamics6Dof.IV + 1];
        }

        // The tip-over line's three cones at the last node: a >= |v_xy|, b >= |w_xy|, c >= |(q_x, q_y)| (Scvx6DofConfig.TipLimit).
        if (_nTip > 0)
        {
            int last = _n - 1;
            (int first, int comp)[] cones = [(_oTip, Dynamics6Dof.IV), (_oTip + 1, Dynamics6Dof.IW), (_oTip + 2, Dynamics6Dof.IQ + 1)];
            foreach ((int first, int comp) in cones)
            {
                AddA(row, first, -1.0);
                _b[row++] = 0.0;
                AddA(row, IX(last, comp), -1.0);
                _b[row++] = 0.0;
                AddA(row, IX(last, comp + 1), -1.0);
                _b[row++] = 0.0;
            }
        }

        if (row != _nCone)
            throw new InvalidOperationException($"cone rows: emitted {row}, expected {_nCone}");
    }

    // ------------------------------------------------------------ audit tools

    /// <summary>
    /// Packs a known SI primal point into scaled coordinates, for auditing an
    /// externally produced solution against this formulation - same purpose and
    /// same technique as the ECOS port's PackPrimal.
    /// </summary>
    public double[] PackPrimal(double[] x, double[] u, double[] wv, double sigma)
    {
        var z = new double[_nVars];
        Array.Copy(x, 0, z, _oX, _n * NX);
        Array.Copy(u, 0, z, _oU, _n * NU);
        Array.Copy(wv, 0, z, _oW, (_n - 1) * NX);
        z[_iSig] = sigma;
        for (int i = 0; i < _nVars; i++) z[i] /= _colScale[i];
        return z;
    }

    /// <summary>
    /// Residuals of a scaled primal point z against the assembled problem: max
    /// |Az - b| over equality rows, and the worst cone violation (negative slack
    /// on the orthant, ||s[1:]||-s[0] on each SOC).
    /// </summary>
    public (double EqResidual, int EqRow, double ConeViolation, int ConeIndex) CheckPrimal(double[] z)
    {
        double[] az = MultiplyCcs(_A, z, _nCone);
        var s = new double[_nCone];
        for (int i = 0; i < _nCone; i++) s[i] = _b[i] - az[i];

        double eqWorst = 0;
        int eqRow = -1;
        for (int i = 0; i < _nEq; i++)
        {
            double r = Math.Abs(s[i]);   // s should be exactly 0 on equality rows
            if (r > eqWorst) { eqWorst = r; eqRow = i; }
        }

        double coneWorst = 0;
        int coneIdx = -1;
        for (int i = _nEq; i < _nEq + _lDim; i++)
            if (-s[i] > coneWorst) { coneWorst = -s[i]; coneIdx = i; }

        int off = _nEq + _lDim;
        foreach (int d in _socDims)
        {
            double tail = 0;
            for (int i = 1; i < d; i++) tail += s[off + i] * s[off + i];
            double v = Math.Sqrt(tail) - s[off];
            if (v > coneWorst) { coneWorst = v; coneIdx = off; }
            off += d;
        }
        return (eqWorst, eqRow, coneWorst, coneIdx);
    }

    /// <summary>
    /// (1/2) z'Pz + c'z at a scaled point - what SCS actually minimises. Compare
    /// against (reference objective - 1), the same dropped-constant convention
    /// as the ECOS port's LinearObjective.
    /// </summary>
    public double Objective(double[] z)
    {
        double linear = 0;
        for (int i = 0; i < _nVars; i++) linear += _c[i] * z[i];

        double quad = 0;
        int[] jc = _P.ColumnPointers, ir = _P.RowIndices;
        double[] pr = _P.Values;
        for (int col = 0; col < _P.Cols; col++)
            for (int kk = jc[col]; kk < jc[col + 1]; kk++)
            {
                int r = ir[kk];
                double contribution = pr[kk] * z[r] * z[col];
                quad += r == col ? contribution : 2.0 * contribution;   // upper-triangle-only storage
            }
        return 0.5 * quad + linear;
    }

    /// <summary>
    /// Replicates SCS's own scs_init validation checks (validate_lin_sys,
    /// validate_cones in the vendored source) against the assembled data, in
    /// managed code. Written because scs_printf's failure messages go through
    /// the native CRT's stdout, which is fully buffered when the process isn't
    /// attached to a real console (piped output, or a process that throws before
    /// the native side gets to flush) - so a validation failure can be
    /// completely silent on the managed side even though SCS "explained itself"
    /// internally. This reproduces the same checks in C# where nothing can eat
    /// the output.
    /// </summary>
    /// <summary>
    /// Finds the structural cause of an "unbounded" result: a variable that the
    /// objective rewards moving in a direction nothing constrains.
    ///
    /// SCvx subproblems are LINEARISED, so they are unbounded by default - the
    /// trust region is what makes them solvable at all. Every variable therefore
    /// needs either a box, a cone, or a quadratic penalty holding it in. A new
    /// variable added without one does not produce a wrong answer, it produces
    /// "unbounded", with nothing to say which of several hundred columns is at
    /// fault. This names it.
    /// </summary>
    public string DiagnoseUnbounded()
    {
        var sb = new System.Text.StringBuilder();
        int[] ajc = _A.ColumnPointers, air = _A.RowIndices;
        double[] apr = _A.Values;
        int[] pjc = _P.ColumnPointers;

        for (int col = 0; col < _nVars; col++)
        {
            bool hasQuadratic = pjc[col + 1] > pjc[col];
            // A column only BOUNDS its variable through rows that can stop it: a nonneg or SOC row.
            // An equality row constrains it relative to others but cannot on its own stop it running off.
            bool bounded = false;
            for (int k = ajc[col]; k < ajc[col + 1]; k++)
                if (air[k] >= _nEq && apr[k] != 0.0) { bounded = true; break; }

            if (!bounded && !hasQuadratic && _c[col] != 0.0)
                sb.AppendLine($"  col {col} ({NameOf(col)}): c={_c[col]:E2}, " +
                              "no inequality/cone row and no P entry - UNBOUNDED RAY");
        }
        return sb.Length == 0 ? "  no structurally unbounded columns\n" : sb.ToString();
    }

    private string NameOf(int col)
    {
        if (col >= _oSv && _nSv > 0) return $"sideways speed slack {col - _oSv}";
        if (col >= _oGf && _nGf > 0) return $"glide funnel slack {col - _oGf}";
        if (col >= _oVz && _nVz > 0) return $"vz slack {col - _oVz}";
        if (col >= _oGs && _nGs > 0) return $"glideslope slack {col - _oGs}";
        if (col == _iSig) return "sigma";
        if (col >= _oW) return $"Wv[{(col - _oW) / NX},{(col - _oW) % NX}]";
        if (col >= _oU) return $"U[{(col - _oU) / NU},{(col - _oU) % NU}]";
        return $"X[{col / NX},{col % NX}]";
    }

    public string DiagnoseScsValidation()
    {
        var sb = new System.Text.StringBuilder();
        int n = _nVars, m = _A.Rows;

        int[] ajc = _A.ColumnPointers, air = _A.RowIndices;
        int anz = ajc[n];
        sb.AppendLine($"A: {n} cols, {m} rows, nnz={anz} (assembler reports {_A.NonZeros})");
        if (anz != _A.NonZeros)
            sb.AppendLine("  MISMATCH: A.ColumnPointers[n] != A.NonZeros - column pointer array is inconsistent");
        if ((double)anz / m > n || anz < 0)
            sb.AppendLine($"  FAIL (validate_lin_sys): Anz/m={((double)anz / m):F3} > n={n}, or Anz<0");
        int rMaxA = 0;
        for (int i = 0; i < anz; i++) rMaxA = Math.Max(rMaxA, air[i]);
        if (rMaxA > m - 1)
            sb.AppendLine($"  FAIL (validate_lin_sys): max row index in A is {rMaxA}, must be <= {m - 1}");
        else
            sb.AppendLine($"  A row indices OK (max {rMaxA} <= {m - 1})");

        int[] pjc = _P.ColumnPointers, pir = _P.RowIndices;
        int pnz = pjc[n];
        sb.AppendLine($"P: {n}x{n}, nnz={pnz} (assembler reports {_P.NonZeros})");
        if (_P.Rows != n || _P.Cols != n)
            sb.AppendLine($"  FAIL: P is not square / dimension mismatch (P is {_P.Rows}x{_P.Cols}, need {n}x{n})");
        bool upperOk = true;
        for (int col = 0; col < n; col++)
            for (int k = pjc[col]; k < pjc[col + 1]; k++)
                if (pir[k] > col)
                {
                    upperOk = false;
                    sb.AppendLine($"  FAIL (validate_lin_sys): P[{pir[k]},{col}] has row > col - not upper triangular");
                }
        if (upperOk) sb.AppendLine("  P upper-triangular OK");

        int fullConeDims = _nEq + _lDim + _socDims.Sum();
        sb.AppendLine($"cones: z={_nEq} l={_lDim} sum(q)={_socDims.Sum()} qsize={_socDims.Length} " +
                      $"-> total {fullConeDims}, m={m}");
        if (fullConeDims != m)
            sb.AppendLine($"  FAIL (validate_cones): cone dims {fullConeDims} != m={m}");
        if (_socDims.Any(d => d < 0))
            sb.AppendLine("  FAIL (validate_cones): a SOC dimension is negative");

        // c and b finiteness - SCS doesn't explicitly validate this, but a NaN or Infinity here is exactly the kind of thing that would otherwise masquerade as an opaque setup failure.
        int badC = Array.FindIndex(_c, v => !double.IsFinite(v));
        int badB = Array.FindIndex(_b, v => !double.IsFinite(v));
        if (badC >= 0) sb.AppendLine($"  WARN: c[{badC}] = {_c[badC]} is not finite");
        if (badB >= 0) sb.AppendLine($"  WARN: b[{badB}] = {_b[badB]} is not finite");
        int badAv = Array.FindIndex(_A.Values, v => !double.IsFinite(v));
        int badPv = Array.FindIndex(_P.Values, v => !double.IsFinite(v));
        if (badAv >= 0) sb.AppendLine($"  WARN: A.Values[{badAv}] = {_A.Values[badAv]} is not finite");
        if (badPv >= 0) sb.AppendLine($"  WARN: P.Values[{badPv}] = {_P.Values[badPv]} is not finite");

        return sb.ToString();
    }

    private static double[] MultiplyCcs(CcsAssembler m, double[] z, int rows)
    {
        var y = new double[rows];
        int[] jc = m.ColumnPointers, ir = m.RowIndices;
        double[] pr = m.Values;
        for (int col = 0; col < m.Cols; col++)
            for (int k = jc[col]; k < jc[col + 1]; k++)
                y[ir[k]] += pr[k] * z[col];
        return y;
    }
}

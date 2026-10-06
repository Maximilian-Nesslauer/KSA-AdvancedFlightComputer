using AdvancedFlightComputer.Guidance.Conic;

namespace AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// The dynamics linearised about a reference: one set per phase, and the angle-of-attack projection.
///
/// Node K, the ignition instant, carries BOTH sets. The last glide interval ends there with the engine off and the first burn interval starts there with it on, so each sees the node through its own phase's dynamics - 3dof.py's gc[K] and gb[K].
/// </summary>
public sealed class Linearisation3Dof
{
    private const int NX = PointMass3Dof.NX;
    private const int NU = PointMass3Dof.NU;

    public readonly double[] F0Glide, AGlide, BGlide;
    public readonly double[] F0Burn, ABurn, BBurn;
    public readonly double[] P0, DpDv, DpDb;

    public Linearisation3Dof(int n)
    {
        F0Glide = new double[n * NX];
        AGlide = new double[n * NX * NX];
        BGlide = new double[n * NX * NU];
        F0Burn = new double[n * NX];
        ABurn = new double[n * NX * NX];
        BBurn = new double[n * NX * NU];
        P0 = new double[n * 3];
        DpDv = new double[n * 9];
        DpDb = new double[n * 9];
    }
}

/// <summary>
/// The glide-and-burn SCvx subproblem in SCS's native form:
///     minimise    (1/2) x'Px + c'x
///     subject to  Ax + s = b,  s in K = zero(z) x nonneg(l) x SOC(q...)
///
/// Constraint for constraint 3dof.py's loop, plus the blocks the script does not have, each off unless its config switch is on:
///
///   x_0 = x0,  r_N-1 = r_f,  v_N-1 = v_f                      boundary (soft terminal: L1 slacks)
///   x_k+1 = x_k + dtau/2 (g_k + g_k+1) + w_k                   collocation, per phase
///   b_bar_k . b_k = 1                                          unit attitude, tangent plane
///   throttle_k = 0 (glide),  floor &lt;= throttle_k &lt;= ceiling (burn)
///   |p_lin_k| &lt;= radius_k,  b_k . v_hat_bar_k &lt;= 0            angle of attack, retrograde
///   state bounds from node 1;  b_N-1 = b_f;  |b_k+1 - b_k| &lt;= rate sigma dtau;  a . b_0 &gt;= cos(theta)
///   trust box on x, u and both sigmas;  sigma bounds
///
/// The path constraints - the cone, the retrograde row, the bounds and the rate cone - take an L1 slack each when the config makes them soft; see Scvx3DofConfig.PathSlackWeight.
///
/// with g_k = sigma f0_k + sigma_bar (A_k dx_k + B_k du_k) in each phase's own sigma and dtau. The glide's B has its throttle column zeroed: the engine is off there, so its throttle is not a control of the glide's dynamics.
///
/// The structure is fixed at construction and refilled in place every iteration, as the 6-DOF subproblem is. Rows that apply only at some nodes - the angle-of-attack cone and the retrograde row below the speed threshold - are written as vacuous rows with explicit zeros rather than left out, so the sparsity pattern never changes.
/// </summary>
public sealed class Scvx3DofSubproblemScs
{
    private const int NX = PointMass3Dof.NX;
    private const int NU = PointMass3Dof.NU;

    private readonly Scvx3DofConfig _cfg;
    private readonly int _n, _k;
    private readonly double _dtauG, _dtauB;
    private double[] _xs => _cfg.XScale;

    private readonly int _oX, _oU, _oW, _iSigB, _iSigG, _oTm, _nTmPos, _nTmVel, _nTm, _nVars;
    private readonly int _oSa, _oSr, _oSb, _oSw, _nSa, _nSr, _nSb, _nSw, _nSlack;
    private readonly int[] _minChannels, _maxChannels;
    private readonly bool _terminalAttitude, _rateLimit;
    private readonly int _nEq, _lDim, _nRows;
    private readonly int[] _socDims;

    private readonly CcsAssembler _A;
    private readonly CcsAssembler _P;
    private readonly double[] _b, _c;
    private readonly double[] _colScale;

    private readonly ScsWorkspace _ws = new();
    private bool _frozen;

    public int VariableCount => _nVars;
    public int RowCount => _nRows;
    public int Iterations => _ws.Iterations;
    public string StatusText => _ws.StatusText;
    public bool HitIterationLimit => _ws.HitIterationLimit;
    public bool HasWarmStart => _ws.HasWarmStart;

    private int IX(int node, int i) => _oX + node * NX + i;
    private int IU(int node, int j) => _oU + node * NU + j;
    private int IW(int interval, int i) => _oW + interval * NX + i;

    public Scvx3DofSubproblemScs(Scvx3DofConfig cfg)
    {
        if (cfg.Nodes < 3) throw new ArgumentException("at least three nodes");
        if (cfg.GlideIntervals < 0 || cfg.BurnIntervals < 1)
            throw new ArgumentException($"glide intervals {cfg.GlideIntervals} leave no burn in {cfg.Nodes} nodes");

        _cfg = cfg;
        _n = cfg.Nodes;
        _k = cfg.GlideIntervals;
        _dtauG = _k > 0 ? 1.0 / _k : 0.0;
        _dtauB = 1.0 / cfg.BurnIntervals;
        _terminalAttitude = cfg.TerminalAttitude != null;
        _rateLimit = cfg.AttitudeRateMax > 0.0;

        _minChannels = Enumerable.Range(0, NX).Where(i => double.IsFinite(cfg.StateMin[i])).ToArray();
        _maxChannels = Enumerable.Range(0, NX).Where(i => double.IsFinite(cfg.StateMax[i])).ToArray();

        _oX = 0;
        _oU = _oX + _n * NX;
        _oW = _oU + _n * NU;
        _iSigB = _oW + (_n - 1) * NX;
        _iSigG = _k > 0 ? _iSigB + 1 : -1;
        _oTm = _iSigB + (_k > 0 ? 2 : 1);
        _nTmPos = cfg.TerminalMissWeight > 0.0 ? 6 : 0;
        _nTmVel = cfg.TerminalSpeedWeight > 0.0 ? 6 : 0;
        _nTm = _nTmPos + _nTmVel;
        // Path slacks, when soft: one per node for the angle-of-attack cone and the retrograde row, one per bounded channel per node from 1, one per interval for the rate cone.
        bool soft = cfg.SoftPath;
        _nSa = soft ? _n : 0;
        _nSr = soft && cfg.Retrograde ? _n : 0;
        _nSb = soft ? (_n - 1) * (_minChannels.Length + _maxChannels.Length) : 0;
        _nSw = soft && _rateLimit ? _n - 1 : 0;
        _nSlack = _nSa + _nSr + _nSb + _nSw;
        _oSa = _oTm + _nTm;
        _oSr = _oSa + _nSa;
        _oSb = _oSr + _nSr;
        _oSw = _oSb + _nSb;
        _nVars = _oSw + _nSw;

        int sigmas = _k > 0 ? 2 : 1;
        _nEq = NX + 6 + (_terminalAttitude ? 3 : 0) + (_n - 1) * NX
             + (_terminalAttitude ? _n - 1 : _n) + _k;
        _lDim = 2 * (_n - _k)
              + (cfg.Retrograde ? _n : 0)
              + (_n - 1) * (_minChannels.Length + _maxChannels.Length)
              + 2 * _n * NX + 2 * _n * NU
              + 4 * sigmas
              + (cfg.AttitudeAnchor ? 1 : 0)
              + _nTm + _nSlack;
        int rateCones = _rateLimit ? _n - 1 : 0;
        _socDims = new int[_n + rateCones];
        Array.Fill(_socDims, 4);
        _nRows = _nEq + _lDim + _socDims.Sum();

        _A = new CcsAssembler(_nRows, _nVars);
        _P = new CcsAssembler(_nVars, _nVars);
        _b = new double[_nRows];
        _c = new double[_nVars];

        _colScale = new double[_nVars];
        RefreshScales();
    }

    /// <summary>
    /// Physical-unit column scaling, as the 6-DOF subproblem: x = diag(scale) x~. The attitude and the throttle are already order one. Re-read from the config on every assembly, so scales the guidance resizes between solves take effect on the next one.
    /// </summary>
    private void RefreshScales()
    {
        Array.Fill(_colScale, 1.0);
        for (int k = 0; k < _n; k++)
            for (int i = 0; i < NX; i++) _colScale[IX(k, i)] = _xs[i];
        for (int k = 0; k < _n - 1; k++)
            for (int i = 0; i < NX; i++) _colScale[IW(k, i)] = _xs[i];
        _colScale[_iSigB] = _cfg.SigmaScale;
        if (_k > 0) _colScale[_iSigG] = _cfg.SigmaScale;
        // Slacks take the scale of what they relax: a positive and a negative part per axis.
        for (int i = 0; i < _nTmPos; i++) _colScale[_oTm + i] = _xs[PointMass3Dof.IR + i / 2];
        for (int i = 0; i < _nTmVel; i++) _colScale[_oTm + _nTmPos + i] = _xs[PointMass3Dof.IV + i / 2];
        int perNode = _minChannels.Length + _maxChannels.Length;
        for (int i = 0; i < _nSb; i++)
        {
            int slot = i % perNode;
            int channel = slot < _minChannels.Length ? _minChannels[slot] : _maxChannels[slot - _minChannels.Length];
            _colScale[_oSb + i] = _xs[channel];
        }
    }

    private int BoundSlack(int node, int slot) => _oSb + (node - 1) * (_minChannels.Length + _maxChannels.Length) + slot;

    private void AddA(int row, int col, double value) => _A.Add(row, col, value * _colScale[col]);
    private void AddP(int row, int col, double value) => _P.Add(row, col, value * _colScale[row] * _colScale[col]);

    /// <summary>
    /// Fill (first call) or refill (later calls) the problem from a reference and its linearisation.
    /// </summary>
    /// <param name="xf">Terminal position and velocity, 6 long.</param>
    /// <param name="alphaRadius">Per node: the bound on |p|, or 0 or less where the angle-of-attack cone and the retrograde row do not apply.</param>
    /// <param name="anchor">Direction node 0's body axis is held near, 3 long; ignored unless the config allocates the row. A zero vector disables it for this solve.</param>
    /// <param name="anchorCos">cos of the cone's half-angle about <paramref name="anchor"/>.</param>
    public void Assemble(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf,
                         double[] xbar, double[] ubar, double sigGBar, double sigBBar, double tr,
                         Linearisation3Dof lin, double[] alphaRadius,
                         ReadOnlySpan<double> anchor, double anchorCos)
    {
        if (_frozen) { _A.BeginRefill(); _P.BeginRefill(); }
        RefreshScales();
        Array.Clear(_b);
        Array.Clear(_c);

        AssembleObjective(x0[PointMass3Dof.IM], xbar);
        int row = AssembleEqualities(x0, xf, xbar, ubar, sigGBar, sigBBar, lin);
        if (row != _nEq)
            throw new InvalidOperationException($"equality rows: emitted {row}, expected {_nEq}");
        row = AssembleCone(row, xbar, ubar, sigGBar, sigBBar, tr, lin, alphaRadius, anchor, anchorCos);
        if (row != _nRows)
            throw new InvalidOperationException($"cone rows: emitted {row}, expected {_nRows}");

        if (!_frozen) { _A.Freeze(); _P.Freeze(); _frozen = true; }
        else { _A.EndRefill(); _P.EndRefill(); }

        EquilibrateRows();
    }

    public ScsStatus Run(bool warmStart, int maxIterations = ScsWorkspace.DefaultMaxIterations,
                         double epsAbs = ScsWorkspace.DefaultEps, double epsRel = ScsWorkspace.DefaultEps)
    {
        ThrowIfNotFinite();
        return _ws.Solve(_A, _b, _c, _P, _nEq, _lDim, _socDims, warmStart, false,
                         maxIterations, epsAbs, epsRel);
    }

    public void ResetWarmStart() => _ws.ResetWarmStart();

    // ---------------------------------------------------------------- solution

    private double[] Solution
    {
        get
        {
            if (_ws.X.Length != _nVars)
                throw new InvalidOperationException("no solution yet - call Assemble then Run first");
            var x = new double[_nVars];
            for (int i = 0; i < _nVars; i++) x[i] = _ws.X[i] * _colScale[i];
            return x;
        }
    }

    /// <summary>The whole solution in SI, one copy. Slice it with the accessors below.</summary>
    public double[] ReadSolution() => Solution;

    public double[] SliceX(double[] z) => z[_oX..(_oX + _n * NX)];
    public double[] SliceU(double[] z) => z[_oU..(_oU + _n * NU)];
    public double[] SliceWv(double[] z) => z[_oW..(_oW + (_n - 1) * NX)];
    public double SigmaBurnOf(double[] z) => z[_iSigB];
    public double SigmaGlideOf(double[] z) => _k > 0 ? z[_iSigG] : 0.0;

    /// <summary>The objective's terminal-miss penalty at a solution, so the merit can price it the same way.</summary>
    public double TerminalSlackCost(double[] z)
    {
        double cost = 0;
        for (int i = 0; i < _nTmPos; i++)
            cost += _cfg.TerminalMissWeight * z[_oTm + i] / _xs[PointMass3Dof.IR + i / 2];
        for (int i = 0; i < _nTmVel; i++)
            cost += _cfg.TerminalSpeedWeight * z[_oTm + _nTmPos + i] / _xs[PointMass3Dof.IV + i / 2];
        return cost;
    }

    // ---------------------------------------------------------------- objective

    private void AssembleObjective(double mInit, double[] xbar)
    {
        _c[IX(_n - 1, PointMass3Dof.IM)] = -1.0 / mInit * _colScale[IX(_n - 1, PointMass3Dof.IM)];

        // Throttle RATE over the burn, not its magnitude: a magnitude penalty pays the optimiser to stretch the burn (3dof.py README section 6).
        for (int k = _k; k < _n - 1; k++)
        {
            double w = _cfg.WThrottleRate;
            int a = IU(k, PointMass3Dof.ITH), b = IU(k + 1, PointMass3Dof.ITH);
            AddP(a, a, 2.0 * w);
            AddP(b, b, 2.0 * w);
            AddP(a, b, -2.0 * w);
        }

        // Attitude rate, over every node.
        for (int k = 0; k < _n - 1; k++)
            for (int j = 0; j < 3; j++)
            {
                double w = _cfg.WAttitudeRate;
                int a = IU(k, PointMass3Dof.IB + j), b = IU(k + 1, PointMass3Dof.IB + j);
                AddP(a, a, 2.0 * w);
                AddP(b, b, 2.0 * w);
                AddP(a, b, -2.0 * w);
            }

        if (_cfg.ProximalWeight > 0.0)
            for (int k = 0; k < _n; k++)
                for (int i = 0; i < NX; i++)
                {
                    double w = _cfg.ProximalWeight / (_xs[i] * _xs[i]);
                    int col = IX(k, i);
                    AddP(col, col, 2.0 * w);
                    _c[col] += -2.0 * w * xbar[k * NX + i] * _colScale[col];
                }

        for (int k = 0; k < _n - 1; k++)
            for (int i = 0; i < NX; i++)
            {
                double w = _cfg.RhoVc / (_xs[i] * _xs[i]);
                AddP(IW(k, i), IW(k, i), 2.0 * w);
            }

        // L1 on the NORMALISED slack, so the weight is dimensionless next to the fuel fraction. See Scvx6DofSubproblemScs.AssembleObjective.
        for (int i = 0; i < _nTmPos; i++)
            _c[_oTm + i] += _cfg.TerminalMissWeight * _colScale[_oTm + i] / _xs[PointMass3Dof.IR + i / 2];
        for (int i = 0; i < _nTmVel; i++)
            _c[_oTm + _nTmPos + i] +=
                _cfg.TerminalSpeedWeight * _colScale[_oTm + _nTmPos + i] / _xs[PointMass3Dof.IV + i / 2];

        // Path slacks, each normalised to its own unit, so one weight prices them all.
        for (int i = 0; i < _nSlack; i++)
            _c[_oSa + i] += _cfg.PathSlackWeight;
    }

    /// <summary>The objective's path-slack penalty at a solution, so the merit can price the same terms.</summary>
    public double PathSlackCost(double[] z)
    {
        double cost = 0;
        for (int i = 0; i < _nSlack; i++)
            cost += z[_oSa + i] / _colScale[_oSa + i];
        return _cfg.PathSlackWeight * cost;
    }

    // -------------------------------------------------------------- equalities

    private int AssembleEqualities(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf,
                                   double[] xbar, double[] ubar, double sigGBar, double sigBBar,
                                   Linearisation3Dof lin)
    {
        int row = 0;

        for (int i = 0; i < NX; i++)
        {
            AddA(row, IX(0, i), 1.0);
            _b[row++] = x0[i];
        }

        // Terminal position and velocity; mass is free. A softened block reads X - (s+ - s-) = xf.
        for (int i = 0; i < 6; i++)
        {
            AddA(row, IX(_n - 1, i), 1.0);
            if (_nTmPos > 0 && i < 3)
            {
                AddA(row, _oTm + 2 * i, -1.0);
                AddA(row, _oTm + 2 * i + 1, 1.0);
            }
            else if (_nTmVel > 0 && i >= 3)
            {
                AddA(row, _oTm + _nTmPos + 2 * (i - 3), -1.0);
                AddA(row, _oTm + _nTmPos + 2 * (i - 3) + 1, 1.0);
            }
            _b[row++] = xf[i];
        }

        if (_terminalAttitude)
            for (int j = 0; j < 3; j++)
            {
                AddA(row, IU(_n - 1, PointMass3Dof.IB + j), 1.0);
                _b[row++] = _cfg.TerminalAttitude![j];
            }

        for (int k = 0; k < _n - 1; k++)
        {
            bool glide = k < _k;
            double[] A = glide ? lin.AGlide : lin.ABurn;
            double[] B = glide ? lin.BGlide : lin.BBurn;
            double[] f0 = glide ? lin.F0Glide : lin.F0Burn;
            double sigBar = glide ? sigGBar : sigBBar;
            double half = 0.5 * (glide ? _dtauG : _dtauB);
            int iSig = glide ? _iSigG : _iSigB;

            int aK = k * NX * NX, aK1 = (k + 1) * NX * NX;
            int bK = k * NX * NU, bK1 = (k + 1) * NX * NU;
            for (int i = 0; i < NX; i++)
            {
                int r = row + i;
                for (int j = 0; j < NX; j++)
                {
                    double v = -half * sigBar * A[aK + i * NX + j];
                    if (i == j) v -= 1.0;
                    AddA(r, IX(k, j), v);
                }
                for (int j = 0; j < NX; j++)
                {
                    double v = -half * sigBar * A[aK1 + i * NX + j];
                    if (i == j) v += 1.0;
                    AddA(r, IX(k + 1, j), v);
                }
                for (int j = 0; j < NU; j++)
                    AddA(r, IU(k, j), -half * sigBar * B[bK + i * NU + j]);
                for (int j = 0; j < NU; j++)
                    AddA(r, IU(k + 1, j), -half * sigBar * B[bK1 + i * NU + j]);
                AddA(r, iSig, -half * (f0[k * NX + i] + f0[(k + 1) * NX + i]));
                AddA(r, IW(k, i), -1.0);

                double rhs = 0;
                for (int j = 0; j < NX; j++)
                    rhs += A[aK + i * NX + j] * xbar[k * NX + j]
                         + A[aK1 + i * NX + j] * xbar[(k + 1) * NX + j];
                for (int j = 0; j < NU; j++)
                    rhs += B[bK + i * NU + j] * ubar[k * NU + j]
                         + B[bK1 + i * NU + j] * ubar[(k + 1) * NU + j];
                _b[r] = -half * sigBar * rhs;
            }
            row += NX;
        }

        // Unit attitude, as the plane tangent to the sphere at the reference; the solver re-projects on accept. Skipped at the last node when the terminal attitude pins it, where it would be a dependent row.
        int tangentNodes = _terminalAttitude ? _n - 1 : _n;
        for (int k = 0; k < tangentNodes; k++)
        {
            for (int j = 0; j < 3; j++)
                AddA(row, IU(k, PointMass3Dof.IB + j), ubar[k * NU + PointMass3Dof.IB + j]);
            _b[row++] = 1.0;
        }

        // The engine is off in the glide.
        for (int k = 0; k < _k; k++)
        {
            AddA(row, IU(k, PointMass3Dof.ITH), 1.0);
            _b[row++] = 0.0;
        }

        return row;
    }

    // ------------------------------------------------------------------- cone

    private int AssembleCone(int row, double[] xbar, double[] ubar, double sigGBar, double sigBBar,
                             double tr, Linearisation3Dof lin, double[] alphaRadius,
                             ReadOnlySpan<double> anchor, double anchorCos)
    {
        // Throttle box over the burn: a scalar interval, convex because the attitude carries the direction (3dof.py README section 2b).
        for (int k = _k; k < _n; k++)
        {
            AddA(row, IU(k, PointMass3Dof.ITH), -1.0);
            _b[row++] = -_cfg.ThrottleFloor;
            AddA(row, IU(k, PointMass3Dof.ITH), 1.0);
            _b[row++] = _cfg.ThrottleCeiling;
        }

        // Retrograde: b_k . v_hat_bar_k <= 0, vacuous below the speed threshold.
        if (_cfg.Retrograde)
            for (int k = 0; k < _n; k++)
            {
                bool active = alphaRadius[k] > 0.0;
                double vx = xbar[k * NX + PointMass3Dof.IV], vy = xbar[k * NX + PointMass3Dof.IV + 1],
                       vz = xbar[k * NX + PointMass3Dof.IV + 2];
                double sp = Math.Sqrt(vx * vx + vy * vy + vz * vz);
                double inv = active && sp > 0.0 ? 1.0 / sp : 0.0;
                AddA(row, IU(k, PointMass3Dof.IB + 0), vx * inv);
                AddA(row, IU(k, PointMass3Dof.IB + 1), vy * inv);
                AddA(row, IU(k, PointMass3Dof.IB + 2), vz * inv);
                if (_nSr > 0) AddA(row, _oSr + k, -1.0);
                _b[row++] = 0.0;
            }

        // State bounds from node 1.
        for (int k = 1; k < _n; k++)
        {
            int slot = 0;
            foreach (int i in _minChannels)
            {
                AddA(row, IX(k, i), -1.0);
                if (_nSb > 0) AddA(row, BoundSlack(k, slot), -1.0);
                slot++;
                _b[row++] = -_cfg.StateMin[i];
            }
            foreach (int i in _maxChannels)
            {
                AddA(row, IX(k, i), 1.0);
                if (_nSb > 0) AddA(row, BoundSlack(k, slot), -1.0);
                slot++;
                _b[row++] = _cfg.StateMax[i];
            }
        }

        // Trust region: a box on every state, control and duration, in scaled units.
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
                double @ref = ubar[k * NU + j];
                AddA(row, IU(k, j), 1.0);
                _b[row++] = tr + @ref;
                AddA(row, IU(k, j), -1.0);
                _b[row++] = tr - @ref;
            }

        row = SigmaRows(row, _iSigB, sigBBar, tr, _cfg.BurnSigmaMin, _cfg.BurnSigmaMax);
        if (_k > 0)
            row = SigmaRows(row, _iSigG, sigGBar, tr, _cfg.GlideSigmaMin, _cfg.GlideSigmaMax);

        // Node 0's body axis within a cone about the anchor: a . b_0 >= cos(theta), exact to the degree |b| = 1 is.
        if (_cfg.AttitudeAnchor)
        {
            bool on = anchor.Length == 3 && (anchor[0] != 0.0 || anchor[1] != 0.0 || anchor[2] != 0.0);
            for (int j = 0; j < 3; j++)
                AddA(row, IU(0, PointMass3Dof.IB + j), on ? -anchor[j] : 0.0);
            _b[row++] = on ? -anchorCos : 1.0;
        }

        for (int i = 0; i < _nTm; i++)
        {
            AddA(row, _oTm + i, -1.0);
            _b[row++] = 0.0;
        }
        for (int i = 0; i < _nSlack; i++)
        {
            AddA(row, _oSa + i, -1.0);
            _b[row++] = 0.0;
        }

        // ANGLE OF ATTACK, as a second-order cone on the linearised projection:
        //     |p0 + dp/dv (v - v_bar) + dp/db (b - b_bar)| <= radius
        // SCS reads s = b - Ax with s[0] >= |s[1:]|: s[0] is the constant radius, a row with no entries, and s[1..3] are p_lin. A node below the speed threshold gets radius 1 and all-zero coefficients, which holds trivially.
        for (int k = 0; k < _n; k++)
        {
            bool active = alphaRadius[k] > 0.0;
            if (_nSa > 0) AddA(row, _oSa + k, -1.0);
            _b[row++] = active ? alphaRadius[k] : 1.0;
            for (int i = 0; i < 3; i++)
            {
                double rhs = 0.0;
                for (int j = 0; j < 3; j++)
                {
                    double jv = active ? lin.DpDv[k * 9 + i * 3 + j] : 0.0;
                    AddA(row, IX(k, PointMass3Dof.IV + j), -jv);
                    rhs -= jv * xbar[k * NX + PointMass3Dof.IV + j];
                }
                for (int j = 0; j < 3; j++)
                {
                    double jb = active ? lin.DpDb[k * 9 + i * 3 + j] : 0.0;
                    AddA(row, IU(k, PointMass3Dof.IB + j), -jb);
                    rhs -= jb * ubar[k * NU + PointMass3Dof.IB + j];
                }
                _b[row++] = active ? lin.P0[k * 3 + i] + rhs : 0.0;
            }
        }

        // ATTITUDE RATE: |b_k+1 - b_k| <= rate * sigma * dtau, exact - sigma sits in the cone's head as the variable it is.
        if (_rateLimit)
            for (int k = 0; k < _n - 1; k++)
            {
                bool glide = k < _k;
                AddA(row, glide ? _iSigG : _iSigB, -_cfg.AttitudeRateMax * (glide ? _dtauG : _dtauB));
                if (_nSw > 0) AddA(row, _oSw + k, -1.0);
                _b[row++] = 0.0;
                for (int j = 0; j < 3; j++)
                {
                    AddA(row, IU(k + 1, PointMass3Dof.IB + j), -1.0);
                    AddA(row, IU(k, PointMass3Dof.IB + j), 1.0);
                    _b[row++] = 0.0;
                }
            }

        return row;
    }

    private int SigmaRows(int row, int col, double sigBar, double tr, double min, double max)
    {
        double lim = tr * _cfg.SigmaScale;
        AddA(row, col, 1.0);
        _b[row++] = lim + sigBar;
        AddA(row, col, -1.0);
        _b[row++] = lim - sigBar;
        AddA(row, col, -1.0);
        _b[row++] = -min;
        AddA(row, col, 1.0);
        _b[row++] = max;
        return row;
    }

    // ------------------------------------------------------------ conditioning

    /// <summary>
    /// Row equilibration, as the 6-DOF subproblem's and for the same reasons: column scaling to physical units is not enough for ADMM, and every row of a second-order cone must share one scale or the cone is deformed rather than rescaled. Rows that vanish - the cones' radius rows and the vacuous rows below the speed threshold - keep scale 1.
    /// </summary>
    private void EquilibrateRows()
    {
        var rowMax = new double[_nRows];
        int[] jc = _A.ColumnPointers, ir = _A.RowIndices;
        double[] pr = _A.Values;
        for (int col = 0; col < _A.Cols; col++)
            for (int k = jc[col]; k < jc[col + 1]; k++)
            {
                double a = Math.Abs(pr[k]);
                if (a > rowMax[ir[k]]) rowMax[ir[k]] = a;
            }

        int off = _nEq + _lDim;
        foreach (int d in _socDims)
        {
            double blockMax = 0;
            for (int i = off; i < off + d; i++) blockMax = Math.Max(blockMax, rowMax[i]);
            for (int i = off; i < off + d; i++) rowMax[i] = blockMax;
            off += d;
        }

        const double MinRowNorm = 1e-4;
        var inv = new double[_nRows];
        for (int i = 0; i < _nRows; i++)
            inv[i] = rowMax[i] < MinRowNorm ? 1.0 : 1.0 / rowMax[i];

        for (int col = 0; col < _A.Cols; col++)
            for (int k = jc[col]; k < jc[col + 1]; k++)
                pr[k] *= inv[ir[k]];
        for (int i = 0; i < _nRows; i++)
            _b[i] *= inv[i];
    }

    /// <summary>A NaN reaching SCS comes back as nonsense or takes the process down; name it here instead. See Scvx6DofSubproblemScs.ThrowIfNotFinite.</summary>
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
}

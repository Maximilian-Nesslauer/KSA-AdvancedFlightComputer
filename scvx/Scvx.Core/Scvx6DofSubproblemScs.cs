namespace Scvx;

/// <summary>
/// The SCvx subproblem in SCS's native form:
///     minimize    (1/2) x'Px + c'x
///     subject to  Ax + s = b,  s in K = zero(z) x nonneg(l) x SOC(q...)
///
/// This problem was first ported to ECOS; that port is DELETED (see git history
/// if the comparison is ever needed) because ECOS minimises a LINEAR objective
/// only, and every reference to "the ECOS port" below is history explaining why
/// this design is as it is, not a pointer to live code.
///
/// Formulated for a solver that takes a quadratic objective directly. The
/// three sum_squares penalties go into P; there are no epigraph variables and
/// no epigraph cones, so this problem is smaller (n = N*NX + N*NU + (N-1)*NX +
/// 1, vs +4 and +3 extra large SOCs for the ECOS port) and the SOC set is just
/// the N genuine gimbal cones — the ONLY nonlinearity that is actually conic
/// rather than quadratic.
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
public sealed class Scvx6DofSubproblemScs
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    private readonly Scvx6DofConfig _cfg;
    private readonly int _n;
    private readonly double _dtau;
    private readonly double[] _xs, _us;

    private readonly int _oX, _oU, _oW, _iSig, _nVars;
    private readonly int _nEq, _lDim, _nCone;   // _nCone = total rows = nEq + lDim + sum(soc)
    private readonly int[] _socDims;

    private readonly CcsAssembler _A;   // combined: [equalities; positive orthant; SOC]
    private readonly CcsAssembler _P;   // upper triangular, n x n
    private readonly double[] _b, _c;
    private readonly double[] _colScale;

    private readonly ScsWorkspace _ws = new();
    private bool _frozen;
    private int _row;   // cone-row cursor

    public int VariableCount => _nVars;
    public int EqualityCount => _nEq;
    public int RowCount => _nCone;
    public int Iterations => _ws.Iterations;
    public double PrimalObjective => _ws.PrimalObjective;
    public string StatusText => _ws.StatusText;
    public bool HitIterationLimit => _ws.HitIterationLimit;
    public bool HasWarmStart => _ws.HasWarmStart;

    public int IX(int node, int i) => _oX + node * NX + i;
    public int IU(int node, int j) => _oU + node * NU + j;
    public int IW(int interval, int i) => _oW + interval * NX + i;
    public int ISigma => _iSig;

    public Scvx6DofSubproblemScs(Scvx6DofConfig cfg)
    {
        _cfg = cfg;
        _n = cfg.Nodes;
        _dtau = 1.0 / (_n - 1);
        _xs = cfg.XScale;
        _us = cfg.ResolvedUScale;

        _oX = 0;
        _oU = _oX + _n * NX;
        _oW = _oU + _n * NU;
        _iSig = _oW + (_n - 1) * NX;
        _nVars = _iSig + 1;

        _nEq = NX + (NX - 1) + (_n - 1) * NX + (_n - 2);
        _lDim = _n * 6 + 2 * _n * NX + 2 * _n * NU + 4;
        _socDims = new int[_n];
        for (int k = 0; k < _n; k++) _socDims[k] = 3;   // gimbal cone per node
        _nCone = _nEq + _lDim + _socDims.Sum();

        _A = new CcsAssembler(_nCone, _nVars);
        _P = new CcsAssembler(_nVars, _nVars);
        _b = new double[_nCone];
        _c = new double[_nVars];

        // Same physical-unit column scaling as the ECOS port, and for the same
        // reason: raw SI is not solvable, confirmed by three-way validation
        // there. x = diag(scale) x~; linear coefficients pick up one factor of
        // scale (the column's), quadratic (P) coefficients pick up two (row AND
        // column), since x'Px = (S x~)'P(S x~) = x~'(S'PS)x~.
        _colScale = new double[_nVars];
        for (int k = 0; k < _n; k++)
        {
            for (int i = 0; i < NX; i++) _colScale[IX(k, i)] = _xs[i];
            for (int j = 0; j < NU; j++) _colScale[IU(k, j)] = _us[j];
        }
        for (int k = 0; k < _n - 1; k++)
            for (int i = 0; i < NX; i++) _colScale[IW(k, i)] = _xs[i];
        _colScale[_iSig] = cfg.SigmaScale;
    }

    private void AddA(int row, int col, double value) => _A.Add(row, col, value * _colScale[col]);
    private void AddP(int row, int col, double value) =>
        _P.Add(row, col, value * _colScale[row] * _colScale[col]);

    /// <summary>
    /// Fill (first call) or refill (later calls) from a reference trajectory and
    /// linearisation. Unlike the ECOS port there is no persistent native
    /// workspace to reuse across calls — scs_init deep-copies its input and
    /// scs_update only refreshes b/c, never A or P — so every solve pays a fresh
    /// scs_init. What DOES carry across calls is the solution itself: ScsWorkspace
    /// keeps the previous x/y/s and feeds it back as the ADMM starting iterate
    /// when warmStart is requested.
    /// </summary>
    public void Assemble(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf,
                         double[] xbar, double[] ubar, double sigBar, double tr,
                         double[] A, double[] B, double[] f0)
    {
        if (_frozen) { _A.BeginRefill(); _P.BeginRefill(); }
        Array.Clear(_b);
        Array.Clear(_c);

        AssembleObjective(x0[Dynamics6Dof.IM], xbar);
        AssembleEqualities(x0, xf, xbar, ubar, sigBar, A, B, f0);
        AssembleCone(xbar, ubar, sigBar, tr);

        if (!_frozen) { _A.Freeze(); _P.Freeze(); _frozen = true; }
        else { _A.EndRefill(); _P.EndRefill(); }

        // Same row equilibration as the ECOS port, and for the same reason:
        // column scaling to physical units is not enough. Here the symptom was
        // different — not a numerical breakdown but ADMM refusing to converge
        // (100k iterations, still 5% off the reference) — but first-order
        // methods are, if anything, MORE sensitive to row-scale disparity than
        // an interior-point method, so the same fix applies. SCS's own
        // `normalize` does its own equilibration internally, but evidently
        // doesn't fully absorb a problem this poorly scaled to start with.
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

        // Every row of a second-order cone must share one scale — scaling them
        // independently would deform the cone rather than just rescale it. The
        // gimbal block sits at the tail of the combined matrix, after the
        // equality and positive-orthant rows.
        int off = _nEq + _lDim;
        foreach (int d in _socDims)
        {
            double blockMax = 0;
            for (int i = off; i < off + d; i++) blockMax = Math.Max(blockMax, rowMax[i]);
            for (int i = off; i < off + d; i++) rowMax[i] = blockMax;
            off += d;
        }

        // Bound the row norms before inverting, exactly as SCS's own Ruiz
        // equilibration does (apply_limit in scs_matrix.c, "need to bound to 1
        // for rows of all zeros, otherwise blows up").
        //
        // This is not defensive padding — it is load-bearing here. The TILT row
        // linearises R22 >= cos(tilt_max) about the reference quaternion, giving
        // gradient (-4*qx, -4*qy), which is EXACTLY ZERO for a perfectly vertical
        // booster and tiny for a nearly-vertical one — which is the whole flight.
        // Its constant term stays O(0.13). Dividing that row by a ~1e-3 norm
        // amplifies the right-hand side by a thousand and wrecks the conditioning
        // of an otherwise well-scaled problem. Unbounded, this made every SCvx
        // iteration past the first fail to converge in 100k ADMM iterations, at
        // any trust-region size — and shrinking the trust region did not help,
        // which is what gave it away: a genuinely too-large step would have got
        // easier as the region shrank.
        // Only the LOWER bound is applied. SCS pairs it with an upper clamp of
        // 1e4, but that is inside its own Ruiz iteration on already-scaled data;
        // imposing it on raw rows here is actively harmful — the mass trust-region
        // rows legitimately have norm ~2.5e5 (Xscale for mass), and clamping
        // leaves them 25x unnormalised, which broke the FIRST iteration that had
        // previously converged. Large rows are exactly the ones equilibration
        // exists to fix; only vanishing ones need protecting from inversion.
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
                         double epsRel = ScsWorkspace.DefaultEps) =>
        _ws.Solve(_A, _b, _c, _P, _nEq, _lDim, _socDims, warmStart, verbose, maxIterations, epsAbs, epsRel);

    public void ResetWarmStart() => _ws.ResetWarmStart();

    // ---------------------------------------------------------------- solution

    public double[] Solution
    {
        get
        {
            if (_ws.X.Length != _nVars)
                throw new InvalidOperationException(
                    "no solution yet — call Assemble then Run before reading the solution");
            var x = new double[_nVars];
            for (int i = 0; i < _nVars; i++) x[i] = _ws.X[i] * _colScale[i];
            return x;
        }
    }

    public double[] SolutionX => Slice(_oX, _n * NX);
    public double[] SolutionU => Slice(_oU, _n * NU);
    public double[] SolutionWv => Slice(_oW, (_n - 1) * NX);
    public double SolutionSigma => Solution[_iSig];
    private double[] Slice(int offset, int count) => Solution[offset..(offset + count)];

    // ---------------------------------------------------------------- objective

    private void AssembleObjective(double mInit, double[] xbar)
    {
        _c[IX(_n - 1, Dynamics6Dof.IM)] = -1.0 / mInit * _colScale[IX(_n - 1, Dynamics6Dof.IM)];

        // WDu * sum_k,j ((U[k+1,j]-U[k,j])/Uscale[j])^2
        for (int k = 0; k < _n - 1; k++)
            for (int j = 0; j < NU; j++)
            {
                double w = _cfg.WDu / (_us[j] * _us[j]);
                AddP(IU(k, j), IU(k, j), 2.0 * w);
                AddP(IU(k + 1, j), IU(k + 1, j), 2.0 * w);
                AddP(IU(k, j), IU(k + 1, j), -2.0 * w);   // row < col: IU(k,j) < IU(k+1,j)
            }

        // WW * sum_k ||w[k]||^2 — pure diagonal, no coupling between nodes
        for (int k = 0; k < _n; k++)
            for (int i = 0; i < 3; i++)
                AddP(IX(k, Dynamics6Dof.IW + i), IX(k, Dynamics6Dof.IW + i), 2.0 * _cfg.WW);

        // Proximal term: ProximalWeight * sum_k ||(X[k]-Xbar[k])/Xscale||^2.
        // Expanding gives a diagonal P contribution 2*w/xs^2 and a LINEAR term
        // -2*w*xbar/xs^2; the constant xbar'xbar is dropped as it cannot change the
        // argmin. Conditions P exactly as WW did but centred on the reference, so it
        // biases nothing and is zero at convergence.
        if (_cfg.ProximalWeight > 0.0)
            for (int k = 0; k < _n; k++)
                for (int i = 0; i < NX; i++)
                {
                    double w = _cfg.ProximalWeight / (_xs[i] * _xs[i]);
                    int col = IX(k, i);
                    AddP(col, col, 2.0 * w);
                    _c[col] += -2.0 * w * xbar[k * NX + i] * _colScale[col];
                }

        // RhoVc * sum_k ||Wv[k]/Xscale||^2 — pure diagonal
        for (int k = 0; k < _n - 1; k++)
            for (int i = 0; i < NX; i++)
            {
                double w = _cfg.RhoVc / (_xs[i] * _xs[i]);
                AddP(IW(k, i), IW(k, i), 2.0 * w);
            }
    }

    // -------------------------------------------------------------- equalities

    // Identical construction to the deleted ECOS port's AssembleEqualities — same
    // trapezoidal, time-dilated, virtual-controlled dynamics, same interior-only
    // quaternion tangent plane (nodes 0 and N-1 are pinned outright, so including
    // them there is a linearly dependent row; that redundancy was diagnosed
    // against ECOS but is a property of the constraint set, not the solver, so it
    // applies here too).
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
            AddA(row, IX(_n - 1, i), 1.0);
            _b[row++] = xf[i];
        }

        double half = 0.5 * _dtau;
        for (int k = 0; k < _n - 1; k++)
        {
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
                AddA(r, _iSig, -half * (f0[k * NX + i] + f0[(k + 1) * NX + i]));
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

        for (int k = 1; k < _n - 1; k++)
        {
            for (int i = 0; i < 4; i++)
                AddA(row, IX(k, Dynamics6Dof.IQ + i), xbar[k * NX + Dynamics6Dof.IQ + i]);
            _b[row++] = 1.0;
        }

        if (row != _nEq)
            throw new InvalidOperationException($"equality rows: emitted {row}, expected {_nEq}");
    }

    // ------------------------------------------------------------------- cone

    // Same positive-orthant rows as the ECOS port's AssembleCone, minus the
    // three epigraph cones (now handled by P instead), written starting at row
    // offset _nEq since SCS stacks equalities and cone rows into one matrix.
    private void AssembleCone(double[] xbar, double[] ubar, double sigBar, double tr)
    {
        _row = _nEq;
        ref int row = ref _row;

        for (int k = 0; k < _n; k++)
        {
            AddA(row, IU(k, Dynamics6Dof.IT), -1.0);
            _b[row++] = -_cfg.Tmin;
            AddA(row, IU(k, Dynamics6Dof.IT), 1.0);
            _b[row++] = _cfg.Tmax;
            AddA(row, IU(k, Dynamics6Dof.ITAU), 1.0);
            _b[row++] = _cfg.TauRollMax;
            AddA(row, IU(k, Dynamics6Dof.ITAU), -1.0);
            _b[row++] = _cfg.TauRollMax;

            double qx = xbar[k * NX + Dynamics6Dof.IQ + 1];
            double qy = xbar[k * NX + Dynamics6Dof.IQ + 2];
            double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
            double dqx = -4.0 * qx, dqy = -4.0 * qy;
            double rhs = _cfg.CosTilt - r22 + dqx * qx + dqy * qy;
            AddA(row, IX(k, Dynamics6Dof.IQ + 1), -dqx);
            AddA(row, IX(k, Dynamics6Dof.IQ + 2), -dqy);
            _b[row++] = -rhs;

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

        double sigLim = tr * _cfg.SigmaScale;
        AddA(row, _iSig, 1.0);
        _b[row++] = sigLim + sigBar;
        AddA(row, _iSig, -1.0);
        _b[row++] = sigLim - sigBar;
        AddA(row, _iSig, -1.0);
        _b[row++] = -_cfg.SigmaMin;
        AddA(row, _iSig, 1.0);
        _b[row++] = _cfg.SigmaMax;

        for (int k = 0; k < _n; k++)
        {
            AddA(row, IU(k, Dynamics6Dof.IT), -_cfg.TanGimbal);
            _b[row++] = 0;
            AddA(row, IU(k, Dynamics6Dof.ITDX), -1.0);
            _b[row++] = 0;
            AddA(row, IU(k, Dynamics6Dof.ITDY), -1.0);
            _b[row++] = 0;
        }

        if (row != _nCone)
            throw new InvalidOperationException($"cone rows: emitted {row}, expected {_nCone}");
    }

    // ------------------------------------------------------------ audit tools

    /// <summary>
    /// Packs a known SI primal point into scaled coordinates, for auditing an
    /// externally produced solution against this formulation — same purpose and
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
    /// (1/2) z'Pz + c'z at a scaled point — what SCS actually minimises. Compare
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
    /// the native side gets to flush) — so a validation failure can be
    /// completely silent on the managed side even though SCS "explained itself"
    /// internally. This reproduces the same checks in C# where nothing can eat
    /// the output.
    /// </summary>
    public string DiagnoseScsValidation()
    {
        var sb = new System.Text.StringBuilder();
        int n = _nVars, m = _A.Rows;

        int[] ajc = _A.ColumnPointers, air = _A.RowIndices;
        int anz = ajc[n];
        sb.AppendLine($"A: {n} cols, {m} rows, nnz={anz} (assembler reports {_A.NonZeros})");
        if (anz != _A.NonZeros)
            sb.AppendLine("  MISMATCH: A.ColumnPointers[n] != A.NonZeros — column pointer array is inconsistent");
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
                    sb.AppendLine($"  FAIL (validate_lin_sys): P[{pir[k]},{col}] has row > col — not upper triangular");
                }
        if (upperOk) sb.AppendLine("  P upper-triangular OK");

        int fullConeDims = _nEq + _lDim + _socDims.Sum();
        sb.AppendLine($"cones: z={_nEq} l={_lDim} sum(q)={_socDims.Sum()} qsize={_socDims.Length} " +
                      $"-> total {fullConeDims}, m={m}");
        if (fullConeDims != m)
            sb.AppendLine($"  FAIL (validate_cones): cone dims {fullConeDims} != m={m}");
        if (_socDims.Any(d => d < 0))
            sb.AppendLine("  FAIL (validate_cones): a SOC dimension is negative");

        // c and b finiteness — SCS doesn't explicitly validate this, but a NaN
        // or Infinity here is exactly the kind of thing that would otherwise
        // masquerade as an opaque setup failure.
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

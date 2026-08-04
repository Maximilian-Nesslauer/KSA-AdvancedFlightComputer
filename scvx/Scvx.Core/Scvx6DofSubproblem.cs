namespace Scvx;

/// <summary>Vehicle limits and SCvx weights for the 6-DOF subproblem. Defaults mirror 6dof.py.</summary>
public sealed class Scvx6DofConfig
{
    public int Nodes { get; init; } = 30;

    public double Tmax { get; init; } = 6.0e6;
    public double ThrottleFloor { get; init; } = 0.40;          // Tmin = floor * Tmax
    public double GimbalMaxDeg { get; init; } = 10.0;
    public double TauRollMax { get; init; } = 1.0e5;
    public double TiltMaxDeg { get; init; } = 30.0;
    public double GroundFloor { get; init; } = -1.0;            // X[n,2] >= this

    public double RhoVc { get; init; } = 1e5;                   // virtual-control penalty
    public double WDu { get; init; } = 0.2;                     // control-rate smoothing
    public double WW { get; init; } = 1.0;                      // angular-rate damping
    public double SigmaMin { get; init; } = 5.0;
    public double SigmaMax { get; init; } = 25.0;
    public double SigmaScale { get; init; } = 12.0;

    public double[] XScale { get; init; } =
        [100, 100, 300, 50, 50, 50, 1, 1, 1, 1, 1, 1, 1, 250000.0];
    public double[]? UScale { get; init; }                      // defaults from Tmax/gimbal

    public double Tmin => ThrottleFloor * Tmax;
    public double TanGimbal => Math.Tan(GimbalMaxDeg * Math.PI / 180.0);
    public double CosTilt => Math.Cos(TiltMaxDeg * Math.PI / 180.0);

    public double[] ResolvedUScale => UScale ??
        [Tmax * TanGimbal, Tmax * TanGimbal, Tmax, TauRollMax];
}

/// <summary>
/// The convex subproblem SCvx solves at each iteration: minimum fuel over the
/// dynamics linearised about a reference trajectory, in ECOS cone form.
///
/// Port of the CVXPY problem in 6dof.py. CVXPY canonicalises symbolic
/// expressions into cone form invisibly; here it is written out, because that
/// canonicalisation is where the Python reference spends most of its time and
/// because the resulting pattern can then be reused across iterations.
///
/// VARIABLES (n = N*NX + N*NU + (N-1)*NX + 4)
///   X       N x 14   state at each node
///   U       N x 4    control at each node
///   Wv    N-1 x 14   virtual control (dynamics defect slack)
///   sigma            free final time
///   tDu, tW, tVc     epigraph variables for the three quadratic penalties
///
/// EQUALITIES (A x = b)
///   initial state (14), final state (13 — mass free), trapezoidal dynamics
///   (14 per interval), unit-quaternion tangent plane (1 per node)
///
/// CONE (G x + s = h)
///   positive orthant: throttle box, roll-torque box, tilt half-space, ground
///   plane, trust region on X/U/sigma, sigma bounds
///   SOC: gimbal cone per node, plus one epigraph cone per quadratic penalty
///
/// QUADRATIC OBJECTIVE TERMS. ECOS minimises a LINEAR objective, so each
/// sum_squares becomes an epigraph variable t plus a cone. For t >= ||z||^2 the
/// standard rotated-cone form is
///     || [2z ; t-1] || &lt;= t+1
/// which expands to 4||z||^2 + (t-1)^2 &lt;= (t+1)^2, i.e. ||z||^2 &lt;= t.
/// Minimising t then drives the sum of squares down exactly as CVXPY's
/// sum_squares does — matching the reference objective, which matters because
/// the Python solution is the validation oracle.
/// </summary>
public sealed class Scvx6DofSubproblem : IDisposable
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    private readonly Scvx6DofConfig _cfg;
    private readonly int _n;                 // nodes
    private readonly double _dtau;
    private readonly double[] _xs, _us;      // scaling

    // variable offsets
    private readonly int _oX, _oU, _oW, _iSig, _iTdu, _iTw, _iTvc, _nVars;
    // constraint counts
    private readonly int _nEq, _nCone, _lDim;
    private readonly int[] _socDims;

    private readonly CcsAssembler _G, _A;
    private readonly double[] _h, _b, _c;
    private EcosWorkspace? _ws;

    /// <summary>
    /// Per-variable scale: the solver works in x~ where x = diag(scale) x~.
    ///
    /// Raw SI is not solvable here. Positions are O(1e2), thrust O(1e6), inertia
    /// O(1e8), mass O(1e5) — and ECOS breaks down with "unreliable search
    /// direction" on the raw problem even though the formulation is provably
    /// correct (a reference optimum audits as feasible to 1e-6). Its built-in
    /// equilibration is not enough. The same lesson was learned on the 3-DOF
    /// G-FOLD problem, where both ECOS and Clarabel returned false optima until
    /// the problem was non-dimensionalised.
    ///
    /// The scales are the ones the formulation already carries for the trust
    /// region and the objective weights, so this costs nothing to maintain and
    /// makes the trust region literally |x~ - xbar~| &lt;= tr.
    ///
    /// Substituting x = S x~ turns A x = b into (A S) x~ = b, so every
    /// coefficient is multiplied by its COLUMN's scale on the way in, and the
    /// solution is multiplied back out. Row magnitudes are left to ECOS's own
    /// equilibration.
    /// </summary>
    private readonly double[] _colScale;

    public int VariableCount => _nVars;
    public int EqualityCount => _nEq;
    public int ConeRowCount => _nCone;
    public int Iterations => _ws?.Iterations ?? 0;
    public double PrimalCost => _ws?.PrimalCost ?? double.NaN;

    // --- variable indexing ---
    public int IX(int node, int i) => _oX + node * NX + i;
    public int IU(int node, int j) => _oU + node * NU + j;
    public int IW(int interval, int i) => _oW + interval * NX + i;
    public int ISigma => _iSig;

    public Scvx6DofSubproblem(Scvx6DofConfig cfg)
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
        _iTdu = _iSig + 1;
        _iTw = _iTdu + 1;
        _iTvc = _iTw + 1;
        _nVars = _iTvc + 1;

        _nEq = NX                 // initial state
             + (NX - 1)           // final state, mass free
             + (_n - 1) * NX      // dynamics
             + (_n - 2);          // quaternion tangent plane, interior nodes only

        // positive-orthant rows
        _lDim = _n * 6                  // throttle (2), roll torque (2), tilt, ground
              + 2 * _n * NX             // trust region on X
              + 2 * _n * NU             // trust region on U
              + 4;                      // sigma trust region (2) + sigma bounds (2)

        // second-order cones
        var soc = new List<int>();
        for (int k = 0; k < _n; k++) soc.Add(3);         // gimbal: ||U[0:2]|| <= tan*U[2]
        soc.Add((_n - 1) * NU + 2);                      // tDu  epigraph
        soc.Add(_n * 3 + 2);                             // tW   epigraph
        soc.Add((_n - 1) * NX + 2);                      // tVc  epigraph
        _socDims = [.. soc];

        _nCone = _lDim + _socDims.Sum();

        _G = new CcsAssembler(_nCone, _nVars);
        _A = new CcsAssembler(_nEq, _nVars);
        _h = new double[_nCone];
        _b = new double[_nEq];
        _c = new double[_nVars];

        _colScale = new double[_nVars];
        for (int k = 0; k < _n; k++)
            for (int i = 0; i < NX; i++)
                _colScale[IX(k, i)] = _xs[i];
        for (int k = 0; k < _n; k++)
            for (int j = 0; j < NU; j++)
                _colScale[IU(k, j)] = _us[j];
        for (int k = 0; k < _n - 1; k++)
            for (int i = 0; i < NX; i++)
                _colScale[IW(k, i)] = _xs[i];
        _colScale[_iSig] = cfg.SigmaScale;
        // The epigraph variables carry already-normalised quantities, so they are
        // O(1) and left alone. (Scaling them by 1/weight to flatten c from its
        // [WDu, RhoVc] = [0.2, 1e5] span was tried and made no difference to
        // ECOS's breakdown, so the extra indirection is not worth carrying.)
        _colScale[_iTdu] = 1.0;
        _colScale[_iTw] = 1.0;
        _colScale[_iTvc] = 1.0;
    }

    // All coefficients go in through these, so the column scaling can never be
    // applied in one place and forgotten in another.
    private void AddA(int row, int col, double value) => _A.Add(row, col, value * _colScale[col]);
    private void AddG(int row, int col, double value) => _G.Add(row, col, value * _colScale[col]);

    /// <summary>
    /// Fill (first call) or refill (later calls) the problem from a reference
    /// trajectory and linearisation, then solve.
    ///
    /// A/B/f0 are the Jacobians and dynamics at each node from
    /// <see cref="Dynamics6Dof.Jacobian"/>: A is N blocks of NX*NX row-major,
    /// B is N blocks of NX*NU row-major, f0 is N blocks of NX.
    /// </summary>
    public EcosStatus Solve(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf,
                            double[] xbar, double[] ubar, double sigBar, double tr,
                            double[] A, double[] B, double[] f0,
                            bool verbose = false, int maxIterations = 100)
    {
        Assemble(x0, xf, xbar, ubar, sigBar, tr, A, B, f0);
        return Run(verbose, maxIterations);
    }

    /// <summary>
    /// Fill (first call) or refill (later calls) every matrix and vector from a
    /// reference trajectory and linearisation, without solving.
    ///
    /// Separate from <see cref="Run"/> because ECOS EQUILIBRATES G, A, c, h and b
    /// IN PLACE during setup — after the first solve those arrays hold scaled
    /// data, so anything that wants to inspect the problem as formulated (an
    /// audit, a residual check, a dump) has to look between Assemble and Run.
    /// </summary>
    public void Assemble(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf,
                         double[] xbar, double[] ubar, double sigBar, double tr,
                         double[] A, double[] B, double[] f0)
    {
        if (_ws != null)
        {
            _G.BeginRefill();
            _A.BeginRefill();
        }
        Array.Clear(_h);
        Array.Clear(_b);
        Array.Clear(_c);

        AssembleObjective(x0[Dynamics6Dof.IM]);
        AssembleEqualities(x0, xf, xbar, ubar, sigBar, A, B, f0);
        AssembleCone(xbar, ubar, sigBar, tr);

        if (_ws == null)
        {
            _G.Freeze();
            _A.Freeze();
        }
        else
        {
            _G.EndRefill();
            _A.EndRefill();
        }

        EquilibrateRows();
        _assembled = true;
    }

    /// <summary>
    /// Normalise every row by its largest coefficient, so no row of the KKT
    /// system is orders of magnitude larger than another.
    ///
    /// Column scaling alone does not rescue this problem: with x~ made O(1) the
    /// ROWS still span from the quaternion tangent plane (order 1) to the control
    /// trust region (order 1e6), and ECOS still breaks down. Its own
    /// equilibration cannot fully fix that, because a second-order cone may only
    /// be scaled UNIFORMLY — scaling its rows independently would deform the cone
    /// and change the problem — so cone blocks are exactly where equilibration is
    /// least free to help.
    ///
    /// Doing it here is safe for the same reason: equality rows and positive
    /// orthant rows take any positive scale individually, and each SOC gets one
    /// scale applied to all of its rows. Scaling rows leaves the solution
    /// untouched — (D A) x = D b has the same x — so nothing needs unscaling
    /// afterwards, unlike the column scaling.
    /// </summary>
    private void EquilibrateRows()
    {
        ScaleRows(_A, _b, _nEq, socStart: -1);
        ScaleRows(_G, _h, _nCone, socStart: _lDim);
    }

    private void ScaleRows(CcsAssembler m, double[] rhs, int rows, int socStart)
    {
        var rowMax = new double[rows];
        int[] jc = m.ColumnPointers, ir = m.RowIndices;
        double[] pr = m.Values;

        for (int col = 0; col < m.Cols; col++)
            for (int k = jc[col]; k < jc[col + 1]; k++)
            {
                double a = Math.Abs(pr[k]);
                if (a > rowMax[ir[k]]) rowMax[ir[k]] = a;
            }

        // Within each second-order cone every row must share one scale.
        if (socStart >= 0)
        {
            int off = socStart;
            foreach (int d in _socDims)
            {
                double blockMax = 0;
                for (int i = off; i < off + d; i++)
                    blockMax = Math.Max(blockMax, rowMax[i]);
                for (int i = off; i < off + d; i++)
                    rowMax[i] = blockMax;
                off += d;
            }
        }

        var inv = new double[rows];
        for (int i = 0; i < rows; i++)
            inv[i] = rowMax[i] > 0 ? 1.0 / rowMax[i] : 1.0;

        for (int col = 0; col < m.Cols; col++)
            for (int k = jc[col]; k < jc[col + 1]; k++)
                pr[k] *= inv[ir[k]];
        for (int i = 0; i < rows; i++)
            rhs[i] *= inv[i];
    }

    private bool _assembled;

    /// <summary>Solve whatever <see cref="Assemble"/> last produced.</summary>
    public EcosStatus Run(bool verbose = false, int maxIterations = 100)
    {
        if (!_assembled)
            throw new InvalidOperationException("call Assemble first");
        // Setup is deferred to here so the caller gets a window to inspect the
        // un-equilibrated problem.
        _ws ??= new EcosWorkspace(_G, _h, _A, _b, _c, _lDim, _socDims);
        return _ws.Solve(verbose, maxIterations);
    }

    /// <summary>
    /// Solution vector from the last solve, unscaled back into SI: the solver
    /// works in x~, this returns x = diag(scale) x~.
    /// </summary>
    public double[] Solution
    {
        get
        {
            double[] raw = _ws?.X ?? throw new InvalidOperationException("not solved yet");
            var x = new double[_nVars];
            for (int i = 0; i < _nVars; i++) x[i] = raw[i] * _colScale[i];
            return x;
        }
    }

    /// <summary>State trajectory from the last solve, N*NX row-major.</summary>
    public double[] SolutionX => Slice(_oX, _n * NX);

    /// <summary>Control trajectory from the last solve, N*NU row-major.</summary>
    public double[] SolutionU => Slice(_oU, _n * NU);

    /// <summary>Virtual control from the last solve, (N-1)*NX row-major.</summary>
    public double[] SolutionWv => Slice(_oW, (_n - 1) * NX);

    public double SolutionSigma => Solution[_iSig];

    private double[] Slice(int offset, int count) => Solution[offset..(offset + count)];

    /// <summary>
    /// The objective as the reference defines it, recomputed from the solution
    /// rather than read from the solver. The epigraph variables only bound the
    /// quadratic terms from above, so at the optimum they equal them — evaluating
    /// directly is the honest comparison against CVXPY's reported value, and a
    /// gap between the two would mean an epigraph cone is wrong.
    /// </summary>
    public double EvaluateObjective(double mInit)
    {
        double[] x = SolutionX, u = SolutionU, wv = SolutionWv;

        double fuel = (mInit - x[(_n - 1) * NX + Dynamics6Dof.IM]) / mInit;

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

        double vc = 0;
        for (int k = 0; k < _n - 1; k++)
            for (int i = 0; i < NX; i++)
            {
                double d = wv[k * NX + i] / _xs[i];
                vc += d * d;
            }

        return fuel + _cfg.WDu * du + _cfg.WW * ww + _cfg.RhoVc * vc;
    }

    /// <summary>
    /// Packs a known primal point into the solver's variable ordering, so an
    /// externally produced solution can be audited against this formulation.
    /// The epigraph variables are set to exactly their quadratic terms, which is
    /// where they sit at an optimum.
    /// </summary>
    public double[] PackPrimal(double[] x, double[] u, double[] wv, double sigma)
    {
        var z = new double[_nVars];
        Array.Copy(x, 0, z, _oX, _n * NX);
        Array.Copy(u, 0, z, _oU, _n * NU);
        Array.Copy(wv, 0, z, _oW, (_n - 1) * NX);
        z[_iSig] = sigma;

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
        double vc = 0;
        for (int k = 0; k < _n - 1; k++)
            for (int i = 0; i < NX; i++)
            {
                double d = wv[k * NX + i] / _xs[i];
                vc += d * d;
            }
        z[_iTdu] = du;
        z[_iTw] = ww;
        z[_iTvc] = vc;

        // The assembled matrices are in scaled coordinates, so an SI point has to
        // be divided into x~ before it can be substituted into them.
        for (int i = 0; i < _nVars; i++) z[i] /= _colScale[i];
        return z;
    }

    /// <summary>
    /// Residuals of a primal point against the assembled problem:
    /// max |Ax - b| over the equality rows, and the worst cone violation
    /// (negative slack for the orthant, ||s[1:]|| - s[0] for each SOC).
    ///
    /// Feeding a solution the reference solver produced separates a formulation
    /// error from a conditioning one: a correct formulation must report a
    /// feasible point feasible, whatever ECOS then does with it.
    /// </summary>
    public (double EqResidual, int EqRow, double ConeViolation, int ConeIndex)
        CheckPrimal(double[] z)
    {
        double[] ax = MultiplyCcs(_A, z, _nEq);
        double eqWorst = 0;
        int eqRow = -1;
        for (int i = 0; i < _nEq; i++)
        {
            double r = Math.Abs(ax[i] - _b[i]);
            if (r > eqWorst) { eqWorst = r; eqRow = i; }
        }

        double[] gx = MultiplyCcs(_G, z, _nCone);
        var s = new double[_nCone];
        for (int i = 0; i < _nCone; i++) s[i] = _h[i] - gx[i];

        double coneWorst = 0;
        int coneIdx = -1;
        for (int i = 0; i < _lDim; i++)
            if (-s[i] > coneWorst) { coneWorst = -s[i]; coneIdx = i; }

        int off = _lDim;
        for (int c = 0; c < _socDims.Length; c++)
        {
            int d = _socDims[c];
            double tail = 0;
            for (int i = 1; i < d; i++) tail += s[off + i] * s[off + i];
            double v = Math.Sqrt(tail) - s[off];
            if (v > coneWorst) { coneWorst = v; coneIdx = off; }
            off += d;
        }
        return (eqWorst, eqRow, coneWorst, coneIdx);
    }

    /// <summary>
    /// c'x~ for a packed (already scaled) point — what ECOS actually minimises.
    /// Differs from the reference objective by the dropped constant term
    /// mInit/mInit = 1, so compare against (reference - 1).
    /// </summary>
    public double LinearObjective(double[] z)
    {
        double s = 0;
        for (int i = 0; i < _nVars; i++) s += _c[i] * z[i];
        return s;
    }

    /// <summary>
    /// Writes the assembled cone program as triplets, so the very same matrices
    /// can be handed to another solver. Settles "is the assembly wrong or is ECOS
    /// struggling" — a question no amount of staring at the formulation answers.
    /// Must be called between Assemble and Run, before ECOS equilibrates in place.
    /// </summary>
    public void Dump(string path)
    {
        using var w = new StreamWriter(path);
        w.WriteLine($"n {_nVars}");
        w.WriteLine($"p {_nEq}");
        w.WriteLine($"m {_nCone}");
        w.WriteLine($"l {_lDim}");
        w.WriteLine("q " + string.Join(" ", _socDims));
        WriteVector(w, "c", _c);
        WriteVector(w, "b", _b);
        WriteVector(w, "h", _h);
        WriteTriplets(w, "A", _A);
        WriteTriplets(w, "G", _G);
    }

    private static void WriteVector(StreamWriter w, string tag, double[] v)
    {
        w.WriteLine($"{tag} {v.Length}");
        w.WriteLine(string.Join(",", v.Select(z => z.ToString("R"))));
    }

    private static void WriteTriplets(StreamWriter w, string tag, CcsAssembler m)
    {
        int[] jc = m.ColumnPointers, ir = m.RowIndices;
        double[] pr = m.Values;
        w.WriteLine($"{tag} {pr.Length}");
        for (int col = 0; col < m.Cols; col++)
            for (int k = jc[col]; k < jc[col + 1]; k++)
                w.WriteLine($"{ir[k]},{col},{pr[k]:R}");
    }

    /// <summary>Worst offending rows of a primal point, for diagnosing an assembly error.</summary>
    public string Diagnose(double[] z, int top = 6)
    {
        var sb = new System.Text.StringBuilder();
        double[] ax = MultiplyCcs(_A, z, _nEq);
        var eq = Enumerable.Range(0, _nEq)
            .Select(i => (Row: i, Res: Math.Abs(ax[i] - _b[i])))
            .OrderByDescending(t => t.Res).Take(top);
        sb.AppendLine("  worst equality rows:");
        foreach (var (r, res) in eq)
            sb.AppendLine($"    row {r,4} ({DescribeEqRow(r)})  |Ax-b|={res:E3}  Ax={ax[r]:E3} b={_b[r]:E3}");

        double[] gx = MultiplyCcs(_G, z, _nCone);
        var orth = Enumerable.Range(0, _lDim)
            .Select(i => (Row: i, S: _h[i] - gx[i]))
            .OrderBy(t => t.S).Take(top);
        sb.AppendLine("  most negative orthant slacks:");
        foreach (var (r, s) in orth)
            sb.AppendLine($"    row {r,4} ({DescribeConeRow(r)})  s={s:E3}  h={_h[r]:E3} Gx={gx[r]:E3}");
        return sb.ToString();
    }

    private string DescribeEqRow(int r)
    {
        if (r < NX) return $"initial x[{r}]";
        r -= NX;
        if (r < NX - 1) return $"final x[{r}]";
        r -= NX - 1;
        if (r < (_n - 1) * NX) return $"dynamics k={r / NX} comp={r % NX}";
        r -= (_n - 1) * NX;
        return $"quat tangent node {r + 1}";   // interior nodes start at 1
    }

    private string DescribeConeRow(int r)
    {
        if (r < _n * 6)
        {
            string[] kind = ["Tmin", "Tmax", "tau+", "tau-", "tilt", "ground"];
            return $"node {r / 6} {kind[r % 6]}";
        }
        r -= _n * 6;
        if (r < 2 * _n * NX) return $"trust X node {r / (2 * NX)} comp {(r % (2 * NX)) / 2} {(r % 2 == 0 ? "hi" : "lo")}";
        r -= 2 * _n * NX;
        if (r < 2 * _n * NU) return $"trust U node {r / (2 * NU)} comp {(r % (2 * NU)) / 2} {(r % 2 == 0 ? "hi" : "lo")}";
        r -= 2 * _n * NU;
        return $"sigma row {r}";
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

    // ---------------------------------------------------------------- objective

    private void AssembleObjective(double mInit)
    {
        // minimise (mInit - X[N-1, m]) / mInit; the constant drops out of argmin.
        // c'x = c'(S x~) = (S c)' x~, so each entry carries its column's scale.
        SetC(IX(_n - 1, Dynamics6Dof.IM), -1.0 / mInit);
        SetC(_iTdu, _cfg.WDu);
        SetC(_iTw, _cfg.WW);
        SetC(_iTvc, _cfg.RhoVc);
    }

    private void SetC(int col, double value) => _c[col] = value * _colScale[col];

    // -------------------------------------------------------------- equalities

    private void AssembleEqualities(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf,
                                    double[] xbar, double[] ubar, double sigBar,
                                    double[] A, double[] B, double[] f0)
    {
        int row = 0;

        // initial state, all 14 components pinned
        for (int i = 0; i < NX; i++)
        {
            AddA(row, IX(0, i), 1.0);
            _b[row++] = x0[i];
        }

        // final state: r, v, q, w pinned; mass left free (it is the objective)
        for (int i = 0; i < NX - 1; i++)
        {
            AddA(row, IX(_n - 1, i), 1.0);
            _b[row++] = xf[i];
        }

        // Trapezoidal dynamics with time dilation and virtual control:
        //   X[k+1] = X[k] + dtau/2 (g[k] + g[k+1]) + Wv[k]
        //   g[k]   = sigma f0[k] + sigBar ( A[k] (X[k]-Xbar[k]) + B[k] (U[k]-Ubar[k]) )
        //
        // Collected per variable, with the reference terms moved to the RHS. Each
        // coefficient is emitted once, already combined — the identity from
        // X[k+1]-X[k] is folded into the A-block diagonal rather than added
        // separately, so no (row, col) is written twice and refills stay
        // unambiguous.
        double half = 0.5 * _dtau;
        for (int k = 0; k < _n - 1; k++)
        {
            int aK = k * NX * NX, aK1 = (k + 1) * NX * NX;
            int bK = k * NX * NU, bK1 = (k + 1) * NX * NU;

            for (int i = 0; i < NX; i++)
            {
                int r = row + i;

                // X[k]: -I - half*sigBar*A[k]
                for (int j = 0; j < NX; j++)
                {
                    double v = -half * sigBar * A[aK + i * NX + j];
                    if (i == j) v -= 1.0;
                    AddA(r, IX(k, j), v);
                }
                // X[k+1]: +I - half*sigBar*A[k+1]
                for (int j = 0; j < NX; j++)
                {
                    double v = -half * sigBar * A[aK1 + i * NX + j];
                    if (i == j) v += 1.0;
                    AddA(r, IX(k + 1, j), v);
                }
                // U[k], U[k+1]
                for (int j = 0; j < NU; j++)
                    AddA(r, IU(k, j), -half * sigBar * B[bK + i * NU + j]);
                for (int j = 0; j < NU; j++)
                    AddA(r, IU(k + 1, j), -half * sigBar * B[bK1 + i * NU + j]);
                // sigma
                AddA(r, _iSig, -half * (f0[k * NX + i] + f0[(k + 1) * NX + i]));
                // virtual control
                AddA(r, IW(k, i), -1.0);

                // RHS: -half*sigBar*( A[k]Xbar[k] + B[k]Ubar[k]
                //                   + A[k+1]Xbar[k+1] + B[k+1]Ubar[k+1] )
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

        // Unit quaternion, linearised as the tangent plane at the reference:
        // qbar . q = 1. Exact at the fixpoint; the SCvx loop reprojects on accept.
        //
        // INTERIOR NODES ONLY. At node 0 and node N-1 the quaternion is already
        // pinned outright by the boundary conditions, so the tangent row is a
        // linear combination of those four rows — a redundant equality, which
        // leaves the KKT block rank-deficient and makes ECOS bail out with
        // "unreliable search direction" on an otherwise correct problem. (The
        // reference includes them; CVXPY's presolve absorbs the redundancy.)
        // Dropping them also removes a latent infeasibility: with q pinned, the
        // row degenerates to a condition on the REFERENCE, qbar . qf = 1, which
        // no choice of variable can satisfy if the reference has drifted.
        for (int k = 1; k < _n - 1; k++)
        {
            for (int i = 0; i < 4; i++)
                AddA(row, IX(k, Dynamics6Dof.IQ + i), xbar[k * NX + Dynamics6Dof.IQ + i]);
            _b[row++] = 1.0;
        }
    }

    // ------------------------------------------------------------------- cones

    // Cone-row cursor. A field rather than a local so the epigraph helpers can
    // advance it without threading a ref through every emitter; assembly is
    // single-threaded and one pass, so there is no aliasing to worry about.
    private int _row;

    private void AssembleCone(double[] xbar, double[] ubar, double sigBar, double tr)
    {
        ref int row = ref _row;
        row = 0;

        // ---- positive orthant: s = h - Gx >= 0 ----
        for (int k = 0; k < _n; k++)
        {
            // Tmin <= U[k,T]
            AddG(row, IU(k, Dynamics6Dof.IT), -1.0);
            _h[row++] = -_cfg.Tmin;
            // U[k,T] <= Tmax
            AddG(row, IU(k, Dynamics6Dof.IT), 1.0);
            _h[row++] = _cfg.Tmax;
            // |U[k,tau]| <= TauRollMax
            AddG(row, IU(k, Dynamics6Dof.ITAU), 1.0);
            _h[row++] = _cfg.TauRollMax;
            AddG(row, IU(k, Dynamics6Dof.ITAU), -1.0);
            _h[row++] = _cfg.TauRollMax;

            // Tilt: keep R22 = 1 - 2(qx^2 + qy^2) >= cos(tilt_max), linearised in q
            // about the reference quaternion.
            double qx = xbar[k * NX + Dynamics6Dof.IQ + 1];
            double qy = xbar[k * NX + Dynamics6Dof.IQ + 2];
            double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
            double dqx = -4.0 * qx, dqy = -4.0 * qy;
            // r22 + [0,dqx,dqy,0].(q - qbar) >= cosTilt
            //   =>  dqx*qx_var + dqy*qy_var >= cosTilt - r22 + dqx*qx + dqy*qy
            double rhs = _cfg.CosTilt - r22 + dqx * qx + dqy * qy;
            AddG(row, IX(k, Dynamics6Dof.IQ + 1), -dqx);
            AddG(row, IX(k, Dynamics6Dof.IQ + 2), -dqy);
            _h[row++] = -rhs;

            // Ground plane
            AddG(row, IX(k, 2), -1.0);
            _h[row++] = -_cfg.GroundFloor;
        }

        // Trust region on X: |X - Xbar| <= tr * Xscale, componentwise
        for (int k = 0; k < _n; k++)
            for (int i = 0; i < NX; i++)
            {
                double lim = tr * _xs[i], @ref = xbar[k * NX + i];
                AddG(row, IX(k, i), 1.0);
                _h[row++] = lim + @ref;
                AddG(row, IX(k, i), -1.0);
                _h[row++] = lim - @ref;
            }

        // Trust region on U
        for (int k = 0; k < _n; k++)
            for (int j = 0; j < NU; j++)
            {
                double lim = tr * _us[j], @ref = ubar[k * NU + j];
                AddG(row, IU(k, j), 1.0);
                _h[row++] = lim + @ref;
                AddG(row, IU(k, j), -1.0);
                _h[row++] = lim - @ref;
            }

        // sigma: trust region about sigBar, then absolute bounds
        double sigLim = tr * _cfg.SigmaScale;
        AddG(row, _iSig, 1.0);
        _h[row++] = sigLim + sigBar;
        AddG(row, _iSig, -1.0);
        _h[row++] = sigLim - sigBar;
        AddG(row, _iSig, -1.0);
        _h[row++] = -_cfg.SigmaMin;
        AddG(row, _iSig, 1.0);
        _h[row++] = _cfg.SigmaMax;

        // ---- second-order cones: s in Q means s[0] >= ||s[1:]|| ----

        // Gimbal: ||U[k,0:2]|| <= tanGimbal * U[k,T]
        for (int k = 0; k < _n; k++)
        {
            AddG(row, IU(k, Dynamics6Dof.IT), -_cfg.TanGimbal);
            _h[row++] = 0;
            AddG(row, IU(k, Dynamics6Dof.ITDX), -1.0);
            _h[row++] = 0;
            AddG(row, IU(k, Dynamics6Dof.ITDY), -1.0);
            _h[row++] = 0;
        }

        // tDu >= || diff(U) / Uscale ||^2
        EpigraphHead(_iTdu);
        for (int k = 0; k < _n - 1; k++)
            for (int j = 0; j < NU; j++)
            {
                // z = (U[k+1,j] - U[k,j]) / Uscale[j]; the cone row carries 2z
                AddG(row, IU(k + 1, j), -2.0 / _us[j]);
                AddG(row, IU(k, j), 2.0 / _us[j]);
                _h[row++] = 0;
            }
        EpigraphTail(_iTdu);

        // tW >= || w ||^2   (angular-rate damping, unscaled as in the reference)
        EpigraphHead(_iTw);
        for (int k = 0; k < _n; k++)
            for (int i = 0; i < 3; i++)
            {
                AddG(row, IX(k, Dynamics6Dof.IW + i), -2.0);
                _h[row++] = 0;
            }
        EpigraphTail(_iTw);

        // tVc >= || Wv / Xscale ||^2
        EpigraphHead(_iTvc);
        for (int k = 0; k < _n - 1; k++)
            for (int i = 0; i < NX; i++)
            {
                AddG(row, IW(k, i), -2.0 / _xs[i]);
                _h[row++] = 0;
            }
        EpigraphTail(_iTvc);

        if (row != _nCone)
            throw new InvalidOperationException($"cone rows: emitted {row}, expected {_nCone}");
    }

    // t >= ||z||^2 as the second-order cone || [2z ; t-1] || <= t+1.
    // Head row is s0 = t+1, then the caller emits the 2z rows, then the tail row
    // is s_last = t-1.
    private void EpigraphHead(int tIndex)
    {
        AddG(_row, tIndex, -1.0);
        _h[_row++] = 1.0;
    }

    private void EpigraphTail(int tIndex)
    {
        AddG(_row, tIndex, -1.0);
        _h[_row++] = -1.0;
    }

    public void Dispose() => _ws?.Dispose();
}

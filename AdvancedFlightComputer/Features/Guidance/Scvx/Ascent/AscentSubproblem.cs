using AdvancedFlightComputer.Guidance.Conic;

namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// Convex subproblem solved at each ascent SCvx iteration, assembled into a reusable
/// frozen-sparsity <see cref="CcsAssembler"/> and solved with a persistent <see cref="ClarabelWorkspace"/>.
///
///   minimise   (1 - m_N) + lambda sum ||u_k+1 - u_k||^2
///              + rho_nu sum ||D^-1 nu_k||^2 + rho_s ||s||^2
///              + rho_q sum (s^q_k + s^qa_k + s^pro_k) + rho_g sum s^g_k
///   s.t.       x_1 = x_0,  u_1 parallel to straight up                 lift-off
///              x_k+1 = x_k + dtau/2 (g_k + g_k+1) + nu_k               collocation
///              r, v continuous, m drops by the dry mass                staging links
///              lower stages burn their load, the last at most its load propellant
///              sigma_i = the solids' burn where one pins it            solids (#73)
///              u_k = 0 on ballistic coast stages                       coast
///              ||u_k|| &lt;= 1,  u_bar_hat . u_k &gt;= throttle floor        throttle
///              T_bar_k ||u_k|| &lt;= a (backoff m_k + m_bar_k s^g_k)      thrust acceleration
///              r_bar_hat . r_k &gt;= R_floor,  m_k &gt;= m_floor             ground, structure
///              tangent q &lt;= backoff + s^q,  cone q-alpha &lt;= backoff + s^qa   where q_bar &gt; QActive
///              -v_rel_bar_hat . u_k &lt;= s^pro                           thrust with airflow
///              linearised insertion = s,  linearised prograde &gt;= 0     orbit, plane turned by phi
///              trust box on x, u (accel-normalised), sigma, phi;       trust region and bounds
///
/// To keep the CSC pattern identical across pattern-hit refills, assembly never skips explicit
/// zeros: every structural entry is emitted in identical order on both pattern build and refill.
/// </summary>
internal sealed class AscentSubproblem
{
    private const int NX = AscentCase.NX;
    private const int NU = AscentCase.NU;

    private readonly AscentCase _c;

    // Variable layout.
    private readonly int _oU, _oW, _oSig, _oTerm, _oSq, _oSqa, _oSpro, _oSg, _oPhi, _oPhiAbs, _oDR, _oSRec, _nVar;
    /// <summary>Boosters are brought home (AscentCase.Reserves, AscentCase.Needs): each reserve's change, and each booster's recovery slack, are variables.</summary>
    private readonly int _nR, _nNeed;
    /// <summary>The plane turn is a variable: its bounds are not equal.</summary>
    private readonly bool _turnFree;
    private readonly int _nA;
    private readonly int[] _act;

    public AscentSubproblem(AscentCase c, int[] activeNodes)
    {
        _c = c;
        _act = activeNodes;
        // Keep the script's max(len(act), 1) path buffer sizing; an unused slack is priced to zero.
        _nA = Math.Max(activeNodes.Length, 1);
        _oU = c.N * NX;
        _oW = _oU + c.N * NU;
        _oSig = _oW + c.Coll.Length * NX;
        _oTerm = _oSig + c.S;
        _oSq = _oTerm + 5;
        _oSqa = _oSq + _nA;
        _oSpro = _oSqa + _nA;
        _oSg = _oSpro + _nA;
        _oPhi = _oSg + c.LimitedNodes.Length;
        _oPhiAbs = _oPhi + 1;
        _turnFree = c.PlaneTurnMin < c.PlaneTurnMax;
        _oDR = _turnFree ? _oPhiAbs + 1 : _oPhi;
        _nR = c.Reserves.Length;
        _nNeed = c.Needs.Length;
        _oSRec = _oDR + _nR;
        _nVar = _oSRec + _nNeed;
    }

    public int VariableCount => _nVar;

    private static int IX(int k, int i) => k * NX + i;
    private int IU(int k, int j) => _oU + k * NU + j;
    private int IW(int c, int i) => _oW + c * NX + i;
    private int ISig(int s) => _oSig + s;
    private int ITerm(int j) => _oTerm + j;
    private int ISq(int a) => _oSq + a;
    private int ISqa(int a) => _oSqa + a;
    private int ISpro(int a) => _oSpro + a;
    private int ISg(int g) => _oSg + g;
    private int IPhi => _oPhi;
    private int IPhiAbs => _oPhiAbs;
    private int IDR(int r) => _oDR + r;
    private int ISRec(int j) => _oSRec + j;

    /// <summary>
    /// Persistent per-solve conic workspace holding frozen <see cref="CcsAssembler"/> matrices,
    /// structural pattern signatures, scratch buffers, and the native <see cref="ClarabelWorkspace"/>.
    /// </summary>
    public sealed class Workspace : IDisposable
    {
        public readonly ClarabelWorkspace Clarabel;

        public CcsAssembler? A, P;
        public double[] B = [], C = [];
        public int NVar, NEq, NIneq;
        public int[] SocDims = [];
        public int[] ActiveNodes = [];
        public bool[] FloorScales = [];
        public bool[] LimitActive = [];
        public bool[] NeedOk = [];

        internal double[] ScratchLimitThrust = [];
        internal double[] ScratchFloorPerMass = [];
        internal bool[] ScratchFloorScales = [];
        internal bool[] ScratchLimitActive = [];
        internal bool[] ScratchNeedOk = [];
        internal double[] ScratchVerticalThrottle = [];

        public Workspace(AscentSettings st)
        {
            Clarabel = new ClarabelWorkspace
            {
                AllowPersistent = st.UseClarabelPersistent,
                DefaultIterativeRefinement = st.ClarabelIterativeRefinement ? true : null,
                DefaultEquilibrateMaxIter = st.ClarabelEquilibrateMaxIter >= 0 ? (uint)st.ClarabelEquilibrateMaxIter : null,
            };
        }

        public bool ForceRefinement
        {
            get => Clarabel.ForceIterativeRefinement;
            set => Clarabel.ForceIterativeRefinement = value;
        }

        public void ResetSolver() => Clarabel.Reset();

        public bool Matches(int nVar, ReadOnlySpan<int> activeNodes, ReadOnlySpan<bool> floorScales,
                            ReadOnlySpan<bool> limitActive, ReadOnlySpan<bool> needOk)
            => A != null && P != null && NVar == nVar
               && ActiveNodes.AsSpan().SequenceEqual(activeNodes)
               && FloorScales.AsSpan().SequenceEqual(floorScales)
               && LimitActive.AsSpan().SequenceEqual(limitActive)
               && NeedOk.AsSpan().SequenceEqual(needOk);

        public void Dispose() => Clarabel.Dispose();
    }

    /// <summary>
    /// Dual-mode matrix sink: records entries in emission order on a pattern miss to freeze a new
    /// <see cref="CcsAssembler"/>, or writes directly into the frozen matrix and RHS on a pattern hit.
    /// Never skips zero values so refill ordinals match the frozen slot table.
    /// </summary>
    private ref struct RowSink
    {
        private readonly bool _refill;
        private readonly CcsAssembler? _a;
        private readonly double[]? _b;
        private readonly int _nEqRefill, _nIneqRefill;
        private int _eqRow, _ineqRow, _socRow;
        private readonly List<(byte Block, int Row, int Col, double Val)>? _entries;
        private readonly List<double>? _eqRhs, _ineqRhs, _socRhs;
        private readonly List<int>? _socDims;

        private RowSink(bool refill, CcsAssembler? a, double[]? b, int nEqRefill, int nIneqRefill,
                        List<(byte Block, int Row, int Col, double Val)>? entries,
                        List<double>? eqRhs, List<double>? ineqRhs, List<double>? socRhs, List<int>? socDims)
        {
            _refill = refill;
            _a = a;
            _b = b;
            _nEqRefill = nEqRefill;
            _nIneqRefill = nIneqRefill;
            _entries = entries;
            _eqRhs = eqRhs;
            _ineqRhs = ineqRhs;
            _socRhs = socRhs;
            _socDims = socDims;
        }

        public static RowSink ForRecord() => new(false, null, null, 0, 0, [], [], [], [], []);

        public static RowSink ForRefill(Workspace ws)
        {
            ws.A!.BeginRefill();
            return new(true, ws.A, ws.B, ws.NEq, ws.NIneq, null, null, null, null, null);
        }

        public int NewEq(double rhs)
        {
            if (_refill)
            {
                int row = _eqRow++;
                _b![row] = rhs;
                return row;
            }
            _eqRhs!.Add(rhs);
            return _eqRhs.Count - 1;
        }

        public int NewIneq(double rhs)
        {
            if (_refill)
            {
                int row = _ineqRow++;
                _b![_nEqRefill + row] = rhs;
                return row;
            }
            _ineqRhs!.Add(rhs);
            return _ineqRhs.Count - 1;
        }

        public int NewSoc(double rhs)
        {
            if (_refill)
            {
                int row = _socRow++;
                _b![_nEqRefill + _nIneqRefill + row] = rhs;
                return row;
            }
            _socRhs!.Add(rhs);
            return _socRhs.Count - 1;
        }

        public void AddSocDim(int dim)
        {
            if (!_refill)
                _socDims!.Add(dim);
        }

        public void AddEq(int row, int col, double value)
        {
            if (_refill)
                _a!.Add(row, col, value);
            else
                _entries!.Add((0, row, col, value));
        }

        public void AddIneq(int row, int col, double value)
        {
            if (_refill)
                _a!.Add(_nEqRefill + row, col, value);
            else
                _entries!.Add((1, row, col, value));
        }

        public void AddSoc(int row, int col, double value)
        {
            if (_refill)
                _a!.Add(_nEqRefill + _nIneqRefill + row, col, value);
            else
                _entries!.Add((2, row, col, value));
        }

        public void Add(bool isEq, int row, int col, double value)
        {
            if (isEq)
                AddEq(row, col, value);
            else
                AddIneq(row, col, value);
        }

        public void Commit(Workspace ws, int nVar)
        {
            if (_refill)
            {
                _a!.EndRefill();
                int expectedSoc = ws.B.Length - ws.NEq - ws.NIneq;
                if (_eqRow != ws.NEq || _ineqRow != ws.NIneq || _socRow != expectedSoc)
                    throw new InvalidOperationException(
                        $"ascent subproblem refill row mismatch: got ({_eqRow},{_ineqRow},{_socRow}), expected ({ws.NEq},{ws.NIneq},{expectedSoc})");
                return;
            }

            int nEq = _eqRhs!.Count;
            int nIneq = _ineqRhs!.Count;
            int nSoc = _socRhs!.Count;
            int totalRows = nEq + nIneq + nSoc;
            var a = new CcsAssembler(totalRows, nVar);
            List<(byte Block, int Row, int Col, double Val)> entries = _entries!;
            for (int i = 0; i < entries.Count; i++)
            {
                (byte block, int row, int col, double val) = entries[i];
                int baseRow = block == 0 ? 0 : block == 1 ? nEq : nEq + nIneq;
                a.Add(baseRow + row, col, val);
            }
            a.Freeze();

            var b = new double[totalRows];
            _eqRhs.CopyTo(b, 0);
            _ineqRhs.CopyTo(b, nEq);
            _socRhs.CopyTo(b, nEq + nIneq);

            ws.A = a;
            ws.B = b;
            ws.NVar = nVar;
            ws.NEq = nEq;
            ws.NIneq = nIneq;
            ws.SocDims = [.. _socDims!];
        }
    }

    public sealed record Solved(bool Ok, string Status, double[] X, double[] U, double[] Wv,
                                double[] Sigma, double[] STerm, double[] Sq, double[] Sqa, double[] Spro, double[] Sg,
                                int SolverIterations, double SolveMs, double Phi = 0.0, double[]? DR = null, double[]? SRec = null);

    /// <param name="ws">Persistent conic workspace reused across iterations of one SCvx solve.</param>
    /// <param name="eps">Clarabel primal/dual gap and feasibility tolerance for this iteration.</param>
    /// <param name="drBar">Reference reserve changes, canonical (AscentCase.Reserve.R0C), or null for none.</param>
    /// <param name="needs">Booster recovery needs at the reference separation; entries with Ok == false omit their row.</param>
    public Solved Solve(Linearization lin, double[] xbar, double[] ubar, double[] sigBar, double phiBar, double tr,
                        Workspace ws, double eps, double[]? drBar = null, BoostbackOracle.Need[]? needs = null)
    {
        drBar ??= new double[_nR];
        AscentCase c = _c;
        AscentSettings st = c.Settings;
        int n = c.N;
        int[] limited = c.LimitedNodes;

        // Precompute dynamic structural flags to decide between zero-alloc refill and fresh pattern freeze.
        if (ws.ScratchLimitThrust.Length != limited.Length)
            ws.ScratchLimitThrust = new double[limited.Length];
        if (ws.ScratchLimitActive.Length != limited.Length)
            ws.ScratchLimitActive = new bool[limited.Length];
        if (ws.ScratchFloorPerMass.Length != n)
            ws.ScratchFloorPerMass = new double[n];
        if (ws.ScratchFloorScales.Length != n)
            ws.ScratchFloorScales = new bool[n];
        if (ws.ScratchNeedOk.Length != _nNeed)
            ws.ScratchNeedOk = new bool[_nNeed];
        if (ws.ScratchVerticalThrottle.Length != c.VerticalDirections.Length)
            ws.ScratchVerticalThrottle = new double[c.VerticalDirections.Length];

        double[] limitThrust = ws.ScratchLimitThrust;
        bool[] limitActive = ws.ScratchLimitActive;
        double[] floorPerMass = ws.ScratchFloorPerMass;
        bool[] floorScales = ws.ScratchFloorScales;
        bool[] needOk = ws.ScratchNeedOk;
        double[] verticalThrottle = ws.ScratchVerticalThrottle;

        Array.Clear(floorPerMass);
        for (int g = 0; g < limited.Length; g++)
        {
            int k = limited[g], s = c.NodeStage[k];
            limitThrust[g] = c.Dyn.LiquidThrustC(s, xbar.AsSpan(k * NX, NX));
            limitActive[g] = limitThrust[g] > 0.0;
            if (limitActive[g] && c.AccC[s] * st.PathBackoff * xbar[k * NX + AscentDynamics.IM] < limitThrust[g])
                floorPerMass[k] = c.AccC[s] * st.PathBackoff / limitThrust[g];
        }
        for (int k = 0; k < n; k++)
        {
            int stage = c.NodeStage[k];
            floorScales[k] = !c.IsCoast[stage] && floorPerMass[k] > 0.0 && c.ThrottleMin[stage] > c.PhysicalThrottleMin[stage];
        }
        for (int j = 0; j < _nNeed; j++)
            needOk[j] = needs != null && j < needs.Length && needs[j].Ok;

        bool patternHit = ws.Matches(_nVar, _act, floorScales, limitActive, needOk);
        RowSink sink = patternHit ? RowSink.ForRefill(ws) : RowSink.ForRecord();

        // ---- lift-off: x_1 = x_0, and u_1 has no component across straight up.
        for (int i = 0; i < NX; i++)
            sink.AddEq(sink.NewEq(c.X0[i]), IX(0, i), 1.0);
        for (int k = 0; k < c.VerticalDirections.Length; k++)
        {
            Perpendiculars(c.VerticalDirections[k], out double[] p1, out double[] p2);
            int r1 = sink.NewEq(0.0), r2 = sink.NewEq(0.0);
            for (int j = 0; j < NU; j++)
            {
                sink.AddEq(r1, IU(k, j), p1[j]);
                sink.AddEq(r2, IU(k, j), p2[j]);
            }
            verticalThrottle[k] = AscentCase.Norm3(ubar.AsSpan(k * NU, NU));
        }

        if (c.PadTurnStep > 0.0)
        {
            for (int k = 0; k < c.Ends[0]; k++)
            {
                sink.NewSoc(c.PadTurnStep);
                for (int j = 0; j < NU; j++)
                {
                    int r = sink.NewSoc(0.0);
                    sink.AddSoc(r, IU(k, j), 1.0);
                    sink.AddSoc(r, IU(k + 1, j), -1.0);
                }
                sink.AddSocDim(4);
            }
        }

        // ---- collocation, with virtual control; staging links between the blocks.
        bool hasPhiDynamics = _turnFree && c.Dyn.HasBodyForce;
        for (int k = 0; k < n - 1; k++)
        {
            int s = c.NodeStage[k];
            if (c.CollRow[k] >= 0)
            {
                double half = 0.5 * c.Dtau[k];
                double sb = sigBar[s];
                int cw = c.CollRow[k];
                bool coast = c.IsCoast[s];
                for (int r = 0; r < NX; r++)
                {
                    // g_k = sigma f0_k + sb (A_k x_k + B_k u_k + Dphi_k phi) + c_k.
                    double ck = 0.0, ck1 = 0.0;
                    for (int q = 0; q < NX; q++)
                    {
                        ck -= sb * lin.A[(k * NX + r) * NX + q] * xbar[k * NX + q];
                        ck1 -= sb * lin.A[((k + 1) * NX + r) * NX + q] * xbar[(k + 1) * NX + q];
                    }
                    if (!coast)
                    {
                        for (int j = 0; j < NU; j++)
                        {
                            ck -= sb * lin.B[(k * NX + r) * NU + j] * ubar[k * NU + j];
                            ck1 -= sb * lin.B[((k + 1) * NX + r) * NU + j] * ubar[(k + 1) * NU + j];
                        }
                    }
                    bool phiRow = hasPhiDynamics && r is >= 3 and < 6;
                    if (phiRow)
                    {
                        ck -= sb * lin.DPhi[k * NX + r] * phiBar;
                        ck1 -= sb * lin.DPhi[(k + 1) * NX + r] * phiBar;
                    }

                    int row = sink.NewEq(half * (ck + ck1));
                    sink.AddEq(row, IX(k + 1, r), 1.0);
                    sink.AddEq(row, IX(k, r), -1.0);
                    for (int q = 0; q < NX; q++)
                    {
                        sink.AddEq(row, IX(k, q), -half * sb * lin.A[(k * NX + r) * NX + q]);
                        sink.AddEq(row, IX(k + 1, q), -half * sb * lin.A[((k + 1) * NX + r) * NX + q]);
                    }
                    if (!coast)
                    {
                        for (int j = 0; j < NU; j++)
                        {
                            sink.AddEq(row, IU(k, j), -half * sb * lin.B[(k * NX + r) * NU + j]);
                            sink.AddEq(row, IU(k + 1, j), -half * sb * lin.B[((k + 1) * NX + r) * NU + j]);
                        }
                    }
                    if (phiRow)
                        sink.AddEq(row, IPhi, -half * sb * (lin.DPhi[k * NX + r] + lin.DPhi[(k + 1) * NX + r]));
                    sink.AddEq(row, ISig(s), -half * (lin.F[k * NX + r] + lin.F[(k + 1) * NX + r]));
                    sink.AddEq(row, IW(cw, r), -1.0);
                }
            }
            else
            {
                for (int r = 0; r < NX; r++)
                {
                    int row = sink.NewEq(r == AscentDynamics.IM ? -c.DryC[s] : 0.0);
                    if (r == AscentDynamics.IM)
                        for (int v = 0; v < _nR; v++)
                            if (s == c.Reserves[v].Stage)
                                sink.AddEq(row, IDR(v), 1.0);
                    sink.AddEq(row, IX(k + 1, r), 1.0);
                    sink.AddEq(row, IX(k, r), -1.0);
                }
            }
        }

        // ---- propellant: lower runs burn their load exactly; the last run at most its load.
        foreach ((int first, int last, double prop, bool exact) in c.Loads)
        {
            int row = exact ? sink.NewEq(-prop) : sink.NewIneq(prop);
            double sign = exact ? 1.0 : -1.0;
            for (int s = first; s <= last; s++)
            {
                sink.Add(exact, row, IX(c.Ends[s], 6), sign);
                sink.Add(exact, row, IX(c.Starts[s], 6), -sign);
            }
            for (int v = 0; v < _nR; v++)
            {
                AscentCase.Reserve rsv = c.Reserves[v];
                if (last == rsv.Stage)
                    sink.Add(exact, row, IDR(v), -sign / rsv.Share);
                if (rsv.Share < 1.0 && first == rsv.Stage + 1)
                    sink.Add(exact, row, IDR(v), sign * (1.0 / rsv.Share - 1.0));
            }
        }

        // ---- pinned burn times.
        for (int s = 0; s < c.S; s++)
            if (double.IsFinite(c.SigFixed[s]))
                sink.AddEq(sink.NewEq(c.SigFixed[s]), ISig(s), 1.0);

        // ---- coast control pinning: u_k = [0, 0, 0].
        for (int k = 0; k < n; k++)
        {
            if (!c.IsCoast[c.NodeStage[k]])
                continue;
            for (int j = 0; j < NU; j++)
                sink.AddEq(sink.NewEq(0.0), IU(k, j), 1.0);
        }

        // ---- per-node: throttle floor (burn stages only), mass floor, ground floor.
        for (int k = 0; k < n; k++)
        {
            int stage = c.NodeStage[k];
            if (!c.IsCoast[stage])
            {
                double ux = ubar[k * NU], uy = ubar[k * NU + 1], uz = ubar[k * NU + 2];
                if (k < c.VerticalDirections.Length)
                {
                    ux = verticalThrottle[k] * c.VerticalDirections[k][0];
                    uy = verticalThrottle[k] * c.VerticalDirections[k][1];
                    uz = verticalThrottle[k] * c.VerticalDirections[k][2];
                }
                double un = Math.Sqrt(ux * ux + uy * uy + uz * uz);
                double invUn = un > 1e-12 ? -1.0 / un : 0.0;
                double floor = c.ThrottleMin[stage], physicalFloor = c.PhysicalThrottleMin[stage];
                bool scalesWithMass = floorScales[k];
                int row = sink.NewIneq(scalesWithMass ? 0.0 : -Math.Max(floorPerMass[k] > 0.0 ? 0.0 : floor, physicalFloor));
                if (scalesWithMass)
                    sink.AddIneq(row, IX(k, AscentDynamics.IM), floor * floorPerMass[k]);
                // Always emit all 3 control slots (0.0 if un <= 1e-12) to keep CSC sparsity invariant.
                sink.AddIneq(row, IU(k, 0), ux * invUn);
                sink.AddIneq(row, IU(k, 1), uy * invUn);
                sink.AddIneq(row, IU(k, 2), uz * invUn);
                if (scalesWithMass && physicalFloor > 0.0)
                {
                    row = sink.NewIneq(-physicalFloor);
                    sink.AddIneq(row, IU(k, 0), ux * invUn);
                    sink.AddIneq(row, IU(k, 1), uy * invUn);
                    sink.AddIneq(row, IU(k, 2), uz * invUn);
                }
            }

            sink.AddIneq(sink.NewIneq(-c.MFloor), IX(k, 6), -1.0);

            double rx = xbar[k * NX], ry = xbar[k * NX + 1], rz = xbar[k * NX + 2];
            double rn = Math.Sqrt(rx * rx + ry * ry + rz * rz);
            double invRn = rn > 1e-12 ? -1.0 / rn : 0.0;
            int groundRow = sink.NewIneq(-c.RFloor);
            sink.AddIneq(groundRow, IX(k, 0), rx * invRn);
            sink.AddIneq(groundRow, IX(k, 1), ry * invRn);
            sink.AddIneq(groundRow, IX(k, 2), rz * invRn);
        }

        // ---- q (tangent, backed off, slacked) and on burn nodes: q-alpha cone and prograde halfspace.
        for (int a = 0; a < _act.Length; a++)
        {
            int k = _act[a];
            bool coast = c.IsCoast[c.NodeStage[k]];
            double q0 = lin.Q[k];
            double jqXbar = 0.0;
            for (int i = 0; i < NX; i++) jqXbar += lin.Jq[k * NX + i] * xbar[k * NX + i];

            // (q0 + Jq (x - xbar)) / qmax <= backoff + s_q
            int row = sink.NewIneq(st.PathBackoff - (q0 - jqXbar) / c.QMax);
            for (int i = 0; i < NX; i++)
                sink.AddIneq(row, IX(k, i), lin.Jq[k * NX + i] / c.QMax);
            sink.AddIneq(row, ISq(a), -1.0);

            if (coast)
                continue;

            // ||w0 + JwX dx + JwU du|| <= (QAmax / q0)(backoff + s_qa) - (|w0| / q0) Jq dx
            double w0x = lin.W[k * 3], w0y = lin.W[k * 3 + 1], w0z = lin.W[k * 3 + 2];
            double w0n = Math.Sqrt(w0x * w0x + w0y * w0y + w0z * w0z);
            int t = sink.NewSoc(c.QAlphaMax / q0 * st.PathBackoff + w0n / q0 * jqXbar);
            sink.AddSoc(t, ISqa(a), -c.QAlphaMax / q0);
            for (int i = 0; i < NX; i++)
                sink.AddSoc(t, IX(k, i), w0n / q0 * lin.Jq[k * NX + i]);
            for (int r = 0; r < 3; r++)
            {
                double hw = lin.W[k * 3 + r];
                for (int i = 0; i < NX; i++) hw -= lin.JwX[(k * 3 + r) * NX + i] * xbar[k * NX + i];
                for (int j = 0; j < NU; j++) hw -= lin.JwU[(k * 3 + r) * NU + j] * ubar[k * NU + j];
                int wr = sink.NewSoc(hw);
                for (int i = 0; i < NX; i++)
                    sink.AddSoc(wr, IX(k, i), -lin.JwX[(k * 3 + r) * NX + i]);
                for (int j = 0; j < NU; j++)
                    sink.AddSoc(wr, IU(k, j), -lin.JwU[(k * 3 + r) * NU + j]);
            }
            sink.AddSocDim(4);

            // Thrust within 90 deg of relative airflow: always emit row and all 3 control slots.
            double[] air = c.AirVelocity(xbar.AsSpan(k * NX, NX));
            double an = AscentCase.Norm3(air);
            double invAn = an > 1e-12 ? -1.0 / an : 0.0;
            int pro = sink.NewIneq(0.0);
            for (int j = 0; j < NU; j++)
                sink.AddIneq(pro, IU(k, j), air[j] * invAn);
            sink.AddIneq(pro, ISpro(a), -1.0);
        }
        for (int a = 0; a < _nA; a++)
        {
            sink.AddIneq(sink.NewIneq(0.0), ISq(a), -1.0);
            sink.AddIneq(sink.NewIneq(0.0), ISqa(a), -1.0);
            sink.AddIneq(sink.NewIneq(0.0), ISpro(a), -1.0);
        }
        for (int g = 0; g < limited.Length; g++)
            sink.AddIneq(sink.NewIneq(0.0), ISg(g), -1.0);

        // ---- thrust ceiling ||u_k|| <= 1 on burn nodes.
        for (int k = 0; k < n; k++)
        {
            if (c.IsCoast[c.NodeStage[k]])
                continue;
            sink.NewSoc(1.0);
            for (int j = 0; j < NU; j++)
                sink.AddSoc(sink.NewSoc(0.0), IU(k, j), -1.0);
            sink.AddSocDim(4);
        }

        // ---- thrust acceleration T_bar ||u_k|| <= a (backoff m_k + m_bar_k s^g_k), a cone per limited node.
        for (int g = 0; g < limited.Length; g++)
        {
            if (!limitActive[g])
                continue;
            int k = limited[g];
            double a = c.AccC[c.NodeStage[k]], tl = limitThrust[g];
            int top = sink.NewSoc(0.0);
            sink.AddSoc(top, IX(k, AscentDynamics.IM), -a * st.PathBackoff / tl);
            sink.AddSoc(top, ISg(g), -a * xbar[k * NX + AscentDynamics.IM] / tl);
            for (int j = 0; j < NU; j++)
                sink.AddSoc(sink.NewSoc(0.0), IU(k, j), -1.0);
            sink.AddSocDim(4);
        }

        // ---- insertion, linearised about the last node, with slacks; and prograde.
        {
            int o = (n - 1) * NX;
            double rbx = xbar[o], rby = xbar[o + 1], rbz = xbar[o + 2];
            double vbx = xbar[o + 3], vby = xbar[o + 4], vbz = xbar[o + 5];
            double rbn = Math.Max(Math.Sqrt(rbx * rbx + rby * rby + rbz * rbz), 1e-12);
            double vbn = Math.Max(Math.Sqrt(vbx * vbx + vby * vby + vbz * vbz), 1e-12);
            int f = n - 1;

            int row = sink.NewEq(c.RTarget);
            sink.AddEq(row, IX(f, 0), rbx / rbn); sink.AddEq(row, IX(f, 1), rby / rbn); sink.AddEq(row, IX(f, 2), rbz / rbn);
            sink.AddEq(row, ITerm(0), -1.0);

            row = sink.NewEq(c.VTarget);
            sink.AddEq(row, IX(f, 3), vbx / vbn); sink.AddEq(row, IX(f, 4), vby / vbn); sink.AddEq(row, IX(f, 5), vbz / vbn);
            sink.AddEq(row, ITerm(1), -1.0);

            double rbvb = rbx * vbx + rby * vby + rbz * vbz;
            row = sink.NewEq(rbvb + c.RvTarget);
            sink.AddEq(row, IX(f, 0), vbx); sink.AddEq(row, IX(f, 1), vby); sink.AddEq(row, IX(f, 2), vbz);
            sink.AddEq(row, IX(f, 3), rbx); sink.AddEq(row, IX(f, 4), rby); sink.AddEq(row, IX(f, 5), rbz);
            sink.AddEq(row, ITerm(2), -1.0);

            if (!_turnFree)
                phiBar = c.PlaneTurn0;
            double[] nh = c.PlaneNormal(phiBar);
            double[] dn = [-nh[1], nh[0], 0.0];
            double dnr = dn[0] * rbx + dn[1] * rby + dn[2] * rbz;
            double dnv = dn[0] * vbx + dn[1] * vby + dn[2] * vbz;
            row = sink.NewEq(_turnFree ? dnr * phiBar : 0.0);
            for (int i = 0; i < 3; i++) sink.AddEq(row, IX(f, i), nh[i]);
            if (_turnFree) sink.AddEq(row, IPhi, dnr);
            sink.AddEq(row, ITerm(3), -1.0);
            row = sink.NewEq(_turnFree ? dnv * phiBar : 0.0);
            for (int i = 0; i < 3; i++) sink.AddEq(row, IX(f, 3 + i), nh[i]);
            if (_turnFree) sink.AddEq(row, IPhi, dnv);
            sink.AddEq(row, ITerm(4), -1.0);

            double hx = rby * vbz - rbz * vby, hy = rbz * vbx - rbx * vbz, hz = rbx * vby - rby * vbx;
            double hpro = hx * nh[0] + hy * nh[1] + hz * nh[2];
            double[] vxn = [vby * nh[2] - vbz * nh[1], vbz * nh[0] - vbx * nh[2], vbx * nh[1] - vby * nh[0]];
            double[] nxr = [nh[1] * rbz - nh[2] * rby, nh[2] * rbx - nh[0] * rbz, nh[0] * rby - nh[1] * rbx];
            double rhs = hpro - (vxn[0] * rbx + vxn[1] * rby + vxn[2] * rbz) - (nxr[0] * vbx + nxr[1] * vby + nxr[2] * vbz);
            row = sink.NewIneq(rhs);
            for (int i = 0; i < 3; i++)
            {
                sink.AddIneq(row, IX(f, i), -vxn[i]);
                sink.AddIneq(row, IX(f, 3 + i), -nxr[i]);
            }
        }

        // ---- trust region: box on x (scaled), u (accel-normalised on burn nodes), sigma (stage-scaled), and bounds.
        for (int k = 0; k < n; k++)
        {
            for (int i = 0; i < NX; i++)
            {
                double xb = xbar[k * NX + i], w = tr * c.XScale[i];
                sink.AddIneq(sink.NewIneq(xb + w), IX(k, i), 1.0);
                sink.AddIneq(sink.NewIneq(-xb + w), IX(k, i), -1.0);
            }
            if (!c.IsCoast[c.NodeStage[k]])
            {
                double wu = tr * c.ControlTrustScale(k, xbar.AsSpan(k * NX, NX));
                for (int j = 0; j < NU; j++)
                {
                    double ub = k < c.VerticalDirections.Length
                        ? verticalThrottle[k] * c.VerticalDirections[k][j]
                        : ubar[k * NU + j];
                    sink.AddIneq(sink.NewIneq(ub + wu), IU(k, j), 1.0);
                    sink.AddIneq(sink.NewIneq(-ub + wu), IU(k, j), -1.0);
                }
            }
        }
        for (int s = 0; s < c.S; s++)
        {
            if (double.IsFinite(c.SigFixed[s]))
                continue;
            double w = tr * c.StageSigScale(s);
            sink.AddIneq(sink.NewIneq(sigBar[s] + w), ISig(s), 1.0);
            sink.AddIneq(sink.NewIneq(-sigBar[s] + w), ISig(s), -1.0);
            sink.AddIneq(sink.NewIneq(-c.SigMin[s]), ISig(s), -1.0);
            sink.AddIneq(sink.NewIneq(c.SigMax[s]), ISig(s), 1.0);
        }
        if (_turnFree)
        {
            double w = tr * st.PlaneTurnTrust;
            sink.AddIneq(sink.NewIneq(phiBar + w), IPhi, 1.0);
            sink.AddIneq(sink.NewIneq(-phiBar + w), IPhi, -1.0);
            sink.AddIneq(sink.NewIneq(c.PlaneTurnMax), IPhi, 1.0);
            sink.AddIneq(sink.NewIneq(-c.PlaneTurnMin), IPhi, -1.0);
            int up = sink.NewIneq(0.0);
            sink.AddIneq(up, IPhi, 1.0);
            sink.AddIneq(up, IPhiAbs, -1.0);
            int dn = sink.NewIneq(0.0);
            sink.AddIneq(dn, IPhi, -1.0);
            sink.AddIneq(dn, IPhiAbs, -1.0);
        }

        // ---- booster reserves and recovery rows.
        for (int v = 0; v < _nR; v++)
        {
            AscentCase.Reserve rsv = c.Reserves[v];
            double w = tr * st.RecoveryTrust;
            sink.AddIneq(sink.NewIneq(drBar[v] + w), IDR(v), 1.0);
            sink.AddIneq(sink.NewIneq(-drBar[v] + w), IDR(v), -1.0);
            sink.AddIneq(sink.NewIneq(rsv.RMaxC), IDR(v), 1.0);
            sink.AddIneq(sink.NewIneq(-rsv.RMinC), IDR(v), -1.0);
        }
        for (int j = 0; j < _nNeed; j++)
        {
            sink.AddIneq(sink.NewIneq(0.0), ISRec(j), -1.0);
            if (!needOk[j])
                continue;
            BoostbackOracle.Need need = needs![j];
            int v = c.Needs[j].Reserve;
            AscentCase.Reserve rsv = c.Reserves[v];
            AscentDynamics d = c.Dyn;
            int ks = c.Ends[rsv.Stage];
            double needC = need.RequiredKg / d.MU;
            double rhs = rsv.R0C - needC;
            for (int i = 0; i < 3; i++)
            {
                double gr = need.GradPosition[i] * d.LU / d.MU;
                double gv = need.GradVelocity[i] * d.VU / d.MU;
                rhs += gr * xbar[ks * NX + i] + gv * xbar[ks * NX + 3 + i];
            }
            double gt = need.GradTime * d.TU / d.MU;
            for (int s = 0; s <= rsv.Stage; s++)
                rhs += gt * sigBar[s];
            int row = sink.NewIneq(rhs);
            for (int i = 0; i < 3; i++)
            {
                sink.AddIneq(row, IX(ks, i), need.GradPosition[i] * d.LU / d.MU);
                sink.AddIneq(row, IX(ks, 3 + i), need.GradVelocity[i] * d.VU / d.MU);
            }
            for (int s = 0; s <= rsv.Stage; s++)
                sink.AddIneq(row, ISig(s), gt);
            sink.AddIneq(row, IDR(v), -1.0);
            sink.AddIneq(row, ISRec(j), -1.0);
        }

        sink.Commit(ws, _nVar);

        // Objective vectors C and P depend only on structural layout, so rebuild them only on a pattern miss.
        if (!patternHit)
        {
            var cvec = new double[_nVar];
            cvec[IX(n - 1, 6)] = -1.0;
            for (int a = 0; a < _nA; a++)
            {
                cvec[ISq(a)] = st.RhoPath;
                cvec[ISqa(a)] = st.RhoPath;
                cvec[ISpro(a)] = st.RhoPath;
            }
            for (int g = 0; g < limited.Length; g++)
                cvec[ISg(g)] = st.RhoAccel;
            if (_turnFree)
                cvec[IPhiAbs] = st.PlaneTurnPrice;
            for (int j = 0; j < _nNeed; j++)
                cvec[ISRec(j)] = st.RhoRecovery;
            ws.C = cvec;

            // Upper triangle only, as ClarabelWorkspace requires.
            var p = new CcsAssembler(_nVar, _nVar);
            double wdt = st.SmoothingWeight;
            for (int k = 0; k < n - 1; k++)
            {
                if (c.IsCoast[c.NodeStage[k]] || c.IsCoast[c.NodeStage[k + 1]])
                    continue;
                for (int j = 0; j < NU; j++)
                {
                    int ia = IU(k, j), ib = IU(k + 1, j);
                    p.Add(ia, ia, 2.0 * wdt);
                    p.Add(ib, ib, 2.0 * wdt);
                    p.Add(ia, ib, -2.0 * wdt);
                }
            }
            for (int cw = 0; cw < c.Coll.Length; cw++)
                for (int i = 0; i < NX; i++)
                {
                    double d = c.DScale[i];
                    p.Add(IW(cw, i), IW(cw, i), 2.0 * st.RhoVirtualControl / (d * d));
                }
            for (int j = 0; j < 5; j++)
                p.Add(ITerm(j), ITerm(j), 2.0 * st.RhoTerminal);
            p.Freeze();
            ws.P = p;

            ws.ActiveNodes = (int[])_act.Clone();
            ws.FloorScales = (bool[])floorScales.Clone();
            ws.LimitActive = (bool[])limitActive.Clone();
            ws.NeedOk = (bool[])needOk.Clone();
        }

        ws.Clarabel.Solve(ws.A!, ws.B, ws.C, ws.P, ws.NEq, ws.NIneq, ws.SocDims,
            warmStart: false, verbose: false, maxIterations: st.SubproblemMaxIterations,
            epsAbs: eps, epsRel: eps);

        ClarabelExit nativeStatus = ws.Clarabel.NativeStatus;
        string statusText = nativeStatus switch
        {
            ClarabelExit.Solved => "Optimal",
            ClarabelExit.AlmostSolved => "OptimalInaccurate",
            ClarabelExit.PrimalInfeasible => "PrimalInfeasible",
            ClarabelExit.AlmostPrimalInfeasible => "PrimalInfeasibleInaccurate",
            ClarabelExit.DualInfeasible => "DualInfeasible",
            ClarabelExit.AlmostDualInfeasible => "DualInfeasibleInaccurate",
            ClarabelExit.MaxIterations or ClarabelExit.MaxTime => "MaxIterations",
            ClarabelExit.InsufficientProgress or ClarabelExit.NumericalError => "Numerics",
            ClarabelExit.CallbackTerminated => "Interrupted",
            _ => nativeStatus.ToString(),
        };
        double solveMs = ws.Clarabel.LastSetupMs + ws.Clarabel.LastSolveMs;
        bool ok = nativeStatus is ClarabelExit.Solved or ClarabelExit.AlmostSolved && ws.Clarabel.X.Length == _nVar;
        if (!ok)
            return new Solved(false, statusText, [], [], [], [], [], [], [], [], [], ws.Clarabel.Iterations, solveMs);

        double[] x = ws.Clarabel.X;
        return new Solved(true, statusText,
            x[..(n * NX)], x[_oU..(_oU + n * NU)], x[_oW..(_oW + c.Coll.Length * NX)],
            x[_oSig..(_oSig + c.S)], x[_oTerm..(_oTerm + 5)], x[_oSq..(_oSq + _nA)], x[_oSqa..(_oSqa + _nA)], x[_oSpro..(_oSpro + _nA)],
            x[_oSg..(_oSg + limited.Length)], ws.Clarabel.Iterations, solveMs, _turnFree ? x[_oPhi] : c.PlaneTurn0,
            x[_oDR..(_oDR + _nR)], x[_oSRec..(_oSRec + _nNeed)]);
    }

    /// <summary>Two unit vectors perpendicular to d and to each other.</summary>
    private static void Perpendiculars(double[] d, out double[] p1, out double[] p2)
    {
        double[] refv = Math.Abs(d[2]) < 0.9 ? [0.0, 0.0, 1.0] : [1.0, 0.0, 0.0];
        double ax = refv[1] * d[2] - refv[2] * d[1];
        double ay = refv[2] * d[0] - refv[0] * d[2];
        double az = refv[0] * d[1] - refv[1] * d[0];
        double an = Math.Sqrt(ax * ax + ay * ay + az * az);
        p1 = [ax / an, ay / an, az / an];
        p2 = [d[1] * p1[2] - d[2] * p1[1], d[2] * p1[0] - d[0] * p1[2], d[0] * p1[1] - d[1] * p1[0]];
    }
}

/// <summary>Every node's linearisation, flat, node-major.</summary>
internal sealed class Linearization
{
    public readonly double[] F, A, B, DPhi, W, JwX, JwU, Q, Jq;

    public Linearization(int n)
    {
        const int nx = AscentCase.NX, nu = AscentCase.NU;
        F = new double[n * nx];
        A = new double[n * nx * nx];
        B = new double[n * nx * nu];
        DPhi = new double[n * nx];
        W = new double[n * 3];
        JwX = new double[n * 3 * nx];
        JwU = new double[n * 3 * nu];
        Q = new double[n];
        Jq = new double[n * nx];
    }

    public void Fill(AscentCase c, double[] xbar, double[] ubar, double phiBar = 0.0)
    {
        const int nx = AscentCase.NX, nu = AscentCase.NU;
        bool needPhiDeriv = c.PlaneTurnMin < c.PlaneTurnMax && c.Dyn.HasBodyForce;
        for (int k = 0; k < c.N; k++)
        {
            c.Dyn.Linearize(xbar.AsSpan(k * nx, nx), ubar.AsSpan(k * nu, nu), k,
                F.AsSpan(k * nx, nx), A.AsSpan(k * nx * nx, nx * nx), B.AsSpan(k * nx * nu, nx * nu),
                W.AsSpan(k * 3, 3), JwX.AsSpan(k * 3 * nx, 3 * nx), JwU.AsSpan(k * 3 * nu, 3 * nu),
                out Q[k], Jq.AsSpan(k * nx, nx),
                phiBar, needPhiDeriv ? DPhi.AsSpan(k * nx, nx) : default);
        }
    }
}

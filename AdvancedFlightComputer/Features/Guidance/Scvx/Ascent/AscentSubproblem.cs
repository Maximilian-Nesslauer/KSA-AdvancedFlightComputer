using AdvancedFlightComputer.Guidance.Conic;

namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// The convex subproblem solved at each SCvx iteration, assembled for Clarabel. Constraint for constraint the problem of launch3dof.py's loop, which is set out in "The Saturn V Ascent Problem" section 05:
///
///   minimise   (1 - m_N) + lambda sum ||u_k+1 - u_k||^2
///              + rho_nu sum ||D^-1 nu_k||^2 + rho_s ||s||^2 + rho_q sum (s^q_k + s^qa_k)
///   s.t.       x_1 = x_0,  u_1 parallel to straight up                 lift-off
///              x_k+1 = x_k + dtau/2 (g_k + g_k+1) + nu_k               collocation
///              r, v continuous, m drops by the dry mass                staging links
///              lower stages burn their load, the last at most its load propellant
///              sigma_i = the solids' burn where one pins it               solids (#73)
///              ||u_k|| &lt;= 1,  u_bar_hat . u_k &gt;= throttle floor        throttle
///              r_bar_hat . r_k &gt;= R_floor,  m_k &gt;= m_floor              ground, structure
///              tangent q &lt;= backoff + s^q,  cone q-alpha &lt;= backoff + s^qa   where q_bar &gt; QActive
///              -v_rel_bar_hat . u_k &lt;= s^pro                          thrust never against the airflow, there too
///              linearised insertion = s,  linearised prograde &gt;= 0     orbit
///              trust box on x, u, sigma;  sigma_min &lt;= sigma &lt;= sigma_max
///
/// with g_k = sigma_i f_k + sigma_bar_i (A_k dx_k + B_k du_k), the time-scaled dynamics linearised in all three of sigma, x and u.
///
/// A "stage" in the propellant rows is really a run of stages that share one liquid load: a core lit with its boosters keeps burning after they burn out, and only the load's total is fixed, not how it splits across the burnout (AscentCase.Loads).
///
/// Two departures, both conditioning rather than modelling. The lift-off thrust constraint is two rows (the two directions perpendicular to straight up) instead of the script's three rank-2 rows. And the whole problem is assembled fresh each iteration, as CVXPY rebuilt it: the set of nodes carrying a path limit changes with the reference, so the structure is not fixed.
/// </summary>
internal sealed class AscentSubproblem
{
    private const int NX = AscentCase.NX;
    private const int NU = AscentCase.NU;

    private readonly AscentCase _c;

    // Variable layout.
    private readonly int _oU, _oW, _oSig, _oTerm, _oSq, _oSqa, _oSpro, _nVar;
    private readonly int _nA;
    private readonly int[] _act;

    public AscentSubproblem(AscentCase c, int[] activeNodes)
    {
        _c = c;
        _act = activeNodes;
        // The script sizes its path buffers max(len(act), 1): keep that, an unconstrained buffer is simply priced to zero.
        _nA = Math.Max(activeNodes.Length, 1);
        _oU = c.N * NX;
        _oW = _oU + c.N * NU;
        _oSig = _oW + c.Coll.Length * NX;
        _oTerm = _oSig + c.S;
        _oSq = _oTerm + 5;
        _oSqa = _oSq + _nA;
        _oSpro = _oSqa + _nA;
        _nVar = _oSpro + _nA;
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

    /// <summary>A growable block of rows: triplets and a right-hand side.</summary>
    private sealed class Rows
    {
        public readonly List<(int Row, int Col, double Val)> Entries = [];
        public readonly List<double> Rhs = [];
        public int Count => Rhs.Count;

        public int NewRow(double rhs)
        {
            Rhs.Add(rhs);
            return Rhs.Count - 1;
        }

        public void Add(int row, int col, double value)
        {
            if (value != 0.0)
                Entries.Add((row, col, value));
        }
    }

    public sealed record Solved(bool Ok, string Status, double[] X, double[] U, double[] Wv,
                                double[] Sigma, double[] STerm, double[] Sq, double[] Sqa, double[] Spro,
                                int SolverIterations, double SolveMs);

    public Solved Solve(Linearization lin, double[] xbar, double[] ubar, double[] sigBar, double tr)
    {
        AscentCase c = _c;
        AscentSettings st = c.Settings;
        int n = c.N;

        var eq = new Rows();
        var ineq = new Rows();
        var soc = new Rows();
        var socDims = new List<int>();

        // ---- lift-off: x_1 = x_0, and u_1 has no component across straight up.
        for (int i = 0; i < NX; i++)
            eq.Add(eq.NewRow(c.X0[i]), IX(0, i), 1.0);
        Perpendiculars(c.LiftoffDir, out double[] p1, out double[] p2);
        int r1 = eq.NewRow(0.0), r2 = eq.NewRow(0.0);
        for (int j = 0; j < NU; j++)
        {
            eq.Add(r1, IU(0, j), p1[j]);
            eq.Add(r2, IU(0, j), p2[j]);
        }

        // ---- collocation, with virtual control; staging links between the blocks.
        for (int k = 0; k < n - 1; k++)
        {
            int s = c.NodeStage[k];
            if (c.CollRow[k] >= 0)
            {
                double half = 0.5 * c.Dtau[k];
                double sb = sigBar[s];
                int cw = c.CollRow[k];
                for (int r = 0; r < NX; r++)
                {
                    // g_k = sigma f0_k + sb (A_k x_k + B_k u_k) + c_k, with c_k = -sb (A_k xbar_k + B_k ubar_k).
                    double ck = 0.0, ck1 = 0.0;
                    for (int q = 0; q < NX; q++)
                    {
                        ck -= sb * lin.A[(k * NX + r) * NX + q] * xbar[k * NX + q];
                        ck1 -= sb * lin.A[((k + 1) * NX + r) * NX + q] * xbar[(k + 1) * NX + q];
                    }
                    for (int j = 0; j < NU; j++)
                    {
                        ck -= sb * lin.B[(k * NX + r) * NU + j] * ubar[k * NU + j];
                        ck1 -= sb * lin.B[((k + 1) * NX + r) * NU + j] * ubar[(k + 1) * NU + j];
                    }

                    int row = eq.NewRow(half * (ck + ck1));
                    eq.Add(row, IX(k + 1, r), 1.0);
                    eq.Add(row, IX(k, r), -1.0);
                    for (int q = 0; q < NX; q++)
                    {
                        eq.Add(row, IX(k, q), -half * sb * lin.A[(k * NX + r) * NX + q]);
                        eq.Add(row, IX(k + 1, q), -half * sb * lin.A[((k + 1) * NX + r) * NX + q]);
                    }
                    for (int j = 0; j < NU; j++)
                    {
                        eq.Add(row, IU(k, j), -half * sb * lin.B[(k * NX + r) * NU + j]);
                        eq.Add(row, IU(k + 1, j), -half * sb * lin.B[((k + 1) * NX + r) * NU + j]);
                    }
                    eq.Add(row, ISig(s), -half * (lin.F[k * NX + r] + lin.F[(k + 1) * NX + r]));
                    eq.Add(row, IW(cw, r), -1.0);
                }
            }
            else
            {
                for (int r = 0; r < NX; r++)
                {
                    int row = eq.NewRow(r == AscentDynamics.IM ? -c.DryC[s] : 0.0);
                    eq.Add(row, IX(k + 1, r), 1.0);
                    eq.Add(row, IX(k, r), -1.0);
                }
            }
        }

        // ---- propellant: the lower stages burn exactly their load; the last at most its load. A load is per run of stages sharing one (see AscentCase.Loads): the sum of the run's drops, each stage's own from its first node to its last, so the jettisons between them stay out of it.
        foreach ((int first, int last, double prop, bool exact) in c.Loads)
        {
            int row = exact ? eq.NewRow(-prop) : ineq.NewRow(prop);
            Rows rows = exact ? eq : ineq;
            double sign = exact ? 1.0 : -1.0;
            for (int s = first; s <= last; s++)
            {
                rows.Add(row, IX(c.Ends[s], 6), sign);
                rows.Add(row, IX(c.Starts[s], 6), -sign);
            }
        }

        // ---- a pinned burn time: a solid's burn, or a liquid held at full thrust, sets it.
        for (int s = 0; s < c.S; s++)
            if (double.IsFinite(c.SigFixed[s]))
                eq.Add(eq.NewRow(c.SigFixed[s]), ISig(s), 1.0);

        // ---- per-node: throttle floor (tangent halfspace), mass floor, ground (tangent halfspace).
        for (int k = 0; k < n; k++)
        {
            double ux = ubar[k * NU], uy = ubar[k * NU + 1], uz = ubar[k * NU + 2];
            double un = Math.Sqrt(ux * ux + uy * uy + uz * uz);
            int row = ineq.NewRow(-c.ThrottleMin[c.NodeStage[k]]);
            if (un > 1e-12)
            {
                ineq.Add(row, IU(k, 0), -ux / un);
                ineq.Add(row, IU(k, 1), -uy / un);
                ineq.Add(row, IU(k, 2), -uz / un);
            }

            row = ineq.NewRow(-c.MFloor);
            ineq.Add(row, IX(k, 6), -1.0);

            double rx = xbar[k * NX], ry = xbar[k * NX + 1], rz = xbar[k * NX + 2];
            double rn = Math.Sqrt(rx * rx + ry * ry + rz * rz);
            row = ineq.NewRow(-c.RFloor);
            ineq.Add(row, IX(k, 0), -rx / rn);
            ineq.Add(row, IX(k, 1), -ry / rn);
            ineq.Add(row, IX(k, 2), -rz / rn);
        }

        // ---- q (tangent, backed off, slacked) and q-alpha (a cone on the linearised w, backed off, slacked).
        for (int a = 0; a < _act.Length; a++)
        {
            int k = _act[a];
            double q0 = lin.Q[k];
            double jqXbar = 0.0;
            for (int i = 0; i < NX; i++) jqXbar += lin.Jq[k * NX + i] * xbar[k * NX + i];

            // (q0 + Jq (x - xbar)) / qmax <= backoff + s_q
            int row = ineq.NewRow(st.PathBackoff - (q0 - jqXbar) / c.QMax);
            for (int i = 0; i < NX; i++)
                ineq.Add(row, IX(k, i), lin.Jq[k * NX + i] / c.QMax);
            ineq.Add(row, ISq(a), -1.0);

            // ||w0 + JwX dx + JwU du|| <= (QAmax / q0)(backoff + s_qa) - (|w0| / q0) Jq dx
            double w0x = lin.W[k * 3], w0y = lin.W[k * 3 + 1], w0z = lin.W[k * 3 + 2];
            double w0n = Math.Sqrt(w0x * w0x + w0y * w0y + w0z * w0z);
            int t = soc.NewRow(c.QAlphaMax / q0 * st.PathBackoff + w0n / q0 * jqXbar);
            soc.Add(t, ISqa(a), -c.QAlphaMax / q0);
            for (int i = 0; i < NX; i++)
                soc.Add(t, IX(k, i), w0n / q0 * lin.Jq[k * NX + i]);
            for (int r = 0; r < 3; r++)
            {
                double hw = lin.W[k * 3 + r];
                for (int i = 0; i < NX; i++) hw -= lin.JwX[(k * 3 + r) * NX + i] * xbar[k * NX + i];
                for (int j = 0; j < NU; j++) hw -= lin.JwU[(k * 3 + r) * NU + j] * ubar[k * NU + j];
                int wr = soc.NewRow(hw);
                for (int i = 0; i < NX; i++)
                    soc.Add(wr, IX(k, i), -lin.JwX[(k * 3 + r) * NX + i]);
                for (int j = 0; j < NU; j++)
                    soc.Add(wr, IU(k, j), -lin.JwU[(k * 3 + r) * NU + j]);
            }
            socDims.Add(4);

            // Thrust within 90 deg of the air-relative velocity, as a halfspace on the reference's airflow direction, slacked: -v_rel_hat . u <= s_pro. q-alpha is measured by sin(alpha), which is back to zero at 180 deg, so without this a stack whose thrust it cannot throttle meets the q limit by firing against the airflow to bleed off speed - a trajectory no speed-indexed profile can fly (#73). It never binds on a sane ascent.
            double[] air = c.AirVelocity(xbar.AsSpan(k * NX, NX));
            double an = AscentCase.Norm3(air);
            if (an > 1e-12)
            {
                int pro = ineq.NewRow(0.0);
                for (int j = 0; j < NU; j++)
                    ineq.Add(pro, IU(k, j), -air[j] / an);
                ineq.Add(pro, ISpro(a), -1.0);
            }
        }
        for (int a = 0; a < _nA; a++)
        {
            ineq.Add(ineq.NewRow(0.0), ISq(a), -1.0);
            ineq.Add(ineq.NewRow(0.0), ISqa(a), -1.0);
            ineq.Add(ineq.NewRow(0.0), ISpro(a), -1.0);
        }

        // ---- thrust ceiling ||u_k|| <= 1, a cone per node.
        for (int k = 0; k < n; k++)
        {
            soc.NewRow(1.0);
            for (int j = 0; j < NU; j++)
                soc.Add(soc.NewRow(0.0), IU(k, j), -1.0);
            socDims.Add(4);
        }

        // ---- insertion, linearised about the last node, with slacks; and prograde.
        {
            int o = (n - 1) * NX;
            double rbx = xbar[o], rby = xbar[o + 1], rbz = xbar[o + 2];
            double vbx = xbar[o + 3], vby = xbar[o + 4], vbz = xbar[o + 5];
            double rbn = Math.Sqrt(rbx * rbx + rby * rby + rbz * rbz);
            double vbn = Math.Sqrt(vbx * vbx + vby * vby + vbz * vbz);
            int f = n - 1;

            int row = eq.NewRow(c.RTarget);
            eq.Add(row, IX(f, 0), rbx / rbn); eq.Add(row, IX(f, 1), rby / rbn); eq.Add(row, IX(f, 2), rbz / rbn);
            eq.Add(row, ITerm(0), -1.0);

            row = eq.NewRow(c.VTarget);
            eq.Add(row, IX(f, 3), vbx / vbn); eq.Add(row, IX(f, 4), vby / vbn); eq.Add(row, IX(f, 5), vbz / vbn);
            eq.Add(row, ITerm(1), -1.0);

            double rbvb = rbx * vbx + rby * vby + rbz * vbz;
            row = eq.NewRow(rbvb + c.RvTarget);
            eq.Add(row, IX(f, 0), vbx); eq.Add(row, IX(f, 1), vby); eq.Add(row, IX(f, 2), vbz);
            eq.Add(row, IX(f, 3), rbx); eq.Add(row, IX(f, 4), rby); eq.Add(row, IX(f, 5), rbz);
            eq.Add(row, ITerm(2), -1.0);

            double[] nh = c.Nhat;
            row = eq.NewRow(0.0);
            for (int i = 0; i < 3; i++) eq.Add(row, IX(f, i), nh[i]);
            eq.Add(row, ITerm(3), -1.0);
            row = eq.NewRow(0.0);
            for (int i = 0; i < 3; i++) eq.Add(row, IX(f, 3 + i), nh[i]);
            eq.Add(row, ITerm(4), -1.0);

            // (rb x vb).n + (vb x n).(rf - rb) + (n x rb).(vf - vb) >= 0
            double hx = rby * vbz - rbz * vby, hy = rbz * vbx - rbx * vbz, hz = rbx * vby - rby * vbx;
            double hpro = hx * nh[0] + hy * nh[1] + hz * nh[2];
            double[] vxn = [vby * nh[2] - vbz * nh[1], vbz * nh[0] - vbx * nh[2], vbx * nh[1] - vby * nh[0]];
            double[] nxr = [nh[1] * rbz - nh[2] * rby, nh[2] * rbx - nh[0] * rbz, nh[0] * rby - nh[1] * rbx];
            double rhs = hpro - (vxn[0] * rbx + vxn[1] * rby + vxn[2] * rbz) - (nxr[0] * vbx + nxr[1] * vby + nxr[2] * vbz);
            row = ineq.NewRow(rhs);
            for (int i = 0; i < 3; i++)
            {
                ineq.Add(row, IX(f, i), -vxn[i]);
                ineq.Add(row, IX(f, 3 + i), -nxr[i]);
            }
        }

        // ---- trust region: a box on x (scaled), u, and sigma; and sigma's own bounds.
        for (int k = 0; k < n; k++)
        {
            for (int i = 0; i < NX; i++)
            {
                double xb = xbar[k * NX + i], w = tr * c.XScale[i];
                ineq.Add(ineq.NewRow(xb + w), IX(k, i), 1.0);
                ineq.Add(ineq.NewRow(-xb + w), IX(k, i), -1.0);
            }
            for (int j = 0; j < NU; j++)
            {
                double ub = ubar[k * NU + j];
                ineq.Add(ineq.NewRow(ub + tr), IU(k, j), 1.0);
                ineq.Add(ineq.NewRow(-ub + tr), IU(k, j), -1.0);
            }
        }
        for (int s = 0; s < c.S; s++)
        {
            if (double.IsFinite(c.SigFixed[s]))
                continue;
            double w = tr * c.SigScale;
            ineq.Add(ineq.NewRow(sigBar[s] + w), ISig(s), 1.0);
            ineq.Add(ineq.NewRow(-sigBar[s] + w), ISig(s), -1.0);
            ineq.Add(ineq.NewRow(-c.SigMin[s]), ISig(s), -1.0);
            ineq.Add(ineq.NewRow(c.SigMax[s]), ISig(s), 1.0);
        }

        // ---- objective.
        var cvec = new double[_nVar];
        cvec[IX(n - 1, 6)] = -1.0;
        for (int a = 0; a < _nA; a++)
        {
            cvec[ISq(a)] = st.RhoPath;
            cvec[ISqa(a)] = st.RhoPath;
            cvec[ISpro(a)] = st.RhoPath;
        }

        var p = new SparseCcs(_nVar, _nVar);
        double wdt = st.SmoothingWeight;
        for (int k = 0; k < n - 1; k++)
            for (int j = 0; j < NU; j++)
            {
                int ia = IU(k, j), ib = IU(k + 1, j);
                p.Add(ia, ia, 2.0 * wdt);
                p.Add(ib, ib, 2.0 * wdt);
                p.Add(ia, ib, -2.0 * wdt);
            }
        for (int cw = 0; cw < c.Coll.Length; cw++)
            for (int i = 0; i < NX; i++)
            {
                double d = c.DScale[i];
                p.Add(IW(cw, i), IW(cw, i), 2.0 * st.RhoVirtualControl / (d * d));
            }
        for (int j = 0; j < 5; j++)
            p.Add(ITerm(j), ITerm(j), 2.0 * st.RhoTerminal);

        // ---- to Clarabel: equalities, then the orthant, then the cones in order.
        var aMat = new SparseCcs(eq.Count, _nVar);
        foreach ((int row, int col, double v) in eq.Entries) aMat.Add(row, col, v);
        var gMat = new SparseCcs(ineq.Count + soc.Count, _nVar);
        foreach ((int row, int col, double v) in ineq.Entries) gMat.Add(row, col, v);
        foreach ((int row, int col, double v) in soc.Entries) gMat.Add(ineq.Count + row, col, v);
        var h = new double[ineq.Count + soc.Count];
        ineq.Rhs.CopyTo(h, 0);
        soc.Rhs.CopyTo(h, ineq.Count);

        // The thrust cones were emitted after the q-alpha cones, so the dimension list is in row order already.
        var problem = new ConicProblem
        {
            C = cvec,
            P = p,
            A = aMat,
            B = [.. eq.Rhs],
            G = gMat,
            H = h,
            PositiveOrthantDim = ineq.Count,
            SocDims = [.. socDims],
        };

        ConicResult res = ClarabelSolver.Solve(problem, out ClarabelSolver.ClarabelSolveInfo info,
            maxIterations: st.SubproblemMaxIterations, eps: st.SubproblemEps);
        if (!res.IsOptimal || res.X.Length != _nVar)
            return new Solved(false, res.Status.ToString(), [], [], [], [], [], [], [], [], res.Iterations, info.TotalMs);

        double[] x = res.X;
        return new Solved(true, res.Status.ToString(),
            x[..(n * NX)], x[_oU..(_oU + n * NU)], x[_oW..(_oW + c.Coll.Length * NX)],
            x[_oSig..(_oSig + c.S)], x[_oTerm..(_oTerm + 5)], x[_oSq..(_oSq + _nA)], x[_oSqa..(_oSqa + _nA)], x[_oSpro..(_oSpro + _nA)],
            res.Iterations, info.TotalMs);
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
    public readonly double[] F, A, B, W, JwX, JwU, Q, Jq;

    public Linearization(int n)
    {
        const int nx = AscentCase.NX, nu = AscentCase.NU;
        F = new double[n * nx];
        A = new double[n * nx * nx];
        B = new double[n * nx * nu];
        W = new double[n * 3];
        JwX = new double[n * 3 * nx];
        JwU = new double[n * 3 * nu];
        Q = new double[n];
        Jq = new double[n * nx];
    }

    public void Fill(AscentCase c, double[] xbar, double[] ubar)
    {
        const int nx = AscentCase.NX, nu = AscentCase.NU;
        for (int k = 0; k < c.N; k++)
        {
            c.Dyn.Linearize(xbar.AsSpan(k * nx, nx), ubar.AsSpan(k * nu, nu), k,
                F.AsSpan(k * nx, nx), A.AsSpan(k * nx * nx, nx * nx), B.AsSpan(k * nx * nu, nx * nu),
                W.AsSpan(k * 3, 3), JwX.AsSpan(k * 3 * nx, 3 * nx), JwU.AsSpan(k * 3 * nu, 3 * nu),
                out Q[k], Jq.AsSpan(k * nx, nx));
        }
    }
}

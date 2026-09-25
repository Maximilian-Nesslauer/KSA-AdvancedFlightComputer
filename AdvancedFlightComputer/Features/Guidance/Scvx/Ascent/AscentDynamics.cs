using AdvancedFlightComputer.Guidance.Numerics;

namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// The solids at one point of a stage: their mass flow, and their thrust against back pressure.
///
/// At a node the thrust is a spline through the table's row, so the dynamics can differentiate it in altitude; the node's time is fixed (see <see cref="AscentSolidBurn"/>), so nothing else about it moves. Between the nodes - only the seed goes there - it is read straight off the table, linear in time and pressure, and never differentiated.
/// </summary>
internal readonly struct SolidPoint
{
    /// <summary>Canonical mass flow.</summary>
    public readonly double MassFlowC;
    private readonly CubicBSplineNd? _spline;
    private readonly AscentSolidBurn? _table;
    private readonly double _seconds;
    private readonly double _constant;

    private SolidPoint(double massFlowC, CubicBSplineNd? spline, AscentSolidBurn? table, double seconds, double constant)
    {
        MassFlowC = massFlowC;
        _spline = spline;
        _table = table;
        _seconds = seconds;
        _constant = constant;
    }

    public static readonly SolidPoint None = new(0.0, null, null, 0.0, 0.0);

    /// <summary>A node's: a spline through the thrust at each grid pressure, or a constant for a table with one pressure.</summary>
    public static SolidPoint AtNode(double massFlowC, double[] pressureGrid, double[] thrustRow)
    {
        if (pressureGrid.Length < 2)
            return new SolidPoint(massFlowC, null, null, 0.0, thrustRow[0]);
        // Linear edges, as the liquid table: a pad below the mean radius extrapolates along the slope.
        return new SolidPoint(massFlowC, CubicBSplineNd.Fit(pressureGrid, thrustRow, EdgeMode.Linear), null, 0.0, 0.0);
    }

    /// <summary>Anywhere in a stage, off the nodes: the table read directly.</summary>
    public static SolidPoint Between(AscentSolidBurn table, double seconds, double massFlowC)
        => new(massFlowC, null, table, seconds, 0.0);

    /// <summary>Summed thrust at a back pressure, N.</summary>
    public Dual Thrust(Dual pressure)
    {
        if (_spline != null)
        {
            Span<double> p = stackalloc double[1] { pressure.V };
            Span<double> value = stackalloc double[1];
            Span<double> grad = stackalloc double[1];
            _spline.EvaluateWithGradient(p, value, grad);
            return new Dual(value[0], grad[0] * pressure.D);
        }
        if (_table != null)
        {
            int np = _table.Pressures;
            Span<double> row = stackalloc double[np];
            _table.ThrustRow(_seconds, row);
            if (np == 1)
                return new Dual(row[0]);
            double[] grid = _table.PressureGrid;
            double pv = pressure.V;
            int hi = 1;
            while (hi < np - 1 && grid[hi] < pv) hi++;
            int lo = hi - 1;
            double slope = (row[hi] - row[lo]) / (grid[hi] - grid[lo]);
            return new Dual(row[lo] + slope * (pv - grid[lo]), slope * pressure.D);
        }
        return new Dual(_constant);
    }
}

/// <summary>
/// Point-mass ascent dynamics in launch3dof.py's canonical units, and their Jacobians by forward-mode AD. The one piece of hand-written physics in the ascent solver: every slope the subproblem uses comes from differentiating this.
///
///   state    x = [ r(3), v(3), m ]           control  u(3), the throttle vector, |u| &lt;= 1
///   rdot = v
///   vdot = (Tmax(h) u + Ts(h) u/|u|) / m  -  r / |r|^3  +  a_drag
///   mdot = -(mdot_max |u| + mdot_s)
///   a_drag = -(Kd rho(h) Cd(M, alpha) |v_rel| / m) v_rel,    v_rel = v - omega x r
///
/// THE CONTROL IS THROTTLE, NOT THRUST. The script's control is the thrust vector T, bounded by a constant Tmax per stage. Once thrust depends on back pressure (see <see cref="AscentStage"/>) that bound depends on the state, and |T| &lt;= Tmax(h) is no longer a cone. Writing T = Tmax(h) u moves the altitude dependence into the dynamics and leaves |u| &lt;= 1 a cone again. With constant thrust the two are the same problem: the script divides every thrust by its stage's Tmax wherever it bounds, smooths or trusts it, and u is that quotient.
///
/// SOLIDS PUSH ALONG u, WHATEVER ITS LENGTH. The throttle vector's length is the liquid engines' throttle and its direction is the vehicle's attitude; a solid motor ignores the first and follows the second, so its thrust Ts and mass flow mdot_s - read off its table at the node's fixed second of the stage - add on outside the throttle (issue #73). In a stage of solids alone Tmax is zero and only u's direction does anything.
///
/// DRAG DEPENDS ON THE ATTITUDE, which launch3dof.py's does not: alpha is the angle between u and v_rel, and KSA's drag grows with it (see <see cref="KsaAscentAtmosphere"/>). It is atan2(|w|, u_hat . v_rel_hat) with the same lateral vector w the q-alpha limit uses, but with |w| rounded to |w|^2 / sqrt(|w|^2 + e^2), e = <see cref="AscentSettings.AlphaRounding"/>. The drag's |sin alpha| has a corner at alpha = 0, exactly where an ascent flies through the thick air, and a linearisation at the corner promises gains the true drag never pays out: the plan stalls there. The rounding is zero at alpha = 0, so nose-on drag is exact, and reads |w| = e 30 % low and larger angles about e^2 / 2|w| low. Its curvature at the corner, 2/e, is why the trust region's floor sits below the script's; see <see cref="AscentSettings.TrustMin"/>. An atmosphere whose drag ignores the attitude - the script's - never reads alpha, so its arithmetic is unchanged.
///
/// Every regularising epsilon is the script's, and T_EPS2 is carried into throttle units, so a constant-thrust problem matches the script's arithmetic exactly.
/// </summary>
internal sealed class AscentDynamics
{
    public const int NX = 7;
    public const int NU = 3;
    public const int IM = 6;

    private const double REps2 = 1e-8;
    private const double VEps2 = 1e-8;
    private const double TEps2 = 1e-6;

    public readonly double LU, TU, VU, AU, MU, FU;
    public readonly double OmegaC;

    private readonly AscentAtmosphere _atm;
    private readonly AscentStage[] _stages;
    private readonly double[] _mdotC, _kd, _uEps2;
    private readonly double _alphaRounding;
    private int[] _nodeStage = [];
    private SolidPoint[] _nodeSolid = [];

    public AscentDynamics(AscentProblem p)
    {
        LU = p.BodyRadius;
        TU = Math.Sqrt(LU * LU * LU / p.Mu);
        VU = LU / TU;
        AU = LU / (TU * TU);
        MU = p.M0;
        FU = MU * AU;
        OmegaC = p.Omega * TU;

        _atm = p.Atmosphere;
        _stages = p.Stages;
        _alphaRounding = p.Settings.AlphaRounding;
        int s = _stages.Length;
        _mdotC = new double[s];
        _kd = new double[s];
        _uEps2 = new double[s];
        for (int i = 0; i < s; i++)
        {
            AscentStage st = _stages[i];
            _mdotC[i] = st.MassFlow * TU / MU;
            _kd[i] = 0.5 * VU * VU * st.DragArea / FU;
            // A stage of solids alone has no liquid thrust to scale by: its solids' mean stands in.
            double tref = (st.HasLiquid ? st.Thrust : st.Solid!.MeanThrust()) / FU;
            _uEps2[i] = TEps2 / (tref * tref);
        }
    }

    /// <summary>Each node's stage and its solids, from the mesh.</summary>
    public void BindNodes(int[] nodeStage, SolidPoint[] nodeSolid)
    {
        _nodeStage = nodeStage;
        _nodeSolid = nodeSolid;
    }

    public int StageCount => _stages.Length;

    /// <summary>Canonical full-throttle mass flow of a stage's liquid engines.</summary>
    public double MassFlowC(int stage) => _mdotC[stage];

    /// <summary>Canonical reference thrust of a stage's liquid engines.</summary>
    public double ThrustRefC(int stage) => _stages[stage].Thrust / FU;

    /// <summary>The solids of a stage at a stage time, off the nodes.</summary>
    public SolidPoint SolidBetween(int stage, double stageSeconds)
    {
        AscentSolidBurn? table = _stages[stage].Solid;
        return table == null ? SolidPoint.None
            : SolidPoint.Between(table, stageSeconds, table.MassFlowAt(stageSeconds) * TU / MU);
    }

    /// <summary>
    /// Air-relative velocity, radius, air speed and altitude, all canonical except the altitude (metres, for the atmosphere).
    /// </summary>
    private void Air(ReadOnlySpan<Dual> x, out Dual rn, out Dual ax, out Dual ay, out Dual az,
                     out Dual vrn, out Dual hMetres)
    {
        Dual rx = x[0], ry = x[1], rz = x[2];
        rn = Dual.Sqrt(rx * rx + ry * ry + rz * rz + REps2);
        // omega = (0, 0, w), so omega x r = (-w ry, w rx, 0).
        ax = x[3] + OmegaC * ry;
        ay = x[4] - OmegaC * rx;
        az = x[5];
        vrn = Dual.Sqrt(ax * ax + ay * ay + az * az + VEps2);
        hMetres = (rn - 1.0) * LU;
    }

    private void F(ReadOnlySpan<Dual> x, ReadOnlySpan<Dual> u, int stage, in SolidPoint solid, Span<Dual> dx)
    {
        Air(x, out Dual rn, out Dual ax, out Dual ay, out Dual az, out Dual vrn, out Dual h);
        Dual m = x[IM];

        Dual rho = _atm.Density(h);
        Dual mach = vrn * VU / _atm.SpeedOfSound(h);
        Dual pressure = _atm.Pressure(h);
        Dual tmax = _stages[stage].MaxThrust(pressure) / FU;

        Dual umag = Dual.Sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2] + _uEps2[stage]);
        Dual cd = _atm.DragCoefficient(mach, AngleOfAttack(u, umag, ax, ay, az, vrn));
        Dual kdrag = _kd[stage] * rho * cd * vrn / m;
        Dual rn3 = rn * rn * rn;
        // Liquid thrust scales with u; the solids' along u's direction only, so per unit of u they push ts / |u|.
        Dual tm = tmax / m;
        if (solid.MassFlowC > 0.0)
            tm += solid.Thrust(pressure) / FU / (m * umag);

        dx[0] = x[3];
        dx[1] = x[4];
        dx[2] = x[5];
        dx[3] = tm * u[0] - x[0] / rn3 - kdrag * ax;
        dx[4] = tm * u[1] - x[1] / rn3 - kdrag * ay;
        dx[5] = tm * u[2] - x[2] / rn3 - kdrag * az;
        dx[6] = -(_mdotC[stage] * umag + solid.MassFlowC);
    }

    /// <summary>
    /// The angle between the thrust axis and the air-relative velocity, radians: atan2(|w|, u_hat . v_rel_hat), with |w| rounded so the corner at alpha = 0 is smooth.
    /// </summary>
    private Dual AngleOfAttack(ReadOnlySpan<Dual> u, Dual umag, Dual ax, Dual ay, Dual az, Dual vrn, bool rounded = true)
    {
        Dual wx = ax / vrn, wy = ay / vrn, wz = az / vrn;
        Dual dx = u[0] / umag, dy = u[1] / umag, dz = u[2] / umag;
        Dual along = dx * wx + dy * wy + dz * wz;
        Dual lx = dx - along * wx, ly = dy - along * wy, lz = dz - along * wz;
        Dual w2 = lx * lx + ly * ly + lz * lz;
        Dual lateral = rounded
            ? w2 / Dual.Sqrt(w2 + _alphaRounding * _alphaRounding)
            : Dual.Sqrt(w2);
        return Dual.Atan2(lateral, along);
    }

    /// <summary>The true angle of attack at a node, degrees, unrounded, for the report.</summary>
    public double AngleOfAttackDeg(ReadOnlySpan<double> x, ReadOnlySpan<double> u, int stage)
    {
        Span<Dual> dxs = stackalloc Dual[NX];
        Span<Dual> dus = stackalloc Dual[NU];
        for (int i = 0; i < NX; i++) dxs[i] = new Dual(x[i]);
        for (int j = 0; j < NU; j++) dus[j] = new Dual(u[j]);
        Air(dxs, out _, out Dual ax, out Dual ay, out Dual az, out Dual vrn, out _);
        Dual umag = Dual.Sqrt(dus[0] * dus[0] + dus[1] * dus[1] + dus[2] * dus[2] + _uEps2[stage]);
        return AngleOfAttack(dus, umag, ax, ay, az, vrn, rounded: false).V * 180.0 / Math.PI;
    }

    /// <summary>Dynamic pressure, Pa, against the co-rotating air.</summary>
    public Dual Q(ReadOnlySpan<Dual> x)
    {
        Air(x, out _, out _, out _, out _, out Dual vrn, out Dual h);
        Dual vs = vrn * VU;
        return 0.5 * _atm.Density(h) * vs * vs;
    }

    /// <summary>
    /// The lateral thrust direction w = u_hat - (u_hat . v_rel_hat) v_rel_hat, so |w| = sin(alpha). The q-alpha limit is linearised through this vector, not through |w|: at alpha = 0 the gradient of |w| vanishes and a scalar linearisation cannot see the thrust turn.
    /// </summary>
    public void W(ReadOnlySpan<Dual> x, ReadOnlySpan<Dual> u, int stage, Span<Dual> w)
    {
        Air(x, out _, out Dual ax, out Dual ay, out Dual az, out Dual vrn, out _);
        Dual wx = ax / vrn, wy = ay / vrn, wz = az / vrn;
        Dual umag = Dual.Sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2] + _uEps2[stage]);
        Dual dx = u[0] / umag, dy = u[1] / umag, dz = u[2] / umag;
        Dual along = dx * wx + dy * wy + dz * wz;
        w[0] = dx - along * wx;
        w[1] = dy - along * wy;
        w[2] = dz - along * wz;
    }

    /// <summary>
    /// Everything the subproblem linearises at one node, from ten seeded sweeps - one per state and control component. f and w depend on both, q on the state alone.
    /// </summary>
    public void Linearize(ReadOnlySpan<double> x, ReadOnlySpan<double> u, int node,
                          Span<double> f, Span<double> a, Span<double> b,
                          Span<double> w, Span<double> jwx, Span<double> jwu,
                          out double q, Span<double> jq)
    {
        Span<Dual> dxs = stackalloc Dual[NX];
        Span<Dual> dus = stackalloc Dual[NU];
        Span<Dual> fo = stackalloc Dual[NX];
        Span<Dual> wo = stackalloc Dual[3];
        q = 0.0;
        int stage = _nodeStage[node];
        SolidPoint solid = _nodeSolid[node];

        for (int col = 0; col < NX + NU; col++)
        {
            for (int i = 0; i < NX; i++) dxs[i] = new Dual(x[i], col == i ? 1.0 : 0.0);
            for (int j = 0; j < NU; j++) dus[j] = new Dual(u[j], col == NX + j ? 1.0 : 0.0);

            F(dxs, dus, stage, solid, fo);
            W(dxs, dus, stage, wo);

            if (col == 0)
            {
                for (int r = 0; r < NX; r++) f[r] = fo[r].V;
                for (int r = 0; r < 3; r++) w[r] = wo[r].V;
            }

            if (col < NX)
            {
                for (int r = 0; r < NX; r++) a[r * NX + col] = fo[r].D;
                for (int r = 0; r < 3; r++) jwx[r * NX + col] = wo[r].D;
                Dual qd = Q(dxs);
                if (col == 0) q = qd.V;
                jq[col] = qd.D;
            }
            else
            {
                int c = col - NX;
                for (int r = 0; r < NX; r++) b[r * NU + c] = fo[r].D;
                for (int r = 0; r < 3; r++) jwu[r * NU + c] = wo[r].D;
            }
        }
    }

    /// <summary>The dynamics at a node.</summary>
    public void Eval(ReadOnlySpan<double> x, ReadOnlySpan<double> u, int node, Span<double> f)
        => Eval(x, u, _nodeStage[node], _nodeSolid[node], f);

    /// <summary>The dynamics anywhere in a stage, given its solids there.</summary>
    public void Eval(ReadOnlySpan<double> x, ReadOnlySpan<double> u, int stage, in SolidPoint solid, Span<double> f)
    {
        Span<Dual> dxs = stackalloc Dual[NX];
        Span<Dual> dus = stackalloc Dual[NU];
        Span<Dual> fo = stackalloc Dual[NX];
        for (int i = 0; i < NX; i++) dxs[i] = new Dual(x[i]);
        for (int j = 0; j < NU; j++) dus[j] = new Dual(u[j]);
        F(dxs, dus, stage, solid, fo);
        for (int r = 0; r < NX; r++) f[r] = fo[r].V;
    }

    public double QValue(ReadOnlySpan<double> x)
    {
        Span<Dual> dxs = stackalloc Dual[NX];
        for (int i = 0; i < NX; i++) dxs[i] = new Dual(x[i]);
        return Q(dxs).V;
    }

    /// <summary>q times sin(alpha), Pa rad - the true nonlinear value, as the script's qalpha().</summary>
    public double QAlpha(ReadOnlySpan<double> x, ReadOnlySpan<double> u, int stage)
    {
        Span<Dual> dxs = stackalloc Dual[NX];
        Span<Dual> dus = stackalloc Dual[NU];
        Span<Dual> wo = stackalloc Dual[3];
        for (int i = 0; i < NX; i++) dxs[i] = new Dual(x[i]);
        for (int j = 0; j < NU; j++) dus[j] = new Dual(u[j]);
        W(dxs, dus, stage, wo);
        double ww = wo[0].V * wo[0].V + wo[1].V * wo[1].V + wo[2].V * wo[2].V;
        return Q(dxs).V * Math.Sqrt(ww + 1e-9);
    }

    /// <summary>A node's thrust, N, for the report: the liquid engines' at full throttle and at the node's throttle, and the solids'.</summary>
    public void ThrustAt(ReadOnlySpan<double> x, ReadOnlySpan<double> u, int node,
                         out double liquidFull, out double delivered, out double solid)
    {
        double rn = Math.Sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + REps2);
        var pressure = _atm.Pressure(new Dual((rn - 1.0) * LU));
        int stage = _nodeStage[node];
        liquidFull = _stages[stage].MaxThrust(pressure).V;
        SolidPoint sp = _nodeSolid[node];
        solid = sp.MassFlowC > 0.0 ? sp.Thrust(pressure).V : 0.0;
        double um = Math.Sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
        delivered = liquidFull * um + solid;
    }
}

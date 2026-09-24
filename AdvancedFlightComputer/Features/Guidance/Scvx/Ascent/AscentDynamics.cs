using AdvancedFlightComputer.Guidance.Numerics;

namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// Point-mass ascent dynamics in launch3dof.py's canonical units, and their Jacobians by forward-mode AD. The one piece of hand-written physics in the ascent solver: every slope the subproblem uses comes from differentiating this.
///
///   state    x = [ r(3), v(3), m ]           control  u(3), the throttle vector, |u| &lt;= 1
///   rdot = v
///   vdot = Tmax(h) u / m  -  r / |r|^3  +  a_drag
///   mdot = -mdot_max |u|
///   a_drag = -(Kd rho(h) Cd(M) |v_rel| / m) v_rel,    v_rel = v - omega x r
///
/// THE CONTROL IS THROTTLE, NOT THRUST. The script's control is the thrust vector T, bounded by a constant Tmax per stage. Once thrust depends on back pressure (see <see cref="AscentStage"/>) that bound depends on the state, and |T| &lt;= Tmax(h) is no longer a cone. Writing T = Tmax(h) u moves the altitude dependence into the dynamics and leaves |u| &lt;= 1 a cone again. With constant thrust the two are the same problem: the script divides every thrust by its stage's Tmax wherever it bounds, smooths or trusts it, and u is that quotient.
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
        int s = _stages.Length;
        _mdotC = new double[s];
        _kd = new double[s];
        _uEps2 = new double[s];
        for (int i = 0; i < s; i++)
        {
            AscentStage st = _stages[i];
            _mdotC[i] = st.MassFlow * TU / MU;
            _kd[i] = 0.5 * VU * VU * st.DragArea / FU;
            double tref = st.Thrust / FU;
            _uEps2[i] = TEps2 / (tref * tref);
        }
    }

    public int StageCount => _stages.Length;

    /// <summary>Canonical full-throttle mass flow of a stage.</summary>
    public double MassFlowC(int stage) => _mdotC[stage];

    /// <summary>Canonical reference thrust of a stage.</summary>
    public double ThrustRefC(int stage) => _stages[stage].Thrust / FU;

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

    public void F(ReadOnlySpan<Dual> x, ReadOnlySpan<Dual> u, int stage, Span<Dual> dx)
    {
        Air(x, out Dual rn, out Dual ax, out Dual ay, out Dual az, out Dual vrn, out Dual h);
        Dual m = x[IM];

        Dual rho = _atm.Density(h);
        Dual mach = vrn * VU / _atm.SpeedOfSound(h);
        Dual cd = _atm.DragCoefficient(mach);
        Dual tmax = _stages[stage].MaxThrust(_atm.Pressure(h)) / FU;

        Dual umag = Dual.Sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2] + _uEps2[stage]);
        Dual kdrag = _kd[stage] * rho * cd * vrn / m;
        Dual rn3 = rn * rn * rn;
        Dual tm = tmax / m;

        dx[0] = x[3];
        dx[1] = x[4];
        dx[2] = x[5];
        dx[3] = tm * u[0] - x[0] / rn3 - kdrag * ax;
        dx[4] = tm * u[1] - x[1] / rn3 - kdrag * ay;
        dx[5] = tm * u[2] - x[2] / rn3 - kdrag * az;
        dx[6] = -(_mdotC[stage] * umag);
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
    public void Linearize(ReadOnlySpan<double> x, ReadOnlySpan<double> u, int stage,
                          Span<double> f, Span<double> a, Span<double> b,
                          Span<double> w, Span<double> jwx, Span<double> jwu,
                          out double q, Span<double> jq)
    {
        Span<Dual> dxs = stackalloc Dual[NX];
        Span<Dual> dus = stackalloc Dual[NU];
        Span<Dual> fo = stackalloc Dual[NX];
        Span<Dual> wo = stackalloc Dual[3];
        q = 0.0;

        for (int col = 0; col < NX + NU; col++)
        {
            for (int i = 0; i < NX; i++) dxs[i] = new Dual(x[i], col == i ? 1.0 : 0.0);
            for (int j = 0; j < NU; j++) dus[j] = new Dual(u[j], col == NX + j ? 1.0 : 0.0);

            F(dxs, dus, stage, fo);
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

    public void Eval(ReadOnlySpan<double> x, ReadOnlySpan<double> u, int stage, Span<double> f)
    {
        Span<Dual> dxs = stackalloc Dual[NX];
        Span<Dual> dus = stackalloc Dual[NU];
        Span<Dual> fo = stackalloc Dual[NX];
        for (int i = 0; i < NX; i++) dxs[i] = new Dual(x[i]);
        for (int j = 0; j < NU; j++) dus[j] = new Dual(u[j]);
        F(dxs, dus, stage, fo);
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

    /// <summary>Full-throttle thrust of a stage at a canonical state, N - for the report.</summary>
    public double MaxThrustAt(ReadOnlySpan<double> x, int stage)
    {
        double rn = Math.Sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + REps2);
        double h = (rn - 1.0) * LU;
        return _stages[stage].MaxThrust(_atm.Pressure(new Dual(h))).V;
    }
}

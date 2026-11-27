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
/// THE FULL AERO FORCE, LIFT AND SIDE FORCE INCLUDED (#216), where the atmosphere carries a nose-first body-axis table (<see cref="AscentAtmosphere.BodyForce"/>):
///   a_aero = (Kd rho(h) |v_rel|^2 / m) (C_x x_b + C_y y_b + C_z z_b)
/// in place of a_drag. The body axes are the ones the flight holds (GuidanceWindow.AscentRollRef): x_b = u_hat, the thrust axis; y_b the plane's -n square to it, turned about x_b by the held roll (<see cref="AscentProblem.RollAngleRad"/>); z_b = x_b x y_b. The coefficients are read at the air direction in those axes, alpha = atan2(h_z, h_x), beta = atan2(h_y, h_x), h = v_rel_hat. A tilt off the airflow then pushes sideways as well as back: on KSA 5901 the lift at a few degrees is close to the drag, which the drag-only model left out, so the flown path left the plan wherever it flew off the airflow, and the plan could not use the lift to turn.
///  - THE AXIAL COEFFICIENT IS READ AT THE ROUNDED TILT, for the drag's reason above: the tilt's size |w| is rounded as there, its direction kept. The lateral coefficients are read at the true one: the normal force grows from zero smoothly (sin alpha, sin alpha |sin alpha|), and rounding it would hide its slope at alpha = 0 from the linearisation, where a plan starts every turn.
///  - Past the table (NoseAngleMaxDeg, 20 deg) each angle is softly bounded, x / (1 + (x/A)^8)^(1/8), A = <see cref="SoftAngleRad"/>: identity to 0.2 % inside the table (2e-6 at 10 deg), bounded where an ascent coasts at a large tilt in air too thin to matter, so the spline's linear extension never runs away. The Mach number is held inside the table.
///  - Without a body table (an older problem file, an airless body) the drag-only law stands.
///
/// Every regularising epsilon is the script's, and T_EPS2 is carried into throttle units, so a constant-thrust problem matches the script's arithmetic exactly.
/// </summary>
internal sealed class AscentDynamics
{
    /// <summary>Where the soft bound on the body table's alpha and beta levels off, radians (see the type summary): 35 deg, so the table's 20 deg reads 19.97.</summary>
    internal const double SoftAngleRad = 35.0 * Math.PI / 180.0;

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
    private readonly AdvancedFlightComputer.Guidance.Numerics.Flight.BodyAeroTable? _bodyForce;
    private readonly double _nx, _ny, _nz, _rollCos, _rollSin;
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
        // Unturned reference plane normal; BodyForce rotates it about +z by phi.
        _bodyForce = _atm.BodyForce is { NoseFirst: true } table ? table : null;
        double nn = Math.Sqrt(p.PlaneNormal[0] * p.PlaneNormal[0] + p.PlaneNormal[1] * p.PlaneNormal[1] + p.PlaneNormal[2] * p.PlaneNormal[2]);
        _nx = p.PlaneNormal[0] / nn;
        _ny = p.PlaneNormal[1] / nn;
        _nz = p.PlaneNormal[2] / nn;
        _rollCos = Math.Cos(p.RollAngleRad);
        _rollSin = Math.Sin(p.RollAngleRad);
        int s = _stages.Length;
        _mdotC = new double[s];
        _kd = new double[s];
        _uEps2 = new double[s];
        for (int i = 0; i < s; i++)
        {
            AscentStage st = _stages[i];
            _mdotC[i] = st.MassFlow * TU / MU;
            _kd[i] = 0.5 * VU * VU * st.DragArea / FU;
            if (st.IsCoast)
            {
                _uEps2[i] = TEps2;
                continue;
            }
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

    private void F(ReadOnlySpan<Dual> x, ReadOnlySpan<Dual> u, int stage, in SolidPoint solid, Dual phi, Span<Dual> dx)
    {
        Air(x, out Dual rn, out Dual ax, out Dual ay, out Dual az, out Dual vrn, out Dual h);
        Dual m = x[IM];

        Dual rho = _atm.Density(h);
        Dual mach = vrn * VU / _atm.SpeedOfSound(h);
        Dual rn3 = rn * rn * rn;
        if (_stages[stage].IsCoast)
        {
            Dual cAeroX, cAeroY, cAeroZ;
            if (_bodyForce != null)
            {
                Span<Dual> pro = stackalloc Dual[3] { ax, ay, az };
                BodyForce(pro, vrn, ax, ay, az, vrn, mach, _kd[stage] * rho * vrn * vrn / m, phi, out cAeroX, out cAeroY, out cAeroZ);
            }
            else
            {
                Dual cd = _atm.DragCoefficient(mach, new Dual(0.0));
                Dual kdrag = _kd[stage] * rho * cd * vrn / m;
                cAeroX = -kdrag * ax;
                cAeroY = -kdrag * ay;
                cAeroZ = -kdrag * az;
            }
            dx[0] = x[3];
            dx[1] = x[4];
            dx[2] = x[5];
            dx[3] = -x[0] / rn3 + cAeroX;
            dx[4] = -x[1] / rn3 + cAeroY;
            dx[5] = -x[2] / rn3 + cAeroZ;
            dx[6] = new Dual(0.0);
            return;
        }

        Dual pressure = _atm.Pressure(h);
        Dual tmax = _stages[stage].MaxThrust(pressure) / FU;

        Dual umag = Dual.Sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2] + _uEps2[stage]);
        Dual aeroX, aeroY, aeroZ;
        if (_bodyForce != null)
            BodyForce(u, umag, ax, ay, az, vrn, mach, _kd[stage] * rho * vrn * vrn / m, phi, out aeroX, out aeroY, out aeroZ);
        else
        {
            Dual cd = _atm.DragCoefficient(mach, AngleOfAttack(u, umag, ax, ay, az, vrn));
            Dual kdrag = _kd[stage] * rho * cd * vrn / m;
            aeroX = -kdrag * ax;
            aeroY = -kdrag * ay;
            aeroZ = -kdrag * az;
        }
        // Liquid thrust scales with u; the solids' along u's direction only, so per unit of u they push ts / |u|.
        Dual tm = tmax / m;
        if (solid.MassFlowC > 0.0)
            tm += solid.Thrust(pressure) / FU / (m * umag);

        dx[0] = x[3];
        dx[1] = x[4];
        dx[2] = x[5];
        dx[3] = tm * u[0] - x[0] / rn3 + aeroX;
        dx[4] = tm * u[1] - x[1] / rn3 + aeroY;
        dx[5] = tm * u[2] - x[2] / rn3 + aeroZ;
        dx[6] = -(_mdotC[stage] * umag + solid.MassFlowC);
    }

    /// <summary>
    /// The aero acceleration from the nose-first body-axis table (see the type summary): <paramref name="scale"/> = Kd rho |v_rel|^2 / m, canonical, times the table's coefficients turned out of the body axes the flight holds, using the plane normal rotated by <paramref name="phi"/> about the spin axis.
    /// </summary>
    private void BodyForce(ReadOnlySpan<Dual> u, Dual umag, Dual ax, Dual ay, Dual az, Dual vrn, Dual mach, Dual scale,
                           Dual phi, out Dual fx, out Dual fy, out Dual fz)
    {
        var table = _bodyForce!;
        // Body x: the thrust axis.
        Dual xx = u[0] / umag, xy = u[1] / umag, xz = u[2] / umag;
        // Plane normal turned about +z by phi so lateral/normal forces track PlaneTurn.
        Dual cPhi = Dual.Cos(phi), sPhi = Dual.Sin(phi);
        Dual nx = cPhi * _nx - sPhi * _ny;
        Dual ny = sPhi * _nx + cPhi * _ny;
        Dual nz = new(_nz);
        // Body y: -n(phi) square to x, turned about x by the held roll; z = x x y.
        Dual along = -(xx * nx + xy * ny + xz * nz);
        Dual y0x = -nx - along * xx, y0y = -ny - along * xy, y0z = -nz - along * xz;
        Dual y0n = Dual.Sqrt(y0x * y0x + y0y * y0y + y0z * y0z + 1e-12);
        y0x /= y0n;
        y0y /= y0n;
        y0z /= y0n;
        Dual z0x = xy * y0z - xz * y0y, z0y = xz * y0x - xx * y0z, z0z = xx * y0y - xy * y0x;
        Dual yx = _rollCos * y0x + _rollSin * z0x, yy = _rollCos * y0y + _rollSin * z0y, yz = _rollCos * y0z + _rollSin * z0z;
        Dual zx = _rollCos * z0x - _rollSin * y0x, zy = _rollCos * z0y - _rollSin * y0y, zz = _rollCos * z0z - _rollSin * y0z;

        // The air direction in the body: the craft's velocity through the air, unit.
        Dual hx = ax / vrn, hy = ay / vrn, hz = az / vrn;
        Dual bx = hx * xx + hy * xy + hz * xz;
        Dual by = hx * yx + hy * yy + hz * yz;
        Dual bz = hx * zx + hy * zy + hz * zz;

        Dual m = mach;
        if (m.V < table.MachMin) m = new Dual(table.MachMin);
        else if (m.V > table.MachMax) m = new Dual(table.MachMax);

        // Lateral coefficients at the true angles.
        table.Coefficients(m, Soft(Dual.Atan2(bz, bx)), Soft(Dual.Atan2(by, bx)), out _, out Dual cy, out Dual cz);
        // The axial one at the rounded tilt: |w| -> |w|^2 / sqrt(|w|^2 + e^2), its direction kept.
        Dual w2 = by * by + bz * bz;
        Dual k = Dual.Sqrt(w2 + 1e-18) / Dual.Sqrt(w2 + _alphaRounding * _alphaRounding);
        table.Coefficients(m, Soft(Dual.Atan2(k * bz, bx)), Soft(Dual.Atan2(k * by, bx)), out Dual cx, out _, out _);

        fx = scale * (cx * xx + cy * yx + cz * zx);
        fy = scale * (cx * xy + cy * yy + cz * zy);
        fz = scale * (cx * xz + cy * yz + cz * zz);
    }

    /// <summary>The soft bound on a table angle: x / (1 + (x/A)^8)^(1/8), A = <see cref="SoftAngleRad"/>.</summary>
    private static Dual Soft(Dual angle)
    {
        Dual r = angle / SoftAngleRad;
        Dual r2 = r * r;
        Dual r4 = r2 * r2;
        return angle / Dual.Sqrt(Dual.Sqrt(Dual.Sqrt(1.0 + r4 * r4)));
    }

    /// <summary>
    /// The aero acceleration at a state and liquid-only throttle, canonical, whichever law the dynamics fly: for the offline check against the table.
    /// </summary>
    public void AeroAt(ReadOnlySpan<double> x, ReadOnlySpan<double> u, int stage, Span<double> accel, double phi = 0.0)
    {
        Span<Dual> dxs = stackalloc Dual[NX];
        Span<Dual> dus = stackalloc Dual[NU];
        Span<Dual> fo = stackalloc Dual[NX];
        for (int i = 0; i < NX; i++) dxs[i] = new Dual(x[i]);
        for (int j = 0; j < NU; j++) dus[j] = new Dual(u[j]);
        F(dxs, dus, stage, SolidPoint.None, new Dual(phi), fo);
        Air(dxs, out Dual rn, out _, out _, out _, out _, out _);
        // Less gravity and the liquid thrust: what is left is the air's.
        Dual tmax = _stages[stage].MaxThrust(_atm.Pressure((rn - 1.0) * LU)) / FU / dxs[IM];
        Dual rn3 = rn * rn * rn;
        for (int r = 0; r < 3; r++)
            accel[r] = fo[3 + r].V - tmax.V * u[r] + x[r] / rn3.V;
    }

    /// <summary>Whether the dynamics fly the full body-axis force (the type summary), not drag alone.</summary>
    public bool HasBodyForce => _bodyForce != null;

    /// <summary>
    /// The aero acceleration the dynamics apply, SI (m/s^2), at a position and velocity (inertial, m and m/s), mass (kg) and throttle vector in a stage: what a check compares with the table it was built from.
    /// </summary>
    public static double[] AeroAcceleration(AscentProblem p, int stage, double[] r, double[] v, double mass, double[] u, double phi = 0.0)
    {
        var d = new AscentDynamics(p);
        Span<double> x = stackalloc double[NX];
        for (int i = 0; i < 3; i++)
        {
            x[i] = r[i] / d.LU;
            x[3 + i] = v[i] / d.VU;
        }
        x[IM] = mass / d.MU;
        Span<double> a = stackalloc double[3];
        d.AeroAt(x, u, stage, a, phi);
        return [a[0] * d.AU, a[1] * d.AU, a[2] * d.AU];
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
        if (_stages[stage].IsCoast)
            return 0.0;
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
        if (_stages[stage].IsCoast)
        {
            w[0] = w[1] = w[2] = new Dual(0.0);
            return;
        }
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
    /// Everything the subproblem linearises at one node, from ten seeded sweeps (plus one plane-turn sweep when body-force aerodynamics are active and <paramref name="dFphi"/> is provided).
    /// </summary>
    public void Linearize(ReadOnlySpan<double> x, ReadOnlySpan<double> u, int node,
                          Span<double> f, Span<double> a, Span<double> b,
                          Span<double> w, Span<double> jwx, Span<double> jwu,
                          out double q, Span<double> jq,
                          double phi = 0.0, Span<double> dFphi = default)
    {
        Span<Dual> dxs = stackalloc Dual[NX];
        Span<Dual> dus = stackalloc Dual[NU];
        Span<Dual> fo = stackalloc Dual[NX];
        Span<Dual> wo = stackalloc Dual[3];
        q = 0.0;
        int stage = _nodeStage[node];
        SolidPoint solid = _nodeSolid[node];
        Dual phiConst = new(phi);

        for (int col = 0; col < NX + NU; col++)
        {
            for (int i = 0; i < NX; i++) dxs[i] = new Dual(x[i], col == i ? 1.0 : 0.0);
            for (int j = 0; j < NU; j++) dus[j] = new Dual(u[j], col == NX + j ? 1.0 : 0.0);

            F(dxs, dus, stage, solid, phiConst, fo);
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

        if (!dFphi.IsEmpty)
        {
            if (_bodyForce != null)
            {
                for (int i = 0; i < NX; i++) dxs[i] = new Dual(x[i]);
                for (int j = 0; j < NU; j++) dus[j] = new Dual(u[j]);
                F(dxs, dus, stage, solid, Dual.Seed(phi), fo);
                for (int r = 0; r < NX; r++) dFphi[r] = fo[r].D;
            }
            else
            {
                dFphi.Clear();
            }
        }
    }

    /// <summary>The dynamics at a node.</summary>
    public void Eval(ReadOnlySpan<double> x, ReadOnlySpan<double> u, int node, Span<double> f, double phi = 0.0)
        => Eval(x, u, _nodeStage[node], _nodeSolid[node], f, phi);

    /// <summary>The dynamics anywhere in a stage, given its solids there.</summary>
    public void Eval(ReadOnlySpan<double> x, ReadOnlySpan<double> u, int stage, in SolidPoint solid, Span<double> f, double phi = 0.0)
    {
        Span<Dual> dxs = stackalloc Dual[NX];
        Span<Dual> dus = stackalloc Dual[NU];
        Span<Dual> fo = stackalloc Dual[NX];
        for (int i = 0; i < NX; i++) dxs[i] = new Dual(x[i]);
        for (int j = 0; j < NU; j++) dus[j] = new Dual(u[j]);
        F(dxs, dus, stage, solid, new Dual(phi), fo);
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
        if (_stages[stage].IsCoast)
            return 0.0;
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
        int stage = _nodeStage[node];
        if (_stages[stage].IsCoast)
        {
            liquidFull = delivered = solid = 0.0;
            return;
        }
        Dual pressure = PressureAt(x);
        liquidFull = _stages[stage].MaxThrust(pressure).V;
        SolidPoint sp = _nodeSolid[node];
        solid = sp.MassFlowC > 0.0 ? sp.Thrust(pressure).V : 0.0;
        double um = Math.Sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
        delivered = liquidFull * um + solid;
    }

    /// <summary>Canonical full-throttle thrust of a stage's liquid engines at a state's altitude.</summary>
    public double LiquidThrustC(int stage, ReadOnlySpan<double> x) => _stages[stage].MaxThrust(PressureAt(x)).V / FU;

    private Dual PressureAt(ReadOnlySpan<double> x)
    {
        double rn = Math.Sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + REps2);
        return _atm.Pressure(new Dual((rn - 1.0) * LU));
    }
}

/// <summary>The ascent dynamics' aero law, for checks outside the solver (AscentDynamics is internal to it).</summary>
public static class AscentAeroLaw
{
    /// <summary>AscentDynamics.AeroAcceleration: the aero acceleration the plan flies, m/s^2.</summary>
    public static double[] Acceleration(AscentProblem p, int stage, double[] r, double[] v, double mass, double[] u, double phi = 0.0)
        => AscentDynamics.AeroAcceleration(p, stage, r, v, mass, u, phi);
}

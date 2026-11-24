using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

namespace AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// Six-degree-of-freedom rigid-body booster dynamics, and their Jacobians by
/// forward-mode AD. Port of the model in Scvx/6dof.py - the ONE piece of
/// hand-written physics; every slope below comes from differentiating it.
///
///   state    X = [ r(3)  v(3)  q(4)  w(3)  m(1) ]                 (14)
///   control  u = [ tdx, tdy, T, tau_roll, rcs(3) ]                (7)
///
///   rdot = v
///   vdot = g + (R(q) (T_body + K tau_rcs) + F_aero) / m      T_body = [tdx, tdy, T]
///   qdot = 1/2 q (x) [0, w]              Hamilton, scalar-first
///   wdot = I^-1 ( tau + tau_rcs + tau_aero - w x (I w) )      Euler's equation
///   mdot = -(mdot_0 + alpha * T)
///
/// RCS (see <see cref="RcsModel"/>): a signed torque about each body axis, between what the thrusters give one way and the other, and the side force that comes with it, K tau_rcs: on a booster whose thrusters sit at the top every pitch or yaw torque pushes it sideways. Both from the allocator's own measurement of the layout (RcsTorqueAllocator.Characterise).
///  ONE SIGNED CONTROL PER AXIS, NOT A PAIR. The first version split each axis into a + and a - control so that the propellant, |tau| times a flow, could be linear. The two had exactly opposite columns, and SCS took 5-20 times the iterations on the warm re-solves, hitting its cap (2026-11-19: 2,668 a solve at 5 nodes against 347 without RCS; freeing only the + halves brought it back to ~500). The RCS propellant (16-50 kg against ~480 of the main engine) is left out of the mass instead; Scvx6DofConfig.RcsWeight prices the torque.
///
/// Small-angle gimbal: the engine sits at r_T = [0,0,-L] below the CoM, so
/// tau_gimbal = r_T x T_body = [L*tdy, -L*tdx, 0], linear in the controls.
/// The axial component T carries the throttle box and the mass flow; the
/// lateral components are independent linear controls.
///
/// This assembly deliberately references nothing from the game. The solver
/// must never touch KSA state, and keeping the dependency out at project level
/// makes that a compile error rather than a code-review convention.
/// </summary>
public static class Dynamics6Dof
{
    public const int NX = 14;   // state dimension
    /// <summary>The controls every craft has (engine, roll torque, RCS). The fins follow them, one per control surface the craft carries, so a problem's control count is NUBase + its fins (Params.NU, Scvx6DofConfig.NU).</summary>
    public const int NUBase = 7;

    // State layout
    public const int IR = 0;    // position      r(3)
    public const int IV = 3;    // velocity      v(3)
    public const int IQ = 6;    // quaternion    q(4), scalar-first
    public const int IW = 10;   // body rates    w(3)
    public const int IM = 13;   // mass          m(1)

    // Control layout
    public const int ITDX = 0;  // lateral (gimbal) thrust, body x
    public const int ITDY = 1;  // lateral (gimbal) thrust, body y
    public const int IT = 2;    // axial thrust
    public const int ITAU = 3;  // direct body-z (roll) torque
    public const int IRCS = 4;  // RCS torque, one signed control per body axis: x, y, z
    public const int NRCS = 3;
    public const int IFIN = NUBase;  // control-surface deflections, degrees, one per fin (BodyAero6Dof.Fins), as many as the craft has

    /// <summary>Vehicle and environment constants. Defaults mirror 6dof.py.</summary>
    public sealed class Params
    {
        public double Gx = 0.0, Gy = 0.0, Gz = -9.81;   // gravity, inertial
        public double G0 = 9.81;
        public double Isp = 330.0;
        public double LArm = 25.0;                       // CoM -> gimbal pivot offset
        public double Ixx = 1.0e8, Iyy = 1.0e8, Izz = 2.5e6;

        /// <summary>
        /// The engines' own moment about the centre of mass per newton of axial thrust, with the gimbals at rest, model body axes, N m per N: an off-centre engine choice, or a centre of mass off the thrust line. It joins Euler's equation as T times this, so the throttle and the moment are coupled, and the plan holds the gimbal deflection that cancels it from its first node. Zero for a balanced cluster, which is what the model had before.
        /// </summary>
        public double ThrustMomentX, ThrustMomentY, ThrustMomentZ;

        /// <summary>
        /// THE ROLL A SIDEWAYS DEFLECTION BRINGS, for thrust that acts off the axis. The thrust point sits at (e_x, e_y, -L) from the centre of mass, and the moment above is e x z per newton, so e = (-m_y, m_x); a lateral thrust (tdx, tdy) acting there turns the craft by r x f = (L tdy, -L tdx, e_x tdy - e_y tdx). The roll term is what an off-centre engine makes as it steers: zero for its trim deflection (which is parallel to e), the offset over the arm of the lateral torque across it. Without it the plan took a sideways deflection as rollless, the allocator had to hold the roll at zero with whatever it had (a single engine's verniers), and the plan's gimbal cone was sized by that (Ksa6DofSetup.LateralTorqueLimit). Zero for a balanced cluster.
        /// </summary>
        public double GimbalRollPerTdx => -ThrustMomentX;
        public double GimbalRollPerTdy => -ThrustMomentY;

        /// <summary>Mass flow per newton of axial thrust.</summary>
        public double Alpha => 1.0 / (Isp * G0);

        /// <summary>
        /// The aerodynamics, or null for none: drag and lift on the body +Z axis from the 3-DOF model's sweep-measured tables (KsaPointMassModel.AeroForce), read at the model's height plus <see cref="AeroAltitudeOffset"/>. The velocity is the airspeed, the frame turning with the body as the atmosphere does. With <see cref="BodyAero"/> it carries only the share of the force the body-axis tables fade out of.
        /// </summary>
        public KsaPointMassModel? Aero;

        /// <summary>
        /// The body-axis force AND moment for the plan's full attitude (BodyAero6Dof), or null for none: where set, the force is its share plus <see cref="Aero"/>'s for the rest, and its moment joins the gimbal's in Euler's equation.
        /// </summary>
        public BodyAero6Dof? BodyAero;

        /// <summary>Altitude of the model's origin - the site - above the body's mean radius, m.</summary>
        public double AeroAltitudeOffset;

        /// <summary>
        /// Mass flow that does not buy thrust, kg/s: the engine's flow is linear in its throttle, but under back pressure its thrust is not, so the flow over the burn's thrust range is a line that does not pass through zero. 0 is the plain rocket equation, which is right in vacuum.
        /// </summary>
        public double MassFlowOffset = 0.0;

        /// <summary>The RCS torque the plan may use, or null for none, which leaves its three controls without effect (the subproblem pins them at zero).</summary>
        public RcsModel? Rcs;

        /// <summary>
        /// The engine is off: its lateral and axial thrust have no effect and no propellant flows (<see cref="MassFlowOffset"/> included), while the roll torque, the RCS and the air still act. The coast before ignition (Scvx6DofConfig.CoastIntervals) is evaluated with these.
        /// </summary>
        public bool EngineOff;

        /// <summary>The craft's control surfaces (BodyAero6Dof.Fins), one fin control each.</summary>
        public int FinCount => BodyAero?.Fins.Length ?? 0;

        /// <summary>The control count: the base controls and one per fin.</summary>
        public int NU => NUBase + FinCount;

        /// <summary>These parameters with the engine off (<see cref="EngineOff"/>), sharing every table.</summary>
        public Params WithEngineOff()
        {
            var p = (Params)MemberwiseClone();
            p.EngineOff = true;
            return p;
        }
    }

    /// <summary>
    /// What the craft's RCS can do, in MODEL body axes, per axis a (0, 1, 2 = x, y, z): the largest torque each way, and the force that comes with each newton metre, K (N per N m, body axes, three per axis). The force is taken as odd in the torque - the average of what the two ways bring - which is exact for a symmetric layout. The allocator flies the patterns these were measured from.
    /// </summary>
    public sealed class RcsModel
    {
        /// <summary>Largest positive torque per axis, N m.</summary>
        public double[] MaxPosNm { get; init; } = new double[NRCS];

        /// <summary>Largest negative torque per axis, N m, as a magnitude.</summary>
        public double[] MaxNegNm { get; init; } = new double[NRCS];

        /// <summary>Force per newton metre of torque, body axes, three per axis.</summary>
        public double[] ForcePerNm { get; init; } = new double[3 * NRCS];

        /// <summary>Propellant per newton metre, kg/s per N m, per axis (the two ways' average): for the log, not the dynamics.</summary>
        public double[] FlowPerNm { get; init; } = new double[NRCS];

        public bool Any
        {
            get
            {
                for (int a = 0; a < NRCS; a++)
                    if (MaxPosNm[a] > 0.0 || MaxNegNm[a] > 0.0)
                        return true;
                return false;
            }
        }
    }

    /// <summary>
    /// Continuous dynamics: xdot = f(x, u). Generic over Dual so the same code
    /// yields both the value and any Jacobian column depending on how the inputs
    /// are seeded.
    /// </summary>
    public static void F(ReadOnlySpan<Dual> x, ReadOnlySpan<Dual> u, Params p, Span<Dual> xdot)
    {
        Dual vx = x[IV + 0], vy = x[IV + 1], vz = x[IV + 2];
        Dual qw = x[IQ + 0], qx = x[IQ + 1], qy = x[IQ + 2], qz = x[IQ + 3];
        Dual wx = x[IW + 0], wy = x[IW + 1], wz = x[IW + 2];
        Dual m = x[IM];

        Dual tdx = u[ITDX], tdy = u[ITDY], T = u[IT], tauRoll = u[ITAU];
        if (p.EngineOff)
            tdx = tdy = T = default;

        // rdot = v
        xdot[IR + 0] = vx;
        xdot[IR + 1] = vy;
        xdot[IR + 2] = vz;

        // Body -> inertial rotation applied to T_body = [tdx, tdy, T].
        // Rows of R(q), expanded inline to avoid materialising the matrix.
        Dual r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
        Dual r01 = 2.0 * (qx * qy - qw * qz);
        Dual r02 = 2.0 * (qx * qz + qw * qy);
        Dual r10 = 2.0 * (qx * qy + qw * qz);
        Dual r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
        Dual r12 = 2.0 * (qy * qz - qw * qx);
        Dual r20 = 2.0 * (qx * qz - qw * qy);
        Dual r21 = 2.0 * (qy * qz + qw * qx);
        Dual r22 = 1.0 - 2.0 * (qx * qx + qy * qy);

        // The RCS force joins the engine's in body axes, so the one rotation takes both to the site frame.
        Dual bx = tdx, by = tdy, bz = T;
        Dual rcsTx = default, rcsTy = default, rcsTz = default;
        if (p.Rcs != null)
        {
            rcsTx = u[IRCS];
            rcsTy = u[IRCS + 1];
            rcsTz = u[IRCS + 2];
            for (int a = 0; a < NRCS; a++)
            {
                Dual ua = u[IRCS + a];
                bx += p.Rcs.ForcePerNm[3 * a] * ua;
                by += p.Rcs.ForcePerNm[3 * a + 1] * ua;
                bz += p.Rcs.ForcePerNm[3 * a + 2] * ua;
            }
        }
        Dual fx = r00 * bx + r01 * by + r02 * bz;
        Dual fy = r10 * bx + r11 * by + r12 * bz;
        Dual fz = r20 * bx + r21 * by + r22 * bz;

        // vdot = g + (R(q) T_body + F_aero) / m; the air's force and moment include the control surfaces' at their deflections.
        AeroWrench(x, p, vx, vy, vz, r00, r01, r02, r10, r11, r12, r20, r21, r22,
                   out Dual ax, out Dual ay, out Dual az, out Dual aeroTx, out Dual aeroTy, out Dual aeroTz,
                   u.Length > IFIN ? u.Slice(IFIN, Math.Min(u.Length - IFIN, p.FinCount)) : default);
        fx += ax;
        fy += ay;
        fz += az;

        xdot[IV + 0] = p.Gx + fx / m;
        xdot[IV + 1] = p.Gy + fy / m;
        xdot[IV + 2] = p.Gz + fz / m;

        // qdot = 1/2 * q (x) [0, w]   (Hamilton product, scalar-first)
        xdot[IQ + 0] = 0.5 * (-qx * wx - qy * wy - qz * wz);
        xdot[IQ + 1] = 0.5 * (qw * wx + qy * wz - qz * wy);
        xdot[IQ + 2] = 0.5 * (qw * wy - qx * wz + qz * wx);
        xdot[IQ + 3] = 0.5 * (qw * wz + qx * wy - qy * wx);

        // wdot = I^-1 ( tau - w x (I w) ),  tau = [L*tdy, -L*tdx, tau_roll + e_x tdy - e_y tdx] + T * m_thrust, plus the air's
        Dual taux = p.LArm * tdy + p.ThrustMomentX * T + aeroTx + rcsTx;
        Dual tauy = -(p.LArm * tdx) + p.ThrustMomentY * T + aeroTy + rcsTy;
        tauRoll += p.ThrustMomentZ * T + p.GimbalRollPerTdx * tdx + p.GimbalRollPerTdy * tdy + aeroTz + rcsTz;
        Dual iwx = p.Ixx * wx, iwy = p.Iyy * wy, iwz = p.Izz * wz;
        Dual gyrox = wy * iwz - wz * iwy;
        Dual gyroy = wz * iwx - wx * iwz;
        Dual gyroz = wx * iwy - wy * iwx;
        xdot[IW + 0] = (taux - gyrox) / p.Ixx;
        xdot[IW + 1] = (tauy - gyroy) / p.Iyy;
        xdot[IW + 2] = (tauRoll - gyroz) / p.Izz;

        // mdot = -(mdot_0 + alpha * T)
        xdot[IM] = p.EngineOff ? default : -(p.MassFlowOffset + p.Alpha * T);
    }

    /// <summary>
    /// The aerodynamic force, site frame, N, and moment, model body axes, N m: the body-axis tables' share (BodyAero) and the whole-sphere force on the thrust axis for the rest (Aero, at R's third column); zero with neither.
    /// </summary>
    private static void AeroWrench(ReadOnlySpan<Dual> x, Params p, Dual vx, Dual vy, Dual vz,
                                   Dual r00, Dual r01, Dual r02, Dual r10, Dual r11, Dual r12, Dual r20, Dual r21, Dual r22,
                                   out Dual fx, out Dual fy, out Dual fz, out Dual tx, out Dual ty, out Dual tz,
                                   ReadOnlySpan<Dual> finDeg = default)
    {
        fx = fy = fz = tx = ty = tz = default;
        Dual altitude = p.AeroAltitudeOffset + x[IR + 2];
        Dual weight = default;
        if (p.BodyAero != null)
            p.BodyAero.Wrench(altitude, vx, vy, vz, r00, r01, r02, r10, r11, r12, r20, r21, r22,
                              out fx, out fy, out fz, out tx, out ty, out tz, out weight, finDeg);
        if (p.Aero == null || weight.V >= 1.0)
            return;
        p.Aero.AeroForce(altitude, vx, vy, vz, r02, r12, r22, out Dual ax, out Dual ay, out Dual az);
        Dual rest = 1.0 - weight;
        fx += rest * ax;
        fy += rest * ay;
        fz += rest * az;
    }

    /// <summary>The model's aerodynamic force at a state, site frame, N - what the bias estimator takes off the measured acceleration. Zero with no aero.</summary>
    public static void AeroForce(ReadOnlySpan<double> x, Params p, out double fx, out double fy, out double fz)
    {
        Span<Dual> dx = stackalloc Dual[NX];
        for (int i = 0; i < NX; i++) dx[i] = new Dual(x[i]);
        Dual qw = dx[IQ + 0], qx = dx[IQ + 1], qy = dx[IQ + 2], qz = dx[IQ + 3];
        Dual n = Dual.Sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (!(n.V > 0.0))
        {
            fx = fy = fz = 0.0;
            return;
        }
        qw /= n; qx /= n; qy /= n; qz /= n;
        AeroWrench(dx, p, dx[IV], dx[IV + 1], dx[IV + 2],
                   1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy),
                   2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qw * qx),
                   2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy),
                   out Dual ax, out Dual ay, out Dual az, out _, out _, out _);
        fx = ax.V;
        fy = ay.V;
        fz = az.V;
    }

    /// <summary>
    /// The model's whole aerodynamic wrench at a state with the fins at <paramref name="finDeg"/> (empty: at rest): <paramref name="wrench"/> gets the force in the site frame, N (0-2), and the moment in model body axes, N m (3-5). For comparing the model with the game's own air (SixDof's AERO CHECK). False without a quaternion.
    /// </summary>
    public static bool AeroWrenchAt(ReadOnlySpan<double> x, Params p, ReadOnlySpan<double> finDeg, Span<double> wrench)
    {
        wrench.Clear();
        Span<Dual> dx = stackalloc Dual[NX];
        for (int i = 0; i < NX; i++) dx[i] = new Dual(x[i]);
        Dual qw = dx[IQ + 0], qx = dx[IQ + 1], qy = dx[IQ + 2], qz = dx[IQ + 3];
        Dual n = Dual.Sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (!(n.V > 0.0))
            return false;
        qw /= n; qx /= n; qy /= n; qz /= n;
        Span<Dual> fins = stackalloc Dual[finDeg.Length];
        for (int i = 0; i < finDeg.Length; i++) fins[i] = new Dual(finDeg[i]);
        AeroWrench(dx, p, dx[IV], dx[IV + 1], dx[IV + 2],
                   1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy),
                   2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qw * qx),
                   2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy),
                   out Dual fx, out Dual fy, out Dual fz, out Dual tx, out Dual ty, out Dual tz, fins);
        wrench[0] = fx.V; wrench[1] = fy.V; wrench[2] = fz.V;
        wrench[3] = tx.V; wrench[4] = ty.V; wrench[5] = tz.V;
        return true;
    }

    /// <summary>
    /// f(x, u) and its Jacobians A = df/dx (NX x NX) and B = df/du (NX x NU) at
    /// one node, both row-major.
    ///
    /// One sweep per input column, seeding that input's derivative to 1. The
    /// value is seed-independent, so f is taken from the first sweep rather than
    /// evaluated separately. Pass null for a Jacobian to skip nothing - the
    /// sweeps are shared - but to skip storing it.
    ///
    /// The RCS columns of B are written from their closed form instead of more sweeps: the RCS terms are linear in those controls, so each column is the force rotated into the site frame over the mass, and the torque over the inertia. Each sweep costs a full evaluation of the aero tables, so this keeps a craft with RCS at the cost of one without.
    ///  The control surfaces' columns are swept, one for each of the craft's fins (BodyAero6Dof.Fins): their torque is the air's, read from their tables at the node's flight condition.
    /// </summary>
    public static void Jacobian(ReadOnlySpan<double> x, ReadOnlySpan<double> u, Params p,
                                Span<double> f, Span<double> A, Span<double> B)
    {
        int NU = p.NU;
        if (u.Length < NU)
            throw new ArgumentException($"The controls are {u.Length} long; this craft has {NU} ({p.FinCount} fins).");
        Span<Dual> dx = stackalloc Dual[NX];
        Span<Dual> du = stackalloc Dual[NU];
        Span<Dual> dout = stackalloc Dual[NX];

        for (int col = 0; col < NX + NU; col++)
        {
            // The RCS columns come from their closed form (RcsColumns).
            if (col >= NX + IRCS && col < NX + IFIN)
                continue;
            for (int i = 0; i < NX; i++)
                dx[i] = new Dual(x[i], col == i ? 1.0 : 0.0);
            for (int j = 0; j < NU; j++)
                du[j] = new Dual(u[j], col == NX + j ? 1.0 : 0.0);

            F(dx, du, p, dout);

            if (col == 0 && !f.IsEmpty)
                for (int r = 0; r < NX; r++)
                    f[r] = dout[r].V;

            if (col < NX)
            {
                if (!A.IsEmpty)
                    for (int r = 0; r < NX; r++)
                        A[r * NX + col] = dout[r].D;
            }
            else if (!B.IsEmpty)
            {
                int bcol = col - NX;
                for (int r = 0; r < NX; r++)
                    B[r * NU + bcol] = dout[r].D;
            }
        }

        if (!B.IsEmpty)
            RcsColumns(x, p, B);
    }

    /// <summary>The propellant flow at one node's controls, kg/s, as F's mass row has it: the engine's line in its axial thrust.</summary>
    public static double MassFlow(Params p, ReadOnlySpan<double> u)
        => p.EngineOff ? 0.0 : p.MassFlowOffset + p.Alpha * u[IT];

    /// <summary>The RCS columns of B, row-major NX x NU: d(xdot)/d(u_rcs), exactly what F's RCS terms differentiate to.</summary>
    private static void RcsColumns(ReadOnlySpan<double> x, Params p, Span<double> B)
    {
        int NU = p.NU;
        for (int j = 0; j < NRCS; j++)
            for (int r = 0; r < NX; r++)
                B[r * NU + IRCS + j] = 0.0;
        if (p.Rcs == null)
            return;

        double qw = x[IQ], qx = x[IQ + 1], qy = x[IQ + 2], qz = x[IQ + 3];
        double r00 = 1.0 - 2.0 * (qy * qy + qz * qz), r01 = 2.0 * (qx * qy - qw * qz), r02 = 2.0 * (qx * qz + qw * qy);
        double r10 = 2.0 * (qx * qy + qw * qz), r11 = 1.0 - 2.0 * (qx * qx + qz * qz), r12 = 2.0 * (qy * qz - qw * qx);
        double r20 = 2.0 * (qx * qz - qw * qy), r21 = 2.0 * (qy * qz + qw * qx), r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
        double m = x[IM];
        for (int a = 0; a < NRCS; a++)
        {
            int c = IRCS + a;
            double fbx = p.Rcs.ForcePerNm[3 * a], fby = p.Rcs.ForcePerNm[3 * a + 1], fbz = p.Rcs.ForcePerNm[3 * a + 2];
            B[(IV + 0) * NU + c] = (r00 * fbx + r01 * fby + r02 * fbz) / m;
            B[(IV + 1) * NU + c] = (r10 * fbx + r11 * fby + r12 * fbz) / m;
            B[(IV + 2) * NU + c] = (r20 * fbx + r21 * fby + r22 * fbz) / m;
            double inertia = a == 0 ? p.Ixx : a == 1 ? p.Iyy : p.Izz;
            B[(IW + a) * NU + c] = 1.0 / inertia;
        }
    }

    /// <summary>Value only, no derivatives - for the nonlinear defect / merit function.</summary>
    public static void Eval(ReadOnlySpan<double> x, ReadOnlySpan<double> u, Params p, Span<double> f)
    {
        int NU = Math.Min(p.NU, u.Length);
        Span<Dual> dx = stackalloc Dual[NX];
        Span<Dual> du = stackalloc Dual[NU];
        Span<Dual> dout = stackalloc Dual[NX];
        for (int i = 0; i < NX; i++) dx[i] = new Dual(x[i]);
        for (int j = 0; j < NU; j++) du[j] = new Dual(u[j]);
        F(dx, du, p, dout);
        for (int r = 0; r < NX; r++) f[r] = dout[r].V;
    }
}

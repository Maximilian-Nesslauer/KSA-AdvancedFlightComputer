using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

namespace AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// Six-degree-of-freedom rigid-body booster dynamics, and their Jacobians by
/// forward-mode AD. Port of the model in Scvx/6dof.py - the ONE piece of
/// hand-written physics; every slope below comes from differentiating it.
///
///   state    X = [ r(3)  v(3)  q(4)  w(3)  m(1) ]                 (14)
///   control  u = [ tdx, tdy, T, tau_roll ]                        (4)
///
///   rdot = v
///   vdot = g + (R(q) T_body + F_aero) / m      T_body = [tdx, tdy, T]
///   qdot = 1/2 q (x) [0, w]              Hamilton, scalar-first
///   wdot = I^-1 ( tau + tau_aero - w x (I w) )      Euler's equation
///   mdot = -(mdot_0 + alpha * T)
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
    public const int NU = 4;    // control dimension

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

    /// <summary>Vehicle and environment constants. Defaults mirror 6dof.py.</summary>
    public sealed class Params
    {
        public double Gx = 0.0, Gy = 0.0, Gz = -9.81;   // gravity, inertial
        public double G0 = 9.81;
        public double Isp = 330.0;
        public double LArm = 25.0;                       // CoM -> gimbal pivot offset
        public double Ixx = 1.0e8, Iyy = 1.0e8, Izz = 2.5e6;

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

        Dual fx = r00 * tdx + r01 * tdy + r02 * T;
        Dual fy = r10 * tdx + r11 * tdy + r12 * T;
        Dual fz = r20 * tdx + r21 * tdy + r22 * T;

        // vdot = g + (R(q) T_body + F_aero) / m
        AeroWrench(x, p, vx, vy, vz, r00, r01, r02, r10, r11, r12, r20, r21, r22,
                   out Dual ax, out Dual ay, out Dual az, out Dual aeroTx, out Dual aeroTy, out Dual aeroTz);
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

        // wdot = I^-1 ( tau - w x (I w) ),  tau = [L*tdy, -L*tdx, tau_roll] plus the air's
        Dual taux = p.LArm * tdy + aeroTx;
        Dual tauy = -(p.LArm * tdx) + aeroTy;
        tauRoll += aeroTz;
        Dual iwx = p.Ixx * wx, iwy = p.Iyy * wy, iwz = p.Izz * wz;
        Dual gyrox = wy * iwz - wz * iwy;
        Dual gyroy = wz * iwx - wx * iwz;
        Dual gyroz = wx * iwy - wy * iwx;
        xdot[IW + 0] = (taux - gyrox) / p.Ixx;
        xdot[IW + 1] = (tauy - gyroy) / p.Iyy;
        xdot[IW + 2] = (tauRoll - gyroz) / p.Izz;

        // mdot = -(mdot_0 + alpha * T)
        xdot[IM] = -(p.MassFlowOffset + p.Alpha * T);
    }

    /// <summary>
    /// The aerodynamic force, site frame, N, and moment, model body axes, N m: the body-axis tables' share (BodyAero) and the whole-sphere force on the thrust axis for the rest (Aero, at R's third column); zero with neither.
    /// </summary>
    private static void AeroWrench(ReadOnlySpan<Dual> x, Params p, Dual vx, Dual vy, Dual vz,
                                   Dual r00, Dual r01, Dual r02, Dual r10, Dual r11, Dual r12, Dual r20, Dual r21, Dual r22,
                                   out Dual fx, out Dual fy, out Dual fz, out Dual tx, out Dual ty, out Dual tz)
    {
        fx = fy = fz = tx = ty = tz = default;
        Dual altitude = p.AeroAltitudeOffset + x[IR + 2];
        Dual weight = default;
        if (p.BodyAero != null)
            p.BodyAero.Wrench(altitude, vx, vy, vz, r00, r01, r02, r10, r11, r12, r20, r21, r22,
                              out fx, out fy, out fz, out tx, out ty, out tz, out weight);
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
    /// f(x, u) and its Jacobians A = df/dx (NX x NX) and B = df/du (NX x NU) at
    /// one node, both row-major.
    ///
    /// One sweep per input column, seeding that input's derivative to 1. The
    /// value is seed-independent, so f is taken from the first sweep rather than
    /// evaluated separately. Pass null for a Jacobian to skip nothing - the
    /// sweeps are shared - but to skip storing it.
    /// </summary>
    public static void Jacobian(ReadOnlySpan<double> x, ReadOnlySpan<double> u, Params p,
                                Span<double> f, Span<double> A, Span<double> B)
    {
        Span<Dual> dx = stackalloc Dual[NX];
        Span<Dual> du = stackalloc Dual[NU];
        Span<Dual> dout = stackalloc Dual[NX];

        for (int col = 0; col < NX + NU; col++)
        {
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
    }

    /// <summary>Value only, no derivatives - for the nonlinear defect / merit function.</summary>
    public static void Eval(ReadOnlySpan<double> x, ReadOnlySpan<double> u, Params p, Span<double> f)
    {
        Span<Dual> dx = stackalloc Dual[NX];
        Span<Dual> du = stackalloc Dual[NU];
        Span<Dual> dout = stackalloc Dual[NX];
        for (int i = 0; i < NX; i++) dx[i] = new Dual(x[i]);
        for (int j = 0; j < NU; j++) du[j] = new Dual(u[j]);
        F(dx, du, p, dout);
        for (int r = 0; r < NX; r++) f[r] = dout[r].V;
    }
}

using AdvancedFlightComputer.Guidance.Numerics;

namespace AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// Point-mass dynamics with the attitude as a control, as in Scvx/3dof.py.
///
///   state    x = [ r(3)  v(3)  m ]                        (7)
///   control  u = [ b(3)  throttle ]                        (4)
///
/// b is the body axis, a unit vector, and sets BOTH the thrust direction and the angle of attack. Lift and drag come from b's angle to the airflow, not from the thrust, so the attitude still steers with the engine off - which is what makes a glide a phase worth planning.
///
/// THE THROTTLE IS A FRACTION, not newtons. 3dof.py's control is the thrust magnitude Tm with a constant Tmax, and every place it bounds, smooths or trusts Tm it divides by Tmax first; the fraction is that quotient. It also leaves room for thrust that depends on back pressure, the way <see cref="Ascent.AscentDynamics"/> does, without the throttle box stopping being a box.
///
/// The model is an interface because the same solver runs two of them: <see cref="Toy3DofModel"/>, which is 3dof.py's physics line for line so the port can be diffed against the script, and the KSA model the guidance flies with.
/// </summary>
public interface IPointMassModel
{
    /// <summary>xdot = f(x, u), generic over Dual so one evaluation yields any Jacobian column.</summary>
    void F(ReadOnlySpan<Dual> x, ReadOnlySpan<Dual> u, Span<Dual> xdot);

    /// <summary>Dynamic pressure at a state, Pa, for the q-alpha limit. Values only, never differentiated.</summary>
    double DynamicPressure(ReadOnlySpan<double> x);
}

public static class PointMass3Dof
{
    public const int NX = 7;
    public const int NU = 4;

    public const int IR = 0;    // position   r(3)
    public const int IV = 3;    // velocity   v(3)
    public const int IM = 6;    // mass

    public const int IB = 0;    // body axis  b(3)
    public const int ITH = 3;   // throttle fraction

    /// <summary>Softens |v| at zero speed in the angle-of-attack projection. 3dof.py's V_EPS2.</summary>
    public const double VEps2 = 1e-4;

    /// <summary>
    /// f(x, u) and its Jacobians A = df/dx (NX x NX) and B = df/du (NX x NU), both row-major, by one forward sweep per input.
    /// </summary>
    public static void Jacobian(IPointMassModel model, ReadOnlySpan<double> x, ReadOnlySpan<double> u,
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

            model.F(dx, du, dout);

            if (col == 0)
                for (int r = 0; r < NX; r++)
                    f[r] = dout[r].V;

            if (col < NX)
                for (int r = 0; r < NX; r++)
                    A[r * NX + col] = dout[r].D;
            else
                for (int r = 0; r < NX; r++)
                    B[r * NU + col - NX] = dout[r].D;
        }
    }

    /// <summary>Value only, for the nonlinear defect.</summary>
    public static void Eval(IPointMassModel model, ReadOnlySpan<double> x, ReadOnlySpan<double> u, Span<double> f)
    {
        Span<Dual> dx = stackalloc Dual[NX];
        Span<Dual> du = stackalloc Dual[NU];
        Span<Dual> dout = stackalloc Dual[NX];
        for (int i = 0; i < NX; i++) dx[i] = new Dual(x[i]);
        for (int j = 0; j < NU; j++) du[j] = new Dual(u[j]);
        model.F(dx, du, dout);
        for (int r = 0; r < NX; r++) f[r] = dout[r].V;
    }

    /// <summary>
    /// The body axis projected perpendicular to the velocity, p = b - (b . v_hat) v_hat, whose length is sin(alpha), and its slopes with respect to v (3x3) and b (3x3), row-major. 3dof.py's p_vec.
    ///
    /// The angle-of-attack limit bounds |p| rather than cos(alpha) because p has a non-zero slope even when b lies along v, where cos(alpha) is stationary - the same trap the 6-DOF tilt row fell into at vertical.
    /// </summary>
    public static void ProjectionJacobian(ReadOnlySpan<double> x, ReadOnlySpan<double> u,
                                          Span<double> p, Span<double> dpdv, Span<double> dpdb)
    {
        for (int col = 0; col < 6; col++)
        {
            Dual vx = new(x[IV + 0], col == 0 ? 1.0 : 0.0);
            Dual vy = new(x[IV + 1], col == 1 ? 1.0 : 0.0);
            Dual vz = new(x[IV + 2], col == 2 ? 1.0 : 0.0);
            Dual bx = new(u[IB + 0], col == 3 ? 1.0 : 0.0);
            Dual by = new(u[IB + 1], col == 4 ? 1.0 : 0.0);
            Dual bz = new(u[IB + 2], col == 5 ? 1.0 : 0.0);
            Project(vx, vy, vz, bx, by, bz, out Dual px, out Dual py, out Dual pz);

            if (col == 0)
            {
                p[0] = px.V;
                p[1] = py.V;
                p[2] = pz.V;
            }
            Span<double> J = col < 3 ? dpdv : dpdb;
            int c = col % 3;
            J[0 * 3 + c] = px.D;
            J[1 * 3 + c] = py.D;
            J[2 * 3 + c] = pz.D;
        }
    }

    private static void Project(Dual vx, Dual vy, Dual vz, Dual bx, Dual by, Dual bz,
                                out Dual px, out Dual py, out Dual pz)
    {
        Dual vmag = Dual.Sqrt(vx * vx + vy * vy + vz * vz + VEps2);
        Dual hx = vx / vmag, hy = vy / vmag, hz = vz / vmag;
        Dual bv = bx * hx + by * hy + bz * hz;
        px = bx - bv * hx;
        py = by - bv * hy;
        pz = bz - bv * hz;
    }
}

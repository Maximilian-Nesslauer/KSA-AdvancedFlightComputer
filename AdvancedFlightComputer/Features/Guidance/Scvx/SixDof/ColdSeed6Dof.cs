namespace AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// A cold-solve reference that obeys the translational dynamics: the cubic from the measured position and velocity to the target ones over the burn time, the thrust that flies it, the attitude that points the thrust, and the mass that thrust burns.
/// SCvx moves its reference by at most a trust region per accepted step, 0.1 of each scale. A reference with hover thrust and an attitude blended straight to upright is ten or more accepted steps from a burn that brakes at many times the local gravity with the thrust axis near the horizon, and a spread solve that starts again from the same reference after a few refused steps never gets there.
/// </summary>
public static class ColdSeed6Dof
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    // The share of the burn over which the attitude turns from the thrust direction to the target attitude.
    private const double UprightShare = 0.15;

    public static void Build(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf, double sigma, int nodes,
                             Scvx6DofConfig cfg, Dynamics6Dof.Params dyn, out double[] xSeed, out double[] uSeed)
    {
        if (nodes < 2 || x0.Length < NX || xf.Length < NX || !(sigma > 0.0))
            throw new ArgumentException("The cold seed needs two nodes, complete states and a positive duration.");

        xSeed = new double[nodes * NX];
        uSeed = new double[nodes * NU];
        double dt = sigma / (nodes - 1);
        double mass = x0[Dynamics6Dof.IM];
        double massFloor = 0.2 * mass;
        Span<double> gravity = [dyn.Gx, dyn.Gy, dyn.Gz];
        Span<double> axis = stackalloc double[3];
        Span<double> q = stackalloc double[4];
        Span<double> turned = stackalloc double[4];
        x0.Slice(Dynamics6Dof.IQ, 4).CopyTo(q);
        double previousThrust = 0.0;
        int uprightFrom = Math.Max((int)Math.Ceiling((1.0 - UprightShare) * (nodes - 1)), 1);

        for (int k = 0; k < nodes; k++)
        {
            double s = (double)k / (nodes - 1);
            // The cubic Hermite segment and its first two derivatives in the normalised time s.
            double h00 = 2 * s * s * s - 3 * s * s + 1, h10 = s * s * s - 2 * s * s + s, h01 = -2 * s * s * s + 3 * s * s, h11 = s * s * s - s * s;
            double d00 = 6 * s * s - 6 * s, d10 = 3 * s * s - 4 * s + 1, d01 = -6 * s * s + 6 * s, d11 = 3 * s * s - 2 * s;
            double a00 = 12 * s - 6, a10 = 6 * s - 4, a01 = -12 * s + 6, a11 = 6 * s - 2;

            int o = k * NX;
            double thrust = 0.0;
            for (int i = 0; i < 3; i++)
            {
                double r0 = x0[Dynamics6Dof.IR + i], rf = xf[Dynamics6Dof.IR + i];
                double v0 = x0[Dynamics6Dof.IV + i] * sigma, vf = xf[Dynamics6Dof.IV + i] * sigma;
                xSeed[o + Dynamics6Dof.IR + i] = h00 * r0 + h10 * v0 + h01 * rf + h11 * vf;
                xSeed[o + Dynamics6Dof.IV + i] = (d00 * r0 + d10 * v0 + d01 * rf + d11 * vf) / sigma;
                double accel = (a00 * r0 + a10 * v0 + a01 * rf + a11 * vf) / (sigma * sigma);
                axis[i] = accel - gravity[i];
                thrust += axis[i] * axis[i];
            }
            thrust = Math.Sqrt(thrust);

            // The mass the thrust of the interval before this node burned.
            if (k > 0)
                mass = Math.Max(mass - dt * (dyn.MassFlowOffset + dyn.Alpha * previousThrust), massFloor);
            xSeed[o + Dynamics6Dof.IM] = mass;
            double force = Math.Clamp(thrust * mass, cfg.Tmin, cfg.Tmax);
            uSeed[k * NU + Dynamics6Dof.IT] = force;
            previousThrust = force;

            // The attitude follows the thrust direction by the shortest turn from the node before, which keeps the measured roll about the thrust axis, and blends to the target attitude over the last part of the burn.
            if (k > 0 && thrust > 1e-9)
            {
                for (int i = 0; i < 3; i++) axis[i] /= thrust;
                PointBodyZ(q, axis, turned);
                turned.CopyTo(q);
            }
            Span<double> node = xSeed.AsSpan(o + Dynamics6Dof.IQ, 4);
            if (k >= uprightFrom)
                Slerp(q, xf.Slice(Dynamics6Dof.IQ, 4), (double)(k - uprightFrom + 1) / (nodes - uprightFrom), node);
            else
                q.CopyTo(node);
        }

        // Node 0 is the measured state exactly, because the solver pins it by an equality inside the trust region.
        x0[..NX].CopyTo(xSeed);

        // Body rates that turn each node's attitude into the next, w = 2 conj(q) (q1 - q) / dt for a small turn.
        for (int k = 1; k < nodes - 1; k++)
        {
            ReadOnlySpan<double> a = xSeed.AsSpan(k * NX + Dynamics6Dof.IQ, 4);
            ReadOnlySpan<double> b = xSeed.AsSpan((k + 1) * NX + Dynamics6Dof.IQ, 4);
            double sign = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] < 0.0 ? -1.0 : 1.0;
            double dw = sign * b[0] - a[0], dx = sign * b[1] - a[1], dy = sign * b[2] - a[2], dz = sign * b[3] - a[3];
            int w = k * NX + Dynamics6Dof.IW;
            xSeed[w] = 2.0 * (a[0] * dx - a[1] * dw - a[2] * dz + a[3] * dy) / dt;
            xSeed[w + 1] = 2.0 * (a[0] * dy + a[1] * dz - a[2] * dw - a[3] * dx) / dt;
            xSeed[w + 2] = 2.0 * (a[0] * dz - a[1] * dy + a[2] * dx - a[3] * dw) / dt;
        }
    }

    /// <summary>
    /// Makes a reference roll with a craft that has no roll torque (Scvx6DofConfig.TerminalRollFree): each node is turned about its own thrust axis by the angle the measured roll rate covers up to that node, and carries that roll rate. The thrust axis of no node moves. Without it the reference stops the roll, which the dynamics cannot do, and every node past the first is a roll angle away from where the craft will be, further with each node and far outside a trust region by the end.
    /// </summary>
    public static void KeepRoll(double[] xSeed, int nodes, double sigma)
    {
        double rollRate = xSeed[Dynamics6Dof.IW + 2];
        for (int k = 1; k < nodes; k++)
        {
            double angle = rollRate * sigma * k / (nodes - 1);
            double c = Math.Cos(0.5 * angle), s = Math.Sin(0.5 * angle);
            int q = k * NX + Dynamics6Dof.IQ;
            double qw = xSeed[q], qx = xSeed[q + 1], qy = xSeed[q + 2], qz = xSeed[q + 3];
            // q (c, 0, 0, s), a turn about the body z axis.
            xSeed[q] = qw * c - qz * s;
            xSeed[q + 1] = qx * c + qy * s;
            xSeed[q + 2] = qy * c - qx * s;
            xSeed[q + 3] = qz * c + qw * s;

            // The pitch and yaw rates are the same vectors, read in the turned body axes.
            int w = k * NX + Dynamics6Dof.IW;
            double wx = xSeed[w], wy = xSeed[w + 1];
            double ca = Math.Cos(angle), sa = Math.Sin(angle);
            xSeed[w] = wx * ca + wy * sa;
            xSeed[w + 1] = wy * ca - wx * sa;
            xSeed[w + 2] = rollRate;
        }
    }

    /// <summary>
    /// How far the straight-line reference is from the burn the states ask for: the thrust that turns the measured velocity into the target one over the burn time, as a multiple of the hover thrust the straight line holds.
    /// </summary>
    public static double BrakingLoad(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf, double sigma, Dynamics6Dof.Params dyn)
    {
        double gravity = Math.Sqrt(dyn.Gx * dyn.Gx + dyn.Gy * dyn.Gy + dyn.Gz * dyn.Gz);
        if (!(sigma > 0.0) || !(gravity > 0.0))
            return 0.0;
        double ax = (xf[Dynamics6Dof.IV] - x0[Dynamics6Dof.IV]) / sigma - dyn.Gx;
        double ay = (xf[Dynamics6Dof.IV + 1] - x0[Dynamics6Dof.IV + 1]) / sigma - dyn.Gy;
        double az = (xf[Dynamics6Dof.IV + 2] - x0[Dynamics6Dof.IV + 2]) / sigma - dyn.Gz;
        return Math.Sqrt(ax * ax + ay * ay + az * az) / gravity;
    }

    /// <summary>The attitude q turned by the shortest rotation that brings its body +Z onto the unit vector axis. Scalar-first, body to inertial. The turn has no part about the body Z axis, so a chain of them from a measured attitude keeps the measured roll.</summary>
    public static void PointBodyZ(ReadOnlySpan<double> q, ReadOnlySpan<double> axis, Span<double> result)
    {
        double qw = q[0], qx = q[1], qy = q[2], qz = q[3];
        double zx = 2.0 * (qx * qz + qw * qy), zy = 2.0 * (qy * qz - qw * qx), zz = 1.0 - 2.0 * (qx * qx + qy * qy);
        double cx = zy * axis[2] - zz * axis[1], cy = zz * axis[0] - zx * axis[2], cz = zx * axis[1] - zy * axis[0];
        double dot = zx * axis[0] + zy * axis[1] + zz * axis[2];
        // The half-angle form of the rotation between two unit vectors. Opposite vectors have no shortest turn, so the attitude stays for that node.
        double w = 1.0 + dot;
        if (w < 1e-9)
        {
            q.CopyTo(result);
            return;
        }
        double norm = Math.Sqrt(w * w + cx * cx + cy * cy + cz * cz);
        w /= norm; cx /= norm; cy /= norm; cz /= norm;
        result[0] = w * qw - cx * qx - cy * qy - cz * qz;
        result[1] = w * qx + cx * qw + cy * qz - cz * qy;
        result[2] = w * qy - cx * qz + cy * qw + cz * qx;
        result[3] = w * qz + cx * qy - cy * qx + cz * qw;
    }

    public static void Slerp(ReadOnlySpan<double> a, ReadOnlySpan<double> b, double t, Span<double> result)
    {
        double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
        double sign = dot < 0.0 ? -1.0 : 1.0;
        dot = Math.Min(Math.Abs(dot), 1.0);
        double wa = 1.0 - t, wb = t;
        if (dot < 0.9995)
        {
            double theta = Math.Acos(dot), sin = Math.Sin(theta);
            wa = Math.Sin((1.0 - t) * theta) / sin;
            wb = Math.Sin(t * theta) / sin;
        }
        double norm = 0.0;
        for (int i = 0; i < 4; i++)
        {
            result[i] = wa * a[i] + wb * sign * b[i];
            norm += result[i] * result[i];
        }
        norm = Math.Sqrt(norm);
        for (int i = 0; i < 4; i++)
            result[i] /= norm;
    }
}

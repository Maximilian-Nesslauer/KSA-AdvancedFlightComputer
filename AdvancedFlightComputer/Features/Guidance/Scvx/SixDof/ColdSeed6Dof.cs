namespace AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// A cold-solve reference that obeys the translational dynamics: the cubic from the measured position and velocity to the target ones over the burn time, the thrust that flies it, the attitude that points the thrust, and the mass that thrust burns.
/// SCvx moves its reference by at most a trust region per accepted step, 0.1 of each scale. A reference with hover thrust and an attitude blended straight to upright is ten or more accepted steps from a burn that brakes at many times the local gravity with the thrust axis near the horizon, and a spread solve that starts again from the same reference after a few refused steps never gets there.
/// </summary>
/// <summary>
/// A path to lay a cold seed along (ColdSeed6Dof.BuildAlongPath): in phase 0, the coast, or 1, the burn, at fraction f of that phase, the position and velocity (site frame, as the 6-DOF state), the body z axis wanted - the attitude in the coast, the thrust direction in the burn - and the thrust, N. False where it has no sample.
/// </summary>
public delegate bool SeedPath(int phase, double f, Span<double> rv, Span<double> axis, out double thrustN);

public static class ColdSeed6Dof
{
    private const int NX = Dynamics6Dof.NX;

    // The share of the burn over which the attitude turns from the thrust direction to the target attitude.
    private const double UprightShare = 0.15;

    public static void Build(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf, double sigma, int nodes,
                             Scvx6DofConfig cfg, Dynamics6Dof.Params dyn, out double[] xSeed, out double[] uSeed)
    {
        int NU = cfg.NU;
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
    /// A cold reference for a plan with a coast before ignition (Scvx6DofConfig.CoastIntervals): the coast flown out with the engine off and the RCS damping the body rates, so its nodes obey the dynamics, then <see cref="Build"/>'s burn from where the coast leaves the craft. The coast's controls are the damping's; nothing steers.
    /// </summary>
    public static void BuildWithCoast(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf, double coastSigma, double sigma,
                                      int nodes, int coastIntervals, Scvx6DofConfig cfg, Dynamics6Dof.Params dyn,
                                      out double[] xSeed, out double[] uSeed)
    {
        int NU = cfg.NU;
        if (coastIntervals <= 0)
        {
            Build(x0, xf, sigma, nodes, cfg, dyn, out xSeed, out uSeed);
            return;
        }
        if (coastIntervals > nodes - 2)
            throw new ArgumentException("The coast leaves the burn fewer than two nodes.");

        xSeed = new double[nodes * NX];
        uSeed = new double[nodes * NU];
        Dynamics6Dof.Params coast = dyn.WithEngineOff();
        double[] x = x0[..NX].ToArray();
        double[] u = new double[NU];
        double dt = Math.Max(coastSigma, 0.0) / coastIntervals;
        const int Substeps = 8;
        // The heading the plan ends on, which the coast holds (HoldHeading).
        double heading = 2.0 * Math.Atan2(xf[Dynamics6Dof.IQ + 3], xf[Dynamics6Dof.IQ]);
        for (int k = 0; k < coastIntervals; k++)
        {
            HoldHeading(x, cfg, dyn, heading, u);
            Array.Copy(x, 0, xSeed, k * NX, NX);
            Array.Copy(u, 0, uSeed, k * NU, NU);
            for (int s = 0; s < Substeps; s++)
            {
                HoldHeading(x, cfg, dyn, heading, u);
                Rk4(x, u, coast, dt / Substeps);
            }
        }

        // With the end heading free the burn ends upright on the heading it starts with, rather than turning back to one nothing asks for.
        double[] burnEnd = xf[..NX].ToArray();
        if (cfg.TerminalRollFree)
        {
            double qw = x[Dynamics6Dof.IQ], qz = x[Dynamics6Dof.IQ + 3], norm = Math.Sqrt(qw * qw + qz * qz);
            if (norm > 1e-9)
            {
                burnEnd[Dynamics6Dof.IQ] = qw / norm;
                burnEnd[Dynamics6Dof.IQ + 1] = 0.0;
                burnEnd[Dynamics6Dof.IQ + 2] = 0.0;
                burnEnd[Dynamics6Dof.IQ + 3] = qz / norm;
            }
        }
        Build(x, burnEnd, sigma, nodes - coastIntervals, cfg, dyn, out double[] xb, out double[] ub);
        // THE ROLL RATE CARRIES ACROSS IGNITION. Build starts the burn's nodes from no roll, so a coast the air had rolled up (172 deg/s by ignition in the 2026-11-20 13:13 seed, against 22 kN m of RCS roll) jumped to none in the first burn interval: a roll-rate defect of 2-3 rad/s that the cold solve took off at 0.1 rad/s a step, the "wz at 20" of every stalled coast standby. With the heading free the burn keeps rolling at that rate, as a burn-only seed keeps the measured roll (KeepRoll).
        if (cfg.TerminalRollFree)
            KeepRoll(xb, nodes - coastIntervals, sigma);
        Array.Copy(xb, 0, xSeed, coastIntervals * NX, xb.Length);
        Array.Copy(ub, 0, uSeed, coastIntervals * NU, ub.Length);
        x0[..NX].CopyTo(xSeed);
    }

    /// <summary>
    /// A cold reference for a plan with a coast laid ALONG A PATH another planner already flies: entry guidance's glide and burn, sampled at the nodes (<see cref="SeedPath"/>), its two ends pulled onto the measured state and the target.
    ///  WHY. <see cref="BuildWithCoast"/> lets the coast fall free and brakes from wherever that leaves the craft along a cubic. Entry guidance steers its glide with the air to an ignition aimed at the pad and runs its glide's attitude on into its burn's; a free fall over 15 s ends somewhere else, and the cubic from there opens its burn on a different thrust direction. In the 2026-11-20 14:16 seed the tilt went from 32 deg on the last coast node to 59 deg on the first burn node; the plan the solve converged on turned the other way, from 34 deg down to 11 at ignition, as the craft flew it. Entry's plan is that answer to within its point-mass model: positions, velocities and thrust that brake onto the pad, and one continuous thrust direction across ignition.
    ///  THE ENDS. The path starts where entry measured the craft last and ends on entry's target, not touchdown guidance's. The difference at the start is taken out over the coast, at the end put in over the burn, each by the cubic that matches position and velocity, so no node jumps; the end's correction's acceleration is added to the thrust.
    ///  THE ATTITUDE. Body z on the path's axis at every node - entry's commanded body axis in the glide, its thrust direction in the burn - turned the shortest way from the node before starting from the measured attitude, so the measured roll is kept (PointBodyZ), then blended onto the target attitude over the burn's last UprightShare as <see cref="Build"/> does. Rates are the turns between nodes; the RCS and gimbal are left to the solve.
    /// </summary>
    public static void BuildAlongPath(ReadOnlySpan<double> x0, ReadOnlySpan<double> xf, double coastSigma, double sigma,
                                      int nodes, int coastIntervals, Scvx6DofConfig cfg, Dynamics6Dof.Params dyn, SeedPath path,
                                      out double[] xSeed, out double[] uSeed)
    {
        int NU = cfg.NU;
        if (coastIntervals <= 0 || coastIntervals > nodes - 2 || !(sigma > 0.0))
            throw new ArgumentException("A seed along a path needs a coast and a burn of at least two nodes.");
        int burnNodes = nodes - coastIntervals;
        coastSigma = Math.Max(coastSigma, 0.0);
        xSeed = new double[nodes * NX];
        uSeed = new double[nodes * NU];

        // The path at every node: position and velocity, the axis, the thrust.
        var rv = new double[nodes * 6];
        var axes = new double[nodes * 3];
        var thrust = new double[nodes];
        var time = new double[nodes];
        for (int k = 0; k < nodes; k++)
        {
            bool burn = k >= coastIntervals;
            double f = burn ? (double)(k - coastIntervals) / (burnNodes - 1) : (double)k / coastIntervals;
            time[k] = burn ? coastSigma + f * sigma : f * coastSigma;
            if (!path(burn ? 1 : 0, f, rv.AsSpan(k * 6, 6), axes.AsSpan(k * 3, 3), out double t) || !double.IsFinite(t))
                throw new ArgumentException("The seed path has no sample at node " + k + ".");
            thrust[k] = burn ? Math.Clamp(t, cfg.Tmin, cfg.Tmax) : 0.0;
        }

        // The corrections: the start's out over the coast, the end's in over the burn.
        Span<double> d0 = stackalloc double[6];
        Span<double> df = stackalloc double[6];
        for (int i = 0; i < 6; i++)
        {
            d0[i] = x0[i] - rv[i];
            df[i] = xf[i] - rv[(nodes - 1) * 6 + i];
        }
        Span<double> force = stackalloc double[3];
        Span<double> axis = stackalloc double[3];
        Span<double> q = stackalloc double[4];
        Span<double> turned = stackalloc double[4];
        x0.Slice(Dynamics6Dof.IQ, 4).CopyTo(q);
        double mass = x0[Dynamics6Dof.IM], massFloor = 0.2 * mass;
        int uprightFrom = coastIntervals + Math.Max((int)Math.Ceiling((1.0 - UprightShare) * (burnNodes - 1)), 1);
        for (int k = 0; k < nodes; k++)
        {
            int o = k * NX;
            bool burn = k >= coastIntervals;
            // The mass the thrust of the interval before this node burned.
            if (k > coastIntervals)
                mass = Math.Max(mass - (sigma / (burnNodes - 1)) * (dyn.MassFlowOffset + dyn.Alpha * uSeed[(k - 1) * NU + Dynamics6Dof.IT]), massFloor);
            xSeed[o + Dynamics6Dof.IM] = mass;
            // The start's correction runs from the measured difference to none at ignition; with no coast to run over, over the burn.
            double span0 = coastSigma > 1e-6 ? coastSigma : sigma;
            double s0 = Math.Min(time[k] / span0, 1.0);
            double sf = burn ? (time[k] - coastSigma) / sigma : 0.0;
            for (int i = 0; i < 3; i++)
            {
                // (h00, h10 span) on the start's difference toward nothing, (h01, h11 span) on the end's from nothing.
                Hermite(s0, d0[i], d0[3 + i] * span0, 0.0, 0.0, out double p0, out double v0, out _);
                Hermite(sf, 0.0, 0.0, df[i], df[3 + i] * sigma, out double pf, out double vf, out double af);
                xSeed[o + Dynamics6Dof.IR + i] = rv[k * 6 + i] + p0 + pf;
                xSeed[o + Dynamics6Dof.IV + i] = rv[k * 6 + 3 + i] + v0 / span0 + (burn ? vf / sigma : 0.0);
                force[i] = thrust[k] * axes[k * 3 + i] + (burn ? mass * af / (sigma * sigma) : 0.0);
            }
            double magnitude = burn ? Math.Sqrt(force[0] * force[0] + force[1] * force[1] + force[2] * force[2]) : 0.0;

            uSeed[k * NU + Dynamics6Dof.IT] = burn ? Math.Clamp(magnitude, cfg.Tmin, cfg.Tmax) : 0.0;

            // Body z on the path's axis (the thrust's, with the end's correction, in the burn), the shortest turn from the node before.
            double norm = 0.0;
            for (int i = 0; i < 3; i++)
            {
                axis[i] = burn && magnitude > 1e-6 ? force[i] : axes[k * 3 + i];
                norm += axis[i] * axis[i];
            }
            norm = Math.Sqrt(norm);
            if (k > 0 && norm > 1e-9)
            {
                for (int i = 0; i < 3; i++) axis[i] /= norm;
                PointBodyZ(q, axis, turned);
                turned.CopyTo(q);
            }
            Span<double> node = xSeed.AsSpan(o + Dynamics6Dof.IQ, 4);
            if (k >= uprightFrom)
                Slerp(q, EndAttitude(xf, q, cfg), (double)(k - uprightFrom + 1) / (nodes - uprightFrom), node);
            else
                q.CopyTo(node);
        }
        // Node 0 is the measured state exactly, the last the target's rates.
        x0[..NX].CopyTo(xSeed);
        for (int i = 0; i < 3; i++)
            xSeed[(nodes - 1) * NX + Dynamics6Dof.IW + i] = xf[Dynamics6Dof.IW + i];

        // Body rates that turn each node's attitude into the next over its own interval.
        for (int k = 1; k < nodes - 1; k++)
        {
            double dt = time[k + 1] - time[k];
            if (!(dt > 1e-9))
                continue;
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

    /// <summary>The attitude a seed blends onto at the end: the target's, or with the end heading free, upright on the heading the seed has there.</summary>
    private static double[] EndAttitude(ReadOnlySpan<double> xf, ReadOnlySpan<double> q, Scvx6DofConfig cfg)
    {
        double[] end = xf.Slice(Dynamics6Dof.IQ, 4).ToArray();
        if (cfg.TerminalRollFree)
        {
            double norm = Math.Sqrt(q[0] * q[0] + q[3] * q[3]);
            if (norm > 1e-9)
                end = [q[0] / norm, 0.0, 0.0, q[3] / norm];
        }
        return end;
    }

    /// <summary>The cubic Hermite through (p0, m0) at s 0 and (p1, m1) at s 1, the tangents per unit s: its value and first and second derivatives in s.</summary>
    private static void Hermite(double s, double p0, double m0, double p1, double m1, out double p, out double dp, out double ddp)
    {
        p = (2 * s * s * s - 3 * s * s + 1) * p0 + (s * s * s - 2 * s * s + s) * m0 + (-2 * s * s * s + 3 * s * s) * p1 + (s * s * s - s * s) * m1;
        dp = (6 * s * s - 6 * s) * p0 + (3 * s * s - 4 * s + 1) * m0 + (-6 * s * s + 6 * s) * p1 + (3 * s * s - 2 * s) * m1;
        ddp = (12 * s - 6) * p0 + (6 * s - 4) * m0 + (-12 * s + 6) * p1 + (6 * s - 2) * m1;
    }

    /// <summary>
    /// The coast's RCS: the roll held on the plan's end heading, the pitch and yaw rates damped so the air can still turn the craft onto its flight path.
    ///  HELD, NOT ONLY DAMPED. Damping the rates alone let the air's roll moment turn the 2026-11-20 booster 75 deg about its axis over a 15 s coast (C_l -0.002 to -0.005 on a roll inertia of 65,000 kg m^2). The burn after it kept that heading and blended back to the end heading over its last 15 %: 84 deg of roll in under a second, an 88 deg/s spike that took the cold solve about 15 iterations to take out at 0.1 rad/s a step - past the standby's 12, so it started again from the same seed, every time (--6dof-coast --roll-pinned --seed-defects).
    /// </summary>
    private static void HoldHeading(double[] x, Scvx6DofConfig cfg, Dynamics6Dof.Params dyn, double heading, double[] u)
    {
        Damp(x, cfg, dyn, u);
        // A plan that leaves the end heading free holds none: the rates are damped and the roll goes where the air and the RCS take it.
        if (dyn.Rcs == null || cfg.TerminalRollFree)
            return;
        // The heading as TerminalAttitude reads it: the twist part of the attitude about the site's vertical.
        double now = 2.0 * Math.Atan2(x[Dynamics6Dof.IQ + 3], x[Dynamics6Dof.IQ]);
        double error = Math.IEEERemainder(now - heading, 2.0 * Math.PI);
        const double Kp = 1.0, Kd = 2.0;   // 1 rad/s, critically damped
        (double pos, double neg) = cfg.RcsBounds(2);
        u[Dynamics6Dof.IRCS + 2] = Math.Clamp(-dyn.Izz * (Kp * error + Kd * x[Dynamics6Dof.IW + 2]), -neg, pos);
    }

    /// <summary>The RCS torque that takes out the body rates over half a second, inside the RCS's bounds; zero without RCS.</summary>
    private static void Damp(double[] x, Scvx6DofConfig cfg, Dynamics6Dof.Params dyn, double[] u)
    {
        Array.Clear(u);
        if (dyn.Rcs == null)
            return;
        const double Tau = 0.5;
        for (int a = 0; a < Dynamics6Dof.NRCS; a++)
        {
            double inertia = a == 0 ? dyn.Ixx : a == 1 ? dyn.Iyy : dyn.Izz;
            (double pos, double neg) = cfg.RcsBounds(a);
            u[Dynamics6Dof.IRCS + a] = Math.Clamp(-inertia * x[Dynamics6Dof.IW + a] / Tau, -neg, pos);
        }
    }

    private static void Rk4(double[] x, double[] u, Dynamics6Dof.Params p, double h)
    {
        Span<double> k1 = stackalloc double[NX], k2 = stackalloc double[NX], k3 = stackalloc double[NX], k4 = stackalloc double[NX];
        double[] tmp = new double[NX];
        Dynamics6Dof.Eval(x, u, p, k1);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k1[i];
        Dynamics6Dof.Eval(tmp, u, p, k2);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k2[i];
        Dynamics6Dof.Eval(tmp, u, p, k3);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + h * k3[i];
        Dynamics6Dof.Eval(tmp, u, p, k4);
        for (int i = 0; i < NX; i++) x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        double norm = Math.Sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
        for (int i = 6; i < 10; i++) x[i] /= norm;
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

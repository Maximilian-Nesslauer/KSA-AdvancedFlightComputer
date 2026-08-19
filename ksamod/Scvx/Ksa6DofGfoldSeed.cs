using System;
using Gfold;
using Scvx;

/// <summary>
/// Builds the 6-DOF cold-start seed by solving the 3-DOF G-FOLD problem first.
///
/// SCvx does not search for a trajectory, it REFINES one. It linearises about the
/// reference it was handed and can only walk a trust region's distance per iteration,
/// so the quality of the initial guess sets both how many iterations the cold solve
/// needs and whether it finds a sensible trajectory at all. The default seed is a
/// straight line from the vehicle to the target at constant thrust, which satisfies
/// neither the dynamics nor the constraints and is nowhere near the optimum.
///
/// G-FOLD is a much better guess and costs almost nothing to obtain. It solves the
/// same landing under a 3-DOF point-mass model, but it is CONVEX - lossless
/// convexification turns the non-convex thrust-magnitude bounds into a second-order
/// cone - so it has no local minima, needs no initial guess of its own, and returns
/// the global optimum of its own problem in a handful of milliseconds. A golden-section
/// search over time of flight then gives a burn duration that is close to optimal
/// rather than a guess.
///
/// What it does NOT model is attitude: it commands an acceleration vector directly and
/// assumes the vehicle can point wherever it likes instantly. That is exactly the part
/// SCvx adds, and it is why this is a seed and not an answer. But the position,
/// velocity, mass and burn-time profiles are close to right, and the commanded
/// acceleration DIRECTION is a good guess at where the vehicle should be pointing -
/// which gives the attitude channel a sensible seed too.
///
/// FRAMES: G-FOLD works in a local frame with x UP; the 6-DOF model has z up. The
/// mapping used here is gfold(x,y,z) = model(z,x,y), a cyclic permutation and so
/// right-handed, applied consistently in both directions.
/// </summary>
public static class Ksa6DofGfoldSeed
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    /// <summary>Nodes for the G-FOLD solve itself. Independent of the 6-DOF count; the result is resampled.</summary>
    private const int GfoldNodes = 60;

    /// <summary>
    /// Try to produce a seed. Returns false if G-FOLD cannot solve the case, in which
    /// case the caller should fall back to the straight-line seed — a worse guess is
    /// much better than no plan.
    /// </summary>
    /// <summary>Milliseconds spent inside G-FOLD on the last TryBuild, for measurement.</summary>
    public static double LastGfoldMs { get; private set; }

    /// <summary>
    /// Search over time of flight, or take a single solve at an estimated duration?
    ///
    /// The search costs an ECOS solve pair per sample and dominates the seed's cost,
    /// while the SEED does not need the optimal burn time - SCvx re-optimises sigma
    /// anyway. Measured by Scvx.Console --seed.
    /// </summary>
    public static bool SearchTimeOfFlight { get; set; }

    public static bool TryBuild(double[] x0, double[] xf, Scvx6DofConfig cfg,
                                Dynamics6Dof.Params dyn, int nodes,
                                out double[] xSeed, out double[] uSeed, out double sigma,
                                out string note)
    {
        xSeed = [];
        uSeed = [];
        sigma = 0.0;
        note = "";

        try
        {
            double g = Math.Abs(dyn.Gz);
            double m0 = x0[Dynamics6Dof.IM];
            double isp = dyn.Isp;
            if (!(g > 0.0) || !(m0 > 0.0) || !(isp > 0.0) || !(cfg.Tmax > 0.0))
            {
                note = "degenerate vehicle constants";
                return false;
            }

            // G-FOLD needs a dry/fuel split. The 6-DOF model has no dry mass, so give
            // it a generous notional budget: the seed only has to be a good SHAPE, and
            // a fuel-starved G-FOLD would refuse to solve rather than return one.
            double fuel = 0.5 * m0;

            var p = new GfoldParams
            {
                GravityMag = g,
                DryMass = Math.Max(m0 - fuel, 1.0),
                FuelMass = fuel,
                Isp = isp,
                ThrustMax = cfg.Tmax,
                ThrottleMin = Math.Clamp(cfg.ThrottleFloor, 0.01, 0.95),
                ThrottleMax = 1.0,
                // Deliberately permissive. These are the SEED's constraints, not the
                // flight constraints: the 6-DOF solve re-imposes the real tilt cone,
                // glideslope and speed limits, and a G-FOLD run that refuses to solve
                // because of a tight corridor gives us nothing to start from.
                VMax = Math.Max(4.0 * Speed(x0), 200.0),
                GlideSlopeDeg = 1.0,
                PointingMaxDeg = Math.Clamp(cfg.TiltMaxDeg, 1.0, 89.0),
                R0 = ToGfold(x0[0], x0[1], x0[2]),
                V0 = ToGfold(x0[3], x0[4], x0[5]),
                Rf = ToGfold(xf[0], xf[1], xf[2]),
                Vf = ToGfold(xf[3], xf[4], xf[5]),
            };

            var sw = System.Diagnostics.Stopwatch.StartNew();
            GfoldTrajectory traj;
            double tof;
            int solves;

            if (SearchTimeOfFlight)
            {
                GfoldPlanner.SearchResult r = GfoldPlanner.SearchMinFuel(p, GfoldNodes);
                if (r?.Trajectory == null || r.Trajectory.Nodes < 2)
                {
                    LastGfoldMs = sw.Elapsed.TotalMilliseconds;
                    note = "G-FOLD found no solution";
                    return false;
                }
                traj = r.Trajectory; tof = r.TimeOfFlight; solves = r.Solves;
            }
            else
            {
                // Estimate the burn time rather than searching for it. SCvx
                // re-optimises sigma from here, so the seed only has to be in the
                // right region, and the search is what makes G-FOLD expensive.
                // MIN-ERROR (P3), not min-fuel (P4). P4 needs a landing point that
                // only P3 can supply, so min fuel is inherently two solves - and for a
                // SEED the distinction does not matter: SCvx re-optimises fuel from
                // here, and what it needs is a dynamically sensible SHAPE.
                tof = EstimateTimeOfFlight(x0, xf, g, cfg, m0);
                traj = GfoldPlanner.SolveMinError(p, tof, GfoldNodes);
                solves = 1;
                if (traj == null || traj.Nodes < 2)
                {
                    LastGfoldMs = sw.Elapsed.TotalMilliseconds;
                    note = $"G-FOLD single solve failed at tf {tof:F1} s";
                    return false;
                }
            }
            LastGfoldMs = sw.Elapsed.TotalMilliseconds;

            Resample(traj, nodes, x0, xf, tof, out xSeed, out uSeed);
            sigma = Math.Clamp(tof, cfg.SigmaMin, cfg.SigmaMax);
            note = $"G-FOLD seed: {tof:F1} s, {solves} solve(s), {LastGfoldMs:F0} ms";
            return true;
        }
        catch (Exception e)
        {
            // A seed is an optimisation, never a requirement. Anything unexpected here
            // must degrade to the straight-line seed rather than stop the engage.
            note = "G-FOLD seed failed: " + e.Message;
            return false;
        }
    }

    private static double Speed(double[] x) =>
        Math.Sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);

    /// <summary>model (x,y,z) with z up  ->  gfold (x,y,z) with x up.</summary>
    private static double[] ToGfold(double mx, double my, double mz) => [mz, mx, my];

    /// <summary>
    /// Resample the G-FOLD trajectory onto the 6-DOF node count and turn it into a
    /// full 14-state seed.
    ///
    /// The attitude channel is the interesting part. G-FOLD's commanded acceleration
    /// EXCLUDES gravity, so the thrust direction is simply its direction, and pointing
    /// the body +Z axis along it makes the thrust purely axial — which in turn means
    /// the lateral control channels seed to zero and are consistent with the attitude.
    /// A seed whose attitude and control disagree would start the solver off with a
    /// large defect for no reason.
    /// </summary>
    /// <summary>
    /// Time to descend the remaining height, braking from the current sink rate. The
    /// seed only needs the right order of magnitude.
    /// </summary>
    private static double EstimateTimeOfFlight(double[] x0, double[] xf, double g,
                                               Scvx6DofConfig cfg, double m0)
    {
        double h = Math.Max(x0[2] - xf[2], 1.0);
        double sink = Math.Max(-x0[5], 0.0);
        double twr = cfg.Tmax / Math.Max(m0 * g, 1.0);
        double decel = Math.Max((twr - 1.0) * g, 0.5 * g);
        double tBrake = sink / decel;
        double dBrake = 0.5 * sink * tBrake;
        double rest = Math.Max(h - dBrake, 0.0);
        double vAvg = Math.Max(0.5 * sink, Math.Sqrt(Math.Max(rest, 1.0) * g) * 0.35);
        return Math.Clamp(tBrake + rest / Math.Max(vAvg, 1.0), cfg.SigmaMin, cfg.SigmaMax);
    }

    private static void Resample(GfoldTrajectory t, int nodes, double[] x0, double[] xf,
                                 double tof, out double[] xSeed, out double[] uSeed)
    {
        xSeed = new double[nodes * NX];
        uSeed = new double[nodes * NU];
        int src = t.Nodes;

        for (int k = 0; k < nodes; k++)
        {
            double s = (double)k / (nodes - 1) * (src - 1);
            int i0 = Math.Clamp((int)Math.Floor(s), 0, src - 1);
            int i1 = Math.Min(i0 + 1, src - 1);
            double a = s - i0;

            double[] r0 = t.Position[i0], r1 = t.Position[i1];
            double[] v0 = t.Velocity[i0], v1 = t.Velocity[i1];
            double[] u0 = t.AccelCmd[i0], u1 = t.AccelCmd[i1];

            // gfold (up, a, b) -> model (a, b, up)
            xSeed[k * NX + 0] = Lerp(r0[1], r1[1], a);
            xSeed[k * NX + 1] = Lerp(r0[2], r1[2], a);
            xSeed[k * NX + 2] = Lerp(r0[0], r1[0], a);
            xSeed[k * NX + 3] = Lerp(v0[1], v1[1], a);
            xSeed[k * NX + 4] = Lerp(v0[2], v1[2], a);
            xSeed[k * NX + 5] = Lerp(v0[0], v1[0], a);

            double m = Math.Max(Lerp(t.Mass[i0], t.Mass[i1], a), 1.0);
            xSeed[k * NX + Dynamics6Dof.IM] = m;

            double ux = Lerp(u0[1], u1[1], a);
            double uy = Lerp(u0[2], u1[2], a);
            double uz = Lerp(u0[0], u1[0], a);
            double un = Math.Sqrt(ux * ux + uy * uy + uz * uz);

            // Attitude: body +Z along the commanded acceleration. Fall back to
            // straight up where the command is degenerate.
            if (un > 1e-6)
                PointZAt(ux / un, uy / un, uz / un, xSeed.AsSpan(k * NX + Dynamics6Dof.IQ, 4));
            else
                PointZAt(0.0, 0.0, 1.0, xSeed.AsSpan(k * NX + Dynamics6Dof.IQ, 4));

            // Control: thrust along the body axis, so purely axial.
            uSeed[k * NU + Dynamics6Dof.IT] = un * m;
        }

        // BODY RATES MUST MATCH THE ATTITUDE PROFILE. Leaving them at zero while the
        // seeded attitude rotates is internally INCONSISTENT - the seed asserts the
        // vehicle turns with no angular velocity, a large dynamics defect the solver
        // then spends iterations undoing. A straight-line seed with identity attitude
        // everywhere is at least self-consistent, which is how a "better" seed can
        // lose to it. Finite-difference the quaternion instead: w = 2 * conj(q) * dq/dt.
        double dt = tof / Math.Max(nodes - 1, 1);
        for (int k = 0; k < nodes; k++)
        {
            int a = Math.Max(k - 1, 0), b = Math.Min(k + 1, nodes - 1);
            if (a == b) continue;
            double span = (b - a) * dt;
            Span<double> qa = xSeed.AsSpan(a * NX + Dynamics6Dof.IQ, 4);
            Span<double> qb = xSeed.AsSpan(b * NX + Dynamics6Dof.IQ, 4);
            Span<double> qk = xSeed.AsSpan(k * NX + Dynamics6Dof.IQ, 4);
            double dw = (qb[0] - qa[0]) / span, dx = (qb[1] - qa[1]) / span;
            double dy = (qb[2] - qa[2]) / span, dz = (qb[3] - qa[3]) / span;
            xSeed[k * NX + Dynamics6Dof.IW + 0] = 2.0 * (qk[0] * dx - qk[1] * dw - qk[2] * dz + qk[3] * dy);
            xSeed[k * NX + Dynamics6Dof.IW + 1] = 2.0 * (qk[0] * dy + qk[1] * dz - qk[2] * dw - qk[3] * dx);
            xSeed[k * NX + Dynamics6Dof.IW + 2] = 2.0 * (qk[0] * dz - qk[1] * dy + qk[2] * dx - qk[3] * dw);
        }

        // Node 0 IS the measured state, exactly. The subproblem pins it by equality and
        // the trust region applies there too, so any disagreement between the seed and
        // x0 makes the first subproblem infeasible rather than merely inaccurate.
        Array.Copy(x0, 0, xSeed, 0, NX);
    }

    private static double Lerp(double a, double b, double t) => a * (1.0 - t) + b * t;

    /// <summary>
    /// Shortest-arc quaternion (scalar-first) taking body +Z onto the given unit
    /// vector. Roll about the thrust axis is left at whatever the shortest rotation
    /// gives, because a landing does not care about it.
    /// </summary>
    private static void PointZAt(double x, double y, double z, Span<double> q)
    {
        // Rotation from (0,0,1) to (x,y,z): axis = cross, angle from dot.
        double dot = z;
        if (dot > 1.0 - 1e-12)
        {
            q[0] = 1.0; q[1] = q[2] = q[3] = 0.0;
            return;
        }
        if (dot < -1.0 + 1e-12)
        {
            // Antiparallel: a half turn about any perpendicular axis.
            q[0] = 0.0; q[1] = 1.0; q[2] = q[3] = 0.0;
            return;
        }
        // axis = (0,0,1) x (x,y,z) = (-y, x, 0)
        double ax = -y, ay = x, az = 0.0;
        double s = Math.Sqrt((1.0 + dot) * 2.0);
        q[0] = s * 0.5;
        q[1] = ax / s;
        q[2] = ay / s;
        q[3] = az / s;

        double n = Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (n > 1e-12) for (int i = 0; i < 4; i++) q[i] /= n;
    }
}

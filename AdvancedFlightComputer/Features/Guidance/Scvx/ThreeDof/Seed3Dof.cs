namespace AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// A physically consistent cold seed for the glide-and-burn problem.
///
/// SCvx refines a trajectory rather than finding one, so the seed sets how many iterations a cold solve needs and which local optimum it lands in. 3dof.py's straight line is a fine seed for its toy case, but over a real glide it asserts motion the dynamics never make, and the first iterations spend themselves paying that defect off - or, with hard path limits it breaks by more than a trust radius, never start at all.
///
/// THE GLIDE is integrated with the model itself, engine off, flown at a fixed tilt off the airflow: the attitude the PID glide is actually holding at engage, passed in, rather than alpha 0. That matters beyond the defect - if the airframe's lift grows faster than the angle near zero, a seed flown at exactly alpha 0 can show the linearisation no lift worth having.
///
/// IGNITION is the latest point along that glide from which a retrograde burn at a fraction of full thrust still stops the sink above the target, found by bisection with the burn SIMULATED - drag included. A closed-form stopping distance is no use here: from 30 km at Mach 3 the air does about half the braking, and the constant-deceleration estimate lights the engine at the top of the atmosphere. The fraction leaves the optimiser headroom; SCvx moves the ignition from there.
///
/// THE BURN is that simulated burn, with a correction ramped in linearly so it ends exactly on the target. Its dynamics are the model's except for the ramp, which is as small as the glide's miss is.
///
/// Finally the attitude sequence is slewed forward at the rate limit, since the ignition jump from glide attitude to burn attitude is otherwise instant.
/// </summary>
public static class Seed3Dof
{
    private const int NX = PointMass3Dof.NX;
    private const int NU = PointMass3Dof.NU;

    public sealed record Options
    {
        /// <summary>Tilt off the airflow the glide is seeded at, radians, and the direction of that tilt (any vector; only its part across the airflow is used). Zero tilt seeds alpha 0.</summary>
        public double GlideTilt { get; init; }
        public double[] GlideTiltDirection { get; init; } = [0, 0, 1];

        /// <summary>Fraction of full thrust the ignition search's simulated burn uses.</summary>
        public double IgnitionThrustFraction { get; init; } = 0.7;

        /// <summary>Integration step, s.</summary>
        public double Step { get; init; } = 0.1;

        /// <summary>Longest glide searched, s.</summary>
        public double MaxGlide { get; init; } = 300.0;

        /// <summary>Up, in the model's frame: the seed reads "height" along it.</summary>
        public double[] Up { get; init; } = [0, 0, 1];
    }

    /// <summary>Build the seed. False only when the inputs are unusable.</summary>
    public static bool TryBuild(KsaPointMassModel model, Scvx3DofConfig cfg, ReadOnlySpan<double> x0,
                                ReadOnlySpan<double> xf, Options opt,
                                out double[] xSeed, out double[] uSeed, out double sigmaGlide, out double sigmaBurn)
    {
        int n = cfg.Nodes, k = cfg.GlideIntervals;
        xSeed = new double[n * NX];
        uSeed = new double[n * NU];
        sigmaGlide = sigmaBurn = 0.0;
        if (!(x0[PointMass3Dof.IM] > 0.0) || !(model.VacuumThrust > 0.0) || !(model.MassFlow > 0.0))
            return false;

        double[] up = Normalise(opt.Up);
        double[] target = xf.ToArray();
        double targetHeight = Dot3(target, 0, up);
        double sinkF = -Dot3(target, 3, up);

        // ---- the glide, engine off, to the ground or the horizon.
        var path = new List<double[]> { x0.ToArray() };
        if (k > 0)
        {
            double[] x = x0.ToArray();
            for (double t = 0; t < opt.MaxGlide; t += opt.Step)
            {
                x = Rk4(model, x, opt.Step, s => (GlideAttitude(s, opt), 0.0));
                path.Add(x);
                if (Dot3(x, 0, up) < targetHeight) break;
            }
        }

        // ---- ignition: the latest glide point a simulated burn still stops from.
        int ign = 0;
        if (k > 0)
        {
            ign = LatestIgnition(path, opt, i => SimulateBurn(model, cfg, path[i], opt, up, targetHeight, sinkF, null).StopHeight - targetHeight);
            sigmaGlide = Math.Clamp(ign * opt.Step, cfg.GlideSigmaMin, cfg.GlideSigmaMax);
        }

        Span<double> sx = stackalloc double[NX];
        for (int node = 0; node <= k; node++)
        {
            double tn = k > 0 ? sigmaGlide * node / k : 0.0;
            SamplePath(path, opt.Step, tn, sx);
            sx.CopyTo(xSeed.AsSpan(node * NX, NX));
            double[] b = GlideAttitude(sx.ToArray(), opt);
            Array.Copy(b, 0, uSeed, node * NU, 3);
        }
        Array.Copy(x0.ToArray(), 0, xSeed, 0, NX);

        // ---- the burn: simulated from the ignition state, ramped onto the target.
        double[] xi = xSeed.AsSpan(k * NX, NX).ToArray();
        var burn = new List<(double[] X, double[] B, double Throttle)>();
        Burn result = SimulateBurn(model, cfg, xi, opt, up, targetHeight, sinkF, burn);
        sigmaBurn = Math.Clamp(Math.Max(result.Time, opt.Step), cfg.BurnSigmaMin, cfg.BurnSigmaMax);

        int burnNodes = n - k;
        double[] end = burn[^1].X;
        for (int j = 0; j < burnNodes; j++)
        {
            int node = k + j;
            double s = (double)j / (burnNodes - 1);
            double t = s * sigmaBurn;
            int i = Math.Min((int)Math.Round(t / opt.Step), burn.Count - 1);
            (double[] bx, double[] bb, double th) = burn[i];
            for (int c = 0; c < NX; c++)
                xSeed[node * NX + c] = bx[c];
            for (int c = 0; c < 6; c++)
                xSeed[node * NX + c] += s * (target[c] - end[c]);
            Array.Copy(bb, 0, uSeed, node * NU, 3);
            uSeed[node * NU + PointMass3Dof.ITH] = th;
        }
        Array.Copy(xi, 0, xSeed, k * NX, NX);
        for (int node = 0; node < k; node++)
            uSeed[node * NU + PointMass3Dof.ITH] = 0.0;
        if (cfg.TerminalAttitude != null)
            Array.Copy(cfg.TerminalAttitude, 0, uSeed, (n - 1) * NU, 3);

        SlewLimit(cfg, uSeed, n, k, sigmaGlide, sigmaBurn);
        return true;
    }

    /// <summary>
    /// The latest path index from which the simulated burn stops above the target (margin >= 0).
    ///
    /// NOT a bisection from "light now". Lighting too early fails as surely as lighting too late - from the top of the atmosphere the propellant runs out before the sink is gone - so the ignitions that work are a window in the middle of the glide, not a prefix of it. The window's late edge is what is wanted, and it is near the ground, so the scan walks back from the end a second at a time, where each simulated burn is also short, and bisects inside the second it finds. With no window at all it takes the ignition that comes closest.
    /// </summary>
    private static int LatestIgnition(List<double[]> path, Options opt, Func<int, double> margin)
    {
        int last = path.Count - 1;
        int stride = Math.Max(1, (int)Math.Round(1.0 / opt.Step));
        int best = 0;
        double bestMargin = double.NegativeInfinity;
        int failed = -1;
        for (int i = last; i >= 0; i -= stride)
        {
            double m = margin(i);
            if (m >= 0.0)
            {
                // Bisect between this success and the failure a stride later.
                int lo = i, hi = failed >= 0 ? failed : i;
                while (hi - lo > 1)
                {
                    int mid = (lo + hi) / 2;
                    if (margin(mid) >= 0.0) lo = mid; else hi = mid;
                }
                return lo;
            }
            failed = i;
            if (m > bestMargin)
            {
                bestMargin = m;
                best = i;
            }
        }
        return best;
    }

    private readonly record struct Burn(double StopHeight, double Time);

    /// <summary>
    /// A retrograde burn at the search's thrust fraction from <paramref name="x"/>, until the sink is down to the terminal sink, the vehicle reaches the target height, the propellant runs out or the burn outlasts the longest allowed. Returns the height it stopped at.
    /// </summary>
    private static Burn SimulateBurn(KsaPointMassModel model, Scvx3DofConfig cfg, double[] x, Options opt, double[] up,
                                     double targetHeight, double sinkF,
                                     List<(double[] X, double[] B, double Throttle)>? record)
    {
        double throttle = Math.Clamp(opt.IgnitionThrustFraction, cfg.ThrottleFloor, cfg.ThrottleCeiling);
        double dry = cfg.StateMin[PointMass3Dof.IM];
        double[] s = (double[])x.Clone();
        double t = 0.0;
        record?.Add(((double[])s.Clone(), Retrograde(s, up), throttle));
        while (t < cfg.BurnSigmaMax)
        {
            double sink = -Dot3(s, 3, up);
            double height = Dot3(s, 0, up);
            if (sink <= sinkF || height < targetHeight || (double.IsFinite(dry) && s[PointMass3Dof.IM] <= dry))
                break;
            s = Rk4(model, s, opt.Step, y => (Retrograde(y, up), throttle));
            t += opt.Step;
            record?.Add(((double[])s.Clone(), Retrograde(s, up), throttle));
        }
        double stopHeight = Dot3(s, 0, up);
        // Out of propellant or out of time while still falling: it did not stop, wherever it is.
        if (-Dot3(s, 3, up) > sinkF + 1.0)
            stopHeight = Math.Min(stopHeight, targetHeight - 1.0);
        return new Burn(stopHeight, t);
    }

    private static double[] Retrograde(double[] x, double[] up)
    {
        double vx = x[3], vy = x[4], vz = x[5];
        double sp = Math.Sqrt(vx * vx + vy * vy + vz * vz);
        return sp > 1.0 ? [-vx / sp, -vy / sp, -vz / sp] : [up[0], up[1], up[2]];
    }

    private static double[] GlideAttitude(double[] x, Options opt)
    {
        double vx = x[3], vy = x[4], vz = x[5];
        double sp = Math.Sqrt(vx * vx + vy * vy + vz * vz);
        if (!(sp > 1e-6)) return [0, 0, 1];
        double[] h = [vx / sp, vy / sp, vz / sp];
        double[] b = [-h[0], -h[1], -h[2]];
        if (opt.GlideTilt == 0.0) return b;
        double[] d = opt.GlideTiltDirection;
        double along = d[0] * h[0] + d[1] * h[1] + d[2] * h[2];
        double[] across = [d[0] - along * h[0], d[1] - along * h[1], d[2] - along * h[2]];
        double len = Math.Sqrt(across[0] * across[0] + across[1] * across[1] + across[2] * across[2]);
        if (!(len > 1e-9)) return b;
        double c = Math.Cos(opt.GlideTilt), sn = Math.Sin(opt.GlideTilt);
        return [c * b[0] + sn * across[0] / len, c * b[1] + sn * across[1] / len, c * b[2] + sn * across[2] / len];
    }

    /// <summary>
    /// Turn each node's body axis towards the next by no more than the rate limit allows across the interval between them, working forward from node 0. The terminal attitude is left where the config pins it.
    /// </summary>
    private static void SlewLimit(Scvx3DofConfig cfg, double[] u, int n, int k, double sigG, double sigB)
    {
        if (!(cfg.AttitudeRateMax > 0.0)) return;
        int last = cfg.TerminalAttitude != null ? n - 1 : n;
        for (int node = 1; node < last; node++)
        {
            double dt = Scvx3DofSolver.NodeTime(node, k, n, sigG, sigB) - Scvx3DofSolver.NodeTime(node - 1, k, n, sigG, sigB);
            double maxAngle = cfg.AttitudeRateMax * dt;
            Span<double> a = u.AsSpan((node - 1) * NU, 3);
            Span<double> b = u.AsSpan(node * NU, 3);
            double cos = Math.Clamp(a[0] * b[0] + a[1] * b[1] + a[2] * b[2], -1.0, 1.0);
            double angle = Math.Acos(cos);
            if (angle <= maxAngle || angle < 1e-9) continue;
            // Slerp from a towards b by maxAngle.
            double sinA = Math.Sin(angle);
            double wa = Math.Sin(angle - maxAngle) / sinA, wb = Math.Sin(maxAngle) / sinA;
            for (int j = 0; j < 3; j++) b[j] = wa * a[j] + wb * b[j];
        }
    }

    private static void SamplePath(List<double[]> path, double step, double t, Span<double> x)
    {
        double s = t / step;
        int i = Math.Clamp((int)s, 0, path.Count - 1);
        int j = Math.Min(i + 1, path.Count - 1);
        double f = Math.Clamp(s - i, 0.0, 1.0);
        for (int c = 0; c < NX; c++)
            x[c] = (1 - f) * path[i][c] + f * path[j][c];
    }

    private static double[] Rk4(KsaPointMassModel model, double[] x, double h, Func<double[], (double[] B, double Throttle)> control)
    {
        double[] k1 = new double[NX], k2 = new double[NX], k3 = new double[NX], k4 = new double[NX], tmp = new double[NX];
        Deriv(model, x, control, k1);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k1[i];
        Deriv(model, tmp, control, k2);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k2[i];
        Deriv(model, tmp, control, k3);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + h * k3[i];
        Deriv(model, tmp, control, k4);
        var next = new double[NX];
        for (int i = 0; i < NX; i++) next[i] = x[i] + h / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
        return next;
    }

    private static void Deriv(KsaPointMassModel model, double[] x, Func<double[], (double[] B, double Throttle)> control, double[] f)
    {
        (double[] b, double th) = control(x);
        PointMass3Dof.Eval(model, x, [b[0], b[1], b[2], th], f);
    }

    private static double Dot3(double[] v, int o, double[] up) => v[o] * up[0] + v[o + 1] * up[1] + v[o + 2] * up[2];

    private static double[] Normalise(double[] v)
    {
        double len = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        return [v[0] / len, v[1] / len, v[2] / len];
    }
}

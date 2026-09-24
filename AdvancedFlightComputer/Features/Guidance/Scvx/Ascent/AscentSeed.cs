namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// The seed: a gravity turn FLOWN through the same dynamics, so its collocation defects are physically small from the first iteration, with the kick angle chosen by search. A port of launch3dof.py's build_seed, seed_cost and kick search.
///
///   1. Rise vertically at full thrust.
///   2. At an air-relative speed of PitchOverSpeed, rotate v_rel by the kick angle toward downrange, keeping its speed. The crossing is found by bisection inside the RK4 sub-step, so the seed does not depend on the mesh.
///   3. Follow v_rel from then on (a zero-alpha gravity turn), with the thrust PROJECTED INTO THE TARGET PLANE. Without the projection the thrust follows the out-of-plane part of v_rel that the body's rotation gives it, and the plane error feeds back until it is more than one trust region can absorb.
///   4. Lower stages burn to depletion at full throttle; the last burns FinalBurnFraction of its load.
///
/// The kick angle minimises the insertion ALTITUDE and PLANE miss plus the merit's own path violation - deliberately not the full insertion miss: speed and flight-path angle are the free burn times' job, and scoring them favours a seed that arrives at the right speed three hundred kilometres too high (see the script's seed_cost for the measurements).
/// </summary>
internal static class AscentSeed
{
    private const double Deg = Math.PI / 180.0;
    private static readonly double InvPhi = (Math.Sqrt(5.0) - 1.0) / 2.0;

    /// <param name="LeastMaxQ">The lowest peak dynamic pressure any scanned seed flew, Pa: near the least this vehicle can manage at full throttle, since the steepest seed is nearly vertical.</param>
    public sealed record Result(bool Ok, double[] X, double[] U, double[] Sigma, double KickDeg,
                                double Cost, int Evaluations, string Note, double LeastMaxQ);

    /// <summary>Full-throttle burn times, with the last stage at its seed fraction.</summary>
    public static double[] SeedSigma(AscentCase c)
    {
        var sig = new double[c.S];
        for (int i = 0; i < c.S; i++)
            sig[i] = c.PropC[i] / c.Dyn.MassFlowC(i);
        sig[c.S - 1] *= c.Settings.FinalBurnFraction;
        return sig;
    }

    public static Result Search(AscentCase c)
    {
        AscentSettings st = c.Settings;
        double[] sig = SeedSigma(c);
        int evals = 0;

        // THE PATH TERM COUNTS ONLY WHAT THE BEST-BEHAVED SEED DOES NOT ALREADY HAVE. The script scores the merit's own path violation so that a kick steep enough to graze the atmosphere on the way in is rejected rather than merely disliked. On a vehicle whose every gravity turn exceeds a limit - a high-thrust stack cannot climb through the thick air under 35 kPa at full throttle whatever it does - that term instead dominates everything and drives the kick toward straight up, which lofts the seed a thousand kilometres, and below a fraction of a degree has it fall behind the rotating air and fly retrograde. So the violation is measured from the least any scanned seed manages, and within a band above that (SeedPathBand of it, plus as much again) seeds tie on it and are ranked on the insertion miss alone; a seed that grazes the atmosphere on the way in is still far outside the band. The script's own seed keeps its own cost: floor and band are both zero there.
        double pathFloor = double.PositiveInfinity, leastMaxQ = double.PositiveInfinity;
        double Band() => st.SeedPathBand <= 0.0 || !double.IsFinite(pathFloor) ? 0.0
            : pathFloor + st.SeedPathBand * (1.0 + pathFloor);
        double Score(Flight f) => !f.Ok ? st.BadSeedCost : f.Miss + Math.Max(0.0, f.Path - Band());
        double Cost(double kick) { evals++; return Score(Fly(c, sig, kick)); }

        double lo = st.KickMinDeg * Deg, hi = st.KickMaxDeg * Deg;
        int nScan = Math.Max(3, st.KickScan);
        var scanK = new double[nScan];
        var scanF = new Flight[nScan];
        int best = 0;
        string note = "";

        // COARSE SCAN BEFORE THE GOLDEN SECTION: past a few degrees the vehicle pitches over too fast, re-enters, and the cost jumps by orders of magnitude, so a bare golden section could bracket that cliff and converge inside it.
        for (int slide = 0; ; slide++)
        {
            for (int i = 0; i < nScan; i++)
            {
                scanK[i] = st.KickScanLog
                    ? lo * Math.Pow(hi / lo, (double)i / (nScan - 1))
                    : lo + (hi - lo) * i / (nScan - 1);
                scanF[i] = Fly(c, sig, scanK[i]);
                evals++;
                if (scanF[i].Ok)
                {
                    pathFloor = Math.Min(pathFloor, scanF[i].Path);
                    leastMaxQ = Math.Min(leastMaxQ, scanF[i].MaxQ);
                }
            }
            best = 0;
            for (int i = 1; i < nScan; i++)
                if (Score(scanF[i]) < Score(scanF[best])) best = i;

            bool atEdge = best == 0 || best == nScan - 1;
            if (!atEdge || slide >= st.KickWindowSlides)
            {
                if (atEdge)
                    note = $"kick scan bottomed out at {scanK[best] / Deg:F2} deg";
                break;
            }
            // Recentre the window on the edge the scan bottomed out at - not in the script, which only warns; it never runs when the best kick is inside the window.
            double centre = scanK[best];
            if (st.KickScanLog)
            {
                double ratio = Math.Sqrt(hi / lo);
                lo = Math.Max(0.01 * Deg, centre / ratio);
                hi = Math.Min(60.0 * Deg, centre * ratio);
            }
            else
            {
                double width = hi - lo;
                lo = Math.Max(0.05 * Deg, centre - 0.5 * width);
                hi = Math.Min(30.0 * Deg, lo + width);
            }
        }

        double a = scanK[Math.Max(best - 1, 0)], b = scanK[Math.Min(best + 1, nScan - 1)];
        double cc = b - InvPhi * (b - a), fc = Cost(cc);
        double d = a + InvPhi * (b - a), fd = Cost(d);
        double tol = st.KickToleranceDeg * Deg;
        while (b - a > tol)
        {
            if (fc < fd)
            {
                b = d; d = cc; fd = fc;
                cc = b - InvPhi * (b - a); fc = Cost(cc);
            }
            else
            {
                a = cc; cc = d; fc = fd;
                d = a + InvPhi * (b - a); fd = Cost(d);
            }
        }
        double kickAngle = 0.5 * (a + b);
        // A golden section bracketed next to a region the seed cannot fly (re-entry, or retrograde) can end just inside it; the best scanned kick is flyable by construction, so it is the fallback.
        if (Score(Fly(c, sig, kickAngle)) >= st.BadSeedCost && Score(scanF[best]) < st.BadSeedCost)
            kickAngle = scanK[best];
        bool ok = Build(c, sig, kickAngle, out double[] x, out double[] u, out double[] sigUsed);
        double cost = ok ? Score(Fly(c, sig, kickAngle)) : st.BadSeedCost;
        if (ok && cost >= st.BadSeedCost)
            ok = false;
        sig = sigUsed;
        if (ok && pathFloor > 0.0 && double.IsFinite(pathFloor))
            note = (note.Length > 0 ? note + "; " : "") + $"no gravity turn meets the path limits (least violation {pathFloor:G3})";
        return new Result(ok, x, u, sig, kickAngle / Deg, cost, evals, note, leastMaxQ);
    }

    private readonly record struct Flight(bool Ok, double Miss, double Path, double MaxQ = double.NaN);

    /// <summary>
    /// One seed flight, scored the script's way: insertion altitude and plane miss, and the merit's own path violation. Not flyable if it left the model's validity, or ended RETROGRADE - the subproblem's prograde constraint is a hard one, and a reference that violates it leaves the first subproblem with no feasible point at all.
    /// </summary>
    private static Flight Fly(AscentCase c, double[] sig, double kick)
    {
        if (!Build(c, sig, kick, out double[] x, out double[] u, out _))
            return new Flight(false, 0.0, 0.0);

        Span<double> res = stackalloc double[5];
        c.TerminalResidual(x, res);
        double miss = res[0] * res[0] + res[3] * res[3] + res[4] * res[4];

        int o = (c.N - 1) * AscentCase.NX;
        double hx = x[o + 1] * x[o + 5] - x[o + 2] * x[o + 4];
        double hy = x[o + 2] * x[o + 3] - x[o] * x[o + 5];
        double hz = x[o] * x[o + 4] - x[o + 1] * x[o + 3];
        if (!(hx * c.Nhat[0] + hy * c.Nhat[1] + hz * c.Nhat[2] > 0.0))
            return new Flight(false, miss, 0.0);

        double path = 0.0, maxQ = 0.0;
        for (int n = 0; n < c.N; n++)
        {
            ReadOnlySpan<double> xn = x.AsSpan(n * AscentCase.NX, AscentCase.NX);
            ReadOnlySpan<double> un = u.AsSpan(n * AscentCase.NU, AscentCase.NU);
            double qa = c.Dyn.QAlpha(xn, un, c.NodeStage[n]);
            double q = c.Dyn.QValue(xn);
            maxQ = Math.Max(maxQ, q);
            path += Math.Max(0.0, qa / c.QAlphaMax - 1.0) + Math.Max(0.0, q / c.QMax - 1.0);
        }
        bool finite = double.IsFinite(miss) && double.IsFinite(path);
        return new Flight(finite, miss, path, maxQ);
    }

    /// <summary>
    /// Forward-integrate the seed for one kick angle. False if it left the model's validity: re-entered, or blew up.
    ///
    /// <paramref name="sigUsed"/> is the burn times actually flown. With <see cref="AscentSettings.SeedEnergyCutoff"/> the last stage's is not the fixed fraction the script uses but however long the last stage takes to bring the orbital energy up to the target's, found by flying that stage once more finely before its nodes are laid down. The script's own analysis names this as the next step: its seed burns 360 s of S-IVB when the optimum is 216, and paying that back is half its iterations. On a vehicle whose last stage carries far more than the orbit needs, the fixed fraction lofts the seed a thousand kilometres, which no trust region closes.
    /// </summary>
    public static bool Build(AscentCase c, double[] sig, double kick, out double[] xb, out double[] ub,
                             out double[] sigUsed)
    {
        const int nx = AscentCase.NX, nu = AscentCase.NU;
        AscentSettings st = c.Settings;
        xb = new double[c.N * nx];
        ub = new double[c.N * nu];
        sigUsed = (double[])sig.Clone();

        Span<double> x = stackalloc double[nx];
        c.X0.CopyTo(xb, 0);

        bool kicked = AirSpeed(c, xb) > st.PitchOverSpeed;
        if (kicked)
            Kick(c, xb.AsSpan(0, nx), kick);

        double reentry = 1.0 - 50e3 / c.Dyn.LU;
        int last = c.S - 1;
        for (int n = 0; n < c.N; n++)
        {
            int s = c.NodeStage[n];
            if (st.SeedEnergyCutoff && n == c.Starts[last])
            {
                xb.AsSpan(n * nx, nx).CopyTo(x);
                sigUsed[last] = EnergyCutoff(c, x, kicked, kick, reentry);
            }

            Direction(c, xb.AsSpan(n * nx, nx), kicked, ub.AsSpan(n * nu, nu));
            if (n == c.N - 1)
                break;

            if (c.NodeStage[n + 1] != s)
            {
                // Staging: jettison the dry structure; r and v carry straight through.
                Array.Copy(xb, n * nx, xb, (n + 1) * nx, nx);
                xb[(n + 1) * nx + 6] -= c.DryC[s];
                continue;
            }

            double h = sigUsed[s] * c.Dtau[n] / st.SeedSubsteps;
            xb.AsSpan(n * nx, nx).CopyTo(x);
            for (int sub = 0; sub < st.SeedSubsteps; sub++)
                if (!Substep(c, x, h, s, ref kicked, kick, reentry))
                    return false;
            x.CopyTo(xb.AsSpan((n + 1) * nx, nx));
        }

        foreach (double v in xb)
            if (!double.IsFinite(v)) return false;
        return true;
    }

    /// <summary>
    /// How long the last stage burns to bring the specific orbital energy up to the target orbit's, canonical time, by flying it with the seed's law at a fine step. Clamped to the stage's burn-time bounds; its full load if the energy is never reached.
    /// </summary>
    private static double EnergyCutoff(AscentCase c, ReadOnlySpan<double> start, bool kicked, double kick, double reentry)
    {
        const int steps = 1000;
        int last = c.S - 1;
        double full = c.PropC[last] / c.Dyn.MassFlowC(last);
        double target = 0.5 * c.VTarget * c.VTarget - 1.0 / c.RTarget;
        Span<double> x = stackalloc double[AscentCase.NX];
        start.CopyTo(x);
        double h = full / steps;
        double e0 = Energy(x);
        if (e0 >= target)
            return c.SigMin[last];
        for (int i = 1; i <= steps; i++)
        {
            if (!Substep(c, x, h, last, ref kicked, kick, reentry))
                return full;
            double e1 = Energy(x);
            if (e1 >= target)
            {
                double f = (target - e0) / (e1 - e0);
                return Math.Clamp((i - 1 + f) * h, c.SigMin[last], full);
            }
            e0 = e1;
        }
        return full;
    }

    private static double Energy(ReadOnlySpan<double> x)
        => 0.5 * (x[3] * x[3] + x[4] * x[4] + x[5] * x[5]) - 1.0 / AscentCase.Norm3(x);

    /// <summary>
    /// One RK4 sub-step of the seed under its thrust law, pitching over inside the step if the trigger speed is crossed in it: the crossing is bisected, the kick applied there, and the rest of the step flown under the new law. False if the state left the model's validity.
    /// </summary>
    private static bool Substep(AscentCase c, Span<double> x, double h, int s, ref bool kicked, double kick, double reentry)
    {
        const int nx = AscentCase.NX;
        AscentSettings st = c.Settings;
        Span<double> xt = stackalloc double[nx];
        Span<double> tmp = stackalloc double[nx];
        if (kicked)
        {
            Rk4(c, x, h, s, true, xt);
            xt.CopyTo(x);
        }
        else
        {
            Rk4(c, x, h, s, false, xt);
            if (AirSpeed(c, xt) <= st.PitchOverSpeed)
            {
                xt.CopyTo(x);
            }
            else
            {
                double lo = 0.0, hiT = h;
                for (int it = 0; it < 60; it++)
                {
                    double mid = 0.5 * (lo + hiT);
                    Rk4(c, x, mid, s, false, tmp);
                    if (AirSpeed(c, tmp) > st.PitchOverSpeed) hiT = mid;
                    else lo = mid;
                }
                Rk4(c, x, hiT, s, false, tmp);
                Kick(c, tmp, kick);
                kicked = true;
                Rk4(c, tmp, h - hiT, s, true, x);
            }
        }

        bool finite = true;
        for (int i = 0; i < nx; i++) finite &= double.IsFinite(x[i]);
        return finite && AscentCase.Norm3(x) >= reentry;
    }

    /// <summary>Radial until the pitch-over, then along v_rel; projected into the target plane either way.</summary>
    private static void Direction(AscentCase c, ReadOnlySpan<double> x, bool kicked, Span<double> d)
    {
        double w = c.Dyn.OmegaC;
        if (kicked)
        {
            d[0] = x[3] + w * x[1];
            d[1] = x[4] - w * x[0];
            d[2] = x[5];
        }
        else
        {
            d[0] = x[0]; d[1] = x[1]; d[2] = x[2];
        }
        double n0 = AscentCase.Norm3(d);
        for (int i = 0; i < 3; i++) d[i] /= n0;
        double along = d[0] * c.Nhat[0] + d[1] * c.Nhat[1] + d[2] * c.Nhat[2];
        for (int i = 0; i < 3; i++) d[i] -= along * c.Nhat[i];
        double n1 = AscentCase.Norm3(d);
        for (int i = 0; i < 3; i++) d[i] /= n1;
    }

    private static void Rk4(AscentCase c, ReadOnlySpan<double> x, double h, int stage, bool kicked,
                            Span<double> result)
    {
        const int nx = AscentCase.NX;
        Span<double> k1 = stackalloc double[nx];
        Span<double> k2 = stackalloc double[nx];
        Span<double> k3 = stackalloc double[nx];
        Span<double> k4 = stackalloc double[nx];
        Span<double> y = stackalloc double[nx];

        Deriv(c, x, stage, kicked, k1);
        for (int i = 0; i < nx; i++) y[i] = x[i] + 0.5 * h * k1[i];
        Deriv(c, y, stage, kicked, k2);
        for (int i = 0; i < nx; i++) y[i] = x[i] + 0.5 * h * k2[i];
        Deriv(c, y, stage, kicked, k3);
        for (int i = 0; i < nx; i++) y[i] = x[i] + h * k3[i];
        Deriv(c, y, stage, kicked, k4);
        for (int i = 0; i < nx; i++)
            result[i] = x[i] + (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    /// <summary>The seed's thrust law evaluated at the state it is stepped from, full throttle.</summary>
    private static void Deriv(AscentCase c, ReadOnlySpan<double> x, int stage, bool kicked, Span<double> dx)
    {
        Span<double> u = stackalloc double[3];
        Direction(c, x, kicked, u);
        c.Dyn.Eval(x, u, stage, dx);
    }

    private static double AirSpeed(AscentCase c, ReadOnlySpan<double> x)
    {
        double w = c.Dyn.OmegaC;
        double ax = x[3] + w * x[1], ay = x[4] - w * x[0], az = x[5];
        return Math.Sqrt(ax * ax + ay * ay + az * az) * c.Dyn.VU;
    }

    /// <summary>Rotate v_rel by the kick angle toward downrange, keeping its speed.</summary>
    private static void Kick(AscentCase c, Span<double> x, double angle)
    {
        double w = c.Dyn.OmegaC;
        double ax = x[3] + w * x[1], ay = x[4] - w * x[0], az = x[5];
        double vrn = Math.Sqrt(ax * ax + ay * ay + az * az);
        double nx = Math.Cos(angle) * ax / vrn + Math.Sin(angle) * c.E2[0];
        double ny = Math.Cos(angle) * ay / vrn + Math.Sin(angle) * c.E2[1];
        double nz = Math.Cos(angle) * az / vrn + Math.Sin(angle) * c.E2[2];
        double nn = Math.Sqrt(nx * nx + ny * ny + nz * nz);
        // Back to inertial: v = v_rel + omega x r.
        x[3] = vrn * nx / nn - w * x[1];
        x[4] = vrn * ny / nn + w * x[0];
        x[5] = vrn * nz / nn;
    }
}

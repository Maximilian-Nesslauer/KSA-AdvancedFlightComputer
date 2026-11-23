using AdvancedFlightComputer.Guidance.Scvx.Bellyflop;

/// <summary>
/// --bellyflop: the belly-first terminal descent planner (docs/guidance/bellyflop-design.md) on a synthetic belly flop: a flat plate whose normal force goes as the square of the airflow's component on it (so L/D = tan(tilt), 0.36 at 20 deg), 70 t on 64 m^2, an Earth-like rotating body and an exponential atmosphere. From Mach 3, handed over at 50 deg belly angle of attack as the entry flies: a cold solve onto sites short of, beyond and beside the belly-first fall, then a closed loop flying the plan in air 10 % denser and drag 5 % weaker than planned, re-planning every 2 s warm, to the ground. Exit 1 on a plan that does not converge or a closed loop that misses by more than 500 m.
/// </summary>
internal static class BellyflopCheck
{
    const double Deg = Math.PI / 180.0;
    const double Mu = 3.986004418e14, R = 6_371_000.0, Omega = 7.2921159e-5, Lat = 28.6 * Deg;
    const double CN0 = 6.0;

    static BellyflopModel Model(double densityScale = 1.0, double forceScale = 1.0) => new()
    {
        Mu = Mu, BodyRadius = R,
        OmegaX = 0.0, OmegaY = Omega * Math.Cos(Lat), OmegaZ = Omega * Math.Sin(Lat),
        Density = h => 1.225 * Math.Exp(-Math.Max(h, 0.0) / 7200.0),
        SpeedOfSound = 340.0, Mass = 70_000.0, ReferenceArea = 64.0,
        Coefficients = FlatPlate,
        BearingX = 1.0, BearingY = 0.0,
        DensityScale = densityScale, ForceScale = forceScale,
    };

    /// <summary>A flat belly: the force along the belly normal, against the travel, as the square of the airflow's component on it; a little skin friction along the airflow.</summary>
    static void FlatPlate(double mach, double alpha, double beta, out double cn, out double cs, out double cb)
    {
        double ta = Math.Tan(alpha), tb = Math.Tan(beta);
        double inv = 1.0 / Math.Sqrt(ta * ta + tb * tb + 1);
        double un = ta * inv, us = tb * inv, ub = inv;
        double normal = CN0 * ub * ub;
        const double friction = 0.05;
        cn = -friction * un;
        cs = -friction * us;
        cb = -normal - friction * ub;
    }

    /// <summary>The hand-over: Mach <paramref name="mach"/> at <paramref name="altitude"/>, flying towards +x on the entry's path (15 deg down at Mach 3, 3 deg at Mach 4, as flown); the site's offset from the craft is applied by the caller.</summary>
    static double[] HandOver(double downrangeToSite, double crossToSite, double mach, double altitude)
    {
        double v = mach * 340.0, gamma = (mach >= 4.5 ? 0.0 : mach >= 3.5 ? -3.0 : -15.0) * Deg;
        return [-downrangeToSite, -crossToSite, altitude, v * Math.Cos(gamma), 0.0, v * Math.Sin(gamma)];
    }

    public static int Run(bool verbose)
    {
        int bad = 0;
        // The first flight handed over at Mach 3, 30 km; the default is now Mach 4, 32 km, for the time and reach a late hand-over lacked.
        foreach (var (mach, altitude) in new[] { (3.0, 30_000.0), (4.0, 32_000.0) })
        {
            Console.WriteLine(FormattableString.Invariant($" hand-over at Mach {mach:F0}, {altitude / 1000:F0} km:"));
            bad += RunFrom(mach, altitude, glide: false);
        }
        // The take-over from the entry itself: at Mach 5, level at 38 km, at the entry's 50 deg (40 from nose-first), with the site as far ahead as the entry would still glide (the second flight was 147 km out here). Bellyflop may glide on at up to BELLYFLOP PITCH UP.
        Console.WriteLine(" take-over at Mach 5, 38 km, from the entry's attitude:");
        bad += RunFrom(5.0, 38_000.0, glide: true);
        Console.WriteLine(" as flown 2026-11-23 (taken over at Mach 4.85, 10 km across, the entry banked 63 deg), force 15 % weak:");
        bad += FlightLike();
        Console.WriteLine(bad == 0 ? "PASS bellyflop" : $"FAIL bellyflop ({bad})");
        return bad == 0 ? 0 : 1;
    }

    /// <summary>
    /// The 2026-11-23 take-over: Mach 5 at 38 km, level, at the entry's 50 deg of pitch and its 63 deg of bank (bellyflop's yaw), the site beyond the flat fall and 10 km to the side, in air whose force is 15 % short of the model's (the 0.8 measured transonic). Closed loop, re-planned every 2 s warm; the horizontal distance to the site as the craft passes 12, 8 and 4 km says whether the miss is taken out while there is speed to, and the end within 500 m passes.
    /// </summary>
    static int FlightLike()
    {
        int bad = 0;
        var cfg = new BellyflopConfig();
        var model = Model();
        double a0 = 50 * Deg, y0 = 63 * Deg;
        double fall = new BellyflopPlanner().Solve(model, new BellyflopConfig { MissWeight = 0.0, ColdIterations = 1 }, HandOver(0, 0, 5.0, 38_000.0), 0.0, a0, 0, 0, null) is { Plan: { } pr } ? pr.X[pr.Nodes - 1, 0] : 70_000.0;
        foreach (var (name, down, cross, yaw) in new[] { ("18 km beyond the fall, 10 km beside, banked 63", 18_000.0, 10_000.0, y0), ("18 km beyond, 10 km beside, the other way", 18_000.0, -10_000.0, y0), ("flat fall, 10 km beside, wings level", 0.0, 10_000.0, 0.0) })
        {
            var truth = Model(1.0, 0.85);
            double[] x = HandOver(fall + down, cross, 5.0, 38_000.0);
            var planner = new BellyflopPlanner();
            BellyflopPlan? plan = null;
            double t = 0, nextPlan = 0, a = a0, b = 0, y = yaw;
            int solves = 0, refused = 0;
            var at = new Dictionary<int, double>();
            double yaw30 = double.NaN, cross30 = double.NaN;
            const double dt = 0.05;
            Span<double> k1 = stackalloc double[6], k2 = stackalloc double[6], k3 = stackalloc double[6], k4 = stackalloc double[6], xt = stackalloc double[6];
            while (truth.Altitude(x) > 0 && t < 900)
            {
                if (t >= nextPlan)
                {
                    var res = planner.Solve(model, cfg, x, t, a, b, y, plan);
                    solves++;
                    if (res.Plan != null && res.Plan.Converged) plan = res.Plan; else refused++;
                    nextPlan = t + 2.0;
                }
                if (plan != null)
                {
                    var (pa, pb, py) = plan.ControlAt(t);
                    double step = cfg.AngleRateDegS * Deg * dt;
                    a += Math.Clamp(pa - a, -step, step); b += Math.Clamp(pb - b, -step, step);
                    y += Math.Clamp(py - y, -cfg.YawRateDegS * Deg * dt, cfg.YawRateDegS * Deg * dt);
                }
                if (double.IsNaN(yaw30) && t >= 30) { yaw30 = y; cross30 = x[1]; }
                foreach (int km in new[] { 12, 8, 4 })
                    if (!at.ContainsKey(km) && truth.Altitude(x) <= km * 1000.0)
                        at[km] = Math.Sqrt(x[0] * x[0] + x[1] * x[1]);
                truth.Derivative(x, a, b, y, k1);
                for (int j = 0; j < 6; j++) xt[j] = x[j] + 0.5 * dt * k1[j];
                truth.Derivative(xt, a, b, y, k2);
                for (int j = 0; j < 6; j++) xt[j] = x[j] + 0.5 * dt * k2[j];
                truth.Derivative(xt, a, b, y, k3);
                for (int j = 0; j < 6; j++) xt[j] = x[j] + dt * k3[j];
                truth.Derivative(xt, a, b, y, k4);
                for (int j = 0; j < 6; j++) x[j] += dt / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
                t += dt;
            }
            double miss = Math.Sqrt(x[0] * x[0] + x[1] * x[1]);
            bool ok = miss <= 500 && plan != null;
            if (!ok) bad++;
            Console.WriteLine(FormattableString.Invariant(
                $"  {(ok ? "ok  " : "FAIL")} {name}: lands {miss,5:F0} m off after {t:F0} s; {solves} solves, {refused} refused; off the site {at.GetValueOrDefault(12, double.NaN) / 1000:F2} / {at.GetValueOrDefault(8, double.NaN) / 1000:F2} / {at.GetValueOrDefault(4, double.NaN) / 1000:F2} km at 12 / 8 / 4 km up; yaw {yaw30 / Deg:F0} deg and {cross30 / 1000:F1} km across at 30 s"));
        }
        return bad;
    }

    static int RunFrom(double mach, double altitude, bool glide)
    {
        double[] HandOver(double d, double c) => BellyflopCheck.HandOver(d, c, mach, altitude);
        int bad = 0;
        var cfg = new BellyflopConfig();
        var model = Model();
        double alpha0 = 50 * Deg;

        // Where the craft falls flying belly-first from the hand-over: the site's distance a plan can reach either side of it.
        var probe = new BellyflopPlanner().Solve(model, new BellyflopConfig { MissWeight = 0.0, ColdIterations = 1 }, HandOver(0, 0), 0.0, alpha0, 0, 0, null);
        double fall = probe.Plan != null ? probe.Plan.X[probe.Plan.Nodes - 1, 0] : double.NaN;
        Console.WriteLine(FormattableString.Invariant($"  belly-first fall: {fall / 1000:F1} km downrange (seed, before any shaping)"));

        var cases = glide
            ? new[] { ("20 km beyond the fall", 20_000.0, 0.0), ("50 km beyond", 50_000.0, 0.0), ("80 km beyond", 80_000.0, 0.0), ("100 km beyond, 10 km beside", 100_000.0, 10_000.0), ("on the fall", 0.0, 0.0) }
            : null;
        foreach (var (name, down, cross) in cases ?? new[]
                 {
                     ("on the fall", 0.0, 0.0), ("2 km short", -2000.0, 0.0), ("3 km long", 3000.0, 0.0),
                     ("2 km beside", 0.0, 2000.0), ("4 km long, 2 km beside", 4000.0, 2000.0),
                     ("5 km beside (first flight)", 0.0, 5000.0), ("8 km short", -8000.0, 0.0), ("8 km long", 8000.0, 0.0),
                     ("15 km short", -15000.0, 0.0), ("15 km long", 15000.0, 0.0), ("10 km beside", 0.0, 10000.0), ("20 km beside", 0.0, 20000.0),
                 })
        {
            double[] x0 = HandOver(fall + down, cross);
            var res = new BellyflopPlanner().Solve(model, cfg, x0, 0.0, alpha0, 0, 0, null);
            var p = res.Plan;
            if (p == null) { Console.WriteLine($"  FAIL {name}: no plan ({res.Why})"); bad++; continue; }
            double aMax = 0, bMax = 0, yMax = 0;
            for (int k = 1; k < p.Nodes; k++)
            {
                aMax = Math.Max(aMax, Math.Abs(p.U[k, 0])); bMax = Math.Max(bMax, Math.Abs(p.U[k, 1])); yMax = Math.Max(yMax, Math.Abs(p.U[k, 2]));
            }
            bool ok = p.Converged;
            // The wide cases probe the reach and report it; a miss there is not a failure.
            bool wide = Math.Abs(down) > 5000 || Math.Abs(cross) > 4000;
            if (!ok && !wide) bad++;
            Console.WriteLine(FormattableString.Invariant(
                $"  {(ok ? "ok  " : wide ? "out " : "FAIL")} cold, {name,-24}: miss {p.MissM,6:F0} m, defect {p.MaxDefect:F3}, {p.Iterations} it, {p.SolveMs,5:F0} ms, {p.Sigma,5:F0} s; |pitch| {aMax / Deg,4:F1} |bank| {bMax / Deg,4:F1} |yaw| {yMax / Deg,4:F1} deg{(ok ? "" : "  " + res.Why)}"));
        }

        // Closed loop: denser air, weaker drag; re-plan every 2 s warm from the true state and the angles flown.
        var loops = glide
            ? new[] { ("closed loop, 60 km beyond the fall, 5 km beside", 60_000.0, 5000.0), ("closed loop, 100 km beyond", 100_000.0, 0.0) }
            : new[] { ("closed loop, 3 km long, 1.5 km beside", 3000.0, 1500.0), ("closed loop, 2 km short", -2000.0, 0.0) };
        foreach (var (name, down, cross) in loops)
        {
            var truth = Model(1.10, 0.95);
            double[] x = HandOver(fall + down, cross);
            var planner = new BellyflopPlanner();
            BellyflopPlan? plan = null;
            double t = 0, nextPlan = 0, a = alpha0, b = 0, y = 0;
            int solves = 0, refused = 0;
            double worstMs = 0, aMax = 0, bMax = 0, gMax = 0, qMax = 0;
            const double dt = 0.05;
            Span<double> k1 = stackalloc double[6], k2 = stackalloc double[6], k3 = stackalloc double[6], k4 = stackalloc double[6], xt = stackalloc double[6];
            while (truth.Altitude(x) > 0 && t < 900)
            {
                if (t >= nextPlan)
                {
                    var res = planner.Solve(model, cfg, x, t, a, b, y, plan);
                    solves++;
                    if (res.Plan != null && res.Plan.Converged) { plan = res.Plan; worstMs = Math.Max(worstMs, plan.SolveMs); }
                    else refused++;
                    nextPlan = t + 2.0;
                }
                if (plan != null)
                {
                    var (pa, pb, py) = plan.ControlAt(t);
                    // The attitude follows the plan at the flaps' rate.
                    double step = cfg.AngleRateDegS * Deg * dt;
                    a += Math.Clamp(pa - a, -step, step); b += Math.Clamp(pb - b, -step, step);
                    y += Math.Clamp(py - y, -cfg.YawRateDegS * Deg * dt, cfg.YawRateDegS * Deg * dt);
                }
                if (t > 10) { aMax = Math.Max(aMax, Math.Abs(a)); bMax = Math.Max(bMax, Math.Abs(b)); }
                if (truth.AirForce(x, a, b, y, out double fx, out double fy, out double fz))
                    gMax = Math.Max(gMax, Math.Sqrt(fx * fx + fy * fy + fz * fz) / truth.Mass / 9.81);
                double vNow = Math.Sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
                qMax = Math.Max(qMax, 0.5 * truth.Density(truth.Altitude(x)) * truth.DensityScale * vNow * vNow);
                truth.Derivative(x, a, b, y, k1);
                for (int j = 0; j < 6; j++) xt[j] = x[j] + 0.5 * dt * k1[j];
                truth.Derivative(xt, a, b, y, k2);
                for (int j = 0; j < 6; j++) xt[j] = x[j] + 0.5 * dt * k2[j];
                truth.Derivative(xt, a, b, y, k3);
                for (int j = 0; j < 6; j++) xt[j] = x[j] + dt * k3[j];
                truth.Derivative(xt, a, b, y, k4);
                for (int j = 0; j < 6; j++) x[j] += dt / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
                t += dt;
            }
            double miss = Math.Sqrt(x[0] * x[0] + x[1] * x[1]);
            double vEnd = Math.Sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
            bool ok = miss <= 500 && plan != null;
            if (!ok) bad++;
            Console.WriteLine(FormattableString.Invariant(
                $"  {(ok ? "ok  " : "FAIL")} {name}: lands {miss,5:F0} m from the site at {vEnd:F0} m/s after {t:F0} s; {solves} solves, {refused} refused, worst {worstMs:F0} ms; |pitch| {aMax / Deg:F1} |bank| {bMax / Deg:F1} deg after 10 s; load {gMax:F2} g, q {qMax / 1000:F1} kPa at most"));
        }
        return bad;
    }
}

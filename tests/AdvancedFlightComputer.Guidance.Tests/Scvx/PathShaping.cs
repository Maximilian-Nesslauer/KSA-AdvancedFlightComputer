using System.Globalization;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// --gs-far deg: the touchdown checks' outer glideslope cone, deg above the horizon, in place of the one each check sets (most set none). --gs-funnel: the progressive glideslope as flight sets it (Ksa6DofSetup.GlideFunnelKnots), half-angles from vertical --gs-funnel-deg (default "20,30,60") at one craft length (--gs-len, 40 m for the checks' boosters), 300 m and 1000 m; --gs-weight its price (10). --gs-side the sideways-speed cone, 1/s (0, off).
/// </summary>
internal static class PathShaping
{
    static string? Text(string name)
    {
        string[] args = Environment.GetCommandLineArgs();
        int at = Array.IndexOf(args, name);
        return at >= 0 && at + 1 < args.Length ? args[at + 1] : null;
    }

    static double Arg(string name, double fallback)
        => Text(name) is { } t ? double.Parse(t, CultureInfo.InvariantCulture) : fallback;

    static bool On => Environment.GetCommandLineArgs().Contains("--gs-funnel");

    internal static double FarDeg(double own) => Arg("--gs-far", own);
    internal static double FunnelWeight => Arg("--gs-weight", 10.0);
    internal static double SideSpeedPerM => Arg("--gs-side", 0.0);

    /// <summary>--slew-share s: the plan's gimbal slew is this share of the hardware's (1), as flight plans at its torque share (Ksa6DofSetup); the plant keeps the hardware's.</summary>
    internal static double SlewShare => Arg("--slew-share", 1.0);

    /// <summary>--plant-slew: the plant's gimbal ANGLE - lateral thrust over axial - moves towards the commanded one no faster than the hardware slews (the plan's GimbalSlewPerS over --slew-share, times tan(gimbal), per second), as the game's does, instead of taking the command at once. Rewrites u's lateral thrust in place at u's thrust; (gx, gy) is the gimbal's angle as td/T, carried between steps, NaN to start at the command.</summary>
    internal static void SlewPlant(Scvx6DofConfig cfg, double[] u, double dt, ref double gx, ref double gy)
    {
        if (!Environment.GetCommandLineArgs().Contains("--plant-slew") || !(cfg.GimbalSlewPerS > 0.0))
            return;
        double thrust = Math.Max(u[Dynamics6Dof.IT], 1e-9);
        double cx = u[Dynamics6Dof.ITDX] / thrust, cy = u[Dynamics6Dof.ITDY] / thrust;
        if (!double.IsFinite(gx) || !double.IsFinite(gy)) { gx = cx; gy = cy; return; }
        double step = cfg.GimbalSlewPerS / SlewShare * cfg.TanGimbal * dt;
        gx += Math.Clamp(cx - gx, -step, step);
        gy += Math.Clamp(cy - gy, -step, step);
        u[Dynamics6Dof.ITDX] = gx * thrust;
        u[Dynamics6Dof.ITDY] = gy * thrust;
    }

    /// <summary>--anchor: a warm re-solve's first gimbal command is held within --anchor-s (0.3 s) of the plan's slew of where the gimbal is - the plant's angle (gx, gy) at node 0's thrust under --plant-slew, else node 0 of the shifted seed, the command being flown - as flight does (Ksa6DofGuidance.FirstGimbalAnchorAt). --anchor-cmd: the command even under --plant-slew. Off: free, as before.</summary>
    internal static void Anchor(Scvx6DofConfig cfg, double[] shiftedU, bool coasting = false, double gx = double.NaN, double gy = double.NaN)
    {
        // --vc-per-interval: virtual control priced per interval, as before 2026-11-27 (Scvx6DofConfig.VirtualControlRefIntervalS 0); set here, before every warm re-solve.
        if (Environment.GetCommandLineArgs().Contains("--vc-per-interval"))
            cfg.VirtualControlRefIntervalS = 0.0;
        if (!Environment.GetCommandLineArgs().Contains("--anchor") || coasting || cfg.CoastIntervals > 0 || !(cfg.GimbalSlewPerS > 0.0) || cfg.TorqueDemand)
        {
            cfg.FirstGimbalAnchor = null;
            return;
        }
        double thrust = shiftedU[Dynamics6Dof.IT];
        double band = cfg.GimbalSlewPerS * Arg("--anchor-s", 0.3) * cfg.TanGimbal * Math.Max(thrust, cfg.Tmin);
        bool actual = double.IsFinite(gx) && double.IsFinite(gy) && !Environment.GetCommandLineArgs().Contains("--anchor-cmd");
        cfg.FirstGimbalAnchor = actual
            ? [gx * thrust, gy * thrust, band]
            : [shiftedU[Dynamics6Dof.ITDX], shiftedU[Dynamics6Dof.ITDY], band];
    }

    /// <summary>--gate-per-s: the closed-loop checks judge re-solves with flight's per-second gate (Ksa6DofGuidance.GateReferenceIntervalS, 0.35 s, --gate-ref) instead of a flat tolerance per interval; 0 when off.</summary>
    internal static double GateRefS => Environment.GetCommandLineArgs().Contains("--gate-per-s") ? Arg("--gate-ref", 0.35) : 0.0;

    /// <summary>--late-its k: a re-solve of a plan with under 2 s left gets k SCvx iterations (3, as the others), as flight gives a short plan a larger budget (Ksa6DofGuidance.LateCycleBudgetMs).</summary>
    internal static int Iterations(double planLeftS) => planLeftS < 2.0 ? (int)Arg("--late-its", 3.0) : 3;

    /// <summary>--soft-end: the check's end attitude, pitch and yaw rate soft and its end speed and height priced as flight prices them against the miss across (Ksa6DofSetup, 2026-11-27): a m/s, a degree and a deg/s at five metres each, a metre of height at twenty.</summary>
    internal static Scvx6DofConfig SoftEnd(Scvx6DofConfig cfg)
    {
        if (!Environment.GetCommandLineArgs().Contains("--soft-end") || !(cfg.TerminalMissWeight > 0.0))
            return cfg;
        double perM = cfg.TerminalMissWeight / cfg.TerminalMissPriceScale;
        cfg.TerminalSpeedWeight = Arg("--end-speed", 5.0) * perM * cfg.TerminalSpeedPriceScale;
        cfg.TerminalHeightMissFactor = Arg("--end-height", 20.0);
        cfg.TerminalAttitudeWeight = Arg("--end-att", 5.0) * perM;
        cfg.TerminalRateWeight = Arg("--end-rate", 5.0) * perM;
        // --phase: flight's last-seconds prices against the check's metre of miss (400 a metre in flight: 100 a degree squared second of tilt, 25 a (deg/s) squared second of rate, over the last 3 s), and its cheap sideways end speed.
        if (Environment.GetCommandLineArgs().Contains("--phase"))
        {
            cfg.TerminalPhaseS = Arg("--phase-s", 3.0);
            cfg.TerminalPhaseTiltWeight = Arg("--phase-w", 1.0) * perM * 100.0 / 400.0;
            cfg.TerminalPhaseRateWeight = Arg("--phase-w", 1.0) * perM * 25.0 / 400.0;
            cfg.TerminalSideSpeedFactor = 0.05;
        }
        // --vc-att: attitude and body-rate virtual control priced on flight's tight scales (Ksa6DofSetup.VirtualControlAttitudeScale, VirtualControlRateScaleRadS), not on 1 and 1 rad/s.
        if (Environment.GetCommandLineArgs().Contains("--vc-att") && cfg.VirtualControlScale is { Length: >= 13 } vcs)
        {
            for (int i = 6; i < 10; i++) vcs[i] = Arg("--vc-q", 0.005);
            for (int i = 10; i < 13; i++) vcs[i] = Arg("--vc-w", 0.01);
            cfg.VirtualControlScale = vcs;
        }
        // --tip: the tip-over line, on a craft standing --tip-r (3 m) out with its centre of mass --tip-h (13 m, the checks' target) up and --tip-i (1e6 kg m^2) of inertia.
        if (Environment.GetCommandLineArgs().Contains("--tip"))
        {
            cfg.TipLimit = true;
            cfg.TipRadiusM = Arg("--tip-r", 3.0);
            cfg.TipHeightM = Arg("--tip-h", 13.0);
            cfg.TipInertiaKgM2 = Arg("--tip-i", 1.0e6);
            cfg.TipWeight = perM * 2.0e4 / 400.0;
        }
        return cfg;
    }

    static readonly List<double> Drifts = [];

    /// <summary>How far a kept plan's attitudes stray from its own rates (fly its rates from node 0; the worst miss over the plan, degrees), noted for plans with under 2.5 s left: what the 2026-11-27 Starship landings showed at 5-12 deg.</summary>
    internal static void NoteDrift(Scvx6DofSolver s, int n, double planLeftS)
    {
        if (!(planLeftS < 2.5))
            return;
        double[] x = s.ReferenceX;
        const int NX = Dynamics6Dof.NX, Q = Dynamics6Dof.IQ, W = Dynamics6Dof.IW;
        double qw = x[Q], qx = x[Q + 1], qy = x[Q + 2], qz = x[Q + 3], worst = 0.0;
        for (int k = 0; k < n - 1; k++)
        {
            double dt = s.NodeTime(k + 1) - s.NodeTime(k);
            const int sub = 40;
            for (int i = 0; i < sub; i++)
            {
                double f = (i + 0.5) / sub;
                double wx = x[k * NX + W] * (1 - f) + x[(k + 1) * NX + W] * f;
                double wy = x[k * NX + W + 1] * (1 - f) + x[(k + 1) * NX + W + 1] * f;
                double wz = x[k * NX + W + 2] * (1 - f) + x[(k + 1) * NX + W + 2] * f;
                double h = 0.5 * dt / sub;
                double nw = qw + h * (-qx * wx - qy * wy - qz * wz);
                double nx = qx + h * (qw * wx + qy * wz - qz * wy);
                double ny = qy + h * (qw * wy - qx * wz + qz * wx);
                double nz = qz + h * (qw * wz + qx * wy - qy * wx);
                double norm = Math.Sqrt(nw * nw + nx * nx + ny * ny + nz * nz);
                qw = nw / norm; qx = nx / norm; qy = ny / norm; qz = nz / norm;
            }
            int b = (k + 1) * NX + Q;
            double dot = Math.Abs(qw * x[b] + qx * x[b + 1] + qy * x[b + 2] + qz * x[b + 3]);
            worst = Math.Max(worst, 2.0 * Math.Acos(Math.Min(1.0, dot)) * 180.0 / Math.PI);
        }
        Drifts.Add(worst);
    }

    /// <summary>The late plans' drift noted since the last call, for the loop's line, and forgets it.</summary>
    internal static string DriftSummary()
    {
        if (Drifts.Count == 0)
            return "";
        Drifts.Sort();
        string text = FormattableString.Invariant($"; late plans off their own rates median {Drifts[Drifts.Count / 2]:F1} max {Drifts[^1]:F1} deg ({Drifts.Count})");
        Drifts.Clear();
        return text;
    }

    internal static double[]? FunnelHeightsM => On ? [Arg("--gs-len", 40.0), 300.0, 1000.0] : null;

    internal static double[]? FunnelDeg => On
        ? (Text("--gs-funnel-deg") ?? "20,30,60").Split(',').Select(v => double.Parse(v, CultureInfo.InvariantCulture)).ToArray()
        : null;

    /// <summary>--glide-funnel: the progressive glideslope's radius is continuous and its slope is its run, it is the plain cone when off, it holds the half-angles at the knots, and no tangent lies outside it (convex in height).</summary>
    internal static int Check()
    {
        int failures = 0;
        void Expect(string what, bool ok) { Console.WriteLine((ok ? "  ok   " : "  FAIL ") + what); if (!ok) failures++; }
        double[] h = [40.0, 300.0, 1000.0], a = [20.0, 30.0, 60.0];
        double plain = Scvx6DofConfig.Radius(10.0, null, null, 100.0);
        Expect($"off, it is the plain cone: {plain:F2} m at 100 m", Math.Abs(plain - 100.0 / Math.Tan(10.0 * Math.PI / 180.0)) < 1e-9);
        Expect("below one craft length it is the 20 deg cone", Math.Abs(Scvx6DofConfig.Radius(10.0, h, a, 30.0) - 30.0 * Math.Tan(20.0 * Math.PI / 180.0)) < 1e-9);
        Expect("at 300 m it allows 300 tan 30 deg, at 1000 m 1000 tan 60 deg (the angle seen from the target)",
            Math.Abs(Scvx6DofConfig.Radius(10.0, h, a, 300.0) - 300.0 * Math.Tan(30.0 * Math.PI / 180.0)) < 1e-9
            && Math.Abs(Scvx6DofConfig.Radius(10.0, h, a, 1000.0) - 1000.0 * Math.Tan(60.0 * Math.PI / 180.0)) < 1e-9);
        Console.WriteLine(FormattableString.Invariant($"  offsets allowed: {Scvx6DofConfig.Radius(10.0, h, a, 40.0):F0} m at 40 m, {Scvx6DofConfig.Radius(10.0, h, a, 100.0):F0} m at 100 m, {Scvx6DofConfig.Radius(10.0, h, a, 300.0):F0} m at 300 m, {Scvx6DofConfig.Radius(10.0, h, a, 1000.0):F0} m at 1000 m, {Scvx6DofConfig.Radius(10.0, h, a, 2000.0):F0} m at 2000 m"));
        double worstSlope = 0.0, worstTangent = double.NegativeInfinity;
        for (double z = 0.0; z < 2500.0; z += 2.5)
        {
            double r0 = Scvx6DofConfig.Radius(10.0, h, a, z), r1 = Scvx6DofConfig.Radius(10.0, h, a, z + 2.5);
            if (!h.Any(k => k > z && k < z + 2.5))
                worstSlope = Math.Max(worstSlope, Math.Abs((r1 - r0) / 2.5 - Scvx6DofConfig.RunPerM(10.0, h, a, z + 1.25)));
            for (double href = 0.0; href < 2500.0; href += 37.5)
                worstTangent = Math.Max(worstTangent, Scvx6DofConfig.Radius(10.0, h, a, href) + Scvx6DofConfig.RunPerM(10.0, h, a, href) * (z - href) - r0);
        }
        Expect($"its slope is its run away from the heights (worst {worstSlope:E1})", worstSlope < 1e-3);
        Expect($"no tangent lies outside it (worst {worstTangent:F4} m)", worstTangent < 1e-6);
        Console.WriteLine(failures == 0 ? "PASS - the progressive glideslope" : "FAIL - see above");
        return failures == 0 ? 0 : 1;
    }
}

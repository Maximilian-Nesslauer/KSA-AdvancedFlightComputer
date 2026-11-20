using System.Globalization;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;


/// <summary>
/// --6dof-coast: TOUCHDOWN GUIDANCE OWNING IGNITION (Scvx6DofConfig.CoastIntervals). On 2026-11-20 touchdown guidance's standby was converged 15 s before ignition, but over the last two seconds of the glide the booster flew 4-5 deg off entry guidance's command, the standby's forecast missed, its re-solves at ignition were refused, and it took the burn late (52 m up, or 158 m up and 16 m off the target with 5 s left). Here touchdown guidance takes the craft in the glide instead, a few seconds before ignition, with a coast phase ahead of the burn whose duration - the ignition time - is free.
///  Compared, from the same glide state on the same plant (its air moment 10 % stronger than the plan's, and a steady cross-wind push the plan does not know of):
///   TWO-PHASE: one plan from the glide to the ground, re-solved warm every 0.1 s, its ignition locked a second out and pinned at zero once lit.
///   TODAY'S WAY: the craft coasts holding its rates at zero with the RCS (as entry guidance holds its attitude) to the ignition time the two-phase plan first chose, and touchdown guidance takes the burn there, converged cold on the state it finds.
/// </summary>
internal static class SixDofCoastCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NUBase;
    private const double Cycle = 0.1, Dt = 0.01, LockS = 1.0, WarmTrustRegion = 0.05;

    // The cross-wind push the plan does not model, m/s^2, site frame: entry guidance's logged cross-flow bias was 0.4-0.8 m/s^2.
    private static readonly double[] Push = [0.0, 0.5, 0.0];

    // The glide: the logged 11:16 booster 1.4 km up, 104 m/s, coasting at the logged 2 deg off the air. Each case starts this long before the coast reaches 240 m, where entry guidance lit it, with this many nodes and coast intervals.
    private static readonly double[] GlideR = [457.0, 15.0], GlideV = [-44.0, -2.0, -95.0];
    private const double GlideH = 2000.0, GlideTiltDeg = 2.0, IgnitionAlt = 240.0, BurnSeed = 5.5;
    private static readonly (double lead, int n, int c)[] Cases = [(5.0, 40, 12), (10.0, 50, 20), (15.0, 50, 20)];

    internal static int Run(bool verbose)
    {
        string[] args = Environment.GetCommandLineArgs();
        double Num(string flag, double dflt) { int i = Array.IndexOf(args, flag); return i >= 0 && i + 1 < args.Length ? double.Parse(args[i + 1], CultureInfo.InvariantCulture) : dflt; }
        SixDofRcsCheck.RcsWeight = Num("--rcs-weight", 0.1);
        SixDofRcsCheck.RateMaxDegS = Num("--rate-max", 15.0);
        SixDofRcsCheck.ProbeEps = Num("--eps", 1e-3);
        Push[1] = Num("--push", 0.5);
        SixDofRcsCheck.AbsTerminal = args.Contains("--abs-terminal");
        SixDofRcsCheck.RateScale = Num("--rate-scale", 1.0);
        int only = (int)Num("--case", -1);
        SpreadDt = Num("--spread", 0.0);
        SixDofRcsCheck.IrollUsed = Num("--izz", SixDofRcsCheck.IrollUsed);
        SixDofRcsCheck.RollCl = Num("--roll-cl", 0.0);
        SixDofRcsCheck.RcsRollNm = Num("--rcs-roll", 0.0);
        SixDofRcsCheck.RollPinned = args.Contains("--roll-pinned");
        SeedDefects = args.Contains("--seed-defects");
        PathSeed = args.Contains("--path-seed");
        SixDofRcsCheck.CoastTurnMs = Num("--coast-turn", 0.0);
        SpreadFree = args.Contains("--spread-free");
        SpreadResample = args.Contains("--spread-resample");
        bool coldOnly = args.Contains("--cold-only");
        Console.WriteLine($"  terminal priced {(SixDofRcsCheck.AbsTerminal ? "per 250 m and 100 m/s" : "on the problem's scales")}, rate scale {SixDofRcsCheck.RateScale} rad/s, ignition scales {args.Contains("--ignition-scales")}");
        Console.WriteLine($"TOUCHDOWN GUIDANCE OWNING IGNITION: RCS weight {SixDofRcsCheck.RcsWeight}, rate limit {SixDofRcsCheck.RateMaxDegS} deg/s, eps {SixDofRcsCheck.ProbeEps}, unmodelled push {Push[1]} m/s^2 sideways, plant moment x{SixDofRcsCheck.PlantMomentFactor}");

        Dynamics6Dof.RcsModel rcs = SixDofRcsCheck.RcsModel();
        // The test's air tables glide this booster at 140 m/s where the logged one glided at 100-105: their reference area is scaled up to the logged speed, and the moment table down by as much, so the air's turn stays the logged 0.04 rad/s^2 per degree at 100 m/s.
        double area = Num("--area", 2.2);
        Dynamics6Dof.Params dyn = SixDofRcsCheck.Params(SixDofRcsCheck.MomentPerDeg(1.0 / area), rcs, area);
        Dynamics6Dof.Params plant = SixDofRcsCheck.Params(SixDofRcsCheck.MomentPerDeg(SixDofRcsCheck.PlantMomentFactor / area), rcs, area);

        // The glide on the plan's own model, to 240 m: where each case starts, as entry guidance's plan would forecast it.
        var glide = new List<(double t, double[] x)>();
        {
            double[] x = SixDofRcsCheck.State(GlideH, GlideR, GlideV, GlideTiltDeg, 16_400.0);
            var hold = new double[NU];
            double t = 0.0;
            while (x[2] > IgnitionAlt && t < 60.0)
            {
                glide.Add((t, (double[])x.Clone()));
                Damp(x, dyn, hold);
                SixDofRcsCheck.Step(x, hold, dyn, Dt);
                t += Dt;
            }
            glide.Add((t, (double[])x.Clone()));
        }
        double ignitionT = glide[^1].t;
        Glide = glide;
        if (args.Contains("--ignition-scales")) ScaleFrom = glide[^1].x;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  the glide from {GlideH:F0} m reaches {IgnitionAlt:F0} m in {ignitionT:F1} s, {Math.Sqrt(glide[^1].x[0] * glide[^1].x[0] + glide[^1].x[1] * glide[^1].x[1]):F0} m from the target at {Speed(glide[^1].x):F0} m/s"));

        bool ok = true;
        for (int ci = 0; ci < Cases.Length; ci++)
        {
            if (only >= 0 && ci != only) continue;
            (double lead, int n, int c) = Cases[ci];
            double[] x0 = (double[])glide.MinBy(p => Math.Abs(p.t - (ignitionT - lead))).x.Clone();
            // --wz0 / --wx0: the body rates the craft starts the plan with, rad/s (entry guidance's craft is never still).
            x0[12] = Num("--wz0", x0[12]);
            x0[10] = Num("--wx0", x0[10]);
            Console.WriteLine();
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"  {lead:F0} s before ignition: {x0[2]:F0} m, {Speed(x0):F0} m/s, {Math.Sqrt(x0[0] * x0[0] + x0[1] * x0[1]):F0} m from the target; {n} nodes, {c} in the coast"));
            (string cold, double _, bool solved) = Cold(dyn, x0, lead, BurnSeed, n, c, verbose);
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"    sideways: plan bends up to {LastSideways:F1} m off the straight line to the pad"));
            ok &= solved;
            Console.WriteLine($"    two-phase   cold: {cold}");
            if (coldOnly) continue;
            Console.WriteLine($"    two-phase   loop: {Loop(dyn, plant, x0, lead, BurnSeed, n, c, verbose)}");

            // Today's way: hold the rates through the glide to the forecast ignition, then a burn-only plan from there.
            double[] x = (double[])x0.Clone();
            var hold = new double[NU];
            for (double t = 0.0; t < lead - 1e-9; t += Dt)
            {
                Damp(x, dyn, hold);
                Step(x, hold, plant, Dt);
            }
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"    today's way: coasted {lead:F1} s holding the rates to {x[2]:F0} m, {Math.Sqrt(x[0] * x[0] + x[1] * x[1]):F1} m from the target ({x[1]:F1} m across), {SixDofRcsCheck.OffAir(x) * 180.0 / Math.PI:F1} deg off the air"));
            Console.WriteLine($"    burn only  loop: {Loop(dyn, plant, x, 0.0, BurnSeed, n - c, 0, verbose)}");
        }
        Console.WriteLine();
        Console.WriteLine(ok ? "PASS - the two-phase plans converge (landings reported above)" : "FAIL - a two-phase cold solve did not reach the gate");
        return ok ? 0 : 1;
    }

    // --ignition-scales: the two-phase solver scaled on the state at the forecast ignition rather than the hand-over, as a burn-only solver there would be.
    private static double[]? ScaleFrom;

    private static double[] WithMass(double[] x, double mass) { var y = (double[])x.Clone(); y[13] = mass; return y; }

    private static double Speed(double[] x) => Math.Sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);

    private static Scvx6DofSolver Solver(Dynamics6Dof.Params dyn, double[] x0, double burn, int n, int c, out Scvx6DofConfig cfg)
    {
        double[] xf = SixDofRcsCheck.Target(x0[13]);
        cfg = SixDofRcsCheck.Config(n, burn, ScaleFrom != null && c > 0 ? WithMass(ScaleFrom, x0[13]) : x0, xf, dyn, c);
        cfg.CoastSigmaMax = 30.0;
        return SixDofRcsCheck.Solver(cfg, dyn);
    }

    private static (string, double, bool) Cold(Dynamics6Dof.Params dyn, double[] x0, double coast, double burn, int n, int c, bool verbose)
    {
        Scvx6DofSolver s = Solver(dyn, x0, burn, n, c, out Scvx6DofConfig cfg);
        double[] xf = SixDofRcsCheck.Target(x0[13]);
        double[] xs, us;
        if (PathSeed)
            ColdSeed6Dof.BuildAlongPath(x0, xf, coast, burn, n, c, cfg, dyn, EntryLikePath(dyn, coast, burn, n - c), out xs, out us);
        else
            ColdSeed6Dof.BuildWithCoast(x0, xf, coast, burn, n, c, cfg, dyn, out xs, out us);
        s.Initialize(x0, xf, xs, us, burn, coastSigmaSeed: coast);
        if (SeedDefects)
            PrintSeedDefects(xs, us, n, c, burn, coast, dyn);
        double[] tol = SixDofRcsCheck.Tolerance(x0[13]);
        int toGate = -1, toFine = -1;
        double best = double.PositiveInfinity;
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var admm = new List<int>();
        double[] craft = (double[])x0.Clone();
        var hold = new double[NU];
        for (int i = 1; i <= 40; i++)
        {
            // --spread dt: as the standby's spread cold solve flies (Ksa6DofGuidance.StepCold) - the craft glides on between iterations, held by the RCS as entry guidance holds it, and the reference is re-anchored on it; the coast is shortened by the time gone (--spread-free: left as the solver had it).
            if (SpreadDt > 0.0 && i > 1)
            {
                for (double h = 0.0; h < SpreadDt - 1e-9; h += Dt)
                {
                    Damp(craft, dyn, hold);
                    SixDofRcsCheck.Step(craft, hold, dyn, Dt);
                }
                double coastNow = SpreadFree ? s.CoastSigma : Math.Max(s.CoastSigma - SpreadDt, 0.0);
                double[] rx = (double[])s.ReferenceX.Clone(), ru = (double[])s.ReferenceU.Clone();
                if (SpreadResample)
                    ResampleCoast(s, n, c, SpreadDt, coastNow, out rx, out ru);
                ColdReferenceShift.Apply(rx, n, s.Sigma, craft, c, coastNow);
                s.Reseed(craft, rx, ru, s.Sigma, trustRegion: s.TrustRegion, coastSigma: coastNow);
            }
            ScvxIteration it = s.Iterate();
            admm.Add(it.SolverIterations);
            double ratio = it.Accepted ? s.WeightedDefect(tol, n - 2, 1.0, out int ch, out int node, out double raw) : double.PositiveInfinity;
            best = Math.Min(best, ratio);
            if (verbose)
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"        {i,2}: solved {it.Solved,-5} accepted {it.Accepted,-5} trust {it.TrustRegion,6:G3} coast {s.CoastSigma,5:F2} burn {s.Sigma,5:F2} gate {ratio,7:F3}x scs {it.SolverIterations} worst {Scvx6DofSolver.ChannelName(it.DefectChannel)} at {it.DefectNode} ({it.DefectNorm:G3} scaled){(it.Solved ? "" : " FAILED " + s.LastFailureReason)}"));
            if (toGate < 0 && ratio <= 1.0) toGate = i;
            if (ratio <= 0.05) { toFine = i; break; }
        }
        {
            double ex = x0[0], ey = x0[1], len = Math.Max(Math.Sqrt(ex * ex + ey * ey), 1e-9);
            LastSideways = 0.0;
            for (int k = 0; k < n; k++)
            {
                double side = (-ey * s.ReferenceX[k * NX] + ex * s.ReferenceX[k * NX + 1]) / len;
                if (Math.Abs(side) > Math.Abs(LastSideways)) LastSideways = side;
            }
        }
        double peak = SixDofRcsCheck.PeakRcs(s.ReferenceU, n, out double share);
        int ign = c * NX;
        return (string.Create(CultureInfo.InvariantCulture,
            $"gate in {(toGate < 0 ? "-" : toGate.ToString())}, 5 % of it in {(toFine < 0 ? "- (40 tried)" : toFine.ToString())}, best {best:F3}x, coast {s.CoastSigma:F2} s to ignition at {s.ReferenceX[ign + 2]:F0} m, burn {s.Sigma:F2} s, {clock.Elapsed.TotalMilliseconds / 1e3:F1} s, SCS {SixDofRcsCheck.Median(admm)}/iteration; peak RCS {peak / 1e3:F0} kN m ({share:P0} of reach)"),
            s.CoastSigma, toGate > 0);
    }

    private static bool SeedDefects;
    private static double LastSideways;

    // --path-seed: the cold seed laid along a path as entry guidance's plan gives one (ColdSeed6Dof.BuildAlongPath) - here the glide to ignition and a converged burn-only plan from there.
    private static bool PathSeed;
    private static List<(double t, double[] x)> Glide = [];

    /// <summary>A path like entry guidance's plan: the glide's last <paramref name="coast"/> s, then a burn-only plan converged from where the glide ends, its thrust along body z.</summary>
    private static SeedPath EntryLikePath(Dynamics6Dof.Params dyn, double coast, double burn, int burnNodes)
    {
        double[] ign = Glide[^1].x;
        Scvx6DofSolver b = Solver(dyn, ign, burn, burnNodes, 0, out Scvx6DofConfig bcfg);
        double[] bxf = SixDofRcsCheck.Target(ign[13]);
        ColdSeed6Dof.Build(ign, bxf, burn, burnNodes, bcfg, dyn, out double[] bx, out double[] bu);
        b.Initialize(ign, bxf, bx, bu, burn);
        double[] tol = SixDofRcsCheck.Tolerance(ign[13]);
        int it = 0;
        for (; it < 40; it++)
        {
            ScvxIteration step = b.Iterate();
            if (step.Accepted && b.WeightedDefect(tol, burnNodes - 2, 1.0, out _, out _, out _) <= 0.05) break;
        }
        double[] px = (double[])b.ReferenceX.Clone(), pu = (double[])b.ReferenceU.Clone();
        double sigma = b.Sigma, tEnd = Glide[^1].t;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"    path: the glide's last {coast:F1} s, then a burn-only plan of {sigma:F2} s converged in {it + 1} iterations"));
        return (int phase, double f, Span<double> rv, Span<double> axis, out double thrustN) =>
        {
            double[] x;
            double t = 0.0;
            if (phase == 0)
            {
                double at = tEnd - coast * (1.0 - f);
                int i = Math.Clamp(Glide.FindIndex(g => g.t >= at), 1, Glide.Count - 1);
                double w = Math.Clamp((at - Glide[i - 1].t) / Math.Max(Glide[i].t - Glide[i - 1].t, 1e-9), 0.0, 1.0);
                x = new double[NX];
                for (int j = 0; j < NX; j++) x[j] = (1 - w) * Glide[i - 1].x[j] + w * Glide[i].x[j];
            }
            else
            {
                double u = f * (burnNodes - 1);
                int i = Math.Min((int)u, burnNodes - 2);
                double w = u - i;
                x = new double[NX];
                for (int j = 0; j < NX; j++) x[j] = (1 - w) * px[i * NX + j] + w * px[(i + 1) * NX + j];
                t = (1 - w) * pu[i * NU + Dynamics6Dof.IT] + w * pu[(i + 1) * NU + Dynamics6Dof.IT];
            }
            for (int j = 0; j < 6; j++) rv[j] = x[j];
            double qw = x[6], qx = x[7], qy = x[8], qz = x[9];
            axis[0] = 2.0 * (qx * qz + qw * qy); axis[1] = 2.0 * (qy * qz - qw * qx); axis[2] = 1.0 - 2.0 * (qx * qx + qy * qy);
            thrustN = t;
            return true;
        };
    }

    /// <summary>--seed-defects: where the cold seed itself breaks the dynamics - each interval's trapezoid defect on the body rates and the attitude, and the seed's rates and roll angle, around ignition and at the end.</summary>
    private static void PrintSeedDefects(double[] x, double[] u, int n, int c, double burn, double coast, Dynamics6Dof.Params dyn)
    {
        Dynamics6Dof.Params off = dyn.WithEngineOff();
        var f0 = new double[NX];
        var f1 = new double[NX];
        Console.WriteLine("        seed: interval, dt, defect on wx wy wz (rad/s), on the attitude (deg), node rates wx wy wz (deg/s), twist about the axis (deg), tilt (deg)");
        for (int k = 0; k < n - 1; k++)
        {
            Dynamics6Dof.Params p = k < c ? off : dyn;
            double dt = k < c ? coast / c : burn / (n - 1 - c);
            Dynamics6Dof.Eval(x.AsSpan(k * NX, NX), u.AsSpan(k * NU, NU), p, f0);
            Dynamics6Dof.Eval(x.AsSpan((k + 1) * NX, NX), u.AsSpan((k + 1) * NU, NU), p, f1);
            var d = new double[NX];
            for (int i = 0; i < NX; i++) d[i] = x[(k + 1) * NX + i] - x[k * NX + i] - 0.5 * dt * (f0[i] + f1[i]);
            double dq = 2.0 * Math.Sqrt(d[7] * d[7] + d[8] * d[8] + d[9] * d[9]) * 180.0 / Math.PI;
            if (k < 3 || (k >= c - 2 && k <= c + 3) || k >= n - 8 || Math.Abs(d[12]) > 0.05 || Math.Abs(d[10]) > 0.05 || Math.Abs(d[11]) > 0.05)
            {
                int o = (k + 1) * NX;
                double twist = 2.0 * Math.Atan2(x[o + 9], x[o + 6]) * 180.0 / Math.PI;
                double tilt = 2.0 * Math.Asin(Math.Min(1.0, Math.Sqrt(x[o + 7] * x[o + 7] + x[o + 8] * x[o + 8]))) * 180.0 / Math.PI;
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                    $"          {k,2}{(k < c ? " coast" : k == c ? " IGN  " : " burn ")} dt {dt,5:F3}  d(w) {d[10],7:F3} {d[11],7:F3} {d[12],7:F3}  d(q) {dq,6:F2}  w[{k + 1}] {x[o + 10] * 57.3,7:F1} {x[o + 11] * 57.3,7:F1} {x[o + 12] * 57.3,7:F1}  twist {twist,7:F1}  tilt {tilt,6:F1}"));
            }
        }
    }

    // --spread dt, --spread-free, --spread-resample: see Cold.
    private static double SpreadDt;
    private static bool SpreadFree, SpreadResample;

    /// <summary>The solver's reference moved on by elapsed s onto a coast of coastNow, sampled in time (as Ksa6DofGuidance.ResamplePlan does for a warm seed).</summary>
    private static void ResampleCoast(Scvx6DofSolver s, int n, int c, double elapsed, double coastNow, out double[] rx, out double[] ru)
    {
        var p = new Plan(s.ReferenceX, s.ReferenceU, s.Sigma, s.CoastSigma, 0.0);
        rx = new double[n * NX];
        ru = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double tk = elapsed + Scvx6DofSolver.NodeTime(k, n, c, s.Sigma, coastNow);
            if (c > 0 && k >= c) tk = Math.Max(tk, p.Coast);
            if (c > 0 && k < c) tk = Math.Min(tk, p.Coast);
            Array.Copy(Sample(p.X, NX, n, c, p.Burn, p.Coast, tk, burnSide: k >= c), 0, rx, k * NX, NX);
            Array.Copy(Sample(p.U, NU, n, c, p.Burn, p.Coast, tk, burnSide: k >= c), 0, ru, k * NU, NU);
        }
    }

    private static string Loop(Dynamics6Dof.Params dyn, Dynamics6Dof.Params plant, double[] x0, double coast, double burn, int n, int c, bool verbose)
    {
        Scvx6DofSolver s = Solver(dyn, x0, burn, n, c, out Scvx6DofConfig cfg);
        double[] xf = SixDofRcsCheck.Target(x0[13]);
        ColdSeed6Dof.BuildWithCoast(x0, xf, coast, burn, n, c, cfg, dyn, out double[] xs, out double[] us);
        s.Initialize(x0, xf, xs, us, burn, coastSigmaSeed: coast);
        double[] tol = SixDofRcsCheck.Tolerance(x0[13]);
        bool planned = false;
        for (int i = 0; i < 40; i++)
        {
            ScvxIteration it = s.Iterate();
            double gate = it.Accepted ? s.WeightedDefect(tol, n - 2, 1.0, out _, out _, out _) : double.PositiveInfinity;
            if (gate <= 1.0) planned = true;
            if (gate <= 0.05) break;
        }

        var plan = new Plan(s.ReferenceX, s.ReferenceU, s.Sigma, s.CoastSigma, 0.0);
        double[] x = (double[])x0.Clone();
        int refused = 0, kept = 0;
        var gates = new List<double>();
        var loopAdmm = new List<int>();
        double t = 0.0, mass0 = x0[13], peakRate = 0.0, ignitionAt = double.NaN, ignitionAlt = double.NaN, lockedIgnition = double.NaN;
        bool lit = c == 0;
        while (t < 30.0)
        {
            double left = plan.Coast + plan.Burn - (t - plan.Start);
            if (left > 0.4 && t > 0.0)
            {
                // THE IGNITION LOCK: a second out the ignition time is pinned, and once lit, the coast is pinned at zero.
                if (c > 0)
                {
                    if (lit)
                        cfg.CoastSigmaFixed = 0.0;
                    else if (double.IsFinite(lockedIgnition))
                        cfg.CoastSigmaFixed = Math.Max(lockedIgnition - t, 0.0);
                    else if (plan.Start + plan.Coast - t < LockS)
                    {
                        lockedIgnition = plan.Start + plan.Coast;
                        cfg.CoastSigmaFixed = Math.Max(lockedIgnition - t, 0.0);
                    }
                }
                Shift(plan, t - plan.Start, n, c, out double[] sx, out double[] su, out double ss, out double sc);
                ColdReferenceShift.Apply(sx, n, ss, x);
                s.Reseed(x, sx, su, ss, trustRegion: Math.Clamp(s.TrustRegion, WarmTrustRegion, s.TrustRegionMax), coastSigma: sc);
                double ratio = double.PositiveInfinity;
                for (int i = 0; i < 3; i++)
                {
                    ScvxIteration it = s.Iterate();
                    loopAdmm.Add(it.SolverIterations);
                    ratio = it.Accepted ? s.WeightedDefect(tol, n - 2, 1.0, out _, out _, out _) : double.PositiveInfinity;
                    if (ratio <= 0.05) break;
                }
                gates.Add(ratio);
                if (ratio <= 1.0)
                {
                    plan = new Plan(s.ReferenceX, s.ReferenceU, s.Sigma, s.CoastSigma, t);
                    kept++;
                }
                else
                    refused++;
                if (verbose)
                    Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                        $"        t {t,5:F1} alt {x[2],6:F1} off ({x[0],6:F1},{x[1],6:F1}) v ({x[3],6:F1},{x[4],6:F1},{x[5],6:F1}) to ignition {plan.Start + plan.Coast - t,5:F2}{(cfg.CoastSigmaFixed.HasValue ? " LOCKED" : "")} burn {plan.Burn,5:F2} gate {ratio,7:F3}x {(ratio <= 1.0 ? "kept" : "REFUSED")}"));
            }
            bool landed = false;
            for (double h = 0; h < Cycle - 1e-9; h += Dt)
            {
                double[] u = Control(plan, n, c, t - plan.Start);
                if (u[Dynamics6Dof.IT] > 0.0 && !lit)
                {
                    lit = true;
                    ignitionAt = t;
                    ignitionAlt = x[2];
                }
                if (!double.IsFinite(ignitionAlt) && lit && c == 0)
                {
                    ignitionAt = t;
                    ignitionAlt = x[2];
                }
                Step(x, u, plant, Dt);
                peakRate = Math.Max(peakRate, Math.Sqrt(x[10] * x[10] + x[11] * x[11] + x[12] * x[12]) * 180.0 / Math.PI);
                t += Dt;
                if (x[2] <= SixDofRcsCheck.TargetZ || (lit && x[5] > 0.0 && x[2] < SixDofRcsCheck.TargetZ + 20.0)) { landed = true; break; }
            }
            if (landed) break;
        }
        gates.Sort();
        double med = gates.Count > 0 ? gates[gates.Count / 2] : double.NaN;
        double miss = Math.Sqrt(x[0] * x[0] + x[1] * x[1]);
        double side = Math.Sqrt(x[3] * x[3] + x[4] * x[4]);
        return string.Create(CultureInfo.InvariantCulture,
            $"first plan {(planned ? "inside" : "OUTSIDE")} the gate; {kept} kept, {refused} refused, median gate {med:F2}x, SCS {SixDofRcsCheck.Median(loopAdmm)}/iteration; lit at {ignitionAt:F1} s, {ignitionAlt:F0} m; ends {x[2]:F1} m up, {miss:F1} m off, {-x[5]:F1} m/s down, {side:F1} m/s sideways; {mass0 - x[13]:F0} kg burned; peak rate {peakRate:F1} deg/s");
    }

    private sealed record Plan(double[] X, double[] U, double Burn, double Coast, double Start);

    /// <summary>The control a plan flies at time tau from its start: the coast's (engine off, the RCS interpolated) before ignition, the burn's after.</summary>
    private static double[] Control(Plan p, int n, int c, double tau)
    {
        bool coast = c > 0 && tau < p.Coast;
        double[] u = Sample(p.U, NU, n, c, p.Burn, p.Coast, tau, burnSide: !coast);
        if (coast)
        {
            u[Dynamics6Dof.ITDX] = 0.0;
            u[Dynamics6Dof.ITDY] = 0.0;
            u[Dynamics6Dof.IT] = 0.0;
        }
        return u;
    }

    /// <summary>A plan's values at time t: interpolated across the coast's nodes before ignition, the burn's after; at ignition itself, burnSide picks which.</summary>
    private static double[] Sample(double[] v, int width, int n, int c, double burn, double coast, double t, bool burnSide)
    {
        int a;
        double fr;
        if (c > 0 && (t < coast || (!burnSide && t <= coast)))
        {
            double idx = coast > 0.0 ? Math.Clamp(t / (coast / c), 0.0, c) : 0.0;
            a = Math.Min((int)idx, c - 1);
            fr = idx - a;
        }
        else
        {
            int nb = n - 1 - c;
            double idx = Math.Clamp((t - coast) / (burn / nb), 0.0, nb);
            int ab = Math.Min((int)idx, nb - 1);
            fr = idx - ab;
            a = c + ab;
        }
        var r = new double[width];
        for (int i = 0; i < width; i++)
            r[i] = v[a * width + i] * (1 - fr) + v[(a + 1) * width + i] * fr;
        return r;
    }

    /// <summary>The plan moved on by elapsed seconds, on the same nodes: the coast shortened (to zero once past), the burn shortened once in it.</summary>
    private static void Shift(Plan p, double elapsed, int n, int c, out double[] sx, out double[] su, out double ss, out double sc)
    {
        sc = c > 0 ? Math.Max(p.Coast - elapsed, 0.0) : 0.0;
        double intoBurn = Math.Max(elapsed - p.Coast, 0.0);
        ss = Math.Max(p.Burn - intoBurn, 0.3);
        sx = new double[n * NX];
        su = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double tk = elapsed + Scvx6DofSolver.NodeTime(k, n, c, ss, sc);
            if (c > 0 && k >= c)
                tk = Math.Max(tk, p.Coast);   // the burn's nodes from the old burn, never the coast's
            Array.Copy(Sample(p.X, NX, n, c, p.Burn, p.Coast, tk, burnSide: k >= c), 0, sx, k * NX, NX);
            Array.Copy(Sample(p.U, NU, n, c, p.Burn, p.Coast, tk, burnSide: k >= c), 0, su, k * NU, NU);
        }
    }

    /// <summary>The RCS torque that takes out the body rates over half a second (ColdSeed6Dof's coast).</summary>
    private static void Damp(double[] x, Dynamics6Dof.Params dyn, double[] u)
    {
        Array.Clear(u);
        for (int a = 0; a < Dynamics6Dof.NRCS; a++)
        {
            double inertia = a == 0 ? dyn.Ixx : a == 1 ? dyn.Iyy : dyn.Izz;
            u[Dynamics6Dof.IRCS + a] = Math.Clamp(-inertia * x[Dynamics6Dof.IW + a] / 0.5, -dyn.Rcs!.MaxNegNm[a], dyn.Rcs.MaxPosNm[a]);
        }
    }

    /// <summary>The plant: SixDofRcsCheck's, plus the push.</summary>
    private static void Step(double[] x, double[] u, Dynamics6Dof.Params p, double h)
    {
        SixDofRcsCheck.Step(x, u, p, h);
        for (int i = 0; i < 3; i++)
        {
            x[i] += 0.5 * Push[i] * h * h;
            x[3 + i] += Push[i] * h;
        }
    }
}

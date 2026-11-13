using System.Globalization;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// The cold solve at a braking-burn hand-over, offline, from a logged engage state: a lander with a thrust-to-weight ratio of 17 on an airless moon, 7.4 km up and 22 km uprange at 1100 m/s, 40 s from the pad, with its thrust axis 99 degrees off vertical.
///
/// The solve is run the way the spread cold solve runs it: one iteration at a time, the reference moved with the craft before each one, the ADMM iterations capped, and a restart from a new seed after six refused iterations in a row. An iteration takes the sim time its ADMM iterations cost in the game.
///
/// As flown, the seed was the straight line with hover thrust, a subproblem was cut at the 200 ms flight budget (about 1435 ADMM iterations at 50 nodes), and the craft flew on at the throttle and attitude the braking burn left. That case is printed for the record, from four engage heights a metre apart at 0.08 s per iteration, and is no pass criterion, because its outcome turns on such differences: it needs 16 to 56 iterations when it finds a plan, and now and then finds none in the ten seconds it has, as the flight found none in 240 dispatches.
///
/// What has to pass is the hand-over as it runs now: the braking seed (ColdSeed6Dof), a subproblem budget of one second, and the braking burn flying the craft while the solve runs. It has to give a plan that passes the flight gate within a few iterations, on the logged state and on the same approach at 43 % and 20 % of its range, speed and thrust.
///
/// Options for a single run: --seed-kind line|braking, --cap N, --fly held|burn, --moving 0|1, --stuck N, --n nodes, --sigma s, --max iterations, --iter-s s, --ms-per-admm ms, --state-scale f, --thrust-scale f.
/// </summary>
internal static class BrakingHandoverCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    internal static bool Verbose;

    // The craft and the state, as logged.
    private const double Tmax = 952331.4;
    private const double MinThrust = 94756.2;
    private const double Isp = 438.0;
    private const double Gravity = 1.62;
    private const double HeldThrottle = 0.882;

    internal sealed class Options
    {
        public string Seed = "line";
        public int Nodes = 50;
        public double Sigma = 40.0;
        public bool Moving = true;
        public int AdmmCap = 1435;
        public int StuckRefusals = 6;
        public double IterationS = 0.08;
        public int MaxIterations = 240;
        // The model the game built for the craft at this hand-over: effective gimbal range, roll torque at full thrust, engine arm and inertia.
        public double GimbalDeg = 2.0;
        public double Arm = 2.81;
        public double Inertia = 194648.0;
        public double InertiaZ = 95683.0;
        public double TauRoll = 26800.0;
        public double Scale = 1.0;
        public double MsPerAdmm = 0.14;
        public string Fly = "held";
        public double ThrustScale = 1.0;
        public double AltitudeOffset;
    }

    internal static int Run(string[] args)
    {
        if (!args.Any(arg => arg is "--seed-kind" or "--cap" or "--fly" or "--moving" or "--state-scale"))
            return Suite();
        var o = new Options();
        for (int i = 0; i < args.Length - 1; i++)
        {
            string v = args[i + 1];
            switch (args[i])
            {
                case "--seed-kind": o.Seed = v; break;
                case "--n": o.Nodes = int.Parse(v, CultureInfo.InvariantCulture); break;
                case "--sigma": o.Sigma = double.Parse(v, CultureInfo.InvariantCulture); break;
                case "--moving": o.Moving = v == "1"; break;
                case "--cap": o.AdmmCap = int.Parse(v, CultureInfo.InvariantCulture); break;
                case "--stuck": o.StuckRefusals = int.Parse(v, CultureInfo.InvariantCulture); break;
                case "--iter-s": o.IterationS = double.Parse(v, CultureInfo.InvariantCulture); break;
                case "--max": o.MaxIterations = int.Parse(v, CultureInfo.InvariantCulture); break;
                case "--gimbal": o.GimbalDeg = double.Parse(v, CultureInfo.InvariantCulture); break;
                case "--arm": o.Arm = double.Parse(v, CultureInfo.InvariantCulture); break;
                case "--inertia": o.Inertia = double.Parse(v, CultureInfo.InvariantCulture); break;
                case "--tau-roll": o.TauRoll = double.Parse(v, CultureInfo.InvariantCulture); break;
                case "--state-scale": o.Scale = double.Parse(v, CultureInfo.InvariantCulture); break;
                case "--fly": o.Fly = v; break;
                case "--alt-offset": o.AltitudeOffset = double.Parse(v, CultureInfo.InvariantCulture); break;
                case "--thrust-scale": o.ThrustScale = double.Parse(v, CultureInfo.InvariantCulture); break;
                case "--ms-per-admm": o.MsPerAdmm = double.Parse(v, CultureInfo.InvariantCulture); break;
            }
        }
        Result r = Solve(o);
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"seed {o.Seed}, {o.Nodes} nodes, sigma {o.Sigma:F0} s, moving {o.Moving}, cap {o.AdmmCap}: "
            + $"{(r.Converged ? "CONVERGED" : "NO PLAN")} after {r.Iterations} iterations ({r.Accepted} accepted, {r.Restarts} restarts), "
            + $"{r.SimSeconds:F1} s of flight, alt {r.Altitude:F0} m, sigma {r.Sigma:F1} s, gate {r.GateRatio:F1}x on {r.GateChannel} at interval {r.GateNode}, {r.WallMs:F0} ms, {r.Admm} ADMM"));
        return r.Converged ? 0 : 1;
    }

    private static int Suite()
    {
        Console.WriteLine("6-DOF COLD SOLVE AT A BRAKING-BURN HAND-OVER (T/W 17, 7.4 km up, 22 km uprange, 1100 m/s, 40 s to go)");
        Console.WriteLine();
        Console.WriteLine("  seed       ADMM cap   craft flies     result");
        bool ok = SeedIsConsistent();

        // As flown, from four engage heights a metre apart, because the outcome turns on such differences.
        for (int offset = 0; offset < 4; offset++)
            Row("line", 1435, "held", 1.0, 0.0, 125, required: false, offset);
        Row("braking", 1435, "burn", 1.0, 0.14, 60, required: false);
        foreach (double scale in new[] { 1.0, 0.43, 0.2 })
            ok &= Row("braking", 7000, "burn", scale, 0.14, 40, required: true);

        ok &= BranchChangeKeepsThePlan();

        Console.WriteLine();
        Console.WriteLine(ok ? "PASS - the braking seed with a one-second subproblem budget gives a plan while the burn flies the craft"
                             : "FAIL - see above");
        return ok ? 0 : 1;
    }

    private static bool Row(string seed, int cap, string fly, double scale, double msPerAdmm, int max, bool required, double altitudeOffset = 0.0)
    {
        var o = new Options { Seed = seed, AdmmCap = cap, Fly = fly, Scale = scale, ThrustScale = scale, MsPerAdmm = msPerAdmm, MaxIterations = max, AltitudeOffset = altitudeOffset };
        Result r = Solve(o);
        // A plan in time for the hand-over: within ten seconds of the 40 the burn has left.
        bool pass = r.Converged && r.SimSeconds <= 10.0;
        string verdict = required ? (pass ? "ok  " : "FAIL") : "info";
        string head = FormattableString.Invariant($"  {seed,-9} {cap,9}   {(fly == "burn" ? "braking burn" : "held command"),-13}   {verdict} x{scale:F2}: ");
        string body = FormattableString.Invariant($"{(r.Converged ? "plan" : "NO PLAN")} after {r.Iterations} iterations ({r.Accepted} accepted, {r.Restarts} restarts), {r.SimSeconds:F1} s of flight, {r.Admm} ADMM");
        string gate = r.Converged ? "" : FormattableString.Invariant($", gate {r.GateRatio:F1}x on {r.GateChannel}");
        Console.WriteLine(head + body + gate);
        return pass || !required;
    }

    // The braking seed starts at the measured state, ends on the target, keeps unit attitudes, and asks for thrust inside the engine's range that explains its own velocity change.
    private static bool SeedIsConsistent()
    {
        var o = new Options();
        double[] x0 = EngageState();
        var xf = new double[NX];
        xf[2] = 6.0; xf[6] = 1.0;
        (Scvx6DofConfig cfg, Dynamics6Dof.Params dyn) = Model(o, x0, xf);
        int n = o.Nodes;
        ColdSeed6Dof.Build(x0, xf, o.Sigma, n, cfg, dyn, out double[] xs, out double[] us);

        bool ok = true;
        for (int i = 0; i < NX; i++)
            ok &= xs[i] == x0[i];
        for (int i = 0; i < 6; i++)
            ok &= Math.Abs(xs[(n - 1) * NX + i] - xf[i]) < 1e-6;
        double worstDefect = 0.0;
        Span<double> f0 = stackalloc double[NX], f1 = stackalloc double[NX];
        double dt = o.Sigma / (n - 1);
        for (int k = 0; k < n; k++)
        {
            double norm = 0.0;
            for (int i = 6; i < 10; i++) norm += xs[k * NX + i] * xs[k * NX + i];
            ok &= Math.Abs(norm - 1.0) < 1e-9;
            ok &= us[k * NU + 2] >= cfg.Tmin - 1e-6 && us[k * NU + 2] <= cfg.Tmax + 1e-6;
            if (k == n - 1 || k >= (int)(0.8 * n))
                continue;
            // The trapezoid defect of the velocity before the turn to upright, where the thrust points along the path's own acceleration.
            Dynamics6Dof.Eval(xs.AsSpan(k * NX, NX), us.AsSpan(k * NU, NU), dyn, f0);
            Dynamics6Dof.Eval(xs.AsSpan((k + 1) * NX, NX), us.AsSpan((k + 1) * NU, NU), dyn, f1);
            for (int i = 3; i < 6; i++)
                worstDefect = Math.Max(worstDefect, Math.Abs(xs[(k + 1) * NX + i] - xs[k * NX + i] - 0.5 * dt * (f0[i] + f1[i])));
        }
        ok &= worstDefect < 2.0;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  braking seed: starts on the state, ends on the target, unit attitudes, thrust in range, worst velocity defect {worstDefect:F2} m/s per interval: {(ok ? "ok" : "FAIL")}"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  the burn asks for {ColdSeed6Dof.BrakingLoad(x0, xf, o.Sigma, dyn):F1} times the hover thrust that the straight-line seed holds"));
        return ok;
    }

    // The measured attitude changes its quaternion sign in flight, which is the same rotation. The plan follows it onto the other branch, and the re-solve from there must still find a step, with the end attitude on that branch too.
    private static bool BranchChangeKeepsThePlan()
    {
        var o = new Options { Seed = "braking", AdmmCap = 7000 };
        double[] x0 = EngageState();
        var xf = new double[NX];
        xf[2] = 6.0; xf[6] = 1.0;
        (Scvx6DofConfig cfg, Dynamics6Dof.Params dyn) = Model(o, x0, xf);
        var solver = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = 3e-4 };
        solver.MaxSubproblemIterations = o.AdmmCap;
        solver.EscalatedSubproblemIterations = o.AdmmCap;
        BuildSeed(o.Seed, x0, xf, o.Sigma, o.Nodes, cfg, dyn, out double[] xs, out double[] us);
        solver.Initialize(x0, xf, xs, us, o.Sigma);
        int before = 0;
        for (int i = 0; i < 4; i++)
            if (solver.Iterate().Accepted) before++;

        xs = (double[])solver.ReferenceX.Clone();
        us = (double[])solver.ReferenceU.Clone();
        for (int k = 0; k < o.Nodes; k++)
            for (int i = 0; i < 4; i++)
                xs[k * NX + Dynamics6Dof.IQ + i] = -xs[k * NX + Dynamics6Dof.IQ + i];
        solver.Reseed(xs.AsSpan(0, NX), xs, us, solver.Sigma, trustRegion: solver.TrustRegion);
        ScvxIteration it = solver.Iterate();
        double end = 0.0;
        for (int i = 0; i < 4; i++)
            end += solver.ReferenceX[(o.Nodes - 1) * NX + Dynamics6Dof.IQ + i] * xf[Dynamics6Dof.IQ + i];
        bool ok = before > 0 && it.Solved && end < -0.99;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  attitude sign change: {before} accepted steps before it, then subproblem solved {it.Solved}, step accepted {it.Accepted}, end attitude dot target {end:F3}: {(ok ? "ok" : "FAIL " + solver.LastFailureReason)}"));
        return ok;
    }

    internal readonly record struct Result(bool Converged, int Iterations, int Accepted, int Restarts, double SimSeconds,
                                           double Altitude, double Sigma, double GateRatio, string GateChannel, int GateNode, double WallMs, int Admm);

    internal static double[] EngageState(double scale = 1.0)
    {
        var x0 = new double[NX];
        x0[0] = -22055.1 * scale; x0[2] = 6.0 + (7367.3 - 6.0) * scale;
        x0[3] = 1101.6 * scale; x0[5] = -205.8 * scale;
        // Thrust axis 9.3 degrees below the horizon, against the velocity: a turn of -99.3 degrees about the site frame's Y.
        double half = 0.5 * -99.3 * Math.PI / 180.0;
        x0[6] = Math.Cos(half); x0[8] = Math.Sin(half);
        x0[13] = 33100.0;
        return x0;
    }

    internal static (Scvx6DofConfig cfg, Dynamics6Dof.Params dyn) Model(Options o, double[] x0, double[] xf)
    {
        double length = Math.Sqrt((x0[0] - xf[0]) * (x0[0] - xf[0]) + (x0[1] - xf[1]) * (x0[1] - xf[1]) + (x0[2] - xf[2]) * (x0[2] - xf[2]));
        double speed = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
        double velocity = Math.Max(Math.Max(speed, Math.Sqrt(length * Gravity)), 1.0);
        const double tiltMax = 120.0;
        double omegaScale = Math.Max(tiltMax * Math.PI / 180.0 / Math.Max(o.Sigma * 0.5, 1.0), 1e-3);
        var cfg = new Scvx6DofConfig
        {
            Nodes = o.Nodes,
            Tmax = Tmax * o.ThrustScale,
            // The plan's floor: the engines' minimum throttle plus the 0.05 margin the tracker works in.
            ThrottleFloor = MinThrust / Tmax + 0.05,
            GimbalMaxDeg = o.GimbalDeg,
            TauRollMax = o.TauRoll,
            RollTorqueFollowsThrust = true,
            TiltMaxDeg = tiltMax,
            GroundFloor = -1.0,
            WW = 0.002 / (o.Nodes * 3.0 * omegaScale * omegaScale),
            WDu = 0.05,
            ProximalWeight = 0.05,
            GlideSlopeDeg = 10.0,
            VzMax = 0.0,
            TerminalMissWeight = 1.0e5,
            TerminalSpeedWeight = 1.0e4,
            SigmaMin = o.Sigma * 0.15,
            SigmaMax = o.Sigma * 4.0,
            SigmaScale = o.Sigma,
            XScale = [length, length, length, velocity, velocity, velocity, 1, 1, 1, 1, 1, 1, 1, x0[13]],
        };
        var dyn = new Dynamics6Dof.Params
        {
            Gx = 0.0, Gy = 0.0, Gz = -Gravity,
            G0 = 9.80665, Isp = Isp,
            LArm = o.Arm, Ixx = o.Inertia, Iyy = o.Inertia, Izz = o.InertiaZ,
        };
        return (cfg, dyn);
    }

    internal static Result Solve(Options o)
    {
        double[] x0 = EngageState(o.Scale);
        x0[2] += o.AltitudeOffset;
        var xf = new double[NX];
        xf[2] = 6.0; xf[6] = 1.0;
        (Scvx6DofConfig cfg, Dynamics6Dof.Params dyn) = Model(o, x0, xf);
        int n = o.Nodes;

        var solver = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = 3e-4 };
        solver.MaxSubproblemIterations = o.AdmmCap;
        solver.EscalatedSubproblemIterations = o.AdmmCap;

        double[] state = (double[])x0.Clone();
        void Begin(double sigma)
        {
            BuildSeed(o.Seed, state, xf, sigma, n, cfg, dyn, out double[] xs, out double[] us);
            solver.Initialize(state, xf, xs, us, sigma);
        }
        Begin(o.Sigma);

        // The warm gate of the flying loop: 1 m, 0.5 m/s, 1 degree, 0.01 rad/s and a thousandth of the mass over the first second, loosening to 25 times that at the end of the horizon.
        double[] tol = new double[NX];
        for (int i = 0; i < 3; i++) { tol[i] = 1.0; tol[3 + i] = 0.5; tol[10 + i] = 0.01; }
        for (int i = 0; i < 4; i++) tol[6 + i] = Math.Sin(0.5 * Math.PI / 180.0);
        tol[13] = x0[13] * 1e-3;

        // The held command: the thrust the braking burn left, along the body axis, with no torque.
        double[] held = [0.0, 0.0, HeldThrottle * cfg.Tmax, 0.0];
        // The braking burn that keeps flying the craft while the solve runs: the path that brings it to rest over the pad in the burn time.
        const double BurnDt = 0.02;
        int burnNodes = (int)Math.Round(o.Sigma / BurnDt) + 1;
        double[] burnXf = (double[])xf.Clone();
        burnXf[2] = 106.0; burnXf[5] = -20.0;
        ColdSeed6Dof.Build(x0, burnXf, o.Sigma, burnNodes, cfg, dyn, out double[] burn, out _);
        var sw = System.Diagnostics.Stopwatch.StartNew();
        int accepted = 0, refusedRun = 0, restarts = 0, iterations = 0;
        double ratio = double.PositiveInfinity, t = 0.0, lastStepS = o.IterationS;
        int admmTotal = 0;
        int channel = -1, node = -1;
        bool converged = false;
        for (; iterations < o.MaxIterations && state[2] > 50.0; iterations++)
        {
            if (iterations > 0)
            {
                if (o.Moving)
                {
                    // The craft flies on for as long as the iteration before took in the game: the dispatch interval, or the solve itself when that is longer.
                    if (o.Fly == "burn")
                    {
                        int at = Math.Min((int)Math.Round((t + lastStepS) / BurnDt), burnNodes - 1);
                        Array.Copy(burn, at * NX, state, 0, NX);
                    }
                    else
                        for (double left = lastStepS; left > 1e-9; left -= 0.02)
                            Integrate(state, held, dyn, Math.Min(left, 0.02));
                    t += lastStepS;
                }
                double[] xs = (double[])solver.ReferenceX.Clone();
                double[] us = (double[])solver.ReferenceU.Clone();
                ColdReferenceShift.Apply(xs, n, solver.Sigma, state);
                solver.Reseed(state, xs, us, solver.Sigma, trustRegion: solver.TrustRegion);
            }
            ScvxIteration it = solver.Iterate();
            lastStepS = Math.Max(o.IterationS, it.SolverIterations * o.MsPerAdmm / 1000.0);
            admmTotal += it.SolverIterations;
            if (it.Accepted) accepted++;
            refusedRun = it.Accepted ? 0 : refusedRun + 1;
            if (it.Accepted)
            {
                double dt = solver.Sigma / (n - 1);
                ratio = solver.WeightedDefect(tol, (int)Math.Floor(1.0 / dt), 25.0, out channel, out node, out _);
            }
            if (Verbose)
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                    $"  {iterations,3} t {t,5:F1} alt {state[2],6:F0}  solved {it.Solved,-5} accepted {it.Accepted,-5} rho {it.Rho,8:G3} trust {it.TrustRegion,7:G3} sigma {it.Sigma,5:F1} "
                    + $"admm {it.SolverIterations,5} defect {it.DefectNorm * cfg.XScale[0],9:F1} m on {Scvx6DofSolver.ChannelName(it.DefectChannel)}@{it.DefectNode} gate {ratio,8:F1}x {Scvx6DofSolver.ChannelName(channel)}@{node}  {(it.Solved ? "" : solver.LastFailureReason)}"));
            if (it.Accepted && ratio <= 1.0)
            {
                converged = true;
                iterations++;
                break;
            }
            if (refusedRun >= o.StuckRefusals)
            {
                Begin(Math.Clamp(solver.Sigma, cfg.SigmaMin, cfg.SigmaMax));
                refusedRun = 0;
                restarts++;
                ratio = double.PositiveInfinity;
                if (Verbose)
                    Console.WriteLine("  -- stuck, starting again from a new seed");
            }
        }
        t += lastStepS;
        return new Result(converged, iterations, accepted, restarts, t, state[2], solver.Sigma, ratio,
                          Scvx6DofSolver.ChannelName(channel), node, sw.Elapsed.TotalMilliseconds, admmTotal);
    }

    private static void BuildSeed(string kind, double[] x0, double[] xf, double sigma, int n, Scvx6DofConfig cfg, Dynamics6Dof.Params dyn,
                                  out double[] xs, out double[] us)
    {
        if (kind == "braking")
        {
            ColdSeed6Dof.Build(x0, xf, sigma, n, cfg, dyn, out xs, out us);
            return;
        }
        // The straight line of Ksa6DofGuidance.BuildColdSeed.
        xs = new double[n * NX];
        us = new double[n * NU];
        double m0 = x0[13];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xs[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xs[k * NX + 3 + i] = x0[3 + i] + t * (xf[3 + i] - x0[3 + i]);
            }
            ColdSeed6Dof.Slerp(x0.AsSpan(6, 4), xf.AsSpan(6, 4), t, xs.AsSpan(k * NX + 6, 4));
            for (int i = 0; i < 3; i++)
                xs[k * NX + 10 + i] = x0[10 + i] * (1.0 - t);
            xs[k * NX + 13] = m0 * (1.0 - 0.08 * t);
            us[k * NU + 2] = 1.05 * m0 * Math.Abs(dyn.Gz);
        }
        Array.Copy(x0, 0, xs, 0, NX);
    }

    // RK4 on the true dynamics, quaternion renormalised.
    private static void Integrate(double[] x, double[] u, Dynamics6Dof.Params p, double dt)
    {
        Span<double> k1 = stackalloc double[NX], k2 = stackalloc double[NX], k3 = stackalloc double[NX], k4 = stackalloc double[NX];
        Span<double> tmp = stackalloc double[NX];
        Dynamics6Dof.Eval(x, u, p, k1);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * dt * k1[i];
        Dynamics6Dof.Eval(tmp, u, p, k2);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * dt * k2[i];
        Dynamics6Dof.Eval(tmp, u, p, k3);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + dt * k3[i];
        Dynamics6Dof.Eval(tmp, u, p, k4);
        for (int i = 0; i < NX; i++) x[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        double norm = Math.Sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
        for (int i = 6; i < 10; i++) x[i] /= norm;
    }
}

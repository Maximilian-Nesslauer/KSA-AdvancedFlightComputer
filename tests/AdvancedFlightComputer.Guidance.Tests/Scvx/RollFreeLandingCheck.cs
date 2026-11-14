using System.Globalization;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// The touchdown plan and its flying loop for a craft whose gimbals give no roll torque, offline: a 0.9 t lander with one gimballed engine on the thrust line among six fixed ones, on an airless moon, 1.8 km up and 3.8 km uprange at 208 m/s, thrust axis 62 degrees off vertical, rolling at up to 0.09 rad/s when touchdown guidance takes it.
///
/// Such a craft keeps its roll rate. A plan that must end at a fixed heading with no roll rate can reach that only through the coupling of the pitch and yaw rates, so its subproblems are badly conditioned: the conic solver runs to its iteration cap, a re-solve takes seconds, and the loop loses the craft. With the end roll left free (Scvx6DofConfig.TerminalRollFree) and a reference that rolls with the craft (ColdSeed6Dof.KeepRoll), the same loop lands it.
///
/// The loop is the flying loop reduced to its solver side: the plant flies the published plan's controls on the true dynamics with the lateral thrust held to the real gimbal range and no roll torque, each re-solve is anchored at the state it was dispatched from and lands after the sim time its conic iterations cost, and a plan is published only when it passes the flight gate.
///
/// The pinned end state is printed for the record and is no pass criterion. What has to pass: the rolling reference keeps every thrust axis, a cold plan with the roll free ends upright with the roll rate it entered with, and the loop lands the craft from three entry rates.
///
/// Options for a single run: --rates wx,wy,wz, --roll-free 0|1, --gimbal deg (the model's), --fly-gimbal deg (the plant's), --n nodes, --sigma s, --cap N, --warm-cap N, --verbose.
/// </summary>
internal static class RollFreeLandingCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    private const double Tmax = 6393.5;
    private const double MinThrust = 636.9;
    private const double Isp = 318.0;
    private const double Gravity = 1.62;
    // What one conic iteration costs in the game at 50 nodes, ms.
    private const double MsPerAdmm = 0.125;
    private const double Cadence = 0.1;
    private const double Step = 0.02;

    internal sealed class Options
    {
        public int Nodes = 50;
        public double Sigma = 40.0;
        public int AdmmCap = 7000;
        public int WarmCap = 9000;
        public int MaxIterations = 20;
        // The gimbal range the craft has per newton of all its thrust: one engine with 4 degrees and 27 % of the thrust.
        public double GimbalDeg = 1.09;
        public double PlantGimbalDeg = 1.09;
        public double[] Rates = [0.0023, -0.0016, 0.062];
        public bool RollFree = true;
        public bool Verbose;
    }

    internal readonly record struct Result(bool Converged, bool Landed, double MissM, double SidewaysMs, double TiltDeg, double RollRate,
                                           int Published, int Refused, int Solves, int AdmmPerSolve, double OldestPlanS, double EndRollRate, double EndTiltDeg);

    internal static int Run(string[] args)
    {
        if (!args.Any(arg => arg is "--rates" or "--roll-free" or "--gimbal" or "--fly-gimbal"))
            return Suite();
        var o = new Options { Verbose = args.Contains("--verbose") };
        for (int i = 0; i < args.Length - 1; i++)
        {
            string v = args[i + 1];
            switch (args[i])
            {
                case "--n": o.Nodes = int.Parse(v, CultureInfo.InvariantCulture); break;
                case "--sigma": o.Sigma = double.Parse(v, CultureInfo.InvariantCulture); break;
                case "--cap": o.AdmmCap = int.Parse(v, CultureInfo.InvariantCulture); break;
                case "--warm-cap": o.WarmCap = int.Parse(v, CultureInfo.InvariantCulture); break;
                case "--gimbal": o.GimbalDeg = double.Parse(v, CultureInfo.InvariantCulture); break;
                case "--fly-gimbal": o.PlantGimbalDeg = double.Parse(v, CultureInfo.InvariantCulture); break;
                case "--roll-free": o.RollFree = v == "1"; break;
                case "--rates": o.Rates = v.Split(',').Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray(); break;
            }
        }
        Result r = Fly(o);
        Console.WriteLine(Describe(o, r));
        return r.Landed ? 0 : 1;
    }

    private static int Suite()
    {
        Console.WriteLine("6-DOF TOUCHDOWN FOR A CRAFT WITHOUT ROLL TORQUE (0.9 t, one gimballed engine among seven, 1.8 km up, 208 m/s, rolling)");
        Console.WriteLine();
        bool ok = SeedKeepsTheThrustAxes();
        ok &= GfoldSeedRollsToo();
        ok &= ColdPlanKeepsTheRoll();

        Console.WriteLine();
        Console.WriteLine("  end roll   entry rates (rad/s)        flying loop");
        Row(new Options { RollFree = false }, required: false);
        foreach (double[] rates in new[] { new[] { -0.092, 0.06, 0.0856 }, new[] { -0.062, -0.069, 0.063 }, new[] { 0.05, 0.05, -0.09 } })
            ok &= Row(new Options { Rates = rates }, required: true);

        Console.WriteLine();
        Console.WriteLine(ok ? "PASS - with the end roll free and a reference that rolls with the craft, the loop lands a craft that cannot stop its roll"
                             : "FAIL - see above");
        return ok ? 0 : 1;
    }

    private static bool Row(Options o, bool required)
    {
        Result r = Fly(o);
        string verdict = required ? (r.Landed ? "ok  " : "FAIL") : "info";
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  {(o.RollFree ? "free  " : "pinned")}     {o.Rates[0],6:F3} {o.Rates[1],6:F3} {o.Rates[2],6:F3}   {verdict}: ") + Outcome(r));
        return r.Landed || !required;
    }

    private static string Describe(Options o, Result r) => string.Create(CultureInfo.InvariantCulture,
        $"end roll {(o.RollFree ? "free" : "pinned")}, model gimbal {o.GimbalDeg:F2} deg, plant {o.PlantGimbalDeg:F2} deg, rates {o.Rates[0]:F3} {o.Rates[1]:F3} {o.Rates[2]:F3}: ") + Outcome(r);

    private static string Outcome(Result r) => !r.Converged
        ? "no cold plan"
        : string.Create(CultureInfo.InvariantCulture,
            $"{(r.Landed ? "landed" : "LOST")}, {r.MissM:F1} m from the pad, {r.SidewaysMs:F1} m/s sideways, tilt {r.TiltDeg:F1} deg, roll rate {r.RollRate:F3} rad/s; "
            + $"{r.Published} of {r.Solves} re-solves published, {r.AdmmPerSolve} conic iterations per re-solve, oldest plan flown {r.OldestPlanS:F1} s");

    // The rolling reference turns each node about its own thrust axis, so no thrust axis moves, every attitude stays a unit and every node carries the entry roll rate.
    private static bool SeedKeepsTheThrustAxes()
    {
        var o = new Options { Rates = [-0.092, 0.06, 0.0856] };
        double[] x0 = EngageState(o);
        double[] xf = EndState(x0);
        (Scvx6DofConfig cfg, Dynamics6Dof.Params dyn) = Model(o, x0, xf);
        ColdSeed6Dof.Build(x0, xf, o.Sigma, o.Nodes, cfg, dyn, out double[] plain, out _);
        double[] rolling = (double[])plain.Clone();
        ColdSeed6Dof.KeepRoll(rolling, o.Nodes, o.Sigma);

        double worstAxis = 0.0, worstNorm = 0.0, worstRate = 0.0, endTurn = 0.0;
        for (int k = 0; k < o.Nodes; k++)
        {
            Span<double> a = plain.AsSpan(k * NX + 6, 4), b = rolling.AsSpan(k * NX + 6, 4);
            double dot = 0.0, norm = 0.0;
            ThrustAxis(a, out double ax, out double ay, out double az);
            ThrustAxis(b, out double bx, out double by, out double bz);
            dot = ax * bx + ay * by + az * bz;
            for (int i = 0; i < 4; i++) norm += b[i] * b[i];
            worstAxis = Math.Max(worstAxis, Math.Abs(1.0 - dot));
            worstNorm = Math.Max(worstNorm, Math.Abs(1.0 - norm));
            worstRate = Math.Max(worstRate, Math.Abs(rolling[k * NX + 12] - x0[12]));
            if (k == o.Nodes - 1)
            {
                double same = Math.Abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
                endTurn = 2.0 * Math.Acos(Math.Clamp(same, 0.0, 1.0));
            }
        }
        // The last node has turned by the roll rate times the burn time, read modulo a full turn.
        double expected = Math.Abs(Math.IEEERemainder(x0[12] * o.Sigma, 2.0 * Math.PI));
        // The seed's attitudes are units to about 1e-7, and the turn keeps what it is given.
        bool ok = worstAxis < 1e-5 && worstNorm < 1e-5 && worstRate < 1e-12 && Math.Abs(endTurn - expected) < 1e-5;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  rolling reference: thrust axes kept to {worstAxis:E1}, unit attitudes, roll rate carried, last node turned {endTurn:F3} rad for {expected:F3}: {(ok ? "ok" : "FAIL")}"));
        return ok;
    }

    // The seed from G-FOLD gets the same rolling reference: with the end roll free every node carries the entry roll rate on the thrust axes G-FOLD gave and the roll angle goes on from the measured one, and with the end pinned the seed is as G-FOLD made it.
    private static bool GfoldSeedRollsToo()
    {
        var o = new Options { Rates = [-0.092, 0.06, 0.0856] };
        double[] x0 = EngageState(o);
        double[] xf = EndState(x0);
        (Scvx6DofConfig free, Dynamics6Dof.Params dyn) = Model(o, x0, xf);
        (Scvx6DofConfig pinned, _) = Model(new Options { Rates = o.Rates, RollFree = false }, x0, xf);
        if (!Ksa6DofGfoldSeed.TryBuild(x0, xf, pinned, dyn, o.Nodes, out double[] plain, out _, out double sigma, out string note)
            || !Ksa6DofGfoldSeed.TryBuild(x0, xf, free, dyn, o.Nodes, out double[] rolling, out _, out _, out note))
        {
            Console.WriteLine("  G-FOLD seed: FAIL, no seed (" + note + ")");
            return false;
        }

        double worstAxis = 0.0, worstRate = 0.0, plainRate = 0.0, worstRoll = 0.0, plainRoll = 0.0;
        double rollPerNode = x0[12] * sigma / (o.Nodes - 1);
        for (int k = 1; k < o.Nodes; k++)
        {
            ThrustAxis(plain.AsSpan(k * NX + 6, 4), out double ax, out double ay, out double az);
            ThrustAxis(rolling.AsSpan(k * NX + 6, 4), out double bx, out double by, out double bz);
            worstAxis = Math.Max(worstAxis, Math.Abs(1.0 - (ax * bx + ay * by + az * bz)));
            worstRate = Math.Max(worstRate, Math.Abs(rolling[k * NX + 12] - x0[12]));
            plainRate = Math.Max(plainRate, Math.Abs(plain[k * NX + 12]));
            worstRoll = Math.Max(worstRoll, Math.Abs(RollBetween(rolling.AsSpan((k - 1) * NX + 6, 4), rolling.AsSpan(k * NX + 6, 4)) - rollPerNode));
            if (k == 1)
                plainRoll = Math.Abs(RollBetween(plain.AsSpan(0, NX).Slice(6, 4), plain.AsSpan(NX + 6, 4)));
        }
        bool ok = worstAxis < 1e-5 && worstRate < 1e-12 && plainRate < 0.5 * Math.Abs(x0[12]) && worstRoll < 0.02;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  G-FOLD seed ({sigma:F1} s): with the roll free the thrust axes are kept to {worstAxis:E1}, every node carries the roll rate, and the roll angle from one node to the next is within {worstRoll:F4} rad of the {rollPerNode:F4} rad the roll rate covers, from the measured attitude on; with the roll pinned the largest roll rate of a node is {plainRate:F4} rad/s and the first node is {plainRoll:F2} rad of roll from the measured attitude: {(ok ? "ok" : "FAIL")}"));
        return ok;
    }

    // The turn about the body Z axis in the rotation from attitude a to attitude b, rad.
    private static double RollBetween(ReadOnlySpan<double> a, ReadOnlySpan<double> b)
    {
        // d = conj(a) b, scalar-first.
        double w = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
        double z = a[0] * b[3] - a[3] * b[0] - a[1] * b[2] + a[2] * b[1];
        double angle = 2.0 * Math.Atan2(z, w);
        if (angle > Math.PI) angle -= 2.0 * Math.PI;
        if (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    // A cold plan with the end roll free ends upright and at rest in pitch and yaw, and still has the roll rate it entered with.
    private static bool ColdPlanKeepsTheRoll()
    {
        var o = new Options();
        double[] x0 = EngageState(o);
        double[] xf = EndState(x0);
        (Scvx6DofConfig cfg, Dynamics6Dof.Params dyn) = Model(o, x0, xf);
        Scvx6DofSolver solver = Cold(o, cfg, dyn, x0, xf, out bool converged, out int iterations);
        int last = (o.Nodes - 1) * NX;
        double[] x = solver.ReferenceX;
        double tilt = 2.0 * Math.Asin(Math.Min(Math.Sqrt(x[last + 7] * x[last + 7] + x[last + 8] * x[last + 8]), 1.0)) * 180.0 / Math.PI;
        double lateral = Math.Sqrt(x[last + 10] * x[last + 10] + x[last + 11] * x[last + 11]);
        double roll = x[last + 12];
        bool ok = converged && tilt < 0.01 && lateral < 1e-4 && Math.Abs(roll - x0[12]) < 0.02;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  cold plan, roll free: {(converged ? $"passes the flight gate after {iterations} iterations" : "NO PLAN")}, ends {tilt:F4} deg off upright with {lateral:E1} rad/s of pitch and yaw rate and a roll rate of {roll:F3} for {x0[12]:F3} at the start: {(ok ? "ok" : "FAIL")}"));
        return ok;
    }

    private static void ThrustAxis(ReadOnlySpan<double> q, out double x, out double y, out double z)
    {
        x = 2.0 * (q[1] * q[3] + q[0] * q[2]);
        y = 2.0 * (q[2] * q[3] - q[0] * q[1]);
        z = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    }

    internal static double[] EngageState(Options o) =>
    [
        -3774.38, 4.29388, 1826.64, 196.263, 0.013709, -70.0317,
        0.394249, -0.45946, -0.239952, 0.758872,
        o.Rates[0], o.Rates[1], o.Rates[2], 915.522,
    ];

    // Upright over the pad at the heading the craft has now, as the game sets the end state.
    private static double[] EndState(double[] x0)
    {
        var xf = new double[NX];
        xf[2] = 1.35;
        double yaw = Math.Sqrt(x0[6] * x0[6] + x0[9] * x0[9]);
        xf[6] = x0[6] / yaw;
        xf[9] = x0[9] / yaw;
        return xf;
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
            Tmax = Tmax,
            ThrottleFloor = MinThrust / Tmax + 0.05,
            GimbalMaxDeg = o.GimbalDeg,
            // What the allocator hands back about the thrust axis for a gimbal on it.
            TauRollMax = 1e-3,
            RollTorqueFollowsThrust = true,
            TerminalRollFree = o.RollFree,
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
            VirtualControlScale = [0, 0, 0, 5, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0],
            XScale = [length, length, length, velocity, velocity, velocity, 1, 1, 1, 1, 1, 1, 1, x0[13]],
        };
        var dyn = new Dynamics6Dof.Params
        {
            Gx = 0.0, Gy = 0.0, Gz = -Gravity,
            G0 = 9.80665, Isp = Isp,
            LArm = 0.52, Ixx = 573.0, Iyy = 643.0, Izz = 797.0,
        };
        return (cfg, dyn);
    }

    // The warm gate of the flying loop: 1 m, 0.5 m/s, 1 degree, 0.01 rad/s and a thousandth of the mass over the first second, loosening to 25 times that at the end of the horizon.
    private static double[] Tolerances(double mass)
    {
        double[] tol = new double[NX];
        for (int i = 0; i < 3; i++) { tol[i] = 1.0; tol[3 + i] = 0.5; tol[10 + i] = 0.01; }
        for (int i = 0; i < 4; i++) tol[6 + i] = Math.Sin(0.5 * Math.PI / 180.0);
        tol[13] = mass * 1e-3;
        return tol;
    }

    private static Scvx6DofSolver Cold(Options o, Scvx6DofConfig cfg, Dynamics6Dof.Params dyn, double[] x0, double[] xf, out bool converged, out int iterations)
    {
        int n = o.Nodes;
        var solver = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = 3e-4 };
        solver.MaxSubproblemIterations = o.AdmmCap;
        solver.EscalatedSubproblemIterations = o.AdmmCap;
        ColdSeed6Dof.Build(x0, xf, o.Sigma, n, cfg, dyn, out double[] xs, out double[] us);
        if (o.RollFree)
            ColdSeed6Dof.KeepRoll(xs, n, o.Sigma);
        solver.Initialize(x0, xf, xs, us, o.Sigma);
        double[] tol = Tolerances(x0[13]);
        converged = false;
        for (iterations = 0; iterations < o.MaxIterations && !converged;)
        {
            ScvxIteration it = solver.Iterate();
            iterations++;
            double ratio = double.PositiveInfinity;
            if (it.Accepted)
                ratio = solver.WeightedDefect(tol, (int)Math.Floor(1.0 / (solver.Sigma / (n - 1))), 25.0, out _, out _, out _);
            if (o.Verbose)
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                    $"    cold {iterations,2}: solved {it.Solved,-5} accepted {it.Accepted,-5} trust {it.TrustRegion,6:G3} sigma {it.Sigma,5:F1} conic {it.SolverIterations,5} gate {ratio,7:F2}x"));
            converged = it.Accepted && ratio <= 1.0;
        }
        return solver;
    }

    internal static Result Fly(Options o)
    {
        double[] x0 = EngageState(o);
        double[] xf = EndState(x0);
        (Scvx6DofConfig cfg, Dynamics6Dof.Params dyn) = Model(o, x0, xf);
        int n = o.Nodes;
        Scvx6DofSolver solver = Cold(o, cfg, dyn, x0, xf, out bool converged, out _);
        if (!converged)
            return default;
        double[] tol = Tolerances(x0[13]);

        double[] pubX = (double[])solver.ReferenceX.Clone(), pubU = (double[])solver.ReferenceU.Clone();
        double pubSigma = solver.Sigma, pubTime = 0.0;
        double[] nextX = pubX, nextU = pubU;
        double nextSigma = 0.0;
        double[] state = (double[])x0.Clone();
        double time = 0.0, landsAt = double.NaN, dispatchedAt = 0.0, lastSolve = 0.0, oldest = 0.0;
        bool landsOk = false;
        int published = 0, refused = 0, failures = 0, admmTotal = 0, solves = 0;
        double plantTan = Math.Tan(o.PlantGimbalDeg * Math.PI / 180.0);
        solver.MaxSubproblemIterations = o.WarmCap;
        solver.EscalatedSubproblemIterations = o.WarmCap;
        var u = new double[NU];
        while (state[2] > xf[2] + 0.3 && time < 3.0 * o.Sigma)
        {
            if (!double.IsNaN(landsAt) && time >= landsAt)
            {
                if (landsOk)
                {
                    pubX = nextX; pubU = nextU; pubSigma = nextSigma; pubTime = dispatchedAt;
                    published++;
                    failures = 0;
                }
                else
                {
                    refused++;
                    failures++;
                }
                lastSolve = time;
                landsAt = double.NaN;
            }
            if (double.IsNaN(landsAt) && time - lastSolve >= Cadence - 1e-9 && pubSigma - (time - pubTime) > 0.5)
            {
                // From the published plan in the warm trust region, and once from the wide one after a refusal, as the flying loop re-solves.
                double sigma = Math.Max(cfg.SigmaMin, pubSigma - (time - pubTime));
                (double[] xs, double[] us) = Shift(pubX, pubU, n, pubSigma, time - pubTime, sigma, state);
                solver.Reseed(state, xs, us, sigma, trustRegion: 0.05);
                ScvxStatus status = solver.Solve(5);
                int admm = solver.Trace.Sum(t => t.SolverIterations);
                bool ok = Gate(status, solver, state, tol, n);
                if (!ok && failures < 2)
                {
                    solver.Reseed(state, xs, us, sigma, trustRegion: solver.TrustRegionMax);
                    status = solver.Solve(15);
                    admm += solver.Trace.Sum(t => t.SolverIterations);
                    ok = Gate(status, solver, state, tol, n);
                }
                admmTotal += admm;
                solves++;
                if (o.Verbose && solves <= 12)
                    Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                        $"    re-solve {solves,2} at {time,5:F2} s: {status}, {(ok ? "published" : "refused")}, {solver.Trace.Count(t => t.Accepted)} of {solver.Trace.Count} steps accepted, {admm} conic iterations"));
                landsOk = ok;
                dispatchedAt = time;
                landsAt = time + Math.Max(admm * MsPerAdmm / 1000.0, Step);
                if (ok)
                {
                    nextX = (double[])solver.ReferenceX.Clone();
                    nextU = (double[])solver.ReferenceU.Clone();
                    nextSigma = solver.Sigma;
                }
            }

            Sample(pubU, n, pubSigma, Math.Min(time - pubTime, pubSigma), u);
            // The plant's gimbal turns the thrust by no more than its range, and nothing on the craft makes roll torque.
            double lateral = Math.Sqrt(u[0] * u[0] + u[1] * u[1]);
            double limit = Math.Max(u[2], 0.0) * plantTan;
            if (lateral > limit && lateral > 0.0)
            {
                u[0] *= limit / lateral;
                u[1] *= limit / lateral;
            }
            u[3] = 0.0;
            Integrate(state, u, dyn, Step);
            time += Step;
            oldest = Math.Max(oldest, time - pubTime);
        }

        double miss = Math.Sqrt(state[0] * state[0] + state[1] * state[1]);
        double sideways = Math.Sqrt(state[3] * state[3] + state[4] * state[4]);
        double tilt = Math.Acos(Math.Clamp(1.0 - 2.0 * (state[7] * state[7] + state[8] * state[8]), -1.0, 1.0)) * 180.0 / Math.PI;
        // The loop has no thrust tracker and stops 0.3 m over the end of the plan, so the sink it ends with is not judged. Landed here is at the pad, upright, with no sideways speed, on plans that kept coming.
        bool landed = state[2] <= xf[2] + 0.3 && miss < 15.0 && sideways < 1.5 && tilt < 10.0 && refused * 10 <= solves;
        return new Result(true, landed, miss, sideways, tilt, state[12], published, refused, solves, admmTotal / Math.Max(solves, 1), oldest, state[12], tilt);
    }

    // The flight gate: anchored at the craft, and every channel within its tolerance.
    private static bool Gate(ScvxStatus status, Scvx6DofSolver s, double[] x0, double[] tol, int n)
    {
        if (status is ScvxStatus.Failed or ScvxStatus.TrustRegionCollapsed)
            return false;
        double[] r = s.ReferenceX;
        double anchor = Math.Sqrt((r[0] - x0[0]) * (r[0] - x0[0]) + (r[1] - x0[1]) * (r[1] - x0[1]) + (r[2] - x0[2]) * (r[2] - x0[2]));
        if (anchor > 1.0)
            return false;
        int commit = (int)Math.Floor(1.0 / Math.Max(s.Sigma / (n - 1), 1e-6));
        return s.WeightedDefect(tol, commit, 25.0, out _, out _, out _) <= 1.0;
    }

    // A trajectory resampled from `elapsed` onto a horizon of `sigmaNew`, on the measured attitude's quaternion branch, node 0 replaced by the measured state.
    private static (double[] xs, double[] us) Shift(double[] x, double[] u, int n, double sigma, double elapsed, double sigmaNew, double[] state)
    {
        var xs = new double[n * NX];
        var us = new double[n * NU];
        double dtOld = sigma / (n - 1), dtNew = sigmaNew / (n - 1);
        for (int k = 0; k < n; k++)
        {
            double s = Math.Clamp((elapsed + k * dtNew) / dtOld, 0.0, n - 1.0);
            int a = Math.Min((int)s, n - 2);
            double f = s - a;
            for (int i = 0; i < NX; i++)
                xs[k * NX + i] = x[a * NX + i] * (1 - f) + x[(a + 1) * NX + i] * f;
            for (int i = 0; i < NU; i++)
                us[k * NU + i] = u[a * NU + i] * (1 - f) + u[(a + 1) * NU + i] * f;
            double qn = 0;
            for (int i = 6; i < 10; i++) qn += xs[k * NX + i] * xs[k * NX + i];
            qn = Math.Sqrt(qn);
            if (qn > 1e-12) for (int i = 6; i < 10; i++) xs[k * NX + i] /= qn;
        }
        double sign = 0.0;
        for (int i = 6; i < 10; i++) sign += xs[i] * state[i];
        if (sign < 0.0)
            for (int k = 0; k < n; k++)
                for (int i = 6; i < 10; i++) xs[k * NX + i] = -xs[k * NX + i];
        Array.Copy(state, 0, xs, 0, NX);
        return (xs, us);
    }

    private static void Sample(double[] u, int n, double sigma, double t, double[] into)
    {
        double s = Math.Clamp(t / (sigma / (n - 1)), 0.0, n - 1.0);
        int a = Math.Min((int)s, n - 2);
        double f = s - a;
        for (int i = 0; i < NU; i++)
            into[i] = u[a * NU + i] * (1 - f) + u[(a + 1) * NU + i] * f;
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

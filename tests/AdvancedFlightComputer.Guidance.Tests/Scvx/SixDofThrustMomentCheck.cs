using System.Globalization;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// --6dof-thrust-moment: the engines' own moment in touchdown guidance's plan (Dynamics6Dof.Params.ThrustMoment*), as an off-centre landing engine choice makes it.
///
/// 1. Slopes: the thrust column of the angular rows against central differences with a moment set.
/// 2. A landing on an engine set whose thrust centre sits 0.8 m off the axis on a 15 m arm, its trim half of a 6 deg gimbal: planned with the moment, the plan holds the trim at every burning node (L tdy + m_x T = 0 within a share of the cone) and, flown open loop on the true dynamics, lands as the balanced craft does. The same plan made without the moment and flown on the true dynamics turns away and misses - what the model did before.
/// </summary>
internal static class SixDofThrustMomentCheck
{
    private const int NX = Dynamics6Dof.NX;

    private const double Tmax = 600e3, Floor = 0.3, Isp = 282.0, Gravity = 9.81, Mass = 26_000.0, Arm = 15.0, Length = 40.0;
    private const double GimbalDeg = 6.0;
    // The thrust centre's offset from the axis, model body x and y, m: the moment per newton is r x z = (dy, -dx, 0).
    private const double OffsetX = 0.48, OffsetY = 0.64;

    internal static int Run(bool verbose)
    {
        Console.WriteLine("THE ENGINES' OWN MOMENT IN TOUCHDOWN GUIDANCE'S PLAN");
        bool ok = Slopes();
        ok &= Landing(verbose);
        Console.WriteLine();
        Console.WriteLine(ok ? "PASS - the thrust moment in the touchdown plan" : "FAIL - see above");
        return ok ? 0 : 1;
    }

    private static Dynamics6Dof.Params Params(bool moment) => new()
    {
        Gx = 0, Gy = 0, Gz = -Gravity, G0 = 9.80665, Isp = Isp,
        LArm = Arm, Ixx = Mass * Length * Length / 12.0, Iyy = Mass * Length * Length / 12.0, Izz = 0.5 * Mass * 1.8 * 1.8,
        ThrustMomentX = moment ? OffsetY : 0.0,
        ThrustMomentY = moment ? -OffsetX : 0.0,
    };

    private static double[] State(double h, double[] v)
        => [-120, 40, h, v[0], v[1], v[2], 1, 0, 0, 0, 0, 0, 0, Mass];

    private static bool Slopes()
    {
        Console.WriteLine();
        Console.WriteLine("1. slopes: the thrust column of the angular rows against central differences");
        Dynamics6Dof.Params p = Params(moment: true);
        int nu = p.NU;
        double[] x = [10, -5, 400, 3, -2, -60, 0.99, 0.05, -0.08, 0.02, 0.01, -0.02, 0.03, Mass];
        double n = Math.Sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
        for (int i = 6; i < 10; i++) x[i] /= n;
        var u = new double[nu];
        u[Dynamics6Dof.ITDX] = 2e3;
        u[Dynamics6Dof.ITDY] = -3e3;
        u[Dynamics6Dof.IT] = 350e3;
        var A = new double[NX * NX];
        var B = new double[NX * nu];
        Dynamics6Dof.Jacobian(x, u, p, new double[NX], A, B);
        const double h = 1.0;
        double[] up = (double[])u.Clone(), um = (double[])u.Clone();
        up[Dynamics6Dof.IT] += h;
        um[Dynamics6Dof.IT] -= h;
        var fp = new double[NX];
        var fm = new double[NX];
        Dynamics6Dof.Eval(x, up, p, fp);
        Dynamics6Dof.Eval(x, um, p, fm);
        double worst = 0;
        for (int row = Dynamics6Dof.IW; row < Dynamics6Dof.IW + 3; row++)
        {
            double fd = (fp[row] - fm[row]) / (2 * h);
            worst = Math.Max(worst, Math.Abs(B[row * nu + Dynamics6Dof.IT] - fd) / Math.Max(Math.Abs(fd), 1e-12));
        }
        // And the moment itself: at rest, the angular acceleration is T m / I.
        var f0 = new double[NX];
        var rest = (double[])x.Clone();
        rest[10] = rest[11] = rest[12] = 0;
        var u0 = new double[nu];
        u0[Dynamics6Dof.IT] = 350e3;
        Dynamics6Dof.Eval(rest, u0, p, f0);
        double wx = 350e3 * OffsetY / p.Ixx, wy = -350e3 * OffsetX / p.Iyy;
        bool momentOk = Math.Abs(f0[Dynamics6Dof.IW] - wx) <= 1e-9 * Math.Abs(wx) + 1e-12
                        && Math.Abs(f0[Dynamics6Dof.IW + 1] - wy) <= 1e-9 * Math.Abs(wy) + 1e-12;
        bool pass = worst < 1e-6 && momentOk;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  thrust column worst {worst:E1}, rest acceleration {f0[Dynamics6Dof.IW]:E3} {f0[Dynamics6Dof.IW + 1]:E3} against {wx:E3} {wy:E3}  {(pass ? "ok" : "FAIL")}"));
        return pass;
    }

    private static bool Landing(bool verbose)
    {
        Console.WriteLine();
        double trimShare = Math.Sqrt(OffsetX * OffsetX + OffsetY * OffsetY) / Arm / Math.Tan(GimbalDeg * Math.PI / 180.0);
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"2. a landing with the thrust centre {Math.Sqrt(OffsetX * OffsetX + OffsetY * OffsetY):F2} m off the axis on a {Arm:F0} m arm: its trim takes {trimShare:P0} of a {GimbalDeg:F0} deg gimbal"));
        double[] x0 = State(600, [20, -6, -80]);
        double[] xf = new double[NX];
        xf[2] = 5.0;
        xf[6] = 1.0;

        Dynamics6Dof.Params balanced = Params(moment: false), offCentre = Params(moment: true);

        bool baseOk = Plan(balanced, x0, xf, verbose, out Scvx6DofSolver bs, out int bi);
        double[] bEnd = baseOk ? Fly(bs, balanced, x0) : [];
        double bMiss = baseOk ? Miss(bEnd, xf) : double.PositiveInfinity, bSpeed = baseOk ? Speed(bEnd) : double.PositiveInfinity;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  balanced: {(baseOk ? $"plan in {bi} iterations, {bs.Sigma:F1} s; flown open loop ends {bMiss:F1} m from its aim at {bSpeed:F1} m/s" : "NO PLAN  FAIL")}"));
        if (!baseOk)
            return false;

        bool ok = Plan(offCentre, x0, xf, verbose, out Scvx6DofSolver s, out int iterations);
        if (!ok)
        {
            Console.WriteLine($"  off centre, modelled: NO PLAN after {iterations} iterations  FAIL");
            return false;
        }
        int nu = offCentre.NU, n = s.ReferenceX.Length / NX;
        // THE TRIM, on average: the plan starts and ends near rest, so over the burn its gimbal torque must cancel the engines' moment, along the moment's direction, whatever it spends on its own turns in between.
        double mx = offCentre.ThrustMomentX, my = offCentre.ThrustMomentY, mn = Math.Sqrt(mx * mx + my * my);
        double held = 0.0, made = 0.0;
        for (int k = 0; k < n; k++)
        {
            double t = s.ReferenceU[k * nu + Dynamics6Dof.IT];
            if (!(t > 0.0))
                continue;
            double gx = Arm * s.ReferenceU[k * nu + Dynamics6Dof.ITDY], gy = -Arm * s.ReferenceU[k * nu + Dynamics6Dof.ITDX];
            held += -(gx * mx + gy * my) / mn;
            made += mn * t;
        }
        double ratio = made > 0.0 ? held / made : 0.0;
        double[] end = Fly(s, offCentre, x0);
        double miss = Miss(end, xf), speed = Speed(end);
        bool trims = ratio is > 0.85 and < 1.15;
        bool lands = miss <= 2.0 * bMiss + 5.0 && speed <= 2.0 * bSpeed + 2.0;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  off centre, modelled: plan in {iterations} iterations, {s.Sigma:F1} s; its gimbal holds {ratio:P0} of the engines' moment over the burn {(trims ? "ok" : "FAIL")}"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"    flown open loop: ends {miss:F1} m from its aim at {speed:F1} m/s, as the balanced craft {(lands ? "ok" : "FAIL")}"));

        // The balanced plan on the off-centre craft: what touchdown guidance flew before the moment was modelled.
        double[] blind = Fly(bs, offCentre, x0);
        double blindMiss = Miss(blind, xf);
        bool blindWorse = blindMiss > 5.0 * Math.Max(miss, 1.0);
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  off centre, not modelled (the balanced plan flown open loop): ends {blindMiss:F0} m from its aim at {Speed(blind):F0} m/s, well beyond the modelled plan {(blindWorse ? "ok" : "FAIL")}"));
        if (!(trims && lands && blindWorse))
            return false;

        // 3. THE TRIM BEYOND THE CONE: the gimbals' weakest direction gives a cone narrower than the trim (one engine of a triple plate in flight: 324 %), while the direction that holds the moment has room. Centred on zero the plan is asked for a deflection outside its own cone; centred on the trim it plans and lands.
        const double narrowDeg = 1.5;
        double narrowShare = Math.Sqrt(OffsetX * OffsetX + OffsetY * OffsetY) / Arm / Math.Tan(narrowDeg * Math.PI / 180.0);
        Console.WriteLine();
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"3. the same landing with a {narrowDeg:F1} deg cone, which the trim alone takes {narrowShare:P0} of"));
        bool zeroOk = Plan(offCentre, x0, xf, verbose, out Scvx6DofSolver zs, out int zi, narrowDeg, centred: false);
        double zeroMiss = zeroOk ? Miss(Fly(zs, offCentre, x0), xf) : double.PositiveInfinity;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  cone centred on zero: {(zeroOk ? $"plan in {zi} iterations, flown open loop ends {zeroMiss:F0} m from its aim" : $"NO PLAN after {zi} iterations")} (what touchdown guidance had)"));
        bool centredOk = Plan(offCentre, x0, xf, verbose, out Scvx6DofSolver cs, out int ci, narrowDeg, centred: true);
        if (!centredOk)
        {
            Console.WriteLine($"  cone centred on the trim: NO PLAN after {ci} iterations  FAIL");
            return false;
        }
        double[] cEnd = Fly(cs, offCentre, x0);
        double cMiss = Miss(cEnd, xf), cSpeed = Speed(cEnd);
        bool cLands = cMiss <= 2.0 * bMiss + 5.0 && cSpeed <= 2.0 * bSpeed + 2.0;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  cone centred on the trim: plan in {ci} iterations, {cs.Sigma:F1} s; flown open loop ends {cMiss:F1} m from its aim at {cSpeed:F1} m/s, as the balanced craft {(cLands ? "ok" : "FAIL")}"));
        return cLands;
    }

    private static double Miss(double[] end, double[] xf) => Math.Sqrt(end[0] * end[0] + end[1] * end[1] + (end[2] - xf[2]) * (end[2] - xf[2]));
    private static double Speed(double[] end) => Math.Sqrt(end[3] * end[3] + end[4] * end[4] + end[5] * end[5]);

    private static bool Plan(Dynamics6Dof.Params dyn, double[] x0, double[] xf, bool verbose, out Scvx6DofSolver solver, out int iterations,
                             double gimbalDeg = GimbalDeg, bool centred = false)
    {
        const int n = 40;
        const double sigma = 9.0;
        double length = Math.Sqrt(x0[0] * x0[0] + x0[1] * x0[1] + (x0[2] - xf[2]) * (x0[2] - xf[2]));
        double speed = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
        double velocity = Math.Max(Math.Max(speed, Math.Sqrt(length * Gravity)), 1.0);
        double omegaScale = 60.0 * Math.PI / 180.0 / (sigma * 0.5);
        var cfg = new Scvx6DofConfig
        {
            Nodes = n,
            Tmax = Tmax,
            ThrottleFloor = Floor,
            GimbalMaxDeg = gimbalDeg,
            GimbalTrimX = centred ? dyn.ThrustMomentY / Arm : 0.0,
            GimbalTrimY = centred ? -dyn.ThrustMomentX / Arm : 0.0,
            TauRollMax = 5e4,
            TerminalRollFree = true,
            TiltMaxDeg = 60.0,
            GroundFloor = -1.0,
            WW = 0.002 / (n * 3.0 * omegaScale * omegaScale),
            WDu = 0.05,
            ProximalWeight = 0.05,
            GlideSlopeDeg = 0.0,
            VzMax = 0.0,
            TerminalMissWeight = 1.0e5,
            TerminalSpeedWeight = 1.0e4,
            SigmaMin = sigma * 0.4,
            SigmaMax = sigma * 3.0,
            SigmaScale = sigma,
            VirtualControlScale = [0, 0, 0, 5, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0],
            XScale = [length, length, length, velocity, velocity, velocity, 1, 1, 1, 1, 1, 1, 1, x0[13]],
        };
        solver = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = 3e-4 };
        solver.MaxSubproblemIterations = 9000;
        solver.EscalatedSubproblemIterations = 9000;
        ColdSeed6Dof.Build(x0, xf, sigma, n, cfg, dyn, out double[] xs, out double[] us);
        solver.Initialize(x0, xf, xs, us, sigma);
        double[] tol = new double[NX];
        for (int i = 0; i < 3; i++) { tol[i] = 1.0; tol[3 + i] = 0.5; tol[10 + i] = 0.01; }
        for (int i = 0; i < 4; i++) tol[6 + i] = Math.Sin(0.5 * Math.PI / 180.0);
        tol[13] = x0[13] * 1e-3;
        for (iterations = 0; iterations < 60;)
        {
            ScvxIteration it = solver.Iterate();
            iterations++;
            double ratio = it.Accepted ? solver.WeightedDefect(tol, n - 2, 1.0, out _, out _, out _) : double.PositiveInfinity;
            if (verbose)
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                    $"    {iterations,2}: solved {it.Solved,-5} accepted {it.Accepted,-5} trust {it.TrustRegion,6:G3} sigma {it.Sigma,5:F2} gate {ratio,7:F2}x"));
            if (it.Accepted && ratio <= 1.0)
                return true;
        }
        return false;
    }

    /// <summary>The plan's controls, linearly interpolated, flown on <paramref name="plant"/> by RK4 for the plan's duration; the state at its end.</summary>
    private static double[] Fly(Scvx6DofSolver s, Dynamics6Dof.Params plant, double[] x0)
    {
        int nu = plant.NU;
        int n = s.ReferenceX.Length / NX;
        double dt = s.Sigma / (n - 1);
        const int sub = 20;
        double h = dt / sub;
        double[] x = (double[])x0.Clone();
        var u = new double[nu];
        var k1 = new double[NX]; var k2 = new double[NX]; var k3 = new double[NX]; var k4 = new double[NX];
        var tmp = new double[NX];
        for (int k = 0; k < n - 1; k++)
            for (int j = 0; j < sub; j++)
            {
                double f0 = (double)j / sub, f1 = (j + 0.5) / sub, f2 = (j + 1.0) / sub;
                void Controls(double f)
                {
                    for (int c = 0; c < nu; c++)
                        u[c] = s.ReferenceU[k * nu + c] * (1 - f) + s.ReferenceU[(k + 1) * nu + c] * f;
                }
                Controls(f0);
                Dynamics6Dof.Eval(x, u, plant, k1);
                Controls(f1);
                for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k1[i];
                Dynamics6Dof.Eval(tmp, u, plant, k2);
                for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k2[i];
                Dynamics6Dof.Eval(tmp, u, plant, k3);
                Controls(f2);
                for (int i = 0; i < NX; i++) tmp[i] = x[i] + h * k3[i];
                Dynamics6Dof.Eval(tmp, u, plant, k4);
                for (int i = 0; i < NX; i++) x[i] += h / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
                double qn = Math.Sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
                for (int i = 6; i < 10; i++) x[i] /= qn;
            }
        return x;
    }
}

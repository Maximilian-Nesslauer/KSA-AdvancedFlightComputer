using System.Globalization;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// --6dof-fins: control surfaces in touchdown guidance's plan, one deflection control per fin (BodyAero6Dof.Fins, Dynamics6Dof.IFIN on), as many as the craft has.
///
/// 1. Slopes: the dynamics' Jacobian columns for the fins against central differences, between swept deflections, on one, and at rest, for a craft with four fins and one with three.
/// 2. A landing against a steady pitching moment, the gimbal narrowed so it cannot hold it alone: planned with the fins, which must take the moment, stay inside their range and move no faster than their rate; flown open loop on the true dynamics, it must land as a plan whose gimbal can hold the moment on its own does. With three fins as with four.
/// </summary>
internal static class SixDofFinCheck
{
    private const int NX = Dynamics6Dof.NX;

    private const double Tmax = 600e3, Floor = 0.3, Isp = 282.0, Gravity = 9.81, Mass = 26_000.0, Arm = 15.0, Length = 40.0;
    private const double Area = ThreeDofScenario.Area;
    private const double FinMaxDeg = 45.0, FinRateDegS = 10.0;

    // The model's +Z (thrust axis) along the game's +x (nose), its x and y along the game's y and z (SixDofAeroCheck's).
    private static readonly double[] GameAxes = [0, 0, 1, 1, 0, 0, 0, 1, 0];
    private static readonly double[] Deflections = [-45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0];

    internal static int Run(bool verbose)
    {
        Console.WriteLine("CONTROL SURFACES IN TOUCHDOWN GUIDANCE'S PLAN");
        bool ok = Slopes(4) & Slopes(3);
        ok &= Landing(4, verbose);
        ok &= Landing(3, verbose);
        Console.WriteLine();
        Console.WriteLine(ok ? "PASS - the fins in the touchdown plan" : "FAIL - see above");
        return ok ? 0 : 1;
    }

    // ------------------------------------------------------------------- the model

    private static BodyAeroTable ConstantTable(double c0, double c1, double c2)
    {
        double[] mach = AeroTable.DefaultMachBreakpoints;
        double[] angles = { -20, -16, -12, -8, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 8, 12, 16, 20 };
        var c = new double[mach.Length * angles.Length * angles.Length * 3];
        for (int i = 0; i < c.Length; i += 3)
        {
            c[i] = c0;
            c[i + 1] = c1;
            c[i + 2] = c2;
        }
        return new BodyAeroTable(mach, angles, angles, c);
    }

    /// <summary>
    /// <paramref name="count"/> fins spaced evenly round the nose axis (game x), fin i at azimuth 2 pi i / count. Each turns the craft about the transverse axis square to its own radius with a curve that stalls (full slope to 15 deg, half beyond, a quarter past 30), rolls it a fifth as much, and adds drag that grows with the square of its deflection. In the game's body axes: force (x along the nose) and moment coefficients.
    /// </summary>
    private static BodyAero6Dof.FinTable[] Fins(int count)
    {
        var fins = new BodyAero6Dof.FinTable[count];
        for (int i = 0; i < count; i++)
        {
            double az = 2.0 * Math.PI * i / count;
            // The transverse axis square to the fin's radius (cos az, sin az) in the game's y-z plane.
            double ay = -Math.Sin(az), azz = Math.Cos(az);
            var force = new BodyAeroTable[Deflections.Length];
            var moment = new BodyAeroTable[Deflections.Length];
            for (int k = 0; k < Deflections.Length; k++)
            {
                double d = Deflections[k], m = Math.Abs(d);
                double lift = Math.Sign(d) * (Math.Min(m, 15.0) + 0.5 * Math.Clamp(m - 15.0, 0.0, 15.0) + 0.25 * Math.Max(m - 30.0, 0.0));
                double c = 4.0e-4 * lift;
                double drag = -2.0e-6 * d * d;
                force[k] = ConstantTable(drag, 0.0, 0.0);
                moment[k] = ConstantTable(0.2 * c, c * ay, c * azz);
            }
            fins[i] = new BodyAero6Dof.FinTable
            {
                DeflectionDeg = Deflections, Force = force, Moment = moment,
                MinDeg = -FinMaxDeg, MaxDeg = FinMaxDeg, RateDegS = FinRateDegS,
            };
        }
        return fins;
    }

    private static BodyAero6Dof Body(BodyAeroTable force, BodyAeroTable? moment, BodyAero6Dof.FinTable[] fins) => new()
    {
        Atmosphere = ThreeDofScenario.Model().Atmosphere!,
        Force = force,
        Moment = moment,
        ReferenceArea = Area,
        ReferenceLength = Length,
        ModelToTable = GameAxes,
        Fins = fins,
    };

    private static Dynamics6Dof.Params Params(BodyAero6Dof body) => new()
    {
        Gx = 0, Gy = 0, Gz = -Gravity, G0 = 9.80665, Isp = Isp,
        LArm = Arm, Ixx = Mass * Length * Length / 12.0, Iyy = Mass * Length * Length / 12.0, Izz = 0.5 * Mass * 1.8 * 1.8,
        Aero = ThreeDofScenario.Model(),
        BodyAero = body,
    };

    private static double[] Pointing(double[] t)
    {
        double[] z = [0, 0, 1];
        double[] u = ThreeDofScenario.Normalise(t);
        double c = Math.Clamp(ThreeDofScenario.Dot(z, u), -1, 1);
        double[] axis = ThreeDofScenario.Cross(z, u);
        double s = Math.Sqrt(ThreeDofScenario.Dot(axis, axis));
        if (s < 1e-12) return c > 0 ? [1, 0, 0, 0] : [0, 1, 0, 0];
        double half = 0.5 * Math.Acos(c);
        return [Math.Cos(half), axis[0] / s * Math.Sin(half), axis[1] / s * Math.Sin(half), axis[2] / s * Math.Sin(half)];
    }

    private static double[] State(double h, double[] v, double tiltDeg, double[] rates)
    {
        double sp = Math.Sqrt(ThreeDofScenario.Dot(v, v));
        double[] retro = [-v[0] / sp, -v[1] / sp, -v[2] / sp];
        double a = tiltDeg * Math.PI / 180;
        double[] axis = [retro[0], retro[1] * Math.Cos(a) - retro[2] * Math.Sin(a), retro[1] * Math.Sin(a) + retro[2] * Math.Cos(a)];
        double[] q = Pointing(axis);
        return [-200, 30, h, v[0], v[1], v[2], q[0], q[1], q[2], q[3], rates[0], rates[1], rates[2], Mass];
    }

    // ------------------------------------------------------------------- 1. slopes

    private static bool Slopes(int count)
    {
        Console.WriteLine();
        Console.WriteLine($"1. slopes, {count} fins: the fin columns against central differences");
        Dynamics6Dof.Params p = Params(Body(ThreeDofScenario.BodyTable(0.05, -0.03), ConstantTable(0.002, 0.07, -0.03), Fins(count)));
        int nu = p.NU;
        bool ok = nu == Dynamics6Dof.NUBase + count;
        Console.WriteLine($"  {nu} controls: the base {Dynamics6Dof.NUBase} and {count} fins  {(ok ? "ok" : "FAIL")}");
        double[] x = State(600, [40, -5, -85], 5, [0.01, -0.02, 0.03]);
        // Between swept deflections, on one (15 deg, where the slope halves), at rest, and on the far side of rest.
        foreach ((string name, double deg) in new[] { ("between, 7 deg", 7.0), ("on 15 deg", 15.0), ("at rest", 0.0), ("between, -22 deg", -22.0) })
        {
            var u = new double[nu];
            u[Dynamics6Dof.IT] = 350e3;
            for (int i = 0; i < count; i++)
                u[Dynamics6Dof.IFIN + i] = deg * (i % 2 == 0 ? 1.0 : -0.5);
            var A = new double[NX * NX];
            var B = new double[NX * nu];
            Dynamics6Dof.Jacobian(x, u, p, new double[NX], A, B);
            double worst = 0;
            for (int j = Dynamics6Dof.IFIN; j < nu; j++)
            {
                const double h = 1e-3;
                double[] up = (double[])u.Clone(), um = (double[])u.Clone();
                up[j] += h; um[j] -= h;
                var fp = new double[NX];
                var fm = new double[NX];
                Dynamics6Dof.Eval(x, up, p, fp);
                Dynamics6Dof.Eval(x, um, p, fm);
                for (int row = 3; row < NX; row++)
                {
                    if (row >= 6 && row < 10) continue;
                    double fd = (fp[row] - fm[row]) / (2 * h);
                    double ad = B[row * nu + j];
                    worst = Math.Max(worst, Math.Abs(ad - fd) / Math.Max(Math.Abs(fd), 1e-6));
                }
            }
            bool pass = worst < 1e-4;
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {name,-18} worst {worst:E1}  {(pass ? "ok" : "FAIL")}"));
            ok &= pass;
        }
        return ok;
    }

    // ------------------------------------------------------------------- 2. landing

    private static bool Landing(int count, bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine($"2. a landing against a steady pitching moment, {count} fins, the gimbal narrowed so it cannot hold the moment alone early on");
        BodyAeroTable force = ThreeDofScenario.BodyTable(0.02, 0.0);
        BodyAeroTable pitch = ConstantTable(0.0, 0.07, 0.0);
        double[] x0 = State(600, [25, -4, -90], 0, [0, 0, 0]);
        double[] xf = new double[NX];
        xf[2] = 5.0;
        xf[6] = 1.0;

        // The baseline: a wide gimbal that holds the moment by itself, no fins.
        Dynamics6Dof.Params wide = Params(Body(force, pitch, []));
        bool baseOk = Plan(wide, x0, xf, 8.0, 0, verbose, out Scvx6DofSolver bs, out int bi);
        double[] bEnd = baseOk ? Fly(bs, wide, x0) : [];
        double bMiss = baseOk ? Miss(bEnd, xf) : double.PositiveInfinity, bSpeed = baseOk ? Speed(bEnd) : double.PositiveInfinity;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  gimbal 8 deg, no fins: {(baseOk ? $"plan in {bi} iterations, {bs.Sigma:F1} s; flown open loop ends {bMiss:F1} m from its aim at {bSpeed:F1} m/s" : "NO PLAN")}"));

        // With the fins and a 1.5 deg gimbal.
        Dynamics6Dof.Params finned = Params(Body(force, pitch, Fins(count)));
        bool ok = Plan(finned, x0, xf, 1.5, count, verbose, out Scvx6DofSolver s, out int iterations);
        if (!ok)
        {
            Console.WriteLine($"  gimbal 1.5 deg, {count} fins: NO PLAN after {iterations} iterations  FAIL");
            return false;
        }
        int nu = finned.NU, n = s.ReferenceX.Length / NX;
        double dt = s.Sigma / (n - 1), peakDeg = 0.0, peakRate = 0.0, mean = 0.0;
        for (int k = 0; k < n; k++)
            for (int i = 0; i < count; i++)
            {
                double d = s.ReferenceU[k * nu + Dynamics6Dof.IFIN + i];
                peakDeg = Math.Max(peakDeg, Math.Abs(d));
                mean += Math.Abs(d) / (n * count);
                if (k + 1 < n)
                    peakRate = Math.Max(peakRate, Math.Abs(s.ReferenceU[(k + 1) * nu + Dynamics6Dof.IFIN + i] - d) / dt);
            }
        double[] end = Fly(s, finned, x0);
        double miss = Miss(end, xf), speed = Speed(end);
        bool used = mean > 1.0;
        bool inRange = peakDeg <= FinMaxDeg * 1.001;
        bool inRate = peakRate <= FinRateDegS * 1.02;
        bool lands = miss <= 2.0 * bMiss + 5.0 && speed <= 2.0 * bSpeed + 2.0;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  gimbal 1.5 deg, {count} fins: plan in {iterations} iterations, {s.Sigma:F1} s; fins mean {mean:F1} deg {(used ? "ok" : "FAIL")}, peak {peakDeg:F1} of {FinMaxDeg:F0} deg {(inRange ? "ok" : "FAIL")}, fastest {peakRate:F1} of {FinRateDegS:F0} deg/s {(inRate ? "ok" : "FAIL")}"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"    flown open loop: ends {miss:F1} m from its aim at {speed:F1} m/s, as the wide gimbal {(lands ? "ok" : "FAIL")}"));
        if (verbose)
            for (int k = 0; k < n; k += 4)
                Console.WriteLine("    node " + k + ": " + string.Join(" ", Enumerable.Range(0, count).Select(i => s.ReferenceU[k * nu + Dynamics6Dof.IFIN + i].ToString("F1", CultureInfo.InvariantCulture))));
        return used && inRange && inRate && lands;
    }

    private static double Miss(double[] end, double[] xf) => Math.Sqrt(end[0] * end[0] + end[1] * end[1] + (end[2] - xf[2]) * (end[2] - xf[2]));
    private static double Speed(double[] end) => Math.Sqrt(end[3] * end[3] + end[4] * end[4] + end[5] * end[5]);

    private static bool Plan(Dynamics6Dof.Params dyn, double[] x0, double[] xf, double gimbalDeg, int fins, bool verbose,
                             out Scvx6DofSolver solver, out int iterations)
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
            FinMinDeg = Enumerable.Repeat(-FinMaxDeg, fins).ToArray(),
            FinMaxDeg = Enumerable.Repeat(FinMaxDeg, fins).ToArray(),
            FinRateDegS = Enumerable.Repeat(FinRateDegS, fins).ToArray(),
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

using System.Globalization;
using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// --6dof-aero: touchdown guidance's body-axis force and moment (BodyAero6Dof in Dynamics6Dof).
///
/// 1. Slopes: the dynamics' forward-mode Jacobian against central differences, in full air and inside both fades (low speed, large tilt).
/// 2. Agreement: a table built from the roll-averaged drag and lift gives the old thrust-axis force; a constant moment comes back about the model axes the game's body axes map to, and a centre-of-mass shift adds its arm.
/// 3. A booster landing from 600 m at 90 m/s against a steady pitching moment of the size the 2026-11-18 booster's tables show: planned with the moment and without it, each plan's controls flown open loop on the true dynamics. The plan that knows the moment must land where it planned to; the one that does not is printed beside it.
/// </summary>
internal static class SixDofAeroCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    // A returning booster near the end of its propellant, as ThreeDofScenario's, on one engine it can throttle below its weight.
    private const double Tmax = 600e3, Floor = 0.3, Isp = 282.0, Gravity = 9.81, Mass = 26_000.0, Arm = 15.0, Length = 40.0;
    private const double Area = ThreeDofScenario.Area;

    internal static int Run(bool verbose)
    {
        Console.WriteLine("TOUCHDOWN GUIDANCE'S BODY-AXIS FORCE AND MOMENT");
        bool ok = Slopes();
        ok &= Agreement();
        ok &= Landing(verbose);
        Console.WriteLine();
        Console.WriteLine(ok ? "PASS - touchdown guidance's body-axis aerodynamics" : "FAIL - see above");
        return ok ? 0 : 1;
    }

    // ------------------------------------------------------------------- the model

    // The model's +Z (thrust axis) along the game's +x (nose), and its other axes as KsaFrameBridge.BodyAxes builds them from a fixed game axis.
    private static readonly double[] GameAxes = [0, 0, 1, 1, 0, 0, 0, 1, 0];

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

    private static Dynamics6Dof.Params Params(BodyAero6Dof? body, bool oldAero = true) => new()
    {
        Gx = 0, Gy = 0, Gz = -Gravity, G0 = 9.80665, Isp = Isp,
        LArm = Arm, Ixx = Mass * Length * Length / 12.0, Iyy = Mass * Length * Length / 12.0, Izz = 0.5 * Mass * 1.8 * 1.8,
        Aero = oldAero ? ThreeDofScenario.Model() : null,
        BodyAero = body,
    };

    private static BodyAero6Dof Body(BodyAeroTable force, BodyAeroTable? moment, double[] axes, double shiftX = 0, double shiftY = 0, double shiftZ = 0) => new()
    {
        Atmosphere = ThreeDofScenario.Model().Atmosphere!,
        Force = force,
        Moment = moment,
        ReferenceArea = Area,
        ReferenceLength = Length,
        ModelToTable = axes,
        ComShiftX = shiftX, ComShiftY = shiftY, ComShiftZ = shiftZ,
    };

    /// <summary>The quaternion (scalar first) turning the model's +Z onto <paramref name="t"/>, the shortest way.</summary>
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

    /// <summary>A state at altitude h moving at v, the thrust axis tilted <paramref name="tiltDeg"/> off retrograde about the site x axis.</summary>
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

    private static bool Slopes()
    {
        Console.WriteLine();
        Console.WriteLine("1. slopes: AD against central differences, full air and both fades");
        BodyAero6Dof body = Body(ThreeDofScenario.BodyTable(0.05, -0.03), ConstantTable(0.002, 0.07, -0.03), GameAxes, 0.4, -0.1, 0.2);
        Dynamics6Dof.Params p = Params(body);
        (string name, double[] x)[] states =
        {
            ("full air, 5 deg", State(600, [40, -5, -85], 5, [0.01, -0.02, 0.03])),
            ("speed fade, 10 m/s", State(40, [3, 1, -9.4], 4, [0, 0.01, 0])),
            ("tilt fade, 24 deg", State(400, [30, 0, -70], 24, [0.02, 0, 0])),
        };
        double[] u = [3000, -2000, 350e3, 500];
        bool ok = true;
        foreach ((string name, double[] x) in states)
        {
            var A = new double[NX * NX];
            var B = new double[NX * NU];
            var f = new double[NX];
            Dynamics6Dof.Jacobian(x, u, p, f, A, B);
            double worst = 0;
            for (int col = 0; col < NX + NU; col++)
            {
                if (col == Dynamics6Dof.IM) continue;
                double h = col < NX ? (col < 3 ? 1e-2 : col < 6 ? 1e-3 : col < 10 ? 1e-6 : 1e-6) : (col == NX + Dynamics6Dof.IT ? 1.0 : 1e-1);
                double[] xp = (double[])x.Clone(), xm = (double[])x.Clone(), up = (double[])u.Clone(), um = (double[])u.Clone();
                if (col < NX) { xp[col] += h; xm[col] -= h; } else { up[col - NX] += h; um[col - NX] -= h; }
                var fp = new double[NX];
                var fm = new double[NX];
                Dynamics6Dof.Eval(xp, up, p, fp);
                Dynamics6Dof.Eval(xm, um, p, fm);
                for (int row = 3; row < NX; row++)
                {
                    if (row >= 6 && row < 10) continue;   // the quaternion rate is the kinematics, untouched
                    double fd = (fp[row] - fm[row]) / (2 * h);
                    double ad = col < NX ? A[row * NX + col] : B[row * NU + col - NX];
                    double scale = Math.Max(Math.Abs(fd), 1e-4);
                    worst = Math.Max(worst, Math.Abs(ad - fd) / scale);
                }
            }
            bool pass = worst < 1e-4;
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {name,-20} worst {worst:E1}  {(pass ? "ok" : "FAIL")}"));
            ok &= pass;
        }
        return ok;
    }

    // ------------------------------------------------------------------- 2. agreement

    private static bool Agreement()
    {
        Console.WriteLine();
        Console.WriteLine("2. agreement: the old force from a symmetric table, and a moment about the right axes");
        bool ok = true;
        double[] u0 = [0, 0, 0, 0];

        // The symmetric table against the old thrust-axis force, at tilts inside the full-air region.
        Dynamics6Dof.Params oldOnly = Params(null);
        Dynamics6Dof.Params tableOnly = Params(Body(ThreeDofScenario.BodyTable(), null, GameAxes), oldAero: false);
        double worst = 0;
        foreach (double tilt in new[] { 0.0, 2.0, 5.0, 10.0, 15.0 })
        {
            double[] x = State(600, [40, -5, -85], tilt, [0, 0, 0]);
            Dynamics6Dof.AeroForce(x, oldOnly, out double ox, out double oy, out double oz);
            Dynamics6Dof.AeroForce(x, tableOnly, out double nx, out double ny, out double nz);
            double on = Math.Sqrt(ox * ox + oy * oy + oz * oz);
            worst = Math.Max(worst, Math.Sqrt((nx - ox) * (nx - ox) + (ny - oy) * (ny - oy) + (nz - oz) * (nz - oz)) / on);
        }
        bool agree = worst < 0.02;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  symmetric table vs the old force, 0 to 15 deg: worst {worst:E1} of the force  {(agree ? "ok" : "FAIL")}"));
        ok &= agree;

        // A constant moment and a shift: the body rates' derivatives against q S L c + d x F, mapped to model axes.
        const double cl = 0.004, cm = 0.07, cn = -0.03;
        double[] shift = [0.5, -0.2, 0.3];
        BodyAero6Dof body = Body(ConstantTable(-0.6, 0.05, -0.02), ConstantTable(cl, cm, cn), GameAxes, shift[0], shift[1], shift[2]);
        Dynamics6Dof.Params p = Params(body, oldAero: false);
        double[] xs = State(600, [40, -5, -85], 3, [0, 0, 0]);
        var f = new double[NX];
        Dynamics6Dof.Eval(xs, u0, p, f);
        double speed = Math.Sqrt(xs[3] * xs[3] + xs[4] * xs[4] + xs[5] * xs[5]);
        double qs = 0.5 * body.Atmosphere.Density(xs[2]) * speed * speed * Area;
        double[] force = [qs * -0.6, qs * 0.05, qs * -0.02];
        double[] arm = ThreeDofScenario.Cross(shift, force);
        double[] game = [qs * Length * cl + arm[0], qs * Length * cm + arm[1], qs * Length * cn + arm[2]];
        // Model axes: M transposed.
        double[] want = [GameAxes[0] * game[0] + GameAxes[3] * game[1] + GameAxes[6] * game[2],
                         GameAxes[1] * game[0] + GameAxes[4] * game[1] + GameAxes[7] * game[2],
                         GameAxes[2] * game[0] + GameAxes[5] * game[1] + GameAxes[8] * game[2]];
        Dynamics6Dof.Params plain = Params(null, oldAero: false);
        double[] got = [f[10] * plain.Ixx, f[11] * plain.Iyy, f[12] * plain.Izz];
        double err = Math.Sqrt((got[0] - want[0]) * (got[0] - want[0]) + (got[1] - want[1]) * (got[1] - want[1]) + (got[2] - want[2]) * (got[2] - want[2]))
                   / Math.Sqrt(ThreeDofScenario.Dot(want, want));
        bool moment = err < 1e-9;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  constant moment and a {Math.Sqrt(ThreeDofScenario.Dot(shift, shift)):F2} m shift: {got[0] / 1000:F1} / {got[1] / 1000:F1} / {got[2] / 1000:F1} kN m about model x/y/z, off by {err:E1}  {(moment ? "ok" : "FAIL")}"));
        ok &= moment;
        return ok;
    }

    // ------------------------------------------------------------------- 3. landing

    private static bool Landing(bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine("3. a landing against a steady pitching moment, planned with it and without, flown open loop on the true dynamics");
        BodyAeroTable force = ThreeDofScenario.BodyTable(0.02, 0.0);
        BodyAero6Dof truthBody = Body(force, ConstantTable(0.0, 0.07, 0.0), GameAxes);
        Dynamics6Dof.Params truth = Params(truthBody);
        Dynamics6Dof.Params blind = Params(Body(force, null, GameAxes));

        double[] x0 = State(600, [25, -4, -90], 0, [0, 0, 0]);
        double[] xf = new double[NX];
        xf[2] = 5.0;
        xf[6] = 1.0;

        // Open loop, even a plan of the true dynamics drifts: the plan holds its defect to a metre and half a metre a second an interval, and an attitude flown without feedback carries every error on. The baseline is that drift with no moment anywhere: the same landing, planned and flown in air that has none.
        var rows = new (string name, Dynamics6Dof.Params model, Dynamics6Dof.Params plant)[]
        {
            ("no moment at all", blind, blind),
            ("with the moment", truth, truth),
            ("without it", blind, truth),
        };
        var miss = new double[rows.Length];
        var speed = new double[rows.Length];
        for (int r = 0; r < rows.Length; r++)
        {
            (string name, Dynamics6Dof.Params model, Dynamics6Dof.Params plant) = rows[r];
            if (!Plan(model, x0, xf, verbose, out Scvx6DofSolver s, out int iterations))
            {
                Console.WriteLine($"  {name,-18} NO PLAN after {iterations} iterations");
                miss[r] = speed[r] = double.PositiveInfinity;
                continue;
            }
            double[] end = Fly(s, plant, x0, out _);
            miss[r] = Math.Sqrt(end[0] * end[0] + end[1] * end[1] + (end[2] - xf[2]) * (end[2] - xf[2]));
            speed[r] = Math.Sqrt(end[3] * end[3] + end[4] * end[4] + end[5] * end[5]);
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"  {name,-18} plan in {iterations,2} iterations, {s.Sigma:F1} s; flown open loop: ends {miss[r]:F1} m from its aim at {speed[r]:F1} m/s"));
        }
        // Knowing the moment must fly as well as a world with none, give or take, and far better than not knowing it.
        bool asBaseline = miss[1] <= 2.0 * miss[0] + 5.0 && speed[1] <= 2.0 * speed[0] + 2.0;
        bool betterThanBlind = miss[2] > 3.0 * miss[1] && speed[2] > 3.0 * speed[1];
        Console.WriteLine($"  the plan that knows the moment flies as the baseline {(asBaseline ? "ok" : "FAIL")}, and far better than the one that does not {(betterThanBlind ? "ok" : "FAIL")}");
        return asBaseline && betterThanBlind;
    }

    private static bool Plan(Dynamics6Dof.Params dyn, double[] x0, double[] xf, bool verbose, out Scvx6DofSolver solver, out int iterations)
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
            GimbalMaxDeg = 8.0,
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

    /// <summary>The plan's controls, interpolated, flown on <paramref name="plant"/> by RK4 for the plan's duration; the state at its end.</summary>
    private static double[] Fly(Scvx6DofSolver s, Dynamics6Dof.Params plant, double[] x0, out double peakMoment)
    {
        int n = s.ReferenceX.Length / NX;
        double sigma = s.Sigma, dt = 0.01;
        double[] x = (double[])x0.Clone();
        double[] u = new double[NU];
        peakMoment = 0;
        for (double t = 0; t < sigma - 1e-9; t += dt)
        {
            double h = Math.Min(dt, sigma - t);
            double sIdx = Math.Clamp(t / (sigma / (n - 1)), 0.0, n - 1.0);
            int a = Math.Min((int)sIdx, n - 2);
            double fr = sIdx - a;
            for (int i = 0; i < NU; i++)
                u[i] = s.ReferenceU[a * NU + i] * (1 - fr) + s.ReferenceU[(a + 1) * NU + i] * fr;
            Span<double> k1 = stackalloc double[NX], k2 = stackalloc double[NX], k3 = stackalloc double[NX], k4 = stackalloc double[NX], tmp = stackalloc double[NX];
            Dynamics6Dof.Eval(x, u, plant, k1);
            for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k1[i];
            Dynamics6Dof.Eval(tmp, u, plant, k2);
            for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k2[i];
            Dynamics6Dof.Eval(tmp, u, plant, k3);
            for (int i = 0; i < NX; i++) tmp[i] = x[i] + h * k3[i];
            Dynamics6Dof.Eval(tmp, u, plant, k4);
            for (int i = 0; i < NX; i++) x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            double norm = Math.Sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
            for (int i = 6; i < 10; i++) x[i] /= norm;
        }
        return x;
    }
}

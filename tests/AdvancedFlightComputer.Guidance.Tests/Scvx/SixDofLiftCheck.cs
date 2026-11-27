using System.Globalization;
using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// --6dof-lift: does lift as strong as KSA 5901's stop touchdown guidance converging and landing?
///
/// Touchdown guidance's own plans show the air it planned with. Before KSA 5901 (KSAero): drag 5-7 m/s^2 and lift 0.1-0.4 m/s^2 per degree, both at 100 m/s. On 5901: drag 9-10 and lift 1.5 per degree. The lift pushes against the thrust's own sideways part, which is about 0.5 m/s^2 per degree.
/// Before 5901 every re-solve converged in one iteration at about 1 % of the gate. On 5901 the gate sits near or over 1, and the landings miss by 7-17 m.
///
/// A booster like the logged one (16.4 t, 2171 kN, 2 deg gimbals at 2 deflections a second, the plans' arm-to-inertia ratio) starts from three logged hand-overs and lands in three kinds of air, the plan's thrust-axis aero model (KsaPointMassModel) scaled to each:
///   1. A cold solve at 50 nodes: iterations to the gate and to 5 % of it.
///   2. A closed loop at 15 nodes: a warm re-solve from the true state every 0.1 s, at most 3 iterations, kept only inside the gate, as in the game. The plan's controls are flown between re-solves on a plant whose lift is 9 % stronger than the model's, as measured on 2026-11-18.
/// Reports; it passes or fails nothing.
/// </summary>
internal static class SixDofLiftCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NUBase;
    private const double WarmTrustRegion = 0.05;   // Ksa6DofGuidance.WarmTrustRegion
    private const double Tmax = 2171e3, Floor = 0.10, Isp = 290.0, Gravity = 9.81;
    private const double Arm = 10.0, Ixx = 1.14e6, Izz = 2.7e4;    // LArm/Ixx = 8.8e-6, from the 23:05 plan's rates against its gimbal
    private const double TargetZ = 13.0, AccelMax = 31.0, GimbalDeg = 2.0, SlewPerS = 2.0;
    private const double PlantLiftFactor = 1.09;

    private static readonly (string name, double sigma, double[] x)[] States =
    [
        ("17 Nov 15:45 (pre-5901)", 4.81, [17.34, 22.15, 244.31, -11.631, -15.215, -99.836, 0.70726, -0.023510, 0.13068, 0.69437, 0.00934, -0.02733, -0.00003, 18164]),
        ("18 Nov 23:03", 6.48, [42.97, 17.22, 177.91, -17.185, -3.448, -75.618, 0.95749, -0.030383, 0.080936, -0.27520, 0.09858, -0.04092, -0.06774, 16602]),
        ("18 Nov 23:05", 6.22, [8.95, 1.45, 205.73, -15.905, -9.006, -88.892, 0.96154, -0.033297, 0.039578, -0.26976, 0.02567, 0.00956, -0.05240, 16426]),
    ];

    private static readonly (string name, double drag, double lift)[] Airs =
    [
        ("pre-5901 air", 6.0, 0.15),
        ("5901 drag, old lift", 9.5, 0.15),
        ("5901 air", 9.5, 1.5),
    ];

    internal static int Run(bool verbose)
    {
        Console.WriteLine("TOUCHDOWN GUIDANCE AGAINST THE LIFT OF KSA 5901: same booster, same hand-overs, only the air differs");
        Console.WriteLine("  drag in m/s^2 and lift in m/s^2 per degree of AoA, both at 100 m/s; lift pushes against the thrust's sideways part");
        foreach ((string sname, double sigma, double[] x0) in States)
        {
            Console.WriteLine();
            Console.WriteLine($"{sname}: {x0[2]:F0} m up, {Math.Sqrt(x0[0] * x0[0] + x0[1] * x0[1]):F0} m off, {Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4]):F1} m/s sideways, {-x0[5]:F0} m/s down");
            foreach ((string aname, double drag, double lift) in Airs)
            {
                KsaPointMassModel model = Model(drag, lift, x0[13], 1.0);
                KsaPointMassModel truth = Model(drag, lift, x0[13], PlantLiftFactor);
                Dynamics6Dof.Params dyn = Params(model), plant = Params(truth);
                string cold = Cold(dyn, x0, sigma, verbose);
                string loop = Loop(dyn, plant, x0, sigma, verbose);
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {aname,-20} (drag {drag,4:F1}, lift {lift,4:F2})  cold: {cold}"));
                Console.WriteLine($"  {"",-20}                          loop: {loop}");
            }
        }
        return 0;
    }

    // ------------------------------------------------------------------- the model

    private static KsaPointMassModel Model(double dragPer100, double liftPerDeg100, double mass, double liftFactor)
    {
        KsaPointMassModel unit = Build(1.0, 1.0);
        Measure(unit, out double d, out double l);
        return Build(dragPer100 * mass / d, liftFactor * liftPerDeg100 * mass / l);
    }

    private static KsaPointMassModel Build(double dragScale, double liftScale)
    {
        KsaPointMassModel s = ThreeDofScenario.Model();
        return new KsaPointMassModel
        {
            Mu = s.Mu, MeanRadius = s.MeanRadius, CentreZ = s.CentreZ,
            OmegaX = s.OmegaX, OmegaY = s.OmegaY, OmegaZ = s.OmegaZ,
            Atmosphere = s.Atmosphere, Drag = s.Drag, LiftSlope = s.LiftSlope,
            ReferenceArea = s.ReferenceArea, VacuumThrust = s.VacuumThrust, ExitArea = s.ExitArea, MassFlow = s.MassFlow,
            DragScale = dragScale, LiftScale = liftScale,
        };
    }

    /// <summary>Drag, N, and lift, N per degree, at 100 m/s straight down, 100 m up, from 2 deg off the airflow.</summary>
    private static void Measure(KsaPointMassModel m, out double drag, out double liftPerDeg)
    {
        m.AeroForce(new Dual(100.0), new Dual(0.0), new Dual(0.0), new Dual(-100.0), new Dual(0.0), new Dual(0.0), new Dual(1.0), out _, out _, out Dual fz);
        double a = 2.0 * Math.PI / 180.0;
        m.AeroForce(new Dual(100.0), new Dual(0.0), new Dual(0.0), new Dual(-100.0), new Dual(Math.Sin(a)), new Dual(0.0), new Dual(Math.Cos(a)), out Dual fx, out _, out _);
        drag = fz.V;
        liftPerDeg = Math.Abs(fx.V) / 2.0;
    }

    private static Dynamics6Dof.Params Params(KsaPointMassModel aero) => new()
    {
        Gx = 0, Gy = 0, Gz = -Gravity, G0 = 9.80665, Isp = Isp,
        LArm = Arm, Ixx = Ixx, Iyy = Ixx, Izz = Izz,
        Aero = aero,
    };

    private static double[] Target(double mass)
    {
        var xf = new double[NX];
        xf[2] = TargetZ;
        xf[6] = 1.0;
        xf[13] = mass;
        return xf;
    }

    private static Scvx6DofConfig Config(int n, double sigma, double[] x0, double[] xf)
    {
        double length = Math.Sqrt(x0[0] * x0[0] + x0[1] * x0[1] + (x0[2] - xf[2]) * (x0[2] - xf[2]));
        double speed = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
        double velocity = Math.Max(Math.Max(speed, Math.Sqrt(length * Gravity)), 1.0);
        double omegaScale = 60.0 * Math.PI / 180.0 / (sigma * 0.5);
        return new Scvx6DofConfig
        {
            Nodes = n,
            Tmax = Tmax,
            ThrottleFloor = Floor,
            GimbalMaxDeg = GimbalDeg,
            GimbalSlewPerS = SlewPerS * PathShaping.SlewShare,
            TauRollMax = 1.0e4,
            TerminalRollFree = true,
            TiltMaxDeg = 60.0,
            GroundFloor = -1.0,
            AccelMax = AccelMax,
            WW = 0.002 / (n * 3.0 * omegaScale * omegaScale),
            WDu = 0.05,
            ProximalWeight = 0.05,
            GlideSlopeDeg = PathShaping.FarDeg(0.0), GlideFunnelHeightsM = PathShaping.FunnelHeightsM, GlideFunnelHalfAngleDeg = PathShaping.FunnelDeg, GlideFunnelWeight = PathShaping.FunnelWeight, SideSpeedPerM = PathShaping.SideSpeedPerM,
            VzMax = 0.0,
            TerminalMissWeight = 1.0e5,
            TerminalSpeedWeight = 1.0e4,
            SigmaMin = sigma * 0.15,
            SigmaMax = sigma * 4.0,
            SigmaScale = sigma,
            VirtualControlScale = [0, 0, 0, 5, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0],
            XScale = [length, length, length, velocity, velocity, velocity, 1, 1, 1, 1, 1, 1, 1, x0[13]],
        };
    }

    private static Scvx6DofSolver Solver(Scvx6DofConfig cfg, Dynamics6Dof.Params dyn)
    {
        var s = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = 3e-4 };
        s.MaxSubproblemIterations = 9000;
        s.EscalatedSubproblemIterations = 9000;
        return s;
    }

    /// <summary>The game's gate: 1 m, 0.5 m/s, 0.5 deg, 0.01 rad/s, 0.1 % of the mass, an interval.</summary>
    private static double[] Tolerance(double mass)
    {
        var tol = new double[NX];
        for (int i = 0; i < 3; i++) { tol[i] = 1.0; tol[3 + i] = 0.5; tol[10 + i] = 0.01; }
        for (int i = 0; i < 4; i++) tol[6 + i] = Math.Sin(0.5 * Math.PI / 180.0);
        tol[13] = mass * 1e-3;
        return tol;
    }

    // ------------------------------------------------------------------- 1. cold

    private static string Cold(Dynamics6Dof.Params dyn, double[] x0, double sigma, bool verbose)
    {
        const int n = 50;
        double[] xf = Target(x0[13]);
        Scvx6DofConfig cfg = Config(n, sigma, x0, xf);
        Scvx6DofSolver s = Solver(cfg, dyn);
        ColdSeed6Dof.Build(x0, xf, sigma, n, cfg, dyn, out double[] xs, out double[] us);
        s.Initialize(x0, xf, xs, us, sigma);
        double[] tol = Tolerance(x0[13]);
        int toGate = -1, toFine = -1;
        double best = double.PositiveInfinity;
        for (int i = 1; i <= 40; i++)
        {
            ScvxIteration it = s.Iterate();
            double ratio = it.Accepted ? s.WeightedDefect(tol, n - 2, 1.0, out _, out _, out _) : double.PositiveInfinity;
            best = Math.Min(best, ratio);
            if (verbose)
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"      {i,2}: accepted {it.Accepted,-5} trust {it.TrustRegion,6:G3} sigma {it.Sigma,5:F2} gate {ratio,7:F3}x"));
            if (toGate < 0 && ratio <= 1.0) toGate = i;
            if (ratio <= 0.05) { toFine = i; break; }
        }
        return string.Create(CultureInfo.InvariantCulture,
            $"gate in {(toGate < 0 ? "-" : toGate.ToString())}, 5 % of it in {(toFine < 0 ? "- (40 tried)" : toFine.ToString())} iterations, best {best:F3}x, burn {s.Sigma:F1} s");
    }

    // ------------------------------------------------------------------- 2. closed loop

    private static string Loop(Dynamics6Dof.Params dyn, Dynamics6Dof.Params plant, double[] x0, double sigma, bool verbose)
    {
        const int n = 15;
        const double cycle = 0.1, dt = 0.01;
        double[] xf = Target(x0[13]);
        Scvx6DofConfig cfg = Config(n, sigma, x0, xf);
        Scvx6DofSolver s = Solver(cfg, dyn);
        ColdSeed6Dof.Build(x0, xf, sigma, n, cfg, dyn, out double[] xs, out double[] us);
        s.Initialize(x0, xf, xs, us, sigma);
        double[] tol = Tolerance(x0[13]);
        for (int i = 0; i < 40; i++)
        {
            ScvxIteration it = s.Iterate();
            if (it.Accepted && s.WeightedDefect(tol, n - 2, 1.0, out _, out _, out _) <= 0.05)
                break;
        }

        // The plan being flown, and when it started.
        double[] fx = (double[])s.ReferenceX.Clone(), fu = (double[])s.ReferenceU.Clone();
        double fs = s.Sigma, f0 = 0.0;
        double[] x = (double[])x0.Clone();
        double plantGx = double.NaN, plantGy = double.NaN;
        int refused = 0, kept = 0;
        var gates = new List<double>();
        double t = 0.0;
        while (t < 20.0)
        {
            double left = fs - (t - f0);
            if (left > 0.4 && t > 0.0)
            {
                Shift(fx, fu, fs, t - f0, n, out double[] sx, out double[] su, out double ss);
                // As the game re-seeds a warm solve (Ksa6DofGuidance): the whole reference moved with the measured state, not node 0 alone, and the trust region opened to its warm start rather than left where the last solve shrank it.
                ColdReferenceShift.Apply(sx, n, ss, x);
                PathShaping.Anchor(cfg, su, gx: plantGx, gy: plantGy);
                s.Reseed(x, sx, su, ss, trustRegion: Math.Clamp(s.TrustRegion, WarmTrustRegion, s.TrustRegionMax));
                double ratio = double.PositiveInfinity;
                for (int i = 0; i < 3; i++)
                {
                    ScvxIteration it = s.Iterate();
                    ratio = it.Accepted ? s.WeightedDefect(tol, n - 2, 1.0, out _, out _, out _) : double.PositiveInfinity;
                    if (ratio <= 0.05) break;
                }
                gates.Add(ratio);
                if (ratio <= 1.0)
                {
                    fx = (double[])s.ReferenceX.Clone();
                    fu = (double[])s.ReferenceU.Clone();
                    fs = s.Sigma;
                    f0 = t;
                    kept++;
                }
                else
                    refused++;
                if (verbose)
                    Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                        $"      t {t,5:F1} alt {x[2],6:F1} off ({x[0],6:F1},{x[1],6:F1}) v ({x[3],6:F1},{x[4],6:F1},{x[5],6:F1}) gate {ratio,7:F3}x {(ratio <= 1.0 ? "kept" : "REFUSED")}"));
            }
            bool landed = false;
            for (double h = 0; h < cycle - 1e-9; h += dt)
            {
                double[] u = Sample(fu, NU, fs, n, t - f0);
                PathShaping.SlewPlant(cfg, u, dt, ref plantGx, ref plantGy);
                Step(x, u, plant, dt);
                t += dt;
                if (x[2] <= TargetZ || (x[5] > 0.0 && x[2] < TargetZ + 20.0)) { landed = true; break; }
            }
            if (landed) break;
        }
        gates.Sort();
        double med = gates.Count > 0 ? gates[gates.Count / 2] : double.NaN;
        double miss = Math.Sqrt(x[0] * x[0] + x[1] * x[1]);
        double side = Math.Sqrt(x[3] * x[3] + x[4] * x[4]);
        return string.Create(CultureInfo.InvariantCulture,
            $"{kept} kept, {refused} refused, median gate {med:F2}x; ends {x[2]:F1} m up, {miss:F1} m off, {-x[5]:F1} m/s down, {side:F1} m/s sideways");
    }

    /// <summary>The plan resampled at n nodes from <paramref name="elapsed"/> to its end.</summary>
    private static void Shift(double[] X, double[] U, double sigma, double elapsed, int n, out double[] sx, out double[] su, out double ss)
    {
        ss = Math.Max(sigma - elapsed, 0.3);
        sx = new double[n * NX];
        su = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double tk = elapsed + k * ss / (n - 1);
            Array.Copy(Sample(X, NX, sigma, n, tk), 0, sx, k * NX, NX);
            Array.Copy(Sample(U, NU, sigma, n, tk), 0, su, k * NU, NU);
        }
    }

    private static double[] Sample(double[] V, int width, double sigma, int n, double t)
    {
        double sIdx = Math.Clamp(t / (sigma / (n - 1)), 0.0, n - 1.0);
        int a = Math.Min((int)sIdx, n - 2);
        double fr = sIdx - a;
        var r = new double[width];
        for (int i = 0; i < width; i++)
            r[i] = V[a * width + i] * (1 - fr) + V[(a + 1) * width + i] * fr;
        return r;
    }

    private static void Step(double[] x, double[] u, Dynamics6Dof.Params p, double h)
    {
        Span<double> k1 = stackalloc double[NX], k2 = stackalloc double[NX], k3 = stackalloc double[NX], k4 = stackalloc double[NX], tmp = stackalloc double[NX];
        Dynamics6Dof.Eval(x, u, p, k1);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k1[i];
        Dynamics6Dof.Eval(tmp, u, p, k2);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k2[i];
        Dynamics6Dof.Eval(tmp, u, p, k3);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + h * k3[i];
        Dynamics6Dof.Eval(tmp, u, p, k4);
        for (int i = 0; i < NX; i++) x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        double norm = Math.Sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
        for (int i = 6; i < 10; i++) x[i] /= norm;
    }
}

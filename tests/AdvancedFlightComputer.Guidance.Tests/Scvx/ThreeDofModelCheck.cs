using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// --3dof-model: the KSA point-mass model on its own, before any solver sees it.
///
///  1. JACOBIANS. Forward-mode AD against central differences, at states across the glide and the burn - this is the whole linearisation, tables, nozzle and frame terms included.
///  2. THE FRAME. An engine-off, engine-first coast integrated in the rotating site frame against the same coast integrated in CCI by DragCoastSystem, the impact predictor's model, then carried into the site frame. Gravity, Coriolis, the centrifugal term and the co-rotating air all have to be right for the two to land on each other.
///  3. FORCES. Lift along the engine end's tilt with the table's magnitude, and the nozzle's thrust law.
/// </summary>
internal static class ThreeDofModelCheck
{
    private const int NX = PointMass3Dof.NX;
    private const int NU = PointMass3Dof.NU;

    public static int Run()
    {
        bool ok = true;
        ok &= CheckJacobians();
        ok &= CheckFrame();
        ok &= CheckForces();
        Console.WriteLine();
        Console.WriteLine(ok ? "PASS - the KSA point-mass model" : "FAIL");
        return ok ? 0 : 1;
    }

    private static bool CheckJacobians()
    {
        KsaPointMassModel model = ThreeDofScenario.Model() with { DragScale = 1.07, LiftScale = 0.9 };
        var rng = new Random(7);
        double worst = 0;
        string where = "";
        Console.WriteLine("Jacobians: forward-mode AD against central differences");
        for (int c = 0; c < 12; c++)
        {
            // Glide-like and burn-like states: 0.5-30 km up, Mach 0.3-4, tilted 0-14 degrees off the airflow.
            double alt = 500 + rng.NextDouble() * 29500;
            double speed = 100 + rng.NextDouble() * 1250;
            double fpa = -(20 + rng.NextDouble() * 65) * Math.PI / 180;
            double az = rng.NextDouble() * 2 * Math.PI;
            double[] v = [speed * Math.Cos(fpa) * Math.Cos(az), speed * Math.Cos(fpa) * Math.Sin(az), speed * Math.Sin(fpa)];
            double[] x = [rng.NextDouble() * 8000 - 4000, rng.NextDouble() * 8000 - 4000, alt, v[0], v[1], v[2],
                          ThreeDofScenario.DryMass + rng.NextDouble() * 5000];
            double tilt = rng.NextDouble() * 14 * Math.PI / 180;
            double[] h = ThreeDofScenario.Normalise(v);
            double[] side = ThreeDofScenario.Normalise(ThreeDofScenario.Cross(h, [rng.NextDouble() - 0.5, rng.NextDouble() - 0.5, 1]));
            double[] b = [-Math.Cos(tilt) * h[0] + Math.Sin(tilt) * side[0], -Math.Cos(tilt) * h[1] + Math.Sin(tilt) * side[1],
                          -Math.Cos(tilt) * h[2] + Math.Sin(tilt) * side[2]];
            double[] u = [b[0], b[1], b[2], c % 2 == 0 ? 0.0 : 0.4 + 0.6 * rng.NextDouble()];

            var f = new double[NX];
            var A = new double[NX * NX];
            var B = new double[NX * NU];
            PointMass3Dof.Jacobian(model, x, u, f, A, B);

            var fp = new double[NX];
            var fm = new double[NX];
            for (int col = 0; col < NX + NU; col++)
            {
                double[] xp = (double[])x.Clone(), xm = (double[])x.Clone();
                double[] up = (double[])u.Clone(), um = (double[])u.Clone();
                double step;
                if (col < NX)
                {
                    step = 1e-6 * Math.Max(1.0, Math.Abs(x[col]));
                    xp[col] += step;
                    xm[col] -= step;
                }
                else
                {
                    step = 1e-7;
                    up[col - NX] += step;
                    um[col - NX] -= step;
                }
                PointMass3Dof.Eval(model, xp, up, fp);
                PointMass3Dof.Eval(model, xm, um, fm);
                for (int r = 0; r < NX; r++)
                {
                    double fd = (fp[r] - fm[r]) / (2 * step);
                    double ad = col < NX ? A[r * NX + col] : B[r * NU + col - NX];
                    double scale = Math.Max(1e-3, Math.Max(Math.Abs(fd), Math.Abs(ad)));
                    // Rows of the acceleration are O(10) m/s^2; compare relative to a floor that ignores round-off on slopes that are zero.
                    double err = Math.Abs(fd - ad) / Math.Max(scale, 1e-2);
                    if (err > worst)
                    {
                        worst = err;
                        where = $"case {c}, d f[{r}] / d {(col < NX ? "x" : "u")}[{(col < NX ? col : col - NX)}]: AD {ad:E4}, FD {fd:E4}";
                    }
                }
            }
        }
        bool pass = worst < 1e-5;
        Console.WriteLine($"  12 states, worst relative error {worst:E2}  ({where})  {(pass ? "ok" : "FAIL")}");
        return pass;
    }

    private static bool CheckFrame()
    {
        Console.WriteLine();
        Console.WriteLine("frame: a 60 s engine-first coast, site frame against DragCoastSystem in CCI");
        KsaPointMassModel model = ThreeDofScenario.Model(lift: false);
        ThreeDofScenario.Axes(out double[] ex, out double[] ey, out double[] ez);
        double R = ThreeDofScenario.Radius, w = ThreeDofScenario.Omega;
        double mass = ThreeDofScenario.WetMass;

        double[] rl = [-15_000, 4_000, 38_000];
        double[] vl = [650, -120, -700];

        // Into CCI: position from the site, inertial velocity = surface velocity + omega x r.
        double[] origin = [ez[0] * R, ez[1] * R, ez[2] * R];
        double[] rc = Add(origin, Local(rl, ex, ey, ez));
        double[] omega = [0, 0, w];
        double[] vc = Add(Local(vl, ex, ey, ez), ThreeDofScenario.Cross(omega, rc));

        var coast = new DragCoastSystem
        {
            Mu = ThreeDofScenario.Mu,
            OmegaZ = w,
            MeanRadius = R,
            AreaOverMass = ThreeDofScenario.Area / mass,
            Alpha = 0.0,
            Table = model.Drag,
            Atmosphere = model.Atmosphere,
        };

        const double T = 60.0, dt = 0.02;
        double[] xc = [rc[0], rc[1], rc[2], vc[0], vc[1], vc[2]];
        double[] xl = [rl[0], rl[1], rl[2], vl[0], vl[1], vl[2], mass];
        for (double t = 0; t < T - 1e-9; t += dt)
        {
            xc = Rk4(xc, dt, (s, d) => CoastDeriv(coast, s, d));
            xl = Rk4(xl, dt, (s, d) => ModelDeriv(model, s, d));
        }

        // Back into the site frame as it stands at T: everything rotated by omega T about CCI z.
        double ang = w * T;
        double[] Rz(double[] a) => [Math.Cos(ang) * a[0] - Math.Sin(ang) * a[1], Math.Sin(ang) * a[0] + Math.Cos(ang) * a[1], a[2]];
        double[] exT = Rz(ex), eyT = Rz(ey), ezT = Rz(ez), originT = Rz(origin);
        double[] rT = [xc[0], xc[1], xc[2]];
        double[] vSurf = Sub([xc[3], xc[4], xc[5]], ThreeDofScenario.Cross(omega, rT));
        double[] d = Sub(rT, originT);
        double[] rFromCci = [ThreeDofScenario.Dot(d, exT), ThreeDofScenario.Dot(d, eyT), ThreeDofScenario.Dot(d, ezT)];
        double[] vFromCci = [ThreeDofScenario.Dot(vSurf, exT), ThreeDofScenario.Dot(vSurf, eyT), ThreeDofScenario.Dot(vSurf, ezT)];

        double ep = Math.Sqrt(Sq(rFromCci[0] - xl[0]) + Sq(rFromCci[1] - xl[1]) + Sq(rFromCci[2] - xl[2]));
        double ev = Math.Sqrt(Sq(vFromCci[0] - xl[3]) + Sq(vFromCci[1] - xl[4]) + Sq(vFromCci[2] - xl[5]));
        double travelled = Math.Sqrt(Sq(xl[0] - rl[0]) + Sq(xl[1] - rl[1]) + Sq(xl[2] - rl[2]));

        // The same coast with the rotation terms left out shows what they are worth.
        KsaPointMassModel flat = model with { OmegaX = 0, OmegaY = 0, OmegaZ = 0 };
        double[] xf = [rl[0], rl[1], rl[2], vl[0], vl[1], vl[2], mass];
        for (double t = 0; t < T - 1e-9; t += dt)
            xf = Rk4(xf, dt, (s, dd) => ModelDeriv(flat, s, dd));
        double eRot = Math.Sqrt(Sq(rFromCci[0] - xf[0]) + Sq(rFromCci[1] - xf[1]) + Sq(rFromCci[2] - xf[2]));

        bool pass = ep < 0.01 && ev < 1e-3;
        Console.WriteLine($"  travelled {travelled / 1000:F1} km; site frame vs CCI: position {ep:E2} m, velocity {ev:E2} m/s  {(pass ? "ok" : "FAIL")}");
        Console.WriteLine($"  (without the Coriolis and centrifugal terms the same coast misses by {eRot:F0} m)");
        return pass;
    }

    private static bool CheckForces()
    {
        Console.WriteLine();
        Console.WriteLine("forces: lift direction and size, and the nozzle law");
        KsaPointMassModel model = ThreeDofScenario.Model() with { AlphaRounding = 1e-9 };
        KsaPointMassModel noAero = model with { Drag = null, LiftSlope = null };
        double alt = 12_000, speed = 600;
        double[] x = [0, 0, alt, 0, 0, -speed, ThreeDofScenario.WetMass];
        double alphaDeg = 8;
        double a = alphaDeg * Math.PI / 180;
        // Falling straight down, engine first, the engine end tilted towards -x: b = up tilted towards +x.
        double[] b = [Math.Sin(a), 0, Math.Cos(a)];
        var f = new double[NX];
        var f0 = new double[NX];
        PointMass3Dof.Eval(model, x, [b[0], b[1], b[2], 0.0], f);
        PointMass3Dof.Eval(noAero, x, [b[0], b[1], b[2], 0.0], f0);
        double m = x[PointMass3Dof.IM];
        double liftX = (f[3] - f0[3]) * m;           // across the airflow
        double q = 0.5 * model.Atmosphere!.Density(alt) * speed * speed;
        double mach = model.Atmosphere.Mach(speed);
        double[] machGrid = AeroTable.DefaultMachBreakpoints, alphaGrid = AeroTable.DefaultAlphaBreakpointsDeg;
        var clTable = new AeroTable(machGrid, alphaGrid, ThreeDofScenario.LiftTable(machGrid, alphaGrid));
        double expected = -q * ThreeDofScenario.Area * clTable.Cd(mach, a);   // towards the engine end's tilt, -x
        double liftErr = Math.Abs(liftX - expected) / Math.Abs(expected);
        bool liftOk = liftErr < 0.02 && Math.Sign(liftX) == Math.Sign(expected);
        Console.WriteLine($"  lift at {alphaDeg} deg: {liftX / 1000:F2} kN, table says {expected / 1000:F2} kN (rel {liftErr:E1}), towards the engine end's tilt  {(liftOk ? "ok" : "FAIL")}");

        double pa = model.Atmosphere.Pressure(alt);
        double th = 0.55;
        PointMass3Dof.Eval(noAero, x, [0, 0, 1, th], f);
        PointMass3Dof.Eval(noAero, x, [0, 0, 1, 0.0], f0);   // engine off: no thrust and no back-pressure loss
        double thrust = (f[5] - f0[5]) * m;
        double thrustExpected = th * ThreeDofScenario.VacuumThrust - pa * ThreeDofScenario.ExitArea;
        double thrustErr = Math.Abs(thrust - thrustExpected) / thrustExpected;
        double flow = -f[PointMass3Dof.IM];
        double flowErr = Math.Abs(flow - th * model.MassFlow) / (th * model.MassFlow);
        bool thrustOk = thrustErr < 1e-9 && flowErr < 1e-12;
        Console.WriteLine($"  thrust at {th:P0}: {(thrustExpected) / 1000:F1} kN after a {pa * ThreeDofScenario.ExitArea / 1000:F1} kN back-pressure loss; mass flow {flow:F1} kg/s  {(thrustOk ? "ok" : "FAIL")}");
        return liftOk && thrustOk;
    }

    private static void CoastDeriv(DragCoastSystem sys, double[] s, double[] d)
    {
        Span<Dual> x = stackalloc Dual[6];
        Span<Dual> dx = stackalloc Dual[6];
        for (int i = 0; i < 6; i++) x[i] = new Dual(s[i]);
        sys.Derivative(new Dual(0), x, dx);
        for (int i = 0; i < 6; i++) d[i] = dx[i].V;
    }

    private static void ModelDeriv(KsaPointMassModel model, double[] s, double[] d)
    {
        double sp = Math.Sqrt(s[3] * s[3] + s[4] * s[4] + s[5] * s[5]);
        double[] u = [-s[3] / sp, -s[4] / sp, -s[5] / sp, 0.0];
        PointMass3Dof.Eval(model, s, u, d);
    }

    private static double[] Rk4(double[] x, double h, Action<double[], double[]> deriv)
    {
        int n = x.Length;
        double[] k1 = new double[n], k2 = new double[n], k3 = new double[n], k4 = new double[n], t = new double[n];
        deriv(x, k1);
        for (int i = 0; i < n; i++) t[i] = x[i] + 0.5 * h * k1[i];
        deriv(t, k2);
        for (int i = 0; i < n; i++) t[i] = x[i] + 0.5 * h * k2[i];
        deriv(t, k3);
        for (int i = 0; i < n; i++) t[i] = x[i] + h * k3[i];
        deriv(t, k4);
        var o = new double[n];
        for (int i = 0; i < n; i++) o[i] = x[i] + h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
        return o;
    }

    private static double[] Local(double[] l, double[] ex, double[] ey, double[] ez) =>
        [l[0] * ex[0] + l[1] * ey[0] + l[2] * ez[0], l[0] * ex[1] + l[1] * ey[1] + l[2] * ez[1], l[0] * ex[2] + l[1] * ey[2] + l[2] * ez[2]];
    private static double[] Add(double[] a, double[] b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
    private static double[] Sub(double[] a, double[] b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    private static double Sq(double v) => v * v;
}

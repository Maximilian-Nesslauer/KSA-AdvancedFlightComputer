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
        ok &= CheckLiftingGlide();
        ok &= CheckCrossBiasEstimator();
        ok &= CheckBodyAero();
        ok &= CheckBodyAeroCoast();
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

    /// <summary>
    /// The boostback's planned glide: DragCoastSystem held at 7.5 degrees with its lift up the trajectory, against the 3-DOF model flown at the same angle with its body axis placed so the lift points up. Two independent writings of the same lifting glide must land on each other, and the lift must stretch the range.
    /// </summary>
    private static bool CheckLiftingGlide()
    {
        Console.WriteLine();
        Console.WriteLine("lifting glide: the predictor at 7.5 deg, lift up, against the 3-DOF model flown the same way, 60 s");
        const double Alpha = 7.5 * Math.PI / 180;
        KsaPointMassModel model = ThreeDofScenario.Model() with { AlphaRounding = 1e-9 };
        ThreeDofScenario.Axes(out double[] ex, out double[] ey, out double[] ez);
        double R = ThreeDofScenario.Radius, w = ThreeDofScenario.Omega, mass = ThreeDofScenario.WetMass;
        double[] machGrid = AeroTable.DefaultMachBreakpoints, alphaGrid = AeroTable.DefaultAlphaBreakpointsDeg;
        var clTable = new AeroTable(machGrid, alphaGrid, ThreeDofScenario.LiftTable(machGrid, alphaGrid));

        double[] rl = [-15_000, 4_000, 38_000];
        double[] vl = [650, -120, -700];
        double[] origin = [ez[0] * R, ez[1] * R, ez[2] * R];
        double[] omega = [0, 0, w];
        double[] rc = Add(origin, Local(rl, ex, ey, ez));
        double[] vc = Add(Local(vl, ex, ey, ez), ThreeDofScenario.Cross(omega, rc));

        DragCoastSystem Coast(double alpha) => new()
        {
            Mu = ThreeDofScenario.Mu, OmegaZ = w, MeanRadius = R, AreaOverMass = ThreeDofScenario.Area / mass,
            Alpha = alpha, Table = model.Drag, LiftTable = clTable, Atmosphere = model.Atmosphere,
        };
        DragCoastSystem lifting = Coast(Alpha), ballistic = Coast(0.0);

        // The 3-DOF body axis for lift up: tail Alpha off the airflow, tilted so the engine end leans up - the side positive C_L lifts towards.
        void Glide(double[] s, double[] d)
        {
            double sp = Math.Sqrt(s[3] * s[3] + s[4] * s[4] + s[5] * s[5]);
            double[] h = [s[3] / sp, s[4] / sp, s[5] / sp];
            double[] up = ThreeDofScenario.Normalise([s[0] - model.CentreX, s[1] - model.CentreY, s[2] - model.CentreZ]);
            double uh = ThreeDofScenario.Dot(up, h);
            double[] across = ThreeDofScenario.Normalise([up[0] - uh * h[0], up[1] - uh * h[1], up[2] - uh * h[2]]);
            double[] b = [-Math.Cos(Alpha) * h[0] - Math.Sin(Alpha) * across[0],
                          -Math.Cos(Alpha) * h[1] - Math.Sin(Alpha) * across[1],
                          -Math.Cos(Alpha) * h[2] - Math.Sin(Alpha) * across[2]];
            PointMass3Dof.Eval(model, s, [b[0], b[1], b[2], 0.0], d);
        }

        const double T = 60.0, dt = 0.02;
        double[] xc = [rc[0], rc[1], rc[2], vc[0], vc[1], vc[2]];
        double[] xb = (double[])xc.Clone();
        double[] xl = [rl[0], rl[1], rl[2], vl[0], vl[1], vl[2], mass];
        for (double t = 0; t < T - 1e-9; t += dt)
        {
            xc = Rk4(xc, dt, (s, d) => CoastDeriv(lifting, s, d));
            xb = Rk4(xb, dt, (s, d) => CoastDeriv(ballistic, s, d));
            xl = Rk4(xl, dt, Glide);
        }

        double ang = w * T;
        double[] Rz(double[] a) => [Math.Cos(ang) * a[0] - Math.Sin(ang) * a[1], Math.Sin(ang) * a[0] + Math.Cos(ang) * a[1], a[2]];
        double[] exT = Rz(ex), eyT = Rz(ey), ezT = Rz(ez), originT = Rz(origin);
        double[] ToSite(double[] c)
        {
            double[] d = Sub([c[0], c[1], c[2]], originT);
            return [ThreeDofScenario.Dot(d, exT), ThreeDofScenario.Dot(d, eyT), ThreeDofScenario.Dot(d, ezT)];
        }
        double[] pc = ToSite(xc), pb = ToSite(xb);
        double err = Math.Sqrt(Sq(pc[0] - xl[0]) + Sq(pc[1] - xl[1]) + Sq(pc[2] - xl[2]));
        double stretch = Math.Sqrt(Sq(pc[0] - pb[0]) + Sq(pc[1] - pb[1]) + Sq(pc[2] - pb[2]));
        double climb = pc[2] - pb[2];
        // The two read lift differently between grid points - a spline through C_L against one through C_L / sin(alpha) - so they agree to the table's interpolation, not to round-off.
        bool pass = stretch > 100.0 && climb > 0.0 && err < 0.02 * stretch + 1.0;
        Console.WriteLine($"  the lift moves the glide {stretch:F0} m ({climb:F0} m higher) against the ballistic coast; predictor vs 3-DOF {err:F1} m  {(pass ? "ok" : "FAIL")}");
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

    /// <summary>
    /// The estimator against a craft whose lift falls short of the table's by a FIXED force across the airflow - what the 2026-11-18 booster measured, about 1 m/s^2 at 1 deg and at 7 deg alike. Flown at 1, 3 and 7 deg in turn, 30 s in all at the re-solve cadence: the learned bias must reach the true force within 5 %, hold it however the angle moves (the lift scale this replaced swung from 0.5 to 1.1 on the same input), and leave the drag scale at 1.
    /// </summary>
    private static bool CheckCrossBiasEstimator()
    {
        Console.WriteLine("cross-flow bias estimator: a fixed 1 m/s^2 shortfall across the airflow at 1, 3 and 7 deg");
        KsaPointMassModel model = ThreeDofScenario.Model();
        // A glide at 8 km, 160 m/s, falling steeply downrange.
        double[] x = [-3000, 0, 8000, 60, 0, -148, ThreeDofScenario.WetMass * 0.6];
        double sp = Math.Sqrt(x[3] * x[3] + x[5] * x[5]);
        double[] h = [x[3] / sp, 0, x[5] / sp];
        // The shortfall points down the lift's direction for a tilt in the x-z plane: across the airflow, in that plane.
        double[] across = [-h[2], 0, h[0]];
        const double shortfall = 1.0;
        var est = new AeroScaleEstimator();
        double[] alphasDeg = [1.0, 3.0, 7.0];
        double worstHold = 0.0, learned = 0.0;
        for (int step = 0; step < 300; step++)
        {
            double a = alphasDeg[(step / 100) % 3] * Math.PI / 180.0;
            // Engine end leading, tilted off the airflow by a in the x-z plane.
            double[] b = [-h[0] * Math.Cos(a) + across[0] * Math.Sin(a), 0, -h[2] * Math.Cos(a) + across[2] * Math.Sin(a)];
            double[] aero = AeroScaleEstimator.PredictedAero(model, x, b);
            double[] measured = [aero[0] - shortfall * across[0], aero[1], aero[2] - shortfall * across[2]];
            est.Update(model, x, b, 0.0, measured, 0.1);
            learned = est.BiasAcceleration(model, x);
            if (step >= 100)
                worstHold = Math.Max(worstHold, Math.Abs(learned - shortfall));
        }
        bool ok = worstHold < 0.05 * shortfall && Math.Abs(est.DragScale - 1.0) < 1e-6;
        Console.WriteLine($"  learned {learned:F3} m/s^2 of {shortfall:F3}, worst after the first 10 s {worstHold:F3} m/s^2 off, drag scale {est.DragScale:F4}  {(ok ? "ok" : "FAIL")}");
        return ok;
    }

    /// <summary>
    /// The body-axis table (BodyAeroTable) read by the model at a held roll. Built from the scenario's own axisymmetric drag and lift - drag against the airflow, lift along the engine end's tilt, both functions of the total angle only - it must give the old model's force at tilts in pitch, in yaw and between, to within the tables' interpolation. With a fixed side force on body +y added at every node, as an asymmetric fin set at zero angle would make, the model must give exactly that force more, along b x d. And its Jacobian in the attitude and the velocity must match central differences.
    /// </summary>
    private static bool CheckBodyAero()
    {
        Console.WriteLine();
        Console.WriteLine("body-axis aero table at a held roll: axisymmetric agreement, an asymmetric side force, and its slopes");
        KsaPointMassModel model = ThreeDofScenario.Model();
        double[] mach = AeroTable.DefaultMachBreakpoints;
        double[] alphaGrid = AeroTable.DefaultAlphaBreakpointsDeg;
        var drag = new AeroTable();
        var lift = new AeroTable(mach, alphaGrid, ThreeDofScenario.LiftTable(mach, alphaGrid));
        double[] angles = { -20, -16, -12, -8, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 8, 12, 16, 20 };
        const double side = 0.05;
        BodyAeroTable Build(double sideCoefficient)
        {
            var c = new double[mach.Length * angles.Length * angles.Length * 3];
            for (int i = 0; i < mach.Length; i++)
                for (int ja = 0; ja < angles.Length; ja++)
                    for (int jb = 0; jb < angles.Length; jb++)
                    {
                        (double ux, double uy, double uz) = BodyAeroTable.AirDirection(angles[ja] * Math.PI / 180, angles[jb] * Math.PI / 180);
                        double total = Math.Acos(Math.Clamp(-ux, -1, 1));
                        double cd = drag.Cd(mach[i], total), cl = lift.Cd(mach[i], total);
                        // Lift along -p, p = x - (x . u) u with x the body axis (1, 0, 0): the engine end's tilt off the airflow.
                        double px = 1 - ux * ux, py = -ux * uy, pz = -ux * uz;
                        double pn = Math.Sqrt(px * px + py * py + pz * pz);
                        double lx = pn > 1e-12 ? -px / pn : 0, ly = pn > 1e-12 ? -py / pn : 0, lz = pn > 1e-12 ? -pz / pn : 0;
                        int n = ((i * angles.Length + ja) * angles.Length + jb) * 3;
                        c[n] = -cd * ux + cl * lx;
                        c[n + 1] = -cd * uy + cl * ly + sideCoefficient;
                        c[n + 2] = -cd * uz + cl * lz;
                    }
            return new BodyAeroTable(mach, angles, angles, c);
        }
        // Held wings level along +x of the site frame: d = (1, 0, 0).
        KsaPointMassModel body = model with { BodyAero = Build(0.0), RollRefX = 1.0 };
        KsaPointMassModel asym = model with { BodyAero = Build(side), RollRefX = 1.0 };

        double alt = 6000, speed = 160;
        double worstRel = 0, worstSide = 0;
        bool ok = true;
        // Falling steeply towards +x; the body axis tilted off retrograde in pitch (the x-z plane), in yaw (towards y) and between.
        double[] h0 = [Math.Sin(20 * Math.PI / 180), 0, -Math.Cos(20 * Math.PI / 180)];
        (double pitch, double yaw)[] tilts = { (2, 0), (5, 0), (-3, 0), (0, 3), (0, -6), (3, 3), (-4, 2), (8, -5) };
        foreach ((double pitch, double yaw) in tilts)
        {
            // b: retrograde, turned by pitch about the site y axis and by yaw about the axis across both.
            double[] r = [-h0[0], -h0[1], -h0[2]];
            double pr = pitch * Math.PI / 180, yr = yaw * Math.PI / 180;
            double[] pitched = [r[0] * Math.Cos(pr) + r[2] * Math.Sin(pr), r[1], -r[0] * Math.Sin(pr) + r[2] * Math.Cos(pr)];
            double[] yawed = [pitched[0], pitched[1] * Math.Cos(yr) - Math.Sin(yr) * Math.Sqrt(pitched[0] * pitched[0] + pitched[2] * pitched[2]), pitched[2]];
            double bn = Math.Sqrt(yawed[0] * yawed[0] + yawed[1] * yawed[1] + yawed[2] * yawed[2]);
            double[] b = [yawed[0] / bn, yawed[1] / bn, yawed[2] / bn];
            Force(model, alt, speed, h0, b, out double ox, out double oy, out double oz);
            Force(body, alt, speed, h0, b, out double nx, out double ny, out double nz);
            Force(asym, alt, speed, h0, b, out double sx, out double sy, out double sz);
            double on = Math.Sqrt(ox * ox + oy * oy + oz * oz);
            double rel = Math.Sqrt((nx - ox) * (nx - ox) + (ny - oy) * (ny - oy) + (nz - oz) * (nz - oz)) / on;
            worstRel = Math.Max(worstRel, rel);
            // The extra force must be q S side along b x d.
            double q = 0.5 * model.Atmosphere!.Density(alt) * speed * speed;
            double yx = b[1] * 0 - b[2] * 0, yy = b[2] * 1 - b[0] * 0, yz = b[0] * 0 - b[1] * 1;
            double yn = Math.Sqrt(yx * yx + yy * yy + yz * yz);
            double ex = q * ThreeDofScenario.Area * side * yx / yn, ey = q * ThreeDofScenario.Area * side * yy / yn, ez = q * ThreeDofScenario.Area * side * yz / yn;
            double dside = Math.Sqrt(Sq(sx - nx - ex) + Sq(sy - ny - ey) + Sq(sz - nz - ez)) / Math.Sqrt(ex * ex + ey * ey + ez * ez);
            worstSide = Math.Max(worstSide, dside);
        }
        bool agree = worstRel < 0.02, sideOk = worstSide < 1e-6;
        Console.WriteLine($"  axisymmetric table vs the old model, 8 tilts in pitch and yaw: worst {worstRel:E1} of the force  {(agree ? "ok" : "FAIL")}");
        Console.WriteLine($"  a fixed side force on body +y: reproduced along b x d to {worstSide:E1}  {(sideOk ? "ok" : "FAIL")}");
        ok &= agree && sideOk;

        // Slopes: AD against central differences in the attitude and the velocity.
        double[] x = [-3000, 200, alt, speed * h0[0], 0, speed * h0[2], ThreeDofScenario.WetMass];
        double[] u = [-h0[0] + 0.05, 0.03, -h0[2], 0.0];
        double worstJac = 0;
        var A = new double[NX * NX];
        var B = new double[NX * PointMass3Dof.NU];
        var fAd = new double[NX];
        PointMass3Dof.Jacobian(asym, x, u, fAd, A, B);
        for (int col = 0; col < NX + PointMass3Dof.NU; col++)
        {
            if (col < NX && (col < 3 || col == PointMass3Dof.IM)) continue;   // the velocity and the attitude are what the table adds to
            double hstep = col < NX ? 1e-3 : 1e-6;
            double[] xp = (double[])x.Clone(), xm = (double[])x.Clone(), up = (double[])u.Clone(), um = (double[])u.Clone();
            if (col < NX) { xp[col] += hstep; xm[col] -= hstep; } else { up[col - NX] += hstep; um[col - NX] -= hstep; }
            var fp = new double[NX];
            var fm = new double[NX];
            PointMass3Dof.Eval(asym, xp, up, fp);
            PointMass3Dof.Eval(asym, xm, um, fm);
            for (int row = 3; row < 6; row++)
            {
                double fd = (fp[row] - fm[row]) / (2 * hstep);
                double ad = col < NX ? A[row * NX + col] : B[row * PointMass3Dof.NU + col - NX];
                double scale = Math.Max(Math.Abs(fd), 1e-3);
                worstJac = Math.Max(worstJac, Math.Abs(ad - fd) / scale);
            }
        }
        bool jacOk = worstJac < 1e-4;
        Console.WriteLine($"  AD against central differences in velocity and attitude: worst {worstJac:E1}  {(jacOk ? "ok" : "FAIL")}");
        return ok && jacOk;
    }

    /// <summary>The model's aerodynamic force, N, at an altitude, air speed along h and body axis b.</summary>
    private static void Force(KsaPointMassModel model, double alt, double speed, double[] h, double[] b, out double fx, out double fy, out double fz)
    {
        model.AeroForce(new Dual(alt), new Dual(speed * h[0]), new Dual(speed * h[1]), new Dual(speed * h[2]),
                        new Dual(b[0]), new Dual(b[1]), new Dual(b[2]), out Dual dx, out Dual dy, out Dual dz);
        fx = dx.V;
        fy = dy.V;
        fz = dz.V;
    }

    /// <summary>
    /// The landing-point prediction on the body-axis table (DragCoastSystem.BodyAero). From a table built out of the roll-averaged drag and lift it must give the old coast's acceleration - engine first, and tilted up the trajectory at a planned angle - to within the tables' interpolation. A fixed side force on body +y must come out along b x w with the roll reference turned with the ground to the time asked, so a prediction minutes long keeps the roll the craft holds. And its slope in the velocity must match central differences.
    /// </summary>
    private static bool CheckBodyAeroCoast()
    {
        Console.WriteLine();
        Console.WriteLine("landing-point prediction on the body-axis table: the old coast from an axisymmetric table, a side force turning with the ground, and its slopes");
        KsaPointMassModel model = ThreeDofScenario.Model();
        ThreeDofScenario.Axes(out double[] ex, out double[] ey, out double[] ez);
        double R = ThreeDofScenario.Radius, w = ThreeDofScenario.Omega, mass = ThreeDofScenario.WetMass;
        double[] mach = AeroTable.DefaultMachBreakpoints, alphaGrid = AeroTable.DefaultAlphaBreakpointsDeg;
        var lift = new AeroTable(mach, alphaGrid, ThreeDofScenario.LiftTable(mach, alphaGrid));
        AeroTable drag = model.Drag!;
        double[] angles = { -20, -16, -12, -8, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 8, 12, 16, 20 };
        const double side = 0.05;
        BodyAeroTable Build(double sideCoefficient)
        {
            var c = new double[mach.Length * angles.Length * angles.Length * 3];
            for (int i = 0; i < mach.Length; i++)
                for (int ja = 0; ja < angles.Length; ja++)
                    for (int jb = 0; jb < angles.Length; jb++)
                    {
                        (double ux, double uy, double uz) = BodyAeroTable.AirDirection(angles[ja] * Math.PI / 180, angles[jb] * Math.PI / 180);
                        double total = Math.Acos(Math.Clamp(-ux, -1, 1));
                        double cd = drag.Cd(mach[i], total), cl = lift.Cd(mach[i], total);
                        double px = 1 - ux * ux, py = -ux * uy, pz = -ux * uz;
                        double pn = Math.Sqrt(px * px + py * py + pz * pz);
                        double lx = pn > 1e-12 ? -px / pn : 0, ly = pn > 1e-12 ? -py / pn : 0, lz = pn > 1e-12 ? -pz / pn : 0;
                        int n = ((i * angles.Length + ja) * angles.Length + jb) * 3;
                        c[n] = -cd * ux + cl * lx;
                        c[n + 1] = -cd * uy + cl * ly + sideCoefficient;
                        c[n + 2] = -cd * uz + cl * lz;
                    }
            return new BodyAeroTable(mach, angles, angles, c);
        }
        BodyAeroTable sym = Build(0.0), asym = Build(side);

        double[] rl = [-6_000, 1_500, 9_000];
        double[] vl = [180, -40, -260];
        double[] origin = [ez[0] * R, ez[1] * R, ez[2] * R];
        double[] rc = Add(origin, Local(rl, ex, ey, ez));
        double[] vc = Add(Local(vl, ex, ey, ez), ThreeDofScenario.Cross([0, 0, w], rc));
        // The held roll: towards the site and down, as GuidanceWindow.ReturnRollRefCci.
        double[] wRef = ThreeDofScenario.Normalise([ex[0] - ez[0], ex[1] - ez[1], ex[2] - ez[2]]);

        DragCoastSystem Coast(double alpha, BodyAeroTable? body) => new()
        {
            Mu = ThreeDofScenario.Mu, OmegaZ = w, MeanRadius = R, AreaOverMass = ThreeDofScenario.Area / mass,
            Alpha = alpha, Table = drag, LiftTable = lift, Atmosphere = model.Atmosphere,
            BodyAero = body, RollRefX = body != null ? wRef[0] : 0, RollRefY = body != null ? wRef[1] : 0, RollRefZ = body != null ? wRef[2] : 0,
        };
        double[] Accel(DragCoastSystem sys, double t, int seed = -1)
        {
            var x = new Dual[6];
            for (int k = 0; k < 6; k++)
                x[k] = new Dual(k < 3 ? rc[k] : vc[k - 3], k == seed ? 1.0 : 0.0);
            var d = new Dual[6];
            sys.Derivative(new Dual(t), x, d);
            return seed < 0 ? [d[3].V, d[4].V, d[5].V] : [d[3].D, d[4].D, d[5].D];
        }

        bool ok = true;
        double worst = 0;
        foreach (double deg in new[] { 0.0, 3.0, 7.5 })
        {
            double alpha = deg * Math.PI / 180;
            double[] gOld = Accel(Coast(alpha, null), 0), gNew = Accel(Coast(alpha, sym), 0);
            double[] gGrav = Accel(Coast(alpha, null) with { Table = null }, 0);
            double[] aOld = [gOld[0] - gGrav[0], gOld[1] - gGrav[1], gOld[2] - gGrav[2]];
            double diff = Math.Sqrt(Sq(gNew[0] - gOld[0]) + Sq(gNew[1] - gOld[1]) + Sq(gNew[2] - gOld[2]));
            worst = Math.Max(worst, diff / Math.Sqrt(ThreeDofScenario.Dot(aOld, aOld)));
        }
        bool agree = worst < 0.02;
        Console.WriteLine($"  axisymmetric table vs the old coast at 0, 3 and 7.5 deg: worst {worst:E1} of the aero force  {(agree ? "ok" : "FAIL")}");
        ok &= agree;

        // The side force, engine first: q S side / m along (-h) x w(t), w turned by omega t about the spin axis.
        double worstSide = 0, unturned = 0;
        double[] air = [vc[0] + w * rc[1], vc[1] - w * rc[0], vc[2]];
        double speed = Math.Sqrt(ThreeDofScenario.Dot(air, air));
        double[] h = [air[0] / speed, air[1] / speed, air[2] / speed];
        double rho = model.Atmosphere!.Density(Math.Sqrt(ThreeDofScenario.Dot(rc, rc)) - R);
        double k = 0.5 * rho * speed * speed * ThreeDofScenario.Area / mass * side;
        foreach (double t in new[] { 0.0, 300.0 })
        {
            double[] s0 = Accel(Coast(0, sym), t), s1 = Accel(Coast(0, asym), t);
            double c = Math.Cos(w * t), sn = Math.Sin(w * t);
            double[] wt = [c * wRef[0] - sn * wRef[1], sn * wRef[0] + c * wRef[1], wRef[2]];
            double[] y = ThreeDofScenario.Normalise(ThreeDofScenario.Cross([-h[0], -h[1], -h[2]], wt));
            double[] y0 = ThreeDofScenario.Normalise(ThreeDofScenario.Cross([-h[0], -h[1], -h[2]], wRef));
            double[] got = [s1[0] - s0[0], s1[1] - s0[1], s1[2] - s0[2]];
            worstSide = Math.Max(worstSide, Math.Sqrt(Sq(got[0] - k * y[0]) + Sq(got[1] - k * y[1]) + Sq(got[2] - k * y[2])) / k);
            if (t > 0)
                unturned = Math.Sqrt(Sq(got[0] - k * y0[0]) + Sq(got[1] - k * y0[1]) + Sq(got[2] - k * y0[2])) / k;
        }
        bool sideOk = worstSide < 1e-6 && unturned > 1e-3;
        Console.WriteLine($"  a side force on body +y, at t 0 and 300 s: along b x w(t) to {worstSide:E1} (unturned w off by {unturned:E1})  {(sideOk ? "ok" : "FAIL")}");
        ok &= sideOk;

        // Slopes in the velocity, at a planned angle, through the table: AD against central differences.
        DragCoastSystem tilted = Coast(3.0 * Math.PI / 180, asym);
        double worstJac = 0;
        for (int col = 3; col < 6; col++)
        {
            double[] ad = Accel(tilted, 0, col);
            const double hstep = 1e-3;
            double[] keep = (double[])vc.Clone();
            vc[col - 3] = keep[col - 3] + hstep;
            double[] fp = Accel(tilted, 0);
            vc[col - 3] = keep[col - 3] - hstep;
            double[] fm = Accel(tilted, 0);
            vc[col - 3] = keep[col - 3];
            for (int row = 0; row < 3; row++)
            {
                double fd = (fp[row] - fm[row]) / (2 * hstep);
                worstJac = Math.Max(worstJac, Math.Abs(ad[row] - fd) / Math.Max(Math.Abs(fd), 1e-4));
            }
        }
        bool jacOk = worstJac < 1e-4;
        Console.WriteLine($"  AD against central differences in velocity: worst {worstJac:E1}  {(jacOk ? "ok" : "FAIL")}");
        return ok && jacOk;
    }
}

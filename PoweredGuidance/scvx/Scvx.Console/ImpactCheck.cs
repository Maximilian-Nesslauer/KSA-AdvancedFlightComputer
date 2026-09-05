using PoweredGuidance.Flight;
using PoweredGuidance.Numerics;

/// <summary>
/// Does the impact predictor integrate the right physics, and does its answer
/// differentiate?
///
/// Two separate questions and both matter. The first is ordinary numerical work:
/// fourth order really is fourth order, energy is conserved when drag is off, the
/// crossing refinement lands on the target radius rather than near it. The second is
/// the one the whole design exists for - that d(impact point)/d(initial state) falls
/// out of a seeded sweep, INCLUDING the part that comes through the time of flight,
/// which is the term an event-terminated integration usually loses.
///
/// The finite-difference comparison is the decisive test. If it passes, the boostback
/// steering law can have its sensitivities for free later; if it fails, every one of
/// them has to be derived by hand.
/// </summary>
internal static class ImpactCheck
{
    // A KSA-Earth-like world and an F9-class booster, so the numbers below are the
    // ones the mod will actually see rather than a synthetic scenario.
    private const double Mu = 3.986004418e14;
    private const double R = 6371000.0;
    private const double Omega = 7.2921159e-5;
    private const double Mass = 30000.0;      // dry-ish booster
    private const double RefArea = 10.75;     // KSA nose face for a 3.7 m stack

    internal static int Run()
    {
        int fails = 0;
        void Check(string name, bool ok, string detail = "")
        {
            Console.WriteLine($"  {(ok ? "PASS" : "FAIL")}  {name,-48} {detail}");
            if (!ok) fails++;
        }

        Console.WriteLine("IMPACT PREDICTOR: RK4 to the surface, under forward-mode AD");
        Console.WriteLine();

        var atm = ExponentialAtmosphere.Earth;
        var tab = new AeroTable();

        DragCoastSystem Sys(bool drag) => new()
        {
            Mu = Mu,
            OmegaZ = Omega,
            MeanRadius = R,
            AreaOverMass = RefArea / Mass,
            Alpha = 0.0,                      // retrograde-first: engine into the wind
            Table = drag ? tab : null,
            Atmosphere = drag ? atm : null,
        };

        // A booster past apogee on a boostback-ish arc: 90 km up, coming down and
        // downrange. Chosen so it is unambiguously suborbital and spends a long time
        // in the air, which is where the model actually does something.
        double[] X0 =
        {
            R + 90000.0, 0.0, 0.0,
            -350.0, 1500.0, 0.0,
        };

        Span<Dual> scratch = stackalloc Dual[ImpactPredictor.ScratchLength];
        var pathBuf = new double[ImpactPredictor.PathStrideDoubles * 4096];

        static void Load(Span<Dual> x, double[] src, int seed = -1)
        {
            for (int i = 0; i < src.Length; i++)
                x[i] = seed == i ? Dual.Seed(src[i]) : new Dual(src[i]);
        }

        // ---- it flies, and it lands ------------------------------------
        Span<Dual> x = stackalloc Dual[6];
        Load(x, X0);
        var opt = ImpactOptions.Default(R);
        ImpactPrediction p = ImpactPredictor.Predict(Sys(true), x, opt, scratch, pathBuf);

        Console.WriteLine($"    status        {p.Status}");
        Console.WriteLine($"    time of flight{p.TimeOfFlight.V,9:F2} s");
        Console.WriteLine($"    steps         {p.Steps,9}");
        Console.WriteLine($"    impact speed  {Speed(p),9:F1} m/s");
        Console.WriteLine($"    path points   {p.PathPoints,9}");
        Console.WriteLine();

        Check("suborbital arc reaches the ground", p.Hit, p.Status.ToString());

        double hitR = Math.Sqrt(p.Rx.V * p.Rx.V + p.Ry.V * p.Ry.V + p.Rz.V * p.Rz.V);
        Check("lands ON the target radius", Math.Abs(hitR - R) < 1.0,
              $"|r| - R = {hitR - R:F4} m");

        // ---- drag is doing something -----------------------------------
        //
        // NOT "drag shortens the range", which is the obvious assertion and is FALSE
        // for this regime. Drag retards the descent as well as the downrange motion,
        // and gravity keeps re-supplying vertical velocity while nothing re-supplies
        // horizontal - so a steep coast spends much longer in the air and drifts
        // FURTHER, not less far. Measured below; the sign flips for shallow, fast
        // entries where killing the horizontal velocity dominates instead.
        //
        // This is the whole reason a drag-aware impact point is worth having: the
        // vacuum IIP is not merely imprecise here, it is wrong in a direction most
        // people would guess backwards.
        Load(x, X0);
        ImpactPrediction vac = ImpactPredictor.Predict(Sys(false), x, opt, scratch, default);
        double dragRange = Downrange(p, X0), vacRange = Downrange(vac, X0);
        Check("drag moves the impact point substantially",
              vac.Hit && Math.Abs(dragRange - vacRange) > 5000.0,
              $"drag {dragRange / 1000:F1} km vs vacuum {vacRange / 1000:F1} km "
            + $"({(dragRange - vacRange) / 1000:+0.0;-0.0} km, "
            + $"{100 * (dragRange / vacRange - 1):+0.0;-0.0}%)");
        Check("a steep coast lands LONG of the vacuum point", dragRange > vacRange,
              "drag delays the descent more than it kills downrange speed");
        Check("drag slows the impact", Speed(p) < Speed(vac),
              $"{Speed(p):F0} vs {Speed(vac):F0} m/s");

        // ---- energy conservation, drag off ------------------------------
        // The cleanest statement that gravity and the integrator are both right: with
        // no dissipation, specific orbital energy must not move.
        {
            Span<Dual> xi = stackalloc Dual[6];
            Span<Dual> big = stackalloc Dual[Rk4.IntegrateScratch(6)];
            Load(xi, X0);
            double e0 = Energy(xi);
            Span<Dual> outx = stackalloc Dual[6];
            Rk4.Integrate(Sys(false), new Dual(0.0), xi, 120.0, 0.5, outx, big);
            double e1 = Energy(outx);
            Check("specific energy conserved with drag off",
                  Math.Abs(e1 - e0) / Math.Abs(e0) < 1e-12,
                  $"rel drift {Math.Abs(e1 - e0) / Math.Abs(e0):E2} over 120 s");
        }

        // ---- fourth-order convergence -----------------------------------
        // Halving the step must cut the error by ~16. Measured on the impact point of
        // the DRAG case, so it exercises the table and the atmosphere too, against a
        // very fine reference.
        {
            var fine = ImpactOptions.Default(R);
            fine.StepVacuum = 0.05; fine.StepAir = 0.05; fine.MaxSteps = 2_000_000;
            fine.PathStride = 0;
            Load(x, X0);
            ImpactPrediction refP = ImpactPredictor.Predict(Sys(true), x, fine, scratch, default);

            double Err(double step)
            {
                var o = ImpactOptions.Default(R);
                o.StepVacuum = step; o.StepAir = step; o.PathStride = 0;
                o.MaxSteps = 2_000_000;
                Span<Dual> xx = stackalloc Dual[6];
                Span<Dual> sc = stackalloc Dual[ImpactPredictor.ScratchLength];
                Load(xx, X0);
                ImpactPrediction q = ImpactPredictor.Predict(Sys(true), xx, o, sc, default);
                return Dist(q, refP);
            }

            // The DEFAULT options, which are what the overlay actually runs. They are
            // deliberately coarse - an overlay wants milliseconds more than it wants
            // millimetres - so the error they carry is asserted rather than assumed.
            Load(x, X0);
            var deflt = ImpactOptions.Default(R);
            deflt.PathStride = 0;
            ImpactPrediction dp = ImpactPredictor.Predict(Sys(true), x, deflt, scratch, default);
            double defaultErr = Dist(dp, refP);
            Check("default step sizes are accurate to a metre", defaultErr < 1.0,
                  $"{defaultErr:F3} m from converged, in {dp.Steps} steps");

            double e2 = Err(2.0), e1 = Err(1.0), eh = Err(0.5);
            double order1 = Math.Log2(e2 / e1), order2 = Math.Log2(e1 / eh);
            Console.WriteLine($"    impact error   h=2.0 {e2,9:F3} m   h=1.0 {e1,9:F3} m   "
                            + $"h=0.5 {eh,9:F3} m");
            Check("convergence is fourth order", order1 > 3.5 && order2 > 3.5,
                  $"observed order {order1:F2}, {order2:F2}");
        }

        // ---- which way drag moves the point, across the regime ----------
        // Printed rather than asserted: it is a characterisation, and the number that
        // decides whether a drag-aware IIP is worth building. The sign is not
        // constant, so no single correction factor can stand in for integrating.
        Console.WriteLine();
        Console.WriteLine("  drag vs vacuum impact point, by trajectory shape");
        Console.WriteLine($"    {"case",-16} {"gamma",6} {"vacuum",10} {"drag",10} {"delta",10}");
        (string name, double alt, double vr, double vt)[] shapes =
        {
            ("boostback-ish",   90000, -350,  1500),
            ("steep",           90000, -2500,  800),
            ("shallow, fast",   90000,  -50,  4000),
            ("very shallow",   120000,  -20,  6000),
        };
        foreach ((string name, double alt, double vr, double vt) in shapes)
        {
            double[] s0 = { R + alt, 0, 0, vr, vt, 0 };
            var o = ImpactOptions.Default(R);
            o.MaxTime = 6000.0; o.MaxSteps = 200000; o.PathStride = 0;
            Span<Dual> xs = stackalloc Dual[6];
            Load(xs, s0); ImpactPrediction d = ImpactPredictor.Predict(Sys(true), xs, o, scratch, default);
            Load(xs, s0); ImpactPrediction v = ImpactPredictor.Predict(Sys(false), xs, o, scratch, default);
            double gamma = Math.Atan2(-vr, vt) * 180.0 / Math.PI;
            if (!d.Hit || !v.Hit)
            {
                Console.WriteLine($"    {name,-16} {gamma,5:F1}d {"(no impact)",10}");
                continue;
            }
            double dr = Downrange(d, s0), vr2 = Downrange(v, s0);
            Console.WriteLine($"    {name,-16} {gamma,5:F1}d {vr2 / 1000,9:F1}k {dr / 1000,9:F1}k "
                            + $"{(dr - vr2) / 1000,9:+0.0;-0.0}k  {(dr > vr2 ? "long" : "short")}");
        }

        // ---- rotation actually enters -----------------------------------
        {
            DragCoastSystem still = Sys(true);
            still.OmegaZ = 0.0;
            Load(x, X0);
            ImpactPrediction nospin = ImpactPredictor.Predict(still, x, opt, scratch, default);
            double shift = Dist(p, nospin);
            // The co-rotating atmosphere is worth hundreds of m/s of airspeed at these
            // latitudes, so the two impact points must be kilometres apart. If this
            // ever reads zero, the omega x r term has been dropped.
            Check("co-rotating atmosphere moves the impact point", shift > 1000.0,
                  $"{shift / 1000:F2} km");
        }

        // ---- THE test: AD against central differences --------------------
        // Seed one initial-state component at a time and compare every output's
        // derivative against a central difference of the whole prediction. This covers
        // the trajectory AND the stopping condition: time of flight is in the list.
        Console.WriteLine();
        Console.WriteLine("  d(impact)/d(x0), analytic vs central differences");
        {
            double worst = 0; int worstCol = -1; string worstOut = "";
            // Position steps in metres, velocity in m/s; both far above the noise
            // floor of a ~1e-13-relative integration and far below any curvature.
            double[] h = { 20, 20, 20, 0.05, 0.05, 0.05 };

            for (int col = 0; col < 6; col++)
            {
                Span<Dual> xs = stackalloc Dual[6];
                Load(xs, X0, col);
                ImpactPrediction a = ImpactPredictor.Predict(Sys(true), xs, opt, scratch, default);

                double[] plus = (double[])X0.Clone(); plus[col] += h[col];
                double[] minus = (double[])X0.Clone(); minus[col] -= h[col];
                Load(xs, plus); ImpactPrediction pp = ImpactPredictor.Predict(Sys(true), xs, opt, scratch, default);
                Load(xs, minus); ImpactPrediction mm = ImpactPredictor.Predict(Sys(true), xs, opt, scratch, default);

                (string name, double ad, double fd)[] outs =
                {
                    ("Rx",  a.Rx.D,           (pp.Rx.V - mm.Rx.V) / (2 * h[col])),
                    ("Ry",  a.Ry.D,           (pp.Ry.V - mm.Ry.V) / (2 * h[col])),
                    ("Rz",  a.Rz.D,           (pp.Rz.V - mm.Rz.V) / (2 * h[col])),
                    ("Vx",  a.Vx.D,           (pp.Vx.V - mm.Vx.V) / (2 * h[col])),
                    ("Vy",  a.Vy.D,           (pp.Vy.V - mm.Vy.V) / (2 * h[col])),
                    ("Vz",  a.Vz.D,           (pp.Vz.V - mm.Vz.V) / (2 * h[col])),
                    ("tof", a.TimeOfFlight.D, (pp.TimeOfFlight.V - mm.TimeOfFlight.V) / (2 * h[col])),
                };

                foreach ((string name, double ad, double fd) in outs)
                {
                    double rel = Math.Abs(ad - fd) / (1 + Math.Abs(fd));
                    if (rel > worst) { worst = rel; worstCol = col; worstOut = name; }
                }

                Console.WriteLine($"    x0[{col}]  d(Rx)={outs[0].ad,12:F4}  d(Ry)={outs[1].ad,12:F4}"
                                + $"  d(tof)={outs[6].ad,10:F5}");
            }

            // 1e-6 rather than 1e-9: the reference is a central difference of an
            // event-terminated integration, so its own truncation is the floor here,
            // not the AD's accuracy.
            Check("AD == central differences (7 outputs x 6 columns)", worst < 1e-6,
                  $"max rel {worst:E2}" + (worstCol >= 0 ? $" at d({worstOut})/dx0[{worstCol}]" : ""));
        }

        // ---- d(impact)/d(velocity), which is what steering needs --------
        Console.WriteLine();
        Console.WriteLine("  d(ground impact)/d(v0), the 3x3 a boostback law inverts");
        {
            Span<double> dG = stackalloc double[9];
            Span<double> dI = stackalloc double[9];
            Span<double> dT = stackalloc double[3];
            ImpactPrediction nom = ImpactPredictor.VelocityJacobian(
                Sys(true), X0, opt, scratch, dG, dI, dT);
            Check("velocity Jacobian is available", nom.Hit, nom.Status.ToString());

            Console.WriteLine($"    {"",6} {"d/dvx",11} {"d/dvy",11} {"d/dvz",11}");
            string[] rows = { "gx", "gy", "gz" };
            for (int i = 0; i < 3; i++)
                Console.WriteLine($"    {rows[i],6} {dG[i * 3 + 0],11:F4} {dG[i * 3 + 1],11:F4} "
                                + $"{dG[i * 3 + 2],11:F4}");
            Console.WriteLine($"    {"tof",6} {dT[0],11:F5} {dT[1],11:F5} {dT[2],11:F5}");

            // Against central differences of the GROUND point, recomputed end to end.
            // This is the check that the rotation into the co-rotating frame carries
            // its time-of-flight term: an implementation that rotated the .V parts
            // outside the Dual chain passes every other test in this file and fails
            // only here.
            double worst = 0; int wi = -1, wj = -1;
            double[] hv = { 0.05, 0.05, 0.05 };
            for (int j = 0; j < 3; j++)
            {
                double[] plus = (double[])X0.Clone(); plus[3 + j] += hv[j];
                double[] minus = (double[])X0.Clone(); minus[3 + j] -= hv[j];
                Span<Dual> xs = stackalloc Dual[6];
                Load(xs, plus); ImpactPrediction pp = ImpactPredictor.Predict(Sys(true), xs, opt, scratch, default);
                Load(xs, minus); ImpactPrediction mm = ImpactPredictor.Predict(Sys(true), xs, opt, scratch, default);
                double[] fd =
                {
                    (pp.Fx.V - mm.Fx.V) / (2 * hv[j]),
                    (pp.Fy.V - mm.Fy.V) / (2 * hv[j]),
                    (pp.Fz.V - mm.Fz.V) / (2 * hv[j]),
                };
                for (int i = 0; i < 3; i++)
                {
                    double rel = Math.Abs(dG[i * 3 + j] - fd[i]) / (1 + Math.Abs(fd[i]));
                    if (rel > worst) { worst = rel; wi = i; wj = j; }
                }
            }
            Check("d(ground)/dv == central differences", worst < 1e-6,
                  $"max rel {worst:E2}" + (wi >= 0 ? $" at d(g{wi})/d(v{wj})" : ""));

            // How much of the ground answer comes from the rotation's dependence on
            // the time of flight, as opposed to rotating the inertial sensitivity?
            // Printed rather than asserted - it is a property of the trajectory - but
            // it is the number that says whether the extra term was worth carrying.
            double ang = -Omega * nom.TimeOfFlight.V;
            double ca = Math.Cos(ang), sa = Math.Sin(ang);
            double biggest = 0, naive = 0;
            for (int j = 0; j < 3; j++)
            {
                // Just rotating the inertial columns, i.e. what you get by ignoring
                // d(angle)/dv entirely.
                double gx = dI[0 * 3 + j] * ca - dI[1 * 3 + j] * sa;
                double gy = dI[0 * 3 + j] * sa + dI[1 * 3 + j] * ca;
                double dx = dG[0 * 3 + j] - gx, dy = dG[1 * 3 + j] - gy;
                double miss = Math.Sqrt(dx * dx + dy * dy);
                double mag = Math.Sqrt(dG[0 * 3 + j] * dG[0 * 3 + j] + dG[1 * 3 + j] * dG[1 * 3 + j]);
                if (miss > biggest) { biggest = miss; naive = mag; }
            }
            Console.WriteLine($"    ignoring d(tof)/dv in the frame rotation would be wrong by");
            Console.WriteLine($"    up to {biggest:F2} m per m/s, against a column of {naive:F2} "
                            + $"({100 * biggest / naive:F1}%)");

            // Cost. Seeded sweeps lose the unseeded fast path in AeroTable.Cd, so a
            // Jacobian is more than three times one prediction.
            const int Reps = 100;
            for (int i = 0; i < 10; i++)
                ImpactPredictor.VelocityJacobian(Sys(true), X0, opt, scratch, dG, dI, dT);
            var sw3 = System.Diagnostics.Stopwatch.StartNew();
            for (int i = 0; i < Reps; i++)
                ImpactPredictor.VelocityJacobian(Sys(true), X0, opt, scratch, dG, dI, dT);
            sw3.Stop();
            Console.WriteLine($"    3-column velocity Jacobian: "
                            + $"{sw3.Elapsed.TotalMilliseconds / Reps:F2} ms");
        }

        // ---- steering on it: greedy vs Gauss-Newton ---------------------
        // The question this answers: does the raw Jacobian give the direction to
        // steer? It gives a DESCENT direction (its transpose does), which is not the
        // same thing as the direction to the target.
        Console.WriteLine();
        Console.WriteLine("  steering: -J^T m (greedy) vs -J^+ m (Gauss-Newton)");
        {
            Span<double> dG = stackalloc double[9];
            Span<double> dT = stackalloc double[3];
            ImpactPrediction nom = ImpactPredictor.VelocityJacobian(
                Sys(true), X0, opt, scratch, dG, default, dT);

            double[] hit = { nom.Fx.V, nom.Fy.V, nom.Fz.V };
            double groundR = Math.Sqrt(hit[0] * hit[0] + hit[1] * hit[1] + hit[2] * hit[2]);
            double[] nrm = { hit[0] / groundR, hit[1] / groundR, hit[2] / groundR };

            // A target 20 km short along the ground track, plus 8 km across it. Both
            // in the ground frame, both reachable in principle.
            double[] along = { -nrm[1], nrm[0], 0 };            // eastward tangent
            double aLen = Math.Sqrt(along[0] * along[0] + along[1] * along[1]);
            for (int i = 0; i < 3; i++) along[i] /= aLen;
            double[] cross = { nrm[1] * along[2] - nrm[2] * along[1],
                               nrm[2] * along[0] - nrm[0] * along[2],
                               nrm[0] * along[1] - nrm[1] * along[0] };
            // Stepped along the tangent plane, then put BACK on the surface. A 21.5 km
            // tangent chord leaves a sphere of this radius by chord^2/2R = 36 m, and
            // that 36 m is radial - which is exactly the component no velocity change
            // can produce. Left in, the iteration below converges to a 36 m floor and
            // looks like a solver limit when it is really an unreachable target.
            double[] target = new double[3];
            for (int i = 0; i < 3; i++)
                target[i] = hit[i] - 20000.0 * along[i] + 8000.0 * cross[i];
            double tLen = Math.Sqrt(target[0] * target[0] + target[1] * target[1]
                                  + target[2] * target[2]);
            for (int i = 0; i < 3; i++) target[i] *= groundR / tLen;

            Span<double> miss = stackalloc double[3];
            for (int i = 0; i < 3; i++) miss[i] = hit[i] - target[i];

            Span<double> dv = stackalloc double[3];
            Span<double> greedy = stackalloc double[3];
            bool solved = ImpactSteering.Correction(dG, miss, nrm, ImpactSteering.DefaultLambda,
                                                    dv, greedy);
            Check("steering solve succeeds", solved);

            // IS THE TANGENT PROJECTION DOING ANYTHING? The impact point is driven to
            // |g| = TargetRadius by the Newton refinement, and that holds for every
            // perturbed trajectory too - so d|g|/dv is zero, which means every row of
            // J is already tangent to the surface and n is a LEFT NULL VECTOR of J.
            //
            // If that is right, the radial part of a miss is annihilated by
            // J^T (J J^T + lam^2 I)^-1 all on its own and the projection is redundant.
            // Measured rather than argued, because "obviously tangent" is exactly the
            // kind of thing that stops being true when a target radius starts varying.
            double jFro = 0, nTJ = 0;
            for (int j = 0; j < 3; j++)
            {
                double col = 0;
                for (int i = 0; i < 3; i++)
                {
                    col += nrm[i] * dG[i * 3 + j];
                    jFro += dG[i * 3 + j] * dG[i * 3 + j];
                }
                nTJ += col * col;
            }
            jFro = Math.Sqrt(jFro); nTJ = Math.Sqrt(nTJ);
            Check("J's rows are tangent to the surface (n^T J = 0)", nTJ / jFro < 1e-9,
                  $"|n^T J| / |J| = {nTJ / jFro:E2}");

            // So the projection should be a no-op. Same solve, no normal supplied.
            Span<double> dvNoProj = stackalloc double[3];
            ImpactSteering.Correction(dG, miss, default, ImpactSteering.DefaultLambda,
                                      dvNoProj, default);
            double projDiff = Math.Sqrt(
                  (dv[0] - dvNoProj[0]) * (dv[0] - dvNoProj[0])
                + (dv[1] - dvNoProj[1]) * (dv[1] - dvNoProj[1])
                + (dv[2] - dvNoProj[2]) * (dv[2] - dvNoProj[2]));
            Check("the tangent projection is redundant, not load-bearing",
                  projDiff < 1e-9,
                  $"projected vs not: {projDiff:E2} m/s apart");

            // What the damping actually does, as filter factors sigma^2/(sigma^2+lam^2)
            // on each singular direction. Near 1 = untouched, near 0 = suppressed. This
            // is the number that says lambda is set sensibly: it must leave the useful
            // directions alone and kill only the null one.
            Span<double> sv = stackalloc double[3];
            SingularValues(dG, sv);
            double lam = ImpactSteering.DefaultLambda;
            Console.WriteLine($"    singular values {sv[0]:F2} {sv[1]:F2} {sv[2]:E1}"
                            + $"   (condition {sv[0] / Math.Max(sv[2], 1e-300):E1})");
            Console.WriteLine($"    damping filter  "
                            + $"{sv[0] * sv[0] / (sv[0] * sv[0] + lam * lam):F5} "
                            + $"{sv[1] * sv[1] / (sv[1] * sv[1] + lam * lam):F5} "
                            + $"{sv[2] * sv[2] / (sv[2] * sv[2] + lam * lam):F5}"
                            + "   <- lambda only bites on the null direction");

            double dvLen = Math.Sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);
            double gLen = Math.Sqrt(greedy[0] * greedy[0] + greedy[1] * greedy[1]
                                  + greedy[2] * greedy[2]);
            double cos = 0;
            for (int i = 0; i < 3; i++) cos += (dv[i] / dvLen) * (greedy[i] / gLen);
            double angle = Math.Acos(Math.Clamp(cos, -1, 1)) * 180.0 / Math.PI;

            double missLen = Math.Sqrt(miss[0] * miss[0] + miss[1] * miss[1] + miss[2] * miss[2]);
            Console.WriteLine($"    miss {missLen / 1000:F1} km  ->  dv {dvLen:F2} m/s");
            Console.WriteLine($"    angle between greedy and Gauss-Newton: {angle:F1} deg");

            // THE test: fly the correction and see whether the impact actually lands
            // on the target. Not a claim about the maths - a measurement of the whole
            // chain, Jacobian and solve together.
            double[] corrected = (double[])X0.Clone();
            for (int i = 0; i < 3; i++) corrected[3 + i] += dv[i];
            Span<Dual> xd = stackalloc Dual[6];
            Load(xd, corrected);
            ImpactPrediction after = ImpactPredictor.Predict(Sys(true), xd, opt, scratch, default);

            double residual = double.NaN;
            if (after.Hit)
            {
                double[] h2 = { after.Fx.V, after.Fy.V, after.Fz.V };
                residual = Math.Sqrt((h2[0] - target[0]) * (h2[0] - target[0])
                                   + (h2[1] - target[1]) * (h2[1] - target[1])
                                   + (h2[2] - target[2]) * (h2[2] - target[2]));
            }
            Console.WriteLine($"    applying it leaves {residual / 1000:F2} km of the "
                            + $"{missLen / 1000:F1} km miss");
            Check("one Gauss-Newton step removes most of the miss",
                  after.Hit && residual < 0.25 * missLen,
                  $"{100 * (1 - residual / missLen):F1}% removed in one step");

            // HOW LOCAL IS "LOCALLY OPTIMAL"? One step is a linearisation, so it
            // leaves the second-order residual behind. Iterating - re-linearising at
            // the corrected state each time - is what turns it into an answer.
            //
            // The interesting number is not that it converges (Gauss-Newton on a
            // zero-residual problem does) but what the FIRST step cost against what
            // convergence cost. If they are close, the greedy step is near enough for
            // a guidance cycle to fly and correct next cycle. If they diverge, the
            // linearisation is being pushed too far per step.
            Console.WriteLine();
            Console.WriteLine("    iterating the correction (re-linearising each time)");
            {
                double[] state = (double[])X0.Clone();
                double cumulative = 0.0;
                double firstStep = 0.0;
                Span<double> jg = stackalloc double[9];
                Span<double> jt = stackalloc double[3];
                Span<double> step = stackalloc double[3];

                for (int it = 0; it < 5; it++)
                {
                    ImpactPrediction cur = ImpactPredictor.VelocityJacobian(
                        Sys(true), state, opt, scratch, jg, default, jt);
                    if (!cur.Hit) break;

                    double[] hc = { cur.Fx.V, cur.Fy.V, cur.Fz.V };
                    Span<double> mc = stackalloc double[3];
                    for (int i = 0; i < 3; i++) mc[i] = hc[i] - target[i];
                    double mlen = Math.Sqrt(mc[0] * mc[0] + mc[1] * mc[1] + mc[2] * mc[2]);

                    Console.WriteLine($"      iter {it}: miss {mlen / 1000,8:F3} km   "
                                    + $"cumulative dv {cumulative,7:F2} m/s");
                    if (mlen < 1.0) break;

                    double hcLen = Math.Sqrt(hc[0] * hc[0] + hc[1] * hc[1] + hc[2] * hc[2]);
                    Span<double> nc = stackalloc double[3]
                        { hc[0] / hcLen, hc[1] / hcLen, hc[2] / hcLen };
                    if (!ImpactSteering.Correction(jg, mc, nc, ImpactSteering.DefaultLambda,
                                                   step, default))
                        break;

                    double slen = Math.Sqrt(step[0] * step[0] + step[1] * step[1]
                                          + step[2] * step[2]);
                    if (it == 0) firstStep = slen;
                    cumulative += slen;
                    for (int i = 0; i < 3; i++) state[3 + i] += step[i];
                }

                Console.WriteLine($"      first step {firstStep:F2} m/s, converged total "
                                + $"{cumulative:F2} m/s "
                                + $"({100 * (cumulative / firstStep - 1):+0.0;-0.0}%)");

                // The number that says whether one greedy step is good enough to fly:
                // if convergence costs barely more than the first step, a guidance
                // cycle can take the step, fly it, and re-solve next cycle.
                Check("iterating costs little more than the first step",
                      cumulative < 1.25 * firstStep,
                      $"{100 * (cumulative / firstStep - 1):F1}% more than the greedy step");
            }

            // WHY IS THE ANSWER THE MINIMUM dv? Nothing in the code runs an
            // optimiser, so the minimisation has to be coming from somewhere.
            //
            // It comes from the SHAPE of the formula. Velocity space splits into the
            // row space of J and its null space, which are orthogonal. Any dv splits
            // the same way, and by Pythagoras
            //
            //     |dv|^2 = |dv_row|^2 + |dv_null|^2
            //
            // The row part is forced - it is what produces the impact movement. The
            // null part is free: adding any amount of it changes nothing downrange.
            // So the shortest dv is the one with NO null component.
            //
            // And dv = -J^T y is in the row space of J for ANY y, because the row
            // space of J is the column space of J^T. Writing the answer in that form
            // is what makes it minimum-norm - there is no separate minimisation step,
            // the J^T on the outside IS the minimisation.
            //
            // Both halves of that are checked below.
            {
                Span<double> jg = stackalloc double[9];
                ImpactPredictor.VelocityJacobian(Sys(true), X0, opt, scratch, jg, default, default);

                // The null direction: cross product of the two most independent rows.
                double[] nul = new double[3];
                double best = -1;
                for (int a = 0; a < 3; a++)
                    for (int b = a + 1; b < 3; b++)
                    {
                        double[] u = { jg[a * 3 + 0], jg[a * 3 + 1], jg[a * 3 + 2] };
                        double[] w = { jg[b * 3 + 0], jg[b * 3 + 1], jg[b * 3 + 2] };
                        double[] c = { u[1] * w[2] - u[2] * w[1],
                                       u[2] * w[0] - u[0] * w[2],
                                       u[0] * w[1] - u[1] * w[0] };
                        double cl = Math.Sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
                        if (cl > best) { best = cl; for (int i = 0; i < 3; i++) nul[i] = c[i] / cl; }
                    }

                // Confirm it really is the null direction: J n = 0.
                double[] Jn = new double[3];
                for (int i = 0; i < 3; i++)
                    Jn[i] = jg[i * 3 + 0] * nul[0] + jg[i * 3 + 1] * nul[1] + jg[i * 3 + 2] * nul[2];
                double jnLen = Math.Sqrt(Jn[0] * Jn[0] + Jn[1] * Jn[1] + Jn[2] * Jn[2]);
                Console.WriteLine();
                Console.WriteLine("    minimum-norm, demonstrated");
                Console.WriteLine($"      null direction n = ({nul[0]:F4}, {nul[1]:F4}, {nul[2]:F4})");
                Check("J n = 0 (n really is the useless direction)", jnLen / sv[0] < 1e-6,
                      $"|J n| = {jnLen:E2} m per m/s");

                // (a) the computed dv has NO component along n
                double nullPart = dv[0] * nul[0] + dv[1] * nul[1] + dv[2] * nul[2];
                double dvLen2 = Math.Sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);
                Check("dv has zero component along the null direction",
                      Math.Abs(nullPart) / dvLen2 < 1e-9,
                      $"n.dv / |dv| = {Math.Abs(nullPart) / dvLen2:E2}  -> it lies in the row space");

                // (b) adding any of n leaves the impact identical but the dv longer
                Console.WriteLine($"      {"added n",10} {"|dv|",10} {"impact shift",14}");
                foreach (double t in new[] { 0.0, 25.0, 50.0, 100.0 })
                {
                    double[] alt = { dv[0] + t * nul[0], dv[1] + t * nul[1], dv[2] + t * nul[2] };
                    double[] moved = new double[3];
                    for (int i = 0; i < 3; i++)
                        moved[i] = jg[i * 3 + 0] * (alt[0] - dv[0])
                                 + jg[i * 3 + 1] * (alt[1] - dv[1])
                                 + jg[i * 3 + 2] * (alt[2] - dv[2]);
                    double shift = Math.Sqrt(moved[0] * moved[0] + moved[1] * moved[1]
                                           + moved[2] * moved[2]);
                    double alen = Math.Sqrt(alt[0] * alt[0] + alt[1] * alt[1] + alt[2] * alt[2]);
                    Console.WriteLine($"      {t,8:F0} m/s {alen,9:F2} m/s {shift,12:F6} m");
                }
                Console.WriteLine("      -> every one of these hits the SAME point; ours is the shortest.");
            }

            // FLIGHT-PATH-ANGLE SHAPING: is the free direction any use?
            //
            // The targeting correction minimises dv and nothing else, and the cheapest
            // way to drag an impact point backwards is often to pitch DOWN. The free
            // direction is the one place to fix that without disturbing the target, so
            // the question is whether it has any VERTICAL authority - a free direction
            // lying in the local horizontal would be useless for this however much dv
            // went into it.
            {
                Span<double> jg = stackalloc double[9];
                ImpactPredictor.VelocityJacobian(Sys(true), X0, opt, scratch, jg, default, default);

                Span<double> nf = stackalloc double[3];
                bool got = ImpactSteering.FreeDirection(jg, nf);
                Check("FreeDirection finds one", got);

                // It must actually be free: J n = 0.
                double jn = 0;
                for (int i = 0; i < 3; i++)
                {
                    double row = 0;
                    for (int j = 0; j < 3; j++) row += jg[i * 3 + j] * nf[j];
                    jn += row * row;
                }
                Check("the free direction moves the impact nowhere",
                      Math.Sqrt(jn) / sv[0] < 1e-9, $"|J n| = {Math.Sqrt(jn):E2} m per m/s");

                // Vertical authority: how much of it is along the local vertical.
                double rl = Math.Sqrt(X0[0] * X0[0] + X0[1] * X0[1] + X0[2] * X0[2]);
                double[] up = { X0[0] / rl, X0[1] / rl, X0[2] / rl };
                double vert = nf[0] * up[0] + nf[1] * up[1] + nf[2] * up[2];
                Console.WriteLine();
                Console.WriteLine("    flight-path-angle shaping");
                Console.WriteLine($"      free direction   ({nf[0]:F4}, {nf[1]:F4}, {nf[2]:F4})");
                Console.WriteLine($"      vertical component {vert:F4}   "
                                + "<- how much authority it has to re-aim the burn");
                Check("the free direction has usable vertical authority",
                      Math.Abs(vert) > 0.05, $"|n . up| = {Math.Abs(vert):F3}");

                // THE PROBLEM, quantified: how much of the correction points down.
                Span<double> dG2 = stackalloc double[9];
                Span<double> mm2 = stackalloc double[3];
                ImpactPrediction nom2 = ImpactPredictor.VelocityJacobian(
                    Sys(true), X0, opt, scratch, dG2, default, default);
                double[] hit2 = { nom2.Fx.V, nom2.Fy.V, nom2.Fz.V };
                for (int i = 0; i < 3; i++) mm2[i] = hit2[i] - target[i];
                Span<double> dvS = stackalloc double[3];
                double hl2 = Math.Sqrt(hit2[0] * hit2[0] + hit2[1] * hit2[1] + hit2[2] * hit2[2]);
                Span<double> nr2 = stackalloc double[3] { hit2[0] / hl2, hit2[1] / hl2, hit2[2] / hl2 };
                ImpactSteering.Correction(dG2, mm2, nr2, ImpactSteering.DefaultLambda, dvS, default);

                double cmdLen = Math.Sqrt(dvS[0] * dvS[0] + dvS[1] * dvS[1] + dvS[2] * dvS[2]);
                double down = dvS[0] * up[0] + dvS[1] * up[1] + dvS[2] * up[2];
                double belowDeg = Math.Asin(Math.Clamp(down / cmdLen, -1, 1)) * 180.0 / Math.PI;
                Console.WriteLine($"      correction {cmdLen:F1} m/s, of which {down:F1} m/s is "
                                + $"DOWNWARD ({belowDeg:F1} deg below horizon)");
                Check("the correction really does point into the ground", down < 0.0,
                      "which is the dive the shaping exists to fix");

                // Levelling the COMMAND, and what it would cost. Compare with levelling
                // the VELOCITY, which is the tempting-but-wrong target.
                double tLevelCmd = -down / vert;
                double radial = X0[3] * up[0] + X0[4] * up[1] + X0[5] * up[2];
                double tLevelVel = -radial / vert;
                Console.WriteLine($"      levelling the COMMAND costs {Math.Abs(tLevelCmd):F1} m/s; "
                                + $"levelling the VELOCITY would cost {Math.Abs(tLevelVel):F1} m/s");

                // THE KNOB: sweep the minimum-pitch target the way the mod exposes it,
                // using the same bracket-and-bisect solve. Three properties matter -
                // that any target below the geometric ceiling is REACHED (there is no
                // dv cap), that it is a FLOOR, and that the ceiling really is where the
                // free direction points.
                double PitchDeg(double[] vec)
                {
                    double l = Math.Sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
                    double d = vec[0] * up[0] + vec[1] * up[1] + vec[2] * up[2];
                    return l > 1e-9 ? Math.Asin(Math.Clamp(d / l, -1, 1)) * 180.0 / Math.PI : 0.0;
                }
                // Point the free direction up, as the mod does, then copy both out of
                // their spans - a local function cannot close over a Span.
                if (vert < 0) { for (int i = 0; i < 3; i++) nf[i] = -nf[i]; vert = -vert; }
                double ceiling = Math.Asin(Math.Clamp(vert, -1, 1)) * 180.0 / Math.PI;
                double[] dvA = dvS.ToArray();
                double[] nA = nf.ToArray();

                double PitchAt(double tt)
                    => PitchDeg(new[] { dvA[0] + tt * nA[0], dvA[1] + tt * nA[1], dvA[2] + tt * nA[2] });

                double[] baseCmd = { dvA[0], dvA[1], dvA[2] };
                double basePitch = PitchDeg(baseCmd);
                double cmdLen0 = Math.Sqrt(dvA[0] * dvA[0] + dvA[1] * dvA[1] + dvA[2] * dvA[2]);

                Console.WriteLine();
                Console.WriteLine($"    minimum-pitch knob (burn starts at {basePitch:F1} deg, "
                                + $"correction {cmdLen0:F1} m/s, ceiling {ceiling:F1} deg)");
                Console.WriteLine($"      {"target",7} {"shaping",9} {"achieved",9} "
                                + $"{"total dv",9} {"ratio",7} {"leak",8}");

                bool reaches = true, isFloor = true, monotone = true;
                double prevPitch = PitchAt(0.0);
                for (double tt = 10; tt <= 2000; tt += 10)
                {
                    double pp = PitchAt(tt);
                    if (pp < prevPitch - 1e-9) monotone = false;
                    prevPitch = pp;
                }

                foreach (double targetDeg in new[] { -20.0, -10.0, 0.0, 10.0, 20.0, 40.0 })
                {
                    if (basePitch >= targetDeg)
                    {
                        Console.WriteLine($"      {targetDeg,6:F0}d {"-",9} {basePitch,8:F1}d "
                                        + $"{cmdLen0,8:F1} {"-",7} {"-",8}   (already above)");
                        continue;
                    }
                    // Same bracket-and-bisect the mod runs.
                    double lo2 = 0.0, hi2 = cmdLen0;
                    for (int i = 0; i < 40 && PitchAt(hi2) < targetDeg; i++) hi2 *= 2.0;
                    for (int i = 0; i < 60; i++)
                    {
                        double mid = 0.5 * (lo2 + hi2);
                        if (PitchAt(mid) < targetDeg) lo2 = mid; else hi2 = mid;
                    }
                    double gotDeg = PitchAt(hi2);
                    double[] cmd = { dvA[0] + hi2 * nA[0], dvA[1] + hi2 * nA[1], dvA[2] + hi2 * nA[2] };
                    double tot = Math.Sqrt(cmd[0] * cmd[0] + cmd[1] * cmd[1] + cmd[2] * cmd[2]);

                    double[] st2 = (double[])X0.Clone();
                    for (int i = 0; i < 3; i++) st2[3 + i] += hi2 * nA[i];
                    Span<Dual> xs3 = stackalloc Dual[6];
                    Load(xs3, X0);
                    ImpactPrediction b0 = ImpactPredictor.Predict(Sys(true), xs3, opt, scratch, default);
                    Load(xs3, st2);
                    ImpactPrediction b1 = ImpactPredictor.Predict(Sys(true), xs3, opt, scratch, default);
                    double leak = Math.Sqrt(
                          (b1.Fx.V - b0.Fx.V) * (b1.Fx.V - b0.Fx.V)
                        + (b1.Fy.V - b0.Fy.V) * (b1.Fy.V - b0.Fy.V)
                        + (b1.Fz.V - b0.Fz.V) * (b1.Fz.V - b0.Fz.V));

                    Console.WriteLine($"      {targetDeg,6:F0}d {hi2,8:F1} {gotDeg,8:F1}d "
                                    + $"{tot,8:F1} {hi2 / cmdLen0,6:F2}x {leak / 1000,6:F2} km");

                    if (Math.Abs(gotDeg - targetDeg) > 0.5) reaches = false;
                    if (gotDeg < basePitch - 0.5) isFloor = false;
                }

                Check("pitch is monotonic along the free direction", monotone);
                Check("every target below the ceiling is REACHED", reaches,
                      "no dv cap - the geometry is the only limit");
                Check("the knob is a floor - never lowers the burn", isFloor);
                Console.WriteLine("      (leak grows with the square of the nudge; the closed");
                Console.WriteLine("       loop absorbs it as targeting error the next cycle)");
            }

            // WHERE DOES THE RANK DEFICIENCY COME FROM?
            //
            // Not from a constraint - nothing here constrains anything. It comes from
            // what we chose to REPORT. The predictor's output is the state at the
            // moment the trajectory crosses radius R, and a crossing point of a sphere
            // is on that sphere, so |p| = R identically for every initial state. A
            // function whose values all lie on a sphere has a derivative that lands in
            // that sphere's tangent plane - so n^T J = 0 falls out of the definition,
            // and three inputs map onto two output dimensions.
            //
            // An earlier version of docs/impact-steering.md said the surface AND the
            // trajectory being planar were two independent reasons that "stacked up".
            // That cannot be right: two independent deficiencies would leave rank 1 and
            // TWO zero singular values, and there is only one. The in-plane 2x2 block
            // being rank 1 is the same fact restricted to the plane, not a second one.
            //
            // The test: take an INCLINED, non-planar arc. If planarity were a separate
            // cause, removing it should recover a rank the planar case did not have.
            {
                double[] tilted =
                {
                    R + 90000.0, 0.0, 12000.0,
                    -350.0, 1400.0, 520.0,          // out-of-plane position and velocity
                };
                Span<double> jt2 = stackalloc double[9];
                ImpactPrediction ti = ImpactPredictor.VelocityJacobian(
                    Sys(true), tilted, opt, scratch, jt2, default, default);
                Span<double> svT = stackalloc double[3];
                SingularValues(jt2, svT);
                Console.WriteLine();
                Console.WriteLine("    rank, on a NON-planar arc (tests whether planarity is a cause)");
                Console.WriteLine($"      singular values {svT[0]:F2} {svT[1]:F2} {svT[2]:E2}");
                Check("a non-planar arc is still rank 2, not 3",
                      ti.Hit && svT[1] > 1.0 && svT[2] / svT[0] < 1e-9,
                      "planarity fixes WHERE the null direction points, not that there is one");
            }

            // WHY DOES GREEDY NEED TO ITERATE AT ALL?
            //
            // The obvious answer - "because J is only a linearisation" - is only half
            // of it, and the smaller half. Freeze J and m and work the PURELY LINEAR
            // problem below: no re-integration, no nonlinearity anywhere, just
            // minimise |m + J dv| over dv. Gauss-Newton solves that exactly in one
            // step, by construction. Steepest descent does NOT, and the reason is
            // pure linear algebra:
            //
            //   -J^T m is perpendicular to the level set of |m + J dv|, and the level
            //   sets are ellipses. Perpendicular-to-an-ellipse points at its centre
            //   only when the ellipse is a CIRCLE, i.e. when J^T J is a multiple of I.
            //   Otherwise the gradient overshoots along one axis and undershoots along
            //   another, and fixing that mismatch is what the iterations are for.
            //
            // So greedy iterates for two independent reasons - conditioning and
            // nonlinearity - and this block shows the first one on its own.
            Console.WriteLine();
            Console.WriteLine("    the LINEAR problem alone (J and m frozen - no nonlinearity)");
            {
                Span<double> jgS = stackalloc double[9];
                ImpactPrediction nomL = ImpactPredictor.VelocityJacobian(
                    Sys(true), X0, opt, scratch, jgS, default, default);
                // Copied out of the span: a local function cannot close over one.
                double[] jg = jgS.ToArray();
                double[] hl = { nomL.Fx.V, nomL.Fy.V, nomL.Fz.V };
                double[] m0 = new double[3];
                for (int i = 0; i < 3; i++) m0[i] = hl[i] - target[i];

                double Residual(double[] dvv)
                {
                    double r2 = 0;
                    for (int i = 0; i < 3; i++)
                    {
                        double e = m0[i];
                        for (int j = 0; j < 3; j++) e += jg[i * 3 + j] * dvv[j];
                        r2 += e * e;
                    }
                    return Math.Sqrt(r2);
                }

                // Gauss-Newton: one shot.
                Span<double> gnv = stackalloc double[3];
                ImpactSteering.Correction(jg, m0, default, ImpactSteering.DefaultLambda,
                                          gnv, default);
                double[] gnArr = { gnv[0], gnv[1], gnv[2] };
                Console.WriteLine($"      Gauss-Newton, ONE step: residual "
                                + $"{Residual(gnArr):F6} m   (was {Residual(new double[3]) / 1000:F3} km)");

                // Steepest descent with exact line search, on the same frozen problem.
                double[] sd = new double[3];
                for (int it = 1; it <= 6; it++)
                {
                    double[] e = new double[3];
                    for (int i = 0; i < 3; i++)
                    {
                        e[i] = m0[i];
                        for (int j = 0; j < 3; j++) e[i] += jg[i * 3 + j] * sd[j];
                    }
                    double[] g = new double[3];
                    for (int j = 0; j < 3; j++)
                        g[j] = jg[0 * 3 + j] * e[0] + jg[1 * 3 + j] * e[1] + jg[2 * 3 + j] * e[2];
                    double[] Jg = new double[3];
                    for (int i = 0; i < 3; i++)
                        Jg[i] = jg[i * 3 + 0] * g[0] + jg[i * 3 + 1] * g[1] + jg[i * 3 + 2] * g[2];
                    double gg = g[0] * g[0] + g[1] * g[1] + g[2] * g[2];
                    double jj = Jg[0] * Jg[0] + Jg[1] * Jg[1] + Jg[2] * Jg[2];
                    if (jj <= 0) break;
                    for (int i = 0; i < 3; i++) sd[i] -= (gg / jj) * g[i];
                    Console.WriteLine($"      steepest descent, step {it}: residual "
                                    + $"{Residual(sd),10:F3} m");
                }
                // The ~36 m both settle on is not a failure to converge. m is the
                // CHORD between two points on the sphere, and a chord is not tangent:
                // its radial part is chord^2/2R = 21541^2/(2*6371000) = 36.4 m, which
                // no tangent displacement can produce. Both laws reach that floor -
                // Gauss-Newton in one step, steepest descent in five. In the NONLINEAR
                // iteration above it does not appear, because as the miss shrinks the
                // chord shortens and its radial part falls off as the square.
                double floorM = 21541.0 * 21541.0 / (2.0 * R);
                Console.WriteLine($"      (the ~{floorM:F0} m floor is the chord's radial part, "
                                + "chord^2/2R - unreachable by construction)");
                Console.WriteLine("      -> greedy iterates even with ZERO nonlinearity;");
                Console.WriteLine("         the gradient is perpendicular to an ellipse, not aimed at its centre.");
            }

            // A FAIR comparison against pure-Jacobian greedy guidance. Steepest
            // descent with an ARBITRARY step length is a straw man; give it its own
            // exact line search instead:
            //
            //   dv = -alpha J^T m,   alpha = |J^T m|^2 / |J J^T m|^2
            //
            // which is the alpha minimising |m + J dv|^2 along that direction. Then
            // iterate both laws and compare what each costs to reach the same miss.
            Console.WriteLine();
            Console.WriteLine("    steepest descent with exact line search, iterated");
            {
                double[] state = (double[])X0.Clone();
                double cumulative = 0.0;
                int iters = 0;
                Span<double> jg = stackalloc double[9];
                Span<double> jt = stackalloc double[3];

                for (int it = 0; it < 40; it++)
                {
                    ImpactPrediction cur = ImpactPredictor.VelocityJacobian(
                        Sys(true), state, opt, scratch, jg, default, jt);
                    if (!cur.Hit) break;

                    double[] hc = { cur.Fx.V, cur.Fy.V, cur.Fz.V };
                    double[] mc = new double[3];
                    for (int i = 0; i < 3; i++) mc[i] = hc[i] - target[i];
                    double mlen = Math.Sqrt(mc[0] * mc[0] + mc[1] * mc[1] + mc[2] * mc[2]);
                    if (it < 4 || mlen < 10.0)
                        Console.WriteLine($"      iter {it}: miss {mlen / 1000,8:F3} km   "
                                        + $"cumulative dv {cumulative,7:F2} m/s");
                    if (mlen < 10.0) { iters = it; break; }

                    // g = J^T m, then Jg, then the exact step length.
                    double[] g = new double[3];
                    for (int j = 0; j < 3; j++)
                        g[j] = jg[0 * 3 + j] * mc[0] + jg[1 * 3 + j] * mc[1] + jg[2 * 3 + j] * mc[2];
                    double[] Jg = new double[3];
                    for (int i = 0; i < 3; i++)
                        Jg[i] = jg[i * 3 + 0] * g[0] + jg[i * 3 + 1] * g[1] + jg[i * 3 + 2] * g[2];
                    double gg = g[0] * g[0] + g[1] * g[1] + g[2] * g[2];
                    double jgjg = Jg[0] * Jg[0] + Jg[1] * Jg[1] + Jg[2] * Jg[2];
                    if (jgjg <= 0) break;
                    double alpha = gg / jgjg;

                    double slen = alpha * Math.Sqrt(gg);
                    cumulative += slen;
                    for (int i = 0; i < 3; i++) state[3 + i] -= alpha * g[i];
                    iters = it + 1;
                }
                Console.WriteLine($"      converged in {iters} iterations, total "
                                + $"{cumulative:F2} m/s");
            }

            // And the greedy direction, at the same dv magnitude, does not.
            double[] greedyStep = (double[])X0.Clone();
            for (int i = 0; i < 3; i++) greedyStep[3 + i] += greedy[i] / gLen * dvLen;
            Load(xd, greedyStep);
            ImpactPrediction ag = ImpactPredictor.Predict(Sys(true), xd, opt, scratch, default);
            if (ag.Hit)
            {
                double[] h3 = { ag.Fx.V, ag.Fy.V, ag.Fz.V };
                double rg = Math.Sqrt((h3[0] - target[0]) * (h3[0] - target[0])
                                    + (h3[1] - target[1]) * (h3[1] - target[1])
                                    + (h3[2] - target[2]) * (h3[2] - target[2]));
                Console.WriteLine($"    the same dv along the GREEDY direction leaves "
                                + $"{rg / 1000:F2} km");
            }
        }

        // ---- an orbit does not hit the ground ---------------------------
        {
            double v = Math.Sqrt(Mu / (R + 400000.0));
            double[] orbit = { R + 400000.0, 0, 0, 0, v, 0 };
            Span<Dual> xo = stackalloc Dual[6];
            Load(xo, orbit);
            var o = ImpactOptions.Default(R);
            o.MaxTime = 600.0; o.PathStride = 0;
            ImpactPrediction q = ImpactPredictor.Predict(Sys(true), xo, o, scratch, default);
            Check("a circular orbit reports no impact",
                  q.Status == ImpactStatus.NoImpactWithinHorizon,
                  $"{q.Status}, min alt {q.MinAltitude / 1000:F1} km");
        }

        // ---- starting underground is refused, not integrated ------------
        {
            double[] under = { R - 100.0, 0, 0, 0, 0, 0 };
            Span<Dual> xu = stackalloc Dual[6];
            Load(xu, under);
            ImpactPrediction q = ImpactPredictor.Predict(Sys(true), xu, opt, scratch, default);
            Check("a state below the surface is rejected",
                  q.Status == ImpactStatus.StartedBelowSurface, q.Status.ToString());
        }

        // ---- cost -------------------------------------------------------
        Console.WriteLine();
        {
            const int Reps = 200;
            Load(x, X0);
            for (int i = 0; i < 20; i++)
                ImpactPredictor.Predict(Sys(true), x, opt, scratch, default);

            var sw = System.Diagnostics.Stopwatch.StartNew();
            for (int i = 0; i < Reps; i++)
            {
                Load(x, X0);
                ImpactPredictor.Predict(Sys(true), x, opt, scratch, default);
            }
            sw.Stop();
            double ms = sw.Elapsed.TotalMilliseconds / Reps;
            Console.WriteLine($"  one prediction  : {ms,7:F2} ms  ({p.Steps} steps)");
            Console.WriteLine($"  a 6-column Jacobian: {ms * 6,7:F2} ms");
            Console.WriteLine();
            Console.WriteLine("  Too slow for every frame, which is why the overlay throttles and");
            Console.WriteLine("  caches. Well inside a background solve's budget.");
        }

        Console.WriteLine();
        Console.WriteLine(fails == 0 ? "IMPACT: all checks passed" : $"IMPACT: {fails} check(s) FAILED");
        return fails == 0 ? 0 : 1;
    }

    /// <summary>Singular values of a row-major 3x3, descending: sqrt of the
    /// eigenvalues of J^T J, by the closed form for a symmetric 3x3.</summary>
    private static void SingularValues(ReadOnlySpan<double> j, Span<double> sv)
    {
        Span<double> a = stackalloc double[9];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
            {
                double s = 0;
                for (int k = 0; k < 3; k++) s += j[k * 3 + r] * j[k * 3 + c];
                a[r * 3 + c] = s;
            }

        double p1 = a[1] * a[1] + a[2] * a[2] + a[5] * a[5];
        double q = (a[0] + a[4] + a[8]) / 3.0;
        double p2 = (a[0] - q) * (a[0] - q) + (a[4] - q) * (a[4] - q)
                  + (a[8] - q) * (a[8] - q) + 2 * p1;
        double pp = Math.Sqrt(p2 / 6.0);
        double e1, e2, e3;
        if (pp <= 0)
        {
            e1 = e2 = e3 = q;
        }
        else
        {
            Span<double> b = stackalloc double[9];
            for (int i = 0; i < 9; i++)
                b[i] = (a[i] - (i % 4 == 0 ? q : 0.0)) / pp;
            double detB = b[0] * (b[4] * b[8] - b[5] * b[7])
                        - b[1] * (b[3] * b[8] - b[5] * b[6])
                        + b[2] * (b[3] * b[7] - b[4] * b[6]);
            double phi = Math.Acos(Math.Clamp(detB / 2.0, -1.0, 1.0)) / 3.0;
            e1 = q + 2 * pp * Math.Cos(phi);
            e3 = q + 2 * pp * Math.Cos(phi + 2.0 * Math.PI / 3.0);
            e2 = 3 * q - e1 - e3;
        }
        sv[0] = Math.Sqrt(Math.Max(e1, 0));
        sv[1] = Math.Sqrt(Math.Max(e2, 0));
        sv[2] = Math.Sqrt(Math.Max(e3, 0));
    }

    private static double Speed(in ImpactPrediction p)
        => Math.Sqrt(p.Vx.V * p.Vx.V + p.Vy.V * p.Vy.V + p.Vz.V * p.Vz.V);

    private static double Dist(in ImpactPrediction a, in ImpactPrediction b)
        => Math.Sqrt((a.Rx.V - b.Rx.V) * (a.Rx.V - b.Rx.V)
                   + (a.Ry.V - b.Ry.V) * (a.Ry.V - b.Ry.V)
                   + (a.Rz.V - b.Rz.V) * (a.Rz.V - b.Rz.V));

    /// <summary>Great-circle-ish downrange from the start direction to the impact.</summary>
    private static double Downrange(in ImpactPrediction p, double[] x0)
    {
        double n0 = Math.Sqrt(x0[0] * x0[0] + x0[1] * x0[1] + x0[2] * x0[2]);
        double n1 = Math.Sqrt(p.Rx.V * p.Rx.V + p.Ry.V * p.Ry.V + p.Rz.V * p.Rz.V);
        double dot = (x0[0] * p.Rx.V + x0[1] * p.Ry.V + x0[2] * p.Rz.V) / (n0 * n1);
        return R * Math.Acos(Math.Clamp(dot, -1.0, 1.0));
    }

    private static double Energy(ReadOnlySpan<Dual> x)
    {
        double r = Math.Sqrt(x[0].V * x[0].V + x[1].V * x[1].V + x[2].V * x[2].V);
        double v2 = x[3].V * x[3].V + x[4].V * x[4].V + x[5].V * x[5].V;
        return 0.5 * v2 - Mu / r;
    }
}

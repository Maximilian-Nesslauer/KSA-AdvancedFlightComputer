using System.Diagnostics;
using Navbox.Flight;
using Navbox.Numerics;

/// <summary>
/// Does the tabulated aero surrogate actually compose with our forward-mode AD?
///
/// The question is not whether the spline interpolates - that is checked here too,
/// but it is the easy half. The question is whether Cd(M, alpha) can be written
/// inline inside a Dual-valued function the way every other term in the dynamics
/// is, and come out with the right slope. If it can, aero is expressible in the
/// existing framework and needs no special-casing in the Jacobian sweep; if it
/// cannot, every aero term would have to be differentiated by hand.
///
/// The decisive test is the last one: a full drag-acceleration function written in
/// Duals - dynamic pressure, Mach, angle of attack, table lookup, all composed -
/// swept for its Jacobian exactly as Dynamics6Dof.Jacobian sweeps the dynamics,
/// and diffed against central differences of the same function.
/// </summary>
internal static class AeroCheck
{
    internal static int Run()
    {
        int fails = 0;
        void Check(string name, bool ok, string detail = "")
        {
            Console.WriteLine($"  {(ok ? "PASS" : "FAIL")}  {name,-46} {detail}");
            if (!ok) fails++;
        }

        Console.WriteLine("AERO SURROGATE: tabulated Cd(Mach, alpha) under forward-mode AD");
        Console.WriteLine();

        var tab = new AeroTable();
        const double Deg = Math.PI / 180.0;

        // ---- the table itself ------------------------------------------
        Console.WriteLine("  table shape (Cd, alpha in degrees, RETROGRADE-FIRST:");
        Console.WriteLine("  a=0 is engine into the wind, a=180 is nose-first)");
        Console.WriteLine($"    {"Mach",6} {"a=0",8} {"a=30",8} {"a=90",8} {"a=150",8} {"a=180",8}");
        foreach (double m in new[] { 0.3, 0.8, 0.95, 1.05, 1.25, 1.6, 2.0, 3.0, 5.0 })
        {
            Console.WriteLine($"    {m,6:F2} {tab.Cd(m, 0),8:F4} {tab.Cd(m, 30 * Deg),8:F4} "
                            + $"{tab.Cd(m, 90 * Deg),8:F4} {tab.Cd(m, 150 * Deg),8:F4} "
                            + $"{tab.Cd(m, 180 * Deg),8:F4}");
        }
        Console.WriteLine();

        // Physical sanity: the drag rise must peak transonically, and Cd must rise
        // from the tail-first attitude toward broadside. A fit that rings breaks the
        // second.
        bool risePeaks = tab.Cd(1.2, 0) > tab.Cd(0.5, 0) && tab.Cd(1.2, 0) > tab.Cd(4.0, 0);
        Check("transonic drag rise peaks near M=1.2", risePeaks,
              $"Cd(0.5)={tab.Cd(0.5, 0):F3} Cd(1.2)={tab.Cd(1.2, 0):F3} Cd(4.0)={tab.Cd(4.0, 0):F3}");

        // Alpha is retrograde-first and spans the full 0 - 180, so Cd is NOT
        // monotone: it climbs from the tail-first value to a maximum near broadside
        // and falls away to the nose-first value. The rise is the half a boostback
        // actually flies, so that is the half checked.
        //
        // The peak is NOT at exactly 90. The base term blends tail-first drag into
        // nose-first drag as w = (1 + cos alpha)/2, so it is still falling at 90 while
        // the sin^2 cross-flow term has already flattened - which puts the maximum at
        // 80 - 83 degrees depending on Mach. That is a property of the generator, not
        // a fitting artefact, so the monotone window stops short of it rather than
        // the check tolerating a reversal.
        const double RiseTo = 75.0;
        bool monotoneAlpha = true;
        double worstDip = 0;
        for (double m = 0.05; m <= 5.0; m += 0.05)
        {
            double prev = tab.Cd(m, 0);
            for (double a = 0.5; a <= RiseTo; a += 0.5)
            {
                double c = tab.Cd(m, a * Deg);
                if (c < prev) { monotoneAlpha = false; worstDip = Math.Max(worstDip, prev - c); }
                prev = c;
            }
        }
        Check($"Cd increases from tail-first to alpha {RiseTo:F0}", monotoneAlpha,
              monotoneAlpha ? "" : $"worst reversal {worstDip:E2}");

        // And the peak really is near broadside rather than somewhere the fit put it.
        bool peakPlaced = true;
        double worstPeak = 0;
        for (double m = 0.1; m <= 5.0; m += 0.1)
        {
            double bestA = 0, bestC = -1;
            for (double a = 0; a <= 180.0; a += 0.25)
            {
                double c = tab.Cd(m, a * Deg);
                if (c > bestC) { bestC = c; bestA = a; }
            }
            if (bestA < 70.0 || bestA > 95.0) { peakPlaced = false; worstPeak = bestA; }
        }
        Check("Cd peaks near broadside (70 - 95 deg)", peakPlaced,
              peakPlaced ? "" : $"peak at {worstPeak:F1} deg");

        // The ordering that makes the convention legible: a blunt base into the wind
        // costs more than a nose does, and broadside costs most of all. If this ever
        // inverts, the alpha axis has been flipped somewhere.
        bool ordering = true;
        for (double m = 0.1; m <= 5.0; m += 0.1)
            ordering &= tab.Cd(m, 90 * Deg) > tab.Cd(m, 0)
                     && tab.Cd(m, 0) > tab.Cd(m, 180 * Deg);
        Check("Cd(broadside) > Cd(tail-first) > Cd(nose-first)", ordering,
              $"at M=1: {tab.Cd(1, 90 * Deg):F3} / {tab.Cd(1, 0):F3} / {tab.Cd(1, 180 * Deg):F3}");

        // Fidelity BETWEEN the breakpoints, which is the only part the fit is
        // responsible for. Note that a fit SHOULD exceed the largest sampled value
        // where the true curve peaks between samples - it does, near M = 1.25.
        // Overshoot alone is not evidence of ringing; disagreement with the
        // underlying curve is.
        {
            double worstAbs = 0, worstRel = 0, atM = 0, atA = 0;
            for (int i = 0; i < OffNodeReference.GetLength(0); i++)
            {
                double m = OffNodeReference[i, 0];
                double a = OffNodeReference[i, 1] * Deg;
                double want = OffNodeReference[i, 2];
                double rel = Math.Abs(tab.Cd(m, a) - want) / want;
                if (rel > worstRel)
                {
                    worstRel = rel; worstAbs = Math.Abs(tab.Cd(m, a) - want);
                    atM = m; atA = OffNodeReference[i, 1];
                }
            }
            Check("matches the source curve between breakpoints", worstRel < 0.01,
                  $"worst {100 * worstRel:F3}% ({worstAbs:E2} Cd) at M={atM:F3} a={atA:F1}deg");
        }

        // ---- analytic gradient vs finite differences --------------------
        {
            double worst = 0;
            var rnd = new Random(4242);
            for (int i = 0; i < 20000; i++)
            {
                double m = 0.05 + rnd.NextDouble() * 4.9;
                double a = rnd.NextDouble() * 179.5 * Deg;
                tab.Cd(m, a, out double dM, out double dA);
                const double h = 1e-6;
                double fdM = (tab.Cd(m + h, a) - tab.Cd(m - h, a)) / (2 * h);
                double fdA = (tab.Cd(m, a + h) - tab.Cd(m, a - h)) / (2 * h);
                worst = Math.Max(worst, Math.Abs(dM - fdM) / (1 + Math.Abs(fdM)));
                worst = Math.Max(worst, Math.Abs(dA - fdA) / (1 + Math.Abs(fdA)));
            }
            Check("analytic gradient == finite differences", worst < 1e-7, $"max rel {worst:E2}");
        }

        // ---- the Dual bridge, seeded one input at a time ----------------
        {
            double worst = 0;
            var rnd = new Random(99);
            for (int i = 0; i < 20000; i++)
            {
                double m = 0.05 + rnd.NextDouble() * 4.9;
                double a = rnd.NextDouble() * 179.5 * Deg;
                tab.Cd(m, a, out double dM, out double dA);

                Dual seedM = tab.Cd(Dual.Seed(m), new Dual(a));
                Dual seedA = tab.Cd(new Dual(m), Dual.Seed(a));

                worst = Math.Max(worst, Math.Abs(seedM.V - tab.Cd(m, a)));
                worst = Math.Max(worst, Math.Abs(seedM.D - dM) / (1 + Math.Abs(dM)));
                worst = Math.Max(worst, Math.Abs(seedA.D - dA) / (1 + Math.Abs(dA)));
            }
            Check("Dual bridge reproduces value and both slopes", worst < 1e-12,
                  $"max rel {worst:E2}");
        }

        // A Dual carrying no seed must come back with zero derivative, and a
        // linear combination of seeds must chain correctly - this is what makes
        // the table usable mid-expression rather than only at a leaf.
        {
            double m = 1.35, a = 12.0 * Deg;
            tab.Cd(m, a, out double dM, out double dA);
            Dual constant = tab.Cd(new Dual(m), new Dual(a));
            // Seed a shared upstream variable t with M = 2t, alpha = 3t.
            Dual chained = tab.Cd(new Dual(m, 2.0), new Dual(a, 3.0));
            bool ok = Math.Abs(constant.D) < 1e-15
                   && Math.Abs(chained.D - (2.0 * dM + 3.0 * dA)) < 1e-12;
            Check("constants carry zero slope; chain rule composes", ok,
                  $"chained {chained.D:F8} want {2 * dM + 3 * dA:F8}");
        }

        // ---- off-table behaviour ----------------------------------------
        {
            // Past M = 5 the extension must stay linear and keep its slope, not
            // flatten. A gradient that drops to zero at the edge is what makes an
            // optimiser stall exactly when it steps off the table.
            // Compare AT the boundary, not just inside it: the last span is still
            // curving at M = 4.99, so its slope legitimately differs from the
            // boundary slope the extension carries outward.
            tab.Cd(5.00, 10 * Deg, out double dIn, out _);
            tab.Cd(5.60, 10 * Deg, out double dOut, out _);
            double v5 = tab.Cd(5.0, 10 * Deg), v6 = tab.Cd(6.0, 10 * Deg), v7 = tab.Cd(7.0, 10 * Deg);
            Check("slope survives the Mach edge", Math.Abs(dIn - dOut) < 1e-6,
                  $"dCd/dM {dIn:F5} inside -> {dOut:F5} outside");
            Check("extension is linear past the last breakpoint",
                  Math.Abs(v7 - 2 * v6 + v5) < 1e-9, $"2nd difference {v7 - 2 * v6 + v5:E2}");
        }

        // ---- the real test: a composed aero function, swept for its Jacobian ----
        Console.WriteLine();
        Console.WriteLine("  composed drag function under a full AD sweep");
        {
            var p = new DragParams { Rho = 0.35, SpeedOfSound = 295.0, Area = 10.8 };
            double worst = 0;
            int worstIn = -1, worstOut = -1;
            var rnd = new Random(2024);

            for (int trial = 0; trial < 500; trial++)
            {
                // Body-frame velocity spanning subsonic through hypersonic, with a
                // realistic cross-flow component, plus mass.
                double[] s =
                {
                    (rnd.NextDouble() - 0.5) * 60.0,     // vbx
                    (rnd.NextDouble() - 0.5) * 60.0,     // vby
                    -(50.0 + rnd.NextDouble() * 1400.0), // vbz, falling engine-first
                    18_000.0 + rnd.NextDouble() * 40_000.0
                };

                // AD sweep: one pass per input, exactly as Dynamics6Dof.Jacobian does.
                var jac = new double[3, 4];
                Span<Dual> outp = stackalloc Dual[3];
                for (int col = 0; col < 4; col++)
                {
                    Span<Dual> d = stackalloc Dual[4];
                    for (int i = 0; i < 4; i++) d[i] = new Dual(s[i], col == i ? 1.0 : 0.0);
                    DragAccelBody(tab, d[0], d[1], d[2], d[3], p, outp);
                    for (int r = 0; r < 3; r++) jac[r, col] = outp[r].D;
                }

                // Central differences on the same function.
                for (int col = 0; col < 4; col++)
                {
                    double h = Math.Max(1e-5, Math.Abs(s[col]) * 1e-6);
                    var plus = new double[3];
                    var minus = new double[3];
                    EvalValue(tab, s, col, +h, p, plus);
                    EvalValue(tab, s, col, -h, p, minus);
                    for (int r = 0; r < 3; r++)
                    {
                        double fd = (plus[r] - minus[r]) / (2 * h);
                        double rel = Math.Abs(jac[r, col] - fd) / (1 + Math.Abs(fd));
                        if (rel > worst) { worst = rel; worstIn = col; worstOut = r; }
                    }
                }
            }
            Check("AD Jacobian == finite differences (3x4, 500 states)", worst < 1e-6,
                  $"max rel {worst:E2}" + (worstIn >= 0 ? $" at d(a{worstOut})/d(in{worstIn})" : ""));
        }

        // ---- cost -------------------------------------------------------
        Console.WriteLine();
        {
            const int N = 200_000;
            var p = new DragParams { Rho = 0.35, SpeedOfSound = 295.0, Area = 10.8 };
            Span<Dual> outp = stackalloc Dual[3];

            // warm
            for (int i = 0; i < 20000; i++)
                DragAccelBody(tab, new Dual(10), new Dual(5), new Dual(-400), new Dual(30000), p, outp);

            var sw = Stopwatch.StartNew();
            double acc = 0;
            for (int i = 0; i < N; i++)
            {
                DragAccelBody(tab, new Dual(10), new Dual(5), new Dual(-400 - i % 500),
                              new Dual(30000), p, outp);
                acc += outp[2].V;
            }
            sw.Stop();
            double nsOne = sw.Elapsed.TotalMilliseconds * 1e6 / N;

            // Decompose it: how much of the drag call is the table, and how much is
            // the surrounding arithmetic? That decides whether caching is worth anything.
            var swv = Stopwatch.StartNew();
            double accv = 0;
            for (int i = 0; i < N; i++) accv += tab.Cd(0.5 + (i % 400) * 0.01, 0.1);
            swv.Stop();
            double nsValue = swv.Elapsed.TotalMilliseconds * 1e6 / N;

            swv.Restart();
            double accg = 0;
            for (int i = 0; i < N; i++)
                accg += tab.Cd(0.5 + (i % 400) * 0.01, 0.1, out double a1, out double a2) + a1 + a2;
            swv.Stop();
            double nsGrad = swv.Elapsed.TotalMilliseconds * 1e6 / N;

            Console.WriteLine($"  table, value only        : {nsValue,7:F0} ns");
            Console.WriteLine($"  table, value + gradient  : {nsGrad,7:F0} ns   (what the Dual bridge uses)");
            Console.WriteLine($"  one Dual drag evaluation : {nsOne,7:F0} ns   "
                            + $"({100 * nsGrad / nsOne:F0}% of it is the table)");
            Console.WriteLine($"  (checksums {accv:F0} {accg:F0})");
            Console.WriteLine($"  an 18-column sweep       : {nsOne * 18,7:F0} ns per node");
            Console.WriteLine($"  at 30 nodes              : {nsOne * 18 * 30 / 1000.0,7:F1} us per Jacobian pass");
            Console.WriteLine($"  (checksum {acc:F1})");
            Console.WriteLine();
            Console.WriteLine("  Every column of the sweep re-queries the table at the SAME Mach and");
            Console.WriteLine("  alpha, because only the seed differs, so one cached (M, alpha) result");
            Console.WriteLine($"  per node would remove ~18 of every 19 table calls. That caps the win at");
            Console.WriteLine($"  the table's share above - about {100 * nsGrad / nsOne:F0}% - so it is worth roughly a third");
            Console.WriteLine("  of the sweep, not all of it. The rest is Atan2, Sqrt and the Dual");
            Console.WriteLine("  arithmetic, which caching does not touch. Left out for now.");
        }

        // ---- the atmosphere ---------------------------------------------
        // The other half of a self-contained aero model: rho(h) with no game behind
        // it. The mirror against KSA's own function can only be checked in the mod -
        // it needs the game loaded - so what is checkable here is that the formula is
        // the one we think it is, and that it differentiates.
        Console.WriteLine();
        Console.WriteLine("  atmosphere (KSA Earth: 1.225 kg/m^3, 1 atm, 8 km)");
        {
            var atm = ExponentialAtmosphere.Earth;

            Console.WriteLine($"    top          {atm.TopAltitude / 1000.0,8:F2} km");
            Console.WriteLine($"    a            {atm.SpeedOfSound,8:F2} m/s");

            Check("rho(0) == rho0", Math.Abs(atm.Density(0) - 1.225) < 1e-12);
            Check("rho(H) == rho0/e",
                  Math.Abs(atm.Density(8000) - 1.225 / Math.E) < 1e-12,
                  $"{atm.Density(8000):F6}");

            // KSA's boundary is the LAST of the two floors to be reached. For Earth
            // density is the binding one (1.225 -> 1e-9 takes longer than 101325 ->
            // 1e-4), so rho at the top should be exactly the density floor.
            Check("top is set by the density floor",
                  Math.Abs(atm.Density(atm.TopAltitude - 1e-6) - 1e-9) < 1e-15,
                  $"rho(top) = {atm.Density(atm.TopAltitude - 1e-6):E3}");
            Check("rho is exactly zero above the top", atm.Density(atm.TopAltitude) == 0.0
                                                    && atm.Density(1e9) == 0.0);

            // The clamp below mean radius, mirrored from the game deliberately. It is
            // checked rather than merely commented so that removing it is a test
            // failure and therefore a decision.
            Check("rho is clamped below mean radius (mirrors KSA)",
                  atm.Density(-5000) == atm.Density(0));

            // Isothermal means one speed of sound everywhere; the value is derived
            // from P0/rho0, not assumed. 340.3 m/s is the right answer for Earth.
            Check("speed of sound == sqrt(gamma P0 / rho0)",
                  Math.Abs(atm.SpeedOfSound - Math.Sqrt(1.4 * 101325.0 / 1.225)) < 1e-12,
                  $"{atm.SpeedOfSound:F2} m/s");

            // The Dual bridge. Away from the two kinks the slope must be exact.
            double worstRho = 0;
            for (double h = 500; h < atm.TopAltitude - 500; h += 250)
            {
                Dual d = atm.Density(Dual.Seed(h));
                const double step = 1e-3;
                double fd = (atm.Density(h + step) - atm.Density(h - step)) / (2 * step);
                worstRho = Math.Max(worstRho, Math.Abs(d.D - fd) / (1 + Math.Abs(fd)));
            }
            Check("d(rho)/dh == finite differences", worstRho < 1e-9, $"max rel {worstRho:E2}");

            // Value agreement between the two overloads, which is what stops the
            // differentiable path from quietly drifting from the plain one.
            double worstVal = 0;
            for (double h = -2000; h < atm.TopAltitude + 2000; h += 137)
                worstVal = Math.Max(worstVal,
                    Math.Abs(atm.Density(new Dual(h)).V - atm.Density(h)));
            // EXACT equality, not a tolerance. The two overloads run the same
            // arithmetic on purpose - see the note in Density(Dual) - so any
            // difference at all means they have drifted apart.
            Check("Dual and double overloads agree", worstVal == 0.0, $"max {worstVal:E2}");
        }

        Console.WriteLine();
        Console.WriteLine(fails == 0 ? "AERO: all checks passed" : $"AERO: {fails} check(s) FAILED");
        return fails == 0 ? 0 : 1;
    }

    // (Mach, alphaDeg, Cd) sampled OFF the breakpoints from the same smooth model
    // the table itself was generated from. At the breakpoints an interpolating
    // spline is exact by construction, so those prove nothing; these are the only
    // points that measure the fit.
    private static readonly double[,] OffNodeReference =
    {
        { 0.100, 1.0, 1.050435 },  { 0.350, 7.0, 1.070814 },  { 0.550, 35.0, 1.502399 },
        { 0.750, 13.0, 1.224246 },  { 0.825, 55.0, 2.270155 },  { 0.875, 22.0, 1.697413 },
        { 0.925, 95.0, 3.077561 },  { 0.975, 17.0, 2.127845 },  { 1.025, 1.5, 2.202798 },
        { 1.075, 115.0, 3.268670 },  { 1.125, 27.0, 2.928830 },  { 1.175, 4.0, 2.561061 },
        { 1.250, 19.0, 2.832451 },  { 1.350, 145.0, 1.826839 },  { 1.500, 25.0, 2.969418 },
        { 1.700, 68.0, 4.248681 },  { 1.900, 2.5, 2.467070 },  { 2.250, 29.0, 2.784467 },
        { 2.750, 178.0, 0.613752 },  { 3.250, 21.0, 2.013068 },  { 3.750, 10.5, 1.655940 },
        { 4.500, 160.0, 0.681829 },
    };

    internal struct DragParams
    {
        public double Rho;           // kg/m^3
        public double SpeedOfSound;  // m/s
        public double Area;          // m^2, reference area
    }

    /// <summary>
    /// Drag acceleration in body axes. Written exactly the way a term in
    /// Dynamics6Dof.F is written: generic over Dual, no derivative bookkeeping,
    /// with the table lookup sitting mid-expression like any other operation.
    /// </summary>
    private static void DragAccelBody(AeroTable tab, Dual vbx, Dual vby, Dual vbz, Dual mass,
                                      DragParams p, Span<Dual> accel)
    {
        Dual v2 = vbx * vbx + vby * vby + vbz * vbz + 1e-18;
        Dual speed = Dual.Sqrt(v2);
        Dual mach = speed / p.SpeedOfSound;
        Dual alpha = AeroTable.AngleOfAttack(vbx, vby, vbz);

        Dual cd = tab.Cd(mach, alpha);

        // D = 1/2 rho V^2 S Cd, opposing the velocity: a = -(D / (m V)) * v
        Dual k = 0.5 * p.Rho * v2 * p.Area * cd / (mass * speed);
        accel[0] = -(k * vbx);
        accel[1] = -(k * vby);
        accel[2] = -(k * vbz);
    }

    /// <summary>Value-only evaluation with one input perturbed, for the FD reference.</summary>
    private static void EvalValue(AeroTable tab, double[] s, int col, double h,
                                  DragParams p, double[] outv)
    {
        Span<Dual> d = stackalloc Dual[4];
        for (int i = 0; i < 4; i++) d[i] = new Dual(s[i] + (i == col ? h : 0.0));
        Span<Dual> r = stackalloc Dual[3];
        DragAccelBody(tab, d[0], d[1], d[2], d[3], p, r);
        for (int i = 0; i < 3; i++) outv[i] = r[i].V;
    }
}

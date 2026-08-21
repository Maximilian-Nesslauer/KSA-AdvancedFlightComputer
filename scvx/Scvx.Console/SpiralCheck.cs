using Scvx;

/// <summary>
/// Reproduces the spiralling descent seen in flight, from the exact logged initial
/// condition, and tests what removes it.
///
/// The flight logs established that the PLAN is the spiral, not the tracking: at
/// engage the plan swept +319 degrees of bearing around the target, swinging out
/// from 235 m to 311 m before coming in, and the vehicle followed it to within 4
/// degrees of thrust direction. So the question is entirely why the optimiser picks
/// that trajectory, and it can be asked offline.
///
/// The suspicion under test: the vehicle arrives with far more time than it needs.
/// Descending 1971 m from 167 m/s takes about 13 s flown hard, but the solve settled
/// on sigma = 27 s. With a thrust FLOOR the engine cannot be shut down, so surplus
/// time cannot be coasted away - and the only way to shed vertical thrust while
/// staying lit is to TILT. Once tilted at the limit the lateral thrust has to point
/// somewhere, and rotating it produces no net translation: an orbit.
/// </summary>
internal static class SpiralCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    internal static int Run()
    {
        // Exactly as logged at engage.
        var x0 = new double[NX];
        x0[0] = -230.867; x0[1] = 41.572; x0[2] = 1971.040;
        x0[3] = 0.107; x0[4] = -0.460; x0[5] = -167.122;
        // THE LOGGED ATTITUDE, verbatim. qw ~ 0, qz ~ -1 is a 180 degree YAW: the
        // vehicle is upright (body +Z is still within a fraction of a degree of
        // vertical) but pointing backwards about its own thrust axis.
        x0[6] = 0.00178578458; x0[7] = -0.000683048758;
        x0[8] = -0.00125673693; x0[9] = -0.999997383;
        x0[13] = 129495.15;

        var xf = new double[NX];
        xf[2] = 10.0;
        xf[6] = 1.0;

        const double Tmax = 6030312.0;
        double m0 = x0[13];

        Console.WriteLine("SPIRAL REPRODUCTION - from the logged engage state");
        Console.WriteLine($"  {x0[2]:F0} m up, {Math.Sqrt(x0[0] * x0[0] + x0[1] * x0[1]):F0} m downrange, "
                          + $"descending {-x0[5]:F0} m/s, {m0 / 1000:F0} t, Tmax {Tmax / 1e6:F2} MN "
                          + $"(TWR {Tmax / (m0 * 9.81):F1})");
        Console.WriteLine();
        Console.WriteLine("  A min-fuel solve with a thrust FLOOR should want the SHORTEST burn:");
        Console.WriteLine("  the engine cannot be switched off, so every extra second costs propellant.");
        Console.WriteLine();
        Console.WriteLine($"  {"seed",5} {"floor",6} {"tilt",5} {"iters",6} {"status",-16} "
                          + $"{"variant",-9} {"sigma",6} {"sweep",7} {"maxRange",9} {"peak tilt",10}  verdict");

        // Vary the burn-time seed, the throttle floor and the tilt limit independently.
        (double seed, double floor, double tilt, int iters, bool upright, bool freeRoll)[] cases =
        [
            (20.0, 0.10, 60.0,  25, false, false),   // exactly as flown
            (20.0, 0.10, 60.0, 150, false, false),   // as flown, solved to convergence
            (20.0, 0.10, 60.0, 150, true,  false),   // IDENTITY start: no 180 deg roll to undo
            (20.0, 0.10, 60.0, 150, false, true),    // logged start, but terminal ROLL left free
            (20.0, 0.01, 60.0, 150, false, false),   // near-zero floor
            (20.0, 0.10, 25.0, 150, false, false),   // tight tilt limit
        ];

        foreach ((double seed, double floor, double tilt, int iters, bool upright, bool freeRoll) in cases)
        {
            var xs0 = (double[])x0.Clone();
            if (upright) { xs0[6] = 1.0; xs0[7] = xs0[8] = xs0[9] = 0.0; }
            // "Free roll" approximates letting the terminal yaw be whatever it
            // already is, instead of demanding identity. For a landing, roll about
            // the thrust axis is the one attitude degree of freedom that does not
            // matter, so pinning it is asking for work with no purpose.
            var xft = (double[])xf.Clone();
            if (freeRoll) { xft[6] = xs0[6]; xft[7] = 0.0; xft[8] = 0.0; xft[9] = xs0[9]; }
            var cfg = new Scvx6DofConfig
            {
                Nodes = 50,
                Tmax = Tmax,
                ThrottleFloor = floor,
                TiltMaxDeg = tilt,
                GimbalMaxDeg = 10.0,
                TauRollMax = 1.0e5,
                WDu = 0.05,
                WW = 0.002,
                ProximalWeight = 0.05,
                SigmaScale = seed,
                SigmaMin = seed * 0.15,
                SigmaMax = seed * 4.0,
                XScale = Scale(x0, xf),
            };
            var dyn = new Dynamics6Dof.Params { Gz = -9.81, Isp = 300.0, LArm = 25.0 };

            var xSeed = new double[50 * NX];
            var uSeed = new double[50 * NU];
            for (int k = 0; k < 50; k++)
            {
                double t = k / 49.0;
                for (int i = 0; i < 3; i++)
                {
                    xSeed[k * NX + i] = xs0[i] + t * (xft[i] - xs0[i]);
                    xSeed[k * NX + 3 + i] = xs0[3 + i] * (1.0 - t);
                }
                for (int i = 0; i < 4; i++)
                    xSeed[k * NX + 6 + i] = xs0[6 + i] * (1.0 - t) + xft[6 + i] * t;
                Norm(xSeed, k);
                xSeed[k * NX + 13] = m0 * (1.0 - 0.10 * t);
                uSeed[k * NU + 2] = 1.05 * m0 * 9.81;
            }
            Array.Copy(xs0, 0, xSeed, 0, NX);

            var solver = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
            solver.Initialize(xs0, xft, xSeed, uSeed, seed);
            ScvxStatus st = solver.Solve(iters);

            double[] x = solver.ReferenceX;
            (double sweep, double maxRange, double peakTilt) = Shape(x, 50);
            string verdict = Math.Abs(sweep) > 120 ? "SPIRAL"
                           : Math.Abs(sweep) > 45 ? "curved"
                           : "direct";
            Console.WriteLine($"  {seed,5:F0} {floor,6:F2} {tilt,5:F0} {iters,6} {st,-16} "
                              + $"{(upright ? "up" : freeRoll ? "roll" : "as-flown"),-9} "
                              + $"{solver.Sigma,6:F1} {sweep,6:F0}d {maxRange,8:F0}m {peakTilt,9:F1}d  {verdict}");
        }

        Console.WriteLine();
        Console.WriteLine("  sweep = degrees of bearing travelled AROUND the target;");
        Console.WriteLine("  a direct approach is near 0, the flown plan was +319.");
        return 0;
    }

    private static void Norm(double[] x, int k)
    {
        double m = 0.0;
        for (int i = 6; i < 10; i++) m += x[k * NX + i] * x[k * NX + i];
        m = Math.Sqrt(m);
        if (m > 1e-12) for (int i = 6; i < 10; i++) x[k * NX + i] /= m;
        else { x[k * NX + 6] = 1.0; x[k * NX + 7] = x[k * NX + 8] = x[k * NX + 9] = 0.0; }
    }

    private static double[] Scale(double[] x0, double[] xf)
    {
        double L = Math.Sqrt((x0[0] - xf[0]) * (x0[0] - xf[0]) +
                             (x0[1] - xf[1]) * (x0[1] - xf[1]) +
                             (x0[2] - xf[2]) * (x0[2] - xf[2]));
        double sp = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
        double V = Math.Max(Math.Max(sp, Math.Sqrt(L * 9.81)), 1.0);
        return [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, x0[13]];
    }

    private static (double sweep, double maxRange, double peakTilt) Shape(double[] x, int n)
    {
        double sweep = 0.0, maxRange = 0.0, peakTilt = 0.0, prev = 0.0;
        for (int k = 0; k < n; k++)
        {
            double px = x[k * NX], py = x[k * NX + 1];
            double b = Math.Atan2(py, px);
            if (k > 0)
            {
                double dd = b - prev;
                while (dd > Math.PI) dd -= 2 * Math.PI;
                while (dd < -Math.PI) dd += 2 * Math.PI;
                // Bearing is meaningless once essentially on top of the target.
                if (Math.Sqrt(px * px + py * py) > 5.0) sweep += dd;
            }
            prev = b;
            maxRange = Math.Max(maxRange, Math.Sqrt(px * px + py * py));
            double qx = x[k * NX + 7], qy = x[k * NX + 8];
            peakTilt = Math.Max(peakTilt,
                Math.Acos(Math.Clamp(1 - 2 * (qx * qx + qy * qy), -1, 1)) * 180.0 / Math.PI);
        }
        return (sweep * 180.0 / Math.PI, maxRange, peakTilt);
    }
}

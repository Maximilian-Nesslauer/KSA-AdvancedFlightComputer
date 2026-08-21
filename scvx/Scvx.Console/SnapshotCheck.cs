using Scvx;

/// <summary>
/// Is a solve a PURE FUNCTION of its inputs?
///
/// This is the property that has to hold before the solve can move off the sim
/// thread, and it is worth nailing down while everything is still single-threaded and
/// deterministic. Once there is a worker, a violation shows up as a plan that is
/// occasionally and unreproducibly wrong — and telling that apart from a race, a stale
/// seed, or a genuine convergence failure means log forensics on a nondeterministic
/// system, which is the worst place to be doing first threading work.
///
/// Three things, and they are different:
///
///   REPLAY. The same problem solved twice must give the same plan bit for bit.
///   Anything hidden — a cached value, a clock, a static — shows up here as a
///   difference the inputs cannot account for.
///
///   THE INPUTS MUST MATTER. Changing the dynamics parameters must change the answer.
///   Without this the replay check is vacuous: a solve that ignored its inputs
///   entirely would pass it.
///
///   NO CROSS-TALK. Two solvers alive at once, with different parameters, must not
///   influence each other. That is the shape the threaded version takes — a worker
///   holding one solver while the sim thread builds the next at a new node count —
///   and shared mutable state between instances would be invisible until then.
///
/// WHAT THIS DOES NOT COVER. The guidance-level rule — that published inputs reach the
/// model only at a solve's entry, never during — cannot be exercised here, because
/// Ksa6DofGuidance depends on KSA types this harness cannot reference. That half is
/// structural rather than behavioural and holds by inspection: every write to _dyn in
/// Ksa6DofGuidance lives inside CommitInputs, and CommitInputs is called only at the
/// top of Plan, Update, StepCold, SeedFrom and BeginCold. It is a one-line grep to
/// re-verify, and worth doing after any change to that file.
/// </summary>
internal static class SnapshotCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;

    /// <summary>Engage state from flight 20260808-104651: 1552 m, 126 m/s down, 92.1 deg tilt.</summary>
    private static readonly double[] EngageState =
    [
        69.0586, -15.5324, 1552.0303,
        -0.4775, -0.2973, -126.1480,
        0.00027153, 0.72014772, -0.00198499, -0.69381788,
        1e-4, -1e-4, 1e-4,
        132131.141,
    ];

    internal static int Run()
    {
        Console.WriteLine("SNAPSHOT BOUNDARY: A SOLVE IS A PURE FUNCTION OF ITS INPUTS");
        Console.WriteLine("  the property the solve has to have before it can leave the sim thread");
        Console.WriteLine();

        const double g0 = -9.81;
        const double biased = -9.81 + 3.0;      // as if a 3 m/s^2 bias had been committed

        double[] a1 = Solve(g0, out double sigmaA);
        double[] a2 = Solve(g0, out _);
        double[] b1 = Solve(biased, out double sigmaB);

        bool replay = Identical(a1, a2);
        bool matters = !Identical(a1, b1);

        Console.WriteLine($"  solve at gz {g0:F2}                        : sigma {sigmaA:F4} s");
        Console.WriteLine($"  same again                                : "
                          + $"{(replay ? "bit-identical" : "DIFFERS - something is not in the inputs")}");
        Console.WriteLine($"  solve at gz {biased:F2} (3 m/s^2 of bias)     : sigma {sigmaB:F4} s   "
                          + $"{(matters ? "differs - the inputs reach the model" : "IDENTICAL - inputs ignored")}");

        // Cross-talk: two solvers alive at once, interleaved, must be unaffected by
        // each other. This is the shape the threaded version takes.
        (double[] x, double[] y) = Interleaved(g0, biased);
        bool noCrossTalk = Identical(x, a1) && Identical(y, b1);
        Console.WriteLine($"  two solvers interleaved                   : "
                          + $"{(noCrossTalk ? "both match their solo runs" : "DIFFER - shared state between instances")}");

        Console.WriteLine();
        Console.WriteLine($"    a solve replays bit-for-bit          : {(replay ? "yes" : "NO")}");
        Console.WriteLine($"    its inputs actually reach the model  : {(matters ? "yes" : "NO")}");
        Console.WriteLine($"    instances do not influence each other: {(noCrossTalk ? "yes" : "NO")}");
        Console.WriteLine();

        bool ok = replay && matters && noCrossTalk;
        Console.WriteLine(ok
            ? "PASS - the solve depends on its inputs and nothing else"
            : "FAIL - see above");
        return ok ? 0 : 1;
    }

    /// <summary>Two independent solvers, stepped alternately rather than one after the other.</summary>
    private static (double[] a, double[] b) Interleaved(double gzA, double gzB)
    {
        (Scvx6DofSolver sa, _) = Build(gzA);
        (Scvx6DofSolver sb, _) = Build(gzB);
        for (int i = 0; i < 8; i++)
        {
            sa.Iterate();
            sb.Iterate();
        }
        return (sa.ReferenceX.ToArray(), sb.ReferenceX.ToArray());
    }

    private static double[] Solve(double gz, out double sigma)
    {
        (Scvx6DofSolver s, _) = Build(gz);
        for (int i = 0; i < 8; i++)
            s.Iterate();
        sigma = s.Sigma;
        return s.ReferenceX.ToArray();
    }

    private static bool Identical(double[] a, double[] b)
    {
        if (a.Length != b.Length) return false;
        for (int i = 0; i < a.Length; i++)
            if (a[i] != b[i]) return false;
        return true;
    }

    private static (Scvx6DofSolver s, double L) Build(double gz)
    {
        var x0 = (double[])EngageState.Clone();
        double alt = x0[2], down = Math.Sqrt(x0[0] * x0[0] + x0[1] * x0[1]);
        var xf = new double[NX];
        xf[2] = 10.0; xf[6] = 1.0;

        const int n = 30;
        double m0 = x0[13];
        double L = Math.Sqrt(down * down + (alt - 10.0) * (alt - 10.0));
        double V = Math.Max(Math.Max(Math.Abs(x0[5]), Math.Sqrt(L * 9.81)), 1.0);
        double seed = Math.Max(2.0 * L / Math.Max(V, 1.0), 4.0);

        var cfg = new Scvx6DofConfig
        {
            Nodes = n,
            Tmax = 6.03e6,
            ThrottleFloor = 0.10,
            TiltMaxDeg = 120.0,
            WDu = 0.05,
            WW = 0.002,
            ProximalWeight = 0.05,
            SigmaScale = seed,
            SigmaMin = seed * 0.15,
            SigmaMax = seed * 4.0,
            XScale = [L, L, L, V, V, V, 1, 1, 1, 1, 1, 1, 1, m0],
        };
        var dyn = new Dynamics6Dof.Params { Gz = gz, Isp = 300.0 };

        var xSeed = new double[n * NX];
        var uSeed = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double t = (double)k / (n - 1);
            for (int i = 0; i < 3; i++)
            {
                xSeed[k * NX + i] = x0[i] + t * (xf[i] - x0[i]);
                xSeed[k * NX + 3 + i] = x0[3 + i] * (1.0 - t);
            }
            double qn = 0.0;
            for (int i = 0; i < 4; i++)
            {
                double q = x0[6 + i] * (1.0 - t) + xf[6 + i] * t;
                xSeed[k * NX + 6 + i] = q;
                qn += q * q;
            }
            qn = Math.Sqrt(qn);
            if (qn > 1e-12)
                for (int i = 0; i < 4; i++) xSeed[k * NX + 6 + i] /= qn;
            xSeed[k * NX + 13] = m0 * (1.0 - 0.08 * t);
            uSeed[k * NU + 2] = 1.05 * m0 * 9.81;
        }
        Array.Copy(x0, 0, xSeed, 0, NX);

        var s = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = Scvx6DofSolver.RealTimeEps };
        s.Initialize(x0, xf, xSeed, uSeed, seed);
        return (s, L);
    }
}

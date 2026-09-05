using Scvx;

/// <summary>
/// Does the solve survive being driven from another thread, and does the publish
/// pattern give the reader a whole plan every time?
///
/// Two separate worries, and the second is the subtle one:
///
///   THE SOLVER OFF-THREAD. SCS is native and holds a workspace per solver instance.
///   Driving one from a background thread must produce exactly what driving it inline
///   produces — not approximately, exactly, since nothing about the arithmetic should
///   care which thread runs it. A difference here would mean shared mutable state
///   somewhere in the native layer, and the whole approach would be off.
///
///   THE PUBLISH. A plan is four things written together: controls, burn time, anchor
///   time, node count. Read as four fields, a solve landing between two of them pairs
///   new controls with an old anchor, and the vehicle is commanded from the wrong
///   point of the right trajectory — which looks exactly like guidance ignoring its
///   own plan, and would be maddening to find in a log. Published as one immutable
///   object swapped by reference, the reader gets the whole previous plan or the whole
///   new one. This hammers the read while solves land underneath it and checks that
///   every observation was internally consistent.
///
/// A passing run does not prove the mod's threading is correct — the ownership rule
/// around rebuilds and cold restarts lives in PoweredGuidance6Dof, which needs KSA
/// types this harness cannot reference. It proves the foundation those rules sit on.
/// </summary>
internal static class ThreadCheck
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

    /// <summary>Mirrors Ksa6DofPlan: the four things that must move together.</summary>
    private sealed record Snapshot(double[] U, double Sigma, double SolveTime, int Nodes);

    private static volatile Snapshot _published;

    internal static int Run()
    {
        Console.WriteLine("SOLVING OFF THE SIM THREAD");
        Console.WriteLine();

        // 1. Same answer on a worker as inline.
        double[] inline = SolveInline(out double sigmaInline);
        double[] worker = SolveOnWorker(out double sigmaWorker);
        bool identical = Identical(inline, worker);
        Console.WriteLine($"  inline  : sigma {sigmaInline:F4} s");
        Console.WriteLine($"  worker  : sigma {sigmaWorker:F4} s   "
                          + $"{(identical ? "bit-identical" : "DIFFERS - the thread changed the answer")}");

        // 2. Hammer the published snapshot while solves land underneath it.
        (long reads, long tears, int publishes) = HammerPublish();
        Console.WriteLine($"  publish : {publishes} plans published, {reads:N0} reads, {tears} torn");
        Console.WriteLine();

        Console.WriteLine($"    a solve gives the same answer off-thread : {(identical ? "yes" : "NO")}");
        Console.WriteLine($"    no reader ever saw a half-published plan : {(tears == 0 ? "yes" : "NO")}");
        Console.WriteLine();

        bool ok = identical && tears == 0 && publishes > 0 && reads > 0;
        Console.WriteLine(ok
            ? "PASS - the solve is thread-agnostic and the publish is atomic to a reader"
            : "FAIL - see above");
        return ok ? 0 : 1;
    }

    /// <summary>
    /// One thread publishes internally-consistent snapshots as fast as it can; another
    /// reads and checks consistency. The fields are correlated on purpose — Sigma,
    /// SolveTime and U all carry the same generation number — so a torn read is
    /// detectable rather than merely possible.
    /// </summary>
    private static volatile bool _stopProducing;

    private static (long reads, long tears, int publishes) HammerPublish()
    {
        const long TargetReads = 2_000_000;
        long reads = 0, tears = 0;
        int publishes = 0;

        // THE READER DRIVES. An earlier version had the producer run a fixed number of
        // generations and the reader spin until it finished - and the producer was done
        // before the reader's first iteration, so it made zero observations and passed
        // by measuring nothing. A concurrency check that can trivially observe nothing
        // is worse than none, because it reports success.
        _stopProducing = false;
        _published = null;

        var producer = new System.Threading.Thread(() =>
        {
            int gen = 0;
            while (!_stopProducing)
            {
                gen++;
                var u = new double[8 * NU];
                for (int i = 0; i < u.Length; i++) u[i] = gen;
                _published = new Snapshot(u, gen, gen, 8);
                publishes = gen;
            }
        }) { IsBackground = true };

        producer.Start();
        while (reads < TargetReads)
        {
            Snapshot s = _published;
            if (s == null) continue;
            reads++;
            // Every field of a generation carries the same number. If any disagrees,
            // the reader has seen parts of two different plans.
            if (s.Sigma != s.SolveTime || s.U[0] != s.Sigma || s.U[^1] != s.Sigma)
                tears++;
        }
        _stopProducing = true;
        producer.Join(TimeSpan.FromSeconds(10));

        return (reads, tears, publishes);
    }

    private static double[] SolveInline(out double sigma)
    {
        Scvx6DofSolver s = Build();
        for (int i = 0; i < 8; i++) s.Iterate();
        sigma = s.Sigma;
        return s.ReferenceX.ToArray();
    }

    private static double[] SolveOnWorker(out double sigma)
    {
        Scvx6DofSolver s = Build();
        double[] result = null;
        double sig = 0.0;
        var t = new System.Threading.Thread(() =>
        {
            for (int i = 0; i < 8; i++) s.Iterate();
            sig = s.Sigma;
            result = s.ReferenceX.ToArray();
        }) { IsBackground = true };
        t.Start();
        if (!t.Join(TimeSpan.FromSeconds(60)))
            throw new TimeoutException("the worker never finished - a deadlock, or a solve gone wild");
        sigma = sig;
        return result;
    }

    private static bool Identical(double[] a, double[] b)
    {
        if (a == null || b == null || a.Length != b.Length) return false;
        for (int i = 0; i < a.Length; i++)
            if (a[i] != b[i]) return false;
        return true;
    }

    private static Scvx6DofSolver Build()
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
        var dyn = new Dynamics6Dof.Params { Gz = -9.81, Isp = 300.0 };

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
        return s;
    }
}

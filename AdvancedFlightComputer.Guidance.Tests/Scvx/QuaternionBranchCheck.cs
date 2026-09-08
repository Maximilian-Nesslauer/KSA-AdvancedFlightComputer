using Scvx;

/// <summary>
/// Does a quaternion SIGN FLIP in the measured attitude wreck the plan, and does
/// branch alignment fix it?
///
/// q and -q are the same rotation. Nothing physical distinguishes them, and a vehicle
/// rotating through qw = 0 will hand back one and then the other from one frame to the
/// next. The collocation defect, however, is plain arithmetic on the components: an
/// interval whose two ends sit on opposite branches shows a jump of up to 2 in a
/// channel whose scale is 1, and the plan gets refused for a discrepancy that does not
/// exist.
///
/// Flight log 20260808-122824, three consecutive frames:
///
///     t+3.92   qw +0.000009  qx +0.68365  qz -0.72981    defect    0.2 m
///     t+3.93   qw +0.000027  qx -0.68301  qz +0.73041    defect    0.2 m
///     t+3.95   qw +0.000066  qx -0.68235  qz +0.73102    defect 3548.0 m
///
/// reported as "worst on qz (attitude) at interval 0 = 1.47" — and 1.47 is exactly
/// 0.73041 - (-0.72897), the antipodal gap. Fifteen refusals followed, then a cold
/// restart, which repaired it only because the cold seed goes through Slerp and Slerp
/// already takes the short way round.
///
/// This reproduces that from the same engage state and checks both halves: that the
/// flip really does cause it, and that aligning the plan onto the measurement's branch
/// removes it completely rather than merely reducing it.
/// </summary>
internal static class QuaternionBranchCheck
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
        Console.WriteLine("QUATERNION DOUBLE COVER: q AND -q ARE THE SAME ROTATION");
        Console.WriteLine("  a plan is built, then node 0 is replaced by the SAME attitude written");
        Console.WriteLine("  antipodally - the exact thing the vehicle did at t+3.93 in flight");
        Console.WriteLine("  20260808-122824, where nothing moved and the defect went to 3548 m");
        Console.WriteLine();

        const int n = 30;
        (Scvx6DofSolver s, double L) = Build(n);
        s.Solve(12);

        double[] plan = (double[])s.ReferenceX.Clone();
        double[] u = (double[])s.ReferenceU.Clone();
        double sigma = s.Sigma;

        double baseline = Defect(s, plan, u, sigma, L, out string baseChan, out int baseNode);
        Console.WriteLine($"  as planned         : {baseline,9:F2} m   worst on {baseChan} at interval {baseNode}");

        // The flip. Physically a no-op: same attitude, opposite representation.
        double[] flipped = (double[])plan.Clone();
        for (int i = 0; i < 4; i++)
            flipped[Dynamics6Dof.IQ + i] = -flipped[Dynamics6Dof.IQ + i];

        double flippedDefect = Defect(s, flipped, u, sigma, L, out string flipChan, out int flipNode);
        Console.WriteLine($"  node 0 sign-flipped: {flippedDefect,9:F2} m   worst on {flipChan} at interval {flipNode}");

        // The fix: walk forward from node 0 putting the plan on ITS branch. Node 0 is
        // the measurement and is never touched.
        double[] aligned = (double[])flipped.Clone();
        int flips = AlignBranch(aligned, n);
        double alignedDefect = Defect(s, aligned, u, sigma, L, out string alChan, out int alNode);
        Console.WriteLine($"  after alignment    : {alignedDefect,9:F2} m   worst on {alChan} at interval {alNode}"
                          + $"   ({flips} of {n - 1} nodes flipped)");
        Console.WriteLine();

        // 1. The flip must actually be catastrophic, or there is nothing to fix and
        //    this check proves nothing.
        bool flipHurts = flippedDefect > 100.0 * Math.Max(baseline, 1e-9);
        // 2. Alignment must RESTORE the original number, not merely improve it. The
        //    two trajectories are physically identical, so any residual difference
        //    would mean the alignment is doing something of its own.
        bool restored = Math.Abs(alignedDefect - baseline) <= 1e-6 * Math.Max(baseline, 1.0);
        // 3. And it must be the attitude channel at interval 0 that breaks - the exact
        //    signature in the flight log. A different channel would mean this
        //    reproduction is of some other fault.
        bool rightSignature = flipChan.StartsWith("q") && flipNode == 0;
        // 4. Inert on a plan that never flipped: the common case must cost nothing.
        double[] untouched = (double[])plan.Clone();
        int idleFlips = AlignBranch(untouched, n);
        bool inert = idleFlips == 0 && SameTrajectory(untouched, plan);

        Console.WriteLine($"    a sign flip is catastrophic, not costly : {(flipHurts ? "yes" : "NO")}"
                          + $" ({flippedDefect / Math.Max(baseline, 1e-9):G3}x)");
        Console.WriteLine($"    it shows up on attitude at interval 0   : {(rightSignature ? "yes" : "NO")}");
        Console.WriteLine($"    alignment RESTORES the original defect   : {(restored ? "yes" : "NO")}");
        Console.WriteLine($"    and is a no-op when nothing flipped      : {(inert ? "yes" : "NO")}");
        Console.WriteLine();

        bool ok = flipHurts && restored && rightSignature && inert;
        Console.WriteLine(ok
            ? "PASS - the double cover causes it, and branch alignment removes it exactly"
            : "FAIL - see above");
        return ok ? 0 : 1;
    }

    /// <summary>
    /// The fix under test, mirroring Ksa6DofGuidance.AlignQuaternionBranch: walk
    /// forward from node 0 and negate any node on the opposite branch from its
    /// predecessor. Node 0 is the measurement and is never rewritten.
    /// </summary>
    private static int AlignBranch(double[] traj, int n)
    {
        int flips = 0;
        for (int k = 1; k < n; k++)
        {
            int q = k * NX + Dynamics6Dof.IQ, p = (k - 1) * NX + Dynamics6Dof.IQ;
            double dot = traj[q] * traj[p] + traj[q + 1] * traj[p + 1]
                       + traj[q + 2] * traj[p + 2] + traj[q + 3] * traj[p + 3];
            if (dot >= 0.0) continue;
            for (int i = 0; i < 4; i++) traj[q + i] = -traj[q + i];
            flips++;
        }
        return flips;
    }

    private static bool SameTrajectory(double[] a, double[] b)
    {
        for (int i = 0; i < a.Length; i++)
            if (a[i] != b[i]) return false;
        return true;
    }

    private static double Defect(Scvx6DofSolver s, double[] x, double[] u, double sigma,
                                 double L, out string channel, out int node)
    {
        (_, double norm) = s.TrueCost(x, u, sigma, out int chan, out node);
        channel = Scvx6DofSolver.ChannelName(chan);
        return norm * L;
    }

    private static (Scvx6DofSolver s, double L) Build(int n)
    {
        var x0 = (double[])EngageState.Clone();
        double alt = x0[2], down = Math.Sqrt(x0[0] * x0[0] + x0[1] * x0[1]);
        var xf = new double[NX];
        xf[2] = 10.0; xf[6] = 1.0;

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
        return (s, L);
    }
}

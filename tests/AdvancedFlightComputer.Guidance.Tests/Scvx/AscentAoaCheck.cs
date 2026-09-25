using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// Does the convex ascent pay for the drag its attitude costs?
///
/// KSA's drag is a six-face box plus an isotropic skin term, blended by |v_body| (see KsaAeroSweep), so the flank faces add drag in proportion to |sin alpha| from the first degree. The planner read its drag table nose-first, the attitude free of charge, and flew 2 to 13 deg off the airflow from max q down, wherever q-alpha let it. A force diagnostic flying 2stage_new on 2026-09-25 measured the drag along the airflow at 4 % above the model at 1 deg, 12 % at 2.9, 17 % at 4.2, 25 % at 6.4, 40 % at 10 and 50 to 60 % at 12.5, a shortfall of up to 5 m/s^2 that left the vehicle 8 km low and 370 m/s slow at staging.
///
/// This plans that vehicle - the problem the game wrote out, lift-off mass corrected - twice: on the old nose-first reading, and on a table of the game's box law sized to those measurements (95 m^2 nose-on, flanks adding 218 m^2 per radian of sin alpha, which gives 4, 12, 17, 25 and 40 % at those angles). The nose-first plan is then charged the drag it left out, as the flight would charge it.
/// </summary>
internal static class AscentAoaCheck
{
    private const double DragArea = 38.12363600646471;
    // The box law, as m^2 of CdA: nose face 0.3 Ax, tail face 1.0 Ax, the flanks 1.2 A_flank (roll-averaged) times |sin alpha|, and the skin term.
    private const double NoseFace = 0.3 * 9.6, TailFace = 1.0 * 9.6, Flanks = 217.6, Skin = 94.9 - NoseFace;

    private static readonly double[] Grid = [0, 6332.8, 12665.6, 18998.4, 25331.2, 31664.1, 37996.9, 44329.7, 50662.5, 56995.3, 63328.1, 69660.9, 75993.8, 82326.6, 88659.4, 94992.2, 101325];
    private static readonly double[] BoosterThrust = [3822083.2, 3694246.4, 3558746.4, 3426148.6, 3294791.8, 3164066.5, 3115768.5, 3080225.8, 3052355.0, 3027763.5, 3005066.8, 2983943.0, 2964147.0, 2945490.5, 2927823.0, 2911023.0, 2894991.8];
    private static readonly double[] UpperThrust = [952331.4, 918940.5, 884915, 851532.4, 818355, 786212.1, 775534.1, 768028.4, 761262.8, 755081.2, 749373.3, 744058.6, 739075.9, 734378.2, 729927.8, 725694.1, 721652.6];

    private static int _fails;

    private static void Check(string name, bool ok, string detail = "")
    {
        Console.WriteLine($"  {(ok ? "PASS" : "FAIL")}  {name,-62} {detail}");
        if (!ok) _fails++;
    }

    /// <summary>The box law's Cd against the table's retrograde-first angle, referenced to the stage drag area, flat in Mach as KSA's is.</summary>
    private static AeroTable Table(bool attitude)
    {
        double[] mach = AeroTable.DefaultMachBreakpoints;
        double[] alpha = AeroTable.DefaultAlphaBreakpointsDeg;
        var cd = new double[mach.Length * alpha.Length];
        for (int j = 0; j < alpha.Length; j++)
        {
            double nose = (180.0 - alpha[j]) * Math.PI / 180.0;   // from nose-first
            double c = Math.Cos(nose), s = Math.Abs(Math.Sin(nose));
            double cdA = attitude
                ? (c >= 0 ? NoseFace * c : -TailFace * c) + Flanks * s + Skin
                : NoseFace + Skin;
            for (int i = 0; i < mach.Length; i++)
                cd[i * alpha.Length + j] = cdA / DragArea;
        }
        return new AeroTable(mach, alpha, cd);
    }

    private static AscentProblem Problem(AeroTable table)
    {
        AscentStage[] stages =
        [
            new(2913395.50317855, 891.5881347656249, 122583.26034570113, 8265.653716798872, DragArea, Grid, BoosterThrust),
            new(952331.4375, 222.73204040527344, 27686.954449899145, 0.0, DragArea, Grid, UpperThrust),
        ];
        return new AscentProblem
        {
            Mu = 3.98601877170e14,
            BodyRadius = 6371000.0,
            Omega = 7.29211585454431e-5,
            R0 = [5575934.356880226, 512805.5320312318, 3040261.6868541157],
            V0 = [-26.85501961544462, 407.5482226419444, 5.739747681757143],
            M0 = 163763.546875,
            Stages = stages,
            Atmosphere = new KsaAscentAtmosphere(new ExponentialAtmosphere(1.225, 101325.0, 8000.0, 1.4), table),
            GroundRadius = 6371000.0,
            TargetRadius = 6571000.0,
            TargetSpeed = 7788.502008228982,
            PlaneNormal = [-0.4044214560127441, -0.40673482055719995, 0.8191520442889918],
            QMax = 35000.0,
            QAlphaMax = 3500.0,
        };
    }

    private sealed record Report(AscentSolution S, double MaxAlpha, double MeanAlpha, double UnmodelledDv, double StagingAltKm, double StagingSpeed);

    /// <summary>
    /// A plan's angle of attack where the air loads it (q over 1 kPa), and the dV the box law's drag takes beyond the drag the plan modelled: the integral of (D_box - D_plan) / m over its burns, trapezoid over the nodes as the collocation integrates.
    /// </summary>
    private static Report Measure(AscentSolution s, AeroTable box, AeroTable planned, double rounding = 0.0)
    {
        const double omega = 7.29211585454431e-5, r = 6371000.0;
        var atm = new ExponentialAtmosphere(1.225, 101325.0, 8000.0, 1.4);
        double max = 0.0, sum = 0.0, dv = 0.0;
        int n = 0;
        var extra = new double[s.Nodes];
        for (int k = 0; k < s.Nodes; k++)
        {
            double px = s.Position[k * 3], py = s.Position[k * 3 + 1], pz = s.Position[k * 3 + 2];
            double ax = s.Velocity[k * 3] + omega * py, ay = s.Velocity[k * 3 + 1] - omega * px, az = s.Velocity[k * 3 + 2];
            double speed = Math.Sqrt(ax * ax + ay * ay + az * az);
            double h = Math.Sqrt(px * px + py * py + pz * pz) - r;
            double q = 0.5 * atm.Density(new Dual(h)).V * speed * speed;
            double a = s.AngleOfAttackDeg[k];
            double mach = speed / atm.SpeedOfSound;
            double cdBox = box.Cd(mach, box.AlphaMaxRad - a * Math.PI / 180.0);
            // The plan read its table at the rounded angle (AscentSettings.AlphaRounding).
            double ar = a * Math.PI / 180.0;
            double s2 = Math.Sin(ar) * Math.Sin(ar);
            double sr = rounding > 0.0 ? s2 / Math.Sqrt(s2 + rounding * rounding) : Math.Sqrt(s2);
            double cdPlan = planned.Cd(mach, planned.AlphaMaxRad - Math.Atan2(sr, Math.Cos(ar)));
            extra[k] = q * DragArea * (cdBox - cdPlan) / s.Mass[k];
            if (q > 1000.0)
            {
                max = Math.Max(max, a);
                sum += a;
                n++;
            }
        }
        for (int k = 0; k < s.Nodes - 1; k++)
            if (s.NodeStage[k] == s.NodeStage[k + 1])
                dv += 0.5 * (extra[k] + extra[k + 1]) * (s.Time[k + 1] - s.Time[k]);
        int stage = Array.LastIndexOf(s.NodeStage, 0);
        double sx = s.Position[stage * 3], sy = s.Position[stage * 3 + 1], sz = s.Position[stage * 3 + 2];
        double vx = s.Velocity[stage * 3] + omega * sy, vy = s.Velocity[stage * 3 + 1] - omega * sx, vz = s.Velocity[stage * 3 + 2];
        return new Report(s, max, n > 0 ? sum / n : 0.0, dv, (Math.Sqrt(sx * sx + sy * sy + sz * sz) - r) / 1000.0, Math.Sqrt(vx * vx + vy * vy + vz * vz));
    }

    /// <summary>The dV the box law's drag beyond its nose-on value takes over a plan's burns.</summary>
    private static double AttitudeDv(AscentSolution s, AeroTable box) => Measure(s, box, Table(attitude: false)).UnmodelledDv;

    private static double MaxAlphaAbove(AscentSolution s, double q)
    {
        double max = 0.0;
        for (int k = 0; k < s.Nodes; k++)
            if (s.DynamicPressure[k] > q) max = Math.Max(max, s.AngleOfAttackDeg[k]);
        return max;
    }

    /// <summary>AFC_AOA_TRACE=1 prints every iteration.</summary>
    private static void Trace(AscentIteration it)
    {
        if (Environment.GetEnvironmentVariable("AFC_AOA_TRACE") == "1")
            Console.WriteLine($"    iter {it.Index}: rho={it.Rho:+0.00;-0.00} {it.Status,-7} tr={it.TrustRegion:F4} sig={string.Join("/", it.SigmaSeconds.Select(x => x.ToString("F1")))} "
                + $"pred={it.Predicted:E2} defect={it.DefectNorm:E2} path={it.PathViolation:E1} term={it.TerminalViolation:E1} J={it.Cost:E8}");
    }

    private static AscentSolution Solve(string what, AeroTable table)
    {
        AscentSolution s = AscentScvx.Solve(Problem(table), Trace);
        Console.WriteLine($"  {what}: {s.Message}, {s.Iterations} iterations, {s.SolveSeconds:F1} s"
            + (s.Nodes > 0 ? $"; {s.FinalMass / 1000.0:F2} t to orbit, burns {string.Join(" / ", s.BurnTime.Select(b => b.ToString("F1")))} s, max q {s.DynamicPressure.Max() / 1000.0:F1} kPa, "
                           + $"defects {s.MaxDefectPosition:F1} m / {s.MaxDefectVelocity:F3} m/s" : ""));
        return s;
    }

    internal static int Run(bool verbose)
    {
        _fails = 0;
        Console.WriteLine("ASCENT SCVX: drag that grows with the angle of attack, on 2stage_new");
        AeroTable box = Table(attitude: true), noseOnly = Table(attitude: false);
        Console.WriteLine($"  box law: Cd x A {box.Cd(0.5, Math.PI) * DragArea:F1} m^2 nose-first, "
            + string.Join(", ", new[] { 1.0, 2.9, 4.2, 6.4, 10.0, 12.5 }.Select(a => $"+{100.0 * (box.Cd(0.5, Math.PI - a * Math.PI / 180.0) / box.Cd(0.5, Math.PI) - 1.0):F0} % at {a:F1} deg")));

        if (Environment.GetEnvironmentVariable("AFC_AOA_SWEEP") == "1")
        {
            AscentSolution nose = AscentScvx.Solve(Problem(noseOnly));
            double leftOut = Measure(nose, box, noseOnly).UnmodelledDv;
            Console.WriteLine($"  sweep: nose-first plan {nose.Message}, {nose.FinalMass / 1000.0:F3} t, leaves out {leftOut:F0} m/s");
            // AFC_AOA_SWEEP=1 plans the vehicle at each rounding in AFC_AOA_E (comma-separated), 0.02 to 0.1 by default.
            foreach (double e in (Environment.GetEnvironmentVariable("AFC_AOA_E") is string es ? es.Split(',').Select(x => double.Parse(x, System.Globalization.CultureInfo.InvariantCulture)).ToArray() : [0.02, 0.03, 0.05, 0.07, 0.1]))
            {
                AscentProblem pe = Problem(box);
                pe.Settings.AlphaRounding = e;
                AscentSolution se = AscentScvx.Solve(pe, Trace);
                Report re = Measure(se, box, box, e);
                Console.WriteLine($"  sweep e={e:F3}: {se.Message,-90} {se.FinalMass / 1000.0:F3} t, leaves out {re.UnmodelledDv,5:F1} m/s, attitude costs {AttitudeDv(se, box),5:F0} m/s, "
                                + $"mean {re.MeanAlpha:F1} deg, max above 5 kPa {MaxAlphaAbove(se, 5000.0):F2} deg; to orbit less what it left out ~ {(se.FinalMass * Math.Exp(-re.UnmodelledDv / 4276.0)) / 1000.0:F3} t");
            }
            return 0;
        }

        AscentSolution before = Solve("nose-first drag (as before)", noseOnly);
        AscentSolution after = Solve("drag at the flown attitude", box);
        Check("the nose-first plan converges", before.Converged, before.Message);
        Check("the attitude-aware plan converges", after.Converged, after.Message);
        if (before.Nodes == 0 || after.Nodes == 0)
            return 1;

        double rounding = new AscentSettings().AlphaRounding;
        Report b = Measure(before, box, noseOnly), a = Measure(after, box, box, rounding);
        Console.WriteLine($"  nose-first plan:     max {b.MaxAlpha:F1} deg, mean {b.MeanAlpha:F1} deg off the airflow where q > 1 kPa; stages at {b.StagingAltKm:F1} km, {b.StagingSpeed:F0} m/s; "
                        + $"the box law's drag takes {b.UnmodelledDv:F0} m/s it never planned for");
        Console.WriteLine($"  attitude-aware plan: max {a.MaxAlpha:F1} deg, mean {a.MeanAlpha:F1} deg; stages at {a.StagingAltKm:F1} km, {a.StagingSpeed:F0} m/s; "
                        + $"{a.UnmodelledDv:F0} m/s unplanned; {after.FinalMass / 1000.0:F2} t to orbit against the nose-first plan's {before.FinalMass / 1000.0:F2} t, which it could not have delivered");
        if (verbose)
            foreach ((string name, AscentSolution sol) in new[] { ("nose-first", before), ("attitude", after) })
                for (int k = 0; k < sol.Nodes; k += 3)
                    Console.WriteLine($"    {name,-10} k={k,3} st={sol.NodeStage[k]} t={sol.Time[k],6:F1} q={sol.DynamicPressure[k] / 1000.0,6:F2} kPa aoa={sol.AngleOfAttackDeg[k],6:F2} deg qa={sol.QAlpha[k],6:F0}");
        Check("the nose-first plan leaves out drag worth over 100 m/s", b.UnmodelledDv > 100.0, $"{b.UnmodelledDv:F0} m/s");
        Check("the attitude-aware plan models nearly all of it", a.UnmodelledDv < 0.1 * b.UnmodelledDv, $"{a.UnmodelledDv:F1} m/s left out by rounding the corner at {rounding:F3}");
        // What flying off the airflow costs each plan under the box law: the drag beyond nose-on, as dV. The attitude-aware plan still pitches off the airflow as q fades - 8 deg at 5 kPa, 16 at 2.7, 20 at 1.2 - buying loft with drag it now pays for knowingly, so it pays about a third of what the nose-first plan does.
        double costBefore = AttitudeDv(before, box), costAfter = AttitudeDv(after, box);
        Check("and pays under half as much for its attitude", costAfter < 0.5 * costBefore, $"{costAfter:F0} m/s against {costBefore:F0} m/s");
        Check("flying within 2 deg of the airflow where q is over 10 kPa", MaxAlphaAbove(after, 10000.0) < 2.0, $"max {MaxAlphaAbove(after, 10000.0):F2} deg (nose-first plan {MaxAlphaAbove(before, 10000.0):F2})");
        Check("dynamically feasible", after.MaxDefectPosition < 35.0 && after.MaxDefectVelocity < 0.8,
              $"{after.MaxDefectPosition:F2} m, {after.MaxDefectVelocity:F4} m/s");
        Check("inserts", Math.Abs(after.TerminalResidual[0]) < 150.0 && Math.Abs(after.TerminalResidual[1]) < 0.2,
              $"{after.TerminalResidual[0]:F1} m, {after.TerminalResidual[1]:F3} m/s");

        // The problem file carries the attitude axis, so a replay plans with it.
        AscentProblem p = Problem(box);
        var round = (KsaAscentAtmosphere)AscentProblemFile.FromJson(AscentProblemFile.ToJson(p)).Atmosphere;
        double at10 = round.DragCoefficient(new Dual(0.5), new Dual(10.0 * Math.PI / 180.0)).V;
        Check("the problem file keeps the attitude axis", Math.Abs(at10 / box.Cd(0.5, Math.PI - 10.0 * Math.PI / 180.0) - 1.0) < 1e-6, $"Cd at 10 deg {at10:F3}");

        Console.WriteLine();
        Console.WriteLine(_fails == 0 ? "ALL PASS" : $"{_fails} FAILED");
        return _fails == 0 ? 0 : 1;
    }
}

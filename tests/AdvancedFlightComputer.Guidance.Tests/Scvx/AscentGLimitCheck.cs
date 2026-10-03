using System.Text.RegularExpressions;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// The thrust acceleration limit in the convex ascent (AscentStage.AccelerationLimit), on launch3dof.py's Saturn V.
///
/// The unlimited plan reaches 4.3 g at S-IC burnout and about 2.1 g at S-II burnout, so a 3 g limit binds on S-IC alone and a 10 g limit on nothing.
///
///   1. 3 g on every stage: the plan holds it, throttles S-IC down toward burnout, burns S-IC longer and delivers less to orbit.
///   2. 10 g on every stage: the plan is the unlimited one, iteration for iteration.
///   3. The problem file keeps the limit, and a file without it reads back unlimited.
///   4. AscentPhasePlan gives the limit only to stages of throttleable liquid engines alone.
///   5. A long S-II on a weak, efficient engine, which a 0.8 g limit stretches past the default 700 s burn-time box, still burns its load and converges.
/// </summary>
internal static class AscentGLimitCheck
{
    private const double G0 = 9.80665;

    private static int _fails;

    private static void Check(string name, bool ok, string detail = "")
    {
        Console.WriteLine($"  {(ok ? "PASS" : "FAIL")}  {name,-58} {detail}");
        if (!ok) _fails++;
    }

    /// <summary>The Saturn V with the same limit on every stage.</summary>
    private static AscentProblem SaturnV(double limit)
    {
        AscentProblem p = AscentCheck.SaturnV();
        AscentStage[] stages = p.Stages.Select(s => new AscentStage(s.Thrust, s.MassFlow, s.PropellantMass, s.JettisonMass, s.DragArea,
                                                                     accelerationLimit: limit)).ToArray();
        return new AscentProblem
        {
            Mu = p.Mu, BodyRadius = p.BodyRadius, Omega = p.Omega, R0 = p.R0, V0 = p.V0, M0 = p.M0, Stages = stages,
            Atmosphere = p.Atmosphere, GroundRadius = p.GroundRadius, TargetRadius = p.TargetRadius, TargetSpeed = p.TargetSpeed,
            TargetRadialRate = p.TargetRadialRate, PlaneNormal = p.PlaneNormal, QMax = p.QMax, QAlphaMax = p.QAlphaMax,
        };
    }

    private static AscentSolution Solve(string what, AscentProblem p, bool verbose)
    {
        AscentSolution s = AscentScvx.Solve(p, it =>
        {
            if (verbose)
                Console.WriteLine($"    iter {it.Index}: rho={it.Rho:+0.00;-0.00} {it.Status,-7} tr={it.TrustRegion:F3} "
                    + $"sig={string.Join("/", it.SigmaSeconds.Select(x => x.ToString("F1")))}s pred={it.Predicted:E1} "
                    + $"defect={it.DefectNorm:E2} path={it.PathViolation:E1} term={it.TerminalViolation:E1} J={it.Cost:E5}");
        });
        Console.WriteLine($"  {what}: {s.Message}, {s.Iterations} iterations ({s.Accepted} accepted), {s.SolveSeconds:F1} s"
            + (s.Nodes > 0
                ? $"; {s.FinalMass / 1000.0:F2} t to orbit, burns {string.Join(" / ", s.BurnTime.Select(b => b.ToString("F1")))} s, "
                  + $"peak {PeakG(s, -1):F3} g, max q {s.DynamicPressure.Max() / 1000.0:F1} kPa, max q-alpha {s.QAlpha.Max():F0}"
                : ""));
        return s;
    }

    /// <summary>The highest thrust acceleration over a stage, or over the whole plan for -1, g.</summary>
    private static double PeakG(AscentSolution s, int stage)
    {
        double peak = 0.0;
        for (int k = 0; k < s.Nodes; k++)
            if (stage < 0 || s.NodeStage[k] == stage)
                peak = Math.Max(peak, s.Thrust[k] / s.Mass[k] / G0);
        return peak;
    }

    private static double ThrottleAt(AscentSolution s, int k)
        => Math.Sqrt(s.Throttle[k * 3] * s.Throttle[k * 3] + s.Throttle[k * 3 + 1] * s.Throttle[k * 3 + 1] + s.Throttle[k * 3 + 2] * s.Throttle[k * 3 + 2]);

    private static void FeasibleAndInserts(AscentSolution s)
    {
        Check("insertion radius", Math.Abs(s.TerminalResidual[0]) < 150.0, $"{s.TerminalResidual[0]:F1} m");
        Check("insertion speed", Math.Abs(s.TerminalResidual[1]) < 0.2, $"{s.TerminalResidual[1]:F3} m/s");
        Check("in the plane", Math.Abs(s.TerminalResidual[3]) < 150.0 && Math.Abs(s.TerminalResidual[4]) < 0.2,
              $"{s.TerminalResidual[3]:F1} m, {s.TerminalResidual[4]:F3} m/s");
        Check("dynamically feasible", s.MaxDefectPosition < 35.0 && s.MaxDefectVelocity < 0.8,
              $"{s.MaxDefectPosition:F2} m, {s.MaxDefectVelocity:F4} m/s");
    }

    internal static int Run(bool verbose)
    {
        _fails = 0;
        Console.WriteLine("ASCENT SCVX WITH A THRUST ACCELERATION LIMIT: launch3dof.py's Saturn V");
        Binding(verbose);
        NotBinding(verbose);
        RoundTrip();
        PhasePlan();
        LongStage(verbose);
        Console.WriteLine();
        Console.WriteLine(_fails == 0 ? "ALL PASS" : $"{_fails} FAILED");
        return _fails == 0 ? 0 : 1;
    }

    // ---- 1. 3 g binds on S-IC.
    private static void Binding(bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine("3 g on every stage: S-IC reaches it before burnout");
        const double limitG = 3.0;
        AscentProblem p = SaturnV(limitG * G0);
        AscentSolution s = Solve("plan", p, verbose);
        Check("converged", s.Converged, s.Message);
        if (s.Nodes == 0) return;
        double peak = PeakG(s, -1);
        Check("the plan never passes the limit", peak <= limitG * (1.0 + p.Settings.PathTolerance), $"peak {peak:F4} g");
        Check("S-IC rides the limit at the back-off", PeakG(s, 0) >= limitG * p.Settings.PathBackoff * 0.995, $"{PeakG(s, 0):F4} g");
        int sicEnd = Array.LastIndexOf(s.NodeStage, 0);
        double lastThrottle = ThrottleAt(s, sicEnd);
        Check("S-IC is throttled down toward burnout", lastThrottle < 0.9, $"{lastThrottle:P1} at burnout");
        Check("S-IC burns longer than at full thrust", s.BurnTime[0] > 177.2 + 5.0, $"{s.BurnTime[0]:F1} s against 177.2 s");
        Check("the limit costs mass to orbit", s.FinalMass <= 148.5e3 + 0.5e3 && s.FinalMass > 140e3, $"{s.FinalMass / 1000.0:F2} t against 148.5 t");
        FeasibleAndInserts(s);
    }

    // ---- 2. 10 g binds nowhere.
    private static void NotBinding(bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine("10 g on every stage: the unlimited plan");
        AscentSolution s = Solve("plan", SaturnV(10.0 * G0), verbose);
        Check("converged", s.Converged, s.Message);
        if (s.Nodes == 0) return;
        Check("mass to orbit", Math.Abs(s.FinalMass - 148.5e3) < 0.5e3, $"{s.FinalMass / 1000.0:F2} t (script 148.5)");
        Check("S-IC burns its load", Math.Abs(s.BurnTime[0] - 177.2) < 1.0, $"{s.BurnTime[0]:F1} s (script 177.2)");
        Check("S-II burns its load", Math.Abs(s.BurnTime[1] - 371.8) < 1.5, $"{s.BurnTime[1]:F1} s (script 371.8)");
        Check("S-IVB burn", Math.Abs(s.BurnTime[2] - 215.7) < 3.0, $"{s.BurnTime[2]:F1} s (script 215.7)");
        FeasibleAndInserts(s);
        // A limit no stage can reach is dropped before the subproblem is built, so the solve is the unlimited one to the last digit.
        AscentSolution free = Solve("unlimited", AscentCheck.SaturnV(), verbose: false);
        Check("the solve is the unlimited one", free.Iterations == s.Iterations && free.FinalMass == s.FinalMass,
              $"{s.Iterations} against {free.Iterations} iterations, {s.FinalMass - free.FinalMass:E2} kg");
    }

    // ---- 3. The dump.
    private static void RoundTrip()
    {
        Console.WriteLine();
        Console.WriteLine("the problem file carries the limit");
        AscentProblem p = SaturnV(3.0 * G0);
        string json = AscentProblemFile.ToJson(p);
        AscentProblem q = AscentProblemFile.FromJson(json);
        Check("the limit survives", q.Stages.Zip(p.Stages).All(pair => pair.First.AccelerationLimit == pair.Second.AccelerationLimit),
              $"{q.Stages[0].AccelerationLimit:F5} m/s^2");
        string older = Regex.Replace(json, @"\s*""AccelerationLimit"":\s*[^,\r\n]+,?", "");
        AscentProblem r = AscentProblemFile.FromJson(older);
        Check("a file without it reads back unlimited", !older.Contains("AccelerationLimit") && r.Stages.All(s => double.IsPositiveInfinity(s.AccelerationLimit)));
        AscentProblem free = AscentProblemFile.FromJson(AscentProblemFile.ToJson(AscentCheck.SaturnV()));
        Check("an unlimited problem stays unlimited", free.Stages.All(s => double.IsPositiveInfinity(s.AccelerationLimit)));

        bool solidRefused = false, nanRefused = false;
        var burn = new AscentSolidBurn([0.0, 10.0], [100.0, 100.0], [0.0], [250e3, 250e3]);
        try { _ = new AscentStage(0.0, 0.0, 1000.0, 0.0, 10.0, solid: burn, fixedBurnTime: 10.0, accelerationLimit: 30.0); }
        catch (ArgumentException) { solidRefused = true; }
        try { _ = new AscentStage(1e6, 300.0, 1000.0, 0.0, 10.0, accelerationLimit: double.NaN); }
        catch (ArgumentException) { nanRefused = true; }
        Check("a stage with solids cannot take a limit", solidRefused);
        Check("a limit that is not a positive number is refused", nanRefused);
    }

    // ---- 4. Which stages AscentPhasePlan limits.
    private static void PhasePlan()
    {
        Console.WriteLine();
        Console.WriteLine("AscentPhasePlan: a core held at full thrust beside a booster, the booster alone, a liquid upper stage, an engine that cannot throttle");
        const double limit = 10.0;
        const double coreThrust = 2896e3, coreFlow = 891.6, boosterFlow = 1000.0, boosterThrust = 2.6e6;
        const double upperThrust = 952.3e3, upperFlow = 222.7, upperLoad = 55.4e3;
        var motor = new AscentSolidMotor
        {
            Name = "booster",
            Time = [0.0, 100.0],
            MassFlow = [boosterFlow, boosterFlow],
            Thrust = [boosterThrust, boosterThrust],
        };
        double m0 = 400e3;
        double end0 = m0 - coreFlow * 60.0 - boosterFlow * 60.0, end1 = end0 - boosterFlow * 40.0;
        double start2 = end1 - 20e3, end2 = start2 - upperLoad, start3 = end2 - 5e3;
        var phases = new[]
        {
            new AscentPhase { StartMass = m0, EndMass = end0, Duration = 60.0, Solids = [0],
                              LiquidThrust = coreThrust, LiquidMassFlow = coreFlow, LiquidEngines = [1] },
            new AscentPhase { StartMass = end0, EndMass = end1, Duration = 40.0, Solids = [0] },
            new AscentPhase { StartMass = start2, EndMass = end2, Duration = upperLoad / upperFlow,
                              LiquidThrust = upperThrust, LiquidMassFlow = upperFlow, LiquidEngines = [2] },
            new AscentPhase { StartMass = start3, EndMass = start3 - 10e3, Duration = 10e3 / 50.0,
                              LiquidThrust = 2e6, LiquidMassFlow = 50.0, LiquidEngines = [3], Throttleable = false },
        };
        AscentPhasePlan.Result plan = AscentPhasePlan.Build(phases, [motor], null, m0, 10.0, 0.9, 0.99, accelerationLimit: limit);
        foreach (string d in plan.Describe) Console.WriteLine("    " + d);
        foreach (string n in plan.Notes) Console.WriteLine("    note: " + n);
        Check("four stages, the core held", plan.Stages.Length == 4 && !plan.CoreThrottledDown, $"{plan.Stages.Length} stages");
        if (plan.Stages.Length != 4) return;
        Check("the core held beside the booster has no limit", double.IsPositiveInfinity(plan.Stages[0].AccelerationLimit));
        Check("the booster alone has none", double.IsPositiveInfinity(plan.Stages[1].AccelerationLimit));
        Check("the throttleable upper stage has it", plan.Stages[2].AccelerationLimit == limit);
        Check("the engine that cannot throttle has none", double.IsPositiveInfinity(plan.Stages[3].AccelerationLimit));
        bool Noted(int stage) => plan.Notes.Any(n => n.StartsWith($"stage {stage} cannot be throttled"));
        Check("a note names each unlimited stage whose full thrust passes the limit, and only those",
              Noted(1) && Noted(2) && !Noted(3) && Noted(4),
              string.Join(" | ", plan.Notes.Where(n => n.Contains("cannot be throttled"))));
        Check("a stack cut at the upper stage keeps its limit", AscentPhasePlan.Truncate(plan.Stages, 3)[2].AccelerationLimit == limit);
    }

    // ---- 5. A limit that stretches a long stage past the burn-time box.
    private static void LongStage(bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine("a long S-II on a weak engine, held at 0.8 g: the limited burn is longer than the default 700 s box");
        AscentProblem baseline = AscentCheck.SaturnV();
        const double s2Thrust = 3.5e6, s2Isp = 480.0, limitG = 0.8;
        double s2Flow = s2Thrust / (s2Isp * G0);
        AscentStage s1 = baseline.Stages[0], s2 = baseline.Stages[1], s3 = baseline.Stages[2];
        AscentStage[] stages =
        [
            s1,
            new AscentStage(s2Thrust, s2Flow, s2.PropellantMass, s2.JettisonMass, s2.DragArea, accelerationLimit: limitG * G0),
            s3,
        ];
        var p = new AscentProblem
        {
            Mu = baseline.Mu, BodyRadius = baseline.BodyRadius, Omega = baseline.Omega, R0 = baseline.R0, V0 = baseline.V0, M0 = baseline.M0,
            Stages = stages, Atmosphere = baseline.Atmosphere, GroundRadius = baseline.GroundRadius, TargetRadius = baseline.TargetRadius,
            TargetSpeed = baseline.TargetSpeed, TargetRadialRate = baseline.TargetRadialRate, PlaneNormal = baseline.PlaneNormal,
            QMax = baseline.QMax, QAlphaMax = baseline.QAlphaMax,
        };
        double fullBurn = s2.PropellantMass / s2Flow;
        AscentSolution s = Solve("plan", p, verbose);
        Check("converged", s.Converged, s.Message);
        if (s.Nodes == 0) return;
        // The limited burn is about 708 s against the 605 s full-throttle burn. The margin over the box is wider than the trapezoid's error on the falling throttle, so a box that ignores the limit fails here.
        Check("S-II burns past the default box", s.BurnTime[1] > p.Settings.SigmaMaxSeconds + 5.0,
              $"{s.BurnTime[1]:F1} s against {fullBurn:F1} s at full thrust");
        double peak = PeakG(s, 1);
        Check("S-II holds the limit", peak <= limitG * (1.0 + p.Settings.PathTolerance), $"peak {peak:F4} g");
        FeasibleAndInserts(s);
    }
}

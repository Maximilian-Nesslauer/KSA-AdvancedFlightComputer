using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// --3dof-mpc: the glide-and-burn guidance flown closed loop against a vehicle that is not the model.
///
/// The truth is the same KSA point-mass physics with its drag, lift and thrust scaled off nominal and an attitude loop that lags its command and turns at a finite rate; the guidance plans with the nominal model, corrected by the aero-scale estimator through the glide. The simulation steps at 50 Hz and the guidance re-solves at 10 Hz from the true state, through the free-ignition glide, the lock and the swap to the burn-only problem, and the burn, to the 20 m aim point where the terminal hover would take over.
///
/// Measured per case: miss and speeds at the aim point, propellant, how far the ignition time moved between the first plan and the lock, refused cycles, solve time, and how far the burn-only plan's opening state jumped from where the two-phase plan put it.
/// </summary>
internal static class ThreeDofMpcCheck
{
    private const int NX = PointMass3Dof.NX;
    private const double SimStep = 0.02;
    private const double GuidanceStep = 0.1;
    private const double ColdDelay = 1.0;     // the spread cold solve, flown on the PID glide meanwhile

    private static readonly double[] X0 = [-9000, 1500, 32000, 300, -30, -950, ThreeDofScenario.WetMass];
    private static readonly double[] Xf = [0, 0, 20, 0, 0, -2];

    private sealed record Case(string Name, double Drag = 1.0, double Lift = 1.0, double Thrust = 1.0, double LagS = 0.3,
                               double[]? Start = null);

    public static int Run(bool verbose)
    {
        Case[] cases =
        [
            new("nominal"),
            new("drag +25 %", Drag: 1.25),
            new("drag -20 %", Drag: 0.8),
            new("lift -30 %", Lift: 0.7),
            new("thrust -3 %", Thrust: 0.97),
            new("attitude lag 1 s", LagS: 1.0),
            new("combined", Drag: 1.2, Lift: 0.8, Thrust: 0.97, LagS: 0.8),
            new("2 km cross-range", Start: [-9000, 3500, 32000, 300, -30, -950, ThreeDofScenario.WetMass]),
        ];

        string[] args = Environment.GetCommandLineArgs();
        string? Arg(string name)
        {
            int at = Array.IndexOf(args, name);
            return at >= 0 && at + 1 < args.Length ? args[at + 1] : null;
        }
        string? only = Arg("--case");
        // --scaling fixed | lock | every | all: when the solver's units are resized. "all" flies every case under each and compares them.
        ScalingMode[] modes = Arg("--scaling")?.ToLowerInvariant() switch
        {
            "fixed" => [ScalingMode.Fixed],
            "lock" => [ScalingMode.AtLock],
            "all" => [ScalingMode.Fixed, ScalingMode.AtLock, ScalingMode.EverySolve],
            _ => [ScalingMode.EverySolve],
        };

        bool all = true;
        var summary = new List<string>();
        foreach (ScalingMode mode in modes)
        {
            Console.WriteLine($"scaling: {mode}");
            Console.WriteLine($"{"case",-18} {"miss m",7} {"vh m/s",7} {"vz m/s",7} {"prop kg",8} {"ign drift s",11} {"refused",7} {"admm/cyc",8} {"solve p50/max ms",17} {"swap jump m",11}  result");
            var results = new List<Result>();
            foreach (Case c in cases)
                if (only == null || c.Name.StartsWith(only, StringComparison.OrdinalIgnoreCase))
                    results.Add(Fly(c, mode, verbose));
            bool modePass = results.All(r => r.Pass);
            all &= modePass;
            var flown = results.Where(r => r.Flown).ToList();
            summary.Add($"{mode,-11} {results.Count(r => r.Pass),2}/{results.Count} pass   refused {results.Sum(r => r.Refused),4}   " +
                        $"admm/cycle {(flown.Count > 0 ? flown.Average(r => r.AdmmPerCycle) : double.NaN),6:F0}   " +
                        $"solve p50 {(flown.Count > 0 ? flown.Average(r => r.P50) : double.NaN),5:F1} ms, worst {(flown.Count > 0 ? flown.Max(r => r.Max) : double.NaN),5:F0} ms   " +
                        $"|vz err| mean {(flown.Count > 0 ? flown.Average(r => Math.Abs(r.Vz - Xf[5])) : double.NaN),5:F2} m/s   " +
                        $"miss worst {(flown.Count > 0 ? flown.Max(r => r.Miss) : double.NaN),5:F1} m");
            Console.WriteLine();
        }
        if (modes.Length > 1)
        {
            Console.WriteLine("summary:");
            foreach (string line in summary)
                Console.WriteLine("  " + line);
            Console.WriteLine();
        }
        Console.WriteLine(all ? "PASS - closed-loop glide and burn lands on the aim point in every case" : "FAIL");
        return all ? 0 : 1;
    }

    private sealed record Result(bool Pass, bool Flown, double Miss, double Vz, int Refused, double AdmmPerCycle, double P50, double Max);

    private static Result Fly(Case c, ScalingMode mode, bool verbose)
    {
        KsaPointMassModel nominal = ThreeDofScenario.Model();
        KsaPointMassModel truth = nominal with
        {
            DragScale = c.Drag,
            LiftScale = c.Lift,
            VacuumThrust = nominal.VacuumThrust * c.Thrust,
        };
        Scvx3DofConfig glide = ThreeDofScenario.Config(anchor: true);
        Scvx3DofConfig burn = ThreeDofScenario.Config(nodes: 20, glideIntervals: 0, anchor: true);
        // No wall-clock deadlines offline: they make the iteration count, and so the flight, depend on the machine's speed. The iteration caps still bound every cycle.
        // Anchored on the last command, tight, as the game flies it.
        var guidance = new Guidance3Dof(glide, burn, nominal, new Guidance3Dof.Settings
        {
            CycleBudgetMs = 0, RecoveryBudgetMs = 0, Scaling = mode,
            AnchorSeconds = 0.3, AnchorFloor = 1.0 * Math.PI / 180.0,
        });
        var estimator = new AeroScaleEstimator();

        double[] x = (double[])(c.Start ?? X0).Clone();
        double[] att = Retro(x);
        double t = 0.0;

        if (!guidance.BeginCold(x, Xf, t, new Seed3Dof.Options()))
            return Report(c, "no seed");
        while (guidance.Phase == Phase3Dof.Converging)
            guidance.StepCold(5);
        if (guidance.Phase != Phase3Dof.Glide)
            return Report(c, guidance.Error);
        double firstIgnition = double.NaN;   // from the first warm plan: the cold one is a second stale by then

        // The PID glide flies engine first while the cold solve runs.
        for (double s = 0; s < ColdDelay - 1e-9; s += SimStep)
        {
            x = Step(truth, x, att, 0.0);
            t += SimStep;
        }

        var solveMs = new List<double>();
        long admm = 0;
        int refused = 0;
        double lockIgnition = double.NaN, swapJump = double.NaN;
        double nextGuidance = t;
        double[] cmd = att;
        double throttle = 0.0;
        bool engine = false;
        Plan3Dof? lastGlidePlan = guidance.Published;
        while (t < 400)
        {
            if (t >= nextGuidance - 1e-9)
            {
                nextGuidance += GuidanceStep;
                // What an accelerometer would read: the truth's aerodynamics and thrust, at the attitude flown and the throttle commanded.
                double[] measured = AeroScaleEstimator.NonGravitational(truth, x, att, throttle);
                estimator.Update(nominal, x, att, throttle, measured, GuidanceStep);
                guidance.Model = estimator.Apply(nominal);
                Phase3Dof before = guidance.Phase;
                bool ok = guidance.Update(x, cmd, t);
                solveMs.Add(guidance.LastSolveMs);
                admm += guidance.LastAdmmIterations;
                if (!ok) refused++;
                if (ok && double.IsNaN(firstIgnition)) firstIgnition = guidance.Published!.IgnitionTime;
                if (before == Phase3Dof.Glide && guidance.Phase == Phase3Dof.Locked)
                {
                    lockIgnition = guidance.CommittedIgnition;
                    // Where the last two-phase plan put the vehicle at ignition, against where the burn-only plan starts it.
                    double[] xi = new double[NX], ui = new double[4];
                    lastGlidePlan!.Sample(lockIgnition, xi, ui);
                    Plan3Dof bp = guidance.Published!;
                    swapJump = Math.Sqrt(Sq(bp.X[0] - xi[0]) + Sq(bp.X[1] - xi[1]) + Sq(bp.X[2] - xi[2]));
                }
                if (guidance.Phase == Phase3Dof.Glide) lastGlidePlan = guidance.Published;
                if (verbose && (solveMs.Count % 10 == 0 || before != guidance.Phase))
                    Console.WriteLine($"    t {t,6:F1}  {guidance.Phase,-9} alt {x[2],7:F0} v {Speed(x),5:F0}  ign {guidance.Published?.IgnitionTime ?? double.NaN,6:F2}  " +
                                      $"tb {guidance.Published?.SigmaBurn ?? double.NaN,5:F1}  kd {estimator.DragScale:F2} kl {estimator.LiftScale:F2} kt {estimator.ThrustScale:F2}  " +
                                      $"{guidance.LastIterations} it {guidance.LastSolveMs,4:F0} ms  {guidance.Error}");
            }

            Plan3Dof plan = guidance.Published!;
            double[] xp = new double[NX], up = new double[4];
            plan.Sample(t, xp, up);
            engine = t >= plan.IgnitionTime;
            cmd = [up[0], up[1], up[2]];
            throttle = engine ? up[3] : 0.0;

            att = Turn(att, cmd, c.LagS, glide.AttitudeRateMax * 1.2);
            x = Step(truth, x, att, throttle);
            t += SimStep;

            if (x[2] <= Xf[2] || x[6] <= ThreeDofScenario.DryMass || (engine && t > plan.EndTime + 3.0))
                break;
        }

        double miss = Math.Sqrt(Sq(x[0] - Xf[0]) + Sq(x[1] - Xf[1]));
        double vh = Math.Sqrt(x[3] * x[3] + x[4] * x[4]);
        double vz = x[5];
        double prop = (c.Start ?? X0)[6] - x[6];
        double drift = lockIgnition - firstIgnition;
        solveMs.Sort();
        double p50 = solveMs.Count > 0 ? solveMs[solveMs.Count / 2] : double.NaN;
        double max = solveMs.Count > 0 ? solveMs[^1] : double.NaN;
        bool landed = x[2] <= Xf[2] + 1.0;
        bool pass = landed && miss < 15.0 && vh < 2.0 && Math.Abs(vz - Xf[5]) < 2.0 && x[6] > ThreeDofScenario.DryMass;
        double admmPerCycle = solveMs.Count > 0 ? (double)admm / solveMs.Count : double.NaN;
        Console.WriteLine($"{c.Name,-18} {miss,7:F1} {vh,7:F2} {vz,7:F2} {prop,8:F0} {drift,11:F2} {refused,7} {admmPerCycle,8:F0} {p50,8:F0}/{max,-8:F0} {swapJump,11:F1}  " +
                          $"{(pass ? "ok" : landed ? "OFF TARGET" : $"FAILED at {x[2]:F0} m: {guidance.Error}")}");
        return new Result(pass, true, miss, vz, refused, admmPerCycle, p50, max);
    }

    private static Result Report(Case c, string why)
    {
        Console.WriteLine($"{c.Name,-18} FAILED before flight: {why}");
        return new Result(false, false, double.NaN, double.NaN, 0, double.NaN, double.NaN, double.NaN);
    }

    /// <summary>One sim step with the control held: RK4 on the truth model.</summary>
    private static double[] Step(KsaPointMassModel model, double[] x, double[] b, double throttle)
    {
        double[] u = [b[0], b[1], b[2], throttle];
        double h = SimStep;
        double[] k1 = new double[NX], k2 = new double[NX], k3 = new double[NX], k4 = new double[NX], tmp = new double[NX];
        PointMass3Dof.Eval(model, x, u, k1);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k1[i];
        PointMass3Dof.Eval(model, tmp, u, k2);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k2[i];
        PointMass3Dof.Eval(model, tmp, u, k3);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + h * k3[i];
        PointMass3Dof.Eval(model, tmp, u, k4);
        var o = new double[NX];
        for (int i = 0; i < NX; i++) o[i] = x[i] + h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
        return o;
    }

    /// <summary>An attitude loop: a first-order lag towards the command, capped at a turn rate.</summary>
    private static double[] Turn(double[] att, double[] cmd, double lagS, double rate)
    {
        double cos = Math.Clamp(att[0] * cmd[0] + att[1] * cmd[1] + att[2] * cmd[2], -1, 1);
        double angle = Math.Acos(cos);
        double step = Math.Min(angle * (1 - Math.Exp(-SimStep / Math.Max(lagS, 1e-3))), rate * SimStep);
        return Guidance3Dof.Slew(att, cmd, step);
    }

    private static double[] Retro(double[] x)
    {
        double s = Speed(x);
        return [-x[3] / s, -x[4] / s, -x[5] / s];
    }

    private static double Speed(double[] x) => Math.Sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
    private static double Sq(double v) => v * v;
}

using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// --3dof-mpc: the glide-and-burn guidance flown closed loop against a vehicle that is not the model.
///
/// The truth is the same KSA point-mass physics with its drag, lift and thrust scaled off nominal, an attitude loop that lags its command and turns at a finite rate, and an engine that may light late; the guidance plans with the nominal model, corrected by the aero-scale estimator through the glide. The simulation steps at 50 Hz and the guidance re-solves at 10 Hz from the true state, through the free-ignition glide, the lock and the swap to the burn-only problem, and the burn, to touchdown at zero/zero, cutting the engine where a craft that cannot hover stops short, as the game does. The burn is flown as the game flies it, through BurnTracker3Dof, at every step.
///
/// Measured per case: miss and speeds at touchdown, the height of any stop-short cut, propellant, how far the ignition time moved between the first plan and the lock, refused cycles, solve time, and how far the burn-only plan's opening state jumped from where the two-phase plan put it.
/// </summary>
internal static class ThreeDofMpcCheck
{
    private const int NX = PointMass3Dof.NX;
    private const double SimStep = 0.02;
    private const double GuidanceStep = 0.1;
    private const double ColdDelay = 1.0;     // the spread cold solve, flown on the PID glide meanwhile

    private static readonly double[] X0 = [-9000, 1500, 32000, 300, -30, -950, ThreeDofScenario.WetMass];
    // The end state: zero/zero, on the ground at rest (the scenario has no vehicle height, so z is the legs). --aim-height and --aim-sink end the burn above the ground instead, as the game did before (20 m, 2 m/s).
    private static double[] Xf = [0, 0, 0, 0, 0, 0];

    /// <param name="IgnitionDelay">Seconds the engine lights after the guidance commands it, the guidance not told.</param>
    private sealed record Case(string Name, double Drag = 1.0, double Lift = 1.0, double Thrust = 1.0, double LagS = 0.3,
                               double[]? Start = null, double IgnitionDelay = 0.0);

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
            new("late ignition 1 s", IgnitionDelay: 1.0),
            new("thrust -8 %", Thrust: 0.92),
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
        // --track off flies the plan's own throttle and attitude in the burn, as the game did before the tracker; --glide-ceiling and --burn-ceiling override the guidance's plan throttle ceilings, and --track-kp, --track-kd, --track-tilt (deg) the tracker's.
        double Num(string name, double fallback)
            => double.TryParse(Arg(name), System.Globalization.CultureInfo.InvariantCulture, out double v) ? v : fallback;
        var defaults = new Guidance3Dof.Settings();
        var tracking = new BurnTracker3Dof();
        tracking = tracking with
        {
            Kp = Num("--track-kp", tracking.Kp),
            Kd = Num("--track-kd", tracking.Kd),
            MaxTiltRad = Num("--track-tilt", tracking.MaxTiltRad * 180.0 / Math.PI) * Math.PI / 180.0,
        };
        // --accel-g: the burn's thrust acceleration limit, as the game's default of 3 g unless given; 0 for none. The glide plan gets it over Settings.AccelHeadroom, as in the game.
        // --turn-accel: the plan's turn-acceleration limit, deg/s^2, 0 for none; --plant-accel the vehicle's, which the truth's attitude loop changes its turn rate no faster than, 0 for instantly as before.
        var flight = new Flight(
            Arg("--track")?.ToLowerInvariant() == "off" ? null : tracking,
            Num("--glide-ceiling", defaults.GlideThrottleCeiling),
            Num("--burn-ceiling", defaults.BurnThrottleCeiling),
            Num("--accel-g", 3.0) * 9.80665,
            Num("--engine-scale", 1.0),
            Num("--turn-accel", 0.0) * Math.PI / 180.0,
            Num("--plant-accel", 0.0) * Math.PI / 180.0,
            Num("--turn-accel-burn", Num("--turn-accel", 0.0)) * Math.PI / 180.0,
            Arg("--turn-anchor")?.ToLowerInvariant() != "off",
            Num("--vc-velocity", 0.0));
        Xf = [0, 0, Num("--aim-height", 0.0), 0, 0, -Num("--aim-sink", 0.0)];
        Console.WriteLine($"burn tracking {(flight.Tracker is { } k ? $"Kp {k.Kp:F2} Kd {k.Kd:F2} tilt {k.MaxTiltRad * 180.0 / Math.PI:F0} deg" : "off")}, " +
                          $"plan throttle ceiling {flight.GlideCeiling:F2} glide, {flight.BurnCeiling:F2} burn, accel limit {(flight.AccelMax > 0.0 ? $"{flight.AccelMax / 9.80665:F1} g" : "off")}, engines x{flight.EngineScale:F1}, end state {Xf[2]:F0} m at {-Xf[5]:F1} m/s, " +
                          $"turn accel plan {(flight.TurnAccel > 0.0 ? $"{flight.TurnAccel * 180.0 / Math.PI:F1}" : "off")} vehicle {(flight.PlantAccel > 0.0 ? $"{flight.PlantAccel * 180.0 / Math.PI:F1} deg/s2" : "instant")}");

        bool all = true;
        var summary = new List<string>();
        foreach (ScalingMode mode in modes)
        {
            Console.WriteLine($"scaling: {mode}");
            Console.WriteLine($"{"case",-18} {"miss m",7} {"vh m/s",7} {"vz m/s",7} {"prop kg",8} {"ign drift s",11} {"refused",7} {"admm/cyc",8} {"solve p50/max ms",17} {"swap jump m",11} {"cut m",6} {"peak g",6} {"burn s",6}  result");
            var results = new List<Result>();
            foreach (Case c in cases)
                if (only == null || c.Name.StartsWith(only, StringComparison.OrdinalIgnoreCase))
                    results.Add(Fly(c, mode, flight, verbose));
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

    /// <summary>How the burn is flown: the tracker, or the plan's own command when null, the plan's throttle ceilings, and its thrust acceleration limit (m/s^2, 0 for none).</summary>
    /// <param name="EngineScale">Multiplies the scenario's engines - thrust, mass flow and nozzle - for a booster with more thrust to weight, as the game's have (about 6 at landing against the scenario's 3).</param>
    /// <param name="TurnAccel">The plan's turn-acceleration limit, rad/s^2, 0 for none; see Scvx3DofConfig.AttitudeAccelMax.</param>
    /// <param name="PlantAccel">The truth's: its attitude loop changes its turn rate no faster than this, rad/s^2, 0 for instantly.</param>
    /// <param name="TurnAccelBurn">The burn-only plan's limit, --turn-accel-burn, the glide's unless given.</param>
    /// <param name="TurnAnchor">Whether each plan starts from the command's turn rate (--turn-anchor off for not).</param>
    /// <param name="VcVelocity">The scale velocity's virtual control is priced on, m/s (--vc-velocity), 0 for XScale as the scenario always has; see Scvx3DofConfig.VirtualControlScale.</param>
    private sealed record Flight(BurnTracker3Dof? Tracker, double GlideCeiling, double BurnCeiling, double AccelMax, double EngineScale,
                                 double TurnAccel = 0.0, double PlantAccel = 0.0, double TurnAccelBurn = 0.0, bool TurnAnchor = true,
                                 double VcVelocity = 0.0);

    private static Result Fly(Case c, ScalingMode mode, Flight flight, bool verbose)
    {
        KsaPointMassModel scenario = ThreeDofScenario.Model();
        KsaPointMassModel nominal = scenario with
        {
            VacuumThrust = scenario.VacuumThrust * flight.EngineScale,
            MassFlow = scenario.MassFlow * flight.EngineScale,
            ExitArea = scenario.ExitArea * flight.EngineScale,
        };
        KsaPointMassModel truth = nominal with
        {
            DragScale = c.Drag,
            LiftScale = c.Lift,
            VacuumThrust = nominal.VacuumThrust * c.Thrust,
        };
        Scvx3DofConfig glide = ThreeDofScenario.Config(anchor: true, accelMax: new Guidance3Dof.Settings().GlideAccelMax(flight.AccelMax), turnAccel: flight.TurnAccel, vcVelocity: flight.VcVelocity);
        Scvx3DofConfig burn = ThreeDofScenario.Config(nodes: 20, glideIntervals: 0, anchor: true, accelMax: flight.AccelMax, turnAccel: flight.TurnAccelBurn, vcVelocity: flight.VcVelocity);
        // No wall-clock deadlines offline: they make the iteration count, and so the flight, depend on the machine's speed. The iteration caps still bound every cycle.
        // Anchored on the last command, tight, as the game flies it.
        var guidance = new Guidance3Dof(glide, burn, nominal, new Guidance3Dof.Settings
        {
            CycleBudgetMs = 0, RecoveryBudgetMs = 0, Scaling = mode,
            AnchorSeconds = 0.3, AnchorFloor = 1.0 * Math.PI / 180.0,
            GlideThrottleCeiling = flight.GlideCeiling, BurnThrottleCeiling = flight.BurnCeiling,
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
        double[] cmd = att, cmdRate = new double[3], prevCmd = att;
        double[] turn = new double[3];   // the truth's axis rate, for an attitude loop with a turn-acceleration limit
        double throttle = 0.0;
        bool engine = false, lit = false, cut = false;
        double commanded = double.NaN, cutHeight = double.NaN, peakAccel = 0.0;
        Plan3Dof? lastGlidePlan = guidance.Published;
        while (t < 400)
        {
            if (!cut && t >= nextGuidance - 1e-9)
            {
                nextGuidance += GuidanceStep;
                // What an accelerometer would read: the truth's aerodynamics and thrust, at the attitude flown and the throttle commanded. An engine lit late is reported as off until it lights, so a delayed ignition tests the burn's recovery rather than the thrust estimate.
                double flown = lit ? throttle : 0.0;
                double[] measured = AeroScaleEstimator.NonGravitational(truth, x, att, flown);
                estimator.Update(nominal, x, att, flown, measured, GuidanceStep);
                guidance.Model = estimator.Apply(nominal);
                Phase3Dof before = guidance.Phase;
                // The command's own turn since the last cycle, which the plan continues from, as the game passes it.
                for (int j = 0; j < 3; j++) cmdRate[j] = (cmd[j] - prevCmd[j]) / GuidanceStep;
                prevCmd = (double[])cmd.Clone();
                bool ok = guidance.Update(x, cmd, t, flight.TurnAnchor && (flight.TurnAccel > 0.0 || flight.TurnAccelBurn > 0.0) ? cmdRate : null);
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
            if (engine && double.IsNaN(commanded)) commanded = t;
            if (engine && flight.Tracker != null)
            {
                // The game's command: the tracker's thrust, as the throttle that gives it by the engine as now learned, inside what the engine can do.
                KsaPointMassModel model = guidance.Model;
                double thrust = flight.Tracker.Command(plan, t, x, out cmd);
                throttle = Math.Clamp(model.ThrottleFor(thrust, model.Altitude(x)), burn.ThrottleFloor, 1.0);
            }
            else
            {
                cmd = [up[0], up[1], up[2]];
                throttle = engine ? up[3] : 0.0;
            }
            lit = engine && t >= commanded + c.IgnitionDelay;

            // STOPPED SHORT, the game's rule: a craft that cannot throttle below its weight climbs from wherever it stops, so the engine is cut there and it drops the rest of the way. The scenario's floor makes this one such a craft.
            if (lit && !cut && x[5] >= 0.0 && x[2] > Xf[2] && MinTwr(truth, x, burn.ThrottleFloor) > 1.0)
            {
                cut = true;
                cutHeight = x[2] - Xf[2];
            }
            if (cut) lit = false;

            if (lit)
                peakAccel = Math.Max(peakAccel, truth.Thrust(throttle, truth.Altitude(x)) / x[PointMass3Dof.IM]);

            if (flight.PlantAccel > 0.0)
                att = TurnLimited(att, cmd, c.LagS, glide.AttitudeRateMax * 1.2, flight.PlantAccel, turn);
            else
                att = Turn(att, cmd, c.LagS, glide.AttitudeRateMax * 1.2);
            x = Step(truth, x, att, lit ? throttle : 0.0);
            t += SimStep;

            if (x[2] <= Xf[2] || x[6] <= ThreeDofScenario.DryMass || (engine && !cut && t > plan.EndTime + 3.0))
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
        Console.WriteLine($"{c.Name,-18} {miss,7:F1} {vh,7:F2} {vz,7:F2} {prop,8:F0} {drift,11:F2} {refused,7} {admmPerCycle,8:F0} {p50,8:F0}/{max,-8:F0} {swapJump,11:F1} {(cut ? cutHeight.ToString("F1") : "-"),6} {peakAccel / 9.80665,6:F2} {t - commanded,6:F1}  " +
                          $"{(pass ? "ok" : landed ? "OFF TARGET" : $"FAILED at {x[2]:F0} m: {guidance.Error}")}");
        return new Result(pass, true, miss, vz, refused, admmPerCycle, p50, max);
    }

    /// <summary>Thrust at the throttle floor over the weight, where the craft is.</summary>
    private static double MinTwr(KsaPointMassModel model, double[] x, double floor)
    {
        double altitude = model.Altitude(x);
        double g = model.Mu / Sq(model.MeanRadius + altitude);
        return model.Thrust(floor, altitude) / (x[PointMass3Dof.IM] * g);
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

    /// <summary>
    /// An attitude loop that, as a real vehicle's, cannot change its turn rate instantly: it wants to close on the command as the lag would, no faster than the rate cap and no faster than it can stop from, and its axis rate <paramref name="turn"/> (carried between calls) moves towards that by at most <paramref name="accel"/> * dt a step.
    /// </summary>
    private static double[] TurnLimited(double[] att, double[] cmd, double lagS, double rate, double accel, double[] turn)
    {
        double cos = Math.Clamp(att[0] * cmd[0] + att[1] * cmd[1] + att[2] * cmd[2], -1, 1);
        double angle = Math.Acos(cos);
        double[] e = [cmd[0] - cos * att[0], cmd[1] - cos * att[1], cmd[2] - cos * att[2]];
        double el = Math.Sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        double speed = Math.Min(Math.Min(rate, Math.Sqrt(2.0 * accel * angle)), angle / Math.Max(lagS, 1e-3));
        double[] want = el > 1e-12 ? [e[0] / el * speed, e[1] / el * speed, e[2] / el * speed] : new double[3];
        double[] d = [want[0] - turn[0], want[1] - turn[1], want[2] - turn[2]];
        double dl = Math.Sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]), max = accel * SimStep;
        double k = dl > max ? max / dl : 1.0;
        for (int j = 0; j < 3; j++) turn[j] += k * d[j];
        double[] b = [att[0] + turn[0] * SimStep, att[1] + turn[1] * SimStep, att[2] + turn[2] * SimStep];
        double bl = Math.Sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
        b = [b[0] / bl, b[1] / bl, b[2] / bl];
        // Keep the rate in the new tangent plane.
        double r = turn[0] * b[0] + turn[1] * b[1] + turn[2] * b[2];
        for (int j = 0; j < 3; j++) turn[j] -= r * b[j];
        return b;
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

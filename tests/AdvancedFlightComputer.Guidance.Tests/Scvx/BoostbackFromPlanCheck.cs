using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// --boostback-replay &lt;problem.json&gt;: solve a dumped convex ascent that brings a booster home, then fly the in-flight boostback planner (BoostbackShooter, as GuidanceWindow.UpdateBoostbackPlan calls it cold) from where the plan separates the booster, after the coast to the burn. Says whether the burn the plan priced is one the booster's own planner can find.
/// </summary>
internal static class BoostbackFromPlanCheck
{
    internal static int Run(string path, string[] args)
    {
        if (!File.Exists(path))
        {
            Console.WriteLine($"no such problem file: '{path}'");
            return 2;
        }
        AscentProblem p = AscentProblemFile.FromJson(File.ReadAllText(path));
        AscentRecovery? rec = p.Recovery;
        if (rec == null)
        {
            Console.WriteLine("the problem brings no booster home");
            return 2;
        }
        // The file keeps the margin it was planned with; --margin tries another.
        double margin = Arg(args, "--margin", rec.BoostbackMargin);
        if (margin != rec.BoostbackMargin)
        {
            rec = new AscentRecovery
            {
                Stage = rec.Stage, InitialReserveKg = rec.InitialReserveKg, BoosterDryKg = rec.BoosterDryKg, ExhaustVelocity = rec.ExhaustVelocity,
                SiteCci = rec.SiteCci, ReferenceArea = rec.ReferenceArea, Drag = rec.Drag, Lift = rec.Lift, Alpha = rec.Alpha, Air = rec.Air,
                LandingFactor = rec.LandingFactor, BoostbackMargin = margin, BurnDelayS = rec.BurnDelayS,
            };
            p = p.WithRecovery(rec);
        }
        AscentSolution sol = AscentScvx.Solve(p);
        Console.WriteLine($"plan: {sol.Message}, {sol.FinalMass / 1000.0:F2} t to orbit; booster keeps {sol.RecoveryReserveKg:F0} kg for a {sol.BoostbackDv:F0} m/s boostback, separates at T+{sol.SeparationTime:F1} s");
        if (!double.IsFinite(sol.RecoveryReserveKg))
            return 1;

        int k = 0;
        for (int i = 1; i < sol.Nodes; i++)
            if (Math.Abs(sol.Time[i] - sol.SeparationTime) < Math.Abs(sol.Time[k] - sol.SeparationTime))
                k = i;
        double[] r = [sol.Position[3 * k], sol.Position[3 * k + 1], sol.Position[3 * k + 2]];
        double[] v = [sol.Velocity[3 * k], sol.Velocity[3 * k + 1], sol.Velocity[3 * k + 2]];
        double mass0 = rec.BoosterDryKg + sol.RecoveryReserveKg;
        AscentStage stage = p.Stages[rec.Stage];
        double rr = Math.Sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
        double vv = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        Console.WriteLine($"separation: node {k} at T+{sol.Time[k]:F1} s, {(rr - p.BodyRadius) / 1000.0:F1} km up at {vv:F0} m/s inertial; booster {mass0 / 1000.0:F2} t, {stage.Thrust / 1000.0:F0} kN, {stage.MassFlow:F1} kg/s, ve {stage.Thrust / stage.MassFlow:F0} m/s");

        double delay = Arg(args, "--delay", rec.BurnDelayS);
        var coast = new DragCoastSystem
        {
            Mu = p.Mu, OmegaZ = p.Omega, MeanRadius = p.BodyRadius,
            AreaOverMass = rec.ReferenceArea / mass0, Alpha = rec.Alpha,
            Table = rec.Drag, LiftTable = rec.Lift, Atmosphere = rec.Air,
        };
        Span<Dual> s0 = stackalloc Dual[6];
        Span<Dual> s1 = stackalloc Dual[6];
        for (int i = 0; i < 3; i++) { s0[i] = r[i]; s0[3 + i] = v[i]; }
        var scratch = new Dual[Math.Max(Rk4.IntegrateScratch(6), BoostbackShooter.ScratchLength)];
        Rk4.Integrate(in coast, 0.0, s0, delay, 1.0, s1, scratch);
        double[] x0 = new double[6];
        for (int i = 0; i < 6; i++) x0[i] = s1[i].V;

        double t = sol.SeparationTime + delay;
        double a = p.Omega * t, c = Math.Cos(a), sn = Math.Sin(a);
        double[] site = rec.SiteCci;
        double[] target = [c * site[0] - sn * site[1], sn * site[0] + c * site[1], site[2]];
        double siteRadius = Math.Sqrt(site[0] * site[0] + site[1] * site[1] + site[2] * site[2]);

        var sys = new PoweredBurnSystem
        {
            Mu = p.Mu, OmegaZ = p.Omega, MeanRadius = p.BodyRadius,
            ReferenceArea = rec.ReferenceArea, Thrust = stage.Thrust, MassFlow = stage.MassFlow,
            Table = rec.Drag, Atmosphere = rec.Air, GlideAlpha = rec.Alpha, LiftTable = rec.Lift,
        };
        var opt = ImpactOptions.Default(p.BodyRadius);
        opt.MaxTime = 3600.0;
        opt.PathStride = 0;
        opt.StepAir = 3.0;
        opt.StepVacuum = 16.0;
        opt.TargetRadius = siteRadius;

        // The landing burn's propellant is not the boostback's: the game bounds the burn by everything aboard, so do the same, then by the boostback's share alone.
        double ve = stage.Thrust / stage.MassFlow;
        double impulsiveTgo = mass0 / stage.MassFlow * (1.0 - Math.Exp(-sol.BoostbackDv / ve));
        double pitchMin = Arg(args, "--min-pitch", 0.0);
        foreach ((string label, double maxBurn, double guessPitch, double guessDur, int sweeps) in new[]
                 {
                     ("as the game calls it cold", sol.RecoveryReserveKg / stage.MassFlow, Math.Max(pitchMin, 10.0), Math.Clamp(impulsiveTgo, 5.0, 300.0), 10),
                     ("no propellant bound", double.PositiveInfinity, Math.Max(pitchMin, 10.0), Math.Clamp(impulsiveTgo, 5.0, 300.0), 10),
                     ("no bound, 30 sweeps", double.PositiveInfinity, Math.Max(pitchMin, 10.0), Math.Clamp(impulsiveTgo, 5.0, 300.0), 30),
                     ("no bound, level start", double.PositiveInfinity, 0.0, Math.Clamp(impulsiveTgo, 5.0, 300.0), 30),
                 })
        {
            var guess = new BurnParameters { PitchDeg = guessPitch, Duration = guessDur };
            var sw = System.Diagnostics.Stopwatch.StartNew();
            BoostbackShooter.SolveResult shot = BoostbackShooter.Solve(in sys, x0, mass0, target, in guess, in opt, scratch,
                minPitchDeg: pitchMin, maxDuration: maxBurn, searchRates: false, maxSweeps: sweeps, initialPitchStepDeg: 8.0);
            string why = shot.Converged ? "converged"
                : shot.Shot.OutOfPropellant ? "not enough propellant"
                : shot.Shot.FlewIntoGround ? "flies into the ground"
                : shot.Shot.Infeasible ? "no burn reaches the site"
                : "did not converge";
            double left = sol.RecoveryReserveKg - shot.PropellantKg, landing = rec.BoosterDryKg * (Math.Exp(new BoostbackOracle(p, rec).LandingDv / ve) - 1.0);
            if (shot.Converged)
                Console.WriteLine($"  {label,-28} leaves {left:F0} kg for a landing that needs {landing:F0} kg: {(left >= landing ? "enough" : "SHORT")}");
            Console.WriteLine($"  {label,-28} {why,-24} miss {shot.MissM / 1000.0,8:F1} km, burn {shot.Parameters.Duration,6:F1} s ({shot.Parameters.Duration * stage.MassFlow / 1000.0:F1} t of {sol.RecoveryReserveKg / 1000.0:F1} t), pitch {shot.Parameters.PitchDeg,6:F1} deg, {shot.Shots} shots, {sw.Elapsed.TotalMilliseconds:F0} ms; guess {guessDur:F0} s at {guessPitch:F0} deg");
        }
        return 0;
    }

    private static double Arg(string[] args, string name, double fallback)
    {
        int i = Array.IndexOf(args, name);
        return i >= 0 && i + 1 < args.Length && double.TryParse(args[i + 1], System.Globalization.CultureInfo.InvariantCulture, out double v) ? v : fallback;
    }
}

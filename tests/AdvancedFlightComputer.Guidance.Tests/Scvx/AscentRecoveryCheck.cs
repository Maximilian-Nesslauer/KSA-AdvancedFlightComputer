using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// The booster reserve chosen by the solve (#84): the reserve is a variable, held to what the booster needs to come home from wherever the plan separates it.
///
/// On the AoA check's 2stage_new stack (an 8.3 t booster, KSA's air, the box-law drag table) returning to its pad:
///   - the oracle: a boostback impulse that is finite and positive, a slope that agrees with a central difference, and the landing burn at 1.3 times a terminal velocity of a sensible size;
///   - the free reserve: converged, the booster's need met and not exceeded, and the need recomputed at the plan's own separation agreeing with the plan's;
///   - against fixed reserves either side of it: those short of it leave the booster unable to come home, and none that can comes home with more to orbit;
///   - from a different starting reserve, a plan that also comes home and does as well: the problem is not convex, so not the same plan.
/// </summary>
internal static class AscentRecoveryCheck
{
    private static int _fails;

    private static void Check(string name, bool ok, string detail = "")
    {
        Console.WriteLine($"  {(ok ? "PASS" : "FAIL")}  {name,-64} {detail}");
        if (!ok) _fails++;
    }

    private static AscentProblem Base() => AscentAoaCheck.Problem(AscentAoaCheck.Table(attitude: true));

    /// <summary>The stack with <paramref name="reserveKg"/> kept in its booster, as AscentPhasePlan keeps it; with <paramref name="free"/>, the solve free to change it.</summary>
    private static (AscentProblem P, AscentRecovery Rec) WithReserve(double reserveKg, bool free)
    {
        AscentProblem p = Base();
        AscentStage b = p.Stages[0];
        double dry = b.JettisonMass;
        var stages = (AscentStage[])p.Stages.Clone();
        stages[0] = b.With(b.PropellantMass - reserveKg, dry + reserveKg, b.LiquidCarriesOver);
        double rn = Math.Sqrt(p.R0[0] * p.R0[0] + p.R0[1] * p.R0[1] + p.R0[2] * p.R0[2]);
        var air = (KsaAscentAtmosphere)p.Atmosphere;
        var rec = new AscentRecovery
        {
            Stage = 0,
            InitialReserveKg = reserveKg,
            BoosterDryKg = dry,
            ExhaustVelocity = b.Thrust / b.MassFlow,
            SiteCci = [p.R0[0] / rn * p.BodyRadius, p.R0[1] / rn * p.BodyRadius, p.R0[2] / rn * p.BodyRadius],
            ReferenceArea = AscentAoaCheck.DragArea,
            Drag = air.Table!,
            Air = air.Atmosphere,
        };
        return (new AscentProblem
        {
            Mu = p.Mu, BodyRadius = p.BodyRadius, Omega = p.Omega, R0 = p.R0, V0 = p.V0, M0 = p.M0, Stages = stages,
            Atmosphere = p.Atmosphere, GroundRadius = p.GroundRadius, TargetRadius = p.TargetRadius, TargetSpeed = p.TargetSpeed,
            TargetRadialRate = p.TargetRadialRate, PlaneNormal = p.PlaneNormal, QMax = p.QMax, QAlphaMax = p.QAlphaMax,
            Recovery = free ? rec : null,
        }, rec);
    }

    /// <summary>The booster's need recomputed at a plan's own separation: its stage's last node.</summary>
    private static BoostbackOracle.Need NeedAt(AscentProblem p, AscentRecovery rec, AscentSolution s)
    {
        int k = Array.LastIndexOf(s.NodeStage, 0);
        var oracle = new BoostbackOracle(p, rec);
        return oracle.Evaluate(s.Position.AsSpan(k * 3, 3), s.Velocity.AsSpan(k * 3, 3), s.Time[k], null, gradient: false);
    }

    internal static int Run(bool verbose)
    {
        _fails = 0;
        Console.WriteLine("ASCENT RECOVERY: the booster reserve chosen by the solve, held to what the booster needs to come home");

        // ---- the oracle, at a fixed 10 t reserve's separation.
        (AscentProblem fixed10, AscentRecovery rec10) = WithReserve(10e3, free: false);
        AscentSolution f10 = AscentScvx.Solve(fixed10);
        Console.WriteLine($"  fixed 10 t: {f10.Message}, {f10.FinalMass / 1000.0:F2} t to orbit");
        var oracle = new BoostbackOracle(fixed10, rec10);
        int ks = Array.LastIndexOf(f10.NodeStage, 0);
        double[] r = f10.Position[(ks * 3)..(ks * 3 + 3)], v = f10.Velocity[(ks * 3)..(ks * 3 + 3)];
        double t = f10.Time[ks];
        BoostbackOracle.Need n = oracle.Evaluate(r, v, t, null, gradient: true);
        Console.WriteLine($"  separation at {t:F1} s: boostback {n.BoostbackDv:F0} m/s, landing {oracle.LandingDv:F0} m/s (terminal {oracle.TerminalVelocity:F1} m/s), needs {n.RequiredKg:F0} kg");
        Check("oracle: the booster lands", n.Ok);
        Check("oracle: a boostback impulse of a sensible size", n.Ok && n.BoostbackDv > 50.0 && n.BoostbackDv < 6000.0, $"{n.BoostbackDv:F0} m/s");
        Check("oracle: a terminal velocity of a sensible size", oracle.TerminalVelocity > 20.0 && oracle.TerminalVelocity < 400.0, $"{oracle.TerminalVelocity:F1} m/s");
        Check("oracle: landing burn is 1.3 x terminal velocity", Math.Abs(oracle.LandingDv - 1.3 * oracle.TerminalVelocity) < 1e-9, $"{oracle.LandingDv:F1} m/s");
        if (n.Ok)
        {
            // The slope along the velocity gradient, against a central difference of 20 m/s.
            double gn = Math.Sqrt(n.GradVelocity.Sum(g => g * g));
            double[] d = n.GradVelocity.Select(g => g / gn).ToArray();
            const double h = 10.0;
            double[] vp = [v[0] + h * d[0], v[1] + h * d[1], v[2] + h * d[2]], vm = [v[0] - h * d[0], v[1] - h * d[1], v[2] - h * d[2]];
            BoostbackOracle.Need np = oracle.Evaluate(r, vp, t, n.Impulse, false), nm = oracle.Evaluate(r, vm, t, n.Impulse, false);
            double central = (np.RequiredKg - nm.RequiredKg) / (2.0 * h);
            Check("oracle: the velocity slope agrees with a central difference", np.Ok && nm.Ok && Math.Abs(central - gn) < 0.1 * Math.Abs(central),
                $"{gn:F2} against {central:F2} kg per m/s");
            Check("oracle: the need is a fraction of the booster's load", n.RequiredKg > 0.0 && n.RequiredKg < fixed10.Stages[0].PropellantMass + 10e3, $"{n.RequiredKg:F0} kg");
        }

        // ---- the free reserve, from no reserve at all.
        (AscentProblem freeP, AscentRecovery freeRec) = WithReserve(0.0, free: true);
        AscentSolution free = AscentScvx.Solve(freeP, onPhase: ph => Console.WriteLine("    -- " + ph));
        Console.WriteLine($"  free, from none: {free.Message}, {free.FinalMass / 1000.0:F2} t to orbit, keeps {free.RecoveryReserveKg:F0} kg against a need of {free.RecoveryRequiredKg:F0} kg "
                        + $"(boostback {free.BoostbackDv:F0} m/s, landing {free.LandingDv:F0} m/s), separates at {free.SeparationTime:F1} s");
        Check("free reserve: usable", free.Usable, free.Message);
        Check("free reserve: the need is met", free.RecoveryReserveKg >= free.RecoveryRequiredKg - 1e-4 * freeP.M0,
            $"{free.RecoveryReserveKg:F0} kg against {free.RecoveryRequiredKg:F0} kg");
        Check("free reserve: and not exceeded", free.RecoveryReserveKg <= free.RecoveryRequiredKg * 1.01 + 50.0,
            $"{free.RecoveryReserveKg:F0} kg against {free.RecoveryRequiredKg:F0} kg");
        BoostbackOracle.Need again = NeedAt(freeP, freeRec, free);
        Check("free reserve: the need at the plan's own separation agrees", again.Ok && Math.Abs(again.RequiredKg - free.RecoveryRequiredKg) < 0.01 * free.RecoveryRequiredKg + 20.0,
            $"{again.RequiredKg:F0} against {free.RecoveryRequiredKg:F0} kg");

        // ---- fixed reserves either side of the free one.
        double best = 0.0;
        double rStar = free.RecoveryReserveKg;
        foreach (double f in new[] { 0.8, 0.9, 1.0, 1.1, 1.25 })
        {
            double res = f * rStar;
            (AscentProblem p, AscentRecovery rec) = WithReserve(res, free: false);
            AscentSolution s = AscentScvx.Solve(p);
            BoostbackOracle.Need need = s.Nodes > 0 ? NeedAt(p, rec, s) : default;
            bool home = need.Ok && res >= need.RequiredKg - 1e-4 * p.M0;
            Console.WriteLine($"  fixed {res / 1000.0:F2} t ({f:F2} x): {s.Message}, {s.FinalMass / 1000.0:F2} t to orbit, needs {need.RequiredKg / 1000.0:F2} t: {(home ? "comes home" : "short")}");
            if (f < 0.95)
                Check($"fixed {f:F2} x: short of the need", !home, $"{res:F0} kg against {need.RequiredKg:F0} kg");
            if (home && s.Usable)
                best = Math.Max(best, s.FinalMass);
        }
        Check("free reserve: no fixed reserve that comes home does better", best > 0.0 && free.FinalMass >= best - 50.0,
            $"{free.FinalMass / 1000.0:F2} t against {best / 1000.0:F2} t");

        // ---- from a reserve well above the need. Not the same plan to the kilogram - the problem is not convex, and the reserve and the separation feed each other, so each start finds its own nearby plan - but one that also brings the booster home, and no worse than any fixed reserve that does.
        AscentProblem highP = WithReserve(1.5 * rStar, free: true).P;
        if (Environment.GetEnvironmentVariable("AFC_RECOVERY_DUMP") is { Length: > 0 } dumpPath)
            File.WriteAllText(dumpPath, AscentProblemFile.ToJson(highP));
        AscentSolution high = AscentScvx.Solve(highP, onPhase: ph => Console.WriteLine("    -- " + ph));
        Console.WriteLine($"  free, from {1.5 * rStar / 1000.0:F2} t: {high.Message}, {high.FinalMass / 1000.0:F2} t to orbit, keeps {high.RecoveryReserveKg:F0} kg against a need of {high.RecoveryRequiredKg:F0} kg, separates at {high.SeparationTime:F1} s");
        Check("free from above: usable", high.Usable, high.Message);
        Check("free from above: the need is met", high.RecoveryReserveKg >= high.RecoveryRequiredKg - 1e-4 * freeP.M0, $"{high.RecoveryReserveKg:F0} kg against {high.RecoveryRequiredKg:F0} kg");
        Check("free from above: a reserve near the other start's", Math.Abs(high.RecoveryReserveKg - rStar) < 0.1 * rStar, $"{high.RecoveryReserveKg:F0} against {rStar:F0} kg");
        Check("free from above: no fixed reserve that comes home does better", best > 0.0 && high.FinalMass >= best - 50.0, $"{high.FinalMass / 1000.0:F2} t against {best / 1000.0:F2} t");

        // ---- a problem the game writes out solves the same once read back.
        AscentSolution replay = AscentScvx.Solve(AscentProblemFile.FromJson(AscentProblemFile.ToJson(freeP)));
        // Not to the kilogram: the file samples the drag table, which moves its values in the sixteenth digit, and the solve carries that to 0.25 kg on a plain ascent and some 15 kg with the booster's need.
        Check("problem file: the recovery survives the round trip", Math.Abs(replay.RecoveryReserveKg - free.RecoveryReserveKg) < 0.01 * free.RecoveryReserveKg && Math.Abs(replay.FinalMass - free.FinalMass) < 50.0,
            $"{replay.RecoveryReserveKg:F0} kg, {replay.FinalMass / 1000.0:F3} t against {free.RecoveryReserveKg:F0} kg, {free.FinalMass / 1000.0:F3} t");

        Console.WriteLine();
        Console.WriteLine(_fails == 0 ? "ALL PASS" : $"{_fails} FAILED");
        return _fails == 0 ? 0 : 1;
    }
}

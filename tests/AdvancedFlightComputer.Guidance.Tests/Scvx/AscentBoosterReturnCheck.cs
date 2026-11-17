using AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// The booster's way down from a plan's separation (BoosterReturn), and the optimal return (a drone ship), on the AoA check's 2stage_new stack.
///   - Sent to its pad: the track lands on the pad, and its boostback is the plan's.
///   - Optimal return: the reserve is the landing burn alone, with no boostback, and the plan reaches orbit with more than the return to the pad does. A site set where its track lands needs next to no boostback, which is what lets a ship placed there take it.
///   - Expendable: its fall is worked out from a plan with no reserve.
/// </summary>
internal static class AscentBoosterReturnCheck
{
    private static int _fails;

    private static void Check(string name, bool ok, string detail = "")
    {
        Console.WriteLine($"  {(ok ? "PASS" : "FAIL")}  {name,-72} {detail}");
        if (!ok) _fails++;
    }

    private static (AscentProblem P, AscentRecovery Rec) Stack(double[]? site, bool optimal)
    {
        AscentProblem p = AscentAoaCheck.Problem(AscentAoaCheck.Table(attitude: true));
        AscentStage b = p.Stages[0];
        double rn = Math.Sqrt(p.R0[0] * p.R0[0] + p.R0[1] * p.R0[1] + p.R0[2] * p.R0[2]);
        var air = (KsaAscentAtmosphere)p.Atmosphere;
        var rec = new AscentRecovery
        {
            Stage = 0,
            InitialReserveKg = 0.0,
            BoosterDryKg = b.JettisonMass,
            OptimalReturn = optimal,
            ExhaustVelocity = b.Thrust / b.MassFlow,
            SiteCci = site ?? [p.R0[0] / rn * p.BodyRadius, p.R0[1] / rn * p.BodyRadius, p.R0[2] / rn * p.BodyRadius],
            ReferenceArea = AscentAoaCheck.DragArea,
            Drag = air.Table!,
            Air = air.Atmosphere,
        };
        return (p.WithRecovery(rec), rec);
    }

    private static double Distance(double[] a, double[] b)
        => Math.Sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));

    internal static int Run(bool verbose)
    {
        _fails = 0;
        Console.WriteLine("ASCENT BOOSTER RETURN: each booster's way down from the plan, and the optimal return");

        // ---- to the pad.
        (AscentProblem padP, AscentRecovery padRec) = Stack(null, optimal: false);
        AscentSolution pad = AscentScvx.Solve(padP);
        Console.WriteLine($"  to the pad: {pad.Message}, {pad.FinalMass / 1000.0:F2} t to orbit, keeps {pad.RecoveryReserveKg:F0} kg, boostback {pad.BoostbackDv:F0} m/s");
        Check("to the pad: usable", pad.Usable, pad.Message);
        BoosterReturnTrack padTrack = BoosterReturn.Compute(padP, padRec, BoosterReturnMode.Site, pad);
        Check("to the pad: the track comes down", padTrack.Ok, padTrack.Note);
        if (padTrack.Ok)
        {
            Check("to the pad: and lands on the pad", Distance(padTrack.Impact, padRec.SiteCci) < 50.0, $"{Distance(padTrack.Impact, padRec.SiteCci):F1} m off");
            Check("to the pad: with the plan's boostback", Math.Abs(padTrack.BoostbackDv - pad.BoostbackDv) < 0.01 * pad.BoostbackDv + 1.0,
                $"{padTrack.BoostbackDv:F1} against {pad.BoostbackDv:F1} m/s");
            Check("to the pad: from the plan's separation", Math.Abs(padTrack.Time[0] - pad.SeparationTime) < 1e-6 && padTrack.Points > 20,
                $"T+{padTrack.Time[0]:F1} s, {padTrack.Points} points");
        }

        // ---- optimal return.
        (AscentProblem optP, AscentRecovery optRec) = Stack(null, optimal: true);
        AscentSolution opt = AscentScvx.Solve(optP);
        double landingOnly = new BoostbackOracle(optP, optRec).RequiredKg(0.0);
        Console.WriteLine($"  optimal return: {opt.Message}, {opt.FinalMass / 1000.0:F2} t to orbit, keeps {opt.RecoveryReserveKg:F0} kg (landing burn {opt.LandingDv:F0} m/s needs {landingOnly:F0} kg)");
        Check("optimal return: usable", opt.Usable, opt.Message);
        Check("optimal return: no boostback", opt.BoostbackDv == 0.0, $"{opt.BoostbackDv} m/s");
        Check("optimal return: keeps the landing burn alone", Math.Abs(opt.RecoveryReserveKg - landingOnly) < 0.01 * landingOnly + 20.0,
            $"{opt.RecoveryReserveKg:F0} against {landingOnly:F0} kg");
        Check("optimal return: more to orbit than the return to the pad", opt.FinalMass > pad.FinalMass, $"{opt.FinalMass / 1000.0:F2} t against {pad.FinalMass / 1000.0:F2} t");
        BoosterReturnTrack optTrack = BoosterReturn.Compute(optP, optRec, BoosterReturnMode.OptimalReturn, opt);
        Check("optimal return: the track comes down", optTrack.Ok, optTrack.Note);
        if (optTrack.Ok)
        {
            double down = Distance(optTrack.Impact, optRec.SiteCci) / 1000.0;
            Console.WriteLine($"    lands {down:F0} km from the pad, at T+{optTrack.Time[^1]:F0} s");
            Check("optimal return: downrange of the pad", down > 20.0, $"{down:F0} km");

            // A ship placed where it lands: a site there, and the plan solved again for it from the optimal return's own climb. From the gravity-turn seed instead the solve settles on another local optimum (78 m/s of boostback, 9.13 t to orbit), as the free reserve does elsewhere; from this start the separation, and so the coast onto the point, is the optimal return's.
            (AscentProblem shipP, AscentRecovery shipRec) = Stack(optTrack.Impact, optimal: false);
            AscentSolution ship = AscentScvx.Solve(shipP, start: opt);
            Console.WriteLine($"  site at that point: {ship.Message}, {ship.FinalMass / 1000.0:F2} t to orbit, keeps {ship.RecoveryReserveKg:F0} kg, boostback {ship.BoostbackDv:F0} m/s");
            Check("ship at that point: usable", ship.Usable, ship.Message);
            Check("ship at that point: next to no boostback", ship.BoostbackDv < 50.0, $"{ship.BoostbackDv:F1} m/s");
            Check("ship at that point: about the optimal return's payload", Math.Abs(ship.FinalMass - opt.FinalMass) < 0.01 * opt.FinalMass,
                $"{ship.FinalMass / 1000.0:F2} t against {opt.FinalMass / 1000.0:F2} t");
        }

        // ---- the landing burn at sea level: a vacuum-tuned engine's exhaust velocity near the ground is far below the stage's (a heavy's core read 436 s and landed at 342 s), and the reserve must grow by exactly what the rocket equation says.
        var seaLevel = new AscentRecovery
        {
            Stage = 0, InitialReserveKg = 0.0, BoosterDryKg = optRec.BoosterDryKg, OptimalReturn = true,
            ExhaustVelocity = optRec.ExhaustVelocity, LandingExhaustVelocity = optRec.ExhaustVelocity * 342.0 / 436.0,
            SiteCci = optRec.SiteCci, ReferenceArea = optRec.ReferenceArea, Drag = optRec.Drag, Air = optRec.Air,
        };
        var vacOracle = new BoostbackOracle(optP, optRec);
        var slOracle = new BoostbackOracle(optP, seaLevel);
        double wantSl = seaLevel.BoosterDryKg * (Math.Exp(slOracle.LandingDv / seaLevel.LandingExhaustVelocity) - 1.0);
        Check("sea level: the landing reserve is priced at the sea-level exhaust velocity", Math.Abs(slOracle.RequiredKg(0.0) - wantSl) < 1e-6,
            $"{slOracle.RequiredKg(0.0):F1} against {wantSl:F1} kg");
        Check("sea level: and is larger than at vacuum", slOracle.RequiredKg(0.0) > vacOracle.RequiredKg(0.0) * 1.2,
            $"{slOracle.RequiredKg(0.0):F0} against {vacOracle.RequiredKg(0.0):F0} kg");
        double bb = 1000.0;
        double wantBoth = seaLevel.BoosterDryKg * (Math.Exp(seaLevel.BoostbackMargin * bb / seaLevel.ExhaustVelocity + slOracle.LandingDv / seaLevel.LandingExhaustVelocity) - 1.0);
        Check("sea level: the boostback stays at the vacuum exhaust velocity", Math.Abs(slOracle.RequiredKg(bb) - wantBoth) < 1e-6,
            $"{slOracle.RequiredKg(bb):F1} against {wantBoth:F1} kg");
        AscentProblem slBack = AscentProblemFile.FromJson(AscentProblemFile.ToJson(optP.WithRecovery(seaLevel)));
        Check("sea level: the problem file keeps it", slBack.Recovery?.LandingExhaustVelocity == seaLevel.LandingExhaustVelocity);
        AscentProblem vacBack = AscentProblemFile.FromJson(AscentProblemFile.ToJson(optP));
        Check("sea level: and an unset one stays unset", vacBack.Recovery != null && double.IsNaN(vacBack.Recovery.LandingExhaustVelocity));

        // ---- the landing burn held to an acceleration limit: braking from terminal velocity v at a against gravity g costs v a / (a - g), 1.5 v at 3 g, with its margin; the flat 1.3 v undersized it (2026-11-17).
        double g0 = optP.Mu / (optP.BodyRadius * optP.BodyRadius);
        var limited = new AscentRecovery
        {
            Stage = 0, InitialReserveKg = 0.0, BoosterDryKg = optRec.BoosterDryKg, OptimalReturn = true,
            ExhaustVelocity = optRec.ExhaustVelocity, LandingAccel = 3.0 * g0,
            SiteCci = optRec.SiteCci, ReferenceArea = optRec.ReferenceArea, Drag = optRec.Drag, Air = optRec.Air,
        };
        var limOracle = new BoostbackOracle(optP, limited);
        double wantLanding = limited.LandingMargin * limOracle.TerminalVelocity * 1.5;
        Check("landing at 3 g: the burn brakes from terminal velocity against gravity", Math.Abs(limOracle.LandingDv - wantLanding) < 1e-9,
            $"{limOracle.LandingDv:F1} against {wantLanding:F1} m/s ({limOracle.TerminalVelocity:F1} m/s terminal)");
        Check("landing at 3 g: more than the flat 1.3 times terminal velocity", limOracle.LandingDv > 1.3 * limOracle.TerminalVelocity);
        var weak = new AscentRecovery
        {
            Stage = 0, InitialReserveKg = 0.0, BoosterDryKg = optRec.BoosterDryKg, OptimalReturn = true,
            ExhaustVelocity = optRec.ExhaustVelocity, LandingAccel = 0.5 * g0,
            SiteCci = optRec.SiteCci, ReferenceArea = optRec.ReferenceArea, Drag = optRec.Drag, Air = optRec.Air,
        };
        double weakDv = new BoostbackOracle(optP, weak).LandingDv;
        Check("landing under 1 g: priced large and finite", double.IsFinite(weakDv) && weakDv > 5.0 * limOracle.TerminalVelocity, $"{weakDv:F0} m/s");
        AscentProblem limBack = AscentProblemFile.FromJson(AscentProblemFile.ToJson(optP.WithRecovery(limited)));
        Check("landing at 3 g: the problem file keeps it", limBack.Recovery?.LandingAccel == limited.LandingAccel && limBack.Recovery?.LandingMargin == limited.LandingMargin);

        // ---- expendable.
        AscentProblem bare = optP.WithRecovery(null);
        AscentSolution exp = AscentScvx.Solve(bare);
        var fall = new AscentRecovery
        {
            Stage = 0, InitialReserveKg = 0.0, BoosterDryKg = optRec.BoosterDryKg, ExhaustVelocity = optRec.ExhaustVelocity,
            SiteCci = optRec.SiteCci, ReferenceArea = optRec.ReferenceArea, Drag = optRec.Drag, Air = optRec.Air, LandingFactor = 0.0,
        };
        BoosterReturnTrack expTrack = BoosterReturn.Compute(bare, fall, BoosterReturnMode.Expendable, exp);
        Console.WriteLine($"  expendable: {exp.Message}, {exp.FinalMass / 1000.0:F2} t to orbit, falls {(expTrack.Ok ? Distance(expTrack.Impact, fall.SiteCci) / 1000.0 : double.NaN):F0} km from the pad");
        Check("expendable: its fall comes down", expTrack.Ok, expTrack.Note);
        Check("expendable: the most to orbit", exp.FinalMass > opt.FinalMass, $"{exp.FinalMass / 1000.0:F2} t against {opt.FinalMass / 1000.0:F2} t");

        // ---- the problem file keeps the optimal return.
        AscentProblem back = AscentProblemFile.FromJson(AscentProblemFile.ToJson(optP));
        Check("problem file: the optimal return survives the round trip", back.Recovery?.OptimalReturn == true);

        Console.WriteLine();
        Console.WriteLine(_fails == 0 ? "ALL PASS" : $"{_fails} FAILED");
        return _fails == 0 ? 0 : 1;
    }
}

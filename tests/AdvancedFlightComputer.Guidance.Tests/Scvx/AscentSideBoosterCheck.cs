using AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// A side-booster reserve on one throttle (#176): two side boosters and a core burning together, the boosters leaving with a reserve while the core burns on, then the upper stage.
///
/// The stack is the AoA check's 2stage_new booster three times over: two side boosters with its load and dry mass, and a core with half as much again, so it burns on alone after them. One throttle drives all nine engines, as KSA flies them, and the side boosters draw two thirds of the first stage's flow.
///
/// Checked:
///   - the free reserve is usable, and meets the boosters' need without overshooting it;
///   - the first stage burns the reserve over the boosters' share less, the boosters leave with their dry mass and the reserve, and the core burns its own share of what was kept back in the next stage;
///   - the same reserve modelled as the boosters' whole stage (a flow share of one, the old model) reaches orbit with less, because it throws the core's share away;
///   - the problem file keeps the flow share.
/// </summary>
internal static class AscentSideBoosterCheck
{
    private static int _fails;

    private const double SideLoadKg = 122583.26034570113;
    private const double DryKg = 8265.653716798872;
    private const double CoreLoadKg = 1.5 * SideLoadKg;
    private const double Share = 2.0 / 3.0;

    private static void Check(string name, bool ok, string detail = "")
    {
        Console.WriteLine($"  {(ok ? "PASS" : "FAIL")}  {name,-72} {detail}");
        if (!ok) _fails++;
    }

    private static AscentProblem Heavy(double flowShare)
    {
        AscentProblem p = AscentAoaCheck.Problem(AscentAoaCheck.Table(attitude: true));
        AscentStage core = p.Stages[0], upper = p.Stages[1];
        double[] three = AscentAoaCheck.BoosterThrust.Select(t => 3.0 * t).ToArray();
        double coreAlongside = SideLoadKg;   // the core burns as long as each side booster, at the same flow
        AscentStage[] stages =
        [
            new(3.0 * core.Thrust, 3.0 * core.MassFlow, 2.0 * SideLoadKg + coreAlongside, 2.0 * DryKg, AscentAoaCheck.DragArea, AscentAoaCheck.Grid, three),
            new(core.Thrust, core.MassFlow, CoreLoadKg - coreAlongside, DryKg, AscentAoaCheck.DragArea, AscentAoaCheck.Grid, AscentAoaCheck.BoosterThrust),
            upper,
        ];
        double payload = p.M0 - core.PropellantMass - core.JettisonMass - upper.PropellantMass;
        double m0 = stages.Sum(s => s.PropellantMass + s.JettisonMass) + payload;
        double rn = Math.Sqrt(p.R0[0] * p.R0[0] + p.R0[1] * p.R0[1] + p.R0[2] * p.R0[2]);
        var air = (KsaAscentAtmosphere)p.Atmosphere;
        var rec = new AscentRecovery
        {
            Stage = 0,
            InitialReserveKg = 0.0,
            BoosterDryKg = 2.0 * DryKg,
            FlowShare = flowShare,
            ExhaustVelocity = core.Thrust / core.MassFlow,
            SiteCci = [p.R0[0] / rn * p.BodyRadius, p.R0[1] / rn * p.BodyRadius, p.R0[2] / rn * p.BodyRadius],
            ReferenceArea = AscentAoaCheck.DragArea,
            Drag = air.Table!,
            Air = air.Atmosphere,
        };
        return new AscentProblem
        {
            Mu = p.Mu, BodyRadius = p.BodyRadius, Omega = p.Omega, R0 = p.R0, V0 = p.V0, M0 = m0, Stages = stages,
            Atmosphere = p.Atmosphere, GroundRadius = p.GroundRadius, TargetRadius = p.TargetRadius, TargetSpeed = p.TargetSpeed,
            TargetRadialRate = p.TargetRadialRate, PlaneNormal = p.PlaneNormal, QMax = p.QMax, QAlphaMax = p.QAlphaMax,
            Recovery = rec,
        };
    }

    // v turned by angle about the unit axis k (Rodrigues).
    private static double[] Rotate(double[] v, double[] k, double angle)
    {
        double c = Math.Cos(angle), s = Math.Sin(angle);
        double kv = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
        double[] cross = [k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0]];
        return [v[0] * c + cross[0] * s + k[0] * kv * (1 - c), v[1] * c + cross[1] * s + k[1] * kv * (1 - c), v[2] * c + cross[2] * s + k[2] * kv * (1 - c)];
    }

    // A stage's first and last node: the last node of one stage and the first of the next are the same instant, either side of the separation.
    private static (int First, int Last) Nodes(AscentSolution s, int stage)
        => (Array.IndexOf(s.NodeStage, stage), Array.LastIndexOf(s.NodeStage, stage));

    internal static int Run(bool verbose)
    {
        _fails = 0;
        Console.WriteLine("ASCENT SIDE BOOSTERS: a reserve kept in side boosters beside a core on one throttle");

        AscentProblem p = Heavy(Share);
        Check("the problem is valid", p.Recovery!.Validate(p).Length == 0, p.Recovery.Validate(p));
        AscentSolution s = AscentScvx.Solve(p, onPhase: verbose ? ph => Console.WriteLine("    -- " + ph) : null);
        double r = s.RecoveryReserveKg;
        Console.WriteLine($"  share {Share:P0}: {s.Message}, {s.FinalMass / 1000.0:F2} t to orbit, keeps {r:F0} kg against a need of {s.RecoveryRequiredKg:F0} kg "
                        + $"(boostback {s.BoostbackDv:F0} m/s, landing {s.LandingDv:F0} m/s), separates at {s.SeparationTime:F1} s");
        Check("free reserve: usable", s.Usable, s.Message);
        Check("free reserve: the need is met", r >= s.RecoveryRequiredKg - 1e-4 * p.M0, $"{r:F0} kg against {s.RecoveryRequiredKg:F0} kg");
        Check("free reserve: and not exceeded", r <= s.RecoveryRequiredKg * 1.01 + 50.0, $"{r:F0} kg against {s.RecoveryRequiredKg:F0} kg");

        if (s.Nodes > 0 && double.IsFinite(r))
        {
            (int f0, int l0) = Nodes(s, 0);
            (int f1, int l1) = Nodes(s, 1);
            double burned0 = s.Mass[f0] - s.Mass[l0];
            double dropped = s.Mass[l0] - s.Mass[f1];
            double burned1 = s.Mass[f1] - s.Mass[l1];
            double want0 = p.Stages[0].PropellantMass - r / Share;
            double want1 = p.Stages[1].PropellantMass + r * (1.0 / Share - 1.0);
            // The collocation carries the mass to a fraction of a kilogram per node; a few kilograms over a stage is the integration, not the bookkeeping.
            const double tol = 50.0;
            Check("stage 1 burns its load less the reserve over the boosters' share", Math.Abs(burned0 - want0) < tol, $"{burned0:F0} kg against {want0:F0} kg");
            Check("the boosters leave with their dry mass and the reserve", Math.Abs(dropped - (2.0 * DryKg + r)) < tol, $"{dropped:F0} kg against {2.0 * DryKg + r:F0} kg");
            Check("the core burns its share of the reserve in stage 2", Math.Abs(burned1 - want1) < tol, $"{burned1:F0} kg against {want1:F0} kg");
        }

        // The old model: the reserve as the boosters' whole stage, the core's share of the shortened burn thrown away with them.
        AscentProblem lumped = Heavy(1.0);
        AscentSolution old = AscentScvx.Solve(lumped);
        Console.WriteLine($"  share 100%: {old.Message}, {old.FinalMass / 1000.0:F2} t to orbit, keeps {old.RecoveryReserveKg:F0} kg");
        Check("the lumped model reaches orbit with less", old.Usable && s.Usable && s.FinalMass > old.FinalMass,
            $"{s.FinalMass / 1000.0:F2} t against {old.FinalMass / 1000.0:F2} t");

        AscentProblem back = AscentProblemFile.FromJson(AscentProblemFile.ToJson(p));
        Check("problem file: the flow share survives the round trip", back.Recovery != null && Math.Abs(back.Recovery.FlowShare - Share) < 1e-12,
            $"{back.Recovery?.FlowShare:F6}");

        // The same bookkeeping when the stage list already holds a reserve back (AscentProblem.WithInitialReserve).
        AscentProblem held = p.WithInitialReserve(6000.0);
        Check("an initial reserve shortens stage 1 by it over the share", Math.Abs(held.Stages[0].PropellantMass - (p.Stages[0].PropellantMass - 6000.0 / Share)) < 1e-6);
        Check("and the core's share burns in stage 2", Math.Abs(held.Stages[1].PropellantMass - (p.Stages[1].PropellantMass + 6000.0 * (1.0 / Share - 1.0))) < 1e-6);
        Check("and lift-off mass is unchanged", Math.Abs(held.Stages.Sum(st => st.PropellantMass + st.JettisonMass) - p.Stages.Sum(st => st.PropellantMass + st.JettisonMass)) < 1e-6);

        ThreeHome(s);

        Console.WriteLine();
        Console.WriteLine(_fails == 0 ? "ALL PASS" : $"{_fails} FAILED");
        return _fails == 0 ? 0 : 1;
    }

    /// <summary>
    /// Every booster home at once (#130): the two side boosters, each to its own site, sharing their stage's reserve, and the core with a reserve of its own from its own separation.
    /// </summary>
    private static void ThreeHome(AscentSolution sideOnly)
    {
        AscentProblem side = Heavy(Share);
        AscentRecovery a = side.Recovery!;
        // Side booster B lands 20 km east of A's pad; the core lands on A's.
        double turn = 20e3 / side.BodyRadius;
        double[] siteA = a.SiteCci;
        double[] siteB = [Math.Cos(turn) * siteA[0] - Math.Sin(turn) * siteA[1], Math.Sin(turn) * siteA[0] + Math.Cos(turn) * siteA[1], siteA[2]];
        AscentRecovery b = new()
        {
            Stage = a.Stage, InitialReserveKg = a.InitialReserveKg, BoosterDryKg = a.BoosterDryKg, FlowShare = a.FlowShare,
            ExhaustVelocity = a.ExhaustVelocity, SiteCci = siteB, ReferenceArea = a.ReferenceArea, Drag = a.Drag, Air = a.Air,
        };
        // The core separates too fast to come back to the pad (a strap-on stack's core needed some 4 km/s for that), so it lands downrange, as a heavy's core does on a drone ship: 300 km on along the plane from where the side-booster plan separates it.
        (_, int coreEnd) = Nodes(sideOnly, 1);
        double[] sep = sideOnly.Position[(coreEnd * 3)..(coreEnd * 3 + 3)];
        double sepR = Math.Sqrt(sep[0] * sep[0] + sep[1] * sep[1] + sep[2] * sep[2]);
        double[] siteCore = Rotate(sep.Select(x => x / sepR * side.BodyRadius).ToArray(), side.PlaneNormal, 300e3 / side.BodyRadius);
        AscentRecovery core = new()
        {
            Stage = 1, InitialReserveKg = 0.0, BoosterDryKg = DryKg, FlowShare = 1.0,
            ExhaustVelocity = a.ExhaustVelocity, SiteCci = siteCore, ReferenceArea = a.ReferenceArea, Drag = a.Drag, Air = a.Air,
        };
        AscentProblem p = side.WithRecoveries([a, b, core]);
        Check("three home: the problem is valid", p.Validate().Length == 0, p.Validate());

        AscentSolution s = AscentScvx.Solve(p);
        Console.WriteLine($"  three home: {s.Message}, {s.FinalMass / 1000.0:F2} t to orbit");
        foreach (AscentRecoveryResult r in s.Recoveries)
            Console.WriteLine($"    stage {r.Stage + 1}: keeps {r.ReserveKg:F0} kg against a need of {r.RequiredKg:F0} kg (boostback {r.BoostbackDv:F0} m/s), separates at {r.SeparationTime:F1} s");
        Check("three home: usable", s.Usable, s.Message);
        Check("three home: a result per booster", s.Recoveries.Length == 3, $"{s.Recoveries.Length}");
        if (s.Recoveries.Length != 3)
            return;
        AscentRecoveryResult ra = s.Recoveries[0], rb = s.Recoveries[1], rc = s.Recoveries[2];
        double tol = 1e-4 * p.M0;
        Check("three home: the side boosters share one reserve", ra.Stage == 0 && rb.Stage == 0 && ra.ReserveKg == rb.ReserveKg);
        Check("three home: it meets both side boosters' needs", ra.ReserveKg >= ra.RequiredKg - tol && ra.ReserveKg >= rb.RequiredKg - tol,
            $"{ra.ReserveKg:F0} kg against {ra.RequiredKg:F0} and {rb.RequiredKg:F0} kg");
        Check("three home: and is held to the costlier of them", ra.ReserveKg <= Math.Max(ra.RequiredKg, rb.RequiredKg) * 1.01 + 50.0,
            $"{ra.ReserveKg:F0} kg against {Math.Max(ra.RequiredKg, rb.RequiredKg):F0} kg");
        Check("three home: the two sites cost differently", Math.Abs(ra.RequiredKg - rb.RequiredKg) > 10.0, $"{ra.RequiredKg:F0} and {rb.RequiredKg:F0} kg");
        Check("three home: the core's reserve meets its own need", rc.Stage == 1 && rc.ReserveKg >= rc.RequiredKg - tol && rc.ReserveKg <= rc.RequiredKg * 1.01 + 50.0,
            $"{rc.ReserveKg:F0} kg against {rc.RequiredKg:F0} kg");
        Check("three home: the core separates after the side boosters", rc.SeparationTime > ra.SeparationTime + 5.0,
            $"T+{ra.SeparationTime:F0} s against T+{rc.SeparationTime:F0} s");

        (int f1, int l1) = Nodes(s, 1);
        (int f2, _) = Nodes(s, 2);
        double burned1 = s.Mass[f1] - s.Mass[l1];
        double want1 = p.Stages[1].PropellantMass + ra.ReserveKg * (1.0 / Share - 1.0) - rc.ReserveKg;
        double dropped1 = s.Mass[l1] - s.Mass[f2];
        Check("three home: the core burns the side boosters' share, less its own reserve", Math.Abs(burned1 - want1) < 50.0, $"{burned1:F0} kg against {want1:F0} kg");
        Check("three home: the core leaves with its dry mass and its reserve", Math.Abs(dropped1 - (DryKg + rc.ReserveKg)) < 50.0, $"{dropped1:F0} kg against {DryKg + rc.ReserveKg:F0} kg");
        Check("three home: bringing the core home costs payload", sideOnly.Usable && s.FinalMass < sideOnly.FinalMass,
            $"{s.FinalMass / 1000.0:F2} t against {sideOnly.FinalMass / 1000.0:F2} t");

        AscentProblem back = AscentProblemFile.FromJson(AscentProblemFile.ToJson(p));
        Check("three home: the problem file keeps every booster", back.Recoveries.Count == 3 && back.Recoveries[2].Stage == 1 && back.Recoveries[1].SiteCci[0] == siteB[0]);
    }
}

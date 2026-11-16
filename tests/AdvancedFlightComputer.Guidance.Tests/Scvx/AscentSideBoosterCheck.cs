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

        Console.WriteLine();
        Console.WriteLine(_fails == 0 ? "ALL PASS" : $"{_fails} FAILED");
        return _fails == 0 ? 0 : 1;
    }
}

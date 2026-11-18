using AdvancedFlightComputer.Guidance.Scvx.Ascent;

internal static class AscentFairingCheck
{
    internal static int Run()
    {
        AscentProblem baseline = AscentCheck.SaturnV();
        AscentSolution seed = AscentScvx.Solve(baseline);
        Console.WriteLine($"Baseline {seed.Message}");
        if (!seed.Usable)
            return 1;

        var request = new AscentFairingRequest(2, [(101, 48.0), (102, 52.0)], baseline.BodyRadius + 150.0,
            30_000.0, 8000.0, Atmospheric: true);
        PlannedFairingRelease? evt = AscentFairingPlanner.Refine(request, baseline, seed, (p, s) =>
            AscentScvx.Solve(p, start: s),
            out AscentProblem split, out AscentSolution solved, out string reason);
        if (evt == null)
        {
            Console.WriteLine(reason);
            return 1;
        }
        int failures = 0;
        void Check(string label, bool pass, string detail = "")
        {
            Console.WriteLine($"  {(pass ? "PASS" : "FAIL")}  {label}  {detail}");
            if (!pass) failures++;
        }

        int phase = evt.Phase;
        AscentStage before = split.Stages[phase], after = split.Stages[phase + 1], old = baseline.Stages[phase];
        int end = Array.LastIndexOf(solved.NodeStage, phase);
        int first = Array.IndexOf(solved.NodeStage, phase);
        int last = Array.LastIndexOf(solved.NodeStage, phase + 1);
        double massJump = solved.Mass[end] - solved.Mass[end + 1];
        Check("the revised path clears both gates at the pinned event", solved.Usable && solved.GatesMet && evt.Row == 2
            && evt.Q <= request.QCeilingPa && evt.Radius - request.ClearanceDatum >= request.AltitudeFloorM
            && Math.Abs(evt.Time - solved.Time[end]) < 1e-9 && solved.Time[end] == solved.Time[end + 1],
            $"T+{evt.Time:F3} s, q {evt.Q:F1} Pa, clearance {evt.Radius - request.ClearanceDatum:F0} m");
        Check("100 kg leaves exactly once, with no position or velocity jump",
            Math.Abs(massJump - 100.0) < 0.01
            && MaxGap(solved.Position, end) < 1e-4 && MaxGap(solved.Velocity, end) < 1e-4
            && Math.Abs(baseline.FinalMassFloor - split.FinalMassFloor - 100.0) < 1e-6,
            $"mass jump {massJump:F6} kg, terminal floor delta {baseline.FinalMassFloor - split.FinalMassFloor:F1} kg");
        Check("the same engine and one propellant load span the event, then the real stage drops",
            split.Stages.Length == baseline.Stages.Length + 1 && before.IsPinned && before.LiquidCarriesOver
            && !after.IsPinned && !after.LiquidCarriesOver
            && before.Thrust == old.Thrust && after.Thrust == old.Thrust
            && before.MassFlow == old.MassFlow && after.MassFlow == old.MassFlow
            && before.JettisonMass == 100.0 && after.JettisonMass == old.JettisonMass
            && Math.Abs(before.PropellantMass + after.PropellantMass - old.PropellantMass) < 1e-6
            && Math.Abs(solved.Mass[first] - solved.Mass[last] - massJump - old.PropellantMass) < 0.1);
        Check("an airless request keeps the baseline and gives an explicit fallback",
            AscentFairingPlanner.Refine(request with { Atmospheric = false }, baseline, seed, (_, _) =>
                throw new InvalidOperationException("unexpected timed solve"),
                out AscentProblem retained, out AscentSolution fallback, out string why) == null
            && ReferenceEquals(retained, baseline) && ReferenceEquals(fallback, seed) && why.Length > 0);
        Check("an event too close to engine ignition keeps the original phase",
            !AscentFairingSplit.TryBuild(baseline, seed, phase, 0.1, 100,
                out AscentProblem unchanged, out _, out string boundaryReason) && ReferenceEquals(unchanged, baseline)
            && boundaryReason.Length > 0);
        Check("the pinned drop and shared load survive a problem dump",
            AscentProblemFile.FromJson(AscentProblemFile.ToJson(split)) is { } replay
            && replay.Stages[phase].FixedBurnTime == before.FixedBurnTime
            && replay.Stages[phase].LiquidCarriesOver && replay.Stages[phase].JettisonMass == 100.0
            && replay.FinalMassFloor == split.FinalMassFloor);
        return failures == 0 ? 0 : 1;
    }

    private static double MaxGap(double[] samples, int before)
    {
        int i = 3 * before;
        return Math.Max(Math.Abs(samples[i] - samples[i + 3]),
            Math.Max(Math.Abs(samples[i + 1] - samples[i + 4]), Math.Abs(samples[i + 2] - samples[i + 5])));
    }
}

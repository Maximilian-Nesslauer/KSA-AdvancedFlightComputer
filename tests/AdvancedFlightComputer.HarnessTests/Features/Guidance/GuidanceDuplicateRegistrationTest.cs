using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The oracle for the registration count is the game's own per-sequence part list, which
// SequencePerformanceList fills on every recompute and which the adapter mirrors from the sequenced
// modules of each part. The arithmetic is then checked on real parts against a phase built by hand.
// The test saves list every engine part under one sequence, so a hand-built sequence list that
// carries a part's number twice drives the mirror's count to two. That is the count the mirror
// reports for a part with modules in two sequences, such as the stock escape tower with its engine
// and two decouplers; the game itself keys its lists by number and would list the part once.
public sealed class GuidanceDuplicateRegistrationTest : AfcTest
{
    public override string Name => "afc-guidance-duplicate-registration";

    private static readonly string[] DefaultSaves = { "Test Vehicle 2", "Test Vehicle 1" };

    private static readonly AccessTools.FieldRef<SequencePerformanceList, List<List<Part>>> SequenceParts =
        AccessTools.FieldRefAccess<SequencePerformanceList, List<List<Part>>>("_sequencePartsScratch");

    protected override void Execute(TestContext t)
    {
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(DefaultSaves);
        if (saves.Count == 0)
        {
            t.Skip($"none of '{string.Join("', '", DefaultSaves)}' is in the game's Vehicles folder.");
            return;
        }

        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        Vehicle? previousFocus = Program.ControlledVehicle;
        try
        {
            Vehicle vehicle;
            try
            {
                vehicle = AutoStageFlightSupport.SpawnFromSave(t, saves[0], "AfcGuidanceDuplicateRegistration", out _);
            }
            catch (InvalidOperationException e)
            {
                t.Fail("spawn", e.Message);
                return;
            }

            PhysicsBubble._forceOffRails = true;
            Program.ControlledVehicle = vehicle;
            driver.Step(0.05, 40);

            MirrorMatchesTheGame(t, vehicle);
            AUniformCountDividesThePhase(t, vehicle);
            MixedCountsTakeTheLiquidOutOnce(t, vehicle);
        }
        finally
        {
            AutoStageFlightSupport.CleanupAfterFlight(t.System, preexisting);
            Program.ControlledVehicle = previousFocus;
        }
    }

    // Every engine part the drain simulation burned, in every sequence, counted by the game's lists
    // and by the mirror.
    private static void MirrorMatchesTheGame(TestContext t, Vehicle vehicle)
    {
        SequencePerformanceList performanceList = vehicle.Parts.PerformanceSequences;
        performanceList.RecomputeForFlight(0f);
        List<List<Part>> lists = SequenceParts(performanceList);
        ReadOnlySpan<Sequence> sequences = vehicle.Parts.SequenceList.Sequences;
        ReadOnlySpan<SequencePerformance> performance = performanceList.PerformanceSequences;
        int count = Math.Min(sequences.Length, performance.Length);

        int compared = 0, mismatches = 0, duplicates = 0;
        var seen = new HashSet<Part>();
        for (int k = 0; k < count; k++)
        {
            List<HashSet<Part>>? phaseParts = performance[k].PhaseEngineParts;
            if (phaseParts == null)
                continue;
            seen.Clear();
            foreach (HashSet<Part> parts in phaseParts)
            {
                foreach (Part part in parts)
                {
                    if (!seen.Add(part))
                        continue;
                    int game = 0;
                    for (int n = 0; n <= k && n < lists.Count; n++)
                        if (lists[n].Contains(part))
                            game++;
                    int mirror = KsaVehicleAdapter.RegistrationCount(sequences, k, part);
                    compared++;
                    if (game != mirror)
                    {
                        mismatches++;
                        t.Info($"sequence {k}: '{part.Id}' is listed {game} time(s) by the game and {mirror} by the mirror.");
                    }
                    if (game > 1)
                    {
                        duplicates++;
                        t.Info($"sequence {k}: '{part.Id}' is registered {game} times.");
                    }
                }
            }
        }
        t.Check("the drain simulation attributes engine parts to its phases", compared > 0, $"{compared} part(s)");
        t.Check("the mirror counts every engine part the way the game lists it", mismatches == 0,
            $"{mismatches} of {compared} differ");
        t.Info($"{duplicates} engine part registration(s) on '{vehicle.Id}' are duplicates.");
    }

    // A phase whose only part is listed twice is divided by two.
    private static void AUniformCountDividesThePhase(TestContext t, Vehicle vehicle)
    {
        Part? booster = FindEnginePart(vehicle, solid: true, out int number);
        if (booster == null)
        {
            t.Skip("no solid engine part with a sequence on this save.");
            return;
        }
        Sequence[] sequences = FakeSequences(vehicle, number, number);
        SequencePerformance perf = Phase(booster);

        double thrust = 2000.0, flow = 2.0, duration = 10.0;
        KsaVehicleAdapter.CorrectDuplicateRegistration(vehicle.Parts, sequences, perf, 0, 0, ref thrust, ref flow, ref duration);
        t.Check("a part listed once keeps its phase", thrust == 2000.0 && flow == 2.0 && duration == 10.0,
            $"{thrust:F1} N, {flow:F3} kg/s, {duration:F2} s");

        KsaVehicleAdapter.CorrectDuplicateRegistration(vehicle.Parts, sequences, perf, 1, 0, ref thrust, ref flow, ref duration);
        t.CheckRel("a solid listed twice gets half the thrust", thrust, 1000.0, 1e-12);
        t.CheckRel("a solid listed twice gets half the flow", flow, 1.0, 1e-12);
        t.CheckRel("a solid listed twice burns its propellant over twice the time", duration, 20.0, 1e-12);
    }

    // The liquid core listed once beside a solid booster listed twice: the core is taken out at its
    // design figures and the solid gets what is left.
    private static void MixedCountsTakeTheLiquidOutOnce(TestContext t, Vehicle vehicle)
    {
        Part? booster = FindEnginePart(vehicle, solid: true, out int boosterNumber);
        Part? core = FindEnginePart(vehicle, solid: false, out int coreNumber);
        if (booster == null || core == null || boosterNumber == coreNumber)
        {
            t.Skip("the save has no solid engine part and liquid engine part in different sequences.");
            return;
        }
        Sequence[] sequences = FakeSequences(vehicle, boosterNumber, boosterNumber, coreNumber);
        int last = sequences.Length - 1;
        int coreCount = KsaVehicleAdapter.RegistrationCount(sequences, last, core);
        int boosterCount = KsaVehicleAdapter.RegistrationCount(sequences, last, booster);
        if (coreCount != 1 || boosterCount != 2)
        {
            t.Skip($"the fake list registers the core {coreCount} and the booster {boosterCount} time(s), which is the save's shape rather than the code's.");
            return;
        }

        (float3 coreThrustVec, double coreFlow) = KsaVehicleAdapter.PartDesignPerformance(vehicle.Parts, core, 0.0);
        (float3 solidThrustVec, double solidFlow) = KsaVehicleAdapter.PartDesignPerformance(vehicle.Parts, booster, 0.0);
        double coreThrust = coreThrustVec.Length();
        if (!t.Check("both parts have design performance", coreThrust > 0.0 && coreFlow > 0.0
                && solidThrustVec.Length() > 0.0 && solidFlow > 0.0))
            return;
        // The model paces a solid from its grain, so its registered figures are not the design ones.
        double pacedThrust = 0.8 * solidThrustVec.Length(), pacedFlow = 0.8 * solidFlow;

        double thrust = coreThrust + 2.0 * pacedThrust;
        double flow = coreFlow + 2.0 * pacedFlow;
        double duration = 10.0;
        double burned = flow * duration;
        SequencePerformance perf = Phase(core, booster);
        KsaVehicleAdapter.CorrectDuplicateRegistration(vehicle.Parts, sequences, perf, last, 0, ref thrust, ref flow, ref duration);
        t.CheckRel("the corrected thrust counts the core once and the solid once", thrust, coreThrust + pacedThrust, 1e-9);
        t.CheckRel("the corrected flow counts the core once and the solid once", flow, coreFlow + pacedFlow, 1e-9);
        t.CheckRel("the propellant burned in the phase is unchanged", flow * duration, burned, 1e-9);
    }

    // The first engine part of the kind, with the sequence its engine belongs to.
    private static Part? FindEnginePart(Vehicle vehicle, bool solid, out int number)
    {
        foreach (Part part in vehicle.Parts.Parts)
        {
            if (!part.IsSequenceable || !HasCores(part, solid))
                continue;
            foreach (ISequenced module in part.GetSubtreeSequencedModules())
            {
                if (module.Sequence >= ISequenced.FirstSequence)
                {
                    number = module.Sequence;
                    return part;
                }
            }
        }
        number = 0;
        return null;
    }

    private static bool HasCores(Part part, bool solid)
    {
        bool any = false;
        foreach (EngineController engine in part.Modules.Get<EngineController>())
        {
            if (engine.Cores == null)
                continue;
            foreach (RocketCore core in engine.Cores)
            {
                if (core is SolidMotor != solid)
                    return false;
                any = true;
            }
        }
        return any;
    }

    private static Sequence[] FakeSequences(Vehicle vehicle, params int[] numbers)
    {
        var sequences = new Sequence[numbers.Length];
        for (int i = 0; i < numbers.Length; i++)
            sequences[i] = new Sequence(vehicle.Parts, numbers[i]);
        return sequences;
    }

    private static SequencePerformance Phase(params Part[] parts) => new()
    {
        PhaseEngineParts = new List<HashSet<Part>> { new HashSet<Part>(parts) },
    };
}

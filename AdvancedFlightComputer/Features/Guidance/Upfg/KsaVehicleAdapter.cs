#nullable disable

using System;
using System.Collections.Generic;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance.Upfg;

// Builds the UPFG stage list from KSA's own staging simulator.
//
// The game models staging through PartTree.PerformanceSequences. It holds one SequencePerformance per entry in SequenceList.Sequences and drives the in-game stage menu's delta-v and TWR readout. Its mole masses are seeded from the live tank and grain states, so every figure describes the remaining vehicle.
//
// ALWAYS START AT INDEX 0. The game's Recompute() seeds its simulated mole masses from the live tanks once and drains them forward through the list, so every index continues from the state the previous one left. In flight a sequence numbered below SequenceList.ActiveSequence registers no engines, so its entry carries no phases, and the first entry with phases is the burn in progress. Reading in order steps over those spent entries, and a later entry with nothing left to burn yields no usable phase, which the filter below drops.
//
// We consume SequencePerformance.Phases rather than the headline Thrust/Isp. A sequence is split into phases wherever the number of burning engines changes - solid boosters flaming out under a still-burning core, asparagus drops - and each phase is constant-thrust, so it maps one-to-one onto a UPFG Mode 1 stage. The headline Thrust/MassFlowRate are only phase 0's, and in flight mode they are the *live throttled* values for the active sequence while DeltaV still comes from the design-condition drain sim, so the two must never be combined.
//
// Solid boosters come out right for free. Their grain lives in SolidGrainSegment rather than a Tank (so a tank walk sees no propellant at all), part of it is permanently unburnable, and thrust follows the burning area over the burn. The game's sim handles all three, and it paces each solid at the mean mass flow and the mean thrust of the burn it has left, which it reads from SolidMotor.VacuumThrustProfile or AtmosphericThrustProfile by the sequence's environment.
//
// Pressure: the drain sim uses each sequence's own Environment setting (Vacuum = 0 Pa, Atmospheric = 101325 Pa), NOT the ambient pressure passed to RecomputeForFlight. ApplyAmbientPressure scales only the burning stage to the craft's pressure, because later stages burn where the model puts them. AnyAtmosphericSequence reports a sequence the player set to sea level so the UI can flag it.
public static class KsaVehicleAdapter
{
    private const double G0 = 9.80665;

    // Phases shorter than this are numerical dust from the drain simulation's fixed iteration budget; their mass change is still carried forward so the stage chain stays continuous.
    private const double MinPhaseSeconds = 1e-3;

    // ...and a phase can be long enough to survive that and still be dust: a tank trickling out its last few grams paces a "burn" of seconds that delivers no dV worth planning around. A stage under this is dropped rather than shown.
    private const double MinStageDv = 1.0;   // m/s

    private const double SeaLevelPressure = 101325.0;

    // ambientPressure is in Pa, the pressure the live nozzles run at. With a pressure grid, every stage also carries its full-throttle thrust at each grid pressure (see UpfgStage.ThrustAtPressure), scaled to the drain simulation's own figure at the sequence's model pressure so the duplicate-registration repair carries through.
    public static UpfgVehicle Build(Vehicle vehicle, double ambientPressure, double[] pressureGrid = null)
    {
        var result = new UpfgVehicle();

        PartTree tree = vehicle?.Parts;
        SequenceList sequenceList = tree?.SequenceList;
        SequencePerformanceList performanceList = tree?.PerformanceSequences;
        if (sequenceList == null || performanceList == null)
            return result;

        ReadOnlySpan<Sequence> sequences = sequenceList.Sequences;
        ReadOnlySpan<SequencePerformance> performance = performanceList.PerformanceSequences;
        int count = Math.Min(sequences.Length, performance.Length);
        if (count == 0)
            return result;

        UpfgStage burningStage = null;
        HashSet<Part> burningPhaseParts = null;
        double burningModelPressure = 0.0;

        for (int i = 0; i < count; i++)
        {
            SequencePerformance perf = performance[i];
            List<SequencePhaseInfo> phases = perf.Phases;
            if (phases == null)
                continue;

            // Each sequence's WetMass already accounts for the burns and the jettisoned structure of every sequence before it, so the running mass is re-seeded here rather than carried across the boundary.
            double mass = perf.WetMass;
            for (int j = 0; j < phases.Count; j++)
            {
                SequencePhaseInfo phase = phases[j];
                double thrust = phase.Thrust;
                double massFlow = phase.MassFlowRate;
                double duration = phase.Duration;
                if (!(thrust > 0.0) || !(massFlow > 0.0) || !(duration > 0.0))
                    continue;

                CorrectDuplicateRegistration(tree, sequences, perf, i, j, ref thrust, ref massFlow, ref duration);

                double burnout = mass - massFlow * duration;
                if (duration >= MinPhaseSeconds && mass > 0.0 && burnout > 0.0)
                {
                    var stage = new UpfgStage
                    {
                        Mode = 1,
                        Thrust = thrust,
                        Isp = thrust / (massFlow * G0),
                        MassTotal = mass,
                        MassDry = burnout,
                        GLim = 1e9,
                        Seq = i,
                        Engines = phase.ActiveEngineCount,
                    };
                    if (pressureGrid != null)
                        SampleThrustVsPressure(tree, stage, PhaseParts(perf, j), ModelPressure(sequences[i].Environment), pressureGrid);
                    result.Stages.Add(stage);
                    if (burningStage == null)
                    {
                        burningStage = stage;
                        burningPhaseParts = PhaseParts(perf, j);
                        burningModelPressure = ModelPressure(sequences[i].Environment);
                    }
                }
                mass = burnout;
            }
        }

        Coalesce(result);
        // Coalesce can drop the first stage, and the phase parts describe that stage only.
        if (burningStage != null && result.Stages.Count > 0 && ReferenceEquals(result.Stages[0], burningStage))
            ApplyAmbientPressure(tree, result, burningStage, burningPhaseParts, burningModelPressure, ambientPressure);
        return result;
    }

    // Fold the drain simulation's phase decomposition back into actual STAGES.
    //
    // A UPFG stage is a constant-thrust arc that ends in a discontinuity - a jettison, an engine set changing. The game's phases are not that: they are whatever intervals its drain simulation happened to break the burn into, and it breaks one wherever the number of drawing engine cores changes for even a single iteration. TANKS ARE WHAT DRIVES THAT. Its inner loop steps from one tank emptying to the next, splitting the demand across the tanks in a level and spilling what is left to the next level, so a stack with several tanks - a capsule tank plumbed to the stack, an asymmetric pair, anything that does not run dry at the same instant - takes several iterations to finish the burn, and any core that misses its full draw on one of them drops out and comes back. Each of those became a separate row in the stage table with its own slice of the stage's dV, which is what "one stage showing up as several" is.
    //
    // Two adjacent stages are the same stage if they have the same thrust and the same exhaust velocity and no mass went overboard between them. Merging is exact - dV is ve*ln(m0/m1), so ve*ln(m0/mid) + ve*ln(mid/m1) is the same number - and it leaves every real boundary (a booster drop, an engine cutting out, the g-limit split applied later) intact, because those all change thrust, Isp or mass.
    private static void Coalesce(UpfgVehicle vehicle)
    {
        List<UpfgStage> stages = vehicle.Stages;
        for (int i = stages.Count - 2; i >= 0; i--)
        {
            UpfgStage a = stages[i], b = stages[i + 1];
            if (!Close(a.Thrust, b.Thrust) || !Close(a.Isp, b.Isp))
                continue;
            // Mass continuity: anything jettisoned between the two is a real stage boundary however alike the two burns look. The tolerance is loose enough to absorb the float arithmetic the game's masses arrive in (10 kg on a 100 t stack) and far tighter than any real separation.
            if (Math.Abs(a.MassDry - b.MassTotal) > 1e-4 * Math.Max(a.MassTotal, 1.0))
                continue;

            a.MassDry = b.MassDry;
            a.Engines = Math.Max(a.Engines, b.Engines);
            stages.RemoveAt(i + 1);
        }

        // Whatever survives that and still carries no useful dV is numerical dust from the drain simulation's fixed iteration budget. Dropping it is safe: UPFG reads each stage's own four numbers and reconciles stage 0 against the live mass, so nothing downstream depends on the chain being gapless.
        for (int i = stages.Count - 1; i >= 0; i--)
        {
            UpfgStage s = stages[i];
            if (!(s.MassTotal > 0.0) || !(s.MassDry > 0.0) || s.MassDry >= s.MassTotal
                || s.Isp * G0 * Math.Log(s.MassTotal / s.MassDry) < MinStageDv)
                stages.RemoveAt(i);
        }
    }

    private static bool Close(double a, double b) =>
        Math.Abs(a - b) <= 1e-3 * Math.Max(Math.Abs(a), Math.Abs(b));

    // Adjusts a phase when KSA's drain simulation registers an engine part more than once.
    //
    // SequencePerformanceList.SnapshotSequenceParts lists a part under every sequence that a sequenced module of the part or of its sub-parts belongs to, and Recompute registers the engine controllers of every listed part for every sequence up to the one it computes, with no de-duplication.
    // A part with an engine in one sequence and another sequenced module in a later sequence is therefore registered twice from that later sequence on, as long as it is still attached.
    // Thrust and mass flow are multiplied and the propellant is not, so the mass ratio and the stock delta-v readout stay right while the thrust reads double and the burn time half, which are the two figures UPFG steers on.
    //
    // The count is taken the way the game lists the parts, so the repair is exact when every engine part of the phase is registered the same number of times, liquid or solid: the phase is divided by that count and its duration stretched to keep the propellant burned unchanged.
    // With mixed counts the liquid parts are taken out at their single registration, computed from their design conditions at the sequence's own pressure the way the model registered them, and the solid parts, whose flow the model paces from the grain, share what is left.
    // That leaves thrust and flow right and the phase boundaries approximate, because the model drained the doubled tanks at the wrong rate, and it stands down when the solids of one phase carry different counts.
    // The liquid figures count every core of a part, so a part with a core the model did not register is over-attributed, and the guard on the liquid total catches the large version.
    // Where KSA registers every part once, this is a no-op.
    internal static void CorrectDuplicateRegistration(
        PartTree tree, ReadOnlySpan<Sequence> sequences, SequencePerformance perf, int seqIdx, int phaseIdx,
        ref double thrust, ref double massFlow, ref double duration)
    {
        HashSet<Part> parts = PhaseParts(perf, phaseIdx);
        if (parts == null || parts.Count == 0)
            return;

        int first = 0, maxCount = 0;
        bool uniform = true;
        var registrations = new List<(Part part, int count)>(parts.Count);
        foreach (Part part in parts)
        {
            // A part the model burned was registered at least once, so a count the mirror cannot see leaves it alone.
            int count = Math.Max(1, RegistrationCount(sequences, seqIdx, part));
            registrations.Add((part, count));
            if (first == 0)
                first = count;
            uniform &= count == first;
            maxCount = Math.Max(maxCount, count);
        }
        if (maxCount <= 1)
            return;

        double burned = massFlow * duration;    // the one figure the model gets right
        if (uniform)
        {
            thrust /= first;
            massFlow /= first;
            duration = burned / massFlow;
            return;
        }

        double pressure = ModelPressure(sequences[seqIdx].Environment);
        double liquidThrustOnce = 0.0, liquidFlowOnce = 0.0, liquidThrustModel = 0.0, liquidFlowModel = 0.0;
        int solidCount = 0;
        foreach ((Part part, int count) in registrations)
        {
            if (HasSolidCore(part))
            {
                if (solidCount != 0 && count != solidCount)
                    return;
                solidCount = count;
                continue;
            }
            (float3 thrustVec, double flow) = PartDesignPerformance(tree, part, pressure);
            double force = thrustVec.Length();
            liquidThrustOnce += force;
            liquidFlowOnce += flow;
            liquidThrustModel += force * count;
            liquidFlowModel += flow * count;
        }
        // The liquids at their counts cannot exceed the model's phase, so a model that reads less than that is one the mirror does not describe.
        if (liquidFlowModel > massFlow * (1.0 + 1e-3))
            return;

        // What the liquids do not explain is the solids at their count, plus the difference between the model's vector sum and this scalar one.
        int divisor = Math.Max(solidCount, 1);
        double correctedThrust = liquidThrustOnce + (thrust - liquidThrustModel) / divisor;
        double correctedFlow = liquidFlowOnce + (massFlow - liquidFlowModel) / divisor;
        if (!(correctedThrust > 0.0) || !(correctedFlow > 0.0) || correctedFlow >= massFlow)
            return;

        thrust = correctedThrust;
        massFlow = correctedFlow;
        duration = burned / correctedFlow;
    }

    // How many times Recompute registers this part's engines for the sequence at seqIdx: once per sequence up to it that lists the part, which SnapshotSequenceParts decides by the sequenced modules of the part and its sub-parts.
    internal static int RegistrationCount(ReadOnlySpan<Sequence> sequences, int seqIdx, Part part)
    {
        if (part == null || !part.IsSequenceable)
            return 0;
        int count = 0;
        for (int n = 0; n <= seqIdx && n < sequences.Length; n++)
            if (part.HasSubtreeSequencedModule(sequences[n].Number))
                count++;
        return count;
    }

    private static bool HasSolidCore(Part part)
    {
        Span<EngineController> engines = part.Modules.Get<EngineController>();
        for (int i = 0; i < engines.Length; i++)
        {
            RocketCore[] cores = engines[i].Cores;
            if (cores == null)
                continue;
            for (int j = 0; j < cores.Length; j++)
                if (cores[j] is SolidMotor)
                    return true;
        }
        return false;
    }

    // The pressure the drain simulation registers a sequence's engines at.
    private static double ModelPressure(PerformanceEnvironment environment)
        => environment == PerformanceEnvironment.Atmospheric ? SeaLevelPressure : 0.0;

    private static void ApplyAmbientPressure(PartTree tree, UpfgVehicle result, UpfgStage stage,
                                             HashSet<Part> phaseParts, double modelPressure, double ambientPressure)
    {
        if (phaseParts == null || phaseParts.Count == 0
            || !double.IsFinite(ambientPressure) || ambientPressure < 0.0)
            return;
        if (Math.Abs(ambientPressure - modelPressure) <= 1.0)
            return;   // not worth two engine sums

        double ambientThrust = PhaseThrustAtPressure(tree, phaseParts, ambientPressure);
        double modelThrust = PhaseThrustAtPressure(tree, phaseParts, modelPressure);
        if (!(ambientThrust > 0.0) || !(modelThrust > 0.0))
            return;

        double ratio = ambientThrust / modelThrust;
        if (!double.IsFinite(ratio) || ratio <= 0.0)
            return;

        // Mass flow does not depend on back pressure (DeLavalNozzleConfig.ComputePerformance), so Isp moves with thrust and the masses stay.
        stage.Thrust *= ratio;
        stage.Isp *= ratio;
        result.BurningStageThrustRatio = ratio;
    }

    // The stage's thrust across a range of back pressures, as the ratio of the phase's design thrust at each pressure to its design thrust at the pressure the drain simulation used, times the stage's own thrust. A ratio rather than the raw engine sum so the figure agrees with Thrust at the model pressure exactly. Left unset when the phase's engines cannot be summed.
    private static void SampleThrustVsPressure(PartTree tree, UpfgStage stage, HashSet<Part> phaseParts,
                                               double modelPressure, double[] pressureGrid)
    {
        if (phaseParts == null || phaseParts.Count == 0)
            return;
        double modelThrust = PhaseThrustAtPressure(tree, phaseParts, modelPressure);
        if (!(modelThrust > 0.0))
            return;
        var thrust = new double[pressureGrid.Length];
        for (int g = 0; g < pressureGrid.Length; g++)
        {
            double ratio = PhaseThrustAtPressure(tree, phaseParts, pressureGrid[g]) / modelThrust;
            if (!double.IsFinite(ratio) || ratio <= 0.0)
                return;
            thrust[g] = stage.Thrust * ratio;
        }
        stage.PressureGrid = (double[])pressureGrid.Clone();
        stage.ThrustAtPressure = thrust;
    }

    // Design conditions, not live ones, make the ratio of two calls a pure pressure response, also for a solid part way through its grain.
    private static double PhaseThrustAtPressure(PartTree tree, HashSet<Part> parts, double pressure)
    {
        float3 thrustVec = float3.Zero;
        foreach (Part part in parts)
            thrustVec += PartDesignPerformance(tree, part, pressure).thrustVec;
        return thrustVec.Length();
    }

    // The part's engines at their design conditions and the given pressure, the way SequencePerformanceList.AccumulateCoreThrustAtPressure registers them.
    internal static (float3 thrustVec, double massFlow) PartDesignPerformance(PartTree tree, Part part, double pressure)
    {
        float3 thrustVec = float3.Zero;
        double massFlow = 0.0;
        if (tree?.RocketNozzles == null)
            return (thrustVec, massFlow);
        float ambient = (float)Math.Clamp(pressure, 0.0, float.MaxValue);
        Span<EngineController> engines = part.Modules.Get<EngineController>();
        for (int i = 0; i < engines.Length; i++)
        {
            RocketCore[] cores = engines[i].Cores;
            if (cores == null)
                continue;
            for (int j = 0; j < cores.Length; j++)
            {
                RocketNozzle[] nozzles = cores[j]?.Rocket?.Nozzles;
                if (nozzles == null)
                    continue;
                ref readonly RocketCoreConditions conditions = ref cores[j].DesignConditions;
                foreach (var nozzle in tree.RocketNozzles.GetModulesAndStates(nozzles.AsSpan()))
                {
                    RocketPerformance performance = nozzle.Module.ComputePerformance(in conditions, ambient).GetRocketPerformance();
                    thrustVec += performance.TotalThrust * nozzle.State.ThrustDirectionVehicleAsmb;
                    massFlow += performance.MassFlowRate;
                }
            }
        }
        return (thrustVec, massFlow);
    }

    // RunDrainSimulation fills PhaseEngineParts in step with Phases. It holds the engines the model burns, lit or not, and as a set it counts a double-registered engine once.
    private static HashSet<Part> PhaseParts(SequencePerformance perf, int phaseIdx)
    {
        List<HashSet<Part>> phaseParts = perf.PhaseEngineParts;
        return phaseParts != null && phaseIdx >= 0 && phaseIdx < phaseParts.Count ? phaseParts[phaseIdx] : null;
    }

    // True if any sequence is set to compute at sea level instead of in vacuum. That is a per-sequence player setting saved with the vehicle, so the mod surfaces it rather than overwriting it.
    public static bool AnyAtmosphericSequence(Vehicle vehicle)
    {
        SequenceList sequenceList = vehicle?.Parts?.SequenceList;
        if (sequenceList == null)
            return false;

        ReadOnlySpan<Sequence> sequences = sequenceList.Sequences;
        for (int i = 0; i < sequences.Length; i++)
            if (sequences[i].Environment == PerformanceEnvironment.Atmospheric)
                return true;
        return false;
    }

    // This returns the current start mass from the game's model, which is index 0's WetMass. The UI compares it with Vehicle.TotalMass because the values come from different code paths.
    public static double CurrentStageWetMass(Vehicle vehicle)
    {
        PartTree tree = vehicle?.Parts;
        SequencePerformanceList performanceList = tree?.PerformanceSequences;
        if (performanceList == null)
            return 0.0;

        ReadOnlySpan<SequencePerformance> performance = performanceList.PerformanceSequences;
        return performance.Length > 0 ? performance[0].WetMass : 0.0;
    }
}

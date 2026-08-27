using System;
using System.Collections.Generic;
using KSA;

namespace PoweredGuidance.Upfg;

// Builds the UPFG stage list from KSA's own staging simulator.
//
// Since the August 2026 build the game models staging itself: PartTree
// .PerformanceSequences holds one SequencePerformance per entry in
// SequenceList.Sequences (same index order) and is what drives the in-game stage
// menu's delta-v / TWR readout. For sequence k it jettisons the parts whose
// decoupler fires at k, burns with every still-attached engine ignited in
// sequences 0..k, and stops when the engines that the NEXT jettison will drop are
// spent — which is precisely a UPFG stage boundary. Its mole masses are seeded
// from the live tank/grain states, so every figure is "from here on".
//
// ALWAYS START AT INDEX 0. The game's Recompute() seeds its simulated mole
// masses from the live tanks at index 0 and then drains them forward, so index 0
// is the burn in progress and every later index continues from the state the
// previous one left. It is tempting to skip ahead to the entry matching
// SequenceList.ActiveSequence, but that entry's propellant has already been
// consumed by index 0's own simulation: a spent sequence survives in the list as
// long as any of its parts are still attached, which is the normal SRB layout
// (sequence 0 ignites core + boosters, sequence 1 drops the boosters, sequence 0
// stays because the core engine hangs off it). Skipping forward then reads a
// drained, fuel-less entry, yields no stages at all, and silently falls back to
// the single-stage live-engine model. Leftover spent sequences are harmless read
// in order — they simply produce zero-duration phases, which are filtered below.
//
// We consume SequencePerformance.Phases rather than the headline Thrust/Isp. A
// sequence is split into phases wherever the number of burning engines changes —
// solid boosters flaming out under a still-burning core, asparagus drops — and
// each phase is constant-thrust, so it maps one-to-one onto a UPFG Mode 1 stage.
// The headline Thrust/MassFlowRate are only phase 0's, and in flight mode they
// are the *live throttled* values for the active sequence while DeltaV still
// comes from the design-condition drain sim, so the two must never be combined.
//
// Solid boosters come out right for free. Their grain lives in SolidGrainSegment
// rather than a Tank (so a tank walk sees no propellant at all), part of it is
// permanently unburnable, and thrust follows the burning area over the burn —
// the game's sim handles all three, linearising the thrust curve to a constant
// mass flow of usable-grain/BurnSeconds with thrust scaled to preserve Isp.
//
// Pressure: the drain sim uses each sequence's own Environment setting (Vacuum =
// 0 Pa, Atmospheric = 101325 Pa), NOT the ambient pressure passed to
// RecomputeForFlight. Sequences default to Vacuum, which is what UPFG wants;
// AnyAtmosphericSequence reports the exception so the UI can flag it.
public static class KsaVehicleAdapter
{
    private const double G0 = 9.80665;

    // Phases shorter than this are numerical dust from the drain simulation's
    // fixed iteration budget; their mass change is still carried forward so the
    // stage chain stays continuous.
    private const double MinPhaseSeconds = 1e-3;

    /// <summary>
    /// How far past the engines' own capability a phase's mass flow has to be before
    /// <see cref="CorrectDuplicateRegistration"/> treats it as double-counted. The
    /// real fault is a whole multiple — x2, x3 — so this sits well above anything a
    /// modelling difference could produce and well below the smallest real case.
    /// </summary>
    private const double DuplicateFlowRatio = 1.5;

    // ...and a phase can be long enough to survive that and still be dust: a tank
    // trickling out its last few grams paces a "burn" of seconds that delivers no
    // dV worth planning around. A stage under this is dropped rather than shown.
    private const double MinStageDv = 1.0;   // m/s

    public static UpfgVehicle Build(Vehicle vehicle)
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

        for (int i = 0; i < count; i++)
        {
            SequencePerformance perf = performance[i];
            List<SequencePhaseInfo> phases = perf.Phases;
            if (phases == null)
                continue;

            // Each sequence's WetMass already accounts for the burns and the
            // jettisoned structure of every sequence before it, so the running
            // mass is re-seeded here rather than carried across the boundary.
            double mass = perf.WetMass;
            for (int j = 0; j < phases.Count; j++)
            {
                SequencePhaseInfo phase = phases[j];
                double thrust = phase.Thrust;
                double massFlow = phase.MassFlowRate;
                double duration = phase.Duration;
                if (!(thrust > 0.0) || !(massFlow > 0.0) || !(duration > 0.0))
                    continue;

                // Cross-check the model against the engines that are actually in this
                // phase, and believe them (see CorrectDuplicateRegistration).
                CorrectDuplicateRegistration(perf, i, j, sequences[i].Environment,
                                             ref thrust, ref massFlow, ref duration);

                double burnout = mass - massFlow * duration;
                if (duration >= MinPhaseSeconds && mass > 0.0 && burnout > 0.0)
                {
                    result.Stages.Add(new UpfgStage
                    {
                        Mode = 1,
                        Thrust = thrust,
                        Isp = thrust / (massFlow * G0),
                        MassTotal = mass,
                        MassDry = burnout,
                        GLim = 1e9,
                        Seq = i,
                        Engines = phase.ActiveEngineCount,
                    });
                }
                mass = burnout;
            }
        }

        CorrectBurningSolids(vehicle, result);
        Coalesce(result);
        return result;
    }

    // Fold the drain simulation's phase decomposition back into actual STAGES.
    //
    // A UPFG stage is a constant-thrust arc that ends in a discontinuity — a jettison,
    // an engine set changing. The game's phases are not that: they are whatever
    // intervals its drain simulation happened to break the burn into, and it breaks
    // one wherever the number of drawing engine cores changes for even a single
    // iteration. TANKS ARE WHAT DRIVES THAT. Its inner loop steps from one tank
    // emptying to the next, splitting the demand across the tanks in a level and
    // spilling what is left to the next level, so a stack with several tanks — a
    // capsule tank plumbed to the stack, an asymmetric pair, anything that does not
    // run dry at the same instant — takes several iterations to finish the burn, and
    // any core that misses its full draw on one of them drops out and comes back.
    // Each of those became a separate row in the stage table with its own slice of the
    // stage's dV, which is what "one stage showing up as several" is.
    //
    // Two adjacent stages are the same stage if they have the same thrust and the same
    // exhaust velocity and no mass went overboard between them. Merging is exact —
    // dV is ve·ln(m0/m1), so ve·ln(m0/mid) + ve·ln(mid/m1) is the same number — and it
    // leaves every real boundary (a booster drop, an engine cutting out, the g-limit
    // split applied later) intact, because those all change thrust, Isp or mass.
    private static void Coalesce(UpfgVehicle vehicle)
    {
        List<UpfgStage> stages = vehicle.Stages;
        for (int i = stages.Count - 2; i >= 0; i--)
        {
            UpfgStage a = stages[i], b = stages[i + 1];
            if (!Close(a.Thrust, b.Thrust) || !Close(a.Isp, b.Isp))
                continue;
            // Mass continuity: anything jettisoned between the two is a real stage
            // boundary however alike the two burns look. The tolerance is loose
            // enough to absorb the float arithmetic the game's masses arrive in
            // (10 kg on a 100 t stack) and far tighter than any real separation.
            if (Math.Abs(a.MassDry - b.MassTotal) > 1e-4 * Math.Max(a.MassTotal, 1.0))
                continue;

            a.MassDry = b.MassDry;
            a.Engines = Math.Max(a.Engines, b.Engines);
            stages.RemoveAt(i + 1);
        }

        // Whatever survives that and still carries no useful dV is numerical dust from
        // the drain simulation's fixed iteration budget. Dropping it is safe: UPFG
        // reads each stage's own four numbers and reconciles stage 0 against the live
        // mass, so nothing downstream depends on the chain being gapless.
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

    /// <summary>
    /// Repairs a phase whose engines KSA's drain simulation counted more than once.
    ///
    /// SINCE MODULE-LEVEL SEQUENCING (KSA 2026.8.22) A PART CAN BE IN SEVERAL
    /// SEQUENCES. SnapshotSequenceParts used to map one part to one sequence:
    ///
    ///     if (part.Sequenceable &amp;&amp; _sequenceIdxByNumber.TryGetValue(part.Sequence, ...))
    ///         _sequencePartsScratch[value].Add(part);
    ///
    /// It now walks each part's sequenced MODULES and adds the part to every sequence
    /// any of them belongs to. A part carrying an engine module in one sequence and a
    /// decoupler module in another therefore appears in two lists — and Recompute's
    /// registration loop, which is unchanged, walks sequences 0..k and registers every
    /// EngineController of every part it finds, with no de-duplication:
    ///
    ///     for (int n = 0; n &lt;= k; n++)
    ///         foreach (Part item in _sequencePartsScratch[n])
    ///             ... RegisterDrainCore(item, core2, ...)
    ///
    /// So that part's cores are registered once per list it appears in. Thrust and
    /// mass flow are both multiplied; the MASS RATIO is not, because the same
    /// propellant is still drained — just faster. Which is why the stock delta-v
    /// readout looks correct while thrust reads double and the burn time reads half,
    /// and those two are exactly what UPFG steers on. (The same shape of error as the
    /// burning-solid pacing below, from an unrelated cause.)
    ///
    /// The repair takes the engines the game itself says are in this phase —
    /// PhaseEngineParts is a HashSet, so it holds each part ONCE however many times it
    /// was registered — and sums their vacuum capability. If the model claims
    /// materially more than those engines can produce, the measured figures replace
    /// it and the duration is stretched to keep the propellant burned unchanged.
    ///
    /// Deliberately conservative. It only ever reduces thrust, it needs a discrepancy
    /// far larger than modelling noise (duplication is a factor of two or more), and
    /// it stands down where the comparison is not like-for-like: an atmospheric
    /// sequence is not computed at vacuum, and a solid is deliberately paced away from
    /// its design flow. Where KSA is behaving, this is a no-op.
    /// </summary>
    private static void CorrectDuplicateRegistration(
        SequencePerformance perf, int seqIdx, int phaseIdx, PerformanceEnvironment environment,
        ref double thrust, ref double massFlow, ref double duration)
    {
        if (environment != PerformanceEnvironment.Vacuum)
            return;                             // model is at sea level; VacuumData is not

        List<HashSet<Part>> phaseParts = perf.PhaseEngineParts;
        if (phaseParts == null || phaseIdx >= phaseParts.Count)
            return;
        HashSet<Part> parts = phaseParts[phaseIdx];
        if (parts == null || parts.Count == 0)
            return;

        double realThrust = 0.0, realFlow = 0.0;
        foreach (Part part in parts)
        {
            Span<EngineController> engines = part.Modules.Get<EngineController>();
            for (int i = 0; i < engines.Length; i++)
            {
                // A solid's modelled flow is paced from the grain remaining, not its
                // design figure, so it is not comparable — leave the whole phase to
                // CorrectBurningSolids rather than half-correcting it here.
                RocketCore[] cores = engines[i].Cores;
                for (int j = 0; j < cores.Length; j++)
                    if (cores[j] is SolidMotor)
                        return;

                realThrust += engines[i].VacuumData.ThrustMax.Length();
                realFlow += engines[i].VacuumData.MassFlowRateMax;
            }
        }
        if (!(realThrust > 0.0) || !(realFlow > 0.0))
            return;

        // Duplication is x2 at least. Anything smaller is the difference between a
        // vector thrust sum and a scalar one, or a throttle, and is not ours to touch.
        if (massFlow < realFlow * DuplicateFlowRatio)
            return;

        double burned = massFlow * duration;    // the one figure the model gets right
        thrust = realThrust;
        massFlow = realFlow;
        duration = burned / realFlow;
    }

    // Repairs the one place the game's staging model is wrong for a solid that is
    // already burning.
    //
    // SequencePerformanceList.ComputeSolidPacingMassFlowRate paces a solid at
    // (usable grain REMAINING) / (BurnSeconds of a FULL grain), and scales its
    // thrust by the same ratio to preserve exhaust velocity. At ignition that is
    // exact. Part-way through it is not: with a fraction f of the grain left the
    // model reports f x the true thrust and mass flow, and — because the burn
    // time it implies is remaining/(remaining/BurnSeconds) — predicts a further
    // FULL BurnSeconds of burn no matter how little grain is left.
    //
    // The mass ratio survives that (f cancels), which is why the stock stage
    // menu's delta-v looks right, but thrust and burn time are exactly what UPFG
    // steers on — hence the visible attitude jump when the boosters finally go.
    //
    // The fix takes the solid's LIVE nozzle performance as truth (solids have
    // MinimumThrottle = 1, so live is full-throttle by definition — no throttle
    // contamination), works out what the model contributed, and swaps one for the
    // other in the stage that is burning now. Solids that are attached but not yet
    // lit have a full grain, so f = 1 and this is a no-op for them.
    private static void CorrectBurningSolids(Vehicle vehicle, UpfgVehicle result)
    {
        if (result.Stages.Count == 0)
            return;

        PartTree tree = vehicle.Parts;
        if (tree?.Moles == null || tree.RocketNozzles == null)
            return;
        ReadOnlySpan<MoleState> moles = tree.Moles.States;

        double deltaThrust = 0.0, deltaFlow = 0.0;
        double solidBurnLeft = double.PositiveInfinity;

        // We want the preview's BurnSeconds, not the curve. As of KSA 2026.8.19 the samples
        // are a ThrustCurveSamples of three parallel spans rather than one Span<float>,
        // and an EMPTY one is explicitly valid (IsValid short-circuits on IsEmpty) — the
        // resample loop at the end of TrySampleThrustCurve simply runs zero times while
        // the preview is still filled in. So ask for no samples at all and skip the
        // buffer entirely, rather than stackallocing one we never read.
        var curve = new SolidMotor.ThrustCurveSamples
        {
            ThrustNewtons = default,
            IspSeconds = default,
            ChamberPressurePascals = default,
        };

        Span<EngineController> engines = tree.Modules.Get<EngineController>();
        for (int i = 0; i < engines.Length; i++)
        {
            if (!engines[i].IsActive)
                continue;
            RocketCore[] cores = engines[i].Cores;
            for (int j = 0; j < cores.Length; j++)
            {
                if (cores[j] is not SolidMotor solid || solid.Rocket == null || !solid.Stack.IsValid)
                    continue;

                // What this motor is actually doing right now.
                double liveThrust = 0.0, liveFlow = 0.0;
                var nozzles = tree.RocketNozzles.GetModulesAndStates(solid.Rocket.Nozzles.AsSpan());
                foreach (var nozzle in nozzles)
                {
                    liveThrust += nozzle.State.Performance.TotalThrust;
                    liveFlow += nozzle.State.Performance.MassFlowRate;
                }
                if (liveThrust <= 0.0 || liveFlow <= 0.0)
                    continue;   // mid-transient; leave the model alone this step

                // Grain left that can actually burn — the residue never does.
                double usable = 0.0;
                SolidGrainSegment[] segments = solid.Stack.Segments;
                for (int k = 0; k < segments.Length; k++)
                {
                    Mole grain = segments[k].Grain;
                    if (grain != null)
                        usable += Math.Max(0.0,
                            moles[grain.StatesIdx].Mass - segments[k].UnburnableGrainMass);
                }
                if (usable <= 0.0)
                    continue;

                if (!solid.TrySampleThrustCurve(curve, out SolidMotor.ThrustCurvePreview preview)
                    || preview.BurnSeconds <= 0.0f)
                    continue;

                // The pacing the model used, and the exhaust velocity it kept.
                double modelFlow = usable / preview.BurnSeconds;
                double exhaustVel = liveThrust / liveFlow;

                deltaFlow += liveFlow - modelFlow;
                deltaThrust += exhaustVel * (liveFlow - modelFlow);
                solidBurnLeft = Math.Min(solidBurnLeft, usable / liveFlow);
            }
        }

        if (deltaFlow == 0.0 && double.IsPositiveInfinity(solidBurnLeft))
            return;

        UpfgStage stage = result.Stages[0];
        double modelStageFlow = stage.Thrust / (stage.Isp * G0);
        double thrust = stage.Thrust + deltaThrust;
        double flow = modelStageFlow + deltaFlow;
        if (!(thrust > 0.0) || !(flow > 0.0))
            return;

        // The model's own phase length still bounds us — whatever else runs dry
        // first (a core tank) is unaffected by the solid's pacing.
        double duration = (stage.MassTotal - stage.MassDry) / modelStageFlow;
        if (!double.IsPositiveInfinity(solidBurnLeft))
            duration = Math.Min(duration, solidBurnLeft);

        double burnout = stage.MassTotal - flow * duration;
        if (!(burnout > 0.0) || burnout >= stage.MassTotal)
            return;

        // Propellant the model burned during a stage that is now shorter is still
        // aboard. It is whatever the non-solid engines would not have consumed,
        // and it lives in the tanks the next stage burns to depletion — so hand it
        // to that stage's start mass and leave its burnout mass alone.
        double restored = burnout - stage.MassDry;
        if (restored > 0.0 && result.Stages.Count > 1)
            result.Stages[1].MassTotal += restored;

        stage.Thrust = thrust;
        stage.Isp = thrust / (flow * G0);
        stage.MassDry = burnout;
    }

    // True if any sequence is set to compute at sea level instead of in vacuum.
    // That is a per-sequence player setting saved with the vehicle, so the mod
    // surfaces it rather than overwriting it.
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

    // Start mass the game's model believes the vehicle currently has — index 0's
    // WetMass, i.e. the burn in progress. Compared against Vehicle.TotalMass in
    // the UI: the two are computed by different code paths and any persistent gap
    // is worth seeing, though UPFG reconciles stage 0 against the live mass each
    // step regardless.
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

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
                    });
                }
                mass = burnout;
            }
        }

        CorrectBurningSolids(vehicle, result);
        return result;
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

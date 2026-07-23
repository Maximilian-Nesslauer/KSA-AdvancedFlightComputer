using AdvancedFlightComputer.Features.MultiPass;
using Brutal.Numerics;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates SequenceBurnState (AFC's staged-dV analyzer) against the game's own
// SequencePerformanceList for real vehicles. The oracle is always the game's own
// per-sequence numbers, never a re-derivation: for each Vacuum sequence, AFC's burnable
// fuel, mass flow and exhaust velocity must match the game's BurnedFuelMass /
// MassFlowRate / (Thrust / MassFlowRate). It also exercises the two fuel-accounting
// fixes: toggling a tank's PropellantUseEnabled and setting a non-default engine
// FlowRule must keep AFC in agreement with the game.
//
// AFC's SequenceBurnState is vacuum-only (it reads EngineController.VacuumData). The game
// evaluates each sequence at its own PerformanceEnvironment, except the active sequence,
// which RecomputeForFlight(0f) pins to 0 Pa. So a sequence is comparable when it is Vacuum
// or is the active sequence; a non-active Atmospheric sequence is computed at sea level and
// skipped. Stock's side is computed on a private SequencePerformanceList so the test never
// touches the game's shared, worker-thread-recomputed instance.
//
// Start mass legitimately differs and is logged, not asserted: AFC uses Vehicle.TotalMass
// (the real physics mass) while SequencePerformanceList.WetMass sums only top-level
// InertMass and omits sub-part inert mass, so the game's start mass runs a little light.
public sealed class SequenceBurnStateTest : IHarnessTest
{
    private const double SpawnAltitudeM = 500_000.0; // an arbitrary stable orbit; altitude does not feed the compared thrust
    private const double MDotTol = 5e-3;             // relative; the same nozzle data feeds both sides
    private const double VeTol = 5e-3;
    private const double FuelTol = 1e-2;
    private const double MinDropKg = 1.0;            // a disabled fuel tank must remove at least this much

    // Vehicles shipped in Content/Core/defaultvehicles, so this test needs no machine-specific save.
    private static readonly string[] DefaultVehicles = { "Rocket", "Gemini7", "Polaris", "Hunter", "Banjo" };
    private const string MultiStageVehicle = "Rocket"; // known multi-stage; carries the fix sub-cases

    public string Name => "afc-sequence-burnstate";

    public int Run(HeadlessSession session)
    {
        if (!ManeuverTestSupport.RequireHome(Name, session, out IParentBody home))
            return 1;

        SimDriver driver = session.CreateDriver();
        bool ok = true;
        bool any = false;

        foreach (string saveId in DefaultVehicles)
        {
            VehicleSave? save = DefaultVehicleSaves.FindSave(saveId);
            if (save == null)
            {
                HarnessLog.Line($"[{Name}] '{saveId}': not shipped, skipping.");
                continue;
            }
            any = true;
            ok &= WithVehicle(session, driver, home, save, v =>
                CrossCheck($"{saveId} baseline", v, SequenceBurnState.Analyze(v)));
        }

        if (!any)
        {
            HarnessLog.Line($"[{Name}] SKIP: no shipped default vehicle available.");
            return 0;
        }

        if (DefaultVehicleSaves.FindSave(MultiStageVehicle) is VehicleSave rocket)
        {
            ok &= WithVehicle(session, driver, home, rocket, DisabledTankCase);
            ok &= WithVehicle(session, driver, home, rocket, FlowRuleCase);
        }

        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    // Spawns a fresh copy of a default save, steps twice so the game initializes nozzle
    // states (thrust direction) and mass, runs the body, and always despawns.
    private bool WithVehicle(HeadlessSession session, SimDriver driver, IParentBody home,
        VehicleSave save, Func<Vehicle, bool> body)
    {
        Vehicle vehicle = Spawn(session, home, save, $"BurnState_{save.Id}");
        try
        {
            driver.Step(1e-3, 2);
            return body(vehicle);
        }
        finally
        {
            VehicleSpawner.Despawn(vehicle);
        }
    }

    // Replicates VehicleSpawner.SpawnFromSave, but sourced from DefaultVehicleSaves (the shipped
    // defaults) instead of the machine-specific Vehicles folder. VehicleSaveData is populated in the
    // UncompressedVehicleSave constructor, so no GPU Load is needed.
    private static Vehicle Spawn(HeadlessSession session, IParentBody home, VehicleSave save, string id)
    {
        PartInstance design = save.VehicleSaveData.RootPartInstance
            ?? throw new InvalidOperationException($"default vehicle '{save.Id}' has no root part instance.");
        PartTree tree = PartTree.Deserialize(design);
        SimTime now = Universe.GetElapsedSimTime();
        Orbit orbit = VehicleSpawner.CircularCci(home, home.MeanRadius + SpawnAltitudeM, now);
        Vehicle vehicle = Vehicle.CreateVehicle(
            session.System, doubleQuat.Identity, double3.Zero, home, id, tree.Root, orbit);
        vehicle.Parts.SequenceList.SetActiveSequence(save.VehicleSaveData.ActiveSequence);
        vehicle.Parts.SequenceList.ApplyEnvironments(save.VehicleSaveData.SequenceEnvironments);
        vehicle.Parts.FuelLinks.ApplySaveData(save.VehicleSaveData.FuelLinks, design);
        home.Children.Add(vehicle);
        return vehicle;
    }

    // Per-sequence agreement of an AFC analysis against a fresh, vacuum stock recompute.
    private bool CrossCheck(string label, Vehicle vehicle, SequenceBurnState afc)
    {
        if (afc.Sequences.Count == 0)
        {
            HarnessLog.Line($"[{Name}] {label}: no usable sequences, skipping.");
            return true;
        }

        SequencePerformanceList stock = new SequencePerformanceList(vehicle.Parts);
        stock.RecomputeForFlight(0f);

        // PerformanceSequences is index-aligned with SequenceList.Sequences; map by Number to line up
        // with AFC's firing-order list (AFC drops sequences with no usable engines).
        ReadOnlySpan<Sequence> seqs = vehicle.Parts.SequenceList.Sequences;
        ReadOnlySpan<SequencePerformance> perf = stock.PerformanceSequences;
        int count = Math.Min(seqs.Length, perf.Length);

        // The game evaluates a sequence at 0 Pa when it is Vacuum, or when it is the active sequence
        // (RecomputeForFlight(0f) forces the active sequence's flight pressure to 0). A non-active
        // Atmospheric sequence is computed at sea level, which AFC's vacuum-only analyzer cannot
        // match, so mark it not-comparable.
        int activeSeq = vehicle.Parts.SequenceList.ActiveSequence;
        var byNumber = new Dictionary<int, (SequencePerformance Perf, bool SeaLevel)>(count);
        for (int i = 0; i < count; i++)
        {
            bool seaLevel = seqs[i].Environment == PerformanceEnvironment.Atmospheric && seqs[i].Number != activeSeq;
            byNumber[seqs[i].Number] = (perf[i], seaLevel);
        }

        bool ok = true;
        foreach (SequenceInfo s in afc.Sequences)
        {
            if (!byNumber.TryGetValue(s.Number, out (SequencePerformance Perf, bool SeaLevel) match))
            {
                HarnessLog.Line($"[{Name}] {label} seq {s.Number}: no matching stock sequence => FAIL");
                ok = false;
                continue;
            }
            if (match.SeaLevel)
            {
                // The game computes this non-active Atmospheric sequence at sea level; AFC is
                // vacuum-only, so they are not comparable here.
                HarnessLog.Line($"[{Name}] {label} seq {s.Number}: non-active Atmospheric sequence, AFC is vacuum-only, skipping.");
                continue;
            }

            SequencePerformance p = match.Perf;
            double stockVe = p.MassFlowRate > 0f ? p.Thrust / p.MassFlowRate : 0.0;
            bool fuelOk = Near(s.FuelMassKg, p.BurnedFuelMass, FuelTol);
            bool mDotOk = Near(s.MassFlowKgPerSec, p.MassFlowRate, MDotTol);
            bool veOk = Near(s.ExhaustVelocityMs, stockVe, VeTol);
            bool seqOk = fuelOk && mDotOk && veOk;
            ok &= seqOk;
            HarnessLog.Line($"[{Name}] {label} seq {s.Number}: " +
                $"fuel {s.FuelMassKg:F1}/{p.BurnedFuelMass:F1} mDot {s.MassFlowKgPerSec:F2}/{p.MassFlowRate:F2} " +
                $"Ve {s.ExhaustVelocityMs:F1}/{stockVe:F1} start {s.StartMassKg:F1}/{p.WetMass:F1} " +
                $"=> {TestSupport.Verdict(seqOk)}");
        }
        return ok;
    }

    // A propellant-disabled tank must drop out of the burnable fuel, and AFC must still match the
    // game's BurnedFuelMass afterwards (the game skips PropellantUseEnabled=false tanks too).
    private bool DisabledTankCase(Vehicle vehicle)
    {
        double fuelBefore = TotalFuel(SequenceBurnState.Analyze(vehicle));

        Tank? target = PickLargestEnabledTank(vehicle);
        if (target == null)
        {
            HarnessLog.Line($"[{Name}] {MultiStageVehicle} disabled-tank: no enabled fuel tank, skipping.");
            return true;
        }
        double tankMass = target.ComputeSubstanceMass(vehicle.Parts.Moles.States);
        target.PropellantUseEnabled = false;

        SequenceBurnState after = SequenceBurnState.Analyze(vehicle);
        double fuelAfter = TotalFuel(after);
        bool dropped = fuelAfter < fuelBefore - MinDropKg;
        bool agree = CrossCheck($"{MultiStageVehicle} disabled-tank", vehicle, after);
        bool ok = dropped && agree;
        HarnessLog.Line($"[{Name}] {MultiStageVehicle} disabled-tank: disabled tank {target.InstanceId} " +
            $"({tankMass:F1}kg); AFC burnable {fuelBefore:F1} -> {fuelAfter:F1} (dropped={dropped}) " +
            $"=> {TestSupport.Verdict(ok)}");
        return ok;
    }

    // With a non-default (cross-stage) FlowRule, AFC must read the same ConsumptionOrder the game
    // drains from, so the per-sequence numbers stay in agreement.
    private bool FlowRuleCase(Vehicle vehicle)
    {
        const FlowRule rule = FlowRule.FurtherestToNearest; // default engine rule is FurtherestToNearestSameStage
        int changed = 0;
        ReadOnlySpan<Part> parts = vehicle.Parts.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            Span<EngineController> engines = parts[i].Modules.Get<EngineController>();
            for (int e = 0; e < engines.Length; e++)
            {
                foreach (RocketCore core in engines[e].Cores)
                {
                    if (core.ResourceManager == null)
                        continue;
                    core.ResourceManager.FlowRule = rule;
                    changed++;
                }
            }
        }
        if (changed == 0)
        {
            HarnessLog.Line($"[{Name}] {MultiStageVehicle} flow-rule: no engine cores, skipping.");
            return true;
        }

        bool ok = CrossCheck($"{MultiStageVehicle} flow-rule {rule}", vehicle, SequenceBurnState.Analyze(vehicle));
        HarnessLog.Line($"[{Name}] {MultiStageVehicle} flow-rule: set {rule} on {changed} core(s) => {TestSupport.Verdict(ok)}");
        return ok;
    }

    private static double TotalFuel(SequenceBurnState state)
    {
        double total = 0.0;
        foreach (SequenceInfo s in state.Sequences)
            total += s.FuelMassKg;
        return total;
    }

    private static Tank? PickLargestEnabledTank(Vehicle vehicle)
    {
        ReadOnlySpan<MoleState> moles = vehicle.Parts.Moles.States;
        Span<Tank> tanks = vehicle.Parts.Tanks.Modules;
        Tank? best = null;
        double bestMass = 0.0;
        for (int i = 0; i < tanks.Length; i++)
        {
            Tank tank = tanks[i];
            if (!tank.PropellantUseEnabled)
                continue;
            double mass = tank.ComputeSubstanceMass(moles);
            if (mass > bestMass)
            {
                bestMass = mass;
                best = tank;
            }
        }
        return best;
    }

    private static bool Near(double actual, double expected, double relTol)
    {
        double denom = Math.Max(Math.Abs(expected), 1e-6);
        return Math.Abs(actual - expected) <= relTol * denom;
    }
}

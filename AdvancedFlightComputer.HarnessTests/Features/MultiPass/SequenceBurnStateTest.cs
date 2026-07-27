using System.IO;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates SequenceBurnState (AFC's staged-dV analyzer) against the game's own
// SequencePerformanceList for real vehicles. The oracle is always the game's own
// per-sequence numbers, never a re-derivation: for each Vacuum sequence, AFC's burnable
// fuel, mass flow and exhaust velocity must match the game's BurnedFuelMass /
// MassFlowRate / (Thrust / MassFlowRate). It also covers the two fuel-accounting edge
// cases: toggling a tank's PropellantUseEnabled and setting a non-default engine
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
public sealed class SequenceBurnStateTest : AfcTest
{
    private const double SpawnAltitudeM = 500_000.0; // an arbitrary stable orbit; altitude does not feed the compared thrust
    private const double MDotTol = 5e-3;             // relative; the same nozzle data feeds both sides
    private const double VeTol = 5e-3;
    private const double FuelTol = 1e-2;
    private const double ElementFloor = 1e-6;        // keeps a near-zero expectation off bit-exactness
    private const double MinDropKg = 1.0;            // a disabled fuel tank must remove at least this much

    // Vehicles shipped in Content/Core/defaultvehicles, so this test needs no machine-specific save.
    private static readonly string[] DefaultVehicles = { "Rocket", "Gemini7", "Polaris", "Hunter", "Banjo" };
    private const string MultiStageVehicle = "Rocket"; // known multi-stage; carries the fix sub-cases

    public override string Name => "afc-sequence-burnstate";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        SimDriver driver = t.Session.CreateDriver();

        foreach (string saveId in DefaultVehicles)
        {
            VehicleSave? save = DefaultVehicleSaves.FindSave(saveId);
            if (save == null)
            {
                t.Skip($"'{saveId}': not shipped.");
                continue;
            }
            WithVehicle(t, driver, home, save.VehicleSaveData, v =>
                CrossCheck(t, $"{saveId} baseline", v, SequenceBurnState.Analyze(v)));
        }

        // The shipped defaults are all liquid, so a synthetically-built single-SRB vehicle is the
        // only coverage of the SolidMotor fuel and jettison-mass path. It is built from Core content,
        // so it is always available (no machine-specific save, unlike the flight test).
        WithVehicle(t, driver, home, BuildSyntheticSrb(), v =>
            CrossCheck(t, "SRB baseline", v, SequenceBurnState.Analyze(v)));

        if (DefaultVehicleSaves.FindSave(MultiStageVehicle) is VehicleSave rocket)
        {
            WithVehicle(t, driver, home, rocket.VehicleSaveData, v => DisabledTankCase(t, v));
            WithVehicle(t, driver, home, rocket.VehicleSaveData, v => FlowRuleCase(t, v));
        }
    }

    // Spawns a fresh copy of a default save, steps twice so the game initializes nozzle
    // states (thrust direction) and mass, runs the body, and always despawns.
    private static void WithVehicle(
        TestContext t, SimDriver driver, IParentBody home, VehicleSaveData data, Action<Vehicle> body)
    {
        Vehicle vehicle = VehicleFixtures.SpawnFromSaveData(
            t.System, home, data, $"BurnState_{data.Id}",
            OrbitFixtures.CircularAt(home, SpawnAltitudeM, Universe.GetElapsedSimTime()));
        try
        {
            driver.Step(1e-3, 2);
            body(vehicle);
        }
        finally
        {
            VehicleSpawner.Despawn(vehicle);
        }
    }

    // A one-part solid booster, deserialized exactly like a real vehicle.xml (VehicleSaveData.LoadFrom)
    // so the game builds the full part - RocketEngineController wrapping the SolidMotor core, which
    // feeds its own grain segment, no cross-part connectors required. This is enough for the stack to
    // resolve, the grain to fill, and both AFC and SequencePerformanceList to score the SRB sequence.
    private static VehicleSaveData BuildSyntheticSrb()
    {
        // GrainGeometryLibrary.LoadAll runs in Program.Main, which the headless bring-up exits before,
        // so the library is empty here even though its templates loaded with Core content. Instantiate
        // them once so SolidGrainSegment.CreateComponents can resolve the default geometry.
        if (GrainGeometryLibrary.All().IsEmpty)
            GrainGeometryLibrary.LoadAll();

        const string xml =
            """
            <?xml version="1.0" encoding="utf-8"?>
            <VehicleSaveData Id="SrbSynthetic" ActiveSequence="0">
              <RootPartRef InstanceOf="CorePropulsionA_Prefab_SRBA1" LocalInstanceId="1">
                <Transform>
                  <Position X="0" Y="0" Z="0" />
                  <Rotation X="0" Y="0" Z="0" />
                  <Scale X="1" Y="1" Z="1" />
                </Transform>
              </RootPartRef>
            </VehicleSaveData>
            """;
        using var reader = new StringReader(xml);
        var data = (VehicleSaveData)DefaultVehicleSaves.VehicleSerializer.Deserialize(reader)!;
        // Matches VehicleSaveData.LoadFrom, which passes Mod.Empty (= new Mod()); the field itself
        // is not visible to this assembly, so construct the equivalent empty mod directly.
        data.OnDataLoad(new KSA.Mod());
        return data;
    }

    // Per-sequence agreement of an AFC analysis against a fresh, vacuum stock recompute.
    private static void CrossCheck(TestContext t, string label, Vehicle vehicle, SequenceBurnState afc)
    {
        if (afc.Sequences.Count == 0)
        {
            t.Skip($"{label}: no usable sequences.");
            return;
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

        foreach (SequenceInfo s in afc.Sequences)
        {
            if (!byNumber.TryGetValue(s.Number, out (SequencePerformance Perf, bool SeaLevel) match))
            {
                t.Fail($"{label} seq {s.Number}", "no matching stock sequence");
                continue;
            }
            if (match.SeaLevel)
            {
                t.Skip($"{label} seq {s.Number}: non-active Atmospheric sequence, AFC is vacuum-only.");
                continue;
            }

            SequencePerformance p = match.Perf;
            double stockVe = p.MassFlowRate > 0f ? p.Thrust / p.MassFlowRate : 0.0;
            bool fuelOk = Approx.Rel(s.FuelMassKg, p.BurnedFuelMass, FuelTol, ElementFloor);
            bool mDotOk = Approx.Rel(s.MassFlowKgPerSec, p.MassFlowRate, MDotTol, ElementFloor);
            bool veOk = Approx.Rel(s.ExhaustVelocityMs, stockVe, VeTol, ElementFloor);
            t.Check($"{label} seq {s.Number}", fuelOk && mDotOk && veOk,
                $"fuel {s.FuelMassKg:F1}/{p.BurnedFuelMass:F1} " +
                $"mDot {s.MassFlowKgPerSec:F2}/{p.MassFlowRate:F2} " +
                $"Ve {s.ExhaustVelocityMs:F1}/{stockVe:F1} start {s.StartMassKg:F1}/{p.WetMass:F1}");
        }
    }

    // A propellant-disabled tank must drop out of the burnable fuel, and AFC must still match the
    // game's BurnedFuelMass afterwards (the game skips PropellantUseEnabled=false tanks too).
    private static void DisabledTankCase(TestContext t, Vehicle vehicle)
    {
        double fuelBefore = TotalFuel(SequenceBurnState.Analyze(vehicle));

        Tank? target = PickLargestEnabledTank(vehicle);
        if (target == null)
        {
            t.Skip($"{MultiStageVehicle} disabled-tank: no enabled fuel tank.");
            return;
        }
        double tankMass = target.ComputeSubstanceMass(vehicle.Parts.Moles.States);
        target.PropellantUseEnabled = false;

        SequenceBurnState after = SequenceBurnState.Analyze(vehicle);
        double fuelAfter = TotalFuel(after);
        t.Check($"{MultiStageVehicle} disabled-tank drops it", fuelAfter < fuelBefore - MinDropKg,
            $"tank {target.InstanceId} ({tankMass:F1}kg); AFC burnable {fuelBefore:F1} -> {fuelAfter:F1}");
        CrossCheck(t, $"{MultiStageVehicle} disabled-tank", vehicle, after);
    }

    // With a non-default (cross-stage) FlowRule, AFC must read the same ConsumptionOrder the game
    // drains from, so the per-sequence numbers stay in agreement.
    private static void FlowRuleCase(TestContext t, Vehicle vehicle)
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
                    // FlowRule moved onto Combustor; solid cores have none.
                    if (core is not Combustor combustor || combustor.ResourceManager == null)
                        continue;
                    combustor.ResourceManager.FlowRule = rule;
                    changed++;
                }
            }
        }
        if (changed == 0)
        {
            t.Skip($"{MultiStageVehicle} flow-rule: no engine cores.");
            return;
        }

        t.Info($"{MultiStageVehicle} flow-rule: set {rule} on {changed} core(s).");
        CrossCheck(t, $"{MultiStageVehicle} flow-rule {rule}", vehicle, SequenceBurnState.Analyze(vehicle));
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
}

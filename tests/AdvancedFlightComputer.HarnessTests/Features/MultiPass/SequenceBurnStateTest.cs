using System.IO;
using Brutal.Numerics;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Compare the adapter with a private SequencePerformanceList calculation for each vehicle.
// Check sequence numbers, fuel, mass flow, exhaust velocity, and the correction for inert mass in subparts.
// Vehicle.TotalMass provides a separate mass check when no parts were jettisoned.
// These checks verify the adapter rather than the stock performance model.
public sealed class SequenceBurnStateTest : AfcTest
{
    private const double SpawnAltitudeM = 500_000.0; // Altitude does not affect the thrust compared here.
    private const double MDotTol = 5e-3;             // Both calculations use the same nozzle data.
    private const double VeTol = 5e-3;
    private const double FuelTol = 1e-2;
    private const double ElementFloor = 1e-6;        // Allow a small absolute error when the expected value is near zero.
    private const double MinDropKg = 1.0;            // A disabled fuel tank must remove at least this much mass.

    // Use shipped vehicles so the test does not need a local save.
    private static readonly string[] DefaultVehicles = { "Rocket", "Gemini7", "Polaris", "Hunter", "Banjo" };
    private const string MultiStageVehicle = "Rocket"; // Use the staged vehicle for tank and flow rule checks.

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

        // The synthetic solid booster covers fuel stored in a grain rather than a liquid tank.
        // PartTree.RecomputeSolidMotorStacks fills a changed grain only in the editor, so this fixture must refill it after spawning.
        WithVehicle(t, driver, home, BuildSyntheticSrb(),
            v =>
            {
                SequenceBurnState state = SequenceBurnState.Analyze(v);
                CrossCheck(t, "SRB baseline", v, state);
                CheckAllocations(t, state);
                CheckHohmannSchedule(t, v);
                ActiveEngineSubQuantumDrainCacheCase(t, v);
                EngineDesignCacheCase(t, v);
            },
            afterSpawn: v => v.RefillConsumables());

        if (DefaultVehicleSaves.FindSave(MultiStageVehicle) is VehicleSave rocket)
        {
            WithVehicle(t, driver, home, rocket.VehicleSaveData, v => DisabledTankCase(t, v));
            WithVehicle(t, driver, home, rocket.VehicleSaveData, v => FlowRuleCase(t, v));
            WithVehicle(t, driver, home, rocket.VehicleSaveData, v => DecouplerCacheCase(t, v));
            WithVehicle(t, driver, home, rocket.VehicleSaveData, v => PropellantDistributionCacheCase(t, v));
        }
    }

    // The first two steps initialize nozzle states and mass before the assertions read them.
    private static void WithVehicle(
        TestContext t, SimDriver driver, IParentBody home, VehicleSaveData data, Action<Vehicle> body,
        Action<Vehicle>? afterSpawn = null)
    {
        Vehicle vehicle = VehicleFixtures.SpawnFromSaveData(
            t.System, home, data, $"BurnState_{data.Id}",
            OrbitFixtures.CircularAt(home, SpawnAltitudeM, Universe.GetElapsedTime()));
        try
        {
            afterSpawn?.Invoke(vehicle);
            driver.Step(1e-3, 2);
            body(vehicle);
        }
        finally
        {
            VehicleSpawner.Despawn(vehicle);
        }
    }

    // Deserialize the solid booster through the normal vehicle save path so the game builds its engine and grain modules.
    private static VehicleSaveData BuildSyntheticSrb()
    {
        // The harness stops before Program.Main loads grain geometries, so load them before the fixture creates its solid motor.
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
                <EngineController InstanceOf="SRBMotor" LocalInstanceId="2" ActiveInStage="true" />
              </RootPartRef>
            </VehicleSaveData>
            """;
        using var reader = new StringReader(xml);
        var data = (VehicleSaveData)DefaultVehicleSaves.VehicleSerializer.Deserialize(reader)!;
        // VehicleSaveData.LoadFrom passes Mod.Empty, which is not accessible here.
        // A new KSA.Mod supplies the same empty context.
        data.OnDataLoad(new KSA.Mod());
        return data;
    }

    private static void CrossCheck(TestContext t, string label, Vehicle vehicle, SequenceBurnState afc)
    {
        if (afc.Sequences.Count == 0)
        {
            // Some stock saves assign decouplers to the first sequence, which can leave the stock model with no usable engines.
            // Skip those cases because there is no stock result to compare.
            t.Skip($"{label}: no usable sequences (stock reports none for this vehicle).");
            return;
        }

        SequencePerformanceList stock = new SequencePerformanceList(vehicle.Parts);
        stock.RecomputeForFlight(0f);

        // Stock performance entries follow sequence list order.
        // Match them by sequence number because the adapter omits entries without usable engines.
        ReadOnlySpan<Sequence> seqs = vehicle.Parts.SequenceList.Sequences;
        ReadOnlySpan<SequencePerformance> perf = stock.PerformanceSequences;
        int count = Math.Min(seqs.Length, perf.Length);

        var byNumber = new Dictionary<int, SequencePerformance>(count);
        for (int i = 0; i < count; i++)
            byNumber[seqs[i].Number] = perf[i];

        foreach (SequenceInfo s in afc.Sequences)
        {
            if (!byNumber.TryGetValue(s.Number, out SequencePerformance p))
            {
                t.Fail($"{label} seq {s.Number}", "no matching stock sequence");
                continue;
            }
            double stockVe = p.MassFlowRate > 0f ? p.Thrust / p.MassFlowRate : 0.0;
            bool fuelOk = Approx.Rel(s.FuelMassKg, p.BurnedFuelMass, FuelTol, ElementFloor);
            bool mDotOk = Approx.Rel(s.MassFlowKgPerSec, p.MassFlowRate, MDotTol, ElementFloor);
            bool veOk = Approx.Rel(s.ExhaustVelocityMs, stockVe, VeTol, ElementFloor);

            // Stock WetMass omits inert mass from subparts.
            // Calculate that mass independently from the attached parts to check the adapter correction.
            double subPartInert = 0.0;
            int attachedCount = 0;
            if (p.AttachedParts != null)
            {
                attachedCount = p.AttachedParts.Count;
                foreach (Part part in p.AttachedParts)
                {
                    ReadOnlySpan<Part> subParts = part.SubParts;
                    for (int i = 0; i < subParts.Length; i++)
                        subPartInert += subParts[i].InertMass?.MassPropertiesAsmb.Props.Mass ?? 0f;
                }
            }
            bool startOk = Approx.Mixed(s.StartMassKg, p.WetMass + subPartInert, 0.5, 1e-6);
            // When nothing has been jettisoned, the corrected start mass must also match Vehicle.TotalMass.
            if (attachedCount == vehicle.Parts.Parts.Length)
                startOk &= Approx.Rel(s.StartMassKg, vehicle.TotalMass, 1e-3, ElementFloor);

            t.Check($"{label} seq {s.Number}", fuelOk && mDotOk && veOk && startOk,
                $"fuel {s.FuelMassKg:F1}/{p.BurnedFuelMass:F1} " +
                $"mDot {s.MassFlowKgPerSec:F2}/{p.MassFlowRate:F2} " +
                $"Ve {s.ExhaustVelocityMs:F1}/{stockVe:F1} " +
                $"start {s.StartMassKg:F1} (wet {p.WetMass:F1} + subInert {subPartInert:F1}, " +
                $"total {vehicle.TotalMass:F1})");
        }
    }

    // A disabled tank must reduce burnable fuel while the adapter remains consistent with the stock calculation.
    private static void DisabledTankCase(TestContext t, Vehicle vehicle)
    {
        double fuelBefore = TotalFuel(SequenceBurnState.Analyze(vehicle));
        if (!(fuelBefore > 0.0))
        {
            // A fuel reduction cannot be measured when the baseline has no burnable fuel.
            t.Skip($"{MultiStageVehicle} disabled-tank: stock reports no burnable fuel for this vehicle.");
            return;
        }

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

    // Changing FlowRule must leave the adapter consistent with the consumption order used by the game.
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
                    // Only liquid combustors have a FlowRule.
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

    private static void DecouplerCacheCase(TestContext t, Vehicle vehicle)
    {
        foreach (Part part in vehicle.Parts.Parts)
        {
            foreach (ISequenced module in part.GetSubtreeSequencedModules())
            {
                if (module is not Decoupler decoupler || !decoupler.IsEnabled
                    || decoupler.Connector.Connection == null) continue;
                try
                {
                    MultiPassPreviewCache.Reset();
                    SequenceBurnState before = MultiPassPreviewCache.GetSequenceState(vehicle);
                    decoupler.SetIsEnabled(false);
                    SequenceBurnState cached = MultiPassPreviewCache.GetSequenceState(vehicle);
                    SequenceBurnState fresh = SequenceBurnState.Analyze(vehicle);
                    t.Check("decoupler toggle invalidates sequence snapshot", !ReferenceEquals(before, cached));
                    t.Check("decoupler cache matches fresh stock analysis", SameState(cached, fresh));
                }
                finally
                {
                    decoupler.SetIsEnabled(true);
                    MultiPassPreviewCache.Reset();
                }
                return;
            }
        }
        t.Skip("decoupler cache: no enabled connected decoupler in fixture.");
    }

    private static void PropellantDistributionCacheCase(TestContext t, Vehicle vehicle)
    {
        var disabledDecouplers = new List<Decoupler>();
        foreach (Part part in vehicle.Parts.Parts)
        {
            foreach (ISequenced module in part.GetSubtreeSequencedModules())
            {
                if (module is Decoupler { IsEnabled: true } decoupler)
                {
                    decoupler.SetIsEnabled(false);
                    disabledDecouplers.Add(decoupler);
                }
            }
        }

        try
        {
            if (!TryFindTransfer(vehicle, out Tank donorTank, out Mole donor,
                    out Tank receiverTank, out Mole receiver, out float transferKg))
            {
                t.Skip("propellant distribution cache: no connected donor and compatible receiver.");
                return;
            }

            bool receiverWasEnabled = receiverTank.PropellantUseEnabled;
            receiverTank.PropellantUseEnabled = false;
            var donorState = vehicle.Parts.Moles.GetModuleAndAllMutableStatesForInitialization(donor);
            var receiverState = vehicle.Parts.Moles.GetModuleAndAllMutableStatesForInitialization(receiver);
            float donorMass = donorState.State.Mass;
            float receiverMass = receiverState.State.Mass;
            try
            {
                MultiPassPreviewCache.Reset();
                SequenceBurnState before = MultiPassPreviewCache.GetSequenceState(vehicle);
                float totalBefore = TotalMoleMass(vehicle);

                donorState.State.Mass -= transferKg;
                receiverState.State.Mass += transferKg;

                SequenceBurnState cached = MultiPassPreviewCache.GetSequenceState(vehicle);
                SequenceBurnState fresh = SequenceBurnState.Analyze(vehicle);
                float totalAfter = TotalMoleMass(vehicle);
                t.Check("propellant redistribution keeps total mole mass",
                    Approx.Mixed(totalAfter, totalBefore, 1e-3, 1e-7),
                    $"moved {transferKg:F3}kg from tank {donorTank.InstanceId} to disabled tank {receiverTank.InstanceId}");
                t.Check("propellant redistribution changes stock performance", !SameState(before, fresh));
                t.Check("propellant redistribution invalidates sequence snapshot", !ReferenceEquals(before, cached));
                t.Check("propellant redistribution cache matches fresh stock analysis", SameState(cached, fresh));
            }
            finally
            {
                donorState.State.Mass = donorMass;
                receiverState.State.Mass = receiverMass;
                receiverTank.PropellantUseEnabled = receiverWasEnabled;
                MultiPassPreviewCache.Reset();
            }
        }
        finally
        {
            foreach (Decoupler decoupler in disabledDecouplers)
                decoupler.SetIsEnabled(true);
        }
    }

    private static bool TryFindTransfer(
        Vehicle vehicle, out Tank donorTank, out Mole donor,
        out Tank receiverTank, out Mole receiver, out float transferKg)
    {
        donorTank = null!;
        donor = null!;
        receiverTank = null!;
        receiver = null!;
        transferKg = 0f;
        ReadOnlySpan<MoleState> states = vehicle.Parts.Moles.States;
        Span<Tank> tanks = vehicle.Parts.Tanks.Modules;

        ReadOnlySpan<Part> parts = vehicle.Parts.Parts;
        for (int p = 0; p < parts.Length; p++)
        {
            Span<EngineController> engines = parts[p].Modules.Get<EngineController>();
            for (int e = 0; e < engines.Length; e++)
            {
                foreach (RocketCore core in engines[e].Cores)
                {
                    if (core is not Combustor { ResourceManager: not null } combustor)
                        continue;
                    Tank[][]? levels = combustor.ResourceManager.ConsumptionOrder;
                    if (levels == null) continue;
                    foreach (Tank[]? level in levels)
                    {
                        if (level == null) continue;
                        foreach (Tank? candidateDonor in level)
                        {
                            if (candidateDonor == null || !candidateDonor.PropellantUseEnabled)
                                continue;
                            foreach (Mole candidateMole in candidateDonor.Moles)
                            {
                                float donorMass = states[candidateMole.StatesIdx].Mass;
                                if (!(donorMass > 0f)) continue;
                                for (int t = 0; t < tanks.Length; t++)
                                {
                                    Tank candidateReceiver = tanks[t];
                                    if (candidateReceiver == candidateDonor
                                        || !candidateReceiver.TryGetMole(candidateMole.SubstancePhase, out Mole receiverMole))
                                        continue;
                                    float receiverMass = states[receiverMole.StatesIdx].Mass;
                                    float capacity = receiverMole.GetStoredMass(receiverMole.ContainerVolume) - receiverMass;
                                    float amount = TransferAcrossMassBucket(donorMass, receiverMass, capacity);
                                    if (!(amount > 0f)) continue;
                                    donorTank = candidateDonor;
                                    donor = candidateMole;
                                    receiverTank = candidateReceiver;
                                    receiver = receiverMole;
                                    transferKg = amount;
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    private static float TransferAcrossMassBucket(float donorMass, float receiverMass, float capacity)
    {
        const float quantumKg = 100f;
        const float marginKg = 0.01f;
        float donorBoundary = donorMass - MathF.Floor(donorMass / quantumKg) * quantumKg + marginKg;
        if (donorBoundary < donorMass && donorBoundary <= capacity)
            return donorBoundary;
        float receiverBoundary = (MathF.Floor(receiverMass / quantumKg) + 1f) * quantumKg
            - receiverMass + marginKg;
        return receiverBoundary < donorMass && receiverBoundary <= capacity ? receiverBoundary : 0f;
    }

    private static void EngineDesignCacheCase(TestContext t, Vehicle vehicle)
    {
        SolidMotor? motor = null;
        foreach (RocketCore core in vehicle.Parts.RocketCores.Modules)
        {
            if (core is SolidMotor candidate)
            {
                motor = candidate;
                break;
            }
        }
        if (motor == null || !(motor.MaxAreaRatioBound > motor.MinAreaRatioBound))
        {
            t.Skip("engine design cache: no adjustable solid motor.");
            return;
        }

        float? original = motor.ManualAreaRatio;
        float current = motor.AreaRatio;
        float replacement = MathF.Abs(current - motor.MinAreaRatioBound)
            > MathF.Abs(current - motor.MaxAreaRatioBound)
            ? motor.MinAreaRatioBound
            : motor.MaxAreaRatioBound;
        try
        {
            MultiPassPreviewCache.Reset();
            SequenceBurnState before = MultiPassPreviewCache.GetSequenceState(vehicle);
            long massBucket = (long)(vehicle.TotalMass / 100.0);

            motor.ManualAreaRatio = replacement;
            vehicle.Parts.RecomputeAllDerivedData();

            SequenceBurnState cached = MultiPassPreviewCache.GetSequenceState(vehicle);
            SequenceBurnState fresh = SequenceBurnState.Analyze(vehicle);
            t.Check("engine design keeps vehicle mass bucket",
                (long)(vehicle.TotalMass / 100.0) == massBucket);
            t.Check("engine design changes stock performance", !SameState(before, fresh));
            t.Check("engine design invalidates sequence snapshot", !ReferenceEquals(before, cached));
            t.Check("engine design cache matches fresh stock analysis", SameState(cached, fresh));
        }
        finally
        {
            motor.ManualAreaRatio = original;
            vehicle.Parts.RecomputeAllDerivedData();
            MultiPassPreviewCache.Reset();
        }
    }

    private static void ActiveEngineSubQuantumDrainCacheCase(TestContext t, Vehicle vehicle)
    {
        SolidMotor? motor = null;
        foreach (RocketCore core in vehicle.Parts.RocketCores.Modules)
        {
            if (core is SolidMotor candidate)
            {
                motor = candidate;
                break;
            }
        }
        if (motor == null || motor.Controller is not EngineController { IsActive: true }
            || motor.Stack.Segments.Length == 0 || motor.Rocket.Nozzles.Length == 0)
        {
            t.Skip("sub-quantum drain cache: no active solid motor state.");
            return;
        }

        Mole? grain = motor.Stack.Segments[0].Grain;
        if (grain == null)
        {
            t.Skip("sub-quantum drain cache: solid motor has no grain state.");
            return;
        }
        var grainState = vehicle.Parts.Moles.GetModuleAndAllMutableStatesForInitialization(grain);
        RocketNozzle nozzle = motor.Rocket.Nozzles[0];
        var nozzleState = vehicle.Parts.RocketNozzles.GetModuleAndAllMutableStatesForInitialization(nozzle);
        float originalMass = grainState.State.Mass;
        RocketPerformance originalPerformance = nozzleState.State.Performance;
        float roomInBucket = originalMass - MathF.Floor(originalMass / 100f) * 100f;
        float drainKg = MathF.Min(1f, roomInBucket * 0.5f);
        if (!(drainKg > 0.01f))
        {
            t.Skip("sub-quantum drain cache: grain mass is on a cache boundary.");
            return;
        }

        try
        {
            MultiPassPreviewCache.Reset();
            SequenceBurnState before = MultiPassPreviewCache.GetSequenceState(vehicle);
            long moleBucket = (long)(originalMass / 100f);

            grainState.State.Mass -= drainKg;
            nozzleState.State.Performance.MassFlowRate += MathF.Max(
                MathF.Abs(nozzleState.State.Performance.MassFlowRate) * 1e-4f, 1e-6f);

            SequenceBurnState cached = MultiPassPreviewCache.GetSequenceState(vehicle);
            t.Check("sub-quantum drain keeps grain mass bucket",
                (long)(grainState.State.Mass / 100f) == moleBucket);
            t.Check("sub-quantum active drain keeps sequence snapshot", ReferenceEquals(before, cached));
        }
        finally
        {
            grainState.State.Mass = originalMass;
            nozzleState.State.Performance = originalPerformance;
            MultiPassPreviewCache.Reset();
        }
    }

    private static bool SameState(SequenceBurnState left, SequenceBurnState right)
    {
        if (left.HasUsableEngines != right.HasUsableEngines
            || left.Sequences.Count != right.Sequences.Count)
            return false;
        for (int i = 0; i < left.Sequences.Count; i++)
        {
            if (left.Sequences[i] != right.Sequences[i])
                return false;
        }
        return true;
    }

    private static float TotalMoleMass(Vehicle vehicle)
    {
        float total = 0f;
        ReadOnlySpan<MoleState> states = vehicle.Parts.Moles.States;
        for (int i = 0; i < states.Length; i++)
            total += states[i].Mass;
        return total;
    }

    private static void CheckAllocations(TestContext t, SequenceBurnState state)
    {
        t.Check("splitter fixture has fuel", state.HasUsableEngines);
        if (!state.HasUsableEngines) return;
        foreach (SplitMode mode in Enum.GetValues<SplitMode>())
        {
            PassAllocation[] single = Splitter.Allocate(100.0, 1, mode, state);
            PassAllocation[] split = Splitter.Allocate(100.0, 4, mode, state);
            double splitTime = 0.0;
            foreach (PassAllocation pass in split) splitTime += pass.EstimatedBurnTimeSec;
            t.Check($"{mode} split conserves delivered delta-v",
                Approx.Mixed(Splitter.SumDvCapacityMs(split), single[0].DvCapacityMs, 1e-6, 1e-6));
            t.Check($"{mode} split conserves burn time",
                Approx.Mixed(splitTime, single[0].EstimatedBurnTimeSec, 1e-6, 1e-6));
            PassAllocation[] shortSplit = Splitter.Allocate(1e9, 4, mode, state);
            t.Check($"{mode} fuel shortage stays visible",
                Splitter.SumDvCapacityMs(shortSplit) < 1e9);
        }
        t.Check("splitter clamps pass count",
            Splitter.Allocate(100.0, 100, SplitMode.EqualDv, SequenceBurnState.Empty).Length == Splitter.MaxPasses);
    }

    private static void CheckHohmannSchedule(TestContext t, Vehicle vehicle)
    {
        Orbit parking = vehicle.Orbit;
        UniverseTime now = Universe.GetElapsedTime();
        double rp = parking.Periapsis;
        double apo = rp * 8.0;
        Orbit target = OrbitFixtures.EllipticalAt(parking.Parent,
            rp - parking.Parent.MeanRadius, apo - parking.Parent.MeanRadius, now);
        double targetSpeed = target.GetOrbitalSpeed(rp);
        double parkingSpeed = parking.GetOrbitalSpeed(rp);
        const double rotation = 0.01;
        var input = new HohmannMultiPassPlanner.HohmannPlanInput(
            vehicle, now + 30.0 * parking.Period,
            new double3(targetSpeed * Math.Cos(rotation) - parkingSpeed,
                targetSpeed * Math.Sin(rotation), 0.0), false, 0.0, apo);

        // Check the departure schedule against the orbits propagated by the game.
        for (int count = 2; count <= 4; count++)
        {
            PassPreviewResult result = HohmannMultiPassPlanner.Plan(vehicle, input, count, 0,
                parking.Period, SequenceBurnState.Empty, now, SplitMode.EqualDv);
            t.Check($"Hohmann {count} passes planned", !result.Failed && result.Passes.Length == count,
                result.FailureReason ?? "");
            if (result.Failed || result.Passes.Length != count) continue;
            double periodSum = 0.0;
            for (int i = 0; i < count - 1; i++)
            {
                PassPreview pass = result.Passes[i];
                Orbit post = pass.FlightPlan.Patches[0].Orbit;
                double coast = (result.Passes[i + 1].BurnTime - pass.BurnTime).Seconds();
                t.Check($"Hohmann {count} pass {i} returns after one orbit",
                    Approx.Mixed(coast, post.Period, 0.01, 1e-7));
                t.Check($"Hohmann {count} pass {i} preserves periapsis",
                    Approx.Mixed(post.Periapsis, rp, 0.1, 1e-7));
                periodSum += post.Period / parking.Period;
            }
            t.Check($"Hohmann {count} preserves parking phase",
                Approx.Mixed(periodSum, Math.Round(periodSum), 1e-7, 1e-7));
            t.Check($"Hohmann {count} final time",
                Math.Abs((result.Passes[^1].BurnTime - input.TFinal).Seconds()) < 0.01);
            Orbit final = result.Passes[^1].FlightPlan.Patches[0].Orbit;
            t.Check($"Hohmann {count} final apoapsis",
                Approx.Mixed(final.Apoapsis, apo, 1.0, 1e-6));
            t.Check($"Hohmann {count} plane rotation",
                Approx.Mixed(final.GetRelativeInclination(parking).Value(), rotation, 1e-6, 1e-6));
        }
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

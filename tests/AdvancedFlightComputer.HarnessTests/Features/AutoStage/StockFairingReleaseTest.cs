using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class StockFairingReleaseTest : AfcTest
{
    public override string Name => "afc-autostage-stock-fairing";
    public override bool OptIn => true;

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        CheckFixture(t, home, "AFC Fairing 5901", mixed: false);
        CheckFixture(t, home, "AFC Fairing Mixed 5901", mixed: true);
    }

    private static void CheckFixture(TestContext t, IParentBody home, string save, bool mixed)
    {
        var driver = t.Session.CreateDriver();
        HashSet<string> before = TestSupport.CollectVehicleIds(t.System);
        Vehicle craft;
        try
        {
            string name = mixed ? "HarnessMixedFairing" : "HarnessFairing";
            Orbit orbit = OrbitFixtures.CircularAt(home, 150_000, driver.Elapsed);
            string? fixtureDirectory = Environment.GetEnvironmentVariable("AFC_FAIRING_FIXTURES");
            if (string.IsNullOrEmpty(fixtureDirectory))
                craft = VehicleSpawner.SpawnFromSave(save, t.System, home, name, orbit);
            else
            {
                VehicleSave fixture = UncompressedVehicleSave.FromDirectory(Path.Combine(fixtureDirectory, save))
                    ?? throw new InvalidOperationException("The stock fairing fixture is absent.");
                craft = VehicleSpawner.SpawnFromSaveData(fixture.VehicleSaveData, t.System, home, name, orbit);
            }
        }
        catch (InvalidOperationException e)
        {
            t.Skip($"{save}: {e.Message}");
            return;
        }
        bool rails = PhysicsBubble._forceOffRails;
        try
        {
            PhysicsBubble._forceOffRails = true;
            driver.Step(0.02, 3);
            craft.Parts.SequenceList.SetActiveSequence(1);
            StagingState state = StagingDetector.StateOf(craft);
            FairingJettison.Row? row = FairingJettison.Rows(craft, state).SingleOrDefault(r => r.Number == 2);
            if (!t.Check($"{save}: stock row resolves both shrouds", row is { Leaves.Length: 2 } && row.Mixed == mixed))
                return;
            t.Check($"{save}: only a pure fairing row is skipped for automatic engine staging",
                FairingJettison.NextAutomaticSequence(craft, state)?.Number == (mixed ? 2 : 3));
            Part[] shrouds = row!.Leaves.Select(leaf => leaf.Shroud).ToArray();
            int oldSequence = craft.Parts.SequenceList.ActiveSequence;
            var otherParts = new HashSet<Part>();
            foreach (Part part in craft.Parts.Parts)
                if (!shrouds.Contains(part))
                    otherParts.Add(part);
            EngineController[] engines = craft.Parts.Modules.Get<EngineController>().ToArray();
            bool[] engineStates = engines.Select(engine => engine.IsActive).ToArray();
            var fairing = FairingDrainSnapshot.Capture(craft, automatic: true, out _);
            if (!mixed && t.Check("the live drain model can identify both attached shrouds", fairing is { Parts.Length: 2 }))
            {
                var attached = craft.Parts.Parts.ToArray().ToHashSet();
                bool stockAlreadyCountsThem = fairing!.MassExcludedByStock(attached) == 0.0;
                attached.ExceptWith(shrouds);
                double restored = fairing.MassExcludedByStock(attached);
                foreach (var leaf in fairing.Parts)
                    attached.Remove(leaf.Mount);
                t.Check("UPFG restores only shrouds omitted before their mounts stage",
                    stockAlreadyCountsThem && Math.Abs(restored - row.MassKg) < 1e-6
                    && fairing.MassExcludedByStock(attached) == 0.0);
            }

            // Pressure and clearance are exercised independently. Here the real input drain
            // must detach only the two saved stock joints and preserve the next staging row.
            FairingJettison.TryRelease(craft, state, driver.Elapsed.Seconds());
            if (mixed)
            {
                driver.Step(0.02, 3);
                t.Check("mixed row queues no independent split", state.FairingPending == null
                    && shrouds.All(part => part.Tree == craft.Parts)
                    && craft.Parts.SequenceList.ActiveSequence == oldSequence);
                return;
            }
            if (!t.Check("pure row queues the resolved joints", state.FairingPending != null))
                return;
            driver.Step(0.02, 3);
            FairingJettison.AwaitRelease(craft, state, driver.Elapsed.Seconds());
            t.Check("only the two shrouds leave, with both mounts retained and retired",
                shrouds.All(part => part.Tree != craft.Parts) && otherParts.All(part => part.Tree == craft.Parts)
                && row.Leaves.All(leaf => !leaf.Mount.IsEnabled && leaf.Mount.Connector.Connection == null));
            t.Check("stock engine activations and sequence progress are untouched",
                craft.Parts.SequenceList.ActiveSequence == oldSequence && engines.Select(e => e.IsActive).SequenceEqual(engineStates)
                && craft.Parts.SequenceList.GetNextSequenceNumber() == 3);
            var released = FairingDrainSnapshot.Capture(craft, automatic: true, out _);
            float pressure = craft.PhysicsEnvironment.AtmosphericPressure;
            craft.Parts.PerformanceSequences.RecomputeForFlight(pressure);
            UpfgVehicle model = KsaVehicleAdapter.BuildWithFairing(craft, pressure, null, null, released);
            UpfgVehicle stock = KsaVehicleAdapter.Build(craft, pressure);
            t.Check("UPFG reads the live reduced stack after release", released == null
                && model.Stages.Count > 0 && !model.FairingMassReconciled
                && model.Stages.Select(stage => stage.MassTotal).SequenceEqual(stock.Stages.Select(stage => stage.MassTotal)));
        }
        finally
        {
            PhysicsBubble._forceOffRails = rails;
            StagingDetector.ForgetVehicle(craft);
            TestSupport.DespawnNewVehicles(t.System, before);
        }
    }
}

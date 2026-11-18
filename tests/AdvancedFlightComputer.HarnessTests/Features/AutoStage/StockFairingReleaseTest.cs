using AdvancedFlightComputer.Features.AutoStage;
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
            craft = VehicleSpawner.SpawnFromSave(save, t.System, home, mixed ? "HarnessMixedFairing" : "HarnessFairing",
                OrbitFixtures.CircularAt(home, 150_000, driver.Elapsed));
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
        }
        finally
        {
            PhysicsBubble._forceOffRails = rails;
            StagingDetector.ForgetVehicle(craft);
            TestSupport.DespawnNewVehicles(t.System, before);
        }
    }
}

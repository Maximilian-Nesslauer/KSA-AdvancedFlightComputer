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

    private enum Case { Release, Mixed, EngineRowFirst }

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        CheckFixture(t, home, "AFC Fairing 5901", Case.Release);
        CheckFixture(t, home, "AFC Fairing Mixed 5901", Case.Mixed);
        CheckFixture(t, home, "AFC Fairing 5901", Case.EngineRowFirst);
    }

    private static void CheckFixture(TestContext t, IParentBody home, string save, Case test)
    {
        bool mixed = test == Case.Mixed;
        var driver = t.Session.CreateDriver();
        HashSet<string> before = TestSupport.CollectVehicleIds(t.System);
        Vehicle craft;
        try
        {
            string name = "HarnessFairing" + test;
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
            // The release keeps the retained craft controllable, so it needs crew in the capsule.
            VehicleSpawner.SeatRandomCrew(craft);
            driver.Step(0.02, 3);
            craft.Parts.SequenceList.SetActiveSequence(1);
            StagingState state = StagingDetector.StateOf(craft);
            FairingJettison.Row? row = FairingJettison.Rows(craft, state).SingleOrDefault(r => r.Number == 2);
            if (!t.Check($"{save} ({test}): stock row 2 resolves both shrouds", row is { Leaves.Length: 2 } && row.Mixed == mixed))
                return;
            t.Check($"{save} ({test}): only a pure fairing row is skipped for automatic engine staging",
                FairingJettison.NextAutomaticSequence(craft, state)?.Number == (mixed ? 2 : 3));
            Part[] shrouds = row!.Leaves.Select(leaf => leaf.Shroud).ToArray();

            if (test == Case.EngineRowFirst)
            {
                // Engine staging passes the fairing row, which must stay attached and stay a release candidate.
                StagingExecution.ActivateNextSequenceSplit(craft, FairingJettison.NextAutomaticSequence(craft, state));
                SequenceList list = craft.Parts.SequenceList;
                int manualNext = list.GetNextSequenceNumber();
                t.Check("a passed fairing row is marked, so the stock staging key does not step back to it",
                    list.ActiveSequence == 3 && row.Sequence.Activated && manualNext != 2
                    && shrouds.All(part => part.Tree == craft.Parts),
                    $"active {list.ActiveSequence}, next {manualNext}");
                t.Check("the passed row is still released by its joints",
                    FairingJettison.Rows(craft, state).Any(r => !r.Mixed && ReferenceEquals(r.Sequence, row.Sequence)));
                StagingDetector.Arm(craft, true);
                StagingDetector.Arm(craft, false);
                t.Check("a disarm gives the passed row back to the stock staging key",
                    !row.Sequence.Activated && list.GetNextSequenceNumber() == row.Number
                    && shrouds.All(part => part.Tree == craft.Parts),
                    $"next {list.GetNextSequenceNumber()}");
                driver.Step(0.02, 3);
                return;
            }

            int oldSequence = craft.Parts.SequenceList.ActiveSequence;
            var otherParts = new HashSet<Part>();
            foreach (Part part in craft.Parts.Parts)
                if (!shrouds.Contains(part))
                    otherParts.Add(part);
            EngineController[] engines = craft.Parts.Modules.Get<EngineController>().ToArray();
            bool[] engineStates = engines.Select(engine => engine.IsActive).ToArray();

            // The gate is covered by the UI logic tests. Here the real input drain must detach only the
            // two stock joints and leave the stock staging progress as it was.
            FairingJettison.TryRelease(craft, state, driver.Elapsed.Seconds());
            if (mixed)
            {
                driver.Step(0.02, 3);
                t.Check("a mixed row queues no release of its own", state.FairingPending == null
                    && shrouds.All(part => part.Tree == craft.Parts)
                    && craft.Parts.SequenceList.ActiveSequence == oldSequence);
                return;
            }
            if (!t.Check("a pure row queues the resolved joints", state.FairingPending != null))
                return;
            driver.Step(0.02, 3);
            FairingJettison.AwaitRelease(craft, state, driver.Elapsed.Seconds());
            t.Check("only the two shrouds leave, with both mounts retained and retired",
                shrouds.All(part => part.Tree != craft.Parts) && otherParts.All(part => part.Tree == craft.Parts)
                && row.Leaves.All(leaf => !leaf.Mount.IsEnabled && leaf.Mount.Connector.Connection == null));
            t.Check("stock engine activations and sequence progress are untouched",
                craft.Parts.SequenceList.ActiveSequence == oldSequence && engines.Select(e => e.IsActive).SequenceEqual(engineStates)
                && craft.Parts.SequenceList.GetNextSequenceNumber() == 3);
            t.Check("the release is reported once the shrouds are gone",
                state.FairingPending == null && state.FairingLastRelease.StartsWith("ROW ", StringComparison.Ordinal),
                state.FairingLastRelease);
        }
        finally
        {
            PhysicsBubble._forceOffRails = rails;
            StagingDetector.ForgetVehicle(craft);
            TestSupport.DespawnNewVehicles(t.System, before);
        }
    }
}

using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates the precondition PassCompletionPatch checks before it re-asserts stock's
// _transferCalculated for a running Hohmann multi-pass. That flag gates the selected-entry block in
// TransferPlanner.DrawPlanWindow, which reads _transferInfo.PorkChopData[...].TransferData with no
// null check, so anything that sets the flag is asserting the array behind it is populated.
//
// Two game states break that: TransferTask.Run installs an all-null PorkChopEntry[292, 292] while it
// works, and TransferPlanner.SetTransferInfo installs an equally empty one on every source,
// destination and plan-type change, with BestDvTransferIndex left at the default (0, 0). Neither is
// caught by stock's own BestDvTransferIndex.X != -1 test, because nothing ever writes that
// sentinel - TransferTask.Run is its only assignment site and always writes a real index.
//
// The decision is exercised through its explicit-input form, so the test constructs the states from
// real game types instead of writing TransferPlanner's private statics and having to restore them.
public sealed class StockPinGuardTest : AfcTest
{
    private const double SpawnAltitudeM = 500_000.0;
    private const string SpawnFrom = "Rocket";

    public override string Name => "afc-stock-pin-guard";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        VehicleSave? save = DefaultVehicleSaves.FindSave(SpawnFrom);
        if (save?.VehicleSaveData.RootPartInstance == null)
        {
            t.Fail("default vehicle", $"'{SpawnFrom}' is not shipped");
            return;
        }

        Vehicle vehicle = VehicleFixtures.SpawnDesign(
            t.System, home, save.VehicleSaveData.RootPartInstance, "StockPinGuard_Source",
            OrbitFixtures.CircularAt(home, SpawnAltitudeM, Universe.GetElapsedSimTime()));
        try
        {
            // A porkchop cell only has to be non-null for the decision; its contents are never read.
            var cell = new OrbitalTransfers.PorkChopEntry(
                new OrbitalTransfers.TransferData(), FlightPlan.CreateUninitialized(vehicle.Hash));

            // What SetTransferInfo hands out: a full-size array with every cell null and
            // BestDvTransferIndex at the int2 default.
            var empty = new OrbitalTransfers.TransferInfo(vehicle, vehicle, vehicle, usePorkChopData: true);
            CheckDecision(t, "refuses an all-null porkchop array (fresh TransferInfo)",
                expected: false, StockPlanner.CanIndexPorkChopData(cell, false, empty));

            // What a completed calculation leaves behind. Both best indices default to (0, 0), so
            // filling that one cell satisfies the whole decision.
            var populated = new OrbitalTransfers.TransferInfo(vehicle, vehicle, vehicle, usePorkChopData: true);
            populated.PorkChopData[0, 0] = cell;
            CheckDecision(t, "allows a populated porkchop array",
                expected: true, StockPlanner.CanIndexPorkChopData(cell, false, populated));

            // The window TransferTask.Run is inside: cells can be null or half-merged even when the
            // best index still points at a filled one from the previous run.
            CheckDecision(t, "refuses while a calculation is in flight",
                expected: false, StockPlanner.CanIndexPorkChopData(cell, true, populated));

            // Stock keeps _selectedEntry through every reset that empties the array, so its absence
            // is not what makes the array safe - but its presence is still required, because the
            // block guards on it before indexing.
            CheckDecision(t, "refuses without a selected entry",
                expected: false, StockPlanner.CanIndexPorkChopData(null, false, populated));
            CheckDecision(t, "refuses without a TransferInfo",
                expected: false, StockPlanner.CanIndexPorkChopData(cell, false, null));

            // usePorkChopData: false allocates PorkChopEntry[0, 0]; the default (0, 0) index is then
            // out of bounds rather than null.
            var unsized = new OrbitalTransfers.TransferInfo(vehicle, vehicle, vehicle, usePorkChopData: false);
            CheckDecision(t, "refuses a zero-sized porkchop array",
                expected: false, StockPlanner.CanIndexPorkChopData(cell, false, unsized));

            // A best index outside the array would throw rather than return null, so it has to be
            // range-checked and not just null-checked.
            var outOfRange = new OrbitalTransfers.TransferInfo(vehicle, vehicle, vehicle, usePorkChopData: true);
            outOfRange.PorkChopData[0, 0] = cell;
            outOfRange.BestDvTransferIndex = new int2(OrbitalTransfers.PORKCHOP_HEIGHT, 0);
            CheckDecision(t, "refuses an out-of-range best index",
                expected: false, StockPlanner.CanIndexPorkChopData(cell, false, outOfRange));

            // Stock reads whichever index the "Select Best Dv" checkbox picks, so a populated
            // best-dV cell is not enough on its own.
            var oneSided = new OrbitalTransfers.TransferInfo(vehicle, vehicle, vehicle, usePorkChopData: true);
            oneSided.PorkChopData[0, 0] = cell;
            oneSided.BestDvRelSpeedTransferIndex = new int2(1, 1);
            CheckDecision(t, "refuses when only one of the two best indices resolves",
                expected: false, StockPlanner.CanIndexPorkChopData(cell, false, oneSided));
        }
        finally
        {
            VehicleSpawner.Despawn(vehicle);
        }
    }

    private static void CheckDecision(TestContext t, string label, bool expected, bool actual)
        => t.Check(label, actual == expected, $"expect {expected}, got {actual}");
}

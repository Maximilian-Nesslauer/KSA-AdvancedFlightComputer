using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class VehicleFixturesTest : AfcTest
{
    public override string Name => "afc-vehicle-fixtures";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        VehicleSave? save = DefaultVehicleSaves.FindSave("Rocket");
        if (save?.VehicleSaveData.RootPartInstance == null)
        {
            t.Fail("default vehicle", "Rocket is not available");
            return;
        }

        CheckConstructionRollback(t, home, save.VehicleSaveData);
        CheckRestoreRollback(t, home, save.VehicleSaveData);
        CheckOccupiedIdRejection(t, home, save.VehicleSaveData);
    }

    private static void CheckConstructionRollback(
        TestContext t, IParentBody home, VehicleSaveData data)
    {
        string id = "FixtureConstructionRollback_" + Guid.NewGuid().ToString("N");
        int systemCount = t.System.Count;
        int childCount = home.Children.Count;
        int audioCount = GameAudio.AllIAudio.Count;
        Orbit orbit = OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime());
        bool threw = false;
        try
        {
            VehicleFixtures.SpawnDesign(t.System, null!, data.RootPartInstance!, id, orbit);
        }
        catch (NullReferenceException)
        {
            threw = true;
        }

        t.Check("construction failure escapes", threw);
        bool rollbackComplete = CheckRollback(
            t, home, id, systemCount, childCount, audioCount, "construction");
        if (!rollbackComplete)
        {
            RemovePartialRegistration(t.System, id);
            return;
        }

        Vehicle? replacement = null;
        try
        {
            replacement = VehicleFixtures.SpawnDesign(
                t.System, home, data.RootPartInstance!, id,
                OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime()));
            t.Check("construction failure does not block id reuse", IsRegistered(t.System, id, replacement));
        }
        finally
        {
            if (replacement != null)
                VehicleSpawner.Despawn(replacement);
        }
        CheckCleanState(t, home, systemCount, childCount, audioCount, "construction");
    }

    private static void CheckRestoreRollback(TestContext t, IParentBody home, VehicleSaveData data)
    {
        string id = "FixtureRestoreRollback_" + Guid.NewGuid().ToString("N");
        int systemCount = t.System.Count;
        int childCount = home.Children.Count;
        int audioCount = GameAudio.AllIAudio.Count;
        Orbit orbit = OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime());
        var invalid = new VehicleSaveData
        {
            Id = data.Id,
            RootPartInstance = data.RootPartInstance,
            ActiveSequence = data.ActiveSequence,
            SequenceEnvironments = [null!],
            FuelLinks = data.FuelLinks,
        };
        bool threw = false;
        try
        {
            VehicleFixtures.SpawnFromSaveData(t.System, home, invalid, id, orbit);
        }
        catch (NullReferenceException)
        {
            threw = true;
        }

        t.Check("restore failure escapes", threw);
        bool rollbackComplete = CheckRollback(
            t, home, id, systemCount, childCount, audioCount, "restore");
        if (!rollbackComplete)
        {
            RemoveCompleteVehicle(t.System, id);
            return;
        }

        Vehicle? replacement = null;
        try
        {
            replacement = VehicleFixtures.SpawnFromSaveData(
                t.System, home, data, id,
                OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime()));
            t.Check("restore failure does not block id reuse", IsRegistered(t.System, id, replacement));
        }
        finally
        {
            if (replacement != null)
                VehicleSpawner.Despawn(replacement);
        }
        CheckCleanState(t, home, systemCount, childCount, audioCount, "restore");
    }

    private static void CheckOccupiedIdRejection(
        TestContext t, IParentBody home, VehicleSaveData data)
    {
        string id = "FixtureOccupiedId_" + Guid.NewGuid().ToString("N");
        int systemCount = t.System.Count;
        int childCount = home.Children.Count;
        int audioCount = GameAudio.AllIAudio.Count;
        Vehicle? existing = null;
        try
        {
            existing = VehicleFixtures.SpawnDesign(
                t.System, home, data.RootPartInstance!, id,
                OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime()));
            int occupiedSystemCount = t.System.Count;
            int occupiedChildCount = home.Children.Count;
            int occupiedAudioCount = GameAudio.AllIAudio.Count;

            CheckOccupiedSpawn(t, "design", () => VehicleFixtures.SpawnDesign(
                t.System, home, data.RootPartInstance!, id,
                OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime())));
            CheckOccupiedSpawn(t, "restore", () => VehicleFixtures.SpawnFromSaveData(
                t.System, home, data, id,
                OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime())));

            t.Check("occupied id keeps original registration", IsRegistered(t.System, id, existing));
            t.Check("occupied id keeps system count", t.System.Count == occupiedSystemCount);
            t.Check("occupied id keeps parent children", home.Children.Count == occupiedChildCount);
            t.Check("occupied id keeps audio count", GameAudio.AllIAudio.Count == occupiedAudioCount);
        }
        finally
        {
            if (existing != null)
                VehicleSpawner.Despawn(existing);
        }
        CheckCleanState(t, home, systemCount, childCount, audioCount, "occupied id");
    }

    private static void CheckOccupiedSpawn(TestContext t, string label, Action spawn)
    {
        bool threw = false;
        try
        {
            spawn();
        }
        catch (InvalidOperationException)
        {
            threw = true;
        }
        t.Check($"occupied id rejects {label} spawn", threw);
    }

    private static bool CheckRollback(
        TestContext t, IParentBody home, string id, int systemCount, int childCount,
        int audioCount, string label)
    {
        bool registrationClear = t.System.All.Get(id) == null;
        bool systemCountRestored = t.System.Count == systemCount;
        bool childrenRestored = home.Children.Count == childCount;
        bool audioRestored = GameAudio.AllIAudio.Count == audioCount;
        t.Check($"{label} failure removes system registration", registrationClear);
        t.Check($"{label} failure restores system count", systemCountRestored);
        t.Check($"{label} failure restores parent children", childrenRestored);
        t.Check($"{label} failure restores audio count", audioRestored);
        return registrationClear && systemCountRestored && childrenRestored && audioRestored;
    }

    private static bool IsRegistered(CelestialSystem system, string id, Vehicle vehicle)
        => ReferenceEquals(system.All.Get(id), vehicle);

    private static void CheckCleanState(
        TestContext t, IParentBody home, int systemCount, int childCount, int audioCount, string label)
    {
        t.Check($"{label} case leaves later tests clean",
            t.System.Count == systemCount &&
            home.Children.Count == childCount &&
            GameAudio.AllIAudio.Count == audioCount);
    }

    private static void RemovePartialRegistration(CelestialSystem system, string id)
    {
        if (system.All.Get(id) is Astronomical registered)
            system.All.Deregister(registered);
    }

    private static void RemoveCompleteVehicle(CelestialSystem system, string id)
    {
        if (system.All.Get(id) is Vehicle vehicle)
        {
            VehicleSpawner.Despawn(vehicle);
            return;
        }

        for (int i = 0; i < GameAudio.AllIAudio.Count; i++)
        {
            if (GameAudio.AllIAudio[i] is Vehicle audioVehicle && audioVehicle.Id == id)
            {
                VehicleSpawner.Despawn(audioVehicle);
                return;
            }
        }
    }
}

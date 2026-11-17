using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The aero sweep against the game's force function, on the stock Rocket.
// Over the home body it measures the aero mesh of the craft, so the table has drag, and the sweep stays inside a time bound because it runs on the main thread.
// Over an airless moon the game applies no aerodynamic force, so the sweep measures nothing and gives no atmosphere and an all-zero table, without an error, which is what the ascent plans a vacuum climb from.
public sealed class GuidanceAeroSweepTest : AfcTest
{
    public override string Name => "afc-guidance-aero-sweep";

    // The sweep runs in the Vehicle.PrepareWorker prefix and holds the frame while it runs.
    private const double MaxSweepMs = 5000.0;

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
        Celestial? moonBody = TestWorld.FindMoon(home);

        Vehicle? air = null, airless = null;
        try
        {
            air = VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance,
                "AfcAeroSweepAir", OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime()));
            if (moonBody is IParentBody moon && moon.GetPhysicalAtmosphere() == null)
                airless = VehicleFixtures.SpawnDesign(t.System, moon, save.VehicleSaveData.RootPartInstance,
                    "AfcAeroSweepAirless", OrbitFixtures.CircularAt(moon, 100_000, Universe.GetElapsedTime()));
            SimDriver driver = t.Session.CreateDriver();
            driver.Step(0.02, 2);

            if (home.GetPhysicalAtmosphere() == null)
                t.Skip("the home body has no atmosphere to measure.");
            else
                CheckAir(t, air, home);

            if (airless == null)
                t.Skip("no airless moon in this system.");
            else
                CheckAirless(t, airless, (IParentBody)moonBody!);
        }
        finally
        {
            if (air != null)
                VehicleSpawner.Despawn(air);
            if (airless != null)
                VehicleSpawner.Despawn(airless);
        }
    }

    private static void CheckAir(TestContext t, Vehicle vehicle, IParentBody home)
    {
        t.Check("the stock Rocket has an aero mesh after a step", KsaAeroSweep.LiveMeshHash(vehicle) != 0UL);
        bool built = KsaAeroSweep.TryBuild(vehicle, home, 0.0, out KsaAeroSweep.Result aero, out string error);
        if (!t.Check("the sweep over the home body succeeds", built, error))
            return;
        t.Info(FormattableString.Invariant(
            $"Cd tail-first {aero.CdTailFirst:F3}, broadside {aero.CdBroadside:F3}, nose-first {aero.CdNoseFirst:F3}, L/D up to {aero.LiftToDragMax:F2}, Mach spread {aero.MachSpread * 100.0:F0} %, roll spread {aero.RollSpread * 100.0:F1} %, {aero.SweepMillis:F0} ms"));
        t.Check("the table measures drag", aero.Table != null && aero.CdTailFirst > 0.0 && aero.CdNoseFirst > 0.0);
        t.Check("the sweep comes with the body's atmosphere", aero.Atmosphere != null);
        t.Check("the sweep keys its table on the aero mesh it measured", aero.MeshHash == KsaAeroSweep.LiveMeshHash(vehicle),
            $"{aero.MeshHash:X} against {KsaAeroSweep.LiveMeshHash(vehicle):X}");
        t.Check("the sweep stays inside its time bound", aero.SweepMillis < MaxSweepMs,
            FormattableString.Invariant($"{aero.SweepMillis:F0} ms, bound {MaxSweepMs:F0} ms"));
        var state = new VehicleAutopilotState { Aero = aero };
        ulong hash = aero.MeshHash;
        t.Check("the current mesh keeps the table", !state.AeroStale(aero.SampledExtents, hash, home));
        aero.MeshHash = 0;
        t.Check("a published mesh invalidates a table with no hash", state.AeroStale(aero.SampledExtents, hash, home));
        t.Check("an unavailable live hash keeps the geometry fallback", !state.AeroStale(aero.SampledExtents, 0, home));
        aero.MeshHash = hash;
    }

    private static void CheckAirless(TestContext t, Vehicle vehicle, IParentBody moon)
    {
        bool built = KsaAeroSweep.TryBuild(vehicle, moon, 0.0, out KsaAeroSweep.Result aero, out string error);
        if (!t.Check("the sweep over an airless body gives no error", built && error.Length == 0, error))
            return;
        t.Check("an airless body gives no atmosphere", aero.Atmosphere == null);
        t.Check("an airless body gives no drag", aero.Table != null && Array.TrueForAll(aero.CdTable, cd => cd == 0.0)
            && Array.TrueForAll(aero.LiftTable, cl => cl == 0.0));
    }
}

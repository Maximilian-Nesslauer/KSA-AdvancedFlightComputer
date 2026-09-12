using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Two vehicles, one armed: the switch, the request and the state belong to the vehicle, not to
// whichever craft is controlled. A requested row activates on the armed vehicle only, a request on
// a disarmed vehicle waits until it is armed, and a disposed vehicle takes its state with it.
public sealed class AutoStagePerVehicleTest : AfcTest
{
    public override string Name => "afc-autostage-per-vehicle";

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

        Vehicle? a = null;
        Vehicle? b = null;
        Vehicle? originalControlled = Program.ControlledVehicle;
        using AutoStageTestPatches.Scope patches = AutoStageTestPatches.Apply();
        try
        {
            UniverseTime now = Universe.GetElapsedTime();
            a = VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance,
                "AfcStagingA_" + Guid.NewGuid().ToString("N"), OrbitFixtures.CircularAt(home, 500_000, now));
            b = VehicleFixtures.SpawnDesign(t.System, home, save.VehicleSaveData.RootPartInstance,
                "AfcStagingB_" + Guid.NewGuid().ToString("N"), OrbitFixtures.CircularAt(home, 900_000, now));
            StagingConfig.EngineDelays.Clear();
            StagingConfig.DecouplerDelays.Clear();
            Program.ControlledVehicle = a;
            SimDriver driver = t.Session.CreateDriver();
            driver.Step(0.05, 2);

            b.ToggleEnum(AfcAutoStageToggle.Enabled);
            t.Check("the gauge toggle arms the vehicle it was pressed on", StagingDetector.IsArmed(b));
            t.Check("the other vehicle stays disarmed", !StagingDetector.IsArmed(a));
            // The gauge calls the instantiation closed over Enum, which is the one patched.
            t.Check("IsSet answers per vehicle", b.IsSet<Enum>(AfcAutoStageToggle.Enabled, false) && !a.IsSet<Enum>(AfcAutoStageToggle.Enabled, false));

            int nextA = a.Parts.SequenceList.GetNextSequenceNumber();
            int nextB = b.Parts.SequenceList.GetNextSequenceNumber();
            StagingDetector.RequestStaging(a);
            StagingDetector.RequestStaging(b);
            driver.Step(0.05, 1);
            t.Check("a request on the armed vehicle activates its next row",
                b.Parts.SequenceList.GetNextSequenceNumber() != nextB || FirstRowActivated(b));
            t.Check("a request on a disarmed vehicle waits", a.Parts.SequenceList.GetNextSequenceNumber() == nextA && !FirstRowActivated(a));

            // The controlled vehicle is A, so B staging proves the machine does not follow the focus.
            t.Check("the armed vehicle is not the controlled one", !ReferenceEquals(Program.ControlledVehicle, b));

            StagingDetector.Arm(a, true);
            driver.Step(0.05, 1);
            t.Check("the pending request fires once the vehicle is armed", FirstRowActivated(a));

            StagingDetector.Arm(b, false);
            t.Check("disarming one vehicle leaves the other armed", !StagingDetector.IsArmed(b) && StagingDetector.IsArmed(a));

            Vehicle despawned = b;
            VehicleSpawner.Despawn(b);
            b = null;
            t.Check("a disposed vehicle takes its staging state with it", !StagingDetector.HasState(despawned) && StagingDetector.IsArmed(a));
        }
        finally
        {
            Program.ControlledVehicle = originalControlled;
            if (a != null)
                VehicleSpawner.Despawn(a);
            if (b != null)
                VehicleSpawner.Despawn(b);
        }
    }

    private static bool FirstRowActivated(Vehicle vehicle)
    {
        foreach (Sequence sequence in vehicle.Parts.SequenceList.Sequences)
        {
            if (!sequence.Parts.IsEmpty)
                return sequence.Activated;
        }
        return false;
    }
}

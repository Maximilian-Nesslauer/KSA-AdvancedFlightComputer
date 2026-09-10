using System.Collections;
using System.Reflection;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Uses real separation, then calls adoption directly for each detached vehicle.
public sealed class GuidanceBoosterHandoverTest : AfcTest
{
    public override string Name => "afc-guidance-booster-handover";

    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;
    private const double WindowS = 30.0;

    private static double _now;

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(RcsTestVehicles.Candidates);
        if (saves.Count == 0)
        {
            t.Skip("no test vehicle save present.");
            return;
        }

        // The test replaces the shared clock and clears records, so it needs an idle guidance session.
        if (Records.Count != 0 || VehicleAutopilotState.Snapshot().Length != 0)
        {
            t.Skip("another craft holds guidance state or a pending hand-over.");
            return;
        }

        FieldInfo ambient = typeof(GuidanceWindow).GetField("_s", PrivateStatic)!;
        object? previousAmbient = ambient.GetValue(null);
        Harmony harmony = new("com.maxi.afc.harnesstests.guidance.handover");
        harmony.Patch(Method("SimNow"),
            prefix: new HarmonyMethod(typeof(GuidanceBoosterHandoverTest), nameof(Clock)));
        try
        {
            _now = 100.0;
            EverySeparatedVehicleIsAdopted(t, home, saves[0], ambient);
            TwoPendingSeparationsCoexist(t, home, saves[0], ambient);
            AnUnclaimedRecordExpires(t, home, saves[0], ambient);
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
            ambient.SetValue(null, previousAmbient);
            Records.Clear();
            foreach (KeyValuePair<Vehicle, VehicleAutopilotState> entry in VehicleAutopilotState.Snapshot())
                VehicleAutopilotState.Remove(entry.Key);
        }
    }

    private static void EverySeparatedVehicleIsAdopted(TestContext t, IParentBody home, string save,
                                                       FieldInfo ambient)
    {
        SimDriver driver = t.Session.CreateDriver();
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        bool railsBefore = PhysicsBubble._forceOffRails;
        try
        {
            Vehicle parent;
            try
            {
                parent = VehicleSpawner.SpawnFromSave(save, t.System, home, "HarnessHandover",
                    OrbitFixtures.CircularAt(home, 500_000.0, driver.Elapsed));
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"{save}: {e.Message}");
                return;
            }

            PhysicsBubble._forceOffRails = true;
            driver.Step(0.05, 20);

            Arm(ambient, parent);
            int armed = Records.Count;
            if (armed == 0)
            {
                t.Skip("the next sequence of the save separates nothing.");
                return;
            }
            t.Info($"armed {armed} hand-over record(s)");

            // Activation queues the separation, and SimDriver.Step drains the input buffer.
            parent.Parts.SequenceList.ActivateNextSequence(parent);
            parent.UpdateAfterPartTreeModification();
            driver.Step(0.05, 20);

            List<Vehicle> children = new();
            foreach (Astronomical body in t.System.All.AsSpan())
            {
                if (body is Vehicle child && child is not KittenEva
                    && !preexisting.Contains(child.Id) && !ReferenceEquals(child, parent))
                    children.Add(child);
            }
            if (children.Count == 0)
            {
                t.Fail("separation", "the activation produced no separated vehicle");
                return;
            }
            t.Info($"the separation produced {children.Count} vehicle(s)");

            int adopted = 0;
            foreach (Vehicle child in children)
            {
                if (Adopt(child))
                    adopted++;
            }
            t.Check("every separated vehicle adopts a record", adopted == children.Count,
                $"{adopted} of {children.Count}");
            t.Check("each adoption consumes exactly one record", Records.Count == armed - adopted);

            int withState = 0;
            foreach (Vehicle child in children)
            {
                if (VehicleAutopilotState.TryGet(child, out _))
                    withState++;
            }
            t.Check("an adopted vehicle has guidance state of its own", withState == adopted);

            if (children.Count < 2)
                t.Skip("the save separates one vehicle, so two records from one sequence are untested.");
        }
        finally
        {
            PhysicsBubble._forceOffRails = railsBefore;
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    // Pending handovers from different vehicles must survive arming and adoption elsewhere.
    private static void TwoPendingSeparationsCoexist(TestContext t, IParentBody home, string save,
                                                     FieldInfo ambient)
    {
        SimDriver driver = t.Session.CreateDriver();
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        try
        {
            Vehicle first, second;
            try
            {
                first = VehicleSpawner.SpawnFromSave(save, t.System, home, "HarnessHandoverFirst",
                    OrbitFixtures.CircularAt(home, 700_000.0, driver.Elapsed));
                second = VehicleSpawner.SpawnFromSave(save, t.System, home, "HarnessHandoverSecond",
                    OrbitFixtures.CircularAt(home, 800_000.0, driver.Elapsed));
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"{save}: {e.Message}");
                return;
            }

            Arm(ambient, first);
            int afterFirst = Records.Count;
            if (afterFirst == 0)
            {
                t.Skip("the next sequence of the save separates nothing.");
                return;
            }
            Arm(ambient, second);
            t.Check("arming a second separation keeps the records of the first",
                Records.Count > afterFirst, $"{afterFirst} then {Records.Count}");

            int pending = Records.Count;
            t.Check("adopting from one separation leaves the other pending",
                Adopt(second) && Records.Count == pending - 1 && Records.Count >= afterFirst);
        }
        finally
        {
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static void AnUnclaimedRecordExpires(TestContext t, IParentBody home, string save,
                                                 FieldInfo ambient)
    {
        SimDriver driver = t.Session.CreateDriver();
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        try
        {
            Vehicle vehicle;
            try
            {
                vehicle = VehicleSpawner.SpawnFromSave(save, t.System, home, "HarnessHandoverExpiry",
                    OrbitFixtures.CircularAt(home, 600_000.0, driver.Elapsed));
            }
            catch (InvalidOperationException e)
            {
                t.Skip($"{save}: {e.Message}");
                return;
            }

            Arm(ambient, vehicle);
            if (!t.Check("a separation leaves a record behind", Records.Count > 0))
                return;

            _now += WindowS + 1.0;
            t.Check("a record nobody claimed expires",
                !Adopt(vehicle) && Records.Count == 0);
        }
        finally
        {
            TestSupport.DespawnNewVehicles(t.System, preexisting);
        }
    }

    private static IList Records =>
        (IList)typeof(GuidanceWindow).GetField("_handovers", PrivateStatic)!.GetValue(null)!;

    private static MethodInfo Method(string name) =>
        typeof(GuidanceWindow).GetMethod(name, PrivateStatic)
        ?? throw new MissingMethodException(nameof(GuidanceWindow), name);

    private static bool Clock(ref double __result)
    {
        __result = _now;
        return false;
    }

    // Arming reads the landing target from the current vehicle state.
    private static void Arm(FieldInfo ambient, Vehicle vehicle)
    {
        ambient.SetValue(null, VehicleAutopilotState.For(vehicle));
        Method("ArmBoosterHandover").Invoke(null, [vehicle, vehicle.Parts.SequenceList, _now]);
    }

    private static bool Adopt(Vehicle vehicle) =>
        (bool)Method("TryAdoptBooster").Invoke(null, [vehicle])!;
}

using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class ReturnableStageControlTest : AfcTest
{
    public override string Name => "afc-guidance-returnable-stage-control";

    private const string Capsule = "CoreCommandA_Prefab_MediumCapsuleVariantA";
    private const string Cabin = "CoreIVASpaceA_Prefab_MediumCapsuleA";
    private const string Probe = "CoreCommandC_Prefab_ProbeCoreSmallA";
    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        CheckDesign(t, home, withProbe: false);
        CheckDesign(t, home, withProbe: true);
    }

    private static void CheckDesign(TestContext t, IParentBody home, bool withProbe)
    {
        VehicleSaveData design = Design(withProbe);
        Vehicle v = VehicleFixtures.SpawnFromSaveData(t.System, home, design, design.Id,
            OrbitFixtures.CircularAt(home, 500_000.0, Universe.GetElapsedTime()));
        var seats = v.Crew.ToArray();
        var assignments = seats.Select(seat => seat.AssignedKittenHash).ToArray();
        try
        {
            Part joint = v.Parts.Modules.Get<Decoupler>()[0].Parent.FullPart;
            int separation = Array.FindIndex(v.Parts.SequenceList.Sequences.ToArray(), row => row.Number == 2);
            if (!t.Check("stock fixture has the separation row", separation >= 0
                && v.Parts.SequenceList.GetNextSequenceNumber() == 2))
                return;
            HashSet<Part> departing = GuidanceWindow.PartsLeavingWith(v, joint, separation);
            foreach (IVASeat seat in seats)
                seat.AssignedKittenHash = KeyHash.Zero;
            if (withProbe)
                CheckProbe(t, v, joint, departing);
            else
                CheckCrew(t, v, joint, departing);
        }
        finally
        {
            for (int i = 0; i < seats.Length; i++)
                seats[i].AssignedKittenHash = assignments[i];
            VehicleSpawner.Despawn(v);
        }

        void CheckCrew(TestContext context, Vehicle vehicle, Part joint, HashSet<Part> departing)
        {
            IVASeat? upper = Array.Find(seats, seat => !departing.Contains(seat.Parent.FullPart));
            IVASeat? lower = Array.Find(seats, seat => departing.Contains(seat.Parent.FullPart));
            KeyHash occupant = Array.Find(assignments, hash => hash != KeyHash.Zero);
            if (!context.Check("stock pods have seats and the fixture has a rostered kitten",
                upper != null && lower != null && occupant != KeyHash.Zero))
                return;

            CheckReturn(context, vehicle, joint, VehicleControl.NoCrew, "Its command pod has no crew.");
            context.Check("an already uncontrolled stack does not trip the last-control guard",
                !JettisonAnalysis.WouldSeparateLastControl(vehicle));

            upper!.AssignedKittenHash = occupant;
            CheckReturn(context, vehicle, joint, VehicleControl.NoCrew, "Its command pod has no crew.");
            context.Check("crew in the retained capsule keeps staging clear",
                !JettisonAnalysis.WouldSeparateLastControl(vehicle));

            upper.AssignedKittenHash = KeyHash.Zero;
            lower!.AssignedKittenHash = occupant;
            CheckReturn(context, vehicle, joint, VehicleControl.Controllable);
            context.Check("the last crewed pod is protected when an empty pod stays",
                JettisonAnalysis.WouldSeparateLastControl(vehicle));
        }
    }

    private static void CheckProbe(TestContext t, Vehicle v, Part joint, HashSet<Part> departing)
    {
        PowerConsumer probe = v.Parts.PowerConsumers.Modules.ToArray().Single(consumer => consumer.PowerSwitch);
        var batteries = v.Parts.Batteries;
        foreach (Battery battery in batteries.Modules)
        {
            ref BatteryState state = ref batteries.GetModuleAndAllMutableStatesForInitialization(battery).State;
            if (departing.Contains(battery.Parent.FullPart))
                battery.Refill(ref state);
            else
                state = BatteryState.Empty;
        }

        probe.Mode = PowerSwitchMode.Auto;
        probe.Hold = PowerHold.AutoStandby;
        CheckReturn(t, v, joint, VehicleControl.Controllable);
        t.Check("the last powered probe is protected when an empty pod stays",
            JettisonAnalysis.WouldSeparateLastControl(v));
        t.Check("a selection with no control modules has no control source",
            StageControl.Evaluate(v, new HashSet<Part> { joint }) == null);

        foreach (Battery battery in batteries.Modules)
        {
            ref BatteryState state = ref batteries.GetModuleAndAllMutableStatesForInitialization(battery).State;
            if (departing.Contains(battery.Parent.FullPart))
                state = BatteryState.Empty;
            else
                battery.Refill(ref state);
        }
        t.Check("the attached probe can draw charge from the retained stack",
            probe.PowerManager.ResourceAvailable(batteries.States));
        CheckReturn(t, v, joint, VehicleControl.ProbeCoreUnpowered, "Its probe core has no power after separation.");

        probe.Mode = PowerSwitchMode.Off;
        CheckReturn(t, v, joint, VehicleControl.ProbeCoreOff, "Its probe core is switched off.");
        t.Check("a switched-off probe is not a usable control source",
            !JettisonAnalysis.WouldSeparateLastControl(v));
    }

    private static void CheckReturn(TestContext t, Vehicle v, Part joint, VehicleControl expected, string? reason = null)
    {
        FieldInfo ambient = typeof(GuidanceWindow).GetField("_s", PrivateStatic)!;
        object? previous = ambient.GetValue(null);
        var state = new VehicleAutopilotState();
        try
        {
            ambient.SetValue(null, state);
            typeof(GuidanceWindow).GetMethod("RefreshReturnableStages", PrivateStatic)!.Invoke(null, [v]);
            if (expected == VehicleControl.Controllable)
                t.Check("the usable stage is listed for return", state.ReturnableStages.Count == 1
                    && state.ReturnableStages[0].RootId == joint.InstanceId && state.ReturnControlRefusals.Count == 0);
            else
                t.Check($"the unavailable stage is excluded with its LAUNCH reason ({expected})",
                    state.ReturnableStages.Count == 0 && state.ReturnControlRefusals.Count == 1
                    && state.ReturnControlRefusals[0].Reason == expected
                    && GuidanceWindow.ReturnControlReason(expected) == reason);
        }
        finally
        {
            ambient.SetValue(null, previous);
        }
    }

    private static VehicleSaveData Design(bool withProbe)
    {
        PartInstance upper = Instance(Capsule, 1, 0.0);
        PartInstance lower = Instance(Capsule, 4, -6.0);
        PartInstance joint = Instance("CoreFairingA_Prefab_Interstage3W3HB", 3, -3.0);
        PartInstance upperEngine = Instance("CorePropulsionA_Prefab_EngineA5", 6, -0.5);
        upperEngine.Components.Add(new EngineController.SaveData { InstanceOf = "LR91-AJ-3", LocalInstanceId = 11, Sequence = 2 });
        joint.Components.Add(new Decoupler.SaveData { InstanceOf = "", LocalInstanceId = 12, Sequence = 2 });
        upper.Children = [Instance(Cabin, 2, 0.0), joint, upperEngine];
        joint.Children = [lower];
        lower.Children = [Instance(Cabin, 5, -6.0)];
        Connect(upper, 1, joint, 1);
        Connect(joint, 0, lower, 0);
        if (withProbe)
        {
            PartInstance probe = Instance(Probe, 7, -8.0);
            lower.Children.Add(probe);
            Connect(lower, 1, probe, 1);
        }
        return new VehicleSaveData
        {
            Id = withProbe ? "HarnessProbeReturnControl" : "HarnessCrewReturnControl",
            RootPartInstance = upper
        };
    }

    private static PartInstance Instance(string template, uint id, double x) => new()
    {
        InstanceOf = template, LocalInstanceId = id,
        Transform = new TransformReference { PositionValue = new double3(x, 0.0, 0.0) }
    };

    private static void Connect(PartInstance first, int firstIndex, PartInstance second, int secondIndex)
    {
        (first.Connectors ??= []).Add(new PartConnectorInstance { Index = firstIndex, ConnectedLocalInstanceId = second.LocalInstanceId });
        (second.Connectors ??= []).Add(new PartConnectorInstance { Index = secondIndex, ConnectedLocalInstanceId = first.LocalInstanceId });
    }
}

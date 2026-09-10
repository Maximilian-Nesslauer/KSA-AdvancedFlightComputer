using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class VehicleCommandSinkTest : AfcTest
{
    public override string Name => "afc-command-sink";

    protected override void Execute(TestContext t)
    {
        ReceiptTakesTheEarliestWake(t);
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(RcsTestVehicles.Candidates);
        if (saves.Count == 0)
        {
            t.Skip("no RCS test vehicle save present.");
            return;
        }
        using RcsTestPatches.Scope patches = RcsTestPatches.Apply();
        WriterFailuresAndStockValues(t, home, saves[0]);
        ACommandedVehicleStaysOffRails(t, home, saves[0]);
    }

    private static void ReceiptTakesTheEarliestWake(TestContext t)
    {
        VehicleCommandSink.Receipt receipt = new();
        t.Check("a fresh receipt commands nothing",
            !receipt.Commanded && double.IsPositiveInfinity(receipt.WakeupSec));
        receipt.Wake(9.0);
        receipt.Wake(5.0);
        receipt.Wake(7.0);
        t.Check("the receipt keeps the earliest wake", receipt.WakeupSec == 5.0);
        receipt.Command();
        t.Check("a command is recorded", receipt.Commanded);
    }

    // A writer failure must not escape and abort the rest of the vehicle worker job, and whatever
    // the writer reported before it failed still has to reach the output.
    private static void WriterFailuresAndStockValues(TestContext t, IParentBody home, string save)
    {
        RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, "HarnessSinkFault",
            (vehicle, driver) =>
            {
                FlightComputer fc = vehicle.FlightComputer;
                FlightComputerNavigation nav = default;

                bool rcsBefore = SharedVehicleHooks.RcsEnabled;
                try
                {
                    SharedVehicleHooks.RcsEnabled = false;
                    FlightComputerOutput stock = default;
                    stock.AnyActuatorCommanded = true;
                    stock.NextWakeupDeltaTime = 0.02;
                    VehicleCommandSink.Run(fc, in nav, ref stock);
                    t.Check("the sink leaves stock values alone",
                        stock.AnyActuatorCommanded && stock.NextWakeupDeltaTime == 0.02);
                }
                finally
                {
                    SharedVehicleHooks.RcsEnabled = rcsBefore;
                }

                t.Check("a fault before any write adds no command",
                    !RunWithWriter(fc, in nav, nameof(ThrowPrefix), out double wake) && double.IsNaN(wake));

                t.Check("a report before a fault still reaches the output",
                    RunWithWriter(fc, in nav, nameof(CommandThenThrowPrefix), out double reported)
                    && reported == 0.25);
            });
    }

    /// <summary>Runs the sink with the RCS writer replaced, and reports what reached the output.
    /// The wake is NaN when the sink left the stock value untouched.</summary>
    private static bool RunWithWriter(FlightComputer fc, in FlightComputerNavigation nav,
                                      string prefix, out double wakeupSec)
    {
        Harmony harmony = new("com.maxi.afc.harnesstests.sink." + prefix);
        harmony.Patch(
            AccessTools.Method(typeof(RcsComputeControlPatch), nameof(RcsComputeControlPatch.Command)),
            prefix: new HarmonyMethod(typeof(VehicleCommandSinkTest), prefix));
        try
        {
            FlightComputerOutput outputs = default;
            outputs.NextWakeupDeltaTime = 1.0;
            VehicleCommandSink.Run(fc, in nav, ref outputs);
            wakeupSec = outputs.NextWakeupDeltaTime == 1.0 ? double.NaN : outputs.NextWakeupDeltaTime;
            return outputs.AnyActuatorCommanded;
        }
        finally
        {
            harmony.UnpatchAll(harmony.Id);
        }
    }

    private static bool ThrowPrefix() => throw new InvalidOperationException("Injected writer fault");

    private static bool CommandThenThrowPrefix(ref VehicleCommandSink.Receipt receipt)
    {
        receipt.Command();
        receipt.Wake(0.25);
        throw new InvalidOperationException("Injected writer fault after reporting");
    }

    // The sink exists so a commanded vehicle keeps running under full physics. The flight fixture
    // forces that result for its own tests, so this case switches the override off and lets the
    // vehicle reach rails first, otherwise the assertion would hold for another reason.
    private static void ACommandedVehicleStaysOffRails(TestContext t, IParentBody home, string save)
    {
        RcsFlightSupport.RunOnSave(t, home, save, 500_000.0, "HarnessSinkRails",
            (vehicle, driver) =>
            {
                FlightComputer fc = vehicle.FlightComputer;
                RcsFlightSupport.CleanupBurns(fc);
                fc.BurnMode = FlightComputerBurnMode.Manual;
                TestSupport.SetManualControlInputs(vehicle, 0f, engineOn: false);
                PhysicsBubble._forceOffRails = false;

                driver.Step(0.05, 40);
                if (!RcsCapability.Probe(vehicle).HasAnyTranslation)
                {
                    t.Skip("save has no active RCS translation capability.");
                    return;
                }

                driver.Step(0.05, 400);
                if (!vehicle.Situation.IsOnRails())
                {
                    t.Skip("the idle vehicle stayed in full physics, so the commanded case would prove nothing.");
                    return;
                }

                RcsFlightSupport.BurnSetup? setup =
                    RcsFlightSupport.AddBurn(vehicle, driver, double3.UnitX, 1.0, 5.0);
                if (setup == null)
                {
                    t.Fail("off-rails setup", "no patch or loaded burn target");
                    return;
                }
                RcsExecutor.Activate(vehicle);
                driver.Step(0.05, 20);

                bool active = RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec) && exec.IsActive;
                t.Check("the execution is still running when the situation is read", active);
                t.Check("an executing RCS burn holds its vehicle in full physics",
                    !vehicle.Situation.IsOnRails());
            });
    }
}

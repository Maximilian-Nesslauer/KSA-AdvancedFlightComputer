using System.Reflection;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using AdvancedFlightComputer.Features.Ui.Logic;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

public sealed class ProbePowerTest : AfcTest
{
    public override string Name => "afc-guidance-probe-power";

    private const string Probe = "CoreCommandC_Prefab_ProbeCoreSmallA";
    private const string Panel = "CoreElectricalA_Prefab_SolarPanelFixedSmallV1A";
    private const BindingFlags PrivateStatic = BindingFlags.NonPublic | BindingFlags.Static;

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        if (Universe.IsAutoWarpActive)
        {
            t.Skip("the world is already warping");
            return;
        }

        var design = new PartInstance
        {
            InstanceOf = Probe, LocalInstanceId = 1,
            Children =
            [
                new PartInstance
                {
                    InstanceOf = Panel, LocalInstanceId = 2,
                    Connectors = [new PartConnectorInstance { Index = 0, ConnectedLocalInstanceId = 1 }]
                },
                new PartInstance
                {
                    InstanceOf = Probe, LocalInstanceId = 3,
                    Connectors = [new PartConnectorInstance { Index = 0, ConnectedLocalInstanceId = 1 }]
                }
            ]
        };
        Vehicle probe = VehicleFixtures.SpawnDesign(t.System, home, design, "HarnessProbePower",
            OrbitFixtures.CircularAt(home, 500_000.0, Universe.GetElapsedTime()));
        try
        {
            CheckForecast(t, probe);
            CheckWarp(t, probe);
        }
        finally
        {
            VehicleSpawner.Despawn(probe);
        }

        var pod = new PartInstance
        {
            InstanceOf = "CoreCommandA_Prefab_MediumCapsuleVariantA", LocalInstanceId = 1,
            Children = [new PartInstance { InstanceOf = "CoreIVASpaceA_Prefab_MediumCapsuleA", LocalInstanceId = 2 }]
        };
        Vehicle crewed = VehicleFixtures.SpawnDesign(t.System, home, pod, "HarnessCrewPower",
            OrbitFixtures.CircularAt(home, 520_000.0, Universe.GetElapsedTime()));
        try
        {
            t.Check("able crew in a command zone have no probe power limit", double.IsPositiveInfinity(ProbePower.RemainingSeconds(crewed)));
        }
        finally
        {
            VehicleSpawner.Despawn(crewed);
        }
    }

    private static void CheckForecast(TestContext t, Vehicle vehicle)
    {
        PartTree tree = vehicle.Parts;
        tree.EnsureDerived(DerivedData.All);
        foreach (Battery battery in tree.Batteries.Modules)
            if (battery.Parent.FullPart == tree.Root)
                battery.Refill(ref tree.Batteries.GetModuleAndAllMutableStatesForInitialization(battery).State);
        PowerConsumer core = PrimarySwitch(vehicle);
        PowerConsumer backup = tree.PowerConsumers.Modules.ToArray().Single(item => item.PowerSwitch && item != core);
        core.Mode = PowerSwitchMode.On;
        backup.Mode = PowerSwitchMode.Off;

        double remaining = ProbePower.RemainingSeconds(vehicle);
        double charge = 0.0;
        int circuit = tree.ElectricalCircuits.IndexFor(core.Parent.FullPart);
        foreach (Battery battery in tree.ElectricalCircuits.Batteries(circuit))
            charge += tree.Batteries.States[battery.StatesIdx].Charge.Value();
        double expected = charge / core.Consumed.Value();
        string warning = PageText.ProbePowerWarning(remaining, expected + 180.0, "BURN");
        t.Check("the small stock core warns for a burn after its battery expires",
            expected > 300.0 && Math.Abs(remaining - expected) < 0.001 && warning.Contains("BEFORE BURN", StringComparison.Ordinal)
            && warning.Contains("ECLIPSE", StringComparison.Ordinal), $"{remaining:F2} s, {warning}");
        t.Check("a burn inside the estimate has no warning", PageText.ProbePowerWarning(remaining, 300.0, "BURN").Length == 0);

        var panels = tree.SolarPanels;
        ref SolarPanelState panel = ref panels.GetModuleAndAllMutableStatesForInitialization(panels.Modules[0]).State;
        panel = new SolarPanelState { Active = true, Produced = core.Consumed * 2f };
        t.Check("current generation can cover the probe demand",
            double.IsPositiveInfinity(ProbePower.RemainingSeconds(vehicle)));
        panel = default;

        core.Mode = PowerSwitchMode.Auto;
        core.Hold = PowerHold.AutoStandby;
        t.Check("an Auto core on standby forecasts its draw when it wakes",
            Math.Abs(ProbePower.RemainingSeconds(vehicle) - remaining) < 0.001);
        backup.Mode = PowerSwitchMode.Auto;
        backup.Hold = PowerHold.AutoStandby;
        t.Check("both standby cores on a shared circuit charge against the same battery",
            Math.Abs(ProbePower.RemainingSeconds(vehicle) - remaining * 0.5) < 0.001);
        core.Hold = PowerHold.None;
        core.Mode = PowerSwitchMode.Off;
        backup.Hold = PowerHold.None;
        backup.Mode = PowerSwitchMode.On;
        t.Check("another usable probe extends control when the primary is off",
            Math.Abs(ProbePower.RemainingSeconds(vehicle) - remaining) < 0.001);
        backup.Mode = PowerSwitchMode.Off;
        t.Check("a switched-off probe does not extend the forecast", double.IsNaN(ProbePower.RemainingSeconds(vehicle)));
        core.Mode = PowerSwitchMode.On;
    }

    private static void CheckWarp(TestContext t, Vehicle v)
    {
        Vehicle? focus = Program.ControlledVehicle;
        FieldInfo ambient = typeof(GuidanceWindow).GetField("_s", PrivateStatic)!;
        object? priorAmbient = ambient.GetValue(null);
        FieldInfo limit = typeof(Universe).GetField("_stepLimitTime", PrivateStatic)!;
        object? priorLimit = limit.GetValue(null);
        bool active = GuidanceWindow.ModActive;
        double speed = Universe.GetSimulationSpeed();
        var harmony = new Harmony("com.maxi.afc.harnesstests.probe-power");
        var stockWarning = AccessTools.DeclaredMethod(typeof(Vehicle), "UpdateProbePowerWarning",
            [typeof(int), typeof(ProbeCorePower), typeof(UniverseTime), typeof(UniverseTime)]);
        var moon = typeof(GuidanceWindow).GetMethod("StepMoonWarp", PrivateStatic)!;
        var deorbit = typeof(GuidanceWindow).GetMethod("StepStockDeorbitWarp", PrivateStatic)!;
        VehicleAutopilotState state = VehicleAutopilotState.For(v);
        double now = Universe.GetElapsedSeconds();
        double target = now + 700.0;
        try
        {
            GuidanceWindow.ApplyProbePowerWarpPatch(harmony);
            GuidanceWindow.SetModActive(true);
            Program.ControlledVehicle = v;
            ambient.SetValue(null, state);
            if (!t.Check("the probe craft can accept warp commands", v.IsControllable))
                return;

            state.MoonChain = MoonChainStage.Correction;
            moon.Invoke(null, [v, target]);
            t.Check("the moon chain starts its original warp", Universe.AutoWarpTime == new UniverseTime(target));
            Universe.AutoWarpStop(true);
            moon.Invoke(null, [v, target]);
            t.Check("manual cancel leaves 1x with no probe reoffer",
                !Universe.IsAutoWarpActive && state.MoonChainPowerWarp == ProbeWarpRetry.Spent);

            state.MoonChainPowerWarp = ProbeWarpRetry.None;
            Universe.AutoWarpTo(new UniverseTime(target + 100.0));
            WarnLow();
            moon.Invoke(null, [v, target]);
            t.Check("an interruption on another warp target leaves no offer",
                !Universe.IsAutoWarpActive && state.MoonChainPowerWarp == ProbeWarpRetry.Spent);

            state.MoonChainWarpAsked = false;
            state.MoonChainPowerWarp = ProbeWarpRetry.None;
            state.MoonChainWarpTime = double.NaN;
            moon.Invoke(null, [v, target]);
            WarnLow();
            t.Check("the stock warning interrupts the moon chain's own warp",
                !Universe.IsAutoWarpActive && state.MoonChainPowerWarp == ProbeWarpRetry.Interrupted);
            moon.Invoke(null, [v, target]);
            t.Check("the interruption offers warp while it remains at 1x",
                !Universe.IsAutoWarpActive && state.MoonChainPowerWarp == ProbeWarpRetry.Offered);
            state.MoonChainPowerWarp = ProbeWarpRetry.Requested;
            Universe.ResetSimulationSpeed();
            moon.Invoke(null, [v, target]);
            t.Check("stock 1x cancels a retry before the guidance step consumes it",
                !Universe.IsAutoWarpActive && state.MoonChainPowerWarp == ProbeWarpRetry.Spent);

            state.MoonChainWarpAsked = false;
            state.MoonChainPowerWarp = ProbeWarpRetry.None;
            state.MoonChainWarpTime = double.NaN;
            moon.Invoke(null, [v, target]);
            WarnLow();
            moon.Invoke(null, [v, target]);
            state.MoonChainPowerWarp = ProbeWarpRetry.Requested;
            moon.Invoke(null, [v, target]);
            t.Check("accepting the offer warps to the moon chain's next burn",
                Universe.AutoWarpTime == new UniverseTime(target) && state.MoonChainPowerWarp == ProbeWarpRetry.Spent);
            WarnLow();
            moon.Invoke(null, [v, target]);
            t.Check("a second low warning in the same wait leaves no offer",
                !Universe.IsAutoWarpActive && state.MoonChainPowerWarp == ProbeWarpRetry.Spent);

            state.MoonChain = MoonChainStage.Idle;
            state.LandingPhase = GuidanceWindow.LandingPhase.DeorbitBurn;
            state.DeorbitAutoArmed = true;
            state.DeorbitTarget = new BurnTarget
            {
                IgnitionTime = new UniverseTime(now + 1000.0),
                AlignmentStartTime = new UniverseTime(now + 900.0)
            };
            state.DeorbitWarpRequested = true;
            deorbit.Invoke(null, [v]);
            t.Check("stock deorbit starts the requested warp", Universe.AutoWarpTime == new UniverseTime(now + 890.0));
            WarnLow();
            deorbit.Invoke(null, [v]);
            t.Check("stock deorbit reoffers its own interrupted warp", state.DeorbitPowerWarp == ProbeWarpRetry.Offered);
            PrimarySwitch(v).Mode = PowerSwitchMode.Off;
            state.DeorbitPowerWarp = ProbeWarpRetry.Requested;
            deorbit.Invoke(null, [v]);
            t.Check("the offer cannot start warp after probe power becomes unavailable",
                !Universe.IsAutoWarpActive && state.DeorbitPowerWarp == ProbeWarpRetry.Spent);
        }
        finally
        {
            Universe.AutoWarpStop(false);
            Universe.SetSimulationSpeed(speed, alert: false);
            limit.SetValue(null, priorLimit);
            Program.ControlledVehicle = focus;
            ambient.SetValue(null, priorAmbient);
            VehicleAutopilotState.Remove(v);
            GuidanceWindow.SetModActive(active);
            harmony.UnpatchAll(harmony.Id);
        }

        void WarnLow()
        {
            stockWarning.Invoke(v, [0, ProbeCorePower.Powered, UniverseTime.EndOfTime, UniverseTime.EndOfTime]);
            stockWarning.Invoke(v, [0, ProbeCorePower.Powered, new UniverseTime(299.0), new UniverseTime(500.0)]);
        }
    }

    private static PowerConsumer PrimarySwitch(Vehicle vehicle)
        => vehicle.Parts.PowerConsumers.Modules.ToArray().Single(item => item.PowerSwitch && item.Parent.FullPart == vehicle.Parts.Root);
}

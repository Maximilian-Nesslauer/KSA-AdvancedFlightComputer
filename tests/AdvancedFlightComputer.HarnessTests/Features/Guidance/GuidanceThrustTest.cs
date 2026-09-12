using System.Reflection;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Uses the game's nozzle curve for inversion and synthetic engine states for selection.
// The synthetic objects are never registered with the simulation.
public sealed class GuidanceThrustTest : AfcTest
{
    public override string Name => "afc-guidance-thrust";

    protected override void Execute(TestContext t)
    {
        NozzleCurveRoundTrip(t);
        EngineSelection(t);
        CoreMinimum(t);
    }

    private static void NozzleCurveRoundTrip(TestContext t)
    {
        var nozzle = new DeLavalNozzleConfig
        {
            ThroatArea = 0.01f,
            ExitArea = 0.1f,
            FlowEfficiency = 1f,
            ExpansionEfficiency = 1f,
        };
        var gas = new GasProperties { Gamma = 1.2f, SpecificGasConstant = 350f };
        double Thrust(double throttle, float pressure)
        {
            var chamber = new GasConditions { Pressure = 5000000f * (float)throttle, Temperature = 3000f };
            return nozzle.ComputePerformance(nozzle.ComputeConditions(gas, chamber), pressure).GetTotalThrust();
        }

        foreach (float pressure in new[] { 0f, 101325f, 200000f })
        {
            Func<double, double> curve = throttle => Thrust(throttle, pressure);
            foreach (double target in new[] { 0.25, 0.4, 0.7, 0.95 })
            {
                double demand = curve(target);
                KsaEnginePerf.ThrustCommand result = KsaEnginePerf.InvertThrust(demand, 0.2, curve);
                t.CheckAbs($"round trip at {pressure} Pa, throttle {target}", result.Throttle, target, 2e-6);
                t.CheckAbs($"delivered thrust at {pressure} Pa, throttle {target}",
                    result.DeliveredThrust, demand, Math.Max(0.1, demand * 2e-6));
            }

            KsaEnginePerf.ThrustCommand low = KsaEnginePerf.InvertThrust(curve(0.2) / 2.0, 0.2, curve);
            t.Check($"a demand below the minimum runs at the minimum at {pressure} Pa",
                low.EngineOn && low.Status == KsaEnginePerf.ThrustStatus.BelowMinimum);
            t.CheckAbs($"minimum command at {pressure} Pa", low.Throttle, 0.2, 1e-7);
            t.CheckAbs($"minimum delivered thrust at {pressure} Pa", low.DeliveredThrust, curve(0.2), 0.1);

            KsaEnginePerf.ThrustCommand high = KsaEnginePerf.InvertThrust(curve(1) * 1.2, 0.2, curve);
            t.Check($"a demand above the maximum saturates at {pressure} Pa",
                high.Status == KsaEnginePerf.ThrustStatus.AboveMaximum && high.Throttle == 1);
        }

        t.Check("ambient pressure costs thrust", Thrust(1, 101325) < Thrust(1, 0));
        t.Check("the ambient curve is not linear in throttle",
            Math.Abs(Thrust(0.4, 101325) - 0.4 * Thrust(1, 101325)) > 100);

        KsaEnginePerf.ThrustCommand off = KsaEnginePerf.InvertThrust(
            0, 0.2, _ => throw new InvalidOperationException("an off command evaluated the engine curve"));
        t.Check("off is a separate command, not a small demand",
            !off.EngineOn && off.DeliveredThrust == 0 && off.Status == KsaEnginePerf.ThrustStatus.Off);

        t.Check("a demand that is not a number is refused",
            KsaEnginePerf.InvertThrust(double.NaN, 0.2, _ => 1).Status == KsaEnginePerf.ThrustStatus.InvalidDemand);
        t.Check("an engine with no thrust has no authority",
            KsaEnginePerf.InvertThrust(100, 0.2, _ => 0).Status == KsaEnginePerf.ThrustStatus.NoAuthority);
        t.Check("a curve that is not a number has no authority",
            KsaEnginePerf.InvertThrust(100, 0.2, _ => double.NaN).Status == KsaEnginePerf.ThrustStatus.NoAuthority);

        // Two cores with different floors, so the vehicle floor alone cannot describe the curve.
        KsaEnginePerf.ThrustCommand mixed = KsaEnginePerf.InvertThrust(
            750, 0.1, throttle => 1000 * Math.Max(throttle, 0.2) + 500 * Math.Max(throttle, 0.6));
        t.CheckAbs("cores with different floors", mixed.Throttle, 0.45, 1e-6);
    }

    private static void EngineSelection(TestContext t)
    {
        Vehicle vehicle = Uninitialized<Vehicle>();
        PartTree tree = Uninitialized<PartTree>();
        tree.States = new ModuleStateList();
        tree.EngineThrottleMin = 0.1f;
        vehicle.Parts = tree;

        EngineController active = Engine(1, 1000, isActive: true);
        EngineController inactive = Engine(2, 9000, isActive: false);
        EngineController dry = Engine(3, 7000, isActive: true);
        Add(active, supplied: true);
        Add(inactive, supplied: true);
        Add(dry, supplied: false);

        t.CheckAbs("an inactive and a dry engine are excluded",
            KsaEnginePerf.ActiveThrustCapability(vehicle, 0), 1000, 0.01);
        t.Check("an active liquid engine supports throttle control",
            KsaEnginePerf.SupportsThrottleControl(vehicle));

        SetActive(active, false);
        t.CheckAbs("an attached engine is not fallback authority",
            KsaEnginePerf.AtPressure(vehicle, 0).thrust, 0, 0);
        SetActive(active, true);

        active.Cores = [Uninitialized<SolidMotor>()];
        AddCore(active.Cores[0], 200, supplied: true);
        t.Check("a solid core refuses throttle control", !KsaEnginePerf.SupportsThrottleControl(vehicle));
        t.CheckAbs("a solid engine still counts toward capability",
            KsaEnginePerf.ActiveThrustCapability(vehicle, 0), 1000, 0.01);
        t.Check("a command against a solid core is refused",
            KsaEnginePerf.CommandForThrust(vehicle, 500, 0).Status == KsaEnginePerf.ThrustStatus.UnsupportedEngine);

        ProbeCore suppliedCore = Probe(0.4f);
        ProbeCore dryCore = Probe(0.2f);
        suppliedCore.LastThrottle = dryCore.LastThrottle = -1f;
        active.Cores = [suppliedCore, dryCore];
        AddCore(suppliedCore, 201, supplied: true);
        AddCore(dryCore, 202, supplied: false);

        t.CheckAbs("a partly supplied controller bypasses its full vacuum figure",
            KsaEnginePerf.ActiveThrustCapability(vehicle, 0), 0, 0);
        t.CheckAbs("the supplied core is evaluated", suppliedCore.LastThrottle, 1, 0);
        t.CheckAbs("the dry core is not evaluated", dryCore.LastThrottle, -1, 0);

        KsaEnginePerf.ThrustAtThrottle(vehicle, 0.01, 101325);
        t.CheckAbs("a partly supplied controller keeps the stock core floor",
            suppliedCore.LastThrottle, 0.4, 1e-7);

        BurningSolidBlocksLiquidControl(t);

        void Add(EngineController engine, bool supplied)
        {
            tree.States.AddNew<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>(engine);
            EngineController.TryGetFrom(tree.States, out var states);
            Unsafe.AsRef(in states.States[engine.StatesIdx]).IsPropellantAvailable = supplied;
            AddCore(engine.Cores[0], engine.InstanceId + 100, supplied);
        }

        void AddCore(RocketCore core, ulong id, bool supplied)
        {
            typeof(ModuleBase).GetField(nameof(ModuleBase.InstanceId))!.SetValue(core, id);
            tree.States.AddNew<RocketCore, RocketCoreState, RocketCoreGlobalState, EmptyStruct>(core);
            RocketCore.TryGetFrom(tree.States, out var states);
            Unsafe.AsRef(in states.States[core.StatesIdx]).IsPropellantAvailable = supplied;
        }
    }

    // A motor lit before its controller was switched off keeps burning, so it still adds
    // thrust that nothing here commands. An unlit one on the same inactive controller is
    // inert and must not cost the liquid engine its control.
    private static void BurningSolidBlocksLiquidControl(TestContext t)
    {
        Vehicle vehicle = Uninitialized<Vehicle>();
        PartTree tree = Uninitialized<PartTree>();
        tree.States = new ModuleStateList();
        tree.EngineThrottleMin = 0.1f;
        vehicle.Parts = tree;

        EngineController liquid = Engine(11, 1000, isActive: true);
        tree.States.AddNew<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>(liquid);
        EngineController.TryGetFrom(tree.States, out var liquidStates);
        Unsafe.AsRef(in liquidStates.States[liquid.StatesIdx]).IsPropellantAvailable = true;
        AddCoreState(tree, liquid.Cores[0], 300, supplied: true);

        EngineController solid = Engine(12, 4000, isActive: false);
        solid.Cores = [Uninitialized<SolidMotor>()];
        tree.States.AddNew<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>(solid);
        EngineController.TryGetFrom(tree.States, out var solidStates);
        Unsafe.AsRef(in solidStates.States[solid.StatesIdx]).IsPropellantAvailable = true;
        AddCoreState(tree, solid.Cores[0], 301, supplied: true);

        t.Check("an unlit solid on an inactive controller leaves liquid control alone",
            KsaEnginePerf.GetThrottleControlStatus(vehicle) == KsaEnginePerf.ThrustStatus.Available);

        RocketCore.TryGetFrom(tree.States, out var cores);
        Unsafe.AsRef(in cores.States[solid.Cores[0].StatesIdx]).Throttle = 1f;
        t.Check("a burning solid refuses control even with its controller switched off",
            KsaEnginePerf.GetThrottleControlStatus(vehicle) == KsaEnginePerf.ThrustStatus.UnsupportedEngine);
        t.Check("and the command is refused with it",
            KsaEnginePerf.CommandForThrust(vehicle, 500, 0).Status == KsaEnginePerf.ThrustStatus.UnsupportedEngine);
    }

    private static void AddCoreState(PartTree tree, RocketCore core, ulong id, bool supplied)
    {
        typeof(ModuleBase).GetField(nameof(ModuleBase.InstanceId))!.SetValue(core, id);
        tree.States.AddNew<RocketCore, RocketCoreState, RocketCoreGlobalState, EmptyStruct>(core);
        RocketCore.TryGetFrom(tree.States, out var states);
        Unsafe.AsRef(in states.States[core.StatesIdx]).IsPropellantAvailable = supplied;
    }

    private static void CoreMinimum(TestContext t)
    {
        ProbeCore core = Probe(0.4f);
        RocketControllerData.ComputeFromCores(new RocketCore[] { core }, float3.Zero, 101325f, 0.1f);
        t.CheckAbs("the stock computation applies the per-core floor", core.LastThrottle, 0.4, 1e-7);
    }

    private static ProbeCore Probe(float minimum)
    {
        ProbeCore core = Uninitialized<ProbeCore>();
        core.MinimumThrottle = minimum;
        core.Rocket = Uninitialized<Rocket>();
        core.Rocket.Nozzles = [];
        return core;
    }

    private static EngineController Engine(ulong id, float thrust, bool isActive)
    {
        EngineController engine = Uninitialized<EngineController>();
        typeof(ModuleBase).GetField(nameof(ModuleBase.InstanceId))!.SetValue(engine, id);
        engine.Cores = [Uninitialized<Combustor>()];
        engine.VacuumData = new RocketControllerData
        {
            ThrustMax = new float3(thrust, 0, 0),
            MassFlowRateMax = thrust / 3000,
            MinimumThrottle = 0.2f,
        };
        SetActive(engine, isActive);
        return engine;
    }

    private static void SetActive(EngineController engine, bool isActive)
        => typeof(EngineController).GetProperty(nameof(EngineController.IsActive))!.SetValue(engine, isActive);

    // Records the throttle passed by RocketControllerData.ComputeFromCores.
    // Other operations throw because this probe cannot simulate a live engine.
    private sealed class ProbeCore : RocketCore
    {
        public float LastThrottle;

        private ProbeCore() : base("probe") { }

        public override float MaxChamberPressure => 1;

        public override RocketCoreConditions ComputeConditions(float throttle)
        {
            LastThrottle = throttle;
            return RocketCoreConditions.Zero;
        }

        public override RocketCoreConditions ComputeDesignConditions() => throw new NotSupportedException();

        public override bool ComputePropellantAvailable(ReadOnlySpan<MoleState> states, bool isBurning)
            => throw new NotSupportedException();

        public override void ConsumePropellant(Span<MoleState> states, float mass)
            => throw new NotSupportedException();

        public override bool TryPrepareDrain(CoreDrainState slot, in DrainContext context)
            => throw new NotSupportedException();

        public override bool TryAccumulateDrain(CoreDrainState slot, in DrainContext context)
            => throw new NotSupportedException();
    }

    // Constructors are skipped, so finalizers must not access the incomplete state.
    private static T Uninitialized<T>() where T : class
    {
        var value = (T)RuntimeHelpers.GetUninitializedObject(typeof(T));
        GC.SuppressFinalize(value);
        return value;
    }
}

using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using HarmonyLib;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The stock structural limit of a real craft is far above what its engines reach, so the check lowers
// FlightComputer.MaxGLoad on the main-thread copy KsaEnginePerf reads. No step runs while it is lowered,
// because FlightComputer.ReadMeasurements sets it again from VehicleStructuralLimits.EffectiveMaxGLoad on the worker.
// The planners receive the same limit as an acceleration bound and plan with the uncapped thrust beside it.
public sealed class GuidanceGLoadCapTest : AfcTest
{
    private static readonly AccessTools.FieldRef<VehicleAutopilotState> AmbientState =
        AccessTools.StaticFieldRefAccess<VehicleAutopilotState>(
            AccessTools.Field(typeof(GuidanceWindow), "_s"));

    public override string Name => "afc-guidance-gload-cap";

    // Where FlightComputer.SolveGLoadThrottleCap holds the thrust, as a fraction of the full thrust,
    // inside the throttle range and clear of the engine minimum.
    private const double CapFraction = 0.6;
    private const double StockGLoadMargin = 0.9;
    private const double StandardGravity = 9.80665;

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

        string vehicleId = "GLoadCap_" + Guid.NewGuid().ToString("N");
        Vehicle? vehicle = null;
        float realMaxGLoad = float.NaN;
        VehicleAutopilotState previousAmbient = AmbientState();
        try
        {
            vehicle = VehicleFixtures.SpawnDesign(t.System, home,
                save.VehicleSaveData.RootPartInstance, vehicleId,
                OrbitFixtures.CircularAt(home, 500_000, Universe.GetElapsedTime()));
            SimDriver driver = t.Session.CreateDriver();
            driver.Step(0.05, 2);
            AutoStageFlightSupport.IgniteFirstStage(vehicle, driver);

            FlightComputer fc = vehicle.FlightComputer;
            double pressure = fc.AmbientPressure;
            double massKg = fc.TotalMassPropsBody.Mass;
            double fullN = KsaEnginePerf.ThrustAtThrottle(vehicle, 1.0, pressure);
            if (!t.Check("the first stage has thrust and the flight computer has measured the mass",
                    fullN > 0.0 && massKg > 0.0, $"thrust {fullN:F0} N, mass {massKg:F0} kg"))
                return;
            t.Check("the real structural limit leaves the throttle open",
                KsaEnginePerf.StructuralThrottleCap(vehicle, pressure) == 1.0,
                $"MaxGLoad {fc.MaxGLoad:F1} g");

            realMaxGLoad = fc.MaxGLoad;
            fc.MaxGLoad = (float)(CapFraction * fullN / (StockGLoadMargin * StandardGravity * massKg));
            double limitN = StockGLoadMargin * fc.MaxGLoad * StandardGravity * massKg;
            double cap = KsaEnginePerf.StructuralThrottleCap(vehicle, pressure);
            t.Check("a lower limit caps the throttle", cap < 1.0 && cap > vehicle.GetMinThrottle(),
                $"cap {cap:F4}, minimum {vehicle.GetMinThrottle():F4}");
            t.CheckRel("full-throttle capability stops at the structural limit",
                KsaEnginePerf.ActiveThrustCapability(vehicle, pressure), limitN, 1e-2);

            KsaEnginePerf.ThrustCommand above = KsaEnginePerf.CommandForThrust(vehicle, 2.0 * limitN, pressure);
            t.Check("a demand above the limit saturates at the cap",
                above.Status == KsaEnginePerf.ThrustStatus.AboveMaximum && Math.Abs(above.Throttle - cap) < 1e-6,
                $"status {above.Status}, throttle {above.Throttle:F6}, cap {cap:F6}");
            KsaEnginePerf.ThrustCommand below = KsaEnginePerf.CommandForThrust(vehicle, 0.5 * limitN, pressure);
            t.Check("a demand under the limit stays below the cap",
                below.Status == KsaEnginePerf.ThrustStatus.Available && below.Throttle < cap,
                $"status {below.Status}, throttle {below.Throttle:F6}");
            t.CheckRel("a demand under the limit is delivered", below.DeliveredThrust, 0.5 * limitN, 1e-4);

            CheckPlanningLimit(t, vehicle, fc, pressure, fullN);
        }
        finally
        {
            AmbientState() = previousAmbient;
            if (vehicle != null)
            {
                if (float.IsFinite(realMaxGLoad))
                    vehicle.FlightComputer.MaxGLoad = realMaxGLoad;
                VehicleSpawner.Despawn(vehicle);
            }
        }
    }

    // The planners' side of the same limit, with FlightComputer.MaxGLoad still lowered.
    private static void CheckPlanningLimit(TestContext t, Vehicle vehicle, FlightComputer fc, double pressure, double fullN)
    {
        double accelLimit = KsaEnginePerf.StructuralAccelerationLimit(vehicle);
        t.CheckRel("the planning limit is the throttle cap as an acceleration",
            accelLimit, (double)(0.9f * fc.MaxGLoad * 9.80665f), 1e-9);
        t.CheckRel("planners see the uncapped full thrust", KsaEnginePerf.UncappedAtPressure(vehicle, pressure).thrust, fullN, 1e-9);

        // The player's g-limit wins only where it is lower.
        AmbientState() = new VehicleAutopilotState { GLimitEnabled = true, GLimitG = 1000.0 };
        t.CheckRel("a high g-limit leaves the structural limit", GuidanceWindow.EffectiveAccelLimitG(vehicle),
            accelLimit / StandardGravity, 1e-12);
        double lowG = 0.5 * accelLimit / StandardGravity;
        AmbientState() = new VehicleAutopilotState { GLimitEnabled = true, GLimitG = lowG };
        t.CheckRel("a lower g-limit takes over", GuidanceWindow.EffectiveAccelLimitG(vehicle), lowG, 1e-12);
        AmbientState() = new VehicleAutopilotState { GLimitEnabled = false, GLimitG = lowG };
        t.CheckRel("a g-limit switched off does not", GuidanceWindow.EffectiveAccelLimitG(vehicle),
            accelLimit / StandardGravity, 1e-12);

        // UPFG: a stage that reaches the limit mid-burn is split where full thrust meets it, and flies its tail at constant acceleration.
        double massAtLimit = fullN / accelLimit;
        var model = new UpfgVehicle();
        model.Stages.Add(new UpfgStage { Thrust = fullN, Isp = 300.0, MassTotal = 1.5 * massAtLimit, MassDry = 0.5 * massAtLimit, GLim = 1e9 });
        GuidanceWindow.ApplyGLimit(model, accelLimit / StandardGravity);
        t.Check("UPFG splits the stage at the limit",
            model.Stages.Count == 2 && model.Stages[0].Mode == 1 && model.Stages[1].Mode == 2
            && Math.Abs(model.Stages[0].MassDry - massAtLimit) < 1e-6 * massAtLimit,
            string.Join(", ", model.Stages.Select(s => $"mode {s.Mode} {s.MassTotal:F0}->{s.MassDry:F0} kg")));
    }
}

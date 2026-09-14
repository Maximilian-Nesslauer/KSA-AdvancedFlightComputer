using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AdvancedFlightComputer.HarnessTests.Framework;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The oracle is the thrust PartTree.RecomputeRocketControls stores per engine at 0 Pa (VacuumData)
// and 101325 Pa (SeaLevelData), summed over the engines the drain simulation puts in the burning
// phase. That set includes boosters that are not lit, because the model counts them. The check runs
// again after a burn, because a correction from live chamber conditions would drift with the grain.
public sealed class GuidanceAmbientThrustTest : AfcTest
{
    public override string Name => "afc-guidance-ambient-thrust";

    private static readonly string[] DefaultSaves = { "Test Vehicle 2", "Test Vehicle 1" };
    private const double SeaLevelPressure = 101325.0;
    private const double G0 = 9.80665;
    private const double BurnDt = 0.5;
    private const double BurnSeconds = 30.0;

    protected override void Execute(TestContext t)
    {
        IReadOnlyList<string> saves = TestSupport.ResolveVehicleSaves(DefaultSaves);
        if (saves.Count == 0)
        {
            t.Skip($"none of '{string.Join("', '", DefaultSaves)}' is in the game's Vehicles folder.");
            return;
        }

        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        Vehicle? previousFocus = Program.ControlledVehicle;
        try
        {
            Vehicle vehicle;
            try
            {
                vehicle = AutoStageFlightSupport.SpawnFromSave(t, saves[0], "AfcGuidanceAmbientThrust", out _);
            }
            catch (InvalidOperationException e)
            {
                t.Fail("spawn", e.Message);
                return;
            }

            PhysicsBubble._forceOffRails = true;
            Program.ControlledVehicle = vehicle;
            driver.Step(0.05, 40);

            // Not IgniteFirstStage, which does nothing once any engine is active, as in these saves,
            // and would leave the boosters unlit.
            vehicle.Parts.SequenceList.ActivateNextSequence(vehicle);
            driver.Step(1.0);
            if (!Compare(t, vehicle, "at ignition"))
                return;

            // A manual burn has no g-load cap, so the core is held low. Lit solids ignore the throttle.
            AutoStageFlightSupport.HoldPrograde(vehicle, 0.3f);
            for (double burnt = 0.0; burnt < BurnSeconds; burnt += BurnDt)
                driver.Step(BurnDt);
            Compare(t, vehicle, $"after {BurnSeconds:F0} s of burn");
        }
        finally
        {
            AutoStageFlightSupport.CleanupAfterFlight(t.System, preexisting);
            Program.ControlledVehicle = previousFocus;
        }
    }

    private static bool Compare(TestContext t, Vehicle vehicle, string when)
    {
        if (!StagingHelpers.HasActiveEngineWithPropellant(vehicle))
        {
            t.Skip($"{when}: no engine is lit with propellant, so there is no pressure response to measure.");
            return false;
        }
        if (KsaVehicleAdapter.AnyAtmosphericSequence(vehicle))
        {
            t.Skip($"{when}: the save computes a sequence at sea level, so neither build is the uncorrected one.");
            return false;
        }

        // One recompute for the oracle and every build, with no step between, so only the pressure differs.
        vehicle.Parts.PerformanceSequences.RecomputeForFlight(0f);
        HashSet<Part>? burning = BurningPhaseParts(vehicle);
        if (burning == null || burning.Count == 0)
        {
            t.Skip($"{when}: the drain simulation attributes no engine to the burn in progress.");
            return false;
        }
        (double gameVacuum, double gameSeaLevel, int engines) = PhaseEngineThrust(burning);
        if (!(gameVacuum > 0.0) || !(gameSeaLevel > 0.0) || gameSeaLevel >= gameVacuum)
        {
            t.Skip($"{when}: the burning stage's engines lose no thrust at sea level, so the correction cannot be measured.");
            return false;
        }
        double expected = gameSeaLevel / gameVacuum;

        UpfgVehicle vacuum = KsaVehicleAdapter.Build(vehicle, 0.0);
        UpfgVehicle seaLevel = KsaVehicleAdapter.Build(vehicle, SeaLevelPressure);
        double livePressure = vehicle.PhysicsEnvironment.AtmosphericPressure;
        UpfgVehicle live = KsaVehicleAdapter.Build(vehicle, livePressure);
        t.Info($"{when}: {engines} engine(s) on {burning.Count} part(s) in the burning stage, the game's stored "
             + $"figures {gameVacuum / 1000.0:F1} kN in vacuum and {gameSeaLevel / 1000.0:F1} kN at sea level, so the "
             + $"response is {expected:F4}; the model plans that stage on "
             + $"{(vacuum.Stages.Count > 0 ? vacuum.Stages[0].Thrust / 1000.0 : 0.0):F1} kN; "
             + $"mass {vehicle.TotalMass / 1000.0:F1} t, craft pressure {livePressure:F3} Pa");
        t.Info($"{when}: vacuum model {GuidanceLog.DescribeStages(vacuum)}");
        t.Info($"{when}: sea level model {GuidanceLog.DescribeStages(seaLevel)}");
        if (!t.Check($"{when}: the drain simulation yields a stage list to plan on",
                vacuum.Stages.Count > 0 && seaLevel.Stages.Count == vacuum.Stages.Count,
                $"{vacuum.Stages.Count} stage(s) in vacuum, {seaLevel.Stages.Count} at sea level"))
            return false;

        UpfgStage vacuumFirst = vacuum.Stages[0], seaLevelFirst = seaLevel.Stages[0];
        double ratio = seaLevelFirst.Thrust / vacuumFirst.Thrust;
        t.Check($"{when}: back pressure never adds thrust to the burning stage", ratio <= 1.0,
            $"{vacuumFirst.Thrust / 1000.0:F1} kN in vacuum, {seaLevelFirst.Thrust / 1000.0:F1} kN at sea level");
        t.CheckRel($"{when}: the burning stage follows the engines' own pressure response", ratio, expected, 1e-2);
        t.CheckRel($"{when}: the exhaust velocity moves with the thrust",
            seaLevelFirst.Isp / vacuumFirst.Isp, expected, 1e-2);
        t.CheckRel($"{when}: the burning stage keeps its mass flow",
            seaLevelFirst.Thrust / (seaLevelFirst.Isp * G0), vacuumFirst.Thrust / (vacuumFirst.Isp * G0), 1e-9);
        t.Check($"{when}: the burning stage keeps the propellant the drain simulation gave it",
            seaLevelFirst.MassTotal == vacuumFirst.MassTotal && seaLevelFirst.MassDry == vacuumFirst.MassDry,
            $"{vacuumFirst.MassTotal:F1} to {vacuumFirst.MassDry:F1} kg in vacuum, "
            + $"{seaLevelFirst.MassTotal:F1} to {seaLevelFirst.MassDry:F1} kg at sea level");
        t.CheckRel($"{when}: the model records what it corrected the burning stage by",
            seaLevel.BurningStageThrustRatio, ratio, 1e-9);
        t.Check($"{when}: the vacuum model records no correction",
            vacuum.BurningStageThrustRatio == 1.0, $"{vacuum.BurningStageThrustRatio:F6}");

        bool laterUntouched = true;
        for (int i = 1; i < vacuum.Stages.Count; i++)
        {
            UpfgStage a = vacuum.Stages[i], b = seaLevel.Stages[i];
            laterUntouched &= a.Thrust == b.Thrust && a.Isp == b.Isp
                && a.MassTotal == b.MassTotal && a.MassDry == b.MassDry;
        }
        if (vacuum.Stages.Count > 1)
            t.Check($"{when}: the later stages keep the environment they burn in", laterUntouched);
        else
            t.Info($"{when}: the craft has one stage, so nothing later could be compared.");

        t.Check($"{when}: the craft is where there is no atmosphere left", livePressure < 1.0, $"{livePressure:F3} Pa");
        t.Check($"{when}: the craft's own pressure leaves the model alone in vacuum",
            live.Stages.Count == vacuum.Stages.Count
            && live.Stages[0].Thrust == vacuumFirst.Thrust && live.Stages[0].Isp == vacuumFirst.Isp);
        return true;
    }

    // The first phase with thrust, flow and duration, which is the one the adapter scales.
    private static HashSet<Part>? BurningPhaseParts(Vehicle vehicle)
    {
        SequencePerformanceList? performanceList = vehicle.Parts?.PerformanceSequences;
        if (performanceList == null)
            return null;
        ReadOnlySpan<SequencePerformance> performance = performanceList.PerformanceSequences;
        for (int i = 0; i < performance.Length; i++)
        {
            SequencePerformance perf = performance[i];
            List<SequencePhaseInfo>? phases = perf.Phases;
            List<HashSet<Part>>? parts = perf.PhaseEngineParts;
            if (phases == null || parts == null)
                continue;
            for (int j = 0; j < phases.Count && j < parts.Count; j++)
            {
                SequencePhaseInfo phase = phases[j];
                if (phase.Thrust > 0f && phase.MassFlowRate > 0f && phase.Duration >= 1e-3f)
                    return parts[j];
            }
        }
        return null;
    }

    private static (double vacuum, double seaLevel, int count) PhaseEngineThrust(HashSet<Part> parts)
    {
        double vacuum = 0.0, seaLevel = 0.0;
        int count = 0;
        foreach (Part part in parts)
        {
            Span<EngineController> engines = part.Modules.Get<EngineController>();
            for (int i = 0; i < engines.Length; i++)
            {
                count++;
                vacuum += engines[i].VacuumData.ThrustMax.Length();
                seaLevel += engines[i].SeaLevelData.ThrustMax.Length();
            }
        }
        return (vacuum, seaLevel, count);
    }
}

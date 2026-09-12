using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.HarnessTests.Framework;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// A decoupler-only sequence must fire its decoupler DecouplerDelayS after it activates, and the
// engine sequence after it must ignite EngineDelayS after its own activation. The delays are
// injected into the in-memory config per part variant.
//
// Needs a save with a decoupler-only sequence after the launch stage and an engine sequence after
// that; anything else skips. What is measured is the delay between a sequence activating and its
// parts firing, so how the activation came about does not matter. Reaching it gets a generous cap,
// the delay that follows gets a tight one.
public sealed class StagingDelayTest : AfcTest
{
    private const double DecouplerDelayS = 3.0;
    private const double EngineDelayS = 5.0;
    // The countdown ticks once per solver step and the fired activation lands through the input
    // queue on the following step.
    private const double DelayTolS = 1.5;
    private const double MeasureDt = 0.5;
    private const double MaxStagingSeconds = 900.0;
    private const double MaxPhaseSeconds = 30.0;
    // Part throttle, because a full-throttle stack that survives several stagings runs itself past
    // VehicleStructuralLimits.EffectiveMaxGLoad and is destroyed mid-test.
    private const float Throttle = 0.4f;

    public override string Name => "afc-autostage-delays";

    protected override void Execute(TestContext t)
    {
        string? saveId = Environment.GetEnvironmentVariable(TestSupport.VehicleEnvVar);
        if (string.IsNullOrEmpty(saveId))
        {
            t.Skip($"{TestSupport.VehicleEnvVar} not set");
            return;
        }

        HashSet<string> preexisting = TestSupport.CollectVehicleIds(t.System);
        SimDriver driver = t.Session.CreateDriver();
        double time = 0.0;
        using AutoStageTestPatches.Scope patches = AutoStageTestPatches.Apply();
        try
        {
            Vehicle vehicle;
            try
            {
                vehicle = AutoStageFlightSupport.SpawnFromSave(t, saveId, "AfcAutoStageDelay", out _);
            }
            catch (InvalidOperationException e)
            {
                t.Fail("spawn", e.Message);
                return;
            }

            PhysicsBubble._forceOffRails = true;
            Program.ControlledVehicle = vehicle;
            StagingConfig.DropSpentStages = StagingConfig.DropSpentStagesDefault;

            if (!TryFindDelaySequences(vehicle, out Sequence? decouplerSeq, out Sequence? engineSeq))
            {
                t.Skip("the save has no decoupler-only sequence followed by an engine sequence");
                return;
            }
            ConfigureDelays(vehicle, decouplerSeq!, engineSeq!);
            t.Info($"decoupler sequence {decouplerSeq!.Number} delayed {DecouplerDelayS:F1}s, " +
                   $"engine sequence {engineSeq!.Number} delayed {EngineDelayS:F1}s");

            if (!AutoStageFlightSupport.Arm(t, vehicle))
                return;
            AutoStageFlightSupport.HoldPrograde(vehicle, Throttle);
            AutoStageFlightSupport.IgniteFirstStage(vehicle, driver);

            bool StepUntil(Func<bool> condition, double capSeconds, string what)
            {
                double deadline = time + capSeconds;
                while (!condition())
                {
                    if (time >= deadline)
                        return t.Fail(what, $"timed out after {capSeconds:F0}s");
                    driver.Step(MeasureDt);
                    time += MeasureDt;
                }
                return true;
            }

            // The Sequence object, never its number: SequenceList.Remove renumbers, and
            // RemoveSpentSequences runs inside ActivateNextSequence and Vehicle.Split.
            if (!StepUntil(() => decouplerSeq!.Activated, MaxStagingSeconds, "the decoupler sequence activates"))
                return;
            double tDecouplerSeq = time;
            StructuralLoad loadBeforeSplit = vehicle.StructuralLoad;

            // The vehicle's own part count, not the system-wide vehicle count: shed boosters can be
            // destroyed on the frame they separate, which leaves the vehicle count flat.
            int partsBefore = vehicle.Parts.Count;
            if (!StepUntil(() => vehicle.IsDisposed || vehicle.Parts.Count < partsBefore, MaxPhaseSeconds, "the decoupler split"))
            {
                LogSequenceState(t, vehicle, decouplerSeq!);
                return;
            }
            if (vehicle.IsDisposed)
            {
                t.Fail("the vehicle survives the decoupler split",
                    $"destroyed with {DescribeLoad(loadBeforeSplit)} before it; the scenario, not the delay, is at fault, lower the throttle");
                return;
            }
            double splitDelay = time - tDecouplerSeq;

            if (!StepUntil(() => engineSeq!.Activated, MaxStagingSeconds, "the engine sequence activates"))
                return;
            double tEngineSeq = time;

            // The ignition check is vehicle-wide, so it only measures this sequence while nothing else burns.
            if (StagingHelpers.HasActiveEngineWithPropellant(vehicle))
            {
                t.Fail("the vehicle is dry when the engine sequence activates",
                    "still under thrust, so the ignition delay cannot be measured");
                return;
            }
            if (!StepUntil(() => StagingHelpers.HasActiveEngineWithPropellant(vehicle), MaxPhaseSeconds, "upper-stage ignition"))
                return;
            double igniteDelay = time - tEngineSeq;

            t.CheckAbs("decoupler fires after its delay", splitDelay, DecouplerDelayS, DelayTolS);
            t.CheckAbs("engine ignites after its delay", igniteDelay, EngineDelayS, DelayTolS);
        }
        finally
        {
            AutoStageFlightSupport.CleanupAfterFlight(t.System, preexisting);
        }
    }

    private static string DescribeLoad(in StructuralLoad load) =>
        $"g-load {load.PeakGLoad:F1}/{load.MaxGLoad:F1} ({load.GLoadFraction:P0} of the limit), " +
        $"dynamic pressure {load.DynamicPressureFraction:P0} of the limit";

    // A timeout means the sequence never carried the decouplers the search picked it for, or the
    // configured delay was not found for it. Both look the same from outside, so name them.
    private static void LogSequenceState(TestContext t, Vehicle vehicle, Sequence seq)
    {
        t.Info($"decoupler sequence at timeout: number={seq.Number}, activated={seq.Activated}, parts={seq.Parts.Length}, " +
               $"configuredDelay={StagingConfig.GetSequenceDecouplerDelay(vehicle, seq.Number):F1}s, " +
               $"nextSequence={vehicle.Parts.SequenceList.GetNextSequenceNumber()}");
        ReadOnlySpan<Part> parts = seq.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            foreach (ISequenced module in parts[i].InSequence(seq.Number))
            {
                if (module is not Decoupler decoupler)
                    continue;
                t.Info($"  '{parts[i].Id}' {SequencedModules.Describe(module)} active={decoupler.IsActive} " +
                       $"enabled={decoupler.IsEnabled} connected={decoupler.Connector.Connection != null} " +
                       $"delayKey={SequencedModules.DelayKey(module)}");
            }
        }
    }

    // Engines (the launch stage), then a decoupler-only sequence, then an engine sequence, so each
    // delay is observable in isolation.
    private static bool TryFindDelaySequences(Vehicle vehicle, out Sequence? decouplerSeq, out Sequence? engineSeq)
    {
        decouplerSeq = null;
        engineSeq = null;
        bool sawLaunchEngines = false;
        foreach (Sequence seq in vehicle.Parts.SequenceList.Sequences)
        {
            if (seq.Activated || seq.Parts.IsEmpty)
                continue;
            bool hasEngine = false;
            bool hasDecoupler = false;
            ReadOnlySpan<Part> parts = seq.Parts;
            for (int i = 0; i < parts.Length; i++)
            {
                foreach (ISequenced module in parts[i].InSequence(seq.Number))
                {
                    hasEngine |= module is EngineController;
                    hasDecoupler |= module is Decoupler;
                }
            }

            if (!sawLaunchEngines)
            {
                sawLaunchEngines = hasEngine;
            }
            else if (decouplerSeq == null)
            {
                if (hasDecoupler && !hasEngine)
                    decouplerSeq = seq;
            }
            else if (hasEngine && !hasDecoupler)
            {
                engineSeq = seq;
                return true;
            }
        }
        return false;
    }

    // Keyed the way the detector resolves a delay, so staging looks up the value set here.
    private static void ConfigureDelays(Vehicle vehicle, Sequence decouplerSeq, Sequence engineSeq)
    {
        StagingConfig.EngineDelays.Clear();
        StagingConfig.DecouplerDelays.Clear();
        foreach (Sequence seq in vehicle.Parts.SequenceList.Sequences)
        {
            if (seq != decouplerSeq && seq != engineSeq)
                continue;
            ReadOnlySpan<Part> parts = seq.Parts;
            for (int i = 0; i < parts.Length; i++)
            {
                foreach (ISequenced module in parts[i].InSequence(seq.Number))
                {
                    if (seq == decouplerSeq && module is Decoupler)
                        StagingConfig.DecouplerDelays[SequencedModules.DelayKey(module)] = DecouplerDelayS;
                    if (seq == engineSeq && module is EngineController)
                        StagingConfig.EngineDelays[SequencedModules.DelayKey(module)] = EngineDelayS;
                }
            }
        }
    }
}

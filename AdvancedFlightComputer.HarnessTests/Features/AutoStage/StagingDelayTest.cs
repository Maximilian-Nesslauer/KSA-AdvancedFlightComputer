using AutoStage.Core;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AutoStage.HarnessTests;

// Verifies the delayed split staging: a decoupler-only sequence must fire its decoupler
// DecouplerDelayS after the sequence activates, and the following engine sequence must ignite
// EngineDelayS after ITS activation, with the burn resuming afterwards. The delays are injected
// into the in-memory global config (per part variant); CleanupAfterFlight reloads the user's file.
//
// Needs a save with a decoupler-only sequence somewhere after the launch stage and an engine
// sequence after that; anything else skips. The vehicle comes from KSA_HEADLESS_VEHICLE, shared
// with the harness flight test.
//
// What is measured is the delay between a sequence activating and its parts firing, so how the
// sequence came to activate does not matter and the staging triggers are left at their shipped
// settings. Reaching an activation depends on burn durations and gets a generous cap; the delay
// that follows it is the actual assertion and gets a tight one.
public sealed class DelayTest : IHarnessTest
{
    private const double DecouplerDelayS = 3.0;
    private const double EngineDelayS = 5.0;
    // The countdown ticks once per solver step and the fired activation lands via the input queue
    // on the following step, so the observed delay runs a step or two past the configured one.
    private const double DelayTolS = 1.5;
    private const double MeasureDt = 0.5;
    private const double MaxStagingSeconds = 900.0;
    private const double MaxPhaseSeconds = 30.0;
    // Part throttle, unlike the other flying tests. Those burn a stage and end; this one has to
    // survive several stagings to reach the sequence pair it measures, and a full-throttle stack
    // that keeps shedding mass runs itself past VehicleStructuralLimits.EffectiveMaxGLoad and is
    // destroyed mid-test. The delays being measured do not depend on the throttle.
    private const float Throttle = 0.4f;

    public string Name => "autostage-delays";

    public int Run(HeadlessSession session)
    {
        string? saveId = Environment.GetEnvironmentVariable(TestSupport.VehicleEnvVar);
        if (string.IsNullOrEmpty(saveId))
        {
            HarnessLog.Line($"[autostage-delays] SKIP: {TestSupport.VehicleEnvVar} not set.");
            return 0;
        }

        AutoStageHost.EnsureInitialized();
        if (!AutoStageHost.CoreOk || !Mod.IgnitionDelayAvailable)
        {
            HarnessLog.Line("[autostage-delays] FAIL: AutoStage delay patches are not active (see autostage-api-drift).");
            return 1;
        }

        CelestialSystem system = session.System;
        HashSet<string> preexisting = TestSupport.CollectVehicleIds(system);
        SimDriver driver = session.CreateDriver();
        double t = 0.0;
        bool ok = true;
        try
        {
            // Inside the try: Astronomical's constructor registers with the system before the
            // spawner finishes, so a throw part-way still leaves a vehicle for cleanup to remove.
            Vehicle vehicle;
            try
            {
                vehicle = AutoStageHost.SpawnFromSave(session, saveId, "AutoStageDelayTest", out _);
            }
            catch (InvalidOperationException e)
            {
                HarnessLog.Line($"[autostage-delays] FAIL: {e.Message}");
                return 1;
            }

            PhysicsBubble._forceOffRails = true;
            Program.ControlledVehicle = vehicle;

            // Pinned to the shipped default rather than the local file, so the run covers the same
            // trigger set a player gets whatever autostage.toml says. CleanupAfterFlight reloads it.
            Config.DropSpentStages = Config.DropSpentStagesDefault;

            if (!TryFindDelaySequences(vehicle, out Sequence? decouplerSeq, out Sequence? engineSeq))
            {
                HarnessLog.Line("[autostage-delays] SKIP: the save has no decoupler-only sequence " +
                                "followed by an engine sequence.");
                return 0;
            }
            ConfigureDelays(vehicle, decouplerSeq!, engineSeq!);
            HarnessLog.Line($"[autostage-delays] decoupler sequence {decouplerSeq!.Number} delayed {DecouplerDelayS:F1}s, " +
                            $"engine sequence {engineSeq!.Number} delayed {EngineDelayS:F1}s.");

            vehicle.ToggleEnum(AutoStageToggle.Enabled);
            if (!Mod.AutoStageEnabled)
            {
                HarnessLog.Line("[autostage-delays] FAIL: ToggleEnum(AutoStageToggle) did not enable the mod.");
                return 1;
            }
            AutoStageHost.HoldPrograde(vehicle, Throttle);
            AutoStageHost.IgniteFirstStage(vehicle, driver);

            bool StepUntil(Func<bool> condition, double capSeconds, string what)
            {
                double deadline = t + capSeconds;
                while (!condition())
                {
                    if (t >= deadline)
                    {
                        HarnessLog.Line($"[autostage-delays] FAIL: timed out after {capSeconds:F0}s waiting for {what}.");
                        return false;
                    }
                    driver.Step(MeasureDt);
                    t += MeasureDt;
                }
                return true;
            }

            // The Sequence object, never its number: SequenceList.Remove decrements every number at
            // or above a removed sequence, and RemoveSpentSequences runs inside both
            // ActivateNextSequence and Vehicle.Split, so a number captured here can name a different
            // sequence by the time the staging lands.
            if (!StepUntil(() => decouplerSeq!.Activated,
                    MaxStagingSeconds, "the decoupler sequence to activate"))
                return 1;
            double tDecouplerSeq = t;
            StructuralLoad loadBeforeSplit = vehicle.StructuralLoad;

            // The part count on the vehicle itself, not the system-wide vehicle count: shed
            // boosters can be destroyed on the same frame they separate (a radial stack drops
            // four of them into each other), which leaves the net count flat or falling while
            // the decouplers did fire exactly on time.
            int partsBefore = vehicle.Parts.Count;
            if (!StepUntil(() => vehicle.IsDisposed || vehicle.Parts.Count < partsBefore,
                    MaxPhaseSeconds, "the decoupler split"))
            {
                LogSequenceState(vehicle, decouplerSeq!, "decoupler sequence at timeout");
                return 1;
            }
            if (vehicle.IsDisposed)
            {
                HarnessLog.Line("[autostage-delays] FAIL: the vehicle was destroyed during the " +
                                $"decoupler split ({DescribeLoad(loadBeforeSplit)} before it). " +
                                "The scenario, not the delay, is at fault: lower the throttle.");
                return 1;
            }
            double splitDelay = t - tDecouplerSeq;

            if (!StepUntil(() => engineSeq!.Activated,
                    MaxStagingSeconds, "the engine sequence to activate"))
                return 1;
            double tEngineSeq = t;

            // The ignition check below is vehicle-wide, so it only measures this sequence's delay
            // while nothing else is burning. Say so instead of reporting a zero-second ignition.
            if (StagingHelpers.HasActiveEngineWithPropellant(vehicle))
            {
                HarnessLog.Line("[autostage-delays] FAIL: the vehicle was still under thrust when the " +
                                "engine sequence activated, so its ignition delay cannot be measured.");
                return 1;
            }
            if (!StepUntil(() => StagingHelpers.HasActiveEngineWithPropellant(vehicle),
                    MaxPhaseSeconds, "upper-stage ignition"))
                return 1;
            double igniteDelay = t - tEngineSeq;

            bool splitOk = Math.Abs(splitDelay - DecouplerDelayS) <= DelayTolS;
            bool igniteOk = Math.Abs(igniteDelay - EngineDelayS) <= DelayTolS;
            HarnessLog.Line($"[autostage-delays] decoupler fired {splitDelay:F1}s after sequence activation " +
                            $"(configured {DecouplerDelayS:F1}s, tol {DelayTolS:F1}s) => {(splitOk ? "ok" : "FAIL")}");
            HarnessLog.Line($"[autostage-delays] engine ignited {igniteDelay:F1}s after sequence activation " +
                            $"(configured {EngineDelayS:F1}s, tol {DelayTolS:F1}s) => {(igniteOk ? "ok" : "FAIL")}");
            ok = splitOk && igniteOk;
        }
        finally
        {
            AutoStageHost.CleanupAfterFlight(system, preexisting);
        }

        HarnessLog.Line($"[autostage-delays] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private static string DescribeLoad(in StructuralLoad load) =>
        $"g-load {load.PeakGLoad:F1}/{load.MaxGLoad:F1} ({load.GLoadFraction:P0} of the limit), " +
        $"dynamic pressure {load.DynamicPressureFraction:P0} of the limit";

    // A timeout here means either the sequence never carried the decouplers the search picked it
    // for, or the configured delay was not found for it. Both are indistinguishable from the
    // outside, so name them.
    private static void LogSequenceState(Vehicle vehicle, Sequence seq, string label)
    {
        HarnessLog.Line($"[autostage-delays] {label}: number={seq.Number}, activated={seq.Activated}, " +
                        $"parts={seq.Parts.Length}, " +
                        $"configuredDelay={Config.GetSequenceDecouplerDelay(vehicle, seq.Number):F1}s, " +
                        $"nextSequence={vehicle.Parts.SequenceList.GetNextSequenceNumber()}");
        ReadOnlySpan<Part> parts = seq.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            foreach (ISequenced module in parts[i].InSequence(seq.Number))
            {
                if (module is not Decoupler decoupler) continue;
                HarnessLog.Line($"[autostage-delays]   '{parts[i].Id}' {SequencedModules.Describe(module)} " +
                                $"active={decoupler.IsActive} enabled={decoupler.IsEnabled} " +
                                $"connected={decoupler.Connector.Connection != null} " +
                                $"delayKey={SequencedModules.DelayKey(module)}");
            }
        }
    }

    // The measurement needs a stage layout of: engines (the launch stage), a decoupler-only
    // sequence, then an engine sequence, so each delay is observable in isolation.
    private static bool TryFindDelaySequences(Vehicle vehicle,
        out Sequence? decouplerSeq, out Sequence? engineSeq)
    {
        decouplerSeq = null;
        engineSeq = null;
        bool sawLaunchEngines = false;
        foreach (Sequence seq in vehicle.Parts.SequenceList.Sequences)
        {
            if (seq.Activated || seq.Parts.IsEmpty)
                continue;
            // Per module: else a part whose motor belongs to a later row makes this one read
            // as an engine row.
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

    // Keyed the way the mod resolves a delay, so staging looks up the value set here.
    private static void ConfigureDelays(Vehicle vehicle, Sequence decouplerSeq, Sequence engineSeq)
    {
        Config.EngineDelays.Clear();
        Config.DecouplerDelays.Clear();
        foreach (Sequence seq in vehicle.Parts.SequenceList.Sequences)
        {
            if (seq != decouplerSeq && seq != engineSeq) continue;
            ReadOnlySpan<Part> parts = seq.Parts;
            for (int i = 0; i < parts.Length; i++)
            {
                foreach (ISequenced module in parts[i].InSequence(seq.Number))
                {
                    if (seq == decouplerSeq && module is Decoupler)
                        Config.DecouplerDelays[SequencedModules.DelayKey(module)] = DecouplerDelayS;
                    if (seq == engineSeq && module is EngineController)
                        Config.EngineDelays[SequencedModules.DelayKey(module)] = EngineDelayS;
                }
            }
        }
    }
}

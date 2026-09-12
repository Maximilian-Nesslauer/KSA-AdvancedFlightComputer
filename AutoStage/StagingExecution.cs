using System;
using System.Collections.Generic;
using AutoStage.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AutoStage;

/// <summary>
/// Stock's ActivateNextSequence, reimplemented so decouplers and engines can be
/// held back independently. Both delays run from the staging trigger. Sorting
/// per module is what lets one part fire its motor and its two mounts on three
/// different rows.
/// </summary>
static class StagingExecution
{
    public static PendingStaging? ActivateNextSequenceSplit(Vehicle vehicle)
    {
        // Up front so every return path invalidates.
        StagingHelpers.InvalidateSequenceCache();

        // Stock opens ActivateNextSequence with this.
        SequenceList.RequestScrollToBottom();

        SequenceList seqList = vehicle.Parts.SequenceList;
        ReadOnlySpan<Sequence> sequences = seqList.Sequences;

        // Find the next unactivated, non-empty sequence (same logic as stock)
        Sequence? target = null;
        for (int i = 0; i < sequences.Length; i++)
        {
            Sequence seq = sequences[i];
            if (seq.Activated) continue;

            if (seq.Parts.IsEmpty)
            {
                seq.Activated = true;
                continue;
            }

            target = seq;
            break;
        }

        if (target == null)
            return null;

        int seqNumber = target.Number;
        double engineDelay = Config.GetSequenceEngineDelay(vehicle, seqNumber);
        double decouplerDelay = Config.GetSequenceDecouplerDelay(vehicle, seqNumber);

        // Private setter, so reflection; ValidateAll gates the only patch that
        // reaches here. The public SetActiveSequence is not a substitute: it
        // rewrites Activated list-wide and runs ResetCaches, stock does neither.
        GameReflection.SequenceList_ActiveSequence!.SetValue(seqList, seqNumber);
        TimedAlert.Create($"Sequence {seqNumber} activated", Color.Yellow, 3.0);

        // Stock's own guard: ResetCaches early-returns while this is set, so a
        // re-entrant reset cannot rebuild _partsCache under the span below.
        GameReflection.SequenceList_updatingSequence!.SetValue(seqList, true);
        target.Activated = true;

        ReadOnlySpan<Part> parts = target.Parts;
        int partCount = parts.Length;
        List<ISequenced>? pendingEngines = null;
        List<ISequenced>? pendingDecouplers = null;
        int firedNow = 0;

        // The same walk stock performs through ActivateSubtreeInStage.
        for (int i = parts.Length - 1; i >= 0; i--)
        {
            foreach (ISequenced module in parts[i].InSequence(seqNumber))
            {
                if (!module.IsActive && module is EngineController && engineDelay > 0.0)
                {
                    pendingEngines ??= new List<ISequenced>();
                    pendingEngines.Add(module);
                }
                else if (!module.IsActive && module is Decoupler && decouplerDelay > 0.0)
                {
                    pendingDecouplers ??= new List<ISequenced>();
                    pendingDecouplers.Add(module);
                }
                else
                {
                    module.Activate(vehicle);
                    firedNow++;
                }
            }
        }

        GameReflection.SequenceList_updatingSequence!.SetValue(seqList, false);
        seqList.ResetCaches();
        // Prunes the empty rows the scan above marked activated, as stock does.
        seqList.RemoveSpentSequences();
        StagingHelpers.InvalidateSequenceCache();

        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug(
                $"[AutoStage] Activated sequence {seqNumber}: {partCount} part(s), "
                + $"{firedNow} module(s) fired now, "
                + $"{pendingEngines?.Count ?? 0} engine(s) and "
                + $"{pendingDecouplers?.Count ?? 0} decoupler(s) held.");

        // Nothing is drained or refreshed here. Activating only appends to
        // IActivateInputBuffer, so the tree is still unchanged; stock drains it
        // in Program.PrepareFrame and refreshes from Vehicle.Split at the frame
        // sync point. Doing either from this solver apply would mutate a vehicle
        // list PhysicsBubble is iterating, and UpdateAfterPartTreeModification
        // would take the non-blocking ConstraintSim.UnlockShapes and throw while
        // a solver step is in flight.

        bool anyPending = (pendingEngines != null && pendingEngines.Count > 0)
                       || (pendingDecouplers != null && pendingDecouplers.Count > 0);
        if (!anyPending)
            return null;

        if (DebugConfig.IgnitionDelay)
        {
            int eng = pendingEngines?.Count ?? 0;
            int dec = pendingDecouplers?.Count ?? 0;
            DefaultCategory.Log.Debug(
                $"[AutoStage] Split activation: seq {seqNumber}, " +
                $"{dec} decouplers pending (delay={decouplerDelay:F1}s), " +
                $"{eng} engines pending (delay={engineDelay:F1}s)");
        }

        return new PendingStaging(vehicle,
            pendingDecouplers, decouplerDelay,
            pendingEngines, engineDelay,
            Universe.GetElapsedSeconds());
    }

    /// <summary>An earlier decouple may have carried a module's part away.</summary>
    public static void ActivatePendingModules(Vehicle vehicle, List<ISequenced> modules, string label)
    {
        int activated = 0;
        foreach (ISequenced module in modules)
        {
            Part part = module.Parent.FullPart;
            if (part.Tree != vehicle.Parts)
            {
                DefaultCategory.Log.Warning(
                    $"[AutoStage] {label} module {SequencedModules.Describe(module)} on "
                    + $"'{part.DisplayName}' no longer belongs to vehicle, skipping.");
                continue;
            }
            module.Activate(vehicle);
            activated++;
        }

        if (DebugConfig.IgnitionDelay)
            DefaultCategory.Log.Debug(
                $"[AutoStage] Fired {activated}/{modules.Count} {label} modules");

        vehicle.Parts.SequenceList.ResetCaches();
        StagingHelpers.InvalidateSequenceCache();
    }
}

/// <summary>
/// Deadlines in sim time, not a countdown fed by DeltaTime: the detector also
/// runs on frames the game spends paused, where the last non-zero DeltaTime
/// would keep draining a countdown.
/// </summary>
class PendingStaging
{
    public Vehicle Vehicle { get; }
    public List<ISequenced>? DecouplerModules { get; private set; }
    public List<ISequenced>? EngineModules { get; private set; }

    public double DecouplerDelay { get; }
    public double EngineDelay { get; }
    public double DecouplerDeadline { get; }
    public double EngineDeadline { get; }

    public bool DecouplersPending => DecouplerModules != null && DecouplerModules.Count > 0;
    public bool EnginesPending => EngineModules != null && EngineModules.Count > 0;
    public bool AnyPending => DecouplersPending || EnginesPending;

    public PendingStaging(Vehicle vehicle,
        List<ISequenced>? decouplerModules, double decouplerDelay,
        List<ISequenced>? engineModules, double engineDelay, double now)
    {
        Vehicle = vehicle;
        DecouplerModules = decouplerModules;
        EngineModules = engineModules;
        DecouplerDelay = decouplerDelay;
        EngineDelay = engineDelay;
        DecouplerDeadline = now + decouplerDelay;
        EngineDeadline = now + engineDelay;
    }

    public void ClearDecouplers() => DecouplerModules = null;
    public void ClearEngines() => EngineModules = null;
}

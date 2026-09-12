using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

// Stock's SequenceList.ActivateNextSequence, reimplemented so decouplers and engines can be held
// back independently. Both delays run from the staging trigger. Sorting per module lets one part
// fire its motor and its two mounts on three different rows.
internal static class StagingExecution
{
    public static PendingStaging? ActivateNextSequenceSplit(Vehicle vehicle)
    {
        StagingHelpers.InvalidateSequenceCache();
        SequenceList.RequestScrollToBottom();

        SequenceList seqList = vehicle.Parts.SequenceList;
        ReadOnlySpan<Sequence> sequences = seqList.Sequences;

        Sequence? target = null;
        for (int i = 0; i < sequences.Length; i++)
        {
            Sequence seq = sequences[i];
            if (seq.Activated)
                continue;
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
        double engineDelay = StagingConfig.GetSequenceEngineDelay(vehicle, seqNumber);
        double decouplerDelay = StagingConfig.GetSequenceDecouplerDelay(vehicle, seqNumber);

        // The public SetActiveSequence is no substitute: it rewrites Activated list-wide and resets caches.
        GameReflection.SequenceList_ActiveSequence!.SetValue(seqList, seqNumber);
        TimedAlert.Create($"Sequence {seqNumber} activated", Color.Yellow, 3.0);

        // Stock's own guard: ResetCaches early-returns while this is set, so a re-entrant reset
        // cannot rebuild the parts cache under the span below.
        GameReflection.SequenceList_updatingSequence!.SetValue(seqList, true);
        target.Activated = true;

        ReadOnlySpan<Part> parts = target.Parts;
        List<ISequenced>? pendingEngines = null;
        List<ISequenced>? pendingDecouplers = null;
        int firedNow = 0;

        // The same walk stock performs through Part.ActivateSubtreeInStage.
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
        seqList.RemoveSpentSequences();
        StagingHelpers.InvalidateSequenceCache();

        // Nothing is drained or refreshed here. Activation only appends to IActivateInputBuffer;
        // stock drains it in Program.PrepareFrame and refreshes from Vehicle.Split at the frame
        // sync point. Doing either from the solver apply would mutate a list PhysicsBubble is
        // iterating.

        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug(
                $"[AFC] AutoStage activated sequence {seqNumber}: {parts.Length} part(s), {firedNow} module(s) fired now, " +
                $"{pendingEngines?.Count ?? 0} engine(s) held {engineDelay:F1}s, {pendingDecouplers?.Count ?? 0} decoupler(s) held {decouplerDelay:F1}s.");

        bool anyPending = pendingEngines is { Count: > 0 } || pendingDecouplers is { Count: > 0 };
        if (!anyPending)
            return null;

        return new PendingStaging(pendingDecouplers, decouplerDelay, pendingEngines, engineDelay,
            Universe.GetElapsedSeconds());
    }

    // An earlier decouple may have carried a module's part away.
    public static void ActivatePendingModules(Vehicle vehicle, List<ISequenced> modules, string label)
    {
        foreach (ISequenced module in modules)
        {
            Part part = module.Parent.FullPart;
            if (part.Tree != vehicle.Parts)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] AutoStage: {label} {SequencedModules.Describe(module)} on '{part.DisplayName}' no longer belongs to the vehicle, skipping.");
                continue;
            }
            module.Activate(vehicle);
        }

        vehicle.Parts.SequenceList.ResetCaches();
        StagingHelpers.InvalidateSequenceCache();
    }
}

// Deadlines in sim time, not a countdown fed by DeltaTime: the detector also runs on frames the
// game spends paused, where the last non-zero DeltaTime would keep draining a countdown.
internal sealed class PendingStaging
{
    public List<ISequenced>? DecouplerModules { get; private set; }
    public List<ISequenced>? EngineModules { get; private set; }

    public double DecouplerDelay { get; }
    public double EngineDelay { get; }
    public double DecouplerDeadline { get; }
    public double EngineDeadline { get; }

    public bool DecouplersPending => DecouplerModules is { Count: > 0 };
    public bool EnginesPending => EngineModules is { Count: > 0 };
    public bool AnyPending => DecouplersPending || EnginesPending;

    public PendingStaging(
        List<ISequenced>? decouplerModules, double decouplerDelay,
        List<ISequenced>? engineModules, double engineDelay, double now)
    {
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

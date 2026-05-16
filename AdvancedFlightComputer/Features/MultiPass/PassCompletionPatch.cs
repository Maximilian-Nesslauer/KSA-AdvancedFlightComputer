using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Per-vehicle Postfix on <see cref="Vehicle.UpdateFromTaskResults"/>
/// that drives multi-pass plans forward by detecting burn completion
/// and scheduling the next pass.
///
/// Completion is detected as an Auto -> Manual mode transition where
/// <c>DeltaVToGoCci . DeltaVTargetCci &lt;= 0</c>: only a real
/// completion reverses the to-go vector, an "out of fuel" event flips
/// the mode without flipping the dot-product sign.
/// </summary>
[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.UpdateFromTaskResults),
    new[] { typeof(VehicleUpdateData), typeof(Vehicle), typeof(ReadOnlySpan<Vehicle>) },
    new[] { ArgumentType.Ref, ArgumentType.Normal, ArgumentType.Normal })]
internal static class PassCompletionPatch
{
    private const int MaxAwaitingMaterializationTicks = 4;
    private const int MaxConsecutiveScheduleFailures = 5;

    // Tolerance for matching fc.Burn (the active BurnTarget) to our
    // exec.CurrentBurn by ImpulsiveInstant. Avoids triggering pass
    // completion on someone else's burn finishing.
    private const double BurnIdentityToleranceSec = 0.5;

    // Keyed by vehicle id (string) not FlightComputer reference because
    // UpdateFromTaskResults can swap FlightComputer in-place; a ref-keyed
    // entry would lose its prevMode reading on the swap tick.
    private static readonly Dictionary<string, FlightComputerBurnMode> _lastBurnMode = new();

    public static void Reset() => _lastBurnMode.Clear();

    /// <summary>
    /// Drops the burn-mode tracking entry for one vehicle when an
    /// outside-postfix site (e.g. the Cancel button) removes the
    /// registry entry directly. The internal CancelExecution path
    /// does this inline.
    /// </summary>
    public static void OnRegistryRemovedExternally(string vehicleId)
        => _lastBurnMode.Remove(vehicleId);

    static void Postfix(Vehicle __instance)
    {
        if (!MultiPassRegistry.TryGet(__instance.Id, out var exec))
            return;

        // Per-tick per-tracked-vehicle. Skipped when no execution is
        // active for this vehicle (the TryGet early-out above), so
        // the measurement reflects only real multi-pass work.
#if DEBUG
        using var _perf = new PerfTracker.Scope("PassCompletionPatch.Postfix");
#endif

        FlightComputer fc = __instance.FlightComputer;

        // Mutations here are in-memory only; SaveLoadObserver flushes
        // the registry to disk on KSA save events.
        try
        {
            ReconcileResult reconcile = ReconcileAfterLoad(__instance.Id, exec, fc);
            if (reconcile != ReconcileResult.Proceed)
                return;

            UpdateBurnModeTracking(__instance.Id, fc, out var prevMode, out var hadPrev);

            if (UpdateMaterializationTracking(__instance.Id, exec, fc) == MaterializationResult.Cancelled)
                return;

            if (DetectCompletion(exec, fc, prevMode, hadPrev))
                CommitCompletion(__instance.Id, exec, fc);

            if (DetectExternalDelete(exec, fc))
            {
                CancelExecution(__instance.Id,
                    "pending burn was deleted externally; cancelling");
                return;
            }

            if (exec.CurrentBurn == null && exec.PassIndex >= exec.PassCountTotal)
            {
                CompleteExecution(__instance.Id, exec);
                return;
            }

            if (exec.CurrentBurn == null)
                TryScheduleNext(__instance, exec);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error(
                $"[AFC] MultiPass: vehicle={__instance.Id} postfix threw, cancelling execution: {ex}");
            CancelExecution(__instance.Id, reason: null);
        }
    }

    #region Phases

    private enum ReconcileResult { Proceed, SkipTick, Cancelled }
    private enum MaterializationResult { Proceed, Cancelled }

    /// <summary>
    /// After a save load, CurrentBurn is null but CurrentBurnTimeSec /
    /// CurrentBurnDvMagnitudeMs identify the burn we expect in the
    /// restored BurnPlan. Reattach if found; SkipTick otherwise so a
    /// transient deserialize state (the burn appears a tick later) is
    /// tolerated.
    /// </summary>
    private static ReconcileResult ReconcileAfterLoad(
        string vehicleId, MultiPassExecution exec, FlightComputer fc)
    {
        if (exec.CurrentBurn != null || !exec.CurrentBurnTimeSec.HasValue)
            return ReconcileResult.Proceed;

        Burn? matched = exec.TryResolveCurrentBurn(fc.BurnPlan);
        if (matched != null)
        {
            exec.ReattachAfterLoad(matched);
            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPass.Reconcile: vehicle='{vehicleId}' reattached to " +
                    $"burn t={matched.Time.Seconds():F1}s dv={matched.DeltaVVlf.Length():F2}m/s");
            return ReconcileResult.Proceed;
        }

        if (DebugConfig.MultiPass)
        {
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass.Reconcile: vehicle='{vehicleId}' could not match " +
                $"persisted burn (t={exec.CurrentBurnTimeSec:F1}s, " +
                $"dv={exec.CurrentBurnDvMagnitudeMs:F2}m/s); BurnPlan dump follows.");
            MultiPassDebug.LogBurnPlan($"Reconcile no-match vehicle='{vehicleId}'", fc.BurnPlan);
        }
        return ReconcileResult.SkipTick;
    }

    private static void UpdateBurnModeTracking(
        string vehicleId, FlightComputer fc,
        out FlightComputerBurnMode prevMode, out bool hadPrev)
    {
        hadPrev = _lastBurnMode.TryGetValue(vehicleId, out prevMode);
        _lastBurnMode[vehicleId] = fc.BurnMode;
    }

    /// <summary>
    /// Tracks how long we have been waiting for the queued burn to
    /// appear in the BurnPlan. The buffered Add applies on the next
    /// frame boundary, so a missing burn is normal for a tick or two;
    /// past <see cref="MaxAwaitingMaterializationTicks"/> we assume
    /// the queue dropped the add and cancel rather than letting the
    /// external-delete check stay suppressed forever.
    /// </summary>
    private static MaterializationResult UpdateMaterializationTracking(
        string vehicleId, MultiPassExecution exec, FlightComputer fc)
    {
        if (!exec.AwaitingMaterialization || exec.CurrentBurn == null)
            return MaterializationResult.Proceed;

        if (fc.BurnPlan.TryGetBurn(exec.CurrentBurn))
        {
            exec.AwaitingMaterialization = false;
            exec.AwaitingMaterializationTicks = 0;
            return MaterializationResult.Proceed;
        }

        exec.AwaitingMaterializationTicks++;
        if (exec.AwaitingMaterializationTicks > MaxAwaitingMaterializationTicks)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPass: vehicle={vehicleId} queued burn never " +
                $"appeared in BurnPlan after {exec.AwaitingMaterializationTicks} " +
                "ticks; cancelling execution (queue likely dropped the add).");
            CancelExecution(vehicleId, reason: null);
            return MaterializationResult.Cancelled;
        }
        return MaterializationResult.Proceed;
    }

    private static bool DetectCompletion(
        MultiPassExecution exec, FlightComputer fc,
        FlightComputerBurnMode prevMode, bool hadPrev)
    {
        if (!hadPrev
            || prevMode != FlightComputerBurnMode.Auto
            || fc.BurnMode != FlightComputerBurnMode.Manual
            || fc.Burn == null
            || exec.CurrentBurn == null)
            return false;

        // Identity check: a non-multi-pass Burn ahead of ours in the
        // plan would also trigger Auto->Manual + reversed dot. Only
        // count the transition when fc.Burn is OUR burn.
        bool isOurBurn = Math.Abs(fc.Burn.ImpulsiveInstant.Seconds()
                                  - exec.CurrentBurn.Time.Seconds())
                         < BurnIdentityToleranceSec;
        float dot = float3.Dot(fc.Burn.DeltaVToGoCci, fc.Burn.DeltaVTargetCci);

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(string.Format(
                System.Globalization.CultureInfo.InvariantCulture,
                "[AFC] MultiPass.DetectCompletion: vehicle='{0}' Auto->Manual, " +
                "dot={1:F4} ourBurn={2} -> {3}",
                exec.VehicleId, dot, isOurBurn ? "yes" : "no",
                (isOurBurn && dot <= 0f) ? "completed" : "skipped"));

        return isOurBurn && dot <= 0f;
    }

    private static void CommitCompletion(string vehicleId, MultiPassExecution exec, FlightComputer fc)
    {
        if (DebugConfig.MultiPass)
            MultiPassDebug.LogBurnPlan(
                $"CommitCompletion vehicle='{vehicleId}' pre-delete", fc.BurnPlan);

        // Queue the delete via BurnUpdateBuffer rather than calling
        // fc.RemoveBurn directly: this postfix runs before the buffer's
        // ApplyAll, and direct mutation here would race any user-queued
        // burn ops from the same frame.
        if (exec.CurrentBurn != null && fc.BurnPlan.TryGetBurn(exec.CurrentBurn))
        {
            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPass.CommitCompletion: vehicle='{vehicleId}' " +
                    $"queueing delete of burn t={exec.CurrentBurn.Time.Seconds():F1}s " +
                    $"dv={exec.CurrentBurn.DeltaVVlf.Length():F2}m/s");
            InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
            {
                Burn = exec.CurrentBurn,
                FlightComputer = fc,
                DeleteBurn = true,
            });
        }
        else if (DebugConfig.MultiPass)
        {
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass.CommitCompletion: vehicle='{vehicleId}' no live burn " +
                $"to delete (CurrentBurn={(exec.CurrentBurn != null ? "set" : "null")}, " +
                $"in plan={(exec.CurrentBurn != null && fc.BurnPlan.TryGetBurn(exec.CurrentBurn) ? "yes" : "no")})");
        }

        exec.ClearCurrentBurn();
        exec.ConsecutiveScheduleFailures = 0;
        exec.PassIndex++;

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass: vehicle={vehicleId} pass {exec.PassIndex}/{exec.PassCountTotal} completed");
    }

    private static bool DetectExternalDelete(MultiPassExecution exec, FlightComputer fc)
    {
        bool fired = exec.CurrentBurn != null
            && !exec.AwaitingMaterialization
            && !fc.BurnPlan.TryGetBurn(exec.CurrentBurn);

        if (fired && DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass.DetectExternalDelete: vehicle='{exec.VehicleId}' " +
                $"burn t={exec.CurrentBurn!.Time.Seconds():F1}s no longer in BurnPlan; " +
                "treating as user delete");
        return fired;
    }

    private static void CompleteExecution(string vehicleId, MultiPassExecution exec)
    {
        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass: vehicle={vehicleId} multi-pass complete ({exec.PassCountTotal} passes)");
        CancelExecution(vehicleId, reason: null);
    }

    private static void CancelExecution(string vehicleId, string? reason)
    {
        if (reason != null && DebugConfig.MultiPass)
            DefaultCategory.Log.Debug($"[AFC] MultiPass: vehicle={vehicleId} {reason}");
        MultiPassRegistry.Remove(vehicleId);
        _lastBurnMode.Remove(vehicleId);
    }

    /// <summary>
    /// Plans and queues the next pass. After
    /// <see cref="MaxConsecutiveScheduleFailures"/> consecutive
    /// failures, cancels the execution so persistent failures surface
    /// as a warning instead of an indefinite silent stall.
    /// </summary>
    private static bool TryScheduleNext(Vehicle vehicle, MultiPassExecution exec)
    {
        string? failure = MultiPassCommitter.TryCommitNext(vehicle, exec);
        if (failure != null)
        {
            exec.ConsecutiveScheduleFailures++;
            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPass: vehicle={vehicle.Id} schedule attempt " +
                    $"{exec.ConsecutiveScheduleFailures}/{MaxConsecutiveScheduleFailures} " +
                    $"failed: {failure}");

            if (exec.ConsecutiveScheduleFailures >= MaxConsecutiveScheduleFailures)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] MultiPass: vehicle={vehicle.Id} cancelling after " +
                    $"{exec.ConsecutiveScheduleFailures} consecutive schedule " +
                    $"failures (last reason: {failure}).");
                CancelExecution(vehicle.Id, reason: null);
            }
            return false;
        }

        exec.ConsecutiveScheduleFailures = 0;
        if (DebugConfig.MultiPass && exec.CurrentBurn != null)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass: vehicle={vehicle.Id} scheduled pass {exec.PassIndex + 1}/{exec.PassCountTotal} dV={exec.CurrentBurn.DeltaVVlf.Length():F1} m/s at t={exec.CurrentBurn.Time.Seconds():F0}s");
        return true;
    }

    #endregion
}

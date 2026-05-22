using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
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

        // Hohmann piggybacks on stock's Transfer Planning window: once a
        // pass ignites, stock's DrawPlanWindow auto-clears
        // _transferCalculated in the "_transferBurn.Time < now" branch,
        // which wipes the entire selected-entry block - hiding our
        // inline UI, the 3D overlay gate, and stock's own
        // DrawSelectedTransferUi call. We re-set it every physics tick
        // so by the time the next OnPreRender / DrawPlanWindow runs it
        // is true again. Scoped to "plan window is actually on this
        // vehicle's Hohmann transfer" so a user who dropdowns to a
        // different source/type doesn't get their stock UI pinned by
        // some other vehicle's still-running exec.
        if (exec.Intent is HohmannTransferIntent
            && IsPlanWindowOnVehicleHohmann(__instance))
            KeepStockTransferCalculatedInSync();

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

            // Re-sync stock _transferBurn now that the new pass burn is
            // actually in the plan; the sync in TryScheduleNext fires
            // before ApplyInputEvents, so stock's line-172 auto-clear
            // (which checks plan.TryGetBurn) may have wiped _transferBurn
            // in between. This second sync closes that window.
            if (exec.Intent is HohmannTransferIntent)
                KeepStockTransferBurnInSync(exec.CurrentBurn);

            // Carry the user's Auto over from the prior pass: stock's
            // LoadBurn (just ran via BurnUpdateBuffer.ApplyAll) reset
            // BurnMode to Manual; flip it back so the queued pass fires
            // without a manual re-toggle.
            if (exec.ReengageAutoOnNextBurn)
            {
                fc.BurnMode = FlightComputerBurnMode.Auto;
                exec.ReengageAutoOnNextBurn = false;
                if (DebugConfig.MultiPass)
                    DefaultCategory.Log.Debug(
                        $"[AFC] MultiPass: vehicle={vehicleId} re-engaged Auto " +
                        $"for pass {exec.PassIndex + 1}/{exec.PassCountTotal}");
            }

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

        // DetectCompletion required prevMode == Auto, so the user had
        // Auto on when this pass finished. Carry the intent forward to
        // the next queued burn; final-pass completions skip the flag
        // (Auto stays off naturally, no more passes to fire).
        if (exec.PassIndex < exec.PassCountTotal)
            exec.ReengageAutoOnNextBurn = true;

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
        Patch_DrawPlanWindow.OnMultiPassCompleted();
    }

    private static void CancelExecution(string vehicleId, string? reason)
    {
        if (reason != null && DebugConfig.MultiPass)
            DefaultCategory.Log.Debug($"[AFC] MultiPass: vehicle={vehicleId} {reason}");
        MultiPassRegistry.Remove(vehicleId);
        _lastBurnMode.Remove(vehicleId);
        // Clear the Hohmann inline-UI cache so a stale preview chain
        // doesn't outlive the registry entry; harmless no-op when the
        // ended exec was a different intent kind.
        HohmannMultiPassUI.OnExecutionEnded(vehicleId);
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
            // Distinguish "intent already satisfied" (early convergence
            // due to the splitter over-allocating dV per pass) from a
            // genuine planning failure. The former is success-as-
            // completion, not a 5-strikes-and-cancel scenario.
            if (exec.Intent.IsSatisfied(vehicle))
            {
                if (DebugConfig.MultiPass)
                    DefaultCategory.Log.Debug(
                        $"[AFC] MultiPass: vehicle={vehicle.Id} intent already " +
                        $"satisfied at pass {exec.PassIndex + 1}/{exec.PassCountTotal} " +
                        $"(planner: {failure}); completing execution early.");
                CompleteExecution(vehicle.Id, exec);
                return true;
            }

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

        // For Hohmann executions, keep stock TransferPlanner._transferBurn
        // pointing to the current pass burn so stock's Create-button guard
        // (if (_transferBurn == null && ...)) blocks re-clicks while
        // multi-pass is active. Without this, our interceptor would fire
        // on a re-click and return the live burn, which would then be
        // double-added to BurnPlan and lead to a use-after-Dispose when
        // the duplicate is later removed.
        if (exec.Intent is HohmannTransferIntent && exec.CurrentBurn != null)
            KeepStockTransferBurnInSync(exec.CurrentBurn);

        return true;
    }

    /// <summary>Reflection setter so the stock TransferPlanner._transferBurn
    /// field tracks our current pass during a Hohmann multi-pass.
    /// Stock's line-172 auto-clear (when the burn leaves the plan) will
    /// reset it next frame if our burn happens to be missing, which is
    /// the desired behaviour at multi-pass completion.</summary>
    private static void KeepStockTransferBurnInSync(Burn currentBurn)
    {
        try
        {
            GameReflection.TransferPlanner_transferBurn?.SetValue(null, currentBurn);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] PassCompletionPatch: failed to sync _transferBurn: {ex.Message}");
        }
    }

    /// <summary>True when the stock Transfer Planning window is open
    /// AND showing Hohmann AND the source vehicle matches
    /// <paramref name="execVehicle"/>. The <see cref="KeepStockTransferCalculatedInSync"/>
    /// sync targets a process-global static; without this scope check
    /// a vehicle's still-running exec would pin the flag for unrelated
    /// vehicles / transfer types the user is meanwhile inspecting,
    /// breaking stock's source-change / type-change resets.</summary>
    private static bool IsPlanWindowOnVehicleHohmann(Vehicle execVehicle)
    {
        try
        {
            if (!(bool)(GameReflection.TransferPlanner_showPlanWindow?.GetValue(null) ?? false))
                return false;
            var transferType = (TransferType?)GameReflection.TransferPlanner_transferType?
                .GetValue(null);
            if (transferType == null || transferType.Value.GetKey() != ManeuverTools.ManeuverTools.KeyStockHohmann)
                return false;
            var sourceBody = (TransferObject?)GameReflection.TransferPlanner_sourceBody?
                .GetValue(null);
            return sourceBody?.Body is Vehicle v && v.Id == execVehicle.Id;
        }
        catch
        {
            return false;
        }
    }

    /// <summary>Suppresses stock's "burn fired, drop UI" auto-clear of
    /// <c>_transferCalculated</c>. Without this, every pass ignition
    /// flips the flag false and hides our inline section + 3D overlay
    /// until the user manually clicks Re-Calculate. Caller must verify
    /// the plan window scope via <see cref="IsPlanWindowOnVehicleHohmann"/>;
    /// the field is process-global and pinning it for the wrong scope
    /// stomps stock's normal source/type-change resets.</summary>
    private static void KeepStockTransferCalculatedInSync()
    {
        try
        {
            GameReflection.TransferPlanner_transferCalculated?.SetValue(null, true);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] PassCompletionPatch: failed to sync _transferCalculated: {ex.Message}");
        }
    }

    #endregion
}

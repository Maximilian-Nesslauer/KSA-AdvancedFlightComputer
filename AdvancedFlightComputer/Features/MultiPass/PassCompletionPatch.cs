using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.PlanWindow;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.RcsTranslation;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// Run on the main thread before input events are drained. Stock completion changes Auto to Manual when the target dot product reaches zero. Fuel exhaustion can leave a positive dot product.
internal static class PassCompletionPatch
{
    private const int MaxAwaitingMaterializationTicks = 4;
    private const int MaxConsecutiveScheduleFailures = 5;

    // Match the active burn time so completion of another burn cannot advance this execution.
    private const double BurnIdentityToleranceSec = 0.5;

    // Key observations by vehicle ID because FlightComputer.CopyFrom can overwrite a computer and loading can replace it.
    private static readonly Dictionary<string, FlightComputerBurnMode> _lastBurnMode = new();

    public static void Reset() => _lastBurnMode.Clear();

    public static void OnRegistryRemovedExternally(string vehicleId)
        => _lastBurnMode.Remove(vehicleId);

    internal static void OnRcsBurnCompleted(Vehicle vehicle, Burn completedBurn)
    {
        if (!MultiPassRegistry.TryGet(vehicle.Id, out MultiPassExecution? exec)
            || !ReferenceEquals(exec.CurrentBurn, completedBurn))
            return;

        try
        {
            if (MultiPassDebug.Enabled)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPass: vehicle='{vehicle.Id}' received RCS completion " +
                    $"for pass {exec.PassIndex + 1}/{exec.PassCountTotal}.");
            CommitCompletion(vehicle.Id, exec, vehicle.FlightComputer, removeBurnImmediately: true);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error(
                $"[AFC] MultiPass: vehicle={vehicle.Id} RCS completion threw, cancelling execution: {ex}");
            CancelExecution(vehicle.Id, reason: null);
        }
    }

    internal static void TickVehicle(Vehicle vehicle)
    {
        if (!MultiPassRegistry.TryGet(vehicle.Id, out var exec))
            return;

        FlightComputer fc = vehicle.FlightComputer;

        // Keep active execution controls available after ignition clears the stock calculated flag.
        if (exec.Intent is HohmannTransferIntent
            && IsPlanWindowOnVehicleHohmann(vehicle))
            KeepStockTransferCalculatedInSync();

        // Execution changes remain in memory until the game is saved.
        try
        {
            ReconcileResult reconcile = ReconcileAfterLoad(vehicle.Id, exec, fc);
            if (reconcile != ReconcileResult.Proceed)
                return;

            UpdateBurnModeTracking(vehicle.Id, fc, out var prevMode, out var hadPrev);
            ObserveAutoEngagement(vehicle, exec, fc, prevMode, hadPrev);

            if (UpdateMaterializationTracking(vehicle, exec, fc) == MaterializationResult.Cancelled)
                return;

            AdvanceExecution(vehicle, exec, fc, prevMode, hadPrev);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error(
                $"[AFC] MultiPass: vehicle={vehicle.Id} tick threw, cancelling execution: {ex}");
            CancelExecution(vehicle.Id, reason: null);
        }
    }

    private static void ObserveAutoEngagement(
        Vehicle vehicle, MultiPassExecution exec, FlightComputer fc,
        FlightComputerBurnMode prevMode, bool hadPrev)
    {
        bool rcsFlyingPass = RcsIsFlyingPass(vehicle, exec);

        // A new attempt on this pass permits another stall hint if the user retries it.
        if (fc.BurnMode == FlightComputerBurnMode.Auto || rcsFlyingPass)
            exec.StallHintShown = false;

        // Match the target before recording Auto for this pass.
        if (fc.BurnMode == FlightComputerBurnMode.Auto
            && fc.Burn != null
            && exec.CurrentBurn != null
            && Math.Abs((fc.Burn.ImpulsiveInstant - exec.CurrentBurn.Time).Seconds())
               < BurnIdentityToleranceSec
            && !exec.BurnAutoEngagedThisPass)
        {
            exec.BurnAutoEngagedThisPass = true;
            exec.PassArmed = true;
            if (MultiPassDebug.Enabled)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPass: vehicle='{vehicle.Id}' pass " +
                    $"{exec.PassIndex + 1}/{exec.PassCountTotal} Auto engaged " +
                    $"(burn t={exec.CurrentBurn.Time.Seconds():F1}s).");
        }

        // An RCS pass stays in Manual, so its active execution shows that it started.
        if (rcsFlyingPass && !exec.PassArmed)
        {
            exec.PassArmed = true;
            if (MultiPassDebug.Enabled)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPass: vehicle='{vehicle.Id}' pass " +
                    $"{exec.PassIndex + 1}/{exec.PassCountTotal} armed for RCS " +
                    $"(burn t={exec.CurrentBurn!.Time.Seconds():F1}s).");
        }

        if (MultiPassDebug.Enabled && hadPrev && prevMode != fc.BurnMode)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass: vehicle='{vehicle.Id}' BurnMode " +
                $"{prevMode} -> {fc.BurnMode} (passIndex={exec.PassIndex}/" +
                $"{exec.PassCountTotal}, CurrentBurn={(exec.CurrentBurn != null ? "set" : "null")}, " +
                $"fc.Burn={(fc.Burn != null ? "set" : "null")}).");
    }

    private static void AdvanceExecution(
        Vehicle vehicle, MultiPassExecution exec, FlightComputer fc,
        FlightComputerBurnMode prevMode, bool hadPrev)
    {
        // Check completion before external deletion because another completion subscriber can remove the burn.
        bool didCommit = false;
        if (DetectCompletion(exec, fc, prevMode, hadPrev)
            || DetectImplicitCompletion(vehicle.Id, exec, fc))
        {
            CommitCompletion(vehicle.Id, exec, fc);
            didCommit = true;
        }

        if (!didCommit && DetectExternalDelete(exec, fc))
        {
            CancelExecution(vehicle.Id,
                "pending burn was deleted externally; cancelling");
            return;
        }

        // Retain a stopped burn with remaining delta v. Show one hint and let the user resume or cancel.
        if (!didCommit && exec.CurrentBurn != null)
            MaybeAlertStalledPass(vehicle, exec, fc);

        if (exec.CurrentBurn == null && exec.PassIndex >= exec.PassCountTotal)
        {
            CompleteExecution(vehicle.Id, exec);
            return;
        }

        if (exec.CurrentBurn == null)
            TryScheduleNext(vehicle, exec);
    }

    #region Phases

    private enum ReconcileResult { Proceed, SkipTick }
    private enum MaterializationResult { Proceed, Cancelled }

    // Persisted time and delta v identify the pass after loading removes its live reference.
    private static ReconcileResult ReconcileAfterLoad(
        string vehicleId, MultiPassExecution exec, FlightComputer fc)
    {
        if (exec.CurrentBurn != null || !exec.CurrentBurnTimeSec.HasValue)
            return ReconcileResult.Proceed;

        Burn? matched = exec.TryResolveCurrentBurn(fc.BurnPlan);
        if (matched != null)
        {
            exec.ReattachAfterLoad(matched);
            if (MultiPassDebug.Enabled)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPass.Reconcile: vehicle='{vehicleId}' reattached to " +
                    $"burn t={matched.Time.Seconds():F1}s dv={matched.DeltaVVlf.Length():F2}m/s");
            return ReconcileResult.Proceed;
        }

        if (MultiPassDebug.Enabled)
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

    // A buffered addition can be absent for several ticks. Limit this grace period before treating the absence as deletion.
    private static MaterializationResult UpdateMaterializationTracking(
        Vehicle vehicle, MultiPassExecution exec, FlightComputer fc)
    {
        string vehicleId = vehicle.Id;
        if (!exec.AwaitingMaterialization || exec.CurrentBurn == null)
            return MaterializationResult.Proceed;

        if (fc.BurnPlan.TryGetBurn(exec.CurrentBurn))
        {
            exec.AwaitingMaterialization = false;
            exec.AwaitingMaterializationTicks = 0;

            // Restore the stock transfer burn after it appears in the plan. DrawPlanWindow may have cleared the earlier reference before ApplyInputEvents added it.
            if (exec.Intent is HohmannTransferIntent)
                KeepStockTransferBurnInSync(exec.CurrentBurn);

            // Carry Auto into the next pass after FlightComputer.LoadBurn resets the mode to Manual.
            // RCS can accept this request while the burn mode stays in Manual.
            if (exec.ReengageAutoOnNextBurn)
            {
                vehicle.SetEnum(FlightComputerBurnMode.Auto);
                exec.ReengageAutoOnNextBurn = false;
                // Record the pass only if Auto or RCS accepted the request.
                if (fc.BurnMode == FlightComputerBurnMode.Auto || RcsIsFlyingPass(vehicle, exec))
                    exec.PassArmed = true;
                if (MultiPassDebug.Enabled)
                    DefaultCategory.Log.Debug(
                        $"[AFC] MultiPass: vehicle={vehicleId} re-engaged execution " +
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

        // Require a matching burn time because an unrelated burn can also change from Auto to Manual with a reversed dot product.
        bool isOurBurn = Math.Abs((fc.Burn.ImpulsiveInstant - exec.CurrentBurn.Time).Seconds())
                         < BurnIdentityToleranceSec;
        float dot = float3.Dot(fc.Burn.DeltaVToGoCci, fc.Burn.DeltaVTargetCci);

        if (MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(string.Format(
                System.Globalization.CultureInfo.InvariantCulture,
                "[AFC] MultiPass.DetectCompletion: vehicle='{0}' Auto->Manual, " +
                "dot={1:F4} ourBurn={2} -> {3}",
                exec.VehicleId, dot, isOurBurn ? "yes" : "no",
                (isOurBurn && dot <= 0f) ? "completed" : "skipped"));

        return isOurBurn && dot <= 0f;
    }

    private static void CommitCompletion(
        string vehicleId, MultiPassExecution exec, FlightComputer fc,
        bool removeBurnImmediately = false)
    {
        if (exec.CurrentBurn != null && fc.BurnPlan.TryGetBurn(exec.CurrentBurn))
        {
            if (MultiPassDebug.Enabled)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPass.CommitCompletion: vehicle='{vehicleId}' " +
                    $"{(removeBurnImmediately ? "removing" : "queueing delete of")} " +
                    $"burn t={exec.CurrentBurn.Time.Seconds():F1}s " +
                    $"dv={exec.CurrentBurn.DeltaVVlf.Length():F2}m/s");

            if (removeBurnImmediately)
                fc.RemoveBurn(exec.CurrentBurn);
            else
            {
                // Buffer stock completion removal so it stays ordered with user input. RCS completion removes immediately so a later event subscriber cannot also remove a burn that AFC queued for deletion.
                InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
                {
                    Burn = exec.CurrentBurn,
                    FlightComputer = fc,
                    DeleteBurn = true,
                });
            }
        }
        else if (MultiPassDebug.Enabled)
        {
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass.CommitCompletion: vehicle='{vehicleId}' no live burn " +
                $"to delete (CurrentBurn={(exec.CurrentBurn != null ? "set" : "null")}, " +
                $"in plan={(exec.CurrentBurn != null && fc.BurnPlan.TryGetBurn(exec.CurrentBurn) ? "yes" : "no")})");
        }

        exec.ClearCurrentBurn();
        exec.ConsecutiveScheduleFailures = 0;
        exec.PassIndex++;

        // Completion carries automatic execution into the next pass after the new burn loads.
        if (exec.PassIndex < exec.PassCountTotal)
            exec.ReengageAutoOnNextBurn = true;

        // Stock draws the final queued orbit. Hide the coincident selected transfer overlay.
        if (exec.Intent is HohmannTransferIntent
            && exec.PassIndex == exec.PassCountTotal - 1)
            DisableStockHohmannOrbitPreview();

        if (MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass: vehicle={vehicleId} pass {exec.PassIndex}/{exec.PassCountTotal} completed");
    }

    // A burn removed after Auto was observed at its scheduled time may have been removed by another completion subscriber.
    private static bool DetectImplicitCompletion(
        string vehicleId, MultiPassExecution exec, FlightComputer fc)
    {
        if (exec.CurrentBurn == null) return false;
        if (exec.AwaitingMaterialization) return false;
        if (fc.BurnPlan.TryGetBurn(exec.CurrentBurn)) return false;
        // Without Auto evidence, a missing burn is a deletion rather than a completed pass.
        if (!exec.BurnAutoEngagedThisPass) return false;
        // Allow one second for differences in burn time bookkeeping.
        UniverseTime simNow = Universe.GetElapsedTime();
        if (simNow < exec.CurrentBurn.Time - 1.0)
            return false;

        if (MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass.DetectImplicitCompletion: vehicle='{vehicleId}' " +
                $"burn t={exec.CurrentBurn.Time.Seconds():F1}s removed from BurnPlan " +
                $"after firing (Auto engaged this pass, sim t={simNow.Seconds():F1}s); " +
                "treating as natural completion, another completion subscriber removed it.");
        return true;
    }

    private static bool DetectExternalDelete(MultiPassExecution exec, FlightComputer fc)
    {
        bool fired = exec.CurrentBurn != null
            && !exec.AwaitingMaterialization
            && !fc.BurnPlan.TryGetBurn(exec.CurrentBurn);

        if (fired && MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass.DetectExternalDelete: vehicle='{exec.VehicleId}' " +
                $"burn t={exec.CurrentBurn!.Time.Seconds():F1}s removed at sim t=" +
                $"{Universe.GetElapsedTime().Seconds():F1}s " +
                $"(autoEngaged={exec.BurnAutoEngagedThisPass}); treating as user delete.");
        return fired;
    }

    // Manual can mean exhausted fuel or a user pause. Preserve the execution and show only one hint.
    private static void MaybeAlertStalledPass(
        Vehicle vehicle, MultiPassExecution exec, FlightComputer fc)
    {
        if (exec.StallHintShown) return;
        if (exec.AwaitingMaterialization) return;
        if (exec.CurrentBurn == null) return;
        // A pass that was never armed can still be waiting for execution.
        if (!exec.PassArmed) return;
        // The RCS executor flies with the burn mode at Manual, so a running execution is not a stall.
        if (RcsIsFlyingPass(vehicle, exec)) return;
        if (fc.BurnMode != FlightComputerBurnMode.Manual) return;
        if (!fc.BurnPlan.TryGetBurn(exec.CurrentBurn)) return;
        // Auto includes alignment. Wait until the planned impulsive time before reporting a stall.
        if (Universe.GetElapsedTime() < exec.CurrentBurn.Time) return;

        exec.StallHintShown = true;
        DefaultCategory.Log.Warning(
            $"[AFC] MultiPass: vehicle={vehicle.Id} pass {exec.PassIndex + 1}/" +
            $"{exec.PassCountTotal} stopped with dV remaining. It ran out of " +
            "propellant or execution was disengaged. The execution stays paused " +
            "until it is engaged again or the plan is cancelled.");
        TimedAlert.Create(
            $"Multi-pass pass {exec.PassIndex + 1}/{exec.PassCountTotal} stopped with " +
            "dV remaining. Re-engage Auto to continue, or cancel the remaining passes.",
            Color.Yellow, 6.0);
    }

    // RCS stays in Manual. Match its active burn by time because it can fly another burn on the vehicle.
    private static bool RcsIsFlyingPass(Vehicle vehicle, MultiPassExecution exec)
    {
        if (exec.CurrentBurn == null) return false;
        if (!SharedVehicleHooks.RcsEnabled) return false;
        if (!RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? rcs)) return false;
        if (rcs.ActiveBurnTimeSec is not double activeTimeSec) return false;
        return Math.Abs(activeTimeSec - exec.CurrentBurn.Time.Seconds()) < BurnIdentityToleranceSec;
    }

    private static void CompleteExecution(string vehicleId, MultiPassExecution exec)
    {
        if (MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass: vehicle={vehicleId} multi-pass complete ({exec.PassCountTotal} passes)");
        CancelExecution(vehicleId, reason: null);
        Patch_DrawPlanWindow.OnMultiPassCompleted();
    }

    private static void CancelExecution(string vehicleId, string? reason)
    {
        if (reason != null && MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug($"[AFC] MultiPass: vehicle={vehicleId} {reason}");
        MultiPassRegistry.Remove(vehicleId);
        _lastBurnMode.Remove(vehicleId);
        // Clear the Hohmann preview when execution ends so it cannot outlive its registry entry.
        HohmannMultiPassUI.OnExecutionEnded(vehicleId);
    }

    private static bool TryScheduleNext(Vehicle vehicle, MultiPassExecution exec)
    {
        string? failure = MultiPassCommitter.TryCommitNext(vehicle, exec);
        if (failure != null)
        {
            // An intent already met is successful completion. Do not count it as a planning failure and retry.
            if (exec.Intent.IsSatisfied(vehicle))
            {
                if (MultiPassDebug.Enabled)
                    DefaultCategory.Log.Debug(
                        $"[AFC] MultiPass: vehicle={vehicle.Id} intent already " +
                        $"satisfied at pass {exec.PassIndex + 1}/{exec.PassCountTotal} " +
                        $"(planner: {failure}); completing execution early.");
                CompleteExecution(vehicle.Id, exec);
                return true;
            }

            exec.ConsecutiveScheduleFailures++;
            if (MultiPassDebug.Enabled)
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
        if (MultiPassDebug.Enabled && exec.CurrentBurn != null)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass: vehicle={vehicle.Id} scheduled pass {exec.PassIndex + 1}/{exec.PassCountTotal} dV={exec.CurrentBurn.DeltaVVlf.Length():F1} m/s at t={exec.CurrentBurn.Time.Seconds():F0}s");

        // Keep the live burn visible to stock creation logic so it cannot create a duplicate.
        if (exec.Intent is HohmannTransferIntent && exec.CurrentBurn != null)
            KeepStockTransferBurnInSync(exec.CurrentBurn);

        return true;
    }

    private static void KeepStockTransferBurnInSync(Burn currentBurn)
        => StockPlanner.TransferBurn = currentBurn;

    // Stock planner fields are global. Pin them only for the matching vehicle and transfer type.
    private static bool IsPlanWindowOnVehicleHohmann(Vehicle execVehicle)
    {
        try
        {
            if (!StockPlanner.ShowPlanWindow) return false;
            if (StockPlanner.TransferTypeKey != ManeuverTools.ManeuverTools.KeyStockHohmann)
                return false;
            return StockPlanner.SourceVehicle is Vehicle v && v.Id == execVehicle.Id;
        }
        catch (Exception ex)
        {
            // CelestialSystem.GetIndex can throw for an invalid source index. Do not let that escape
            // from the ApplyVehicleSolvers postfix.
            LogHelper.WarnOnce("multipass-plan-window-source:" + ex.GetType().Name,
                $"[AFC] PassCompletionPatch: could not read the stock plan-window source for " +
                $"vehicle='{execVehicle.Id}': {ex}");
            return false;
        }
    }

    // DrawPlanWindow indexes the selected entry when the calculated flag is set. Restore that flag only while the underlying transfer array remains available.
    private static void KeepStockTransferCalculatedInSync()
    {
        if (!StockPlanner.SelectedTransferBlockIsSafe) return;
        StockPlanner.TransferCalculated = true;
    }

    private static void DisableStockHohmannOrbitPreview()
        => StockPlanner.DisplaySelectedTransfer = false;

    #endregion
}

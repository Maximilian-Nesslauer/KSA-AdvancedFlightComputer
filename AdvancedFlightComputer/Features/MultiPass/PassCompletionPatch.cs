using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.PlanWindow;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.RcsTranslation;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// The multi-pass state machine, ticked on the main thread before input events are drained.
// Stock ends every pass. FlightComputer.RaisePendingAlerts acts on the completion or propellant stop that the vehicle worker raised for an engine pass or the RCS executor raised for an RCS pass, and FlightComputer.EndAutoBurn removes the pass when another burn follows it. SharedVehicleHooks hands that outcome to OnStockBurnEnded.
internal static class PassCompletionPatch
{
    private const int MaxAwaitingMaterializationTicks = 4;
    private const int MaxConsecutiveScheduleFailures = 5;

    // Runs after FlightComputer.RaisePendingAlerts with the workers joined, so the plan can be changed directly.
    // keptByStock says whether FlightComputer.EndAutoBurn left the burn in the plan, which it does only for the last planned burn.
    internal static void OnStockBurnEnded(Vehicle vehicle, Burn burn, bool completed, bool keptByStock)
    {
        if (!MultiPassRegistry.TryGet(vehicle.Id, out MultiPassExecution? exec)
            || !ReferenceEquals(exec.CurrentBurn, burn))
            return;

        try
        {
            if (completed)
            {
                if (MultiPassDebug.Enabled)
                    DefaultCategory.Log.Debug(
                        $"[AFC] MultiPass: vehicle='{vehicle.Id}' pass {exec.PassIndex + 1}/{exec.PassCountTotal} " +
                        $"completed, {(keptByStock ? "kept" : "removed")} by stock.");
                CompletePass(vehicle.Id, exec, vehicle.FlightComputer);
            }
            else
            {
                StopPass(vehicle, exec, keptByStock);
            }
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error(
                $"[AFC] MultiPass: vehicle={vehicle.Id} stock burn end threw, cancelling execution: {ex}");
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

            ObserveArming(vehicle, exec, fc);

            if (UpdateMaterializationTracking(vehicle, exec, fc) == MaterializationResult.Cancelled)
                return;

            AdvanceExecution(vehicle, exec, fc);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error(
                $"[AFC] MultiPass: vehicle={vehicle.Id} tick threw, cancelling execution: {ex}");
            CancelExecution(vehicle.Id, reason: null);
        }
    }

    // Auto counts only on the pass itself, because an unrelated burn loaded ahead of it can be armed too.
    private static void ObserveArming(Vehicle vehicle, MultiPassExecution exec, FlightComputer fc)
    {
        bool autoOnPass = fc.BurnMode == FlightComputerBurnMode.Auto && IsLoadedPass(fc, exec);
        if (!autoOnPass && !RcsIsFlyingPass(vehicle, exec))
            return;

        // A new attempt on this pass permits another stall hint if the user retries it.
        exec.StallHintShown = false;
        if (exec.PassArmed)
            return;
        exec.PassArmed = true;
        if (MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPass: vehicle='{vehicle.Id}' pass " +
                $"{exec.PassIndex + 1}/{exec.PassCountTotal} armed {(autoOnPass ? "in Auto" : "for RCS")} " +
                $"(burn t={exec.CurrentBurn!.Time.Seconds():F1}s).");
    }

    private static void AdvanceExecution(Vehicle vehicle, MultiPassExecution exec, FlightComputer fc)
    {
        // OnStockBurnEnded settles a pass that completed or stopped, so a pass burn missing from the plan here was deleted.
        if (DetectExternalDelete(exec, fc))
        {
            CancelExecution(vehicle.Id,
                "pending burn was deleted externally; cancelling");
            return;
        }

        if (exec.CurrentBurn != null)
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

            // Carry Auto into the next pass after FlightComputer.LoadBurn has loaded it in Manual.
            // FlightComputer.AddBurn loads the pass only when it is the first executable burn, so with another burn ahead of it the request is dropped rather than arming that burn.
            // RCS can accept this request while the burn mode stays in Manual.
            if (exec.ReengageAutoOnNextBurn)
            {
                exec.ReengageAutoOnNextBurn = false;
                if (IsLoadedPass(fc, exec))
                {
                    vehicle.SetEnum(FlightComputerBurnMode.Auto);
                    // Record the pass only if Auto or RCS accepted the request.
                    if (fc.BurnMode == FlightComputerBurnMode.Auto || RcsIsFlyingPass(vehicle, exec))
                        exec.PassArmed = true;
                }
                if (MultiPassDebug.Enabled)
                    DefaultCategory.Log.Debug(
                        $"[AFC] MultiPass: vehicle={vehicleId} pass {exec.PassIndex + 1}/{exec.PassCountTotal} " +
                        (exec.PassArmed ? "re-engaged." : "left in Manual, another burn is loaded first or the request was refused."));
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

    // Removes the pass burn while it is still planned, which is the case for a pass stock kept as the last burn, then moves on to the next pass.
    private static void CompletePass(string vehicleId, MultiPassExecution exec, FlightComputer fc)
    {
        if (exec.CurrentBurn != null && fc.BurnPlan.TryGetBurn(exec.CurrentBurn))
        {
            if (MultiPassDebug.Enabled)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPass.CompletePass: vehicle='{vehicleId}' removing " +
                    $"burn t={exec.CurrentBurn.Time.Seconds():F1}s " +
                    $"dv={exec.CurrentBurn.DeltaVVlf.Length():F2}m/s");
            fc.RemoveBurn(exec.CurrentBurn);
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

    // Stock has already warned about the stop.
    // A pass stock kept waits in Manual for the player, who can refuel and engage it again. A pass stock removed is planned again from the current orbit on the next tick, also in Manual, so the player can stage and arm it.
    // With no propellant left anywhere a pass planned again could never fly, so the execution ends instead of planning it.
    // An unstaged engine counts, because staging it lets the pass fly, and so do the RCS thrusters when the RCS executor can fly a pass.
    private static void StopPass(Vehicle vehicle, MultiPassExecution exec, bool keptByStock)
    {
        string vehicleId = vehicle.Id;
        if (!keptByStock && !VehiclePropellant.AnyUsable(vehicle, includeRcs: SharedVehicleHooks.RcsEnabled))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPass: vehicle={vehicleId} pass {exec.PassIndex + 1}/{exec.PassCountTotal} " +
                "stopped and no engine or RCS thruster has propellant left, so the execution is cancelled.");
            TimedAlert.CreateWarning(ReferenceEquals(vehicle, Program.ControlledVehicle)
                ? "Multi-pass cancelled: no propellant left for the remaining passes."
                : $"{vehicleId}: multi-pass cancelled, no propellant left.");
            CancelExecution(vehicleId, reason: null);
            return;
        }

        if (!keptByStock)
            exec.ClearCurrentBurn();
        exec.ReengageAutoOnNextBurn = false;
        exec.StallHintShown = true;

        DefaultCategory.Log.Warning(
            $"[AFC] MultiPass: vehicle={vehicleId} pass {exec.PassIndex + 1}/{exec.PassCountTotal} " +
            "stopped with dV remaining because its engines or RCS thrusters ran out of propellant. " +
            (keptByStock
                ? "The pass stays planned until it is engaged again or the plan is cancelled."
                : "The game removed it, so it is planned again from the current orbit."));
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
                $"{Universe.GetElapsedTime().Seconds():F1}s (armed={exec.PassArmed}); treating as user delete.");
        return fired;
    }

    // The worker drops a completed or dry engine pass to Manual in the same step that raises the stock flag, and FlightComputer.RaisePendingAlerts acts on that flag later in this frame. The RCS executor raises its flag after this tick, so an RCS pass is still flying here.
    // While a flag is set, OnStockBurnEnded settles the pass, so only Manual without a flag on an armed pass means it was disengaged.
    // Preserve the execution and show only one hint.
    private static void MaybeAlertStalledPass(
        Vehicle vehicle, MultiPassExecution exec, FlightComputer fc)
    {
        if (exec.StallHintShown) return;
        if (fc.AutoBurnCompleted || fc.AutoBurnStoppedOutOfPropellant) return;
        if (exec.AwaitingMaterialization) return;
        if (exec.CurrentBurn == null) return;
        // A pass that was never armed can still be waiting for execution.
        if (!exec.PassArmed) return;
        // The RCS executor flies with the burn mode at Manual, so a running execution is not a stall.
        if (RcsIsFlyingPass(vehicle, exec)) return;
        if (fc.BurnMode != FlightComputerBurnMode.Manual) return;
        if (!fc.BurnPlan.TryGetBurn(exec.CurrentBurn)) return;
        // Auto includes alignment. Wait until the planned impulsive time before reporting a stall.
        if (Universe.GetElapsedTime() < exec.CurrentBurn.ImpulseTime) return;

        exec.StallHintShown = true;
        DefaultCategory.Log.Warning(
            $"[AFC] MultiPass: vehicle={vehicle.Id} pass {exec.PassIndex + 1}/" +
            $"{exec.PassCountTotal} stopped with dV remaining because execution was " +
            "disengaged. The execution stays paused until it is engaged again or the plan is cancelled.");
        TimedAlert.Create(
            $"Multi-pass pass {exec.PassIndex + 1}/{exec.PassCountTotal} stopped with " +
            "dV remaining. Re-engage Auto to continue, or cancel the remaining passes.",
            Color.Yellow, 6.0);
    }

    // The loaded target belongs to the pass, which FlightComputer.LoadBurn makes true only for the first executable burn.
    private static bool IsLoadedPass(FlightComputer fc, MultiPassExecution exec)
        => exec.CurrentBurn != null
           && ReferenceEquals(fc.BurnPlan.FindFirstExecutableBurn(), exec.CurrentBurn)
           && StockBurnIdentity.IsLoaded(fc.Burn, exec.CurrentBurn);

    // RCS stays in Manual and can fly another burn on the vehicle.
    // The RCS driver resolves its burn reference again after a load, and until then only the persisted time identifies the burn.
    private static bool RcsIsFlyingPass(Vehicle vehicle, MultiPassExecution exec)
    {
        if (exec.CurrentBurn == null) return false;
        if (!SharedVehicleHooks.RcsEnabled) return false;
        if (!RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? rcs)) return false;
        if (rcs.ActiveBurn != null) return ReferenceEquals(rcs.ActiveBurn, exec.CurrentBurn);
        if (rcs.ActiveBurnTimeSec is not double activeTimeSec) return false;
        return Math.Abs(activeTimeSec - exec.CurrentBurn.Time.Seconds()) <= StockBurnIdentity.ToleranceSec;
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
            // CelestialSystem.GetIndex can throw for an invalid source index.
            // Do not let that escape from the ApplyVehicleSolvers postfix.
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

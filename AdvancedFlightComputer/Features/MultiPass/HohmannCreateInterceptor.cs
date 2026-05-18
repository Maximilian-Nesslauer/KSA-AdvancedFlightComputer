using System;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Transpiler-injected wrapper around <see cref="Burn.Create"/> at the
/// stock Hohmann Create button site (<c>TransferPlanner.DrawPlanWindow</c>
/// line ~443). Routes to multi-pass when the inline UI is armed (N>1
/// with a valid preview); otherwise returns a stock single-burn.
///
/// Lets users click ONE Create button - stock's "Create" - and get
/// either single-burn or multi-pass based on the pass count selector.
/// No separate "Create Multi-Pass" button needed.
/// </summary>
internal static class HohmannCreateInterceptor
{
    /// <summary>
    /// Drop-in replacement for <see cref="Burn.Create(OrbitPointCce, double, double3, PatchedConic, Vehicle)"/>.
    /// Same signature so the transpiler can swap the call instruction
    /// directly without rewriting stack effects.
    /// </summary>
    public static Burn CreateMaybeMultiPass(
        OrbitPointCce point, double time, double3 deltaVVlf,
        PatchedConic patch, Vehicle vehicle)
    {
        try
        {
            // Active execution already exists for this vehicle. Stock's
            // Create-button guard `if (_transferBurn == null)` normally
            // blocks re-clicks - PassCompletionPatch keeps _transferBurn
            // pointing to the current pass burn for exactly this reason -
            // so this branch is a safety net for the narrow window after
            // a save load (before ReconcileAfterLoad runs) or any other
            // case where the sync slipped. Surface an alert and don't
            // start a new exec; the existing one continues.
            if (MultiPassRegistry.Has(vehicle.Id))
            {
                TimedAlert.Create(
                    "Multi-pass already running for this vehicle. " +
                    "Cancel it from the inline section before starting a new one.",
                    Color.Yellow, 5.0);
                DefaultCategory.Log.Warning(
                    $"[AFC] HohmannCreateInterceptor: re-click for vehicle={vehicle.Id} " +
                    "with active exec (sync gap?); falling back to stock single burn. " +
                    "User should cancel the active multi-pass first.");
                return Burn.Create(point, time, deltaVVlf, patch, vehicle);
            }

            // Fall through to stock single burn: either the user picked
            // N=1 (intended single), or TryGetArmedState found a problem
            // (preview failed, source mismatch, etc.). The failed-preview
            // case gets a TimedAlert below so we don't silently give the
            // user something different from what they asked for.
            if (!HohmannMultiPassUI.TryGetArmedState(vehicle,
                    out int passCount, out HohmannTransferIntent? intent))
            {
                if (HohmannMultiPassUI.WantedMultiPassButPreviewFailed())
                    TimedAlert.Create(
                        "Multi-pass preview failed; firing single burn instead.",
                        Color.Yellow, 4.0);
                return Burn.Create(point, time, deltaVVlf, patch, vehicle);
            }

            Burn? multiPassBurn = TryStartMultiPass(vehicle, intent!, passCount);
            if (multiPassBurn == null)
            {
                TimedAlert.Create(
                    "Multi-pass setup failed, falling back to single burn",
                    Color.Yellow, 4.0);
                return Burn.Create(point, time, deltaVVlf, patch, vehicle);
            }
            return multiPassBurn;
        }
        catch (Exception ex)
        {
            // Don't let our interceptor crash stock's UI. On any failure,
            // fall through to the original Burn.Create semantics.
            DefaultCategory.Log.Warning(
                $"[AFC] HohmannCreateInterceptor: {ex}; falling back to stock single burn.");
            return Burn.Create(point, time, deltaVVlf, patch, vehicle);
        }
    }

    private static Burn? TryStartMultiPass(
        Vehicle vehicle, HohmannTransferIntent intent, int passCount)
    {
        var exec = new MultiPassExecution
        {
            SaveId = SaveLoadObserver.CurrentSaveId,
            VehicleId = vehicle.Id,
            Intent = intent,
            // SplitMode is ignored by HohmannTransferIntent.RecomputePass
            // (per-pass dV is derived from the K-sequence). EqualBurnTime
            // is the stable serialization default; see HohmannMultiPassUI.
            Mode = SplitMode.EqualBurnTime,
            PassCountTotal = passCount,
            PassIndex = 0,
        };

        var plan = intent.RecomputePass(vehicle, 0, passCount, SplitMode.EqualBurnTime);
        if (plan.Pass == null)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] HohmannCreateInterceptor: pass 0 plan failed: " +
                $"{plan.FailureReason ?? "unknown"}");
            return null;
        }

        PassPreview preview = plan.Pass.Value;
        PatchedConic? prePatch = vehicle.FlightPlan.TryFindPatch(preview.BurnTime);
        if (prePatch == null)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] HohmannCreateInterceptor: no patch at pass 0 time " +
                $"{preview.BurnTime.Seconds():F0}s");
            return null;
        }

        OrbitPointCce point = prePatch.Orbit.GetPointAt(preview.BurnTime);
        Burn burn = Burn.Create(
            point, preview.BurnTime.Seconds(), preview.DvVlf, prePatch, vehicle);
        burn.IsGizmoActive = false;

        exec.AssignCurrentBurn(burn);
        MultiPassRegistry.Add(exec);

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] HohmannCreateInterceptor: started {passCount}-pass execution " +
                $"for vehicle={vehicle.Id}, pass 0 t={preview.BurnTime.Seconds():F0}s " +
                $"dv={preview.DvVlf.Length():F1}m/s.");
        return burn;
    }
}

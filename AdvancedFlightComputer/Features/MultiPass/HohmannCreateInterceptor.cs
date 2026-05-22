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
///
/// Also exposes <see cref="ShouldAllowCreateClick"/>, a click-time gate
/// injected one IL slot after the stock "Create" <c>DrawButton</c> call.
/// It absorbs the click while a multi-pass exec is already running for
/// the source vehicle, closing the post-load sync gap (stock's
/// <c>_transferBurn == null</c> guard would otherwise enter the
/// Burn.Create branch and queue a duplicate burn alongside the
/// reattached active-pass burn).
/// </summary>
internal static class HohmannCreateInterceptor
{
    private const string ActiveExecAlert =
        "Multi-pass already running for this vehicle. " +
        "Cancel it from the inline section before starting a new one.";

    /// <summary>
    /// Click gate for the stock "Create" button. See the file-level
    /// summary for the sync-gap rationale and the injection site.
    /// </summary>
    public static bool ShouldAllowCreateClick(bool wasClicked)
    {
        // Fast-path: ImGui's DrawButton returns true only on the frame
        // of an actual click, so 99.9% of frames bail here without
        // paying the reflection or registry lookup.
        if (!wasClicked) return false;
        try
        {
            if (GameReflection.TransferPlanner_Source == null) return true; // gate unwired; let stock proceed
            if (GameReflection.TransferPlanner_Source.Invoke(null, null) is not Vehicle source) return true;
            if (!MultiPassRegistry.Has(source.Id)) return true;

            TimedAlert.Create(ActiveExecAlert, Color.Yellow, 5.0);
            DefaultCategory.Log.Warning(
                $"[AFC] HohmannCreateInterceptor: Create click absorbed for " +
                $"vehicle={source.Id} (active multi-pass exec running; cancel it first).");
            return false;
        }
        catch (Exception ex)
        {
            // Fail-open: never break stock's UI on our reflection or
            // registry error. Worst case is the pre-gate behavior.
            DefaultCategory.Log.Warning(
                $"[AFC] HohmannCreateInterceptor.ShouldAllowCreateClick: {ex}; allowing click.");
            return true;
        }
    }

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
            // Defense-in-depth fallback. The primary protection is the
            // click-time gate ShouldAllowCreateClick injected at the
            // "Create" DrawButton site, which absorbs the click before
            // it ever reaches Burn.Create. Reaching this branch means
            // the gate let the click through; that happens in three
            // cases:
            //   1. The transpiler couldn't find the DrawButton anchor
            //      (warned at patch time).
            //   2. GameReflection.TransferPlanner_Source resolved null
            //      (warned by ValidateMultiPass at load).
            //   3. The gate's Invoke threw and it failed open.
            // In all three the click is unguarded by us, so stock
            // queues a fresh Burn from Burn.Create. This is the pre-
            // gate regression (mod-only - duplicate burn alongside the
            // reattached pass burn, no use-after-Dispose since the two
            // are distinct Burn references). We surface the alert plus
            // a warning so the broken-gate condition is visible.
            if (MultiPassRegistry.Has(vehicle.Id))
            {
                TimedAlert.Create(ActiveExecAlert, Color.Yellow, 5.0);
                DefaultCategory.Log.Warning(
                    $"[AFC] HohmannCreateInterceptor: re-click for vehicle={vehicle.Id} " +
                    "with active exec reached Burn.Create swap (click gate not active); " +
                    "falling back to stock single burn. Duplicate burn may queue.");
                return Burn.Create(point, time, deltaVVlf, patch, vehicle);
            }

            // Fall through to stock single burn: either the user picked
            // N=1 (intended single), or TryGetArmedState found a problem
            // (preview failed, source mismatch, etc.). The failed-preview
            // case gets a TimedAlert below so we don't silently give the
            // user something different from what they asked for.
            if (!HohmannMultiPassUI.TryGetArmedState(vehicle,
                    out int passCount, out HohmannTransferIntent? intent,
                    out SplitMode mode))
            {
                if (HohmannMultiPassUI.WantedMultiPassButPreviewFailed())
                    TimedAlert.Create(
                        "Multi-pass preview failed; firing single burn instead.",
                        Color.Yellow, 4.0);
                return Burn.Create(point, time, deltaVVlf, patch, vehicle);
            }

            Burn? multiPassBurn = TryStartMultiPass(vehicle, intent!, passCount, mode);
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
        Vehicle vehicle, HohmannTransferIntent intent, int passCount, SplitMode mode)
    {
        var exec = new MultiPassExecution
        {
            SaveId = SaveLoadObserver.CurrentSaveId,
            VehicleId = vehicle.Id,
            Intent = intent,
            // SplitMode drives the real-K schedule via Splitter.Allocate;
            // EqualBurnTime is the literature-standard default for
            // finite-burn loss across N periapsis kicks.
            Mode = mode,
            PassCountTotal = passCount,
            PassIndex = 0,
        };

        var plan = intent.RecomputePass(vehicle, 0, passCount, mode);
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

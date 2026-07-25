using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Transpiler-injected wrapper around <see cref="Burn.Create"/> at the
/// stock Hohmann Create button site, the single Burn.Create call in
/// <c>TransferPlanner.DrawPlanWindow</c>'s own body. Routes to multi-pass
/// when the inline UI is armed (N>1 with a valid preview); otherwise
/// returns a stock single-burn.
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
            if (StockPlanner.SourceVehicle is not Vehicle source) return true;
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
            // the gate let the click through; that happens in two
            // cases:
            //   1. The transpiler couldn't find the DrawButton anchor
            //      (warned at patch time).
            //   2. Resolving the source threw and the gate failed open.
            // In both the click is unguarded by us, so stock
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

            // Multi-pass armed (N > 1 with a valid preview). The intent is
            // flyby-retargeted inside BuildIntent when the flyby option is on,
            // so the split departure produces the flyby.
            if (HohmannMultiPassUI.TryGetArmedState(vehicle,
                    out int passCount, out HohmannTransferIntent? intent,
                    out SplitMode mode))
            {
                Burn? multiPassBurn = TryStartMultiPass(vehicle, intent!, passCount, mode);
                if (multiPassBurn != null) return multiPassBurn;
                TimedAlert.Create(
                    "Multi-pass setup failed, falling back to single burn",
                    Color.Yellow, 4.0);
                return Burn.Create(point, time, deltaVVlf, patch, vehicle);
            }

            // Single-burn flyby (N == 1, or multi-pass preview failed): fire the
            // retargeted flyby departure directly instead of the center-aimed
            // stock burn, eliminating the separate impact-to-flyby correction.
            if (HohmannFlybyUI.TryGetArmed(vehicle, out FlybyTargeting.FlybyResult flyby))
            {
                // Reaching here with a split selected means multi-pass could not be
                // armed (failed preview, or the click-time intent rebuild failing
                // after a good preview). Say so rather than quietly firing one burn.
                if (HohmannMultiPassUI.WantsMultiPass)
                {
                    TimedAlert.Create(
                        "Multi-pass could not be armed; firing a single flyby burn instead.",
                        Color.Yellow, 4.0);
                    DefaultCategory.Log.Warning(
                        $"[AFC] HohmannCreateInterceptor: vehicle={vehicle.Id} requested a split " +
                        "but multi-pass was not armed; falling back to a single flyby burn.");
                }

                Burn? flybyBurn = TryCreateFlybyBurn(vehicle, flyby);
                if (flybyBurn != null) return flybyBurn;
                TimedAlert.Create(
                    "Flyby retarget failed; firing the stock (impact-aimed) burn.",
                    Color.Yellow, 4.0);
                return Burn.Create(point, time, deltaVVlf, patch, vehicle);
            }

            // No AFC option armed: fall through to the stock single burn. The
            // failed-multi-pass-preview case gets an alert so we don't silently
            // give the user something different from what they asked for.
            if (HohmannMultiPassUI.WantedMultiPassButPreviewFailed())
                TimedAlert.Create(
                    "Multi-pass preview failed; firing single burn instead.",
                    Color.Yellow, 4.0);
            else if (HohmannFlybyUI.FlybyRequested)
            {
                // Checkbox is on but the retarget never armed (no valid cached
                // result for this vehicle / geometry). Without this the user
                // would get the center-aimed impact burn with no indication.
                TimedAlert.Create(
                    "Flyby not applied (no valid retarget); firing the stock impact-aimed burn.",
                    Color.Yellow, 4.0);
                DefaultCategory.Log.Warning(
                    $"[AFC] HohmannCreateInterceptor: flyby requested for vehicle={vehicle.Id} " +
                    "but TryGetArmed returned false; stock center-aimed burn created.");
            }

            if (DebugConfig.Flyby)
                DefaultCategory.Log.Debug(
                    $"[AFC] HohmannCreateInterceptor: stock single burn for vehicle={vehicle.Id} " +
                    $"t={time:F0}s dv={deltaVVlf.Length():F1}m/s " +
                    $"(flybyRequested={HohmannFlybyUI.FlybyRequested}).");
            return Burn.Create(point, time, deltaVVlf, patch, vehicle);
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

    /// <summary>Builds a single flyby departure burn from a retargeted
    /// <see cref="FlybyTargeting.FlybyResult"/>. Returns null if no flight-plan
    /// patch covers the (possibly nudged) flyby burn time. The caller (stock's
    /// Create path) adds the returned burn to the burn plan, matching the
    /// single-burn and multi-pass-pass-0 handoff.</summary>
    private static Burn? TryCreateFlybyBurn(Vehicle vehicle, FlybyTargeting.FlybyResult flyby)
    {
        PatchedConic? patch = vehicle.FlightPlan.TryFindPatch(flyby.BurnTime);
        if (patch == null)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] HohmannCreateInterceptor: no patch at flyby burn time " +
                $"{flyby.BurnTime.Seconds():F0}s for vehicle={vehicle.Id}");
            return null;
        }

        OrbitPointCce pointCce = patch.Orbit.GetPointAt(flyby.BurnTime);
        Burn burn = Burn.Create(
            pointCce, flyby.BurnTime.Seconds(), flyby.DvVlf, patch, vehicle);
        burn.IsGizmoActive = false;

        if (DebugConfig.Flyby)
            DefaultCategory.Log.Debug(
                $"[AFC] HohmannCreateInterceptor: single flyby burn for vehicle={vehicle.Id} " +
                $"t={flyby.BurnTime.Seconds():F0}s dv={flyby.DvVlf.Length():F1}m/s " +
                $"rp={flyby.TargetPeRadiusMeters:F0}m b={flyby.ImpactParameterMeters:F0}m.");
        return burn;
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

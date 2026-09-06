using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// Stock adds the returned burn. The click gate must reject an active execution before this replacement runs.
internal static class HohmannCreateInterceptor
{
    private const string ActiveExecAlert =
        "Multi-pass already running for this vehicle. " +
        "Cancel it from the inline section before starting a new one.";

    public static bool ShouldAllowCreateClick(bool wasClicked)
    {
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
            // Keep the stock UI available if the gate fails.
            DefaultCategory.Log.Warning(
                $"[AFC] HohmannCreateInterceptor.ShouldAllowCreateClick: {ex}; allowing click.");
            return true;
        }
    }

    // Keep the Burn.Create signature so the stack arguments remain valid.
    public static Burn CreateMaybeMultiPass(
        OrbitPointCce point, double time, double3 deltaVVlf,
        PatchedConic patch, Vehicle vehicle)
    {
        try
        {
            // A failed gate must return a separate stock burn, never a reference to the active pass.
            if (MultiPassRegistry.Has(vehicle.Id))
            {
                TimedAlert.Create(ActiveExecAlert, Color.Yellow, 5.0);
                DefaultCategory.Log.Warning(
                    $"[AFC] HohmannCreateInterceptor: re-click for vehicle={vehicle.Id} " +
                    "with active exec reached Burn.Create swap (click gate not active); " +
                    "falling back to stock single burn. Duplicate burn may queue.");
                return Burn.Create(point, time, deltaVVlf, patch, vehicle);
            }

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

            if (HohmannFlybyUI.TryGetArmed(vehicle, out FlybyTargeting.FlybyResult flyby))
            {
                // Alert the user if the requested split falls back to a single burn.
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

            if (HohmannMultiPassUI.WantedMultiPassButPreviewFailed())
                TimedAlert.Create(
                    "Multi-pass preview failed; firing single burn instead.",
                    Color.Yellow, 4.0);
            else if (HohmannFlybyUI.FlybyRequested)
            {
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
            DefaultCategory.Log.Warning(
                $"[AFC] HohmannCreateInterceptor: {ex}; falling back to stock single burn.");
            return Burn.Create(point, time, deltaVVlf, patch, vehicle);
        }
    }

    // Stock adds the returned burn, so adding it here would queue it twice.
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

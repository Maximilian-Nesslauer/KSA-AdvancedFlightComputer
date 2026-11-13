using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Transfer;

/// <summary>
/// The CREATE of the TRANSFER page. It builds the burn stock's planner would build for the selected
/// porkchop entry (refined for the finite burn, see TransferFlownPlan), or the first pass of a Hohmann
/// multi-pass, or the retargeted flyby departure refined the same way, and
/// then runs the sequence stock runs after its own CREATE in TransferPlanner.DrawPlanWindow:
/// IsGizmoActive off and a BurnUpdateData with AddBurn through InputEvents.BurnUpdateBuffer. The session
/// keeps the created burn as the guard against a second CREATE.
/// </summary>
internal static class TransferCommitter
{
    private const string ActiveExecAlert =
        "Multi-pass already running for this vehicle. Cancel it from the AFC head before starting a new one.";

    /// <summary>Why CREATE cannot run now, or empty. The TRANSFER footer shows the same refusal it acts on.</summary>
    public static string Refusal()
    {
        Vehicle? source = TransferSession.Source;
        if (source == null)
            return "NO CRAFT";
        if (TransferSession.Calculating)
            return "SOLVING";
        if (!TransferSession.Calculated)
            return "SOLVE FIRST";
        if (TransferSession.SelectedEntry == null)
            return "NO TRANSFER SELECTED";
        if (TransferSession.Refining)
            return "REFINING INTERCEPT";
        if (TransferSession.TransferBurn != null)
            return "TRANSFER CREATED";
        if (MultiPassRegistry.Has(source.Id))
            return "MULTI-PASS ACTIVE";
        if (source.FlightPlan.TryFindPatch(TransferSession.SelectedEntry.TransferData.Start) == null)
            return "DEPARTURE NOT ON TRAJECTORY";
        return HohmannFlybyUI.CreateRefusal(source);
    }

    public static void Create()
    {
        if (Refusal().Length > 0)
            return;
        Vehicle source = TransferSession.Source!;
        OrbitalTransfers.PorkChopEntry entry = TransferSession.SelectedEntry!;
        PatchedConic? patch = source.FlightPlan.TryFindPatch(entry.TransferData.Start);
        if (patch == null)
            return;

        OrbitPointCce point = patch.Orbit.GetPointAt(entry.TransferData.Start);
        // Stock's CREATE gives Burn.Create the delta v refined for the finite burn (TransferPlanner.SelectedPlan), so the single transfer burn here does the same.
        double3 dvVlf = TransferFlownPlan.DvVlf(source, entry);
        Burn? burn = CreateRequestedBurn(point, entry.TransferData.Start.Seconds(), dvVlf, patch, source);
        if (burn == null)
            return;
        burn.IsGizmoActive = false;
        InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
        {
            Burn = burn,
            FlightComputer = source.FlightComputer,
            AddBurn = true,
        });
        TransferSession.OnCreated(burn);
    }

    // Null when a flyby is requested and no burn can fly it. The transfer aimed at the centre of the body is another trajectory, one that impacts, so it is never created in place of a flyby.
    private static Burn? CreateRequestedBurn(
        OrbitPointCce point, double time, double3 deltaVVlf,
        PatchedConic patch, Vehicle vehicle)
    {
        if (MultiPassRegistry.Has(vehicle.Id))
        {
            AfcAlerts.Info(ActiveExecAlert, Color.Yellow, 5.0, vehicle);
            return Burn.Create(point, time, deltaVVlf, patch, vehicle);
        }

        try
        {
            if (HohmannMultiPassUI.TryGetArmedState(vehicle,
                    out int passCount, out HohmannTransferIntent? intent,
                    out SplitMode mode))
            {
                Burn? multiPassBurn = TryStartMultiPass(vehicle, intent!, passCount, mode);
                if (multiPassBurn != null) return multiPassBurn;

                if (HohmannFlybyUI.TryGetSingleDeparture(
                        vehicle, out FlybyTargeting.FlybyResult singleFlyby))
                {
                    Burn? flybyBurn = TryCreateFlybyBurn(vehicle, singleFlyby);
                    if (flybyBurn != null)
                    {
                        AfcAlerts.Info("Multi-pass setup failed; firing a single flyby burn instead.", Color.Yellow, 4.0, vehicle);
                        return flybyBurn;
                    }
                }

                if (HohmannFlybyUI.FlybyRequested)
                    return RefuseFlyby(vehicle, "the passes and the single flyby departure could not be set up");
                AfcAlerts.Info("Multi-pass setup failed, falling back to single burn", Color.Yellow, 4.0, vehicle);
                return Burn.Create(point, time, deltaVVlf, patch, vehicle);
            }

            if (HohmannFlybyUI.TryGetArmed(vehicle, out FlybyTargeting.FlybyResult flyby))
            {
                // Alert the user if the requested split falls back to a single burn.
                if (HohmannMultiPassUI.WantsMultiPass)
                {
                    AfcAlerts.Info("Multi-pass could not be armed; firing a single flyby burn instead.", Color.Yellow, 4.0, vehicle);
                    DefaultCategory.Log.Warning(
                        $"[AFC] TransferCommitter: vehicle={vehicle.Id} requested a split " +
                        "but multi-pass was not armed; falling back to a single flyby burn.");
                }

                Burn? flybyBurn = TryCreateFlybyBurn(vehicle, flyby);
                if (flybyBurn != null) return flybyBurn;
                return RefuseFlyby(vehicle, "the departure time is not on the trajectory");
            }

            if (HohmannFlybyUI.FlybyRequested)
                return RefuseFlyby(vehicle, "no valid flyby departure");
            if (HohmannMultiPassUI.WantedMultiPassButPreviewFailed())
                AfcAlerts.Info("Multi-pass preview failed; firing single burn instead.", Color.Yellow, 4.0, vehicle);

            if (DebugConfig.Flyby)
                DefaultCategory.Log.Debug(
                    $"[AFC] TransferCommitter: single burn for vehicle={vehicle.Id} " +
                    $"t={time:F0}s dv={deltaVVlf.Length():F1}m/s " +
                    $"(flybyRequested={HohmannFlybyUI.FlybyRequested}).");
            return Burn.Create(point, time, deltaVVlf, patch, vehicle);
        }
        catch (Exception ex)
        {
            if (HohmannFlybyUI.FlybyRequested)
            {
                DefaultCategory.Log.Warning($"[AFC] TransferCommitter: {ex}");
                return RefuseFlyby(vehicle, "the flyby setup failed");
            }
            DefaultCategory.Log.Warning(
                $"[AFC] TransferCommitter: {ex}; falling back to the single transfer burn.");
            return Burn.Create(point, time, deltaVVlf, patch, vehicle);
        }
    }

    private static Burn? RefuseFlyby(Vehicle vehicle, string reason)
    {
        AfcAlerts.Info($"No burn created: {reason}. Change the flyby or turn TARGET FLYBY PERIAPSIS off.", Color.Yellow, 6.0, vehicle);
        DefaultCategory.Log.Warning($"[AFC] TransferCommitter: flyby requested for vehicle={vehicle.Id}, no burn created: {reason}.");
        return null;
    }

    // The caller adds the returned burn, so adding it here would queue it twice.
    private static Burn? TryCreateFlybyBurn(Vehicle vehicle, FlybyTargeting.FlybyResult flyby)
    {
        PatchedConic? patch = vehicle.FlightPlan.TryFindPatch(flyby.BurnTime);
        if (patch == null)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] TransferCommitter: no patch at flyby burn time " +
                $"{flyby.BurnTime.Seconds():F0}s for vehicle={vehicle.Id}");
            return null;
        }

        OrbitPointCce pointCce = patch.Orbit.GetPointAt(flyby.BurnTime);
        // The flyby preview draws the departure refined for the finite burn, so the burn flies the same refinement.
        double3 dvVlf = HohmannFlybyUI.RefineDeparture(vehicle, patch, flyby);
        Burn burn = Burn.Create(
            pointCce, flyby.BurnTime.Seconds(), dvVlf, patch, vehicle);
        burn.IsGizmoActive = false;

        if (DebugConfig.Flyby)
            DefaultCategory.Log.Debug(
                $"[AFC] TransferCommitter: single flyby burn for vehicle={vehicle.Id} " +
                $"t={flyby.BurnTime.Seconds():F0}s dv={dvVlf.Length():F1}m/s " +
                $"(impulsive {flyby.DvVlf.Length():F1}m/s) " +
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
                $"[AFC] TransferCommitter: pass 0 plan failed: " +
                $"{plan.FailureReason ?? "unknown"}");
            return null;
        }

        PassPreview preview = plan.Pass.Value;
        PatchedConic? prePatch = vehicle.FlightPlan.TryFindPatch(preview.BurnTime);
        if (prePatch == null)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] TransferCommitter: no patch at pass 0 time " +
                $"{preview.BurnTime.Seconds():F0}s");
            return null;
        }

        OrbitPointCce point = prePatch.Orbit.GetPointAt(preview.BurnTime);
        Burn burn = Burn.Create(
            point, preview.BurnTime.Seconds(), preview.DvVlf, prePatch, vehicle);
        burn.IsGizmoActive = false;

        exec.AssignCurrentBurn(burn);
        MultiPassRegistry.Add(exec);

        if (MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(
                $"[AFC] TransferCommitter: started {passCount}-pass execution " +
                $"for vehicle={vehicle.Id}, pass 0 t={preview.BurnTime.Seconds():F0}s " +
                $"dv={preview.DvVlf.Length():F1}m/s.");
        return burn;
    }
}

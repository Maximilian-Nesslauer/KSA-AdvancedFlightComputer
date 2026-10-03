using AdvancedFlightComputer.Features.AutoRemove;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.RcsTranslation;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Core;

internal static class SharedVehicleHooks
{
    // Core remains patched when a feature fails to load. Only complete feature blocks may drive vehicles.
    internal static bool MultiPassEnabled { get; set; }
    internal static bool RcsEnabled { get; set; }
    internal static bool GuidanceEnabled { get; set; }
    internal static bool AutoStageEnabled { get; set; }
    internal static bool AutoRemoveEnabled { get; set; }

    internal static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(ApplySolversPatch)).Patch();
        harmony.CreateClassProcessor(typeof(PendingAlertsPatch)).Patch();
        harmony.CreateClassProcessor(typeof(DisposePatch)).Patch();
        harmony.CreateClassProcessor(typeof(SetNamePatch)).Patch();
    }

    internal static void Reset()
    {
        VehicleControlOwnership.Clear();
        MultiPassEnabled = false;
        RcsEnabled = false;
        GuidanceEnabled = false;
        AutoStageEnabled = false;
        AutoRemoveEnabled = false;
    }

    internal static void TickVehicles(ReadOnlySpan<Astronomical> bodies)
    {
        // Staging first, so a staging that keeps an Auto burn alive has taken back the stock out-of-propellant stop before anything else reads it.
        if (AutoStageEnabled)
            StagingDetector.Evaluate();

        foreach (Astronomical body in bodies)
        {
            if (body is not Vehicle vehicle || vehicle.IsDisposed)
                continue;

            // The pass state machine reads the burn mode before the RCS driver can change it.
            if (MultiPassEnabled)
                PassCompletionPatch.TickVehicle(vehicle);
            if (RcsEnabled)
                RcsDriverPatch.TickVehicle(vehicle);
        }
    }

    // The end of a burn, after FlightComputer.RaisePendingAlerts has acted on it. The worker raises the flags for an engine Auto burn and the RCS executor raises them for an RCS burn, so both arrive here.
    // MultiPass first, so a finished pass is advanced before AutoRemove looks at the plan.
    internal static void OnStockBurnEnded(Vehicle vehicle, in StockBurnEnd ended)
    {
        bool keptByStock = ReferenceEquals(vehicle.FlightComputer.BurnPlan.FindFirstExecutableBurn(), ended.Burn);
        if (MultiPassEnabled)
        {
            try
            {
                PassCompletionPatch.OnStockBurnEnded(vehicle, ended.Burn!, ended.Completed, keptByStock);
            }
            catch (Exception ex)
            {
                LogHelper.WarnOnce("stock-burn-ended-multipass:" + ex.GetType().Name,
                    $"[AFC] MultiPass threw on the end of a stock burn for '{vehicle.Id}': {ex}");
            }
        }
        if (AutoRemoveEnabled && ended.Completed)
        {
            try
            {
                FinishedBurnRemover.OnBurnCompleted(vehicle, ended.Burn!);
            }
            catch (Exception ex)
            {
                LogHelper.WarnOnce("stock-burn-ended-autoremove:" + ex.GetType().Name,
                    $"[AFC] AutoRemove threw on the end of a stock burn for '{vehicle.Id}': {ex}");
            }
        }
    }

    internal static void OnDisposed(Vehicle vehicle)
    {
        VehicleControlOwnership.ReleaseAll(vehicle);

        // A failed feature can still have loaded entries that the save observer will persist.
        VehicleDisposePatch.Remove(vehicle);
        RcsVehicleDisposePatch.Remove(vehicle);
        StagingConfig.RemoveVehicle(vehicle.Id);

        // Vehicle.Dispose has finished, so guidance releases only process resources.
        if (GuidanceEnabled)
            GuidanceWindow.ReleaseDisposedVehicle(vehicle);

        // Vehicle.Dispose leaves the part graph intact. Clear caches that can keep it reachable.
        MultiPassPreviewCache.OnVehicleDisposed(vehicle);
        HohmannMultiPassUI.OnVehicleDisposed(vehicle.Id);
        HohmannMultiPassPlanner.OnVehicleDisposed(vehicle.Id);
        StagingDetector.ForgetVehicle(vehicle);
    }

    // Program.PrepareFrame joins the workers before this call, then drains input events, then lets FlightComputer.RaisePendingAlerts end the burns the workers or the RCS executor finished or stopped.
    // The prefix runs before the worker results overwrite the burn state the staging detector compares against.
    [HarmonyPatch(typeof(Universe), nameof(Universe.ApplyVehicleSolvers), new Type[0])]
    private static class ApplySolversPatch
    {
        static void Prefix()
        {
            if (AutoStageEnabled)
                StagingDetector.Sample();
        }

        static void Postfix() => TickVehicles(LoadedVehicles.All);
    }

    // FlightComputer.RaisePendingAlerts clears the completion and propellant-stop flags and runs FlightComputer.EndAutoBurn, which removes the first executable burn only while more than one burn is planned.
    // The prefix records which burn the flags refer to before they are cleared, and the postfix sees whether stock kept that burn.
    [HarmonyPatch(typeof(FlightComputer), nameof(FlightComputer.RaisePendingAlerts), new[] { typeof(Vehicle) })]
    private static class PendingAlertsPatch
    {
        static void Prefix(FlightComputer __instance, out StockBurnEnd __state)
            => __state = MultiPassEnabled || AutoRemoveEnabled ? StockBurnEnd.Capture(__instance) : default;

        static void Postfix(Vehicle vehicle, StockBurnEnd __state)
        {
            if (__state.Burn != null && !vehicle.IsDisposed)
                OnStockBurnEnded(vehicle, in __state);
        }
    }

    // Move ID-keyed state while keeping the execution objects and their pending cleanup.
    internal static void OnRenamed(Vehicle vehicle, string oldId)
    {
        VehicleControlOwnership.NoteRename(vehicle);

        // A failed feature can still have loaded entries that the save observer will persist.
        RcsExecRegistry.RenameVehicle(oldId, vehicle.Id);
        MultiPassRegistry.RenameVehicle(oldId, vehicle.Id);
    }

    // Vehicle.Dispose() delegates to Dispose(bool), and EVA boarding calls the bool overload directly.
    [HarmonyPatch(typeof(Vehicle), nameof(Vehicle.Dispose), new[] { typeof(bool) })]
    private static class DisposePatch
    {
        static void Postfix(Vehicle __instance) => OnDisposed(__instance);
    }

    // A refused name leaves the vehicle unchanged, so it must not move registry entries.
    [HarmonyPatch(typeof(Vehicle), nameof(Vehicle.SetName))]
    private static class SetNamePatch
    {
        static void Prefix(Vehicle __instance, out string __state) => __state = __instance.Id;

        static void Postfix(Vehicle __instance, bool __result, string __state)
        {
            if (__result)
                OnRenamed(__instance, __state);
        }
    }
}

/// <summary>
/// The planned burn an engine Auto burn or an RCS burn ended on, taken before <c>FlightComputer.RaisePendingAlerts</c> clears the flags.
/// <see cref="Completed"/> separates <c>FlightComputer.AutoBurnCompleted</c>, the burn delivered its delta-V, from <c>FlightComputer.AutoBurnStoppedOutOfPropellant</c>, every active engine or RCS thruster ran dry.
/// </summary>
internal readonly struct StockBurnEnd(Burn? burn, bool completed)
{
    public Burn? Burn { get; } = burn;
    public bool Completed { get; } = completed;

    // The worker raises a flag only in Auto and drops the mode to Manual with it, and the RCS executor holds Manual while it flies, so one frame raises at most one of them.
    // The loaded target has to belong to the first executable burn, which is the one FlightComputer.EndAutoBurn removes.
    public static StockBurnEnd Capture(FlightComputer fc)
    {
        if (!fc.AutoBurnCompleted && !fc.AutoBurnStoppedOutOfPropellant)
            return default;
        Burn? burn = fc.BurnPlan.FindFirstExecutableBurn();
        return StockBurnIdentity.IsLoaded(fc.Burn, burn) ? new StockBurnEnd(burn, fc.AutoBurnCompleted) : default;
    }
}

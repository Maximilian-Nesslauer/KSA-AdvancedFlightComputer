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
        // Staging first: it restores Auto after a stage-induced Manual drop before MultiPass could read that drop as a stall.
        if (AutoStageEnabled)
            StagingDetector.Evaluate();

        foreach (Astronomical body in bodies)
        {
            if (body is not Vehicle vehicle || vehicle.IsDisposed)
                continue;

            // Observe stock burn completion before the RCS driver can change the burn mode.
            if (MultiPassEnabled)
                PassCompletionPatch.TickVehicle(vehicle);
            if (RcsEnabled)
                RcsDriverPatch.TickVehicle(vehicle);
        }

        // Last, so MultiPass observes the stock completion before the finished burn leaves the plan.
        if (AutoRemoveEnabled)
            FinishedBurnRemover.Tick();
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

    // Program.PrepareFrame joins the workers before this call and drains input events afterwards.
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

    // Move ID-keyed state while keeping the execution objects and their pending cleanup.
    internal static void OnRenamed(Vehicle vehicle, string oldId)
    {
        VehicleControlOwnership.NoteRename(vehicle);

        // A failed feature can still have loaded entries that the save observer will persist.
        RcsExecRegistry.RenameVehicle(oldId, vehicle.Id);
        MultiPassRegistry.RenameVehicle(oldId, vehicle.Id);
        PassCompletionPatch.RenameVehicle(oldId, vehicle.Id);
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

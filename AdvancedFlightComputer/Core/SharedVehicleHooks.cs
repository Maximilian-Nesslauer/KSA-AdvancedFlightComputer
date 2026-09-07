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

    internal static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(ApplySolversPatch)).Patch();
        harmony.CreateClassProcessor(typeof(DisposePatch)).Patch();
    }

    internal static void Reset()
    {
        MultiPassEnabled = false;
        RcsEnabled = false;
    }

    internal static void TickVehicles(ReadOnlySpan<Astronomical> bodies)
    {
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
    }

    internal static void OnDisposed(Vehicle vehicle)
    {
        // A failed feature can still have loaded entries that the save observer will persist.
        VehicleDisposePatch.Remove(vehicle);
        RcsVehicleDisposePatch.Remove(vehicle);

        // Vehicle.Dispose leaves the part graph intact. Clear caches that can keep it reachable.
        MultiPassPreviewCache.OnVehicleDisposed(vehicle);
        HohmannMultiPassUI.OnVehicleDisposed(vehicle.Id);
        HohmannMultiPassPlanner.OnVehicleDisposed(vehicle.Id);
    }

    // Program.PrepareFrame joins the workers before this call and drains input events afterwards.
    [HarmonyPatch(typeof(Universe), nameof(Universe.ApplyVehicleSolvers), new Type[0])]
    private static class ApplySolversPatch
    {
        static void Postfix() => TickVehicles(LoadedVehicles.All);
    }

    // Vehicle.Dispose() delegates to Dispose(bool), and EVA boarding calls the bool overload directly.
    [HarmonyPatch(typeof(Vehicle), nameof(Vehicle.Dispose), new[] { typeof(bool) })]
    private static class DisposePatch
    {
        static void Postfix(Vehicle __instance) => OnDisposed(__instance);
    }
}

using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// Remove saved execution state so a reused vehicle ID cannot inherit it.
internal static class VehicleDisposePatch
{
    internal static void Remove(Vehicle vehicle)
    {
        string vehicleId = vehicle.Id;
        if (string.IsNullOrEmpty(vehicleId)) return;
        if (!MultiPassRegistry.Has(vehicleId)) return;

        MultiPassRegistry.Remove(vehicleId);
        PassCompletionPatch.OnRegistryRemovedExternally(vehicleId);
    }
}

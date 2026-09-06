using System;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// Remove saved execution state so a reused vehicle ID cannot inherit it.
internal static class VehicleDisposePatch
{
    internal static void Remove(Vehicle vehicle)
    {
        try
        {
            string vehicleId = vehicle.Id;
            if (string.IsNullOrEmpty(vehicleId)) return;
            if (!MultiPassRegistry.Has(vehicleId)) return;

            MultiPassRegistry.Remove(vehicleId);
            PassCompletionPatch.OnRegistryRemovedExternally(vehicleId);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] VehicleDisposePatch: {ex}");
        }
    }
}

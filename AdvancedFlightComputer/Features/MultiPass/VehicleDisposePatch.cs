using System;
using AdvancedFlightComputer.Core;
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

            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(
                    $"[AFC] VehicleDisposePatch: vehicle='{vehicleId}' disposed, " +
                    "removing multi-pass execution.");

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

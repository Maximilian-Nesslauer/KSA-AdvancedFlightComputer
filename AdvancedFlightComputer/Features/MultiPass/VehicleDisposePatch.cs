using System;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Postfix on <see cref="Vehicle.Dispose"/> that drops the vehicle's
/// MultiPassRegistry entry when its underlying object goes away
/// (decoupled, destroyed, despawned). Without this, the entry would
/// persist in memory, get serialised on the next save, and could
/// attach to a freshly-created vehicle re-using the same Id.
/// </summary>
[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.Dispose), new Type[0])]
internal static class VehicleDisposePatch
{
    static void Postfix(Vehicle __instance)
    {
        try
        {
            string vehicleId = __instance.Id;
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
                $"[AFC] VehicleDisposePatch: {ex.Message}");
        }
    }
}

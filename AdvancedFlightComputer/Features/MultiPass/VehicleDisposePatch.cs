using System;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Postfix on <see cref="Vehicle.Dispose(bool)"/> that drops the vehicle's
/// MultiPassRegistry entry when its underlying object goes away
/// (decoupled, destroyed, despawned). Without this, the entry would
/// persist in memory, get serialised on the next save, and could
/// attach to a freshly-created vehicle re-using the same Id.
/// </summary>
// Dispose(bool) is the real teardown; the parameterless overload only delegates to it, and
// the EVA-boarding path calls Dispose(endMission: false) directly.
[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.Dispose), new[] { typeof(bool) })]
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
                $"[AFC] VehicleDisposePatch: {ex}");
        }
    }
}

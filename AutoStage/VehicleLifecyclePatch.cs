using AutoStage.Core;
using HarmonyLib;
using KSA;

namespace AutoStage;

// Dispose(bool), not Dispose(): the parameterless overload only delegates here, and the
// EVA-boarding path calls Dispose(endMission: false) directly, so patching the delegate
// would miss a kitten vehicle absorbed by boarding. Targeting the name alone is also
// ambiguous across the two overloads and throws at patch time.
[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.Dispose), new[] { typeof(bool) })]
static class Patch_Vehicle_Dispose
{
    // Prefix so Vehicle.Id is still valid.
    static void Prefix(Vehicle __instance)
    {
        Config.RemoveVehicle(__instance.Id);
        // Every per-vehicle cache keys on the Vehicle reference and would
        // otherwise pin the disposed vehicle and its whole part graph until
        // some other vehicle happens to displace it.
        StagingHelpers.ForgetVehicle(__instance);
        JettisonAnalysis.ForgetVehicle(__instance);
        StagingDetector.ForgetVehicle(__instance);
    }
}

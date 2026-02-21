using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Staging;

/// <summary>
/// Debug hook that runs StageAnalyzer after each staging event and logs the results.
/// Hooks into StageList.ActivateNextStage (which fires on every staging action)
/// to trigger a fresh analysis.
///
/// Also runs an initial analysis when the vehicle is first controlled, triggered
/// by a Postfix on Vehicle.UpdateFromTaskResults with a one-shot flag.
/// </summary>
static class StageAnalyzerDebug
{
    private static string? _lastVehicleId;

    internal static void Reset()
    {
        _lastVehicleId = null;
    }

    /// <summary>
    /// Runs after ActivateNextStage to re-analyze the vehicle post-staging.
    /// </summary>
    [HarmonyPatch(typeof(StageList), nameof(StageList.ActivateNextStage))]
    static class Patch_AnalyzeAfterStaging
    {
        static void Postfix(Vehicle vehicle)
        {
            if (vehicle != Program.ControlledVehicle)
                return;

            DefaultCategory.Log.Info("[AFC] Staging event detected, running StageAnalyzer...");
            StageAnalyzer.Analyze(vehicle, log: Mod.DebugMode);
        }
    }

    /// <summary>
    /// Runs periodically via UpdateFromTaskResults to catch the initial vehicle
    /// load or vehicle switch. Only triggers once per vehicle to avoid spam.
    /// </summary>
    [HarmonyPatch(typeof(Vehicle), "UpdateFromTaskResults")]
    static class Patch_InitialAnalysis
    {
        static void Postfix(Vehicle __instance)
        {
            if (__instance != Program.ControlledVehicle)
                return;

            string vehicleId = __instance.Id;
            if (vehicleId == _lastVehicleId)
                return;

            _lastVehicleId = vehicleId;

            DefaultCategory.Log.Info($"[AFC] Vehicle '{vehicleId}' detected, running initial StageAnalyzer...");
            StageAnalyzer.Analyze(__instance, log: Mod.DebugMode);
        }
    }
}

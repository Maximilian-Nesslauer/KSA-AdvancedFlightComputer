using Brutal.Logging;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

/// <summary>
/// Detects propellant depletion during auto-burns and triggers staging.
///
/// FlightComputer.UpdateBurnTarget (background thread) checks whether any
/// active engine has propellant. When none do, it sets BurnMode = Manual,
/// aborting the burn. We detect that Auto-to-Manual transition when the
/// background job results are applied back on the main thread, and call
/// ActivateNextStage to continue the burn with the next stage's engines.
///
/// Hook point: Vehicle.UpdateFromTaskResults runs on the main thread after
/// the VehicleUpdateTask job completes. This is safe because:
/// - ActivateNextStage modifies the part tree (Decoupler.Decouple creates
///   new vehicles, adds to parent.Children) and therefore must be on main thread
/// - Cannot use UpdatePerFrameData: it runs inside UpdatePerFrameDataTree
///   which iterates parent.Children . autostaging would modify that collection
///   mid-iteration (InvalidOperationException)
/// - Cannot use UpdateBurnTarget: it runs on the background worker thread
///
/// Prefix captures the BurnMode before the FC copy overwrites it, giving
/// us a clean transition detection without stale static state.
///
/// After staging, a grace period maintains BurnMode = Auto while
/// Rocket.UpdateRockets propagates IsPropellantAvailable to the newly
/// activated engines. Without this, the next frame's background job would
/// see engines without propellant and set BurnMode = Manual again.
///
/// Why 3 frames (1 needed, 2 margin):
///   Frame N:   Staging happens here. PrepareWorker snapshots the new engine
///              states (IsActive=true, IsPropellantAvailable=false).
///   N -> N+1 job: UpdateBurnTarget reads IsPropellantAvailable=false, sets
///              BurnMode=Manual. Then UpdateModules/Rocket.UpdateRockets
///              checks propellant and sets IsPropellantAvailable=true in the
///              state buffer. (ComputeControl runs before UpdateModules.)
///   Frame N+1: ApplyResults writes BurnMode=Manual AND the updated engine
///              states (IsPropellantAvailable=true) back to the Vehicle.
///              Grace forces BurnMode=Auto. PrepareWorker now snapshots
///              BurnMode=Auto + IsPropellantAvailable=true.
///   N+1 -> N+2 job: Propellant check passes. Burn continues normally.
/// </summary>
[HarmonyPatch(typeof(Vehicle), "UpdateFromTaskResults")]
static class Patch_AutoStageExecution
{
    private static int _graceFrames;

    internal static void Reset()
    {
        _graceFrames = 0;
    }

    static void Prefix(Vehicle __instance, out FlightComputerBurnMode __state)
    {
        __state = __instance.FlightComputer.BurnMode;
    }

    static void Postfix(Vehicle __instance, FlightComputerBurnMode __state)
    {
        if (__instance != Program.ControlledVehicle) return;

        if (!AutoStage.Enabled)
        {
            _graceFrames = 0;
            return;
        }

        FlightComputer fc = __instance.FlightComputer;

        if (fc.Burn == null)
        {
            _graceFrames = 0;
            return;
        }

        bool burnIncomplete = float3.Dot(fc.Burn.DeltaVToGoCci, fc.Burn.DeltaVTargetCci) > 0f;

        if (_graceFrames > 0)
        {
            _graceFrames--;
            if (burnIncomplete && fc.BurnMode == FlightComputerBurnMode.Manual)
                fc.BurnMode = FlightComputerBurnMode.Auto;
            return;
        }

        // __state = BurnMode before the background job's FC copy was applied.
        // fc.BurnMode = BurnMode from the background job (after UpdateBurnTarget).
        // Auto -> Manual with remaining dV means propellant depletion, not completion.
        if (__state == FlightComputerBurnMode.Auto
            && fc.BurnMode == FlightComputerBurnMode.Manual
            && burnIncomplete)
        {
            bool hasNextEngineStage = false;
            foreach (Stage stage in __instance.Parts.StageList.Stages)
            {
                if (!stage.Activated && stage.ContainsEngine)
                {
                    hasNextEngineStage = true;
                    break;
                }
            }

            if (hasNextEngineStage)
            {
                if (Mod.DebugMode)
                {
                    DefaultCategory.Log.Debug(
                        $"[AFC] Auto-staging: dV remaining = {fc.Burn.DeltaVToGoCci.Length():F1} m/s");
                }

                __instance.Parts.StageList.ActivateNextStage(__instance);
                fc.BurnMode = FlightComputerBurnMode.Auto;
                _graceFrames = 3; // see class doc comment for timing analysis
            }
        }
    }
}

using System.Diagnostics;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

/// <summary>
/// Detects propellant depletion and triggers staging. Works during auto-burns,
/// manual burns, and even without a planned burn (e.g. manual ascent).
///
/// Detection: The Prefix captures whether any active engine has propellant
/// (via ResourceManager.ResourceAvailable, the same check the game uses in
/// Rocket.UpdateRockets). The Postfix checks again after UpdateFromTaskResults
/// has applied the background job's state updates. A transition from "has
/// propellant" to "no propellant" triggers ActivateNextStage.
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
/// After staging, a grace period prevents re-triggering while
/// Rocket.UpdateRockets propagates IsPropellantAvailable to the newly
/// activated engines. For auto-burns, the grace period also maintains
/// BurnMode = Auto so the flight computer continues the burn.
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
///              Grace forces BurnMode=Auto (if auto-burn). PrepareWorker
///              now snapshots BurnMode=Auto + IsPropellantAvailable=true.
///   N+1 -> N+2 job: Propellant check passes. Burn continues normally.
/// </summary>
[HarmonyPatch(typeof(Vehicle), "UpdateFromTaskResults")]
static class Patch_AutoStageExecution
{
    private static int _graceFrames;

    /// <summary>
    /// The BurnMode that was active when staging was triggered.
    /// Used during grace to decide whether to force BurnMode=Auto
    /// (only for auto-burns; manual burns keep Manual).
    /// </summary>
    private static FlightComputerBurnMode _triggeredMode;

    internal static void Reset()
    {
        _graceFrames = 0;
        _triggeredMode = FlightComputerBurnMode.Manual;
    }

    static void Prefix(Vehicle __instance,
        out (FlightComputerBurnMode burnMode, bool hadPropellant) __state)
    {
        if (__instance != Program.ControlledVehicle)
        {
            __state = default;
            return;
        }
        __state = (
            __instance.FlightComputer.BurnMode,
            AutoStage.HasActiveEngineWithPropellant(__instance)
        );
    }

    static void Postfix(Vehicle __instance,
        (FlightComputerBurnMode burnMode, bool hadPropellant) __state)
    {
#if DEBUG
        long perfStart = DebugConfig.Performance ? Stopwatch.GetTimestamp() : 0;
#endif
        if (__instance != Program.ControlledVehicle) return;

        if (!AutoStage.Enabled)
        {
            _graceFrames = 0;
            return;
        }

        FlightComputer fc = __instance.FlightComputer;
        bool hasPropellant = AutoStage.HasActiveEngineWithPropellant(__instance);

        if (_graceFrames > 0)
        {
            _graceFrames--;

            // For auto-burns, maintain BurnMode=Auto during grace so the
            // background thread doesn't abort the burn before new engines
            // have their propellant state propagated.
            if (_triggeredMode == FlightComputerBurnMode.Auto
                && fc.Burn != null
                && IsBurnIncomplete(fc)
                && fc.BurnMode == FlightComputerBurnMode.Manual)
            {
                fc.BurnMode = FlightComputerBurnMode.Auto;
            }

            // On grace expiry, cascade if still no propellant.
            // Uses _triggeredMode (not __state.burnMode) because during grace
            // the background thread keeps setting BurnMode=Manual, so the
            // Prefix always captures Manual. The original trigger mode is what
            // matters for cascading.
            if (_graceFrames == 0
                && !hasPropellant
                && !IsBurnComplete(fc)
                && AutoStage.HasNextEngineStage(__instance))
            {
                ExecuteStaging(__instance, fc, _triggeredMode);
            }
        }
        else if (__state.hadPropellant && !hasPropellant
            && !IsBurnComplete(fc)
            && AutoStage.HasNextEngineStage(__instance))
        {
            // Propellant transition: had propellant before the update, lost it after.
            ExecuteStaging(__instance, fc, __state.burnMode);
        }

#if DEBUG
        if (DebugConfig.Performance)
            PerfTracker.Record("AutoStageExecution.Postfix", Stopwatch.GetTimestamp() - perfStart);
#endif
    }

    /// <summary>
    /// Returns true if a burn is planned and has been completed (remaining dV
    /// has reversed direction relative to target dV). A completed burn should
    /// not trigger staging even if propellant is depleted.
    /// Returns false if no burn is planned (staging is always allowed).
    /// </summary>
    private static bool IsBurnComplete(FlightComputer fc)
    {
        return fc.Burn != null
            && float3.Dot(fc.Burn.DeltaVToGoCci, fc.Burn.DeltaVTargetCci) <= 0f;
    }

    /// <summary>
    /// Returns true if a burn is planned and still has remaining dV.
    /// </summary>
    private static bool IsBurnIncomplete(FlightComputer fc)
    {
        return fc.Burn != null
            && float3.Dot(fc.Burn.DeltaVToGoCci, fc.Burn.DeltaVTargetCci) > 0f;
    }

    private static void ExecuteStaging(Vehicle vehicle, FlightComputer fc,
        FlightComputerBurnMode originalBurnMode)
    {
        if (DebugConfig.AutoStage)
        {
            string dvInfo = fc.Burn != null
                ? $"dV remaining = {fc.Burn.DeltaVToGoCci.Length():F1} m/s"
                : "no burn planned";
            DefaultCategory.Log.Debug(
                $"[AFC] Auto-staging ({originalBurnMode} mode): {dvInfo}");
        }

        vehicle.Parts.StageList.ActivateNextStage(vehicle);

        _triggeredMode = originalBurnMode;
        if (originalBurnMode == FlightComputerBurnMode.Auto && fc.Burn != null)
            fc.BurnMode = FlightComputerBurnMode.Auto;

        _graceFrames = 3; // see class doc comment for timing analysis
    }
}

using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

internal enum ProbeWarpRetry
{
    None,
    Interrupted,
    Offered,
    Requested,
    Spent
}

public static partial class GuidanceWindow
{
    private readonly record struct ProbeWarpStop(VehicleAutopilotState State, bool Moon);

    internal static void ApplyProbePowerWarpPatch(Harmony harmony)
    {
        var target = GameReflection.Vehicle_UpdateProbePowerWarning;
        if (target == null)
        {
            DefaultCategory.Log.Warning("[AFC] Probe power warp interruptions cannot be identified on this game version.");
            return;
        }
        harmony.Patch(target,
            prefix: new HarmonyMethod(typeof(GuidanceWindow), nameof(BeforeProbePowerWarning)),
            postfix: new HarmonyMethod(typeof(GuidanceWindow), nameof(AfterProbePowerWarning)));
        harmony.Patch(AccessTools.DeclaredMethod(typeof(Universe), nameof(Universe.ResetSimulationSpeed), Type.EmptyTypes),
            postfix: new HarmonyMethod(typeof(GuidanceWindow), nameof(CancelQueuedProbeWarp)));
        harmony.Patch(AccessTools.DeclaredMethod(typeof(InputEvents.SimSpeedData), nameof(InputEvents.SimSpeedData.Apply)),
            postfix: new HarmonyMethod(typeof(GuidanceWindow), nameof(AfterProbeSimulationSpeedInput)));
    }

    private static void BeforeProbePowerWarning(Vehicle __instance, out ProbeWarpStop? __state)
    {
        __state = null;
        if (!ModActive || !ReferenceEquals(__instance, Program.ControlledVehicle) || Universe.AutoWarpTime is not { } warp
            || !VehicleAutopilotState.TryGet(__instance, out var state))
            return;
        if (state.MoonChainPowerWarp == ProbeWarpRetry.None && state.MoonChainWarpAsked
            && state.MoonChain is MoonChainStage.Injection or MoonChainStage.Correction or MoonChainStage.Capture or MoonChainStage.Coast
            && double.IsFinite(state.MoonChainWarpTime) && warp == new UniverseTime(state.MoonChainWarpTime))
            __state = new ProbeWarpStop(state, Moon: true);
        else if (state.DeorbitPowerWarp == ProbeWarpRetry.None && state.DeorbitAutoArmed && state.LandingPhase == LandingPhase.DeorbitBurn
            && double.IsFinite(state.DeorbitWarpTime) && warp == new UniverseTime(state.DeorbitWarpTime))
            __state = new ProbeWarpStop(state, Moon: false);
    }

    private static void RetireEndedWarp(ref ProbeWarpRetry retry, double ownedWarp)
    {
        // A warp that stopped outside the stock warning cannot qualify later in the same wait.
        if (retry == ProbeWarpRetry.None && double.IsFinite(ownedWarp)
            && Universe.AutoWarpTime != new UniverseTime(ownedWarp))
            retry = ProbeWarpRetry.Spent;
    }

    // Vehicle.UpdateProbePowerWarning stops warp only on the new low-power warning.
    private static void AfterProbePowerWarning(ProbeWarpStop? __state)
    {
        if (__state is not { } stop || Universe.IsAutoWarpActive)
            return;
        if (stop.Moon)
            stop.State.MoonChainPowerWarp = ProbeWarpRetry.Interrupted;
        else
            stop.State.DeorbitPowerWarp = ProbeWarpRetry.Interrupted;
    }

    private static void CancelQueuedProbeWarp()
    {
        if (Program.ControlledVehicle is not { } vehicle || !VehicleAutopilotState.TryGet(vehicle, out var state))
            return;
        if (state.MoonChainPowerWarp is ProbeWarpRetry.Interrupted or ProbeWarpRetry.Offered or ProbeWarpRetry.Requested)
            state.MoonChainPowerWarp = ProbeWarpRetry.Spent;
        if (state.DeorbitPowerWarp is ProbeWarpRetry.Interrupted or ProbeWarpRetry.Offered or ProbeWarpRetry.Requested)
            state.DeorbitPowerWarp = ProbeWarpRetry.Spent;
    }

    // The stock Speed menu sets a SimSpeed directly, even when the current speed is already 1x.
    private static void AfterProbeSimulationSpeedInput(ref InputEvents.SimSpeedData __instance)
    {
        if (__instance.Speed is { } speed && speed.Approximately(1.0))
            CancelQueuedProbeWarp();
    }

    // Called only while the chain still owns the same wait. The console answers the offer on the main thread.
    private static bool StepProbeWarpRetry(Vehicle vehicle, ref ProbeWarpRetry retry, double target)
    {
        if (retry is ProbeWarpRetry.None or ProbeWarpRetry.Spent)
            return false;
        if (!ReferenceEquals(vehicle, Program.ControlledVehicle) || !vehicle.IsControllable
            || Universe.IsAutoWarpActive || Universe.GetSimulationSpeed() != 1.0
            || !double.IsFinite(target) || target <= SimNow() + 1.0)
        {
            retry = ProbeWarpRetry.Spent;
            return false;
        }
        if (retry == ProbeWarpRetry.Offered)
            return false;
        if (!(ProbePower.RemainingSeconds(vehicle) > 0.0))
        {
            retry = ProbeWarpRetry.Spent;
            return false;
        }
        if (retry != ProbeWarpRetry.Requested)
        {
            retry = ProbeWarpRetry.Offered;
            return false;
        }
        retry = ProbeWarpRetry.Spent;
        Universe.AutoWarpTo(new UniverseTime(target));
        return Universe.IsAutoWarpActive;
    }
}

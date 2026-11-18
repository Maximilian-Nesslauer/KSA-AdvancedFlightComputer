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
        var target = AccessTools.DeclaredMethod(typeof(Vehicle), "UpdateProbePowerWarning",
            [typeof(int), typeof(ProbeCorePower), typeof(UniverseTime), typeof(UniverseTime)]);
        if (target == null)
        {
            DefaultCategory.Log.Warning("[AFC] Probe power warp interruptions cannot be identified on this game version.");
            return;
        }
        harmony.Patch(target,
            prefix: new HarmonyMethod(typeof(GuidanceWindow), nameof(BeforeProbePowerWarning)),
            postfix: new HarmonyMethod(typeof(GuidanceWindow), nameof(AfterProbePowerWarning)));
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

    // Called only while the chain still owns the same wait. The console answers the offer on the main thread.
    private static bool StepProbeWarpRetry(Vehicle vehicle, ref ProbeWarpRetry retry, double target)
    {
        if (retry is ProbeWarpRetry.None or ProbeWarpRetry.Spent)
            return false;
        if (!ReferenceEquals(vehicle, Program.ControlledVehicle) || !vehicle.IsControllable
            || Universe.IsAutoWarpActive || Universe.GetSimulationSpeed() != 1.0
            || !double.IsFinite(target) || target <= SimNow() + 1.0
            || !(ProbePower.RemainingSeconds(vehicle) > 0.0))
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

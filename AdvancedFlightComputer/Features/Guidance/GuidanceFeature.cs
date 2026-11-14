using AdvancedFlightComputer.Core;
using KSA;
using Brutal.Logging;
using HarmonyLib;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// The guidance feature's three patch blocks. The diagnostics block owns the hook the AFC menu draws
/// from. The driver block owns the per-vehicle step and the worker hook, and its enable flag is what
/// the guidance pages of the console and the gimbal writer are gated on, so a driver that fails to
/// load still leaves a menu that says so. The save block, applied only after the driver block,
/// owns the save record postfix of the RCS hold, so its loss costs only that repair.
/// </summary>
internal static class GuidanceFeature
{
    internal const string UnavailableReason = "Guidance is unavailable because its game hooks did not load.";

    internal static void ApplyDiagnosticPatches(Harmony harmony)
    {
        harmony.Patch(GameReflection.Program_DrawProgramMenusHook!,
            postfix: new HarmonyMethod(typeof(AfcMenu), nameof(AfcMenu.Draw)));
        KeepMenuHookCall(harmony);
    }

    // A DrawMenuBar patch made before the hook patch can have the empty hook inlined. Patching it again rebuilds it with the call.
    private static void KeepMenuHookCall(Harmony harmony)
    {
        var menuBar = GameReflection.Program_DrawMenuBar;
        if (menuBar == null)
        {
            DefaultCategory.Log.Warning("[AFC] Program.DrawMenuBar was not found, so a mod that patches the menu bar can hide the AFC menu.");
            return;
        }

        try
        {
            string[] others = Harmony.GetPatchInfo(menuBar)?.Owners.Where(owner => owner != harmony.Id).ToArray() ?? [];
            harmony.Patch(menuBar, transpiler: new HarmonyMethod(typeof(GuidanceFeature), nameof(KeepMenuBar)));
            if (others.Length > 0)
                DefaultCategory.Log.Info($"[AFC] Program.DrawMenuBar is also patched by {string.Join(", ", others)}. It was compiled again so that the menu bar keeps its call to the menu hook.");
        }
        catch (Exception ex)
        {
            // Not rethrown, because a feature failure would also remove the guidance driver.
            DefaultCategory.Log.Warning($"[AFC] Could not compile Program.DrawMenuBar again, so a mod that patches the menu bar can hide the AFC menu: {ex}");
        }
    }

    private static IEnumerable<CodeInstruction> KeepMenuBar(IEnumerable<CodeInstruction> instructions) => instructions;

    internal static void ApplyDriverPatches(Harmony harmony)
    {
        harmony.Patch(GameReflection.Vehicle_PrepareWorker!,
            prefix: new HarmonyMethod(typeof(GuidanceFeature), nameof(OnPrepareWorker)));
        harmony.Patch(GameReflection.FlightComputer_UpdateAttitudeTarget!,
            postfix: new HarmonyMethod(typeof(GuidanceFeature), nameof(OnUpdateAttitudeTarget)));

        // A save replaces every vehicle, so the state keyed on the old ones is dropped.
        SaveLoadObserver.SaveLoaded += GuidanceWindow.ReleaseAllVehicles;
    }

    /// <summary>The postfix that gives a save record the player's RCS mode. It has its own patch set, so a failure costs this repair and not the driver.</summary>
    internal static void ApplySaveRcsPatch(Harmony harmony)
    {
        if (GameReflection.FlightComputer_SerializeSave is not { } serialize)
        {
            DefaultCategory.Log.Warning("[AFC] FlightComputer.SerializeSave was not found, so a save made while guidance holds RCS off for a landing burn loads with RCS off. Guidance runs as before.");
            return;
        }
        harmony.Patch(serialize, postfix: new HarmonyMethod(typeof(GuidanceFeature), nameof(OnSerializeFlightComputer)));
    }

    /// <summary>Releases what the driver holds when its block failed or the mod unloads.</summary>
    internal static void DisableDriver()
    {
        SaveLoadObserver.SaveLoaded -= GuidanceWindow.ReleaseAllVehicles;

        // The feature flag is already off, so no later step retries a failed cleanup.
        GuidanceWindow.ReleaseAllVehicles();
    }

    private static void OnSerializeFlightComputer(FlightComputer __instance, FlightComputerData __result)
    {
        try
        {
            GuidanceWindow.SaveRcsAsThePlayerHasIt(__instance, __result);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"guidance-save-rcs:{ex.GetType().Name}", $"[AFC] Guidance could not give a save the player's RCS mode: {ex}");
        }
    }

    // Runs on the main thread inside Universe.PrepareVehicleWorkers for every vehicle. See the handle in GameReflection for why this is the site.
    private static void OnPrepareWorker(Vehicle __instance) => StepVehicle(__instance);

    private static void StepVehicle(Vehicle vehicle)
    {
        // The flag outlives a patch that failed to unpatch, and a kitten on EVA is a vehicle too.
        if (!SharedVehicleHooks.GuidanceEnabled || vehicle is KittenEva)
            return;

        try
        {
            // A stop the console asked for on a craft that is not on screen is applied here, before the modes step.
            GuidanceWindow.DrainRequests(vehicle);
            GuidanceWindow.ApplyAutopilot(vehicle);
            GuidanceWindow.StepSucceeded(vehicle);
            // After the modes stepped, so it reads the claim and the phase this step left. It catches its own faults, which never count as a failed step.
            GuidanceWindow.StepLandingLegs(vehicle);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"guidance-step-{vehicle.Id}:{ex.GetType().Name}",
                $"[AFC] Guidance step failed on '{vehicle.Id}': {ex}");

            // A bad step keeps the craft on its last command, and faults that keep coming, or a fault on the ground, give it back. Counting reads the craft's situation, so it sits inside the guard that keeps a second fault out of the game's loop.
            try
            {
                if (!GuidanceWindow.CountFailedStep(vehicle))
                    return;
                LogHelper.WarnOnce($"guidance-step-release-{vehicle.Id}",
                    $"[AFC] Guidance released '{vehicle.Id}' after its steps kept failing.");
                GuidanceWindow.FailAutopilot(vehicle, ex);
            }
            catch (Exception release)
            {
                LogHelper.WarnOnce($"guidance-release-{vehicle.Id}:{release.GetType().Name}",
                    $"[AFC] Guidance could not release '{vehicle.Id}' after a failed step: {release}");
            }
        }
    }

    // Runs on the vehicle worker after stock builds the attitude target and before it reads the target rate. Both gates match the gimbal writer, so the worker-side writers stop together, and the catch keeps an escape from ending the worker's step for the whole physics bubble.
    private static void OnUpdateAttitudeTarget(FlightComputer __instance)
    {
        if (!SharedVehicleHooks.GuidanceEnabled || !GuidanceWindow.ModActive)
            return;

        try
        {
            KsaAttitudeRate.OnUpdateAttitudeTarget(__instance);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"guidance-rate:{ex.GetType().Name}",
                $"[AFC] Guidance rate feedforward failed on the vehicle worker: {ex}");
        }
    }

    internal static void Reset()
    {
        DisableDriver();
        GuidanceClock.Reset();
        ViewDemand.Reset();
    }
}

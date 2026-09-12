using System.Reflection;
using AdvancedFlightComputer.Core;
using KSA;
using Brutal.ImGuiApi;
using HarmonyLib;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// The guidance feature's two patch blocks. The diagnostics block owns the menu and the managed
/// resolver. The driver block owns the per-vehicle step and the worker hook, and its enable flag is
/// what the panel and the gimbal writer are gated on, so a driver that fails to load still leaves a
/// menu that says so.
/// </summary>
internal static class GuidanceFeature
{
    internal const string UnavailableReason = "Guidance is unavailable because its game hooks did not load.";

    private static bool _resolverRegistered;

    internal static void ApplyDiagnosticPatches(Harmony harmony)
    {
        if (!_resolverRegistered)
        {
            AppDomain.CurrentDomain.AssemblyResolve += ResolveManagedLibrary;
            _resolverRegistered = true;
        }

        harmony.Patch(GameReflection.Program_DrawProgramMenusHook!,
            postfix: new HarmonyMethod(typeof(GuidanceFeature), nameof(DrawMenu)));
    }

    internal static void ApplyDriverPatches(Harmony harmony)
    {
        harmony.Patch(GameReflection.Vehicle_PrepareWorker!,
            prefix: new HarmonyMethod(typeof(GuidanceFeature), nameof(OnPrepareWorker)));
        harmony.Patch(GameReflection.FlightComputer_UpdateAttitudeTarget!,
            postfix: new HarmonyMethod(typeof(GuidanceFeature), nameof(OnUpdateAttitudeTarget)));

        // A save replaces every vehicle, so the state keyed on the old ones is dropped.
        SaveLoadObserver.SaveLoaded += GuidanceWindow.ReleaseAllVehicles;
    }

    /// <summary>Releases what the driver holds when its block failed or the mod unloads.</summary>
    internal static void DisableDriver()
    {
        SaveLoadObserver.SaveLoaded -= GuidanceWindow.ReleaseAllVehicles;

        // The feature flag is already off, so no later step retries a failed cleanup.
        GuidanceWindow.ReleaseAllVehicles();
    }

    internal static void DrawGui()
    {
        // Guidance flies a craft, so its panel and overlays are flight UI and stay out of the
        // editor. Guidance itself keeps running; only the drawing is skipped.
        if (!SharedVehicleHooks.GuidanceEnabled || Program.IsEditorOpen)
            return;

        try
        {
            GuidanceWindow.Draw(Program.MainViewport);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"guidance-draw:{ex.GetType().Name}",
                $"[AFC] Guidance panel draw failed for '{Program.ControlledVehicle?.Id}': {ex}");
        }
    }

    // Runs on the main thread inside Universe.PrepareVehicleWorkers for every vehicle. See the
    // handle in GameReflection for why this is the site.
    private static void OnPrepareWorker(Vehicle __instance) => StepVehicle(__instance);

    private static void StepVehicle(Vehicle vehicle)
    {
        // The flag outlives a patch that failed to unpatch, and a kitten on EVA is a vehicle too.
        if (!SharedVehicleHooks.GuidanceEnabled || vehicle is KittenEva)
            return;

        try
        {
            GuidanceWindow.ApplyAutopilot(vehicle);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"guidance-step-{vehicle.Id}:{ex.GetType().Name}",
                $"[AFC] Guidance step failed on '{vehicle.Id}', releasing the craft: {ex}");
            try
            {
                GuidanceWindow.FailAutopilot(vehicle, ex);
            }
            catch (Exception release)
            {
                LogHelper.WarnOnce($"guidance-release-{vehicle.Id}:{release.GetType().Name}",
                    $"[AFC] Guidance could not release '{vehicle.Id}' after a failed step: {release}");
            }
        }
    }

    // Runs on the vehicle worker after stock builds the attitude target and before it reads the
    // target rate. Both gates match the gimbal writer, so the worker-side writers stop together,
    // and the catch keeps an escape from ending the worker's step for the whole physics bubble.
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

    // The menu carries the driver's only off switch, so a fault here is logged once per kind and
    // the menu keeps drawing. BeginMenu and EndMenu stay paired through the finally. The menu is
    // flight UI, so the editor's menu bar does not get it.
    private static void DrawMenu()
    {
        try
        {
            if (Program.IsEditorOpen || !ImGui.BeginMenu("AFC Guidance"u8))
                return;
            try
            {
                if (SharedVehicleHooks.GuidanceEnabled)
                {
                    bool active = GuidanceWindow.ModActive;
                    if (ImGui.MenuItem("Enabled", "", ref active, true))
                        GuidanceWindow.SetModActive(active);

                    // The panel is the only place a mode is started from, and it starts hidden,
                    // so this is how the player reaches it.
                    bool visible = GuidanceWindow.PanelVisible;
                    if (ImGui.MenuItem("Show panel", "", ref visible, active))
                        GuidanceWindow.PanelVisible = visible;
                }
                else
                    ImGui.Text(UnavailableReason);

                // Keep release errors visible while the guidance panel is hidden.
                string failure = GuidanceWindow.ReleaseFailure(Program.ControlledVehicle);
                if (failure.Length > 0)
                    ImGui.Text(failure);
            }
            finally
            {
                ImGui.EndMenu();
            }
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"guidance-menu:{ex.GetType().Name}",
                $"[AFC] Guidance menu failed: {ex}");
        }
    }

    private static Assembly? ResolveManagedLibrary(object? sender, ResolveEventArgs args)
    {
        string? name = new AssemblyName(args.Name).Name;
        if (name is not ("AdvancedFlightComputer.Guidance.Numerics"
            or "AdvancedFlightComputer.Guidance.Gfold"
            or "AdvancedFlightComputer.Guidance.Scvx"))
            return null;

        string? directory = Path.GetDirectoryName(typeof(GuidanceFeature).Assembly.Location);
        if (string.IsNullOrEmpty(directory))
            return null;

        string path = Path.Combine(directory, name + ".dll");
        return File.Exists(path) ? Assembly.LoadFrom(path) : null;
    }

    internal static void Reset()
    {
        DisableDriver();
        if (_resolverRegistered)
            AppDomain.CurrentDomain.AssemblyResolve -= ResolveManagedLibrary;
        _resolverRegistered = false;
    }
}

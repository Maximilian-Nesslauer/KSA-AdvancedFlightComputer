using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoRemove;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.HyperbolicTargets;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.RcsTranslation;
using AdvancedFlightComputer.Features.Ui.Console;
using Brutal.Logging;
using HarmonyLib;
using KSA;
using StarMap.API;

namespace AdvancedFlightComputer;

[StarMapMod]
public sealed class Mod
{
    private const string TestedGameVersion = "v2026.11.10.5801";

    private static readonly FeaturePatchSet _patches = new("com.maxi.advancedflightcomputer");

    // The AUTOSTAGE gauge button resolves its enum by name while the game reads Gauges.xml, which happens before any AllModsLoaded hook.
    // StarMap hooks [StarMapImmediateLoad] to Mod.PrepareSystems, which KSA 2026.11.10.5801 removed, so the enum goes in before Program.Main.
    // GaugeButtonFlightComputer's static constructor only builds the enum list, so touching it this early is safe.
    [StarMapBeforeMain]
    public void OnBeforeMain() => AutoStageFeature.InjectGaugeEnumAtLoad();

    [StarMapAllModsLoaded]
    public void OnFullyLoaded()
    {
        string gameVersion = VersionInfo.Current.VersionString;
        DefaultCategory.Log.Info($"[AFC] Game version: {gameVersion}");
        if (gameVersion != TestedGameVersion)
            DefaultCategory.Log.Warning(
                $"[AFC] Tested against {TestedGameVersion}, current is {gameVersion}. Some features may not work correctly.");

        // First, so every feature block can read its UI preferences. A broken file costs the preferences, not the load.
        try
        {
            UiConfig.Load();
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] UI preferences could not be loaded: {ex.Message}");
        }

        bool coreReady = Validated("Core", GameReflection.ValidateCore)
            && _patches.TryApply("Core", PatchCore);

        // Its own owner, so a settings-window rename costs the one AFC line in Settings > Mods and nothing else.
        if (Validated("SettingsPage", GameReflection.ValidateSettingsPage))
            _patches.TryApply("SettingsPage", ModSettingsPage.ApplyPatches);

        if (Validated("HyperbolicTargets", GameReflection.ValidateHyperbolicTargets))
            HyperbolicTargets.Enabled = _patches.TryApply("HyperbolicTargets", HyperbolicTargets.ApplyPatches);

        // Its own block, so a failed world-line postfix costs only the MAP lines of ORBIT and TRANSFER, and a failed ORBIT block leaves the TRANSFER lines alone.
        Patch_CelestialSystem_OnPreRender.Applied = _patches.TryApply("ConsoleWorldLines", PatchConsoleWorldLines);

        // The ORBIT tools draw in the console and need only the right-click anchor.
        bool orbitTools = _patches.TryApply("OrbitTools", ManeuverTools.ApplyPatches);
        if (orbitTools)
            OrbitToolsPanel.Enabled = true;
        else
            DisableOrbitTools();

        if (coreReady && Validated("MultiPass", GameReflection.ValidateMultiPass))
        {
            SharedVehicleHooks.MultiPassEnabled = _patches.TryApply("MultiPass", PatchMultiPass);
            if (!SharedVehicleHooks.MultiPassEnabled)
                DisableMultiPass();
        }

        if (coreReady && Validated("RcsTranslation", GameReflection.ValidateRcsTranslation))
        {
            SharedVehicleHooks.RcsEnabled = _patches.TryApply("RcsTranslation", PatchRcsTranslation);
            if (!SharedVehicleHooks.RcsEnabled)
                DisableRcsTranslation();
        }

        // Keep the menu available when the driver fails, because it holds the guidance off switch.
        bool guidanceMenu = Validated("GuidanceDiagnostics", GameReflection.ValidateGuidanceDiagnostics)
            && _patches.TryApply("GuidanceDiagnostics", GuidanceFeature.ApplyDiagnosticPatches);

        if (coreReady && guidanceMenu && Validated("Guidance", GameReflection.ValidateGuidance))
        {
            SharedVehicleHooks.GuidanceEnabled = _patches.TryApply("Guidance", GuidanceFeature.ApplyDriverPatches);
            if (!SharedVehicleHooks.GuidanceEnabled)
                GuidanceFeature.DisableDriver();
        }

        if (coreReady && Validated("AutoStage", GameReflection.ValidateAutoStage)
            && AutoStageFeature.StandaloneModAbsent())
        {
            SharedVehicleHooks.AutoStageEnabled = _patches.TryApply("AutoStage", AutoStageFeature.ApplyPatches);
            if (!SharedVehicleHooks.AutoStageEnabled)
            {
                AutoStageFeature.Disable();
                // The button bound its enum while the game read Gauges.xml, so it still draws.
                DefaultCategory.Log.Warning("[AFC] AutoStage is off, so the AUTOSTAGE gauge button does nothing this session.");
            }
        }

        if (coreReady && Validated("AutoRemove", GameReflection.ValidateAutoRemove))
        {
            AutoRemoveFeature.WarnIfStandaloneInstalled();
            SharedVehicleHooks.AutoRemoveEnabled = _patches.TryApply("AutoRemove", AutoRemoveFeature.ApplyPatches);
            if (!SharedVehicleHooks.AutoRemoveEnabled)
                AutoRemoveFeature.Disable();
        }

        // The planner key routing is soft: without the setter prefix the console's frame check routes the key alone.
        _patches.TryApply("PlannerRoute", PlannerRoute.ApplyPatch);

        if (guidanceMenu)
            ConsoleMenu.Register();
        ModSettingsPage.Register(ConsoleMenu.DrawModsSettingsLine);
        AfcAlerts.History = ConsoleMessages.RecordAlert;
        AfcConsole.LoadPreferences();
        AfcConsole.Enabled = true;
        PlannerRoute.MenuMissing = !guidanceMenu;
        if (PlannerRoute.MenuMissing)
            DefaultCategory.Log.Warning("[AFC] The AFC menu did not load, so the planner key opens the AFC console this session even when its switch is off.");
        PlannerRoute.Active = PlannerRoute.RoutingPreference || PlannerRoute.MenuMissing;

        DefaultCategory.Log.Info("[AFC] Loaded and patched.");
    }

    private static bool Validated(string feature, Func<bool> validate)
    {
        if (validate())
            return true;
        DefaultCategory.Log.Warning($"[AFC] {feature} disabled - reflection targets not found.");
        return false;
    }

    private static void PatchCore(Harmony harmony)
    {
        SharedVehicleHooks.ApplyPatches(harmony);
        SaveLoadObserver.ApplyPatches(harmony);

        // Keep the shared command sink installed when an individual feature fails to load.
        VehicleCommandSink.ApplyPatches(harmony);
    }

    // CelestialSystem.OnPreRender is public and runs every frame, so the world lines of ORBIT and TRANSFER draw without a patch on a planner member.
    private static void PatchConsoleWorldLines(Harmony harmony)
        => harmony.CreateClassProcessor(typeof(Patch_CelestialSystem_OnPreRender)).Patch();

    // A failed ORBIT tools block leaves the MANEUVER page saying so and hides the orbit right-click entries.
    private static void DisableOrbitTools()
    {
        BurnMenuLauncher.Enabled = false;
        OrbitToolsPanel.Enabled = false;
    }

    // The transfer passes and the flyby read the AFC transfer session, so they need no planner anchor.
    private static void PatchMultiPass(Harmony harmony)
    {
        MultiPassRegistry.Init();
        MultiPassUI.Enabled = true;
        HohmannMultiPassUI.Enabled = true;
        HohmannFlybyUI.Enabled = true;
    }

    // A mid-block failure can leave a flag set for a patch that never applied.
    // Every flag here belongs to that block, so clearing them wholesale is safe.
    private static void DisableMultiPass()
    {
        MultiPassUI.Enabled = false;
        HohmannMultiPassUI.Enabled = false;
        HohmannFlybyUI.Enabled = false;
    }

    private static void PatchRcsTranslation(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(RcsSetEnumPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsWarpPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsGaugePatches.IsDisabledPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsGaugePatches.PackDataPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsGaugePatches.HoveredPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsBurnWindowUi)).Patch();
        harmony.CreateClassProcessor(typeof(RcsBurnCanvasUi)).Patch();

        RcsExecRegistry.Init();
        SaveLoadObserver.SaveLoaded += RcsExecRegistry.Load;
        SaveLoadObserver.SaveWritten += OnRcsSaveWritten;
    }

    private static void DisableRcsTranslation()
    {
        SaveLoadObserver.SaveLoaded -= RcsExecRegistry.Load;
        SaveLoadObserver.SaveWritten -= OnRcsSaveWritten;
        RcsExecRegistry.Reset();
        RcsCommandChannel.Reset();
    }

    // The AFC console is its own ImGui window rather than a patch on a stock window, so it draws from the loader's hook after stock has drawn its viewports. This also runs the planner key frame check, before the stock planner draws in the same frame.
    [StarMapAfterGui]
    public void DrawGui(double dt)
    {
        try
        {
            AfcConsole.Draw(Program.MainViewport);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"afc-console-entry:{ex.GetType().Name}", $"[AFC] AFC console failed: {ex}");
        }
        UiConfig.Tick();
    }

    [StarMapUnload]
    public void Unload()
    {
        // Before the patches come off: a held staging row is already marked activated, so nothing else would fire it.
        StagingDetector.FlushPendingForUnload();
        // No frame between the unload steps may route the planner key or draw the console, and the transfer workers are cancelled.
        AfcConsole.Unload();
        SharedVehicleHooks.Reset();
        // Also removes the setter prefix, so the planner key is stock again.
        _patches.UnpatchAll();
        GuidanceFeature.Reset();
        AfcMenu.Reset();

        // Persistence is driven by UncompressedSave.Write, so a quit without saving drops in-memory registry mutations on purpose.
        SaveScopedState.ResetAll();
        // AfcConsole.Reset owns the console state of the guidance pages too, such as the tier scope and the page probe.
        AfcConsole.Reset();

        // Everything below resets only on unload, because the feature gates belong to the patch state, the load path reloads the registries from disk instead of clearing them, and the dedup sets live for the whole mod load.
        // LogHelper stays after UnpatchAll, or the transpiler re-runs that unpatching triggers print their once-only lines again.
        DisableMultiPass();
        BurnMenuLauncher.Enabled = false;
        OrbitToolsPanel.Enabled = false;
        Patch_CelestialSystem_OnPreRender.Applied = false;
        HyperbolicTargets.Enabled = false;
        RcsExecRegistry.Reset();
        MultiPassRegistry.Reset();
        AutoStageFeature.Disable();
        AutoStageFeature.RemoveGaugeEnum();
        AutoRemoveFeature.Disable();
        ModSettingsPage.Reset();
        SaveLoadObserver.Reset();
        HyperbolicTargets.ResetAlerts();
        UiConfig.Reset();
        LogHelper.Reset();
#if DEBUG
        PerfTracker.Reset();
#endif

        DefaultCategory.Log.Info("[AFC] Unloaded.");
    }

    private static void OnRcsSaveWritten(string oldSaveId, string newSaveId)
    {
        RcsExecRegistry.RekeyTo(oldSaveId, newSaveId);
        RcsExecRegistry.Save();
    }
}

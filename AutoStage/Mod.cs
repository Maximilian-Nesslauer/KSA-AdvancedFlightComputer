using AutoStage.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;
using StarMap.API;

namespace AutoStage;

[StarMapMod]
public sealed class Mod
{
    private static Harmony? _harmony;

    // Keep in sync with README.md.
    private const string TestedGameVersion = "v2026.9.7.5402";

    internal static bool AutoStageEnabled;
    internal static bool IgnitionDelayAvailable;

    private static bool _enumInjected;

    /// <summary>
    /// Injects our enum into GaugeButtonFlightComputer.EnumTypes before the
    /// game processes Gauges.xml, so EngineControlPatch.xml can resolve
    /// Action="AutoStageToggle". The game looks up the entry by Type.Name,
    /// which matches "AutoStageToggle".
    /// </summary>
    [StarMapImmediateLoad]
    public void OnImmediateLoad(KSA.Mod mod)
    {
        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug("[AutoStage] ImmediateLoad: injecting enum...");

        _enumInjected = InjectEnumType();
    }

    [StarMapAllModsLoaded]
    public void OnFullyLoaded()
    {
        string gameVersion = VersionInfo.Current.VersionString;
        DefaultCategory.Log.Info($"[AutoStage] Game version: {gameVersion}");
        if (gameVersion != TestedGameVersion)
            DefaultCategory.Log.Warning(
                $"[AutoStage] Tested against {TestedGameVersion}, current is {gameVersion}. " +
                "Some features may not work correctly.");

        Config.Init();

#if DEBUG
        PerfTracker.Reset();
#endif

        _harmony = new Harmony("com.maxi.autostage");

        bool coreOk = _enumInjected && GameReflection.ValidateAll();
        if (coreOk)
        {
            _harmony.CreateClassProcessor(typeof(Patch_ToggleEnum)).Patch();
            _harmony.CreateClassProcessor(typeof(Patch_IsSet)).Patch();
            _harmony.CreateClassProcessor(typeof(Patch_IsFlightComputerDisabled)).Patch();
            _harmony.CreateClassProcessor(typeof(Patch_SequenceList_ActivateNextSequence)).Patch();
            _harmony.CreateClassProcessor(typeof(Patch_SequenceList_ResetCaches)).Patch();
            _harmony.CreateClassProcessor(typeof(Patch_Universe_ApplyVehicleSolvers)).Patch();
            _harmony.CreateClassProcessor(typeof(Patch_Vehicle_Dispose)).Patch();

            // The settings window carries the only in-game switch for the
            // spent-stage drop, which uses none of the ignition-delay reflection
            // targets, so it must not go down with them. The delay tables inside
            // it hide themselves via Mod.IgnitionDelayAvailable.
            _harmony.CreateClassProcessor(typeof(SettingsTabPatch)).Patch();

            if (DebugConfig.AutoStage)
                DefaultCategory.Log.Debug("[AutoStage] Core patches applied.");
        }
        else
        {
            DefaultCategory.Log.Warning("[AutoStage] Disabled, reflection targets not found.");
        }

        if (coreOk && GameReflection.ValidateIgnitionDelay())
        {
            IgnitionDelayAvailable = true;
            _harmony.CreateClassProcessor(typeof(PartWindowPatch)).Patch();

            if (DebugConfig.IgnitionDelay)
                DefaultCategory.Log.Debug("[AutoStage] IgnitionDelay patches applied.");
        }
        else if (coreOk)
        {
            IgnitionDelayAvailable = false;
            DefaultCategory.Log.Warning(
                "[AutoStage] IgnitionDelay disabled, reflection targets not found.");
        }

        DefaultCategory.Log.Info("[AutoStage] Loaded.");
    }

    [StarMapUnload]
    public void Unload()
    {
        // Before the patches come off. Reset below drops the pending slot
        // unfired, stranding an already-activated row.
        StagingDetector.FlushPendingForUnload();
        _harmony?.UnpatchAll(_harmony.Id);
        _harmony = null;
        AutoStageEnabled = false;
        IgnitionDelayAvailable = false;
        StagingDetector.Reset();
        StagingHelpers.Reset();
        SettingsTabPatch.Reset();
        Config.Reset();
        LogHelper.Reset();
        if (_enumInjected)
        {
            RemoveEnumType();
            _enumInjected = false;
        }
#if DEBUG
        PerfTracker.Reset();
#endif
        DefaultCategory.Log.Info("[AutoStage] Unloaded.");
    }

    private static bool InjectEnumType()
    {
        if (!TryGetEnumTypes(out var list))
            return false;

        // Guard against duplicate entries on reload, since GaugeButtonFlightComputer
        // matches by Type.Name and would happily pick the first hit.
        if (list.Any(opt => opt.Type == typeof(AutoStageToggle)))
        {
            if (DebugConfig.AutoStage)
                DefaultCategory.Log.Debug(
                    $"[AutoStage] AutoStageToggle already present in EnumTypes ({list.Count} entries).");
            return true;
        }

        list.Add(new EnumTypeOption(typeof(AutoStageToggle)));

        if (DebugConfig.AutoStage)
            DefaultCategory.Log.Debug(
                $"[AutoStage] Appended AutoStageToggle to EnumTypes ({list.Count} entries).");
        return true;
    }

    private static void RemoveEnumType()
    {
        if (!TryGetEnumTypes(out var list))
            return;

        list.RemoveAll(opt => opt.Type == typeof(AutoStageToggle));
    }

    private static bool TryGetEnumTypes(out List<EnumTypeOption> list)
    {
        list = null!;
        if (GameReflection.GaugeButton_EnumTypes == null)
        {
            DefaultCategory.Log.Error(
                "[AutoStage] GaugeButtonFlightComputer.EnumTypes not found.");
            return false;
        }
        if (GameReflection.GaugeButton_EnumTypes.GetValue(null) is not List<EnumTypeOption> found)
        {
            DefaultCategory.Log.Error("[AutoStage] EnumTypes is null or unexpected type.");
            return false;
        }
        list = found;
        return true;
    }
}

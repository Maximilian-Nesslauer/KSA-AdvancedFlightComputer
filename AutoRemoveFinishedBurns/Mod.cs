using AutoRemoveFinishedBurns.Core;
using AutoRemoveFinishedBurns.Features;
using Brutal.Logging;
using HarmonyLib;
using KSA;
using StarMap.API;

namespace AutoRemoveFinishedBurns;

[StarMapMod]
public sealed class Mod
{
    private static Harmony? _harmony;

    // Keep in sync with README.md.
    private const string TestedGameVersion = "v2026.8.22.5348";

    [StarMapAllModsLoaded]
    public void OnFullyLoaded()
    {
        string gameVersion = VersionInfo.Current.VersionString;
        DefaultCategory.Log.Info($"[AutoRemoveFinishedBurns] Game version: {gameVersion}");
        if (gameVersion != TestedGameVersion)
            DefaultCategory.Log.Warning(
                $"[AutoRemoveFinishedBurns] Tested against {TestedGameVersion}, current is {gameVersion}. " +
                "Some features may not work correctly.");

        if (DebugConfig.Any)
            DefaultCategory.Log.Debug(
                $"[AutoRemoveFinishedBurns] Debug flags: Detection={DebugConfig.Detection}, " +
                $"Settings={DebugConfig.Settings}, Performance={DebugConfig.Performance}");

        Config.Init();

#if DEBUG
        // Re-anchor the report-interval clock so the first sample is a full
        // ReportIntervalSeconds wide, not however long elapsed since type init.
        PerfTracker.Reset();
#endif

        _harmony = new Harmony("com.maxi.autoremovefinishedburns");

        if (GameReflection.ValidateDetection())
        {
            _harmony.CreateClassProcessor(typeof(BurnRemovalPatch)).Patch();
            if (DebugConfig.Detection)
                DefaultCategory.Log.Debug(
                    "[AutoRemoveFinishedBurns] Detection patch applied.");
        }
        else
        {
            DefaultCategory.Log.Warning(
                "[AutoRemoveFinishedBurns] Detection disabled - reflection targets not found.");
        }

        if (GameReflection.ValidateSettings())
        {
            _harmony.CreateClassProcessor(typeof(SettingsTabPatch)).Patch();
            if (DebugConfig.Settings)
                DefaultCategory.Log.Debug(
                    "[AutoRemoveFinishedBurns] Settings tab patch applied.");
        }
        else
        {
            DefaultCategory.Log.Warning(
                "[AutoRemoveFinishedBurns] Settings tab disabled - reflection targets not found.");
        }

        // TryEnable reports why it declined; a failure is not an error here
        // because the AFC dependency is optional.
        if (AfcRcsInterop.TryEnable())
            DefaultCategory.Log.Info(
                "[AutoRemoveFinishedBurns] AdvancedFlightComputer RCS interop active.");

        DefaultCategory.Log.Info("[AutoRemoveFinishedBurns] Loaded.");
    }

    [StarMapUnload]
    public void Unload()
    {
        _harmony?.UnpatchAll(_harmony.Id);
        _harmony = null;
        AfcRcsInterop.Disable();
        BurnRemovalPatch.Reset();
        Config.Reset();
        LogHelper.Reset();
#if DEBUG
        PerfTracker.Reset();
#endif
        DefaultCategory.Log.Info("[AutoRemoveFinishedBurns] Unloaded.");
    }
}

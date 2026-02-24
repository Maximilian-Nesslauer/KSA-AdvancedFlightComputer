using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features;
using AdvancedFlightComputer.Features.HyperbolicTargets;
using AdvancedFlightComputer.Features.StageInfo;
using Brutal.Logging;
using HarmonyLib;
using KSA;
using StarMap.API;

namespace AdvancedFlightComputer;

[StarMapMod]
public class Mod
{
    private static Harmony? _harmony;

    private const string TestedGameVersion = "v2026.2.35.3667";

#if DEBUG
    public static bool DebugMode = true;
#else
    public static bool DebugMode = false;
#endif

    /// <summary>
    /// Runs during Mod.PrepareSystems(), BEFORE the game processes Gauges.xml.
    /// Injects our custom enum into the gauge button lookup so that
    /// BurnControlPatch.xml can resolve Action="AfcAutoStage" during OnDataLoad.
    /// </summary>
    [StarMapImmediateLoad]
    public void OnImmediateLoad(KSA.Mod mod)
    {
        if (DebugMode)
            DefaultCategory.Log.Debug("[AFC] ImmediateLoad: ensuring enum injection...");

        AutoStage.InjectEnumLookup();
    }

    [StarMapAllModsLoaded]
    public void OnFullyLoaded()
    {
        string gameVersion = VersionInfo.Current.VersionString;
        DefaultCategory.Log.Info($"[AFC] Game version: {gameVersion}");
        if (gameVersion != TestedGameVersion)
            DefaultCategory.Log.Warning(
                $"[AFC] Tested against {TestedGameVersion}, current is {gameVersion}. " +
                "Some features may not work correctly.");

        _harmony = new Harmony("com.maxi.advancedflightcomputer");

        if (GameReflection.ValidateHyperbolicTargets())
            HyperbolicTargets.ApplyPatches(_harmony);
        else
            DefaultCategory.Log.Warning("[AFC] HyperbolicTargets disabled - reflection targets not found.");

        if (GameReflection.ValidateAutoStage())
            AutoStage.ApplyPatches(_harmony);
        else
            DefaultCategory.Log.Warning("[AFC] AutoStage disabled - reflection targets not found.");

        if (GameReflection.ValidateStageInfo())
            BetterBurnTime.ApplyPatches(_harmony);
        else
            DefaultCategory.Log.Warning("[AFC] StageInfo disabled - reflection targets not found.");

        DefaultCategory.Log.Info("[AFC] Loaded and patched.");
    }

    [StarMapUnload]
    public void Unload()
    {
        _harmony?.UnpatchAll(_harmony.Id);
        _harmony = null;
        AutoStage.Enabled = false;
        Patch_AutoStageExecution.Reset();
        StageAnalyzerDebug.Reset();
        BetterBurnTime.Reset();
        LogHelper.Reset();
        DefaultCategory.Log.Info("[AFC] Unloaded.");
    }
}

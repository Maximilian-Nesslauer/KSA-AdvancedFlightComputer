using AdvancedFlightComputer.Features;
using AdvancedFlightComputer.Features.StageInfo;
using Brutal.Logging;
using HarmonyLib;
using StarMap.API;

namespace AdvancedFlightComputer;

[StarMapMod]
public class Mod
{
    private static Harmony? _harmony;

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
        if (!HyperbolicTargets.ValidateReflectionTargets())
        {
            DefaultCategory.Log.Error("[AFC] Skipping patches - reflection targets missing (game version changed?).");
            return;
        }

        _harmony = new Harmony("com.maxi.advancedflightcomputer");
        _harmony.PatchAll(typeof(Mod).Assembly);
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
        DefaultCategory.Log.Info("[AFC] Unloaded.");
    }
}

using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.HyperbolicTargets;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.Logging;
using HarmonyLib;
using KSA;
using StarMap.API;

namespace AdvancedFlightComputer;

[StarMapMod]
public class Mod
{
    private static Harmony? _harmony;

    private const string TestedGameVersion = "v2026.4.16.4170";

    public static bool DebugMode => DebugConfig.Any;

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

        if (GameReflection.ValidateManeuverTools())
        {
            ManeuverTools.InjectTransferTypes();
            ManeuverTools.ApplyPatches(_harmony);
        }
        else
            DefaultCategory.Log.Warning("[AFC] ManeuverTools disabled - reflection targets not found.");

        DefaultCategory.Log.Info("[AFC] Loaded and patched.");
    }

    [StarMapUnload]
    public void Unload()
    {
        _harmony?.UnpatchAll(_harmony.Id);
        _harmony = null;
        ManeuverToolsWindow.Reset();
        Patch_DrawPlanWindow.Reset();
        LogHelper.Reset();
#if DEBUG
        PerfTracker.Reset();
#endif
        DefaultCategory.Log.Info("[AFC] Unloaded.");
    }
}

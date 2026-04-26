using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.HyperbolicTargets;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.Logging;
using HarmonyLib;
using KSA;
using StarMap.API;

namespace AdvancedFlightComputer;

[StarMapMod]
public sealed class Mod
{
    private static Harmony? _harmony;
    private static bool _maneuverTypesInjected;

    private const string TestedGameVersion = "v2026.4.17.4184";

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
            _maneuverTypesInjected = true;
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

        if (_maneuverTypesInjected)
        {
            ManeuverTools.RemoveTransferTypes();
            _maneuverTypesInjected = false;
        }

        ManeuverToolsWindow.Reset();
        Patch_DrawPlanWindow.Reset();
        LogHelper.Reset();
#if DEBUG
        PerfTracker.Reset();
#endif
        DefaultCategory.Log.Info("[AFC] Unloaded.");
    }
}

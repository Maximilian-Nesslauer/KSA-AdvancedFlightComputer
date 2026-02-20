using AdvancedFlightComputer.Features;
using Brutal.Logging;
using HarmonyLib;
using StarMap.API;

namespace AdvancedFlightComputer;

[StarMapMod]
public class Mod
{
    private static Harmony? _harmony;

    public static bool DebugMode = true;

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
        DefaultCategory.Log.Info("[AFC] Unloaded.");
    }
}

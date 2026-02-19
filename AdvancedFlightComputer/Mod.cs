using HarmonyLib;
using StarMap.API;

namespace AdvancedFlightComputer;

[StarMapMod]
public class Mod
{
    private static Harmony? _harmony;

    [StarMapAllModsLoaded]
    public void OnFullyLoaded()
    {
        _harmony = new Harmony("com.maxi.advancedflightcomputer");
        _harmony.PatchAll(typeof(Mod).Assembly);
        Console.WriteLine("[AdvancedFlightComputer] Loaded and patched.");
    }

    [StarMapUnload]
    public void Unload()
    {
        _harmony?.UnpatchAll(_harmony.Id);
        _harmony = null;
        Console.WriteLine("[AdvancedFlightComputer] Unloaded.");
    }

    [StarMapAfterGui]
    public void OnAfterGui(double dt)
    {
        // Future: settings window
    }
}

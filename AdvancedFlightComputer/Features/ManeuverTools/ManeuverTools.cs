using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

internal static class ManeuverTools
{
    // The game-free OrbitToolCatalog defines the keys, and the mod code names them here. The quick tools keep the keys they had as plan types in the transfer planner, so the multi-pass intents and multipass.toml keep reading the same strings.
    internal const string KeySetPeriapsis = OrbitToolCatalog.KeySetPeriapsis;
    internal const string KeySetApoapsis = OrbitToolCatalog.KeySetApoapsis;
    internal const string KeyMatchInclination = OrbitToolCatalog.KeyMatchInclination;
    internal const string KeySetInclination = OrbitToolCatalog.KeySetInclination;
    internal const string KeyCircularizeApoapsis = OrbitToolCatalog.KeyCircularizeApoapsis;
    internal const string KeyCircularizePeriapsis = OrbitToolCatalog.KeyCircularizePeriapsis;

    internal const string KeyStockHohmann = "Hohmann";

    public static void ApplyPatches(Harmony harmony)
    {
        // If the shortcut patch fails, the quick tools stay available on the MANEUVER page of the console.
        if (Patch_BurnContextMenu_Launcher.IsAnchorPresent)
        {
            try
            {
                harmony.CreateClassProcessor(typeof(Patch_BurnContextMenu_Launcher)).Patch();
                BurnMenuLauncher.Enabled = true;
            }
            catch (Exception ex)
            {
                BurnMenuLauncher.Enabled = false;
                DefaultCategory.Log.Warning(
                    $"[AFC] Burn context-menu shortcuts disabled - patching BurnContextMenu.Draw failed: {ex}");
            }
        }
        else
            DefaultCategory.Log.Warning(
                "[AFC] Burn context-menu shortcuts disabled - BurnContextMenu.Draw not found.");

        if (DebugConfig.ManeuverTools)
            DefaultCategory.Log.Debug("[AFC] ManeuverTools: all patches applied.");
    }
}

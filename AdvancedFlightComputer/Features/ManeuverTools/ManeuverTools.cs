using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

internal static class ManeuverTools
{
    internal const string KeySetPeriapsis = "AFC Set Periapsis";
    internal const string KeySetApoapsis = "AFC Set Apoapsis";
    internal const string KeyMatchInclination = "AFC Match Inclination";
    internal const string KeySetInclination = "AFC Set Inclination";

    // These stock keys are handled by AFC but must remain in the dropdown on unload.
    internal const string KeyStockCircularizeApoapsis = "Circularize Apoapsis";
    internal const string KeyStockCircularizePeriapsis = "Circularize Periapsis";

    internal const string KeyStockHohmann = "Hohmann";

    public static void InjectTransferTypes()
    {
        var types = TransferPlanner.TransferTypes;

        if (types.Exists(t => t.GetKey() == KeySetPeriapsis))
            return;

        types.Add(new TransferType(KeySetPeriapsis, "Set Periapsis"));
        types.Add(new TransferType(KeySetApoapsis, "Set Apoapsis"));
        types.Add(new TransferType(KeyMatchInclination, "Match Inclination"));
        types.Add(new TransferType(KeySetInclination, "Set Inclination"));

        if (DebugConfig.ManeuverTools)
            DefaultCategory.Log.Debug(
                $"[AFC] ManeuverTools: injected 4 transfer types ({types.Count} total).");
    }

    public static void RemoveTransferTypes()
    {
        var types = TransferPlanner.TransferTypes;
        types.RemoveAll(t => IsOurType(t.GetKey()));

        // Select a stock type on unload because stock cannot draw the window for a removed AFC type.
        if (StockPlanner.TransferTypeKey is string key && IsOurType(key) && types.Count > 0)
            StockPlanner.TransferType = types[0];
    }

    internal static bool IsOurType(string key)
    {
        return key == KeySetPeriapsis
            || key == KeySetApoapsis
            || key == KeyMatchInclination
            || key == KeySetInclination;
    }

    internal static bool IsCircularizeType(string key)
    {
        return key == KeyStockCircularizeApoapsis
            || key == KeyStockCircularizePeriapsis;
    }

    internal static bool IsHandledType(string key)
    {
        return IsOurType(key) || IsCircularizeType(key);
    }

    public static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(Patch_DrawPlanWindow)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_OnPreRender)).Patch();

        // If the shortcut patch fails, the quick tools must remain available in the planner.
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

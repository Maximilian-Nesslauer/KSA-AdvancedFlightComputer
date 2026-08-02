using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// Adds maneuver quick-tools (Set Periapsis, Set Apoapsis, Match Inclination,
/// Set Inclination) to the stock Transfer Planner dropdown. Each tool computes
/// a single burn via vis-viva or plane-change math and creates a maneuver node.
/// </summary>
internal static class ManeuverTools
{
    internal const string KeySetPeriapsis = "AFC Set Periapsis";
    internal const string KeySetApoapsis = "AFC Set Apoapsis";
    internal const string KeyMatchInclination = "AFC Match Inclination";
    internal const string KeySetInclination = "AFC Set Inclination";

    // Stock-injected keys we claim via Patch_DrawPlanWindow so we can wrap them
    // in our multi-pass pipeline. Match TransferPlanner.TransferTypes verbatim.
    internal const string KeyStockCircularizeApoapsis = "Circularize Apoapsis";
    internal const string KeyStockCircularizePeriapsis = "Circularize Periapsis";

    // Stock Hohmann key. AFC does not take over its plan window; the
    // constant exists so Hohmann-related guards (HohmannMultiPassUI,
    // Patch_TransferPlanner_*, HohmannTransferIntent.TypeKey) share one
    // string instead of duplicating the literal.
    internal const string KeyStockHohmann = "Hohmann";

    /// <summary>
    /// Adds our plan types to the stock TransferPlanner dropdown.
    /// Called from Mod.OnFullyLoaded before patches are applied.
    /// Idempotent, checks for existing entries before adding.
    /// </summary>
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

    /// <summary>
    /// Removes our plan types from the stock dropdown so the planner UI
    /// returns to a clean state on mod unload.
    /// </summary>
    public static void RemoveTransferTypes()
    {
        var types = TransferPlanner.TransferTypes;
        types.RemoveAll(t => IsOurType(t.GetKey()));

        // Stock keeps the selected plan type in its own static and its window
        // body only has branches for stock keys, so a selection left pointing
        // at a removed AFC type would reopen to a window drawing nothing but
        // the dropdown. Writing only the type is enough because every path
        // that selects an AFC type also clears _transferCalculated, so stock
        // re-enters its own type with the flag down and rebuilds TransferInfo
        // itself.
        if (StockPlanner.TransferTypeKey is string key && IsOurType(key) && types.Count > 0)
            StockPlanner.TransferType = types[0];
    }

    /// <summary>
    /// Returns true if the given transfer type key is one we injected. Used by
    /// <see cref="RemoveTransferTypes"/> to scope removal to AFC entries; stock
    /// keys we claim (see <see cref="IsCircularizeType"/>) must not be removed.
    /// </summary>
    internal static bool IsOurType(string key)
    {
        return key == KeySetPeriapsis
            || key == KeySetApoapsis
            || key == KeyMatchInclination
            || key == KeySetInclination;
    }

    /// <summary>Stock circularize keys whose plan-window UI AFC takes over.</summary>
    internal static bool IsCircularizeType(string key)
    {
        return key == KeyStockCircularizeApoapsis
            || key == KeyStockCircularizePeriapsis;
    }

    /// <summary>
    /// True when AFC owns the Transfer Planning window for this plan type:
    /// the four AFC-injected quick-tools plus the two stock circularize
    /// entries we claim via Patch_DrawPlanWindow. The takeover swaps in our
    /// window layout for everyone, single-burn included; multi-pass is one
    /// extra capability that rides on top.
    /// </summary>
    internal static bool IsHandledType(string key)
    {
        return IsOurType(key) || IsCircularizeType(key);
    }

    /// <summary>
    /// Applies all ManeuverTools Harmony patches. Called from Mod.cs
    /// after GameReflection.ValidateManeuverTools() passes.
    /// </summary>
    public static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(Patch_DrawPlanWindow)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_OnPreRender)).Patch();

        // The shortcuts only link to the window the prefix above draws, so they are
        // gated on that prefix having applied. Their own failure is contained here
        // rather than left to the caller's TryPatchBlock: this block owns the four
        // quick-tools, and a block-level failure strips them from stock's dropdown.
        // A convenience shortcut must not be able to cost that, and the gate alone
        // cannot cover every shape change BurnContextMenu.Draw could take.
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

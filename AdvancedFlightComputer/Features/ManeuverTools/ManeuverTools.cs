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
    }

    /// <summary>
    /// Returns true if the given transfer type key is one of ours.
    /// </summary>
    internal static bool IsOurType(string key)
    {
        return key == KeySetPeriapsis
            || key == KeySetApoapsis
            || key == KeyMatchInclination
            || key == KeySetInclination;
    }

    /// <summary>
    /// Applies all ManeuverTools Harmony patches. Called from Mod.cs
    /// after GameReflection.ValidateManeuverTools() passes.
    /// </summary>
    public static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(Patch_DrawPlanWindow)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_OnPreRender)).Patch();

        if (DebugConfig.ManeuverTools)
            DefaultCategory.Log.Debug("[AFC] ManeuverTools: all patches applied.");
    }
}

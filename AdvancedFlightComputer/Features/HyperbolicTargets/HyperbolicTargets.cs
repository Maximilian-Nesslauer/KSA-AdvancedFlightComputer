using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.HyperbolicTargets;

/// <summary>
/// Enables the Transfer Planner to target objects on hyperbolic orbits
/// (Oumuamua, 2I/Borisov, 3I/ATLAS, etc.).
///
/// The Lambert solver handles any orbit geometry, but surrounding code
/// assumes targets have a finite Period, positive SMA, and meaningful SOI.
/// HyperbolicBodies.xml injects Mass and SOI via KittenExtensions. The
/// Harmony patches here fix everything else: target filtering, time-of-flight
/// estimation, departure alignment, and encounter detection.
/// </summary>
internal static class HyperbolicTargets
{
    /// <summary>Min transfer ToF as fraction of Hohmann estimate.</summary>
    internal const double MinTofRatio = 0.3;

    /// <summary>Max transfer ToF as fraction of Hohmann estimate.</summary>
    internal const double MaxTofRatio = 4.0;

    /// <summary>
    /// Applies all HyperbolicTargets Harmony patches. Called from Mod.cs
    /// after GameReflection.ValidateHyperbolicTargets() passes.
    /// </summary>
    public static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(Patch_PopulateWithPlanets)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_HohmannFlight)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_SetTransferInfo)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_AlignmentTime)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_TryFindIntercept)).Patch();

        // The XML patch gives the comets an SOI whether or not this guard applies,
        // and without the guard stock's encounter scan throws on them, so a missing
        // anchor is an error rather than a quiet degrade.
        if (Patch_FindClosestApproaches.IsAnchorPresent)
            harmony.CreateClassProcessor(typeof(Patch_FindClosestApproaches)).Patch();
        else
            DefaultCategory.Log.Error(
                "[AFC] HyperbolicTargets: PatchedConic.FindClosestApproaches not found. " +
                "Stock's encounter search may throw on a comet that has an SOI; " +
                "re-verify this feature before flying with HyperbolicBodies.xml enabled.");

        if (DebugConfig.HyperbolicTargets)
            DefaultCategory.Log.Debug("[AFC] HyperbolicTargets: all patches applied.");
    }

    /// <summary>
    /// Walks up the parent chain to find the star the vehicle orbits.
    /// Returns null if no star is found (shouldn't happen in normal gameplay).
    /// </summary>
    internal static StellarBody? GetParentStar(Vehicle source)
    {
        IParentBody? current = source.Parent;
        while (current != null)
        {
            if (current is StellarBody star)
                return star;
            current = (current as Celestial)?.Parent;
        }
        LogHelper.WarnOnce($"no-parent-star-{source.Id}",
            $"[AFC] No parent star found for vehicle {source.Id}; HyperbolicTargets disabled for this vehicle.");
        return null;
    }
}

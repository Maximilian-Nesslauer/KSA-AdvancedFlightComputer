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
/// estimation, departure alignment, encounter detection, and orbit rendering.
/// </summary>
static class HyperbolicTargets
{
    /// <summary>
    /// Margin from the exact asymptote angle (acos(-1/e)) to avoid
    /// non-finite positions at the mathematical limit.
    /// </summary>
    internal const double AsymptoteMargin = 0.999;

    /// <summary>Number of points generated for orbit rendering lines.</summary>
    internal const int OrbitPointCount = 2000;

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
        harmony.CreateClassProcessor(typeof(Patch_GetPlanetList)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_HohmannFlight)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_SetTransferInfo)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_AlignmentTime)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_TryFindIntercept)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_ClipPointGeneration)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_DiagnosticLog)).Patch();

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
        return null;
    }
}

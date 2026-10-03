using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.HyperbolicTargets;

/// <summary>
/// Lets the Transfer Planner target bodies on unbound orbits, which are the
/// interstellar comets. The Lambert solver handles any geometry, but the code
/// around it assumes a finite Period, a positive semi major axis and a usable
/// SOI. HyperbolicBodies.xml supplies mass and SOI through KittenExtensions.
/// The patches here cover the rest, namely the target listing, the time of
/// flight estimate, the departure alignment, the encounter detection, and the
/// stock closest approach search, which throws on an unbound body.
/// </summary>
internal static class HyperbolicTargets
{
    /// <summary>Transfer window as fractions of the Hohmann estimate. Stock sizes
    /// the window from the target's Period, which is NaN on an unbound orbit.</summary>
    internal const double MinTofRatio = 0.3;
    internal const double MaxTofRatio = 4.0;

    public static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(Patch_PopulateWithPlanets)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_HohmannFlight)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_SetTransferInfo)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_AlignmentTime)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_TryFindIntercept)).Patch();

        // The XML patch gives the comets an SOI whether or not this guard applies, and without the guard stock's encounter scan throws on them, so a missing anchor is an error rather than a quiet degrade.
        if (Patch_FindClosestApproaches.IsAnchorPresent)
            harmony.CreateClassProcessor(typeof(Patch_FindClosestApproaches)).Patch();
        else
            DefaultCategory.Log.Error(
                "[AFC] HyperbolicTargets: PatchedConic.FindClosestApproaches not found. " +
                "Stock's encounter search may throw on a comet that has an SOI; " +
                "re-verify this feature before flying with HyperbolicBodies.xml enabled.");
    }

    /// <summary>The body a transfer from <paramref name="source"/> is solved around.
    /// <c>TransferPlanner.SetTransferInfo</c> departs from the source's parent when
    /// that parent orbits something, so the Lambert solve is about the parent's
    /// parent, and otherwise from the vehicle itself about its own parent.</summary>
    internal static IParentBody? TransferFrame(Vehicle source)
        => (source.Parent as IOrbiter)?.Parent ?? source.Parent;
}

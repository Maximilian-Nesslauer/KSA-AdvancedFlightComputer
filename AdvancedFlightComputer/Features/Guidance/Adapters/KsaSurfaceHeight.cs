#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using KSA;
using KSA.Rendering.Water.Data;

/// <summary>
/// The height of what a craft comes down on, above the body's mean-radius sphere: the terrain, or the sea where the sea covers it.
///
/// THE SEA IS A SURFACE TOO. KSA's terrain height under an ocean is the seabed. A site out at sea measured on it put the landing hundreds of metres under the water, so a booster flew its landing burn into the sea and arrived at the surface still falling. The game floats a craft at the ocean's own radius, MeanRadius + OceanReference.Level (PhysicsEnvironment.ComputeOceanRadius), so that is the surface here wherever it is higher than the terrain. That is what lets a booster set down softly on the water, and later on a drone ship.
///
/// THE STILL SEA, not the waves. The game's per-position wave height comes from the ocean renderer, which a headless run and the planners' worker threads do not have, and which moves with time where a site has to stay put. The level is the mean the waves move about.
/// </summary>
internal static class KsaSurfaceHeight
{
    /// <summary>The surface height in the body-fixed direction <paramref name="dirCcf"/>, m above the mean radius. Zero for a body with no terrain to sample.</summary>
    internal static double At(IParentBody parent, double3 dirCcf)
    {
        double terrain = (parent as Celestial)?.GetTerrainHeightFromDirCcf(dirCcf) ?? 0.0;
        if (!double.IsFinite(terrain))
            terrain = 0.0;
        return Math.Max(terrain, SeaLevel(parent));
    }

    /// <summary>The sea's height above the mean radius, m, or negative infinity for a body with no ocean.</summary>
    internal static double SeaLevel(IParentBody parent)
    {
        OceanReference ocean = parent?.GetOceanReference();
        if (ocean == null)
            return double.NegativeInfinity;
        double level = ocean.Level;
        return double.IsFinite(level) ? level : double.NegativeInfinity;
    }

    /// <summary>True when the sea, not the ground, is the surface in direction <paramref name="dirCcf"/>.</summary>
    internal static bool IsWater(IParentBody parent, double3 dirCcf)
    {
        double sea = SeaLevel(parent);
        if (!double.IsFinite(sea))
            return false;
        double terrain = (parent as Celestial)?.GetTerrainHeightFromDirCcf(dirCcf) ?? double.NaN;
        return double.IsFinite(terrain) && terrain < sea;
    }
}

using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>Draws multi-pass preview orbit lines. Earlier passes
/// are dimmer; the final pass keeps full BurnPatchColor.</summary>
internal static class MultiPassRenderer
{
    /// <summary>When <paramref name="skipFirst"/> is true, passes[0] is
    /// omitted - used during active execution where stock already
    /// renders the queued burn's post-burn orbit. When <paramref name="skipLast"/>
    /// is true, the last pass is omitted - used for Hohmann where stock
    /// already renders the selected-entry's flight plan (= final pass
    /// trajectory). The dim ramp keeps the unrendered last pass's slot
    /// so intermediate brightness still slopes correctly.</summary>
    public static void RenderPassOrbits(
        IViewport viewport, Vehicle source, PassPreview[] passes,
        bool skipFirst = false, bool skipLast = false)
    {
        int start = skipFirst ? 1 : 0;
        int end = passes.Length - (skipLast ? 1 : 0);
        if (end - start <= 0) return;

        // rampCount includes the skipped-last slot so intermediate passes
        // get the same brightness they would in the full ramp; without
        // this the second-to-last pass would jump to full BurnPatchColor.
        int rampCount = passes.Length - start;

        for (int i = start; i < end; i++)
        {
            FlightPlan fp = passes[i].FlightPlan;
            if (fp.Patches.Count == 0)
                continue;

            ApplyPassColor(fp, i - start, rampCount);
            EnsurePatchPointsCached(fp);

            // isActive=true matches stock's selected-porkchop rendering;
            // without it the lines look ghosted. Ground-track danger markers go on
            // the final trajectory only, as stock does for its own burn plan; the
            // index is unreachable while skipLast holds, where stock renders the
            // final pass itself and every pass shown here is an intermediate one.
            fp.AddLineInstances(viewport, source, isActive: true,
                drawVehiclePosition: false,
                TrueAnomaly.NaN, TrueAnomaly.NaN,
                drawDangerGroundTrack: i == passes.Length - 1,
                isPostBurnOrbit: true);
        }
    }

    // 40-100% brightness ramp; final shown pass at full BurnPatchColor.
    // Skip Darken at the final pass: it is NOT identity at factor=1.0
    // (HSL roundtrip with sat/lightness floor at 0.1).
    private static void ApplyPassColor(FlightPlan fp, int shownIndex, int shownCount)
    {
        byte4 color = BurnPlan.BurnPatchColor;
        if (shownCount > 1 && shownIndex < shownCount - 1)
        {
            float brightness = 0.4f + 0.6f * shownIndex / (shownCount - 1);
            color = color.Darken(brightness);
        }
        foreach (PatchedConic patch in fp.Patches)
            patch.Orbit.OrbitLineColor = color;
    }

    // Freshly built patches have no cached points; AddLineInstances
    // would draw nothing without this.
    private static void EnsurePatchPointsCached(FlightPlan fp)
    {
        foreach (PatchedConic patch in fp.Patches)
        {
            patch.HidePatch = false;
            if (patch.Orbit.IsMissingPoints())
                patch.Orbit.UpdateCachedPoints(UpdateTaskUtils.GenerateSpacedPoints(patch));
        }
    }
}

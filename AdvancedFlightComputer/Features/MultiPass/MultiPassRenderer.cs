using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>Draws multi-pass preview orbit lines. Earlier passes
/// are dimmer; the final pass keeps full BurnPatchColor.</summary>
internal static class MultiPassRenderer
{
    /// <summary>When <paramref name="skipFirst"/> is true, passes[0] is
    /// omitted - used during active execution where stock already
    /// renders the queued burn's post-burn orbit.</summary>
    public static void RenderPassOrbits(
        Viewport viewport, Vehicle source, PassPreview[] passes, bool skipFirst = false)
    {
        int start = skipFirst ? 1 : 0;
        int shown = passes.Length - start;
        if (shown <= 0) return;

        for (int i = start; i < passes.Length; i++)
        {
            FlightPlan fp = passes[i].FlightPlan;
            if (fp.Patches.Count == 0)
                continue;

            ApplyPassColor(fp, i - start, shown);
            EnsurePatchPointsCached(fp);

            // isActive=true matches stock's selected-porkchop rendering;
            // without it the lines look ghosted.
            fp.AddLineInstances(viewport, source, isActive: true,
                drawVehiclePosition: false,
                TrueAnomaly.NaN, TrueAnomaly.NaN);
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

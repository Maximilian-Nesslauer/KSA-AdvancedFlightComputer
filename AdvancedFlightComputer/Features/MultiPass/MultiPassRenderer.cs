using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>Draws multi-pass preview orbit lines. Earlier passes
/// are dimmer; the final pass keeps full BurnPatchColor.</summary>
internal static class MultiPassRenderer
{
    public static void RenderPassOrbits(
        Viewport viewport, Vehicle source, PassPreview[] passes)
    {
        for (int i = 0; i < passes.Length; i++)
        {
            FlightPlan fp = passes[i].FlightPlan;
            if (fp.Patches.Count == 0)
                continue;

            ApplyPassColor(fp, i, passes.Length);
            EnsurePatchPointsCached(fp);

            // isActive=true matches stock's selected-porkchop rendering;
            // without it the lines look ghosted.
            fp.AddLineInstances(viewport, source, isActive: true,
                drawVehiclePosition: false,
                TrueAnomaly.NaN, TrueAnomaly.NaN);
        }
    }

    // 40-100% brightness ramp; final pass at full BurnPatchColor.
    // Skip Darken at the final pass: it is NOT identity at factor=1.0
    // (HSL roundtrip with sat/lightness floor at 0.1).
    private static void ApplyPassColor(FlightPlan fp, int passIndex, int totalPasses)
    {
        byte4 color = BurnPlan.BurnPatchColor;
        if (totalPasses > 1 && passIndex < totalPasses - 1)
        {
            float brightness = 0.4f + 0.6f * passIndex / (totalPasses - 1);
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

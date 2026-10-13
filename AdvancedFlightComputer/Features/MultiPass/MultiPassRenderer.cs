using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// Earlier passes are dimmer so the final trajectory remains distinct.
internal static class MultiPassRenderer
{
    // A skipped first pass is the queued burn, which stock draws itself.
    public static void RenderPassOrbits(
        IViewport viewport, Vehicle source, PassPreview[] passes,
        bool skipFirst = false)
    {
        int start = skipFirst ? 1 : 0;
        int rampCount = passes.Length - start;
        if (rampCount <= 0) return;

        for (int i = start; i < passes.Length; i++)
        {
            FlightPlan fp = passes[i].FlightPlan;
            if (fp.Patches.Count == 0)
                continue;

            ApplyPassColor(fp, i - start, rampCount);
            EnsurePatchPointsCached(fp);

            // Use the selected stock orbit style. Restrict danger ground tracks to the final pass.
            fp.AddLineInstances(viewport, source, isActive: true,
                drawVehiclePosition: false,
                nextBurnTime: null,
                drawDangerGroundTrack: i == passes.Length - 1,
                isPostBurnOrbit: true);
        }
    }

    // Scale brightness from 40 to 100 percent. Do not darken the final pass because the HSL floor makes Darken change some colors even at a factor of one.
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

    // AddLineInstances needs cached points. Generate them if the preview has not done so yet.
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

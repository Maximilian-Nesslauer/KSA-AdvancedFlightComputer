using System;
using System.Diagnostics;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance.Buffers;
using KSA;

namespace AdvancedFlightComputer.Features.OberthMultiPass;

/// <summary>
/// Renders multi-pass orbit lines and pass markers in the 3D view and as
/// ImGui overlays. Reads from MultiPassState only; contains no planning logic.
/// </summary>
static class MultiPassRenderer
{
    /// <summary>
    /// Renders preview orbit lines in the 3D view. Called from
    /// Patch_OberthPreviewRender.Postfix (OnPreRender, Vulkan only, no ImGui).
    /// Follows the same pattern as stock DrawSelectedTransfer.
    ///
    /// Also renders active split burn orbits with per-pass colors. The game's
    /// own BurnPlan.AddLineInstances clips intermediate burns via startTa/nextBurnTa,
    /// which breaks for multi-pass apse burns (all passes at the same TA).
    /// </summary>
    internal static void RenderPreviewLines(Viewport inViewport)
    {
#if DEBUG
        long perfStart = DebugConfig.Performance ? Stopwatch.GetTimestamp() : 0;
#endif
        if (MultiPassState.HasPreview && MultiPassState.ShowOrbitPreview
            && MultiPassState.PreviewSource != null && MultiPassState.PreviewPasses != null)
        {
            Vehicle source = MultiPassState.PreviewSource;
            foreach (PassResult pass in MultiPassState.PreviewPasses)
                RenderFlightPlanLines(inViewport, pass.PreviewFP, source);
        }

        if (MultiPassState.HasActiveSplit && MultiPassState.PassBurns != null
            && MultiPassState.Vehicle != null)
        {
            Vehicle source = MultiPassState.Vehicle;
            var burns = MultiPassState.PassBurns;
            for (int i = 0; i < burns.Count; i++)
            {
                SetPassColors(burns[i].FlightPlan, i, burns.Count);
                RenderFlightPlanLines(inViewport, burns[i].FlightPlan, source);
            }

            // Patch_BurnPlanAddLineInstances suppresses the game's BurnPlan rendering
            // while a split is active to avoid TA-clipping artifacts. Re-render any
            // other burns the user may have added outside our split (BurnCount/TryGetBurn
            // are both public).
            BurnPlan bp = source.FlightComputer.BurnPlan;
            int burnCount = bp.BurnCount;
            for (int i = 0; i < burnCount; i++)
            {
                if (!bp.TryGetBurn(i, out Burn? other) || other == null)
                    continue;
                if (burns.Contains(other))
                    continue;
                RenderFlightPlanLines(inViewport, other.FlightPlan, source);
            }
        }
#if DEBUG
        if (DebugConfig.Performance)
            PerfTracker.Record("MultiPassPlanner.RenderPreviewLines",
                Stopwatch.GetTimestamp() - perfStart);
#endif
    }

    /// <summary>
    /// Renders "Pass N Ap/Pe" markers on orbits. Called during the ImGui pass
    /// from Patch_DrawPlanWindow (after ImGui.End). Draws directly via
    /// ImGuiHelper.DrawTextOnScreen, bypassing PatchedConic.DrawUi which has
    /// internal conditions that fail for multi-pass burn FlightPlans.
    /// </summary>
    internal static void RenderPreviewMarkers(Viewport inViewport)
    {
        if (MultiPassState.HasPreview && MultiPassState.ShowOrbitPreview
            && MultiPassState.PreviewSource != null && MultiPassState.PreviewPasses != null)
        {
            int count = MultiPassState.PreviewPasses.Count;
            for (int i = 0; i < count; i++)
            {
                FlightPlan previewFp = MultiPassState.PreviewPasses[i].PreviewFP;
                if (previewFp.Patches.Count == 0) continue;
                DrawOrbitMarkers(inViewport, previewFp.Patches[0].Orbit, i, i == count - 1);
            }
        }

        if (MultiPassState.HasActiveSplit && MultiPassState.PassBurns != null
            && MultiPassState.Vehicle != null)
        {
            int count = MultiPassState.PassBurns.Count;
            for (int i = 0; i < count; i++)
            {
                FlightPlan fp = MultiPassState.PassBurns[i].FlightPlan;
                if (fp.Patches.Count > 0)
                    DrawOrbitMarkers(inViewport, fp.Patches[0].Orbit, i, i == count - 1);
            }
        }
    }

    /// <summary>
    /// Sets orbit line color on all patches in a FlightPlan.
    /// Last pass gets full brightness, intermediate passes get a single
    /// dimming step.
    /// Called from MultiPassPlanner during preview computation.
    /// </summary>
    internal static void SetPassColors(FlightPlan fp, int passIndex, int totalPasses)
    {
        byte4 color = BurnPlan.BurnPatchColor;
        if (passIndex < totalPasses - 1)
            color = color.Darken(0.6f);
        foreach (PatchedConic patch in fp.Patches)
            patch.Orbit.OrbitLineColor = color;
    }

    private static void RenderFlightPlanLines(Viewport inViewport, FlightPlan fp, Vehicle source)
    {
        if (fp.Patches.Count == 0)
            return;
        foreach (PatchedConic patch in fp.Patches)
        {
            patch.HidePatch = false;
            if (patch.Orbit.IsMissingPoints())
                patch.Orbit.UpdateCachedPoints(UpdateTaskUtils.GenerateSpacedPoints(patch));
        }
        fp.AddLineInstances(inViewport, source, isActive: true,
            drawVehiclePosition: false, TrueAnomaly.NaN, TrueAnomaly.NaN);
    }

    private static void DrawOrbitMarkers(Viewport inViewport, Orbit orbit, int passIndex,
        bool isLastPass)
    {
        if (!orbit.IsBound()) return;

        Camera camera = inViewport.GetCamera();
        float2 vpPos = inViewport.Position;

        ImDrawListPtr drawList = ImGui.GetBackgroundDrawList();
        byte4 color = orbit.OrbitLineColor;
        doubleQuat orb2Cce = orbit.GetOrb2ParentCce();
        double parentRadius = orbit.Parent.MeanRadius;
        float2 mousePos = ImGui.GetIO().MousePos;

        double3 apCce = orbit.GetApoapsisPositionOrb().Transform(orb2Cce);
        double3 apEcl = orbit.Parent.GetPositionEclFromCce(apCce);
        float2 apScreen = vpPos + camera.EgoToScreen(camera.EclToEgo(apEcl));

        if (!float.IsNaN(apScreen.X) && !float.IsNaN(apScreen.Y))
            DrawMarkerAtPoint(drawList, apScreen, mousePos, color, passIndex, "Ap",
                orbit.Apoapsis - parentRadius, isLastPass);

        double3 peCce = orbit.GetPeriapsisPositionOrb().Transform(orb2Cce);
        double3 peEcl = orbit.Parent.GetPositionEclFromCce(peCce);
        float2 peScreen = vpPos + camera.EgoToScreen(camera.EclToEgo(peEcl));

        if (!float.IsNaN(peScreen.X) && !float.IsNaN(peScreen.Y))
            DrawMarkerAtPoint(drawList, peScreen, mousePos, color, passIndex, "Pe",
                orbit.Periapsis - parentRadius, isLastPass);
    }

    private static void DrawMarkerAtPoint(ImDrawListPtr drawList, float2 screen, float2 mousePos,
        byte4 color, int passIndex, string apseName, double altitudeMeters, bool isLastPass)
    {
        if (isLastPass)
        {
            string label = $"Pass {passIndex + 1} {apseName}";
            ImGuiHelper.DrawTextOnScreen(drawList, screen, label, color);

            if (Math.Abs(screen.X - mousePos.X) < 80f
                && Math.Abs(screen.Y - mousePos.Y) < 80f)
            {
                float2 below = screen;
                below.Y += 15f;
                string alt = DistanceReference.ToNearest(altitudeMeters).ToString();
                ImGuiHelper.DrawTextOnScreen(drawList, below, alt, color);
            }
            return;
        }

        // Intermediate passes: inverted triangle marker, text on hover.
        float s = 6f;
        float2 p1 = new float2(screen.X - s, screen.Y - s);
        float2 p2 = new float2(screen.X + s, screen.Y - s);
        float2 p3 = new float2(screen.X, screen.Y + s);
        drawList.AddTriangleFilled(p1, p2, p3, color);

        if (Math.Abs(screen.X - mousePos.X) < 80f
            && Math.Abs(screen.Y - mousePos.Y) < 80f)
        {
            float2 textPos = screen;
            textPos.Y += s + 4f;
            string label = $"Pass {passIndex + 1} {apseName}";
            ImGuiHelper.DrawTextOnScreen(drawList, textPos, label, color);

            textPos.Y += 15f;
            string alt = DistanceReference.ToNearest(altitudeMeters).ToString();
            ImGuiHelper.DrawTextOnScreen(drawList, textPos, alt, color);
        }
    }
}

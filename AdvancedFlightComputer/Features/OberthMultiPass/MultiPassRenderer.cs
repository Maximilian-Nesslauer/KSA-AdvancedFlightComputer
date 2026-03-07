using System;
using System.Diagnostics;
using System.Globalization;
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
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

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
            PerfTracker.Record("MultiPassRenderer.RenderPreviewLines",
                Stopwatch.GetTimestamp() - perfStart);
#endif
    }

    /// <summary>
    /// Renders Ap/Pe and SOI transition markers on orbits. Called during the
    /// ImGui pass from Patch_DrawPlanWindow (after ImGui.End). Draws directly via
    /// ImGuiHelper.DrawTextOnScreen, bypassing PatchedConic.DrawUi which has
    /// internal conditions that fail for multi-pass burn FlightPlans.
    ///
    /// First and last passes show full labels. Intermediate passes show an inverted
    /// triangle with labels appearing on hover.
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
                DrawOrbitMarkers(inViewport, previewFp, i, i == 0 || i == count - 1);
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
                    DrawOrbitMarkers(inViewport, fp, i, i == 0 || i == count - 1);
            }
        }
    }

    /// <summary>
    /// Sets orbit line color on all patches in a FlightPlan. Earlier passes are
    /// progressively dimmer; the last pass gets full brightness.
    /// </summary>
    internal static void SetPassColors(FlightPlan fp, int passIndex, int totalPasses)
    {
        byte4 color = BurnPlan.BurnPatchColor;
        if (totalPasses > 1 && passIndex < totalPasses - 1)
        {
            float brightness = 0.4f + 0.6f * ((float)passIndex / (totalPasses - 1));
            color = color.Darken(brightness);
        }
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

    /// <summary>
    /// Draws Ap/Pe markers from the first patch and SOI transition (escape, encounter,
    /// impact) markers from all patches of the given FlightPlan.
    /// showFull: true for first and last pass (full labels), false for intermediate (triangle + hover).
    /// </summary>
    private static void DrawOrbitMarkers(Viewport inViewport, FlightPlan fp,
        int passIndex, bool showFull)
    {
        Orbit firstOrbit = fp.Patches[0].Orbit;

        Camera camera = inViewport.GetCamera();
        float2 vpPos = inViewport.Position;
        ImDrawListPtr drawList = ImGui.GetBackgroundDrawList();
        byte4 color = firstOrbit.OrbitLineColor;
        float2 mousePos = ImGui.GetIO().MousePos;

        // Ap/Pe markers from the first post-burn orbit.
        if (firstOrbit.IsBound())
        {
            doubleQuat orb2Cce = firstOrbit.GetOrb2ParentCce();
            double parentRadius = firstOrbit.Parent.MeanRadius;

            double3 apCce = firstOrbit.GetApoapsisPositionOrb().Transform(orb2Cce);
            double3 apEcl = firstOrbit.Parent.GetPositionEclFromCce(apCce);
            float2 apScreen = vpPos + camera.EgoToScreen(camera.EclToEgo(apEcl));

            if (!float.IsNaN(apScreen.X) && !float.IsNaN(apScreen.Y))
                DrawMarkerAtPoint(drawList, apScreen, mousePos, color,
                    $"Pass {passIndex + 1} Ap",
                    DistanceReference.ToNearest(firstOrbit.Apoapsis - parentRadius).ToString(),
                    showFull);

            double3 peCce = firstOrbit.GetPeriapsisPositionOrb().Transform(orb2Cce);
            double3 peEcl = firstOrbit.Parent.GetPositionEclFromCce(peCce);
            float2 peScreen = vpPos + camera.EgoToScreen(camera.EclToEgo(peEcl));

            if (!float.IsNaN(peScreen.X) && !float.IsNaN(peScreen.Y))
                DrawMarkerAtPoint(drawList, peScreen, mousePos, color,
                    $"Pass {passIndex + 1} Pe",
                    DistanceReference.ToNearest(firstOrbit.Periapsis - parentRadius).ToString(),
                    showFull);
        }

        // Per-patch markers: SOI transitions, AN/DN relative to target, closest approach.
        foreach (PatchedConic patch in fp.Patches)
        {
            Orbit o = patch.Orbit;
            if (o.Parent == null) continue;

            doubleQuat patchOrb2Cce = o.GetOrb2ParentCce();

            // SOI transition markers (escape, encounter, impact) at patch end.
            PatchTransition t = patch.EndTransition;
            if (t == PatchTransition.Escape || t == PatchTransition.Encounter
                || t == PatchTransition.Impact)
            {
                double3 posCce = o.GetPositionOrb(patch.EndTrueAnomaly).Transform(patchOrb2Cce);
                double3 posEcl = o.Parent.GetPositionEclFromCce(posCce);
                float2 tScreen = vpPos + camera.EgoToScreen(camera.EclToEgo(posEcl));

                if (!float.IsNaN(tScreen.X) && !float.IsNaN(tScreen.Y))
                {
                    string tLabel = t switch
                    {
                        PatchTransition.Escape    => $"Pass {passIndex + 1} Exit SOI",
                        PatchTransition.Encounter => $"Pass {passIndex + 1} Enter SOI",
                        PatchTransition.Impact    => $"Pass {passIndex + 1} Impact",
                        _                         => string.Empty
                    };
                    if (tLabel.Length > 0)
                        DrawMarkerAtPoint(drawList, tScreen, mousePos, color, tLabel, null, showFull);
                }
            }

            // AN/DN markers relative to target.
            if (patch.TargetData.HasValue)
            {
                TargetData td = patch.TargetData.Value;
                string relIncStr = string.Format(Inv, "{0:F2} deg", td.RelativeInclination);

                if (PatchedConic.TrueAnomalyInPatch(td.AnTrueAnomaly,
                        patch.StartTrueAnomaly, patch.EndTrueAnomaly))
                {
                    double3 anCce = o.GetPositionOrb(td.AnTrueAnomaly).Transform(patchOrb2Cce);
                    float2 anScreen = vpPos + camera.EgoToScreen(
                        camera.EclToEgo(o.Parent.GetPositionEclFromCce(anCce)));
                    if (!float.IsNaN(anScreen.X) && !float.IsNaN(anScreen.Y))
                        DrawMarkerAtPoint(drawList, anScreen, mousePos, color,
                            $"Pass {passIndex + 1} AN", relIncStr, showFull);
                }

                if (PatchedConic.TrueAnomalyInPatch(td.DnTrueAnomaly,
                        patch.StartTrueAnomaly, patch.EndTrueAnomaly))
                {
                    double3 dnCce = o.GetPositionOrb(td.DnTrueAnomaly).Transform(patchOrb2Cce);
                    float2 dnScreen = vpPos + camera.EgoToScreen(
                        camera.EclToEgo(o.Parent.GetPositionEclFromCce(dnCce)));
                    if (!float.IsNaN(dnScreen.X) && !float.IsNaN(dnScreen.Y))
                        DrawMarkerAtPoint(drawList, dnScreen, mousePos, color,
                            $"Pass {passIndex + 1} DN", relIncStr, showFull);
                }
            }

            // Closest approach markers.
            foreach (Encounter enc in patch.ClosestApproaches)
            {
                if (!PatchedConic.TrueAnomalyInPatch(enc.TaMainOrbit,
                        patch.StartTrueAnomaly, patch.EndTrueAnomaly))
                    continue;

                double3 encCce = o.GetPositionOrb(enc.TaMainOrbit).Transform(patchOrb2Cce);
                float2 encScreen = vpPos + camera.EgoToScreen(
                    camera.EclToEgo(o.Parent.GetPositionEclFromCce(encCce)));
                if (float.IsNaN(encScreen.X) || float.IsNaN(encScreen.Y)) continue;

                string distStr = DistanceReference.ToNearest(enc.ClosestDistance).ToString();
                DrawMarkerAtPoint(drawList, encScreen, mousePos, color,
                    $"Pass {passIndex + 1} Closest", distStr, showFull);
            }
        }
    }

    /// <summary>
    /// Draws a single orbit marker at a screen position.
    /// showFull = true: draws the label text directly (and hoverExtra below on hover).
    /// showFull = false: draws an inverted triangle; label + hoverExtra appear on hover.
    /// </summary>
    private static void DrawMarkerAtPoint(ImDrawListPtr drawList, float2 screen, float2 mousePos,
        byte4 color, string label, string? hoverExtra, bool showFull)
    {
        bool hovered = Math.Abs(screen.X - mousePos.X) < 80f
            && Math.Abs(screen.Y - mousePos.Y) < 80f;

        if (showFull)
        {
            ImGuiHelper.DrawTextOnScreen(drawList, screen, label, color);

            if (hovered && hoverExtra != null)
            {
                float2 below = screen;
                below.Y += 15f;
                ImGuiHelper.DrawTextOnScreen(drawList, below, hoverExtra, color);
            }
            return;
        }

        // Intermediate passes: inverted triangle, text on hover.
        float s = 6f;
        float2 p1 = new float2(screen.X - s, screen.Y - s);
        float2 p2 = new float2(screen.X + s, screen.Y - s);
        float2 p3 = new float2(screen.X, screen.Y + s);
        drawList.AddTriangleFilled(p1, p2, p3, color);

        if (hovered)
        {
            float2 textPos = new float2(screen.X, screen.Y + s + 4f);
            ImGuiHelper.DrawTextOnScreen(drawList, textPos, label, color);

            if (hoverExtra != null)
            {
                textPos.Y += 15f;
                ImGuiHelper.DrawTextOnScreen(drawList, textPos, hoverExtra, color);
            }
        }
    }
}

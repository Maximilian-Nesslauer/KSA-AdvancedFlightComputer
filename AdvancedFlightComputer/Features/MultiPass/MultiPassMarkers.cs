using System;
using System.Globalization;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>Per-pass marker overlays for the multi-pass preview.
/// First (next-to-execute) pass uses plain labels ("Ap", "Pe"), final
/// pass prefixes "Final", intermediate passes collapse to an inverted
/// triangle with the same info on hover. With skipLast=true (Hohmann),
/// the second-to-last pass uses PreFinalRaise mode to call out its
/// apoapsis as "Pre-SOI-escape AP" - the highest orbit reached before
/// the final ejection burn.</summary>
internal static class MultiPassMarkers
{
    private enum MarkerMode { Full, FinalFull, Triangle, PreFinalRaise }

    // Special-case label for the pre-final raising pass's apoapsis. Spelt
    // out in caps to stand visually apart from the standard Ap / Pe. The
    // embedded newline triggers the centered multi-line render path in
    // DrawMarker so the long label doesn't shoot off to the right of the
    // dot anchor like a single-line render would.
    private const string PreFinalRaiseApLabel = "Pre-SOI-\nescape AP";

    private const float HoverRadiusPx = 100f;

    /// <summary>When <paramref name="skipFirst"/> is true, passes[0] is
    /// omitted; the first shown pass starts as Triangle (stock owns the
    /// "next" position). When <paramref name="skipLast"/> is true, the
    /// last pass is omitted - Hohmann uses this because stock renders
    /// the selected-entry markers ("Escape 0", "Ap" etc.) for the final
    /// pass trajectory via <c>FlightPlan.DrawUi</c>.
    /// <paramref name="firstPassDisplayNumber"/> is the 1-based pass
    /// number for passes[0] so hover labels reflect the absolute pass
    /// number (e.g. "Ap Pass 4" mid-execution instead of restarting
    /// at 1).</summary>
    public static void Draw(
        Viewport viewport, Vehicle source, PassPreview[] passes,
        int firstPassDisplayNumber = 1,
        bool skipFirst = false, bool skipLast = false)
    {
        int start = skipFirst ? 1 : 0;
        int end = passes.Length - (skipLast ? 1 : 0);

        // rampCount includes any skipped-last slot so FinalFull mapping
        // still refers to the actual final pass; with skipLast=true no
        // rendered pass is at rel == rampCount - 1, so no FinalFull is
        // produced (stock's own marker labels the final pass instead).
        int rampCount = passes.Length - start;

        Camera camera = viewport.GetCamera();
        float2 vpPos = viewport.Position;
        // Not the bare GetBackgroundDrawList(): that leaves the viewport
        // argument null instead of naming the main one. Stock routes every
        // overlay through this helper since it fixed orbit lines bleeding into
        // the crew portrait views.
        ImDrawListPtr drawList = ImGuiHelper.GetOverlayDrawList(viewport);
        float2 mousePos = ImGui.GetIO().MousePos;

        // Special case at PassIndex == N-2: the cache contains only the
        // queued pre-final-raise pass (passes[0]) and the final ejection
        // (passes[1]). Both are skipped by skipFirst / skipLast for orbit
        // rendering, but the user is now coasting toward exactly the
        // pre-final-raise apoapsis - the most informative moment of the
        // sequence. Draw passes[0]'s PreFinalRaise marker (label only,
        // stock's BurnPlan integration is rendering the orbit itself).
        if (skipFirst && skipLast && passes.Length == 2)
        {
            DrawPass(passes[0], MarkerMode.PreFinalRaise,
                firstPassDisplayNumber, drawList, camera, vpPos, mousePos);
            return;
        }

        if (end - start <= 0) return;

        for (int i = start; i < end; i++)
        {
            int rel = i - start;
            // PreFinalRaise takes precedence over Full when both apply
            // (N=2 init: pass 0 is both "first rendered" and "the only
            // raising pass before escape"; the latter is the more
            // informative label).
            MarkerMode mode =
                skipLast && rel == rampCount - 2 ? MarkerMode.PreFinalRaise
                : rel == rampCount - 1 ? MarkerMode.FinalFull
                : rel == 0 && !skipFirst ? MarkerMode.Full
                : MarkerMode.Triangle;
            int passNumber = firstPassDisplayNumber + i;
            DrawPass(passes[i], mode, passNumber, drawList, camera, vpPos, mousePos);
        }
    }

    private static void DrawPass(
        PassPreview pass, MarkerMode mode, int passNumber,
        ImDrawListPtr drawList, Camera camera, float2 vpPos, float2 mousePos)
    {
        FlightPlan fp = pass.FlightPlan;
        if (fp.Patches.Count == 0) return;
        Orbit firstOrbit = fp.Patches[0].Orbit;
        byte4 color = firstOrbit.OrbitLineColor;

        // PreFinalRaise highlights only this orbit's apoapsis (the
        // "highest pre-escape" point). Everything else on this pass
        // collapses to Triangle to keep the overlay tidy and avoid
        // pulling attention from the actual key marker.
        bool isPreFinalRaise = mode == MarkerMode.PreFinalRaise;
        MarkerMode secondaryMode = isPreFinalRaise ? MarkerMode.Triangle : mode;

        // Ap / Pe of the immediate post-burn orbit; unbound passes skip
        // (planner already flagged the result as Failed).
        if (firstOrbit.IsBound() && firstOrbit.Parent != null)
        {
            double parentRadius = firstOrbit.Parent.MeanRadius;
            doubleQuat orb2Cce = firstOrbit.GetOrb2ParentCce();

            string apLabel = isPreFinalRaise ? PreFinalRaiseApLabel : "Ap";
            MarkerMode apMode = isPreFinalRaise ? MarkerMode.Full : mode;

            DrawAt(firstOrbit.Parent, firstOrbit.GetApoapsisPositionOrb().Transform(orb2Cce),
                apLabel, ManeuverToolsWindow.FormatDistance(firstOrbit.Apoapsis - parentRadius),
                color, apMode, passNumber, drawList, camera, vpPos, mousePos);
            DrawAt(firstOrbit.Parent, firstOrbit.GetPeriapsisPositionOrb().Transform(orb2Cce),
                "Pe", ManeuverToolsWindow.FormatDistance(firstOrbit.Periapsis - parentRadius),
                color, secondaryMode, passNumber, drawList, camera, vpPos, mousePos);
        }

        // Per-patch markers: SOI transitions, AN/DN, closest approaches.
        foreach (PatchedConic patch in fp.Patches)
        {
            Orbit o = patch.Orbit;
            if (o.Parent == null) continue;
            doubleQuat patchOrb2Cce = o.GetOrb2ParentCce();

            DrawSoiTransition(patch, patchOrb2Cce, color, secondaryMode, passNumber,
                drawList, camera, vpPos, mousePos);
            DrawAnDn(patch, patchOrb2Cce, color, secondaryMode, passNumber,
                drawList, camera, vpPos, mousePos);
            DrawClosestApproaches(patch, patchOrb2Cce, color, secondaryMode, passNumber,
                drawList, camera, vpPos, mousePos);
        }
    }

    private static void DrawSoiTransition(
        PatchedConic patch, doubleQuat patchOrb2Cce, byte4 color, MarkerMode mode,
        int passNumber,
        ImDrawListPtr drawList, Camera camera, float2 vpPos, float2 mousePos)
    {
        string? label = patch.EndTransition switch
        {
            PatchTransition.Escape => "Exit SOI",
            PatchTransition.Encounter => "Enter SOI",
            PatchTransition.Impact => "Impact",
            _ => null,
        };
        if (label == null) return;

        Orbit o = patch.Orbit;
        double3 posCce = o.GetPositionOrb(patch.EndTrueAnomaly).Transform(patchOrb2Cce);
        DrawAt(o.Parent, posCce, label, null,
            color, mode, passNumber, drawList, camera, vpPos, mousePos);
    }

    private static void DrawAnDn(
        PatchedConic patch, doubleQuat patchOrb2Cce, byte4 color, MarkerMode mode,
        int passNumber,
        ImDrawListPtr drawList, Camera camera, float2 vpPos, float2 mousePos)
    {
        if (!patch.TargetData.HasValue) return;
        TargetData td = patch.TargetData.Value;
        string relIncStr = string.Format(CultureInfo.InvariantCulture,
            "{0:F2} deg", td.RelativeInclination);
        Orbit o = patch.Orbit;

        if (PatchedConic.TrueAnomalyInPatch(td.AnTrueAnomaly,
                patch.StartTrueAnomaly, patch.EndTrueAnomaly))
        {
            double3 anCce = o.GetPositionOrb(td.AnTrueAnomaly).Transform(patchOrb2Cce);
            DrawAt(o.Parent, anCce, "AN", relIncStr,
                color, mode, passNumber, drawList, camera, vpPos, mousePos);
        }

        if (PatchedConic.TrueAnomalyInPatch(td.DnTrueAnomaly,
                patch.StartTrueAnomaly, patch.EndTrueAnomaly))
        {
            double3 dnCce = o.GetPositionOrb(td.DnTrueAnomaly).Transform(patchOrb2Cce);
            DrawAt(o.Parent, dnCce, "DN", relIncStr,
                color, mode, passNumber, drawList, camera, vpPos, mousePos);
        }
    }

    private static void DrawClosestApproaches(
        PatchedConic patch, doubleQuat patchOrb2Cce, byte4 color, MarkerMode mode,
        int passNumber,
        ImDrawListPtr drawList, Camera camera, float2 vpPos, float2 mousePos)
    {
        Orbit o = patch.Orbit;
        foreach (Encounter enc in patch.ClosestApproaches)
        {
            if (!PatchedConic.TrueAnomalyInPatch(enc.TaMainOrbit,
                    patch.StartTrueAnomaly, patch.EndTrueAnomaly))
                continue;
            double3 encCce = o.GetPositionOrb(enc.TaMainOrbit).Transform(patchOrb2Cce);
            DrawAt(o.Parent, encCce,
                "Closest",
                ManeuverToolsWindow.FormatDistance(enc.ClosestDistance),
                color, mode, passNumber, drawList, camera, vpPos, mousePos);
        }
    }

    private static void DrawAt(
        IParentBody parent, double3 posCce, string label, string? hoverExtra,
        byte4 color, MarkerMode mode, int passNumber,
        ImDrawListPtr drawList, Camera camera, float2 vpPos, float2 mousePos)
    {
        double3 posEcl = parent.GetPositionEclFromCce(posCce);
        float2 screen = vpPos + camera.EgoToScreen(camera.EclToEgo(posEcl));
        if (float.IsNaN(screen.X) || float.IsNaN(screen.Y)) return;
        DrawMarker(drawList, screen, mousePos, color, label, hoverExtra, mode, passNumber);
    }

    private static void DrawMarker(
        ImDrawListPtr drawList, float2 screen, float2 mousePos, byte4 color,
        string label, string? hoverExtra, MarkerMode mode, int passNumber)
    {
        bool hovered = Math.Abs(screen.X - mousePos.X) < HoverRadiusPx
                    && Math.Abs(screen.Y - mousePos.Y) < HoverRadiusPx;

        if (mode == MarkerMode.Triangle)
        {
            const float s = 6f;
            drawList.AddTriangleFilled(
                new float2(screen.X - s, screen.Y - s),
                new float2(screen.X + s, screen.Y - s),
                new float2(screen.X, screen.Y + s),
                color);
            if (hovered)
            {
                // Hover label disambiguates which intermediate pass.
                string hoverLabel = string.Format(CultureInfo.InvariantCulture,
                    "{0} Pass {1}", label, passNumber);
                float2 textPos = new float2(screen.X, screen.Y + s + 4f);
                ImGuiHelper.DrawTextOnScreen(drawList, textPos, hoverLabel, color);
                if (hoverExtra != null)
                {
                    textPos.Y += ImGui.GetTextLineHeight();
                    ImGuiHelper.DrawTextOnScreen(drawList, textPos, hoverExtra, color);
                }
            }
            return;
        }

        // Full / FinalFull: text marker, hover adds extra below.
        string display = mode == MarkerMode.FinalFull ? "Final " + label : label;
        int lineCount = DrawLabel(drawList, screen, color, display);
        if (hovered && hoverExtra != null)
        {
            float2 below = screen;
            below.Y += lineCount * ImGui.GetTextLineHeight();
            DrawLabel(drawList, below, color, hoverExtra);
        }
    }

    /// <summary>Renders <paramref name="text"/> at <paramref name="screen"/>.
    /// Single-line labels (no embedded newline) keep the historical
    /// top-left anchor used by stock orbit markers so short tags like
    /// "Ap" / "Pe" sit in the same place they always have. Labels with
    /// an embedded newline are split, each line horizontally centered on
    /// the anchor X - this is how the long "Pre-SOI-escape AP" tag stays
    /// visually pinned to its dot instead of trailing off to the right.
    /// Returns the number of lines drawn so callers can offset any
    /// follow-up text (e.g. the hover-extra row).</summary>
    private static int DrawLabel(
        ImDrawListPtr drawList, float2 anchor, byte4 color, string text)
    {
        if (string.IsNullOrEmpty(text)) return 0;
        if (!text.Contains('\n'))
        {
            ImGuiHelper.DrawTextOnScreen(drawList, anchor, text, color);
            return 1;
        }

        string[] lines = text.Split('\n');
        float lineHeight = ImGui.GetTextLineHeight();
        for (int i = 0; i < lines.Length; i++)
        {
            float lineWidth = ImGui.CalcTextSize(lines[i]).X;
            var pos = new float2(
                anchor.X - lineWidth * 0.5f,
                anchor.Y + i * lineHeight);
            ImGuiHelper.DrawTextOnScreen(drawList, pos, lines[i], color);
        }
        return lines.Length;
    }
}

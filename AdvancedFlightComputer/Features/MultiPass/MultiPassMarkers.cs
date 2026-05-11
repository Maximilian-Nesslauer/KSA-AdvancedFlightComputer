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
/// triangle with the same info on hover.</summary>
internal static class MultiPassMarkers
{
    private enum MarkerMode { Full, FinalFull, Triangle }

    private const float HoverRadiusPx = 100f;

    public static void Draw(Viewport viewport, Vehicle source, PassPreview[] passes)
    {
        if (passes.Length == 0) return;

        Camera camera = viewport.GetCamera();
        float2 vpPos = viewport.Position;
        ImDrawListPtr drawList = ImGui.GetBackgroundDrawList();
        float2 mousePos = ImGui.GetIO().MousePos;

        for (int i = 0; i < passes.Length; i++)
        {
            MarkerMode mode = (i == 0) ? MarkerMode.Full
                : (i == passes.Length - 1) ? MarkerMode.FinalFull
                : MarkerMode.Triangle;
            DrawPass(passes[i], mode, i + 1, drawList, camera, vpPos, mousePos);
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

        // Ap / Pe of the immediate post-burn orbit; unbound passes skip
        // (planner already flagged the result as Failed).
        if (firstOrbit.IsBound() && firstOrbit.Parent != null)
        {
            double parentRadius = firstOrbit.Parent.MeanRadius;
            doubleQuat orb2Cce = firstOrbit.GetOrb2ParentCce();

            DrawAt(firstOrbit.Parent, firstOrbit.GetApoapsisPositionOrb().Transform(orb2Cce),
                "Ap", ManeuverToolsWindow.FormatDistance(firstOrbit.Apoapsis - parentRadius),
                color, mode, passNumber, drawList, camera, vpPos, mousePos);
            DrawAt(firstOrbit.Parent, firstOrbit.GetPeriapsisPositionOrb().Transform(orb2Cce),
                "Pe", ManeuverToolsWindow.FormatDistance(firstOrbit.Periapsis - parentRadius),
                color, mode, passNumber, drawList, camera, vpPos, mousePos);
        }

        // Per-patch markers: SOI transitions, AN/DN, closest approaches.
        foreach (PatchedConic patch in fp.Patches)
        {
            Orbit o = patch.Orbit;
            if (o.Parent == null) continue;
            doubleQuat patchOrb2Cce = o.GetOrb2ParentCce();

            DrawSoiTransition(patch, patchOrb2Cce, color, mode, passNumber,
                drawList, camera, vpPos, mousePos);
            DrawAnDn(patch, patchOrb2Cce, color, mode, passNumber,
                drawList, camera, vpPos, mousePos);
            DrawClosestApproaches(patch, patchOrb2Cce, color, mode, passNumber,
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
                    textPos.Y += 15f;
                    ImGuiHelper.DrawTextOnScreen(drawList, textPos, hoverExtra, color);
                }
            }
            return;
        }

        // Full / FinalFull: text marker, hover adds extra below.
        string display = mode == MarkerMode.FinalFull ? "Final " + label : label;
        ImGuiHelper.DrawTextOnScreen(drawList, screen, display, color);
        if (hovered && hoverExtra != null)
        {
            float2 below = screen;
            below.Y += 15f;
            ImGuiHelper.DrawTextOnScreen(drawList, below, hoverExtra, color);
        }
    }
}

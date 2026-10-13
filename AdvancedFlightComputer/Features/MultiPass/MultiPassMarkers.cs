using System;
using System.Globalization;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// The first pass uses plain apsis labels and the final pass adds Final. Intermediate passes use triangles with hover details.
internal static class MultiPassMarkers
{
    private enum MarkerMode { Full, FinalFull, Triangle }

    private enum DetailKind { None, Distance, Inclination }

    private readonly record struct MarkerDetail(DetailKind Kind, double Value)
    {
        public string? Format() => Kind switch
        {
            DetailKind.Distance => ManeuverToolsWindow.FormatDistance(Value),
            DetailKind.Inclination => string.Format(CultureInfo.InvariantCulture, "{0:F2} deg", Value),
            _ => null,
        };
    }

    private const float HoverRadiusPx = 100f;

    // A skipped first pass keeps its position in the label sequence.
    public static void Draw(
        IViewport viewport, Vehicle source, PassPreview[] passes,
        int firstPassDisplayNumber = 1, bool skipFirst = false)
    {
        int start = skipFirst ? 1 : 0;
        int rampCount = passes.Length - start;
        if (rampCount <= 0) return;

        Camera camera = viewport.GetCamera();
        float2 vpPos = viewport.Position;
        // Use the overlay viewport so markers do not appear over a portrait viewport.
        ImDrawListPtr drawList = ImGuiHelper.GetOverlayDrawList(viewport);
        float2 mousePos = ImGui.GetIO().MousePos;

        for (int i = start; i < passes.Length; i++)
        {
            int rel = i - start;
            MarkerMode mode =
                rel == rampCount - 1 ? MarkerMode.FinalFull
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

        // Only bound orbits have both apsides immediately after the burn.
        if (firstOrbit.IsBound() && firstOrbit.Parent != null)
        {
            double parentRadius = firstOrbit.Parent.MeanRadius;
            doubleQuat orb2Cce = firstOrbit.GetOrb2ParentCce();

            DrawAt(firstOrbit.Parent, firstOrbit.GetApoapsisPositionOrb().Transform(orb2Cce),
                "Ap", new MarkerDetail(DetailKind.Distance, firstOrbit.Apoapsis - parentRadius),
                color, mode, passNumber, drawList, camera, vpPos, mousePos);
            DrawAt(firstOrbit.Parent, firstOrbit.GetPeriapsisPositionOrb().Transform(orb2Cce),
                "Pe", new MarkerDetail(DetailKind.Distance, firstOrbit.Periapsis - parentRadius),
                color, mode, passNumber, drawList, camera, vpPos, mousePos);
        }

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
        DrawAt(o.Parent, posCce, label, default,
            color, mode, passNumber, drawList, camera, vpPos, mousePos);
    }

    private static void DrawAnDn(
        PatchedConic patch, doubleQuat patchOrb2Cce, byte4 color, MarkerMode mode,
        int passNumber,
        ImDrawListPtr drawList, Camera camera, float2 vpPos, float2 mousePos)
    {
        if (!patch.TargetData.HasValue) return;
        TargetData td = patch.TargetData.Value;
        var detail = new MarkerDetail(DetailKind.Inclination, td.RelativeInclination);
        Orbit o = patch.Orbit;

        if (PatchedConic.TrueAnomalyInPatch(td.AnTrueAnomaly,
                patch.StartTrueAnomaly, patch.EndTrueAnomaly))
        {
            double3 anCce = o.GetPositionOrb(td.AnTrueAnomaly).Transform(patchOrb2Cce);
            DrawAt(o.Parent, anCce, "AN", detail,
                color, mode, passNumber, drawList, camera, vpPos, mousePos);
        }

        if (PatchedConic.TrueAnomalyInPatch(td.DnTrueAnomaly,
                patch.StartTrueAnomaly, patch.EndTrueAnomaly))
        {
            double3 dnCce = o.GetPositionOrb(td.DnTrueAnomaly).Transform(patchOrb2Cce);
            DrawAt(o.Parent, dnCce, "DN", detail,
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
                new MarkerDetail(DetailKind.Distance, enc.ClosestDistance),
                color, mode, passNumber, drawList, camera, vpPos, mousePos);
        }
    }

    private static void DrawAt(
        IParentBody parent, double3 posCce, string label, MarkerDetail detail,
        byte4 color, MarkerMode mode, int passNumber,
        ImDrawListPtr drawList, Camera camera, float2 vpPos, float2 mousePos)
    {
        double3 posEcl = parent.GetPositionEclFromCce(posCce);
        float2 screen = vpPos + camera.EgoToScreen(camera.EclToEgo(posEcl));
        if (float.IsNaN(screen.X) || float.IsNaN(screen.Y)) return;
        DrawMarker(drawList, screen, mousePos, color, label, detail, mode, passNumber);
    }

    private static void DrawMarker(
        ImDrawListPtr drawList, float2 screen, float2 mousePos, byte4 color,
        string label, MarkerDetail detail, MarkerMode mode, int passNumber)
    {
        bool hovered = Math.Abs(screen.X - mousePos.X) < HoverRadiusPx
                    && Math.Abs(screen.Y - mousePos.Y) < HoverRadiusPx;

        string? hoverExtra = hovered ? detail.Format() : null;

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
                // Include the pass number in hover labels so overlapping markers remain identifiable.
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

        string display = mode == MarkerMode.FinalFull ? "Final " + label : label;
        ImGuiHelper.DrawTextOnScreen(drawList, screen, display, color);
        if (hovered && hoverExtra != null)
        {
            float2 below = screen;
            below.Y += ImGui.GetTextLineHeight();
            ImGuiHelper.DrawTextOnScreen(drawList, below, hoverExtra, color);
        }
    }
}

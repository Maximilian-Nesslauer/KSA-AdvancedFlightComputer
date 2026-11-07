using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Pictures;

/// <summary>The ORBIT pictures. The orbit sketch is a schematic, because at true scale a low orbit lies on the body and two low orbits lie on each other, so the periapsis and the apoapsis of an orbit run on a root scale (SketchMath.SchematicRadius), one smooth conic runs through those two points (SketchMath.ConicShare), and the labels hold the true numbers. The transfer sketch is to scale.</summary>
internal static class OrbitSketch
{
    private const int Samples = 96;
    private static readonly PointBuffer _points = new();
    private static readonly List<P2> _world = new(Samples * 6);
    private static readonly Labels _labels = new();
    private static readonly string[] _legendNames = new string[6];
    private static readonly Picture.Swatch[] _shownLegend = new Picture.Swatch[6];

    private const float LineClearGap = 4f;

    /// <summary>One orbit the sketch draws, with its colour and weight.</summary>
    internal readonly record struct Track(Orbit Orbit, ImColor8 Colour, float Thickness, bool Dashed);

    // The frame of a schematic sketch on the screen.
    private readonly record struct SketchFrame(float2 Centre, float Outer, double3 U, double3 V, double Body, double Top, double Air)
    {
        public double Share(double radius) => SketchMath.SchematicRadius(radius, Body, Top, airRadius: Air);
    }

    private readonly record struct OrbitShape(bool Closed, double3 ToPe, double3 Side, double PeShare, double ApShare);

    private static OrbitShape ShapeOf(Orbit orbit, in SketchFrame frame)
    {
        if (!(orbit.Eccentricity < 1.0) || !double.IsFinite(orbit.Apoapsis))
            return default;
        double3 toPe = Normalize(orbit.GetPositionCci(new TrueAnomaly(0.0)));
        double3 onward = orbit.GetPositionCci(new TrueAnomaly(Math.PI * 0.5));
        return new OrbitShape(true, toPe, Normalize(onward - toPe * double3.Dot(onward, toPe)), frame.Share(orbit.Periapsis), frame.Share(orbit.Apoapsis));
    }

    // The screen point of a point in the frame of the body, on the conic of its orbit, with the direction it has in the plane of the sketch.
    private static float2 ScreenOf(double3 p, in OrbitShape shape, in SketchFrame frame)
    {
        P2 flat = Proj(p, frame.U, frame.V);
        double planar = Math.Sqrt(flat.X * flat.X + flat.Y * flat.Y);
        double full = p.Length();
        if (!(planar > 0.0) || !(full > 0.0))
            return frame.Centre;
        double share = shape.Closed
            ? SketchMath.ConicShare(shape.PeShare, shape.ApShare, Math.Atan2(double3.Dot(p, shape.Side), double3.Dot(p, shape.ToPe))) * (planar / full)
            : frame.Share(planar);
        float sr = (float)share * frame.Outer;
        return frame.Centre + new float2((float)(flat.X / planar) * sr, -(float)(flat.Y / planar) * sr);
    }

    /// <summary><paramref name="current"/> sets the frame, <paramref name="labelled"/> is the orbit whose apsides the dim labels name, and <paramref name="result"/> is the orbit after the burn, whose apsides the bright labels name where they differ.</summary>
    public static void Draw(in PictureFrame f, Orbit current, Orbit labelled, IReadOnlyList<Track> tracks, double3? burnCci, string big, Orbit? result,
        string head, ReadOnlySpan<Picture.Swatch> legend, string burnLabel = "BURN", bool resultShort = false)
    {
        IParentBody? parent = current.Parent;
        if (parent == null || !TryPlane(current, out double3 u, out double3 v))
        {
            Picture.Empty(in f, PageText.NoOrbit);
            return;
        }

        float s = f.Scale;
        double radius = parent.MeanRadius;
        double atmo = radius + (parent.GetAtmosphereReference()?.Physical?.Height ?? 0.0);
        double top = Math.Max(MaxClosedRadius(tracks), radius * 1.001);

        bool hasAir = atmo > radius;
        int names = Math.Min(legend.Length, _legendNames.Length - (hasAir ? 1 : 0));
        for (int i = 0; i < names; i++)
            _legendNames[i] = legend[i].Name;
        if (hasAir)
            _legendNames[names++] = PageText.AirKey;
        PictureLanes lanes = Picture.Lanes(in f, Picture.HeadHeight(in f, big, text: head.Length > 0), 0f, names > 0 ? Picture.LegendWidth(_legendNames.AsSpan(0, names)) : 0f);
        PicBox plot = lanes.Plot;

        string apNow = "", peNow = "", apAfter = "", peAfter = "";
        ApsisTexts(labelled, radius, out apNow, out peNow);
        if (result != null)
            ApsisTexts(result, radius, out apAfter, out peAfter);
        if (apAfter == apNow)
            apAfter = "";
        if (peAfter == peNow)
            peAfter = "";
        bool peInAir = peAfter.Length > 0 && result != null && SketchMath.PeInAir(result.Periapsis - radius, atmo - radius);
        ConsoleUi.PushSmall();
        float rowH = ImGui.GetFontSize() + 2f * s;
        float apW = Math.Max(ImGui.CalcTextSize(apNow).X, ImGui.CalcTextSize(apAfter).X);
        float peW = Math.Max(Math.Max(ImGui.CalcTextSize(peNow).X, ImGui.CalcTextSize(peAfter).X), peInAir ? ImGui.CalcTextSize(PageText.PeInAir).X : 0f);
        ConsoleStyle.PopFont();
        float gap = 8f * s;
        float columns = Math.Max(apW, peW) + gap;
        float outer = Math.Max(20f * s, Math.Min(plot.H * 0.5f - 6f * s, plot.W * 0.5f - columns - 6f * s));
        var c = new float2(plot.X + plot.W * 0.5f, plot.Y + plot.H * 0.5f);
        var frame = new SketchFrame(c, outer, u, v, radius, top, hasAir ? atmo : 0.0);

        Labels labels = _labels;
        labels.Begin(f.Dl, in plot, s);

        float bodyEdge = (float)(SketchMath.SketchBodyShare + (hasAir ? SketchMath.SketchAirShare : 0.0)) * outer;
        if (hasAir)
            f.Dl.AddCircleFilled(c, bodyEdge, Palette.Air.A(0.55f));
        f.Dl.AddCircleFilled(c, (float)SketchMath.SketchBodyShare * outer, new ImColor8(40, 46, 56, 255));

        bool dashedSeen = DashedLineSeen(tracks, radius, top, hasAir ? atmo : 0.0, outer, LineClearGap * s);
        using (new PlotClip(f.Dl, in plot))
        {
            for (int k = 0; k < tracks.Count; k++)
            {
                Track t = tracks[k];
                if (t.Dashed && !dashedSeen)
                    continue;
                float2[] pts = _points.Get(Samples + 1);
                int count = TrackPoints(t.Orbit, in frame, top * 1.0001, pts);
                if (count < 2)
                    continue;
                if (t.Dashed)
                    Glyphs.DashedPolyline(f.Dl, pts.AsSpan(0, count), t.Colour, t.Thickness * s, 5f * s, 3f * s);
                else
                    f.Dl.AddPolyline(pts.AsSpan(0, count), t.Colour, ImDrawFlags.None, t.Thickness * s);
                labels.Curve(pts.AsSpan(0, count));
            }

            if (labelled.Eccentricity < 1.0 && SketchMath.ApsisDotsShow(labelled.Eccentricity))
                ApsisDots(in f, labelled, in frame, Palette.Dim.Solid, labels);
            if (result != null && result.Eccentricity < 1.0 && SketchMath.ApsisDotsShow(result.Eccentricity))
                ApsisDots(in f, result, in frame, Palette.Plan.Solid, labels);
        }

        Column(in f, labels, c.X - outer - gap, c.Y, rowH, alignRight: true, apAfter, apNow, afterShort: resultShort);
        Column(in f, labels, c.X + outer + gap, c.Y, rowH, alignRight: false, peAfter, peNow, peInAir, resultShort);

        if (burnCci is double3 b && IsFinite(b))
        {
            OrbitShape burnShape = ShapeOf(result ?? labelled, in frame);
            float2 at = ScreenOf(b, in burnShape, in frame);
            Glyphs.Diamond(f.Dl, at, 6f * s, Palette.Plan.Solid);
            labels.Mark(at, 7f * s);
            labels.Reserve(new PicBox(c.X - bodyEdge, c.Y - bodyEdge, bodyEdge * 2f, bodyEdge * 2f));
            labels.Point(burnLabel, at, 9f * s, Palette.Plan.Solid);
        }

        Picture.HeadText(in f, lanes.Head, head, Palette.Text.Solid, big);
        Picture.BigNumber(in f, big, Palette.Text.Solid);

        int shown = 0;
        foreach (Picture.Swatch swatch in legend)
        {
            if ((swatch.Kind != SwatchKind.Dashed || dashedSeen) && shown < _shownLegend.Length - 1)
                _shownLegend[shown++] = swatch;
        }
        if (hasAir)
            _shownLegend[shown++] = new Picture.Swatch(PageText.AirKey, Palette.Air, SwatchKind.Filled);
        Picture.Legend(in f, lanes.Right, _shownLegend.AsSpan(0, shown));
    }

    private static int TrackPoints(Orbit orbit, in SketchFrame frame, double limit, float2[] pts)
    {
        OrbitShape shape = ShapeOf(orbit, in frame);
        (double from, double to) = SketchMath.DrawableAnomaly(orbit.Eccentricity);
        int count = 0;
        for (int i = 0; i <= Samples; i++)
        {
            double3 p = orbit.GetPositionCci(new TrueAnomaly(from + (to - from) * i / Samples));
            if (!IsFinite(p))
                continue;
            P2 flat = Proj(p, frame.U, frame.V);
            if (!shape.Closed && Math.Sqrt(flat.X * flat.X + flat.Y * flat.Y) > limit)
                continue;
            pts[count++] = ScreenOf(p, in shape, in frame);
        }
        return count;
    }

    private static bool DashedLineSeen(IReadOnlyList<Track> tracks, double radius, double top, double airRadius, float outer, float gap)
    {
        Span<double> solidAp = tracks.Count <= 16 ? stackalloc double[16] : new double[tracks.Count];
        Span<double> solidPe = tracks.Count <= 16 ? stackalloc double[16] : new double[tracks.Count];
        int solid = 0;
        foreach (Track t in tracks)
        {
            if (t.Dashed || !(t.Orbit.Eccentricity < 1.0))
                continue;
            solidAp[solid] = SketchMath.SchematicRadius(t.Orbit.Apoapsis, radius, top, airRadius: airRadius) * outer;
            solidPe[solid++] = SketchMath.SchematicRadius(t.Orbit.Periapsis, radius, top, airRadius: airRadius) * outer;
        }
        foreach (Track t in tracks)
        {
            if (!t.Dashed || !(t.Orbit.Eccentricity < 1.0))
                continue;
            double ap = SketchMath.SchematicRadius(t.Orbit.Apoapsis, radius, top, airRadius: airRadius) * outer;
            double pe = SketchMath.SchematicRadius(t.Orbit.Periapsis, radius, top, airRadius: airRadius) * outer;
            if (SketchMath.LineStandsClear(ap, solidAp.Slice(0, solid), gap) || SketchMath.LineStandsClear(pe, solidPe.Slice(0, solid), gap))
                return true;
        }
        return false;
    }

    public static void DrawTransfer(in PictureFrame f, Orbit source, Orbit target, IReadOnlyList<Track> tracks, double3? departCci, double3? arriveCci, ReadOnlySpan<double3> arc, bool arcIsEstimate = false)
    {
        IParentBody? parent = source.Parent;
        if (parent == null || !TryPlane(source, out double3 u, out double3 v))
        {
            Picture.Empty(in f, PageText.NoOrbit);
            return;
        }

        double radius = parent.MeanRadius;
        double minX = -radius, maxX = radius, minY = -radius, maxY = radius;
        double limit = Math.Max(radius * 3.0, MaxClosedRadius(tracks) * 1.4);
        var spans = new List<(int Start, int Count)>(tracks.Count);
        SampleTracks(tracks, u, v, double.PositiveInfinity, spans);
        foreach (P2 q in _world)
        {
            minX = Math.Min(minX, q.X);
            maxX = Math.Max(maxX, q.X);
            minY = Math.Min(minY, q.Y);
            maxY = Math.Max(maxY, q.Y);
        }
        minX = Math.Max(minX, -limit);
        maxX = Math.Min(maxX, limit);
        minY = Math.Max(minY, -limit);
        maxY = Math.Min(maxY, limit);

        float s = f.Scale;
        bool lane = departCci != null || arriveCci != null;
        float laneH = lane ? Picture.TextLaneHeight() : 0f;
        var fit = SketchMath.Fit(minX, maxX, minY, maxY, f.Min.X, f.Min.Y, f.Width, f.Height - laneH, 10.0 * s);
        float2 Screen(P2 p)
        {
            P2 q = SketchMath.ToScreen(p, fit);
            return new float2((float)q.X, (float)q.Y);
        }

        Labels labels = _labels;
        PicBox box = f.Box;
        labels.Begin(f.Dl, in box, s);
        f.Dl.AddCircleFilled(Screen(new P2(0.0, 0.0)), Math.Max(2f * s, (float)(radius * fit.Scale)), new ImColor8(40, 46, 56, 255));
        for (int k = 0; k < tracks.Count; k++)
        {
            (int start, int count) = spans[k];
            if (count < 2)
                continue;
            float2[] pts = _points.Get(count);
            for (int i = 0; i < count; i++)
                pts[i] = Screen(_world[start + i]);
            f.Dl.AddPolyline(pts.AsSpan(0, count), tracks[k].Colour, ImDrawFlags.None, tracks[k].Thickness * s);
            labels.Curve(pts.AsSpan(0, count));
        }
        if (arc.Length > 1)
        {
            float2[] pts = _points.Get(arc.Length);
            int m = 0;
            foreach (double3 p in arc)
            {
                if (IsFinite(p))
                    pts[m++] = Screen(Proj(p, u, v));
            }
            if (m > 1)
            {
                if (arcIsEstimate)
                    Glyphs.DashedPolyline(f.Dl, pts.AsSpan(0, m), Palette.Plan.A(0.6f), 1.5f * s, 5f * s, 3f * s);
                else
                    f.Dl.AddPolyline(pts.AsSpan(0, m), Palette.Plan.Solid, ImDrawFlags.None, 2f * s);
                labels.Curve(pts.AsSpan(0, m));
            }
        }

        if (departCci is double3 d && IsFinite(d))
        {
            float2 at = Screen(Proj(d, u, v));
            Glyphs.Diamond(f.Dl, at, 5f * s, Palette.Plan.Solid);
            labels.Mark(at, 6f * s);
        }
        if (arriveCci is double3 a && IsFinite(a))
        {
            float2 at = Screen(Proj(a, u, v));
            Glyphs.Ring(f.Dl, at, 5f * s, Palette.Target.Solid, 1.5f * s);
            labels.Mark(at, 6f * s);
        }
        if (lane)
        {
            var row = new PicBox(f.Min.X, f.Max.Y - laneH, f.Width, laneH);
            Picture.LegendRow(in f, in row, new Picture.Swatch[]
            {
                new("DEPART", Palette.Plan, SwatchKind.Diamond),
                new("ARRIVE", Palette.Target, SwatchKind.Ring),
            });
        }
    }

    /// <summary>
    /// The plane change edge on: the current plane horizontal, the plane after the burn turned by the
    /// </summary>
    public static void DrawPlane(in PictureFrame f, double changeDeg, bool descending, string nowLabel, string otherLabel, string big, string change, string head)
    {
        float s = f.Scale;
        PictureLanes lanes = Picture.Lanes(in f, Picture.HeadHeight(in f, big, text: head.Length > 0), 0f, 0f, Picture.TextLaneHeight());
        PicBox plot = lanes.Plot;
        var node = new float2(plot.X + plot.W * 0.5f, plot.Y + plot.H * 0.5f);
        float reach = plot.W * 0.36f;

        Labels labels = _labels;
        labels.Begin(f.Dl, in plot, s);
        float2 nowA = node - new float2(reach, 0f), nowB = node + new float2(reach, 0f);
        f.Dl.AddLine(nowA, nowB, Palette.Flown.Solid, 2f * s);
        labels.Curve(new[] { nowA, nowB });

        double shown = Math.Clamp(Math.Abs(changeDeg), 0.0, 80.0) * Math.PI / 180.0;
        // Small angles are drawn at least visible; the number says the real angle.
        double draw = Math.Max(shown, 3.0 * Math.PI / 180.0);
        var dir = new float2((float)Math.Cos(draw), -(float)Math.Sin(draw) * (changeDeg < 0 ? -1f : 1f));
        float otherH = labels.Size(otherLabel).Y;
        float afterReach = (float)SketchMath.PlaneLineReach(reach, plot.H * 0.5f - otherH - 6f * s, draw * 180.0 / Math.PI);
        float2 afterA = node - dir * afterReach, afterB = node + dir * afterReach;
        f.Dl.AddLine(afterA, afterB, Palette.Plan.Solid, 2f * s);
        labels.Curve(new[] { afterA, afterB });
        Glyphs.Diamond(f.Dl, node, 6f * s, Palette.Plan.Solid);
        labels.Mark(node, 7f * s);

        float2 otherSize = labels.Size(otherLabel), nowSize = labels.Size(nowLabel);
        float air = 4f * s;
        float sign = dir.Y < 0f ? 1f : -1f;
        ReadOnlySpan<float2> otherSpots = new[]
        {
            new float2(afterB.X - otherSize.X, sign > 0f ? afterB.Y - otherSize.Y - air : afterB.Y + air),
            new float2(afterB.X + air, afterB.Y - otherSize.Y * 0.5f),
            new float2(afterB.X - otherSize.X, sign > 0f ? afterB.Y + air : afterB.Y - otherSize.Y - air),
            new float2(afterA.X, sign > 0f ? afterA.Y + air : afterA.Y - otherSize.Y - air),
        };
        labels.Spots(otherLabel, otherSpots, Palette.Plan.Solid);
        ReadOnlySpan<float2> nowSpots = new[]
        {
            new float2(nowA.X, sign > 0f ? nowA.Y - nowSize.Y - air : nowA.Y + air),
            new float2(nowA.X, sign > 0f ? nowA.Y + air : nowA.Y - nowSize.Y - air),
            new float2(nowB.X - nowSize.X, sign > 0f ? nowB.Y + air : nowB.Y - nowSize.Y - air),
        };
        labels.Spots(nowLabel, nowSpots, Palette.Flown.Solid);

        Picture.HeadText(in f, lanes.Head, head, Palette.Text.Solid, big);
        Picture.BigNumber(in f, big, Palette.Text.Solid);
        Picture.LaneText(in f, lanes.Bottom, PageText.BurnAtNode(descending), Palette.Plan.Solid);
        Picture.LaneText(in f, lanes.Bottom, change, Palette.Dim.Solid, 1);
    }

    // The plane of an orbit: u to its periapsis, v ninety degrees on in the direction of travel.
    private static bool TryPlane(Orbit orbit, out double3 u, out double3 v)
    {
        u = Normalize(orbit.GetPositionCci(new TrueAnomaly(0.0)));
        double3 w = orbit.GetPositionCci(new TrueAnomaly(Math.PI * 0.5));
        v = Normalize(w - u * double3.Dot(w, u));
        return u.Length() > 0.5 && v.Length() > 0.5;
    }

    private static void SampleTracks(IReadOnlyList<Track> tracks, double3 u, double3 v, double limit, List<(int Start, int Count)> spans)
    {
        _world.Clear();
        foreach (Track t in tracks)
        {
            int start = _world.Count;
            (double from, double to) = SketchMath.DrawableAnomaly(t.Orbit.Eccentricity);
            for (int i = 0; i <= Samples; i++)
            {
                double nu = from + (to - from) * i / Samples;
                double3 p = t.Orbit.GetPositionCci(new TrueAnomaly(nu));
                if (!IsFinite(p))
                    continue;
                P2 q = SketchMath.Project(p.X, p.Y, p.Z, u.X, u.Y, u.Z, v.X, v.Y, v.Z);
                if (Math.Sqrt(q.X * q.X + q.Y * q.Y) > limit)
                    continue;
                _world.Add(q);
            }
            spans.Add((start, _world.Count - start));
        }
    }

    private static void ApsisTexts(Orbit orbit, double radius, out string ap, out string pe)
    {
        pe = orbit.Periapsis < radius ? PageText.SketchPeBelowGround : "PE " + Fmt.Km((orbit.Periapsis - radius) / 1000.0);
        ap = orbit.Eccentricity < 1.0 && double.IsFinite(orbit.Apoapsis) ? "AP" + Fmt.Km((orbit.Apoapsis - radius) / 1000.0) : PageText.Escapes;
    }

    private static void ApsisDots(in PictureFrame f, Orbit orbit, in SketchFrame frame, ImColor8 color, Labels labels)
    {
        float s = f.Scale;
        OrbitShape shape = ShapeOf(orbit, in frame);
        float2 pe = ScreenOf(orbit.GetPositionCci(new TrueAnomaly(0.0)), in shape, in frame);
        float2 ap = ScreenOf(orbit.GetPositionCci(new TrueAnomaly(Math.PI)), in shape, in frame);
        Glyphs.Dot(f.Dl, pe, 3f * s, color);
        Glyphs.Dot(f.Dl, ap, 3f * s, color);
        labels.Mark(pe, 4f * s);
        labels.Mark(ap, 4f * s);
    }

    private static void Column(in PictureFrame f, Labels labels, float x, float y, float rowH, bool alignRight, string after, string now, bool afterInAir = false, bool afterShort = false)
    {
        int rows = (after.Length > 0 ? 1 : 0) + (now.Length > 0 ? 1 : 0) + (afterInAir ? 1 : 0);
        float rowY = y - rows * rowH * 0.5f;
        if (after.Length > 0)
        {
            Row(labels, x, rowY, alignRight, after, afterInAir || afterShort ? Palette.Warn.Solid : Palette.Plan.Solid);
            rowY += rowH;
        }
        if (afterInAir)
        {
            Row(labels, x, rowY, alignRight, PageText.PeInAir, Palette.Warn.Solid);
            rowY += rowH;
        }
        if (now.Length > 0)
            Row(labels, x, rowY, alignRight, now, Palette.Dim.Solid);
    }

    private static void Row(Labels labels, float x, float y, bool alignRight, string text, ImColor8 color)
    {
        float2 size = labels.Size(text);
        labels.Spots(text, new[] { new float2(alignRight ? x - size.X : x, y) }, color);
    }

    private static double MaxClosedRadius(IReadOnlyList<Track> tracks)
    {
        double r = 0.0;
        foreach (Track t in tracks)
        {
            if (t.Orbit.Eccentricity < 1.0 && double.IsFinite(t.Orbit.Apoapsis))
                r = Math.Max(r, t.Orbit.Apoapsis);
            else if (double.IsFinite(t.Orbit.Periapsis))
                r = Math.Max(r, t.Orbit.Periapsis * 2.0);
        }
        return r;
    }

    // Unlike double3.Normalize, a zero or non-finite vector gives zero instead of throwing or NaN.
    private static double3 Normalize(double3 a)
    {
        double l = a.Length();
        return l > 0.0 && double.IsFinite(l) ? a / l : default;
    }

    private static bool IsFinite(double3 p) => double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Z);

    private static P2 Proj(double3 p, double3 u, double3 v) => SketchMath.Project(p.X, p.Y, p.Z, u.X, u.Y, u.Z, v.X, v.Y, v.Z);
}

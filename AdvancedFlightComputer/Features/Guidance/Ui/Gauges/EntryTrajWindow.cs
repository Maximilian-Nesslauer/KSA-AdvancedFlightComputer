#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Guidance.Scvx.OrbitalEntry;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

/// <summary>
/// ENTRY TRAJ: orbital entry guidance's picture, after the Shuttle's ENTRY TRAJ pages. It is the picture of LAND > ENTRY > ORBITAL ENTRY (DrawOrbitalEntryPicture), and pops out into a window of its own while the entry flies, display only: the commit, the stop and the settings are the console's.
///
///  TOP: range to go across, speed down the page (fast at the top), so the craft's bug moves down and to the left as it slows and closes. The plan is the nominal line. Either side of it are the corridor's edges, recomputed with every plan: the HIGH-ENERGY line, the range left if the craft flew the most drag its limits allow from that speed on, and the LOW-ENERGY line, the range left on the equilibrium glide at the lateral-reserve bank. A bug left of the high-energy line is too close to stop short of the target; right of the low-energy line, too far to reach it. Five trailers mark the bug every 30 s.
///  BOTTOM: drag against speed. The plan's reference drag, the corridor (the most drag the limits allow, the least the craft can hold), and the drag measured from the game's own aero force, with the current drag and reference marked as on the Shuttle's drag scale.
///  The window shows while orbital entry guidance flies, until it is closed; a new start of the entry, or POP OUT on the picture, opens it again.
/// </summary>
public static partial class GuidanceWindow
{
    private const string EntryTrajId = "afc-entry-traj";
    private static bool _entryTrajClosed;

    private static void DrawEntryTrajWindow(Vehicle vehicle, IParentBody parent)
    {
        using var _perf = new AdvancedFlightComputer.Core.PerfTracker.Scope("Ui.EntryTraj");
        var st = _s.OrbitalEntry;
        // Closed for this entry only: the next one, also one that DEORBIT ONLY starts by itself, opens it again.
        if (!OrbitalEntryLive)
            _entryTrajClosed = false;
        if (!OrbitalEntryLive || _entryTrajClosed)
            return;

        ConsoleStyle.EnsureInitialized();
        float s = ImGuiHelper.InterfaceScale;
        bool open = true;
        if (!ConsoleStyle.BeginWindow(EntryTrajId.AsSpan(), "ENTRY TRAJ".AsSpan(), "AFC".AsSpan(), ref open,
                new float2(620f * s, 680f * s), ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoScrollWithMouse,
                ImGuiCond.FirstUseEver, pinToMainViewport: false))
            return;
        try
        {
            if (!open)
                _entryTrajClosed = true;
            ConsoleLayout layout = ConsoleStyle.CurrentLayout;
            float pad = 8f * s;
            // The body, inside the window: everything below is placed within it and nothing is drawn past its edges.
            float2 min = new float2(layout.BodyPos.X + pad, layout.BodyPos.Y + pad);
            float2 max = new float2(layout.WindowPos.X + layout.WindowSize.X - pad, layout.WindowPos.Y + layout.WindowSize.Y - pad);
            float width = max.X - min.X, height = max.Y - min.Y;
            if (width < 240f * s || height < 160f * s)
            {
                ImGui.SetCursorScreenPos(min);
                ImGui.Text("Make the window larger.");
                return;
            }
            float line = ImGui.GetFontSize() + 3f * s;

            // ---- the status, wrapped over the width ----
            ImGui.SetCursorScreenPos(min);
            ImGui.PushTextWrapPos(ImGui.GetCursorPosX() + Math.Max(40f, width));
            ImGui.PushStyleColor(ImGuiCol.Text, st.StatusWarn ? Palette.Warn.Solid : Palette.Text.Solid);
            ImGui.TextWrapped(st.Status.Replace("%", "%%"));
            ImGui.PopStyleColor();
            ImGui.PopTextWrapPos();
            float y = Math.Max(ImGui.GetCursorScreenPos().Y, min.Y + line) + 4f * s;
            DrawEntryTrajBody(ImGui.GetWindowDrawList(), st, new float2(min.X, y), max, line, s);
        }
        finally
        {
            ConsoleStyle.EndWindow();
        }
    }

    /// <summary>The plots over the readouts, in the box from <paramref name="min"/> to <paramref name="max"/>: two columns of readouts where the width allows, one where it does not, and the plots in what is left above them.</summary>
    private static void DrawEntryTrajBody(ImDrawListPtr dl, OrbitalEntryState st, float2 min, float2 max, float line, float s)
    {
        float width = max.X - min.X;
        EntryPlan plan = st.Plan;
        string[] read = EntryReadouts(st, plan);
        bool twoColumns = width >= 560f * s;
        int rows = twoColumns ? (read.Length + 1) / 2 : read.Length;
        float readH = rows * line + 4f * s;
        float plotsTop = min.Y, plotsBottom = max.Y - readH - 8f * s;
        if (plotsBottom - plotsTop >= 140f * s)
            DrawTrajPlot(dl, st, plan, new float2(min.X, plotsTop), new float2(max.X, plotsBottom), s);
        else
            dl.AddText(new float2(min.X, plotsTop), Palette.Dim.Solid, "Make the picture taller to see the plots.");
        float ry = max.Y - readH + 2f * s;
        for (int i = 0; i < read.Length; i++)
        {
            int col = twoColumns ? i / rows : 0, row = twoColumns ? i % rows : i;
            ImColor8 c = col == 0 ? Palette.Text.Solid : Palette.Dim.Solid;
            float x = min.X + col * width * 0.5f;
            dl.AddText(new float2(x, ry + row * line), c, Fit(read[i], twoColumns ? width * 0.5f - 6f * s : width));
        }
    }

    /// <summary>The ENTRY TRAJ plot inside a console picture frame, the pop-out window's drawing. The readouts stand under the picture as rows of the form (DrawOrbitalEntryContent), so the plot keeps the height.</summary>
    private static void DrawEntryTrajInFrame(in PictureFrame f)
        => DrawTrajPlot(f.Dl, _s.OrbitalEntry, _s.OrbitalEntry.Plan, f.Min, f.Max, ImGuiHelper.InterfaceScale);

    /// <summary>POP OUT on the picture: the ENTRY TRAJ window again, after it was closed.</summary>
    private static bool EntryTrajWindowShown => OrbitalEntryLive && !_entryTrajClosed;

    private static void ToggleEntryTrajWindow() => _entryTrajClosed = !_entryTrajClosed;

    /// <summary>The text cut to what fits in the width, with an ellipsis.</summary>
    private static string Fit(string text, float width)
    {
        if (ImGui.CalcTextSize(text).X <= width)
            return text;
        int n = text.Length;
        while (n > 4 && ImGui.CalcTextSize(text.AsSpan(0, n).ToString() + "...").X > width)
            n--;
        return text.Substring(0, n) + "...";
    }

    // ---------------------------------------------------------------- a plot area

    /// <summary>
    /// One plot's frame and its data-to-pixel mapping: x and y from their ranges, clamped to the frame, so nothing is drawn outside it, and a segment with any coordinate not finite is not drawn at all.
    /// </summary>
    private readonly struct PlotArea
    {
        public readonly float2 P0, P1;
        readonly double _x0, _x1, _y0, _y1;

        public PlotArea(float2 p0, float2 p1, double x0, double x1, double y0, double y1)
        {
            P0 = p0; P1 = p1;
            _x0 = x0; _x1 = x1; _y0 = y0; _y1 = y1;
            if (!(Math.Abs(_x1 - _x0) > 1e-9)) _x1 = _x0 + 1.0;
            if (!(Math.Abs(_y1 - _y0) > 1e-9)) _y1 = _y0 + 1.0;
        }

        public bool Map(double x, double y, out float2 at)
        {
            at = default;
            if (!double.IsFinite(x) || !double.IsFinite(y))
                return false;
            double fx = Math.Clamp((x - _x0) / (_x1 - _x0), 0.0, 1.0);
            double fy = Math.Clamp((y - _y0) / (_y1 - _y0), 0.0, 1.0);
            at = new float2(P0.X + (P1.X - P0.X) * (float)fx, P1.Y - (P1.Y - P0.Y) * (float)fy);
            return true;
        }

        public void Line(ImDrawListPtr dl, double xa, double ya, double xb, double yb, ImColor8 col, float thick)
        {
            if (Map(xa, ya, out float2 a) && Map(xb, yb, out float2 b))
                dl.AddLine(a, b, col, thick);
        }

        /// <summary>A segment cut to the area (Liang-Barsky), so a page shows only its own part of a line rather than the rest pressed along its edges.</summary>
        public void ClippedLine(ImDrawListPtr dl, double xa, double ya, double xb, double yb, ImColor8 col, float thick)
        {
            if (!double.IsFinite(xa) || !double.IsFinite(ya) || !double.IsFinite(xb) || !double.IsFinite(yb))
                return;
            double lo = Math.Min(_x0, _x1), hi = Math.Max(_x0, _x1), ylo = Math.Min(_y0, _y1), yhi = Math.Max(_y0, _y1);
            double t0 = 0, t1 = 1, dx = xb - xa, dy = yb - ya;
            if (!Clip(-dx, xa - lo, ref t0, ref t1) || !Clip(dx, hi - xa, ref t0, ref t1)
                || !Clip(-dy, ya - ylo, ref t0, ref t1) || !Clip(dy, yhi - ya, ref t0, ref t1))
                return;
            Line(dl, xa + t0 * dx, ya + t0 * dy, xa + t1 * dx, ya + t1 * dy, col, thick);
        }

        static bool Clip(double pp, double q, ref double t0, ref double t1)
        {
            if (pp == 0) return q >= 0;
            double t = q / pp;
            if (pp < 0) { if (t > t1) return false; if (t > t0) t0 = t; }
            else { if (t < t0) return false; if (t < t1) t1 = t; }
            return true;
        }

        public bool Inside(double x, double y)
            => x >= Math.Min(_x0, _x1) && x <= Math.Max(_x0, _x1) && y >= Math.Min(_y0, _y1) && y <= Math.Max(_y0, _y1);
    }

    /// <summary>How many pages the range-to-go plot splits the plan into, the Shuttle's ENTRY TRAJ way.</summary>
    private const int TrajPages = 4;

    /// <summary>
    /// The Mach band of each page: the plan's Mach where its range to go has fallen by a quarter, a half and three quarters. Each page then spans a like share of the range, and early on, where the speed hardly falls over thousands of km, its own few Mach fill the height instead of a sliver at the top.
    /// </summary>
    private static double[] TrajPageEdges(EntryPlan plan)
    {
        var edges = new double[TrajPages + 1];
        double s0 = plan.S[0], sF = plan.S[^1];
        edges[0] = plan.Mach[0];
        edges[TrajPages] = plan.Mach[^1];
        int k = 0;
        for (int page = 1; page < TrajPages; page++)
        {
            double sCut = s0 + (sF - s0) * page / TrajPages;
            while (k < plan.Count - 2 && plan.S[k + 1] > sCut) k++;
            double ds = plan.S[k + 1] - plan.S[k];
            double f = ds != 0 ? Math.Clamp((sCut - plan.S[k]) / ds, 0.0, 1.0) : 0.0;
            edges[page] = plan.Mach[k] + f * (plan.Mach[k + 1] - plan.Mach[k]);
        }
        return edges;
    }

    /// <summary>
    /// The pages, fixed for the entry as the Shuttle's pre-printed ENTRY TRAJ pages were: built from the first plan the window sees and kept until the entry is reset. The axes and the corridor's high- and low-energy lines are that plan's, so they stay where they are; the bug, its trailers, the flown path and the current plan move across them, in at the top of page 1. Whatever leaves a page is cut at its edge, and the bug pegs there.
    /// </summary>
    private sealed class TrajLayout
    {
        public readonly double[] Edges = new double[TrajPages + 1];   // the pages' Mach bands, fast to slow
        public readonly double[] Top = new double[TrajPages], Bottom = new double[TrajPages];
        public readonly double[] XMin = new double[TrajPages], XMax = new double[TrajPages];
        public double[] Mach, RangeShortM, RangeLongM;               // the corridor, as the first plan drew it

        public int PageAt(double mach)
        {
            int page = 0;
            while (page < TrajPages - 1 && mach < Edges[page + 1]) page++;
            return page;
        }
    }

    /// <summary>The entry's fixed layout, kept on its state (a reset clears it), built at the first plan.</summary>
    private static TrajLayout LayoutFor(OrbitalEntryState st, EntryPlan plan)
    {
        if (st.TrajLayout is TrajLayout kept)
            return kept;
        if (plan == null || plan.Count < 2)
            return null;
        var built = BuildLayout(plan);
        st.TrajLayout = built;
        return built;
    }

    private static TrajLayout BuildLayout(EntryPlan plan)
    {
        var L = new TrajLayout
        {
            Mach = (double[])plan.Mach.Clone(),
            RangeShortM = (double[])plan.RangeShortM.Clone(),
            RangeLongM = (double[])plan.RangeLongM.Clone(),
        };
        double[] edges = TrajPageEdges(plan);
        Array.Copy(edges, L.Edges, edges.Length);
        double r0 = plan.BodyRadius;
        for (int page = 0; page < TrajPages; page++)
        {
            double hi = edges[page], lo = edges[page + 1];
            double mPad = 0.05 * Math.Max(hi - lo, 0.2);
            // Page 1 reaches above the plan's start, so the bug comes in from the top while the craft coasts down to it.
            L.Top[page] = hi + (page == 0 ? Math.Max(mPad, 0.015 * hi) : mPad);
            L.Bottom[page] = Math.Max(0, lo - mPad);

            double xLo = double.PositiveInfinity, xHi = double.NegativeInfinity;
            FitBand(ref xLo, ref xHi, i => plan.S[i] * r0, i => plan.Mach[i], plan.Count, L.Bottom[page], L.Top[page]);
            FitBand(ref xLo, ref xHi, i => plan.RangeShortM[i], i => plan.Mach[i], plan.Count, L.Bottom[page], L.Top[page]);
            if (double.IsFinite(xHi))
            {
                // The low-energy edge runs out far beyond: show it to about twice the band's span past the plan.
                double longLo = double.PositiveInfinity, longHi = double.NegativeInfinity;
                FitBand(ref longLo, ref longHi, i => plan.RangeLongM[i], i => plan.Mach[i], plan.Count, L.Bottom[page], L.Top[page]);
                if (double.IsFinite(longHi)) xHi = Math.Max(xHi, Math.Min(longHi, xHi + 1.5 * Math.Max(xHi - xLo, 0.25 * Math.Abs(xHi))));
            }
            if (!double.IsFinite(xLo) || !double.IsFinite(xHi)) { xLo = 0; xHi = Math.Max(plan.S[0] * r0, 1000.0); }
            // Room for later plans to move, and for the bug to wander, before anything meets an edge.
            double xPad = 0.12 * Math.Max(xHi - xLo, 1000.0);
            double xMin = xLo >= 0 ? Math.Max(0, xLo - xPad) : xLo - xPad;
            double xMax = xHi + xPad;
            double step = NiceCeiling((xMax - xMin) / 4.0);
            if (step > 0) { xMin = Math.Floor(xMin / step) * step; xMax = Math.Ceiling(xMax / step) * step; }
            L.XMin[page] = xMin;
            L.XMax[page] = xMax;
        }
        return L;
    }

    /// <summary>Widens [lo, hi] to the x of a polyline's part inside the band in y, its crossings of the band's edges included.</summary>
    private static void FitBand(ref double lo, ref double hi, Func<int, double> x, Func<int, double> v, int count, double vLo, double vHi)
    {
        for (int i = 0; i < count; i++)
        {
            double xi = x(i), vi = v(i);
            if (!double.IsFinite(xi) || !double.IsFinite(vi)) continue;
            if (vi >= vLo && vi <= vHi) { lo = Math.Min(lo, xi); hi = Math.Max(hi, xi); }
            if (i == 0) continue;
            double xp = x(i - 1), vp = v(i - 1);
            if (!double.IsFinite(xp) || !double.IsFinite(vp) || vp == vi) continue;
            for (int j = 0; j < 2; j++)
            {
                double edge = j == 0 ? vLo : vHi;
                if ((vp - edge) * (vi - edge) < 0)
                {
                    double xe = xp + (edge - vp) / (vi - vp) * (xi - xp);
                    lo = Math.Min(lo, xe); hi = Math.Max(hi, xe);
                }
            }
        }
    }

    private static double Finite(double v, double fallback) => double.IsFinite(v) ? v : fallback;

    // ---------------------------------------------------------------- range to go against Mach

    private static void DrawTrajPlot(ImDrawListPtr dl, OrbitalEntryState st, EntryPlan plan, float2 a, float2 b, float s)
    {
        float left = 50f * s, bottom = 16f * s, top = ImGui.GetFontSize() + 4f * s;
        float2 p0 = new float2(a.X + left, a.Y + top), p1 = new float2(b.X - 4f * s, b.Y - bottom);
        dl.AddText(new float2(a.X + left, a.Y), Palette.Dim.Solid, "UP: MACH   ACROSS: RANGE TO GO, km   BOX: DRAG ERROR");
        if (p1.Y - p0.Y < 30f * s)
            return;
        Frame(dl, p0, p1);

        // The page holding the craft's Mach, on the entry's fixed axes.
        var tr = st.Trace;
        TrajLayout L = LayoutFor(st, plan);
        double xMin, xMax, mMin, mMax;
        if (L != null)
        {
            int page = L.PageAt(Finite(st.Mach, L.Edges[0]));
            xMin = L.XMin[page]; xMax = L.XMax[page]; mMin = L.Bottom[page]; mMax = L.Top[page];
            string pageName = FormattableString.Invariant($"PAGE {page + 1}/{TrajPages}");
            dl.AddText(new float2(p1.X - ImGui.CalcTextSize(pageName).X, a.Y), Palette.Dim.Solid, pageName);
        }
        else
        {
            xMin = 0; xMax = NiceCeiling(Math.Max(Finite(st.RangeM, 0), 1000.0) * 1.1);
            mMin = 0; mMax = Math.Max(Finite(st.Mach, 0), 1.0) * 1.02;
        }

        // Range across (low at the left), Mach up the side (fast at the top).
        var area = new PlotArea(p0, p1, xMin, xMax, mMin, mMax);
        AxisLabels(dl, p0, p1, s, $"{xMax / 1000:F0} km", $"{xMin / 1000:F0}", $"M{mMax:F1}", $"M{mMin:F1}");

        if (L != null)
        {
            // The corridor as the first plan drew it: fixed, as the Shuttle's printed lines were.
            ImColor8 edge = Palette.Text.Solid;
            double labelBelow = mMax - 0.3 * (mMax - mMin);
            int hiLabel = -1, loLabel = -1;
            for (int k = 0; k < L.Mach.Length - 1; k++)
            {
                area.ClippedLine(dl, L.RangeShortM[k], L.Mach[k], L.RangeShortM[k + 1], L.Mach[k + 1], edge, 2f * s);
                area.ClippedLine(dl, L.RangeLongM[k], L.Mach[k], L.RangeLongM[k + 1], L.Mach[k + 1], edge, 2f * s);
                if (hiLabel < 0 && L.Mach[k] < labelBelow && area.Inside(L.RangeShortM[k], L.Mach[k])) hiLabel = k;
                if (loLabel < 0 && L.Mach[k] < labelBelow && area.Inside(L.RangeLongM[k], L.Mach[k])) loLabel = k;
            }
            if (hiLabel >= 0 && area.Map(L.RangeShortM[hiLabel], L.Mach[hiLabel], out float2 hi))
                dl.AddText(new float2(Math.Max(p0.X + 2f * s, hi.X - 84f * s), hi.Y), Palette.Dim.Solid, "HIGH ENERGY");
            if (loLabel >= 0 && area.Map(L.RangeLongM[loLabel], L.Mach[loLabel], out float2 lo))
                dl.AddText(new float2(Math.Min(lo.X + 6f * s, p1.X - 80f * s), lo.Y), Palette.Dim.Solid, "LOW ENERGY");
        }
        if (plan != null)
        {
            double r0 = plan.BodyRadius;
            for (int k = 0; k < plan.Count - 1; k++)
                area.ClippedLine(dl, plan.S[k] * r0, plan.Mach[k], plan.S[k + 1] * r0, plan.Mach[k + 1], Palette.Plan.Solid, 2.5f * s);
        }

        // The flown path, and a trailer every 30 s for the last 150 s.
        for (int i = 1; i < tr.Count; i++)
            area.ClippedLine(dl, tr[i - 1].RangeM, tr[i - 1].Mach, tr[i].RangeM, tr[i].Mach, Palette.Flown.Solid, 1.2f * s);
        for (int n = 1; n <= 5; n++)
        {
            int i = tr.Count - 1 - 30 * n;
            if (i < 0) break;
            if (area.Inside(tr[i].RangeM, tr[i].Mach) && area.Map(tr[i].RangeM, tr[i].Mach, out float2 at))
                Bug(dl, at, 4f * s, Palette.Flown.Solid, filled: false);
        }
        if (st.Mach > 0 && area.Map(st.RangeM, st.Mach, out float2 now))
        {
            DragErrorBox(dl, st, now, p0, p1, s);
            Bug(dl, now, 6f * s, Palette.Target.Solid, filled: true);
        }
    }

    /// <summary>A tenth of the drag off its reference moves the guidance box a tenth of the plot's width.</summary>
    private const float DragBoxWidthPerError = 1.0f;

    /// <summary>
    /// The Shuttle's guidance box: a square at the bug's height, set off sideways by the drag error, so the bug sits in it when the drag is on its reference. Drag above the reference is the high-energy (short) side, left on this page, so the bug then sits left of the box; below it, right. The offset is the error as a share of the reference drag (floored at 1 m/s²), a tenth of the width per tenth of the drag, pegged at the frame.
    /// </summary>
    private static void DragErrorBox(ImDrawListPtr dl, OrbitalEntryState st, float2 bug, float2 p0, float2 p1, float s)
    {
        // The error smoothed over DragErrorSmoothS (OrbitalEntry), so the box shows the trend rather than every flutter of the measured drag.
        double dRef = st.Last.DragRefMs2, err = st.DragErrorF;
        if (!(dRef > 0) || !double.IsFinite(err) || st.Last.Phase != EntryTracker.Phase.Tracking)
            return;
        double share = err / Math.Max(dRef, 1.0);
        float half = 7f * s;
        float x = bug.X + (float)(share * DragBoxWidthPerError * (p1.X - p0.X));
        x = Math.Clamp(x, p0.X + half, p1.X - half);
        var c = Palette.Plan.Solid;
        dl.AddRect(new float2(x - half, bug.Y - half), new float2(x + half, bug.Y + half), c, 0f, ImDrawFlags.None, 1.5f * s);
        if (Math.Abs(x - bug.X) > half)
            dl.AddLine(new float2(bug.X, bug.Y), new float2(x + (x > bug.X ? -half : half), bug.Y), c, 1f * s);
        string label = FormattableString.Invariant($"{err:+0.0;-0.0}");
        float tw = ImGui.CalcTextSize(label).X;
        float lx = x > bug.X ? Math.Min(x + half + 3f * s, p1.X - tw) : Math.Max(x - half - 3f * s - tw, p0.X);
        dl.AddText(new float2(lx, bug.Y - ImGui.GetFontSize() * 0.5f), c, label);
    }

    // ---------------------------------------------------------------- numbers

    private static string[] EntryReadouts(OrbitalEntryState st, EntryPlan plan)
    {
        var o = st.Last;
        return new[]
        {
            FormattableString.Invariant($"DRAG {st.DragMs2:F1}  REF {o.DragRefMs2:F1}  ERR {st.DragErrorF:+0.0;-0.0} m/s²"),
            FormattableString.Invariant($"HDOT {st.HdotMs:F0}  REF {o.HdotRefMs:F0} m/s"),
            FormattableString.Invariant($"BANK {(o.BankDeg >= 0 ? "R" : "L")}{Math.Abs(o.BankDeg):F0}  PLAN {o.BankRefDeg:F0}°{(o.Overbanked ? "  OVERBANK" : "")}"),
            FormattableString.Invariant($"AoA {st.AlphaFlownDeg:F0}° (ASKED {st.AlphaDeg:F0})  MACH {st.Mach:F1}"),
            FormattableString.Invariant($"ALT {st.AltitudeM / 1000:F1} km  RANGE {st.RangeM / 1000:F0} km"),
            FormattableString.Invariant($"CROSS {st.CrossrangeM / 1000:F1} km  HDG ERR {st.HeadingErrorDeg:+0.0;-0.0}°  BAND ±{o.DeadbandRad * 180.0 / Math.PI:F1}°"),
            FormattableString.Invariant($"L/D {st.LiftToDrag:F2} (x{st.LiftToDragScale:F2})  AIR x{st.DensityScale:F2}"),
            FormattableString.Invariant($"BELLY {st.BellyAxisName}  TRIM {(st.Trim != null ? st.Trim.TrimmedRows + "/" + st.Trim.Mach.Length : "-")}"),
            FormattableString.Invariant($"PLANS {st.Solves} ({st.Refusals} REFUSED)  {st.LastSolveMs:F0} ms"),
            plan != null ? FormattableString.Invariant($"MISS {plan.TerminalRangeErrorM / 1000:F1} km  DEFECT {plan.MaxDefect:F3}") : "NO PLAN YET",
            FormattableString.Invariant($"REVERSALS {st.Reversals}  {st.Phase.ToString().ToUpperInvariant()}"),
        };
    }

    // ---------------------------------------------------------------- bits

    private static void Frame(ImDrawListPtr dl, float2 p0, float2 p1)
    {
        ImColor8 c = Palette.Grid.Solid;
        dl.AddLine(p0, new float2(p1.X, p0.Y), c, 1f);
        dl.AddLine(new float2(p1.X, p0.Y), p1, c, 1f);
        dl.AddLine(p1, new float2(p0.X, p1.Y), c, 1f);
        dl.AddLine(new float2(p0.X, p1.Y), p0, c, 1f);
    }

    private static void AxisLabels(ImDrawListPtr dl, float2 p0, float2 p1, float s, string xRight, string xLeft, string yTop, string yBottom)
    {
        ImColor8 c = Palette.Dim.Solid;
        float fh = ImGui.GetFontSize();
        dl.AddText(new float2(p0.X, p1.Y + 1f * s), c, xLeft);
        float w = ImGui.CalcTextSize(xRight).X;
        dl.AddText(new float2(p1.X - w, p1.Y + 1f * s), c, xRight);
        dl.AddText(new float2(p0.X - ImGui.CalcTextSize(yTop).X - 3f * s, p0.Y), c, yTop);
        dl.AddText(new float2(p0.X - ImGui.CalcTextSize(yBottom).X - 3f * s, p1.Y - fh), c, yBottom);
    }

    private static void Bug(ImDrawListPtr dl, float2 at, float r, ImColor8 col, bool filled)
    {
        float2 a = at + new float2(0, -r), b = at + new float2(r, r * 0.8f), c = at + new float2(-r, r * 0.8f);
        if (filled)
            dl.AddTriangleFilled(a, b, c, col);
        else
        {
            dl.AddLine(a, b, col, 1.5f);
            dl.AddLine(b, c, col, 1.5f);
            dl.AddLine(c, a, col, 1.5f);
        }
    }
}

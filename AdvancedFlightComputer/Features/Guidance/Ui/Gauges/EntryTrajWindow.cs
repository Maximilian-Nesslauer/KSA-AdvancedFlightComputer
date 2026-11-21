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
/// ENTRY TRAJ: orbital entry guidance's own window, after the Shuttle's ENTRY TRAJ pages.
///
///  TOP: range to go across, speed down the page (fast at the top), so the craft's bug moves down and to the left as it slows and closes. The plan is the nominal line. Either side of it are the corridor's edges, recomputed with every plan: the HIGH-ENERGY line, the range left if the craft flew the most drag its limits allow from that speed on, and the LOW-ENERGY line, the range left on the equilibrium glide at the lateral-reserve bank. A bug left of the high-energy line is too close to stop short of the target; right of the low-energy line, too far to reach it. Five trailers mark the bug every 30 s.
///  BOTTOM: drag against speed. The plan's reference drag, the corridor (the most drag the limits allow, the least the craft can hold), and the drag measured from the game's own aero force, with the current drag and reference marked as on the Shuttle's drag scale.
///  It shows while orbital entry guidance is engaged, and while the craft is on a path into the air above the hand-over, so the player can engage it.
/// </summary>
public static partial class GuidanceWindow
{
    private const string EntryTrajId = "afc-entry-traj";
    private static bool _entryTrajClosed;
    private static bool _entryTrajParams;

    /// <summary>Whether the craft is on its way into the air from above: periapsis inside the atmosphere, still faster than the hand-over.</summary>
    private static bool OnEntryPath(Vehicle vehicle, IParentBody parent)
    {
        var atm = parent?.GetPhysicalAtmosphere();
        if (atm == null || !(atm.Height > 0.0))
            return false;
        Orbit orbit = vehicle.Orbit;
        double3 r = orbit.StateVectors.PositionCci;
        double alt = r.Length() - parent.MeanRadius;
        double v = (orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), r)).Length();
        double pe = orbit.Periapsis - parent.MeanRadius;
        return alt > _s.OrbitalEntryHandoverAltKm * 1000.0 + 5000.0 && pe < atm.Height && v > 1500.0;
    }

    private static void DrawEntryTrajWindow(Vehicle vehicle, IParentBody parent)
    {
        var st = _s.OrbitalEntry;
        bool live = OrbitalEntryLive || st.EngagePending;
        if (!live && !(OnEntryPath(vehicle, parent) && !_entryTrajClosed))
        {
            if (!OnEntryPath(vehicle, parent))
                _entryTrajClosed = false;
            return;
        }

        ConsoleStyle.EnsureInitialized();
        float s = ImGuiHelper.InterfaceScale;
        bool open = true;
        if (!ConsoleStyle.BeginWindow(EntryTrajId.AsSpan(), "ENTRY TRAJ".AsSpan(), "AFC".AsSpan(), ref open,
                new float2(600f * s, 640f * s), ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoScrollWithMouse,
                ImGuiCond.FirstUseEver, pinToMainViewport: false))
            return;
        try
        {
            if (!open && !live)
                _entryTrajClosed = true;
            ConsoleLayout layout = ConsoleStyle.CurrentLayout;
            float pad = 8f * s;
            float2 min = new float2(layout.BodyPos.X + pad, layout.BodyPos.Y + pad);
            float2 max = new float2(layout.WindowPos.X + layout.WindowSize.X - pad, layout.WindowPos.Y + layout.WindowSize.Y - pad);
            if (max.X - min.X < 200f || max.Y - min.Y < 200f)
                return;
            ImDrawListPtr dl = ImGui.GetWindowDrawList();
            float line = ImGui.GetFontSize() + 3f * s;

            // ---- status and buttons ----
            ImColor8 statusCol = st.StatusWarn ? Palette.Warn.Solid : Palette.Text.Solid;
            string status = st.Status.Length > 0 ? st.Status : (live ? "" : "ON AN ENTRY PATH. ENGAGE TO FLY IT.");
            dl.AddText(min, statusCol, status);
            ImGui.SetCursorScreenPos(new float2(max.X - 90f * s, min.Y - 2f * s));
            if (live)
            {
                if (ImGui.SmallButton("STOP##afc-oe-stop"u8))
                    st.StopPending = true;
            }
            else if (ImGui.SmallButton("ENGAGE##afc-oe-engage"u8))
                st.EngagePending = true;
            ImGui.SetCursorScreenPos(new float2(max.X - 170f * s, min.Y - 2f * s));
            if (ImGui.SmallButton(_entryTrajParams ? "PLOTS##afc-oe-params"u8 : "PARAMS##afc-oe-params"u8))
                _entryTrajParams = !_entryTrajParams;
            float y = min.Y + line * 1.4f;
            if (_entryTrajParams)
            {
                DrawEntryParams(new float2(min.X, y), max.X - min.X, line);
                return;
            }

            EntryPlan plan = st.Plan;
            float readH = line * 6.2f;
            float plotsH = max.Y - y - readH;
            float trajH = plotsH * 0.56f, dragH = plotsH - trajH - pad;
            DrawTrajPlot(dl, st, plan, new float2(min.X, y), new float2(max.X, y + trajH), s);
            DrawDragPlot(dl, st, plan, new float2(min.X, y + trajH + pad), new float2(max.X, y + trajH + pad + dragH), s);
            DrawEntryReadouts(dl, st, plan, new float2(min.X, max.Y - readH + line * 0.3f), max.X - min.X, line);
        }
        finally
        {
            ConsoleStyle.EndWindow();
        }
    }

    // ---------------------------------------------------------------- range to go against speed

    private static void DrawTrajPlot(ImDrawListPtr dl, OrbitalEntryState st, EntryPlan plan, float2 a, float2 b, float s)
    {
        float left = 46f * s, bottom = 16f * s;
        float2 p0 = new float2(a.X + left, a.Y), p1 = new float2(b.X, b.Y - bottom);
        Frame(dl, p0, p1);
        dl.AddText(new float2(p0.X + 4f * s, p0.Y + 2f * s), Palette.Dim.Solid, "RANGE TO GO vs SPEED");

        double vMax = Math.Max(st.SpeedMs, 1000.0), vMin = 0.0, xMax = Math.Max(st.RangeM, 1000.0);
        if (plan != null)
        {
            vMax = Math.Max(vMax, plan.SpeedMs[0]);
            vMin = plan.SpeedMs[^1];
            xMax = Math.Max(xMax, plan.S[0] * plan.BodyRadius);
            xMax = Math.Max(xMax, Math.Min(plan.RangeLongM[0], 3.0 * plan.S[0] * plan.BodyRadius));
        }
        vMax *= 1.02;
        xMax = NiceCeiling(xMax * 1.05);
        float2 P(double rangeM, double speed) => new float2(
            p0.X + (p1.X - p0.X) * (float)Math.Clamp(rangeM / xMax, -0.02, 1.02),
            p0.Y + (p1.Y - p0.Y) * (float)Math.Clamp((vMax - speed) / Math.Max(vMax - vMin, 1.0), -0.02, 1.02));

        AxisLabels(dl, p0, p1, s, $"{xMax / 1000:F0} km", "0", $"{vMax / 1000:F1} km/s", $"{vMin / 1000:F1}");

        if (plan != null)
        {
            ImColor8 edge = Palette.Text.Solid;
            for (int k = 0; k < plan.Count - 1; k++)
            {
                dl.AddLine(P(plan.RangeShortM[k], plan.SpeedMs[k]), P(plan.RangeShortM[k + 1], plan.SpeedMs[k + 1]), edge, 2f * s);
                dl.AddLine(P(Math.Min(plan.RangeLongM[k], xMax * 1.02), plan.SpeedMs[k]),
                           P(Math.Min(plan.RangeLongM[k + 1], xMax * 1.02), plan.SpeedMs[k + 1]), edge, 2f * s);
                dl.AddLine(P(plan.S[k] * plan.BodyRadius, plan.SpeedMs[k]), P(plan.S[k + 1] * plan.BodyRadius, plan.SpeedMs[k + 1]),
                           Palette.Plan.Solid, 2.5f * s);
            }
            int mid = plan.Count / 2;
            dl.AddText(P(plan.RangeShortM[mid], plan.SpeedMs[mid]) + new float2(-70f * s, 0), Palette.Dim.Solid, "HIGH ENERGY");
            dl.AddText(P(Math.Min(plan.RangeLongM[mid], xMax), plan.SpeedMs[mid]) + new float2(6f * s, 0), Palette.Dim.Solid, "LOW ENERGY");
        }

        // The flown path, and a trailer every 30 s for the last 150 s.
        var tr = st.Trace;
        for (int i = 1; i < tr.Count; i++)
            dl.AddLine(P(tr[i - 1].RangeM, tr[i - 1].SpeedMs), P(tr[i].RangeM, tr[i].SpeedMs), Palette.Flown.Solid, 1.2f * s);
        for (int n = 1; n <= 5; n++)
        {
            int i = tr.Count - 1 - 30 * n;
            if (i < 0) break;
            Bug(dl, P(tr[i].RangeM, tr[i].SpeedMs), 4f * s, Palette.Flown.Solid, filled: false);
        }
        if (st.SpeedMs > 0)
            Bug(dl, P(st.RangeM, st.SpeedMs), 6f * s, Palette.Target.Solid, filled: true);
    }

    // ---------------------------------------------------------------- drag against speed

    private static void DrawDragPlot(ImDrawListPtr dl, OrbitalEntryState st, EntryPlan plan, float2 a, float2 b, float s)
    {
        float left = 46f * s, bottom = 16f * s;
        float2 p0 = new float2(a.X + left, a.Y), p1 = new float2(b.X, b.Y - bottom);
        Frame(dl, p0, p1);
        dl.AddText(new float2(p0.X + 4f * s, p0.Y + 2f * s), Palette.Dim.Solid, "DRAG vs SPEED  (REF, ACTUAL, CORRIDOR)");

        double vMax = Math.Max(st.SpeedMs, 1000.0), vMin = 0.0, dMax = Math.Max(st.DragMs2, 5.0);
        if (plan != null)
        {
            vMax = Math.Max(vMax, plan.SpeedMs[0]);
            vMin = plan.SpeedMs[^1];
            for (int k = 0; k < plan.Count; k++)
                dMax = Math.Max(dMax, Math.Max(plan.DragMs2[k], Math.Min(plan.DragMaxMs2[k], 60.0)));
        }
        dMax = NiceCeiling(dMax * 1.1);
        // Speed falls to the right, as the craft flies.
        float2 P(double speed, double d) => new float2(
            p0.X + (p1.X - p0.X) * (float)Math.Clamp((vMax - speed) / Math.Max(vMax - vMin, 1.0), -0.02, 1.02),
            p1.Y - (p1.Y - p0.Y) * (float)Math.Clamp(d / dMax, -0.02, 1.02));
        AxisLabels(dl, p0, p1, s, $"{vMin / 1000:F1} km/s", $"{vMax / 1000:F1}", $"{dMax:F0} m/s²", "0");

        if (plan != null)
        {
            for (int k = 0; k < plan.Count - 1; k++)
            {
                dl.AddLine(P(plan.SpeedMs[k], plan.DragMaxMs2[k]), P(plan.SpeedMs[k + 1], plan.DragMaxMs2[k + 1]), Palette.Bad.Solid, 1.5f * s);
                dl.AddLine(P(plan.SpeedMs[k], plan.DragMinMs2[k]), P(plan.SpeedMs[k + 1], plan.DragMinMs2[k + 1]), Palette.Text.Solid, 1.5f * s);
                dl.AddLine(P(plan.SpeedMs[k], plan.DragMs2[k]), P(plan.SpeedMs[k + 1], plan.DragMs2[k + 1]), Palette.Plan.Solid, 2.5f * s);
            }
        }
        var tr = st.Trace;
        for (int i = 1; i < tr.Count; i++)
            dl.AddLine(P(tr[i - 1].SpeedMs, tr[i - 1].DragMs2), P(tr[i].SpeedMs, tr[i].DragMs2), Palette.Flown.Solid, 2f * s);

        // The drag scale, as on the Shuttle's page: the arrow is the reference, the triangle the drag now.
        float sx = p1.X + 2f * s;
        if (st.Last.DragRefMs2 > 0)
        {
            float yr = P(vMax, st.Last.DragRefMs2).Y;
            dl.AddLine(new float2(sx, yr), new float2(sx + 8f * s, yr - 4f * s), Palette.Plan.Solid, 2f * s);
            dl.AddLine(new float2(sx, yr), new float2(sx + 8f * s, yr + 4f * s), Palette.Plan.Solid, 2f * s);
        }
        if (st.SpeedMs > 0)
            Bug(dl, P(st.SpeedMs, st.DragMs2), 5f * s, Palette.Target.Solid, filled: true);
    }

    // ---------------------------------------------------------------- numbers

    private static void DrawEntryReadouts(ImDrawListPtr dl, OrbitalEntryState st, EntryPlan plan, float2 at, float width, float line)
    {
        var o = st.Last;
        string[] left =
        {
            FormattableString.Invariant($"DRAG     {st.DragMs2,5:F1}  REF {o.DragRefMs2,5:F1} m/s²"),
            FormattableString.Invariant($"HDOT     {st.HdotMs,5:F0}  REF {o.HdotRefMs,5:F0} m/s"),
            FormattableString.Invariant($"BANK CMD {(o.BankDeg >= 0 ? "R" : "L")}{Math.Abs(o.BankDeg),3:F0}  REF {o.BankRefDeg,3:F0}°{(o.Overbanked ? "  OVERBANK" : "")}"),
            FormattableString.Invariant($"ALPHA    {st.AlphaDeg,4:F0}°  MACH {st.Mach,4:F1}  ALT {st.AltitudeM / 1000,5:F1} km"),
            FormattableString.Invariant($"RANGE    {st.RangeM / 1000,6:F0} km  CROSS {st.CrossrangeM / 1000,5:F1} km (band {o.DeadbandRad * (plan?.BodyRadius ?? 0) / 1000,4:F1})"),
        };
        string[] right =
        {
            FormattableString.Invariant($"L/D {st.LiftToDrag,4:F2} (x{st.LiftToDragScale:F2})  AIR x{st.DensityScale:F2}"),
            FormattableString.Invariant($"BELLY {st.BellyAxisName}  TRIM {(st.Trim != null ? st.Trim.TrimmedRows + "/" + st.Trim.Mach.Length : "-")}"),
            FormattableString.Invariant($"PLANS {st.Solves} ({st.Refusals} REFUSED)  {st.LastSolveMs:F0} ms"),
            plan != null ? FormattableString.Invariant($"PLAN MISS {plan.TerminalRangeErrorM / 1000:F1} km  DEFECT {plan.MaxDefect:F3}") : "NO PLAN YET",
            FormattableString.Invariant($"REVERSALS {st.Reversals}  PHASE {st.Phase.ToString().ToUpperInvariant()}"),
        };
        for (int i = 0; i < left.Length; i++)
        {
            dl.AddText(new float2(at.X, at.Y + i * line), Palette.Text.Solid, left[i]);
            dl.AddText(new float2(at.X + width * 0.58f, at.Y + i * line), Palette.Dim.Solid, right[i]);
        }
    }

    // ---------------------------------------------------------------- parameters

    private static void DrawEntryParams(float2 at, float width, float line)
    {
        float fieldW = 90f * ImGuiHelper.InterfaceScale;
        float y = at.Y;
        foreach (var spec in AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs.OrbitalEntryDoubles)
        {
            if (!SettingAccess.TryGetDouble(_s, spec.Key, out double v))
                continue;
            ImGui.SetCursorScreenPos(new float2(at.X, y));
            ImGui.Text(spec.Unit.Length > 0 ? spec.Label + " (" + spec.Unit + ")" : spec.Label);
            if (ImGui.IsItemHovered())
                ConsoleUi.Tooltip(spec.Help.AsSpan());
            ImGui.SetCursorScreenPos(new float2(at.X + width - fieldW, y));
            ImGui.SetNextItemWidth(fieldW);
            if (ImGui.InputDouble("##oe-" + spec.Key, ref v, 0.0, 0.0, spec.Format, ImGuiInputTextFlags.CharsDecimal))
                SettingAccess.SetDouble(_s, spec.Key, Math.Clamp(v, spec.Min, spec.Max));
            y += line * 1.15f;
        }
        foreach (var spec in AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs.OrbitalEntryBools)
        {
            if (!SettingAccess.TryGetBool(_s, spec.Key, out bool v))
                continue;
            ImGui.SetCursorScreenPos(new float2(at.X, y));
            ImGui.Text(spec.Label);
            if (ImGui.IsItemHovered())
                ConsoleUi.Tooltip(spec.Help.AsSpan());
            ImGui.SetCursorScreenPos(new float2(at.X + width - fieldW, y));
            if (ImGui.SmallButton((v ? "ON##oe-" : "OFF##oe-") + spec.Key))
                SettingAccess.SetBool(_s, spec.Key, !v);
            y += line * 1.15f;
        }
        ImGui.SetCursorScreenPos(new float2(at.X, y + line * 0.3f));
        ImGui.Text("Limits and the angle of attack apply from the next re-solve; the belly axis and trim at the next engage.");
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

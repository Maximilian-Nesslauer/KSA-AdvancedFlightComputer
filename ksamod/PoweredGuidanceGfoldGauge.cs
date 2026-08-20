using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using Gfold;
using KSA;

// The Powered landing tab's G-FOLD content: a side view of the descent, the two
// debug toggles, and every solver parameter folded away behind them.
//
// The plot is in the PAD FRAME, which is the frame G-FOLD itself plans in: horizontal
// axis is range to the pad, vertical is height above touchdown, pad at bottom right.
// That makes it a true side elevation of the descent rather than a projection of one.
public static partial class PoweredGuidanceWindow
{
    private const int GfoldTraceCapacity = 400;
    private const double GfoldTraceIntervalSec = 0.2;
    private const double GfoldAxisRelatchLeadSec = 5.0;
    private const float GfoldAxisMargin = 1.15f;

    private static readonly ImColor8 GfoldPlanCol = new ImColor8(90, 190, 255);
    private static readonly ImColor8 GfoldFlownCol = new ImColor8(255, 120, 200);
    private static readonly ImColor8 GfoldThrustCol = new ImColor8(255, 176, 64);

    /// <summary>
    /// Samples the flown path, in the pad frame the plan lives in. Called from the
    /// guidance step, so the trace is even under time warp and exists for craft that
    /// are not on screen.
    /// </summary>
    private static void RecordGfoldTrace(double3 local)
    {
        double now = SimNow();
        if (now - _s.GfoldTraceLastTime < GfoldTraceIntervalSec)
            return;
        _s.GfoldTraceLastTime = now;

        _s.GfoldTrace ??= new float2[GfoldTraceCapacity];
        if (_s.GfoldTraceCount >= GfoldTraceCapacity)
        {
            // Full: drop every other sample and carry on at half the rate. The shape
            // of the descent is what matters, and halving keeps the WHOLE descent
            // rather than a ring buffer's most recent window of it.
            for (int i = 0; i < GfoldTraceCapacity / 2; i++)
                _s.GfoldTrace[i] = _s.GfoldTrace[i * 2];
            _s.GfoldTraceCount = GfoldTraceCapacity / 2;
        }

        // x = up in the pad frame; y,z span the horizon. Altitude is stored RAW —
        // centre of mass above the site terrain — because that is the convention the
        // plan uses: it targets [VehicleHeightM, 0, 0], legs down rather than CoM
        // down. Subtracting the vehicle height here offset the flown path from the
        // plan by exactly that, so the two curves failed to meet at the vehicle.
        _s.GfoldTrace[_s.GfoldTraceCount++] = new float2(
            (float)Math.Sqrt(local.Y * local.Y + local.Z * local.Z),
            (float)local.X);
    }

    private static void ResetGfoldTrace()
    {
        _s.GfoldTraceCount = 0;
        _s.GfoldTraceLastTime = double.NegativeInfinity;
        _s.GfoldAxisLockedAt = double.NegativeInfinity;
        _s.GfoldAxisStage = 0;
        _s.GfoldFlightTime0 = 0.0;
    }

    // --- the side view ------------------------------------------------------

    private static void DrawGfoldPlot(ImDrawListPtr dl, float2 min, float2 size)
    {
        dl.AddRect(min, min + size, SchemSpent, 3f);

        GfoldTrajectory plan = _s.GfoldPlan;
        bool live = _s.LandingPhase == LandingPhase.GfoldDescent;
        if (!live || plan == null || plan.Nodes < 2)
        {
            dl.AddText(min + new float2(6f, 4f), SchemDim, "no G-FOLD solution");
            return;
        }

        LockGfoldAxes(plan);
        double axRange = Math.Max(_s.GfoldAxisRangeM, 1.0);
        double axAlt = Math.Max(_s.GfoldAxisAltM, 1.0);

        float padL = 8f, padR = 10f, padT = 18f, padB = 12f;
        float2 plotMin = min + new float2(padL, padT);
        float2 plotSize = new float2(size.X - padL - padR, size.Y - padT - padB);
        if (plotSize.X < 24f || plotSize.Y < 16f)
            return;

        // Pad at the BOTTOM RIGHT: range runs down to zero as the vehicle arrives, so
        // the descent reads left-to-right the way it is flown.
        float2 ToPlot(double rangeM, double altM) => new float2(
            plotMin.X + plotSize.X * (float)(1.0 - Math.Clamp(rangeM / axRange, 0.0, 1.0)),
            plotMin.Y + plotSize.Y * (float)(1.0 - Math.Clamp(altM / axAlt, 0.0, 1.0)));

        // Ground and the pad.
        float groundY = ToPlot(0.0, 0.0).Y;
        dl.AddLine(new float2(plotMin.X, groundY), new float2(plotMin.X + plotSize.X, groundY),
            SchemSpent, 1.5f);
        dl.AddCircleFilled(ToPlot(0.0, 0.0), 3f, SchemInk);

        DrawGfoldPlannedPath(dl, plan, ToPlot);
        DrawGfoldFlownPath(dl, ToPlot);

        dl.AddText(min + new float2(6f, 4f), SchemDim,
            $"alt {_s.GfoldAltM:F0} m   {_s.GfoldSpeedMs:F0} m/s   thr {_s.GfoldThrottle * 100:F0}");
        string axes = $"{axRange / 1000.0:F1} x {axAlt / 1000.0:F1} km";
        dl.AddText(new float2(min.X + size.X - 6f - ImGui.CalcTextSize(axes).X, min.Y + 4f),
            SchemDim, axes);
    }

    /// <summary>
    /// Latches the axis extents, in FOUR steps: when the descent begins, at half the
    /// flight remaining, at a quarter remaining, and at five seconds to touchdown.
    /// Each fires once, in order, so the picture holds still between them — refitting
    /// every frame zooms it continuously and nothing stays put long enough to read.
    /// </summary>
    private static void LockGfoldAxes(GfoldTrajectory plan)
    {
        double now = SimNow();
        double remaining = _s.GfoldArrivalTime - now;
        // A NaN here would make every threshold comparison false and re-latch on every
        // frame — the exact behaviour the staging exists to prevent.
        if (!double.IsFinite(remaining))
            remaining = plan.TimeOfFlight;

        if (_s.GfoldAxisStage == 0)
        {
            // Total flight, measured once, so the later fractions are of the WHOLE
            // descent rather than of a remainder that shrinks as it is measured.
            _s.GfoldFlightTime0 = Math.Max(remaining, 1.0);
        }
        else
        {
            double due = _s.GfoldAxisStage switch
            {
                1 => _s.GfoldFlightTime0 * 0.5,
                2 => _s.GfoldFlightTime0 * 0.25,
                3 => GfoldAxisRelatchLeadSec,
                _ => double.NegativeInfinity,   // all four done
            };
            if (remaining > due)
                return;
        }

        _s.GfoldAxisStage++;
        _s.GfoldAxisLockedAt = now;

        // Everything still ahead of the vehicle: the remaining plan, plus where the
        // vehicle is now. As the descent proceeds the plan covers less and less, so
        // the same expression frames each later stage progressively closer.
        //
        // Deliberately NOT seeded from _s.GfoldAltM: that is height above TOUCHDOWN
        // while everything here is height above TERRAIN, and mixing the two put the
        // axis out by the vehicle height.
        double range = 0.0;
        double alt = 0.0;
        for (int i = 0; i < plan.Nodes; i++)
        {
            double[] pos = plan.Position[i];
            range = Math.Max(range, Math.Sqrt(pos[1] * pos[1] + pos[2] * pos[2]));
            alt = Math.Max(alt, pos[0]);
        }
        // The vehicle's CURRENT position only, not the whole flown path: including
        // where it started would keep the final view zoomed out to the top of the
        // descent, which is exactly what the second latch exists to escape.
        if (_s.GfoldTraceCount > 0)
        {
            float2 at = _s.GfoldTrace[_s.GfoldTraceCount - 1];
            range = Math.Max(range, at.X);
            alt = Math.Max(alt, at.Y);
        }

        _s.GfoldAxisRangeM = Math.Max(range * GfoldAxisMargin, 50.0);
        _s.GfoldAxisAltM = Math.Max(alt * GfoldAxisMargin, 50.0);
    }

    private static void DrawGfoldPlannedPath(ImDrawListPtr dl, GfoldTrajectory plan,
                                             Func<double, double, float2> toPlot)
    {
        float2 PlanPoint(int i)
        {
            double[] pos = plan.Position[i];
            return toPlot(Math.Sqrt(pos[1] * pos[1] + pos[2] * pos[2]), pos[0]);
        }

        for (int i = 0; i < plan.Nodes - 1; i++)
            dl.AddLine(PlanPoint(i), PlanPoint(i + 1), GfoldPlanCol, 2f);

        // Thrust direction at a handful of nodes. AccelCmd is in the same pad frame,
        // so its up component is x and its horizontal component projects onto the
        // range direction — which points AWAY from the pad, i.e. leftward on the plot.
        int stride = Math.Max(1, plan.Nodes / 8);
        for (int i = 0; i < plan.Nodes; i += stride)
        {
            double[] pos = plan.Position[i];
            double[] acc = plan.AccelCmd[i];
            double horiz = Math.Sqrt(pos[1] * pos[1] + pos[2] * pos[2]);
            if (horiz < 1e-6)
                continue;

            // Component of thrust along "away from the pad".
            double alongRange = (acc[1] * pos[1] + acc[2] * pos[2]) / horiz;
            double mag = Math.Sqrt(acc[0] * acc[0] + alongRange * alongRange);
            if (mag < 1e-6)
                continue;

            float2 at = PlanPoint(i);
            float len = GfoldThrustGlyphPx;
            // Screen: +x is toward the pad (decreasing range), -y is up.
            float2 tip = at + new float2((float)(-alongRange / mag) * len,
                                         (float)(-acc[0] / mag) * len);
            dl.AddLine(at, tip, GfoldThrustCol, 1.5f);
        }
    }

    private static void DrawGfoldFlownPath(ImDrawListPtr dl, Func<double, double, float2> toPlot)
    {
        for (int i = 0; i < _s.GfoldTraceCount - 1; i++)
        {
            float2 a = _s.GfoldTrace[i], b = _s.GfoldTrace[i + 1];
            dl.AddLine(toPlot(a.X, a.Y), toPlot(b.X, b.Y), GfoldFlownCol, 2f);
        }
        if (_s.GfoldTraceCount > 0)
        {
            float2 last = _s.GfoldTrace[_s.GfoldTraceCount - 1];
            dl.AddCircleFilled(toPlot(last.X, last.Y), 3.5f, GfoldFlownCol);
        }
    }

    // --- the tab ------------------------------------------------------------

    private static void DrawGfoldLandingContent(float innerW, float rowH)
    {
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        float plotH = rowH * 6.5f;
        DrawGfoldPlot(dl, ImGui.GetCursorScreenPos(), new float2(innerW, plotH));
        ImGui.Dummy(new float2(innerW, plotH));

        // Legend, so the two traces are never ambiguous.
        dl.AddText(ImGui.GetCursorScreenPos(), GfoldPlanCol, "plan");
        float2 legend = ImGui.GetCursorScreenPos();
        dl.AddText(legend + new float2(38f, 0f), GfoldFlownCol, "flown");
        dl.AddText(legend + new float2(90f, 0f), GfoldThrustCol, "thrust");
        ImGui.Dummy(new float2(innerW, ImGui.GetTextLineHeight()));

        // Debug toggles stay OUT of the fold: they are things you reach for while
        // something is going wrong, which is exactly when you do not want to go
        // hunting through a collapsed section for them.
        ImGui.Checkbox("Show G-FOLD overlay (world)", ref _showGfoldOverlay);
        ImGui.SameLine();
        ImGui.Checkbox("G-FOLD debug", ref _showGfoldDebug);

        // These two are up here rather than with the solver tuning because they are
        // properties of the AIRFRAME and the flight plan, not of the optimiser: the
        // hover handoff decides where G-FOLD stops flying, and the vehicle height is
        // what "on the ground" means. Both get changed per vehicle; the rest rarely
        // get touched at all.
        if (ImGuiHelper.BeginRegion("Descent", ImGuiTreeNodeFlags.DefaultOpen
                | ImGuiTreeNodeFlags.SpanAllColumns, innerW))
        {
            GaugeRow("Hover handoff alt (m)", "##gfhand", ref _s.GfoldHoverHandoffAltM);
            GaugeRow("Vehicle height (m)", "##gfvh", ref _s.VehicleHeightM);
            ImGuiHelper.EndRegion();
        }

        if (ImGuiHelper.BeginRegion("G-FOLD parameters", ImGuiTreeNodeFlags.SpanAllColumns, innerW))
        {
            // The UPFG-to-G-FOLD handoff gate is deliberately NOT here: it governs
            // when the braking burn ends, which is a descent decision. It lives with
            // the approach parameters on the Descent tab.
            GaugeRow("Glide slope (deg)", "##gfglide", ref _s.GfoldGlideSlopeDeg);
            GaugeRow("Thrust pointing (deg)", "##gfpoint", ref _s.GfoldPointingDeg);
            GaugeRow("Max speed (m/s)", "##gfvmax", ref _s.GfoldVMaxMs);
            GaugeRow("Solver min thrust (frac)", "##gftmin", ref _s.GfoldThrottleMin);
            GaugeRow("Solver max thrust (frac)", "##gftmax", ref _s.GfoldThrottleMax);
            GaugeRow("Thrust smoothing (0=off)", "##gfslew", ref _s.GfoldSlewReg);
            GaugeRow("Re-solve interval (s)", "##gfint", ref _s.GfoldIntervalS);

            ImGui.Text("Nodes");
            ImGui.NextColumn();
            ImGui.PushItemWidth(-1f);
            ImGui.InputInt("##gfnodes", ref _s.GfoldNodes);
            ImGui.PopItemWidth();
            ImGui.NextColumn();

            GaugeRow("Track gain Kp", "##gfkp", ref _s.GfoldKp);
            GaugeRow("Track gain Kd", "##gfkd", ref _s.GfoldKd);
            GaugeRow("Command smoothing (s)", "##gftau", ref _s.GfoldSmoothTau);
            ImGuiHelper.EndRegion();
        }
    }
}

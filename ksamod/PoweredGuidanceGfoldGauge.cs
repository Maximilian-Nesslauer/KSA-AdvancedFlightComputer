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
    private const double GfoldAxisFloorM = 50.0;
    // Refit a plan-less view once the content is inside this fraction of the frame.
    private const double GfoldRefitFraction = 0.4;

    private static readonly ImColor8 GfoldPlanCol = new ImColor8(90, 190, 255);
    private static readonly ImColor8 GfoldFlownCol = new ImColor8(255, 120, 200);
    private static readonly ImColor8 GfoldThrustCol = new ImColor8(255, 176, 64);

    /// <summary>
    /// Samples the flown path. Takes the two plotted quantities OUTRIGHT — horizontal
    /// range to the pad and height above the site terrain, both in metres — rather
    /// than a local position, because the two site frames in this mod do not agree on
    /// which axis is up: KsaGfold.BuildFrame is X-up, KsaFrameBridge.BuildSiteFrame is
    /// Z-up. Passing a double3 and picking an axis in here silently read the 6-DOF
    /// plan's altitude off a horizontal component.
    ///
    /// Called from the guidance step, so the trace is even under time warp and exists
    /// for craft that are not on screen.
    /// </summary>
    private static void RecordGfoldTrace(double rangeM, double altM)
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

        // Altitude is stored RAW — centre of mass above the site terrain — because
        // that is the convention the plans use: G-FOLD targets [VehicleHeightM, 0, 0],
        // legs down rather than CoM down. Subtracting the vehicle height here offset
        // the flown path from the plan by exactly that, so the two curves failed to
        // meet at the vehicle.
        _s.GfoldTrace[_s.GfoldTraceCount++] = new float2((float)rangeM, (float)altM);
    }

    /// <summary>
    /// The trace's last sample, read SAFELY. The buffer and the count are written by
    /// the guidance step and read by the draw, so a reset landing between a
    /// "count > 0" test and the indexing that followed it would index [-1]. Both are
    /// snapshotted once here and validated against each other.
    /// </summary>
    private static bool TryLastTraceSample(out float2 at)
    {
        float2[] buf = _s.GfoldTrace;
        int n = _s.GfoldTraceCount;
        if (buf == null || n <= 0 || n > buf.Length)
        {
            at = default;
            return false;
        }
        at = buf[n - 1];
        return true;
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

        // Every powered phase feeds the same trace, so one plot spans all of them.
        // What differs is the PLAN drawn behind it: G-FOLD has a trajectory, 6-DOF has
        // its own, and the hover has none at all because it flies a rate profile.
        bool live = _s.LandingPhase == LandingPhase.GfoldDescent
                 || _s.LandingPhase == LandingPhase.TerminalHover
                 || _s.Active;
        if (!live)
        {
            dl.AddText(min + new float2(6f, 4f), SchemDim, "no powered descent");
            return;
        }
        GfoldTrajectory plan = !_s.Active && _s.GfoldPlan != null && _s.GfoldPlan.Nodes >= 2
            ? _s.GfoldPlan : null;

        LockGfoldAxes(plan);
        double axRange = Math.Max(_s.GfoldAxisRangeM, 1.0);
        double axAlt = Math.Max(_s.GfoldAxisAltM, 1.0);

        float padL = 8f, padR = 10f, padT = 18f, padB = 12f;
        float2 plotMin = min + new float2(padL, padT);
        float2 plotSize = new float2(size.X - padL - padR, size.Y - padT - padB);
        if (plotSize.X < 24f || plotSize.Y < 16f)
            return;

        // The two axes scale INDEPENDENTLY, each filling the box.
        //
        // A shared metres-per-pixel is the honest way to draw an elevation, and I tried
        // it - but the box is about five times wider than tall, and a descent is far
        // taller than it is wide by the end, so the isotropic fit left the trajectory
        // as a sliver against the right edge with 90% of the plot empty. A distorted
        // picture that fills the frame beats a true one too small to read; the corner
        // label states both spans so the scales are never implied to match.
        //
        // Pad at the BOTTOM RIGHT: range runs down to zero as the vehicle arrives, so
        // the descent reads left-to-right the way it is flown. NOT clamped - points
        // outside the frame keep their true coordinates so they can be clipped away
        // rather than smeared along the edge.
        float2 plotMax = plotMin + plotSize;
        float2 ToPlot(double rangeM, double altM) => new float2(
            plotMax.X - plotSize.X * (float)(rangeM / axRange),
            plotMax.Y - plotSize.Y * (float)(altM / axAlt));

        // Ground and the pad.
        float groundY = ToPlot(0.0, 0.0).Y;
        dl.AddLine(new float2(plotMin.X, groundY), new float2(plotMax.X, groundY),
            SchemSpent, 1.5f);
        dl.AddCircleFilled(ToPlot(0.0, 0.0), 3f, SchemInk);

        if (plan != null)
            DrawGfoldPlannedPath(dl, plan, ToPlot, plotMin, plotMax);
        else if (_s.Active)
            Draw6DofPlannedPath(dl, ToPlot, plotMin, plotMax);
        DrawGfoldFlownPath(dl, ToPlot, plotMin, plotMax);

        // _s.GfoldAltM / GfoldSpeedMs / GfoldThrottle are written by the G-FOLD and
        // hover steps only, so under 6-DOF they hold whatever the last descent left
        // behind - which is why this line read "alt 0 m 0 m/s thr 0" throughout.
        // 6-DOF reports from its own plan and command instead.
        double altNow = TryLastTraceSample(out float2 lastSample) ? lastSample.Y : _s.GfoldAltM;
        double speedNow = _s.GfoldSpeedMs;
        double thrNow = _s.GfoldThrottle;
        if (_s.Active && _s.Guidance != null && _s.Guidance.HasPlan)
        {
            ReadOnlySpan<double> px0 = _s.Guidance.PlanState;
            if (px0.Length >= 14)
                speedNow = Math.Sqrt(px0[3] * px0[3] + px0[4] * px0[4] + px0[5] * px0[5]);
            thrNow = _s.LastThrottle;
        }
        dl.AddText(min + new float2(6f, 4f), SchemDim,
            $"alt {altNow:F0} m   {speedNow:F0} m/s   thr {thrNow * 100:F0}");
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

        // Without an arrival time to stage against - 6-DOF and the hover - the frame
        // is refitted whenever the picture no longer suits it, in EITHER direction.
        //
        // The previous rule only ever shrank: it refitted when the vehicle dropped
        // below a fraction of the frame, and never when the content outgrew it. With
        // the 6-DOF plan not counted at all, the first latch floored at the minimum
        // and the plan was then clamped to the edges for the rest of the descent.
        if (plan == null)
        {
            DescentExtent(null, out double r, out double a);
            bool first = _s.GfoldAxisStage == 0;
            bool doesNotFit = r > _s.GfoldAxisRangeM || a > _s.GfoldAxisAltM;
            // EITHER axis, not both. Requiring both is why the horizontal never
            // tightened in the final phase: range collapses to nearly nothing while
            // altitude is still coming down slowly, so the range axis stayed at
            // whatever it was latched at kilometres earlier.
            bool farTooWide = r < _s.GfoldAxisRangeM * GfoldRefitFraction
                           || a < _s.GfoldAxisAltM * GfoldRefitFraction;
            if (!first && !doesNotFit && !farTooWide)
                return;

            _s.GfoldAxisStage = 1;
            _s.GfoldAxisLockedAt = now;
            SetAxes(r, a);
            return;
        }

        double remaining = _s.GfoldArrivalTime - now;
        // A NaN here would make every threshold comparison false and re-latch on every
        // frame - the exact behaviour the staging exists to prevent.
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
        DescentExtent(plan, out double range, out double alt);
        SetAxes(range, alt);
    }

    private static void SetAxes(double rangeM, double altM)
    {
        _s.GfoldAxisRangeM = Math.Max(rangeM * GfoldAxisMargin, GfoldAxisFloorM);
        _s.GfoldAxisAltM = Math.Max(altM * GfoldAxisMargin, GfoldAxisFloorM);
    }

    /// <summary>
    /// How much of the plot the descent needs: whatever is still to be flown, plus the
    /// vehicle itself. Covers all three sources - a G-FOLD trajectory, the 6-DOF plan,
    /// or neither during the hover - so the frame is never sized off one of them while
    /// another is what is actually drawn.
    ///
    /// The vehicle contributes its CURRENT position only, not the whole flown path:
    /// including where it started would keep the view pinned to the top of the descent
    /// all the way down.
    /// </summary>
    private static void DescentExtent(GfoldTrajectory gfold, out double rangeM, out double altM)
    {
        rangeM = 0.0;
        altM = 0.0;

        if (gfold != null)
        {
            for (int i = 0; i < gfold.Nodes; i++)
            {
                double[] pos = gfold.Position[i];   // X-up
                rangeM = Math.Max(rangeM, Math.Sqrt(pos[1] * pos[1] + pos[2] * pos[2]));
                altM = Math.Max(altM, pos[0]);
            }
        }
        else if (_s.Active && _s.Guidance != null && _s.Guidance.HasPlan)
        {
            ReadOnlySpan<double> px = _s.Guidance.PlanState;
            int n = _s.Guidance.Nodes;
            if (px.Length >= n * 14)
            {
                for (int k = 0; k < n; k++)
                {
                    int b = k * 14;                 // Z-up
                    rangeM = Math.Max(rangeM, Math.Sqrt(px[b + 0] * px[b + 0] + px[b + 1] * px[b + 1]));
                    altM = Math.Max(altM, px[b + 2]);
                }
            }
        }

        if (TryLastTraceSample(out float2 at))
        {
            rangeM = Math.Max(rangeM, at.X);
            altM = Math.Max(altM, at.Y);
        }
    }

    /// <summary>
    /// Liang-Barsky: trims a segment to the plot rectangle, or reports it entirely
    /// outside. Clipping rather than clamping matters here - a clamped point sits on
    /// the boundary and joins up with its neighbours, so a descent beginning off the
    /// top of the frame was drawn as a flat line along it, which looks exactly like a
    /// long level cruise the vehicle never flew.
    /// </summary>
    private static bool ClipToBox(ref float2 a, ref float2 b, float2 lo, float2 hi)
    {
        float t0 = 0f, t1 = 1f;
        float dx = b.X - a.X, dy = b.Y - a.Y;

        for (int edge = 0; edge < 4; edge++)
        {
            float p, q;
            switch (edge)
            {
                case 0: p = -dx; q = a.X - lo.X; break;
                case 1: p = dx; q = hi.X - a.X; break;
                case 2: p = -dy; q = a.Y - lo.Y; break;
                default: p = dy; q = hi.Y - a.Y; break;
            }

            if (MathF.Abs(p) < 1e-6f)
            {
                if (q < 0f)
                    return false;   // parallel to this edge, and outside it
                continue;
            }

            float r = q / p;
            if (p < 0f)
            {
                if (r > t1) return false;
                if (r > t0) t0 = r;
            }
            else
            {
                if (r < t0) return false;
                if (r < t1) t1 = r;
            }
        }

        float2 a0 = a;
        a = new float2(a0.X + t0 * dx, a0.Y + t0 * dy);
        b = new float2(a0.X + t1 * dx, a0.Y + t1 * dy);
        return true;
    }

    private static void ClipLine(ImDrawListPtr dl, float2 a, float2 b, float2 lo, float2 hi,
                                 ImColor8 col, float thickness)
    {
        if (ClipToBox(ref a, ref b, lo, hi))
            dl.AddLine(a, b, col, thickness);
    }

    private static bool Inside(float2 p, float2 lo, float2 hi)
        => p.X >= lo.X && p.X <= hi.X && p.Y >= lo.Y && p.Y <= hi.Y;

    private static void DrawGfoldPlannedPath(ImDrawListPtr dl, GfoldTrajectory plan,
                                             Func<double, double, float2> toPlot,
                                             float2 lo, float2 hi)
    {
        float2 PlanPoint(int i)
        {
            double[] pos = plan.Position[i];
            return toPlot(Math.Sqrt(pos[1] * pos[1] + pos[2] * pos[2]), pos[0]);
        }

        for (int i = 0; i < plan.Nodes - 1; i++)
            ClipLine(dl, PlanPoint(i), PlanPoint(i + 1), lo, hi, GfoldPlanCol, 2f);

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
            if (!Inside(at, lo, hi))
                continue;
            float len = GfoldThrustGlyphPx;
            // Screen: +x is toward the pad (decreasing range), -y is up.
            float2 tip = at + new float2((float)(-alongRange / mag) * len,
                                         (float)(-acc[0] / mag) * len);
            ClipLine(dl, at, tip, lo, hi, GfoldThrustCol, 1.5f);
        }
    }

    /// <summary>
    /// The 6-DOF plan, in the same side view. Node states are laid out flat, fourteen
    /// doubles each with position first — the same buffer the world overlay reads.
    /// </summary>
    private static void Draw6DofPlannedPath(ImDrawListPtr dl, Func<double, double, float2> toPlot,
                                            float2 lo, float2 hi)
    {
        Ksa6DofGuidance g = _s.Guidance;
        if (g == null || !g.HasPlan)
            return;

        int n = g.Nodes;
        if (n < 2)
            return;

        // Materialised up front: a Span cannot be captured by a local function, and
        // projecting each node once is cheaper than doing it per use anyway.
        var pt = new float2[n];
        var thrust = new float2[n];
        bool haveThrust;
        {
            ReadOnlySpan<double> px = g.PlanState;
            ReadOnlySpan<double> pu = g.PlanControl;
            if (px.Length < n * 14)
                return;
            haveThrust = pu.Length >= n * 4;

            // KsaFrameBridge's site frame is Z-UP — (ex, ey, up) — unlike the G-FOLD
            // frame above it, which is X-up. So height is index 2 and the horizon is
            // 0 and 1, the other way round from DrawGfoldPlannedPath.
            for (int k = 0; k < n; k++)
            {
                int b = k * 14;
                double horiz = Math.Sqrt(px[b + 0] * px[b + 0] + px[b + 1] * px[b + 1]);
                pt[k] = toPlot(horiz, px[b + 2]);

                if (!haveThrust)
                    continue;

                // The control is thrust in BODY coordinates, four doubles per node —
                // NOT three, and not in the site frame. It has to be rotated by the
                // node's attitude quaternion before it means anything spatially, which
                // is exactly what the world overlay does.
                int c = k * 4;
                KsaFrameBridge.QuatToMatrix(px[b + 6], px[b + 7], px[b + 8], px[b + 9],
                    out double3 b0, out double3 b1, out double3 b2);
                double3 tSite = b0 * pu[c + 0] + b1 * pu[c + 1] + b2 * pu[c + 2];

                // Up component, and the component along "away from the pad". Directly
                // over the pad there is no meaningful range direction, so only the
                // vertical part survives.
                double alongRange = horiz > 1e-6
                    ? (tSite.X * px[b + 0] + tSite.Y * px[b + 1]) / horiz
                    : 0.0;
                double mag = Math.Sqrt(tSite.Z * tSite.Z + alongRange * alongRange);
                if (mag > 1e-6)
                    thrust[k] = new float2((float)(-alongRange / mag), (float)(-tSite.Z / mag));
            }
        }

        for (int k = 0; k < n - 1; k++)
            ClipLine(dl, pt[k], pt[k + 1], lo, hi, GfoldPlanCol, 2f);

        int stride = Math.Max(1, n / 8);
        for (int k = 0; k < n; k += stride)
        {
            if ((thrust[k].X == 0f && thrust[k].Y == 0f) || !Inside(pt[k], lo, hi))
                continue;
            ClipLine(dl, pt[k], pt[k] + thrust[k] * GfoldThrustGlyphPx, lo, hi,
                GfoldThrustCol, 1.5f);
        }
    }

    private static void DrawGfoldFlownPath(ImDrawListPtr dl, Func<double, double, float2> toPlot,
                                           float2 lo, float2 hi)
    {
        // Snapshotted once: the guidance step appends to this while the draw walks it.
        float2[] buf = _s.GfoldTrace;
        int n = buf == null ? 0 : Math.Min(_s.GfoldTraceCount, buf.Length);

        for (int i = 0; i < n - 1; i++)
        {
            float2 a = buf[i], b = buf[i + 1];
            ClipLine(dl, toPlot(a.X, a.Y), toPlot(b.X, b.Y), lo, hi, GfoldFlownCol, 2f);
        }
        if (n > 0)
        {
            float2 p = toPlot(buf[n - 1].X, buf[n - 1].Y);
            if (Inside(p, lo, hi))
                dl.AddCircleFilled(p, 3.5f, GfoldFlownCol);
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

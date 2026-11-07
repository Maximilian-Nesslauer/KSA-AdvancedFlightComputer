#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using AdvancedFlightComputer.Guidance.Gfold;
using KSA;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

// The G-FOLD content of the LAND page and of the tuning page: a side view of the descent on the page, the airframe values in the page and its folds, and every solver parameter and debug toggle on the tuning page.
//  The plot is in the PAD FRAME, which is the frame G-FOLD itself plans in: horizontal axis is range to the pad, vertical is height above touchdown, pad at bottom right.
// That makes it a true side elevation of the descent rather than a projection of one.
public static partial class GuidanceWindow
{
    private const int GfoldTraceCapacity = 400;
    private const double GfoldTraceIntervalSec = 0.2;
    private const double GfoldAxisRelatchLeadSec = 5.0;
    private const float GfoldAxisMargin = 1.15f;
    private const double GfoldAxisFloorM = 50.0;
    // Refit a plan-less view once the content is inside this fraction of the frame.
    private const double GfoldRefitFraction = 0.4;

    private static readonly ImColor8 GfoldPlanCol = Palette.Plan.Solid;
    private static readonly ImColor8 GfoldFlownCol = Palette.Flown.Solid;
    private static readonly ImColor8 GfoldThrustCol = Palette.Plan.Solid;

    private const float PlanBurnThick = 2.5f, PlanCoastThick = 1f;
    private const float PlanCoastAlpha = 0.55f;

    private const double PlanBurnShare = 0.02;

    private static readonly string[] _descentLegendNames = { "Flown", "Burn", "Coast", PageText.ThrustKey };
    private static readonly Picture.Swatch[] _descentSwatches =
    {
        new("Flown", Palette.Flown),
        new("Burn", Palette.Plan, SwatchKind.Thick),
        new("Coast", Palette.Plan),
    };
    private static readonly Picture.Swatch _thrustSwatch = new(PageText.ThrustKey, Palette.Plan, SwatchKind.Tick);
    private static readonly Picture.Swatch[] _descentShown = new Picture.Swatch[4];

    // The plot drew a mark of the plan thrust in this frame, so the legend names it.
    private static bool _thrustDrawn;

    /// <summary>
    /// Samples the flown path. Takes the two plotted quantities OUTRIGHT - horizontal range to the pad and height above the site terrain, both in metres - rather than a local position, because the two site frames in this mod do not agree on which axis is up: KsaGfold.BuildFrame is X-up, KsaFrameBridge.BuildSiteFrame is Z-up. Passing a double3 and picking an axis in here silently read the 6-DOF plan's altitude off a horizontal component.
    ///  Called from the guidance step, so the trace is even under time warp and exists for craft that are not on screen.
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
            // Full: drop every other sample and carry on at half the rate. The shape of the descent is what matters, and halving keeps the WHOLE descent rather than a ring buffer's most recent window of it.
            for (int i = 0; i < GfoldTraceCapacity / 2; i++)
                _s.GfoldTrace[i] = _s.GfoldTrace[i * 2];
            _s.GfoldTraceCount = GfoldTraceCapacity / 2;
        }

        // Altitude is stored RAW - centre of mass above the site terrain - because that is the convention the plans use: G-FOLD targets [VehicleHeightM, 0, 0], legs down rather than CoM down. Subtracting the vehicle height here offset the flown path from the plan by exactly that, so the two curves failed to meet at the vehicle.
        _s.GfoldTrace[_s.GfoldTraceCount++] = new float2((float)rangeM, (float)altM);
    }

    /// <summary>
    /// The trace's last sample, read SAFELY. The buffer and the count are written by the guidance step and read by the draw, so a reset landing between a "count > 0" test and the indexing that followed it would index [-1]. Both are snapshotted once here and validated against each other.
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

    private static bool PoweredDescentDrawn
        => _s.LandingPhase == LandingPhase.GfoldDescent || _s.LandingPhase == LandingPhase.TerminalHover || TerminalBurnActive || _s.Active;

    private static bool PoweredDescentStopped
        => !PoweredDescentDrawn && _s.GfoldTraceCount > 1 && !_s.SixDofStandby && !_s.EngagePending
           && Program.ControlledVehicle is Vehicle craft
           && FlightView.StoppedPathShows(Ui.Console.ConsoleFrame.ReadFacts(craft).Where, SimNow() - _s.GfoldTraceLastTime);

    private static readonly System.Runtime.CompilerServices.ConditionalWeakTable<VehicleAutopilotState, PlanJump> _planJumps = new();

    private static double DescentPlanToGo()
    {
        if (_s.Active && _s.Guidance != null && _s.Guidance.HasPlan && _s.Guidance.Published is { } published)
            return published.Sigma - (SimNow() - published.SolveTime);
        return _s.LandingPhase == LandingPhase.GfoldDescent && _s.GfoldPlan != null ? _s.GfoldArrivalTime - SimNow() : double.NaN;
    }

    private static void DrawGfoldPlot(in PictureFrame f, Vehicle vehicle)
    {
        ImDrawListPtr dl = f.Dl;
        float s = f.Scale;
        bool stopped = !PoweredDescentDrawn;

        // Every powered phase feeds the same trace, so one plot spans all of them.
        // What differs is the PLAN drawn behind it: G-FOLD has a trajectory, 6-DOF has its own, and the hover has none at all because it flies a rate profile.
        GfoldTrajectory plan = !_s.Active && _s.GfoldPlan != null && _s.GfoldPlan.Nodes >= 2
            ? _s.GfoldPlan : null;

        if (stopped)
            FitStoppedAxes(plan);
        else
            LockGfoldAxes(plan);
        double axRange = Math.Max(_s.GfoldAxisRangeM, 1.0);
        double axAlt = Math.Max(_s.GfoldAxisAltM, 1.0);

        PictureLanes lanes = Picture.Lanes(in f, Picture.TextLaneHeight(), Picture.TickWidth(AxisTicks.Distance(axAlt)),
            Picture.LegendWidth(_descentLegendNames), Picture.TextLaneHeight());
        PicBox plot = lanes.Plot;

        // The two axes scale INDEPENDENTLY, each filling the box.
        //  A shared metres-per-pixel would draw a true elevation, but the box is about five times wider than tall and a descent is far taller than it is wide by the end, so an isotropic fit leaves the trajectory as a sliver against the right edge with most of the plot empty. A distorted picture that fills the frame beats a true one too small to read, and the ticks on both axes state both scales, so they are never implied to match.
        //  Pad at the BOTTOM RIGHT: range runs down to zero as the vehicle arrives, so the descent reads left-to-right the way it is flown. NOT clamped - points outside the frame keep their true coordinates so they can be clipped away rather than smeared along the edge.
        float2 plotMin = new float2(plot.X + 4f * s, plot.Y + 4f * s);
        float2 plotMax = new float2(plot.Right - 8f * s, plot.Bottom - 5f * s);
        float2 plotSize = plotMax - plotMin;
        if (plotSize.X < 24f || plotSize.Y < 16f)
            return;
        float2 ToPlot(double rangeM, double altM) => new float2(
            plotMax.X - plotSize.X * (float)(rangeM / axRange),
            plotMax.Y - plotSize.Y * (float)(altM / axAlt));

        Picture.AxisX(in f, in lanes, 0.0, axRange, time: false, r => ToPlot(r, 0.0).X, r => r == 0.0 ? PageText.SiteTick : AxisTicks.Distance(r));
        Picture.AxisY(in f, in lanes, 0.0, axAlt, a => ToPlot(0.0, a).Y, AxisTicks.Distance);

        // Ground and the pad.
        float groundY = ToPlot(0.0, 0.0).Y;
        dl.AddLine(new float2(plotMin.X, groundY), new float2(plotMax.X, groundY),
            SchemSpent, 1.5f);
        Glyphs.Site(dl, ToPlot(0.0, 0.0), 5f * s, Palette.Target.Solid, 1.5f * s);

        int legend = 1;
        _thrustDrawn = false;
        if (plan != null)
            legend = DrawGfoldPlannedPath(dl, plan, ToPlot, plotMin, plotMax, stopped);
        else if (_s.Active)
            legend = Draw6DofPlannedPath(dl, ToPlot, plotMin, plotMax);
        DrawGfoldFlownPath(dl, ToPlot, plotMin, plotMax);

        double thrNow = _s.Active && _s.Guidance != null && _s.Guidance.HasPlan ? _s.LastThrottle : _s.GfoldThrottle;
        double altNow = TryLastTraceSample(out float2 lastSample) ? lastSample.Y : _s.GfoldAltM;
        string caption = PlotShowsTouchdown ? SixDofCaption() : "";
        bool newPlan = !stopped && _planJumps.GetOrCreateValue(_s).Update(DescentPlanToGo(), GuidanceClock.NowMs / 1000.0);
        string head = stopped
            ? (vehicle.Situation.HasAnyContact() ? PageText.FlownPath : PageText.StoppedAt(altNow))
            : (newPlan ? PageText.NewPlan + "   " : caption.Length > 0 ? caption + "   " : "") + (plan != null && thrNow < 0.01 && PlanCoastsNow(plan) ? PageText.CoastWord : "Throttle " + Fmt.Percent(thrNow));
        Picture.HeadText(in f, lanes.Head, PageText.HeightAboveSite + "   " + head, newPlan ? Palette.Text.Solid : Palette.Dim.Solid);
        _descentSwatches.AsSpan(0, legend).CopyTo(_descentShown);
        if (_thrustDrawn)
            _descentShown[legend++] = _thrustSwatch;
        Picture.Legend(in f, lanes.Right, _descentShown.AsSpan(0, legend));
    }

    private static bool PlanCoastsNow(GfoldTrajectory plan)
    {
        if (!(plan.Dt > 0.0) || plan.Nodes < 2)
            return false;
        double topSigma = 0.0;
        for (int i = 0; i < plan.Nodes; i++)
            topSigma = Math.Max(topSigma, plan.Sigma[i]);
        int now = Math.Clamp((int)((SimNow() - _s.GfoldPlanStart) / plan.Dt), 0, plan.Nodes - 1);
        return plan.Sigma[now] <= topSigma * PlanBurnShare;
    }

    // The axes of a descent that no longer flies: the whole flown path and the plan it had, with no latch, because nothing moves any more.
    private static void FitStoppedAxes(GfoldTrajectory plan)
    {
        DescentExtent(plan, out double range, out double alt);
        float2[] buf = _s.GfoldTrace;
        int n = buf == null ? 0 : Math.Min(_s.GfoldTraceCount, buf.Length);
        for (int i = 0; i < n; i++)
        {
            range = Math.Max(range, buf[i].X);
            alt = Math.Max(alt, buf[i].Y);
        }
        SetAxes(range, alt);
    }

    /// <summary>
    /// Latches the axis extents, in FOUR steps: when the descent begins, at half the flight remaining, at a quarter remaining, and at five seconds to touchdown.
    /// Each fires once, in order, so the picture holds still between them - refitting every frame zooms it continuously and nothing stays put long enough to read.
    /// </summary>
    private static void LockGfoldAxes(GfoldTrajectory plan)
    {
        double now = SimNow();

        // Without an arrival time to stage against - 6-DOF and the hover - the frame is refitted whenever the picture no longer suits it, in EITHER direction.
        //  The previous rule only ever shrank: it refitted when the vehicle dropped below a fraction of the frame, and never when the content outgrew it. With the 6-DOF plan not counted at all, the first latch floored at the minimum and the plan was then clamped to the edges for the rest of the descent.
        if (plan == null)
        {
            DescentExtent(null, out double r, out double a);
            bool first = _s.GfoldAxisStage == 0;
            bool doesNotFit = r > _s.GfoldAxisRangeM || a > _s.GfoldAxisAltM;
            // EITHER axis, not both. Requiring both is why the horizontal never tightened in the final phase: range collapses to nearly nothing while altitude is still coming down slowly, so the range axis stayed at whatever it was latched at kilometres earlier.
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
        // A NaN here would make every threshold comparison false and re-latch on every frame - the exact behaviour the staging exists to prevent.
        if (!double.IsFinite(remaining))
            remaining = plan.TimeOfFlight;

        if (_s.GfoldAxisStage == 0)
        {
            // Total flight, measured once, so the later fractions are of the WHOLE descent rather than of a remainder that shrinks as it is measured.
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
    /// How much of the plot the descent needs: whatever is still to be flown, plus the vehicle itself. Covers all three sources - a G-FOLD trajectory, the 6-DOF plan, or neither during the hover - so the frame is never sized off one of them while another is what is actually drawn.
    ///  The vehicle contributes its CURRENT position only, not the whole flown path:
    /// including where it started would keep the view pinned to the top of the descent all the way down.
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
    /// Liang-Barsky: trims a segment to the plot rectangle, or reports it entirely outside. Clipping rather than clamping matters here - a clamped point sits on the boundary and joins up with its neighbours, so a descent beginning off the top of the frame was drawn as a flat line along it, which looks exactly like a long level cruise the vehicle never flew.
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

    private static int DrawGfoldPlannedPath(ImDrawListPtr dl, GfoldTrajectory plan,
                                            Func<double, double, float2> toPlot,
                                            float2 lo, float2 hi, bool stopped)
    {
        float2 PlanPoint(int i)
        {
            double[] pos = plan.Position[i];
            return toPlot(Math.Sqrt(pos[1] * pos[1] + pos[2] * pos[2]), pos[0]);
        }

        double topSigma = 0.0;
        for (int i = 0; i < plan.Nodes; i++)
            topSigma = Math.Max(topSigma, plan.Sigma[i]);
        double burnSigma = topSigma * PlanBurnShare;
        bool coast = false;
        for (int i = 0; i < plan.Nodes - 1; i++)
        {
            bool burns = plan.Sigma[i] > burnSigma;
            coast |= !burns;
            DrawPlanSegment(dl, PlanPoint(i), PlanPoint(i + 1), lo, hi, burns, stopped);
        }

        // Thrust direction at a handful of nodes. AccelCmd is in the same pad frame, so its up component is x and its horizontal component projects onto the range direction - which points AWAY from the pad, i.e. leftward on the plot.
        int stride = Math.Max(1, plan.Nodes / 8);
        for (int i = 0; i < plan.Nodes; i += stride)
        {
            double[] pos = plan.Position[i];
            double[] acc = plan.AccelCmd[i];
            double horiz = Math.Sqrt(pos[1] * pos[1] + pos[2] * pos[2]);
            if (horiz < 1e-6 || plan.Sigma[i] <= burnSigma)
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
            ClipLine(dl, at, tip, lo, hi, stopped ? Palette.Plan.A(0.4f) : GfoldThrustCol, 1.5f);
            _thrustDrawn = true;
        }
        return coast ? 3 : 2;
    }

    private static void DrawPlanSegment(ImDrawListPtr dl, float2 a, float2 b, float2 lo, float2 hi, bool burns, bool stopped)
    {
        ImColor8 ink = stopped ? Palette.Plan.A(0.4f) : burns ? GfoldPlanCol : Palette.Plan.A(PlanCoastAlpha);
        ClipLine(dl, a, b, lo, hi, ink, burns ? PlanBurnThick : PlanCoastThick);
    }

    /// <summary>
    /// The 6-DOF plan, in the same side view. Node states are laid out flat, fourteen doubles each with position first - the same buffer the world overlay reads.
    /// </summary>
    private static int Draw6DofPlannedPath(ImDrawListPtr dl, Func<double, double, float2> toPlot,
                                           float2 lo, float2 hi)
    {
        Ksa6DofGuidance g = _s.Guidance;
        if (g == null || !g.HasPlan)
            return 1;

        int n = g.Nodes;
        if (n < 2)
            return 1;

        // Materialised up front: a Span cannot be captured by a local function, and projecting each node once is cheaper than doing it per use anyway.
        var pt = new float2[n];
        var thrust = new float2[n];
        var thrustMag = new double[n];
        double topThrust = 0.0;
        bool haveThrust;
        {
            ReadOnlySpan<double> px = g.PlanState;
            ReadOnlySpan<double> pu = g.PlanControl;
            if (px.Length < n * 14)
                return 1;
            haveThrust = pu.Length >= n * 4;

            // KsaFrameBridge's site frame is Z-UP - (ex, ey, up) - unlike the G-FOLD frame above it, which is X-up. So height is index 2 and the horizon is 0 and 1, the other way round from DrawGfoldPlannedPath.
            for (int k = 0; k < n; k++)
            {
                int b = k * 14;
                double horiz = Math.Sqrt(px[b + 0] * px[b + 0] + px[b + 1] * px[b + 1]);
                pt[k] = toPlot(horiz, px[b + 2]);

                if (!haveThrust)
                    continue;

                // The control is thrust in BODY coordinates, four doubles per node - NOT three, and not in the site frame. It has to be rotated by the node's attitude quaternion before it means anything spatially, which is exactly what the world overlay does.
                int c = k * 4;
                KsaFrameBridge.QuatToMatrix(px[b + 6], px[b + 7], px[b + 8], px[b + 9],
                    out double3 b0, out double3 b1, out double3 b2);
                double3 tSite = b0 * pu[c + 0] + b1 * pu[c + 1] + b2 * pu[c + 2];

                // Up component, and the component along "away from the pad". Directly over the pad there is no meaningful range direction, so only the vertical part survives.
                double alongRange = horiz > 1e-6
                    ? (tSite.X * px[b + 0] + tSite.Y * px[b + 1]) / horiz
                    : 0.0;
                double mag = Math.Sqrt(tSite.Z * tSite.Z + alongRange * alongRange);
                if (mag > 1e-6)
                    thrust[k] = new float2((float)(-alongRange / mag), (float)(-tSite.Z / mag));
                thrustMag[k] = mag;
                topThrust = Math.Max(topThrust, mag);
            }
        }

        double burnThrust = haveThrust ? topThrust * PlanBurnShare : -1.0;
        bool coast = false;
        for (int k = 0; k < n - 1; k++)
        {
            bool burns = thrustMag[k] > burnThrust;
            coast |= !burns;
            DrawPlanSegment(dl, pt[k], pt[k + 1], lo, hi, burns, stopped: false);
        }

        int stride = Math.Max(1, n / 8);
        for (int k = 0; k < n; k += stride)
        {
            if ((thrust[k].X == 0f && thrust[k].Y == 0f) || thrustMag[k] <= burnThrust || !Inside(pt[k], lo, hi))
                continue;
            ClipLine(dl, pt[k], pt[k] + thrust[k] * GfoldThrustGlyphPx, lo, hi,
                GfoldThrustCol, 1.5f);
            _thrustDrawn = true;
        }
        return coast ? 3 : 2;
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

    internal static void DrawGfoldDebugInline()
    {
        if (_s.LandingSeverity == Severity.Diag && _s.LandingStatus.Length > 0)
            ConsoleUi.Readout("SOLVER".AsSpan(), Fmt.Sentence(_s.LandingStatus).AsSpan());
        GfoldTrajectory plan = _s.GfoldPlan;
        if (plan == null)
        {
            ConsoleUi.MutedWrapped(PageText.DiagEmpty(TuneDiagnostics.Gfold));
            return;
        }

        double elapsed = Math.Clamp(SimNow() - _s.GfoldPlanStart, 0.0, plan.TimeOfFlight);
        float cursor = plan.TimeOfFlight > 1e-9 ? (float)(elapsed / plan.TimeOfFlight) : 0f;
        int n = plan.Nodes;

        // Solve cost is a first-class readout, not a diagnostic: this runs on the sim thread, so anything past a frame time (16.7 ms at 60 Hz) is a stutter the player feels once every re-solve interval.
        ConsoleUi.Readout("STATUS".AsSpan(), _s.GfoldStatus.ToString().AsSpan());
        ConsoleUi.Readout("PLAN".AsSpan(), FormattableString.Invariant($"{n} points, dt {plan.Dt:F2} s, tf {plan.TimeOfFlight:F1} s").AsSpan());
        if (_s.GfoldThrustStatus.Length > 0)
            ConsoleUi.Readout("THRUST".AsSpan(), Fmt.Sentence(_s.GfoldThrustStatus).AsSpan());
        ConsoleUi.Readout("LAST SOLVE".AsSpan(), FormattableString.Invariant($"{_s.GfoldSolveMs:F1} ms, {_s.GfoldSolveMs / 16.7:F1} frames at 60 Hz").AsSpan());
        ConsoleUi.Readout("FUEL USED".AsSpan(), FormattableString.Invariant($"{plan.FuelUsed:F0} kg").AsSpan());
        ConsoleUi.Readout("LANDING ERROR".AsSpan(), FormattableString.Invariant($"{plan.LandingErrorNorm:F1} m").AsSpan());
        ConsoleUi.Readout("PLAN TIME".AsSpan(), FormattableString.Invariant($"t+{elapsed:F1} s").AsSpan());

        PlotPair("r.up (m)", i => plan.Position[i][0],
                 "r.y (m)", i => plan.Position[i][1], n, cursor);
        PlotPair("r.z (m)", i => plan.Position[i][2],
                 "speed (m/s)", i => VecLen(plan.Velocity[i]), n, cursor);
        PlotPair("v.up (m/s)", i => plan.Velocity[i][0],
                 "v.y (m/s)", i => plan.Velocity[i][1], n, cursor);
        PlotPair("v.z (m/s)", i => plan.Velocity[i][2],
                 "|u| (m/s2)", i => VecLen(plan.AccelCmd[i]), n, cursor);
        PlotPair("u.up (m/s2)", i => plan.AccelCmd[i][0],
                 "u.y (m/s2)", i => plan.AccelCmd[i][1], n, cursor);
        PlotPair("u.z (m/s2)", i => plan.AccelCmd[i][2],
                 "sigma (m/s2)", i => plan.Sigma[i], n, cursor);
        PlotPair("plan thrust (% of max)", i =>
                 {
                     int at = Math.Min(Math.Max(i, 1), n - 1);
                     return 100.0 * plan.Sigma[at] * plan.Mass[at] / Math.Max(_s.GfoldThrustMax, 1.0);
                 },
                 "mass (kg)", i => plan.Mass[i], n, cursor);
    }

    private static double VecLen(double[] v)
        => Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    // Two equal columns, measured once, because the room that is left after the first plot is less than half.
    private static void PlotPair(string labelA, Func<int, double> a,
                                 string labelB, Func<int, double> b, int n, float cursor)
    {
        float gap = ImGui.GetStyle().ItemSpacing.X;
        float width = MathF.Max(120f * ImGuiHelper.InterfaceScale, (ConsoleStyle.ContentAvailWidth() - gap) * 0.5f);
        PlotSeries(labelA, a, n, cursor, width);
        ImGui.SameLine(0f, gap);
        PlotSeries(labelB, b, n, cursor, width);
    }

    private static void PlotSeries(string label, Func<int, double> series, int n, float cursor, float width)
    {
        float scale = ImGuiHelper.InterfaceScale;
        var size = new float2(width, 84f * scale);
        float2 origin = ImGui.GetCursorScreenPos();
        ImGui.Dummy(size);
        ConsoleWidgets.InputWell(origin, size);
        ImDrawListPtr dl = ImGui.GetWindowDrawList();

        double min = double.PositiveInfinity, max = double.NegativeInfinity;
        for (int i = 0; i < n; i++)
        {
            double v = series(i);
            if (!double.IsFinite(v)) continue;
            if (v < min) min = v;
            if (v > max) max = v;
        }

        float pad = 6f * scale;
        ConsoleUi.PushSmall();
        dl.PushClipRect(origin, origin + size, true);
        try
        {
            float font = ImGui.GetFontSize();
            float titleY = origin.Y + 3f * scale;
            ImColor8 nameInk = Palette.Text.Solid, valueInk = Palette.Dim.Solid;
            if (!double.IsFinite(min))
            {
                dl.AddText(new float2(origin.X + pad, titleY), Palette.Warn.Solid, label + ", no data");
                return;
            }

            double range = Math.Max(Math.Abs(min), Math.Abs(max));
            int ci = Math.Clamp((int)MathF.Round(cursor * (n - 1)), 0, n - 1);
            string now = "now " + TuneRules.PlotValue(series(ci), range);
            float nowW = ImGui.CalcTextSize(now).X;
            dl.AddText(new float2(origin.X + size.X - pad - nowW, titleY), nameInk, now);
            dl.AddText(new float2(origin.X + pad, titleY), valueInk, ConsoleUi.FitEllipsis(label, size.X - pad * 3f - nowW));

            string maxText = TuneRules.PlotValue(max, range), minText = TuneRules.PlotValue(min, range);
            float lane = MathF.Max(ImGui.CalcTextSize(maxText).X, ImGui.CalcTextSize(minText).X) + pad;
            float left = origin.X + lane + pad, right = origin.X + size.X - pad;
            float top = titleY + font + 4f * scale, bottom = origin.Y + size.Y - 5f * scale;
            dl.AddText(new float2(origin.X + lane - ImGui.CalcTextSize(maxText).X, top - font * 0.25f), valueInk, maxText);
            dl.AddText(new float2(origin.X + lane - ImGui.CalcTextSize(minText).X, bottom - font), valueInk, minText);

            double span = max - min;
            if (span < 1e-9) span = 1.0;
            float w = right - left, h = bottom - top;
            var points = new float2[n];
            for (int i = 0; i < n; i++)
            {
                double v = series(i);
                if (!double.IsFinite(v)) v = min;
                points[i] = new float2(
                    left + (n > 1 ? (float)i / (n - 1) : 0f) * w,
                    bottom - (float)((v - min) / span) * h);
            }
            dl.AddPolyline(points, Palette.Plan.Solid, ImDrawFlags.None, 2f);

            float cx = left + cursor * w;
            _ovSeg[0] = new float2(cx, top);
            _ovSeg[1] = new float2(cx, bottom);
            dl.AddPolyline(_ovSeg, Palette.Flown.Solid, ImDrawFlags.None, 1f);
            dl.AddCircleFilled(points[ci], 3f, Palette.Flown.Solid);
        }
        finally
        {
            dl.PopClipRect();
            ConsoleStyle.PopFont();
        }
    }

    // The descent plot and its legend are the LAND picture above this form. The hover handoff is a property of the flight plan, not of the optimiser, so it is in the HAND-OVERS (DrawHandOversFold), and the vehicle height is in the details of the form, because every way of landing reads it. This content is the solver tuning.
    private static void DrawGfoldLandingContent(float innerW)
    {
        if (GuidanceRegion.Begin("G-FOLD PARAMETERS", GuidanceRegion.Tuning))
        {
            // The UPFG-to-G-FOLD handoff gate is deliberately NOT here: it governs when the braking burn ends, which is a descent decision. It lives with the approach parameters.
            DrawTuningGroup("LIMITS", "GfoldLimits", true, Specs.TuningGfoldLimits, null, DrawGfoldLimitRows, PageText.GfoldLimitsTip);
            DrawTuningGroup("TRACKING", "GfoldTracking", true, Specs.TuningGfoldTracking, null, DrawGfoldTrackingRows, PageText.GfoldTrackingTip);
            DrawTuningGroup("SOLVER", "GfoldSolver", false, Specs.TuningGfoldSolver, null, DrawGfoldSolverRows, PageText.GfoldSolverTip);
            GuidanceRegion.End();
        }
    }

    private static void DrawGfoldLimitRows()
    {
        GaugeRow(Specs.GfoldGlideSlopeDeg, ref _s.GfoldGlideSlopeDeg);
        GaugeRow(Specs.GfoldPointingDeg, ref _s.GfoldPointingDeg);
        GaugeRow(Specs.GfoldVMaxMs, ref _s.GfoldVMaxMs);
        GaugeRow(Specs.GfoldThrottleMin, ref _s.GfoldThrottleMin);
        GaugeRow(Specs.GfoldThrottleMax, ref _s.GfoldThrottleMax);
    }

    private static void DrawGfoldTrackingRows()
    {
        GaugeRow(Specs.GfoldKp, ref _s.GfoldKp);
        GaugeRow(Specs.GfoldKd, ref _s.GfoldKd);
        GaugeRow(Specs.GfoldSmoothTau, ref _s.GfoldSmoothTau);
        GaugeRow(Specs.GfoldIntervalS, ref _s.GfoldIntervalS);
    }

    private static void DrawGfoldSolverRows()
    {
        GaugeRowInt(Specs.GfoldNodes, ref _s.GfoldNodes);
        GaugeRow(Specs.GfoldSlewReg, ref _s.GfoldSlewReg);
        // The time limit stays a knob because Clarabel reports hitting it as a first-class status. There is no tolerance knob: an interior-point method's cost scales with log(1/eps), so accuracy is nearly free.
        GaugeRow(Specs.GfoldSolveTimeLimitS, ref _s.GfoldSolveTimeLimitS);
    }
}

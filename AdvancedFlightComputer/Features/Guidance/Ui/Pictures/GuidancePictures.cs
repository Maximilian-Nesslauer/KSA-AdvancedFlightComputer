#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The pictures of the LAUNCH and LAND pages, and the booster lanes of LAUNCH > BOOSTERS. Each draws from the guidance state of the craft on screen and never writes flight state: a switch in a picture header flips a view switch, and a picture that needs work done (an impact prediction, a pass list) marks a ViewDemand the guidance step answers. All of them keep to the lanes of Picture, so a big number, a tick or a legend can never meet a curve, and a label stands in the plot only where Labels finds free room.
public static partial class GuidanceWindow
{
    private static readonly CultureInfo PicInv = CultureInfo.InvariantCulture;
    private static readonly string[] LaunchViews = { "PROFILE", "PITCH", "Q", "THROTTLE" };
    private static int _launchView;
    private static readonly CanvasView _launchCanvas = new();
    private static readonly CanvasView _boostbackCanvas = new();
    private static readonly Labels _pictureLabels = new();
    private static readonly PointBuffer _picturePoints = new();
    private static readonly PointBuffer _pictureFlown = new();
    private static readonly PointBuffer _pictureTrack = new();

    #region LAUNCH

    internal static bool LaunchPictureHasData => LaunchPictureShows;

    private static AscentPlan LaunchPicturePlan => _s.Running ? _s.FlyingPlan ?? _s.AscentPlan : _s.AscentPlan;

    private static bool LaunchPictureShows
        => LaunchPicturePlan is { Series: not null, Solution.Nodes: > 1 } || FlownClimbShows(_s.TraceParent);

    private static bool LaunchOrbitBandDraws => LaunchPictureShows && _launchView == 0;

    private static bool FlownClimbShows(IParentBody parent)
    {
        if (parent == null || _s.Trace == null || _s.TraceCount < 2)
            return false;
        int cap = _s.Trace.Length;
        int start = (_s.TraceHead - _s.TraceCount + cap) % cap;
        double radius = parent.MeanRadius;
        double airTopKm = (parent.GetPhysicalAtmosphere()?.Height ?? 0.0) / 1000.0;
        return SketchMath.TrackIsClimb((_s.Trace[start].Length() - radius) / 1000.0, (_s.Trace[(_s.TraceHead - 1 + cap) % cap].Length() - radius) / 1000.0, airTopKm);
    }

    private static readonly string[] LaunchLegendNames = { "Q-ALPHA", "FLOWN", "RETURN", "EXPENDABLE" };
    private const string FlownKeySample = "SEQ 0, NOW 0";
    private static readonly string[] _launchLegendNames = new string[16];

    private static float LaunchLegendWidth(int stages)
    {
        int count = 0;
        foreach (string name in LaunchLegendNames)
            _launchLegendNames[count++] = name;
        if (StageStackFlew)
            _launchLegendNames[count++] = FlownKeySample;
        for (int stage = 1; stage <= stages && count < _launchLegendNames.Length; stage++)
            _launchLegendNames[count++] = StageKey(stage);
        return Picture.LegendWidth(_launchLegendNames.AsSpan(0, count));
    }

    private static readonly Picture.Swatch[] _launchSwatches = new Picture.Swatch[12];
    private static readonly int[] _shownStages = new int[12];
    private static readonly Picture.Swatch[] _planSwatch = { new("PLAN", Palette.Plan) };
    private static readonly Picture.Swatch[] _qSwatches = { new("Q", Palette.Plan), new("Q-ALPHA", Palette.QAlpha, SwatchKind.LongDash) };

    /// <summary>
    /// FLIGHT PROFILE: altitude against downrange with the target orbit band, the atmosphere, the convex plan with its staging marks and its max-q point, the hand-over altitude, and the track flown so far. The views PITCH, Q and THROTTLE plot the same plan against time. The views are folder tabs on the top edge of the frame, the legend stands in the right lane, and the stage line with the verbs of the plan stands under the frame. With no plan and no climb the picture is one sentence, and the stage line stays. A plan that can no longer fly draws dim with its marks.
    /// </summary>
    internal static void DrawLaunchPicture(Vehicle vehicle, float height)
    {
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;

        bool busy = AscentPlanBusy;
        AscentPlan plan = LaunchPicturePlan;
        bool hasPlan = plan != null;
        AscentPlanSeries series = plan?.Series != null && plan.Solution.Nodes > 1 ? plan.Series : null;
        if (series == null)
            _launchView = 0;

        // The marks of the plan read the same stack as the bar under the picture, which draws later in the frame.
        if (StagingStages is { } stages && stages.Stages.Count > 0)
            StageNumbersOf(stages);

        if (Picture.Shows("AfcLaunchPic", LaunchPictureShows))
        {
            float2 viewRow = ImGui.GetCursorScreenPos();
            float rowW = ConsoleStyle.ContentAvailWidth();
            float choiceW = ConsoleUi.ViewChoiceWidth(PageText.ShowInMap, PageText.MapChoices);
            Picture.Header("FLIGHT PROFILE", series != null ? LaunchViews : Array.Empty<string>(), ref _launchView, "AfcLaunchPic",
                reserve: choiceW, viewCards: PageText.LaunchViewCards);

            float2 underRow = ImGui.GetCursorScreenPos();
            ImGui.SetCursorScreenPos(new float2(viewRow.X + rowW - choiceW, viewRow.Y));
            int map = ConsoleUi.ViewChoice(PageText.ShowInMap, "AfcLaunchMap", PageText.MapChoices, FormRules.MapChoice(_showAscentOverlay, _s.ShowConvexPlan), PageText.MapChoiceTip);
            if (map >= 0)
            {
                (_showAscentOverlay, _s.ShowConvexPlan) = FormRules.ApplyMapChoice(map, _s.ShowConvexPlan);
                SaveOverlayPreferences();
            }
            ImGui.SetCursorScreenPos(underRow);

            bool old = series != null && (!_s.Running || _s.FlyingPlan == null)
                && LaunchPlanWhy(vehicle, orbit, parent).Length > 0;
            using (PictureScope pic = Picture.Open(height))
            {
                PictureFrame f = pic.Frame;
                switch (_launchView)
                {
                    case 1 when series != null:
                        DrawSeriesView(in f, series.Time, series.PitchDeg, null, double.NaN, "", double.NaN, double.NaN, AxisTicks.Degrees,
                            series.StagingNodes, "", _planSwatch, old, axisStep: PitchAxisStepDeg);
                        break;
                    case 2 when series != null:
                        int qLen = series.QFraction.Length;
                        int qaLen = series.QAlphaFraction.Length;
                        if (_qScratch.Length != qLen)
                            _qScratch = new double[qLen];
                        if (_qaScratch.Length != qaLen)
                            _qaScratch = new double[qaLen];
                        double peak = 0.0;
                        for (int i = 0; i < qLen; i++)
                        {
                            _qScratch[i] = 100.0 * series.QFraction[i];
                            if (i < qaLen)
                                _qaScratch[i] = 100.0 * series.QAlphaFraction[i];
                            if (double.IsFinite(series.QFraction[i]))
                                peak = Math.Max(peak, series.QFraction[i]);
                        }
                        PicBox qLegend = DrawSeriesView(in f, series.Time, _qScratch, _qaScratch, 100.0, PageText.QLimits(plan.QMaxKpa, plan.QAlphaMax), 0.0, 100.0, AxisTicks.Percent,
                            series.StagingNodes, PageText.PeakQ(peak * plan.QMaxKpa), _qSwatches, old, boldAtLimit: true, refShort: PageText.QLimitShort(plan.QMaxKpa));
                        if (qLegend.W > 0f)
                        {
                            ImGui.SetCursorScreenPos(new float2(qLegend.X + 4f * f.Scale, qLegend.Bottom - ConsoleStyle.FrameHeightPx));
                            if (ConsoleUi.Link(PageText.QTuneLink, "AfcLaunchPicLink", PageText.QTuneTip))
                            {
                                Ui.Console.AfcConsole.TuneSet = TuneSet.Ascent;
                                PendingNavigation = ConsolePages.CueTarget(ConsolePage.Tune);
                            }
                        }
                        break;
                    case 3 when series != null:
                        DrawSeriesView(in f, series.Time, series.ThrottlePct, null, plan.ThrottleMinPct, PageText.MinThrottle(plan.ThrottleMinPct), 0.0, 100.0, AxisTicks.Percent,
                            series.StagingNodes, "", _planSwatch, old);
                        break;
                    default:
                        DrawProfileView(in f, parent, series != null ? plan : null, old);
                        break;
                }
            }
        }
        else
            Picture.EmptyState(busy ? LaunchProgressText() : PageText.LaunchNoPlan(vehicle.Situation.HasAnyContact()));

        string calc = busy ? "CANCEL" : hasPlan ? "RE-SOLVE" : "SOLVE";
        string[] verbs = _s.Running || _s.ConvexLaunchPending ? Array.Empty<string>() : hasPlan && !busy ? new[] { calc, "CLEAR" } : new[] { calc };
        string[] tips =
        {
            busy ? "Cancels the solve. A launch that waits for it is canceled too."
                : "Solves the climb for the liftoff the launch would commit to, for a preview.",
            "Drops the solved plan.",
        };
        int legend;
        // After lift-off the line and the bar under it say what the craft holds now, in the names of the stock panel and of STAGING.
        bool liveStages = !vehicle.Situation.HasAnyContact();
        string liveStage = "";
        Rgb liveHue = default;
        if (liveStages)
            LiveStageKey(out liveStage, out liveHue);
        using (ConsoleUi.Dim(!_s.Running && !vehicle.Situation.HasAnyContact()))
            legend = Picture.LegendLine(StagingLine(), verbs, tips, "AfcLaunchLegend", key: liveStage, keyHue: liveHue);
        switch (legend)
        {
            case 0 when busy:
                _s.AscentPlanJob?.Cancel();
                _s.AscentPlanRequested = false;
                _s.ConvexLaunchPending = false;
                break;
            case 0:
                RequestAscentPlan(WindowInstant() > SimNow() ? WindowInstant() : double.NaN);
                break;
            case 1:
                _s.AscentPlan = null;
                _s.AscentPlanStatus = "";
                break;
        }

        DrawStagingBarOnly(ConsoleStyle.ContentAvailWidth(), liveStages);
        DrawConvexLaunchBanner();
        ImGui.Spacing();
    }

    private static string LaunchProgressText()
    {
        AscentPlanJob job = _s.AscentPlanJob;
        return job != null ? PageText.PlanProgressShort(job.Progress, job.ElapsedSeconds) : PageText.PlanStarting + ".";
    }

    private static ImColor8 PlanInk(bool old) => old ? Palette.Plan.A(0.4f) : Palette.Plan.Solid;

    private const double PitchAxisStepDeg = 45.0;

    private const float OldPlanAlpha = 0.4f;

    private static ImColor8 StageInk(int stage, bool old) => old ? ((Rgb)ColorRules.Stage(stage)).A(OldPlanAlpha) : Palette.Stage(stage);

    private static double[] _qScratch = Array.Empty<double>();
    private static double[] _qaScratch = Array.Empty<double>();
    private static double[] _traceDownCache = Array.Empty<double>();
    private static double[] _traceAltCache = Array.Empty<double>();
    private static VehicleAutopilotState _traceCacheState;
    private static double3[] _traceCacheRing;
    private static IParentBody _traceCacheParent;
    private static int _traceCacheStart = -1;
    private static int _traceCacheProcessed;
    private static int _traceCacheFlownCount;
    private static double _traceCacheMaxDown;
    private static double _traceCacheTopAlt;
    private static double _traceCacheClimbEnd = double.NaN;
    private static double3 _traceCacheFirst;
    private static readonly List<(float X, string Label)> _plannedSiteMarkers = new();
    private static readonly List<(float X, string Label)> _pendingSiteMarkers = new();

    private static void ResetPictureCaches()
    {
        _traceCacheState = null;
        _traceCacheRing = null;
        _traceCacheParent = null;
        _traceCacheStart = -1;
        _traceCacheProcessed = 0;
        _traceCacheFlownCount = 0;
        _traceCacheMaxDown = 0.0;
        _traceCacheTopAlt = 0.0;
        _traceCacheClimbEnd = double.NaN;
        _traceCacheFirst = default;
    }

    private static void DrawProfileView(in PictureFrame f, IParentBody parent, AscentPlan plan, bool old)
    {
        AscentPlanSeries series = plan?.Series;
        double radius = parent.MeanRadius;
        double atmoKm = (parent.GetPhysicalAtmosphere()?.Height ?? 0.0) / 1000.0;

        // The flown track, from the ring of CCI positions the step records under thrust. Downrange is the arc from the first sample over the rotating ground, the same frame as AscentPlanSeries.Build, so the two lines share one x axis. The body turns about CCI +Z.
        int flownCount = 0;
        double maxDown = 50.0, flownTop = 0.0, planTop = 0.0;
        double[] flownDown = null, flownAlt = null;
        if (FlownClimbShows(parent))
        {
            int cap = _s.Trace.Length;
            int start = (_s.TraceHead - _s.TraceCount + cap) % cap;
            double3 first = double3.Normalize(_s.Trace[start]);
            double[] times = TraceTimes();
            double spin = parent.GetAngularVelocity();
            double climbEnd = ClimbEndTime();
            bool cacheValid = ReferenceEquals(_traceCacheState, _s)
                && ReferenceEquals(_traceCacheRing, _s.Trace)
                && ReferenceEquals(_traceCacheParent, parent)
                && _traceCacheStart == start
                && _s.TraceCount >= _traceCacheProcessed
                && _traceCacheClimbEnd == climbEnd
                && _traceCacheFirst == first;
            if (!cacheValid)
            {
                if (_traceDownCache.Length < cap)
                {
                    _traceDownCache = new double[cap];
                    _traceAltCache = new double[cap];
                }
                _traceCacheState = _s;
                _traceCacheRing = _s.Trace;
                _traceCacheParent = parent;
                _traceCacheStart = start;
                _traceCacheProcessed = 0;
                _traceCacheFlownCount = 0;
                _traceCacheMaxDown = 0.0;
                _traceCacheTopAlt = 0.0;
                _traceCacheClimbEnd = climbEnd;
                _traceCacheFirst = first;
            }
            for (int i = _traceCacheProcessed; i < _s.TraceCount; i++)
            {
                int slot = (start + i) % cap;
                if (times != null && times[slot] > climbEnd)
                    break;
                double3 p = _s.Trace[slot];
                double turn = times != null ? -spin * (times[slot] - times[start]) : 0.0;
                var ground = new double3(p.X * Math.Cos(turn) - p.Y * Math.Sin(turn), p.X * Math.Sin(turn) + p.Y * Math.Cos(turn), p.Z);
                double downKm = AngleBetween(first, ground) * radius / 1000.0;
                double altKm = (p.Length() - radius) / 1000.0;
                _traceDownCache[i] = downKm;
                _traceAltCache[i] = altKm;
                _traceCacheMaxDown = Math.Max(_traceCacheMaxDown, downKm);
                _traceCacheTopAlt = Math.Max(_traceCacheTopAlt, altKm);
                _traceCacheFlownCount = i + 1;
                _traceCacheProcessed = i + 1;
            }
            flownDown = _traceDownCache;
            flownAlt = _traceAltCache;
            flownCount = _traceCacheFlownCount;
            maxDown = Math.Max(maxDown, _traceCacheMaxDown);
            flownTop = Math.Max(flownTop, _traceCacheTopAlt);
        }
        if (series != null)
        {
            for (int i = 0; i < series.DownrangeKm.Length; i++)
            {
                if (double.IsFinite(series.DownrangeKm[i]))
                    maxDown = Math.Max(maxDown, series.DownrangeKm[i]);
                if (double.IsFinite(series.AltitudeKm[i]))
                    planTop = Math.Max(planTop, series.AltitudeKm[i]);
            }
        }
        // Each booster's way down from the plan's separation (AscentPlan.BoosterTracks), on the same ground-fixed downrange: a return to the pad folds back towards zero, a drone ship lies on out downrange.
        BoosterTrackView[] boosterTracks = plan?.BoosterTracks ?? Array.Empty<BoosterTrackView>();
        foreach (BoosterTrackView t in boosterTracks)
        {
            if (double.IsFinite(t.SiteDownrangeKm))
                maxDown = Math.Max(maxDown, t.SiteDownrangeKm);
            if (t.Track.Ok)
                foreach (double d in t.DownrangeKm)
                    if (double.IsFinite(d))
                        maxDown = Math.Max(maxDown, d);
        }
        double topKm = SketchMath.ProfileTopKm(planTop, flownTop, Math.Min(_s.PeKm, _s.ApKm), atmoKm);

        // No decision needs the distance, so it stands small in the head lane and the plot takes the room of a big number. While the ascent flies the head holds the numbers, and the card of those numbers the velocity to go.
        string downrange = !_s.Running && series != null ? string.Format(PicInv, "{0:F0} km DOWNRANGE", series.DownrangeKm[^1]) : "";
        float s = f.Scale;
        ImDrawListPtr dl = f.Dl;
        PictureLanes lanes = Picture.Lanes(in f, Picture.HeadHeight(in f, "", text: true), Picture.TickWidth(AxisTicks.Km(Math.Round(topKm))),
            LaunchLegendWidth(series?.StagingNodes?.Length ?? 0), Picture.TextLaneHeight());
        PicBox plot = lanes.Plot;

        CanvasView v = _launchCanvas;
        v.SetRect(new float2(plot.X, plot.Y), new float2(plot.Right, plot.Bottom));
        v.Fit(0.0, maxDown * 1.05, 0.0, topKm);
        CanvasInput input = v.Capture("##AfcLaunchCanvas"u8, zoom: true);
        Picture.ZoomHint(in f, lanes.Head, input.Hovered);
        Picture.LaneText(in f, lanes.Head, downrange, old ? Palette.Dim.A(OldPlanAlpha) : Palette.Dim.Solid, 1);

        Picture.AxisX(in f, in lanes, Math.Max(v.X0, 0.0), v.X1, time: false, x => v.Sx(x), AxisTicks.Km);
        Picture.AxisY(in f, in lanes, Math.Max(v.Y0, 0.0), v.Y1, y => v.Sy(y), y => Math.Abs(y) < 1e-6 ? "" : AxisTicks.Km(y));

        Labels labels = _pictureLabels;
        labels.Begin(dl, in plot, s);
        labels.Reserve(new PicBox(plot.X, plot.Bottom - StageRibbonHeight * s, plot.W, StageRibbonHeight * s));

        double highKm = Math.Max(_s.PeKm, _s.ApKm), lowKm = Math.Min(_s.PeKm, _s.ApKm);
        float bandTop = Math.Max(v.Sy(highKm), plot.Y);
        float bandBottom = v.Sy(lowKm);
        bool hasHandover = _s.FlyConvexAscent && double.IsFinite(_s.HandoverAltKm) && (!_s.Running || _s.FlyingPlan != null);
        int n = series != null ? Math.Min(series.DownrangeKm.Length, series.AltitudeKm.Length) : 0;
        int shownStages = 0;

        using (new PlotClip(dl, in plot))
        {
            if (atmoKm > 0.0)
            {
                dl.AddRectFilled(new float2(plot.X, v.Sy(atmoKm)), new float2(plot.Right, v.Sy(0.0)), Palette.Air.A(0.08f), 0f);
                Picture.GuideY(in f, in plot, v.Sy(atmoKm), Palette.Air.A(0.6f));
            }
            dl.AddRectFilled(new float2(plot.X, bandTop - 1f * s), new float2(plot.Right, bandBottom + 1f * s), Palette.Target.A(0.15f), 0f);
            if (hasHandover)
                Picture.GuideY(in f, in plot, v.Sy(_s.HandoverAltKm), Palette.Dim.A(0.6f));

            if (n > 0 && series.StagingNodes != null)
                DrawStageRibbon(in f, in plot, series.StagingNodes, series.DownrangeKm, n, x => v.Sx(x), old);

            if (n > 0)
            {
                float2[] pts = _picturePoints.Get(n);
                int m = 0;
                for (int i = 0; i < n; i++)
                {
                    if (double.IsFinite(series.DownrangeKm[i]) && double.IsFinite(series.AltitudeKm[i]))
                        pts[m++] = v.ToScreen(series.DownrangeKm[i], series.AltitudeKm[i]);
                }
                if (m > 1)
                {
                    dl.AddPolyline(pts.AsSpan(0, m), PlanInk(old), ImDrawFlags.None, 2f * s);
                    labels.Curve(pts.AsSpan(0, m));
                }
            }

            foreach (BoosterTrackView t in boosterTracks)
            {
                int tn = t.Track.Ok ? Math.Min(t.DownrangeKm.Length, t.AltitudeKm.Length) : 0;
                if (tn < 2)
                    continue;
                float2[] tp = _pictureTrack.Get(tn);
                for (int i = 0; i < tn; i++)
                    tp[i] = v.ToScreen(t.DownrangeKm[i], Math.Max(t.AltitudeKm[i], 0.0));
                bool expendable = t.Mode == BoosterReturnMode.Expendable;
                Rgb hue = expendable ? Palette.Dim : t.Mode == BoosterReturnMode.OptimalReturn ? Palette.Target : Palette.Good;
                ImColor8 ink = hue.A(expendable ? 0.5f : old ? OldPlanAlpha : 1f);
                dl.AddPolyline(tp.AsSpan(0, tn), ink, ImDrawFlags.None, (expendable ? 1f : 1.5f) * s);
                if (expendable)
                    Glyphs.Ring(dl, tp[tn - 1], 3.5f * s, ink, 1f * s);
                else
                    Glyphs.Dot(dl, tp[tn - 1], 3.5f * s, ink);
                labels.Mark(tp[tn - 1], 3.5f * s);
            }

            if (flownCount > 1)
            {
                float2[] flown = _pictureFlown.Get(flownCount);
                for (int i = 0; i < flownCount; i++)
                    flown[i] = v.ToScreen(flownDown[i], flownAlt[i]);
                dl.AddPolyline(flown.AsSpan(0, flownCount), Palette.Flown.Solid, ImDrawFlags.None, 2f * s);
                Glyphs.Dot(dl, flown[flownCount - 1], 4f * s, Palette.Flown.Solid);
                labels.Curve(flown.AsSpan(0, flownCount));
                labels.Mark(flown[flownCount - 1], 4f * s);
            }

            for (int stage = 1; n > 0 && series.StagingNodes != null && stage <= series.StagingNodes.Length; stage++)
            {
                int node = series.StagingNodes[stage - 1];
                if (node < 0 || node >= n)
                    continue;
                float2 at = v.ToScreen(series.DownrangeKm[node], series.AltitudeKm[node]);
                Glyphs.Diamond(dl, at, 4f * s, StageInk(stage, old));
                labels.Mark(at, 4f * s);
                if (shownStages < _shownStages.Length)
                    _shownStages[shownStages++] = stage;
            }
        }

        bool targetNamed = labels.Guide(v.Sy(highKm) < plot.Y ? "ORBIT, AP " + Fmt.Km(highKm) : "ORBIT", bandBottom, Palette.Target.Solid);
        bool handoverNamed = true;
        if (hasHandover)
        {
            handoverNamed = labels.Guide("UPFG FROM " + Fmt.Km(_s.HandoverAltKm), v.Sy(_s.HandoverAltKm), Palette.Dim.Solid);
        }
        bool airNamed = atmoKm <= 0.0 || labels.Guide("AIR", v.Sy(atmoKm), Palette.Air.Solid);
        if (plan != null && v.Y0 <= 0.0 && v.Y1 >= 0.0 && plot.W > 8f * s && plot.H > 8f * s)
            DrawProfileSites(dl, labels, v, in plot, s, plan, old);
        int qPeak = n > 0 ? ArgMaxFinite(series.QFraction) : -1;
        if (qPeak >= 0 && qPeak < n)
        {
            float2 ring = v.ToScreen(series.DownrangeKm[qPeak], series.AltitudeKm[qPeak]);
            float labelH = labels.Size("MAX Q").Y;
            float low = plot.Bottom - StageRibbonHeight * s - labelH - 1f * s;
            ImColor8 ringInk = old ? Palette.Text.A(OldPlanAlpha) : Palette.Text.Solid;
            float spotY = Math.Min(ring.Y - labelH * 0.5f, low);
            if (labels.Spots("MAX Q", new[] { new float2(ring.X + 8f * s, spotY), new float2(ring.X + 24f * s, spotY), new float2(ring.X + 44f * s, spotY) }, ringInk))
                Glyphs.Ring(dl, ring, 5f * s, ringInk, 1.5f * s);
        }

        int entries = 0;
        if (series != null)
            _launchSwatches[entries++] = new Picture.Swatch("PLAN", Palette.Plan, Alpha: old ? OldPlanAlpha : 1f);
        if (flownCount > 1)
            _launchSwatches[entries++] = new Picture.Swatch("FLOWN", Palette.Flown);
        foreach (BoosterTrackView t in boosterTracks)
            if (t.Track.Ok && t.Mode != BoosterReturnMode.Expendable)
            {
                _launchSwatches[entries++] = new Picture.Swatch("RETURN", Palette.Good, Alpha: old ? OldPlanAlpha : 1f);
                break;
            }
        foreach (BoosterTrackView t in boosterTracks)
            if (t.Track.Ok && t.Mode == BoosterReturnMode.Expendable)
            {
                _launchSwatches[entries++] = new Picture.Swatch("EXPENDABLE", Palette.Dim, Alpha: 0.5f);
                break;
            }
        if (!targetNamed)
            _launchSwatches[entries++] = new Picture.Swatch("ORBIT", Palette.Target, SwatchKind.Filled);
        if (!handoverNamed)
            _launchSwatches[entries++] = new Picture.Swatch("UPFG", Palette.Dim, SwatchKind.Dashed);
        if (!airNamed)
            _launchSwatches[entries++] = new Picture.Swatch("AIR", Palette.Air, SwatchKind.Dashed);
        for (int i = 0; i < shownStages && entries < _launchSwatches.Length; i++)
            _launchSwatches[entries++] = new Picture.Swatch(StageKey(_shownStages[i]), ColorRules.Stage(_shownStages[i]), SwatchKind.Diamond, old ? OldPlanAlpha : 1f);
        Picture.Legend(in f, lanes.Right, _launchSwatches.AsSpan(0, entries));
    }

    private const float StageRibbonHeight = 4f;

    private static void DrawStageRibbon(in PictureFrame f, in PicBox plot, int[] nodes, double[] xs, int n, Func<double, float> toX, bool old)
    {
        float top = plot.Bottom - StageRibbonHeight * f.Scale;
        float from = float.NaN;
        for (int i = 0; i < n && double.IsNaN(from); i++)
        {
            if (double.IsFinite(xs[i]))
                from = toX(xs[i]);
        }
        float last = float.NaN;
        for (int i = n - 1; i >= 0 && double.IsNaN(last); i--)
        {
            if (double.IsFinite(xs[i]))
                last = toX(xs[i]);
        }
        if (float.IsNaN(from) || float.IsNaN(last))
            return;
        for (int stage = 0; stage <= nodes.Length; stage++)
        {
            int node = stage < nodes.Length ? nodes[stage] : -1;
            float to = stage == nodes.Length ? last : node >= 0 && node < n && double.IsFinite(xs[node]) ? toX(xs[node]) : float.NaN;
            if (float.IsNaN(to))
                continue;
            f.Dl.AddRectFilled(new float2(from, top), new float2(to, plot.Bottom), StageInk(stage, old), 0f);
            from = to;
        }
    }

    private static int ArgMaxFinite(double[] v)
    {
        int best = -1;
        for (int i = 0; i < v.Length; i++)
            if (double.IsFinite(v[i]) && (best < 0 || v[i] > v[best]))
                best = i;
        return best;
    }

    // One or two plan series against the time of the plan. The y axis is fitted to the data and rounded outward, and it holds at least the range from yLo to yHi where those are finite, in steps of axisStep where that is finite. The reference line marks a limit, and its name stands in the head lane because it carries the meaning of the axis, in the form of refShort where the long one does not fit. A lane on top of the plot holds a short tag at each staging line, in the colour of its stage, so no curve can remove the name. With boldAtLimit a curve is thick where it lies at the reference line. Returns the legend lane.
    private static PicBox DrawSeriesView(in PictureFrame f, double[] xs, double[] ys, double[] ys2, double refY, string refLabel, double yLo, double yHi,
        Func<double, string> yText, int[] marks, string big, ReadOnlySpan<Picture.Swatch> legend, bool old, double axisStep = double.NaN, bool boldAtLimit = false, string refShort = "")
    {
        int n = Math.Min(xs.Length, ys.Length);
        double x0 = double.PositiveInfinity, x1 = double.NegativeInfinity, y0 = double.PositiveInfinity, y1 = double.NegativeInfinity;
        for (int i = 0; i < n; i++)
        {
            if (!double.IsFinite(xs[i]) || !double.IsFinite(ys[i]))
                continue;
            x0 = Math.Min(x0, xs[i]);
            x1 = Math.Max(x1, xs[i]);
            y0 = Math.Min(y0, ys[i]);
            y1 = Math.Max(y1, ys[i]);
            if (ys2 != null && double.IsFinite(ys2[i]))
            {
                y0 = Math.Min(y0, ys2[i]);
                y1 = Math.Max(y1, ys2[i]);
            }
        }
        if (!double.IsFinite(x0) || n < 2 || !(x1 > x0))
        {
            Picture.Empty(in f, "NO DATA");
            return default;
        }
        if (double.IsFinite(refY))
        {
            y0 = Math.Min(y0, refY);
            y1 = Math.Max(y1, refY);
        }
        if (double.IsFinite(yLo))
            y0 = Math.Min(y0, yLo);
        if (double.IsFinite(yHi))
            y1 = Math.Max(y1, yHi);
        double yStep = double.IsFinite(axisStep) ? axisStep : AxisTicks.NiceStep(Math.Max(y1 - y0, 1e-6), 4);
        (y0, y1) = AxisTicks.Outward(y0, y1, yStep);

        float s = f.Scale;
        ImDrawListPtr dl = f.Dl;
        PictureLanes lanes = Picture.Lanes(in f, Picture.HeadHeight(in f, big, text: refLabel.Length > 0), Picture.TickWidth(yText(y1), yText(y0)),
            LaunchLegendWidth(marks?.Length ?? 0), Picture.TextLaneHeight());
        PicBox plot = lanes.Plot;
        bool tagged = marks != null && marks.Length > 0;
        float tagLane = tagged ? Picture.TextLaneHeight() : 3f * s;
        float top = plot.Y + tagLane, bottom = plot.Bottom - (StageRibbonHeight + 2f) * s;
        double sx = plot.W / (x1 - x0);
        double sy = (bottom - top) / Math.Max(y1 - y0, 1e-9);
        float X(double x) => (float)(plot.X + (x - x0) * sx);
        double y0Local = y0;
        float Y(double y) => (float)(bottom - (y - y0Local) * sy);

        Picture.AxisX(in f, in lanes, x0, x1, time: true, X, AxisTicks.Time);
        Picture.AxisY(in f, in lanes, y0, y1, Y, yText, axisStep);

        using (new PlotClip(dl, in plot))
        {
            if (double.IsFinite(refY))
                Picture.GuideY(in f, in plot, Y(refY), Palette.Dim.A(0.9f));
            if (marks != null)
                DrawStageRibbon(in f, in plot, marks, xs, n, X, old);
            if (marks != null)
            {
                int stage = 1;
                foreach (int k in marks)
                {
                    if (k >= 0 && k < n && double.IsFinite(xs[k]))
                        Glyphs.DashedLine(dl, new float2(X(xs[k]), top), new float2(X(xs[k]), plot.Bottom), StageInk(stage, old), 1f * s, 3f * s, 3f * s);
                    stage++;
                }
            }
            float2[] pts = _picturePoints.Get(n);
            DrawPlanSeries(dl, xs, ys, n, X, Y, pts, PlanInk(old), boldAtLimit ? refY : double.NaN, s);
            if (ys2 != null)
                DrawPlanSeries(dl, xs, ys2, n, X, Y, pts, old ? Palette.QAlpha.A(OldPlanAlpha) : Palette.QAlpha.Solid, boldAtLimit ? refY : double.NaN, s, longDash: true);
        }

        if (tagged)
        {
            ConsoleUi.PushSmall();
            float tagY = plot.Y + (tagLane - ImGui.GetFontSize()) * 0.5f;
            float nextFree = plot.X;
            int stage = 1;
            foreach (int k in marks)
            {
                if (k >= 0 && k < n && double.IsFinite(xs[k]))
                {
                    string tag = StageNumbers.ShortTag(StageTag(stage));
                    float w = ImGui.CalcTextSize(tag).X;
                    float x = Math.Min(Math.Max(X(xs[k]) - w * 0.5f, nextFree), plot.Right - w);
                    dl.AddText(new float2(x, tagY), StageInk(stage, old), tag);
                    nextFree = x + w + 3f * s;
                }
                stage++;
            }
            ConsoleStyle.PopFont();
        }

        if (double.IsFinite(refY) && refLabel.Length > 0)
        {
            PicBox head = lanes.Head;
            float keyY = head.Y + head.H * 0.5f;
            Glyphs.DashedLine(dl, new float2(head.X + 4f * s, keyY), new float2(head.X + 24f * s, keyY), Palette.Dim.Solid, 1f * s, 5f * s, 2.5f * s);
            var nameBox = new PicBox(head.X + 25f * s, head.Y, head.W - 25f * s, head.H);
            string refName = refShort.Length > 0 && !Picture.HeadTextFits(in f, in nameBox, refLabel, big) ? refShort : refLabel;
            Picture.HeadText(in f, in nameBox, refName, Palette.Dim.Solid, big);
        }
        Picture.Legend(in f, lanes.Right, legend);
        Picture.BigNumber(in f, big, old ? Palette.Dim.Solid : Palette.Text.Solid);
        return lanes.Right;
    }

    private static void DrawPlanSeries(ImDrawListPtr dl, double[] xs, double[] ys, int n, Func<double, float> toX, Func<double, float> toY, float2[] pts, ImColor8 ink, double limit, float s, bool longDash = false)
    {
        int m = 0;
        for (int i = 0; i < n && i < ys.Length; i++)
        {
            if (double.IsFinite(xs[i]) && double.IsFinite(ys[i]))
                pts[m++] = new float2(toX(xs[i]), toY(ys[i]));
        }
        if (longDash)
            Glyphs.DashedPolyline(dl, pts.AsSpan(0, m), ink, 2f * s, 8f * s, 4f * s);
        else
            dl.AddPolyline(pts.AsSpan(0, m), ink, ImDrawFlags.None, 2f * s);
        if (!double.IsFinite(limit))
            return;
        for (int i = 1; i < n && i < ys.Length; i++)
        {
            if (double.IsFinite(xs[i - 1]) && double.IsFinite(xs[i]) && SketchMath.AtLimit(ys[i - 1], limit) && SketchMath.AtLimit(ys[i], limit))
                dl.AddLine(new float2(toX(xs[i - 1]), toY(ys[i - 1])), new float2(toX(xs[i]), toY(ys[i])), ink, 4f * s);
        }
    }

    #endregion

    #region BOOSTBACK

    private static readonly string[] _sideLegendNames = { "PREDICTED" };
    private static readonly Picture.Swatch[] _sideSwatches = { new("PREDICTED", Palette.Plan, SwatchKind.Dashed) };

    /// <summary>
    /// RETURN PROFILE of LAND > BOOSTBACK: the side view of the booster, distance to the site against altitude, with the predicted path, the impact point and the miss as the big number. On the pad, before any booster flies, it says where the stage sites are set.
    /// </summary>
    internal static void DrawBoostbackPicture(Vehicle vehicle, float height)
    {
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;

        bool onPad = !BoostbackLive && vehicle.Situation.HasAnyContact();
        if (Picture.Shows("AfcBoostbackPic", BoostbackPictureHasData(vehicle)))
        {
            int noView = 0;
            if (Picture.Header("RETURN PROFILE", default, ref noView, "AfcBoostbackPic",
                    new[] { PageText.ShowInWorld }, new[] { _showImpactOverlay },
                    new[] { "Draws the predicted impact trajectory and point in the world. The side view and IMPACT PREDICTION show it with this switch on or off." }) == 0)
            {
                _showImpactOverlay = !_showImpactOverlay;
                SaveOverlayPreferences();
                if (_showImpactOverlay)
                    _s.ImpactTick = 0;
            }

            using (PictureScope pic = Picture.Open(height))
                DrawReturnSideView(in pic.Frame, vehicle, orbit, parent);
        }
        else if (onPad || HasImpactPath)
            Picture.EmptyState(onPad ? (NoStageHome ? PageText.BoostbackNoStage : PageText.BoostbackNotYet) : PageText.NoSiteToDraw);

        if (!BoostbackLive && !NoStageHome && Picture.LegendLine("", default, default, "AfcBoostbackLegend", default, "STAGE SITES: LAUNCH",
                "Opens the booster return settings on LAUNCH.") == 0)
        {
            CraftActivity activity = ReadMode(vehicle).Activity;
            if (FlightView.PageFlies(ConsolePage.Launch, LandWay.FromOrbit, LandSolver.Entry, CurrentAscentChoice(), in activity, transferPasses: false))
                ConsoleUi.OpenFold(FlightView.PlanFoldId);
            PendingNavigation = ConsolePages.CueTarget(ConsolePage.Launch);
        }
    }

    private static void DrawProfileSites(ImDrawListPtr draw, Labels labels, CanvasView view, in PicBox plot,
                                         float scale, AscentPlan plan, bool old)
    {
        List<(float X, string Label)> planned = _plannedSiteMarkers;
        List<(float X, string Label)> pending = _pendingSiteMarkers;
        planned.Clear();
        pending.Clear();
        foreach (BoosterTrackView track in plan.BoosterTracks)
            if (track.SiteDownrangeKm >= view.X0 && track.SiteDownrangeKm <= view.X1 && track.SiteLabel.Length > 0)
                planned.Add((view.Sx(track.SiteDownrangeKm), track.SiteLabel));

        if (!_s.Running && plan.StartCcf.LengthSquared() > 0.0)
        {
            var stages = _s.ReturnableStages;
            for (int i = 0; i < stages.Count; i++)
            {
                ReturnableStage stage = stages[i];
                if (!stage.HasTarget || !double.IsFinite(stage.TargetLatDeg) || !double.IsFinite(stage.TargetLonDeg)
                    || PlanHasBoosterSite(plan, stage))
                    continue;
                double down = AngleBetween(plan.StartCcf, SiteDirCcf(stage.TargetLatDeg, stage.TargetLonDeg)) * plan.BodyRadius / 1000.0;
                if (down >= view.X0 && down <= view.X1)
                    pending.Add((view.Sx(down), ReturnableStageIdText(stages, i)));
            }
        }

        float left = plot.X + 4f * scale, right = plot.Right - 4f * scale;
        float ground = view.Sy(0.0) - StageRibbonHeight * scale;
        float top = plot.Y + 4f * scale, bottom = plot.Bottom - 4f * scale;
        DrawGroups(planned, Math.Clamp(ground, top, bottom), isPending: false);
        DrawGroups(pending, Math.Clamp(ground - 10f * scale, top, bottom), isPending: true);

        void DrawGroups(List<(float X, string Label)> markers, float y, bool isPending)
        {
            markers.Sort((a, b) => a.X.CompareTo(b.X));
            for (int i = 0; i < markers.Count;)
            {
                float x = markers[i].X;
                string names = markers[i++].Label;
                while (i < markers.Count && markers[i].X - x < 8f * scale)
                    names += "/" + markers[i++].Label;
                var position = new float2(Math.Clamp(x, left, right), y);
                ImColor8 ink = isPending ? Palette.Warn.Solid : Palette.Good.A(old ? OldPlanAlpha : 1f);
                if (isPending)
                    Glyphs.Ring(draw, position, 4f * scale, ink, scale);
                else
                    Glyphs.Diamond(draw, position, 4f * scale, ink);
                labels.Mark(position, 4f * scale);
                labels.Point(names + (isPending ? " PENDING" : " SITE"), position, 6f * scale, ink);
            }
        }
    }

    private static bool PlanHasBoosterSite(AscentPlan plan, ReturnableStage stage)
    {
        string key = RecoverySiteKey(stage);
        foreach (BoosterTrackView track in plan.BoosterTracks)
            if (track.RootId == stage.RootId && track.SiteKey == key)
                return true;
        return false;
    }

    private static bool FallPictureShows(IParentBody parent) => HasImpactPath && !NoSitePicked(parent);

    private static bool BoostbackPictureHasData(Vehicle vehicle)
    {
        bool onPad = !BoostbackLive && vehicle.Situation.HasAnyContact();
        return BoostbackLive || (!onPad && HasImpactPath && !NoSitePicked(vehicle.Orbit?.Parent));
    }

    private static bool HasImpactPath => _s.HasImpact && _s.ImpactPathCcf != null && Math.Min(_s.ImpactPathCount, _s.ImpactPathCcf.Length) > 1;

    private static void DrawFallPicture(Vehicle vehicle, Orbit orbit, IParentBody parent, float height)
    {
        ViewDemand.Mark(vehicle, ViewDemand.Kind.Aero);
        ViewDemand.Mark(vehicle, ViewDemand.Kind.Impact);
        bool shows = FallPictureShows(parent);
        ConsoleUi.NoteShape("AfcFallPic", shows ? 1 : 0);
        if (!shows)
            return;
        using (PictureScope pic = Picture.Open(height))
            DrawReturnSideView(in pic.Frame, vehicle, orbit, parent);
    }

    private static void DrawReturnSideView(in PictureFrame f, Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        float s = f.Scale;
        double radius = parent.MeanRadius;
        double3 site = SiteDirCcf();
        double3 craftCcf = orbit.StateVectors.PositionCci.Transform(parent.GetCci2Ccf());
        double craftKm = AngleBetween(craftCcf, site) * radius / 1000.0;
        int n = _s.HasImpact && _s.ImpactPathCcf != null ? Math.Min(_s.ImpactPathCount, _s.ImpactPathCcf.Length) : 0;
        // The zero of the height is the ground where the path ends, because a path ends on the terrain and the mean radius can lie far under it. With no path it is the ground of the site.
        double groundRadius = n > 0 ? _s.ImpactPathCcf[n - 1].Length() : radius + SiteTerrainHeight(parent);
        double craftAlt = (craftCcf.Length() - groundRadius) / 1000.0;

        double maxKm = Math.Max(craftKm, 5.0), maxAlt = Math.Max(craftAlt, 1.0);
        for (int i = 0; i < n; i++)
        {
            double3 p = _s.ImpactPathCcf[i];
            maxKm = Math.Max(maxKm, AngleBetween(p, site) * radius / 1000.0);
            maxAlt = Math.Max(maxAlt, (p.Length() - groundRadius) / 1000.0);
        }

        double miss = ImpactMissDistance(parent);
        // Green is for what a mode that flies confirms, and a glide or an entry on its plan is what can say that. A burn that is still to come moves the point. Entry guidance with no plan confirms nothing, so its fall is the crash it looks like.
        bool glideFlies = BoostbackLive && _s.BoostbackPhase == BoostbackPhase.Glide;
        bool entryOnPlan = ThreeDofLive && _s.ThreeDofPlan != null && !_s.ThreeDofRefusing;
        bool confirms = glideFlies || entryOnPlan;
        ImpactMark mark = ImpactMarks.Of(miss / 1000.0, confirms, burnToCome: (BoostbackLive && !glideFlies) || entryOnPlan);
        ImColor8 impactInk = mark.Ink switch
        {
            ImpactInk.Good => Palette.Good.Solid,
            ImpactInk.Plain => Palette.Text.Solid,
            _ => Palette.Bad.Solid,
        };
        bool farFromSite = FormRules.SiteOutOfReach(craftKm * 1000.0, ImpactSeconds(vehicle, guidanceBound: true), vehicle.GetSurfaceSpeed());
        PlanTone bigTone = ColorRules.MissTone(miss / 1000.0, confirms, farFromSite, PlanWords.OnTargetKm);
        string big = !double.IsFinite(miss) ? "NO IMPACT PREDICTION" : BoostbackLive ? "" : "MISS " + Fmt.KmFine(miss / 1000.0);
        ImDrawListPtr dl = f.Dl;
        PictureLanes lanes = Picture.Lanes(in f, Picture.HeadHeight(in f, big, text: true), Picture.TickWidth(AxisTicks.Km(Math.Round(maxAlt * 1.2))),
            Picture.LegendWidth(_sideLegendNames), Picture.TextLaneHeight());
        PicBox plot = lanes.Plot;

        CanvasView v = _boostbackCanvas;
        v.SetRect(new float2(plot.X, plot.Y), new float2(plot.Right, plot.Bottom));
        v.Fit(-maxKm * 1.08, maxKm * 0.08, -maxAlt * 0.08, maxAlt * 1.2);
        CanvasInput input = v.Capture("##AfcBoostbackCanvas"u8, zoom: true);
        Picture.ZoomHint(in f, lanes.Head, input.Hovered, big);
        if (!input.Hovered)
            Picture.HeadText(in f, lanes.Head, PageText.HeightAboveGround, Palette.Dim.Solid, big);

        Picture.AxisX(in f, in lanes, v.X0, Math.Min(v.X1, 0.0), time: false, x => v.Sx(x), x => x == 0.0 ? PageText.SiteTick : AxisTicks.Km(-x));
        Picture.AxisY(in f, in lanes, Math.Max(v.Y0, 0.0), v.Y1, y => v.Sy(y), AxisTicks.Km);

        Labels labels = _pictureLabels;
        labels.Begin(dl, in plot, s);
        float2 impact = default;
        using (new PlotClip(dl, in plot))
        {
            dl.AddLine(new float2(plot.X, v.Sy(0.0)), new float2(plot.Right, v.Sy(0.0)), Palette.Grid.Solid, 1f * s);
            if (n > 1)
            {
                float2[] pts = _picturePoints.Get(n);
                for (int i = 0; i < n; i++)
                {
                    double3 p = _s.ImpactPathCcf[i];
                    pts[i] = v.ToScreen(-AngleBetween(p, site) * radius / 1000.0, (p.Length() - groundRadius) / 1000.0);
                }
                Glyphs.DashedPolyline(dl, pts.AsSpan(0, n), Palette.Plan.Solid, 1.5f * s, 5f * s, 3f * s);
                impact = pts[n - 1];
                labels.Curve(pts.AsSpan(0, n));
                labels.Mark(impact, 8f * s);
            }
            float2 siteAt = v.ToScreen(0.0, 0.0);
            Glyphs.Site(dl, siteAt, 5f * s, Palette.Target.Solid, 1.5f * s);
            labels.Mark(siteAt, 8f * s);
            // The cross stands over the ring and is larger than it, so a crash on the site does not hide in the mark of the site.
            if (n > 1)
            {
                Glyphs.Cross(dl, impact, 7f * s, Palette.Shadow, 4.5f * s);
                Glyphs.Cross(dl, impact, 7f * s, impactInk, 2.5f * s);
            }
            float2 craft = v.ToScreen(-craftKm, craftAlt);
            Glyphs.Dot(dl, craft, 4f * s, Palette.Flown.Solid);
            labels.Mark(craft, 5f * s);
        }
        if (n > 1)
            labels.Point(mark.Word, impact, 9f * s, impactInk);

        Picture.Legend(in f, lanes.Right, _sideSwatches);
        Picture.BigNumber(in f, big, Palette.Tone(bigTone).Solid);
    }

    #endregion

    #region LAND

    /// <summary>The LAND picture of the way on screen: the return profile of a booster, the approach and the passes, the powered descent plot with a fall side view before it, or the hover dial. With no <paramref name="sentence"/> a picture that cannot draw has no box with its one sentence.</summary>
    internal static void DrawLandPicture(Vehicle vehicle, float height, LandWay way, bool sentence = true)
    {
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;
        if (way == LandWay.Boostback)
        {
            DrawBoostbackPicture(vehicle, height);
            return;
        }

        if (OrbitalEntryPictureFor(vehicle, way))
        {
            DrawOrbitalEntryPicture(height, sentence);
            return;
        }

        SituationFacts facts = Ui.Console.ConsoleFrame.ReadFacts(vehicle);
        CraftSituation where = facts.Where;
        switch (way)
        {
            // Where there is no air, ENTRY/BRAKING is the braking burn of the deorbit's landing machine: while that flies its picture is the deorbit's approach, the same one DEORBIT draws.
            case LandWay.FromOrbit:
            case LandWay.Entry when !facts.HasAir && DescentLive:
                DrawDeorbitPicture(vehicle, orbit, parent, height, where);
                break;
            case LandWay.Hover:
                DrawHoverPicture(vehicle, orbit, parent, height, where);
                break;
            default:
                DrawHerePicture(vehicle, orbit, parent, height, where, sentence);
                break;
        }
    }

    // The approach only while the deorbit chain flies, because before that there is no path to draw. The passes while the orbit is one, and above them stands nothing else: they are the picture of the decision.
    private static void DrawDeorbitPicture(Vehicle vehicle, Orbit orbit, IParentBody parent, float height, CraftSituation where)
    {
        bool live = DescentLive;
        bool strip = where == CraftSituation.Orbit && (live || !NoSitePicked(parent));
        ConsoleUi.NoteShape("AfcLandPic", (live ? 1 : 0) + (strip ? 2 : 0) + (where == CraftSituation.Ground ? 4 : 0));
        if (live)
            using (PictureScope pic = Picture.Open(height))
                DrawApproachView(in pic.Frame, vehicle, orbit, parent, strip);
        else if (SituationText.IsFalling(where))
            DrawFallPicture(vehicle, orbit, parent, height);

        if (strip && !live)
            DrawPassStrip(big: true);
        else if (strip)
            ViewDemand.Mark(Program.ControlledVehicle, ViewDemand.Kind.Passes);

        if (live && StagingStages is { } stages && stages.Stages.Count > 0)
        {
            LiveStageKey(out string liveStage, out Rgb liveHue);
            Picture.LegendLine(StagingLine(), default, default, "AfcDeorbitStages", key: liveStage, keyHue: liveHue);
            DrawStagingBarOnly(ConsoleStyle.ContentAvailWidth(), live: true);
        }
    }

    private static void DrawHerePicture(Vehicle vehicle, Orbit orbit, IParentBody parent, float height, CraftSituation where, bool sentence)
    {
        if (!Picture.Shows("AfcLandPic", LandPictureHasData(LandWay.FromHere)))
        {
            // Entry guidance with no plan keeps the side view of the fall, because the pilot decides on a take-over from it. The word of the head says that no plan stands.
            if (LandDrawnSolver == LandSolver.Entry && ThreeDofLive)
            {
                ViewDemand.Mark(vehicle, ViewDemand.Kind.Aero);
                ViewDemand.Mark(vehicle, ViewDemand.Kind.Impact);
                if (FallPictureShows(parent))
                {
                    DrawFallPicture(vehicle, orbit, parent, height);
                    return;
                }
            }
            string text = LandEmptyText(where);
            ConsoleUi.NoteShape("AfcLandEmpty", text.Length > 0 ? 1 : 0);
            if (text.Length > 0)
            {
                if (sentence)
                    Picture.EmptyState(text);
            }
            else if (SituationText.IsFalling(where))
                DrawFallPicture(vehicle, orbit, parent, height);
            return;
        }

        bool entry = LandDrawnSolver == LandSolver.Entry;
        if (!entry)
        {
            int noView = 0;
            bool touchdown = PlotShowsTouchdown;
            bool world = touchdown ? _show6DofOverlay : _showGfoldOverlay;
            if (Picture.Header("", default, ref noView, "AfcLandPicHere",
                    new[] { PageText.ShowInWorld }, new[] { world }, new[] { "Draws the planned descent in the world while it flies." }) == 0)
            {
                if (touchdown)
                    _show6DofOverlay = !_show6DofOverlay;
                else
                    _showGfoldOverlay = !_showGfoldOverlay;
                SaveOverlayPreferences();
            }
        }

        using PictureScope pic = Picture.Open(height);
        if (entry)
            Draw3DofPlot(in pic.Frame);
        else
            DrawGfoldPlot(in pic.Frame, vehicle);
    }

    private static void DrawHoverPicture(Vehicle vehicle, Orbit orbit, IParentBody parent, float height, CraftSituation where)
    {
        if (!Picture.Shows("AfcLandPic", _s.LandingPhase == LandingPhase.TerminalHover))
        {
            bool flies = where != CraftSituation.Ground && KsaEnginePerf.SupportsThrottleControl(vehicle);
            double minTwr = flies ? TerminalMinThrottleTwr(vehicle, orbit, parent.Mu) : double.NaN;
            double fullTwr = flies ? TerminalTwr(vehicle, orbit, parent.Mu) : double.NaN;
            bool fails = flies && PageText.HoverThrustFails(minTwr, fullTwr);
            ConsoleUi.NoteShape("AfcHoverVerdict", !flies ? 0 : fails ? 2 : 1);
            if (!fails)
                Picture.EmptyState(PageText.HoverIdle);
            if (fails)
                ConsoleUi.Warning(PageText.HoverVerdict(minTwr, fullTwr).AsSpan());
            else if (flies)
                ConsoleUi.Muted(PageText.HoverVerdict(minTwr, fullTwr).AsSpan());
            return;
        }

        using (PictureScope pic = Picture.Open(height))
            DrawHoverView(in pic.Frame, orbit, parent);
        bool far = PageText.HoverIsFar(orbit.StateVectors.PositionCci.Length() - parent.MeanRadius);
        ConsoleUi.NoteShape("AfcHoverFar", far ? 1 : 0);
        if (far)
            ConsoleUi.Warning(PageText.HoverFarAbove.AsSpan());
    }

    /// <summary>ORBITAL ENTRY's picture: the ENTRY TRAJ plot of its plan and the craft on it, with POP OUT for the same picture in a window of its own.</summary>
    private static void DrawOrbitalEntryPicture(float height, bool sentence)
    {
        var st = _s.OrbitalEntry;
        bool live = OrbitalEntryLive;
        if (!Picture.Shows("AfcEntryPic", live && st.Plan != null && st.Plan.Count >= 2))
        {
            if (sentence)
                Picture.EmptyState(live || st.EngagePending ? PageText.OrbitalEntryFirstPlan : PageText.OrbitalEntryIdle);
            return;
        }
        int noView = 0;
        if (Picture.Header("ENTRY TRAJ", default, ref noView, "AfcEntryPicHead",
                new[] { PageText.PopOut }, new[] { EntryTrajWindowShown }, new[] { PageText.PopOutTip }) == 0)
            ToggleEntryTrajWindow();
        using PictureScope pic = Picture.Open(height);
        DrawEntryTrajInFrame(in pic.Frame);
    }

    private static bool PlotShowsTouchdown => LandSolverRoute == LandSolver.Touchdown || _s.Active;

    // The entry plot needs a plan. A descent plot needs a powered phase that flies, or the path of one that stopped, because the player gets the craft at that moment and the path is what he has to go on.
    private static bool LandPictureHasData(LandWay way)
    {
        if (way == LandWay.Hover)
            return _s.LandingPhase == LandingPhase.TerminalHover;
        if (way != LandWay.FromHere)
            return DescentLive;
        return LandDrawnSolver == LandSolver.Entry ? EntryPlanDrawn(_s.ThreeDofPlan) : PoweredDescentDrawn || PoweredDescentStopped;
    }

    private static string LandEmptyText(CraftSituation where)
    {
        if (LandDrawnSolver == LandSolver.Entry && ThreeDofLive)
            return EntryNoPlanLasts(_s) ? PageText.EntryNoPlan : PageText.EntryFirstPlan;
        if (LandDrawnSolver == LandSolver.Touchdown && _s.SixDofStandby)
            return PageText.TouchdownStandby;
        if (LandDrawnSolver == LandSolver.Touchdown && _s.EngagePending)
            return PageText.TouchdownEngaging;
        return "";
    }

    // Engaged has no caption, because the head says that it flies.
    private static string SixDofCaption()
        => _s.Active ? (_s.Converging ? "TOUCHDOWN: FIRST PLAN" : "")
            : _s.SixDofStandby ? PageText.TouchdownStandby
            : _s.EngagePending ? "TOUCHDOWN: STARTING" : "TOUCHDOWN: IDLE";

    private static readonly string[] _approachLegendNames = { "PROFILE", PageText.CoastKey, PageText.BrakingKey };
    private static readonly double[] _pathKm = new double[160];
    private static readonly double[] _pathAlt = new double[160];
    private static readonly Picture.Swatch[] _approachSwatches = new Picture.Swatch[3];

    // The approach with a broken distance axis: the last kilometres to the site linear on the right, everything farther squeezed on a log scale on the left, and the height on a root scale, so a craft high up and a braking point low down both have room. The craft draws at its place with its name, and a thin white dotted line joins it to where the path after the planned burn starts. The path ends where it first comes down to the braking start, because the coast past that point never flies. From there one dotted line goes to the site, as an estimate, because that part is the braking burn and the descent, which are planned only when they come. The big number is when the arc of the deorbit burn reaches the braking point: DeorbitPlanner.Transfer aims that arc at the site, so the plan holds one arrival and no miss, and a miss of the pass list would be the miss of the orbit before the burn.
    private static void DrawApproachView(in PictureFrame f, Vehicle vehicle, Orbit orbit, IParentBody parent, bool passesShow)
    {
        float s = f.Scale;
        double radius = parent.MeanRadius;
        double3 r = orbit.StateVectors.PositionCci;
        double craftKm = AngleBetween(r, SiteDirCciAt(parent, 0)) * radius / 1000.0;
        double craftAlt = (r.Length() - radius) / 1000.0;
        double brakingKm = _s.BrakingAltitudeKm;
        const double NearKm = 30.0;
        const double NearShare = 0.55;

        // The path ahead. Vehicle.FlightPlan is the coast without burns, so after a planned deorbit node the descent is the trajectory of the last node.
        Burn finalBurn = vehicle.FlightComputer?.BurnPlan?.FindFinalNontrivialBurn();
        FlightPlan plan = finalBurn?.FlightPlan ?? vehicle.FlightPlan;
        int m = 0;
        double farPath = 0.0, topPath = 0.0;
        bool reachedBraking = false;
        if (plan != null && plan.Patches.Count > 0)
        {
            double now = SimNow();
            foreach (PatchedConic patch in plan.Patches)
            {
                if (!ReferenceEquals(patch.Orbit.Parent, parent))
                    continue;
                double t0 = Math.Max(patch.StartTime.Seconds(), now);
                double t1 = Math.Min(patch.EndTime.Seconds(), t0 + Math.Min(double.IsFinite(orbit.Period) ? orbit.Period : 7200.0, 7200.0));
                if (!(t1 > t0))
                    continue;
                for (int k = 0; k <= 60 && m < _pathKm.Length && !reachedBraking; k++)
                {
                    double t = t0 + (t1 - t0) * k / 60.0;
                    double3 p = patch.Orbit.GetStateVectorsAt(new UniverseTime(t)).PositionCci;
                    double alt = (p.Length() - radius) / 1000.0;
                    double km = AngleBetween(p, SiteDirCciAt(parent, t - now)) * radius / 1000.0;
                    _pathKm[m] = km;
                    _pathAlt[m++] = alt;
                    farPath = Math.Max(farPath, km);
                    topPath = Math.Max(topPath, alt);
                    reachedBraking = alt <= brakingKm;
                }
                if (reachedBraking)
                    break;
            }
        }
        double farKm = Math.Max(Math.Max(craftKm, farPath) * 1.1, NearKm * 3.0);
        double maxAlt = Math.Max(Math.Max(Math.Max(craftAlt, topPath), brakingKm * 1.5), 1.0) * 1.05;

        bool burnAhead = IsStockDeorbitPhase(_s.LandingPhase) && _s.DeorbitPlan != null;
        string big = passesShow && burnAhead ? PageText.BrakingIn(Math.Max(0.0, _s.DeorbitPlan.ArrivalTime - SimNow())) : "";

        Span<double> yTicks = stackalloc double[4];
        int yCount = AxisTicks.RootTicks(maxAlt, yTicks);
        ImDrawListPtr dl = f.Dl;
        PictureLanes lanes = Picture.Lanes(in f, Picture.HeadHeight(in f, big), Picture.TickWidth(AxisTicks.Km(yCount > 0 ? yTicks[yCount - 1] : maxAlt)),
            Picture.LegendWidth(_approachLegendNames), Picture.TextLaneHeight());
        PicBox plot = lanes.Plot;
        float left = plot.X + 4f * s, right = plot.Right - 10f * s;
        float top = plot.Y + 6f * s, bottom = plot.Bottom - 9f * s;
        float X(double km) => left + (float)SketchMath.BrokenAxis(km, NearKm, farKm, NearShare) * (right - left);
        float Y(double altKm) => bottom - (float)SketchMath.RootShare(altKm, maxAlt) * (bottom - top);

        Span<double> xTicks = stackalloc double[12];
        int xCount = AxisTicks.BrokenTicks(NearKm, farKm, xTicks);
        Picture.TicksX(in f, in lanes, xTicks.Slice(0, xCount), X, x => x == 0.0 ? PageText.SiteTick : AxisTicks.Km(x));
        Picture.TicksY(in f, in lanes, yTicks.Slice(0, yCount), Y, AxisTicks.Km);

        Labels labels = _pictureLabels;
        labels.Begin(dl, in plot, s);
        float brakingY = Y(brakingKm);
        float gateY = Y(_s.LandingVerticalGateM / 1000.0);
        float siteX = X(0.0);
        var craftAt = new float2(X(craftKm), Y(craftAlt));
        bool coast = false;
        bool toSite = false;
        using (new PlotClip(dl, in plot))
        {
            dl.AddLine(new float2(left, bottom), new float2(right, bottom), Palette.Grid.Solid, 1f * s);
            // The break of the axis is a small mark on the axis, because a line through the plot reads as a line of the flight.
            float breakX = X(NearKm);
            dl.AddLine(new float2(breakX - 4f * s, bottom + 3f * s), new float2(breakX, bottom - 3f * s), Palette.Dim.Solid, 1f * s);
            dl.AddLine(new float2(breakX, bottom + 3f * s), new float2(breakX + 4f * s, bottom - 3f * s), Palette.Dim.Solid, 1f * s);
            // A thin solid stroke, because the label names this line and the dotted lines of the plot are parts of the flight.
            dl.AddLine(new float2(plot.X, brakingY), new float2(plot.Right, brakingY), Palette.Dim.A(0.7f), 1f * s);
            dl.AddLine(new float2(siteX, gateY), new float2(siteX, bottom), Palette.Plan.Solid, 2f * s);
            labels.Curve(new[] { new float2(siteX, gateY), new float2(siteX, bottom) });
            Glyphs.Site(dl, new float2(siteX, bottom), 5f * s, Palette.Target.Solid, 1.5f * s);
            labels.Mark(new float2(siteX, bottom), 8f * s);

            if (m > 1)
            {
                float2[] pts = _picturePoints.Get(m);
                for (int i = 0; i < m; i++)
                    pts[i] = new float2(X(_pathKm[i]), Y(_pathAlt[i]));
                Glyphs.DashedPolyline(dl, pts.AsSpan(0, m), Palette.Plan.Solid, 1.5f * s, 5f * s, 3f * s);
                labels.Curve(pts.AsSpan(0, m));
                float dx = pts[0].X - craftAt.X, dy = pts[0].Y - craftAt.Y;
                coast = dx * dx + dy * dy > 16f * s * s;
                if (coast)
                {
                    Glyphs.DashedLine(dl, craftAt, pts[0], Palette.Flown.A(0.8f), 1f * s, 1f * s, 2f * s);
                    labels.Curve(new[] { craftAt, pts[0] });
                }
                toSite = reachedBraking;
                if (toSite)
                {
                    var estimateEnd = new float2(siteX, gateY);
                    Glyphs.DashedLine(dl, pts[m - 1], estimateEnd, Palette.Plan.A(0.75f), 1f * s, 1f * s, 3f * s);
                    labels.Curve(new[] { pts[m - 1], estimateEnd });
                }
            }
            Glyphs.Dot(dl, craftAt, 4f * s, Palette.Flown.Solid);
            labels.Mark(craftAt, 5f * s);
        }

        labels.Guide(PageText.BrakingStart(brakingKm), brakingY, Palette.Text.Solid, under: plot.H < ShortApproachPlot * s);
        labels.Point(PageText.VerticalGate(_s.LandingVerticalGateM), new float2(siteX, gateY), 6f * s, Palette.Plan.Solid);
        labels.Point(PageText.CraftKey, craftAt, 6f * s, Palette.Flown.Solid);

        int legend = 0;
        _approachSwatches[legend++] = new Picture.Swatch("PROFILE", Palette.Plan, SwatchKind.Dashed);
        if (coast)
            _approachSwatches[legend++] = new Picture.Swatch(PageText.CoastKey, Palette.Flown, SwatchKind.Dotted);
        if (toSite)
            _approachSwatches[legend++] = new Picture.Swatch(PageText.BrakingKey, Palette.Plan, SwatchKind.Dotted);
        Picture.Legend(in f, lanes.Right, _approachSwatches.AsSpan(0, legend));
        Picture.BigNumber(in f, big, Palette.Text.Solid);
        if ((StagingStages == null || StagingStages.Stages.Count == 0) && ConsoleUi.Hovering(f.Min, f.Min + new float2(f.Width, f.Height)))
            ConsoleUi.Tooltip(PageText.StagesAtBraking.AsSpan());
    }

    private const float ShortApproachPlot = 120f;

    private static readonly string[] _hoverLegendNames = { "AIM", "DRIFT" };
    private static readonly Picture.Swatch[] _hoverSwatches = { new("AIM", Palette.Plan, SwatchKind.Filled), new("DRIFT", Palette.Flown, SwatchKind.Filled) };

    // The hover from above while it flies: the velocity setpoints as an arrow from the centre, the flown horizontal velocity as a dot on the same scale, and on the right a bar for the vertical velocity with its setpoint and a mark for what flies. A velocity past the scale lies on the ring or on the end of the bar, and the picture says that it is off scale, because a mark that stays in the centre reads as no drift.
    private static void DrawHoverView(in PictureFrame f, Orbit orbit, IParentBody parent)
    {
        float s = f.Scale;
        ImDrawListPtr dl = f.Dl;
        double3 r = orbit.StateVectors.PositionCci;
        double3 up = double3.Normalize(r);
        double3 vSrf = orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), r);
        (double3 east, double3 north) = EnuBasis(up);

        PictureLanes lanes = Picture.Lanes(in f, Picture.TextLaneHeight(), 0f, Picture.LegendWidth(_hoverLegendNames), Picture.TextLaneHeight());
        PicBox plot = lanes.Plot;
        float barZone = 62f * s;
        float dialW = Math.Max(40f * s, plot.W - barZone);
        var c = new float2(plot.X + dialW * 0.5f, plot.Y + plot.H * 0.5f);
        float reach = Math.Max(10f * s, Math.Min(dialW * 0.5f - 18f * s, plot.H * 0.5f - 16f * s));
        double scaleMs = Math.Max(5.0, Math.Max(Math.Abs(_s.TermSetE), Math.Abs(_s.TermSetN)) * 1.5);

        dl.AddCircle(c, reach, Palette.Grid.Solid, 0, 1f * s);
        dl.AddLine(c - new float2(reach, 0f), c + new float2(reach, 0f), Palette.Grid.Solid, 1f * s);
        dl.AddLine(c - new float2(0f, reach), c + new float2(0f, reach), Palette.Grid.Solid, 1f * s);

        (double setX, double setY, _) = SketchMath.DialPoint(_s.TermSetE, _s.TermSetN, scaleMs);
        var set = c + new float2((float)setX * reach, -(float)setY * reach);
        dl.AddLine(c, set, Palette.Plan.Solid, 2f * s);
        Glyphs.Diamond(dl, set, 5f * s, Palette.Plan.Solid);

        (double driftX, double driftY, bool driftOff) = SketchMath.DialPoint(double3.Dot(vSrf, east), double3.Dot(vSrf, north), scaleMs);
        var drift = c + new float2((float)driftX * reach, -(float)driftY * reach);
        Glyphs.Dot(dl, drift, 4f * s, Palette.Flown.Solid);

        float barX = plot.Right - 30f * s;
        float barTop = plot.Y + 16f * s, barBottom = plot.Bottom - 16f * s, mid = (barTop + barBottom) * 0.5f;
        double upScale = Math.Max(2.0, Math.Abs(_s.TermSetUp) * 1.5);
        dl.AddLine(new float2(barX, barTop), new float2(barX, barBottom), Palette.Grid.Solid, 1f * s);
        dl.AddLine(new float2(barX - 3f * s, mid), new float2(barX + 3f * s, mid), Palette.Grid.Solid, 1f * s);
        float half = (barBottom - barTop) * 0.5f;
        float biasY = mid - (float)SketchMath.BarPoint(_s.TermSetUp, upScale).Share * half;
        dl.AddLine(new float2(barX, mid), new float2(barX, biasY), Palette.Plan.Solid, 3f * s);
        (double climbShare, bool climbOff) = SketchMath.BarPoint(double3.Dot(vSrf, up), upScale);
        float climbY = mid - (float)climbShare * half;
        dl.AddTriangleFilled(new float2(barX + 4f * s, climbY), new float2(barX + 11f * s, climbY - 4f * s), new float2(barX + 11f * s, climbY + 4f * s), Palette.Flown.Solid);

        string upTop = "UP +" + AxisTicks.Number(upScale) + " m/s", upLow = "-" + AxisTicks.Number(upScale) + " m/s";
        ConsoleUi.PushSmall();
        float font = ImGui.GetFontSize();
        dl.AddText(new float2(Math.Min(barX - ImGui.CalcTextSize(upTop).X * 0.5f, plot.Right - ImGui.CalcTextSize(upTop).X), barTop - font - 1f * s), Palette.Dim.Solid, upTop);
        dl.AddText(new float2(Math.Min(barX - ImGui.CalcTextSize(upLow).X * 0.5f, plot.Right - ImGui.CalcTextSize(upLow).X), barBottom + 1f * s), Palette.Dim.Solid, upLow);
        ConsoleStyle.PopFont();

        Labels labels = _pictureLabels;
        labels.Begin(dl, in plot, s);
        labels.Reserve(new PicBox(plot.X + dialW, plot.Y, plot.W - dialW, plot.H));
        labels.Mark(drift, 5f * s);
        labels.Mark(set, 6f * s);
        if (driftOff)
            labels.Point(PageText.OffScale, drift, 7f * s, Palette.Warn.Solid);
        if (climbOff)
        {
            float2 offSize = labels.Size(PageText.OffScale);
            ConsoleUi.PushSmall();
            // One full row inside the end of the bar, because the label of the scale stands at that end.
            dl.AddText(new float2(barX - offSize.X - 2f * s, climbShare > 0.0 ? climbY + font * 0.5f : climbY - font * 1.5f), Palette.Warn.Solid, PageText.OffScale);
            ConsoleStyle.PopFont();
        }
        labels.Spots("N", new[] { new float2(c.X - labels.Size("N").X * 0.5f, c.Y - reach - labels.Size("N").Y - 1f * s) }, Palette.Dim.Solid);
        labels.Spots("E", new[] { new float2(c.X + reach + 3f * s, c.Y - labels.Size("E").Y * 0.5f) }, Palette.Dim.Solid);

        Picture.HeadText(in f, lanes.Head, "THROTTLE " + Fmt.Percent(_s.GfoldThrottle), Palette.Text.Solid);
        Picture.LaneText(in f, lanes.Bottom, "AIM " + PageText.HoverSetpoint(_s.TermSetE, _s.TermSetN, _s.TermSetUp), Palette.Dim.Solid);
        Picture.Legend(in f, lanes.Right, _hoverSwatches);
    }

    #endregion
}

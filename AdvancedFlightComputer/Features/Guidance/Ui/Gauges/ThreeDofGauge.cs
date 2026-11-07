#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

public static partial class GuidanceWindow
{
    private static void Draw3DofLandingContent(Vehicle vehicle, float innerW, bool liftNote = true)
    {
        if (TierScope.IsPrimaryPass)
        {
            if (liftNote)
                DrawLiftNote(vehicle, LandWay.FromHere);
            if (!EntryLive(_s) && _s.ThreeDofStatus.Length > 0 && _s.ThreeDofSeverity >= Severity.Warning)
                ConsoleUi.Warning(Fmt.Sentence(_s.ThreeDofStatus).AsSpan());
            DrawEntryLiveRows();
        }

        if (GuidanceRegion.Begin("##entry", GuidanceRegion.Diagnostics))
        {
            Draw3DofPlanRows();
            GuidanceRegion.End();
        }

        if (GuidanceRegion.Begin("ENTRY GUIDANCE PARAMETERS", GuidanceRegion.Tuning))
        {
            Draw3DofParameters();
            GuidanceRegion.End();
        }
    }

    private static void DrawEntryLiveRows()
    {
        var plan = _s.ThreeDofPlan;
        if (plan == null)
            return;
        if (_s.ThreeDofPhase == ThreeDofPhase.Burn)
            GaugeRowText("THROTTLE", FormattableString.Invariant($"{_s.ThreeDofThrottle * 100.0:F0} %  (PLAN {_s.ThreeDofPlanThrottle * 100.0:F0} %)"), ConsoleStyle.TextMuted);
        else if (_s.ThreeDofPhase == ThreeDofPhase.Glide)
            GaugeRowText("BURN TIME", Fmt.TimeFine(plan.SigmaBurn));
    }

    private static void Draw3DofPlanRows()
    {
        const UiTier D = UiTier.Diagnostics;
        double now = SimNow();
        GaugeRowText(D, "STATUS", _s.ThreeDofStatus.Length > 0 ? Fmt.Sentence(_s.ThreeDofStatus) : "-", ConsoleStyle.TextMuted);
        var est = _s.ThreeDofEstimator;
        GaugeRowText(D, "MODEL CORRECTION", est != null
            ? FormattableString.Invariant($"DRAG x{est.DragScale:F2}  LIFT x{est.LiftScale:F2}  THRUST x{est.ThrustScale:F2}") : "-", ConsoleStyle.TextMuted);
        // As of the solver's last idle moment: the guidance is the worker's while it solves, so the step caches this rather than the draw reading it.
        GaugeRowText(D, "LAST SOLVE", _s.ThreeDofSolveText.Length > 0 ? _s.ThreeDofSolveText : "-",
            _s.ThreeDofRefusing ? ConsoleUi.WarnInk : ConsoleStyle.TextMuted);
        GaugeRowText(D, "TURN ACCEL IN USE", _s.ThreeDofTurnAccelInUse > 0.0 ? FormattableString.Invariant($"{_s.ThreeDofTurnAccelInUse:F1} deg/s^2") : "NO LIMIT",
            ConsoleStyle.TextMuted);
        GaugeRowText(D, "TOUCHDOWN STANDBY", SixDofStandbyText(now), SixDofStandbyReady(now) ? ConsoleStyle.TextPrimary : ConsoleStyle.TextMuted);
    }

    private static void Draw3DofParameters()
    {
        GaugeRow(Specs.ThreeDofSmoothTau, ref _s.ThreeDofSmoothTau);
        // 0 is the craft's own, read from its RCS when the cold solve starts. The value in use is in DIAG.
        GaugeRow(Specs.ThreeDofTurnAccelDegS2, ref _s.ThreeDofTurnAccelDegS2);
        GaugeRow(Specs.ThreeDofGlideCeiling, ref _s.ThreeDofGlideCeiling);
        GaugeRow(Specs.ThreeDofBurnCeiling, ref _s.ThreeDofBurnCeiling);
        // One value with touchdown guidance: both plans keep this much throttle above the engine minimum for their feedback.
        GaugeRow(Specs.LandingPlanFloorMargin, ref _s.LandingPlanFloorMargin);
        GaugeRow(Specs.ThreeDofTrackKp, ref _s.ThreeDofTrackKp);
        GaugeRow(Specs.ThreeDofTrackKd, ref _s.ThreeDofTrackKd);
        GaugeRow(Specs.ThreeDofTrackTiltDeg, ref _s.ThreeDofTrackTiltDeg);
    }

    /// <summary>The DIAG view of entry guidance, registered with its mode (EntryModeHooks).</summary>
    private static void DrawEntryDiagnostics(Vehicle vehicle, float innerW)
    {
        using (TierScope.Push(UiTier.Diagnostics))
            Draw3DofLandingContent(vehicle, innerW);
    }

    private static readonly ImColor8 ThreeDofIgnitionCol = Palette.Text.Solid;

    private static readonly string[] _entryLegendNames = { "GLIDE", "BURN", "FLOWN", "IGNITION" };
    private static readonly Picture.Swatch[] _entrySwatches =
    {
        new("GLIDE", Palette.Plan),
        new("BURN", Palette.Plan, SwatchKind.Thick),
        new("FLOWN", Palette.Flown),
        new("IGNITION", Palette.Text, SwatchKind.Cross),
    };

    private const int EntryPlanStateSize = 7;

    private static bool EntryPlanDrawn(AdvancedFlightComputer.Guidance.Scvx.ThreeDof.Plan3Dof plan)
        => plan != null && plan.Nodes >= 2 && plan.X.Length >= plan.Nodes * EntryPlanStateSize;

    private static void Draw3DofPlot(in PictureFrame f)
    {
        var plan = _s.ThreeDofPlan;
        if (!EntryPlanDrawn(plan))
            return;
        ImDrawListPtr dl = f.Dl;
        float s = f.Scale;
        const int nx = EntryPlanStateSize;

        double maxR = 1.0, maxA = 1.0;
        for (int k = 0; k < plan.Nodes; k++)
        {
            maxR = Math.Max(maxR, Math.Sqrt(plan.X[k * nx] * plan.X[k * nx] + plan.X[k * nx + 1] * plan.X[k * nx + 1]));
            maxA = Math.Max(maxA, plan.X[k * nx + 2]);
        }
        if (TryLastTraceSample(out float2 craft))
        {
            maxR = Math.Max(maxR, craft.X);
            maxA = Math.Max(maxA, craft.Y);
        }
        double axR = NiceCeiling(maxR * 1.1), axA = NiceCeiling(maxA * 1.1);

        PictureLanes lanes = Picture.Lanes(in f, Picture.TextLaneHeight(), Picture.TickWidth(AxisTicks.Distance(axA)),
            Picture.LegendWidth(_entryLegendNames), Picture.TextLaneHeight());
        PicBox plot = lanes.Plot;
        float2 plotMin = new float2(plot.X + 4f * s, plot.Y + 4f * s);
        float2 plotMax = new float2(plot.Right - 8f * s, plot.Bottom - 5f * s);
        float2 plotSize = plotMax - plotMin;
        if (plotSize.X < 24f || plotSize.Y < 16f)
            return;
        float2 ToPlot(double rangeM, double altM) => new float2(
            plotMax.X - plotSize.X * (float)(rangeM / axR),
            plotMax.Y - plotSize.Y * (float)(altM / axA));
        float2 Node(int k) => ToPlot(Math.Sqrt(plan.X[k * nx] * plan.X[k * nx] + plan.X[k * nx + 1] * plan.X[k * nx + 1]),
                                     plan.X[k * nx + 2]);

        Picture.AxisX(in f, in lanes, 0.0, axR, time: false, r => ToPlot(r, 0.0).X, r => r == 0.0 ? PageText.SiteTick : AxisTicks.Distance(r));
        Picture.AxisY(in f, in lanes, 0.0, axA, a => ToPlot(0.0, a).Y, AxisTicks.Distance);

        float groundY = ToPlot(0.0, 0.0).Y;
        dl.AddLine(new float2(plotMin.X, groundY), new float2(plotMax.X, groundY), SchemSpent, 1.5f);
        Glyphs.Site(dl, ToPlot(0.0, 0.0), 5f * s, Palette.Target.Solid, 1.5f * s);

        // Glide intervals first, then the burn; the node between them is the ignition.
        int ignition = Math.Clamp(plan.GlideIntervals, 0, plan.Nodes - 1);
        for (int k = 0; k < plan.Nodes - 1; k++)
            ClipLine(dl, Node(k), Node(k + 1), plotMin, plotMax, GfoldPlanCol, k < ignition ? 1.5f : 3f);
        DrawGfoldFlownPath(dl, ToPlot, plotMin, plotMax);

        if (_s.ThreeDofPhase == ThreeDofPhase.Glide && plan.IgnitionTime > SimNow())
        {
            float2 x = Node(ignition);
            float r = 5f;
            dl.AddLine(x + new float2(-r, -r), x + new float2(r, r), ThreeDofIgnitionCol, 2.5f);
            dl.AddLine(x + new float2(-r, r), x + new float2(r, -r), ThreeDofIgnitionCol, 2.5f);
        }

        Picture.HeadText(in f, lanes.Head, PageText.HeightAboveSite, Palette.Dim.Solid);
        Picture.Legend(in f, lanes.Right, _entrySwatches);
    }

    /// <summary>The next 1, 2 or 5 times a power of ten at or above <paramref name="v"/>.</summary>
    private static double NiceCeiling(double v)
    {
        double p = Math.Pow(10.0, Math.Floor(Math.Log10(Math.Max(v, 1e-9))));
        foreach (double m in new[] { 1.0, 2.0, 5.0, 10.0 })
            if (m * p >= v)
                return m * p;
        return 10.0 * p;
    }
}

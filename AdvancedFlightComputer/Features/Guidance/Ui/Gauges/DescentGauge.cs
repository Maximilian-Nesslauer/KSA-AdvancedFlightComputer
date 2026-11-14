#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Canvas;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Ui.Logic;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

public static partial class GuidanceWindow
{
    /// <summary>
    /// Phase as the operator thinks of it. The landing machine's own names are about which solver is driving, which is not the question being asked here.
    /// </summary>
    private static string DescentPhaseLabel()
    {
        switch (_s.LandingPhase)
        {
            case LandingPhase.DeorbitPlanning: return "PLANNING";
            case LandingPhase.DeorbitCoast: return "DEORBIT COAST";
            case LandingPhase.DeorbitNodePending: return "NODE PENDING";
            case LandingPhase.DeorbitBurn: return "DEORBIT BURN";
            case LandingPhase.TransferPlanning: return "BRAKING PLAN";
            case LandingPhase.TransferCoast: return "COAST";
            case LandingPhase.Coast: return "COAST TO BRAKING";
            case LandingPhase.Prep: return "BRAKE READY";
            case LandingPhase.Burn: return "BRAKING BURN";
            case LandingPhase.GfoldDescent: return "G-FOLD";
            case LandingPhase.TerminalHover: return "HOVER";
            case LandingPhase.TerminalCoast: return "COAST TO LANDING BURN";
            case LandingPhase.TerminalBrake: return "LANDING BURN";
            case LandingPhase.Done: return "DONE";
            default: return "IDLE";
        }
    }

    private static bool DescentLive =>
        _s.LandingPhase != LandingPhase.Idle && _s.LandingPhase != LandingPhase.Done;

    /// <summary>
    /// The countdown the console head shows for a descent. Which countdown is meaningful depends on the phase: before ignition it is time TO the burn, during it the burn's own tgo, and under G-FOLD the predicted arrival. Showing one of them in all three would be wrong twice.
    /// </summary>
    private static double DescentStatusTgo()
    {
        switch (_s.LandingPhase)
        {
            case LandingPhase.DeorbitPlanning:
            case LandingPhase.DeorbitNodePending:
            case LandingPhase.TransferPlanning:
                return double.NaN;
            case LandingPhase.DeorbitCoast:
                return Math.Max(0, (_s.DeorbitPlan?.DepartureTime ?? SimNow()) - SimNow());
            case LandingPhase.DeorbitBurn:
                return _s.DeorbitTarget == null ? double.NaN
                    : Math.Max(0, _s.DeorbitTarget.IgnitionTime.Seconds() - SimNow()) + _s.DeorbitTarget.BurnDuration;
            case LandingPhase.TransferCoast:
            case LandingPhase.Coast:
            case LandingPhase.Prep:
                return Math.Max(0.0, _s.BurnStartTime - SimNow());
            case LandingPhase.GfoldDescent:
                return Math.Max(0.0, _s.GfoldArrivalTime - SimNow());
            case LandingPhase.TerminalCoast:
                return double.NaN;
            case LandingPhase.TerminalBrake:
                return _s.TerminalBurnSeconds;
            default:
                return _s.Upfg.Tgo;
        }
    }

    /// <summary>The countdown the console header shows, NaN where the phase has none.</summary>
    private static double DescentTgoSeconds()
        => _s.LandingPhase is LandingPhase.Idle or LandingPhase.Done or LandingPhase.TerminalHover
            ? double.NaN
            : DescentStatusTgo();

    /// <summary>
    /// The DEORBIT way of the LAND page form: the approach shaping. The phase is in the console head, the approach picture and the pass strip are above the form, and the stock deorbit warp offer is the head warp line. The landing site is drawn by the page itself, because every way of landing shares it.
    /// </summary>
    private static void DrawDescentContent(float innerW)
    {
        DrawApproachSection(innerW);
    }

    private static bool PassesCount => Program.ControlledVehicle is Vehicle craft && Ui.Console.ConsoleFrame.ReadFacts(craft).Where == CraftSituation.Orbit && !NoSitePicked(craft.Orbit?.Parent);

    // How far the closest pass stays from the site, or NaN while it is close enough or no pass has a solution. The planner still looks for a way, so this is an outline on the verb and no refusal.
    private static double ClosestPassFarKm()
    {
        if (!PassesCount)
            return double.NaN;
        int closest = ClosestPassIndex();
        return closest >= 0 && _s.Passes[closest].minKm >= PassFarKm && _s.Passes[closest].minKm < PassNoSolutionKm
            ? _s.Passes[closest].minKm
            : double.NaN;
    }

    internal static string PressTimingText()
    {
        if (!PassesCount)
            return "";
        int closest = ClosestPassIndex();
        return closest >= 0 && closest != NextPassIndex() && _s.Passes[closest].minKm < PassNoSolutionKm ? PageText.PressPlansNext : "";
    }

    private static void DrawDeorbitFigures()
    {
        ConsoleUi.BeginSection("LandDeorbitFigures", FoldNames.Deorbit);
        if (DescentLive)
            GaugeRowText("BURN DOWNRANGE",
                FormattableString.Invariant($"{_s.BurnDownrangeKm:F1} km  ({_s.DownrangeFactor:F2}x)"));
        foreach (var row in DeorbitReadout())
            GaugeRowText(row.label.ToUpperInvariant(), row.value);
        ConsoleUi.EndSection();
    }

    // Shaping the approach is expert work and tuning, not aiming, so the region has no Player rows.
    private static void DrawApproachSection(float innerW)
    {
        if (!GuidanceRegion.Begin(FoldNames.Approach, GuidanceRegion.ExpertTuning))
            return;

        using (new ImGuiDisabledScope(DeorbitTargetLocked))
        {
            GaugeRow(Specs.BrakingAltitudeKm, ref _s.BrakingAltitudeKm);
            GaugeRowCheck(Specs.DeorbitArrivalAngleEnabled, ref _s.DeorbitArrivalAngleEnabled);
            using (new ImGuiDisabledScope(!_s.DeorbitArrivalAngleEnabled))
                GaugeRow(Specs.DeorbitArrivalDescentDeg, ref _s.DeorbitArrivalDescentDeg);
            if (TierScope.Show(UiTier.Expert))
                ConsoleUi.NoteShape("AfcApproachTolerance", _s.DeorbitArrivalAngleEnabled ? 1 : 0);
            if (_s.DeorbitArrivalAngleEnabled)
                GaugeRowText(UiTier.Expert, "ANGLE TOLERANCE", FormattableString.Invariant($"+/-{DeorbitPlanner.ArrivalAngleToleranceDeg:F0} deg AT BRAKING POINT"), ConsoleStyle.TextMuted);
            GaugeRow(Specs.DownrangeFactor, ref _s.DownrangeFactor);
            GaugeRow(Specs.AimAltKm, ref _s.AimAltKm);
            GaugeRow(Specs.DescentRate, ref _s.DescentRate);
            GaugeRow(Specs.GateUprangeKm, ref _s.GateUprangeKm);
            // Where the braking burn ends and the powered descent, 6-DOF or G-FOLD, takes over. It shapes this phase, so it belongs here rather than with either solver's tuning.
            GaugeRow(Specs.GfoldHandoffTgo, ref _s.GfoldHandoffTgo);
            GaugeRow(Specs.LandingVerticalGateM, ref _s.LandingVerticalGateM);
        }
        GaugeRowCheck(Specs.LandingGimbalSteers, ref _s.LandingGimbalSteers);

        GuidanceRegion.End();
    }

    /// <summary>
    /// How close each upcoming orbit brings the ground track to the site, as one horizontal strip in its own frame: the site is the violet centre line, in a violet band out to the limit of a good pass, with a dim orange zone where a pass is far where the scale reaches that far, and every pass is placed left or right of it by its SIGNED closest approach - which side of the track the site fell on. Reading across the strip you see the track walking past the site orbit by orbit, and whether it is converging on it or drifting away.
    ///  The closest pass is the one worth waiting for, so it is a block drawn last in the colour of its key: green only inside the limit, the plan colour past it, and the warning colour when it is far. It is not red, because nothing has failed and the planner still looks for a way. While a deorbit flies it can no longer be chosen, so it is a tick like the others. With <paramref name="big"/> the big number is the distance of the next pass, because that is what a press flies, and on an idle page it has no green, because nothing is confirmed yet. The row under the strip names the closest. The rest are thin ticks - they are context, not choices. The next pass has a bright tick with a mark at each end, and while a deorbit flies only the pass that flies has one. The row under the strip ties the marks and the band to their meaning.
    /// </summary>
    private static void DrawPassStrip(bool big)
    {
        // The guidance step rebuilds the passes while the strip asked for them in the last second (see StepReadouts), so the draw only reads them.
        ViewDemand.Mark(Program.ControlledVehicle, ViewDemand.Kind.Passes);
        int closest = ClosestPassIndex();
        int next = NextPassIndex();
        if (closest >= 0 && _s.Passes[closest].minKm >= PassNoSolutionKm)
            closest = -1;
        int flown = -1;
        if (DescentLive && _s.DeorbitPlan != null)
        {
            var passTimes = new double[_s.Passes.Count];
            for (int i = 0; i < passTimes.Length; i++)
                passTimes[i] = _s.Passes[i].tSec;
            flown = SketchMath.NearestTime(passTimes, _s.DeorbitPlan.ArrivalTime - SimNow());
        }

        // Scale to the widest pass, with a floor so a single very close pass doesn't blow one kilometre up to half the strip and imply a precision we don't have.
        float scaleKm = PassStripMinScaleKm;
        double lastT = 0.0;
        for (int i = 0; i < _s.Passes.Count; i++)
        {
            scaleKm = MathF.Max(scaleKm, (float)Math.Abs(_s.Passes[i].crossKm));
            lastT = Math.Max(lastT, _s.Passes[i].tSec);
        }
        // Quantised, so the axis holds still. Scaling to the exact widest pass meant every small change in it re-scaled the strip and slid every other block - motion that looked like the passes moving when it was only the ruler.
        scaleKm = NiceScale(scaleKm);

        bool nextKnown = next >= 0 && _s.Passes[next].minKm < PassNoSolutionKm;
        bool flownKnown = flown >= 0 && _s.Passes[flown].minKm < PassNoSolutionKm;
        int block = flown < 0 ? closest : -1;
        string miss = big && nextKnown ? PageText.PassTrack(_s.Passes[next].minKm, _s.Passes[next].tSec) : "";
        PictureFrame probe = Picture.Probe();
        float s = probe.Scale;
        float rowH = Picture.TextLaneHeight();
        float headH = Picture.HeadHeight(in probe, miss, text: true);
        using (PictureScope pic = Picture.Open(Picture.FrameHeight(headH + 26f * s + rowH * 2f)))
        {
            PictureFrame f = pic.Frame;
            ImDrawListPtr dl = f.Dl;
            PictureLanes lanes = Picture.Lanes(in f, headH, 0f, 0f, rowH * 2f, Picture.StripPlot);
            var scaleLane = new PicBox(lanes.Bottom.X, lanes.Bottom.Y, lanes.Bottom.W, rowH);
            var rowLane = new PicBox(lanes.Bottom.X, lanes.Bottom.Y + rowH, lanes.Bottom.W, rowH);
            PicBox plot = lanes.Plot;
            var stripMin = new float2(plot.X + 4f * s, plot.Y + 2f * s);
            var stripMax = new float2(plot.Right - 4f * s, plot.Bottom - 2f * s);
            float midX = (stripMin.X + stripMax.X) * 0.5f;
            float blockW = PassStripBlockW * s;

            dl.AddRectFilled(stripMin, stripMax, SchemTrack, 3f);
            float half = (stripMax.X - stripMin.X) * 0.5f - blockW;
            float PassX(double km) => midX + (float)(km / scaleKm) * half;

            float band = Math.Min((float)(PassGreenKm / scaleKm) * half, half);
            dl.AddRectFilled(new float2(midX - band, stripMin.Y), new float2(midX + band, stripMax.Y), Palette.Target.A(PassBandAlpha), 0f);
            if (scaleKm > PassFarKm)
            {
                dl.AddRectFilled(stripMin, new float2(PassX(-PassFarKm), stripMax.Y), Palette.Warn.A(0.14f), 0f);
                dl.AddRectFilled(new float2(PassX(PassFarKm), stripMin.Y), stripMax, Palette.Warn.A(0.14f), 0f);
            }
            dl.AddRect(stripMin, stripMax, SchemSpent, 3f);
            dl.AddLine(new float2(midX, stripMin.Y + 1f), new float2(midX, stripMax.Y - 1f), Palette.Target.Solid, 1.5f * s);

            for (int i = 0; i < _s.Passes.Count; i++)
            {
                var pass = _s.Passes[i];
                if (i == block || pass.minKm >= PassNoSolutionKm)
                    continue;   // the block comes last, and a revolution with no solution has no place
                float timeT = lastT > 1.0 ? (float)(pass.tSec / lastT) : 0f;
                float x = PassX(pass.crossKm);
                dl.AddLine(new float2(x, stripMin.Y + 3f), new float2(x, stripMax.Y - 3f), PassTickColour(timeT), 1.5f * s);
            }
            if (block >= 0)
            {
                float x = PassX(_s.Passes[block].crossKm);
                dl.AddRectFilled(new float2(x - blockW * 0.5f, stripMin.Y + 3f), new float2(x + blockW * 0.5f, stripMax.Y - 3f), PassToneRgb(_s.Passes[block].minKm).Solid, 2f);
            }
            if (flown < 0 && nextKnown && next != closest)
                Glyphs.Beam(dl, PassX(_s.Passes[next].crossKm), stripMin.Y, stripMax.Y, 4f * s, 2f * s, SchemBody);
            if (flownKnown)
                Glyphs.Beam(dl, PassX(_s.Passes[flown].crossKm), stripMin.Y, stripMax.Y, 4f * s, 2f * s, Palette.Afc.Solid);

            Picture.HeadText(in f, lanes.Head, miss.Length > 0 ? PageText.NextPassTitle : PageText.PassesTitle, Palette.Dim.Solid, miss);
            PlanTone missTone = nextKnown ? ColorRules.MissTone(_s.Passes[next].minKm, flying: false, _s.Passes[next].minKm >= PassFarKm, PassGreenKm) : PlanTone.Plain;
            Picture.BigNumber(in f, miss, Palette.Tone(missTone).Solid);

            string scaleText = AxisTicks.Number(scaleKm) + " km";
            Picture.LaneText(in f, in scaleLane, "-" + scaleText, Palette.Dim.Solid);
            Picture.LaneText(in f, in scaleLane, "SITE", Palette.Dim.Solid, 0);
            Picture.LaneText(in f, in scaleLane, "+" + scaleText, Palette.Dim.Solid, 1);

            // The closest pass as the block of its tone with its distance, and the next pass with its mark, because the big number holds its distance. The band and the ticks stand last, and a row with no room leaves them out.
            if (closest < 0 && !flownKnown)
                Picture.LaneText(in f, in rowLane, PageText.NoPassSolution, Palette.Dim.Solid);
            else
            {
                var bandKey = new Picture.Swatch(PageText.PassWithin(PassGreenKm), Palette.Target, SwatchKind.Filled, 0.5f);
                var ticksKey = new Picture.Swatch(PageText.OtherPassesKey, Palette.Dim, SwatchKind.Tick);
                ReadOnlySpan<Picture.Swatch> marks = flownKnown
                    ? new[]
                    {
                        new Picture.Swatch(PageText.PassFlies(_s.Passes[flown].minKm, _s.Passes[flown].tSec), Palette.Afc, SwatchKind.Beam),
                        bandKey,
                        ticksKey,
                    }
                    : flown >= 0
                    ? new[] { bandKey, ticksKey }
                    : closest == next
                    ? new[]
                    {
                        new Picture.Swatch(PageText.PassNext(alsoClosest: true), PassToneRgb(_s.Passes[closest].minKm), SwatchKind.Filled),
                        bandKey,
                        ticksKey,
                    }
                    : new[]
                    {
                        new Picture.Swatch(PageText.PassClosest(_s.Passes[closest].minKm, _s.Passes[closest].tSec), PassToneRgb(_s.Passes[closest].minKm), SwatchKind.Filled),
                        nextKnown ? new Picture.Swatch(PageText.PassNext(alsoClosest: false), Palette.Text, SwatchKind.Beam) : default,
                        bandKey,
                        ticksKey,
                    };
                Picture.LegendRow(in f, in rowLane, marks);
            }
        }
    }

    /// <summary>Round up to a 1-2-5 step, so an auto-scaled axis stops jittering.</summary>
    private static float NiceScale(float v)
    {
        float mag = MathF.Pow(10f, MathF.Floor(MathF.Log10(MathF.Max(v, 1e-3f))));
        float norm = v / mag;
        float step = norm <= 1f ? 1f : norm <= 2f ? 2f : norm <= 5f ? 5f : 10f;
        return step * mag;
    }

    private static ImColor8 PassTickColour(float timeT)
    {
        (byte r, byte g, byte b) = PassNearRgb;
        float k = Math.Clamp(timeT, 0f, 1f) * PassFadeDepth;
        return new ImColor8(
            (byte)(r + (PassFarRgb.R - r) * k),
            (byte)(g + (PassFarRgb.G - g) * k),
            (byte)(b + (PassFarRgb.B - b) * k));
    }

    private const float PassBandAlpha = 0.25f;

    private static Rgb PassToneRgb(double minKm) => SketchMath.PassToneOf(minKm, PassGreenKm, PassFarKm) switch
    {
        PassTone.Inside => Palette.Good,
        PassTone.Far => Palette.Warn,
        _ => Palette.Plan,
    };
}

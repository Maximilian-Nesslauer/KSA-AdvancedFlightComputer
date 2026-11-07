#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Canvas;
using AdvancedFlightComputer.Features.Ui.Console;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

public static partial class GuidanceWindow
{
    // The two ends of the pass strip's distance fade, in components. ImColor8 exposes no way to read a colour back out once built, so the blend needs the numbers - and the two swatches below are built from these so there is one source of truth.
    // Declared first: static initialisers run in textual order.
    private static readonly (byte R, byte G, byte B) PassNearRgb = (232, 238, 245);
    private static readonly (byte R, byte G, byte B) PassFarRgb = (30, 33, 38);

    // The colours of the guidance pictures and the staging bar.
    private static readonly ImColor8 SchemInk = new ImColor8(205, 215, 225);
    private static readonly ImColor8 SchemDim = new ImColor8(120, 128, 138);
    private static readonly ImColor8 SchemBody = new ImColor8(PassNearRgb.R, PassNearRgb.G, PassNearRgb.B);
    private static readonly ImColor8 SchemSpent = new ImColor8(70, 76, 84);
    private static readonly ImColor8 SchemTrack = new ImColor8(PassFarRgb.R, PassFarRgb.G, PassFarRgb.B);

    private const float PassStripBlockW = 7f;
    private const float PassStripMinScaleKm = 25f;
    private const double PassGreenKm = 10.0;
    private const double PassFarKm = 400.0;
    private const double PassNoSolutionKm = 1e6;
    // How far a distant pass fades toward the strip: 1 would erase it entirely.
    private const float PassFadeDepth = 0.75f;

    // Length of a thrust tick on the G-FOLD side view, in pixels.
    private const float GfoldThrustGlyphPx = 12f;

    /// <summary>
    /// Seconds until engine cutoff. Read off the time LATCHED when the terminal phase began, not off UPFG's tgo: the solver keeps iterating through the freeze, and its tgo over a near-zero arc is exactly the quantity that misbehaves there.
    /// </summary>
    private static double AscentCutoffIn() => _s.CutoffTime - SimNow();

    /// <summary>
    /// Phase as the operator thinks of it, rather than as the state machine names it:
    /// what is steering right now.
    /// </summary>
    private static string AscentPhaseLabel()
    {
        if (!_s.Running)
            return _s.LaunchArmed ? "ARMED" : _s.ConvexLaunchPending ? "PLANNING" : "IDLE";
        switch (_s.Phase)
        {
            case AscentPhase.Vertical: return "VERTICAL";
            // Not "gravity turn": that name belongs to the gravity turn ascent method.
            case AscentPhase.Turn: return "SIMPLE TURN";
            case AscentPhase.Profile: return "CALCULATED CLIMB";
            case AscentPhase.ClosedLoop: return double.IsFinite(_s.ConvexBlendStart) ? "UPFG (TAKING OVER)" : "UPFG";
            case AscentPhase.Terminal:
                return AscentCutoffIn() > 0.0 ? "FINAL SECONDS" : "CUTOFF";
            default: return _s.Phase.ToString();
        }
    }

    /// <summary>Seconds to engine cutoff of the running ascent, as the console head shows it, or NaN while no ascent runs.</summary>
    private static double AscentTgoSeconds()
    {
        if (!_s.Running)
            return double.NaN;
        return _s.Phase == AscentPhase.Terminal
            ? Math.Max(0.0, AscentCutoffIn())
            : Math.Max(0.0, _s.Upfg.Tgo - Math.Max(0.0, SimNow() - _s.LastSolveTime));
    }

    private sealed class StageMemo
    {
        public int[] Stack;
        public AscentPlan Plan;
        public bool Flew;
    }

    private static readonly ConditionalWeakTable<VehicleAutopilotState, StageMemo> _stageMemos = new();

    private static int[] StageNumbersOf(AdvancedFlightComputer.Features.Guidance.Upfg.UpfgVehicle stages)
    {
        int[] left = StockSequences(stages);
        StageMemo memo = _stageMemos.GetValue(_s, _ => new StageMemo());
        AscentPlan plan = _s.AscentPlan ?? _s.FlyingPlan;
        bool flies = _s.Running || _s.LandingPhase != LandingPhase.Idle;
        if (!StageNumbers.KeepsStack(memo.Stack != null, memo.Stack != null && StageNumbers.Fits(memo.Stack, left), flies, memo.Flew, plan != null, ReferenceEquals(memo.Plan, plan), memo.Plan != null))
        {
            memo.Stack = left;
            memo.Plan = plan;
            memo.Flew = false;
        }
        if (flies)
        {
            memo.Flew = true;
            memo.Plan = plan ?? memo.Plan;
        }
        return StageNumbers.Of(memo.Stack, left);
    }

    // The name of a stage of the stack in the marks of the plan and in the bar: the stock sequence it had when the plan was made.
    private static string StageTag(int index)
        => StageNumbers.Tag(_stageMemos.TryGetValue(_s, out StageMemo memo) && memo.Stack != null ? memo.Stack : ReadOnlySpan<int>.Empty, index);

    private static bool StageStackFlew => _stageMemos.TryGetValue(_s, out StageMemo memo) && memo.Flew;

    private static string StageKey(int stage)
    {
        var stages = StagingStages;
        if (stages == null || stages.Stages.Count == 0 || !_stageMemos.TryGetValue(_s, out StageMemo memo) || memo.Stack == null)
            return StageTag(stage);
        int[] left = StockSequences(stages);
        return StageNumbers.KeyOf(memo.Stack, left, StageNumbers.Of(memo.Stack, left), stage);
    }

    // The stage that burns now, in the name the stock panel and STAGING read, with its hue. False with no stages.
    private static bool LiveStageKey(out string name, out Rgb hue)
    {
        name = "";
        hue = default;
        var stages = StagingStages;
        if (stages == null || stages.Stages.Count == 0)
            return false;
        name = StageNumbers.Tag(StockSequences(stages), 0);
        hue = ColorRules.Stage(StageNumbersOf(stages)[0]);
        return true;
    }

    private static int[] StockSequences(AdvancedFlightComputer.Features.Guidance.Upfg.UpfgVehicle stages)
    {
        var left = new int[stages.Stages.Count];
        for (int i = 0; i < left.Length; i++)
            left[i] = stages.Stages[i].Seq;
        return left;
    }

    /// <summary>
    /// The stage list of the staging bar of <paramref name="vehicle"/>: the stock sequence of each row, its number in the whole stack and its delta-v, for StageHues, so every page gives a stage the hue of its block on the bar. Returns false while the craft has no list.
    /// </summary>
    internal static bool ReadStageStack(Vehicle vehicle, out int[] left, out int[] numbers, out float[] dv)
    {
        left = numbers = Array.Empty<int>();
        dv = Array.Empty<float>();
        VehicleAutopilotState previous = _s;
        _s = VehicleAutopilotState.For(vehicle);
        try
        {
            var stages = StagingStages;
            if (stages == null || stages.Stages.Count == 0)
                return false;
            numbers = StageNumbersOf(stages);
            left = new int[stages.Stages.Count];
            dv = new float[stages.Stages.Count];
            for (int i = 0; i < left.Length; i++)
            {
                left[i] = stages.Stages[i].Seq;
                StageFigures(stages.Stages[i], out dv[i], out _);
            }
            return true;
        }
        finally
        {
            _s = previous;
        }
    }

    /// <summary>The staging bar under the stage line of the LAUNCH picture, and of the LAND > DEORBIT picture while the deorbit chain flies. With no stages the line says why, and no bar draws. With <paramref name="live"/> the craft has left the ground, so the bar is a live fact.</summary>
    private static void DrawStagingBarOnly(float innerW, bool live)
    {
        float2 origin = ImGui.GetCursorScreenPos();
        float barH = DrawStagingBar(ImGui.GetWindowDrawList(), origin, innerW, ShowProvenance, live);
        if (barH > 0f)
        {
            ImGui.Dummy(new float2(innerW, barH));
            ConsoleUi.NoteItem("AfcStageBar");
        }
    }

    private static AdvancedFlightComputer.Features.Guidance.Upfg.UpfgVehicle StagingStages
        => (_s.Running || _s.LandingPhase != LandingPhase.Idle) ? _s.UpfgVehicle : _s.StageModel;

    private static void StageFigures(AdvancedFlightComputer.Features.Guidance.Upfg.UpfgStage st, out float dv, out float burn)
    {
        double ve = st.Isp * 9.80665;
        double stageDv = ve * Math.Log(st.MassTotal / st.MassDry);
        double t = st.Mode == 2
            ? stageDv / (st.GLim * 9.80665)
            : (st.MassTotal - st.MassDry) / (st.Thrust / ve);
        burn = (float)Math.Max(0.0, t);
        dv = (float)Math.Max(0.0, stageDv);
    }

    private static string StagingLine()
    {
        var stages = StagingStages;
        if (stages == null || stages.Stages.Count == 0)
            return _s.LandingPhase != LandingPhase.Idle ? "AFC reads the stages at the braking burn" : "No stage data yet";
        int n = stages.Stages.Count;
        Span<float> dv = n <= 12 ? stackalloc float[12] : new float[n];
        float total = 0f, totalDv = 0f;
        for (int i = 0; i < n; i++)
        {
            StageFigures(stages.Stages[i], out dv[i], out float burn);
            total += burn;
            totalDv += dv[i];
        }
        if (total <= 1e-3f)
            return "No burn time";

        // KSA's own figure for the same stack, when it disagrees by more than a percent. KSA has no ambient correction, so the compare leaves it out too, and a gap is then a real disagreement about the staging that a plausible-looking stage list will not otherwise show.
        double ksaDv = _s.StageModelKsaDv;
        double modelDv = UncorrectedTotalDv(stages, dv, n, totalDv);
        bool sameStack = ksaDv > 1.0 && Math.Abs(ksaDv - modelDv) <= 0.01 * ksaDv;
        string cross = ShowProvenance && ksaDv > 1.0 && !sameStack
            ? string.Format(PicInv, "   (KSA {0:F0} m/s)", ksaDv) : "";
        // While both read the same stack the line prints the stock figure, so LAUNCH, STAGING and the stock panel show one number for one craft.
        return "ON BOARD   " + Fmt.DvWhole(sameStack ? ksaDv : totalDv) + "   " + Fmt.Time(total) + " burn" + cross;
    }

    // The seq/eng provenance, the G flag and the KSA cross-check of the staging bar are diagnostics, shown only with SHOW STAGING DIAGNOSTICS on.
    private static bool ShowProvenance => UiConfig.GetBool(UiConfig.KeyShowDiagnostics, false);

    // Total dV without the ambient correction. A g-limit can split the burning stage into leading rows with the same thrust and Isp and a continuous mass.
    private static double UncorrectedTotalDv(
        AdvancedFlightComputer.Features.Guidance.Upfg.UpfgVehicle stages, ReadOnlySpan<float> dv, int n, double totalDv)
    {
        double ratio = stages.BurningStageThrustRatio;
        if (n == 0 || !(ratio > 0.0) || Math.Abs(ratio - 1.0) < 1e-9)
            return totalDv;

        AdvancedFlightComputer.Features.Guidance.Upfg.UpfgStage first = stages.Stages[0];
        double corrected = dv[0];
        for (int i = 1; i < n; i++)
        {
            AdvancedFlightComputer.Features.Guidance.Upfg.UpfgStage s = stages.Stages[i];
            if (s.Thrust != first.Thrust || s.Isp != first.Isp || s.MassTotal != stages.Stages[i - 1].MassDry)
                break;
            corrected += dv[i];
        }
        return totalDv + corrected * (1.0 / ratio - 1.0);
    }

    /// <summary>
    /// The staging plan as a horizontal bar, each stage's width proportional to its burn time - this one IS to scale - with its tag in it where the tag fits, and a hover that shows its dV and burn time. With SHOW STAGING DIAGNOSTICS the same stages stand under the bar as rows with their provenance. The stage model describes the stack still to burn, so a stage that has gone leaves the bar, and the stages left keep the colours of the marks in the profile (StageNumbersOf). With <paramref name="live"/> the craft has left the ground, and the stock numbers have moved on from the names of the plan, so the bar is thin and holds no text, its hover and the line over it say the live name, and the marks of the plot keep the plan names.
    /// </summary>
    private static float DrawStagingBar(ImDrawListPtr dl, float2 min, float width, bool showProvenance, bool live)
    {
        float lineH = ImGui.GetTextLineHeight();
        var stages = StagingStages;
        if (stages == null || stages.Stages.Count == 0)
            return 0f;

        int n = stages.Stages.Count;
        Span<float> burn = n <= 12 ? stackalloc float[12] : new float[n];
        Span<float> dv = n <= 12 ? stackalloc float[12] : new float[n];
        float total = 0f;
        for (int i = 0; i < n; i++)
        {
            StageFigures(stages.Stages[i], out dv[i], out burn[i]);
            total += burn[i];
        }
        if (total <= 1e-3f)
            return 0f;

        int[] number = StageNumbersOf(stages);
        bool windowHovered = ImGui.IsWindowHovered(ImGuiHoveredFlags.ChildWindows);
        float scale = ImGuiHelper.InterfaceScale;
        ConsoleUi.PushSmall();
        float tagH = ImGui.GetFontSize();
        float barH = live ? StageBarThin * scale : tagH + 2f * scale;
        float x = min.X;
        int hoveredStage = -1;
        bool shortTags = false;
        for (int i = 0; i < n; i++)
            shortTags |= ImGui.CalcTextSize(StageTag(number[i])).X + 4f * scale > width * (burn[i] / total);
        for (int i = 0; i < n; i++)
        {
            float wSeg = width * (burn[i] / total);
            float2 a = new float2(x, min.Y);
            float2 b = new float2(MathF.Min(x + wSeg, min.X + width), min.Y + barH);
            dl.AddRectFilled(a, b, Palette.Stage(number[i]), 1f);
            string tag = StageTag(number[i]);
            if (shortTags)
                tag = StageNumbers.ShortTag(tag);
            float tagW = ImGui.CalcTextSize(tag).X;
            if (!live && tagW + 4f * scale <= wSeg)
                dl.AddText(new float2(x + (wSeg - tagW) * 0.5f, min.Y + (barH - tagH) * 0.5f), ConsoleStyle.ApplyAlpha(ConsoleStyle.PanelU32), tag);
            if (windowHovered && ImGui.IsMouseHoveringRect(a, b))
                hoveredStage = i;
            x += wSeg;
        }
        ConsoleStyle.PopFont();
        if (hoveredStage >= 0)
        {
            string name = live ? StageNumbers.Tag(StockSequences(stages), hoveredStage) : StageTag(number[hoveredStage]);
            string row = name + "   " + StageRowValues(dv[hoveredStage], burn[hoveredStage]);
            if (showProvenance)
                row += "   " + StageProvenance(stages.Stages[hoveredStage]);
            ConsoleUi.Tooltip(row.AsSpan());
        }

        float y = min.Y + barH + 4f * scale;
        if (!showProvenance)
            return y - min.Y;

        // The same stages as text, keyed by colour to the bar above. The provenance names the stock staging sequence the arc came from (SEQ, in the stock colour) and how many engine cores the game's own drain simulation had burning across it. Two rows with the same pair would be one physical stage sliced in two upstream; the adapter coalesces those, so seeing them here means it found a real difference in thrust, Isp or mass between them.
        float colDv = width * 0.16f;
        float colBurn = width * 0.42f;
        float colSeq = width * 0.64f;
        for (int i = 0; i < n; i++)
        {
            float ry = y + lineH * i;
            dl.AddText(new float2(min.X, ry), Palette.Stage(number[i]), StageTag(number[i]));
            dl.AddText(new float2(min.X + colDv, ry), SchemInk, Fmt.DvWhole(dv[i]));
            dl.AddText(new float2(min.X + colBurn, ry), SchemInk, Fmt.Time(burn[i]));
            dl.AddText(new float2(min.X + colSeq, ry), Palette.Stock.Solid, StageProvenance(stages.Stages[i]));
        }
        y += lineH * n;

        return y - min.Y;
    }

    private const float StageBarThin = 6f;

    private static string StageRowValues(float dv, float burn) => Fmt.DvWhole(dv) + "   " + Fmt.Time(burn);

    // Seq < 0 is the fallback model: no usable staging sequences, so the stack was measured off the live engines instead. G marks a g-limited stage.
    private static string StageProvenance(AdvancedFlightComputer.Features.Guidance.Upfg.UpfgStage st)
    {
        string prov = st.Seq < 0 ? "Live engines" : string.Format(PicInv, "Seq {0}  {1} eng", st.Seq, st.Engines);
        return st.Mode == 2 ? prov + "  G" : prov;
    }
}

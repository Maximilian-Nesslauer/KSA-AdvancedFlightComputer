#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Ui.Logic;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

//  DELIBERATELY NO TRAJECTORY PLOT. The hover flies a rate profile rather than a trajectory, so there is no plan to draw a flown path against and the picture says nothing the numbers above it do not. It still FEEDS the shared trace, because hover is the tail of the same descent, so the G-FOLD and 6-DOF content shows the whole thing including this phase.
public static partial class GuidanceWindow
{
    private static void DrawHoverContent(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                            double mu, float innerW)
    {
        bool active = _s.LandingPhase == LandingPhase.TerminalHover;

        if (TierScope.IsPrimaryPass && LandFlies && KsaEnginePerf.SupportsThrottleControl(vehicle))
        {
            // Hover is impossible below a TWR of one, because the engine cannot hold the weight, let alone descend on a profile, so this is a go/no-go, not a statistic.
            double twr = TerminalTwr(vehicle, orbit, mu);
            // The TWR itself is in the picture, so the form only warns.
            if (twr < 1.0)
                ConsoleUi.Warning(FormattableString.Invariant($"TWR {twr:F2}, below 1. No hover, the thrust cannot hold the weight.").AsSpan());
            double minTwr = TerminalMinThrottleTwr(vehicle, orbit, mu);
            if (minTwr > 1.0)
                ConsoleUi.Warning(FormattableString.Invariant($"TWR {minTwr:F2} at minimum throttle, above 1. No hover, the craft climbs whenever the engine runs.").AsSpan());
        }

        DrawHoverReadout(orbit, parent, active);
        if (TierScope.IsPrimaryPass)
        {
            ConsoleUi.NoteShape("AfcHoverPad", active ? 1 : 0);
            if (active)
                DrawHoverSetpoints(innerW);
        }

        if (GuidanceRegion.Begin("HOVER GAINS", GuidanceRegion.Tuning))
        {
            GaugeRow(Specs.TermQuadK, ref _s.TermQuadK);
            GaugeRow(Specs.TermKpV, ref _s.TermKpV);
            GaugeRow(Specs.TermKiV, ref _s.TermKiV);
            GaugeRow(Specs.TermKdV, ref _s.TermKdV);
            GaugeRow(Specs.TermKpL, ref _s.TermKpL);
            GaugeRow(Specs.TermKiL, ref _s.TermKiL);
            GaugeRow(Specs.TermKdL, ref _s.TermKdL);
            GuidanceRegion.End();
        }
    }

    private static void DrawHoverLimitsSection()
    {
        if (!GuidanceRegion.Begin(FoldNames.HoverLimits, GuidanceRegion.Expert))
            return;
        GaugeRow(Specs.TermTouchdownRate, ref _s.TermTouchdownRate);
        GaugeRow(Specs.TermConstAltM, ref _s.TermConstAltM);
        GaugeRow(Specs.TermMaxDescRate, ref _s.TermMaxDescRate);
        GaugeRow(Specs.TermMaxTiltDeg, ref _s.TermMaxTiltDeg);
        GaugeRow(Specs.TermNudgeStep, ref _s.TermNudgeStep);
        GuidanceRegion.End();
    }

    private static void DrawHoverReadout(Orbit orbit, IParentBody parent, bool active)
    {
        if (!active || !TierScope.IsPrimaryPass)
            return;

        double3 r = orbit.StateVectors.PositionCci;
        double3 up = double3.Normalize(r);
        double3 vSrf = orbit.StateVectors.VelocityCci
                     - double3.Cross(parent.GetAngularVelocityCci(), r);
        (double3 east, double3 north) = EnuBasis(up);

        GaugeRowText("DRIFT", FormattableString.Invariant($"E {double3.Dot(vSrf, east):F1}, N {double3.Dot(vSrf, north):F1} m/s"));
    }

    /// <summary>The numpad nudges, polled from the console frame of the craft on screen, so they work whatever the console shows. Not while a text field takes the keys.</summary>
    internal static void PollHoverNumpad()
    {
        if (_s.LandingPhase != LandingPhase.TerminalHover || !UiConfig.GetBool(UiConfig.KeyHoverNumpad, false)
            || ImGui.GetIO().WantTextInput)
            return;
        if (ImGui.IsKeyPressed(ImGuiKey.Keypad8)) _s.TermSetN += _s.TermNudgeStep;
        if (ImGui.IsKeyPressed(ImGuiKey.Keypad2)) _s.TermSetN -= _s.TermNudgeStep;
        if (ImGui.IsKeyPressed(ImGuiKey.Keypad6)) _s.TermSetE += _s.TermNudgeStep;
        if (ImGui.IsKeyPressed(ImGuiKey.Keypad4)) _s.TermSetE -= _s.TermNudgeStep;
        if (ImGui.IsKeyPressed(ImGuiKey.Keypad9)) _s.TermSetUp += _s.TermNudgeStep;
        if (ImGui.IsKeyPressed(ImGuiKey.Keypad3)) _s.TermSetUp -= _s.TermNudgeStep;
        if (ImGui.IsKeyPressed(ImGuiKey.Keypad5, false)) ZeroTerminalSetpoints();
    }

    private static void DrawHoverSetpoints(float innerW)
    {
        bool numpad = UiConfig.GetBool(UiConfig.KeyHoverNumpad, false);

        // Laid out as the numpad it mirrors, so the keys and the buttons are the same control rather than two things that happen to agree. A NUDGE label column sits in front, as in the other rows of the page.
        float labelW = ConsoleUi.LabelColumnWidth;
        float scale = ImGuiHelper.InterfaceScale;
        float w = MathF.Max(48f * scale, MathF.Min(80f * scale, (innerW - labelW) / 4.5f));
        var pad = new float2(w, ImGui.GetTextLineHeightWithSpacing() * 1.2f);

        // Compass on the left as a 3x3, vertical on the right as its own stack, with a clear gap between them: they are different axes, and interleaving Up and Down into the compass rows made one control out of two.
        float step = w + ImGui.GetStyle().ItemSpacing.X;
        float gap = w * 0.45f;
        float2 rowStart = ImGui.GetCursorScreenPos();
        ConsoleStyle.PushLabelFont();
        ImGui.GetWindowDrawList().AddText(rowStart + new float2(0f, (pad.Y - ImGui.GetFontSize()) * 0.5f),
            ConsoleStyle.ApplyAlpha(ConsoleStyle.TextLabelU32), "NUDGE");
        ConsoleStyle.PopFont();
        if (!numpad && ConsoleUi.Hovering(rowStart, rowStart + new float2(labelW, pad.Y)))
            ConsoleUi.Tooltip(PageText.NumpadHint.AsSpan());
        float2 origin = rowStart + new float2(labelW, 0f);
        float colX = origin.X + step * 3f + gap;

        void At(int col, int row) =>
            ImGui.SetCursorScreenPos(new float2(origin.X + step * col,
                origin.Y + (pad.Y + ImGui.GetStyle().ItemSpacing.Y) * row));

        bool Nudge(string label, string key, string id, string tip)
        {
            bool hit = ConsoleWidgets.Button((numpad ? label + " (" + key + ")" : label).AsSpan(), id.AsSpan(), pad);
            if (ImGui.IsItemHovered())
                ConsoleUi.Tooltip(tip.AsSpan());
            return hit;
        }

        At(1, 0); if (Nudge("N", "8", "AfcHoverN", PageText.NudgeNorth)) _s.TermSetN += _s.TermNudgeStep;
        At(0, 1); if (Nudge("W", "4", "AfcHoverW", PageText.NudgeWest)) _s.TermSetE -= _s.TermNudgeStep;
        At(1, 1); if (Nudge("ZERO", "5", "AfcHoverZERO", PageText.NudgeZero)) ZeroTerminalSetpoints();
        At(2, 1); if (Nudge("E", "6", "AfcHoverE", PageText.NudgeEast)) _s.TermSetE += _s.TermNudgeStep;
        At(1, 2); if (Nudge("S", "2", "AfcHoverS", PageText.NudgeSouth)) _s.TermSetN -= _s.TermNudgeStep;

        float rowH2 = pad.Y + ImGui.GetStyle().ItemSpacing.Y;
        ImGui.SetCursorScreenPos(new float2(colX, origin.Y + rowH2 * 0.5f));
        if (Nudge("UP", "9", "AfcHoverUP", PageText.NudgeUp)) _s.TermSetUp += _s.TermNudgeStep;
        ImGui.SetCursorScreenPos(new float2(colX, origin.Y + rowH2 * 1.5f));
        if (Nudge("DOWN", "3", "AfcHoverDOWN", PageText.NudgeDown)) _s.TermSetUp -= _s.TermNudgeStep;

        // The buttons were placed absolutely, so the layout cursor never advanced.
        ImGui.SetCursorScreenPos(rowStart);
        ImGui.Dummy(new float2(innerW, rowH2 * 3f));

        if (numpad)
            ConsoleUi.Muted("Numpad: 8 and 2 N and S, 4 and 6 W and E, 9 and 3 up and down, 5 zero, while the craft hovers.".AsSpan());
    }
}

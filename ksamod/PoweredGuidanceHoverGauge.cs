using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The Hover sub-tab: the last stretch of the descent, flown by hand on velocity
// setpoints. The live readout and the nudge pad are visible, tuning is folded away.
//
// DELIBERATELY NO TRAJECTORY PLOT. The hover flies a rate profile rather than a
// trajectory, so there is no plan to draw a flown path against and the picture says
// nothing the numbers above it do not. It still FEEDS the shared trace — hover is the
// tail of the same descent — so the G-FOLD and 6-DOF pages show the whole thing
// including this phase.
public static partial class PoweredGuidanceWindow
{
    private static void DrawHoverTabContent(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                            double mu, double bodyRadius, float innerW)
    {
        bool active = _s.LandingPhase == LandingPhase.TerminalHover;

        // Hover is impossible below a TWR of one — the engine cannot hold the weight,
        // let alone descend on a profile — so this is a go/no-go, not a statistic.
        double twr = TerminalTwr(vehicle, orbit, mu);
        if (twr < 1.0)
            ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f),
                $"TWR {twr:F2} < 1 - hover NOT possible (thrust cannot hold weight).");
        else
            ImGui.TextColored(new float4(0.7f, 0.7f, 0.7f, 1f), $"TWR {twr:F2} (local gravity)");

        DrawHoverReadout(orbit, parent, bodyRadius, active, innerW);
        DrawHoverSetpoints(active, innerW);

        if (ImGuiHelper.BeginRegion("Hover parameters", ImGuiTreeNodeFlags.SpanAllColumns, innerW))
        {
            GaugeRow("Touchdown rate (m/s)", "##tmtd", ref _s.TermTouchdownRate);
            GaugeRow("Constant-rate zone (m)", "##tmconst", ref _s.TermConstAltM);
            GaugeRow("Profile quad k (1/(m s))", "##tmquad", ref _s.TermQuadK);
            GaugeRow("Max descent rate (m/s)", "##tmmaxdesc", ref _s.TermMaxDescRate);
            GaugeRow("Max tilt (deg)", "##tmtilt", ref _s.TermMaxTiltDeg);
            GaugeRow("Vertical Kp", "##tmkpv", ref _s.TermKpV);
            GaugeRow("Vertical Ki", "##tmkiv", ref _s.TermKiV);
            GaugeRow("Vertical Kd", "##tmkdv", ref _s.TermKdV);
            GaugeRow("Lateral Kp", "##tmkpl", ref _s.TermKpL);
            GaugeRow("Lateral Ki", "##tmkil", ref _s.TermKiL);
            GaugeRow("Lateral Kd", "##tmkdl", ref _s.TermKdL);
            GaugeRow("Nudge step (m/s)", "##tmnudge", ref _s.TermNudgeStep);
            ImGuiHelper.EndRegion();
        }
    }

    private static void DrawHoverReadout(Orbit orbit, IParentBody parent, double bodyRadius,
                                         bool active, float innerW)
    {
        if (!ImGuiHelper.BeginRegion("Hover", ImGuiTreeNodeFlags.DefaultOpen
                | ImGuiTreeNodeFlags.SpanAllColumns, innerW))
            return;

        if (active)
        {
            double3 r = orbit.StateVectors.PositionCci;
            double3 up = double3.Normalize(r);
            double3 vSrf = orbit.StateVectors.VelocityCci
                         - double3.Cross(parent.GetAngularVelocityCci(), r);
            (double3 east, double3 north) = EnuBasis(up);

            GaugeRowText("Alt (legs)", $"{TerminalHeight(orbit, parent, bodyRadius):F1} m");
            GaugeRowText("Vertical", $"{double3.Dot(vSrf, up):F1} m/s");
            GaugeRowText("East / North",
                $"{double3.Dot(vSrf, east):F1} / {double3.Dot(vSrf, north):F1} m/s");
            GaugeRowText("Throttle", $"{_s.GfoldThrottle * 100:F0} %");
        }
        else
        {
            GaugeRowText("State", "idle - EXECUTE starts the hover");
        }

        ImGuiHelper.EndRegion();
    }

    private static void DrawHoverSetpoints(bool active, float innerW)
    {
        ImGui.SeparatorText("Velocity setpoints (m/s)");
        ImGui.Text($"East {_s.TermSetE,6:F1}   North {_s.TermSetN,6:F1}   Vertical bias {_s.TermSetUp,6:F1}");

        // Numpad nudges while hovering. Unlikely to clash with game bindings, and the
        // buttons below always work regardless.
        if (active)
        {
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad8)) _s.TermSetN += _s.TermNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad2)) _s.TermSetN -= _s.TermNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad6)) _s.TermSetE += _s.TermNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad4)) _s.TermSetE -= _s.TermNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad9)) _s.TermSetUp += _s.TermNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad3)) _s.TermSetUp -= _s.TermNudgeStep;
            if (ImGui.IsKeyPressed(ImGuiKey.Keypad5, false)) ZeroTerminalSetpoints();
        }

        // Laid out as the numpad it mirrors, so the keys and the buttons are the same
        // control rather than two things that happen to agree.
        float w = MathF.Max(48f, MathF.Min(80f, innerW / 4.5f));
        var pad = new float2(w, ImGui.GetTextLineHeightWithSpacing() * 1.2f);

        // Compass on the left as a 3x3, vertical on the right as its own stack, with a
        // clear gap between them: they are different axes, and interleaving Up and Down
        // into the compass rows made one control out of two.
        float step = w + ImGui.GetStyle().ItemSpacing.X;
        float gap = w * 0.45f;
        float2 origin = ImGui.GetCursorScreenPos();
        float colX = origin.X + step * 3f + gap;

        void At(int col, int row) =>
            ImGui.SetCursorScreenPos(new float2(origin.X + step * col,
                origin.Y + (pad.Y + ImGui.GetStyle().ItemSpacing.Y) * row));

        At(1, 0); if (ImGui.Button("N (8)", pad)) _s.TermSetN += _s.TermNudgeStep;
        At(0, 1); if (ImGui.Button("W (4)", pad)) _s.TermSetE -= _s.TermNudgeStep;
        At(1, 1); if (ImGui.Button("ZERO (5)", pad)) ZeroTerminalSetpoints();
        At(2, 1); if (ImGui.Button("E (6)", pad)) _s.TermSetE += _s.TermNudgeStep;
        At(1, 2); if (ImGui.Button("S (2)", pad)) _s.TermSetN -= _s.TermNudgeStep;

        float rowH2 = pad.Y + ImGui.GetStyle().ItemSpacing.Y;
        ImGui.SetCursorScreenPos(new float2(colX, origin.Y + rowH2 * 0.5f));
        if (ImGui.Button("Up (9)", pad)) _s.TermSetUp += _s.TermNudgeStep;
        ImGui.SetCursorScreenPos(new float2(colX, origin.Y + rowH2 * 1.5f));
        if (ImGui.Button("Down (3)", pad)) _s.TermSetUp -= _s.TermNudgeStep;

        // The buttons were placed absolutely, so the layout cursor never advanced.
        ImGui.SetCursorScreenPos(origin);
        ImGui.Dummy(new float2(innerW, rowH2 * 3f));

        ImGui.TextColored(new float4(0.7f, 0.7f, 0.7f, 1f),
            "Numpad: 8/2 N/S, 4/6 W/E, 9/3 up/down, 5 zero (while hovering).");
    }
}

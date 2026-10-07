#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The Powered landing tab's 3-DOF content, beside the 6-DOF page: what the guidance is doing, how it engages, where the burn aims, and the 6-DOF handover. The guidance is Modes/ThreeDof.cs.
public static partial class GuidanceWindow
{
    private static void Draw3DofLandingContent(Vehicle vehicle, float innerW)
    {
        var good = new float4(0.4f, 1f, 0.4f, 1f);
        var warn = new float4(1f, 0.8f, 0.3f, 1f);
        var dim = new float4(0.7f, 0.7f, 0.7f, 1f);

        // A FIXED LAYOUT: every row is drawn every frame, with placeholders when it has nothing to say, and the numbers sit in their own fixed-width rows rather than in the status line. The panel sizes itself to its content, so a row that comes and goes - the solve row used to, every time the solver thread went busy - or a line whose length changes every frame shakes the whole window.
        ImGui.Text("3-DOF");
        ImGui.SameLine();
        ImGui.TextColored(ThreeDofLive ? good : dim, ThreeDofPhaseName(_s.ThreeDofPhase));
        ImGui.TextColored(ThreeDofLive ? dim : warn, _s.ThreeDofStatus.Length > 0 ? _s.ThreeDofStatus : " ");

        if (ThreeDofLive)
        {
            // Draw-time writes are erased by the sim copy-back, so the abort only sets state the next step acts on; see Disengage3Dof.
            if (ImGui.Button("Abort 3-DOF"))
                Disengage3Dof("Aborted.");
        }
        else if (ImGui.Button("Engage 3-DOF"))
        {
            _s.AutoStage = true;
            _s.ThreeDofEngagePending = true;
        }

        if (ImGuiHelper.BeginRegion("Plan", ImGuiTreeNodeFlags.DefaultOpen | ImGuiTreeNodeFlags.SpanAllColumns, innerW))
        {
            var plan = _s.ThreeDofPlan;
            double now = SimNow();
            bool gliding = plan != null && _s.ThreeDofPhase == ThreeDofPhase.Glide;
            GaugeRowText("Ignition in", gliding ? $"{plan.IgnitionTime - now,8:F1} s" : "       -");
            GaugeRowText("Burn", plan == null ? "       -"
                : _s.ThreeDofPhase == ThreeDofPhase.Burn ? $"{plan.EndTime - now,8:F1} s left" : $"{plan.SigmaBurn,8:F1} s");
            GaugeRowText("Throttle", _s.ThreeDofPhase == ThreeDofPhase.Burn ? $"{_s.ThreeDofThrottle,8:P0}" : "       -", dim);
            var est = _s.ThreeDofEstimator;
            GaugeRowText("Model correction", est != null
                ? $"drag x{est.DragScale:F2}  lift x{est.LiftScale:F2}  thrust x{est.ThrustScale:F2}" : "       -", dim);
            // As of the solver's last idle moment: the guidance is the worker's while it solves, so the step caches this rather than the draw reading it.
            GaugeRowText("Last solve", _s.ThreeDofSolveText.Length > 0 ? _s.ThreeDofSolveText : "       -",
                _s.ThreeDofRefusing ? warn : dim);
            ImGuiHelper.EndRegion();
        }

        if (ImGuiHelper.BeginRegion("Descent", ImGuiTreeNodeFlags.DefaultOpen | ImGuiTreeNodeFlags.SpanAllColumns, innerW))
        {
            GaugeRowCheck("Take over boostback glides", "##tdglide", ref _s.ThreeDofFromGlide);
            using (new ImGuiDisabledScope(!_s.ThreeDofFromGlide))
                GaugeRow("At time to impact (s)", "##tdtgo", ref _s.ThreeDofEngageTgoS);
            GaugeRow("Aim height (m)", "##tdaimh", ref _s.ThreeDofAimHeightM);
            GaugeRow("Aim sink (m/s)", "##tdaimv", ref _s.ThreeDofAimSinkMs);
            GaugeRow("Command smoothing (s)", "##tdtau", ref _s.ThreeDofSmoothTau);
            GaugeRowCheck("Hand burn to 6-DOF", "##td6", ref _s.ThreeDofSixDofHandover);
            using (new ImGuiDisabledScope(!_s.ThreeDofSixDofHandover))
                GaugeRow("Below speed (m/s)", "##td6v", ref _s.ThreeDofSixDofSpeedMs);
            ImGuiHelper.EndRegion();
        }
    }
}

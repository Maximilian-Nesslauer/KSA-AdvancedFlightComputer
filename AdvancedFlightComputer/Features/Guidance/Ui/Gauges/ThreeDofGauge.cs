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

        // The side view: the plan, glide then burn, against the flown path, with the ignition marked until it happens.
        float plotH = ImGui.GetTextLineHeightWithSpacing() * 6.5f;
        Draw3DofPlot(ImGui.GetWindowDrawList(), ImGui.GetCursorScreenPos(), new float2(innerW, plotH));
        ImGui.Dummy(new float2(innerW, plotH));
        ImDrawListPtr legendDl = ImGui.GetWindowDrawList();
        float2 legend = ImGui.GetCursorScreenPos();
        legendDl.AddText(legend, GfoldPlanCol, "glide");
        legendDl.AddText(legend + new float2(44f, 0f), GfoldThrustCol, "burn");
        legendDl.AddText(legend + new float2(84f, 0f), GfoldFlownCol, "flown");
        legendDl.AddText(legend + new float2(132f, 0f), ThreeDofIgnitionCol, "x ignition");
        ImGui.Dummy(new float2(innerW, ImGui.GetTextLineHeight()));

        // A FIXED LAYOUT: every row is drawn every frame, with placeholders when it has nothing to say, and the numbers sit in their own fixed-width rows rather than in the status line. The panel sizes itself to its content, so a row that comes and goes - the solve row used to, every time the solver thread went busy - or a line whose length changes every frame shakes the whole window.
        ImGui.Text("Entry guidance");
        ImGui.SameLine();
        ImGui.TextColored(ThreeDofLive ? good : dim, ThreeDofPhaseName(_s.ThreeDofPhase));
        ImGui.TextColored(ThreeDofLive ? dim : warn, _s.ThreeDofStatus.Length > 0 ? _s.ThreeDofStatus : " ");

        if (ThreeDofLive)
        {
            // Draw-time writes are erased by the sim copy-back, so the abort only sets state the next step acts on; see Disengage3Dof.
            if (ImGui.Button("Abort entry guidance"))
                Disengage3Dof("Aborted.");
        }
        else if (ImGui.Button("Engage entry guidance"))
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
            GaugeRowText("Throttle", _s.ThreeDofPhase == ThreeDofPhase.Burn
                ? $"{_s.ThreeDofThrottle,8:P0}  (plan {_s.ThreeDofPlanThrottle:P0})" : "       -", dim);
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
            VehicleHeightRows("td");
            GaugeRow("Max burn accel (g, 0 off)", "##tdaccel", ref _s.LandingAccelLimitG);
            GaugeRow("Command smoothing (s)", "##tdtau", ref _s.ThreeDofSmoothTau);
            // 0 is the vehicle's own, read from its RCS when the cold solve starts; the value in use beside it.
            GaugeRow($"Plan turn accel, deg/s2 (0 auto: {_s.ThreeDofTurnAccelInUse:F1})", "##tdturn", ref _s.ThreeDofTurnAccelDegS2);
            GaugeRow("Plan max throttle, glide", "##tdceilg", ref _s.ThreeDofGlideCeiling);
            GaugeRow("Plan max throttle, burn", "##tdceilb", ref _s.ThreeDofBurnCeiling);
            // Shared with the touchdown page: both plans keep this much throttle above the engines' minimum for their feedback.
            GaugeRow("Plan min throttle above engine min", "##tdfloor", ref _s.LandingPlanFloorMargin);
            _s.LandingPlanFloorMargin = Math.Clamp(_s.LandingPlanFloorMargin, 0.0, 0.5);
            GaugeRow("Burn track Kp (1/s2)", "##tdkp", ref _s.ThreeDofTrackKp);
            GaugeRow("Burn track Kd (1/s)", "##tdkd", ref _s.ThreeDofTrackKd);
            GaugeRow("Burn track max tilt (deg)", "##tdtilt", ref _s.ThreeDofTrackTiltDeg);
            GaugeRowCheck("Hand burn to touchdown", "##td6", ref _s.ThreeDofSixDofHandover);
            using (new ImGuiDisabledScope(!_s.ThreeDofSixDofHandover))
                GaugeRow("With burn left (s)", "##td6t", ref _s.ThreeDofSixDofTimeS);
            ImGuiHelper.EndRegion();
        }
    }

    private static readonly ImColor8 ThreeDofIgnitionCol = new ImColor8(255, 70, 70);

    /// <summary>
    /// Entry guidance's side view, range to the site against height: the plan's glide and burn, the flown path, and a red cross at the planned ignition while the craft is still gliding. Axes fit the plan and the craft, rounded up to a step so they do not crawl as the plan is re-solved.
    /// </summary>
    private static void Draw3DofPlot(ImDrawListPtr dl, float2 min, float2 size)
    {
        dl.AddRect(min, min + size, SchemSpent, 3f);
        var plan = _s.ThreeDofPlan;
        const int nx = 7;
        if (plan == null || plan.Nodes < 2 || plan.X.Length < plan.Nodes * nx)
        {
            dl.AddText(min + new float2(6f, 4f), SchemDim, ThreeDofLive ? "no entry plan yet" : "entry guidance idle");
            return;
        }

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

        float padL = 8f, padR = 10f, padT = 18f, padB = 12f;
        float2 plotMin = min + new float2(padL, padT);
        float2 plotSize = new float2(size.X - padL - padR, size.Y - padT - padB);
        if (plotSize.X < 24f || plotSize.Y < 16f)
            return;
        float2 plotMax = plotMin + plotSize;
        float2 ToPlot(double rangeM, double altM) => new float2(
            plotMax.X - plotSize.X * (float)(rangeM / axR),
            plotMax.Y - plotSize.Y * (float)(altM / axA));
        float2 Node(int k) => ToPlot(Math.Sqrt(plan.X[k * nx] * plan.X[k * nx] + plan.X[k * nx + 1] * plan.X[k * nx + 1]),
                                     plan.X[k * nx + 2]);

        float groundY = ToPlot(0.0, 0.0).Y;
        dl.AddLine(new float2(plotMin.X, groundY), new float2(plotMax.X, groundY), SchemSpent, 1.5f);
        dl.AddCircleFilled(ToPlot(0.0, 0.0), 3f, SchemInk);

        // Glide intervals first, then the burn; the node between them is the ignition.
        int ignition = Math.Clamp(plan.GlideIntervals, 0, plan.Nodes - 1);
        for (int k = 0; k < plan.Nodes - 1; k++)
            ClipLine(dl, Node(k), Node(k + 1), plotMin, plotMax, k < ignition ? GfoldPlanCol : GfoldThrustCol, 2f);
        DrawGfoldFlownPath(dl, ToPlot, plotMin, plotMax);

        double now = SimNow();
        if (_s.ThreeDofPhase == ThreeDofPhase.Glide && plan.IgnitionTime > now)
        {
            float2 x = Node(ignition);
            float r = 5f;
            dl.AddLine(x + new float2(-r, -r), x + new float2(r, r), ThreeDofIgnitionCol, 2.5f);
            dl.AddLine(x + new float2(-r, r), x + new float2(r, -r), ThreeDofIgnitionCol, 2.5f);
        }

        string head = _s.ThreeDofPhase == ThreeDofPhase.Glide && plan.IgnitionTime > now
            ? $"ignition in {plan.IgnitionTime - now:F0} s"
            : _s.ThreeDofPhase == ThreeDofPhase.Burn ? $"burning, {Math.Max(plan.EndTime - now, 0.0):F0} s left" : "";
        dl.AddText(min + new float2(6f, 4f), SchemDim, head);
        string axes = $"{axR / 1000.0:F1} x {axA / 1000.0:F1} km";
        dl.AddText(new float2(min.X + size.X - 6f - ImGui.CalcTextSize(axes).X, min.Y + 4f), SchemDim, axes);
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

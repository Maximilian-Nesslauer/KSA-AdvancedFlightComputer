using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The Powered landing tab's 6-DOF content. Same shape as the G-FOLD page: what the
// guidance is doing, the one knob that gets set per flight, the debug toggles, and
// every solver parameter folded away.
//
// The one visible knob is TARGET ALTITUDE, because it is the only one that describes
// the flight rather than the optimiser: it is where the plan levels off, and it is
// set per landing. The rest tune how the problem is solved and are left alone once
// they work.
public static partial class PoweredGuidanceWindow
{
    private static void Draw6DofLandingContent(Vehicle vehicle, float innerW)
    {
        // Same descent plot the G-FOLD and hover pages carry, drawing this solver's
        // plan against the flown path.
        float plotH = ImGui.GetTextLineHeightWithSpacing() * 6.5f;
        DrawGfoldPlot(ImGui.GetWindowDrawList(), ImGui.GetCursorScreenPos(),
            new float2(innerW, plotH));
        ImGui.Dummy(new float2(innerW, plotH));

        Draw6DofStatusLine(vehicle);

        if (ImGuiHelper.BeginRegion("Descent", ImGuiTreeNodeFlags.DefaultOpen
                | ImGuiTreeNodeFlags.SpanAllColumns, innerW))
        {
            GaugeRow("Target altitude (m)", "##sdtargetalt", ref _s.SixDofTargetAltM);

            // HANDOFF SITS WITH TARGET ALTITUDE, not in the parameter fold, because the
            // two only mean anything relative to each other: the handoff must be ABOVE
            // the target or it never fires, and the guidance then flies to the target
            // and sits there. Separating them by a collapsed section made it possible
            // to move one and forget the other, which reads in flight as the hover
            // simply not happening.
            GaugeRowCheck("Hand off to terminal hover", "##sdhover", ref _s.SixDofHoverHandoff);
            using (new ImGuiDisabledScope(!_s.SixDofHoverHandoff))
                GaugeRow("Handoff altitude (m)", "##sdhoveralt", ref _s.SixDofHoverHandoffAltM);
            if (_s.SixDofHoverHandoff && _s.SixDofHoverHandoffAltM <= _s.SixDofTargetAltM)
                GaugeRowText("  warning", "at or below target altitude - will never fire",
                    new float4(1f, 0.8f, 0.3f, 1f));

            ImGuiHelper.EndRegion();
        }

        // Debug toggles stay OUT of the fold, as on the G-FOLD page: they are what you
        // reach for while something is going wrong.
        ImGui.Checkbox("Show plan overlay", ref _show6DofOverlay);
        ImGui.SameLine();
        ImGui.Checkbox("Log telemetry to file", ref _s.SixDofLogging);
        Draw6DofLogStatus();

        if (_s.Error.Length > 0)
            ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f), _s.Error);

        if (ImGuiHelper.BeginRegion("6-DOF parameters", ImGuiTreeNodeFlags.SpanAllColumns, innerW))
        {
            Draw6DofParameters();
            ImGuiHelper.EndRegion();
        }
    }

    private static void Draw6DofStatusLine(Vehicle vehicle)
    {
        if (!_s.Active)
        {
            ImGui.TextColored(new float4(0.7f, 0.7f, 0.7f, 1f),
                _s.EngagePending ? "Engaging..." : "Idle - EXECUTE engages 6-DOF guidance.");
            // The cold solve is ~1.7 s of sim thread unless it is threaded or spread,
            // which is worth knowing BEFORE committing rather than after the hitch.
            if (!_s.SixDofThreaded && !_s.SixDofSpreadCold)
                ImGui.TextWrapped("Cold solve blocks the sim thread for ~1.7 s - engage during a coast, "
                    + "or turn on threading or cold-solve spreading in the parameters.");

            // Whether a plan can exist AT ALL, before committing rather than after a
            // failed solve. This is the readout that explains a refusal to engage.
            Draw6DofFeasibility(vehicle);
            return;
        }

        Ksa6DofGuidance g = _s.Guidance;
        if (g == null || !g.HasPlan)
        {
            ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                _s.Converging ? "Cold solve in progress..." : "Engaged - no plan yet.");
            return;
        }

        ImGui.TextColored(new float4(0.4f, 1f, 0.5f, 1f),
            $"Engaged - burn {g.Sigma,5:F1} s over {g.Nodes} nodes, throttle {_s.LastThrottle * 100,3:F0} %");
    }

    private static void Draw6DofLogStatus()
    {
        // One global sink with one owner, so say plainly whether it is recording THIS
        // craft: "logging" on a vehicle whose rows are going nowhere is worse than
        // silence.
        if (SixDofLog.Enabled && ReferenceEquals(SixDofLog.Owner, _s))
        {
            ImGui.TextColored(new float4(0.4f, 1f, 0.5f, 1f),
                $"logging {SixDofLog.RunName} - {SixDofLog.RowsWritten} rows");
            ImGui.SameLine();
            if (ImGui.Button("Flush"))
                SixDofLog.Flush();
        }
        else if (SixDofLog.Enabled)
            ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                $"another vehicle owns the log ({SixDofLog.RunName}) - this one is not being recorded");
        else if (_s.SixDofLogging)
            ImGui.Text("logging will start when guidance engages");

        if (SixDofLog.LastError.Length > 0)
            ImGui.TextColored(new float4(1f, 0.5f, 0.3f, 1f), "log error: " + SixDofLog.LastError);
    }

    private static void Draw6DofParameters()
    {
        ImGui.Text("Nodes");
        ImGui.NextColumn();
        ImGui.PushItemWidth(-1f);
        ImGui.InputInt("##sdnodes", ref _s.SixDofNodes);
        ImGui.PopItemWidth();
        ImGui.NextColumn();
        _s.SixDofNodes = Math.Clamp(_s.SixDofNodes, MinNodes, MaxNodes);

        GaugeRow("Tilt limit (deg)", "##sdtilt", ref _s.SixDofTiltDeg);

        GaugeRowCheck("Throttle floor from vehicle", "##sdfloorauto", ref _s.SixDofFloorAuto);
        using (new ImGuiDisabledScope(_s.SixDofFloorAuto))
            GaugeRow("Throttle floor", "##sdfloor", ref _s.SixDofThrottleFloor);

        // Seeds the cold solve from a G-FOLD solution instead of a straight line.
        // SCvx refines a reference rather than searching for one, so the seed decides
        // how many iterations the cold solve needs and which local solution it walks
        // toward - and G-FOLD also supplies a far better burn time than a fixed guess.
        // Strictly an optimisation: a failure falls back to the straight-line seed.
        GaugeRowCheck("Seed cold solve from G-FOLD", "##sdgfseed", ref _s.SixDofGfoldSeed);

        GaugeRowCheck("Fixed burn time", "##sdfixedtime", ref _s.SixDofFixedTime);
        GaugeRow("Burn time seed (s)", "##sdsigmaseed", ref _s.SixDofSigmaSeed);
        if (_s.SixDofFixedTime)
        {
            ImGui.Text("Burn-time samples");
            ImGui.NextColumn();
            ImGui.PushItemWidth(-1f);
            ImGui.InputInt("##sdsigmasamples", ref _s.SixDofSigmaSamples);
            ImGui.PopItemWidth();
            ImGui.NextColumn();
        }

        GaugeRow("Glide slope (deg, 0 = off)", "##sdglide", ref _s.SixDofGlideSlopeDeg);

        GaugeRowCheck("Limit climb rate", "##sdvz", ref _s.SixDofVzEnabled);
        using (new ImGuiDisabledScope(!_s.SixDofVzEnabled))
            GaugeRow("Max climb rate (m/s)", "##sdvzmax", ref _s.SixDofVzMaxMs);

        GaugeRowCheck("Reduce nodes on approach", "##sdgates", ref _s.SixDofNodeGates);
        using (new ImGuiDisabledScope(!_s.SixDofNodeGates))
            GaugeRow("Target node spacing (s)", "##sdnodedt", ref _s.SixDofNodeDtTarget);
        if (_s.SixDofNodeGates && _s.Guidance != null && _s.Guidance.HasPlan)
            GaugeRowText("  actual",
                $"{_s.Guidance.Sigma / Math.Max(_s.Guidance.Nodes - 1, 1):F2} s");

        // Switching threading mid-flight is the point of having the toggle, so it has
        // to be safe: stop the worker on the way off, start one on the way on.
        if (GaugeRowCheck("Solve on a background thread", "##sdthreaded", ref _s.SixDofThreaded))
        {
            if (!_s.SixDofThreaded) { _s.Worker?.Dispose(); _s.Worker = null; }
            else if (_s.Active && _s.Worker == null) _s.Worker = new Ksa6DofSolveWorker();
        }
        if (_s.SixDofThreaded && _s.Worker != null)
            GaugeRowText("  worker",
                $"{_s.Worker.Completed} solves, {_s.Worker.Skipped} skipped, {_s.Worker.LastSolveMs:F0} ms"
                + (_s.Worker.IsBusy ? "  [solving]" : ""));

        GaugeRowCheck("Spread cold solve over frames", "##sdspread", ref _s.SixDofSpreadCold);
        using (new ImGuiDisabledScope(!_s.SixDofSpreadCold))
            GaugeRow("Gap between iterations (s)", "##sdcoldint", ref _s.SixDofColdIntervalS);
        _s.SixDofColdIntervalS = Math.Clamp(_s.SixDofColdIntervalS, 0.0, 1.0);

        GaugeRowCheck("Estimate unmodelled accel", "##sdbias", ref _s.SixDofBiasEnabled);

        GaugeRow("Re-solve every (s)", "##sdreplan", ref _s.SixDofReplanSec);
        GaugeRow("Thrust fraction", "##sdthrustfrac", ref _s.SixDofThrustFrac);
        GaugeRow("Rate damping (fuel share)", "##sdratedamp", ref _s.SixDofRateDampShare);
        GaugeRow("Control smoothing (W_DU)", "##sdsmooth", ref _s.SixDofControlSmooth);
        GaugeRow("Proximal conditioning", "##sdprox", ref _s.SixDofProximal);
    }
}

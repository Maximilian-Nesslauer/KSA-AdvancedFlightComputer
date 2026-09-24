#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

// The convex ascent's controls and its plan, on the Ascent tab. The point of drawing the plan here before EXECUTE is to see that the calculation worked: the altitude profile should rise smoothly to the target, the pitch should fall steadily with speed, and q-alpha should ride its limit through max-q rather than cross it. See Modes/ConvexAscent.cs for what the button sets going.
public static partial class GuidanceWindow
{
    private static readonly float4 CvxGood = new(0.4f, 1f, 0.4f, 1f);
    private static readonly float4 CvxWarn = new(1f, 0.8f, 0.3f, 1f);
    private static readonly float4 CvxLive = new(0.5f, 0.9f, 1f, 1f);
    private static readonly float4 CvxDim = new(0.7f, 0.7f, 0.7f, 1f);

    private static void DrawConvexAscentSection(Vehicle vehicle, Orbit orbit, IParentBody parent, float innerW)
    {
        if (!ImGuiHelper.BeginRegion("Convex ascent",
                ImGuiTreeNodeFlags.DefaultOpen | ImGuiTreeNodeFlags.SpanAllColumns, innerW))
            return;

        bool busy = _s.AscentPlanJob != null || _s.AscentPlanRequested;

        GaugeRowCheck("Fly convex profile", "##flyconvex", ref _s.FlyConvexAscent);
        // The limits are part of the problem, so they are frozen while it is being solved.
        using (new ImGuiDisabledScope(busy))
        {
            GaugeRow("Max q (kPa)", "##cvxq", ref _s.ConvexQMaxKpa);
            GaugeRow("Max q-alpha (Pa rad)", "##cvxqa", ref _s.ConvexQAlphaMax);
            // The script's 99 % keeps the plan at full throttle. Lower lets it throttle the liquid stages - through max-q, typically - and the profile then flies that throttle too; solid stages stay at full thrust whatever this says.
            GaugeRow("Min throttle (%)", "##cvxthrottle", ref _s.ConvexThrottleMinPct);
        }
        GaugeRow("UPFG from (km)", "##cvxhandover", ref _s.ConvexHandoverAltKm);
        GaugeRowCheck("Show plan", "##cvxshow", ref _s.ShowConvexPlan);

        ImGui.Text("");
        ImGui.NextColumn();
        DrawConvexButtons();
        ImGui.NextColumn();

        DrawConvexStatusRows(vehicle, orbit, parent);

        ImGuiHelper.EndRegion();

        AscentPlan plan = _s.AscentPlan;
        if (_s.ShowConvexPlan && plan?.Series != null && plan.Solution.Nodes > 1)
            DrawConvexPlots(plan, orbit, parent, ImGui.GetContentRegionAvail().X);
    }

    /// <summary>Calculate, or cancel the calculation in progress, and clear.</summary>
    private static void DrawConvexButtons()
    {
        bool busy = _s.AscentPlanJob != null || _s.AscentPlanRequested;
        if (busy)
        {
            if (ImGui.Button("Cancel calculation##cvx"))
            {
                _s.AscentPlanJob?.Cancel();
                _s.AscentPlanRequested = false;
            }
            return;
        }

        // Not while flying: the flight has latched its plan, and a new one solved from mid-air would start from a state that is not a lift-off.
        using (new ImGuiDisabledScope(_s.Running))
        {
            if (ImGui.Button("CALCULATE ASCENT##cvx"))
            {
                _s.AscentPlanRequested = true;
                _s.AscentPlanStatus = "Starting...";
            }
        }
        if (_s.AscentPlan != null)
        {
            ImGui.SameLine();
            if (ImGui.Button("Clear plan##cvx"))
            {
                _s.AscentPlan = null;
                _s.AscentPlanStatus = "";
            }
        }
    }

    private static void DrawConvexStatusRows(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        AscentPlanJob job = _s.AscentPlanJob;
        if (job != null)
            GaugeRowText("Status", $"{job.Progress} ({job.ElapsedSeconds:F0} s)", CvxLive);
        else if (_s.AscentPlanStatus.Length > 0)
            GaugeRowText("Status", _s.AscentPlanStatus, _s.AscentPlan?.Usable == true ? CvxGood : CvxWarn);

        AscentPlan plan = _s.AscentPlan;
        AscentSolution sol = plan?.Solution;
        if (sol == null || sol.Nodes == 0)
            return;

        GaugeRowText("Solve", $"{sol.Message}, {sol.Iterations} iterations ({sol.Accepted} accepted), {plan.WallSeconds:F1} s",
            sol.Converged ? CvxDim : CvxWarn);
        GaugeRowText("Stages planned", plan.StagesPlanned == plan.StagesAvailable
            ? $"{plan.StagesPlanned}"
            : $"{plan.StagesPlanned} of {plan.StagesAvailable} (the rest ride as payload)");
        double propLeft = sol.FinalMass - plan.FinalMassFloor;
        GaugeRowText("To orbit", $"{sol.FinalMass / 1000.0:F2} t ({Math.Max(propLeft, 0.0) / 1000.0:F2} t propellant left in the last stage)");
        GaugeRowText("Burns", string.Join(" / ", Array.ConvertAll(sol.BurnTime, b => b.ToString("F1"))) + $" s, {sol.TotalDeltaV:F0} m/s ideal");
        double lowest = plan.Profile != null ? 100.0 * plan.Profile.MinThrottle : double.NaN;
        GaugeRowText("Throttle", $"floor {plan.ThrottleMinPct:F0} %, lowest planned {lowest:F0} %"
            + (plan.SolidStages > 0 && plan.ThrottleMinPct < 99.0 ? $", {plan.SolidStages} solid stage(s) held at full" : ""), CvxDim);

        int qi = ArgMax(sol.DynamicPressure), qai = ArgMax(sol.QAlpha);
        GaugeRowText("Max q", $"{sol.DynamicPressure[qi] / 1000.0:F1} kPa at {sol.Time[qi]:F0} s (limit {plan.QMaxKpa:F0})",
            sol.DynamicPressure[qi] > plan.QMaxKpa * 1000.0 * 1.002 ? CvxWarn : CvxDim);
        if (plan.QRelaxed)
            GaugeRowText("", $"{plan.QMaxRequestedKpa:F0} kPa is out of reach at full throttle", CvxWarn);
        GaugeRowText("Max q-alpha", $"{sol.QAlpha[qai]:F0} Pa rad at {sol.Time[qai]:F0} s (limit {plan.QAlphaMax:F0})",
            sol.QAlpha[qai] > plan.QAlphaMax * 1.002 ? CvxWarn : CvxDim);
        GaugeRowText("Insertion", $"{plan.InsertionAltKm:F0} km, miss {sol.TerminalResidual[0]:F0} m / {sol.TerminalResidual[1]:F2} m/s, "
            + $"plane {sol.TerminalResidual[3]:F0} m", CvxDim);

        if (_s.Running && _s.FlyingPlan != null)
        {
            GaugeRowText("Flight", _s.Phase == AscentPhase.Profile
                ? $"flying the profile to {_s.ConvexHandoverAltKm:F0} km"
                : "handed over to UPFG", CvxLive);
            return;
        }
        if (!_s.FlyConvexAscent)
        {
            GaugeRowText("EXECUTE", "flies the gravity turn (convex profile off)", CvxDim);
            return;
        }
        if (ConvexPlanFlyable(vehicle, orbit, parent, out string why))
            GaugeRowText("EXECUTE", "flies this plan", CvxGood);
        else
            GaugeRowText("EXECUTE", "flies the gravity turn: " + why, CvxWarn);
        if (plan.TargetDiffers(_s.PeKm, _s.ApKm, _s.IncDeg, _s.LanDeg))
            GaugeRowText("", "target orbit changed since - recalculate", CvxWarn);
    }

    private static int ArgMax(double[] v)
    {
        int best = 0;
        for (int i = 1; i < v.Length; i++)
            if (v[i] > v[best]) best = i;
        return best;
    }

    // --- plots -----------------------------------------------------------------

    private static void DrawConvexPlots(AscentPlan plan, Orbit orbit, IParentBody parent, float width)
    {
        AscentPlanSeries s = plan.Series;
        float w = Math.Max(width, 120f);
        const float h = 110f;

        // The vehicle's own position on each curve, while the profile is being flown.
        double cursorSpeed = double.NaN, cursorTime = double.NaN, cursorRange = double.NaN;
        if (_s.Running && ReferenceEquals(_s.FlyingPlan, plan))
        {
            cursorSpeed = AirSpeed(orbit.StateVectors.PositionCci, orbit.StateVectors.VelocityCci, parent);
            cursorTime = _s.ConvexPlanTime;
        }

        ConvexPlot("altitude (km) against downrange (km)", s.DownrangeKm, s.AltitudeKm, null,
            w, h, _s.ConvexHandoverAltKm, "UPFG", cursorRange, s.StagingNodes);
        double[] pitch = s.PitchDeg;
        ConvexPlot("pitch (deg) against air speed (m/s)", s.AirSpeed, pitch, null,
            w, h, double.NaN, "", cursorSpeed, s.StagingNodes);
        var qPct = new double[s.QFraction.Length];
        var qaPct = new double[s.QAlphaFraction.Length];
        for (int i = 0; i < qPct.Length; i++)
        {
            qPct[i] = 100.0 * s.QFraction[i];
            qaPct[i] = 100.0 * s.QAlphaFraction[i];
        }
        ConvexPlot("q (green) and q-alpha (amber), % of limit, against time (s)", s.Time, qPct, qaPct,
            w, h, 100.0, "limit", cursorTime, s.StagingNodes);
        // Only when the plan throttles at all: at the script's floor it is a flat line at 100.
        if (plan.Profile != null && plan.Profile.MinThrottle < 0.985)
            ConvexPlot("throttle (% of full) against time (s)", s.Time, s.ThrottlePct, null,
                w, h, plan.ThrottleMinPct, "floor", cursorTime, s.StagingNodes);
    }

    /// <summary>
    /// A draw-list line plot of one or two series against a shared x, with an optional horizontal reference line, the staging nodes ticked along the bottom, and an optional cursor.
    /// </summary>
    private static void ConvexPlot(string title, double[] xs, double[] ys, double[] ys2, float width, float height,
                                   double refY, string refLabel, double cursorX, int[] marks)
    {
        var size = new float2(width, height);
        float2 origin = ImGui.GetCursorScreenPos();
        ImGui.Dummy(size);
        ImDrawListPtr dl = ImGui.GetWindowDrawList();
        dl.AddRectFilled(origin, origin + size, new ImColor8(16, 20, 26), 4f);
        dl.AddRect(origin, origin + size, new ImColor8(95, 100, 105), 4f);

        int n = Math.Min(xs.Length, ys.Length);
        double xMin = double.PositiveInfinity, xMax = double.NegativeInfinity;
        double yMin = double.PositiveInfinity, yMax = double.NegativeInfinity;
        for (int i = 0; i < n; i++)
        {
            if (!double.IsFinite(xs[i]) || !double.IsFinite(ys[i])) continue;
            xMin = Math.Min(xMin, xs[i]); xMax = Math.Max(xMax, xs[i]);
            yMin = Math.Min(yMin, ys[i]); yMax = Math.Max(yMax, ys[i]);
            if (ys2 != null && double.IsFinite(ys2[i]))
            {
                yMin = Math.Min(yMin, ys2[i]); yMax = Math.Max(yMax, ys2[i]);
            }
        }
        var textCol = new ImColor8(205, 215, 225);
        var axisCol = new ImColor8(160, 165, 170);
        if (!double.IsFinite(xMin) || n < 2)
        {
            dl.AddText(origin + new float2(8f, 8f), new ImColor8(200, 120, 120), title + " (no data)");
            return;
        }
        if (double.IsFinite(refY))
        {
            yMin = Math.Min(yMin, refY);
            yMax = Math.Max(yMax, refY);
        }
        double xSpan = Math.Max(xMax - xMin, 1e-9), ySpan = Math.Max(yMax - yMin, 1e-9);

        const float padL = 44f, padR = 8f, padTop = 18f, padBot = 16f;
        float pw = size.X - padL - padR, ph = size.Y - padTop - padBot;
        float2 P(double x, double y) => new(
            origin.X + padL + (float)((x - xMin) / xSpan) * pw,
            origin.Y + padTop + ph - (float)((y - yMin) / ySpan) * ph);

        if (double.IsFinite(refY))
        {
            ScreenLine(dl, P(xMin, refY), P(xMax, refY), new ImColor8(120, 125, 130), 1f);
            if (refLabel.Length > 0)
                dl.AddText(P(xMax, refY) + new float2(-40f, -14f), axisCol, refLabel);
        }

        var pts = new float2[n];
        for (int i = 0; i < n; i++) pts[i] = P(xs[i], ys[i]);
        dl.AddPolyline(pts, new ImColor8(60, 220, 90), ImDrawFlags.None, 2f);
        if (ys2 != null)
        {
            for (int i = 0; i < n; i++) pts[i] = P(xs[i], ys2[i]);
            dl.AddPolyline(pts, new ImColor8(255, 190, 60), ImDrawFlags.None, 2f);
        }

        // Staging: a tick on the bottom axis at each separation.
        if (marks != null)
            foreach (int m in marks)
                if (m >= 0 && m < n)
                {
                    float2 b = P(xs[m], yMin);
                    ScreenLine(dl, b, b - new float2(0f, 6f), new ImColor8(235, 235, 235), 1.5f);
                }

        if (double.IsFinite(cursorX) && cursorX >= xMin && cursorX <= xMax)
            ScreenLine(dl, P(cursorX, yMin), P(cursorX, yMax), new ImColor8(255, 215, 60), 1f);

        dl.AddText(origin + new float2(padL, 2f), textCol, title);
        dl.AddText(origin + new float2(4f, padTop - 2f), axisCol, $"{yMax:G4}");
        dl.AddText(origin + new float2(4f, size.Y - padBot - 14f), axisCol, $"{yMin:G4}");
        dl.AddText(origin + new float2(padL, size.Y - padBot + 1f), axisCol, $"{xMin:G4}");
        string xm = $"{xMax:G4}";
        dl.AddText(origin + new float2(size.X - padR - 8f * xm.Length, size.Y - padBot + 1f), axisCol, xm);
    }

    // --- the legacy window ---------------------------------------------------------

    /// <summary>The same controls in the legacy window's flat style, without the plots.</summary>
    private static void DrawConvexAscentLegacy(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        if (!ImGui.CollapsingHeader("Convex ascent (SCvx)", ImGuiTreeNodeFlags.DefaultOpen))
            return;
        bool busy = _s.AscentPlanJob != null || _s.AscentPlanRequested;
        ImGui.Checkbox("Fly convex profile", ref _s.FlyConvexAscent);
        using (new ImGuiDisabledScope(busy))
        {
            ImGui.InputDouble("Max q (kPa)", ref _s.ConvexQMaxKpa);
            ImGui.InputDouble("Max q-alpha (Pa rad)", ref _s.ConvexQAlphaMax);
            ImGui.InputDouble("Min throttle (%)", ref _s.ConvexThrottleMinPct);
        }
        ImGui.InputDouble("UPFG from (km)", ref _s.ConvexHandoverAltKm);
        ImGui.Checkbox("Show plan", ref _s.ShowConvexPlan);
        DrawConvexButtons();

        AscentPlanJob job = _s.AscentPlanJob;
        if (job != null)
            ImGui.TextColored(CvxLive, $"{job.Progress} ({job.ElapsedSeconds:F0} s)");
        else if (_s.AscentPlanStatus.Length > 0)
            ImGui.TextColored(_s.AscentPlan?.Usable == true ? CvxGood : CvxWarn, _s.AscentPlanStatus);
        if (_s.AscentPlan?.Usable == true && !_s.Running)
            ImGui.TextColored(ConvexPlanFlyable(vehicle, orbit, parent, out string why) ? CvxGood : CvxWarn,
                why.Length == 0 ? "EXECUTE flies this plan." : "EXECUTE flies the gravity turn: " + why);
    }
}

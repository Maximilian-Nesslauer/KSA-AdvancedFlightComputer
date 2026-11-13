#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Ui.Logic;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

public static partial class GuidanceWindow
{
    private static void Draw6DofLandingContent(Vehicle vehicle, float innerW)
    {
        // The descent plot is the LAND picture above this form, so the form starts with the status.
        if (TierScope.IsPrimaryPass)
        {
            Draw6DofStatusLine();

            if (!_s.Active && !_s.EngagePending && _s.Error.Length > 0 && _s.ErrorSeverity >= Severity.Warning)
            {
                bool failed = _s.ErrorSeverity == Severity.Error;
                float4 ink = failed ? ConsoleStyle.Danger : ConsoleUi.WarnInk;
                ConsoleUi.Sentence((failed && _s.FailureWhat.Length > 0 ? _s.FailureWhat + "." : Fmt.Sentence(_s.Error)).AsSpan(), in ink);
            }
        }

        if (GuidanceRegion.Begin("TOUCHDOWN GUIDANCE PARAMETERS", GuidanceRegion.Tuning))
        {
            Draw6DofParameters();
            GuidanceRegion.End();
        }

        if (GuidanceRegion.Begin("##sixdofdiag", GuidanceRegion.Diagnostics))
        {
            Draw6DofDiagnostics();
            GuidanceRegion.End();
        }
    }

    private static void Draw6DofStatusLine()
    {
        if (!_s.Active)
        {
            // Solving on standby under the landing burn of entry guidance or the braking burn, ready to take it over.
            if (_s.SixDofStandby)
            {
                double now = SimNow();
                ConsoleUi.Sentence(("STANDBY: " + SixDofStandbyText(now)).AsSpan(),
                    SixDofStandbyReady(now) ? ConsoleStyle.TextPrimary : ConsoleStyle.TextMuted);
                return;
            }
            if (_s.EngagePending)
                ConsoleUi.Muted("STARTING".AsSpan());
            return;
        }

        Ksa6DofGuidance g = _s.Guidance;
        if (g == null || !g.HasPlan)
        {
            ConsoleUi.Sentence((_s.Converging ? "SOLVING FIRST PLAN" : "STARTED, NO PLAN").AsSpan(), in ConsoleStyle.TextPrimary);
        }
    }

    // The live figures of the guidance: the standby under entry guidance, and how far the craft is off the plan between re-solves.
    private static void Draw6DofDiagnostics()
    {
        const UiTier D = UiTier.Diagnostics;
        double now = SimNow();
        GaugeRowText(D, "STATE", _s.Active ? (_s.Converging ? "FIRST PLAN" : "ACTIVE")
            : _s.SixDofStandby ? "STANDBY" : _s.EngagePending ? "STARTING" : "IDLE");
        GaugeRowText(D, "SOLVER", _s.Error.Length > 0 ? Fmt.Sentence(_s.Error) : "-", ConsoleStyle.TextMuted);
        ConsoleUi.NoteShape("AfcDiagStandby", _s.SixDofStandby ? 1 : 0);
        if (_s.SixDofStandby)
            GaugeRowText(D, "STANDBY", SixDofStandbyText(now), SixDofStandbyReady(now) ? ConsoleStyle.TextPrimary : ConsoleStyle.TextMuted);
        if (_s.SixDofStandbyRefusal.Length > 0)
            GaugeRowText(D, "STANDBY REFUSED", _s.SixDofStandbyRefusal, ConsoleUi.WarnInk);
        Ksa6DofGuidance g = _s.Guidance;
        if (g != null && g.HasPlan)
            GaugeRowText(D, "BURN TIME", Fmt.TimeFine(g.Sigma));
        if (_s.SixDofTrack && g != null && g.HasPlan)
            GaugeRowText(D, "OFF PLAN",
                FormattableString.Invariant($"{g.TrackingError.PositionM:F1} m, {g.TrackingError.VelocityMs:F1} m/s, THRUST {g.TrackingError.ThrustN / 1e3:+0;-0} kN"));
        else
            GaugeRowText(D, "OFF PLAN", _s.SixDofTrack ? "-" : "THRUST TRACKING OFF", ConsoleStyle.TextMuted);
    }

    internal static void Draw6DofLogDiag(Vehicle vehicle, float innerW)
    {
        using (TierScope.Push(UiTier.Tuning))
        {
            bool logRequested = _s.SixDofLogging;
            if (GaugeRowCheck(UiTier.Tuning, FoldNames.TelemetryLog, "##sdlog", ref logRequested))
                SetSixDofLogging(vehicle, logRequested);
            Draw6DofLogStatus();
        }
    }

    private static void Draw6DofLogStatus()
    {
        const UiTier T = UiTier.Tuning;
        // One global sink with one owner, so say plainly whether it is recording THIS craft: "logging" on a vehicle whose rows are going nowhere is worse than silence.
        if (SixDofLog.Enabled && ReferenceEquals(SixDofLog.Owner, _s))
        {
            GaugeRowText(T, "", $"Writes {SixDofLog.RunName}, {SixDofLog.RowsWritten} rows.", ConsoleStyle.TextPrimary);
            if (ConsoleUi.ButtonRight("FLUSH", "AfcSixDofFlush"))
                SixDofLog.Flush();
        }
        else if (SixDofLog.Enabled)
            GaugeRowText(T, "", $"Another craft owns the log ({SixDofLog.RunName}), so this craft is not recorded.", ConsoleUi.WarnInk);
        else if (_s.SixDofLogging)
            GaugeRowText(T, "", _s.Active ? "No log is open. Switch it off and on to try again."
                : "Starts when touchdown guidance starts.", ConsoleStyle.TextMuted);

        if (SixDofLog.LastError.Length > 0)
            GaugeRowText(T, "LOG ERROR", SixDofLog.LastError, ConsoleStyle.Danger);
    }

    // The touchdown set in three groups, so a player finds a limit or a gain without reading the solver rows. Each group head counts its changed values, because a closed group hides its marks.
    private static void Draw6DofParameters()
    {
        DrawTuningGroup("LIMITS", "SixDofLimits", true, Specs.TuningSixDofLimits, Specs.TuningSixDofLimitBools, Draw6DofLimitRows, PageText.TouchdownLimitsTip);
        DrawTuningGroup("TRACKING", "SixDofTracking", true, Specs.TuningSixDofTracking, Specs.TuningSixDofTrackingBools, Draw6DofTrackingRows, PageText.TouchdownTrackingTip);
        DrawTuningGroup("SOLVER", "SixDofSolver", false, Specs.TuningSixDofSolver, Specs.TuningSixDofSolverBools, Draw6DofSolverRows, PageText.TouchdownSolverTip);
    }

    private static void DrawTuningGroup(string name, string id, bool openByDefault, SettingSpec[] doubles, BoolSpec[] bools, Action rows, string tip)
    {
        string label = FoldRules.Label(name, default, SettingAccess.CountChanged(_s, doubles, bools));
        if (ConsoleUi.Fold(label, "Tune" + id, openByDefault || UiConfig.GetBool(Ui.Console.AfcConsole.KeyExpertOpen, false), tip))
            rows();
    }

    private static void Draw6DofLimitRows()
    {
        GaugeRow(Specs.SixDofTiltDeg, ref _s.SixDofTiltDeg);

        GaugeRowCheck(Specs.SixDofFloorAuto, ref _s.SixDofFloorAuto);
        using (new ImGuiDisabledScope(_s.SixDofFloorAuto))
            GaugeRow(Specs.SixDofThrottleFloor, ref _s.SixDofThrottleFloor);
        GaugeRow(Specs.LandingPlanFloorMargin, ref _s.LandingPlanFloorMargin);

        GaugeRow(Specs.SixDofGlideSlopeDeg, ref _s.SixDofGlideSlopeDeg);

        GaugeRowCheck(Specs.SixDofVzEnabled, ref _s.SixDofVzEnabled);
        using (new ImGuiDisabledScope(!_s.SixDofVzEnabled))
            GaugeRow(Specs.SixDofVzMaxMs, ref _s.SixDofVzMaxMs);
    }

    private static void Draw6DofTrackingRows()
    {
        GaugeRowCheck(Specs.SixDofTrack, ref _s.SixDofTrack);
        using (new ImGuiDisabledScope(!_s.SixDofTrack))
        {
            GaugeRow(Specs.SixDofTrackKp, ref _s.SixDofTrackKp);
            GaugeRow(Specs.SixDofTrackKd, ref _s.SixDofTrackKd);
        }
        GaugeRowCheck(Specs.SixDofBiasEnabled, ref _s.SixDofBiasEnabled);
        GaugeRow(Specs.SixDofReplanSec, ref _s.SixDofReplanSec);
    }

    private static void Draw6DofSolverRows()
    {
        const UiTier T = UiTier.Tuning;
        GaugeRowInt(Specs.SixDofNodes, ref _s.SixDofNodes);
        _s.SixDofNodes = Math.Clamp(_s.SixDofNodes, MinNodes, MaxNodes);

        // Seeds the cold solve from a G-FOLD solution instead of a straight line.
        // SCvx refines a reference rather than searching for one, so the seed decides how many iterations the cold solve needs and which local solution it walks toward, and G-FOLD also supplies a far better burn time than a fixed guess.
        // Strictly an optimisation: a failure falls back to the straight-line seed.
        GaugeRowCheck(Specs.SixDofGfoldSeed, ref _s.SixDofGfoldSeed);

        GaugeRowCheck(Specs.SixDofFixedTime, ref _s.SixDofFixedTime);
        GaugeRow(Specs.SixDofSigmaSeed, ref _s.SixDofSigmaSeed);
        if (_s.SixDofFixedTime)
            GaugeRowInt(Specs.SixDofSigmaSamples, ref _s.SixDofSigmaSamples);

        GaugeRowCheck(Specs.SixDofNodeGates, ref _s.SixDofNodeGates);
        using (new ImGuiDisabledScope(!_s.SixDofNodeGates))
            GaugeRow(Specs.SixDofNodeDtTarget, ref _s.SixDofNodeDtTarget);
        if (_s.SixDofNodeGates && _s.Guidance != null && _s.Guidance.HasPlan)
            GaugeRowText(T, "ACTUAL POINT SPACING",
                FormattableString.Invariant($"{_s.Guidance.Sigma / Math.Max(_s.Guidance.Nodes - 1, 1):F2} s"));

        // Switching threading mid-flight is the point of having the toggle, so it has to be safe: stop the worker on the way off, start one on the way on.
        GaugeRowCheck(Specs.SixDofThreaded, ref _s.SixDofThreaded);
        if (_s.SixDofThreaded && _s.Worker != null)
            GaugeRowText(T, "WORKER",
                FormattableString.Invariant($"{_s.Worker.Completed} SOLVES, {_s.Worker.Skipped} SKIPPED, {_s.Worker.LastSolveMs:F0} ms")
                + (_s.Worker.IsBusy ? "  [SOLVING]" : ""));

        GaugeRowCheck(Specs.SixDofSpreadCold, ref _s.SixDofSpreadCold);
        using (new ImGuiDisabledScope(!_s.SixDofSpreadCold))
            GaugeRow(Specs.SixDofColdIntervalS, ref _s.SixDofColdIntervalS);
        _s.SixDofColdIntervalS = Math.Clamp(_s.SixDofColdIntervalS, 0.0, 1.0);

        GaugeRow(Specs.SixDofRateDampShare, ref _s.SixDofRateDampShare);
        GaugeRow(Specs.SixDofControlSmooth, ref _s.SixDofControlSmooth);
        GaugeRow(Specs.SixDofProximal, ref _s.SixDofProximal);
    }
}

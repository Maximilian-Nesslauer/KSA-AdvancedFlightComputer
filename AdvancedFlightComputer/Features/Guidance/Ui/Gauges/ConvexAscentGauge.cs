#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;
using AdvancedFlightComputer.Features.Ui.Logic;
using Specs = AdvancedFlightComputer.Features.Ui.Logic.GuidanceSettingSpecs;

// The convex ascent's controls and its plan, on the LAUNCH page. The point of drawing the plan here before the launch is to see that the calculation worked: the altitude profile should rise smoothly to the target, the pitch should fall steadily with speed, and q-alpha should ride its limit through max-q rather than cross it. See Modes/ConvexAscent.cs for what the launch sets going.
public static partial class GuidanceWindow
{
    private static readonly float4 CvxWarn = Ui.Canvas.Palette.Warn.F4;
    private static readonly float4 CvxLive = ConsoleStyle.TextPrimary;
    private static readonly float4 CvxDim = ConsoleStyle.TextMuted;
    private static readonly float4 CvxFail = ConsoleStyle.Danger;

    /// <summary>
    /// The convex ascent's lines under the FLIGHT PROFILE picture of the LAUNCH page: a retry of the calculation and, when the launch plan did not converge, the choice that leaves. They stand above the folds, so a launch refused for want of a plan is never behind a fold.
    /// </summary>
    private static void DrawConvexLaunchBanner()
    {
        string retry = _s.AscentPlanJob?.Retry ?? "";
        if (retry.Length > 0)
            ColoredWrapped(CvxWarn, "Calculated climb: " + retry);

        AscentPlan shown = _s.AscentPlan;
        bool relaxed = !_s.Running && shown?.Usable == true && shown.QRelaxed;
        ConsoleUi.NoteShape("AfcLaunchRelaxed", relaxed ? 1 : 0);
        if (relaxed)
            ConsoleUi.TickedNote(ConsoleUi.EscapeFormat(PageText.QRelaxedNote(shown.QMaxRequestedKpa, shown.QMaxKpa)));

        if (_s.ConvexLaunchFailure.Length == 0)
            return;
        ColoredWrapped(CvxFail, _s.ConvexLaunchFailure);
        // The banner only chooses. The footer verb then reads LAUNCH (BACKUP) and launches through the same commit and take-over rule as every other launch.
        if (_s.ConvexBackupChosen)
        {
            ColoredWrapped(CvxWarn, "The simple turn is chosen. " + CommitResolver.BackupLaunchVerb + " in the footer launches it.");
            if (ConsoleWidgets.Button(PageText.BannerKeep.AsSpan(), "AfcConvexKeep".AsSpan(), default))
                _s.ConvexBackupChosen = false;
        }
        else
        {
            ColoredWrapped(CvxWarn, "Nothing has launched. Change settings and try again, or fly the simple turn.");
            // Not while another launch is committed: the backup is an answer to this failure, not a second ascent.
            using (new ImGuiDisabledScope(_s.Running || _s.LaunchArmed || _s.ConvexLaunchPending))
            {
                if (ConsoleWidgets.Button(PageText.BannerBackup.AsSpan(), "AfcConvexBackup".AsSpan(), default))
                    _s.ConvexBackupChosen = true;
            }
        }
        ImGui.SameLine();
        if (ConsoleWidgets.Button(PageText.BannerHide.AsSpan(), "AfcConvexDismiss".AsSpan(), default))
        {
            _s.ConvexLaunchFailure = "";
            _s.ConvexBackupChosen = false;
        }
    }

    private static void ColoredWrapped(float4 color, string text) => ConsoleUi.Sentence(Fmt.Sentence(text).AsSpan(), in color);

    private static void DrawConvexAscentSection(Vehicle vehicle, Orbit orbit, IParentBody parent, float innerW)
    {
        if (!GuidanceRegion.Begin(FoldNames.ConvexPlan, GuidanceRegion.ExpertTuningDiagnostics))
            return;

        using (ConsoleUi.Dim(_s.Running))
        using (new ImGuiDisabledScope(AscentPlanBusy))
        {
            if (HasPhysicalAtmosphere(parent))
                GaugeRow(Specs.ConvexQAlphaMax, ref _s.ConvexQAlphaMax);
            else
                GaugeRowText(Specs.ConvexQAlphaMax.Tier, Specs.ConvexQAlphaMax.Label, PageText.NoAirRow, CvxDim);
            // The default 80 % lets the plan throttle the liquid stages a little, through max-q typically, and the profile then flies that throttle too; solid stages stay at full thrust whatever this says.
            GaugeRow(Specs.ConvexThrottleMinPct, ref _s.ConvexThrottleMinPct);
            // With a target it chooses when to lift off near the window, with none the LAN, so the name says which.
            GaugeRowCheck(Specs.ConvexOptimiseTiming with { Label = PageText.OptimiseTimingLabel(_s.TargetId.Length > 0) }, ref _s.ConvexOptimiseTiming);
        }
        GaugeRow(Specs.ConvexHandoverAltKm, ref _s.ConvexHandoverAltKm);

        DrawConvexStatusRows(vehicle, orbit, parent);

        GuidanceRegion.End();
    }

    private static string PlanRowText(Vehicle vehicle, Orbit orbit, IParentBody parent, out bool fine, out bool old)
    {
        fine = false;
        old = false;
        if (_s.AscentPlan?.Usable == true && _s.AscentPlanJob == null && !_s.AscentPlanRequested && !_s.Running
            && !ConvexPlanFlyable(vehicle, orbit, parent, ExecuteLaunchInstant(), out string why))
        {
            old = true;
            return PageText.PlanState(why);
        }
        fine = _s.AscentPlan?.Usable == true;
        return ConvexPlanSummary();
    }

    private static string ConvexPlanSummary()
    {
        AscentPlanJob job = _s.AscentPlanJob;
        if (job != null)
            return PageText.PlanProgress(job.Progress, job.ElapsedSeconds);
        if (_s.AscentPlanRequested)
            return PageText.PlanStarting;
        if (_s.Running)
            return _s.FlyingPlan != null ? "Flying this plan" : "Not in use: the simple turn flies";
        if (_s.AscentPlanStatus.Length > 0)
            return _s.AscentPlanStatus;
        return "Not calculated";
    }

    private static void DrawConvexStatusRows(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        const UiTier D = UiTier.Diagnostics;
        if (!TierScope.Show(D))
            return;

        AscentPlanJob job = _s.AscentPlanJob;
        if (job != null)
        {
            if (job.Retry.Length > 0)
                GaugeRowText(D, "RETRYING", job.Retry, CvxWarn);
            GaugeRowText(D, "STATUS", ConvexPlanSummary(), CvxLive);
        }
        else if (_s.AscentPlanStatus.Length > 0)
        {
            string state = PlanRowText(vehicle, orbit, parent, out bool fine, out bool old);
            GaugeRowText(D, "STATUS", state == PageText.PlanOld ? PageText.PlanOldPlain : state, old || fine ? ConsoleStyle.TextPrimary : CvxWarn);
        }

        AscentPlan plan = _s.AscentPlan;
        AscentSolution sol = plan?.Solution;
        if (sol == null || sol.Nodes == 0)
            return;

        GaugeRowText(D, "SOLVE", PageText.PlanSolve(sol.Message, sol.Iterations, sol.Accepted, plan.WallSeconds), sol.Usable ? ConsoleStyle.TextPrimary : CvxWarn);
        if (double.IsFinite(sol.RecoveryReserveKg))
            GaugeRowText(D, "BOOSTER", PageText.PlanBooster(sol.RecoveryReserveKg, sol.BoostbackDv, sol.LandingDv, sol.SeparationTime),
                sol.RecoveryReserveKg >= sol.RecoveryRequiredKg - 50.0 ? ConsoleStyle.TextPrimary : CvxWarn);
        if (plan.LaunchDelayS != 0.0)
            GaugeRowText(D, "LAUNCH TIME", PageText.PlanLaunchShift(plan.LaunchDelayS));
        else if (plan.LanChosen)
            GaugeRowText(D, "LAN", PageText.PlanLanChosen(plan.LanDeg));
        if (plan.StagesPlanned != plan.StagesAvailable)
            GaugeRowText(D, "STAGES PLANNED", $"{plan.StagesPlanned} of {plan.StagesAvailable} (the rest ride as payload)");
        double propLeft = sol.FinalMass - plan.FinalMassFloor;
        GaugeRowText(D, "TO ORBIT", FormattableString.Invariant($"{sol.FinalMass / 1000.0:F2} t ({Math.Max(propLeft, 0.0) / 1000.0:F2} t propellant left in the last stage)"));
        DrawPlanBurnsRow(sol);
        GaugeRowText(D, "", "The stage line counts all the propellant on board, and the plan leaves some.", CvxDim);
        double lowest = plan.Profile != null ? 100.0 * plan.Profile.MinThrottle : double.NaN;
        GaugeRowText(D, "THROTTLE", FormattableString.Invariant($"floor {plan.ThrottleMinPct:F0} %, lowest planned {lowest:F0} %")
            + (plan.SolidStages > 0 ? $", solids burn in {PageText.Stages(plan.SolidStages)} on their own curve" : "")
            + (plan.CoreThrottledDown ? ", core throttled to outlast its boosters" : ""));

        int qi = ArgMax(sol.DynamicPressure), qai = ArgMax(sol.QAlpha);
        GaugeRowText(D, "MAX Q", FormattableString.Invariant($"{sol.DynamicPressure[qi] / 1000.0:F1} kPa at {sol.Time[qi]:F0} s (limit {plan.QMaxKpa:F1} kPa)"),
            LimitInk(sol.DynamicPressure[qi], plan.QMaxKpa * 1000.0));
        if (plan.QRelaxed)
            GaugeRowText(D, "", FormattableString.Invariant($"did not converge at {plan.QMaxRequestedKpa:F0} kPa (out of reach at full throttle): retried looser"), CvxWarn);
        GaugeRowText(D, "MAX Q-ALPHA", FormattableString.Invariant($"{sol.QAlpha[qai]:F0} Pa rad at {sol.Time[qai]:F0} s (limit {plan.QAlphaMax:F0} Pa rad)"),
            LimitInk(sol.QAlpha[qai], plan.QAlphaMax));
        GaugeRowText(D, "PLAN INSERTION", FormattableString.Invariant($"{plan.InsertionAltKm:F0} km, miss {Fmt.Fixed(sol.TerminalResidual[0], 0)} m, {Fmt.Fixed(sol.TerminalResidual[1], 2)} m/s, plane {Fmt.Fixed(sol.TerminalResidual[3], 0)} m"));

        if (_s.Running && _s.FlyingPlan != null)
            GaugeRowText(D, "FLIGHT", _s.Phase == AscentPhase.Profile
                ? FormattableString.Invariant($"flying the profile to {_s.ConvexHandoverAltKm:F0} km")
                : "handed over to UPFG", CvxLive);
    }

    private static float4 LimitInk(double value, double limit)
        => value > limit * 1.002 ? CvxWarn : TuneRules.NearLimit(value, limit) ? Ui.Canvas.Palette.WarnDim.F4 : ConsoleStyle.TextPrimary;

    private static void DrawPlanBurnsRow(AscentSolution sol)
    {
        var times = new string[sol.BurnTime.Length];
        var hues = new Rgb8[sol.BurnTime.Length];
        for (int i = 0; i < times.Length; i++)
        {
            times[i] = PageText.PlanBurn(sol.BurnTime[i]);
            hues[i] = ColorRules.Stage(i);
        }
        ConsoleUi.StageValues("PLAN BURNS", times, hues, PageText.PlanIdeal(sol.TotalDeltaV));
    }

    private static int ArgMax(double[] v)
    {
        int best = 0;
        for (int i = 1; i < v.Length; i++)
            if (v[i] > v[best]) best = i;
        return best;
    }
}

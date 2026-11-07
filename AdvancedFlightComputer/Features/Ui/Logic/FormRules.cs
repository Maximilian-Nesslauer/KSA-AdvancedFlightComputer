using System;
using AdvancedFlightComputer.Features.MultiPass;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal enum LaunchWindowRow
{
    None,

    NodeChoice,

    Countdown,

    NoCrossing,
}

internal static class FormRules
{
    public const double PassesMatterMs = 1.0;

    public static bool PassesCanMatter(double dvMs, double burnS, double periodS) => PassesSaving(dvMs, burnS, periodS) >= PassesMatterMs;

    /// <summary>What a split into two passes saves of finite burn loss, in m/s, or NaN while the plan knows no burn time or no period, such as a craft with no engine data or an escape path.</summary>
    public static double PassesSaving(double dvMs, double burnS, double periodS)
    {
        if (!(dvMs > 0.0) || !(burnS > 0.0) || !(periodS > 0.0) || !double.IsFinite(periodS))
            return double.NaN;
        double ratio = burnS / periodS;
        return dvMs * (MultiPassLoss.FiniteBurnLossFraction(ratio) - MultiPassLoss.FiniteBurnLossFraction(ratio / 2.0));
    }

    /// <summary>The ASCENT row shows while any method can fly, also with only one, so the pilot sees which method climbs and that it is a choice.</summary>
    public static bool AscentRowShows(int availableMethods) => availableMethods >= 1;

    public const double SiteNearM = 20000.0;

    public static bool SiteOutOfReach(double siteM, double impactS, double surfaceSpeedMs)
        => double.IsFinite(siteM) && double.IsFinite(impactS) && impactS >= 0.0 && double.IsFinite(surfaceSpeedMs)
            && siteM > Math.Max(SiteNearM, 2.0 * Math.Abs(surfaceSpeedMs) * impactS);

    public static bool MarkOwnRow(float lastRowWidth, float markWidth, float rowWidth, float gap, bool longMark)
        => markWidth > 0f && lastRowWidth > 0f && (longMark || lastRowWidth + gap + markWidth > rowWidth);

    public static bool FlightNoticeShows(CraftSituation where, LandWay way) => where switch
    {
        CraftSituation.Ground or CraftSituation.Escaping => false,
        CraftSituation.Orbit => way == LandWay.FromOrbit,
        _ => true,
    };

    public const double LegsNoteHeightM = 20000.0;

    public const double LegsNoteImpactS = 300.0;

    public const double LegsUrgentS = 60.0;

    public static bool LegsUrgent(double impactS) => double.IsFinite(impactS) && impactS >= 0.0 && impactS < LegsUrgentS;

    public static bool LegsNoteShows(CraftSituation where, double heightM, double impactS, bool landingFlies)
    {
        bool down = SituationText.IsFalling(where) || (landingFlies && where != CraftSituation.Ground);
        return down && (heightM < LegsNoteHeightM || (double.IsFinite(impactS) && impactS >= 0.0 && impactS < LegsNoteImpactS));
    }

    public static bool PlanForShows(int craftInFrame, bool plansForOther) => craftInFrame > 1 || plansForOther;

    /// <summary>Without a target the node is only the first heading of the pitch program (GuidanceWindow.TurnDir), and a calculated climb does not read it, so the choice shows only while the pitch program would fly.</summary>
    public static LaunchWindowRow WindowRow(bool hasTarget, bool windowKnown, bool planeUnreachable, bool pitchProgramFlies)
    {
        if (hasTarget)
            return windowKnown ? LaunchWindowRow.Countdown
                : planeUnreachable ? LaunchWindowRow.NoCrossing
                : LaunchWindowRow.None;
        return pitchProgramFlies ? LaunchWindowRow.NodeChoice : LaunchWindowRow.None;
    }

    public static int MapChoice(bool overlayOn, bool planOn) => !overlayOn ? 0 : planOn ? 2 : 1;

    public static (bool OverlayOn, bool PlanOn) ApplyMapChoice(int picked, bool planOn) => picked switch
    {
        0 => (false, planOn),
        1 => (true, false),
        _ => (true, true),
    };
}

internal readonly record struct FlownAscent(string Body, double PeKm, double ApKm, double GoalPeKm, double GoalApKm)
{
    public string Line(in SituationFacts now)
        => now.Body == Body && now.Where is CraftSituation.Orbit or CraftSituation.CoastingUp && double.IsFinite(PeKm) && double.IsFinite(ApKm)
            ? PageText.FlownAgainstGoal(PeKm, ApKm, GoalPeKm, GoalApKm)
            : "";
}

internal static class LandSite
{
    public const string DefaultBody = "Luna";

    public static bool NonePicked(double latDeg, double lonDeg, string body)
        => latDeg == GuidanceSettingSpecs.SiteLatDeg.Default && lonDeg == GuidanceSettingSpecs.SiteLonDeg.Default && body != DefaultBody;
}

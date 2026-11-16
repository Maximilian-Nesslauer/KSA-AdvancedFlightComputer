using System;
using System.Globalization;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal enum LegsStep
{
    /// <summary>Nothing to do on this step.</summary>
    Wait,

    /// <summary>Command every landing leg to its deployed end now, and remember that this landing did it.</summary>
    Deploy,

    /// <summary>No landing flies the craft any more, so the next landing may deploy the legs again.</summary>
    Rearm,
}

/// <summary>The clock a landing reads the time to the touchdown from, for the leg deploy.</summary>
internal enum LegsClock
{
    /// <summary>No time to go was read.</summary>
    None,

    /// <summary>The end of the plan of touchdown guidance (6-DOF), which is where the craft meets the ground.</summary>
    TouchdownPlan,

    /// <summary>The end of the glide-and-burn plan of entry guidance (3-DOF).</summary>
    EntryPlan,

    /// <summary>The planned arrival of the G-FOLD descent, on the ground or at the gate of the vertical approach.</summary>
    GfoldPlan,

    /// <summary>The end of the plan of touchdown guidance on standby, while another mode flies the burn it will take over.</summary>
    TouchdownStandby,

    /// <summary>The predicted impact of a glide that has no plan yet.</summary>
    Impact,

    /// <summary>No plan gives a time: the fall of the legs to the ground at the descent rate, with gravity.</summary>
    Fall,
}

/// <summary>The times to the touchdown, s, that each clock of a landing gives now, NaN for a clock that gives none.</summary>
internal readonly record struct LegsClocks(double TouchdownPlanS, double EntryPlanS, double GfoldPlanS, double StandbyPlanS,
                                           double ImpactS, double FallS);

/// <summary>
/// When a landing deploys the landing legs by itself: once per landing, while guidance holds the craft for a landing, when the time to the touchdown comes down to the time the legs take to open plus the buffer the player set, and never on the ground. The time to the touchdown is the one of the plan that flies the landing, and without a plan the fall time of the legs to the ground. A landing that deployed them, or found the player had done it, does nothing more, so a retract or a deploy by hand stands.
/// </summary>
internal static class LegsRules
{
    public static LegsStep Decide(bool enabled, bool landingFlies, bool holdsCraft, bool firedThisLanding,
                                  bool groundContact, double toGoS, double deployAtS)
    {
        if (!landingFlies || !holdsCraft)
            return firedThisLanding && !landingFlies && !holdsCraft ? LegsStep.Rearm : LegsStep.Wait;
        if (firedThisLanding || !enabled || groundContact)
            return LegsStep.Wait;
        if (!double.IsFinite(toGoS) || !double.IsFinite(deployAtS) || toGoS > deployAtS)
            return LegsStep.Wait;
        return LegsStep.Deploy;
    }

    /// <summary>The time to the touchdown at which a landing deploys the legs, s: the time the legs take to open plus the buffer.</summary>
    public static double DeployAtS(double legsDeployS, double bufferS)
        => Math.Max(0.0, double.IsFinite(legsDeployS) ? legsDeployS : 0.0) + Math.Max(0.0, double.IsFinite(bufferS) ? bufferS : 0.0);

    /// <summary>The time from <paramref name="nowS"/> to the end <paramref name="endS"/> of a plan, or NaN when the plan gives no end ahead of now.</summary>
    public static double PlanToGo(double endS, double nowS)
        => double.IsFinite(endS) && double.IsFinite(nowS) && endS > nowS ? endS - nowS : double.NaN;

    /// <summary>
    /// The time the legs take to fall <paramref name="heightM"/> to the ground from a descent rate of <paramref name="sinkMs"/> (positive down) under <paramref name="gravity"/> with the engine off. Thrust and drag only slow a fall, so a landing that flies no plan reaches the ground no sooner than this. With no gravity it is the height divided by the descent rate, and a craft that does not descend then has no time to go.
    /// </summary>
    public static double FallSeconds(double heightM, double sinkMs, double gravity)
    {
        if (!double.IsFinite(heightM) || !double.IsFinite(sinkMs))
            return double.NaN;
        if (heightM <= 0.0)
            return 0.0;
        double g = double.IsFinite(gravity) && gravity > 0.0 ? gravity : 0.0;
        // h = v t + g t^2 / 2 solved for t, in the form that stays exact for a small g.
        double denominator = Math.Sqrt(sinkMs * sinkMs + 2.0 * g * heightM) + sinkMs;
        return denominator > 0.0 ? 2.0 * heightM / denominator : double.PositiveInfinity;
    }

    /// <summary>
    /// The time to the touchdown of a landing, s, and through <paramref name="clock"/> where it comes from: the first clock of <paramref name="c"/> that gives a time, in the order the modes take the craft over (touchdown guidance flies after entry guidance and G-FOLD, and a standby plan or a glide only while nothing else plans), and the fall time when no plan gives one. NaN with the clock None when nothing does.
    /// </summary>
    public static double ToGo(in LegsClocks c, out LegsClock clock)
    {
        clock = double.IsFinite(c.TouchdownPlanS) ? LegsClock.TouchdownPlan
            : double.IsFinite(c.EntryPlanS) ? LegsClock.EntryPlan
            : double.IsFinite(c.GfoldPlanS) ? LegsClock.GfoldPlan
            : double.IsFinite(c.StandbyPlanS) ? LegsClock.TouchdownStandby
            : double.IsFinite(c.ImpactS) ? LegsClock.Impact
            : double.IsFinite(c.FallS) ? LegsClock.Fall
            : LegsClock.None;
        return clock switch
        {
            LegsClock.TouchdownPlan => c.TouchdownPlanS,
            LegsClock.EntryPlan => c.EntryPlanS,
            LegsClock.GfoldPlan => c.GfoldPlanS,
            LegsClock.TouchdownStandby => c.StandbyPlanS,
            LegsClock.Impact => c.ImpactS,
            LegsClock.Fall => c.FallS,
            _ => double.NaN,
        };
    }

    public static string ClockName(LegsClock clock) => clock switch
    {
        LegsClock.TouchdownPlan => "touchdown plan",
        LegsClock.EntryPlan => "entry plan",
        LegsClock.GfoldPlan => "G-FOLD plan",
        LegsClock.TouchdownStandby => "touchdown plan on standby",
        LegsClock.Impact => "predicted impact",
        LegsClock.Fall => "fall at the descent rate",
        _ => "no clock",
    };

    /// <summary>The value of the LEGS fold: what a landing does with the legs of this craft.</summary>
    public static string Summary(bool enabled, double deployAtS, int legs)
        => legs <= 0 ? "NONE ON CRAFT"
            : !enabled ? "OFF"
            : "AUTO AT " + ToGo(deployAtS);

    public static string Label(bool enabled, double deployAtS, int legs) => "LEGS: " + Summary(enabled, deployAtS, legs);

    /// <summary>What the landing that flies did with the legs, or empty before it did anything.</summary>
    public static string Done(bool fired, double firedToGoS, double firedAtM, int commanded)
    {
        if (!fired)
            return "";
        if (!double.IsFinite(firedToGoS))
            return "DEPLOYED BY THIS LANDING.";
        string at = ToGo(firedToGoS) + (double.IsFinite(firedAtM) ? ", " + Height(firedAtM) : "");
        return commanded > 0
            ? "DEPLOYED AT " + at + "."
            : "ALREADY DOWN AT " + at + ".";
    }

    /// <summary>A time to the touchdown as the LEGS fold and the legs note show it.</summary>
    public static string ToGo(double s) => s.ToString("F1", CultureInfo.InvariantCulture) + " s TO GO";

    private static string Height(double m) => Math.Round(m).ToString("F0", CultureInfo.InvariantCulture) + " m";
}

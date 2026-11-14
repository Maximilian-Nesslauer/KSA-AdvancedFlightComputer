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

/// <summary>
/// When a landing deploys the landing legs by itself: once per landing, while guidance holds the craft for a landing, below the height the player set for the legs over the ground under the craft, and never on the ground. A landing that deployed them, or found the player had done it, does nothing more, so a retract or a deploy by hand stands.
/// </summary>
internal static class LegsRules
{
    public static LegsStep Decide(bool enabled, bool landingFlies, bool holdsCraft, bool firedThisLanding,
                                  bool groundContact, double legsHeightM, double deployHeightM)
    {
        if (!landingFlies || !holdsCraft)
            return firedThisLanding && !landingFlies && !holdsCraft ? LegsStep.Rearm : LegsStep.Wait;
        if (firedThisLanding || !enabled || groundContact)
            return LegsStep.Wait;
        if (!double.IsFinite(legsHeightM) || !double.IsFinite(deployHeightM) || legsHeightM > deployHeightM)
            return LegsStep.Wait;
        return LegsStep.Deploy;
    }

    /// <summary>The value of the LEGS fold: what a landing does with the legs of this craft.</summary>
    public static string Summary(bool enabled, double deployHeightM, int legs)
        => legs <= 0 ? "NONE ON CRAFT"
            : !enabled ? "OFF"
            : "AUTO AT " + Height(deployHeightM);

    public static string Label(bool enabled, double deployHeightM, int legs) => "LEGS: " + Summary(enabled, deployHeightM, legs);

    /// <summary>What the landing that flies did with the legs, or empty before it did anything.</summary>
    public static string Done(bool fired, double firedAtM, int commanded)
    {
        if (!fired)
            return "";
        if (!double.IsFinite(firedAtM))
            return "DEPLOYED BY THIS LANDING.";
        return commanded > 0
            ? "DEPLOYED AT " + Height(firedAtM) + "."
            : "ALREADY DOWN AT " + Height(firedAtM) + ".";
    }

    private static string Height(double m) => Math.Round(m).ToString("F0", CultureInfo.InvariantCulture) + " m";
}

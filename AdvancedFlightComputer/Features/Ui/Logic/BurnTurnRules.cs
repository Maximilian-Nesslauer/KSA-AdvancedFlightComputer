using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal enum PointingWarning
{
    None,
    RcsOff,
    NoRcs,
    CannotTurn,
    StallBand,
    ShortBurn,
}

/// <summary>What the pointing check found for the next node, with the numbers its text names.</summary>
internal readonly record struct PointingCheck(PointingWarning Warning, double StallEdgeRadians = 0.0, double BurnSeconds = 0.0, double MinimumSeconds = 0.0)
{
    public static readonly PointingCheck None = new(PointingWarning.None);

    public string Caution => BurnTurnRules.Caution(in this);

    public string Card => BurnTurnRules.Card(in this);
}

/// <summary>
/// The rules of the warp to the turn of a node and of the pointing check before stock AUTO flies a node.
/// The game values come in as numbers, so these rules hold no game types.
/// </summary>
/// <remarks>
/// FlightComputer.ComputeRcsTrackAxis adds an impulse on an axis only when its rate crosses
/// EvaluateUpperSwitchingLine or EvaluateLowerSwitchingLine. For an error between 0.5 * AngleDeadband and
/// 0.5 * AngleDeadband + AngleTurnaround, the upper line is zero and the lower line is negative, so a craft at rest
/// there never starts to turn, while FlightComputer.ComputeControl holds the ignition of an Auto burn until the pitch
/// and yaw errors are inside AngleDeadband. A burn attitude is inertially fixed, so the rate deadband equals RateBit,
/// and FlightComputer.UpdateRcsParams makes AngleTurnaround the larger of RateBit^2 / RcsTorqueAuthority and
/// 10 * RateBit. Without an RCS pulse on an axis, UpdateRcsParams makes RateBit infinite, and ComputeControl then
/// lights the engine on time without the hold, pointed wherever the craft points.
/// </remarks>
internal static class BurnTurnRules
{
    /// <summary>The warp to the turn ends this long before stock starts to turn the craft, so the turn runs at 1x.</summary>
    public const double WarpLeadSeconds = 30.0;

    /// <summary>A warp shorter than this is not offered.</summary>
    public const double WarpMinimumSeconds = 60.0;

    // A craft rarely comes to rest just outside the deadband, so a stall band narrower than this is left out.
    public const double StallBandWarnRadians = 5.0 * Math.PI / 180.0;

    private const double TurnaroundRateDeadbands = 10.0;

    public const string WarpVerb = "WARP TO TURN";

    public const string StopWarpVerb = "STOP WARP";

    public const string WarpTip = "Warps to 30 s before STOCK AUTO starts to turn the craft, so the turn and the burn run at 1x. STOCK AUTO stays on. The stock WARP runs to the ignition and turns the craft under warp.";

    public const string StopWarpTip = "Stops the warp to the turn. STOCK AUTO stays on. A warp started elsewhere is not stopped here.";

    /// <summary>Whether a warp from now to the turn is long enough to offer, with <paramref name="secondsToTurn"/> to the alignment start of the node.</summary>
    public static bool WarpWorthIt(double secondsToTurn) => secondsToTurn > WarpLeadSeconds + WarpMinimumSeconds;

    /// <summary>The largest error, in radians, at which stock leaves a craft at rest unturned on one axis. Infinite without RCS authority on that axis.</summary>
    public static double StallEdge(double deadband, double rateBit, double authority)
    {
        if (!(rateBit > 0) || !double.IsFinite(rateBit) || !(authority > 0) || !double.IsFinite(authority))
            return double.PositiveInfinity;
        double turnaround = Math.Max(rateBit * rateBit / authority, TurnaroundRateDeadbands * rateBit);
        return 0.5 * deadband + turnaround;
    }

    /// <summary>
    /// The shortest burn at <paramref name="throttle"/> that FlightComputer.ComputeMinimumDurationThrottleCap accepts:
    /// twice the time the gimbal loop needs to settle, and the slew through <paramref name="pointingError"/> at half the
    /// gimbal authority, which scales with the throttle. Its cap is the throttle at which the burn is exactly this long.
    /// </summary>
    public static double MinimumBurnSeconds(double gimbalAuthority, double pointingError, double settleSeconds, double throttle = 1.0)
    {
        double slew = gimbalAuthority > 0 && pointingError > 0 && throttle > 0
            ? 2.0 * Math.Sqrt(pointingError / (0.5 * gimbalAuthority * throttle)) : 0.0;
        return slew + settleSeconds / 0.5;
    }

    /// <summary>
    /// The lowest throttle stock flies the node with: its planned throttle, or else the least throttle of the engines,
    /// because FlightComputer.SolveBurnThrottle lowers the throttle of an unplanned burn down to that.
    /// </summary>
    public static double LowestThrottle(float? plannedThrottle, float minThrottle)
    {
        float least = Math.Clamp(minThrottle, 0f, 1f);
        return Math.Clamp(plannedThrottle ?? least, least, 1f);
    }

    /// <summary>The burn length at <paramref name="throttle"/>, with the mass flow taken as linear in the throttle.</summary>
    public static double FlownSeconds(double fullThrottleSeconds, double throttle)
        => throttle > 0 ? fullThrottleSeconds / throttle : double.PositiveInfinity;

    /// <summary>
    /// The first problem of a node that stock AUTO will fly. The RCS checks apply only while RCS holds pitch and yaw,
    /// because a lit engine steers with its gimbal and stock does not need RCS then.
    /// </summary>
    public static PointingCheck Classify(bool gimbalSteers, bool rcsOnPitchAndYaw, bool rcsSwitchedOff, double deadband, double stallEdge,
        double burnSeconds, double minimumSeconds)
    {
        if (!gimbalSteers)
        {
            if (!rcsOnPitchAndYaw)
                return new PointingCheck(rcsSwitchedOff ? PointingWarning.RcsOff : PointingWarning.NoRcs);
            if (stallEdge >= Math.PI)
                return new PointingCheck(PointingWarning.CannotTurn, stallEdge);
            if (stallEdge > deadband && stallEdge >= StallBandWarnRadians)
                return new PointingCheck(PointingWarning.StallBand, stallEdge);
        }
        if (burnSeconds > 0 && double.IsFinite(minimumSeconds) && burnSeconds < minimumSeconds)
            return new PointingCheck(PointingWarning.ShortBurn, 0.0, burnSeconds, minimumSeconds);
        return PointingCheck.None;
    }

    public static string Caution(in PointingCheck c) => c.Warning switch
    {
        PointingWarning.RcsOff => "RCS OFF: CRAFT DOES NOT TURN",
        PointingWarning.NoRcs => "NO RCS FOR PITCH AND YAW",
        PointingWarning.CannotTurn => "RCS TOO STRONG TO AIM",
        PointingWarning.StallBand => string.Create(Fmt.Inv, $"MAY NOT TURN FROM {Degrees(c.StallEdgeRadians):F0} DEG OFF"),
        PointingWarning.ShortBurn => string.Create(Fmt.Inv, $"BURN {c.BurnSeconds:F1} s, AIM NEEDS {c.MinimumSeconds:F1} s"),
        _ => "",
    };

    public static string Card(in PointingCheck c) => c.Warning switch
    {
        PointingWarning.RcsOff => "STOCK AUTO turns the craft to the burn with RCS before the engine lights. With RCS off the craft does not turn, and the engine lights on time pointed wherever the craft points. Turn RCS on.",
        PointingWarning.NoRcs => "STOCK AUTO turns the craft to the burn with RCS before the engine lights. No thruster with propellant turns this craft in pitch and yaw, so the engine lights pointed wherever the craft points, or the burn waits.",
        PointingWarning.CannotTurn => "One RCS pulse turns this light craft so fast that STOCK AUTO does not start the turn from rest, and the burn waits. Smaller thrusters or more mass fix it.",
        PointingWarning.StallBand => string.Create(Fmt.Inv, $"From rest less than {Degrees(c.StallEdgeRadians):F0} deg off the burn attitude, STOCK AUTO does not fire the RCS of this craft, and the burn waits until the craft drifts out. Point the craft farther away before the turn starts."),
        PointingWarning.ShortBurn => string.Create(Fmt.Inv, $"The engine needs about {c.MinimumSeconds:F1} s to settle its aim, and STOCK AUTO cannot stretch this burn further. The burn ends with an error in direction, so check the orbit after it."),
        _ => "",
    };

    private static double Degrees(double radians) => radians * (180.0 / Math.PI);
}

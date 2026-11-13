using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>
/// The metadata of every guidance input: who it is for, its unit, range, default and help text.
/// Defaults are the field initializers of VehicleAutopilotState, which a harness test checks.
/// The tier arrays let a page count and reset the values it shows.
/// </summary>
internal static class GuidanceSettingSpecs
{
    private const UiTier P = UiTier.Player;
    private const UiTier E = UiTier.Expert;
    private const UiTier T = UiTier.Tuning;

    #region Ascent

    public static readonly SettingSpec PeKm = new("PeKm", "PERIAPSIS", "km", P, 200.0, 0.0, 1e7, "%.1f",
        "Periapsis altitude of the target orbit. A value above APOAPSIS swaps the two.");
    public static readonly SettingSpec ApKm = new("ApKm", "APOAPSIS", "km", P, 420.0, 0.0, 1e7, "%.1f",
        "Apoapsis altitude of the target orbit.");
    public static readonly SettingSpec IncDeg = new("IncDeg", "INCLINATION", "deg", P, 51.6, 0.0, 180.0, "%.1f",
        "Orbit inclination to the equator. A new value resets LAN over the launch site.");
    public static readonly SettingSpec LanDeg = new("LanDeg", "LAN", "deg", P, 250.0, -360.0, 360.0, "%.1f",
        "Longitude of the ascending node. Starts over the launch site, and no reset changes it.");
    public static readonly SettingSpec ArgPeDeg = new("ArgPeDeg", "ARG. OF PE", "deg", E, 0.0, -360.0, 360.0, "%.1f",
        "Argument of periapsis. Used only with FIX ARG. OF PE on.");
    public static readonly SettingSpec ChaseOffsetKm = new("ChaseOffsetKm", "SMA OFFSET", "km", E, 20.0, -1e5, 1e5, "%.1f",
        "Semi-major axis of the chase orbit below the target's.");
    public static readonly SettingSpec MoonTransferPeKm = new("MoonTransferPeKm", "PERIAPSIS AT MOON", "km", P, 100.0, 10.0, 1e6, "%.1f",
        "THEN: the periapsis altitude at the moon that the injection and its corrections aim for. The capture burn makes its orbit there.");
    public static readonly SettingSpec GLimitG = new("GLimitG", "MAX ACCELERATION", "g", P, 4.0, 0.11, 100.0, "%.2f",
        "Acceleration limit of the whole flight with G-LIMIT on. A landing refuses to start at 0.1 g or less.");
    public static readonly SettingSpec TurnStartAltKm = new("TurnStartAltKm", "TURN START ALTITUDE", "km", E, 0.5, 0.0, 1000.0, "%.1f",
        "GRAVITY TURN: altitude where the turn starts.");
    public static readonly SettingSpec TurnRateDegS = new("TurnRateDegS", "TURN RATE", "deg/s", E, 1.0, 0.0, 90.0, "%.2f",
        "GRAVITY TURN: pitch rate of the turn.");
    public static readonly SettingSpec ForceRollDeg = new("ForceRollDeg", "ROLL ANGLE", "deg", E, 180.0, -360.0, 360.0, "%.1f",
        "Roll angle held with FORCE ROLL on.");
    public static readonly SettingSpec ConvexQMaxKpa = new("ConvexQMaxKpa", "MAX Q", "kPa", P, 195.0, 0.0, 1e4, "%.1f",
        "Dynamic pressure limit of the solved climb.");
    public static readonly SettingSpec ConvexThrottleMinPct = new("ConvexThrottleMinPct", "MIN THROTTLE", "%", E, 80.0, 0.0, 100.0, "%.0f",
        "Lowest throttle of the solved climb on a liquid stage. Solid stages stay at full thrust.");
    public static readonly SettingSpec ConvexQAlphaMax = new("ConvexQAlphaMax", "MAX Q-ALPHA", "Pa rad", T, 3500.0, 0.0, 1e7, "%.1f",
        "Limit of dynamic pressure times angle of attack for the solved climb.");
    public static readonly SettingSpec ConvexHandoverAltKm = new("ConvexHandoverAltKm", "UPFG FROM", "km", T, 80.0, 0.0, 1e4, "%.1f",
        "Altitude of the handover from the solved climb to UPFG.");

    public static readonly BoolSpec GLimitEnabled = new("GLimitEnabled", "G-LIMIT", P, false,
        "Caps acceleration of the whole ascent at MAX ACCELERATION.");
    public static readonly BoolSpec AutoLaunch = new("AutoLaunch", "AUTO WARP TO WINDOW", P, true,
        "Offers a time warp to the launch window when the launch is armed.");
    public static readonly BoolSpec MatchTargetArgPe = new("MatchTargetArgPe", "COPY TARGET ARG. PE", E, true,
        "Copies the target's argument of periapsis into the chase orbit.");
    public static readonly BoolSpec ArgPeFixed = new("ArgPeFixed", "FIX ARG. OF PE", E, false,
        "Holds periapsis at ARG. OF PE. The insertion moves along the ellipse.");
    public static readonly BoolSpec OptimiseInsertion = new("OptimiseInsertion", "OPTIMIZE INSERTION", E, true,
        "Moves the insertion away from periapsis where that saves delta-v.");
    public static readonly BoolSpec InsertBeforeApoapsis = new("InsertBeforeApoapsis", "INSERT BEFORE APOAPSIS", E, false,
        "Searches for the insertion before apoapsis, not after periapsis.");
    public static readonly BoolSpec FlyConvexAscent = new("FlyConvexAscent", "SOLVED CLIMB", P, true,
        "Flies the solved climb. Off flies the vertical rise and the gravity turn.");
    public static readonly BoolSpec ConvexOptimiseTiming = new("ConvexOptimiseTiming", "SOLVE LIFTOFF", E, true,
        "Sets the liftoff time (with a target, up to 10 minutes from the launch window) or the LAN (no target) for the most mass to orbit. LAUNCH arms for that time and LAN is set to it. Off launches at the window, or now, into the LAN on the page.");
    public static readonly BoolSpec ShowConvexPlan = new("ShowConvexPlan", "PLAN ON MAP", E, true,
        "Draws the solved plan in the map view with SHOW IN MAP on. The FLIGHT PROFILE picture always shows it.");
    public static readonly BoolSpec ForceRoll = new("ForceRoll", "FORCE ROLL", E, true,
        "Holds the ROLL ANGLE during the ascent.");
    public static readonly BoolSpec LaunchDescending = new("LaunchDescending", "DESC NODE", P, false,
        "Launches at the descending node with no target.");

    #endregion

    #region Craft switches

    public static readonly BoolSpec Engage = new("Engage", "AFC MAY STEER", P, true,
        "Lets AFC command attitude. Off ends a mode that holds it.");
    public static readonly BoolSpec AutoStage = new("AutoStage", "AFC MAY USE ENGINES", P, true,
        "Sets whether AFC lights the engines, throttles and stages.");

    #endregion

    #region Boostback

    public static readonly SettingSpec BoostbackSeparationS = new("BoostbackSeparationS", "SETTLING BURN", "s", E, 0.2, 0.0, 60.0, "%.1f",
        "Duration of the settling burn after separation, before the turn.");
    public static readonly SettingSpec BoostbackPitchDeg = new("BoostbackPitchDeg", "MIN PITCH", "deg", E, 0.0, -180.0, 90.0, "%.1f",
        "Lowest pitch of the burn against the horizon. A very negative value switches the shaping off.");
    public static readonly SettingSpec GlideMaxAoaDeg = new("GlideMaxAoaDeg", "GLIDE MAX AoA", "deg", E, 15.0, 0.0, 60.0, "%.1f",
        "Largest angle of attack (AoA) the glide may fly to steer the impact point.");
    public static readonly SettingSpec GlideNominalAoaDeg = new("GlideNominalAoaDeg", "GLIDE PLAN AoA", "deg", E, 7.5, 0.0, 60.0, "%.1f",
        "Angle of attack that every impact prediction assumes for the glide, with the lift pointed up the trajectory. 0 plans an engine-first ballistic coast.");
    public static readonly SettingSpec BoostbackClearanceM = new("BoostbackClearanceM", "CLEARANCE AT SEPARATION", "m", E, 30.0, 0.0, 1e4, "%.1f",
        "Distance the booster coasts, engine off, away from the stage before the settling burn lights.");
    public static readonly SettingSpec BoostbackSlewDegS = new("BoostbackSlewDegS", "SLEW RATE", "deg/s", T, 30.0, 0.0, 360.0, "%.2f",
        "Rotation rate of the turn after separation.");
    public static readonly SettingSpec GlideKp = new("GlideKp", "GLIDE KP", "", T, 15.0, 0.0, 1000.0, "%.3f",
        "Proportional gain of the glide on the miss distance.");
    public static readonly SettingSpec GlideKi = new("GlideKi", "GLIDE KI", "1/s", T, 0.0, 0.0, 1000.0, "%.3f",
        "Integral gain of the glide on the miss distance.");
    public static readonly SettingSpec GlideKd = new("GlideKd", "GLIDE KD", "s", T, 0.0, 0.0, 1000.0, "%.3f",
        "Derivative gain of the glide on the miss distance.");

    #endregion

    #region Landing

    public static readonly SettingSpec SiteLatDeg = new("SiteLatDeg", "LATITUDE", "deg", P, 0.67408, -90.0, 90.0, "%.6f",
        "Latitude of the landing site.");
    public static readonly SettingSpec SiteLonDeg = new("SiteLonDeg", "LONGITUDE", "deg", P, 23.47297, -360.0, 360.0, "%.6f",
        "Longitude of the landing site.");
    public static readonly SettingSpec BrakingAltitudeKm = new("BrakingAltitudeKm", "BRAKING START ALTITUDE", "km", E, 11.0, 0.0, 1e4, "%.1f",
        "Height above the landing site where the braking burn starts.");
    public static readonly SettingSpec DeorbitArrivalDescentDeg = new("DeorbitArrivalDescentDeg", "ARRIVAL ANGLE DOWN", "deg", E, 5.0, 0.0, 30.0, "%.1f",
        "Flight path angle at the braking point while SET DEORBIT ARRIVAL ANGLE is on.");
    public static readonly SettingSpec DescentRate = new("DescentRate", "DESCENT RATE", "m/s", E, 20.0, 0.0, 1e4, "%.1f",
        "Sink rate at the gate.");
    public static readonly SettingSpec LandingVerticalGateM = new("LandingVerticalGateM", "VERTICAL GATE", "m", E, 500.0, 0.0, 1e6, "%.1f",
        "Height above the site where the final vertical approach starts. DEORBIT + LAND with G-FOLD needs 100 m or more.");
    public static readonly SettingSpec DownrangeFactor = new("DownrangeFactor", "DOWNRANGE FACTOR", "", T, 1.2, 0.0, 100.0, "%.3f",
        "The braking burn lights at this multiple of the predicted braking distance.");
    public static readonly SettingSpec AimAltKm = new("AimAltKm", "AIM ALTITUDE", "km", T, 0.1, 0.0, 1e4, "%.2f",
        "Altitude of the high gate above the site.");
    public static readonly SettingSpec GateUprangeKm = new("GateUprangeKm", "GATE UPRANGE", "km", T, 0.0, -1e4, 1e4, "%.1f",
        "Offset of the gate against the approach direction.");
    public static readonly SettingSpec GfoldHandoffTgo = new("GfoldHandoffTgo", "DESCENT FROM", "s", T, 40.0, 0.0, 1e4, "%.1f",
        "Time to the gate at which the braking burn hands over to the powered descent.");
    public static readonly SettingSpec VehicleHeightM = new("VehicleHeightM", "COM ABOVE LEGS", "m", P, 15.0, 0.0, 1e4, "%.1f",
        "Height of the center of mass (COM) above the legs, the reference for on the ground in every landing. Measured while MEASURE COM ABOVE LEGS is on, switch it off to type a value.");
    public static readonly SettingSpec GfoldHoverHandoffAltM = new("GfoldHoverHandoffAltM", "HOVER FROM", "m", P, 10.0, 0.0, 1e5, "%.1f",
        "Height where G-FOLD hands the descent to the hover.");
    public static readonly SettingSpec LandingAccelLimitG = new("LandingAccelLimitG", "MAX BURN ACCEL", "g", E, 3.0, 0.0, 100.0, "%.2f",
        "Largest thrust acceleration in the ENTRY and TOUCHDOWN plans, one value for both. 0 switches the limit off.");
    public static readonly SettingSpec LandingPlanFloorMargin = new("LandingPlanFloorMargin", "PLAN MIN THROTTLE MARGIN", "frac", T, 0.05, 0.0, 0.5, "%.2f",
        "Throttle margin above the real engine minimum in the ENTRY and TOUCHDOWN plans, kept for thrust feedback. One value for both, 0 plans on the minimum itself.");

    public static readonly BoolSpec DeorbitArrivalAngleEnabled = new("DeorbitArrivalAngleEnabled", "SET DEORBIT ARRIVAL ANGLE", E, false,
        "Plans the deorbit for a given flight path angle at the braking point.");
    public static readonly BoolSpec VehicleHeightAuto = new("VehicleHeightAuto", "MEASURE COM ABOVE LEGS", P, true,
        "Measures the COM above the legs from the colliders of the craft, with the legs deployed, and keeps it current.");

    #endregion

    #region G-FOLD

    public static readonly SettingSpec GfoldGlideSlopeDeg = new("GfoldGlideSlopeDeg", "GLIDE SLOPE", "deg", T, 1.0, 0.0, 90.0, "%.1f",
        "Smallest angle above the horizon the descent may approach the site from.");
    public static readonly SettingSpec GfoldPointingDeg = new("GfoldPointingDeg", "MAX TILT", "deg", T, 90.0, 0.0, 180.0, "%.1f",
        "Largest angle of the thrust from vertical.");
    public static readonly SettingSpec GfoldVMaxMs = new("GfoldVMaxMs", "MAX SPEED", "m/s", T, 500.0, 0.0, 1e5, "%.1f",
        "Speed limit of the planned descent.");
    public static readonly SettingSpec GfoldThrottleMin = new("GfoldThrottleMin", "PLAN MIN THROTTLE", "frac", T, 0.05, 0.0, 1.0, "%.2f",
        "Lowest thrust the solver may plan, as a share of full thrust.");
    public static readonly SettingSpec GfoldThrottleMax = new("GfoldThrottleMax", "PLAN MAX THROTTLE", "frac", T, 0.90, 0.0, 1.0, "%.2f",
        "Highest thrust the solver may plan, as a share of full thrust.");
    public static readonly SettingSpec GfoldSlewReg = new("GfoldSlewReg", "CONTROL SMOOTHING", "weight", T, 0.05, 0.0, 100.0, "%.3f",
        "Weight on thrust changes in the plan. 0 switches it off.");
    public static readonly SettingSpec GfoldIntervalS = new("GfoldIntervalS", "RE-SOLVE EVERY", "s", T, 0.25, 0.0, 60.0, "%.2f",
        "Time between two solves during the descent.");
    public static readonly SettingSpec GfoldNodes = new("GfoldNodes", "SOLVER POINTS", "", T, 50.0, 2.0, 1000.0, "%.0f",
        "Number of points of the planned descent.");
    public static readonly SettingSpec GfoldKp = new("GfoldKp", "TRACK KP", "1/s^2", T, 0.08, 0.0, 100.0, "%.3f",
        "Position feedback gain on the plan.");
    public static readonly SettingSpec GfoldKd = new("GfoldKd", "TRACK KD", "1/s", T, 0.30, 0.0, 100.0, "%.3f",
        "Velocity feedback gain on the plan.");
    public static readonly SettingSpec GfoldSmoothTau = new("GfoldSmoothTau", "COMMAND SMOOTHING", "s", T, 0.15, 0.0, 60.0, "%.2f",
        "Time constant of the command low-pass filter.");
    public static readonly SettingSpec GfoldSolveTimeLimitS = new("GfoldSolveTimeLimitS", "SOLVE TIME LIMIT", "s", T, 0.040, 0.0, 60.0, "%.3f",
        "Time limit of one solve. 0 means no limit.");

    #endregion

    #region Touchdown guidance (6-DOF)

    public static readonly SettingSpec SixDofNodes = new("SixDofNodes", "SOLVER POINTS", "", T, 50.0, 5.0, 50.0, "%.0f",
        "Number of points of the touchdown guidance plan.");
    public static readonly SettingSpec SixDofTiltDeg = new("SixDofTiltDeg", "MAX TILT", "deg", T, 120.0, 0.0, 180.0, "%.1f",
        "Largest tilt of the craft from vertical.");
    public static readonly SettingSpec SixDofThrottleFloor = new("SixDofThrottleFloor", "THROTTLE FLOOR", "frac", T, 0.40, 0.0, 1.0, "%.3f",
        "Lowest throttle of the plan while THROTTLE FLOOR FROM CRAFT is off.");
    public static readonly SettingSpec SixDofSigmaSeed = new("SixDofSigmaSeed", "FIRST BURN TIME GUESS", "s", T, 20.0, 0.0, 1e4, "%.1f",
        "First guess of the burn time of the first plan.");
    public static readonly SettingSpec SixDofSigmaSamples = new("SixDofSigmaSamples", "BURN-TIME SAMPLES", "", T, 5.0, 1.0, 12.0, "%.0f",
        "Burn times tried while FIXED BURN TIME is on.");
    public static readonly SettingSpec SixDofGlideSlopeDeg = new("SixDofGlideSlopeDeg", "GLIDE SLOPE", "deg", T, 10.0, 0.0, 90.0, "%.1f",
        "Smallest angle above the horizon of the approach. 0 switches it off.");
    public static readonly SettingSpec SixDofVzMaxMs = new("SixDofVzMaxMs", "MAX CLIMB RATE", "m/s", T, 0.0, -1e4, 1e4, "%.1f",
        "Largest climb rate while LIMIT CLIMB RATE is on.");
    public static readonly SettingSpec SixDofNodeDtTarget = new("SixDofNodeDtTarget", "POINT SPACING", "s", T, 0.60, 0.0, 60.0, "%.2f",
        "Time between plan points the plan aims for while FEWER POINTS ON APPROACH is on.");
    public static readonly SettingSpec SixDofColdIntervalS = new("SixDofColdIntervalS", "ITERATION GAP", "s", T, 0.25, 0.0, 1.0, "%.2f",
        "Time between two iterations of a first plan spread over frames.");
    public static readonly SettingSpec SixDofReplanSec = new("SixDofReplanSec", "RE-SOLVE EVERY", "s", T, 0.1, 0.0, 60.0, "%.2f",
        "Time between two solves while touchdown guidance flies.");
    public static readonly SettingSpec SixDofRateDampShare = new("SixDofRateDampShare", "RATE DAMPING", "fuel share", T, 0.002, 0.0, 10.0, "%.3f",
        "Weight of the rotation rate against the fuel use.");
    public static readonly SettingSpec SixDofControlSmooth = new("SixDofControlSmooth", "CONTROL SMOOTHING", "weight", T, 0.05, 0.0, 100.0, "%.3f",
        "Weight on control changes between points.");
    public static readonly SettingSpec SixDofProximal = new("SixDofProximal", "PROXIMAL CONDITIONING", "", T, 0.05, 0.0, 100.0, "%.3f",
        "Proximal weight that holds each iteration near the last.");
    public static readonly SettingSpec SixDofTrackKp = new("SixDofTrackKp", "TRACK KP", "1/s^2", T, 0.25, 0.0, 100.0, "%.3f",
        "Position gain of the thrust feedback between re-solves.");
    public static readonly SettingSpec SixDofTrackKd = new("SixDofTrackKd", "TRACK KD", "1/s", T, 0.8, 0.0, 100.0, "%.3f",
        "Velocity gain of the thrust feedback between re-solves.");

    public static readonly BoolSpec SixDofTrack = new("SixDofTrack", "TRACK PLAN THRUST", T, true,
        "Corrects the plan thrust by feedback between re-solves. Off flies it open loop, with the torques from the plan either way.");
    public static readonly BoolSpec SixDofFloorAuto = new("SixDofFloorAuto", "THROTTLE FLOOR FROM CRAFT", T, true,
        "Uses the real minimum throttle of the engine as the floor.");
    public static readonly BoolSpec SixDofGfoldSeed = new("SixDofGfoldSeed", "FIRST PLAN FROM G-FOLD", T, false,
        "Starts the first plan from a G-FOLD solution instead of a straight line.");
    public static readonly BoolSpec SixDofFixedTime = new("SixDofFixedTime", "FIXED BURN TIME", T, false,
        "Holds the burn time fixed and samples it instead of solving for it.");
    public static readonly BoolSpec SixDofVzEnabled = new("SixDofVzEnabled", "LIMIT CLIMB RATE", T, true,
        "Limits the climb rate of the plan.");
    public static readonly BoolSpec SixDofNodeGates = new("SixDofNodeGates", "FEWER POINTS ON APPROACH", T, true,
        "Uses fewer points as the burn gets shorter.");
    public static readonly BoolSpec SixDofThreaded = new("SixDofThreaded", "SOLVE ON WORKER THREAD", T, true,
        "Solves on a worker thread. A solve does not stop the simulation.");
    public static readonly BoolSpec SixDofSpreadCold = new("SixDofSpreadCold", "SPREAD FIRST PLAN", T, true,
        "Spreads the iterations of a first plan over several frames.");
    public static readonly BoolSpec SixDofBiasEnabled = new("SixDofBiasEnabled", "ESTIMATE UNMODELED ACCEL", T, true,
        "Estimates the acceleration the model does not include and corrects for it.");

    #endregion

    #region Entry guidance (3-DOF)

    public static readonly SettingSpec ThreeDofEngageTgoS = new("ThreeDofEngageTgoS", "AT IMPACT T-", "s", P, 45.0, 0.0, 1e4, "%.1f",
        "Predicted time to impact at which ENTRY takes over a boostback glide.");
    public static readonly SettingSpec ThreeDofSixDofTimeS = new("ThreeDofSixDofTimeS", "AT BURN LEFT", "s", P, 5.0, 0.0, 1e3, "%.1f",
        "Burn left in the plan at which TOUCHDOWN takes over the burn, once its standby plan is ready.");
    public static readonly SettingSpec ThreeDofSmoothTau = new("ThreeDofSmoothTau", "COMMAND SMOOTHING", "s", T, 0.05, 0.0, 60.0, "%.2f",
        "Time constant of the low-pass filter on the plan attitude and throttle. 0 switches it off.");
    public static readonly SettingSpec ThreeDofTurnAccelDegS2 = new("ThreeDofTurnAccelDegS2", "PLAN TURN ACCEL", "deg/s^2", T, 0.0, -1.0, 1e3, "%.2f",
        "Largest turn acceleration of the plan. 0 uses the RCS turn acceleration of the craft, read when a first plan starts. A negative value sets no limit.");
    public static readonly SettingSpec ThreeDofGlideCeiling = new("ThreeDofGlideCeiling", "PLAN MAX THROTTLE, GLIDE", "frac", T, 0.88, 0.1, 1.0, "%.2f",
        "Highest throttle the plan uses before the ignition is committed. The burn feedback can use the rest. Read when a first plan starts.");
    public static readonly SettingSpec ThreeDofBurnCeiling = new("ThreeDofBurnCeiling", "PLAN MAX THROTTLE, BURN", "frac", T, 0.95, 0.1, 1.0, "%.2f",
        "Highest throttle the plan uses after the ignition is committed. The burn feedback can use the rest. Read when a first plan starts.");
    public static readonly SettingSpec ThreeDofTrackKp = new("ThreeDofTrackKp", "TRACK KP", "1/s^2", T, 0.25, 0.0, 100.0, "%.3f",
        "Position gain of the burn plan feedback.");
    public static readonly SettingSpec ThreeDofTrackKd = new("ThreeDofTrackKd", "TRACK KD", "1/s", T, 0.8, 0.0, 100.0, "%.3f",
        "Velocity gain of the burn plan feedback.");
    public static readonly SettingSpec ThreeDofTrackTiltDeg = new("ThreeDofTrackTiltDeg", "TRACK MAX TILT", "deg", T, 10.0, 0.0, 30.0, "%.1f",
        "Largest tilt off the axis of the plan the burn feedback may command.");

    public static readonly BoolSpec ThreeDofFromGlide = new("ThreeDofFromGlide", "ENTRY ON GLIDE", P, true,
        "ENTRY takes over a boostback glide at AT IMPACT T- and flies the glide and the landing burn. Off, the glide continues until another mode takes the craft or it is stopped, and no landing burn is planned.");
    public static readonly BoolSpec ThreeDofSixDofHandover = new("ThreeDofSixDofHandover", "TOUCHDOWN FOR BURN END", P, true,
        "TOUCHDOWN flies the last seconds of the burn and models the turn of the craft. Off, ENTRY flies the burn to touchdown.");

    #endregion

    #region Hover

    public static readonly SettingSpec TermTouchdownRate = new("TermTouchdownRate", "TOUCHDOWN RATE", "m/s", E, 0.5, 0.0, 100.0, "%.1f",
        "Constant sink rate of the final meters.");
    public static readonly SettingSpec TermConstAltM = new("TermConstAltM", "CONSTANT-RATE ZONE", "m", E, 0.25, 0.0, 1e4, "%.2f",
        "Height of the zone flown at the touchdown rate.");
    public static readonly SettingSpec TermQuadK = new("TermQuadK", "SINK CURVE", "1/(m s)", T, 0.2, 0.0, 100.0, "%.3f",
        "Shape of the sink rate profile above the constant-rate zone.");
    public static readonly SettingSpec TermMaxDescRate = new("TermMaxDescRate", "MAX DESCENT RATE", "m/s", E, 15.0, 0.0, 1e4, "%.1f",
        "Cap of the sink rate profile.");
    public static readonly SettingSpec TermMaxTiltDeg = new("TermMaxTiltDeg", "MAX TILT", "deg", E, 20.0, 0.0, 90.0, "%.1f",
        "Largest tilt of the thrust from vertical during the hover.");
    public static readonly SettingSpec TermNudgeStep = new("TermNudgeStep", "NUDGE STEP", "m/s", E, 0.5, 0.0, 100.0, "%.1f",
        "Change of the aim per nudge.");
    public static readonly SettingSpec TermKpV = new("TermKpV", "VERTICAL KP", "", T, 1.5, 0.0, 1000.0, "%.3f", "Vertical proportional gain.");
    public static readonly SettingSpec TermKiV = new("TermKiV", "VERTICAL KI", "", T, 0.0, 0.0, 1000.0, "%.3f", "Vertical integral gain.");
    public static readonly SettingSpec TermKdV = new("TermKdV", "VERTICAL KD", "", T, 0.0, 0.0, 1000.0, "%.3f", "Vertical derivative gain.");
    public static readonly SettingSpec TermKpL = new("TermKpL", "LATERAL KP", "", T, 0.10, 0.0, 1000.0, "%.3f", "Lateral proportional gain.");
    public static readonly SettingSpec TermKiL = new("TermKiL", "LATERAL KI", "", T, 0.0, 0.0, 1000.0, "%.3f", "Lateral integral gain.");
    public static readonly SettingSpec TermKdL = new("TermKdL", "LATERAL KD", "", T, 0.0, 0.0, 1000.0, "%.3f", "Lateral derivative gain.");

    #endregion

    #region Tier arrays

    // LanDeg is left out of every count: it is seeded from the launch site, so the fixed default would mark it changed on every flight. The values a picked target writes stay in the arrays, and SettingAccess.SetBy keeps them out of the count while the chase writes them.
    public static readonly SettingSpec[] LaunchPlayer = { ConvexQMaxKpa, MoonTransferPeKm };
    public static readonly BoolSpec[] LaunchPlayerBools = { FlyConvexAscent, LaunchDescending };
    public static readonly SettingSpec[] LaunchExpert =
        { ChaseOffsetKm, ArgPeDeg, ConvexThrottleMinPct, TurnStartAltKm, TurnRateDegS, ForceRollDeg };
    public static readonly BoolSpec[] LaunchExpertBools =
        { MatchTargetArgPe, ArgPeFixed, OptimiseInsertion, InsertBeforeApoapsis, ConvexOptimiseTiming, ShowConvexPlan, ForceRoll };

    public static readonly SettingSpec[] LandBoostbackExpert =
        { BoostbackClearanceM, BoostbackSeparationS, BoostbackPitchDeg, GlideMaxAoaDeg, GlideNominalAoaDeg };

    /// <summary>The HANDOVER settings of a boostback glide and of entry guidance: who takes the glide and who takes the end of the burn.</summary>
    public static readonly SettingSpec[] LandHandOvers = { ThreeDofEngageTgoS, ThreeDofSixDofTimeS };
    public static readonly BoolSpec[] LandHandOverBools = { ThreeDofFromGlide, ThreeDofSixDofHandover };

    public static readonly SettingSpec[] LandGfoldPlayer = { GfoldHoverHandoffAltM };

    public static readonly SettingSpec[] LandFromOrbitExpert = { BrakingAltitudeKm, DeorbitArrivalDescentDeg, DescentRate, LandingVerticalGateM };
    public static readonly BoolSpec[] LandFromOrbitExpertBools = { DeorbitArrivalAngleEnabled };
    public static readonly SettingSpec[] LandEntryExpert = { LandingAccelLimitG };
    public static readonly BoolSpec[] LandEntryExpertBools = Array.Empty<BoolSpec>();
    public static readonly SettingSpec[] LandTouchdownExpert = { LandingAccelLimitG };
    public static readonly SettingSpec[] LandGfoldExpert = Array.Empty<SettingSpec>();

    public static readonly SettingSpec[] HoverExpert =
        { TermTouchdownRate, TermConstAltM, TermMaxDescRate, TermMaxTiltDeg, TermNudgeStep };

    public static readonly SettingSpec[] TuningAscent = { ConvexQAlphaMax, ConvexHandoverAltKm };
    public static readonly SettingSpec[] TuningBoostback = { BoostbackSlewDegS, GlideKp, GlideKi, GlideKd };
    public static readonly SettingSpec[] TuningApproach = { DownrangeFactor, AimAltKm, GateUprangeKm, GfoldHandoffTgo };

    // LandingPlanFloorMargin is one value of both plans, so it is in both landing sets and counted once in AllTuning.
    public static readonly SettingSpec[] TuningThreeDof =
    {
        ThreeDofSmoothTau, ThreeDofTurnAccelDegS2, ThreeDofGlideCeiling, ThreeDofBurnCeiling, LandingPlanFloorMargin,
        ThreeDofTrackKp, ThreeDofTrackKd, ThreeDofTrackTiltDeg,
    };
    public static readonly SettingSpec[] TuningGfold =
    {
        GfoldGlideSlopeDeg, GfoldPointingDeg, GfoldVMaxMs, GfoldThrottleMin, GfoldThrottleMax, GfoldSlewReg,
        GfoldIntervalS, GfoldNodes, GfoldKp, GfoldKd, GfoldSmoothTau, GfoldSolveTimeLimitS,
    };
    public static readonly SettingSpec[] TuningGfoldLimits = { GfoldGlideSlopeDeg, GfoldPointingDeg, GfoldVMaxMs, GfoldThrottleMin, GfoldThrottleMax };
    public static readonly SettingSpec[] TuningGfoldTracking = { GfoldKp, GfoldKd, GfoldSmoothTau, GfoldIntervalS };
    public static readonly SettingSpec[] TuningGfoldSolver = { GfoldNodes, GfoldSlewReg, GfoldSolveTimeLimitS };

    public static readonly SettingSpec[] TuningSixDof =
    {
        SixDofNodes, SixDofTiltDeg, SixDofThrottleFloor, SixDofSigmaSeed, SixDofSigmaSamples, SixDofGlideSlopeDeg,
        SixDofVzMaxMs, SixDofNodeDtTarget, SixDofColdIntervalS, SixDofReplanSec, SixDofRateDampShare,
        SixDofControlSmooth, SixDofProximal, SixDofTrackKp, SixDofTrackKd, LandingPlanFloorMargin,
    };
    public static readonly BoolSpec[] TuningSixDofBools =
    {
        SixDofFloorAuto, SixDofGfoldSeed, SixDofFixedTime, SixDofVzEnabled, SixDofNodeGates, SixDofThreaded,
        SixDofSpreadCold, SixDofBiasEnabled, SixDofTrack,
    };
    public static readonly SettingSpec[] TuningSixDofLimits =
        { SixDofTiltDeg, SixDofThrottleFloor, SixDofGlideSlopeDeg, SixDofVzMaxMs, LandingPlanFloorMargin };
    public static readonly BoolSpec[] TuningSixDofLimitBools = { SixDofFloorAuto, SixDofVzEnabled };
    public static readonly SettingSpec[] TuningSixDofTracking = { SixDofTrackKp, SixDofTrackKd, SixDofReplanSec };
    public static readonly BoolSpec[] TuningSixDofTrackingBools = { SixDofTrack, SixDofBiasEnabled };
    public static readonly SettingSpec[] TuningSixDofSolver =
    {
        SixDofNodes, SixDofNodeDtTarget, SixDofSigmaSeed, SixDofSigmaSamples, SixDofColdIntervalS, SixDofRateDampShare,
        SixDofControlSmooth, SixDofProximal,
    };
    public static readonly BoolSpec[] TuningSixDofSolverBools = { SixDofNodeGates, SixDofGfoldSeed, SixDofFixedTime, SixDofThreaded, SixDofSpreadCold };

    public static readonly SettingSpec[] TuningHover = { TermQuadK, TermKpV, TermKiV, TermKdV, TermKpL, TermKiL, TermKdL };

    public static readonly SettingSpec[] TuningLanding = Distinct(
        Concat(Concat(TuningApproach, TuningThreeDof), Concat(Concat(TuningGfold, TuningSixDof), TuningHover)));

    public static readonly SettingSpec[] AllTuning = Distinct(
        Concat(Concat(Concat(TuningAscent, TuningBoostback), Concat(TuningApproach, TuningThreeDof)),
            Concat(Concat(TuningGfold, TuningSixDof), TuningHover)));

    public static readonly BoolSpec[] AllTuningBools = TuningSixDofBools;

    /// <summary>Every numeric spec, for the uniqueness and default checks.</summary>
    public static readonly SettingSpec[] AllDoubles =
    {
        PeKm, ApKm, IncDeg, LanDeg, ArgPeDeg, ChaseOffsetKm, MoonTransferPeKm, GLimitG, TurnStartAltKm, TurnRateDegS, ForceRollDeg,
        ConvexQMaxKpa, ConvexThrottleMinPct, ConvexQAlphaMax, ConvexHandoverAltKm,
        BoostbackSeparationS, BoostbackPitchDeg, GlideMaxAoaDeg, GlideNominalAoaDeg, BoostbackClearanceM,
        BoostbackSlewDegS, GlideKp, GlideKi, GlideKd,
        SiteLatDeg, SiteLonDeg, BrakingAltitudeKm, DeorbitArrivalDescentDeg, DescentRate, LandingVerticalGateM,
        DownrangeFactor, AimAltKm, GateUprangeKm, GfoldHandoffTgo, VehicleHeightM, GfoldHoverHandoffAltM,
        LandingAccelLimitG, LandingPlanFloorMargin,
        GfoldGlideSlopeDeg, GfoldPointingDeg, GfoldVMaxMs, GfoldThrottleMin, GfoldThrottleMax, GfoldSlewReg,
        GfoldIntervalS, GfoldNodes, GfoldKp, GfoldKd, GfoldSmoothTau, GfoldSolveTimeLimitS,
        SixDofNodes, SixDofTiltDeg, SixDofThrottleFloor, SixDofSigmaSeed,
        SixDofSigmaSamples, SixDofGlideSlopeDeg, SixDofVzMaxMs, SixDofNodeDtTarget, SixDofColdIntervalS,
        SixDofReplanSec, SixDofRateDampShare, SixDofControlSmooth, SixDofProximal, SixDofTrackKp, SixDofTrackKd,
        ThreeDofEngageTgoS, ThreeDofSixDofTimeS, ThreeDofSmoothTau, ThreeDofTurnAccelDegS2, ThreeDofGlideCeiling,
        ThreeDofBurnCeiling, ThreeDofTrackKp, ThreeDofTrackKd, ThreeDofTrackTiltDeg,
        TermTouchdownRate, TermConstAltM, TermQuadK, TermMaxDescRate, TermMaxTiltDeg, TermNudgeStep,
        TermKpV, TermKiV, TermKdV, TermKpL, TermKiL, TermKdL,
    };

    public static readonly BoolSpec[] AllBools =
    {
        GLimitEnabled, AutoLaunch, MatchTargetArgPe, ArgPeFixed, OptimiseInsertion, InsertBeforeApoapsis,
        FlyConvexAscent, ConvexOptimiseTiming, ShowConvexPlan, ForceRoll, LaunchDescending, Engage, AutoStage, DeorbitArrivalAngleEnabled,
        VehicleHeightAuto, SixDofFloorAuto, SixDofGfoldSeed, SixDofFixedTime, SixDofVzEnabled, SixDofNodeGates,
        SixDofThreaded, SixDofSpreadCold, SixDofBiasEnabled, SixDofTrack, ThreeDofFromGlide, ThreeDofSixDofHandover,
    };

    #endregion

    private static T[] Concat<T>(T[] a, T[] b)
    {
        var all = new T[a.Length + b.Length];
        Array.Copy(a, all, a.Length);
        Array.Copy(b, 0, all, a.Length, b.Length);
        return all;
    }

    private static SettingSpec[] Distinct(SettingSpec[] specs)
    {
        var keys = new HashSet<string>(StringComparer.Ordinal);
        var list = new List<SettingSpec>(specs.Length);
        foreach (SettingSpec spec in specs)
        {
            if (keys.Add(spec.Key))
                list.Add(spec);
        }
        return list.ToArray();
    }
}

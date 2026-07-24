namespace AdvancedFlightComputer.Core;

/// <summary>
/// Per-feature debug toggles. In DEBUG builds all flags default to true;
/// set individual flags to false at the top of this file to reduce log
/// noise while debugging a specific feature. In Release builds everything
/// defaults to off.
/// </summary>
internal static class DebugConfig
{
#if DEBUG
    public static bool HyperbolicTargets = true;
    public static bool ManeuverTools = true;
    public static bool MultiPass = true;
    public static bool RcsTranslation = true;
    public static bool Flyby = true;
    public static bool Performance = true;
#else
    public static bool HyperbolicTargets = false;
    public static bool ManeuverTools = false;
    public static bool MultiPass = false;
    public static bool RcsTranslation = false;
    public static bool Flyby = false;
    public static bool Performance = false;
#endif

    public static bool Any => HyperbolicTargets || ManeuverTools || MultiPass
        || RcsTranslation || Flyby || Performance;
}

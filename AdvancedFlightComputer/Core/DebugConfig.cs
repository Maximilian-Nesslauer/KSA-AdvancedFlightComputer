namespace AdvancedFlightComputer.Core;

/// <summary>
/// Per-feature debug logging switches. They are on in DEBUG builds and off in Release. Flip one
/// here to quiet a feature while debugging another.
/// </summary>
internal static class DebugConfig
{
#if DEBUG
    public static bool HyperbolicTargets = true;
    public static bool ManeuverTools = true;
    public static bool MultiPass = true;
    public static bool RcsTranslation = true;
    public static bool Flyby = true;
    public static bool AutoStage = true;
    public static bool Performance = true;
#else
    public static bool HyperbolicTargets = false;
    public static bool ManeuverTools = false;
    public static bool MultiPass = false;
    public static bool RcsTranslation = false;
    public static bool Flyby = false;
    public static bool AutoStage = false;
    public static bool Performance = false;
#endif
}

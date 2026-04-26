namespace AdvancedFlightComputer.Core;

/// <summary>
/// Per-feature debug toggles. In DEBUG builds all flags default to true;
/// set individual flags to false at the top of this file to reduce log
/// noise while debugging a specific feature. In Release builds everything
/// is off and the JIT eliminates dead branches.
/// </summary>
internal static class DebugConfig
{
#if DEBUG
    public static bool HyperbolicTargets = true;
    public static bool AutoStage = true;
    public static bool StageInfo = true;
    public static bool ManeuverTools = true;
    public static bool Performance = true;
#else
    public static bool HyperbolicTargets = false;
    public static bool AutoStage = false;
    public static bool StageInfo = false;
    public static bool ManeuverTools = false;
    public static bool Performance = false;
#endif

    public static bool Any =>
        HyperbolicTargets || AutoStage || StageInfo || ManeuverTools || Performance;
}

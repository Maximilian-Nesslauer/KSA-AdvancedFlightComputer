namespace AdvancedFlightComputer.Core;

/// <summary>
/// Per-feature debug toggles. In DEBUG builds all flags default to true;
/// set individual flags to false at the top of this file to reduce log
/// noise while debugging a specific feature. In Release builds everything
/// is off and the JIT eliminates dead branches.
/// </summary>
static class DebugConfig
{
#if DEBUG
    public static bool HyperbolicTargets = true;
    public static bool StageInfo = true;
    public static bool ManeuverTools = true;
    public static bool OberthMultiPass = true;
    public static bool AutoRemoveBurn = true;
    public static bool Performance = false;
#else
    public static bool HyperbolicTargets = false;
    public static bool StageInfo = false;
    public static bool ManeuverTools = false;
    public static bool OberthMultiPass = false;
    public static bool AutoRemoveBurn = false;
    public static bool Performance = false;
#endif

    public static bool Any =>
        HyperbolicTargets || StageInfo || ManeuverTools
        || OberthMultiPass || AutoRemoveBurn || Performance;
}

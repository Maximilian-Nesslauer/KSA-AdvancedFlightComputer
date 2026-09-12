namespace AutoRemoveFinishedBurns.Core;

static class DebugConfig
{
#if DEBUG
    public static bool Detection = true;
    public static bool Settings = true;
    public static bool Performance = true;
#else
    public static bool Detection = false;
    public static bool Settings = false;
    public static bool Performance = false;
#endif

    public static bool Any => Detection || Settings || Performance;
}

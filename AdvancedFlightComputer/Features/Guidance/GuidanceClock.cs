namespace AdvancedFlightComputer.Features.Guidance;

// The clock the guidance step gates its costly refreshes on: the stage model, the insertion search, the launch window, the deorbit checks and the housekeeping retries.
// The gates measure real time on purpose, because under time warp a sim-time gate would let the work run on every step.
// A harness test sets the source to the simulation time, so a flight takes the same steps however fast the machine runs it.
internal static class GuidanceClock
{
    internal static Func<long> Source = RealTimeMs;

    internal static long NowMs => Source();

    internal static void Reset() => Source = RealTimeMs;

    private static long RealTimeMs() => Environment.TickCount64;
}

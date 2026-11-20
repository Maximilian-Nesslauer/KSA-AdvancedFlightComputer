using System.Diagnostics;

namespace AdvancedFlightComputer.HarnessTests;

/// <summary>Waits for the aero sweeps that guidance runs on their own threads (KsaAeroSweep.Job).</summary>
internal static class AeroSweepWait
{
    /// <summary>The longest wait for a sweep, s of wall time. A sweep takes about a second, and the sweeps of one craft run in parallel.</summary>
    internal const double TimeoutS = 30.0;

    /// <summary>
    /// Polls <paramref name="done"/> until it is true or <see cref="TimeoutS"/> of wall time passes, without a sim step. The sweep runs against the wall clock, so a bound in sim time would depend on how fast the machine steps, and a sim that stands still keeps the craft in the state the test set up.
    /// </summary>
    internal static bool Until(Func<bool> done)
    {
        var clock = Stopwatch.StartNew();
        while (!done())
        {
            if (clock.Elapsed.TotalSeconds >= TimeoutS)
                return false;
            Thread.Sleep(10);
        }
        return true;
    }
}

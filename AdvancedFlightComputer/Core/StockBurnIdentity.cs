using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Whether the flight computer's loaded burn target belongs to a planned burn.
/// The target is a separate object that outlives its plan entry, because <c>Vehicle.UpdateFromTaskResultsUnsynchronized</c> runs <c>FlightComputer.DeleteInvalidBurns</c> before <c>FlightComputer.CopyFrom</c> writes the worker's target back, so a reference to the plan entry alone cannot prove which burn is loaded.
/// <c>BurnTarget.UpdateFromBurn</c> takes its impulse instant from <see cref="Burn.ImpulseTime"/>, which lies half the burn after <see cref="Burn.Time"/> for an interstellar burn, so the match is against that.
/// </summary>
internal static class StockBurnIdentity
{
    // Wide enough for a burn time edit the worker has not copied into the target yet, narrow enough to tell adjacent burns apart.
    internal const double ToleranceSec = 0.5;

    internal static bool IsLoaded(BurnTarget? target, Burn? burn)
        => target != null && burn != null
           && Math.Abs((target.ImpulsiveInstant - burn.ImpulseTime).Seconds()) <= ToleranceSec;
}

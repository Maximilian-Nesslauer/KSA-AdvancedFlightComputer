using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Public interop surface: raised on the main thread after an RCS
/// translation burn completes, before the burn node is touched. Consumers
/// (AutoRemoveFinishedBurns) bind this via reflection soft-dependency, so
/// the type name, event name, and signature are a cross-mod API - renaming
/// any of them breaks consumers silently. The parameter types are
/// deliberately all from KSA.dll so a consumer never needs an
/// AdvancedFlightComputer reference to build a matching delegate.
/// </summary>
public static class RcsBurnCompletions
{
    public static event Action<Vehicle, Burn>? Completed;

    internal static void Raise(Vehicle vehicle, Burn burn)
    {
        Delegate[]? subscribers = Completed?.GetInvocationList();
        if (subscribers == null)
            return;
        // Per-subscriber isolation: a plain multicast invoke would let one
        // throwing subscriber deny delivery to every later one.
        foreach (Delegate subscriber in subscribers)
        {
            try
            {
                ((Action<Vehicle, Burn>)subscriber).Invoke(vehicle, burn);
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] RcsBurnCompletions subscriber threw for vehicle='{vehicle.Id}': {ex}");
            }
        }
    }

    internal static void Reset() => Completed = null;
}

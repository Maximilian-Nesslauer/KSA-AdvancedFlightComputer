using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>Raised on the main thread after execution teardown and before the burn node is changed. Keep the type, event name, and signature stable for consumers that bind by reflection. KSA parameter types let consumers bind without an AFC reference.</summary>
public static class RcsBurnCompletions
{
    public static event Action<Vehicle, Burn>? Completed;

    internal static void Raise(Vehicle vehicle, Burn burn)
    {
        Delegate[]? subscribers = Completed?.GetInvocationList();
        if (subscribers == null)
            return;
        // One failed subscriber must not prevent delivery to the remaining subscribers.
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

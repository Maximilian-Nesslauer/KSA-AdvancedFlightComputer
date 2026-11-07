using AdvancedFlightComputer.Core;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// The one way a player cancels an RCS burn, from the console head, the head drawer and the block next
/// to the stock burn editor alike. An explicit stop must not give stock AUTO back, so the forced
/// MANUAL flag is cleared before the cancel request.
/// </summary>
internal static class RcsStop
{
    public static bool Cancel(Vehicle vehicle)
    {
        if (vehicle == null || !SharedVehicleHooks.RcsEnabled || !RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec))
            return false;
        Cancel(exec);
        return true;
    }

    public static void Cancel(RcsExecution exec)
    {
        exec.ForcedBurnManual = false;
        RcsExecutor.RequestCancel(exec, "user request");
    }

    /// <summary>Tries the control cleanup of a burn that stopped after a fault again.</summary>
    public static void RetryCleanup(RcsExecution exec) => RcsExecutor.RequestCancel(exec, "retry fault cleanup");
}

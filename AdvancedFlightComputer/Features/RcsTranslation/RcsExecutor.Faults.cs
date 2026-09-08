using AdvancedFlightComputer.Core;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

internal static partial class RcsExecutor
{
    internal const int MaxCleanupAttempts = 3;

    internal static void StopAfterFault(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        RcsCommandChannel.Clear(fc.BurnPlan);
        if (!RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec))
            return;

        // Keep the burn key and outstanding ownership until cleanup succeeds. Faulted prevents all execution and survives a save.
        exec.Faulted = true;
        exec.ControlTaken = false;
        exec.LastPublishedCommand = null;
        exec.LastFuel = default;
        exec.CancelRequestReason = null;
        RetryFaultCleanup(fc, exec);
    }

    private static void RetryFaultCleanup(FlightComputer fc, RcsExecution exec)
    {
        RcsCommandChannel.Clear(fc.BurnPlan);
        if (!exec.CleanupPending)
        {
            exec.ClearActive();
            exec.CleanupError = null;
            return;
        }
        if (exec.CleanupAttempts >= MaxCleanupAttempts)
            return;

        exec.CleanupAttempts++;
        try
        {
            EndExecution(fc, exec);
            exec.CleanupError = null;
        }
        catch (Exception ex)
        {
            exec.CleanupError = ex.Message;
            LogHelper.WarnOnce($"rcs-cleanup-{exec.VehicleId}-{exec.CleanupAttempts}",
                $"[AFC] RCS fault cleanup failed on '{exec.VehicleId}' " +
                $"(attempt {exec.CleanupAttempts}/{MaxCleanupAttempts}): {ex}");
        }
    }
}

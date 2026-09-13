#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using KSA;

/// <summary>
/// Releases guidance state on lifecycle events and queues cleanup for vehicle steps.
/// </summary>
public static partial class GuidanceWindow
{
    // A step that throws keeps its craft for a short run, because the flight computer still holds the last attitude target and engine command, and one bad step is not worth an engine cut. A run this long releases the craft.
    internal const int MaxFailedSteps = 30;

    // The dispose postfix runs after the vehicle is removed, so only process resources need cleanup.
    internal static void ReleaseDisposedVehicle(Vehicle vehicle) => DropVehicle(vehicle, release: false);

    internal static void ReleaseAllVehicles()
    {
        // Try to restore live vehicles before unload. Vehicles replaced by a save load need only resource cleanup.
        foreach (var entry in VehicleAutopilotState.Snapshot())
            DropVehicle(entry.Key, release: !entry.Key.IsDisposed);
        ClearLifecycleRequests();
        _s = new VehicleAutopilotState();
    }

    // Remove terminal state even if cleanup fails because no later vehicle step can retry it.
    private static void DropVehicle(Vehicle vehicle, bool release)
    {
        if (!VehicleAutopilotState.TryGet(vehicle, out var state))
            return;
        var previous = _s;
        try
        {
            _s = state;
            if (release)
            {
                HandBackVehicle(vehicle);
                // No later step retries this release, so a failed one must not leave stock's burn mode forced to Manual for good.
                RestoreBurnMode(vehicle, giveBack: true);
            }
            DiscardResources();
        }
        finally
        {
            VehicleAutopilotState.Remove(vehicle);
            // The removed state must not stay in the ambient reference, or it outlives its craft.
            _s = ReferenceEquals(previous, state) ? new VehicleAutopilotState() : previous;
        }
    }

    private static void DiscardResources()
    {
        try
        {
            _s.Worker?.Dispose();
        }
        catch (Exception error)
        {
            Console.Error.WriteLine("[AFC Guidance] Worker cleanup failed: " + error.Message);
        }
        _s.Worker = null;
        _s.Guidance = null;

        try
        {
            ReportLogStop(SixDofLog.Stop(_s));
        }
        catch (Exception error)
        {
            ReportLogStop(error.Message);
        }
    }

    private static void ReportLogStop(string error)
    {
        if (error.Length > 0)
            Console.Error.WriteLine("[AFC Guidance] Log cleanup failed: " + error);
    }

    // Defer control writes to the vehicle step so worker results cannot overwrite them.
    internal static void QueueAllReleases()
    {
        foreach (var entry in VehicleAutopilotState.Snapshot())
            entry.Value.FcResetPending = true;
        ClearLifecycleRequests();
    }

    private static void ClearLifecycleRequests()
    {
        _warpPromptActive = false;
        _warpLabel = "";
        _warpDeclinedLabel = "";
        _warpTargetSimSec = 0;
        _handovers.Clear();
        _separationDrops.Clear();
        _retargetArmed = false;
        _autoLaunchStepped = false;
    }

    internal static void StepSucceeded(Vehicle vehicle)
    {
        if (VehicleAutopilotState.TryGet(vehicle, out var state))
            state.FailedSteps = 0;
    }

    /// <summary>Counts a step that threw and returns true once the run is long enough to release the craft.</summary>
    internal static bool CountFailedStep(Vehicle vehicle) =>
        VehicleAutopilotState.TryGet(vehicle, out var state) && ++state.FailedSteps >= MaxFailedSteps;

    internal static void FailAutopilot(Vehicle vehicle, Exception error)
    {
        if (!VehicleAutopilotState.TryGet(vehicle, out var state))
            return;
        _s = state;
        _s.FailedSteps = 0;
        _s.Error = "Guidance stopped: " + error.Message;
        HandBackVehicle(vehicle);
    }

    internal static string ReleaseFailure(Vehicle vehicle) =>
        VehicleAutopilotState.TryGet(vehicle, out var state) ? state.ReleaseError : "";
}

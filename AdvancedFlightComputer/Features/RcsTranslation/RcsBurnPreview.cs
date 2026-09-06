using System.Runtime.CompilerServices;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

internal static class RcsBurnPreview
{
    // Reuse scratch targets without retaining burns after the editor and plan release them.
    private static readonly ConditionalWeakTable<Burn, BurnTarget> Targets = new();

    internal static bool TryGetEstimates(Burn burn, Vehicle vehicle, FlightComputer fc,
        RcsExecution? exec, out RcsEstimates estimates, out bool currentVehicle)
    {
        currentVehicle = !ReferenceEquals(burn, fc.BurnPlan.FindFirstExecutableBurn())
            || !RcsBurnUi.HasEstimatesFor(burn.Time.Seconds(), fc.Burn, exec);
        if (!currentVehicle)
        {
            estimates = exec!.Estimates;
            return true;
        }

        // Use the planned direction with current vehicle data. This does not load or command a burn.
        BurnTarget target = Targets.GetValue(burn, static _ => new BurnTarget());
        target.UpdateFromBurn(burn);
        RcsCapabilitySnapshot capability = RcsExecutor.ProbeCached(vehicle);
        estimates = RcsExecutor.ComputeEstimates(vehicle, target, in capability);
        return estimates.Valid;
    }
}

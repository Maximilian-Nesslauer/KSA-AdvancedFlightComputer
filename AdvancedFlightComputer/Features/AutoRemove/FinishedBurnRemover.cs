using AdvancedFlightComputer.Core;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.AutoRemove;

// Removes the finished burn stock leaves in the plan.
// FlightComputer.EndAutoBurn removes a finished burn itself while another burn follows it, so what reaches this class still planned is the last burn of the plan. Engine auto-burns and RCS burns end the same way, because the RCS executor raises FlightComputer.AutoBurnCompleted too.
internal static class FinishedBurnRemover
{
    // Called after FlightComputer.RaisePendingAlerts, so a burn still in the plan is one stock kept.
    // Every vehicle, like the stock removal, and only while the burn is still in the plan, because MultiPass can have removed its own pass first.
    // Burn equality is by time and delta-V, so the plan's own instance is resolved and removed.
    internal static void OnBurnCompleted(Vehicle vehicle, Burn burn)
    {
        if (!AutoRemoveConfig.Enabled)
            return;
        FlightComputer fc = vehicle.FlightComputer;
        int index = fc.BurnPlan.TryGetBurnIndex(burn);
        if (index < 0 || !fc.BurnPlan.TryGetBurn(index, out Burn? planBurn) || planBurn == null)
            return;

        if (DebugConfig.AutoRemove)
            DefaultCategory.Log.Debug(
                $"[AFC] AutoRemove: '{vehicle.Id}' finished its last planned burn (dv={planBurn.DeltaVVlf.Length():F2}m/s); removing it from the plan.");

        fc.RemoveBurn(planBurn);
    }
}

using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

[HarmonyPatch(typeof(Universe), nameof(Universe.AutoWarpTo),
    new Type[] { typeof(UniverseTime), typeof(double) })]
internal static class RcsWarpPatch
{
    static void Prefix(UniverseTime endTime, ref double simTimeMargin)
    {
        Vehicle? vehicle = Program.ControlledVehicle;
        if (vehicle != null)
            TryAdjustMargin(vehicle, endTime, ref simTimeMargin);
    }

    internal static bool TryAdjustMargin(
        Vehicle vehicle, UniverseTime endTime, ref double simTimeMargin)
    {
        FlightComputer fc = vehicle.FlightComputer;
        BurnTarget? bt = fc.Burn;
        if (bt == null || endTime != bt.IgnitionTime || !double.IsFinite(simTimeMargin))
            return false;

        Burn? burn = fc.BurnPlan.FindFirstExecutableBurn();
        if (burn == null
            || Math.Abs((bt.ImpulsiveInstant - burn.Time).Seconds()) > RcsExecutor.BurnIdentityToleranceSec
            || !RcsExecutor.WouldExecuteRcs(vehicle, out RcsCapabilitySnapshot capability))
        {
            return false;
        }

        RcsEstimates estimates = RcsExecutor.ComputeEstimates(vehicle, bt, in capability);
        if (!estimates.Valid)
            return false;

        UniverseTime controlTime = bt.ImpulsiveInstant - RcsExecutor.AlignLeadSeconds(in estimates);
        double requiredMargin = (endTime - controlTime).Seconds();
        if (!double.IsFinite(requiredMargin) || requiredMargin <= simTimeMargin)
            return false;

        simTimeMargin = requiredMargin;
        return true;
    }
}

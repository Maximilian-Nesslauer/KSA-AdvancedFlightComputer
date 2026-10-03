using AdvancedFlightComputer.Core;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

// Every stock warp to the next burn passes through this one overload. FlightComputer.Handle and FlightComputer.ToggleAction call it directly, and Universe.WarpToNext reaches it through InputEvents.AutoWarpData.
// They aim at BurnTarget.IgnitionTime with FlightComputer.IGNITION_TIME_AUTO_WARP_MARGIN, which is too short for an RCS burn that has to align first. Warps to any other time are left alone.
// Only the margin grows, so Universe.AutoWarpTime still reads the ignition time that Vehicle.IsSet compares against, and isUncapped passes through untouched.
[HarmonyPatch(typeof(Universe), nameof(Universe.AutoWarpTo),
    new Type[] { typeof(UniverseTime), typeof(double), typeof(bool) })]
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
        if (!StockBurnIdentity.IsLoaded(bt, burn)
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

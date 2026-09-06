using AdvancedFlightComputer.Core;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

internal static class RcsDriverPatch
{
    internal static void TickVehicle(Vehicle vehicle)
    {
        try
        {
            RcsExecutor.Tick(vehicle);
        }
        catch (Exception ex)
        {
            // A published worker command can outlive a failed tick and keep engine control disabled. Clear it to return control to stock.
            RcsCommandChannel.Clear(vehicle.FlightComputer.BurnPlan);
            LogHelper.WarnOnce($"rcs-driver-{vehicle.Id}",
                $"[AFC] RcsDriverPatch for vehicle='{vehicle.Id}': {ex}");
        }
    }
}

/// <summary>
/// Intercepts the BurnMode leg of <see cref="Vehicle.SetEnum"/> (the sink
/// of both the gauge Auto button and the burn-mode hotkeys) so a burn that
/// resolves to RCS engages this executor instead of the stock engine
/// autopilot, and a click while running cancels.
/// </summary>
[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.SetEnum))]
internal static class RcsSetEnumPatch
{
    static bool Prefix(Vehicle __instance, Enum? enumValue)
    {
        if (enumValue is not FlightComputerBurnMode mode)
            return true;
        return RcsExecutor.OnBurnModeSetEnum(__instance, mode);
    }
}

internal static class RcsVehicleDisposePatch
{
    internal static void Remove(Vehicle vehicle)
        => RcsExecRegistry.Remove(vehicle.Id);
}

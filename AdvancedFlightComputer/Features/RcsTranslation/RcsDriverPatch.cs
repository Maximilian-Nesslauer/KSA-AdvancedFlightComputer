using AdvancedFlightComputer.Core;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

internal static class RcsDriverPatch
{
    internal static void TickVehicle(Vehicle vehicle)
    {
        // Check ownership before an active execution or pending cleanup writes control.
        if (!RcsExecutor.ReconcileClaim(vehicle))
            return;

        try
        {
            RcsExecutor.Tick(vehicle);
        }
        catch (Exception ex)
        {
            RcsExecutor.StopAfterFault(vehicle);
            LogHelper.WarnOnce($"rcs-driver-{vehicle.Id}:{ex.GetType().Name}",
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
[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.SetEnum), new Type[] { typeof(Enum) })]
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

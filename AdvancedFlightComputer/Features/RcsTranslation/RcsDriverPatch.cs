using AdvancedFlightComputer.Core;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Main-thread per-tick driver hook: a postfix on
/// <see cref="Universe.ApplyVehicleSolvers"/> (the same site the multi-pass
/// PassCompletionPatch uses) that runs the RCS executor state machine once
/// per vehicle, after the solver results have been copied back to every
/// vehicle and before <c>InputEvents.ApplyInputEvents</c> drains the buffers
/// the executor queues into.
///
/// Every vehicle is walked, not just the ones with a registry entry: the
/// executor creates the entry itself once a burn resolves to RCS, so the burn
/// editor has estimates before the first click.
/// </summary>
[HarmonyPatch(typeof(Universe), nameof(Universe.ApplyVehicleSolvers))]
internal static class RcsDriverPatch
{
    static void Postfix()
    {
        foreach (Astronomical astro in LoadedVehicles.All)
        {
            if (astro is not Vehicle vehicle || vehicle.IsDisposed)
                continue;
            try
            {
                RcsExecutor.Tick(vehicle);
            }
            catch (Exception ex)
            {
                // Fail closed. Tick owns every terminal path of the executor, so
                // a throw leaves the last RcsWorkerCommand published and the
                // worker keeps acting on it: RcsComputeControlPatch zeroes the
                // engine commands for that BurnPlan on every solver pass, which
                // would silently deny engine burns for the rest of the session.
                // Dropping the command hands the vehicle back to stock.
                RcsCommandChannel.Clear(vehicle.FlightComputer.BurnPlan);

                // Once per vehicle per load: this runs at frame rate and a
                // persistent failure would otherwise flood the log.
                LogHelper.WarnOnce($"rcs-driver-{vehicle.Id}",
                    $"[AFC] RcsDriverPatch for vehicle='{vehicle.Id}': {ex}");
            }
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

/// <summary>Drops a disposed vehicle's registry entry so a recycled id
/// cannot inherit an orphaned execution, mirroring the multi-pass
/// VehicleDisposePatch.</summary>
[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.Dispose), new[] { typeof(bool) })]
internal static class RcsVehicleDisposePatch
{
    static void Postfix(Vehicle __instance)
        => RcsExecRegistry.Remove(__instance.Id);
}

using AdvancedFlightComputer.Core;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Main-thread per-tick driver hook: a postfix on
/// <see cref="Vehicle.UpdateFromTaskResults"/> (the same site the
/// multi-pass PassCompletionPatch uses) that runs the RCS executor state
/// machine after the solver results have been copied back to the vehicle.
/// </summary>
[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.UpdateFromTaskResults),
    new[] { typeof(VehicleUpdateData), typeof(BubbleOrigin), typeof(Vehicle), typeof(ReadOnlySpan<Vehicle>), typeof(double3), typeof(double3) },
    new[] { ArgumentType.Ref, ArgumentType.Ref, ArgumentType.Normal, ArgumentType.Normal, ArgumentType.Normal, ArgumentType.Normal })]
internal static class RcsDriverPatch
{
    static void Postfix(Vehicle __instance)
    {
        try
        {
            RcsExecutor.Tick(__instance);
        }
        catch (Exception ex)
        {
            // Once per vehicle per load: this runs at solver rate and a
            // persistent failure would otherwise flood the log.
            LogHelper.WarnOnce($"rcs-driver-{__instance.Id}",
                $"[AFC] RcsDriverPatch for vehicle='{__instance.Id}': {ex}");
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
[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.Dispose), new Type[0])]
internal static class RcsVehicleDisposePatch
{
    static void Postfix(Vehicle __instance)
        => RcsExecRegistry.Remove(__instance.Id);
}

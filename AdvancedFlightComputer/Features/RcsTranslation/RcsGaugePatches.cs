using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Makes the stock gauge Auto button usable for RCS-resolved burns. Stock
/// hard-gates it on engine data: IsFlightComputerDisabled requires
/// Burn.BurnDuration &gt; 0 (computed from engine mass flow, always zero on
/// an engineless vehicle) and the hover tooltip claims "no engine". These
/// patches target the gauge binding itself instead of the generic
/// Vehicle.IsFlightComputerDisabled&lt;T&gt;/IsSet&lt;T&gt; helpers, which
/// Harmony cannot reliably patch.
/// </summary>
internal static class RcsGaugePatches
{
    /// <summary>The bound enum is fixed after gauge XML load, and these
    /// patches run per rendered frame; cache the reflective read per
    /// button instance.</summary>
    private sealed class BoundEnum
    {
        public Enum? Value;
    }

    private static readonly System.Runtime.CompilerServices.ConditionalWeakTable<GaugeButtonFlightComputer, BoundEnum>
        _boundEnums = new();

    private static bool IsAutoBurnButton(GaugeButtonFlightComputer button)
    {
        BoundEnum bound = _boundEnums.GetValue(button, static b => new BoundEnum
        {
            Value = GameReflection.GaugeButtonFlightComputer_enumValue?.GetValue(b) as Enum,
        });
        return bound.Value is FlightComputerBurnMode mode && mode == FlightComputerBurnMode.Auto;
    }

    private static bool ShouldOverride(out bool active)
    {
        active = false;
        Vehicle? vehicle = Program.ControlledVehicle;
        if (vehicle == null)
            return false;
        active = RcsExecutor.IsActive(vehicle);
        return active || RcsExecutor.WouldExecuteRcsCached(vehicle);
    }

    [HarmonyPatch(typeof(GaugeButtonFlightComputer), nameof(GaugeButtonFlightComputer.IsDisabled))]
    internal static class IsDisabledPatch
    {
        static void Postfix(GaugeButtonFlightComputer __instance, ref bool __result)
        {
            if (!__result || !IsAutoBurnButton(__instance))
                return;
            if (ShouldOverride(out _))
                __result = false;
        }
    }

    /// <summary>PackData drives the rendered button state: bit0 clicked,
    /// bit1 set/lit, bit2 disabled, bit3 enabled. While an RCS execution is
    /// armed or running, stock computes "disabled + unlit" (BurnMode stays
    /// Manual internally); rewrite the bits so the button reads enabled and,
    /// while running, lit.</summary>
    [HarmonyPatch(typeof(GaugeButtonFlightComputer), nameof(GaugeButtonFlightComputer.PackData))]
    internal static class PackDataPatch
    {
        static void Postfix(GaugeButtonFlightComputer __instance, ref uint2 __result)
        {
            if (!IsAutoBurnButton(__instance))
                return;
            if (!ShouldOverride(out bool active))
                return;
            uint bits = __result.X;
            bits &= ~0b0100u;
            bits |= 0b1000u;
            if (active)
                bits |= 0b0010u;
            __result = new uint2(bits, __result.Y);
        }
    }

    /// <summary>Replaces the red "no engine" hover warning with the RCS
    /// explanation whenever the button acts as the RCS trigger.</summary>
    [HarmonyPatch]
    internal static class HoveredPatch
    {
        static System.Reflection.MethodBase TargetMethod()
            => GameReflection.Vehicle_Hovered_BurnMode!;

        static bool Prefix(Vehicle __instance, FlightComputerBurnMode requestedMode)
        {
            if (requestedMode != FlightComputerBurnMode.Auto)
                return true;
            if (Program.ControlledVehicle != __instance)
                return true;
            if (RcsExecutor.IsActive(__instance))
            {
                DrawTooltip("RCS burn running - click to cancel");
                return false;
            }
            if (RcsExecutor.WouldExecuteRcsCached(__instance))
            {
                DrawTooltip("Execute burn with RCS");
                return false;
            }
            return true;
        }

        private static void DrawTooltip(string text)
        {
            try
            {
                ImGuiHelper.DrawTooltip(text, ImColor8.White);
            }
            catch (Exception ex)
            {
                // Cosmetic, but a silent swallow would hide a signature
                // drift after the prefix already suppressed stock's tooltip.
                LogHelper.WarnOnce("rcs-gauge-tooltip",
                    $"[AFC] RCS gauge tooltip failed to draw: {ex.Message}");
            }
        }
    }
}

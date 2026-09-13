#nullable disable

using System;
using AdvancedFlightComputer.Core;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance.Upfg;

/// <summary>
/// Paces a burning solid motor at its live mass flow in the staging drain simulation.
///
/// <c>SequencePerformanceList.ComputeSolidPacingMassFlowRate</c> paces a solid at the usable grain
/// left divided by the full grain's burn time, and scales its thrust by the same ratio. At ignition
/// that is the design figure. Part-way through the burn it is not, because the remaining grain
/// shrinks while the divisor stays the whole burn, so the model always gives the motor its full
/// burn time again and a nearly spent booster is modelled as a trickle that outlasts the core. The
/// game's staging window shows those figures and the ascent stage model plans against them. With
/// the postfix the pacing is the flow the nozzles report right now, so the remaining burn is the
/// grain left over the live flow and the phases end where the boosters do.
///
/// A motor that is lit but not yet flowing, or not lit at all, keeps the game's own pacing.
/// </summary>
internal static class SolidPacingPatch
{
    /// <summary>True while the postfix is installed, so the adapter's own correction stands down.</summary>
    internal static bool Active { get; private set; }

    internal static void Apply(Harmony harmony)
    {
        harmony.Patch(GameReflection.SequencePerformanceList_ComputeSolidPacingMassFlowRate!,
            postfix: new HarmonyMethod(typeof(SolidPacingPatch), nameof(Postfix)));
        Active = true;
    }

    internal static void Disable() => Active = false;

    // The span argument of the original is not needed, so it is not declared.
    private static void Postfix(SolidMotor solid, ref float __result)
    {
        try
        {
            if (TryLiveMassFlow(solid, out float flow))
                __result = flow;
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"solid-pacing:{ex.GetType().Name}",
                $"[AFC] Solid pacing read failed, the game's own pacing stands: {ex}");
        }
    }

    /// <summary>The mass flow the motor's nozzles report right now, or false while it does not flow.</summary>
    internal static bool TryLiveMassFlow(SolidMotor solid, out float flow)
    {
        flow = 0f;
        Rocket rocket = solid?.Rocket;
        PartTree tree = rocket?.Parent?.FullPart?.Tree;
        if (tree?.RocketNozzles == null || rocket.Nozzles == null)
            return false;

        double total = 0.0;
        foreach (var nozzle in tree.RocketNozzles.GetModulesAndStates(rocket.Nozzles.AsSpan()))
            total += nozzle.State.Performance.MassFlowRate;
        if (!(total > 0.0))
            return false;

        flow = (float)total;
        return true;
    }
}

using System.Runtime.CompilerServices;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Worker-side half of the RCS translation executor: a postfix on
/// <see cref="FlightComputer.ComputeControl"/> that runs after the stock
/// attitude/engine logic each flight-computer tick. It consumes the
/// command the main-thread driver published (keyed by the shared BurnPlan
/// reference) and turns the burn's remaining delta-V into per-thruster
/// pulse times, the same actuator channel stock's SelectJetsToFire uses.
///
/// Runs on vehicle-update worker threads at up to the flight-computer
/// rate: no allocations, no locks, reads one immutable command reference.
/// </summary>
[HarmonyPatch(typeof(FlightComputer), nameof(FlightComputer.ComputeControl))]
internal static class RcsComputeControlPatch
{
    static void Postfix(FlightComputer __instance, ref FlightComputerNavigation nav, ref FlightComputerOutput outputs)
    {
        // Commands exist only while an execution is running: the driver
        // publishes on activation and clears on completion/cancel. Merely
        // RCS-resolving burns publish nothing, so engine burns keep their
        // stock timing marks untouched.
        if (!RcsCommandChannel.TryGet(__instance.BurnPlan, out RcsWorkerCommand cmd) || !cmd.Active)
            return;

        // Engines must never fire while the RCS execution runs, whatever
        // flips BurnMode meanwhile (mods writing the field directly, e.g. a
        // multi-pass re-engage racing a still-active RCS burn).
        ZeroEngineCommands(ref outputs);

        BurnTarget? bt = __instance.Burn;
        if (bt == null)
            return;

        float3 togo = bt.DeltaVToGoCci;
        float3 impulse = float3.Pack(
            double3.Unpack(togo).Transform(nav.Body2Cci.Inverse())) * __instance.TotalMassPropsBody.Mass;

        // Mirror RCS timing into the shared BurnTarget: stock recomputes
        // BurnDuration/IgnitionTime from engine mass flow every tick, which
        // is meaningless for an RCS burn, and the countdown / warp-to-burn
        // UI reads these fields. Like stock, BurnDuration is the REMAINING
        // burn time derived from the current to-go, so the display counts
        // down instead of freezing at the total. The LP model only applies
        // while its pattern could actually fire: during an align slew the
        // projection onto the stale solve direction decays with the
        // rotating body frame (a countdown for a burn that is not firing),
        // and after staging the pattern no longer maps onto the thruster
        // list; both fall back to the stable group model.
        bool lpUsable = cmd.LpSecondsPerImpulse != null
            && cmd.LpSecondsPerImpulse.Length == __instance.VehicleConfig.Thrusters.Count
            && (!cmd.RequireAttitude || !RcsExecutor.OutsideAlignGate(__instance));
        bt.BurnDuration = RemainingDurationSec(cmd, impulse, lpUsable);
        bt.IgnitionTime = new SimTime(cmd.IgnitionTimeSec);

        double toIgnition = cmd.IgnitionTimeSec - nav.Time.Seconds();
        if (toIgnition > 0.0)
        {
            outputs.NextWakeupDeltaTime = Math.Min(outputs.NextWakeupDeltaTime, toIgnition);
            return;
        }

        if (float3.Dot(togo, bt.DeltaVTargetCci) <= 0f)
            return;

        if (cmd.RequireAttitude && RcsExecutor.OutsideAlignGate(__instance))
        {
            outputs.NextWakeupDeltaTime = Math.Min(outputs.NextWakeupDeltaTime, RcsExecutor.MaxPulseSec);
            return;
        }

        // Committed pulses also stamp LastThrustTime. Stock stamps it at the end
        // of ComputeControl, before this postfix fires anything, and only for
        // its own engine command or its own translation pulses, neither of
        // which an RCS execution produces, so without the stamp here
        // Vehicle.ShouldForceMarkerLabels never treats an RCS burn as thrust
        // and the forced orbit marker labels stay hidden in vacuum. Only
        // translation pulses count, matching the split stock draws with
        // ManualTranslationFired; the align slew returns above without firing.

        // LP pattern path: fire every thruster of the solved pattern for
        // its share of the demanded impulse. The length guard covers
        // staging swapping VehicleConfig between driver ticks; the group
        // path below stays the fallback whenever no solution is published.
        float[]? lp = cmd.LpSecondsPerImpulse;
        if (lp != null && lp.Length == __instance.VehicleConfig.Thrusters.Count)
        {
            if (FireLpPattern(__instance, ref outputs, cmd, lp, impulse))
                __instance.LastThrustTime = nav.Time;
            return;
        }

        impulse.X = ShapeAxis(impulse.X, cmd.AxisForcePos.X, cmd.AxisForceNeg.X,
            cmd.AxisMinImpulsePos.X, cmd.AxisMinImpulseNeg.X, cmd.MaxPulseSec);
        impulse.Y = ShapeAxis(impulse.Y, cmd.AxisForcePos.Y, cmd.AxisForceNeg.Y,
            cmd.AxisMinImpulsePos.Y, cmd.AxisMinImpulseNeg.Y, cmd.MaxPulseSec);
        impulse.Z = ShapeAxis(impulse.Z, cmd.AxisForcePos.Z, cmd.AxisForceNeg.Z,
            cmd.AxisMinImpulsePos.Z, cmd.AxisMinImpulseNeg.Z, cmd.MaxPulseSec);
        if (impulse.IsExactlyZero())
        {
            outputs.NextWakeupDeltaTime = Math.Min(outputs.NextWakeupDeltaTime, RcsExecutor.MaxPulseSec);
            return;
        }

        // Uniform pulse per signed axis group: every thruster whose intended
        // force serves a commanded direction fires for J_axis / F_group. A
        // thruster serving several commanded axes fires for the longest of
        // its groups (summing would double-deliver on the shared axes; the
        // closed loop absorbs the remaining cross-feed either way). Not the
        // stock ForceFraction dot product on purpose: those fractions divide
        // by the conservative two-sided authority and zero out on one-sided
        // layouts.
        float minCommanded = float.PositiveInfinity;
        var enumerator = outputs.Thrusters
            .GetModulesAndNewStates(__instance.VehicleConfig.Thrusters.AsSpan()).GetEnumerator();
        while (enumerator.MoveNext())
        {
            var current = enumerator.Current;
            ref ThrusterControllerState state = ref current.State;
            if (!state.IsPropellantAvailable)
                continue;

            float pulse = 0f;
            pulse = MaxAxisPulse(pulse, impulse.X, state.IntendedForce.X, cmd.AxisForcePos.X, cmd.AxisForceNeg.X);
            pulse = MaxAxisPulse(pulse, impulse.Y, state.IntendedForce.Y, cmd.AxisForcePos.Y, cmd.AxisForceNeg.Y);
            pulse = MaxAxisPulse(pulse, impulse.Z, state.IntendedForce.Z, cmd.AxisForcePos.Z, cmd.AxisForceNeg.Z);
            if (pulse <= 0f)
                continue;

            if (pulse > state.CommandPulseTime)
                state.CommandPulseTime = pulse;
            minCommanded = Math.Min(minCommanded, pulse);
            outputs.AnyActuatorCommanded = true;
        }
        if (float.IsFinite(minCommanded))
        {
            outputs.NextWakeupDeltaTime = Math.Min(
                outputs.NextWakeupDeltaTime, Math.Min(RcsExecutor.MaxPulseSec, minCommanded));
            __instance.LastThrustTime = nav.Time;
        }
    }

    /// <summary>Remaining burn time for the current residual impulse: the
    /// LP pattern's throughput cap, or for groups the slowest demanded
    /// axis (axes fire in parallel, so the longest one bounds the burn).
    /// Conservative on purpose: minimum-pulse round-up on small pattern
    /// members delivers slightly faster than modeled, so the countdown
    /// converges to zero a bit early rather than hanging past it.</summary>
    internal static float RemainingDurationSec(RcsWorkerCommand cmd, float3 impulseBody, bool lpUsable)
    {
        if (lpUsable && cmd.LpSecondsPerImpulse != null && cmd.LpImpulseCapNs > 0f)
        {
            float j = Math.Max(float3.Dot(impulseBody, cmd.LpDirBody), 0f);
            return j * cmd.MaxPulseSec / cmd.LpImpulseCapNs;
        }
        float duration = 0f;
        duration = Math.Max(duration, AxisDurationSec(
            impulseBody.X, cmd.AxisForcePos.X, cmd.AxisForceNeg.X));
        duration = Math.Max(duration, AxisDurationSec(
            impulseBody.Y, cmd.AxisForcePos.Y, cmd.AxisForceNeg.Y));
        duration = Math.Max(duration, AxisDurationSec(
            impulseBody.Z, cmd.AxisForcePos.Z, cmd.AxisForceNeg.Z));
        return duration;
    }

    private static float AxisDurationSec(float j, float forcePos, float forceNeg)
    {
        float force = j >= 0f ? forcePos : forceNeg;
        return force > 0f ? Math.Abs(j) / force : 0f;
    }

    /// <summary>Returns true when at least one pulse was committed.</summary>
    private static bool FireLpPattern(
        FlightComputer fc, ref FlightComputerOutput outputs, RcsWorkerCommand cmd,
        float[] secondsPerImpulse, float3 impulseBody)
    {
        // The pattern is valid along its solved direction only; project the
        // remaining impulse onto it and let the driver re-solve when the
        // direction drifts. A negative projection fires nothing.
        float j = float3.Dot(impulseBody, cmd.LpDirBody);
        j = Math.Min(j, cmd.LpImpulseCapNs);
        if (j <= 0f)
        {
            outputs.NextWakeupDeltaTime = Math.Min(outputs.NextWakeupDeltaTime, RcsExecutor.MaxPulseSec);
            return false;
        }

        float minCommanded = float.PositiveInfinity;
        int idx = 0;
        var enumerator = outputs.Thrusters
            .GetModulesAndNewStates(fc.VehicleConfig.Thrusters.AsSpan()).GetEnumerator();
        while (enumerator.MoveNext())
        {
            var current = enumerator.Current;
            float pulse = secondsPerImpulse[idx] * j;
            idx++;
            if (pulse <= 0f || !current.State.IsPropellantAvailable)
                continue;
            // Below its floor the core would clamp the pulse UP to the
            // minimum and overdeliver; drop the thruster and let the closed
            // loop absorb the pattern error.
            if (pulse < RcsExecutor.MinImpulseSuppressionFactor * current.Module.MinimumPulseTime)
                continue;
            if (pulse > current.State.CommandPulseTime)
                current.State.CommandPulseTime = pulse;
            minCommanded = Math.Min(minCommanded, pulse);
            outputs.AnyActuatorCommanded = true;
        }
        if (float.IsFinite(minCommanded))
        {
            outputs.NextWakeupDeltaTime = Math.Min(
                outputs.NextWakeupDeltaTime, Math.Min(RcsExecutor.MaxPulseSec, minCommanded));
            return true;
        }
        outputs.NextWakeupDeltaTime = Math.Min(outputs.NextWakeupDeltaTime, RcsExecutor.MaxPulseSec);
        return false;
    }

    /// <summary>Caps one axis of the impulse demand to a single control
    /// period and suppresses commands below half the group's minimum
    /// impulse, where a pulse would overshoot more than it corrects.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float ShapeAxis(
        float j, float forcePos, float forceNeg, float minImpPos, float minImpNeg, float maxPulse)
    {
        if (j > 0f)
        {
            if (forcePos <= 0f || j < RcsExecutor.MinImpulseSuppressionFactor * minImpPos)
                return 0f;
            return Math.Min(j, forcePos * maxPulse);
        }
        if (j < 0f)
        {
            if (forceNeg <= 0f || -j < RcsExecutor.MinImpulseSuppressionFactor * minImpNeg)
                return 0f;
            return Math.Max(j, -forceNeg * maxPulse);
        }
        return 0f;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float MaxAxisPulse(
        float pulse, float j, float thrusterForce, float groupForcePos, float groupForceNeg)
    {
        if (j > 0f && thrusterForce > 0f && groupForcePos > 0f)
            return Math.Max(pulse, j / groupForcePos);
        if (j < 0f && thrusterForce < 0f && groupForceNeg > 0f)
            return Math.Max(pulse, -j / groupForceNeg);
        return pulse;
    }

    private static void ZeroEngineCommands(ref FlightComputerOutput outputs)
    {
        var enumerator = outputs.Engines.GetModulesAndNewStates().GetEnumerator();
        while (enumerator.MoveNext())
        {
            var current = enumerator.Current;
            if (current.State.CommandThrottle != 0f || current.State.CommandBurnTime != 0.0)
            {
                current.State.CommandThrottle = 0f;
                current.State.CommandBurnTime = 0.0;
            }
        }
    }
}

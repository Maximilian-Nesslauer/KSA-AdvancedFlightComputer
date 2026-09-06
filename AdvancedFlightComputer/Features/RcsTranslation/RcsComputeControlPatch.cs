using System.Runtime.CompilerServices;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>This postfix runs on the vehicle worker after FlightComputer.ComputeControl. It reads the published command and converts the remaining delta V into thruster pulses without allocating memory or taking locks.</summary>
[HarmonyPatch(typeof(FlightComputer), nameof(FlightComputer.ComputeControl))]
internal static class RcsComputeControlPatch
{
    static void Postfix(FlightComputer __instance, ref FlightComputerNavigation nav, ref FlightComputerOutput outputs)
    {
        // Only active executions may override stock engine commands and burn timing.
        if (!RcsCommandChannel.TryGet(__instance.BurnPlan, out RcsWorkerCommand cmd) || !cmd.Active)
            return;

        // Suppress engine commands even if another caller changed BurnMode directly.
        ZeroEngineCommands(ref outputs);

        BurnTarget? bt = __instance.Burn;
        if (bt == null)
            return;

        float3 togo = bt.DeltaVToGoCci;
        float3 impulse = float3.Pack(
            double3.Unpack(togo).Transform(doubleQuat.Concatenate(nav.Ctrl2Body, nav.Body2Cci).Inverse()))
            * __instance.TotalMassPropsBody.Mass;

        // Stock UpdateBurnTarget writes engine timing every tick. Replace it with remaining RCS duration. Stale LP geometry uses the group model.
        bool lpUsable = cmd.LpSecondsPerImpulse != null
            && cmd.LpSecondsPerImpulse.Length == __instance.VehicleConfig.Thrusters.Count
            && (!cmd.RequireAttitude || !RcsExecutor.OutsideAlignGate(__instance));
        bt.BurnDuration = RemainingDurationSec(cmd, impulse, lpUsable);
        bt.IgnitionTime = cmd.IgnitionTime;

        // Mirror timing before this gate. FlightComputer.UpdateBurnTarget otherwise leaves engine timing on a disabled RCS tick.
        if (__instance.RCSMode != FlightComputerRCSMode.Enabled)
            return;

        double toIgnition = (cmd.IgnitionTime - nav.Time).Seconds();
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

        // Stock updates the thrust timestamp before this postfix, so record any translation pulses committed here. A changed thruster count invalidates the LP pattern and falls back to groups.
        float[]? lp = cmd.LpSecondsPerImpulse;
        if (lp != null && lp.Length == __instance.VehicleConfig.Thrusters.Count)
        {
            if (FireLpPattern(__instance, ref outputs, cmd, lp, impulse))
                __instance.LastThrustTime = nav.Time;
            return;
        }

        if (FireGroups(__instance, ref outputs, cmd, impulse))
            __instance.LastThrustTime = nav.Time;
    }

    private static bool FireGroups(
        FlightComputer fc, ref FlightComputerOutput outputs, RcsWorkerCommand cmd, float3 impulse)
    {
        impulse.X = ShapeAxis(impulse.X, cmd.AxisForcePos.X, cmd.AxisForceNeg.X,
            cmd.AxisMinImpulsePos.X, cmd.AxisMinImpulseNeg.X, cmd.MaxPulseSec);
        impulse.Y = ShapeAxis(impulse.Y, cmd.AxisForcePos.Y, cmd.AxisForceNeg.Y,
            cmd.AxisMinImpulsePos.Y, cmd.AxisMinImpulseNeg.Y, cmd.MaxPulseSec);
        impulse.Z = ShapeAxis(impulse.Z, cmd.AxisForcePos.Z, cmd.AxisForceNeg.Z,
            cmd.AxisMinImpulsePos.Z, cmd.AxisMinImpulseNeg.Z, cmd.MaxPulseSec);
        if (impulse.IsExactlyZero())
        {
            outputs.NextWakeupDeltaTime = Math.Min(outputs.NextWakeupDeltaTime, RcsExecutor.MaxPulseSec);
            return false;
        }

        // A thruster serving several groups takes their longest pulse, not their sum. Group forces support one sided layouts.
        float minCommanded = float.PositiveInfinity;
        var enumerator = outputs.Thrusters
            .GetModulesAndNewStates(fc.VehicleConfig.Thrusters.AsSpan()).GetEnumerator();
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
            return true;
        }
        return false;
    }

    /// <summary>The LP duration is limited by the busiest thruster. Group duration is limited by the slowest required axis because groups fire in parallel. Minimum pulse rounding can make the actual delivery slightly faster than the estimate.</summary>
    internal static float RemainingDurationSec(RcsWorkerCommand cmd, float3 impulseCtrl, bool lpUsable)
    {
        if (lpUsable && cmd.LpSecondsPerImpulse != null && cmd.LpImpulseCapNs > 0f)
        {
            float j = Math.Max(float3.Dot(impulseCtrl, cmd.LpDirCtrl), 0f);
            return j * cmd.MaxPulseSec / cmd.LpImpulseCapNs;
        }
        float duration = 0f;
        duration = Math.Max(duration, AxisDurationSec(
            impulseCtrl.X, cmd.AxisForcePos.X, cmd.AxisForceNeg.X));
        duration = Math.Max(duration, AxisDurationSec(
            impulseCtrl.Y, cmd.AxisForcePos.Y, cmd.AxisForceNeg.Y));
        duration = Math.Max(duration, AxisDurationSec(
            impulseCtrl.Z, cmd.AxisForcePos.Z, cmd.AxisForceNeg.Z));
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
        float[] secondsPerImpulse, float3 impulseCtrl)
    {
        // Project onto the solved direction. A negative projection cannot fire this pattern.
        float j = float3.Dot(impulseCtrl, cmd.LpDirCtrl);
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
            // Drop pulses below the floor because the core would round them up and overdeliver.
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

    /// <summary>Limit each pulse to one control period. Suppress an impulse below half the group minimum because firing would overshoot more than it corrects.</summary>
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

using System.Buffers;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>Signed control-frame groups use the order +X, -X, +Y, -Y, +Z, -Z. ThrusterController.ComputeControlMap uses +X forward, +Y right, and +Z down.</summary>
internal struct RcsAxisGroup
{
    public float ForceN;
    public float MassFlowKgS;
    public float MinImpulseNs;

    /// <summary>The smallest target impulse that makes a common group pulse reduce the residual after each core applies its own minimum pulse time.</summary>
    public float MinCorrectingImpulseNs;

    /// <summary>Net torque in N m in the control frame when this group fires at ForceN. Off-center thrusters require attitude control to counter it.</summary>
    public float3 TorqueNm;

    public readonly bool IsUsable
        => float.IsFinite(ForceN) && ForceN > 0f
           && float.IsFinite(MassFlowKgS) && MassFlowKgS > 0f
           && float.IsFinite(MinCorrectingImpulseNs);

    /// <summary>Effective exhaust speed along the group axis in m/s. Off-axis force consumes propellant without adding axis delta V.</summary>
    public readonly float AxisVeMs => MassFlowKgS > 0f ? ForceN / MassFlowKgS : 0f;
}

internal struct RcsSharedContribution
{
    public float MassFlowKgS;
    public float3 TorqueNm;
}

internal struct RcsPulseContribution : IComparable<RcsPulseContribution>
{
    public int GroupIndex;
    public float ForceN;
    public float MinimumPulseTimeSec;

    public int CompareTo(RcsPulseContribution other)
    {
        int groupOrder = GroupIndex.CompareTo(other.GroupIndex);
        return groupOrder != 0
            ? groupOrder
            : MinimumPulseTimeSec.CompareTo(other.MinimumPulseTimeSec);
    }
}

// Base 3 digits identify each signed axis as absent, positive, or negative.
// Thrusters with the same membership receive the same maximum axis pulse, so their flow and torque can share a bucket.
[InlineArray(27)]
internal struct RcsSharedContributions
{
    private RcsSharedContribution _first;
}

internal struct RcsCapabilitySnapshot
{
    public bool HasAnyTranslation;

    // Store only membership for multiple axes. Inline storage avoids allocations when the capability is probed each tick.
    public RcsSharedContributions SharedContributions;

    /// <summary>Use the cached control frame so membership and live magnitudes refer to the same axes until Rocket.UpdateThrusterCache catches a control-point change.</summary>
    public floatQuat Ctrl2Body;

    /// <summary>Indexed +X, -X, +Y, -Y, +Z, -Z.</summary>
    public RcsAxisGroup Ax0, Ax1, Ax2, Ax3, Ax4, Ax5;

    /// <summary>Combined mass flow in kg/s for both signs of each rotation axis. Used for slew estimates and torque cost.</summary>
    public float3 RotationMassFlowKgS;

    /// <summary>Combined live torque magnitude in N m for the same rotation thrusters. Flow divided by torque prices residual angular impulse.</summary>
    public float3 RotationTorqueNm;

    public RcsAxisGroup Get(int idx) => idx switch
    {
        0 => Ax0, 1 => Ax1, 2 => Ax2, 3 => Ax3, 4 => Ax4, _ => Ax5,
    };

    public void Set(int idx, in RcsAxisGroup g)
    {
        switch (idx)
        {
            case 0: Ax0 = g; break;
            case 1: Ax1 = g; break;
            case 2: Ax2 = g; break;
            case 3: Ax3 = g; break;
            case 4: Ax4 = g; break;
            default: Ax5 = g; break;
        }
    }

    public static float3 AxisDirection(int idx) => idx switch
    {
        0 => new float3(1f, 0f, 0f),
        1 => new float3(-1f, 0f, 0f),
        2 => new float3(0f, 1f, 0f),
        3 => new float3(0f, -1f, 0f),
        4 => new float3(0f, 0f, 1f),
        _ => new float3(0f, 0f, -1f),
    };

    public int BestAxis()
    {
        int best = -1;
        float bestForce = 0f;
        for (int i = 0; i < 6; i++)
        {
            RcsAxisGroup g = Get(i);
            if (g.IsUsable && g.ForceN > bestForce)
            {
                bestForce = g.ForceN;
                best = i;
            }
        }
        return best;
    }
}

internal static class RcsCapability
{
    /// <summary>Membership follows cached intended-action signs. Recompute force and mass flow together because ThrusterControllerGlobalState.IsCacheValid tolerates pressure and mass drift.</summary>
    public static RcsCapabilitySnapshot Probe(Vehicle vehicle)
    {
        RcsCapabilitySnapshot snap = default;
        if (!ModuleStateful<ThrusterController, ThrusterControllerState, ThrusterControllerGlobalState, EmptyStruct>
                .TryGetFrom(vehicle.Parts.States, out var stateList))
            return snap;

        ReadOnlySpan<RocketCoreState> coreStates = vehicle.Parts.RocketCores.States;
        float3 com = vehicle.TotalMassPropsAsmb.Offset;
        float ambientPressure = vehicle.PhysicsEnvironment.AtmosphericPressure;
        RcsCtrlFrame ctrl = new(stateList.GlobalState.CachedCtrl2Body);
        snap.Ctrl2Body = ctrl.Ctrl2Body;

        // Use the same module list for the buffer size and the fill pass.
        Span<ThrusterController> thrusters = stateList.Modules;
        int contributionCapacity = 0;
        for (int i = 0; i < thrusters.Length; i++)
            contributionCapacity += thrusters[i].Cores.Length * 3;
        RcsPulseContribution[] contributionBuffer =
            ArrayPool<RcsPulseContribution>.Shared.Rent(contributionCapacity);
        int contributionCount = 0;

        try
        {
            var enumerator = new ModuleStateful<ThrusterController, ThrusterControllerState, ThrusterControllerGlobalState, EmptyStruct>
                .StateList.ModuleAndStateEnumerator(stateList);
            while (enumerator.MoveNext())
            {
                var current = enumerator.Current;
                ThrusterController thruster = current.Module;
                ref readonly ThrusterControllerState state = ref current.State;
                if (!thruster.IsActive || !state.IsPropellantAvailable)
                    continue;

                int contributionStart = contributionCount;
                float3 forceAsmb = float3.Zero;
                float3 torqueAsmb = float3.Zero;
                float massFlow = 0f;
                foreach (RocketCore core in thruster.Cores)
                {
                    if (!coreStates[core.StatesIdx].IsPropellantAvailable)
                        continue;
                    RcsWrenchTable.ComputeLiveCoreAsmb(core, com, ambientPressure,
                        out float3 coreForce, out float3 coreTorque, out float coreMassFlow);
                    forceAsmb += coreForce;
                    torqueAsmb += coreTorque;
                    massFlow += coreMassFlow;
                    AddCorePulseContributions(
                        contributionBuffer, ref contributionCount,
                        ctrl.ToCtrl(coreForce), core.MinimumPulseTime);
                }
                if (massFlow <= 0f)
                {
                    contributionCount = contributionStart;
                    continue;
                }

                float3 force = ctrl.ToCtrl(forceAsmb);
                float3 torque = ctrl.ToCtrl(torqueAsmb);
                AccumulateTranslation(ref snap, state.IntendedForce, force, massFlow, torque,
                    thruster.MinimumPulseTime, out int x, out int y, out int z);
                AssignPulseGroups(
                    contributionBuffer.AsSpan(contributionStart, contributionCount - contributionStart),
                    x, y, z);

                if (!state.IntendedTorque.X.IsExactlyZero())
                {
                    snap.RotationMassFlowKgS.X += massFlow;
                    snap.RotationTorqueNm.X += Math.Abs(torque.X);
                }
                if (!state.IntendedTorque.Y.IsExactlyZero())
                {
                    snap.RotationMassFlowKgS.Y += massFlow;
                    snap.RotationTorqueNm.Y += Math.Abs(torque.Y);
                }
                if (!state.IntendedTorque.Z.IsExactlyZero())
                {
                    snap.RotationMassFlowKgS.Z += massFlow;
                    snap.RotationTorqueNm.Z += Math.Abs(torque.Z);
                }
            }

            FinalizeMinimumImpulseModel(
                ref snap, contributionBuffer.AsSpan(0, contributionCount));
        }
        finally
        {
            ArrayPool<RcsPulseContribution>.Shared.Return(contributionBuffer);
        }

        for (int i = 0; i < 6; i++)
        {
            if (snap.Get(i).IsUsable)
            {
                snap.HasAnyTranslation = true;
                break;
            }
        }
        return snap;
    }

    internal static void AccumulateTranslation(ref RcsCapabilitySnapshot snap, float3 intendedForce,
        float3 liveForce, float massFlow, float3 torque, float minPulse)
    {
        AccumulateTranslation(ref snap, intendedForce, liveForce, massFlow, torque, minPulse,
            out _, out _, out _);
    }

    private static void AccumulateTranslation(ref RcsCapabilitySnapshot snap, float3 intendedForce,
        float3 liveForce, float massFlow, float3 torque, float minPulse,
        out int x, out int y, out int z)
    {
        x = AccumulateComponent(ref snap, 0, intendedForce.X, liveForce.X, massFlow, torque, minPulse);
        y = AccumulateComponent(ref snap, 2, intendedForce.Y, liveForce.Y, massFlow, torque, minPulse);
        z = AccumulateComponent(ref snap, 4, intendedForce.Z, liveForce.Z, massFlow, torque, minPulse);
        int axes = (x != 0 ? 1 : 0) + (y != 0 ? 1 : 0) + (z != 0 ? 1 : 0);
        if (axes < 2)
            return;
        ref RcsSharedContribution shared = ref snap.SharedContributions[x + 3 * y + 9 * z];
        shared.MassFlowKgS += massFlow;
        shared.TorqueNm += torque;
    }

    private static int AccumulateComponent(ref RcsCapabilitySnapshot snap, int positiveGroup,
        float intendedForce, float liveForce, float massFlow, float3 torque, float minPulse)
    {
        if (AccumulateAxis(ref snap, positiveGroup, intendedForce, liveForce, massFlow, torque, minPulse, true))
            return 1;
        return AccumulateAxis(ref snap, positiveGroup + 1, intendedForce, liveForce, massFlow, torque, minPulse, false)
            ? 2 : 0;
    }

    /// <summary>Cached and live force must agree in sign so group membership matches the worker in the cached control frame.</summary>
    private static bool AccumulateAxis(
        ref RcsCapabilitySnapshot snap, int idx, float intendedForce, float liveForce,
        float massFlow, float3 torque, float minPulse, bool positive)
    {
        if (positive ? intendedForce <= 0f : intendedForce >= 0f)
            return false;
        if (positive ? liveForce <= 0f : liveForce >= 0f)
            return false;
        float f = Math.Abs(liveForce);
        RcsAxisGroup g = snap.Get(idx);
        g.ForceN += f;
        g.MassFlowKgS += massFlow;
        g.MinImpulseNs += minPulse * f;
        // Each group contains the full member torque. Combined-axis estimates must account for shared thrusters.
        g.TorqueNm += torque;
        snap.Set(idx, g);
        return true;
    }

    private static void AddCorePulseContributions(
        Span<RcsPulseContribution> contributions, ref int count,
        float3 force, float minimumPulseTime)
    {
        contributions[count++] = new RcsPulseContribution { GroupIndex = 0, ForceN = force.X, MinimumPulseTimeSec = minimumPulseTime };
        contributions[count++] = new RcsPulseContribution { GroupIndex = 2, ForceN = force.Y, MinimumPulseTimeSec = minimumPulseTime };
        contributions[count++] = new RcsPulseContribution { GroupIndex = 4, ForceN = force.Z, MinimumPulseTimeSec = minimumPulseTime };
    }

    private static void AssignPulseGroups(
        Span<RcsPulseContribution> contributions, int x, int y, int z)
    {
        for (int i = 0; i < contributions.Length; i++)
        {
            ref RcsPulseContribution contribution = ref contributions[i];
            int sign = contribution.GroupIndex switch { 0 => x, 2 => y, _ => z };
            if (sign == 0)
            {
                contribution.GroupIndex = -1;
                continue;
            }
            contribution.GroupIndex += sign - 1;
            if (sign == 2)
                contribution.ForceN = -contribution.ForceN;
        }
    }

    internal static void FinalizeMinimumImpulseModel(
        ref RcsCapabilitySnapshot snap, Span<RcsPulseContribution> contributions)
    {
        contributions.Sort();
        int start = 0;
        for (int groupIndex = 0; groupIndex < 6; groupIndex++)
        {
            while (start < contributions.Length && contributions[start].GroupIndex < groupIndex)
                start++;
            int end = start;
            while (end < contributions.Length && contributions[end].GroupIndex == groupIndex)
                end++;

            RcsAxisGroup group = snap.Get(groupIndex);
            group.MinCorrectingImpulseNs = ComputeMinimumCorrectingImpulse(
                group.ForceN, contributions[start..end]);
            snap.Set(groupIndex, in group);
            start = end;
        }
    }

    private static float ComputeMinimumCorrectingImpulse(
        float totalForce, ReadOnlySpan<RcsPulseContribution> sortedContributions)
    {
        if (!float.IsFinite(totalForce))
            return float.PositiveInfinity;
        if (!(totalForce > 0f))
            return 0f;
        if (sortedContributions.IsEmpty)
            return float.PositiveInfinity;

        double highForce = 0.0;
        double highImpulse = 0.0;
        bool hasOpposingForce = false;
        for (int i = 0; i < sortedContributions.Length; i++)
        {
            ref readonly RcsPulseContribution contribution = ref sortedContributions[i];
            if (!float.IsFinite(contribution.ForceN)
                || !float.IsFinite(contribution.MinimumPulseTimeSec)
                || contribution.MinimumPulseTimeSec < 0f)
                return float.PositiveInfinity;
            if (contribution.ForceN < 0f)
                hasOpposingForce = true;
            highForce += contribution.ForceN;
            highImpulse += contribution.ForceN * contribution.MinimumPulseTimeSec;
        }
        if (hasOpposingForce)
        {
            double cappedImpulse = 0.0;
            for (int i = 0; i < sortedContributions.Length; i++)
            {
                ref readonly RcsPulseContribution contribution = ref sortedContributions[i];
                cappedImpulse += contribution.ForceN
                    * Math.Max(RcsExecutor.MaxPulseSec, contribution.MinimumPulseTimeSec);
            }
            // A mixed group can be nonmonotonic below the cap, so every accepted residual commands the cap and is compared with its actual rounded response.
            if (!(cappedImpulse > 0.0) || !double.IsFinite(cappedImpulse))
                return float.PositiveInfinity;
            double threshold = Math.Max(
                totalForce * RcsExecutor.MaxPulseSec,
                RcsExecutor.MinImpulseSuppressionFactor * cappedImpulse);
            return double.IsFinite(threshold) && threshold <= float.MaxValue
                ? (float)threshold
                : float.PositiveInfinity;
        }

        int next = 0;
        while (next < sortedContributions.Length)
        {
            double lowForce = totalForce - highForce;
            double duration = RcsExecutor.MinImpulseSuppressionFactor * highImpulse
                / (totalForce - RcsExecutor.MinImpulseSuppressionFactor * lowForce);
            if (duration < sortedContributions[next].MinimumPulseTimeSec)
            {
                double threshold = duration * totalForce;
                return double.IsFinite(threshold) && threshold >= 0.0 && threshold <= float.MaxValue
                    ? (float)threshold
                    : float.PositiveInfinity;
            }

            RcsPulseContribution contribution = sortedContributions[next++];
            highForce -= contribution.ForceN;
            highImpulse -= contribution.ForceN * contribution.MinimumPulseTimeSec;
        }

        return 0f;
    }
}

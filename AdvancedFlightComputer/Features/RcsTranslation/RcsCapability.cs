using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Main-thread probe of a vehicle's RCS translation capability, grouped
/// by the six signed body axes the stock ControlMap flags cover.
/// Axis order: +X, -X, +Y, -Y, +Z, -Z (body: +X forward, +Y right,
/// +Z down, matching ThrusterController.ComputeControlMap).
/// </summary>
internal struct RcsAxisGroup
{
    public float ForceN;
    public float MassFlowKgS;
    public float MinImpulseNs;

    public readonly bool IsUsable => ForceN > 0f && MassFlowKgS > 0f;

    /// <summary>Effective exhaust speed of the group along its axis. Off-axis
    /// thrust components are spent but produce no axis dV, so this is the
    /// fuel-per-axis-dV number, not the physical nozzle exhaust speed.</summary>
    public readonly float AxisVeMs => MassFlowKgS > 0f ? ForceN / MassFlowKgS : 0f;
}

internal struct RcsCapabilitySnapshot
{
    public bool HasAnyTranslation;

    /// <summary>Indexed +X,-X,+Y,-Y,+Z,-Z.</summary>
    public RcsAxisGroup Ax0, Ax1, Ax2, Ax3, Ax4, Ax5;

    /// <summary>Per rotation axis (roll, pitch, yaw): combined mass flow of the
    /// thrusters that produce torque about it, both signs. Used only for the
    /// slew-cost estimate.</summary>
    public float3 RotationMassFlowKgS;

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

    /// <summary>Signed-axis unit direction in body frame for a group index.</summary>
    public static float3 AxisDirection(int idx) => idx switch
    {
        0 => new float3(1f, 0f, 0f),
        1 => new float3(-1f, 0f, 0f),
        2 => new float3(0f, 1f, 0f),
        3 => new float3(0f, -1f, 0f),
        4 => new float3(0f, 0f, 1f),
        _ => new float3(0f, 0f, -1f),
    };

    /// <summary>Group index with the highest axis force, -1 when none is usable.</summary>
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
    /// <summary>
    /// Builds the capability snapshot from the vehicle's live thruster
    /// modules and states. Read-only over the applied state buffer, which
    /// is safe even while solver jobs are in flight: workers stage their
    /// writes into a separate new-state buffer that is applied back on the
    /// main thread.
    /// </summary>
    public static RcsCapabilitySnapshot Probe(Vehicle vehicle)
    {
        RcsCapabilitySnapshot snap = default;
        if (!ModuleStateful<ThrusterController, ThrusterControllerState, ThrusterControllerGlobalState, EmptyStruct>
                .TryGetFrom(vehicle.Parts.States, out var stateList))
            return snap;

        var enumerator = new ModuleStateful<ThrusterController, ThrusterControllerState, ThrusterControllerGlobalState, EmptyStruct>
            .StateList.ModuleAndStateEnumerator(stateList);
        while (enumerator.MoveNext())
        {
            var current = enumerator.Current;
            ThrusterController thruster = current.Module;
            ref readonly ThrusterControllerState state = ref current.State;
            if (!thruster.IsActive || !state.IsPropellantAvailable)
                continue;

            float massFlow = 0f;
            foreach (RocketCore core in thruster.Cores)
                massFlow += core.MaxConsumptionRate;

            AccumulateAxis(ref snap, 0, state.IntendedForce.X, massFlow, thruster.MinimumPulseTime, positive: true);
            AccumulateAxis(ref snap, 1, state.IntendedForce.X, massFlow, thruster.MinimumPulseTime, positive: false);
            AccumulateAxis(ref snap, 2, state.IntendedForce.Y, massFlow, thruster.MinimumPulseTime, positive: true);
            AccumulateAxis(ref snap, 3, state.IntendedForce.Y, massFlow, thruster.MinimumPulseTime, positive: false);
            AccumulateAxis(ref snap, 4, state.IntendedForce.Z, massFlow, thruster.MinimumPulseTime, positive: true);
            AccumulateAxis(ref snap, 5, state.IntendedForce.Z, massFlow, thruster.MinimumPulseTime, positive: false);

            if (!state.IntendedTorque.X.IsExactlyZero())
                snap.RotationMassFlowKgS.X += massFlow;
            if (!state.IntendedTorque.Y.IsExactlyZero())
                snap.RotationMassFlowKgS.Y += massFlow;
            if (!state.IntendedTorque.Z.IsExactlyZero())
                snap.RotationMassFlowKgS.Z += massFlow;
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

    private static void AccumulateAxis(
        ref RcsCapabilitySnapshot snap, int idx, float axisForce, float massFlow, float minPulse, bool positive)
    {
        if (positive ? axisForce <= 0f : axisForce >= 0f)
            return;
        float f = Math.Abs(axisForce);
        RcsAxisGroup g = snap.Get(idx);
        g.ForceN += f;
        g.MassFlowKgS += massFlow;
        g.MinImpulseNs += minPulse * f;
        snap.Set(idx, g);
    }
}

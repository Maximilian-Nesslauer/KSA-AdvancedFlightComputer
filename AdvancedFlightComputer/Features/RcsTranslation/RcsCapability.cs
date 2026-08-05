using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Main-thread probe of a vehicle's RCS translation capability, grouped
/// by the six signed control-frame axes the stock ControlMap flags cover.
/// Axis order: +X, -X, +Y, -Y, +Z, -Z (+X forward, +Y right, +Z down,
/// matching ThrusterController.ComputeControlMap).
/// </summary>
internal struct RcsAxisGroup
{
    public float ForceN;
    public float MassFlowKgS;
    public float MinImpulseNs;

    /// <summary>Net torque (N m, control frame) the whole group produces when it
    /// fires at the force <see cref="ForceN"/>: the sum of the member
    /// thrusters' live torques. Near zero on a balanced layout, nonzero when
    /// the group's thrusters sit off the CoM. Divided by <see cref="ForceN"/>
    /// it is the torque per unit axis force the attitude hold must null, which
    /// is what makes a Hold burn cost more than its bare translation.</summary>
    public float3 TorqueNm;

    public readonly bool IsUsable => ForceN > 0f && MassFlowKgS > 0f;

    /// <summary>Effective exhaust speed of the group along its axis. Off-axis
    /// thrust components are spent but produce no axis dV, so this is the
    /// fuel-per-axis-dV number, not the physical nozzle exhaust speed.</summary>
    public readonly float AxisVeMs => MassFlowKgS > 0f ? ForceN / MassFlowKgS : 0f;
}

internal struct RcsCapabilitySnapshot
{
    public bool HasAnyTranslation;

    /// <summary>The control frame this snapshot describes, taken from the
    /// thruster cache it was built against. A snapshot outlives a control-point
    /// change, so the driver compares this against the live cache to know when
    /// the axis groups stopped describing the frame the worker fires in.</summary>
    public floatQuat Ctrl2Body;

    /// <summary>Indexed +X,-X,+Y,-Y,+Z,-Z.</summary>
    public RcsAxisGroup Ax0, Ax1, Ax2, Ax3, Ax4, Ax5;

    /// <summary>Per rotation axis (roll, pitch, yaw): combined mass flow of the
    /// thrusters that produce torque about it, both signs. Feeds the
    /// slew-cost estimate and, together with RotationTorqueNm, the LP's
    /// torque-slack price.</summary>
    public float3 RotationMassFlowKgS;

    /// <summary>Per rotation axis: combined live torque magnitude of the
    /// same thrusters, N m. Flow over torque is what the attitude hold
    /// pays per newton-meter-second of residual angular impulse.</summary>
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

    /// <summary>Signed-axis unit direction in the control frame for a group index.</summary>
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
    ///
    /// Membership comes from the cached state (IsPropellantAvailable and
    /// the IntendedForce/IntendedTorque signs are exactly what the worker
    /// and the stock attitude control fire by), but magnitudes are
    /// recomputed live via RcsWrenchTable.ComputeLive: the game's thruster
    /// cache revalidates only on 0.1 percent mass, CoM, 100 Pa pressure drift or
    /// a control-frame change, so a cached IntendedForce can carry a different
    /// vintage than a fresh mass-flow read and misstate force per flow.
    ///
    /// The control frame comes from that same cache rather than from the live
    /// vehicle, because the sign test in AccumulateAxis pairs a cached
    /// IntendedForce component with a freshly computed one: reading the live
    /// frame would compare across frames for the tick between a control-point
    /// change and the worker refreshing the cache, and silently drop thrusters.
    /// </summary>
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

        var enumerator = new ModuleStateful<ThrusterController, ThrusterControllerState, ThrusterControllerGlobalState, EmptyStruct>
            .StateList.ModuleAndStateEnumerator(stateList);
        while (enumerator.MoveNext())
        {
            var current = enumerator.Current;
            ThrusterController thruster = current.Module;
            ref readonly ThrusterControllerState state = ref current.State;
            if (!thruster.IsActive || !state.IsPropellantAvailable)
                continue;

            RcsWrenchTable.ComputeLive(thruster, coreStates, com, ambientPressure, in ctrl,
                out float3 force, out float3 torque, out float massFlow);
            if (massFlow <= 0f)
                continue;

            AccumulateAxis(ref snap, 0, state.IntendedForce.X, force.X, massFlow, torque, thruster.MinimumPulseTime, positive: true);
            AccumulateAxis(ref snap, 1, state.IntendedForce.X, force.X, massFlow, torque, thruster.MinimumPulseTime, positive: false);
            AccumulateAxis(ref snap, 2, state.IntendedForce.Y, force.Y, massFlow, torque, thruster.MinimumPulseTime, positive: true);
            AccumulateAxis(ref snap, 3, state.IntendedForce.Y, force.Y, massFlow, torque, thruster.MinimumPulseTime, positive: false);
            AccumulateAxis(ref snap, 4, state.IntendedForce.Z, force.Z, massFlow, torque, thruster.MinimumPulseTime, positive: true);
            AccumulateAxis(ref snap, 5, state.IntendedForce.Z, force.Z, massFlow, torque, thruster.MinimumPulseTime, positive: false);

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

    /// <summary>Adds one thruster to a signed-axis group. The cached
    /// intendedForce component gates membership (matching the worker's
    /// MaxAxisPulse selection); the live component supplies the magnitude.
    /// Both must agree in sign, else the thruster is skipped for the axis.</summary>
    private static void AccumulateAxis(
        ref RcsCapabilitySnapshot snap, int idx, float intendedForce, float liveForce,
        float massFlow, float3 torque, float minPulse, bool positive)
    {
        if (positive ? intendedForce <= 0f : intendedForce >= 0f)
            return;
        if (positive ? liveForce <= 0f : liveForce >= 0f)
            return;
        float f = Math.Abs(liveForce);
        RcsAxisGroup g = snap.Get(idx);
        g.ForceN += f;
        g.MassFlowKgS += massFlow;
        g.MinImpulseNs += minPulse * f;
        // The group fires all its members at one duty, so its net torque at
        // full group force is the plain sum of the member torques. A thruster
        // serving several axes adds its torque to each group it joins - the
        // same cross-feed the force/flow accumulation already carries.
        g.TorqueNm += torque;
        snap.Set(idx, g);
    }
}

using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>Control-frame wrenches follow FlightComputer.VehicleConfig.Thrusters by index. Keep the full nozzle wrench because intended-action values omit components excluded by the stock control map. Moment arms are measured from the center of mass.</summary>
internal sealed class RcsWrenchTable
{
    public int Count;
    public float3[] ForceCtrl = Array.Empty<float3>();
    public float3[] TorqueCtrl = Array.Empty<float3>();
    public double[] MassFlow = Array.Empty<double>();
    public bool[] Usable = Array.Empty<bool>();
    public ThrusterController?[] Modules = Array.Empty<ThrusterController?>();
    public int UsableCount;
    public floatQuat Ctrl2Body = floatQuat.Identity;

    /// <summary>Compare module references because the flight-computer configuration swaps with its worker copy and staging can replace a list without changing its count.</summary>
    public bool Matches(List<ThrusterController> thrusters, in RcsCtrlFrame ctrl)
    {
        if (Count != thrusters.Count || Ctrl2Body != ctrl.Ctrl2Body)
            return false;
        for (int i = 0; i < Count; i++)
        {
            if (!ReferenceEquals(Modules[i], thrusters[i]))
                return false;
        }
        return true;
    }

    public void Build(Vehicle vehicle, FlightComputer fc, in RcsCtrlFrame ctrl)
    {
        List<ThrusterController> thrusters = fc.VehicleConfig.Thrusters;
        Count = thrusters.Count;
        if (ForceCtrl.Length < Count)
        {
            ForceCtrl = new float3[Count];
            TorqueCtrl = new float3[Count];
            MassFlow = new double[Count];
            Usable = new bool[Count];
            Modules = new ThrusterController?[Count];
        }
        UsableCount = 0;
        Ctrl2Body = ctrl.Ctrl2Body;

        ReadOnlySpan<RocketCoreState> coreStates = vehicle.Parts.RocketCores.States;
        float3 com = vehicle.TotalMassPropsAsmb.Offset;
        float ambientPressure = vehicle.PhysicsEnvironment.AtmosphericPressure;

        for (int i = 0; i < Count; i++)
        {
            ThrusterController thruster = thrusters[i];
            ComputeLive(thruster, coreStates, com, ambientPressure, in ctrl,
                out float3 force, out float3 torque, out float massFlow);

            ForceCtrl[i] = force;
            TorqueCtrl[i] = torque;
            MassFlow[i] = massFlow;
            Modules[i] = thruster;
            Usable[i] = massFlow > 0f && !force.IsExactlyZero();
            if (Usable[i])
                UsableCount++;
        }
    }

    /// <summary>Compute thrust and mass flow from the same full-throttle nozzle performance, as ThrusterController.RecomputeDynamicData does for force. Use live conditions because the stock cache tolerates pressure and mass drift.</summary>
    public static void ComputeLive(
        ThrusterController thruster, ReadOnlySpan<RocketCoreState> coreStates,
        float3 com, float ambientPressure, in RcsCtrlFrame ctrl,
        out float3 force, out float3 torque, out float massFlow)
    {
        float3 forceAsmb = float3.Zero;
        float3 torqueAsmb = float3.Zero;
        massFlow = 0f;
        if (thruster.IsActive)
        {
            foreach (RocketCore core in thruster.Cores)
            {
                if (!coreStates[core.StatesIdx].IsPropellantAvailable)
                    continue;
                ComputeLiveCoreAsmb(core, com, ambientPressure,
                    out float3 coreForce, out float3 coreTorque, out float coreMassFlow);
                forceAsmb += coreForce;
                torqueAsmb += coreTorque;
                massFlow += coreMassFlow;
            }
        }
        force = ctrl.ToCtrl(forceAsmb);
        torque = ctrl.ToCtrl(torqueAsmb);
    }

    internal static void ComputeLiveCoreAsmb(
        RocketCore core, float3 com, float ambientPressure,
        out float3 force, out float3 torque, out float massFlow)
    {
        float3 forceAsmb = float3.Zero;
        float3 torqueAsmb = float3.Zero;
        massFlow = 0f;
        // Full-throttle probe because thruster pulses always command throttle 1.
        RocketCoreConditions combustion = core.ComputeConditions(1f);
        foreach (RocketNozzle nozzle in core.Rocket.Nozzles)
        {
            float4x4 matrix = float4x4.Pack(nozzle.Parent.MatrixAsmb2VehicleAsmb);
            floatQuat rotation = floatQuat.Pack(nozzle.Parent.Asmb2VehicleAsmb);
            NozzlePerformance perf = nozzle.ComputePerformance(in combustion, ambientPressure);
            float3 nozzleForce = perf.GetTotalThrust()
                * (-nozzle.ExhaustDirectionAsmb).Transform(rotation);
            float3 offset = nozzle.LocationAsmb.Transform(matrix) - com;
            forceAsmb += nozzleForce;
            torqueAsmb += float3.Cross(offset, nozzleForce);
            massFlow += perf.MassFlowRate;
        }
        force = forceAsmb;
        torque = torqueAsmb;
    }
}

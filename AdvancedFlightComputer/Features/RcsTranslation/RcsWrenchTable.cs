using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Per-thruster wrench data for the LP allocator, index-aligned with
/// FlightComputer.VehicleConfig.Thrusters (the list the worker enumerates).
/// Built on the main thread from the raw nozzle geometry, the same math
/// ThrusterController.RecomputeDynamicData runs - deliberately NOT the
/// axis-projected IntendedForce/IntendedTorque state values, because those
/// drop every component the stock ControlMap thresholds gate out, and
/// exploiting exactly those off-axis components is the LP's point.
/// </summary>
internal sealed class RcsWrenchTable
{
    public int Count;
    public float3[] ForceBody = Array.Empty<float3>();
    public float3[] TorqueBody = Array.Empty<float3>();
    public double[] MassFlow = Array.Empty<double>();
    public bool[] Usable = Array.Empty<bool>();
    public ThrusterController?[] Modules = Array.Empty<ThrusterController?>();
    public int UsableCount;

    /// <summary>True while the table still describes the live thruster
    /// list. Compares module references because the VehicleConfig object
    /// itself ping-pongs between the vehicle's FlightComputer and its
    /// worker copy every tick, and staging can swap the list without
    /// changing its count.</summary>
    public bool Matches(List<ThrusterController> thrusters)
    {
        if (Count != thrusters.Count)
            return false;
        for (int i = 0; i < Count; i++)
        {
            if (!ReferenceEquals(Modules[i], thrusters[i]))
                return false;
        }
        return true;
    }

    public void Build(Vehicle vehicle, FlightComputer fc)
    {
        List<ThrusterController> thrusters = fc.VehicleConfig.Thrusters;
        Count = thrusters.Count;
        if (ForceBody.Length < Count)
        {
            ForceBody = new float3[Count];
            TorqueBody = new float3[Count];
            MassFlow = new double[Count];
            Usable = new bool[Count];
            Modules = new ThrusterController?[Count];
        }
        UsableCount = 0;

        ReadOnlySpan<RocketCoreState> coreStates = vehicle.Parts.RocketCores.States;
        float3 com = vehicle.TotalMassPropsAsmb.Offset;
        float ambientPressure = vehicle.PhysicsEnvironment.AtmosphericPressure;

        for (int i = 0; i < Count; i++)
        {
            ThrusterController thruster = thrusters[i];
            ComputeLive(thruster, coreStates, com, ambientPressure,
                out float3 force, out float3 torque, out float massFlow);

            ForceBody[i] = force;
            TorqueBody[i] = torque;
            MassFlow[i] = massFlow;
            Modules[i] = thruster;
            Usable[i] = massFlow > 0f && !force.IsExactlyZero();
            if (Usable[i])
                UsableCount++;
        }
    }

    /// <summary>Live wrench and mass flow of one thruster from nozzle
    /// performance at the current ambient pressure. Thrust and flow come
    /// from the SAME performance evaluation, matching the physics exactly
    /// (ActiveNozzle applies Performance.TotalThrust and consumes
    /// Performance.MassFlowRate through one ComputeThrustMod). The cached
    /// state values are not a substitute: the game's thruster cache
    /// revalidates only on 0.1 percent mass, CoM, or 100 Pa pressure drift
    /// (ThrusterControllerGlobalState.IsCacheValid), so IntendedForce can
    /// carry spawn-time conditions through a whole burn while a fresh
    /// MaxConsumptionRate does not, and force over flow then misstates the
    /// real exhaust velocity.</summary>
    public static void ComputeLive(
        ThrusterController thruster, ReadOnlySpan<RocketCoreState> coreStates,
        float3 com, float ambientPressure,
        out float3 force, out float3 torque, out float massFlow)
    {
        force = float3.Zero;
        torque = float3.Zero;
        massFlow = 0f;
        if (!thruster.IsActive)
            return;
        foreach (RocketCore core in thruster.Cores)
        {
            if (!coreStates[core.StatesIdx].IsPropellantAvailable)
                continue;
            // Full-throttle probe; thruster pulses always command throttle 1.
            RocketCoreConditions combustion = core.ComputeConditions(1f);
            foreach (RocketNozzle nozzle in core.Rocket.Nozzles)
            {
                float4x4 matrix = float4x4.Pack(nozzle.Parent.MatrixAsmb2VehicleAsmb);
                floatQuat rotation = floatQuat.Pack(nozzle.Parent.Asmb2VehicleAsmb);
                NozzlePerformance perf = nozzle.ComputePerformance(in combustion, ambientPressure);
                float3 f = perf.GetTotalThrust() * (-nozzle.ExhaustDirectionAsmb).Transform(rotation);
                float3 r = nozzle.LocationAsmb.Transform(matrix) - com;
                force += f;
                torque += float3.Cross(r, f);
                massFlow += perf.MassFlowRate;
            }
        }
    }
}

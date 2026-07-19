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
            float3 force = float3.Zero;
            float3 torque = float3.Zero;
            double massFlow = 0.0;
            bool usable = false;

            if (thruster.IsActive)
            {
                foreach (RocketCore core in thruster.Cores)
                {
                    if (!coreStates[core.StatesIdx].IsPropellantAvailable)
                        continue;
                    RocketCoreConditions combustion = core.ComputeConditions(1f);
                    foreach (RocketNozzle nozzle in core.Rocket.Nozzles)
                    {
                        float4x4 matrix = float4x4.Pack(nozzle.Parent.MatrixAsmb2VehicleAsmb);
                        floatQuat rotation = floatQuat.Pack(nozzle.Parent.Asmb2VehicleAsmb);
                        float thrust = nozzle.ComputePerformance(in combustion, ambientPressure).GetTotalThrust();
                        float3 f = thrust * (-nozzle.ExhaustDirectionAsmb).Transform(rotation);
                        float3 r = nozzle.LocationAsmb.Transform(matrix) - com;
                        force += f;
                        torque += float3.Cross(r, f);
                    }
                    massFlow += core.MaxConsumptionRate;
                    usable = true;
                }
            }

            ForceBody[i] = force;
            TorqueBody[i] = torque;
            MassFlow[i] = massFlow;
            Modules[i] = thruster;
            Usable[i] = usable && massFlow > 0.0 && !force.IsExactlyZero();
            if (Usable[i])
                UsableCount++;
        }
    }
}

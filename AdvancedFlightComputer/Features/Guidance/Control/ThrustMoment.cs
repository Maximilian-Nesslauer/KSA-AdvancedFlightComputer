#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using Brutal.Numerics;
using KSA;

/// <summary>
/// The moment a set of engines makes about the centre of mass with every gimbal at rest: zero for a cluster balanced about the centre of mass, and for an off-centre choice (one engine of a ring, a pair beside the center) the moment the gimbals must hold for as long as the engines burn.
///  IN PROPORTION TO THE THRUST. Every nozzle, the turbine exhaust of a gas generator cycle included, is taken at full throttle in vacuum, the conditions of GimbalController.Data.MaximumThrust (GimbalController.RecomputeStaticData), so the moment is on the same scale as the gimbals' capability, and the moment over the force is the moment per newton the engines make at any throttle.
///  AT REST, because the allocator is incremental from there (KsaTvcAllocator.FillColumns differentiates about zero deflection): the torque the craft gets is this moment plus what the allocator was asked for.
/// </summary>
internal static class KsaThrustMoment
{
    /// <summary>The engines' moment about the centre of mass now, N m, and their force, N, vehicle assembly frame.</summary>
    internal static (double3 Moment, double3 Force) Rest(Vehicle vehicle, IEnumerable<EngineController> engines)
    {
        float3 c = vehicle.CenterOfMassAsmbF;
        var com = new double3(c.X, c.Y, c.Z);
        var gimbalOf = new Dictionary<RocketNozzle, GimbalController>();
        foreach (GimbalController gc in vehicle.Parts.Modules.Get<GimbalController>())
            foreach (RocketNozzle nozzle in gc.Data.Nozzles)
                gimbalOf[nozzle] = gc;

        double3 moment = double3.Zero, force = double3.Zero;
        foreach (EngineController engine in engines)
            foreach (Rocket rocket in engine.Rockets)
                foreach (RocketNozzle nozzle in rocket.AllNozzles)
                {
                    if (nozzle.Core == null || nozzle.Parent is not { } part)
                        continue;
                    RocketCoreConditions combustion = nozzle.Core.ComputeConditions(1f);
                    double thrust = nozzle.ComputePerformance(in combustion, 0f).GetTotalThrust();
                    if (!double.IsFinite(thrust) || thrust <= 0.0)
                        continue;
                    double3 at, direction;
                    if (gimbalOf.TryGetValue(nozzle, out GimbalController gc))
                    {
                        at = KsaTvcAllocator.ThrustLocation(gc, 0.0, 0.0);
                        direction = KsaTvcAllocator.ThrustDirection(gc, 0.0, 0.0);
                    }
                    else
                    {
                        float3 p = nozzle.LocationAsmb.Transform(float4x4.Pack(part.MatrixAsmb2VehicleAsmb));
                        float3 d = (-nozzle.ExhaustDirectionAsmb).Transform(floatQuat.Pack(part.Asmb2VehicleAsmb));
                        at = new double3(p.X, p.Y, p.Z);
                        direction = new double3(d.X, d.Y, d.Z);
                    }
                    double length = direction.Length();
                    if (!(length > 0.0))
                        continue;
                    double3 f = direction * (thrust / length);
                    moment += double3.Cross(at - com, f);
                    force += f;
                }
        return (moment, force);
    }

    /// <summary>
    /// The engines' moment across their thrust axis, N m, and the most the gimbals that move them can hold against it, N m: the allocator's deliverable torque in the opposing direction (as Ksa6DofSetup.TorqueAlong measures the plan's capability), both at the thrust of Data.MaximumThrust. Zero capability for engines on no gimbal.
    /// </summary>
    internal static (double LateralNm, double CapabilityNm) LateralAgainstGimbals(Vehicle vehicle, IReadOnlyCollection<EngineController> engines)
    {
        (double3 moment, double3 force) = Rest(vehicle, engines);
        double fLength = force.Length();
        if (!(fLength > 0.0))
            return (0.0, 0.0);
        double3 axis = force / fLength;
        double3 lateral = moment - axis * double3.Dot(moment, axis);
        double lateralNm = lateral.Length();
        if (!(lateralNm > 1e-6))
            return (0.0, double.PositiveInfinity);

        var gimbals = new List<GimbalController>();
        foreach (GimbalController gc in vehicle.Parts.Modules.Get<GimbalController>())
            foreach (RocketNozzle nozzle in gc.Data.Nozzles)
                if (nozzle.Rocket?.Controller is EngineController owner && Contains(engines, owner))
                {
                    gimbals.Add(gc);
                    break;
                }
        if (gimbals.Count == 0)
            return (lateralNm, 0.0);

        var thrusts = new double[gimbals.Count];
        double total = 0.0;
        for (int i = 0; i < gimbals.Count; i++)
        {
            thrusts[i] = gimbals[i].Data.MaximumThrust;
            total += thrusts[i];
        }
        double3 against = -lateral / lateralNm;
        var commands = new double[2 * gimbals.Count];
        // Far beyond anything the gimbals can give, so what comes back along the direction is the saturation point.
        TvcAllocationResult a = KsaTvcAllocator.Solve(gimbals.ToArray(), thrusts, vehicle.CenterOfMassAsmbF,
                                                      against * (Math.Max(total, 1.0) * 1000.0), commands);
        return (lateralNm, Math.Max(double3.Dot(a.AchievedTorque, against), 0.0));
    }

    private static bool Contains(IReadOnlyCollection<EngineController> engines, EngineController engine)
    {
        foreach (EngineController e in engines)
            if (ReferenceEquals(e, engine))
                return true;
        return false;
    }
}

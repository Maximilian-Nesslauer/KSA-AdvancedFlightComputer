#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using KSA;

// FlightComputer.VehicleConfigInfo has no main-engine totals, so this adapter calculates them.
internal static class KsaEnginePerf
{
    // Configuration totals include inactive engines and do not depend on fuel supply.
    // Thrust is in N and mass flow is in kg/s.
    internal static (double thrust, double massFlow) Vacuum(Vehicle vehicle)
    {
        if (vehicle?.Parts == null)
            return (0.0, 0.0);
        double thrust = 0.0, massFlow = 0.0;
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
        {
            thrust += engine.VacuumData.ThrustMax.Length();
            massFlow += engine.VacuumData.MassFlowRateMax;
        }
        return (thrust, massFlow);
    }

    internal static double VacuumThrust(Vehicle vehicle) => Vacuum(vehicle).thrust;

    internal static double VacuumExhaustVelocity(Vehicle vehicle)
    {
        (double thrust, double massFlow) = Vacuum(vehicle);
        return massFlow > 0.0 ? thrust / massFlow : 0.0;
    }

    // Full-throttle capability includes active, supplied engines even when EngineOn is false.
    internal static (double thrust, double massFlow) AtPressure(Vehicle vehicle, double ambientPressure)
        => ActivePerformance(vehicle, 1.0, ambientPressure);

    internal static double ActiveThrustCapability(Vehicle vehicle, double ambientPressure)
        => AtPressure(vehicle, ambientPressure).thrust;

    internal static double ThrustAtThrottle(Vehicle vehicle, double throttle, double ambientPressure)
        => ActivePerformance(vehicle, throttle, ambientPressure).thrust;

    private static (double thrust, double massFlow) ActivePerformance(Vehicle vehicle, double throttle, double ambientPressure)
    {
        if (vehicle?.Parts?.States == null || !double.IsFinite(throttle) || !double.IsFinite(ambientPressure)
            || !ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
                .TryGetFrom(vehicle.Parts.States, out var states)
            || !ModuleStateful<RocketCore, RocketCoreState, RocketCoreGlobalState, EmptyStruct>
                .TryGetFrom(vehicle.Parts.States, out var coreStates))
            return (0.0, 0.0);

        // Vehicle.PrepareWorker applies the vehicle floor. ComputeActivePerformance then applies each core's floor through RocketControllerData.ComputeFromCores.
        float command = (float)Math.Clamp(throttle, vehicle.GetMinThrottle(), 1.0);
        float pressure = (float)Math.Clamp(ambientPressure, 0.0, float.MaxValue);
        double thrust = 0.0, massFlow = 0.0;
        foreach (var engine in states.ModulesAndStates)
        {
            if (!engine.Module.IsActive || !engine.State.IsPropellantAvailable
                || engine.Module.Cores == null || engine.Module.Cores.Length == 0)
                continue;
            RocketCore[] cores = engine.Module.Cores;
            bool allSupplied = true;
            foreach (RocketCore core in cores)
                allSupplied &= coreStates.States[core.StatesIdx].IsPropellantAvailable;
            RocketControllerData data;
            if (allSupplied)
                data = engine.Module.ComputeActivePerformance(in engine.State, vehicle.TotalMassPropsAsmb.Offset, pressure, command);
            else
            {
                // Rocket.UpdateRockets marks a controller supplied when any core has fuel.
                // Sum only supplied cores, while preserving the controller's vector sum.
                data = RocketControllerData.Zero;
                for (int i = 0; i < cores.Length; i++)
                {
                    if (!coreStates.States[cores[i].StatesIdx].IsPropellantAvailable)
                        continue;
                    RocketControllerData coreData = RocketControllerData.ComputeFromCores(
                        cores.AsSpan(i, 1), vehicle.TotalMassPropsAsmb.Offset, pressure, command);
                    data.ThrustMax += coreData.ThrustMax;
                    data.MassFlowRateMax += coreData.MassFlowRateMax;
                }
            }
            double force = data.ThrustMax.Length();
            if (!double.IsFinite(force) || !float.IsFinite(data.MassFlowRateMax))
                return (0.0, 0.0);
            thrust += force;
            massFlow += data.MassFlowRateMax;
        }
        return (thrust, massFlow);
    }

    internal static bool SupportsThrottleControl(Vehicle vehicle)
        => GetThrottleControlStatus(vehicle) == ThrustStatus.Available;

    internal static ThrustStatus GetThrottleControlStatus(Vehicle vehicle)
    {
        if (vehicle?.Parts?.States == null
            || !ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
                .TryGetFrom(vehicle.Parts.States, out var states)
            || !ModuleStateful<RocketCore, RocketCoreState, RocketCoreGlobalState, EmptyStruct>
                .TryGetFrom(vehicle.Parts.States, out var coreStates))
            return ThrustStatus.NoAuthority;
        bool any = false;
        foreach (var engine in states.ModulesAndStates)
        {
            bool supplied = engine.Module.IsActive && engine.State.IsPropellantAvailable;
            if (engine.Module.Cores == null || engine.Module.Cores.Length == 0)
            {
                if (supplied)
                    return ThrustStatus.NoAuthority;
                continue;
            }
            foreach (RocketCore core in engine.Module.Cores)
            {
                if (core is not Combustor)
                {
                    // SolidMotor.UpdateState can keep a motor burning after its controller is disabled.
                    // IsNonzero also covers ignition and shutdown transitions.
                    // An inactive, unignited motor does not block liquid control.
                    if (coreStates.States[core.StatesIdx].IsNonzero()
                        || (supplied && coreStates.States[core.StatesIdx].IsPropellantAvailable))
                        return ThrustStatus.UnsupportedEngine;
                    continue;
                }
                if (supplied && coreStates.States[core.StatesIdx].IsPropellantAvailable)
                    any = true;
            }
        }
        return any ? ThrustStatus.Available : ThrustStatus.NoAuthority;
    }

    internal enum ThrustStatus { Off, Available, BelowMinimum, AboveMaximum, NoAuthority, UnsupportedEngine, InvalidDemand }

    internal readonly record struct ThrustCommand(double Throttle, double DeliveredThrust, ThrustStatus Status)
    {
        internal bool EngineOn => Throttle > 0.0;
        internal string Message => Status switch
        {
            ThrustStatus.BelowMinimum => "Thrust demand is below the engine minimum.",
            ThrustStatus.AboveMaximum => "Thrust demand exceeds available thrust.",
            ThrustStatus.NoAuthority => "No active engine has thrust available.",
            ThrustStatus.UnsupportedEngine => "Throttle control requires active liquid engines.",
            ThrustStatus.InvalidDemand => "Thrust demand is not finite.",
            _ => ""
        };
    }

    internal static ThrustCommand CommandForThrust(Vehicle vehicle, double demandN, double ambientPressure)
    {
        if (!double.IsFinite(demandN))
            return new(0.0, 0.0, ThrustStatus.InvalidDemand);
        ThrustStatus status = GetThrottleControlStatus(vehicle);
        if (status != ThrustStatus.Available)
            return new(0.0, 0.0, status);
        return InvertThrust(demandN, vehicle.GetMinThrottle(),
            throttle => ThrustAtThrottle(vehicle, throttle, ambientPressure));
    }

    // Returns -1 when throttle control is unavailable.
    // Callers that need saturation status must use CommandForThrust.
    internal static double ThrottleForThrust(Vehicle vehicle, double demandN, double ambientPressure)
    {
        ThrustCommand command = CommandForThrust(vehicle, demandN, ambientPressure);
        return command.Status is ThrustStatus.NoAuthority or ThrustStatus.UnsupportedEngine or ThrustStatus.InvalidDemand
            ? -1.0 : command.Throttle;
    }

    // Bisection requires a monotone thrust curve and handles its nonlinear pressure response.
    // Zero demand switches the engine off instead of commanding its minimum throttle.
    internal static ThrustCommand InvertThrust(double demandN, double minimumThrottle, Func<double, double> thrustAtThrottle)
    {
        if (!double.IsFinite(demandN))
            return new(0.0, 0.0, ThrustStatus.InvalidDemand);
        if (demandN <= 0.0)
            return new(0.0, 0.0, ThrustStatus.Off);
        if (!double.IsFinite(minimumThrottle) || minimumThrottle < 0.0 || minimumThrottle > 1.0)
            return new(0.0, 0.0, ThrustStatus.NoAuthority);

        double lo = Math.Max(minimumThrottle, float.Epsilon), hi = 1.0;
        double full = thrustAtThrottle(hi), minimum = thrustAtThrottle(lo);
        if (!double.IsFinite(full) || !double.IsFinite(minimum) || full <= 0.0 || minimum < 0.0 || minimum > full)
            return new(0.0, 0.0, ThrustStatus.NoAuthority);
        if (demandN < minimum)
            return new(lo, minimum, ThrustStatus.BelowMinimum);
        if (demandN >= full)
            return new(hi, full, demandN > full ? ThrustStatus.AboveMaximum : ThrustStatus.Available);

        for (int i = 0; i < 24; i++)
        {
            double mid = 0.5 * (lo + hi);
            double thrust = thrustAtThrottle(mid);
            if (!double.IsFinite(thrust))
                return new(0.0, 0.0, ThrustStatus.NoAuthority);
            if (thrust < demandN) lo = mid;
            else hi = mid;
        }
        // Return the realizable upper endpoint, including the game's float rounding.
        float throttle = (float)hi;
        return new(throttle, thrustAtThrottle(throttle), ThrustStatus.Available);
    }

    internal static double AmbientPressureAt(IParentBody parent, double altitudeAslM)
    {
        PhysicalAtmosphereReference physical = parent?.GetAtmosphereReference()?.Physical;
        if (physical == null)
            return 0.0;
        double pressure = physical.GetAtmosphericPressureAtAltitude(altitudeAslM);
        return double.IsFinite(pressure) && pressure > 0.0 ? pressure : 0.0;
    }
}

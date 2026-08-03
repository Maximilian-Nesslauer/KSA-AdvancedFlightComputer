using System;
using KSA;

// Vehicle-wide propulsion totals for the main engines.
//
// KSA's FlightComputer.VehicleConfigInfo used to expose TotalEngineVacuumThrust
// and TotalEngineExhaustVelocity; the August 2026 build dropped both (it now
// carries only the RCS thrusters and gimbals), so we sum the same quantities
// ourselves. EngineController.VacuumData is the game's own precomputed vacuum
// performance for that controller's cores — thrust vector and max mass flow at
// full throttle — refreshed by PartTree whenever the part tree changes.
//
// These are deliberately NOT filtered by EngineController.IsActive. The callers
// (G-FOLD planning, terminal hover) size a burn that may be commanded from a
// coast: both cut the engine when the commanded throttle falls near zero, so an
// active-only total would read zero exactly when the next solve needs it. This
// is a vehicle-configuration figure, not a live-thrust one — for live thrust use
// Vehicle.ComputeActiveThrust(ambientPressure).
internal static class KsaEnginePerf
{
    // Full-throttle vacuum thrust (N) and mass flow (kg/s) of the vehicle's
    // engines. Both zero if the vehicle has no engine controllers.
    internal static (double thrust, double massFlow) Vacuum(Vehicle vehicle)
    {
        PartTree tree = vehicle?.Parts;
        if (tree == null)
            return (0.0, 0.0);

        double thrust = 0.0, massFlow = 0.0;
        Span<EngineController> engines = tree.Modules.Get<EngineController>();
        for (int i = 0; i < engines.Length; i++)
        {
            thrust += engines[i].VacuumData.ThrustMax.Length();
            massFlow += engines[i].VacuumData.MassFlowRateMax;
        }
        return (thrust, massFlow);
    }

    // Full-throttle vacuum thrust (N) alone, for callers that don't need Isp.
    internal static double VacuumThrust(Vehicle vehicle) => Vacuum(vehicle).thrust;

    // Effective exhaust velocity (m/s) = thrust / mass flow. Zero if the vehicle
    // has no engines, which callers treat as "nothing to plan with".
    internal static double VacuumExhaustVelocity(Vehicle vehicle)
    {
        (double thrust, double massFlow) = Vacuum(vehicle);
        return massFlow > 0.0 ? thrust / massFlow : 0.0;
    }
}

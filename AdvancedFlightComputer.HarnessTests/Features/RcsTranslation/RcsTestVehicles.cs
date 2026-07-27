namespace AdvancedFlightComputer.HarnessTests;

// The vehicle saves the RCS flight tests fly when KSA_HEADLESS_VEHICLES is
// not set: the big staged rocket, the small RCS-heavy single-stage, and an
// asymmetric layout (unbalanced axis authority, thrusters well off the CoM)
// that exercises the high-attitude-fight / high-slew regime the two balanced
// saves never reach. The harness helper TestSupport.ResolveVehicleSaves turns
// this list into the present (or KSA_HEADLESS_VEHICLES-overridden) save set.
//
// All three currently ship dev propellant (a partially present reactant mix),
// so ResourceManager.MassChange under-withdraws at full thrust and the fuel
// buckets are not a clean absolute reference (the fuel telemetry's negative
// attitude bucket is how that surfaces); a fully stocked asymmetric save would
// let the estimator factors be tuned numerically instead of structurally.
internal static class RcsTestVehicles
{
    public static readonly string[] Candidates =
        { "Test Vehicle 1", "RCS Test 1", "RCS Test 1 Asymetric" };
}

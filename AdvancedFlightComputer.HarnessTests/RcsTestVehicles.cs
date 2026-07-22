namespace AdvancedFlightComputer.HarnessTests;

// The vehicle saves the RCS flight tests fly when KSA_HEADLESS_VEHICLES is
// not set: the big staged rocket and the small RCS-heavy single-stage, so
// the suite covers both without machine-specific configuration. The harness
// helper TestSupport.ResolveVehicleSaves turns this list into the present
// (or KSA_HEADLESS_VEHICLES-overridden) save set.
internal static class RcsTestVehicles
{
    public static readonly string[] Candidates = { "Test Vehicle 1", "RCS Test 1" };
}

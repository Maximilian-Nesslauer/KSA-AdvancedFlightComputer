using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Resolves which vehicle saves the RCS flight tests fly: every present
// candidate, so the suite covers both the big staged rocket and the small
// RCS-heavy single-stage without machine-specific configuration.
// KSA_HEADLESS_RCS_VEHICLES (comma-separated) overrides the list. It is a
// separate variable on purpose: run-headless.ps1 always exports
// KSA_HEADLESS_VEHICLE for the harness flight test, so treating that one
// as a pin would permanently reduce these tests to a single vehicle.
internal static class RcsTestVehicles
{
    public const string EnvVar = "KSA_HEADLESS_RCS_VEHICLES";

    private static readonly string[] Candidates = { "Test Vehicle 1", "RCS Test 1" };

    public static IReadOnlyList<string> Resolve()
    {
        string? pinned = Environment.GetEnvironmentVariable(EnvVar);
        if (!string.IsNullOrEmpty(pinned))
        {
            List<string> list = new();
            foreach (string entry in pinned.Split(','))
            {
                string trimmed = entry.Trim();
                if (trimmed.Length > 0)
                    list.Add(trimmed);
            }
            return list;
        }

        List<string> present = new();
        foreach (string candidate in Candidates)
        {
            if (SaveExists(candidate))
                present.Add(candidate);
        }
        return present;
    }

    private static bool SaveExists(string saveId)
    {
        foreach (VehicleSave save in VehicleSaves.AsSpan())
        {
            if (string.Equals(save.Id, saveId, StringComparison.Ordinal))
                return true;
        }
        return false;
    }
}

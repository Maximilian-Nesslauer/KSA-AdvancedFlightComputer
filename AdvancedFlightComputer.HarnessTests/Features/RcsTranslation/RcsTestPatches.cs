using AdvancedFlightComputer.Features.RcsTranslation;
using HarmonyLib;
using HeadlessHarness.Core;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The harness manifest runs only Core + HeadlessHarness, so the mod's
// OnFullyLoaded never applies its patches; the RCS flight tests apply exactly
// the executor-relevant subset once per process (gauge/UI patches stay off:
// nothing renders headless).
internal static class RcsTestPatches
{
    private static Harmony? _harmony;

    public static void Ensure()
    {
        if (_harmony != null)
            return;
        RcsExecRegistry.Init();
        _harmony = new Harmony("com.maxi.afc.harnesstests.rcs");
        _harmony.CreateClassProcessor(typeof(RcsComputeControlPatch)).Patch();
        _harmony.CreateClassProcessor(typeof(RcsDriverPatch)).Patch();
        _harmony.CreateClassProcessor(typeof(RcsSetEnumPatch)).Patch();
        _harmony.CreateClassProcessor(typeof(RcsCancelLogPatch)).Patch();
    }
}

// Cancels log their reason to the game log, which headless runs never
// write; mirror the reason into the harness log so a failed flight test
// explains itself instead of just going inactive. The recorded reason also
// gates the align scenario's propellant-exhaustion SKIP.
[HarmonyPatch(typeof(RcsExecutor), nameof(RcsExecutor.Cancel))]
internal static class RcsCancelLogPatch
{
    internal static string? LastReason;

    static void Postfix(Vehicle vehicle, string reason)
    {
        LastReason = reason;
        HarnessLog.Line($"[afc-rcs] executor cancelled '{vehicle.Id}': {reason}");
    }
}

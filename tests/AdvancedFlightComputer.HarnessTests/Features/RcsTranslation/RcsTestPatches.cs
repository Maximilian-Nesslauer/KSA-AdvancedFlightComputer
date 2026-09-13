using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.RcsTranslation;
using HarmonyLib;
using HeadlessHarness.Core;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The harness does not load the mod, so each RCS flight test applies only the
// patches that the executor needs. It removes them when the test ends.
internal static class RcsTestPatches
{
    public static Scope Apply() => new();

    internal sealed class Scope : IDisposable
    {
        private readonly Harmony _harmony;
        private readonly bool _rcsEnabledBefore;

        internal Scope()
        {
            _rcsEnabledBefore = SharedVehicleHooks.RcsEnabled;
            RcsExecRegistry.Init();
            _harmony = new Harmony("com.maxi.afc.harnesstests.rcs");
            VehicleCommandSink.ApplyPatches(_harmony);
            SharedVehicleHooks.ApplyPatches(_harmony);
            _harmony.CreateClassProcessor(typeof(RcsSetEnumPatch)).Patch();
            _harmony.CreateClassProcessor(typeof(RcsWarpPatch)).Patch();
            _harmony.CreateClassProcessor(typeof(RcsWarpObservationPatch)).Patch();
            _harmony.CreateClassProcessor(typeof(RcsCancelLogPatch)).Patch();
            SharedVehicleHooks.RcsEnabled = true;
        }

        public void Dispose()
        {
            SharedVehicleHooks.RcsEnabled = _rcsEnabledBefore;
            RcsExecRegistry.Reset();
            RcsCommandChannel.Reset();
            RcsExecutor.ResetUiCache();
            RcsWarpObservationPatch.Reset();
            RcsCancelLogPatch.LastReason = null;
            _harmony.UnpatchAll(_harmony.Id);
        }
    }
}

[HarmonyPatch(typeof(Universe), nameof(Universe.AutoWarpTo),
    new Type[] { typeof(UniverseTime), typeof(double) })]
internal static class RcsWarpObservationPatch
{
    internal static UniverseTime? LastEndTime;
    internal static double LastMargin = double.NaN;

    internal static void Reset()
    {
        LastEndTime = null;
        LastMargin = double.NaN;
    }

    static void Postfix(UniverseTime endTime, double simTimeMargin)
    {
        LastEndTime = endTime;
        LastMargin = simTimeMargin;
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

using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoStage;
using HarmonyLib;

namespace AdvancedFlightComputer.HarnessTests;

// The harness never runs Mod.OnFullyLoaded, so each staging test applies the feature's patches on
// its own owner and removes them when it ends.
internal static class AutoStageTestPatches
{
    public static Scope Apply() => new();

    internal sealed class Scope : IDisposable
    {
        private readonly Harmony _harmony = new("com.maxi.afc.harnesstests.autostage");
        private readonly bool _enabledBefore = SharedVehicleHooks.AutoStageEnabled;

        internal Scope()
        {
            try
            {
                SharedVehicleHooks.ApplyPatches(_harmony);
                AutoStageFeature.ApplyPatches(_harmony);
                SharedVehicleHooks.AutoStageEnabled = true;
            }
            catch
            {
                Dispose();
                throw;
            }
        }

        public void Dispose()
        {
            SharedVehicleHooks.AutoStageEnabled = _enabledBefore;
            AutoStageFeature.Disable();
            AutoStageFeature.RemoveGaugeEnum();
            _harmony.UnpatchAll(_harmony.Id);
        }
    }
}

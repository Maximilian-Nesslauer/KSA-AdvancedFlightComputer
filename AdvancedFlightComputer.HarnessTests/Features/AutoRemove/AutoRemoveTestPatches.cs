using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.AutoRemove;
using AdvancedFlightComputer.Features.RcsTranslation;
using HarmonyLib;

namespace AdvancedFlightComputer.HarnessTests;

// The harness never runs Mod.OnFullyLoaded, so each burn-removal test applies the shared tick on
// its own owner, wires the RCS subscription, and removes both when it ends. The config is set
// directly rather than loaded, so the suite never touches the player's mod directory.
internal static class AutoRemoveTestPatches
{
    public static Scope Apply() => new();

    internal sealed class Scope : IDisposable
    {
        private readonly Harmony _harmony = new("com.maxi.afc.harnesstests.autoremove");
        private readonly bool _enabledBefore = SharedVehicleHooks.AutoRemoveEnabled;

        internal Scope()
        {
            try
            {
                SharedVehicleHooks.ApplyPatches(_harmony);
                RcsBurnCompletions.Completed += FinishedBurnRemover.OnRcsBurnCompleted;
                SharedVehicleHooks.AutoRemoveEnabled = true;
                AutoRemoveConfig.Enabled = true;
            }
            catch
            {
                Dispose();
                throw;
            }
        }

        public void Dispose()
        {
            SharedVehicleHooks.AutoRemoveEnabled = _enabledBefore;
            RcsBurnCompletions.Completed -= FinishedBurnRemover.OnRcsBurnCompleted;
            FinishedBurnRemover.Reset();
            AutoRemoveConfig.Reset();
            _harmony.UnpatchAll(_harmony.Id);
        }
    }
}

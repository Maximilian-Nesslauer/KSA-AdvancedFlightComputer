using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.RcsTranslation;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Every piece of mod state whose lifetime is one save game, in one place.
///
/// A save load replaces the world: <c>Universe.DeserializeSave</c> runs
/// <c>CelestialSystem.DestroyAllVehicles</c> and then rebuilds every vehicle, so
/// anything keyed on a vehicle id, a body id or a LookupIndex, and anything
/// holding a Burn, Vehicle or FlightPlan reference, describes a world that no
/// longer exists. None of it self-heals on its own: <c>Vehicle.Dispose</c> leaves
/// the destroyed vehicle's FlightComputer and BurnPlan intact, so a kept Burn
/// still resolves and still looks live, and <c>LookupCollection.Deregister</c>
/// swap-removes, so a kept index resolves to a different body rather than to
/// nothing.
///
/// The list is here because the resets used to be spelled out twice - once in
/// <see cref="MultiPass.SaveLoadObserver"/>'s load postfix, once in
/// <c>Mod.Unload</c> - and the two drifted, with several holders reset on unload
/// and forgotten on load. New save-scoped state goes here and both paths get it.
///
/// Out of scope on purpose: the registries the load path reloads from disk
/// immediately afterwards (<see cref="MultiPassRegistry"/>,
/// <see cref="RcsExecRegistry"/>), the per-feature Enabled flags, and the dedup
/// sets whose lifetime is deliberately the whole mod load rather than one save
/// (<see cref="LogHelper"/>, <c>Patch_AlignmentTime</c>).
/// </summary>
internal static class SaveScopedState
{
    public static void ResetAll()
    {
        // Preview and plan caches keyed on vehicle / target id.
        MultiPassPreviewCache.Reset();
        HohmannMultiPassPlanner.ResetShiftCache();
        MultiPassUI.Reset();
        HohmannMultiPassUI.Reset();
        HohmannFlybyUI.Reset();

        // Plan-window state held across frames: the burn we created plus the
        // source vehicle and porkchop entry the preview draws from.
        Patch_DrawPlanWindow.Reset();

        // Match Inclination's target selection and the per-source input defaults.
        ManeuverToolsWindow.Reset();

        // Per-vehicle burn-mode tracking. Kept, the first post-load tick would
        // compare a pre-load mode against the restored one for the same id.
        PassCompletionPatch.Reset();

        // Vehicle-id-keyed capability snapshot behind the RCS burn UI.
        RcsExecutor.ResetUiCache();
    }
}

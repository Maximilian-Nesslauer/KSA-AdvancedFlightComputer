using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.PlanWindow;
using AdvancedFlightComputer.Features.RcsTranslation;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// Every piece of mod state whose lifetime is one save game, in one list, so the save-load path
/// (<see cref="SaveLoadObserver"/>'s load postfix) and <c>Mod.Unload</c> cannot disagree.
///
/// A save load replaces the world. <c>Universe.DeserializeSave</c> runs
/// <c>CelestialSystem.DestroyAllVehicles</c> and rebuilds every vehicle, so anything keyed on a
/// vehicle id, a body id or a LookupIndex, and anything holding a Burn, Vehicle, BurnPlan or
/// FlightPlan reference, describes a world that no longer exists. None of it self-heals.
/// <c>Vehicle.Dispose</c> leaves the destroyed vehicle's FlightComputer and BurnPlan intact, so a
/// kept Burn still looks live, and <c>LookupCollection.Deregister</c> swap-removes, so a kept
/// index resolves to a different body rather than to nothing.
///
/// Three kinds of state stay out on purpose, because their lifetime is not the save. The load
/// path reloads <see cref="MultiPassRegistry"/> and <see cref="RcsExecRegistry"/> from disk right
/// after this runs, the per-feature Enabled flags belong to the patch state, and the dedup sets in
/// <see cref="LogHelper"/> and <c>Patch_SetTransferInfo</c> live for the whole mod load.
/// </summary>
internal static class SaveScopedState
{
    public static void ResetAll()
    {
        MultiPassPreviewCache.Reset();
        HohmannMultiPassPlanner.ResetShiftCache();
        MultiPassUI.Reset();
        HohmannMultiPassUI.Reset();
        HohmannFlybyUI.Reset();
        Patch_DrawPlanWindow.Reset();
        ManeuverToolsWindow.Reset();
        PassCompletionPatch.Reset();
        RcsExecutor.ResetUiCache();
        RcsCommandChannel.Reset();
    }
}

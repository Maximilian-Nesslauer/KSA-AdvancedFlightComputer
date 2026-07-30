using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Tracks the active KSA save game so MultiPassRegistry can scope its
/// entries by save id. Two Harmony patches:
/// <list type="bullet">
/// <item><c>UncompressedSave.Load</c> Postfix: capture the loaded
/// save id and refresh the registry from disk so in-memory state
/// matches whatever was persisted for that save.</item>
/// <item><c>UncompressedSave.Write</c> Postfix: move the session's
/// entries to the just-written save id when it differs from the loaded
/// one, then flush the registry to disk. This is the one auto-save
/// point - mid-execution mutations are in-memory only.</item>
/// </list>
/// </summary>
internal static class SaveLoadObserver
{
    /// <summary>Save id of the most recently loaded / written save.
    /// Empty in the default starting situation (no save loaded yet).</summary>
    public static string CurrentSaveId { get; private set; } = string.Empty;

    /// <summary>Raised after a save was loaded / written and the MultiPass
    /// registry work is done. Other features (RcsTranslation) hang their
    /// save-scoped registries here so the UncompressedSave methods carry
    /// exactly one patch pair regardless of how many features are enabled.
    /// The write event receives (oldSaveId, newSaveId): by the time it fires
    /// <see cref="CurrentSaveId"/> already holds the new id, so a subscriber
    /// that needs the pre-write id for its own rekey must take it from the
    /// event rather than read the property.</summary>
    public static event Action? SaveLoaded;
    public static event Action<string, string>? SaveWritten;

    public static void Reset()
    {
        CurrentSaveId = string.Empty;
        SaveLoaded = null;
        SaveWritten = null;
    }

    public static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(LoadPatch)).Patch();
        harmony.CreateClassProcessor(typeof(WritePatch)).Patch();

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug("[AFC] SaveLoadObserver: patches applied.");
    }

    [HarmonyPatch(typeof(UncompressedSave), nameof(UncompressedSave.Load), new Type[0])]
    private static class LoadPatch
    {
        static void Postfix(UncompressedSave __instance)
        {
            try
            {
                CurrentSaveId = __instance.Id ?? string.Empty;

                // Everything keyed on the world this load just replaced. The
                // membership lives in SaveScopedState so this path and Mod.Unload
                // cannot drift apart, which is how Patch_DrawPlanWindow and
                // ManeuverToolsWindow came to be reset on unload but not here.
                SaveScopedState.ResetAll();

                // Refresh registry from disk so in-memory exec state
                // matches whatever was persisted for the just-loaded
                // save (handles "reload to revert" mid-game).
                MultiPassRegistry.Load();

                SaveLoaded?.Invoke();

                if (DebugConfig.MultiPass)
                {
                    DefaultCategory.Log.Debug(
                        $"[AFC] SaveLoadObserver.LoadPatch: loaded save '{CurrentSaveId}', " +
                        $"registry has {MultiPassRegistry.Count} total entries " +
                        $"({MultiPassRegistry.CountForCurrentSave} for this save).");
                    MultiPassDebug.LogRegistry(
                        "SaveLoadObserver.LoadPatch (post-load)", MultiPassRegistry.Snapshot);

                    // Dump the BurnPlan of the controlled vehicle so
                    // we can see whether the burn we expect to reattach
                    // to is actually present.
                    Vehicle? controlled = Program.ControlledVehicle;
                    if (controlled != null)
                        MultiPassDebug.LogBurnPlan(
                            $"SaveLoadObserver.LoadPatch vehicle='{controlled.Id}'",
                            controlled.FlightComputer.BurnPlan);
                }
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] SaveLoadObserver Load Postfix: {ex}");
            }
        }
    }

    [HarmonyPatch(typeof(UncompressedSave), nameof(UncompressedSave.Write), new Type[0])]
    private static class WritePatch
    {
        static void Postfix(UncompressedSave __instance)
        {
            try
            {
                string newSaveId = __instance.Id ?? string.Empty;
                // The id the session's registry entries are keyed under. A write
                // can carry a different id than the loaded save (Save-As, first
                // save of an unsaved session, overwriting another save from the
                // saves list), and the rekey below needs both ids.
                string oldSaveId = CurrentSaveId;

                if (DebugConfig.MultiPass)
                {
                    MultiPassDebug.LogRegistry(
                        $"SaveLoadObserver.WritePatch (pre-rekey, save '{oldSaveId}' -> '{newSaveId}')",
                        MultiPassRegistry.Snapshot);

                    Vehicle? controlled = Program.ControlledVehicle;
                    if (controlled != null)
                        MultiPassDebug.LogBurnPlan(
                            $"SaveLoadObserver.WritePatch vehicle='{controlled.Id}' (pre-save)",
                            controlled.FlightComputer.BurnPlan);
                }

                // Rekey before Save() so the moved entries hit disk.
                MultiPassRegistry.RekeyTo(oldSaveId, newSaveId);

                CurrentSaveId = newSaveId;

                MultiPassRegistry.Save();

                SaveWritten?.Invoke(oldSaveId, newSaveId);

                if (DebugConfig.MultiPass)
                    DefaultCategory.Log.Debug(
                        $"[AFC] SaveLoadObserver.WritePatch: wrote save '{CurrentSaveId}', " +
                        $"persisted registry alongside.");
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] SaveLoadObserver Write Postfix: {ex}");
            }
        }
    }
}

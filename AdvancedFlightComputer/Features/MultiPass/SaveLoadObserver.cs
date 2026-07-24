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
/// <item><c>UncompressedSave.Write</c> Postfix: rekey transient
/// entries to the just-saved save id and flush the registry to disk.
/// This is the one auto-save point - mid-execution mutations are
/// in-memory only.</item>
/// </list>
/// </summary>
internal static class SaveLoadObserver
{
    /// <summary>Save id of the most recently loaded / written save.
    /// Empty in the default starting situation (no save loaded yet).</summary>
    public static string CurrentSaveId { get; private set; } = string.Empty;

    public static void Reset()
    {
        CurrentSaveId = string.Empty;
    }

    public static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(LoadPatch)).Patch();
        harmony.CreateClassProcessor(typeof(WritePatch)).Patch();

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug("[AFC] SaveLoadObserver: patches applied.");
    }

    [HarmonyPatch(typeof(UncompressedSave), nameof(UncompressedSave.Load))]
    private static class LoadPatch
    {
        static void Postfix(UncompressedSave __instance)
        {
            try
            {
                CurrentSaveId = __instance.Id ?? string.Empty;

                // Drop the per-frame preview cache: its inputs (vehicle
                // id, mass, engine signature) can match a same-named
                // vehicle in the just-loaded save and return a stale
                // SequenceBurnState / PassPreviewResult from the
                // previous world.
                MultiPassPreviewCache.Reset();
                // Same reasoning for the Hohmann shift cache: keyed on
                // vehicle / target id, would otherwise survive a save
                // load with a same-id vehicle in a different geometry.
                HohmannMultiPassPlanner.ResetShiftCache();
                MultiPassUI.Reset();
                HohmannMultiPassUI.Reset();
                // Same reasoning: the flyby arm flag + inputs + result cache are
                // keyed on the source vehicle and would otherwise carry into the
                // just-loaded save, silently baking a flyby the user did not set.
                HohmannFlybyUI.Reset();

                // Refresh registry from disk so in-memory exec state
                // matches whatever was persisted for the just-loaded
                // save (handles "reload to revert" mid-game).
                MultiPassRegistry.Load();

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

    [HarmonyPatch(typeof(UncompressedSave), nameof(UncompressedSave.Write))]
    private static class WritePatch
    {
        static void Postfix(UncompressedSave __instance)
        {
            try
            {
                string newSaveId = __instance.Id ?? string.Empty;

                if (DebugConfig.MultiPass)
                {
                    MultiPassDebug.LogRegistry(
                        $"SaveLoadObserver.WritePatch (pre-rekey, target save='{newSaveId}')",
                        MultiPassRegistry.Snapshot);

                    Vehicle? controlled = Program.ControlledVehicle;
                    if (controlled != null)
                        MultiPassDebug.LogBurnPlan(
                            $"SaveLoadObserver.WritePatch vehicle='{controlled.Id}' (pre-save)",
                            controlled.FlightComputer.BurnPlan);
                }

                // Rekey before Save() so the promoted entries hit disk.
                MultiPassRegistry.RekeyTransientsTo(newSaveId);

                CurrentSaveId = newSaveId;

                MultiPassRegistry.Save();

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

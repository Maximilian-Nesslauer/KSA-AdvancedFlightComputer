using System;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// Loading restores the saved registry. Writing moves entries to the written save ID before persisting them. Execution changes between saves stay in memory.
internal static class SaveLoadObserver
{
    // The default world has an empty save ID until it is saved.
    public static string CurrentSaveId { get; private set; } = string.Empty;

    // Notify subscribers after registry work is complete. SaveWritten supplies both IDs because CurrentSaveId already contains the new ID when the event fires.
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

                SaveScopedState.ResetAll();

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

    internal static void OnSaveWritten(string newSaveId)
    {
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

        // Rekey before writing so the file uses the ID of the save just written.
        MultiPassRegistry.RekeyTo(oldSaveId, newSaveId);

        CurrentSaveId = newSaveId;

        MultiPassRegistry.Save();

        SaveWritten?.Invoke(oldSaveId, newSaveId);

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] SaveLoadObserver.WritePatch: wrote save '{CurrentSaveId}', " +
                $"persisted registry alongside.");
    }

    [HarmonyPatch(typeof(UncompressedSave), nameof(UncompressedSave.Write), new Type[0])]
    private static class WritePatch
    {
        static void Postfix(UncompressedSave __instance)
        {
            try
            {
                OnSaveWritten(__instance.Id ?? string.Empty);
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] SaveLoadObserver Write Postfix: {ex}");
            }
        }
    }
}

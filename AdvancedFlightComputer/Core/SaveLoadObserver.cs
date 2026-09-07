using System;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Core;

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
            // UncompressedSave.Load returns normally when GameSaves.RefusedInEditor refuses the load,
            // so its postfix still runs. Read the same editor state without raising a second alert.
            if (Program.IsEditorOpen)
            {
                if (DebugConfig.MultiPass)
                    DefaultCategory.Log.Debug(
                        $"[AFC] SaveLoadObserver.LoadPatch: ignored the refused load of " +
                        $"'{__instance.Id}' because the vehicle editor is open.");
                return;
            }

            string saveId = __instance.Id ?? string.Empty;
            CurrentSaveId = saveId;

            // Keep each stage independent so one failure does not prevent the others.
            RunStage(saveId, "reset the save-scoped state", SaveScopedState.ResetAll);
            RunStage(saveId, "restore the multi-pass registry", MultiPassRegistry.Load);
            RunStage(saveId, "notify the save-loaded subscribers", () => SaveLoaded?.Invoke());
            if (DebugConfig.MultiPass)
                RunStage(saveId, "log the loaded state", () => LogLoadedState(saveId));
        }

        private static void RunStage(string saveId, string stageName, Action stage)
        {
            try
            {
                stage();
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] SaveLoadObserver.LoadPatch: could not {stageName} for save '{saveId}': {ex}");
            }
        }

        private static void LogLoadedState(string saveId)
        {
            DefaultCategory.Log.Debug(
                $"[AFC] SaveLoadObserver.LoadPatch: loaded save '{saveId}', " +
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

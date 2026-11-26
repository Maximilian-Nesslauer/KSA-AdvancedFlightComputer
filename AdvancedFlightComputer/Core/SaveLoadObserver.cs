using System;
using System.IO;
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

    // The folder of the save that was loaded or written last, so a subscriber can keep its own file with that save. Null for the default world and for a save whose folder is not known.
    public static DirectoryInfo? CurrentSaveDirectory { get; private set; }

    // Notify subscribers after registry work is complete. SaveWritten supplies both IDs because CurrentSaveId already contains the new ID when the event fires.
    public static event Action? SaveLoaded;
    public static event Action? SystemSwapped;
    public static event Action<string, string>? SaveWritten;

    public static void Reset()
    {
        CurrentSaveId = string.Empty;
        CurrentSaveDirectory = null;
        SaveLoaded = null;
        SystemSwapped = null;
        SaveWritten = null;
    }

    public static void ApplyPatches(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(LoadPatch)).Patch();
        harmony.CreateClassProcessor(typeof(WritePatch)).Patch();
        if (GameReflection.Program_SwapSystem is { } swapSystem && swapSystem.ReturnType == typeof(SystemSwapResult))
        {
            try
            {
                harmony.Patch(swapSystem,
                    postfix: new HarmonyMethod(typeof(SaveLoadObserver), nameof(OnSystemSwapped)));
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning($"[AFC] Could not patch Program.SwapSystem, so star-system swaps without a save load will not reset save-scoped state: {ex}");
            }
        }
        else
        {
            DefaultCategory.Log.Warning("[AFC] Program.SwapSystem was not found with the expected signature, so star-system swaps without a save load will not reset save-scoped state.");
        }

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug("[AFC] SaveLoadObserver: patches applied.");
    }

    private static void OnSystemSwapped(SystemSwapResult __result)
    {
        if (__result != SystemSwapResult.Swapped)
            return;

        CurrentSaveId = string.Empty;
        CurrentSaveDirectory = null;

        RunStage(string.Empty, "reset the save-scoped state on system swap", SaveScopedState.ResetAll);
        RunStage(string.Empty, "notify the system-swapped subscribers", () => SystemSwapped?.Invoke());
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
                $"[AFC] SaveLoadObserver: could not {stageName} for save '{saveId}': {ex}");
        }
    }

    // UncompressedSave.Load only reads the file and hands ApplyLoaded to SaveSystemSwap.RequestLoad, which runs it at once for a save of the running system, on a later frame through SaveSystemSwap.OnFrame after Program.SwapSystem for a save of another system, in a new process for a save whose system needs a restart, and never for a save of a system that is not installed or a load the editor refuses.
    // ApplyLoaded is the step that calls Universe.DeserializeSave, so its postfix runs exactly once for each save that is really loaded.
    [HarmonyPatch(typeof(UncompressedSave), "ApplyLoaded", new Type[0])]
    private static class LoadPatch
    {
        static void Postfix(UncompressedSave __instance)
        {
            string saveId = __instance.Id ?? string.Empty;
            CurrentSaveId = saveId;
            CurrentSaveDirectory = __instance.Directory;

            // Keep each stage independent so one failure does not prevent the others.
            RunStage(saveId, "reset the save-scoped state", SaveScopedState.ResetAll);
            RunStage(saveId, "restore the multi-pass registry", MultiPassRegistry.Load);
            RunStage(saveId, "notify the save-loaded subscribers", () => SaveLoaded?.Invoke());
            if (DebugConfig.MultiPass)
                RunStage(saveId, "log the loaded state", () => LogLoadedState(saveId));
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

    internal static void OnSaveWritten(string newSaveId, DirectoryInfo? saveDirectory = null)
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
        CurrentSaveDirectory = saveDirectory;

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
        static void Postfix(UncompressedSave __instance, bool __result)
        {
            // A failed write keeps the registry on the previous save.
            if (!__result)
                return;

            try
            {
                OnSaveWritten(__instance.Id ?? string.Empty, __instance.Directory);
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] SaveLoadObserver Write Postfix: {ex}");
            }
        }
    }
}

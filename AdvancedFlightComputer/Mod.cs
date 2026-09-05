using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.HyperbolicTargets;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.RcsTranslation;
using Brutal.Logging;
using HarmonyLib;
using KSA;
using StarMap.API;

namespace AdvancedFlightComputer;

[StarMapMod]
public sealed class Mod
{
    private const string TestedGameVersion = "v2026.9.7.5402";

    private static Harmony? _harmony;
    private static bool _maneuverTypesInjected;
    private static bool _saveObserverPatched;

    [StarMapAllModsLoaded]
    public void OnFullyLoaded()
    {
        string gameVersion = VersionInfo.Current.VersionString;
        DefaultCategory.Log.Info($"[AFC] Game version: {gameVersion}");
        if (gameVersion != TestedGameVersion)
            DefaultCategory.Log.Warning(
                $"[AFC] Tested against {TestedGameVersion}, current is {gameVersion}. Some features may not work correctly.");

        Harmony harmony = new Harmony("com.maxi.advancedflightcomputer");
        _harmony = harmony;

        if (Validated("HyperbolicTargets", GameReflection.ValidateHyperbolicTargets))
            TryPatchBlock("HyperbolicTargets", () => HyperbolicTargets.ApplyPatches(harmony));

        if (Validated("ManeuverTools", GameReflection.ValidateManeuverTools))
        {
            // The quick-tools and MultiPass are separate blocks so that a MultiPass failure does
            // not roll back the quick-tools.
            if (!TryPatchBlock("ManeuverTools", () => PatchManeuverTools(harmony)))
                DisableManeuverTools();
            else if (Validated("MultiPass", GameReflection.ValidateMultiPass)
                     && !TryPatchBlock("MultiPass", () => PatchMultiPass(harmony)))
                DisableMultiPass();
        }

        if (Validated("RcsTranslation", GameReflection.ValidateRcsTranslation))
            TryPatchBlock("RcsTranslation", () => PatchRcsTranslation(harmony));

        DefaultCategory.Log.Info("[AFC] Loaded and patched.");
    }

    private static bool Validated(string feature, Func<bool> validate)
    {
        if (validate())
            return true;
        DefaultCategory.Log.Warning($"[AFC] {feature} disabled - reflection targets not found.");
        return false;
    }

    private static void PatchManeuverTools(Harmony harmony)
    {
        ManeuverTools.InjectTransferTypes();
        _maneuverTypesInjected = true;
        ManeuverTools.ApplyPatches(harmony);
    }

    // With the DrawPlanWindow prefix possibly missing, the injected types would sit in stock's
    // dropdown with no window body.
    private static void DisableManeuverTools()
    {
        BurnMenuLauncher.Enabled = false;
        RemoveTransferTypes();
    }

    private static void PatchMultiPass(Harmony harmony)
    {
        MultiPassRegistry.Init();
        harmony.CreateClassProcessor(typeof(PassCompletionPatch)).Patch();
        harmony.CreateClassProcessor(typeof(VehicleDisposePatch)).Patch();
        PatchSaveObserverOnce(harmony);
        MultiPassUI.Enabled = true;

        // The inline Hohmann UI, the flyby targeting drawn inside it, and the fallback injection all
        // hang off the one transpiler anchor.
        if (PatchIfAnchored(harmony, typeof(Patch_DrawPlanWindow_HohmannMultiPass),
                Patch_DrawPlanWindow_HohmannMultiPass.IsAnchorPresent, "HohmannMultiPass", "DrawCorrectionTransfer"))
        {
            HohmannMultiPassUI.Enabled = true;
            HohmannFlybyUI.Enabled = true;
            DefaultCategory.Log.Info("[AFC] Flyby targeting enabled (Hohmann plan window).");

            // Stock's center-aimed preview is two draws on one toggle, hence two patches, and the
            // flyby itself works without either.
            PatchIfAnchored(harmony, typeof(Patch_TransferPlanner_DrawSelectedTransfer_Flyby),
                Patch_TransferPlanner_DrawSelectedTransfer_Flyby.IsAnchorPresent,
                "Flyby stock-preview suppression", "DrawSelectedTransfer");
            PatchIfAnchored(harmony, typeof(Patch_TransferPlanner_DrawSelectedTransferUi_Flyby),
                Patch_TransferPlanner_DrawSelectedTransferUi_Flyby.IsAnchorPresent,
                "Flyby stock-marker suppression", "DrawSelectedTransferUi");
            PatchIfAnchored(harmony, typeof(Patch_DrawPlanWindow_HohmannFallback),
                Patch_DrawPlanWindow_HohmannFallback.IsAnchorPresent,
                "HohmannFallback", "ConsoleStyle.PopWidgetStyle");
        }

        PatchIfAnchored(harmony, typeof(Patch_DrawPlanWindow_CreateInterceptor),
            Patch_DrawPlanWindow_CreateInterceptor.IsAnchorPresent, "HohmannCreateInterceptor", "Burn.Create");
        harmony.CreateClassProcessor(typeof(Patch_TransferPlanner_OnPreRender_Hohmann)).Patch();
        harmony.CreateClassProcessor(typeof(Patch_TransferPlanner_DrawPlanWindow_HohmannMarkers)).Patch();
    }

    // A mid-block failure can leave a flag set for a patch that never applied. Every flag here
    // belongs to that block, so clearing them wholesale is safe.
    private static void DisableMultiPass()
    {
        MultiPassUI.Enabled = false;
        HohmannMultiPassUI.Enabled = false;
        HohmannFlybyUI.Enabled = false;
    }

    private static void PatchRcsTranslation(Harmony harmony)
    {
        harmony.CreateClassProcessor(typeof(RcsComputeControlPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsDriverPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsSetEnumPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsVehicleDisposePatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsGaugePatches.IsDisabledPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsGaugePatches.PackDataPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsGaugePatches.HoveredPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsBurnWindowUi)).Patch();
        harmony.CreateClassProcessor(typeof(RcsBurnCanvasUi)).Patch();

        RcsExecRegistry.Init();
        PatchSaveObserverOnce(harmony);
        SaveLoadObserver.SaveLoaded += RcsExecRegistry.Load;
        SaveLoadObserver.SaveWritten += OnRcsSaveWritten;
    }

    // MultiPass and RcsTranslation share one patch pair on UncompressedSave. The flag is set before
    // the call on purpose, so a half-applied pair is not retried by the second feature and
    // SaveLoadObserver keeps its one-pair contract.
    private static void PatchSaveObserverOnce(Harmony harmony)
    {
        if (_saveObserverPatched)
            return;
        _saveObserverPatched = true;
        SaveLoadObserver.ApplyPatches(harmony);
    }

    private static bool PatchIfAnchored(Harmony harmony, Type patch, bool anchorPresent, string feature, string anchor)
    {
        if (!anchorPresent)
        {
            DefaultCategory.Log.Warning($"[AFC] {feature} disabled - {anchor} anchor not found.");
            return false;
        }
        harmony.CreateClassProcessor(patch).Patch();
        return true;
    }

    /// <summary>Contains a patching failure to one feature. Several targets are attribute-bound
    /// and outside the reflection validation, so a game-side rename throws out of Patch() where
    /// the validation gate cannot catch it, and StarMap invokes every mod's AllModsLoaded hook
    /// from one loop, so an escaping exception would also skip every later mod's hook. Harmony
    /// applies patches one by one, so everything before the failing call stays live, and there is
    /// no rollback.</summary>
    private static bool TryPatchBlock(string feature, Action apply)
    {
        try
        {
            apply();
            return true;
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] {feature} patching failed (game version may have changed); " +
                $"patches applied before the failure stay live: {ex}");
            return false;
        }
    }

    private static void RemoveTransferTypes()
    {
        if (!_maneuverTypesInjected)
            return;
        ManeuverTools.RemoveTransferTypes();
        _maneuverTypesInjected = false;
    }

    [StarMapUnload]
    public void Unload()
    {
        _harmony?.UnpatchAll(_harmony.Id);
        _harmony = null;
        _saveObserverPatched = false;
        RemoveTransferTypes();

        // Persistence is driven by UncompressedSave.Write, so a quit without saving drops
        // in-memory registry mutations on purpose.
        SaveScopedState.ResetAll();

        // Everything below resets only on unload, because the feature gates belong to the patch
        // state, the load path reloads the registries from disk instead of clearing them, and the
        // dedup sets live for the whole mod load. LogHelper stays after UnpatchAll, or the
        // transpiler re-runs that unpatching triggers print their once-only lines again.
        DisableMultiPass();
        BurnMenuLauncher.Enabled = false;
        RcsExecRegistry.Reset();
        RcsBurnCompletions.Reset();
        MultiPassRegistry.Reset();
        SaveLoadObserver.Reset();
        Patch_SetTransferInfo.Reset();
        LogHelper.Reset();
#if DEBUG
        PerfTracker.Reset();
#endif

        DefaultCategory.Log.Info("[AFC] Unloaded.");
    }

    private static void OnRcsSaveWritten(string oldSaveId, string newSaveId)
    {
        RcsExecRegistry.RekeyTo(oldSaveId, newSaveId);
        RcsExecRegistry.Save();
    }
}

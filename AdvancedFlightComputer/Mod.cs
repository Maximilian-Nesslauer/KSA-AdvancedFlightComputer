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

    private static readonly FeaturePatchSet _patches = new("com.maxi.advancedflightcomputer");
    private static bool _maneuverTypesInjected;

    [StarMapAllModsLoaded]
    public void OnFullyLoaded()
    {
        string gameVersion = VersionInfo.Current.VersionString;
        DefaultCategory.Log.Info($"[AFC] Game version: {gameVersion}");
        if (gameVersion != TestedGameVersion)
            DefaultCategory.Log.Warning(
                $"[AFC] Tested against {TestedGameVersion}, current is {gameVersion}. Some features may not work correctly.");

        bool coreReady = Validated("Core", GameReflection.ValidateCore)
            && _patches.TryApply("Core", PatchCore);

        if (Validated("HyperbolicTargets", GameReflection.ValidateHyperbolicTargets))
            _patches.TryApply("HyperbolicTargets", HyperbolicTargets.ApplyPatches);

        if (Validated("ManeuverTools", GameReflection.ValidateManeuverTools))
        {
            // The quick-tools and MultiPass are separate blocks so that a MultiPass failure does
            // not roll back the quick-tools.
            if (!_patches.TryApply("ManeuverTools", PatchManeuverTools))
                DisableManeuverTools();
            else if (coreReady && Validated("MultiPass", GameReflection.ValidateMultiPass))
            {
                SharedVehicleHooks.MultiPassEnabled = _patches.TryApply("MultiPass", PatchMultiPass);
                if (!SharedVehicleHooks.MultiPassEnabled)
                    DisableMultiPass();
            }
        }

        if (coreReady && Validated("RcsTranslation", GameReflection.ValidateRcsTranslation))
        {
            SharedVehicleHooks.RcsEnabled = _patches.TryApply("RcsTranslation", PatchRcsTranslation);
            if (!SharedVehicleHooks.RcsEnabled)
                DisableRcsTranslation();
        }

        DefaultCategory.Log.Info("[AFC] Loaded and patched.");
    }

    private static bool Validated(string feature, Func<bool> validate)
    {
        if (validate())
            return true;
        DefaultCategory.Log.Warning($"[AFC] {feature} disabled - reflection targets not found.");
        return false;
    }

    private static void PatchCore(Harmony harmony)
    {
        SharedVehicleHooks.ApplyPatches(harmony);
        SaveLoadObserver.ApplyPatches(harmony);
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
        harmony.CreateClassProcessor(typeof(RcsSetEnumPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsGaugePatches.IsDisabledPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsGaugePatches.PackDataPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsGaugePatches.HoveredPatch)).Patch();
        harmony.CreateClassProcessor(typeof(RcsBurnWindowUi)).Patch();
        harmony.CreateClassProcessor(typeof(RcsBurnCanvasUi)).Patch();

        RcsExecRegistry.Init();
        SaveLoadObserver.SaveLoaded += RcsExecRegistry.Load;
        SaveLoadObserver.SaveWritten += OnRcsSaveWritten;
    }

    private static void DisableRcsTranslation()
    {
        SaveLoadObserver.SaveLoaded -= RcsExecRegistry.Load;
        SaveLoadObserver.SaveWritten -= OnRcsSaveWritten;
        RcsExecRegistry.Reset();
        RcsCommandChannel.Reset();
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
        SharedVehicleHooks.Reset();
        _patches.UnpatchAll();
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

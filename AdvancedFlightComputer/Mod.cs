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
    private static Harmony? _harmony;
    private static bool _maneuverTypesInjected;

    private const string TestedGameVersion = "v2026.7.10.5056";

    [StarMapAllModsLoaded]
    public void OnFullyLoaded()
    {
        string gameVersion = VersionInfo.Current.VersionString;
        DefaultCategory.Log.Info($"[AFC] Game version: {gameVersion}");
        if (gameVersion != TestedGameVersion)
            DefaultCategory.Log.Warning(
                $"[AFC] Tested against {TestedGameVersion}, current is {gameVersion}. Some features may not work correctly.");

        _harmony = new Harmony("com.maxi.advancedflightcomputer");
        bool saveObserverApplied = false;

        if (GameReflection.ValidateHyperbolicTargets())
            HyperbolicTargets.ApplyPatches(_harmony);
        else
            DefaultCategory.Log.Warning("[AFC] HyperbolicTargets disabled - reflection targets not found.");

        if (GameReflection.ValidateManeuverTools())
        {
            ManeuverTools.InjectTransferTypes();
            _maneuverTypesInjected = true;
            ManeuverTools.ApplyPatches(_harmony);

            // Without PassCompletionPatch an execution started from the UI
            // has no way to advance to pass 2; gate MultiPassUI.Enabled
            // inside the validation block so it stays hidden if patching
            // is impossible.
            if (GameReflection.ValidateMultiPass())
            {
                MultiPassRegistry.Init();
                _harmony.CreateClassProcessor(typeof(PassCompletionPatch)).Patch();
                _harmony.CreateClassProcessor(typeof(VehicleDisposePatch)).Patch();
                SaveLoadObserver.ApplyPatches(_harmony);
                saveObserverApplied = true;
                MultiPassUI.Enabled = true;

                // Hohmann multi-pass needs the transpiler to inject the
                // inline UI into stock's DrawPlanWindow. Without the
                // anchor we'd patch but never draw, so gate both the
                // patch application and the UI flag on it.
                if (Patch_DrawPlanWindow_HohmannMultiPass.IsAnchorPresent)
                {
                    _harmony.CreateClassProcessor(typeof(Patch_DrawPlanWindow_HohmannMultiPass)).Patch();
                    HohmannMultiPassUI.Enabled = true;

                    // Flyby targeting draws inside the same inline injection and
                    // rides the same Create interceptor, so it shares this anchor.
                    HohmannFlybyUI.Enabled = true;
                    DefaultCategory.Log.Info(
                        "[AFC] Flyby targeting enabled (Hohmann plan window).");

                    // Hides stock's center-aimed (impact) preview while a flyby is
                    // armed, so the 3D view shows only what Create will fly. Lines
                    // and markers are separate stock methods on one toggle, hence
                    // two patches. The flyby itself still works without them, just
                    // alongside stock's overlay, so a missing anchor is a warning
                    // rather than a gate.
                    if (Patch_TransferPlanner_DrawSelectedTransfer_Flyby.IsAnchorPresent)
                        _harmony.CreateClassProcessor(
                            typeof(Patch_TransferPlanner_DrawSelectedTransfer_Flyby)).Patch();
                    else
                        DefaultCategory.Log.Warning(
                            "[AFC] Flyby stock-preview suppression disabled - " +
                            "DrawSelectedTransfer anchor not found.");

                    if (Patch_TransferPlanner_DrawSelectedTransferUi_Flyby.IsAnchorPresent)
                        _harmony.CreateClassProcessor(
                            typeof(Patch_TransferPlanner_DrawSelectedTransferUi_Flyby)).Patch();
                    else
                        DefaultCategory.Log.Warning(
                            "[AFC] Flyby stock-marker suppression disabled - " +
                            "DrawSelectedTransferUi anchor not found.");

                    // Fallback DrawInline injection at the outermost
                    // ImGui.End() in DrawPlanWindow. Fires regardless of
                    // stock's _transferCalculated state so the active-
                    // exec status + Cancel button stay reachable after
                    // F4 close + reopen (where the primary injection
                    // above would be gated out). DrawInline self-dedups
                    // per ImGui frame so the normal-flow render does not
                    // double up. Nested under the primary anchor check
                    // because without DrawInline being enabled the
                    // fallback would modify IL for no rendered effect.
                    if (Patch_DrawPlanWindow_HohmannFallback.IsAnchorPresent)
                        _harmony.CreateClassProcessor(typeof(Patch_DrawPlanWindow_HohmannFallback)).Patch();
                    else
                        DefaultCategory.Log.Warning(
                            "[AFC] HohmannFallback disabled - ImGui.End anchor not found.");
                }
                else
                    DefaultCategory.Log.Warning(
                        "[AFC] HohmannMultiPass disabled - DrawCorrectionTransfer anchor not found.");

                // Second transpiler: intercepts the stock Create button's
                // Burn.Create call so a single button click routes to
                // multi-pass when armed for N>1 or falls through to a
                // single burn otherwise. Without this the inline UI
                // would need its own Create button.
                if (Patch_DrawPlanWindow_CreateInterceptor.IsAnchorPresent)
                    _harmony.CreateClassProcessor(typeof(Patch_DrawPlanWindow_CreateInterceptor)).Patch();
                else
                    DefaultCategory.Log.Warning(
                        "[AFC] HohmannCreateInterceptor disabled - Burn.Create anchor not found.");

                // 3D orbit-line overlay for the multi-pass preview, gated
                // on stock's "Preview Selected Transfer" checkbox.
                _harmony.CreateClassProcessor(typeof(Patch_TransferPlanner_OnPreRender_Hohmann)).Patch();

                // Per-pass marker overlay (Ap / Pe / AN / DN / SOI / closest).
                // Postfix on DrawPlanWindow so the markers keep rendering
                // after F4 close + reopen (where the older anchor on
                // private DrawSelectedTransferUi would not fire because
                // _transferCalculated is false).
                _harmony.CreateClassProcessor(typeof(Patch_TransferPlanner_DrawPlanWindow_HohmannMarkers)).Patch();
            }
            else
                DefaultCategory.Log.Warning(
                    "[AFC] MultiPass disabled - reflection targets not found.");
        }
        else
            DefaultCategory.Log.Warning("[AFC] ManeuverTools disabled - reflection targets not found.");

        // Independent of ManeuverTools: RCS translation only needs the
        // shared save/tick hooks plus the gauge button internals. The patch
        // block is guarded because several targets are attribute-bound
        // (ComputeControl, SetEnum, the gauge methods) and outside the
        // reflection validation; a game-side rename must degrade this
        // feature with a warning, not abort the remaining mod load.
        if (GameReflection.ValidateRcsTranslation())
        {
            try
            {
                _harmony.CreateClassProcessor(typeof(RcsComputeControlPatch)).Patch();
                _harmony.CreateClassProcessor(typeof(RcsDriverPatch)).Patch();
                _harmony.CreateClassProcessor(typeof(RcsSetEnumPatch)).Patch();
                _harmony.CreateClassProcessor(typeof(RcsVehicleDisposePatch)).Patch();
                _harmony.CreateClassProcessor(typeof(RcsGaugePatches.IsDisabledPatch)).Patch();
                _harmony.CreateClassProcessor(typeof(RcsGaugePatches.PackDataPatch)).Patch();
                _harmony.CreateClassProcessor(typeof(RcsGaugePatches.HoveredPatch)).Patch();
                _harmony.CreateClassProcessor(typeof(RcsBurnWindowUi)).Patch();
                _harmony.CreateClassProcessor(typeof(RcsBurnCanvasUi)).Patch();

                RcsExecRegistry.Init();
                if (!saveObserverApplied)
                    SaveLoadObserver.ApplyPatches(_harmony);
                SaveLoadObserver.SaveLoaded += OnRcsSaveLoaded;
                SaveLoadObserver.SaveWritten += OnRcsSaveWritten;
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] RcsTranslation disabled - patching failed (game version may have changed): {ex}");
            }
        }
        else
            DefaultCategory.Log.Warning("[AFC] RcsTranslation disabled - reflection targets not found.");

        DefaultCategory.Log.Info("[AFC] Loaded and patched.");
    }

    [StarMapUnload]
    public void Unload()
    {
        _harmony?.UnpatchAll(_harmony.Id);
        _harmony = null;

        if (_maneuverTypesInjected)
        {
            ManeuverTools.RemoveTransferTypes();
            _maneuverTypesInjected = false;
        }

        // No MultiPassRegistry.Save() here: persistence is driven by
        // UncompressedSave.Write events so a quit without KSA-saving
        // intentionally drops in-memory mutations.

        // Everything scoped to one save game, shared with SaveLoadObserver's
        // load path so the two lists cannot drift.
        SaveScopedState.ResetAll();

        // Unload-only: the feature gates, the registries the load path reloads
        // from disk instead of clearing, and the dedup sets whose lifetime is the
        // mod load rather than one save.
        MultiPassUI.Enabled = false;
        HohmannMultiPassUI.Enabled = false;
        HohmannFlybyUI.Enabled = false;
        RcsExecRegistry.Reset();
        RcsCommandChannel.Reset();
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

    private static void OnRcsSaveLoaded()
    {
        RcsCommandChannel.Reset();
        RcsExecRegistry.Load();
    }

    private static void OnRcsSaveWritten(string newSaveId)
    {
        RcsExecRegistry.RekeyTransientsTo(newSaveId);
        RcsExecRegistry.Save();
    }
}

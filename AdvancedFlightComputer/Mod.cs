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

    private const string TestedGameVersion = "v2026.9.7.5402";

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
        bool saveObserverApplied = false;

        // Every feature block runs through TryPatchBlock: several patch targets
        // are attribute-bound and outside the reflection validation, so a
        // game-side rename throws out of CreateClassProcessor().Patch() where
        // the validation gate cannot catch it. An escaping exception would
        // abort the rest of this hook AND every later mod's AllModsLoaded hook
        // (StarMap invokes them in one unguarded loop), so each feature
        // degrades on its own instead.
        if (GameReflection.ValidateHyperbolicTargets())
            TryPatchBlock("HyperbolicTargets", () => HyperbolicTargets.ApplyPatches(harmony));
        else
            DefaultCategory.Log.Warning("[AFC] HyperbolicTargets disabled - reflection targets not found.");

        if (GameReflection.ValidateManeuverTools())
        {
            // Two blocks, not one: a failure in the MultiPass half must not
            // roll back the quick-tools whose own window prefix applied fine.
            bool maneuverPatched = TryPatchBlock("ManeuverTools", () =>
            {
                ManeuverTools.InjectTransferTypes();
                _maneuverTypesInjected = true;
                ManeuverTools.ApplyPatches(harmony);
            });
            if (!maneuverPatched)
            {
                // With the DrawPlanWindow prefix possibly missing, the injected
                // types would sit in stock's dropdown with no window body.
                BurnMenuLauncher.Enabled = false;
                if (_maneuverTypesInjected)
                {
                    ManeuverTools.RemoveTransferTypes();
                    _maneuverTypesInjected = false;
                }
            }
            // Without PassCompletionPatch an execution started from the UI
            // has no way to advance to pass 2; gate MultiPassUI.Enabled
            // inside the validation block so it stays hidden if patching
            // is impossible.
            else if (GameReflection.ValidateMultiPass())
            {
                bool multiPassPatched = TryPatchBlock("MultiPass", () =>
                {
                    MultiPassRegistry.Init();
                    harmony.CreateClassProcessor(typeof(PassCompletionPatch)).Patch();
                    harmony.CreateClassProcessor(typeof(VehicleDisposePatch)).Patch();
                    // Flag set BEFORE the call: ApplyPatches has no idempotence
                    // guard, and a partial failure inside it must not make the
                    // RCS block register a second LoadPatch. The trade (a
                    // half-applied pair is not retried there) keeps
                    // SaveLoadObserver's one-patch-pair contract intact.
                    saveObserverApplied = true;
                    SaveLoadObserver.ApplyPatches(harmony);
                    MultiPassUI.Enabled = true;

                    // Hohmann multi-pass needs the transpiler to inject the
                    // inline UI into stock's DrawPlanWindow. Without the
                    // anchor we'd patch but never draw, so gate both the
                    // patch application and the UI flag on it.
                    if (Patch_DrawPlanWindow_HohmannMultiPass.IsAnchorPresent)
                    {
                        harmony.CreateClassProcessor(typeof(Patch_DrawPlanWindow_HohmannMultiPass)).Patch();
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
                            harmony.CreateClassProcessor(
                                typeof(Patch_TransferPlanner_DrawSelectedTransfer_Flyby)).Patch();
                        else
                            DefaultCategory.Log.Warning(
                                "[AFC] Flyby stock-preview suppression disabled - " +
                                "DrawSelectedTransfer anchor not found.");

                        if (Patch_TransferPlanner_DrawSelectedTransferUi_Flyby.IsAnchorPresent)
                            harmony.CreateClassProcessor(
                                typeof(Patch_TransferPlanner_DrawSelectedTransferUi_Flyby)).Patch();
                        else
                            DefaultCategory.Log.Warning(
                                "[AFC] Flyby stock-marker suppression disabled - " +
                                "DrawSelectedTransferUi anchor not found.");

                        // Fallback DrawInline injection before the
                        // ConsoleStyle.PopWidgetStyle that closes DrawPlanWindow's
                        // body, so the UI still lands inside the body child with
                        // the widget style pushed. Fires regardless of stock's
                        // _transferCalculated state so the active-exec status +
                        // Cancel button stay reachable after F4 close + reopen
                        // (where the primary injection above would be gated out).
                        // DrawInline self-dedups per ImGui frame so the
                        // normal-flow render does not double up. Nested under the
                        // primary anchor check because without DrawInline being
                        // enabled the fallback would modify IL for no rendered
                        // effect.
                        if (Patch_DrawPlanWindow_HohmannFallback.IsAnchorPresent)
                            harmony.CreateClassProcessor(typeof(Patch_DrawPlanWindow_HohmannFallback)).Patch();
                        else
                            DefaultCategory.Log.Warning(
                                "[AFC] HohmannFallback disabled - ConsoleStyle.PopWidgetStyle anchor not found.");
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
                        harmony.CreateClassProcessor(typeof(Patch_DrawPlanWindow_CreateInterceptor)).Patch();
                    else
                        DefaultCategory.Log.Warning(
                            "[AFC] HohmannCreateInterceptor disabled - Burn.Create anchor not found.");

                    // 3D orbit-line overlay for the multi-pass preview, gated
                    // on stock's "Preview Selected Transfer" checkbox.
                    harmony.CreateClassProcessor(typeof(Patch_TransferPlanner_OnPreRender_Hohmann)).Patch();

                    // Per-pass marker overlay (Ap / Pe / AN / DN / SOI / closest).
                    // Postfix on DrawPlanWindow so the markers keep rendering
                    // after F4 close + reopen (where the older anchor on
                    // private DrawSelectedTransferUi would not fire because
                    // _transferCalculated is false).
                    harmony.CreateClassProcessor(typeof(Patch_TransferPlanner_DrawPlanWindow_HohmannMarkers)).Patch();
                });

                // A mid-block failure can leave a UI flag set for a patch that
                // never applied; the flags are safe to clear wholesale because
                // every one of them belongs to this block. The quick-tools
                // stay up: their own block already succeeded.
                if (!multiPassPatched)
                {
                    MultiPassUI.Enabled = false;
                    HohmannMultiPassUI.Enabled = false;
                    HohmannFlybyUI.Enabled = false;
                }
            }
            else
                DefaultCategory.Log.Warning(
                    "[AFC] MultiPass disabled - reflection targets not found.");
        }
        else
            DefaultCategory.Log.Warning("[AFC] ManeuverTools disabled - reflection targets not found.");

        // Independent of ManeuverTools: RCS translation only needs the
        // shared save/tick hooks plus the gauge button internals.
        if (GameReflection.ValidateRcsTranslation())
        {
            TryPatchBlock("RcsTranslation", () =>
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
                if (!saveObserverApplied)
                    SaveLoadObserver.ApplyPatches(harmony);
                SaveLoadObserver.SaveLoaded += OnRcsSaveLoaded;
                SaveLoadObserver.SaveWritten += OnRcsSaveWritten;
            });
        }
        else
            DefaultCategory.Log.Warning("[AFC] RcsTranslation disabled - reflection targets not found.");

        DefaultCategory.Log.Info("[AFC] Loaded and patched.");
    }

    /// <summary>Runs one feature's patch block and contains any patching
    /// failure to that feature. The message is deliberately honest about the
    /// partial state: Harmony applies patches one by one, so everything before
    /// the failing call stays live and there is no rollback here (per-feature
    /// Harmony ids would enable one; tracked in the backlog).</summary>
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
        BurnMenuLauncher.Enabled = false;
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

    private static void OnRcsSaveWritten(string oldSaveId, string newSaveId)
    {
        RcsExecRegistry.RekeyTo(oldSaveId, newSaveId);
        RcsExecRegistry.Save();
    }
}

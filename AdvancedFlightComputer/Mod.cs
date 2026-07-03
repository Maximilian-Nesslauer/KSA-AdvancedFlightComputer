using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.HyperbolicTargets;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
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

    private const string TestedGameVersion = "v2026.7.3.4826";

    [StarMapAllModsLoaded]
    public void OnFullyLoaded()
    {
        string gameVersion = VersionInfo.Current.VersionString;
        DefaultCategory.Log.Info($"[AFC] Game version: {gameVersion}");
        if (gameVersion != TestedGameVersion)
            DefaultCategory.Log.Warning(
                $"[AFC] Tested against {TestedGameVersion}, current is {gameVersion}. Some features may not work correctly.");

        _harmony = new Harmony("com.maxi.advancedflightcomputer");

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
                MultiPassUI.Enabled = true;

                // Hohmann multi-pass needs the transpiler to inject the
                // inline UI into stock's DrawPlanWindow. Without the
                // anchor we'd patch but never draw, so gate both the
                // patch application and the UI flag on it.
                if (Patch_DrawPlanWindow_HohmannMultiPass.IsAnchorPresent)
                {
                    _harmony.CreateClassProcessor(typeof(Patch_DrawPlanWindow_HohmannMultiPass)).Patch();
                    HohmannMultiPassUI.Enabled = true;

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
        Patch_DrawPlanWindow.Reset();
        MultiPassUI.Enabled = false;
        MultiPassUI.Reset();
        HohmannMultiPassUI.Enabled = false;
        HohmannMultiPassUI.Reset();
        MultiPassPreviewCache.Reset();
        MultiPassRegistry.Reset();
        PassCompletionPatch.Reset();
        SaveLoadObserver.Reset();
        ManeuverToolsWindow.Reset();
        Patch_AlignmentTime.Reset();
        LogHelper.Reset();
#if DEBUG
        PerfTracker.Reset();
#endif

        DefaultCategory.Log.Info("[AFC] Unloaded.");
    }
}

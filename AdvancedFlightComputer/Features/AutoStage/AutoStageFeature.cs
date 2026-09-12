using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

internal static class AutoStageFeature
{
    // The marker the standalone AutoStage mod injects at its own immediate load.
    private const string StandaloneToggleName = "AutoStageToggle";

    private static bool _enumInjected;

    internal static bool GaugeEnumInjected =>
        TryGetEnumTypes(out List<EnumTypeOption> list) && list.Any(o => o.Type == typeof(AfcAutoStageToggle));

    // Runs at immediate load, before the game reads Gauges.xml and binds the AUTOSTAGE button's
    // Action="AfcAutoStageToggle" once. The binding cannot be undone later, so a build whose
    // staging keys do not resolve gets no entry and one warning instead of a live-looking button.
    internal static void InjectGaugeEnumAtLoad()
    {
        if (!GameReflection.ValidateAutoStage())
        {
            DefaultCategory.Log.Warning("[AFC] AutoStage reflection keys did not resolve; the AUTOSTAGE gauge button is not registered.");
            return;
        }
        if (!InjectGaugeEnum())
            DefaultCategory.Log.Warning("[AFC] GaugeButtonFlightComputer.EnumTypes is not a List<EnumTypeOption>; the AUTOSTAGE gauge button is not registered.");
    }

    // Idempotent, because a reload would otherwise add a second entry.
    internal static bool InjectGaugeEnum()
    {
        if (!TryGetEnumTypes(out List<EnumTypeOption> list))
            return false;
        if (!list.Any(o => o.Type == typeof(AfcAutoStageToggle)))
            list.Add(new EnumTypeOption(typeof(AfcAutoStageToggle)));
        _enumInjected = true;
        return true;
    }

    internal static void RemoveGaugeEnum()
    {
        if (_enumInjected && TryGetEnumTypes(out List<EnumTypeOption> list))
            list.RemoveAll(o => o.Type == typeof(AfcAutoStageToggle));
        _enumInjected = false;
    }

    // Two stagers on one burnout would activate two rows, so the built-in one stands down while
    // the standalone mod is installed. Its marker is visible here because every immediate load
    // runs before any AllModsLoaded hook.
    internal static bool StandaloneModAbsent()
    {
        if (!TryGetEnumTypes(out List<EnumTypeOption> list))
            return true;
        foreach (EnumTypeOption option in list)
        {
            if (option.Type.Name == StandaloneToggleName)
            {
                DefaultCategory.Log.Warning(
                    "[AFC] The standalone AutoStage mod is installed, so AFC's built-in automatic staging stays off and its AUTOSTAGE button does nothing. Remove AutoStage; AdvancedFlightComputer contains it.");
                return false;
            }
        }
        return true;
    }

    internal static void ApplyPatches(Harmony harmony)
    {
        if (!InjectGaugeEnum())
            throw new InvalidOperationException("GaugeButtonFlightComputer.EnumTypes is not a List<EnumTypeOption>.");
        StagingConfig.Init();
        harmony.CreateClassProcessor(typeof(AutoStageGaugePatches.TogglePatch)).Patch();
        harmony.CreateClassProcessor(typeof(AutoStageGaugePatches.IsSetPatch)).Patch();
        harmony.CreateClassProcessor(typeof(AutoStageGaugePatches.IsDisabledPatch)).Patch();
        harmony.CreateClassProcessor(typeof(SequenceListPatches.ActivateNextSequencePatch)).Patch();
        harmony.CreateClassProcessor(typeof(SequenceListPatches.ResetCachesPatch)).Patch();
        harmony.CreateClassProcessor(typeof(StagingDelayPartWindow)).Patch();
        ModSettingsPage.Register(AutoStageSettingsPage.DrawSection);
    }

    internal static void Disable()
    {
        ModSettingsPage.Unregister(AutoStageSettingsPage.DrawSection);
        StagingDetector.Reset();
        AutoStageSettingsPage.Reset();
        StagingConfig.Reset();
    }

    private static bool TryGetEnumTypes(out List<EnumTypeOption> list)
    {
        list = (GameReflection.GaugeButtonFlightComputer_EnumTypes?.GetValue(null) as List<EnumTypeOption>)!;
        return list != null;
    }
}

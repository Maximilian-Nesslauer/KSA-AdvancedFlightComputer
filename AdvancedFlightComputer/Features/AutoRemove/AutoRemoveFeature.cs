using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.RcsTranslation;
using Brutal.ImGuiApi;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoRemove;

internal static class AutoRemoveFeature
{
    private const string StandaloneModType = "AutoRemoveFinishedBurns.Mod";

    // No patch of its own. The tick runs through SharedVehicleHooks, the RCS completion through
    // AFC's own event, and the settings section through ModSettingsPage.
    internal static void ApplyPatches(Harmony harmony)
    {
        AutoRemoveConfig.Init();
        RcsBurnCompletions.Completed += FinishedBurnRemover.OnRcsBurnCompleted;
        ModSettingsPage.Register(DrawSection);
    }

    // The standalone mod removes the same burn a tick earlier or later, which the plan lookup
    // tolerates, so this is a hint rather than a stand-down.
    internal static void WarnIfStandaloneInstalled()
    {
        if (AccessTools.TypeByName(StandaloneModType) != null)
            DefaultCategory.Log.Warning(
                "[AFC] The standalone AutoRemoveFinishedBurns mod is installed next to AFC's built-in copy. Remove it; AdvancedFlightComputer contains it.");
    }

    internal static void Disable()
    {
        RcsBurnCompletions.Completed -= FinishedBurnRemover.OnRcsBurnCompleted;
        ModSettingsPage.Unregister(DrawSection);
        FinishedBurnRemover.Reset();
        AutoRemoveConfig.Reset();
    }

    internal static void DrawSection()
    {
        ConsoleWidgets.Rule();
        ConsoleWidgets.RegionHeader("AUTO REMOVE FINISHED BURNS".AsSpan());

        bool enabled = AutoRemoveConfig.Enabled;
        if (ConsoleUi.CheckboxRow("ENABLED".AsSpan(), "AfcAutoRemoveEnabled".AsSpan(), ref enabled))
        {
            AutoRemoveConfig.Enabled = enabled;
            AutoRemoveConfig.Save();
        }

        ImGui.TextWrapped(
            "When on, finished auto-burns and RCS burns are removed from the burn plan. Detection only fires " +
            "for completed burns, never manual ones. Out-of-fuel cases are left in place so you can resume them after staging.");
    }
}

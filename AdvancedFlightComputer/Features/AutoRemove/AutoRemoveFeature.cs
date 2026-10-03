using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoRemove;

internal static class AutoRemoveFeature
{
    private const string StandaloneModType = "AutoRemoveFinishedBurns.Mod";

    // No patch of its own.
    // A finished burn arrives through SharedVehicleHooks, and the settings section through ModSettingsPage.
    internal static void ApplyPatches(Harmony harmony)
    {
        AutoRemoveConfig.Init();
        ModSettingsPage.Register(DrawSection);
    }

    // The standalone mod removes a finished burn before FlightComputer.RaisePendingAlerts runs, and FlightComputer.EndAutoBurn then removes the next planned burn as well.
    // AFC cannot stop that from here, so the warning names the burn loss.
    internal static void WarnIfStandaloneInstalled()
    {
        if (AccessTools.TypeByName(StandaloneModType) != null)
            DefaultCategory.Log.Warning(
                "[AFC] The standalone AutoRemoveFinishedBurns mod is installed next to AFC's built-in copy. Remove the standalone mod, because together with the game's own burn removal it also deletes the next planned burn, and AdvancedFlightComputer already removes finished burns.");
    }

    internal static void Disable()
    {
        ModSettingsPage.Unregister(DrawSection);
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
            "The game removes a finished or stopped auto-burn by itself when another burn follows it. When on, " +
            "the last finished burn is removed as well, for engine and RCS burns alike. Manual burns, and a " +
            "last burn that stopped without propellant, stay in the plan.");
    }
}

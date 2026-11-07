using AdvancedFlightComputer.Core;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.AutoRemove;

internal static class AutoRemoveFeature
{
    private const string StandaloneModType = "AutoRemoveFinishedBurns.Mod";

    // No patch of its own.
    internal static void ApplyPatches(Harmony harmony)
    {
        AutoRemoveConfig.Init();
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
        AutoRemoveConfig.Reset();
    }

    /// <summary>The AUTO REMOVE FINISHED BURNS switch, saved to autoremove.toml on a click.</summary>
    internal static void DrawSwitch()
    {
        bool enabled = AutoRemoveConfig.Enabled;
        if (ConsoleUi.CheckboxRow("AUTO REMOVE FINISHED BURNS".AsSpan(), "AfcAutoRemoveEnabled".AsSpan(), ref enabled,
                ("The game removes a finished or stopped auto-burn by itself when another burn follows it, but keeps " +
                 "a burn that stopped because a stage ran dry and a later stage can continue it. When on, " +
                 "the last finished burn is removed as well, for engine and RCS burns alike. Manual burns, and a " +
                 "last burn that stopped without propellant, stay in the plan.\n" + Ui.Logic.PageText.TagEveryCraft).AsSpan()))
        {
            AutoRemoveConfig.Enabled = enabled;
            AutoRemoveConfig.Save();
        }
    }
}

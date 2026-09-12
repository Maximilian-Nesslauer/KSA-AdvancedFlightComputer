#if DEBUG
using System.Diagnostics;
#endif
using System.Reflection;
using System.Reflection.Emit;
using AutoRemoveFinishedBurns.Core;
using Brutal.ImGuiApi;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AutoRemoveFinishedBurns.Features;

/// <summary>
/// Appends the mod's settings to the Mods page of the game's Settings window.
///
/// The settings window has no tab bar any more; it is a nav rail whose pages all
/// render into one body child, closed by a single ConsoleStyle.PopWidgetStyle.
/// Inserting the drawer call before that lands inside the body with the console
/// widget style still pushed, and it composes with any other mod doing the same
/// because nothing is replaced. The drawer checks which page is open, since the
/// whole body is one code path now.
/// </summary>
[HarmonyPatch(typeof(GameSettings), nameof(GameSettings.OnDrawUi),
    new[] { typeof(Camera) })]
static class SettingsTabPatch
{
    // Internal so the HarnessTests consumer can run it against the live game IL as a drift check.
    [HarmonyTranspiler]
    internal static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        var codes = new List<CodeInstruction>(instructions);
        MethodInfo? anchor = GameReflection.ConsoleStyle_PopWidgetStyle;
        MethodInfo drawerCall = AccessTools.Method(typeof(SettingsTabPatch),
            nameof(DrawSettingsPage), Type.EmptyTypes)!;

        if (anchor == null)
        {
            DefaultCategory.Log.Warning(
                "[AutoRemoveFinishedBurns] Transpiler: ConsoleStyle.PopWidgetStyle not found. " +
                "Settings page not patched.");
            return codes;
        }

        int anchorIdx = -1;
        for (int i = 0; i < codes.Count; i++)
        {
            if (codes[i].Calls(anchor))
            {
                anchorIdx = i;
                break;
            }
        }

        if (anchorIdx < 0)
        {
            DefaultCategory.Log.Warning(
                $"[AutoRemoveFinishedBurns] Transpiler: no ConsoleStyle.PopWidgetStyle() call " +
                $"in GameSettings.OnDrawUi (scanned {codes.Count} IL instructions). " +
                "Settings page not patched.");
            return codes;
        }

        // Insert, do not replace. Labels stay on the anchor so jumps targeting
        // it skip our drawer instead of landing mid-call.
        codes.Insert(anchorIdx, new CodeInstruction(OpCodes.Call, drawerCall));

        return codes;
    }

    public static void DrawSettingsPage()
    {
        if (!GameReflection.IsModsSettingsPageOpen())
            return;
#if DEBUG
        long perfStart = DebugConfig.Performance ? Stopwatch.GetTimestamp() : 0;
#endif
        try
        {
            ConsoleWidgets.Rule();
            ConsoleWidgets.RegionHeader("AUTO REMOVE FINISHED BURNS".AsSpan());
            DrawSettings();
        }
        catch (Exception ex)
        {
            LogHelper.ErrorOnce("Settings.Draw:" + ex.GetType().Name,
                $"[AutoRemoveFinishedBurns] Settings draw threw: {ex}");
        }
#if DEBUG
        if (DebugConfig.Performance)
            PerfTracker.Record("SettingsTabPatch.DrawSettingsPage",
                Stopwatch.GetTimestamp() - perfStart);
#endif
    }

    private static void DrawSettings()
    {
        bool enabled = Config.Enabled;
        ConsoleWidgets.BeginRow("ENABLED".AsSpan());
        if (ConsoleWidgets.Checkbox("ArfbEnabled".AsSpan(), ref enabled, pending: false))
        {
            Config.Enabled = enabled;
            Config.Save();
        }
        ConsoleWidgets.EndRow();

        ImGui.TextWrapped(
            "When on, finished auto-burns are automatically removed from the " +
            "burn plan. Detection only fires for completed auto-burns, never " +
            "manual burns. Out-of-fuel cases are left in place so you can " +
            "resume them after staging.");
    }
}

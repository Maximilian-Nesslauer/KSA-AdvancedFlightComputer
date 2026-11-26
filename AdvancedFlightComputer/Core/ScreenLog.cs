using System.Reflection;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// LOG LINES ON SCREEN (AFC menu and TUNE > PREFERENCES), off by default: off stops the game's console from showing
/// new log lines over the flight while it is closed, KSA's and AFC's alike, for footage. Nothing is
/// lost: the lines still go to the log file and to the console's history (open it with its key).
/// It uses the console's own switch, ConsoleWindow.DisableMessageFading, which makes a new line's
/// time on screen zero. KSA turns showing back on when loading completes, so the switch is held
/// every frame (Mod.DrawGui). Reached by reflection, so AFC needs no reference to
/// Brutal.ImGui.Abstractions; on a build without them the switch does nothing.
/// </summary>
internal static class ScreenLog
{
    internal const string Key = "log_lines_on_screen";

    private static bool _hiding;

    internal static bool Shown => UiConfig.GetBool(Key, false);

    internal static void Set(bool shown)
    {
        UiConfig.Set(Key, shown);
        Apply();
    }

    /// <summary>Once per frame: holds the console's on-screen lines off while the switch is off, and gives them back once when it is switched on.</summary>
    internal static void Apply()
    {
        object? console = ConsoleField?.GetValue(null);
        if (console == null)
            return;
        if (!Shown)
        {
            Disable?.Invoke(console, null);
            _hiding = true;
        }
        else if (_hiding)
        {
            Enable?.Invoke(console, null);
            _hiding = false;
        }
    }

    /// <summary>Restores stock on-screen console log fading when the mod unloads.</summary>
    internal static void RestoreOnUnload()
    {
        if (!_hiding)
            return;
        object? console = ConsoleField?.GetValue(null);
        if (console != null)
            Enable?.Invoke(console, null);
        _hiding = false;
    }

    private static readonly FieldInfo? ConsoleField =
        typeof(Program).GetField("ConsoleWindow", BindingFlags.Public | BindingFlags.Static);

    private static readonly MethodInfo? Disable =
        ConsoleField?.FieldType.GetMethod("DisableMessageFading", BindingFlags.Public | BindingFlags.Instance, Type.EmptyTypes);

    private static readonly MethodInfo? Enable =
        ConsoleField?.FieldType.GetMethod("EnableMessageFading", BindingFlags.Public | BindingFlags.Instance, Type.EmptyTypes);
}

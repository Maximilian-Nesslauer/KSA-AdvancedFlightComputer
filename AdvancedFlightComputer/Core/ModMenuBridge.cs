using Brutal.ImGuiApi;
using Brutal.Logging;

namespace AdvancedFlightComputer.Core;

/// <summary>The AFC entry in the Mods menu of ModMenu, found by attribute name, so AFC needs no reference to ModMenu.</summary>
/// <remarks>ModMenu writes Active in the menu bar pass with no exception handler, so the setter stays trivial and the class has no static initializers.
/// ModMenu also calls any public static CreateModMenu or DrawModMenu, so AFC must not use those names.</remarks>
internal static class ModMenuBridge
{
    internal const string MenuName = "AFC Guidance";

    private static bool _active;
    private static int _hookCalls;
    private static bool _loggedModMenuHost;
    private static bool _loggedTopBarHost;

    /// <summary>Set by ModMenu through reflection. It only turns on, so the menu never shows in both places.</summary>
    internal static bool Active
    {
        get => _active;
        private set
        {
            if (value)
                _active = true;
        }
    }

    /// <summary>The menu items that both hosts draw. Null while the menu block is not loaded.</summary>
    internal static Action? DrawItems;

    [ModMenuEntry(MenuName, nameof(Active))]
    public static void DrawEntry()
    {
        // A call proves that ModMenu hosts the menu.
        _active = true;
        try
        {
            NoteModMenuHost();
            Action? draw = DrawItems;
            if (draw == null)
            {
                ImGui.TextDisabled("This menu did not load. Read the game log."u8);
                return;
            }
            draw();
        }
        catch (Exception ex)
        {
            // ModMenu would log it on every frame.
            LogHelper.WarnOnce($"modmenu-entry:{ex.GetType().Name}", $"[AFC] {MenuName} menu failed in the Mods menu: {ex}");
        }
    }

    /// <summary>ModMenu sets Active after the first hook call, so a false flag at the second call means the top bar hosts the menu.</summary>
    internal static void NoteTopBarHook()
    {
        if (_active)
        {
            NoteModMenuHost();
            return;
        }
        if (_loggedTopBarHost || ++_hookCalls < 2)
            return;
        _loggedTopBarHost = true;
        DefaultCategory.Log.Info($"[AFC] The {MenuName} menu is in the top menu bar.");
    }

    private static void NoteModMenuHost()
    {
        if (_loggedModMenuHost)
            return;
        _loggedModMenuHost = true;
        DefaultCategory.Log.Info($"[AFC] The {MenuName} menu is in the Mods menu of ModMenu.");
    }

    /// <summary>Active stays, because ModMenu writes it only once.</summary>
    internal static void Reset()
    {
        DrawItems = null;
        _hookCalls = 0;
        _loggedModMenuHost = false;
        _loggedTopBarHost = false;
    }
}

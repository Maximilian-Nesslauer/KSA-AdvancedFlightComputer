using System.Reflection;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AutoRemoveFinishedBurns.Core;

static class GameReflection
{
    #region Detection

    // Applies every physics bubble's solver results on the main thread. The
    // parameter list is pinned so a future overload cannot silently take over
    // the patch.
    public static readonly MethodInfo? Universe_ApplyVehicleSolvers =
        AccessTools.Method(typeof(Universe), nameof(Universe.ApplyVehicleSolvers),
            Type.EmptyTypes);

    #endregion

    #region Settings

    public static readonly MethodInfo? GameSettings_OnDrawUi =
        AccessTools.Method(typeof(GameSettings), nameof(GameSettings.OnDrawUi),
            new[] { typeof(Camera) });

    // Closes the settings window body. The settings pages all render into one
    // body child now, so this single call is where a mod section can still be
    // appended with the console widget style pushed.
    public static readonly MethodInfo? ConsoleStyle_PopWidgetStyle =
        AccessTools.Method(typeof(ConsoleStyle), nameof(ConsoleStyle.PopWidgetStyle),
            Type.EmptyTypes);

    // Which settings page the nav rail has open. The enum is private to
    // GameSettings, so the Mods member is resolved as a boxed value once and
    // compared by equality rather than named in code.
    public static readonly FieldInfo? GameSettings_openTab =
        AccessTools.Field(typeof(GameSettings), "_openTab");

    private static readonly object? ModsTab = ResolveModsTab();

    public static bool IsModsSettingsPageOpen()
    {
        FieldInfo? field = GameSettings_openTab;
        return ModsTab != null && field != null && ModsTab.Equals(field.GetValue(null));
    }

    private static object? ResolveModsTab()
    {
        Type? type = GameSettings_openTab?.FieldType;
        if (type == null || !type.IsEnum)
            return null;
        return Enum.TryParse(type, "Mods", out object? value) ? value : null;
    }

    #endregion

    #region Validation

    public static bool ValidateDetection()
    {
        var targets = new (string name, object? target)[]
        {
            ("Universe.ApplyVehicleSolvers()", Universe_ApplyVehicleSolvers),
        };
        return ValidateTargets("Detection", targets);
    }

    public static bool ValidateSettings()
    {
        var targets = new (string name, object? target)[]
        {
            ("GameSettings.OnDrawUi(Camera)", GameSettings_OnDrawUi),
            ("ConsoleStyle.PopWidgetStyle()", ConsoleStyle_PopWidgetStyle),
            ("GameSettings._openTab (Mods page)", ModsTab),
        };
        return ValidateTargets("Settings", targets);
    }

    private static bool ValidateTargets(string feature, (string name, object? target)[] targets)
    {
        bool allOk = true;
        foreach (var (name, target) in targets)
        {
            if (target == null)
            {
                DefaultCategory.Log.Error(
                    $"[AutoRemoveFinishedBurns] {feature}: {name} not found - game version may have changed.");
                allOk = false;
            }
        }
        return allOk;
    }

    #endregion
}

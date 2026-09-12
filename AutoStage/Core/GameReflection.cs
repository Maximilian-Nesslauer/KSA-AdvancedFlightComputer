using System.Reflection;
using Brutal.Logging;
using HarmonyLib;
using KSA;

namespace AutoStage.Core;

static class GameReflection
{
    public static readonly FieldInfo? GaugeButton_EnumTypes =
        AccessTools.Field(typeof(GaugeButtonFlightComputer), "EnumTypes");

    // Closed against System.Enum because PackData calls them on _enumValue
    // typed as Enum.
    public static readonly MethodInfo? Vehicle_IsSet_Enum =
        FindGenericVehicleMethod("IsSet", parameterCount: 2)?.MakeGenericMethod(typeof(System.Enum));

    public static readonly MethodInfo? Vehicle_IsFlightComputerDisabled_Enum =
        FindGenericVehicleMethod("IsFlightComputerDisabled", parameterCount: 1)?.MakeGenericMethod(typeof(System.Enum));

    public static readonly PropertyInfo? SequenceList_ActiveSequence =
        AccessTools.Property(typeof(SequenceList), "ActiveSequence");

    public static readonly FieldInfo? SequenceList_updatingSequence =
        AccessTools.Field(typeof(SequenceList), "_updatingSequence");

    public static readonly FieldInfo? ModLibrary_AllParts =
        AccessTools.Field(typeof(ModLibrary), "AllParts");

    // Pinned to the bool overload: Dispose() only delegates to it, and the EVA-boarding
    // path calls Dispose(endMission: false) directly, so the delegate misses that vehicle.
    public static readonly MethodInfo? Vehicle_Dispose =
        AccessTools.Method(typeof(Vehicle), nameof(Vehicle.Dispose), new[] { typeof(bool) });

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

    public static bool ValidateAll()
    {
        var targets = new (string name, object? target)[]
        {
            ("GaugeButtonFlightComputer.EnumTypes", GaugeButton_EnumTypes),
            ("Vehicle.IsSet<Enum>", Vehicle_IsSet_Enum),
            ("Vehicle.IsFlightComputerDisabled<Enum>", Vehicle_IsFlightComputerDisabled_Enum),
            ("SequenceList.ActiveSequence", SequenceList_ActiveSequence),
            // Core, not IgnitionDelay: every auto-stage runs the split
            // activation, and stock brackets its own activation loop with this
            // flag because SequenceList.ResetCaches early-returns on it. Without
            // it the loop iterates target.Parts while a re-entrant ResetCaches
            // is free to rebuild that cache.
            ("SequenceList._updatingSequence", SequenceList_updatingSequence),
            // Core, not IgnitionDelay: the settings page carries the spent-stage
            // switch, which uses none of the ignition-delay targets.
            ("GameSettings._openTab (Mods page)", ModsTab),
            // Core, not IgnitionDelay: the disposal hook drops the per-vehicle
            // caches every feature builds, not just the delay overrides.
            ("Vehicle.Dispose(bool)", Vehicle_Dispose),
        };

        bool allOk = true;
        foreach (var (name, target) in targets)
        {
            if (target == null)
            {
                DefaultCategory.Log.Error(
                    $"[AutoStage] {name} not found, game version may have changed.");
                allOk = false;
            }
        }
        return allOk;
    }

    public static bool ValidateIgnitionDelay()
    {
        var targets = new (string name, object? target)[]
        {
            ("ModLibrary.AllParts", ModLibrary_AllParts),
        };

        bool allOk = true;
        foreach (var (name, target) in targets)
        {
            if (target == null)
            {
                DefaultCategory.Log.Error(
                    $"[AutoStage] IgnitionDelay: {name} not found, game version may have changed.");
                allOk = false;
            }
        }
        return allOk;
    }

    private static MethodInfo? FindGenericVehicleMethod(string name, int parameterCount)
    {
        foreach (MethodInfo method in typeof(Vehicle).GetMethods())
        {
            if (method.Name == name
                && method.IsGenericMethodDefinition
                && method.GetParameters().Length == parameterCount)
                return method;
        }
        return null;
    }
}

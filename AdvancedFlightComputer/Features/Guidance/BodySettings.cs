using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// Keeps the settings of a craft on the defaults of the body whose sphere of influence it is in (<see cref="BodyDefaults"/>).
/// When the body changes, a value that still stands at the old body's default takes the new body's default, and a value the
/// player changed is left as it is. So "changed" keeps meaning "differs from the default here", for the marks of the pages,
/// their resets and the record a save keeps, and nothing has to remember who wrote a value.
/// </summary>
internal static class BodySettings
{
    /// <summary>Follows the body <paramref name="vehicle"/> is in now. A craft with no parent body keeps the defaults it has.</summary>
    internal static void Follow(VehicleAutopilotState state, Vehicle? vehicle)
    {
        string? body = vehicle?.Orbit?.Parent?.Id;
        if (state == null || string.IsNullOrEmpty(body))
            return;
        Follow(state, body);
    }

    /// <summary>Moves the defaults of <paramref name="state"/> to <paramref name="body"/> and returns the count of values that moved with them.</summary>
    internal static int Follow(VehicleAutopilotState state, string body)
    {
        string from = state.DefaultsBody;
        if (string.Equals(from, body, StringComparison.Ordinal))
            return 0;

        int moved = 0;
        foreach (SettingSpec spec in GuidanceSettingSpecs.AllDoubles)
        {
            if (!BodyDefaults.Overrides(from, spec.Key) && !BodyDefaults.Overrides(body, spec.Key))
                continue;
            // A value another writer owns (the chase of a target, the measured height, the floor from the craft) is that writer's to set.
            if (SettingAccess.SetBy(state, spec.Key).Length > 0 || !SettingAccess.TryGetDouble(state, spec.Key, out double value))
                continue;
            if (SettingMath.IsChanged(BodyDefaults.On(spec, from), value))
                continue;
            double next = BodyDefaults.DefaultOf(spec, body);
            if (next != value)
            {
                SettingAccess.SetDouble(state, spec.Key, next);
                moved++;
            }
        }
        foreach (BoolSpec spec in GuidanceSettingSpecs.AllBools)
        {
            if (!BodyDefaults.Overrides(from, spec.Key) && !BodyDefaults.Overrides(body, spec.Key))
                continue;
            if (SettingAccess.SetBy(state, spec.Key).Length > 0 || !SettingAccess.TryGetBool(state, spec.Key, out bool value))
                continue;
            if (SettingMath.IsChanged(BodyDefaults.On(spec, from), value))
                continue;
            bool next = BodyDefaults.DefaultOf(spec, body);
            if (next != value)
            {
                SettingAccess.SetBool(state, spec.Key, next);
                moved++;
            }
        }
        state.DefaultsBody = body;
        return moved;
    }

    #region File

    /// <summary>Reads body-defaults.toml from the mod folder over the built-in table. A missing file leaves the built-in table alone.</summary>
    internal static void LoadFile()
    {
        string path = Path.Combine(Constants.DocumentsFolderPath, "mods", "AdvancedFlightComputer", BodyDefaults.FileName);
        try
        {
            if (!File.Exists(path))
            {
                BodyDefaults.Reset();
                return;
            }
            int taken = BodyDefaults.Load(File.ReadAllLines(path), out string unread);
            if (unread.Length > 0)
                DefaultCategory.Log.Warning($"[AFC] {BodyDefaults.FileName}: {taken} default(s) taken, skipped {unread}.");
            else
                DefaultCategory.Log.Info($"[AFC] {BodyDefaults.FileName}: {taken} default(s) taken.");
        }
        catch (Exception ex)
        {
            BodyDefaults.Reset();
            DefaultCategory.Log.Warning($"[AFC] Could not read {path}, so the built-in body defaults hold: {ex.Message}");
        }
    }

    #endregion
}

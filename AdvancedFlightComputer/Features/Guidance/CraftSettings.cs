using System.Globalization;
using System.Text.Json;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// The guidance settings of each craft, kept with the game save. After UncompressedSave.Write the values that differ from
/// their defaults are written to a file in the folder of that save, so the game copies, overwrites and deletes them with
/// the save, and after UncompressedSave.Load they are applied to the craft of the same Vehicle.Id. A rename needs nothing,
/// because the state is keyed on the vehicle and the file is written with the id the craft has at the save.
/// Only settings are kept, never the state of a mode that flies (Active, LaunchArmed, the chain to a moon), so a load
/// starts nothing. A value another writer owns (SettingAccess.SetBy) is not written, and neither is the LAN, which the
/// launch page seeds from the launch site. A craft with no record keeps its defaults, and a file of another format, a key
/// this version does not know and a value out of the range of its setting are skipped.
/// </summary>
internal static class CraftSettings
{
    internal const string FileName = "afc-craft-settings.toml";
    internal const int Format = 1;

    private const string CraftTable = "[[craft]]";
    private const string FormatKey = "format";
    private const string IdKey = "id";
    private const string AscentMethodKey = nameof(VehicleAutopilotState.AscentMethod);
    private const string MoonKey = nameof(VehicleAutopilotState.MoonTransferId);
    private const string MoonCaptureKey = nameof(VehicleAutopilotState.MoonTransferCapture);
    private const string EngineGroupsKey = "LandingEngineGroups";
    // The body whose defaults the record is measured against (BodySettings). A record without one, from before per-body defaults, is measured against the defaults of every body.
    private const string BodyKey = nameof(VehicleAutopilotState.DefaultsBody);

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    // SeedLaunchLan sets the LAN over the launch site at the first draw of the launch page. A kept LAN would aim a later launch at the plane that was over the site at the time of the save.
    private static readonly HashSet<string> NotKept = new(StringComparer.Ordinal) { nameof(VehicleAutopilotState.LanDeg) };

    #region Save events

    internal static void OnSaveWritten(string oldSaveId, string newSaveId)
    {
        try
        {
            WriteTo(SaveLoadObserver.CurrentSaveDirectory, Universe.CurrentSystem);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("craft-settings-write:" + ex.GetType().Name,
                $"[AFC] Could not write the guidance settings of the craft into save '{newSaveId}', so they load with their defaults: {ex}");
        }
    }

    internal static void OnSaveLoaded()
    {
        try
        {
            RestoreFrom(SaveLoadObserver.CurrentSaveDirectory, Universe.CurrentSystem);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("craft-settings-read:" + ex.GetType().Name,
                $"[AFC] Could not read the guidance settings of the craft of save '{SaveLoadObserver.CurrentSaveId}', so they keep their defaults: {ex}");
        }
    }

    #endregion

    #region Save folder

    /// <summary>Writes the record of every craft of <paramref name="system"/> that has a changed setting and returns their count. With no such craft the save gets no file.</summary>
    internal static int WriteTo(DirectoryInfo? saveDirectory, CelestialSystem? system)
    {
        if (saveDirectory == null || system == null)
            return 0;

        var crafts = new List<(string Id, List<(string Key, string Value)> Record)>();
        foreach (KeyValuePair<Vehicle, VehicleAutopilotState> entry in VehicleAutopilotState.Snapshot())
        {
            Vehicle vehicle = entry.Key;
            // A state can outlive its craft until the next lifecycle step, so only a craft the system still holds under its id is written.
            if (vehicle is KittenEva || vehicle.IsDisposed || !system.All.TryGet(vehicle.Id, out Astronomical? found) || !ReferenceEquals(found, vehicle))
                continue;
            List<(string Key, string Value)> record = Record(entry.Value);
            if (record.Count > 0)
                crafts.Add((vehicle.Id, record));
        }

        string path = Path.Combine(saveDirectory.FullName, FileName);
        if (crafts.Count == 0)
        {
            if (File.Exists(path))
                File.Delete(path);
            return 0;
        }

        crafts.Sort((a, b) => string.CompareOrdinal(a.Id, b.Id));
        string temp = path + "." + Guid.NewGuid().ToString("N") + ".tmp";
        try
        {
            using (var writer = new StreamWriter(temp))
                Write(writer, crafts);
            File.Move(temp, path, overwrite: true);
        }
        finally
        {
            if (File.Exists(temp))
                File.Delete(temp);
        }
        return crafts.Count;
    }

    /// <summary>Applies the records of the save in <paramref name="saveDirectory"/> to the craft of <paramref name="system"/> and returns the count of craft that got one.</summary>
    internal static int RestoreFrom(DirectoryInfo? saveDirectory, CelestialSystem? system)
    {
        if (saveDirectory == null || system == null)
            return 0;
        string path = Path.Combine(saveDirectory.FullName, FileName);
        if (!File.Exists(path))
            return 0;

        Dictionary<string, Dictionary<string, string>> records = Parse(File.ReadAllLines(path), out string unread);
        int restored = 0, skipped = 0;
        foreach ((string id, Dictionary<string, string> record) in records)
        {
            if (!system.All.TryGet(id, out Astronomical? found) || found is not Vehicle vehicle || vehicle is KittenEva)
                continue;
            skipped += record.Count - Apply(VehicleAutopilotState.For(vehicle), record);
            restored++;
        }

        if (unread.Length > 0 || skipped > 0)
            DefaultCategory.Log.Info($"[AFC] Guidance settings of save '{SaveLoadObserver.CurrentSaveId}': {restored} craft restored, {skipped} value(s) this version does not take were skipped{(unread.Length > 0 ? ", " + unread : "")}.");
        else if (DebugConfig.Guidance)
            DefaultCategory.Log.Debug($"[AFC] Guidance settings of save '{SaveLoadObserver.CurrentSaveId}': {restored} craft restored.");
        return restored;
    }

    #endregion

    #region Records

    /// <summary>The settings of a craft that differ from their defaults, as keys and values in the text of the file. Empty for a craft with every default.</summary>
    internal static List<(string Key, string Value)> Record(VehicleAutopilotState state)
    {
        var record = new List<(string Key, string Value)>();
        foreach (SettingSpec spec in GuidanceSettingSpecs.AllDoubles)
        {
            if (Kept(state, spec.Key) && SettingAccess.TryGetDouble(state, spec.Key, out double value)
                && double.IsFinite(value) && SettingMath.IsChanged(BodyDefaults.On(spec, state.DefaultsBody), value))
                record.Add((spec.Key, value.ToString("R", Inv)));
        }
        foreach (BoolSpec spec in GuidanceSettingSpecs.AllBools)
        {
            if (Kept(state, spec.Key) && SettingAccess.TryGetBool(state, spec.Key, out bool value) && SettingMath.IsChanged(BodyDefaults.On(spec, state.DefaultsBody), value))
                record.Add((spec.Key, value ? "true" : "false"));
        }
        // Only a record that holds a value needs its body, and a body with defaults of its own is all that changes what a value is measured against.
        if (record.Count > 0 && state.DefaultsBody.Length > 0)
            record.Insert(0, (BodyKey, Quote(state.DefaultsBody)));
        if (state.AscentMethod != GuidanceWindow.AscentMethod.CatsUpfg)
            record.Add((AscentMethodKey, Quote(state.AscentMethod.ToString())));
        if (!string.IsNullOrEmpty(state.MoonTransferId))
        {
            record.Add((MoonKey, Quote(state.MoonTransferId)));
            if (state.MoonTransferCapture)
                record.Add((MoonCaptureKey, "true"));
        }
        if (state.EngineGroups.Count != 0)
            record.Add((EngineGroupsKey, Quote(JsonSerializer.Serialize(state.EngineGroups))));
        return record;
    }

    /// <summary>Applies a record to a craft and returns the count of values it took. A key this version does not know, a value that does not parse and a value out of the range of its setting are skipped.</summary>
    internal static int Apply(VehicleAutopilotState state, IReadOnlyDictionary<string, string> record)
    {
        int applied = 0;
        // The craft first takes the defaults of the body the record was measured against, so a value the record leaves out is that body's default and one it holds stands as the player's.
        if (record.TryGetValue(BodyKey, out string? bodyRaw) && TryUnquote(bodyRaw, out string body) && body.Length > 0)
        {
            BodySettings.Follow(state, body);
            applied++;
        }
        foreach (SettingSpec spec in GuidanceSettingSpecs.AllDoubles)
        {
            if (!NotKept.Contains(spec.Key) && record.TryGetValue(spec.Key, out string? raw)
                && double.TryParse(raw, NumberStyles.Float, Inv, out double value) && value >= spec.Min && value <= spec.Max)
            {
                SettingAccess.SetDouble(state, spec.Key, value);
                applied++;
            }
        }
        foreach (BoolSpec spec in GuidanceSettingSpecs.AllBools)
        {
            if (record.TryGetValue(spec.Key, out string? raw) && bool.TryParse(raw, out bool value))
            {
                SettingAccess.SetBool(state, spec.Key, value);
                applied++;
            }
        }
        if (record.TryGetValue(AscentMethodKey, out string? method) && TryUnquote(method, out string name)
            && Enum.TryParse(name, ignoreCase: false, out GuidanceWindow.AscentMethod ascent) && Enum.IsDefined(ascent))
        {
            state.AscentMethod = ascent;
            applied++;
        }
        if (record.TryGetValue(MoonKey, out string? moon) && TryUnquote(moon, out string moonId) && moonId.Length > 0)
        {
            state.MoonTransferId = moonId;
            applied++;
            if (record.TryGetValue(MoonCaptureKey, out string? capture) && bool.TryParse(capture, out bool captureOn))
            {
                state.MoonTransferCapture = captureOn;
                applied++;
            }
        }
        if (record.TryGetValue(EngineGroupsKey, out string? engineRaw) && TryUnquote(engineRaw, out string engineJson))
        {
            try
            {
                var groups = JsonSerializer.Deserialize<Dictionary<string, LandingEngineGroups>>(engineJson);
                if (groups != null && groups.Values.All(settings => settings != null && settings.IsValid()))
                {
                    state.EngineGroups.Clear();
                    foreach ((string scope, LandingEngineGroups settings) in groups)
                        state.EngineGroups.Add(scope, settings);
                    applied++;
                }
            }
            catch (JsonException)
            {
                // An unreadable group record must not prevent the other craft settings from loading.
            }
        }
        return applied;
    }

    private static bool Kept(VehicleAutopilotState state, string key)
        => !NotKept.Contains(key) && SettingAccess.SetBy(state, key).Length == 0;

    #endregion

    #region File text

    internal static void Write(TextWriter writer, IEnumerable<(string Id, List<(string Key, string Value)> Record)> crafts)
    {
        writer.WriteLine("# AdvancedFlightComputer guidance settings of the craft in this save, the values that differ from the defaults.");
        writer.WriteLine("# Written with the save. A key that AFC does not know is skipped when the save loads.");
        writer.WriteLine($"{FormatKey} = {Format.ToString(Inv)}");
        foreach ((string id, List<(string Key, string Value)> record) in crafts)
        {
            writer.WriteLine();
            writer.WriteLine(CraftTable);
            writer.WriteLine($"{IdKey} = {Quote(id)}");
            foreach ((string key, string value) in record)
                writer.WriteLine($"{key} = {value}");
        }
    }

    /// <summary>
    /// The records of the file by craft id, each with its keys and raw values. A file without format <see cref="Format"/>
    /// gives none, a table other than [[craft]] and a craft without an id are skipped, and <paramref name="unread"/> says
    /// what was skipped that way, empty when nothing was.
    /// </summary>
    internal static Dictionary<string, Dictionary<string, string>> Parse(IEnumerable<string> lines, out string unread)
    {
        var records = new Dictionary<string, Dictionary<string, string>>(StringComparer.Ordinal);
        var unreadParts = new List<string>();
        Dictionary<string, string>? current = null;
        bool inOtherTable = false;
        string? format = null;

        foreach (string rawLine in lines)
        {
            string line = StripComment(rawLine).Trim();
            if (line.Length == 0)
                continue;
            if (line == CraftTable)
            {
                Close(current, records, unreadParts);
                current = new Dictionary<string, string>(StringComparer.Ordinal);
                inOtherTable = false;
                continue;
            }
            if (line[0] == '[')
            {
                Close(current, records, unreadParts);
                current = null;
                inOtherTable = true;
                unreadParts.Add("table " + line);
                continue;
            }
            int eq = line.IndexOf('=');
            if (eq < 1 || inOtherTable)
                continue;
            string key = line.Substring(0, eq).Trim();
            string value = line.Substring(eq + 1).Trim();
            if (current != null)
                current[key] = value;
            else if (key == FormatKey)
                format = value;
        }
        Close(current, records, unreadParts);

        if (format != Format.ToString(Inv))
        {
            unread = $"the file has format {format ?? "none"} and this version reads format {Format.ToString(Inv)}, so no craft was restored";
            return new Dictionary<string, Dictionary<string, string>>(StringComparer.Ordinal);
        }
        unread = string.Join(", ", unreadParts);
        return records;
    }

    private static void Close(Dictionary<string, string>? block, Dictionary<string, Dictionary<string, string>> records, List<string> unread)
    {
        if (block == null)
            return;
        if (!block.Remove(IdKey, out string? rawId) || !TryUnquote(rawId, out string id) || id.Length == 0)
        {
            unread.Add("a craft without an id");
            return;
        }
        records[id] = block;
    }

    private static string StripComment(string line)
    {
        bool inString = false;
        for (int i = 0; i < line.Length; i++)
        {
            char c = line[i];
            if (c == '\\' && inString)
                i++;
            else if (c == '"')
                inString = !inString;
            else if (c == '#' && !inString)
                return line.Substring(0, i);
        }
        return line;
    }

    private static string Quote(string value)
        => "\"" + value.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";

    private static bool TryUnquote(string raw, out string value)
    {
        value = string.Empty;
        if (raw.Length < 2 || raw[0] != '"' || raw[^1] != '"')
            return false;
        var sb = new System.Text.StringBuilder(raw.Length);
        for (int i = 1; i < raw.Length - 1; i++)
        {
            char c = raw[i];
            if (c == '\\')
            {
                if (i + 1 >= raw.Length - 1)
                    return false;
                c = raw[++i];
            }
            else if (c == '"')
                return false;
            sb.Append(c);
        }
        value = sb.ToString();
        return true;
    }

    #endregion
}

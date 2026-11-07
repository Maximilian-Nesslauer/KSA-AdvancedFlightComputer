using System.Globalization;
using System.Text;
using Brutal.ImGuiApi;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Core;

/// <summary>
/// UI preferences in ui.toml next to the other AFC files: which page the console shows, where
/// its windows sit and the switches of TUNE > PREFERENCES. One flat key = value per line, # comments,
/// invariant culture. A file that exists but does not parse is never written back in that
/// session, so a hand edit with a typo is not lost.
/// The console windows need their placement here because ConsoleStyle.BeginWindowCore adds
/// ImGuiWindowFlags.NoSavedSettings, so imgui.ini keeps nothing for them.
/// </summary>
internal static class UiConfig
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private static readonly Dictionary<string, string> _values = new(StringComparer.Ordinal);

    // The keys that the console shell and the guidance pages both read.
    internal const string KeyShowDiagnostics = "show_diagnostics";
    internal const string KeyHoverNumpad = "hover_numpad";

    private const double SaveIntervalSeconds = 1.0;

    private static string _path = string.Empty;
    private static bool _loadFailed;
    private static bool _dirty;
    private static DateTime _lastSave = DateTime.MinValue;

    public static void Load()
    {
        _values.Clear();
        _loadFailed = false;
        _dirty = false;
        _path = Path.Combine(KSA.Constants.DocumentsFolderPath, "mods", "AdvancedFlightComputer", "ui.toml");

        if (!File.Exists(_path))
        {
            Save();
            return;
        }

        try
        {
            int lineNo = 0;
            foreach (string rawLine in File.ReadAllLines(_path))
            {
                lineNo++;
                string line = StripComment(rawLine).Trim();
                if (line.Length == 0)
                    continue;
                int eq = line.IndexOf('=');
                if (eq < 1)
                    throw new FormatException($"line {lineNo} has no key = value");
                string key = line.Substring(0, eq).Trim();
                string value = line.Substring(eq + 1).Trim();
                if (key.Length == 0 || value.Length == 0)
                    throw new FormatException($"line {lineNo} has an empty key or value");
                if (value[0] == '"' && !TryUnquote(value, out _))
                    throw new FormatException($"line {lineNo} has an unterminated string");
                _values[key] = value;
            }
        }
        catch (Exception ex)
        {
            _loadFailed = true;
            _values.Clear();
            DefaultCategory.Log.Warning($"[AFC] Could not read {_path}, so the UI uses its defaults and does not write the file this session: {ex.Message}");
        }
    }

    /// <summary>Saves at most once per second while something changed. Called once per frame from the console draw.</summary>
    public static void Tick()
    {
        if (_dirty && (DateTime.UtcNow - _lastSave).TotalSeconds >= SaveIntervalSeconds)
            Save();
    }

    public static void Flush()
    {
        if (_dirty)
            Save();
    }

    public static void Reset()
    {
        _values.Clear();
        _dirty = false;
        _loadFailed = false;
    }

    #region Typed access

    public static bool GetBool(string key, bool fallback)
        => _values.TryGetValue(key, out string? raw) && bool.TryParse(raw, out bool value) ? value : fallback;

    public static float GetFloat(string key, float fallback)
        => _values.TryGetValue(key, out string? raw) && float.TryParse(raw, NumberStyles.Float, Inv, out float value)
           && float.IsFinite(value) ? value : fallback;

    public static string GetString(string key, string fallback)
        => _values.TryGetValue(key, out string? raw) && TryUnquote(raw, out string value) ? value : fallback;

    public static void Set(string key, bool value) => SetRaw(key, value ? "true" : "false");

    public static void Set(string key, float value)
    {
        if (float.IsFinite(value))
            SetRaw(key, value.ToString("R", Inv));
    }

    public static void Set(string key, string value) => SetRaw(key, Quote(value));

    /// <summary>Writes a default for a key the file does not have yet, so a new or older file lists every preference.</summary>
    public static void SetDefault(string key, bool value)
    {
        if (!_values.ContainsKey(key))
            Set(key, value);
    }

    public static void SetDefault(string key, string value)
    {
        if (!_values.ContainsKey(key))
            Set(key, value);
    }

    public static bool Has(string key) => _values.ContainsKey(key);

    public static void Remove(string key)
    {
        if (_values.Remove(key))
            _dirty = true;
    }

    private static void SetRaw(string key, string raw)
    {
        if (_values.TryGetValue(key, out string? old) && old == raw)
            return;
        _values[key] = raw;
        _dirty = true;
    }

    #endregion

    private static void Save()
    {
        _dirty = false;
        _lastSave = DateTime.UtcNow;
        if (_loadFailed || _path.Length == 0)
            return;

        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(_path)!);
            var sb = new StringBuilder();
            sb.AppendLine("# AdvancedFlightComputer UI preferences. Written by the mod; unknown keys are kept.");
            foreach (string key in _values.Keys.OrderBy(k => k, StringComparer.Ordinal))
                sb.Append(key).Append(" = ").AppendLine(_values[key]);
            File.WriteAllText(_path, sb.ToString());
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("ui-config-save:" + ex.GetType().Name, $"[AFC] Failed to save {_path}: {ex.Message}");
        }
    }

    private static string StripComment(string line)
    {
        bool inString = false;
        for (int i = 0; i < line.Length; i++)
        {
            char c = line[i];
            if (c == '\\' && inString)
            {
                i++;
                continue;
            }
            if (c == '"')
                inString = !inString;
            else if (c == '#' && !inString)
                return line.Substring(0, i);
        }
        return line;
    }

    private static string Quote(string value)
    {
        var sb = new StringBuilder(value.Length + 2);
        sb.Append('"');
        foreach (char c in value)
        {
            if (c == '"' || c == '\\')
                sb.Append('\\');
            if (c >= ' ')
                sb.Append(c);
        }
        sb.Append('"');
        return sb.ToString();
    }

    private static bool TryUnquote(string raw, out string value)
    {
        value = string.Empty;
        if (raw.Length < 2 || raw[0] != '"' || raw[^1] != '"')
            return false;
        var sb = new StringBuilder(raw.Length);
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
}

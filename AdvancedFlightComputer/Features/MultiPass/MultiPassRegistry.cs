using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.IO;
using AdvancedFlightComputer.Core;
using Brutal.Logging;

namespace AdvancedFlightComputer.Features.MultiPass;

// Entries belong to one vehicle in one save. Lookups use the current save, and the intent reader map permits additional intent types.
internal static class MultiPassRegistry
{
    private static readonly IReadOnlyDictionary<string, Func<IReadOnlyDictionary<string, string>, IManeuverIntent?>>
        IntentDeserializers = new Dictionary<string, Func<IReadOnlyDictionary<string, string>, IManeuverIntent?>>()
    {
        [ApseIntent.SetApoapsisKind] = block => ApseIntent.FromToml(block, isSetApoapsis: true),
        [ApseIntent.SetPeriapsisKind] = block => ApseIntent.FromToml(block, isSetApoapsis: false),
        [MatchInclinationIntent.MatchInclinationKind] = MatchInclinationIntent.FromToml,
        [SetInclinationIntent.SetInclinationKind] = SetInclinationIntent.FromToml,
        [CircularizeIntent.CircularizeApoapsisKind] = block => CircularizeIntent.FromToml(block, isAtApoapsis: true),
        [CircularizeIntent.CircularizePeriapsisKind] = block => CircularizeIntent.FromToml(block, isAtApoapsis: false),
        [HohmannTransferIntent.HohmannTransferKind] = HohmannTransferIntent.FromToml,
    };

    private static readonly Dictionary<(string SaveId, string VehicleId), MultiPassExecution>
        _byKey = new();

    private static string _modDir = string.Empty;
    private static string _configPath = string.Empty;

    public static void Init()
    {
        string userDocs = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
        _modDir = Path.Combine(userDocs, "My Games", "Kitten Space Agency",
            "mods", "AdvancedFlightComputer");
        _configPath = Path.Combine(_modDir, "multipass.toml");

        // Load at initialization because the default world may never call UncompressedSave.Load. Otherwise its first save could overwrite entries from other saves.
        Load();
    }

    public static int Count => _byKey.Count;

    public static int CountForCurrentSave
    {
        get
        {
            string saveId = SaveLoadObserver.CurrentSaveId;
            if (string.IsNullOrEmpty(saveId)) return 0;

            int count = 0;
            foreach (var key in _byKey.Keys)
                if (key.SaveId == saveId) count++;
            return count;
        }
    }

    // The default world has no save ID until its first write.
    public static bool TryGet(string vehicleId,
        [MaybeNullWhen(false)] out MultiPassExecution exec)
    {
        string saveId = SaveLoadObserver.CurrentSaveId;
        return _byKey.TryGetValue((saveId, vehicleId), out exec);
    }

    public static bool Has(string vehicleId)
    {
        string saveId = SaveLoadObserver.CurrentSaveId;
        return _byKey.ContainsKey((saveId, vehicleId));
    }

    // Keep execution changes in memory until the game is saved.
    public static void Add(MultiPassExecution exec)
    {
        _byKey[(exec.SaveId, exec.VehicleId)] = exec;
        if (MultiPassDebug.Enabled)
            MultiPassDebug.LogExec("MultiPassRegistry.Add", exec);
    }

    public static void Remove(string vehicleId)
    {
        string saveId = SaveLoadObserver.CurrentSaveId;
        bool removed = _byKey.Remove((saveId, vehicleId));
        if (MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPassRegistry.Remove: save='{saveId}' vehicle='{vehicleId}' " +
                $"-> {(removed ? "removed" : "not found")}");
    }

    public static IReadOnlyDictionary<(string, string), MultiPassExecution> Snapshot
        => _byKey;

    // Move the current world to its written save ID and discard entries from an overwritten destination.
    public static void RekeyTo(string oldSaveId, string newSaveId)
    {
        if (string.IsNullOrEmpty(newSaveId) || oldSaveId == newSaveId) return;

        var stale = new List<(string, string)>();
        var moved = new List<MultiPassExecution>();
        foreach (var (key, exec) in _byKey)
        {
            if (key.SaveId == newSaveId)
                stale.Add(key);
            else if (key.SaveId == oldSaveId)
                moved.Add(exec);
        }

        foreach (var key in stale)
        {
            _byKey.Remove(key);
            if (MultiPassDebug.Enabled)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPassRegistry: dropped stale exec for " +
                    $"vehicle={key.Item2} of overwritten save '{newSaveId}'.");
        }

        foreach (MultiPassExecution exec in moved)
        {
            _byKey.Remove((oldSaveId, exec.VehicleId));
            exec.SaveId = newSaveId;
            _byKey[(newSaveId, exec.VehicleId)] = exec;

            if (MultiPassDebug.Enabled)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPassRegistry: rekeyed exec for " +
                    $"vehicle={exec.VehicleId} from save='{oldSaveId}' to '{newSaveId}'.");
        }
    }

    public static void Reset() => _byKey.Clear();

    public static void Load()
    {
        if (string.IsNullOrEmpty(_configPath)) return;

        try
        {
            var loaded = new Dictionary<(string SaveId, string VehicleId), MultiPassExecution>();
            if (!ParseFile(_configPath, loaded))
                return;
            _byKey.Clear();
            foreach (var entry in loaded)
                _byKey.Add(entry.Key, entry.Value);
            if (MultiPassDebug.Enabled)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPassRegistry: loaded {_byKey.Count} entries from {_configPath}");
        }
        catch (FileNotFoundException) { }
        catch (DirectoryNotFoundException) { }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: failed to load {_configPath}: {ex}");
        }
    }

    public static void Save()
    {
        if (string.IsNullOrEmpty(_configPath)) return;

        // Avoid creating a file when no saved execution exists and the feature has never written one.
        bool hasPersistable = false;
        foreach (var exec in _byKey.Values)
        {
            if (!string.IsNullOrEmpty(exec.SaveId)) { hasPersistable = true; break; }
        }
        if (!hasPersistable && !File.Exists(_configPath))
            return;

        // Write a temporary file before replacement so an interrupted write leaves the previous file intact.
        string tempPath = _configPath + "." + Guid.NewGuid().ToString("N") + ".tmp";
        try
        {
            Directory.CreateDirectory(_modDir);
            using (var writer = new StreamWriter(tempPath))
            {
                int written = WriteToml(writer, _byKey.Values);
                if (MultiPassDebug.Enabled)
                    DefaultCategory.Log.Debug($"[AFC] MultiPassRegistry: saved {written} persistent entries");
            }

            File.Move(tempPath, _configPath, overwrite: true);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error(
                $"[AFC] MultiPassRegistry: failed to save {_configPath}: {ex}");

            // Cleanup must not hide the original write failure.
            try
            {
                if (File.Exists(tempPath)) File.Delete(tempPath);
            }
            catch { }
        }
    }

    internal static int WriteToml(TextWriter writer, IEnumerable<MultiPassExecution> executions)
    {
        writer.WriteLine("# AdvancedFlightComputer multi-pass execution state.");
        writer.WriteLine("# Auto-managed; manual edits are overwritten on the next save.");
        writer.WriteLine();

        int written = 0;
        foreach (var exec in executions)
        {
            // Transient entries have no saved scope and must not be written.
            if (string.IsNullOrEmpty(exec.SaveId)) continue;

            writer.WriteLine("[[execution]]");
            writer.WriteLine($"save_id = \"{TomlIo.Escape(exec.SaveId)}\"");
            writer.WriteLine($"vehicle_id = \"{TomlIo.Escape(exec.VehicleId)}\"");
            writer.WriteLine($"kind = \"{TomlIo.Escape(exec.Intent.Kind)}\"");
            writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "mode = \"{0}\"", exec.Mode));
            writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "pass_count_total = {0}", exec.PassCountTotal));
            writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "pass_index = {0}", exec.PassIndex));
            if (exec.CurrentBurnTimeSec.HasValue)
            {
                writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "current_burn_time_sec = {0:R}", exec.CurrentBurnTimeSec.Value));
            }
            if (exec.CurrentBurnDvMagnitudeMs.HasValue)
            {
                writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "current_burn_dv_ms = {0:R}", exec.CurrentBurnDvMagnitudeMs.Value));
            }
            exec.Intent.WriteToToml(writer);
            writer.WriteLine();
            written++;
        }

        return written;
    }

    #region TOML parser

    // Keep the table header line so parse errors can identify the affected entry.
    private sealed class PendingBlock
    {
        public readonly Dictionary<string, string> Fields = new();
        public int HeaderLine;
    }

    private static bool ParseFile(
        string path,
        Dictionary<(string SaveId, string VehicleId), MultiPassExecution> sink)
        => ParseLines(File.ReadAllLines(path), path, sink);

    internal static bool ParseLines(
        string[] lines, string path,
        Dictionary<(string SaveId, string VehicleId), MultiPassExecution> sink)
    {
        PendingBlock? current = null;
        bool success = true;

        for (int li = 0; li < lines.Length; li++)
        {
            string line = lines[li].Trim();
            int lineNumber = li + 1;
            if (line.Length == 0 || line[0] == '#') continue;

            if (line == "[[execution]]")
            {
                success &= FlushBlock(current, sink);
                current = new PendingBlock { HeaderLine = lineNumber };
                continue;
            }

            // An unknown table still ends the previous block. Otherwise its fields could corrupt a known entry.
            if (line.Length > 0 && line[0] == '[')
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] MultiPassRegistry: {Path.GetFileName(path)}:{lineNumber} " +
                    $"unrecognised TOML header '{line}', skipping until next [[execution]].");
                success = false;
                success &= FlushBlock(current, sink);
                current = null;
                continue;
            }

            if (current == null)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] MultiPassRegistry: {Path.GetFileName(path)}:{lineNumber} " +
                    $"key '{line}' outside any [[execution]] block, ignoring.");
                success = false;
                continue;
            }

            int eq = line.IndexOf('=');
            if (eq < 1)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] MultiPassRegistry: {Path.GetFileName(path)}:{lineNumber} " +
                    "expected 'key = value' assignment, ignoring.");
                success = false;
                continue;
            }
            string key = line.Substring(0, eq).Trim();
            string val = line.Substring(eq + 1).Trim();

            // A hash inside a quoted vehicle ID is literal text.
            if (val.Length >= 2 && val[0] == '"')
            {
                int closeIdx = FindClosingQuote(val, openAt: 0);
                if (closeIdx > 0)
                    val = TomlIo.Unescape(val.Substring(1, closeIdx - 1));
                else
                {
                    DefaultCategory.Log.Warning(
                        $"[AFC] MultiPassRegistry: {Path.GetFileName(path)}:{lineNumber} " +
                        $"unterminated string for key '{key}', ignoring.");
                    success = false;
                    continue;
                }
            }
            else
            {
                int commentIdx = val.IndexOf('#');
                if (commentIdx >= 0) val = val.Substring(0, commentIdx).Trim();
            }

            current.Fields[key] = val;
        }

        success &= FlushBlock(current, sink);
        return success;
    }

    private static bool FlushBlock(
        PendingBlock? pending,
        Dictionary<(string SaveId, string VehicleId), MultiPassExecution> sink)
    {
        if (pending == null) return true;
        var block = pending.Fields;

        // Require a save ID so malformed entries cannot enter the default world scope.
        if (!block.TryGetValue("save_id", out string? saveId) || string.IsNullOrEmpty(saveId))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (missing save_id).");
            return false;
        }
        if (!block.TryGetValue("vehicle_id", out string? vehicleId)
            || string.IsNullOrEmpty(vehicleId))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (missing or empty vehicle_id).");
            return false;
        }
        if (!block.TryGetValue("kind", out string? kind)
            || string.IsNullOrEmpty(kind))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (missing or empty kind).");
            return false;
        }
        if (!block.TryGetValue("mode", out string? modeStr) ||
            !Enum.TryParse(modeStr, out SplitMode mode) || !Enum.IsDefined(mode))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (missing or invalid mode '{modeStr ?? "<null>"}').");
            return false;
        }
        if (!block.TryGetValue("pass_count_total", out string? totalStr) ||
            !int.TryParse(totalStr, NumberStyles.Integer, CultureInfo.InvariantCulture, out int total))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (missing or invalid pass_count_total).");
            return false;
        }
        if (!block.TryGetValue("pass_index", out string? idxStr) ||
            !int.TryParse(idxStr, NumberStyles.Integer, CultureInfo.InvariantCulture, out int idx))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (missing or invalid pass_index).");
            return false;
        }

        if (!IntentDeserializers.TryGetValue(kind, out var deserializer))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (unknown intent kind '{kind}').");
            return false;
        }
        IManeuverIntent? intent = deserializer(block);
        if (intent == null)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (intent '{kind}' deserialiser failed).");
            return false;
        }

        double? currentBurnTimeSec = ParseOptionalDouble(block, "current_burn_time_sec");
        double? currentBurnDvMs = ParseOptionalDouble(block, "current_burn_dv_ms");

        sink[(saveId, vehicleId)] = new MultiPassExecution
        {
            SaveId = saveId,
            VehicleId = vehicleId,
            Intent = intent,
            Mode = mode,
            PassCountTotal = total,
            PassIndex = idx,
            CurrentBurn = null,
            CurrentBurnTimeSec = currentBurnTimeSec,
            CurrentBurnDvMagnitudeMs = currentBurnDvMs,
        };
        return true;
    }

    private static double? ParseOptionalDouble(
        Dictionary<string, string> block, string key)
    {
        if (block.TryGetValue(key, out string? s)
            && double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out double v))
            return v;
        return null;
    }

    private static int FindClosingQuote(string s, int openAt)
    {
        for (int i = openAt + 1; i < s.Length; i++)
        {
            char c = s[i];
            if (c == '\\' && i + 1 < s.Length) { i++; continue; }
            if (c == '"') return i;
        }
        return -1;
    }

    #endregion
}

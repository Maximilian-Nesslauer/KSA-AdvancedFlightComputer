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

    // True when the last load could not read all entries from the file.
    private static bool _lastLoadWasDefective;

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

    /// <summary>Gets a read-only view of the live entry map.</summary>
    public static IReadOnlyDictionary<(string, string), MultiPassExecution> Snapshot
        => _byKey;

    // Keep the same execution so renaming does not interrupt the pass sequence.
    public static void RenameVehicle(string oldVehicleId, string newVehicleId)
    {
        if (oldVehicleId == newVehicleId) return;

        string saveId = SaveLoadObserver.CurrentSaveId;
        if (!_byKey.Remove((saveId, oldVehicleId), out MultiPassExecution? exec))
            return;

        // Replace stale state left under a reused vehicle name.
        _byKey.Remove((saveId, newVehicleId));
        exec.VehicleId = newVehicleId;
        _byKey[(saveId, newVehicleId)] = exec;

        if (MultiPassDebug.Enabled)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPassRegistry: rekeyed exec in save='{saveId}' " +
                $"from vehicle='{oldVehicleId}' to '{newVehicleId}'.");
    }

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

    public static void Reset()
    {
        _byKey.Clear();
        _lastLoadWasDefective = false;
    }

    public static void Load()
    {
        if (string.IsNullOrEmpty(_configPath)) return;

        try
        {
            var loaded = new Dictionary<(string SaveId, string VehicleId), MultiPassExecution>();
            // Keep readable blocks, but do not keep entries from the previous save.
            bool clean = ParseFile(_configPath, loaded, out int droppedBlocks);
            _lastLoadWasDefective = !clean;
            _byKey.Clear();
            foreach (var entry in loaded)
                _byKey.Add(entry.Key, entry.Value);

            if (!clean)
                DefaultCategory.Log.Warning(
                    $"[AFC] MultiPassRegistry: could not read all of {_configPath}. " +
                    $"{droppedBlocks} block(s) were dropped, {_byKey.Count} entry(ies) were kept, and " +
                    "details appear in the earlier warnings. The next save writes only the kept entries.");
            else if (MultiPassDebug.Enabled)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPassRegistry: loaded {_byKey.Count} entries from {_configPath}");
        }
        catch (FileNotFoundException) { }
        catch (DirectoryNotFoundException) { }
        catch (Exception ex)
        {
            _lastLoadWasDefective = true;
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
        // Do not erase an unreadable file when there are no entries to preserve.
        if (!hasPersistable && (!File.Exists(_configPath) || _lastLoadWasDefective))
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
            _lastLoadWasDefective = false;
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
        Dictionary<(string SaveId, string VehicleId), MultiPassExecution> sink,
        out int droppedBlocks)
        => ParseLines(File.ReadAllLines(path), path, sink, out droppedBlocks);

    internal static bool ParseLines(
        string[] lines, string path,
        Dictionary<(string SaveId, string VehicleId), MultiPassExecution> sink)
        => ParseLines(lines, path, sink, out _);

    internal static bool ParseLines(
        string[] lines, string path,
        Dictionary<(string SaveId, string VehicleId), MultiPassExecution> sink,
        out int droppedBlocks)
    {
        PendingBlock? current = null;
        bool success = true;
        droppedBlocks = 0;

        for (int li = 0; li < lines.Length; li++)
        {
            string line = lines[li].Trim();
            int lineNumber = li + 1;
            if (line.Length == 0 || line[0] == '#') continue;

            if (line == "[[execution]]")
            {
                success &= FlushBlock(current, sink, ref droppedBlocks);
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
                success &= FlushBlock(current, sink, ref droppedBlocks);
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

        success &= FlushBlock(current, sink, ref droppedBlocks);
        return success;
    }

    private static bool FlushBlock(
        PendingBlock? pending,
        Dictionary<(string SaveId, string VehicleId), MultiPassExecution> sink,
        ref int droppedBlocks)
    {
        if (FlushBlock(pending, sink)) return true;
        droppedBlocks++;
        return false;
    }

    private static bool FlushBlock(
        PendingBlock? pending,
        Dictionary<(string SaveId, string VehicleId), MultiPassExecution> sink)
    {
        if (pending == null) return true;
        var block = pending.Fields;
        int line = pending.HeaderLine;

        // A save ID is required so that a malformed entry cannot enter the default world scope.
        if (!RequireString(block, "save_id", line, out string saveId)
            || !RequireString(block, "vehicle_id", line, out string vehicleId)
            || !RequireString(block, "kind", line, out string kind)
            || !RequireEnum(block, "mode", line, out SplitMode mode)
            || !RequireInt(block, "pass_count_total", line, out int total)
            || !RequireInt(block, "pass_index", line, out int idx))
            return false;

        if (!IntentDeserializers.TryGetValue(kind, out var deserializer))
        {
            DropBlock(line, $"unknown intent kind '{kind}'");
            return false;
        }
        IManeuverIntent? intent = deserializer(block);
        if (intent == null)
        {
            DropBlock(line, $"the deserialiser of intent '{kind}' failed");
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

    private static bool RequireString(
        Dictionary<string, string> block, string key, int headerLine, out string value)
    {
        if (block.TryGetValue(key, out string? raw) && !string.IsNullOrEmpty(raw))
        {
            value = raw;
            return true;
        }
        value = string.Empty;
        DropBlock(headerLine, $"missing or empty {key}");
        return false;
    }

    private static bool RequireInt(
        Dictionary<string, string> block, string key, int headerLine, out int value)
    {
        if (block.TryGetValue(key, out string? raw)
            && int.TryParse(raw, NumberStyles.Integer, CultureInfo.InvariantCulture, out value))
            return true;
        value = 0;
        DropBlock(headerLine, $"missing or invalid {key}");
        return false;
    }

    private static bool RequireEnum<TEnum>(
        Dictionary<string, string> block, string key, int headerLine, out TEnum value)
        where TEnum : struct, Enum
    {
        block.TryGetValue(key, out string? raw);
        if (raw != null && Enum.TryParse(raw, out value) && Enum.IsDefined(value))
            return true;
        value = default;
        DropBlock(headerLine, $"missing or invalid {key} '{raw ?? "<null>"}'");
        return false;
    }

    private static void DropBlock(int headerLine, string reason)
        => DefaultCategory.Log.Warning(
            $"[AFC] MultiPassRegistry: dropping block at line {headerLine} ({reason}).");

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

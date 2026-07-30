using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.IO;
using AdvancedFlightComputer.Core;
using Brutal.Logging;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// One-multi-pass-per-(save, vehicle) registry, persisted to a TOML
/// file in the mod's user folder. Keyed by <c>(SaveId, VehicleId)</c>
/// so a Vehicle.Id collision across saves is harmless: lookups always
/// implicitly scope to <see cref="SaveLoadObserver.CurrentSaveId"/>.
///
/// New intent types register a (kind -> factory) pair in
/// <see cref="IntentDeserializers"/>; nothing else in the registry
/// needs to change.
/// </summary>
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

        // The game never runs UncompressedSave.Load for the default starting
        // universe, so without this eager load a session that starts fresh
        // and then saves would rewrite the file from an empty registry and
        // wipe every other save's persisted entries.
        Load();
    }

    /// <summary>Total entries across all saves (diagnostics only).</summary>
    public static int Count => _byKey.Count;

    /// <summary>Entries that match <see cref="SaveLoadObserver.CurrentSaveId"/>.
    /// User-facing count.</summary>
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

    /// <summary>Looks up by vehicleId in the active save context.
    /// With no save loaded the lookup uses the transient bucket
    /// (SaveId="") so a multi-pass started in an unsaved session
    /// stays visible to the postfix.</summary>
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

    /// <summary>Add / Remove mutate in-memory only. Persistence is
    /// the caller's job; <see cref="SaveLoadObserver"/> drives it
    /// from KSA save events.</summary>
    public static void Add(MultiPassExecution exec)
    {
        _byKey[(exec.SaveId, exec.VehicleId)] = exec;
        if (DebugConfig.MultiPass)
            MultiPassDebug.LogExec($"MultiPassRegistry.Add", exec);
    }

    public static void Remove(string vehicleId)
    {
        string saveId = SaveLoadObserver.CurrentSaveId;
        bool removed = _byKey.Remove((saveId, vehicleId));
        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPassRegistry.Remove: save='{saveId}' vehicle='{vehicleId}' " +
                $"-> {(removed ? "removed" : "not found")}");
    }

    /// <summary>Read-only snapshot for debug logging.</summary>
    public static IReadOnlyDictionary<(string, string), MultiPassExecution> Snapshot
        => _byKey;

    /// <summary>
    /// Moves the session's entries to <paramref name="newSaveId"/> when a
    /// save is written under a different id than the one the world came
    /// from: Save-As, the first save of an unsaved session (old id ""), and
    /// overwriting a different save from the saves list all write with an id
    /// that does not match <see cref="SaveLoadObserver.CurrentSaveId"/>.
    /// Without the move the active execution stays keyed to the old id,
    /// where the CurrentSaveId-scoped lookups (and Remove) can no longer
    /// reach it, so it silently stops advancing and leaks.
    ///
    /// Entries already keyed to <paramref name="newSaveId"/> belong to the
    /// save being overwritten, whose world this write replaces wholesale,
    /// so they are dropped rather than kept beside the moved ones.
    /// </summary>
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
            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPassRegistry: dropped stale exec for " +
                    $"vehicle={key.Item2} of overwritten save '{newSaveId}'.");
        }

        foreach (MultiPassExecution exec in moved)
        {
            _byKey.Remove((oldSaveId, exec.VehicleId));
            exec.SaveId = newSaveId;
            _byKey[(newSaveId, exec.VehicleId)] = exec;

            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPassRegistry: rekeyed exec for " +
                    $"vehicle={exec.VehicleId} from save='{oldSaveId}' to '{newSaveId}'.");
        }
    }

    public static void Reset() => _byKey.Clear();

    /// <summary>
    /// Loads entries from disk. Drops entries with unknown kind,
    /// missing required fields, or empty save_id - only saved-game-
    /// scoped entries persist.
    /// </summary>
    public static void Load()
    {
        if (string.IsNullOrEmpty(_configPath)) return;

        _byKey.Clear();
        if (!File.Exists(_configPath))
            return;

        try
        {
            ParseFile(_configPath, _byKey);
            if (DebugConfig.MultiPass)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPassRegistry: loaded {_byKey.Count} entries from {_configPath}");
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: failed to load {_configPath}: {ex}");
            _byKey.Clear();
        }
    }

    public static void Save()
    {
        if (string.IsNullOrEmpty(_configPath)) return;

        // Skip the write entirely when there is nothing persistable
        // and no file to overwrite. Avoids touching disk on every KSA
        // save for users who never opened the multi-pass UI.
        bool hasPersistable = false;
        foreach (var exec in _byKey.Values)
        {
            if (!string.IsNullOrEmpty(exec.SaveId)) { hasPersistable = true; break; }
        }
        if (!hasPersistable && !File.Exists(_configPath))
            return;

        // Atomic write: serialize to .tmp first, then rename over the
        // real file. A crash mid-Write leaves the previous good
        // registry intact rather than a half-truncated TOML.
        string tempPath = _configPath + ".tmp";
        try
        {
            Directory.CreateDirectory(_modDir);
            using (var writer = new StreamWriter(tempPath))
            {
                writer.WriteLine("# AdvancedFlightComputer multi-pass execution state.");
                writer.WriteLine("# Auto-managed; manual edits are overwritten on the next save.");
                writer.WriteLine();

                int written = 0;
                foreach (var (key, exec) in _byKey)
                {
                    // Skip transient entries: they have no save scope
                    // so persisting them would just clutter the file.
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

                if (DebugConfig.MultiPass)
                    DefaultCategory.Log.Debug(
                        $"[AFC] MultiPassRegistry: saved {written} persistent entries " +
                        $"({_byKey.Count - written} transient skipped)");
            }

            File.Move(tempPath, _configPath, overwrite: true);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error(
                $"[AFC] MultiPassRegistry: failed to save {_configPath}: {ex}");

            // Best-effort cleanup of the half-written temp file.
            try
            {
                if (File.Exists(tempPath)) File.Delete(tempPath);
            }
            catch { }
        }
    }

    #region TOML parser

    /// <summary>
    /// One entry block under construction during parsing, plus the
    /// line number where its <c>[[execution]]</c> header was seen. The
    /// line number is used in skip warnings so users can locate the
    /// offending block in their <c>multipass.toml</c>.
    /// </summary>
    private sealed class PendingBlock
    {
        public readonly Dictionary<string, string> Fields = new();
        public int HeaderLine;
    }

    private static void ParseFile(
        string path,
        Dictionary<(string SaveId, string VehicleId), MultiPassExecution> sink)
    {
        PendingBlock? current = null;
        string[] lines = File.ReadAllLines(path);

        for (int li = 0; li < lines.Length; li++)
        {
            string line = lines[li].Trim();
            int lineNumber = li + 1;
            if (line.Length == 0 || line[0] == '#') continue;

            if (line == "[[execution]]")
            {
                FlushBlock(current, sink);
                current = new PendingBlock { HeaderLine = lineNumber };
                continue;
            }

            // An unknown table header (e.g. typo "[[exection]]") would
            // otherwise be silently swallowed and every subsequent
            // key/value would attach to the previous block. Warn and
            // close out the current block so the typo is visible.
            if (line.Length > 0 && line[0] == '[')
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] MultiPassRegistry: {Path.GetFileName(path)}:{lineNumber} " +
                    $"unrecognised TOML header '{line}', skipping until next [[execution]].");
                FlushBlock(current, sink);
                current = null;
                continue;
            }

            if (current == null)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] MultiPassRegistry: {Path.GetFileName(path)}:{lineNumber} " +
                    $"key '{line}' outside any [[execution]] block, ignoring.");
                continue;
            }

            int eq = line.IndexOf('=');
            if (eq < 1)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] MultiPassRegistry: {Path.GetFileName(path)}:{lineNumber} " +
                    "expected 'key = value' assignment, ignoring.");
                continue;
            }
            string key = line.Substring(0, eq).Trim();
            string val = line.Substring(eq + 1).Trim();

            // Skip comment-stripping inside quoted strings so a
            // vehicle named "Ariane#5" is not truncated to "Ariane".
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

        FlushBlock(current, sink);
    }

    private static void FlushBlock(
        PendingBlock? pending,
        Dictionary<(string SaveId, string VehicleId), MultiPassExecution> sink)
    {
        if (pending == null) return;
        var block = pending.Fields;

        // save_id is required: an entry without one cannot be scoped to a save
        // game, so it would collide with the default starting situation.
        if (!block.TryGetValue("save_id", out string? saveId) || string.IsNullOrEmpty(saveId))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (missing save_id).");
            return;
        }
        if (!block.TryGetValue("vehicle_id", out string? vehicleId)
            || string.IsNullOrEmpty(vehicleId))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (missing or empty vehicle_id).");
            return;
        }
        if (!block.TryGetValue("kind", out string? kind)
            || string.IsNullOrEmpty(kind))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (missing or empty kind).");
            return;
        }
        if (!block.TryGetValue("mode", out string? modeStr) ||
            !Enum.TryParse(modeStr, out SplitMode mode))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (missing or invalid mode '{modeStr ?? "<null>"}').");
            return;
        }
        if (!block.TryGetValue("pass_count_total", out string? totalStr) ||
            !int.TryParse(totalStr, NumberStyles.Integer, CultureInfo.InvariantCulture, out int total))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (missing or invalid pass_count_total).");
            return;
        }
        if (!block.TryGetValue("pass_index", out string? idxStr) ||
            !int.TryParse(idxStr, NumberStyles.Integer, CultureInfo.InvariantCulture, out int idx))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (missing or invalid pass_index).");
            return;
        }

        if (!IntentDeserializers.TryGetValue(kind, out var deserializer))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (unknown intent kind '{kind}').");
            return;
        }
        IManeuverIntent? intent = deserializer(block);
        if (intent == null)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassRegistry: dropping block at line {pending.HeaderLine} (intent '{kind}' deserialiser failed).");
            return;
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
    }

    private static double? ParseOptionalDouble(
        Dictionary<string, string> block, string key)
    {
        if (block.TryGetValue(key, out string? s)
            && double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out double v))
            return v;
        return null;
    }

    /// <summary>
    /// Index of the closing <c>"</c> for a TOML basic string opened at
    /// <paramref name="openAt"/>. Honours \\ and \" escapes. -1 on
    /// no match.
    /// </summary>
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

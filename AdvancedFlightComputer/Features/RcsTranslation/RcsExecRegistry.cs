using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.Logging;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>Persist AFC burn options and active execution metadata per save and vehicle. Stock retains the burn nodes, so removing AFC leaves them intact.</summary>
internal static class RcsExecRegistry
{
    private static readonly Dictionary<(string SaveId, string VehicleId), RcsExecution> _byKey = new();

    private static string _configPath = string.Empty;

    // True when the last load could not read all entries from the file.
    private static bool _lastLoadWasDefective;

    public static void Init()
    {
        string userDocs = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
        string modDir = Path.Combine(userDocs, "My Games", "Kitten Space Agency",
            "mods", "AdvancedFlightComputer");
        _configPath = Path.Combine(modDir, "rcs-exec.toml");

        // UncompressedSave.Load does not run for the default world. Load now so its first save preserves entries from other saves.
        Load();
    }

    public static bool TryGet(string vehicleId, [MaybeNullWhen(false)] out RcsExecution exec)
    {
        string saveId = SaveLoadObserver.CurrentSaveId;
        return _byKey.TryGetValue((saveId, vehicleId), out exec);
    }

    public static RcsExecution GetOrCreate(string vehicleId)
    {
        string saveId = SaveLoadObserver.CurrentSaveId;
        if (_byKey.TryGetValue((saveId, vehicleId), out RcsExecution? exec))
            return exec;
        exec = new RcsExecution { SaveId = saveId, VehicleId = vehicleId };
        _byKey[(saveId, vehicleId)] = exec;
        return exec;
    }

    public static void Remove(string vehicleId)
    {
        string saveId = SaveLoadObserver.CurrentSaveId;
        _byKey.Remove((saveId, vehicleId));
    }

    /// <summary>Keeps execution and cleanup reachable under the vehicle's new ID.</summary>
    public static void RenameVehicle(string oldVehicleId, string newVehicleId)
    {
        if (oldVehicleId == newVehicleId) return;

        string saveId = SaveLoadObserver.CurrentSaveId;
        if (!_byKey.Remove((saveId, oldVehicleId), out RcsExecution? exec))
            return;

        // Replace stale state left under a reused vehicle name.
        _byKey.Remove((saveId, newVehicleId));
        exec.VehicleId = newVehicleId;
        _byKey[(saveId, newVehicleId)] = exec;
    }

    /// <summary>Move the current entries on first save, Save-As, or overwrite of another save so save-scoped lookups can still reach their teardown.</summary>
    public static void RekeyTo(string oldSaveId, string newSaveId)
    {
        RekeyEntries(_byKey, oldSaveId, newSaveId);
    }

    internal static void RekeyEntries(
        Dictionary<(string SaveId, string VehicleId), RcsExecution> entries,
        string oldSaveId, string newSaveId)
    {
        if (string.IsNullOrEmpty(newSaveId) || oldSaveId == newSaveId) return;

        List<(string, string)> stale = new();
        List<RcsExecution> moved = new();
        foreach (var (key, exec) in entries)
        {
            if (key.SaveId == newSaveId)
                stale.Add(key);
            else if (key.SaveId == oldSaveId)
                moved.Add(exec);
        }
        foreach (var key in stale)
            entries.Remove(key);
        foreach (RcsExecution exec in moved)
        {
            entries.Remove((oldSaveId, exec.VehicleId));
            exec.SaveId = newSaveId;
            entries[(newSaveId, exec.VehicleId)] = exec;
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
            var loaded = new Dictionary<(string SaveId, string VehicleId), RcsExecution>();
            // Keep readable blocks, but do not keep entries from the previous save.
            bool clean = ParseFile(_configPath, loaded, out int droppedBlocks);
            _lastLoadWasDefective = !clean;
            _byKey.Clear();
            foreach (var entry in loaded)
                _byKey.Add(entry.Key, entry.Value);

            if (!clean)
                DefaultCategory.Log.Warning(
                    $"[AFC] RcsExecRegistry: could not read all of '{_configPath}'. " +
                    $"{droppedBlocks} block(s) were dropped, {_byKey.Count} vehicle entry(ies) were kept, " +
                    "and details appear in the earlier warnings. The next save writes only the kept entries.");
            else if (DebugConfig.RcsTranslation)
                DefaultCategory.Log.Debug(
                    $"[AFC] RcsExecRegistry: loaded {_byKey.Count} vehicle entries from {_configPath}");
        }
        catch (FileNotFoundException) { }
        catch (DirectoryNotFoundException) { }
        catch (Exception ex)
        {
            _lastLoadWasDefective = true;
            LogHelper.WarnOnce($"rcs-registry-load-{ex.GetType().FullName}",
                $"[AFC] RcsExecRegistry failed to load '{_configPath}' for save '{SaveLoadObserver.CurrentSaveId}': {ex}");
        }
    }

    public static void Save()
    {
        if (string.IsNullOrEmpty(_configPath)) return;

        bool hasPersistable = false;
        foreach (var exec in _byKey.Values)
        {
            if (!string.IsNullOrEmpty(exec.SaveId) && exec.Options.Count > 0)
            {
                hasPersistable = true;
                break;
            }
        }
        // Do not erase an unreadable file when there are no entries to preserve.
        if (!hasPersistable && (!File.Exists(_configPath) || _lastLoadWasDefective))
            return;

        try
        {
            WriteFile(_configPath, _byKey.Values);
            _lastLoadWasDefective = false;
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce($"rcs-registry-save-{ex.GetType().FullName}",
                $"[AFC] RcsExecRegistry failed to save '{_configPath}' for save '{SaveLoadObserver.CurrentSaveId}' with {_byKey.Count} vehicle entries: {ex}");
        }
    }

    internal static void WriteFile(string path, IEnumerable<RcsExecution> executions)
    {
        // A separate temporary file per write keeps another writer's cleanup from deleting this write.
        string tempPath = path + "." + Guid.NewGuid().ToString("N") + ".tmp";
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path))!);
            using (var writer = new StreamWriter(tempPath))
                WriteToml(writer, executions);
            File.Move(tempPath, path, overwrite: true);
        }
        finally
        {
            try
            {
                File.Delete(tempPath);
            }
            catch (IOException) { }
            catch (UnauthorizedAccessException) { }
        }
    }

    #region TOML serialization

    internal static void WriteToml(TextWriter writer, IEnumerable<RcsExecution> execs)
    {
        writer.WriteLine("# AdvancedFlightComputer RCS translation state.");
        writer.WriteLine("# Auto-managed; manual edits are overwritten on the next save.");
        writer.WriteLine();

        foreach (var exec in execs)
        {
            if (string.IsNullOrEmpty(exec.SaveId)) continue;

            foreach (RcsBurnOptions o in exec.Options)
            {
                bool active = exec.ActiveBurnTimeSec.HasValue
                    && o.Matches(exec.ActiveBurnTimeSec.Value, exec.ActiveBurnDvMs ?? o.BurnDvMs);

                writer.WriteLine("[[rcs_burn]]");
                writer.WriteLine($"save_id = \"{TomlIo.Escape(exec.SaveId)}\"");
                writer.WriteLine($"vehicle_id = \"{TomlIo.Escape(exec.VehicleId)}\"");
                writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "burn_time_sec = {0:R}", o.BurnTimeSec));
                writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "burn_dv_ms = {0:R}", o.BurnDvMs));
                writer.WriteLine($"mode = \"{o.Mode}\"");
                writer.WriteLine($"attitude = \"{o.Attitude}\"");
                writer.WriteLine($"allocator = \"{o.Allocator}\"");
                if (active)
                {
                    writer.WriteLine($"active = {(exec.Faulted ? "false" : "true")}");
                    if (exec.Faulted)
                        writer.WriteLine("faulted = true");
                    writer.WriteLine($"resolved_strategy = \"{exec.ResolvedStrategy}\"");
                    writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                        "resolved_axis = {0}", exec.ResolvedAxis));
                    writer.WriteLine($"resolved_allocator = \"{exec.ResolvedAllocator}\"");
                    writer.WriteLine($"align_commanded = {(exec.AlignCommanded ? "true" : "false")}");
                    writer.WriteLine($"forced_rcs_on = {(exec.ForcedRcsOn ? "true" : "false")}");
                    if (exec.ForcedBurnManual)
                        writer.WriteLine("forced_burn_manual = true");
                }
                writer.WriteLine();
            }
        }
    }

    private static bool ParseFile(
        string path,
        Dictionary<(string SaveId, string VehicleId), RcsExecution> into,
        out int droppedBlocks)
        => ParseLines(File.ReadAllLines(path), Path.GetFileName(path), into, out droppedBlocks);

    internal static bool ParseLines(
        string[] lines, string sourceName,
        Dictionary<(string SaveId, string VehicleId), RcsExecution> into)
        => ParseLines(lines, sourceName, into, out _);

    internal static bool ParseLines(
        string[] lines, string sourceName,
        Dictionary<(string SaveId, string VehicleId), RcsExecution> into,
        out int droppedBlocks)
    {
        Dictionary<string, string>? current = null;
        int headerLine = 0;
        bool success = true;
        droppedBlocks = 0;

        for (int li = 0; li < lines.Length; li++)
        {
            string line = lines[li].Trim();
            int lineNumber = li + 1;
            if (line.Length == 0 || line[0] == '#') continue;

            if (line == "[[rcs_burn]]")
            {
                success &= FlushBlock(current, headerLine, sourceName, into, ref droppedBlocks);
                current = new Dictionary<string, string>();
                headerLine = lineNumber;
                continue;
            }
            if (line[0] == '[')
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] RcsExecRegistry: {sourceName}:{lineNumber} " +
                    $"unrecognised TOML header '{line}', skipping until next [[rcs_burn]].");
                success = false;
                success &= FlushBlock(current, headerLine, sourceName, into, ref droppedBlocks);
                current = null;
                continue;
            }
            if (current == null)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] RcsExecRegistry: {sourceName}:{lineNumber} " +
                    $"key '{line}' outside any [[rcs_burn]] block, ignoring.");
                success = false;
                continue;
            }

            int eq = line.IndexOf('=');
            if (eq < 1)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] RcsExecRegistry: {sourceName}:{lineNumber} " +
                    "expected 'key = value' assignment, ignoring.");
                success = false;
                continue;
            }
            string key = line.Substring(0, eq).Trim();
            string val = line.Substring(eq + 1).Trim();
            if (val.Length >= 2 && val[0] == '"')
            {
                // Skip escaped quotes so an embedded quote does not truncate a vehicle ID.
                int close = FindClosingQuote(val, openAt: 0);
                if (close < 0)
                {
                    DefaultCategory.Log.Warning(
                        $"[AFC] RcsExecRegistry: {sourceName}:{lineNumber} " +
                        $"unterminated string for key '{key}', ignoring.");
                    success = false;
                    continue;
                }
                val = TomlIo.Unescape(val.Substring(1, close - 1));
            }
            else
            {
                int commentIdx = val.IndexOf('#');
                if (commentIdx >= 0) val = val.Substring(0, commentIdx).Trim();
            }
            current[key] = val;
        }
        success &= FlushBlock(current, headerLine, sourceName, into, ref droppedBlocks);
        return success;
    }

    private static bool FlushBlock(
        Dictionary<string, string>? block, int headerLine, string sourceName,
        Dictionary<(string SaveId, string VehicleId), RcsExecution> into,
        ref int droppedBlocks)
    {
        if (FlushBlock(block, headerLine, sourceName, into)) return true;
        droppedBlocks++;
        return false;
    }

    private static bool FlushBlock(
        Dictionary<string, string>? block, int headerLine, string sourceName,
        Dictionary<(string SaveId, string VehicleId), RcsExecution> into)
    {
        if (block == null) return true;

        if (!block.TryGetValue("save_id", out string? saveId) || string.IsNullOrEmpty(saveId)
            || !block.TryGetValue("vehicle_id", out string? vehicleId) || string.IsNullOrEmpty(vehicleId)
            || !TryParseDouble(block, "burn_time_sec", out double timeSec)
            || !TryParseDouble(block, "burn_dv_ms", out double dvMs)
            || !double.IsFinite(timeSec) || timeSec < 0.0
            || !double.IsFinite(dvMs) || dvMs < 0.0)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] RcsExecRegistry: dropping block at line {headerLine} of " +
                $"{sourceName} (missing required fields).");
            return false;
        }

        RcsExecutionMode mode = ParseEnumField(
            block, "mode", RcsExecutionMode.Default, sourceName, headerLine);
        RcsAttitudeStrategy attitude = ParseEnumField(
            block, "attitude", RcsAttitudeStrategy.Auto, sourceName, headerLine);
        RcsAllocator allocator = ParseEnumField(
            block, "allocator", RcsAllocator.Groups, sourceName, headerLine);

        if (!into.TryGetValue((saveId, vehicleId), out RcsExecution? exec))
        {
            exec = new RcsExecution { SaveId = saveId, VehicleId = vehicleId };
            into[(saveId, vehicleId)] = exec;
        }
        exec.Options.Add(new RcsBurnOptions
        {
            BurnTimeSec = timeSec,
            BurnDvMs = dvMs,
            Mode = mode,
            Attitude = attitude,
            Allocator = allocator,
        });

        bool faulted = block.TryGetValue("faulted", out string? faultStr)
            && bool.TryParse(faultStr, out bool faultValue) && faultValue;
        bool forcedBurnManual = block.TryGetValue("forced_burn_manual", out string? burnModeStr)
            && bool.TryParse(burnModeStr, out bool burnModeValue) && burnModeValue;
        if (faulted || (block.TryGetValue("active", out string? activeStr)
            && bool.TryParse(activeStr, out bool active) && active))
        {
            if (!TryParseActiveState(block, mode, attitude, allocator, timeSec, dvMs,
                    out RcsAttitudeStrategy resolved, out int axis,
                    out RcsAllocator resolvedAllocator, out bool alignCommanded,
                    out bool forcedRcs))
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] RcsExecRegistry: dropping invalid active state at line " +
                    $"{headerLine} of {sourceName}.");
                return true;
            }

            exec.ActiveBurnTimeSec = timeSec;
            exec.ActiveBurnDvMs = dvMs;
            exec.ResolvedStrategy = resolved;
            exec.ResolvedAxis = axis;
            exec.ResolvedAllocator = resolvedAllocator;
            exec.AlignCommanded = alignCommanded;
            exec.ForcedRcsOn = forcedRcs;
            exec.ForcedBurnManual = forcedBurnManual;
            exec.Faulted = faulted;
        }
        return true;
    }

    private static bool TryParseActiveState(
        Dictionary<string, string> block,
        RcsExecutionMode mode,
        RcsAttitudeStrategy attitude,
        RcsAllocator allocator,
        double timeSec,
        double dvMs,
        out RcsAttitudeStrategy resolved,
        out int axis,
        out RcsAllocator resolvedAllocator,
        out bool alignCommanded,
        out bool forcedRcs)
    {
        resolved = default;
        axis = default;
        resolvedAllocator = default;
        alignCommanded = default;
        forcedRcs = default;

        if (!(timeSec >= 0.0) || !(dvMs > 0.0)
            || !TryParseDefinedEnum(block, "mode", out RcsExecutionMode storedMode)
            || !TryParseDefinedEnum(block, "attitude", out RcsAttitudeStrategy storedAttitude)
            || !TryParseDefinedEnum(block, "allocator", out RcsAllocator storedAllocator)
            || !TryParseDefinedEnum(block, "resolved_strategy", out resolved)
            || !TryParseDefinedEnum(block, "resolved_allocator", out resolvedAllocator)
            || !block.TryGetValue("resolved_axis", out string? axisStr)
            || !int.TryParse(axisStr, NumberStyles.Integer, CultureInfo.InvariantCulture, out axis)
            || !block.TryGetValue("align_commanded", out string? alignStr)
            || !bool.TryParse(alignStr, out alignCommanded)
            || !block.TryGetValue("forced_rcs_on", out string? forcedRcsStr)
            || !bool.TryParse(forcedRcsStr, out forcedRcs))
            return false;

        if (storedMode != mode || storedAttitude != attitude || storedAllocator != allocator
            || mode == RcsExecutionMode.Engine || resolvedAllocator != allocator)
            return false;

        return resolved switch
        {
            RcsAttitudeStrategy.Align => axis is >= 0 and < 6,
            RcsAttitudeStrategy.Hold => axis == -1 && !alignCommanded,
            _ => false,
        };
    }

    private static bool TryParseDefinedEnum<TEnum>(
        Dictionary<string, string> block,
        string key,
        out TEnum value) where TEnum : struct, Enum
    {
        value = default;
        return block.TryGetValue(key, out string? raw)
            && Enum.TryParse(raw, out value)
            && Enum.IsDefined(value);
    }

    private static bool TryParseDouble(Dictionary<string, string> block, string key, out double value)
    {
        value = 0.0;
        return block.TryGetValue(key, out string? s)
            && double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
    }

    /// <summary>A missing optional enum uses its default. Warn when a stored value is no longer defined so a changed user choice is visible.</summary>
    private static TEnum ParseEnumField<TEnum>(
        Dictionary<string, string> block, string key, TEnum fallback,
        string sourceName, int headerLine) where TEnum : struct, Enum
    {
        if (!block.TryGetValue(key, out string? raw))
            return fallback;
        if (Enum.TryParse(raw, out TEnum value) && Enum.IsDefined(value))
            return value;
        DefaultCategory.Log.Warning(
            $"[AFC] RcsExecRegistry: {sourceName} block at line {headerLine} " +
            $"has unrecognised {key} '{raw}', using {fallback}.");
        return fallback;
    }

    private static int FindClosingQuote(string s, int openAt)
    {
        for (int i = openAt + 1; i < s.Length; i++)
        {
            char c = s[i];
            if (c == '\\' && i + 1 < s.Length)
            {
                i++;
                continue;
            }
            if (c == '"')
                return i;
        }
        return -1;
    }

    #endregion
}

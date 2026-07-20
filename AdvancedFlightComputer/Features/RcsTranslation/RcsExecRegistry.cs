using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.Logging;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Per-(save, vehicle) RCS translation state, persisted to rcs-exec.toml in
/// the mod's user folder. The stock save file is never touched: burns
/// themselves live in the KSA save, this file only carries the AFC-side
/// metadata (per-burn mode/attitude and the active execution), so removing
/// the mod leaves every burn intact.
///
/// File format: one flat [[rcs_burn]] block per configured burn. The active
/// execution's block additionally carries active = true plus the resolved
/// strategy. Flat blocks keep the parser identical in shape to the
/// multi-pass registry's.
/// </summary>
internal static class RcsExecRegistry
{
    private static readonly Dictionary<(string SaveId, string VehicleId), RcsExecution> _byKey = new();

    private static string _modDir = string.Empty;
    private static string _configPath = string.Empty;

    public static void Init()
    {
        string userDocs = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
        _modDir = Path.Combine(userDocs, "My Games", "Kitten Space Agency",
            "mods", "AdvancedFlightComputer");
        _configPath = Path.Combine(_modDir, "rcs-exec.toml");

        // The game never runs UncompressedSave.Load for the default starting
        // universe, so without this eager load a session that starts fresh
        // and then saves would rewrite the file from an empty registry and
        // wipe every other save's persisted entries.
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

    public static void RekeyTransientsTo(string newSaveId)
    {
        if (string.IsNullOrEmpty(newSaveId)) return;

        List<RcsExecution> transients = new();
        foreach (var (key, exec) in _byKey)
        {
            if (string.IsNullOrEmpty(key.SaveId))
                transients.Add(exec);
        }
        foreach (RcsExecution exec in transients)
        {
            _byKey.Remove((string.Empty, exec.VehicleId));
            exec.SaveId = newSaveId;
            _byKey[(newSaveId, exec.VehicleId)] = exec;
        }
    }

    public static void Reset() => _byKey.Clear();

    public static void Load()
    {
        if (string.IsNullOrEmpty(_configPath)) return;

        _byKey.Clear();
        if (!File.Exists(_configPath))
            return;

        try
        {
            ParseFile(_configPath);
            if (DebugConfig.RcsTranslation)
                DefaultCategory.Log.Debug(
                    $"[AFC] RcsExecRegistry: loaded {_byKey.Count} vehicle entries from {_configPath}");
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] RcsExecRegistry: failed to load {_configPath}: {ex}");
            _byKey.Clear();
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
        if (!hasPersistable && !File.Exists(_configPath))
            return;

        // Atomic write, same rationale as the multi-pass registry: a crash
        // mid-write must leave the previous good file intact.
        string tempPath = _configPath + ".tmp";
        try
        {
            Directory.CreateDirectory(_modDir);
            using (var writer = new StreamWriter(tempPath))
            {
                writer.WriteLine("# AdvancedFlightComputer RCS translation state.");
                writer.WriteLine("# Auto-managed; manual edits are overwritten on the next save.");
                writer.WriteLine();

                foreach (var exec in _byKey.Values)
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
                            writer.WriteLine("active = true");
                            writer.WriteLine($"resolved_strategy = \"{exec.ResolvedStrategy}\"");
                            writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                                "resolved_axis = {0}", exec.ResolvedAxis));
                            writer.WriteLine($"resolved_allocator = \"{exec.ResolvedAllocator}\"");
                            writer.WriteLine($"align_commanded = {(exec.AlignCommanded ? "true" : "false")}");
                        }
                        writer.WriteLine();
                    }
                }
            }
            File.Move(tempPath, _configPath, overwrite: true);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error($"[AFC] RcsExecRegistry: failed to save {_configPath}: {ex}");
            try
            {
                if (File.Exists(tempPath)) File.Delete(tempPath);
            }
            catch { }
        }
    }

    #region TOML parser

    private static void ParseFile(string path)
    {
        Dictionary<string, string>? current = null;
        int headerLine = 0;
        string[] lines = File.ReadAllLines(path);

        for (int li = 0; li < lines.Length; li++)
        {
            string line = lines[li].Trim();
            int lineNumber = li + 1;
            if (line.Length == 0 || line[0] == '#') continue;

            if (line == "[[rcs_burn]]")
            {
                FlushBlock(current, headerLine, path);
                current = new Dictionary<string, string>();
                headerLine = lineNumber;
                continue;
            }
            if (line[0] == '[')
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] RcsExecRegistry: {Path.GetFileName(path)}:{lineNumber} " +
                    $"unrecognised TOML header '{line}', skipping until next [[rcs_burn]].");
                FlushBlock(current, headerLine, path);
                current = null;
                continue;
            }
            if (current == null)
                continue;

            int eq = line.IndexOf('=');
            if (eq < 1)
                continue;
            string key = line.Substring(0, eq).Trim();
            string val = line.Substring(eq + 1).Trim();
            if (val.Length >= 2 && val[0] == '"')
            {
                // Escape-aware close-quote scan: Save() escapes embedded
                // quotes in ids, so a plain IndexOf would truncate a
                // vehicle name containing '"' and mis-key its options.
                int close = FindClosingQuote(val, openAt: 0);
                if (close < 0)
                    continue;
                val = TomlIo.Unescape(val.Substring(1, close - 1));
            }
            else
            {
                int commentIdx = val.IndexOf('#');
                if (commentIdx >= 0) val = val.Substring(0, commentIdx).Trim();
            }
            current[key] = val;
        }
        FlushBlock(current, headerLine, path);
    }

    private static void FlushBlock(Dictionary<string, string>? block, int headerLine, string path)
    {
        if (block == null) return;

        if (!block.TryGetValue("save_id", out string? saveId) || string.IsNullOrEmpty(saveId)
            || !block.TryGetValue("vehicle_id", out string? vehicleId) || string.IsNullOrEmpty(vehicleId)
            || !TryParseDouble(block, "burn_time_sec", out double timeSec)
            || !TryParseDouble(block, "burn_dv_ms", out double dvMs))
        {
            DefaultCategory.Log.Warning(
                $"[AFC] RcsExecRegistry: dropping block at line {headerLine} of " +
                $"{Path.GetFileName(path)} (missing required fields).");
            return;
        }

        RcsExecutionMode mode = RcsExecutionMode.Default;
        if (block.TryGetValue("mode", out string? modeStr))
            Enum.TryParse(modeStr, out mode);
        RcsAttitudeStrategy attitude = RcsAttitudeStrategy.Auto;
        if (block.TryGetValue("attitude", out string? attStr))
            Enum.TryParse(attStr, out attitude);
        RcsAllocator allocator = RcsAllocator.Groups;
        if (block.TryGetValue("allocator", out string? allocStr))
            Enum.TryParse(allocStr, out allocator);

        if (!_byKey.TryGetValue((saveId, vehicleId), out RcsExecution? exec))
        {
            exec = new RcsExecution { SaveId = saveId, VehicleId = vehicleId };
            _byKey[(saveId, vehicleId)] = exec;
        }
        exec.Options.Add(new RcsBurnOptions
        {
            BurnTimeSec = timeSec,
            BurnDvMs = dvMs,
            Mode = mode,
            Attitude = attitude,
            Allocator = allocator,
        });

        if (block.TryGetValue("active", out string? activeStr)
            && bool.TryParse(activeStr, out bool active) && active)
        {
            exec.ActiveBurnTimeSec = timeSec;
            exec.ActiveBurnDvMs = dvMs;
            if (block.TryGetValue("resolved_strategy", out string? strategyStr)
                && Enum.TryParse(strategyStr, out RcsAttitudeStrategy resolved))
                exec.ResolvedStrategy = resolved;
            if (block.TryGetValue("resolved_axis", out string? axisStr)
                && int.TryParse(axisStr, NumberStyles.Integer, CultureInfo.InvariantCulture, out int axis))
                exec.ResolvedAxis = axis;
            if (block.TryGetValue("resolved_allocator", out string? resAllocStr)
                && Enum.TryParse(resAllocStr, out RcsAllocator resolvedAllocator))
                exec.ResolvedAllocator = resolvedAllocator;
            if (block.TryGetValue("align_commanded", out string? alignStr)
                && bool.TryParse(alignStr, out bool alignCommanded))
                exec.AlignCommanded = alignCommanded;
        }
    }

    private static bool TryParseDouble(Dictionary<string, string> block, string key, out double value)
    {
        value = 0.0;
        return block.TryGetValue(key, out string? s)
            && double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
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

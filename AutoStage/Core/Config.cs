using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using Brutal.Logging;
using KSA;

namespace AutoStage.Core;

/// <summary>
/// Two layers: autostage.toml (per part variant, plus the behaviour switches)
/// and vehicles/{id}.toml (per sequence), the latter winning. Decoupler delays
/// default to 0, so stock timing holds until the user sets one. All files live
/// in the mod's own directory, never in a game save.
/// </summary>
static class Config
{
    private static string _modDir = string.Empty;
    private static string _vehiclesDir = string.Empty;
    private static string _configPath = string.Empty;

    public const bool DropSpentStagesDefault = true;

    /// <summary>
    /// Stage early when the next sequence would shed nothing but burnt-out
    /// engines, instead of waiting for the whole vehicle to run dry.
    /// </summary>
    public static bool DropSpentStages { get; set; } = DropSpentStagesDefault;

    // Part Template ID -> delay in seconds
    public static Dictionary<string, double> EngineDelays { get; } = new();
    public static Dictionary<string, double> DecouplerDelays { get; } = new();

    // Vehicle ID -> (Sequence number -> delay in seconds)
    private static readonly Dictionary<string, Dictionary<int, double>> _vehicleEngineOverrides = new();
    private static readonly Dictionary<string, Dictionary<int, double>> _vehicleDecouplerOverrides = new();

    private static readonly HashSet<string> _dirtyVehicles = new();

    // Blocks SaveGlobalConfig after a parse failure so we don't write the
    // half-loaded state back over the user's file.
    private static bool _globalConfigLoadFailed;

    public static void Init()
    {
        string userProfile = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
        _modDir = Path.Combine(userProfile, "My Games", "Kitten Space Agency", "mods", "AutoStage");
        _vehiclesDir = Path.Combine(_modDir, "vehicles");
        _configPath = Path.Combine(_modDir, "autostage.toml");

        LoadGlobalConfig();
    }

    public static void Reset()
    {
        // Before the dirty set is dropped: the part window only flushes on
        // IsItemDeactivatedAfterEdit, so an override typed right before an
        // unload would otherwise be lost without a trace.
        FlushPendingSaves();
        EngineDelays.Clear();
        DecouplerDelays.Clear();
        DropSpentStages = DropSpentStagesDefault;
        _vehicleEngineOverrides.Clear();
        _vehicleDecouplerOverrides.Clear();
        _dirtyVehicles.Clear();
        _globalConfigLoadFailed = false;
    }

    private static void SetDefaults()
    {
        EngineDelays["CorePropulsionA_Prefab_EngineA1_Dev"] = 2.0;
        EngineDelays["CorePropulsionA_Prefab_EngineA2"] = 2.0;
        EngineDelays["CorePropulsionA_Prefab_EngineA3"] = 3.0;
        EngineDelays["CorePropulsionA_Prefab_EngineA4"] = 1.5;
        EngineDelays["CorePropulsionA_Prefab_EngineA5"] = 3.0;
        EngineDelays["CorePropulsionA_Prefab_EngineA6"] = 3.0;
    }

    #region TOML Parsing

    /// <summary>
    /// Parses a minimal TOML file into sections of key-value pairs.
    /// Supports comments (#), [section] headers, and key = value lines.
    /// The root section (before any header) uses key "".
    /// </summary>
    private static Dictionary<string, Dictionary<string, string>> ParseToml(string path)
    {
        var result = new Dictionary<string, Dictionary<string, string>>();
        string currentSection = "";
        result[currentSection] = new Dictionary<string, string>();

        foreach (string rawLine in File.ReadAllLines(path))
        {
            string line = rawLine.Trim();
            if (line.Length == 0 || line[0] == '#')
                continue;

            if (line[0] == '[')
            {
                int end = line.IndexOf(']');
                if (end > 1)
                {
                    currentSection = line.Substring(1, end - 1).Trim();
                    if (!result.ContainsKey(currentSection))
                        result[currentSection] = new Dictionary<string, string>();
                }
                continue;
            }

            int eq = line.IndexOf('=');
            if (eq < 1) continue;

            string key = line.Substring(0, eq).Trim();
            string value = line.Substring(eq + 1).Trim();

            int commentIdx = value.IndexOf('#');
            if (commentIdx >= 0)
                value = value.Substring(0, commentIdx).Trim();

            if (!result.ContainsKey(currentSection))
                result[currentSection] = new Dictionary<string, string>();
            result[currentSection][key] = value;
        }

        return result;
    }

    private static bool TryParseDelay(string value, out double delay)
    {
        if (double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out delay))
        {
            delay = Math.Max(0.0, delay);
            return true;
        }
        delay = 0.0;
        return false;
    }

    private static bool ReadFlag(Dictionary<string, Dictionary<string, string>> sections,
        string section, string key, bool fallback)
    {
        if (!sections.TryGetValue(section, out var entries)
            || !entries.TryGetValue(key, out string? raw))
            return fallback;

        if (bool.TryParse(raw, out bool parsed))
            return parsed;

        // Worth saying out loud: bool.TryParse rejects 0/no/off, and the next
        // save rewrites the line, so a user who meant to switch something off
        // would otherwise get the default back with no trace of why.
        DefaultCategory.Log.Warning(
            $"[AutoStage] Config: [{section}] {key} = '{raw}' is not true or false, "
            + $"using {(fallback ? "true" : "false")}.");
        return fallback;
    }

    #endregion

    #region Global Config

    public static void LoadGlobalConfig()
    {
        EngineDelays.Clear();
        DecouplerDelays.Clear();
        DropSpentStages = DropSpentStagesDefault;
        _globalConfigLoadFailed = false;

        if (!File.Exists(_configPath))
        {
            SetDefaults();
            SaveGlobalConfig();
            return;
        }

        try
        {
            var sections = ParseToml(_configPath);

            if (sections.TryGetValue("engine_delays", out var engines))
                LoadDelaySection(engines, EngineDelays);

            if (sections.TryGetValue("decoupler_delays", out var decouplers))
                LoadDelaySection(decouplers, DecouplerDelays);

            DropSpentStages = ReadFlag(sections, "staging", "drop_spent_stages",
                DropSpentStagesDefault);

            // Either flag: the delays belong to IgnitionDelay, drop_spent_stages
            // to AutoStage, and one line reports both.
            if (DebugConfig.IgnitionDelay || DebugConfig.AutoStage)
                DefaultCategory.Log.Debug(
                    $"[AutoStage] Config loaded: {EngineDelays.Count} engine delays, " +
                    $"{DecouplerDelays.Count} decoupler delays, " +
                    $"drop_spent_stages={DropSpentStages}");
        }
        catch (Exception ex)
        {
            EngineDelays.Clear();
            DecouplerDelays.Clear();
            DropSpentStages = DropSpentStagesDefault;
            _globalConfigLoadFailed = true;
            DefaultCategory.Log.Error(
                $"[AutoStage] Failed to load config ({_configPath}): {ex.Message}");
        }
    }

    private static void LoadDelaySection(Dictionary<string, string> raw,
        Dictionary<string, double> target)
    {
        foreach (var kvp in raw)
        {
            if (TryParseDelay(kvp.Value, out double d))
                target[kvp.Key] = d;
        }
    }

    public static void SaveGlobalConfig()
    {
        if (_globalConfigLoadFailed)
        {
            DefaultCategory.Log.Warning(
                "[AutoStage] Skipping save, last load failed and the in-memory "
                + "config is empty. Fix the file manually before saving from the UI.");
            return;
        }

        try
        {
            Directory.CreateDirectory(_modDir);
            using var writer = new StreamWriter(_configPath);
            writer.WriteLine("# AutoStage configuration.");
            writer.WriteLine();
            writer.WriteLine("[staging]");
            writer.WriteLine("# Stage as soon as the next sequence would shed nothing but");
            writer.WriteLine("# burnt-out engines, instead of waiting for the whole vehicle to");
            writer.WriteLine("# run dry. This is what drops spent boosters off a core stage.");
            writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "drop_spent_stages = {0}", DropSpentStages ? "true" : "false"));
            writer.WriteLine();
            writer.WriteLine("# Per-part-variant delays keyed by Part Template ID.");
            writer.WriteLine("# Values are seconds to wait after staging before the part activates.");
            writer.WriteLine();
            writer.WriteLine("[engine_delays]");
            WriteDelaySection(writer, EngineDelays);
            writer.WriteLine();
            writer.WriteLine("[decoupler_delays]");
            WriteDelaySection(writer, DecouplerDelays);

            if (DebugConfig.IgnitionDelay)
                DefaultCategory.Log.Debug("[AutoStage] Config saved.");
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error($"[AutoStage] Failed to save config: {ex.Message}");
        }
    }

    private static void WriteDelaySection(StreamWriter writer, Dictionary<string, double> source)
    {
        var keys = new List<string>(source.Keys);
        keys.Sort(StringComparer.Ordinal);
        foreach (string key in keys)
        {
            writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "{0} = {1:F1}", key, source[key]));
        }
    }

    #endregion

    #region Delay Lookup

    public static double GetEngineDelay(string partTemplateId)
        => EngineDelays.TryGetValue(partTemplateId, out double d) ? d : 0.0;

    public static double GetDecouplerDelay(string partTemplateId)
        => DecouplerDelays.TryGetValue(partTemplateId, out double d) ? d : 0.0;

    /// <summary>
    /// Effective engine ignition delay for a sequence. Priority:
    /// 1. Per-sequence vehicle override
    /// 2. Max engine-variant delay across the sequence's engine parts
    /// </summary>
    public static double GetSequenceEngineDelay(Vehicle vehicle, int sequenceNumber)
    {
        if (_vehicleEngineOverrides.TryGetValue(vehicle.Id, out var overrides)
            && overrides.TryGetValue(sequenceNumber, out double overrideDelay))
            return overrideDelay;
        return ComputeSequenceEngineDelay(vehicle, sequenceNumber);
    }

    /// <summary>
    /// Effective decoupler delay for a sequence. Priority:
    /// 1. Per-sequence vehicle override
    /// 2. Max decoupler-variant delay across the sequence's decoupler parts
    /// </summary>
    public static double GetSequenceDecouplerDelay(Vehicle vehicle, int sequenceNumber)
    {
        if (_vehicleDecouplerOverrides.TryGetValue(vehicle.Id, out var overrides)
            && overrides.TryGetValue(sequenceNumber, out double overrideDelay))
            return overrideDelay;
        return ComputeSequenceDecouplerDelay(vehicle, sequenceNumber);
    }

    public static double ComputeSequenceEngineDelay(Vehicle vehicle, int sequenceNumber)
        => ComputeSequenceMaxDelay(vehicle, sequenceNumber, DelayKind.Engine);

    public static double ComputeSequenceDecouplerDelay(Vehicle vehicle, int sequenceNumber)
        => ComputeSequenceMaxDelay(vehicle, sequenceNumber, DelayKind.Decoupler);

    /// <summary>
    /// Longest delay among the modules of one kind this row fires. Per module,
    /// not per part: else a decoupler-only row inherits the part's engine delay.
    /// </summary>
    private static double ComputeSequenceMaxDelay(Vehicle vehicle, int sequenceNumber, DelayKind kind)
    {
        double maxDelay = 0.0;
        foreach (Sequence seq in vehicle.Parts.SequenceList.Sequences)
        {
            if (seq.Number != sequenceNumber) continue;
            ReadOnlySpan<Part> parts = seq.Parts;
            for (int i = 0; i < parts.Length; i++)
            {
                foreach (ISequenced module in parts[i].InSequence(sequenceNumber))
                {
                    if (!SequencedModules.Matches(module, kind)) continue;
                    string key = SequencedModules.DelayKey(module);
                    double delay = kind == DelayKind.Engine
                        ? GetEngineDelay(key)
                        : GetDecouplerDelay(key);
                    maxDelay = Math.Max(maxDelay, delay);
                }
            }
            break;
        }
        return maxDelay;
    }

    public static bool HasSequenceEngineOverride(Vehicle vehicle, int sequenceNumber)
        => _vehicleEngineOverrides.TryGetValue(vehicle.Id, out var o)
           && o.ContainsKey(sequenceNumber);

    public static bool HasSequenceDecouplerOverride(Vehicle vehicle, int sequenceNumber)
        => _vehicleDecouplerOverrides.TryGetValue(vehicle.Id, out var o)
           && o.ContainsKey(sequenceNumber);

    public static void SetSequenceEngineOverride(Vehicle vehicle, int sequenceNumber, double delay)
        => SetSequenceOverride(_vehicleEngineOverrides, vehicle.Id, sequenceNumber, delay);

    public static void SetSequenceDecouplerOverride(Vehicle vehicle, int sequenceNumber, double delay)
        => SetSequenceOverride(_vehicleDecouplerOverrides, vehicle.Id, sequenceNumber, delay);

    public static void ClearSequenceEngineOverride(Vehicle vehicle, int sequenceNumber)
        => ClearSequenceOverride(_vehicleEngineOverrides, vehicle.Id, sequenceNumber);

    public static void ClearSequenceDecouplerOverride(Vehicle vehicle, int sequenceNumber)
        => ClearSequenceOverride(_vehicleDecouplerOverrides, vehicle.Id, sequenceNumber);

    private static void SetSequenceOverride(
        Dictionary<string, Dictionary<int, double>> store,
        string vehicleId, int sequenceNumber, double delay)
    {
        if (!store.TryGetValue(vehicleId, out var overrides))
        {
            overrides = new Dictionary<int, double>();
            store[vehicleId] = overrides;
        }
        overrides[sequenceNumber] = Math.Max(0.0, delay);
        _dirtyVehicles.Add(vehicleId);
    }

    private static void ClearSequenceOverride(
        Dictionary<string, Dictionary<int, double>> store,
        string vehicleId, int sequenceNumber)
    {
        if (store.TryGetValue(vehicleId, out var overrides) && overrides.Remove(sequenceNumber))
            _dirtyVehicles.Add(vehicleId);
    }

    public static void FlushPendingSaves()
    {
        foreach (string vehicleId in _dirtyVehicles)
            SaveVehicleOverrides(vehicleId);
        _dirtyVehicles.Clear();
    }

    #endregion

    #region Per-Vehicle Persistence

    public static void LoadVehicleOverrides(string vehicleId)
    {
        // Seed both stores so we don't keep re-reading a missing/malformed
        // file on every per-frame call from the part window.
        bool alreadyLoaded = _vehicleEngineOverrides.ContainsKey(vehicleId)
                             && _vehicleDecouplerOverrides.ContainsKey(vehicleId);
        if (alreadyLoaded) return;

        var engine = new Dictionary<int, double>();
        var decoupler = new Dictionary<int, double>();
        _vehicleEngineOverrides[vehicleId] = engine;
        _vehicleDecouplerOverrides[vehicleId] = decoupler;

        string path = GetVehiclePath(vehicleId);
        if (!File.Exists(path)) return;

        try
        {
            var sections = ParseToml(path);
            if (sections.TryGetValue("sequence_delays", out var engineSection))
                LoadSequenceDelays(engineSection, engine);
            if (sections.TryGetValue("decoupler_delays", out var decouplerSection))
                LoadSequenceDelays(decouplerSection, decoupler);

            if ((engine.Count > 0 || decoupler.Count > 0) && DebugConfig.IgnitionDelay)
                DefaultCategory.Log.Debug(
                    $"[AutoStage] Loaded {engine.Count} engine + {decoupler.Count} decoupler " +
                    $"sequence overrides for {vehicleId}");
        }
        catch (Exception ex)
        {
            // Seeds stay: removing them clears alreadyLoaded, and the part window
            // calls this per frame, so an unreadable file would repeat forever.
            engine.Clear();
            decoupler.Clear();
            DefaultCategory.Log.Error(
                $"[AutoStage] Failed to load vehicle overrides for {vehicleId}: {ex.Message}");
        }
    }

    public static void RemoveVehicle(string vehicleId)
    {
        if (_dirtyVehicles.Contains(vehicleId))
        {
            SaveVehicleOverrides(vehicleId);
            _dirtyVehicles.Remove(vehicleId);
        }
        _vehicleEngineOverrides.Remove(vehicleId);
        _vehicleDecouplerOverrides.Remove(vehicleId);
    }

    private static void LoadSequenceDelays(Dictionary<string, string> raw, Dictionary<int, double> target)
    {
        foreach (var kvp in raw)
        {
            if (int.TryParse(kvp.Key, out int seqNum) && TryParseDelay(kvp.Value, out double d))
                target[seqNum] = d;
        }
    }

    private static void SaveVehicleOverrides(string vehicleId)
    {
        string path = GetVehiclePath(vehicleId);

        try
        {
            _vehicleEngineOverrides.TryGetValue(vehicleId, out var engine);
            _vehicleDecouplerOverrides.TryGetValue(vehicleId, out var decoupler);

            bool hasEngine = engine != null && engine.Count > 0;
            bool hasDecoupler = decoupler != null && decoupler.Count > 0;
            if (!hasEngine && !hasDecoupler)
            {
                if (File.Exists(path))
                    File.Delete(path);
                return;
            }

            Directory.CreateDirectory(_vehiclesDir);
            using var writer = new StreamWriter(path);
            writer.WriteLine("# Per-sequence delay overrides for this vehicle.");
            if (hasEngine)
            {
                writer.WriteLine();
                writer.WriteLine("[sequence_delays]");
                WriteSequenceDelays(writer, engine!);
            }
            if (hasDecoupler)
            {
                writer.WriteLine();
                writer.WriteLine("[decoupler_delays]");
                WriteSequenceDelays(writer, decoupler!);
            }
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error(
                $"[AutoStage] Failed to save vehicle overrides for {vehicleId}: {ex.Message}");
        }
    }

    private static void WriteSequenceDelays(StreamWriter writer, Dictionary<int, double> source)
    {
        var keys = new List<int>(source.Keys);
        keys.Sort();
        foreach (int key in keys)
        {
            writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "{0} = {1:F1}", key, source[key]));
        }
    }

    private static string GetVehiclePath(string vehicleId)
    {
        string safeId = vehicleId;
        foreach (char c in Path.GetInvalidFileNameChars())
            safeId = safeId.Replace(c, '_');
        return Path.Combine(_vehiclesDir, safeId + ".toml");
    }

    #endregion
}

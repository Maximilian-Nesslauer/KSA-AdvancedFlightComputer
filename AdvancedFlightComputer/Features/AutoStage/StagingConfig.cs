using System.Globalization;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.AutoStage;

// Two layers: autostage.toml holds the per-part-variant delays and the behaviour switches,
// autostage-vehicles/{id}.toml the per-sequence overrides, which win. Nothing lives in a save.
internal static class StagingConfig
{
    public const bool DropSpentStagesDefault = true;

    private static string _modDir = string.Empty;
    private static string _vehiclesDir = string.Empty;
    private static string _configPath = string.Empty;

    public static bool DropSpentStages { get; set; } = DropSpentStagesDefault;

    // Part template id to seconds.
    public static Dictionary<string, double> EngineDelays { get; } = new();
    public static Dictionary<string, double> DecouplerDelays { get; } = new();

    // Vehicle id to sequence number to seconds.
    private static readonly Dictionary<string, Dictionary<int, double>> _vehicleEngineOverrides = new();
    private static readonly Dictionary<string, Dictionary<int, double>> _vehicleDecouplerOverrides = new();
    private static readonly HashSet<string> _dirtyVehicles = new();

    // A file that failed to load is never written back, so the half-loaded state cannot overwrite it.
    private static bool _globalConfigLoadFailed;
    private static readonly HashSet<string> _failedVehicleLoads = new();

    public static void Init()
    {
        string modsDir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            "My Games", "Kitten Space Agency", "mods");
        _modDir = Path.Combine(modsDir, "AdvancedFlightComputer");
        _vehiclesDir = Path.Combine(_modDir, "autostage-vehicles");
        _configPath = Path.Combine(_modDir, "autostage.toml");
        ImportStandaloneConfig(Path.Combine(modsDir, "AutoStage"));
        LoadGlobalConfig();
    }

    public static void Reset()
    {
        // The part window only flushes on IsItemDeactivatedAfterEdit, so an override typed right before unload is still dirty.
        FlushPendingSaves();
        EngineDelays.Clear();
        DecouplerDelays.Clear();
        DropSpentStages = DropSpentStagesDefault;
        _vehicleEngineOverrides.Clear();
        _vehicleDecouplerOverrides.Clear();
        _dirtyVehicles.Clear();
        _globalConfigLoadFailed = false;
        _failedVehicleLoads.Clear();
    }

    // The files the standalone AutoStage mod wrote carry over once, on the first load without an AFC copy.
    private static void ImportStandaloneConfig(string legacyDir)
    {
        string legacyConfig = Path.Combine(legacyDir, "autostage.toml");
        if (File.Exists(_configPath) || !File.Exists(legacyConfig))
            return;
        try
        {
            Directory.CreateDirectory(_modDir);
            File.Copy(legacyConfig, _configPath);
            string legacyVehicles = Path.Combine(legacyDir, "vehicles");
            if (Directory.Exists(legacyVehicles))
            {
                Directory.CreateDirectory(_vehiclesDir);
                foreach (string file in Directory.GetFiles(legacyVehicles, "*.toml"))
                {
                    string target = Path.Combine(_vehiclesDir, Path.GetFileName(file));
                    if (!File.Exists(target))
                        File.Copy(file, target);
                }
            }
            DefaultCategory.Log.Info($"[AFC] Imported the AutoStage settings from {legacyDir}.");
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] Could not import the AutoStage settings from {legacyDir}: {ex.Message}");
        }
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

    #region TOML parsing

    // Comments, [section] headers and key = value lines. The root section has the key "".
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
            if (eq < 1)
                continue;

            string key = line.Substring(0, eq).Trim();
            string value = line.Substring(eq + 1).Trim();
            int comment = value.IndexOf('#');
            if (comment >= 0)
                value = value.Substring(0, comment).Trim();
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
        if (!sections.TryGetValue(section, out var entries) || !entries.TryGetValue(key, out string? raw))
            return fallback;
        if (bool.TryParse(raw, out bool parsed))
            return parsed;

        // bool.TryParse rejects 0/no/off, and the next save rewrites the line, so say why the default came back.
        DefaultCategory.Log.Warning(
            $"[AFC] autostage.toml: [{section}] {key} = '{raw}' is not true or false, using {(fallback ? "true" : "false")}.");
        return fallback;
    }

    #endregion

    #region Global config

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
            DropSpentStages = ReadFlag(sections, "staging", "drop_spent_stages", DropSpentStagesDefault);

            if (DebugConfig.AutoStage)
                DefaultCategory.Log.Debug(
                    $"[AFC] AutoStage config loaded: {EngineDelays.Count} engine delays, " +
                    $"{DecouplerDelays.Count} decoupler delays, drop_spent_stages={DropSpentStages}");
        }
        catch (Exception ex)
        {
            EngineDelays.Clear();
            DecouplerDelays.Clear();
            DropSpentStages = DropSpentStagesDefault;
            _globalConfigLoadFailed = true;
            DefaultCategory.Log.Error($"[AFC] Failed to load {_configPath}: {ex.Message}");
        }
    }

    private static void LoadDelaySection(Dictionary<string, string> raw, Dictionary<string, double> target)
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
                "[AFC] Skipping the autostage.toml save, the last load failed and the in-memory config is empty. Fix the file before saving from the UI.");
            return;
        }

        try
        {
            Directory.CreateDirectory(_modDir);
            using var writer = new StreamWriter(_configPath);
            writer.WriteLine("# AdvancedFlightComputer automatic staging.");
            writer.WriteLine();
            writer.WriteLine("[staging]");
            writer.WriteLine("# Stage as soon as the next sequence would shed nothing but burnt-out");
            writer.WriteLine("# engines, instead of waiting for the whole vehicle to run dry.");
            writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "drop_spent_stages = {0}", DropSpentStages ? "true" : "false"));
            writer.WriteLine();
            writer.WriteLine("# Per-part-variant delays keyed by part template id, in seconds after the staging trigger.");
            writer.WriteLine();
            writer.WriteLine("[engine_delays]");
            WriteDelaySection(writer, EngineDelays);
            writer.WriteLine();
            writer.WriteLine("[decoupler_delays]");
            WriteDelaySection(writer, DecouplerDelays);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error($"[AFC] Failed to save {_configPath}: {ex.Message}");
        }
    }

    private static void WriteDelaySection(StreamWriter writer, Dictionary<string, double> source)
    {
        var keys = new List<string>(source.Keys);
        keys.Sort(StringComparer.Ordinal);
        foreach (string key in keys)
            writer.WriteLine(string.Format(CultureInfo.InvariantCulture, "{0} = {1:F1}", key, source[key]));
    }

    #endregion

    #region Delay lookup

    public static double GetEngineDelay(string partTemplateId)
        => EngineDelays.TryGetValue(partTemplateId, out double d) ? d : 0.0;

    public static double GetDecouplerDelay(string partTemplateId)
        => DecouplerDelays.TryGetValue(partTemplateId, out double d) ? d : 0.0;

    // The per-sequence override wins over the longest variant delay in the row.
    public static double GetSequenceEngineDelay(Vehicle vehicle, int sequenceNumber)
        => TryGetOverride(_vehicleEngineOverrides, vehicle, sequenceNumber, out double delay)
            ? delay : ComputeSequenceEngineDelay(vehicle, sequenceNumber);

    public static double GetSequenceDecouplerDelay(Vehicle vehicle, int sequenceNumber)
        => TryGetOverride(_vehicleDecouplerOverrides, vehicle, sequenceNumber, out double delay)
            ? delay : ComputeSequenceDecouplerDelay(vehicle, sequenceNumber);

    public static double ComputeSequenceEngineDelay(Vehicle vehicle, int sequenceNumber)
        => ComputeSequenceMaxDelay(vehicle, sequenceNumber, DelayKind.Engine);

    public static double ComputeSequenceDecouplerDelay(Vehicle vehicle, int sequenceNumber)
        => ComputeSequenceMaxDelay(vehicle, sequenceNumber, DelayKind.Decoupler);

    // Per module, not per part, or a decoupler-only row would inherit the part's engine delay.
    private static double ComputeSequenceMaxDelay(Vehicle vehicle, int sequenceNumber, DelayKind kind)
    {
        double maxDelay = 0.0;
        foreach (Sequence seq in vehicle.Parts.SequenceList.Sequences)
        {
            if (seq.Number != sequenceNumber)
                continue;
            ReadOnlySpan<Part> parts = seq.Parts;
            for (int i = 0; i < parts.Length; i++)
            {
                foreach (ISequenced module in parts[i].InSequence(sequenceNumber))
                {
                    if (!SequencedModules.Matches(module, kind))
                        continue;
                    string key = SequencedModules.DelayKey(module);
                    double delay = kind == DelayKind.Engine ? GetEngineDelay(key) : GetDecouplerDelay(key);
                    maxDelay = Math.Max(maxDelay, delay);
                }
            }
            break;
        }
        return maxDelay;
    }

    private static bool TryGetOverride(Dictionary<string, Dictionary<int, double>> store,
        Vehicle vehicle, int sequenceNumber, out double delay)
    {
        delay = 0.0;
        return store.TryGetValue(vehicle.Id, out var overrides) && overrides.TryGetValue(sequenceNumber, out delay);
    }

    public static bool HasSequenceEngineOverride(Vehicle vehicle, int sequenceNumber)
        => TryGetOverride(_vehicleEngineOverrides, vehicle, sequenceNumber, out _);

    public static bool HasSequenceDecouplerOverride(Vehicle vehicle, int sequenceNumber)
        => TryGetOverride(_vehicleDecouplerOverrides, vehicle, sequenceNumber, out _);

    public static void SetSequenceEngineOverride(Vehicle vehicle, int sequenceNumber, double delay)
        => SetSequenceOverride(_vehicleEngineOverrides, vehicle.Id, sequenceNumber, delay);

    public static void SetSequenceDecouplerOverride(Vehicle vehicle, int sequenceNumber, double delay)
        => SetSequenceOverride(_vehicleDecouplerOverrides, vehicle.Id, sequenceNumber, delay);

    public static void ClearSequenceEngineOverride(Vehicle vehicle, int sequenceNumber)
        => ClearSequenceOverride(_vehicleEngineOverrides, vehicle.Id, sequenceNumber);

    public static void ClearSequenceDecouplerOverride(Vehicle vehicle, int sequenceNumber)
        => ClearSequenceOverride(_vehicleDecouplerOverrides, vehicle.Id, sequenceNumber);

    private static void SetSequenceOverride(Dictionary<string, Dictionary<int, double>> store,
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

    private static void ClearSequenceOverride(Dictionary<string, Dictionary<int, double>> store,
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

    #region Per-vehicle persistence

    public static void LoadVehicleOverrides(string vehicleId)
    {
        // Both stores are seeded first, so a missing or malformed file is not re-read on every part-window frame.
        if (_vehicleEngineOverrides.ContainsKey(vehicleId) && _vehicleDecouplerOverrides.ContainsKey(vehicleId))
            return;

        var engine = new Dictionary<int, double>();
        var decoupler = new Dictionary<int, double>();
        _vehicleEngineOverrides[vehicleId] = engine;
        _vehicleDecouplerOverrides[vehicleId] = decoupler;

        string path = GetVehiclePath(vehicleId);
        if (!File.Exists(path))
            return;

        try
        {
            var sections = ParseToml(path);
            if (sections.TryGetValue("sequence_delays", out var engineSection))
                LoadSequenceDelays(engineSection, engine);
            if (sections.TryGetValue("decoupler_delays", out var decouplerSection))
                LoadSequenceDelays(decouplerSection, decoupler);
        }
        catch (Exception ex)
        {
            engine.Clear();
            decoupler.Clear();
            _failedVehicleLoads.Add(vehicleId);
            DefaultCategory.Log.Error($"[AFC] Failed to load the staging overrides for {vehicleId}, edits are not saved: {ex}");
        }
    }

    public static void RemoveVehicle(string vehicleId)
    {
        if (_dirtyVehicles.Remove(vehicleId))
            SaveVehicleOverrides(vehicleId);
        _vehicleEngineOverrides.Remove(vehicleId);
        _vehicleDecouplerOverrides.Remove(vehicleId);
        _failedVehicleLoads.Remove(vehicleId);
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
        if (_failedVehicleLoads.Contains(vehicleId))
        {
            DefaultCategory.Log.Warning($"[AFC] Skipping the staging override save for {vehicleId}, its file failed to load.");
            return;
        }

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
            writer.WriteLine("# Per-sequence staging delay overrides for this vehicle.");
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
            DefaultCategory.Log.Error($"[AFC] Failed to save the staging overrides for {vehicleId}: {ex.Message}");
        }
    }

    private static void WriteSequenceDelays(StreamWriter writer, Dictionary<int, double> source)
    {
        var keys = new List<int>(source.Keys);
        keys.Sort();
        foreach (int key in keys)
            writer.WriteLine(string.Format(CultureInfo.InvariantCulture, "{0} = {1:F1}", key, source[key]));
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

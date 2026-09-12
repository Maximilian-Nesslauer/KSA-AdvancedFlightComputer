using System.Globalization;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.AutoRemove;

// The in-game switch, persisted to autoremove.toml next to the other AFC files. Defaults to on.
internal static class AutoRemoveConfig
{
    private static string _modDir = string.Empty;
    private static string _configPath = string.Empty;

    public static bool Enabled { get; set; } = true;

    public static void Init()
    {
        string modsDir = Path.Combine(Constants.DocumentsFolderPath, "mods");
        _modDir = Path.Combine(modsDir, "AdvancedFlightComputer");
        _configPath = Path.Combine(_modDir, "autoremove.toml");
        ImportStandaloneConfig(Path.Combine(modsDir, "AutoRemoveFinishedBurns", "autoremovefinishedburns.toml"));
        Load();
    }

    public static void Reset() => Enabled = true;

    // The switch the standalone AutoRemoveFinishedBurns mod saved carries over once.
    private static void ImportStandaloneConfig(string legacyConfig)
    {
        if (File.Exists(_configPath) || !File.Exists(legacyConfig))
            return;
        try
        {
            Directory.CreateDirectory(_modDir);
            File.Copy(legacyConfig, _configPath);
            DefaultCategory.Log.Info($"[AFC] Imported the AutoRemoveFinishedBurns setting from {legacyConfig}.");
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] Could not import the AutoRemoveFinishedBurns setting from {legacyConfig}: {ex.Message}");
        }
    }

    public static void Load()
    {
        if (!File.Exists(_configPath))
        {
            Save();
            return;
        }

        try
        {
            // One key. Strip line comments, look for enabled = true|false.
            foreach (string rawLine in File.ReadAllLines(_configPath))
            {
                int hash = rawLine.IndexOf('#');
                string line = (hash >= 0 ? rawLine.Substring(0, hash) : rawLine).Trim();
                int eq = line.IndexOf('=');
                if (eq < 1 || line.Substring(0, eq).Trim() != "enabled")
                    continue;
                if (bool.TryParse(line.Substring(eq + 1).Trim(), out bool b))
                {
                    Enabled = b;
                    break;
                }
            }
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] Failed to load {_configPath}: {ex.Message}");
        }
    }

    public static void Save()
    {
        try
        {
            Directory.CreateDirectory(_modDir);
            using var writer = new StreamWriter(_configPath);
            writer.WriteLine("# AdvancedFlightComputer automatic removal of finished burns.");
            writer.WriteLine(string.Format(CultureInfo.InvariantCulture, "enabled = {0}", Enabled ? "true" : "false"));
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] Failed to save {_configPath}: {ex.Message}");
        }
    }
}

using System.Globalization;
using Brutal.Logging;
using KSA;

namespace AutoRemoveFinishedBurns.Core;

/// <summary>
/// Persists the in-game toggle to a TOML file in the mod's user directory.
/// Defaults to enabled.
/// </summary>
static class Config
{
    private static readonly string ModDir = Path.Combine(
        Constants.DocumentsFolderPath, "mods", "AutoRemoveFinishedBurns");

    private static readonly string ConfigPath = Path.Combine(
        ModDir, "autoremovefinishedburns.toml");

    public static bool Enabled { get; set; } = true;

    public static void Init() => Load();

    public static void Reset() => Enabled = true;

    public static void Load()
    {
        if (!File.Exists(ConfigPath))
        {
            Save();
            return;
        }

        try
        {
            // One-key file. Strip line comments, look for `enabled = true|false`.
            foreach (string rawLine in File.ReadAllLines(ConfigPath))
            {
                int hash = rawLine.IndexOf('#');
                string line = (hash >= 0 ? rawLine.Substring(0, hash) : rawLine).Trim();
                int eq = line.IndexOf('=');
                if (eq < 1) continue;
                if (line.Substring(0, eq).Trim() != "enabled") continue;
                if (bool.TryParse(line.Substring(eq + 1).Trim(), out bool b))
                {
                    Enabled = b;
                    break;
                }
            }

            if (DebugConfig.Settings)
                DefaultCategory.Log.Debug(
                    $"[AutoRemoveFinishedBurns] Config loaded from '{ConfigPath}': enabled={Enabled}");
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error(
                $"[AutoRemoveFinishedBurns] Failed to load config from '{ConfigPath}': {ex.Message}");
        }
    }

    public static void Save()
    {
        try
        {
            Directory.CreateDirectory(ModDir);
            using var writer = new StreamWriter(ConfigPath);
            writer.WriteLine("# AutoRemoveFinishedBurns configuration.");
            writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "enabled = {0}", Enabled ? "true" : "false"));

            if (DebugConfig.Settings)
                DefaultCategory.Log.Debug(
                    $"[AutoRemoveFinishedBurns] Config saved to '{ConfigPath}': enabled={Enabled}");
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Error(
                $"[AutoRemoveFinishedBurns] Failed to save config to '{ConfigPath}': {ex.Message}");
        }
    }
}

using Brutal.Logging;
using KSA;

namespace AdvancedFlightComputer.Features.LandingSites;

/// <summary>
/// The named landing sites of every enabled mod, read once at load from the <see cref="LandingSiteFile.FileName"/> in
/// the mod's folder. AFC and a mod that adds landing sites do not reference each other: without such a file the list
/// is empty and nothing changes. A site that names a system or a body that is not loaded is left out when the first
/// list is asked for in that system, with one log line.
/// </summary>
internal static class LandingSiteRegistry
{
    private static readonly object _gate = new();
    private static LandingSite[] _all = [];
    private static object? _resolvedFor;
    private static Dictionary<string, LandingSite[]> _byBody = new(StringComparer.Ordinal);
    private static readonly HashSet<string> _logged = new(StringComparer.Ordinal);
    private static readonly Dictionary<string, LandingSite[]> NoSites = new(StringComparer.Ordinal);

    internal static IReadOnlyList<LandingSite> All => _all;

    /// <summary>
    /// Reads the site files of the enabled mods that the game loaded. StarMap raises the all-mods-loaded hook after ModLibrary.LoadAll, so ModLibrary.Find gives each mod with the folder that Mod.MakeUsing read it from, and a mod whose mod.toml the game refused has no sites either.
    /// </summary>
    internal static void Load()
    {
        var folders = new List<(string ModId, string Folder)>();
        ModManifest? manifest = ModLibrary.Manifest;
        if (manifest != null)
        {
            foreach (ModEntry entry in manifest.Mods)
            {
                if (entry.Enabled && !string.IsNullOrEmpty(entry.Id) && ModLibrary.Find(entry.Id) is { DirectoryPath.Length: > 0 } mod)
                    folders.Add((entry.Id, mod.DirectoryPath));
            }
        }
        LoadFrom(folders);
    }

    /// <summary>Replaces the sites with those in the site files of these folders. A file that cannot be read costs its own sites, not the load.</summary>
    internal static void LoadFrom(IEnumerable<(string ModId, string Folder)> folders)
    {
        var sites = new List<LandingSite>();
        var notes = new List<string>();
        int files = 0;
        foreach ((string modId, string folder) in folders)
        {
            string path = Path.Combine(folder, LandingSiteFile.FileName);
            if (!File.Exists(path))
                continue;
            files++;
            try
            {
                LandingSiteFile.Parse(File.ReadAllLines(path), $"{modId}/{LandingSiteFile.FileName}", sites, notes);
            }
            catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
            {
                notes.Add($"{modId}/{LandingSiteFile.FileName} could not be read: {ex.Message}");
            }
        }
        foreach (string note in notes)
            DefaultCategory.Log.Warning("[AFC] Landing sites: " + note);
        if (files > 0)
            DefaultCategory.Log.Info($"[AFC] Landing sites: {sites.Count} read from {files} mod file(s).");
        lock (_gate)
        {
            _all = sites.ToArray();
            Forget();
        }
    }

    internal static void Reset()
    {
        lock (_gate)
        {
            _all = [];
            Forget();
        }
    }

    private static void Forget()
    {
        _resolvedFor = null;
        _byBody = new Dictionary<string, LandingSite[]>(StringComparer.Ordinal);
        _logged.Clear();
    }

    /// <summary>The sites on <paramref name="parent"/> in the loaded system. The draw code asks every frame, so the answer is an array made once per system.</summary>
    internal static IReadOnlyList<LandingSite> On(IParentBody? parent)
    {
        if (parent == null || _all.Length == 0)
            return [];
        return Resolved(Universe.CurrentSystem).TryGetValue(parent.Id, out LandingSite[]? sites) ? sites : [];
    }

    // The sites whose system and body are loaded, by the id of the body, worked out once per system. The game looks a body up without regard to letter case (CelestialSystem.Get goes through KeyHash.Make), so a kept site takes the id of the body it found and the lists match it exactly.
    private static Dictionary<string, LandingSite[]> Resolved(CelestialSystem? system)
    {
        if (system == null)
            return NoSites;
        lock (_gate)
        {
            if (ReferenceEquals(_resolvedFor, system))
                return _byBody;
            var kept = new Dictionary<string, List<LandingSite>>(StringComparer.Ordinal);
            foreach (LandingSite site in _all)
            {
                IParentBody? body = system.Get(site.Body) as IParentBody;
                string why = site.System.Length > 0 && !string.Equals(site.System, system.Id, StringComparison.OrdinalIgnoreCase) ? $"is for the system '{site.System}'"
                    : body == null ? $"is on the body '{site.Body}', which the system has not"
                    : "";
                if (why.Length > 0)
                {
                    if (_logged.Add(site.Id + "\n" + system.Id))
                        DefaultCategory.Log.Info($"[AFC] Landing sites: '{site.Id}' ({site.Source}) {why}, so it is left out in '{system.Id}'.");
                    continue;
                }
                if (!kept.TryGetValue(body!.Id, out List<LandingSite>? list))
                    kept[body.Id] = list = new List<LandingSite>();
                list.Add(site with { Body = body.Id });
            }
            _byBody = kept.ToDictionary(pair => pair.Key, pair => pair.Value.ToArray(), StringComparer.Ordinal);
            _resolvedFor = system;
            return _byBody;
        }
    }

    /// <summary>The site that a target at this latitude and longitude on <paramref name="parent"/> is on, or null.</summary>
    internal static LandingSite? Holding(IParentBody? parent, double latDeg, double lonDeg)
        => parent == null ? null : LandingSiteRules.Holding(On(parent), latDeg, lonDeg, parent.MeanRadius);

    /// <summary>
    /// The height of a deck above the mean radius when the point is on a fixed site. The surface height query reads the sea under a deck that floats or stands in the water, so a fixed site gives its own height.
    /// </summary>
    internal static bool TryFixedHeight(IParentBody? parent, double latDeg, double lonDeg, out double heightM)
    {
        heightM = double.NaN;
        if (_all.Length == 0 || Holding(parent, latDeg, lonDeg) is not { Fixed: true } site)
            return false;
        heightM = site.HeightM;
        return true;
    }

    /// <summary><see cref="TryFixedHeight(IParentBody?, double, double, out double)"/> at a body-fixed (CCF) direction or position.</summary>
    internal static bool TryFixedHeight(IParentBody? parent, Brutal.Numerics.double3 ccf, out double heightM)
    {
        heightM = double.NaN;
        if (_all.Length == 0 || parent == null)
            return false;
        Brutal.Numerics.double3 lla = parent.GetLlaFromCcf(ccf);
        return TryFixedHeight(parent, lla.X, lla.Y, out heightM);
    }
}

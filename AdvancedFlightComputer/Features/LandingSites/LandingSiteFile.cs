using System.Globalization;
using AdvancedFlightComputer.Features.Ui.Logic;

namespace AdvancedFlightComputer.Features.LandingSites;

/// <summary>
/// A named landing site that a mod lists in its <see cref="LandingSiteFile.FileName"/>. Latitude and longitude are
/// body-fixed in degrees, in the convention of the landing site of the LAND page. A fixed site is a deck at
/// <see cref="HeightM"/> above the mean radius of the body. A terrain site has no height of its own, because the
/// surface height query gives it.
/// </summary>
internal sealed record LandingSite(
    string Id,
    string Name,
    string Group,
    string System,
    string Body,
    double LatDeg,
    double LonDeg,
    bool Fixed,
    double HeightM,
    double UsableRadiusM,
    string Source);

/// <summary>
/// Reads the landing site file of a mod. The file is a list of [[site]] tables with bare keys and string or number
/// values, which is the part of TOML the format needs. A bad entry is left out with one note that names the file and
/// the line, and the parse never throws.
/// </summary>
internal static class LandingSiteFile
{
    public const string FileName = "AfcLandingSites.toml";

    public const string HeightTerrain = "terrain";
    public const string HeightFixed = "fixed";

    public const double DefaultUsableRadiusM = 20.0;

    // A fixed site gives its deck height to every target in its radius, so a radius of kilometres from a typo would put landings on land at the height of a deck.
    public const double MaxUsableRadiusM = 500.0;

    private static readonly HashSet<string> KnownKeys = new(StringComparer.Ordinal)
    {
        "id", "name", "group", "system", "body", "latitude", "longitude", "height", "heightAboveMeanRadius", "usableRadius",
    };

    /// <summary>Adds the valid sites of <paramref name="lines"/> to <paramref name="into"/> and one note per entry or line that was left out or partly read.</summary>
    public static void Parse(IReadOnlyList<string> lines, string source, List<LandingSite> into, List<string> notes)
    {
        Dictionary<string, (string Value, bool Quoted)>? entry = null;
        int entryLine = 0;
        bool skipping = false;
        for (int i = 0; i < lines.Count; i++)
        {
            string line = StripComment(lines[i]).Trim();
            int lineNumber = i + 1;
            if (line.Length == 0)
                continue;
            if (line.Replace(" ", "").Replace("	", "") == "[[site]]")
            {
                Flush(entry, entryLine, source, into, notes);
                entry = new Dictionary<string, (string, bool)>(StringComparer.Ordinal);
                entryLine = lineNumber;
                skipping = false;
                continue;
            }
            if (line[0] == '[')
            {
                Flush(entry, entryLine, source, into, notes);
                entry = null;
                skipping = true;
                notes.Add($"{source}:{lineNumber}: the table '{line}' is not [[site]], so its keys are ignored.");
                continue;
            }
            if (skipping)
                continue;
            if (entry == null)
            {
                notes.Add($"{source}:{lineNumber}: a key outside any [[site]] table is ignored.");
                continue;
            }
            if (!TrySplit(line, out string key, out string value, out bool quoted))
            {
                notes.Add($"{source}:{lineNumber}: '{line}' is not key = value, so it is ignored.");
                continue;
            }
            if (entry.ContainsKey(key))
                notes.Add($"{source}:{lineNumber}: '{key}' is given twice in one site, the last value counts.");
            entry[key] = (value, quoted);
        }
        Flush(entry, entryLine, source, into, notes);
    }

    private static void Flush(Dictionary<string, (string Value, bool Quoted)>? entry, int line, string source, List<LandingSite> into, List<string> notes)
    {
        if (entry == null)
            return;
        string where = $"{source}:{line}";
        string id = Text(entry, "id");
        string label = id.Length > 0 ? $"site '{id}'" : "a site";
        var problems = new List<string>();
        string name = Text(entry, "name");
        string body = Text(entry, "body");
        if (id.Length == 0)
            problems.Add("no id");
        if (name.Length == 0)
            problems.Add("no name");
        if (body.Length == 0)
            problems.Add("no body");
        double lat = Number(entry, "latitude", problems);
        double lon = Number(entry, "longitude", problems);
        if (double.IsFinite(lat) && Math.Abs(lat) > 90.0)
            problems.Add("a latitude outside -90 to 90");
        if (double.IsFinite(lon) && Math.Abs(lon) > 360.0)
            problems.Add("a longitude outside -360 to 360");

        string height = entry.TryGetValue("height", out var h) ? h.Value.Trim() : HeightTerrain;
        bool isFixed = height == HeightFixed;
        double heightM = double.NaN;
        if (!isFixed && height != HeightTerrain)
            problems.Add($"height '{height}', which is neither '{HeightTerrain}' nor '{HeightFixed}'");
        else if (isFixed)
        {
            if (!entry.ContainsKey("heightAboveMeanRadius"))
                problems.Add("height 'fixed' with no heightAboveMeanRadius");
            else
                heightM = Number(entry, "heightAboveMeanRadius", problems);
        }

        double radius = DefaultUsableRadiusM;
        if (entry.ContainsKey("usableRadius"))
        {
            radius = Number(entry, "usableRadius", problems);
            if (double.IsFinite(radius) && (radius <= 0.0 || radius > MaxUsableRadiusM))
                problems.Add($"a usableRadius that is not above zero and at most {MaxUsableRadiusM:F0} m");
        }

        if (problems.Count > 0)
        {
            notes.Add($"{where}: {label} is left out because it has {string.Join(", ", problems)}.");
            return;
        }

        var unknown = entry.Keys.Where(k => !KnownKeys.Contains(k)).ToList();
        if (unknown.Count > 0)
            notes.Add($"{where}: {label} has keys that AFC does not read: {string.Join(", ", unknown)}.");
        if (!isFixed && entry.ContainsKey("heightAboveMeanRadius"))
            notes.Add($"{where}: {label} has height 'terrain', so its heightAboveMeanRadius is not used.");
        if (into.Exists(s => s.Id == id))
        {
            notes.Add($"{where}: {label} is left out because an earlier site has the same id.");
            return;
        }

        into.Add(new LandingSite(id, name, Text(entry, "group"), Text(entry, "system"), body,
            lat, NormalizeLonDeg(lon), isFixed, heightM, radius, source));
    }

    private static string Text(Dictionary<string, (string Value, bool Quoted)> entry, string key)
        => entry.TryGetValue(key, out var v) ? v.Value.Trim() : "";

    private static double Number(Dictionary<string, (string Value, bool Quoted)> entry, string key, List<string> problems)
    {
        if (!entry.TryGetValue(key, out var v))
        {
            problems.Add("no " + key);
            return double.NaN;
        }
        if (v.Quoted || !double.TryParse(v.Value.Replace("_", ""), NumberStyles.Float, CultureInfo.InvariantCulture, out double x) || !double.IsFinite(x))
        {
            problems.Add($"{key} '{v.Value}', which is not a number");
            return double.NaN;
        }
        return x;
    }

    /// <summary>The longitude in [-180, 180).</summary>
    public static double NormalizeLonDeg(double lonDeg)
    {
        double lon = (lonDeg + 180.0) % 360.0;
        if (lon < 0.0)
            lon += 360.0;
        return lon - 180.0;
    }

    // A '#' starts a comment only outside a string.
    private static string StripComment(string line)
    {
        char quote = '\0';
        for (int i = 0; i < line.Length; i++)
        {
            char c = line[i];
            if (quote == '"' && c == '\\')
            {
                i++;
                continue;
            }
            if (quote != '\0')
            {
                if (c == quote)
                    quote = '\0';
                continue;
            }
            if (c is '"' or '\'')
                quote = c;
            else if (c == '#')
                return line.Substring(0, i);
        }
        return line;
    }

    private static bool TrySplit(string line, out string key, out string value, out bool quoted)
    {
        key = value = "";
        quoted = false;
        int eq = line.IndexOf('=');
        if (eq < 1)
            return false;
        key = line.Substring(0, eq).Trim();
        if (key.Length >= 2 && key[0] == '"' && key[^1] == '"')
            key = key.Substring(1, key.Length - 2);
        string raw = line.Substring(eq + 1).Trim();
        if (key.Length == 0 || raw.Length == 0)
            return false;
        if (raw[0] == '\'')
        {
            if (raw.Length < 2 || raw[^1] != '\'')
                return false;
            value = raw.Substring(1, raw.Length - 2);
            quoted = true;
            return true;
        }
        if (raw[0] != '"')
        {
            value = raw;
            return true;
        }
        if (raw.Length < 2 || raw[^1] != '"')
            return false;
        var text = new System.Text.StringBuilder();
        for (int i = 1; i < raw.Length - 1; i++)
        {
            char c = raw[i];
            if (c == '"')
                return false;
            if (c != '\\')
            {
                text.Append(c);
                continue;
            }
            if (++i >= raw.Length - 1)
                return false;
            switch (raw[i])
            {
                case '"': text.Append('"'); break;
                case '\\': text.Append('\\'); break;
                case 'n': text.Append('\n'); break;
                case 't': text.Append('\t'); break;
                default: return false;
            }
        }
        value = text.ToString();
        quoted = true;
        return true;
    }
}

/// <summary>One line of a site list: the site, its distance from where the list is measured, and which booster holds it.</summary>
internal readonly record struct SiteChoice(LandingSite Site, double DistanceM, bool GroupStart, string TakenBy, bool? InReach);

/// <summary>The order of the site lists and which site a target is on. A target is on a site when it lies within the usable radius of that site, and on the nearest of them when circles overlap.</summary>
internal static class LandingSiteRules
{
    private const double DegToRad = Math.PI / 180.0;

    /// <summary>The great-circle distance on a sphere of <paramref name="radiusM"/>, m.</summary>
    public static double DistanceM(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg, double radiusM)
    {
        double p1 = lat1Deg * DegToRad, p2 = lat2Deg * DegToRad;
        double dp = p2 - p1, dl = (lon2Deg - lon1Deg) * DegToRad;
        double a = Math.Sin(dp / 2) * Math.Sin(dp / 2) + Math.Cos(p1) * Math.Cos(p2) * Math.Sin(dl / 2) * Math.Sin(dl / 2);
        return 2.0 * radiusM * Math.Asin(Math.Min(1.0, Math.Sqrt(a)));
    }

    /// <summary>
    /// The sites in list order: groups by their nearest site, and inside a group by distance. A site with no group stands as a group of its own.
    /// <paramref name="takenBy"/> names the booster that holds a site, or gives "" for a free site. <paramref name="inReach"/> gives the reach of a distance, or null where no reach is known.
    /// </summary>
    public static List<SiteChoice> Order(IReadOnlyList<LandingSite> sites, double fromLatDeg, double fromLonDeg, double radiusM,
        Func<LandingSite, string>? takenBy = null, Func<double, bool?>? inReach = null)
    {
        var measured = new List<(LandingSite Site, double M, string Key)>(sites.Count);
        foreach (LandingSite site in sites)
            measured.Add((site, DistanceM(fromLatDeg, fromLonDeg, site.LatDeg, site.LonDeg, radiusM), site.Group.Length > 0 ? site.Group : "\0" + site.Id));
        var nearest = new Dictionary<string, double>(StringComparer.Ordinal);
        foreach (var m in measured)
            nearest[m.Key] = nearest.TryGetValue(m.Key, out double d) ? Math.Min(d, m.M) : m.M;
        measured.Sort((a, b) =>
        {
            int byGroup = nearest[a.Key].CompareTo(nearest[b.Key]);
            if (byGroup == 0)
                byGroup = string.CompareOrdinal(a.Key, b.Key);
            if (byGroup != 0)
                return byGroup;
            int byDistance = a.M.CompareTo(b.M);
            return byDistance != 0 ? byDistance : string.CompareOrdinal(a.Site.Id, b.Site.Id);
        });
        var list = new List<SiteChoice>(measured.Count);
        for (int i = 0; i < measured.Count; i++)
        {
            var m = measured[i];
            bool start = m.Site.Group.Length > 0 && (i == 0 || measured[i - 1].Key != m.Key);
            list.Add(new SiteChoice(m.Site, m.M, start, takenBy?.Invoke(m.Site) ?? "", inReach?.Invoke(m.M)));
        }
        return list;
    }

    /// <summary>The site that a target at this latitude and longitude is on, or null.</summary>
    public static LandingSite? Holding(IReadOnlyList<LandingSite> sites, double latDeg, double lonDeg, double radiusM)
    {
        if (!double.IsFinite(latDeg) || !double.IsFinite(lonDeg))
            return null;
        LandingSite? best = null;
        double bestM = double.PositiveInfinity;
        foreach (LandingSite site in sites)
        {
            double m = DistanceM(latDeg, lonDeg, site.LatDeg, site.LonDeg, radiusM);
            if (m <= site.UsableRadiusM && m < bestM)
            {
                best = site;
                bestM = m;
            }
        }
        return best;
    }

    /// <summary>
    /// The booster other than <paramref name="except"/> whose target is on <paramref name="site"/>, or null. One site takes one booster, so a list gives a held site to no other booster.
    /// </summary>
    public static uint? Owner(LandingSite site, IReadOnlyList<LandingSite> sites, IEnumerable<KeyValuePair<uint, (double LatDeg, double LonDeg)>> targets, uint except, double radiusM)
    {
        foreach (var target in targets)
            if (target.Key != except && ReferenceEquals(Holding(sites, target.Value.LatDeg, target.Value.LonDeg, radiusM), site))
                return target.Key;
        return null;
    }

    /// <summary>
    /// Whether a site at <paramref name="distanceM"/> is in reach of a falling craft, by the rule of the LAND page (FormRules.SiteOutOfReach), or null when the impact time or the surface speed is not known.
    /// </summary>
    public static bool? InReachOfFall(double distanceM, double impactS, double surfaceSpeedMs)
        => double.IsFinite(impactS) && impactS >= 0.0 && double.IsFinite(surfaceSpeedMs)
            ? !FormRules.SiteOutOfReach(distanceM, impactS, surfaceSpeedMs)
            : null;
}

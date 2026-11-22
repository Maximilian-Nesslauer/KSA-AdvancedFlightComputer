using AdvancedFlightComputer.Features.LandingSites;
using AdvancedFlightComputer.Features.Ui.Logic;

// The landing site file of a mod and the site lists: the parse of valid and bad entries, the order by group and distance, which site a target is on, and one site per booster.
internal static partial class Program
{
    private const double EarthRadiusM = 6_371_000.0;

    private static readonly string[] SiteFileLines =
    [
        "# Two pads and a deck.",
        "[[site]]",
        "id = \"Example.PadA\"",
        "name = \"LZ A\"",
        "group = \"Example complex\"",
        "system = \"SolLite\"",
        "body = \"Earth\"",
        "latitude = 28.607215467",
        "longitude = -80.607800204",
        "height = \"terrain\"",
        "usableRadius = 30.0",
        "",
        "[[site]]",
        "id = \"Example.PadB\"",
        "name = \"LZ B # not a comment\"  # a comment",
        "group = \"Example complex\"",
        "body = \"Earth\"",
        "latitude = 28.604435672",
        "longitude = 279.391351508",
        "",
        "[[site]]",
        "id = 'Example.Deck'",
        "name = \"Deck\"",
        "group = \"Recovery barge\"",
        "body = \"Earth\"",
        "latitude = 28.604",
        "longitude = -80.25",
        "height = \"fixed\"",
        "heightAboveMeanRadius = 3.35",
        "usableRadius = 22.0",
        "colour = \"red\"",
    ];

    private static void LandingSiteParseRules()
    {
        var sites = new List<LandingSite>();
        var notes = new List<string>();
        LandingSiteFile.Parse(SiteFileLines, "Example/AfcLandingSites.toml", sites, notes);
        Check("the valid sites are read", sites.Count == 3, $"{sites.Count} sites, notes: {string.Join(" | ", notes)}");
        if (sites.Count != 3)
            return;
        LandingSite a = sites[0], b = sites[1], deck = sites[2];
        Check("a terrain pad keeps its place and radius", a.Id == "Example.PadA" && a.Name == "LZ A" && a.Group == "Example complex" && a.System == "SolLite"
            && a.Body == "Earth" && a.LatDeg == 28.607215467 && a.LonDeg == -80.607800204 && !a.Fixed && double.IsNaN(a.HeightM) && a.UsableRadiusM == 30.0);
        Check("a hash in a string is text, and a comment after it is not", b.Name == "LZ B # not a comment");
        Check("a site with no system is for any system", b.System.Length == 0);
        Check("a site with no height is a terrain site with the default radius", !b.Fixed && b.UsableRadiusM == LandingSiteFile.DefaultUsableRadiusM);
        Check("a longitude above 180 comes back into -180 to 180", Math.Abs(b.LonDeg - -80.608648492) < 1e-9, b.LonDeg.ToString("R"));
        Check("a literal string is read", deck.Id == "Example.Deck");
        Check("a fixed site keeps its height above the mean radius", deck.Fixed && deck.HeightM == 3.35 && deck.UsableRadiusM == 22.0);
        Check("an unknown key is a note, and the site stays", notes.Count == 1 && notes[0].Contains("colour") && notes[0].Contains(":21:"), string.Join(" | ", notes));

        string[] bad =
        [
            "id = \"Outside\"",
            "[[site]]",
            "id = \"NoName\"",
            "body = \"Earth\"",
            "latitude = 1",
            "longitude = 2",
            "[[site]]",
            "id = \"NoLatitude\"",
            "name = \"x\"",
            "body = \"Earth\"",
            "longitude = 2",
            "[[site]]",
            "id = \"TextLatitude\"",
            "name = \"x\"",
            "body = \"Earth\"",
            "latitude = \"1\"",
            "longitude = 2",
            "[[site]]",
            "id = \"FarNorth\"",
            "name = \"x\"",
            "body = \"Earth\"",
            "latitude = 91",
            "longitude = 2",
            "[[site]]",
            "id = \"FixedNoHeight\"",
            "name = \"x\"",
            "body = \"Earth\"",
            "latitude = 1",
            "longitude = 2",
            "height = \"fixed\"",
            "[[site]]",
            "id = \"OddHeight\"",
            "name = \"x\"",
            "body = \"Earth\"",
            "latitude = 1",
            "longitude = 2",
            "height = \"water\"",
            "[[ site ]]",
            "id = \"HugeRadius\"",
            "name = \"x\"",
            "body = \"Earth\"",
            "latitude = 1",
            "longitude = 2",
            "usableRadius = 100000",
            "[[site]]",
            "id = \"ZeroRadius\"",
            "name = \"x\"",
            "body = \"Earth\"",
            "latitude = 1",
            "longitude = 2",
            "usableRadius = 0",
            "[[site]]",
            "id = \"Good\"",
            "name = \"Good\"",
            "body = \"Mars\"",
            "latitude = 1",
            "longitude = 2",
            "broken line",
            "[[site]]",
            "id = \"Good\"",
            "name = \"Twice\"",
            "body = \"Earth\"",
            "latitude = 1",
            "longitude = 2",
            "[other]",
            "id = \"Ignored\"",
            "[[site]]",
            "name = \"No id\"",
            "body = \"Earth\"",
            "latitude = 1",
            "longitude = 2",
        ];
        var kept = new List<LandingSite>();
        var badNotes = new List<string>();
        LandingSiteFile.Parse(bad, "Bad/AfcLandingSites.toml", kept, badNotes);
        Check("only the valid site of a bad file is read, on any body name", kept.Count == 1 && kept[0].Id == "Good" && kept[0].Name == "Good" && kept[0].Body == "Mars",
            string.Join(", ", kept.Select(s => s.Id)));
        foreach ((string id, string why) in new[]
        {
            ("NoName", "no name"), ("NoLatitude", "no latitude"), ("TextLatitude", "not a number"), ("FarNorth", "latitude outside"),
            ("FixedNoHeight", "no heightAboveMeanRadius"), ("OddHeight", "neither"), ("ZeroRadius", "not above zero"), ("HugeRadius", "at most 500 m"), ("Good", "same id"),
        })
            Check($"a bad site says why it is left out: {id}", badNotes.Exists(n => n.Contains($"'{id}'") && n.Contains(why)), string.Join(" | ", badNotes));
        Check("a key outside any site, a line that is no key, another table and a site with no id are notes",
            badNotes.Exists(n => n.Contains("outside any")) && badNotes.Exists(n => n.Contains("not key = value")) && badNotes.Exists(n => n.Contains("[other]"))
            && badNotes.Exists(n => n.Contains("a site is left out") && n.Contains("no id")), string.Join(" | ", badNotes));
        Check("a key of another table is not a note of its own", !badNotes.Exists(n => n.Contains("Ignored")));
        var empty = new List<LandingSite>();
        var emptyNotes = new List<string>();
        LandingSiteFile.Parse(Array.Empty<string>(), "Empty", empty, emptyNotes);
        Check("an empty file has no sites and no notes", empty.Count == 0 && emptyNotes.Count == 0);
    }

    private static void LandingSiteListRules()
    {
        var sites = new List<LandingSite>();
        LandingSiteFile.Parse(SiteFileLines, "Example", sites, new List<string>());
        LandingSite a = sites[0], b = sites[1], deck = sites[2];

        // From a pad 3.5 km north of the deck, the deck is nearer than the complex, and inside the complex B comes before A from a point south of both.
        List<SiteChoice> fromSea = LandingSiteRules.Order(sites, 28.636, -80.25, EarthRadiusM);
        Check("the nearest group comes first", fromSea[0].Site == deck && fromSea[0].GroupStart, string.Join(", ", fromSea.Select(c => c.Site.Id)));
        Check("a group stands together with its head on the first site", fromSea[1].GroupStart && !fromSea[2].GroupStart
            && fromSea[1].Site.Group == fromSea[2].Site.Group);
        List<SiteChoice> fromSouth = LandingSiteRules.Order(sites, 28.5, -80.608, EarthRadiusM);
        Check("inside a group the nearer site comes first", fromSouth[0].Site == b && fromSouth[1].Site == a && fromSouth[2].Site == deck,
            string.Join(", ", fromSouth.Select(c => c.Site.Id)));
        Check("the distance is the great circle", Math.Abs(fromSouth[0].DistanceM - LandingSiteRules.DistanceM(28.5, -80.608, b.LatDeg, b.LonDeg, EarthRadiusM)) < 1e-6
            && Math.Abs(LandingSiteRules.DistanceM(0, 0, 0, 1, EarthRadiusM) - EarthRadiusM * Math.PI / 180.0) < 1e-6);
        Check("a list with no reach rule marks nothing", fromSouth.TrueForAll(c => c.InReach == null && c.TakenBy.Length == 0));

        var loose = new List<LandingSite>
        {
            a with { Id = "L1", Group = "" },
            a with { Id = "L2", Group = "", LatDeg = a.LatDeg + 0.1 },
        };
        List<SiteChoice> ungrouped = LandingSiteRules.Order(loose, a.LatDeg, a.LonDeg, EarthRadiusM);
        Check("a site with no group has no head", ungrouped.TrueForAll(c => !c.GroupStart) && ungrouped[0].Site.Id == "L1");

        Check("a target on a pad is on that pad", LandingSiteRules.Holding(sites, a.LatDeg, a.LonDeg + 0.0001, EarthRadiusM) == a);
        Check("a target off every usable radius is on no site", LandingSiteRules.Holding(sites, a.LatDeg + 0.001, a.LonDeg, EarthRadiusM) == null);
        Check("no target is on no site", LandingSiteRules.Holding(sites, double.NaN, double.NaN, EarthRadiusM) == null);

        // Two boosters: 7 holds pad A, 9 asks for its list.
        var targets = new Dictionary<uint, (double LatDeg, double LonDeg)> { [7] = (a.LatDeg, a.LonDeg), [9] = (b.LatDeg, b.LonDeg) };
        Check("a pad that another booster holds names it", LandingSiteRules.Owner(a, sites, targets, 9, EarthRadiusM) == 7u);
        Check("the booster's own pad is free to it", LandingSiteRules.Owner(b, sites, targets, 9, EarthRadiusM) == null);
        Check("a pad nobody holds is free", LandingSiteRules.Owner(deck, sites, targets, 9, EarthRadiusM) == null);
        List<SiteChoice> forNine = LandingSiteRules.Order(sites, 28.5, -80.608, EarthRadiusM,
            site => LandingSiteRules.Owner(site, sites, targets, 9, EarthRadiusM) is uint owner ? "S" + owner : "");
        Check("the list of the second booster gives the held pad to nobody else", forNine.Single(c => c.Site == a).TakenBy == "S7"
            && forNine.Where(c => c.Site != a).All(c => c.TakenBy.Length == 0));

        Check("the fixed site gives its deck height and a terrain pad gives none", deck.Fixed && deck.HeightM == 3.35 && !a.Fixed && !b.Fixed);
        Check("a target on the deck is on the fixed site", LandingSiteRules.Holding(sites, deck.LatDeg + 0.0001, deck.LonDeg, EarthRadiusM) is { Fixed: true, HeightM: 3.35 });

        Check("reach is unknown with no impact time", LandingSiteRules.InReachOfFall(1000.0, double.NaN, 100.0) == null);
        Check("a site near the fall is in reach", LandingSiteRules.InReachOfFall(10_000.0, 60.0, 300.0) == true);
        Check("a site far from the fall is out of reach", LandingSiteRules.InReachOfFall(100_000.0, 60.0, 300.0) == false);
        Check("a line of the list holds the name, the distance, the holder and the reach",
            PageText.SiteChoice("LZ A", 15_000.0, "S3.1", true) == "LZ A  15.0 km  S3.1  IN REACH" && PageText.SiteChoice("Deck", 20.0, "", null) == "Deck  20 m",
            PageText.SiteChoice("LZ A", 15_000.0, "S3.1", true));
        Check("a booster on a named site says its name", PageText.StageReturn(true, false, 1.0, 2.0, "LZ A") == "TO LZ A"
            && PageText.StageReturn(true, false, 1.0, 2.0) == "TO SITE " + Fmt.LatLon(1.0, 2.0));
    }
}

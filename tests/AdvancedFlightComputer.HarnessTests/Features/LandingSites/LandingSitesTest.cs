using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.LandingSites;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// The named landing sites of a mod, read from a site file in a test folder on the home body of the loaded system.
//   load:    a terrain pad and a fixed deck are read, and a site for another system and a site on a body the system has not are left out.
//   lists:   both sites stand in the RETURN TO list of a booster measured from the pad and in the LAND AT list measured from the deck, nearest first.
//   targets: a pick writes the file's latitude and longitude as the booster's target, and a second booster on the stage list cannot take the same pad.
//   height:  the site height the landing reads is the surface at the pad, the file's height on the deck, and the surface again just off the deck.
public sealed class LandingSitesTest : AfcTest
{
    private const double PadLat = 28.607215467, PadLon = -80.607800204;
    private const double DeckLat = 28.604, DeckLon = -80.25, DeckHeightM = 3.35, DeckRadiusM = 22.0;

    public override string Name => "afc-landing-sites";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;
        if (!t.Check("the harness system is the loaded system", ReferenceEquals(Universe.CurrentSystem, t.System)))
            return;

        string folder = Path.Combine(Path.GetTempPath(), "AfcLandingSitesTest_" + Guid.NewGuid().ToString("N"));
        var ambient = AccessTools.Field(typeof(GuidanceWindow), "_s");
        object? previous = ambient.GetValue(null);
        bool hadSites = LandingSiteRegistry.All.Count > 0;
        try
        {
            Directory.CreateDirectory(folder);
            File.WriteAllLines(Path.Combine(folder, LandingSiteFile.FileName), SiteFile(t.System.Id, home.Id));
            LandingSiteRegistry.LoadFrom([("AfcLandingSitesTest", folder)]);

            IReadOnlyList<LandingSite> on = LandingSiteRegistry.On(home);
            t.Info($"read {LandingSiteRegistry.All.Count} sites, {on.Count} on {home.Id}: {string.Join(", ", on.Select(s => s.Id))}");
            LandingSite? pad = on.FirstOrDefault(s => s.Id == "Test.Pad");
            LandingSite? deck = on.FirstOrDefault(s => s.Id == "Test.Deck");
            t.Check("all four sites are read from the file", LandingSiteRegistry.All.Count == 4);
            t.Check("the sites of another system and of a missing body are left out", on.Count == 2 && pad != null && deck != null);
            if (pad == null || deck == null)
                return;
            t.Check("the pad keeps the file's place", pad.LatDeg == PadLat && pad.LonDeg == PadLon && !pad.Fixed);
            t.Check("the deck keeps the file's height, and its body in other letter case is the home body", deck.Body == home.Id && deck.Fixed && deck.HeightM == DeckHeightM && deck.UsableRadiusM == DeckRadiusM);

            double3 padCcf = GuidanceWindow.SiteDirCcf(PadLat, PadLon) * home.MeanRadius;
            double3 deckCcf = GuidanceWindow.SiteDirCcf(DeckLat, DeckLon) * home.MeanRadius;
            var state = new VehicleAutopilotState();
            // Two side boosters on one row, as the stage list holds them before separation.
            state.ReturnableStages.Add(new GuidanceWindow.ReturnableStage { RootId = 101, Name = "Booster", SequenceIndex = 0, SequenceNumber = 3 });
            state.ReturnableStages.Add(new GuidanceWindow.ReturnableStage { RootId = 102, Name = "Booster", SequenceIndex = 0, SequenceNumber = 3 });

            List<SiteChoice> launch = GuidanceWindow.StageSiteChoices(state, home, padCcf, 101);
            t.Info("RETURN TO from the pad: " + string.Join(", ", launch.Select(c => $"{c.Site.Name} {c.DistanceM:F0} m")));
            t.Check("RETURN TO lists the pad first and the deck", launch.Count == 2 && launch[0].Site == pad && launch[1].Site == deck && launch[0].DistanceM < 1.0);
            List<SiteChoice> land = GuidanceWindow.LandSiteChoices(home, deckCcf, double.NaN, 0.0);
            t.Info("LAND AT from the deck: " + string.Join(", ", land.Select(c => $"{c.Site.Name} {c.DistanceM:F0} m")));
            t.Check("LAND AT lists the deck first and the pad, with no reach mark on the ground", land.Count == 2 && land[0].Site == deck && land[1].Site == pad
                && land.TrueForAll(c => c.InReach == null));

            t.Check("a booster takes the pad", GuidanceWindow.SetStageTarget(state, 101, pad.LatDeg, pad.LonDeg, home));
            t.Check("its target is the file's latitude and longitude", state.StageTargets.TryGetValue(101, out double2 target) && target.X == PadLat && target.Y == PadLon,
                $"{target.X:R}, {target.Y:R}");
            t.Check("a second booster cannot take the same pad", !GuidanceWindow.SetStageTarget(state, 102, pad.LatDeg, pad.LonDeg, home) && !state.StageTargets.ContainsKey(102),
                state.LandingStatus);
            List<SiteChoice> second = GuidanceWindow.StageSiteChoices(state, home, padCcf, 102);
            t.Check("the list of the second booster shows the pad as held by the first", second.Single(c => c.Site == pad).TakenBy == "S3.1"
                && second.Single(c => c.Site == deck).TakenBy.Length == 0);
            t.Check("the second booster takes the deck", GuidanceWindow.SetStageTarget(state, 102, deck.LatDeg, deck.LonDeg, home)
                && state.StageTargets[102].X == DeckLat && state.StageTargets[102].Y == DeckLon);

            // The booster hand-over gives the target to the separated booster as its site, which the landing reads its height at.
            ambient.SetValue(null, state);
            var siteHeight = AccessTools.Method(typeof(GuidanceWindow), "SiteTerrainHeight");
            double Height(double lat, double lon)
            {
                state.SiteLatDeg = lat;
                state.SiteLonDeg = lon;
                return (double)siteHeight.Invoke(null, [home])!;
            }
            double padSurface = KsaSurfaceHeight.At(home, GuidanceWindow.SiteDirCcf(PadLat, PadLon));
            double padHeight = Height(PadLat, PadLon);
            t.Info($"pad: site height {padHeight:F3} m, surface {padSurface:F3} m");
            t.CheckAbs("the pad's height is the surface height", padHeight, padSurface, 1e-9);
            double deckSurface = KsaSurfaceHeight.At(home, GuidanceWindow.SiteDirCcf(DeckLat, DeckLon));
            double deckHeight = Height(state.StageTargets[102].X, state.StageTargets[102].Y);
            t.Info($"deck: site height {deckHeight:F3} m, surface under it {deckSurface:F3} m");
            t.CheckAbs("the deck's height is the file's height", deckHeight, DeckHeightM, 1e-12);
            double offLat = DeckLat + 2.0 * DeckRadiusM / home.MeanRadius * 180.0 / Math.PI;
            t.CheckAbs("just off the deck the height is the surface again", Height(offLat, DeckLon),
                KsaSurfaceHeight.At(home, GuidanceWindow.SiteDirCcf(offLat, DeckLon)), 1e-9);
            t.Check("the deorbit planner reads the same deck height", LandingSiteRegistry.TryFixedHeight(home, DeckLat, DeckLon, out double planned) && planned == DeckHeightM
                && !LandingSiteRegistry.TryFixedHeight(home, PadLat, PadLon, out _));
            t.Check("the height under a craft over the deck is the deck", LandingSiteRegistry.TryFixedHeight(home, deckCcf * 1.001, out double under) && under == DeckHeightM);
        }
        finally
        {
            ambient.SetValue(null, previous);
            LandingSiteRegistry.Reset();
            if (hadSites)
                LandingSiteRegistry.Load();
            try
            {
                Directory.Delete(folder, recursive: true);
            }
            catch (IOException)
            {
            }
        }
    }

    private static string[] SiteFile(string system, string body) =>
    [
        "[[site]]",
        "id = \"Test.Pad\"",
        "name = \"Test pad\"",
        "group = \"Test complex\"",
        $"system = \"{system}\"",
        $"body = \"{body}\"",
        FormattableString.Invariant($"latitude = {PadLat:R}"),
        FormattableString.Invariant($"longitude = {PadLon:R}"),
        "height = \"terrain\"",
        "usableRadius = 30.0",
        "",
        "[[site]]",
        "id = \"Test.Deck\"",
        "name = \"Test deck\"",
        "group = \"Test barge\"",
        $"body = \"{body.ToUpperInvariant()}\"",
        FormattableString.Invariant($"latitude = {DeckLat:R}"),
        FormattableString.Invariant($"longitude = {DeckLon:R}"),
        "height = \"fixed\"",
        FormattableString.Invariant($"heightAboveMeanRadius = {DeckHeightM:R}"),
        FormattableString.Invariant($"usableRadius = {DeckRadiusM:R}"),
        "",
        "[[site]]",
        "id = \"Test.OtherSystem\"",
        "name = \"Elsewhere\"",
        "system = \"NoSuchSystem\"",
        $"body = \"{body}\"",
        "latitude = 1",
        "longitude = 2",
        "",
        "[[site]]",
        "id = \"Test.NoBody\"",
        "name = \"Nowhere\"",
        "body = \"NoSuchBody\"",
        "latitude = 1",
        "longitude = 2",
    ];
}

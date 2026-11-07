using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal enum ConsolePage { Launch, Land, Orbit, Transfer, Rcs, Staging, Tune }

/// <summary>The ways of the LAND page. The saved keys are the enum names, so a new way goes at the end; the WAY row draws them in <see cref="LandWays.DisplayOrder"/>.</summary>
internal enum LandWay { FromOrbit, FromHere, Hover, Boostback }
internal enum TuneSet { Afc, Ascent, Recover, Landing, Diag }
internal enum TuneLanding { Approach, Entry, SixDof, Gfold, Hover }
internal enum TuneDiagnostics { Ascent, Boostback, Aero, Atmosphere, Gfold, Log, FlightPlan }
internal enum AscentChoice { CatsUpfg, GravityTurn }

internal static class ConsolePages
{
    /// <summary>A page key that ui.toml files and cue targets of earlier versions can hold. It reads as LAND, and ResolveLegacy gives the page and the way from its saved step.</summary>
    public const string RecoverAlias = "RECOVER";

    public static readonly ConsolePage[] All =
    {
        ConsolePage.Launch, ConsolePage.Orbit, ConsolePage.Transfer, ConsolePage.Rcs, ConsolePage.Land,
        ConsolePage.Staging, ConsolePage.Tune,
    };

    public const ConsolePage FirstSetupPage = ConsolePage.Staging;

    public static string Label(ConsolePage page) => page switch
    {
        ConsolePage.Launch => "LAUNCH",
        ConsolePage.Land => "LAND",
        ConsolePage.Orbit => "MANEUVER",
        ConsolePage.Transfer => "TRANSFER",
        ConsolePage.Rcs => "BURNS",
        ConsolePage.Staging => "STAGING",
        ConsolePage.Tune => "TUNE",
        _ => page.ToString().ToUpperInvariant(),
    };

    /// <summary>The key a page is saved and cued under. It is not the label, so a renamed page keeps its saved key.</summary>
    public static string Key(ConsolePage page) => page switch
    {
        ConsolePage.Launch => "LAUNCH",
        ConsolePage.Land => "LAND",
        ConsolePage.Orbit => "ORBIT",
        ConsolePage.Transfer => "TRANSFER",
        ConsolePage.Rcs => "RCS",
        ConsolePage.Staging => "STAGING",
        ConsolePage.Tune => "TUNE",
        _ => page.ToString().ToUpperInvariant(),
    };

    /// <summary>The hover card of a page in the chooser.</summary>
    public static string HoverCard(ConsolePage page) => PageText.ChooserCard(page);

    public static bool WorksInEditor(ConsolePage page)
        => page is ConsolePage.Transfer or ConsolePage.Staging or ConsolePage.Tune;

    /// <summary>The page of a saved or cued key. The old booster key reads as LAND; <see cref="ResolveLegacy"/> gives its way.</summary>
    public static ConsolePage PageFromKey(string? key, ConsolePage fallback)
    {
        if (string.IsNullOrWhiteSpace(key))
            return fallback;
        string k = key.Trim();
        foreach (ConsolePage page in All)
        {
            if (string.Equals(Key(page), k, StringComparison.OrdinalIgnoreCase))
                return page;
        }
        if (string.Equals(k, RecoverAlias, StringComparison.OrdinalIgnoreCase))
            return ConsolePage.Land;
        return fallback;
    }

    /// <summary>
    /// Where an old booster key and its step lead now: BEFORELAUNCH to LAUNCH, where the stage sites
    /// are, and every other step to LAND on way BOOSTBACK. False for any other key.
    /// </summary>
    public static bool ResolveLegacy(string? key, string? step, out ConsolePage page, out LandWay? way)
    {
        page = ConsolePage.Land;
        way = null;
        if (key == null || !string.Equals(key.Trim(), RecoverAlias, StringComparison.OrdinalIgnoreCase))
            return false;
        if (step != null && string.Equals(step.Trim(), "BEFORELAUNCH", StringComparison.OrdinalIgnoreCase))
        {
            page = ConsolePage.Launch;
            return true;
        }
        way = LandWay.Boostback;
        return true;
    }

    /// <summary>Sub-page keys are the enum names in upper case.</summary>
    public static string EnumKey<T>(T value) where T : struct, Enum => value.ToString().ToUpperInvariant();

    public static T EnumFromKey<T>(string? key, T fallback) where T : struct, Enum
        => !string.IsNullOrWhiteSpace(key) && Enum.TryParse(key.Trim(), ignoreCase: true, out T value)
           && Enum.IsDefined(value)
            ? value
            : fallback;

    public static string TuneSetLabel(TuneSet set) => set switch
    {
        TuneSet.Afc => "PREFERENCES",
        TuneSet.Ascent => "ASCENT",
        TuneSet.Recover => "BOOSTBACK",
        TuneSet.Landing => "LANDING",
        _ => "DIAG",
    };

    /// <summary>The guidance mode the footer of a page commits, or null for a page that commits no guidance mode.</summary>
    public static GuidanceModeId? ModeIdFor(ConsolePage page, LandWay way, LandSolver solver, AscentChoice ascent)
        => page switch
        {
            ConsolePage.Launch => ascent == AscentChoice.GravityTurn ? GuidanceModeId.AscentGravityTurn : GuidanceModeId.AscentUpfg,
            ConsolePage.Land => way switch
            {
                LandWay.FromOrbit => GuidanceModeId.DeorbitLand,
                LandWay.Boostback => GuidanceModeId.Boostback,
                LandWay.FromHere => solver switch
                {
                    LandSolver.Entry => GuidanceModeId.EntryHere,
                    LandSolver.Touchdown => GuidanceModeId.SixDofHere,
                    _ => GuidanceModeId.GfoldHere,
                },
                _ => GuidanceModeId.Hover,
            },
            _ => null,
        };

    /// <summary>The page a stop target belongs to, for OPEN on the strip and for the first page while something flies. A multi-pass of a transfer belongs to TRANSFER, one of an orbit tool to MANEUVER, and a registered mode to the page of its descriptor.</summary>
    public static ConsolePage? PageOf(StopTarget target, bool transferPasses = false, GuidanceModeId? registered = null) => target switch
    {
        StopTarget.Ascent or StopTarget.LaunchArmed => ConsolePage.Launch,
        StopTarget.Boostback => ConsolePage.Land,
        StopTarget.Landing or StopTarget.SixDof or StopTarget.Hover => ConsolePage.Land,
        StopTarget.Rcs or StopTarget.StockAuto => ConsolePage.Rcs,
        StopTarget.MultiPass => transferPasses ? ConsolePage.Transfer : ConsolePage.Orbit,
        StopTarget.Registered when registered is GuidanceModeId id => GuidanceModes.Of(id)?.Page,
        _ => null,
    };

    /// <summary>Cue targets are "PAGE" or "PAGE/SUBPAGE", for example "LAND/HOVER".</summary>
    public static string CueTarget(ConsolePage page, string subPage = "")
        => subPage.Length > 0 ? Key(page) + "/" + subPage : Key(page);

    public static bool TryParseCue(string cue, out ConsolePage page, out string subPage)
    {
        subPage = "";
        page = ConsolePage.Launch;
        if (string.IsNullOrEmpty(cue))
            return false;
        int slash = cue.IndexOf('/');
        string head = slash >= 0 ? cue.Substring(0, slash) : cue;
        subPage = slash >= 0 ? cue.Substring(slash + 1) : "";
        ConsolePage parsed = PageFromKey(head, (ConsolePage)(-1));
        if ((int)parsed < 0)
            return false;
        page = parsed;
        return true;
    }
}

/// <summary>The WAY row of the LAND page: the ways from high energy down to the ground, each one phase lower than the one before it.</summary>
internal static class LandWays
{
    public static readonly LandWay[] DisplayOrder = { LandWay.FromOrbit, LandWay.Boostback, LandWay.FromHere, LandWay.Hover };

    public static string Label(LandWay way) => way switch
    {
        LandWay.FromOrbit => "DEORBIT",
        LandWay.Boostback => "BOOSTBACK",
        LandWay.FromHere => "LAND NOW",
        _ => "HOVER",
    };

    /// <summary>The labels in <see cref="DisplayOrder"/>.</summary>
    public static readonly string[] Labels = Array.ConvertAll(DisplayOrder, Label);

    public static int DisplayIndex(LandWay way) => Array.IndexOf(DisplayOrder, way);

    /// <summary>Whether a way picks a powered-descent solver.</summary>
    public static bool HasSolver(LandWay way) => way is LandWay.FromOrbit or LandWay.FromHere;

    public static LandWay? Follow(LiveLanding previous, LiveLanding live)
    {
        if (live == previous)
            return null;
        return live switch
        {
            LiveLanding.Boostback => LandWay.Boostback,
            LiveLanding.Entry when previous == LiveLanding.Boostback => LandWay.FromHere,
            LiveLanding.Touchdown or LiveLanding.Gfold => LandWay.FromHere,
            LiveLanding.Hover => LandWay.Hover,
            _ => null,
        };
    }

    public static LandWay? FitsNow(CraftSituation where, bool hasAir, bool belowBrakingStart) => where switch
    {
        CraftSituation.Orbit => LandWay.FromOrbit,
        CraftSituation.CoastingUp => hasAir ? LandWay.Boostback : LandWay.FromOrbit,
        CraftSituation.FallingAir => LandWay.FromHere,
        CraftSituation.FallingNoAir => belowBrakingStart ? LandWay.FromHere : LandWay.FromOrbit,
        _ => null,
    };
}

/// <summary>
/// Facts about the subject craft that choose the page the console opens on when no page was used in
/// this flight yet. The game side fills it, so the choice can be tested without the game.
/// </summary>
internal readonly record struct FlightFacts(
    bool Editor, StopTarget Running, bool Landed, bool HasEngines, bool InOrbit, bool CelestialTargetNotParent, bool TransferPasses = false,
    GuidanceModeId? RegisteredMode = null, CraftSituation Situation = CraftSituation.Unknown);

internal static class FlightContext
{
    /// <summary>
    /// The first page: the page of a running mode first, then the situation. In the editor STAGING,
    /// on the ground with engines LAUNCH, a craft that falls LAND, in orbit with a celestial target
    /// that is not the parent TRANSFER, any other craft that flies MANEUVER, else LAUNCH.
    /// </summary>
    public static ConsolePage FirstPage(in FlightFacts f)
    {
        if (f.Editor)
            return ConsolePage.Staging;
        if (ConsolePages.PageOf(f.Running, f.TransferPasses, f.RegisteredMode) is ConsolePage running)
            return running;
        if (f.Landed && f.HasEngines)
            return ConsolePage.Launch;
        if (SituationText.IsFalling(f.Situation))
            return ConsolePage.Land;
        if (f.InOrbit && f.CelestialTargetNotParent)
            return ConsolePage.Transfer;
        if (f.InOrbit || f.Situation is CraftSituation.Orbit or CraftSituation.CoastingUp or CraftSituation.Escaping)
            return ConsolePage.Orbit;
        return ConsolePage.Launch;
    }
}

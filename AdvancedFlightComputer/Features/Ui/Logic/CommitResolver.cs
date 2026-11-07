using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal enum GuidanceModeId { AscentUpfg, AscentGravityTurn, Boostback, DeorbitLand, GfoldHere, SixDofHere, Hover, EntryHere }

/// <summary>
/// Facts about the craft that can refuse a commit before it is clicked, or change its verb. An empty
/// refusal text means no refusal. <see cref="PlansFirst"/> is true when the launch commit has no convex
/// plan that fits and calculates one before it launches or arms. <see cref="BackupChosen"/> is true when
/// the player answered a failed convex plan with the backup gravity turn, which the launch verb then flies.
/// </summary>
internal readonly record struct CommitGates(
    bool HasTarget, bool LandingSwitchesOff, string DeorbitRefusal, string GravityTurnRefusal, bool HoverTwrBelowOne,
    bool PlansFirst = false, bool BackupChosen = false,
    CraftSituation Where = CraftSituation.Unknown, bool HasAir = false, bool BelowBrakingStart = false, bool GfoldChosen = false,
    string HoverProblem = "", string PageFact = "", string NoSiteNote = "", string FarSiteNote = "",
    ConsolePage? PageFactNext = null, LandSolver? PageFactSolver = null, bool PageFactCalm = false, bool NoStageHome = false);

/// <summary><see cref="PressEnds"/> says that the mode ends a press at once: StartTerminalHover refuses a craft with a hover problem before it claims anything, and PrepareLandingEngines ends G-FOLD on its first step on a body with air, after its start has claimed the craft.</summary>
internal readonly record struct CommitView(string Verb, bool Enabled, string Reason, bool NeedsTakeOver, bool Live, string Detail = "", VerbFit Fit = default, bool PressEnds = false)
{
    public string ButtonLabel => NeedsTakeOver ? PlanVerbs.TakeOver(Verb) : Verb;

    public bool Lit => Enabled && Fit.Fits && !NeedsTakeOver;

    public bool EndsOnly => PressEnds && NeedsTakeOver;
}

internal static class CommitResolver
{
    public const string GravityTurnUnavailable = GuidanceModes.GravityTurnUnavailable;
    public const string LandingSwitchesOffReason = "NEEDS AFC MAY STEER + MAY USE ENGINES";
    public const string DeorbitSettingsReason = "LANDING SETTING OUT OF RANGE";

    /// <summary>The status of a landing while stock AUTO flies its deorbit burn, so the footer does not give that burn to the landing.</summary>
    public const string StockDeorbitStatus = "STOCK AUTO: DEORBIT BURN";

    public static readonly string LandingSwitchesOffDetail =
        "LANDING NEEDS " + GuidanceSettingSpecs.Engage.Label + " + "
        + GuidanceSettingSpecs.AutoStage.Label + " ON. BOTH IN GNC SWITCHES (MORE). HEAD SHOWS WHEN ONE IS OFF.";

    public static CommitView Resolve(GuidanceModeId mode, in CraftActivity a, in CommitGates g)
    {
        (string verb, string reason, bool live) = mode switch
        {
            GuidanceModeId.AscentUpfg => (LaunchVerb(in g),
                a.AscentRunning ? "AFC: ASCENT"
                : a.LaunchArmed ? "LAUNCH ARMED"
                : a.ConvexLaunchPending ? "PLANNING ASCENT"
                : "",
                a.AscentRunning || a.LaunchArmed || a.ConvexLaunchPending),
            GuidanceModeId.AscentGravityTurn when g.GravityTurnRefusal.Length > 0 => ("LAUNCH", g.GravityTurnRefusal, false),
            GuidanceModeId.Boostback => ("BOOSTBACK",
                a.BoostbackLive ? "AFC: BOOSTBACK" : "", a.BoostbackLive),
            GuidanceModeId.DeorbitLand => ("DEORBIT + LAND",
                g.LandingSwitchesOff ? LandingSwitchesOffReason
                : g.DeorbitRefusal.Length > 0 ? DeorbitSettingsReason
                : a.DescentLive ? (a.LandingStockAuto ? StockDeorbitStatus : "AFC: LANDING")
                : "",
                !g.LandingSwitchesOff && g.DeorbitRefusal.Length == 0 && a.DescentLive),
            GuidanceModeId.GfoldHere => (GfoldVerb,
                a.LandingMachineLive ? "AFC: POWERED DESCENT" : "", a.LandingMachineLive),
            GuidanceModeId.SixDofHere => ("TOUCHDOWN",
                a.SixDofEngaged ? "AFC: TOUCHDOWN" : "", a.SixDofEngaged),
            GuidanceModeId.Hover => ("HOVER",
                a.HoverLive ? "AFC: HOVER"
                : g.HoverTwrBelowOne ? "NO HOVER: TWR BELOW 1"
                : "",
                a.HoverLive),
            _ => RegisteredView(mode, in a),
        };

        string detail = reason == LandingSwitchesOffReason ? LandingSwitchesOffDetail
            : reason == DeorbitSettingsReason ? g.DeorbitRefusal
            : "";
        bool enabled = reason.Length == 0;
        StopTarget running = StopResolver.Resolve(in a);
        bool hoverRefused = mode == GuidanceModeId.Hover && g.HoverProblem.Length > 0;
        bool pressEnds = enabled && (hoverRefused || (mode == GuidanceModeId.GfoldHere && g.HasAir));
        bool takeOver = enabled && !hoverRefused && ((StopResolver.IsGuidance(running) && !SameMode(mode, running, in a))
            || ReplacesLandingWay(mode, in a));
        VerbFit fit = !enabled ? VerbFit.Yes
            : a.AscentRunning && mode is GuidanceModeId.Boostback or GuidanceModeId.DeorbitLand or GuidanceModeId.GfoldHere or GuidanceModeId.SixDofHere or GuidanceModeId.EntryHere or GuidanceModeId.Hover
                ? (pressEnds && takeOver ? SituationVerbs.AscentFliesEndsFit : SituationVerbs.AscentFliesLandFit)
            : a.DeorbitChainLive && mode is GuidanceModeId.Boostback or GuidanceModeId.GfoldHere or GuidanceModeId.SixDofHere or GuidanceModeId.EntryHere or GuidanceModeId.Hover
                ? SituationVerbs.DeorbitFliesFitFor(mode, in g)
                : SituationVerbs.For(mode, in g);
        if (enabled && fit.Fits && g.PageFact.Length > 0)
            fit = new VerbFit(g.PageFact, g.PageFactNext, Plain: g.PageFactCalm, NextSolver: g.PageFactSolver);
        string endsWhy = !pressEnds ? "" : hoverRefused ? g.HoverProblem : SituationVerbs.GfoldNoAir;
        if (endsWhy.Length > 0 && !fit.Text.StartsWith(endsWhy, StringComparison.Ordinal))
            detail = endsWhy;
        return new CommitView(verb, enabled, reason, takeOver, live && !enabled, detail, fit, pressEnds);
    }

    /// <summary>
    /// The verb and refusal of a mode that registered later than the built-in ones, read from its
    /// descriptor: refused with its own reason while unavailable, "&lt;Mode&gt; flies" while it flies.
    /// </summary>
    private static (string Verb, string Reason, bool Live) RegisteredView(GuidanceModeId mode, in CraftActivity a)
    {
        GuidanceModeDescriptor? d = GuidanceModes.Of(mode);
        if (d == null)
            return ("", "UNKNOWN MODE", false);
        string verb = d.CommitVerb.Length > 0 ? d.CommitVerb : d.MethodLabel;
        if (!d.Available)
            return (verb, d.UnavailableReason.Length > 0 ? d.UnavailableReason : "NOT AVAILABLE", false);
        bool live = a.RegisteredModeLive == mode;
        return (verb, live ? "AFC: " + (d.ModeName.Length > 0 ? d.ModeName : d.MethodLabel) : "", live);
    }

    public const string BackupLaunchVerb = "LAUNCH (BACKUP)";

    public const string GfoldVerb = "G-FOLD";

    /// <summary>The launch verb: LAUNCH or ARM, SOLVE + LAUNCH / ARM while the commit solves a convex plan first, or LAUNCH (BACKUP) after the backup choice.</summary>
    public static string LaunchVerb(in CommitGates g)
        => g.BackupChosen ? BackupLaunchVerb
        : g.PlansFirst
            ? (g.HasTarget ? "SOLVE + ARM" : "SOLVE + LAUNCH")
            : (g.HasTarget ? "ARM" : "LAUNCH");

    public static bool LaunchVerbArms(string verb) => verb is "ARM" or "SOLVE + ARM";

    private static bool SameMode(GuidanceModeId mode, StopTarget running, in CraftActivity a) => mode switch
    {
        GuidanceModeId.AscentUpfg => running is StopTarget.Ascent or StopTarget.LaunchArmed,
        GuidanceModeId.Boostback => running == StopTarget.Boostback,
        GuidanceModeId.SixDofHere => running == StopTarget.SixDof,
        GuidanceModeId.DeorbitLand or GuidanceModeId.GfoldHere or GuidanceModeId.Hover => running is StopTarget.Landing or StopTarget.Hover,
        _ => running == StopTarget.Registered && a.RegisteredModeLive == mode,
    };

    // Every landing way ends with the landing stop, so a switch from one way to another is a take-over of its own. The hover after the powered descent or its terminal burn continues the same descent (StartTerminalHover keeps its flown path), so it does not ask.
    private static bool ReplacesLandingWay(GuidanceModeId mode, in CraftActivity a) => mode switch
    {
        GuidanceModeId.GfoldHere => a.DeorbitChainLive || a.HoverLive,
        GuidanceModeId.Hover => a.DeorbitChainLive,
        _ => false,
    };

    public static string ModeName(StopTarget running, GuidanceModeId? registered = null) => running switch
    {
        StopTarget.SixDof => "TOUCHDOWN",
        StopTarget.Landing => "LANDING",
        StopTarget.Hover => "HOVER",
        StopTarget.Boostback => "BOOSTBACK",
        StopTarget.Ascent => "ASCENT",
        StopTarget.LaunchArmed => "ARMED LAUNCH",
        StopTarget.Registered when registered is GuidanceModeId id && GuidanceModes.Of(id) is GuidanceModeDescriptor d
            => d.ModeName.Length > 0 ? d.ModeName : d.MethodLabel + " MODE",
        _ => "ANOTHER MODE",
    };

    public static string TakeOverMessage(StopTarget running, string verb, GuidanceModeId? registered = null)
        => verb + " ENDS " + PageText.InSentence(ModeName(running, registered)) + ". CONTINUE?";

    public static string EndsOnlyMessage(StopTarget running, GuidanceModeId? registered = null)
        => "ONLY ENDS " + PageText.InSentence(ModeName(running, registered)) + ". CONTINUE?";
}

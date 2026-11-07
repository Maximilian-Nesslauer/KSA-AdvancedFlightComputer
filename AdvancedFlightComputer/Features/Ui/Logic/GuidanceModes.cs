using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>
/// The rules of one guidance mode that the shell decides by, free of the game: the page it lives on,
/// its method label, when it is live, which stop ends it, the order it outranks other modes in for the
/// one stop, and its phase list for the head phase strip. The game side keys its draw and commit by
/// the same id: an ascent method through its AscentMethodUi entry, and the commit through
/// GuidanceWindow.Commit. <see cref="CommitVerb"/> and the stop texts are read only for a mode that
/// registered through <see cref="GuidanceModes.Registered"/>, because the built-in modes keep their
/// verbs and stop texts in CommitResolver and StopResolver.
/// </summary>
internal sealed record GuidanceModeDescriptor(
    GuidanceModeId Id,
    ConsolePage Page,
    string MethodLabel,
    int StopOrder,
    Func<CraftActivity, bool> IsLive,
    Func<CraftActivity, StopTarget> Stop,
    IReadOnlyList<string> Phases,
    bool Available = true,
    string UnavailableReason = "",
    string CommitVerb = "",
    string StopLabel = "",
    string StopOutcome = "",
    string StopDetail = "",
    string ModeName = "");

internal enum StepPlace { Done, Now, Next, Dropped }

internal enum StepWho { Afc, Stock, Pilot }

internal readonly record struct LiveStep(string Text, StepPlace Place, StepWho Who = StepWho.Afc);

/// <summary>
/// The guidance modes the shell knows. The stop order, the page dots, the head phase strip and the
/// availability of a commit read the descriptors. A new mode, such as a gravity turn ascent with its
/// own phases, needs a GuidanceModeId value and one call of GuidanceWindow.RegisterMode with a
/// descriptor from <see cref="Registered"/> and its game-side hooks (live check, phase label, commit,
/// stop, and optionally its LAUNCH method draw and a TUNE > DIAG view). The stop, the commit rules,
/// the head and the page dots then read it with no other change. A mode on LAND also adds its case to
/// ConsolePages.ModeIdFor, which picks the mode of the page from the WAY row.
/// </summary>
internal static class GuidanceModes
{
    public static readonly string[] AscentPhases =
        { "ARMED", "PLANNING", "VERTICAL", "SIMPLE TURN", "CALCULATED CLIMB", "UPFG", "FINAL SECONDS", "CUTOFF" };

    private static readonly string[] PlanClimbPhases = { "ARMED", "PLANNING", "CALCULATED CLIMB", "UPFG", "FINAL SECONDS", "CUTOFF" };
    private static readonly string[] PitchClimbPhases = { "ARMED", "VERTICAL", "SIMPLE TURN", "UPFG", "FINAL SECONDS", "CUTOFF" };

    public static IReadOnlyList<string> AscentPhasesFor(bool planFlies) => planFlies ? PlanClimbPhases : PitchClimbPhases;

    public static readonly string[] BoostbackPhases = { "SETTLE", "TURN", "BURN", "GLIDE", "ENDED" };

    private static readonly string[] BoostbackToEntryPhases = { "SETTLE", "TURN", "BURN", "GLIDE", "ENTRY" };

    public static IReadOnlyList<string> BoostbackPhasesFor(bool entryFollows) => entryFollows ? BoostbackToEntryPhases : BoostbackPhases;

    public static readonly string[] DeorbitPhases =
    {
        "PLANNING", "MAKING THE BURN", "CHECKING THE TIME", "DEORBIT BURN", "PLANNING BRAKING",
        "COAST", "COAST TO BRAKING", "READY TO BRAKE", "BRAKING BURN", "G-FOLD", "HOVER", "DONE",
    };

    public const string WaitStep = "WAIT";

    /// <summary>The step that stock AUTO flies, which has the STOCK chip in the chain.</summary>
    public const string DeorbitBurnStep = "DEORBIT BURN";

    private const string CoastStep = "COAST";

    public static IReadOnlyList<string> DeorbitStepsFor(bool stockBurn, string method, bool hoverFlies)
    {
        var steps = new List<string>(7) { "PLANNING" };
        if (stockBurn)
            steps.AddRange([WaitStep, DeorbitBurnStep]);
        steps.AddRange([CoastStep, "BRAKING BURN", method]);
        if (hoverFlies && method == "G-FOLD")
            steps.Add("HOVER");
        return steps;
    }

    public static string DeorbitStepOf(string phaseLabel, bool burnWaits) => phaseLabel switch
    {
        "MAKING THE BURN" or "CHECKING THE TIME" => "PLANNING",
        DeorbitBurnStep => burnWaits ? WaitStep : DeorbitBurnStep,
        "PLANNING BRAKING" or "COAST TO BRAKING" or "READY TO BRAKE" => CoastStep,
        _ => phaseLabel,
    };

    public static readonly string[] GfoldPhases = { "G-FOLD", "HOVER", "DONE" };

    // At the vertical gate a craft that cannot hover burns to the ground (TryStartTerminalBurn), and with no gate it flies its plan there.
    private static readonly string[] GfoldNoHoverPhases = { "G-FOLD", "LANDING BURN", "DONE" };

    private static readonly string[] GfoldCoastPhases = { "G-FOLD", "COAST TO LANDING BURN", "LANDING BURN", "DONE" };

    public static IReadOnlyList<string> GfoldPhasesFor(bool hoverFlies, bool coastNow = false)
        => coastNow ? GfoldCoastPhases : hoverFlies ? GfoldPhases : GfoldNoHoverPhases;

    private static readonly string[] RcsAlignPhases = { "WAITING", "ALIGNING", "FIRING" };
    private static readonly string[] RcsHoldPhases = { "WAITING", "FIRING" };

    public static IReadOnlyList<string> RcsPhasesFor(bool aligns) => aligns ? RcsAlignPhases : RcsHoldPhases;

    public static readonly string[] SixDofPhases = { "STARTING", "PLANNING", "TOUCHDOWN" };
    public static readonly string[] HoverPhases = { "HOVER", "DONE" };

    /// <summary>The gravity turn ascent slot. It lists its own phases once its mode is in, and stays unavailable until then.</summary>
    public const string GravityTurnUnavailable = "Available with the gravity turn mode";

    private static readonly List<GuidanceModeDescriptor> _modes = BuiltIns();

    private static List<GuidanceModeDescriptor> BuiltIns() => new()
    {
        // While the G-FOLD landing machine flies, the landing stop ends the descent, also when touchdown guidance is still engaged beside it.
        new(GuidanceModeId.SixDofHere, ConsolePage.Land, "TOUCHDOWN GUIDANCE", 10,
            a => a.SixDofEngaged && !a.LandingMachineLive, _ => StopTarget.SixDof, SixDofPhases),
        new(GuidanceModeId.Hover, ConsolePage.Land, "HOVER", 20,
            a => a.HoverLive, _ => StopTarget.Hover, HoverPhases),
        new(GuidanceModeId.GfoldHere, ConsolePage.Land, "G-FOLD", 30,
            a => a.LandingMachineLive, _ => StopTarget.Landing, GfoldPhases),
        new(GuidanceModeId.DeorbitLand, ConsolePage.Land, "DEORBIT AND LAND", 40,
            a => a.DescentLive, _ => StopTarget.Landing, DeorbitPhases),
        new(GuidanceModeId.Boostback, ConsolePage.Land, "BOOSTBACK", 50,
            a => a.BoostbackLive, _ => StopTarget.Boostback, BoostbackPhases),
        new(GuidanceModeId.AscentUpfg, ConsolePage.Launch, "CAT-S / UPFG", 60,
            a => a.AscentRunning || a.LaunchArmed || a.ConvexLaunchPending,
            a => a.AscentRunning ? StopTarget.Ascent : StopTarget.LaunchArmed, AscentPhases),
        new(GuidanceModeId.AscentGravityTurn, ConsolePage.Launch, "GRAVITY TURN", 70,
            _ => false, _ => StopTarget.Ascent, Array.Empty<string>(),
            Available: false, UnavailableReason: GravityTurnUnavailable),
    };

    private static GuidanceModeDescriptor[]? _byStopOrder;

    /// <summary>The modes in the order the one stop checks them, the mode closest to the ground first.</summary>
    public static GuidanceModeDescriptor[] ByStopOrder
    {
        get
        {
            if (_byStopOrder != null)
                return _byStopOrder;
            var sorted = _modes.ToArray();
            Array.Sort(sorted, (x, y) => x.StopOrder.CompareTo(y.StopOrder));
            return _byStopOrder = sorted;
        }
    }

    public static GuidanceModeDescriptor? Of(GuidanceModeId id)
    {
        foreach (GuidanceModeDescriptor mode in _modes)
        {
            if (mode.Id == id)
                return mode;
        }
        return null;
    }

    /// <summary>Adds or replaces a mode, for a mode that arrives later than the built-in ones.</summary>
    public static void Register(GuidanceModeDescriptor mode)
    {
        _modes.RemoveAll(m => m.Id == mode.Id);
        _modes.Add(mode);
        _byStopOrder = null;
    }

    /// <summary>
    /// The descriptor of a mode that registers later than the built-in ones. It is live while
    /// CraftActivity.RegisteredModeLive names it, and its stop is <see cref="StopTarget.Registered"/>
    /// with the label and outcome given here. A stop order below 60 puts it ahead of the ascent.
    /// </summary>
    public static GuidanceModeDescriptor Registered(
        GuidanceModeId id, ConsolePage page, string methodLabel, int stopOrder, IReadOnlyList<string> phases,
        string commitVerb, string stopLabel, string stopOutcome, string stopDetail, string modeName = "")
        => new(id, page, methodLabel, stopOrder,
            a => a.RegisteredModeLive == id, _ => StopTarget.Registered, phases,
            CommitVerb: commitVerb, StopLabel: stopLabel, StopOutcome: stopOutcome, StopDetail: stopDetail,
            ModeName: modeName);

    /// <summary>Back to the built-in modes, for the mod unload.</summary>
    public static void ResetToBuiltIns()
    {
        _modes.Clear();
        _modes.AddRange(BuiltIns());
        _byStopOrder = null;
    }

    /// <summary>The live mode of a craft, or null. The first live mode in stop order wins.</summary>
    public static GuidanceModeDescriptor? Live(in CraftActivity a)
    {
        foreach (GuidanceModeDescriptor mode in ByStopOrder)
        {
            if (mode.IsLive(a))
                return mode;
        }
        return null;
    }

    /// <summary>Whether any live mode lives on <paramref name="page"/>, for the dot on the page chooser.</summary>
    public static bool AnyLiveOn(ConsolePage page, in CraftActivity a)
    {
        foreach (GuidanceModeDescriptor mode in _modes)
        {
            if (mode.Page == page && mode.IsLive(a))
                return true;
        }
        return false;
    }

    public const string StepArrow = " > ";

    public const string DroppedMark = "..";

    public static List<LiveStep> LiveChain(IReadOnlyList<string> phases, string current, Func<string, float> measure, float room, bool pilotNow = false, float chipRoom = 0f)
    {
        var now = new LiveStep("[ " + current + " ]", StepPlace.Now, pilotNow ? StepWho.Pilot : WhoDoes(current));
        var steps = new List<LiveStep>(5) { now };
        int i = phases.Count == 0 ? -1 : PhaseIndex(phases, current);
        if (i < 0)
            return steps;
        foreach ((bool before, bool after) in new[] { (true, true), (false, true), (false, false) })
        {
            steps.Clear();
            AddChainSteps(steps, phases, i, now, before, after);
            if (ChainWidth(steps, measure, chipRoom) <= room)
                break;
        }
        return steps;
    }

    private static void AddChainSteps(List<LiveStep> steps, IReadOnlyList<string> phases, int index, LiveStep now, bool withBefore, bool withAfter)
    {
        bool before = withBefore && index > 0;
        bool after = withAfter && index + 1 < phases.Count;
        int hiddenBefore = index - (before ? 1 : 0);
        int hiddenAfter = phases.Count - index - 1 - (after ? 1 : 0);
        if (hiddenBefore > 0)
            steps.Add(new LiveStep(DroppedMark, StepPlace.Dropped));
        if (before)
            steps.Add(new LiveStep(phases[index - 1], StepPlace.Done, WhoDoes(phases[index - 1])));
        steps.Add(now);
        if (after)
            steps.Add(new LiveStep(phases[index + 1], StepPlace.Next, WhoDoes(phases[index + 1])));
        if (hiddenAfter > 0)
            steps.Add(new LiveStep(DroppedMark, StepPlace.Dropped));
    }

    private static StepWho WhoDoes(string step)
        => step is WaitStep or DeorbitBurnStep ? StepWho.Stock : StepWho.Afc;

    public static bool HasStockChip(in LiveStep step)
        => step.Text.Contains(DeorbitBurnStep, StringComparison.Ordinal);

    private static float ChainWidth(List<LiveStep> steps, Func<string, float> measure, float chipRoom)
    {
        float width = 0f;
        for (int i = 0; i < steps.Count; i++)
        {
            width += measure(steps[i].Text) + (HasStockChip(steps[i]) ? chipRoom : 0f);
            if (i > 0)
                width += measure(JoinBefore(steps, i));
        }
        return width;
    }

    public static string JoinBefore(IReadOnlyList<LiveStep> steps, int i)
        => steps[i].Place == StepPlace.Dropped || steps[i - 1].Place == StepPlace.Dropped ? " " : StepArrow;

    public static string PhaseChain(IReadOnlyList<string> phases, string current, Func<string, float> measure, float room)
    {
        List<LiveStep> steps = LiveChain(phases, current, measure, room);
        string line = "";
        for (int i = 0; i < steps.Count; i++)
            line += (i > 0 ? JoinBefore(steps, i) : "") + steps[i].Text;
        return line;
    }

    /// <summary>
    /// The index of <paramref name="current"/> in the phase list, or -1. A phase with a note in
    /// parentheses, such as "UPFG (BLENDING)", is the list entry before the note.
    /// </summary>
    public static int PhaseIndex(IReadOnlyList<string> phases, string current)
    {
        int paren = current.IndexOf(" (", StringComparison.Ordinal);
        ReadOnlySpan<char> name = paren > 0 ? current.AsSpan(0, paren) : current.AsSpan();
        for (int k = 0; k < phases.Count; k++)
        {
            if (name.Equals(phases[k].AsSpan(), StringComparison.OrdinalIgnoreCase))
                return k;
        }
        return -1;
    }
}

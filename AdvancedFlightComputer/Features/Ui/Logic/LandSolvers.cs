using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>The solver of a powered landing, in the order a landing flies them: entry guidance, then touchdown guidance. G-FOLD is the older solver, not the next step of that chain.</summary>
internal enum LandSolver { Entry, Touchdown, Gfold }

/// <summary>The guidance mode that flies the landing of a craft now, for the console to follow a hand-over.</summary>
internal enum LiveLanding { None, Boostback, Entry, Touchdown, Gfold, Hover }

/// <summary>
/// The solver choice of the LAND page as plain rules. Two switches of the craft hold it:
/// UseThreeDofLanding (entry guidance) and UseSixDofLanding (touchdown guidance, else G-FOLD). The
/// braking burn of DEORBIT + LAND reads only UseSixDofLanding, so a pick of ENTRY leaves it as it
/// is, and DEORBIT + LAND offers only TOUCHDOWN and G-FOLD.
/// </summary>
internal static class LandSolvers
{
    public static readonly string[] Labels = { "ENTRY", "TOUCHDOWN", "G-FOLD" };

    public static readonly string[] TableHeads = { "MODE", "USE", "END" };

    public const string HeldDeorbitEntry = "DEORBIT + LAND hands its braking burn to TOUCHDOWN or G-FOLD.";
    public const string HeldWhileDeorbit = "Held during a deorbit. Both change its braking gate.";
    public const string HeldWhileGfold = "Held during the G-FOLD descent. TOUCHDOWN does not take a live descent over from this page.";

    public const string HeldNote = "HELD";

    public const string GfoldHoldsRow = "TOUCHDOWN HELD: G-FOLD ACTIVE.";
    public const string EntryTakesOver = " ENTRY can take it over.";

    public static string Label(LandSolver solver) => solver switch
    {
        LandSolver.Entry => "ENTRY",
        LandSolver.Touchdown => "TOUCHDOWN",
        _ => "G-FOLD",
    };

    public static string How(LandSolver solver) => solver switch
    {
        LandSolver.Entry => "GLIDE + BURN (ENTRY)",
        LandSolver.Touchdown => "POWERED (TOUCHDOWN)",
        _ => "MIN FUEL (G-FOLD)",
    };

    public static string Suits(LandSolver solver) => solver switch
    {
        LandSolver.Entry => "IN AIR",
        LandSolver.Touchdown => "HEAVY/AIR",
        _ => "NO AIR/RCS",
    };

    public static string Ends(LandSolver solver, double hoverHandoffM, bool hoverFlies = true, bool hasAir = true) => solver switch
    {
        LandSolver.Gfold when hoverFlies => double.IsFinite(hoverHandoffM) ? string.Create(Fmt.Inv, $"HOVER FROM {hoverHandoffM:F0} m") : "HOVER",
        LandSolver.Entry when !hasAir => LandChain.Impact,
        _ => "LANDED",
    };

    public static string[] TableRow(LandSolver solver, double hoverHandoffM, bool hoverFlies = true, bool hasAir = true, string note = "")
        => new[] { How(solver), note.Length > 0 ? note : Suits(solver), Ends(solver, hoverHandoffM, hoverFlies, hasAir) };

    /// <summary>Above the weight at the minimum throttle StartTerminalHover refuses the craft, and G-FOLD flies on to the ground.</summary>
    public static bool HoverFlies(in DescentFacts f) => f.Known && HoverFlies(f.TwrAtCraftMin);

    public static bool HoverFlies(double twrAtCraftMin) => twrAtCraftMin <= 1.0;

    public static string SuitsSentence(LandSolver solver) => solver switch
    {
        LandSolver.Entry => "A glide through air with the engine off, then the landing burn.",
        LandSolver.Touchdown => "Heavy boosters and descents through air. It models the rotation of the craft.",
        _ => "Airless bodies and landers that point with RCS.",
    };

    public static RowNote NoteOf(LandSolver solver, in LandNowFacts f)
    {
        bool held = f.Way == LandWay.FromOrbit ? f.DeorbitFlies && LockedByDeorbit(solver) : f.GfoldFlies && solver == LandSolver.Touchdown;
        if (held)
            return new RowNote(HeldNote);
        switch (solver)
        {
            case LandSolver.Entry when !f.HasAir:
                return new RowNote("NEEDS AIR", Stops: true);
            case LandSolver.Gfold when f.HasAir:
                return new RowNote(f.Body.Length > 0 ? f.Body + " HAS AIR" : "HAS AIR", Stops: true);
            case LandSolver.Touchdown when f.Descent.Known && f.Descent.NoGimbal:
                return new RowNote("NO GIMBAL", Stops: true);
            case LandSolver.Touchdown when f.Descent.Known && f.Descent.OverPowered:
                return new RowNote("FLOOR TOO HIGH", Stops: true);
            case LandSolver.Touchdown when f.Descent.Known && f.Descent.TwrAtFloor > 1.0:
                return new RowNote("TWR >1 AT FLOOR");
            default:
                return RowNote.None;
        }
    }

    public static string HeldLine(in LandNowFacts f)
        => f.Way != LandWay.FromOrbit && f.GfoldFlies ? GfoldHoldsRow : "";

    public const string TouchdownFallback = "If TOUCHDOWN cannot solve at the handover, G-FOLD flies the descent.";

    /// <summary>The braking burn of DEORBIT + LAND hands a craft that touchdown guidance refuses to G-FOLD (GuidanceWindow.StepLanding), so the DEORBIT table says so while TOUCHDOWN is chosen.</summary>
    public static string FallbackLine(in LandNowFacts f)
        => f.Way == LandWay.FromOrbit && f.Chosen == LandSolver.Touchdown ? TouchdownFallback : "";

    public static void MarksOf(LandSolver solver, int row, bool chosen, in RowNote note, bool hasAir, List<TableMark> marks)
    {
        if (solver == LandSolver.Entry && !hasAir)
            marks.Add(new TableMark(row, 2, chosen ? TableInk.DangerOnBand : TableInk.DangerDim));
        if (note.Stops)
            marks.Add(new TableMark(row, 1, chosen ? TableInk.Warn : TableInk.WarnDim));
        else if (note.Text.Length > 0 && note.Text != HeldNote)
            marks.Add(new TableMark(row, 1, chosen ? TableInk.WarnOnBand : TableInk.WarnDim));
    }

    public static TableMark MethodMark(int row, bool cannotFly, bool heldOther) => new(row, 0, cannotFly || heldOther ? TableInk.MethodDim : TableInk.Method);

    public static bool ChainCannotFly(LandWay way, LandSolver solver, bool hasAir, CraftSituation where, bool noStageHome, bool belowBrakingStart = false, bool flies = false)
    {
        if (LandWays.HasSolver(way) && RowIsDim(solver, hasAir, false))
            return true;
        if (where == CraftSituation.Ground)
            return way == LandWay.Boostback && noStageHome;
        return !flies && SituationVerbs.WayNow(way, where, hasAir, belowBrakingStart).Length > 0;
    }

    public static string Description(LandSolver solver) => solver switch
    {
        LandSolver.Entry => "Flies the glide with the engine off and lights it at the planned ignition.",
        LandSolver.Touchdown => "Plans and flies a powered descent to the ground.",
        _ => "Minimum-fuel powered descent.",
    };

    public static string Card(LandSolver solver, double hoverHandoffM, bool handToTouchdown, double touchdownLastS, bool hoverFlies = true, bool hasAir = true, bool hoverKnown = true)
    {
        var inv = Fmt.Inv;
        string ends = solver switch
        {
            LandSolver.Entry when !hasAir => "IMPACT (NO AIR, NO PLAN)",
            LandSolver.Gfold when !hoverFlies && !hoverKnown => "LANDED (NO LIT ENGINE TO JUDGE HOVER)",
            LandSolver.Gfold when !hoverFlies => "LANDED (THRUST AT MIN THROTTLE ABOVE WEIGHT)",
            LandSolver.Entry => handToTouchdown && double.IsFinite(touchdownLastS)
                ? string.Create(inv, $"LANDED, OR TOUCHDOWN FOR LAST {touchdownLastS:F0} s")
                : "LANDED",
            LandSolver.Touchdown => "LANDED",
            _ => double.IsFinite(hoverHandoffM) ? string.Create(inv, $"HOVER FROM {hoverHandoffM:F0} m") : "HOVER",
        };
        string air = solver switch
        {
            LandSolver.Entry => "required",
            LandSolver.Touchdown => "either",
            _ => "not allowed",
        };
        return Description(solver) + "\n\nUse: " + SuitsSentence(solver) + "\nAir: " + air + "\nEnd: " + ends;
    }

    /// <summary>Why each row of the table is held, in enum order, empty for a row that can be picked.</summary>
    public static string[] Held(LandWay way, bool deorbitLocked, bool descentLive, bool hasAir)
    {
        var held = new string[Labels.Length];
        for (int i = 0; i < held.Length; i++)
            held[i] = "";
        if (way == LandWay.FromOrbit)
            held[(int)LandSolver.Entry] = HeldDeorbitEntry;
        if (deorbitLocked)
        {
            held[(int)LandSolver.Touchdown] = HeldWhileDeorbit;
            held[(int)LandSolver.Gfold] = HeldWhileDeorbit;
        }
        else if (way != LandWay.FromOrbit && descentLive)
            held[(int)LandSolver.Touchdown] = hasAir ? HeldWhileGfold + EntryTakesOver : HeldWhileGfold;
        return held;
    }

    public static LandSolver[] RowsOf(LandWay way)
        => way == LandWay.FromOrbit ? new[] { LandSolver.Touchdown, LandSolver.Gfold } : new[] { LandSolver.Entry, LandSolver.Touchdown, LandSolver.Gfold };

    public static bool RowIsDim(LandSolver solver, bool hasAir, bool held)
        => held || (solver == LandSolver.Entry && !hasAir) || (solver == LandSolver.Gfold && hasAir);

    /// <summary>The solver the two switches choose.</summary>
    public static LandSolver Chosen(bool useEntry, bool useTouchdown)
        => useEntry ? LandSolver.Entry : useTouchdown ? LandSolver.Touchdown : LandSolver.Gfold;

    /// <summary>The two switches after a pick. ENTRY keeps the touchdown switch, because the braking burn reads it.</summary>
    public static (bool UseEntry, bool UseTouchdown) Apply(LandSolver picked, bool useTouchdown) => picked switch
    {
        LandSolver.Entry => (true, useTouchdown),
        LandSolver.Touchdown => (false, true),
        _ => (false, false),
    };

    /// <summary>
    /// TOUCHDOWN and G-FOLD change the braking gate of a committed deorbit, so they are held while one
    /// flies. ENTRY is not: entry guidance engages from a boostback glide or from its own commit.
    /// </summary>
    public static bool LockedByDeorbit(LandSolver solver) => solver != LandSolver.Entry;

    /// <summary>The landing mode that flies now, the one closest to the ground first.</summary>
    public static LiveLanding Live(bool touchdownActive, bool entryLive, bool hoverLive, bool landingMachineLive, bool boostbackLive)
        => touchdownActive ? LiveLanding.Touchdown
            : entryLive ? LiveLanding.Entry
            : hoverLive ? LiveLanding.Hover
            : landingMachineLive ? LiveLanding.Gfold
            : boostbackLive ? LiveLanding.Boostback
            : LiveLanding.None;

    /// <summary>The solver choice that matches a flying mode, so the page, its commit and the stop act on it. Null leaves the choice alone.</summary>
    public static LandSolver? SolverFor(LiveLanding live) => live switch
    {
        LiveLanding.Entry => LandSolver.Entry,
        LiveLanding.Touchdown => LandSolver.Touchdown,
        LiveLanding.Gfold => LandSolver.Gfold,
        _ => null,
    };

    /// <summary>
    /// The cue for a change of the flying mode, or null. A G-FOLD descent and the hover raise their own
    /// cues from the landing step, and the head cues a boostback when it shows a craft that flies one
    /// (ConsoleHead.BuildLines), so only entry and touchdown guidance get a cue here.
    /// </summary>
    public static StatusLine? CueFor(LiveLanding previous, LiveLanding live)
    {
        if (live == previous)
            return null;
        string fromHere = ConsolePages.CueTarget(ConsolePage.Land, ConsolePages.EnumKey(LandWay.FromHere));
        return live switch
        {
            LiveLanding.Entry => new StatusLine(Severity.Cue, "AFC: ENTRY", fromHere),
            LiveLanding.Touchdown => new StatusLine(Severity.Cue, previous == LiveLanding.Entry
                ? "HANDOVER: TOUCHDOWN"
                : "AFC: TOUCHDOWN", fromHere),
            _ => null,
        };
    }
}

/// <summary>Where entry guidance is when the stop is pressed.</summary>
internal enum EntryStage { Requested, Glide, Burn }

/// <summary>
/// The shell rules of entry guidance, a mode that registers through GuidanceWindow.RegisterMode: its
/// descriptor, and the stop texts, which follow what GuidanceWindow.Disengage3Dof does.
/// </summary>
internal static class EntryGuidance
{
    public const string MethodLabel = "ENTRY GUIDANCE";
    public const string CommitVerb = "ENTRY";
    public const string StopLabel = "STOP ENTRY";

    /// <summary>Ahead of the boostback (50), which a glide hands to entry guidance, and behind touchdown guidance (10), which takes its burn.</summary>
    public const int StopOrder = 15;

    public static readonly string[] Phases = { "START", "GLIDE", "LANDING BURN", "TOUCHDOWN" };

    public static GuidanceModeDescriptor Descriptor()
        => GuidanceModes.Registered(GuidanceModeId.EntryHere, ConsolePage.Land, MethodLabel, StopOrder, Phases,
            CommitVerb, StopLabel, FallbackOutcome, FallbackDetail, "ENTRY GUIDANCE");

    // The descriptor texts are read only when the stage texts of StopText are missing, so they hold for every stage.
    private const string FallbackOutcome = "ENGINE LIT IN BURN, ELSE CUT";
    private const string FallbackDetail =
        "Ends entry guidance. In the landing burn the engine stays lit. In the glide it is cut.";

    public static (string Outcome, string Detail) StopText(EntryStage stage, bool controlAcquired)
    {
        if (stage == EntryStage.Requested)
            return ("NOTHING STARTED, ENGINE UNCHANGED",
                "Cancels the entry guidance request before it starts. The engine is not touched.");
        if (!controlAcquired)
            return ("ENGINE UNCHANGED",
                "Ends entry guidance. It holds no control yet. The engine is not touched.");
        return stage == EntryStage.Burn
            ? ("ENGINE LIT", "Engine stays lit at the last throttle. AFC stops steering.")
            : ("ENGINE OFF", "Engine stays off. AFC stops steering.");
    }

    public const string CommitHint =
        "Flies the glide with the engine off and lights it at the planned ignition. With TOUCHDOWN FOR BURN END on, TOUCHDOWN flies the last seconds of the burn once its standby plan is ready. "
        + "A lit engine goes off at start. Without air it gets no plan and holds the engine off.";

    public const string NoAirText =
        "NO AIR: NO PLAN, ENGINE OFF. STOP, THEN USE TOUCHDOWN OR G-FOLD.";
}

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
/// braking burn of DEORBIT AND LAND reads only UseSixDofLanding, so a pick of ENTRY leaves it as it
/// is, and DEORBIT AND LAND offers only TOUCHDOWN and G-FOLD.
/// </summary>
internal static class LandSolvers
{
    public static readonly string[] Labels = { "ENTRY", "TOUCHDOWN", "G-FOLD" };

    public static readonly string[] TableHeads = { "HOW IT LANDS", "SUITS", "ENDS" };

    public const string HeldDeorbitEntry = "DEORBIT AND LAND hands its braking burn to TOUCHDOWN or G-FOLD.";
    public const string HeldWhileDeorbit = "Held while a deorbit flies, because they change its braking gate.";
    public const string HeldWhileGfold = "Held while the G-FOLD descent flies, because touchdown guidance does not take a live descent over from this page.";

    public const string HeldNote = "held";

    public const string GfoldHoldsRow = "G-FOLD flies, so TOUCHDOWN is held.";
    public const string EntryTakesOver = " ENTRY can take it over.";

    public static string Label(LandSolver solver) => solver switch
    {
        LandSolver.Entry => "ENTRY",
        LandSolver.Touchdown => "TOUCHDOWN",
        _ => "G-FOLD",
    };

    public static string How(LandSolver solver) => solver switch
    {
        LandSolver.Entry => "Glide, burn (ENTRY)",
        LandSolver.Touchdown => "Powered (TOUCHDOWN)",
        _ => "Fuel-best (G-FOLD)",
    };

    public static string Suits(LandSolver solver) => solver switch
    {
        LandSolver.Entry => "through air",
        LandSolver.Touchdown => "heavy, or in air",
        _ => "airless, agile",
    };

    public static string Ends(LandSolver solver, double hoverHandoffM, bool hoverFlies = true, bool hasAir = true) => solver switch
    {
        LandSolver.Gfold when hoverFlies => double.IsFinite(hoverHandoffM) ? string.Create(Fmt.Inv, $"at {hoverHandoffM:F0} m, hover") : "then hover",
        LandSolver.Entry when !hasAir => LandChain.Impact,
        _ => "on the ground",
    };

    public static string[] TableRow(LandSolver solver, double hoverHandoffM, bool hoverFlies = true, bool hasAir = true, string note = "")
        => new[] { How(solver), note.Length > 0 ? note : Suits(solver), Ends(solver, hoverHandoffM, hoverFlies, hasAir) };

    /// <summary>Above the weight at the minimum throttle StartTerminalHover refuses the craft, and G-FOLD flies on to the ground.</summary>
    public static bool HoverFlies(in DescentFacts f) => f.Known && HoverFlies(f.TwrAtCraftMin);

    public static bool HoverFlies(double twrAtCraftMin) => twrAtCraftMin <= 1.0;

    public static string SuitsSentence(LandSolver solver) => solver switch
    {
        LandSolver.Entry => "A glide through air with the engine off, then the landing burn.",
        LandSolver.Touchdown => "Heavy boosters and descents through air, because it models the rotation of the craft.",
        _ => "Airless bodies and agile landers that point with RCS.",
    };

    public static RowNote NoteOf(LandSolver solver, in LandNowFacts f)
    {
        bool held = f.Way == LandWay.FromOrbit ? f.DeorbitFlies && LockedByDeorbit(solver) : f.GfoldFlies && solver == LandSolver.Touchdown;
        if (held)
            return new RowNote(HeldNote);
        switch (solver)
        {
            case LandSolver.Entry when !f.HasAir:
                return new RowNote("no air here", Stops: true);
            case LandSolver.Gfold when f.HasAir:
                return new RowNote(f.Body.Length > 0 ? f.Body + " has air" : "air here", Stops: true);
            case LandSolver.Touchdown when f.Descent.Known && f.Descent.NoGimbal:
                return new RowNote("no gimbal", Stops: true);
            case LandSolver.Touchdown when f.Descent.Known && f.Descent.OverPowered:
                return new RowNote("floor too high", Stops: true);
            case LandSolver.Touchdown when f.Descent.Known && f.Descent.TwrAtFloor > 1.0:
                return new RowNote("path curves");
            default:
                return RowNote.None;
        }
    }

    public static string HeldLine(in LandNowFacts f)
        => f.Way != LandWay.FromOrbit && f.GfoldFlies ? GfoldHoldsRow : "";

    public const string TouchdownFallback = "If TOUCHDOWN cannot plan for this craft at the handoff, G-FOLD flies the descent and stays the choice for this craft.";

    /// <summary>The braking burn of DEORBIT AND LAND hands a craft that touchdown guidance refuses to G-FOLD (GuidanceWindow.StepLanding), so the DEORBIT table says so while TOUCHDOWN is chosen.</summary>
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
        _ => "A fuel-optimal powered descent.",
    };

    public static string Card(LandSolver solver, double hoverHandoffM, bool handToTouchdown, double touchdownLastS, bool hoverFlies = true, bool hasAir = true, bool hoverKnown = true)
    {
        var inv = Fmt.Inv;
        string ends = solver switch
        {
            LandSolver.Entry when !hasAir => "in an impact, because with no air it gets no plan",
            LandSolver.Gfold when !hoverFlies && !hoverKnown => "on the ground, with no lit engine to judge a hover",
            LandSolver.Gfold when !hoverFlies => "on the ground, because the thrust at minimum throttle is above the weight",
            LandSolver.Entry => handToTouchdown && double.IsFinite(touchdownLastS)
                ? string.Create(inv, $"on the ground, or TOUCHDOWN for the last {touchdownLastS:F0} s")
                : "on the ground",
            LandSolver.Touchdown => "on the ground",
            _ => double.IsFinite(hoverHandoffM) ? string.Create(inv, $"at {hoverHandoffM:F0} m, then hover") : "then hover",
        };
        string air = solver switch
        {
            LandSolver.Entry => "must have it",
            LandSolver.Touchdown => "with or without",
            _ => "not allowed",
        };
        return Description(solver) + "\n\nSuits: " + SuitsSentence(solver) + "\nAir: " + air + "\nEnds: " + ends;
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
            LiveLanding.Entry => new StatusLine(Severity.Cue, "Entry guidance flies this craft.", fromHere),
            LiveLanding.Touchdown => new StatusLine(Severity.Cue, previous == LiveLanding.Entry
                ? "Touchdown took over from entry."
                : "Touchdown guidance flies this craft.", fromHere),
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
    public const string CommitVerb = "START ENTRY";
    public const string StopLabel = "STOP ENTRY";

    /// <summary>Ahead of the boostback (50), which a glide hands to entry guidance, and behind touchdown guidance (10), which takes its burn.</summary>
    public const int StopOrder = 15;

    public static readonly string[] Phases = { "STARTING", "GLIDE", "LANDING BURN", "TOUCHDOWN" };

    public static GuidanceModeDescriptor Descriptor()
        => GuidanceModes.Registered(GuidanceModeId.EntryHere, ConsolePage.Land, MethodLabel, StopOrder, Phases,
            CommitVerb, StopLabel, FallbackOutcome, FallbackDetail, "Entry guidance");

    // The descriptor texts are read only when the stage texts of StopText are missing, so they hold for every stage.
    private const string FallbackOutcome = "Engine stays in a burn, else cut";
    private const string FallbackDetail =
        "Entry guidance ends. In the landing burn the engine stays lit, because a cut in flight drops the craft. In the glide the engine is cut.";

    public static (string Outcome, string Detail) StopText(EntryStage stage, bool controlAcquired)
    {
        if (stage == EntryStage.Requested)
            return ("Nothing started, engine not touched",
                "Cancels the entry guidance request before it starts. The engine is not touched.");
        if (!controlAcquired)
            return ("Engine not touched",
                "Entry guidance ends. It holds no control of this craft yet, so the engine is not touched.");
        return stage == EntryStage.Burn
            ? ("Engine stays lit", "Engine stays lit at its last throttle. AFC stops steering. You fly.")
            : ("Engine stays off", "Engine stays off. AFC stops steering. You fly.");
    }

    public const string CommitHint =
        "Entry guidance flies the glide with the engine off and lights it at the planned ignition. With TOUCHDOWN ENDS THE BURN on, touchdown guidance takes the last seconds of the burn when its standby plan is ready. "
        + "A lit engine goes off when it starts. It plans with the air: with no air it gets no plan and holds the engine off.";

    public const string NoAirText =
        "Entry guidance: no air here, so it gets no plan and holds the engine off to the ground. Stop it and land with TOUCHDOWN or G-FOLD.";
}

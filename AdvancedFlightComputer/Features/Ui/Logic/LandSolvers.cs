using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>The solver of a powered landing, in the order a landing flies them: entry guidance, then touchdown guidance. The player picks none of them. Touchdown guidance lands every craft, entry guidance flies first where it applies (LandSolvers.EntryApplies), and G-FOLD flies only as the fallback for a craft that touchdown guidance has no plan for.</summary>
internal enum LandSolver { Entry, Touchdown, Gfold }

/// <summary>The guidance mode that flies the landing of a craft now, for the console to follow a hand-over.</summary>
internal enum LiveLanding { None, Boostback, Entry, Touchdown, Gfold, Hover }

/// <summary>
/// The solver choice of the LAND page as plain rules. Two switches of the craft hold it:
/// UseThreeDofLanding (entry guidance) and UseSixDofLanding (touchdown guidance, else G-FOLD). The
/// LAND page sets UseThreeDofLanding from the situation of the craft (EntryApplies). Only the
/// fallback turns UseSixDofLanding off, for a craft that touchdown guidance has no plan for, and
/// the commit of the next landing turns it on again. The braking burn of DEORBIT + LAND reads only
/// UseSixDofLanding, and DEORBIT + LAND shows only TOUCHDOWN.
/// </summary>
internal static class LandSolvers
{
    public static readonly string[] Labels = { EntryMethods.GlideBurnLabel, "TOUCHDOWN", "G-FOLD" };

    public static readonly string[] TableHeads = { "MODE", "USE", "END" };

    public const string HeldDeorbitEntry = "DEORBIT + LAND hands its braking burn to TOUCHDOWN or G-FOLD.";
    public const string HeldWhileDeorbit = "Held during a deorbit. Both change its braking gate.";
    public const string HeldWhileGfold = "Held during the G-FOLD descent. TOUCHDOWN does not take a live descent over from this page.";

    public const string HeldNote = "HELD";

    public const string GfoldHoldsRow = "TOUCHDOWN HELD: G-FOLD ACTIVE.";
    public const string EntryTakesOver = " " + EntryMethods.GlideBurnLabel + " can take it over.";

    public static string Label(LandSolver solver) => solver switch
    {
        LandSolver.Entry => EntryMethods.GlideBurnLabel,
        LandSolver.Touchdown => "TOUCHDOWN",
        _ => "G-FOLD",
    };

    public static string How(LandSolver solver) => solver switch
    {
        LandSolver.Entry => "ENGINE-OFF GLIDE (" + EntryMethods.GlideBurnLabel + ")",
        LandSolver.Touchdown => "POWERED (TOUCHDOWN)",
        _ => "POINT MASS (G-FOLD)",
    };

    public static string Suits(LandSolver solver) => solver switch
    {
        LandSolver.Entry => "IN AIR",
        LandSolver.Touchdown => "ANY BODY",
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
        LandSolver.Touchdown => "Powered descents on any body. It models the rotation of the craft.",
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
            // Neither a gimbal nor RCS that steers stops the landing: touchdown guidance points its thrust with either (Ksa6DofSetup.RcsSteers), and G-FOLD needs a gimbal (GuidanceWindow.GfoldCanFly). G-FOLD flies a craft whose floor is too high, so that note stops nothing.
            case LandSolver.Touchdown when f.Descent.Known && f.Descent.NoSteering:
                return new RowNote("NO GIMBAL OR RCS", Stops: true);
            case LandSolver.Touchdown when f.Descent.Known && f.Descent.OverPowered:
                return new RowNote("FLOOR TOO HIGH");
            case LandSolver.Touchdown when f.Descent.Known && f.Descent.TwrAtFloor > 1.0:
                return new RowNote("TWR >1 AT FLOOR");
            default:
                return RowNote.None;
        }
    }

    public static string HeldLine(in LandNowFacts f)
        => f.Way != LandWay.FromOrbit && f.GfoldFlies ? GfoldHoldsRow : "";

    public const string TouchdownFallback = "If TOUCHDOWN has no plan for this craft, G-FOLD flies the descent where it can: no air, gimballed engine. Else LAND refuses the craft.";

    public const string EntryFirst = EntryMethods.GlideBurnLabel + " flies the fall through air first and hands its burn to TOUCHDOWN.";

    /// <summary>A fall shorter than this goes to touchdown guidance directly, s, because entry guidance needs the time for its glide plan.</summary>
    public const double EntryMinFallS = 60.0;

    /// <summary>
    /// Whether entry guidance flies first: a craft that comes down through air with the engine off and with the time for a glide. An airless body has no glide, a lit engine is a powered descent that touchdown guidance takes as it is, and a short fall leaves no time for the glide plan.
    /// </summary>
    public static bool EntryApplies(bool hasAir, CraftSituation where, bool thrust, double fallS)
        => hasAir && !thrust && (where is CraftSituation.FallingAir or CraftSituation.CoastingUp) && fallS >= EntryMinFallS;

    /// <summary>The braking burn of DEORBIT + LAND (GuidanceWindow.StepBrakingHandover) and touchdown guidance in the air (GuidanceWindow.FallBackToGfold) hand a craft that touchdown guidance has no plan for to G-FOLD when G-FOLD can fly it, so the table says so while TOUCHDOWN is chosen.</summary>
    public static string FallbackLine(in LandNowFacts f)
        => f.Chosen == LandSolver.Touchdown && LandWays.HasSolver(f.Way) ? TouchdownFallback : "";

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
        _ => "Point-mass powered descent, no rotation of the craft. Fallback when TOUCHDOWN cannot plan.",
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
    public static string[] Held(LandWay way, bool deorbitLocked, bool descentLive, bool hasAir, bool entryFirst = false)
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
        else if (way != LandWay.FromOrbit && entryFirst)
            held[(int)LandSolver.Touchdown] = EntryFirst;
        return held;
    }

    /// <summary>The rows of the table: TOUCHDOWN, with ENTRY above it on LAND NOW only where entry guidance flies first, and G-FOLD only while its fallback descent flies, so the page shows what flies the craft.</summary>
    public static LandSolver[] RowsOf(LandWay way, bool gfoldFlies = false, bool entryFirst = false)
    {
        var rows = new List<LandSolver>(3);
        if (way != LandWay.FromOrbit && entryFirst)
            rows.Add(LandSolver.Entry);
        rows.Add(LandSolver.Touchdown);
        if (gfoldFlies)
            rows.Add(LandSolver.Gfold);
        return rows.ToArray();
    }

    public static bool RowIsDim(LandSolver solver, bool hasAir, bool held)
        => held || (solver == LandSolver.Entry && !hasAir) || (solver == LandSolver.Gfold && hasAir);

    /// <summary>The solver the two switches choose.</summary>
    public static LandSolver Chosen(bool useEntry, bool useTouchdown)
        => useEntry ? LandSolver.Entry : useTouchdown ? LandSolver.Touchdown : LandSolver.Gfold;

    /// <summary>The solver the page shows: G-FOLD only while its descent flies, because the player cannot choose it.</summary>
    public static LandSolver Shown(LandSolver chosen, bool gfoldFlies)
        => chosen == LandSolver.Gfold && !gfoldFlies ? LandSolver.Touchdown : chosen;

    /// <summary>The two switches after a pick. ENTRY keeps the touchdown switch, because the braking burn reads it. G-FOLD is not a choice of the player, so a pick of it gives touchdown guidance.</summary>
    public static (bool UseEntry, bool UseTouchdown) Apply(LandSolver picked, bool useTouchdown) => picked switch
    {
        LandSolver.Entry => (true, useTouchdown),
        _ => (false, true),
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
            LiveLanding.Entry => new StatusLine(Severity.Cue, "AFC: " + EntryMethods.GlideBurnLabel, fromHere),
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
    public const string MethodLabel = EntryMethods.GlideBurnLabel;
    public const string CommitVerb = EntryMethods.GlideBurnLabel;
    public const string StopLabel = "STOP " + EntryMethods.GlideBurnLabel;

    /// <summary>Ahead of the boostback (50), which a glide hands to entry guidance, and behind touchdown guidance (10), which takes its burn.</summary>
    public const int StopOrder = 15;

    public static readonly string[] Phases = { "START", "GLIDE", "LANDING BURN", "TOUCHDOWN" };

    public static GuidanceModeDescriptor Descriptor()
        => GuidanceModes.Registered(GuidanceModeId.EntryHere, ConsolePage.Land, MethodLabel, StopOrder, Phases,
            CommitVerb, StopLabel, FallbackOutcome, FallbackDetail, EntryMethods.GlideBurnLabel);

    // The descriptor texts are read only when the stage texts of StopText are missing, so they hold for every stage.
    private const string FallbackOutcome = "ENGINE LIT IN BURN, ELSE CUT";
    private const string FallbackDetail =
        "Ends GLIDE + BURN. In the landing burn the engine stays lit. In the glide it is cut.";

    public static (string Outcome, string Detail) StopText(EntryStage stage, bool controlAcquired)
    {
        if (stage == EntryStage.Requested)
            return ("NOTHING STARTED, ENGINE UNCHANGED",
                "Cancels the GLIDE + BURN request before it starts. The engine is not touched.");
        if (!controlAcquired)
            return ("ENGINE UNCHANGED",
                "Ends GLIDE + BURN. It holds no control yet. The engine is not touched.");
        return stage == EntryStage.Burn
            ? ("ENGINE LIT", "Engine stays lit at the last throttle. AFC stops steering.")
            : ("ENGINE OFF", "Engine stays off. AFC stops steering.");
    }

    public const string CommitHint =
        "Flies the glide with the engine off and lights it at the planned ignition. With TOUCHDOWN FOR BURN END on, TOUCHDOWN flies the last seconds of the burn once its standby plan is ready. "
        + "A lit engine goes off at start. Without air it gets no plan and holds the engine off.";

    public const string NoAirText =
        "NO AIR: NO PLAN, ENGINE OFF. STOP, THEN USE TOUCHDOWN.";
}

/// <summary>The METHOD row of LAND > ENTRY, as plain rules: ORBITAL ENTRY from the top of the air, or GLIDE + BURN (the 3-DOF) from wherever the craft falls through air.</summary>
internal static class EntryMethods
{
    public const string OrbitalLabel = "ORBITAL ENTRY";

    /// <summary>The 3-DOF's name in the console: its glide with the engine off, then its landing burn.</summary>
    public const string GlideBurnLabel = "GLIDE + BURN";

    public static readonly string[] Labels = { OrbitalLabel, GlideBurnLabel };

    public static string Label(EntryMethod method) => method == EntryMethod.GlideBurn ? GlideBurnLabel : OrbitalLabel;

    public static string Card(EntryMethod method) => method == EntryMethod.GlideBurn
        ? "From the current state through air: glides with the engine off, then flies the landing burn to the site. Starts from any fall through air, also a reloaded one."
        : "From orbit or the top of the air: flies the entry belly first, banking to hold its planned drag, and hands over to BELLYFLOP to fall onto the site, and TOUCHDOWN for the flip and the landing burn. Engaged below the hand-over, it starts at BELLYFLOP.";

    public static readonly string[] Cards = { Card(EntryMethod.Orbital), Card(EntryMethod.GlideBurn) };

    /// <summary>The method of a mode that flies, so the row follows it; null for any other mode.</summary>
    public static EntryMethod? Of(GuidanceModeId? live) => live switch
    {
        GuidanceModeId.OrbitalEntry => EntryMethod.Orbital,
        GuidanceModeId.EntryHere => EntryMethod.GlideBurn,
        _ => null,
    };
}

/// <summary>Where orbital entry guidance is when the stop is pressed.</summary>
internal enum OrbitalEntryStage { Requested, Sweeping, Entry, Holding, Bellyflop }

/// <summary>
/// The shell rules of orbital entry guidance (Modes/OrbitalEntry.cs, with bellyflop as its last phase), a mode that registers through GuidanceWindow.RegisterMode: its descriptor and its stop texts, which follow what GuidanceWindow.StopOrbitalEntry does.
/// </summary>
internal static class OrbitalEntryGuidance
{
    public const string MethodLabel = EntryMethods.OrbitalLabel;
    public const string CommitVerb = EntryMethods.OrbitalLabel;
    public const string StopLabel = "STOP ORBITAL ENTRY";

    /// <summary>Ahead of the deorbit (40), whose entry burn hands the craft to it, and behind GLIDE + BURN (15) and touchdown guidance (10), which fly lower.</summary>
    public const int StopOrder = 35;

    public static readonly string[] Phases = { "SWEEP", "ENTRY", "HOLD", "BELLYFLOP" };

    public static GuidanceModeDescriptor Descriptor()
        => GuidanceModes.Registered(GuidanceModeId.OrbitalEntry, ConsolePage.Land, MethodLabel, StopOrder, Phases,
            CommitVerb, StopLabel, "ENGINE UNCHANGED", "Ends orbital entry guidance. AFC stops steering. The engine is not touched.", EntryMethods.OrbitalLabel);

    public static (string Outcome, string Detail) StopText(OrbitalEntryStage stage) => stage switch
    {
        OrbitalEntryStage.Requested => ("NOTHING STARTED", "Cancels the ORBITAL ENTRY request before it starts."),
        OrbitalEntryStage.Sweeping => ("NOTHING FLOWN YET", "Ends orbital entry guidance while it measures the craft. AFC stops steering."),
        OrbitalEntryStage.Bellyflop => ("ATTITUDE FREE", "Ends BELLYFLOP. AFC stops steering and the craft falls as it is."),
        _ => ("ATTITUDE FREE", "Ends orbital entry guidance. AFC stops steering. The engine is not touched."),
    };

    public const string CommitHint =
        "Measures the craft belly first, then flies the entry to the hand-over at HAND-OVER MACH, banking to hold the planned drag and turning towards the site with roll reversals. With BELLYFLOP on, bellyflop falls onto the site from there. "
        + "Starts from orbit or from the air, with the engine off. Engaged already past the hand-over (a fall in the low air), it skips the entry and starts at BELLYFLOP, holding the attitude it has until bellyflop's first plan.";

    public const string NoAirText = "NO AIR ON THIS BODY. ORBITAL ENTRY NEEDS AN ATMOSPHERE.";
}

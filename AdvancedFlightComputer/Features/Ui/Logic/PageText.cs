namespace AdvancedFlightComputer.Features.Ui.Logic;

internal readonly record struct TabFacts(CraftSituation Where, bool BurnPlanned = false, bool AutoStageArmed = false, bool Editor = false);

/// <summary>The texts of the console. They are pure data, so their length and case can be checked without the game.</summary>
internal static class PageText
{
    private static readonly System.Globalization.CultureInfo Inv = Fmt.Inv;

    public static string BandPurpose(ConsolePage page, LandWay way, CraftSituation where = CraftSituation.Unknown) => page switch
    {
        ConsolePage.Launch => "From the pad to orbit",
        ConsolePage.Land => way switch
        {
            LandWay.FromOrbit => SituationText.IsFalling(where) ? "Down to the site" : "From orbit down to the site",
            LandWay.Boostback => "Spent booster back to its site",
            LandWay.FromHere => "Land from where you are",
            _ => "Sink and hold near the ground",
        },
        ConsolePage.Orbit => "Change the orbit with a stock burn",
        ConsolePage.Transfer => "To another body or craft",
        ConsolePage.Rcs => "Planned burns, who flies them",
        ConsolePage.Staging => "When stages drop",
        _ => "Preferences and tuning",
    };

    public static string WayCard(LandWay way) => way switch
    {
        LandWay.FromOrbit => "From orbit to the site. Stock AUTO flies the burn out of orbit, then AFC lands with TOUCHDOWN or G-FOLD. ENTRY is on LAND NOW.",
        LandWay.Boostback => "A spent booster back to its site. AFC turns it and aims its fall. Entry guidance can take the glide.",
        LandWay.FromHere => "From where the craft is now, with ENTRY, TOUCHDOWN or G-FOLD.",
        _ => "AFC stops the drift and sinks to the ground. You nudge the touchdown point.",
    };

    public static string ChooserCard(ConsolePage page) => page switch
    {
        ConsolePage.Launch => "From the pad into orbit.",
        ConsolePage.Land => "Bring the craft down: from orbit, as a spent booster, or from where it is.",
        ConsolePage.Orbit => "Change your orbit with a stock burn.",
        ConsolePage.Transfer => "Go to another body or craft.",
        ConsolePage.Rcs => "See your planned burns and pick who flies them.",
        ConsolePage.Staging => "When stages drop, and their delays.",
        _ => "How AFC flies, what it reads, and your preferences. Most flights need nothing here.",
    };

    public static string TabNow(ConsolePage page, in TabFacts f)
    {
        if (f.Editor)
        {
            return page switch
            {
                ConsolePage.Transfer => "In the editor only the link to the stock planner works.",
                ConsolePage.Staging => "Works in the editor.",
                ConsolePage.Tune => "PREFERENCES works in the editor.",
                _ => "",
            };
        }
        if (f.Where == CraftSituation.Unknown)
            return "";
        return page switch
        {
            ConsolePage.Launch => f.Where == CraftSituation.Ground ? "" : "Works on the ground.",
            ConsolePage.Orbit => f.Where == CraftSituation.Ground ? SituationVerbs.NeedsOrbit : "",
            ConsolePage.Transfer => f.Where is CraftSituation.Ground or CraftSituation.FallingAir or CraftSituation.FallingNoAir ? SituationVerbs.NeedsOrbit : "",
            ConsolePage.Land => f.Where == CraftSituation.Ground ? "Works in flight." : "",
            ConsolePage.Rcs => f.BurnPlanned ? "A burn is planned." : "No burn planned yet.",
            ConsolePage.Staging => f.AutoStageArmed ? "AutoStage is on for this craft." : "AutoStage is off for this craft.",
            _ => "",
        };
    }

    public static string TabCard(ConsolePage page, in TabFacts f)
    {
        string now = TabNow(page, in f);
        return now.Length > 0 ? ChooserCard(page) + " " + now : ChooserCard(page);
    }

    public static string WideCard(bool wide, bool panesFit, bool pageStaysNarrow = false)
        => (pageStaysNarrow ? "This page has no picture, so it keeps the narrow window, and this icon does nothing here. It works on a page with a picture."
            : !panesFit ? "This screen has no room for two panes, so this only widens the page."
            : wide ? "Back to one column."
            : "Two panes: the picture and its result at the left, the inputs and the folds at the right. A page that flies gives its picture the whole width, and a page with no picture stays narrow.")
           + "\n" + TagEveryCraft;

    public const string FlightOnlyInEditor = "In flight only. In the editor STAGING and TUNE work, and TRANSFER opens the stock planner.";

    public static string VerbWhen(GuidanceModeId mode) => mode switch
    {
        GuidanceModeId.AscentUpfg or GuidanceModeId.AscentGravityTurn => "Press it when the orbit is set.",
        GuidanceModeId.DeorbitLand => "Press it in orbit, when the site is set.",
        GuidanceModeId.Boostback => "Press it when the site of this stage is set.",
        GuidanceModeId.EntryHere => "Press it in a fall through air, when the site is set.",
        GuidanceModeId.SixDofHere => "Press it in a descent, when the site is set.",
        GuidanceModeId.GfoldHere => "Press it in a descent over a body with no air, when the site is set.",
        GuidanceModeId.Hover => "Press it near the ground.",
        _ => "",
    };

    public static string CreateCard(string goal, bool passes)
        => "Press it when " + goal + " is right. " + (passes ? "Stock AUTO flies each pass, and AFC adds the next when one ends." : "Stock AUTO flies the burn once you press FLY BURN or AUTO on the BURN gauge.");

    public const string FlyStockTip = "Hands this burn to stock AUTO, the same as AUTO on the BURN gauge. It burns at the planned time.";

    public const string FlyRcsTip = "AFC burns it with the RCS thrusters. STOP at the top of the window ends it.";

    public const string KeptRule = "A value you change stays with this craft until you load a save.";

    // How long a value lives, as the last line of its tooltip.
    public const string TagEveryCraft = "Saved for every craft.";
    public const string TagSaveDefaults = "Saved with SAVE DEFAULTS.";
    public const string TagCraftDesign = "Saved for this craft design.";
    public const string TagBurn = "Saved for this burn.";
    public const string TagNotKept = "Not kept, off after every load.";
    public const string TagGameSession = "Kept until the game closes, on at every start.";

    /// <summary>The owner colour of the chip at the left of the head, from the who-flies text the chip shows.</summary>
    public static Owner HeadOwner(string whoFlies) => OwnerOfFlownBy(whoFlies) switch
    {
        Owner.None => Owner.Player,
        Owner owner => owner,
    };

    public const string FailureOkTip = "You read it. The band goes, and the drawer keeps the message.";

    public const string TriesAgain = "A press tries again.";

    public const string StockChipTip = "This part belongs to the stock game, and AFC only uses it.";

    public static string HeadOwnerCard(string who, bool pilotHolds = false) => who switch
    {
        FlownBy.Afc => "AFC holds this craft now. STOP gives it back to you.",
        FlownBy.StockAuto => "Stock AUTO flies this burn now. STOP gives the craft back to you.",
        FlownBy.AfcPlansStockAuto => "AFC made the burn, and stock AUTO flies it. STOP ends both.",
        _ => pilotHolds ? "You fly this craft now. AFC holds no control of it, and STOP ends the mode." : "You fly this craft now.",
    };

    public static string FlownByCard(Owner owner) => owner switch
    {
        Owner.Afc => "After the press AFC flies the craft. STOP at the top of the window ends it.",
        Owner.Stock => "After the press stock AUTO flies the burn. STOP at the top of the window ends it.",
        Owner.AfcPlansStockFlies => "After the press the stock flight plan holds the burn, and you still fly. Stock AUTO flies it once you press FLY BURN or AUTO on the BURN gauge.",
        Owner.StockThenAfc => "After the press stock AUTO flies the burn out of orbit, and then AFC lands. STOP at the top of the window ends both.",
        _ => "",
    };

    public const string OnPressCard = "ON PRESS";

    public static string MoreLabel(int count, bool open)
        => "More" + (count > 0 ? " (" + count.ToString(Inv) + ")" : "") + (open ? " ^" : " v");

    public static string MoreCard(int messages, int otherCraft)
    {
        string count = (messages > 0 ? string.Create(Inv, $"{messages} new message{(messages == 1 ? "" : "s")}. ") : "")
            + (otherCraft > 0 ? string.Create(Inv, $"{otherCraft} other craft under AFC. ") : "");
        return count + "This craft, what runs, other craft, messages.";
    }

    public const string DrawerEmpty = "Nothing runs. No messages.";

    public const string BurnWarpLine = "To the burn";

    public const string BurnWarpTip = "Warps to the burn time, the same as WARP on the stock BURN gauge. A second press stops the warp.";

    public static string MayOffLine(bool steerOff, bool enginesOff)
        => steerOff && enginesOff ? "AFC may not steer or use the engines >"
            : steerOff ? "AFC may not steer >"
            : enginesOff ? "AFC may not use the engines >"
            : "";

    public const string MayOffTip = "Opens the drawer with the switches of this craft. With AFC MAY STEER off AFC does not steer: a mode that holds the craft ends, the engine is cut and you fly. With AFC MAY USE ENGINES off AFC steers, and you keep the engine and the stages.";

    /// <summary>Touchdown guidance reads neither switch (GuidanceWindow.ApplyAutopilot).</summary>
    public static string SteerSwitchNote(bool touchdown, bool holdsCraft)
        => touchdown ? "Touchdown guidance does not read this switch."
            : holdsCraft ? "If you turn it off: the mode ends, the engine is cut and you fly."
            : "";

    public static string EnginesSwitchNote(bool touchdown, bool holdsCraft)
        => touchdown ? "Touchdown guidance does not read this switch."
            : holdsCraft ? "If you turn it off: AFC keeps steering, and you keep the engine and the stages. A landing from orbit ends."
            : "";

    public static string StockLine(bool hasNode, string inTime, double dvMs)
        => hasNode ? string.Create(Inv, $"Next burn in {inTime}, {dvMs:F0} m/s.") : "No burn planned.";

    #region Choices

    /// <summary>The cards of the ASCENT choice, CAT-S / UPFG first, then the gravity turn.</summary>
    public static readonly string[] AscentCards =
    {
        "A climb that AFC plans or a simple turn, then UPFG steers into orbit.",
        "A simple pitch-over climb.",
    };

    /// <summary>
    /// The segments of the ASCENT choice. A method that is not there yet carries NotYetSuffix while the row has room for it, else only its name, because the right pane of the wide console is too narrow for both names with the suffix. The held plate and the card of that segment still say that it comes later.
    /// </summary>
    public static string[] AscentSegments(ReadOnlySpan<string> names, ReadOnlySpan<bool> available, Func<string[], bool> fits)
    {
        var full = new string[names.Length];
        bool suffixed = false;
        for (int i = 0; i < names.Length; i++)
        {
            bool later = i < available.Length && !available[i];
            full[i] = later ? names[i] + NotYetSuffix : names[i];
            suffixed |= later;
        }
        return !suffixed || fits(full) ? full : names.ToArray();
    }

    public static readonly string[] ClimbLabels = { "CALCULATED", "SIMPLE TURN" };

    public static readonly string[] ClimbCards =
    {
        "Calculates a climb inside MAX Q, then UPFG flies to orbit.",
        "Rises straight up, then tilts at a set rate, then UPFG flies to orbit.",
    };

    public static string ClimbTip(string method) => method + " flies the ascent. Pick how it climbs until UPFG takes over.";

    public static readonly string[] LaunchViewCards =
    {
        "Height against distance. Look here first, for the whole climb.",
        "Pitch against time. Look here for a hard turn.",
        "Dynamic pressure against time, as a share of the limit. Look here when the plan comes close to MAX Q.",
        "Throttle against time. Look here to see where the plan holds back.",
    };

    public static readonly string[] ClimbLines =
    {
        "A planned climb inside MAX Q, then UPFG.",
        "Straight up, a set turn, then UPFG.",
    };

    public static readonly string[] NodeLabels = { "ASCENDING NODE", "DESCENDING NODE" };

    public static readonly string[] NodeCards =
    {
        "The first turn heads north.",
        "The first turn heads south.",
    };

    public static readonly string[] FlyWithLabels = { "ENGINE, ELSE RCS", "RCS ONLY" };

    public static readonly string[] FlyWithCards =
    {
        "Stock AUTO burns with the engine. With no engine that has propellant, AFC uses RCS.",
        "AFC flies the burn with the RCS thrusters.",
    };

    public const string FlyWithTip =
        "The choice for this burn. WILL FLY in the list says who flies it with the craft as it is now.\n\n" + TagBurn;

    public static readonly string[] AttitudeLabels = { "CHEAPEST", "HOLD", "ALIGN" };

    public static readonly string[] AttitudeCards =
    {
        "Picks HOLD or ALIGN, whichever uses less propellant.",
        "Keeps the attitude and fires the thruster mix that points at the burn.",
        "Turns the strongest RCS axis onto the burn first, then fires.",
    };

    public const string AttitudeTip = "How AFC points the craft while RCS flies this burn.\n\n" + TagBurn;

    /// <summary>The ATTITUDE description while ENGINE, ELSE RCS resolves to the engine.</summary>
    public const string AttitudeIfRcsNote = "Used only if RCS takes this burn, when no engine has propellant at ignition.";

    public static readonly string[] AllocatorLabels = { "BY AXIS", "LEAST FUEL" };

    public static readonly string[] AllocatorCards =
    {
        "Fires the thrusters in axis groups, and attitude control holds the torque. LEAST FUEL falls back to it.",
        "Solves for the thruster mix that uses the least propellant, by a linear program. Falls back to BY AXIS when none fits.",
    };

    public const string AllocatorTip = "How AFC splits the burn among the RCS thrusters.\n\n" + TagBurn;

    public const string SolverTip = "Pick how it lands.";

    #endregion

    #region Tooltips

    public static string SettingTooltip(string help, bool changed, string defaultText, string setBy)
    {
        string tail = setBy.Length > 0 ? setBy : changed && defaultText.Length > 0 ? "Changed. " + defaultText + "." : "";
        return help.Length > 0 && tail.Length > 0 ? help + "\n" + tail : help + tail;
    }

    public const string TransferSelectTip = "Picks what the porkchop colors by, and jumps the selection to the best cell of that kind. A click on the active segment jumps back to it.";

    public static readonly string[] TransferSelectCards =
    {
        "The cell with the lowest departure delta-v.",
        "A cell with a low departure delta-v and also a low arrival speed at the destination. Use it to meet a craft.",
    };

    public static readonly string[] MoveLabels = { "^", "v", "<", ">" };

    public static readonly string[] MoveTips =
    {
        "Departs earlier: one row up the porkchop.",
        "Departs later: one row down the porkchop.",
        "A shorter flight time: one column left on the porkchop.",
        "A longer flight time: one column right on the porkchop.",
    };

    public const string MoveDisabled = "After CALCULATE";

    public const string MovePick = "MOVE PICK";

    public const string DepartAxis = "Depart in";

    public const string FlightTimeAxis = "Flight time";

    public const string PorkchopNoTransfer = "No transfer here: an impact, or a departure that cannot fly.";

    public const string CraftTargetChipTip = "Also sets the stock target of the craft.";

    #endregion

    #region Folds

    public const string HowItFlies = "HOW IT FLIES";

    public const string HowItFliesTip = "The finer settings of this page. The defaults fly. " + KeptRule;

    public static string PassesRowDetail(int passes, bool savingKnown = true)
        => passes <= 1 && savingKnown ? "A split saves under 1 m/s" : "";

    public static string PassesRowTip(bool savingKnown)
        => savingKnown ? "How many burns the craft flies. A split into more passes saves little on a burn this short." : "How many burns the craft flies.";

    public static string SearchLabel(double minS, double maxS) => "SEARCH: Flight time " + Fmt.Time(minS) + " to " + Fmt.Time(maxS);

    public const string SearchTip = "What CALCULATE searches, and whether the destination also becomes the stock target of the craft.";

    public const string TimeFieldTip = "Type a time such as 14m 24s, 1h 28m or 267d 6h, and press Enter.";

    public const string Numbers = "NUMBERS";

    public const string NumbersTip = "What the plan rests on. Nothing here changes it.";

    public const string Delays = "DELAYS";

    public const string DelaysTip = "How long after the staging trigger a part fires, for this craft and as defaults. The part window of a part also shows the delays of the stages it fires in.";

    public const string DelaysRule = "Delays count from the staging trigger. A decoupler with a shorter delay than the engine fires first.";

    public const string CraftDelaysDetail = "Seconds, saved at once";

    public const string DefaultDelaysDetail = "Seconds";

    public const string SaveDefaultsVerb = "SAVE DEFAULTS";

    public static string AllPartsLabel(int parts) => string.Create(Inv, $"ALL PARTS: {parts} parts");

    public const string AutoStageOffUntilLaunch = "A launch arms AutoStage at lift-off.";

    public const string SequenceCaption = "Delta-v of each box in m/s";

    public const string ResetFold = "RESET TO DEFAULTS";

    public const string ResetVerb = "RESET";

    public static string ResetFoldQuestion(int changed)
        => changed == 1 ? "Put the changed value of this fold back to its default?" : string.Create(Inv, $"Put the {changed} changed values of this fold back to their defaults?");

    public static string ResetSetQuestion(int changed, string set)
        => changed == 1 ? $"Put 1 {set} value back to its default?" : string.Create(Inv, $"Put {changed} {set} values back to their defaults?");

    public const string TuningRule = "Tuning changes how AFC flies. Values are live and reset on a save load.";

    public const string QAlphaNextLaunch = "MAX Q-ALPHA counts at the next launch.";

    public static string TuneSetsNeed(bool editor, bool hasCraft)
        => editor ? "In flight only."
            : hasCraft ? "Guidance is off. GUIDANCE ENABLED under PREFERENCES turns it on."
            : "Tuning values and diagnostics need a craft under control.";

    public static string TuneSetCard(TuneSet set) => set switch
    {
        TuneSet.Afc => "The guidance switch, the key of this console and how it opens.",
        TuneSet.Ascent => "How the climb flies. Look here when it misses its orbit.",
        TuneSet.Recover => "How a boostback flies. Look here when a booster misses its site.",
        TuneSet.Landing => "How a landing flies, one part at a time.",
        _ => "What AFC reads and calculates now. Nothing here changes a flight.",
    };

    public static readonly string[] TuneLandingCards =
    {
        "The way from orbit to the start of the descent.",
        "The glide and the landing burn of entry guidance.",
        "The powered descent of touchdown guidance.",
        "The powered descent of G-FOLD.",
        "The hover that sets the craft down.",
    };

    public static readonly string[] DiagViewCards =
    {
        "The calculated plan and the climb that flies.",
        "The plan of a boostback and its predicted impact.",
        "The air model of this craft.",
        "The air around the craft against the model.",
        "The committed plan of a G-FOLD descent.",
        "The live figures of touchdown guidance and its log.",
        "The stock table of the flight plan.",
    };

    public static string DiagEmpty(TuneDiagnostics view) => view switch
    {
        TuneDiagnostics.Ascent => "Empty until a plan is calculated or a climb flies.",
        TuneDiagnostics.Boostback => "Empty until a boostback flies.",
        TuneDiagnostics.Aero => "Empty until AFC samples the air around this craft.",
        TuneDiagnostics.Atmosphere => "Empty until AFC samples the air around this craft.",
        TuneDiagnostics.Log => "Empty until touchdown guidance flies.",
        TuneDiagnostics.FlightPlan => "Empty until a burn is planned or the craft has an orbit.",
        _ => "Empty until a G-FOLD descent solves.",
    };

    public static string PlanSolve(string message, int iterations, int accepted, double wallSeconds)
    {
        string what = Fmt.Sentence(message);
        if (!message.Contains("iterations", StringComparison.Ordinal))
            what += string.Create(Inv, $", {iterations} iterations");
        return what + string.Create(Inv, $" ({accepted} accepted), {wallSeconds:F1} s");
    }

    public static string PlanBurn(double seconds) => seconds.ToString("F1", Inv) + " s";

    public static string PlanIdeal(double idealDvMs) => Fmt.DvWhole(idealDvMs) + " ideal";

    public static string Stages(int count) => count == 1 ? "1 stage" : count.ToString(Inv) + " stages";

    public static string MirrorError(double share) => share < 1e-9 ? "0" : (share * 100.0).ToString("F4", Inv) + " %";

    public const string FlightPlanLink = "TUNE";

    public const string FlightPlanLinkTip = "Opens the stock table of this flight plan on TUNE > DIAG > FLIGHT PLAN. It reads the plan on this page as it stands now.";

    public const string FlightPlanOfBurn = "The stock table of the burn on MANEUVER, with its times as stock counts them.";

    public const string FlightPlanOfCraft = "The stock table of the flight plan of this craft, with its times as stock counts them.";

    public const string FlightPlanBackTip = "Back to the tool that made this plan.";

    public const string LanFromPositionTip = "Sets the LAN so that the plane of the orbit passes over the craft now.";

    public static string LimitsTip(bool hasAir)
        => hasAir ? "Open it to keep the climb under a dynamic pressure or an acceleration. The plan reads MAX Q." : "Open it to keep the climb under an acceleration. With no air here, MAX Q counts for nothing.";

    public static string BoostersTip(int stages)
        => "Open it to give a stage a site to fly home to. The calculated climb then sizes the fuel the stage keeps for that." + (stages <= 0 ? " No stage of this craft can use it now, because no part that separates carries a command pod." : "");

    public const string EngineDelaysTip = "How long after the staging trigger each engine lights.";

    public const string DecouplerDelaysTip = "How long after the staging trigger each decoupler fires.";

    public const string EarlierMessagesTip = "The older messages of this flight.";

    public const string TouchdownLimitsTip = "How hard the plan of touchdown guidance may fly. A change counts at its next plan.";

    public const string TouchdownTrackingTip = "How touchdown guidance follows its plan. A change counts at once.";

    public const string TouchdownSolverTip = "How touchdown guidance calculates. Open it only when a plan takes too long or fails.";

    public const string GfoldLimitsTip = "How hard the plan of G-FOLD may fly. A change counts at its next solve.";

    public const string GfoldTrackingTip = "How G-FOLD follows its plan. A change counts at once.";

    public const string GfoldSolverTip = "How G-FOLD calculates. Open it only when a plan takes too long or fails.";

    public static string FoldHolds(string contents) => "Holds: " + contents + ".";

    public static string LimitsLabel(bool hasAir, double maxQKpa, bool gLimitOn, double gLimit)
    {
        string g = gLimitOn ? string.Create(Inv, $"G-limit {gLimit:0.##} g") : "G-limit off";
        return hasAir ? string.Create(Inv, $"LIMITS: Max Q {maxQKpa:0.#} kPa, {g}") : "LIMITS: " + g;
    }

    public static string BoostersLabel(int stages, int withSite, double reserveKg)
    {
        string reserve = reserveKg > 0.0 ? "keeps " + Fmt.Tons(reserveKg) : "";
        if (stages <= 0)
            return reserve.Length > 0 ? "BOOSTERS: " + Fmt.Sentence(reserve) : "BOOSTERS: No stage can fly home";
        string sites = string.Create(Inv, $"{withSite} of {stages} with a site");
        return "BOOSTERS: " + sites + (reserve.Length > 0 ? ", " + reserve : "");
    }

    public const string BoostersNone = "No stage of this craft can fly home, because no part that separates carries a command pod.";

    /// <summary>
    /// The RESERVE row of BOOSTERS: what the booster with a stage site keeps for its way home. The calculated climb sizes it, so there is nothing to set. Empty when no stage has a site and nothing was staged. <paramref name="warn"/> marks a reserve that does not arm.
    /// </summary>
    public static string BoosterReserve(bool hasSite, bool armed, bool staged, double reserveKg, double boostbackDvMs, double landingDvMs, string note, out bool warn)
    {
        warn = false;
        if (staged)
            return "Staged. The booster was handed to the boostback.";
        if (!hasSite)
            return "";
        if (!armed)
        {
            warn = true;
            return "No reserve: " + (note.Length > 0 ? note : "waiting for a stage model");
        }
        if (!(reserveKg > 0.0))
            return double.IsFinite(boostbackDvMs) ? "Keeps nothing, the plan needs no reserve" : ReserveWaitsForPlan;
        if (double.IsFinite(boostbackDvMs) && double.IsFinite(landingDvMs))
            return string.Create(Inv, $"Keeps {reserveKg / 1000.0:F2} t for a {boostbackDvMs:F0} m/s boostback and a {landingDvMs:F0} m/s landing");
        return string.Create(Inv, $"Keeps {reserveKg / 1000.0:F2} t");
    }

    public const string ReserveWaitsForPlan = "Sized when the climb is calculated";

    #endregion

    #region Pictures with nothing to show

    public static string LaunchNoPlan(bool onGround)
        => onGround ? "CALCULATE draws the climb." : "No plan and no flown track.";

    public const string NoOrbit = "No orbit";

    public const string PorkchopEmpty = "CALCULATE fills this plot.";

    public const string PorkchopPassesFly = "No plot for these passes.";

    public const string NoBurnPlanned = "No burn planned.";

    public const string PassNotAddedYet = "AFC adds this pass when the pass before it ends, so it has no options yet.";

    public const string BurnListTip = "Every burn of the stock flight plan and the passes still to come. A press on a row picks the burn that BURN WITH sets. Options are kept per burn, so a later burn can be set up before it comes.";

    public const string RcsInUse = "IN USE";

    public const string RcsOnBoard = "RCS on board";

    public const string RcsNotEnough = "Not enough";

    public const string RcsCannotTranslate = "No active thruster with propellant can translate.";

    public const string NoRcsThrusters = "No RCS thrusters on this craft.";

    public const string NoStages = "No stages.";

    public const string AllStagesFired = "Every stage has fired.";

    public const string BoostbackNotYet = "No boostback yet. A stage with a stage site flies home after separation.";

    public const string BoostbackNoStage = "A stage can fly itself home only when the part that separates carries a command pod.";

    public const string NoSiteToDraw = "No site picked on this body.";

    public const string EntryFirstPlan = "The first plan takes a few seconds.";

    public const string EntryNoPlan = "No plan. The glide flies on with the engine off.";

    public const string HoverIdle = "The dial shows while the hover flies.";

    public static string SiteNonePicked(string body) => "SITE " + body + ": none picked";

    public const string SiteTypeRow = "DEFAULT, OR TYPE IT";

    public const string SiteTypeRowShort = "DEFAULT";

    public const string SiteDefaultTip = "These numbers are the stored default, a place on Luna. A press with no pick flies to them on this body.";

    public static string NoSiteAnswer(string verb, double latDeg, double lonDeg)
        => string.Create(Inv, $"No site. {verb} flies to {latDeg:F3}, {lonDeg:F3}.");

    public const string SiteFar = "Site too far for this fall. Pick a site below you.";

    /// <summary>The card of PICK SITE while a committed deorbit holds the site: a click then only says so (GuidanceWindow.RetargetLandingSite).</summary>
    public const string SiteLockedByDeorbit = "Site locked by the deorbit. Stop it and plan again to move the site.";

    /// <summary>The card of a value or switch that a committed deorbit reads, while it is disabled.</summary>
    public const string DeorbitLockedSetting = "Locked while a deorbit waits or flies, because a change stops the deorbit. Stop it and plan again to change this.";

    public static string HoverVerdict(double minTwr, double fullTwr)
        => minTwr > 1.0 ? string.Create(Inv, $"TWR {minTwr:F2} here at minimum throttle, so no hover now.")
            : fullTwr < 1.0 ? string.Create(Inv, $"TWR {fullTwr:F2} here at full throttle, so no hover now.")
            : string.Create(Inv, $"TWR {minTwr:F2} here at minimum throttle, so it can hover.");

    public static bool HoverThrustFails(double minTwr, double fullTwr) => minTwr > 1.0 || fullTwr < 1.0;

    public const string HoverFarAbove = "Far above hover range. STOP HOVER, cut the engine.";

    public const string OffScale = "off scale";

    private const double HoverFarMeters = 5000.0;

    public static bool HoverIsFar(double altitudeM) => altitudeM > HoverFarMeters;

    public const string HeightAboveSite = "Height above site";

    public const string HeightAboveGround = "Height above ground";

    public const string NewPlan = "New plan";

    public const string FlownPath = "Flown path";

    public const string SiteTick = "Site";

    public const string SequenceWord = "SEQUENCE";

    public static string BrakingStart(double km) => "Braking start " + Fmt.KmFine(km);

    public static string VerticalGate(double meters) => "Vertical " + Fmt.Distance(meters);

    public const string StagesAtBraking = "AFC reads the stages at the braking burn.";

    public static string PeakQ(double kpa) => "Peak Q " + kpa.ToString("F0", Inv) + " kPa";

    public static string QLimits(double maxQKpa, double maxQAlphaPaRad)
        => "Limit Q " + maxQKpa.ToString("F0", Inv) + " kPa, Q-alpha " + maxQAlphaPaRad.ToString("F0", Inv) + " Pa rad";

    public static string QLimitShort(double maxQKpa) => "Limit Q " + maxQKpa.ToString("F0", Inv) + " kPa";

    public static string MinThrottle(double percent) => "Min " + percent.ToString("F0", Inv) + " %";

    public const string QTuneLink = "TUNE";

    public const string QTuneTip = "Opens TUNE > ASCENT, which holds MAX Q-ALPHA. MAX Q stands in LIMITS on this page.";

    public static string PlaneChange(double deg) => "Plane change " + Fmt.Deg(Math.Abs(deg));

    public static string BurnAtNode(bool descending) => descending ? "Burn at descending node" : "Burn at ascending node";

    public const string Escapes = "Escapes";

    public static string StoppedAt(double metersAboveSite) => "Stopped at " + Fmt.Distance(metersAboveSite);

    public const string ZoomHint = "Wheel zooms, middle click fits";

    public const string PassesTitle = "PASSES, track from site";

    public const string NextPassTitle = "NEXT PASS, track from site";

    public static string PassTrack(double km, double inSeconds) => Fmt.KmFine(km) + " in " + Fmt.Time(inSeconds);

    public static string PassNext(bool alsoClosest) => alsoClosest ? "The next pass is the closest" : "Next pass";

    public static string PassClosest(double km, double inSeconds) => "Closest " + PassTrack(km, inSeconds);

    public static string PassFlies(double km, double inSeconds) => "Flies the pass in " + Fmt.Time(inSeconds) + ", now " + Fmt.KmFine(km) + " off";

    public static string BrakingIn(double seconds) => "Braking in " + Fmt.Time(seconds);

    public const string NoPassSolution = "No pass with a solution";

    public const string CoastKey = "Coast";

    public const string BrakingKey = "Braking burn";

    public const string CraftKey = "Craft";

    public const string CoastWord = "Coast";

    public const string ThrustKey = "Thrust";

    public static string PassWithin(double km) => "Within " + Fmt.Km(km);

    public const string OtherPassesKey = "Other passes";

    public static string PorkchopRampName(bool arrival) => arrival ? "Arrival speed" : "Departure delta-v";

    public const string PorkchopNoneKey = "No transfer";

    public static string PorkchopOnBoard(double onBoardMs)
        => double.IsFinite(onBoardMs) ? "Has " + Fmt.DvWhole(onBoardMs) : "";

    public const string NoSketchClimb = "No orbit to sketch until cutoff.";

    public const string NoSketchFall = "This path ends on the ground, so no orbit to sketch.";

    public const string PeInAir = "in the air";

    public const string PeBelowGround = "PE below ground";

    public const string SketchPeBelowGround = "Pe below ground";

    public const string AirKey = "Air";

    public const string TouchdownStandby = "Touchdown guidance stands by under entry guidance.";

    public const string TouchdownEngaging = "Touchdown guidance starts.";

    #endregion

    #region Footer

    /// <summary>The owner a who-flies text names, for a footer that sets no owner of its own.</summary>
    public static Owner OwnerOfFlownBy(string flownBy) => flownBy switch
    {
        FlownBy.Afc => Owner.Afc,
        FlownBy.StockAuto => Owner.Stock,
        FlownBy.AfcPlansStockAuto => Owner.AfcPlansStockFlies,
        FlownBy.You => Owner.Player,
        _ => Owner.None,
    };

    public static OwnerMark FooterMark(Owner owner, string flownBy, int burns = 1) => owner switch
    {
        Owner.Afc => new OwnerMark(FlownBy.OnPress + " " + FlownBy.Afc),
        Owner.Stock => new OwnerMark(FlownBy.OnPress, FlownBy.StockAuto),
        Owner.AfcPlansStockFlies => burns > 1 ? new OwnerMark(FlownBy.PlansMany(burns), FlownBy.StockChip, FlownBy.BurnsWord) : new OwnerMark(FlownBy.PlansA, FlownBy.StockChip, FlownBy.BurnWord),
        Owner.StockThenAfc => new OwnerMark(FlownBy.OnPress, FlownBy.StockAutoName, FlownBy.ThenAfc),
        Owner.Player => new OwnerMark(FlownBy.OnPress + " " + FlownBy.You),
        Owner.Settings => OwnerMark.None,
        _ => flownBy == FlownBy.Nobody ? OwnerMark.None : new OwnerMark(FlownBy.OnPress + " " + flownBy),
    };

    /// <summary>A value that is not known is NaN or empty, and the sentence leaves it out.</summary>
    internal readonly record struct OutcomeFacts(
        bool AutoStageArms = false, string Target = "", string Solver = "", bool HandToTouchdown = false, double TouchdownLastS = double.NaN,
        double HoverFromM = double.NaN, bool MayFly = true, bool HoverFlies = true, bool Falling = false, bool HoverKnown = true);

    /// <summary>The end of every launch outcome while guidance will arm AutoStage when the ascent starts.</summary>
    public const string AutoStageArmsNote = "AutoStage arms at lift-off.";

    public static string GuidanceOutcome(GuidanceModeId mode, string verb, in OutcomeFacts f)
    {
        string autoStage = f.AutoStageArms ? " " + AutoStageArmsNote : "";
        if (verb == CommitResolver.BackupLaunchVerb)
            return "Flies the simple turn, not the calculated plan." + autoStage;
        if (verb == "CALCULATE AND LAUNCH")
            return "A press calculates the plan and launches." + autoStage;
        if (verb == "CALCULATE AND ARM")
            return "A press calculates the plan and arms for the window." + autoStage;
        if (verb == "ARM FOR WINDOW")
            return (f.Target.Length > 0
                ? "Waits for the window of " + f.Target + ", then launches."
                : "Waits for the launch window, then launches.") + autoStage;
        switch (mode)
        {
            case GuidanceModeId.AscentUpfg or GuidanceModeId.AscentGravityTurn:
                return ((f.MayFly ? "" : MayOffLaunch) + autoStage).TrimStart();
            case GuidanceModeId.Boostback:
                return "Turns the booster back to its site.";
            case GuidanceModeId.DeorbitLand:
                return f.Falling
                    ? (f.Solver.Length > 0 ? "Lands with " + f.Solver + ", with a stock burn first if it needs one." : "Lands, with a stock burn first if it needs one.")
                    : (f.Solver.Length > 0 ? "Burns out of orbit, then lands with " + f.Solver + "." : "Burns out of orbit, then lands.");
            case GuidanceModeId.EntryHere:
                return "Glides engine off, then flies the landing burn."
                       + (f.HandToTouchdown && double.IsFinite(f.TouchdownLastS)
                           ? string.Create(Inv, $" TOUCHDOWN flies the last {f.TouchdownLastS:F0} s.")
                           : "");
            case GuidanceModeId.SixDofHere:
                return "Flies a powered descent to the ground.";
            case GuidanceModeId.GfoldHere:
                return !f.HoverFlies ? (f.HoverKnown ? GfoldNoHoverOutcome : "Flies a G-FOLD descent.")
                    : double.IsFinite(f.HoverFromM)
                        ? string.Create(Inv, $"Flies a G-FOLD descent, then hovers down from {f.HoverFromM:F0} m.")
                        : "Flies a G-FOLD descent, then hovers down.";
            case GuidanceModeId.Hover:
                return "Stops the drift and sinks to the ground. Nudge to move.";
            default:
                return "";
        }
    }

    public const string GfoldNoHoverOutcome = "Flies a G-FOLD descent to the ground.";

    public const string ClimbFliesCard = "AFC flies the climb, and AutoStage drops the stages.";

    public const string MayOffLaunch = "AFC MAY STEER or AFC MAY USE ENGINES is off, so the launch only starts guidance.";

    public static string TakeOverOutcome(GuidanceModeId mode, string from)
    {
        string running = InSentence(from);
        return mode switch
        {
            GuidanceModeId.GfoldHere or GuidanceModeId.SixDofHere or GuidanceModeId.EntryHere or GuidanceModeId.Hover or GuidanceModeId.Boostback or GuidanceModeId.DeorbitLand
                => "A press replaces " + running + " with " + SituationVerbs.VerbName(mode) + ".",
            GuidanceModeId.AscentUpfg or GuidanceModeId.AscentGravityTurn => "A press replaces " + running + " with LAUNCH.",
            _ => "A press ends " + running + ".",
        };
    }

    public static string InSentence(string text)
        => text.Length > 1 && char.IsUpper(text[0]) && char.IsLower(text[1])
            ? char.ToLowerInvariant(text[0]) + text.Substring(1)
            : text;

    public static bool FailureOf(string what, GuidanceModeId mode)
    {
        if (what.StartsWith("AFC ", StringComparison.Ordinal))
            return true;
        string word = mode switch
        {
            GuidanceModeId.GfoldHere => "G-FOLD",
            GuidanceModeId.SixDofHere => "touchdown",
            GuidanceModeId.EntryHere => "entry",
            GuidanceModeId.Hover => "hover",
            GuidanceModeId.Boostback => "boostback",
            GuidanceModeId.DeorbitLand => "landing",
            GuidanceModeId.AscentUpfg or GuidanceModeId.AscentGravityTurn => "ascent",
            _ => "",
        };
        return word.Length > 0 && what.Contains(word, StringComparison.OrdinalIgnoreCase);
    }

    public const string CreateOutcome = "Nothing burns yet.";

    public static string BurnAnswer(double dvMs, string when, double budgetMs, bool chained = false)
    {
        string what = Fmt.Dv(dvMs) + (when.Length > 0 ? " " + when : "");
        if (!double.IsFinite(budgetMs) || budgetMs <= 0.0)
            return what;
        double percent = dvMs / budgetMs * 100.0;
        string share = dvMs > 0.0 && percent < 1.0 ? "under 1 %" : string.Create(Inv, $"{percent:F0} %");
        return string.Create(Inv, $"{what}, {share} of ~{budgetMs:F0} m/s{(chained ? " left" : "")}");
    }

    public static string PassesOutcome(int passes)
        => string.Create(Inv, $"Adds pass 1 of {passes}. AFC adds each next pass.");

    public const string CorrectionOutcome = "Adds a burn that corrects the course. Nothing burns yet.";

    public const string FlyOutcome = "Nothing burns until you press.";

    public const string SaveDefaultsOutcome = "Writes the default delays for every new craft.";

    public static string ResetSetOutcome(int changed)
        => changed == 1 ? "Puts 1 changed value back to its default."
            : string.Create(Inv, $"Puts {changed} changed values back to their defaults.");

    public const string DeleteNodeTip = "Removes this burn from the stock flight plan. Nothing burns. Plan it again on MANEUVER or TRANSFER.";
    public const string DeleteCreatedTip = "Removes the burn you created from the stock flight plan. Nothing burns. Plan it again on this page.";

    public const string DeletePassTip = "Removes this pass from the stock flight plan, and the passes end. Nothing burns. Plan them again on the page that made them.";

    public static string DeletePassLine(int pass, int total)
        => pass < total ? string.Create(Inv, $"DELETE PASS {pass} also ends the passes after it.") : "";

    public const string DeleteEndsLanding = "DELETE BURN also ends the landing.";

    public const string DeleteDeorbitTip = "Removes this burn from the stock flight plan, and the landing ends, because it needs this burn. Nothing burns.";

    public const string OutlineStillWorks = "It does not fit now, but a press still acts.";

    public const string OutlinePressEnds = "It does not fit now. A press ends at once.";

    public const string CalculateOutcome = "Calculates every departure time and flight time. Nothing burns.";

    public const string CalculateTip = "Press it when the destination is right. RE-CALCULATE under the picture runs it again.";

    public static string FuelShort(double neededMs, double onBoardMs, bool chained = false)
        => neededMs > onBoardMs ? string.Create(Inv, $"Needs {neededMs:F0} m/s, {(chained ? "your last burn leaves" : "has")} ~{onBoardMs:F0}.") : "";

    public const string BurnAfterImpact = "This burn comes after the impact.";

    public static string NoDestination(string body)
        => body.Length > 0
            ? "Nothing to reach from " + body + ". TRANSFER lists other craft here and bodies that share its parent."
            : "Nothing to reach from here.";

    public static string NextLabel(ConsolePage page, LandWay? way)
        => (way is LandWay w ? LandWays.Label(w) : ConsolePages.Label(page)) + " >";

    public const string FitsNowNote = "The dot: this way fits where the craft is now.";

    public const string PageRunsNote = "The dot: something runs on this page for this craft.";

    public const string HoverNoEngine = "No lit liquid engine, so no hover.";

    public const string NoGimbalFact = "No gimballed engine, so TOUCHDOWN cannot fly.";

    public static string OverPoweredFact(double floor) => string.Create(Inv, $"Throttle floor too high: no descent above {floor:F2}.");

    public static string LandingStopped(bool engineWasOff, bool burnDeleted)
        => "Stopped. " + (engineWasOff ? "Engine off" : "Engine cut") + (burnDeleted ? ", burn deleted." : ".");

    public static string PickRowTip(LandSolver solver) => "Picks " + LandSolvers.Label(solver) + " in the table.";

    public const string NoLiftModel = "No lift data, so the glide holds retrograde.";

    public const string NoLiftAirframe = "No usable lift, so the glide holds retrograde.";

    public const string LegsStowed = "Legs stowed. Press Deploy in the window of each leg.";

    public static string LegsStowedCard(double deployedM, double nowM)
        => string.Create(Inv, $"The landing plans with the legs down, so COM ABOVE LEGS is {deployedM:F1} m. With the legs as they are now it is {nowM:F1} m.");

    public static string ApseMargin(bool setPeriapsis, double marginKm)
        => string.Create(Inv, $"Target must be at least {marginKm:F0} km {(setPeriapsis ? "below the apoapsis" : "above the periapsis")}");

    public const string AtTargetInclination = "Already at the target inclination";

    public static string NodeChoice(bool descending, double dvMs, double inS)
        => (descending ? "DESC " : "ASC ") + NodeCost(dvMs, inS);

    public static string NodeLine(double ascendingMs, double ascendingS, double descendingMs, double descendingS)
        => "Ascending " + NodeCost(ascendingMs, ascendingS) + ".\nDescending " + NodeCost(descendingMs, descendingS) + ".";

    private static string NodeCost(double dvMs, double inS)
        => double.IsFinite(dvMs) && double.IsFinite(inS) ? Fmt.DvWhole(dvMs) + ", " + Fmt.TimeShort(inS) : "none";

    /// <summary>The refusal that StartTerminalHover reads, in the words of the footer.</summary>
    public const string HoverNoThrust = "No hover at this thrust.";

    public const string PickAnotherWay = "Pick another way.";

    public static string HoverProblemLine(string problem) => problem == HoverNoThrust ? PickAnotherWay : problem;

    public const string PlanCalculating = "Calculating the plan";

    public static string PlanProgress(string progress, double elapsedS)
        => string.Create(Inv, $"{progress.Replace(": ", ", ")} ({elapsedS:F0} s)");

    public static string PlanProgressShort(string progress, double elapsedS)
    {
        int colon = progress.IndexOf(':');
        string what = colon > 0 ? progress.Substring(0, colon) : progress;
        return what.Length > 0 ? string.Create(Inv, $"Calculating, {what} ({elapsedS:F0} s)") : string.Create(Inv, $"Calculating ({elapsedS:F0} s)");
    }

    public static string LaunchWaitVerb(bool atWindow) => atWindow ? "ARMS WHEN READY" : "LAUNCHES WHEN READY";

    public static string LaunchWaitTip(bool atWindow)
        => atWindow ? "It arms for the launch window when the plan is ready." : "It launches when the plan is ready.";

    public static string WhenReadyVerb(bool atWindow) => atWindow ? "ARM WHEN READY" : "LAUNCH WHEN READY";

    public const string NoLaunchFollows = "No launch follows unless you press.";

    public static string CalculatingVerbTip(bool atWindow)
        => atWindow ? "A press arms for the window when the plan is ready. Without one nothing arms." : "A press launches when the plan is ready. Without one nothing launches.";

    public const string CancelVerb = "CANCEL";

    public const string CancelWaitTip = "Cancels the calculation and the launch that waits on it.";

    public const string CancelTransferTip = "Cancels the calculation.";

    public static string FlownAgainstGoal(double peKm, double apKm, double goalPeKm, double goalApKm)
        => (peKm < 0.0 ? "Flown to Ap " + Fmt.Km(apKm) : FlownWord + " " + Fmt.Orbit(peKm, apKm)) + ", goal " + Fmt.Orbit(goalPeKm, goalApKm) + ".";

    public const string FlownWord = "Flown";

    public static bool GoalReached(double peKm, double apKm, double goalPeKm, double goalApKm)
        => double.IsFinite(peKm) && double.IsFinite(apKm) && peKm >= 0.0
            && Math.Abs(peKm - goalPeKm) <= Math.Max(5.0, 0.02 * Math.Abs(goalPeKm))
            && Math.Abs(apKm - goalApKm) <= Math.Max(5.0, 0.02 * Math.Abs(goalApKm));

    public const string PlanStarting = "Starting the calculation";

    /// <summary>The WINDOW row: the time to the ignition and its node. <paramref name="planShiftS"/> is how far the calculated climb moved its own lift-off from the window, s, positive for later; the countdown then runs to that lift-off.</summary>
    public static string WindowCountdown(double waitS, bool descending, double leadS, double planShiftS = 0.0)
    {
        string node = descending ? ", descending node, " : ", ascending node, ";
        if (planShiftS == 0.0)
            return "T-" + Fmt.Time(waitS) + node + string.Create(Inv, $"{leadS:F0} s lead");
        return "T-" + Fmt.Time(waitS + planShiftS) + node
            + string.Create(Inv, $"the plan's own time, {Math.Abs(planShiftS):F0} s {(planShiftS > 0.0 ? "after" : "before")} the {leadS:F0} s lead");
    }

    /// <summary>The sentence under an armed launch. <paramref name="planShiftS"/> is how far the calculated climb moved its own lift-off from the window, s, positive for later. The launch then goes at that lift-off, which the WINDOW row and the head also count to.</summary>
    public static string ArmedLaunch(bool warping, bool autoLaunch, double planShiftS = 0.0)
    {
        // Below half a second the shift would print as "0 s", so the sentence names the window.
        if (!double.IsFinite(planShiftS) || Math.Abs(planShiftS) < 0.5)
            return warping ? "Warping to the launch window."
                : autoLaunch ? "Armed. Launches at the window."
                : "Armed. Warp to the window yourself, it launches there.";
        string shift = PlanLaunchShift(planShiftS);
        return warping ? "Warping to the planned lift-off, " + shift + "."
            : autoLaunch ? "Armed. Launches at the planned lift-off, " + shift + "."
            : "Armed. Warp to the planned lift-off yourself, " + shift + ", it launches there.";
    }

    /// <summary>The name of the switch that lets the calculated climb choose its timing: with a target it chooses the launch time, with none the LAN.</summary>
    public static string OptimiseTimingLabel(bool hasTarget) => hasTarget ? "CHOOSE LAUNCH TIME" : "CHOOSE LAN";

    /// <summary>The DIAG line of a booster reserve the plan sized: what the booster keeps, for what, and when it separates.</summary>
    public static string PlanBooster(double reserveKg, double boostbackDvMs, double landingDvMs, double separationS)
        => string.Create(Inv, $"keeps {reserveKg / 1000.0:F2} t: {boostbackDvMs:F0} m/s boostback + {landingDvMs:F0} m/s landing, separates at T+{separationS:F0} s");

    /// <summary>The DIAG line of a lift-off the plan moved from the launch window, s, positive for later.</summary>
    public static string PlanLaunchShift(double delayS)
        => string.Create(Inv, $"{Math.Abs(delayS):F0} s {(delayS > 0.0 ? "after" : "before")} the window");

    /// <summary>The DIAG line of a LAN the plan chose itself.</summary>
    public static string PlanLanChosen(double lanDeg) => string.Create(Inv, $"{lanDeg:F2} deg, chosen for the pad");

    public const string NoCrossing = "Inclination below site latitude";

    public static string TargetOrbit(double peKm, double apKm, double incDeg) => Fmt.Orbit(peKm, apKm) + ", inc " + Fmt.Deg(incDeg);

    public static string LaunchAnswer(string why, string stored, bool planUsable, bool flyConvex, double massToOrbitKg, int stagesPlanned, int stagesAvailable)
    {
        bool failed = !planUsable && stored.Length > 0 && !IsPlanStatusTransient(stored);
        if (failed)
            return stored;
        if (why.Length > 0)
            return PlanState(why);
        return flyConvex
            ? string.Create(Inv, $"{PlanReadyWord}: {Fmt.Tons(massToOrbitKg)} to orbit, {stagesPlanned} of {stagesAvailable} stages")
            : "Ready, the simple turn flies";
    }

    public const string PlanReadyWord = "Plan ready";

    private static bool IsPlanStatusTransient(string stored) => stored is "Starting..." or "Calculating...";

    public static string BurnFuelLine(string payer, double propellantKg, double shortMs)
        => shortMs <= 0.5 && payer.Length > 0 ? "Paid by " + payer + ", ~" + Fmt.Mass(propellantKg) + "." : "";

    public static string PassesPaidBy(string payer) => payer.Length > 0 ? "Paid by " + payer : "";

    public static string PassesPlan(string goal, int passes)
        => goal.Length > 0 ? string.Create(Inv, $"{goal} in {passes} passes") : "";

    public static string PassesGoal(string goal, int passes)
        => goal.Length > 0 ? "Goal: " + PassesPlan(goal, passes) + "." : "";

    public static string BurnGoal(double nowPeKm, double nowApKm, double afterPeKm, double afterApKm, bool closes)
    {
        if (!closes)
            return Escapes;
        double pe = Math.Abs(afterPeKm - nowPeKm), ap = Math.Abs(afterApKm - nowApKm);
        return pe < 1.0 && ap < 1.0 ? "Orbit " + Fmt.Orbit(afterPeKm, afterApKm)
            : ap >= pe ? "Ap " + Fmt.Km(afterApKm)
            : "Pe " + Fmt.Km(afterPeKm);
    }

    public const string HowItFliesDefaults = HowItFlies + ": All defaults";

    public const string GoalOrbitReached = "Goal orbit reached. Pick a tool for another orbit.";

    public const string PassFuelEnds = "fuel ends";

    public static string PassesShort(double shortMs) => string.Create(Inv, $"The fuel ends ~{shortMs:F0} m/s short of the goal.");

    public const string PickSiteCue = "Pick site: click map or world. Right click cancels.";

    public const string PickSite = "PICK SITE";

    public const string PlanTip = "Setup of this page. The sentence inside says what counts now.";

    public static string PassWaits(bool rcs) => rcs ? "Press: AFC RCS burns it." : "Press: stock AUTO burns it.";

    public static string FlyPassTip(bool rcs)
        => (rcs ? "AFC burns this pass with the RCS thrusters." : "Hands this pass to stock AUTO, the same as AUTO on the BURN gauge.")
            + " It burns at the planned time. AFC adds the next pass when this one ends.";

    /// <summary><paramref name="marginMs"/> is the delta-v on board less what guidance still needs, NaN while no solve says that.</summary>
    public static HeadValue[] AscentNumbers(bool bound, double apKm, double apGoalKm, double peKm, double peGoalKm, double marginMs = double.NaN)
    {
        var values = new List<HeadValue>(3);
        if (bound)
            values.Add(new HeadValue(string.Create(Inv, $"AP {apKm:F0} km, target {apGoalKm:F0}"), 1, string.Create(Inv, $"AP {apKm:F0} km")));
        bool peLow = peKm < 0.0;
        var margin = new HeadValue(string.Create(Inv, $"MARGIN {marginMs:+0;-0} m/s"), peLow ? 0 : 2, Tone: marginMs <= 0.0 ? PlanTone.Warn : PlanTone.Plain);
        if (peLow && double.IsFinite(marginMs))
            values.Add(margin);
        values.Add(peLow
            ? new HeadValue(PeBelowGround, 3)
            : new HeadValue(string.Create(Inv, $"PE {peKm:F0} km, target {peGoalKm:F0}"), 0, string.Create(Inv, $"PE {peKm:F0} km")));
        if (!peLow && double.IsFinite(marginMs))
            values.Add(margin);
        return values.ToArray();
    }

    /// <summary>The numbers of a boostback glide in the head: the miss, its two parts where the row has room, and the time until entry guidance takes over, NaN when it does not.</summary>
    public static HeadValue[] GlideNumbers(double downM, double crossM, double entryInS = double.NaN)
    {
        double missKm = Math.Sqrt(downM * downM + crossM * crossM) / 1000.0;
        string miss = "MISS " + Fmt.KmFine(missKm);
        string parts = string.Create(Inv, $"{Math.Abs(downM) / 1000.0:F1} {(downM < 0.0 ? "short" : "long")}, {Math.Abs(crossM) / 1000.0:F1} {(crossM < 0.0 ? "right" : "left")}");
        var head = new HeadValue(miss + ", " + parts, 0, miss, PlanWords.MissTone(missKm, flying: true));
        return double.IsFinite(entryInS) && entryInS > 0.0
            ? [head, new HeadValue("ENTRY IN " + Fmt.Time(entryInS))]
            : [head];
    }

    public static string AscentToGo(double speedMs, double distanceKm) => string.Create(Inv, $"Speed to gain {speedMs:F0} m/s. Distance to go {distanceKm:F0} km.");

    public static string BrakingToGo(double speedMs) => string.Create(Inv, $"Speed still to change {speedMs:F0} m/s.");

    public static string ImpactCard(double seconds) => "Impact in " + Fmt.Time(seconds) + " with no further burn.";

    public const string GlideNumbersCard = "Long and short are along the path, left and right across it, from the site to where the craft comes down.";

    public const string HoverNumbersCard = "ALT is the height of the legs above the ground under the craft.";

    public const string OnSite = "On site";

    public const string LandsHereNow = "Lands here now";

    public const string ImpactWord = "Impact";

    public const string ImpactOnSite = "Impact on site";

    public static string ImpactOff(double km) => "Impact " + Fmt.KmFine(km) + " off";

    public const string LandsIn = "LANDS IN";

    /// <summary>The name of a burn that stock AUTO flies in the head, with the step it stands on.</summary>
    public static string StockBurnName(bool burning) => burning ? "BURN, BURNING" : "BURN, WAITING";

    /// <summary>The time beside the delta-v of a burn that stock AUTO flies: the wait to its ignition, until an engine burns.</summary>
    public static string StockBurnWhen(bool burning, string toIgnition) => burning ? "Burning" : "IN " + toIgnition;

    public static string StepOf(int step, int steps) => string.Create(Inv, $"Step {step} of {steps}");

    public const string OpenLandNowTip = "Opens LAND NOW, which starts a landing mode from here.";

    public const string TakeOverTouchdown = "TAKE OVER: TOUCHDOWN";

    public const string TakeOverTouchdownTip = "Ends entry guidance and starts touchdown guidance now, with no question, because this glide has no plan and the engine stays off to the ground.";

    public static string RefusedAnswer(string fitText, string line)
    {
        const string word = "refused: ";
        string why = fitText.Length > 0 ? fitText : line;
        int at = why.IndexOf(word, StringComparison.OrdinalIgnoreCase);
        return "Refused: " + InSentence(at >= 0 ? why.Substring(at + word.Length) : why);
    }

    public const string PickArmed = "PICK A SPOT";

    public const string PickSiteTip = "The next click on the map or in the world moves the landing site. A right click cancels.";

    public const string PickArmedTip = "Waits for a click on the map or in the world that moves the landing site. A right click or this button cancels.";

    public const string NoThrustToJudge = "No lit engine, so no TWR yet.";

    public static string HandOversLabel(bool entryTakesGlide, double glideAtS, bool touchdownTakesBurn, double burnLastS)
    {
        string glide = entryTakesGlide ? string.Create(Inv, $"Entry {glideAtS:F0} s to impact") : "Entry off";
        string burn = touchdownTakesBurn ? string.Create(Inv, $"touchdown last {burnLastS:F0} s") : "touchdown off";
        return "HAND-OVERS: " + glide + ", " + burn;
    }

    public static string HoverHandOverLabel(double fromM, bool hoverCanFly, bool known = true)
        => !hoverCanFly ? "HAND-OVERS: No hover at this thrust"
            : known ? string.Create(Inv, $"HAND-OVERS: Hover from {fromM:F0} m")
            : string.Create(Inv, $"HAND-OVERS: If it hovers, from {fromM:F0} m");

    public const string HandOversTip = "Who takes the glide of a boostback, and who takes the end of the landing burn.";

    public const string HoverHandOverTip = "Where G-FOLD stops flying and the hover takes over, if the craft can hover there.";

    public const string NoHandOversLabel = "HAND-OVERS: None";

    public static string NoHandOversTip(LandSolver solver) => solver == LandSolver.Entry
        ? "With no air ENTRY gets no plan, so it hands over to nothing."
        : LandSolvers.Label(solver) + " flies to the ground, so it hands over to nothing.";

    public const string BoostbackTakesCraft = "A press takes the craft, also on the pad. No stage can fly home.";

    public const string StopWord = "STOP";

    public const string HoverNotUsed = "The thrust at minimum throttle is above the weight, so this value does nothing now.";

    public const string NumpadHint = "TUNE > PREFERENCES > NUMPAD NUDGES THE HOVER lets the numpad keys nudge the aim.";

    public const string PressPlansNext = "A press flies the next pass.";

    public const string PassesFar = "No pass comes near the site. Move the site or the plane.";

    public static string QRelaxedNote(double askedKpa, double usedKpa)
        => string.Create(Inv, $"Max Q {askedKpa:0.#} kPa is out of reach, so the plan flies to {usedKpa:0.#} kPa.");

    public static string FlightTimeTip(string hohmannTime) => "The range CALCULATE searches. A Hohmann transfer takes " + hohmannTime + ".";

    public static string EstimateBasis(bool loadedBurn) => loadedBurn ? "Estimate from the loaded burn" : "Estimate from the craft now";

    public const string NoAirRow = "No air here";

    public const string NotYetSuffix = " (LATER)";

    public const string ShowInWorld = "SHOW IN WORLD";

    public const string ShowInMap = "SHOW IN MAP";

    public static readonly string[] MapChoices = { "OFF", "ORBIT", "PLAN" };

    public const string MapChoiceTip = "What the map view draws: nothing, the target orbit and the flown track, or those and the calculated plan.";

    public const string NoStageLeft = "No later stage lights an engine, so AutoStage has nothing to do.";

    public const string DropSpentNeedsAutoStage = "Acts only with AutoStage on.";

    public const string ArmsAtLiftOffDrawer = "A launch arms it at lift-off.";

    /// <summary>The line under BURN WITH while stock AUTO flies the node.</summary>
    public const string FlyWithHeld = "Stock AUTO flies this burn. Stop it to change this.";

    public const string NoPlanYet = "No plan yet";

    public static string NoPlanAnswer(bool arms) => arms ? "No plan yet. A press calculates it and arms." : "No plan yet. A press calculates it and launches.";

    /// <summary>Why the launch verb calculates first, from the reason of GuidanceWindow.ConvexPlanFlyable, or empty when the plan can fly.</summary>
    public static string PlanState(string why) => why switch
    {
        "" => "",
        "no plan calculated" => NoPlanYet,
        "the plan did not converge" => "The plan did not converge",
        _ when why.StartsWith(PlanTimeWhy, StringComparison.Ordinal) => PlanOld,
        _ => "Plan out of date: " + why,
    };

    public const string PlanTimeWhy = "it was planned for a lift-off";

    public const string PlanOld = "Plan is old. A press calculates it again and launches.";

    public const string PlanOldPlain = "Plan is old.";

    public const string PlanOldArms = "Plan is old. A press calculates it again and arms.";

    public static string BackupPlanState(string status) => status == PlanOld ? PlanOldPlain : status;

    #endregion

    #region Other places

    public const string CutoffCue = "Ascent done.";

    public const string AscentDoneCircular = "Ascent done. The orbit is circular.";

    public const string BurnDone = "Burn done";

    public const string PassesDone = "Passes done";

    public const string DrawerLink = "MORE >";

    public static string PassDone(int done, int next, string inTime)
        => string.Create(Inv, $"Pass {done} done. Pass {next} in {inTime}.");

    public static string PassLink(int next) => string.Create(Inv, $"PASS {next} >");

    public static string NextBurnIn(string inTime) => BurnDone + ". Next burn in " + inTime + ".";

    public static string Finished(string what, in SituationFacts f) => f.Where switch
    {
        CraftSituation.Orbit when double.IsFinite(f.PeKm) && double.IsFinite(f.ApKm) => what + ", " + Fmt.Orbit(f.PeKm, f.ApKm) + ".",
        CraftSituation.FallingAir or CraftSituation.FallingNoAir => what + ". Next: land.",
        _ => what + ".",
    };

    public static string FinishedBrief(string message)
    {
        if (message.StartsWith(BurnDone + ", ", StringComparison.Ordinal))
            return BurnDone + ".";
        return message.StartsWith(PassesDone + ", ", StringComparison.Ordinal) ? PassesDone + "." : message;
    }

    public static string Landed(string body) => body.Length > 0 ? "Landed, " + body + "." : "Landed.";

    public static string BurnCreated(double dvMs, string inTime) => "Burn created: " + Fmt.Dv(dvMs) + " in " + inTime + ".";

    public static string TransferCreated(double dvMs, string inTime) => "Transfer created: " + Fmt.Dv(dvMs) + " in " + inTime + ".";

    public const string CreatedNext = "Fly it, or plan the next burn.";

    public const string NextToolLink = "NEXT TOOL";

    public const string NextToolTip = "Keeps this burn and shows the tools again. The next burn starts from the orbit after this one.";

    public const string StockDrawsBurn = "The stock game draws the burn that was made.";

    public const string AlreadyCircular = "The orbit is already circular";

    public const string AlreadyCoplanar = "The orbits are already in one plane";

    public static string AtTargetApse(bool setPeriapsis) => setPeriapsis ? "Already at this periapsis" : "Already at this apoapsis";

    // The three answers of the convex launch banner.
    public const string BannerBackup = "FLY THE SIMPLE TURN";
    public const string BannerKeep = "KEEP WAITING FOR THE PLAN";
    public const string BannerHide = "HIDE";

    public const string AutoStageRowLabel = "AUTOSTAGE";
    public const string AutoStageHudNote = "The AUTOSTAGE button on the HUD needs KittenExtensions Continued.";
    public const string ArmedByGuidance = "Armed by AFC while it flies this craft.";

    /// <summary>The tooltip note of an AutoStage switch that guidance armed. Guidance keeps a player disarm for the rest of the flight.</summary>
    public const string ArmedByGuidanceTip = "AFC armed AutoStage for this flight. A click disarms it, and AFC keeps it off for the rest of the flight.";

    // The tooltips of the nudge buttons of the hover, each with what it moves.
    public const string NudgeNorth = "Moves the aim one NUDGE STEP to the north.";
    public const string NudgeSouth = "Moves the aim one NUDGE STEP to the south.";
    public const string NudgeEast = "Moves the aim one NUDGE STEP to the east.";
    public const string NudgeWest = "Moves the aim one NUDGE STEP to the west.";
    public const string NudgeZero = "Sets the aim back to zero: no drift, the normal sink rate.";
    public const string NudgeUp = "Sinks one NUDGE STEP slower, or climbs.";
    public const string NudgeDown = "Sinks one NUDGE STEP faster.";

    public static string HoverSetpoint(double east, double north, double up)
        => string.Create(Inv, $"E {east:F1}  N {north:F1}  Up {up:F1} m/s");

    #endregion
}

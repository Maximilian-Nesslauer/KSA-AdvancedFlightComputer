namespace AdvancedFlightComputer.Features.Ui.Logic;

internal readonly record struct TabFacts(CraftSituation Where, bool BurnPlanned = false, bool AutoStageArmed = false, bool Editor = false);

/// <summary>The texts of the console. They are pure data, so their length and case can be checked without the game.</summary>
internal static class PageText
{
    private static readonly System.Globalization.CultureInfo Inv = Fmt.Inv;

    public static string WayCard(LandWay way) => way switch
    {
        LandWay.FromOrbit => "Orbit to site. STOCK AUTO flies the deorbit burn, then AFC lands with TOUCHDOWN or G-FOLD.",
        LandWay.Boostback => "Spent booster to its site. AFC slews and aims the fall, ENTRY can take the glide.",
        LandWay.FromHere => "From the current position with ENTRY, TOUCHDOWN or G-FOLD.",
        _ => "AFC stops the drift and sinks to the ground. NUDGE moves the touchdown point.",
    };

    public static string ChooserCard(ConsolePage page) => page switch
    {
        ConsolePage.Launch => "Ascent from the pad into orbit.",
        ConsolePage.Land => "Landing from orbit or from a spent booster. LAND NOW starts from the current state.",
        ConsolePage.Orbit => "Orbit changes with a stock burn.",
        ConsolePage.Transfer => "Transfer to another body or vehicle.",
        ConsolePage.Rcs => "Planned burns and their owner: AFC, STOCK AUTO or MANUAL.",
        ConsolePage.Staging => "Staging conditions and delays.",
        _ => "GNC settings and preferences. Most flights need no change here.",
    };

    public static string TabNow(ConsolePage page, in TabFacts f)
    {
        if (f.Editor)
        {
            return page switch
            {
                ConsolePage.Transfer => "Editor: only the stock planner link works.",
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
            ConsolePage.Rcs => f.BurnPlanned ? "A burn is planned." : "No burn planned.",
            ConsolePage.Staging => f.AutoStageArmed ? "AutoStage is on." : "AutoStage is off.",
            _ => "",
        };
    }

    public static string TabCard(ConsolePage page, in TabFacts f)
    {
        string now = TabNow(page, in f);
        return now.Length > 0 ? ChooserCard(page) + " " + now : ChooserCard(page);
    }

    public static string WideCard(bool wide, bool panesFit, bool pageStaysNarrow = false)
        => (pageStaysNarrow ? "WIDE needs a picture. This page has none."
            : !panesFit ? "No room for two panes. WIDE only widens the page."
            : wide ? "Back to one column."
            : "Two panes: picture and result left, inputs and folds right. A page with no picture stays narrow.")
           + "\n" + TagEveryCraft;

    public const string FlightOnlyInEditor = "FLIGHT ONLY. EDITOR: STAGING, TUNE, STOCK PLANNER LINK ON TRANSFER.";

    public static string VerbWhen(GuidanceModeId mode) => mode switch
    {
        GuidanceModeId.AscentUpfg or GuidanceModeId.AscentGravityTurn => "Needs a set orbit.",
        GuidanceModeId.DeorbitLand => "Needs orbit and a set site.",
        GuidanceModeId.Boostback => "Needs a set stage site.",
        GuidanceModeId.EntryHere => "Needs a fall through air and a set site.",
        GuidanceModeId.SixDofHere => "Needs a descent and a set site.",
        GuidanceModeId.GfoldHere => "Needs a descent over an airless body and a set site.",
        GuidanceModeId.Hover => "Needs a position near the ground.",
        _ => "",
    };

    public static string CreateCard(string goal, bool passes)
        => "Creates the plan once " + goal + " is set. " + (passes ? "STOCK AUTO flies each pass. AFC adds the next pass when one ends." : "STOCK AUTO flies the burn after FLY BURN or AUTO on the BURN gauge.");

    public const string FlyStockTip = "Hands the burn to STOCK AUTO, like AUTO on the BURN gauge. It burns at the planned time.";

    public const string FlyRcsTip = "AFC burns it with the RCS thrusters. STOP ends it.";

    public const string KeptRule = "Changed values stay with the craft until a save loads.";

    // How long a value lives, as the last line of its tooltip.
    public const string TagEveryCraft = "Saved for every craft.";
    public const string TagSaveDefaults = "Saved with SAVE DEFAULTS.";
    public const string TagCraftDesign = "Saved per craft design.";
    public const string TagBurn = "Saved per burn.";
    public const string TagNotKept = "Not kept, off after every load.";
    public const string TagGameSession = "Kept until the game closes, on at every start.";

    /// <summary>The owner colour of the chip at the left of the head, from the who-flies text the chip shows.</summary>
    public static Owner HeadOwner(string whoFlies) => OwnerOfFlownBy(whoFlies) switch
    {
        Owner.None => Owner.Player,
        Owner owner => owner,
    };

    public const string FailureOkTip = "Dismisses the band. The message stays in the LOG.";

    public const string TriesAgain = "Retries.";

    public const string StockChipTip = "Function of the stock game. AFC only uses it.";

    public static string HeadOwnerCard(string who, bool pilotHolds = false) => who switch
    {
        FlownBy.Afc => "AFC holds the craft. STOP returns it to MANUAL.",
        FlownBy.StockAuto => "STOCK AUTO flies the burn. STOP returns the craft to MANUAL.",
        FlownBy.AfcPlansStockAuto => "AFC planned the burn and STOCK AUTO flies it. STOP ends both.",
        _ => pilotHolds ? "Manual control, no AFC hold. STOP ends the mode." : "Manual control.",
    };

    public static string FlownByCard(Owner owner) => owner switch
    {
        Owner.Afc => "After EXEC, AFC flies the craft. STOP at the top of the window ends it.",
        Owner.Stock => "After EXEC, STOCK AUTO flies the burn. STOP at the top of the window ends it.",
        Owner.AfcPlansStockFlies => "After EXEC, the stock flight plan holds the burn and the craft stays MANUAL. FLY BURN or AUTO on the BURN gauge hands it to STOCK AUTO.",
        Owner.StockThenAfc => "After EXEC, STOCK AUTO flies the deorbit burn, then AFC lands. STOP at the top of the window ends both.",
        _ => "",
    };

    public const string OnPressCard = "EXEC";

    public static string MoreLabel(int count, bool open)
        => "MORE" + (count > 0 ? " (" + count.ToString(Inv) + ")" : "") + (open ? " ^" : " v");

    public static string MoreCard(int messages, int otherCraft)
    {
        string count = (messages > 0 ? string.Create(Inv, $"{messages} new message{(messages == 1 ? "" : "s")}. ") : "")
            + (otherCraft > 0 ? string.Create(Inv, $"Other vehicles under AFC: {otherCraft}. ") : "");
        return count + "GNC switches, active modes, other vehicles, messages.";
    }

    public const string DrawerEmpty = "NONE ACTIVE. NO MESSAGES.";

    public const string BurnWarpLine = "TO BURN";

    public const string BurnWarpTip = "Warps to the burn time, as WARP on the stock BURN gauge. The button then reads STOP WARP.";

    public static string MayOffLine(bool steerOff, bool enginesOff)
        => steerOff && enginesOff ? "AFC MAY STEER + USE ENGINES OFF >"
            : steerOff ? "AFC MAY STEER OFF >"
            : enginesOff ? "AFC MAY USE ENGINES OFF >"
            : "";

    public const string MayOffTip = "Opens the GNC switches: AFC MAY STEER off ends the mode and cuts the engine, AFC MAY USE ENGINES off leaves engine and stages to MANUAL.";

    /// <summary>Touchdown guidance reads neither switch (GuidanceWindow.ApplyAutopilot).</summary>
    public static string SteerSwitchNote(bool touchdown, bool holdsCraft)
        => touchdown ? "Touchdown ignores this switch."
            : holdsCraft ? "Off ends the mode and cuts the engine."
            : "";

    public static string EnginesSwitchNote(bool touchdown, bool holdsCraft)
        => touchdown ? "Touchdown ignores this switch."
            : holdsCraft ? "Off: AFC keeps steering, engine and stages stay MANUAL. A landing from orbit ends."
            : "";

    public static string StockLine(bool hasNode, string inTime, double dvMs)
        => hasNode ? string.Create(Inv, $"NEXT BURN IN {inTime}, {dvMs:F0} m/s") : "NO BURN PLANNED";

    #region Choices

    /// <summary>The cards of the ASCENT choice, CAT-S / UPFG first, then the gravity turn.</summary>
    public static readonly string[] AscentCards =
    {
        "Solved climb or gravity turn, then UPFG to orbit.",
        "Pitch-over climb.",
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

    public static readonly string[] ClimbLabels = { "SOLVED", "GRAVITY TURN" };

    public static readonly string[] ClimbCards =
    {
        "Solved climb inside MAX Q, then UPFG to orbit.",
        "Vertical rise, turn at a set rate, then UPFG to orbit.",
    };

    public static string ClimbTip(string method) => method + " flies the ascent. Selects the climb up to the UPFG handover.";

    public static readonly string[] LaunchViewCards =
    {
        "Altitude against downrange. Shows the whole climb.",
        "Pitch against time. Shows hard turns.",
        "Dynamic pressure against time, as a share of the limit. Shows how close the plan comes to MAX Q.",
        "Throttle against time. Shows where the plan throttles back.",
    };

    public static readonly string[] NodeLabels = { "ASC NODE", "DESC NODE" };

    public static readonly string[] NodeCards =
    {
        "First turn heads north.",
        "First turn heads south.",
    };

    public static readonly string[] FlyWithLabels = { "ENGINE/RCS", "RCS ONLY" };

    public static readonly string[] FlyWithCards =
    {
        "STOCK AUTO burns with the engine. AFC uses RCS when no engine has propellant.",
        "AFC burns with the RCS thrusters.",
    };

    public const string FlyWithTip =
        "Selects engine or RCS for this burn. BY in the list shows who flies it now.\n\n" + TagBurn;

    public static readonly string[] AttitudeLabels = { "FREE", "HOLD", "ALIGN" };

    public static readonly string[] AttitudeCards =
    {
        "Selects HOLD or ALIGN, whichever uses less propellant.",
        "Holds the attitude and fires the thruster mix that points at the burn.",
        "Slews the strongest RCS axis onto the burn, then fires.",
    };

    public const string AttitudeTip = "Sets the attitude AFC holds during an RCS burn.\n\n" + TagBurn;

    /// <summary>The ATTITUDE description while ENGINE/RCS resolves to the engine.</summary>
    public const string AttitudeIfRcsNote = "Applies only if RCS takes the burn, when no engine has propellant at ignition.";

    public static readonly string[] AllocatorLabels = { "BY AXIS", "MIN FUEL" };

    public static readonly string[] AllocatorCards =
    {
        "Fires the thrusters in axis groups while attitude control holds the torque. MIN FUEL falls back to it.",
        "Solves a linear program for the thruster mix with the least propellant. Falls back to BY AXIS when none fits.",
    };

    public const string AllocatorTip = "Sets how AFC splits the burn among the RCS thrusters.\n\n" + TagBurn;

    public const string SolverTip = "Selects the landing mode.";

    #endregion

    #region Tooltips

    public static string SettingTooltip(string help, bool changed, string defaultText, string setBy)
    {
        string tail = setBy.Length > 0 ? setBy : changed && defaultText.Length > 0 ? "CHANGED " + defaultText + "." : "";
        return help.Length > 0 && tail.Length > 0 ? help + "\n" + tail : help + tail;
    }

    public const string TransferSelectTip = "Sets the porkchop color scale and moves the selection to the best cell of that kind. A click on the active segment returns to that cell.";

    public static readonly string[] TransferSelectCards =
    {
        "The cell with the lowest departure delta-v.",
        "A cell with low departure delta-v and low arrival speed at the destination. For a rendezvous.",
    };

    public static readonly string[] MoveLabels = { "^", "v", "<", ">" };

    public static readonly string[] MoveTips =
    {
        "Earlier departure: one row up the porkchop.",
        "Later departure: one row down the porkchop.",
        "Shorter flight time: one column left on the porkchop.",
        "Longer flight time: one column right on the porkchop.",
    };

    public const string MoveDisabled = "AFTER SOLVE";

    public const string MovePick = "MOVE PICK";

    public const string DepartAxis = "DEPART IN";

    public const string FlightTimeAxis = "FLIGHT TIME";

    public const string PorkchopNoTransfer = "No transfer here: impact or infeasible departure.";

    public const string CraftTargetChipTip = "Also sets the stock target.";

    #endregion

    #region Folds

    public const string HowItFlies = "GNC";

    public const string HowItFliesTip = "Finer settings of this page. The defaults need no change." + KeptRule;

    public static string PassesRowDetail(int passes, bool savingKnown = true)
        => passes <= 1 && savingKnown ? "SPLIT SAVES UNDER 1 m/s" : "";

    public static string PassesRowTip(bool savingKnown)
        => savingKnown ? "Number of burns. A split into more passes saves little on a short burn." : "Number of burns.";

    public static string SearchLabel(double minS, double maxS) => "SEARCH: TOF " + Fmt.Time(minS) + " TO " + Fmt.Time(maxS);

    public const string SearchTip = "Sets the quantity SOLVE searches and whether the destination also becomes the stock target.";

    public const string TimeFieldTip = "Time entry, such as 14m 24s, 1h 28m or 267d 6h. Enter applies it.";

    public const string Numbers = "PARAMETERS";

    public const string NumbersTip = "Inputs of the plan, read-only.";

    public const string Delays = "DELAYS";

    public const string DelaysTip = "Delay from the staging trigger to the firing of a part, per craft and as defaults. The part window also shows the delays of its stages.";

    public const string DelaysRule = "Delays count from the staging trigger. A decoupler with a shorter delay than the engine fires first.";

    public const string CraftDelaysDetail = "SECONDS, SAVED AT ONCE";

    public const string DefaultDelaysDetail = "SECONDS";

    public const string SaveDefaultsVerb = "SAVE DEFAULTS";

    public static string AllPartsLabel(int parts) => string.Create(Inv, $"ALL PARTS: {parts}");

    public const string AutoStageOffUntilLaunch = "A launch arms AutoStage at lift-off.";

    public const string SequenceCaption = "DV PER SEQUENCE (m/s)";

    public const string ResetFold = "RESET";

    public const string ResetVerb = "RESET";

    public static string ResetFoldQuestion(int changed)
        => changed == 1 ? "Reset the changed value of this fold to its default?" : string.Create(Inv, $"Reset the {changed} changed values of this fold to their defaults?");

    public static string ResetSetQuestion(int changed, string set)
        => changed == 1 ? $"Reset 1 {set} value to its default?" : string.Create(Inv, $"Reset {changed} {set} values to their defaults?");

    public const string TuningRule = "Tuning changes AFC flight behavior. Values are live and reset on save load.";

    public const string QAlphaNextLaunch = "MAX Q-ALPHA applies at the next launch.";

    public static string TuneSetsNeed(bool editor, bool hasCraft)
        => editor ? "IN FLIGHT ONLY"
            : hasCraft ? "GUIDANCE OFF. SET GUIDANCE ENABLED IN PREFERENCES."
            : "Tuning and diagnostics need a controlled craft.";

    public static string TuneSetCard(TuneSet set) => set switch
    {
        TuneSet.Afc => "Guidance switch, console key and opening behavior.",
        TuneSet.Ascent => "Ascent tuning. Check here on a missed orbit.",
        TuneSet.Recover => "Boostback tuning. Check here on a missed site.",
        TuneSet.Landing => "Landing tuning, one mode at a time.",
        _ => "Live AFC values and solver output. Read-only.",
    };

    public static readonly string[] TuneLandingCards =
    {
        "Flight from orbit to the start of the descent.",
        "Glide and landing burn of ENTRY.",
        "Powered descent of TOUCHDOWN.",
        "Powered descent of G-FOLD.",
        "Hover to touchdown.",
    };

    public static readonly string[] DiagViewCards =
    {
        "Solved plan and flown ascent.",
        "Boostback plan and predicted impact.",
        "Air model.",
        "Measured air against the air model.",
        "Committed G-FOLD plan.",
        "TOUCHDOWN live values and log.",
        "Stock flight plan table.",
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
        string what = message.ToUpperInvariant();
        if (!what.Contains("ITERATIONS", StringComparison.Ordinal))
            what += string.Create(Inv, $", {iterations} ITERATIONS");
        return what + string.Create(Inv, $" ({accepted} ACCEPTED), {wallSeconds:F1} s");
    }

    public static string PlanBurn(double seconds) => seconds.ToString("F1", Inv) + " s";

    public static string PlanIdeal(double idealDvMs) => Fmt.DvWhole(idealDvMs) + " IDEAL";

    public static string Stages(int count) => count == 1 ? "1 STAGE" : count.ToString(Inv) + " STAGES";

    public static string MirrorError(double share) => share < 1e-9 ? "0" : (share * 100.0).ToString("F4", Inv) + " %";

    public const string FlightPlanLink = "TUNE";

    public const string FlightPlanLinkTip = "Opens the stock table of this flight plan on TUNE > DIAG > FLIGHT PLAN. It shows the plan as it stands on this page.";

    public const string FlightPlanOfBurn = "Stock table of the burn on MANEUVER, with stock times.";

    public const string FlightPlanOfCraft = "Stock table of the flight plan, with stock times.";

    public const string FlightPlanBackTip = "Returns to the tool that made this plan.";

    public const string LanFromPositionTip = "Sets LAN to put the orbit plane over the current position.";

    public static string LimitsTip(bool hasAir)
        => hasAir ? "Limits dynamic pressure and acceleration of the climb. The plan uses MAX Q." : "Limits acceleration of the climb. MAX Q has no effect without atmosphere.";

    public static string BoostersTip(int stages)
        => "Sets a return site for a stage. The solved climb sizes the propellant the stage keeps for it." + (stages <= 0 ? " No stage can use it: no separating part carries a command pod." : "");

    public const string EngineDelaysTip = "Delay from the staging trigger to the ignition of each engine.";

    public const string DecouplerDelaysTip = "Delay from the staging trigger to the firing of each decoupler.";

    public const string EarlierMessagesTip = "Older messages of this flight.";

    public const string TouchdownLimitsTip = "Limits of the TOUCHDOWN plan. A change applies at the next solve.";

    public const string TouchdownTrackingTip = "Tracking of the TOUCHDOWN plan. A change applies at once.";

    public const string TouchdownSolverTip = "Solver settings of TOUCHDOWN. Change only when a solve takes too long or fails.";

    public const string GfoldLimitsTip = "Limits of the G-FOLD plan. A change applies at the next solve.";

    public const string GfoldTrackingTip = "Tracking of the G-FOLD plan. A change applies at once.";

    public const string GfoldSolverTip = "Solver settings of G-FOLD. Change only when a solve takes too long or fails.";

    public static string FoldHolds(string contents) => "HOLDS: " + contents + ".";

    public static string LimitsLabel(bool hasAir, double maxQKpa, bool gLimitOn, double gLimit)
    {
        string g = gLimitOn ? string.Create(Inv, $"G-LIMIT {gLimit:0.##} g") : "G-LIMIT OFF";
        return hasAir ? string.Create(Inv, $"LIMITS: MAX Q {maxQKpa:0.#} kPa, {g}") : "LIMITS: " + g;
    }

    public static string BoostersLabel(int stages, int withSite, double reserveKg)
    {
        string reserve = reserveKg > 0.0 ? "RESERVE " + Fmt.Tons(reserveKg) : "";
        if (stages <= 0)
            return reserve.Length > 0 ? "BOOSTERS: " + Fmt.Sentence(reserve) : "BOOSTERS: NONE";
        string sites = string.Create(Inv, $"{withSite}/{stages} WITH SITE");
        return "BOOSTERS: " + sites + (reserve.Length > 0 ? ", " + reserve : "");
    }

    public const string BoostersNone = "No stage can fly home: no separating part carries a command pod.";

    /// <summary>
    /// The RESERVE row of BOOSTERS: what the booster with a stage site keeps for its way home. The calculated climb sizes it, so there is nothing to set. Empty when no stage has a site and nothing was staged. <paramref name="warn"/> marks a reserve that does not arm.
    /// </summary>
    public static string BoosterReserve(bool hasSite, bool armed, bool staged, double reserveKg, double boostbackDvMs, double landingDvMs, string note, out bool warn)
    {
        warn = false;
        if (staged)
            return "STAGED. HANDOVER: BOOSTBACK.";
        if (!hasSite)
            return "";
        if (!armed)
        {
            warn = true;
            return "NOT ARMED: " + (note.Length > 0 ? note : "WAITING FOR STAGE MODEL");
        }
        if (!(reserveKg > 0.0))
            return double.IsFinite(boostbackDvMs) ? "NONE NEEDED" : ReserveWaitsForPlan;
        if (double.IsFinite(boostbackDvMs) && double.IsFinite(landingDvMs))
            return string.Create(Inv, $"{reserveKg / 1000.0:F2} t: {boostbackDvMs:F0} m/s BOOSTBACK + {landingDvMs:F0} m/s LANDING");
        return string.Create(Inv, $"{reserveKg / 1000.0:F2} t");
    }

    public const string ReserveWaitsForPlan = "SIZED AT CLIMB SOLVE";

    #endregion

    #region Pictures with nothing to show

    public static string LaunchNoPlan(bool onGround)
        => onGround ? "SOLVE DRAWS THE CLIMB." : "NO PLAN OR FLOWN TRACK.";

    public const string NoOrbit = "NO ORBIT";

    public const string PorkchopEmpty = "SOLVE fills this plot.";

    public const string PorkchopPassesFly = "No plot for these passes.";

    public const string NoBurnPlanned = "NO BURN PLANNED";

    public const string PassNotAddedYet = "AFC adds this pass when the previous pass ends. It has no options yet.";

    public const string BurnListTip = "Burns of the stock flight plan and the passes to come. A row selects the burn that BURN WITH sets, and options are kept per burn.";

    public const string RcsInUse = "IN USE";

    public const string RcsOnBoard = "RCS ON BOARD";

    public const string RcsNotEnough = "NOT ENOUGH";

    public const string RcsCannotTranslate = "No active thruster with propellant can translate.";

    public const string NoRcsThrusters = "No RCS thrusters.";

    public const string NoStages = "NO STAGES";

    public const string AllStagesFired = "ALL STAGES FIRED";

    public const string BoostbackNotYet = "No boostback yet. A stage with a site flies home after separation.";

    public const string BoostbackNoStage = "A stage flies home only when the separating part carries a command pod.";

    public const string NoSiteToDraw = "NO SITE PICKED";

    public const string EntryFirstPlan = "First solve takes a few seconds.";

    public const string EntryNoPlan = "NO PLAN. GLIDE CONTINUES, ENGINE OFF.";

    public const string HoverIdle = "The dial shows during the hover.";

    public static string SiteNonePicked(string body) => "SITE " + body + ": NONE";

    public const string SiteTypeRow = "DEFAULT/TYPE";

    public const string SiteTypeRowShort = "DEFAULT";

    public const string SiteDefaultTip = "Stored default coordinates, a point on Luna. With no site picked, the mode flies to them on this body.";

    public static string NoSiteAnswer(string verb, double latDeg, double lonDeg)
        => string.Create(Inv, $"NO SITE. {verb} TO {latDeg:F3}, {lonDeg:F3}.");

    public const string SiteFar = "SITE TOO FAR FOR FALL. PICK SITE BELOW.";

    /// <summary>The card of PICK SITE while a committed deorbit holds the site: a click then only says so (GuidanceWindow.RetargetLandingSite).</summary>
    public const string SiteLockedByDeorbit = "SITE LOCKED: DEORBIT ACTIVE. STOP TO EDIT.";

    /// <summary>The card of a value or switch that a committed deorbit reads, while it is disabled.</summary>
    public const string DeorbitLockedSetting = "LOCKED: DEORBIT ACTIVE. STOP TO EDIT.";

    public static string HoverVerdict(double minTwr, double fullTwr)
        => minTwr > 1.0 ? string.Create(Inv, $"TWR {minTwr:F2} AT MIN THROTTLE. NO HOVER.")
            : fullTwr < 1.0 ? string.Create(Inv, $"TWR {fullTwr:F2} AT FULL THROTTLE. NO HOVER.")
            : string.Create(Inv, $"TWR {minTwr:F2} AT MIN THROTTLE. HOVER POSSIBLE.");

    public static bool HoverThrustFails(double minTwr, double fullTwr) => minTwr > 1.0 || fullTwr < 1.0;

    public const string HoverFarAbove = "ABOVE HOVER RANGE. STOP HOVER + CUT ENGINE.";

    public const string OffScale = "OFF SCALE";

    private const double HoverFarMeters = 5000.0;

    public static bool HoverIsFar(double altitudeM) => altitudeM > HoverFarMeters;

    public const string HeightAboveSite = "ALT ABOVE SITE";

    public const string HeightAboveGround = "ALT ABOVE GROUND";

    public const string NewPlan = "NEW PLAN";

    public const string FlownPath = "FLOWN PROFILE";

    public const string SiteTick = "SITE";

    public const string SequenceWord = "SEQUENCE";

    public static string BrakingStart(double km) => "BRAKING START " + Fmt.KmFine(km);

    public static string VerticalGate(double meters) => "VERTICAL GATE " + Fmt.Distance(meters);

    public const string StagesAtBraking = "AFC reads the stages at the braking burn.";

    public static string PeakQ(double kpa) => "MAX Q " + kpa.ToString("F0", Inv) + " kPa";

    public static string QLimits(double maxQKpa, double maxQAlphaPaRad)
        => "Q LIMIT " + maxQKpa.ToString("F0", Inv) + " kPa, Q-ALPHA " + maxQAlphaPaRad.ToString("F0", Inv) + " Pa rad";

    public static string QLimitShort(double maxQKpa) => "Q LIMIT " + maxQKpa.ToString("F0", Inv) + " kPa";

    public static string MinThrottle(double percent) => "MIN " + percent.ToString("F0", Inv) + " %";

    public const string QTuneLink = "TUNE";

    public const string QTuneTip = "Opens TUNE > ASCENT for MAX Q-ALPHA. MAX Q is in LIMITS on this page.";

    public static string PlaneChange(double deg) => "PLANE CHANGE " + Fmt.Deg(Math.Abs(deg));

    public static string BurnAtNode(bool descending) => descending ? "BURN AT DESC NODE" : "BURN AT ASC NODE";

    public const string Escapes = "ESCAPE";

    public static string StoppedAt(double metersAboveSite) => "STOPPED AT " + Fmt.Distance(metersAboveSite);

    public const string ZoomHint = "Wheel zooms, middle click fits";

    public const string PassesTitle = "PASSES, TRACK FROM SITE";

    public const string NextPassTitle = "NEXT PASS, TRACK FROM SITE";

    public static string PassTrack(double km, double inSeconds) => Fmt.KmFine(km) + " IN " + Fmt.Time(inSeconds);

    public static string PassNext(bool alsoClosest) => alsoClosest ? "NEXT PASS IS CLOSEST" : "NEXT PASS";

    public static string PassClosest(double km, double inSeconds) => "CLOSEST " + PassTrack(km, inSeconds);

    public static string PassFlies(double km, double inSeconds) => "PASS IN " + Fmt.Time(inSeconds) + ", NOW " + Fmt.KmFine(km) + " OFF";

    public static string BrakingIn(double seconds) => "BRAKING T-" + Fmt.Time(seconds);

    public const string NoPassSolution = "NO PASS WITH SOLUTION";

    public const string CoastKey = "COAST";

    public const string BrakingKey = "BRAKING BURN";

    public const string CraftKey = "CRAFT";

    public const string CoastWord = "COAST";

    public const string ThrustKey = "THRUST";

    public static string PassWithin(double km) => "WITHIN " + Fmt.Km(km);

    public const string OtherPassesKey = "OTHER PASSES";

    public static string PorkchopRampName(bool arrival) => arrival ? "ARRIVAL SPEED" : "DEPARTURE DV";

    public const string PorkchopNoneKey = "NO TRANSFER";

    public static string PorkchopOnBoard(double onBoardMs)
        => double.IsFinite(onBoardMs) ? "HAS " + Fmt.DvWhole(onBoardMs) : "";

    public const string NoSketchClimb = "NO ORBIT UNTIL CUTOFF.";

    public const string NoSketchFall = "NO ORBIT: TRAJECTORY ENDS ON GROUND.";

    public const string PeInAir = "IN AIR";

    public const string PeBelowGround = "PE BELOW GROUND";

    public const string SketchPeBelowGround = "PE BELOW GROUND";

    public const string AirKey = "AIR";

    public const string TouchdownStandby = "TOUCHDOWN: STANDBY (ENTRY)";

    public const string TouchdownEngaging = "TOUCHDOWN: STARTING";

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
            return "Flies the GRAVITY TURN, not the solved plan." + autoStage;
        if (verb == "SOLVE + LAUNCH")
            return "Solves the plan and launches." + autoStage;
        if (verb == "SOLVE + ARM")
            return "Solves the plan and arms for the window." + autoStage;
        if (verb == "ARM")
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
                    ? (f.Solver.Length > 0 ? "Lands with " + f.Solver + ", with a stock burn first if needed." : "Lands, with a stock burn first if needed.")
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

    public const string ClimbFliesCard = "AFC flies the climb. AutoStage separates the stages.";

    public const string MayOffLaunch = "AFC MAY STEER OR AFC MAY USE ENGINES OFF. LAUNCH STARTS GUIDANCE ONLY.";

    public static string TakeOverOutcome(GuidanceModeId mode, string from)
    {
        string running = InSentence(from);
        return mode switch
        {
            GuidanceModeId.GfoldHere or GuidanceModeId.SixDofHere or GuidanceModeId.EntryHere or GuidanceModeId.Hover or GuidanceModeId.Boostback or GuidanceModeId.DeorbitLand
                => "Replaces " + running + " with " + SituationVerbs.VerbName(mode) + ".",
            GuidanceModeId.AscentUpfg or GuidanceModeId.AscentGravityTurn => "Replaces " + running + " with LAUNCH.",
            _ => "Ends " + running + ".",
        };
    }

    public static string InSentence(string text)
        => text.Length > 1 && char.IsUpper(text[0]) && char.IsLower(text[1])
            ? char.ToLowerInvariant(text[0]) + text.Substring(1)
            : text;

    public static bool FailureOf(string what, GuidanceModeId mode)
    {
        if (what.StartsWith("GUIDANCE FAULT", StringComparison.Ordinal))
            return true;
        string word = mode switch
        {
            GuidanceModeId.GfoldHere => "G-FOLD",
            GuidanceModeId.SixDofHere => "TOUCHDOWN",
            GuidanceModeId.EntryHere => "ENTRY",
            GuidanceModeId.Hover => "HOVER",
            GuidanceModeId.Boostback => "BOOSTBACK",
            GuidanceModeId.DeorbitLand => "LANDING",
            GuidanceModeId.AscentUpfg or GuidanceModeId.AscentGravityTurn => "ASCENT",
            _ => "",
        };
        return word.Length > 0 && what.Contains(word, StringComparison.OrdinalIgnoreCase);
    }

    public static string BurnAnswer(double dvMs, string when, double budgetMs, bool chained = false)
    {
        string what = Fmt.Dv(dvMs) + (when.Length > 0 ? " " + when : "");
        if (!double.IsFinite(budgetMs) || budgetMs <= 0.0)
            return what;
        double percent = dvMs / budgetMs * 100.0;
        string share = dvMs > 0.0 && percent < 1.0 ? "UNDER 1 %" : string.Create(Inv, $"{percent:F0} %");
        return string.Create(Inv, $"{what}, {share} OF ~{budgetMs:F0} m/s{(chained ? " LEFT" : "")}");
    }

    public static string PassesOutcome(int passes)
        => string.Create(Inv, $"Adds pass 1 of {passes}. AFC adds each next pass.");

    public const string CorrectionOutcome = "Adds a correction burn.";

    public const string SaveDefaultsOutcome = "Saves the delays as defaults for every new craft.";

    public static string ResetSetOutcome(int changed)
        => changed == 1 ? "Resets 1 changed value to its default."
            : string.Create(Inv, $"Resets {changed} changed values to their defaults.");

    public const string DeleteNodeTip = "Removes the burn from the stock flight plan. Plan it again on MANEUVER or TRANSFER.";
    public const string DeleteCreatedTip = "Removes the created burn from the stock flight plan.";

    public const string DeletePassTip = "Removes the pass from the stock flight plan and ends the passes. Plan them again on the page that made them.";

    public static string DeletePassLine(int pass, int total)
        => pass < total ? string.Create(Inv, $"DELETE PASS {pass} also ends the passes after it.") : "";

    public const string DeleteEndsLanding = "DELETE BURN also stops the landing.";

    public const string DeleteDeorbitTip = "Removes the burn from the stock flight plan and stops the landing.";

    public const string OutlineStillWorks = "Does not fit now. The button still acts.";

    public const string OutlinePressEnds = "Does not fit now. The button ends the mode.";

    public const string CalculateOutcome = "Solves every departure time and flight time.";

    public const string CalculateTip = "Needs a set destination. RE-SOLVE under the picture runs it again.";

    public static string FuelShort(double neededMs, double onBoardMs, bool chained = false)
        => neededMs > onBoardMs ? string.Create(Inv, $"NEEDS {neededMs:F0} m/s, {(chained ? "LAST BURN LEAVES" : "HAS")} ~{onBoardMs:F0}") : "";

    public const string BurnAfterImpact = "BURN AFTER IMPACT";

    public static string NoDestination(string body)
        => body.Length > 0
            ? "No destination from " + body + ". TRANSFER lists other craft and bodies with the same parent."
            : "No destination from here.";

    public static string NextLabel(ConsolePage page, LandWay? way)
        => (way is LandWay w ? LandWays.Label(w) : ConsolePages.Label(page)) + " >";

    public const string FitsNowNote = "Dot: mode fits the current situation.";

    public const string PageRunsNote = "Dot: something on this page is active.";

    public const string HoverNoEngine = "NO HOVER: NO LIT LIQUID ENGINE.";

    public const string NoGimbalFact = "NO GIMBAL: TOUCHDOWN CANNOT FLY.";

    public static string OverPoweredFact(double floor) => string.Create(Inv, $"FLOOR TOO HIGH: NO DESCENT ABOVE {floor:F2}.");

    public static string LandingStopped(bool engineWasOff, bool burnDeleted)
        => "STOPPED. " + (engineWasOff ? "ENGINE OFF" : "ENGINE CUT") + (burnDeleted ? ", BURN DELETED." : ".");

    public static string PickRowTip(LandSolver solver) => "Selects " + LandSolvers.Label(solver) + " in the table.";

    public const string NoLiftModel = "NO LIFT DATA. GLIDE HOLDS RETROGRADE.";

    public const string NoLiftAirframe = "NO USABLE LIFT. GLIDE HOLDS RETROGRADE.";

    public const string LegsStowed = "LEGS STOWED. DEPLOY IN EACH LEG WINDOW.";

    public static string LegsStowedCard(double deployedM, double nowM)
        => string.Create(Inv, $"The plan assumes the legs down: COM ABOVE LEGS is {deployedM:F1} m. With the legs as they are, it is {nowM:F1} m.");

    public static string ApseMargin(bool setPeriapsis, double marginKm)
        => string.Create(Inv, $"Target at least {marginKm:F0} km {(setPeriapsis ? "below AP" : "above PE")}");

    public const string AtTargetInclination = "AT TARGET INCLINATION";

    public static string NodeChoice(bool descending, double dvMs, double inS)
        => (descending ? "DESC " : "ASC ") + NodeCost(dvMs, inS);

    public static string NodeLine(double ascendingMs, double ascendingS, double descendingMs, double descendingS)
        => "ASC NODE " + NodeCost(ascendingMs, ascendingS) + "\nDESC NODE " + NodeCost(descendingMs, descendingS);

    private static string NodeCost(double dvMs, double inS)
        => double.IsFinite(dvMs) && double.IsFinite(inS) ? Fmt.DvWhole(dvMs) + ", " + Fmt.TimeShort(inS) : "NONE";

    /// <summary>The refusal that StartTerminalHover reads, in the words of the footer.</summary>
    public const string HoverNoThrust = "NO HOVER (THRUST)";

    public const string PickAnotherWay = "SELECT OTHER TAB.";

    public static string HoverProblemLine(string problem) => problem == HoverNoThrust ? PickAnotherWay : problem;

    public const string PlanCalculating = "SOLVING";

    public static string PlanProgress(string progress, double elapsedS)
        => string.Create(Inv, $"{progress.Replace(": ", ", ")} ({elapsedS:F0} s)");

    public static string PlanProgressShort(string progress, double elapsedS)
    {
        int colon = progress.IndexOf(':');
        string what = colon > 0 ? progress.Substring(0, colon) : progress;
        return what.Length > 0 ? string.Create(Inv, $"SOLVING, {what} ({elapsedS:F0} s)") : string.Create(Inv, $"SOLVING ({elapsedS:F0} s)");
    }

    public static string LaunchWaitVerb(bool atWindow) => atWindow ? "ARMED" : "LAUNCH ARMED";

    public static string LaunchWaitTip(bool atWindow)
        => atWindow ? "Arms for the launch window when the plan is ready." : "Launches when the plan is ready.";

    public static string WhenReadyVerb(bool atWindow) => atWindow ? "ARM" : "LAUNCH";

    public const string NoLaunchFollows = "NO LAUNCH QUEUED";

    public static string CalculatingVerbTip(bool atWindow)
        => atWindow ? "Arms for the window when the plan is ready." : "Launches when the plan is ready.";

    public const string CancelVerb = "CANCEL";

    public const string CancelWaitTip = "Cancels the solve and the launch waiting on it.";

    public const string CancelTransferTip = "Cancels the solve.";

    public static string FlownAgainstGoal(double peKm, double apKm, double goalPeKm, double goalApKm)
        => (peKm < 0.0 ? "FLOWN TO AP " + Fmt.Km(apKm) : FlownWord + " " + Fmt.Orbit(peKm, apKm)) + ", GOAL " + Fmt.Orbit(goalPeKm, goalApKm);

    public const string FlownWord = "FLOWN";

    public static bool GoalReached(double peKm, double apKm, double goalPeKm, double goalApKm)
        => double.IsFinite(peKm) && double.IsFinite(apKm) && peKm >= 0.0
            && Math.Abs(peKm - goalPeKm) <= Math.Max(5.0, 0.02 * Math.Abs(goalPeKm))
            && Math.Abs(apKm - goalApKm) <= Math.Max(5.0, 0.02 * Math.Abs(goalApKm));

    public const string PlanStarting = "STARTING";

    /// <summary>The WINDOW row: the time to the ignition and its node. <paramref name="planShiftS"/> is how far the calculated climb moved its own lift-off from the window, s, positive for later; the countdown then runs to that lift-off.</summary>
    public static string WindowCountdown(double waitS, bool descending, double leadS, double planShiftS = 0.0)
    {
        string node = descending ? ", DESC NODE, " : ", ASC NODE, ";
        if (planShiftS == 0.0)
            return "T-" + Fmt.Time(waitS) + node + string.Create(Inv, $"{leadS:F0} s LEAD");
        return "T-" + Fmt.Time(waitS + planShiftS) + node
            + string.Create(Inv, $"PLAN TIME, {Math.Abs(planShiftS):F0} s {(planShiftS > 0.0 ? "AFTER" : "BEFORE")} {leadS:F0} s LEAD");
    }

    /// <summary>The sentence under an armed launch. <paramref name="planShiftS"/> is how far the calculated climb moved its own lift-off from the window, s, positive for later. The launch then goes at that lift-off, which the WINDOW row and the head also count to.</summary>
    public static string ArmedLaunch(bool warping, bool autoLaunch, double planShiftS = 0.0)
    {
        // Below half a second the shift would print as "0 s", so the sentence names the window.
        if (!double.IsFinite(planShiftS) || Math.Abs(planShiftS) < 0.5)
            return warping ? "WARPING TO WINDOW"
                : autoLaunch ? "ARMED. LAUNCH AT WINDOW."
                : "ARMED. WARP TO WINDOW, LAUNCH FOLLOWS.";
        string shift = PlanLaunchShift(planShiftS);
        return warping ? "WARPING TO PLANNED LIFTOFF, " + shift + "."
            : autoLaunch ? "ARMED. LAUNCH AT PLANNED LIFTOFF, " + shift + "."
            : "ARMED. WARP TO PLANNED LIFTOFF, " + shift + ", LAUNCH FOLLOWS.";
    }

    /// <summary>The name of the switch that lets the calculated climb choose its timing: with a target it chooses the launch time, with none the LAN.</summary>
    public static string OptimiseTimingLabel(bool hasTarget) => hasTarget ? "SOLVE LIFTOFF" : "SOLVE LAN";

    /// <summary>The DIAG line of a booster reserve the plan sized: what the booster keeps, for what, and when it separates.</summary>
    public static string PlanBooster(double reserveKg, double boostbackDvMs, double landingDvMs, double separationS)
        => string.Create(Inv, $"RESERVE {reserveKg / 1000.0:F2} t: {boostbackDvMs:F0} m/s BOOSTBACK + {landingDvMs:F0} m/s LANDING, SEPARATION T+{separationS:F0} s");

    /// <summary>The DIAG line of a lift-off the plan moved from the launch window, s, positive for later.</summary>
    public static string PlanLaunchShift(double delayS)
        => string.Create(Inv, $"{Math.Abs(delayS):F0} s {(delayS > 0.0 ? "AFTER" : "BEFORE")} WINDOW");

    /// <summary>The DIAG line of a LAN the plan chose itself.</summary>
    public static string PlanLanChosen(double lanDeg) => string.Create(Inv, $"{lanDeg:F2} deg, SOLVED FOR PAD");

    public const string NoCrossing = "INC BELOW SITE LATITUDE";

    public const string ReachLabel = "REACH";

    /// <summary>The REACH row: the plane change the launch flies, its price, and, when the inclination is the cause, the band a direct launch reaches.</summary>
    public static string Reach(in ReachFacts f)
    {
        string cost = string.Create(Inv, $"{f.PlaneChangeDeg:F1} deg PLANE CHANGE, ~{f.PlaneChangeMs:F0} m/s");
        if (!f.OutOfReach)
            return cost + ". FROM POSITION PUTS THE PLANE OVER THE SITE";
        double low = LaunchReach.LowestDirectIncDeg(f.SiteLatDeg);
        return cost + string.Create(Inv, $". DIRECT: INC {low:F1} TO {180.0 - low:F1}");
    }

    public static string ReachTip(in ReachFacts f, bool gravityTurn) => f.OutOfReach
        ? string.Create(Inv, $"A launch from latitude {Math.Abs(f.SiteLatDeg):F1} deg reaches only planes it lies in, so its inclination is at least {LaunchReach.LowestDirectIncDeg(f.SiteLatDeg):F1} deg. ")
          + (gravityTurn
              ? "The gravity turn cannot fly a plane change, so LAUNCH refuses this inclination."
              : "The ascent turns the rest of the plane with engine thrust. The price is that turn at the apoapsis of the target orbit, the cheapest single burn. A dog-leg in the climb costs about the same.")
        : "The body has turned the site away from the plane since its LAN was set. FROM POSITION on the PLANE row sets the plane over the site again.";

    /// <summary>Why LAUNCH refuses a gravity turn into a plane the site does not lie in, and the nearest inclination it can fly. A target sets the plane, so there is no inclination to offer for it.</summary>
    public static string ReachRefusal(double siteLatDeg, double incDeg, bool targetPlane)
    {
        double low = LaunchReach.LowestDirectIncDeg(siteLatDeg);
        string head = string.Create(Inv, $"REFUSED: {(targetPlane ? "TARGET " : "")}INC {incDeg:F1} deg OUT OF REACH FROM LATITUDE {Math.Abs(siteLatDeg):F1} deg (DIRECT: INC {low:F1} TO {180.0 - low:F1}). THE GRAVITY TURN CANNOT FLY A PLANE CHANGE.");
        return targetPlane ? head : head + string.Create(Inv, $" NEAREST: INC {LaunchReach.NearestDirectIncDeg(siteLatDeg, incDeg):F1} deg.");
    }

    /// <summary>The refusal of a gravity turn into a target's plane off its launch window, with the wait to the next window, or NaN on a body that does not turn.</summary>
    public static string WindowRefusal(double offPlaneDeg, double nextWindowS, bool descending, bool planeReachesSite = true)
    {
        string head = string.Create(Inv, $"REFUSED: SITE {offPlaneDeg:F1} deg OUT OF THE TARGET PLANE. THE GRAVITY TURN CANNOT FLY A PLANE CHANGE.");
        if (!planeReachesSite)
            return head + " THE PLANE NEVER PASSES OVER THE SITE, SO NO WINDOW COMES.";
        return double.IsFinite(nextWindowS)
            ? head + " NEXT WINDOW: T-" + Fmt.Time(nextWindowS) + (descending ? ", DESC NODE" : ", ASC NODE") + ". LAUNCH ARMS FOR IT."
            : head + " THE BODY DOES NOT TURN, SO NO WINDOW COMES.";
    }

    public const string LanOverPad = "OVER THE PAD";

    public const string LanOverPadTip = "The gravity turn flies the plane over the pad, at the node of the WINDOW row. LAN follows the pad until LAUNCH.";

    public const string LaunchNowTip = "Launches at once and skips the window.";

    public const string LaunchNowGravityTurnTip = "The gravity turn cannot skip the window, because it cannot fly a plane change. NOW refuses and the countdown stays.";

    /// <summary>
    /// The status after an ascent whose stages have no propellant left before its cutoff: what the craft flies now. A periapsis under the surface is suborbital; one under <paramref name="airTopKm"/>, the top of the atmosphere, is an orbit that the air lowers.
    /// </summary>
    public static string AscentOutOfPropellant(bool bound, double peKm, double apKm, double incDeg, double airTopKm)
    {
        string where = !bound ? "ESCAPE TRAJECTORY"
            : peKm < 0.0 ? "SUBORBITAL, AP " + Fmt.Km(apKm)
            : "ORBIT " + Fmt.Orbit(peKm, apKm) + ", INC " + Fmt.Deg(incDeg) + (peKm < airTopKm ? ", PE IN THE AIR" : "");
        return "ASCENT ENDED: OUT OF PROPELLANT BEFORE CUTOFF. " + where + ". MANUAL.";
    }

    public const string TwrLabel = "TWR";

    /// <summary>The TWR row: a first stage that cannot lift the craft off the pad, or that falls under its weight before the first stage is spent.</summary>
    public static string Twr(in LaunchTwrFacts f) => f.CannotLiftOff
        ? string.Create(Inv, $"{f.LiftoffTwr:F2} AT LIFT-OFF. THE CRAFT DOES NOT LEAVE THE PAD")
        : string.Create(Inv, $"{f.LowestTwr:F2} WHEN STAGE {f.LowestStage + 1} STARTS, STILL IN THE FIRST STAGE");

    public static string TwrTip(in LaunchTwrFacts f, bool gravityTurn) => f.CannotLiftOff
        ? "The thrust of the first stage at the air pressure of the pad is less than the weight of the craft."
        : "A first stage that burns on after a drop, such as a core after its boosters, has less thrust than weight from that stage on, at the air pressure of the pad. Under 1 the craft loses vertical speed until it is lighter. "
          + (gravityTurn
              ? "The gravity turn has no plan for that and can hold the craft level until it falls. The solved climb plans the climb for the thrust it has."
              : "The solved climb plans for it; a plan that does not converge falls back to the gravity turn, which can hold the craft level until it falls.");

    public static string ReachFix(double incDeg) => string.Create(Inv, $"USE INC {incDeg:F1} deg");

    public const string ReachFixTip = "Sets the nearest inclination a direct launch reaches, and the plane over the site.";

    public static string TargetOrbit(double peKm, double apKm, double incDeg) => Fmt.Orbit(peKm, apKm) + ", INC " + Fmt.Deg(incDeg);

    public static string LaunchAnswer(string why, string stored, bool planUsable, bool flyConvex, double massToOrbitKg, int stagesPlanned, int stagesAvailable)
    {
        bool failed = !planUsable && stored.Length > 0 && !IsPlanStatusTransient(stored);
        if (failed)
            return stored;
        if (why.Length > 0)
            return PlanState(why);
        return flyConvex
            ? string.Create(Inv, $"{PlanReadyWord}: {Fmt.Tons(massToOrbitKg)} TO ORBIT, {stagesPlanned}/{stagesAvailable} STAGES")
            : "READY: GRAVITY TURN";
    }

    public const string PlanReadyWord = "PLAN READY";

    private static bool IsPlanStatusTransient(string stored) => stored is "STARTING..." or "SOLVING...";

    public static string BurnFuelLine(string payer, double propellantKg, double shortMs)
        => shortMs <= 0.5 && payer.Length > 0 ? "PAID BY " + payer + ", ~" + Fmt.Mass(propellantKg) : "";

    public static string PassesPaidBy(string payer) => payer.Length > 0 ? "PAID BY " + payer : "";

    public static string PassesPlan(string goal, int passes)
        => goal.Length > 0 ? string.Create(Inv, $"{goal} IN {passes} PASSES") : "";

    public static string PassesGoal(string goal, int passes)
        => goal.Length > 0 ? "GOAL: " + PassesPlan(goal, passes) + "." : "";

    public static string BurnGoal(double nowPeKm, double nowApKm, double afterPeKm, double afterApKm, bool closes)
    {
        if (!closes)
            return Escapes;
        double pe = Math.Abs(afterPeKm - nowPeKm), ap = Math.Abs(afterApKm - nowApKm);
        return pe < 1.0 && ap < 1.0 ? "ORBIT " + Fmt.Orbit(afterPeKm, afterApKm)
            : ap >= pe ? "AP " + Fmt.Km(afterApKm)
            : "PE " + Fmt.Km(afterPeKm);
    }

    public const string HowItFliesDefaults = HowItFlies + ": ALL DEFAULTS";

    public const string GoalOrbitReached = "GOAL ORBIT REACHED. SELECT A TOOL FOR ANOTHER ORBIT.";

    public const string PassFuelEnds = "OUT OF FUEL";

    public static string PassesShort(double shortMs) => string.Create(Inv, $"Fuel ~{shortMs:F0} m/s short of the goal.");

    public const string PickSiteCue = "PICK SITE: CLICK MAP OR WORLD. RMB CANCELS.";

    public const string PickSite = "PICK SITE";

    public const string PlanTip = "Page setup. The line inside states what applies now.";

    public static string PassWaits(bool rcs) => rcs ? "EXEC: AFC RCS burns it." : "EXEC: STOCK AUTO burns it.";

    public static string FlyPassTip(bool rcs)
        => (rcs ? "AFC burns the pass with the RCS thrusters." : "Hands the pass to STOCK AUTO, like AUTO on the BURN gauge.")
            + " It burns at the planned time. AFC adds the next pass when this one ends.";

    /// <summary><paramref name="marginMs"/> is the delta-v on board less what guidance still needs, NaN while no solve says that.</summary>
    public static HeadValue[] AscentNumbers(bool bound, double apKm, double apGoalKm, double peKm, double peGoalKm, double marginMs = double.NaN)
    {
        var values = new List<HeadValue>(3);
        if (bound)
            values.Add(new HeadValue(string.Create(Inv, $"AP {apKm:F0} km, TARGET {apGoalKm:F0}"), 1, string.Create(Inv, $"AP {apKm:F0} km")));
        bool peLow = peKm < 0.0;
        var margin = new HeadValue(string.Create(Inv, $"MARGIN {marginMs:+0;-0} m/s"), peLow ? 0 : 2, Tone: marginMs <= 0.0 ? PlanTone.Warn : PlanTone.Plain);
        if (peLow && double.IsFinite(marginMs))
            values.Add(margin);
        values.Add(peLow
            ? new HeadValue(PeBelowGround, 3)
            : new HeadValue(string.Create(Inv, $"PE {peKm:F0} km, TARGET {peGoalKm:F0}"), 0, string.Create(Inv, $"PE {peKm:F0} km")));
        if (!peLow && double.IsFinite(marginMs))
            values.Add(margin);
        return values.ToArray();
    }

    /// <summary>The numbers of a boostback glide in the head: the miss, its two parts where the row has room, and the time until entry guidance takes over, NaN when it does not.</summary>
    public static HeadValue[] GlideNumbers(double downM, double crossM, double entryInS = double.NaN)
    {
        double missKm = Math.Sqrt(downM * downM + crossM * crossM) / 1000.0;
        string miss = "MISS " + Fmt.KmFine(missKm);
        string parts = string.Create(Inv, $"{Math.Abs(downM) / 1000.0:F1} {(downM < 0.0 ? "SHORT" : "LONG")}, {Math.Abs(crossM) / 1000.0:F1} {(crossM < 0.0 ? "RIGHT" : "LEFT")}");
        var head = new HeadValue(miss + ", " + parts, 0, miss, PlanWords.MissTone(missKm, flying: true));
        return double.IsFinite(entryInS) && entryInS > 0.0
            ? [head, new HeadValue("ENTRY T-" + Fmt.Time(entryInS))]
            : [head];
    }

    public static string AscentToGo(double speedMs, double distanceKm) => string.Create(Inv, $"SPEED TO GAIN {speedMs:F0} m/s, DISTANCE TO GO {distanceKm:F0} km");

    public static string BrakingToGo(double speedMs) => string.Create(Inv, $"DV TO GO {speedMs:F0} m/s.");

    public static string ImpactCard(double seconds) => "IMPACT T-" + Fmt.Time(seconds) + " with no further burn.";

    public const string GlideNumbersCard = "Long and short are downrange, left and right crossrange, from the site to the impact point.";

    public const string HoverNumbersCard = "ALT is the height of the legs above the ground under the craft.";

    public const string OnSite = "ON SITE";

    public const string LandsHereNow = "LANDS HERE";

    public const string ImpactWord = "IMPACT";

    public const string ImpactOnSite = "IMPACT ON SITE";

    public static string ImpactOff(double km) => "IMPACT " + Fmt.KmFine(km) + " OFF";

    public const string LandsIn = "LANDS T-";

    /// <summary>The time to go of a mode beside its word. A word that ends in T- (IMPACT T-) takes the time with no space, as in IMPACT T-42 s.</summary>
    public static string Tgo(string label, string time) => label.EndsWith('-') ? label + time : label + " " + time;

    /// <summary>The name of a burn that stock AUTO flies in the head, with the step it stands on.</summary>
    public static string StockBurnName(bool burning) => burning ? "BURN ACTIVE" : "BURN ARMED";

    /// <summary>The time beside the delta-v of a burn that stock AUTO flies: the wait to its ignition, until an engine burns.</summary>
    public static string StockBurnWhen(bool burning, string toIgnition) => burning ? "ACTIVE" : "IN " + toIgnition;

    public static string StepOf(int step, int steps) => string.Create(Inv, $"STEP {step}/{steps}");

    public const string OpenLandNowTip = "Opens LAND NOW to start a landing mode from the current position.";

    public const string TakeOverTouchdown = "TAKE OVER: TOUCHDOWN";

    public const string TakeOverTouchdownTip = "Ends entry guidance and starts touchdown guidance. The engine stays off until touchdown.";

    public static string RefusedAnswer(string fitText, string line)
    {
        const string word = "REFUSED: ";
        string why = fitText.Length > 0 ? fitText : line;
        int at = why.IndexOf(word, StringComparison.OrdinalIgnoreCase);
        return "REFUSED: " + InSentence(at >= 0 ? why.Substring(at + word.Length) : why);
    }

    public const string PickArmed = "CANCEL PICK";

    public const string PickSiteTip = "The next click on the map or in the world moves the landing site. Right click cancels.";

    public const string PickArmedTip = "Waits for a click on the map or in the world to move the landing site. Right click or this button cancels.";

    public const string NoThrustToJudge = "NO LIT ENGINE. NO TWR.";

    public static string HandOversLabel(bool entryTakesGlide, double glideAtS, bool touchdownTakesBurn, double burnLastS)
    {
        string glide = entryTakesGlide ? string.Create(Inv, $"ENTRY AT IMPACT T-{glideAtS:F0} s") : "ENTRY OFF";
        string burn = touchdownTakesBurn ? string.Create(Inv, $"TOUCHDOWN LAST {burnLastS:F0} s") : "TOUCHDOWN OFF";
        return "HANDOVER: " + glide + ", " + burn;
    }

    public static string HoverHandOverLabel(double fromM, bool hoverCanFly, bool known = true)
        => !hoverCanFly ? "HANDOVER: NO HOVER (THRUST)"
            : known ? string.Create(Inv, $"HANDOVER: HOVER FROM {fromM:F0} m")
            : string.Create(Inv, $"HANDOVER: HOVER FROM {fromM:F0} m (IF HOVER)");

    public const string HandOversTip = "Handover of the boostback glide, and handover of the end of the landing burn.";

    public const string HoverHandOverTip = "Height where G-FOLD hands over to the hover, if the craft can hover there.";

    public const string NoHandOversLabel = "HANDOVER: NONE";

    public static string NoHandOversTip(LandSolver solver) => solver == LandSolver.Entry
        ? "No air, no ENTRY plan. Nothing to hand over."
        : LandSolvers.Label(solver) + " flies to the ground. Nothing to hand over.";

    public const string BoostbackTakesCraft = "Takes the craft, also on the pad. No stage can fly home.";

    public const string StopWord = "STOP";

    public const string HoverNotUsed = "Not used: thrust at minimum throttle is above the weight.";

    public const string NumpadHint = "The numpad keys nudge the aim with TUNE > PREFERENCES > NUMPAD NUDGES THE HOVER on.";

    public const string PressPlansNext = "Flies the next pass.";

    public const string PassesFar = "NO PASS NEAR SITE. MOVE SITE OR PLANE.";

    public static string QRelaxedNote(double askedKpa, double usedKpa)
        => string.Create(Inv, $"MAX Q {askedKpa:0.#} kPa OUT OF REACH. PLAN USES {usedKpa:0.#} kPa.");

    public static string FlightTimeTip(string hohmannTime) => "The range SOLVE searches. A Hohmann transfer takes " + hohmannTime + ".";

    public static string EstimateBasis(bool loadedBurn) => loadedBurn ? "ESTIMATE: LOADED BURN" : "ESTIMATE: CRAFT NOW";

    public const string NoAirRow = "NO AIR";

    public const string NotYetSuffix = " (LATER)";

    public const string ShowInWorld = "SHOW IN WORLD";

    public const string ShowInMap = "SHOW IN MAP";

    public static readonly string[] MapChoices = { "OFF", "ORBIT", "PLAN" };

    public const string MapChoiceTip = "Map view content: OFF, ORBIT (target orbit and flown track) or PLAN (both and the solved plan).";

    public const string NoStageLeft = "No later stage lights an engine. AutoStage is idle.";

    public const string DropSpentNeedsAutoStage = "Acts only with AutoStage on.";

    public const string ArmsAtLiftOffDrawer = "A launch arms it at liftoff.";

    /// <summary>The line under BURN WITH while stock AUTO flies the node.</summary>
    public const string FlyWithHeld = "LOCKED: STOCK AUTO BURN. STOP TO EDIT.";

    public const string NoPlanYet = "NO PLAN";

    public static string NoPlanAnswer(bool arms) => arms ? "NO PLAN. SOLVE + ARM." : "NO PLAN. SOLVE + LAUNCH.";

    /// <summary>Why the launch verb calculates first, from the reason of GuidanceWindow.ConvexPlanFlyable, or empty when the plan can fly.</summary>
    public static string PlanState(string why) => why switch
    {
        "" => "",
        "NO PLAN" => NoPlanYet,
        "NO SOLUTION" => "NO SOLUTION",
        _ when why.StartsWith(PlanTimeWhy, StringComparison.Ordinal) => PlanOld,
        _ => "PLAN OUT OF DATE: " + why,
    };

    public const string PlanTimeWhy = "PLANNED FOR LIFTOFF";

    public const string PlanOld = "PLAN OLD. SOLVE + LAUNCH.";

    public const string PlanOldPlain = "PLAN OLD";

    public const string PlanOldArms = "PLAN OLD. SOLVE + ARM.";

    public static string BackupPlanState(string status) => status == PlanOld ? PlanOldPlain : status;

    #endregion

    #region Other places

    public const string CutoffCue = "ASCENT COMPLETE";

    public const string AscentDoneCircular = "ASCENT COMPLETE. ORBIT CIRCULAR.";

    public const string BurnDone = "BURN COMPLETE";

    public const string PassesDone = "PASSES COMPLETE";

    public const string DrawerLink = "MORE >";

    public static string PassDone(int done, int next, string inTime)
        => string.Create(Inv, $"PASS {done} COMPLETE. PASS {next} IN {inTime}.");

    public static string PassLink(int next) => string.Create(Inv, $"PASS {next} >");

    public static string NextBurnIn(string inTime) => BurnDone + ". NEXT BURN IN " + inTime + ".";

    public static string Finished(string what, in SituationFacts f) => f.Where switch
    {
        CraftSituation.Orbit when double.IsFinite(f.PeKm) && double.IsFinite(f.ApKm) => what + ", " + Fmt.Orbit(f.PeKm, f.ApKm) + ".",
        CraftSituation.FallingAir or CraftSituation.FallingNoAir => what + ". NEXT: LAND.",
        _ => what + ".",
    };

    public static string FinishedBrief(string message)
    {
        if (message.StartsWith(BurnDone + ", ", StringComparison.Ordinal))
            return BurnDone + ".";
        return message.StartsWith(PassesDone + ", ", StringComparison.Ordinal) ? PassesDone + "." : message;
    }

    public static string Landed(string body) => body.Length > 0 ? "LANDED, " + body : "LANDED";

    public static string BurnCreated(double dvMs, string inTime) => "BURN CREATED: " + Fmt.Dv(dvMs) + " IN " + inTime + ".";

    public static string TransferCreated(double dvMs, string inTime) => "TRANSFER CREATED: " + Fmt.Dv(dvMs) + " IN " + inTime + ".";

    public const string CreatedNext = "FLY OR PLAN NEXT BURN";

    public const string NextToolLink = "NEXT TOOL";

    public const string NextToolTip = "Keeps the burn and shows the tools. The next burn starts from the orbit after this one.";

    public const string StockDrawsBurn = "Stock draws the created burn.";

    public const string AlreadyCircular = "ALREADY CIRCULAR";

    public const string AlreadyCoplanar = "ALREADY COPLANAR";

    public static string AtTargetApse(bool setPeriapsis) => setPeriapsis ? "ALREADY AT THIS PE" : "ALREADY AT THIS AP";

    // The three answers of the convex launch banner.
    public const string BannerBackup = "FLY GRAVITY TURN";
    public const string BannerKeep = "WAIT FOR PLAN";
    public const string BannerHide = "HIDE";

    public const string AutoStageRowLabel = "AUTOSTAGE";
    public const string AutoStageHudNote = "The AUTOSTAGE button on the engine gauge panel sets the same switch.";
    public const string ArmedByGuidance = "Armed by AFC during guidance.";

    /// <summary>The tooltip note of an AutoStage switch that guidance armed. Guidance keeps a player disarm for the rest of the flight.</summary>
    public const string ArmedByGuidanceTip = "AFC armed AutoStage for this flight. A click disarms it for the rest of the flight.";

    // The tooltips of the nudge buttons of the hover, each with what it moves.
    public const string NudgeNorth = "Moves the aim one NUDGE STEP to the north.";
    public const string NudgeSouth = "Moves the aim one NUDGE STEP to the south.";
    public const string NudgeEast = "Moves the aim one NUDGE STEP to the east.";
    public const string NudgeWest = "Moves the aim one NUDGE STEP to the west.";
    public const string NudgeZero = "Resets the aim to zero: no drift, normal sink rate.";
    public const string NudgeUp = "Sinks one NUDGE STEP slower, or climbs.";
    public const string NudgeDown = "Sinks one NUDGE STEP faster.";

    public static string HoverSetpoint(double east, double north, double up)
        => string.Create(Inv, $"E {east:F1}  N {north:F1}  UP {up:F1} m/s");

    #endregion
}

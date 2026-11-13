using AdvancedFlightComputer.Features.Ui.Logic;

// Plain checks of the console's pure UI rules. Each failure prints one line, and any failure ends with exit code 1.
internal static class Program
{
    private static int _failures;
    private static int _checks;

    private static int Main()
    {
        StopPriority();
        EngineOutcomes();
        CommitRules();
        TakeOver();
        TakeOverQuestion();
        StatusOrdering();
        SettingMathRules();
        SpecKeysAreUnique();
        TierArraysHoldOneTier();
        EveryValueHasAHome();
        FoldLabels();
        PageKeys();
        OrbitToolKeys();
        RouteDecisions();
        DockPlacement();
        PorkchopGrid();
        SketchGeometry();
        PictureLaneRules();
        AxisTickRules();
        BurnBudgets();
        PlanVerbRules();
        FirstPages();
        SituationVerbCells();
        FooterVerbRules();
        ManeuverSeeds();
        NextSteps();
        VerbGuards();
        LandSolverRules();
        LandFormRules();
        ModeRegistry();
        ComprehensionTexts();
        FormRows();
        LaunchReachRules();
        AscentEndRules();
        Formats();
        RowLabelsFit();
        LogLines();
        FlightViewRules();
        TuneRulesCheck();
        StagingRulesCheck();
        ColorRulesCheck();
        FormStepRules();
        HeadRules();
        WordRules();
        FormWords();
        SentenceAndVerbRules();

        Console.WriteLine(_failures == 0
            ? $"PASS {_checks} checks"
            : $"FAIL {_failures} of {_checks} checks");
        return _failures == 0 ? 0 : 1;
    }

    private static void FormWords()
    {
        // The widths are the texts of the TOOL row measured on the pictures at scale 1.2, and the room of the row at 624 px.
        float[] tools = { 119.4f, 66.1f, 65.7f, 60.3f, 75.2f };
        float text = tools.Sum();
        const float Room = 583f, Pad = 9.6f, Least = 2.4f;
        Check("the tool labels hold MATCH, and its card says the plane", OrbitToolCatalog.ToolLabels[3] == "MATCH" && OrbitToolCatalog.Description(3).Contains("plane"));
        foreach ((float room, string where) in new[] { (Room, "624 px"), (440f, "the pane of WIDE") })
        {
            float pad = DockLayout.TabPad(text, tools.Length, room, Pad, Least);
            float x = 0f;
            for (int i = 0; i < tools.Length; i++)
            {
                float w = tools[i] + pad * 2f;
                Check("the tool row has one row in " + where + ", tab " + i, x + w <= room + DockLayout.TabRowTolerance);
                x += w + pad;
            }
        }
        // A row that closes up fills its room exactly, so a float sum can pass the end by a rounding error, and that must not send the last tab away.
        float[] wide = { 119.137f, 66.291f, 65.113f, 120.777f, 75.391f };
        float closed = DockLayout.TabPad(wide.Sum(), wide.Length, Room, 10f, Least);
        Check("a row that does not fit at the normal padding closes up", closed < 10f && closed >= Least);
        float end = 0f;
        for (int i = 0; i < wide.Length; i++)
        {
            float w = wide[i] + closed * 2f;
            Check("a closed row keeps tab " + i, end + w <= Room + DockLayout.TabRowTolerance);
            end += w + closed;
        }
        Check("a row too long to close up keeps its padding and wraps", DockLayout.TabPad(600f, 5, Room, Pad, Least) == Pad);

        static float Width(string s) => s.Length * 10f;
        Check("a summary that fits stays whole", FoldRules.FitSummary("Plane, insertion, turn", 300f, Width) == "Plane, insertion, turn");
        Check("a long summary lists what fits", FoldRules.FitSummary("Plane, insertion, turn, calculated", 240f, Width) == "Plane, insertion, turn");
        Check("the count of changed values stays", FoldRules.FitSummary("Plane, insertion, turn (2 CHANGED)", 240f, Width) == "Plane (2 CHANGED)");
        Check("one long item keeps whole words", FoldRules.FitSummary("No stage can fly home", 130f, Width) == "No stage can");
        Check("no room shows nothing", FoldRules.FitSummary("Calculated", 40f, Width).Length == 0);
        foreach (float room in new[] { 40f, 70f, 90f, 120f, 170f, 210f, 280f, 330f })
        {
            string shown = FoldRules.FitSummary("Orbit now, burn, resulting orbit, flight plan", room, Width);
            Check("a summary fits its room at " + room, Width(shown) <= room);
            Check("a summary has no cut word at " + room, shown.Length == 0 || "Orbit now, burn, resulting orbit, flight plan".StartsWith(shown, StringComparison.Ordinal)
                && (shown.Length == 45 || "Orbit now, burn, resulting orbit, flight plan"[shown.Length] is ',' or ' '));
        }
        FoldPart[] split = { new(FoldNames.Split, true, null, "Equal burn time") };
        Check("a part with a value lists the value", FoldRules.Contents(split) == "equal burn time");
        Check("a part with no name stays out of the summary", FoldRules.Contents([new FoldPart("", true), new FoldPart(FoldNames.Approach, true)]) == "approach");

        ChainStep[] one = LandChain.For(LandWay.FromHere, LandSolver.Touchdown, CraftSituation.FallingNoAir, false, true, true);
        Check("a chain of one step does not show", one.Length == 1 && !LandChain.Shows(one));
        Check("a chain of more steps shows", LandChain.Shows(LandChain.For(LandWay.FromOrbit, LandSolver.Touchdown, CraftSituation.Orbit, false, true, true)));
        Check("G-FOLD with no answer about the hover has one step", !LandChain.Shows(LandChain.For(LandWay.FromHere, LandSolver.Gfold, CraftSituation.FallingNoAir, false, true, true, false, hoverFlies: false)));
        Check("a way that ends in a crash says so", LandChain.EndsInImpact(LandChain.For(LandWay.FromHere, LandSolver.Entry, CraftSituation.FallingNoAir, false, true, true))
            && !LandChain.EndsInImpact(one) && !LandChain.EndsInImpact([]));

        Check("the mark has no comma after the chip", PageText.FooterMark(Owner.StockThenAfc, FlownBy.Afc).After == "THEN AFC" && !FlownBy.ThenAfc.Contains(','));
        Check("the words AFC keep their colour in the mark", ColorRules.AfcWordAt(FlownBy.ThenAfc) == 5);
        var next = new CommitAction("LAND NOW >", "", "", () => { }, Danger: false, Lead: true);
        var afterStep = new CommitSpec("HOVER", FlownBy.Afc, Caution: "Falling.", Act: () => { }, Owner: Owner.Afc, Secondary: next);
        Check("a next step takes the mark away", afterStep.NextLeads && afterStep.Mark.IsEmpty);
        Check("the card of the small verb says who flies", afterStep.VerbCard.Contains(PageText.FlownByCard(Owner.Afc)) && afterStep.VerbCard.Contains(PageText.OutlineStillWorks));
        var plain = new CommitSpec("HOVER", FlownBy.Afc, Caution: "Falling.", Act: () => { }, Owner: Owner.Afc);
        Check("an outline with no next step keeps its mark", !plain.Mark.IsEmpty);
        var ends = plain with { PressEnds = true };
        Check("a press that ends at once has no mark", ends.Mark.IsEmpty && ends.VerbCard.Contains(PageText.OutlinePressEnds));
        var deletion = new CommitAction(PlanVerbs.DeleteBurn, "", "", () => { });
        Check("a delete beside a verb keeps the mark", !(plain with { Secondary = deletion }).Mark.IsEmpty);

        var open = new CommitGates(false, false, "", "", false, Where: CraftSituation.FallingNoAir);
        CommitView hover = CommitResolver.Resolve(GuidanceModeId.Hover, default, open with { HoverProblem = PageText.HoverNoThrust });
        Check("a refused hover ends the press", hover.PressEnds && hover.Enabled);
        Check("G-FOLD with air ends the press", CommitResolver.Resolve(GuidanceModeId.GfoldHere, default, open with { Where = CraftSituation.FallingAir, HasAir = true }).PressEnds);
        Check("a boostback with no stage still starts", !CommitResolver.Resolve(GuidanceModeId.Boostback, default, open with { Where = CraftSituation.Ground, NoStageHome = true }).PressEnds);
        Check("a press that flies does not end", !CommitResolver.Resolve(GuidanceModeId.SixDofHere, default, open).PressEnds);

        VerbFit far = SituationVerbs.For(GuidanceModeId.SixDofHere, open with { FarSiteNote = PageText.SiteFar });
        Check("a far site has PICK SITE as its next step, in the warning colour", far.NextPicksSite && far.Reason == PageText.SiteFar && !far.Calm);
        VerbFit none = SituationVerbs.For(GuidanceModeId.GfoldHere, open with { NoSiteNote = "No site." });
        Check("no site has PICK SITE as its next step, calm", none.NextPicksSite && none.Calm);
        VerbFit rowFirst = SituationVerbs.For(GuidanceModeId.DeorbitLand, new CommitGates(false, false, "", "", false, Where: CraftSituation.Orbit, HasAir: true, GfoldChosen: true, NoSiteNote: "No site."));
        Check("the row comes before the site, so one gold stands", rowFirst.NextSolver == LandSolver.Touchdown && !rowFirst.NextPicksSite);
        Check("the sentence of a far site holds one row", PageText.SiteFar.Length * 10.9f <= 584f);

        CommitView climb = CommitResolver.Resolve(GuidanceModeId.GfoldHere, new CraftActivity { AscentRunning = true }, open with { Where = CraftSituation.CoastingUp, HasAir = true });
        Check("a landing in a climb is no next action", !climb.Fit.Fits && climb.Fit.Reason == SituationVerbs.AscentFliesEnds && climb.Fit.Calm && climb.Fit.NextQuiet && climb.Enabled && climb.NeedsTakeOver);
        Check("the way back in a climb is an outline", climb.Fit.NextLabel == SituationVerbs.BackToFlight && climb.Fit.NextPage == ConsolePage.Launch);
        CommitView flying = CommitResolver.Resolve(GuidanceModeId.GfoldHere, new CraftActivity { DescentLive = true, LandingMachineLive = false }, open);
        Check("a press under a deorbit says what it does", flying.Fit.Reason == "TAKE OVER replaces the deorbit with G-FOLD." && flying.Fit.NextLabel == SituationVerbs.BackToFlight);
        var heldFar = new LandNowFacts(LandWay.FromHere, LandSolver.Gfold, false, true, false);
        Check("off its way a deorbit has no sentence under the table", LandSolvers.HeldLine(in heldFar).Length == 0);
        Check("on its way the line on top of the form says it, so the table has no sentence", LandSolvers.HeldLine(heldFar with { Way = LandWay.FromOrbit }).Length == 0);

        Check("no thrust to read has no hover on its card, and says why", LandSolvers.Card(LandSolver.Gfold, 10.0, false, 5.0, false, false, hoverKnown: false).Contains("NO LIT ENGINE"));
        Check("a known refusal says the thrust", LandSolvers.Card(LandSolver.Gfold, 10.0, false, 5.0, false, false).Contains("ABOVE WEIGHT"));
        Check("the sentence of an unknown hover makes no claim", PageText.GuidanceOutcome(GuidanceModeId.GfoldHere, "G-FOLD", new PageText.OutcomeFacts(HoverFlies: false, HoverKnown: false)) == "Flies a G-FOLD descent.");
        Check("a hover row ends on the ground with no answer", LandSolvers.Ends(LandSolver.Gfold, 10.0, LandSolvers.HoverFlies(default(DescentFacts))) == "LANDED");
        Check("the same minimum throttle decides", LandSolvers.HoverFlies(1.0) && !LandSolvers.HoverFlies(1.01));

        Check("a mark that fits stays on the row", !FormRules.MarkOwnRow(300f, 200f, 584f, 10f, false));
        Check("a mark that does not fit moves", FormRules.MarkOwnRow(400f, 200f, 584f, 10f, false));
        Check("a long mark always moves", FormRules.MarkOwnRow(100f, 200f, 584f, 10f, true));
        Check("with no sentence the mark has the row", !FormRules.MarkOwnRow(0f, 200f, 584f, 10f, true));
        Check("no mark has no row", !FormRules.MarkOwnRow(400f, 0f, 584f, 10f, true));

        Check("a real burn never reads as nothing", PageText.BurnAnswer(16.8, "IN 1h 37m", 9607.0) == "16.8 m/s IN 1h 37m, UNDER 1 % OF ~9607 m/s");
        Check("the fuel sentence holds one row", PageText.FuelShort(9757.0, 1949.0).Length * 10.9f <= 584f);
        Check("a node names its cost and its time", PageText.NodeChoice(false, 9969.6, 4500.0).StartsWith("ASC ", StringComparison.Ordinal) && PageText.NodeChoice(true, 9757.0, 1500.0).StartsWith("DESC 9757 m/s, ", StringComparison.Ordinal));
        Check("a node out of reach has no numbers", PageText.NodeChoice(true, double.NaN, double.NaN) == "DESC NONE");
        Check("both nodes on one line", PageText.NodeLine(9969.6, 4500.0, double.NaN, double.NaN).EndsWith("DESC NODE NONE", StringComparison.Ordinal));
        Check("the goal of passes", PageText.PassesGoal("Ap 500 km", 3) == "GOAL: Ap 500 km IN 3 PASSES." && PageText.PassesGoal("", 3).Length == 0);
        Check("the delete of a pass says what follows", PageText.DeletePassLine(1, 3) == "DELETE PASS 1 also ends the passes after it." && PageText.DeletePassLine(3, 3).Length == 0);
        Check("the verb of a pass delete names it", PlanVerbs.DeletePass(2) == "DELETE PASS 2");

        Check("a goal that is not round needs no circularize", !NextStep.CircularizeFits(200.0, 422.0, 200.0, 420.0));
        Check("a round goal has it", NextStep.CircularizeFits(200.0, 202.0, 200.0, 200.0));
        Check("a goal not reached has it", NextStep.CircularizeFits(100.0, 422.0, 200.0, 420.0));
        Check("the cue names no tool when it does not fit", NextStep.AfterAscent(0.02, false).CueTarget == ConsolePages.Key(ConsolePage.Orbit));
        Check("the cue names the tool when it fits", NextStep.AfterAscent(0.02, true).CueTarget == ConsolePages.CueTarget(ConsolePage.Orbit, OrbitToolCatalog.CueCircularize));

        Check("the way dot of a coast with air is BOOSTBACK, because a spent booster has no later stage to count", LandWays.FitsNow(CraftSituation.CoastingUp, true, false) == LandWay.Boostback);
        CraftActivity flyingFromHere = new() { SixDofEngaged = true };
        Check("a picked row keeps the page of what flies, with PLAN open or closed", FlightView.PickHolds(LandWay.FromHere, in flyingFromHere));
        Check("another way does not", !FlightView.PickHolds(LandWay.Hover, in flyingFromHere));
        Check("legs turn urgent near an impact", FormRules.LegsUrgent(30.0) && !FormRules.LegsUrgent(120.0) && !FormRules.LegsUrgent(double.NaN));
        Check("the words of a plan calculating", PageText.WhenReadyVerb(false) == "LAUNCH" && PageText.WhenReadyVerb(true) == "ARM" && PageText.NoLaunchFollows == "NO LAUNCH QUEUED");
    }

    private static void SentenceAndVerbRules()
    {
        static float Width(string text) => text.Length * 10f;
        Check("a sentence that fits has no break", SentenceWrap.Break("short", 100f, Width) == "short");
        string site = SentenceWrap.Break("No site. A press flies to 0.674, 23.473.", 260f, Width);
        Check("a decimal never breaks", !site.Contains("0.\n") && !site.Contains("23.\n") && site.Replace('\n', ' ') == "No site. A press flies to 0.674, 23.473.", site);
        Check("the last row of a broken text", SentenceWrap.LastRow("one\ntwo three") == "two three" && SentenceWrap.LastRow("one") == "one");
        Check("a word has company on the last row", SentenceWrap.Break("aaaa bbbb cccc dddd", 150f, Width) == "aaaa bbbb\ncccc dddd", SentenceWrap.Break("aaaa bbbb cccc dddd", 150f, Width));
        Check("a row of two words above keeps them", SentenceWrap.Break("aaaaaa bbbbbb cccccc", 130f, Width) == "aaaaaa bbbbbb\ncccccc", SentenceWrap.Break("aaaaaa bbbbbb cccccc", 130f, Width));
        Check("a word that is alone above stays there", SentenceWrap.Break("aaaaaaaaaaaaaaaa bbbb", 100f, Width) == "aaaaaaaaaaaaaaaa\nbbbb");
        Check("a line end of the text holds", SentenceWrap.Break("aa bb\ncc dd", 100f, Width) == "aa bb\ncc dd");

        Check("the mark stands beside a short last row", !FormRules.MarkOwnRow(100f, 165f, 584f, 10f, false));
        Check("a last row with no room moves the mark", FormRules.MarkOwnRow(500f, 165f, 584f, 10f, false));

        var climb = new CraftActivity { AscentRunning = true };
        var air = new CommitGates(false, false, "", "", false, Where: CraftSituation.CoastingUp, HasAir: true);
        CommitView ends = CommitResolver.Resolve(GuidanceModeId.GfoldHere, in climb, in air);
        Check("a row that cannot start says so in a climb", ends.EndsOnly && ends.Fit.Reason == SituationVerbs.AscentFliesEnds && ends.Fit.NextQuiet && ends.Fit.NextLabel == SituationVerbs.BackToFlight);
        Check("a row that can start keeps the calm sentence", CommitResolver.Resolve(GuidanceModeId.SixDofHere, in climb, in air) is { EndsOnly: false, Fit.Reason: SituationVerbs.AscentFliesLand });
        Check("the verb has the name of that end", PlanVerbs.EndsOnly(StopResolver.Label(StopTarget.Ascent)) == "END ASCENT" && PlanVerbs.EndsOnly(StopResolver.Label(StopTarget.Landing)) == "END LANDING" && PlanVerbs.EndsOnly(StopResolver.Label(StopTarget.LaunchArmed)) == "END LAUNCH");
        var endSpec = new CommitSpec("G-FOLD", FlownBy.Afc, NeedsTakeOver: true, EndsLabel: "END ASCENT", PressEnds: true, Act: () => { }, TakeOverFrom: "ASCENT");
        Check("the label wins over the take-over", endSpec.ButtonLabel == "END ASCENT" && endSpec.Mark.IsEmpty);
        TakeOverAsk.Raise(endSpec.Verb, endSpec.TakeOverFrom, CommitResolver.EndsOnlyMessage(StopTarget.Ascent), () => { }, new DateTime(2000, 1, 1));
        CommitSpec asked = TakeOverAsk.Apply(endSpec, new DateTime(2000, 1, 1).AddSeconds(1.0));
        TakeOverAsk.Clear();
        Check("the question keeps that name", asked.ButtonLabel == "CONFIRM: END ASCENT" && asked.Status == "ONLY ENDS ASCENT. CONTINUE?" && asked.EndsLabel.Length == 0);
        Check("the question names no verb that is not on the page", CommitResolver.TakeOverMessage(StopTarget.SixDof, CommitResolver.GfoldVerb) == "G-FOLD ENDS TOUCHDOWN. CONTINUE?");
        Check("a sentence beside a next step says no press", !SituationVerbs.DeorbitFlies(GuidanceModeId.GfoldHere).Contains("press", StringComparison.OrdinalIgnoreCase)
            && !SituationVerbs.GfoldEndsDeorbit.Contains("press", StringComparison.OrdinalIgnoreCase) && !PageText.NoSiteAnswer("TOUCHDOWN", 0.674, 23.473).Contains("press", StringComparison.OrdinalIgnoreCase));
        Check("the sentence of no site holds one row", PageText.NoSiteAnswer("DEORBIT + LAND", -80.674, -123.473).Length <= 53);

        Check("a word in capitals stays in a sentence", PageText.InSentence("TWR 1.60") == "TWR 1.60" && PageText.InSentence("G-FOLD") == "G-FOLD" && PageText.InSentence("No hover") == "no hover");
        Check("a refused hover has lower case after the colon", PageText.RefusedAnswer(PageText.HoverNoThrust, "") == "REFUSED: NO HOVER (THRUST)");
        Check("the idle footer of a refused hover says what to do", PageText.HoverProblemLine(PageText.HoverNoThrust) == PageText.PickAnotherWay && PageText.HoverProblemLine(PageText.HoverNoEngine) == PageText.HoverNoEngine);
        foreach (string verdict in new[] { PageText.HoverVerdict(2.87, 28.7), PageText.HoverVerdict(0.3, 0.9), PageText.HoverVerdict(0.7, 7.0) })
            Check("the verdict of a hover holds one row and says where: " + verdict, verdict.Length <= 54 && verdict.StartsWith("TWR ", StringComparison.Ordinal));
        Check("no plan says what a press does, on one row", PageText.NoPlanAnswer(false) == "NO PLAN. SOLVE + LAUNCH." && PageText.NoPlanAnswer(true).EndsWith("ARM.", StringComparison.Ordinal) && PageText.NoPlanAnswer(false).Length <= 53);
        Check("the card of the launch verb says who drops the stages", PageText.ClimbFliesCard.Contains("AutoStage"));
        Check("the plan of passes and of one burn", PageText.PassesPlan("Ap 500 km", 3) == "Ap 500 km IN 3 PASSES" && PageText.PassesPlan("", 3).Length == 0
            && PageText.BurnGoal(200.0, 421.0, 200.0, 5000.0, true) == "AP 5000 km" && PageText.BurnGoal(200.0, 421.0, 421.0, 421.0, true) == "PE 421 km"
            && PageText.BurnGoal(200.0, 421.0, 200.0, 421.0, true) == "ORBIT 200 x 421 km" && PageText.BurnGoal(200.0, 421.0, 200.0, -1.0, false) == PageText.Escapes);
        Check("the fold with no value changed has a value", FoldRules.Split(PageText.HowItFliesDefaults) == (PageText.HowItFlies, "ALL DEFAULTS"));
        Check("a plan of several burns counts them", PageText.FooterMark(Owner.AfcPlansStockFlies, FlownBy.StockAuto, 3) == new OwnerMark("EXEC: PLANS 3", "STOCK", "BURNS"));
        Check("the mark of a spec counts its burns", new CommitSpec("CREATE 3 PASSES", FlownBy.AfcPlansStockAuto, Act: () => { }, Owner: Owner.AfcPlansStockFlies, PlannedBurns: 3).Mark.After == "BURNS");
        Check("the fold has no promise with no answer", PageText.HoverHandOverLabel(10.0, true, known: false) == "HANDOVER: HOVER FROM 10 m (IF HOVER)");
        Check("a segment has no seconds above a minute", PageText.NodeChoice(true, 9757.0, 1798.0) == "DESC 9757 m/s, 30m" && PageText.NodeChoice(false, 9970.0, 4560.0) == "ASC 9970 m/s, 1h 16m");
        Check("the short time", Fmt.TimeShort(45.0) == "45 s" && Fmt.TimeShort(1798.0) == "30m" && Fmt.TimeShort(3590.0) == Fmt.Time(3590.0) && Fmt.TimeShort(double.NaN) == "N/A");
        Check("the line of a narrow pane has one row per node", PageText.NodeLine(9970.0, 4560.0, 9757.0, 1798.0).Split('\n').Length == 2);
        Check("the first notice names no link", !ContainsLinkMark(PageText.DeleteEndsLanding) && PageText.DeleteEndsLanding.Contains("landing"));
        Check("the PLAN line of a deorbit says the lock once", FlightView.LandPlanLine(LandWay.FromOrbit, LandSolver.Gfold, true) == "LOCKED: SITE, APPROACH, METHOD (DEORBIT ACTIVE). STOP TO EDIT.");
        foreach (LandSolver solver in new[] { LandSolver.Entry, LandSolver.Touchdown, LandSolver.Gfold })
            Check("a PLAN line holds one row where it can: " + solver, !FlightView.LandPlanLine(LandWay.FromHere, solver, false).Contains("at once"));
    }

    private static bool ContainsLinkMark(string text) => text.Contains(" >", StringComparison.Ordinal);

    // At scale 1.2 and 624 px the answer row beside the mark holds about 37 characters of the row font, and a full row 53, so a sentence of two rows stays under 90.
    private static void WordRules()
    {
        Check("a detail of a row stands whole", FoldRules.WholeOrNone("SPLIT SAVES UNDER 1 m/s", 100f, text => text.Length) == "SPLIT SAVES UNDER 1 m/s");
        Check("a detail of a row with no room stands not at all", FoldRules.WholeOrNone("SPLIT SAVES UNDER 1 m/s", 10f, text => text.Length).Length == 0);
        Check("the card of PLAN says that the sentence inside decides", PageText.PlanTip.Contains("line inside") && !PageText.PlanTip.Contains("at once"));
        Check("a card of the table says how it lands", PageText.SolverTip == "The landing mode. AFC sets it for the craft and the body.");
        Check("a method with no hand-over says why", PageText.NoHandOversTip(LandSolver.Touchdown).Contains("TOUCHDOWN") && PageText.NoHandOversTip(LandSolver.Entry).Contains("No air"));
        (float tabRoom, float tabInset) = DockLayout.TabRowPlace(840f, 100f, 920f, 500f, 520f);
        Check("in WIDE that grew to the left the tabs keep the room and the place of the narrow window", tabRoom == 440f && tabInset == 400f);
        (tabRoom, tabInset) = DockLayout.TabRowPlace(840f, 100f, 920f, 100f, 520f);
        Check("in WIDE that grew to the right the tabs start at the left edge", tabRoom == 440f && tabInset == 0f);
        (tabRoom, tabInset) = DockLayout.TabRowPlace(440f, 500f, 520f, 500f, 520f);
        Check("in the narrow window the tabs have all the room", tabRoom == 440f && tabInset == 0f);
        var heavy = new DescentFacts(true, 1e6, 1e6, 1e5, 1.62, 0.0, 0.1, 0.1, 1.6, 1.6, 0.0, 15.0, 0.0, 0.0);
        var light = heavy with { TwrAtCraftMin = 0.6, TwrAtFloor = 0.6 };
        Check("a thrust above the weight at minimum throttle has no hover", !LandSolvers.HoverFlies(in heavy) && LandSolvers.HoverFlies(in light));
        Check("no thrust to read makes no hover promise", !LandSolvers.HoverFlies(default(DescentFacts)));
        var noHover = new PageText.OutcomeFacts(HoverFromM: 10.0, HoverFlies: false);
        Check("the footer of G-FOLD with no hover", PageText.GuidanceOutcome(GuidanceModeId.GfoldHere, CommitResolver.GfoldVerb, in noHover) == "Flies a G-FOLD descent to the ground.");
        Check("the footer of G-FOLD with a hover", PageText.GuidanceOutcome(GuidanceModeId.GfoldHere, CommitResolver.GfoldVerb, new PageText.OutcomeFacts(HoverFromM: 10.0)) == "Flies a G-FOLD descent, then hovers down from 10 m.");
        Check("the card of G-FOLD with no hover says where it ends", LandSolvers.Card(LandSolver.Gfold, 10.0, true, 5.0, hoverFlies: false).Contains("End: LANDED"));
        Check("the PLAN line of G-FOLD with no hover has no hover", !FlightView.LandPlanLine(LandWay.FromHere, LandSolver.Gfold, false, hoverFlies: false).Contains("HOVER"));
        Check("the verb of G-FOLD stays lit on such a craft, because the sentence is true",
            CommitResolver.Resolve(GuidanceModeId.GfoldHere, new CraftActivity(), new CommitGates(false, false, "", "", false, Where: CraftSituation.FallingNoAir)).Lit);

        Check("the press of the mode that failed tries again", PageText.FailureOf("G-FOLD: NO TRAJECTORY", GuidanceModeId.GfoldHere));
        Check("the press of another mode starts that mode", !PageText.FailureOf("G-FOLD: NO TRAJECTORY", GuidanceModeId.SixDofHere));
        Check("touchdown has its own failures", PageText.FailureOf("TOUCHDOWN: NO PLAN", GuidanceModeId.SixDofHere) && !PageText.FailureOf("TOUCHDOWN: NO PLAN", GuidanceModeId.GfoldHere));
        Check("entry has its own failures", PageText.FailureOf("NO ENTRY PLAN", GuidanceModeId.EntryHere));
        Check("a failure of AFC counts for every verb", PageText.FailureOf("GUIDANCE FAULT (REPEATED)", GuidanceModeId.Hover));
        var fall = new CommitGates(false, false, "", "", false, Where: CraftSituation.FallingNoAir);
        CommitView retry = CommitResolver.Resolve(GuidanceModeId.GfoldHere, new CraftActivity(), fall with { PageFact = PageText.TriesAgain, PageFactCalm = true });
        Check("a try again is an outline in the normal colour", retry is { Enabled: true, Lit: false } && retry.Fit.Calm);
        Check("the way out stays lit", CommitResolver.Resolve(GuidanceModeId.SixDofHere, new CraftActivity(), in fall).Lit);
        string head = FailureBand.Text(new BandFacts("G-FOLD: NO TRAJECTORY", EngineNow.Off, FlownBy.You, 75.0));
        Check("the failure says why and the engine", head == "G-FOLD: NO TRAJECTORY. ENGINE OFF. MANUAL. IMPACT T-75 s", head);

        var land = new VerbFit(PageText.BurnAfterImpact, ConsolePage.Land, LandWay.FromHere);
        Check("a burn after the impact leads to LAND NOW, in the normal colour", land.Calm && PageText.NextLabel(ConsolePage.Land, LandWay.FromHere) == "LAND NOW >");
        VerbFit hover = SituationVerbs.For(GuidanceModeId.Hover, fall with { HoverProblem = "No hover." });
        Check("a refused hover in a fall leads to LAND NOW", hover.NextPage == ConsolePage.Land && hover.NextWay == LandWay.FromHere && hover.Calm && hover.Reason == "No hover.");
        VerbFit padHover = SituationVerbs.For(GuidanceModeId.Hover, fall with { Where = CraftSituation.Ground, HoverProblem = PageText.HoverNoEngine });
        Check("the hover on the pad answers as the other ways", padHover.Reason == SituationVerbs.OnGround && padHover.NextPage == ConsolePage.Launch);

        CommitView takeOver = CommitResolver.Resolve(GuidanceModeId.GfoldHere, new CraftActivity { DescentLive = true, LandingStockAuto = true }, new CommitGates(false, false, "", "", false, Where: CraftSituation.Orbit));
        Check("LAND NOW under a deorbit does not send the pilot to the deorbit", takeOver.Fit.Reason == SituationVerbs.DeorbitFlies(GuidanceModeId.GfoldHere) && takeOver.Fit.NextWay == LandWay.FromOrbit && takeOver.Fit.NextLabel == "BACK TO FLIGHT >");
        Check("that press is still a take-over that works", takeOver is { Enabled: true, NeedsTakeOver: true, ButtonLabel: "TAKE OVER: G-FOLD" });
        var deorbitFlies = new CraftActivity { DescentLive = true };
        CommitView hoverUnderDeorbit = CommitResolver.Resolve(GuidanceModeId.Hover, deorbitFlies, fall);
        Check("a hover that can fly replaces the deorbit and asks", hoverUnderDeorbit.Fit.Reason == SituationVerbs.DeorbitFlies(GuidanceModeId.Hover) && hoverUnderDeorbit.NeedsTakeOver && !hoverUnderDeorbit.PressEnds);
        CommitView refusedUnderDeorbit = CommitResolver.Resolve(GuidanceModeId.Hover, deorbitFlies, fall with { HoverProblem = PageText.HoverNoThrust });
        Check("a refused hover says why and asks nothing, because the deorbit flies on", refusedUnderDeorbit is { NeedsTakeOver: false, PressEnds: true, Enabled: true, Detail: "" } && refusedUnderDeorbit.Fit.Reason == PageText.HoverNoThrust && refusedUnderDeorbit.Fit.NextLabel == SituationVerbs.BackToFlight);
        CommitView gfoldInAirUnderDeorbit = CommitResolver.Resolve(GuidanceModeId.GfoldHere, deorbitFlies, fall with { Where = CraftSituation.FallingAir, HasAir = true });
        Check("G-FOLD with air says that the press only ends the deorbit, and asks", gfoldInAirUnderDeorbit is { NeedsTakeOver: true, PressEnds: true, Detail: "" } && gfoldInAirUnderDeorbit.Fit.Reason == SituationVerbs.GfoldEndsDeorbit && gfoldInAirUnderDeorbit.Fit.NextLabel == SituationVerbs.BackToFlight);
        var climb = new CraftActivity { AscentRunning = true };
        CommitView refusedInClimb = CommitResolver.Resolve(GuidanceModeId.Hover, climb, fall with { HoverProblem = PageText.HoverNoThrust });
        Check("a refused hover in a climb keeps the calm sentence, asks nothing and has its reason on the card", refusedInClimb is { NeedsTakeOver: false, PressEnds: true, Detail: PageText.HoverNoThrust } && refusedInClimb.Fit.Reason == SituationVerbs.AscentFliesLand);
        CommitView gfoldInClimb = CommitResolver.Resolve(GuidanceModeId.GfoldHere, climb, fall with { Where = CraftSituation.FallingAir, HasAir = true });
        Check("G-FOLD with air in a climb asks, because it ends the climb, and has its reason on the card", gfoldInClimb is { NeedsTakeOver: true, PressEnds: true, Detail: SituationVerbs.GfoldNoAir });
        Check("a press that ends with its reason in the row adds no card line", CommitResolver.Resolve(GuidanceModeId.GfoldHere, default, fall with { Where = CraftSituation.FallingAir, HasAir = true }).Detail.Length == 0);
        Check("a first obstacle over a refused hover keeps the reason on the card", CommitResolver.Resolve(GuidanceModeId.Hover, default, fall with { Where = CraftSituation.Orbit, HoverProblem = PageText.HoverNoEngine }).Detail == PageText.HoverNoEngine);
        Check("the verb of the deorbit itself keeps its status", CommitResolver.Resolve(GuidanceModeId.DeorbitLand, new CraftActivity { DescentLive = true, LandingStockAuto = true }, new CommitGates(false, false, "", "", false, Where: CraftSituation.Orbit)).Reason == CommitResolver.StockDeorbitStatus);

        foreach (GuidanceModeId mode in new[] { GuidanceModeId.EntryHere, GuidanceModeId.SixDofHere, GuidanceModeId.GfoldHere, GuidanceModeId.Hover })
            foreach (bool air in new[] { true, false })
            {
                VerbFit inOrbit = SituationVerbs.For(mode, new CommitGates(false, false, "", "", false, Where: CraftSituation.Orbit, HasAir: air));
                Check($"in orbit DEORBIT stays on every row: {mode}, air {air}", inOrbit.Reason == SituationVerbs.DeorbitFirst && inOrbit.NextWay == LandWay.FromOrbit);
            }
        VerbFit gfoldAir = SituationVerbs.For(GuidanceModeId.GfoldHere, new CommitGates(false, false, "", "", false, Where: CraftSituation.FallingAir, HasAir: true));
        Check("G-FOLD in air names the row that flies, as a button", gfoldAir.NextSolver == LandSolver.Touchdown && gfoldAir.Calm);
        VerbFit deorbitAir = SituationVerbs.For(GuidanceModeId.DeorbitLand, new CommitGates(false, false, "", "", false, Where: CraftSituation.Orbit, HasAir: true, GfoldChosen: true));
        Check("also on DEORBIT", deorbitAir.NextSolver == LandSolver.Touchdown && deorbitAir.Reason == SituationVerbs.GfoldNeedsNoAir);
        VerbFit entryNoAir = SituationVerbs.For(GuidanceModeId.EntryHere, new CommitGates(false, false, "", "", false, Where: CraftSituation.FallingNoAir));
        Check("entry with no air names the way out", entryNoAir.Reason == "No air. Use TOUCHDOWN." && entryNoAir.NextSolver == LandSolver.Touchdown);
        CommitView farPass = CommitResolver.Resolve(GuidanceModeId.DeorbitLand, new CraftActivity(), new CommitGates(false, false, "", "", false, Where: CraftSituation.Orbit, PageFact: PageText.PassesFar, PageFactNext: ConsolePage.Orbit, PageFactCalm: true));
        Check("a far pass says what to do, with MANEUVER", farPass.Fit is { Reason: PageText.PassesFar, NextPage: ConsolePage.Orbit, Calm: true });
        CommitView curve = CommitResolver.Resolve(GuidanceModeId.SixDofHere, new CraftActivity(), new CommitGates(false, false, "", "", false, Where: CraftSituation.FallingNoAir));
        Check("a path that curves leaves the verb lit on LAND NOW, as on DEORBIT", curve.Lit);

        VerbFit noStage = SituationVerbs.For(GuidanceModeId.Boostback, new CommitGates(false, false, "", "", false, Where: CraftSituation.Ground, NoStageHome: true));
        Check("the footer reads the fact of the BOOSTERS fold", noStage.Reason == "NO STAGE CAN FLY HOME." && noStage.NextPage == null && noStage.Calm);
        Check("with such a stage it starts after separation", SituationVerbs.For(GuidanceModeId.Boostback, new CommitGates(false, false, "", "", false, Where: CraftSituation.Ground)).Reason == SituationVerbs.AfterSeparation);

        foreach (VerbFit normal in new[]
                 {
                     SituationVerbs.ForManeuver(CraftSituation.Ground), SituationVerbs.ForTransfer(CraftSituation.Ground), SituationVerbs.ForTransfer(CraftSituation.FallingAir),
                     SituationVerbs.For(GuidanceModeId.AscentUpfg, new CommitGates(false, false, "", "", false, Where: CraftSituation.Orbit)),
                     SituationVerbs.For(GuidanceModeId.AscentUpfg, new CommitGates(false, false, "", "", false, Where: CraftSituation.CoastingUp)),
                     SituationVerbs.For(GuidanceModeId.SixDofHere, new CommitGates(false, false, "", "", false, Where: CraftSituation.Escaping)),
                     SituationVerbs.For(GuidanceModeId.DeorbitLand, new CommitGates(false, false, "", "", false, Where: CraftSituation.Ground)),
                 })
            Check("a normal state is calm: " + normal.Reason, !normal.Fits && normal.Calm);
        Check("the words of the ground", SituationVerbs.OnGround == "LAUNCH FIRST." && SituationVerbs.ForManeuver(CraftSituation.Ground).Reason == SituationVerbs.OnGround);
        Check("the words of an orbit", SituationVerbs.InOrbitShape == "NEXT: SHAPE ORBIT.");

        Check("a take-over says what ends and what flies", PageText.TakeOverOutcome(GuidanceModeId.GfoldHere, "TOUCHDOWN") == "Replaces TOUCHDOWN with G-FOLD.");
        Check("also from a landing", PageText.TakeOverOutcome(GuidanceModeId.SixDofHere, "LANDING") == "Replaces LANDING with TOUCHDOWN.");
        Check("a verb that takes over names the mode", PlanVerbs.TakeOver("TOUCHDOWN") == "TAKE OVER: TOUCHDOWN" && PlanVerbs.TakeOver("DEORBIT + LAND") == "TAKE OVER: DEORBIT + LAND");

        Check("a stopped deorbit says what happened", PageText.LandingStopped(false, true) == "STOPPED. ENGINE CUT, BURN DELETED.");
        Check("with the engine off it says that", PageText.LandingStopped(true, true) == "STOPPED. ENGINE OFF, BURN DELETED.");

        Check("a stale plan does not tick", PageText.PlanState("PLANNED FOR LIFTOFF 186 s EARLIER") == PageText.PlanOld && PageText.PlanOld == "PLAN OLD. SOLVE + LAUNCH.");
        Check("the other reasons keep their words", PageText.PlanState("ORBIT CHANGED") == "PLAN OUT OF DATE: ORBIT CHANGED");
        Check("the box before a plan does not repeat the footer", PageText.LaunchNoPlan(true) == "SOLVE DRAWS THE CLIMB.");
        Check("the site has one precision", FlightView.LandReads(LandWay.FromHere, false, -3.005, -23.415) == "SITE -3.005, -23.415" && PageText.NoSiteAnswer("TOUCHDOWN", 0.674, 23.473).Contains("0.674, 23.473"));
        Check("the pick has one name", PageText.PickSite == "PICK SITE" && PageText.PickSiteTip.StartsWith("The next click"));
        Check("the climb has plain names", PageText.ClimbLabels[0] == "SOLVED" && PageText.ClimbLabels[1] == "GRAVITY TURN" && !PageText.ClimbCards[0].Contains("convex plan") && !PageText.ClimbCards[1].Contains("pitch program"));
        Check("the head and the idle chain share their words", GuidanceModes.BoostbackPhasesFor(false)[0] == "SETTLE" && GuidanceModes.BoostbackPhasesFor(true)[^1] == "ENTRY"
            && string.Join(">", LandChain.For(LandWay.Boostback, LandSolver.Entry, CraftSituation.FallingAir, true, true, false).Take(5).Select(c => c.Text.ToUpperInvariant())) == "SETTLE>SLEW>BURN>GLIDE>ENTRY");
        Check("a G-FOLD descent reads the hover", GuidanceModes.GfoldPhasesFor(true).Contains("HOVER") && !GuidanceModes.GfoldPhasesFor(false).Contains("HOVER") && GuidanceModes.GfoldPhasesFor(false).Contains("LANDING BURN"));
        foreach (string jargon in new[] { "COLD SOLVE", "CONVERGING", "CONVEX" })
            Check("no jargon in the phases: " + jargon, !GuidanceModes.AscentPhases.Concat(GuidanceModes.DeorbitPhases).Concat(GuidanceModes.GfoldPhases).Concat(GuidanceModes.SixDofPhases).Any(p => p.Contains(jargon)));
        Check("a switch has one name", GuidanceSettingSpecs.AutoStage.Label == "AFC MAY USE ENGINES" && PageText.MayOffTip.Contains("AFC MAY USE ENGINES"));
        Check("the hand-over rows say who takes what", GuidanceSettingSpecs.ThreeDofFromGlide.Label == "ENTRY ON GLIDE" && GuidanceSettingSpecs.ThreeDofSixDofHandover.Label == "TOUCHDOWN FOR BURN END" && GuidanceSettingSpecs.GfoldHoverHandoffAltM.Label == "HOVER FROM");
        Check("the height says what it measures", GuidanceSettingSpecs.VehicleHeightM.Label == "COM ABOVE LEGS" && GuidanceSettingSpecs.VehicleHeightAuto.Label == "MEASURE COM ABOVE LEGS");
        Check("the tuning points are no nodes", GuidanceSettingSpecs.GfoldNodes.Label == "SOLVER POINTS" && GuidanceSettingSpecs.SixDofNodes.Label == "SOLVER POINTS");
        Check("the verbs are the mode names", new[] { CommitResolver.GfoldVerb, EntryGuidance.CommitVerb }.All(v => v.Length > 0 && !v.StartsWith("START ", StringComparison.Ordinal)));
        Check("the plain mix names", PageText.AllocatorLabels.SequenceEqual(new[] { "BY AXIS", "MIN FUEL" }) && PageText.AllocatorCards[1].Contains("BY AXIS"));
        Check("an armed launch is canceled", StopResolver.Label(StopTarget.LaunchArmed) == "STOP LAUNCH");
        Check("the words say burn", !StopResolver.OutcomeShort(StopTarget.StockAuto, new CraftActivity()).Contains("node") && !PageText.FlownByCard(Owner.AfcPlansStockFlies).Contains("node") && !PageText.PassesOutcome(2).Contains("node"));

        foreach (string sentence in new[]
                 {
                     SituationVerbs.OnGround, SituationVerbs.InOrbitShape, SituationVerbs.CoastingShape, SituationVerbs.EscapingShape, SituationVerbs.FallingLand, SituationVerbs.AfterSeparation,
                     SituationVerbs.NoStageHome, SituationVerbs.NoImpact, SituationVerbs.BelowBrakingStart, SituationVerbs.DeorbitFirst, SituationVerbs.BoostbackNoAir, SituationVerbs.AscentFlies,
                     SituationVerbs.GuidanceFlies, SituationVerbs.DeorbitFlies(GuidanceModeId.GfoldHere), SituationVerbs.AscentFliesLand, SituationVerbs.EntryNeedsAir, SituationVerbs.GfoldNeedsNoAir, SituationVerbs.GfoldEndsDeorbit,
                     PageText.PassesFar, PageText.PressPlansNext, PageText.GfoldNoHoverOutcome, PageText.CorrectionOutcome, PageText.TriesAgain,
                     PageText.NoGimbalFact, PageText.OverPoweredFact(0.45), PageText.PlanOld, PageText.BurnAfterImpact, PageText.HoverNoEngine,
                     PageText.FuelShort(9757.0, 1978.0), PageText.FuelShort(9757.0, 1978.0, chained: true), PageText.BurnAnswer(9757.5, "IN 12h 44m", 11978.0),
                     PageText.TakeOverOutcome(GuidanceModeId.SixDofHere, "TOUCHDOWN"), PageText.TakeOverOutcome(GuidanceModeId.AscentUpfg, "BOOSTBACK"),
                     PageText.PassesOutcome(12), PageText.LandingStopped(true, true), PageText.SiteFar,
                 })
        {
            Check("a footer sentence holds two rows: " + sentence, sentence.Length <= 90 && sentence.All(ch => ch < 128));
            Check("a footer sentence has no word the list took away: " + sentence, !sentence.Contains("node") && !sentence.Contains("Engage") && !sentence.Contains("ENGAGE"));
        }
        Check("a fuel sentence holds one row beside the mark", PageText.FuelShort(2002.0, 1978.0).Length <= 37, PageText.FuelShort(2002.0, 1978.0));
        foreach (GuidanceModeId mode in new[] { GuidanceModeId.AscentUpfg, GuidanceModeId.Boostback, GuidanceModeId.DeorbitLand, GuidanceModeId.EntryHere, GuidanceModeId.SixDofHere, GuidanceModeId.GfoldHere, GuidanceModeId.Hover })
            Check("an outcome holds two rows: " + mode, PageText.GuidanceOutcome(mode, "X", new PageText.OutcomeFacts(AutoStageArms: true, Solver: "TOUCHDOWN", HandToTouchdown: true, TouchdownLastS: 5.0, HoverFromM: 10.0)).Length <= 90);
        // The mark takes the small level at about 8.7 px per character at scale 1.2. The sentence beside it needs the rest of 584 px, at 10.9 px per character.
        foreach (Owner owner in new[] { Owner.Afc, Owner.Stock, Owner.AfcPlansStockFlies, Owner.StockThenAfc })
        {
            OwnerMark mark = PageText.FooterMark(owner, FlownBy.Afc);
            float markPx = (mark.Before.Length + mark.Chip.Length + mark.After.Length) * 8.7f + 30f;
            Check("the mark leaves the sentence room: " + owner, 584f - markPx >= 26f * 10.9f, markPx.ToString());
        }
        foreach (string[] labels in new[] { PageText.ClimbLabels, PageText.AllocatorLabels, OrbitToolCatalog.ToolLabels, LandWays.Labels })
            Check("the choices fit the narrow width: " + labels[0], labels.Sum(l => l.Length) + labels.Length * 2 <= 53, string.Join(" ", labels));
        Check("the strip of passes holds its row", PageText.PassClosest(19999.9, 86400.0).Length <= 53 && PageText.PassFlies(19999.9, 86400.0).Length <= 53);
    }

    // At the default width at interface scale 1.2 a head row is 584 px at 10.9 px per character.
    private static void HeadRules()
    {
        static float Chars(string text) => text.Length * 10.9f;
        const float Row = 584f;
        const float Least = 47f * 10.9f;

        var fall = new SituationFacts(CraftSituation.FallingNoAir, "Luna", HeightM: 8400.0, ImpactS: 57.0);
        Check("a fall says its height and its impact", SituationText.Line(in fall) == "FALL, Luna, 8400 m. IMPACT T-57 s. MANUAL", SituationText.Line(in fall));
        Check("above 10 km the height is in km", SituationText.Line(fall with { HeightM = 12400.0 }).Contains("12.4 km"));
        Check("with room it stands whole", SituationText.LineThatFits(in fall, Chars, Row) == SituationText.Line(in fall));
        Check("the body goes first", SituationText.LineThatFits(in fall, Chars, 40f * 10.9f) == "FALL, 8400 m. IMPACT T-57 s. MANUAL", SituationText.LineThatFits(in fall, Chars, 40f * 10.9f));
        Check("then the height", SituationText.LineThatFits(in fall, Chars, 34f * 10.9f) == "FALL. IMPACT T-57 s. MANUAL", SituationText.LineThatFits(in fall, Chars, 34f * 10.9f));
        Check("then the word of the fall", SituationText.LineThatFits(in fall, Chars, 26f * 10.9f) == "IMPACT T-57 s. MANUAL", SituationText.LineThatFits(in fall, Chars, 26f * 10.9f));
        Check("who flies goes last", SituationText.LineThatFits(in fall, Chars, 10f * 10.9f) == "MANUAL");
        var longFall = new SituationFacts(CraftSituation.FallingAir, "Earth", HeightM: 123456.0, ImpactS: 3000.0, Thrust: true);
        Check("the longest fall fits beside the drawer control with a count", Chars(SituationText.LineThatFits(in longFall, Chars, Row - 11f * 10.9f)) <= Row - 11f * 10.9f);
        Check("the longest fall fits at the least width", Chars(SituationText.LineThatFits(in longFall, Chars, Least - 11f * 10.9f)) <= Least - 11f * 10.9f);
        Check("one fall has one word, also under power", SituationText.Line(in longFall).StartsWith("FALL, Earth, 123.5 km. IMPACT T-50m 00s"), SituationText.Line(in longFall));
        Check("the longest fall keeps who flies in the row", SituationText.LineThatFits(in longFall, Chars, Row - 11f * 10.9f).EndsWith(SituationText.YouFly, StringComparison.Ordinal));
        Check("the fall after a stop keeps who flies beside a count", SituationText.LineThatFits(new SituationFacts(CraftSituation.FallingNoAir, "Luna", HeightM: 4319.0, ImpactS: 32.0, Thrust: true), Chars, Row - 11f * 10.9f) == "FALL, Luna, 4319 m. IMPACT T-32 s. MANUAL");
        Check("an engine at work in a fall has its line", SituationText.EngineLeftLine(in fall, 10.0) == "ENGINE AT 10 %.");
        Check("an engine that is off has none", SituationText.EngineLeftLine(in fall, 0.0).Length == 0 && SituationText.EngineLeftLine(in fall, null).Length == 0);
        Check("only a fall has that line", SituationText.EngineLeftLine(new SituationFacts(CraftSituation.CoastingUp, "Earth"), 100.0).Length == 0);
        Check("a message says who flies where it has room", SituationText.MessageLine("ASCENT COMPLETE", Chars, Row) == "ASCENT COMPLETE. MANUAL");
        Check("the longest stop line holds the row with who flies", Chars(PageText.LandingStopped(true, true) + " " + SituationText.YouFly) <= Row - 7f * 10.9f - 8f, PageText.LandingStopped(true, true));
        Check("a message that needs the row keeps its words", SituationText.MessageLine(PageText.PickSiteCue, Chars, Row - 11f * 10.9f) == PageText.PickSiteCue);
        Check("a finished burn gives up its orbit before who flies", SituationText.MessageLine("BURN COMPLETE, 200 x 421 km.", text => text.Length, 25f) == "BURN COMPLETE. MANUAL");
        Check("a finished burn that fits says all", SituationText.MessageLine("BURN COMPLETE, 200 x 421 km.", text => text.Length, 35f) == "BURN COMPLETE, 200 x 421 km. MANUAL");
        Check("the last pass gives up its orbit too", PageText.FinishedBrief("PASSES COMPLETE, 200 x 5000 km.") == "PASSES COMPLETE.");
        Check("any other message stays", PageText.FinishedBrief("BURN COMPLETE. NEXT: LAND.") == "BURN COMPLETE. NEXT: LAND.");
        Check("a fall with no ground ahead has no impact", SituationText.Line(fall with { ImpactS = double.NaN }) == "FALL, Luna, 8400 m. MANUAL");
        Check("an impact that passed is not said", !SituationText.Line(fall with { ImpactS = -1.0 }).Contains("IMPACT"));
        Check("no value is the line it was", SituationText.Line(new SituationFacts(CraftSituation.FallingNoAir, "Luna")) == "FALL, Luna, NO AIR. MANUAL");
        Check("a line that is no fall does not change", SituationText.LineThatFits(new SituationFacts(CraftSituation.Orbit, "Earth", 200.0, 423.0), Chars, 1f) == "ORBIT, Earth, 200 x 423 km. MANUAL");
        Check("the time to impact is known in a fall", SituationText.ImpactKnown(fall with { ImpactS = 59.0 }));
        Check("no time, no impact to name", !SituationText.ImpactKnown(fall with { ImpactS = double.NaN }) && !SituationText.ImpactKnown(new SituationFacts(CraftSituation.Orbit, "Earth", ImpactS: 5.0)));
        (int impactAt, int impactLength) = SituationText.ImpactWords("FALL, 7058 m. IMPACT T-59 s. MANUAL");
        Check("the words of the impact alone take the warning", "FALL, 7058 m. IMPACT T-59 s. MANUAL".Substring(impactAt, impactLength) == "IMPACT T-59 s.");
        Check("a row with no impact has no such words", SituationText.ImpactWords("FALL, Luna, NO AIR. MANUAL").Start < 0);
        Check("the words hold when the row ends in dots", "FALL. IMPACT T-5...".Substring(SituationText.ImpactWords("FALL. IMPACT T-5...").Start, SituationText.ImpactWords("FALL. IMPACT T-5...").Length) == "IMPACT T-5.");
        SituationFacts falls = new(CraftSituation.FallingNoAir, "Luna", ImpactS: 70.0);
        SituationFacts orbits = new(CraftSituation.Orbit, "Luna");
        Check("the strip stays while AFC flies", FailureBand.StripStays(true, false, false, in orbits));
        Check("the strip stays while a failure waits", FailureBand.StripStays(false, true, false, in orbits));
        Check("the strip stays after OK while the craft falls", FailureBand.StripStays(false, false, true, in falls));
        Check("the strip goes after OK once nothing falls", !FailureBand.StripStays(false, false, true, in orbits));
        Check("no failure, no strip in a fall", !FailureBand.StripStays(false, false, false, in falls));
        Check("the row of that strip says the time", FailureBand.ToImpact(70.0) == "IMPACT T-70 s");

        Check("no word flips while it climbs", SituationText.FlyingPlace(new SituationFacts(CraftSituation.CoastingUp, "Earth", -10.0, 47.0, Thrust: true)) == "Earth, AP 47 km");
        Check("numbers that hold the orbit take it from the place", SituationText.FlyingPlace(new SituationFacts(CraftSituation.CoastingUp, "Earth", -10.0, 47.0), orbitShown: true) == "Earth");
        Check("a fall names its body and its height", SituationText.FlyingPlace(in fall, impactShown: true) == "Luna, 8400 m");
        Check("and its impact when the numbers lack it", SituationText.FlyingPlace(in fall, heightShown: true) == "Luna, IMPACT T-57 s");
        Check("numbers that hold both leave the body", SituationText.FlyingPlace(in fall, heightShown: true, impactShown: true) == "Luna");
        Check("an orbit keeps its size in the place", SituationText.FlyingPlace(new SituationFacts(CraftSituation.Orbit, "Luna", 211.0, 215.0)) == "ORBIT, Luna, 211 x 215 km");
        Check("the ground keeps its word", SituationText.FlyingPlace(new SituationFacts(CraftSituation.Ground, "Earth")) == "LANDED, Earth");

        Check("a message takes the one row", StatusSelector.IdleRow(false, false, true) == HeadRowHolds.Message);
        Check("with none the row holds the situation", StatusSelector.IdleRow(false, false, false) == HeadRowHolds.Situation);
        Check("a fall keeps its line", StatusSelector.IdleRow(false, true, true) == HeadRowHolds.Situation);
        Check("a failure goes first", StatusSelector.IdleRow(true, true, true) == HeadRowHolds.Failure);
        var refused = new List<StatusLine> { new(Severity.Warning, "Hover refused: TWR 1.60 at minimum throttle.", ""), new(Severity.Info, "Plain", "") };
        Check("a refused press takes no row while another line has one", StatusSelector.PickRow(refused, false, "Hover refused: TWR 1.60 at minimum throttle.") == 1);
        Check("and with no other line no row at all", StatusSelector.PickRow(refused.GetRange(0, 1), false, "Hover refused: TWR 1.60 at minimum throttle.") == -1);
        Check("no line to leave out picks as before", StatusSelector.PickRow(refused, false) == 0);
        Check("the footer says the word", PageText.RefusedAnswer("", "Hover refused: TWR 1.60 at minimum throttle.") == "REFUSED: TWR 1.60 at minimum throttle.");
        Check("the reason of the verb goes first", PageText.RefusedAnswer("Thrust is too high, so no hover.", "Hover refused: TWR 1.60.") == "REFUSED: thrust is too high, so no hover.");
        Check("a line with no word gets it", PageText.RefusedAnswer("", "the deorbit window expired.") == "REFUSED: the deorbit window expired.");

        const float Tgo = 16f * 10.9f + 8f;
        HeadValue[] landing = [new("ALT 8985 m"), new("VS -185.5 m/s"), new("SITE 1425.4 km", 1)];
        string shown = HeadNumbers.Fit(landing, Chars, Row - Tgo);
        Check("what has room stays, in order", shown == "ALT 8985 m   VS -185.5 m/s", shown);
        Check("the site leaves whole", !shown.Contains("SITE"));
        Check("with room all of it stands", HeadNumbers.Fit(landing, Chars, Row) == "ALT 8985 m   VS -185.5 m/s   SITE 1425.4 km");
        Check("the card names what left", HeadNumbers.Card(landing, shown, "Card") == "SITE 1425.4 km\n\nCard");
        Check("a row with room has only the card", HeadNumbers.Card(landing, HeadNumbers.Fit(landing, Chars, Row), "Card") == "Card");
        Check("a row with nothing is empty", HeadNumbers.Fit(null, Chars, Row) == "" && HeadNumbers.Fit(Array.Empty<HeadValue>(), Chars, Row) == "" && HeadNumbers.Card(null, "", "") == "");
        Check("a row too narrow for the value has none", HeadNumbers.Fit([new HeadValue("ALT 8985 m")], Chars, 10f) == "");
        List<HeadCell> oneCell = HeadNumbers.Cells([new HeadValue("ALT 89 m")], Chars, Row);
        Check("the word stands after the numbers where it has room", HeadNumbers.WordX(oneCell, "Yours until the burn", Chars, Row) == 12f * 10.9f);
        Check("a row with no numbers has the word at its start", HeadNumbers.WordX(new List<HeadCell>(), "Yours until the burn", Chars, Row) == 0f);
        Check("a row with no room sends the word to the card", HeadNumbers.WordX(oneCell, "Yours until the burn", Chars, 20f * 10.9f) < 0f);
        Check("a short value can stay where a long one left", HeadNumbers.Fit([new HeadValue("LONG VALUE HERE", 2), new HeadValue("MID", 1)], Chars, 8f * 10.9f) == "MID");
        foreach (float room in new[] { Row - Tgo, Least - Tgo, 2f * Row })
        {
            foreach (HeadValue[] values in new[]
            {
                landing,
                PageText.AscentNumbers(true, 1192.4, 1420.0, -6364.2, 1200.0, 2915.0),
                PageText.AscentNumbers(true, 420.0, 420.0, 120.0, 200.0, -30.0),
                PageText.GlideNumbers(100.0, -300.0, 135.0),
            })
            {
                string fit = HeadNumbers.Fit(values, Chars, room);
                Check("a row fits its room", Chars(fit) <= room, fit);
                foreach (string part in fit.Split(HeadNumbers.Gap))
                    Check("a row shows whole values", Array.Exists(values, v => v.Text == part || v.ShortText == part), part);
            }
        }

        HeadValue[] climb = PageText.AscentNumbers(true, 3.0, 420.0, -6362.0, 200.0);
        Check("a climb under ground says so", HeadNumbers.Fit(climb, Chars, Row - 14f * 10.9f) == "AP 3 km, TARGET 420   PE BELOW GROUND", HeadNumbers.Fit(climb, Chars, Row - 14f * 10.9f));
        HeadValue[] final = PageText.AscentNumbers(true, 420.0, 420.0, 120.0, 200.0, 2915.0);
        string finalRow = HeadNumbers.Fit(final, Chars, Row - 14f * 10.9f);
        Check("at the end the periapsis has its target", finalRow == "AP 420 km   PE 120 km, TARGET 200", finalRow);
        Check("a wide row has the margin", HeadNumbers.Fit(final, Chars, 2f * Row).Contains("MARGIN +2915 m/s"));
        Check("a margin above zero is plain", Array.Find(PageText.AscentNumbers(true, 300.0, 420.0, 50.0, 200.0, 10.0), v => v.Text.StartsWith("MARGIN")).Tone == PlanTone.Plain);
        Check("a margin at zero warns", Array.Find(PageText.AscentNumbers(true, 300.0, 420.0, 50.0, 200.0, 0.0), v => v.Text.StartsWith("MARGIN")).Tone == PlanTone.Warn);
        Check("a margin under zero warns", Array.Find(PageText.AscentNumbers(true, 300.0, 420.0, 50.0, 200.0, -40.0), v => v.Text.StartsWith("MARGIN")).Tone == PlanTone.Warn);
        Check("a climb on a plan says what it flies", PlanWords.FlyingPlan(80.0).Text == "AFC: PLAN TO 80 km" && !PlanWords.FlyingPlan(80.0).Good && !PlanWords.FlyingPlan(80.0).Warns);
        Check("no value, no height in the word", PlanWords.FlyingPlan(double.NaN).Text == "AFC: PLAN");
        Check("a simple turn has its word and no green", PlanWords.FlyingTurn.Shows && !PlanWords.FlyingTurn.Good);
        Check("a crash on the site reads as a crash", ImpactMarks.Of(0.0, false, false) == new ImpactMark(PageText.ImpactOnSite, ImpactInk.Bad));
        Check("a crash near the site says how far", ImpactMarks.Of(0.5, false, false) == new ImpactMark("IMPACT 0.5 km OFF", ImpactInk.Bad));
        Check("a far crash says how far", ImpactMarks.Of(72.2, false, false).Word == "IMPACT 72.2 km OFF");
        Check("a mode that says on target has the green word", ImpactMarks.Of(0.5, true, false) == new ImpactMark(PageText.OnSite, ImpactInk.Good));
        Check("a glide that is off stays a crash", ImpactMarks.Of(5.0, true, false).Ink == ImpactInk.Bad);
        Check("a burn to come moves the point, so the mark is plain", ImpactMarks.Of(72.2, false, true) == new ImpactMark(PageText.LandsHereNow, ImpactInk.Plain));
        Check("on target beats a burn to come", ImpactMarks.Of(0.5, true, true).Ink == ImpactInk.Good);
        Check("no value, no distance", ImpactMarks.Of(double.NaN, false, false).Word == PageText.ImpactWord);
        HeadValue[] climbing = PageText.AscentNumbers(true, 192.0, 420.0, -2000.0, 200.0, 2000.0);
        string climbingRow = HeadNumbers.Fit(climbing, Chars, Row - 14f * 10.9f - 8f);
        Check("while the periapsis is under the ground the margin keeps the row", climbingRow.Contains("MARGIN +2000 m/s") && !climbingRow.Contains("PE"), climbingRow);
        Check("the periapsis under the ground stands where there is room", HeadNumbers.Fit(climbing, Chars, 2f * Row).Contains(PageText.PeBelowGround));
        Check("a value holds its width while its number counts", HeadNumbers.HeldWidthText("AP 3 km") == "AP 003 km" && HeadNumbers.HeldWidthText("AP 192 km") == "AP 192 km" && HeadNumbers.HeldWidthText("VS -42.5 m/s") == "VS -042.5 m/s" && HeadNumbers.HeldWidthText("PE BELOW GROUND") == "PE BELOW GROUND");
        List<HeadCell> early = HeadNumbers.Cells(PageText.AscentNumbers(true, 3.0, 420.0, -6000.0, 200.0, 2000.0), Chars, Row - 14f * 10.9f - 8f);
        List<HeadCell> later = HeadNumbers.Cells(PageText.AscentNumbers(true, 192.0, 420.0, -2000.0, 200.0, 1990.0), Chars, Row - 14f * 10.9f - 8f);
        Check("the columns do not move as the numbers count", early.Count == 2 && later.Count == 2 && early[1].X == later[1].X, $"{early.Count} {later.Count}");
        Check("the columns stand apart", later[1].X >= Chars("AP 192 km") + Chars(HeadNumbers.Gap));
        Check("a cell keeps the tone of its value", HeadNumbers.Cells([new HeadValue("MISS 0.3 km", Tone: PlanTone.Good)], Chars, Row)[0].Tone == PlanTone.Good);
        Check("an open path has no apoapsis", HeadNumbers.Fit(PageText.AscentNumbers(false, double.NaN, 420.0, 300.0, 200.0), Chars, Row) == "PE 300 km, TARGET 200");
        string glide = HeadNumbers.Fit(PageText.GlideNumbers(100.0, -300.0, 135.0), Chars, Row - 17f * 10.9f);
        Check("a glide has its miss and the next event", glide == "MISS 0.3 km   ENTRY T-2m 15s", glide);
        Check("with no hand-over the miss has its parts", HeadNumbers.Fit(PageText.GlideNumbers(100.0, -300.0), Chars, Row - 17f * 10.9f) == "MISS 0.3 km, 0.1 LONG, 0.3 RIGHT");
        Check("short and left have their words", PageText.GlideNumbers(-100.0, 300.0)[0].Text == "MISS 0.3 km, 0.1 SHORT, 0.3 LEFT");

        Check("a plan is on plan", PlanWords.Of(true, false) == PlanWords.OnPlan);
        Check("a failed solve flies the last plan", PlanWords.Of(true, false, lastPlan: true) == PlanWords.LastPlan);
        Check("a short wait is planning", PlanWords.Of(false, false) == PlanWords.Planning);
        Check("a long wait is no plan, and it warns", PlanWords.Of(false, true) is { Text: "NO PLAN", Warns: true });
        Check("close to the plan", PlanWords.Tracking(3.0) == PlanWords.OnPlan && PlanWords.Tracking(double.NaN) == PlanWords.OnPlan);
        Check("far from the plan names how far", PlanWords.Tracking(PlanWords.OffPlanM + 20.0).Text == "OFF PLAN 120 m");
        Check("a miss has its word", PlanWords.Miss(0.3) == PlanWords.OnTarget && PlanWords.Miss(5.0) == PlanWords.OffTarget && !PlanWords.Miss(double.NaN).Shows);
        Check("a craft held only later is yours", PlanWords.Yours("WINDOW").Text == "MANUAL UNTIL WINDOW");

        Check("the glide leads to entry when it hands over", GuidanceModes.PhaseChain(GuidanceModes.BoostbackPhasesFor(true), "GLIDE", s => s.Length, 100f).Contains("[ GLIDE ] > ENTRY"));
        Check("and ends when it does not", GuidanceModes.PhaseChain(GuidanceModes.BoostbackPhasesFor(false), "GLIDE", s => s.Length, 100f).Contains("[ GLIDE ] > ENDED"));
        Check("entry with no plan is in its glide", Array.IndexOf(EntryGuidance.Phases, "PLANNING") < 0 && Array.IndexOf(EntryGuidance.Phases, "GLIDE") >= 0);

        var band = new BandFacts("G-FOLD gave up", EngineNow.Off, FlownBy.You, 31.0);
        Check("the row has the whole band with room", FailureBand.RowLine(in band, Chars, Row) == "G-FOLD gave up. ENGINE OFF. MANUAL. IMPACT T-31 s");
        Check("the engine goes first", FailureBand.RowLine(in band, Chars, 45f * 10.9f) == "G-FOLD gave up. MANUAL. IMPACT T-31 s");
        var reason = new BandFacts("G-FOLD: NO TRAJECTORY", EngineNow.Off, FlownBy.You, 73.0);
        Check("then the reason, which the band keeps", FailureBand.RowLine(in reason, Chars, Row - 11f * 10.9f) == "G-FOLD FAILED. MANUAL. IMPACT T-73 s", FailureBand.RowLine(in reason, Chars, Row - 11f * 10.9f));
        Check("what happened and the time stay", FailureBand.RowLine(in band, Chars, 30f * 10.9f) == "G-FOLD FAILED. IMPACT T-31 s");
        var touchdownBand = new BandFacts("TOUCHDOWN: NO PLAN", band.Engine, band.Who, 57.0);
        Check("the name of the mode gets shorter so that the time fits", FailureBand.RowLine(in touchdownBand, Chars, 42f * 10.9f) == "TOUCHDOWN: NO PLAN. MANUAL. IMPACT T-57 s", FailureBand.RowLine(in touchdownBand, Chars, 42f * 10.9f));
        Check("a failure with a short name keeps it", FailureBand.ShortWhat("NO ENTRY PLAN") == "NO ENTRY PLAN");

        Check("a height near the ground has its tenth", Fmt.Height(8.34) == "8.3 m");
        Check("a height has one rule", Fmt.Height(8985.0) == "8985 m" && Fmt.Height(12400.0) == "12.4 km");
        Check("a time keeps its width", Fmt.Time(186.0) == "3m 06s" && Fmt.Time(3900.0) == "1h 05m");
    }

    private static void Check(string name, bool ok, string detail = "")
    {
        _checks++;
        if (ok)
            return;
        _failures++;
        Console.WriteLine("FAIL " + name + (detail.Length > 0 ? ": " + detail : ""));
    }

    private static void StopPriority()
    {
        Check("idle resolves to None", StopResolver.Resolve(new CraftActivity()) == StopTarget.None);
        Check("6-DOF wins over a descent",
            StopResolver.Resolve(new CraftActivity { SixDofEngaged = true, DescentLive = true }) == StopTarget.SixDof);
        Check("landing machine takes the stop from 6-DOF",
            StopResolver.Resolve(new CraftActivity { SixDofEngaged = true, LandingMachineLive = true, DescentLive = true }) == StopTarget.Landing);
        Check("hover has its own stop",
            StopResolver.Resolve(new CraftActivity { DescentLive = true, HoverLive = true }) == StopTarget.Hover);
        Check("landing wins over boostback",
            StopResolver.Resolve(new CraftActivity { DescentLive = true, BoostbackLive = true }) == StopTarget.Landing);
        Check("boostback wins over ascent",
            StopResolver.Resolve(new CraftActivity { BoostbackLive = true, AscentRunning = true }) == StopTarget.Boostback);
        Check("ascent running", StopResolver.Resolve(new CraftActivity { AscentRunning = true }) == StopTarget.Ascent);
        Check("launch armed disarms", StopResolver.Resolve(new CraftActivity { LaunchArmed = true }) == StopTarget.LaunchArmed);
        Check("convex launch pending disarms", StopResolver.Resolve(new CraftActivity { ConvexLaunchPending = true }) == StopTarget.LaunchArmed);
        Check("ascent wins over RCS", StopResolver.Resolve(new CraftActivity { AscentRunning = true, RcsFlying = true }) == StopTarget.Ascent);
        Check("RCS wins over multi-pass", StopResolver.Resolve(new CraftActivity { RcsFlying = true, MultiPassActive = true }) == StopTarget.Rcs);
        Check("multi-pass wins over stock AUTO",
            StopResolver.Resolve(new CraftActivity { MultiPassActive = true, StockAutoOnNode = true }) == StopTarget.MultiPass);
        Check("multi-pass alone", StopResolver.Resolve(new CraftActivity { MultiPassActive = true }) == StopTarget.MultiPass);
        Check("stock AUTO alone", StopResolver.Resolve(new CraftActivity { StockAutoOnNode = true }) == StopTarget.StockAuto);
        Check("stock AUTO is not AFC flying", !new CraftActivity { StockAutoOnNode = true }.AnyLive);

        Check("label SixDof", StopResolver.Label(StopTarget.SixDof) == "STOP TOUCHDOWN");
        Check("label Landing", StopResolver.Label(StopTarget.Landing) == "STOP LANDING");
        Check("label Hover", StopResolver.Label(StopTarget.Hover) == "STOP HOVER");
        Check("label Boostback", StopResolver.Label(StopTarget.Boostback) == "STOP BOOSTBACK");
        Check("label Ascent", StopResolver.Label(StopTarget.Ascent) == "STOP ASCENT");
        Check("label LaunchArmed", StopResolver.Label(StopTarget.LaunchArmed) == "STOP LAUNCH");
        Check("label Rcs", StopResolver.Label(StopTarget.Rcs) == "STOP RCS BURN");
        Check("label MultiPass", StopResolver.Label(StopTarget.MultiPass) == "STOP PASSES");
        Check("label StockAuto", StopResolver.Label(StopTarget.StockAuto) == "STOP STOCK AUTO");
        Check("nothing running has no stop label", StopResolver.Label(StopTarget.None).Length == 0);
        Check("no stop on offer has no label", StopSpec.None.Label.Length == 0 && !StopSpec.None.Enabled);
        foreach (StopTarget t in Enum.GetValues<StopTarget>())
        {
            Check("stop label is upper case " + t, StopResolver.Label(t) == StopResolver.Label(t).ToUpperInvariant());
            if (t != StopTarget.None)
            {
                Check("every stop has the one verb " + t, StopResolver.Label(t).StartsWith("STOP ", StringComparison.Ordinal));
                Check("the sizes of the stop know its label " + t, Array.IndexOf(StopResolver.BuiltInLabels, StopResolver.Label(t)) >= 0);
                // At the narrowest window a mode name has what the chip and the stop leave: about 155 px at 9.1 px per character with a label of 15.
                Check("a stop label leaves room for the mode name " + t, StopResolver.Label(t).Length <= 15);
            }
        }

        // Every landing way against each other: the stop follows what flies, not which way the page shows.
        var deorbitChain = new CraftActivity { DescentLive = true };
        var gfold = new CraftActivity { DescentLive = true, LandingMachineLive = true };
        var hover = new CraftActivity { DescentLive = true, HoverLive = true };
        var sixDof = new CraftActivity { SixDofEngaged = true };
        Check("deorbit chain stops as landing", StopResolver.Resolve(deorbitChain) == StopTarget.Landing);
        Check("G-FOLD stops as landing", StopResolver.Resolve(gfold) == StopTarget.Landing);
        Check("hover stops as hover", StopResolver.Resolve(hover) == StopTarget.Hover);
        Check("6-DOF stops as 6-DOF", StopResolver.Resolve(sixDof) == StopTarget.SixDof);
    }

    private static void EngineOutcomes()
    {
        var idle = new CraftActivity();
        var held = new CraftActivity { ControlAcquired = true };
        Check("ascent with control cuts", StopResolver.EngineOutcome(StopTarget.Ascent, held) == "ENGINE CUT. MANUAL.");
        Check("ascent without control leaves the engine", StopResolver.EngineOutcome(StopTarget.Ascent, idle) == "ENGINE UNCHANGED. MANUAL.");
        Check("short ascent outcome", StopResolver.OutcomeShort(StopTarget.Ascent, held) == "ENGINE CUT");
        Check("short armed outcome", StopResolver.OutcomeShort(StopTarget.LaunchArmed, idle) == "NO LAUNCH. ENGINE UNCHANGED.");
        Check("landing in the air keeps the engine",
            StopResolver.EngineOutcome(StopTarget.Landing, new CraftActivity { LandingAirborne = true })
            == "ENGINE UNCHANGED. MANUAL");
        Check("a landing in flight reads the throttle", StopResolver.OutcomeShort(StopTarget.Landing, new CraftActivity { LandingAirborne = true, EnginePercent = 10.2 }) == "ENGINE 10 %"
            && StopResolver.EngineOutcome(StopTarget.Landing, new CraftActivity { LandingAirborne = true, EnginePercent = 10.2 }) == "ENGINE 10 %. MANUAL");
        Check("a landing in flight with the engine off says that the craft falls", StopResolver.OutcomeShort(StopTarget.Landing, new CraftActivity { LandingAirborne = true, EngineOff = true, EnginePercent = 0.0 }) == "ENGINE OFF"
            && StopResolver.EngineOutcome(StopTarget.Landing, new CraftActivity { LandingAirborne = true, EngineOff = true }) == "ENGINE OFF. MANUAL. CRAFT FALLS.");
        Check("no stop card speaks of air", !StopResolver.EngineOutcome(StopTarget.Hover, idle).Contains("air", StringComparison.Ordinal));
        Check("short landing in the air", StopResolver.OutcomeShort(StopTarget.Landing, new CraftActivity { LandingAirborne = true }) == "ENGINE UNCHANGED");
        Check("a landing that holds the craft cuts", StopResolver.OutcomeShort(StopTarget.Landing, held) == "ENGINE CUT" && StopResolver.EngineOutcome(StopTarget.Landing, held) == "ENGINE CUT. MANUAL.");
        Check("a landing whose deorbit stock AUTO burns cuts and deletes the burn", StopResolver.OutcomeShort(StopTarget.Landing, new CraftActivity { LandingStockAuto = true }) == "ENGINE CUT, BURN DELETED");
        Check("the wait for that burn has nothing to cut", StopResolver.OutcomeShort(StopTarget.Landing, new CraftActivity { LandingStockAuto = true, EngineOff = true }) == "ENGINE OFF, BURN DELETED");
        Check("a glide or a turn has nothing to cut", StopResolver.OutcomeShort(StopTarget.Boostback, held with { EngineOff = true }) == "ENGINE OFF"
            && StopResolver.OutcomeLine(StopTarget.Boostback, held with { EngineOff = true }) == "ON STOP: ENGINE OFF"
            && StopResolver.EngineOutcome(StopTarget.Boostback, held with { EngineOff = true }).StartsWith("ENGINE OFF", StringComparison.Ordinal));
        Check("an engine that burns is cut", StopResolver.OutcomeShort(StopTarget.Boostback, held) == "ENGINE CUT");
        Check("a craft nothing holds is not touched, off or on", StopResolver.OutcomeShort(StopTarget.Ascent, idle with { EngineOff = true }) == "ENGINE UNCHANGED");
        Check("a stop in flight names an engine that is off", StopResolver.OutcomeShort(StopTarget.Landing, new CraftActivity { LandingAirborne = true, EngineOff = true }) == "ENGINE OFF");
        Check("a waiting coast leaves the engine", StopResolver.OutcomeShort(StopTarget.Landing, idle) == "ENGINE UNCHANGED"
            && StopResolver.EngineOutcome(StopTarget.Landing, idle).StartsWith("Engine unchanged", StringComparison.Ordinal));
        Check("hover keeps the engine", StopResolver.OutcomeShort(StopTarget.Hover, idle) == "ENGINE UNCHANGED");
        Check("6-DOF with control cuts",
            StopResolver.EngineOutcome(StopTarget.SixDof, held) == "ENGINE CUT. MANUAL. CRAFT FALLS.");
        Check("6-DOF without control leaves the engine",
            StopResolver.EngineOutcome(StopTarget.SixDof, idle) == "ENGINE UNCHANGED. MANUAL.");
        Check("RCS outcome", StopResolver.EngineOutcome(StopTarget.Rcs, idle) == "RCS stops. STOCK AUTO does not return. The engine is unchanged.");
        Check("multi-pass outcome",
            StopResolver.EngineOutcome(StopTarget.MultiPass, idle) == "Removes the queued and later passes. Flown burns stay.");
        Check("short multi-pass outcome", StopResolver.OutcomeShort(StopTarget.MultiPass, idle) == "DELETES REMAINING PASSES");
        Check("a pass that stock AUTO flies says the engine goes off",
            StopResolver.OutcomeShort(StopTarget.MultiPass, new CraftActivity { StockAutoOnNode = true }) == "DELETES REMAINING PASSES. ENGINE OFF.");
        Check("stock AUTO outcome", StopResolver.OutcomeShort(StopTarget.StockAuto, idle) == "ENGINE OFF. BURN STAYS.");

        Check("the line says on stop", StopResolver.OutcomeLine(StopTarget.Ascent, held) == "ON STOP: ENGINE CUT");
        Check("the line keeps a word in capitals", StopResolver.OutcomeLine(StopTarget.Rcs, idle) == "ON STOP: RCS STOPS. ENGINE UNCHANGED.");
        Check("no stop has no line", StopResolver.OutcomeLine(StopTarget.None, idle).Length == 0);
        foreach (StopTarget t in Enum.GetValues<StopTarget>())
        {
            if (t == StopTarget.None)
                continue;
            foreach (CraftActivity a in new[] { idle, held, new CraftActivity { LandingAirborne = true }, new CraftActivity { StockAutoOnNode = true }, new CraftActivity { LandingStockAuto = true } })
            {
                Check("every stop has a line " + t, StopResolver.OutcomeLine(t, a).StartsWith("ON STOP: ", StringComparison.Ordinal));
                // At the narrowest window a head row is 426 px at 9.1 px per character.
                Check("the line fits the narrow head " + t, StopResolver.OutcomeLine(t, a).Length <= 46);
                Check("a full stop ends only a line of two parts " + t, !StopResolver.OutcomeLine(t, a).EndsWith(".", StringComparison.Ordinal) || StopResolver.OutcomeLine(t, a).Count(ch => ch == '.') >= 2);
            }
        }
        Check("release outcome names the cut", StopResolver.ReleaseOutcome.Contains("cuts the engine"));
        Check("release on an idle craft cuts nothing", StopResolver.ReleaseOutcomeFor(idle) == StopResolver.ReleaseIdleOutcome);
        Check("release with control held names the cut",
            StopResolver.ReleaseOutcomeFor(held) == StopResolver.ReleaseOutcome);
        Check("nothing to release on an idle craft", StopResolver.NothingToRelease(idle));
        Check("release while control is held", !StopResolver.NothingToRelease(held));
        Check("release while a mode flies", !StopResolver.NothingToRelease(new CraftActivity { BoostbackLive = true }));
        Check("multi-pass alone needs no release", StopResolver.NothingToRelease(new CraftActivity { MultiPassActive = true }));
        Check("release while guidance state is left", !StopResolver.NothingToRelease(new CraftActivity { ReleaseWouldAct = true }));
        Check("release on leftover state cuts nothing",
            StopResolver.ReleaseOutcomeFor(new CraftActivity { ReleaseWouldAct = true }) == StopResolver.ReleaseIdleOutcome);
    }

    private static readonly CommitGates Open = new(false, false, "", "", false);

    private static void CommitRules()
    {
        var idle = new CraftActivity();

        CommitView v = CommitResolver.Resolve(GuidanceModeId.AscentUpfg, idle, Open);
        Check("ascent without target launches", v.Verb == "LAUNCH" && v.Enabled);
        v = CommitResolver.Resolve(GuidanceModeId.AscentUpfg, idle, Open with { HasTarget = true });
        Check("ascent with target arms", v.Verb == "ARM" && v.Enabled);
        v = CommitResolver.Resolve(GuidanceModeId.AscentUpfg, idle, Open with { PlansFirst = true });
        Check("ascent without a fitting plan calculates and launches", v.Verb == "SOLVE + LAUNCH" && v.Enabled);
        v = CommitResolver.Resolve(GuidanceModeId.AscentUpfg, idle, Open with { PlansFirst = true, HasTarget = true });
        Check("ascent without a fitting plan calculates and arms", v.Verb == "SOLVE + ARM" && v.Enabled);
        v = CommitResolver.Resolve(GuidanceModeId.AscentUpfg, idle, Open with { PlansFirst = true, BackupChosen = true });
        Check("the backup choice makes the footer verb LAUNCH (BACKUP)", v.Verb == CommitResolver.BackupLaunchVerb && v.Enabled);
        Check("ascent running refuses",
            CommitResolver.Resolve(GuidanceModeId.AscentUpfg, new CraftActivity { AscentRunning = true }, Open).Reason == "AFC: ASCENT");
        Check("ascent armed refuses",
            CommitResolver.Resolve(GuidanceModeId.AscentUpfg, new CraftActivity { LaunchArmed = true }, Open).Reason == "LAUNCH ARMED");
        Check("ascent planning refuses",
            CommitResolver.Resolve(GuidanceModeId.AscentUpfg, new CraftActivity { ConvexLaunchPending = true }, Open).Reason == "PLANNING ASCENT");
        Check("running ascent is live", CommitResolver.Resolve(GuidanceModeId.AscentUpfg, new CraftActivity { AscentRunning = true }, Open).Live);

        v = CommitResolver.Resolve(GuidanceModeId.AscentGravityTurn, idle,
            Open with { GravityTurnRefusal = CommitResolver.GravityTurnUnavailable });
        Check("gravity turn unavailable", v.Verb == "LAUNCH" && !v.Enabled && v.Reason == CommitResolver.GravityTurnUnavailable);
        Check("gravity turn reason names the mode", CommitResolver.GravityTurnUnavailable == "NEEDS GRAVITY TURN");
        Check("gravity turn slot stays refused while its descriptor is unavailable",
            !CommitResolver.Resolve(GuidanceModeId.AscentGravityTurn, idle, Open).Enabled);

        v = CommitResolver.Resolve(GuidanceModeId.Boostback, idle, Open);
        Check("boostback verb", v.Verb == "BOOSTBACK" && v.Enabled);
        Check("boostback running refuses",
            CommitResolver.Resolve(GuidanceModeId.Boostback, new CraftActivity { BoostbackLive = true }, Open).Reason == "AFC: BOOSTBACK");

        v = CommitResolver.Resolve(GuidanceModeId.DeorbitLand, idle, Open);
        Check("deorbit verb", v.Verb == "DEORBIT + LAND" && v.Enabled);
        v = CommitResolver.Resolve(GuidanceModeId.DeorbitLand, idle, Open with { LandingSwitchesOff = true });
        Check("deorbit switches off refuses", v.Reason == CommitResolver.LandingSwitchesOffReason);
        Check("deorbit switches off names both switches",
            v.Detail.Contains(GuidanceSettingSpecs.Engage.Label) && v.Detail.Contains(GuidanceSettingSpecs.AutoStage.Label));
        v = CommitResolver.Resolve(GuidanceModeId.DeorbitLand, idle, Open with { DeorbitRefusal = "Check it." });
        Check("deorbit refusal is short", v.Reason == CommitResolver.DeorbitSettingsReason);
        Check("deorbit refusal text in the detail", v.Detail == "Check it.");
        Check("deorbit running refuses",
            CommitResolver.Resolve(GuidanceModeId.DeorbitLand, new CraftActivity { DescentLive = true }, Open).Reason == "AFC: LANDING");
        Check("deorbit switches checked before the running state",
            CommitResolver.Resolve(GuidanceModeId.DeorbitLand, new CraftActivity { DescentLive = true },
                Open with { LandingSwitchesOff = true }).Reason == CommitResolver.LandingSwitchesOffReason);

        v = CommitResolver.Resolve(GuidanceModeId.GfoldHere, idle, Open);
        Check("the G-FOLD verb names the solver", v.Verb == "G-FOLD" && v.Verb == CommitResolver.GfoldVerb && v.Enabled);
        v = CommitResolver.Resolve(GuidanceModeId.GfoldHere,
            new CraftActivity { LandingMachineLive = true, TerminalBurnActive = true, DescentLive = true }, Open);
        Check("land now refuses during the terminal burn", !v.Enabled && v.Reason == "AFC: POWERED DESCENT" && v.Live);
        v = CommitResolver.Resolve(GuidanceModeId.GfoldHere,
            new CraftActivity { LandingMachineLive = true, DescentLive = true }, Open);
        Check("a flying powered descent is not planned again", !v.Enabled && v.Reason == "AFC: POWERED DESCENT" && v.Live);

        v = CommitResolver.Resolve(GuidanceModeId.SixDofHere, idle, Open);
        Check("touchdown verb", v.Verb == "TOUCHDOWN" && v.Enabled);
        v = CommitResolver.Resolve(GuidanceModeId.SixDofHere, new CraftActivity { SixDofEngaged = true }, Open);
        Check("engaged touchdown guidance is not planned again", !v.Enabled && v.Reason == "AFC: TOUCHDOWN" && v.Live);

        foreach (string reason in new[] { CommitResolver.GravityTurnUnavailable, CommitResolver.LandingSwitchesOffReason, CommitResolver.DeorbitSettingsReason })
            Check("footer reason is short and in capitals: " + reason, reason.Length <= 40 && reason == reason.ToUpperInvariant());

        v = CommitResolver.Resolve(GuidanceModeId.Hover, idle, Open);
        Check("hover verb", v.Verb == "HOVER" && v.Enabled);
        v = CommitResolver.Resolve(GuidanceModeId.Hover, new CraftActivity { HoverLive = true, DescentLive = true }, Open);
        Check("a running hover is not started again", !v.Enabled && v.Reason == "AFC: HOVER" && v.Live);
        Check("hover low TWR refuses",
            CommitResolver.Resolve(GuidanceModeId.Hover, idle, Open with { HoverTwrBelowOne = true }).Reason
            == "NO HOVER: TWR BELOW 1");
    }

    private static void TakeOver()
    {
        var ascent = new CraftActivity { AscentRunning = true };
        CommitView v = CommitResolver.Resolve(GuidanceModeId.Boostback, ascent, Open);
        Check("boostback over a running ascent asks first", v.NeedsTakeOver && v.ButtonLabel == "TAKE OVER: BOOSTBACK");
        Check("same mode never asks",
            !CommitResolver.Resolve(GuidanceModeId.Hover, new CraftActivity { DescentLive = true, LandingMachineLive = true }, Open).NeedsTakeOver);
        Check("land now over 6-DOF asks",
            CommitResolver.Resolve(GuidanceModeId.GfoldHere, new CraftActivity { SixDofEngaged = true }, Open).NeedsTakeOver);
        Check("land now over the deorbit chain asks",
            CommitResolver.Resolve(GuidanceModeId.GfoldHere, new CraftActivity { DescentLive = true }, Open).NeedsTakeOver);
        Check("land now over the hover asks",
            CommitResolver.Resolve(GuidanceModeId.GfoldHere, new CraftActivity { DescentLive = true, HoverLive = true }, Open).NeedsTakeOver);
        Check("hover over the deorbit chain asks",
            CommitResolver.Resolve(GuidanceModeId.Hover, new CraftActivity { DescentLive = true }, Open).NeedsTakeOver);
        Check("hover over the terminal burn does not ask",
            !CommitResolver.Resolve(GuidanceModeId.Hover,
                new CraftActivity { DescentLive = true, LandingMachineLive = true, TerminalBurnActive = true }, Open).NeedsTakeOver);
        Check("a disabled commit never asks",
            !CommitResolver.Resolve(GuidanceModeId.AscentUpfg, ascent, Open).NeedsTakeOver);
        Check("RCS is not a take-over",
            !CommitResolver.Resolve(GuidanceModeId.AscentUpfg, new CraftActivity { RcsFlying = true }, Open).NeedsTakeOver);
        Check("multi-pass is not a take-over",
            !CommitResolver.Resolve(GuidanceModeId.DeorbitLand, new CraftActivity { MultiPassActive = true }, Open).NeedsTakeOver);
        Check("take-over message",
            CommitResolver.TakeOverMessage(StopTarget.Ascent, "LAND NOW") == "LAND NOW ENDS ASCENT. CONTINUE?");
    }

    private static void TakeOverQuestion()
    {
        TakeOverAsk.Clear();
        var start = new DateTime(2026, 10, 31, 12, 0, 0, DateTimeKind.Utc);
        DateTime later = start.AddSeconds(TakeOverAsk.ArmDelaySeconds + 0.1);
        int asked = 0, confirmed = 0;
        string from = CommitResolver.ModeName(StopTarget.SixDof);
        string status = from + " flies now";
        var page = new CommitSpec("G-FOLD", FlownBy.Afc, "", status, true, "tip", true, () => asked++, Owner.Afc, "Flies a G-FOLD descent.", TakeOverFrom: from);

        Check("no question leaves the commit alone", ReferenceEquals(TakeOverAsk.Apply(page, start), page));

        TakeOverAsk.Raise(page.Verb, from, "Ends it. Continue?", () => confirmed++, start);
        CommitSpec ask = TakeOverAsk.Apply(page, start);
        Check("the verb asks to confirm", ask.ButtonLabel == "CONFIRM: TAKE OVER" && !ask.NeedsTakeOver);
        Check("the answer row asks", ask.Status == "Ends it. Continue?" && ask.Outcome.Length == 0 && ask.Refusal.Length == 0);
        Check("a press that lands at once does not count", !ask.Enabled);
        Check("cancel stands beside it as an outline", ask.Secondary is { Verb: "CANCEL", Enabled: true, Danger: false });
        Check("the question leaves the commit of the page alone", page.NeedsTakeOver && page.Status == status);

        ask = TakeOverAsk.Apply(page, later);
        Check("the verb arms after the delay", ask.Enabled);
        ask.Act!();
        Check("confirm runs the action once", confirmed == 1 && asked == 0);
        Check("confirm ends the question", !TakeOverAsk.Active);

        TakeOverAsk.Raise(page.Verb, from, "Q", () => confirmed++, start);
        TakeOverAsk.Apply(page, later).Secondary!.Act!();
        Check("cancel ends the question and runs nothing", !TakeOverAsk.Active && confirmed == 1);

        TakeOverAsk.Raise(page.Verb, from, "Q", () => confirmed++, start);
        CommitSpec plain = new("LAUNCH", FlownBy.Afc, Act: () => { });
        Check("a page with no take-over ends the question", ReferenceEquals(TakeOverAsk.Apply(plain, later), plain) && !TakeOverAsk.Active);

        TakeOverAsk.Raise(page.Verb, from, "Q", () => confirmed++, start);
        CommitSpec other = page with { Verb = "HOVER" };
        Check("another verb ends the question", ReferenceEquals(TakeOverAsk.Apply(other, later), other) && !TakeOverAsk.Active);

        TakeOverAsk.Raise(page.Verb, from, "Q", () => confirmed++, start);
        CommitSpec otherMode = page with { TakeOverFrom = CommitResolver.ModeName(StopTarget.Hover) };
        Check("another running mode ends the question", ReferenceEquals(TakeOverAsk.Apply(otherMode, later), otherMode) && !TakeOverAsk.Active);

        TakeOverAsk.Raise(page.Verb, from, "Q", () => confirmed++, start);
        CommitSpec newWords = page with { Status = "Other words" };
        Check("new words in the answer row leave the question", TakeOverAsk.Apply(newWords, later).Verb == "CONFIRM: TAKE OVER" && TakeOverAsk.Active);
        TakeOverAsk.Clear();
    }

    private static void StatusOrdering()
    {
        Check("plain lines of one craft within a moment are one event", StatusSelector.SameEvent(Severity.Info, Severity.Info, true, 0.2));
        Check("plain lines apart are two events", !StatusSelector.SameEvent(Severity.Info, Severity.Info, true, StatusSelector.SameEventSeconds + 0.1));
        Check("a warning stands on its own", !StatusSelector.SameEvent(Severity.Warning, Severity.Info, true, 0.2) && !StatusSelector.SameEvent(Severity.Info, Severity.Warning, true, 0.2) && !StatusSelector.SameEvent(Severity.Info, Severity.Error, true, 0.2) && !StatusSelector.SameEvent(Severity.Info, Severity.Cue, true, 0.2));
        Check("lines of two craft are two events", !StatusSelector.SameEvent(Severity.Info, Severity.Info, false, 0.2));
        var lines = new List<StatusLine>
        {
            new(Severity.Info, "first info", ""),
            new(Severity.Warning, "warn", ""),
            new(Severity.Info, "", ""),
            new(Severity.Warning, "second warn", ""),
            new(Severity.Cue, "cue", "LAND/HOVER"),
        };
        Check("highest severity wins", StatusSelector.Pick(lines) == 1);
        Check("more count skips empty lines", StatusSelector.MoreCount(lines) == 3);
        lines.Add(new StatusLine(Severity.Error, "error", ""));
        Check("error wins", StatusSelector.Pick(lines) == 5);
        Check("empty list picks nothing", StatusSelector.Pick(new List<StatusLine>()) == -1);
        Check("only empty lines pick nothing", StatusSelector.Pick(new List<StatusLine> { new(Severity.Error, "", "") }) == -1);
        Check("solver text never reaches the head", !StatusSelector.ShowsInHead(Severity.Diag, true, 0.0, false));
        Check("a line stays while a mode flies", StatusSelector.ShowsInHead(Severity.Info, true, 1000.0, false)
            && StatusSelector.ShowsInHead(Severity.Warning, true, 1000.0, false) && StatusSelector.ShowsInHead(Severity.Error, true, 1000.0, true));
        Check("a note fades once nothing flies", StatusSelector.ShowsInHead(Severity.Info, false, StatusSelector.EndedNoteSeconds - 1.0, false)
            && !StatusSelector.ShowsInHead(Severity.Info, false, StatusSelector.EndedNoteSeconds, false));
        Check("a warning fades later than a note", StatusSelector.ShowsInHead(Severity.Warning, false, StatusSelector.EndedNoteSeconds, false)
            && !StatusSelector.ShowsInHead(Severity.Warning, false, StatusSelector.EndedWarningSeconds, false));
        Check("a failure stays until it is acknowledged", StatusSelector.ShowsInHead(Severity.Error, false, 1000.0, false)
            && !StatusSelector.ShowsInHead(Severity.Error, false, 0.0, true));
        Check("a cue keeps its own timer", StatusSelector.ShowsInHead(Severity.Cue, false, 1000.0, true));
        double realWait = StatusSelector.NoPlanWaitRealSeconds;
        Check("a short no-plan state is no failure", !StatusSelector.NoPlanSustained(StatusSelector.NoPlanWaitSeconds - 0.1, realWait)
            && StatusSelector.NoPlanSustained(StatusSelector.NoPlanWaitSeconds, realWait) && !StatusSelector.NoPlanSustained(double.NaN, realWait));
        Check("a first plan under a time warp is no failure", !StatusSelector.NoPlanSustained(StatusSelector.NoPlanWaitSeconds * 10.0, realWait - 0.1));
        Check("a short step fault is no failure", !StatusSelector.GuidanceFaultSustained(StatusSelector.SustainedFailSteps - 1)
            && StatusSelector.GuidanceFaultSustained(StatusSelector.SustainedFailSteps));
        int lastingStreak = StatusSelector.SustainedFailSteps;
        Check("a step fault that lasts is a failure while its step runs", StatusSelector.StepFaultLasts(true, "boom", lastingStreak));
        Check("a step fault says nothing once its step ended", !StatusSelector.StepFaultLasts(false, "boom", lastingStreak + 1000));
        Check("a short or empty step fault is no failure", !StatusSelector.StepFaultLasts(true, "boom", lastingStreak - 1) && !StatusSelector.StepFaultLasts(true, "", lastingStreak));

        var ages = new LineAges();
        var noneAcknowledged = new HashSet<string>();
        var aged = new List<StatusLine> { new(Severity.Info, "note", ""), new(Severity.Warning, "warning", "") };
        ages.Filter(aged, false, noneAcknowledged, 100.0);
        Check("new lines stay", aged.Count == 2);
        aged = new List<StatusLine> { new(Severity.Info, "note", ""), new(Severity.Warning, "warning", "") };
        ages.Filter(aged, false, noneAcknowledged, 100.0 + StatusSelector.EndedNoteSeconds);
        Check("the note goes first", aged.Count == 1 && aged[0].Text == "warning");
        aged = new List<StatusLine> { new(Severity.Info, "note", ""), new(Severity.Warning, "warning", "") };
        ages.Filter(aged, true, noneAcknowledged, 100.0 + StatusSelector.EndedNoteSeconds);
        Check("a mode that flies keeps both", aged.Count == 2);
        ages.Filter(new List<StatusLine>(), false, noneAcknowledged, 200.0);
        aged = new List<StatusLine> { new(Severity.Info, "note", "") };
        ages.Filter(aged, false, noneAcknowledged, 201.0);
        Check("a text that came back counts anew", aged.Count == 1);
        var acknowledgedLine = new HashSet<string> { "gave up" };
        aged = new List<StatusLine> { new(Severity.Error, "gave up", ""), new(Severity.Error, "rcs fault", "") };
        ages.Filter(aged, false, acknowledgedLine, 202.0);
        Check("only the line of the acknowledged failure leaves", aged.Count == 1 && aged[0].Text == "rcs fault");
        aged = new List<StatusLine> { new(Severity.Error, "gave up", ""), new(Severity.Error, "rcs fault", "") };
        ages.Filter(aged, true, acknowledgedLine, 203.0);
        Check("a mode that flies keeps an acknowledged line", aged.Count == 2);

        Check("the band says what, engine, who and impact", FailureBand.Text(new BandFacts("G-FOLD gave up", EngineNow.Off, FlownBy.You, 31.2))
            == "G-FOLD gave up. ENGINE OFF. MANUAL. IMPACT T-31 s", FailureBand.Text(new BandFacts("G-FOLD gave up", EngineNow.Off, FlownBy.You, 31.2)));
        Check("the band leaves out an unknown impact", FailureBand.Text(new BandFacts("LANDING STOPPED", EngineNow.On, FlownBy.Afc))
            == "LANDING STOPPED. ENGINE ON. AFC.");
        Check("the band leaves out an impact in the past", !FailureBand.Text(new BandFacts("x", EngineNow.Off, FlownBy.You, -3.0)).Contains("IMPACT"));
        Check("stock AUTO can hold the engine", FailureBand.Text(new BandFacts("x", EngineNow.StockAuto, FlownBy.StockAuto)).Contains("ENGINE: STOCK AUTO"));

        var acks = new FailureAcks();
        var gaveUp = new CraftFailure("G-FOLD gave up", "", Serial: 1);
        Check("a failure starts unacknowledged", !acks.IsAcknowledged("craft", gaveUp.What, gaveUp.Serial));
        acks.Acknowledge("craft", gaveUp.What, gaveUp.Serial);
        acks.Update("craft", new[] { gaveUp });
        Check("an acknowledged failure stays away", acks.IsAcknowledged("craft", gaveUp.What, gaveUp.Serial));
        Check("the acknowledgement is per craft", !acks.IsAcknowledged("other", gaveUp.What, gaveUp.Serial));
        var gaveUpAgain = gaveUp with { Serial = 2 };
        acks.Update("craft", new[] { gaveUpAgain });
        Check("a press that fails again with the same words is a new failure, with no frame between the two", !acks.IsAcknowledged("craft", gaveUpAgain.What, gaveUpAgain.Serial));
        acks.Acknowledge("craft", gaveUpAgain.What, gaveUpAgain.Serial);
        var rcsFault = new CraftFailure("RCS BURN STOPPED: ERROR", "");
        acks.Update("craft", new[] { gaveUpAgain, rcsFault });
        Check("an acknowledged failure leaves a later one free", acks.IsAcknowledged("craft", gaveUpAgain.What, gaveUpAgain.Serial) && !acks.IsAcknowledged("craft", rcsFault.What, rcsFault.Serial));
        acks.Acknowledge("craft", rcsFault.What, rcsFault.Serial);
        Check("two failures can stand acknowledged", acks.IsAcknowledged("craft", gaveUpAgain.What, gaveUpAgain.Serial) && acks.IsAcknowledged("craft", rcsFault.What, rcsFault.Serial));
        acks.Update("craft", new[] { rcsFault });
        Check("a failure that ended forgets only its own acknowledgement", !acks.IsAcknowledged("craft", gaveUpAgain.What, gaveUpAgain.Serial) && acks.IsAcknowledged("craft", rcsFault.What, rcsFault.Serial));
        acks.Update("craft", Array.Empty<CraftFailure>());
        Check("the same words after the end are a new failure", !acks.IsAcknowledged("craft", rcsFault.What, rcsFault.Serial));
        Check("no failure is never acknowledged", !acks.IsAcknowledged("craft", "", 0));

        Check("nothing runs: you fly", FlownBy.Now(new CraftActivity()) == FlownBy.You);
        Check("a mode flies: AFC flies", FlownBy.Now(new CraftActivity { AscentRunning = true }) == FlownBy.Afc
            && FlownBy.Now(new CraftActivity { DescentLive = true }) == FlownBy.Afc && FlownBy.Now(new CraftActivity { RcsFlying = true }) == FlownBy.Afc);
        Check("a deorbit burn belongs to stock AUTO", FlownBy.Now(new CraftActivity { DescentLive = true, LandingStockAuto = true }) == FlownBy.AfcPlansStockAuto);
        Check("a launch that waits with no control is flown by the pilot", FlownBy.Now(new CraftActivity { LaunchArmed = true, HoldsNoControl = true }) == FlownBy.You
            && FlownBy.Now(new CraftActivity { ConvexLaunchPending = true, HoldsNoControl = true }) == FlownBy.You);
        Check("a landing that coasts with no control is flown by the pilot", FlownBy.Now(new CraftActivity { DescentLive = true, HoldsNoControl = true }) == FlownBy.You);
        Check("a wait that holds control stays with AFC", FlownBy.Now(new CraftActivity { LaunchArmed = true }) == FlownBy.Afc);
        Check("stock AUTO on the deorbit burn is not the pilot", FlownBy.Now(new CraftActivity { DescentLive = true, LandingStockAuto = true, HoldsNoControl = true }) == FlownBy.AfcPlansStockAuto);
        Check("an RCS burn stays with AFC beside a mode with no control", FlownBy.Now(new CraftActivity { LaunchArmed = true, HoldsNoControl = true, RcsFlying = true }) == FlownBy.Afc);
        Check("no live mode, no word about a mode", !new CraftActivity { HoldsNoControl = true }.PilotHolds && new CraftActivity { LaunchArmed = true, HoldsNoControl = true }.PilotHolds);
        Check("the card follows the chip", PageText.HeadOwnerCard(FlownBy.You, pilotHolds: true).Contains("no AFC hold") && !PageText.HeadOwnerCard(FlownBy.You).Contains("no AFC hold"));
        Check("a pass that waits is flown by the pilot", FlownBy.Now(new CraftActivity { MultiPassActive = true }) == FlownBy.You);
        Check("a pass under stock AUTO has both", FlownBy.Now(new CraftActivity { MultiPassActive = true, StockAutoOnNode = true }) == FlownBy.AfcPlansStockAuto);
        Check("a node under stock AUTO", FlownBy.Now(new CraftActivity { StockAutoOnNode = true }) == FlownBy.StockAuto);
        Check("the words read as a sentence", FlownBy.Sentence(FlownBy.You) == "MANUAL" && FlownBy.Sentence(FlownBy.Afc) == "AFC"
            && FlownBy.Sentence(FlownBy.StockAuto) == "STOCK AUTO" && FlownBy.Sentence(FlownBy.AfcPlansStockAuto) == "AFC PLANS + STOCK AUTO");

        Check("the deorbit burn is no landing that flies", CommitResolver.Resolve(GuidanceModeId.DeorbitLand, new CraftActivity { DescentLive = true, LandingStockAuto = true },
            new CommitGates(false, false, "", "", false)).Reason == CommitResolver.StockDeorbitStatus);
        Check("the landing itself flies after the burn", CommitResolver.Resolve(GuidanceModeId.DeorbitLand, new CraftActivity { DescentLive = true },
            new CommitGates(false, false, "", "", false)).Reason == "AFC: LANDING");
    }

    private static void SettingMathRules()
    {
        var spec = new SettingSpec("X", "X", "deg", UiTier.Expert, 5.0, 0.0, 10.0, "%.2f", "");
        Check("clamp low", SettingMath.Clamp(spec, -1.0) == 0.0);
        Check("clamp high", SettingMath.Clamp(spec, 11.0) == 10.0);
        Check("clamp inside", SettingMath.Clamp(spec, 7.5) == 7.5);
        Check("clamp NaN to default", SettingMath.Clamp(spec, double.NaN) == 5.0);
        Check("unchanged at default", !SettingMath.IsChanged(spec, 5.0));
        Check("unchanged at rounding noise", !SettingMath.IsChanged(spec, 5.0 + 1e-12));
        Check("changed", SettingMath.IsChanged(spec, 5.01));
        var small = new SettingSpec("Y", "Y", "", UiTier.Tuning, 0.002, 0.0, 1.0, "%.3f", "");
        Check("small default compares at absolute scale", SettingMath.IsChanged(small, 0.0021));
        Check("bool changed", SettingMath.IsChanged(new BoolSpec("B", "B", UiTier.Expert, true, ""), false));
        Check("bool unchanged", !SettingMath.IsChanged(new BoolSpec("B", "B", UiTier.Expert, true, ""), true));
        Check("ghost text", SettingMath.GhostText(spec) == "DEFAULT 5.00 deg", SettingMath.GhostText(spec));
        Check("ghost text without unit", SettingMath.GhostText(small) == "DEFAULT 0.002", SettingMath.GhostText(small));
        Check("display label keeps the SI case of its unit", SettingMath.DisplayLabel(spec) == "X (deg)");
        Check("format of a count", SettingMath.DotNetFormat("%.0f") == "F0");
        Check("format fallback", SettingMath.DotNetFormat("%d") == "G");
    }

    private static void SpecKeysAreUnique()
    {
        var seen = new HashSet<string>();
        foreach (SettingSpec s in GuidanceSettingSpecs.AllDoubles)
            Check("unique key " + s.Key, seen.Add(s.Key));
        foreach (BoolSpec b in GuidanceSettingSpecs.AllBools)
            Check("unique key " + b.Key, seen.Add(b.Key));
        foreach (SettingSpec s in GuidanceSettingSpecs.AllDoubles)
        {
            Check("default inside range " + s.Key, s.Default >= s.Min && s.Default <= s.Max);
            Check("label is upper case " + s.Key, s.Label.Replace("AoA", "AOA", StringComparison.Ordinal) == s.Label.ToUpperInvariant());
            Check("format is printf " + s.Key, s.Format.StartsWith('%'));
        }
    }

    private static void TierArraysHoldOneTier()
    {
        AllOf("LaunchExpert", GuidanceSettingSpecs.LaunchExpert, UiTier.Expert);
        AllOf("LaunchExpertBools", GuidanceSettingSpecs.LaunchExpertBools, UiTier.Expert);
        AllOf("LaunchPlayer", GuidanceSettingSpecs.LaunchPlayer, UiTier.Player);
        AllOf("LaunchPlayerBools", GuidanceSettingSpecs.LaunchPlayerBools, UiTier.Player);
        AllOf("LandBoostbackExpert", GuidanceSettingSpecs.LandBoostbackExpert, UiTier.Expert);
        AllOf("LandHandOvers", GuidanceSettingSpecs.LandHandOvers, UiTier.Player);
        AllOf("LandHandOverBools", GuidanceSettingSpecs.LandHandOverBools, UiTier.Player);
        AllOf("LandGfoldPlayer", GuidanceSettingSpecs.LandGfoldPlayer, UiTier.Player);
        AllOf("LandFromOrbitExpert", GuidanceSettingSpecs.LandFromOrbitExpert, UiTier.Expert);
        AllOf("LandFromOrbitExpertBools", GuidanceSettingSpecs.LandFromOrbitExpertBools, UiTier.Expert);
        AllOf("LandGfoldExpert", GuidanceSettingSpecs.LandGfoldExpert, UiTier.Expert);
        AllOf("LandEntryExpert", GuidanceSettingSpecs.LandEntryExpert, UiTier.Expert);
        AllOf("LandEntryExpertBools", GuidanceSettingSpecs.LandEntryExpertBools, UiTier.Expert);
        AllOf("LandTouchdownExpert", GuidanceSettingSpecs.LandTouchdownExpert, UiTier.Expert);
        AllOf("HoverExpert", GuidanceSettingSpecs.HoverExpert, UiTier.Expert);
        AllOf("TuningAscent", GuidanceSettingSpecs.TuningAscent, UiTier.Tuning);
        AllOf("TuningBoostback", GuidanceSettingSpecs.TuningBoostback, UiTier.Tuning);
        AllOf("TuningApproach", GuidanceSettingSpecs.TuningApproach, UiTier.Tuning);
        AllOf("TuningThreeDof", GuidanceSettingSpecs.TuningThreeDof, UiTier.Tuning);
        AllOf("TuningGfold", GuidanceSettingSpecs.TuningGfold, UiTier.Tuning);
        AllOf("TuningSixDof", GuidanceSettingSpecs.TuningSixDof, UiTier.Tuning);
        AllOf("TuningSixDofBools", GuidanceSettingSpecs.TuningSixDofBools, UiTier.Tuning);
        AllOf("AllTuningBools", GuidanceSettingSpecs.AllTuningBools, UiTier.Tuning);
        AllOf("TuningHover", GuidanceSettingSpecs.TuningHover, UiTier.Tuning);
        AllOf("AllTuning", GuidanceSettingSpecs.AllTuning, UiTier.Tuning);
        Check("G-FOLD tuning has twelve values", GuidanceSettingSpecs.TuningGfold.Length == 12);
        Check("LAN is in no count", Array.IndexOf(GuidanceSettingSpecs.LaunchExpert, GuidanceSettingSpecs.LanDeg) < 0);
        Check("LAN changes what the craft does, so it is on the page", GuidanceSettingSpecs.LanDeg.Tier == UiTier.Player);
        Check("the hover profile shape is a gain, so it is on TUNE", GuidanceSettingSpecs.TermQuadK.Tier == UiTier.Tuning
            && Array.IndexOf(GuidanceSettingSpecs.TuningHover, GuidanceSettingSpecs.TermQuadK) >= 0
            && Array.IndexOf(GuidanceSettingSpecs.HoverExpert, GuidanceSettingSpecs.TermQuadK) < 0);

        // One value of both landing plans: it is in both landing sets and counted once on the TUNE dot.
        SettingSpec margin = GuidanceSettingSpecs.LandingPlanFloorMargin;
        Check("floor margin is in the entry set", Array.IndexOf(GuidanceSettingSpecs.TuningThreeDof, margin) >= 0);
        Check("floor margin is in the touchdown set", Array.IndexOf(GuidanceSettingSpecs.TuningSixDof, margin) >= 0);
        var tuningKeys = new HashSet<string>();
        foreach (SettingSpec s in GuidanceSettingSpecs.AllTuning)
            Check("tuning value counted once " + s.Key, tuningKeys.Add(s.Key));
        Check("max burn accel is in the entry and touchdown folds",
            Array.IndexOf(GuidanceSettingSpecs.LandEntryExpert, GuidanceSettingSpecs.LandingAccelLimitG) >= 0
            && Array.IndexOf(GuidanceSettingSpecs.LandTouchdownExpert, GuidanceSettingSpecs.LandingAccelLimitG) >= 0);
        Check("the glide take-over time is a LAND hand-over value",
            Array.IndexOf(GuidanceSettingSpecs.LandHandOvers, GuidanceSettingSpecs.ThreeDofEngageTgoS) >= 0);

        SettingSpec[] promoted =
        {
            GuidanceSettingSpecs.ThreeDofEngageTgoS, GuidanceSettingSpecs.ThreeDofSixDofTimeS, GuidanceSettingSpecs.GfoldHoverHandoffAltM,
            GuidanceSettingSpecs.ConvexQMaxKpa,
        };
        BoolSpec[] promotedBools =
        {
            GuidanceSettingSpecs.ThreeDofSixDofHandover, GuidanceSettingSpecs.FlyConvexAscent, GuidanceSettingSpecs.LaunchDescending,
        };
        SettingSpec[][] expertArrays =
        {
            GuidanceSettingSpecs.LaunchExpert, GuidanceSettingSpecs.LandBoostbackExpert, GuidanceSettingSpecs.LandFromOrbitExpert,
            GuidanceSettingSpecs.LandEntryExpert, GuidanceSettingSpecs.LandTouchdownExpert, GuidanceSettingSpecs.LandGfoldExpert,
            GuidanceSettingSpecs.HoverExpert,
        };
        BoolSpec[][] expertBoolArrays =
        {
            GuidanceSettingSpecs.LaunchExpertBools, GuidanceSettingSpecs.LandFromOrbitExpertBools, GuidanceSettingSpecs.LandEntryExpertBools,
        };
        foreach (SettingSpec s in promoted)
        {
            Check("promoted value is Player " + s.Key, s.Tier == UiTier.Player);
            foreach (SettingSpec[] expert in expertArrays)
                Check("promoted value is in the count of no fold " + s.Key, Array.IndexOf(expert, s) < 0);
        }
        foreach (BoolSpec b in promotedBools)
        {
            Check("promoted switch is Player " + b.Key, b.Tier == UiTier.Player);
            foreach (BoolSpec[] expert in expertBoolArrays)
                Check("promoted switch is in the count of no fold " + b.Key, Array.IndexOf(expert, b) < 0);
        }
        Check("SHOW PLAN IN MAP is a switch of the fold", GuidanceSettingSpecs.ShowConvexPlan.Tier == UiTier.Expert
            && GuidanceSettingSpecs.ShowConvexPlan.Label == "PLAN ON MAP"
            && Array.IndexOf(GuidanceSettingSpecs.LaunchExpertBools, GuidanceSettingSpecs.ShowConvexPlan) >= 0);
        Check("G-FOLD has no fold value of its own", GuidanceSettingSpecs.LandGfoldExpert.Length == 0);
        Check("take over boostback glides is a player switch", GuidanceSettingSpecs.ThreeDofFromGlide.Tier == UiTier.Player
            && GuidanceSettingSpecs.ThreeDofFromGlide.Default);
        Check("vehicle height is measured by default", GuidanceSettingSpecs.VehicleHeightAuto.Default
            && GuidanceSettingSpecs.VehicleHeightAuto.Tier == UiTier.Player);
        Check("glide Kp default follows the field", GuidanceSettingSpecs.GlideKp.Default == 15.0);
        foreach (SettingSpec s in GuidanceSettingSpecs.AllDoubles)
            Check("removed touchdown key is gone " + s.Key, s.Key is not ("SixDofTargetAltM" or "SixDofHoverHandoffAltM"));
        foreach (BoolSpec b in GuidanceSettingSpecs.AllBools)
            Check("removed touchdown switch is gone " + b.Key, b.Key != "SixDofHoverHandoff");
    }

    private static void FormStepRules()
    {
        Check("one ascent method still shows its row", FormRules.AscentRowShows(1));
        Check("two ascent methods show the row", FormRules.AscentRowShows(2));
        Check("no method that can fly is no row", !FormRules.AscentRowShows(0));

        Check("a site 1425 km away from a fall of 67 s is out of reach", FormRules.SiteOutOfReach(1425e3, 67.0, 167.3));
        Check("a site under the craft is not", !FormRules.SiteOutOfReach(98.0, 67.0, 167.3));
        Check("a site the craft flies over is not", !FormRules.SiteOutOfReach(300e3, 200.0, 1600.0));
        Check("with no impact the rule says nothing", !FormRules.SiteOutOfReach(1425e3, double.NaN, 167.3));
        Check("the far site says what to do", PageText.SiteFar == "SITE TOO FAR FOR FALL. PICK SITE BELOW.");

        Check("no notice on the ground", !FormRules.FlightNoticeShows(CraftSituation.Ground, LandWay.FromHere));
        Check("in orbit only the way that leaves it", FormRules.FlightNoticeShows(CraftSituation.Orbit, LandWay.FromOrbit) && !FormRules.FlightNoticeShows(CraftSituation.Orbit, LandWay.FromHere) && !FormRules.FlightNoticeShows(CraftSituation.Orbit, LandWay.Boostback));
        Check("a fall has its notices", FormRules.FlightNoticeShows(CraftSituation.FallingNoAir, LandWay.FromHere));
        Check("no legs row on the pad", !FormRules.LegsNoteShows(CraftSituation.Ground, 0.0, double.NaN, false));
        Check("no legs row in an orbit", !FormRules.LegsNoteShows(CraftSituation.Orbit, 200e3, double.NaN, false));
        Check("a legs row near the ground", FormRules.LegsNoteShows(CraftSituation.FallingNoAir, 8000.0, 57.0, false));
        Check("a legs row close to the impact", FormRules.LegsNoteShows(CraftSituation.FallingAir, 50e3, 200.0, false));
        Check("no legs row high up with time left", !FormRules.LegsNoteShows(CraftSituation.FallingAir, 90e3, 900.0, false));
        Check("the legs row names the stock control and leaves the time to the head", PageText.LegsStowed == "LEGS STOWED. DEPLOY IN EACH LEG WINDOW.");

        Check("the stored default on another body is no pick", LandSite.NonePicked(GuidanceSettingSpecs.SiteLatDeg.Default, GuidanceSettingSpecs.SiteLonDeg.Default, "Earth"));
        Check("the stored default on its own body is a place", !LandSite.NonePicked(GuidanceSettingSpecs.SiteLatDeg.Default, GuidanceSettingSpecs.SiteLonDeg.Default, LandSite.DefaultBody));
        Check("any other value is a pick", !LandSite.NonePicked(28.6, -80.6, "Earth"));
        Check("the answer names its verb and has no word of a press", PageText.NoSiteAnswer("DEORBIT + LAND", 0.67408, 23.47297) == "NO SITE. DEORBIT + LAND TO 0.674, 23.473.");

        var noSite = new CommitGates(false, false, "", "", false, Where: CraftSituation.Orbit, HasAir: true, GfoldChosen: false, NoSiteNote: "No site");
        VerbFit fit = SituationVerbs.For(GuidanceModeId.DeorbitLand, in noSite);
        Check("no site is a plain reason on a way that flies to one", fit.Reason == "No site" && fit.Calm && !fit.Fits);
        Check("the hover reads no site", SituationVerbs.For(GuidanceModeId.Hover, noSite with { Where = CraftSituation.FallingAir }).Fits);
        Check("a press from the wrong place says that first", SituationVerbs.For(GuidanceModeId.DeorbitLand, noSite with { Where = CraftSituation.Ground }).Reason == SituationVerbs.OnGround);
        var far = new CommitGates(false, false, "", "", false, Where: CraftSituation.FallingNoAir, FarSiteNote: "Far");
        Check("a far site dims G-FOLD and it is no normal state", SituationVerbs.For(GuidanceModeId.GfoldHere, in far) is { Reason: "Far", Calm: false });
        Check("and TOUCHDOWN", SituationVerbs.For(GuidanceModeId.SixDofHere, in far).Reason == "Far");
        Check("a far site does not touch a deorbit", SituationVerbs.For(GuidanceModeId.DeorbitLand, in far).Fits);

        var airless = new CommitGates(false, false, "", "", false, Where: CraftSituation.FallingNoAir, BelowBrakingStart: true);
        fit = SituationVerbs.For(GuidanceModeId.Boostback, in airless);
        Check("a boostback with no air ends in an impact", fit.Reason == SituationVerbs.BoostbackNoAir && fit.NextPage == ConsolePage.Land && fit.NextWay == LandWay.FromHere && fit.Calm);
        Check("above the braking start the next way is DEORBIT", SituationVerbs.For(GuidanceModeId.Boostback, airless with { BelowBrakingStart = false }).NextWay == LandWay.FromOrbit);
        Check("with air it stays lit", SituationVerbs.For(GuidanceModeId.Boostback, airless with { Where = CraftSituation.FallingAir, HasAir = true }).Fits);

        Check("a burn planned in the climb is an outline", SituationVerbs.ForManeuver(CraftSituation.CoastingUp, ascentFlies: true) is { Reason: SituationVerbs.AscentFlies, Calm: true });
        Check("and under another mode", SituationVerbs.ForManeuver(CraftSituation.Orbit, otherModeFlies: true) is { Reason: SituationVerbs.GuidanceFlies, Calm: true });
        Check("and on TRANSFER", SituationVerbs.ForTransfer(CraftSituation.Orbit, ascentFlies: true).Reason == SituationVerbs.AscentFlies);
        Check("the ground goes first", SituationVerbs.ForManeuver(CraftSituation.Ground, ascentFlies: true).Reason == SituationVerbs.OnGround);

        var calm = new CommitSpec("X", FlownBy.Afc, Caution: "A state", Calm: true);
        Check("a normal state keeps the normal colour and stays an outline", !calm.AnswerInWarnInk && !calm.Lit && calm.AnswerStatus == "A state");
        Check("a fact to fix takes the warning colour", new CommitSpec("X", FlownBy.Afc, Caution: "A fact").AnswerInWarnInk);

        Check("a mass under a tonne is in kg", Fmt.Mass(463.4) == "463 kg");
        Check("a mass above is in tons", Fmt.Mass(618852.0) == "618.9 t");
        foreach ((string text, double seconds) in new[] { ("14m 24s", 864.0), ("1h 28m", 5280.0), ("267d 6h", 23090400.0), ("45 s", 45.0), ("90", 90.0), ("2m", 120.0) })
            Check("a time reads " + text, Fmt.TryParseTime(text, out double read) && Math.Abs(read - seconds) < 1e-6);
        foreach (string text in new[] { "", "soon", "1x", "-3m" })
            Check("no time reads " + text, !Fmt.TryParseTime(text, out _));
        foreach (double seconds in new[] { 45.0, 864.0, 5280.0, 23090400.0 })
            Check("the form of a time reads back " + seconds, Fmt.TryParseTime(Fmt.Time(seconds), out double back) && Math.Abs(back - seconds) < 1.0);

        var stages = new[] { new StageFuel(1, 10000.0, 3000.0, 3000.0), new StageFuel(2, 5000.0, 2000.0, 3500.0) };
        double whole = BurnBudget.OnBoard(stages);
        StageFuel[] left = BurnBudget.After(stages, new[] { 500.0 });
        Check("what a burn leaves", Math.Abs(BurnBudget.OnBoard(left) - (whole - 500.0)) < 1e-6);
        left = BurnBudget.After(stages, new[] { 500.0, 300.0 });
        Check("what two burns leave", Math.Abs(BurnBudget.OnBoard(left) - (whole - 800.0)) < 1e-6);
        left = BurnBudget.After(stages, new[] { whole * 2.0 });
        Check("burns over the budget leave nothing", left.Length == 0 && BurnBudget.OnBoard(left) == 0.0);
        left = BurnBudget.After(stages, new[] { BurnBudget.OnBoard(new[] { stages[0] }) + 100.0 });
        Check("an empty stage goes", left.Length == 1 && left[0].Number == 2);
        Check("no burn leaves all", BurnBudget.After(stages, Array.Empty<double>()).Length == 2);

        Check("DEORBIT has only TOUCHDOWN", LandSolvers.RowsOf(LandWay.FromOrbit).SequenceEqual(new[] { LandSolver.Touchdown }));
        Check("LAND NOW has only TOUCHDOWN", LandSolvers.RowsOf(LandWay.FromHere).SequenceEqual(new[] { LandSolver.Touchdown }));
        Check("ENTRY has a row on LAND NOW only where it flies first", LandSolvers.RowsOf(LandWay.FromHere, false, true).SequenceEqual(new[] { LandSolver.Entry, LandSolver.Touchdown })
            && LandSolvers.RowsOf(LandWay.FromOrbit, false, true).SequenceEqual(new[] { LandSolver.Touchdown }));
        Check("G-FOLD has a row only while its fallback flies", LandSolvers.RowsOf(LandWay.FromOrbit, true).SequenceEqual(new[] { LandSolver.Touchdown, LandSolver.Gfold })
            && LandSolvers.RowsOf(LandWay.FromHere, true).SequenceEqual(new[] { LandSolver.Touchdown, LandSolver.Gfold }));
        Check("entry flies first for a long fall through air with the engine off", LandSolvers.EntryApplies(true, CraftSituation.FallingAir, false, 120.0) && LandSolvers.EntryApplies(true, CraftSituation.CoastingUp, false, 300.0));
        Check("no air, a lit engine or a short fall go to touchdown directly", !LandSolvers.EntryApplies(false, CraftSituation.FallingNoAir, false, 120.0)
            && !LandSolvers.EntryApplies(true, CraftSituation.FallingAir, true, 120.0) && !LandSolvers.EntryApplies(true, CraftSituation.FallingAir, false, 20.0)
            && !LandSolvers.EntryApplies(true, CraftSituation.Orbit, false, double.PositiveInfinity) && !LandSolvers.EntryApplies(true, CraftSituation.FallingAir, false, double.NaN));
        Check("touchdown is held while entry flies first", LandSolvers.Held(LandWay.FromHere, false, false, true, true)[(int)LandSolver.Touchdown] == LandSolvers.EntryFirst
            && LandSolvers.Held(LandWay.FromOrbit, false, false, true, true)[(int)LandSolver.Touchdown].Length == 0);
        Check("the refusal of a landing names both reasons", PageText.LandingRefused("no gimballed engine.", "G-FOLD needs a gimballed engine.") == "NO LANDING GUIDANCE CAN FLY THIS CRAFT. TOUCHDOWN: no gimballed engine. G-FOLD: G-FOLD needs a gimballed engine.");
        Check("the page shows G-FOLD only while it flies", LandSolvers.Shown(LandSolver.Gfold, false) == LandSolver.Touchdown && LandSolvers.Shown(LandSolver.Gfold, true) == LandSolver.Gfold
            && LandSolvers.Shown(LandSolver.Entry, false) == LandSolver.Entry);
        Check("a row dims with no plan on this body", LandSolvers.RowIsDim(LandSolver.Entry, false, false) && LandSolvers.RowIsDim(LandSolver.Gfold, true, false));
        Check("a row dims while held", LandSolvers.RowIsDim(LandSolver.Touchdown, true, true));
        Check("a row that can fly stays bright", !LandSolvers.RowIsDim(LandSolver.Touchdown, true, false) && !LandSolvers.RowIsDim(LandSolver.Entry, true, false) && !LandSolvers.RowIsDim(LandSolver.Gfold, false, false));

        var orbiting = new DescentFacts(true, 5.73e6, 5.73e6, 0.25e6, 1.4, 0.0, 0.1, 0.1, 2.27, 2.27, 0.0, 15.0, 188.9, 195900.0, SinkMs: 0.0, HeightM: 214900.0);
        Check("a craft in an orbit does not sink, whatever the frame of the site says", orbiting.Descending && !orbiting.Sinking);
        var sinking = orbiting with { SinkMs = 50.0, HeightM = 300.0, ThrustN = 2.0, WeightN = 1.0, GravityMs2 = 9.81 };
        Check("a craft that sinks has its rows", sinking.Sinking);
        Check("the stop reads the sink rate", Math.Abs(sinking.SinkStopM - 2500.0 / (2.0 * 9.81)) < 1e-6);

        Check("the map holds three states", FormRules.MapChoice(false, true) == 0 && FormRules.MapChoice(true, false) == 1 && FormRules.MapChoice(true, true) == 2);
        Check("off keeps the plan switch", FormRules.ApplyMapChoice(0, false) == (false, false) && FormRules.ApplyMapChoice(0, true) == (false, true));
        Check("orbit and plan set both", FormRules.ApplyMapChoice(1, true) == (true, false) && FormRules.ApplyMapChoice(2, false) == (true, true));

        Check("the calculation in plain words", PageText.PlanProgressShort("4 stages: searching for a seed", 3.2) == "SOLVING, 4 stages (3 s)");
        Check("a calculation that has no name yet", PageText.PlanProgressShort("", 3.2) == "SOLVING (3 s)");
        Check("the row of passes says what a split saves beside its field", PageText.PassesRowDetail(1) == "SPLIT SAVES UNDER 1 m/s" && PageText.PassesRowDetail(3).Length == 0);
        Check("a saving that nothing calculated is not said", PageText.PassesRowDetail(1, savingKnown: false).Length == 0 && !PageText.PassesRowTip(false).Contains("saves") && PageText.PassesRowTip(true).Contains("saves"));
        Check("the saving knows when it is unknown", double.IsNaN(FormRules.PassesSaving(915.0, 400.0, double.NaN)) && double.IsNaN(FormRules.PassesSaving(915.0, 0.0, 5500.0)) && double.IsFinite(FormRules.PassesSaving(915.0, 24.0, 5500.0)));
        Check("the search range in the time form", PageText.SearchLabel(864.0, 2592.0) == "SEARCH: TOF 14m 24s TO 43m 12s");
        Check("G-FOLD names where the hover starts", PageText.HoverHandOverLabel(10.0, true) == "HANDOVER: HOVER FROM 10 m");
        Check("a flown orbit against its goal", PageText.FlownAgainstGoal(200.0, 421.0, 200.0, 420.0) == "FLOWN 200 x 421 km, GOAL 200 x 420 km");
        Check("a coast has no orbit yet", PageText.FlownAgainstGoal(-812.0, 192.0, 200.0, 420.0) == "FLOWN TO AP 192 km, GOAL 200 x 420 km");
        var flown = new FlownAscent("Earth", 200.0, 421.0, 200.0, 420.0);
        Check("the line reads the end of the ascent, not the orbit of now", flown.Line(new SituationFacts(CraftSituation.Orbit, "Earth", 300.0, 900.0)) == "FLOWN 200 x 421 km, GOAL 200 x 420 km");
        Check("no ascent, no line", new FlownAscent().Line(new SituationFacts(CraftSituation.Orbit, "Earth", 300.0, 310.0)).Length == 0);
        Check("another body has no line", flown.Line(new SituationFacts(CraftSituation.Orbit, "Luna", 100.0, 110.0)).Length == 0);
        Check("a craft that came back down has no line", flown.Line(new SituationFacts(CraftSituation.FallingNoAir, "Earth", -10.0, 110.0)).Length == 0);
        Check("an orbit that could not be read has no line", new FlownAscent("Earth", double.NaN, 421.0, 200.0, 420.0).Line(new SituationFacts(CraftSituation.Orbit, "Earth")).Length == 0);
        Check("the card of a calculation says what the press does", PageText.CalculatingVerbTip(true).Contains("Arms") && !PageText.CalculatingVerbTip(true).Contains("aunches")
            && PageText.CalculatingVerbTip(false).Contains("Launches") && CommitResolver.LaunchVerbArms("ARM") && CommitResolver.LaunchVerbArms("SOLVE + ARM") && !CommitResolver.LaunchVerbArms("LAUNCH") && !CommitResolver.LaunchVerbArms("SOLVE + LAUNCH"));
        Check("passes with one payer say it once", PageText.PassesPaidBy("Seq 1") == "PAID BY Seq 1");
        Check("a phase with no list stands alone", GuidanceModes.LiveChain(Array.Empty<string>(), "X", s => s.Length, 100f) is [{ Place: StepPlace.Now }]);
    }

    private static void EveryValueHasAHome()
    {
        SettingSpec[][] folds =
        {
            GuidanceSettingSpecs.LaunchExpert, GuidanceSettingSpecs.LandBoostbackExpert, GuidanceSettingSpecs.LandFromOrbitExpert,
            GuidanceSettingSpecs.LandEntryExpert, GuidanceSettingSpecs.LandTouchdownExpert, GuidanceSettingSpecs.HoverExpert,
        };
        BoolSpec[][] foldBools = { GuidanceSettingSpecs.LaunchExpertBools, GuidanceSettingSpecs.LandFromOrbitExpertBools };
        foreach (SettingSpec s in GuidanceSettingSpecs.AllDoubles)
        {
            if (s.Tier == UiTier.Expert)
                Check("an Expert value is in a fold " + s.Key, folds.Any(f => Array.IndexOf(f, s) >= 0));
            else if (s.Tier == UiTier.Tuning)
                Check("a Tuning value is in a set " + s.Key, Array.IndexOf(GuidanceSettingSpecs.AllTuning, s) >= 0);
        }
        foreach (BoolSpec b in GuidanceSettingSpecs.AllBools)
        {
            if (b.Tier == UiTier.Expert)
                Check("an Expert switch is in a fold " + b.Key, foldBools.Any(f => Array.IndexOf(f, b) >= 0));
            else if (b.Tier == UiTier.Tuning)
                Check("a Tuning switch is in a set " + b.Key, Array.IndexOf(GuidanceSettingSpecs.AllTuningBools, b) >= 0);
        }
    }

    private static void FoldLabels()
    {
        static FoldPart On(string name) => new(name, true);
        static FoldPart Off(string name) => new(name, false);
        const string How = PageText.HowItFlies;

        Check("a fold lists the parts that show", FoldRules.Label(How, [On(FoldNames.Plane), On(FoldNames.Insertion), On(FoldNames.Turn)]) == "GNC: PLANE, INSERTION, TURN");
        Check("a part with no rows stays out of the label", FoldRules.Label(How, [On(FoldNames.Turn), Off(FoldNames.InMap)]) == "GNC: TURN"
            && FoldRules.Label(How, [Off(FoldNames.InMap), On(FoldNames.Turn)]) == "GNC: TURN");
        Check("the label follows the state", FoldRules.Label(How, [On(FoldNames.Turn), On(FoldNames.InMap)]) == "GNC: TURN, SHOW IN MAP");
        Check("a switch of the picture has the name of its row in the fold", FoldNames.InMap == PageText.ShowInMap && FoldNames.InWorld == PageText.ShowInWorld);
        Check("the label counts the changed values", FoldRules.Label(How, [On(FoldNames.Turn)], 2) == "GNC: TURN (2 CHANGED)");
        Check("the reset stays out of the label", FoldRules.Label(How, [new("", true), On(FoldNames.Turn)], 1) == "GNC: TURN (1 CHANGED)");
        Check("a fold with no part is its name", FoldRules.Label(How, [Off(FoldNames.Turn)]) == How);

        Check("no part with rows, no fold", !FoldRules.HasRows([Off(FoldNames.Turn), Off(FoldNames.InMap)]) && !FoldRules.HasRows([]));
        Check("the reset alone does not make a fold show", !FoldRules.HasRows([new("", true)]));
        Check("one part with rows, a fold", FoldRules.HasRows([Off(FoldNames.Turn), On(FoldNames.InMap)]));

        var drawn = new List<string>();
        FoldPart[] parts =
        {
            new(FoldNames.Plane, true, () => drawn.Add(FoldNames.Plane)),
            new(FoldNames.Turn, false, () => drawn.Add(FoldNames.Turn)),
            new(FoldNames.ConvexPlan, true, () => drawn.Add(FoldNames.ConvexPlan)),
        };
        foreach (FoldPart part in parts)
            if (part.Shows)
                part.Draw?.Invoke();
        Check("the rows and the label agree", string.Join(", ", drawn).ToLowerInvariant() == FoldRules.Contents(parts));

        Check("a label has a name and a detail", FoldRules.Split("LIMITS: Max Q 140 kPa") == ("LIMITS", "Max Q 140 kPa"));
        Check("the detail starts in capitals", FoldRules.Split("PLAN: site 28.60, -80.60") == ("PLAN", "Site 28.60, -80.60"));
        Check("a plain name has no detail", FoldRules.Split("FLIGHT PLAN") == ("FLIGHT PLAN", ""));
        foreach (string verdict in new[] { "Cannot stop in time: needs TWR 3.10, has 2.00.", "TWR 1.30 at minimum throttle: the craft climbs.", "Over-powered: no descent with a floor above 0.40." })
            Check("a verdict is one sentence " + verdict, FoldRules.Split(verdict) == (verdict, ""));
        foreach (string label in new[]
                 {
                     PageText.LimitsLabel(true, 140.0, false, 4.0), PageText.BoostersLabel(2, 1, 4200.0), PageText.HandOversLabel(true, 45.0, true, 5.0),
                     PageText.HoverHandOverLabel(10.0, true), PageText.HoverHandOverLabel(10.0, false), PageText.SearchLabel(864.0, 2592.0), FlightView.PlanLabel("site 28.60, -80.60"),
                 })
        {
            (string name, string detail) = FoldRules.Split(label);
            Check("a fold of a page has both levels " + label, name == name.ToUpperInvariant() && detail.Length > 0 && !label.Contains("EXPERT"));
        }

        // The small level takes about 8.7 px per character at scale 1.2, and about 421 px are left of the row of 584 px past the column of the detail.
        const int Fits = 48;
        (string Name, FoldPart[] Parts)[] longest =
        {
            ("LAUNCH", [On(FoldNames.Plane), On(FoldNames.Insertion), On(FoldNames.Turn), On(FoldNames.ConvexPlan)]),
            ("DEORBIT", [On(FoldNames.VehicleHeight), On(FoldNames.Approach)]),
            ("BOOSTBACK", [On(FoldNames.VehicleHeight), On(FoldNames.Burn), On(FoldNames.Glide)]),
            ("LAND NOW", [On(FoldNames.VehicleHeight), On(FoldNames.BurnLimit)]),
            ("HOVER", [On(FoldNames.VehicleHeight), On(FoldNames.HoverLimits)]),
            ("MANEUVER", [On(FoldNames.Passes), On(FoldNames.Split), On(FoldNames.CraftTarget)]),
            ("BURNS", [On(FoldNames.Allocator)]),
            ("STAGING", [On(FoldNames.ThisCraft), On(FoldNames.NewCraft)]),
        };
        foreach ((string name, FoldPart[] list) in longest)
        {
            string detail = FoldRules.Split(FoldRules.Label(How, list, 9)).Detail;
            Check("the list of a fold fits its row " + name, detail.Length <= Fits, detail);
        }
        Check("a switch of the picture fits too, with no count", FoldRules.Split(FoldRules.Label(How, [On(FoldNames.VehicleHeight), On(FoldNames.Burn), On(FoldNames.Glide), On(FoldNames.InWorld)])).Detail.Length <= Fits
            && FoldRules.Split(FoldRules.Label(How, [On(FoldNames.VehicleHeight), On(FoldNames.BurnLimit), On(FoldNames.InWorld)])).Detail.Length <= Fits);

        Check("PLAN FOR needs another craft", !FormRules.PlanForShows(1, false) && FormRules.PlanForShows(2, false));
        Check("a plan for another craft keeps PLAN FOR", FormRules.PlanForShows(1, true));
        Check("the plan says the limit it flies to", PageText.QRelaxedNote(140.0, 162.5) == "MAX Q 140 kPa OUT OF REACH. PLAN USES 162.5 kPa.");
        Check("the reset asks in one sentence", PageText.ResetFoldQuestion(1).Contains("value of this fold") && PageText.ResetFoldQuestion(3).Contains("3 changed values"));
        Check("the card of a verdict lists the fold", PageText.FoldHolds(FoldRules.Contents([On(FoldNames.Distance), On(FoldNames.Terrain)])) == "HOLDS: distance, terrain.");
    }

    private static void AllOf(string name, SettingSpec[] specs, UiTier tier)
    {
        foreach (SettingSpec s in specs)
            Check(name + " holds only " + tier + " (" + s.Key + ")", s.Tier == tier);
    }

    private static void AllOf(string name, BoolSpec[] specs, UiTier tier)
    {
        foreach (BoolSpec s in specs)
            Check(name + " holds only " + tier + " (" + s.Key + ")", s.Tier == tier);
    }


    private static void PageKeys()
    {
        Check("seven pages", ConsolePages.All.Length == 7 && Enum.GetValues<ConsolePage>().Length == 7);
        Check("one tab row in the order of a mission", ConsolePages.All.SequenceEqual(new[]
        {
            ConsolePage.Launch, ConsolePage.Orbit, ConsolePage.Transfer, ConsolePage.Rcs, ConsolePage.Land, ConsolePage.Staging, ConsolePage.Tune,
        }));
        Check("the setup pages start at STAGING", ConsolePages.FirstSetupPage == ConsolePage.Staging);
        // The interface font at scale 1 has an advance of about 9.1 px, and the tab row of the narrowest console has 427 px.
        float tabText = ConsolePages.All.Sum(p => ConsolePages.Label(p).Length) * 9.1f;
        Check("the seven labels fit at 520 px", DockLayout.TabPadding(487f, tabText, ConsolePages.All.Length, 13f, 1f) * 14f + tabText + 13f <= 487f + 0.01f
            && DockLayout.TabPadding(487f, tabText, ConsolePages.All.Length, 13f, 1f) >= 5f);
        Check("the seven labels fit at the narrow limit", DockLayout.TabPadding(427f, tabText, ConsolePages.All.Length, 13f, 1f) * 14f + tabText + 13f <= 427f + 0.01f);
        Check("a wide console does not spread the tabs", Math.Abs(DockLayout.TabPadding(887f, tabText, ConsolePages.All.Length, 13f, 1f) - DockLayout.MaxTabPadding) < 0.01f);
        foreach (ConsolePage page in Enum.GetValues<ConsolePage>())
        {
            Check("page in the tab row once " + page, ConsolePages.All.Count(p => p == page) == 1);
            Check("page key round trip " + page, ConsolePages.PageFromKey(ConsolePages.Key(page), ConsolePage.Tune) == page);
            Check("page label is upper case " + page, ConsolePages.Label(page) == ConsolePages.Label(page).ToUpperInvariant());
        }
        Check("MANEUVER keeps the key ORBIT", ConsolePages.Key(ConsolePage.Orbit) == "ORBIT" && ConsolePages.Label(ConsolePage.Orbit) == "MANEUVER");
        Check("BURNS keeps the key RCS", ConsolePages.Key(ConsolePage.Rcs) == "RCS" && ConsolePages.Label(ConsolePage.Rcs) == "BURNS");
        Check("situation on the ground", SituationText.Line(new SituationFacts(CraftSituation.Ground, "Earth")) == "LANDED, Earth. MANUAL");
        Check("situation in orbit", SituationText.Line(new SituationFacts(CraftSituation.Orbit, "Earth", 200.0, 423.0)) == "ORBIT, Earth, 200 x 423 km. MANUAL");
        Check("an escape path names no orbit", SituationText.Line(new SituationFacts(CraftSituation.Escaping, "Luna", 10.0, 20.0)) == "ESCAPE, Luna. MANUAL");
        Check("coasting up names the apoapsis", SituationText.Line(new SituationFacts(CraftSituation.CoastingUp, "Earth", -812.0, 180.0)) == "COAST, Earth, AP 180 km. MANUAL");
        Check("under power the craft climbs", SituationText.Line(new SituationFacts(CraftSituation.CoastingUp, "Earth", -812.0, 180.0, Thrust: true)) == "ASCENT, Earth, AP 180 km. MANUAL");
        Check("falling with air names the body", SituationText.Line(new SituationFacts(CraftSituation.FallingAir, "Earth", -812.0, 180.0)) == "FALL, Earth. MANUAL");
        Check("falling with no air says so", SituationText.Line(new SituationFacts(CraftSituation.FallingNoAir, "Luna")) == "FALL, Luna, NO AIR. MANUAL");
        Check("under power the craft descends", SituationText.Line(new SituationFacts(CraftSituation.FallingNoAir, "Luna", Thrust: true)) == "DESCENT, Luna, NO AIR. MANUAL");
        Check("situation not known says who flies", SituationText.Line(default) == "MANUAL");
        Check("the place stands without who flies", SituationText.Place(new SituationFacts(CraftSituation.Ground, "Luna")) == "LANDED, Luna" && SituationText.Place(default).Length == 0);
        Check("contact is the ground, whatever the path", SituationText.Classify(true, true, 7000e3, 6500e3, 10.0, true) == CraftSituation.Ground);
        Check("a closed path over the near surface is an orbit", SituationText.Classify(false, true, 6600e3, 6500e3, -10.0, true) == CraftSituation.Orbit);
        Check("a closed low path that rises coasts up", SituationText.Classify(false, true, 6450e3, 6500e3, 10.0, true) == CraftSituation.CoastingUp);
        Check("a closed low path that sinks falls with air", SituationText.Classify(false, true, 6450e3, 6500e3, -10.0, true) == CraftSituation.FallingAir);
        Check("a closed low path that sinks falls with no air", SituationText.Classify(false, true, 1700e3, 1750e3, -10.0, false) == CraftSituation.FallingNoAir);
        Check("an open path that clears the surface is an escape, also on the way in", SituationText.Classify(false, false, 7000e3, 6500e3, -10.0, true) == CraftSituation.Escaping);
        Check("an open path that rises is an escape, also with a low periapsis", SituationText.Classify(false, false, 6000e3, 6500e3, 10.0, true) == CraftSituation.Escaping);
        Check("an open path into the ground falls", SituationText.Classify(false, false, 6000e3, 6500e3, -10.0, true) == CraftSituation.FallingAir);
        Check("a value that is not known gives no answer", SituationText.Classify(false, true, double.NaN, 6500e3, 1.0, true) == CraftSituation.Unknown
            && SituationText.Classify(false, true, 7000e3, 6500e3, double.NaN, true) == CraftSituation.Unknown);
        Check("only the two falling situations fall", SituationText.IsFalling(CraftSituation.FallingAir) && SituationText.IsFalling(CraftSituation.FallingNoAir)
            && !SituationText.IsFalling(CraftSituation.CoastingUp) && !SituationText.IsFalling(CraftSituation.Unknown));
        Check("the drawer control counts only above zero", PageText.MoreLabel(0, false) == "MORE v" && PageText.MoreLabel(2, true) == "MORE (2) ^");
        Check("the card of the control names what it counts", PageText.MoreCard(1, 0).StartsWith("1 new message. ") && PageText.MoreCard(2, 1).StartsWith("2 new messages. "));
        Check("the card has no count at zero", PageText.MoreCard(0, 0) == "GNC switches, active modes, other vehicles, messages.");
        Check("the sheet stays inside its limit", DockLayout.DrawerMaxHeight == 360f);
        Check("no warning while both switches are on", PageText.MayOffLine(false, false).Length == 0);
        Check("a switch says nothing while it does not act at once", PageText.SteerSwitchNote(false, false).Length == 0 && PageText.EnginesSwitchNote(false, false).Length == 0);
        Check("steer off names the hand-back", PageText.SteerSwitchNote(false, true).Contains("ends the mode") && PageText.SteerSwitchNote(false, true).Contains("cuts the engine"));
        Check("engines off keeps the steering", PageText.EnginesSwitchNote(false, true).Contains("keeps steering"));
        Check("touchdown guidance reads neither switch", PageText.SteerSwitchNote(true, true) == PageText.EnginesSwitchNote(true, true));
        Check("a switch that is off is a warning that names it", PageText.MayOffLine(true, false) == "AFC MAY STEER OFF >"
            && PageText.MayOffLine(false, true) == "AFC MAY USE ENGINES OFF >" && PageText.MayOffLine(true, true) == "AFC MAY STEER + USE ENGINES OFF >");
        Check("the old booster key reads as LAND", ConsolePages.PageFromKey("RECOVER", ConsolePage.Tune) == ConsolePage.Land);
        Check("an old booster page after separation is LAND > BOOSTBACK",
            ConsolePages.ResolveLegacy("RECOVER", "AFTERSEPARATION", out ConsolePage afterPage, out LandWay? afterWay)
            && afterPage == ConsolePage.Land && afterWay == LandWay.Boostback);
        Check("an old booster page before launch is LAUNCH",
            ConsolePages.ResolveLegacy("recover", "BEFORELAUNCH", out ConsolePage beforePage, out LandWay? beforeWay)
            && beforePage == ConsolePage.Launch && beforeWay == null);
        Check("an old booster cue without a step is LAND > BOOSTBACK",
            ConsolePages.ResolveLegacy("RECOVER", "", out ConsolePage barePage, out LandWay? bareWay)
            && barePage == ConsolePage.Land && bareWay == LandWay.Boostback);
        Check("a current key is no old booster key", !ConsolePages.ResolveLegacy("LAND", "BOOSTBACK", out _, out _));
        Check("land ways in display order", LandWays.DisplayOrder.Length == 4 && LandWays.DisplayOrder.Distinct().Count() == 4
            && LandWays.DisplayOrder[0] == LandWay.FromOrbit && LandWays.DisplayOrder[1] == LandWay.Boostback
            && LandWays.DisplayOrder[2] == LandWay.FromHere && LandWays.DisplayOrder[3] == LandWay.Hover);
        Check("every land way is in the row", Enum.GetValues<LandWay>().All(w => LandWays.DisplayIndex(w) >= 0));
        Check("land way labels", LandWays.Labels.SequenceEqual(new[] { "DEORBIT", "BOOSTBACK", "LAND NOW", "HOVER" }));
        Check("the old land way keys stay", ConsolePages.EnumKey(LandWay.FromOrbit) == "FROMORBIT"
            && ConsolePages.EnumKey(LandWay.FromHere) == "FROMHERE" && ConsolePages.EnumKey(LandWay.Hover) == "HOVER");
        Check("boostback way key round trip",
            ConsolePages.EnumFromKey(ConsolePages.EnumKey(LandWay.Boostback), LandWay.FromOrbit) == LandWay.Boostback);
        Check("the boostback tuning set keeps its key", ConsolePages.EnumKey(TuneSet.Recover) == "RECOVER"
            && ConsolePages.TuneSetLabel(TuneSet.Recover) == "BOOSTBACK");
        Check("a boostback flies, LAND follows to BOOSTBACK", LandWays.Follow(LiveLanding.None, LiveLanding.Boostback) == LandWay.Boostback);
        Check("entry takes the glide, LAND follows to LAND NOW", LandWays.Follow(LiveLanding.Boostback, LiveLanding.Entry) == LandWay.FromHere);
        Check("entry from the page leaves the way", LandWays.Follow(LiveLanding.None, LiveLanding.Entry) == null);
        Check("no change leaves the way", LandWays.Follow(LiveLanding.Boostback, LiveLanding.Boostback) == null);
        Check("the braking burn hands to G-FOLD, LAND follows to LAND NOW", LandWays.Follow(LiveLanding.None, LiveLanding.Gfold) == LandWay.FromHere);
        Check("the braking burn hands to touchdown, LAND follows to LAND NOW", LandWays.Follow(LiveLanding.None, LiveLanding.Touchdown) == LandWay.FromHere);
        Check("G-FOLD hands to the hover, LAND follows to HOVER", LandWays.Follow(LiveLanding.Gfold, LiveLanding.Hover) == LandWay.Hover);
        Check("an end leaves the way", LandWays.Follow(LiveLanding.Hover, LiveLanding.None) == null);
        Check("only DEORBIT and LAND NOW pick a solver", LandWays.HasSolver(LandWay.FromOrbit) && LandWays.HasSolver(LandWay.FromHere)
            && !LandWays.HasSolver(LandWay.Boostback) && !LandWays.HasSolver(LandWay.Hover));
        foreach (ConsolePage page in ConsolePages.All)
        {
            Check("chooser card is not empty " + page, ConsolePages.HoverCard(page).Length > 0);
        }
        Check("head chip owners", PageText.HeadOwner(FlownBy.Afc) == Owner.Afc && PageText.HeadOwner(FlownBy.StockAuto) == Owner.Stock
            && PageText.HeadOwner(FlownBy.AfcPlansStockAuto) == Owner.AfcPlansStockFlies && PageText.HeadOwner(FlownBy.You) == Owner.Player);
        Check("stock line, nothing planned", PageText.StockLine(false, "", 0.0) == "NO BURN PLANNED");
        Check("stock line, next node", PageText.StockLine(true, "4m 12s", 120.4) == "NEXT BURN IN 4m 12s, 120 m/s",
            PageText.StockLine(true, "4m 12s", 120.4));
        foreach (string tag in new[] { PageText.TagEveryCraft, PageText.TagSaveDefaults, PageText.TagCraftDesign, PageText.TagBurn,
                     PageText.KeptRule, PageText.TagNotKept, PageText.TagGameSession })
            Check("persistence tag is a sentence in ASCII " + tag, tag.Length > 0 && tag != tag.ToUpperInvariant() && tag.EndsWith('.') && tag.All(ch => ch < 128));
        Check("unknown key falls back", ConsolePages.PageFromKey("NOPE", ConsolePage.Launch) == ConsolePage.Launch);
        Check("old STATUS key falls back", ConsolePages.PageFromKey("STATUS", ConsolePage.Orbit) == ConsolePage.Orbit);
        Check("editor pages", ConsolePages.WorksInEditor(ConsolePage.Staging) && ConsolePages.WorksInEditor(ConsolePage.Tune)
            && ConsolePages.WorksInEditor(ConsolePage.Transfer) && !ConsolePages.WorksInEditor(ConsolePage.Launch)
            && !ConsolePages.WorksInEditor(ConsolePage.Orbit) && !ConsolePages.WorksInEditor(ConsolePage.Rcs));
        Check("sub-page round trip",
            ConsolePages.EnumFromKey(ConsolePages.EnumKey(LandWay.Hover), LandWay.FromOrbit) == LandWay.Hover);
        Check("bad sub-page falls back", ConsolePages.EnumFromKey("17", LandWay.FromHere) == LandWay.FromHere);
        Check("tune set round trip", ConsolePages.EnumFromKey(ConsolePages.EnumKey(TuneSet.Diag), TuneSet.Afc) == TuneSet.Diag);
        Check("launch commits UPFG",
            ConsolePages.ModeIdFor(ConsolePage.Launch, LandWay.FromOrbit, LandSolver.Gfold, AscentChoice.CatsUpfg) == GuidanceModeId.AscentUpfg);
        Check("launch commits gravity turn",
            ConsolePages.ModeIdFor(ConsolePage.Launch, LandWay.FromOrbit, LandSolver.Gfold, AscentChoice.GravityTurn) == GuidanceModeId.AscentGravityTurn);
        Check("land boostback commits boostback",
            ConsolePages.ModeIdFor(ConsolePage.Land, LandWay.Boostback, LandSolver.Gfold, AscentChoice.CatsUpfg) == GuidanceModeId.Boostback);
        Check("land hover commits hover",
            ConsolePages.ModeIdFor(ConsolePage.Land, LandWay.Hover, LandSolver.Entry, AscentChoice.CatsUpfg) == GuidanceModeId.Hover);
        Check("running boostback opens LAND", ConsolePages.PageOf(StopTarget.Boostback) == ConsolePage.Land);
        Check("land from here on touchdown guidance",
            ConsolePages.ModeIdFor(ConsolePage.Land, LandWay.FromHere, LandSolver.Touchdown, AscentChoice.CatsUpfg) == GuidanceModeId.SixDofHere);
        Check("land from here on G-FOLD",
            ConsolePages.ModeIdFor(ConsolePage.Land, LandWay.FromHere, LandSolver.Gfold, AscentChoice.CatsUpfg) == GuidanceModeId.GfoldHere);
        Check("land from here on entry guidance",
            ConsolePages.ModeIdFor(ConsolePage.Land, LandWay.FromHere, LandSolver.Entry, AscentChoice.CatsUpfg) == GuidanceModeId.EntryHere);
        Check("deorbit and land ignores the entry choice",
            ConsolePages.ModeIdFor(ConsolePage.Land, LandWay.FromOrbit, LandSolver.Entry, AscentChoice.CatsUpfg) == GuidanceModeId.DeorbitLand);
        Check("tune landing round trip", ConsolePages.EnumFromKey(ConsolePages.EnumKey(TuneLanding.Entry), TuneLanding.Approach) == TuneLanding.Entry);
        Check("orbit commits no guidance",
            ConsolePages.ModeIdFor(ConsolePage.Orbit, LandWay.FromHere, LandSolver.Gfold, AscentChoice.CatsUpfg) == null);
        Check("cue parse",
            ConsolePages.TryParseCue("LAND/HOVER", out ConsolePage cuePage, out string sub) && cuePage == ConsolePage.Land && sub == "HOVER");
        Check("cue parse refuses junk", !ConsolePages.TryParseCue("JUNK", out _, out _));
        Check("running ascent opens LAUNCH", ConsolePages.PageOf(StopTarget.Ascent) == ConsolePage.Launch);
        Check("running hover opens LAND", ConsolePages.PageOf(StopTarget.Hover) == ConsolePage.Land);
        Check("running RCS opens RCS", ConsolePages.PageOf(StopTarget.Rcs) == ConsolePage.Rcs);
        Check("running passes open ORBIT", ConsolePages.PageOf(StopTarget.MultiPass) == ConsolePage.Orbit);
        Check("running transfer passes open TRANSFER", ConsolePages.PageOf(StopTarget.MultiPass, transferPasses: true) == ConsolePage.Transfer);
        Check("stock AUTO opens RCS whatever the passes", ConsolePages.PageOf(StopTarget.StockAuto, transferPasses: true) == ConsolePage.Rcs);
        Check("nothing running opens no page", ConsolePages.PageOf(StopTarget.None) == null);
    }

    private static void OrbitToolKeys()
    {
        string[] keys =
        {
            OrbitToolCatalog.KeySetPeriapsis, OrbitToolCatalog.KeySetApoapsis, OrbitToolCatalog.KeyMatchInclination,
            OrbitToolCatalog.KeySetInclination, OrbitToolCatalog.KeyCircularizeApoapsis, OrbitToolCatalog.KeyCircularizePeriapsis,
        };
        Check("tool keys keep the planner strings",
            OrbitToolCatalog.KeySetApoapsis == "AFC Set Apoapsis" && OrbitToolCatalog.KeyCircularizePeriapsis == "Circularize Periapsis");
        foreach (string key in keys)
        {
            Check("is tool " + key, OrbitToolCatalog.IsTool(key));
            CircularizeAt at = CircularizeAt.Apoapsis;
            OrbitToolCatalog.CircularizeSide(key, ref at);
            Check("TOOL segment round trip " + key, OrbitToolCatalog.KeyForTool(OrbitToolCatalog.ToolIndex(key), at) == key);
        }
        CircularizeAt keep = CircularizeAt.Periapsis;
        OrbitToolCatalog.CircularizeSide("Hohmann", ref keep);
        Check("an unknown key keeps the side", keep == CircularizeAt.Periapsis);
        Check("Hohmann is no tool", !OrbitToolCatalog.IsTool("Hohmann") && OrbitToolCatalog.ToolIndex("Hohmann") == -1);
        Check("five TOOL segments", OrbitToolCatalog.ToolLabels.Length == 5);
        Check("plane tools", OrbitToolCatalog.IsPlaneTool(OrbitToolCatalog.KeyMatchInclination) && OrbitToolCatalog.IsPlaneTool(OrbitToolCatalog.KeySetInclination)
            && !OrbitToolCatalog.IsPlaneTool(OrbitToolCatalog.KeySetApoapsis));
    }

    // Every row of the planner key table: what a write to TransferPlanner.ShowPlanWindow does, and what a routed press does to the console.
    private static void RouteDecisions()
    {
        Check("a close always passes", RouteDecision.Decide(false, true, false, true) == RouteAction.Pass);
        Check("a close passes with routing off", RouteDecision.Decide(false, true, false, false) == RouteAction.Pass);
        Check("the per-frame write of an open window passes", RouteDecision.Decide(true, true, false, true) == RouteAction.Pass);
        Check("the STOCK PLANNER link passes", RouteDecision.Decide(true, false, true, true) == RouteAction.Pass);
        Check("routing off gives the key to stock", RouteDecision.Decide(true, false, false, false) == RouteAction.Pass);
        Check("an open from closed is routed", RouteDecision.Decide(true, false, false, true) == RouteAction.Route);

        Check("frame check routes a stray stock window", RouteDecision.FrameCheckRoutes(stockOpen: true, stockLinkActive: false, active: true));
        Check("frame check leaves the linked stock window", !RouteDecision.FrameCheckRoutes(true, true, true));
        Check("frame check off with routing off", !RouteDecision.FrameCheckRoutes(true, false, false));
        Check("frame check idle while stock is closed", !RouteDecision.FrameCheckRoutes(false, false, true));
        Check("frame check adopts a stock window opened without routing", RouteDecision.FrameCheckAdopts(stockOpen: true, stockLinkActive: false, active: false));
        Check("frame check does not adopt while routing", !RouteDecision.FrameCheckAdopts(true, false, true));
        Check("frame check does not adopt a linked window again", !RouteDecision.FrameCheckAdopts(true, true, false));
        Check("frame check adopts nothing while stock is closed", !RouteDecision.FrameCheckAdopts(false, false, false));

        Check("the key is routed in flight with a craft", RouteDecision.RoutesKey(active: true, editor: false, hasControlledVehicle: true));
        Check("the key is routed in the editor", RouteDecision.RoutesKey(true, true, false));
        Check("without a craft the key goes to stock", !RouteDecision.RoutesKey(true, false, false));
        Check("routing off gives the key to stock with a craft", !RouteDecision.RoutesKey(false, false, true));

        Check("closed console opens", RouteDecision.Toggle(consoleOpen: false, stripShown: false, editor: false) == ConsoleToggle.Open);
        Check("open console closes", RouteDecision.Toggle(true, false, false) == ConsoleToggle.Close);
        Check("open console closes also with a strip", RouteDecision.Toggle(true, true, false) == ConsoleToggle.Close);
        Check("the strip opens the running page", RouteDecision.Toggle(false, true, false) == ConsoleToggle.OpenRunningPage);
        Check("the editor opens the editor state", RouteDecision.Toggle(false, false, true) == ConsoleToggle.OpenEditor);
        Check("the editor closes an open console", RouteDecision.Toggle(true, false, true) == ConsoleToggle.Close);
    }

    private static void DockPlacement()
    {
        // WIDE at interface scale 1.2 on a screen of 3840 px: 1104 px wide, the right pane about 440 and the left about 620 with its lane.
        (float wideMin, float wideMax) = DockLayout.WidthLimits(1.2f, wide: true, 3840f);
        Check("WIDE may shrink to the room of two panes and grow to the screen", Math.Abs(wideMin - (DockLayout.RightPaneWidth + DockLayout.LeastLeftPane + DockLayout.PaneFrameRoom) * 1.2f) < 0.5f && Math.Abs(wideMax - (3840f - 32f)) < 0.5f, wideMin + " " + wideMax);
        PaneSplit panes = DockLayout.SplitPanes(1090.8f, 26.8f, 1.2f);
        Check("the right pane holds about 440 px of inputs", Math.Abs(panes.RightContent - 440f) < 1f, panes.RightContent.ToString());
        Check("the left pane is about 620 px with its lane", Math.Abs(panes.LeftBody - 620f) < 8f, panes.LeftBody.ToString());
        Check("the panes fill the body", Math.Abs(panes.LeftBody + panes.RightBody - 1090.8f) < 0.01f);
        Check("a screen of 1920 px has room for two panes", DockLayout.WideHasPanes(1f, 1920f) && DockLayout.WideHasPanes(1.2f, 1920f));
        Check("a very narrow screen keeps one pane", !DockLayout.WideHasPanes(1.2f, 800f));
        Check("narrow has one body", DockLayout.PanesFor(false, true, false, false) == PaneMode.Full);
        Check("WIDE has two panes", DockLayout.PanesFor(true, true, false, false) == PaneMode.Two);
        Check("a page that flies gives the picture the whole width", DockLayout.PanesFor(true, true, true, false) == PaneMode.Full);
        Check("the form of a page that flies opens beside the picture", DockLayout.PanesFor(true, true, true, true) == PaneMode.Two);
        Check("no room means one body", DockLayout.PanesFor(true, false, false, false) == PaneMode.Full);
        foreach (ConsolePage page in new[] { ConsolePage.Launch, ConsolePage.Orbit, ConsolePage.Transfer, ConsolePage.Land, ConsolePage.Staging })
        {
            Check("WIDE widens " + page, DockLayout.PageTakesWide(page, true, false));
            Check("WIDE does not widen " + page + " with nothing to draw", !DockLayout.PageTakesWide(page, false, false));
        }
        Check("WIDE does not widen TUNE, which has no picture", !DockLayout.PageTakesWide(ConsolePage.Tune, true, false) && !DockLayout.PageTakesWide(ConsolePage.Tune, true, true));
        Check("WIDE does not widen BURNS with no burn", !DockLayout.PageTakesWide(ConsolePage.Rcs, true, false));
        Check("WIDE widens BURNS with its list", DockLayout.PageTakesWide(ConsolePage.Rcs, true, true));
        Check("the WIDE card says why a page stays narrow", PageText.WideCard(true, true, pageStaysNarrow: true).Contains("needs a picture") && PageText.WideCard(false, true, pageStaysNarrow: true).Contains("needs a picture"));
        Check("the WIDE card says that a page with no picture stays narrow", PageText.WideCard(false, true).Contains("no picture stays narrow") && !PageText.WideCard(false, true).Contains("TUNE"));

        // The sizes at scale 1.2, with a label column of 108 px, names of 78 and 95 px, buttons of 56 px and a gap of 8 px.
        float moveRow = DockLayout.MoveRowWidth(108f, 78f, 95f, 56f, 8f);
        Check("MOVE PICK has one row in the narrow window", moveRow <= 584f, moveRow.ToString());
        Check("MOVE PICK has two rows in the right pane, because its second pair would leave the pane", moveRow > 440f);
        Check("two rows of MOVE PICK fit the right pane", 108f + 95f + 8f + 56f * 2f + 8f * 2f <= 440f);

        Check("WIDE from the right edge grows left", DockLayout.WideLeft(1872f, 624f, 1104f, 0f, 1920f) == 768f);
        Check("WIDE from the left edge grows right and keeps its left edge", DockLayout.WideLeft(700f, 624f, 1104f, 0f, 1920f) == 76f);
        Check("WIDE never leaves the screen on the left", DockLayout.WideLeft(1000f, 624f, 1104f, 0f, 1280f) >= 0f);
        Check("WIDE never leaves the screen on the right", DockLayout.WideLeft(900f, 624f, 1104f, 0f, 1280f) + 1104f <= 1280f);
        Check("a move in WIDE keeps the corner of a right edge", DockLayout.NarrowRight(768f, 1104f, 624f, growsRight: false) == 1872f);
        Check("a move in WIDE keeps the corner of a left edge", DockLayout.NarrowRight(76f, 1104f, 624f, growsRight: true) == 700f);
        Check("the side of WIDE does not flip after its own move", DockLayout.WideGrowsRight(76f, 700f, 0f, 1920f)
            && !DockLayout.WideGrowsRight(1872f - 624f, 1872f, 0f, 1920f));

        float tall = DockLayout.NextPictureGrowth(0f, 500f, 597f, 1.2f);
        Check("a picture grows beside taller inputs, up to a limit", tall > 0f && tall <= 597f * 0.62f - DockLayout.NaturalPictureHeight(597f, 1.2f) + 0.01f);
        Check("a picture does not grow past the inputs", DockLayout.NextPictureGrowth(40f, -10f, 597f, 1.2f) == 30f);
        Check("a picture gives its growth back", DockLayout.NextPictureGrowth(10f, -500f, 597f, 1.2f) == 0f);

        CardPlace? card = DockLayout.CardBeside(2688f, 3792f, 0f, 3840f, 1045f, 300f, 34f, 400f, 9.6f);
        Check("a card stands left of a window at the right edge", card is CardPlace { PivotX: 1f, PivotY: 0f } c1 && c1.X == 2688f - 9.6f && c1.Y == 300f);
        CardPlace? low = DockLayout.CardBeside(2688f, 3792f, 0f, 3840f, 1045f, 1500f, 34f, 400f, 9.6f);
        Check("a card under the middle of the screen grows up from its row", low is CardPlace { PivotY: 1f } c2 && c2.Y == 1534f);
        CardPlace? right = DockLayout.CardBeside(100f, 724f, 0f, 3840f, 1045f, 300f, 34f, 400f, 9.6f);
        Check("a card stands right of a window at the left edge", right is CardPlace { PivotX: 0f } c3 && c3.X == 724f + 9.6f);
        Check("with no room on both sides a card follows the mouse", DockLayout.CardBeside(100f, 1200f, 0f, 1280f, 360f, 300f, 34f, 400f, 9.6f) == null);

        DockRect r = DockLayout.Console(0f, 0f, 1920f, 1080f, 1f, wide: false);
        Check("dock width", Math.Abs(r.W - 520f) < 0.01f, r.W.ToString());
        Check("dock right margin of the stock planner", Math.Abs(1920f - r.Right - 40f) < 0.01f, r.Right.ToString());
        Check("dock top of the stock planner", Math.Abs(r.Y - 50f) < 0.01f);
        Check("dock height is the floor", Math.Abs(r.H - 600f) < 0.01f, r.H.ToString());
        DockRect w = DockLayout.Console(0f, 0f, 1920f, 1080f, 1f, wide: true);
        Check("wide width", Math.Abs(w.W - 920f) < 0.01f);
        Check("wide keeps the right edge", Math.Abs(w.Right - r.Right) < 0.01f);
        DockRect scaled = DockLayout.Console(0f, 0f, 3840f, 2160f, 2f, wide: false);
        Check("dock scales", Math.Abs(scaled.W - 1040f) < 0.01f && Math.Abs(scaled.Y - 100f) < 0.01f);
        DockRect small = DockLayout.Console(0f, 0f, 800f, 600f, 1f, wide: true);
        Check("wide clamps to a small screen", small.W <= 800f - 80f + 0.01f && small.X >= 0f, small.W.ToString());
        (float min, float max) = DockLayout.WidthLimits(1f, wide: false, 1920f);
        Check("narrow limits: the pilot may widen to the screen", min == 460f && max == 1920f - 32f, min + " " + max);
        Check("stock left edge", Math.Abs(DockLayout.StockLeft(0f, 1920f, 1f) - 1480f) < 0.01f);
        DockRect strip = DockLayout.StripBesideStock(0f, 1920f, 1f, 50f, 460f, 200f);
        Check("strip beside the stock planner", Math.Abs(strip.Right - (1480f - 8f)) < 0.01f);

        // The stock panel at 1920 x 1080: its top at 820 px, from x 1124 to 1901.
        var panel = new DockRect(1124f, 820f, 777f, 240f);
        (float floor, float limit) = DockLayout.HeightRange(50f, 1360f, 1880f, 1080f, 1f, [panel], float.NaN);
        Check("a short page has no tall floor", Math.Abs(floor - 200f) < 0.01f);
        Check("limit at the top of the panel less the gap", Math.Abs(limit - (820f - 8f - 50f)) < 0.01f, limit.ToString());
        (_, limit) = DockLayout.HeightRange(50f, 200f, 720f, 1080f, 1f, [panel], float.NaN);
        Check("a window beside the panel has the fixed limit, inside the screen", Math.Abs(limit - Math.Min(1050f, 1080f - 50f - 8f)) < 0.01f, limit.ToString());
        (_, limit) = DockLayout.HeightRange(900f, 1360f, 1880f, 2160f, 1f, [panel], float.NaN);
        Check("a window below the top of the panel has the fixed limit", Math.Abs(limit - 1050f) < 0.01f, limit.ToString());
        (_, limit) = DockLayout.HeightRange(50f, 1360f, 1880f, 1080f, 1f, [], float.NaN);
        Check("with no panel the limit is the fixed one, inside the screen", Math.Abs(limit - 1022f) < 0.01f, limit.ToString());
        (floor, limit) = DockLayout.HeightRange(50f, 1360f, 1880f, 1080f, 1f, [panel], float.NaN, rows: 300f);
        Check("the rows of a page are its floor", Math.Abs(floor - 300f) < 0.01f && limit > floor);
        (floor, limit) = DockLayout.HeightRange(500f, 1360f, 1880f, 1080f, 1f, [panel], float.NaN, rows: 400f);
        Check("the rows win over the panel", Math.Abs(limit - 400f) < 0.01f && Math.Abs(floor - 400f) < 0.01f, floor + " " + limit);
        (floor, limit) = DockLayout.HeightRange(500f, 1360f, 1880f, 1080f, 1f, [panel], float.NaN, rows: 900f);
        Check("the screen wins over the rows", Math.Abs(limit - 572f) < 0.01f && Math.Abs(floor - 572f) < 0.01f, floor + " " + limit);
        (floor, limit) = DockLayout.HeightRange(50f, 1360f, 1880f, 1080f, 1f, [panel], 500f);
        Check("the dragged height is the height", Math.Abs(limit - 500f) < 0.01f && Math.Abs(floor - 500f) < 0.01f);
        (floor, limit) = DockLayout.HeightRange(50f, 1360f, 1880f, 1080f, 1f, [panel], 500f, rows: 600f);
        Check("the saved height wins over the rows", Math.Abs(limit - 500f) < 0.01f && Math.Abs(floor - 500f) < 0.01f);
        (_, limit) = DockLayout.HeightRange(50f, 1360f, 1880f, 3000f, 1f, [], 1400f);
        Check("a saved height replaces the fixed limit", Math.Abs(limit - 1400f) < 0.01f, limit.ToString());
        (_, limit) = DockLayout.HeightRange(50f, 1360f, 1880f, 1080f, 1f, [panel], 900f);
        Check("a dragged height may cover the panel", Math.Abs(limit - 900f) < 0.01f, limit.ToString());
        Check("a dragged height stays after the release", Math.Abs(DockLayout.FollowHeight(900f, 1000f, 900f, limit, true) - 900f) < 0.01f);
        (_, limit) = DockLayout.HeightRange(50f, 1360f, 1880f, 1080f, 1f, [], 2000f);
        Check("the screen wins over the saved height", Math.Abs(limit - 1022f) < 0.01f, limit.ToString());

        var burnControl = new DockRect(3195f, 848f, 608f, 402f);
        var rendezvous = new DockRect(2949f, 1264f, 854f, 314f);
        var autopilot = new DockRect(2300f, 1589f, 1503f, 464f);
        (_, limit) = DockLayout.HeightRange(60f, 3168f, 3792f, 2090f, 1.2f, [autopilot], float.NaN);
        Check("the autopilot panel stops the window, past the fixed limit", Math.Abs(limit - (1589f - 9.6f - 60f)) < 0.01f, limit.ToString());
        (_, limit) = DockLayout.HeightRange(60f, 3168f, 3792f, 2090f, 1.2f, [rendezvous, autopilot], float.NaN);
        Check("the target panel stops it higher", Math.Abs(limit - (1264f - 9.6f - 60f)) < 0.01f, limit.ToString());
        (_, limit) = DockLayout.HeightRange(60f, 3168f, 3792f, 2090f, 1.2f, [burnControl, rendezvous, autopilot], float.NaN);
        Check("the burn panel stops it highest", Math.Abs(limit - (848f - 9.6f - 60f)) < 0.01f, limit.ToString());
        Check("a node that appears takes the window to the panel at once", Math.Abs(DockLayout.FollowHeight(1200f, 1500f, 400f, limit, false) - limit) < 0.01f);
        (_, float noPanel) = DockLayout.HeightRange(60f, 3168f, 3792f, 2090f, 1.2f, [rendezvous, autopilot], float.NaN);
        Check("a node that goes changes nothing for a page that fits", Math.Abs(DockLayout.FollowHeight(600f, 600f, 400f, noPanel, false) - 600f) < 0.01f);
        // 1920 x 1080 at scale 1: the burn panel starts at 438 px, so a window at its dock has 380 px, which holds the frame and a short page body.
        var burnSmall = new DockRect(1587f, 438f, 314f, 208f);
        (floor, limit) = DockLayout.HeightRange(50f, 1360f, 1880f, 1080f, 1f, [burnSmall], float.NaN, rows: 310f);
        Check("on 1080p the window stays over the burn panel", Math.Abs(limit - 380f) < 0.01f && Math.Abs(floor - 310f) < 0.01f, floor + " " + limit);

        Check("the first height is the content", Math.Abs(DockLayout.FollowHeight(float.NaN, 700f, 600f, 800f, false) - 700f) < 0.01f);
        Check("height grows at once", Math.Abs(DockLayout.FollowHeight(650f, 700f, 600f, 800f, false) - 700f) < 0.01f);
        Check("height does not shrink for a live value", Math.Abs(DockLayout.FollowHeight(700f, 650f, 600f, 800f, false) - 700f) < 0.01f);
        Check("height shrinks on a page change", Math.Abs(DockLayout.FollowHeight(700f, 650f, 600f, 800f, true) - 650f) < 0.01f);
        Check("height stops at the floor", Math.Abs(DockLayout.FollowHeight(700f, 300f, 600f, 800f, true) - 600f) < 0.01f);
        Check("height stops at the limit", Math.Abs(DockLayout.FollowHeight(700f, 2000f, 600f, 800f, false) - 800f) < 0.01f);
        Check("a lower limit moves the height", Math.Abs(DockLayout.FollowHeight(900f, 2000f, 600f, 800f, false) - 800f) < 0.01f);

        Check("picture keeps its aspect with room", Math.Abs(DockLayout.PictureHeight(436f, 0f, 1f) - 190f) < 0.5f);
        Check("picture gives up height down to the minimum", Math.Abs(DockLayout.PictureHeight(436f, 20f, 1f) - 170f) < 0.5f
            && Math.Abs(DockLayout.PictureHeight(436f, 100f, 1f) - 150f) < 0.5f);
        Check("overflow takes the picture to the minimum and no further", Math.Abs(DockLayout.NextPictureShrink(0f, 500f, 436f, 1f) - 40f) < 0.5f);
        Check("room gives the picture back", Math.Abs(DockLayout.NextPictureShrink(40f, -15f, 436f, 1f) - 25f) < 0.5f
            && DockLayout.NextPictureShrink(10f, -50f, 436f, 1f) == 0f);
    }

    private static void PorkchopGrid()
    {
        // A bowl: the lowest value at (100, 200), every entry usable but one block.
        double Value(int r, int c) => 1000.0 + Math.Abs(r - 100) * 3.0 + Math.Abs(c - 200) * 2.0;
        bool Usable(int r, int c) => !(r < 4 && c < 4);
        PorkchopRaster raster = PorkchopRaster.Build(Value, Usable);
        Check("raster finds the minimum", Math.Abs(raster.MinValue - 1000.0) < 1e-9);
        (int row, int col) = raster.EntryOf(100 / PorkchopRaster.Block, 200 / PorkchopRaster.Block);
        Check("cell maps back to its lowest entry", row == 100 && col == 200, $"{row},{col}");
        Check("unusable block has no entry", raster.EntryOf(0, 0) == (-1, -1));
        Check("outside the grid has no entry", raster.EntryOf(-1, 0) == (-1, -1) && raster.EntryOf(0, PorkchopRaster.Cells) == (-1, -1));

        int cells = 0;
        bool rowsCovered = true;
        var perRow = new int[PorkchopRaster.Cells];
        foreach (PorkchopRaster.Run run in raster.Runs)
        {
            cells += run.Col1 - run.Col0 + 1;
            perRow[run.Row] += run.Col1 - run.Col0 + 1;
            if (run.Col0 > run.Col1)
                rowsCovered = false;
        }
        Check("runs cover every cell once", cells == PorkchopRaster.Cells * PorkchopRaster.Cells && rowsCovered);
        Check("every row is complete", Array.TrueForAll(perRow, n => n == PorkchopRaster.Cells));
        Check("runs merge equal neighbours", raster.Runs.Count < PorkchopRaster.Cells * PorkchopRaster.Cells);
        Check("at most 5329 rectangles", raster.Runs.Count <= 5329);

        Check("level of the minimum is 0", PorkchopRaster.LevelOf(10.0, 10.0, 100.0) == 0);
        Check("level of the maximum is the top level", PorkchopRaster.LevelOf(100.0, 10.0, 100.0) == PorkchopRaster.Levels - 1);
        byte middle = PorkchopRaster.LevelOf(Math.Sqrt(1000.0), 10.0, 100.0);
        Check("level is logarithmic", middle == PorkchopRaster.Levels / 2 || middle == PorkchopRaster.Levels / 2 - 1, middle.ToString());
        Check("no value has no level", PorkchopRaster.LevelOf(double.NaN, 10.0, 100.0) == PorkchopRaster.NoLevel);
        static Rgb8 Of((byte R, byte G, byte B) c) => new(c.R, c.G, c.B);
        Rgb8 low = Of(PorkchopRaster.LevelColor(0));
        Rgb8 high = Of(PorkchopRaster.LevelColor(PorkchopRaster.Levels - 1));
        Check("the lowest level is the bright end of viridis", Of(PorkchopRaster.RampColor(0.0)) == new Rgb8(253, 231, 37) && low.R > 200 && low.G > 200, low.ToString());
        Check("the highest level is the dark end of viridis", Of(PorkchopRaster.RampColor(1.0)) == new Rgb8(68, 1, 84) && high.R < 90 && high.G < 30, high.ToString());
        Check("the ramp holds the stops of viridis", Of(PorkchopRaster.RampColor(0.25)) == new Rgb8(94, 201, 98) && Of(PorkchopRaster.RampColor(0.5)) == new Rgb8(33, 145, 140) && Of(PorkchopRaster.RampColor(0.75)) == new Rgb8(59, 82, 139));
        bool darker = true, noRed = true, hasHue = true;
        double lastLight = double.MaxValue;
        for (int level = 0; level < PorkchopRaster.Levels; level++)
        {
            Rgb8 c = Of(PorkchopRaster.LevelColor((byte)level));
            double light = ColorRules.Luminance(c);
            darker &= light <= lastLight;
            lastLight = light;
            double hue = ColorRules.Hue(c);
            noRed &= hue > 40.0 && hue < 320.0;
            hasHue &= ColorRules.Saturation(c) > 0.4;
        }
        Check("a higher level is always darker, so low reads as bright and good", darker);
        Check("no level is red, so no window reads as a warning", noRed);
        Check("every level has a clear hue", hasHue);
        Rgb8 none = Of(PorkchopRaster.LevelColor(PorkchopRaster.NoLevel));
        Check("a cell with no transfer has no hue, so it reads as a hole and not as a value", ColorRules.Saturation(none) < 0.2);
        bool legendSame = true;
        for (int level = 0; level < PorkchopRaster.Levels; level++)
            legendSame &= PorkchopRaster.LegendColor(level, PorkchopRaster.Levels) == PorkchopRaster.LevelColor((byte)level);
        Check("the legend and the plot read one ramp", legendSame);
        Check("the legend starts at the lowest value", Of(PorkchopRaster.LegendColor(0, 16)).G > Of(PorkchopRaster.LegendColor(15, 16)).G);

        PorkchopRaster paid = PorkchopRaster.Build((r, c) => 1000.0 + r * c, (r, c) => true);
        Check("a raster with a value everywhere has no hole", !paid.HasNoValue);
        Check("a raster with a gap has a hole", PorkchopRaster.Build((r, c) => 1000.0 + r, (r, c) => r > PorkchopRaster.Block).HasNoValue);
        Check("a craft that can pay nothing dims every cell", paid.PaidLevel(paid.MinValue - 1.0) == 0 && PorkchopRaster.CellColor(0, paid.PaidLevel(paid.MinValue - 1.0)) != PorkchopRaster.LevelColor(0));
        Check("the ramp says that a craft is below the plot", paid.BudgetBelow(paid.MinValue - 1.0) && !paid.BudgetBelow(paid.MinValue + 1.0) && !paid.BudgetBelow(double.NaN));
        Check("and above it", paid.BudgetAbove(paid.MaxValue + 1.0) && !paid.BudgetAbove(paid.MaxValue - 1.0) && !paid.BudgetAbove(double.NaN));
        Check("a craft that can pay all dims nothing", paid.PaidLevel(paid.MaxValue + 1.0) == PorkchopRaster.NothingDims);
        Check("no value to read dims nothing", paid.PaidLevel(double.NaN) == PorkchopRaster.NothingDims);
        int budget = paid.PaidLevel(Math.Sqrt(paid.MinValue * paid.MaxValue));
        Check("a budget inside the range splits the levels", budget > 0 && budget < PorkchopRaster.Levels - 1, budget.ToString());
        Check("a cell the craft can pay keeps the full colour", PorkchopRaster.CellColor(0, budget) == PorkchopRaster.LevelColor(0));
        Check("the level of the budget dims, because it also holds cells over the budget", PorkchopRaster.CellColor((byte)budget, budget) != PorkchopRaster.LevelColor((byte)budget) && PorkchopRaster.CellColor((byte)(budget - 1), budget) == PorkchopRaster.LevelColor((byte)(budget - 1)));
        Check("a level above all budgets dims nothing", PorkchopRaster.CellColor(PorkchopRaster.Levels - 1, PorkchopRaster.NothingDims) == PorkchopRaster.LevelColor(PorkchopRaster.Levels - 1));
        Check("a cell the craft cannot pay is dim and keeps its hue", ColorRules.Luminance(Of(PorkchopRaster.CellColor(PorkchopRaster.Levels - 1, budget))) < ColorRules.Luminance(high)
            && Math.Abs(ColorRules.Hue(Of(PorkchopRaster.CellColor(PorkchopRaster.Levels - 1, budget))) - ColorRules.Hue(high)) < 6.0);
        Check("a hole stays a hole whatever the craft can pay", PorkchopRaster.CellColor(PorkchopRaster.NoLevel, budget) == PorkchopRaster.LevelColor(PorkchopRaster.NoLevel));
        Check("the tick stands inside the range", Math.Abs(paid.LegendShare(Math.Sqrt(paid.MinValue * paid.MaxValue)) - 0.5) < 1e-6);
        Check("no tick outside the range", double.IsNaN(paid.LegendShare(paid.MinValue - 1.0)) && double.IsNaN(paid.LegendShare(paid.MaxValue + 1.0)));
        Check("the legend says what the craft has", PageText.PorkchopOnBoard(1949.0) == "HAS 1949 m/s");
        Check("the legend stays short enough for each width", PageText.PorkchopOnBoard(99999.0).Length <= 13);
        Check("the legend says nothing with no value", PageText.PorkchopOnBoard(double.NaN).Length == 0);
        Check("a mark in the ring goes, so the ring holds nothing", !PorkchopRaster.MarkShows(100, 100, 100, 100, 6.0) && !PorkchopRaster.MarkShows(100, 103, 100, 100, 6.0));
        Check("a mark clear of the ring stays", PorkchopRaster.MarkShows(100, 120, 100, 100, 6.0));
        Check("a mark shows with no selection", PorkchopRaster.MarkShows(100, 100, -1, -1, 6.0));
        Check("a mark with no cell does not show", !PorkchopRaster.MarkShows(-1, -1, 100, 100, 6.0));
        Check("the legend dims all of itself with no cell in reach", paid.PaidShare(paid.MinValue - 1.0) == 0.0 && PorkchopRaster.LegendColor(0, 16, 0.0) != PorkchopRaster.LegendColor(0, 16));
        Check("the legend dims nothing with every cell in reach", paid.PaidShare(paid.MaxValue + 1.0) == 1.0 && paid.PaidShare(double.NaN) == 1.0);
        double paidMiddle = paid.PaidShare(Math.Sqrt(paid.MinValue * paid.MaxValue));
        Check("the legend dims from the value on", paidMiddle > 0.0 && paidMiddle < 1.0 && PorkchopRaster.LegendColor(0, 16, paidMiddle) == PorkchopRaster.LegendColor(0, 16) && PorkchopRaster.LegendColor(15, 16, paidMiddle) != PorkchopRaster.LegendColor(15, 16));
        Check("a mark clear of the ring stands on its cell", PorkchopRaster.MarkPlace(100, 120, 100, 100, 6.0, -1) == (100.5, 120.5));
        Check("a mark under the ring stands on its outer edge", PorkchopRaster.MarkPlace(100, 103, 100, 100, 6.0, -1) == (100.5, 94.5));
        Check("two marks under one ring stay apart", PorkchopRaster.MarkPlace(100, 100, 100, 100, 6.0, 1) == (100.5, 106.5));
        Check("no cell, no mark", PorkchopRaster.MarkPlace(-1, -1, 100, 100, 6.0, 1) == null);

        PorkchopRaster empty = PorkchopRaster.Build((r, c) => double.NaN, (r, c) => false);
        Check("an empty result has no range", double.IsNaN(empty.MinValue) && empty.Runs.Count == PorkchopRaster.Cells);
    }

    private static void SketchGeometry()
    {
        Check("profile top fits the plan, not the apoapsis", Math.Abs(SketchMath.ProfileTopKm(200.0, 0.0, 200.0, 70.0) - 230.0) < 0.5,
            SketchMath.ProfileTopKm(200.0, 0.0, 200.0, 70.0).ToString(System.Globalization.CultureInfo.InvariantCulture));
        Check("profile top with no plan is the air", Math.Abs(SketchMath.ProfileTopKm(0.0, 0.0, 0.0, 70.0) - 80.5) < 1e-9);
        Check("profile top follows the flown track", Math.Abs(SketchMath.ProfileTopKm(100.0, 150.0, 120.0, 70.0) - 172.5) < 1e-9);
        Check("profile top has a floor of 10 km", Math.Abs(SketchMath.ProfileTopKm(0.0, 0.0, 0.0, 0.0) - 11.5) < 1e-9);
        Check("profile top ignores a value that is not finite", Math.Abs(SketchMath.ProfileTopKm(double.NaN, 0.0, 0.0, 70.0) - 80.5) < 1e-9);
        (double from, double to) = SketchMath.DrawableAnomaly(2.0);
        Check("hyperbola stays inside its asymptotes", to < Math.Acos(-0.5) && from == -to);
        Check("ellipse draws whole", SketchMath.DrawableAnomaly(0.3) == (-Math.PI, Math.PI));
        P2 q = SketchMath.Project(1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        Check("projection drops the normal", q == new P2(1.0, 2.0));

        var fit = SketchMath.Fit(-10.0, 10.0, -5.0, 5.0, 0.0, 0.0, 200.0, 100.0, 0.0);
        P2 centre = SketchMath.ToScreen(new P2(0.0, 0.0), fit);
        Check("fit centres the box", Math.Abs(centre.X - 100.0) < 1e-9 && Math.Abs(centre.Y - 50.0) < 1e-9);
        P2 up = SketchMath.ToScreen(new P2(0.0, 5.0), fit);
        Check("world up is screen up", up.Y < centre.Y);
        Check("fit keeps the aspect", Math.Abs(fit.Scale - 10.0) < 1e-9);

        Check("site at the right edge", SketchMath.BrokenAxis(0.0, 30.0, 3000.0, 0.55) == 1.0);
        Check("break point", Math.Abs(SketchMath.BrokenAxis(30.0, 30.0, 3000.0, 0.55) - 0.45) < 1e-9);
        Check("far end at the left edge", Math.Abs(SketchMath.BrokenAxis(3000.0, 30.0, 3000.0, 0.55)) < 1e-9);
        Check("beyond the far end stays at the edge", SketchMath.BrokenAxis(1e7, 30.0, 3000.0, 0.55) == 0.0);
        Check("near part is linear", Math.Abs(SketchMath.BrokenAxis(15.0, 30.0, 3000.0, 0.55) - (1.0 - 0.275)) < 1e-9);
        double a = SketchMath.BrokenAxis(100.0, 30.0, 3000.0, 0.55), b = SketchMath.BrokenAxis(1000.0, 30.0, 3000.0, 0.55);
        Check("far part falls with distance", a > b && b > 0.0);
    }

    private static void PictureLaneRules()
    {
        var frame = new PicBox(10f, 20f, 400f, 200f);
        PictureLanes lanes = PictureLayout.Split(frame, 30f, 40f, 60f, 16f, 48f);
        Check("the plot is what the lanes leave", lanes.Plot == new PicBox(50f, 50f, 300f, 154f), lanes.Plot.ToString());
        Check("no lane overlaps the plot", !lanes.Head.Overlaps(lanes.Plot) && !lanes.Left.Overlaps(lanes.Plot) && !lanes.Right.Overlaps(lanes.Plot) && !lanes.Bottom.Overlaps(lanes.Plot));
        Check("the head spans the frame", lanes.Head == new PicBox(10f, 20f, 400f, 30f));
        Check("the bottom spans the frame", lanes.Bottom == new PicBox(10f, 204f, 400f, 16f));
        Check("the legend goes first in a narrow frame", PictureLayout.Split(new PicBox(0f, 0f, 110f, 200f), 0f, 40f, 40f, 0f, 48f).Right.W == 0f
            && PictureLayout.Split(new PicBox(0f, 0f, 110f, 200f), 0f, 40f, 40f, 0f, 48f).Left.W == 40f);
        Check("the x ticks go before the head in a low frame", PictureLayout.Split(new PicBox(0f, 0f, 200f, 70f), 20f, 0f, 0f, 20f, 48f).Bottom.H == 0f
            && PictureLayout.Split(new PicBox(0f, 0f, 200f, 70f), 20f, 0f, 0f, 20f, 48f).Head.H == 20f);
        PictureLanes stripLanes = PictureLayout.Split(new PicBox(0f, 10f, 200f, 20f + 26f + 36f), 20f, 0f, 0f, 36f, 16f);
        Check("a low strip keeps its head and its bottom lane", stripLanes.Head.H == 20f && stripLanes.Bottom.H == 36f && stripLanes.Plot.H == 26f && stripLanes.Bottom.Y == 10f + 20f + 26f);
        Check("a strip with no bottom lane keeps its head", PictureLayout.Split(new PicBox(0f, 0f, 200f, 20f + 28f), 20f, 0f, 0f, 0f, 16f).Head.H == 20f);

        var box = new PicBox(10f, 10f, 20f, 10f);
        Check("a segment through a box crosses it", box.Crossed(0f, 15f, 50f, 15f));
        Check("a segment that ends in a box crosses it", box.Crossed(0f, 0f, 15f, 15f));
        Check("a segment beside a box does not", !box.Crossed(0f, 25f, 50f, 25f) && !box.Crossed(0f, 0f, 5f, 40f));
        Check("a diagonal past a corner does not", !box.Crossed(0f, 12f, 8f, 0f));

        var placer = new LabelPlacer();
        var area = new PicBox(0f, 0f, 200f, 100f);
        placer.Reset(in area);
        Check("a free box is taken", placer.Take(new PicBox(10f, 10f, 30f, 10f)));
        Check("a label does not take a place twice", !placer.Take(new PicBox(20f, 15f, 30f, 10f)));
        Check("a label stays in the plot", !placer.Take(new PicBox(190f, 10f, 30f, 10f)) && !placer.Take(new PicBox(10f, -2f, 30f, 10f)));
        placer.ReserveSegment(0f, 50f, 200f, 50f);
        Check("a label does not take a place on a curve", !placer.Take(new PicBox(50f, 45f, 30f, 10f)));
        Check("a label takes a place beside a curve", placer.Take(new PicBox(50f, 52f, 30f, 10f)));
        placer.Reserve(new PicBox(100f, 70f, 20f, 10f));
        Check("a reserved box keeps a label off", !placer.Take(new PicBox(105f, 72f, 5f, 5f)));

        placer.Reset(in area);
        Check("a label of a mark takes the corner right below it first", placer.TakePoint(100f, 50f, 5f, 20f, 10f, out PicBox first) && first.X == 105f && first.Y == 52f, first.ToString());
        Check("a second label of the mark moves on", placer.TakePoint(100f, 50f, 5f, 20f, 10f, out PicBox second) && second != first && !second.Overlaps(first), second.ToString());
        placer.Reset(in area);
        placer.ReserveSegment(100f, 0f, 100f, 100f);
        placer.ReserveSegment(0f, 50f, 200f, 50f);
        placer.TakePoint(100f, 50f, 5f, 20f, 10f, out PicBox past);
        Check("a label of a mark stays off the curves through the mark", !past.Crossed(100f, 0f, 100f, 100f) && !past.Crossed(0f, 50f, 200f, 50f), past.ToString());
        placer.Reset(new PicBox(0f, 0f, 30f, 12f));
        placer.ReserveSegment(0f, 6f, 30f, 6f);
        Check("a label with no room goes", !placer.TakePoint(15f, 6f, 2f, 20f, 8f, out _));

        placer.Reset(in area);
        placer.ReserveSegment(0f, 0f, 40f, 100f);
        Check("a label on a guide line moves off a curve to the other end", placer.TakeOnGuide(50f, 40f, 10f, 2f, out PicBox guide) && guide.X > 100f && guide.Bottom <= 50f, guide.ToString());
        placer.Reset(in area);
        Check("a label on a guide line at the top goes under it", placer.TakeOnGuide(5f, 40f, 10f, 2f, out PicBox under) && under.Y >= 5f, under.ToString());

        placer.Reset(in area);
        int mark = placer.Mark;
        placer.Take(new PicBox(10f, 10f, 30f, 10f));
        placer.Rollback(mark);
        Check("a group that goes gives its places back", placer.Take(new PicBox(10f, 10f, 30f, 10f)));

        Check("a schematic body keeps its share", Math.Abs(SketchMath.SchematicRadius(6371.0, 6371.0, 6792.0) - SketchMath.SketchBodyShare) < 1e-9);
        Check("the top of a schematic is the outer radius", Math.Abs(SketchMath.SchematicRadius(6792.0, 6371.0, 6792.0) - 1.0) < 1e-9);
        double low = SketchMath.SchematicRadius(6571.0, 6371.0, 6792.0);
        Check("a low orbit stands clear of the body and of the top", low > SketchMath.SketchBodyShare + 0.2 && low < 0.95, low.ToString(System.Globalization.CultureInfo.InvariantCulture));
        Check("a schematic radius never leaves its range", SketchMath.SchematicRadius(1.0, 6371.0, 6792.0) == SketchMath.SketchBodyShare && SketchMath.SchematicRadius(1e9, 6371.0, 6792.0) == 1.0);
        double band = SketchMath.SketchBodyShare + SketchMath.SketchAirShare;
        foreach (double topKm in new[] { 300.0, 3000.0, 20000.0 })
        {
            Check("the air is one thin band on any orbit: " + topKm, Math.Abs(SketchMath.SchematicRadius(6371.0 + 140.0, 6371.0, 6371.0 + topKm, airRadius: 6371.0 + 140.0) - band) < 1e-9);
            Check("the top of a sketch with air is the outer radius: " + topKm, Math.Abs(SketchMath.SchematicRadius(6371.0 + topKm, 6371.0, 6371.0 + topKm, airRadius: 6371.0 + 140.0) - 1.0) < 1e-9);
        }
        Check("an orbit under the top of the air lies in the band", SketchMath.SchematicRadius(6371.0 + 100.0, 6371.0, 6792.0, airRadius: 6371.0 + 140.0) < band);
        Check("an orbit over the top of the air lies outside the band", SketchMath.SchematicRadius(6371.0 + 150.0, 6371.0, 6792.0, airRadius: 6371.0 + 140.0) > band);
        Check("a sketch all inside the air has no band", SketchMath.SchematicRadius(6371.0 + 50.0, 6371.0, 6371.0 + 100.0, airRadius: 6371.0 + 140.0) == SketchMath.SchematicRadius(6371.0 + 50.0, 6371.0, 6371.0 + 100.0));
        Check("a circular orbit has no marks of its apsides", !SketchMath.ApsisDotsShow(0.0005) && SketchMath.ApsisDotsShow(0.0166));
        Check("a sketch orbit passes through its periapsis", Math.Abs(SketchMath.ConicShare(0.5, 0.9, 0.0) - 0.5) < 1e-9);
        Check("and through its apoapsis", Math.Abs(SketchMath.ConicShare(0.5, 0.9, Math.PI) - 0.9) < 1e-9);
        Check("a round orbit stays round", Math.Abs(SketchMath.ConicShare(0.7, 0.7, 1.0) - 0.7) < 1e-9);
        double mostStep = 0.0;
        for (int step = 0; step < 96; step++)
            mostStep = Math.Max(mostStep, Math.Abs(SketchMath.ConicShare(0.47, 1.0, -Math.PI + 2.0 * Math.PI * (step + 1) / 96.0) - SketchMath.ConicShare(0.47, 1.0, -Math.PI + 2.0 * Math.PI * step / 96.0)));
        Check("a sketch orbit has no jump between two samples, so it has no corner", mostStep < 0.03, mostStep.ToString());
        Check("a line under its neighbour has no legend row", !SketchMath.LineStandsClear(100.0, new[] { 98.0, 140.0 }, 4.0));
        Check("a line clear of its neighbours has one", SketchMath.LineStandsClear(110.0, new[] { 98.0, 140.0 }, 4.0));
        Check("a flat plane line keeps its length", SketchMath.PlaneLineReach(200.0, 80.0, 10.0) == 200.0);
        Check("a steep plane line ends inside the plot", SketchMath.PlaneLineReach(200.0, 80.0, 79.3) * Math.Sin(79.3 * Math.PI / 180.0) <= 80.0 + 1e-9);
        Check("a track from the pad is a climb", SketchMath.TrackIsClimb(1.0, 120.0, 140.0));
        Check("a burn in orbit is no climb", !SketchMath.TrackIsClimb(207.0, 207.5, 140.0));
        Check("a burn in the air on the way down is no climb", !SketchMath.TrackIsClimb(100.0, 50.0, 140.0));
        Check("a climb from a body with no air", SketchMath.TrackIsClimb(1.0, 50.0, 0.0) && !SketchMath.TrackIsClimb(100.0, 110.0, 0.0));
        Check("a plan at its limit lies at it", SketchMath.AtLimit(99.8, 100.0) && !SketchMath.AtLimit(90.0, 100.0) && !SketchMath.AtLimit(double.NaN, 100.0));
        Check("the root share gives the low part room", Math.Abs(SketchMath.RootShare(25.0, 100.0) - 0.5) < 1e-9 && SketchMath.RootShare(200.0, 100.0) == 1.0 && SketchMath.RootShare(-5.0, 100.0) == 0.0);

        (double dx, double dy, bool off) = SketchMath.DialPoint(3.0, 4.0, 10.0);
        Check("a drift on the scale stands where it is", Math.Abs(dx - 0.3) < 1e-9 && Math.Abs(dy - 0.4) < 1e-9 && !off);
        (dx, dy, off) = SketchMath.DialPoint(6032.0, 4495.0, 5.0);
        Check("a drift past the scale lies on the ring and says so", off && Math.Abs(Math.Sqrt(dx * dx + dy * dy) - 1.0) < 1e-9 && dx > dy && dy > 0.0);
        Check("a bar value on the scale stands where it is", SketchMath.BarPoint(1.0, 2.0) == (0.5, false));
        Check("a bar value past the scale lies on the end and says so", SketchMath.BarPoint(347.0, 2.0) == (1.0, true) && SketchMath.BarPoint(-347.0, 2.0) == (-1.0, true));

        Check("a pass is green only inside the limit", SketchMath.PassToneOf(5.0, 10.0, 400.0) == PassTone.Inside && SketchMath.PassToneOf(97.7, 10.0, 400.0) == PassTone.Plain);
        Check("a far pass reads far", SketchMath.PassToneOf(1003.5, 10.0, 400.0) == PassTone.Far);
        Check("the strip says track and no miss", PageText.PassesTitle.Contains("TRACK FROM SITE") && PageText.NextPassTitle.Contains("TRACK FROM SITE") && !PageText.NextPassTitle.Contains("MISS"));
        Check("the band of the strip has its name", PageText.PassWithin(10.0) == "WITHIN 10 km");
        Check("the distance of a pass has a clock time", PageText.PassTrack(97.74, 38678.0) == "97.7 km IN 10h 44m");
        Check("the closest pass has its row", PageText.PassClosest(97.74, 38678.0) == "CLOSEST 97.7 km IN 10h 44m");
        Check("a time to go in the T- form has no space", PageText.Tgo("IMPACT T-", "3m 00s") == "IMPACT T-3m 00s" && PageText.Tgo(PageText.LandsIn, "42s") == "LANDS T-42s" && PageText.Tgo("BURN IN", "2h 07m") == "BURN IN 2h 07m");
        Check("the next pass has its mark", PageText.PassNext(false) == "NEXT PASS");
        Check("next and closest share one row", PageText.PassNext(true) == "NEXT PASS IS CLOSEST");
        Check("the pass of a deorbit that flies is an estimate with its time first", PageText.PassFlies(112.6, 7585.0) == "PASS IN 2h 06m, NOW 112.6 km OFF");
        Check("the big number of a deorbit is the arrival, with no miss", PageText.BrakingIn(7560.0) == "BRAKING T-2h 06m");

        Check("the cause of no hover is the minimum throttle", PageText.HoverVerdict(1.60, 16.0) == "TWR 1.60 AT MIN THROTTLE. NO HOVER." && PageText.HoverThrustFails(1.60, 16.0));
        Check("a craft that can hover says so", PageText.HoverVerdict(0.70, 7.0) == "TWR 0.70 AT MIN THROTTLE. HOVER POSSIBLE." && !PageText.HoverThrustFails(0.70, 7.0));
        Check("a craft too heavy at full throttle says the same as the refused verb", PageText.HoverVerdict(0.30, 0.90) == "TWR 0.90 AT FULL THROTTLE. NO HOVER." && PageText.HoverThrustFails(0.30, 0.90));
        Check("a hover far up is called out", PageText.HoverIsFar(208000.0) && !PageText.HoverIsFar(200.0));
        Check("a stop says its height", PageText.StoppedAt(4700.0) == "STOPPED AT 4700 m" && PageText.StoppedAt(12000.0) == "STOPPED AT 12.0 km");
        Check("the Q limit names both values", PageText.QLimits(140.0, 3500.0) == "Q LIMIT 140 kPa, Q-ALPHA 3500 Pa rad");
        Check("the Q limit fits the head lane beside the big number", PageText.QLimits(140.0, 3500.0).Length * 8.7f <= 323f);
        Check("the peak Q has its unit", PageText.PeakQ(57.4) == "MAX Q 57 kPa");
        Check("the short name of the limit has its unit", PageText.QLimitShort(140.0) == "Q LIMIT 140 kPa");
        Check("the throttle floor has a name", PageText.MinThrottle(90.0) == "MIN 90 %");
        Check("the ramp names its value", PageText.PorkchopRampName(false) == "DEPARTURE DV" && PageText.PorkchopRampName(true) == "ARRIVAL SPEED");

        Check("a sequence names its stage", StageNumbers.Tag([1, 3, 5], 1) == "SEQ 3");
        Check("a split sequence has letters", StageNumbers.Tag([1, 1, 3], 0) == "SEQ 1a" && StageNumbers.Tag([1, 1, 3], 1) == "SEQ 1b" && StageNumbers.Tag([1, 1, 3], 2) == "SEQ 3");
        Check("a stage with no sequence has a number", StageNumbers.Tag([-1], 0) == "S1" && StageNumbers.Tag([], 2) == "S3");
        Check("a narrow block keeps the number", StageNumbers.ShortTag("SEQ 12") == "12" && StageNumbers.ShortTag("SEQ 1a") == "1a" && StageNumbers.ShortTag("S1") == "S1");
    }

    private static void AxisTickRules()
    {
        Check("a nice step is 1, 2 or 5 times a power of ten", AxisTicks.NiceStep(100.0, 4) == 50.0 && AxisTicks.NiceStep(758.0, 4) == 200.0 && AxisTicks.NiceStep(8.0, 4) == 2.0);
        Check("a nice step stays small for a small range", Math.Abs(AxisTicks.NiceStep(0.8, 4) - 0.2) < 1e-9);
        Check("a time step is round", AxisTicks.TimeStep(236.0, 4) == 60.0 && AxisTicks.TimeStep(236.0, 8) == 30.0 && AxisTicks.TimeStep(3600.0, 4) == 900.0);
        Check("a long time step is in days", AxisTicks.TimeStep(86400.0 * 780.0, 4) == 86400.0 * 200.0);
        Span<double> ticks = stackalloc double[8];
        int n = AxisTicks.Fill(0.0, 758.0, 200.0, ticks);
        Check("ticks are multiples of the step", n == 4 && ticks[0] == 0.0 && ticks[3] == 600.0);
        n = AxisTicks.Fill(-21.47, 90.0, 50.0, ticks);
        Check("ticks start at the first multiple in the range", n == 2 && ticks[0] == 0.0 && ticks[1] == 50.0);
        Check("a zero tick is exact", AxisTicks.Fill(-0.2, 0.2, 0.1, ticks) == 5 && ticks[2] == 0.0);
        Check("a tick count is limited by its buffer", AxisTicks.Fill(0.0, 1000.0, 1.0, ticks) == 8);
        Check("no step gives no ticks", AxisTicks.Fill(0.0, 1.0, 0.0, ticks) == 0 && AxisTicks.Fill(double.NaN, 1.0, 0.1, ticks) == 0);
        Check("a range widens to round values", AxisTicks.Outward(-21.47, 88.0, 20.0) == (-40.0, 100.0));
        Span<double> root = stackalloc double[4];
        Check("a root axis has four round ticks", AxisTicks.RootTicks(215.0, root) == 4 && root[0] == 0.0 && root[1] == 10.0 && root[2] == 50.0 && root[3] == 200.0, $"{root[1]},{root[2]},{root[3]}");
        Span<double> broken = stackalloc double[12];
        n = AxisTicks.BrokenTicks(30.0, 3000.0, broken);
        Check("a broken axis has the near part and the powers of ten", n == 6 && broken[0] == 0.0 && broken[1] == 10.0 && broken[3] == 30.0 && broken[4] == 100.0 && broken[5] == 1000.0, n.ToString());
        Check("a km tick has its unit", AxisTicks.Km(200.0) == "200 km" && AxisTicks.Km(2.5) == "2.5 km" && AxisTicks.Km(0.0) == "0");
        Check("a distance tick reads in its unit", AxisTicks.Distance(500.0) == "500 m" && AxisTicks.Distance(2000.0) == "2 km" && AxisTicks.Distance(2500.0) == "2.5 km" && AxisTicks.Distance(0.0) == "0");
        Check("a time tick has the one time form", AxisTicks.Time(0.0) == "0" && AxisTicks.Time(60.0) == "60 s" && AxisTicks.Time(120.0) == "2m" && AxisTicks.Time(210.0) == "3m 30s" && AxisTicks.Time(86400.0 * 200.0) == "200d");
        Check("an angle tick has its unit", AxisTicks.Degrees(-20.0) == "-20 deg");
        Check("a percent tick has its unit", AxisTicks.Percent(100.0) == "100 %");
        Check("a tick has no false digits", AxisTicks.Number(0.30000000000000004) == "0.3");
    }

    private static void BurnBudgets()
    {
        const double ve = 3000.0;
        double m = BurnBudget.MassForDv(10000.0, 1000.0, ve);
        Check("rocket equation round trip", Math.Abs(BurnBudget.DvFromMass(10000.0, 10000.0 - m, ve) - 1000.0) < 1e-6);
        var stages = new[]
        {
            new StageFuel(2, 10000.0, 3000.0, ve),
            new StageFuel(1, 5000.0, 2000.0, 3500.0),
        };
        double onBoard = BurnBudget.OnBoard(stages);
        Check("on board sums the stages", Math.Abs(onBoard - (ve * Math.Log(10000.0 / 7000.0) + 3500.0 * Math.Log(5000.0 / 3000.0))) < 1e-6);

        BurnPay[] pays = BurnBudget.Pay(stages, new[] { 700.0, 700.0 });
        Check("first burn paid by the first stage", pays[0].FirstStage == 0 && pays[0].LastStage == 0 && pays[0].ShortMs == 0.0);
        Check("second burn spills into the next stage", pays[1].FirstStage == 0 && pays[1].LastStage == 1, $"{pays[1].FirstStage}-{pays[1].LastStage}");
        Check("propellant is positive", pays[0].PropellantKg > 0.0 && pays[1].PropellantKg > 0.0);

        BurnPay[] tooMuch = BurnBudget.Pay(stages, new[] { onBoard + 100.0 });
        Check("a burn beyond the craft is short", Math.Abs(tooMuch[0].ShortMs - 100.0) < 1e-3, tooMuch[0].ShortMs.ToString());
        BurnPay[] noStages = BurnBudget.Pay(Array.Empty<StageFuel>(), new[] { 10.0 });
        Check("no stages pay nothing", noStages[0].FirstStage == -1 && Math.Abs(noStages[0].ShortMs - 10.0) < 1e-9);
    }

    private static void PlanVerbRules()
    {
        Check("one pass creates a node", PlanVerbs.Create(1) == "CREATE BURN");
        Check("passes say how many", PlanVerbs.Create(3) == "CREATE 3 PASSES");
        Check("one pass is flown by stock AUTO", PlanVerbs.FlownByFor(1) == FlownBy.StockAuto);
        Check("passes are flown per pass", PlanVerbs.FlownByFor(4) == FlownBy.AfcPlansStockAuto);
        Check("one verb flies a burn, and the mark names who", PlanVerbs.FlyBurn == "FLY BURN");
        Check("the delete verb says burn, as stock does", PlanVerbs.DeleteBurn == "DELETE BURN");
        var withSecondary = new CommitSpec("GO", FlownBy.StockAuto, "NO", Secondary: new CommitAction("DEL", "", "", () => { }));
        Check("a secondary verb does not enable a refused primary", !withSecondary.Enabled && withSecondary.Secondary!.Enabled);
        var secondaryRefused = new CommitSpec("GO", FlownBy.StockAuto, Act: () => { }, Secondary: new CommitAction("DEL", "NO", "", () => { }));
        Check("a refused secondary verb leaves the primary enabled", secondaryRefused.Enabled && !secondaryRefused.Secondary!.Enabled);
        Check("a status-only commit is disabled", !CommitSpec.StatusOnly("X").Enabled);
        Check("a refused commit is disabled", !new CommitSpec("GO", FlownBy.Afc, "NO", Act: () => { }).Enabled);
        Check("a commit without an action is disabled", !new CommitSpec("GO", FlownBy.Afc).Enabled);
        Check("a commit with an action is enabled", new CommitSpec("GO", FlownBy.Afc, Act: () => { }).Enabled);
        Check("a take-over says so", new CommitSpec("LAND NOW", FlownBy.Afc, NeedsTakeOver: true).ButtonLabel == "TAKE OVER: LAND NOW");
        Check("a stop without an action is disabled", !StopSpec.None.Enabled);
        Check("a verb is loud unless it says otherwise", !new CommitSpec("GO", FlownBy.Afc).Quiet && new CommitSpec("RESET", FlownBy.Nobody, Quiet: true).Quiet);
    }

    private static void FirstPages()
    {
        Check("editor opens STAGING", FlightContext.FirstPage(new FlightFacts(true, StopTarget.Ascent, true, true, false, false)) == ConsolePage.Staging);
        Check("a running mode wins", FlightContext.FirstPage(new FlightFacts(false, StopTarget.Boostback, false, true, false, false)) == ConsolePage.Land);
        Check("the pad opens LAUNCH", FlightContext.FirstPage(new FlightFacts(false, StopTarget.None, true, true, false, false)) == ConsolePage.Launch);
        Check("orbit with a target opens TRANSFER", FlightContext.FirstPage(new FlightFacts(false, StopTarget.None, false, true, true, true)) == ConsolePage.Transfer);
        Check("orbit opens MANEUVER", FlightContext.FirstPage(new FlightFacts(false, StopTarget.None, false, true, true, false)) == ConsolePage.Orbit);
        Check("anything else opens LAUNCH", FlightContext.FirstPage(new FlightFacts(false, StopTarget.None, false, false, false, false)) == ConsolePage.Launch);
        var flying = new FlightFacts(false, StopTarget.None, false, true, false, false);
        Check("a craft that falls with air opens LAND", FlightContext.FirstPage(flying with { Situation = CraftSituation.FallingAir }) == ConsolePage.Land);
        Check("a craft that falls with no air opens LAND", FlightContext.FirstPage(flying with { Situation = CraftSituation.FallingNoAir }) == ConsolePage.Land);
        Check("a fall wins over the wider orbit rule", FlightContext.FirstPage(flying with { InOrbit = true, Situation = CraftSituation.FallingAir }) == ConsolePage.Land);
        Check("a running mode wins over a fall", FlightContext.FirstPage(flying with { Running = StopTarget.Ascent, Situation = CraftSituation.FallingAir }) == ConsolePage.Launch);
        Check("a craft that coasts up opens MANEUVER", FlightContext.FirstPage(flying with { Situation = CraftSituation.CoastingUp }) == ConsolePage.Orbit);
        Check("an escape path opens MANEUVER", FlightContext.FirstPage(flying with { Situation = CraftSituation.Escaping }) == ConsolePage.Orbit);
        Check("the ground opens LAUNCH on any body", FlightContext.FirstPage(flying with { Landed = true, Situation = CraftSituation.Ground }) == ConsolePage.Launch);
    }

    private static void Outline(string name, in CommitView v, string reason, ConsolePage? next = null, LandWay? way = null)
        => Check(name, v.Enabled && !v.Lit && v.Fit.Text == reason && v.Fit.NextPage == next && v.Fit.NextWay == way,
            $"enabled {v.Enabled}, lit {v.Lit}, reason '{v.Fit.Text}', next {v.Fit.NextPage}, way {v.Fit.NextWay}");

    private static void LitVerb(string name, in CommitView v)
        => Check(name, v.Enabled && v.Lit && v.Fit.Fits, $"enabled {v.Enabled}, lit {v.Lit}, reason '{v.Fit.Text}'");

    private static void SituationVerbCells()
    {
        var idle = new CraftActivity();
        // Entry guidance arrives through the registry, as GuidanceWindow.EnsureAfcModes registers it on the game side.
        GuidanceModes.Register(EntryGuidance.Descriptor());
        CommitView At(GuidanceModeId mode, CraftSituation where, bool air, bool below = false, bool gfold = false)
            => CommitResolver.Resolve(mode, idle, Open with { Where = where, HasAir = air, BelowBrakingStart = below, GfoldChosen = gfold });

        LitVerb("ground: LAUNCH", At(GuidanceModeId.AscentUpfg, CraftSituation.Ground, true));
        LitVerb("ground on a body with no air: LAUNCH", At(GuidanceModeId.AscentUpfg, CraftSituation.Ground, false));
        Outline("ground: DEORBIT", At(GuidanceModeId.DeorbitLand, CraftSituation.Ground, true), SituationVerbs.OnGround, ConsolePage.Launch);
        Outline("ground: LAND NOW with G-FOLD", At(GuidanceModeId.GfoldHere, CraftSituation.Ground, false), SituationVerbs.OnGround, ConsolePage.Launch);
        Outline("ground: LAND NOW with TOUCHDOWN", At(GuidanceModeId.SixDofHere, CraftSituation.Ground, true), SituationVerbs.OnGround, ConsolePage.Launch);
        Outline("ground: LAND NOW with ENTRY", At(GuidanceModeId.EntryHere, CraftSituation.Ground, true), SituationVerbs.OnGround, ConsolePage.Launch);
        Outline("ground: BOOSTBACK", At(GuidanceModeId.Boostback, CraftSituation.Ground, true), SituationVerbs.AfterSeparation, ConsolePage.Launch);

        Outline("orbit: LAUNCH", At(GuidanceModeId.AscentUpfg, CraftSituation.Orbit, true), SituationVerbs.InOrbitShape, ConsolePage.Orbit);
        LitVerb("orbit: DEORBIT", At(GuidanceModeId.DeorbitLand, CraftSituation.Orbit, true));
        Outline("orbit: BOOSTBACK", At(GuidanceModeId.Boostback, CraftSituation.Orbit, true), SituationVerbs.DeorbitFirst, ConsolePage.Land, LandWay.FromOrbit);
        Outline("orbit: LAND NOW with TOUCHDOWN", At(GuidanceModeId.SixDofHere, CraftSituation.Orbit, true), SituationVerbs.DeorbitFirst, ConsolePage.Land, LandWay.FromOrbit);
        Outline("orbit with air: LAND NOW with ENTRY", At(GuidanceModeId.EntryHere, CraftSituation.Orbit, true), SituationVerbs.DeorbitFirst, ConsolePage.Land, LandWay.FromOrbit);
        Outline("orbit with no air: LAND NOW with ENTRY", At(GuidanceModeId.EntryHere, CraftSituation.Orbit, false), SituationVerbs.DeorbitFirst, ConsolePage.Land, LandWay.FromOrbit);
        Outline("orbit with air: LAND NOW with G-FOLD", At(GuidanceModeId.GfoldHere, CraftSituation.Orbit, true), SituationVerbs.DeorbitFirst, ConsolePage.Land, LandWay.FromOrbit);
        Outline("orbit with no air: LAND NOW with G-FOLD", At(GuidanceModeId.GfoldHere, CraftSituation.Orbit, false), SituationVerbs.DeorbitFirst, ConsolePage.Land, LandWay.FromOrbit);
        Outline("orbit: HOVER", At(GuidanceModeId.Hover, CraftSituation.Orbit, true), SituationVerbs.DeorbitFirst, ConsolePage.Land, LandWay.FromOrbit);

        Outline("coasting up: LAUNCH", At(GuidanceModeId.AscentUpfg, CraftSituation.CoastingUp, true), SituationVerbs.CoastingShape, ConsolePage.Orbit);
        LitVerb("coasting up: DEORBIT", At(GuidanceModeId.DeorbitLand, CraftSituation.CoastingUp, true));
        LitVerb("coasting up, also under the braking start: DEORBIT", At(GuidanceModeId.DeorbitLand, CraftSituation.CoastingUp, true, below: true));
        LitVerb("coasting up: BOOSTBACK", At(GuidanceModeId.Boostback, CraftSituation.CoastingUp, true));
        LitVerb("coasting up: LAND NOW with TOUCHDOWN", At(GuidanceModeId.SixDofHere, CraftSituation.CoastingUp, true));

        Outline("falling with air: LAUNCH", At(GuidanceModeId.AscentUpfg, CraftSituation.FallingAir, true), SituationVerbs.FallingLand, ConsolePage.Land);
        LitVerb("falling with air, above the braking start: DEORBIT", At(GuidanceModeId.DeorbitLand, CraftSituation.FallingAir, true));
        Outline("falling with air, below the braking start: DEORBIT", At(GuidanceModeId.DeorbitLand, CraftSituation.FallingAir, true, below: true), SituationVerbs.BelowBrakingStart, ConsolePage.Land, LandWay.FromHere);
        LitVerb("falling with air: BOOSTBACK", At(GuidanceModeId.Boostback, CraftSituation.FallingAir, true));
        LitVerb("falling with air: LAND NOW with ENTRY", At(GuidanceModeId.EntryHere, CraftSituation.FallingAir, true));
        LitVerb("falling with air: LAND NOW with TOUCHDOWN", At(GuidanceModeId.SixDofHere, CraftSituation.FallingAir, true));
        Outline("falling with air: LAND NOW with G-FOLD", At(GuidanceModeId.GfoldHere, CraftSituation.FallingAir, true), SituationVerbs.GfoldNeedsNoAir);

        Outline("falling with no air: LAUNCH", At(GuidanceModeId.AscentUpfg, CraftSituation.FallingNoAir, false), SituationVerbs.FallingLand, ConsolePage.Land);
        LitVerb("falling with no air, above the braking start: DEORBIT", At(GuidanceModeId.DeorbitLand, CraftSituation.FallingNoAir, false));
        Outline("falling with no air, below the braking start: DEORBIT", At(GuidanceModeId.DeorbitLand, CraftSituation.FallingNoAir, false, below: true), SituationVerbs.BelowBrakingStart, ConsolePage.Land, LandWay.FromHere);
        Outline("falling with no air: LAND NOW with ENTRY", At(GuidanceModeId.EntryHere, CraftSituation.FallingNoAir, false), SituationVerbs.EntryNeedsAir);
        Outline("falling with no air, above the braking start: BOOSTBACK", At(GuidanceModeId.Boostback, CraftSituation.FallingNoAir, false), SituationVerbs.BoostbackNoAir, ConsolePage.Land, LandWay.FromOrbit);
        Outline("falling with no air, below the braking start: BOOSTBACK", At(GuidanceModeId.Boostback, CraftSituation.FallingNoAir, false, below: true), SituationVerbs.BoostbackNoAir, ConsolePage.Land, LandWay.FromHere);
        LitVerb("falling with no air: LAND NOW with G-FOLD", At(GuidanceModeId.GfoldHere, CraftSituation.FallingNoAir, false));
        LitVerb("falling with no air: LAND NOW with TOUCHDOWN", At(GuidanceModeId.SixDofHere, CraftSituation.FallingNoAir, false));

        Outline("escape path: LAUNCH", At(GuidanceModeId.AscentUpfg, CraftSituation.Escaping, true), SituationVerbs.EscapingShape, ConsolePage.Orbit);
        Outline("escape path: BOOSTBACK", At(GuidanceModeId.Boostback, CraftSituation.Escaping, true), SituationVerbs.NoImpact);
        Outline("escape path: LAND NOW with TOUCHDOWN", At(GuidanceModeId.SixDofHere, CraftSituation.Escaping, true), SituationVerbs.NoImpact);
        Outline("escape path with air: LAND NOW with ENTRY", At(GuidanceModeId.EntryHere, CraftSituation.Escaping, true), SituationVerbs.NoImpact);
        Outline("escape path with no air: LAND NOW with ENTRY", At(GuidanceModeId.EntryHere, CraftSituation.Escaping, false), SituationVerbs.NoImpact);
        Outline("escape path with no air: LAND NOW with G-FOLD", At(GuidanceModeId.GfoldHere, CraftSituation.Escaping, false), SituationVerbs.NoImpact);
        Outline("escape path: HOVER", At(GuidanceModeId.Hover, CraftSituation.Escaping, false), SituationVerbs.NoImpact);
        LitVerb("coasting up: HOVER", At(GuidanceModeId.Hover, CraftSituation.CoastingUp, true));
        LitVerb("falling: HOVER", At(GuidanceModeId.Hover, CraftSituation.FallingNoAir, false));
        LitVerb("ground: HOVER, because a lit engine lifts off", At(GuidanceModeId.Hover, CraftSituation.Ground, true));

        Outline("a deorbit that ends with G-FOLD on a body with air", At(GuidanceModeId.DeorbitLand, CraftSituation.Orbit, true, gfold: true), SituationVerbs.GfoldNeedsNoAir);
        LitVerb("a deorbit that ends with G-FOLD on a body with no air", At(GuidanceModeId.DeorbitLand, CraftSituation.Orbit, false, gfold: true));

        foreach (GuidanceModeId mode in new[] { GuidanceModeId.AscentUpfg, GuidanceModeId.DeorbitLand, GuidanceModeId.Boostback, GuidanceModeId.GfoldHere, GuidanceModeId.SixDofHere, GuidanceModeId.Hover })
            LitVerb("a situation that is not known fits " + mode, CommitResolver.Resolve(mode, idle, Open));

        Outline("the first obstacle goes first: a hover the start would refuse in orbit", CommitResolver.Resolve(GuidanceModeId.Hover, idle, Open with { Where = CraftSituation.Orbit, HoverProblem = PageText.HoverNoEngine }), SituationVerbs.DeorbitFirst, ConsolePage.Land, LandWay.FromOrbit);
        Outline("the hover test needs no known situation", CommitResolver.Resolve(GuidanceModeId.Hover, idle, Open with { HoverProblem = PageText.HoverNoThrust }), PageText.PickAnotherWay, ConsolePage.Land, LandWay.FromHere);
        CommitView refused = CommitResolver.Resolve(GuidanceModeId.Hover, idle, Open with { HoverTwrBelowOne = true, HoverProblem = PageText.HoverNoEngine });
        Check("the refusal that exists today stays hard", !refused.Enabled && refused.Fit.Fits);
        Check("a page fact makes an outline", CommitResolver.Resolve(GuidanceModeId.SixDofHere, idle, Open with { PageFact = "Cannot stop." }) is { Enabled: true, Lit: false, Fit.Reason: "Cannot stop." });
        Check("the situation comes before a page fact",
            CommitResolver.Resolve(GuidanceModeId.SixDofHere, idle, Open with { Where = CraftSituation.Ground, PageFact = "Cannot stop." }).Fit.Reason == SituationVerbs.OnGround);
        CommitView off = CommitResolver.Resolve(GuidanceModeId.DeorbitLand, idle, Open with { LandingSwitchesOff = true, Where = CraftSituation.Ground });
        Check("a refusal keeps its own reason", !off.Enabled && off.Fit.Fits && off.Reason == CommitResolver.LandingSwitchesOffReason);
        CommitView live = CommitResolver.Resolve(GuidanceModeId.AscentUpfg, new CraftActivity { AscentRunning = true }, Open with { Where = CraftSituation.CoastingUp });
        Check("the mode that flies has no fit to say", live.Live && !live.Enabled && !live.Lit && live.Fit.Fits);
        CommitView takeOver = CommitResolver.Resolve(GuidanceModeId.Boostback, new CraftActivity { AscentRunning = true }, Open);
        Check("a take-over is not lit, and it still works", takeOver.Enabled && takeOver.NeedsTakeOver && !takeOver.Lit);

        Check("orbit: MANEUVER is lit", SituationVerbs.ForManeuver(CraftSituation.Orbit).Fits);
        Check("coasting up: MANEUVER stays lit, circularize is the next step", SituationVerbs.ForManeuver(CraftSituation.CoastingUp).Fits);
        Check("falling: MANEUVER stays lit", SituationVerbs.ForManeuver(CraftSituation.FallingAir).Fits && SituationVerbs.ForManeuver(CraftSituation.FallingNoAir).Fits);
        Check("ground: MANEUVER", SituationVerbs.ForManeuver(CraftSituation.Ground) is { Reason: SituationVerbs.OnGround, NextPage: ConsolePage.Launch });
        Check("orbit: TRANSFER is lit", SituationVerbs.ForTransfer(CraftSituation.Orbit).Fits);
        Check("coasting up: TRANSFER is lit", SituationVerbs.ForTransfer(CraftSituation.CoastingUp).Fits);
        Check("ground: TRANSFER", SituationVerbs.ForTransfer(CraftSituation.Ground) is { Reason: SituationVerbs.OnGround, NextPage: ConsolePage.Launch });
        Check("falling: TRANSFER", SituationVerbs.ForTransfer(CraftSituation.FallingAir) is { Reason: SituationVerbs.FallingLand, NextPage: ConsolePage.Land }
            && SituationVerbs.ForTransfer(CraftSituation.FallingNoAir) is { Reason: SituationVerbs.FallingLand, NextPage: ConsolePage.Land });
        Check("an unknown situation fits MANEUVER and TRANSFER", SituationVerbs.ForManeuver(CraftSituation.Unknown).Fits && SituationVerbs.ForTransfer(CraftSituation.Unknown).Fits);
        Check("the word is the ground and not the pad", !SituationVerbs.OnGround.Contains("pad"));

        Check("orbit fits DEORBIT", LandWays.FitsNow(CraftSituation.Orbit, true, false) == LandWay.FromOrbit);
        Check("coasting up with air fits BOOSTBACK", LandWays.FitsNow(CraftSituation.CoastingUp, true, true) == LandWay.Boostback);
        Check("coasting up with no air fits DEORBIT", LandWays.FitsNow(CraftSituation.CoastingUp, false, false) == LandWay.FromOrbit);
        Check("falling with air fits LAND NOW", LandWays.FitsNow(CraftSituation.FallingAir, true, false) == LandWay.FromHere);
        Check("falling with no air above the braking start fits DEORBIT", LandWays.FitsNow(CraftSituation.FallingNoAir, false, false) == LandWay.FromOrbit);
        Check("falling with no air below the braking start fits LAND NOW", LandWays.FitsNow(CraftSituation.FallingNoAir, false, true) == LandWay.FromHere);
        Check("no way fits the ground, an escape or no knowledge", LandWays.FitsNow(CraftSituation.Ground, true, true) == null
            && LandWays.FitsNow(CraftSituation.Escaping, false, false) == null && LandWays.FitsNow(CraftSituation.Unknown, false, false) == null);

        Check("the next link names the page", PageText.NextLabel(ConsolePage.Orbit, null) == "MANEUVER >");
        Check("the next link names the way", PageText.NextLabel(ConsolePage.Land, LandWay.FromHere) == "LAND NOW >");
        Check("the fuel sentence says how much is short", PageText.FuelShort(4220.0, 1991.0) == "NEEDS 4220 m/s, HAS ~1991");
        Check("no fuel sentence when there is enough", PageText.FuelShort(1000.0, 1991.0).Length == 0);
        Check("the card of DEORBIT names who flies", PageText.WayCard(LandWay.FromOrbit).Contains("STOCK AUTO") && PageText.WayCard(LandWay.FromOrbit).Contains("lands with TOUCHDOWN"));
        Check("the plan state of a plan that can fly is empty", PageText.PlanState("").Length == 0);
        Check("the plan state of no plan", PageText.PlanState("NO PLAN") == "NO PLAN");
        Check("the plan state of an old plan says why", PageText.PlanState("the vehicle has moved since it was planned") == "PLAN OUT OF DATE: the vehicle has moved since it was planned");
        Check("no destination names the body", PageText.NoDestination("Luna").StartsWith("No destination from Luna.", StringComparison.Ordinal) && PageText.NoDestination("").Length > 0);
        GuidanceModes.ResetToBuiltIns();
    }

    private static void FooterVerbRules()
    {
        Action act = () => { };
        var lit = new CommitSpec("GO", FlownBy.Afc, Act: act, Status: "Ready", Outcome: "Goes.", Tooltip: "Tip.");
        Check("a verb that can act and fits is lit", lit.Lit && lit.Enabled);
        Check("a refused verb is not lit and cannot act", lit with { Refusal = "No." } is { Lit: false, Enabled: false });
        Check("a verb with no action is not lit", new CommitSpec("GO", FlownBy.Afc) is { Lit: false, Enabled: false });
        Check("a quiet verb is not lit", lit with { Quiet = true } is { Lit: false, Enabled: true });
        Check("a take-over verb is not lit and still acts", lit with { NeedsTakeOver = true } is { Lit: false, Enabled: true });
        Check("a verb with a caution is not lit and still acts", lit with { Caution = "Careful." } is { Lit: false, Enabled: true });

        Check("the answer row is the status", lit.AnswerStatus == "Ready" && !lit.AnswerWarns);
        Check("a caution comes before the status", (lit with { Caution = "Careful." }) is { AnswerStatus: "Careful.", AnswerWarns: true });
        Check("a refusal comes before a caution", (lit with { Refusal = "No.", Caution = "Careful." }) is { AnswerStatus: "No.", AnswerWarns: true });
        Check("a lit verb shows its sentence", lit.ShownOutcome == "Goes.");
        Check("a caution leaves only the reason on the row", (lit with { Caution = "Careful." }).ShownOutcome.Length == 0);
        Check("a refusal leaves only the reason on the row", (lit with { Refusal = "No." }).ShownOutcome.Length == 0);
        Check("the card of a lit verb promises nothing and is its tip", lit.VerbCard == "Tip.");
        Check("the card of a quiet verb is its tip", (lit with { Quiet = true }).VerbCard == "Tip.");
        Check("the card of an outline says that it works and what it does", (lit with { Caution = "Careful." }).VerbCard == PageText.OutlineStillWorks + "\n\nGoes.\n\nTip.");
        Check("the card of a refused verb starts with the refusal", (lit with { Refusal = "No." }).VerbCard == "No.\n\nGoes.\n\nTip.");
        Check("a lit verb with no words has an empty card", new CommitSpec("GO", FlownBy.Afc, Act: act).VerbCard.Length == 0);
        Check("a quiet verb with no words has an empty card", new CommitSpec("GO", FlownBy.Afc, Act: act, Quiet: true).VerbCard.Length == 0);

        CommitSpec flying = CommitSpec.StatusOnly("AFC: ASCENT", true, FlownBy.Afc) with { Owner = Owner.Afc };
        Check("no verb shows while the mode of the page flies", flying.Verb.Length == 0 && !flying.Enabled && !flying.Lit && flying.ShownOutcome.Length == 0 && flying.AnswerStatus == "AFC: ASCENT");

        Check("the verb of TRANSFER before a porkchop", PlanVerbs.Calculate == "SOLVE");

        TakeOverAsk.Clear();
        var start = new DateTime(2026, 10, 31, 12, 0, 0, DateTimeKind.Utc);
        CommitSpec takeOver = new("HOVER", FlownBy.Afc, Status: "The ascent flies now", NeedsTakeOver: true, Act: act, TakeOverFrom: "ASCENT", Caution: "Careful.");
        Check("the take-over with a caution stands as an outline", !takeOver.Lit && takeOver.ButtonLabel == "TAKE OVER: HOVER");
        TakeOverAsk.Raise("HOVER", "ASCENT", "Ends it. Continue?", act, start);
        CommitSpec ask = TakeOverAsk.Apply(takeOver, start);
        Check("the question stands in the answer row and not the caution", ask.AnswerStatus == "Ends it. Continue?" && !ask.AnswerWarns);
        Check("CONFIRM is an outline until it arms", !ask.Lit);
        Check("CONFIRM is lit once it arms", TakeOverAsk.Apply(takeOver, start.AddSeconds(TakeOverAsk.ArmDelaySeconds + 0.1)).Lit);
        TakeOverAsk.Clear();

        StageFuel[] stages = { new(1, 1000.0, 500.0, 3000.0) };
        double onBoard = BurnBudget.OnBoard(stages);
        Check("a burn inside the budget is not short", !BurnBudget.IsShort(onBoard, onBoard));
        Check("a burn past the budget is short", BurnBudget.IsShort(onBoard, onBoard * 1.01));
        Check("a burn after the budget is gone is short", BurnBudget.IsShort(0.0, 1000.0));
    }

    private static void ModeRegistry()
    {
        Check("entry guidance is not built in", GuidanceModes.Of(GuidanceModeId.EntryHere) == null);
        // Entry guidance arrives through the registry, as GuidanceWindow.EnsureAfcModes registers it on the game side.
        GuidanceModes.Register(EntryGuidance.Descriptor());
        foreach (GuidanceModeId id in Enum.GetValues<GuidanceModeId>())
            Check("every mode id is registered " + id, GuidanceModes.Of(id) != null);
        GuidanceModeDescriptor? gravity = GuidanceModes.Of(GuidanceModeId.AscentGravityTurn);
        Check("gravity turn slot is not available", gravity != null && !gravity.Available && gravity.UnavailableReason == CommitResolver.GravityTurnUnavailable);
        Check("gravity turn is never live", gravity != null && !gravity.IsLive(new CraftActivity { AscentRunning = true }));
        Check("ascent lives on LAUNCH", GuidanceModes.AnyLiveOn(ConsolePage.Launch, new CraftActivity { AscentRunning = true }));
        Check("ascent is not on LAND", !GuidanceModes.AnyLiveOn(ConsolePage.Land, new CraftActivity { AscentRunning = true }));
        Check("boostback lives on LAND", GuidanceModes.AnyLiveOn(ConsolePage.Land, new CraftActivity { BoostbackLive = true }));
        Check("live mode of a hover", GuidanceModes.Live(new CraftActivity { HoverLive = true, DescentLive = true })?.Id == GuidanceModeId.Hover);
        Check("no live mode when idle", GuidanceModes.Live(new CraftActivity()) == null);
        int previous = int.MinValue;
        foreach (GuidanceModeDescriptor mode in GuidanceModes.ByStopOrder)
        {
            Check("stop order rises " + mode.Id, mode.StopOrder > previous);
            previous = mode.StopOrder;
            foreach (string phase in mode.Phases)
                Check("phase is upper case " + phase, phase == phase.ToUpperInvariant());
        }
        Check("ascent phases use GRAVITY TURN", Array.IndexOf(GuidanceModes.AscentPhases, "GRAVITY TURN") >= 0
            && Array.IndexOf(GuidanceModes.AscentPhases, "SIMPLE TURN") < 0);

        Func<string, float> chars = text => text.Length;
        string chain = GuidanceModes.PhaseChain(GuidanceModes.BoostbackPhases, "BURN", chars, 100f);
        Check("phase chain middle marks what it leaves out", chain == ".. SLEW > [ BURN ] > GLIDE ..", chain);
        chain = GuidanceModes.PhaseChain(GuidanceModes.BoostbackPhases, "SETTLE", chars, 100f);
        Check("phase chain start", chain == "[ SETTLE ] > SLEW ..", chain);
        chain = GuidanceModes.PhaseChain(GuidanceModes.BoostbackPhases, "ENDED", chars, 100f);
        Check("phase chain end", chain == ".. GLIDE > [ ENDED ]", chain);
        chain = GuidanceModes.PhaseChain(GuidanceModes.BoostbackPhases, "SOMETHING ELSE", chars, 100f);
        Check("an unknown phase shows alone", chain == "[ SOMETHING ELSE ]", chain);
        chain = GuidanceModes.PhaseChain(GuidanceModes.AscentPhases, "UPFG (BLENDING)", chars, 100f);
        Check("a phase with a note keeps its neighbours", chain.Contains("SOLVED CLIMB > [ UPFG (BLENDING) ] > TERMINAL"), chain);
        chain = GuidanceModes.PhaseChain(GuidanceModes.BoostbackPhases, "BURN", chars, 22f);
        Check("a narrow chain drops the phase before first", chain == ".. [ BURN ] > GLIDE ..", chain);
        chain = GuidanceModes.PhaseChain(GuidanceModes.BoostbackPhases, "BURN", chars, 12f);
        Check("a narrower chain keeps the current phase", chain == ".. [ BURN ] ..", chain);
        Check("no arrow stands without a step", !chain.StartsWith(">", StringComparison.Ordinal) && !chain.EndsWith(">", StringComparison.Ordinal));
        IReadOnlyList<string> deorbit = GuidanceModes.DeorbitStepsFor(true, "G-FOLD", true);
        Check("a deorbit chain is the chain of the page", string.Join(" ", deorbit) == "PLANNING WAIT DEORBIT BURN COAST BRAKING BURN G-FOLD HOVER");
        Check("a direct approach has no stock burn", !GuidanceModes.DeorbitStepsFor(false, "TOUCHDOWN", true).Contains(GuidanceModes.DeorbitBurnStep));
        Check("touchdown has no hover after it", !GuidanceModes.DeorbitStepsFor(true, "TOUCHDOWN", true).Contains("HOVER"));
        Check("a craft that cannot hover has no hover step", !GuidanceModes.DeorbitStepsFor(true, "G-FOLD", false).Contains("HOVER"));
        Check("while the burn waits the bracket stands on the wait", GuidanceModes.DeorbitStepOf("DEORBIT BURN", burnWaits: true) == GuidanceModes.WaitStep);
        Check("in the burn it stands on the burn", GuidanceModes.DeorbitStepOf("DEORBIT BURN", burnWaits: false) == GuidanceModes.DeorbitBurnStep);
        foreach (string label in new[] { "PLANNING", "DEORBIT COAST", "NODE PENDING", "BRAKING PLAN", "COAST", "COAST TO BRAKING", "BRAKE READY", "BRAKING BURN" })
            Check("each phase of a deorbit has its step: " + label, GuidanceModes.PhaseIndex(deorbit, GuidanceModes.DeorbitStepOf(label, false)) >= 0);
        List<LiveStep> wait = GuidanceModes.LiveChain(deorbit, GuidanceModes.WaitStep, chars, 100f);
        Check("the stock burn is stock", wait.Exists(step => step.Text == GuidanceModes.DeorbitBurnStep && step.Who == StepWho.Stock && step.Place == StepPlace.Next));
        Check("the wait is stock and it is the present step", wait.Exists(step => step.Text == "[ WAIT ]" && step.Who == StepWho.Stock && step.Place == StepPlace.Now));
        Check("the stock burn has its chip", wait.Exists(step => GuidanceModes.HasStockChip(in step)) && !wait.Exists(step => step.Text == "[ WAIT ]" && GuidanceModes.HasStockChip(in step)));
        List<LiveStep> coast = GuidanceModes.LiveChain(deorbit, "COAST", chars, 100f);
        Check("after the burn the present step is AFC work", coast.Exists(step => step.Text == "[ COAST ]" && step.Who == StepWho.Afc));
        Check("a mode with no control yet has a present step of the pilot", GuidanceModes.LiveChain(GuidanceModes.AscentPhasesFor(true), "ARMED", chars, 100f, pilotNow: true)[0] is { Who: StepWho.Pilot, Place: StepPlace.Now });
        Check("the chip takes room in the chain", GuidanceModes.PhaseChain(deorbit, GuidanceModes.WaitStep, chars, 100f).Contains("DEORBIT BURN") && !GuidanceModes.LiveChain(deorbit, GuidanceModes.WaitStep, chars, 16f + 12f, chipRoom: 60f).Exists(step => step.Text == GuidanceModes.DeorbitBurnStep));
        Check("Q-alpha stands clear of every hue of the stage bar", ColorRules.Stages.All(stage => ColorRules.Distance(ColorRules.QAlpha, stage) > 90.0));
        Check("Q-alpha stands clear of the plan and of gold", ColorRules.Distance(ColorRules.QAlpha, ColorRules.Afc) > 90.0 && ColorRules.Distance(ColorRules.QAlpha, ColorRules.Gold) > 90.0);
        Check("the coast before the landing burn stands in the chain only while it flies", !GuidanceModes.GfoldPhasesFor(false).Contains("COAST TO LANDING BURN") && GuidanceModes.GfoldPhasesFor(false, coastNow: true).Contains("COAST TO LANDING BURN"));
        Check("the chain of G-FOLD says what the plot says", GuidanceModes.PhaseChain(GuidanceModes.GfoldPhasesFor(false), "G-FOLD", chars, 100f).StartsWith("[ G-FOLD ] > LANDING BURN"));
        Check("the card of the chain has the count", PageText.StepOf(3, 8) == "STEP 3/8");
        Check("an RCS burn has more than one step", GuidanceModes.RcsPhasesFor(false).Count > 1 && GuidanceModes.RcsPhasesFor(true).Contains("ALIGNING"));
        var jump = new PlanJump();
        Check("a time to go that counts down is no new plan", !jump.Update(65.0, 100.0) && !jump.Update(64.0, 101.0));
        Check("a rise is a new plan", jump.Update(101.0, 102.0));
        Check("the word stands for its time", jump.Update(100.0, 103.0) && !jump.Update(99.0, 102.0 + PlanJump.ShowSeconds));
        Check("a chain has no number that reads as a value", !GuidanceModes.PhaseChain(GuidanceModes.BoostbackPhases, "BURN", chars, 100f).Contains('+'));
        Check("a climb on a plan names no pitch program", GuidanceModes.AscentPhasesFor(true).Contains("SOLVED CLIMB") && !GuidanceModes.AscentPhasesFor(true).Contains("GRAVITY TURN") && !GuidanceModes.AscentPhasesFor(true).Contains("VERTICAL"));
        Check("a pitch program names no plan", GuidanceModes.AscentPhasesFor(false).Contains("GRAVITY TURN") && !GuidanceModes.AscentPhasesFor(false).Contains("SOLVED CLIMB") && !GuidanceModes.AscentPhasesFor(false).Contains("PLANNING"));
        Check("the chain of a plan goes from the plan to UPFG", GuidanceModes.PhaseChain(GuidanceModes.AscentPhasesFor(true), "SOLVED CLIMB", chars, 100f).Contains("PLANNING > [ SOLVED CLIMB ] > UPFG"));

        Check("a whole stack keeps its numbers", StageNumbers.Of([1, 2, 3, 4], [1, 2, 3, 4]).SequenceEqual(new[] { 0, 1, 2, 3 }));
        int[] flownStack = [1, 1, 2, 5];
        Check("on the pad a key is the plan name", StageNumbers.KeyOf(flownStack, flownStack, StageNumbers.Of(flownStack, flownStack), 1) == "SEQ 1b" && StageNumbers.KeyOf(flownStack, flownStack, StageNumbers.Of(flownStack, flownStack), 3) == "SEQ 5");
        int[] lastLeft = [1];
        Check("the stage that burns says both names", StageNumbers.KeyOf(flownStack, lastLeft, StageNumbers.Of(flownStack, lastLeft), 3) == "SEQ 5, NOW 1");
        Check("a stage that has gone says so", StageNumbers.KeyOf(flownStack, lastLeft, StageNumbers.Of(flownStack, lastLeft), 1) == StageNumbers.DroppedWord && StageNumbers.KeyOf(flownStack, lastLeft, StageNumbers.Of(flownStack, lastLeft), 2) == StageNumbers.DroppedWord);
        int[] twoLeft = [1, 4];
        Check("a stage that waits keeps its plan name", StageNumbers.KeyOf(flownStack, twoLeft, StageNumbers.Of(flownStack, twoLeft), 3) == "SEQ 5");
        Check("the first of two that are left says both", StageNumbers.KeyOf(flownStack, twoLeft, StageNumbers.Of(flownStack, twoLeft), 2) == "SEQ 2, NOW 1");
        int[] splitLeft = [1, 2, 5];
        Check("the rest of a split stage has not gone", StageNumbers.KeyOf(flownStack, splitLeft, StageNumbers.Of(flownStack, splitLeft), 1) == "SEQ 1b");
        Check("a stage keeps its number when the stages under it have gone", StageNumbers.Of([1, 2, 3, 4], [3, 4]).SequenceEqual(new[] { 2, 3 }));
        Check("the last stage keeps its number", StageNumbers.Of([1, 2, 3, 4], [4]).SequenceEqual(new[] { 3 }));
        Check("the rows of a split stage share a number", StageNumbers.Of([1, 2, 3], [2, 2, 3]).SequenceEqual(new[] { 1, 1, 2 }));
        Check("the numbers of stock move down when spent sequences go, and the last stage stays the last", StageNumbers.Of([1, 2, 3, 4], [1]).SequenceEqual(new[] { 3 }) && StageNumbers.Of([1, 2, 3, 4], [1, 2]).SequenceEqual(new[] { 2, 3 }));
        Check("the rows of a split stage share a number at the end of a stack", StageNumbers.Of([1, 2], [1, 1]).SequenceEqual(new[] { 1, 1 }));
        Check("a split in the stack keeps its rows", StageNumbers.Of([1, 1, 2], [1, 1, 2]).SequenceEqual(new[] { 0, 1, 2 }));
        Check("more stages than the stack count from the first number", StageNumbers.Of([1, 2], [1, 2, 3]).SequenceEqual(new[] { 0, 1, 2 }));
        Check("what is left fits a longer stack", StageNumbers.Fits([1, 2, 3], [1]) && !StageNumbers.Fits([1], [1, 2]));
        Check("a stack with no sequence starts at the first number", StageNumbers.Of([], [5, 6]).SequenceEqual(new[] { 0, 1 }));
        Check("the fallback stage of live engines is one stage", StageNumbers.Of([-1], [-1]).SequenceEqual(new[] { 0 }));
        Check("a phase with a note has the index of its entry",
            GuidanceModes.PhaseIndex(GuidanceModes.AscentPhases, "UPFG (BLENDING)") == Array.IndexOf(GuidanceModes.AscentPhases, "UPFG"));

        // A later mode registers and the stop finds it without a change of the resolver.
        var later = new GuidanceModeDescriptor(GuidanceModeId.AscentGravityTurn, ConsolePage.Launch, "GRAVITY TURN", 65,
            a => a.AscentRunning, _ => StopTarget.Ascent, new[] { "LANDED", "INITIAL", "PITCH" });
        GuidanceModes.Register(later);
        Check("a registered mode is found", GuidanceModes.Of(GuidanceModeId.AscentGravityTurn)?.Available == true);
        Check("a registered mode keeps the stop order", GuidanceModes.ByStopOrder[^2].Id == GuidanceModeId.AscentGravityTurn
            || GuidanceModes.ByStopOrder[^1].Id == GuidanceModeId.AscentGravityTurn);
        GuidanceModes.Register(new GuidanceModeDescriptor(GuidanceModeId.AscentGravityTurn, ConsolePage.Launch, "GRAVITY TURN", 70,
            _ => false, _ => StopTarget.Ascent, Array.Empty<string>(), Available: false, UnavailableReason: GuidanceModes.GravityTurnUnavailable));
        Check("registering again replaces", GuidanceModes.Of(GuidanceModeId.AscentGravityTurn)?.Available == false);

        RegisteredModeRules();
        EntryGuidanceRules();
        GuidanceModes.ResetToBuiltIns();
        Check("reset brings back the unavailable slot", GuidanceModes.Of(GuidanceModeId.AscentGravityTurn)?.Available == false
            && GuidanceModes.ByStopOrder.Length == Enum.GetValues<GuidanceModeId>().Length - 1);
        Check("reset drops the registered entry guidance until it registers again", GuidanceModes.Of(GuidanceModeId.EntryHere) == null);
    }

    // Entry guidance through the registry: it outranks the boostback whose glide it takes over, touchdown guidance outranks it when it takes the burn, and the stop says what Disengage3Dof does to the engine in each stage.
    private static void EntryGuidanceRules()
    {
        GuidanceModes.Register(EntryGuidance.Descriptor());
        var open = new CommitGates(false, false, "", "", false);
        CraftActivity Entry(EntryStage stage, bool control = true)
        {
            (string outcome, string detail) = EntryGuidance.StopText(stage, control);
            return new CraftActivity
            {
                RegisteredModeLive = GuidanceModeId.EntryHere, ControlAcquired = control,
                RegisteredStopOutcome = outcome, RegisteredStopDetail = detail,
            };
        }

        CraftActivity glide = Entry(EntryStage.Glide);
        CraftActivity burn = Entry(EntryStage.Burn);
        Check("entry guidance stops as Registered", StopResolver.Resolve(glide) == StopTarget.Registered);
        Check("entry stop label", StopResolver.Label(StopTarget.Registered, glide) == "STOP ENTRY");
        Check("entry in the glide has nothing to cut", StopResolver.OutcomeShort(StopTarget.Registered, glide) == "ENGINE OFF");
        Check("entry in the burn keeps the engine", StopResolver.OutcomeShort(StopTarget.Registered, burn) == "ENGINE LIT");
        Check("the card of an entry stop says who flies", StopResolver.EngineOutcome(StopTarget.Registered, burn) == "Engine stays lit at the last throttle. AFC stops steering."
            && StopResolver.EngineOutcome(StopTarget.Registered, glide) == "Engine stays off. AFC stops steering.");
        Check("entry request only cancels", StopResolver.OutcomeShort(StopTarget.Registered, Entry(EntryStage.Requested)) == "NOTHING STARTED, ENGINE UNCHANGED");
        Check("entry without control leaves the engine",
            StopResolver.OutcomeShort(StopTarget.Registered, Entry(EntryStage.Glide, control: false)) == "ENGINE UNCHANGED");
        Check("entry without its own texts falls back to the descriptor",
            StopResolver.OutcomeShort(StopTarget.Registered, glide with { RegisteredStopOutcome = null }) == "ENGINE LIT IN BURN, ELSE CUT");
        Check("a line of entry guidance fits the narrow head", StopResolver.OutcomeLine(StopTarget.Registered, Entry(EntryStage.Requested)).Length <= 46
            && StopResolver.OutcomeLine(StopTarget.Registered, glide with { RegisteredStopOutcome = null }).Length <= 46);
        Check("the descriptor fallback holds in the burn too",
            StopResolver.EngineOutcome(StopTarget.Registered, burn with { RegisteredStopDetail = null }).Contains("the engine stays lit"));
        Check("entry outranks the boostback it took over", StopResolver.Resolve(glide with { BoostbackLive = true }) == StopTarget.Registered);
        Check("touchdown guidance outranks entry", StopResolver.Resolve(burn with { SixDofEngaged = true }) == StopTarget.SixDof);
        Check("entry is guidance that flies", glide.AnyGuidanceLive && !StopResolver.NothingToRelease(glide));
        Check("entry lives on LAND", GuidanceModes.AnyLiveOn(ConsolePage.Land, glide)
            && ConsolePages.PageOf(StopTarget.Registered, false, GuidanceModeId.EntryHere) == ConsolePage.Land);
        Check("entry phases name the hand-over",
            GuidanceModes.PhaseChain(EntryGuidance.Phases, "LANDING BURN", text => text.Length, 100f).Contains("GLIDE > [ LANDING BURN ] > TOUCHDOWN"));

        CommitView v = CommitResolver.Resolve(GuidanceModeId.EntryHere, new CraftActivity(), open);
        Check("entry commit verb", v.Enabled && v.Verb == "ENTRY" && !v.NeedsTakeOver);
        v = CommitResolver.Resolve(GuidanceModeId.EntryHere, glide, open);
        Check("entry refuses while it flies", !v.Enabled && v.Live && v.Reason == "AFC: ENTRY GUIDANCE");
        v = CommitResolver.Resolve(GuidanceModeId.EntryHere, new CraftActivity { BoostbackLive = true }, open);
        Check("entry over a boostback asks first", v.Enabled && v.NeedsTakeOver);
        v = CommitResolver.Resolve(GuidanceModeId.EntryHere, new CraftActivity { SixDofEngaged = true }, open);
        Check("entry over touchdown guidance asks first", v.Enabled && v.NeedsTakeOver);
        Check("take-over names entry guidance",
            CommitResolver.TakeOverMessage(StopTarget.Registered, "LAND NOW", GuidanceModeId.EntryHere)
            == "LAND NOW ENDS ENTRY GUIDANCE. CONTINUE?");
        Check("take-over names touchdown guidance",
            CommitResolver.TakeOverMessage(StopTarget.SixDof, "G-FOLD")
            == "G-FOLD ENDS TOUCHDOWN. CONTINUE?");
        Check("entry commit hint names the engine", EntryGuidance.CommitHint.Contains("engine off"));
    }

    private static void LandFormRules()
    {
        static bool Ascii(string t) => t.All(ch => ch < 128);
        static bool NoPromise(string t)
        {
            string u = " " + t.ToUpperInvariant() + " ";
            return !u.Contains(" OK") && !u.Contains("READY") && !u.Contains("POSSIBLE") && !u.Contains("WILL WORK");
        }

        Check("the table has three columns", LandSolvers.TableHeads.Length == 3);
        foreach (LandSolver s in Enum.GetValues<LandSolver>())
        {
            string[] row = LandSolvers.TableRow(s, 10.0);
            Check("a table row has a cell per head " + s, row.Length == LandSolvers.TableHeads.Length && row.All(c => c.Length > 0 && Ascii(c)));
            Check("the method name stays visible " + s, LandSolvers.How(s).Contains(LandSolvers.Label(s)));
            Check("the row fits the narrow width " + s, row.Sum(c => c.Length) <= 49, string.Join(" | ", row));
            Check("the card says what the solver suits " + s, LandSolvers.Card(s, 10.0, false, 4.0).Contains("Use: " + LandSolvers.SuitsSentence(s)));
            Check("the rule of a solver is in its long form " + s, Ascii(LandSolvers.SuitsSentence(s)) && LandSolvers.SuitsSentence(s).Length > LandSolvers.Suits(s).Length);
        }
        Check("G-FOLD ends at the live hover handoff", LandSolvers.Ends(LandSolver.Gfold, 25.0) == "HOVER FROM 25 m");
        Check("touchdown and entry end on the ground", LandSolvers.Ends(LandSolver.Touchdown, 25.0) == "LANDED" && LandSolvers.Ends(LandSolver.Entry, 25.0) == "LANDED");
        Check("G-FOLD with no hover ends on the ground", LandSolvers.Ends(LandSolver.Gfold, 25.0, hoverFlies: false) == "LANDED");
        Check("entry with no air ends in an impact", LandSolvers.Ends(LandSolver.Entry, 25.0, hasAir: false) == "IMPACT");
        Check("the suits words follow the rule", LandSolvers.Suits(LandSolver.Gfold).Contains("NO AIR") && LandSolvers.Suits(LandSolver.Touchdown) == "ANY BODY" && LandSolvers.Suits(LandSolver.Entry).Contains("AIR"));

        var airless = new DescentFacts(true, 1e6, 1e6, 2.5e5, 1.62, 0.0, 0.1, 0.1, 0.4, 0.4, 0.0, 15.0, 0.0, 0.0);
        var noAirHere = new LandNowFacts(LandWay.FromHere, LandSolver.Touchdown, false, false, false, airless);
        Check("no air stands on the row of entry and it stops that row", LandSolvers.NoteOf(LandSolver.Entry, in noAirHere) == new RowNote("NEEDS AIR", true));
        Check("the fact of entry stands on no other row", LandSolvers.NoteOf(LandSolver.Touchdown, in noAirHere).Text.Length == 0 && LandSolvers.NoteOf(LandSolver.Gfold, in noAirHere).Text.Length == 0);
        var airOfEarth = new LandNowFacts(LandWay.FromHere, LandSolver.Gfold, true, false, false, Body: "Earth");
        Check("air stands on the row of G-FOLD with the name of the body", LandSolvers.NoteOf(LandSolver.Gfold, in airOfEarth) == new RowNote("Earth HAS AIR", true));
        Check("a craft high over the air still says the true thing", !LandSolvers.NoteOf(LandSolver.Gfold, in airOfEarth).Text.Contains("here"));
        var curves = airless with { TwrAtFloor = 2.28, NeedTiltDeg = 64.0, TiltLimitDeg = 89.0, TwrAtCraftMin = 2.28 };
        var curving = noAirHere with { Descent = curves };
        Check("a path that curves stands on its row and stops nothing", LandSolvers.NoteOf(LandSolver.Touchdown, in curving) == new RowNote("TWR >1 AT FLOOR"));
        Check("no hover is no obstacle of the row, its end says it", LandSolvers.NoteOf(LandSolver.Gfold, in curving).Text.Length == 0);
        Check("no gimbal stops the row of touchdown, because G-FOLD needs a gimbal too", LandSolvers.NoteOf(LandSolver.Touchdown, curving with { Descent = curves with { NoGimbal = true } }) == new RowNote("NO GIMBAL", true));
        var over = curves with { NeedTiltDeg = 80.0, TiltLimitDeg = 15.0 };
        Check("an over-powered floor is named in place of the curve", LandSolvers.NoteOf(LandSolver.Touchdown, curving with { Descent = over }) == new RowNote("FLOOR TOO HIGH"));
        var heldRows = new LandNowFacts(LandWay.FromHere, LandSolver.Touchdown, false, true, false);
        Check("a flying deorbit holds its rows on its own way, and the line on top of the form says so", LandSolvers.NoteOf(LandSolver.Touchdown, heldRows with { Way = LandWay.FromOrbit }).Text == LandSolvers.HeldNote && LandSolvers.NoteOf(LandSolver.Gfold, heldRows with { Way = LandWay.FromOrbit }).Text == LandSolvers.HeldNote
            && LandSolvers.HeldLine(in heldRows).Length == 0 && LandSolvers.HeldLine(heldRows with { Way = LandWay.FromOrbit }).Length == 0);
        Check("off its own way the rows keep what they suit, because the verb takes over", LandSolvers.NoteOf(LandSolver.Touchdown, in heldRows).Text.Length == 0 && LandSolvers.NoteOf(LandSolver.Gfold, in heldRows).Text.Length == 0);
        var gfoldHolds = new LandNowFacts(LandWay.FromHere, LandSolver.Gfold, false, false, true);
        Check("a flying G-FOLD holds touchdown", LandSolvers.NoteOf(LandSolver.Touchdown, in gfoldHolds).Text == LandSolvers.HeldNote && LandSolvers.HeldLine(in gfoldHolds) == "TOUCHDOWN HELD: G-FOLD ACTIVE.");
        Check("with nothing held there is no sentence", LandSolvers.HeldLine(in noAirHere).Length == 0);
        Check("DEORBIT with TOUCHDOWN chosen names the G-FOLD fallback", LandSolvers.FallbackLine(heldRows with { Way = LandWay.FromOrbit }) == LandSolvers.TouchdownFallback);
        Check("LAND NOW with TOUCHDOWN chosen names the G-FOLD fallback too", LandSolvers.FallbackLine(in heldRows) == LandSolvers.TouchdownFallback);
        Check("ENTRY, a flying G-FOLD or HOVER has no fallback line", LandSolvers.FallbackLine(heldRows with { Chosen = LandSolver.Entry }).Length == 0
            && LandSolvers.FallbackLine(heldRows with { Way = LandWay.FromOrbit, Chosen = LandSolver.Gfold }).Length == 0 && LandSolvers.FallbackLine(heldRows with { Way = LandWay.Hover }).Length == 0);
        static List<TableMark> Marks(LandSolver solver, int row, bool chosen, RowNote note, bool air)
        {
            var marks = new List<TableMark>();
            LandSolvers.MarksOf(solver, row, chosen, in note, air, marks);
            return marks;
        }
        Check("an obstacle on the chosen row takes the warning colour", Marks(LandSolver.Gfold, 2, true, new RowNote("Earth HAS AIR", true), true) is [{ Row: 2, Column: 1, Ink: TableInk.Warn }]);
        Check("an obstacle on another row takes the dim warning colour", Marks(LandSolver.Gfold, 2, false, new RowNote("Earth HAS AIR", true), true) is [{ Row: 2, Column: 1, Ink: TableInk.WarnDim }]);
        Check("a path that curves has a soft mark on every row, lighter on the band", Marks(LandSolver.Touchdown, 1, true, new RowNote("TWR >1 AT FLOOR"), false) is [{ Column: 1, Ink: TableInk.WarnOnBand }]
            && Marks(LandSolver.Touchdown, 1, false, new RowNote("TWR >1 AT FLOOR"), false) is [{ Column: 1, Ink: TableInk.WarnDim }]);
        Check("a crash on the chosen row takes the danger colour of the band on its word, and its cause the warning colour", Marks(LandSolver.Entry, 0, true, new RowNote("NEEDS AIR", true), false) is [{ Row: 0, Column: 2, Ink: TableInk.DangerOnBand }, { Row: 0, Column: 1, Ink: TableInk.Warn }]);
        Check("a crash on another row stays a crash in a dim colour", Marks(LandSolver.Entry, 0, false, new RowNote("NEEDS AIR", true), false) is [{ Column: 2, Ink: TableInk.DangerDim }, { Column: 1, Ink: TableInk.WarnDim }]);
        Check("a row with no obstacle takes no colour, so a table claims no success", Marks(LandSolver.Touchdown, 1, true, RowNote.None, true).Count == 0 && Marks(LandSolver.Gfold, 2, false, RowNote.None, false).Count == 0);
        Check("a held row takes no warning", Marks(LandSolver.Touchdown, 1, false, new RowNote(LandSolvers.HeldNote), false).Count == 0);
        Check("the chain of a way that cannot fly is dim", LandSolvers.ChainCannotFly(LandWay.FromHere, LandSolver.Gfold, true, CraftSituation.Orbit, false)
            && LandSolvers.ChainCannotFly(LandWay.FromOrbit, LandSolver.Gfold, true, CraftSituation.Ground, false)
            && LandSolvers.ChainCannotFly(LandWay.FromHere, LandSolver.Entry, false, CraftSituation.Orbit, false)
            && LandSolvers.ChainCannotFly(LandWay.Boostback, LandSolver.Entry, true, CraftSituation.Ground, true));
        Check("the chain of a way that can fly stays lit", !LandSolvers.ChainCannotFly(LandWay.FromOrbit, LandSolver.Touchdown, true, CraftSituation.Ground, true)
            && !LandSolvers.ChainCannotFly(LandWay.FromHere, LandSolver.Gfold, false, CraftSituation.FallingNoAir, true)
            && !LandSolvers.ChainCannotFly(LandWay.FromOrbit, LandSolver.Touchdown, false, CraftSituation.Orbit, true));
        Check("a spent booster has its way lit in flight, because the stage list counts later stages only", !LandSolvers.ChainCannotFly(LandWay.Boostback, LandSolver.Entry, true, CraftSituation.FallingAir, true)
            && !LandSolvers.ChainCannotFly(LandWay.Boostback, LandSolver.Entry, true, CraftSituation.CoastingUp, true));
        Check("a way that cannot start from here dims in flight", LandSolvers.ChainCannotFly(LandWay.Boostback, LandSolver.Entry, true, CraftSituation.Orbit, true)
            && LandSolvers.ChainCannotFly(LandWay.FromHere, LandSolver.Touchdown, false, CraftSituation.Orbit, false)
            && LandSolvers.ChainCannotFly(LandWay.FromOrbit, LandSolver.Touchdown, false, CraftSituation.FallingNoAir, false, belowBrakingStart: true));
        Check("a way that flies stays lit where it could not start", !LandSolvers.ChainCannotFly(LandWay.FromHere, LandSolver.Touchdown, false, CraftSituation.Orbit, false, flies: true));
        Check("a row that cannot fly on this body stays dim also while its way flies", LandSolvers.ChainCannotFly(LandWay.FromHere, LandSolver.Entry, false, CraftSituation.FallingNoAir, false, flies: true));
        Check("the method word follows the test of the chain", LandSolvers.MethodMark(1, false, false) == new TableMark(1, 0, TableInk.Method)
            && LandSolvers.MethodMark(1, true, false).Ink == TableInk.MethodDim && LandSolvers.MethodMark(1, false, true).Ink == TableInk.MethodDim);
        Check("the cause of a crash in the chain warns", LandChain.For(LandWay.FromHere, LandSolver.Entry, CraftSituation.Orbit, false, false, false) is [{ Warn: true, Danger: false }, { Danger: true }]);
        foreach (LandWay way in new[] { LandWay.FromOrbit, LandWay.FromHere })
            foreach (LandSolver s in Enum.GetValues<LandSolver>())
                foreach (bool air in new[] { true, false })
                    foreach (DescentFacts d in new[] { default, airless, curves, over })
                    {
                        var rowFacts = new LandNowFacts(way, s, air, false, false, d, "Earth");
                        string now = LandSolvers.NoteOf(s, in rowFacts).Text;
                        Check("a note on a row makes no promise", NoPromise(now) && Ascii(now), now);
                        string[] noted = LandSolvers.TableRow(s, 10.0, LandSolvers.HoverFlies(in d), air, now);
                        Check("a row with its note fits the narrow width", noted.Sum(c => c.Length) <= 49, string.Join(" | ", noted));
                    }

        ChainStep[] chain = LandChain.For(LandWay.FromOrbit, LandSolver.Touchdown, CraftSituation.Orbit, false, true, true);
        Check("the deorbit chain starts with the stock burn", chain[0] == new ChainStep(LandChain.StockBurn, true) && chain.Skip(1).All(c => !c.Stock));
        Check("the deorbit chain of touchdown", string.Join(" > ", chain.Select(c => c.Text)) == "DEORBIT BURN > COAST > BRAKING BURN > TOUCHDOWN");
        chain = LandChain.For(LandWay.FromOrbit, LandSolver.Gfold, CraftSituation.Orbit, false, true, true);
        Check("the deorbit chain of G-FOLD has the hover", string.Join(" > ", chain.Select(c => c.Text)) == "DEORBIT BURN > COAST > BRAKING BURN > G-FOLD > HOVER");
        chain = LandChain.For(LandWay.FromOrbit, LandSolver.Gfold, CraftSituation.FallingNoAir, false, true, true);
        Check("a craft that falls may need no stock burn", chain[0] == new ChainStep(LandChain.StockBurnIfNeeded, true));
        chain = LandChain.For(LandWay.FromHere, LandSolver.Touchdown, CraftSituation.FallingNoAir, false, true, true);
        Check("touchdown from here is one step", string.Join(" > ", chain.Select(c => c.Text)) == "TOUCHDOWN" && chain.All(c => !c.Stock));
        chain = LandChain.For(LandWay.FromHere, LandSolver.Entry, CraftSituation.FallingAir, true, true, true);
        Check("entry hands its burn to touchdown", string.Join(" > ", chain.Select(c => c.Text)) == "GLIDE > LANDING BURN > TOUCHDOWN");
        chain = LandChain.For(LandWay.FromHere, LandSolver.Entry, CraftSituation.FallingAir, true, true, false);
        Check("entry can fly its own burn to the ground", string.Join(" > ", chain.Select(c => c.Text)) == "GLIDE > LANDING BURN");
        chain = LandChain.For(LandWay.FromHere, LandSolver.Entry, CraftSituation.FallingNoAir, false, true, true);
        Check("entry with no air has no plan", chain[^1] == new ChainStep("IMPACT", Danger: true));
        chain = LandChain.For(LandWay.Boostback, LandSolver.Entry, CraftSituation.CoastingUp, true, true, true);
        Check("a boostback glide goes to entry", string.Join(" > ", chain.Select(c => c.Text)) == "SETTLE > SLEW > BURN > GLIDE > ENTRY > LANDING BURN > TOUCHDOWN");
        chain = LandChain.For(LandWay.Boostback, LandSolver.Entry, CraftSituation.CoastingUp, true, false, true);
        Check("a boostback alone ends in the impact", string.Join(" > ", chain.Select(c => c.Text)) == "SETTLE > SLEW > BURN > GLIDE > IMPACT" && chain[^1].Danger);
        chain = LandChain.For(LandWay.Boostback, LandSolver.Entry, CraftSituation.CoastingUp, false, true, true);
        Check("a boostback with no air has no glide", string.Join(" > ", chain.Select(c => c.Text)) == "SETTLE > SLEW > BURN > FALL > IMPACT" && chain[^1].Danger);
        chain = LandChain.For(LandWay.Hover, LandSolver.Touchdown, CraftSituation.Ground, true, true, true);
        Check("a hover has no chain", chain.Length == 0);
        chain = LandChain.For(LandWay.FromOrbit, LandSolver.Gfold, CraftSituation.FallingNoAir, false, true, true, belowBrakingStart: true);
        Check("below the braking start only the steps left", string.Join(" > ", chain.Select(c => c.Text)) == "G-FOLD > HOVER" && chain.All(c => !c.Stock));
        chain = LandChain.For(LandWay.FromHere, LandSolver.Gfold, CraftSituation.FallingNoAir, false, true, true, hoverFlies: false);
        Check("a craft that cannot hover has no hover step", string.Join(" > ", chain.Select(c => c.Text)) == "G-FOLD");
        chain = LandChain.For(LandWay.FromOrbit, LandSolver.Gfold, CraftSituation.Orbit, false, true, true, hoverFlies: false);
        Check("also from orbit", string.Join(" > ", chain.Select(c => c.Text)) == "DEORBIT BURN > COAST > BRAKING BURN > G-FOLD");

        // The closed loop figures in the comment of GuidanceWindow.ReadDescentFacts: 300 m up at 50 m/s down.
        var twr2 = new DescentFacts(true, 2.0, 2.0, 1.0, 9.81, 0.0, 0.1, 0.1, 0.2, 0.2, 0.0, 15.0, 50.0, 300.0, SinkMs: 50.0, HeightM: 300.0);
        Check("TWR 2 stops in 127 m", Math.Abs(twr2.StopM - 127.4) < 0.5 && !twr2.CannotStop && !twr2.Marginal, twr2.StopM.ToString(Fmt.Inv));
        var twr125 = twr2 with { ThrustN = 1.25 };
        Check("TWR 1.25 needs 510 m of 300 m", Math.Abs(twr125.StopM - 509.7) < 0.5 && twr125.CannotStop, twr125.StopM.ToString(Fmt.Inv));
        var twrLow = twr2 with { ThrustN = 0.9 };
        Check("under the weight it never stops", double.IsInfinity(twrLow.StopM) && twrLow.CannotStop);
        Check("a stop close to the height left is marginal", (twr2 with { AltToGoM = 150.0 }).Marginal);
        Check("no sink means no stop check", !(twr2 with { DescentMs = 0.5 }).Descending);
        Check("the feasible floor follows the tilt limit", Math.Abs((twr2 with { TiltLimitDeg = 60.0 }).FeasibleFloor - 0.25) < 1e-9);

        Check("no thrust is said plainly", LandVerdict.ForDescent(LandSolver.Touchdown, default, true).Text == PageText.NoThrustToJudge);
        LandVerdict v = LandVerdict.ForDescent(LandSolver.Touchdown, twr125, true);
        Check("cannot stop is the worst", v.Severity == Severity.Error && v.Text.StartsWith("NO SOLUTION: TWR", StringComparison.Ordinal), v.Text);
        Check("the stop check can be left out", LandVerdict.ForDescent(LandSolver.Touchdown, twr125, false).Severity == Severity.Info);
        // A craft that climbs toward a far site: its motion reads as a fast descent in the frame of the site.
        DescentFacts climbsToFarSite = twr125 with { SinkMs = -120.0, HeightM = 60000.0 };
        Check("a craft that does not sink has no stop verdict", climbsToFarSite.CannotStop && !climbsToFarSite.CannotStopNow
            && LandVerdict.ForDescent(LandSolver.Touchdown, climbsToFarSite, true).Severity == Severity.Info
            && !LandVerdict.Notes(LandSolver.Touchdown, climbsToFarSite, true).Any(n => n.Text.StartsWith("Cannot stop", StringComparison.Ordinal)));
        DescentFacts climbsTight = twr2 with { AltToGoM = 150.0, SinkMs = -120.0, HeightM = 60000.0 };
        Check("and no marginal one", climbsTight.Marginal && !climbsTight.MarginalNow && !LandVerdict.ForDescent(LandSolver.Touchdown, climbsTight, true).Text.StartsWith("Marginal", StringComparison.Ordinal));
        Check("the two rules agree on one craft that sinks", twr125.CannotStopNow && (twr2 with { AltToGoM = 150.0 }).MarginalNow);
        v = LandVerdict.ForDescent(LandSolver.Touchdown, twr2 with { NeedTiltDeg = 80.0, TiltLimitDeg = 15.0 }, true);
        Check("over-powered names the floor", v.Severity == Severity.Error && v.Text.Contains("FLOOR ABOVE"), v.Text);
        Check("over-powered is a fact of touchdown only", LandVerdict.ForDescent(LandSolver.Gfold, twr2 with { NeedTiltDeg = 80.0, TiltLimitDeg = 15.0 }, true).Severity == Severity.Info);
        Check("thrust under the weight warns", LandVerdict.ForDescent(LandSolver.Gfold, twrLow with { DescentMs = 0.0 }, true).Severity == Severity.Warning);
        v = LandVerdict.ForDescent(LandSolver.Touchdown, twr2 with { AltToGoM = 150.0 }, true);
        Check("marginal warns", v.Severity == Severity.Warning && v.Text.StartsWith("MARGINAL", StringComparison.Ordinal), v.Text);
        v = LandVerdict.ForDescent(LandSolver.Touchdown, twr2, true);
        Check("a plain verdict is its numbers", v == new LandVerdict("TWR 2.00 AT SITE, 0.20 AT MIN THROTTLE."), v.Text);
        Check("G-FOLD reads the minimum of the craft", LandVerdict.ForDescent(LandSolver.Gfold, twr2 with { TwrAtCraftMin = 0.3 }, true).Text.Contains("0.30 AT MIN THROTTLE"));
        Check("the fold value of a powered way is one short number", LandVerdict.TwrAtSite(twr2) == "TWR 2.00 AT SITE" && LandVerdict.TwrAtSite(default).Length == 0);
        v = LandVerdict.ForImpact(true, true, 175.0, 428.8, false, 3600.0, 0.0, "");
        Check("an impact says when and how fast", v.Text == "IMPACT T-2m 55s AT 428.8 m/s" && v.Severity == Severity.Info, v.Text);
        Check("no surrogate warns", LandVerdict.ForImpact(false, false, 0.0, 0.0, false, 3600.0, 0.0, "").Severity == Severity.Warning);
        v = LandVerdict.ForImpact(true, false, 0.0, 0.0, true, 3600.0, 200000.0, "");
        Check("an orbit has its number, and the footer has the sentence", v.Text == "LOWEST POINT 200.0 km", v.Text);
        Check("another status of the prediction is shown", LandVerdict.ForImpact(true, false, 0.0, 0.0, false, 3600.0, 0.0, "Failed").Text == "Failed");
        foreach (LandVerdict text in new[]
                 {
                     LandVerdict.ForDescent(LandSolver.Touchdown, twr125, true),
                     LandVerdict.ForDescent(LandSolver.Touchdown, twr2 with { NeedTiltDeg = 80.0, TiltLimitDeg = 15.0 }, true),
                     LandVerdict.ForDescent(LandSolver.Touchdown, twr2 with { AltToGoM = 150.0 }, true),
                     LandVerdict.ForDescent(LandSolver.Touchdown, twr2 with { ThrustN = 22.82, TwrAtFloor = 2.28 }, true),
                     LandVerdict.ForDescent(LandSolver.Gfold, twrLow with { DescentMs = 0.0 }, true),
                     LandVerdict.ForImpact(true, true, 175.0, 428.8, false, 3600.0, 0.0, ""),
                 })
        {
            Check("the verdict makes no promise", NoPromise(text.Text) && Ascii(text.Text), text.Text);
            Check("the verdict fits its fold row", text.Text.Length <= 50, text.Text);
        }
        Check("the notes say what to do", LandVerdict.Notes(LandSolver.Touchdown, twr125, true).First().Text.Contains("BURN EARLIER"));
        Check("no thrust has no notes", !LandVerdict.Notes(LandSolver.Touchdown, default, true).Any());
        Check("touchdown says it can hover", LandVerdict.Notes(LandSolver.Touchdown, twr2, true).Single().Text.StartsWith("CAN HOVER", StringComparison.Ordinal));
        Check("G-FOLD with too much thrust has its one sentence in the footer", !LandVerdict.Notes(LandSolver.Gfold, twr2 with { TwrAtCraftMin = 1.5 }, true).Any());
        Check("entry has no note of its own", !LandVerdict.Notes(LandSolver.Entry, twr2, false).Any());

        Check("the hand-overs say their values", PageText.HandOversLabel(true, 45.0, true, 5.0) == "HANDOVER: ENTRY AT IMPACT T-45 s, TOUCHDOWN LAST 5 s");
        Check("the hand-overs say when they are off", PageText.HandOversLabel(false, 45.0, false, 5.0) == "HANDOVER: ENTRY OFF, TOUCHDOWN OFF");
        Check("the pick button keeps its size", PageText.PickSite.Length <= PageText.PickArmed.Length);

        Span<float> columns = stackalloc float[3];
        DockLayout.TableColumns(new float[] { 100f, 80f, 60f }, 300f, columns);
        Check("columns that fit share the rest", columns[0] == 120f && columns[1] == 100f && columns[2] == 80f);
        DockLayout.TableColumns(new float[] { 200f, 100f, 100f }, 200f, columns);
        Check("columns that do not fit share the room in proportion", columns[0] == 100f && columns[1] == 50f && columns[2] == 50f);
        DockLayout.TableColumns(new float[] { 0f, 0f }, 100f, columns);
        Check("empty columns share equally", columns[0] == 50f && columns[1] == 50f);
    }

    // The solver choice of LAND and the hand-over follow: the page, its commit and its stop act on the mode that flies.
    private static void LandSolverRules()
    {
        Check("entry is chosen first", LandSolvers.Chosen(true, true) == LandSolver.Entry && LandSolvers.Chosen(true, false) == LandSolver.Entry);
        Check("touchdown chosen", LandSolvers.Chosen(false, true) == LandSolver.Touchdown);
        Check("G-FOLD chosen", LandSolvers.Chosen(false, false) == LandSolver.Gfold);
        Check("entry keeps the braking choice of touchdown", LandSolvers.Apply(LandSolver.Entry, true) == (true, true));
        Check("entry keeps the braking choice of G-FOLD", LandSolvers.Apply(LandSolver.Entry, false) == (true, false));
        Check("touchdown clears entry", LandSolvers.Apply(LandSolver.Touchdown, false) == (false, true));
        Check("a pick of G-FOLD gives touchdown, because the player cannot choose G-FOLD", LandSolvers.Apply(LandSolver.Gfold, false) == (false, true));
        Check("entry is not held by a deorbit", !LandSolvers.LockedByDeorbit(LandSolver.Entry));
        Check("touchdown and G-FOLD are held by a deorbit",
            LandSolvers.LockedByDeorbit(LandSolver.Touchdown) && LandSolvers.LockedByDeorbit(LandSolver.Gfold));
        Check("one SOLVER row in solver order", LandSolvers.Labels.SequenceEqual(new[] { "ENTRY", "TOUCHDOWN", "G-FOLD" })
            && Enum.GetValues<LandSolver>().All(s => LandSolvers.Labels[(int)s] == LandSolvers.Label(s)));
        foreach (LandSolver s in Enum.GetValues<LandSolver>())
        {
            string d = LandSolvers.Description(s);
            Check("solver description " + s, d.Length > 0 && d.Length <= 90 && d.All(ch => ch < 128), d);
        }
        string[] deorbit = LandSolvers.Held(LandWay.FromOrbit, false, false, true);
        Check("deorbit holds ENTRY only", deorbit[(int)LandSolver.Entry] == LandSolvers.HeldDeorbitEntry
            && deorbit[(int)LandSolver.Touchdown] == "" && deorbit[(int)LandSolver.Gfold] == "");
        string[] deorbitFlies = LandSolvers.Held(LandWay.FromOrbit, true, false, true);
        Check("a flying deorbit holds every solver", deorbitFlies.All(h => h.Length > 0));
        string[] here = LandSolvers.Held(LandWay.FromHere, false, false, true);
        Check("land now offers all three", here.All(h => h.Length == 0));
        string[] hereLocked = LandSolvers.Held(LandWay.FromHere, true, true, true);
        Check("a flying deorbit holds touchdown and G-FOLD on land now", hereLocked[(int)LandSolver.Entry] == ""
            && hereLocked[(int)LandSolver.Touchdown] == LandSolvers.HeldWhileDeorbit && hereLocked[(int)LandSolver.Gfold] == LandSolvers.HeldWhileDeorbit);
        string[] hereDescent = LandSolvers.Held(LandWay.FromHere, false, true, false);
        Check("a card names ENTRY as a take-over only where ENTRY can plan", !hereDescent[(int)LandSolver.Touchdown].Contains("ENTRY")
            && LandSolvers.Held(LandWay.FromHere, false, true, true)[(int)LandSolver.Touchdown] == LandSolvers.HeldWhileGfold + LandSolvers.EntryTakesOver);
        Check("a flying G-FOLD descent holds touchdown only", hereDescent[(int)LandSolver.Touchdown] == LandSolvers.HeldWhileGfold
            && hereDescent[(int)LandSolver.Entry] == "" && hereDescent[(int)LandSolver.Gfold] == "");
        Check("every orbit tool has a description", OrbitToolCatalog.ToolDescriptions.Length == OrbitToolCatalog.ToolLabels.Length
            && OrbitToolCatalog.ToolDescriptions.All(d => d.Length > 0));

        Check("touchdown flies first", LandSolvers.Live(true, true, true, true, true) == LiveLanding.Touchdown);
        Check("entry over the boostback", LandSolvers.Live(false, true, false, false, true) == LiveLanding.Entry);
        Check("hover over G-FOLD", LandSolvers.Live(false, false, true, true, false) == LiveLanding.Hover);
        Check("G-FOLD", LandSolvers.Live(false, false, false, true, false) == LiveLanding.Gfold);
        Check("BOOSTBACK", LandSolvers.Live(false, false, false, false, true) == LiveLanding.Boostback);
        Check("nothing", LandSolvers.Live(false, false, false, false, false) == LiveLanding.None);
        Check("entry moves the choice", LandSolvers.SolverFor(LiveLanding.Entry) == LandSolver.Entry);
        Check("touchdown moves the choice", LandSolvers.SolverFor(LiveLanding.Touchdown) == LandSolver.Touchdown);
        Check("G-FOLD moves the choice", LandSolvers.SolverFor(LiveLanding.Gfold) == LandSolver.Gfold);
        Check("hover leaves the choice", LandSolvers.SolverFor(LiveLanding.Hover) == null);

        string fromHere = ConsolePages.CueTarget(ConsolePage.Land, ConsolePages.EnumKey(LandWay.FromHere));
        StatusLine? cue = LandSolvers.CueFor(LiveLanding.Boostback, LiveLanding.Entry);
        Check("glide to entry raises a cue", cue is StatusLine c1 && c1.Severity == Severity.Cue && c1.CueTarget == fromHere);
        cue = LandSolvers.CueFor(LiveLanding.Entry, LiveLanding.Touchdown);
        Check("entry to touchdown raises its own cue", cue is StatusLine c2 && c2.Text == "HANDOVER: TOUCHDOWN"
            && c2.CueTarget == fromHere);
        Check("no change raises no cue", LandSolvers.CueFor(LiveLanding.Entry, LiveLanding.Entry) == null);
        Check("G-FOLD and hover raise their own cues", LandSolvers.CueFor(LiveLanding.None, LiveLanding.Gfold) == null
            && LandSolvers.CueFor(LiveLanding.Gfold, LiveLanding.Hover) == null);
        Check("an end raises no cue", LandSolvers.CueFor(LiveLanding.Touchdown, LiveLanding.None) == null);
        Check("the head cues a boostback, not the landing follow", LandSolvers.CueFor(LiveLanding.None, LiveLanding.Boostback) == null);
    }

    // A mode that registers through GuidanceModes.Registered: the stop, the commit rules, the page and the head find it with no change of the resolvers.
    private static void RegisteredModeRules()
    {
        GuidanceModes.Register(GuidanceModes.Registered(GuidanceModeId.AscentGravityTurn, ConsolePage.Launch, "GRAVITY TURN", 55,
            new[] { "LANDED", "INITIAL", "PITCH" }, "LAUNCH", "STOP GRAVITY TURN", "ENGINE CUT", "The gravity turn ends. Engine: cut."));
        var flying = new CraftActivity { RegisteredModeLive = GuidanceModeId.AscentGravityTurn, ControlAcquired = true };
        var idle = new CraftActivity();
        var open = new CommitGates(false, false, "", "", false);

        Check("registered mode is guidance", flying.AnyGuidanceLive && flying.AnyLive);
        Check("registered mode stops as Registered", StopResolver.Resolve(flying) == StopTarget.Registered);
        Check("registered stop is guidance", StopResolver.IsGuidance(StopTarget.Registered));
        Check("registered stop label", StopResolver.Label(StopTarget.Registered, flying) == "STOP GRAVITY TURN");
        Check("registered short outcome", StopResolver.OutcomeShort(StopTarget.Registered, flying) == "ENGINE CUT");
        Check("registered stop detail", StopResolver.EngineOutcome(StopTarget.Registered, flying) == "The gravity turn ends. Engine: cut.");
        Check("built-in labels ignore the activity", StopResolver.Label(StopTarget.Ascent, flying) == "STOP ASCENT");
        Check("a mode with no outcome of its own still has a line", StopResolver.OutcomeLine(StopTarget.Registered, idle) == "ON STOP: MODE ENDS");
        Check("a landing outranks a registered ascent", StopResolver.Resolve(flying with { DescentLive = true }) == StopTarget.Landing);
        Check("a registered ascent outranks the UPFG ascent", StopResolver.Resolve(flying with { AscentRunning = true }) == StopTarget.Registered);
        Check("registered mode lives on LAUNCH", GuidanceModes.AnyLiveOn(ConsolePage.Launch, flying)
            && GuidanceModes.Live(flying)?.Phases.Count == 3);
        Check("registered mode page", ConsolePages.PageOf(StopTarget.Registered, false, GuidanceModeId.AscentGravityTurn) == ConsolePage.Launch);
        Check("registered stop without a mode opens no page", ConsolePages.PageOf(StopTarget.Registered) == null);
        Check("registered mode is the first page while it flies",
            FlightContext.FirstPage(new FlightFacts(false, StopTarget.Registered, false, true, true, false, false, GuidanceModeId.AscentGravityTurn)) == ConsolePage.Launch);

        CommitView v = CommitResolver.Resolve(GuidanceModeId.AscentGravityTurn, idle, open);
        Check("registered mode commits with its verb", v.Enabled && v.Verb == "LAUNCH" && !v.NeedsTakeOver);
        v = CommitResolver.Resolve(GuidanceModeId.AscentGravityTurn, flying, open);
        Check("registered mode refuses while it flies", !v.Enabled && v.Live && v.Reason == "AFC: GRAVITY TURN");
        v = CommitResolver.Resolve(GuidanceModeId.AscentGravityTurn, new CraftActivity { AscentRunning = true }, open);
        Check("registered mode takes over the UPFG ascent", v.Enabled && v.NeedsTakeOver);
        v = CommitResolver.Resolve(GuidanceModeId.AscentUpfg, flying, open);
        Check("UPFG takes over a registered ascent", v.Enabled && v.NeedsTakeOver);
        Check("take-over names the registered mode",
            CommitResolver.TakeOverMessage(StopTarget.Registered, "LAUNCH", GuidanceModeId.AscentGravityTurn)
            == "LAUNCH ENDS GRAVITY TURN MODE. CONTINUE?");
        Check("gravity turn gate still refuses", !CommitResolver.Resolve(GuidanceModeId.AscentGravityTurn, idle,
            open with { GravityTurnRefusal = CommitResolver.GravityTurnUnavailable }).Enabled);
    }

    private static void ComprehensionTexts()
    {
        static bool Ascii(string t) => t.All(ch => ch < 128);
        const string NL = "\n";
        foreach (ConsolePage page in ConsolePages.All)
        {
            Check("chooser hover card " + page, PageText.ChooserCard(page).Length > 0);
            foreach (CraftSituation where in Enum.GetValues<CraftSituation>())
            {
                var tabFacts = new TabFacts(where);
                Check("the card of a tab starts with when to pick it " + page + " " + where, PageText.TabCard(page, in tabFacts).StartsWith(PageText.ChooserCard(page), StringComparison.Ordinal) && Ascii(PageText.TabCard(page, in tabFacts)));
            }
        }
        var onGround = new TabFacts(CraftSituation.Ground);
        var inOrbit = new TabFacts(CraftSituation.Orbit);
        Check("LAUNCH says nothing on the ground, where nothing stands against it", PageText.TabNow(ConsolePage.Launch, in onGround).Length == 0 && PageText.TabNow(ConsolePage.Launch, in inOrbit) == "Works on the ground.");
        Check("MANEUVER needs an orbit", PageText.TabNow(ConsolePage.Orbit, in onGround) == SituationVerbs.NeedsOrbit && PageText.TabNow(ConsolePage.Orbit, in inOrbit).Length == 0);
        Check("TRANSFER reads the rule of its verb", SituationVerbs.ForTransfer(CraftSituation.FallingAir).Fits == (PageText.TabNow(ConsolePage.Transfer, new TabFacts(CraftSituation.FallingAir)).Length == 0)
            && SituationVerbs.ForTransfer(CraftSituation.Orbit).Fits == (PageText.TabNow(ConsolePage.Transfer, in inOrbit).Length == 0)
            && SituationVerbs.ForManeuver(CraftSituation.Orbit).Fits == (PageText.TabNow(ConsolePage.Orbit, in inOrbit).Length == 0));
        Check("BURNS says whether a burn is planned", PageText.TabNow(ConsolePage.Rcs, new TabFacts(CraftSituation.Orbit, BurnPlanned: true)) == "A burn is planned." && PageText.TabNow(ConsolePage.Rcs, in inOrbit) == "No burn planned.");
        Check("STAGING says whether AutoStage is on", PageText.TabNow(ConsolePage.Staging, new TabFacts(CraftSituation.Ground, AutoStageArmed: true)) == "AutoStage is on.");
        Check("a tab says nothing with no craft", PageText.TabNow(ConsolePage.Launch, new TabFacts(CraftSituation.Unknown)).Length == 0);
        Check("the editor keeps its pages true", PageText.TabNow(ConsolePage.Staging, new TabFacts(CraftSituation.Unknown, Editor: true)) == "Works in the editor.");
        foreach (LandWay way in LandWays.DisplayOrder)
            Check("the card of a way " + way, PageText.WayCard(way).Length > 0 && Ascii(PageText.WayCard(way)));
        Check("the card of a way reads the rule of its verb", SituationVerbs.WayNow(LandWay.FromHere, CraftSituation.Orbit, false, false) == SituationVerbs.DeorbitFirst
            && SituationVerbs.WayNow(LandWay.Boostback, CraftSituation.Ground, true, false) == SituationVerbs.AfterSeparation
            && SituationVerbs.WayNow(LandWay.FromOrbit, CraftSituation.Orbit, false, false).Length == 0
            && SituationVerbs.WayNow(LandWay.Hover, CraftSituation.Escaping, false, false) == SituationVerbs.NoImpact);
        Check("the card of a way says the sentence of its page", SituationVerbs.WayNow(LandWay.Boostback, CraftSituation.Ground, true, false, noStageHome: true) == SituationVerbs.NoStageHome);
        foreach (GuidanceModeId mode in Enum.GetValues<GuidanceModeId>())
            Check("the card of a verb says when to press it " + mode, PageText.VerbWhen(mode).Length > 0 && Ascii(PageText.VerbWhen(mode)));
        foreach (string who in new[] { FlownBy.You, FlownBy.Afc, FlownBy.StockAuto, FlownBy.AfcPlansStockAuto })
            Check("the owner chip has a card " + who, PageText.HeadOwnerCard(who).Length > 0 && Ascii(PageText.HeadOwnerCard(who)));
        Check("the WIDE card promises nothing that the screen has no room for", !PageText.WideCard(false, panesFit: false).Contains("Two panes") && PageText.WideCard(false, panesFit: true).Contains("Two panes"));
        Check("no card says every picture gets more room", !PageText.WideCard(false, true).Contains("every picture"));
        Check("the views have a card each", PageText.LaunchViewCards.Length == 4 && PageText.LaunchViewCards.All(c => c.Length > 0 && Ascii(c)));
        Check("the parts of TUNE have a card each", PageText.TuneLandingCards.Length == Enum.GetValues<TuneLanding>().Length && PageText.DiagViewCards.Length == Enum.GetValues<TuneDiagnostics>().Length
            && Enum.GetValues<TuneSet>().All(s => PageText.TuneSetCard(s).Length > 0));
        Check("the fold cards tell their case", PageText.LimitsTip(false).Contains("without atmosphere") && !PageText.LimitsTip(true).Contains("without atmosphere")
            && PageText.BoostersTip(0).Contains("command pod") && !PageText.BoostersTip(2).Contains("command pod"));
        Check("HOW IT FLIES says the kept rule once", PageText.HowItFliesTip.Contains(PageText.KeptRule));
        Check("the card words stay plain", !PageText.PlanTip.Contains("form of this page") && !PageText.SolverTip.Contains("card of a row") && !PageText.FlownByCard(Owner.Afc).Contains("in the head") && !PageText.FlyPassTip(false).StartsWith("Flies this pass now", StringComparison.Ordinal));
        Check("the card of FLY PASS says who burns it, as the row beside it does", PageText.FlyPassTip(false).Contains("STOCK AUTO") && PageText.FlyPassTip(true).Contains("RCS") && !PageText.FlyPassTip(true).Contains("STOCK AUTO"));
        Check("the plain words of a climb to go", PageText.AscentToGo(8717.2, 1328.4) == "SPEED TO GAIN 8717 m/s, DISTANCE TO GO 1328 km");
        Check("the card of the fold is a sentence in ASCII", PageText.HowItFliesTip != PageText.HowItFliesTip.ToUpperInvariant() && Ascii(PageText.HowItFliesTip));

        foreach (LandSolver solver in Enum.GetValues<LandSolver>())
        {
            string card = LandSolvers.Card(solver, 12.0, true, 4.0);
            Check("solver card says where it ends " + solver, card.StartsWith(LandSolvers.Description(solver), StringComparison.Ordinal)
                && card.Contains("Air: ") && card.Contains("End: ") && Ascii(card), card);
        }
        Check("entry must have air", LandSolvers.Card(LandSolver.Entry, 12.0, false, 4.0).Contains("Air: required"));
        Check("touchdown flies with or without air", LandSolvers.Card(LandSolver.Touchdown, 12.0, false, 4.0).Contains("Air: either"));
        Check("G-FOLD is not allowed in air", LandSolvers.Card(LandSolver.Gfold, 12.0, false, 4.0).Contains("Air: not allowed"));
        Check("G-FOLD ends at the hover handoff", LandSolvers.Card(LandSolver.Gfold, 12.0, false, 4.0).EndsWith("End: HOVER FROM 12 m"));
        Check("entry hands the last seconds to touchdown",
            LandSolvers.Card(LandSolver.Entry, 12.0, true, 4.0).EndsWith("End: LANDED, OR TOUCHDOWN FOR LAST 4 s"));
        Check("entry without the hand-over ends on the ground", LandSolvers.Card(LandSolver.Entry, 12.0, false, 4.0).EndsWith("End: LANDED"));

        foreach (string[] lines in new[] { PageText.AscentCards, PageText.ClimbCards, PageText.FlyWithCards, PageText.AttitudeCards })
            foreach (string line in lines)
                Check("description line fits: " + line, line.Length > 0 && line.Length <= 90 && Ascii(line));
        Check("choice cards match their labels", PageText.ClimbCards.Length == PageText.ClimbLabels.Length
            && PageText.FlyWithCards.Length == PageText.FlyWithLabels.Length && PageText.AttitudeCards.Length == PageText.AttitudeLabels.Length
            && PageText.AllocatorCards.Length == PageText.AllocatorLabels.Length && PageText.MoveTips.Length == PageText.MoveLabels.Length);
        Check("each SELECT segment has a card", PageText.TransferSelectCards.Length == 2
            && Array.TrueForAll(PageText.TransferSelectCards, card => card.Length > 0 && Ascii(card)));
        for (int tool = 0; tool < OrbitToolCatalog.ToolLabels.Length; tool++)
            Check("orbit tool description " + OrbitToolCatalog.ToolLabels[tool], OrbitToolCatalog.Description(tool).Length > 0);

        foreach (SettingSpec spec in GuidanceSettingSpecs.AllDoubles)
            Check("setting has help " + spec.Key, spec.Help.Length > 0);
        foreach (BoolSpec spec in GuidanceSettingSpecs.AllBools)
            Check("switch has help " + spec.Key, spec.Help.Length > 0);

        Check("tooltip: unchanged value has no default line",
            PageText.SettingTooltip("Does X.", false, "DEFAULT 5 m", "") == "Does X.");
        Check("tooltip: changed value names its default",
            PageText.SettingTooltip("Does X.", true, "DEFAULT 5 m", "") == "Does X." + NL + "CHANGED DEFAULT 5 m.");
        Check("tooltip: a value guidance sets names who sets it",
            PageText.SettingTooltip("Does X.", true, "DEFAULT 5 m", "Set by guidance.") == "Does X." + NL + "Set by guidance.");

        Check("footer owner of guidance", PageText.OwnerOfFlownBy(FlownBy.Afc) == Owner.Afc
            && PageText.OwnerOfFlownBy(FlownBy.StockAuto) == Owner.Stock && PageText.OwnerOfFlownBy(FlownBy.AfcPlansStockAuto) == Owner.AfcPlansStockFlies
            && PageText.OwnerOfFlownBy(FlownBy.Nobody) == Owner.None);
        Check("a set owner wins over the who-flies text", new CommitSpec("CREATE BURN", FlownBy.StockAuto, Owner: Owner.AfcPlansStockFlies).ShownOwner == Owner.AfcPlansStockFlies
            && new CommitSpec("FLY", FlownBy.StockAuto).ShownOwner == Owner.Stock);
        Check("a plan makes a stock burn and nothing flies yet", PageText.FooterMark(Owner.AfcPlansStockFlies, FlownBy.StockAuto) == new OwnerMark("EXEC: PLANS ", "STOCK", "BURN"));
        Check("the footer says who flies after the press, with its tense", PageText.FooterMark(Owner.Afc, FlownBy.Afc) == new OwnerMark("EXEC: AFC")
            && PageText.FooterMark(Owner.Stock, FlownBy.StockAuto) == new OwnerMark("EXEC:", "STOCK AUTO"));
        Check("a deorbit says who flies first", PageText.FooterMark(Owner.StockThenAfc, FlownBy.Afc) == new OwnerMark("EXEC:", "STOCK AUTO", "THEN AFC"));
        Check("a setting has no owner mark", PageText.FooterMark(Owner.Settings, FlownBy.Nobody).IsEmpty);
        Check("a refusal takes the mark away", new CommitSpec("GO", FlownBy.Afc, "NO", Act: () => { }).Mark.IsEmpty);
        Check("an outline that still works keeps its mark", !new CommitSpec("GO", FlownBy.Afc, Caution: "Not now", Act: () => { }).Mark.IsEmpty);
        Check("a verb place that only waits has no mark", new CommitSpec("LAUNCH ARMED", FlownBy.Afc, Owner: Owner.Settings).Mark.IsEmpty);
        Check("the mark of a plan is short enough for the first line", new OwnerMark("EXEC:", "STOCK BURN").Before.Length + new OwnerMark("EXEC:", "STOCK BURN").Chip.Length <= 20);
        Check("a card explains each mark", PageText.FlownByCard(Owner.StockThenAfc).Length > 0 && PageText.FlownByCard(Owner.AfcPlansStockFlies).Contains("stays MANUAL"));
        Check("a verb has one word for its pass", PlanVerbs.FlyPass(2) == "FLY PASS 2");
        Check("nobody gets no mark", PageText.FooterMark(Owner.None, FlownBy.Nobody).IsEmpty);

        var facts = new PageText.OutcomeFacts(AutoStageArms: true, Target: "Station", Solver: "TOUCHDOWN",
            HandToTouchdown: true, TouchdownLastS: 4, HoverFromM: 15);
        string launch = PageText.GuidanceOutcome(GuidanceModeId.AscentUpfg, "LAUNCH", in facts);
        Check("launch outcome is only what the mark does not say", launch == PageText.AutoStageArmsNote, launch);
        Check("arm outcome names the target", PageText.GuidanceOutcome(GuidanceModeId.AscentUpfg, "ARM", in facts)
            == "Waits for the window of Station, then launches. AutoStage arms at lift-off.");
        Check("calculate first outcome", PageText.GuidanceOutcome(GuidanceModeId.AscentUpfg, "SOLVE + LAUNCH", in facts) == "Solves the plan and launches. AutoStage arms at lift-off.");
        Check("backup outcome", PageText.GuidanceOutcome(GuidanceModeId.AscentUpfg, CommitResolver.BackupLaunchVerb, in facts)
            == "Flies the GRAVITY TURN, not the solved plan. AutoStage arms at lift-off.");
        Check("calculate first outcome keeps the AutoStage note", PageText.GuidanceOutcome(GuidanceModeId.AscentUpfg, "SOLVE + ARM", in facts)
            .EndsWith(PageText.AutoStageArmsNote));
        Check("a landing outcome has no AutoStage note", !PageText.GuidanceOutcome(GuidanceModeId.Hover, "HOVER", in facts).Contains("AutoStage"));
        Check("the ATTITUDE note fits one line", PageText.AttitudeIfRcsNote.Length <= 90 && Ascii(PageText.AttitudeIfRcsNote));
        Check("the guidance arm note is a tooltip sentence", PageText.ArmedByGuidanceTip.Length > 0 && Ascii(PageText.ArmedByGuidanceTip));
        Check("deorbit outcome names stock AUTO and the solver", PageText.GuidanceOutcome(GuidanceModeId.DeorbitLand, "DEORBIT + LAND", in facts)
            == "Burns out of orbit, then lands with TOUCHDOWN.");
        Check("from a fall a deorbit promises no burn", PageText.GuidanceOutcome(GuidanceModeId.DeorbitLand, "DEORBIT + LAND", facts with { Falling = true }) == "Lands with TOUCHDOWN, with a stock burn first if needed.");
        Check("the deorbit burn is the phase of the chain", GuidanceModes.DeorbitPhases.Contains("DEORBIT BURN") && LandChain.StockBurn == "DEORBIT BURN");
        Check("entry outcome with the hand-over", PageText.GuidanceOutcome(GuidanceModeId.EntryHere, "ENTRY", in facts)
            == "Glides engine off, then flies the landing burn. TOUCHDOWN flies the last 4 s.");
        Check("G-FOLD outcome", PageText.GuidanceOutcome(GuidanceModeId.GfoldHere, CommitResolver.GfoldVerb, in facts)
            == "Flies a G-FOLD descent, then hovers down from 15 m.");
        foreach (GuidanceModeId mode in new[] { GuidanceModeId.AscentUpfg, GuidanceModeId.Boostback, GuidanceModeId.DeorbitLand, GuidanceModeId.EntryHere,
                     GuidanceModeId.SixDofHere, GuidanceModeId.GfoldHere, GuidanceModeId.Hover })
        {
            string o = PageText.GuidanceOutcome(mode, "X", in facts);
            Check("outcome is a sentence in ASCII " + mode, o.Length > 0 && char.IsUpper(o[0]) && o != o.ToUpperInvariant() && Ascii(o), o);
        }
        string plainLaunch = PageText.GuidanceOutcome(GuidanceModeId.AscentUpfg, "LAUNCH", new PageText.OutcomeFacts(Target: ""));
        Check("a launch with no AutoStage note has no sentence, because the mark says who flies", plainLaunch.Length == 0, plainLaunch);
        Check("a launch with an AFC MAY switch off says so", PageText.GuidanceOutcome(GuidanceModeId.AscentUpfg, "LAUNCH", new PageText.OutcomeFacts(MayFly: false)) == PageText.MayOffLaunch);
        Check("passes outcome", PageText.PassesOutcome(3) == "Adds pass 1 of 3. AFC adds each next pass.");
        Check("reset outcome", PageText.ResetSetOutcome(1) == "Resets 1 changed value to its default."
            && PageText.ResetSetOutcome(3) == "Resets 3 changed values to their defaults.");
        Check("hover setpoint is invariant", PageText.HoverSetpoint(0.0, 0.5, -0.5) == "E 0.0  N 0.5  UP -0.5 m/s");
        Check("the cutoff cue leaves the next step to its link", PageText.CutoffCue == "ASCENT COMPLETE");
        foreach (string label in new[] { PageText.BannerBackup, PageText.BannerKeep, PageText.BannerHide, PageText.AutoStageRowLabel })
            Check("panel label is upper case " + label, label == label.ToUpperInvariant() && Ascii(label));
    }

    // At the narrowest window a row has 427 px at 9.1 px per character, less a number field of 110 px and the stock gap of 16 px.
    private static void LaunchReachRules()
    {
        const double Mu = 3.986004418e14, R = 6371e3, Starbase = 25.996;
        double vCirc = Math.Sqrt(Mu / (R + 200e3));
        double vEquator = 2.0 * Math.PI * R / (23.9344695944 * 3600.0);
        Check("a site reaches no inclination below its latitude", LaunchReach.LowestDirectIncDeg(Starbase) == Starbase && LaunchReach.LowestDirectIncDeg(-39.26) == 39.26);
        Check("a site reaches no retrograde inclination above 180 less its latitude", Math.Abs(LaunchReach.HighestDirectIncDeg(Starbase) - 154.004) < 1e-9);
        Check("an equatorial orbit from Starbase needs its latitude as plane change", Math.Abs(LaunchReach.LeastPlaneChangeDeg(Starbase, 0.0) - Starbase) < 1e-9);
        Check("the station plane from Starbase needs none", LaunchReach.LeastPlaneChangeDeg(Starbase, 51.6) == 0.0 && LaunchReach.LeastPlaneChangeDeg(Starbase, 97.4) == 0.0);
        Check("a retrograde plane beyond reach needs the rest", Math.Abs(LaunchReach.LeastPlaneChangeDeg(Starbase, 170.0) - 15.996) < 1e-9);
        double pc = LaunchReach.PlaneChangeMs(Mu, R + 200e3, R + 200e3, Starbase);
        Check("26 deg at 200 km costs about 3.5 km/s", Math.Abs(pc - 3504.0) < 5.0, pc.ToString("F1"));
        double pcEcc = LaunchReach.PlaneChangeMs(Mu, R + 200e3, R + 2000e3, 10.0);
        Check("an eccentric orbit turns cheaper at its apoapsis than a circle at its periapsis", pcEcc < LaunchReach.PlaneChangeMs(Mu, R + 200e3, R + 200e3, 10.0));
        Check("no plane change costs nothing", LaunchReach.PlaneChangeMs(Mu, R + 200e3, R + 200e3, 0.0) == 0.0);
        Check("the nearest direct inclination rounds into the band", LaunchReach.NearestDirectIncDeg(Starbase, 0.0) == 26.0 && LaunchReach.NearestDirectIncDeg(Starbase, 170.0) == 154.0 && LaunchReach.NearestDirectIncDeg(Starbase, 51.6) == 51.6);
        Check("the rounded offer is itself in reach", LaunchReach.LeastPlaneChangeDeg(Starbase, LaunchReach.NearestDirectIncDeg(Starbase, 0.0)) == 0.0);
        double lat = Starbase * Math.PI / 180.0;
        Check("a plane through the site needs no plane change", LaunchReach.PlaneChangeDeg(0.0, -Math.Sin(lat), Math.Cos(lat), 0.0, Math.Cos(lat), Math.Sin(lat)) < 1e-9);
        Check("the equator plane from the site needs its latitude", Math.Abs(LaunchReach.PlaneChangeDeg(0, 0, 2.0, Math.Cos(lat), 0, Math.Sin(lat)) - Starbase) < 1e-9);
        double loss = LaunchReach.AzimuthCostMs(Starbase, 51.6, vCirc, vEquator);
        Check("the station plane costs about 135 m/s over due east from Starbase", Math.Abs(loss - 135.0) < 3.0, loss.ToString("F1"));
        double polar = LaunchReach.AzimuthCostMs(Starbase, 90.0, vCirc, vEquator);
        Check("a polar plane costs about 430 m/s over due east", Math.Abs(polar - 429.0) < 3.0, polar.ToString("F1"));
        Check("due east costs nothing over itself", Math.Abs(LaunchReach.AzimuthCostMs(Starbase, Starbase, vCirc, vEquator)) < 1e-6);
        Check("a plane out of direct reach has no azimuth", double.IsNaN(LaunchReach.GroundRelativeSpeedMs(Starbase, 0.0, vCirc, vEquator)));

        var free = new ReachFacts(Starbase, 0.0, Starbase, Starbase, pc, false);
        Check("an equatorial launch from Starbase shows the row and offers 26.0", free.Shows && free.OutOfReach && free.OfferIncDeg == 26.0);
        Check("the row says the plane change, its price and the band", PageText.Reach(in free) == "26.0 deg PLANE CHANGE, ~3504 m/s. DIRECT: INC 26.0 TO 154.0", PageText.Reach(in free));
        Check("the fix names the inclination", PageText.ReachFix(26.0) == "USE INC 26.0 deg");
        var target = free with { HasTarget = true };
        Check("a target sets the plane, so nothing is offered", target.Shows && double.IsNaN(target.OfferIncDeg));
        var stale = new ReachFacts(Starbase, 51.6, 12.0, 0.0, 1600.0, false);
        Check("a LAN the site turned away from shows, with no inclination offered", stale.Shows && !stale.OutOfReach && double.IsNaN(stale.OfferIncDeg) && PageText.Reach(in stale).Contains("FROM POSITION"));
        Check("a plane over the site shows nothing", !new ReachFacts(Starbase, 51.6, 0.2, 0.0, 30.0, false).Shows);
        Check("the reach texts are ASCII", (PageText.Reach(in free) + PageText.ReachTip(in free, false) + PageText.ReachTip(in free, true) + PageText.ReachTip(in stale, true) + PageText.ReachFixTip).All(ch => ch < 128));
        Check("the gravity turn tip says LAUNCH refuses", PageText.ReachTip(in free, true).Contains("LAUNCH refuses") && !PageText.ReachTip(in free, false).Contains("refuses"));

        string refusal = PageText.ReachRefusal(Starbase, 0.0, false);
        Check("the refusal names the inclination, the band and the nearest one", refusal == "REFUSED: INC 0.0 deg OUT OF REACH FROM LATITUDE 26.0 deg (DIRECT: INC 26.0 TO 154.0). THE GRAVITY TURN CANNOT FLY A PLANE CHANGE. NEAREST: INC 26.0 deg.", refusal);
        string targetRefusal = PageText.ReachRefusal(-Starbase, 10.0, true);
        Check("a target's plane is refused without an offer", targetRefusal.StartsWith("REFUSED: TARGET INC 10.0 deg") && !targetRefusal.Contains("NEAREST"), targetRefusal);
        Check("a retrograde plane beyond reach offers the highest", PageText.ReachRefusal(Starbase, 170.0, false).EndsWith("NEAREST: INC 154.0 deg."));
        Check("the refusals are ASCII", (refusal + targetRefusal).All(ch => ch < 128));
    }

    private static void AscentEndRules()
    {
        string orbit = PageText.AscentOutOfPropellant(true, 204.4, 15559.0, 93.26, 140.0);
        Check("an ascent out of propellant says so and the orbit it left", orbit == "ASCENT ENDED: OUT OF PROPELLANT BEFORE CUTOFF. ORBIT 204 x 15559 km, INC 93.3 deg. MANUAL.", orbit);
        Check("not ON PLAN", !orbit.Contains("ON PLAN"));
        string sub = PageText.AscentOutOfPropellant(true, -3000.0, 85.0, 30.0, 140.0);
        Check("a periapsis under the surface is suborbital", sub == "ASCENT ENDED: OUT OF PROPELLANT BEFORE CUTOFF. SUBORBITAL, AP 85 km. MANUAL.", sub);
        string low = PageText.AscentOutOfPropellant(true, 161.0, 31081.4, 51.42, 200.0);
        Check("a periapsis in the air is an orbit the air lowers", low == "ASCENT ENDED: OUT OF PROPELLANT BEFORE CUTOFF. ORBIT 161 x 31081 km, INC 51.4 deg, PE IN THE AIR. MANUAL.", low);
        Check("an open orbit is an escape", PageText.AscentOutOfPropellant(false, 200.0, -1.0, 30.0, 140.0).Contains("ESCAPE TRAJECTORY"));

        const double G = 9.81;
        // A craft of 100 t: boosters and core at lift-off, the core alone after the drop, then an upper stage on its own engine.
        var strapOn = new[] { new TwrStage(1500e3, 100e3, true), new TwrStage(500e3, 60e3, true), new TwrStage(100e3, 15e3, false) };
        LaunchTwrFacts f = LaunchTwr.Of(strapOn, G);
        Check("lift-off TWR", Math.Abs(f.LiftoffTwr - 1500e3 / (100e3 * G)) < 1e-12 && !f.CannotLiftOff);
        Check("the core after the drop is the lowest first-stage TWR", f.LowestStage == 1 && Math.Abs(f.LowestTwr - 500e3 / (60e3 * G)) < 1e-12 && f.LowestTwr < 1.0 && f.Shows, f.ToString());
        Check("the row names stage 2", PageText.Twr(in f) == "0.85 WHEN STAGE 2 STARTS, STILL IN THE FIRST STAGE", PageText.Twr(in f));
        Check("an upper stage under 1 is not the first stage", !LaunchTwr.Of(new[] { new TwrStage(1500e3, 100e3, true), new TwrStage(100e3, 15e3, false) }, G).Shows);
        Check("a stage after the upper stage does not count either", LaunchTwr.Of(new[] { new TwrStage(1500e3, 100e3, true), new TwrStage(400e3, 50e3, false), new TwrStage(1e3, 40e3, true) }, G).LowestStage == -1);
        LaunchTwrFacts heavy = LaunchTwr.Of(new[] { new TwrStage(900e3, 100e3, true) }, G);
        Check("a craft heavier than its thrust cannot lift off", heavy.CannotLiftOff && heavy.Shows && PageText.Twr(in heavy) == "0.92 AT LIFT-OFF. THE CRAFT DOES NOT LEAVE THE PAD", PageText.Twr(in heavy));
        Check("a strong first stage shows nothing", !LaunchTwr.Of(new[] { new TwrStage(1500e3, 100e3, true), new TwrStage(800e3, 60e3, true) }, G).Shows);
        Check("no stages, no facts", !LaunchTwr.Of(Array.Empty<TwrStage>(), G).Known && !LaunchTwr.Unknown.Shows);
        Check("the TWR texts are ASCII", (PageText.TwrTip(in f, true) + PageText.TwrTip(in f, false) + PageText.TwrTip(in heavy, true)).All(ch => ch < 128));
    }

    private static void FormRows()
    {
        Check("a short burn has no PASSES row", !FormRules.PassesCanMatter(915.0, 24.0, 5500.0));
        Check("a small burn has no PASSES row", !FormRules.PassesCanMatter(14.7, 0.4, 5500.0));
        Check("a long burn of a weak engine has a PASSES row", FormRules.PassesCanMatter(915.0, 400.0, 5500.0));
        Check("no burn time or no period says no", !FormRules.PassesCanMatter(915.0, 0.0, 5500.0) && !FormRules.PassesCanMatter(915.0, 400.0, double.NaN)
            && !FormRules.PassesCanMatter(915.0, 400.0, 0.0) && !FormRules.PassesCanMatter(0.0, 400.0, 5500.0));

        Check("no target and a calculated climb: no window row", FormRules.WindowRow(false, false, false, false) == LaunchWindowRow.None);
        Check("no target and the pitch program: the node choice", FormRules.WindowRow(false, false, false, true) == LaunchWindowRow.NodeChoice);
        Check("a target with a window counts down", FormRules.WindowRow(true, true, false, false) == LaunchWindowRow.Countdown
            && FormRules.WindowRow(true, true, false, true) == LaunchWindowRow.Countdown);
        Check("a plane out of reach has no window", FormRules.WindowRow(true, false, true, true) == LaunchWindowRow.NoCrossing);
        Check("a target that is gone has no window and no choice", FormRules.WindowRow(true, false, false, true) == LaunchWindowRow.None);

        Span<float> widths = stackalloc float[2];
        DockLayout.SegmentWidths(new[] { 60f, 80f }, 200f, 8f, widths);
        Check("segments that fit share the bar equally", widths[0] == 100f && widths[1] == 100f);
        DockLayout.SegmentWidths(new[] { 130f, 216f }, 475f, 9.6f, widths);
        Check("a long name is not cut beside a short one", widths[1] >= 216f + 2f * 9.6f && widths[0] >= 130f + 2f * 9.6f, widths[1].ToString());
        Check("the segments fill the bar", Math.Abs(widths[0] + widths[1] - 475f) < 0.01f);
        DockLayout.SegmentWidths(new[] { 300f, 300f }, 400f, 8f, widths);
        Check("a bar too narrow for its names stays inside its width", Math.Abs(widths[0] + widths[1] - 400f) < 0.01f);

        Check("an empty picture is low", DockLayout.EmptyPictureHeight < DockLayout.MinPictureHeight);

        Check("a plan that can fly says what it brings", PageText.LaunchAnswer("", "Plan ready: 16.27 t to orbit", true, true, 16270.0, 4, 4)
            == "PLAN READY: 16.3 t TO ORBIT, 4/4 STAGES");
        Check("no plan yet", PageText.LaunchAnswer("NO PLAN", "", false, true, double.NaN, 0, 0) == "NO PLAN");
        Check("an old plan says why", PageText.LaunchAnswer("the vehicle has moved since it was planned", "Plan ready: 16.27 t", true, true, 16270.0, 4, 4)
            == "PLAN OUT OF DATE: the vehicle has moved since it was planned");
        Check("a calculation that failed keeps its reason", PageText.LaunchAnswer("NO SOLUTION", "No converged plan: stalled. Raise Max q above 180 kPa.", false, true, 0.0, 4, 4)
            == "No converged plan: stalled. Raise Max q above 180 kPa.");
        Check("a calculation that failed says so with the pitch program too", PageText.LaunchAnswer("", "Cannot calculate: no stages.", false, false, double.NaN, 0, 0)
            == "Cannot calculate: no stages.");
        Check("the simple turn says that it flies", PageText.LaunchAnswer("", "", false, false, double.NaN, 0, 0) == "READY: GRAVITY TURN"
            && PageText.LaunchAnswer("", "Plan ready: 16.27 t", true, false, 16270.0, 4, 4) == "READY: GRAVITY TURN");
        Check("a word of the planner on its way is no reason", PageText.LaunchAnswer("NO PLAN", "STARTING...", false, true, double.NaN, 0, 0) == "NO PLAN");

        Check("the node choice has its cards", PageText.NodeCards.Length == PageText.NodeLabels.Length);

        Check("the LIMITS fold says what it holds", PageText.LimitsLabel(true, 140.0, false, 4.0) == "LIMITS: MAX Q 140 kPa, G-LIMIT OFF");
        Check("the LIMITS fold with a limit on", PageText.LimitsLabel(true, 140.5, true, 4.0) == "LIMITS: MAX Q 140.5 kPa, G-LIMIT 4 g");
        Check("the LIMITS fold on a body with no air", PageText.LimitsLabel(false, 140.0, true, 2.5) == "LIMITS: G-LIMIT 2.5 g");

        Check("the BOOSTERS fold with no stage", PageText.BoostersLabel(0, 0, 0.0) == "BOOSTERS: NONE");
        Check("the BOOSTERS fold with stages", PageText.BoostersLabel(2, 1, 0.0) == "BOOSTERS: 1/2 WITH SITE");
        Check("the BOOSTERS fold with a reserve", PageText.BoostersLabel(2, 1, 4200.0) == "BOOSTERS: 1/2 WITH SITE, RESERVE 4.2 t"
            && PageText.BoostersLabel(0, 0, 4200.0) == "BOOSTERS: RESERVE 4.2 t");
        Check("the BOOSTERS tip names no reserve to set", !PageText.BoostersTip(2).Contains("delta v", StringComparison.OrdinalIgnoreCase)
            && PageText.BoostersTip(2).Contains("sizes"));

        bool reserveWarn;
        Check("no reserve row with no site", PageText.BoosterReserve(false, false, false, 0.0, double.NaN, double.NaN, "", out reserveWarn).Length == 0 && !reserveWarn);
        Check("a staged reserve says so, also with no site left", PageText.BoosterReserve(false, false, true, 0.0, double.NaN, double.NaN, "", out reserveWarn).StartsWith("STAGED") && !reserveWarn);
        Check("a site with no plan waits for the calculated climb", PageText.BoosterReserve(true, true, false, 0.0, double.NaN, double.NaN, "SIZED WHEN PLAN IS SOLVED", out reserveWarn) == PageText.ReserveWaitsForPlan && !reserveWarn);
        Check("a planned reserve names its mass and both burns", PageText.BoosterReserve(true, true, false, 4200.0, 1450.0, 320.0, "", out reserveWarn) == "4.20 t: 1450 m/s BOOSTBACK + 320 m/s LANDING" && !reserveWarn);
        Check("a reserve that does not arm says why, in amber", PageText.BoosterReserve(true, false, false, 0.0, double.NaN, double.NaN, "ARMS AFTER STAGES BELOW SEPARATE", out reserveWarn) == "NOT ARMED: ARMS AFTER STAGES BELOW SEPARATE" && reserveWarn);
        Check("a plan that needs no reserve says so", PageText.BoosterReserve(true, true, false, 0.0, 0.0, 0.0, "", out reserveWarn).StartsWith("NONE") && !reserveWarn);

        Check("the window row names the node and the lead", PageText.WindowCountdown(725.0, true, 180.0) == "T-12m 05s, DESC NODE, 180 s LEAD");
        Check("the window row counts to the lift-off the plan chose", PageText.WindowCountdown(725.0, false, 180.0, 95.0) == "T-13m 40s, ASC NODE, PLAN TIME, 95 s AFTER 180 s LEAD"
            && PageText.WindowCountdown(725.0, false, 180.0, -125.0) == "T-10m 00s, ASC NODE, PLAN TIME, 125 s BEFORE 180 s LEAD");
        Check("the timing switch names what it chooses", PageText.OptimiseTimingLabel(true) == "SOLVE LIFTOFF" && PageText.OptimiseTimingLabel(false) == "SOLVE LAN");
        Check("the timing switch is an expert switch of the launch, on by default", GuidanceSettingSpecs.ConvexOptimiseTiming.Tier == UiTier.Expert
            && GuidanceSettingSpecs.ConvexOptimiseTiming.Default
            && Array.IndexOf(GuidanceSettingSpecs.LaunchExpertBools, GuidanceSettingSpecs.ConvexOptimiseTiming) >= 0
            && Array.IndexOf(GuidanceSettingSpecs.AllBools, GuidanceSettingSpecs.ConvexOptimiseTiming) >= 0);
        Check("both names of the timing switch fit a row", PageText.OptimiseTimingLabel(true).Length <= GuidanceSettingSpecs.ShowConvexPlan.Label.Length + 2);
        Check("the plan's booster line", PageText.PlanBooster(4200.0, 1450.0, 320.0, 142.4) == "RESERVE 4.20 t: 1450 m/s BOOSTBACK + 320 m/s LANDING, SEPARATION T+142 s");
        Check("the plan's launch time", PageText.PlanLaunchShift(-95.0) == "95 s BEFORE WINDOW" && PageText.PlanLaunchShift(40.0) == "40 s AFTER WINDOW");
        Check("the plan's own LAN", PageText.PlanLanChosen(213.456) == "213.46 deg, SOLVED FOR PAD");
        Check("an armed launch at the window says the window", PageText.ArmedLaunch(false, true) == "ARMED. LAUNCH AT WINDOW."
            && PageText.ArmedLaunch(false, false, 0.2) == "ARMED. WARP TO WINDOW, LAUNCH FOLLOWS."
            && PageText.ArmedLaunch(true, true, double.NaN) == "WARPING TO WINDOW");
        Check("an armed launch follows the lift-off the plan chose", PageText.ArmedLaunch(false, true, 95.0) == "ARMED. LAUNCH AT PLANNED LIFTOFF, 95 s AFTER WINDOW."
            && PageText.ArmedLaunch(false, false, -40.0) == "ARMED. WARP TO PLANNED LIFTOFF, 40 s BEFORE WINDOW, LAUNCH FOLLOWS."
            && PageText.ArmedLaunch(true, true, 95.0) == "WARPING TO PLANNED LIFTOFF, 95 s AFTER WINDOW.");

        // The ASCENT row in the right pane of the wide console at interface scale 1.2: 10.9 px per character, 9.6 px of padding on each side of a segment, a 108 px label column.
        static bool AscentFits(float pane, string[] segments) => segments.Sum(s => s.Length * 10.9f + 2f * 9.6f) <= pane - 90f * 1.2f;
        string[] ascentNames = { "CAT-S / UPFG", "GRAVITY TURN" };
        bool[] catsOnly = { true, false };
        float widePane = DockLayout.RightPaneWidth * 1.2f;
        float narrowPane = DockLayout.NarrowWidth * 1.2f - 40f;
        string[] wideAscent = PageText.AscentSegments(ascentNames, catsOnly, s => AscentFits(widePane, s));
        string[] narrowAscent = PageText.AscentSegments(ascentNames, catsOnly, s => AscentFits(narrowPane, s));
        Check("the wide pane drops the suffix of a method that comes later, so both names stand whole", wideAscent[1] == "GRAVITY TURN" && AscentFits(widePane, wideAscent), string.Join(" | ", wideAscent));
        Check("the narrow console keeps the suffix", narrowAscent[1] == "GRAVITY TURN" + PageText.NotYetSuffix && narrowAscent[0] == "CAT-S / UPFG", string.Join(" | ", narrowAscent));
        Check("with every method there, the names stand as they are", PageText.AscentSegments(ascentNames, new[] { true, true }, _ => false).SequenceEqual(ascentNames));
        Check("the defaults follow the state: max q 195 kPa, min throttle 80 %, settling 0.2 s", GuidanceSettingSpecs.ConvexQMaxKpa.Default == 195.0
            && GuidanceSettingSpecs.ConvexThrottleMinPct.Default == 80.0 && GuidanceSettingSpecs.BoostbackSeparationS.Default == 0.2);
        Check("the progress has one colon less", PageText.PlanProgress("4 stages: searching for a seed", 3.2) == "4 stages, searching for a seed (3 s)");

        Check("one burn is one line, and the answer row holds what is on board", PageText.BurnFuelLine("Seq 1", 2930.0, 0.0) == "PAID BY Seq 1, ~2.9 t");
        Check("a burn with no payer has no line, because the answer row holds what is on board", PageText.BurnFuelLine("", 0.0, 0.0).Length == 0);
        Check("a burn after a burn counts what that burn leaves", PageText.FuelShort(1100.0, 1062.0, chained: true) == "NEEDS 1100 m/s, LAST BURN LEAVES ~1062");
        Check("a burn over budget has no line, because the footer says it", PageText.BurnFuelLine("Seq 1", 2930.0, 2229.5).Length == 0);
        Check("passes that fall short say by how much", PageText.PassesShort(11.6) == "Fuel ~12 m/s short of the goal.");
        Check("a press of a deorbit says what it flies and how to get the closest", PageText.PressPlansNext == "Flies the next pass.");
        Check("the pass a deorbit flies is the pass nearest its arrival", SketchMath.NearestTime(new[] { 100.0, 7000.0, 13000.0 }, 6800.0) == 1 && SketchMath.NearestTime(new[] { 100.0 }, double.NaN) == -1 && SketchMath.NearestTime(ReadOnlySpan<double>.Empty, 5.0) == -1);
        Check("the pass that flies has its number", PageText.PassFlies(112.6, 7585.0) == "PASS IN 2h 06m, NOW 112.6 km OFF");
        Check("the answer row of a burn", PageText.BurnAnswer(916.1, "IN 1h 28m", 1978.0) == "916.1 m/s IN 1h 28m, 46 % OF ~1978 m/s");
        Check("with no estimate the share goes", PageText.BurnAnswer(916.1, "IN 1h 28m", double.NaN) == "916.1 m/s IN 1h 28m");
        Check("a chain says the share of what is left", PageText.BurnAnswer(916.1, "IN 1h 28m", 1062.0, chained: true) == "916.1 m/s IN 1h 28m, 86 % OF ~1062 m/s LEFT");
    }

    private static void ManeuverSeeds()
    {
        const double margin = 1.0;

        Check("the phase follows the situation", ManeuverSeed.PhaseOf(CraftSituation.Ground) == SeedPhase.Ground
            && ManeuverSeed.PhaseOf(CraftSituation.Orbit) == SeedPhase.Orbit && ManeuverSeed.PhaseOf(CraftSituation.Escaping) == SeedPhase.Escaping);
        Check("a climb and a fall are one phase, so a typed value survives an apoapsis",
            ManeuverSeed.PhaseOf(CraftSituation.CoastingUp) == ManeuverSeed.PhaseOf(CraftSituation.FallingAir)
            && ManeuverSeed.PhaseOf(CraftSituation.CoastingUp) == ManeuverSeed.PhaseOf(CraftSituation.FallingNoAir));

        Check("SET AP seeds the next even altitude", ManeuverSeed.ApoapsisKm(200.0, 420.0, margin) == 500.0);
        Check("SET AP on a big orbit takes a bigger step", ManeuverSeed.ApoapsisKm(200.0, 5000.0, margin) == 6000.0);
        Check("SET AP on a circular orbit clears the periapsis", ManeuverSeed.ApoapsisKm(207.0, 207.2, margin) == 300.0);
        Check("SET AP on a low body stays small", ManeuverSeed.ApoapsisKm(15.0, 20.0, margin) == 30.0);
        Check("an even value just above is skipped", ManeuverSeed.NextEvenAbove(99.95) == 110.0);
        Check("no value reads as zero", ManeuverSeed.NextEvenAbove(double.NaN) == 10.0 && ManeuverSeed.NextEvenBelow(double.NaN) == 0.0);

        Check("SET PE raises a low periapsis", ManeuverSeed.PeriapsisKm(200.0, 420.0, margin) == 300.0);
        Check("SET PE on a circular orbit lowers it", ManeuverSeed.PeriapsisKm(207.0, 207.2, margin) == 200.0);
        Check("SET PE on a suborbital path has a target over the ground", ManeuverSeed.PeriapsisKm(0.0, 180.0, margin) == 100.0);
        Check("SET PE on a low body", ManeuverSeed.PeriapsisKm(15.0, 20.0, margin) == 10.0);

        foreach ((double pe, double ap) in new[] { (200.0, 420.0), (207.0, 207.2), (200.0, 200.0), (15.0, 20.0), (0.0, 180.0), (100.0, 40000.0), (5.0, 5.5), (214.6, 215.2) })
        {
            double setAp = ManeuverSeed.ApoapsisKm(pe, ap, margin);
            Check($"the SET AP seed opens with no error on {pe} x {ap}",
                ManeuverSeed.ApseFits(false, setAp, pe, ap, margin) && !ManeuverSeed.AtApse(false, setAp, pe, ap), setAp.ToString());
            double setPe = ManeuverSeed.PeriapsisKm(pe, ap, margin);
            Check($"the SET PE seed opens with no error on {pe} x {ap}",
                ManeuverSeed.ApseFits(true, setPe, pe, ap, margin) && !ManeuverSeed.AtApse(true, setPe, pe, ap), setPe.ToString());
        }
        Check("a pad value is out of place in orbit", !ManeuverSeed.ApseFits(false, 0.03, 200.0, 420.0, margin));
        Check("a target at the apsis is a burn of nothing", ManeuverSeed.AtApse(false, 420.0, 200.0, 420.04) && !ManeuverSeed.AtApse(false, 430.0, 200.0, 420.0));

        Check("SET INC seeds the next multiple of five", ManeuverSeed.InclinationDeg(51.6) == 55.0);
        Check("SET INC from the equator", ManeuverSeed.InclinationDeg(0.0) == 5.0);
        Check("SET INC skips a value just above", ManeuverSeed.InclinationDeg(54.5) == 60.0);
        Check("SET INC goes down at the top", ManeuverSeed.InclinationDeg(180.0) == 175.0 && ManeuverSeed.InclinationDeg(179.5) == 175.0);
        for (double deg = 0.0; deg <= 180.0; deg += 0.7)
        {
            double seed = ManeuverSeed.InclinationDeg(deg);
            Check($"the SET INC seed opens with no error at {deg}", !ManeuverSeed.AtInclination(seed, deg) && seed >= 0.0 && seed <= 180.0, seed.ToString());
        }

        Check("MANEUVER starts on CIRCULARIZE", OrbitToolCatalog.StartKey(0.02) == OrbitToolCatalog.KeyCircularizeApoapsis);
        Check("a circular orbit starts on SET AP, because it has nothing to circularize", OrbitToolCatalog.StartKey(0.0005) == OrbitToolCatalog.KeySetApoapsis);
        Check("a cue names the circularize tool", OrbitToolCatalog.KeyOfCue(OrbitToolCatalog.CueCircularize) == OrbitToolCatalog.KeyCircularizeApoapsis && OrbitToolCatalog.KeyOfCue("") == null);
    }

    private static void NextSteps()
    {
        var orbit = new SituationFacts(CraftSituation.Orbit, "Earth", 200.0, 5000.0);
        var falling = new SituationFacts(CraftSituation.FallingAir, "Earth");
        var escaping = new SituationFacts(CraftSituation.Escaping, "Earth");

        StatusLine ascent = NextStep.AfterAscent(0.02);
        Check("after the ascent: CIRCULARIZE", ascent is { Severity: Severity.Cue, Text: PageText.CutoffCue } && NextStep.LinkLabel(in ascent) == "CIRCULARIZE >"
            && ConsolePages.TryParseCue(ascent.CueTarget, out ConsolePage ascentPage, out string ascentSub) && ascentPage == ConsolePage.Orbit && OrbitToolCatalog.KeyOfCue(ascentSub) != null);
        StatusLine circular = NextStep.AfterAscent(0.0005);
        Check("after an ascent to a circular orbit: MANEUVER with no tool", NextStep.LinkLabel(in circular) == "MANEUVER >" && circular.CueTarget == ConsolePages.Key(ConsolePage.Orbit));

        StatusLine pass = NextStep.AfterPass(1, 2, "1h 28m", ConsolePage.Orbit);
        Check("after a pass: the next pass", pass.Text == "PASS 1 COMPLETE. PASS 2 IN 1h 28m." && NextStep.LinkLabel(in pass) == "PASS 2 >" && pass.CueTarget == ConsolePages.Key(ConsolePage.Orbit));
        Check("the passes of a transfer lead to TRANSFER", NextStep.AfterPass(1, 2, "1m", ConsolePage.Transfer).CueTarget == ConsolePages.Key(ConsolePage.Transfer));

        StatusLine passes = NextStep.AfterPasses(in orbit, ConsolePage.Orbit);
        Check("after the last pass: the orbit", passes.Text == "PASSES COMPLETE, 200 x 5000 km." && NextStep.LinkLabel(in passes) == "MANEUVER >");

        StatusLine burn = NextStep.AfterBurn(in orbit, "");
        Check("after a burn: the orbit", burn.Text == "BURN COMPLETE, 200 x 5000 km." && NextStep.LinkLabel(in burn) == "MANEUVER >");
        Check("after a transfer burn: TRANSFER", NextStep.AfterBurn(in orbit, "", ConsolePage.Transfer).CueTarget == ConsolePages.Key(ConsolePage.Transfer));
        StatusLine next = NextStep.AfterBurn(in orbit, "1h 28m");
        Check("after a burn with another planned: BURNS", next.Text == "BURN COMPLETE. NEXT BURN IN 1h 28m." && NextStep.LinkLabel(in next) == "BURNS >");
        StatusLine down = NextStep.AfterBurn(in falling, "");
        Check("after a burn that falls: LAND", down.Text == "BURN COMPLETE. NEXT: LAND." && NextStep.LinkLabel(in down) == "LAND >");
        StatusLine away = NextStep.AfterBurn(in escaping, "");
        Check("after a burn on an escape path: TRANSFER", away.Text == "BURN COMPLETE." && NextStep.LinkLabel(in away) == "TRANSFER >");

        StatusLine landed = NextStep.AfterLanding("Luna");
        Check("after a landing: LAUNCH", landed.Text == "LANDED, Luna" && NextStep.LinkLabel(in landed) == "LAUNCH >");

        Check("a link names the way of LAND", NextStep.LinkLabel(new StatusLine(Severity.Cue, "x", ConsolePages.CueTarget(ConsolePage.Land, ConsolePages.EnumKey(LandWay.Hover)))) == "HOVER >");
        Check("a target that is no page opens the drawer", NextStep.LinkLabel(new StatusLine(Severity.Cue, "x", "DRAWER")) == PageText.DrawerLink);

        // At the narrowest window the head row has 49 characters beside the frame of the link, at 10.9 px per character at scale 1.2.
        const int rowInChars = 49;
        var hints = new[]
        {
            ascent, circular, pass, passes, burn, next, down, away, landed,
            NextStep.AfterPass(10, 11, "267d 23h", ConsolePage.Transfer),
            NextStep.AfterPasses(new SituationFacts(CraftSituation.Orbit, "Earth", 20000.0, 400000.0), ConsolePage.Transfer),
            NextStep.AfterBurn(new SituationFacts(CraftSituation.Orbit, "Earth", 20000.0, 400000.0), ""),
            NextStep.AfterBurn(in orbit, "267d 23h"),
            LandSolvers.CueFor(LiveLanding.Entry, LiveLanding.Touchdown)!.Value,
            LandSolvers.CueFor(LiveLanding.None, LiveLanding.Touchdown)!.Value,
            LandSolvers.CueFor(LiveLanding.None, LiveLanding.Entry)!.Value,
        };
        foreach (StatusLine hint in hints)
        {
            string link = NextStep.LinkLabel(in hint);
            Check("one line and one link, no cut: " + hint.Text, hint.Text.Length + link.Length <= rowInChars, (hint.Text.Length + link.Length).ToString());
            Check("the link says where it leads: " + hint.Text, link.EndsWith(" >") && link == link.ToUpperInvariant());
        }

        Check("the created burn line fits beside its link", PageText.CreatedNext.Length + PageText.NextToolLink.Length + 2 <= rowInChars);

        // A head line with no link has the 49 characters and the room of the link frame.
        const int rowNoLinkInChars = rowInChars + 4;
        Check("the line of the site pick fits the head row", PageText.PickSiteCue.Length <= rowNoLinkInChars, PageText.PickSiteCue.Length.ToString());
        string longestPlace = SituationText.Line(new SituationFacts(CraftSituation.Orbit, "Earth", 20000.0, 400000.0));
        Check("the situation line fits beside the drawer control", longestPlace.Length + PageText.MoreLabel(0, false).Length <= rowNoLinkInChars, longestPlace.Length.ToString());

        foreach (string verb in new[] { "SOLVE + LAUNCH", "SOLVE + ARM", "ARM", CommitResolver.BackupLaunchVerb, "BOOSTBACK", "DEORBIT + LAND", CommitResolver.GfoldVerb, "TOUCHDOWN", EntryGuidance.CommitVerb, "HOVER", "CREATE FLYBY DEPARTURE" })
        {
            foreach (string link in new[] { PageText.NextLabel(ConsolePage.Orbit, null), PageText.NextLabel(ConsolePage.Land, LandWay.FromOrbit), PageText.NextLabel(ConsolePage.Land, LandWay.FromHere), PageText.NextLabel(ConsolePage.Launch, null) })
                Check("a take-over and its link fit the footer: " + verb + ", " + link, new CommitView(verb, true, "", true, false).ButtonLabel.Length + link.Length <= rowInChars);
        }

        // The band wraps at half of the screen, so a text at scale 1.2 on a 1920 px screen takes one line up to 88 characters and two up to 176.
        string band = FailureBand.Text(new BandFacts("TOUCHDOWN STOPPED: STAGED", EngineNow.StockAuto, FlownBy.AfcPlansStockAuto, 7200.0));
        Check("the longest band takes at most two lines", band.Length <= 176, band.Length.ToString());
        string common = FailureBand.Text(new BandFacts("G-FOLD gave up", EngineNow.Off, FlownBy.You, 31.0));
        Check("a common band takes one line", common == "G-FOLD gave up. ENGINE OFF. MANUAL. IMPACT T-31 s" && common.Length <= 88, common);
        Check("the created burn says what was made", PageText.BurnCreated(915.47, "1h 28m") == "BURN CREATED: 915.5 m/s IN 1h 28m.");
        Check("the created transfer says what was made", PageText.TransferCreated(915.47, "1h 28m") == "TRANSFER CREATED: 915.5 m/s IN 1h 28m.");
        Check("a target at the apsis names the apsis", PageText.AtTargetApse(true).Contains("PE") && PageText.AtTargetApse(false).Contains("AP"));
    }

    private static void VerbGuards()
    {
        var guard = new VerbGuard();
        guard.Update("CREATE BURN", false, 0.0);
        Check("a verb that shows with the mouse elsewhere is not held", guard.MayLight && guard.TakesClicks(0.0));
        guard.Update("CREATE BURN", true, 1.0);
        Check("a verb that stays under the mouse is not held", guard.MayLight && guard.TakesClicks(1.0));

        guard.Update("FLY WITH STOCK AUTO", true, 2.0);
        Check("a new verb under the mouse has no gold", !guard.MayLight);
        Check("a double click does not act on it", !guard.TakesClicks(2.0) && !guard.TakesClicks(2.0 + VerbGuard.ArmDelaySeconds * 0.5));
        Check("a later press acts, so it is no refusal", guard.TakesClicks(2.0 + VerbGuard.ArmDelaySeconds + 0.01));
        guard.Update("FLY WITH STOCK AUTO", true, 3.0);
        Check("it stays an outline while the mouse stays", !guard.MayLight);
        guard.Update("FLY WITH STOCK AUTO", false, 3.1);
        Check("it lights once the mouse has left", guard.MayLight && guard.TakesClicks(3.1));
        guard.Update("FLY WITH STOCK AUTO", true, 3.2);
        Check("the mouse that comes back finds it lit", guard.MayLight && guard.TakesClicks(3.2));

        guard.Update("CREATE BURN", false, 4.0);
        Check("a verb that changes with the mouse elsewhere is not held, as on a new page", guard.MayLight && guard.TakesClicks(4.0));
        guard.Update("CREATE BURN", true, 4.1);
        Check("a mouse that moves onto it at once finds it lit", guard.MayLight && guard.TakesClicks(4.1));
        guard.Update("CREATE BURN", false, 4.2);
        guard.Update("FLY WITH STOCK AUTO", false, 5.0);
        guard.Update("FLY WITH STOCK AUTO", true, 5.1, moved: true);
        Check("a new verb the window brings under the mouse has no gold", !guard.MayLight);
        Check("a double click does not act on that one", !guard.TakesClicks(5.1));
        Check("a later press acts on it", guard.TakesClicks(5.0 + VerbGuard.ArmDelaySeconds + 0.01));
        guard.Update("FLY WITH STOCK AUTO", false, 5.2);
        guard.Update("FLY WITH STOCK AUTO", true, 6.0, moved: true);
        Check("a window that moves later holds nothing", guard.MayLight && guard.TakesClicks(6.0));
        guard.Update("", true, 5.0);
        guard.Update("CREATE TRANSFER", true, 9.0);
        Check("a verb that appears under the waiting mouse is held", !guard.MayLight && !guard.TakesClicks(9.0));
        guard.Reset();
        Check("a reset holds nothing", guard.MayLight && guard.TakesClicks(9.0));
    }

    private static void RowLabelsFit()
    {
        const int roomInChars = 33;
        foreach (SettingSpec s in GuidanceSettingSpecs.AllDoubles)
        {
            string label = SettingMath.DisplayLabel(s);
            Check("number label fits beside its field: " + label, label.Length <= roomInChars);
        }
        Check("a template id loses its family", PartNames.Display("CoreFairingA_Prefab_Interstage3W1HA") == "Interstage 3W1HA", PartNames.Display("CoreFairingA_Prefab_Interstage3W1HA"));
        Check("a template id splits at its words", PartNames.Display("CoreCouplingA_Prefab_DecouplerHalfWB") == "Decoupler Half WB", PartNames.Display("CoreCouplingA_Prefab_DecouplerHalfWB"));
        Check("a real display name stays", PartNames.Display("LR91 Vac + Verniers") == "LR91 Vac + Verniers");
        Check("a run of capitals ends at the word after it", PartNames.Display("X_Prefab_SRBCSegmentThrustAssemblyA") == "SRBC Segment Thrust Assembly A", PartNames.Display("X_Prefab_SRBCSegmentThrustAssemblyA"));
        Check("a lone capital is a word", PartNames.Display("X_Prefab_SetAHeightA") == "Set A Height A", PartNames.Display("X_Prefab_SetAHeightA"));
        Check("a size stays whole", PartNames.Display("X_Prefab_Interstage3WHalfHA") == "Interstage 3W Half HA", PartNames.Display("X_Prefab_Interstage3WHalfHA"));
    }

    private static void TuneRulesCheck()
    {
        Check("a set with changed values says how many", TuneRules.CountText(2) == "2" && TuneRules.CountText(0).Length == 0);
        Check("the sets with values have a count, and the others none", TuneRules.HoldsValues(TuneSet.Ascent) && TuneRules.HoldsValues(TuneSet.Recover) && TuneRules.HoldsValues(TuneSet.Landing) && !TuneRules.HoldsValues(TuneSet.Diag) && !TuneRules.HoldsValues(TuneSet.Afc));
        Check("one changed value has only the reset of its row", !TuneRules.ResetSetShows(0, 0) && !TuneRules.ResetSetShows(1, 1) && TuneRules.ResetSetShows(2, 2));
        Check("one changed value with no reset to press has the reset of the set", TuneRules.ResetSetShows(1, 0));
        Check("two changed values have the reset of the set with any rows", TuneRules.ResetSetShows(2, 0) && TuneRules.ResetSetShows(2, 1));
        Check("the sentence of a limit that only the plan reads names it", PageText.QAlphaNextLaunch.StartsWith(GuidanceSettingSpecs.ConvexQAlphaMax.Label, StringComparison.Ordinal));
        Check("the G-FOLD view says Empty as the other views do", Array.TrueForAll(Enum.GetValues<TuneDiagnostics>(), v => PageText.DiagEmpty(v).StartsWith("Empty until", StringComparison.Ordinal)), PageText.DiagEmpty(TuneDiagnostics.Gfold));
        Check("the editor does not say that TRANSFER works", PageText.FlightOnlyInEditor.Contains("STOCK PLANNER LINK ON TRANSFER") && !PageText.FlightOnlyInEditor.Contains("TRANSFER WORK"));
        Check("the sets have one order everywhere", TuneRules.Order[0] == TuneSet.Ascent && TuneRules.Order[^1] == TuneSet.Afc);
        Check("every set has one tab", new HashSet<TuneSet>(TuneRules.Order).Count == Enum.GetValues<TuneSet>().Length);
        Check("the hairline leaves the tuning sets together", TuneRules.Order[TuneRules.DividerAt] == TuneSet.Diag);
        Check("a dim tab of the editor says why in three words", PageText.TuneSetsNeed(true, true) == "IN FLIGHT ONLY");
        Check("a peak within 5 % of its limit is near it", TuneRules.NearLimit(98.0, 100.0) && TuneRules.NearLimit(95.0, 100.0) && !TuneRules.NearLimit(61.0, 100.0));
        Check("a limit of 0 is no limit", !TuneRules.NearLimit(5.0, 0.0));
        Check("DIAG has no press to speak about", !PageText.PlanOldPlain.Contains("press"));
        Check("the backup row of an old plan says no press", PageText.BackupPlanState(PageText.PlanOld) == PageText.PlanOldPlain);
        Check("the backup row of no plan says no calculation", PageText.BackupPlanState(PageText.NoPlanYet) == PageText.NoPlanYet && !PageText.NoPlanYet.Contains("calculates"));
        Check("a backup row keeps any other reason", PageText.BackupPlanState("NO SOLUTION") == "NO SOLUTION" && PageText.BackupPlanState("") == "");
        Check("a lit engine says that the stock burn flies, whatever the clock says", PageText.StockBurnName(true) == "BURN ACTIVE" && PageText.StockBurnWhen(true, "1m 00s") == "ACTIVE");
        Check("an engine that is off says the wait to its ignition", PageText.StockBurnName(false) == "BURN ARMED" && PageText.StockBurnWhen(false, "1m 00s") == "IN 1m 00s");
        Check("the saved set shows with a craft", TuneRules.Shown(TuneSet.Diag, true, false) == TuneSet.Diag);
        Check("the preferences show without a craft", TuneRules.Shown(TuneSet.Diag, false, false) == TuneSet.Afc);
        Check("a pick holds without a craft", TuneRules.Shown(TuneSet.Diag, false, true) == TuneSet.Diag);
        Check("the preferences have their own word and keep their key", ConsolePages.TuneSetLabel(TuneSet.Afc) == "PREFERENCES" && ConsolePages.EnumKey(TuneSet.Afc) == "AFC");
        Check("the flight plan view keeps a saved key", ConsolePages.EnumFromKey(ConsolePages.EnumKey(TuneDiagnostics.FlightPlan), TuneDiagnostics.Ascent) == TuneDiagnostics.FlightPlan);
        Check("no view is empty with no word", Array.TrueForAll(Enum.GetValues<TuneDiagnostics>(), v => PageText.DiagEmpty(v).Length > 0));

        Check("a footer with nothing to say goes", CommitSpec.None.SaysNothing);
        Check("a footer with a status stays", !CommitSpec.StatusOnly("x").SaysNothing);
        Check("a footer with a verb stays", !new CommitSpec("GO", FlownBy.Afc).SaysNothing);

        Check("the reset asks about the set by name", PageText.ResetSetQuestion(1, "BOOSTBACK") == "Reset 1 BOOSTBACK value to its default?" && PageText.ResetSetQuestion(3, "G-FOLD") == "Reset 3 G-FOLD values to their defaults?");
        Check("the question does not use set in two meanings", !PageText.ResetSetQuestion(2, "HOVER").Contains("this set") && !PageText.ResetFoldQuestion(2).StartsWith("Set", StringComparison.Ordinal));

        Check("a solve message with its count keeps one", PageText.PlanSolve("converged after 165 iterations", 165, 150, 35.4) == "CONVERGED AFTER 165 ITERATIONS (150 ACCEPTED), 35.4 s", PageText.PlanSolve("converged after 165 iterations", 165, 150, 35.4));
        Check("a solve message with no count gets it", PageText.PlanSolve("cancelled", 12, 3, 1.0) == "CANCELLED, 12 ITERATIONS (3 ACCEPTED), 1.0 s", PageText.PlanSolve("cancelled", 12, 3, 1.0));
        Check("each burn has its unit", PageText.PlanBurn(77.4) == "77.4 s" && PageText.PlanIdeal(10381.0) == "10381 m/s IDEAL", PageText.PlanIdeal(10381.0));
        Check("stages read as words", PageText.Stages(1) == "1 STAGE" && PageText.Stages(3) == "3 STAGES");
        Check("no science notation on the mirror check", PageText.MirrorError(0.0) == "0" && PageText.MirrorError(0.001) == "0.1000 %");
        Check("no minus on a zero", Fmt.Fixed(-0.001, 2) == "0.00" && Fmt.Fixed(-2.4, 0) == "-2" && Fmt.Fixed(1.25, 1) == "1.2", Fmt.Fixed(1.25, 1));

        Check("noise on a plot is zero", TuneRules.PlotValue(5.185e-9, 113.0) == "0");
        Check("a small value of a small series stays", TuneRules.PlotValue(-0.0002175, 0.02205) == "-0.000218", TuneRules.PlotValue(-0.0002175, 0.02205));
        Check("a large value has no exponent", TuneRules.PlotValue(36750.0, 36750.0) == "36750");
        Check("a plot value has three digits", TuneRules.PlotValue(63.75, 63.75) == "63.8" && TuneRules.PlotValue(181.0, 181.0) == "181" && TuneRules.PlotValue(0.04833, 4.9) == "0.0483", TuneRules.PlotValue(63.75, 63.75));
        Check("a value with no number reads as a dash", TuneRules.PlotValue(double.NaN, 1.0) == "-");
        Check("a tenth of a power of ten has three digits", TuneRules.PlotValue(-0.001, 0.02) == "-0.00100", TuneRules.PlotValue(-0.001, 0.02));
        Check("two ticks of one series have one form", TuneRules.PlotValue(-0.00162, 0.02) == "-0.00162" && TuneRules.PlotValue(1.0, 1.0) == "1.00" && TuneRules.PlotValue(10.0, 10.0) == "10.0" && TuneRules.PlotValue(100.0, 100.0) == "100", TuneRules.PlotValue(10.0, 10.0));
        Check("a value that rounds up keeps three digits", TuneRules.PlotValue(9.996, 10.0) == "10.0" && TuneRules.PlotValue(999.6, 1000.0) == "1000", TuneRules.PlotValue(9.996, 10.0));

        SettingSpec share = GuidanceSettingSpecs.GfoldThrottleMin;
        Check("a share reads in percent", SettingMath.DisplayLabel(share) == "PLAN MIN THROTTLE (%)" && SettingMath.DefaultValue(share) == "5.0" && SettingMath.GhostText(share) == "DEFAULT 5.0 %", SettingMath.GhostText(share));
        Check("the field of a share goes there and back", Math.Abs(SettingMath.ToShown(share, 0.05) - 5.0) < 1e-9 && Math.Abs(SettingMath.FromShown(share, 5.0) - 0.05) < 1e-12);
        Check("a value with another unit stays as it is", SettingMath.ToShown(GuidanceSettingSpecs.GfoldVMaxMs, 500.0) == 500.0 && SettingMath.FromShown(GuidanceSettingSpecs.GfoldVMaxMs, 500.0) == 500.0 && SettingMath.ShownFormat(GuidanceSettingSpecs.GfoldVMaxMs) == "%.1f");
        foreach (SettingSpec s in GuidanceSettingSpecs.AllDoubles)
            Check("no row shows the word frac: " + s.Key, !SettingMath.DisplayLabel(s).Contains("frac"));

        foreach (SettingSpec kp in new[] { GuidanceSettingSpecs.GfoldKp, GuidanceSettingSpecs.SixDofTrackKp, GuidanceSettingSpecs.ThreeDofTrackKp })
            Check("TRACK KP has one unit: " + kp.Key, kp.Unit == "1/s^2");
        foreach (SettingSpec kd in new[] { GuidanceSettingSpecs.GfoldKd, GuidanceSettingSpecs.SixDofTrackKd, GuidanceSettingSpecs.ThreeDofTrackKd })
            Check("TRACK KD has one unit: " + kd.Key, kd.Unit == "1/s");
        Check("the two smoothings read apart", GuidanceSettingSpecs.GfoldSlewReg.Unit == "weight" && GuidanceSettingSpecs.GfoldSmoothTau.Unit == "s");

        SettingSpec gain = GuidanceSettingSpecs.GlideKp;
        Check("a default stands as the row prints it", SettingMath.DefaultValue(gain) == "15.000", SettingMath.DefaultValue(gain));
        Check("a switch default is a word", SettingMath.DefaultValue(GuidanceSettingSpecs.SixDofTrack) == "ON" && SettingMath.DefaultValue(GuidanceSettingSpecs.SixDofFixedTime) == "OFF");

        Check("one name for a re-solve time", GuidanceSettingSpecs.GfoldIntervalS.Label == GuidanceSettingSpecs.SixDofReplanSec.Label && GuidanceSettingSpecs.GfoldIntervalS.Key == "GfoldIntervalS");
        Check("one name for the track gains", GuidanceSettingSpecs.GfoldKp.Label == GuidanceSettingSpecs.SixDofTrackKp.Label && GuidanceSettingSpecs.ThreeDofTrackKp.Label == GuidanceSettingSpecs.SixDofTrackKp.Label
            && GuidanceSettingSpecs.GfoldKd.Label == GuidanceSettingSpecs.SixDofTrackKd.Label && GuidanceSettingSpecs.ThreeDofTrackKd.Label == GuidanceSettingSpecs.SixDofTrackKd.Label);
        Check("one name for a smoothing weight", GuidanceSettingSpecs.GfoldSlewReg.Label == GuidanceSettingSpecs.SixDofControlSmooth.Label && GuidanceSettingSpecs.SixDofControlSmooth.Unit == GuidanceSettingSpecs.GfoldSlewReg.Unit);
        Check("one name for a smoothing time", GuidanceSettingSpecs.GfoldSmoothTau.Label == GuidanceSettingSpecs.ThreeDofSmoothTau.Label);
        Check("one name for a throttle limit", GuidanceSettingSpecs.GfoldThrottleMax.Label == "PLAN MAX THROTTLE" && GuidanceSettingSpecs.GfoldThrottleMin.Label == "PLAN MIN THROTTLE");
        Check("one name for a tilt limit", GuidanceSettingSpecs.SixDofTiltDeg.Label == GuidanceSettingSpecs.TermMaxTiltDeg.Label && GuidanceSettingSpecs.GfoldPointingDeg.Label == GuidanceSettingSpecs.TermMaxTiltDeg.Label);

        var grouped = new List<string>();
        foreach (SettingSpec[] group in new[] { GuidanceSettingSpecs.TuningSixDofLimits, GuidanceSettingSpecs.TuningSixDofTracking, GuidanceSettingSpecs.TuningSixDofSolver })
            foreach (SettingSpec s in group)
                grouped.Add(s.Key);
        var whole = new List<string>();
        foreach (SettingSpec s in GuidanceSettingSpecs.TuningSixDof)
            whole.Add(s.Key);
        grouped.Sort(StringComparer.Ordinal);
        whole.Sort(StringComparer.Ordinal);
        Check("the groups hold every touchdown value once", grouped.SequenceEqual(whole), string.Join(",", grouped));
        var groupedBools = new List<string>();
        foreach (BoolSpec[] group in new[] { GuidanceSettingSpecs.TuningSixDofLimitBools, GuidanceSettingSpecs.TuningSixDofTrackingBools, GuidanceSettingSpecs.TuningSixDofSolverBools })
            foreach (BoolSpec s in group)
                groupedBools.Add(s.Key);
        var wholeBools = new List<string>();
        foreach (BoolSpec s in GuidanceSettingSpecs.TuningSixDofBools)
            wholeBools.Add(s.Key);
        groupedBools.Sort(StringComparer.Ordinal);
        wholeBools.Sort(StringComparer.Ordinal);
        Check("the groups hold every touchdown switch once", groupedBools.SequenceEqual(wholeBools), string.Join(",", groupedBools));
        var gfoldGrouped = new List<string>();
        foreach (SettingSpec[] group in new[] { GuidanceSettingSpecs.TuningGfoldLimits, GuidanceSettingSpecs.TuningGfoldTracking, GuidanceSettingSpecs.TuningGfoldSolver })
            foreach (SettingSpec s in group)
                gfoldGrouped.Add(s.Key);
        var gfoldWhole = new List<string>();
        foreach (SettingSpec s in GuidanceSettingSpecs.TuningGfold)
            gfoldWhole.Add(s.Key);
        gfoldGrouped.Sort(StringComparer.Ordinal);
        gfoldWhole.Sort(StringComparer.Ordinal);
        Check("the groups hold every G-FOLD value once", gfoldGrouped.SequenceEqual(gfoldWhole), string.Join(",", gfoldGrouped));
        Check("a value of two landing parts counts once on the tab", new HashSet<SettingSpec>(GuidanceSettingSpecs.TuningLanding).Count == GuidanceSettingSpecs.TuningLanding.Length
            && Array.IndexOf(GuidanceSettingSpecs.TuningLanding, GuidanceSettingSpecs.LandingPlanFloorMargin) >= 0);
    }

    private static void ColorRulesCheck()
    {
        Check("stock is far from the AFC blue", ColorRules.Distance(ColorRules.Stock, ColorRules.Afc) > 120.0);
        Check("stock is far from the gold of the verb", ColorRules.Distance(ColorRules.Stock, ColorRules.Gold) > 80.0);
        Check("stock is far from the warning orange", ColorRules.Distance(ColorRules.Stock, ColorRules.Warn) > 100.0);
        Check("stock is far from the failure red", ColorRules.Distance(ColorRules.Stock, ColorRules.Bad) > 120.0);
        Check("stock is far from the green of a good word", ColorRules.Distance(ColorRules.Stock, ColorRules.Good) > 100.0);
        Check("stock is muted, the plates of stock gauges and no state", ColorRules.Saturation(ColorRules.Stock) < 0.25 && ColorRules.Saturation(ColorRules.Afc) > 0.5 && ColorRules.Saturation(ColorRules.Gold) > 0.5);
        Check("the letters on a stock chip read", ColorRules.InkOn(ColorRules.Stock) == ColorRules.Panel && ColorRules.Contrast(ColorRules.Stock, ColorRules.Panel) >= 7.0);
        Check("the letters on a dim stock chip read", ColorRules.Contrast(ColorRules.StockDim, ColorRules.Panel) >= 4.5);
        Check("the soft warning reads on the chosen band", ColorRules.Contrast(ColorRules.WarnOnBand, ColorRules.ChosenBand) >= 4.5);
        Check("the crash reads on the chosen band", ColorRules.Contrast(ColorRules.BadOnBand, ColorRules.ChosenBand) >= 4.5);
        Check("the dim method word reads on the chosen band", ColorRules.Contrast(ColorRules.DimOnBand, ColorRules.ChosenBand) >= 4.5);
        Check("the method word reads on the chosen band", ColorRules.Contrast(ColorRules.Afc, ColorRules.ChosenBand) >= 4.5);
        Check("the full warning reads on the chosen band", ColorRules.Contrast(ColorRules.Warn, ColorRules.ChosenBand) >= 4.5);
        Check("the soft warning stays softer than the full one", ColorRules.Contrast(ColorRules.WarnOnBand, ColorRules.ChosenBand) < ColorRules.Contrast(ColorRules.Warn, ColorRules.ChosenBand));
        Check("the letters on an AFC chip read", ColorRules.InkOn(ColorRules.Afc) == ColorRules.Panel && ColorRules.Contrast(ColorRules.Afc, ColorRules.Panel) >= 7.0);
        Check("stock ink reads on the panel", ColorRules.Contrast(ColorRules.Stock, ColorRules.Panel) >= 4.5);

        Check("a changed value reads", ColorRules.Contrast(ColorRules.Changed, ColorRules.Panel) >= 7.0);
        Check("a changed value has a colour", ColorRules.Saturation(ColorRules.Changed) > 0.3);
        Check("a changed value is far from the AFC blue", ColorRules.Distance(ColorRules.Changed, ColorRules.Afc) > 60.0);
        Check("a changed value is far from the good green", ColorRules.Distance(ColorRules.Changed, ColorRules.Good) > 60.0);
        Check("a changed value is far from the gold, the stock plate and the bright text", ColorRules.Distance(ColorRules.Changed, ColorRules.Gold) > 100.0 && ColorRules.Distance(ColorRules.Changed, ColorRules.Stock) > 60.0 && ColorRules.Distance(ColorRules.Changed, ColorRules.Bright) > 60.0);

        Check("the dim warning reads", ColorRules.Contrast(ColorRules.WarnDim, ColorRules.Panel) >= 4.5);
        Check("the dim warning is dimmer than the warning", ColorRules.Luminance(ColorRules.WarnDim) < ColorRules.Luminance(ColorRules.Warn));
        Check("the dim warning keeps the hue of the warning", Math.Abs(ColorRules.Hue(ColorRules.WarnDim) - ColorRules.Hue(ColorRules.Warn)) < 6.0);
        Check("the dim crash reads", ColorRules.Contrast(ColorRules.BadDim, ColorRules.Panel) >= 4.5);
        Check("the dim crash is dimmer than the crash", ColorRules.Luminance(ColorRules.BadDim) < ColorRules.Luminance(ColorRules.Bad));

        bool inkReads = true, clear = true, apart = true;
        Rgb8[] jobs = { ColorRules.Afc, ColorRules.Gold, ColorRules.Warn, ColorRules.Bad, ColorRules.Good, ColorRules.Target, ColorRules.Stock };
        for (int i = 0; i < ColorRules.Stages.Length; i++)
        {
            Rgb8 stage = ColorRules.Stages[i];
            inkReads &= ColorRules.Contrast(stage, ColorRules.InkOn(stage)) >= 4.5;
            foreach (Rgb8 job in jobs)
                clear &= ColorRules.Distance(stage, job) > 45.0;
            apart &= ColorRules.Distance(stage, ColorRules.Stage(i + 1)) > 45.0;
        }
        Check("the text in a stage box reads", inkReads);
        Check("a stage hue keeps clear of every colour with a job", clear);
        Check("two stages side by side can be told apart", apart);
        Check("the stage colours cycle", ColorRules.Stage(0) == ColorRules.Stage(ColorRules.Stages.Length) && ColorRules.Stage(-1) == ColorRules.Stages[^1]);

        ReadOnlySpan<int> stack = [1, 1, 2, 5];
        int[] onPad = StageNumbers.Of(stack, stack);
        Check("the box of a split sequence takes the hues of both rows", StageNumbers.HuesOf(stack, onPad, [], 1) == new StageHueRange(0, 2, 1) && StageNumbers.HuesOf(stack, onPad, [], 1).TwoHues);
        Check("the next box takes the hue of its block on the bar", StageNumbers.HuesOf(stack, onPad, [], 2) == new StageHueRange(2, 1, 2) && StageNumbers.HuesOf(stack, onPad, [], 5) == new StageHueRange(3, 1, 3));
        ReadOnlySpan<int> splitInFlight = [2, 2, 5];
        StageHueRange liveSplit = StageNumbers.HuesOf(splitInFlight, StageNumbers.Of([1, 2, 5], splitInFlight), [], 2);
        Check("a live g-limit split of a kept stage has one hue, not the hue of the next stage", liveSplit == new StageHueRange(1, 2, 1) && !liveSplit.TwoHues);
        ReadOnlySpan<int> splitLast = [2, 5, 5];
        int[] splitLastNumbers = StageNumbers.Of(stack, splitLast);
        Check("the rows of the bar and the box agree on the last stage", splitLastNumbers[1] == 3 && splitLastNumbers[2] == 3 && !StageNumbers.HuesOf(splitLast, splitLastNumbers, [], 5).TwoHues);
        Check("a sequence with no engine stays neutral", !StageNumbers.HuesOf(stack, onPad, [1, 2, 5], 3).Any && StageHueRange.None.Count == 0);
        int[] renumbered = StageNumbers.Of(stack, [1]);
        Check("the last stage keeps its hue when stock renumbers it", StageNumbers.HuesOf([1], renumbered, [1], 1) == new StageHueRange(3, 1, 3));
        Check("with no stage list a sequence counts among those with an engine", StageNumbers.HuesOf([], [], [1, 2, 5], 2) == new StageHueRange(1, 1, 1));
        Check("a sequence off the list takes the first free hue", StageNumbers.HuesOf(stack, onPad, [1, 2, 5, 7], 7) == new StageHueRange(4, 1, 4));
        Check("two sequences off the list have a hue each", StageNumbers.HuesOf(stack, onPad, [1, 7, 2, 9, 5], 9) == new StageHueRange(5, 1, 5));
        bool offListClear = true;
        for (int i = 0; i < stack.Length; i++)
            offListClear &= ColorRules.Stage(StageNumbers.HuesOf(stack, onPad, [1, 2, 5, 7], 7).First) != ColorRules.Stage(onPad[i]);
        Check("a sequence off the list does not take the colour of a stage on it", offListClear);

        Check("no stack is no stack to keep", !StageNumbers.KeepsStack(false, true, true, false, true, true, true));
        Check("a list longer than the stack belongs to another stack", !StageNumbers.KeepsStack(true, false, true, true, true, true, true));
        Check("a flight keeps its stack", StageNumbers.KeepsStack(true, true, true, false, true, false, false));
        Check("after the flight the plan keeps the stack", StageNumbers.KeepsStack(true, true, false, true, true, true, true));
        Check("a plan calculated again takes the stack of now", !StageNumbers.KeepsStack(true, true, false, true, true, false, true));
        Check("a plan cleared takes the stack of now", !StageNumbers.KeepsStack(true, true, false, true, false, false, true));
        Check("a flight with no plan keeps its stack after it", StageNumbers.KeepsStack(true, true, false, true, false, true, false));
        Check("with no plan and no flight the stack is what the craft has now", !StageNumbers.KeepsStack(true, true, false, false, false, true, false));

        Check("the chosen row is bright, also where it cannot fly", ColorRules.RowToneOf(true, true) == RowTone.Bright && ColorRules.RowToneOf(true, false) == RowTone.Bright);
        Check("a row that cannot fly is dim unless chosen", ColorRules.RowToneOf(false, true) == RowTone.Dim && ColorRules.RowToneOf(false, false) == RowTone.Normal);
        var marks = new List<TableMark>();
        BurnList.MarksOf(0, true, rcs: false, fuelShort: true, marks);
        BurnList.MarksOf(1, false, rcs: true, fuelShort: true, marks);
        BurnList.MarksOf(2, false, rcs: false, fuelShort: false, marks);
        Check("who flies a burn has the colour of the owner", marks.Contains(new TableMark(0, 3, TableInk.Stock)) && marks.Contains(new TableMark(1, 3, TableInk.Afc)));
        Check("fuel that falls short warns, full on the chosen row and dim on another", marks.Contains(new TableMark(0, 4, TableInk.Warn)) && marks.Contains(new TableMark(1, 4, TableInk.WarnDim)));
        Check("enough fuel has no colour", !marks.Exists(m => m.Row == 2 && m.Column == 4));
        Check("the column says who flies at the burn", BurnList.Heads[3] == "BY");

        Check("on plan and on target are good", PlanWords.OnPlan.Good && PlanWords.OnTarget.Good);
        Check("the other words are not good", !PlanWords.Turning.Good && !PlanWords.Planning.Good && !PlanWords.LastPlan.Good && !PlanWords.OffTarget.Good && !PlanWords.NoPlan.Good);
        Check("a mode that flies confirms a miss on target", ColorRules.MissTone(0.3, true, false, PlanWords.OnTargetKm) == PlanTone.Good);
        Check("an idle page confirms nothing", ColorRules.MissTone(0.3, false, false, PlanWords.OnTargetKm) == PlanTone.Plain);
        Check("a far miss warns on any page", ColorRules.MissTone(1425.0, false, true, PlanWords.OnTargetKm) == PlanTone.Warn && ColorRules.MissTone(1425.0, true, true, PlanWords.OnTargetKm) == PlanTone.Warn);
        Check("a miss in the middle is plain", ColorRules.MissTone(74.9, true, false, PlanWords.OnTargetKm) == PlanTone.Plain);
        Check("no miss has no tone", ColorRules.MissTone(double.NaN, true, false, PlanWords.OnTargetKm) == PlanTone.Plain);
        Check("the miss of a flying mode has its tone", PlanWords.MissTone(0.5, true) == PlanTone.Good && PlanWords.MissTone(74.9, true) == PlanTone.Plain && PlanWords.MissTone(500.0, true) == PlanTone.Warn);
        Check("the head colours the miss of a glide", PageText.GlideNumbers(100.0, 100.0)[0].Tone == PlanTone.Good && PageText.GlideNumbers(50000.0, 0.0)[0].Tone == PlanTone.Plain);
        CommitSpec ready = new("LAUNCH", FlownBy.Afc, Status: PageText.LaunchAnswer("", "", true, true, 16200.0, 4, 4));
        Check("a plan that can fly has its good word", ready.GoodWord == "PLAN READY" && ready.AnswerStatus.StartsWith(ready.GoodWord));
        Check("a plan that does not fit has no good word", (ready with { Caution = "Not now." }).GoodWord.Length == 0 && (ready with { Refusal = "No." }).GoodWord.Length == 0);
        Check("the other states of a plan have no good word", new CommitSpec("LAUNCH", FlownBy.Afc, Status: PageText.PlanOld).GoodWord.Length == 0 && new CommitSpec("LAUNCH", FlownBy.Afc, Status: "NO PLAN").GoodWord.Length == 0);
        Check("the goal reached is good", PageText.GoalReached(200.0, 422.0, 200.0, 420.0));
        Check("a goal missed is not", !PageText.GoalReached(180.0, 422.0, 200.0, 420.0) && !PageText.GoalReached(-5.0, 420.0, 200.0, 420.0) && !PageText.GoalReached(double.NaN, 420.0, 200.0, 420.0));
        Check("the good word is the first word of the line", PageText.FlownAgainstGoal(200.0, 422.0, 200.0, 420.0).StartsWith(PageText.FlownWord + " "));

        Check("the mark of AFC starts at its word", ColorRules.AfcWordAt("ON PRESS: AFC FLIES") == 10);
        Check("the end of a mark of a deorbit is AFC", ColorRules.AfcWordAt(", THEN AFC") == 7);
        Check("a mark with no AFC stays plain", ColorRules.AfcWordAt("EXEC:") < 0);

        Check("a periapsis under the top of the air is in the air", SketchMath.PeInAir(150000.0, 165000.0));
        Check("a periapsis over the top is not", !SketchMath.PeInAir(200000.0, 165000.0));
        Check("a body with no air has none", !SketchMath.PeInAir(10.0, 0.0));
        Check("a periapsis under the ground is a fall", !SketchMath.PeInAir(-5.0, 165000.0));
    }

    private static void StagingRulesCheck()
    {
        Check("the total has its unit once and no tilde", StagingRules.Total(11702.4, 9) == "11702 m/s, 9 SEQUENCES", StagingRules.Total(11702.4, 9));
        Check("one sequence is singular", StagingRules.Total(1978.0, 1) == "1978 m/s, 1 SEQUENCE");
        Check("a sequence still to fire has a box", StagingRules.HasBox(false, 2, 1, 0.0));
        Check("the sequence that burns has a box while it holds delta-v", StagingRules.HasBox(true, 1, 1, 1978.0));
        Check("a spent active sequence has none", !StagingRules.HasBox(true, 1, 1, 0.0));
        Check("an earlier sequence has none", !StagingRules.HasBox(true, 1, 2, 500.0));
        Check("a box has one form", StagingRules.BoxLabel(5, false, true) == "SEQ 5" && StagingRules.BoxLabel(5, false, false) == "5");
        Check("the next box has the stock mark", StagingRules.BoxLabel(2, true, true) == "> SEQ 2" && StagingRules.BoxLabel(2, true, false) == "> 2");
        Check("the box that burns says so", StagingRules.BoxLabel(1, false, false, burns: true) == "1 BURNS" && StagingRules.BoxLabel(1, false, true, burns: true) == "SEQ 1 BURNS");
        Check("a box with delta-v takes a colour, and a spent one does not", StagingRules.HoldsDv(1163.0) && !StagingRules.HoldsDv(0.0));
        Check("a delay has a word and a unit", StagingRules.Delay(StagingRules.IgnitionWord, 2.0) == "IGNITION +2.0 s" && StagingRules.Delay("", 3.0) == "+3.0 s");
        Check("no code of a delay is left", !StagingRules.Delay(StagingRules.DecouplerWord, 1.0).Contains("D+") && !StagingRules.Delay(StagingRules.IgnitionWord, 1.0).Contains("I+"));
        string card = StagingRules.Card(5, 5799.0, false, true, 3.0, 0.0);
        Check("the card of a box says what the short form leaves out", card.Contains("SEQ 5") && card.Contains("5799 m/s") && card.Contains("Fires next") && card.Contains("light 3.0 s") && !card.Contains("decouplers"), card);
        Check("the card of the burning box says so", StagingRules.Card(1, 1978.0, true, false, 0.0, 0.0).Contains("Burns now"));
        Check("a sequence that burns has no delay under its box, and one still to fire has", !StagingRules.DelayShows(true, 3.0) && StagingRules.DelayShows(false, 3.0) && !StagingRules.DelayShows(false, 0.0));
        Check("a box with delta-v says it, and any other box a dash", StagingRules.BoxDv(1949.4) == "1949" && StagingRules.BoxDv(0.0) == "-");
        Check("the footer joins off and the launch", PageText.AutoStageOffUntilLaunch.Contains("AutoStage") && PageText.AutoStageOffUntilLaunch.Contains("lift-off"));
        Check("the note of a switch with no effect is one short line", PageText.DropSpentNeedsAutoStage == "Acts only with AutoStage on.");
        Check("the list of all parts counts them", PageText.AllPartsLabel(56) == "ALL PARTS: 56" && FoldRules.Split(PageText.AllPartsLabel(56)).Name == "ALL PARTS");
    }

    private static void Formats()
    {
        Check("time below two minutes is seconds", Fmt.Time(45.4) == "45 s", Fmt.Time(45.4));
        Check("time up to 120 s stays in seconds", Fmt.Time(119.4) == "119 s", Fmt.Time(119.4));
        Check("time from 120 s has no count of seconds", Fmt.Time(139.0) == "2m 19s", Fmt.Time(139.0));
        Check("time on a whole minute keeps the seconds", Fmt.Time(180.0) == "3m 00s", Fmt.Time(180.0));
        Check("time in hours", Fmt.Time(5280.0) == "1h 28m", Fmt.Time(5280.0));
        Check("time in days", Fmt.Time(267.0 * 86400.0 + 6.0 * 3600.0) == "267d 6h", Fmt.Time(267.0 * 86400.0 + 6.0 * 3600.0));
        Check("negative time keeps its sign", Fmt.Time(-90.0) == "-90 s", Fmt.Time(-90.0));
        Check("time that is not finite", Fmt.Time(double.NaN) == "N/A" && Fmt.Time(double.PositiveInfinity) == "N/A");
        Check("fine time has a tenth below two minutes", Fmt.TimeFine(8.54) == "8.5 s", Fmt.TimeFine(8.54));
        Check("fine time above two minutes is the plain time", Fmt.TimeFine(5280.0) == "1h 28m", Fmt.TimeFine(5280.0));
        Check("delta-v has one decimal", Fmt.Dv(915.47) == "915.5 m/s", Fmt.Dv(915.47));
        Check("a budget of delta-v is whole", Fmt.DvWhole(1991.4) == "1991 m/s", Fmt.DvWhole(1991.4));
        Check("distance in meters below 10 km", Fmt.Distance(8620.0) == "8620 m", Fmt.Distance(8620.0));
        Check("distance in km from 10 km", Fmt.Distance(252400.0) == "252.4 km", Fmt.Distance(252400.0));
        Check("orbit pair", Fmt.Orbit(200.2, 419.6) == "200 x 420 km", Fmt.Orbit(200.2, 419.6));
        Check("angle", Fmt.Deg(51.6) == "51.6 deg" && Fmt.Percent(0.904) == "90 %" && Fmt.Kpa(35400.0) == "35.4 kPa" && Fmt.Mn(1.25e6) == "1.25 MN");
        Check("mass", Fmt.Tons(16240.0) == "16.2 t" && Fmt.Kg(12.4) == "12 kg" && Fmt.Ratio(1.5) == "1.50");

        System.Globalization.CultureInfo saved = System.Globalization.CultureInfo.CurrentCulture;
        try
        {
            System.Globalization.CultureInfo.CurrentCulture = new System.Globalization.CultureInfo("de-DE");
            Check("the decimal point does not follow the culture", Fmt.Dv(1.5) == "1.5 m/s" && Fmt.Percent(0.5) == "50 %" && Fmt.TimeFine(1.5) == "1.5 s");
            Check("a sentence of a page does not follow the culture", PageText.BurnAnswer(1.5, "IN 1m", 100.5) == "1.5 m/s IN 1m, 1 % OF ~100 m/s"
                && PageText.HoverSetpoint(1.5, 0.0, 0.0).StartsWith("E 1.5"));
        }
        finally
        {
            System.Globalization.CultureInfo.CurrentCulture = saved;
        }

        Check("sentence capitalizes a lower case status", Fmt.Sentence("converging...") == "Converging...");
        Check("sentence leaves a capital or a number", Fmt.Sentence("G-FOLD found") == "G-FOLD found" && Fmt.Sentence("3 stages") == "3 stages" && Fmt.Sentence("") == "");
    }

    private static void FlightViewRules()
    {
        GuidanceModes.Register(EntryGuidance.Descriptor());
        var ascent = new CraftActivity { AscentRunning = true, ControlAcquired = true };
        var armed = new CraftActivity { LaunchArmed = true };
        var pending = new CraftActivity { ConvexLaunchPending = true };
        var boostback = new CraftActivity { BoostbackLive = true, ControlAcquired = true };
        var entry = new CraftActivity { RegisteredModeLive = GuidanceModeId.EntryHere, ControlAcquired = true };
        var touchdown = new CraftActivity { SixDofEngaged = true, ControlAcquired = true };
        var deorbit = new CraftActivity { DescentLive = true };
        var gfold = new CraftActivity { DescentLive = true, LandingMachineLive = true, LandingAirborne = true, ControlAcquired = true };
        var hover = new CraftActivity { DescentLive = true, HoverLive = true, LandingAirborne = true, ControlAcquired = true };
        var passes = new CraftActivity { MultiPassActive = true };
        var nothing = new CraftActivity();
        CraftActivity[] all = { ascent, armed, pending, boostback, entry, touchdown, deorbit, gfold, hover, passes, nothing };

        var open = new CommitGates(false, false, "", "", false);
        foreach (GuidanceModeId mode in Enum.GetValues<GuidanceModeId>())
        {
            if (mode == GuidanceModeId.AscentGravityTurn)
                continue;
            foreach (CraftActivity a in all)
                Check("a mode flies where its verb goes " + mode, FlightView.ModeFlies(mode, in a) == CommitResolver.Resolve(mode, in a, in open).Live);
        }

        bool Flies(ConsolePage page, LandWay way, LandSolver solver, in CraftActivity a, bool transferPasses = false)
            => FlightView.PageFlies(page, way, solver, AscentChoice.CatsUpfg, in a, transferPasses);

        Check("LAUNCH flies with a running ascent", Flies(ConsolePage.Launch, LandWay.FromOrbit, LandSolver.Entry, in ascent));
        Check("LAUNCH flies with an armed launch and with one that waits on its plan", Flies(ConsolePage.Launch, LandWay.FromOrbit, LandSolver.Entry, in armed)
            && Flies(ConsolePage.Launch, LandWay.FromOrbit, LandSolver.Entry, in pending));
        Check("LAND stays a form during an ascent, for a take-over", !Flies(ConsolePage.Land, LandWay.FromHere, LandSolver.Touchdown, in ascent));
        Check("LAUNCH stays a form during a landing", !Flies(ConsolePage.Launch, LandWay.FromOrbit, LandSolver.Entry, in gfold));
        Check("BOOSTBACK flies only on its way", Flies(ConsolePage.Land, LandWay.Boostback, LandSolver.Entry, in boostback)
            && !Flies(ConsolePage.Land, LandWay.FromHere, LandSolver.Entry, in boostback) && !Flies(ConsolePage.Land, LandWay.Hover, LandSolver.Entry, in boostback));
        Check("ENTRY flies on LAND NOW with its solver", Flies(ConsolePage.Land, LandWay.FromHere, LandSolver.Entry, in entry)
            && !Flies(ConsolePage.Land, LandWay.FromHere, LandSolver.Gfold, in entry));
        Check("TOUCHDOWN flies on LAND NOW with its solver", Flies(ConsolePage.Land, LandWay.FromHere, LandSolver.Touchdown, in touchdown)
            && !Flies(ConsolePage.Land, LandWay.FromHere, LandSolver.Entry, in touchdown));
        Check("G-FOLD flies on LAND NOW and in its deorbit", Flies(ConsolePage.Land, LandWay.FromHere, LandSolver.Gfold, in gfold)
            && Flies(ConsolePage.Land, LandWay.FromOrbit, LandSolver.Gfold, in gfold));
        Check("ENTRY stays a form during G-FOLD, for its take-over", !Flies(ConsolePage.Land, LandWay.FromHere, LandSolver.Entry, in gfold));
        Check("the deorbit chain flies on DEORBIT, and LAND NOW stays a take-over", Flies(ConsolePage.Land, LandWay.FromOrbit, LandSolver.Gfold, in deorbit)
            && !Flies(ConsolePage.Land, LandWay.FromHere, LandSolver.Gfold, in deorbit));
        Check("HOVER flies on its way, and G-FOLD stays a take-over", Flies(ConsolePage.Land, LandWay.Hover, LandSolver.Gfold, in hover)
            && !Flies(ConsolePage.Land, LandWay.FromHere, LandSolver.Gfold, in hover));
        Check("passes of a tool fly on MANEUVER", Flies(ConsolePage.Orbit, LandWay.FromOrbit, LandSolver.Entry, in passes, transferPasses: false)
            && !Flies(ConsolePage.Transfer, LandWay.FromOrbit, LandSolver.Entry, in passes, transferPasses: false));
        Check("passes of a transfer fly on TRANSFER", Flies(ConsolePage.Transfer, LandWay.FromOrbit, LandSolver.Entry, in passes, transferPasses: true)
            && !Flies(ConsolePage.Orbit, LandWay.FromOrbit, LandSolver.Entry, in passes, transferPasses: true));
        foreach (ConsolePage page in ConsolePages.All)
        {
            foreach (LandWay way in LandWays.DisplayOrder)
                Check("a hand-back ends the view at once " + page + " " + way, !Flies(page, way, LandSolver.Gfold, in nothing));
            if (page is ConsolePage.Rcs or ConsolePage.Staging or ConsolePage.Tune)
                foreach (CraftActivity a in all)
                    Check("this page stays what it is " + page, !Flies(page, LandWay.FromHere, LandSolver.Gfold, in a));
        }

        Check("the way of what flies", FlightView.FlyingWay(in boostback) == LandWay.Boostback && FlightView.FlyingWay(in entry) == LandWay.FromHere
            && FlightView.FlyingWay(in touchdown) == LandWay.FromHere && FlightView.FlyingWay(in gfold) == LandWay.FromHere
            && FlightView.FlyingWay(in hover) == LandWay.Hover && FlightView.FlyingWay(in deorbit) == LandWay.FromOrbit);
        Check("no way for what is not a landing", FlightView.FlyingWay(in ascent) == null && FlightView.FlyingWay(in passes) == null && FlightView.FlyingWay(in nothing) == null);
        foreach (CraftActivity a in all)
            if (FlightView.FlyingWay(in a) is LandWay flyingWay)
                Check("the way of what flies is a page that flies", Flies(ConsolePage.Land, flyingWay, a.SixDofEngaged ? LandSolver.Touchdown : a.RegisteredModeLive != null ? LandSolver.Entry : LandSolver.Gfold, in a));

        Check("a flying page has no footer", FlightView.FooterGoes(true, false));
        Check("a slot of its own keeps it", !FlightView.FooterGoes(true, true));
        Check("a form keeps its footer", !FlightView.FooterGoes(false, false));

        Check("a boostback, entry, touchdown and G-FOLD read a new site", FlightView.ReadsNewSite(LandWay.Boostback, in boostback, false)
            && FlightView.ReadsNewSite(LandWay.FromHere, in entry, false) && FlightView.ReadsNewSite(LandWay.FromHere, in touchdown, false) && FlightView.ReadsNewSite(LandWay.FromHere, in gfold, false));
        Check("the hover does not aim at the site", !FlightView.ReadsNewSite(LandWay.Hover, in hover, false));
        Check("a deorbit chain does not offer it", !FlightView.ReadsNewSite(LandWay.FromOrbit, in deorbit, false));
        Check("the powered end of a deorbit does", FlightView.ReadsNewSite(LandWay.FromOrbit, in gfold, false));
        Check("a locked site is no offer", !FlightView.ReadsNewSite(LandWay.FromHere, in gfold, true));

        Check("a plain PLAN", FlightView.PlanLabel("") == "PLAN");
        Check("a climb names its goal", FlightView.PlanLabel(FlightView.LaunchReads(200.0, 420.0, 51.6, false, true)) == "PLAN: 200 x 420 km, 51.6 deg");
        Check("an armed launch reads its warp", FlightView.LaunchReads(200.0, 420.0, 51.6, true, true) == "200 x 420 km, 51.6 deg, AUTO WARP ON");

        Check("the PLAN row of a landing gives the site only", FlightView.LandReads(LandWay.Boostback, false, 28.6, -80.6) == "SITE 28.600, -80.600");
        Check("a deorbit holds its site", FlightView.LandReads(LandWay.FromOrbit, true, 28.6, -80.6) == "SITE LOCKED");
        Check("the hover says what its form holds", FlightView.LandReads(LandWay.Hover, false, 28.6, -80.6) == "Mode, site, hover limits");
        Check("the label fits beside PICK SITE", FlightView.PlanLabel(FlightView.LandReads(LandWay.FromHere, false, -89.123, -179.123)).Length <= 39);
        Check("the form of a climb says what the flight reads", FlightView.LaunchPlanLine(true, false).Contains("PE, AP, INC") && FlightView.LaunchPlanLine(true, false).Contains("next launch"));
        Check("the form of a hover says what the flight reads", FlightView.LandPlanLine(LandWay.Hover, LandSolver.Gfold, false).Contains("limits"));
        Check("a deorbit says that it holds its site", FlightView.LandPlanLine(LandWay.FromHere, LandSolver.Gfold, true).Contains("DEORBIT ACTIVE"));
        foreach (LandSolver solver in new[] { LandSolver.Entry, LandSolver.Touchdown, LandSolver.Gfold })
            Check("each mode names the site it reads " + solver, FlightView.LandPlanLine(LandWay.FromHere, solver, false).Contains("SITE"));

        var lines = new List<StatusLine> { new(Severity.Info, "Cutoff - miss 0.4 km.", "") };
        Check("a plain note takes no row while a mode flies", StatusSelector.PickRow(lines, true) == -1);
        Check("the same note takes the row once nothing flies", StatusSelector.PickRow(lines, false) == 0);
        Check("the drawer counts a note no row shows", StatusSelector.MoreCount(lines, rowShown: false) == 1 && StatusSelector.MoreCount(lines, rowShown: true) == 0);
        lines.Add(new StatusLine(Severity.Warning, "PLANNING WAITS: WARP ABOVE 1x.", ""));
        Check("a warning takes the row while a mode flies", StatusSelector.PickRow(lines, true) == 1);
        Check("a cue takes the row while a mode flies", StatusSelector.PickRow(new List<StatusLine> { new(Severity.Cue, "AFC: HOVER", "LAND/HOVER") }, true) == 0);
        Check("an empty list has no row", StatusSelector.PickRow(new List<StatusLine>(), true) == -1);
        Check("a note about a commit takes the row in the mode it started", StatusSelector.PickRow(new List<StatusLine> { new(StatusSelector.CommitNote, "Landing turned on AFC MAY STEER and AFC MAY STAGE.", "") }, true) == 0);
        Check("a flying page does not ask for a verb it lacks", !PageText.PorkchopPassesFly.Contains("SOLVE"));

        var coast = new SituationFacts(CraftSituation.CoastingUp, "Earth", -812.0, 192.0, true);
        Check("the place keeps the apoapsis of a coast", SituationText.Place(in coast) == "ASCENT, Earth, AP 192 km");
        Check("numbers that hold the orbit take it from the place", SituationText.Place(in coast, orbitShown: true) == "ASCENT, Earth");
        var round = new SituationFacts(CraftSituation.Orbit, "Earth", 200.0, 423.0);
        Check("the same in orbit", SituationText.Place(in round, orbitShown: true) == "ORBIT, Earth");

        var burnPanel = new DockRect(3165f, 857f, 660f, 400f);
        (float floor, float limit) = DockLayout.FlyingHeightRange(1518f, 60f, 3168f, 3792f, 1.2f, burnPanel);
        Check("a flying page stays clear of the burn panel", limit <= 790f && limit > 700f, limit.ToString());
        Check("a flying page has no tall floor", floor <= 240.01f, floor.ToString());
        (_, limit) = DockLayout.FlyingHeightRange(1518f, 60f, 3168f, 3792f, 1.2f, null);
        Check("with the form open it may grow", Math.Abs(limit - 1518f) < 0.01f);
        (_, limit) = DockLayout.FlyingHeightRange(1518f, 60f, 200f, 824f, 1.2f, burnPanel);
        Check("a window beside the panel keeps its limit", Math.Abs(limit - 1518f) < 0.01f);
        (_, limit) = DockLayout.FlyingHeightRange(500f, 60f, 3168f, 3792f, 1.2f, burnPanel);
        Check("the height of the pilot still is the maximum", Math.Abs(limit - 500f) < 0.01f);
        Check("a picture gives up its height first", DockLayout.NextPictureShrink(0f, 40f, 584f, 1.2f) > 0f);
        (floor, limit) = DockLayout.FlyingHeightRange(1518f, 700f, 3168f, 3792f, 1.2f, burnPanel, rows: 460f);
        Check("a window close over the panel keeps its rows", Math.Abs(floor - 460f) < 0.01f && Math.Abs(limit - 460f) < 0.01f, floor + " " + limit);
        (floor, limit) = DockLayout.FlyingHeightRange(1518f, 60f, 3168f, 3792f, 1.2f, burnPanel, rows: 460f);
        Check("with room over the panel its rows do not move the limit", Math.Abs(floor - 460f) < 0.01f && limit <= 790f && limit > 700f, floor + " " + limit);
        (floor, limit) = DockLayout.FlyingHeightRange(300f, 700f, 3168f, 3792f, 1.2f, burnPanel, rows: 460f);
        Check("the height of the pilot wins over its rows", Math.Abs(floor - 300f) < 0.01f && Math.Abs(limit - 300f) < 0.01f, floor + " " + limit);
        (floor, _) = DockLayout.FlyingHeightRange(1518f, 60f, 3168f, 3792f, 1.2f, null, rows: 100f);
        Check("few rows still get the least height of a window", Math.Abs(floor - 240f) < 0.01f, floor.ToString());

        Check("a pass waits under its own word", PlanVerbs.FlyPass(2) == "FLY PASS 2" && PageText.PassWaits(false) == "EXEC: STOCK AUTO burns it." && PageText.PassWaits(true).Length <= 27);

        Check("BURNS is an instrument while stock AUTO flies", FlightView.PageFlies(ConsolePage.Rcs, LandWay.FromOrbit, LandSolver.Entry, AscentChoice.CatsUpfg, new CraftActivity { StockAutoOnNode = true }, false));
        Check("BURNS is an instrument while RCS flies", FlightView.PageFlies(ConsolePage.Rcs, LandWay.FromOrbit, LandSolver.Entry, AscentChoice.CatsUpfg, new CraftActivity { RcsFlying = true }, false));
        Check("BURNS is a form while a pass waits", !FlightView.PageFlies(ConsolePage.Rcs, LandWay.FromOrbit, LandSolver.Entry, AscentChoice.CatsUpfg, new CraftActivity { MultiPassActive = true }, false));
        Check("the stop of stock AUTO says the landing ends with it", StopResolver.OutcomeShort(StopTarget.StockAuto, new CraftActivity { LandingStockAuto = true }).Contains("LANDING ENDS")
            && StopResolver.EngineOutcome(StopTarget.StockAuto, new CraftActivity { LandingStockAuto = true }).Contains("anding ends"));
        Check("the stop of plain stock AUTO keeps its words", StopResolver.OutcomeShort(StopTarget.StockAuto, new CraftActivity { StockAutoOnNode = true }) == "ENGINE OFF. BURN STAYS.");

        Check("the first burn of the list is the next", BurnList.Label(0, 0) == "NEXT" && BurnList.Label(2, 0) == "3");
        Check("a pass has its number", BurnList.Label(0, 2) == "PASS 2" && BurnList.Label(3, 4) == "PASS 4");
        Check("the time of a burn has no raw seconds", BurnList.In(5220.0, false) == "1h 27m" && BurnList.In(45.0, false) == "45 s");
        Check("a burn at its time flies or is over", BurnList.In(0.0, true) == "NOW" && BurnList.In(-30.0, false) == "PAST");
        Check("who flies a burn", BurnList.Who(false) == "STOCK AUTO" && BurnList.Who(true) == "AFC RCS");
        var burnStages = new[] { new StageFuel(1, 10000.0, 3000.0, 3000.0), new StageFuel(2, 5000.0, 2000.0, 3000.0) };
        BurnPay[] burnPays = BurnBudget.Pay(burnStages, new[] { 500.0, 300.0, 1000.0, 9000.0 });
        Check("a burn names the sequence that pays it", BurnList.EngineFuel(burnStages, in burnPays[0]).StartsWith("SEQ 1, ~"), BurnList.EngineFuel(burnStages, in burnPays[0]));
        Check("a later burn sees what the earlier ones used", BurnList.EngineFuel(burnStages, in burnPays[1]).StartsWith("SEQ 1"), BurnList.EngineFuel(burnStages, in burnPays[1]));
        Check("a burn over two sequences names both", BurnList.EngineFuel(burnStages, in burnPays[2]).StartsWith("SEQ 1-2, ~"), BurnList.EngineFuel(burnStages, in burnPays[2]));
        Check("a burn past the fuel says how far", BurnList.EngineFuel(burnStages, in burnPays[3]).EndsWith(" m/s SHORT"), BurnList.EngineFuel(burnStages, in burnPays[3]));
        Check("a burn with no engine data has no fuel text", BurnList.EngineFuel(Array.Empty<StageFuel>(), default) == "");
        Check("an RCS burn has its estimate", BurnList.RcsFuel(482.2) == "RCS, ~482 kg" && BurnList.RcsFuel(4782.2) == "RCS, ~4.8 t" && BurnList.RcsFuel(double.NaN) == "RCS");
        string[] burnRow = BurnList.Row(0, 0, 5220.0, false, 916.1, false, "Seq 1, ~2920 kg");
        Check("a row has the columns of the heads", burnRow.Length == BurnList.Heads.Length && burnRow[0] == "NEXT" && burnRow[2] == "916.1 m/s" && burnRow[3] == "STOCK AUTO" && burnRow[4] == "Seq 1, ~2920 kg");
        Check("the RCS picture shows while RCS flies", BurnList.RcsPictureShows(true, false, false));
        Check("the RCS picture shows where RCS takes the burn", BurnList.RcsPictureShows(false, true, true));
        Check("no RCS picture in the engine case", !BurnList.RcsPictureShows(false, false, true));
        Check("no RCS picture with no thruster that can translate", !BurnList.RcsPictureShows(false, true, false));
        Check("RCS takes a burn it flies", BurnList.RcsTakes(true, false, false, false));
        Check("RCS takes a burn that resolves to it where a thruster can translate", BurnList.RcsTakes(false, false, true, true));
        Check("stock AUTO takes ENGINE, ELSE RCS with no thruster that can translate", !BurnList.RcsTakes(false, false, true, false));
        Check("RCS ONLY stays with RCS, so its refusal can name the reason", BurnList.RcsTakes(false, true, true, false));
        Check("the engine case stays with stock AUTO", !BurnList.RcsTakes(false, false, false, true));
        Check("a planned burn asks its whole size", BurnList.Charge(600.0, false, 500.0, 500.0) == 500.0);
        Check("a burn that flies asks what it has to go", BurnList.Charge(5.0, true, 500.0, 120.0) == 120.0 && BurnList.Charge(-5.0, true, 500.0, 120.0) == 120.0);
        Check("a burn past its time that does not fly asks nothing", BurnList.Charge(-30.0, false, 800.0, 800.0) == 0.0 && BurnList.Charge(0.0, false, 800.0, 800.0) == 0.0);
        var budgetStages = new[] { new StageFuel(1, 10000.0, 3700.0, 3000.0) };
        BurnPay[] afterPast = BurnBudget.Pay(budgetStages, new[] { BurnList.Charge(-30.0, false, 800.0, 800.0), BurnList.Charge(600.0, false, 500.0, 500.0) });
        Check("a burn past its time leaves the fuel to the next one", afterPast[0].FirstStage == -1 && afterPast[1].ShortMs == 0.0 && afterPast[1].FirstStage == 0);
        Check("a stopped descent keeps its path in the fall and on the ground", FlightView.StoppedPathShows(CraftSituation.FallingNoAir, 60.0) && FlightView.StoppedPathShows(CraftSituation.FallingAir, 60.0) && FlightView.StoppedPathShows(CraftSituation.Ground, 60.0));
        Check("a craft on another flight has no old path", !FlightView.StoppedPathShows(CraftSituation.Orbit, 60.0) && !FlightView.StoppedPathShows(CraftSituation.CoastingUp, 60.0)
            && !FlightView.StoppedPathShows(CraftSituation.Escaping, 60.0) && !FlightView.StoppedPathShows(CraftSituation.Unknown, 60.0));
        Check("an old path goes after a while", !FlightView.StoppedPathShows(CraftSituation.Ground, FlightView.StoppedPathKeepSeconds) && !FlightView.StoppedPathShows(CraftSituation.FallingNoAir, FlightView.StoppedPathKeepSeconds + 1.0));
        Check("an empty list leads to the page that makes a first burn", BurnList.EmptyNext(CraftSituation.Ground) == ConsolePage.Launch && BurnList.EmptyNext(CraftSituation.Orbit) == ConsolePage.Orbit
            && BurnList.EmptyNext(CraftSituation.FallingNoAir) == ConsolePage.Land && BurnList.EmptyNext(CraftSituation.Escaping) == ConsolePage.Transfer);
        Check("one burn that flies needs no list", !BurnList.ListShowsWhileFlying(1) && BurnList.ListShowsWhileFlying(2));
        Check("the empty sentence is short", PageText.NoBurnPlanned.Length <= 24 && PageText.DrawerEmpty.Length <= 30);
        GuidanceModes.ResetToBuiltIns();
    }

    private static void LogLines()
    {
        Check("a short text is not cut", ShortLines.Cut("abc", 5) == "abc");
        Check("a cut text ends in dots", ShortLines.Cut("abcdefghij", 7) == "abcd...", ShortLines.Cut("abcdefghij", 7));
        Check("a null text cuts to empty", ShortLines.Cut(null, 5) == "");
        Check("nothing to split", ShortLines.Split("", 10).Count == 0 && ShortLines.Split(null, 10).Count == 0);
        List<string> words = ShortLines.Split("one two three four", 9);
        Check("split breaks at spaces", string.Join("|", words) == "one two|three|four", string.Join("|", words));
        List<string> path = ShortLines.Split("C:/a/very/long/path.json", 10);
        Check("a long word breaks inside", string.Join("", path) == "C:/a/very/long/path.json" && path.TrueForAll(p => p.Length <= 10),
            string.Join("|", path));
        string note = "2 of 4 stages at 35 kPa: did not converge in 200 iterations, the trust region collapsed, 16.07 t to orbit";
        string head = "convex note: ";
        List<string> lines = ShortLines.Split(note, ShortLines.Max - head.Length);
        Check("every split line fits the short width", lines.Count > 1 && lines.TrueForAll(l => head.Length + l.Length <= ShortLines.Max && l.Length > 0));
        Check("split keeps every word", string.Join(" ", lines) == note, string.Join("|", lines));
        string failed = "convex ascent did not converge: " + ShortLines.Cut(note, 37) + ".";
        Check("the failure line is short", failed.Length <= ShortLines.Max, failed);
        string planned = FormattableString.Invariant($"convex ascent planned: {123.45:F2} t to orbit, {10} of {10} stages, {127.6:F1} s.");
        Check("the planned line is short with large values", planned.Length <= ShortLines.Max, planned);
    }
}

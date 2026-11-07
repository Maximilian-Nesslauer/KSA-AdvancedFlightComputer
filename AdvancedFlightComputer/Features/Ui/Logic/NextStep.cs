namespace AdvancedFlightComputer.Features.Ui.Logic;

internal static class NextStep
{
    public static StatusLine AfterAscent(double eccentricity, bool circularizeFits = true)
        => eccentricity < OrbitToolCatalog.CircularEccentricity
            ? new StatusLine(Severity.Cue, PageText.AscentDoneCircular, ConsolePages.Key(ConsolePage.Orbit))
            : circularizeFits
                ? new StatusLine(Severity.Cue, PageText.CutoffCue, ConsolePages.CueTarget(ConsolePage.Orbit, OrbitToolCatalog.CueCircularize))
                : new StatusLine(Severity.Cue, PageText.CutoffCue, ConsolePages.Key(ConsolePage.Orbit));

    public static bool CircularizeFits(double peKm, double apKm, double goalPeKm, double goalApKm)
        => !PageText.GoalReached(peKm, apKm, goalPeKm, goalApKm) || Math.Abs(goalApKm - goalPeKm) <= Math.Max(5.0, 0.02 * Math.Abs(goalApKm));

    public static StatusLine AfterPass(int done, int next, string inTime, ConsolePage page)
        => new(Severity.Cue, PageText.PassDone(done, next, inTime), ConsolePages.Key(page), PageText.PassLink(next));

    public static StatusLine AfterPasses(in SituationFacts f, ConsolePage page)
        => new(Severity.Cue, PageText.Finished(PageText.PassesDone, in f), Target(f.Where, page));

    public static StatusLine AfterBurn(in SituationFacts f, string nextBurnIn, ConsolePage page = ConsolePage.Orbit)
        => nextBurnIn.Length > 0
            ? new StatusLine(Severity.Cue, PageText.NextBurnIn(nextBurnIn), ConsolePages.Key(ConsolePage.Rcs))
            : new StatusLine(Severity.Cue, PageText.Finished(PageText.BurnDone, in f), Target(f.Where, page));

    public static StatusLine AfterLanding(string body)
        => new(Severity.Cue, PageText.Landed(body), ConsolePages.Key(ConsolePage.Launch));

    private static string Target(CraftSituation where, ConsolePage inOrbit) => where switch
    {
        CraftSituation.FallingAir or CraftSituation.FallingNoAir => ConsolePages.Key(ConsolePage.Land),
        CraftSituation.Escaping => ConsolePages.Key(ConsolePage.Transfer),
        CraftSituation.Ground => ConsolePages.Key(ConsolePage.Launch),
        _ => ConsolePages.Key(inOrbit),
    };

    public static string LinkLabel(in StatusLine line)
    {
        if (line.CueLabel.Length > 0)
            return line.CueLabel;
        if (!ConsolePages.TryParseCue(line.CueTarget, out ConsolePage page, out string sub))
            return PageText.DrawerLink;
        if (page == ConsolePage.Land && sub.Length > 0)
            return PageText.NextLabel(page, ConsolePages.EnumFromKey(sub, LandWay.FromOrbit));
        if (page == ConsolePage.Orbit && OrbitToolCatalog.KeyOfCue(sub) is string tool)
            return OrbitToolCatalog.ToolLabels[OrbitToolCatalog.ToolIndex(tool)] + " >";
        return PageText.NextLabel(page, null);
    }
}

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>That time counts down by itself, so a rise in one frame is a new plan, and the head of the picture says so for a few seconds because the number in the head of the console jumps with no word.</summary>
internal sealed class PlanJump
{
    public const double RiseSeconds = 3.0;

    public const double ShowSeconds = 6.0;

    private double _lastToGo = double.NaN;
    private double _until = double.NegativeInfinity;

    /// <summary>Tells the time to go now and the real clock in seconds, and says whether the word stands.</summary>
    public bool Update(double toGoSeconds, double nowSeconds)
    {
        if (double.IsFinite(toGoSeconds))
        {
            if (double.IsFinite(_lastToGo) && toGoSeconds > _lastToGo + RiseSeconds)
                _until = nowSeconds + ShowSeconds;
            _lastToGo = toGoSeconds;
        }
        return nowSeconds < _until;
    }
}

/// <summary>The rule reads only what flies now, so a hand-back ends the view in the same frame.</summary>
internal static class FlightView
{
    public const string PlanFoldId = "FlightPlan";

    public static bool ModeFlies(GuidanceModeId mode, in CraftActivity a) => mode switch
    {
        GuidanceModeId.AscentUpfg => a.AscentRunning || a.LaunchArmed || a.ConvexLaunchPending,
        GuidanceModeId.Boostback => a.BoostbackLive,
        GuidanceModeId.DeorbitLand => a.DescentLive,
        GuidanceModeId.GfoldHere => a.LandingMachineLive,
        GuidanceModeId.SixDofHere => a.SixDofEngaged,
        GuidanceModeId.Hover => a.HoverLive,
        _ => a.RegisteredModeLive == mode,
    };

    public static bool PageFlies(ConsolePage page, LandWay way, LandSolver solver, AscentChoice ascent, in CraftActivity a, bool transferPasses) => page switch
    {
        ConsolePage.Orbit => a.MultiPassActive && !transferPasses,
        ConsolePage.Transfer => a.MultiPassActive && transferPasses,
        ConsolePage.Rcs => a.RcsFlying || a.StockAutoOnNode,
        _ => ConsolePages.ModeIdFor(page, way, solver, ascent) is GuidanceModeId mode && ModeFlies(mode, in a),
    };

    public static bool PickHolds(LandWay way, in CraftActivity a) => FlyingWay(in a) == way;

    public static LandWay? FlyingWay(in CraftActivity a)
    {
        if (a.SixDofEngaged || a.LandingMachineLive || a.RegisteredModeLive == GuidanceModeId.EntryHere)
            return LandWay.FromHere;
        if (a.HoverLive)
            return LandWay.Hover;
        if (a.DescentLive)
            return LandWay.FromOrbit;
        return a.BoostbackLive ? LandWay.Boostback : null;
    }

    public static bool FooterGoes(bool flies, bool footerHeld) => flies && !footerHeld;

    /// <summary>How long the flown path of a stopped descent stays on LAND NOW, in seconds of game time.</summary>
    public const double StoppedPathKeepSeconds = 900.0;

    public static bool StoppedPathShows(CraftSituation where, double secondsSinceLastSample)
        => (where == CraftSituation.Ground || SituationText.IsFalling(where)) && secondsSinceLastSample < StoppedPathKeepSeconds;

    public static bool ReadsNewSite(LandWay way, in CraftActivity a, bool deorbitLocked)
    {
        if (deorbitLocked)
            return false;
        return way switch
        {
            LandWay.Hover => false,
            LandWay.FromOrbit => a.LandingMachineLive,
            _ => true,
        };
    }

    public static string PlanLabel(string reads) => reads.Length > 0 ? PlanName + ": " + reads : PlanName;

    public const string PlanName = "PLAN";

    public static string LaunchReads(double peKm, double apKm, double incDeg, bool armed, bool autoWarp)
    {
        string goal = Fmt.Orbit(peKm, apKm) + ", " + Fmt.Deg(incDeg);
        return armed ? goal + ", auto warp " + (autoWarp ? "on" : "off") : goal;
    }

    public static string LandReads(LandWay way, bool siteLocked, double latDeg, double lonDeg)
    {
        if (way == LandWay.Hover)
            return "way, site, hover limits";
        return siteLocked ? "site locked" : string.Create(Fmt.Inv, $"site {latDeg:F3}, {lonDeg:F3}");
    }

    public static string LaunchPlanLine(bool running, bool armed)
        => running ? "This climb reads PE, AP, INC, PLANE and G-LIMIT in LIMITS at once. TARGET, CLIMB and MAX Q count at the next launch."
            : armed ? "The armed launch reads this form at its window."
            : "This form is locked while the plan calculates.";

    public static string LandPlanLine(LandWay way, LandSolver solver, bool siteLocked, bool hoverFlies = true)
    {
        if (way == LandWay.Hover)
            return "The hover reads its limits in HOW IT FLIES. The rest counts at the next landing.";
        if (siteLocked)
            return "The deorbit holds the site, the approach and the method. Stop it to change them.";
        if (way == LandWay.Boostback)
            return "This boostback reads the SITE and the HAND-OVERS now.";
        return solver switch
        {
            LandSolver.Entry => "Entry guidance reads the SITE and the HAND-OVERS now.",
            LandSolver.Touchdown => "Touchdown guidance reads the SITE now.",
            _ => hoverFlies ? "G-FOLD reads the SITE and the hover hand-over now." : "G-FOLD reads the SITE now.",
        };
    }

    public const string PassesPlanLine = "These passes are locked. A change here counts at the next plan.";
}

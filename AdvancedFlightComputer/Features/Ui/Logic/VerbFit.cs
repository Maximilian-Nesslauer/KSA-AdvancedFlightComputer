namespace AdvancedFlightComputer.Features.Ui.Logic;

internal readonly record struct VerbFit(string Reason, ConsolePage? NextPage = null, LandWay? NextWay = null, bool Plain = false, LandSolver? NextSolver = null, string NextLabel = "", bool NextPicksSite = false, bool NextQuiet = false)
{
    public static readonly VerbFit Yes = new("");

    public bool Fits => string.IsNullOrEmpty(Reason);

    public string Text => Reason ?? "";

    public bool Calm => Plain || NextPage != null || NextSolver != null;
}

internal static class SituationVerbs
{
    public const string OnGround = "LAUNCH FIRST.";
    public const string InOrbitShape = "NEXT: SHAPE ORBIT.";
    public const string CoastingShape = InOrbitShape;
    public const string EscapingShape = InOrbitShape;
    public const string FallingLand = "NEXT: LAND.";
    public const string AfterSeparation = "STARTS AFTER SEPARATION.";
    public const string NoStageHome = "NO STAGE CAN FLY HOME.";
    public const string NoImpact = "NO IMPACT PREDICTED.";
    public const string NeedsOrbit = "NEEDS ORBIT.";
    public const string BelowBrakingStart = "TOO LOW FOR DEORBIT.";
    public const string DeorbitFirst = "DEORBIT FIRST.";
    public const string BoostbackNoAir = "NO AIR. ENDS IN IMPACT.";
    public const string AscentFlies = "ASCENT ACTIVE. PLAN AFTER CUTOFF.";
    public const string GuidanceFlies = "AFC ACTIVE. PLAN AFTER IT ENDS.";
    public const string AscentFliesLand = "ASCENT ACTIVE. LAND AFTER CUTOFF.";
    public const string AscentFliesEnds = "ASCENT ACTIVE. G-FOLD CANNOT START.";
    public const string BackToFlight = "BACK TO FLIGHT >";

    public static readonly VerbFit AscentFliesLandFit = new(AscentFliesLand, ConsolePage.Launch, Plain: true, NextLabel: BackToFlight, NextQuiet: true);

    public static readonly VerbFit AscentFliesEndsFit = AscentFliesLandFit with { Reason = AscentFliesEnds };

    public static string VerbName(GuidanceModeId mode) => mode switch
    {
        GuidanceModeId.DeorbitLand => "DEORBIT + LAND",
        GuidanceModeId.Boostback => "BOOSTBACK",
        GuidanceModeId.GfoldHere => LandSolvers.Label(LandSolver.Gfold),
        GuidanceModeId.SixDofHere => LandSolvers.Label(LandSolver.Touchdown),
        GuidanceModeId.EntryHere => LandSolvers.Label(LandSolver.Entry),
        _ => "HOVER",
    };

    public static string DeorbitFlies(GuidanceModeId mode) => "TAKE OVER replaces the deorbit with " + VerbName(mode) + ".";

    public const string EntryNeedsAir = "No air. Use TOUCHDOWN.";

    public const string GfoldNoAir = "G-FOLD needs an airless body.";

    public const string GfoldNeedsNoAir = GfoldNoAir + " Use TOUCHDOWN.";

    public const string GfoldEndsDeorbit = GfoldNoAir + " END LANDING only ends the deorbit.";

    public static VerbFit DeorbitFliesFitFor(GuidanceModeId mode, in CommitGates g)
    {
        string text = mode == GuidanceModeId.Hover && g.HoverProblem.Length > 0 ? g.HoverProblem
            : mode == GuidanceModeId.GfoldHere && g.HasAir ? GfoldEndsDeorbit
            : DeorbitFlies(mode);
        return new VerbFit(text, ConsolePage.Land, LandWay.FromOrbit, NextLabel: BackToFlight);
    }

    public static VerbFit For(GuidanceModeId mode, in CommitGates g)
    {
        VerbFit fit = ForSituation(mode, in g);
        if (!fit.Fits)
            return fit;
        if (g.NoSiteNote.Length > 0 && mode is GuidanceModeId.DeorbitLand or GuidanceModeId.Boostback or GuidanceModeId.EntryHere or GuidanceModeId.SixDofHere or GuidanceModeId.GfoldHere)
            return new VerbFit(g.NoSiteNote, Plain: true, NextPicksSite: true);
        if (g.FarSiteNote.Length > 0 && mode is GuidanceModeId.SixDofHere or GuidanceModeId.GfoldHere)
            return new VerbFit(g.FarSiteNote, NextPicksSite: true);
        return fit;
    }

    private static VerbFit ForSituation(GuidanceModeId mode, in CommitGates g)
    {
        if (mode == GuidanceModeId.Hover)
        {
            if (g.Where == CraftSituation.Ground)
                return g.HoverProblem.Length > 0 ? Grounded : VerbFit.Yes;
            return FirstOf(FromHere(g.Where), g.HoverProblem.Length > 0 ? new VerbFit(PageText.HoverProblemLine(g.HoverProblem), ConsolePage.Land, LandWay.FromHere) : VerbFit.Yes);
        }
        if (g.Where == CraftSituation.Unknown)
            return VerbFit.Yes;
        return mode switch
        {
            GuidanceModeId.AscentUpfg or GuidanceModeId.AscentGravityTurn => Launch(g.Where),
            GuidanceModeId.DeorbitLand => Deorbit(in g),
            GuidanceModeId.Boostback => Boostback(in g),
            GuidanceModeId.GfoldHere => FirstOf(FromHere(g.Where), g.HasAir ? new VerbFit(GfoldNeedsNoAir, NextSolver: LandSolver.Touchdown) : VerbFit.Yes),
            GuidanceModeId.SixDofHere => FromHere(g.Where),
            GuidanceModeId.EntryHere => FirstOf(FromHere(g.Where), g.HasAir ? VerbFit.Yes : new VerbFit(EntryNeedsAir, NextSolver: LandSolver.Touchdown)),
            _ => VerbFit.Yes,
        };
    }

    private static VerbFit FirstOf(in VerbFit first, in VerbFit second) => first.Fits ? second : first;

    private static readonly VerbFit Grounded = new(OnGround, ConsolePage.Launch);

    private static VerbFit FromHere(CraftSituation where) => where switch
    {
        CraftSituation.Ground => Grounded,
        CraftSituation.Orbit => new VerbFit(DeorbitFirst, ConsolePage.Land, LandWay.FromOrbit),
        CraftSituation.Escaping => new VerbFit(NoImpact, Plain: true),
        _ => VerbFit.Yes,
    };

    private static VerbFit Launch(CraftSituation where) => where switch
    {
        CraftSituation.Ground => VerbFit.Yes,
        CraftSituation.FallingAir or CraftSituation.FallingNoAir => new VerbFit(FallingLand, ConsolePage.Land),
        _ => new VerbFit(InOrbitShape, ConsolePage.Orbit),
    };

    private static VerbFit Deorbit(in CommitGates g)
    {
        if (g.Where == CraftSituation.Ground)
            return Grounded;
        if (SituationText.IsFalling(g.Where) && g.BelowBrakingStart)
            return new VerbFit(BelowBrakingStart, ConsolePage.Land, LandWay.FromHere);
        return g.HasAir && g.GfoldChosen ? new VerbFit(GfoldNeedsNoAir, NextSolver: LandSolver.Touchdown) : VerbFit.Yes;
    }

    private static VerbFit Boostback(in CommitGates g) => g.Where switch
    {
        CraftSituation.Ground => g.NoStageHome ? new VerbFit(NoStageHome, Plain: true) : new VerbFit(AfterSeparation, ConsolePage.Launch),
        CraftSituation.Orbit or CraftSituation.Escaping => FromHere(g.Where),
        CraftSituation.CoastingUp or CraftSituation.FallingNoAir when !g.HasAir
            => new VerbFit(BoostbackNoAir, ConsolePage.Land, LandWays.FitsNow(g.Where, false, g.BelowBrakingStart) ?? LandWay.FromHere),
        _ => VerbFit.Yes,
    };

    public static string WayNow(LandWay way, CraftSituation where, bool hasAir, bool belowBrakingStart, bool noStageHome = false)
    {
        GuidanceModeId mode = way switch
        {
            LandWay.FromOrbit => GuidanceModeId.DeorbitLand,
            LandWay.Boostback => GuidanceModeId.Boostback,
            LandWay.FromHere => GuidanceModeId.SixDofHere,
            _ => GuidanceModeId.Hover,
        };
        var gates = new CommitGates(false, false, "", "", false, Where: where, HasAir: hasAir, BelowBrakingStart: belowBrakingStart, NoStageHome: noStageHome);
        return ForSituation(mode, in gates).Text;
    }

    public static VerbFit ForManeuver(CraftSituation where, bool ascentFlies = false, bool otherModeFlies = false)
        => where == CraftSituation.Ground ? Grounded : WhileFlying(ascentFlies, otherModeFlies);

    public static VerbFit ForTransfer(CraftSituation where, bool ascentFlies = false, bool otherModeFlies = false) => where switch
    {
        CraftSituation.Ground => Grounded,
        CraftSituation.FallingAir or CraftSituation.FallingNoAir => new VerbFit(FallingLand, ConsolePage.Land),
        _ => WhileFlying(ascentFlies, otherModeFlies),
    };

    private static VerbFit WhileFlying(bool ascentFlies, bool otherModeFlies)
        => ascentFlies ? new VerbFit(AscentFlies, Plain: true)
            : otherModeFlies ? new VerbFit(GuidanceFlies, Plain: true)
            : VerbFit.Yes;
}

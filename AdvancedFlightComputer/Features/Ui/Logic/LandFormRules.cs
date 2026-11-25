using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal readonly record struct ChainStep(string Text, bool Stock = false, bool Danger = false, bool Warn = false);

/// <summary>
/// The path of modes a LAND way flies, drawn under the way's rows before the engage and marked while it flies:
/// DEORBIT (the burn) > ENTRY/BRAKING (orbital entry where there is air, the braking burn where there is none) >
/// the fall to the landing burn after an entry (FLIP LANDING: bellyflop, or RETRO BURN) > TOUCHDOWN, which ends every
/// powered landing. Each mode declares its own steps and the mode it hands to (GuidanceModeDescriptor.Steps, .Next),
/// and the path follows them from the mode the commit engages (ConsolePages.ModeIdFor), so the line says what the press flies.
/// </summary>
internal static class LandChain
{
    public const string StockBurn = "DEORBIT BURN";

    public const string StockBurnIfNeeded = "BURN IF NEEDED";

    public const string Impact = "IMPACT";

    public const string Bellyflop = "BELLYFLOP";

    /// <summary>The end of an entry for a craft with no engine to land on: orbital entry holds its attitude after the hand-over, and the pilot flies the rest (the parachutes).</summary>
    public const string Hold = "HOLD";

    public const string YouFly = "YOU FLY";

    /// <summary>The end of an entry for a craft with parachutes and no engine to land on: orbital entry arms them at the hand-over, and each opens at its own deploy altitude.</summary>
    public const string Parachutes = "PARACHUTES";

    public static bool EndsInImpact(ReadOnlySpan<ChainStep> steps) => steps.Length > 0 && steps[^1].Danger;

    /// <summary>Every way but HOVER shows its path, also a path of one step: what flies is never left to guess.</summary>
    public static bool Shows(ReadOnlySpan<ChainStep> steps) => steps.Length > 0;

    /// <summary>
    /// The path of a way: from the mode its commit engages (ConsolePages.ModeIdFor), each mode's own steps (GuidanceModeDescriptor.Steps), then the mode it hands to (GuidanceModeDescriptor.Next, else GuidanceModes.DefaultNext), until a mode hands to none. HOVER shows no path.
    /// </summary>
    /// <param name="flip">The way's glide is FLIP LANDING, not RETRO BURN: LAND NOW's own, or the one after an orbital entry.</param>
    /// <param name="canLand">The craft has an engine to land on; with none, an entry ends in PARACHUTES where it has them, else in HOLD and the pilot flies the rest.</param>
    /// <param name="chutes">The craft has parachutes.</param>
    public static ChainStep[] For(LandWay way, LandSolver solver, CraftSituation where, bool hasAir, bool flip = false, bool canLand = true, bool belowBrakingStart = false, bool hoverFlies = true, bool chutes = false)
    {
        if (way == LandWay.Hover)
            return [];
        var facts = new PathFacts(way, hasAir, canLand, chutes, flip, hoverFlies, where, belowBrakingStart,
            GfoldFlies: solver == LandSolver.Gfold, BrakingOnly: way == LandWay.Entry && !hasAir);
        return Follow(ConsolePages.ModeIdFor(ConsolePage.Land, way, solver, AscentChoice.CatsUpfg, hasAir, flip && hasAir && way is LandWay.FromHere or LandWay.Glide), in facts);
    }

    /// <summary>The steps from <paramref name="first"/> on, each mode once.</summary>
    public static ChainStep[] Follow(GuidanceModeId? first, in PathFacts facts)
    {
        var steps = new List<ChainStep>(8);
        var seen = new HashSet<GuidanceModeId>();
        for (GuidanceModeId? id = first; id is GuidanceModeId mode && seen.Add(mode); id = GuidanceModes.NextOf(mode, in facts))
            steps.AddRange(GuidanceModes.StepsOf(mode, in facts));
        return steps.ToArray();
    }

    /// <summary>A step of the path that has settings of its own, so a click on it shows them (GuidanceWindow.DrawStepRows). A coast, a hold, the pilot's part, the parachutes and a crash have none, and nor have a boostback's settle and slew.</summary>
    public static bool HasSettings(string step) => step is StockBurn or StockBurnIfNeeded or Bellyflop or "TOUCHDOWN" or "G-FOLD" or "HOVER" or "BURN" or "GLIDE"
        || step == EntryMethods.OrbitalLabel || step == EntryMethods.GlideBurnLabel || step == EntryMethods.BrakingLabel;
}

/// <summary>What the modes of a landing path decide their steps and their hand-over by: the way, the body's air, the craft (an engine to land on, parachutes, whether it can hover) and the player's glide.</summary>
internal readonly record struct PathFacts(
    LandWay Way, bool HasAir, bool CanLand = true, bool Chutes = false, bool Flip = true, bool HoverFlies = true,
    CraftSituation Where = CraftSituation.Unknown, bool BelowBrakingStart = false, bool GfoldFlies = false, bool BrakingOnly = false);

/// <summary>What the craft can do at the landing site, read once a frame. <see cref="DescentMs"/> and <see cref="AltToGoM"/> stand in the frame of the site, as the stop check reads them. <see cref="SinkMs"/> and <see cref="HeightM"/> are the local vertical speed and the height above the ground under the craft, for the rows that tell a pilot how he sinks: motion past a far site reads as a descent in the frame of the site, and a craft in an orbit does not sink.</summary>
internal readonly record struct DescentFacts(
    bool Known, double ThrustN, double VacuumN, double WeightN, double GravityMs2, double AmbientPa,
    double Floor, double CraftFloor, double TwrAtFloor, double TwrAtCraftMin, double NeedTiltDeg, double TiltLimitDeg,
    double DescentMs, double AltToGoM, bool NoSteering = false, double SinkMs = double.NaN, double HeightM = double.NaN)
{
    public double TwrMax => WeightN > 0.0 ? ThrustN / WeightN : 0.0;

    public bool Sinking => SinkMs > 1.0 && HeightM > 1.0;

    public double SinkStopM => StopFrom(SinkMs);

    public bool Descending => AltToGoM > 1.0 && DescentMs > 1.0;

    public double StopM => StopFrom(DescentMs);

    private double StopFrom(double speedMs)
    {
        double netDecel = (TwrMax - 1.0) * GravityMs2;
        return netDecel > 0.01 ? speedMs * speedMs / (2.0 * netDecel) : double.PositiveInfinity;
    }

    public double TwrNeeded => AltToGoM > 0.0 && GravityMs2 > 0.0 ? 1.0 + DescentMs * DescentMs / (2.0 * AltToGoM * GravityMs2) : double.PositiveInfinity;

    public bool CannotStop => Descending && StopM >= AltToGoM;

    public bool Marginal => Descending && !CannotStop && StopM > 0.8 * AltToGoM;

    public bool CannotStopNow => Sinking && CannotStop;

    public bool MarginalNow => Sinking && Marginal;

    public bool OverPowered => Known && NeedTiltDeg >= TiltLimitDeg;

    public double FeasibleFloor => ThrustN > 0.0 ? Math.Cos(TiltLimitDeg * Math.PI / 180.0) * WeightN / ThrustN : 0.0;
}

internal readonly record struct LandNowFacts(
    LandWay Way, LandSolver Chosen, bool HasAir, bool DeorbitFlies, bool GfoldFlies, DescentFacts Descent = default, string Body = "");

internal readonly record struct TableMark(int Row, int Column, TableInk Ink);

internal enum TableInk { Warn, Danger, WarnDim, DangerDim, Stock, Afc, WarnOnBand, DangerOnBand, Method, MethodDim }

internal readonly record struct CellSwatch(int Row, int Column, Rgb8 Hue, Rgb8? Second = null);

internal readonly record struct RowNote(string Text, bool Stops = false)
{
    public static readonly RowNote None = new("");
}

internal readonly record struct LandRead(DescentFacts Descent, bool Flying);

internal readonly record struct LandVerdict(string Text, Severity Severity = Severity.Info, bool Soft = false)
{
    public static readonly LandVerdict None = new("");

    private static readonly System.Globalization.CultureInfo Inv = Fmt.Inv;

    public static string TwrAtSite(in DescentFacts f, string place = "SITE")
        => f.Known ? string.Create(Inv, $"TWR {f.TwrMax:F2} AT {place}") : "";

    public static LandVerdict ForDescent(LandSolver solver, in DescentFacts f, bool checkStop)
    {
        if (!f.Known)
            return new LandVerdict(PageText.NoThrustToJudge);
        if (checkStop && f.CannotStopNow)
            return new LandVerdict(string.Create(Inv, $"NO SOLUTION: TWR {f.TwrNeeded:F2} NEEDED, {f.TwrMax:F2} AVAILABLE."), Severity.Error);
        if (solver == LandSolver.Touchdown && f.OverPowered)
            return new LandVerdict(string.Create(Inv, $"OVERPOWERED: FLOOR ABOVE {f.FeasibleFloor:F2}. LOWER FLOOR."), Severity.Error);
        if (f.TwrMax < 1.0)
            return new LandVerdict(string.Create(Inv, $"TWR {f.TwrMax:F2} AT SITE, BELOW WEIGHT."), Severity.Warning);
        if (checkStop && f.MarginalNow)
            return new LandVerdict(string.Create(Inv, $"MARGINAL: STOP USES {f.StopM / f.AltToGoM * 100.0:F0} % OF HEIGHT."), Severity.Warning);
        double low = solver == LandSolver.Touchdown ? f.TwrAtFloor : f.TwrAtCraftMin;
        return new LandVerdict(string.Create(Inv, $"TWR {f.TwrMax:F2} AT SITE, {low:F2} AT MIN THROTTLE."));
    }

    public static LandVerdict ForImpact(bool hasAero, bool hasImpact, double impactInS, double impactMs, bool beyondHorizon, double horizonS, double closestM, string otherStatus, bool timeInHead = false)
    {
        if (!hasAero)
            return new LandVerdict("NO AIR DATA. NO PREDICTION.", Severity.Warning);
        if (hasImpact)
            return new LandVerdict(timeInHead ? "IMPACT AT " + Fmt.Speed(impactMs) : "IMPACT T-" + Fmt.Time(impactInS) + " AT " + Fmt.Speed(impactMs));
        return beyondHorizon
            ? new LandVerdict("LOWEST POINT " + Fmt.Distance(closestM))
            : new LandVerdict(otherStatus, Severity.Warning);
    }

    public static IEnumerable<LandVerdict> Notes(LandSolver solver, DescentFacts f, bool checkStop)
    {
        if (!f.Known)
            yield break;
        if (checkStop && f.CannotStopNow)
            yield return new LandVerdict(string.Create(Inv, $"NO SOLUTION: TWR {f.TwrNeeded:F2} NEEDED, {f.TwrMax:F2} AVAILABLE. BURN EARLIER OR ARRIVE SLOWER."), Severity.Error);
        else if (checkStop && f.MarginalNow)
            yield return new LandVerdict(string.Create(Inv, $"MARGINAL: STOP USES {f.StopM / f.AltToGoM * 100.0:F0} % OF ALTITUDE LEFT. TWR {f.TwrNeeded:F2} NEEDED, {f.TwrMax:F2} AVAILABLE."), Severity.Warning);

        if (solver == LandSolver.Touchdown)
        {
            if (f.OverPowered)
                yield return new LandVerdict(string.Create(Inv, $"NO SOLUTION: OVERPOWERED, {f.NeedTiltDeg:F0} deg TILT NEEDED, LIMIT {f.TiltLimitDeg:F0} deg. SET FLOOR BELOW {f.FeasibleFloor:F2} OR RAISE TILT LIMIT."), Severity.Error);
            else if (f.TwrAtFloor > 1.0)
                yield return new LandVerdict(string.Create(Inv, $"MIN THRUST ABOVE WEIGHT: {f.NeedTiltDeg:F0} deg TILT TO HOLD ALTITUDE. NO STRAIGHT DESCENT."), Soft: true);
            else
                yield return new LandVerdict("CAN HOVER AT MIN THROTTLE. STRAIGHT DESCENT.");
        }
    }
}

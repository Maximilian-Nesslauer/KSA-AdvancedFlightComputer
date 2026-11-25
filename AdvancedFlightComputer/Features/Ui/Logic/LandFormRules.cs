using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal readonly record struct ChainStep(string Text, bool Stock = false, bool Danger = false, bool Warn = false);

/// <summary>
/// The path of modes a LAND way flies, drawn under the way's rows before the engage and marked while it flies:
/// DEORBIT (the burn) > ENTRY/BRAKING (orbital entry where there is air, the braking burn where there is none) >
/// the fall to the landing burn after an entry (FLIP LANDING: bellyflop) > TOUCHDOWN, which ends every powered
/// landing. The commit engages the same path (ConsolePages.ModeIdFor), so the line says what the press flies.
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

    public static bool EndsInImpact(ReadOnlySpan<ChainStep> steps) => steps.Length > 0 && steps[^1].Danger;

    /// <summary>A step of the path that has settings of its own, so a click on it shows them (GuidanceWindow.DrawStepRows). A coast, a hold, the pilot's part and a crash have none, and nor have a boostback's settle and slew.</summary>
    public static bool HasSettings(string step) => step is StockBurn or StockBurnIfNeeded or Bellyflop or "TOUCHDOWN" or "G-FOLD" or "HOVER" or "BURN" or "GLIDE"
        || step == EntryMethods.OrbitalLabel || step == EntryMethods.GlideBurnLabel || step == EntryMethods.BrakingLabel;

    /// <summary>Every way but HOVER shows its path, also a path of one step: what flies is never left to guess.</summary>
    public static bool Shows(ReadOnlySpan<ChainStep> steps) => steps.Length > 0;

    /// <param name="flip">LAND NOW's glide is FLIP LANDING, not RETRO BURN.</param>
    /// <param name="canLand">The craft has an engine to land on; with none, an entry ends in HOLD and the pilot flies the rest.</param>
    public static ChainStep[] For(LandWay way, LandSolver solver, CraftSituation where, bool hasAir, bool flip = false, bool canLand = true, bool belowBrakingStart = false, bool hoverFlies = true)
    {
        var steps = new List<ChainStep>(8);
        switch (way)
        {
            case LandWay.FromOrbit when hasAir:
                steps.Add(new ChainStep(StockBurn, Stock: true));
                AddEntry(steps, canLand);
                break;
            case LandWay.FromOrbit:
                if (!(SituationText.IsFalling(where) && belowBrakingStart))
                    steps.AddRange([new ChainStep(SituationText.IsFalling(where) ? StockBurnIfNeeded : StockBurn, Stock: true), new ChainStep("COAST"), new ChainStep(EntryMethods.BrakingLabel)]);
                AddPoweredEnd(steps, solver, hoverFlies);
                break;
            case LandWay.Entry when hasAir:
                AddEntry(steps, canLand);
                break;
            case LandWay.Entry:
                steps.Add(new ChainStep(EntryMethods.BrakingLabel));
                AddPoweredEnd(steps, solver == LandSolver.Entry ? LandSolver.Touchdown : solver, hoverFlies);
                break;
            case LandWay.FromHere when solver == LandSolver.Entry:
                if (!hasAir)
                {
                    steps.Add(new ChainStep("NO PLAN, ENGINE OFF", Warn: true));
                    steps.Add(new ChainStep(Impact, Danger: true));
                    break;
                }
                steps.Add(new ChainStep(flip ? Bellyflop : EntryMethods.GlideBurnLabel));
                steps.Add(new ChainStep(LandSolvers.Label(LandSolver.Touchdown)));
                break;
            case LandWay.FromHere:
                AddPoweredEnd(steps, solver, hoverFlies);
                break;
            case LandWay.Boostback:
                steps.AddRange([new ChainStep("SETTLE"), new ChainStep("SLEW"), new ChainStep("BURN"), new ChainStep(hasAir ? "GLIDE" : "FALL")]);
                if (hasAir)
                {
                    steps.Add(new ChainStep(EntryMethods.GlideBurnLabel));
                    steps.Add(new ChainStep(LandSolvers.Label(LandSolver.Touchdown)));
                }
                else
                    steps.Add(new ChainStep(Impact, Danger: true));
                break;
            default:
                break;
        }
        return steps.ToArray();
    }

    private static void AddEntry(List<ChainStep> steps, bool canLand)
    {
        steps.Add(new ChainStep(EntryMethods.OrbitalLabel));
        if (canLand)
            steps.AddRange([new ChainStep(Bellyflop), new ChainStep(LandSolvers.Label(LandSolver.Touchdown))]);
        else
            steps.AddRange([new ChainStep(Hold), new ChainStep(YouFly, Warn: true)]);
    }

    private static void AddPoweredEnd(List<ChainStep> steps, LandSolver solver, bool hoverFlies)
    {
        steps.Add(new ChainStep(LandSolvers.Label(solver)));
        if (solver == LandSolver.Gfold && hoverFlies)
            steps.Add(new ChainStep("HOVER"));
    }
}

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

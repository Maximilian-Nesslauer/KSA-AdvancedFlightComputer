using System;
using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal readonly record struct ChainStep(string Text, bool Stock = false, bool Danger = false, bool Warn = false);

/// <summary>A deorbit starts with the burn stock AUTO flies, unless the craft already falls and the planner finds a direct approach (DeorbitPlanner.DirectSteps).</summary>
internal static class LandChain
{
    public const string StockBurn = "DEORBIT BURN";

    public const string StockBurnIfNeeded = "BURN IF NEEDED";

    public const string Impact = "IMPACT";

    public static bool EndsInImpact(ReadOnlySpan<ChainStep> steps) => steps.Length > 0 && steps[^1].Danger;

    public static bool Shows(ReadOnlySpan<ChainStep> steps) => steps.Length > 1;

    public static ChainStep[] For(LandWay way, LandSolver solver, CraftSituation where, bool hasAir, bool entryTakesGlide, bool touchdownTakesBurn, bool belowBrakingStart = false, bool hoverFlies = true)
    {
        var steps = new List<ChainStep>(8);
        switch (way)
        {
            case LandWay.FromOrbit:
                if (!(SituationText.IsFalling(where) && belowBrakingStart))
                    steps.AddRange([new ChainStep(SituationText.IsFalling(where) ? StockBurnIfNeeded : StockBurn, Stock: true), new ChainStep("COAST"), new ChainStep("BRAKING BURN")]);
                AddPoweredEnd(steps, solver, hoverFlies);
                break;
            case LandWay.FromHere when solver == LandSolver.Entry:
                if (!hasAir)
                {
                    steps.Add(new ChainStep("NO PLAN, ENGINE OFF", Warn: true));
                    steps.Add(new ChainStep(Impact, Danger: true));
                    break;
                }
                steps.Add(new ChainStep("GLIDE"));
                AddEntryBurnEnd(steps, touchdownTakesBurn);
                break;
            case LandWay.FromHere:
                AddPoweredEnd(steps, solver, hoverFlies);
                break;
            case LandWay.Boostback:
                steps.AddRange([new ChainStep("SETTLE"), new ChainStep("SLEW"), new ChainStep("BURN"), new ChainStep(hasAir ? "GLIDE" : "FALL")]);
                if (hasAir && entryTakesGlide)
                {
                    steps.Add(new ChainStep(LandSolvers.Label(LandSolver.Entry)));
                    AddEntryBurnEnd(steps, touchdownTakesBurn);
                }
                else
                    steps.Add(new ChainStep(Impact, Danger: true));
                break;
            default:
                break;
        }
        return steps.ToArray();
    }

    private static void AddPoweredEnd(List<ChainStep> steps, LandSolver solver, bool hoverFlies)
    {
        steps.Add(new ChainStep(LandSolvers.Label(solver)));
        if (solver == LandSolver.Gfold && hoverFlies)
            steps.Add(new ChainStep("HOVER"));
    }

    private static void AddEntryBurnEnd(List<ChainStep> steps, bool touchdownTakesBurn)
    {
        steps.Add(new ChainStep("LANDING BURN"));
        if (touchdownTakesBurn)
            steps.Add(new ChainStep(LandSolvers.Label(LandSolver.Touchdown)));
    }
}

/// <summary>What the craft can do at the landing site, read once a frame. <see cref="DescentMs"/> and <see cref="AltToGoM"/> stand in the frame of the site, as the stop check reads them. <see cref="SinkMs"/> and <see cref="HeightM"/> are the local vertical speed and the height above the ground under the craft, for the rows that tell a pilot how he sinks: motion past a far site reads as a descent in the frame of the site, and a craft in an orbit does not sink.</summary>
internal readonly record struct DescentFacts(
    bool Known, double ThrustN, double VacuumN, double WeightN, double GravityMs2, double AmbientPa,
    double Floor, double CraftFloor, double TwrAtFloor, double TwrAtCraftMin, double NeedTiltDeg, double TiltLimitDeg,
    double DescentMs, double AltToGoM, bool NoGimbal = false, double SinkMs = double.NaN, double HeightM = double.NaN)
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

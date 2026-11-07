using System.Collections.Generic;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal static class BurnList
{
    public static readonly string[] Heads = { "BURN", "IN", "DELTA-V", "WILL FLY", "FUEL" };

    private const int WhoColumn = 3, FuelColumn = 4;

    public static void MarksOf(int row, bool chosen, bool rcs, bool fuelShort, List<TableMark> marks)
    {
        marks.Add(new TableMark(row, WhoColumn, rcs ? TableInk.Afc : TableInk.Stock));
        if (fuelShort)
            marks.Add(new TableMark(row, FuelColumn, chosen ? TableInk.Warn : TableInk.WarnDim));
    }

    public static int FuelSwatchColumn => FuelColumn;

    public const string Stock = "Stock AUTO";
    public const string Rcs = "AFC RCS";

    public static string Label(int index, int pass)
        => pass > 0 ? "Pass " + pass.ToString(Fmt.Inv)
            : index == 0 ? "NEXT"
            : (index + 1).ToString(Fmt.Inv);

    public static string In(double seconds, bool flies)
        => seconds > 0.0 ? Fmt.Time(seconds)
            : flies ? "Now"
            : "Past";

    public static string Who(bool rcs) => rcs ? Rcs : Stock;

    public static string EngineFuel(StageFuel[] stages, in BurnPay pay)
    {
        if (stages.Length == 0)
            return "";
        if (pay.ShortMs > 0.5)
            return string.Create(Fmt.Inv, $"~{pay.ShortMs:F0} m/s short");
        return pay.FirstStage >= 0
            ? BurnBudget.Payer(stages, in pay) + ", ~" + Fmt.Mass(pay.PropellantKg)
            : "";
    }

    public static string RcsFuel(double kg)
        => double.IsFinite(kg) ? "RCS, ~" + Fmt.Mass(kg) : "RCS";

    public static string[] Row(int index, int pass, double seconds, bool flies, double dvMs, bool rcs, string fuel)
        => new[] { Label(index, pass), In(seconds, flies), Fmt.Dv(dvMs), Who(rcs), fuel };

    /// <summary>With no such thruster ENGINE, ELSE RCS is no RCS burn, because RcsExecutor.WouldExecuteRcs says no and stock AUTO gets the press.</summary>
    public static bool RcsTakes(bool rcsFlies, bool rcsOnly, bool resolvesToRcs, bool canTranslate)
        => rcsFlies || rcsOnly || (resolvesToRcs && canTranslate);

    /// <summary>A burn past its time that does not fly asks nothing, the same as BurnPlan.CalculateNewFlightPlansFromFlightComputerOnly, which takes no mass for it, and such a burn stays in the plan after it flew by hand.</summary>
    public static double Charge(double seconds, bool flies, double dvMs, double toGoMs)
        => flies ? toGoMs : seconds > 0.0 ? dvMs : 0.0;

    public static bool RcsPictureShows(bool rcsFlies, bool resolvesToRcs, bool canTranslate)
        => rcsFlies || (resolvesToRcs && canTranslate);

    public static ConsolePage EmptyNext(CraftSituation where) => where switch
    {
        CraftSituation.Ground => ConsolePage.Launch,
        CraftSituation.FallingAir or CraftSituation.FallingNoAir => ConsolePage.Land,
        CraftSituation.Escaping => ConsolePage.Transfer,
        _ => ConsolePage.Orbit,
    };

    public static bool ListShowsWhileFlying(int rows) => rows > 1;
}

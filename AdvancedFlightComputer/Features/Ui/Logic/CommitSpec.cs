using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

internal static class FlownBy
{
    public const string You = "MANUAL";
    public const string Afc = "AFC";
    public const string StockAuto = "STOCK AUTO";
    public const string AfcPlansStockAuto = "AFC PLANS + STOCK AUTO";
    public const string Nobody = "-";

    public const string OnPress = "EXEC:";

    public const string PlansA = OnPress + " PLANS ";

    public const string StockChip = "STOCK";

    public const string BurnWord = "BURN";

    public const string BurnsWord = "BURNS";

    public static string PlansMany(int burns) => OnPress + " PLANS " + burns.ToString(System.Globalization.CultureInfo.InvariantCulture);

    public const string StockAutoName = "STOCK AUTO";

    public const string ThenAfc = "THEN AFC";

    public const string PlanJoin = " + ";

    /// <summary>Stock AUTO flies the deorbit burn of a landing, and a pass that stock AUTO flies, while AFC holds the plan.</summary>
    public static string Now(in CraftActivity a)
    {
        if (a.LandingStockAuto)
            return AfcPlansStockAuto;
        if ((a.AnyGuidanceLive && !a.PilotHolds) || a.RcsFlying)
            return Afc;
        if (a.MultiPassActive)
            return a.StockAutoOnNode ? AfcPlansStockAuto : You;
        return a.StockAutoOnNode ? StockAuto : You;
    }

    public static string Sentence(string who) => who switch
    {
        Afc => "AFC",
        StockAuto => "STOCK AUTO",
        AfcPlansStockAuto => "AFC PLANS + STOCK AUTO",
        _ => "MANUAL",
    };
}

internal readonly record struct OwnerMark(string Before, string Chip = "", string After = "")
{
    public static readonly OwnerMark None = new("");

    public bool IsEmpty => Before.Length == 0 && Chip.Length == 0 && After.Length == 0;
}

/// <summary>
/// The one commit of a page: the verb on the footer button, who flies the result, the refusal that
/// disables it, the footer status, and the action. A page builds it from the same state its preview
/// reads, so the preview and the commit share one refusal. The action runs after ConsoleStyle.EndFooter,
/// as stock does with its CREATE button in TransferPlanner.DrawPlanWindow. <see cref="Owner"/> colours
/// the chip that shows <see cref="FlownBy"/>, <see cref="Outcome"/> is the sentence next to it that says
/// what the verb will do, and <see cref="Secondary"/> is a second, outlined verb left of the primary one.
/// </summary>
internal sealed record CommitSpec(
    string Verb,
    string FlownBy,
    string Refusal = "",
    string Status = "",
    bool StatusPending = false,
    string Tooltip = "",
    bool NeedsTakeOver = false,
    Action? Act = null,
    Owner Owner = Owner.None,
    string Outcome = "",
    CommitAction? Secondary = null,
    bool Quiet = false,
    string TakeOverFrom = "",
    string Caution = "",
    bool Calm = false,
    bool HoldsFooter = false,
    bool PressEnds = false,
    string EndsLabel = "",
    int PlannedBurns = 1,
    bool MarkDim = false)
{
    public static readonly CommitSpec None = new("", Logic.FlownBy.Nobody);

    public bool SaysNothing => Verb.Length == 0 && (Secondary == null || Secondary.Verb.Length == 0) && AnswerStatus.Length == 0 && Outcome.Length == 0;

    /// <summary>Whether the primary verb can be clicked. The secondary verb has its own refusal and does not change it.</summary>
    public bool Enabled => Verb.Length > 0 && Refusal.Length == 0 && Act != null;

    public bool Lit => Enabled && !Quiet && !NeedsTakeOver && Caution.Length == 0;

    public string ButtonLabel => EndsLabel.Length > 0 ? EndsLabel : NeedsTakeOver ? PlanVerbs.TakeOver(Verb) : Verb;

    public bool NextLeads => Secondary is { Lead: true, Enabled: true };

    public OwnerMark Mark => Verb.Length > 0 && Refusal.Length == 0 && !PressEnds && !NextLeads ? PageText.FooterMark(ShownOwner, FlownBy, PlannedBurns) : OwnerMark.None;

    public string AnswerStatus => Refusal.Length > 0 ? Refusal : Caution.Length > 0 ? Caution : Status;

    public string GoodWord => !AnswerWarns && !StatusPending && Status.StartsWith(PageText.PlanReadyWord, StringComparison.Ordinal) ? PageText.PlanReadyWord : "";

    public bool AnswerWarns => Refusal.Length > 0 || Caution.Length > 0;

    public bool AnswerInWarnInk => AnswerWarns && !Calm;

    public string ShownOutcome => AnswerWarns ? "" : Outcome;

    public string VerbCard
    {
        get
        {
            string detail = AnswerWarns ? Paragraphs(Outcome, Tooltip) : Tooltip;
            if (Refusal.Length > 0)
                return Paragraphs(Refusal, detail);
            if (Caution.Length == 0)
                return detail;
            string head = PressEnds ? PageText.OutlinePressEnds : PageText.OutlineStillWorks;
            return Paragraphs(Paragraphs(head, NextLeads && !PressEnds ? PageText.FlownByCard(ShownOwner) : ""), detail);
        }
    }

    private static string Paragraphs(string a, string b) => a.Length == 0 ? b : b.Length == 0 ? a : a + "\n\n" + b;

    /// <summary>The owner of the footer chip: the one the page set, else the one its who-flies text names.</summary>
    public Owner ShownOwner => Owner != Owner.None ? Owner : PageText.OwnerOfFlownBy(FlownBy);

    /// <summary>A status-only footer without a verb.</summary>
    public static CommitSpec StatusOnly(string status, bool pending = false, string flownBy = Logic.FlownBy.Nobody)
        => new("", flownBy, Status: status, StatusPending: pending);
}

internal sealed record CommitAction(string Verb, string Refusal, string Tooltip, Action? Act, bool Danger = true, bool Lead = false, bool Quiet = false)
{
    public bool Enabled => Verb.Length > 0 && Refusal.Length == 0 && Act != null;
}

internal sealed record StopSpec(StopTarget Target, string Label, string Outcome, string Detail, Action? Act)
{
    public static readonly StopSpec None = new(StopTarget.None, StopResolver.Label(StopTarget.None), "", "", null);

    public bool Enabled => Target != StopTarget.None && Act != null;
}

/// <summary>The verbs of the plan pages, as pure rules so they can be tested.</summary>
internal static class PlanVerbs
{
    public static string Create(int passes, string single = "CREATE BURN")
        => passes > 1 ? $"CREATE {passes} PASSES" : single;

    public static string TakeOver(string verb) => "TAKE OVER: " + verb;

    public static string EndsOnly(string stopLabel) => stopLabel.StartsWith("STOP ", StringComparison.Ordinal) ? "END " + stopLabel.Substring(5) : stopLabel;

    /// <summary>Who flies a plan of <paramref name="passes"/> passes.</summary>
    public static string FlownByFor(int passes) => passes > 1 ? FlownBy.AfcPlansStockAuto : FlownBy.StockAuto;

    public const string FlyBurn = "FLY BURN";

    /// <summary>The verb of a pass that waits for a press, because AFC only adds the burn of a pass and stock AUTO flies it after the press.</summary>
    public static string FlyPass(int pass) => "FLY PASS " + pass.ToString(System.Globalization.CultureInfo.InvariantCulture);

    public const string DeleteBurn = "DELETE BURN";

    /// <summary>PassCompletionPatch cancels the passes when their burn goes, so the passes after it do not come.</summary>
    public static string DeletePass(int pass) => "DELETE PASS " + pass.ToString(System.Globalization.CultureInfo.InvariantCulture);

    public const string Calculate = "SOLVE";
}

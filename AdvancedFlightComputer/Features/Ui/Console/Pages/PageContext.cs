using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Guidance;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Ui.Logic;
using AdvancedFlightComputer.Features.Ui.Pictures;
using Brutal.ImGuiApi;
using KSA;

namespace AdvancedFlightComputer.Features.Ui.Console.Pages;

/// <summary>What a page draws with: the frame facts, the picture height, whether folds start open, and whether the mode of the page flies this craft (FlightView).</summary>
internal readonly record struct PageContext(ConsoleFrame Frame, float PictureHeight, bool FoldsOpen, bool Flying = false)
{
    public Vehicle Subject => Frame.Subject!;

    public float Width => ConsoleStyle.ContentAvailWidth();

    public void ToInputs() => AfcConsole.ShowPane(PaneSide.Inputs);

    public void ToPicture() => AfcConsole.ShowPane(PaneSide.Picture);

    /// <summary>Returns whether the form can draw in this frame.</summary>
    public bool ToPlanForm() => AfcConsole.ShowPlanPane();

    public void PlanHead()
    {
        if (AfcConsole.HasPanes)
            ConsoleUi.SectionHead(FlightView.PlanName);
    }
}

internal static class PageKit
{
    public static void HowItFlies(string id, ReadOnlySpan<FoldPart> parts, int changed, bool openByDefault, string saying = "")
        => ConsoleUi.PartsFold(PageText.HowItFlies, FoldId(id), parts, changed, openByDefault, PageText.HowItFliesTip, saying);

    public static void Numbers(string id, ReadOnlySpan<FoldPart> parts, bool openByDefault)
        => ConsoleUi.PartsFold(PageText.Numbers, "Numbers" + id, parts, 0, openByDefault, PageText.NumbersTip);

    public static string FoldId(string id) => "How" + id;

    /// <summary>The CRAFT row that picks another craft to plan for, rebuilt from Program.VehiclesInFrame every frame, because lookup indices can move after a deregistration.</summary>
    public static void CraftRow(Vehicle source, Vehicle subject, List<TransferObject> buffer, string shapeId, string comboId, Action<Vehicle> pick)
    {
        buffer.Clear();
        foreach (Vehicle vehicle in Program.VehiclesInFrame)
            buffer.Add(new TransferObject(vehicle));
        bool shows = buffer.Count > 0 && FormRules.PlanForShows(buffer.Count, !ReferenceEquals(source, subject));
        ConsoleUi.NoteShape(shapeId, shows ? 1 : 0);
        if (!shows)
            return;
        TransferObject selected = new TransferObject(source);
        string before = selected.GetKey();
        if (ConsoleUi.ComboRow("CRAFT".AsSpan(), comboId.AsSpan(), ref selected, buffer)
            && selected.GetKey() != before && selected.Body is Vehicle picked)
            pick(picked);
    }

    /// <summary>One bar per pass of a preview, numbered from <paramref name="first"/>.</summary>
    public static void AddPassBars(List<BurnBar> bars, PassPreview[] passes, int first, bool firstLit, bool shortOfGoal)
    {
        double now = Universe.GetElapsedSeconds();
        for (int i = 0; i < passes.Length; i++)
            bars.Add(new BurnBar("Pass " + (first + i).ToString(Fmt.Inv), passes[i].DvVlf.Length(), passes[i].EstimatedBurnTimeSec, firstLit && i == 0,
                passes[i].BurnTime.Seconds() - now, shortOfGoal && i == passes.Length - 1));
    }

    /// <summary>The FLY BURN verb of a plan page once its node exists: the same event the stock BURN gauge sends, so stock AUTO flies the node, or the AFC RCS executor when the node resolves to RCS.</summary>
    public static CommitSpec FlyNextNode(Vehicle craft, string status)
    {
        if (VehicleControlOwnership.HolderOf(craft) == ControlClaimant.RcsTranslation)
            return CommitSpec.StatusOnly("RCS flies the burn", true, FlownBy.Afc);
        bool rcs = BurnModeInput.ResolvesToRcs(craft);
        Burn? next = BurnModeInput.NextNode(craft);
        CommitAction? delete = next != null
            ? new CommitAction(PlanVerbs.DeleteBurn, "", PageText.DeleteNodeTip, () => BurnModeInput.DeleteNode(craft, next))
            : null;
        return new CommitSpec(PlanVerbs.FlyBurn, rcs ? FlownBy.Afc : FlownBy.StockAuto, BurnModeInput.FlyNextRefusal(craft), status, false,
            rcs ? PageText.FlyRcsTip : PageText.FlyStockTip, Act: () => BurnModeInput.FlyWithAuto(craft), Owner: rcs ? Owner.Afc : Owner.Stock,
            Outcome: PageText.FlyOutcome, Secondary: delete);
    }

    public static CommitSpec FlyPass(Vehicle craft, MultiPassExecution exec)
    {
        string which = string.Create(Fmt.Inv, $"Pass {exec.PassIndex + 1} of {exec.PassCountTotal}");
        if (VehicleControlOwnership.HolderOf(craft) == ControlClaimant.RcsTranslation)
            return CommitSpec.StatusOnly(which + ": RCS flies it", true, FlownBy.Afc);
        if (BurnModeInput.StockAutoOnNode(craft))
            return CommitSpec.StatusOnly(which + ": stock AUTO flies it", true, FlownBy.AfcPlansStockAuto);
        return BurnModeInput.NextNode(craft) != null && ReferenceEquals(craft, Program.ControlledVehicle)
            ? CommitSpec.StatusOnly(which + " waits for you", false, FlownBy.You)
            : CommitSpec.StatusOnly(which, false, FlownBy.AfcPlansStockAuto);
    }

    public static CommitAction? PassToFly(Vehicle craft)
    {
        if (!MultiPassRegistry.TryGet(craft.Id, out MultiPassExecution? exec)
            || VehicleControlOwnership.HolderOf(craft) == ControlClaimant.RcsTranslation
            || BurnModeInput.StockAutoOnNode(craft)
            || BurnModeInput.NextNode(craft) == null
            || !ReferenceEquals(craft, Program.ControlledVehicle))
            return null;
        return new CommitAction(PlanVerbs.FlyPass(exec.PassIndex + 1), BurnModeInput.FlyNextRefusal(craft), PageText.FlyPassTip(BurnModeInput.ResolvesToRcs(craft)), () => BurnModeInput.FlyWithAuto(craft), Danger: false);
    }

    public static (string Verb, string Card, string Line) DeleteOf(Vehicle craft, Burn node, in CraftActivity a)
    {
        if (MultiPassRegistry.TryGet(craft.Id, out MultiPassExecution? exec) && ReferenceEquals(node, exec.CurrentBurn))
            return (PlanVerbs.DeletePass(exec.PassIndex + 1), PageText.DeletePassTip, PageText.DeletePassLine(exec.PassIndex + 1, exec.PassCountTotal));
        return a.DeorbitChainLive && GuidanceWindow.IsDeorbitBurn(node)
            ? (PlanVerbs.DeleteBurn, PageText.DeleteDeorbitTip, PageText.DeleteEndsLanding)
            : (PlanVerbs.DeleteBurn, PageText.DeleteNodeTip, "");
    }

    /// <summary>The footer while stock AUTO flies the next node: a pending status and DELETE BURN, as the head drawer offers it.</summary>
    public static CommitSpec StockAutoFlies(Vehicle craft)
    {
        Burn? next = BurnModeInput.NextNode(craft);
        CommitAction? delete = next != null
            ? new CommitAction(PlanVerbs.DeleteBurn, "", PageText.DeleteNodeTip, () => BurnModeInput.DeleteNode(craft, next))
            : null;
        return CommitSpec.StatusOnly("Stock AUTO flies the burn", true, FlownBy.StockAuto) with { Secondary = delete };
    }

    /// <summary>Whether the guidance pages can draw, and the footer they show when they cannot.</summary>
    public static bool GuidanceReady(in PageContext ctx, out CommitSpec refusal)
    {
        if (!SharedVehicleHooks.GuidanceEnabled)
        {
            ConsoleUi.MutedWrapped(GuidanceFeature.UnavailableReason);
            refusal = CommitSpec.StatusOnly("Guidance unavailable");
            return false;
        }
        if (!ctx.Frame.GuidanceOn)
        {
            ConsoleUi.MutedWrapped("Guidance is off. Turn it on in the AFC menu or in TUNE > PREFERENCES.");
            refusal = CommitSpec.StatusOnly("Guidance is off");
            return false;
        }
        refusal = CommitSpec.None;
        return true;
    }

    public static CommitAction? NextLink(in VerbFit fit, Vehicle? craft = null)
    {
        if (fit.NextPicksSite)
            return new CommitAction(GuidanceWindow.PickSiteArmed ? PageText.PickArmed : PageText.PickSite, "", GuidanceWindow.PickSiteArmed ? PageText.PickArmedTip : PageText.PickSiteTip, GuidanceWindow.TogglePickSite, Danger: false, Lead: true);
        if (fit.NextSolver is LandSolver solver && craft != null)
            return new CommitAction(LandSolvers.Label(solver) + " >", "", PageText.PickRowTip(solver), () => GuidanceWindow.PickLandSolver(craft, solver), Danger: false, Lead: true);
        if (fit.NextPage is not ConsolePage page)
            return null;
        string cue = ConsolePages.CueTarget(page, fit.NextWay is LandWay way ? ConsolePages.EnumKey(way) : "");
        string label = fit.NextLabel.Length > 0 ? fit.NextLabel : PageText.NextLabel(page, fit.NextWay);
        if (fit.NextWay == null && fit.NextLabel.Length == 0 && ConsoleHead.TakeCueFor(page) is StatusLine cued)
        {
            cue = cued.CueTarget;
            label = NextStep.LinkLabel(in cued);
        }
        return new CommitAction(label, "", ConsolePages.HoverCard(page), () => AfcConsole.Navigate(cue), Danger: false, Lead: true, Quiet: fit.NextQuiet);
    }

    /// <summary>
    /// The commit of a guidance page: the verb and refusal from CommitResolver over the registered
    /// modes, and the action that calls the mode's entry point through GuidanceWindow.Commit, which asks
    /// first when the commit takes the craft from another mode. <paramref name="readyText"/> is the status of a verb that fits, and <paramref name="readyPending"/> shows it as something AFC does now.
    /// </summary>
    public static CommitSpec GuidanceCommit(in PageContext ctx, GuidanceModeId? modeId, string readyText, string hint, bool readyPending = false)
    {
        if (modeId is not GuidanceModeId mode)
            return CommitSpec.StatusOnly(readyText.Length > 0 ? readyText : "Nothing to commit here", false, FlownBy.Afc);

        Vehicle subject = ctx.Subject;
        CommitGates gates = GuidanceWindow.CommitGatesFor(subject, mode, in ctx.Frame.Situation);
        if (ctx.Frame.FailureWhat.Length > 0 && PageText.FailureOf(ctx.Frame.FailureWhat, mode))
            gates = gates with { PageFact = PageText.TriesAgain, PageFactNext = null, PageFactSolver = null, PageFactCalm = true };
        CraftActivity activity = ctx.Frame.Activity;
        CommitView view = CommitResolver.Resolve(mode, in activity, in gates);
        StopTarget running = StopResolver.Resolve(in activity);

        string when = PageText.VerbWhen(mode);
        string tooltip = view.Detail.Length > 0 ? view.Detail + "\n\n" + when : when;
        if (hint.Length > 0)
            tooltip += "\n\n" + hint;
        string flownBy = FlownBy.Afc;
        // A deorbit from orbit starts with the burn stock AUTO flies, and from a fall AFC flies a direct approach or asks for that burn only if it needs one.
        Owner owner = mode == GuidanceModeId.DeorbitLand && !SituationText.IsFalling(ctx.Frame.Situation.Where) ? Owner.StockThenAfc : Owner.Afc;
        PageText.OutcomeFacts facts = GuidanceWindow.CommitOutcomeFacts(subject) with { Falling = SituationText.IsFalling(ctx.Frame.Situation.Where) };
        string outcome = PageText.GuidanceOutcome(mode, view.Verb, in facts);
        if (mode == GuidanceModeId.DeorbitLand && !view.NeedsTakeOver && outcome.Length > 0)
        {
            tooltip = tooltip.Length > 0 ? outcome + "\n\n" + tooltip : outcome;
            outcome = "";
        }
        if (!view.Enabled && view.Live)
        {
            CommitSpec flies = view.Reason == CommitResolver.StockDeorbitStatus
                ? CommitSpec.StatusOnly(view.Reason, true, FlownBy.StockAuto) with { Owner = Owner.Stock }
                : CommitSpec.StatusOnly(view.Reason, true, flownBy) with { Owner = Owner.Afc };
            return ctx.Frame.FailureWhat.Length > 0 ? flies with { Caution = ctx.Frame.FailureWhat + "." } : flies;
        }
        if (!view.Enabled)
            return new CommitSpec(view.Verb, flownBy, view.Reason, "", false, tooltip, Owner: owner, Outcome: outcome, Calm: gates.HoverTwrBelowOne);
        GuidanceModeId? registered = activity.RegisteredModeLive;
        string from = view.NeedsTakeOver ? CommitResolver.ModeName(running, registered) : "";
        string status = view.NeedsTakeOver ? "" : readyText;
        if (view.NeedsTakeOver)
            outcome = view.PressEnds ? "" : PageText.TakeOverOutcome(mode, from);
        string endsLabel = view.EndsOnly ? PlanVerbs.EndsOnly(StopResolver.Label(running, in activity)) : "";
        bool hoverRefused = mode == GuidanceModeId.Hover && gates.HoverProblem.Length > 0;
        string refused = GuidanceWindow.RefusedAnswer(mode, ctx.Frame.Lines, hoverRefused ? gates.HoverProblem : view.Fit.Text);
        return new CommitSpec(view.Verb, flownBy, "", status, readyPending, tooltip, view.NeedsTakeOver,
            () => GuidanceWindow.Commit(subject, mode, view, running, registered), owner, outcome,
            Secondary: NextLink(view.Fit, subject), TakeOverFrom: from, Caution: refused.Length > 0 ? refused : view.Fit.Text,
            Calm: (refused.Length == 0 || gates.HoverProblem == PageText.HoverNoThrust) && view.Fit.Calm,
            PressEnds: view.PressEnds || refused.Length > 0, HoldsFooter: view.EndsOnly, EndsLabel: endsLabel);
    }
}

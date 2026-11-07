using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Ui;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

internal static partial class OrbitToolsPanel
{
    internal enum CommitState
    {
        None,
        MultiPassRunning,
        NodeCreated,
        Blocked,
        Ready,
    }

    internal static CommitState ResolveCommitState(Vehicle source, string typeKey)
    {
        if (MultiPassRegistry.Has(source.Id))
            return CommitState.MultiPassRunning;

        if (_ourBurn != null)
        {
            if (_ourBurn.Time < Universe.GetElapsedTime() && !BurnModeInput.BurnFlies(source))
                _ourBurn = null;
            else
                return CommitState.NodeCreated;
        }

        return MultiPassUI.WantsMultiPassButCannot(typeKey)
            ? CommitState.Blocked
            : CommitState.Ready;
    }

    /// <summary>
    /// The one commit of the MANEUVER page. Before a node exists it creates one node or starts n passes; once its node exists, the answer row names that burn, the verb flies the next node with stock AUTO (or the RCS executor when the node resolves to RCS), the same event the BURN gauge sends, and DELETE removes the burn that was made. The refusals are the ones the preview shows.
    /// </summary>
    internal static CommitSpec BuildCommit(string key, Vehicle source, in Plan plan, string refusal, double budgetMs = double.NaN)
    {
        int passes = MultiPassUI.Enabled ? MultiPassUI.PassCount : 1;
        bool controlled = ReferenceEquals(source, Program.ControlledVehicle);
        switch (plan.State)
        {
            case CommitState.MultiPassRunning:
                return MultiPassRegistry.TryGet(source.Id, out MultiPassExecution? exec)
                    ? Ui.Console.Pages.PageKit.FlyPass(source, exec)
                    : CommitSpec.StatusOnly("Passes run", false, FlownBy.AfcPlansStockAuto);
            case CommitState.NodeCreated:
            {
                if (_ourBurn is not Burn made)
                    return CommitSpec.None;
                string status = PageText.BurnCreated(made.DeltaVVlf.Length(), FormatHelper.FormatDuration(Math.Max(0.0, (made.Time - Universe.GetElapsedTime()).Seconds())));
                if (!controlled)
                    return CommitSpec.StatusOnly(status, false, FlownBy.StockAuto);
                if (BurnModeInput.StockAutoOnNode(source))
                    return Ui.Console.Pages.PageKit.StockAutoFlies(source);
                CommitSpec fly = Ui.Console.Pages.PageKit.FlyNextNode(source, status);
                return fly.Verb.Length > 0
                    ? fly with { Secondary = new CommitAction(PlanVerbs.DeleteBurn, "", PageText.DeleteCreatedTip, () => BurnModeInput.DeleteNode(source, made)) }
                    : fly;
            }
            case CommitState.Blocked:
                return new CommitSpec(PlanVerbs.Create(passes), PlanVerbs.FlownByFor(passes), "Pass preview failed",
                    Owner: Owner.AfcPlansStockFlies, Outcome: Outcome(passes, null), PlannedBurns: passes);
        }

        if (plan.Maneuver is not OrbitManeuvers.ManeuverResult maneuver)
            return new CommitSpec(PlanVerbs.Create(passes), PlanVerbs.FlownByFor(passes), refusal.Length > 0 ? refusal : "No maneuver",
                Owner: Owner.AfcPlansStockFlies, Outcome: Outcome(passes, null));
        if (refusal.Length > 0)
            return new CommitSpec(PlanVerbs.Create(passes), PlanVerbs.FlownByFor(passes), refusal,
                Owner: Owner.AfcPlansStockFlies, Outcome: Outcome(passes, maneuver));

        bool multi = MultiPassUI.IsArmed(key);
        if (multi && plan.Basis.IsChained)
            return new CommitSpec(PlanVerbs.Create(passes), PlanVerbs.FlownByFor(passes), "Passes need a clear plan",
                Tooltip: "A multi-pass cannot start on the trajectory after a planned burn. Use one pass or delete the planned burn first.",
                Owner: Owner.AfcPlansStockFlies, Outcome: Outcome(passes, maneuver));

        Vehicle vehicle = source;
        string toolKey = key;
        PlanningBasis basis = plan.Basis;
        return new CommitSpec(PlanVerbs.Create(multi ? passes : 1), PlanVerbs.FlownByFor(multi ? passes : 1), "",
            Answer(multi ? passes : 1, maneuver, budgetMs, plan.Basis.IsChained), false, PageText.CreateCard("the goal", multi),
            Act: () =>
            {
                if (multi)
                    MultiPassController.Start(vehicle, toolKey);
                else
                    CreateSingleBurn(vehicle, maneuver, basis);
            }, Owner: Owner.AfcPlansStockFlies, Outcome: Outcome(multi ? passes : 1, maneuver), PlannedBurns: multi ? passes : 1);
    }

    private static string Outcome(int passes, OrbitManeuvers.ManeuverResult? maneuver)
        => passes > 1 ? PageText.PassesOutcome(passes) : "";

    private static string Answer(int passes, in OrbitManeuvers.ManeuverResult maneuver, double budgetMs, bool chained)
    {
        string when = passes > 1
            ? string.Create(Fmt.Inv, $"in {passes} passes")
            : "in " + FormatHelper.FormatDuration(Math.Max(0.0, (maneuver.BurnTime - Universe.GetElapsedTime()).Seconds()));
        return PageText.BurnAnswer(maneuver.DvCci.Length(), when, budgetMs, chained);
    }

    internal static void OnManeuverContextChanged()
    {
        _ourBurn = null;
    }

    private static void CreateSingleBurn(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, PlanningBasis basis)
    {
        Burn? burn = MultiPassCommitter.QueueAddBurn(
            source, maneuver.BurnTime, maneuver.DvVlf, basis.Plan);
        if (burn != null)
            _ourBurn = burn;
    }
}

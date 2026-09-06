using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.PlanWindow;

internal static partial class Patch_DrawPlanWindow
{
    private enum CommitState
    {
        None,
        MultiPassRunning,
        NodeCreated,
        Blocked,
        Ready,
    }

    private readonly record struct Commit(
        CommitState State, Vehicle? Source, OrbitManeuvers.ManeuverResult Maneuver,
        string TypeKey, PlanningBasis Basis);

    private static CommitState ResolveCommitState(Vehicle source, string typeKey)
    {
        if (MultiPassRegistry.Has(source.Id))
            return CommitState.MultiPassRunning;

        if (_ourBurn != null)
        {
            if (_ourBurn.Time < Universe.GetElapsedTime())
                _ourBurn = null;
            else
                return CommitState.NodeCreated;
        }

        return MultiPassUI.WantsMultiPassButCannot(typeKey)
            ? CommitState.Blocked
            : CommitState.Ready;
    }

    private static bool DrawFooter(CommitState state)
    {
        switch (state)
        {
            case CommitState.MultiPassRunning:
                ConsoleStyle.FooterStatus("MULTI-PASS RUNNING".AsSpan(), pending: true);
                return false;
            case CommitState.NodeCreated:
                ConsoleStyle.FooterStatus("NODE CREATED".AsSpan(), pending: false);
                return false;
            case CommitState.Blocked:
                ConsoleStyle.FooterWarning("MULTI-PASS PREVIEW FAILED".AsSpan());
                return false;
            case CommitState.Ready:
                ConsoleStyle.FooterStatus("MANEUVER READY".AsSpan(), pending: true);
                ConsoleStyle.FooterRightAlign(ConsoleWidgets.ButtonWidth("CREATE".AsSpan()));
                return ConsoleWidgets.PrimaryButton("CREATE".AsSpan());
            default:
                ConsoleStyle.FooterStatus("NO MANEUVER".AsSpan(), pending: false);
                return false;
        }
    }

    private static void CreateSingleOrMultiPass(in Commit commit)
    {
        if (MultiPassUI.IsArmed(commit.TypeKey))
        {
            if (commit.Basis.IsChained)
                TimedAlert.Create(
                    "Multi-pass cannot start on a pending burn's trajectory; " +
                    "use a single pass or clear the plan first.", Color.Yellow, 4.0);
            else
                MultiPassController.Start(commit.Source!, commit.TypeKey);
        }
        else
            CreateSingleBurn(commit.Source!, commit.Maneuver, commit.Basis);
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

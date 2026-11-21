using AdvancedFlightComputer.Features.Ui;
using AdvancedFlightComputer.Features.Ui.Logic;
using KSA;

namespace AdvancedFlightComputer.Features.Burns;

/// <summary>
/// A time warp to shortly before stock AUTO starts to turn the craft for the loaded node. The stock warp to the burn
/// (FlightComputerAction.WarpToNextBurn) runs to the ignition, past BurnTarget.AlignmentStartTime, so the craft turns
/// under warp. This warp ends <see cref="BurnTurnRules.WarpLeadSeconds"/> before the alignment start and leaves the
/// burn mode alone, so stock AUTO stays armed.
/// </summary>
/// <remarks>
/// The warp is AFC's only while Universe.AutoWarpTime equals the end it set. Any other warp replaces it through
/// Universe.AutoWarpTo or ends it through Universe.AutoWarpStop, and from then on AFC leaves the warp alone.
/// </remarks>
internal static class TurnWarp
{
    private sealed record Owned(UniverseTime End, Vehicle Craft, BurnTarget Target);

    private static Owned? _owned;

    // The alignment start may move a little while the craft coasts. A turn that moves this close to the end of the warp
    // stops it, so the craft still turns at 1x.
    private const double TurnMovedSeconds = 0.5 * BurnTurnRules.WarpLeadSeconds;

    /// <summary>Whether the warp that runs now is the one <see cref="Start"/> began.</summary>
    public static bool Running
    {
        get
        {
            if (_owned is { } owned && Universe.IsAutoWarpActive && Universe.AutoWarpTime == owned.End)
                return true;
            _owned = null;
            return false;
        }
    }

    /// <summary>Whether AFC's warp to the turn runs now for <paramref name="craft"/>.</summary>
    public static bool RunningFor(Vehicle craft) => Running && ReferenceEquals(_owned!.Craft, craft);

    /// <summary>
    /// The end of a warp to the turn of the node that stock AUTO flies on the controlled <paramref name="craft"/>, or null.
    /// Before AUTO, FlightComputer.UpdateBurnTarget times the burn with the manual throttle, so the alignment start is the
    /// one stock flies only once AUTO is on. A warp that another source started may be replaced by this one.
    /// </summary>
    public static UniverseTime? OfferedEnd(Vehicle craft)
    {
        if (Running || !BurnModeInput.WarpOffered(craft) || !BurnModeInput.StockAutoOnNode(craft)
            || craft.FlightComputer.Burn is not BurnTarget target || target.StartedAt.HasValue || target.AlignmentStartTime.IsEndOfTime()
            || InputEvents.AutoWarpBuffer.Count > 0 || InputEvents.SimSpeedBuffer.Count > 0)
            return null;
        UniverseTime now = Universe.GetElapsedTime();
        if (!BurnTurnRules.WarpWorthIt((target.AlignmentStartTime - now).Seconds()))
            return null;
        return target.AlignmentStartTime - new UniverseTime(BurnTurnRules.WarpLeadSeconds);
    }

    /// <summary>Starts the warp to the turn of <paramref name="craft"/>. Returns false when none is offered now.</summary>
    public static bool Start(Vehicle craft)
    {
        if (OfferedEnd(craft) is not UniverseTime end || craft.FlightComputer.Burn is not BurnTarget target)
            return false;
        // Uncapped as FlightComputerAction.WarpToNextBurn, so a long coast warps as fast as the stock warp to the burn.
        Universe.AutoWarpTo(end, 0.0, isUncapped: true);
        _owned = Universe.IsAutoWarpActive && Universe.AutoWarpTime == end ? new Owned(end, craft, target) : null;
        return _owned != null;
    }

    /// <summary>Stops the warp to the turn while it is still AFC's. Any other warp runs on.</summary>
    public static void Cancel()
    {
        if (Running)
            Universe.AutoWarpStop(resetSimulationSpeed: true);
        _owned = null;
    }

    /// <summary>
    /// Stops AFC's warp once it no longer ends before the turn it was started for: the node was removed or replaced, the
    /// burn started, AUTO was switched off, or the alignment start moved close to the end of the warp. Called once per
    /// frame from the solver step, which runs before the frame's queued input is applied, so a change made through input
    /// is seen one frame later and the warp advances for that one frame.
    /// </summary>
    public static void Guard()
    {
        if (!Running || _owned is not { } owned)
            return;
        Vehicle craft = owned.Craft;
        bool stillAhead = !craft.IsDisposed && ReferenceEquals(craft.FlightComputer.Burn, owned.Target)
            && craft.FlightComputer.BurnMode == FlightComputerBurnMode.Auto
            && !owned.Target.StartedAt.HasValue
            && (owned.Target.AlignmentStartTime - owned.End).Seconds() >= TurnMovedSeconds;
        if (!stillAhead)
            Cancel();
    }

    public static void Reset() => _owned = null;
}

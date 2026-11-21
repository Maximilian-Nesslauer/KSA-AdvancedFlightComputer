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
    private static UniverseTime? _ownedEnd;

    /// <summary>Whether the warp that runs now is the one <see cref="Start"/> began.</summary>
    public static bool Running
    {
        get
        {
            if (_ownedEnd is UniverseTime end && Universe.IsAutoWarpActive && Universe.AutoWarpTime == end)
                return true;
            _ownedEnd = null;
            return false;
        }
    }

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
        if (OfferedEnd(craft) is not UniverseTime end)
            return false;
        Universe.AutoWarpTo(end);
        _ownedEnd = Universe.IsAutoWarpActive && Universe.AutoWarpTime == end ? end : null;
        return _ownedEnd != null;
    }

    /// <summary>Stops the warp to the turn while it is still AFC's. Any other warp runs on.</summary>
    public static void Cancel()
    {
        if (Running)
            Universe.AutoWarpStop(resetSimulationSpeed: true);
        _ownedEnd = null;
    }

    public static void Reset() => _ownedEnd = null;
}

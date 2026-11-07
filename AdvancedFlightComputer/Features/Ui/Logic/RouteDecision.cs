namespace AdvancedFlightComputer.Features.Ui.Logic;

internal enum RouteAction { Pass, Route }

/// <summary>What a routed planner key press does to the console.</summary>
internal enum ConsoleToggle { Open, Close, OpenRunningPage, OpenEditor }

/// <summary>
/// The planner key routing as pure rules. A write to TransferPlanner.ShowPlanWindow is routed to the
/// AFC console only when it opens a closed stock window that AFC did not open itself, and only while
/// routing is on. Every close and the per-frame write of the open stock window pass unchanged.
/// </summary>
internal static class RouteDecision
{
    public static RouteAction Decide(bool value, bool current, bool stockLinkPending, bool active)
    {
        if (!value)
            return RouteAction.Pass;
        if (current)
            return RouteAction.Pass;
        if (stockLinkPending)
            return RouteAction.Pass;
        if (!active)
            return RouteAction.Pass;
        return RouteAction.Route;
    }

    /// <summary>
    /// Whether the console takes the key now: routing is on and the console can draw, which is in the
    /// editor or with a controlled craft. Without either the key opens the stock planner, as it would
    /// without AFC, instead of a console that cannot show.
    /// </summary>
    public static bool RoutesKey(bool active, bool editor, bool hasControlledVehicle)
        => active && (editor || hasControlledVehicle);

    /// <summary>
    /// The frame check behind the setter prefix: a stock window that is open although routing is on
    /// and AFC did not open it means the key press bypassed the prefix, so the console takes it over.
    /// </summary>
    public static bool FrameCheckRoutes(bool stockOpen, bool stockLinkActive, bool active)
        => stockOpen && !stockLinkActive && active;

    /// <summary>
    /// A stock window that opened while the key was not routed belongs to the player. The frame check
    /// marks it as holding the dock, so the console steps aside and a later routing does not close it.
    /// </summary>
    public static bool FrameCheckAdopts(bool stockOpen, bool stockLinkActive, bool active)
        => stockOpen && !stockLinkActive && !active;

    /// <summary>What one routed key press does, from the console state when it arrives.</summary>
    public static ConsoleToggle Toggle(bool consoleOpen, bool stripShown, bool editor)
    {
        if (consoleOpen)
            return ConsoleToggle.Close;
        if (editor)
            return ConsoleToggle.OpenEditor;
        if (stripShown)
            return ConsoleToggle.OpenRunningPage;
        return ConsoleToggle.Open;
    }
}

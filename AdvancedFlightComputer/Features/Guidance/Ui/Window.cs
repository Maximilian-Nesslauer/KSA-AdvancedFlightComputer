#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using KSA;

// The guidance UI's per-frame entry point: binds the ambient state to the focused vehicle and draws the gauge panel (Ui/Panel.cs), the tuning popups and the world-space overlays (Ui/Overlays). The modes live in Guidance/Modes, shared plumbing in Modes/Autopilot.cs.
public static partial class GuidanceWindow
{
    /// <summary>
    /// Disabling guidance queues release for each vehicle's next PrepareWorker prefix.
    /// Enabling it preserves settings but does not cancel a queued release or restart guidance.
    /// </summary>
    internal static void SetModActive(bool active)
    {
        if (_modActive == active)
            return;

        _modActive = active;

        if (!active)
            QueueAllReleases();
    }

    public static void Draw(IGameViewport viewport)
    {
        // SWITCHED OFF: draw nothing at all - no panel, no overlays, no warp prompt. The menu entry that turns it back on lives in the game's own menu bar (GuidanceFeature.DrawMenu), not in here, so this can go completely dark without becoming unreachable.
        //  Handing the vehicles back is NOT done here. This runs once per frame for the focused craft only, and the writes it would need are not legal from a draw.
        // ApplyAutopilot does it, per vehicle, from the prefix where they are.
        if (!ModActive)
            return;

        // One panel per frame gets to act on the armed auto-launch, whichever draws first - see DrawAutoLaunchArming.
        _autoLaunchStepped = false;

        Vehicle vehicle = AcquireVehicle();
        if (vehicle != null)
            DrawTrailingWindows(viewport, vehicle);
    }

    /// <summary>
    /// The focused vehicle, with the ambient state bound to it.
    ///  THE FRAME IS ABOUT THE FOCUSED VEHICLE. The sim thread points the ambient state at whichever craft it is servicing - routinely not this one now that a booster can fly itself home unattended - so the draw claims it back before reading or writing anything.
    /// </summary>
    private static Vehicle AcquireVehicle()
    {
        Vehicle vehicle = Program.ControlledVehicle;
        if (vehicle == null)
            return null;
        Use(vehicle);
        return vehicle;
    }

    // Everything that must be drawn AFTER the panel's window has closed: the per-domain tuning popups and the world-space overlays, which are their own ImGui windows and would otherwise nest inside the panel.
    private static void DrawTrailingWindows(IGameViewport viewport, Vehicle vehicle)
    {
        // Claim the ambient state again. AcquireVehicle left it pointing here, but these are separate ImGui windows, and every one of them reads per-vehicle configuration - so they say which vehicle they mean rather than inheriting it.
        Use(vehicle);

        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit.Parent;
        double bodyRadius = parent.MeanRadius;

        // FIRST. Everything below can throw, and GuidanceFeature.DrawGui catches the lot into one log line per fault - so anything drawn at the END of this method is starved by an unrelated fault upstream, and looks exactly like "my window doesn't work".
        DrawGuidancePanel(vehicle, orbit, parent, bodyRadius);

        // Per-domain tuning popups (each no-ops unless opened from its tab) and the G-FOLD debug plots. Ascent tuning is inline in its tab, not a popup.
        DrawGfoldParamsWindow();
        DrawTermParamsWindow();
        DrawGfoldDebugWindow();

        // WHICH TAB IS ON SCREEN in the gauge panel, while it is open. Each phase's overlay is drawn, and does its work, only while its own tab is - with the panel closed, or on another phase's tab, there is nothing to look at and nothing is computed for it. Guidance itself keeps running regardless of which tab is open.
        //  Named rather than inverted: the descent test previously read "anything but Ascent", which silently made every tab added afterwards a descent.
        bool ascentUi = PanelVisible && _panelTab == GuidanceTab.Ascent;
        bool boostbackUi = PanelVisible && _panelTab == GuidanceTab.Boostback;
        bool landingUi = PanelVisible && _panelTab == GuidanceTab.Landing;
        bool descentUi = landingUi || (PanelVisible && _panelTab == GuidanceTab.Descent);

        // World-space overlays (each its own full-screen window, drawn after the panel so they layer correctly). Each also no-ops unless its own toggle is on.
        if (ascentUi)
            DrawAscentOverlay(viewport, orbit, parent, bodyRadius);
        if (landingUi)
        {
            DrawGfoldOverlay(viewport, vehicle, orbit, parent);
            Draw6DofOverlay(viewport, parent);
        }
        if (boostbackUi)
            DrawBoostbackOverlay(viewport, vehicle, orbit, parent);

        // Landing-site marker: shown whenever a descent is on screen, so the target is visible for planning/UPFG, not only during a G-FOLD descent.
        //  AND ON BOOSTBACK, which the comment above previously say it had no business on.
        // That was true while the tab was only an aero workbench; the burn aims the predicted impact point at this same site, so the marker is the other half of the miss line the overlay draws and the thing RETARGET moves.
        if (descentUi || boostbackUi)
            DrawLandingSiteMarker(viewport, parent);

        // Clickable retargeting: while armed, a world click sets the new landing site.
        HandleRetargetClick(viewport, parent);
    }
}

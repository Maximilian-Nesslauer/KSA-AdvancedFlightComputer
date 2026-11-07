#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using KSA;

// The guidance side of the per-frame draw: the guidance off switch, the windows and world overlays drawn after the console window has ended, and the rule for when each overlay draws. The console pages live in Ui/Console, the modes in Guidance/Modes, shared plumbing in Modes/Autopilot.cs.
public static partial class GuidanceWindow
{
    /// <summary>The guidance pages a console can show, for the overlay visibility rule.</summary>
    internal enum GuidancePage { Ascent, Boostback, Deorbit, Land, Hover }

    /// <summary>
    /// Set by the console shell: whether a guidance page is on screen now. Null means no page is shown. Each overlay draws while its page is on screen or while its mode flies, so the work an overlay does runs for a craft that is being watched or flown and not otherwise.
    /// </summary>
    internal static Func<GuidancePage, bool> PageShownProbe;

    private static bool PageShown(GuidancePage page) => PageShownProbe?.Invoke(page) ?? false;

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

    // Everything that must be drawn AFTER the console window has ended: the world-space overlays, which are their own ImGui windows and would otherwise nest inside it, and the retarget click.
    private static void DrawTrailingWindows(IGameViewport viewport, Vehicle vehicle)
    {
        // These are separate ImGui windows, and every one of them reads per-vehicle configuration, so they say which vehicle they mean rather than inheriting it.
        Use(vehicle);

        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit.Parent;
        double bodyRadius = parent.MeanRadius;

        // Each overlay draws, and does its work, while its page is on screen or while its mode flies. Guidance itself keeps running regardless.
        bool ascentUi = PageShown(GuidancePage.Ascent) || _s.Running || _s.LaunchArmed;
        bool boostbackUi = PageShown(GuidancePage.Boostback) || BoostbackLive;
        bool landingUi = PageShown(GuidancePage.Land) || PageShown(GuidancePage.Hover) || LandingMachineLive
            || _s.LandingPhase == LandingPhase.TerminalHover || _s.Active || _s.EngagePending;
        bool descentUi = landingUi || PageShown(GuidancePage.Deorbit) || DescentLive || EntryLive(_s);

        // World-space overlays, each its own full-screen window, drawn after the console so they layer correctly. Each also does nothing unless its own toggle is on.
        if (ascentUi)
            DrawAscentOverlay(viewport, orbit, parent, bodyRadius);
        if (landingUi)
        {
            DrawGfoldOverlay(viewport, vehicle, orbit, parent);
            Draw6DofOverlay(viewport, parent);
        }
        if (boostbackUi)
            DrawBoostbackOverlay(viewport, vehicle, orbit, parent);

        // The landing site marker shows whenever a descent or a boostback is watched or flown: the burn aims the predicted impact point at this same site, so the marker is the other half of the miss line the boostback overlay draws and the thing a retarget click moves.
        if (descentUi || boostbackUi)
            DrawLandingSiteMarker(viewport, parent);

        // Clickable retargeting: while armed, a world click sets the new landing site.
        HandleRetargetClick(viewport, parent);
    }
}

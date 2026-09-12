#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The AFC Guidance panel: the console window every flight phase shares, the commit buttons, and
// the tab bar that switches between the phases. The per-tab content lives in Ui/Gauges, one file
// per phase.
//
// The shell is ConsoleStyle.BeginWindow, the skin the stock transfer planner and AFC's plan window
// use, so the panel moves, resizes, scrolls and closes like every other console window, and it can
// leave the main game window because it is not pinned to the main viewport. The body is plain
// ImGui inside ConsoleStyle.PushWidgetStyle, with ImGuiHelper.BeginRegion for the collapsible
// two-column sections.
public static partial class GuidanceWindow
{
    public enum GuidanceTab { Ascent, Boostback, Descent, Landing }
    public enum LandingSubTab { Powered, Hover }

    private const string PanelId = "afc-guidance";
    private const string PanelTitle = "AFC GUIDANCE";
    private const string PanelSignature = "AFC-GNC";

    private static float PanelWidthPx => 460f * ImGuiHelper.InterfaceScale;
    private static float PanelHeightPx => 720f * ImGuiHelper.InterfaceScale;

    /// <summary>
    /// Whether the panel is drawn. Off at start, so a game start shows no guidance window until
    /// the player opens it from the AFC Guidance menu. The window's close button and the legacy
    /// window's checkbox write it too. Hiding the panel does not stop guidance, that is the
    /// Enabled switch in the same menu.
    /// </summary>
    internal static bool PanelVisible;

    // Which tab the BUTTONS act on. They are drawn above the tab bar, so they read the selection
    // the bar made last frame, one frame of lag on a tab switch, and the alternative is drawing
    // the commit controls below the content they commit.
    private static GuidanceTab _panelTab = GuidanceTab.Ascent;
    private static LandingSubTab _landingSubTab = LandingSubTab.Powered;

    private static void DrawGuidancePanel(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                          double bodyRadius)
    {
        if (!PanelVisible)
            return;

        // Seed the LAN from where the vessel is right now. The legacy tab does this on its own
        // draw; without it here, a user who never opens that tab would launch toward LAN 0.
        if (!_s.LanSeeded)
        {
            _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, orbit.Parent);
            _s.LanSeeded = true;
        }

        // ConsoleStyle.BeginWindow closes the ImGui window itself when it returns false. The size
        // applies on the first use only, so a resize by the player sticks for the session.
        bool open = true;
        if (!ConsoleStyle.BeginWindow(PanelId, PanelTitle, PanelSignature, ref open,
                new float2(PanelWidthPx, PanelHeightPx), ImGuiWindowFlags.None,
                ImGuiCond.FirstUseEver, pinToMainViewport: false))
            return;

        try
        {
            if (!open)
            {
                PanelVisible = false;
                return;
            }

            ConsoleStyle.BeginBody();
            ConsoleStyle.PushWidgetStyle();
            try
            {
                DrawGuidancePanelBody(vehicle, orbit, parent, bodyRadius);
            }
            finally
            {
                ConsoleStyle.PopWidgetStyle();
                ConsoleStyle.EndBody();
            }

            // The release request stays available regardless of the selected tab, so it sits in
            // the footer where no scroll or fold can hide it.
            ConsoleStyle.BeginFooter();
            try
            {
                if (ImGui.Button("RELEASE GUIDANCE"))
                {
                    // The gimbal override needs its own release because it lives outside the
                    // flight computer.
                    _s.GimbalMode = 0;
                    KsaGimbalControl.Disengage(vehicle);
                    ResetFlightComputer();
                }
            }
            finally
            {
                ConsoleStyle.EndFooter();
            }
        }
        finally
        {
            ConsoleStyle.EndWindow();
        }
    }

    private static void DrawGuidancePanelBody(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                              double bodyRadius)
    {
        // Above the tabs deliberately: the commit controls are the one thing that must never be
        // behind a fold, a scroll or a tab switch.
        DrawPanelCommitButtons(vehicle, orbit, parent, bodyRadius);

        // Above the tabs because it is not a phase's concern: both the ascent launch window and
        // the deorbit burn request warps, and a prompt that vanished on a tab switch would strand
        // whichever flow was waiting on it.
        DrawWarpPrompt();

        // The guidance handed over to a powered descent: follow it, at both levels of the tab
        // bar, and make the solver shown agree with the one that actually started. Read here and
        // consumed after the bar, so both levels see it.
        bool followGfold = _s.GfoldTabSelectPending;

        if (ImGui.BeginTabBar("##panel_tabs"))
        {
            // Width is taken INSIDE each tab: the tab bar insets its content, and a region sized
            // to the panel's inner width would overhang it.
            if (ImGui.BeginTabItem("Ascent"))
            {
                _panelTab = GuidanceTab.Ascent;
                DrawAscentTabContent(vehicle, orbit, parent, bodyRadius,
                    ImGui.GetContentRegionAvail().X);
                ImGui.EndTabItem();
            }

            // Next to Ascent because that is the order they are flown in: a booster separates,
            // turns round, and boosts back. Its content is the aero workbench for now, see
            // Ui/Gauges/BoostbackGauge.cs.
            if (ImGui.BeginTabItem("Boostback"))
            {
                _panelTab = GuidanceTab.Boostback;
                DrawBoostbackTabContent(vehicle, orbit, parent,
                    ImGui.GetContentRegionAvail().X);
                ImGui.EndTabItem();
            }

            // The whole chain from orbit: coast, deorbit burn, powered descent, hover, touchdown.
            if (ImGui.BeginTabItem("Deorbit and land"))
            {
                _panelTab = GuidanceTab.Descent;
                DrawDescentTabContent(vehicle, orbit, parent, parent.Mu, bodyRadius,
                    ImGui.GetContentRegionAvail().X);
                ImGui.EndTabItem();
            }

            // Only the terminal part, started from wherever the craft is now, for a craft that
            // is already falling.
            if (ImGui.BeginTabItem("Land from here", followGfold || _s.TermTabSelectPending
                    ? ImGuiTabItemFlags.SetSelected : ImGuiTabItemFlags.None))
            {
                _panelTab = GuidanceTab.Landing;
                DrawLandingTabContent(vehicle, orbit, parent, bodyRadius,
                    ImGui.GetContentRegionAvail().X, followGfold);
                ImGui.EndTabItem();
            }

            ImGui.EndTabBar();
        }

        _s.GfoldTabSelectPending = false;

        if (_s.ReleaseError.Length > 0)
            ImGui.Text(_s.ReleaseError);
    }

    /// <summary>
    /// EXECUTE, ABORT and RETARGET on one row, dispatched to whichever phase the tab bar has
    /// selected. Every tab commits to something: ascent launches, boostback starts the
    /// separate, turn, burn and orient machine from the vehicle's current state, the deorbit tab
    /// starts the whole landing chain, and the land-from-here tab drops straight into the powered
    /// descent.
    /// </summary>
    private static void DrawPanelCommitButtons(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                               double bodyRadius)
    {
        float gap = ImGui.GetStyle().ItemSpacing.X;
        float2 size = new float2(
            MathF.Floor((ImGui.GetContentRegionAvail().X - gap * 2f) / 3f),
            ImGui.GetTextLineHeight() * 1.8f);
        float3 green = ColorRgbReference.GetIndexedRgb(IndexedColor.Green);
        float3 red = ColorRgbReference.GetIndexedRgb(IndexedColor.Red);
        float3 amber = ColorRgbReference.GetIndexedRgb(IndexedColor.Yellow);

        // EXECUTE lights green while that phase is actually doing something: guidance running or
        // a launch armed and waiting for its window on ascent, any live landing phase on the
        // deorbit tab. ABORT is red at all times, so it reads the same whether or not it currently
        // has anything to stop.
        bool lit = _panelTab == GuidanceTab.Ascent
            ? (_s.Running || _s.LaunchArmed)
            : _panelTab == GuidanceTab.Boostback ? BoostbackLive
            : _panelTab == GuidanceTab.Descent ? DescentLive
            : _landingSubTab == LandingSubTab.Hover
                ? _s.LandingPhase == LandingPhase.TerminalHover
                : _s.UseSixDofLanding
                    ? (_s.Active || _s.EngagePending)
                    : _s.LandingPhase == LandingPhase.GfoldDescent;

        if (TintedButton("EXECUTE", size, green, lit))
        {
            if (_panelTab == GuidanceTab.Boostback)
                ExecuteBoostback(vehicle, orbit, parent);
            else if (_panelTab == GuidanceTab.Ascent)
                ExecuteAscent(vehicle, orbit, parent);
            else if (_panelTab == GuidanceTab.Descent)
                ExecuteLanding(vehicle, orbit, parent, parent.Mu, bodyRadius);
            else if (_landingSubTab == LandingSubTab.Hover)
                StartTerminalHover(vehicle);
            else if (_s.UseSixDofLanding)
                Engage6Dof(vehicle);
            else
                StartGfoldNow(vehicle);
        }

        ImGui.SameLine();
        if (TintedButton("ABORT", size, red, true))
        {
            if (_panelTab == GuidanceTab.Boostback)
                AbortBoostback();
            else if (_panelTab == GuidanceTab.Ascent)
                AbortAscent();
            else if (_panelTab == GuidanceTab.Landing
                     && _landingSubTab == LandingSubTab.Powered && _s.UseSixDofLanding)
                Disengage6Dof(vehicle);
            else
                AbortLanding();
        }

        // RETARGET arms a world click that moves the landing site. It means nothing on Ascent,
        // because the site is what a landing aims at, with one exception: a returnable stage's
        // Set button arms a click for that stage, and that is set from the Ascent tab. So the
        // gate follows the binding rather than the tab alone. On Boostback the site is what the
        // correction aims the predicted impact point at, so moving it is how the burn is
        // retargeted.
        bool canRetarget = _panelTab != GuidanceTab.Ascent || _retargetStageId != 0;

        ImGui.SameLine();
        if (!canRetarget)
            ImGui.BeginDisabled();
        if (TintedButton(_retargetArmed ? "CLICK A SPOT" : "RETARGET", size, amber,
                _retargetArmed && canRetarget))
            _retargetArmed = !_retargetArmed;
        if (!canRetarget)
            ImGui.EndDisabled();
    }

    /// <summary>
    /// A button whose face carries the phase colour while lit and the stock button colour
    /// otherwise.
    /// </summary>
    private static bool TintedButton(string label, float2 size, float3 rgb, bool lit)
    {
        if (!lit)
            return ImGui.Button(label, size);

        ImGui.PushStyleColor(ImGuiCol.Button, new float4(rgb * 0.45f, 1f));
        ImGui.PushStyleColor(ImGuiCol.ButtonHovered, new float4(rgb * 0.6f, 1f));
        ImGui.PushStyleColor(ImGuiCol.ButtonActive, new float4(rgb * 0.75f, 1f));
        bool pressed = ImGui.Button(label, size);
        ImGui.PopStyleColor(3);
        return pressed;
    }

    /// <summary>
    /// The land-from-here tab. Two sub-tabs: the powered descent to the pad, and the hover the
    /// last few metres are flown on.
    /// </summary>
    private static void DrawLandingTabContent(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                             double bodyRadius, float innerW, bool selectPowered)
    {
        if (!ImGui.BeginTabBar("##landing_tabs"))
            return;

        if (ImGui.BeginTabItem("Powered landing", selectPowered
                ? ImGuiTabItemFlags.SetSelected : ImGuiTabItemFlags.None))
        {
            _landingSubTab = LandingSubTab.Powered;
            DrawPoweredLandingContent(vehicle, innerW);
            ImGui.EndTabItem();
        }

        // The hover controller sets its own focus flag when it takes over, the same way the
        // powered descent does.
        bool followHover = _s.TermTabSelectPending;
        if (ImGui.BeginTabItem("Hover", followHover
                ? ImGuiTabItemFlags.SetSelected : ImGuiTabItemFlags.None))
        {
            _landingSubTab = LandingSubTab.Hover;
            DrawHoverTabContent(vehicle, orbit, parent, parent.Mu, bodyRadius, innerW);
            ImGui.EndTabItem();
        }
        _s.TermTabSelectPending = false;

        ImGui.EndTabBar();
    }

    /// <summary>
    /// The powered-descent solver choice. One implementation, called from both landing tabs,
    /// because picking it does not depend on which page you happen to be on.
    /// </summary>
    private static void DrawSolverRadios()
    {
        if (ImGui.RadioButton("G-FOLD", !_s.UseSixDofLanding))
            _s.UseSixDofLanding = false;
        ImGui.SameLine();
        if (ImGui.RadioButton("6-DOF", _s.UseSixDofLanding))
            _s.UseSixDofLanding = true;
    }

    private static void DrawPoweredLandingContent(Vehicle vehicle, float innerW)
    {
        // Solver choice above the content, since it selects which content follows.
        ImGui.Text("Solver");
        ImGui.SameLine();
        DrawSolverRadios();
        ImGui.Separator();

        if (_s.UseSixDofLanding)
            Draw6DofLandingContent(vehicle, innerW);
        else
            DrawGfoldLandingContent(innerW, ImGui.GetTextLineHeightWithSpacing());
    }
}

using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The Powered Guidance panel: the gauge shell every flight phase shares, and the tab
// bar that switches between them. The per-tab content lives alongside —
// PoweredGuidanceAscentGauge.cs for Ascent, and placeholders below for the rest.
//
// Layering is stock's own, from KSA.GameSettings: an ImGauge window for the chrome,
// then an ordinary ImGui body wrapped in ImGaugeDressing.PushGaugeWidgetStyle so
// plain widgets take the gauge palette, with ImGuiHelper.BeginRegion for the
// collapsible two-column sections.
public static partial class PoweredGuidanceWindow
{
    public enum GuidanceTab { Ascent, Descent, Landing }
    public enum LandingSubTab { Powered, Hover }

    private static bool _showGuidancePanel = true;
    private static float2 _guidancePanelOffsetUv = new float2(0.012f, 0.06f);

    // Which tab the BUTTONS act on. They are drawn above the tab bar, so they read
    // the selection the bar made last frame — a frame's lag on a tab switch, and the
    // alternative is drawing the commit controls below the content they commit.
    private static GuidanceTab _panelTab = GuidanceTab.Ascent;
    private static LandingSubTab _landingSubTab = LandingSubTab.Powered;

    // Height is a floor only: the window is opened with fitContent, so it grows and
    // shrinks as tabs and sections change. Width is authored and stays put.
    private static readonly float2 GuidancePanelSizeUv = new float2(0.30f, 0.06f);

    private static void DrawGuidancePanel(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                          double bodyRadius)
    {
        if (!_showGuidancePanel)
            return;

        // Seed the LAN from where the vessel is right now. The legacy tab does this on
        // its own draw; without it here, a user who never opens that tab would launch
        // toward LAN 0.
        if (!_s.LanSeeded)
        {
            _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg);
            _s.LanSeeded = true;
        }

        // Rebuilt each frame so the dragged offset takes effect; BeginWindow keys its
        // Vulkan resources off the Id, not this struct.
        ImGaugeWindow win = new ImGaugeWindow(
            "NavboxGuidance", "Navbox Guidance",
            new float2(0f, 0f), new float2(0f, 0f),
            _guidancePanelOffsetUv, GuidancePanelSizeUv);

        // EndWindow is what uploads the instances, so it runs even on the false
        // branch — stock's own idiom, see KSA.Popup.BeginGaugeModal.
        if (!ImGauge.BeginWindow(in win, false, true, out float2 pos, out float2 size))
        {
            ImGauge.EndWindow();
            return;
        }

        try
        {
            DrawGuidancePanelBody(vehicle, orbit, parent, bodyRadius, pos, size);
        }
        finally
        {
            ImGauge.EndWindow();
        }
    }

    private static void DrawGuidancePanelBody(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                              double bodyRadius, float2 pos, float2 size)
    {
        float u = GaugeUnit();
        float margin = GaugeMarginUv * u;
        float spacing = GaugeSpacingUv * u;
        float headerH = GaugeHeaderHeightUv * u;
        float bigButtonH = 0.030f * u;          // half again the standard gauge button
        float innerW = size.X - margin * 2f;

        ImGauge.DrawDressedBox(pos, size);

        // --- header, which doubles as the drag handle ---
        float2 headerPos = pos + new float2(margin, GaugeHeaderTopUv * u);
        ImGauge.Label(headerPos, new float2(innerW, headerH), Gauge("POWERED GUIDANCE"),
            GaugeText(ColorRgbReference.GetIndexedRgb(IndexedColor.White), 0.8f));
        GaugeDrag("##paneldrag", ref _guidancePanelOffsetUv, pos, size,
            pos, new float2(size.X, headerPos.Y + headerH + spacing * 0.5f - pos.Y));

        // --- EXECUTE / ABORT ---
        // Above the tabs deliberately: the commit controls are the one thing that must
        // never be behind a fold, a scroll or a tab switch.
        DrawPanelCommitButtons(vehicle, orbit, parent, bodyRadius,
            new float2(pos.X + margin, headerPos.Y + headerH + spacing),
            innerW, bigButtonH, spacing);

        // --- body ---
        float bodyTop = headerPos.Y + headerH + spacing + bigButtonH * 2f + spacing * 2f;
        ImGui.SetCursorScreenPos(new float2(pos.X + margin, bodyTop));
        // AutoResizeY so the child is exactly as tall as its content; that height is
        // what feeds ReportContentExtent below, and so what the window fits itself to.
        ImGui.BeginChild("~PanelBody", new float2?(new float2(innerW, 0f)),
            ImGuiChildFlags.AutoResizeY, ImGuiWindowFlags.NoBackground);
        ImGaugeDressing.PushGaugeWidgetStyle();

        // Above the tabs because it is not a phase's concern: both the ascent launch
        // window and the deorbit burn request warps, and a prompt that vanished on a
        // tab switch would strand whichever flow was waiting on it.
        DrawWarpPrompt();

        // The guidance handed over to a powered descent: follow it, at both levels of
        // the tab bar, and make the solver shown agree with the one that actually
        // started. Read here and consumed after the bar, so both levels see it.
        bool followGfold = _s.GfoldTabSelectPending;

        if (ImGui.BeginTabBar("##panel_tabs"))
        {
            // Width is taken INSIDE each tab: the tab bar insets its content, and a
            // region sized to the panel's inner width would overhang it.
            if (ImGui.BeginTabItem("Ascent"))
            {
                _panelTab = GuidanceTab.Ascent;
                DrawAscentTabContent(vehicle, orbit, parent, bodyRadius,
                    ImGui.GetContentRegionAvail().X);
                ImGui.EndTabItem();
            }

            if (ImGui.BeginTabItem("Descent"))
            {
                _panelTab = GuidanceTab.Descent;
                DrawDescentTabContent(vehicle, orbit, parent, parent.Mu, bodyRadius,
                    ImGui.GetContentRegionAvail().X);
                ImGui.EndTabItem();
            }

            if (ImGui.BeginTabItem("Landing", followGfold || _s.TermTabSelectPending
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

        ImGaugeDressing.PopGaugeWidgetStyle();
        ImGui.EndChild();

        // Grow/shrink the window to the body. The reserve is the margin the content
        // extent doesn't include.
        ImGauge.SetFitReserve(new float2(margin, GaugeBottomMarginUv * u));
        ImGauge.ReportContentExtent(new float2(pos.X + size.X - margin, ImGui.GetItemRectMax().Y));
    }

    /// <summary>
    /// EXECUTE and ABORT, dispatched to whichever phase the tab bar has selected.
    /// Only Ascent is wired up; the others stripe out rather than disappearing, so
    /// the panel keeps one fixed shape as you move between tabs.
    /// </summary>
    private static void DrawPanelCommitButtons(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                               double bodyRadius, float2 origin, float innerW,
                                               float height, float gap)
    {
        float half = (innerW - gap) * 0.5f;
        float3 green = ColorRgbReference.GetIndexedRgb(IndexedColor.Green);
        float3 red = ColorRgbReference.GetIndexedRgb(IndexedColor.Red);

        // Every tab commits to something now: ascent launches, descent starts the
        // deorbit flow, landing drops straight into the powered descent from wherever
        // the vehicle currently is.
        const bool wired = true;

        // EXECUTE lights green while that phase is actually doing something: guidance
        // running or a launch armed and waiting for its window on ascent, any live
        // landing phase on descent. ABORT is red at all times — it should read the
        // same whether or not it currently has anything to stop.
        bool lit = _panelTab == GuidanceTab.Ascent
            ? (_s.Running || _s.LaunchArmed)
            : _panelTab == GuidanceTab.Descent ? DescentLive
            : _landingSubTab == LandingSubTab.Hover
                ? _s.LandingPhase == LandingPhase.TerminalHover
                : _s.UseSixDofLanding
                    ? (_s.Active || _s.EngagePending)
                    : _s.LandingPhase == LandingPhase.GfoldDescent;

        ImGui.SetCursorScreenPos(origin);
        if (ImGauge.Button("EXECUTE", new float2(half, height),
                GaugeButton(lit ? green : ImGaugeStyle.Default.IdleColor, 0.4f)
                    .WithDisabled(!wired)))
        {
            if (_panelTab == GuidanceTab.Ascent)
                ExecuteAscent(orbit, parent);
            else if (_panelTab == GuidanceTab.Descent)
                ExecuteLanding(vehicle, orbit, parent, parent.Mu, bodyRadius);
            else if (_landingSubTab == LandingSubTab.Hover)
                StartTerminalHover();
            else if (_s.UseSixDofLanding)
                // Consumed by the guidance step, not applied here: engaging runs a
                // cold solve, which belongs on the sim thread rather than the draw.
                _s.EngagePending = true;
            else
                StartGfoldNow();
        }

        ImGui.SetCursorScreenPos(new float2(origin.X + half + gap, origin.Y));
        if (ImGauge.Button("ABORT", new float2(half, height),
                GaugeButton(red, 0.4f).WithDisabled(!wired)))
        {
            if (_panelTab == GuidanceTab.Ascent)
                AbortAscent();
            else if (_panelTab == GuidanceTab.Landing
                     && _landingSubTab == LandingSubTab.Powered && _s.UseSixDofLanding)
                Disengage6Dof(vehicle);
            else
                AbortLanding();
        }

        // RETARGET, full width on its own row. It arms a world click that moves the
        // landing site, so it means nothing on Ascent and stripes out there.
        bool canRetarget = _panelTab != GuidanceTab.Ascent;
        float3 amber = ColorRgbReference.GetIndexedRgb(IndexedColor.Yellow);

        ImGui.SetCursorScreenPos(new float2(origin.X, origin.Y + height + gap));
        if (ImGauge.Button(_retargetArmed ? "CLICK A SPOT" : "RETARGET",
                new float2(innerW, height),
                GaugeButton(_retargetArmed && canRetarget ? amber : ImGaugeStyle.Default.IdleColor, 0.4f)
                    .WithDisabled(!canRetarget)))
            _retargetArmed = !_retargetArmed;
    }

    // --- Landing ------------------------------------------------------------

    /// <summary>
    /// PLACEHOLDER. Two sub-tabs: the powered descent to the pad, and the hover the
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

        // The hover controller sets its own focus flag when it takes over, the same
        // way the powered descent does.
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
    /// The powered-descent solver choice. One implementation, called from both the
    /// Descent and Landing tabs — the whole point is that picking it does not depend
    /// on which page you happen to be on.
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

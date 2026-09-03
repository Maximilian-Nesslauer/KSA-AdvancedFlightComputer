using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The Powered Guidance panel: the gauge shell every flight phase shares, and the tab
// bar that switches between them. The per-tab content lives alongside —
// Ui/Gauges/AscentGauge.cs for Ascent, and placeholders below for the rest.
//
// Layering: an ImGauge window for the chrome, then an ordinary ImGui body wrapped in
// ConsoleStyle.PushWidgetStyle so plain widgets take the stock palette, with
// ImGuiHelper.BeginRegion for the collapsible two-column sections.
//
// This used to be stock's own layering, copied from KSA.GameSettings. KSA 2026.8.19
// retired it: ImGaugeDressing is gone entirely and GameSettings now builds on the new
// ConsoleStyle skin instead. ImGauge itself survives as chrome (DrawDressedBox, Box,
// Screw, Label, Button), so the shell here is unchanged - but the helper that let
// plain ImGui widgets sit inside it is now ConsoleStyle's.
public static partial class PoweredGuidanceWindow
{
    public enum GuidanceTab { Ascent, Boostback, Descent, Landing }
    public enum LandingSubTab { Powered, Hover }

    private static bool _showGuidancePanel = true;
    private static float2 _guidancePanelOffsetUv = new float2(0.012f, 0.06f);

    // Which tab the BUTTONS act on. They are drawn above the tab bar, so they read
    // the selection the bar made last frame — a frame's lag on a tab switch, and the
    // alternative is drawing the commit controls below the content they commit.
    private static GuidanceTab _panelTab = GuidanceTab.Ascent;
    private static LandingSubTab _landingSubTab = LandingSubTab.Powered;

    // Width is authored and stays put. Height is measured, not authored - see
    // _panelHeightUv below - and this is only its floor.
    private const float GuidancePanelWidthUv = 0.30f;
    private const float MinPanelHeightUv = 0.06f;

    // The panel's own fit-to-content. ImGauge used to do this for us (BeginWindow took
    // a fitContent flag and the window grew to whatever ReportContentExtent last said);
    // KSA 2026.8.19 removed all three, and BeginWindow now sizes the window to exactly
    // SizeUv. So we measure the body ourselves at the end of the draw and feed the
    // result back here for the next frame - a frame of lag, which is the same deal the
    // body-scroll decision below already runs on.
    private static float _panelHeightUv = MinPanelHeightUv;

    // Body scrolling. The measured content height decides whether the body auto-sizes
    // or scrolls; the dead band keeps it from flipping between the two.
    private const float MinBodyHeightPx = 120f;
    private const float BodyScrollHysteresisPx = 24f;
    private static float _panelBodyContentH;

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
            _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, orbit.Parent);
            _s.LanSeeded = true;
        }

        // Rebuilt each frame so the dragged offset takes effect; BeginWindow keys its
        // Vulkan resources off the Id, not this struct.
        //
        // The Id is an IDENTITY rather than a label - it is what any persisted window
        // layout is filed under - so changing it is a one-off reset of this panel's
        // saved position, not a cosmetic edit. Done once, deliberately, to finish moving
        // off the obsolete "Navbox" name; it should not move again.
        ImGaugeWindow win = new ImGaugeWindow(
            "PoweredGuidance", "Powered Guidance",
            new float2(0f, 0f), new float2(0f, 0f),
            GaugeScreenUv(_guidancePanelOffsetUv),
            GaugeScreenUv(new float2(GuidancePanelWidthUv, _panelHeightUv)));

        // Since KSA 2026.8.19 BeginWindow takes no flags and always succeeds, so there
        // is no early-out to guard any more. EndWindow still has to run whatever the
        // body does - it is what uploads the gauge instances - hence the try/finally.
        ImGauge.BeginWindow(in win, out float2 pos, out float2 size);

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
        // The logo stands in for the title, drawn to the band the label used.
        float2 headerPos = pos + new float2(margin, GaugeHeaderTopUv * u);
        GaugeLogo(headerPos, new float2(innerW, headerH));
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

        // How much room is left below the header before running off the screen. The
        // window grows to the body (see the fit at the foot of this method), and that
        // fit is clamped to the screen - so past this height the window stops growing
        // whatever the body does. ImGauge.EndWindow used to apply that clamp itself;
        // since KSA 2026.8.19 the clamp is ours, and it has to stay in step with this
        // number or the body would never switch to scrolling.
        ImGuiViewportPtr vp = ImGui.GetMainViewport();
        float maxBodyH = MathF.Max(MinBodyHeightPx,
            vp.Pos.Y + vp.Size.Y - bodyTop - GaugeBottomMarginUv * u);

        // AutoResizeY while the content fits: the window shrinks and grows with the
        // sections, which is the whole point of the folds.
        //
        // Once it does NOT fit, the child has to become a fixed height and scroll.
        // AutoResizeY never scrolls - it just keeps growing - so with every section
        // open the body ran off the bottom of a window that had stopped growing, and
        // the wheel fell through to whatever was behind it. Measured last frame, with
        // hysteresis so it cannot flip modes on alternate frames.
        bool scrollBody = _panelBodyContentH > maxBodyH;

        ImGui.SetCursorScreenPos(new float2(pos.X + margin, bodyTop));
        if (scrollBody)
            ImGui.BeginChild("~PanelBody", new float2?(new float2(innerW, maxBodyH)),
                ImGuiChildFlags.None, ImGuiWindowFlags.NoBackground);
        else
            ImGui.BeginChild("~PanelBody", new float2?(new float2(innerW, 0f)),
                ImGuiChildFlags.AutoResizeY, ImGuiWindowFlags.NoBackground);
        ConsoleStyle.PushWidgetStyle();

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

            // Next to Ascent because that is the order they are flown in: a booster
            // separates, turns round, and boosts back. Its content is the aero
            // workbench for now - see Ui/Gauges/BoostbackGauge.cs.
            if (ImGui.BeginTabItem("Boostback"))
            {
                _panelTab = GuidanceTab.Boostback;
                DrawBoostbackTabContent(vehicle, orbit, parent,
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

        // RESET, the unconditional escape hatch. It used to sit under the legacy
        // window's tabs; with that window hidden this is the only way to reach it.
        //
        // Not a duplicate of ABORT. ABORT is per-mode and dispatches on whichever tab
        // is open, so it stops the thing it can see. This stops everything - guidance,
        // landing phase, armed auto-launch, the pending command - and cuts the engine
        // through FcResetPending, whatever state the mod believes it is in. It exists
        // for when a flow did not end cleanly on its own, which by definition is when
        // the mode-specific control is the one you cannot trust.
        //
        // At the FOOT of the body on purpose: it is a rarely used recovery, not a
        // commit control, and the top of the panel is reserved for the two buttons
        // that must never be behind a fold or a scroll.
        //
        // PLAIN ImGui.Button, not ImGauge.Button. The gauge buttons are a
        // gauge-WINDOW primitive - EXECUTE, ABORT and RETARGET are all drawn outside
        // the body child, positioned absolutely. Calling one inside an ImGui child
        // ends the child as if it were a window: "Must call EndChild() and not End()",
        // followed by a cascade of missing PopStyleColor as the unwound style stack
        // unbalances. Everything else drawn inside the body - the hover numpad, the
        // 6-DOF log controls - uses ImGui.Button for the same reason.
        ImGui.Dummy(new float2(0f, ImGui.GetTextLineHeight() * 0.5f));
        if (ImGui.Button("RESET FLIGHT COMPUTER",
                new float2(ImGui.GetContentRegionAvail().X, ImGui.GetTextLineHeight() * 1.6f)))
        {
            // The TVC override lives outside the flight computer, so a reset would
            // otherwise leave it silently driving the nozzles - same pairing the
            // legacy button used.
            _s.GimbalMode = 0;
            KsaGimbalControl.Disengage(vehicle);
            ResetFlightComputer();
        }

        ConsoleStyle.PopWidgetStyle();

        // Content height, read INSIDE the child where the cursor is content-relative.
        // Drives next frame's choice of mode; the dead band stops a body sitting right
        // on the limit from toggling between scrolling and not on alternate frames.
        float contentH = ImGui.GetCursorPosY();
        _panelBodyContentH = scrollBody && contentH < maxBodyH - BodyScrollHysteresisPx
            ? contentH
            : MathF.Max(contentH, scrollBody ? maxBodyH + 1f : contentH);

        ImGui.EndChild();

        // Grow/shrink the window to the body, for next frame. GetItemRectMax is the
        // body child we just ended, and the bottom margin is the reserve the child's
        // extent doesn't include - the same two numbers SetFitReserve/ReportContentExtent
        // were handed before KSA 2026.8.19 removed them.
        //
        // Clamped to the bottom of the screen, exactly where maxBodyH above assumes the
        // window stops: without the clamp the panel would keep growing off-screen and
        // the body would never flip to scrolling. UV here is isotropic in GaugeUnit()
        // for both orientations (ScreenReference scales the short axis by the aspect
        // ratio), so one divide converts back the way the rest of the file does.
        float fitH = ImGui.GetItemRectMax().Y + GaugeBottomMarginUv * u - pos.Y;
        fitH = MathF.Min(fitH, vp.Pos.Y + vp.Size.Y - pos.Y);
        _panelHeightUv = MathF.Max(MinPanelHeightUv, fitH / u);
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

        // Every tab commits to something now: ascent launches, boostback starts the
        // separate/turn/burn/orient machine from the vehicle's current state, descent
        // starts the deorbit flow, landing drops straight into the powered descent from
        // wherever the vehicle currently is. Nothing stripes out here any more.
        //
        // EXECUTE lights green while that phase is actually doing something: guidance
        // running or a launch armed and waiting for its window on ascent, any live
        // landing phase on descent. ABORT is red at all times — it should read the
        // same whether or not it currently has anything to stop.
        bool lit = _panelTab == GuidanceTab.Ascent
            ? (_s.Running || _s.LaunchArmed)
            : _panelTab == GuidanceTab.Boostback ? BoostbackLive
            : _panelTab == GuidanceTab.Descent ? DescentLive
            : _landingSubTab == LandingSubTab.Hover
                ? _s.LandingPhase == LandingPhase.TerminalHover
                : _s.UseSixDofLanding
                    ? (_s.Active || _s.EngagePending)
                    : _s.LandingPhase == LandingPhase.GfoldDescent;

        ImGui.SetCursorScreenPos(origin);
        if (ImGauge.Button("EXECUTE", new float2(half, height),
                GaugeButton(lit ? green : ImGaugeStyle.Default.IdleColor, 0.4f)))
        {
            if (_panelTab == GuidanceTab.Boostback)
                ExecuteBoostback(vehicle, orbit, parent);
            else if (_panelTab == GuidanceTab.Ascent)
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
                GaugeButton(red, 0.4f)))
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

        // RETARGET, full width on its own row. It arms a world click that moves the
        // landing site, so it means nothing on Ascent and stripes out there. It DOES
        // mean something on Boostback: the site is what the correction aims the
        // predicted impact point at, so moving it is how the burn is retargeted.
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

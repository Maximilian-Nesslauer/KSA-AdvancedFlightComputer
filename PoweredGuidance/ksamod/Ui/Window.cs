using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The in-game Powered Guidance panel: the window frame, the Ascent/Landing tab
// dispatch, and the shared status readout. The tabs themselves live in
// Guidance/Ascent.cs / Guidance/Landing.cs, the G-FOLD descent in
// Guidance/GfoldDescent.cs, shared plumbing in Guidance/Autopilot.cs, and the
// world-space overlay in Ui/Overlays/Overlay.cs.
public static partial class PoweredGuidanceWindow
{
    /// <summary>
    /// Whether the original ImGui window - the Ascent/Landing/Gimbal tab stack and the
    /// status readout - is drawn at all.
    ///
    /// Off: the gauge panel is the entire user interface. The code stays because the
    /// tabs remain the only place several diagnostics are written down, and turning
    /// this back on is how they are read.
    ///
    /// WHAT WENT WITH IT, having been checked rather than assumed:
    ///
    ///   KEPT - Use(vehicle), which binds the ambient _s to the focused craft. That
    ///   lived in DrawBody and everything downstream depends on it; it is in
    ///   AcquireVehicle now, on the path both branches take.
    ///
    ///   KEPT - the LAN seed. DrawAscentTab seeded _s.LanDeg on first sight of a
    ///   vehicle; PoweredGuidancePanel does the same thing already, so it survives.
    ///
    ///   KEPT - the tab-follow flags. GfoldTabSelectPending and TermTabSelectPending
    ///   are set outside these tabs and cleared by the gauge panel, so nothing latches.
    ///
    ///   KEPT - the guidance panel itself, the tuning popups, every world overlay and
    ///   the retarget click. All of those are in DrawTrailingWindows, which is reached
    ///   through the vehicle this returns rather than through the window.
    ///
    ///   KEPT - "Reset flight computer", which was a manual escape hatch and not
    ///   telemetry. It is a gauge row at the foot of the panel body now: ABORT is
    ///   per-mode and depends on the open tab, while this is unconditional.
    ///
    ///   LOST - the Gimbal tab, which is the only writer of a non-zero _s.GimbalMode.
    ///   With it hidden the mode stays 0, which is the state where the flight computer
    ///   has normal control of the engines, so the loss is inert as well as safe. Its
    ///   own text calls it a test tool rather than a guidance mode.
    ///
    ///   LOST - the status readout, the staged vehicle model and the per-tab numbers.
    ///   Telemetry, and the reason this is a flag rather than a deletion.
    /// </summary>
    internal static bool ShowLegacyWindow;

    /// <summary>
    /// Switch the mod on or off from the game's menu bar.
    ///
    /// Off does NOT tear anything down here. The writes that release attitude and cut
    /// the engine are only legal from the PrepareWorker prefix, and the per-vehicle
    /// state has no enumeration, so each craft hands itself back the next time that
    /// prefix runs for it - which is every sim step, so within a frame. See
    /// ApplyAutopilot and HandBackVehicle.
    ///
    /// Turning it back on clears nothing: settings, targets and tuning are all
    /// untouched. What does not come back is anything that was ENGAGED, because it was
    /// genuinely disengaged on the way out.
    /// </summary>
    internal static void SetModActive(bool active)
    {
        if (ModActive == active)
            return;

        ModActive = active;

        // A retarget click armed when the switch flipped would otherwise still be
        // waiting for a world click that can no longer be cancelled from anywhere.
        if (!active)
            _retargetArmed = false;
    }

    public static void Draw(IGameViewport viewport)
    {
        // SWITCHED OFF: draw nothing at all - no window, no panel, no overlays, no
        // warp prompt. The menu entry that turns it back on lives in the game's own
        // menu bar (Mod.OnDrawProgramMenus), not in here, so this can go completely
        // dark without becoming unreachable.
        //
        // Handing the vehicles back is NOT done here. This runs once per frame for the
        // focused craft only, and the writes it would need are not legal from a draw.
        // ApplyAutopilot does it, per vehicle, from the prefix where they are.
        if (!ModActive)
            return;

        // One panel per frame gets to act on the armed auto-launch, whichever draws
        // first — see DrawAutoLaunchArming.
        _autoLaunchStepped = false;

        // The End() is in a finally so that an exception anywhere below cannot leave
        // ImGui inside this window.
        //
        // ImGui keeps a window STACK, so an unwound Begin does not fail where the
        // fault is — it fails at the end of the frame, as "window Powered Guidance:
        // missing End", and then keeps failing every frame afterwards. That message
        // names this function no matter what actually threw, so the real fault (twice
        // now, a null KSA reference several calls deep) is completely hidden. The
        // exception still propagates and is still logged with its stack trace; this
        // only guarantees the ImGui stack is balanced on the way out, so what the
        // game reports is the actual error rather than a misleading structural one.
        Vehicle vehicle;
        if (ShowLegacyWindow)
        {
            vehicle = null;
            try
            {
                vehicle = DrawBody(viewport);
            }
            finally
            {
                ImGui.End();
            }
        }
        else
        {
            // The legacy window is hidden, but it was never ONLY a readout - it bound
            // the ambient state to the focused vehicle and it gated everything below.
            // AcquireVehicle is that half, kept on the live path; the ImGui window and
            // its tabs are what stop being drawn. Begin/End must stay paired, so this
            // branch does neither rather than skipping just the one.
            vehicle = AcquireVehicle();
            // Nothing sets this now, but it is a static that outlives a toggle: leave
            // it stale and the ascent overlay would hide itself for the rest of the
            // session. See DrawTrailingWindows.
            _landingTabActive = false;
            // The checkbox that owns this lives in the window we are not drawing, so
            // an unticked one would be unreachable AND would leave no interface at all.
            _showGuidancePanel = true;
        }

        // Skipped entirely if DrawBody threw — the exception propagates through the
        // finally above, so this is only reached on a clean frame.
        if (vehicle != null)
            DrawTrailingWindows(viewport, vehicle);
    }

    /// <summary>
    /// The window's contents. Returns the controlled vehicle, or null if there was
    /// none and nothing further should be drawn. Deliberately does NOT call End() —
    /// see Draw.
    /// </summary>
    /// <summary>
    /// The focused vehicle, with the ambient state bound to it - the part of DrawBody
    /// that is not drawing.
    ///
    /// THE FRAME IS ABOUT THE FOCUSED VEHICLE. The sim thread points the ambient state
    /// at whichever craft it is servicing - routinely not this one now that a booster
    /// can fly itself home unattended - so the draw claims it back before reading or
    /// writing anything. Shared by both branches of Draw so hiding the window cannot
    /// quietly drop it.
    /// </summary>
    private static Vehicle AcquireVehicle()
    {
        Vehicle vehicle = Program.ControlledVehicle;
        if (vehicle == null)
            return null;
        Use(vehicle);
        return vehicle;
    }

    private static Vehicle DrawBody(IGameViewport viewport)
    {
        ImGui.Begin("Powered Guidance", ImGuiWindowFlags.AlwaysAutoResize);

        Vehicle vehicle = AcquireVehicle();
        if (vehicle == null)
        {
            ImGui.Text("No controlled vehicle.");
            return null;
        }

        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit.Parent;
        double mu = parent.Mu;
        double bodyRadius = parent.MeanRadius;

        _landingTabActive = false;   // set true below only while the Landing tab is open

        if (ImGui.BeginTabBar("##navtabs"))
        {
            if (ImGui.BeginTabItem("Ascent"))
            {
                DrawAscentTab(vehicle, orbit, parent, bodyRadius);
                ImGui.EndTabItem();
            }

            if (ImGui.BeginTabItem("Landing"))
            {
                _landingTabActive = true;
                DrawLandingTab(vehicle, orbit, parent, mu, bodyRadius);
                ImGui.EndTabItem();
            }

            if (ImGui.BeginTabItem("Gimbal"))
            {
                DrawGimbalTab(vehicle);
                ImGui.EndTabItem();
            }

            ImGui.EndTabBar();
        }

        // Manual escape hatch: stop everything and hand the vehicle back, in case
        // a guidance flow didn't end cleanly on its own. Below the tabs so it's
        // reachable regardless of which one is open.
        if (ImGui.Button("Reset flight computer"))
        {
            // The TVC override lives outside the flight computer, so a reset would
            // otherwise leave it silently driving the nozzles.
            _s.GimbalMode = 0;
            KsaGimbalControl.Disengage(vehicle);
            ResetFlightComputer();
        }

        // The rebuilt panel — see Ui/Panel.cs. Its own gauge window, so
        // it is drawn from DrawTrailingWindows rather than here.
        ImGui.SameLine();
        ImGui.Checkbox("Guidance panel", ref _showGuidancePanel);

        // Any warp the mod wants needs the user's OK first. Drawn here only when the
        // gauge panel is not up: it renders the same prompt, and two of them would
        // both be live at once.
        if (!_showGuidancePanel)
            DrawWarpPrompt();

        // NOTHING IS STEPPED FROM THE DRAW. The ascent and landing flows used to run
        // here, which quietly made them focused-vehicle-only: the draw happens once
        // per frame for the craft the player is looking at, so any other vehicle's
        // guidance froze the moment the camera left it. They run from ApplyAutopilot
        // now — the per-vehicle PrepareWorker prefix — and this panel is purely a
        // readout of whichever flight computer is focused.

        if (_s.GuidanceError.Length > 0)
            ImGui.TextColored(new float4(1f, 0.4f, 0.4f, 1f), "Error: " + _s.GuidanceError);
        if (_s.Status.Length > 0)
            ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f), _s.Status);

        // The single-thread invariant the ambient state rests on, if it has ever been
        // seen to break - see PoweredGuidanceWindow._s. Not per-vehicle and not
        // clearable: once the sim step and the draw are on different threads, every
        // number on this panel is suspect and saying so once is the whole point.
        if (OwnerThreadViolation.Length > 0)
            ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f), "THREADING: " + OwnerThreadViolation);

        DrawStatusReadout(vehicle, orbit, bodyRadius);
        return vehicle;
    }

    // Everything that must be drawn AFTER the panel's window has closed: the
    // per-domain tuning popups and the world-space overlays, which are their own
    // ImGui windows and would otherwise nest inside the panel.
    private static void DrawTrailingWindows(IGameViewport viewport, Vehicle vehicle)
    {
        // Claim the ambient state again. DrawBody left it pointing here, but these are
        // separate ImGui windows drawn after it closed, and every one of them reads
        // per-vehicle configuration — so they say which vehicle they mean rather than
        // inheriting it.
        Use(vehicle);

        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit.Parent;
        double bodyRadius = parent.MeanRadius;

        // FIRST. Everything below can throw, and Mod.DrawGui catches the lot into a
        // Console.Error that goes nowhere under StarMap — so anything drawn at the
        // END of this method is starved by an unrelated fault upstream, and looks
        // exactly like "my window doesn't work".
        DrawGuidancePanel(vehicle, orbit, parent, bodyRadius);

        // Per-domain tuning popups (each no-ops unless opened from its tab) and the
        // G-FOLD debug plots. Ascent tuning is inline in its tab, not a popup.
        DrawGfoldParamsWindow();
        DrawTermParamsWindow();
        DrawGfoldDebugWindow();

        // Are we looking at a descent? Either window can say so — the legacy Landing
        // tab, or the gauge panel sitting on a descent tab. Both the ascent overlay
        // and the landing-site marker key off this, so that retargeting works from the
        // new panel and the two overlays don't clutter each other's view. Guidance
        // itself keeps running regardless of which tab is open.
        //
        // Named rather than inverted: this used to read "anything but Ascent", which
        // silently made every tab added afterwards a descent. Boostback is not one —
        // it has no landing site to mark and no retarget click to arm.
        bool descentUi = _landingTabActive
            || (_showGuidancePanel && (_panelTab == GuidanceTab.Descent
                                    || _panelTab == GuidanceTab.Landing));

        // World-space overlays (each its own full-screen window, drawn after the
        // panel so they layer correctly). Each no-ops unless toggled on.
        if (!descentUi)
            DrawAscentOverlay(viewport, orbit, parent, bodyRadius);
        DrawGfoldOverlay(viewport, vehicle, orbit, parent);
        Draw6DofOverlay(viewport, parent);
        // Not gated on descentUi: an impact prediction is worth seeing on the way UP
        // as well, and it no-ops unless its own toggle is on.
        DrawBoostbackOverlay(viewport, vehicle, orbit, parent);

        // Landing-site marker: shown whenever a descent is on screen, so the target is
        // visible for planning/UPFG, not only during a G-FOLD descent.
        //
        // AND ON BOOSTBACK, which the comment above used to say it had no business on.
        // That was true while the tab was only an aero workbench; the burn aims the
        // predicted impact point at this same site, so the marker is the other half of
        // the miss line the overlay draws and the thing RETARGET moves.
        if (descentUi || (_showGuidancePanel && _panelTab == GuidanceTab.Boostback))
            DrawLandingSiteMarker(viewport, parent);

        // Clickable retargeting: while armed, a world click sets the new landing site.
        HandleRetargetClick(viewport, parent);
    }

    // The shared readout below the tabs: guidance solution, staged vehicle model,
    // current vs target orbit, and what the autopilot is doing.
    private static void DrawStatusReadout(Vehicle vehicle, Orbit orbit, double bodyRadius)
    {
        ImGui.SeparatorText("Guidance");
        bool landingActive = _s.LandingPhase != LandingPhase.Idle;
        if (_s.Running || landingActive)
        {
            double3 r = orbit.StateVectors.PositionCci;
            double3 steer = _s.HasCommand ? _s.CommandDir : _s.Upfg.Steering;

            ImGui.Text(landingActive
                ? $"Phase: landing — {_s.LandingPhase} (UPFG mode {_s.Upfg.Mode})"
                : $"Phase: {PhaseName(_s.Phase)}");
            if (!landingActive && _s.Phase == AscentPhase.Terminal)
            {
                double remaining = _s.CutoffTime - SimNow();
                if (remaining > 0)
                    ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                        $"TERMINAL — attitude frozen, cutoff in {remaining,5:F1} s");
                else
                    ImGui.TextColored(new float4(1f, 0.3f, 0.3f, 1f),
                        "CUTOFF — kill throttle now");
            }
            else
            {
                ImGui.TextColored(
                    _s.Upfg.Converged ? new float4(0.4f, 1f, 0.4f, 1f) : new float4(1f, 0.8f, 0.3f, 1f),
                    _s.Upfg.Converged ? "CONVERGED" : "converging...");
                ImGui.Text($"Time-to-go:   {_s.Upfg.Tgo,8:F1} s");
                ImGui.Text($"dV-to-go:     {_s.Upfg.VgoMag,8:F1} m/s");
                if (_s.GLimitEnabled || landingActive)
                    ImGui.Text($"Throttle:     {_s.Upfg.Throttle * 100,8:F0} %");
            }

            (double pitchDeg, double headingDeg) = NavballSteerAngles(r, steer);
            ImGui.Text($"Steer pitch:   {pitchDeg,8:F1} deg (navball)");
            ImGui.Text($"Steer heading: {headingDeg,8:F1} deg (navball)");
        }
        else
        {
            ImGui.Text("Set toggles, then press EXECUTE to begin.");
        }

        // --- Staged vehicle model ---
        // While flying, show the list UPFG is actually steering on (post g-limit
        // split). While idle, show the snapshot the PrepareWorker prefix keeps
        // current anyway — so the staging can be checked on the pad, before
        // committing to a launch, at no extra cost.
        ImGui.SeparatorText("Vehicle stages (UPFG)");
        var stageList = (_s.Running || landingActive) ? _s.UpfgVehicle : _s.StageModel;
        if (stageList != null && stageList.Stages.Count > 0)
        {
            // seq/eng is provenance, not guidance: which staging sequence the arc came
            // out of and how many engine cores the game had burning across it. Two rows
            // carrying the same pair are one physical stage the drain simulation sliced
            // in two, which is worth being able to see at a glance.
            ImGui.Text("       thrust      Isp      wet      dry     burn        dV  seq/eng");
            double totalDv = 0.0;
            for (int i = 0; i < stageList.Stages.Count; i++)
            {
                PoweredGuidance.Upfg.UpfgStage s = stageList.Stages[i];
                double burnTime = s.Mode == 2
                    ? s.Isp * 9.80665 * System.Math.Log(s.MassTotal / s.MassDry) / (s.GLim * 9.80665)
                    : (s.MassTotal - s.MassDry) / (s.Thrust / (s.Isp * 9.80665));
                double stageDv = s.Isp * 9.80665 * System.Math.Log(s.MassTotal / s.MassDry);
                totalDv += stageDv;
                string marker = s.Mode == 2 ? " G" : "";
                ImGui.Text($"S{i + 1}  {s.Thrust / 1000.0,8:F0} kN {s.Isp,5:F0} s {s.MassTotal / 1000.0,7:F1} t {s.MassDry / 1000.0,7:F1} t {burnTime,5:F0} s {stageDv,7:F0}{marker}   {s.Seq,2}/{s.Engines,-2}");
            }
            ImGui.Text($"Total remaining dV: {totalDv,8:F0} m/s");

            // Cross-checks against the game's own model. The stage list comes from
            // KSA's staging simulator (the same one behind the in-game stage menu),
            // so these two are the ways it can silently disagree with reality.
            if (PoweredGuidance.Upfg.KsaVehicleAdapter.AnyAtmosphericSequence(vehicle))
                ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                    "A sequence is set to Atmospheric: its figures are sea-level, not vacuum.");

            double modelMass = PoweredGuidance.Upfg.KsaVehicleAdapter.CurrentStageWetMass(vehicle);
            double liveMass = vehicle.TotalMass;
            if (modelMass > 0 && liveMass > 0
                && System.Math.Abs(modelMass - liveMass) > 0.005 * liveMass)
            {
                ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                    $"Stage mass {modelMass / 1000.0:F1} t vs vehicle {liveMass / 1000.0:F1} t "
                    // %% because TextColored is printf-formatted native-side: the C#
                    // overload takes one string, which ImGui passes as its FORMAT
                    // argument. A lone "%)" is an invalid conversion specifier and
                    // reads a vararg that was never pushed. ImGui.Text is exempt (it
                    // maps to igTextUnformatted); TextColored and TextWrapped are not.
                    + $"({(modelMass - liveMass) / liveMass * 100.0:+0.0;-0.0} %%)");
            }
        }
        else
        {
            ImGui.Text("No staged model — the vehicle has no sequenced engines.");
        }

        // --- Current vs target ---
        ImGui.SeparatorText("Orbit (altitude)");
        ImGui.Text($"            current     target");
        ImGui.Text($"Periapsis  {(orbit.Periapsis - bodyRadius) / 1000.0,8:F1}   {_s.PeKm,8:F1} km");
        ImGui.Text($"Apoapsis   {(orbit.Apoapsis - bodyRadius) / 1000.0,8:F1}   {_s.ApKm,8:F1} km");
        ImGui.Text($"Inclination{PoweredGuidance.Upfg.UpfgTarget.RadToDeg(orbit.Inclination),8:F2}   {_s.IncDeg,8:F2} deg");

        // --- Autopilot ---
        ImGui.SeparatorText("Autopilot");
        if (_s.Engage && _s.Running && _s.HasCommand)
        {
            // The actual flight-computer writes happen in ApplyAutopilot, from the
            // Harmony prefix just before the sim snapshots the FC (Vehicle.
            // PrepareWorker). Writing from here — the UI draw — lands in the
            // window where the sim's copy-back erases it.
            float errDeg = (float)(vehicle.FlightComputer.ErrorAngles.Length() * 180.0 / System.Math.PI);
            ImGui.Text($"Flying {PhaseName(_s.Phase)} attitude. Error: {errDeg:F1} deg");
            ImGui.TextColored(new float4(0.7f, 0.7f, 0.7f, 1f), _s.AutoStage
                ? (_s.CutoffDone
                    ? "(Auto: engines cut off — done.)"
                    : (_s.StagingActive
                        ? "(Auto: STAGING — firing sequences until thrust returns.)"
                        : "(Auto: engines on, full throttle, staging at burnout.)"))
                : "(Steering only — throttle and staging are manual.)");
        }
        else if (_s.Engage && _s.Running)
        {
            ImGui.Text("Waiting for a steering solution...");
        }
        else
        {
            ImGui.Text("Autopilot disengaged.");
        }
    }

}

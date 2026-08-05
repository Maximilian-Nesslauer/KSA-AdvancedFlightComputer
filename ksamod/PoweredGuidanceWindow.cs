using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

// The in-game Powered Guidance panel: the window frame, the Ascent/Landing tab
// dispatch, and the shared status readout. The tabs themselves live in
// PoweredGuidanceAscent.cs / PoweredGuidanceLanding.cs, the G-FOLD descent in
// PoweredGuidanceGfold.cs, shared plumbing in PoweredGuidanceCommon.cs, and the
// world-space overlay in PoweredGuidanceOverlay.cs.
public static partial class PoweredGuidanceWindow
{
    public static void Draw(Viewport viewport)
    {
        ImGui.Begin("Powered Guidance", ImGuiWindowFlags.AlwaysAutoResize);

        Vehicle vehicle = Program.ControlledVehicle;
        if (vehicle == null)
        {
            ImGui.Text("No controlled vehicle.");
            ImGui.End();
            return;
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
            _gimbalMode = 0;
            KsaGimbalControl.Disengage();
            ResetFlightComputer();
        }

        // Any warp the mod wants needs the user's OK first.
        DrawWarpPrompt();

        // --- Run the flows (regardless of visible tab) ---
        StepLanding(vehicle, orbit, parent, mu, bodyRadius);
        StepAscent(vehicle, orbit, parent, mu, bodyRadius);

        if (_error.Length > 0)
            ImGui.TextColored(new float4(1f, 0.4f, 0.4f, 1f), "Error: " + _error);
        if (_status.Length > 0)
            ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f), _status);

        DrawStatusReadout(vehicle, orbit, bodyRadius);

        ImGui.End();

        // Per-domain tuning popups (each no-ops unless opened from its tab) and the
        // G-FOLD debug plots. Ascent tuning is inline in its tab, not a popup.
        DrawGfoldParamsWindow();
        DrawTermParamsWindow();
        DrawGfoldDebugWindow();

        // World-space overlays (each its own full-screen window, drawn after the
        // panel so they layer correctly). Each no-ops unless toggled on. The
        // ascent overlay is also hidden while the Landing tab is open — the same
        // way the G-FOLD overlay only appears for landing — so the two don't
        // clutter each other's view. Ascent guidance keeps running regardless.
        if (!_landingTabActive)
            DrawAscentOverlay(viewport, orbit, parent, bodyRadius);
        DrawGfoldOverlay(viewport, vehicle, orbit, parent);
        Draw6DofOverlay(viewport, parent);

        // Landing-site marker: shown whenever the Landing tab is open, so the target is
        // visible for planning/UPFG, not only during a G-FOLD descent.
        if (_landingTabActive)
            DrawLandingSiteMarker(viewport, parent);

        // Clickable retargeting: while armed, a world click sets the new landing site.
        HandleRetargetClick(viewport, parent);
    }

    // The shared readout below the tabs: guidance solution, staged vehicle model,
    // current vs target orbit, and what the autopilot is doing.
    private static void DrawStatusReadout(Vehicle vehicle, Orbit orbit, double bodyRadius)
    {
        ImGui.SeparatorText("Guidance");
        bool landingActive = _landingPhase != LandingPhase.Idle;
        if (_running || landingActive)
        {
            double3 r = orbit.StateVectors.PositionCci;
            double3 steer = _hasCommand ? _commandDir : Guidance.Steering;

            ImGui.Text(landingActive
                ? $"Phase: landing — {_landingPhase} (UPFG mode {Guidance.Mode})"
                : $"Phase: {PhaseName(_phase)}");
            if (!landingActive && _phase == AscentPhase.Terminal)
            {
                double remaining = _cutoffTime - SimNow();
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
                    Guidance.Converged ? new float4(0.4f, 1f, 0.4f, 1f) : new float4(1f, 0.8f, 0.3f, 1f),
                    Guidance.Converged ? "CONVERGED" : "converging...");
                ImGui.Text($"Time-to-go:   {Guidance.Tgo,8:F1} s");
                ImGui.Text($"dV-to-go:     {Guidance.VgoMag,8:F1} m/s");
                if (_gLimitEnabled || landingActive)
                    ImGui.Text($"Throttle:     {Guidance.Throttle * 100,8:F0} %");
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
        var stageList = (_running || landingActive) ? _upfgVehicle : _stageModel;
        if (stageList != null && stageList.Stages.Count > 0)
        {
            ImGui.Text("       thrust      Isp      wet      dry     burn        dV");
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
                ImGui.Text($"S{i + 1}  {s.Thrust / 1000.0,8:F0} kN {s.Isp,5:F0} s {s.MassTotal / 1000.0,7:F1} t {s.MassDry / 1000.0,7:F1} t {burnTime,5:F0} s {stageDv,7:F0}{marker}");
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
                    + $"({(modelMass - liveMass) / liveMass * 100.0:+0.0;-0.0} %)");
            }
        }
        else
        {
            ImGui.Text("No staged model — the vehicle has no sequenced engines.");
        }

        // --- Current vs target ---
        ImGui.SeparatorText("Orbit (altitude)");
        ImGui.Text($"            current     target");
        ImGui.Text($"Periapsis  {(orbit.Periapsis - bodyRadius) / 1000.0,8:F1}   {_peKm,8:F1} km");
        ImGui.Text($"Apoapsis   {(orbit.Apoapsis - bodyRadius) / 1000.0,8:F1}   {_apKm,8:F1} km");
        ImGui.Text($"Inclination{PoweredGuidance.Upfg.UpfgTarget.RadToDeg(orbit.Inclination),8:F2}   {_incDeg,8:F2} deg");

        // --- Autopilot ---
        ImGui.SeparatorText("Autopilot");
        if (_engage && _running && _hasCommand)
        {
            // The actual flight-computer writes happen in ApplyAutopilot, from the
            // Harmony prefix just before the sim snapshots the FC (Vehicle.
            // PrepareWorker). Writing from here — the UI draw — lands in the
            // window where the sim's copy-back erases it.
            float errDeg = (float)(vehicle.FlightComputer.ErrorAngles.Length() * 180.0 / System.Math.PI);
            ImGui.Text($"Flying {PhaseName(_phase)} attitude. Error: {errDeg:F1} deg");
            ImGui.TextColored(new float4(0.7f, 0.7f, 0.7f, 1f), _autoStage
                ? (_cutoffDone
                    ? "(Auto: engines cut off — done.)"
                    : (_stagingActive
                        ? "(Auto: STAGING — firing sequences until thrust returns.)"
                        : "(Auto: engines on, full throttle, staging at burnout.)"))
                : "(Steering only — throttle and staging are manual.)");
        }
        else if (_engage && _running)
        {
            ImGui.Text("Waiting for a steering solution...");
        }
        else
        {
            ImGui.Text("Autopilot disengaged.");
        }
    }

}

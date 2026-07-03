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

            ImGui.EndTabBar();
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
        // G-FOLD debug plots.
        DrawAscentParamsWindow();
        DrawGfoldParamsWindow();
        DrawTermParamsWindow();
        DrawGfoldDebugWindow();

        // World-space G-FOLD debug overlay (its own full-screen window, drawn after
        // the panel so it layers correctly). No-ops unless toggled on with a plan up.
        DrawGfoldOverlay(viewport, vehicle, orbit, parent);

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
        ImGui.SeparatorText("Vehicle stages (UPFG)");
        if (_upfgVehicle != null && _upfgVehicle.Stages.Count > 0)
        {
            ImGui.Text("       thrust      Isp      wet      dry     burn");
            for (int i = 0; i < _upfgVehicle.Stages.Count; i++)
            {
                PoweredGuidance.Upfg.UpfgStage s = _upfgVehicle.Stages[i];
                double burnTime = s.Mode == 2
                    ? s.Isp * 9.80665 * System.Math.Log(s.MassTotal / s.MassDry) / (s.GLim * 9.80665)
                    : (s.MassTotal - s.MassDry) / (s.Thrust / (s.Isp * 9.80665));
                string marker = s.Mode == 2 ? " G" : "";
                ImGui.Text($"S{i + 1}  {s.Thrust / 1000.0,8:F0} kN {s.Isp,5:F0} s {s.MassTotal / 1000.0,7:F1} t {s.MassDry / 1000.0,7:F1} t {burnTime,5:F0} s{marker}");
            }
        }
        else
        {
            ImGui.Text("No staged model yet — EXECUTE builds it.");
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

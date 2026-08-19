using System;
using System.Collections.Generic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using PoweredGuidance.Upfg;

// The Landing tab and its state machine (UPFG modes 2/3 + the G-FOLD handoff).
// Flow: EXECUTE runs Mode 2 synchronously to convergence to measure how far
// downrange the braking burn reaches, finds when the along-track distance to
// the site shrinks to (factor × that distance), asks to warp there, converges
// Mode 3 during a prep window, then burns with UPFG's throttle command driving
// the cutoff to zero speed over the site.
// Burn = UPFG braking to the high gate; GfoldDescent = convex (G-FOLD) powered
// descent from the gate to the surface (see PoweredGuidanceGfold.cs).
public static partial class PoweredGuidanceWindow
{
    // Public because VehicleAutopilotState holds a vehicle's phase: every craft runs
    // this machine on its own, so the phase is a field on the flight computer rather
    // than one global "the landing".
    public enum LandingPhase { Idle, Coast, Prep, Burn, GfoldDescent, TerminalHover, Done }

    // The site, the approach shaping (downrange factor, gate altitude/uprange, sink
    // rate) and the whole pass scan live on the vehicle — see VehicleAutopilotState.
    private const double PrepLeadTime = 30.0;      // converge + point before ignition

    // Upcoming site passes (time from now, closest ground distance). The scan is
    // ~1200 conic propagations, so it is time-sliced: a fixed sample budget per
    // frame, CSE warm-started between (sequential) samples, and the per-orbit
    // minimum sharpened by parabolic interpolation instead of a refinement pass.
    // Refresh gating is WALL clock, not sim time: under time warp a sim-time gate
    // elapses instantly, which made the scan restart back-to-back every frame and
    // tanked the frame rate. Scanning is also suspended above MaxScanSimSpeed —
    // the results are presentation-only, so there is no reason to pay for them
    // exactly when frames are most expensive.
    private const long PassRefreshIntervalMs = 5000;
    private const double MaxScanSimSpeed = 4.0;
    private const int PassesToShow = 5;
    public const int ScanSamplesPerOrbit = 240;
    private const int ScanSamplesPerFrame = 48;

    // The Landing tab body: a Deorbit sub-tab (UPFG braking to the gate) and a
    // G-FOLD sub-tab (terminal descent). Each carries its own relevant parameters;
    // shared landing status is below the sub-tab bar.
    private static void DrawLandingTab(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                       double mu, double bodyRadius)
    {
        if (ImGui.BeginTabBar("##landingtabs"))
        {
            if (ImGui.BeginTabItem("Deorbit"))
            {
                DrawDeorbitSubTab(vehicle, orbit, parent, mu, bodyRadius);
                ImGui.EndTabItem();
            }

            var gfoldFlags = _s.GfoldTabSelectPending
                ? ImGuiTabItemFlags.SetSelected
                : ImGuiTabItemFlags.None;
            _s.GfoldTabSelectPending = false;   // one-shot: the flag selects this frame
            if (ImGui.BeginTabItem("G-FOLD", gfoldFlags))
            {
                DrawGfoldSubTab(vehicle, orbit, parent, mu, bodyRadius);
                ImGui.EndTabItem();
            }

            var termFlags = _s.TermTabSelectPending
                ? ImGuiTabItemFlags.SetSelected
                : ImGuiTabItemFlags.None;
            _s.TermTabSelectPending = false;    // one-shot
            if (ImGui.BeginTabItem("Terminal", termFlags))
            {
                DrawTerminalTab(vehicle, orbit, parent, mu, bodyRadius);
                ImGui.EndTabItem();
            }

            if (ImGui.BeginTabItem("6dof"))
            {
                Draw6DofTab(vehicle, parent, bodyRadius);
                ImGui.EndTabItem();
            }

            ImGui.EndTabBar();
        }

        DrawLandingStatus(vehicle);
    }

    // Deorbit sub-tab: the landing site, upcoming passes, approach tuning, and the
    // deorbit-burn commit.
    private static void DrawDeorbitSubTab(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                          double mu, double bodyRadius)
    {
        if (ImGui.CollapsingHeader("Landing site"))
        {
            ImGui.InputDouble("Latitude (deg)", ref _s.SiteLatDeg);
            ImGui.InputDouble("Longitude (deg)", ref _s.SiteLonDeg);
        }

        if (ImGui.CollapsingHeader("Approach parameters", ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.InputDouble("Downrange factor", ref _s.DownrangeFactor);
            ImGui.InputDouble("Aim altitude (km)", ref _s.AimAltKm);
            ImGui.InputDouble("Descent rate (m/s)", ref _s.DescentRate);
            ImGui.InputDouble("Gate uprange (km)", ref _s.GateUprangeKm);
            // Where the braking burn ends and G-FOLD takes over. It shapes this
            // phase, so it belongs here rather than with the G-FOLD tuning.
            ImGui.InputDouble("Hand off to G-FOLD at T-gate (s)", ref _s.GfoldHandoffTgo);
        }

        double3 r = orbit.StateVectors.PositionCci;
        double3 siteDir = SiteDirCciAt(parent, 0);
        double distNowKm = AngleBetween(r, siteDir) * bodyRadius / 1000.0;
        ImGui.Text($"Ground distance to site now: {distNowKm,8:F1} km");
        ImGui.Text($"Site terrain height: {SiteTerrainHeight(parent),7:F0} m (gate referenced to it)");

        // --- Upcoming passes: how close the ground track comes to the site ---
        // Time-sliced: start a scan while idle at normal speed, advance it a fixed
        // sample budget per frame — never a whole-scan hitch in one frame.
        ImGui.SeparatorText("Upcoming passes");
        bool passesRefreshOk =
            (_s.LandingPhase == LandingPhase.Idle || _s.LandingPhase == LandingPhase.Done)
            && !Universe.IsAutoWarpActive
            && Universe.SimulationSpeed <= MaxScanSimSpeed;
        if (passesRefreshOk && _s.ScanIndex < 0
            && Environment.TickCount64 - _s.PassesRefreshedAtMs > PassRefreshIntervalMs)
            StartPassScan(orbit, mu);
        StepPassScan(parent, mu, bodyRadius);
        // Always exactly PassesToShow lines, so the layout (and the EXECUTE button
        // below) never jumps while a scan is in flight.
        for (int i = 0; i < PassesToShow; i++)
        {
            if (i < _s.Passes.Count && _s.Passes[i].minKm < 1e6)
                ImGui.Text($"Pass {i + 1}:  closest {_s.Passes[i].minKm,8:F1} km   in {_s.Passes[i].tSec,7:F0} s");
            else if (i < _s.Passes.Count)
                ImGui.Text($"Pass {i + 1}:  (no solution)");
            else
                ImGui.Text($"Pass {i + 1}:  scanning...");
        }

        // --- Commit ---
        ImGui.SeparatorText("Deorbit");
        ImGui.Checkbox("Engage autopilot", ref _s.Engage);
        ImGui.SameLine();
        ImGui.Checkbox("Auto engines/staging", ref _s.AutoStage);
        if (ImGui.Button(_retargetArmed ? "Click the surface...  (right-click cancels)" : "Retarget: click a spot"))
            _retargetArmed = !_retargetArmed;

        if (ImGui.Button("EXECUTE LANDING"))
            ExecuteLanding(vehicle, orbit, parent, mu, bodyRadius);
        ImGui.SameLine();
        if (ImGui.Button("Abort landing"))
            AbortLanding();
    }

    // G-FOLD sub-tab: terminal-descent tuning, overlays/debug, and the manual start.
    private static void DrawGfoldSubTab(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                        double mu, double bodyRadius)
    {
        if (ImGui.Button(_showGfoldParams ? "Close params" : "G-FOLD params..."))
            _showGfoldParams = !_showGfoldParams;

        ImGui.Checkbox("Show G-FOLD overlay (world)", ref _showGfoldOverlay);
        ImGui.SameLine();
        ImGui.Checkbox("G-FOLD debug", ref _showGfoldDebug);

        // Terminal-hover takeover: checking this ends the G-FOLD descent, engages
        // the hover controller, and focuses the Terminal sub-tab; unchecking gives
        // the vehicle back.
        bool termActive = _s.LandingPhase == LandingPhase.TerminalHover;
        if (ImGui.Checkbox("Terminal hover (take over)", ref termActive))
        {
            if (termActive)
                StartTerminalHover();
            else
                AbortLanding();
        }

        if (ImGui.Button(_retargetArmed ? "Click the surface...  (right-click cancels)" : "Retarget: click a spot"))
            _retargetArmed = !_retargetArmed;

        // Skip straight to G-FOLD from the current state (or restart it after a
        // failure), engaging the autopilot + auto engines so it actually flies.
        if (ImGui.Button("Start G-FOLD now", new float2(360f, 40f)))
        {
            _s.Engage = true;
            _s.AutoStage = true;
            _s.Running = false;
            _s.LandingPhase = LandingPhase.GfoldDescent;
            _s.GfoldHandoffTime = SimNow();
            _s.GfoldLastSolveTime = double.NegativeInfinity;
            _s.GfoldPlan = null;
            _s.GfoldFailStreak = 0;
            _s.GfoldTrackInit = false;
            _s.GfoldEngineOn = false;
            _s.HasCommand = false;
            _s.LandingStatus = "G-FOLD started from current state.";
        }
        if (ImGui.Button("Abort landing"))
            AbortLanding();
    }

    private static void AbortLanding()
    {
        _s.LandingPhase = LandingPhase.Done;
        _s.LandingCutPending = true;
        _s.LandingStatus = "Aborted.";
    }

    // Shared landing status, drawn below whichever sub-tab is open.
    private static void DrawLandingStatus(Vehicle vehicle)
    {
        if (_s.LandingStatus.Length > 0)
            ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f), _s.LandingStatus);
        if (_s.LandingPhase != LandingPhase.Idle)
        {
            double tIgn = _s.BurnStartTime - SimNow();
            string phaseText = _s.LandingPhase switch
            {
                LandingPhase.Coast => $"Coasting to burn point — ignition T-{tIgn,6:F0} s",
                LandingPhase.Prep => $"Converging guidance — ignition T-{tIgn,5:F1} s",
                LandingPhase.Burn => $"BURNING — cmd {_s.Upfg.Throttle * 100,4:F0} % / engine {vehicle.GetManualThrottle() * 100,4:F0} %, tgo {_s.Upfg.Tgo,6:F1} s",
                LandingPhase.GfoldDescent => $"G-FOLD [{_s.GfoldStatus}] alt {_s.GfoldAltM,6:F0} m, {_s.GfoldSpeedMs,5:F0} m/s, throttle {_s.GfoldThrottle * 100,3:F0} %, tf~{Math.Max(_s.GfoldArrivalTime - SimNow(), 0),4:F0} s",
                LandingPhase.TerminalHover => $"TERMINAL HOVER alt {_s.GfoldAltM,6:F1} m, {_s.GfoldSpeedMs,5:F1} m/s, throttle {_s.GfoldThrottle * 100,3:F0} %",
                LandingPhase.Done => "Landing guidance ended.",
                _ => "",
            };
            ImGui.TextColored(new float4(0.5f, 0.9f, 1f, 1f), phaseText);
            ImGui.Text($"Predicted burn downrange: {_s.BurnDownrangeKm,7:F1} km  (start at {_s.DownrangeFactor:F2}x)");
        }
    }

    // EXECUTE: measure the braking burn with a synchronous Mode-2 convergence, find
    // the moment our along-track distance to the site equals factor × that length,
    // and ask to warp there. The actual Mode-3 burn starts via StepLanding.
    private static void ExecuteLanding(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                       double mu, double bodyRadius)
    {
        _s.LandingStatus = "";
        UpfgVehicle model = BuildUpfgVehicle(vehicle);
        if (model == null)
        {
            _s.LandingStatus = "No usable engine model on the vehicle.";
            return;
        }
        if (_s.GLimitEnabled && _s.GLimitG > 0.1)
            ApplyGLimit(model, _s.GLimitG);

        double3 r = orbit.StateVectors.PositionCci;
        double3 v = orbit.StateVectors.VelocityCci;

        // First pass: predict the braking-burn downrange from the current state and
        // solve the ignition time from it.
        double downrange = PredictBurnDownrange(r, v, vehicle.TotalMass, mu, model, parent, bodyRadius);
        if (double.IsNaN(downrange) || downrange <= 0)
        {
            _s.LandingStatus = "Burn prediction failed to converge.";
            return;
        }
        double wait = FindBurnStartTime(orbit, parent, mu, bodyRadius, downrange * _s.DownrangeFactor);
        if (double.IsNaN(wait))
        {
            _s.LandingStatus = "No pass within 5 orbits gets inside the burn distance — adjust orbit.";
            return;
        }

        // Second pass: on an elliptical orbit the speed/altitude at the ignition
        // point differ from here, so re-predict at the propagated ignition state
        // and re-solve the window with the corrected distance.
        (double3 rIgn, double3 vIgn, _) = CseRoutine.Run(r, v, Math.Max(wait, 1e-3), mu, CseState.Zero);
        double refined = PredictBurnDownrange(rIgn, vIgn, vehicle.TotalMass, mu, model, parent, bodyRadius);
        if (!double.IsNaN(refined) && refined > 0)
        {
            double refinedWait = FindBurnStartTime(orbit, parent, mu, bodyRadius, refined * _s.DownrangeFactor);
            if (!double.IsNaN(refinedWait))
            {
                downrange = refined;
                wait = refinedWait;
            }
        }
        _s.BurnDownrangeKm = downrange / 1000.0;

        _s.BurnStartTime = SimNow() + wait;
        _s.Running = false;          // landing owns guidance and the autopilot now
        _s.AutoLaunch = false;
        _s.CutoffDone = false;
        _s.StagingActive = false;
        _s.HasCommand = false;
        _s.LandingPhase = LandingPhase.Coast;
        // A fresh EXECUTE re-arms the prompt even if an earlier one was declined.
        _warpDeclinedLabel = "";
        if (wait > PrepLeadTime + WarpLeadTime)
            RequestWarp(_s.BurnStartTime - PrepLeadTime - WarpLeadTime, "the deorbit burn point");
    }

    // How far downrange the braking burn ends if lit at the given state, iterated
    // synchronously to convergence (UPFG is pure math, so unlike the original —
    // which flew its Mode 2 live while already braking — we can converge before
    // ignition in one frame). This is Mode 1 with the same high-gate end state the
    // real burn will fly (aim altitude, sink rate as a straight-down velocity via
    // fpa = -90°), cutoff position free. NaN if it fails to converge.
    private static double PredictBurnDownrange(double3 r, double3 v, double mass, double mu,
                                               UpfgVehicle model, IParentBody parent, double bodyRadius)
    {
        double gateRadius = bodyRadius + SiteTerrainHeight(parent) + _s.AimAltKm * 1000.0;
        var predict = new UpfgTarget
        {
            Radius = gateRadius,
            Velocity = _s.DescentRate,
            Fpa = -Math.PI / 2.0,
            Normal = double3.Normalize(double3.Cross(r, v)),
            Rdes = SiteDirCciAt(parent, 0) * gateRadius,
        };
        // A SCRATCH SOLVER, not the vehicle's. This is a what-if run off the flight
        // path — 400 iterations against a state that may be an hour in the future —
        // and the vehicle's own UPFG may be mid-burn on something else. It used to
        // borrow the shared instance and bracket the loop with Reset() to put it
        // back, which is exactly the kind of state laundering that stops working the
        // moment two craft are flying at once.
        var scratch = new UpfgGuidance();
        bool converged = false;
        for (int i = 0; i < 400; i++)
        {
            scratch.Step(r, v, mass, mu, predict, model, 1);
            if (scratch.Converged)
            {
                converged = true;
                break;
            }
        }
        return converged ? AngleBetween(r, scratch.Rd) * bodyRadius : double.NaN;
    }

    // The phases that are flying the vehicle down under power, and so are the ones
    // a ground contact should terminate.
    private static bool IsPoweredDescentPhase(LandingPhase phase) =>
        phase == LandingPhase.Burn
        || phase == LandingPhase.GfoldDescent
        || phase == LandingPhase.TerminalHover;

    // KSA's own contact switch. The physics step raises a terrain-contact flag on
    // the vehicle whenever ANY part of it makes a Bepu contact with the terrain or
    // launch-pad collider (ConstraintSim.DetectTerrainContact), and ocean entry
    // sets the matching ocean flag — so this fires on the legs, or on whatever
    // else reaches the ground first, without us guessing at leg geometry.
    //
    // This replaces trusting the altitude estimate to notice touchdown. That
    // estimate is terrain-height sampling minus an assumed vehicle height, and
    // when it reads low the hover controller keeps flying a vehicle that is
    // already on the ground.
    private static bool HasTouchedDown(Vehicle vehicle) => vehicle.Situation.HasAnyContact();


    // Per-frame landing state machine, run for this vehicle from ApplyAutopilot (the
    // PrepareWorker prefix) whether or not it is the one on screen.
    private static void StepLanding(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                    double mu, double bodyRadius)
    {
        if (_s.LandingPhase == LandingPhase.Idle || _s.LandingPhase == LandingPhase.Done)
            return;

        double now = SimNow();

        // Touchdown, from the physics rather than from geometry. Checked before
        // any powered phase steps, so the engine is cut on the frame contact is
        // reported instead of the controller fighting the ground.
        //
        // Armed only once the vehicle has actually been off the ground: a vehicle
        // sitting on the pad already reports terrain contact (the launch-pad
        // collider counts), so without this, taking over with terminal hover from
        // the ground would cut the engine on its first step. Re-armed on every
        // phase change, which is free in the air — the same frame clears it.
        bool contact = HasTouchedDown(vehicle);
        if (_s.LandingPhase != _s.TouchdownPrevPhase)
        {
            _s.TouchdownPrevPhase = _s.LandingPhase;
            _s.LandingTouchdownArmed = false;
        }
        if (!contact)
            _s.LandingTouchdownArmed = true;

        if (contact && _s.LandingTouchdownArmed && IsPoweredDescentPhase(_s.LandingPhase))
        {
            _s.GfoldThrottle = 0.0;
            _s.HasCommand = false;
            _s.LandingPhase = LandingPhase.Done;
            _s.LandingCutPending = true;
            _s.LandingStatus = $"TOUCHDOWN — contact detected, engine cut ({_s.GfoldSpeedMs:F1} m/s).";
            return;
        }

        if (_s.LandingPhase == LandingPhase.GfoldDescent)
        {
            StepGfoldDescent(vehicle, orbit, parent, bodyRadius, now);
            return;
        }

        if (_s.LandingPhase == LandingPhase.TerminalHover)
        {
            StepTerminalHover(vehicle, orbit, parent, mu, bodyRadius, now);
            return;
        }

        if (_s.LandingPhase == LandingPhase.Coast)
        {
            if (now >= _s.BurnStartTime - PrepLeadTime)
            {
                if (Universe.IsAutoWarpActive)
                    Universe.AutoWarpStop(true);
                _s.Upfg.Reset();
                _s.LandingPhase = LandingPhase.Prep;
            }
            return;
        }

        // Prep / Burn: run Mode-3 guidance on the live vehicle. The landing target
        // is re-derived every step: the site rotates with the body, and the plane
        // is whatever we are actually flying in.
        try
        {
            UpfgVehicle live = BuildUpfgVehicle(vehicle);
            if (live != null)
            {
                if (_s.GLimitEnabled && _s.GLimitG > 0.1)
                    ApplyGLimit(live, _s.GLimitG);
                _s.UpfgVehicle = live;

                double3 r = orbit.StateVectors.PositionCci;
                double3 v = orbit.StateVectors.VelocityCci;
                double3 planeNormal = double3.Normalize(double3.Cross(r, v));
                // The gate: above the site (terrain-referenced, not the mean
                // sphere) and uprange of it along the approach (rotating a
                // position about +h moves it downrange, so uprange is the
                // negative rotation).
                double gateRadius = bodyRadius + SiteTerrainHeight(parent) + _s.AimAltKm * 1000.0;
                double3 gateDir = RotateAbout(SiteDirCciAt(parent, 0), planeNormal,
                    -_s.GateUprangeKm * 1000.0 / bodyRadius);
                var target = new UpfgTarget
                {
                    Radius = gateRadius,
                    Velocity = 0,                 // no forward speed at the gate
                    DescentRate = _s.DescentRate,   // arrive sinking, not stopped
                    Fpa = 0,
                    Normal = planeNormal,
                    Rdes = gateDir * gateRadius,
                };
                _s.Upfg.Step(r, v, vehicle.TotalMass, mu, target, live, 3);
                _s.CommandDir = _s.Upfg.Steering;
                _s.HasCommand = _s.CommandDir.Length() > 0.5;
            }
            _s.FailStreak = 0;
            _s.GuidanceError = "";
        }
        catch (Exception e)
        {
            _s.FailStreak++;
            _s.GuidanceError = e.Message;
            if (_s.FailStreak > MaxFailStreak)
            {
                _s.LandingPhase = LandingPhase.Done;
                _s.LandingCutPending = true;
                _s.LandingStatus = "Guidance failed repeatedly — landing stopped.";
            }
        }

        if (_s.LandingPhase == LandingPhase.Prep && now >= _s.BurnStartTime)
            _s.LandingPhase = LandingPhase.Burn;

        if (_s.LandingPhase == LandingPhase.Burn)
        {
            // Staging support during the descent burn (ignites the deorbit engine
            // too if its sequence was never fired).
            if (_s.Engage && _s.AutoStage)
                AutoSequence(vehicle);

            // Hand straight to G-FOLD a set time before gate arrival, skipping the
            // UPFG terminal freeze. G-FOLD plans from the current state down.
            if (_s.Upfg.Converged && _s.Upfg.Tgo <= _s.GfoldHandoffTgo)
            {
                _s.LandingPhase = LandingPhase.GfoldDescent;
                _s.GfoldHandoffTime = now;
                _s.GfoldLastSolveTime = double.NegativeInfinity;
                _s.GfoldPlan = null;
                _s.GfoldFailStreak = 0;
                _s.GfoldTrackInit = false;
                _s.GfoldEngineOn = false;
                _s.GfoldTabSelectPending = true;   // focus the G-FOLD sub-tab for the descent
                _s.LandingStatus = "Handoff to G-FOLD descent.";
            }
        }
    }

    // ----- Landing site geometry -----

    // The site's body-fixed (CCF) direction. KSA's own convention: lat = asin(z),
    // lon = atan2(y,x) in CCF.
    private static double3 SiteDirCcf()
    {
        double lat = UpfgTarget.DegToRad(_s.SiteLatDeg);
        double lon = UpfgTarget.DegToRad(_s.SiteLonDeg);
        return new double3(
            Math.Cos(lat) * Math.Cos(lon),
            Math.Cos(lat) * Math.Sin(lon),
            Math.Sin(lat));
    }

    // The site's CCI direction dtFuture seconds from now. A body spins about its
    // own CCI Z axis (per IParentBody.GetAngularVelocityCci), so the future
    // position is the current one carried around Z.
    private static double3 SiteDirCciAt(IParentBody parent, double dtFuture)
    {
        double3 dirNow = SiteDirCcf().Transform(parent.GetCcf2Cci());
        return RotZ(dirNow, parent.GetAngularVelocity() * dtFuture);
    }

    // Terrain height of the site above the body's mean-radius sphere, sampled from
    // KSA's own heightmap (the game places surface objects at MeanRadius + this).
    // The site is fixed in CCF, so the value is cached and only re-sampled when
    // the inputs (or the body) change.

    private static double SiteTerrainHeight(IParentBody parent)
    {
        if (_s.SiteLatDeg != _s.SiteTerrainCacheLat || _s.SiteLonDeg != _s.SiteTerrainCacheLon
            || !ReferenceEquals(parent, _s.SiteTerrainCacheBody))
        {
            _s.SiteTerrainHeightM = (parent as Celestial)?.GetTerrainHeightFromDirCcf(SiteDirCcf()) ?? 0.0;
            if (!double.IsFinite(_s.SiteTerrainHeightM))
                _s.SiteTerrainHeightM = 0.0;
            _s.SiteTerrainCacheLat = _s.SiteLatDeg;
            _s.SiteTerrainCacheLon = _s.SiteLonDeg;
            _s.SiteTerrainCacheBody = parent;
        }
        return _s.SiteTerrainHeightM;
    }

    // ----- Upcoming-passes scan -----

    // Begin an incremental closest-approach scan over the next PassesToShow orbits.
    private static void StartPassScan(Orbit orbit, double mu)
    {
        double sma = (orbit.Periapsis + orbit.Apoapsis) / 2.0;
        if (sma <= 0 || double.IsNaN(sma))
            return;
        double period = 2.0 * Math.PI * Math.Sqrt(sma * sma * sma / mu);
        _s.ScanR0 = orbit.StateVectors.PositionCci;
        _s.ScanV0 = orbit.StateVectors.VelocityCci;
        _s.ScanStep = period / ScanSamplesPerOrbit;
        _s.ScanCser = CseState.Zero;
        _s.ScanResults.Clear();
        _s.ScanIndex = 0;
    }

    // Advance the scan by at most ScanSamplesPerFrame samples. Sequential sample
    // times keep the CSE warm-started; each completed orbit's minimum is sharpened
    // with a parabolic fit through its neighbours before being committed.
    private static void StepPassScan(IParentBody parent, double mu, double bodyRadius)
    {
        // Pause (not abort) while warping — auto OR manual: frames are already
        // expensive there, and the scan is only a display aid.
        if (_s.ScanIndex < 0 || Universe.IsAutoWarpActive
            || Universe.SimulationSpeed > MaxScanSimSpeed)
            return;

        double period = _s.ScanStep * ScanSamplesPerOrbit;
        int total = ScanSamplesPerOrbit * PassesToShow;
        int end = Math.Min(_s.ScanIndex + ScanSamplesPerFrame, total);
        for (; _s.ScanIndex < end; _s.ScanIndex++)
        {
            double t = (_s.ScanIndex + 1) * _s.ScanStep;
            // Conic position is periodic, so propagate by t mod period — the CSE
            // port dropped the original's multi-revolution counter (ascent never
            // needs it), and multi-rev inputs are where it slowed down and went
            // NaN. Only the site rotation needs the full t.
            double tProp = t % period;
            if (_s.ScanIndex % ScanSamplesPerOrbit == 0)
                _s.ScanCser = CseState.Zero; // warm-start doesn't survive the wrap

            double d;
            if (tProp < 1e-3)
            {
                d = AngleBetween(_s.ScanR0, SiteDirCciAt(parent, t)) * bodyRadius;
            }
            else
            {
                double3 rr;
                (rr, _, _s.ScanCser) = CseRoutine.Run(_s.ScanR0, _s.ScanV0, tProp, mu, _s.ScanCser);
                d = AngleBetween(rr, SiteDirCciAt(parent, t)) * bodyRadius;
            }
            if (!double.IsFinite(d))
            {
                d = 1e12; // poisoned sample: ignore it and restart the warm chain
                _s.ScanCser = CseState.Zero;
            }
            _s.ScanOrbitD[_s.ScanIndex % ScanSamplesPerOrbit] = d;

            if ((_s.ScanIndex + 1) % ScanSamplesPerOrbit == 0)
                CommitScanOrbit(_s.ScanIndex + 1 - ScanSamplesPerOrbit);
        }

        if (_s.ScanIndex >= total)
        {
            _s.Passes.Clear();
            _s.Passes.AddRange(_s.ScanResults);
            _s.ScanIndex = -1;
            _s.PassesRefreshedAtMs = Environment.TickCount64;
        }
    }

    private static void CommitScanOrbit(int orbitStartIndex)
    {
        int jMin = 0;
        for (int j = 1; j < ScanSamplesPerOrbit; j++)
            if (_s.ScanOrbitD[j] < _s.ScanOrbitD[jMin])
                jMin = j;

        double tBest = (orbitStartIndex + jMin + 1) * _s.ScanStep;
        double dBest = _s.ScanOrbitD[jMin];
        if (jMin > 0 && jMin < ScanSamplesPerOrbit - 1)
        {
            double d0 = _s.ScanOrbitD[jMin - 1], d1 = _s.ScanOrbitD[jMin], d2 = _s.ScanOrbitD[jMin + 1];
            double denom = d0 - 2.0 * d1 + d2;
            if (Math.Abs(denom) > 1e-9)
            {
                double frac = Math.Clamp(0.5 * (d0 - d2) / denom, -1.0, 1.0);
                tBest += frac * _s.ScanStep;
                dBest = d1 - 0.25 * (d0 - d2) * frac;
            }
        }
        _s.ScanResults.Add((tBest, dBest / 1000.0));
    }

    private static double GroundDistanceAt(double3 r0, double3 v0, double t,
                                           IParentBody parent, double mu, double bodyRadius)
    {
        // Keep the conic solver single-revolution (see StepPassScan): position is
        // periodic, only the site rotation needs the full t.
        double tProp = t;
        double sma = 1.0 / (2.0 / r0.Length() - double3.Dot(v0, v0) / mu);
        if (sma > 0)
        {
            double period = 2.0 * Math.PI * Math.Sqrt(sma * sma * sma / mu);
            tProp = t % period;
        }

        double3 rr = r0;
        if (tProp > 1e-3)
            (rr, _, _) = CseRoutine.Run(r0, v0, tProp, mu, CseState.Zero);
        double d = AngleBetween(rr, SiteDirCciAt(parent, t)) * bodyRadius;
        return double.IsFinite(d) ? d : 1e12;
    }

    // First future moment the along-track distance to the site shrinks through the
    // given threshold (approaching), within the next 5 orbits; NaN if never.
    private static double FindBurnStartTime(Orbit orbit, IParentBody parent, double mu,
                                            double bodyRadius, double thresholdMeters)
    {
        double sma = (orbit.Periapsis + orbit.Apoapsis) / 2.0;
        if (sma <= 0 || double.IsNaN(sma))
            return double.NaN;
        double period = 2.0 * Math.PI * Math.Sqrt(sma * sma * sma / mu);

        double3 r0 = orbit.StateVectors.PositionCci;
        double3 v0 = orbit.StateVectors.VelocityCci;

        double step = period / 720.0;
        double prev = double.NaN;
        CseState cs = CseState.Zero; // warm-started across the sequential samples
        for (double t = 0; t <= 5.0 * period; t += step)
        {
            // Single-revolution propagation (see StepPassScan); reset the warm
            // start at each period wrap.
            double tProp = t % period;
            if (t > 0 && tProp < step)
                cs = CseState.Zero;

            double d;
            if (tProp < 1e-3)
            {
                d = AngleBetween(r0, SiteDirCciAt(parent, t)) * bodyRadius;
            }
            else
            {
                double3 rr;
                (rr, _, cs) = CseRoutine.Run(r0, v0, tProp, mu, cs);
                d = AngleBetween(rr, SiteDirCciAt(parent, t)) * bodyRadius;
            }
            if (!double.IsFinite(d))
            {
                d = 1e12;
                cs = CseState.Zero;
            }
            if (!double.IsNaN(prev) && prev > thresholdMeters && d <= thresholdMeters)
            {
                double lo = t - step, hi = t;
                for (int i = 0; i < 30; i++)
                {
                    double mid = 0.5 * (lo + hi);
                    if (GroundDistanceAt(r0, v0, mid, parent, mu, bodyRadius) > thresholdMeters)
                        lo = mid;
                    else
                        hi = mid;
                }
                return hi;
            }
            prev = d;
        }
        return double.NaN;
    }
}

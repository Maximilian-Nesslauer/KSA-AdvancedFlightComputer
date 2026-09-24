#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Guidance.Upfg;

public static partial class GuidanceWindow
{
    // Public because VehicleAutopilotState holds a vehicle's phase: every craft runs this machine on its own, so the phase is a field on the flight computer rather than one global "the landing".
    // Every landing starts in DeorbitPlanning. A transfer continues through the stock node phases to TransferCoast, a direct approach continues at Coast, and both brake from Prep.
    public enum LandingPhase { Idle,
        DeorbitPlanning, DeorbitCoast, DeorbitNodePending, DeorbitBurn, TransferPlanning, TransferCoast,
        Coast, Prep, Burn, GfoldDescent, TerminalHover, TerminalCoast, TerminalBrake, Done }

    // The site, the approach shaping (downrange factor, gate altitude/uprange, sink rate) and the whole pass scan live on the vehicle - see VehicleAutopilotState.
    private const double PrepLeadTime = 30.0;      // converge + point before ignition

    // Upcoming site passes are computed in closed form - see Guidance/SitePasses.cs.
    private const int PassesToShow = 5;

    // The Landing tab body: a Deorbit sub-tab (UPFG braking to the gate) and a G-FOLD sub-tab (terminal descent).
    // Each carries its own relevant parameters; shared landing status is below the sub-tab bar.
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
            // One-shot, and this window draws FIRST - so it only consumes the flag when the gauge panel is not up to act on it.
            // Consuming unconditionally is why the new panel never followed the handoff to G-FOLD.
            if (!PanelVisible)
                _s.GfoldTabSelectPending = false;
            if (ImGui.BeginTabItem("G-FOLD", gfoldFlags))
            {
                DrawGfoldSubTab(vehicle, orbit, parent, mu, bodyRadius);
                ImGui.EndTabItem();
            }

            var termFlags = _s.TermTabSelectPending
                ? ImGuiTabItemFlags.SetSelected
                : ImGuiTabItemFlags.None;
            // One-shot, and this window draws FIRST - leave it for the gauge panel when that is up, exactly as with the G-FOLD focus flag above.
            if (!PanelVisible)
                _s.TermTabSelectPending = false;
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

    // Deorbit sub-tab: the landing site, upcoming passes, approach tuning, and the deorbit-burn commit.
    private static void DrawDeorbitSubTab(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                          double mu, double bodyRadius)
    {
        if (ImGui.CollapsingHeader("Landing site"))
        {
            ImGui.BeginDisabled(DeorbitTargetLocked);
            ImGui.InputDouble("Latitude (deg)", ref _s.SiteLatDeg);
            ImGui.InputDouble("Longitude (deg)", ref _s.SiteLonDeg);
            ImGui.EndDisabled();
        }

        if (ImGui.CollapsingHeader("Approach parameters", ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.BeginDisabled(DeorbitTargetLocked);
            ImGui.InputDouble("Braking altitude above site (km)", ref _s.BrakingAltitudeKm);
            ImGui.Checkbox("Set deorbit arrival angle", ref _s.DeorbitArrivalAngleEnabled);
            ImGui.BeginDisabled(!_s.DeorbitArrivalAngleEnabled);
            ImGui.InputDouble("Arrival angle down (deg)", ref _s.DeorbitArrivalDescentDeg);
            ImGui.EndDisabled();
            ImGui.TextWrapped($"Positive angles point down. The arrival tolerance is +/-{DeorbitPlanner.ArrivalAngleToleranceDeg:F0} deg at the braking point.");
            ImGui.InputDouble("Downrange factor", ref _s.DownrangeFactor);
            ImGui.InputDouble("Aim altitude (km)", ref _s.AimAltKm);
            ImGui.InputDouble("Descent rate (m/s)", ref _s.DescentRate);
            ImGui.InputDouble("Gate uprange (km)", ref _s.GateUprangeKm);
            // Where the braking burn ends and G-FOLD takes over.
            // It shapes this phase, so it belongs here rather than with the G-FOLD tuning.
            ImGui.InputDouble("Hand off to the descent at T-gate (s)", ref _s.GfoldHandoffTgo);
            ImGui.InputDouble("Vertical approach height (m)", ref _s.LandingVerticalGateM);
            ImGui.TextWrapped($"The braking target is at least {LandingBrakeGateAltitude:F0} m above the site. G-FOLD removes horizontal speed before the final descent.");
            ImGui.EndDisabled();
        }

        double3 r = orbit.StateVectors.PositionCci;
        double3 siteDir = SiteDirCciAt(parent, 0);
        double distNowKm = AngleBetween(r, siteDir) * bodyRadius / 1000.0;
        ImGui.Text($"Ground distance to site now: {distNowKm,8:F1} km");
        ImGui.Text($"Site terrain height: {SiteTerrainHeight(parent),7:F0} m (gate referenced to it)");
        foreach (var row in DeorbitReadout()) ImGui.TextWrapped($"{row.label}: {row.value}");
        DrawStockDeorbitActions();
        if (DeorbitTargetLocked) ImGui.TextWrapped("Abort and plan again to change the transfer target.");

        // --- Upcoming passes: how close the ground track comes to the site --- Time-sliced: start a scan while idle at normal speed, advance it a fixed sample budget per frame - never a whole-scan hitch in one frame.
        ImGui.SeparatorText("Upcoming passes");
        RefreshPasses(orbit, parent, mu, bodyRadius);
        for (int i = 0; i < _s.Passes.Count; i++)
            ImGui.Text($"Pass {i + 1}:  closest {_s.Passes[i].minKm,8:F1} km   in {_s.Passes[i].tSec,7:F0} s");

        // --- Commit ---
        ImGui.SeparatorText("Deorbit");
        ImGui.Checkbox("Engage autopilot", ref _s.Engage);
        ImGui.SameLine();
        ImGui.Checkbox("Auto engines/staging", ref _s.AutoStage);
        if (ImGui.Button(_retargetArmed ? "Click the surface...  (right-click cancels)" : "Retarget: click a spot"))
            _retargetArmed = !_retargetArmed;

        if (ImGui.Button("EXECUTE LANDING"))
            ExecuteLanding(vehicle);
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

        // Terminal-hover takeover: checking this ends the G-FOLD descent, engages the hover controller, and focuses the Terminal sub-tab; unchecking gives the vehicle back.
        bool termActive = _s.LandingPhase == LandingPhase.TerminalHover;
        if (ImGui.Checkbox("Terminal hover (take over)", ref termActive))
        {
            if (termActive)
                StartTerminalHover(vehicle);
            else
                AbortLanding();
        }

        if (ImGui.Button(_retargetArmed ? "Click the surface...  (right-click cancels)" : "Retarget: click a spot"))
            _retargetArmed = !_retargetArmed;

        // Skip straight to G-FOLD from the current state (or restart it after a failure), engaging the autopilot + auto engines so it actually flies.
        if (ImGui.Button("Start G-FOLD now", new float2(360f, 40f)))
            StartGfoldNow(vehicle);
        if (ImGui.Button("Abort landing"))
            AbortLanding();
    }

    /// <summary>
    /// Skip straight to G-FOLD from the current state, or restart it after a failure.
    /// Engages the autopilot and auto engines, because a powered descent that is not
    /// allowed to steer or throttle is not a descent.
    /// </summary>
    private static void StartGfoldNow(Vehicle vehicle)
    {
        ResetLandingEngineWait();
        ClaimVehicle(GuidanceMode.Landing, vehicle);   // the descent takes the vehicle over
        _s.Engage = true;
        _s.AutoStage = true;
        _s.LandingPhase = LandingPhase.GfoldDescent;
        ResetGfoldTrace();   // fresh flown path and a fresh axis latch
        _s.GfoldHandoffTime = SimNow();
        _s.GfoldLastSolveTime = double.NegativeInfinity;
        _s.GfoldPlan = null;
        _s.GfoldFailStreak = 0;
        _s.GfoldTrackInit = false;
        _s.GfoldEngineOn = false;
        _s.GfoldRelightRequestTime = double.NaN;
        _s.GfoldHoverRefused = false;
        _s.HasCommand = false;
        _s.LandingStatus = "G-FOLD started from current state.";
    }

    // An abort in the air hands the craft back with the engine as it is, because a cut there drops the craft. The coast and the deorbit burn keep the cut, which stops the burn and costs a coasting craft nothing.
    private static void AbortLanding()
    {
        ResetLandingEngineWait();
        _s.DeorbitPlanner?.Dispose();
        _s.DeorbitPlanner = null;
        if (!_s.ControlAcquired && !_s.DeorbitNodeClaimed) ClearDeorbitPlanState();
        bool airborne = _s.LandingPhase == LandingPhase.GfoldDescent
            || _s.LandingPhase == LandingPhase.TerminalHover || TerminalBurnActive;
        _s.LandingPhase = LandingPhase.Done;
        if (airborne)
        {
            _s.ReleaseWithoutEngineCut = true;
            _s.LandingStatus = "Aborted, the engine is left as it was.";
        }
        else
        {
            _s.LandingCutPending = true;
            _s.LandingStatus = "Aborted.";
        }
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
                LandingPhase.DeorbitPlanning => "Planning the deorbit transfer",
                LandingPhase.DeorbitCoast => "Waiting to create the stock deorbit node",
                LandingPhase.DeorbitNodePending => "Checking stock ignition timing",
                LandingPhase.DeorbitBurn => $"STOCK AUTO - remaining correction {_s.DeorbitLastResidual:F2} m/s",
                LandingPhase.TransferPlanning => "Calculating braking from the actual orbit",
                LandingPhase.TransferCoast => $"Transfer coast - braking ignition T-{tIgn:F1} s",
                LandingPhase.Coast => $"Coasting to burn point - ignition T-{tIgn,6:F0} s",
                LandingPhase.Prep => $"Converging guidance - ignition T-{tIgn,5:F1} s",
                LandingPhase.Burn => $"BURNING - cmd {_s.Upfg.Throttle * 100,4:F0} % / engine {vehicle.GetManualThrottle() * 100,4:F0} %, tgo {_s.Upfg.Tgo,6:F1} s",
                LandingPhase.GfoldDescent => $"G-FOLD [{_s.GfoldStatus}] alt {_s.GfoldAltM,6:F0} m, {_s.GfoldSpeedMs,5:F0} m/s, throttle {_s.GfoldThrottle * 100,3:F0} %, tf~{Math.Max(_s.GfoldArrivalTime - SimNow(), 0),4:F0} s",
                LandingPhase.TerminalHover => $"TERMINAL HOVER alt {_s.GfoldAltM,6:F1} m, {_s.GfoldSpeedMs,5:F1} m/s, throttle {_s.GfoldThrottle * 100,3:F0} %",
                LandingPhase.TerminalCoast => $"TERMINAL COAST alt {_s.GfoldAltM:F1} m, ignition height {_s.TerminalIgnitionHeight:F1} m",
                LandingPhase.TerminalBrake => $"LANDING BURN alt {_s.GfoldAltM:F1} m, {_s.GfoldSpeedMs:F1} m/s, throttle {_s.GfoldThrottle * 100:F0} %",
                LandingPhase.Done => "Landing guidance ended.",
                _ => "",
            };
            ImGui.TextColored(new float4(0.5f, 0.9f, 1f, 1f), phaseText);
            ImGui.Text($"Predicted burn downrange: {_s.BurnDownrangeKm,7:F1} km  (start at {_s.DownrangeFactor:F2}x)");
        }
    }

    // These phases wait without control, and a release keeps them queued.
    private static bool LandingWaits(LandingPhase phase) =>
        phase is LandingPhase.Coast or LandingPhase.DeorbitPlanning or LandingPhase.DeorbitCoast;

    // Stock Auto flies the deorbit node in these phases while guidance holds only the claim.
    private static bool IsStockDeorbitPhase(LandingPhase phase) =>
        phase is LandingPhase.DeorbitCoast or LandingPhase.DeorbitNodePending or LandingPhase.DeorbitBurn;

    private static bool TerminalBurnActive => _s.LandingPhase is LandingPhase.TerminalCoast or LandingPhase.TerminalBrake;

    private static bool LandingSwitchesOff => !_s.Engage || !_s.AutoStage;

    private const string LandingSwitchesChangedStatus = "Automatic landing stopped because its control switches changed.";

    // These phases use AFC's manual engine and attitude control.
    // The direct coast waits until Prep, and the stock node takes a separate claim without forcing Manual.
    private static bool LandingCommands(LandingPhase phase) =>
        phase is LandingPhase.TransferPlanning or LandingPhase.TransferCoast or LandingPhase.Prep
            or LandingPhase.Burn or LandingPhase.GfoldDescent or LandingPhase.TerminalHover
            or LandingPhase.TerminalCoast or LandingPhase.TerminalBrake;

    // Runs from ApplyAutopilot ahead of the claim, so the step that turns the coast into Prep is the step that claims the craft.
    private static void StepLandingCoast()
    {
        if (_s.LandingPhase == LandingPhase.Coast && SimNow() >= _s.BurnStartTime - PrepLeadTime)
            EnterBrakingPrep();
    }

    private static void EnterBrakingPrep()
    {
        if (Universe.IsAutoWarpActive)
            Universe.AutoWarpStop(true);
        _s.Upfg.Reset();
        _s.LandingPhase = LandingPhase.Prep;
        _s.LandingStatus = "Converging the braking guidance.";
    }

    // The phases that a ground contact ends. Touchdown arming carries across a change between two of them.
    private static bool IsPoweredDescentPhase(LandingPhase phase) =>
        phase is LandingPhase.DeorbitBurn or LandingPhase.TransferCoast or LandingPhase.Burn
            or LandingPhase.GfoldDescent or LandingPhase.TerminalHover or LandingPhase.TerminalCoast or LandingPhase.TerminalBrake;

    // KSA's own contact switch.
    // The physics step raises a terrain-contact flag on the vehicle whenever ANY part of it makes a Bepu contact with the terrain or launch-pad collider (ConstraintSim.DetectTerrainContact), and ocean entry sets the matching ocean flag - so this fires on the legs, or on whatever else reaches the ground first, without us guessing at leg geometry.
    //
    // This replaces trusting the altitude estimate to notice touchdown.
    // That estimate is terrain-height sampling minus an assumed vehicle height, and when it reads low the hover controller keeps flying a vehicle that is already on the ground.
    private static bool HasTouchedDown(Vehicle vehicle) => vehicle.Situation.HasAnyContact();


    // Per-frame landing state machine, run for this vehicle from ApplyAutopilot (the PrepareWorker prefix) whether or not it is the one on screen.
    private static void StepLanding(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                    double mu, double bodyRadius)
    {
        if (_s.LandingPhase == LandingPhase.Idle || _s.LandingPhase == LandingPhase.Done)
            return;

        double now = SimNow();

        // Touchdown, from the physics rather than from geometry.
        // Checked before any powered phase steps, so the engine is cut on the frame contact is reported instead of the controller fighting the ground.
        //
        // Armed only once the vehicle has actually been off the ground: a vehicle sitting on the pad already reports terrain contact (the launch-pad collider counts), so without this, taking over with terminal hover from the ground would cut the engine on its first step.
        // Preserve arming across descent transitions because contact can arrive on the transition step.
        bool contact = HasTouchedDown(vehicle);
        if (_s.LandingPhase != _s.TouchdownPrevPhase)
        {
            bool continuesDescent = IsPoweredDescentPhase(_s.TouchdownPrevPhase) && IsPoweredDescentPhase(_s.LandingPhase);
            _s.TouchdownPrevPhase = _s.LandingPhase;
            if (!continuesDescent) _s.LandingTouchdownArmed = false;
        }
        if (!contact)
            _s.LandingTouchdownArmed = true;

        if (contact && _s.LandingTouchdownArmed && IsPoweredDescentPhase(_s.LandingPhase))
        {
            ResetLandingEngineWait();
            _s.GfoldThrottle = 0.0;
            _s.HasCommand = false;
            _s.LandingPhase = LandingPhase.Done;
            _s.LandingCutPending = true;
            _s.LandingStatus = $"TOUCHDOWN - contact detected, engine cut ({_s.GfoldSpeedMs:F1} m/s).";
            return;
        }

        if (StepDeorbit(vehicle, orbit, parent, now)) return;

        if ((_s.DeorbitRequest != null || TerminalBurnActive) && LandingSwitchesOff)
        {
            AbortLanding();
            _s.LandingStatus = LandingSwitchesChangedStatus;
            return;
        }

        if (TerminalBurnActive)
        {
            StepTerminalBurn(vehicle, orbit, parent, now);
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

        // The coast turned into Prep in StepLandingCoast, ahead of the claim, so a coast that reaches this step is still waiting.
        if (_s.LandingPhase == LandingPhase.Coast)
            return;

        string engineRefusal = BrakingEngineRefusal(vehicle);
        if (engineRefusal.Length > 0) { RefuseDeorbit(engineRefusal); return; }
        if (!PrepareLandingEngines(vehicle, parent, now, requireAirless: false)) return;

        // Prep / Burn: run Mode-3 guidance on the live vehicle.
        // The landing target is re-derived every step: the site rotates with the body, and the plane is whatever we are actually flying in.
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
                // The gate: above the site (terrain-referenced, not the mean sphere) and uprange of it along the approach (rotating a position about +h moves it downrange, so uprange is the negative rotation).
                double gateRadius = bodyRadius + SiteTerrainHeight(parent) + LandingBrakeGateAltitude;
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
#if DEBUG
                using (new AdvancedFlightComputer.Core.PerfTracker.Scope("Guidance.DeorbitSolve"))
#endif
                {
                    _s.Upfg.Step(r, v, vehicle.TotalMass, mu, target, live, 3);
                }
                _s.CommandDir = _s.Upfg.Steering;
                _s.HasCommand = _s.CommandDir.Length() > 0.5;

                // The same sample the ascent logs, so a burn that dives or overshoots can be read after the flight.
                if (GuidanceLog.Enabled && now - _s.LastGuidanceLogTime >= GuidanceLogIntervalS)
                {
                    _s.LastGuidanceLogTime = now;
                    double3 up = double3.Normalize(r);
                    double siteDistanceKm = AngleBetween(r, SiteDirCciAt(parent, 0)) * bodyRadius / 1000.0;
                    GuidanceLog.Debug(vehicle, $"UPFG {_s.LandingPhase}: tgo {_s.Upfg.Tgo:F1} s, vgo {_s.Upfg.VgoMag:F0} m/s, converged {_s.Upfg.Converged}"
                        + $", throttle {_s.Upfg.Throttle:F2}, steer pitch {PitchOf(up, _s.Upfg.Steering):F1} deg, command pitch {PitchOf(up, _s.CommandDir):F1} deg"
                        + $", alt {(r.Length() - bodyRadius - SiteTerrainHeight(parent)) / 1000.0:F1} km over the site, {siteDistanceKm:F1} km to it"
                        + $", speed {v.Length():F0} m/s, sink {-double3.Dot(v, up):F0} m/s, mass {vehicle.TotalMass / 1000.0:F1} t, model {live.Stages.Count} stage(s).");
                }
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
                if (_s.LandingPhase != LandingPhase.Done)
                    GuidanceLog.Info(vehicle, $"landing stopped after {_s.FailStreak} failed steps: {e}");
                _s.LandingPhase = LandingPhase.Done;
                _s.LandingCutPending = true;
                _s.LandingStatus = "Guidance failed repeatedly - landing stopped.";
            }
        }

        if (_s.LandingPhase == LandingPhase.Prep && now >= _s.BurnStartTime)
        {
            if (!CheckBrakingIgnition(vehicle, orbit, now)) return;
            _s.LandingPhase = LandingPhase.Burn;
        }

        if (_s.LandingPhase == LandingPhase.Burn)
        {
            // Hand straight to G-FOLD a set time before gate arrival, skipping the UPFG terminal freeze.
            // G-FOLD plans from the current state down.
            if (_s.Upfg.Converged && _s.Upfg.Tgo <= _s.GfoldHandoffTgo)
            {
                // 6-DOF is the default, but it needs thrust vectoring and a throttle floor it can land on, and G-FOLD needs neither. A craft 6-DOF would refuse goes to G-FOLD, and the craft's solver choice follows it, so the panel shows and aborts the descent that is actually flying.
                string sixDofRefusal = null;
                if (_s.UseSixDofLanding && !Can6DofTake(vehicle, out sixDofRefusal))
                    _s.UseSixDofLanding = false;

                if (_s.UseSixDofLanding)
                {
                    // 6-DOF is EXCLUSIVE - it drives attitude through the TVC allocator rather than the flight computer - so the UPFG landing flow has to let go rather than run alongside it.
                    // Engage6Dof is that let-go (ClaimVehicle) and the request in one; the request is consumed by the next guidance step, which runs the cold solve off the draw.
                    //
                    // Reached from the sim step rather than a button, so the release at the end of this same step sees the queued request and keeps the craft for it (see ApplyAutopilot).
                    // The claim releases the landing machine that got us here and finds no 6-DOF engaged, so it touches neither the engine nor the gimbals on the way through, and the engine holds the burn's command until the engage runs.
                    Engage6Dof(vehicle);
                    _s.LandingStatus = "Handoff to 6-DOF descent.";
                }
                else
                {
                    _s.LandingPhase = LandingPhase.GfoldDescent;
                    ResetGfoldTrace();   // fresh flown path and a fresh axis latch
                    _s.GfoldHandoffTime = now;
                    _s.GfoldLastSolveTime = double.NegativeInfinity;
                    _s.GfoldPlan = null;
                    _s.GfoldApproach = GfoldApproach.BrakeAtGate;
                    _s.GfoldFailStreak = 0;
                    _s.GfoldTrackInit = false;
                    // The engine is lit from the burn, so the tracker's coast hysteresis starts from a lit engine and switches it off only when the plan really coasts.
                    _s.GfoldEngineOn = true;
                    _s.GfoldRelightRequestTime = double.NaN;
                    _s.GfoldHoverRefused = false;
                    // This step already writes the descent's engine command, and G-FOLD has no throttle until its first plan on the next step, so the burn's throttle carries over. Left at zero, the braking burn stops for a step, and for as long as the first solves find no plan. The tracker does not smooth from the carried value, because GfoldTrackInit is reset above.
                    _s.GfoldThrottle = _s.Upfg.Throttle;
                    _s.LandingStatus = sixDofRefusal == null
                        ? "Handoff to G-FOLD descent."
                        : $"Handoff to G-FOLD descent: 6-DOF cannot plan for this craft ({sixDofRefusal}).";
                }
                _s.GfoldTabSelectPending = true;   // focus the powered-landing page
            }
        }
    }

    // ----- Landing site geometry -----

    // The site's body-fixed (CCF) direction.
    // KSA's own convention: lat = asin(z), lon = atan2(y,x) in CCF.
    private static double3 SiteDirCcf() => SiteDirCcf(_s.SiteLatDeg, _s.SiteLonDeg);

    internal static double3 SiteDirCcf(double latDeg, double lonDeg)
    {
        double lat = UpfgTarget.DegToRad(latDeg);
        double lon = UpfgTarget.DegToRad(lonDeg);
        return new double3(
            Math.Cos(lat) * Math.Cos(lon),
            Math.Cos(lat) * Math.Sin(lon),
            Math.Sin(lat));
    }

    // The site's CCI direction dtFuture seconds from now.
    // A body spins about its own CCI Z axis (per IParentBody.GetAngularVelocityCci), so the future position is the current one carried around Z.
    private static double3 SiteDirCciAt(IParentBody parent, double dtFuture)
    {
        double3 dirNow = SiteDirCcf().Transform(parent.GetCcf2Cci());
        return RotZ(dirNow, parent.GetAngularVelocity() * dtFuture);
    }

    // Terrain height of the site above the body's mean-radius sphere, sampled from KSA's own heightmap (the game places surface objects at MeanRadius + this).
    // The site is fixed in CCF, so the value is cached and only re-sampled when the inputs (or the body) change.

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

}

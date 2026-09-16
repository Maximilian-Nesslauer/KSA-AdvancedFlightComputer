#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.Guidance.Upfg;

// The Ascent tab and its guidance: reads the controlled vehicle's live inertial (CCI) state, runs the standalone UPFG guidance toward a target orbit, and (optionally) commands the flight computer through a full ascent profile:
//
// Vertical flight continues straight up from the pad until the turn-start altitude.
// The turn phase pitches down from vertical at a fixed rate in deg/s toward the launch azimuth until the commanded pitch meets UPFG's command or reaches the failsafe altitude.
// The closed-loop phase flies the converged UPFG steering.
// At tgo <= 10 s, the terminal phase freezes the commanded attitude and counts down to cutoff. UPFG keeps iterating for the readouts.
public static partial class GuidanceWindow
{
    // The target orbit, launch-to-target pick, and gravity-turn shaping are stored in VehicleAutopilotState for the vehicle they describe. LAN is seeded from the vessel's position when its panel first draws and can be re-seeded with the adjacent button.
    private const double TerminalTgo = 10.0;
    // Hand over to UPFG no later than this altitude, even if the pitch profiles never crossed - the failsafe against an open-loop runaway vehicle.
    private const double FailsafeAltKm = 50.0;

    /// <summary>
    /// SIM SECONDS BETWEEN UPFG SOLVES. UPFG is a once-per-guidance-cycle algorithm,
    /// not a per-frame filter: each call is one refinement of a recursive solution,
    /// and rbias (rgo - rthrust) is an integrator across cycles. This port was calling
    /// it from the PrepareWorker prefix, i.e. EVERY SIM STEP (~60 Hz, and far more
    /// under warp), which wound that integrator up sixty times faster than the
    /// algorithm is damped for and fed the resulting wobble straight to the flight
    /// computer as a new attitude target every step. The reference implementation this
    /// The guidance cadence is one call per second, which is the interval used to tune these gains.
    ///
    /// It also makes <see cref="UpfgGuidance.Converged"/> mean something again: the
    /// test is "tgo settled between calls", which at 60 Hz passes on the second step
    /// no matter how far from converged the solution is - and the phase machine
    /// promotes the turn to closed loop on that flag.
    ///
    /// THE CADENCE IS NOT WHAT KEEPS THE ATTITUDE SMOOTH - the steering LAW is
    /// evaluated every step between solves (UpfgGuidance.SteeringAt), so the command
    /// is continuous whatever this is set to. All this decides is how often the
    /// solution behind that law is re-derived.
    /// </summary>
    private const double GuidanceCycle = 1.0;

    /// <summary>
    /// HOW FAR AHEAD OF THE PAD the LAN is seeded, in seconds.
    ///
    /// "The plane overhead right now" is the wrong plane to launch into: the pad is
    /// carried east at the body's rotation rate the whole time the vehicle is still
    /// climbing through the vertical rise and the turn, so by the time the ascent is
    /// actually flying, the site - and the vehicle with it - has moved off the plane
    /// that was overhead at lift-off, and the guidance yaws to chase a node it has
    /// already gone past. Seeding the plane over where the pad WILL be instead puts
    /// the ascent in the plane over the part of the flight that matters.
    /// </summary>
    private const double LanLeadSeconds = 180.0;

    /// <summary>
    /// Ceiling on how fast the COMMANDED direction may rotate, deg/s.
    ///
    /// A GUARD, NOT THE SMOOTHING. The steering law is evaluated continuously now, and
    /// a launch vehicle pitches at about 1 deg/s, so in normal flight nothing should
    /// come near this. What is left for it is the genuine discontinuities - the
    /// hand-over out of the open-loop turn, and a stage list changing shape mid-burn -
    /// where it turns an instantaneous attitude snap into a maneuver. The number is
    /// chosen to sit well above any rate the trajectory legitimately asks for, so
    /// binding it at all is a signal that something upstream jumped.
    /// </summary>
    private const double MaxSlewDegS = 5.0;

    /// <summary>Wall-clock gate on re-deriving an unarmed launch window, ms.</summary>
    private const long LaunchWindowIntervalMs = 250;

    /// <summary>
    /// Wall-clock gate on the insertion search, ms. The search's own interval is sim time, and warp packs several guidance cycles into one frame, so without this a warped frame could run a search every frame. A search is milliseconds of solving (see UpfgInsertionSearch), and it is advisory - the goal it moves slews in over tens of seconds anyway - so under warp it simply runs less often.
    /// </summary>
    private const long InsertionSearchIntervalMs = 1000;

    public enum AscentPhase { Vertical, Turn, ClosedLoop, Terminal }

    // Reset at the top of every Draw; see DrawAutoLaunchArming.
    private static bool _autoLaunchStepped;

    // The Ascent tab contains the target orbit, profile tuning, launch-to-target, and commit controls in one panel.
    private static void DrawAscentTab(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                      double bodyRadius)
    {
        // Seed the LAN from where the vessel is right now, once a vehicle exists.
        if (!_s.LanSeeded)
        {
            _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, parent);
            _s.LanSeeded = true;
        }

        // Both sections open by default: everything that shapes the ascent should be visible without hunting for it. They stay collapsible for when the panel needs to be compact.
        if (ImGui.CollapsingHeader("Target orbit", ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.InputDouble("Periapsis (km)", ref _s.PeKm);
            if (ImGui.IsItemDeactivatedAfterEdit())
                OrderApsides();
            ImGui.InputDouble("Apoapsis (km)", ref _s.ApKm);
            if (ImGui.IsItemDeactivatedAfterEdit())
                OrderApsides();
            ImGui.InputDouble("Inclination (deg)", ref _s.IncDeg);
            ImGui.InputDouble("LAN (deg)", ref _s.LanDeg);
            ImGui.SameLine();
            if (ImGui.Button("From position"))
                _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, parent);
            ImGui.Checkbox("Fix arg. of periapsis", ref _s.ArgPeFixed);
            ImGui.SameLine();
            using (new ImGuiDisabledScope(!_s.ArgPeFixed))
                ImGui.InputDouble("Arg. of periapsis (deg)", ref _s.ArgPeDeg);
            using (new ImGuiDisabledScope(_s.ArgPeFixed))
            {
                ImGui.Checkbox("Optimise insertion for dV", ref _s.OptimiseInsertion);
                ImGui.SameLine();
                ImGui.Checkbox("Insert before apoapsis", ref _s.InsertBeforeApoapsis);
            }
        }

        if (ImGui.CollapsingHeader("Ascent params", ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.InputDouble("Turn start alt (km)", ref _s.TurnStartAltKm);
            ImGui.InputDouble("Turn rate (deg/s)", ref _s.TurnRateDegS);
            ImGui.Checkbox("G-limit", ref _s.GLimitEnabled);
            ImGui.SameLine();
            ImGui.InputDouble("Max accel (g)", ref _s.GLimitG);

            // See the gauge panel's copy of this pair, and ForceRoll for what ticking it does to the flight computer's roll mode.
            ImGui.Checkbox("Force roll", ref _s.ForceRoll);
            ImGui.SameLine();
            using (new ImGuiDisabledScope(!_s.ForceRoll))
                ImGui.InputDouble("Roll angle (deg)", ref _s.ForceRollDeg);
        }

        // Draw the launch-to-target controls, which have their own launch-window logic.
        DrawLaunchToTarget(vehicle, orbit, parent, bodyRadius);

        // The toggles are pure configuration: nothing acts until EXECUTE starts the process (or the armed auto-launch fires it at the window). EXECUTE is the single commit point - guidance starts and whatever is toggled goes live at once, so you can warp time freely beforehand.
        ImGui.Checkbox("Engage autopilot", ref _s.Engage);
        ImGui.SameLine();
        ImGui.Checkbox("Auto engines/staging", ref _s.AutoStage);
        ImGui.Checkbox("Show target orbit & track", ref _showAscentOverlay);

        if (ImGui.Button("EXECUTE"))
            ExecuteAscent(vehicle, orbit, parent);
        ImGui.SameLine();
        if (ImGui.Button("Stop / reset"))
            AbortAscent();
        ImGui.SameLine();
        if (ImGui.Button("Clear track"))
            ResetTrace();
    }

    /// <summary>
    /// The single commit point. A TARGET is what decides whether this launches or
    /// arms: with one, there is a launch window to hit and firing now would miss the
    /// plane, so EXECUTE arms and guidance starts when the window arrives. Without a
    /// target - or with a target whose plane the site cannot reach, so there is no
    /// window at all - EXECUTE means EXECUTE.
    ///
    /// The auto-warp setting is deliberately NOT part of this decision. It only says
    /// whether the mod offers to warp to the window for you or you warp there
    /// yourself; either way the launch waits for the window.
    /// </summary>
    private static void ExecuteAscent(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        if (_s.TargetId.Length > 0 && !double.IsNaN(_s.LaunchTargetTime))
        {
            // ARMING TAKES THE VEHICLE TOO. It is a commit - the craft is now waiting to launch and will fire itself at the window - so leaving another mode running underneath it would have that mode flying right up to the moment StepLaunchWindow claimed it out from under itself.
            ClaimVehicle(GuidanceMode.Ascent, vehicle);
            _s.LaunchArmed = true;
            return;
        }
        StartGuidance(vehicle, orbit, parent);
    }

    /// <summary>Stop everything, including a pending armed launch. A panel action, so the craft is the focused one.</summary>
    private static void AbortAscent() => ReleaseAscent("", Program.ControlledVehicle);

    /// <summary>
    /// Hand the vehicle back and return the panel to a clean slate: guidance off, no
    /// pending launch, and the solver reset so nothing downstream reads a stale
    /// solution as if it were live.
    /// </summary>
    private static void ReleaseAscent(string status, Vehicle vehicle)
    {
        if (_s.Running || _s.LaunchArmed)
            GuidanceLog.Info(vehicle, $"ascent {(status.Length > 0 ? "ended: " + status : "stopped by the player")}"
                + $" (phase {PhaseName(_s.Phase)}, tgo {_s.Upfg.Tgo:F1} s, vgo {_s.Upfg.VgoMag:F0} m/s).");
        // The shutdown for this path comes from the release itself, so record the intent. Nothing here queues a one-shot cut that a later takeover could preserve on its own.
        _s.ShutdownRequested = true;
        _s.Running = false;
        _s.LaunchArmed = false;
        _s.HasCommand = false;
        _s.CommandDir = default;
        _s.RollLatched = false;
        _s.Upfg.Reset();
        _s.RgoPeak = 0.0;
        _s.VgoPeak = 0.0;
        _s.Status = status;
    }

    /// <summary>
    /// THE ARMED LAUNCH, STEPPED. Tracks the launch window, keeps the chase orbit
    /// pointed at the target, and fires EXECUTE when the window arrives.
    ///
    /// This runs from the simulation step for the vehicle it controls, so launch-window tracking and the armed launch do not depend on the visible panel.
    ///
    /// Runs from the sim step for the vehicle it belongs to, so the only thing the
    /// draw decides now is whether the status TEXT is visible.
    /// </summary>
    private static void StepLaunchWindow(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                         double bodyRadius)
    {
        if (_s.TargetId.Length == 0 || _s.Running)
            return;

        // ARMED: the launch instant is already latched and absolute, so this needs no geometry at all - and MUST not re-derive it. Checked every step, ahead of everything else here, because a single warp step can cross the window whole.
        if (_s.LaunchArmed && !double.IsNaN(_s.LaunchTargetTime))
        {
            // Ask to warp to just before the window (the warp itself needs the user's confirmation - see DrawWarpPrompt), then press EXECUTE for them. The engage/auto toggles are respected as configured, not forced.
            double waitSec = _s.LaunchTargetTime - SimNow();

            // <= rather than a window: with an absolute target this goes NEGATIVE on overshoot, so a single large warp step past the window still fires.
            if (waitSec <= 1.0)
            {
                if (Universe.IsAutoWarpActive)
                    Universe.AutoWarpStop(true);
                StartGuidance(vehicle, orbit, parent);
                _s.LaunchArmed = false;
            }
            else if (_s.AutoLaunch && waitSec > WarpLeadTime + 5.0 && !Universe.IsAutoWarpActive)
            {
                RequestWarp(_s.LaunchTargetTime - WarpLeadTime, "the launch window");
            }
            return;
        }

        // NOT ARMED: track the target's plane and the next window. Wall-clock gated like the stage model, and for the same two reasons - TryChaseOrbit searches the system for the target vehicle, which is not a per-sim-step cost, and under warp a sim-time gate would not throttle it at all.
        long now = Environment.TickCount64;
        if (now - _s.LaunchWindowTick < LaunchWindowIntervalMs)
            return;
        _s.LaunchWindowTick = now;

        ChaseStatus status = TryChaseOrbit(vehicle, orbit, parent, bodyRadius, out ChasePlan plan);

        // The chase orbit is an OUTPUT of the target pick, recomputed continuously - it has to keep tracking while the fold that displays it is shut, or the launch would fly to whatever plane was last on screen.
        if (status == ChaseStatus.Ok || status == ChaseStatus.PlaneUnreachable)
            ApplyChaseOrbit(in plan);

        if (status == ChaseStatus.Ok)
            _s.LaunchTargetTime = SimNow() + plan.WaitSec;
    }

    // Per-frame ascent stepping, run for this vehicle from ApplyAutopilot (the PrepareWorker prefix) whether or not it is the one on screen.
    private static void StepAscent(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                   double mu, double bodyRadius)
    {
        // Ascent guidance does not run forever: shortly after terminal cutoff it releases. Leaving it engaged kept CommandAttitude running on every sim step (thousands/s under warp) and held the flight computer in active attitude tracking - the lag that appeared with warp and lingered after. Released once the engines are actually out, or on the timeout if auto staging was never going to cut them. Releasing has to CLEAR the solver as well as the flag: leaving UPFG's recursive state loaded meant the panel went on showing the finished ascent's tgo and vgo, which is what made an abort look mandatory to get back to a clean slate.
        if (_s.Running && _s.Phase == AscentPhase.Terminal
            && (SimNow() > _s.CutoffTime + (_s.CutoffDone ? 2.0 : 15.0)))
        {
            ReleaseAscent("Ascent complete - guidance released.", vehicle);
        }

        if (!_s.Running)
            return;

        try
        {
            StepGuidance(vehicle, orbit, parent, mu, bodyRadius);
            if (_s.Engage && _s.AutoStage && !_s.CutoffDone)
                AutoSequence(vehicle);
            _s.FailStreak = 0;
            _s.GuidanceError = "";
        }
        catch (Exception e)
        {
            // Transient failures (staging frames, mid-mutation part trees) skip the step and keep flying the last solution; only a sustained streak means something is actually broken.
            _s.FailStreak++;
            _s.GuidanceError = e.Message;
            if (_s.FailStreak > MaxFailStreak)
            {
                _s.Running = false;
                _s.Status = "Guidance failed repeatedly: " + e.Message;
                GuidanceLog.Info(vehicle, $"ascent stopped after {_s.FailStreak} failed steps: {e}");
            }
        }
    }

    // The EXECUTE button's action - also fired automatically at the launch window.
    /// <summary>
    /// Periapsis is the lower apsis: a pair entered the other way round is swapped rather than flown as given. Called when either field is left after an edit - swapping on every keystroke would swap a value half typed - and again at launch, for a pair that reached the state any other way.
    /// </summary>
    private static void OrderApsides()
    {
        if (_s.ApKm < _s.PeKm)
            (_s.PeKm, _s.ApKm) = (_s.ApKm, _s.PeKm);
    }

    private static void StartGuidance(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        OrderApsides();

        // A target orbit wholly inside the atmosphere has nowhere the insertion floor allows (see UpfgTarget.InsideFloor): refused here rather than flown to a cutoff in the air.
        double atmosphere = parent?.GetAtmosphereRadius() ?? 0.0;
        if (atmosphere > 0.0 && Math.Max(_s.PeKm, _s.ApKm) * 1000.0 + parent.MeanRadius <= atmosphere)
        {
            string refusal = $"Target apoapsis is inside the atmosphere (top at {(atmosphere - parent.MeanRadius) / 1000.0:F0} km) - not launching.";
            if (_s.LaunchArmed)
            {
                ReleaseAscent(refusal, vehicle);
            }
            else
            {
                GuidanceLog.Info(vehicle, $"ascent refused: {refusal}");
                _s.Status = refusal;
            }
            return;
        }

        // Ascent takes over from every other mode: all four drive the same flight-computer command path, and two of them writing it would fight.
        ClaimVehicle(GuidanceMode.Ascent, vehicle);
        // Ours to reset, not the claim's - the deorbit burn flies this same instance.
        _s.Upfg.Reset();
        _s.GuidanceError = "";
        _s.Status = "";
        _s.FailStreak = 0;
        _s.Running = true;
        _s.Phase = AscentPhase.Vertical;
        _s.HasCommand = false;
        _s.CommandDir = default;   // nothing to be continuous with: the slew starts fresh
        _s.LastSolveTime = double.NegativeInfinity;
        _s.LastStepTime = SimNow();
        _s.RollLatched = false;    // re-measure the vehicle's roll at this engagement
        _s.CutoffDone = false;
        _s.ReserveStaged = false;   // a fresh ascent gets its reserve staging back
        _s.StagingActive = false;
        _s.LastSequenceTime = double.NegativeInfinity;
        _s.RgoPeak = 0.0;
        _s.VgoPeak = 0.0;
        _s.LastGuidanceLogTime = double.NegativeInfinity;
        GuidanceLog.Info(vehicle, $"ascent started to {_s.PeKm:F0} x {_s.ApKm:F0} km, inc {_s.IncDeg:F2} deg, LAN {_s.LanDeg:F2} deg"
            + $", arg. Pe {(_s.ArgPeFixed ? _s.ArgPeDeg.ToString("F2") + " deg" : "free")}"
            + $" (engage {_s.Engage}, auto engines and staging {_s.AutoStage}, g-limit {(_s.GLimitEnabled ? _s.GLimitG.ToString("F1") + " g" : "off")}"
            + $", reserve {(_s.ReserveArmed ? _s.ReserveKg.ToString("F0") + " kg" : "off")}, turn from {_s.TurnStartAltKm:F1} km at {_s.TurnRateDegS:F2} deg/s).");
    }

    // The launch-to-target panel: target picker, chase-orbit offset, node direction, window countdown, and (when armed) a warp request plus the launch trigger.
    private static void DrawLaunchToTarget(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                           double bodyRadius)
    {
        ImGui.SeparatorText("Launch to target");

        DrawTargetPicker(vehicle);
        ImGui.InputDouble("SMA offset below target (km)", ref _s.ChaseOffsetKm);
        if (ImGui.RadioButton("Ascending (NE)", !_s.LaunchDescending))
            _s.LaunchDescending = false;
        ImGui.SameLine();
        if (ImGui.RadioButton("Descending (SE)", _s.LaunchDescending))
            _s.LaunchDescending = true;
        ImGui.Checkbox("Auto warp to window", ref _s.AutoLaunch);

        // The geometry itself lives in Guidance/ChaseOrbit.cs, shared with the gauge panel so the two can never drift apart.
        ChaseStatus status = TryChaseOrbit(vehicle, orbit, parent, bodyRadius, out ChasePlan plan);
        switch (status)
        {
            case ChaseStatus.NoTarget:
                return;
            case ChaseStatus.NotFound:
                ImGui.TextColored(new float4(1f, 0.4f, 0.4f, 1f), "Target vehicle not found.");
                return;
            case ChaseStatus.DifferentBody:
                ImGui.TextColored(new float4(1f, 0.4f, 0.4f, 1f), "Target orbits a different body.");
                return;
        }

        ImGui.Text($"Target orbit:  {plan.TargetPeKm,7:F1} x {plan.TargetApKm,7:F1} km  inc {plan.IncDeg,6:F2} deg");
        ImGui.Text($"Chase orbit:   {plan.PeKm,7:F1} x {plan.ApKm,7:F1} km co-elliptic  (SMA {_s.ChaseOffsetKm:F0} km below target)");
        // Applied as soon as it changes, not only by the copy button or auto-launch below, so an ascent started straight from this tab flies the choice on screen.
        if (ImGui.Checkbox("Copy target arg. of periapsis", ref _s.MatchTargetArgPe))
            ApplyChaseOrbit(in plan);
        ImGui.SameLine();
        ImGui.Text(double.IsNaN(plan.ArgPeDeg) ? "(target has none)" : $"({plan.ArgPeDeg:F2} deg)");

        if (status == ChaseStatus.PlaneUnreachable)
        {
            ImGui.TextColored(new float4(1f, 0.6f, 0.3f, 1f),
                "Target inclination is below the site latitude - plane unreachable.");
            return;
        }

        double waitSec = plan.WaitSec;
        // The countdown is to IGNITION, which leads the plane crossing - see LanLeadSeconds - so the lead is named rather than left as an apparent discrepancy between T-0 and the site being in the plane.
        ImGui.Text($"Launch window: T-{waitSec,7:F0} s ({(_s.LaunchDescending ? "descending" : "ascending")} crossing, "
                 + $"{LanLeadSeconds:F0} s lead)");

        if (ImGui.Button("Copy chase orbit to target inputs") || _s.AutoLaunch)
            ApplyChaseOrbit(in plan);

        DrawAutoLaunchArming();
    }

    /// <summary>
    /// The armed auto-launch's STATUS TEXT. Guarded to run at most once per frame
    /// because both the tab and the gauge panel reach it and it would otherwise
    /// print twice with both open.
    ///
    /// StepLaunchWindow performs the launch action when the window arrives.
    /// </summary>
    private static void DrawAutoLaunchArming()
    {
        if (_autoLaunchStepped)
            return;
        _autoLaunchStepped = true;

        // TEXT ONLY. The launch itself is fired by StepLaunchWindow, from the sim step - see there for why it cannot live in a draw.
        double waitSec = _s.LaunchTargetTime - SimNow();
        if (_s.LaunchArmed && !_s.Running && !double.IsNaN(waitSec))
        {
            if (!_s.Engage || !_s.AutoStage)
                ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                    "Note: engage/auto toggles are off - auto-launch will only start guidance.");

            ImGui.TextColored(new float4(0.5f, 0.9f, 1f, 1f), Universe.IsAutoWarpActive
                ? "Auto-warping to the launch window..."
                : (_s.AutoLaunch
                    ? "Armed - will EXECUTE at the window."
                    : "Armed - warp to the window yourself; it will EXECUTE there."));
        }
        else if (_s.LaunchArmed && _s.Running)
        {
            ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                "Guidance already running - auto-launch is waiting (Stop / reset to clear).");
        }
    }

    private static void StepGuidance(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                     double mu, double bodyRadius)
    {
        double3 r = orbit.StateVectors.PositionCci;
        double3 v = orbit.StateVectors.VelocityCci;
        double now = SimNow();

        // Sim-time step for the command shaping below. Sim time, not wall clock, so a warp step is treated as the long interval it is; clamped because the first step after EXECUTE (and any warp jump) would otherwise hand it an interval that makes the slew limit meaningless in one direction or the other.
        double stepDt = Math.Clamp(now - _s.LastStepTime, 0.0, GuidanceCycle);
        _s.LastStepTime = now;

        // The ATTITUDE is frozen in the terminal phase, not the solver. Re-running UPFG over a near-zero remaining arc makes its steering chase itself, which is why the command holds _s.FrozenDir below - but the solve itself keeps running, so tgo, vgo and the stage model stay live in the readouts instead of freezing on whatever they happened to be ten seconds before cutoff.
        //
        // ONE SOLVE PER GUIDANCE CYCLE, not one per sim step - see GuidanceCycle. The phase machine and the commanded attitude below still run every step; only the solve is paced.
        if (now - _s.LastSolveTime >= GuidanceCycle)
        {
            // Rebuild from the live part tree every cycle so UPFG always sees current masses and the actual remaining staging sequence. No usable thrust is a normal transient during staging (old engine gone, new one not yet active): hold the last solution and wait rather than stopping. The cycle clock is NOT advanced in that case - the next step retries immediately rather than sitting out a whole cycle on a transient.
            UpfgVehicle live = BuildUpfgVehicle(vehicle);
            if (live == null)
            {
                if (_s.Status != NoThrustStatus)
                    GuidanceLog.Debug(vehicle, "no thrust to plan with, holding the last solution.");
                _s.Status = NoThrustStatus;
            }
            else
            {
                if (_s.Status == NoThrustStatus)
                    GuidanceLog.Debug(vehicle, "thrust is back, solving again.");
                // The reserve BEFORE the g-limit split, so a stage that gets divided is divided at the masses it will actually fly through. Applied to this copy only - the cached model stays the vehicle as it is, which is what the stage table and the staging cue both need it to be.
                if (_s.ReserveArmed)
                    ApplyAscentReserve(live, _s.ReserveKg);
                if (_s.GLimitEnabled && _s.GLimitG > 0.1)
                    ApplyGLimit(live, _s.GLimitG);
                _s.Status = "";
                _s.UpfgVehicle = live;
                // The first solve of a flight: the insertion search starts again from periapsis.
                bool firstSolve = double.IsNegativeInfinity(_s.LastSolveTime);
                if (firstSolve)
                    _s.InsertionSearch.Reset();
                // Before the target is built from the search's insertion, so a change of apsis is flown from this solve.
                _s.InsertionSearch.SetAnchor(_s.InsertBeforeApoapsis && !_s.ArgPeFixed);
                // The insertion floor is the top of the atmosphere, zero for an airless body: an insertion under it would cut the engines in the air. A free insertion goes where the search has moved it, which is periapsis until it has.
                var target = UpfgTarget.FromOrbit(_s.PeKm, _s.ApKm, _s.IncDeg, _s.LanDeg, bodyRadius, mu,
                    _s.ArgPeFixed ? _s.ArgPeDeg : double.NaN, parent?.GetAtmosphereRadius() ?? 0.0,
                    UpfgTarget.RadToDeg(_s.InsertionSearch.Nu));
                // dt is the interval this solve covers, which is what makes the convergence test rate-independent (see UpfgGuidance.Step).
                double solveDt = firstSolve ? 0.0 : now - _s.LastSolveTime;
                _s.Upfg.Step(r, v, vehicle.TotalMass, mu, target, _s.UpfgVehicle, 1, solveDt);

                // Straight after the solve, from the same state and model: where a free insertion costs least. Searched only from a converged closed-loop solution, because the open-loop turn is not flying the steering the costs assume, and at most once per InsertionSearchIntervalMs of wall clock.
                double goalBefore = _s.InsertionSearch.GoalNu;
                double searchedBefore = _s.InsertionSearch.LastSearchTime;
                long tick = Environment.TickCount64;
                _s.InsertionSearch.Step(now, solveDt, _s.OptimiseInsertion && !_s.ArgPeFixed,
                    _s.Phase == AscentPhase.ClosedLoop && _s.Upfg.Converged
                        && tick - _s.InsertionSearchTick >= InsertionSearchIntervalMs,
                    _s.Upfg, target, r, v, vehicle.TotalMass, mu, _s.UpfgVehicle);
                if (_s.InsertionSearch.LastSearchTime != searchedBefore)
                    _s.InsertionSearchTick = tick;
                if (_s.InsertionSearch.GoalNu != goalBefore)
                    GuidanceLog.Debug(vehicle, $"insertion search moved the goal from {UpfgTarget.RadToDeg(goalBefore):F1} to {UpfgTarget.RadToDeg(_s.InsertionSearch.GoalNu):F1} deg past periapsis"
                        + $" ({_s.InsertionSearch.SavingS:F1} s of burn, {_s.InsertionSearch.SavingMs:F0} m/s under inserting {(_s.InsertionSearch.AtApoapsis ? $"{UpfgInsertionSearch.ApoapsisLeadDeg:F0} deg before apoapsis" : "at periapsis")}; {_s.InsertionSearch.LastSearchPriced} of {_s.InsertionSearch.LastSearchPoints} insertions priced in {_s.InsertionSearch.LastSearchSolves} solves).");
                _s.LastSolveTime = now;

                // A sample of the solution every few seconds, so a log shows the steering the craft was given without a line per cycle.
                if (GuidanceLog.Enabled && now - _s.LastGuidanceLogTime >= GuidanceLogIntervalS)
                {
                    _s.LastGuidanceLogTime = now;
                    double3 up = double3.Normalize(r);
                    UpfgTarget.Insertion aim = _s.Upfg.Aim;
                    GuidanceLog.Debug(vehicle, $"UPFG {PhaseName(_s.Phase)}: tgo {_s.Upfg.Tgo:F1} s, vgo {_s.Upfg.VgoMag:F0} m/s, converged {_s.Upfg.Converged}"
                        + (aim.Valid && (target.ArgPeFixed || aim.FloorLimited || aim.TrueAnomaly != 0.0)
                            ? $", insertion {UpfgTarget.RadToDeg(aim.TrueAnomaly):F1} deg from Pe at {(aim.Radius - bodyRadius) / 1000.0:F0} km, FPA {UpfgTarget.RadToDeg(aim.Fpa):F2} deg{(_s.Upfg.AimHeld ? " (held)" : "")}"
                            : "")
                        + $", steer pitch {PitchOf(up, _s.Upfg.Steering):F1} deg, command pitch {PitchOf(up, _s.CommandDir):F1} deg"
                        + $", alt {(r.Length() - bodyRadius) / 1000.0:F1} km, speed {v.Length():F0} m/s, mass {vehicle.TotalMass / 1000.0:F1} t"
                        + $", model {live.Stages.Count} stage(s), S1 {live.Stages[0].Thrust / 1000.0:F0} kN {(live.Stages[0].MassTotal - live.Stages[0].MassDry) / (live.Stages[0].Thrust / (live.Stages[0].Isp * 9.80665)):F0} s.");
                }
            }
        }

        UpdatePhase(vehicle, r, bodyRadius, stepDt);
    }

    private const string NoThrustStatus = "No thrust - holding last solution (staging/coast).";
    private const double GuidanceLogIntervalS = 5.0;

    // Ascent phase state machine. Transitions cascade naturally over successive frames, so initializing mid-flight fast-forwards to the right phase. stepDt is the sim time since the previous step, for the command slew limit.
    private static void UpdatePhase(Vehicle vehicle, double3 r, double bodyRadius, double stepDt)
    {
        double3 up = double3.Normalize(r);
        double alt = r.Length() - bodyRadius;
        double turnPitch = TurnPitchDeg();

        // THE STEERING LAW EVALUATED ONCE, HERE, and used for everything downstream: the pitch the turn hands over at, the azimuth it flies, the closed-loop command, and the direction the terminal phase freezes. UpfgGuidance.Steering is the tau = 0 sample from the last solve - up to a cycle old - and mixing the two would have the phase machine deciding against one direction while the vehicle flew another.
        //
        // tau is clamped to one cycle: past that the solve is overdue (a staging transient holding the last solution, say) and extrapolating a linearisation nobody has refreshed is how a small rate becomes a large angle.
        double tau = Math.Clamp(SimNow() - _s.LastSolveTime, 0.0, GuidanceCycle);
        double3 steerNow = _s.Upfg.SteeringAt(tau, out double3 steerRate);
        double upfgPitch = PitchOf(up, steerNow);

        switch (_s.Phase)
        {
            case AscentPhase.Vertical:
                if (alt >= _s.TurnStartAltKm * 1000.0)
                {
                    _s.Phase = AscentPhase.Turn;
                    _s.TurnStartTime = SimNow();
                    GuidanceLog.Debug(vehicle, $"turn starts at {alt / 1000.0:F1} km.");
                }
                break;

            case AscentPhase.Turn:
                // Pitch ramps down at the fixed rate; hand over to UPFG when it meets the closed-loop solution - or at the failsafe altitude regardless, so an open-loop profile can't run away.
                if ((_s.Upfg.Converged && turnPitch <= upfgPitch)
                    || alt >= FailsafeAltKm * 1000.0)
                {
                    _s.Phase = AscentPhase.ClosedLoop;
                    GuidanceLog.Debug(vehicle, $"closed loop from {alt / 1000.0:F1} km, turn pitch {turnPitch:F1} deg, UPFG pitch {upfgPitch:F1} deg, converged {_s.Upfg.Converged}.");
                }
                break;

            case AscentPhase.ClosedLoop:
                if (_s.Upfg.Converged && _s.Upfg.Tgo <= TerminalTgo)
                {
                    _s.Phase = AscentPhase.Terminal;
                    _s.FrozenDir = steerNow;
                    // tgo is measured from the SOLVE, not from now - the solution can be most of a guidance cycle old by the time this trips, and counting that cycle twice is a whole second of extra burn.
                    _s.CutoffTime = _s.LastSolveTime + _s.Upfg.Tgo;
                    GuidanceLog.Debug(vehicle, $"terminal phase, attitude frozen at pitch {upfgPitch:F1} deg, cutoff in {_s.CutoffTime - SimNow():F1} s.");
                }
                break;
        }

        // WHERE TO POINT, AND HOW FAST THAT POINT IS MOVING. Both, every step: the flight computer tracks a target's rate as well as its angle (see KsaAttitudeRate), and handing it only the angle declares a moving target stationary, which is the difference between tracking and chasing.
        double3 want;
        double3 wantRate = default;
        switch (_s.Phase)
        {
            case AscentPhase.Vertical:
                want = up;                       // holds still in inertial terms
                break;

            case AscentPhase.Turn:
                want = TurnDir(up, turnPitch, steerNow);
                // The ramp's own rate: pitching down at TurnRateDegS about the axis the turn plane turns about, and zero once it has bottomed out on the horizon. The axis is up x want - a right-handed rotation about it carries the command AWAY from vertical and toward down-range, which is the direction this ramp goes. (up x down-range, the same axis: with up = Z and down-range = X that is +Y, and +Y turns Z toward X.)
                if (turnPitch > 0.0)
                {
                    double3 axis = double3.Cross(up, want);
                    if (axis.Length() > 1e-9)
                        wantRate = double3.Normalize(axis) * UpfgTarget.DegToRad(_s.TurnRateDegS);
                }
                break;

            case AscentPhase.ClosedLoop:
                // The law and the rate it implies, both from the evaluation above. i_f(tau) is continuous between cycles, so the guidance cadence stops being something the attitude has to be protected from.
                want = steerNow;
                wantRate = steerRate;
                break;

            case AscentPhase.Terminal:
                want = _s.FrozenDir;             // frozen: no angle change, no rate
                break;

            default:
                want = default;
                break;
        }

        // The command SLEWS toward the solution rather than jumping to it. With the law evaluated continuously above this should never bind in flight - what is left for it is the hand-over out of the open-loop turn and whatever a stage list changing shape mid-burn does to the solution. When it DOES bind, the commanded rate is the slew's own, not the law's: publishing a feedforward the command is not actually following would have the flight computer drive toward one rate while the target moved at another.
        double maxRad = UpfgTarget.DegToRad(MaxSlewDegS) * stepDt;
        double3 slewed = SlewToward(_s.CommandDir, want, maxRad, out bool clamped);
        if (clamped && stepDt > 1e-9)
        {
            double3 axis = double3.Cross(_s.CommandDir, slewed);
            wantRate = axis.Length() > 1e-12
                ? double3.Normalize(axis) * (maxRad / stepDt)
                : default;
        }

        _s.CommandDir = slewed;
        _s.CommandRate = wantRate;
        _s.HasCommand = _s.CommandDir.Length() > 0.5;
    }

    /// <summary>
    /// Rotate <paramref name="current"/> toward <paramref name="target"/> by at most
    /// maxRad, about the axis between them. An uninitialised or degenerate current
    /// direction snaps straight to the target - there is nothing to be continuous
    /// with on the first command of a launch.
    /// </summary>
    private static double3 SlewToward(double3 current, double3 target, double maxRad,
                                      out bool clamped)
    {
        clamped = false;
        if (target.Length() < 0.5)
            return target;                       // nothing commanded
        target = double3.Normalize(target);
        if (current.Length() < 0.5 || maxRad <= 0.0)
            return target;                       // first command, or a zero-length step

        current = double3.Normalize(current);
        double angle = AngleBetween(current, target);
        if (angle <= maxRad)
            return target;

        double3 axis = double3.Cross(current, target);
        if (axis.Length() < 1e-12)
            return target;                       // exactly opposed: no axis to turn about
        clamped = true;
        return RotateAbout(current, double3.Normalize(axis), maxRad);
    }

    // The open-loop turn's commanded pitch: down from vertical at the fixed rate since the turn started, never below the horizon.
    private static double TurnPitchDeg()
    {
        if (_s.Phase != AscentPhase.Turn)
            return 90.0;
        double elapsed = SimNow() - _s.TurnStartTime;
        return Math.Max(90.0 - _s.TurnRateDegS * elapsed, 0.0);
    }

    // The gravity-turn attitude: the given pitch above the horizon, along UPFG's own launch azimuth once it has converged, with the classic inclination/latitude formula as the fallback until then.
    //
    // TAKING THE AZIMUTH FROM THE SOLVER IS THE POINT, not a shortcut. The turn hands over to that same steering vector, so flying its azimuth is what makes the hand-over seamless - the pitch profiles are matched before the switch and the heading already agrees. Deriving the azimuth from the target plane geometry instead looks more principled and is not: UPFG's azimuth includes the yaw that cancels the launch site's own eastward velocity, so a geometric one differs from it by a few degrees, and the whole of that difference then arrives as a heading change at the hand-over.
    private static double3 TurnDir(double3 up, double pitchDeg, double3 steerNow)
    {
        (double3 east, double3 north) = EnuBasis(up);

        double az;
        double3 steerHoriz = steerNow - double3.Dot(steerNow, up) * up;
        if (_s.Upfg.Converged && steerHoriz.Length() > 1e-3)
        {
            az = Math.Atan2(double3.Dot(steerHoriz, east), double3.Dot(steerHoriz, north));
        }
        else
        {
            double lat = Math.Asin(Math.Clamp(up.Z, -1.0, 1.0));
            double inc = UpfgTarget.DegToRad(_s.IncDeg);
            az = Math.Asin(Math.Clamp(Math.Cos(inc) / Math.Max(Math.Cos(lat), 1e-6), -1.0, 1.0));
            if (_s.LaunchDescending)
                az = Math.PI - az; // south-easterly at the descending crossing
        }

        double pitch = UpfgTarget.DegToRad(pitchDeg);
        return Math.Sin(pitch) * up
             + Math.Cos(pitch) * (Math.Cos(az) * north + Math.Sin(az) * east);
    }

    /// <summary>
    /// The target orbit plane's normal in CCI - the roll reference the commanded
    /// attitude is built on (see AscentRollRef).
    /// </summary>
    private static double3 AscentPlaneNormal() => UpfgTarget.OrbitNormal(
        UpfgTarget.DegToRad(_s.IncDeg), UpfgTarget.DegToRad(_s.LanDeg));

    private static string PhaseName(AscentPhase p) => p switch
    {
        AscentPhase.Vertical => "vertical rise",
        AscentPhase.Turn => "gravity turn (open loop)",
        AscentPhase.ClosedLoop => "UPFG closed loop",
        AscentPhase.Terminal => "terminal (frozen)",
        _ => "?",
    };

    /// <summary>
    /// The LAN of the plane with inclination incDeg that passes over the given CCI
    /// position - not now, but <see cref="LanLeadSeconds"/> from now.
    ///
    /// Spherical trig: sin(lat) = sin(inc)*sin(u) on the orbit, so the node sits
    /// asin(tan lat / tan inc) of right ascension behind the site's meridian. The
    /// lead is a straight addition to the answer: turning the site east by an angle
    /// moves its right ascension by that angle and leaves its latitude alone, and the
    /// node offset depends only on the latitude.
    ///
    /// Ascending-node solution, i.e. a north-easterly launch.
    /// </summary>
    private static double LanOverhead(double3 r, double incDeg, IParentBody parent)
    {
        double len = r.Length();
        if (len < 1)
            return 0;

        double lat = Math.Asin(Math.Clamp(r.Z / len, -1.0, 1.0));
        double ra = Math.Atan2(r.Y, r.X);

        // Where the pad will be by the time the vehicle is actually flying the plane.
        double omega = parent?.GetAngularVelocity() ?? 0.0;
        if (double.IsFinite(omega))
            ra += omega * LanLeadSeconds;

        double inc = UpfgTarget.DegToRad(incDeg);
        // A plane can only contain the site if |inc| >= |lat|; clamp gives the closest achievable plane (node 90 deg back) otherwise.
        double sinDl = Math.Tan(lat) / Math.Tan(Math.Max(Math.Abs(inc), 1e-6));
        double dl = Math.Asin(Math.Clamp(sinDl, -1.0, 1.0));

        double lanDeg = UpfgTarget.RadToDeg(ra - dl) % 360.0;
        if (lanDeg < 0)
            lanDeg += 360.0;
        return lanDeg;
    }
}

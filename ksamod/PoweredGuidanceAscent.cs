using System;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using PoweredGuidance.Upfg;

// The Ascent tab and its guidance: reads the controlled vehicle's live inertial
// (CCI) state, runs the standalone UPFG guidance toward a target orbit, and
// (optionally) commands the flight computer through a full ascent profile:
//
//   Vertical   — straight up off the pad until the turn-start altitude
//   Turn       — open-loop gravity turn: pitch down from vertical at a fixed rate
//                (deg/s) toward the launch azimuth, until the commanded pitch
//                meets UPFG's commanded pitch — or until the failsafe altitude
//   ClosedLoop — fly the converged UPFG steering
//   Terminal   — at tgo <= 10 s, freeze the commanded ATTITUDE (re-solving on a
//                near-zero arc just makes the steering chase itself) and count down
//                to cutoff. UPFG keeps iterating throughout, for the readouts.
public static partial class PoweredGuidanceWindow
{
    // The target orbit, the launch-to-target pick and the gravity-turn shaping all
    // live on the vehicle now (VehicleAutopilotState): they describe one craft's
    // mission, and sharing them meant focusing a second vehicle re-aimed the first.
    // LAN is seeded from the vessel's own position the first time its panel draws,
    // and can be re-seeded with the button next to the input.
    private const double TerminalTgo = 10.0;
    // Hand over to UPFG no later than this altitude, even if the pitch profiles
    // never crossed — the failsafe against an open-loop runaway vehicle.
    private const double FailsafeAltKm = 50.0;

    public enum AscentPhase { Vertical, Turn, ClosedLoop, Terminal }

    // Reset at the top of every Draw; see DrawAutoLaunchArming.
    private static bool _autoLaunchStepped;

    // The Ascent tab body: target orbit, profile tuning, launch-to-target, and the
    // commit controls. Everything the user sets is in this one panel — the profile
    // parameters used to live in a separate popup window, which meant tuning them
    // hid the guidance readout behind a second window.
    private static void DrawAscentTab(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                      double bodyRadius)
    {
        // Seed the LAN from where the vessel is right now, once a vehicle exists.
        if (!_s.LanSeeded)
        {
            _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg);
            _s.LanSeeded = true;
        }

        // Both sections open by default: everything that shapes the ascent should
        // be visible without hunting for it. They stay collapsible for when the
        // panel needs to be compact.
        if (ImGui.CollapsingHeader("Target orbit", ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.InputDouble("Periapsis (km)", ref _s.PeKm);
            ImGui.InputDouble("Apoapsis (km)", ref _s.ApKm);
            ImGui.InputDouble("Inclination (deg)", ref _s.IncDeg);
            ImGui.InputDouble("LAN (deg)", ref _s.LanDeg);
            ImGui.SameLine();
            if (ImGui.Button("From position"))
                _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg);
        }

        if (ImGui.CollapsingHeader("Ascent params", ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.InputDouble("Turn start alt (km)", ref _s.TurnStartAltKm);
            ImGui.InputDouble("Turn rate (deg/s)", ref _s.TurnRateDegS);
            ImGui.Checkbox("G-limit", ref _s.GLimitEnabled);
            ImGui.SameLine();
            ImGui.InputDouble("Max accel (g)", ref _s.GLimitG);
        }

        // --- Launch to target (runs its own launch-window logic, not collapsed) ---
        DrawLaunchToTarget(vehicle, orbit, parent, bodyRadius);

        // The toggles are pure configuration: nothing acts until EXECUTE
        // starts the process (or the armed auto-launch fires it at the
        // window). EXECUTE is the single commit point — guidance starts and
        // whatever is toggled goes live at once, so you can warp time
        // freely beforehand.
        ImGui.Checkbox("Engage autopilot", ref _s.Engage);
        ImGui.SameLine();
        ImGui.Checkbox("Auto engines/staging", ref _s.AutoStage);
        ImGui.Checkbox("Show target orbit & track", ref _showAscentOverlay);

        if (ImGui.Button("EXECUTE"))
            ExecuteAscent(orbit, parent);
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
    /// target — or with a target whose plane the site cannot reach, so there is no
    /// window at all — EXECUTE means EXECUTE.
    ///
    /// The auto-warp setting is deliberately NOT part of this decision. It only says
    /// whether the mod offers to warp to the window for you or you warp there
    /// yourself; either way the launch waits for the window.
    /// </summary>
    private static void ExecuteAscent(Orbit orbit, IParentBody parent)
    {
        if (_s.TargetId.Length > 0 && !double.IsNaN(_s.LaunchTargetTime))
        {
            _s.LaunchArmed = true;
            return;
        }
        StartGuidance(orbit, parent);
    }

    /// <summary>Stop everything, including a pending armed launch.</summary>
    private static void AbortAscent() => ReleaseAscent("");

    /// <summary>
    /// Hand the vehicle back and return the panel to a clean slate: guidance off, no
    /// pending launch, and the solver reset so nothing downstream reads a stale
    /// solution as if it were live.
    /// </summary>
    private static void ReleaseAscent(string status)
    {
        _s.Running = false;
        _s.LaunchArmed = false;
        _s.HasCommand = false;
        _s.Upfg.Reset();
        _s.RgoPeak = 0.0;
        _s.VgoPeak = 0.0;
        _s.Status = status;
    }

    // Per-frame ascent stepping, run for this vehicle from ApplyAutopilot (the
    // PrepareWorker prefix) whether or not it is the one on screen.
    private static void StepAscent(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                   double mu, double bodyRadius)
    {
        // Ascent guidance does not run forever: shortly after terminal cutoff it
        // releases. Leaving it engaged kept CommandAttitude running on every sim
        // step (thousands/s under warp) and held the flight computer in active
        // attitude tracking — the lag that appeared with warp and lingered after.
        // Released once the engines are actually out, or on the timeout if auto
        // staging was never going to cut them. Releasing has to CLEAR the solver as
        // well as the flag: leaving UPFG's recursive state loaded meant the panel went
        // on showing the finished ascent's tgo and vgo, which is what made an abort
        // look mandatory to get back to a clean slate.
        if (_s.Running && _s.Phase == AscentPhase.Terminal
            && (SimNow() > _s.CutoffTime + (_s.CutoffDone ? 2.0 : 15.0)))
        {
            ReleaseAscent("Ascent complete — guidance released.");
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
            // Transient failures (staging frames, mid-mutation part trees) skip
            // the step and keep flying the last solution; only a sustained streak
            // means something is actually broken.
            _s.FailStreak++;
            _s.GuidanceError = e.Message;
            if (_s.FailStreak > MaxFailStreak)
                _s.Running = false;
        }
    }

    // The EXECUTE button's action — also fired automatically at the launch window.
    private static void StartGuidance(Orbit orbit, IParentBody parent)
    {
        _s.LandingPhase = LandingPhase.Idle; // ascent takes over from any landing flow
        _s.Upfg.Reset();
        _s.GuidanceError = "";
        _s.Status = "";
        _s.FailStreak = 0;
        _s.Running = true;
        _s.Phase = AscentPhase.Vertical;
        _s.HasCommand = false;
        _s.CutoffDone = false;
        _s.StagingActive = false;
        _s.LastSequenceTime = double.NegativeInfinity;
        _s.RgoPeak = 0.0;
        _s.VgoPeak = 0.0;
    }

    // The launch-to-target panel: target picker, chase-orbit offset, node direction,
    // window countdown, and (when armed) a warp request plus the launch trigger.
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

        // The geometry itself lives in PoweredGuidanceChaseOrbit.cs, shared with the
        // gauge panel so the two can never drift apart.
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
        ImGui.Text($"Chase orbit:   {plan.PeKm,7:F1} km circular  (SMA {_s.ChaseOffsetKm:F0} km below target)");

        if (status == ChaseStatus.PlaneUnreachable)
        {
            ImGui.TextColored(new float4(1f, 0.6f, 0.3f, 1f),
                "Target inclination is below the site latitude — plane unreachable.");
            return;
        }

        double waitSec = plan.WaitSec;
        ImGui.Text($"Launch window: T-{waitSec,7:F0} s ({(_s.LaunchDescending ? "descending" : "ascending")} crossing)");

        if (ImGui.Button("Copy chase orbit to target inputs") || _s.AutoLaunch)
            ApplyChaseOrbit(in plan);

        DrawAutoLaunchArming(orbit, parent);
    }

    /// <summary>
    /// The armed auto-launch: warp toward the window, then EXECUTE at it. Guarded to
    /// run AT MOST ONCE PER FRAME, because both the legacy tab and the gauge panel
    /// reach it — and it fires real actions (a warp request, and starting guidance),
    /// which must not happen twice because two panels happen to be open. The first
    /// caller of the frame wins; the legacy tab draws first when its tab is selected,
    /// and the gauge panel picks it up when it is not.
    /// </summary>
    private static void DrawAutoLaunchArming(Orbit orbit, IParentBody parent)
    {
        if (_autoLaunchStepped)
            return;
        _autoLaunchStepped = true;

        // Armed: ask to warp to just before the window (the warp itself needs the
        // user's confirmation — see DrawWarpPrompt), then press EXECUTE for them.
        // The engage/auto toggles are respected as configured, not forced.
        double waitSec = _s.LaunchTargetTime - SimNow();
        if (_s.LaunchArmed && !_s.Running && !double.IsNaN(waitSec))
        {
            if (!_s.Engage || !_s.AutoStage)
                ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                    "Note: engage/auto toggles are off — auto-launch will only start guidance.");

            // <= rather than a window: with an absolute target this goes NEGATIVE on
            // overshoot, so a single large warp step past the window still fires.
            if (waitSec <= 1.0)
            {
                if (Universe.IsAutoWarpActive)
                    Universe.AutoWarpStop(true);
                StartGuidance(orbit, parent);
                _s.LaunchArmed = false;
            }
            else if (_s.AutoLaunch && waitSec > WarpLeadTime + 5.0 && !Universe.IsAutoWarpActive)
            {
                RequestWarp(_s.LaunchTargetTime - WarpLeadTime, "the launch window");
            }

            ImGui.TextColored(new float4(0.5f, 0.9f, 1f, 1f), Universe.IsAutoWarpActive
                ? "Auto-warping to the launch window..."
                : (_s.AutoLaunch
                    ? "Armed — will EXECUTE at the window."
                    : "Armed — warp to the window yourself; it will EXECUTE there."));
        }
        else if (_s.LaunchArmed && _s.Running)
        {
            ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                "Guidance already running — auto-launch is waiting (Stop / reset to clear).");
        }
    }

    private static void StepGuidance(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                     double mu, double bodyRadius)
    {
        double3 r = orbit.StateVectors.PositionCci;
        double3 v = orbit.StateVectors.VelocityCci;

        // The ATTITUDE is frozen in the terminal phase, not the solver. Re-running
        // UPFG over a near-zero remaining arc makes its steering chase itself, which
        // is why the command holds _s.FrozenDir below — but the solve itself keeps
        // running, so tgo, vgo and the stage model stay live in the readouts instead
        // of freezing on whatever they happened to be ten seconds before cutoff.
        {
            // Rebuild from the live part tree every step so UPFG always sees current
            // masses and the actual remaining staging sequence. No usable thrust is a
            // normal transient during staging (old engine gone, new one not yet
            // active): hold the last solution and wait rather than stopping.
            UpfgVehicle live = BuildUpfgVehicle(vehicle);
            if (live == null)
            {
                _s.Status = "No thrust — holding last solution (staging/coast).";
            }
            else
            {
                if (_s.GLimitEnabled && _s.GLimitG > 0.1)
                    ApplyGLimit(live, _s.GLimitG);
                _s.Status = "";
                _s.UpfgVehicle = live;
                var target = UpfgTarget.FromOrbit(_s.PeKm, _s.ApKm, _s.IncDeg, _s.LanDeg, bodyRadius, mu);
                _s.Upfg.Step(r, v, vehicle.TotalMass, mu, target, _s.UpfgVehicle);
            }
        }

        UpdatePhase(r, bodyRadius);
    }

    // Ascent phase state machine. Transitions cascade naturally over successive
    // frames, so initializing mid-flight fast-forwards to the right phase.
    private static void UpdatePhase(double3 r, double bodyRadius)
    {
        double3 up = double3.Normalize(r);
        double alt = r.Length() - bodyRadius;
        double upfgPitch = PitchOf(up, _s.Upfg.Steering);
        double turnPitch = TurnPitchDeg();

        switch (_s.Phase)
        {
            case AscentPhase.Vertical:
                if (alt >= _s.TurnStartAltKm * 1000.0)
                {
                    _s.Phase = AscentPhase.Turn;
                    _s.TurnStartTime = SimNow();
                }
                break;

            case AscentPhase.Turn:
                // Pitch ramps down at the fixed rate; hand over to UPFG when it
                // meets the closed-loop solution — or at the failsafe altitude
                // regardless, so an open-loop profile can't run away.
                if ((_s.Upfg.Converged && turnPitch <= upfgPitch)
                    || alt >= FailsafeAltKm * 1000.0)
                    _s.Phase = AscentPhase.ClosedLoop;
                break;

            case AscentPhase.ClosedLoop:
                if (_s.Upfg.Converged && _s.Upfg.Tgo <= TerminalTgo)
                {
                    _s.Phase = AscentPhase.Terminal;
                    _s.FrozenDir = _s.Upfg.Steering;
                    _s.CutoffTime = SimNow() + _s.Upfg.Tgo;
                }
                break;
        }

        switch (_s.Phase)
        {
            case AscentPhase.Vertical:
                _s.CommandDir = up;
                break;
            case AscentPhase.Turn:
                _s.CommandDir = TurnDir(up, turnPitch);
                break;
            case AscentPhase.ClosedLoop:
                _s.CommandDir = _s.Upfg.Steering;
                break;
            case AscentPhase.Terminal:
                _s.CommandDir = _s.FrozenDir;
                break;
        }
        _s.HasCommand = _s.CommandDir.Length() > 0.5;
    }

    // The open-loop turn's commanded pitch: down from vertical at the fixed rate
    // since the turn started, never below the horizon.
    private static double TurnPitchDeg()
    {
        if (_s.Phase != AscentPhase.Turn)
            return 90.0;
        double elapsed = SimNow() - _s.TurnStartTime;
        return Math.Max(90.0 - _s.TurnRateDegS * elapsed, 0.0);
    }

    // The gravity-turn attitude: the given pitch above the horizon, toward the
    // launch azimuth. Azimuth comes from UPFG's converged steering when available
    // (it knows the target plane), with the classic inclination/latitude formula
    // as fallback.
    private static double3 TurnDir(double3 up, double pitchDeg)
    {
        (double3 east, double3 north) = EnuBasis(up);

        double az;
        double3 steerHoriz = _s.Upfg.Steering - double3.Dot(_s.Upfg.Steering, up) * up;
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

    private static string PhaseName(AscentPhase p) => p switch
    {
        AscentPhase.Vertical => "vertical rise",
        AscentPhase.Turn => "gravity turn (open loop)",
        AscentPhase.ClosedLoop => "UPFG closed loop",
        AscentPhase.Terminal => "terminal (frozen)",
        _ => "?",
    };

    // The LAN of the plane with inclination incDeg that passes over the given CCI
    // position right now (ascending-node solution, i.e. a north-easterly launch).
    // Spherical trig: sin(lat) = sin(inc)·sin(u) on the orbit, and the node sits
    // asin(tan lat / tan inc) of right ascension behind the site's meridian.
    private static double LanOverhead(double3 r, double incDeg)
    {
        double len = r.Length();
        if (len < 1)
            return 0;

        double lat = Math.Asin(Math.Clamp(r.Z / len, -1.0, 1.0));
        double ra = Math.Atan2(r.Y, r.X);

        double inc = UpfgTarget.DegToRad(incDeg);
        // A plane can only contain the site if |inc| >= |lat|; clamp gives the
        // closest achievable plane (node 90° back) otherwise.
        double sinDl = Math.Tan(lat) / Math.Tan(Math.Max(Math.Abs(inc), 1e-6));
        double dl = Math.Asin(Math.Clamp(sinDl, -1.0, 1.0));

        double lanDeg = UpfgTarget.RadToDeg(ra - dl) % 360.0;
        if (lanDeg < 0)
            lanDeg += 360.0;
        return lanDeg;
    }
}

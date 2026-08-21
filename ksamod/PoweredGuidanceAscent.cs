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

    /// <summary>
    /// SIM SECONDS BETWEEN UPFG SOLVES. UPFG is a once-per-guidance-cycle algorithm,
    /// not a per-frame filter: each call is one refinement of a recursive solution,
    /// and rbias (rgo - rthrust) is an integrator across cycles. This port was calling
    /// it from the PrepareWorker prefix, i.e. EVERY SIM STEP (~60 Hz, and far more
    /// under warp), which wound that integrator up sixty times faster than the
    /// algorithm is damped for and fed the resulting wobble straight to the flight
    /// computer as a new attitude target every step. The reference implementation this
    /// was ported from (legacy/navbox) ran its simulator at dt = 1 s, so one call per
    /// second is the cadence every gain in here was tuned at.
    ///
    /// It also makes <see cref="UpfgGuidance.Converged"/> mean something again: the
    /// test is "tgo settled between calls", which at 60 Hz passes on the second step
    /// no matter how far from converged the solution is — and the phase machine
    /// promotes the turn to closed loop on that flag.
    /// </summary>
    private const double GuidanceCycle = 1.0;

    /// <summary>
    /// HOW FAR AHEAD OF THE PAD the LAN is seeded, in seconds.
    ///
    /// "The plane overhead right now" is the wrong plane to launch into: the pad is
    /// carried east at the body's rotation rate the whole time the vehicle is still
    /// climbing through the vertical rise and the turn, so by the time the ascent is
    /// actually flying, the site — and the vehicle with it — has moved off the plane
    /// that was overhead at lift-off, and the guidance yaws to chase a node it has
    /// already gone past. Seeding the plane over where the pad WILL be instead puts
    /// the ascent in the plane over the part of the flight that matters.
    /// </summary>
    private const double LanLeadSeconds = 180.0;

    /// <summary>
    /// Ceiling on how fast the COMMANDED direction may rotate, deg/s. The solution
    /// only moves once per <see cref="GuidanceCycle"/> and a real launch vehicle
    /// pitches at about 1 deg/s, so this both smooths the once-a-second step into
    /// continuous motion and keeps a hand-over (or a staging transient) from arriving
    /// as an instantaneous attitude snap. Well above any rate the trajectory actually
    /// asks for, so it shapes the command without changing where it ends up.
    /// </summary>
    private const double MaxSlewDegS = 5.0;

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
            _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, parent);
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
                _s.LanDeg = LanOverhead(orbit.StateVectors.PositionCci, _s.IncDeg, parent);
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
        _s.CommandDir = default;
        _s.RollLatched = false;
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
        _s.CommandDir = default;   // nothing to be continuous with: the slew starts fresh
        _s.LastSolveTime = double.NegativeInfinity;
        _s.LastStepTime = SimNow();
        _s.RollLatched = false;    // re-measure the vehicle's roll at this engagement
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
        // The countdown is to IGNITION, which leads the plane crossing — see
        // LanLeadSeconds — so the lead is named rather than left as an apparent
        // discrepancy between T-0 and the site being in the plane.
        ImGui.Text($"Launch window: T-{waitSec,7:F0} s ({(_s.LaunchDescending ? "descending" : "ascending")} crossing, "
                 + $"{LanLeadSeconds:F0} s lead)");

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
        double now = SimNow();

        // Sim-time step for the command shaping below. Sim time, not wall clock, so a
        // warp step is treated as the long interval it is; clamped because the first
        // step after EXECUTE (and any warp jump) would otherwise hand it an interval
        // that makes the slew limit meaningless in one direction or the other.
        double stepDt = Math.Clamp(now - _s.LastStepTime, 0.0, GuidanceCycle);
        _s.LastStepTime = now;

        // The ATTITUDE is frozen in the terminal phase, not the solver. Re-running
        // UPFG over a near-zero remaining arc makes its steering chase itself, which
        // is why the command holds _s.FrozenDir below — but the solve itself keeps
        // running, so tgo, vgo and the stage model stay live in the readouts instead
        // of freezing on whatever they happened to be ten seconds before cutoff.
        //
        // ONE SOLVE PER GUIDANCE CYCLE, not one per sim step — see GuidanceCycle. The
        // phase machine and the commanded attitude below still run every step; only
        // the solve is paced.
        if (now - _s.LastSolveTime >= GuidanceCycle)
        {
            // Rebuild from the live part tree every cycle so UPFG always sees current
            // masses and the actual remaining staging sequence. No usable thrust is a
            // normal transient during staging (old engine gone, new one not yet
            // active): hold the last solution and wait rather than stopping. The cycle
            // clock is NOT advanced in that case — the next step retries immediately
            // rather than sitting out a whole cycle on a transient.
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
                // dt is the interval this solve covers, which is what makes the
                // convergence test rate-independent (see UpfgGuidance.Step).
                double solveDt = double.IsNegativeInfinity(_s.LastSolveTime) ? 0.0 : now - _s.LastSolveTime;
                _s.Upfg.Step(r, v, vehicle.TotalMass, mu, target, _s.UpfgVehicle, 1, solveDt);
                _s.LastSolveTime = now;
            }
        }

        UpdatePhase(r, bodyRadius, stepDt);
    }

    // Ascent phase state machine. Transitions cascade naturally over successive
    // frames, so initializing mid-flight fast-forwards to the right phase.
    // stepDt is the sim time since the previous step, for the command slew limit.
    private static void UpdatePhase(double3 r, double bodyRadius, double stepDt)
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
                    // tgo is measured from the SOLVE, not from now — the solution can
                    // be most of a guidance cycle old by the time this trips, and
                    // counting that cycle twice is a whole second of extra burn.
                    _s.CutoffTime = _s.LastSolveTime + _s.Upfg.Tgo;
                }
                break;
        }

        double3 want = _s.Phase switch
        {
            AscentPhase.Vertical => up,
            AscentPhase.Turn => TurnDir(up, turnPitch),
            AscentPhase.ClosedLoop => _s.Upfg.Steering,
            AscentPhase.Terminal => _s.FrozenDir,
            _ => default,
        };

        // The command SLEWS toward the solution rather than jumping to it. Every
        // discontinuity upstream — a guidance cycle landing, the hand-over from the
        // open-loop turn, a stage list that changed shape mid-burn — otherwise
        // arrives at the flight computer as an instantaneous new attitude target,
        // and that is what the vehicle jerks against.
        _s.CommandDir = SlewToward(_s.CommandDir, want, UpfgTarget.DegToRad(MaxSlewDegS) * stepDt);
        _s.HasCommand = _s.CommandDir.Length() > 0.5;
    }

    /// <summary>
    /// Rotate <paramref name="current"/> toward <paramref name="target"/> by at most
    /// maxRad, about the axis between them. An uninitialised or degenerate current
    /// direction snaps straight to the target — there is nothing to be continuous
    /// with on the first command of a launch.
    /// </summary>
    private static double3 SlewToward(double3 current, double3 target, double maxRad)
    {
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
        return RotateAbout(current, double3.Normalize(axis), maxRad);
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

    // The gravity-turn attitude: the given pitch above the horizon, along UPFG's own
    // launch azimuth once it has converged, with the classic inclination/latitude
    // formula as the fallback until then.
    //
    // TAKING THE AZIMUTH FROM THE SOLVER IS THE POINT, not a shortcut. The turn hands
    // over to that same steering vector, so flying its azimuth is what makes the
    // hand-over seamless — the pitch profiles are matched before the switch and the
    // heading already agrees. Deriving the azimuth from the target plane geometry
    // instead looks more principled and is not: UPFG's azimuth includes the yaw that
    // cancels the launch site's own eastward velocity, so a geometric one differs from
    // it by a few degrees, and the whole of that difference then arrives as a heading
    // change at the hand-over.
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

    /// <summary>
    /// The target orbit plane's normal in CCI — the roll reference the commanded
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
    /// position — not now, but <see cref="LanLeadSeconds"/> from now.
    ///
    /// Spherical trig: sin(lat) = sin(inc)·sin(u) on the orbit, so the node sits
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

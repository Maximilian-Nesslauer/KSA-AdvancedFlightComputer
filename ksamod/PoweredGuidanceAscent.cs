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
//   Terminal   — at tgo <= 10 s, freeze the attitude and stop iterating guidance
//                (re-solving on a near-zero arc just makes the solution chase
//                itself); count down to cutoff.
public static partial class PoweredGuidanceWindow
{
    // Target orbit inputs (altitudes in km, angles in degrees). Defaults are an ISS
    // launch: the 51.6° ISS plane, inserting at a 200 km perigee with apogee at ISS
    // altitude (~420 km) for the rendezvous transfer. LAN is seeded from the vessel's
    // current position on the first frame (the plane that passes over the pad right
    // now) and can be re-seeded with the button next to the input.
    private static double _peKm = 200.0;
    private static double _apKm = 420.0;
    private static double _incDeg = 51.6;
    private static double _lanDeg = 250;
    private static bool _lanSeeded;

    // Launch-to-target: pick another vehicle, derive its plane (inc/LAN) and a
    // co-elliptic chase orbit some km below it, wait (warping, if confirmed) until
    // the launch site rotates under the target's plane, and launch into it —
    // shuttle-to-ISS style. Ascending = north-easterly launch at the up-going plane
    // crossing; descending = south-easterly at the down-going one.
    private static string _targetId = "";
    private static double _chaseOffsetKm = 20.0;
    private static bool _launchDescending;
    private static bool _autoLaunch;

    // Gravity-turn shaping: at the turn-start altitude the commanded pitch ramps
    // down from vertical at a fixed rate toward the launch azimuth (open loop —
    // atmospheric physics is currently too jank to trust prograde-following).
    private static double _turnStartAltKm = 0.5;
    private static double _turnRateDegS = 1;
    private const double TerminalTgo = 10.0;
    // Hand over to UPFG no later than this altitude, even if the pitch profiles
    // never crossed — the failsafe against an open-loop runaway vehicle.
    private const double FailsafeAltKm = 50.0;

    private enum AscentPhase { Vertical, Turn, ClosedLoop, Terminal }
    private static AscentPhase _phase = AscentPhase.Vertical;
    private static double _turnStartTime;

    // Terminal-phase freeze: the steering at freeze time and the predicted cutoff.
    private static double3 _frozenDir;
    private static double _cutoffTime;

    // The Ascent tab body. Only the things an end user directly sets live here;
    // profile tuning is in the Adjust-params window.
    private static void DrawAscentTab(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                      double bodyRadius)
    {
        // Seed the LAN from where the vessel is right now, once a vehicle exists.
        if (!_lanSeeded)
        {
            _lanDeg = LanOverhead(orbit.StateVectors.PositionCci, _incDeg);
            _lanSeeded = true;
        }

        // --- Target inputs (collapsible to keep the panel compact) ---
        if (ImGui.CollapsingHeader("Target orbit"))
        {
            ImGui.InputDouble("Periapsis (km)", ref _peKm);
            ImGui.InputDouble("Apoapsis (km)", ref _apKm);
            ImGui.InputDouble("Inclination (deg)", ref _incDeg);
            ImGui.InputDouble("LAN (deg)", ref _lanDeg);
            ImGui.SameLine();
            if (ImGui.Button("From position"))
                _lanDeg = LanOverhead(orbit.StateVectors.PositionCci, _incDeg);
        }

        // --- Launch to target (runs its own launch-window logic, not collapsed) ---
        DrawLaunchToTarget(vehicle, orbit, parent, bodyRadius);

        // The toggles are pure configuration: nothing acts until EXECUTE
        // starts the process (or the armed auto-launch fires it at the
        // window). EXECUTE is the single commit point — guidance starts and
        // whatever is toggled goes live at once, so you can warp time
        // freely beforehand.
        ImGui.Checkbox("Engage autopilot", ref _engage);
        ImGui.SameLine();
        ImGui.Checkbox("Auto engines/staging", ref _autoStage);

        if (ImGui.Button("EXECUTE"))
            StartGuidance(orbit, parent);
        ImGui.SameLine();
        if (ImGui.Button("Stop / reset"))
        {
            _running = false;
            _autoLaunch = false;
        }
        ImGui.SameLine();
        if (ImGui.Button(_showAscentParams ? "Close params" : "Ascent params..."))
            _showAscentParams = !_showAscentParams;
    }

    // Ascent-profile tuning, in its own popup so it isn't mixed with the other
    // flows' parameters.
    private static bool _showAscentParams;

    private static void DrawAscentParamsWindow()
    {
        if (!_showAscentParams)
            return;

        ImGui.Begin("Ascent params", ImGuiWindowFlags.AlwaysAutoResize);
        ImGui.InputDouble("Turn start alt (km)", ref _turnStartAltKm);
        ImGui.InputDouble("Turn rate (deg/s)", ref _turnRateDegS);
        ImGui.Checkbox("G-limit", ref _gLimitEnabled);
        ImGui.SameLine();
        ImGui.InputDouble("Max accel (g)", ref _gLimitG);
        if (ImGui.Button("Close"))
            _showAscentParams = false;
        ImGui.End();
    }

    // Per-frame ascent stepping, run from Draw regardless of the visible tab.
    private static void StepAscent(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                   double mu, double bodyRadius)
    {
        // Ascent guidance does not run forever: shortly after terminal cutoff it
        // releases. Leaving it engaged kept CommandAttitude running on every sim
        // step (thousands/s under warp) and held the flight computer in active
        // attitude tracking — the lag that appeared with warp and lingered after.
        if (_running && _phase == AscentPhase.Terminal && SimNow() > _cutoffTime + 15.0)
        {
            _running = false;
            _status = "Ascent complete — guidance released.";
        }

        if (!_running)
            return;

        try
        {
            StepGuidance(vehicle, orbit, parent, mu, bodyRadius);
            if (_engage && _autoStage && !_cutoffDone)
                AutoSequence(vehicle);
            _failStreak = 0;
            _error = "";
        }
        catch (Exception e)
        {
            // Transient failures (staging frames, mid-mutation part trees) skip
            // the step and keep flying the last solution; only a sustained streak
            // means something is actually broken.
            _failStreak++;
            _error = e.Message;
            if (_failStreak > MaxFailStreak)
                _running = false;
        }
    }

    // The EXECUTE button's action — also fired automatically at the launch window.
    private static void StartGuidance(Orbit orbit, IParentBody parent)
    {
        _landingPhase = LandingPhase.Idle; // ascent takes over from any landing flow
        Guidance.Reset();
        _error = "";
        _status = "";
        _failStreak = 0;
        _running = true;
        _phase = AscentPhase.Vertical;
        _hasCommand = false;
        _cutoffDone = false;
        _stagingActive = false;
        _lastSequenceTime = double.NegativeInfinity;
    }

    // The launch-to-target panel: target picker, chase-orbit offset, node direction,
    // window countdown, and (when armed) a warp request plus the launch trigger.
    private static void DrawLaunchToTarget(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                           double bodyRadius)
    {
        ImGui.SeparatorText("Launch to target");

        Vehicle target = FindVehicleById(_targetId, vehicle);
        if (ImGui.BeginCombo("Target", _targetId.Length > 0 ? _targetId : "(none)"))
        {
            if (ImGui.Selectable("(none)", _targetId.Length == 0))
                _targetId = "";
            CelestialSystem system = Universe.CurrentSystem;
            if (system != null)
            {
                ReadOnlySpan<Astronomical> all = system.All.AsSpan();
                for (int i = 0; i < all.Length; i++)
                {
                    if (all[i] is Vehicle v && !ReferenceEquals(v, vehicle))
                    {
                        if (ImGui.Selectable(v.Id, v.Id == _targetId))
                        {
                            _targetId = v.Id;
                            // Mirror into the game's own targeting, so the map and
                            // rendezvous UI agree with us.
                            Universe.SetTarget(vehicle, v);
                        }
                    }
                }
            }
            ImGui.EndCombo();
        }

        ImGui.InputDouble("SMA offset below target (km)", ref _chaseOffsetKm);
        if (ImGui.RadioButton("Ascending (NE)", !_launchDescending))
            _launchDescending = false;
        ImGui.SameLine();
        if (ImGui.RadioButton("Descending (SE)", _launchDescending))
            _launchDescending = true;
        ImGui.Checkbox("Auto warp & launch", ref _autoLaunch);

        if (target == null)
        {
            if (_targetId.Length > 0)
                ImGui.TextColored(new float4(1f, 0.4f, 0.4f, 1f), "Target vehicle not found.");
            return;
        }

        Orbit targetOrbit = target.Orbit;
        if (!ReferenceEquals(targetOrbit.Parent, orbit.Parent))
        {
            ImGui.TextColored(new float4(1f, 0.4f, 0.4f, 1f), "Target orbits a different body.");
            return;
        }

        // Target plane straight from its state vectors: n = r × v. With our LAN
        // convention Normal = (sin i sin Ω, −sin i cos Ω, cos i), so Ω = atan2(nx, −ny).
        double3 rt = targetOrbit.StateVectors.PositionCci;
        double3 vt = targetOrbit.StateVectors.VelocityCci;
        double3 n = double3.Normalize(double3.Cross(rt, vt));
        double incT = Math.Acos(Math.Clamp(n.Z, -1.0, 1.0));
        double lanT = Wrap2Pi(Math.Atan2(n.X, -n.Y));
        double peAltKm = (targetOrbit.Periapsis - bodyRadius) / 1000.0;
        double apAltKm = (targetOrbit.Apoapsis - bodyRadius) / 1000.0;
        // Chase orbit: circular, with semi-major axis the chosen offset below the
        // target's. A true co-elliptic depends on launch phasing anyway — circular
        // is a clean baseline to correct from once up.
        double targetSmaKm = (targetOrbit.Periapsis + targetOrbit.Apoapsis) / 2000.0;
        double chaseAltKm = targetSmaKm - bodyRadius / 1000.0 - _chaseOffsetKm;
        double chasePe = chaseAltKm;
        double chaseAp = chaseAltKm;

        ImGui.Text($"Target orbit:  {peAltKm,7:F1} x {apAltKm,7:F1} km  inc {UpfgTarget.RadToDeg(incT),6:F2} deg");
        ImGui.Text($"Chase orbit:   {chaseAltKm,7:F1} km circular  (SMA {_chaseOffsetKm:F0} km below target)");

        // Launch window: how long until the body's rotation carries the launch site
        // under the target plane, at the requested (ascending/descending) crossing.
        double3 r = orbit.StateVectors.PositionCci;
        double lat = Math.Asin(Math.Clamp(r.Z / r.Length(), -1.0, 1.0));
        double ra = Math.Atan2(r.Y, r.X);
        double tanRatio = Math.Tan(lat) / Math.Tan(Math.Max(incT, 1e-6));
        bool reachable = Math.Abs(tanRatio) <= 1.0;
        if (!reachable)
        {
            ImGui.TextColored(new float4(1f, 0.6f, 0.3f, 1f),
                "Target inclination is below the site latitude — plane unreachable.");
            return;
        }

        double delta = Math.Asin(Math.Clamp(tanRatio, -1.0, 1.0));
        double raRequired = _launchDescending ? lanT + Math.PI - delta : lanT + delta;
        double omega = parent.GetAngularVelocity();
        double waitSec = omega > 1e-12 ? Wrap2Pi(raRequired - ra) / omega : double.NaN;
        ImGui.Text($"Launch window: T-{waitSec,7:F0} s ({(_launchDescending ? "descending" : "ascending")} crossing)");

        bool copyNow = ImGui.Button("Copy chase orbit to target inputs");
        if (copyNow || _autoLaunch)
        {
            _incDeg = UpfgTarget.RadToDeg(incT);
            _lanDeg = UpfgTarget.RadToDeg(lanT);
            _peKm = chasePe;
            _apKm = chaseAp;
            _lanSeeded = true;
        }

        // Armed: ask to warp to just before the window (the warp itself needs the
        // user's confirmation — see DrawWarpPrompt), then press EXECUTE for them.
        // The engage/auto toggles are respected as configured, not forced.
        if (_autoLaunch && !_running && !double.IsNaN(waitSec))
        {
            if (!_engage || !_autoStage)
                ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f),
                    "Note: engage/auto toggles are off — auto-launch will only start guidance.");

            if (waitSec <= 1.0)
            {
                if (Universe.IsAutoWarpActive)
                    Universe.AutoWarpStop(true);
                StartGuidance(orbit, parent);
                _autoLaunch = false;
            }
            else if (waitSec > WarpLeadTime + 5.0 && !Universe.IsAutoWarpActive)
            {
                RequestWarp(SimNow() + waitSec - WarpLeadTime, "the launch window");
            }

            ImGui.TextColored(new float4(0.5f, 0.9f, 1f, 1f), Universe.IsAutoWarpActive
                ? "Auto-warping to the launch window..."
                : "Armed — will EXECUTE at the window.");
        }
        else if (_autoLaunch && _running)
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

        // In the terminal phase the solution is frozen: re-running UPFG over a
        // near-zero remaining arc makes the steering chase itself and destabilizes
        // the attitude right before cutoff.
        if (_phase != AscentPhase.Terminal)
        {
            // Rebuild from the live part tree every step so UPFG always sees current
            // masses and the actual remaining staging sequence. No usable thrust is a
            // normal transient during staging (old engine gone, new one not yet
            // active): hold the last solution and wait rather than stopping.
            UpfgVehicle live = BuildUpfgVehicle(vehicle);
            if (live == null)
            {
                _status = "No thrust — holding last solution (staging/coast).";
            }
            else
            {
                if (_gLimitEnabled && _gLimitG > 0.1)
                    ApplyGLimit(live, _gLimitG);
                _status = "";
                _upfgVehicle = live;
                var target = UpfgTarget.FromOrbit(_peKm, _apKm, _incDeg, _lanDeg, bodyRadius, mu);
                Guidance.Step(r, v, vehicle.TotalMass, mu, target, _upfgVehicle);
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
        double upfgPitch = PitchOf(up, Guidance.Steering);
        double turnPitch = TurnPitchDeg();

        switch (_phase)
        {
            case AscentPhase.Vertical:
                if (alt >= _turnStartAltKm * 1000.0)
                {
                    _phase = AscentPhase.Turn;
                    _turnStartTime = SimNow();
                }
                break;

            case AscentPhase.Turn:
                // Pitch ramps down at the fixed rate; hand over to UPFG when it
                // meets the closed-loop solution — or at the failsafe altitude
                // regardless, so an open-loop profile can't run away.
                if ((Guidance.Converged && turnPitch <= upfgPitch)
                    || alt >= FailsafeAltKm * 1000.0)
                    _phase = AscentPhase.ClosedLoop;
                break;

            case AscentPhase.ClosedLoop:
                if (Guidance.Converged && Guidance.Tgo <= TerminalTgo)
                {
                    _phase = AscentPhase.Terminal;
                    _frozenDir = Guidance.Steering;
                    _cutoffTime = SimNow() + Guidance.Tgo;
                }
                break;
        }

        switch (_phase)
        {
            case AscentPhase.Vertical:
                _commandDir = up;
                break;
            case AscentPhase.Turn:
                _commandDir = TurnDir(up, turnPitch);
                break;
            case AscentPhase.ClosedLoop:
                _commandDir = Guidance.Steering;
                break;
            case AscentPhase.Terminal:
                _commandDir = _frozenDir;
                break;
        }
        _hasCommand = _commandDir.Length() > 0.5;
    }

    // The open-loop turn's commanded pitch: down from vertical at the fixed rate
    // since the turn started, never below the horizon.
    private static double TurnPitchDeg()
    {
        if (_phase != AscentPhase.Turn)
            return 90.0;
        double elapsed = SimNow() - _turnStartTime;
        return Math.Max(90.0 - _turnRateDegS * elapsed, 0.0);
    }

    // The gravity-turn attitude: the given pitch above the horizon, toward the
    // launch azimuth. Azimuth comes from UPFG's converged steering when available
    // (it knows the target plane), with the classic inclination/latitude formula
    // as fallback.
    private static double3 TurnDir(double3 up, double pitchDeg)
    {
        (double3 east, double3 north) = EnuBasis(up);

        double az;
        double3 steerHoriz = Guidance.Steering - double3.Dot(Guidance.Steering, up) * up;
        if (Guidance.Converged && steerHoriz.Length() > 1e-3)
        {
            az = Math.Atan2(double3.Dot(steerHoriz, east), double3.Dot(steerHoriz, north));
        }
        else
        {
            double lat = Math.Asin(Math.Clamp(up.Z, -1.0, 1.0));
            double inc = UpfgTarget.DegToRad(_incDeg);
            az = Math.Asin(Math.Clamp(Math.Cos(inc) / Math.Max(Math.Cos(lat), 1e-6), -1.0, 1.0));
            if (_launchDescending)
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

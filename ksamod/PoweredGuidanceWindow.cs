using System;
using System.Collections.Generic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using Gfold;
using HarmonyLib;
using KSA;
using PoweredGuidance.Upfg;

// The in-game UPFG panel. Reads the controlled vehicle's live inertial (CCI) state,
// runs the standalone UPFG guidance toward a target orbit, displays the solution, and
// (optionally) commands the flight computer through a full ascent profile:
//
//   Vertical   — straight up off the pad until the turn-start altitude
//   Turn       — open-loop gravity turn: pitch down from vertical at a fixed rate
//                (deg/s) toward the launch azimuth, until the commanded pitch
//                meets UPFG's commanded pitch — or until the 80 km failsafe
//   ClosedLoop — fly the converged UPFG steering
//   Terminal   — at tgo <= 10 s, freeze the attitude and stop iterating guidance
//                (re-solving on a near-zero arc just makes the solution chase
//                itself); count down to cutoff. Throttle remains manual.
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
    // co-elliptic chase orbit some km below it, wait (auto-warping if armed) until
    // the launch site rotates under the target's plane, and launch into it —
    // shuttle-to-ISS style. Ascending = north-easterly launch at the up-going plane
    // crossing; descending = south-easterly at the down-going one.
    private static string _targetId = "";
    private static double _chaseOffsetKm = 20.0;
    private static bool _launchDescending;
    private static bool _autoLaunch;
    private const double WarpLeadTime = 10.0;  // end auto-warp this many s early

    // Gravity-turn shaping: at the turn-start altitude the commanded pitch ramps
    // down from vertical at a fixed rate toward the launch azimuth (open loop —
    // atmospheric physics is currently too jank to trust prograde-following).
    private static double _turnStartAltKm = 1.0;
    private static double _turnRateDegS = 0.5;
    // Vehicle-wide acceleration limit: any part of any burn that would exceed this
    // becomes a constant-acceleration (Mode 2) segment in the UPFG stage list, and
    // the auto-throttle follows UPFG's throttle command to hold it.
    private static bool _gLimitEnabled;
    private static double _gLimitG = 4.0;
    private const double TerminalTgo = 10.0;
    // Hand over to UPFG no later than this altitude, even if the pitch profiles
    // never crossed — the failsafe against an open-loop runaway vehicle.
    private const double FailsafeAltKm = 50.0;

    private enum AscentPhase { Vertical, Turn, ClosedLoop, Terminal }
    private static AscentPhase _phase = AscentPhase.Vertical;
    private static double _turnStartTime;

    private static bool _running;
    private static bool _engage = true;      // toggles default on — the normal flow
    private static bool _wasEngaged;
    private static string _error = "";
    private static string _status = "";

    // Guidance must survive transient bad frames (the staging frame reports zero
    // thrust while the next engine ignites; the part tree can be mid-mutation). A
    // single failed step skips that frame and keeps the last solution; only a long
    // unbroken run of failures stops guidance for good.
    private static int _failStreak;
    private const int MaxFailStreak = 600; // ~10 s of consecutive bad frames

    private static readonly UpfgGuidance Guidance = new UpfgGuidance();

    // The staged vehicle model UPFG flies, rebuilt from KSA's sequence list every
    // guidance step (matching original navbox, where the sim feeds UPFG current
    // data each cycle): stage 0 always carries the present masses, so burn times
    // are inherently time-remaining, and manual staging shows up automatically.
    private static UpfgVehicle _upfgVehicle;

    // The attitude the current phase wants, produced by UpdatePhase each frame and
    // applied from the Vehicle.PrepareWorker prefix (see ApplyAutopilot).
    private static double3 _commandDir;
    private static bool _hasCommand;

    // Terminal-phase freeze: the steering at freeze time and the predicted cutoff.
    private static double3 _frozenDir;
    private static double _cutoffTime;

    // Auto engines & staging: while armed (and the autopilot is engaged), the mod
    // ignites the first sequence, fires the next sequence whenever the current
    // powered phase has no thrust or is about to flame out, and shuts the engines
    // down at the terminal cutoff. Throttle is forced to 1 while burning.
    private static bool _autoStage = true;   // defaults on, like _engage
    private static bool _cutoffDone;
    private static bool _stagingActive;
    private static double _lastSequenceTime = double.NegativeInfinity;
    private const double SequenceCooldown = 1.0;   // s between auto activations

    // The engine master switch and throttle live in Vehicle's private
    // _manualControlInputs; the game's own ignite/shutdown actions just set
    // EngineOn there, so we do exactly the same via a field ref.
    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> ManualInputs =
        AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

    // --- Landing (UPFG modes 2/3) ---
    // Flow: EXECUTE runs Mode 2 synchronously to convergence to measure how far
    // downrange the braking burn reaches, finds when the along-track distance to
    // the site shrinks to (factor × that distance), warps there, converges Mode 3
    // during a prep window, then burns with UPFG's throttle command driving the
    // cutoff to zero speed over the site.
    // Burn = UPFG braking to the high gate; Terminal = freeze onto the gate;
    // GfoldDescent = convex (G-FOLD) powered descent from the gate to the surface.
    private enum LandingPhase { Idle, Coast, Prep, Burn, GfoldDescent, VerticalDescent, Done }
    private static LandingPhase _landingPhase = LandingPhase.Idle;
    // Defaults: the Apollo 11 landmark as KSA itself defines it (Content/Core/
    // Astronomicals.xml, Landmark Id="Apollo11" on the Moon).
    private static double _siteLatDeg = 0.67408;
    private static double _siteLonDeg = 23.47297;
    private static double _downrangeFactor = 1.5;  // light the burn this × predicted distance out
    // The burn targets "high gate" — an aim point this far above the site, reached
    // descending at this rate — not the surface. A surface/zero-speed target is
    // geometrically infeasible: burn time is fixed by the energy (worse with a
    // powerful engine), and there is no time to also descend the full orbit
    // altitude. A final-descent mode below the gate is the follow-on.
    private static double _aimAltKm = 2.0;
    private static double _descentRate = 100.0;
    // The gate also sits this far uprange of the site (against the approach
    // direction), so the burn hands over to terminal guidance with the target
    // still ahead rather than directly underneath.
    private static double _gateUprangeKm = 2.0;
    private static double _burnDownrangeKm;        // predicted braking distance
    private static double _burnStartTime;          // sim time of ignition
    private static string _landingStatus = "";
    private const double PrepLeadTime = 30.0;      // converge + point before ignition

    // --- G-FOLD terminal descent (from the gate to the surface) ---
    private static double _gfoldGlideSlopeDeg = 30.0;
    private static double _gfoldPointingDeg = 80.0;
    private static double _gfoldVMaxMs = 120.0;
    private static double _gfoldIntervalS = 0.25;   // re-solve cadence
    private static int _gfoldNodes = 50;
    // Below this altitude G-FOLD hands to the vertical-descent phase: thrust
    // straight up at the constant deceleration that reaches zero velocity exactly
    // at the ground, for an upright touchdown instead of G-FOLD's tilted arrival.
    private static double _gfoldVertAltM = 0.0;
    // G-FOLD targets a point _gfoldVertAltM directly above the pad (Option B): it
    // nulls cross-range up high and delivers the vehicle over the target descending
    // vertically at this rate, so the vertical phase only has to kill the sink.
    private static double _gfoldArrivalRateMs = 0.0;
    // Solver thrust bounds, as a fraction of max thrust. These bound the planned
    // trajectory only — the tracker still uses the full 0-100% throttle range.
    private static double _gfoldThrottleMin = 0.05;
    private static double _gfoldThrottleMax = 0.90;
    // Thrust-slew smoothing weight: penalizes rapid changes in the commanded thrust
    // vector so the plan is gentler on the gimballed autopilot. 0 = off; a little goes
    // a long way (trades a little fuel for a much smoother command).
    private static double _gfoldSlewReg = 0.05;
    // Distance from the vehicle CoM/origin down to the landing legs. The G-FOLD
    // reference point is shifted by this much along the anti-pointing axis (see
    // GfoldRefPos) so the solve plans for the legs touching down, not the CoM.
    private static double _vehicleHeightM = 7.0;
    // Terminal vertical-descent phase (Option B): G-FOLD aims _gfoldVertAltM above the
    // pad descending, then a straight-down phase finishes the landing. Deactivated for
    // now (vehicle-rotation issues in that phase) — kept in code, just not entered.
    // With it off, G-FOLD flies the legs the whole way down to the surface.
    private static bool _gfoldUseVerticalPhase = false;
    // The G-FOLD terminal point, above the terrain. Vertical phase on: a point high up
    // (_gfoldVertAltM) reached descending, handed to the vertical phase. Off: the
    // surface itself (0) — the reference point is the legs (see GfoldRefPos), so they
    // touch the ground at rest with no separate vehicle-height offset on the target.
    private static double GfoldTargetAltM => _gfoldUseVerticalPhase ? _gfoldVertAltM : 0.0;
    private static double GfoldArrivalRateMs => _gfoldUseVerticalPhase ? _gfoldArrivalRateMs : 0.0;
    // Hand UPFG braking over to G-FOLD this many seconds before gate arrival,
    // skipping the UPFG terminal freeze (which oscillates as its solution decays
    // near the gate). G-FOLD plans from wherever we are to the surface.
    private static double _gfoldHandoffTgo = 20.0;
    private static double _gfoldThrottle;
    private static double _gfoldHandoffTime;
    private static double _gfoldLastSolveTime = double.NegativeInfinity;
    private static double _gfoldAltM, _gfoldSpeedMs;
    private static EcosStatus _gfoldStatus = EcosStatus.Optimal;
    private static int _gfoldFailStreak;
    // Set after a retarget so the next solve runs a fresh tf search to the new site
    // instead of the cheap re-solve (which would reuse the old, now-wrong arrival time).
    private static bool _gfoldForceSearch;
    private const double GfoldMinTf = 4.0;
    private const double GfoldCoastThrottle = 0.02;  // below this, cut the engine (true coast)
    

    // Committed-trajectory tracking: the solved descent plan is flown by time
    // index (feed-forward the planned thrust at the current time + light PD
    // feedback on the reference state) and re-solved on a cadence, rather than
    // applying node 0. This follows the plan's coast (throttle down) and brake
    // arcs instead of freezing the first node.
    private static GfoldTrajectory _gfoldPlan;
    private static double _gfoldPlanStart;         // sim time of plan node 0
    private static double _gfoldArrivalTime;       // sim time of planned touchdown
    private static double _gfoldThrustMax = 1.0;   // engine vac thrust at plan time, N
    private static double _gfoldKp = 0.08;         // position feedback gain
    private static double _gfoldKd = 0.30;         // velocity feedback gain
    // Command smoothing: the throttle and thrust direction are low-pass filtered
    // toward the freshly-computed command with this time constant, so per-frame
    // feedback noise and re-solve steps don't reach the engine/gimbal as chatter.
    private static double _gfoldSmoothTau = 0.15;
    private static double _gfoldLastTrackTime;
    private static bool _gfoldTrackInit;
    private static bool _gfoldEngineOn;            // hysteretic engine state
    // One-shot engine cut on reaching Done — after that the player has the vehicle
    // (the final descent below the gate is manual until the follow-on mode exists).
    private static bool _landingCutPending;

    // Upcoming site passes (time from now, closest ground distance). The scan is
    // ~1200 conic propagations, so it is time-sliced: a fixed sample budget per
    // frame, CSE warm-started between (sequential) samples, and the per-orbit
    // minimum sharpened by parabolic interpolation instead of a refinement pass.
    private static readonly List<(double tSec, double minKm)> Passes = new();
    private static double _passesComputedAt = double.NegativeInfinity;
    private const int PassesToShow = 5;
    private const int ScanSamplesPerOrbit = 240;
    private const int ScanSamplesPerFrame = 48;
    private static double3 _scanR0, _scanV0;
    private static double _scanStep;
    private static int _scanIndex = -1;            // -1: no scan in progress
    private static CseState _scanCser = CseState.Zero;
    private static readonly double[] _scanOrbitD = new double[ScanSamplesPerOrbit];
    private static readonly List<(double tSec, double minKm)> _scanResults = new();

    // Trajectory samples (downrange km, altitude km) at a fixed sim-time cadence,
    // downrange measured along the ground from the body-fixed launch point. When the
    // buffer fills we decimate 2:1 and double the cadence, so the plot always spans
    // the whole ascent at bounded memory.
    private static readonly List<float2> TrajSamples = new();
    private static double _sampleInterval = 1.0;
    private static double _nextSampleTime;
    private static double3 _launchDirCcf;
    private const int MaxTrajSamples = 600;

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

        // Seed the LAN from where the vessel is right now, once a vehicle exists.
        if (!_lanSeeded)
        {
            _lanDeg = LanOverhead(orbit.StateVectors.PositionCci, _incDeg);
            _lanSeeded = true;
        }

        _landingTabActive = false;   // set true below only while the Landing tab is open

        if (ImGui.BeginTabBar("##navtabs"))
        {
            if (ImGui.BeginTabItem("Ascent"))
            {
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

                if (ImGui.CollapsingHeader("Ascent profile"))
                {
                    ImGui.InputDouble("Turn start alt (km)", ref _turnStartAltKm);
                    ImGui.InputDouble("Turn rate (deg/s)", ref _turnRateDegS);
                    ImGui.Checkbox("G-limit", ref _gLimitEnabled);
                    ImGui.SameLine();
                    ImGui.InputDouble("Max accel (g)", ref _gLimitG);
                }

                // --- Launch to target (runs its own auto-warp logic, not collapsed) ---
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

        // --- Run landing logic (regardless of visible tab) ---
        StepLanding(vehicle, orbit, parent, mu, bodyRadius);

        // Ascent guidance does not run forever: shortly after terminal cutoff it
        // releases. Leaving it engaged kept CommandAttitude running on every sim
        // step (thousands/s under warp) and held the flight computer in active
        // attitude tracking — the lag that appeared with warp and lingered after.
        if (_running && _phase == AscentPhase.Terminal && SimNow() > _cutoffTime + 15.0)
        {
            _running = false;
            _status = "Ascent complete — guidance released.";
        }

        // --- Run guidance ---
        if (_running)
        {
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

            SampleTrajectory(orbit, parent, bodyRadius);
        }

        if (_error.Length > 0)
            ImGui.TextColored(new float4(1f, 0.4f, 0.4f, 1f), "Error: " + _error);
        if (_status.Length > 0)
            ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f), _status);

        // --- Solution readout ---
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
                UpfgStage s = _upfgVehicle.Stages[i];
                double burnTime = s.Mode == 2
                    ? s.Isp * 9.80665 * Math.Log(s.MassTotal / s.MassDry) / (s.GLim * 9.80665)
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
        ImGui.Text($"Inclination{UpfgTarget.RadToDeg(orbit.Inclination),8:F2}   {_incDeg,8:F2} deg");

        // --- Autopilot ---
        ImGui.SeparatorText("Autopilot");
        if (_engage && _running && _hasCommand)
        {
            // The actual flight-computer writes happen in ApplyAutopilot, which the
            // Bootstrap Harmony prefix calls just before the sim snapshots the FC
            // (Vehicle.PrepareWorker). Writing from here — the UI draw — lands in the
            // window where the sim's copy-back erases it.
            float errDeg = (float)(vehicle.FlightComputer.ErrorAngles.Length() * 180.0 / Math.PI);
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

        // --- Trajectory: downrange vs altitude ---
        ImGui.SeparatorText("Trajectory (downrange vs altitude)");
        DrawTrajectoryPlot();

        ImGui.End();

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

    // The mod's clock: elapsed sim time in seconds. Used for everything time-based
    // (turn ramp, staging cooldown, cutoff, samples) so behavior is correct under
    // time warp, unlike the wall-clock-ish player time.
    private static double SimNow() => Universe.GetElapsedSimTime().Seconds();

    // The Initialize button's action — also fired automatically at the launch window.
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
        TrajSamples.Clear();
        _sampleInterval = 1.0;
        _nextSampleTime = SimNow();
        _launchDirCcf = double3.Normalize(orbit.StateVectors.PositionCci)
            .Transform(parent.GetCci2Ccf());
    }

    // The launch-to-target panel: target picker, chase-orbit offset, node direction,
    // window countdown, and (when armed) automatic time warp plus launch trigger.
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

        // Armed: warp to just before the window, then press EXECUTE for the user.
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
                Universe.AutoWarpTo(Universe.GetElapsedSimTime() + (waitSec - WarpLeadTime));
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

    // ----- Landing -----

    private static void DrawLandingTab(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                       double mu, double bodyRadius)
    {
        // Parameters live in collapsible sections to keep the panel compact.
        if (ImGui.CollapsingHeader("Landing site"))
        {
            ImGui.InputDouble("Latitude (deg)", ref _siteLatDeg);
            ImGui.InputDouble("Longitude (deg)", ref _siteLonDeg);
            ImGui.InputDouble("Downrange factor", ref _downrangeFactor);
            ImGui.InputDouble("Aim altitude (km)", ref _aimAltKm);
            ImGui.InputDouble("Descent rate (m/s)", ref _descentRate);
            ImGui.InputDouble("Gate uprange (km)", ref _gateUprangeKm);
        }

        if (ImGui.CollapsingHeader("G-FOLD terminal descent"))
        {
            ImGui.InputDouble("Handoff at T-gate (s)", ref _gfoldHandoffTgo);
            ImGui.InputDouble("Glide slope (deg)", ref _gfoldGlideSlopeDeg);
            ImGui.InputDouble("Thrust pointing (deg)", ref _gfoldPointingDeg);
            ImGui.InputDouble("Max speed (m/s)", ref _gfoldVMaxMs);
            ImGui.InputDouble("Solver min thrust (frac)", ref _gfoldThrottleMin);
            ImGui.InputDouble("Solver max thrust (frac)", ref _gfoldThrottleMax);
            ImGui.InputDouble("Thrust smoothing (0=off)", ref _gfoldSlewReg);
            ImGui.InputDouble("Re-solve interval (s)", ref _gfoldIntervalS);
            ImGui.InputInt("Nodes", ref _gfoldNodes);
            ImGui.Checkbox("Vertical descent phase", ref _gfoldUseVerticalPhase);
            ImGui.InputDouble("Vertical descent alt (m)", ref _gfoldVertAltM);
            ImGui.InputDouble("Arrival sink rate (m/s)", ref _gfoldArrivalRateMs);
            ImGui.InputDouble("Vehicle height (m)", ref _vehicleHeightM);
            ImGui.InputDouble("Track gain Kp", ref _gfoldKp);
            ImGui.InputDouble("Track gain Kd", ref _gfoldKd);
            ImGui.InputDouble("Command smoothing (s)", ref _gfoldSmoothTau);
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
            (_landingPhase == LandingPhase.Idle || _landingPhase == LandingPhase.Done)
            && !Universe.IsAutoWarpActive;
        if (passesRefreshOk && _scanIndex < 0 && SimNow() - _passesComputedAt > 5.0)
            StartPassScan(orbit, mu);
        StepPassScan(parent, mu, bodyRadius);
        // Always exactly PassesToShow lines, so the layout (and the EXECUTE button
        // below) never jumps while a scan is in flight.
        for (int i = 0; i < PassesToShow; i++)
        {
            if (i < Passes.Count && Passes[i].minKm < 1e6)
                ImGui.Text($"Pass {i + 1}:  closest {Passes[i].minKm,8:F1} km   in {Passes[i].tSec,7:F0} s");
            else if (i < Passes.Count)
                ImGui.Text($"Pass {i + 1}:  (no solution)");
            else
                ImGui.Text($"Pass {i + 1}:  scanning...");
        }

        // --- Commit ---
        ImGui.SeparatorText("Deorbit");
        ImGui.Checkbox("Engage autopilot", ref _engage);
        ImGui.SameLine();
        ImGui.Checkbox("Auto engines/staging", ref _autoStage);
        ImGui.Checkbox("Show G-FOLD overlay (world)", ref _showGfoldOverlay);
        if (ImGui.Button(_retargetArmed ? "Click the surface...  (right-click cancels)" : "Retarget: click a spot"))
            _retargetArmed = !_retargetArmed;

        if (ImGui.Button("EXECUTE LANDING"))
            ExecuteLanding(vehicle, orbit, parent, mu, bodyRadius);
        ImGui.SameLine();
        if (ImGui.Button("Abort landing"))
        {
            _landingPhase = LandingPhase.Done;
            _landingCutPending = true;
            _landingStatus = "Aborted.";
        }

        // Skip straight to G-FOLD from the current state (or restart it after a
        // failure), engaging the autopilot + auto engines so it actually flies.
        if (ImGui.Button("Start G-FOLD now", new float2(360f, 40f)))
        {
            _engage = true;
            _autoStage = true;
            _running = false;
            _landingPhase = LandingPhase.GfoldDescent;
            _gfoldHandoffTime = SimNow();
            _gfoldLastSolveTime = double.NegativeInfinity;
            _gfoldPlan = null;
            _gfoldFailStreak = 0;
            _gfoldTrackInit = false;
            _gfoldEngineOn = false;
            _hasCommand = false;
            _landingStatus = "G-FOLD started from current state.";
        }

        if (_landingStatus.Length > 0)
            ImGui.TextColored(new float4(1f, 0.8f, 0.3f, 1f), _landingStatus);
        if (_landingPhase != LandingPhase.Idle)
        {
            double tIgn = _burnStartTime - SimNow();
            string phaseText = _landingPhase switch
            {
                LandingPhase.Coast => $"Coasting to burn point — ignition T-{tIgn,6:F0} s",
                LandingPhase.Prep => $"Converging guidance — ignition T-{tIgn,5:F1} s",
                LandingPhase.Burn => $"BURNING — cmd {Guidance.Throttle * 100,4:F0} % / engine {vehicle.GetManualThrottle() * 100,4:F0} %, tgo {Guidance.Tgo,6:F1} s",
                LandingPhase.GfoldDescent => $"G-FOLD [{_gfoldStatus}] alt {_gfoldAltM,6:F0} m, {_gfoldSpeedMs,5:F0} m/s, throttle {_gfoldThrottle * 100,3:F0} %, tf~{Math.Max(_gfoldArrivalTime - SimNow(), 0),4:F0} s",
                LandingPhase.VerticalDescent => $"VERTICAL DESCENT alt {_gfoldAltM,5:F0} m, {_gfoldSpeedMs,5:F1} m/s, throttle {_gfoldThrottle * 100,3:F0} %",
                LandingPhase.Done => "Landing guidance ended.",
                _ => "",
            };
            ImGui.TextColored(new float4(0.5f, 0.9f, 1f, 1f), phaseText);
            ImGui.Text($"Predicted burn downrange: {_burnDownrangeKm,7:F1} km  (start at {_downrangeFactor:F2}x)");
        }
    }

    // EXECUTE: measure the braking burn with a synchronous Mode-2 convergence, find
    // the moment our along-track distance to the site equals factor × that length,
    // and warp there. The actual Mode-3 burn starts via StepLanding.
    private static void ExecuteLanding(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                       double mu, double bodyRadius)
    {
        _landingStatus = "";
        UpfgVehicle model = BuildUpfgVehicle(vehicle);
        if (model == null)
        {
            _landingStatus = "No usable engine model on the vehicle.";
            return;
        }
        if (_gLimitEnabled && _gLimitG > 0.1)
            ApplyGLimit(model, _gLimitG);

        double3 r = orbit.StateVectors.PositionCci;
        double3 v = orbit.StateVectors.VelocityCci;

        // First pass: predict the braking-burn downrange from the current state and
        // solve the ignition time from it.
        double downrange = PredictBurnDownrange(r, v, vehicle.TotalMass, mu, model, parent, bodyRadius);
        if (double.IsNaN(downrange) || downrange <= 0)
        {
            _landingStatus = "Burn prediction failed to converge.";
            return;
        }
        double wait = FindBurnStartTime(orbit, parent, mu, bodyRadius, downrange * _downrangeFactor);
        if (double.IsNaN(wait))
        {
            _landingStatus = "No pass within 5 orbits gets inside the burn distance — adjust orbit.";
            return;
        }

        // Second pass: on an elliptical orbit the speed/altitude at the ignition
        // point differ from here, so re-predict at the propagated ignition state
        // and re-solve the window with the corrected distance.
        (double3 rIgn, double3 vIgn, _) = CseRoutine.Run(r, v, Math.Max(wait, 1e-3), mu, CseState.Zero);
        double refined = PredictBurnDownrange(rIgn, vIgn, vehicle.TotalMass, mu, model, parent, bodyRadius);
        if (!double.IsNaN(refined) && refined > 0)
        {
            double refinedWait = FindBurnStartTime(orbit, parent, mu, bodyRadius, refined * _downrangeFactor);
            if (!double.IsNaN(refinedWait))
            {
                downrange = refined;
                wait = refinedWait;
            }
        }
        _burnDownrangeKm = downrange / 1000.0;

        _burnStartTime = SimNow() + wait;
        _running = false;          // landing owns guidance and the autopilot now
        _autoLaunch = false;
        _cutoffDone = false;
        _stagingActive = false;
        _hasCommand = false;
        _landingPhase = LandingPhase.Coast;
        if (wait > PrepLeadTime + WarpLeadTime)
            Universe.AutoWarpTo(Universe.GetElapsedSimTime() + (wait - PrepLeadTime - WarpLeadTime));
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
        double gateRadius = bodyRadius + SiteTerrainHeight(parent) + _aimAltKm * 1000.0;
        var predict = new UpfgTarget
        {
            Radius = gateRadius,
            Velocity = _descentRate,
            Fpa = -Math.PI / 2.0,
            Normal = double3.Normalize(double3.Cross(r, v)),
            Rdes = SiteDirCciAt(parent, 0) * gateRadius,
        };
        Guidance.Reset();
        bool converged = false;
        for (int i = 0; i < 400; i++)
        {
            Guidance.Step(r, v, mass, mu, predict, model, 1);
            if (Guidance.Converged)
            {
                converged = true;
                break;
            }
        }
        double downrange = converged ? AngleBetween(r, Guidance.Rd) * bodyRadius : double.NaN;
        Guidance.Reset();
        return downrange;
    }

    // Per-frame landing state machine (runs whichever tab is visible).
    private static void StepLanding(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                    double mu, double bodyRadius)
    {
        if (_landingPhase == LandingPhase.Idle || _landingPhase == LandingPhase.Done)
            return;

        double now = SimNow();

        if (_landingPhase == LandingPhase.GfoldDescent)
        {
            StepGfoldDescent(vehicle, orbit, parent, bodyRadius, now);
            SampleTrajectory(orbit, parent, bodyRadius);
            return;
        }

        if (_landingPhase == LandingPhase.VerticalDescent)
        {
            StepVerticalDescent(vehicle, orbit, parent, bodyRadius, now);
            SampleTrajectory(orbit, parent, bodyRadius);
            return;
        }

        if (_landingPhase == LandingPhase.Coast)
        {
            if (now >= _burnStartTime - PrepLeadTime)
            {
                if (Universe.IsAutoWarpActive)
                    Universe.AutoWarpStop(true);
                Guidance.Reset();
                _landingPhase = LandingPhase.Prep;
                // Restart the trajectory plot for the descent.
                TrajSamples.Clear();
                _sampleInterval = 1.0;
                _nextSampleTime = now;
                _launchDirCcf = double3.Normalize(orbit.StateVectors.PositionCci)
                    .Transform(parent.GetCci2Ccf());
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
                if (_gLimitEnabled && _gLimitG > 0.1)
                    ApplyGLimit(live, _gLimitG);
                _upfgVehicle = live;

                double3 r = orbit.StateVectors.PositionCci;
                double3 v = orbit.StateVectors.VelocityCci;
                double3 planeNormal = double3.Normalize(double3.Cross(r, v));
                // The gate: above the site (terrain-referenced, not the mean
                // sphere) and uprange of it along the approach (rotating a
                // position about +h moves it downrange, so uprange is the
                // negative rotation).
                double gateRadius = bodyRadius + SiteTerrainHeight(parent) + _aimAltKm * 1000.0;
                double3 gateDir = RotateAbout(SiteDirCciAt(parent, 0), planeNormal,
                    -_gateUprangeKm * 1000.0 / bodyRadius);
                var target = new UpfgTarget
                {
                    Radius = gateRadius,
                    Velocity = 0,                 // no forward speed at the gate
                    DescentRate = _descentRate,   // arrive sinking, not stopped
                    Fpa = 0,
                    Normal = planeNormal,
                    Rdes = gateDir * gateRadius,
                };
                Guidance.Step(r, v, vehicle.TotalMass, mu, target, live, 3);
                _commandDir = Guidance.Steering;
                _hasCommand = _commandDir.Length() > 0.5;
            }
            _failStreak = 0;
            _error = "";
        }
        catch (Exception e)
        {
            _failStreak++;
            _error = e.Message;
            if (_failStreak > MaxFailStreak)
            {
                _landingPhase = LandingPhase.Done;
                _landingCutPending = true;
                _landingStatus = "Guidance failed repeatedly — landing stopped.";
            }
        }

        if (_landingPhase == LandingPhase.Prep && now >= _burnStartTime)
            _landingPhase = LandingPhase.Burn;

        if (_landingPhase == LandingPhase.Burn)
        {
            // Staging support during the descent burn (ignites the deorbit engine
            // too if its sequence was never fired).
            if (_engage && _autoStage)
                AutoSequence(vehicle);

            // Hand straight to G-FOLD a set time before gate arrival, skipping the
            // UPFG terminal freeze. G-FOLD plans from the current state down.
            if (Guidance.Converged && Guidance.Tgo <= _gfoldHandoffTgo)
            {
                _landingPhase = LandingPhase.GfoldDescent;
                _gfoldHandoffTime = now;
                _gfoldLastSolveTime = double.NegativeInfinity;
                _gfoldPlan = null;
                _gfoldFailStreak = 0;
                _gfoldTrackInit = false;
                _gfoldEngineOn = false;
                _landingStatus = "Handoff to G-FOLD descent.";
            }
        }

        SampleTrajectory(orbit, parent, bodyRadius);
    }

    // Convex (G-FOLD) powered descent from the high gate to the surface. Re-solves
    // on a fixed cadence and applies the first node's commanded thrust direction +
    // throttle directly (a simple receding-horizon scheme): build the site frame
    // and parameters from the live vehicle, solve to the origin, map u[0] back to
    // a CCI command. The solve runs inline here — a one-off ~0.5 s search at
    // handoff and ~10-20 ms per re-solve; moving it to a background thread is the
    // obvious next step if the per-solve hitch is noticeable.
    private static void StepGfoldDescent(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                         double bodyRadius, double now)
    {
        if (_engage && _autoStage)
            AutoSequence(vehicle);

        double3 siteCci = SiteDirCciAt(parent, 0) * (bodyRadius + SiteTerrainHeight(parent));
        var frame = KsaGfold.BuildFrame(siteCci);
        double3 v = orbit.StateVectors.VelocityCci;

        // Reference the base of the vehicle (landing legs), not the CoM: shift the
        // position _vehicleHeightM down the anti-pointing axis. The whole descent then
        // reads this shifted position, so G-FOLD targets where the legs touch down.
        double3 r = GfoldRefPos(orbit);
        double3 vSrf = v - double3.Cross(parent.GetAngularVelocityCci(), r);
        _gfoldAltM = double3.Dot(r - siteCci, frame.Ex);
        _gfoldSpeedMs = vSrf.Length();

        // Reached the vertical-descent altitude: hand to the vertical phase, which
        // brings the vehicle straight down to a zero-velocity, upright touchdown.
        if (_gfoldUseVerticalPhase && _gfoldAltM <= _gfoldVertAltM)
        {
            _landingPhase = LandingPhase.VerticalDescent;
            _landingStatus = "Vertical descent.";
            return;
        }
        // Vertical phase off: G-FOLD flies the whole way down. The reference point is
        // the legs, so touchdown is simply the legs reaching the surface (or the
        // descent stopping right above it).
        if (!_gfoldUseVerticalPhase)
        {
            double vUp = double3.Dot(vSrf, frame.Ex);            // + = up
            bool landed = _gfoldAltM <= 0.0 || (vUp >= 0.0 && _gfoldAltM <= 0.5);
            // Failsafe: if it's near the surface but climbing (failed to touch down and
            // bailing back up), cut rather than let it fly away.
            bool bailingUp = vUp > 0.0 && _gfoldAltM <= 50.0;
            if (landed || bailingUp)
            {
                _gfoldThrottle = 0.0;
                _landingPhase = LandingPhase.Done;
                _landingCutPending = true;
                _landingStatus = landed
                    ? $"G-FOLD touchdown ({_gfoldSpeedMs:F1} m/s)."
                    : $"G-FOLD cut — climbing at {_gfoldAltM:F0} m, engines off.";
                return;
            }
        }

        // Inside the last GfoldMinTf seconds before the planned arrival, the distance
        // still to fly is too small for any valid flight time: tf >= TfMin (the search
        // floor) overshoots it, so a re-solve goes degenerate and reports the target
        // unreachable right before the handoff. Freeze the committed plan there — it
        // already terminates at the target — and just fly it down.
        bool terminalWindow = _gfoldPlan != null && _gfoldArrivalTime - now <= GfoldMinTf;
        if (!terminalWindow &&
            (_gfoldPlan == null || now - _gfoldLastSolveTime >= _gfoldIntervalS))
            SolveGfoldPlan(vehicle, parent, frame, siteCci, r, now);

        // Fly the committed plan by time index every frame (feed-forward + PD). Track
        // in the LIVE site frame (rebuilt this step), not the solve-time frame: the
        // site is body-fixed, so its CCI position rotates with the body, and the live
        // frame carries the plan around with it so we keep aiming at the real pad.
        if (_gfoldPlan != null)
            TrackGfoldPlan(frame, r, vSrf, vehicle.TotalMass, now);
    }

    // The G-FOLD reference point: the base of the vehicle (landing legs), found by
    // shifting the CoM _vehicleHeightM along the anti-pointing (anti-thrust) axis. The
    // vehicle holds its thrust axis along _commandDir, so the legs are at
    // r - _vehicleHeightM * _commandDir; before a command exists, fall back to
    // straight down (radial) — the legs directly below the CoM.
    private static double3 GfoldRefPos(Orbit orbit)
    {
        double3 r = orbit.StateVectors.PositionCci;
        double3 pointing = _commandDir.Length() > 0.5
            ? double3.Normalize(_commandDir)
            : double3.Normalize(r);
        return r - _vehicleHeightM * pointing;
    }

    // Final vertical descent: thrust straight up at the constant deceleration that
    // brings the (downward) vertical velocity to zero exactly at the ground —
    // a = v_down^2 / (2h) from v^2 = v0^2 - 2*a*d — plus gravity to hold against the
    // fall, so the vehicle lands upright at ~zero velocity instead of at G-FOLD's
    // tilt. Constant mass over this short phase; recomputed each frame (closed loop:
    // on the ideal profile a is constant, off it the throttle self-corrects to still
    // reach zero at the ground). Engines cut the instant the descent stops.
    private static void StepVerticalDescent(Vehicle vehicle, Orbit orbit, IParentBody parent,
                                            double bodyRadius, double now)
    {
        double3 siteCci = SiteDirCciAt(parent, 0) * (bodyRadius + SiteTerrainHeight(parent));
        var frame = KsaGfold.BuildFrame(siteCci);
        double3 r = GfoldRefPos(orbit);   // base of the vehicle (legs)
        double3 v = orbit.StateVectors.VelocityCci;
        double3 vSrf = v - double3.Cross(parent.GetAngularVelocityCci(), r);

        double h = double3.Dot(r - siteCci, frame.Ex);  // legs above ground
        double vUp = double3.Dot(vSrf, frame.Ex);          // vertical speed, + = up
        _gfoldAltM = h;
        _gfoldSpeedMs = vSrf.Length();

        // Thrust straight up (local vertical) so the vehicle is upright.
        _commandDir = double3.Normalize(r);
        _hasCommand = true;

        // Velocity reached zero / turned upward, or we're on the ground: cut engines.
        if (vUp >= 0.0 || h <= 0.5)
        {
            _gfoldThrottle = 0.0;
            _landingPhase = LandingPhase.Done;
            _landingCutPending = true;
            _landingStatus = $"Vertical descent complete — touchdown ({_gfoldSpeedMs:F1} m/s).";
            return;
        }

        double vDown = -vUp;                               // > 0, descending
        double aDecel = vDown * vDown / (2.0 * Math.Max(h, 0.1));
        double g = parent.Mu / (r.Length() * r.Length());
        double thrustMax = vehicle.FlightComputer.VehicleConfig.TotalEngineVacuumThrust;
        _gfoldThrottle = thrustMax > 0
            ? Math.Clamp((g + aDecel) * vehicle.TotalMass / thrustMax, 0.0, 1.0)
            : 0.0;
    }

    // Solve a fresh descent plan from the current state and commit it. A min-fuel
    // search (so it coasts/brakes optimally — "throttles down") to the site; if the
    // site is unreachable in the remaining time the search floats the touchdown to
    // the closest point, so this degrades gracefully instead of going infeasible.
    private static void SolveGfoldPlan(Vehicle vehicle, IParentBody parent,
                                       KsaGfold.Frame frame, double3 siteCci, double3 refPos, double now)
    {
        GfoldParams p = KsaGfold.BuildParams(
            vehicle, parent, frame, siteCci, refPos, _gfoldGlideSlopeDeg, _gfoldPointingDeg, _gfoldVMaxMs,
            GfoldTargetAltM, GfoldArrivalRateMs, _gfoldThrottleMin, _gfoldThrottleMax);
        if (p == null)
        {
            _landingStatus = "G-FOLD: no engine — holding.";
            return;
        }

        // Mark the attempt now (not only on success) so a failing solve retries on the
        // normal cadence rather than every frame while we keep flying the last plan.
        _gfoldLastSolveTime = now;

        try
        {
            // On the cadence, prefer to keep the arrival time set at handoff (solve
            // the remaining time); only re-run the full search when that fails or
            // we have no plan yet.
            GfoldTrajectory traj = null;
            if (!_gfoldForceSearch && _gfoldPlan != null && _gfoldArrivalTime - now > GfoldMinTf)
            {
                double remaining = _gfoldArrivalTime - now;
                GfoldTrajectory t = GfoldPlanner.SolveMinFuel(
                    p, remaining, _gfoldNodes, [GfoldTargetAltM, 0.0, 0.0],
                    options: GfoldOptions.Descent with { SlewReg = _gfoldSlewReg });
                if (t.Status is EcosStatus.Optimal or EcosStatus.OptimalInaccurate)
                    traj = t;
            }
            if (traj == null)
            {
                GfoldPlanner.SearchResult best = GfoldPlanner.SearchMinFuel(
                    p, _gfoldNodes, tfLo: GfoldMinTf, tfHi: 120.0,
                    options: GfoldOptions.Descent with { SlewReg = _gfoldSlewReg });
                if (best == null)
                {
                    FailGfold($"G-FOLD unreachable: alt {_gfoldAltM:F0} m, {_gfoldSpeedMs:F0} m/s, " +
                              $"TWR {p.ThrustMax / (vehicle.TotalMass * p.GravityMag):F1}, fuel {p.FuelMass:F0} kg");
                    return;
                }
                traj = best.Trajectory;
                _gfoldArrivalTime = now + best.TimeOfFlight;
            }

            _gfoldStatus = traj.Status;
            _gfoldPlan = traj;
            _gfoldPlanStart = now;
            _gfoldThrustMax = p.ThrustMax;
            _gfoldFailStreak = 0;
            _gfoldForceSearch = false;
            _landingStatus = "";
        }
        catch (Exception e)
        {
            FailGfold("G-FOLD: " + e.Message);
        }
    }

    // Fly the committed plan: feed-forward the planned thrust at the current time
    // plus PD feedback on the planned state, expressed in the given (live) site
    // frame so the plan stays locked to the body-fixed, rotating landing pad.
    private static void TrackGfoldPlan(KsaGfold.Frame f, double3 r, double3 vSrf, double mass, double now)
    {
        GfoldTrajectory plan = _gfoldPlan;
        int n = plan.Nodes;
        double elapsed = now - _gfoldPlanStart;

        // Reference STATE at the current time: interpolate from node 0. The plan
        // starts at the current state, so tracking error is ~0 just after a solve;
        // reading it one node ahead (as the feed-forward does) would feed a whole
        // step of expected motion ~v*dt back as phantom error and tilt the command
        // past the pointing cone.
        double sf = Math.Clamp(elapsed / plan.Dt, 0.0, n - 1);
        int s0 = Math.Clamp((int)Math.Floor(sf), 0, n - 2);
        double sfrac = Math.Clamp(sf - s0, 0.0, 1.0);
        double3 refPos = Lerp(Node(plan.Position, s0), Node(plan.Position, s0 + 1), sfrac);
        double3 refVel = Lerp(Node(plan.Velocity, s0), Node(plan.Velocity, s0 + 1), sfrac);

        // Feed-forward THRUST: skip node 0 (its control is the unconstrained
        // artifact), so the first step uses node 1's real, cone-respecting thrust.
        double tf = Math.Max(elapsed / plan.Dt, 1.0);
        int t0 = Math.Clamp((int)Math.Floor(tf), 1, n - 2);
        double tfrac = Math.Clamp(tf - t0, 0.0, 1.0);
        double3 ff = Lerp(Node(plan.AccelCmd, t0), Node(plan.AccelCmd, t0 + 1), tfrac);

        double3 curPos = f.PointToLocal(r);
        double3 curVel = f.VecToLocal(vSrf);
        double3 fb = _gfoldKp * (refPos - curPos) + _gfoldKd * (refVel - curVel);
        double3 cmd = ff + fb; // local thrust acceleration

        double targetThrottle = Math.Clamp(cmd.Length() * mass / Math.Max(_gfoldThrustMax, 1.0), 0.0, 1.0);

        // Direction: clamp the command to within the pointing cone of local up. The
        // plan respects the pointing limit, but the PD feedback can tilt past it, so
        // the limit must be re-applied here or it isn't enforced on the vehicle.
        double3 dirLocal = ClampToCone(cmd, _gfoldPointingDeg);
        double3 targetDir = double3.Normalize(f.VecToCci(dirLocal));
        if (!double.IsFinite(targetDir.X) || !double.IsFinite(targetDir.Y) || !double.IsFinite(targetDir.Z))
            targetDir = _commandDir.Length() > 0.5 ? _commandDir : f.Ex;
        

        // First-order low-pass toward the fresh command, so feedback noise and
        // re-solve steps don't reach the engine/gimbal as chatter.
        double dt = Math.Clamp(now - _gfoldLastTrackTime, 0.0, 0.25);
        _gfoldLastTrackTime = now;
        double a = (!_gfoldTrackInit || _gfoldSmoothTau <= 1e-3)
            ? 1.0
            : 1.0 - Math.Exp(-dt / _gfoldSmoothTau);
        _gfoldTrackInit = true;

        _gfoldThrottle += a * (targetThrottle - _gfoldThrottle);
        double3 blended = _commandDir.Length() > 0.5 ? _commandDir + a * (targetDir - _commandDir) : targetDir;
        if (blended.Length() > 1e-6)
            _commandDir = double3.Normalize(blended);
        _hasCommand = true;
    }

    private static double3 Node(double[][] a, int i) => new double3(a[i][0], a[i][1], a[i][2]);
    private static double3 Lerp(double3 a, double3 b, double t) => a + (b - a) * t;

    // Unit thrust direction of v, clamped to within maxAngleDeg of local up (+X).
    // If v tilts past the cone, it's pushed onto the cone surface keeping azimuth.
    private static double3 ClampToCone(double3 v, double maxAngleDeg)
    {
        double len = v.Length();
        if (len < 1e-9) return new double3(1, 0, 0);
        double3 d = v * (1.0 / len);
        double cosMax = Math.Cos(Math.Clamp(maxAngleDeg, 0.0, 90.0) * Math.PI / 180.0);
        if (d.X >= cosMax) return d;                       // already within the cone
        double sinMax = Math.Sqrt(Math.Max(1.0 - cosMax * cosMax, 0.0));
        var horiz = new double3(0, d.Y, d.Z);
        double h = horiz.Length();
        double3 hUnit = h > 1e-9 ? horiz * (1.0 / h) : new double3(0, 1, 0);
        return new double3(cosMax, sinMax * hUnit.Y, sinMax * hUnit.Z);
    }

    // A failed solve holds the last command briefly; a short run of failures gives
    // the vehicle back rather than flying a stale (often sideways) command in.
    private static void FailGfold(string message)
    {
        _gfoldFailStreak++;
        // A failed re-solve is not fatal once we hold a feasible plan: keep flying the
        // last committed trajectory (the solver usually only chokes on the degenerate
        // last few metres, where the existing plan lands fine) and just tell the user.
        // Only give up when there's nothing to fly — no plan was ever found.
        if (_gfoldPlan != null)
        {
            _landingStatus = $"G-FOLD re-solve failed ({_gfoldFailStreak}) — flying last trajectory. {message}";
            return;
        }
        _landingStatus = message;
        if (_gfoldFailStreak > 3)
        {
            _landingPhase = LandingPhase.Done;
            _landingCutPending = true;
            _landingStatus = "G-FOLD found no trajectory — vehicle is yours.";
        }
    }

    // The site's body-fixed (CCF) direction. KSA's own convention: lat = asin(z),
    // lon = atan2(y,x) in CCF.
    private static double3 SiteDirCcf()
    {
        double lat = UpfgTarget.DegToRad(_siteLatDeg);
        double lon = UpfgTarget.DegToRad(_siteLonDeg);
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
    private static double _siteTerrainCacheLat = double.NaN;
    private static double _siteTerrainCacheLon = double.NaN;
    private static object _siteTerrainCacheBody;
    private static double _siteTerrainHeight;

    private static double SiteTerrainHeight(IParentBody parent)
    {
        if (_siteLatDeg != _siteTerrainCacheLat || _siteLonDeg != _siteTerrainCacheLon
            || !ReferenceEquals(parent, _siteTerrainCacheBody))
        {
            _siteTerrainHeight = (parent as Celestial)?.GetTerrainHeightFromDirCcf(SiteDirCcf()) ?? 0.0;
            if (!double.IsFinite(_siteTerrainHeight))
                _siteTerrainHeight = 0.0;
            _siteTerrainCacheLat = _siteLatDeg;
            _siteTerrainCacheLon = _siteLonDeg;
            _siteTerrainCacheBody = parent;
        }
        return _siteTerrainHeight;
    }

    private static double3 RotZ(double3 v, double angle)
    {
        double c = Math.Cos(angle), s = Math.Sin(angle);
        return new double3(v.X * c - v.Y * s, v.X * s + v.Y * c, v.Z);
    }

    // Rodrigues rotation of vec about a unit axis.
    private static double3 RotateAbout(double3 vec, double3 axis, double angle)
    {
        double c = Math.Cos(angle), s = Math.Sin(angle);
        return vec * c + double3.Cross(axis, vec) * s + axis * (double3.Dot(axis, vec) * (1.0 - c));
    }

    private static double AngleBetween(double3 a, double3 b)
    {
        double d = double3.Dot(double3.Normalize(a), double3.Normalize(b));
        return Math.Acos(Math.Clamp(d, -1.0, 1.0));
    }

    // Begin an incremental closest-approach scan over the next PassesToShow orbits.
    private static void StartPassScan(Orbit orbit, double mu)
    {
        double sma = (orbit.Periapsis + orbit.Apoapsis) / 2.0;
        if (sma <= 0 || double.IsNaN(sma))
            return;
        double period = 2.0 * Math.PI * Math.Sqrt(sma * sma * sma / mu);
        _scanR0 = orbit.StateVectors.PositionCci;
        _scanV0 = orbit.StateVectors.VelocityCci;
        _scanStep = period / ScanSamplesPerOrbit;
        _scanCser = CseState.Zero;
        _scanResults.Clear();
        _scanIndex = 0;
    }

    // Advance the scan by at most ScanSamplesPerFrame samples. Sequential sample
    // times keep the CSE warm-started; each completed orbit's minimum is sharpened
    // with a parabolic fit through its neighbours before being committed.
    private static void StepPassScan(IParentBody parent, double mu, double bodyRadius)
    {
        // Pause (not abort) during warp: frames are already expensive there.
        if (_scanIndex < 0 || Universe.IsAutoWarpActive)
            return;

        double period = _scanStep * ScanSamplesPerOrbit;
        int total = ScanSamplesPerOrbit * PassesToShow;
        int end = Math.Min(_scanIndex + ScanSamplesPerFrame, total);
        for (; _scanIndex < end; _scanIndex++)
        {
            double t = (_scanIndex + 1) * _scanStep;
            // Conic position is periodic, so propagate by t mod period — the CSE
            // port dropped the original's multi-revolution counter (ascent never
            // needs it), and multi-rev inputs are where it slowed down and went
            // NaN. Only the site rotation needs the full t.
            double tProp = t % period;
            if (_scanIndex % ScanSamplesPerOrbit == 0)
                _scanCser = CseState.Zero; // warm-start doesn't survive the wrap

            double d;
            if (tProp < 1e-3)
            {
                d = AngleBetween(_scanR0, SiteDirCciAt(parent, t)) * bodyRadius;
            }
            else
            {
                double3 rr;
                (rr, _, _scanCser) = CseRoutine.Run(_scanR0, _scanV0, tProp, mu, _scanCser);
                d = AngleBetween(rr, SiteDirCciAt(parent, t)) * bodyRadius;
            }
            if (!double.IsFinite(d))
            {
                d = 1e12; // poisoned sample: ignore it and restart the warm chain
                _scanCser = CseState.Zero;
            }
            _scanOrbitD[_scanIndex % ScanSamplesPerOrbit] = d;

            if ((_scanIndex + 1) % ScanSamplesPerOrbit == 0)
                CommitScanOrbit(_scanIndex + 1 - ScanSamplesPerOrbit);
        }

        if (_scanIndex >= total)
        {
            Passes.Clear();
            Passes.AddRange(_scanResults);
            _scanIndex = -1;
            _passesComputedAt = SimNow();
        }
    }

    private static void CommitScanOrbit(int orbitStartIndex)
    {
        int jMin = 0;
        for (int j = 1; j < ScanSamplesPerOrbit; j++)
            if (_scanOrbitD[j] < _scanOrbitD[jMin])
                jMin = j;

        double tBest = (orbitStartIndex + jMin + 1) * _scanStep;
        double dBest = _scanOrbitD[jMin];
        if (jMin > 0 && jMin < ScanSamplesPerOrbit - 1)
        {
            double d0 = _scanOrbitD[jMin - 1], d1 = _scanOrbitD[jMin], d2 = _scanOrbitD[jMin + 1];
            double denom = d0 - 2.0 * d1 + d2;
            if (Math.Abs(denom) > 1e-9)
            {
                double frac = Math.Clamp(0.5 * (d0 - d2) / denom, -1.0, 1.0);
                tBest += frac * _scanStep;
                dBest = d1 - 0.25 * (d0 - d2) * frac;
            }
        }
        _scanResults.Add((tBest, dBest / 1000.0));
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

    private static Vehicle FindVehicleById(string id, Vehicle exclude)
    {
        if (id.Length == 0)
            return null;
        CelestialSystem system = Universe.CurrentSystem;
        if (system == null)
            return null;
        ReadOnlySpan<Astronomical> all = system.All.AsSpan();
        for (int i = 0; i < all.Length; i++)
            if (all[i] is Vehicle v && !ReferenceEquals(v, exclude) && v.Id == id)
                return v;
        return null;
    }

    private static double Wrap2Pi(double a)
    {
        a %= 2.0 * Math.PI;
        return a < 0 ? a + 2.0 * Math.PI : a;
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

    // Vehicle-wide acceleration limit, applied to the freshly built stage list each
    // step (same split the original navbox did per stage): a stage that would cross
    // the limit mid-burn is divided at the mass where full thrust hits the limit —
    // constant thrust before it, constant acceleration (Mode 2) after.
    private static void ApplyGLimit(UpfgVehicle vehicle, double gLim)
    {
        const double g0 = 9.80665;
        double aLim = gLim * g0;
        var limited = new List<UpfgStage>();
        foreach (UpfgStage s in vehicle.Stages)
        {
            if (s.Thrust / s.MassDry <= aLim)
            {
                limited.Add(s); // never reaches the limit
            }
            else if (s.Thrust / s.MassTotal >= aLim)
            {
                s.Mode = 2;     // already at/above the limit: all constant-accel
                s.GLim = gLim;
                limited.Add(s);
            }
            else
            {
                double massAtLimit = s.Thrust / aLim;
                limited.Add(new UpfgStage
                {
                    Mode = 1, Thrust = s.Thrust, Isp = s.Isp, GLim = gLim,
                    MassTotal = s.MassTotal, MassDry = massAtLimit,
                });
                limited.Add(new UpfgStage
                {
                    Mode = 2, Thrust = s.Thrust, Isp = s.Isp, GLim = gLim,
                    MassTotal = massAtLimit, MassDry = s.MassDry,
                });
            }
        }
        vehicle.Stages.Clear();
        vehicle.Stages.AddRange(limited);
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

    // Staging needs an unknown number of sequence activations (decouple, then
    // ignite, sometimes another press before that) — so whenever the vehicle has no
    // engine actually producing thrust (lit AND fed with propellant, per the game's
    // own live engine state), keep firing the next sequence every SequenceCooldown
    // seconds until one is. This single rule covers pad ignition, burnout staging,
    // and decouple-only sequences. Same call pair the game's staging key uses, from
    // the same (main) thread.
    private static void AutoSequence(Vehicle vehicle)
    {
        bool thrustOn = vehicle.IsAnyEngineActive() && vehicle.IsAnyEnginePropellantAvailable();
        if (thrustOn)
        {
            _stagingActive = false;
            return;
        }

        SequenceList sequenceList = vehicle.Parts?.SequenceList;
        if (sequenceList == null)
            return;
        bool anyRemaining = false;
        ReadOnlySpan<Sequence> sequences = sequenceList.Sequences;
        for (int i = 0; i < sequences.Length; i++)
            if (!sequences[i].Activated)
                anyRemaining = true;
        if (!anyRemaining)
        {
            _stagingActive = false;
            return;
        }

        _stagingActive = true;
        double now = SimNow();
        if (now - _lastSequenceTime >= SequenceCooldown)
        {
            sequenceList.ActivateNextSequence(vehicle);
            vehicle.UpdateAfterPartTreeModification();
            _lastSequenceTime = now;
        }
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

    // Local east/north horizon basis at a CCI position (Z = polar axis).
    private static (double3 east, double3 north) EnuBasis(double3 up)
    {
        double3 east = double3.Cross(new double3(0, 0, 1), up);
        east = east.Length() > 1e-6 ? double3.Normalize(east) : new double3(1, 0, 0);
        double3 north = double3.Cross(up, east);
        return (east, north);
    }

    // Elevation of a direction above the local horizon, in degrees.
    private static double PitchOf(double3 up, double3 dir)
    {
        double len = dir.Length();
        if (len < 1e-9)
            return 90.0;
        double c = Math.Clamp(double3.Dot(up, dir) / len, -1.0, 1.0);
        return 90.0 - UpfgTarget.RadToDeg(Math.Acos(c));
    }

    private static string PhaseName(AscentPhase p) => p switch
    {
        AscentPhase.Vertical => "vertical rise",
        AscentPhase.Turn => "gravity turn (open loop)",
        AscentPhase.ClosedLoop => "UPFG closed loop",
        AscentPhase.Terminal => "terminal (frozen)",
        _ => "?",
    };

    // Append a (downrange, altitude) sample on a fixed sim-time cadence while
    // guidance runs. Downrange is the great-circle ground distance from the launch
    // point, measured in the body-fixed (CCE) frame so the planet's rotation drops
    // out. When full, decimate 2:1 and double the cadence.
    private static void SampleTrajectory(Orbit orbit, IParentBody parent, double bodyRadius)
    {
        double now = SimNow();
        if (now < _nextSampleTime)
            return;
        _nextSampleTime += _sampleInterval;
        if (now >= _nextSampleTime) // time warp jumped ahead: re-anchor
            _nextSampleTime = now + _sampleInterval;

        double3 r = orbit.StateVectors.PositionCci;
        double altKm = (r.Length() - bodyRadius) / 1000.0;
        double3 dirCcf = double3.Normalize(r).Transform(parent.GetCci2Ccf());
        double angle = Math.Acos(Math.Clamp(double3.Dot(dirCcf, _launchDirCcf), -1.0, 1.0));
        double downrangeKm = angle * bodyRadius / 1000.0;
        TrajSamples.Add(new float2((float)downrangeKm, (float)altKm));

        if (TrajSamples.Count >= MaxTrajSamples)
        {
            for (int i = 0; i < TrajSamples.Count / 2; i++)
                TrajSamples[i] = TrajSamples[i * 2];
            TrajSamples.RemoveRange(TrajSamples.Count / 2, TrajSamples.Count - TrajSamples.Count / 2);
            _sampleInterval *= 2.0;
        }
    }

    private static void DrawTrajectoryPlot()
    {
        if (TrajSamples.Count < 2)
        {
            ImGui.Text("No trajectory recorded yet — EXECUTE starts recording.");
            return;
        }

        float maxX = 1f, maxY = 1f;
        for (int i = 0; i < TrajSamples.Count; i++)
        {
            maxX = Math.Max(maxX, TrajSamples[i].X);
            maxY = Math.Max(maxY, TrajSamples[i].Y);
        }

        var size = new float2(560f, 240f);
        float2 origin = ImGui.GetCursorScreenPos();
        ImGui.Dummy(size);
        ImDrawListPtr drawList = ImGui.GetWindowDrawList();

        drawList.AddRectFilled(origin, origin + size, new ImColor8(16, 20, 26), 4f);
        drawList.AddRect(origin, origin + size, new ImColor8(95, 100, 105), 4f);

        const float pad = 12f;
        float w = size.X - 2f * pad;
        float h = size.Y - 2f * pad;
        var points = new float2[TrajSamples.Count];
        for (int i = 0; i < TrajSamples.Count; i++)
        {
            points[i] = new float2(
                origin.X + pad + TrajSamples[i].X / maxX * w,
                origin.Y + pad + h - TrajSamples[i].Y / maxY * h);
        }

        drawList.AddPolyline(points, new ImColor8(60, 220, 90), ImDrawFlags.None, 3f);
        drawList.AddCircleFilled(points[points.Length - 1], 5f, new ImColor8(150, 255, 160));

        var axisCol = new ImColor8(160, 165, 170);
        drawList.AddText(origin + new float2(pad, 4f), axisCol, $"{maxY:F0} km alt");
        drawList.AddText(origin + new float2(size.X - 110f, size.Y - 20f), axisCol, $"{maxX:F0} km downrange");

        float2 last = TrajSamples[TrajSamples.Count - 1];
        ImGui.Text($"Downrange {last.X,7:F1} km    altitude {last.Y,7:F1} km");
    }

    // The staged vehicle in UPFG's format: KSA's remaining activation sequences
    // mapped to a list of constant-thrust stages. If the vehicle has no usable
    // sequences (e.g. a single stack with engines already lit and no decouplers),
    // fall back to one stage built from the live engine configuration. Null means
    // no thrust anywhere — a transient the caller waits out.
    private static UpfgVehicle BuildUpfgVehicle(Vehicle vehicle)
    {
        var upfgVehicle = KsaVehicleAdapter.Build(vehicle);
        if (upfgVehicle.Stages.Count > 0)
            return upfgVehicle;

        var cfg = vehicle.FlightComputer.VehicleConfig;
        double thrust = cfg.TotalEngineVacuumThrust;
        double exhaustVel = cfg.TotalEngineExhaustVelocity;
        if (thrust <= 0 || exhaustVel <= 0)
            return null;

        double mass = vehicle.TotalMass;
        upfgVehicle.Stages.Add(new UpfgStage
        {
            Mode = 1,
            Thrust = thrust,
            Isp = exhaustVel / 9.80665,
            MassTotal = mass,
            MassDry = Math.Max(mass - vehicle.PropellantMass, 1.0),
            GLim = 1e9,
        });
        return upfgVehicle;
    }

    // Called from the Harmony prefix on Vehicle.PrepareWorker (see Bootstrap) — i.e.
    // immediately before the sim snapshots the flight computer for this step, the one
    // place where our writes are guaranteed to reach the control loop instead of being
    // erased by the worker copy-back.
    public static void ApplyAutopilot(Vehicle vehicle)
    {
        // Fast path: this runs on every sim step for every vehicle (thousands of
        // calls per second under time warp) — bail before touching anything when
        // the autopilot has nothing to do.
        if (!_engage && !_wasEngaged && !_landingCutPending)
            return;

        if (!ReferenceEquals(vehicle, Program.ControlledVehicle))
            return;

        // The open-loop phases (vertical/kick/prograde) don't need a converged UPFG
        // solution; once flying, keep commanding through transient re-convergence
        // (e.g. right after staging) — dropping to Manual mid-ascent would be far
        // more disruptive.
        bool landingGuides = _landingPhase == LandingPhase.Prep
            || _landingPhase == LandingPhase.Burn
            || _landingPhase == LandingPhase.GfoldDescent
            || _landingPhase == LandingPhase.VerticalDescent;
        bool shouldCommand = _engage && (_running || landingGuides) && _hasCommand;
        var fc = vehicle.FlightComputer;

        // Auto engine control: master switch on at full throttle while flying, off
        // for good once the terminal countdown expires. Written here — the prefix
        // runs just before PrepareWorker snapshots _manualControlInputs — so it
        // reaches the sim exactly like the player's ignite/shutdown key.
        // One-shot engine cut when the landing flow ends (cutoff, abort, failure) —
        // after this the player's inputs are untouched, so the final descent below
        // the gate can be flown manually.
        if (_landingCutPending)
        {
            ref ManualControlInputs cut = ref ManualInputs(vehicle);
            cut.EngineOn = false;
            _landingCutPending = false;
        }

        if (_engage && _autoStage)
        {
            if (landingGuides)
            {
                ref ManualControlInputs inputs = ref ManualInputs(vehicle);
                if (_landingPhase == LandingPhase.Burn)
                {
                    inputs.EngineOn = true;
                    // Mode 3's throttle command stretches the burn onto the site.
                    inputs.EngineThrottle = (float)Guidance.Throttle;
                }
                else if (_landingPhase == LandingPhase.GfoldDescent)
                {
                    // Cut the engine on a planned coast so it genuinely throttles
                    // down. Hysteresis (off below 2%, on above 6%) stops the engine
                    // toggling every step when the command sits near the threshold.
                    if (_gfoldThrottle < GfoldCoastThrottle) _gfoldEngineOn = false;
                    else if (_gfoldThrottle > GfoldCoastThrottle * 3.0) _gfoldEngineOn = true;
                    inputs.EngineOn = _gfoldEngineOn;
                    inputs.EngineThrottle = (float)_gfoldThrottle;
                }
                else if (_landingPhase == LandingPhase.VerticalDescent)
                {
                    inputs.EngineOn = _gfoldThrottle > 0.01;
                    inputs.EngineThrottle = (float)_gfoldThrottle;
                }
                else
                {
                    inputs.EngineOn = false; // Prep (pre-ignition)
                }
            }
            else if (_running)
            {
                ref ManualControlInputs inputs = ref ManualInputs(vehicle);
                if (_phase == AscentPhase.Terminal && SimNow() >= _cutoffTime)
                {
                    inputs.EngineOn = false;
                    _cutoffDone = true;
                }
                else if (!_cutoffDone)
                {
                    inputs.EngineOn = true;
                    // Full throttle unless UPFG is holding the acceleration limit.
                    inputs.EngineThrottle = (float)Guidance.Throttle;
                }
            }
        }

        if (shouldCommand)
        {
            CommandAttitude(vehicle, vehicle.Orbit.Parent, _commandDir, fullEngage: !_wasEngaged);
            _wasEngaged = true;
        }
        else if (_wasEngaged)
        {
            // Disengaged (or guidance stopped): hand attitude back to the player.
            fc.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.None;
            fc.AttitudeMode = FlightComputerAttitudeMode.Manual;
            _wasEngaged = false;
        }
    }

    // Convert a commanded thrust direction into the flight computer's Custom-attitude
    // Euler command. We use KSA's own ComputeBurnBody2Cci to build the body→CCI
    // orientation that points thrust along the steering vector, then express it as
    // Euler angles in the EclBody frame — the exact inverse of the conversion the
    // flight computer applies when it reads CustomAttitudeTarget.
    //
    // fullEngage=true additionally switches the FC into Custom/Auto tracking — done
    // once on engage, exactly like clicking "Apply Euler Target" in the attitude tab.
    private static void CommandAttitude(Vehicle vehicle, IParentBody parent, double3 dir, bool fullEngage)
    {
        double3 r = vehicle.Orbit.StateVectors.PositionCci;

        float3 posDir = float3.Pack(double3.Normalize(r));
        float3 steerDir = float3.Pack(double3.Normalize(dir));

        doubleQuat desiredBody2Cci = BurnTarget.ComputeBurnBody2Cci(posDir, steerDir);
        doubleQuat frame2Cci = VehicleReferenceFrameEx.GetEclBody2Cci(parent.GetCce2Cci());

        // Solve Concatenate(value, frame2Cci) == desired  ->  value = Concatenate(desired, inverse(frame2Cci)).
        doubleQuat value = doubleQuat.Concatenate(desiredBody2Cci, doubleQuat.Inverse(frame2Cci));
        double3 euler = value.ToRollYawPitchRadians();

        var fc = vehicle.FlightComputer;
        fc.CustomAttitudeTarget = euler;
        if (fullEngage)
        {
            fc.AttitudeFrame = VehicleReferenceFrame.EclBody;
            fc.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
        }
    }

    // The steering direction expressed as the same pitch/heading numbers the in-game
    // navball shows in its surface (EnuBody) frame. Computed with KSA's own functions
    // (ComputeBurnBody2Cci + EnuBody frame + RollPitchYaw decomposition + compass
    // wrap) so the readout matches the navball digit-for-digit. Note KSA's ENU frame
    // is East-referenced, so this differs from a real-world compass azimuth by 90°.
    private static (double pitchDeg, double headingDeg) NavballSteerAngles(double3 r, double3 dir)
    {
        if (r.Length() < 1 || dir.Length() < 1e-9) return (0, 0);

        doubleQuat desired = BurnTarget.ComputeBurnBody2Cci(
            float3.Pack(double3.Normalize(r)), float3.Pack(double3.Normalize(dir)));
        doubleQuat enuBody2Cci = VehicleReferenceFrameEx.GetEnuBody2Cci(r) ?? doubleQuat.Identity;

        // Same construction as the navball: frame -> desired-body orientation.
        doubleQuat frame2Desired = doubleQuat.Concatenate(enuBody2Cci, doubleQuat.Inverse(desired));
        double3 angles = VehicleReferenceFrame.EnuBody.QuaternionToEulerAngles(frame2Desired);

        double pitchDeg = angles.Y * 180.0 / Math.PI;
        double headingDeg = MathEx.ToCompassAngle(angles.Z) * 180.0 / Math.PI;
        return (pitchDeg, headingDeg);
    }

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

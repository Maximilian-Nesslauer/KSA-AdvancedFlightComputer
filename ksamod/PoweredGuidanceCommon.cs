using System;
using System.Collections.Generic;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;
using PoweredGuidance.Upfg;

// Shared state and plumbing used by both the Ascent and Landing flows: the
// engage/auto toggles, the commanded attitude, the autopilot writes (applied from
// the Vehicle.PrepareWorker Harmony prefix), auto-staging, the UPFG vehicle
// builder, the warp-confirmation prompt, and small math helpers.
public static partial class PoweredGuidanceWindow
{
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

    // The attitude the current phase wants, produced each frame and applied from
    // the Vehicle.PrepareWorker prefix (see ApplyAutopilot).
    private static double3 _commandDir;
    private static bool _hasCommand;

    // Auto engines & staging: while armed (and the autopilot is engaged), the mod
    // ignites the first sequence, fires the next sequence whenever the current
    // powered phase has no thrust or is about to flame out, and shuts the engines
    // down at the terminal cutoff. Throttle is forced to 1 while burning.
    private static bool _autoStage = true;   // defaults on, like _engage
    private static bool _cutoffDone;
    private static bool _stagingActive;
    private static double _lastSequenceTime = double.NegativeInfinity;
    private const double SequenceCooldown = 1.0;   // s between auto activations

    // Vehicle-wide acceleration limit: any part of any burn that would exceed this
    // becomes a constant-acceleration (Mode 2) segment in the UPFG stage list, and
    // the auto-throttle follows UPFG's throttle command to hold it.
    private static bool _gLimitEnabled;
    private static double _gLimitG = 4.0;

    private const double WarpLeadTime = 10.0;  // end auto-warp this many s early

    // The engine master switch and throttle live in Vehicle's private
    // _manualControlInputs; the game's own ignite/shutdown actions just set
    // EngineOn there, so we do exactly the same via a field ref.
    private static readonly AccessTools.FieldRef<Vehicle, ManualControlInputs> ManualInputs =
        AccessTools.FieldRefAccess<Vehicle, ManualControlInputs>("_manualControlInputs");

    // The mod's clock: elapsed sim time in seconds. Used for everything time-based
    // (turn ramp, staging cooldown, cutoff) so behavior is correct under time
    // warp, unlike the wall-clock-ish player time.
    private static double SimNow() => Universe.GetElapsedSimTime().Seconds();

    // --- Warp confirmation ---
    // Nothing in the mod starts a time warp on its own: flows that want one call
    // RequestWarp, and DrawWarpPrompt asks the user first. This prevents warping
    // into a planet because a predicted burn point happened to be far away.
    private static bool _warpPromptActive;
    private static string _warpLabel = "";
    private static double _warpTargetSimSec;
    private static string _warpDeclinedLabel = "";

    private static void RequestWarp(double targetSimSec, string label)
    {
        if (_warpDeclinedLabel == label)
            return;   // the user said no to this one — don't nag every frame
        _warpPromptActive = true;
        _warpLabel = label;
        _warpTargetSimSec = targetSimSec;
    }

    private static void DrawWarpPrompt()
    {
        if (!_warpPromptActive)
            return;
        double wait = _warpTargetSimSec - SimNow();
        if (wait <= 1.0 || Universe.IsAutoWarpActive)
        {
            _warpPromptActive = false;
            return;
        }
        ImGui.SeparatorText("Time warp");
        ImGui.TextColored(new float4(0.5f, 0.9f, 1f, 1f),
            $"Warp to {_warpLabel}?  (T-{wait,6:F0} s)");
        if (ImGui.Button("Warp"))
        {
            Universe.AutoWarpTo(Universe.GetElapsedSimTime() + wait);
            _warpPromptActive = false;
        }
        ImGui.SameLine();
        if (ImGui.Button("Stay at 1x"))
        {
            _warpPromptActive = false;
            _warpDeclinedLabel = _warpLabel;
        }
    }

    // Staging needs an unknown number of sequence activations (decouple, then
    // ignite, sometimes another press before that) — so whenever the vehicle has no
    // engine actually producing thrust (lit AND fed with propellant, per the game's
    // own live engine state), keep firing the next sequence every SequenceCooldown
    // seconds until one is. That covers pad ignition, burnout staging, and
    // decouple-only sequences. Same call pair the game's staging key uses, from
    // the same (main) thread.
    //
    // Total thrust loss is not the only staging cue, though: strap-on boosters
    // burn out while the core keeps firing, so the vehicle never goes quiet and
    // spent casings would ride all the way to orbit. DropSpentEngines covers that
    // second case.
    private static void AutoSequence(Vehicle vehicle)
    {
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

        bool thrustOn = vehicle.IsAnyEngineActive() && vehicle.IsAnyEnginePropellantAvailable();
        if (thrustOn && !ShouldDropSpentEngines(vehicle, sequenceList))
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
            _stageModelDirty = true;   // the stage list just changed under us
            _spentStagedFor.Clear();
            _spentStagedFor.UnionWith(_spentEngineParts);
        }
    }

    // Engine parts that are lit but out of propellant, and the set we last staged
    // for. Kept as fields so the check allocates nothing on the sim path.
    private static readonly HashSet<uint> _spentEngineParts = new HashSet<uint>();
    private static readonly HashSet<uint> _spentStagedFor = new HashSet<uint>();

    // True when a burnt-out engine is still attached and the next sequence is the
    // one that separates something — i.e. spent boosters waiting to be dropped
    // while the core still burns.
    //
    // Two guards keep this from turning into a staging loop. The next sequence
    // must actually contain a decoupler, so a dead engine with nothing left to
    // separate is ignored; and the same set of dead engines only ever triggers one
    // activation, so if a sequence fires without removing them we stop rather than
    // walk the whole list at one activation per cooldown.
    private static bool ShouldDropSpentEngines(Vehicle vehicle, SequenceList sequenceList)
    {
        _spentEngineParts.Clear();
        if (!ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
                .TryGetFrom(vehicle.Parts.States, out var engineStates))
            return false;

        foreach (var engine in engineStates.ModulesAndStates)
        {
            if (engine.Module.IsActive && !engine.State.IsPropellantAvailable)
                _spentEngineParts.Add(engine.Module.Parent.FullPart.InstanceId);
        }
        if (_spentEngineParts.Count == 0 || _spentEngineParts.SetEquals(_spentStagedFor))
            return false;

        ReadOnlySpan<Sequence> sequences = sequenceList.Sequences;
        for (int i = 0; i < sequences.Length; i++)
        {
            if (sequences[i].Activated)
                continue;
            ReadOnlySpan<Part> parts = sequences[i].Parts;
            for (int j = 0; j < parts.Length; j++)
                if (!parts[j].SubtreeModules.Get<Decoupler>().IsEmpty)
                    return true;
            return false;   // the next sequence separates nothing
        }
        return false;
    }

    // --- Stage model ---
    // KSA models staging itself (PartTree.PerformanceSequences — see
    // KsaVehicleAdapter), but in flight it only recomputes while the stage or
    // engine-control panel is open, so the mod drives it.
    //
    // Where this happens matters. The game's own recompute runs on a vehicle
    // worker thread, concurrently with the UI draw; PerformanceSequences
    // double-buffers its arrays for that, but the Lists inside them are reused,
    // so reading the stage list from the draw can tear. The PrepareWorker prefix
    // can't: Universe.PrepareVehicleWorkers runs on the main thread after
    // VehicleSolvers.Wait() and before the tasks are re-queued, so no worker is
    // in flight. Hence both the recompute and the copy-out happen here, and the
    // guidance step consumes the snapshot. One sim step of staleness is
    // immaterial — UPFG reconciles stage 0 against the live mass every step.
    private static UpfgVehicle _stageModel;
    private static Vehicle _stageModelVehicle;
    private static bool _stageModelDirty = true;
    private static long _stageModelTick;
    private const long StageModelIntervalMs = 250;

    private static void RefreshStageModel(Vehicle vehicle)
    {
        // A different vehicle is being flown: the old snapshot describes someone
        // else's staging, so drop it rather than let it be read for an interval.
        if (!ReferenceEquals(vehicle, _stageModelVehicle))
        {
            _stageModel = null;
            _stageModelVehicle = vehicle;
            _stageModelDirty = true;
        }

        // Wall-clock gated, not sim-time gated: under warp sim time elapses
        // instantly and this would run every step.
        long now = Environment.TickCount64;
        if (!_stageModelDirty && now - _stageModelTick < StageModelIntervalMs)
            return;
        _stageModelTick = now;
        _stageModelDirty = false;

        // A staging frame can catch the part tree mid-rebuild. Losing one
        // refresh is harmless — the previous snapshot stays valid and we retry
        // immediately — but letting it escape would skip the attitude command
        // for that step, which is not.
        try
        {
            // Vacuum. Closed-loop guidance only flies above significant
            // atmosphere, and UPFG re-converges in real time regardless; the
            // pressure argument would in any case only change the active
            // sequence's headline thrust, which the adapter doesn't consume.
            SequencePerformanceList performance = vehicle.Parts?.PerformanceSequences;
            if (performance == null || vehicle.Parts.SequenceList == null)
                return;
            performance.RecomputeForFlight(0f);
            _stageModel = KsaVehicleAdapter.Build(vehicle);
        }
        catch (Exception)
        {
            _stageModelDirty = true;
        }
    }

    // The staged vehicle in UPFG's format, taken from the snapshot above. If the
    // vehicle has no usable sequences (e.g. a single stack with engines already
    // lit and no decouplers), fall back to one stage built from the live engine
    // configuration. Null means "nothing to fly with yet" — either the snapshot
    // hasn't been taken or there is no thrust anywhere — a transient the caller
    // waits out by holding its last solution.
    private static UpfgVehicle BuildUpfgVehicle(Vehicle vehicle)
    {
        UpfgVehicle snapshot = _stageModel;
        if (snapshot == null)
            return null;   // no refresh has landed for this vehicle yet

        if (snapshot.Stages.Count > 0)
        {
            // Hand out fresh stage objects: ApplyGLimit rewrites Mode/GLim in
            // place, and the snapshot has to survive into the next step.
            var copy = new UpfgVehicle();
            foreach (UpfgStage s in snapshot.Stages)
                copy.Stages.Add(new UpfgStage
                {
                    Mode = s.Mode, Thrust = s.Thrust, Isp = s.Isp,
                    MassTotal = s.MassTotal, MassDry = s.MassDry, GLim = s.GLim,
                });
            return copy;
        }

        var upfgVehicle = new UpfgVehicle();
        (double thrust, double massFlow) = KsaEnginePerf.Vacuum(vehicle);
        double exhaustVel = massFlow > 0 ? thrust / massFlow : 0.0;
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

    // Called from the Harmony prefix on Vehicle.PrepareWorker (see Mod) — i.e.
    // immediately before the sim snapshots the flight computer for this step, the one
    // place where our writes are guaranteed to reach the control loop instead of being
    // erased by the worker copy-back.
    public static void ApplyAutopilot(Vehicle vehicle)
    {
        // Fast path: this runs on every sim step for every vehicle (thousands of
        // calls per second under time warp) — do nothing for anything that isn't
        // the vehicle the player is flying.
        if (!ReferenceEquals(vehicle, Program.ControlledVehicle))
            return;

        // Keep the staging model current even while the autopilot is idle: both
        // EXECUTE handlers need a stage list the instant they are pressed, and
        // this is the only point in the frame where it can be built without
        // racing the game's own recompute on the vehicle worker thread. Gated to
        // ~4 Hz on the wall clock, so time warp doesn't multiply it.
        RefreshStageModel(vehicle);

        // Bail before touching the flight computer when the autopilot has nothing
        // to do. Keyed on the mod actually being active (guidance running or a
        // landing in progress), not on the engage toggle, which defaults on and
        // would defeat the bail.
        bool landingActive = _landingPhase != LandingPhase.Idle && _landingPhase != LandingPhase.Done;
        if (!_running && !landingActive && !_wasEngaged && !_landingCutPending)
            return;

        // The open-loop phases (vertical/kick/prograde) don't need a converged UPFG
        // solution; once flying, keep commanding through transient re-convergence
        // (e.g. right after staging) — dropping to Manual mid-ascent would be far
        // more disruptive.
        bool landingGuides = _landingPhase == LandingPhase.Prep
            || _landingPhase == LandingPhase.Burn
            || _landingPhase == LandingPhase.GfoldDescent
            || _landingPhase == LandingPhase.TerminalHover;
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
                else if (_landingPhase == LandingPhase.TerminalHover)
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

    // ----- Small math helpers -----

    private static double3 Node(double[][] a, int i) => new double3(a[i][0], a[i][1], a[i][2]);
    private static double3 Lerp(double3 a, double3 b, double t) => a + (b - a) * t;

    private static double Wrap2Pi(double a)
    {
        a %= 2.0 * Math.PI;
        return a < 0 ? a + 2.0 * Math.PI : a;
    }

    private static double AngleBetween(double3 a, double3 b)
    {
        double d = double3.Dot(double3.Normalize(a), double3.Normalize(b));
        return Math.Acos(Math.Clamp(d, -1.0, 1.0));
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
}

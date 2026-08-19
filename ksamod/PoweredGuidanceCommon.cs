using System;
using System.Collections.Generic;
using System.Reflection;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;
using PoweredGuidance.Upfg;

// Shared plumbing used by every flow: the per-vehicle step-and-apply entry point
// (ApplyAutopilot, called from the Vehicle.PrepareWorker Harmony prefix for each
// craft in turn), the commanded attitude, auto-staging, the UPFG vehicle builder,
// the warp-confirmation prompt, and small math helpers.
//
// The state these act on is NOT here — it lives on the vehicle being serviced, via
// the ambient _s pointer (see VehicleAutopilotState). What remains static in this
// partial class belongs to the panel or to the process, not to a craft.
public static partial class PoweredGuidanceWindow
{

    // Guidance must survive transient bad frames (the staging frame reports zero
    // thrust while the next engine ignites; the part tree can be mid-mutation). A
    // single failed step skips that frame and keeps the last solution; only a long
    // unbroken run of failures stops guidance for good.
    private const int MaxFailStreak = 600; // ~10 s of consecutive bad frames

    // THE UPFG SOLVER IS PER VEHICLE — see VehicleAutopilotState.Upfg. It used to be
    // one static instance shared by every craft, which is wrong for a filter that
    // warm-starts from its own previous state and differences velocities across
    // calls: switching focus fed one vehicle's inertial velocity into another's vgo.

    // The staged vehicle model UPFG flies, rebuilt from KSA's sequence list every
    // guidance step (matching original navbox, where the sim feeds UPFG current
    // data each cycle): stage 0 always carries the present masses, so burn times
    // are inherently time-remaining, and manual staging shows up automatically.

    // Auto engines & staging: while armed (and the autopilot is engaged), the mod
    // ignites the first sequence, fires the next sequence whenever the current
    // powered phase has no thrust or is about to flame out, and shuts the engines
    // down at the terminal cutoff. Throttle is forced to 1 while burning.
    private const double SequenceCooldown = 1.0;   // s between auto activations

    // Vehicle-wide acceleration limit: any part of any burn that would exceed this
    // becomes a constant-acceleration (Mode 2) segment in the UPFG stage list, and
    // the auto-throttle follows UPFG's throttle command to hold it.

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
            _s.StagingActive = false;
            return;
        }

        bool thrustOn = vehicle.IsAnyEngineActive() && vehicle.IsAnyEnginePropellantAvailable();

        // EVALUATED UNCONDITIONALLY, not short-circuited behind thrustOn. Its real
        // output is not just the bool: it refills _spentEngineParts, and the
        // activation below records that set as "already staged for". Skipping the
        // call left the buffer holding whatever the PREVIOUS vehicle in this step's
        // sweep put there, and that craft's engine ids were then written into this
        // one's SpentStagedFor. Harmless while one vehicle was ever serviced; not
        // once the hook runs every craft in sequence.
        bool dropSpent = ShouldDropSpentEngines(vehicle, sequenceList);
        if (thrustOn && !dropSpent)
        {
            _s.StagingActive = false;
            return;
        }

        _s.StagingActive = true;
        double now = SimNow();
        if (now - _s.LastSequenceTime >= SequenceCooldown)
        {
            if (WouldLoseControl(vehicle, sequenceList))
            {
                _s.StagingActive = false;
                _s.Status = "Auto-staging held: the next sequence would separate the control module.";
                return;
            }
            sequenceList.ActivateNextSequence(vehicle);
            vehicle.UpdateAfterPartTreeModification();
            _s.LastSequenceTime = now;
            _s.StageModelDirty = true;   // the stage list just changed under us
            _s.SpentStagedFor.Clear();
            _s.SpentStagedFor.UnionWith(_spentEngineParts);
        }
    }

    private static readonly HashSet<Part> _stagingDropped = new HashSet<Part>();

    // True if firing the next sequence would leave the vehicle we are flying with
    // no control module.
    //
    // Vehicle.Split detaches the TREE-CHILD side of a decoupler's connection into a
    // NEW vehicle and keeps the tree-parent side as the vehicle object the player is
    // still controlling — and nothing in KSA moves control to follow the pod
    // (Program.ControlledVehicle is only reassigned by camera targeting and EVA). So
    // if every Control module sits on the child side, that separation hands the pod
    // away and leaves the player attached to the debris. The symptom is unmistakable
    // once seen: Vehicle.IsControllable is `Parts.Controls.NumModules > 0`, and the
    // flight computer greys out everything it gates on that — the Strict/Balanced/
    // Relaxed attitude profiles included, since those fall through to a bare
    // !IsControllable test.
    //
    // Doing that deliberately is a legitimate thing to want; doing it automatically,
    // mid-ascent, is not the autopilot's call.
    private static bool WouldLoseControl(Vehicle vehicle, SequenceList sequenceList)
    {
        PartTree tree = vehicle.Parts;
        if (tree == null)
            return false;

        // The sequence ActivateNextSequence will actually fire: the first one not
        // yet activated that still has parts (it skips empty ones).
        Sequence next = null;
        ReadOnlySpan<Sequence> sequences = sequenceList.Sequences;
        for (int i = 0; i < sequences.Length; i++)
        {
            if (!sequences[i].Activated && !sequences[i].Parts.IsEmpty)
            {
                next = sequences[i];
                break;
            }
        }
        if (next == null)
            return false;

        _stagingDropped.Clear();
        ReadOnlySpan<Part> parts = next.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            Span<Decoupler> decouplers = parts[i].SubtreeModules.Get<Decoupler>();
            for (int j = 0; j < decouplers.Length; j++)
            {
                Part root = DetachedRoot(decouplers[j]);
                if (root != null)
                    CollectSubtree(root, _stagingDropped);
            }
        }
        if (_stagingDropped.Count == 0)
            return false;   // nothing separates

        Span<Control> controls = tree.Modules.Get<Control>();
        if (controls.Length == 0)
            return false;   // already uncontrollable — nothing left to protect
        for (int i = 0; i < controls.Length; i++)
            if (!_stagingDropped.Contains(controls[i].Parent.FullPart))
                return false;   // at least one control module stays with us
        return true;
    }

    // The part whose subtree separates when this decoupler fires: the tree-child
    // side of its connection — the same rule Vehicle.Split applies.
    private static Part DetachedRoot(Decoupler decoupler)
    {
        Part.Connection conn = decoupler.Connector?.Connection;
        if (conn == null)
            return null;
        Part a = conn.Connectors[0].ConnectionPart;
        Part b = conn.Connectors[1].ConnectionPart;
        if (a == null || b == null)
            return null;
        return !a.TreeChildren.Contains(b) ? a : b;
    }

    private static void CollectSubtree(Part root, HashSet<Part> into)
    {
        if (!into.Add(root))
            return;
        foreach (Part child in root.TreeChildren)
            CollectSubtree(child, into);
    }

    // Engine parts that are lit but out of propellant. SCRATCH: filled and consumed
    // within one vehicle's AutoSequence call, never read across calls — the set we
    // last staged for is per vehicle (VehicleAutopilotState.SpentStagedFor). Kept as a
    // field so the check allocates nothing on the sim path.
    private static readonly HashSet<uint> _spentEngineParts = new HashSet<uint>();

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
        if (_spentEngineParts.Count == 0 || _spentEngineParts.SetEquals(_s.SpentStagedFor))
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
    private const long StageModelIntervalMs = 250;

    private static void RefreshStageModel(Vehicle vehicle)
    {
        // No cross-vehicle invalidation any more. The snapshot lives on the vehicle it
        // describes, so switching craft reads a different one rather than reading
        // someone else's staging for an interval - which is what the old
        // _stageModelVehicle field existed to prevent.

        // Wall-clock gated, not sim-time gated: under warp sim time elapses
        // instantly and this would run every step.
        long now = Environment.TickCount64;
        if (!_s.StageModelDirty && now - _s.StageModelTick < StageModelIntervalMs)
            return;
        _s.StageModelTick = now;
        _s.StageModelDirty = false;

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
            _s.StageModel = KsaVehicleAdapter.Build(vehicle);
        }
        catch (Exception)
        {
            _s.StageModelDirty = true;
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
        UpfgVehicle snapshot = _s.StageModel;
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
    //
    // EVERY MODE IS STEPPED HERE, FOR EVERY VEHICLE. Ascent and the landing machine
    // used to be stepped from the UI draw, which is called once per frame for the
    // focused craft only — so an unfocused vehicle's guidance simply stopped, and its
    // flight computer coasted on whatever attitude it had been left holding until the
    // player looked at it again. That is not an autopilot per vehicle; it is one
    // autopilot that follows the camera. Running the flows from the per-vehicle sim
    // hook is what makes a booster able to fly itself home unwatched, and it also
    // closes the frame of lag that came from computing a command in the draw and only
    // applying it at the NEXT step's prefix.
    public static void ApplyAutopilot(Vehicle vehicle)
    {
        bool focused = ReferenceEquals(vehicle, Program.ControlledVehicle);

        // Point the ambient state at THIS vehicle for everything that follows.
        //
        // For() only for the focused craft, because it creates the entry and this runs
        // for every vehicle on every sim step — thousands of calls a second under time
        // warp — so a craft that has never been engaged must cost one failed lookup
        // and no allocation.
        if (focused)
            Use(vehicle);
        else if (VehicleAutopilotState.TryGet(vehicle, out VehicleAutopilotState state))
            _s = state;
        else
            return;

        bool sixDof = _s.Active || _s.EngagePending;
        bool landingActive = _s.LandingPhase != LandingPhase.Idle && _s.LandingPhase != LandingPhase.Done;
        bool flying = sixDof || _s.Running || landingActive || _s.WasEngaged || _s.LandingCutPending;

        // Nothing engaged and nobody looking: an unfocused idle craft is not worth a
        // stage-model rebuild or a trace sample.
        if (!focused && !flying)
            return;

        // HOUSEKEEPING FIRST, ahead of the 6-DOF dispatch on purpose — that dispatch
        // returns, so anything below it is skipped for a craft flying 6-DOF.
        //
        // Keep the staging model current even while the autopilot is idle: both
        // EXECUTE handlers need a stage list the instant they are pressed, and this is
        // the only point in the frame where it can be built without racing the game's
        // own recompute on the vehicle worker thread. Gated to ~4 Hz on the wall
        // clock, per vehicle, so time warp doesn't multiply it.
        RefreshStageModel(vehicle);

        // Likewise the flown-trajectory trace: sampled off the simulation rather than
        // the frame rate, and recorded whether or not guidance is running so the track
        // is already there when the overlay is switched on — losing it the moment
        // guidance engages would blank the overlay during exactly the descent worth
        // watching.
        RecordTrace(vehicle, vehicle.Orbit);

        // A requested reset runs ahead of everything below: the whole point of the
        // button is to recover when the mod's own state is wrong, so it must not
        // depend on that state saying the autopilot is still active.
        if (_s.FcResetPending)
        {
            ApplyPendingFcReset(vehicle);
            return;
        }

        // 6-DOF is EXCLUSIVE: it drives attitude through the TVC allocator rather than
        // the flight computer, so it must not be mixed with the UPFG / G-FOLD command
        // path below. It has its own engage flag and does not set _s.Running.
        if (sixDof)
        {
            Step6Dof(vehicle);
            return;
        }

        if (!flying)
            return;

        // --- Step this vehicle's guidance, then apply what it produced ---
        Orbit orbit = vehicle.Orbit;
        IParentBody stepParent = orbit.Parent;
        StepLanding(vehicle, orbit, stepParent, stepParent.Mu, stepParent.MeanRadius);
        StepAscent(vehicle, orbit, stepParent, stepParent.Mu, stepParent.MeanRadius);

        // Read straight off _s, not off the values the bail above was computed from:
        // the flows just ran, and a touchdown, an abort or a handoff can have changed
        // the phase on this very step.
        //
        // The open-loop phases (vertical/kick/prograde) don't need a converged UPFG
        // solution; once flying, keep commanding through transient re-convergence
        // (e.g. right after staging) — dropping to Manual mid-ascent would be far
        // more disruptive.
        bool landingGuides = _s.LandingPhase == LandingPhase.Prep
            || _s.LandingPhase == LandingPhase.Burn
            || _s.LandingPhase == LandingPhase.GfoldDescent
            || _s.LandingPhase == LandingPhase.TerminalHover;
        bool shouldCommand = _s.Engage && (_s.Running || landingGuides) && _s.HasCommand;

        // Auto engine control: master switch on at full throttle while flying, off
        // for good once the terminal countdown expires. Written here — the prefix
        // runs just before PrepareWorker snapshots _manualControlInputs — so it
        // reaches the sim exactly like the player's ignite/shutdown key.
        // One-shot engine cut when the landing flow ends (cutoff, abort, failure) —
        // after this the player's inputs are untouched, so the final descent below
        // the gate can be flown manually.
        if (_s.LandingCutPending)
        {
            ref ManualControlInputs cut = ref ManualInputs(vehicle);
            cut.EngineOn = false;
            _s.LandingCutPending = false;
        }

        if (_s.Engage && _s.AutoStage)
        {
            if (landingGuides)
            {
                ref ManualControlInputs inputs = ref ManualInputs(vehicle);
                if (_s.LandingPhase == LandingPhase.Burn)
                {
                    inputs.EngineOn = true;
                    // Mode 3's throttle command stretches the burn onto the site.
                    inputs.EngineThrottle = (float)_s.Upfg.Throttle;
                }
                else if (_s.LandingPhase == LandingPhase.GfoldDescent)
                {
                    // Cut the engine on a planned coast so it genuinely throttles
                    // down. Hysteresis (off below 2%, on above 6%) stops the engine
                    // toggling every step when the command sits near the threshold.
                    if (_s.GfoldThrottle < GfoldCoastThrottle) _s.GfoldEngineOn = false;
                    else if (_s.GfoldThrottle > GfoldCoastThrottle * 3.0) _s.GfoldEngineOn = true;
                    inputs.EngineOn = _s.GfoldEngineOn;
                    inputs.EngineThrottle = (float)_s.GfoldThrottle;
                }
                else if (_s.LandingPhase == LandingPhase.TerminalHover)
                {
                    inputs.EngineOn = _s.GfoldThrottle > 0.01;
                    inputs.EngineThrottle = (float)_s.GfoldThrottle;
                }
                else
                {
                    inputs.EngineOn = false; // Prep (pre-ignition)
                }
            }
            else if (_s.Running)
            {
                ref ManualControlInputs inputs = ref ManualInputs(vehicle);
                if (_s.Phase == AscentPhase.Terminal && SimNow() >= _s.CutoffTime)
                {
                    inputs.EngineOn = false;
                    _s.CutoffDone = true;
                }
                else if (!_s.CutoffDone)
                {
                    inputs.EngineOn = true;
                    // Full throttle unless UPFG is holding the acceleration limit.
                    inputs.EngineThrottle = (float)_s.Upfg.Throttle;
                }
            }
        }

        if (shouldCommand)
        {
            CommandAttitude(vehicle, vehicle.Orbit.Parent, _s.CommandDir, fullEngage: !_s.WasEngaged);
            _s.WasEngaged = true;
        }
        else if (_s.WasEngaged)
        {
            ReleaseAttitude(vehicle);
            _s.WasEngaged = false;
        }
    }

    // Vehicle.FlightComputer is `get; private set;`, so swapping in a new instance
    // needs the non-public setter. Same approach as ManualInputs above.
    private static readonly MethodInfo SetVehicleFlightComputer =
        AccessTools.PropertySetter(typeof(Vehicle), nameof(Vehicle.FlightComputer));

    // Hand the vehicle back to the player by giving it a brand new flight computer,
    // straight off the default template.
    //
    // Restoring individual fields is the wrong shape for this: AngleDeadband and
    // RateLimit are a ratchet (UpdateRcsParams only ever raises them, ONE-WAY,
    // toward the RCS's physical floor) and RateDeadband/RateBit/AttitudeTarget are
    // derived from them each step, so a snapshot taken while our guidance was
    // flying could already be sitting on a ratcheted value — restoring it just
    // restores the corruption. A new FlightComputer starts at the Balanced-profile
    // defaults with nothing ratcheted.
    //
    // It also clears CustomAttitudeTarget, which MUST be cleared and not merely
    // untracked: KSA reads that one field two ways depending on AttitudeTrackTarget
    // — Euler angles under Custom (what we write to steer), but a body RATE command
    // in rad/s under None (see UpdateAttitudeTarget). Leaving our steering angles
    // behind with tracking dropped to None is read as a rate command of up to
    // pi rad/s, and the vehicle tumbles the moment anything selects a rate mode.
    //
    // ReadUpdatedVehicleConfiguration is ESSENTIAL here, not a nicety. A fresh
    // FlightComputer has an empty VehicleConfig — no thrusters, no gimbals — and
    // the game only repopulates it on part-tree modification, refill/deplete, or
    // save load; NOT every step. Without this call the vehicle keeps its attitude
    // commands but has no RCS or TVC to execute them with, and stays that way until
    // the player happens to stage or dock.
    //
    // The new BurnPlan is empty, so any burn node the player had queued is cleared
    // along with everything else. That is the cost of a genuine reset rather than a
    // selective one.
    private static void ReleaseAttitude(Vehicle vehicle)
    {
        var fresh = new FlightComputer();
        if (SetVehicleFlightComputer != null)
        {
            SetVehicleFlightComputer.Invoke(vehicle, new object[] { fresh });
            fresh.ReadUpdatedVehicleConfiguration(vehicle);
            return;
        }

        // Setter not resolvable (a game update changed the property): copy the
        // template onto the existing instance instead. Same resulting state, minus
        // ConservativeFlipTime/DetumbleRateLimit, which CopyFrom skips and
        // UpdateRcsParams recomputes anyway.
        FlightComputer fc = vehicle.FlightComputer;
        fc.CopyFrom(fresh);
        fc.ReadUpdatedVehicleConfiguration(vehicle);
    }

    // The "Reset flight computer" button's action: unconditionally stop every
    // guidance flow, cut the engine, and reset the flight computer — regardless of
    // what state the mod's own bookkeeping thinks it's in. A backstop for the
    // normal disengage path not running (an unhandled exception, a phase the
    // fail-streak logic didn't cover), so it deliberately doesn't rely on any of
    // that bookkeeping being correct.
    //
    // The mod-side flags are set here, but the flight-computer write itself is
    // only REQUESTED — see _s.FcResetPending. This runs from the UI draw, and a
    // flight-computer write from the draw does not survive: within one frame the
    // game applies the worker's results onto the live FC, then runs PrepareWorker
    // and snapshots the FC into NewFlightComputer, and only then draws the UI. So
    // anything the draw writes lands after that snapshot and is overwritten by the
    // next frame's copy-back. The same reason every other FC write in this mod
    // happens from the PrepareWorker prefix.

    private static void ResetFlightComputer()
    {
        _s.Running = false;
        _s.LandingPhase = LandingPhase.Idle;
        _s.AutoLaunch = false;
        _s.HasCommand = false;
        _s.FcResetPending = true;
        _s.Status = "Flight computer reset — autopilot disengaged, engine cut.";
    }

    // Applies a requested reset from inside the PrepareWorker prefix, where writes
    // to the flight computer and to _manualControlInputs actually reach the sim.
    private static void ApplyPendingFcReset(Vehicle vehicle)
    {
        _s.FcResetPending = false;
        ReleaseAttitude(vehicle);

        ref ManualControlInputs inputs = ref ManualInputs(vehicle);
        inputs.EngineOn = false;
        inputs.EngineThrottle = 0f;

        // Cleared last: _s.WasEngaged gates the normal release path, which would
        // otherwise fire again on top of the reset we just did.
        _s.WasEngaged = false;
        _s.LandingCutPending = false;
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

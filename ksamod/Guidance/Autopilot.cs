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
    private static double SimNow() => Universe.GetElapsedSeconds();

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
            Universe.AutoWarpTo(Universe.GetElapsedTime() + wait);
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

        // LEVER TWO: the reserve's own staging cue.
        //
        // The other two cues both wait for propellant to run out - "nothing is producing
        // thrust" and ShouldDropSpentEngines. This one is the opposite: it fires while
        // the stage is still perfectly capable of burning, because the propellant left
        // in it is spoken for. Without it the reserve does nothing at all, since UPFG
        // plans stage boundaries but never commands one.
        bool reserveDone = ShouldStageForReserve(vehicle);

        // EVALUATED UNCONDITIONALLY, not short-circuited behind thrustOn. Its real
        // output is not just the bool: it refills _spentEngineParts, and the
        // activation below records that set as "already staged for". Skipping the
        // call left the buffer holding whatever the PREVIOUS vehicle in this step's
        // sweep put there, and that craft's engine ids were then written into this
        // one's SpentStagedFor. Harmless while one vehicle was ever serviced; not
        // once the hook runs every craft in sequence.
        bool dropSpent = ShouldDropSpentEngines(vehicle, sequenceList);
        if (thrustOn && !dropSpent && !reserveDone)
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
            // Everything the booster needs to be recognised and flown, recorded BEFORE
            // the split - afterwards the parts have moved to a vehicle we have no
            // handle on and the decoupler that named them is gone.
            if (reserveDone)
                ArmBoosterHandover(vehicle, sequenceList, now);

            sequenceList.ActivateNextSequence(vehicle);
            vehicle.UpdateAfterPartTreeModification();
            _s.LastSequenceTime = now;
            _s.StageModelDirty = true;   // the stage list just changed under us
            _s.SpentStagedFor.Clear();
            _s.SpentStagedFor.UnionWith(_spentEngineParts);
        }
    }

    /// <summary>
    /// True when the stage now burning has come down to its reserve and should be
    /// staged even though it could keep going.
    ///
    /// Remaining propellant is taken as live mass minus the CACHED model's burnout
    /// mass, not minus Vehicle.PropellantMass - that figure includes the upper stage's
    /// tanks, and reserving against it would stage almost immediately. The cached model
    /// is the unreserved one (ApplyAscentReserve only ever touches the ascent's copy),
    /// which is what makes this subtraction mean "propellant left in this stage".
    /// </summary>
    private static bool ShouldStageForReserve(Vehicle vehicle)
    {
        if (!_s.Running || _s.ReserveStaged || !_s.ReserveArmed || !(_s.ReserveKg > 0.0))
            return false;

        // NOT ON THE PAD. A reserve bigger than the stage can give would otherwise stage
        // the vehicle where it stands, and a booster dropped at zero altitude is a
        // sillier failure than the one being guarded against. Past the vertical rise the
        // test means what it says.
        if (_s.Phase == AscentPhase.Vertical)
            return false;

        UpfgVehicle model = _s.StageModel;
        if (model == null || model.Stages.Count < 2)
            return false;

        double remaining = vehicle.TotalMass - model.Stages[0].MassDry;
        return remaining <= _s.ReserveKg;
    }

    // --- the hand-over to boostback -----------------------------------------
    //
    // ONE PENDING HAND-OVER AT A TIME, in statics rather than per-vehicle state, and
    // that is a real limitation rather than an oversight. The record has to be read
    // from a vehicle that does not exist yet and has no state of its own, so it cannot
    // live on either party; and a pair of side boosters separating together would need
    // one record each. A returning first stage is one vehicle, which is the case this
    // is for. It expires either way, so a hand-over that finds nothing does not linger.
    private static readonly HashSet<uint> _handoverParts = new HashSet<uint>();
    private static double _handoverSiteLat, _handoverSiteLon;
    private static double _handoverExpiry = double.NegativeInfinity;

    /// <summary>How long a hand-over waits for its booster to show up, s. The split
    /// happens inside the same activation, so this only has to survive a frame or two;
    /// it is generous because the cost of waiting is nothing and the cost of expiring
    /// early is a booster nobody flies.</summary>
    private const double HandoverWindowS = 30.0;

    /// <summary>
    /// Record what the imminent separation is about to throw overboard, so the vehicle
    /// it becomes can be recognised and handed the landing site.
    /// </summary>
    private static void ArmBoosterHandover(Vehicle vehicle, SequenceList sequenceList,
                                           double now)
    {
        _handoverParts.Clear();
        _s.ReserveStaged = true;

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
            return;

        _stagingDropped.Clear();
        Part detached = null;
        ReadOnlySpan<Part> parts = next.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            Span<Decoupler> decouplers = parts[i].SubtreeModules.Get<Decoupler>();
            for (int j = 0; j < decouplers.Length; j++)
            {
                Part root = DetachedRoot(decouplers[j]);
                if (root == null)
                    continue;
                detached ??= root;       // FIRST decoupler's root - see below
                CollectSubtree(root, _stagingDropped);
            }
        }
        foreach (Part part in _stagingDropped)
            _handoverParts.Add(part.InstanceId);

        if (_handoverParts.Count == 0 || detached == null)
            return;

        // The site travels with the booster, and it is THIS STAGE'S site when one was
        // set for it - which is the point of setting them independently. The vehicle's
        // own site is the fallback, because a booster with no state of its own would
        // otherwise separate with no target at all.
        //
        // The id is taken with the SAME RULE RefreshReturnableStages uses - the first
        // decoupler's detached root - and that has to stay true, because the two only
        // agree by construction. Deriving it here instead by scanning the dropped set
        // for a part whose parent stayed behind looks equivalent and is not: a sequence
        // with several decouplers has several such roots, and a HashSet does not
        // promise which comes out first. The lookup would then miss and fall back to
        // the vehicle site without a word.
        ResolveStageTarget(detached.InstanceId, out _handoverSiteLat, out _handoverSiteLon);
        _handoverExpiry = now + HandoverWindowS;
    }

    /// <summary>
    /// A vehicle the sweep has never seen before: is it the booster we just staged?
    /// If so, give it state, the landing site, and a running boostback.
    ///
    /// This is where the hand-over has to happen. A separated booster is a NEW Vehicle
    /// object with no entry in the state table, and the sweep drops unknown unfocused
    /// craft on the floor - which is correct for every other vehicle in the universe and
    /// exactly wrong for this one. Matching on the parts recorded before the split is
    /// what tells the two apart.
    /// </summary>
    private static bool TryAdoptBooster(Vehicle vehicle)
    {
        if (_handoverParts.Count == 0 || SimNow() > _handoverExpiry)
            return false;

        PartTree tree = vehicle?.Parts;
        if (tree == null)
            return false;

        bool mine = false;
        ReadOnlySpan<Part> parts = tree.Parts;
        for (int i = 0; i < parts.Length && !mine; i++)
            mine = _handoverParts.Contains(parts[i].InstanceId);
        if (!mine)
            return false;

        // Consumed on the first match: the record describes one separation, and a second
        // vehicle claiming it would be the upper stage or debris taking the booster's
        // guidance with it.
        _handoverParts.Clear();
        _handoverExpiry = double.NegativeInfinity;

        _s = VehicleAutopilotState.For(vehicle);
        _s.SiteLatDeg = _handoverSiteLat;
        _s.SiteLonDeg = _handoverSiteLon;

        // NOT engaged here, deliberately. This runs on the frame the split happened,
        // which is the frame the part tree is least settled - and ExecuteBoostback needs
        // an aero surrogate fitted to a bounding box that may not exist yet. It refuses
        // rather than throwing, and refusing here would leave a booster that had been
        // adopted and would never be flown, with the hand-over record already consumed.
        // So the engage is retried from the sweep until it takes.
        _s.HandoverPendingUntil = SimNow() + HandoverWindowS;
        return true;
    }

    /// <summary>
    /// Engage boostback on an adopted booster, retrying until the part tree has settled
    /// enough for the aero sweep to fit a surrogate to it. Gives up at the window rather
    /// than retrying a fault forever.
    /// </summary>
    private static void StepBoosterHandover(Vehicle vehicle)
    {
        if (!(_s.HandoverPendingUntil > double.NegativeInfinity))
            return;

        if (BoostbackLive)
        {
            _s.HandoverPendingUntil = double.NegativeInfinity;
            return;
        }

        if (SimNow() > _s.HandoverPendingUntil)
        {
            _s.HandoverPendingUntil = double.NegativeInfinity;
            _s.BoostbackStatus = "Hand-over failed: " + (_s.AeroError.Length > 0
                ? _s.AeroError : "no aero surrogate for the separated booster.");
            return;
        }

        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent != null)
            ExecuteBoostback(vehicle, orbit, parent);
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
            // The game's own total for the same recompute, latched for the panel to
            // check our stage list against. Both are "from here on" — its simulated
            // mole masses are re-seeded from the live tanks every recompute — so a
            // disagreement means the adapter is reading the sequence list wrongly,
            // which is exactly the failure that is invisible in a plausible-looking
            // stage table. TotalDeltaV is a Volatile.Read of a float, so the draw
            // thread can have this even though the Lists behind it can tear.
            _s.StageModelKsaDv = performance.TotalDeltaV;

            // The reserve rides the same tick: both of its inputs - the stage model and
            // the part tree the separation walk reads - only change when staging does,
            // and StageModelDirty is already set exactly then. So does the returnable
            // stage list, which reads the same part tree for the same reason.
            RefreshReserve(vehicle);
            RefreshReturnableStages(vehicle);
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
                    Seq = s.Seq, Engines = s.Engines,
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

    // --- Ascent propellant reserve -----------------------------------------
    //
    // WHAT THIS IS FOR. A booster that flies itself home cannot spend every drop of its
    // propellant reaching staging velocity: boostback, entry and landing all have to
    // come out of the same tanks. So the ascent stops the first stage early, leaving a
    // dV reserve behind, and hands the booster over with a landing site.
    //
    // TWO LEVERS, AND BOTH ARE NEEDED. UPFG never commands staging - it plans around
    // stage boundaries, but the event itself is fired by the tanks running dry (see
    // AutoSequence). So the reserve raises the stage's MassDry, which is what makes
    // UPFG believe the stage is shorter and hand the difference to the upper stage; and
    // it adds a staging CUE, which is what actually cuts the burn. Doing only the first
    // makes UPFG plan a shorter stage and then burn straight through the reserve
    // anyway.

    /// <summary>
    /// The propellant a dV reserve costs, kg, and the booster dry mass it is measured
    /// against. Zero when there is no separation to reserve for.
    ///
    /// THE ARITHMETIC IS THE POINT. For a reserve dv left in a booster of dry mass
    /// m_dry, the rocket equation gives ve*ln((m_dry + m_p)/m_dry) = dv, so
    ///
    ///     m_p = m_dry * (exp(dv/ve) - 1)
    ///
    /// and the UPPER STAGE IS NOT IN IT. That is not an approximation, it is the whole
    /// reason the knob is a dV: the propellant only ever has to lift the booster, so
    /// sizing it against the mass at staging - booster plus upper stage, the number
    /// that is actually to hand - over-reserves by the ratio of the two. On a 20 t
    /// booster under a 40 t upper stage, a 500 m/s reserve is 3.6 t; measured against
    /// the 60 t stack it is 10.9 t, which is 7.3 t the ascent gave up for nothing and
    /// 1303 m/s where 500 was asked for.
    ///
    /// m_dry is read straight off the stage model: the mass that goes overboard at the
    /// first real jettison is the booster, and Coalesce has already used exactly that
    /// discontinuity to decide the boundary is a boundary (KsaVehicleAdapter).
    ///
    /// ve is the stage's VACUUM exhaust velocity, which is what the model carries.
    /// Staging happens high enough that the boostback burn is very nearly a vacuum burn,
    /// so this is close - but it is optimistic, not conservative, and a reserve flown
    /// low would deliver less dV than the knob says.
    /// </summary>
    private static double ReservePropellantKg(UpfgVehicle model, double dvMs,
                                              out double boosterDryKg)
    {
        boosterDryKg = 0.0;
        if (model == null || dvMs <= 0.0 || model.Stages.Count < 2)
            return 0.0;

        UpfgStage first = model.Stages[0], next = model.Stages[1];

        // A real jettison, not a g-limit split or an engine cutting out: those leave
        // the mass continuous and are not a booster going anywhere.
        double dropped = first.MassDry - next.MassTotal;
        if (dropped <= 1e-4 * Math.Max(first.MassTotal, 1.0))
            return 0.0;

        double ve = first.Isp * 9.80665;
        if (!(ve > 0.0))
            return 0.0;

        boosterDryKg = dropped;

        // DELIBERATELY UNCAPPED. Capping it against the propellant still in the stage
        // looks prudent and is in fact self-defeating: MassTotal is re-seeded from the
        // live tanks every refresh, so "what is left" shrinks as the ascent burns, and a
        // reserve capped at a fraction of it shrinks with it - which makes the staging
        // test (propellant left <= reserve) one the burn can never reach. The cue has to
        // compare a shrinking quantity against a FIXED one, so this is the fixed one.
        //
        // A reserve larger than the stage can give is a question the vehicle answers by
        // staging as soon as guidance is flying, and the readout says so before it does.
        return dropped * (Math.Exp(dvMs / ve) - 1.0);
    }

    // Scratch for the separation walk. Same contract as _stagingDropped: filled and
    // consumed inside one call, never read across calls.
    private static readonly HashSet<Part> _separationDrops = new HashSet<Part>();

    /// <summary>
    /// True when the next separation drops every engine currently producing thrust.
    ///
    /// THIS IS THE GUARD THAT KEEPS THE RESERVE OFF A STRAP-ON STACK. "The next thing
    /// that separates" is not the same as "the booster": while solids burn beside a
    /// core, the next separation is the casings, and reserving against that would end
    /// the whole first stage early to leave propellant in something about to be thrown
    /// away. Requiring the separation to take ALL the live engines with it says the
    /// thing that leaves is the thing doing the flying, which is what a boostback needs
    /// to be true. Once the strap-ons are gone it becomes true on its own.
    /// </summary>
    private static bool NextSeparationDropsAllEngines(Vehicle vehicle, out string why)
    {
        why = "";
        PartTree tree = vehicle?.Parts;
        SequenceList sequenceList = tree?.SequenceList;
        if (tree == null || sequenceList == null)
        {
            why = "no part tree";
            return false;
        }

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
        {
            why = "no staging sequence left to fire";
            return false;
        }

        _separationDrops.Clear();
        ReadOnlySpan<Part> parts = next.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            Span<Decoupler> decouplers = parts[i].SubtreeModules.Get<Decoupler>();
            for (int j = 0; j < decouplers.Length; j++)
            {
                Part root = DetachedRoot(decouplers[j]);
                if (root != null)
                    CollectSubtree(root, _separationDrops);
            }
        }
        if (_separationDrops.Count == 0)
        {
            // An ignition-only sequence, say. Not a fault - the reserve simply has
            // nothing to stage into yet, and this becomes true further down the list.
            why = "the next sequence separates nothing";
            return false;
        }

        if (!ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
                .TryGetFrom(tree.States, out var engineStates))
        {
            why = "no engine states";
            return false;
        }

        bool anyLit = false;
        foreach (var engine in engineStates.ModulesAndStates)
        {
            if (!engine.Module.IsActive || !engine.State.IsPropellantAvailable)
                continue;
            anyLit = true;
            if (!_separationDrops.Contains(engine.Module.Parent.FullPart))
            {
                why = "the next separation leaves an engine burning - not a booster";
                return false;
            }
        }
        if (!anyLit)
            why = "nothing is burning";
        return anyLit;
    }

    /// <summary>
    /// Size the reserve and decide whether it is armed. Runs with the stage model, on
    /// its 4 Hz tick, because both inputs - the model and the part tree - only change
    /// when staging does, and StageModelDirty already marks that.
    /// </summary>
    private static void RefreshReserve(Vehicle vehicle)
    {
        _s.ReserveKg = 0.0;
        _s.ReserveBoosterDryKg = 0.0;
        _s.ReserveArmed = false;

        if (!(_s.AscentReserveDvMs > 0.0))
        {
            _s.ReserveNote = "";
            return;
        }

        // ONCE ONLY, and this is not just belt-and-braces. After the booster is gone the
        // next separation is a perfectly good one - an upper stage over a payload, say -
        // and re-arming there would reserve propellant in a stage that is never coming
        // back. The reserve belongs to the first separation of an ascent.
        if (_s.ReserveStaged)
        {
            _s.ReserveNote = "already staged";
            return;
        }

        double kg = ReservePropellantKg(_s.StageModel, _s.AscentReserveDvMs,
                                        out double boosterDry);
        if (!(kg > 0.0))
        {
            _s.ReserveNote = "no separation in the stage model to reserve for";
            return;
        }
        if (!NextSeparationDropsAllEngines(vehicle, out string why))
        {
            _s.ReserveNote = why;
            return;
        }

        _s.ReserveKg = kg;
        _s.ReserveBoosterDryKg = boosterDry;
        _s.ReserveArmed = true;

        // Not a refusal - the vehicle will stage the moment it is flying, which is the
        // right answer to "leave more behind than this stage holds" - but it is not what
        // anyone means to ask for, so it is said out loud rather than discovered.
        double left = Math.Max(_s.StageModel.Stages[0].MassTotal
                             - _s.StageModel.Stages[0].MassDry, 0.0);
        _s.ReserveNote = kg >= left
            ? $"reserve is more than the {left / 1000.0:F1} t left in this stage"
            : "";
    }

    /// <summary>
    /// LEVER ONE: tell UPFG the stage is shorter than it is.
    ///
    /// Raising MassDry is the whole model change, because every other stage quantity is
    /// derived from it - burnTimes[0] = (MassTotal - MassDry)/massflow, and through it
    /// charTimes, tgoi, the thrust integrals, and L, the dV believed to be aboard.
    /// A smaller L is what makes UPFG hand the difference to the upper stage rather
    /// than quietly plan to fly the reserve.
    ///
    /// Applied to the COPY the ascent step gets, never to the cached snapshot: the
    /// stage table and the landing flows read the same cache and should see the vehicle
    /// as it is.
    /// </summary>
    private static void ApplyAscentReserve(UpfgVehicle live, double reserveKg)
    {
        if (live == null || live.Stages.Count == 0 || !(reserveKg > 0.0))
            return;

        UpfgStage s0 = live.Stages[0];

        // burnTimes goes NEGATIVE if MassDry passes MassTotal, and a negative burn time
        // propagates into tgo without complaint. Once the live mass is already inside
        // the reserve there is nothing left to plan on this stage anyway - the staging
        // cue is about to fire - so the floor here is a guard, not a policy.
        double floor = s0.MassTotal - 0.02 * (s0.MassTotal - s0.MassDry);
        s0.MassDry = Math.Min(s0.MassDry + reserveKg, floor);
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
                    Seq = s.Seq, Engines = s.Engines,
                });
                limited.Add(new UpfgStage
                {
                    Mode = 2, Thrust = s.Thrust, Isp = s.Isp, GLim = gLim,
                    MassTotal = massAtLimit, MassDry = s.MassDry,
                    Seq = s.Seq, Engines = s.Engines,
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
        else if (!TryAdoptBooster(vehicle))
            return;

        // SWITCHED OFF: hand this vehicle back and stop touching it.
        //
        // Deliberately AFTER the state lookup and BEFORE anything that steps guidance.
        // Simply not running would be the wrong kind of off - a craft mid-descent would
        // keep the engine lit, keep the TVC override driving the nozzles, and keep the
        // attitude hold the mod took out, with nothing left running to undo any of it.
        // Off has to mean handed back, and the hand-back has to happen here because
        // this prefix is the only place those writes reach the sim.
        if (!ModActive)
        {
            if (!_s.HandedBack)
            {
                HandBackVehicle(vehicle);
                _s.HandedBack = true;
            }
            // One more flush in case a reset was already queued when the switch flipped.
            else if (_s.FcResetPending)
            {
                ApplyPendingFcReset(vehicle);
            }
            return;
        }
        _s.HandedBack = false;

        bool sixDof = _s.Active || _s.EngagePending;
        bool landingActive = _s.LandingPhase != LandingPhase.Idle && _s.LandingPhase != LandingPhase.Done;
        // LaunchArmed counts as flying: a vehicle waiting for its launch window has a
        // window to re-derive and an EXECUTE to fire (StepLaunchWindow), and without
        // it here the step that does both was skipped for exactly the state that
        // needs it.
        // A booster adopted at separation counts as flying before boostback engages:
        // the engage is retried from here, and without this the sweep would drop the
        // vehicle on the floor between adoption and the retry that starts it.
        bool handingOver = _s.HandoverPendingUntil > SimNow();
        bool flying = sixDof || _s.Running || landingActive || BoostbackLive || _s.WasEngaged
                   || _s.LandingCutPending || _s.LaunchArmed || handingOver;

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

        // After the stage model, because both want a part tree that has finished
        // settling after a separation and this is the first point in the frame where
        // that is true.
        StepBoosterHandover(vehicle);

        // What it would cost each returnable stage to come home from here. Housekeeping
        // rather than guidance, and deliberately so: the whole value of the number is
        // watching it through the climb, which means it has to be solved before anything
        // has been staged and whether or not the tab is open. It is throttled to 1 Hz
        // and shares one Jacobian across every stage - see Guidance/ReturnableStages.cs.
        // Gated on a stage actually having a TARGET, not merely on one being returnable.
        // The surrogate is a 72-azimuth sweep of the game's own CdA and it is refitted
        // at every staging; paying for it on a craft nobody has asked to bring back
        // would be a launch-time cost for a number nothing displays.
        if (AnyStageTargeted() && vehicle.Orbit?.Parent != null)
        {
            EnsureBoostbackAero(vehicle, vehicle.Orbit.Parent);
            UpdateReturnDv(vehicle, vehicle.Orbit, vehicle.Orbit.Parent);
        }

        // Likewise the flown-trajectory trace: sampled off the simulation rather than
        // the frame rate, and recorded whether or not guidance is running so the track
        // is already there when the overlay is switched on — losing it the moment
        // guidance engages would blank the overlay during exactly the descent worth
        // watching.
        RecordTrace(vehicle, vehicle.Orbit);

        // Likewise the launch window: tracked, and FIRED, from here rather than from
        // the panel. It is housekeeping in the same sense the two above are — it has
        // to run for a focused vehicle that is not flying yet, which is precisely the
        // state an armed launch sits in. See StepLaunchWindow.
        StepLaunchWindow(vehicle, vehicle.Orbit, vehicle.Orbit.Parent,
                         vehicle.Orbit.Parent.MeanRadius);

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
        StepBoostback(vehicle, orbit, stepParent);

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
        // Every live boostback phase steers, including the settling burn (which holds a
        // latched attitude) and the entry hold (which tracks surface retrograde
        // indefinitely) — so unlike the landing machine there is no sub-phase here that
        // wants the vehicle back.
        bool boostbackGuides = BoostbackLive;
        bool shouldCommand = _s.Engage && (_s.Running || landingGuides || boostbackGuides)
                          && _s.HasCommand;

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
            else if (boostbackGuides)
            {
                // The phase decides; the step recorded it. Deliberately NOT paired with
                // AutoSequence: the machine cuts the engine at boostback cutoff and
                // coasts to entry from there, and the auto-stager reads "no thrust" as
                // a cue to fire the next sequence — so it would work its way down a
                // returning booster's staging list one activation per second, and the
                // sequences left on a first stage are the ones that separate it.
                ref ManualControlInputs inputs = ref ManualInputs(vehicle);
                inputs.EngineOn = _s.BoostbackEngineOn;
                inputs.EngineThrottle = (float)_s.BoostbackThrottle;
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
            // An ascent references its roll to its target plane (see SteerBody2Cci and
            // AscentRollRef); a landing keeps the stock position-derived reference,
            // which is well conditioned there because the thrust axis is fighting the
            // velocity, not lying along the position vector.
            double3? rollRef = _s.Running ? AscentRollRef(vehicle, _s.CommandDir) : null;
            CommandAttitude(vehicle, vehicle.Orbit.Parent, _s.CommandDir,
                            fullEngage: !_s.WasEngaged, rollRef: rollRef);

            // AND THE RATE THAT ATTITUDE IS TURNING AT, published in the same breath
            // so the pair can never describe different instants. The ascent produces
            // one from the steering law's own implied turning rate, and the boostback
            // from its slew and its moving targets; a landing publishes zero, which is
            // exactly what the flight computer assumed before this existed.
            KsaAttitudeRate.Set(vehicle,
                _s.Running || boostbackGuides ? _s.CommandRate : default);
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
        // Before the swap, while the OLD VehicleConfig this vehicle's rate is keyed on
        // is still reachable. The new flight computer brings a new config and so a new
        // (unengaged) slot regardless, but a feedforward is a standing instruction to
        // keep rotating and is not a thing to leave lying around on the strength of an
        // object becoming unreachable.
        KsaAttitudeRate.Clear(vehicle);

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

    /// <summary>
    /// Whether the mod is doing anything at all. Cleared from the game's menu bar - see
    /// Mod.OnDrawProgramMenus - so a player who is not using guidance gets their screen
    /// and their flight computer back completely.
    ///
    /// Read from the sim thread (ApplyAutopilot) and written from the draw, hence
    /// volatile: without it a release build is free to hoist the test out of the
    /// per-vehicle loop and keep servicing craft after the switch has flipped.
    /// </summary>
    internal static volatile bool ModActive = true;

    /// <summary>
    /// Give one vehicle back to the player, completely. Runs from the PrepareWorker
    /// prefix, which is the only context where the flight-computer and manual-input
    /// writes below actually reach the simulation.
    ///
    /// Ordered so that nothing is left half-owned: the 6-DOF worker is stopped and its
    /// TVC override released BEFORE the flight computer is reset, because the override
    /// lives outside the flight computer and a reset would otherwise leave it driving
    /// the nozzles of a craft the player now believes they control.
    /// </summary>
    private static void HandBackVehicle(Vehicle vehicle)
    {
        try
        {
            if (_s.Active || _s.EngagePending || _s.Converging)
                Disengage6Dof(vehicle);

            // Belt and braces: Disengage6Dof does both of these, but it is skipped
            // entirely when 6-DOF was never engaged and the gimbal tab could still
            // have left an override running.
            _s.GimbalMode = 0;
            KsaGimbalControl.Disengage(vehicle);

            // Clears Running, LandingPhase, AutoLaunch and HasCommand, and queues the
            // flight-computer reset that actually releases attitude and cuts the engine.
            ResetFlightComputer();
            ApplyPendingFcReset(vehicle);

            // The reset's status line is for a player who pressed a button. This one
            // was not asked for by anyone looking at a panel, and the panel is gone.
            _s.Status = "";
        }
        catch
        {
            // A vehicle that throws on the way out must not take the sim step with it,
            // and must not be retried forever - HandedBack is set by the caller either
            // way, so a failure here costs this craft its clean release and nothing else.
        }
    }

    private static void ResetFlightComputer()
    {
        _s.Running = false;
        _s.LandingPhase = LandingPhase.Idle;
        _s.BoostbackPhase = BoostbackPhase.Idle;
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
    // rollRef, when supplied, replaces the ROLL REFERENCE that ComputeBurnBody2Cci
    // derives from the position vector — see SteerBody2Cci for why an ascent must not
    // use the stock one.
    //
    // fullEngage=true additionally switches the FC into Custom/Auto tracking — done
    // once on engage, exactly like clicking "Apply Euler Target" in the attitude tab.
    private static void CommandAttitude(Vehicle vehicle, IParentBody parent, double3 dir,
                                        bool fullEngage, double3? rollRef = null)
    {
        double3 r = vehicle.Orbit.StateVectors.PositionCci;

        float3 posDir = float3.Pack(double3.Normalize(r));
        float3 steerDir = float3.Pack(double3.Normalize(dir));

        doubleQuat desiredBody2Cci = rollRef.HasValue
            ? SteerBody2Cci(double3.Normalize(dir), rollRef.Value)
            : BurnTarget.ComputeBurnBody2Cci(posDir, steerDir);
        doubleQuat frame2Cci = VehicleReferenceFrameEx.GetEclBody2Cci(parent.GetCce2Cci());

        // Solve Concatenate(value, frame2Cci) == desired  ->  value = Concatenate(desired, inverse(frame2Cci)).
        doubleQuat value = doubleQuat.Concatenate(desiredBody2Cci, doubleQuat.Inverse(frame2Cci));
        double3 euler = value.ToRollYawPitchRadians();

        var fc = vehicle.FlightComputer;
        fc.CustomAttitudeTarget = euler;

        // WHETHER THE FLIGHT COMPUTER LOOKS AT THE ROLL WE JUST COMMANDED.
        // UpdateAttitudeTrackError computes a roll term only when RollMode is not
        // Decoupled; decoupled is the default and discards the target's roll entirely,
        // tracking pointing alone. That is the right behaviour for an ascent — roll is
        // the axis a launch vehicle has least authority about and nothing in the
        // trajectory needs a particular one — so it stays decoupled unless the roll is
        // being forced deliberately.
        //
        // Written every step, not only on engagement: the box can be ticked mid-ascent,
        // and un-ticking it has to give the roll freedom back.
        if (_s.Running)
            fc.RollMode = _s.ForceRoll
                ? FlightComputerRollMode.Up
                : FlightComputerRollMode.Decoupled;

        if (fullEngage)
        {
            fc.AttitudeFrame = VehicleReferenceFrame.EclBody;
            fc.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
        }
    }

    /// <summary>
    /// The ascent's roll reference: the target orbit plane, turned by the roll the
    /// vehicle ALREADY HAD when guidance engaged.
    ///
    /// The plane on its own is what makes the reference continuous — it is
    /// perpendicular to the steering from the pad to cutoff, where cross(steer,
    /// position) is degenerate for the whole vertical rise (see SteerBody2Cci). But
    /// the plane on its own also picks a particular roll, and a vehicle sitting on the
    /// pad at some other one is then ordered to spin about its long axis the moment it
    /// can move. Nothing about an ascent needs a specific roll, so this measures the
    /// vehicle's own at engagement and holds THAT, fixed relative to the plane: no
    /// roll is ever commanded, and there is still nothing to snap.
    /// </summary>
    private static double3 AscentRollRef(Vehicle vehicle, double3 steer)
    {
        if (steer.Length() < 0.5)
            return -AscentPlaneNormal();

        double3 x = double3.Normalize(steer);

        // The plane-referenced frame perpendicular to the thrust axis. MINUS the
        // normal, so that with the steering along the velocity this is cross(v, r) —
        // the same direction the stock construction produces where it works.
        double3 baseRef = -AscentPlaneNormal();
        double3 yRef = baseRef - double3.Dot(baseRef, x) * x;
        if (yRef.Length() < 1e-9)
            return baseRef;                     // SteerBody2Cci substitutes for this
        yRef = double3.Normalize(yRef);
        double3 zRef = double3.Cross(x, yRef);

        // Forced: the angle the user asked for, in this same frame. Read live rather
        // than latched, so turning the dial moves the vehicle.
        //
        // The latch is deliberately NOT taken while forcing. Un-ticking the box has to
        // leave the vehicle holding the roll it is in AT THAT MOMENT, and latching an
        // engagement-time measurement would instead roll it back to lift-off.
        if (_s.ForceRoll)
        {
            _s.RollLatched = false;
            double forced = UpfgTarget.DegToRad(_s.ForceRollDeg);
            return Math.Cos(forced) * yRef + Math.Sin(forced) * zRef;
        }

        // Latched once per engagement, from the vehicle's own body Y — the axis
        // ComputeBurnBody2Cci puts the roll reference on.
        if (!_s.RollLatched)
        {
            double3 bodyY = new double3(0, 1, 0).Transform(KsaFrameBridge.BodyToCci(vehicle));
            double c = double3.Dot(bodyY, yRef), s = double3.Dot(bodyY, zRef);
            _s.RollOffset = (Math.Abs(c) > 1e-9 || Math.Abs(s) > 1e-9) ? Math.Atan2(s, c) : 0.0;
            _s.RollLatched = true;
        }

        return Math.Cos(_s.RollOffset) * yRef + Math.Sin(_s.RollOffset) * zRef;
    }

    /// <summary>
    /// The body→CCI orientation that points thrust along <paramref name="steerDir"/>,
    /// built exactly as KSA's BurnTarget.ComputeBurnBody2Cci builds it (body X along
    /// thrust, body Y the roll reference, body Z their cross) but with the roll
    /// reference supplied instead of derived from the position vector.
    ///
    /// WHY NOT THE STOCK ONE, ON AN ASCENT. ComputeBurnBody2Cci takes the roll
    /// reference from cross(steer, position) — and on an ascent those two vectors are
    /// THE SAME VECTOR at lift-off and within a fraction of a degree of it through the
    /// start of the pitch-over. The cross product normalises to zero, the stock code
    /// substitutes an arbitrary orthogonal direction, and then, the moment the pitch
    /// program makes the cross product representable in float, the reference SNAPS
    /// from that substitute to the plane normal. The thrust direction never moved; the
    /// commanded roll jumped by whatever the angle between them happened to be, and
    /// the flight computer chased it with everything it had.
    ///
    /// The reference passed in by the ascent is the target plane normal, which is
    /// perpendicular to the steering all the way to orbit and turns as slowly as the
    /// plane does — i.e. the vehicle flies wings-level to its own orbital plane from
    /// the pad to cutoff, with no discontinuity anywhere in between.
    /// </summary>
    private static doubleQuat SteerBody2Cci(double3 steerDir, double3 rollRef)
    {
        double3 x = double3.Normalize(steerDir);

        // Component of the reference perpendicular to the thrust axis. Degenerate only
        // if the two are parallel, which the plane normal never is on an ascent.
        double3 y = rollRef - double3.Dot(rollRef, x) * x;
        y = y.Length() > 1e-9
            ? double3.Normalize(y)
            : double3.Normalize(double3.Cross(x, Math.Abs(x.Z) < 0.9 ? new double3(0, 0, 1) : new double3(1, 0, 0)));

        double3 z = double3.Cross(x, y);
        return doubleQuat.CreateFromRotationMatrix(new double4x4(
            x.X, x.Y, x.Z, 0.0,
            y.X, y.Y, y.Z, 0.0,
            z.X, z.Y, z.Z, 0.0,
            0.0, 0.0, 0.0, 1.0));
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

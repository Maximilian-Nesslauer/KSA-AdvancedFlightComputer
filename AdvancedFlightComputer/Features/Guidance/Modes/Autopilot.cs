#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Core;
using Brutal.Numerics;
using HarmonyLib;
using KSA;
using AdvancedFlightComputer.Features.RcsTorque;
using AdvancedFlightComputer.Features.AutoStage;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using AscentRecoveryResult = AdvancedFlightComputer.Guidance.Scvx.Ascent.AscentRecoveryResult;

// Shared plumbing used by every flow: the per-vehicle step-and-apply entry point (ApplyAutopilot, called from the Vehicle.PrepareWorker Harmony prefix for each craft in turn), the commanded attitude, auto-staging, the UPFG vehicle builder, the warp-confirmation prompt, and small math helpers.
//
// The state these act on is NOT here - it lives on the vehicle being serviced, via the ambient _s pointer (see VehicleAutopilotState). What remains static in this partial class belongs to the console or to the process, not to a craft.
public static partial class GuidanceWindow
{

    // Guidance must survive transient bad frames (the staging frame reports zero thrust while the next engine ignites; the part tree can be mid-mutation). A single failed step skips that frame and keeps the last solution; only a long unbroken run of failures stops guidance for good.
    private const int MaxFailStreak = 600; // ~10 s of consecutive bad frames

    // UPFG state is stored in VehicleAutopilotState.Upfg for each vehicle because the solver warm-starts from its previous state.

    // The staged vehicle model is rebuilt from KSA's sequence list every guidance step. Stage 0 carries the present masses, so burn times are time remaining and manual staging is reflected automatically.

    // Auto engines & staging: while armed (and the autopilot is engaged), the mod ignites the first sequence, fires the next sequence whenever the current powered phase has no thrust or is about to flame out, and shuts the engines down at the terminal cutoff. Throttle is forced to 1 while burning.
    private const double SequenceCooldown = 1.0;   // s between auto activations

    // Vehicle-wide acceleration limit: any part of any burn that would exceed this becomes a constant-acceleration (Mode 2) segment in the UPFG stage list, and the auto-throttle follows UPFG's throttle command to hold it.

    private const double WarpLeadTime = 10.0;  // end auto-warp this many s early

    // Forced roll stays decoupled below this altitude AGL, m: near the pad the vehicle has the least control authority and the least room to recover from a roll it didn't lift off with, so a forced roll only engages once it has some clearance.
    private const double ForceRollMinAltM = 200.0;

    // The game's own ignite and shutdown actions write EngineOn in this private field.
    private static AccessTools.FieldRef<Vehicle, ManualControlInputs> ManualInputs =>
        GameReflection.Vehicle_manualControlInputsRef!;

    // The mod's clock: elapsed sim time in seconds. Used for everything time-based (turn ramp, staging cooldown, cutoff) so behavior is correct under time warp, unlike the wall-clock-ish player time.
    private static double SimNow() => Universe.GetElapsedSeconds();

    // Nothing in the mod starts a time warp on its own: flows that want one call RequestWarp, and the console head asks the user first (DrawHeadWarp). This prevents warping into a planet because a predicted burn point happened to be far away.
    private static bool _warpPromptActive;
    private static string _warpLabel = "";
    private static double _warpTargetSimSec;
    private static string _warpDeclinedLabel = "";

    private static void RequestWarp(double targetSimSec, string label)
    {
        if (_warpDeclinedLabel == label)
            return;   // the user said no to this one - don't nag every frame
        _warpPromptActive = true;
        _warpLabel = label;
        _warpTargetSimSec = targetSimSec;
    }

    /// <summary>The label an armed launch asks to warp to its window under - how the prompt knows to offer Launch now beside the warp.</summary>
    private const string LaunchWindowWarpLabel = "LAUNCH WINDOW";

    // Staging is the AutoStage feature's job: its per-vehicle machine stages on burnout and drops spent boosters, with the delays, the crossfeed refusal and the control-module guard. Guidance arms it for the craft it flies and adds the two cues only a flight plan knows: a cold ignition, when nothing produces thrust, and the reserve boundary of a returning booster. A request while a staging is still in flight is refused by the detector, and the cooldown keeps guidance from asking every step.
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

        if (!SharedVehicleHooks.AutoStageEnabled)
        {
            // The block did not load, or the standalone AutoStage mod holds it off.
            _s.StagingActive = false;
            _s.WarnStatus("AFC CANNOT STAGE: AUTOSTAGE OFF. SEE LOG.");
            return;
        }

        // Armed once per flight, on the first step that needs it, unless the player armed the craft already. After that the switch is the player's, so a disarm holds for the rest of the flight and a re-arm is used as it is.
        if (!_s.StagingArmChecked)
        {
            _s.StagingArmChecked = true;
            if (!StagingDetector.IsArmed(vehicle))
            {
                StagingDetector.Arm(vehicle, true);
                _s.ArmedStaging = true;
                GuidanceLog.Debug(vehicle, "armed AutoStage for this flight.");
            }
        }
        if (!StagingDetector.IsArmed(vehicle))
        {
            // What guidance armed is off now, so the release has nothing of its own to switch off.
            if (_s.ArmedStaging)
                GuidanceLog.Debug(vehicle, "AutoStage was switched off by the player, staging cues stop.");
            _s.ArmedStaging = false;
            _s.StagingActive = false;
            _s.WarnStatus("AUTOSTAGE OFF.");
            return;
        }

        double now = SimNow();
        int activations = StagingDetector.ActivationsOf(vehicle);
        if (activations != _s.SeenStagings)
        {
            // A requested or triggered row fired on this craft: the stage list changed under us.
            GuidanceLog.Debug(vehicle, $"AutoStage fired a row ({_s.SeenStagings}->{activations} activations), the stage model is rebuilt on this step.");
            _s.SeenStagings = activations;
            _s.StageModelDirty = true;
            _s.LastSequenceTime = now;
        }

        if (StagingDetector.IsHeldForControl(vehicle))
        {
            if (_s.StagingActive)
                GuidanceLog.Debug(vehicle, "staging held, the next sequence would separate the control module.");
            _s.StagingActive = false;
            _s.WarnStatus("STAGE HELD: NEXT SEPARATION DROPS CONTROL MODULE.");
            return;
        }

        bool thrustOn = vehicle.IsAnyEngineActive() && vehicle.IsAnyEnginePropellantAvailable();

        // The reserve's own cue fires while the stage could still burn, because the propellant left in it is spoken for. UPFG plans stage boundaries but never commands one.
        bool reserveDone = ShouldStageForReserve(vehicle);
        // A drone-ship booster separates where its live impact reaches its ship, if that comes before its reserve (DroneShipSeparation).
        bool shipDone = !reserveDone && ShouldStageForDroneShip(vehicle);
        reserveDone |= shipDone;
        UpdateSeparationCountdown(vehicle, thrustOn);

        _s.StagingActive = !thrustOn || reserveDone;
        if (!_s.StagingActive || now - _s.LastSequenceTime < SequenceCooldown)
            return;

        // A staging still in flight refuses the request, and the cue asks again after the cooldown. Only a queued request records the reserve as staged, otherwise a booster jettison still in flight would cost the reserve its only staging. A queued request can still stage nothing when the machine holds a row that would separate the control module, and the reserve then stays held with that row.
        _s.LastSequenceTime = now;
        StagingDetector.StagingRequest request = StagingDetector.RequestStaging(vehicle);
        GuidanceLog.Debug(vehicle, $"staging requested for {(shipDone ? "the drone ship" : reserveDone ? "the booster reserve" : "no thrust")}: {request}.");
        if (request != StagingDetector.StagingRequest.Queued)
            return;

        // Each reserve belongs to one separation. Handover is armed when the decouplers fire.
        if (reserveDone)
        {
            _s.ReserveStagedRoots.Add(_s.ReserveRootId);
            foreach (ReturnableStage stage in _s.ReturnableStages)
                if (stage.IsNext && stage.ComesHome)
                    _s.ReserveStagedRoots.Add(stage.RootId);
        }
    }

    private static void DisarmStaging(Vehicle vehicle)
    {
        // The next flight looks at the switch again.
        _s.StagingArmChecked = false;
        if (!_s.ArmedStaging)
            return;
        _s.ArmedStaging = false;
        if (!vehicle.IsDisposed)
            StagingDetector.Arm(vehicle, false);
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

        // NOT ON THE PAD. A reserve bigger than the stage can give would otherwise stage the vehicle where it stands, and a booster dropped at zero altitude is a sillier failure than the one being guarded against. Past the vertical rise the test means what it says.
        if (_s.Phase == AscentPhase.Vertical)
            return false;
        // The convex profile starts on the pad too, with no vertical-rise phase to wait out; the same kilometre the ascent trace ignores is the pad for this purpose.
        if (_s.Phase == AscentPhase.Profile && vehicle.Orbit?.Parent is IParentBody body
            && vehicle.Orbit.StateVectors.PositionCci.Length() - body.MeanRadius < TraceMinAltitude)
            return false;

        // SIDE BOOSTERS: counted in their own engines' tanks. The vehicle's mass against the stage model's burnout mixes in the core's propellant, which burns on, and would stage them with a third of their reserve on the heavy (#176).
        if (_s.ReserveFlowShare < 1.0)
            return ReserveTankMass(vehicle) <= _s.ReserveKg;

        UpfgVehicle model = _s.StageModel;
        if (model == null || model.Stages.Count < 2)
            return false;

        double remaining = vehicle.TotalMass - model.Stages[0].MassDry;
        return remaining <= _s.ReserveKg;
    }

    private static double ReserveTankMass(Vehicle vehicle)
    {
        if (_s.ReserveTanks.Count == 0)
            return double.NaN;

        PartTree tree = vehicle.Parts;
        ReadOnlySpan<MoleState> moles = tree.Moles.States;
        double mass = 0.0;
        foreach (Tank tank in _s.ReserveTanks)
        {
            if (!ReferenceEquals(tank.Parent.FullPart.Tree, tree))
            {
                // A split moves the tank and reindexes its moles against the new tree.
                InvalidateStageModel(_s);
                return double.NaN;
            }
            mass += tank.ComputeSubstanceMass(moles);
        }
        return mass;
    }

    // Keep pending handovers outside vehicle state because the detached vehicles do not exist yet. Each separated root has its own landing target, so two boosters leaving on one sequence go to their own sites (#130).
    private sealed class BoosterHandover
    {
        internal Part AttachmentRoot;
        internal Part.Connector Connector;
        internal Part.Connection Connection;
        internal double SiteLatDeg;
        internal double SiteLonDeg;
        internal double ExpiresAt;

        // An optimal return: flies no boostback, only the glide onto its drone ship's point.
        internal bool GlideOnly;
        internal LandingEngineGroups EngineGroups = LandingEngineGroups.Empty;
        internal Dictionary<string, EngineController> LandingEngineBindings;

        // The stack the booster is being dropped from, which it must clear before it lights, see TryAdoptBooster.
        internal Vehicle From;
    }

    private static readonly List<BoosterHandover> _handovers = new List<BoosterHandover>();

    /// <summary>How long a hand-over waits for its booster to show up, s. The split
    /// happens inside the same activation, so this only has to survive a frame or two;
    /// it is generous because the cost of waiting is nothing and the cost of expiring
    /// early is a booster nobody flies.</summary>
    private const double HandoverWindowS = 30.0;

    /// <summary>
    /// Records a targeted booster while its decoupler still connects it to the stack.
    /// </summary>
    internal static void OnDecouplerActivating(Vehicle vehicle, Decoupler decoupler)
    {
        if (!ModActive || !VehicleAutopilotState.TryGet(vehicle, out VehicleAutopilotState s)
            || (s.StageTargets.Count == 0 && s.StageOptimalReturn.Count == 0)
            || vehicle.IsDisposed || VehicleControlOwnership.KfsOwns(vehicle) || !vehicle.IsControllable
            || !ReferenceEquals(decoupler.Parent.FullPart.Tree, vehicle.Parts))
            return;

        double now = SimNow();
        DropExpiredHandovers(now);
        Part root = DetachedRoot(decoupler);
        if (root == null || AlreadyArmed(root, vehicle))
            return;

        // A drone ship's point comes from the ascent plan and stays fixed after lift-off.
        bool glideOnly = false;
        if (!s.StageTargets.TryGetValue(root.InstanceId, out double2 site)
            && !(glideOnly = s.StageOptimalReturn.Contains(root.InstanceId) && s.OptimalReturnSites.TryGetValue(root.InstanceId, out site)))
        {
            if (s.StageOptimalReturn.Contains(root.InstanceId))
                GuidanceLog.Info(vehicle, $"'{root.DisplayName}' is set to optimal return but no plan found its landing point: not handed over.");
            return;
        }
        HashSet<Part> leaving = PartsLeavingOnActivation(vehicle, root, decoupler);
        if (StageControl.Evaluate(vehicle, leaving) != VehicleControl.Controllable)
        {
            GuidanceLog.Info(vehicle, $"'{root.DisplayName}' has no usable control after separation. Booster hand-over skipped.");
            return;
        }
        string engineScope = LandingEngineGroups.PartAddress(root, vehicle.Parts.Root);
        LandingEngineGroups groups = engineScope == null ? LandingEngineGroups.Empty : s.GetEngineGroups(engineScope);
        _handovers.Add(new BoosterHandover
        {
            AttachmentRoot = root,
            Connector = decoupler.Connector,
            Connection = decoupler.Connector.Connection,
            SiteLatDeg = site.X,
            SiteLonDeg = site.Y,
            ExpiresAt = now + HandoverWindowS,
            From = vehicle,
            GlideOnly = glideOnly,
            EngineGroups = groups,
            LandingEngineBindings = groups.Groups.Length == 0 ? null : LandingEngineGroups.Controllers(vehicle, root, leaving),
        });
        GuidanceLog.Info(vehicle, FormattableString.Invariant(
            $"booster hand-over armed for '{root.DisplayName}' on sequence {decoupler.Sequence}, site lat {site.X:F3} lon {site.Y:F3}, landing engines {(groups.Chosen is { } chosen ? $"'{chosen.Name}' ({chosen.Engines.Length})" : "as staged")}."));
    }

    // The player's staging key and AutoStage can both announce one row, so a root already recorded from this stack is not recorded twice.
    private static bool AlreadyArmed(Part root, Vehicle from)
    {
        for (int i = 0; i < _handovers.Count; i++)
        {
            if (ReferenceEquals(_handovers[i].AttachmentRoot, root) && ReferenceEquals(_handovers[i].From, from))
                return true;
        }
        return false;
    }

    private static void DropExpiredHandovers(double now)
    {
        for (int i = _handovers.Count - 1; i >= 0; i--)
        {
            if (now > _handovers[i].ExpiresAt)
                _handovers.RemoveAt(i);
        }
    }

    /// <summary>
    /// Matches an unknown vehicle to a recorded root and gives it a landing target and a pending boostback.
    /// This lets the sweep adopt a separated booster before it skips unknown, unfocused vehicles.
    /// </summary>
    private static bool TryAdoptBooster(Vehicle vehicle)
    {
        double now = SimNow();
        DropExpiredHandovers(now);
        if (_handovers.Count == 0)
            return false;

        PartTree tree = vehicle?.Parts;
        if (tree == null)
            return false;

        // Vehicle.Split can restore the pre-dock root. The attachment part still belongs to the child,
        // but a nested booster cannot claim its record while its own joint remains connected.
        BoosterHandover mine = _handovers.Find(record => ReferenceEquals(record.AttachmentRoot.Tree, tree)
            && !ReferenceEquals(record.From, vehicle)
            && !ReferenceEquals(record.Connector.Connection, record.Connection));
        if (mine == null)
            return false;

        // Consume only this match so other boosters can still claim their records.
        _handovers.Remove(mine);

        if (VehicleControlOwnership.KfsOwns(vehicle))
            return false;

        bool alreadyClaimed = VehicleControlOwnership.Holds(vehicle, ControlClaimant.Guidance);
        if (!VehicleControlOwnership.TryClaim(vehicle, ControlClaimant.Guidance, out _))
            return false;

        // The child inherits the stack's engine command and can drive back into it. Cut now,
        // then give up this claim until the boostback is ready to fly.
        try
        {
            ManualInputs(vehicle).EngineOn = false;
        }
        finally
        {
            if (!alreadyClaimed)
                VehicleControlOwnership.Release(vehicle, ControlClaimant.Guidance);
        }

        _s = VehicleAutopilotState.For(vehicle);
        _s.SiteLatDeg = mine.SiteLatDeg;
        _s.SiteLonDeg = mine.SiteLonDeg;
        _s.BoostbackGlideOnly = mine.GlideOnly;
        if (mine.EngineGroups.Groups.Length > 0)
            _s.EngineGroups[LandingEngineGroups.CraftScope] = mine.EngineGroups.Rebase(mine.LandingEngineBindings, tree);
        GuidanceLog.Info(vehicle, $"adopted as a separated booster, boostback engages within {HandoverWindowS:F0} s.");

        if (mine.From is { IsDisposed: false } stage)
        {
            _s.BoostbackClearOf = stage;
            _s.BoostbackClearFromM = SeparationDistance(vehicle, stage);
            _s.BoostbackClearGapM = 0.0;
        }

        // NOT engaged here, deliberately. This runs on the frame the split happened, which is the frame the part tree is least settled - and ExecuteBoostback needs an aero surrogate fitted to a bounding box that may not exist yet. It refuses rather than throwing, and refusing here would leave a booster that had been adopted and would never be flown, with the hand-over record already consumed. So the engage is retried from the sweep until it takes.
        _s.HandoverPendingUntil = now + HandoverWindowS;
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

        double now = SimNow();
        if (now > _s.HandoverPendingUntil)
        {
            _s.HandoverPendingUntil = double.NegativeInfinity;
            _s.FailBoostback("BOOSTBACK NOT STARTED", "HANDOVER FAILED: " + (_s.AeroError.Length > 0
                ? _s.AeroError : "NO AERO SURROGATE FOR SEPARATED BOOSTER."));
            GuidanceLog.Info(vehicle, _s.BoostbackStatus);
            return;
        }

        // A refused engage rebuilt the aero surrogate from scratch, so the next try waits.
        if (now < _s.HandoverNextAttempt)
            return;

        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;
        ExecuteBoostback(vehicle, orbit, parent);
        if (!BoostbackLive)
            _s.HandoverNextAttempt = now + HandoverRetryS;
    }

    private const double HandoverRetryS = 0.5;

    /// <summary>Centre-of-mass distance between two craft, m.</summary>
    private static double SeparationDistance(Vehicle a, Vehicle b)
        => (a.Orbit.StateVectors.PositionCci - b.Orbit.StateVectors.PositionCci).Length();

    // The part whose subtree separates when this decoupler fires: the tree-child side of its connection - the same rule Vehicle.Split applies.
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


    // KSA models staging through PartTree.PerformanceSequences, but in flight it only recomputes while the stage or engine-control panel is open, so the mod drives the refresh.
    //
    // The game's recompute can run on a vehicle worker thread while the UI reads the stage list. The PrepareWorker prefix runs after JobSystems.VehicleSolver.Wait(), so the recompute and copy-out are safe there. UPFG reconciles stage 0 against the live mass on every step.
    private const long StageModelIntervalMs = 250;
    private const long HousekeepingRetryMs = 1000;

    internal static void InvalidateStageModel(VehicleAutopilotState state)
    {
        state.StageModelDirty = true;
        state.ReserveArmed = false;
        state.ReserveTanks.Clear();
        state.SeparationInS = double.NaN;
        state.SeparationTakenAt = double.NegativeInfinity;
        state.SeparationDrain.Reset();
    }

    private static void RefreshStageModel(Vehicle vehicle)
    {
        // No cross-vehicle invalidation any more. The snapshot lives on the vehicle it describes, so switching craft reads a different one rather than reading someone else's staging for an interval - which is what the old _stageModelVehicle field existed to prevent.

        // Wall-clock gated, not sim-time gated: under warp sim time elapses instantly and this would run every step.
        long now = GuidanceClock.NowMs;
        if (!_s.StageModelDirty && now - _s.StageModelTick < StageModelIntervalMs)
            return;
        _s.StageModelTick = now;
        _s.StageModelDirty = false;
#if DEBUG
        using var _perf = new PerfTracker.Scope("Guidance.RefreshStageModel");
#endif

        // A staging frame can catch the part tree mid-rebuild. Losing one refresh is harmless, because the previous snapshot stays valid, but letting it escape would skip the attitude command for that step, which is not. The first failure retries on the next step, and a repeat waits for the normal interval, so a refresh that keeps failing does not throw on every step.
        try
        {
            // This pressure moves only the headline figures of the active sequence, not the phases Build reads, so Build scales the burning stage itself.
            SequencePerformanceList performance = vehicle.Parts?.PerformanceSequences;
            if (performance == null || vehicle.Parts.SequenceList == null)
                return;
            float ambientPressure = vehicle.PhysicsEnvironment.AtmosphericPressure;
            performance.RecomputeForFlight(ambientPressure);
            _s.StageModel = KsaVehicleAdapter.Build(vehicle, ambientPressure);
            // The game's own total for the same recompute, latched for the staging bar to check our stage list against. Both are "from here on" - its simulated mole masses are re-seeded from the live tanks every recompute - so a disagreement means the adapter is reading the sequence list wrongly, which is exactly the failure that is invisible in a plausible-looking stage table. TotalDeltaV is a Volatile.Read of a float, so the draw thread can have this even though the Lists behind it can tear.
            _s.StageModelKsaDv = performance.TotalDeltaV;
            _s.PadTwr = !_s.Running && vehicle.Orbit?.Parent is IParentBody body && TryLaunchSite(vehicle.Orbit, body, out _)
                ? MeasurePadTwr(vehicle, body, ambientPressure)
                : Ui.Logic.LaunchTwr.Unknown;

            // One line when the staging changes shape, not one per refresh.
            if (GuidanceLog.Enabled)
            {
                string signature = GuidanceLog.StageSignature(_s.StageModel);
                if (signature != _s.StageModelSignature)
                {
                    _s.StageModelSignature = signature;
                    GuidanceLog.Debug(vehicle, $"stage model: {GuidanceLog.DescribeStages(_s.StageModel)}; KSA total {_s.StageModelKsaDv:F0} m/s.");
                }
            }

            // The returnable stage list and the reserve ride the same tick: their inputs - the stage model and the part tree the separation walks read - only change when staging does, and StageModelDirty is already set exactly then. The list goes first, because the reserve arms on its booster's target.
            RefreshReturnableStages(vehicle);
            RememberStageEngineChoices(vehicle);
            RefreshReserve(vehicle);
            _s.StageModelFailures = 0;
        }
        catch (Exception ex)
        {
            if (++_s.StageModelFailures == 1)
                _s.StageModelDirty = true;
            LogHelper.WarnOnce($"guidance-stage-model-{vehicle.Id}:{ex.GetType().Name}",
                $"[AFC] Guidance stage model refresh failed on '{vehicle.Id}': {ex}");
        }
    }

    // Scratch for MeasurePadTwr, filled and consumed inside one call.
    private static readonly Dictionary<UpfgStage, HashSet<Part>> _padTwrParts = new();
    private static readonly List<Ui.Logic.TwrStage> _padTwrStages = new();

    /// <summary>
    /// The first stage's thrust-to-weight ratio on the pad. A stage list of its own, built with the engine parts of each stage, because the stage model above does not keep them and the first stage is told by them: a later stage belongs to it while it burns an engine the lift-off stage burns. Every stage's thrust is taken at the pad's air pressure, from the same pressure response the calculated climb plans with (KsaVehicleAdapter.Build with a pressure grid). Runs where RefreshStageModel runs, after the game's recompute.
    /// </summary>
    private static Ui.Logic.LaunchTwrFacts MeasurePadTwr(Vehicle vehicle, IParentBody parent, double ambientPressure)
    {
        _padTwrParts.Clear();
        _padTwrStages.Clear();
        try
        {
            UpfgVehicle model = KsaVehicleAdapter.Build(vehicle, ambientPressure, new[] { ambientPressure }, _padTwrParts);
            if (model.Stages.Count == 0 || !_padTwrParts.TryGetValue(model.Stages[0], out HashSet<Part> liftoff) || liftoff.Count == 0)
                return Ui.Logic.LaunchTwr.Unknown;
            foreach (UpfgStage stage in model.Stages)
            {
                double thrust = stage.ThrustAtPressure is { Length: > 0 } atPad ? atPad[0] : stage.Thrust;
                bool first = _padTwrParts.TryGetValue(stage, out HashSet<Part> parts) && parts.Overlaps(liftoff);
                _padTwrStages.Add(new Ui.Logic.TwrStage(thrust, stage.MassTotal, first));
            }
            double r = vehicle.Orbit.StateVectors.PositionCci.Length();
            return Ui.Logic.LaunchTwr.Of(_padTwrStages, parent.Mu / (r * r));
        }
        finally
        {
            _padTwrParts.Clear();
            _padTwrStages.Clear();
        }
    }

    // The staged vehicle in UPFG's format, taken from the snapshot above. If the vehicle has no usable sequences (e.g. a single stack with engines already lit and no decouplers), fall back to one stage built from the live engine configuration. Null means "nothing to fly with yet" - either the snapshot hasn't been taken or there is no thrust anywhere - a transient the caller waits out by holding its last solution.
    private static UpfgVehicle BuildUpfgVehicle(Vehicle vehicle)
    {
        UpfgVehicle snapshot = _s.StageModel;
        if (snapshot == null)
            return null;   // no refresh has landed for this vehicle yet

        if (snapshot.Stages.Count > 0)
        {
            // Hand out fresh stage objects: ApplyGLimit rewrites Mode/GLim in place, and the snapshot has to survive into the next step.
            var copy = new UpfgVehicle();
            foreach (UpfgStage s in snapshot.Stages)
                copy.Stages.Add(new UpfgStage
                {
                    Mode = s.Mode, Thrust = s.Thrust, Isp = s.Isp,
                    MassTotal = s.MassTotal, MassDry = s.MassDry, GLim = s.GLim,
                    Seq = s.Seq, Engines = s.Engines,
                });
            copy.BurningStageThrustRatio = snapshot.BurningStageThrustRatio;
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

    // The ascent propellant reserve protects fuel for the return flight.
    //
    // WHAT THIS IS FOR. A booster that flies itself home cannot spend every drop of its propellant reaching staging velocity: boostback, entry and landing all have to come out of the same tanks. So the ascent stops the first stage early, leaving a dV reserve behind, and hands the booster over with a landing site.
    //
    // TWO LEVERS, AND BOTH ARE NEEDED. UPFG never commands staging - it plans around stage boundaries, but the event itself is fired by the tanks running dry (see AutoSequence). So the reserve raises the stage's MassDry, which is what makes UPFG believe the stage is shorter and hand the difference to the upper stage; and it adds a staging CUE, which is what actually cuts the burn. Doing only the first makes UPFG plan a shorter stage and then burn straight through the reserve anyway.

    /// <summary>
    /// The booster's dry mass, kg: what the first real jettison in the stage model drops. Zero when there is no separation to keep a reserve for.
    ///
    /// Read straight off the stage model: the mass that goes overboard at the first real jettison is the booster, and Coalesce has already used exactly that discontinuity to decide the boundary is a boundary (KsaVehicleAdapter).
    /// </summary>
    private static double BoosterDryKg(UpfgVehicle model)
    {
        if (model == null || model.Stages.Count < 2)
            return 0.0;
        UpfgStage first = model.Stages[0], next = model.Stages[1];
        // A real jettison, not a g-limit split or an engine cutting out: those leave the mass continuous and are not a booster going anywhere.
        double dropped = first.MassDry - next.MassTotal;
        return dropped > 1e-4 * Math.Max(first.MassTotal, 1.0) ? dropped : 0.0;
    }

    // Scratch for the separation walk, filled and consumed inside one call, never read across calls.
    private static readonly HashSet<Part> _separationDrops = new HashSet<Part>();
    // Likewise: the engines the ignition-only sequences ahead of the separation will light.
    private static readonly HashSet<EngineController> _pendingIgnition = new HashSet<EngineController>();

    /// <summary>
    /// True when the next separation takes burning engines with it, so a reserve kept back stays in what leaves. <paramref name="flowShare"/> is their share of the burning engines' propellant flow: one when everything burning leaves, as a core's or an in-line booster's separation does.
    ///
    /// SIDE BOOSTERS LEAVE BESIDE A CORE THAT BURNS ON (#176), so the share is below one, and the reserve is counted in their own tanks (<see cref="CollectReserveTanks"/>). This once refused any separation that left an engine burning. The guard it kept, a core's reserve armed against its strap-ons' separation, is now RefreshReserve's: the targeted booster has to be the next to separate, and casings with no pod are never a target.
    ///
    /// ON THE PAD THE NEXT SEQUENCE IS USUALLY THE IGNITION, which separates nothing.
    /// Stopping there left the reserve unarmed until the engines were lit, and the
    /// convex ascent, planned on the pad before the launch commit, planned the whole first
    /// stage: the reserve then armed in the climb and staged the booster early, and UPFG
    /// took over from a vehicle well off the plan (2stage_controlled, 2026-11-04: 30 deg
    /// planned pitch at the hand-over against UPFG's 51). So sequences that separate
    /// nothing are passed over to the first that does, and the engines they will light
    /// count as burning, as they will be when that separation comes.
    /// </summary>
    private static bool NextSeparationTakesBurningEngines(Vehicle vehicle, out double flowShare, out string why)
    {
        why = "";
        flowShare = 1.0;
        PartTree tree = vehicle?.Parts;
        SequenceList sequenceList = tree?.SequenceList;
        if (tree == null || sequenceList == null)
        {
            why = "NO PART TREE";
            return false;
        }

        _separationDrops.Clear();
        _pendingIgnition.Clear();
        bool anySequence = false;
        FairingJettison.Row[] fairings = DeferredFairingRows(vehicle);
        ReadOnlySpan<Sequence> sequences = sequenceList.Sequences;
        for (int s = 0; s < sequences.Length && _separationDrops.Count == 0; s++)
        {
            if (sequences[s].Activated || sequences[s].Parts.IsEmpty || FairingJettison.IsDeferred(fairings, sequences[s]))
                continue;
            anySequence = true;
            ReadOnlySpan<Part> parts = sequences[s].Parts;
            for (int i = 0; i < parts.Length; i++)
            {
                foreach (ISequenced module in parts[i].InSequence(sequences[s].Number))
                {
                    if (module is not Decoupler decoupler || !StagingHooks.CanActivate(decoupler))
                        continue;
                    Part root = DetachedRoot(decoupler);
                    if (root != null)
                        CollectSubtree(root, _separationDrops);
                }
            }
            // An ignition-only sequence: what it lights is burning by the time a later one separates. The separating sequence's own engines are not, since they light as it fires.
            if (_separationDrops.Count == 0)
                for (int i = 0; i < parts.Length; i++)
                    foreach (ISequenced module in parts[i].InSequence(sequences[s].Number))
                        if (module is EngineController engine)
                            _pendingIgnition.Add(engine);
        }
        if (!anySequence)
        {
            why = "NO STAGING SEQUENCE LEFT";
            return false;
        }
        if (_separationDrops.Count == 0)
        {
            why = "NO SEQUENCE LEFT TO SEPARATE";
            return false;
        }

        if (!ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
                .TryGetFrom(tree.States, out var engineStates))
        {
            why = "NO ENGINE STATES";
            return false;
        }

        // By engine part, as the stage model and the plan's flow share count them (KsaVehicleAdapter.PartDesignPerformance).
        _litLeaving.Clear();
        _litStaying.Clear();
        foreach (var engine in engineStates.ModulesAndStates)
        {
            if ((!engine.Module.IsActive || !engine.State.IsPropellantAvailable) && !_pendingIgnition.Contains(engine.Module))
                continue;
            Part part = engine.Module.Parent;
            if (_separationDrops.Contains(part.FullPart))
                _litLeaving.Add(part);
            else
                _litStaying.Add(part);
        }
        if (_litLeaving.Count == 0)
        {
            why = _litStaying.Count == 0 ? "NO ENGINE BURNING" : "NEXT SEPARATION TAKES NO BURNING ENGINE";
            return false;
        }
        if (_litStaying.Count > 0)
        {
            double leaving = 0.0, staying = 0.0;
            foreach (Part part in _litLeaving)
                leaving += KsaVehicleAdapter.PartDesignPerformance(tree, part, 0.0).massFlow;
            foreach (Part part in _litStaying)
                staying += KsaVehicleAdapter.PartDesignPerformance(tree, part, 0.0).massFlow;
            if (!(leaving > 0.0))
            {
                why = "NO FLOW FOR THE SEPARATING ENGINES";
                return false;
            }
            flowShare = leaving / (leaving + staying);
        }
        return true;
    }

    // Scratch for the separation walk: the burning engine parts that leave with it and that stay.
    private static readonly HashSet<Part> _litLeaving = new HashSet<Part>();
    private static readonly HashSet<Part> _litStaying = new HashSet<Part>();
    private static readonly HashSet<RocketCore> _leavingCores = new HashSet<RocketCore>();

    /// <summary>The tanks in what the next separation drops (<see cref="_separationDrops"/>, as NextSeparationTakesBurningEngines left it) that feed the engines dropped with them: the side boosters' own propellant, which their reserve is counted in. A pod's tank that only RCS draws on is not one.</summary>
    private static void CollectReserveTanks(Vehicle vehicle, List<Tank> into)
    {
        into.Clear();
        _leavingCores.Clear();
        foreach (Part part in _separationDrops)
            foreach (EngineController engine in part.SubtreeModules.Get<EngineController>())
                if (engine.Rockets != null)
                    foreach (Rocket rocket in engine.Rockets)
                        foreach (RocketCore core in rocket.Cores)
                            _leavingCores.Add(core);
        foreach (Part part in _separationDrops)
        {
            Span<Tank> tanks = part.SubtreeModules.Get<Tank>();
            for (int i = 0; i < tanks.Length; i++)
            {
                foreach ((ResourceManager manager, int _) in tanks[i].AvailableConsumers)
                {
                    if (manager.Consumer is PlumbedCore core && _leavingCores.Contains(core))
                    {
                        into.Add(tanks[i]);
                        break;
                    }
                }
            }
        }
        _leavingCores.Clear();
    }

    /// <summary>
    /// The first stage, in staging order, that can fly home and has a landing target set (its SITE in LAUNCH > BOOSTERS), else null. Setting one is what asks for a booster reserve.
    ///
    /// NOT ONLY THE NEXT ONE. A core under strap-on boosters with no command pod is not the next thing to separate on the pad - the strap-ons are - but it is the booster coming home, and the plan has to size its reserve before lift-off. It arms once the strap-ons are gone and it is next (RefreshReserve).
    /// </summary>
    internal static ReturnableStage BoostbackTarget()
        => _s.ReturnableStages.Find(st => st.ComesHome);

    /// <summary>The first targeted stage that is next to separate, else null: the booster whose reserve can arm now (RefreshReserve). Side boosters leaving together share one reserve, so the first of them stands for the row.</summary>
    internal static ReturnableStage NextTargetedBooster()
        => _s.ReturnableStages.Find(st => st.ComesHome && st.IsNext);

    /// <summary>One booster and its site, as a plan records it (AscentPlan.RecoverySite). An optimal return has no site going in: the plan finds it.</summary>
    internal static string RecoverySiteKey(ReturnableStage booster)
        => booster == null ? ""
         : booster.HasTarget ? FormattableString.Invariant($"{booster.RootId}:{booster.TargetLatDeg:F6}:{booster.TargetLonDeg:F6}")
         : booster.OptimalReturn ? FormattableString.Invariant($"{booster.RootId}:OPTIMAL")
         : "";

    /// <summary>Every targeted booster and its site, in staging order, as a plan records them (AscentPlan.RecoverySite): a target set, moved or cleared since makes that plan's reserves stale.</summary>
    internal static string RecoverySitesKey()
    {
        var keys = new List<string>();
        foreach (ReturnableStage stage in _s.ReturnableStages)
            if (stage.ComesHome)
            {
                string key = RecoverySiteKey(stage);
                if (stage.EngineGroupScope.Length > 0 && _s.GetEngineGroups(stage.EngineGroupScope).Chosen is { } group)
                    key += ":engines:" + Uri.EscapeDataString(group.Name) + ":" + string.Join(",", group.Engines);
                keys.Add(key);
            }
        return string.Join(";", keys);
    }

    private static bool HasExplicitBoosterEngines()
        => _s.ReturnableStages.Any(stage => stage.ComesHome && stage.EngineGroupScope.Length > 0
            && _s.GetEngineGroups(stage.EngineGroupScope).Chosen != null);

    /// <summary>The plan's entry for a booster: a usable convex plan for this vehicle, for the targets the boosters have now, that brings the booster rooted at <paramref name="rootId"/> home. False when there is none, or the gravity turn flies instead.</summary>
    internal static bool PlannedRecovery(Vehicle vehicle, uint rootId, out AscentRecoveryResult result)
    {
        result = default;
        AscentPlan p = _s.AscentPlan;
        // With no vehicle given (a page drawing the controlled craft's state), the plan is taken as this state's own.
        if (!_s.FlyConvexAscent || _s.BackupAscent || p == null || !p.Usable || (vehicle != null && p.VehicleId != vehicle.Id))
            return false;
        // Once flying, the targets cannot move the plan - the climb is committed, and a booster that has left takes its target with it - but before then a plan for other sites is not these boosters'.
        if (!_s.Running && p.RecoverySite != RecoverySitesKey())
            return false;
        return p.TryGetRecovery(rootId, out result);
    }

    /// <summary>The reserve the convex plan sized for the booster rooted at <paramref name="rootId"/>, kg (AscentRecoveryResult.ReserveKg); NaN when there is none.</summary>
    private static double PlannedBoosterReserveKg(Vehicle vehicle, uint rootId)
        => PlannedRecovery(vehicle, rootId, out AscentRecoveryResult r) ? r.ReserveKg : double.NaN;

    /// <summary>
    /// Decide whether the booster reserve is armed, and how much it keeps. Runs with the stage model, on
    /// its 4 Hz tick, because its inputs - the model and the part tree - only change
    /// when staging does, and StageModelDirty already marks that.
    ///
    /// NOTHING TO SET. The reserve arms when the booster that separates next has a landing target, and keeps whatever the convex plan sized for it - from nothing, for the boostback to that target and the landing (#84). Armed with no usable plan it keeps nothing, so the booster burns out as before: only the plan knows what bringing it home costs.
    /// </summary>
    private static void RefreshReserve(Vehicle vehicle)
    {
        _s.ReserveKg = 0.0;
        _s.ReserveBoosterDryKg = 0.0;
        _s.ReserveArmed = false;
        _s.ReserveFlowShare = 1.0;
        _s.ReserveTanks.Clear();
        _s.ReserveRootId = 0;

        if (BoostbackTarget() == null)
        {
            _s.ReserveNote = "";
            return;
        }

        // Something else separates first - strap-ons round a core - so the next staging is no targeted booster's. Not a refusal: it arms when they are gone. A heavy arms its side boosters' reserve first, then its core's once they are gone (#130).
        ReturnableStage booster = NextTargetedBooster();
        if (booster == null)
        {
            _s.ReserveNote = "ARMS AFTER STAGES BELOW SEPARATE";
            return;
        }
        _s.ReserveRootId = booster.RootId;

        // ONCE PER BOOSTER, and this is not just belt-and-braces. A booster staged on its reserve can sit in the list until the split lands, and re-arming on it would stage the row after it - an upper stage over a payload, say - early.
        if (_s.ReserveStaged)
        {
            _s.ReserveNote = "ALREADY STAGED";
            return;
        }

        double boosterDry = BoosterDryKg(_s.StageModel);
        if (!(boosterDry > 0.0))
        {
            _s.ReserveNote = "NO SEPARATION IN STAGE MODEL FOR A RESERVE";
            return;
        }
        if (!NextSeparationTakesBurningEngines(vehicle, out double flowShare, out string why))
        {
            _s.ReserveNote = why;
            return;
        }
        if (flowShare < 1.0)
        {
            CollectReserveTanks(vehicle, _s.ReserveTanks);
            if (_s.ReserveTanks.Count == 0)
            {
                _s.ReserveNote = "NO TANKS FEED THE SIDE BOOSTERS' ENGINES";
                return;
            }
        }
        _s.ReserveFlowShare = flowShare;

        // The convex plan sized it for the booster it brings home: stage it there, so the booster leaves where the plan separates it.
        double planned = PlannedBoosterReserveKg(vehicle, booster.RootId);
        double kg = double.IsFinite(planned) ? planned : 0.0;

        _s.ReserveKg = kg;
        _s.ReserveBoosterDryKg = boosterDry;
        _s.ReserveArmed = true;
        _s.ReserveNote = double.IsFinite(planned) ? "" : "SIZED WHEN PLAN IS SOLVED";
        PrepareDroneShipAero(vehicle, booster);
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
    private static void ApplyAscentReserve(UpfgVehicle live, double reserveKg, double flowShare)
    {
        if (live == null || live.Stages.Count == 0 || !(reserveKg > 0.0))
            return;

        UpfgStage s0 = live.Stages[0];

        // burnTimes goes NEGATIVE if MassDry passes MassTotal, and a negative burn time propagates into tgo without complaint. Once the live mass is already inside the reserve there is nothing left to plan on this stage anyway - the staging cue is about to fire - so the floor here is a guard, not a policy.
        // SIDE BOOSTERS end the stage reserve / share sooner, where share is their part of its flow; the core's part of that stays aboard and starts the next stage heavier (#176).
        double share = flowShare > 0.0 && flowShare <= 1.0 ? flowShare : 1.0;
        double floor = s0.MassTotal - 0.02 * (s0.MassTotal - s0.MassDry);
        double before = s0.MassDry;
        s0.MassDry = Math.Min(s0.MassDry + reserveKg / share, floor);
        if (share < 1.0 && live.Stages.Count > 1)
            live.Stages[1].MassTotal += (s0.MassDry - before) * (1.0 - share);
    }

    // The thrust acceleration limit every plan honours, in g, which is the lower of the player's g-limit when it is on and the structural limit stock holds the throttle to (KsaEnginePerf.StructuralAccelerationLimit).
    // Positive infinity when neither applies.
    // The structural limit is VehicleStructuralLimits.EffectiveMaxGLoad of the stack as it flies now, from the radius of its BoundingBoxAsmb, and every solve reads it again.
    // Every stage of a plan takes it. A separation usually leaves a smaller stack with a higher limit, so a later stage is modelled throttled where stock will not throttle it, and the first solve after the staging corrects that.
    // Vehicle.Split keeps the root side in the original vehicle, so its colliders stay in the same assembly frame and its box cannot grow. A flying stack that is split off and re-rooted can get a larger box in its new frame, which is not modelled.
    internal static double EffectiveAccelLimitG(Vehicle vehicle) =>
        Math.Min(UserGLimitG, KsaEnginePerf.StructuralAccelerationLimit(vehicle) / StandardGravity);

    private static double UserGLimitG => _s.GLimitEnabled && _s.GLimitG > 0.1 ? _s.GLimitG : double.PositiveInfinity;

    private const double StandardGravity = 9.80665;

    // The vehicle-wide acceleration limit is applied to the stage list on every step. A stage that crosses the limit mid-burn is divided at the mass where full thrust reaches the limit.
    // Every stage takes the same limit, including one with a solid motor, because FlightComputer.SolveGLoadThrottleCap throttles the liquid engines beside it toward the structural limit too. A stage of solids alone that passes the limit passes it under stock as well, and no plan can change that.
    internal static void ApplyGLimit(UpfgVehicle vehicle, double gLim)
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

    /// <summary>
    /// The five modes that contend for one vehicle. Not the same list as the
    /// console pages: landing from orbit and landing from here are two ways on one
    /// page but one machine (<see cref="LandingPhase"/>), and touchdown guidance
    /// (6-DOF) and entry guidance (3-DOF) are solver choices on that page but
    /// separate modes.
    /// </summary>
    private enum GuidanceMode { Ascent, Landing, Boostback, SixDof, ThreeDof }

    /// <summary>
    /// TAKE THE VEHICLE FOR ONE MODE, releasing every other.
    ///
    /// All four drive the same commanded attitude and the same engine, and two of them
    /// stepping at once means both write <see cref="VehicleAutopilotState.CommandDir"/>
    /// in the same step - whichever is dispatched last in ApplyAutopilot silently wins.
    /// So exactly one may be live, and this is the single place that is true.
    ///
    /// IT REPLACES FIVE HAND-WRITTEN CLEAR-LISTS, and they had already drifted apart -
    /// which is the argument for this existing at all rather than a tidiness one:
    ///
    ///   StartGuidance       cleared landing + boostback, not 6-DOF
    ///   ExecuteLanding      cleared ascent + boostback, not 6-DOF
    ///   ExecuteBoostback    cleared ascent + landing,   not 6-DOF
    ///   StartGfoldNow       cleared Running + boostback, NOT LaunchArmed
    ///   StartTerminalHover  cleared Running only - NOT boostback, NOT LaunchArmed
    ///   6-DOF engage        cleared NOTHING
    ///
    /// The last two are the live bug. Taking terminal hover while a boostback was
    /// running left both machines stepping: LandingPhase.TerminalHover and a live
    /// BoostbackPhase both pass their guard in ApplyAutopilot, both write CommandDir,
    /// and StepBoostback runs after StepLanding - so the hover controller computed a
    /// command every step and the boostback overwrote it, with nothing anywhere saying
    /// the hover was not flying the vehicle. Engaging 6-DOF over a running ascent was
    /// benign only by dispatch order (Step6Dof returns before the others run), but it
    /// left _s.Running latched true, so the ascent gauge went on reporting a live climb
    /// for a vehicle under 6-DOF control.
    ///
    /// WHAT RELEASING DOES NOT DO is reset the shared UPFG solver. Ascent and the
    /// deorbit burn are two modes on one <see cref="VehicleAutopilotState.Upfg"/>
    /// instance, so a reset belongs to whichever mode is claiming (StartGuidance does
    /// its own) and never to the release.
    ///
    /// 6-DOF is the one release with real work behind it - it owns a solver thread and
    /// the TVC override - so it goes through Disengage6Dof, gated on actually being
    /// engaged. Ungated, every commit would cut the engine and stop the log of a
    /// vehicle that was never flying 6-DOF at all.
    /// </summary>
    private static void ClaimVehicle(GuidanceMode mode, Vehicle vehicle)
    {
        GuidanceLog.Debug(vehicle, $"{mode} claims the craft.");
        ResetLandingEngineWait();
        ClearDeorbitPlanState();
        _s.GfoldApproach = GfoldApproach.Direct;
        _s.TerminalBurnSeconds = 0;
        _s.ReleaseWithoutEngineCut = false;
        _s.ShutdownRequested = false;
        _s.TakeoverStop = false;
        _s.ClearStatusLines();
        if (mode != GuidanceMode.Ascent)
        {
            _s.Running = false;
            _s.LaunchArmed = false;
        }

        if (mode != GuidanceMode.Landing)
            _s.LandingPhase = LandingPhase.Idle;

        if (mode != GuidanceMode.Boostback)
        {
            _s.BoostbackEngagePending = false;
            _s.BoostbackPhase = BoostbackPhase.Idle;
        }

        // The 6-DOF standby the 3-DOF's burn keeps is for 6-DOF only; anything else taking the craft ends it.
        if (mode != GuidanceMode.SixDof)
            StopSixDofStandby();

        // The 3-DOF hands over to terminal hover and to 6-DOF with the engine lit, so letting go of it cuts nothing; the incoming mode owns the engine from here.
        if (mode != GuidanceMode.ThreeDof && _s.ThreeDofPhase != ThreeDofPhase.Idle)
        {
            Reset3Dof();
            _s.ThreeDofPhase = ThreeDofPhase.Idle;
        }

        // The engine cut is deliberate and is NOT redundant with the incoming mode's own engine handling. A mode can claim the vehicle in a phase that commands nothing yet - LandingPhase.Coast is the case, coasting to a burn point - and in that phase nothing in ApplyAutopilot writes EngineOn at all. Without the cut, a 6-DOF descent handing over to a deorbit coast would leave the engine lit and throttled the whole way round.
        if (mode != GuidanceMode.SixDof && (_s.Active || _s.EngagePending))
            Disengage6Dof(vehicle);
    }

    // One line per mode phase change, whichever path made it, a step or a console button. Compared against the phases the last line reported, so the draw path needs no log calls of its own.
    private static void LogModeChanges(Vehicle vehicle)
    {
        if (!GuidanceLog.Enabled)
            return;

        if (_s.LandingPhase != _s.LoggedLandingPhase)
        {
            GuidanceLog.Debug(vehicle, $"landing phase {_s.LoggedLandingPhase} -> {_s.LandingPhase}"
                + (_s.LandingStatus.Length > 0 ? $" ({_s.LandingStatus})" : "") + ".");
            _s.LoggedLandingPhase = _s.LandingPhase;
        }
        if (_s.BoostbackPhase != _s.LoggedBoostbackPhase)
        {
            GuidanceLog.Debug(vehicle, $"boostback phase {_s.LoggedBoostbackPhase} -> {_s.BoostbackPhase}"
                + (_s.BoostbackStatus.Length > 0 ? $" ({_s.BoostbackStatus})" : "") + ".");
            _s.LoggedBoostbackPhase = _s.BoostbackPhase;
        }
        if (_s.Active != _s.LoggedSixDofActive)
        {
            GuidanceLog.Debug(vehicle, _s.Active ? "Touchdown guidance is active." : "Touchdown guidance ended.");
            _s.LoggedSixDofActive = _s.Active;
        }
    }

    // Called from GuidanceFeature's prefix on Vehicle.PrepareWorker - i.e. immediately before the sim snapshots the flight computer for this step, the one place where our writes are guaranteed to reach the control loop instead of being erased by the worker copy-back.
    //
    // Every mode is stepped here for every vehicle from the per-vehicle simulation hook.
    public static void ApplyAutopilot(Vehicle vehicle)
    {
#if DEBUG
        using var _perf = new PerfTracker.Scope("Guidance.ApplyAutopilot");
#endif
        // Do not create state while guidance is disabled. Release each vehicle in its PrepareWorker prefix so cleanup reaches the next worker snapshot.
        if (!ModActive)
        {
            RealSpeedHold.Forget(vehicle);
            if (VehicleAutopilotState.TryGet(vehicle, out VehicleAutopilotState disabledState))
            {
                _s = disabledState;
                HandBackVehicle(vehicle);
                if (_s.MoonChain != MoonChainStage.Idle)
                    StopMoonChain(vehicle, "GUIDANCE WAS SWITCHED OFF", disarm: true);
            }
            return;
        }

        // Use creates state, so reserve it for the focused vehicle and reuse existing state elsewhere.
        bool focused = ReferenceEquals(vehicle, Program.ControlledVehicle);
        // A craft just split off a stack takes its stage's landing choice on its first step; one without state gets state for it.
        if (focused)
        {
            Use(vehicle);
            TakeStageEngineChoice(vehicle, createState: false);
        }
        else if (VehicleAutopilotState.TryGet(vehicle, out VehicleAutopilotState state))
        {
            _s = state;
            BodySettings.Follow(_s, vehicle);
            TakeStageEngineChoice(vehicle, createState: false);
        }
        else if (TryAdoptBooster(vehicle))
            TakeStageEngineChoice(vehicle, createState: false);
        else if (!TakeStageEngineChoice(vehicle, createState: true))
            return;

        LogModeChanges(vehicle);
        PublishIdleEngineParking(vehicle);

        // Ahead of the returns below, so a craft whose planning ends releases its hold also when it is not in focus and flies nothing.
        StepRealSpeedHold(vehicle, focused);

        if ((!vehicle.IsControllable || VehicleControlOwnership.KfsOwns(vehicle))
            && (HoldsReleasableState(_s) || _s.MoonChain != MoonChainStage.Idle))
        {
            StopForControlLoss(vehicle);
            return;
        }

        if (_s.FcResetPending)
        {
            RetryRelease(vehicle);
            return;
        }

        if (_s.DeorbitNodeClaimed && _s.LandingPhase is not (LandingPhase.DeorbitNodePending or LandingPhase.DeorbitBurn))
        {
            ReleaseStockDeorbit(vehicle);
            if (_s.LandingPhase == LandingPhase.Done) ClearDeorbitPlanState();
        }

        bool sixDof = _s.Active || _s.EngagePending;
        bool landingActive = _s.LandingPhase != LandingPhase.Idle && _s.LandingPhase != LandingPhase.Done;
        // LaunchArmed counts as flying because a vehicle waiting for its launch window must update the window and fire the commit. A booster adopted at separation also counts as flying until boostback engages.
        bool handingOver = _s.HandoverPendingUntil > SimNow();
        bool flying = sixDof || _s.Running || landingActive || BoostbackLive || ThreeDofLive || _s.ThreeDofEngagePending || _s.BoostbackEngagePending
                   || _s.WasEngaged || _s.LandingCutPending || _s.LaunchArmed || handingOver || _s.MoonChain != MoonChainStage.Idle
                   || _s.LandingEngineCommitPending.HasValue;

        if (_s.ManualEngineChange is { } change)
        {
            _s.ManualEngineChange = null;
            if (focused && !flying && CanSwitchEnginesManually(vehicle))
                ApplyManualEngineChange(vehicle, change);
        }

        // Check ownership per actuator before mode steps can overwrite a changed command. The attitude is compared by value. The engine belongs to whoever arms stock Auto, because Vehicle.PrepareWorker then clears EngineOn on every step after this prefix and stock's own burn logic never writes Auto by itself. Keep cleanup ahead of the idle return so unfocused vehicles can release control too.
        bool attitudeTaken = _s.ControlAcquired
            && !_s.AttitudeOwnership.IsCurrent(vehicle.FlightComputer);
        bool engineTaken = _s.ControlAcquired
            && vehicle.FlightComputer.BurnMode == FlightComputerBurnMode.Auto;
        if (_s.ControlAcquired && ((!sixDof && !_s.Engage)
            || (!sixDof && !_s.Running && !landingActive && !BoostbackLive && !ThreeDofLive && !_s.LaunchArmed)
            || attitudeTaken || engineTaken))
        {
            // Release after a takeover without changing the engine command. A takeover moves one actuator and nothing else, so an engine cut guidance already decided on still happens, whether it sits in the queued one-shot cut or in a release that shuts down by itself. The reason outlives a failed cleanup, so the retry can say it.
            if (attitudeTaken || engineTaken)
            {
                _s.TakeoverStop = true;
                _s.TakeoverReason = engineTaken
                    ? "GUIDANCE STOPPED: STOCK AUTO TOOK ENGINE."
                    : "GUIDANCE STOPPED: ANOTHER WRITER TOOK ATTITUDE.";
                _s.ReleaseWithoutEngineCut = true;
                GuidanceLog.Info(vehicle, engineTaken
                    ? "engine takeover, stock's burn mode reads Auto."
                    : $"attitude takeover, {_s.AttitudeOwnership.DescribeChange(vehicle.FlightComputer)}.");
            }
            else
                GuidanceLog.Debug(vehicle, "nothing flies the craft any more, releasing it.");
            HandBackVehicle(vehicle);
            return;
        }

        // Nothing engaged and nobody looking: an unfocused idle craft is not worth a stage-model rebuild or a trace sample.
        if (!focused && !flying)
            return;

        // Every step below reads the parent body, so a craft without one sits this step out.
        Orbit orbit = vehicle.Orbit;
        IParentBody parent = orbit?.Parent;
        if (parent == null)
            return;

        // HOUSEKEEPING FIRST, ahead of the 6-DOF dispatch on purpose - that dispatch returns, so anything below it is skipped for a craft flying 6-DOF.
        //
        // Both commit handlers need a stage list the instant they are pressed, and this is the only point in the frame where it can be built without racing the game's recompute on the vehicle worker thread. Runs at about 4 Hz per vehicle on the wall clock.
        if (flying || GuidanceUiVisible || AnyStageTargeted())
            RefreshStageModel(vehicle);

        // After the stage model, because both want a part tree that has finished settling after a separation and this is the first point in the frame where that is true. It engages boostback, so a fault here counts as a failed step like a mode's.
        StepBoosterHandover(vehicle);

        // A readout fault costs readouts, never the flight. After one, the readouts wait a second before they try again, so a fault that repeats does not throw on every step.
        if (GuidanceClock.NowMs >= _s.HousekeepingRetryTick)
        {
            try
            {
                StepReadouts(vehicle, orbit, parent);
            }
            catch (Exception ex)
            {
                _s.HousekeepingRetryTick = GuidanceClock.NowMs + HousekeepingRetryMs;
                LogHelper.WarnOnce($"guidance-readouts-{vehicle.Id}:{ex.GetType().Name}",
                    $"[AFC] Guidance readouts failed on '{vehicle.Id}': {ex}");
            }
        }

        // Likewise the launch window: tracked, and FIRED, from here rather than from the console. It has to run for a focused vehicle that is not flying yet, which is precisely the state an armed launch sits in. It starts the ascent, so a fault here counts as a failed step like a mode's. See StepLaunchWindow.
        StepLaunchWindow(vehicle, orbit, parent, parent.MeanRadius);
        StepMoonTransfer(vehicle, orbit, parent);

        // The convex ascent's Calculate request is built here for the same reason as the stage model: this is the one point in the frame where the part tree is settled and nothing else is reading it. The solve itself runs on its own thread. It catches its own faults, which cost the plan, never the step.
        StepAscentPlanner(vehicle, orbit, parent);

        // A requested reset runs ahead of everything below: the whole point of the button is to recover when the mod's own state is wrong, so it must not depend on that state saying the autopilot is still active.
        if (_s.FcResetPending)
        {
            ApplyPendingFcReset(vehicle);
            return;
        }

        // Planning waits without control, and the stock node takes its claim before Auto is armed.
        // A direct braking coast enters Prep before the manual control claim below.
        if (!LandingGroupStillValid(vehicle))
            return;
        StepLandingEngineCommit(vehicle, SimNow());
        StepDeorbitPlanning(vehicle, orbit, parent);
        // An entry guidance request engages HERE, ahead of the claim below, so its first step is claimed like any other mode's - consumed in Step3Dof it wrote engine and attitude before the claim, and while touchdown guidance flew, the exclusive branch returned before it was consumed at all. Engage3Dof's ClaimVehicle disengages touchdown guidance.
        if (_s.ThreeDofEngagePending)
        {
            _s.ThreeDofEngagePending = false;
            bool engaged = Engage3Dof(vehicle, SimNow());
            if (_s.LandingGroupWaiting)
                _s.ThreeDofEngagePending = true;
            else
                FinishLandingGroupReplan(vehicle, engaged);
        }

        // Check the takeover while stock still owns the burn, because its step returns while it is active.
        if (IsStockDeorbitPhase(_s.LandingPhase))
            StepPendingBoostback(vehicle, orbit, parent);
        if (StepStockDeorbit(vehicle, orbit, parent)) return;
        StepLandingCoast();

        StepPendingBoostback(vehicle, orbit, parent);

        CheckPendingBrakingHandover(vehicle, parent);
        CheckPendingEntryTouchdownHandoff(vehicle, parent);
        sixDof = _s.Active || _s.EngagePending;
        // Active, not EngagePending: a setup that Engage6Dof rejects never writes control, and must not leave the craft owned. A coast owns nothing either, see StepLandingCoast.
        if (_s.Active || (_s.Engage && (_s.Running || LandingCommands(_s.LandingPhase) || BoostbackLive || ThreeDofLive)))
        {
            // A refused claim must not reach command writes or restore the holder's attitude fields.
            if (!TryTakeCraft(vehicle, acquire: true))
                return;
        }

        // 6-DOF is EXCLUSIVE: it drives attitude through the TVC allocator rather than the flight computer, so it must not be mixed with the UPFG / G-FOLD command path below. It has its own engage flag and does not set _s.Running.
        if (sixDof)
        {
            // Pending setup can reach command writes in Step6Dof, so it needs a claim too. Taking the claim alone does not set ControlAcquired, so rejected setup does not cut the engine.
            if (!TryTakeCraft(vehicle, acquire: false))
                return;

            LandingPhase landingBefore = _s.LandingPhase;
            bool boostbackBefore = BoostbackLive;
            bool ascentBefore = _s.Running;
            Step6Dof(vehicle);
            // Touchdown guidance keeps the RCS hold of the burn it took over, and the switch stays the player's there too.
            YieldRcsToThePlayer(vehicle);

            // A step that hands the craft to another guidance mode is not a stop. Handing back here would cut the engine the handover kept lit and reset the mode that just started.
            bool startedAnotherMode =
                (_s.LandingPhase != landingBefore
                    && _s.LandingPhase != LandingPhase.Idle && _s.LandingPhase != LandingPhase.Done)
                || (BoostbackLive && !boostbackBefore)
                || (_s.Running && !ascentBefore);
            // The cold solve can run for several steps without a 6-DOF command, so the previous attitude stays held until a flyable plan takes over its steering.
            if (!startedAnotherMode)
            {
                if (!_s.Active)
                    ReleaseAttitude(vehicle);
                else if (_s.SixDofToContact)
                    HoldTouchdownAttitude(vehicle, parent);
                else if (!_s.Converging && _s.Guidance?.Published != null)
                    HoldManualAttitude(vehicle);
            }
            if (!_s.Active && !_s.EngagePending && !startedAnotherMode)
            {
                // Failed cleanup must retain ownership. This also releases a setup-only claim when HandBackVehicle has no resources to clear.
                if (HandBackVehicle(vehicle))
                    VehicleControlOwnership.Release(vehicle, ControlClaimant.Guidance);
            }
            return;
        }

        if (!flying)
            return;

        // Step this vehicle's guidance and apply its command.
        StepLanding(vehicle, orbit, parent, parent.Mu, parent.MeanRadius);
        StepAscent(vehicle, orbit, parent, parent.Mu, parent.MeanRadius);
        StepBoostback(vehicle, orbit, parent);
        Step3Dof(vehicle, orbit, parent);

        // Read straight off _s, not off the values the bail above was computed from: the flows just ran, and a touchdown, an abort or a handoff can have changed the phase on this very step.
        //
        // The open-loop phases (vertical/kick/prograde) don't need a converged UPFG solution; once flying, keep commanding through transient re-convergence (e.g. right after staging) - dropping to Manual mid-ascent would be far more disruptive.
        bool landingGuides = LandingCommands(_s.LandingPhase);
        // Every live boostback phase steers, including the settling burn (which holds a latched attitude) and the glide (which steers its angle of attack indefinitely) - so unlike the landing machine there is no sub-phase here that wants the vehicle back.
        bool boostbackGuides = BoostbackLive;
        // The 3-DOF steers through the flight computer as the boostback does, from its PID glide to the end of its burn.
        bool threeDofGuides = ThreeDofLive;
        bool shouldCommand = _s.Engage && (_s.Running || landingGuides || boostbackGuides || threeDofGuides)
                          && _s.HasCommand;

        // Auto engine control: master switch on at full throttle while flying, off for good once the terminal countdown expires. Written here - the prefix runs just before PrepareWorker snapshots _manualControlInputs - so it reaches the sim exactly like the player's ignite/shutdown key. One-shot engine cut when the landing flow ends (cutoff, abort, failure) - after this the player's inputs are untouched, so the final descent below the gate can be flown manually.
        if (_s.LandingCutPending)
        {
            // An unowned craft's engine is the player's, which an aborted waiting coast reaches, so the cut is dropped there.
            if (_s.ControlAcquired)
            {
                ref ManualControlInputs cut = ref ManualInputs(vehicle);
                cut.EngineOn = false;
            }
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
                else if (_s.LandingPhase is LandingPhase.GfoldDescent or LandingPhase.TerminalHover
                    or LandingPhase.TerminalCoast or LandingPhase.TerminalBrake)
                {
                    inputs.EngineOn = _s.GfoldThrottle > 0.0;
                    inputs.EngineThrottle = (float)_s.GfoldThrottle;
                }
                else
                {
                    inputs.EngineOn = false; // Prep (pre-ignition)
                }
            }
            else if (threeDofGuides)
            {
                // Off through the glide, the plan's throttle through the burn. Not paired with AutoSequence, for the boostback's reason: an engine-off glide reads to the auto-stager as a cue to fire the next sequence.
                ref ManualControlInputs inputs = ref ManualInputs(vehicle);
                inputs.EngineOn = _s.ThreeDofEngineOn;
                inputs.EngineThrottle = (float)_s.ThreeDofThrottle;
            }
            else if (boostbackGuides)
            {
                // The phase decides; the step recorded it. Deliberately NOT paired with AutoSequence: the machine cuts the engine at boostback cutoff and coasts to entry from there, and the auto-stager reads "no thrust" as a cue to fire the next sequence - so it would work its way down a returning booster's staging list one activation per second, and the sequences left on a first stage are the ones that separate it.
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
                    // Full throttle unless UPFG is holding the acceleration limit. While the convex profile flies, the throttle is the plan's own, which already holds the acceleration limit the plan was solved to.
                    inputs.EngineThrottle = _s.Phase == AscentPhase.Profile ? ConvexThrottle(vehicle, parent) : (float)_s.Upfg.Throttle;
                }
            }
        }

        // AFC's own attitude controller flies the ascent (AscentOwnAttitude) and entry guidance's glide (ThreeDofOwnAttitude); anywhere else the stock flight computer steers, and the controller's actuators are let go the first step it does not fly.
        bool ownAttitude = shouldCommand && OwnsAttitude(vehicle, boostbackGuides, threeDofGuides);
        if (_s.OwnAttitudeFlying && !ownAttitude)
            StopOwnAttitude(vehicle);

        if (shouldCommand)
        {
            // An ascent references its roll to its target plane (see SteerBody2Cci and AscentRollRef); a landing keeps the stock position-derived reference, which is well conditioned there because the thrust axis is fighting the velocity, not lying along the position vector.
            // A returning booster is held at a fixed roll, wings level along its ground track (ReturnRollRefCci), so the body-axis aero table it plans with applies (KsaPointMassModel.RollRef), and flies the Strict profile so it turns as fast as the plan asks.
            bool boosterReturn = boostbackGuides || threeDofGuides;
            bool holdRoll = boosterReturn && _s.BoosterRollHold;
            double3? rollRef = _s.Running ? AscentRollRef(vehicle, _s.CommandDir)
                : holdRoll && _s.CommandDir.Length() > 0.5 ? ReturnRollRefCci(vehicle, parent)
                : null;
            if (ownAttitude)
                FlyOwnAttitude(vehicle, parent, rollRef, heldRoll: holdRoll && !_s.Running);
            else
            {
                CommandAttitude(vehicle, parent, _s.CommandDir,
                                fullEngage: !_s.WasEngaged, rollRef: rollRef,
                                rollMode: holdRoll ? FlightComputerRollMode.Up : null);
                if (boosterReturn && _s.BoosterStrictProfile)
                    _s.AttitudeOwnership.WriteProfile(vehicle.FlightComputer, FlightComputerAttitudeProfile.Strict);
            }

            // Publish the attitude and its turning rate together so they describe the same instant.
            KsaAttitudeRate.Set(vehicle,
                _s.Running || boostbackGuides || threeDofGuides ? _s.CommandRate : default);
            _s.WasEngaged = true;
        }
        // A queued 6-DOF engage keeps the attitude of the mode that handed over, see the 6-DOF branch above.
        else if (_s.WasEngaged && !_s.EngagePending)
        {
            ReleaseAttitude(vehicle);
            _s.WasEngaged = false;
        }

        SteerLandingBurnWithGimbal(vehicle, landingGuides);

        // A temporary command gap releases attitude but keeps ownership. Release ownership when the mode ends, before the next frame applies player input. A coast that the commit queued while another mode still owned the craft stays through that release, and claims again at Prep.
        // A 6-DOF engage queued on this step is a handover, not an end: the deorbit burn queues one at its handoff, and the next step's 6-DOF dispatch runs it. Releasing here would drop the request and cut the engine the burn left lit, so the craft falls with nothing flying it.
        bool stillNeedsCraft = _s.Active || _s.EngagePending
            || (_s.Engage && (_s.Running || landingGuides || boostbackGuides || threeDofGuides));
        if (_s.ControlAcquired && !_s.LaunchArmed && !stillNeedsCraft)
            ReleaseVehicle(vehicle, keepWaitingCoast: true);
    }

    private static void StepReadouts(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        // What it would cost each returnable stage to come home from here. Housekeeping rather than guidance, and deliberately so: the whole value of the number is watching it through the climb, which means it has to be solved before anything has been staged and whether or not the page is open. It is throttled to 1 Hz and shares one Jacobian across every stage - see Guidance/ReturnableStages.cs. Gated on a stage actually having a TARGET, not merely on one being returnable. The surrogate is a 72-azimuth sweep of the game's own CdA and it is refitted at every staging; paying for it on a craft nobody has asked to bring back would be a launch-time cost for a number nothing displays.
        if (AnyStageTargeted())
        {
            EnsureBoostbackAero(vehicle, parent);
            UpdateReturnDv(vehicle, orbit, parent);
        }
        // The console asks for the same work while a page shows it: the aero model before a boostback (so the BOOSTBACK verb finds one), a resample from DIAG, the impact of an idle booster, and the pass strip of a deorbit.
        else if (ViewDemand.Wants(vehicle, ViewDemand.Kind.Aero))
            EnsureBoostbackAero(vehicle, parent);
        if (ViewDemand.ConsumeResample(vehicle))
            ResampleBoostbackAero(vehicle, parent, force: true);
        if (!BoostbackLive && ViewDemand.Wants(vehicle, ViewDemand.Kind.Impact))
            UpdateImpactPrediction(vehicle, orbit, parent, force: false);
        if (ViewDemand.Wants(vehicle, ViewDemand.Kind.Passes))
            RefreshPasses(orbit, parent, parent.Mu, parent.MeanRadius);

        // Likewise the flown-trajectory trace: sampled off the simulation rather than the frame rate, and recorded whether or not guidance is running so the track is already there when the overlay is switched on - losing it the moment guidance engages would blank the overlay during exactly the descent worth watching.
        RecordTrace(vehicle, orbit);

        RefreshVehicleHeight(vehicle);
    }

    private const long VehicleHeightRefreshMs = 500;

    // Keep VehicleHeightM on the measured touchdown height while auto is on. It moves as the propellant drains (the CoM climbs) and changes at every staging, so it is re-read twice a second on the wall clock rather than captured once; the measurement is one bounds query per collider.
    private static void RefreshVehicleHeight(Vehicle vehicle)
    {
        if (!_s.VehicleHeightAuto)
        {
            _s.VehicleHeightStatus = "MANUAL";
            _s.SiteEarlyContactM = 0.0;
            _s.SiteSlopeDeg = _s.SiteTipDeg = _s.SiteStandDeg = double.NaN;
            return;
        }
        long now = GuidanceClock.NowMs;
        if (now < _s.VehicleHeightNextMs && SiteSlopeFits(vehicle.Orbit?.Parent))
            return;
        _s.VehicleHeightNextMs = now + VehicleHeightRefreshMs;

        if (!KsaVehicleHeight.TryMeasure(vehicle, out KsaVehicleHeight.Result h))
        {
            if (!SiteSlopeFits(vehicle.Orbit?.Parent))
                ForgetSiteSlope(vehicle.Orbit?.Parent);
            // Keep the last good value: a guess of zero would plan the CoM into the ground.
            _s.VehicleHeightStowed = false;
            _s.VehicleHeightStatus = FormattableString.Invariant($"NO COLLIDERS TO MEASURE: HOLDING {_s.VehicleHeightM:F1} m");
            return;
        }
        _s.VehicleHeightM = h.DeployedM;
        _s.SiteEarlyContactM = SiteEarlyContact(vehicle.Orbit.Parent, h.FootPoints);
        RefreshSiteSlope(vehicle, in h);
        _s.VehicleHeightStowed = h.Stowed > 0;
        _s.VehicleHeightCard = h.Stowed > 0 ? Ui.Logic.PageText.LegsStowedCard(_s.VehicleHeightM, h.CurrentM) : "";
        _s.VehicleHeightStatus = h.Stowed > 0
            ? Ui.Logic.PageText.LegsStowed
            : h.Deployables > 0 ? "MEASURED, LEGS DEPLOYED" : "MEASURED, NO DEPLOYABLE LEGS";
    }

    // Release attitude without replacing the flight computer and losing the player's burn plan or settings.
    private static void ReleaseAttitude(Vehicle vehicle)
    {
        if (!_s.ControlAcquired)
            return;

        // Clear the commanded rotation rate before restoring attitude settings.
        KsaAttitudeRate.Clear(vehicle);
        // A craft on the ground with its engine off or about to be cut has landed, see AttitudeOwnership.Release.
        bool landed = vehicle.Situation.HasAnyContact() && (_s.LandingCutPending || !ManualInputs(vehicle).EngineOn);
        _s.AttitudeOwnership.Release(vehicle.FlightComputer, leaveManual: landed);
        _s.WasEngaged = false;
    }

    // While 6-DOF steers through the gimbal override, the flight computer must not steer too. The override replaces every stock gimbal command, but ComputeRcsControl still fires RCS on each axis whose ActiveControlSystem is Rcs while AttitudeMode is Auto. Restoring the attitude from before guidance would bring back the Auto rate hold that the stock deorbit burn leaves, and at low throttle the live TVC authority can fall far enough below the RCS authority that the stock choice moves an axis to RCS, so that hold would fight the plan.
    // The attitude stays guidance's own, so a player change still counts as a takeover, and ReleaseAttitude restores the original when 6-DOF ends. WasEngaged is cleared so that a G-FOLD fallback engages its attitude target fully.
    private static void HoldManualAttitude(Vehicle vehicle)
    {
        if (!_s.ControlAcquired)
            return;

        FlightComputer fc = vehicle.FlightComputer;
        if (fc.AttitudeMode != FlightComputerAttitudeMode.Manual)
            KsaAttitudeRate.Clear(vehicle);
        _s.AttitudeOwnership.BeginWrite(fc);
        fc.AttitudeMode = FlightComputerAttitudeMode.Manual;
        _s.AttitudeOwnership.EndWrite(fc, writesRoll: false);
        _s.WasEngaged = false;
    }

    /// <summary>
    /// Keeps RCS out of the attitude control while a landing mode steers a lit engine that sits on a gimbal, so the gimbal alone points the craft, and gives RCS back when the engine is off or the mode ends.
    /// Stock chooses per axis between the gimbals and RCS (FlightComputer.UpdateActiveControlSystemSingleAxis): an axis goes to the gimbals only while their angular acceleration is at least a tenth of what RCS gives. On a light craft with strong thrusters RCS wins every axis, the gimbal is zeroed (FlightComputer.ZeroizeTvcs), and the smallest RCS pulse then changes the body rate by several degrees per second, so the thrust axis swings about its target through the whole burn. With FlightComputer.RCSMode at Disabled, FlightComputer.UpdateRcsParams and UpdateActiveControlSystems see no RCS authority and give the pitch and yaw axes to the gimbals.
    /// An axis the gimbals have no authority on would then have no control system at all, so RCS stays on unless the gimbals hold both pitch and yaw (GimbalHoldsPitchAndYaw). A burn that loses an axis while RCS is off gets RCS back on that step and keeps it until the engine is off.
    /// The switch also belongs to the player. A player who turns RCS back on during the burn keeps it on until guidance lets go of the craft.
    /// </summary>
    private static void SteerLandingBurnWithGimbal(Vehicle vehicle, bool landingGuides)
    {
        FlightComputer fc = vehicle.FlightComputer;
        YieldRcsToThePlayer(vehicle);

        bool lit = ManualInputs(vehicle).EngineOn && KsaGimbalControl.LitGimbalThrust(vehicle) > 0.0;
        if (!lit)
            _s.RcsGimbalLostAxis = false;
        bool want = _s.LandingGimbalSteers && !_s.RcsYielded && _s.Engage && landingGuides && _s.ControlAcquired && lit;
        if (want && !GimbalHoldsPitchAndYaw(fc))
        {
            want = false;
            if (_s.RcsHeldOff)
            {
                _s.RcsGimbalLostAxis = true;
                GuidanceLog.Info(vehicle, "The gimbal lost its authority on pitch or yaw. Guidance leaves RCS on for this burn.");
                GiveRcsBack(vehicle);
            }
        }
        want &= !_s.RcsGimbalLostAxis;

        if (want && !_s.RcsHeldOff && fc.RCSMode == FlightComputerRCSMode.Enabled)
        {
            fc.RCSMode = FlightComputerRCSMode.Disabled;
            _s.RcsHeldOff = true;
            GuidanceLog.Info(vehicle, "RCS off while the gimbal steers the landing burn.");
        }
        // A burn handed to touchdown guidance keeps RCS off through the hand-over, because the attitude target of the burn stays in force until the first plan steers, and one step of it with RCS back on can fire a pulse.
        else if (!want && _s.RcsHeldOff && !_s.EngagePending && !_s.Active)
        {
            GiveRcsBack(vehicle);
        }
    }

    /// <summary>
    /// True while the stock attitude controller has gimbal authority on both the pitch and the yaw axis. FlightComputer.UpdateTvcParams takes for each axis the smaller of the torque ranges in its two directions (FlightComputer.TvcState.TorqueRangePositive and TorqueRangeNegative), so an axis reads zero when the gimbals turn about one axis only, or when the trim for a centre of mass off the thrust line uses up one side of the range. With RCS off, FlightComputer.UpdateActiveControlSystemSingleAxis gives such an axis no control system.
    /// </summary>
    internal static bool GimbalHoldsPitchAndYaw(FlightComputer fc)
        => Math.Min(fc.TvcTorqueAuthority.Y, fc.TvcTorqueAuthority.Z) > 0f;

    // A mode that reads Enabled while guidance holds it off is the player's switch.
    private static void YieldRcsToThePlayer(Vehicle vehicle)
    {
        if (!_s.RcsHeldOff || vehicle.FlightComputer.RCSMode == FlightComputerRCSMode.Disabled)
            return;
        _s.RcsHeldOff = false;
        _s.RcsYielded = true;
        GuidanceLog.Info(vehicle, "RCS was switched on during the landing burn. Guidance leaves RCS on.");
    }

    // Switches RCS on again when guidance switched it off and it still is off.
    private static void GiveRcsBack(Vehicle vehicle)
    {
        if (!_s.RcsHeldOff)
            return;
        _s.RcsHeldOff = false;
        FlightComputer fc = vehicle.FlightComputer;
        if (fc.RCSMode == FlightComputerRCSMode.Disabled)
        {
            fc.RCSMode = FlightComputerRCSMode.Enabled;
            GuidanceLog.Info(vehicle, "RCS on again.");
        }
    }

    /// <summary>
    /// Gives the save record of a flight computer the RCS mode the player has while guidance holds RCS off for a landing burn. FlightComputer.SerializeSave writes RCSMode into the record and FlightComputer.DeserializeSave reads it back, and the hold itself does not outlive a load, so a save made during the burn would otherwise load with RCS off and nothing to switch it on again.
    /// </summary>
    internal static void SaveRcsAsThePlayerHasIt(FlightComputer fc, FlightComputerData data)
    {
        if (data == null || data.RCSMode != FlightComputerRCSMode.Disabled)
            return;
        foreach (var entry in VehicleAutopilotState.Snapshot())
        {
            if (entry.Value.RcsHeldOff && ReferenceEquals(entry.Key.FlightComputer, fc))
            {
                data.RCSMode = FlightComputerRCSMode.Enabled;
                return;
            }
        }
    }

    // Read by the vehicle workers and written only through SetModActive, which queues the releases a switch-off needs.
    private static volatile bool _modActive = true;

    /// <summary>Enables guidance processing in ApplyAutopilot.</summary>
    internal static bool ModActive => _modActive;

    // Vehicle.PrepareWorker clears EngineOn while BurnMode is Auto, so hold Manual while guidance controls the engine. The burn target is recorded with the mode, because an Auto given back on a burn the player never armed would start the stock autopilot on it; see StockBurnMode.
    private static void AcquireControl(Vehicle vehicle)
    {
        if (!_s.ControlAcquired)
        {
            BurnTarget loaded = vehicle.FlightComputer.Burn;
            _s.ForcedBurnManual = StockBurnMode.HoldManual(vehicle);
            _s.ForcedBurnTarget = _s.ForcedBurnManual ? loaded : null;
        }
        _s.ControlAcquired = true;
    }

    // Takes the shared claim for guidance, or says who holds the craft. With acquire, the flight computer settings guidance writes are taken as well.
    private static bool TryTakeCraft(Vehicle vehicle, bool acquire)
    {
        if (!ControlAvailable(vehicle))
            return false;
        if (!VehicleControlOwnership.TryClaim(vehicle, ControlClaimant.Guidance, out ControlClaimant holder))
        {
            _s.WarnStatus($"GUIDANCE HELD BY {VehicleControlOwnership.Describe(holder)}.");
            return false;
        }
        if (acquire)
            AcquireControl(vehicle);
        return true;
    }

    private static void RestoreBurnMode(Vehicle vehicle, bool giveBack)
    {
        if (giveBack && _s.ForcedBurnManual)
            StockBurnMode.GiveBackAuto(vehicle, _s.ForcedBurnTarget);
        _s.ForcedBurnManual = false;
        _s.ForcedBurnTarget = null;
    }

    // Not inlined, because a harness test patches it by name to fail a release.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool HandBackVehicle(Vehicle vehicle) => ReleaseVehicle(vehicle, keepWaitingCoast: false);

    /// <summary>
    /// Stops all guidance of a craft that is not controllable, with an engine cut when guidance held it. A craft guidance never acquired keeps its engine, and the stop says the claim was refused.
    /// </summary>
    private static void StopForControlLoss(Vehicle vehicle)
    {
        bool kfs = vehicle.IsControllable && VehicleControlOwnership.KfsOwns(vehicle);
        string reason = kfs ? "GUIDANCE STOPPED: KFS CONTROLS THE VEHICLE."
            : ControlLostReason(vehicle, _s.ControlAcquired ? "GUIDANCE STOPPED" : "GUIDANCE REFUSED");
        GuidanceLog.Info(vehicle, "control lost: " + reason);
        if (_s.MoonChain != MoonChainStage.Idle)
            StopMoonChain(vehicle, kfs ? "KFS CONTROLS THE VEHICLE" : "CONTROL LOST", disarm: !kfs);
        _s.TakeoverStop = true;
        _s.TakeoverReason = reason;
        _s.ReleaseWithoutEngineCut = kfs;
        _s.ShutdownRequested = !kfs;
        HandBackVehicle(vehicle);
        _s.WarnStatus(reason);
    }

    private static bool ControlAvailable(Vehicle vehicle)
    {
        if (vehicle.IsControllable && !VehicleControlOwnership.KfsOwns(vehicle))
            return true;
        StopForControlLoss(vehicle);
        return false;
    }

    /// <summary>The status line for a craft out of control, with the game's own reason (Vehicle.NoControlTooltip).</summary>
    internal static string ControlLostReason(Vehicle vehicle, string head)
    {
        string why = vehicle.NoControlTooltip.Format();
        return string.IsNullOrWhiteSpace(why)
            ? head + ": VEHICLE OUT OF CONTROL."
            : head + ": VEHICLE OUT OF CONTROL. " + why.Trim().ToUpperInvariant();
    }

    // A failed release is retried with the intent it had, and a release the player or a save load asked for drops the coast.
    private static bool RetryRelease(Vehicle vehicle) => ReleaseVehicle(vehicle, _s.ReleaseRetryKeepsCoast);

    /// <summary>Whether a release of <paramref name="s"/> has guidance state to free, besides the staging it always disarms. The console shows RELEASE from the same test.</summary>
    internal static bool HoldsReleasableState(VehicleAutopilotState s)
        => s.ControlAcquired || s.DeorbitNodeClaimed || s.FcResetPending || s.Worker != null
            || s.Active || s.EngagePending || s.Converging || s.Running
            || s.LaunchArmed || s.ConvexLaunchPending || s.LandingCutPending
            || s.ThreeDofEngagePending || s.BoostbackEngagePending || s.LandingEngineCommitPending.HasValue
            || s.HasCommand || s.Guidance != null
            || s.HandoverPendingUntil != double.NegativeInfinity
            || ReferenceEquals(SixDofLog.Owner, s)
            || s.LandingPhase != LandingPhase.Idle || s.BoostbackPhase != BoostbackPhase.Idle
            || s.ThreeDofPhase != ThreeDofPhase.Idle;

    /// <summary>
    /// Releases this vehicle's guidance state, keeping ownership if any cleanup fails.
    /// Run from the PrepareWorker prefix so the worker receives the released state.
    /// With keepWaitingCoast a coast to a deorbit burn stays queued, because it commands nothing and claims the craft on its own at Prep.
    /// </summary>
    private static bool ReleaseVehicle(Vehicle vehicle, bool keepWaitingCoast)
    {
        // Clear the wait even when no guidance resources need release.
        ResetLandingEngineWait();
        ResetLandingEngineSelection();
        DisarmStaging(vehicle);
        if (!HoldsReleasableState(_s))
        {
            // Nothing to release, so the next stop starts from the default again.
            _s.ReleaseWithoutEngineCut = false;
            _s.ShutdownRequested = false;
            _s.TakeoverStop = false;
            return true;
        }

        bool coastWaits = keepWaitingCoast && LandingWaits(_s.LandingPhase);
        // kFS can own actuator allocation without changing the C# attitude fields first.
        // Drop saved settings so cleanup cannot overwrite its command or restore stock Auto.
        if (VehicleControlOwnership.KfsOwns(vehicle) && vehicle.IsControllable)
        {
            _s.AttitudeOwnership.Forget();
            _s.RcsHeldOff = false;
            _s.ForcedBurnManual = false;
            _s.DeorbitThrottleHeld = false;
            _s.ReleaseWithoutEngineCut = true;
            _s.ShutdownRequested = false;
            _s.LandingCutPending = false;
        }
        CancelPendingLaunch();
        _s.ThreeDofEngagePending = false;
        _s.BoostbackEngagePending = false;
        _s.Running = false;
        if (!coastWaits)
        {
            _s.LandingPhase = LandingPhase.Idle;
            ClearDeorbitPlanState();
        }
        _s.BoostbackPhase = BoostbackPhase.Idle;
        if (_s.ThreeDofPhase != ThreeDofPhase.Idle)
        {
            Reset3Dof();
            _s.ThreeDofPhase = ThreeDofPhase.Idle;
        }
        _s.LaunchArmed = false;
        _s.HasCommand = false;
        _s.Active = false;
        _s.EngagePending = false;
        _s.Converging = false;
        _s.SixDofStandby = false;
        _s.HandoverPendingUntil = double.NegativeInfinity;

        string failure = "";
        // One cleanup failure must not prevent the remaining releases.
        void Attempt(Action release)
        {
            try { release(); }
            catch (Exception error)
            {
                if (failure.Length == 0)
                    failure = "RELEASE FAILED: " + error.Message;
            }
        }

        Attempt(() => ReleaseStockDeorbit(vehicle, keepClaim: true, preserve: _s.ReleaseWithoutEngineCut));
        Attempt(() => KsaAttitudeRate.Clear(vehicle));
        Attempt(() => KsaGimbalControl.Disengage(vehicle));
        Attempt(() => KsaRcsTorqueControl.Release(vehicle));
        Attempt(() => GiveRcsBack(vehicle));
        Attempt(_s.CancelGfoldSolve);

        // A cut guidance decided on wins over a no-cut request, whenever it was decided. The request can outlive a failed cleanup, so an abort between two retries would otherwise lose its shutdown.
        bool cutEngine = _s.LandingCutPending || _s.ShutdownRequested || !_s.ReleaseWithoutEngineCut;
        if (_s.ControlAcquired)
        {
            if (cutEngine)
                Attempt(() => vehicle.SetEnum(VehicleEngine.MainShutdown));
            // A craft out of control keeps Manual, because the flight computer steers in Auto whether or not the craft is controllable.
            bool leaveManual = (cutEngine && vehicle.Situation.HasAnyContact()) || !vehicle.IsControllable;
            Attempt(() => _s.AttitudeOwnership.Release(vehicle.FlightComputer, leaveManual: leaveManual));
        }
        Attempt(() =>
        {
            _s.Worker?.Dispose();
            _s.Worker = null;
            _s.Guidance = null;
        });
        Attempt(() => ReportLogStop(SixDofLog.Stop(_s)));

        // Stock Auto comes back last, once nothing of guidance still commands the craft, and only with a cut. A no-cut release leaves the engine as the flight left it, and a restored Auto would have Vehicle.PrepareWorker switch it off on the same step.
        if (failure.Length == 0 && _s.ControlAcquired)
            Attempt(() => RestoreBurnMode(vehicle, giveBack: cutEngine && vehicle.IsControllable));

        if (failure.Length > 0)
        {
            _s.FcResetPending = true;
            _s.ReleaseRetryKeepsCoast = coastWaits;
            if (_s.ReleaseError != failure)
                GuidanceLog.Info(vehicle, failure + " The release is retried on the next step.");
            _s.ReleaseError = failure;
            _s.DiagStatus(failure);
            return false;
        }

        GuidanceLog.Info(vehicle, "released the craft"
            + (!_s.ControlAcquired ? ", nothing was held" : cutEngine ? " with an engine cut" : ", engine left as it was")
            + (coastWaits ? ", the coast to the burn continues" : "")
            + (_s.TakeoverStop ? $" after a takeover: {_s.TakeoverReason}" : "."));
        if (_s.Status == _s.ReleaseError)
            _s.Status = "";
        if (_s.TakeoverStop)
        {
            _s.WarnStatus(_s.TakeoverReason);
            _s.TakeoverStop = false;
        }
        _s.ShutdownRequested = false;
        _s.ControlAcquired = false;
        _s.RcsYielded = false;
        _s.RcsGimbalLostAxis = false;
        VehicleControlOwnership.Release(vehicle, ControlClaimant.Guidance);
        _s.FcResetPending = false;
        _s.ReleaseRetryKeepsCoast = false;
        _s.LandingCutPending = false;
        _s.ReleaseWithoutEngineCut = false;
        _s.WasEngaged = false;
        _s.ReleaseError = "";
        return true;
    }

    // Queue UI requests for the PrepareWorker prefix because worker results can overwrite UI attitude writes.
    private static void ResetFlightComputer()
    {
        ResetLandingEngineWait();
        _s.FcResetPending = true;
        _s.ReleaseRetryKeepsCoast = false;
        _s.Status = "RELEASE REQUESTED.";
    }

    // A queued reset runs the same cleanup as any other release, so a failed attempt keeps its pending request and is retried on a later step.
    private static void ApplyPendingFcReset(Vehicle vehicle) => RetryRelease(vehicle);

    // Convert a commanded thrust direction into the flight computer's Custom-attitude Euler command. We use KSA's own ComputeBurnBody2Cci to build the body->CCI orientation that points thrust along the steering vector, then express it as Euler angles in the EclBody frame - the exact inverse of the conversion the flight computer applies when it reads CustomAttitudeTarget.
    //
    // rollRef, when supplied, replaces the ROLL REFERENCE that ComputeBurnBody2Cci derives from the position vector - see SteerBody2Cci for why an ascent must not use the stock one.
    //
    // fullEngage=true additionally switches the FC into Custom/Auto tracking - done once on engage, exactly like clicking "Apply Euler Target" in the attitude tab.
    private static void CommandAttitude(Vehicle vehicle, IParentBody parent, double3 dir,
                                        bool fullEngage, double3? rollRef = null, FlightComputerRollMode? rollMode = null)
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
        if (!TryTakeCraft(vehicle, acquire: true))
            return;
        _s.AttitudeOwnership.BeginWrite(fc);
        fc.CustomAttitudeTarget = euler;

        // WHETHER THE FLIGHT COMPUTER LOOKS AT THE ROLL WE JUST COMMANDED. UpdateAttitudeTrackError computes a roll term only when RollMode is not Decoupled; decoupled is the default and discards the target's roll entirely, tracking pointing alone. That is the right behaviour for an ascent - roll is the axis a launch vehicle has least authority about and nothing in the trajectory needs a particular one - so it stays decoupled unless the roll is being forced deliberately.
        //
        // Also decoupled below ForceRollMinAltM even with the box ticked: right off the pad is exactly where the vehicle has the least authority to fight a commanded roll and the least room to recover if it can't, so a forced roll waits for some clearance first.
        //
        // Written every step, not only on engagement: the box can be ticked mid-ascent, and un-ticking it has to give the roll freedom back.
        if (rollMode is { } requestedRollMode)
            fc.RollMode = requestedRollMode;
        else if (_s.Running)
        {
            double altM = r.Length() - parent.MeanRadius;
            // An ascent engaged on the pad rolls as soon as it is clear of it (PadHold), while it still climbs straight up: the roll program (RollProgramHolds) holds the pitch-over until the roll is done. ForceRollMinAltM, above the mean radius, only gates an ascent engaged in flight, which has no pad to clear; on a pad near sea level it came 150 m after the pitch-over had begun.
            bool clear = double.IsFinite(_s.LiftoffRadius) ? !PadHold(r) : altM >= ForceRollMinAltM;
            fc.RollMode = _s.ForceRoll && clear
                ? FlightComputerRollMode.Up
                : FlightComputerRollMode.Decoupled;
        }

        if (fullEngage)
        {
            fc.AttitudeFrame = VehicleReferenceFrame.EclBody;
            fc.TrackTarget(FlightComputerAttitudeTrackTarget.Custom);
        }
        _s.AttitudeOwnership.EndWrite(fc, writesRoll: _s.Running || rollMode.HasValue);
    }

    /// <summary>
    /// The ascent's roll reference: the target orbit plane, turned by the roll the
    /// vehicle ALREADY HAD when guidance engaged.
    ///
    /// The plane on its own is what makes the reference continuous - it is
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

        // The plane-referenced frame perpendicular to the thrust axis. MINUS the normal, so that with the steering along the velocity this is cross(v, r) - the same direction the stock construction produces where it works.
        double3 baseRef = -AscentPlaneNormal();
        double3 yRef = baseRef - double3.Dot(baseRef, x) * x;
        if (yRef.Length() < 1e-9)
            return baseRef;                     // SteerBody2Cci substitutes for this
        yRef = double3.Normalize(yRef);
        double3 zRef = double3.Cross(x, yRef);

        // Forced: the angle the user asked for, in this same frame. Read live rather than latched, so turning the dial moves the vehicle.
        //
        // The latch is deliberately NOT taken while forcing. Un-ticking the box has to leave the vehicle holding the roll it is in AT THAT MOMENT, and latching an engagement-time measurement would instead roll it back to lift-off.
        if (_s.ForceRoll)
        {
            _s.RollLatched = false;
            double forced = UpfgTarget.DegToRad(_s.ForceRollDeg);
            return Math.Cos(forced) * yRef + Math.Sin(forced) * zRef;
        }

        // Latched once per engagement, from the vehicle's own body Y - the axis ComputeBurnBody2Cci puts the roll reference on.
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
    /// The roll <see cref="AscentRollRef"/> will hold, radians, in its frame about the thrust axis built on the plane <paramref name="normal"/>: the forced roll, the latched one, or the one the vehicle has now, which the latch takes at engagement. For the convex plan's body-axis force (AscentProblem.RollAngleRad); reads nothing it changes.
    /// </summary>
    internal static double AscentRollAngle(Vehicle vehicle, double3 normal)
    {
        if (_s.ForceRoll)
            return UpfgTarget.DegToRad(_s.ForceRollDeg);
        if (_s.RollLatched)
            return _s.RollOffset;
        double3 x = KsaFrameBridge.ThrustAxisCci(vehicle);
        if (!(x.Length() > 0.5))
            return 0.0;
        x = double3.Normalize(x);
        double3 yRef = -normal - double3.Dot(-normal, x) * x;
        if (!(yRef.Length() > 1e-9))
            return 0.0;
        yRef = double3.Normalize(yRef);
        double3 zRef = double3.Cross(x, yRef);
        double3 bodyY = new double3(0, 1, 0).Transform(KsaFrameBridge.BodyToCci(vehicle));
        double c = double3.Dot(bodyY, yRef), s = double3.Dot(bodyY, zRef);
        return (Math.Abs(c) > 1e-9 || Math.Abs(s) > 1e-9) ? Math.Atan2(s, c) : 0.0;
    }

    /// <summary>
    /// The body->CCI orientation that points thrust along <paramref name="steerDir"/>,
    /// built exactly as KSA's BurnTarget.ComputeBurnBody2Cci builds it (body X along
    /// thrust, body Y the roll reference, body Z their cross) but with the roll
    /// reference supplied instead of derived from the position vector.
    ///
    /// WHY NOT THE STOCK ONE, ON AN ASCENT. ComputeBurnBody2Cci takes the roll
    /// reference from cross(steer, position) - and on an ascent those two vectors are
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
    /// plane does - i.e. the vehicle flies wings-level to its own orbital plane from
    /// the pad to cutoff, with no discontinuity anywhere in between.
    /// </summary>
    /// <summary>
    /// A returning booster's roll reference in CCI now (VehicleAutopilotState.ReturnRollRefCcf): the direction its wing, body +y, is held along - up x d, with up the local vertical and d the horizontal direction to the site, both where the booster is NOW (UpdateReturnRollRef). The body's +y is that direction less its part along the axis (SteerBody2Cci, BodyAeroTable.WorldCoefficients).
    ///  WINGS LEVEL TO THE LOCAL HORIZON, SQUARE TO THE TRACK TO THE SITE. up x d is horizontal, so the booster pitches in the vertical plane through it and the site, and its roll on the navball stays 0 through the boostback burn, the flip, the glide and entry guidance. It is undefined only for an axis along it, flat and square to the track, which a returning booster never flies.
    ///  LIVE, NOT FROZEN (2026-11-21). Until then w was taken once, at first use, and frozen: the normal of the great circle through the booster and the site at that moment. Once the booster was off that circle its pitch plane no longer pointed at the site, and it could only correct the cross-range by skidding across its own wings. It now follows the bearing to the site, turned at no more than ReturnRollTurnDegS so the craft's roll (and the plan's held roll) keep up with it, and stands still within ReturnRollFreezeM of the site, where the bearing to it swings.
    ///  Until 2026-11-20 the reference was w = (d - up) / sqrt 2, 45 deg below the horizon towards the site, with the wing along axis x w. An engine-first glide's axis is the reverse of its velocity, so a glide path through 45 deg put the axis along -w: there axis x w had no direction and swung with the slightest skid, and either side of it pointed opposite ways - the booster rolled through the boostback glide as if it kept redefining "up".
    /// </summary>
    internal static double3 ReturnRollRefCci(Vehicle vehicle, IParentBody parent)
    {
        doubleQuat cci2Ccf = parent.GetCci2Ccf();
        double now = SimNow();
        if (_s.ReturnRollRefCcf.Length() < 0.5 || now > _s.ReturnRollRefAt)
            UpdateReturnRollRef(vehicle, parent, cci2Ccf, now);
        double3 w = _s.ReturnRollRefCcf.Transform(doubleQuat.Inverse(cci2Ccf));
        // The glide's roll offset (UpdateGlideRoll), about the engine-first axis the airflow sets: zero until entry guidance's glide picks one.
        double phi = _s.ReturnRollOffsetRad;
        if (phi == 0.0 || !double.IsFinite(phi))
            return w;
        double3 axis = GlideRollAxisCci(vehicle, parent);
        return axis.Length() > 0.5 ? RotateAbout(w, axis, phi) : w;
    }

    /// <summary>The fastest the held wing direction turns to follow the bearing to the site, deg/s: the glide controller's own roll capture rate (AttitudeController.Settings.CaptureRateDegS), so the craft's roll keeps up with it.</summary>
    private const double ReturnRollTurnDegS = 5.0;

    /// <summary>Within this horizontal distance of the site, m, the held wing direction stops following the bearing to it: over the site the bearing swings round, and a small miss would roll the craft through it in the last seconds.</summary>
    private const double ReturnRollFreezeM = 1000.0;

    /// <summary>
    /// The held wing direction, once a sim step (ReturnRollRefCci): up x d where the booster is now, d the horizontal direction to the site, turned towards from the last at no more than ReturnRollTurnDegS; within ReturnRollFreezeM of the site the last one, laid level to the local horizon. Taken outright the first time; over the site, with no wing direction yet, from the reversed horizontal velocity, where it came from.
    /// </summary>
    private static void UpdateReturnRollRef(Vehicle vehicle, IParentBody parent, doubleQuat cci2Ccf, double now)
    {
        double dt = double.IsFinite(_s.ReturnRollRefAt) ? Math.Clamp(now - _s.ReturnRollRefAt, 0.0, 1.0) : 0.0;
        _s.ReturnRollRefAt = now;
        double3 r = vehicle.Orbit.StateVectors.PositionCci;
        double3 up = r.NormalizeOrZero();
        double3 site = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        double3 toSite = site - r;
        double3 d = toSite - double3.Dot(toSite, up) * up;
        bool held = _s.ReturnRollRefCcf.Length() > 0.5;
        double3 live = d.Length() >= ReturnRollFreezeM || !held ? d : default;
        if (!held && live.Length() < 1.0)
        {
            double3 vAir = vehicle.Orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), r);
            live = -(vAir - double3.Dot(vAir, up) * up);
            if (live.Length() < 1e-6)
                live = double3.Cross(up, Math.Abs(up.Z) < 0.9 ? new double3(0, 0, 1) : new double3(1, 0, 0));
        }
        double3 want = live.Length() > 1e-6 ? double3.Cross(up, double3.Normalize(live)).NormalizeOrZero() : default;
        if (!held)
        {
            _s.ReturnRollRefCcf = want.Transform(cci2Ccf);
            return;
        }
        // The last one, level to the horizon here.
        double3 last = _s.ReturnRollRefCcf.Transform(doubleQuat.Inverse(cci2Ccf));
        last = (last - double3.Dot(last, up) * up).NormalizeOrZero();
        if (last.Length() < 0.5)
            last = want;
        if (want.Length() > 0.5 && last.Length() > 0.5)
        {
            double turn = Math.Atan2(double3.Dot(double3.Cross(last, want), up), double3.Dot(last, want));
            double step = ReturnRollTurnDegS * Math.PI / 180.0 * dt;
            last = RotateAbout(last, up, Math.Clamp(turn, -step, step)).NormalizeOrZero();
        }
        if (last.Length() > 0.5)
            _s.ReturnRollRefCcf = last.Transform(cci2Ccf);
    }

    /// <summary>The engine-first axis of a glide, CCI: the reverse of the craft's velocity through the air. Zero when it is not moving through it.</summary>
    private static double3 GlideRollAxisCci(Vehicle vehicle, IParentBody parent)
    {
        double3 r = vehicle.Orbit.StateVectors.PositionCci;
        double3 vAir = vehicle.Orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), r);
        return vAir.Length() > 1.0 ? -double3.Normalize(vAir) : default;
    }

    /// <summary>How often the glide's roll is chosen again, s, and how fast it may move, deg/s: the attitude controller's own capture rate, so the plan's held roll and the body's stay together.</summary>
    private const double GlideRollEveryS = 0.5, GlideRollRateDegS = 5.0;

    /// <summary>The least gain in rolling moment worth rolling for: an absolute floor, N m, and a share of the moment at the held roll.</summary>
    private const double GlideRollMinGainNm = 1000.0, GlideRollMinGainShare = 0.3;

    /// <summary>
    /// THE GLIDE'S ROLL (ThreeDofLeastRollMoment): rolls the held wing direction toward the nearest roll at which the air's rolling moment is zero and restoring, read from the body moment table at the airflow the commanded attitude meets (AttitudeController.TryLeastRollMoment), at GlideRollRateDegS. A craft that trims there holds its roll by itself, and the fins its pitch and yaw need are not also cancelling a roll - on the 2026-11-20 flight a steady -6.7 kN m of roll, with fin 4 on its stop and fin 2 in the lee. Every consumer of the held roll (entry guidance's model, the steering, the landing-point prediction) reads it through ReturnRollRefCci, so they move with it. Main thread.
    /// </summary>
    private static void UpdateGlideRoll(Vehicle vehicle, IParentBody parent, double3 dir)
    {
        if (!_s.ThreeDofLeastRollMoment || !_s.BoosterRollHold)
            return;
        double now = SimNow();
        if (now - _s.GlideRollAt < GlideRollEveryS)
            return;
        double dt = double.IsFinite(_s.GlideRollAt) ? Math.Min(now - _s.GlideRollAt, 2.0) : GlideRollEveryS;
        _s.GlideRollAt = now;
        double3 r = vehicle.Orbit.StateVectors.PositionCci;
        double3 vAir = vehicle.Orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), r);
        doubleQuat target = SteerBody2Cci(dir, ReturnRollRefCci(vehicle, parent));
        double3 airBody = vAir.Transform(doubleQuat.Inverse(target));
        if (!AttitudeController.TryLeastRollMoment(_s.Aero, airBody, vehicle.PhysicsEnvironment.AtmosphericDensity, _s.MomentLimiterPerPa,
                                                   out double delta, out double nowNm, out double bestNm))
            return;
        _s.GlideRollMomentNm = nowNm;
        _s.GlideRollBestNm = bestNm;
        // Only for a real gain: the least moment is shallow and moves with the flight, and chasing every move of it swung the roll 40 deg each way in the harness (2026-11-20).
        if (Math.Abs(nowNm) - Math.Abs(bestNm) < Math.Max(GlideRollMinGainNm, GlideRollMinGainShare * Math.Abs(nowNm)))
            delta = 0.0;
        double step = GlideRollRateDegS * Math.PI / 180.0 * dt;
        double phi = _s.ReturnRollOffsetRad + Math.Clamp(delta, -step, step);
        _s.ReturnRollOffsetRad = Math.IEEERemainder(phi, 2.0 * Math.PI);
        // Touchdown guidance lands at this roll (TerminalAttitude): the held wing direction in the site frame, where it stays put as the planet turns.
        double3 siteCci = SiteDirCciAt(parent, 0) * (parent.MeanRadius + SiteTerrainHeight(parent));
        _s.GlideWingSite = KsaFrameBridge.BuildSiteFrame(siteCci).VecToLocal(ReturnRollRefCci(vehicle, parent)).NormalizeOrZero();
    }

    /// <summary>Whether AFC's own attitude controller flies this step, with a command to fly: an ascent (AscentOwnAttitude), or a returning booster gliding with its engine off - entry guidance's glide, or the boostback's (BoostbackGlideOwnAttitude) - with the setting on.</summary>
    private static bool OwnsAttitude(Vehicle vehicle, bool boostbackGuides, bool threeDofGuides)
    {
        if (_s.CommandDir.Length() <= 0.5)
            return false;
        if (_s.Running)
            return _s.AscentOwnAttitude;
        if (!_s.ThreeDofOwnAttitude)
            return false;
        if (threeDofGuides)
            return _s.ThreeDofPhase == ThreeDofPhase.Glide;
        return boostbackGuides && _s.BoostbackGlideOwnAttitude && _s.BoostbackPhase == BoostbackPhase.Glide
               && !(KsaEnginePerf.LiveEngineThrust(vehicle) > 0.0);
    }

    /// <summary>
    /// One step of AFC's own attitude controller (AttitudeController) on the commanded direction and roll: the stock flight computer held at Manual so it steers nothing, the target built as CommandAttitude builds it, and the craft's surfaces and RCS driven for it.
    ///  ON AN ASCENT the gimbals take the torque first and the RCS the rest; the fins only with STEER WITH FINS (AscentAeroControl), held at rest otherwise; the roll held only where the stock flight computer would track it (AscentHoldsRoll), free with its rate damped elsewhere; and the air's moment read off the ascent's nose-first tables (AscentControllerAero).
    /// </summary>
    /// <param name="heldRoll">The roll is the returning booster's held one (ReturnRollRefCci), which the glide may turn to its least rolling moment (UpdateGlideRoll).</param>
    private static void FlyOwnAttitude(Vehicle vehicle, IParentBody parent, double3? rollRef, bool heldRoll)
    {
        if (!TryTakeCraft(vehicle, acquire: true))
            return;
        FlightComputer fc = vehicle.FlightComputer;
        if (fc.AttitudeMode != FlightComputerAttitudeMode.Manual)
        {
            KsaAttitudeRate.Clear(vehicle);
            _s.AttitudeOwnership.BeginWrite(fc);
            fc.AttitudeMode = FlightComputerAttitudeMode.Manual;
            _s.AttitudeOwnership.EndWrite(fc, writesRoll: false);
        }
        double3 dir = double3.Normalize(_s.CommandDir);
        if (heldRoll && rollRef.HasValue)
        {
            UpdateGlideRoll(vehicle, parent, dir);
            rollRef = ReturnRollRefCci(vehicle, parent);
        }
        doubleQuat target = rollRef.HasValue
            ? SteerBody2Cci(dir, rollRef.Value)
            : BurnTarget.ComputeBurnBody2Cci(float3.Pack(double3.Normalize(vehicle.Orbit.StateVectors.PositionCci)), float3.Pack(dir));
        bool ascent = _s.Running;
        // THE ASCENT'S GAINS are its own (AscentAttitudeBandwidthRadS, AscentAttitudeDamping, AscentRateFilterS): a stack unstable nose first under gimbals that lag their command oscillated 2 to 5.5 deg about the command through max q at the glide's 1 rad/s, damping 0.8 and 0.2 s rate filter, with or without the filter, where stock held 0.05 to 0.4; at 2 rad/s, damping 1 and a 0.05 s filter it held 0.04 to 0.35 (harness, Test Vehicle 1, 2026-11-21). The fins' reason for the filter - chasing rate jitter - is not the gimbals'.
        var settings = new AttitudeController.Settings(ascent ? _s.AscentAttitudeBandwidthRadS : _s.AttitudeBandwidthRadS,
                                                       ascent ? _s.AscentAttitudeDamping : AttitudeController.Settings.Default.Damping,
                                                       _s.AttitudeDeadbandDeg, AttitudeController.Settings.Default.RateDeadbandDegS)
        {
            UseSurfaces = !ascent || _s.AscentAeroControl,
            UseGimbals = ascent,
            RollFree = ascent && !AscentHoldsRoll(vehicle, parent),
        };
        if (ascent)
            settings = settings with { RateFilterS = _s.AscentRateFilterS, RollCaptureRateDegS = _s.AscentRollRateDegS, RollBandwidthRadS = _s.AscentRollBandwidthRadS };
        KsaAeroSweep.Result aero = _s.Aero;
        if (ascent)
        {
            aero = AscentControllerAero(vehicle, parent);
            UpdateMomentLimiter(vehicle, SimNow());
        }
        AttitudeController.Report report = AttitudeController.Step(vehicle, target, _s.CommandRate, aero, _s.MomentLimiterPerPa, settings);
        if (ascent)
            LogAscentAttitude(vehicle, report, settings);
        _s.OwnAttitudeFlying = true;
        _s.OwnAttitudeVehicle = vehicle;
    }

    /// <summary>
    /// Whether the ascent holds its roll this step, by the stock flight computer's rule (CommandAttitude): only with FORCE ROLL on, and only clear of the pad (PadHold), or ForceRollMinAltM up for an ascent engaged in flight. Elsewhere the roll is free.
    /// </summary>
    private static bool AscentHoldsRoll(Vehicle vehicle, IParentBody parent)
    {
        if (!_s.ForceRoll)
            return false;
        double3 r = vehicle.Orbit.StateVectors.PositionCci;
        return double.IsFinite(_s.LiftoffRadius) ? !PadHold(r) : r.Length() - parent.MeanRadius >= ForceRollMinAltM;
    }

    /// <summary>How long the ascent's controller waits after a failed aero sweep before it asks again, s.</summary>
    private const double AscentAeroRetryS = 5.0;

    /// <summary>The dynamic pressure under which a new sweep is not worth asking for, Pa: the feedforward reads no air below 50 Pa (AttitudeController.TryAirMoment).</summary>
    private const double AscentAeroMinQPa = 50.0;

    /// <summary>
    /// The nose-first aero tables the ascent's controller reads the air's moment from (AscentFlightAero): the plan's sweep while the craft is the craft it swept, then, once staging has changed its aero mesh, a new sweep, nose first, on its own thread. Null - no feedforward, the feedback and integral alone - while none matches the craft.
    /// </summary>
    private static KsaAeroSweep.Result AscentControllerAero(Vehicle vehicle, IParentBody parent)
    {
        ulong live = KsaAeroSweep.LiveMeshHash(vehicle);
        KsaAeroSweep.Result have = _s.AscentFlightAero;
        if (have != null && have.MeshHash == live && ReferenceEquals(have.Parent, parent))
            return have;
        double now = SimNow();
        if (_s.AscentFlightAeroJob != null)
        {
            if (!_s.AscentFlightAeroJob.TryTake(out KsaAeroSweep.Result swept, out string error))
                return null;
            _s.AscentFlightAeroJob = null;
            if (swept == null)
            {
                _s.AscentFlightAeroRetryAt = now + AscentAeroRetryS;
                GuidanceLog.Info(vehicle, $"ascent attitude: no aero table ({GuidanceLog.Cut(error, 39)}); steering without the air's moment.");
                return null;
            }
            _s.AscentFlightAero = swept;
            GuidanceLog.Debug(vehicle, swept.NoseBodyMoment != null
                ? "ascent attitude: aero tables of the craft as it is now, nose first."
                : "ascent attitude: the new aero tables have no nose-first moment; steering without the air's moment.");
            return swept.MeshHash == live && ReferenceEquals(swept.Parent, parent) ? swept : null;
        }
        if (now < _s.AscentFlightAeroRetryAt || KsaAeroSweep.MeshRebuilding(vehicle))
            return null;
        float3 airF = PhysicsStates.ComputeAirVelocityBody(vehicle.BubbleOrigin, vehicle.KinematicStates, vehicle.PhysicsEnvironment);
        double airSpeed = double3.Unpack(in airF).Length();
        if (!(0.5 * vehicle.PhysicsEnvironment.AtmosphericDensity * airSpeed * airSpeed > AscentAeroMinQPa))
            return null;
        if (KsaAeroSweep.TryStart(vehicle, parent, now, out KsaAeroSweep.Job job, out _, noseFirst: true))
            _s.AscentFlightAeroJob = job;
        else
            _s.AscentFlightAeroRetryAt = now + AscentAeroRetryS;
        return null;
    }

    /// <summary>How often the ascent's controller writes its debug line, s.</summary>
    private const double AscentAttitudeLogEveryS = 2.0;

    /// <summary>The ascent controller's state every AscentAttitudeLogEveryS, Debug builds: error, and in kN m about the model's axes the demand, the air's moment it is asked against, and what the gimbals, fins and RCS give.</summary>
    private static void LogAscentAttitude(Vehicle vehicle, in AttitudeController.Report r, in AttitudeController.Settings settings)
    {
        if (!GuidanceLog.Enabled)
            return;
        double now = SimNow();
        if (now - _s.AscentAttitudeLogAt < AscentAttitudeLogEveryS)
            return;
        _s.AscentAttitudeLogAt = now;
        static string K(double3 v) => FormattableString.Invariant($"({v.X / 1000.0:F1}, {v.Y / 1000.0:F1}, {v.Z / 1000.0:F1})");
        GuidanceLog.Debug(vehicle, FormattableString.Invariant(
            $"ascent attitude: {r.ErrorDeg:F2} deg, {r.RateErrorDegS:F2} deg/s off{(settings.RollFree ? ", roll free" : "")}; kN m demand {K(r.DemandNm)}, air {(r.AirKnown ? K(r.AirNm) : "unknown")}, gimbals {K(r.GimbalsNm)}, fins {(settings.UseSurfaces ? K(r.SurfacesNm) : "off")}, RCS {K(r.RcsNm)} at {r.RcsFlowKgS:F2} kg/s."));
    }

    /// <summary>Lets AFC's own attitude controller go: its surfaces home and its RCS released, and the stock flight computer engaged in full on its next command (CommandAttitude's fullEngage).</summary>
    internal static void StopOwnAttitude(Vehicle vehicle)
    {
        AttitudeController.Release(vehicle);
        _s.OwnAttitudeFlying = false;
        _s.OwnAttitudeVehicle = null;
        _s.WasEngaged = false;
    }

    private static doubleQuat SteerBody2Cci(double3 steerDir, double3 rollRef)
    {
        double3 x = double3.Normalize(steerDir);

        // Component of the reference perpendicular to the thrust axis. Degenerate only if the two are parallel, which the plane normal never is on an ascent.
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

    // The steering direction expressed as the same pitch/heading numbers the in-game navball shows in its surface (EnuBody) frame. Computed with KSA's own functions (ComputeBurnBody2Cci + EnuBody frame + RollPitchYaw decomposition + compass wrap) so the readout matches the navball digit-for-digit. Note KSA's ENU frame is East-referenced, so this differs from a real-world compass azimuth by 90 deg.
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

    // Small math helpers.

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
    internal static double3 RotateAbout(double3 vec, double3 axis, double angle)
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

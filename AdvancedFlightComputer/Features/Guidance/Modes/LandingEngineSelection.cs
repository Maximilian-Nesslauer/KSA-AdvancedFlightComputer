using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Ui.Logic;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

internal sealed class LandingEngineActivation(LandingEngineGroup group, EngineController[] engines, double deadline)
{
    internal readonly LandingEngineGroup Group = group;
    internal readonly EngineController[] Engines = engines;
    internal HashSet<Rocket> SuppliedRockets = engines.SelectMany(engine => engine.Rockets).ToHashSet();
    internal readonly double Deadline = deadline;
    internal bool Sent;
    internal bool Ready;
    internal bool IgnitionRecorded;
    internal PartTree? Tree;
    internal Part[] Parts = [];
    internal string Refusal = "";
    // The group's moment was more than its gimbals hold (GimbalsHoldMoment), so this activation flies every engine of the craft instead.
    internal bool Fallback;
}

/// <summary>Main engine output as the craft delivers it, for the landing group checks and log lines.</summary>
internal static class LandingEngineTelemetry
{
    /// <summary>True when a rocket of an engine controller has a throttle above zero. Vehicle.ComputeLiveThrust also counts RCS, so it cannot tell a lit main engine.</summary>
    internal static bool MainEnginesFiring(Vehicle vehicle)
    {
        ReadOnlySpan<RocketState> states = RocketStates.Of(vehicle.Parts);
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
            foreach (Rocket rocket in engine.Rockets)
                if ((uint)rocket.StatesIdx < (uint)states.Length && states[rocket.StatesIdx].Throttle > 0)
                    return true;
        return false;
    }

    internal static (double Thrust, double Flow) MeasuredMainEngineOutput(Vehicle vehicle)
    {
        float3 force = float3.Zero;
        double flow = 0.0;
        foreach (var nozzle in vehicle.Parts.RocketNozzles.ModulesAndStates)
        {
            if (nozzle.Module.Rocket?.Controller is not EngineController)
                continue;
            force += nozzle.State.Performance.TotalThrust * nozzle.State.ThrustDirectionVehicleAsmb;
            flow += nozzle.State.Performance.MassFlowRate;
        }
        return (force.Length(), flow);
    }
}

// The landing group works through the stock engine switch alone: AFC activates the selected controllers and deactivates the others before entry guidance builds its model. Entry guidance, touchdown guidance and their thrust and gimbal models read the active, supplied engines, so they fly the group without a reference to it.
public static partial class GuidanceWindow
{
    private enum LandingEnginePreparation { Ready, Waiting, Refused }
    private const double LandingEngineActivationLimitS = 2.0;

    private static LandingEngineGroup? SelectedLandingEngines
        => _s.GetEngineGroups(LandingEngineGroups.CraftScope).Chosen;

    /// <summary>The share of the gimbals' torque an engine set's own moment may take: the rest is the plan's turns and the attitude feedback's.</summary>
    internal const double LandingMomentShare = 0.8;

    /// <summary>True when the gimbals of these engines can hold the moment the engines make about the centre of mass now within <see cref="LandingMomentShare"/> of what they can give (KsaThrustMoment.LateralAgainstGimbals). The moment and the capability are both in proportion to the thrust, so one check holds at every throttle.</summary>
    private static bool GimbalsHoldMoment(Vehicle vehicle, EngineController[] engines, out double momentNm, out double capabilityNm)
    {
        (momentNm, capabilityNm) = KsaThrustMoment.LateralAgainstGimbals(vehicle, engines);
        return momentNm <= LandingMomentShare * capabilityNm;
    }

    /// <summary>Every engine controller of the craft that the landing models can fly: throttleable rockets only.</summary>
    private static EngineController[] AllLandingEngines(Vehicle vehicle)
    {
        var all = new List<EngineController>();
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
            if (engine.Rockets.Length > 0 && engine.Rockets.All(KsaEnginePerf.FollowsThrottle))
                all.Add(engine);
        return all.ToArray();
    }

    private static bool CanSwitchEnginesManually(Vehicle vehicle)
        => vehicle.IsControllable && vehicle.FlightComputer?.BurnMode == FlightComputerBurnMode.Manual
            && !VehicleControlOwnership.KfsOwns(vehicle)
            && VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.None;

    private static bool CanReuseLandingEngineGroup(Vehicle vehicle)
        => _s.EngineActivation is { Ready: true } activation
            && ReferenceEquals(activation.Group, SelectedLandingEngines) && LandingEngineGroupMatches(vehicle, activation)
            && LandingGimbalsRefreshed(vehicle)
            && !ManualInputs(vehicle).EngineOn && !LandingEngineTelemetry.MainEnginesFiring(vehicle);

    private static void ApplyManualEngineChange(Vehicle vehicle, ManualEngineChange change)
    {
        var candidates = LandingEngineGroups.Controllers(vehicle, vehicle.Parts.Root);
        if (!LandingEngineGroups.Resolve(new LandingEngineGroup("", change.Engines), candidates, out EngineController[] engines))
        {
            _s.WarnLanding("ENGINE COMMAND REFUSED: A CONTROLLER IS MISSING OR AMBIGUOUS.");
            return;
        }
        LandingEngineGroups.Activate(vehicle, engines, change.EnableSingle);
    }

    private static LandingEnginePreparation PrepareSelectedLandingEngines(Vehicle vehicle, double now, bool fromGlide, out string reason)
    {
        reason = "";
        if (SelectedLandingEngines is not { } group)
            return LandingEnginePreparation.Ready;

        if (vehicle.Parts?.Root == null || vehicle.FlightComputer?.BurnMode != FlightComputerBurnMode.Manual
            || !vehicle.IsControllable || VehicleControlOwnership.KfsOwns(vehicle)
            || VehicleControlOwnership.HolderOf(vehicle) == ControlClaimant.RcsTranslation)
        {
            reason = "LANDING GROUP NEEDS MANUAL ENGINE CONTROL.";
            return LandingEnginePreparation.Refused;
        }

        if (_s.EngineActivation is not { } activation || !ReferenceEquals(group, activation.Group))
        {
            var candidates = LandingEngineGroups.Controllers(vehicle, vehicle.Parts.Root);
            if (!LandingEngineGroups.Resolve(group, candidates, out EngineController[] engines)
                || engines.Any(engine => engine.Rockets.Length == 0 || engine.Rockets.Any(rocket => !KsaEnginePerf.FollowsThrottle(rocket))))
            {
                reason = $"LANDING GROUP '{group.Name}' HAS A MISSING OR UNSUPPORTED ENGINE.";
                return LandingEnginePreparation.Refused;
            }
            if (SplitsLandingGimbal(vehicle, engines))
            {
                reason = $"LANDING GROUP '{group.Name}' SHARES A GIMBAL WITH AN ENGINE OUTSIDE THE GROUP.";
                return LandingEnginePreparation.Refused;
            }
            // The landing models take the throttle floor of the whole craft (PartTree.EngineThrottleMin counts every installed controller), so a group that cannot throttle that low would fly a floor they do not know.
            if (engines.Any(engine => engine.MinimumThrottle > vehicle.GetMinThrottle() + 1e-4f))
            {
                reason = $"LANDING GROUP '{group.Name}' CANNOT THROTTLE AS LOW AS THE OTHER ENGINES OF THE CRAFT.";
                return LandingEnginePreparation.Refused;
            }
            // THE MOMENT, from the centre of mass as it is when entry guidance engages: the arm shrinks as the propellant burns and the centre of mass comes down, so the engage is where it is largest. Over what the gimbals can hold, the landing goes on all engines instead.
            bool fallback = false;
            if (!GimbalsHoldMoment(vehicle, engines, out double momentNm, out double capabilityNm))
            {
                EngineController[] all = AllLandingEngines(vehicle);
                bool allLand = all.Length > engines.Length && !SplitsLandingGimbal(vehicle, all)
                    && GimbalsHoldMoment(vehicle, all, out _, out _);
                string share = capabilityNm > 0.0 ? FormattableString.Invariant($"{momentNm / capabilityNm:P0} OF") : "MORE THAN";
                GuidanceLog.Info(vehicle, FormattableString.Invariant(
                    $"Landing group '{group.Name}' moment {momentNm:F0} N m against {capabilityNm:F0} N m its gimbals hold, limit {LandingMomentShare:P0}: {(allLand ? "landing on all engines" : "all engines cannot land either")}."));
                if (!allLand)
                {
                    reason = $"LANDING GROUP '{group.Name}' NEEDS {share} ITS GIMBALS TO HOLD ITS MOMENT, AND ALL ENGINES CANNOT LAND EITHER.";
                    return LandingEnginePreparation.Refused;
                }
                _s.WarnLanding($"LANDING GROUP '{group.Name}' NEEDS {share} ITS GIMBALS TO HOLD ITS MOMENT: LANDING ON ALL ENGINES.");
                engines = all;
                fallback = true;
            }
            double wait = fromGlide && double.IsFinite(_s.GlideTgo)
                ? Math.Clamp(0.1 * _s.GlideTgo, 0.1, LandingEngineActivationLimitS) : LandingEngineActivationLimitS;
            _s.EngineActivation = activation = new LandingEngineActivation(group, engines, now + wait) { Fallback = fallback };
        }

        if (activation.Refusal.Length > 0)
        {
            reason = activation.Refusal;
            return LandingEnginePreparation.Refused;
        }
        if (ManualInputs(vehicle).EngineOn || LandingEngineTelemetry.MainEnginesFiring(vehicle)
            || KsaEnginePerf.RemainingMinimumPulse(vehicle) > 0.0)
        {
            if (fromGlide && now < activation.Deadline)
                return LandingEnginePreparation.Waiting;
            reason = activation.Refusal = "SHUT DOWN THE ENGINES BEFORE SWITCHING TO THE LANDING GROUP.";
            return LandingEnginePreparation.Refused;
        }

        if (!activation.Sent)
        {
            if (!(now < activation.Deadline))
            {
                reason = activation.Refusal = $"NO TIME TO CONFIRM LANDING GROUP '{group.Name}' BEFORE THE BURN.";
                return LandingEnginePreparation.Refused;
            }
            // EngineController.SetIsActive is buffered through the input events, so the change shows on a later frame.
            LandingEngineGroups.Activate(vehicle, activation.Engines);
            activation.Sent = true;
            GuidanceLog.Info(vehicle, $"Landing group '{group.Name}' activation requested at {now:F3} s: {DescribeSelection(activation)}.");
        }

        if (!LandingEngineGroupMatches(vehicle, activation) || !LandingGimbalsRefreshed(vehicle))
        {
            if (now < activation.Deadline)
                return LandingEnginePreparation.Waiting;
            reason = activation.Refusal = $"LANDING GROUP '{group.Name}' ACTIVATION, SUPPLY OR GIMBALS COULD NOT BE CONFIRMED ({LandingGroupMismatch(vehicle, activation)}).";
            GuidanceLog.Info(vehicle, reason);
            return LandingEnginePreparation.Refused;
        }

        IParentBody? parent = vehicle.Orbit?.Parent;
        if (parent == null)
        {
            reason = "LANDING GROUP CANNOT BE CHECKED WITHOUT A PARENT BODY.";
            return LandingEnginePreparation.Refused;
        }
        double altitude = SiteTerrainHeight(parent);
        double pressure = KsaEnginePerf.AmbientPressureAt(parent, altitude);
        double radius = parent.MeanRadius + altitude;
        double lift = KsaEnginePerf.ActiveThrustCapability(vehicle, pressure);
        double g = parent.Mu / (radius * radius);
        double maxAcceleration = lift / vehicle.TotalMass;
        if (_s.LandingAccelLimitG > 0)
            maxAcceleration = Math.Min(maxAcceleration, LandingAccelLimitMs2);
        if (!(maxAcceleration > g))
        {
            reason = activation.Refusal = $"LANDING GROUP '{group.Name}' HAS INSUFFICIENT MAXIMUM THRUST TO BRAKE.";
            return LandingEnginePreparation.Refused;
        }
        if (!activation.Ready)
        {
            activation.Tree = vehicle.Parts;
            activation.Parts = vehicle.Parts.Parts.ToArray();
            activation.Ready = true;
            double floor = KsaEnginePerf.ThrustAtThrottle(vehicle, 0.0, pressure);
            GuidanceLog.Info(vehicle, $"Landing group '{group.Name}' ready at {now:F3} s: {DescribeSelection(activation)}, {floor:F0} to {lift:F0} N at {pressure:F0} Pa.");
        }
        return LandingEnginePreparation.Ready;
    }

    /// <summary>True when exactly the group's controllers are active and exactly the expected rockets have propellant. With <paramref name="surviving"/>, a rocket that lost its supply is allowed and the supplied ones are collected.</summary>
    private static bool LandingEngineGroupMatches(Vehicle vehicle, LandingEngineActivation activation, HashSet<Rocket>? surviving = null)
    {
        PartTree tree = vehicle.Parts;
        if (tree == null || !ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
                .TryGetFrom(tree.States, out var controllers))
            return false;
        ReadOnlySpan<RocketState> states = RocketStates.Of(tree);
        int count = 0;
        foreach (var current in controllers.ModulesAndStates)
        {
            EngineController engine = current.Module;
            // A queued activation change is still pending while the state lags the module.
            if (current.State.WasActive != engine.IsActive)
                return false;
            if (!engine.IsActive)
                continue;
            if (Array.IndexOf(activation.Engines, engine) < 0)
                return false;
            count++;
            bool controllerFed = false;
            foreach (Rocket rocket in engine.Rockets)
            {
                if ((uint)rocket.StatesIdx >= (uint)states.Length)
                    return false;
                bool fed = states[rocket.StatesIdx].IsPropellantAvailable;
                bool expected = activation.SuppliedRockets.Contains(rocket);
                if (fed && !expected || !fed && expected && surviving == null)
                    return false;
                if (fed)
                {
                    controllerFed = true;
                    surviving?.Add(rocket);
                }
            }
            if (current.State.IsPropellantAvailable != controllerFed)
                return false;
        }
        return count == activation.Engines.Length;
    }

    /// <summary>The first reason the group does not match yet, for the refusal text: a switch still pending, a wrong engine on or off, missing propellant, or the flight computer's gimbal list.</summary>
    private static string LandingGroupMismatch(Vehicle vehicle, LandingEngineActivation activation)
    {
        PartTree tree = vehicle.Parts;
        if (tree == null || !ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
                .TryGetFrom(tree.States, out var controllers))
            return "NO ENGINE STATE";
        ReadOnlySpan<RocketState> states = RocketStates.Of(tree);
        foreach (var current in controllers.ModulesAndStates)
        {
            EngineController engine = current.Module;
            bool wanted = Array.IndexOf(activation.Engines, engine) >= 0;
            if (current.State.WasActive != engine.IsActive)
                return $"SWITCH OF {engine.TemplateId} STILL PENDING";
            if (engine.IsActive != wanted)
                return $"{engine.TemplateId} IS {(engine.IsActive ? "ON" : "OFF")}";
            if (!wanted)
                continue;
            foreach (Rocket rocket in engine.Rockets)
                if ((uint)rocket.StatesIdx < (uint)states.Length && activation.SuppliedRockets.Contains(rocket)
                    && !states[rocket.StatesIdx].IsPropellantAvailable)
                    return $"NO PROPELLANT AT {engine.TemplateId}";
        }
        return LandingGimbalsRefreshed(vehicle) ? "UNKNOWN" : "GIMBAL LIST NOT REFRESHED";
    }

    /// <summary>True when the flight computer's gimbal list (FlightComputer.ReadUpdatedVehicleConfiguration) already holds exactly the gimbals of the active engines.</summary>
    private static bool LandingGimbalsRefreshed(Vehicle vehicle)
    {
        var configured = vehicle.FlightComputer.VehicleConfig?.Gimbals;
        if (configured == null)
            return false;
        int active = 0;
        foreach (GimbalController gimbal in vehicle.Parts.Modules.Get<GimbalController>())
        {
            if (!gimbal.HasActiveEngine())
                continue;
            if (!configured.Contains(gimbal))
                return false;
            active++;
        }
        return active == configured.Count;
    }

    private static bool SplitsLandingGimbal(Vehicle vehicle, EngineController[] engines, HashSet<Rocket>? supplied = null)
    {
        foreach (GimbalController gimbal in vehicle.Parts.Modules.Get<GimbalController>())
        {
            bool selected = false, other = false;
            foreach (RocketNozzle nozzle in gimbal.Data.Nozzles)
            {
                if (nozzle.Rocket is { Controller: EngineController engine } rocket
                    && Array.IndexOf(engines, engine) >= 0 && (supplied == null || supplied.Contains(rocket)))
                    selected = true;
                else
                    other = true;
            }
            if (selected && other)
                return true;
        }
        return false;
    }

    /// <summary>True when a gimbal moves only dry rockets of a controller that stays active. Such a gimbal still counts as steering authority in the models, because they select gimbals by activation.</summary>
    private static bool HasDryActiveGimbal(Vehicle vehicle, EngineController[] engines, HashSet<Rocket> supplied)
    {
        foreach (GimbalController gimbal in vehicle.Parts.Modules.Get<GimbalController>())
        {
            bool onActive = false, fed = false;
            foreach (RocketNozzle nozzle in gimbal.Data.Nozzles)
            {
                if (nozzle.Rocket is not { Controller: EngineController engine } rocket || Array.IndexOf(engines, engine) < 0)
                    continue;
                onActive = true;
                fed |= supplied.Contains(rocket);
            }
            if (onActive && !fed)
                return true;
        }
        return false;
    }

    private static string DescribeSelection(LandingEngineActivation activation)
        => (activation.Fallback ? "(all engines, the group's moment was over its gimbals) " : "") + string.Join(", ", activation.Engines.Select(engine =>
            $"{engine.Parent.InstanceId}/{engine.TemplateId}[{string.Join("+", engine.Rockets.Where(activation.SuppliedRockets.Contains).Select(rocket => rocket.InstanceId))}]"));

    private static void StepLandingEngineCommit(Vehicle vehicle, double now)
    {
        if (_s.LandingEngineCommitPending is not { } mode)
            return;
        LandingEnginePreparation preparation = PrepareSelectedLandingEngines(vehicle, now, false, out string reason);
        _s.LandingGroupWaiting = preparation == LandingEnginePreparation.Waiting;
        if (_s.LandingGroupWaiting)
        {
            _s.WarnLanding("WAITING FOR THE LANDING ENGINES.");
            return;
        }
        _s.LandingEngineCommitPending = null;
        if (preparation == LandingEnginePreparation.Ready)
            RunCommit(vehicle, mode, enginesPrepared: true);
        else
            _s.WarnLanding(reason);
    }

    /// <summary>Checks once per step that the craft still flies the confirmed group. A lost supply in the glide restarts entry guidance with the remaining engines; any other change hands the craft back without an engine cut.</summary>
    private static bool LandingGroupStillValid(Vehicle vehicle)
    {
        if (_s.EngineActivation is not { Ready: true } activation
            || !_s.Active && !_s.EngagePending && !ThreeDofLive && !LandingMachineLive && _s.LandingPhase != LandingPhase.TerminalHover
            || HasTouchedDown(vehicle))
            return true;
        bool sameCraft = ReferenceEquals(activation.Group, SelectedLandingEngines)
            && ReferenceEquals(activation.Tree, vehicle.Parts)
            && vehicle.Parts.Parts.SequenceEqual(activation.Parts)
            && LandingGimbalsRefreshed(vehicle);
        if (sameCraft && LandingEngineGroupMatches(vehicle, activation))
        {
            if (!activation.IgnitionRecorded && LandingEngineTelemetry.MainEnginesFiring(vehicle))
            {
                activation.IgnitionRecorded = true;
                LogLandingEngineState(vehicle, SimNow(), "ignition");
            }
            return true;
        }
        // Only the supply changed when the active controllers are still exactly the group's.
        bool lostSupply = sameCraft && LandingEngineGroupMatches(vehicle, activation, new HashSet<Rocket>());
        if (lostSupply && TryReplanLandingGroupInGlide(vehicle, activation))
            return true;

        HandBackLandingGroup(vehicle, lostSupply
            ? $"LANDING GROUP '{activation.Group.Name}' LOST PROPELLANT SUPPLY. TAKE CONTROL."
            : $"LANDING GROUP '{activation.Group.Name}' CHANGED DURING DESCENT. TAKE CONTROL.");
        return false;
    }

    private static void HandBackLandingGroup(Vehicle vehicle, string reason)
    {
        LogLandingEngineState(vehicle, SimNow(), "handback");
        GuidanceLog.Info(vehicle, reason);
        _s.TakeoverStop = true;
        _s.TakeoverReason = reason;
        _s.ReleaseWithoutEngineCut = true;
        HandBackVehicle(vehicle);
        _s.WarnLanding(reason);
    }

    /// <summary>Ends a replan after its entry engage request ran: a refused engage hands the craft back, because entry guidance would otherwise fly on a model of engines that are switched off.</summary>
    private static void FinishLandingGroupReplan(Vehicle vehicle, bool engaged)
    {
        if (!_s.LandingGroupReplanPending)
            return;
        _s.LandingGroupReplanPending = false;
        if (engaged)
        {
            LogLandingEngineState(vehicle, SimNow(), "reduced supply");
            return;
        }
        string why = _s.LandingGroupRefusal.Length > 0 ? _s.LandingGroupRefusal : _s.ThreeDofStatus;
        HandBackLandingGroup(vehicle, $"LANDING GROUP LOST SUPPLY AND COULD NOT REPLAN ({why}). TAKE CONTROL.");
    }

    /// <summary>
    /// When rockets of the group lose their supply while entry guidance still glides with the engines off, and the rest can still stop the craft, the dry controllers are switched off and entry guidance is engaged again, so that it builds its model from the engines that remain.
    /// The dry controllers must go off and not only drop out of the supplied set, because the stock flight computer and the planning gimbals pick a gimbal by its active engine (GimbalController.HasActiveEngine), not by supply.
    /// </summary>
    private static bool TryReplanLandingGroupInGlide(Vehicle vehicle, LandingEngineActivation activation)
    {
        var surviving = new HashSet<Rocket>();
        if (_s.ThreeDofPhase != ThreeDofPhase.Glide || !ThreeDofLive || _s.LandingGroupReplanPending
            || ManualInputs(vehicle).EngineOn || LandingEngineTelemetry.MainEnginesFiring(vehicle)
            || !LandingEngineGroupMatches(vehicle, activation, surviving)
            || surviving.Count == 0 || surviving.Count >= activation.SuppliedRockets.Count
            || vehicle.Orbit?.Parent is not { } parent)
            return false;
        EngineController[] remaining = activation.Engines.Where(engine => engine.Rockets.Any(surviving.Contains)).ToArray();
        if (SplitsLandingGimbal(vehicle, remaining, surviving) || HasDryActiveGimbal(vehicle, remaining, surviving))
            return false;
        // The engines left can be off centre where the whole group was not (one of a pair ran dry): the same moment check as at the engage, from the centre of mass now.
        if (!GimbalsHoldMoment(vehicle, remaining, out double momentNm, out double capabilityNm))
        {
            GuidanceLog.Info(vehicle, FormattableString.Invariant(
                $"Landing group '{activation.Group.Name}' lost supply; the engines left make {momentNm:F0} N m against {capabilityNm:F0} N m their gimbals hold: not replanned."));
            return false;
        }

        double now = SimNow();
        double3 r = vehicle.Orbit.StateVectors.PositionCci;
        double gravity = parent.Mu / r.LengthSquared();
        double pressure = KsaEnginePerf.AmbientPressureAt(parent, r.Length() - parent.MeanRadius);
        double3 up = r.Normalized();
        double3 velocity = vehicle.Orbit.StateVectors.VelocityCci - double3.Cross(parent.GetAngularVelocityCci(), r);
        double height = TerminalHeight(vehicle.Orbit, parent, parent.MeanRadius);
        double braking = KsaEnginePerf.ActiveThrustCapability(vehicle, pressure) / vehicle.TotalMass
            * Math.Cos(15.0 * Math.PI / 180.0) - gravity;
        if (!(braking > 0.15 * gravity) || !(height > 0.0))
            return false;

        // A rough check that the remaining engines can still stop the fall after a new cold solve, which takes about this long.
        const double restartLead = 2.0;
        double sink = Math.Max(-double3.Dot(velocity, up), 0.0);
        double sinkAfterWait = sink + gravity * restartLead;
        double fall = sink * restartLead + 0.5 * gravity * restartLead * restartLead;
        double stop = sinkAfterWait * sinkAfterWait / (2 * braking);
        if (fall + stop >= height || (_s.ThreeDofPlan is { } plan && plan.IgnitionTime <= now + restartLead))
            return false;

        // The new activation switches the dry controllers off and is confirmed like the first one, then the engage request rebuilds entry guidance (FinishLandingGroupReplan).
        double wait = double.IsFinite(_s.GlideTgo) ? Math.Clamp(0.1 * _s.GlideTgo, 0.1, LandingEngineActivationLimitS) : LandingEngineActivationLimitS;
        _s.EngineActivation = new LandingEngineActivation(activation.Group, remaining, now + wait) { Fallback = activation.Fallback };
        _s.LandingGroupReplanPending = true;
        _s.ThreeDofEngagePending = true;
        _s.WarnEntry("LANDING GROUP LOST SUPPLY. REPLANNING WITH THE REMAINING ENGINES.");
        GuidanceLog.Info(vehicle, $"Landing group '{activation.Group.Name}' lost supply at {now:F3} s, replanning with {DescribeSelection(_s.EngineActivation)}.");
        return true;
    }

    private static void LogLandingEngineState(Vehicle vehicle, double now, string phase)
    {
        if (_s.EngineActivation is not { Ready: true } activation)
            return;

        var active = new List<string>();
        var supplied = new List<ulong>();
        var firing = new List<ulong>();
        ReadOnlySpan<RocketState> states = RocketStates.Of(vehicle.Parts);
        foreach (EngineController controller in vehicle.Parts.Modules.Get<EngineController>())
        {
            if (controller.IsActive)
                active.Add($"{controller.Parent.InstanceId}/{controller.TemplateId}");
            foreach (Rocket engine in controller.Rockets)
            {
                if ((uint)engine.StatesIdx >= (uint)states.Length)
                    continue;
                ref readonly RocketState state = ref states[engine.StatesIdx];
                if (state.IsPropellantAvailable)
                    supplied.Add(engine.InstanceId);
                if (state.Throttle > 0)
                    firing.Add(engine.InstanceId);
            }
        }
        (double thrust, double flow) = LandingEngineTelemetry.MeasuredMainEngineOutput(vehicle);
        GuidanceLog.Info(vehicle,
            $"Landing group '{activation.Group.Name}' {phase} at {now:F3} s. Plan: {DescribeSelection(activation)}. Active: {string.Join(", ", active)}. Supplied rockets: {string.Join(", ", supplied)}. Firing rockets: {string.Join(", ", firing)}. Main thrust {thrust:F0} N, flow {flow:F2} kg/s, propellant {vehicle.PropellantMass:F1} kg.");
    }

    private static void ResetLandingEngineSelection()
    {
        _s.EngineActivation = null;
        _s.LandingEngineCommitPending = null;
        _s.LandingGroupWaiting = false;
        _s.LandingGroupRefusal = "";
        _s.LandingGroupReplanPending = false;
    }
}

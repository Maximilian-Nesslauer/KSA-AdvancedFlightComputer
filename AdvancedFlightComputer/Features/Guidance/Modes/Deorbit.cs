#nullable disable

using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using AdvancedFlightComputer.Features.Guidance.Upfg;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

public static partial class GuidanceWindow
{
    // A source change or another burn must settle this long before the state is captured, and a captured request is compared with the flown state at the shorter interval.
    private const long DeorbitSettleMs = 500;
    private const long DeorbitRevalidateMs = 250;
    private const double MinimumVerticalGateM = 100;

    private static double EffectiveGLimit => _s.GLimitEnabled ? _s.GLimitG : 0;

    private static double LandingBrakeGateAltitude => BrakeGateAltitude(_s.VehicleHeightM);

    private static double BrakeGateAltitude(double vehicleHeightM) => Math.Max(_s.AimAltKm * 1000,
        _s.UseSixDofLanding ? 0 : _s.LandingVerticalGateM + vehicleHeightM);

    private static DeorbitSettings CurrentDeorbitSettings() => CurrentDeorbitSettings(_s.VehicleHeightM);

    private static DeorbitSettings CurrentDeorbitSettings(double vehicleHeightM) => new(_s.SiteLatDeg, _s.SiteLonDeg,
        _s.BrakingAltitudeKm * 1000, BrakeGateAltitude(vehicleHeightM), _s.GateUprangeKm * 1000,
        _s.DescentRate, _s.DownrangeFactor, _s.DeorbitArrivalAngleEnabled ? _s.DeorbitArrivalDescentDeg : null);

    // MEASURE COM ABOVE LEGS writes the height again twice a second while propellant moves the centre of mass (RefreshVehicleHeight), so a measured height is compared at its value from the capture of the request, and only a typed height can make the request old.
    private static DeorbitSettings ComparedDeorbitSettings() =>
        CurrentDeorbitSettings(_s.VehicleHeightAuto ? _s.DeorbitRequestHeightM : _s.VehicleHeightM);

    private static bool DeorbitTargetLocked => DeorbitLocks(_s);

    private static bool DeorbitLocks(VehicleAutopilotState s) => (s.DeorbitPlan != null || s.DirectBrakingPlan != null)
        && (IsStockDeorbitPhase(s.LandingPhase) || s.LandingPhase is LandingPhase.TransferPlanning
            or LandingPhase.TransferCoast or LandingPhase.Prep or LandingPhase.Burn);

    private static bool DeorbitRequestCurrent(DeorbitRequest request, IParentBody parent) =>
        request != null && ReferenceEquals(parent, request.Parent)
        && request.Settings == ComparedDeorbitSettings() && request.GLimit == EffectiveGLimit;

    /// <summary>The stop status of a stock deorbit that is no longer current, naming what differs from its request. It only reads the request and the current values.</summary>
    private static string DeorbitChangeText(DeorbitPlan plan, DeorbitRequest request, IParentBody parent)
    {
        if (plan == null || request == null)
            return "DEORBIT STOPPED: PLAN LOST.";
        if (!ReferenceEquals(parent, request.Parent))
            return "DEORBIT STOPPED: ORBITING ANOTHER BODY.";
        if (!_s.Engage || !_s.AutoStage)
        {
            string label = !_s.Engage ? Ui.Logic.GuidanceSettingSpecs.Engage.Label : Ui.Logic.GuidanceSettingSpecs.AutoStage.Label;
            return $"DEORBIT STOPPED: {label} OFF.";
        }
        DeorbitSettings was = request.Settings;
        DeorbitSettings now = ComparedDeorbitSettings();
        var changed = new List<string>();
        if (was.Latitude != now.Latitude || was.Longitude != now.Longitude)
            changed.Add("LANDING SITE");
        if (was.BrakingAltitude != now.BrakingAltitude)
            changed.Add(Ui.Logic.GuidanceSettingSpecs.BrakingAltitudeKm.Label);
        // The gate altitude comes from AIM ALTITUDE, or with G-FOLD from VERTICAL APPROACH plus COM ABOVE LEGS (LandingBrakeGateAltitude), so all of them are named with the two values.
        if (was.GateAltitude != now.GateAltitude)
            changed.Add(FormattableString.Invariant($"BRAKING GATE ALTITUDE ({was.GateAltitude:F3} m, NOW {now.GateAltitude:F3} m; FROM SOLVER, AIM ALTITUDE, VERTICAL APPROACH OR COM ABOVE LEGS)"));
        if (was.GateUprange != now.GateUprange)
            changed.Add(Ui.Logic.GuidanceSettingSpecs.GateUprangeKm.Label);
        if (was.SinkRate != now.SinkRate)
            changed.Add(Ui.Logic.GuidanceSettingSpecs.DescentRate.Label);
        if (was.DownrangeFactor != now.DownrangeFactor)
            changed.Add(Ui.Logic.GuidanceSettingSpecs.DownrangeFactor.Label);
        if (was.ArrivalDescentDeg != now.ArrivalDescentDeg)
            changed.Add(Ui.Logic.GuidanceSettingSpecs.DeorbitArrivalDescentDeg.Label);
        if (request.GLimit != EffectiveGLimit)
            changed.Add("G-LIMIT or MAX ACCELERATION");
        return changed.Count == 0
            ? "DEORBIT SETTINGS OR CONTROL SWITCHES CHANGED."
            : "DEORBIT STOPPED: " + string.Join(", ", changed) + " CHANGED.";
    }

    private static string DeorbitSettingsRefusal()
    {
        DeorbitSettings settings = CurrentDeorbitSettings();
        if (_s.DeorbitArrivalAngleEnabled && (!double.IsFinite(_s.DeorbitArrivalDescentDeg)
            || _s.DeorbitArrivalDescentDeg < 0 || _s.DeorbitArrivalDescentDeg > 30))
            return "SET ARRIVAL ANGLE DOWN TO 0-30 deg.";
        if (!settings.IsValid)
            return "CHECK LANDING APPROACH SETTINGS.";
        if (_s.GLimitEnabled && (!double.IsFinite(_s.GLimitG) || _s.GLimitG <= 0.1))
            return "SET MAX ACCELERATION ON LAUNCH ABOVE 0.1 g OR G-LIMIT OFF.";
        if (!_s.UseSixDofLanding && !(_s.LandingVerticalGateM >= MinimumVerticalGateM))
            return $"VERTICAL APPROACH MIN {MinimumVerticalGateM:F0} m.";
        return "";
    }

    private static void ClearDeorbitPlanState()
    {
        _s.DeorbitPlanner?.Dispose();
        _s.DeorbitPlanner = null;
        _s.DeorbitRequest = null;
        _s.DeorbitPlan = null;
        _s.DirectBrakingPlan = null;
        _s.DeorbitLastResidual = double.PositiveInfinity;
        _s.DeorbitNextValidationTick = 0;
        _s.DeorbitEngineSignature = 0;
        _s.BurnStartTime = 0;
        _s.BurnDownrangeKm = 0;
        _s.DirectBrakingRefusal = "";
    }

    // DEORBIT + LAND claims the landing and starts planning. The search itself runs in slices from StepDeorbitPlanning.
    private static void ExecuteLanding(Vehicle vehicle)
    {
        // Touchdown guidance is the player's only descent solver. G-FOLD flies a craft that touchdown guidance cannot plan for, and that is decided again for each landing, ahead of the settings check, which reads the solver.
        string refusal = LandingSwitchesOff
            ? "LANDING NEEDS AFC MAY STEER + AFC MAY USE ENGINES. SEE GNC SWITCHES (MORE)." : ChooseLandingSolver(vehicle);
        if (refusal.Length == 0)
            refusal = DeorbitSettingsRefusal();
        if (refusal.Length > 0)
        {
            _s.WarnLanding(refusal);
            return;
        }
        ClaimVehicle(GuidanceMode.Landing, vehicle);
        _s.AutoLaunch = false;
        _s.CutoffDone = false;
        _s.StagingActive = false;
        _s.HasCommand = false;
        _s.LandingCutPending = false;
        _s.LandingPhase = LandingPhase.DeorbitPlanning;
        _s.LandingStatus = "PLANNING LANDING APPROACH";
        _warpDeclinedLabel = "";
    }

    private static DeorbitRequest NewDeorbitRequest(Vehicle vehicle, Orbit orbit, UpfgVehicle model,
        DeorbitEngine[] engines, double minimumPulse) =>
        new(orbit, SimNow(), vehicle.TotalMass, vehicle.BoundingSphereRadiusBody, CurrentDeorbitSettings(),
            model, engines, minimumPulse, DeorbitControlStep(vehicle), EffectiveGLimit,
            StockPreparationTime(vehicle.FlightComputer), KsaEnginePerf.StructuralAccelerationLimit(vehicle) / StandardGravity);

    // Stores the request with the engine signature it was captured under, so a later staging or sequence edit invalidates it.
    private static bool TryCaptureDeorbitRequest(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        UpfgVehicle model = BuildUpfgVehicle(vehicle);
        if (model == null || model.Stages.Count == 0) return false;
        var engines = new List<DeorbitEngine>();
        double minimumPulse = 0;
        if (KsaEnginePerf.GetThrottleControlStatus(vehicle) == KsaEnginePerf.ThrustStatus.Available)
        {
            minimumPulse = KsaEnginePerf.MinimumPulse(vehicle);
            double pressure = KsaEnginePerf.AmbientPressureAt(parent, orbit.StateVectors.PositionCci.Length() - parent.MeanRadius);
            foreach (double throttle in new[] { Math.Max(vehicle.GetMinThrottle(), 0.001), 1.0 })
            {
                (double thrust, double flow) = KsaEnginePerf.AtThrottle(vehicle, throttle, pressure);
                if (thrust > 0 && flow > 0 && double.IsFinite(thrust + flow))
                    engines.Add(new(throttle, thrust, flow));
            }
        }
        _s.DeorbitRequestHeightM = _s.VehicleHeightM;
        _s.DeorbitRequest = NewDeorbitRequest(vehicle, orbit, model, engines.ToArray(), minimumPulse);
        _s.DeorbitEngineSignature = DeorbitEngineSignature(vehicle);
        return true;
    }

    private static double DeorbitControlStep(Vehicle vehicle) =>
        Math.Max(1.0 / 60, double.IsFinite(vehicle.KinematicMeasurements.DeltaTime) ? vehicle.KinematicMeasurements.DeltaTime : 1.0 / 60);

    private static int DeorbitEngineSignature(Vehicle vehicle)
    {
        var signature = new HashCode();
        signature.Add(RuntimeHelpers.GetHashCode(vehicle.Parts));
        if (vehicle.Parts.SequenceList is SequenceList list)
        {
            signature.Add(RuntimeHelpers.GetHashCode(list));
            foreach (Sequence sequence in list.Sequences)
            {
                signature.Add(RuntimeHelpers.GetHashCode(sequence));
                signature.Add(sequence.Number);
                signature.Add(sequence.Activated);
                signature.Add(sequence.Environment);
            }
        }
        foreach (Part part in vehicle.Parts.Parts)
        {
            signature.Add(RuntimeHelpers.GetHashCode(part));
            signature.Add(part.SequenceOrder);
        }
        foreach (ISequenced module in vehicle.Parts.Modules.GetUsing<ISequenced>())
        {
            signature.Add(RuntimeHelpers.GetHashCode(module));
            signature.Add(module.Sequence);
        }
        bool hasCores = ModuleStateful<RocketCore, RocketCoreState, RocketCoreGlobalState, EmptyStruct>
            .TryGetFrom(vehicle.Parts.States, out var cores);
        bool hasControllers = ModuleStateful<EngineController, EngineControllerState, EngineControllerGlobalState, EmptyStruct>
            .TryGetFrom(vehicle.Parts.States, out var controllers);
        foreach (EngineController engine in vehicle.Parts.Modules.Get<EngineController>())
        {
            signature.Add(RuntimeHelpers.GetHashCode(engine));
            signature.Add(engine.IsActive);
            signature.Add(hasControllers && controllers.States[engine.StatesIdx].IsPropellantAvailable);
            foreach (RocketCore core in engine.Cores)
            {
                signature.Add(RuntimeHelpers.GetHashCode(core));
                signature.Add(hasCores && cores.States[core.StatesIdx].IsPropellantAvailable);
                signature.Add(core.MinimumPulseTime);
                signature.Add(core.MinimumThrottle);
            }
        }
        return signature.ToHashCode();
    }

    // Planning and the first coast run before acquisition and cannot write an engine or attitude.
    private static void StepDeorbitPlanning(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        if (!LandingWaits(_s.LandingPhase)) return;
        if (_s.ControlAcquired && _s.LandingPhase == LandingPhase.DeorbitPlanning)
        {
            // Release the previous mode before waiting for the engine command that it left behind.
            ReleaseVehicle(vehicle, keepWaitingCoast: true);
            return;
        }
        if (LandingSwitchesOff) { RefuseDeorbit(vehicle, LandingSwitchesChangedStatus); return; }
        string engineRefusal = BrakingEngineRefusal(vehicle);
        if (engineRefusal.Length > 0) { RefuseDeorbit(vehicle, engineRefusal); return; }
        // Only the capture waits for idle engines and 1x warp. StepRealSpeedHold sets 1x before this step, so the warp wait catches only a speed raised between the two. A direct coast checks its braking model again at ignition.
        if (_s.LandingPhase == LandingPhase.DeorbitPlanning)
        {
            if (vehicle.FlightComputer.BurnMode == FlightComputerBurnMode.Auto
                || ManualInputs(vehicle).EngineOn || KsaEnginePerf.RemainingMinimumPulse(vehicle) > 0)
            {
                RestartDeorbitPlanning("PLANNING WAITS: BURN OR ENGINE PULSE ACTIVE.");
                return;
            }
            if (Universe.SimulationSpeed > 1)
            {
                ClearDeorbitPlanState();
                _s.WarnLanding("PLANNING WAITS: WARP ABOVE 1x.");
                return;
            }
        }
        if (_s.DeorbitRequest != null && DeorbitSourceChanged(vehicle, orbit, parent))
        {
            RestartDeorbitPlanning("SOURCE STATE OR SETTINGS CHANGED. PLANNING AGAIN.");
            return;
        }
        if (_s.LandingPhase != LandingPhase.DeorbitPlanning) return;
        if (_s.DeorbitRequest == null && !StartDeorbitSearch(vehicle, orbit, parent)) return;
        DeorbitPlanner planner = _s.DeorbitPlanner;
        planner.Step();
        _s.LandingStatus = planner.Status;
        _s.DirectBrakingRefusal = planner.DirectRefusal;
        if (planner.Complete) FinishDeorbitSearch(vehicle, parent, planner);
    }

    private static void RestartDeorbitPlanning(string status)
    {
        ClearDeorbitPlanState();
        _s.DeorbitNextValidationTick = GuidanceClock.NowMs + DeorbitSettleMs;
        _s.LandingPhase = LandingPhase.DeorbitPlanning;
        _s.LandingStatus = status;
    }

    private static bool DeorbitSourceChanged(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        DeorbitRequest request = _s.DeorbitRequest;
        if (!DeorbitRequestCurrent(request, parent)) return true;
        // Direct braking rebuilds its model at ignition, so RCS use during its coast does not start a transfer search.
        if (_s.LandingPhase == LandingPhase.Coast
            || (_s.LandingPhase != LandingPhase.DeorbitCoast && GuidanceClock.NowMs < _s.DeorbitNextValidationTick)) return false;
        _s.DeorbitNextValidationTick = GuidanceClock.NowMs + DeorbitRevalidateMs;
        StateVectors expected = request.Source.GetStateVectorsAt(new UniverseTime(SimNow()));
        return Math.Abs(vehicle.TotalMass - request.Mass) > Math.Max(1, request.Mass * 0.001)
            || !DeorbitPlanner.SameState(expected, orbit.StateVectors.PositionCci, orbit.StateVectors.VelocityCci)
            || _s.DeorbitEngineSignature != DeorbitEngineSignature(vehicle);
    }

    private static bool StartDeorbitSearch(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        if (GuidanceClock.NowMs < _s.DeorbitNextValidationTick) return false;
        string refusal = DeorbitSettingsRefusal();
        if (refusal.Length > 0) { RefuseDeorbit(vehicle, refusal); return false; }
        if (!TryCaptureDeorbitRequest(vehicle, orbit, parent)) { RefuseDeorbit(vehicle, "NO USABLE ENGINE MODEL."); return false; }
        DeorbitRequest request = _s.DeorbitRequest;
        _s.DeorbitPlanner = new DeorbitPlanner(request);
        _s.DeorbitNextValidationTick = GuidanceClock.NowMs + DeorbitRevalidateMs;
        StateVectors captured = request.Source.StateVectors;
        string thrust = request.Engines.Length == 0 ? "NO USABLE ENGINE"
            : $"MIN THRUST {request.Engines[0].Thrust:F1} N, FULL THRUST {request.Engines[^1].Thrust:F1} N";
        GuidanceLog.Info(vehicle, $"deorbit search: epoch {request.Epoch:F3} s, site {request.Settings.Latitude:F6} / {request.Settings.Longitude:F6} deg"
            + $", requested arrival {(request.Settings.ArrivalDescentDeg is double angle ? $"{angle:F2} deg down" : "automatic")}"
            + $", braking altitude {request.Settings.BrakingAltitude:F1} m, gate altitude {request.Settings.GateAltitude:F1} m, mass {request.Mass:F1} kg"
            + $", step {request.ControlStep:F6} s, minimum pulse {request.MinimumPulse:F6} s, {thrust}, G limit {request.GLimit:F3}, structural limit {request.StructuralGLimit:F3} g"
            + $", position CCI ({captured.PositionCci.X:F4}, {captured.PositionCci.Y:F4}, {captured.PositionCci.Z:F4}) m"
            + $", velocity CCI ({captured.VelocityCci.X:F4}, {captured.VelocityCci.Y:F4}, {captured.VelocityCci.Z:F4}) m/s.");
        return true;
    }

    private static void FinishDeorbitSearch(Vehicle vehicle, IParentBody parent, DeorbitPlanner planner)
    {
        GuidanceLog.Info(vehicle, $"deorbit search complete: {planner.Evaluations} Lambert evaluations, {planner.CandidatesChecked} candidate checks, {planner.RefinementSamples} refinement samples, {planner.WorkMilliseconds:F1} ms of planner work. {planner.Status}");
        _s.DirectBrakingPlan = planner.Direct;
        _s.DeorbitPlan = planner.Plan;
        planner.Dispose();
        _s.DeorbitPlanner = null;
        double now = SimNow();
        if (planner.Direct is { Refusal.Length: 0 } direct)
        {
            if (direct.IgnitionTime < now + PrepLeadTime)
            { RefuseDeorbit(vehicle, "BRAKING WINDOW EXPIRED IN PLANNING. REPEAT DEORBIT + LAND AT 1x WARP."); return; }
            CommitBrakingBurn(vehicle, parent, direct, now, LandingPhase.Coast, "BRAKING BURN PLANNED");
        }
        else if (planner.Plan is DeorbitPlan plan)
        {
            if (plan.DepartureTime < now + PrepLeadTime)
            { RefuseDeorbit(vehicle, "DEORBIT WINDOW EXPIRED IN PLANNING. REPEAT DEORBIT + LAND AT 1x WARP."); return; }
            _s.BurnStartTime = plan.ArrivalTime;
            _s.BurnDownrangeKm = plan.BrakingDistance / 1000;
            _s.LandingPhase = LandingPhase.DeorbitCoast;
            DeorbitSettings settings = _s.DeorbitRequest.Settings;
            GuidanceLog.Info(vehicle, $"deorbit planned: node in {plan.DepartureTime - now:F1} s, delta-v {plan.DeltaV.Length():F2} m/s"
                + $", braking in {plan.ArrivalTime - now:F1} s"
                + $" at {settings.BrakingAltitude / 1000:F2} km above the site and {plan.BrakingDistance * settings.DownrangeFactor / 1000:F2} km uprange"
                + $", arrival angle {DeorbitPlanner.ArrivalAngleDegrees(plan):F2} deg.");
            _s.LastGuidanceLogTime = double.NegativeInfinity;
        }
        else RefuseDeorbit(vehicle, planner.Status);
    }

    // The first plan and the refresh after the stock node commit a direct braking burn the same way.
    private static void CommitBrakingBurn(Vehicle vehicle, IParentBody parent, DirectBrakingPlan direct, double now,
        LandingPhase coastPhase, string label)
    {
        _s.DirectBrakingPlan = direct;
        _s.BurnStartTime = direct.IgnitionTime;
        _s.BurnDownrangeKm = direct.Downrange / 1000;
        _s.LandingPhase = coastPhase;
        _s.LastGuidanceLogTime = double.NegativeInfinity;
        GuidanceLog.Info(vehicle, $"{label}: ignition in {direct.IgnitionTime - now:F1} s"
            + $" at altitude {(direct.Position.Length() - parent.MeanRadius) / 1000:F3} km and {direct.Velocity.Length():F0} m/s"
            + $", {direct.Downrange * _s.DeorbitRequest.Settings.DownrangeFactor / 1000:F3} km uprange.");
        if (direct.IgnitionTime - now > PrepLeadTime + WarpLeadTime)
            RequestWarp(direct.IgnitionTime - PrepLeadTime - WarpLeadTime, "LANDING BURN PREPARATION");
    }

    private static void RefuseDeorbit(Vehicle vehicle, string reason, bool failure = false)
    {
        // Ending a landing between the coast and the braking burn is a failure, except from a closed orbit that clears the surface.
        failure |= reason != LandingSwitchesChangedStatus
            && _s.LandingPhase is LandingPhase.TransferPlanning or LandingPhase.TransferCoast or LandingPhase.Coast or LandingPhase.Prep or LandingPhase.Burn
            && CraftOnWayDown(vehicle);
        ClearDeorbitPlanState();
        _s.HasCommand = false;
        _s.LandingPhase = LandingPhase.Done;
        if (failure)
            _s.FailLanding("LANDING STOPPED", reason);
        else
            _s.WarnLanding(reason);
        if (_s.ControlAcquired) _s.LandingCutPending = true;
    }

    private static bool CheckBrakingIgnition(Vehicle vehicle, Orbit orbit, double now)
    {
        if (_s.DeorbitRequest == null) return true;
        UpfgVehicle model = BuildUpfgVehicle(vehicle);
        if (model == null || model.Stages.Count == 0)
        { RefuseDeorbit(vehicle, "BRAKING IGNITION: NO USABLE ENGINE MODEL."); return false; }
        DeorbitRequest current = NewDeorbitRequest(vehicle, orbit, model, Array.Empty<DeorbitEngine>(), 0);
        StateVectors state = orbit.StateVectors;
        if (!DeorbitPlanner.CanBrake(current, state.PositionCci, state.VelocityCci, now, vehicle.TotalMass, out string refusal))
        { RefuseDeorbit(vehicle, refusal); return false; }
        GuidanceLog.Info(vehicle, $"braking ignition: alt {(state.PositionCci.Length() - current.Parent.MeanRadius) / 1000:F2} km"
            + $", speed {state.VelocityCci.Length():F1} m/s.");
        return true;
    }

    private static string BrakingEngineRefusal(Vehicle vehicle)
    {
        KsaEnginePerf.ThrustStatus status = KsaEnginePerf.GetThrottleControlStatus(vehicle);
        if (status == KsaEnginePerf.ThrustStatus.UnsupportedEngine)
            return "Braking requires liquid engines with throttle and shutdown control.";
        // Check a cold ignition before staging can light a motor that cannot shut down.
        if (status == KsaEnginePerf.ThrustStatus.NoAuthority
            && vehicle.Parts.SequenceList != null)
        {
            foreach (Sequence sequence in vehicle.Parts.SequenceList.Sequences)
            {
                if (sequence.Activated) continue;
                bool hasEngine = false;
                foreach (Part part in sequence.Parts)
                    foreach (ISequenced module in part.GetSubtreeSequencedModules())
                    {
                        if (module.Sequence != sequence.Number || module is not EngineController engine) continue;
                        hasEngine = true;
                        foreach (RocketCore core in engine.Cores)
                            if (!KsaEnginePerf.FollowsThrottle(core))
                                return "The next landing engine cannot obey throttle and shutdown commands.";
                    }
                if (hasEngine) break;
            }
        }
        return "";
    }

    private static bool StepDeorbit(Vehicle vehicle, Orbit orbit, IParentBody parent, double now)
    {
        if (_s.LandingPhase == LandingPhase.DeorbitPlanning || IsStockDeorbitPhase(_s.LandingPhase)) return true;
        if (_s.LandingPhase is not (LandingPhase.TransferPlanning or LandingPhase.TransferCoast)) return false;
        _s.HasCommand = false;
        if (!DeorbitRequestCurrent(_s.DeorbitRequest, parent))
        { RefuseDeorbit(vehicle, "BRAKING SETTINGS CHANGED. STOP, THEN PLAN AGAIN."); return true; }
        if (LandingSwitchesOff) { RefuseDeorbit(vehicle, LandingSwitchesChangedStatus); return true; }
        if (_s.LandingPhase == LandingPhase.TransferPlanning)
            StepTransferPlanning(vehicle, orbit, parent, now);
        if (_s.LandingPhase == LandingPhase.TransferCoast && now >= _s.BurnStartTime - PrepLeadTime)
            EnterBrakingPrep();
        return true;
    }

    // After the stock burn, the braking point is planned again from the measured orbit and mass.
    private static void StepTransferPlanning(Vehicle vehicle, Orbit orbit, IParentBody parent, double now)
    {
        if (_s.DeorbitPlan == null || now >= _s.DeorbitPlan.ArrivalTime - PrepLeadTime)
        { RefuseDeorbit(vehicle, "BRAKING WINDOW PASSED BEFORE PLAN READY."); return; }
        DeorbitPlanner planner = _s.DeorbitPlanner;
        if (planner == null)
        {
            // RocketCore.UpdateState can retain an unexpired minimum pulse after the engine-off command.
            if (KsaEnginePerf.RemainingMinimumPulse(vehicle) > 0) return;
            if (!TryCaptureDeorbitRequest(vehicle, orbit, parent))
            { RefuseDeorbit(vehicle, "DEORBIT DONE: NO USABLE BRAKING MODEL."); return; }
            planner = new DeorbitPlanner(_s.DeorbitRequest, brakingOnly: true, arrivalHint: _s.DeorbitPlan);
            _s.DeorbitPlanner = planner;
        }
        planner.Step();
        _s.LandingStatus = planner.Status;
        if (!planner.Complete) return;
        _s.DeorbitPlanner = null;
        planner.Dispose();
        if (planner.Direct is not { Refusal.Length: 0 } direct)
        { RefuseDeorbit(vehicle, planner.Status); return; }
        if (direct.IgnitionTime < now + PrepLeadTime)
        { RefuseDeorbit(vehicle, "BRAKING WINDOW PASSED DURING SOLVE."); return; }
        CommitBrakingBurn(vehicle, parent, direct, now, LandingPhase.TransferCoast, "ACTUAL-ORBIT BRAKING PLAN");
        _s.LandingStatus = "STOCK DEORBIT COMPLETE. COAST TO AFC BRAKING BURN.";
    }

    private static IEnumerable<(string label, string value)> DeorbitReadout()
    {
        DeorbitRequest request = _s.DeorbitRequest;
        if (request == null) yield break;
        yield return ("ARRIVAL TARGET", request.Settings.ArrivalDescentDeg is double angle
            ? FormattableString.Invariant($"{angle:F1} deg DOWN, +/-{DeorbitPlanner.ArrivalAngleToleranceDeg:F1} deg") : "AUTOMATIC");
        if (_s.DeorbitPlan is DeorbitPlan plan)
        {
            yield return ("DEORBIT BURN", "T-" + Ui.Logic.Fmt.Time(Math.Max(0, plan.DepartureTime - SimNow())));
            yield return ("DEORBIT DV", Ui.Logic.Fmt.Dv(plan.DeltaV.Length()));
            yield return ("DEORBIT EXECUTION", "STOCK AUTO");
            yield return ("DEORBIT THROTTLE", "STOCK ENGINE LIMITS APPLY. AFC G-LIMIT APPLIES TO BRAKING.");
            if (_s.DeorbitTarget != null)
            {
                yield return ("STOCK IGNITION", "T-" + Ui.Logic.Fmt.Time(Math.Max(0, _s.DeorbitTarget.IgnitionTime.Seconds() - SimNow())));
                yield return ("STOCK BURN DURATION", Ui.Logic.Fmt.TimeFine(_s.DeorbitTarget.BurnDuration));
            }
            yield return ("BRAKING IGNITION", "T-" + Ui.Logic.Fmt.Time(Math.Max(0, _s.BurnStartTime - SimNow())));
            yield return ("BRAKING ALTITUDE", Ui.Logic.Fmt.KmFine(request.Settings.BrakingAltitude / 1000) + " ABOVE SITE");
            yield return ("ARRIVAL ANGLE", Ui.Logic.Fmt.Deg(DeorbitPlanner.ArrivalAngleDegrees(plan)) + " FROM HORIZON (NEGATIVE = DOWN)");
            yield return ("LOCAL CLEARANCE", $"{plan.Clearance:F0} m");
            yield return ("BRAKING UPRANGE", Ui.Logic.Fmt.KmFine(plan.BrakingDistance * request.Settings.DownrangeFactor / 1000));
        }
        if (_s.DirectBrakingPlan is DirectBrakingPlan direct)
            yield return ("DIRECT IGNITION ALTITUDE", Ui.Logic.Fmt.KmFine((direct.Position.Length() - request.Parent.MeanRadius) / 1000));
        if (_s.DirectBrakingRefusal.Length > 0)
            yield return ("DIRECT APPROACH", _s.DirectBrakingRefusal);
    }
}

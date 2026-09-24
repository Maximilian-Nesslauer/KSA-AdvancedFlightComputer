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

    private static double LandingBrakeGateAltitude => Math.Max(_s.AimAltKm * 1000,
        _s.UseSixDofLanding ? 0 : _s.LandingVerticalGateM + _s.VehicleHeightM);

    private static DeorbitSettings CurrentDeorbitSettings() => new(_s.SiteLatDeg, _s.SiteLonDeg,
        _s.BrakingAltitudeKm * 1000, LandingBrakeGateAltitude, _s.GateUprangeKm * 1000,
        _s.DescentRate, _s.DownrangeFactor, _s.DeorbitArrivalAngleEnabled ? _s.DeorbitArrivalDescentDeg : null);

    private static bool DeorbitTargetLocked => (_s.DeorbitPlan != null || _s.DirectBrakingPlan != null)
        && (IsStockDeorbitPhase(_s.LandingPhase) || _s.LandingPhase is LandingPhase.TransferPlanning
            or LandingPhase.TransferCoast or LandingPhase.Prep or LandingPhase.Burn);

    private static bool DeorbitRequestCurrent(DeorbitRequest request, IParentBody parent) =>
        request != null && ReferenceEquals(parent, request.Parent)
        && request.Settings == CurrentDeorbitSettings() && request.GLimit == EffectiveGLimit;

    private static string DeorbitSettingsRefusal()
    {
        DeorbitSettings settings = CurrentDeorbitSettings();
        if (_s.DeorbitArrivalAngleEnabled && (!double.IsFinite(_s.DeorbitArrivalDescentDeg)
            || _s.DeorbitArrivalDescentDeg < 0 || _s.DeorbitArrivalDescentDeg > 30))
            return "Set the deorbit arrival angle between 0 and 30 degrees downward.";
        if (!settings.IsValid || (_s.GLimitEnabled && (!double.IsFinite(_s.GLimitG) || _s.GLimitG <= 0.1)))
            return "Check the landing approach settings.";
        if (!_s.UseSixDofLanding && !(_s.LandingVerticalGateM >= MinimumVerticalGateM))
            return $"The vertical approach height must be at least {MinimumVerticalGateM:F0} m.";
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

    // EXECUTE claims the landing and starts planning. The search itself runs in slices from StepDeorbitPlanning.
    private static void ExecuteLanding(Vehicle vehicle)
    {
        string refusal = LandingSwitchesOff
            ? "Automatic landing needs Engage autopilot and Auto engines/staging." : DeorbitSettingsRefusal();
        if (refusal.Length > 0)
        {
            _s.LandingStatus = refusal;
            return;
        }
        ClaimVehicle(GuidanceMode.Landing, vehicle);
        _s.AutoLaunch = false;
        _s.CutoffDone = false;
        _s.StagingActive = false;
        _s.HasCommand = false;
        _s.LandingCutPending = false;
        _s.LandingPhase = LandingPhase.DeorbitPlanning;
        _s.LandingStatus = "Planning the landing approach.";
        _warpDeclinedLabel = "";
    }

    private static DeorbitRequest NewDeorbitRequest(Vehicle vehicle, Orbit orbit, UpfgVehicle model,
        DeorbitEngine[] engines, double minimumPulse) =>
        new(orbit, SimNow(), vehicle.TotalMass, vehicle.BoundingSphereRadiusBody, CurrentDeorbitSettings(),
            model, engines, minimumPulse, DeorbitControlStep(vehicle), EffectiveGLimit,
            StockPreparationTime(vehicle.FlightComputer));

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
        if (LandingSwitchesOff) { RefuseDeorbit(LandingSwitchesChangedStatus); return; }
        string engineRefusal = BrakingEngineRefusal(vehicle);
        if (engineRefusal.Length > 0) { RefuseDeorbit(engineRefusal); return; }
        // Only the capture waits for idle engines and 1x warp. A direct coast checks its braking model again at ignition.
        if (_s.LandingPhase == LandingPhase.DeorbitPlanning)
        {
            if (vehicle.FlightComputer.BurnMode == FlightComputerBurnMode.Auto
                || ManualInputs(vehicle).EngineOn || KsaEnginePerf.RemainingMinimumPulse(vehicle) > 0)
            {
                RestartDeorbitPlanning("Planning waits for the active burn and engine pulse to finish.");
                return;
            }
            if (Universe.SimulationSpeed > 1)
            {
                ClearDeorbitPlanState();
                _s.LandingStatus = "Planning waits for time warp at 1x or less.";
                return;
            }
        }
        if (_s.DeorbitRequest != null && DeorbitSourceChanged(vehicle, orbit, parent))
        {
            RestartDeorbitPlanning("The source state or settings changed. Planning again.");
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
        _s.DeorbitNextValidationTick = Environment.TickCount64 + DeorbitSettleMs;
        _s.LandingPhase = LandingPhase.DeorbitPlanning;
        _s.LandingStatus = status;
    }

    private static bool DeorbitSourceChanged(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        DeorbitRequest request = _s.DeorbitRequest;
        if (!DeorbitRequestCurrent(request, parent)) return true;
        // Direct braking rebuilds its model at ignition, so RCS use during its coast does not start a transfer search.
        if (_s.LandingPhase == LandingPhase.Coast
            || (_s.LandingPhase != LandingPhase.DeorbitCoast && Environment.TickCount64 < _s.DeorbitNextValidationTick)) return false;
        _s.DeorbitNextValidationTick = Environment.TickCount64 + DeorbitRevalidateMs;
        StateVectors expected = request.Source.GetStateVectorsAt(new UniverseTime(SimNow()));
        return Math.Abs(vehicle.TotalMass - request.Mass) > Math.Max(1, request.Mass * 0.001)
            || !DeorbitPlanner.SameState(expected, orbit.StateVectors.PositionCci, orbit.StateVectors.VelocityCci)
            || _s.DeorbitEngineSignature != DeorbitEngineSignature(vehicle);
    }

    private static bool StartDeorbitSearch(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        if (Environment.TickCount64 < _s.DeorbitNextValidationTick) return false;
        string refusal = DeorbitSettingsRefusal();
        if (refusal.Length > 0) { RefuseDeorbit(refusal); return false; }
        if (!TryCaptureDeorbitRequest(vehicle, orbit, parent)) { RefuseDeorbit("No usable engine model on the vehicle."); return false; }
        DeorbitRequest request = _s.DeorbitRequest;
        _s.DeorbitPlanner = new DeorbitPlanner(request);
        _s.DeorbitNextValidationTick = Environment.TickCount64 + DeorbitRevalidateMs;
        StateVectors captured = request.Source.StateVectors;
        string thrust = request.Engines.Length == 0 ? "no usable engine"
            : $"minimum thrust {request.Engines[0].Thrust:F1} N, full thrust {request.Engines[^1].Thrust:F1} N";
        GuidanceLog.Info(vehicle, $"deorbit search: epoch {request.Epoch:F3} s, site {request.Settings.Latitude:F6} / {request.Settings.Longitude:F6} deg"
            + $", requested arrival {(request.Settings.ArrivalDescentDeg is double angle ? $"{angle:F2} deg down" : "automatic")}"
            + $", braking altitude {request.Settings.BrakingAltitude:F1} m, gate altitude {request.Settings.GateAltitude:F1} m, mass {request.Mass:F1} kg"
            + $", step {request.ControlStep:F6} s, minimum pulse {request.MinimumPulse:F6} s, {thrust}, G limit {request.GLimit:F3}"
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
            { RefuseDeorbit("The braking window expired during planning. Execute again at 1x time warp."); return; }
            CommitBrakingBurn(vehicle, parent, direct, now, LandingPhase.Coast, "braking burn planned");
        }
        else if (planner.Plan is DeorbitPlan plan)
        {
            if (plan.DepartureTime < now + PrepLeadTime)
            { RefuseDeorbit("The deorbit window expired during planning. Execute again at 1x time warp."); return; }
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
        else RefuseDeorbit(planner.Status);
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
            RequestWarp(direct.IgnitionTime - PrepLeadTime - WarpLeadTime, "the landing burn preparation");
    }

    private static void RefuseDeorbit(string reason)
    {
        ClearDeorbitPlanState();
        _s.HasCommand = false;
        _s.LandingPhase = LandingPhase.Done;
        _s.LandingStatus = reason;
        if (_s.ControlAcquired) _s.LandingCutPending = true;
    }

    private static bool CheckBrakingIgnition(Vehicle vehicle, Orbit orbit, double now)
    {
        if (_s.DeorbitRequest == null) return true;
        UpfgVehicle model = BuildUpfgVehicle(vehicle);
        if (model == null || model.Stages.Count == 0)
        { RefuseDeorbit("The braking ignition has no usable engine model."); return false; }
        DeorbitRequest current = NewDeorbitRequest(vehicle, orbit, model, Array.Empty<DeorbitEngine>(), 0);
        StateVectors state = orbit.StateVectors;
        if (!DeorbitPlanner.CanBrake(current, state.PositionCci, state.VelocityCci, now, vehicle.TotalMass, out string refusal))
        { RefuseDeorbit(refusal); return false; }
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
                            if (core is not Combustor)
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
        { RefuseDeorbit("The committed braking settings changed. Abort and plan again."); return true; }
        if (LandingSwitchesOff) { RefuseDeorbit(LandingSwitchesChangedStatus); return true; }
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
        { RefuseDeorbit("The braking preparation window passed before the actual-orbit plan was ready."); return; }
        DeorbitPlanner planner = _s.DeorbitPlanner;
        if (planner == null)
        {
            // RocketCore.UpdateState can retain an unexpired minimum pulse after the engine-off command.
            if (KsaEnginePerf.RemainingMinimumPulse(vehicle) > 0) return;
            if (!TryCaptureDeorbitRequest(vehicle, orbit, parent))
            { RefuseDeorbit("The completed deorbit has no usable braking model."); return; }
            planner = new DeorbitPlanner(_s.DeorbitRequest, brakingOnly: true, arrivalHint: _s.DeorbitPlan);
            _s.DeorbitPlanner = planner;
        }
        planner.Step();
        _s.LandingStatus = planner.Status;
        if (!planner.Complete) return;
        _s.DeorbitPlanner = null;
        planner.Dispose();
        if (planner.Direct is not { Refusal.Length: 0 } direct)
        { RefuseDeorbit(planner.Status); return; }
        if (direct.IgnitionTime < now + PrepLeadTime)
        { RefuseDeorbit("The braking window passed while braking was planned from the actual orbit."); return; }
        CommitBrakingBurn(vehicle, parent, direct, now, LandingPhase.TransferCoast, "actual-orbit braking plan");
        _s.LandingStatus = "Stock deorbit complete. Coasting to the AFC braking burn.";
    }

    private static IEnumerable<(string label, string value)> DeorbitReadout()
    {
        DeorbitRequest request = _s.DeorbitRequest;
        if (request == null) yield break;
        yield return ("Arrival target", request.Settings.ArrivalDescentDeg is double angle
            ? $"{angle:F1} deg down, +/-{DeorbitPlanner.ArrivalAngleToleranceDeg:F1} deg" : "Automatic");
        if (_s.DeorbitPlan is DeorbitPlan plan)
        {
            yield return ("Deorbit node", $"T-{Math.Max(0, plan.DepartureTime - SimNow()):F1} s");
            yield return ("Deorbit delta-v", $"{plan.DeltaV.Length():F2} m/s");
            yield return ("Deorbit execution", "Stock Auto");
            yield return ("Deorbit throttle", "Stock engine limits apply. The AFC G limit applies to braking.");
            if (_s.DeorbitTarget != null)
            {
                yield return ("Stock ignition", $"T-{Math.Max(0, _s.DeorbitTarget.IgnitionTime.Seconds() - SimNow()):F1} s");
                yield return ("Stock burn duration", $"{_s.DeorbitTarget.BurnDuration:F2} s");
            }
            yield return ("Braking ignition", $"T-{Math.Max(0, _s.BurnStartTime - SimNow()):F1} s");
            yield return ("Braking altitude", $"{request.Settings.BrakingAltitude / 1000:F2} km above site");
            yield return ("Arrival angle", $"{DeorbitPlanner.ArrivalAngleDegrees(plan):F2} deg from the horizon (negative is down)");
            yield return ("Local clearance", $"{plan.Clearance:F0} m");
            yield return ("Braking uprange", $"{plan.BrakingDistance * request.Settings.DownrangeFactor / 1000:F2} km");
        }
        if (_s.DirectBrakingPlan is DirectBrakingPlan direct)
            yield return ("Direct ignition altitude", $"{(direct.Position.Length() - request.Parent.MeanRadius) / 1000:F2} km");
        if (_s.DirectBrakingRefusal.Length > 0)
            yield return ("Direct approach", _s.DirectBrakingRefusal);
    }
}

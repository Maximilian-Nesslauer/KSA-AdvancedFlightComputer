#nullable disable

using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Guidance;

public static partial class GuidanceWindow
{
    private static void DrawStockDeorbitActions()
    {
        if (_s.LandingPhase != LandingPhase.DeorbitBurn || !_s.DeorbitAutoArmed) return;
        ImGui.TextWrapped("Stock Auto is armed. AFC continues landing after the node completes.");
        if (ImGui.Button("Auto burn and warp")) _s.DeorbitWarpRequested = true;
        if (Universe.IsAutoWarpActive) ImGui.TextWrapped("Time warp is active.");
    }

    // Stock Auto needs this long before ignition to turn the craft onto the burn.
    // FlightComputer.UpdateRcsParams derives the flip time from RCS torque alone and leaves it infinite without RCS, so like stock only a finite value counts.
    private static double StockPreparationTime(FlightComputer fc) => float.IsFinite(fc.ConservativeFlipTime)
        ? Math.Max(PrepLeadTime, 2 * fc.ConservativeFlipTime) : PrepLeadTime;

    private static double StockDeorbitWarpTime(FlightComputer fc, BurnTarget target) =>
        target.IgnitionTime.Seconds() - StockPreparationTime(fc) - WarpLeadTime;

    private static void StepStockDeorbitWarp(Vehicle vehicle)
    {
        if (!_s.DeorbitWarpRequested) return;
        _s.DeorbitWarpRequested = false;
        double target = StockDeorbitWarpTime(vehicle.FlightComputer, _s.DeorbitTarget);
        if (!ReferenceEquals(vehicle, Program.ControlledVehicle) || !double.IsFinite(target)
            || target <= SimNow() + 1)
        {
            _s.LandingStatus = "Stock Auto is armed. Time warp needs this craft in focus and a future preparation window.";
            return;
        }
        // This runs with the vehicle workers joined, after the panel requests the action.
        Universe.AutoWarpTo(new UniverseTime(target));
        _s.DeorbitWarpTime = target;
        _s.LandingStatus = "Stock Auto is armed. Warping to the alignment preparation.";
    }

    // FlightComputer.CopyFrom preserves the BurnTarget reference, so completion evidence survives removal of the node.
    private static bool StockDeorbitComplete => _s.DeorbitAutoArmed && _s.DeorbitTarget != null
        && _s.DeorbitTarget.DeltaVTargetCci.LengthSquared() > 0
        && float3.Dot(_s.DeorbitTarget.DeltaVToGoCci, _s.DeorbitTarget.DeltaVTargetCci) <= 0;

    private static bool StockDeorbitTargetUnchanged => _s.DeorbitTarget != null
        && Math.Abs(_s.DeorbitTarget.ImpulsiveInstant.Seconds() - _s.DeorbitNodeTime) < 0.001
        && (double3.Unpack(_s.DeorbitTarget.DeltaVTargetCci) - _s.DeorbitNodeDeltaVCci).Length()
            < Math.Max(0.001, _s.DeorbitNodeDeltaVCci.Length() * 1e-5);

    private static bool StockDeorbitNodeUnchanged => _s.DeorbitNode != null
        && Math.Abs(_s.DeorbitNode.Time.Seconds() - _s.DeorbitNodeTime) < 0.001
        && (_s.DeorbitNode.DeltaVVlf - _s.DeorbitNodeDeltaV).Length() < 0.001;

    // This runs in the PrepareWorker prefix, after the input queue is drained and before vehicle workers start.
    // ApplyAutopilot releases a claimed node once the phase leaves the stock phases, before this step runs.
    private static bool StepStockDeorbit(Vehicle vehicle, Orbit orbit, IParentBody parent)
    {
        if (!IsStockDeorbitPhase(_s.LandingPhase)) return false;
        if (_s.ControlAcquired)
        {
            ReleaseVehicle(vehicle, keepWaitingCoast: true);
            return true;
        }
        DeorbitPlan plan = _s.DeorbitPlan;
        if (plan == null || !DeorbitRequestCurrent(_s.DeorbitRequest, parent) || LandingSwitchesOff)
        {
            StopStockDeorbit(vehicle, "The deorbit settings or control switches changed.", keepNode: false);
            return true;
        }
        if (vehicle.Situation.HasAnyContact())
        {
            StopStockDeorbit(vehicle, "Contact detected during deorbit. Stock Auto stopped.", keepNode: false);
            return true;
        }
        if (_s.LandingPhase == LandingPhase.DeorbitCoast)
        {
            LoadStockDeorbitNode(vehicle, parent, plan);
            return true;
        }

        FlightComputer fc = vehicle.FlightComputer;
        bool currentTarget = ReferenceEquals(fc.Burn, _s.DeorbitTarget);
        if (!StockDeorbitTargetUnchanged || !StockDeorbitNodeUnchanged || (!currentTarget && fc.Burn != null))
        {
            StopStockDeorbit(vehicle, "The deorbit node or loaded target changed. Landing stopped.", keepNode: true);
            return true;
        }
        _s.DeorbitLastResidual = _s.DeorbitTarget.DeltaVToGoCci.Length();
        bool nodeLoaded = currentTarget && ReferenceEquals(fc.BurnPlan.FindFirstExecutableBurn(), _s.DeorbitNode);
        if (_s.LandingPhase == LandingPhase.DeorbitNodePending)
            ArmStockDeorbitAuto(vehicle, plan, nodeLoaded);
        else
            MonitorStockDeorbit(vehicle, orbit, parent, plan, nodeLoaded);
        return true;
    }

    // FlightComputer.AddBurn loads the node in Manual, so Auto is selected only after the loaded target is verified.
    private static void LoadStockDeorbitNode(Vehicle vehicle, IParentBody parent, DeorbitPlan plan)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (plan.DepartureTime <= SimNow() + PrepLeadTime)
        { StopStockDeorbit(vehicle, "The deorbit node window expired. Execute again at 1x.", keepNode: false); return; }
        if (fc.BurnPlan.FindFirstExecutableBurn() != null || fc.BurnMode == FlightComputerBurnMode.Auto)
        { StopStockDeorbit(vehicle, "Clear the existing maneuver plan before executing the landing.", keepNode: true); return; }
        if (!TryTakeCraft(vehicle, acquire: false)) return;
        _s.DeorbitNodeClaimed = true;
        PatchedConic patch = vehicle.FlightPlan.TryFindPatch(new UniverseTime(plan.DepartureTime));
        if (patch == null || !ReferenceEquals(patch.Orbit.Parent, parent))
        { StopStockDeorbit(vehicle, "The deorbit time has no flight-plan patch on this body.", keepNode: false); return; }
        StateVectors state = patch.Orbit.GetStateVectorsAt(new UniverseTime(plan.DepartureTime));
        if (!DeorbitPlanner.SameState(state, plan.DeparturePosition, plan.DepartureVelocity - plan.DeltaV))
        { StopStockDeorbit(vehicle, "The flight plan changed after the deorbit solve. Execute again.", keepNode: false); return; }

        double3 dvVlf = plan.DeltaV.Transform(state.GetVlf2ParentCci().OrIdentity().Inverse());
        _s.DeorbitNodeDeltaV = dvVlf;
        _s.DeorbitNodeDeltaVCci = plan.DeltaV;
        _s.DeorbitNodeTime = plan.DepartureTime;
        _s.DeorbitNode = Burn.Create(patch.Orbit.GetPointAt(new UniverseTime(plan.DepartureTime)),
            plan.DepartureTime, dvVlf, patch, vehicle);
        _s.DeorbitNode.IsGizmoActive = false;
        fc.AddBurn(_s.DeorbitNode);
        _s.DeorbitTarget = fc.Burn;
        if (!ReferenceEquals(fc.BurnPlan.FindFirstExecutableBurn(), _s.DeorbitNode) || !StockDeorbitTargetUnchanged)
        { StopStockDeorbit(vehicle, "Stock could not load the deorbit node.", keepNode: false); return; }

        // Stock previews the burn at the throttle set here, so the minimum throttle bounds its ignition lead and duration.
        ref ManualControlInputs preview = ref ManualInputs(vehicle);
        _s.DeorbitSavedThrottle = preview.EngineThrottle;
        _s.DeorbitPreviewThrottle = Math.Max(vehicle.GetMinThrottle(), 0.001f);
        _s.DeorbitThrottleHeld = true;
        preview.EngineOn = false;
        preview.EngineThrottle = _s.DeorbitPreviewThrottle;
        _s.LandingPhase = LandingPhase.DeorbitNodePending;
        _s.LandingStatus = "Checking stock ignition timing with the engine off.";
    }

    private static void ArmStockDeorbitAuto(Vehicle vehicle, DeorbitPlan plan, bool nodeLoaded)
    {
        FlightComputer fc = vehicle.FlightComputer;
        ref ManualControlInputs inputs = ref ManualInputs(vehicle);
        if (!nodeLoaded || fc.BurnMode != FlightComputerBurnMode.Manual || inputs.EngineOn
            || inputs.EngineThrottle != _s.DeorbitPreviewThrottle)
        { StopStockDeorbit(vehicle, "Controls changed during the stock timing check. The node remains Manual.", keepNode: true); return; }
        if (plan.DepartureTime <= SimNow() + PrepLeadTime)
        { StopStockDeorbit(vehicle, "Stock timing was not ready before the preparation window. The node remains Manual.", keepNode: true); return; }
        // A new BurnTarget has no duration and an EndOfTime ignition until a vehicle worker updates it.
        if (_s.DeorbitTarget.IgnitionTime == UniverseTime.EndOfTime) return;

        double duration = _s.DeorbitTarget.BurnDuration;
        double ignition = _s.DeorbitTarget.IgnitionTime.Seconds();
        double preparation = StockPreparationTime(fc);
        double latestFinish = plan.DepartureTime + duration / 2 + _s.DeorbitRequest.MinimumPulse;
        // FlightComputer.SolveBurnThrottle cannot select a throttle below its active engine minimum.
        // The stock estimate at that minimum bounds the ignition lead and burn finish before Auto is armed.
        bool minimumPreview = fc.ActiveEnginePerformanceMax.MinThrottle > 0
            && fc.PlannedBurnThrottle <= fc.ActiveEnginePerformanceMax.MinThrottle + 1e-6;
        if (!minimumPreview || !(duration > 0) || !double.IsFinite(latestFinish + ignition + preparation)
            || ignition < SimNow() + preparation || latestFinish >= plan.ArrivalTime - PrepLeadTime)
        { StopStockDeorbit(vehicle, "Stock has no bounded burn window with enough preparation time. The node remains Manual.", keepNode: true); return; }

        RestoreDeorbitPreviewThrottle(vehicle);
        if (!StockBurnMode.GiveBackAuto(vehicle, _s.DeorbitTarget))
        { StopStockDeorbit(vehicle, "Stock could not arm the deorbit node. The node remains Manual.", keepNode: true); return; }
        _s.DeorbitAutoArmed = true;
        _s.LandingPhase = LandingPhase.DeorbitBurn;
        _s.LandingStatus = "Stock Auto will align and execute the deorbit node.";
        GuidanceLog.Info(vehicle, $"stock deorbit node armed: time {plan.DepartureTime:F3} s, delta-v {plan.DeltaV.Length():F3} m/s, minimum-throttle duration {duration:F3} s.");
    }

    private static void MonitorStockDeorbit(Vehicle vehicle, Orbit orbit, IParentBody parent, DeorbitPlan plan, bool nodeLoaded)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (StockDeorbitComplete && fc.BurnMode == FlightComputerBurnMode.Manual)
        {
            // Completion requires the stock residual reversal, including when FinishedBurnRemover has already removed the node.
            // An empty burn plan alone also occurs after a player deletion and cannot prove completion.
            GuidanceLog.Info(vehicle, $"stock deorbit complete: residual {_s.DeorbitLastResidual:F3} m/s, altitude {(orbit.StateVectors.PositionCci.Length() - parent.MeanRadius) / 1000:F3} km.");
            ReleaseStockDeorbit(vehicle, keepClaim: true);
            AcquireControl(vehicle);
            vehicle.SetEnum(VehicleEngine.MainShutdown);
            _s.DirectBrakingPlan = null;
            _s.HasCommand = false;
            _s.LandingPhase = LandingPhase.TransferPlanning;
            _s.LandingStatus = "Waiting for the engines to stop before calculating braking.";
            return;
        }
        if (!nodeLoaded || fc.BurnMode != FlightComputerBurnMode.Auto)
        { StopStockDeorbit(vehicle, "Stock Auto stopped before the deorbit node completed. Landing stopped.", keepNode: true); return; }
        if (!vehicle.IsControllable)
        { StopStockDeorbit(vehicle, "The vehicle lost control during stock deorbit execution.", keepNode: false); return; }
        StepStockDeorbitWarp(vehicle);
        if (SimNow() >= plan.ArrivalTime - PrepLeadTime)
            StopStockDeorbit(vehicle, "Stock Auto did not finish before the braking window.", keepNode: false);
    }

    // With keepNode the node, Auto and the engine stay as the player or stock left them.
    private static void StopStockDeorbit(Vehicle vehicle, string reason, bool keepNode)
    {
        ReleaseStockDeorbit(vehicle, preserve: keepNode);
        RefuseDeorbit(reason);
        GuidanceLog.Info(vehicle, reason);
    }

    private static void ReleaseStockDeorbit(Vehicle vehicle, bool keepClaim = false, bool preserve = false)
    {
        if (!_s.DeorbitNodeClaimed) return;
        StopOwnedDeorbitWarp();
        FlightComputer fc = vehicle.FlightComputer;
        RestoreDeorbitPreviewThrottle(vehicle);
        bool current = _s.DeorbitTarget != null && ReferenceEquals(fc.Burn, _s.DeorbitTarget);
        if (!preserve && current && StockDeorbitTargetUnchanged && fc.BurnMode == FlightComputerBurnMode.Auto)
        {
            StockBurnMode.HoldManual(vehicle);
            vehicle.SetEnum(VehicleEngine.MainShutdown);
        }
        // FlightComputer.RemoveBurn reloads the first remaining target even when a different node was removed.
        // Keep our node if another target has control, so cleanup cannot reset that target or its Auto mode.
        if (!preserve && StockDeorbitNodeUnchanged && (current || fc.Burn == null)
            && ReferenceEquals(fc.BurnPlan.FindFirstExecutableBurn(), _s.DeorbitNode))
            fc.RemoveBurn(_s.DeorbitNode);
        ClearStockDeorbitState();
        if (!keepClaim && !_s.ControlAcquired)
            VehicleControlOwnership.Release(vehicle, ControlClaimant.Guidance);
    }

    private static void StopOwnedDeorbitWarp()
    {
        if (double.IsFinite(_s.DeorbitWarpTime) && Universe.IsAutoWarpActive
            && Universe.AutoWarpTime == new UniverseTime(_s.DeorbitWarpTime))
            Universe.AutoWarpStop(true);
    }

    private static void ClearStockDeorbitState()
    {
        _s.DeorbitNode = null;
        _s.DeorbitTarget = null;
        _s.DeorbitAutoArmed = false;
        _s.DeorbitWarpRequested = false;
        _s.DeorbitWarpTime = double.NaN;
        _s.DeorbitNodeClaimed = false;
        _s.DeorbitNodeDeltaV = default;
        _s.DeorbitNodeDeltaVCci = default;
        _s.DeorbitNodeTime = double.NaN;
        _s.DeorbitThrottleHeld = false;
        _s.DeorbitSavedThrottle = 0;
        _s.DeorbitPreviewThrottle = 0;
    }

    private static void RestoreDeorbitPreviewThrottle(Vehicle vehicle)
    {
        if (!_s.DeorbitThrottleHeld) return;
        ref ManualControlInputs inputs = ref ManualInputs(vehicle);
        if (inputs.EngineThrottle == _s.DeorbitPreviewThrottle)
            inputs.EngineThrottle = _s.DeorbitSavedThrottle;
        _s.DeorbitThrottleHeld = false;
    }
}

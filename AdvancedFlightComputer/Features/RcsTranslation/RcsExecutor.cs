using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

// Main thread RCS burn driver. The worker consumes the published command payload.
internal static partial class RcsExecutor
{
    // One pulse is at most the stock FlightComputer.ComputeControl burn control period.
    public const float MaxPulseSec = 0.1f;

    // Worker suppression and driver completion must use the same minimum impulse fraction.
    public const float MinImpulseSuppressionFactor = 0.5f;

    private const double CapabilityRefreshSec = 1.0;
    private const double EstimateRefreshSec = 1.0;

    // Burn identity tolerance in seconds, also used by the burn editor.
    internal const double BurnIdentityToleranceSec = 0.5;

    private const float ProgressEpsilonMs = 0.001f;

    // Require a margin because the slew propellant estimate is approximate.
    private const double AlignPreferenceFactor = 0.9;

    // Ineligible ticks pause this timeout. Only delivered progress resets it.
    private const double NoProgressTimeoutSec = 15.0;

    // Accumulate slew time across gate crossings so a chattering attitude cannot reset the timeout.
    private const double AlignTimeoutSec = 120.0;

    // Delay tracking until the lead window to avoid attitude control propellant use through a long coast.
    internal const double AlignLeadFactor = 2.0;
    internal const double AlignLeadMarginSec = 15.0;

    // Use a cache lifetime in milliseconds to limit repeated thruster and tank scans from the gauge and burn editor.
    private const long UiCacheTtlMs = 250;

    // Calibrate the mass flow factor for acceleration and braking with a fully stocked vehicle.
    private const double SlewMassFlowFactor = 1.0;

    private const double AttitudeFightFactor = 1.0;

    private const double AlignMinThetaRad = 0.01;

    private const double MinSlewAlphaRadS2 = 1e-6;

    #region Resolution and capability

    public static RcsExecutionMode ResolveMode(Vehicle vehicle, RcsBurnOptions? options)
    {
        RcsExecutionMode mode = options?.Mode ?? RcsExecutionMode.Default;
        if (mode != RcsExecutionMode.Default)
            return mode;
        bool engineUsable = vehicle.IsAnyEngineActive() && vehicle.IsAnyEnginePropellantAvailable();
        return engineUsable ? RcsExecutionMode.Engine : RcsExecutionMode.Rcs;
    }

    public static bool WouldExecuteRcs(Vehicle vehicle)
        => WouldExecuteRcs(vehicle, out _);

    public static bool WouldExecuteRcs(Vehicle vehicle, out RcsCapabilitySnapshot capability)
    {
        capability = default;
        if (!ResolvesToRcs(vehicle))
            return false;
        capability = RcsCapability.Probe(vehicle);
        return capability.HasAnyTranslation;
    }

    // Resolve the burn mode without the expensive thruster probe.
    private static bool ResolvesToRcs(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        if (fc.Burn == null || !vehicle.IsControllable)
            return false;
        Burn? first = fc.BurnPlan.FindFirstExecutableBurn();
        if (first == null)
            return false;
        RcsBurnOptions? options = null;
        if (RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec))
            options = exec.FindOptions(first.Time.Seconds(), first.DeltaVVlf.Length());
        return ResolveMode(vehicle, options) == RcsExecutionMode.Rcs;
    }

    private static string _uiCacheVehicleId = string.Empty;
    private static long _uiCacheAtMs = long.MinValue;
    private static bool _uiVerdict;
    private static RcsCapabilitySnapshot _uiCapability;
    private static double _uiAvailableKg;

    // The UI asks only about the controlled vehicle.
    private static void RefreshUiCache(Vehicle vehicle)
    {
        long now = Environment.TickCount64;
        if (vehicle.Id == _uiCacheVehicleId && now - _uiCacheAtMs < UiCacheTtlMs)
            return;
        // The burn editor needs capability data for all burn modes.
        bool resolves = ResolvesToRcs(vehicle);
        _uiCapability = RcsCapability.Probe(vehicle);
        _uiVerdict = resolves && _uiCapability.HasAnyTranslation;
        _uiAvailableKg = RcsPropellant.AvailableKg(vehicle);
        _uiCacheVehicleId = vehicle.Id;
        _uiCacheAtMs = now;
    }

    public static bool WouldExecuteRcsCached(Vehicle vehicle)
    {
        RefreshUiCache(vehicle);
        return _uiVerdict;
    }

    public static RcsCapabilitySnapshot ProbeCached(Vehicle vehicle)
    {
        RefreshUiCache(vehicle);
        return _uiCapability;
    }

    public static double AvailablePropellantCached(Vehicle vehicle)
    {
        RefreshUiCache(vehicle);
        return _uiAvailableKg;
    }

    public static void ResetUiCache()
    {
        _uiCacheVehicleId = string.Empty;
        _uiCacheAtMs = long.MinValue;
        _uiVerdict = false;
        _uiCapability = default;
        _uiAvailableKg = 0.0;
    }

    public static bool IsActive(Vehicle vehicle)
        => RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec) && exec.IsActive;

    #endregion

    #region SetEnum interception

    public static bool OnBurnModeSetEnum(Vehicle vehicle, FlightComputerBurnMode mode)
    {
        if (RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec) && exec.IsActive)
        {
            // Auto cancels an active execution. Manual also cancels it and allows stock handling to continue.
            Cancel(vehicle, exec, "user request");
            if (mode == FlightComputerBurnMode.Auto)
            {
                // Restore the navball frame because this path skips the stock Manual handler.
                vehicle.SetNavBallFrame(vehicle.VehicleRegion.GetVehicleReferenceFrame());
                return false;
            }
            return true;
        }
        if (mode == FlightComputerBurnMode.Auto && !WouldExecuteRcs(vehicle))
        {
            if (DebugConfig.RcsTranslation)
                LogResolutionDebug(vehicle);
            return true;
        }
        if (mode == FlightComputerBurnMode.Auto)
        {
            // Do not fall through to the engine autopilot after a partial activation.
            try
            {
                Activate(vehicle);
            }
            catch (Exception ex)
            {
                Alert($"RCS burn could not engage on '{vehicle.Id}' (internal error, see log).");
                DefaultCategory.Log.Warning(
                    $"[AFC] RCS activation failed for vehicle='{vehicle.Id}': {ex}");
                if (RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? failed))
                    EndExecution(vehicle.FlightComputer, failed);
                RcsCommandChannel.Clear(vehicle.FlightComputer.BurnPlan);
                vehicle.SetNavBallFrame(vehicle.VehicleRegion.GetVehicleReferenceFrame());
            }
            return false;
        }
        return true;
    }

    #endregion

    #region Activation / cancellation

    public static void Activate(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        Burn? burn = fc.BurnPlan.FindFirstExecutableBurn();
        if (burn == null || fc.Burn == null)
            return;

        double timeSec = burn.Time.Seconds();
        double dvMs = burn.DeltaVVlf.Length();

        // Stock allows zero delta V burns for later gizmo edits. Do not report these as a thruster failure.
        if (!(dvMs > 0.0))
        {
            Alert($"RCS burn not engaged: the burn on '{vehicle.Id}' has no delta-V yet.");
            return;
        }

        RcsCapabilitySnapshot capability = RcsCapability.Probe(vehicle);
        if (!capability.HasAnyTranslation)
        {
            Alert($"RCS burn not engaged: no usable RCS translation on '{vehicle.Id}'.");
            return;
        }

        RcsEstimates estimates = ComputeEstimates(vehicle, fc.Burn, in capability);
        RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? existing);
        RcsBurnOptions? requested = existing?.FindOptions(timeSec, dvMs);
        (RcsAttitudeStrategy strategy, int axis) = ResolveStrategy(
            requested?.Attitude ?? RcsAttitudeStrategy.Auto, in estimates);

        // Hold requires every demanded group. Align requires a feasible slew.
        if (strategy == RcsAttitudeStrategy.Hold && !estimates.HoldFeasible)
        {
            if (estimates.AlignFeasible && estimates.AlignAxis >= 0)
            {
                strategy = RcsAttitudeStrategy.Align;
                axis = estimates.AlignAxis;
            }
            else
            {
                Alert($"RCS burn not engaged: no thruster axis can serve the burn direction on '{vehicle.Id}'.");
                return;
            }
        }

        RcsExecution exec = existing ?? RcsExecRegistry.GetOrCreate(vehicle.Id);
        RcsBurnOptions options = exec.GetOrCreateOptions(timeSec, dvMs);
        exec.Capability = capability;
        exec.CapabilityProbedAtSec = Universe.GetElapsedTime().Seconds();
        exec.Estimates = estimates;
        exec.EstimatesComputedAtSec = exec.CapabilityProbedAtSec;

        PrepareAllocation(vehicle, fc, fc.Burn, exec, options, strategy, dvMs);

        exec.ActiveBurn = burn;
        exec.ActiveBurnTimeSec = timeSec;
        exec.ActiveBurnDvMs = dvMs;
        exec.ResolvedStrategy = strategy;
        exec.ResolvedAxis = axis;
        exec.ControlTaken = false;

        exec.BaselineFuel(fc, exec.CapabilityProbedAtSec);

        if (!BeginControl(vehicle, fc, exec))
            return;
        PublishCommand(vehicle, exec);
        LogEngaged(vehicle, exec, dvMs);
    }

    private static void PrepareAllocation(
        Vehicle vehicle, FlightComputer fc, BurnTarget bt, RcsExecution exec, RcsBurnOptions options,
        RcsAttitudeStrategy strategy, double dvMs)
    {
        // Solve first so the sufficiency warning uses the LP pattern cost.
        exec.ResolvedAllocator = options.Allocator;
        if (exec.ResolvedAllocator == RcsAllocator.Lp)
            EnsureLpSolution(vehicle, fc, exec, ImpulseCtrlFromTogo(vehicle, fc, bt),
                Universe.GetElapsedTime().Seconds());

        // Allow the user to accept a partial burn or refill during execution, even when the estimate exceeds available propellant.
        double neededKg = exec.LpSecondsPerImpulse != null
            ? (exec.LpCostPerImpulse + exec.LpSlackCostPerImpulse)
              * fc.TotalMassPropsBody.Mass * dvMs
            : exec.Estimates.RequiredPropellantKg(strategy);
        double availableKg = RcsPropellant.AvailableKg(vehicle);
        if (neededKg > availableKg)
            Alert($"RCS burn may run out of propellant: needs ~{neededKg:F0} kg, {availableKg:F0} kg available.");
    }

    private static bool BeginControl(Vehicle vehicle, FlightComputer fc, RcsExecution exec)
    {
        // Keep stock engine automation off while the RCS worker owns the burn.
        fc.BurnMode = FlightComputerBurnMode.Manual;
        vehicle.SetNavBallFrame(VehicleReferenceFrame.BurnBody);

        if (!EnsureBurnControl(vehicle, fc, exec, exec.CapabilityProbedAtSec))
        {
            Alert($"RCS burn not engaged: cannot align or hold for the burn direction on '{vehicle.Id}'.");
            EndExecution(fc, exec);
            return false;
        }
        return true;
    }

    private static void LogEngaged(Vehicle vehicle, RcsExecution exec, double dvMs)
    {
        RcsAttitudeStrategy strategy = exec.ResolvedStrategy;
        int axis = exec.ResolvedAxis;

        DefaultCategory.Log.Info(
            $"[AFC] RCS burn engaged: vehicle='{vehicle.Id}' dv={dvMs:F2}m/s " +
            $"strategy={strategy}{(axis >= 0 ? $" axis={AxisName(axis)}" : string.Empty)} " +
            $"allocator={exec.ResolvedAllocator}");
        if (DebugConfig.RcsTranslation)
        {
            ref readonly RcsEstimates est = ref exec.Estimates;
            DefaultCategory.Log.Debug(
                $"[AFC] RCS estimates: hold feasible={est.HoldFeasible} " +
                $"{est.HoldPropellantKg:F1}kg/{est.HoldDurationSec:F0}s, " +
                $"align feasible={est.AlignFeasible} axis={AxisName(Math.Max(est.AlignAxis, 0))} " +
                $"{est.AlignPropellantKg + est.AlignSlewPropellantKg:F1}kg/{est.AlignDurationSec:F0}s " +
                $"(slew {est.AlignSlewDurationSec:F0}s), mass={vehicle.FlightComputer.TotalMassPropsBody.Mass:F0}kg");
        }
    }

    public static void Cancel(Vehicle vehicle, RcsExecution exec, string reason)
    {
        RcsFuelSummary fuel = ComputeFuelSummary(vehicle.FlightComputer, exec);
        EndExecution(vehicle.FlightComputer, exec);
        RcsCommandChannel.Clear(vehicle.FlightComputer.BurnPlan);
        DefaultCategory.Log.Info($"[AFC] RCS burn cancelled ({reason}): vehicle='{vehicle.Id}'");
        LogFuel(vehicle, in fuel);
    }

    // FlightComputer.UpdateActiveControlSystems requires RCSMode.Enabled for rotation authority.
    private static bool ForceRcsOn(FlightComputer fc, RcsExecution exec, bool captureSetting)
    {
        if (captureSetting)
            exec.ForcedRcsOn = fc.RCSMode == FlightComputerRCSMode.Disabled;
        if (fc.RCSMode != FlightComputerRCSMode.Disabled)
            return false;
        fc.RCSMode = FlightComputerRCSMode.Enabled;
        return true;
    }

    private static void RestoreRcsMode(FlightComputer fc, RcsExecution exec)
    {
        if (exec.ForcedRcsOn)
            fc.RCSMode = FlightComputerRCSMode.Disabled;
    }

    // Restore control before ClearActive erases the ownership flags. An uncommanded Align leaves the tracker alone.
    private static void EndExecution(FlightComputer fc, RcsExecution exec)
    {
        if (exec.AlignCommanded)
            fc.SetNullRot(VehicleReferenceFrame.BurnBody);
        RestoreRcsMode(fc, exec);
        exec.ClearActive();
    }

    private static void LogResolutionDebug(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        Burn? first = fc.BurnPlan.FindFirstExecutableBurn();
        if (fc.Burn == null || first == null)
        {
            DefaultCategory.Log.Debug(
                $"[AFC] RCS resolution vehicle='{vehicle.Id}': no loaded burn -> stock Auto.");
            return;
        }
        RcsBurnOptions? options = null;
        if (RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec))
            options = exec.FindOptions(first.Time.Seconds(), first.DeltaVVlf.Length());
        RcsExecutionMode resolved = ResolveMode(vehicle, options);
        RcsCapabilitySnapshot cap = RcsCapability.Probe(vehicle);
        int best = cap.BestAxis();
        DefaultCategory.Log.Debug(
            $"[AFC] RCS resolution vehicle='{vehicle.Id}': burn t={first.Time.Seconds():F1}s " +
            $"dv={first.DeltaVVlf.Length():F2}m/s, " +
            $"options={(options == null ? "none" : $"{options.Mode}/{options.Attitude}")}, " +
            $"resolved={resolved}, engineActive={vehicle.IsAnyEngineActive()} " +
            $"engineFueled={vehicle.IsAnyEnginePropellantAvailable()}, " +
            $"rcsTranslation={cap.HasAnyTranslation}" +
            $"{(best >= 0 ? $" bestAxis={AxisName(best)}" : string.Empty)}, " +
            $"controllable={vehicle.IsControllable} -> stock Auto.");
    }

    internal static (RcsAttitudeStrategy, int) ResolveStrategy(
        RcsAttitudeStrategy requested, in RcsEstimates estimates)
    {
        int alignAxis = estimates.AlignAxis;
        bool alignPossible = estimates.AlignFeasible && alignAxis >= 0;
        switch (requested)
        {
            case RcsAttitudeStrategy.Hold:
                return (RcsAttitudeStrategy.Hold, -1);
            case RcsAttitudeStrategy.Align:
                return alignPossible
                    ? (RcsAttitudeStrategy.Align, alignAxis)
                    : (RcsAttitudeStrategy.Hold, -1);
            default:
                if (!estimates.HoldFeasible && alignPossible)
                    return (RcsAttitudeStrategy.Align, alignAxis);
                if (alignPossible && estimates.Valid
                    && estimates.AlignPropellantKg + estimates.AlignSlewPropellantKg
                       < AlignPreferenceFactor * estimates.HoldPropellantKg)
                    return (RcsAttitudeStrategy.Align, alignAxis);
                return (RcsAttitudeStrategy.Hold, -1);
        }
    }

    // BurnBody +X is the burn direction. Other control axes use a Custom target with a checked Euler round trip.
    private static bool CommandAlignAttitude(FlightComputer fc, int axisIdx)
    {
        fc.AttitudeMode = FlightComputerAttitudeMode.Auto;
        if (axisIdx == 0)
        {
            fc.TrackTarget(FlightComputerAttitudeTrackTarget.PositiveDv);
            return true;
        }
        if (axisIdx == 1)
        {
            fc.TrackTarget(FlightComputerAttitudeTrackTarget.NegativeDv);
            return true;
        }

        double3 axis = double3.Unpack(RcsCapabilitySnapshot.AxisDirection(axisIdx));
        doubleQuat body2Frame = ShortestArc(axis, double3.UnitX);
        double3 euler = VehicleReferenceFrame.BurnBody.QuaternionToEulerAngles(body2Frame);

        // Check the Custom target Euler round trip so an unrepresentable target falls back to Hold.
        doubleQuat roundTrip = VehicleReferenceFrame.BurnBody.EulerAnglesToQuaternion(euler);
        if ((axis.Transform(roundTrip) - double3.UnitX).Length() > 0.001)
        {
            LogHelper.WarnOnce("rcs-align-euler",
                "[AFC] RCS Align: euler round trip degenerate for the chosen axis, holding attitude instead.");
            return false;
        }
        fc.AttitudeFrame = VehicleReferenceFrame.BurnBody;
        fc.AttitudeTrackTarget = FlightComputerAttitudeTrackTarget.Custom;
        fc.CustomAttitudeTarget = euler;
        return true;
    }

    // Convert CCI delta V to control frame impulse in N s.
    private static float3 ImpulseCtrlFromTogo(Vehicle vehicle, FlightComputer fc, BurnTarget bt)
        => float3.Pack(double3.Unpack(bt.DeltaVToGoCci).Transform(vehicle.GetCtrl2Cci().Inverse()))
           * fc.TotalMassPropsBody.Mass;

    private static doubleQuat ShortestArc(double3 from, double3 to)
    {
        double d = double3.Dot(from, to);
        if (d > 1.0 - 1e-12)
            return doubleQuat.Identity;
        double3 axis = double3.Cross(from, to);
        if (axis.IsNearlyZero())
            axis = from.GetAnyOrthogonalDirection();
        else
            axis = axis.Normalized();
        return QuaternionEx.CreateFromAxisAngle(axis, MathEx.SafeAcos(d));
    }

    #endregion

    #region Per-tick driver

    // Universe.ApplyVehicleSolvers has applied worker results before this driver runs.
    public static void Tick(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        bool hasExec = RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec);
        if (!hasExec && fc.Burn == null)
        {
            RcsCommandChannel.Clear(fc.BurnPlan);
            return;
        }

        double nowSec = Universe.GetElapsedTime().Seconds();

        if (hasExec && !exec!.ReconciledAfterLoad)
            Reconcile(vehicle, fc, exec);

        if (hasExec && exec!.IsActive)
        {
            TickActive(vehicle, fc, exec, nowSec);
            return;
        }

        // Clear stale commands before stock can run an engine burn. Keep estimates available before activation.
        RcsCommandChannel.Clear(fc.BurnPlan);
        if (fc.Burn == null || !ResolvesToRcs(vehicle))
            return;

        // Reuse a recent snapshot because the probe solves each thruster nozzle.
        if (hasExec && IsCapabilityFresh(RcsCtrlFrame.For(vehicle).Ctrl2Body, exec!, nowSec))
        {
            if (exec!.Capability.HasAnyTranslation)
                RefreshArmedEstimates(vehicle, fc, exec, nowSec);
            return;
        }

        // Cache negative results to avoid a probe on every frame.
        RcsExecution armed = hasExec ? exec! : RcsExecRegistry.GetOrCreate(vehicle.Id);
        armed.Capability = RcsCapability.Probe(vehicle);
        armed.CapabilityProbedAtSec = nowSec;
        if (armed.Capability.HasAnyTranslation)
            RefreshArmedEstimates(vehicle, fc, armed, nowSec);
    }

    private static void Reconcile(Vehicle vehicle, FlightComputer fc, RcsExecution exec)
    {
        exec.ReconciledAfterLoad = true;
        exec.PruneOrphanedOptions(fc.BurnPlan);
        if (!exec.IsActive)
            return;
        Burn? burn = RcsExecution.FindBurn(
            fc.BurnPlan, exec.ActiveBurnTimeSec!.Value, exec.ActiveBurnDvMs ?? 0.0);
        if (burn == null)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] RCS burn for vehicle='{vehicle.Id}' not found after load, cancelling.");
            // A loaded save can retain forced controls even when its burn vanished. Restore them before clearing state.
            exec.LastFuel = default;
            EndExecution(fc, exec);
            return;
        }
        exec.ActiveBurn = burn;
        exec.ControlTaken = exec.AlignCommanded || exec.ForcedRcsOn;
        // The next driver tick applies the same align lead gate after load.

        // Restart telemetry at load so the summary covers only the observed portion.
        exec.BaselineFuel(fc, Universe.GetElapsedTime().Seconds());

        DefaultCategory.Log.Info(
            $"[AFC] RCS burn reattached after load: vehicle='{vehicle.Id}' t={burn.Time.Seconds():F1}s");
    }

    private static void TickActive(Vehicle vehicle, FlightComputer fc, RcsExecution exec, double nowSec)
    {
        BurnTarget? bt = fc.Burn;
        Burn? burn = exec.ActiveBurn;

        // Cancel a deleted or reordered burn instead of firing at another target.
        if (burn == null || bt == null
            || !fc.BurnPlan.TryGetBurn(burn)
            || Math.Abs((bt.ImpulsiveInstant - burn.Time).Seconds()) > BurnIdentityToleranceSec)
        {
            Cancel(vehicle, exec, "burn no longer loaded");
            return;
        }

        double tickDt = ObserveWorker(exec, nowSec);

        bool ctrlFrameSettled = RefreshCapability(vehicle, exec, nowSec);
        if (!exec.Capability.HasAnyTranslation)
        {
            Alert($"RCS burn stalled: no usable RCS translation left on '{vehicle.Id}' " +
                  "(thrusters inactive, out of propellant, or no usable axis).");
            Cancel(vehicle, exec, "no usable translation");
            return;
        }

        if (!EnsureBurnControl(vehicle, fc, exec, nowSec))
        {
            Alert($"RCS burn cancelled: cannot align or hold for the burn direction on '{vehicle.Id}'.");
            Cancel(vehicle, exec, "cannot align or hold");
            return;
        }

        float3 togo = bt.DeltaVToGoCci;
        float togoMs = togo.Length();
        float3 impulseCtrl = ImpulseCtrlFromTogo(vehicle, fc, bt);

        // Match the worker firing gate. Before Align is commanded, angle error belongs to the user target.
        bool slewing = exec.ResolvedStrategy == RcsAttitudeStrategy.Align
            && exec.AlignCommanded
            && OutsideAlignGate(fc);
        bool firingEligible = ControlCommandDue(exec, bt, nowSec)
            && !slewing && nowSec >= bt.IgnitionTime.Seconds();

        // Do not solve patterns while the frame rotates and the attitude gate prevents firing.
        if (exec.ResolvedAllocator == RcsAllocator.Lp && !slewing)
            EnsureLpSolution(vehicle, fc, exec, impulseCtrl, nowSec);

        AccumulateFuel(exec, fc, bt, impulseCtrl, slewing, firingEligible);

        // Only compare floors when firing is eligible and both impulses use the same control frame.
        bool belowFloor = firingEligible && ctrlFrameSettled
            && (exec.LpSecondsPerImpulse != null
                ? IsBelowLpFloor(impulseCtrl, fc, exec)
                : IsBelowImpulseFloor(impulseCtrl, in exec.Capability));
        if (float3.Dot(togo, bt.DeltaVTargetCci) <= 0f || belowFloor)
        {
            Complete(vehicle, fc, exec, togoMs);
            return;
        }

        if (!CheckProgress(vehicle, exec, togoMs, tickDt, nowSec, slewing, firingEligible))
            return;

        if (nowSec - exec.EstimatesComputedAtSec > EstimateRefreshSec)
        {
            exec.Estimates = ComputeEstimates(vehicle, bt, in exec.Capability);
            exec.EstimatesComputedAtSec = nowSec;
        }

        LogFiringWindow(vehicle, fc, exec, bt, impulseCtrl, togoMs, nowSec);

        PublishCommand(vehicle, exec);
    }

    private static double ObserveWorker(RcsExecution exec, double nowSec)
    {
        // The first active tick contributes no elapsed time to the watchdogs.
        double tickDt = double.IsNaN(exec.LastTickSimSec)
            ? 0.0
            : Math.Max(0.0, nowSec - exec.LastTickSimSec);
        exec.LastTickSimSec = nowSec;
        if (double.IsNaN(exec.LastWorkerReadAtSec) || exec.LastPublishedCommand?.WasConsumed == true)
            exec.LastWorkerReadAtSec = nowSec;
        return tickDt;
    }

    private static bool RefreshCapability(Vehicle vehicle, RcsExecution exec, double nowSec)
    {
        floatQuat liveCtrl2Body = RcsCtrlFrame.For(vehicle).Ctrl2Body;
        if (!IsCapabilityFresh(liveCtrl2Body, exec, nowSec))
        {
            exec.Capability = RcsCapability.Probe(vehicle);
            exec.CapabilityProbedAtSec = nowSec;
        }

        // Rocket.UpdateThrusterCache can lag the live control frame. Do not compare completion floors until they agree.
        return liveCtrl2Body == exec.Capability.Ctrl2Body;
    }

    // A control point change invalidates all groups immediately, even inside the refresh interval.
    private static bool IsCapabilityFresh(floatQuat liveCtrl2Body, RcsExecution exec, double nowSec)
        => liveCtrl2Body == exec.Capability.Ctrl2Body
           && nowSec - exec.CapabilityProbedAtSec <= CapabilityRefreshSec;

    private static bool CheckProgress(
        Vehicle vehicle, RcsExecution exec, float togoMs, double tickDt, double nowSec,
        bool slewing, bool firingEligible)
    {
        // Accumulate only firing eligible time. Slew and coast pause the timeout. Only progress resets it.
        if (!firingEligible)
        {
            // Track coast growth so the next progress check uses the pre burn remainder.
            if (togoMs > exec.WatchdogTogoMs)
                exec.WatchdogTogoMs = togoMs;
        }
        else if (togoMs < exec.WatchdogTogoMs - ProgressEpsilonMs)
        {
            exec.WatchdogTogoMs = togoMs;
            exec.NoProgressAccumSec = 0.0;
            exec.SlewAccumSec = 0.0;
        }
        else
        {
            exec.NoProgressAccumSec += tickDt;
            if (exec.NoProgressAccumSec > NoProgressTimeoutSec)
            {
                Alert($"RCS burn stalled: no progress on '{vehicle.Id}' " +
                      $"({togoMs:F2}m/s to go). Check thruster coverage for the burn direction.");
                WarnUnreadCommand(vehicle, exec, nowSec);
                Cancel(vehicle, exec, "no progress");
                return false;
            }
        }

        // Accumulate slew time across gate crossings so oscillation cannot keep the burn active forever.
        if (slewing)
        {
            exec.SlewAccumSec += tickDt;
            if (exec.SlewAccumSec > AlignTimeoutSec)
            {
                Alert($"RCS burn cancelled: '{vehicle.Id}' cannot reach the burn attitude " +
                      $"(slewing for {AlignTimeoutSec:F0}s without progress).");
                WarnUnreadCommand(vehicle, exec, nowSec);
                Cancel(vehicle, exec, "cannot reach burn attitude");
                return false;
            }
        }

        return true;
    }

    private static void WarnUnreadCommand(Vehicle vehicle, RcsExecution exec, double nowSec)
    {
        if (nowSec - exec.LastWorkerReadAtSec > NoProgressTimeoutSec)
            LogHelper.WarnOnce($"rcs-worker-unread-{vehicle.Id}",
                $"[AFC] RCS burn stalled on '{vehicle.Id}': no worker command read observed " +
                $"for {nowSec - exec.LastWorkerReadAtSec:F1}s. Check the burn-plan command channel.");
    }

    private static void LogFiringWindow(
        Vehicle vehicle, FlightComputer fc, RcsExecution exec, BurnTarget bt,
        float3 impulseCtrl, float togoMs, double nowSec)
    {
        if (DebugConfig.RcsTranslation && !exec.FiringLogged
            && ControlCommandDue(exec, bt, nowSec)
            && nowSec >= bt.IgnitionTime.Seconds())
        {
            exec.FiringLogged = true;
            DefaultCategory.Log.Debug(
                $"[AFC] RCS firing window entered: vehicle='{vehicle.Id}' " +
                $"togo={togoMs:F2}m/s duration est={bt.BurnDuration:F1}s " +
                $"allocator={exec.ResolvedAllocator}");
            if (exec.LpSecondsPerImpulse == null)
            {
                ref readonly RcsCapabilitySnapshot cap = ref exec.Capability;
                DefaultCategory.Log.Debug(
                    $"[AFC]   axis impulse split: X={impulseCtrl.X / 1000f:F1}kNs " +
                    $"(F {(impulseCtrl.X >= 0f ? cap.Ax0.ForceN : cap.Ax1.ForceN) / 1000f:F1}kN), " +
                    $"Y={impulseCtrl.Y / 1000f:F1}kNs " +
                    $"(F {(impulseCtrl.Y >= 0f ? cap.Ax2.ForceN : cap.Ax3.ForceN) / 1000f:F1}kN), " +
                    $"Z={impulseCtrl.Z / 1000f:F1}kNs " +
                    $"(F {(impulseCtrl.Z >= 0f ? cap.Ax4.ForceN : cap.Ax5.ForceN) / 1000f:F1}kN)");
            }
            else
            {
                DefaultCategory.Log.Debug(
                    $"[AFC]   LP pattern throughput ~{exec.LpImpulseCapNs / MaxPulseSec / 1000f:F1}kN " +
                    $"({exec.LpCostPerImpulse * 1e6:F1}mg per Ns)");
            }
        }
    }

    private static void Complete(Vehicle vehicle, FlightComputer fc, RcsExecution exec, float residualMs)
    {
        double burnTime = exec.ActiveBurnTimeSec ?? 0.0;
        double burnDv = exec.ActiveBurnDvMs ?? 0.0;
        Burn? completedBurn = exec.ActiveBurn;
        RcsFuelSummary fuel = ComputeFuelSummary(fc, exec);
        EndExecution(fc, exec);
        RcsBurnOptions? options = exec.FindOptions(burnTime, burnDv);
        if (options != null)
            exec.Options.Remove(options);
        RcsCommandChannel.Clear(fc.BurnPlan);
        float accumMs = fc.Burn?.DeltaVAccumCci.Length() ?? 0f;
        DefaultCategory.Log.Info(
            $"[AFC] RCS burn complete: vehicle='{vehicle.Id}' " +
            $"accumulated={accumMs:F3}m/s of {burnDv:F2}m/s, residual={residualMs:F3}m/s");
        LogFuel(vehicle, in fuel);

        // Notify subscribers after teardown because they can remove the completed burn.
        if (completedBurn != null)
            RcsBurnCompletions.Raise(vehicle, completedBurn);
    }

    // FlightComputer.ComputeRcsTrackAxis coasts inside this corridor. A tighter gate can wait on drift.
    internal static bool OutsideAlignGate(FlightComputer fc)
    {
        float gateY = Math.Max(fc.AngleDeadband, 0.5f * fc.AngleDeadband + fc.AngleTurnaround.Y);
        float gateZ = Math.Max(fc.AngleDeadband, 0.5f * fc.AngleDeadband + fc.AngleTurnaround.Z);
        return Math.Abs(fc.ErrorAngles.Y) > gateY || Math.Abs(fc.ErrorAngles.Z) > gateZ;
    }

    // Must match RcsComputeControlPatch.ShapeAxis component by component, including unusable groups.
    internal static bool IsBelowImpulseFloor(float3 impulseCtrl, in RcsCapabilitySnapshot cap)
    {
        Span<float> components = stackalloc float[6]
        {
            Math.Max(impulseCtrl.X, 0f), Math.Max(-impulseCtrl.X, 0f),
            Math.Max(impulseCtrl.Y, 0f), Math.Max(-impulseCtrl.Y, 0f),
            Math.Max(impulseCtrl.Z, 0f), Math.Max(-impulseCtrl.Z, 0f),
        };
        for (int i = 0; i < 6; i++)
        {
            RcsAxisGroup g = cap.Get(i);
            if (g.IsUsable && components[i] >= g.MinCorrectingImpulseNs)
                return false;
        }
        return true;
    }

    #region Fuel telemetry

    private static void AccumulateFuel(
        RcsExecution exec, FlightComputer fc, BurnTarget bt,
        float3 impulseCtrl, bool slewing, bool firingEligible)
    {
        if (exec.StartMassKg <= 0.0)
            return;
        double massNow = fc.TotalMassPropsBody.Mass;
        // A refill or docking mass gain must not subtract already consumed propellant.
        double burnedTick = Math.Max(0.0, exec.LastTickMassKg - massNow);
        exec.BurnedPropellantKg += burnedTick;
        if (slewing)
            exec.SlewPropellantKg += burnedTick;
        else if (!firingEligible)
            exec.CoastPropellantKg += burnedTick;
        exec.LastTickMassKg = massNow;

        float3 accumNow = bt.DeltaVAccumCci;
        if (firingEligible)
        {
            double deliveredNs = massNow * (accumNow - exec.LastAccumCci).Length();
            if (deliveredNs > 0.0)
            {
                double costPerNs = exec.LpSecondsPerImpulse != null
                    ? exec.LpCostPerImpulse
                    : GroupCostPerNs(in exec.Capability,
                        double3.Unpack(impulseCtrl).NormalizeOrZero());
                exec.TranslationPropellantKg += deliveredNs * costPerNs;
            }
        }
        // Always advance the baseline so coast delta V is not later attributed to translation.
        exec.LastAccumCci = accumNow;
    }

    private static RcsFuelSummary ComputeFuelSummary(FlightComputer fc, RcsExecution exec)
    {
        if (exec.StartMassKg <= 0.0)
        {
            // Do not retain the previous burn summary when no current baseline exists.
            exec.LastFuel = default;
            return default;
        }
        // Mass gains are excluded from accumulated propellant use.
        double totalKg = exec.BurnedPropellantKg;

        double dvMs = 0.0;
        double veMs = 0.0;
        double angleDeg = 0.0;
        BurnTarget? bt = fc.Burn;
        // A cancellation can arrive after the loaded target has changed to another burn.
        bool btMatches = bt != null && exec.ActiveBurnTimeSec.HasValue
            && Math.Abs(bt.ImpulsiveInstant.Seconds() - exec.ActiveBurnTimeSec.Value)
               <= BurnIdentityToleranceSec;
        if (btMatches)
        {
            double3 accum = double3.Unpack(bt!.DeltaVAccumCci);
            double3 target = double3.Unpack(bt.DeltaVTargetCci);
            // Pair observed delta V with observed propellant after load. The angle still describes the whole burn.
            dvMs = (accum - double3.Unpack(exec.StartAccumCci)).Length();
            if (totalKg > 1e-9 && dvMs > 0.0)
                veMs = exec.StartMassKg * dvMs / totalKg;
            if (!accum.IsNearlyZero() && !target.IsNearlyZero())
                angleDeg = MathEx.SafeAcos(double3.Dot(accum.Normalized(), target.Normalized()))
                    * (180.0 / Math.PI);
        }

        RcsFuelSummary fuel = new()
        {
            Valid = true,
            TotalKg = totalKg,
            TranslationKg = exec.TranslationPropellantKg,
            SlewKg = exec.SlewPropellantKg,
            CoastKg = exec.CoastPropellantKg,
            AttitudeKg = totalKg - exec.TranslationPropellantKg
                - exec.SlewPropellantKg - exec.CoastPropellantKg,
            EffectiveVeMs = veMs,
            DvAngleDeg = angleDeg,
            ElapsedSec = Universe.GetElapsedTime().Seconds() - exec.EngagedAtSec,
        };
        exec.LastFuel = fuel;
        return fuel;
    }

    private static void LogFuel(Vehicle vehicle, in RcsFuelSummary fuel)
    {
        if (!fuel.Valid)
            return;
        DefaultCategory.Log.Info(
            $"[AFC] RCS burn fuel: vehicle='{vehicle.Id}' total={fuel.TotalKg:F1}kg " +
            $"(translation {fuel.TranslationKg:F1}kg, slew {fuel.SlewKg:F1}kg, " +
            $"coast {fuel.CoastKg:F1}kg, attitude {fuel.AttitudeKg:F1}kg), " +
            $"ve_eff={fuel.EffectiveVeMs:F0}m/s, " +
            $"dv angle={fuel.DvAngleDeg:F2}deg, elapsed={fuel.ElapsedSec:F1}s");
    }

    #endregion

    internal static double AlignLeadSeconds(in RcsEstimates estimates)
    {
        double slewSec = estimates.AlignSlewDurationSec;
        if (!double.IsFinite(slewSec) || slewSec < 0.0)
            slewSec = 0.0;
        return AlignLeadFactor * slewSec + AlignLeadMarginSec;
    }

    // Stock UpdateBurnTarget still supplies engine timing at activation. Anchor on the impulsive instant instead.
    private static bool ControlCommandDue(RcsExecution exec, BurnTarget bt, double nowSec)
        => exec.ControlTaken
           || nowSec >= bt.ImpulsiveInstant.Seconds() - AlignLeadSeconds(in exec.Estimates);

    private static bool EnsureBurnControl(
        Vehicle vehicle, FlightComputer fc, RcsExecution exec, double nowSec)
    {
        BurnTarget? bt = fc.Burn;
        if (bt == null || !ControlCommandDue(exec, bt, nowSec))
            return true;
        bool takingControl = !exec.ControlTaken;
        exec.ControlTaken = true;

        // FlightComputer.UpdateActiveControlSystems needs enabled RCS before it scans rotation authority.
        if (ForceRcsOn(fc, exec, takingControl) && DebugConfig.RcsTranslation)
            DefaultCategory.Log.Debug(
                $"[AFC] RCS: enabled RCSMode inside the control lead window on vehicle='{vehicle.Id}'.");

        // Rate hold counters residual torque from off center translation thrusters.
        if (fc.AttitudeMode == FlightComputerAttitudeMode.Manual)
        {
            fc.RateHold(vehicle.NavBallData.Frame);
            if (DebugConfig.RcsTranslation)
                DefaultCategory.Log.Debug(
                    $"[AFC] RCS: engaged rate hold inside the control lead window on vehicle='{vehicle.Id}'.");
        }

        return EnsureAlignCommanded(fc, exec);
    }

    // A failed target command falls back to Hold only if Hold is feasible.
    private static bool EnsureAlignCommanded(FlightComputer fc, RcsExecution exec)
    {
        if (exec.ResolvedStrategy != RcsAttitudeStrategy.Align)
            return true;
        if (!exec.AlignCommanded)
        {
            if (CommandAlignAttitude(fc, exec.ResolvedAxis))
            {
                exec.AlignCommanded = true;
                return true;
            }
        }
        else
        {
            bool tracking = fc.AttitudeMode == FlightComputerAttitudeMode.Auto
                && fc.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.None;
            if (tracking || CommandAlignAttitude(fc, exec.ResolvedAxis))
                return true;
        }
        exec.ResolvedStrategy = RcsAttitudeStrategy.Hold;
        exec.ResolvedAxis = -1;
        exec.AlignCommanded = false;
        return exec.Estimates.HoldFeasible;
    }

    private static void Alert(string message)
    {
        TimedAlert.Create(message, Color.Red);
        DefaultCategory.Log.Warning($"[AFC] {message}");
    }

    #endregion

    #region Command publishing

    private static void RefreshArmedEstimates(
        Vehicle vehicle, FlightComputer fc, RcsExecution exec, double nowSec)
    {
        BurnTarget? bt = fc.Burn;
        if (bt == null)
            return;
        if (nowSec - exec.EstimatesComputedAtSec > EstimateRefreshSec || !exec.Estimates.Valid)
        {
            exec.Estimates = ComputeEstimates(vehicle, bt, in exec.Capability);
            exec.EstimatesComputedAtSec = nowSec;
        }
    }

    private static void PublishCommand(Vehicle vehicle, RcsExecution exec)
    {
        FlightComputer fc = vehicle.FlightComputer;
        BurnTarget? bt = fc.Burn;
        if (bt == null)
            return;

        bool align = exec.ResolvedStrategy == RcsAttitudeStrategy.Align;
        double duration = 0.0;
        if (exec.LpSecondsPerImpulse != null && exec.LpImpulseCapNs > 0f)
        {
            // The busiest LP thruster limits throughput. Axis group duration does not describe a sparse pattern.
            double totalImpulse = bt.DeltaVTargetCci.Length() * fc.TotalMassPropsBody.Mass;
            duration = totalImpulse * MaxPulseSec / exec.LpImpulseCapNs;
        }
        else if (exec.Estimates.Valid)
        {
            duration = align ? exec.Estimates.AlignDurationSec : exec.Estimates.HoldDurationSec;
        }
        // Keep ignition centered on the impulse. A nonfinite duration uses zero lead to avoid an invalid UniverseTime.
        double ignitionLead = double.IsFinite(duration) ? 0.5 * duration : 0.0;

        ref readonly RcsCapabilitySnapshot cap = ref exec.Capability;
        RcsWorkerCommand command = new()
        {
            Active = exec.ControlTaken,
            IgnitionTime = bt.ImpulsiveInstant - ignitionLead,
            RequireAttitude = align,
            MaxPulseSec = MaxPulseSec,
            AxisForcePos = new float3(cap.Ax0.ForceN, cap.Ax2.ForceN, cap.Ax4.ForceN),
            AxisForceNeg = new float3(cap.Ax1.ForceN, cap.Ax3.ForceN, cap.Ax5.ForceN),
            AxisMinCorrectingImpulsePos = new float3(cap.Ax0.MinCorrectingImpulseNs, cap.Ax2.MinCorrectingImpulseNs, cap.Ax4.MinCorrectingImpulseNs),
            AxisMinCorrectingImpulseNeg = new float3(cap.Ax1.MinCorrectingImpulseNs, cap.Ax3.MinCorrectingImpulseNs, cap.Ax5.MinCorrectingImpulseNs),
            LpSecondsPerImpulse = exec.LpSecondsPerImpulse,
            LpDirCtrl = exec.LpDirCtrl,
            LpImpulseCapNs = exec.LpImpulseCapNs,
        };
        exec.LastPublishedCommand = command;
        RcsCommandChannel.Publish(fc.BurnPlan, command);
    }

    #endregion

}

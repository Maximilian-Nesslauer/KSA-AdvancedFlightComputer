using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Main-thread service driving RCS translation burns. UI, the SetEnum
/// interception, and future features (multi-pass) all go through this API;
/// the worker-side ComputeControl postfix only consumes the published
/// <see cref="RcsWorkerCommand"/>.
/// </summary>
internal static class RcsExecutor
{
    /// <summary>Stock burn-control cadence; one commanded pulse never exceeds
    /// this so the loop re-plans at the same rate stock auto burns do.</summary>
    public const float MaxPulseSec = 0.1f;

    /// <summary>Fraction of a group's minimum impulse below which firing
    /// overshoots more than it corrects. Shared between the worker's axis
    /// suppression and the main-thread completion floor: the two must agree
    /// or an execution could stay active while the worker never fires.</summary>
    public const float MinImpulseSuppressionFactor = 0.5f;

    private const double CapabilityRefreshSec = 1.0;
    private const double EstimateRefreshSec = 1.0;

    /// <summary>Matches the loaded BurnTarget to a plan burn by impulse
    /// time; also gates the burn editor's estimate display.</summary>
    internal const double BurnIdentityToleranceSec = 0.5;

    /// <summary>Smallest to-go reduction that counts as progress for the
    /// no-progress watchdog.</summary>
    private const float ProgressEpsilonMs = 0.001f;

    /// <summary>Align must beat Hold by this factor before Auto picks it;
    /// absorbs the roughness of the slew propellant estimate.</summary>
    private const double AlignPreferenceFactor = 0.9;

    /// <summary>Firing with no measurable to-go reduction for this long
    /// cancels with a stall alert. Catches layouts whose axes cannot serve
    /// the demanded direction at the current attitude (the axis suppression
    /// then zeroes every pulse and nothing would ever terminate).</summary>
    private const double NoProgressTimeoutSec = 15.0;

    /// <summary>Real-time TTL for the UI-path cache (gauge verdict,
    /// capability, available propellant); the gauge and the burn editor
    /// query per rendered frame and a full thruster/tank walk per frame is
    /// waste (stock's equivalent checks read one cached global bool).</summary>
    private const long UiCacheTtlMs = 250;

    /// <summary>Rough share of the transverse rotation groups' combined
    /// mass flow that fires during a slew (one side of one axis at a time).
    /// Estimate-only; the Auto decision carries a preference margin.</summary>
    private const double SlewMassFlowFactor = 0.25;

    /// <summary>Below this misalignment the slew cost is treated as zero.</summary>
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

    /// <summary>True when the loaded first burn resolves to RCS and the
    /// vehicle can actually translate. Drives the SetEnum interception and
    /// the per-tick armed preview; the capability probe is fresh.</summary>
    public static bool WouldExecuteRcs(Vehicle vehicle)
        => WouldExecuteRcs(vehicle, out _);

    public static bool WouldExecuteRcs(Vehicle vehicle, out RcsCapabilitySnapshot capability)
    {
        capability = default;
        FlightComputer fc = vehicle.FlightComputer;
        if (fc.Burn == null || !vehicle.IsControllable)
            return false;
        Burn? first = fc.BurnPlan.FirstBurn;
        if (first == null)
            return false;
        RcsBurnOptions? options = null;
        if (RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec))
            options = exec.FindOptions(first.Time.Seconds(), first.DeltaVVlf.Length());
        if (ResolveMode(vehicle, options) != RcsExecutionMode.Rcs)
            return false;
        capability = RcsCapability.Probe(vehicle);
        return capability.HasAnyTranslation;
    }

    private static string _uiCacheVehicleId = string.Empty;
    private static long _uiCacheAtMs = long.MinValue;
    private static bool _uiVerdict;
    private static RcsCapabilitySnapshot _uiCapability;
    private static double _uiAvailableKg;

    /// <summary>Single-entry time-throttled cache for the per-frame UI
    /// paths (gauge button, burn editor); they only ever ask about the
    /// controlled vehicle.</summary>
    private static void RefreshUiCache(Vehicle vehicle)
    {
        long now = Environment.TickCount64;
        if (vehicle.Id == _uiCacheVehicleId && now - _uiCacheAtMs < UiCacheTtlMs)
            return;
        _uiVerdict = WouldExecuteRcs(vehicle, out _uiCapability);
        // The verdict path only probes once resolution passes; the burn
        // editor warnings need real capability either way.
        if (!_uiVerdict)
            _uiCapability = RcsCapability.Probe(vehicle);
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

    /// <summary>Called from the Vehicle.SetEnum prefix for BurnMode values.
    /// Returns true when stock should proceed, false when handled here.</summary>
    public static bool OnBurnModeSetEnum(Vehicle vehicle, FlightComputerBurnMode mode)
    {
        if (RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec) && exec.IsActive)
        {
            // Auto while running toggles off (the button is lit); Manual
            // (engine-shutdown hotkey) also cancels but stock may run too.
            Cancel(vehicle, exec, "user request");
            if (mode == FlightComputerBurnMode.Auto)
            {
                // Stock's Manual leg would restore the navball frame; this
                // path skips stock, so mirror it.
                vehicle.SetNavBallFrame(vehicle.VehicleRegion.GetVehicleReferenceFrame());
                return false;
            }
            return true;
        }
        if (mode == FlightComputerBurnMode.Auto && !WouldExecuteRcs(vehicle))
        {
            // The one log line that answers "why did my click run the
            // engine?": every input to the resolution, at click time.
            if (DebugConfig.RcsTranslation)
                LogResolutionDebug(vehicle);
            return true;
        }
        if (mode == FlightComputerBurnMode.Auto)
        {
            // A half-activated execution must never fall through to the
            // stock engine autopilot: on failure, roll back and swallow the
            // click instead of returning true with exec.IsActive set.
            try
            {
                Activate(vehicle);
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] RCS activation failed for vehicle='{vehicle.Id}': {ex}");
                if (RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? failed))
                    failed.ClearActive();
                RcsCommandChannel.Clear(vehicle.FlightComputer.BurnPlan);
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
        Burn? burn = fc.BurnPlan.FirstBurn;
        if (burn == null || fc.Burn == null)
            return;

        RcsExecution exec = RcsExecRegistry.GetOrCreate(vehicle.Id);
        double timeSec = burn.Time.Seconds();
        double dvMs = burn.DeltaVVlf.Length();
        RcsBurnOptions options = exec.GetOrCreateOptions(timeSec, dvMs);

        exec.Capability = RcsCapability.Probe(vehicle);
        exec.CapabilityProbedAtSec = Universe.GetElapsedSimTime().Seconds();
        if (!exec.Capability.HasAnyTranslation)
            return;

        exec.Estimates = ComputeEstimates(vehicle, fc.Burn, in exec.Capability);
        exec.EstimatesComputedAtSec = exec.CapabilityProbedAtSec;

        (RcsAttitudeStrategy strategy, int axis) = ResolveStrategy(options.Attitude, in exec.Estimates);

        // Refuse an execution that cannot make progress instead of firing
        // nothing forever: Hold needs every demanded axis group present at
        // the current attitude, Align needs slew authority.
        if (strategy == RcsAttitudeStrategy.Hold && !exec.Estimates.HoldFeasible)
        {
            if (exec.Estimates.AlignFeasible && exec.Estimates.AlignAxis >= 0)
            {
                strategy = RcsAttitudeStrategy.Align;
                axis = exec.Estimates.AlignAxis;
            }
            else
            {
                Alert($"RCS burn not engaged: no thruster axis can serve the burn direction on '{vehicle.Id}'.");
                return;
            }
        }

        // Warn-only sufficiency check: the user may refill mid-burn or
        // accept a partial burn, so an underfueled activation proceeds.
        double neededKg = exec.Estimates.RequiredPropellantKg(strategy);
        double availableKg = RcsPropellant.AvailableKg(vehicle);
        if (neededKg > availableKg)
            Alert($"RCS burn may run out of propellant: needs ~{neededKg:F0} kg, {availableKg:F0} kg available.");

        exec.ActiveBurn = burn;
        exec.ActiveBurnTimeSec = timeSec;
        exec.ActiveBurnDvMs = dvMs;
        exec.ResolvedStrategy = strategy;
        exec.ResolvedAxis = axis;
        exec.StallAlerted = false;

        // BurnMode stays Manual the whole time so the stock engine path never
        // engages; the gauge button reads as ON through the PackData patch.
        fc.BurnMode = FlightComputerBurnMode.Manual;
        vehicle.SetNavBallFrame(VehicleReferenceFrame.BurnBody);

        // Translation pulses through an off-CoM layout leave residual
        // torque that only an engaged attitude hold corrects; without it a
        // Hold burn spins the vehicle up. Mirror the stabilization toggle.
        if (fc.AttitudeMode == FlightComputerAttitudeMode.Manual)
        {
            fc.RateHold(vehicle.NavBallData.Frame);
            if (DebugConfig.RcsTranslation)
                DefaultCategory.Log.Debug(
                    $"[AFC] RCS: engaged rate hold for vehicle='{vehicle.Id}' (attitude was Manual).");
        }
        if (strategy == RcsAttitudeStrategy.Align && !CommandAlignAttitude(fc, axis))
        {
            strategy = RcsAttitudeStrategy.Hold;
            axis = -1;
            exec.ResolvedStrategy = strategy;
            exec.ResolvedAxis = axis;
            if (!exec.Estimates.HoldFeasible)
            {
                Alert($"RCS burn not engaged: cannot align or hold for the burn direction on '{vehicle.Id}'.");
                exec.ClearActive();
                return;
            }
        }

        PublishCommand(vehicle, exec);
        DefaultCategory.Log.Info(
            $"[AFC] RCS burn engaged: vehicle='{vehicle.Id}' dv={dvMs:F2}m/s " +
            $"strategy={strategy}{(axis >= 0 ? $" axis={AxisName(axis)}" : string.Empty)}");
        if (DebugConfig.RcsTranslation)
        {
            ref readonly RcsEstimates est = ref exec.Estimates;
            DefaultCategory.Log.Debug(
                $"[AFC] RCS estimates: hold feasible={est.HoldFeasible} " +
                $"{est.HoldPropellantKg:F1}kg/{est.HoldDurationSec:F0}s, " +
                $"align feasible={est.AlignFeasible} axis={AxisName(Math.Max(est.AlignAxis, 0))} " +
                $"{est.AlignPropellantKg + est.AlignSlewPropellantKg:F1}kg/{est.AlignDurationSec:F0}s " +
                $"(slew {est.AlignSlewDurationSec:F0}s), mass={fc.TotalMassPropsBody.Mass:F0}kg");
        }
    }

    public static void Cancel(Vehicle vehicle, RcsExecution exec, string reason)
    {
        // Align commanded the attitude tracker; hand the attitude back the
        // same way a completed burn does. Hold never touched it.
        if (exec.ResolvedStrategy == RcsAttitudeStrategy.Align)
            vehicle.FlightComputer.SetNullRot(VehicleReferenceFrame.BurnBody);
        exec.ClearActive();
        RcsCommandChannel.Clear(vehicle.FlightComputer.BurnPlan);
        DefaultCategory.Log.Info($"[AFC] RCS burn cancelled ({reason}): vehicle='{vehicle.Id}'");
    }

    private static void LogResolutionDebug(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        Burn? first = fc.BurnPlan.FirstBurn;
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

    private static (RcsAttitudeStrategy, int) ResolveStrategy(
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

    /// <summary>Points body axis <paramref name="axisIdx"/> at the burn
    /// vector via the stock attitude tracker. +X/-X map onto the stock
    /// PositiveDv/NegativeDv targets; other axes use a Custom target in the
    /// BurnBody frame (whose +X is the burn direction by construction).
    /// False when the euler round trip cannot express the mapping; the
    /// caller must fall back to Hold so the attitude gate is not left
    /// waiting on a target that was never commanded.</summary>
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

        // Guard against a gimbal-degenerate euler decomposition (the Custom
        // path round-trips through roll-yaw-pitch angles). The game's euler
        // conversion handles the +-90 deg cases exactly, so this is not
        // expected to trip; it exists so a game-side change fails loud.
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

    /// <summary>Runs on the main thread once per solver apply for every
    /// vehicle (Vehicle.UpdateFromTaskResults postfix). Cheap early-outs
    /// keep the no-RCS common case free.</summary>
    public static void Tick(Vehicle vehicle)
    {
        FlightComputer fc = vehicle.FlightComputer;
        bool hasExec = RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec);
        if (!hasExec && fc.Burn == null)
        {
            RcsCommandChannel.Clear(fc.BurnPlan);
            return;
        }

        double nowSec = Universe.GetElapsedSimTime().Seconds();

        if (hasExec && !exec!.ReconciledAfterLoad)
            Reconcile(vehicle, fc, exec);

        if (hasExec && exec!.IsActive)
        {
            TickActive(vehicle, fc, exec, nowSec);
            return;
        }

        // Not active: nothing may stay published (a stale command would
        // suppress engine burns and corrupt their timing marks), but keep
        // per-vehicle capability/estimates fresh while the first burn
        // resolves to RCS. The registry entry is created here so the burn
        // editor has estimates before the first click.
        RcsCommandChannel.Clear(fc.BurnPlan);
        if (fc.Burn != null && WouldExecuteRcs(vehicle, out RcsCapabilitySnapshot capability))
        {
            RcsExecution armed = hasExec ? exec! : RcsExecRegistry.GetOrCreate(vehicle.Id);
            armed.Capability = capability;
            armed.CapabilityProbedAtSec = nowSec;
            RefreshArmedEstimates(vehicle, fc, armed, nowSec);
        }
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
            exec.ClearActive();
            return;
        }
        exec.ActiveBurn = burn;
        if (exec.ResolvedStrategy == RcsAttitudeStrategy.Align)
            CommandAlignAttitude(fc, exec.ResolvedAxis);
        DefaultCategory.Log.Info(
            $"[AFC] RCS burn reattached after load: vehicle='{vehicle.Id}' t={burn.Time.Seconds():F1}s");
    }

    private static void TickActive(Vehicle vehicle, FlightComputer fc, RcsExecution exec, double nowSec)
    {
        BurnTarget? bt = fc.Burn;
        Burn? burn = exec.ActiveBurn;

        // The executor only runs while its burn is the loaded first burn;
        // a reordered or deleted plan cancels rather than firing at the
        // wrong target.
        if (burn == null || bt == null
            || !fc.BurnPlan.TryGetBurn(burn)
            || Math.Abs(bt.ImpulsiveInstant.Seconds() - burn.Time.Seconds()) > BurnIdentityToleranceSec)
        {
            Cancel(vehicle, exec, "burn no longer loaded");
            return;
        }

        if (nowSec - exec.CapabilityProbedAtSec > CapabilityRefreshSec)
        {
            exec.Capability = RcsCapability.Probe(vehicle);
            exec.CapabilityProbedAtSec = nowSec;
        }
        if (!exec.Capability.HasAnyTranslation)
        {
            if (!exec.StallAlerted)
            {
                exec.StallAlerted = true;
                Alert("RCS burn stalled: no thruster propellant.");
            }
            Cancel(vehicle, exec, "no translation authority");
            return;
        }

        float3 togo = bt.DeltaVToGoCci;
        float togoMs = togo.Length();
        float3 impulseBody = float3.Pack(
            double3.Unpack(togo).Transform(vehicle.GetBody2Cci().Inverse()))
            * fc.TotalMassPropsBody.Mass;
        if (float3.Dot(togo, bt.DeltaVTargetCci) <= 0f
            || IsBelowImpulseFloor(impulseBody, in exec.Capability))
        {
            Complete(vehicle, fc, exec, togoMs);
            return;
        }

        // No-progress watchdog: firing was due (past ignition) but the
        // to-go has not moved. Covers directions the current layout cannot
        // serve (all pulses suppressed) without any propellant drain to
        // trigger the capability stall above.
        if (togoMs < exec.WatchdogTogoMs - ProgressEpsilonMs)
        {
            exec.WatchdogTogoMs = togoMs;
            exec.WatchdogAtSec = nowSec;
        }
        else if (nowSec >= bt.IgnitionTime.Seconds()
                 && exec.WatchdogAtSec > 0.0
                 && nowSec - exec.WatchdogAtSec > NoProgressTimeoutSec)
        {
            Alert($"RCS burn stalled: no progress on '{vehicle.Id}' " +
                  $"({togoMs:F2}m/s to go). Check thruster coverage for the burn direction.");
            Cancel(vehicle, exec, "no progress");
            return;
        }
        if (exec.WatchdogAtSec <= 0.0)
        {
            exec.WatchdogTogoMs = togoMs;
            exec.WatchdogAtSec = nowSec;
        }

        if (nowSec - exec.EstimatesComputedAtSec > EstimateRefreshSec)
        {
            exec.Estimates = ComputeEstimates(vehicle, bt, in exec.Capability);
            exec.EstimatesComputedAtSec = nowSec;
        }
        if (exec.ResolvedStrategy == RcsAttitudeStrategy.Align)
            EnsureAlignAttitude(fc, exec);

        if (DebugConfig.RcsTranslation && !exec.FiringLogged
            && nowSec >= bt.IgnitionTime.Seconds())
        {
            exec.FiringLogged = true;
            DefaultCategory.Log.Debug(
                $"[AFC] RCS firing window entered: vehicle='{vehicle.Id}' " +
                $"togo={togoMs:F2}m/s duration est={bt.BurnDuration:F1}s");
        }

        PublishCommand(vehicle, exec);
    }

    private static void Complete(Vehicle vehicle, FlightComputer fc, RcsExecution exec, float residualMs)
    {
        double burnTime = exec.ActiveBurnTimeSec ?? 0.0;
        double burnDv = exec.ActiveBurnDvMs ?? 0.0;
        if (exec.ResolvedStrategy == RcsAttitudeStrategy.Align)
            fc.SetNullRot(VehicleReferenceFrame.BurnBody);
        exec.ClearActive();
        RcsBurnOptions? options = exec.FindOptions(burnTime, burnDv);
        if (options != null)
            exec.Options.Remove(options);
        RcsCommandChannel.Clear(fc.BurnPlan);
        float accumMs = fc.Burn?.DeltaVAccumCci.Length() ?? 0f;
        DefaultCategory.Log.Info(
            $"[AFC] RCS burn complete: vehicle='{vehicle.Id}' " +
            $"accumulated={accumMs:F3}m/s of {burnDv:F2}m/s, residual={residualMs:F3}m/s");
    }

    /// <summary>True when the worker's per-axis suppression would command
    /// nothing for this body-frame residual impulse: every signed-axis
    /// component is below its own group's minimum-impulse floor (or has no
    /// usable group). Must mirror RcsComputeControlPatch.ShapeAxis exactly,
    /// component-wise - comparing the residual's magnitude against the
    /// floors instead deadlocks a burn whose remainder sits just under a
    /// strong axis's floor while a weak axis's floor is smaller.</summary>
    internal static bool IsBelowImpulseFloor(float3 impulseBody, in RcsCapabilitySnapshot cap)
    {
        Span<float> components = stackalloc float[6]
        {
            Math.Max(impulseBody.X, 0f), Math.Max(-impulseBody.X, 0f),
            Math.Max(impulseBody.Y, 0f), Math.Max(-impulseBody.Y, 0f),
            Math.Max(impulseBody.Z, 0f), Math.Max(-impulseBody.Z, 0f),
        };
        for (int i = 0; i < 6; i++)
        {
            RcsAxisGroup g = cap.Get(i);
            if (g.IsUsable && components[i] >= MinImpulseSuppressionFactor * g.MinImpulseNs)
                return false;
        }
        return true;
    }

    private static void EnsureAlignAttitude(FlightComputer fc, RcsExecution exec)
    {
        bool tracking = fc.AttitudeMode == FlightComputerAttitudeMode.Auto
            && fc.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.None;
        if (!tracking && !CommandAlignAttitude(fc, exec.ResolvedAxis))
        {
            exec.ResolvedStrategy = RcsAttitudeStrategy.Hold;
            exec.ResolvedAxis = -1;
        }
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
        if (exec.Estimates.Valid)
            duration = align ? exec.Estimates.AlignDurationSec : exec.Estimates.HoldDurationSec;
        // Ignition stays impulse-centred; the align slew happens before it
        // because the attitude command engages at activation.
        double ignition = bt.ImpulsiveInstant.Seconds() - 0.5 * duration;

        ref readonly RcsCapabilitySnapshot cap = ref exec.Capability;
        RcsCommandChannel.Publish(fc.BurnPlan, new RcsWorkerCommand
        {
            Active = true,
            IgnitionTimeSec = ignition,
            BurnDurationSec = (float)duration,
            RequireAttitude = align,
            MaxPulseSec = MaxPulseSec,
            AxisForcePos = new float3(cap.Ax0.ForceN, cap.Ax2.ForceN, cap.Ax4.ForceN),
            AxisForceNeg = new float3(cap.Ax1.ForceN, cap.Ax3.ForceN, cap.Ax5.ForceN),
            AxisMinImpulsePos = new float3(cap.Ax0.MinImpulseNs, cap.Ax2.MinImpulseNs, cap.Ax4.MinImpulseNs),
            AxisMinImpulseNeg = new float3(cap.Ax1.MinImpulseNs, cap.Ax3.MinImpulseNs, cap.Ax5.MinImpulseNs),
        });
    }

    #endregion

    #region Estimates

    /// <summary>
    /// Propellant and duration for both attitude strategies against the
    /// current to-go vector. Constant-mass approximations: these numbers
    /// steer the Auto decision and the UI, the burn itself is closed-loop.
    /// </summary>
    public static RcsEstimates ComputeEstimates(
        Vehicle vehicle, BurnTarget bt, in RcsCapabilitySnapshot cap)
    {
        RcsEstimates est = default;
        est.AlignAxis = -1;
        float3 togo = bt.DeltaVToGoCci;
        float dv = togo.Length();
        if (dv <= 0f || !cap.HasAnyTranslation)
            return est;
        float mass = vehicle.FlightComputer.TotalMassPropsBody.Mass;
        est.Valid = true;

        // Hold: the current attitude fixes the body direction of the burn
        // vector; axis groups fire in the ratio of its components. The net
        // force is limited by the weakest required axis, propellant follows
        // the duty-cycled mass flows.
        double3 uBody = double3.Unpack(togo)
            .Transform(vehicle.GetBody2Cci().Inverse()).NormalizeOrZero();
        est.HoldFeasible = TryHoldPerformance(in cap, uBody, out double holdForce, out double holdMassFlow);
        if (est.HoldFeasible && holdForce > 0.0)
        {
            est.HoldDurationSec = mass * dv / holdForce;
            est.HoldPropellantKg = est.HoldDurationSec * holdMassFlow;
        }

        // Align: strongest single axis pointed at the burn vector, plus the
        // slew there. Slew propellant is a triangular bang-off-bang profile
        // against the transverse torque authority; a rough estimate by
        // design (the decision carries a preference margin).
        int best = cap.BestAxis();
        if (best >= 0)
        {
            RcsAxisGroup g = cap.Get(best);
            est.AlignAxis = best;
            est.AlignFeasible = true;
            est.AlignDurationSec = mass * dv / g.ForceN;
            est.AlignPropellantKg = est.AlignDurationSec * g.MassFlowKgS;

            double3 axisCci = double3.Unpack(RcsCapabilitySnapshot.AxisDirection(best))
                .Transform(vehicle.GetBody2Cci());
            double3 uCci = double3.Unpack(togo).NormalizeOrZero();
            double theta = MathEx.SafeAcos(double3.Dot(axisCci, uCci));
            FlightComputer fc = vehicle.FlightComputer;
            double alpha = Math.Min(fc.RcsTorqueAuthority.Y, fc.RcsTorqueAuthority.Z);
            if (theta > AlignMinThetaRad)
            {
                if (alpha <= MinSlewAlphaRadS2)
                {
                    est.AlignFeasible = false;
                }
                else
                {
                    double omega = Math.Min(fc.RateLimit, Math.Sqrt(theta * alpha));
                    double thrustOn = 2.0 * omega / alpha;
                    double coast = Math.Max(0.0, theta - omega * omega / alpha) / Math.Max(omega, 1e-9);
                    est.AlignSlewDurationSec = thrustOn + coast;
                    double slewMassFlow = SlewMassFlowFactor
                        * (cap.RotationMassFlowKgS.Y + cap.RotationMassFlowKgS.Z);
                    est.AlignSlewPropellantKg = thrustOn * slewMassFlow;
                }
            }
        }
        return est;
    }

    internal static bool TryHoldPerformance(
        in RcsCapabilitySnapshot cap, double3 uBody, out double netForce, out double massFlow)
    {
        netForce = 0.0;
        massFlow = 0.0;
        if (uBody.IsNearlyZero())
            return false;

        // Required signed groups: +u.X needs the +X group and so on.
        Span<double> weight = stackalloc double[6];
        weight[0] = Math.Max(uBody.X, 0.0);
        weight[1] = Math.Max(-uBody.X, 0.0);
        weight[2] = Math.Max(uBody.Y, 0.0);
        weight[3] = Math.Max(-uBody.Y, 0.0);
        weight[4] = Math.Max(uBody.Z, 0.0);
        weight[5] = Math.Max(-uBody.Z, 0.0);

        double maxNet = double.PositiveInfinity;
        for (int i = 0; i < 6; i++)
        {
            if (weight[i] < 1e-4)
                continue;
            RcsAxisGroup g = cap.Get(i);
            if (!g.IsUsable)
                return false;
            maxNet = Math.Min(maxNet, g.ForceN / weight[i]);
        }
        if (!double.IsFinite(maxNet) || maxNet <= 0.0)
            return false;

        netForce = maxNet;
        for (int i = 0; i < 6; i++)
        {
            if (weight[i] < 1e-4)
                continue;
            RcsAxisGroup g = cap.Get(i);
            massFlow += g.MassFlowKgS * (weight[i] * maxNet / g.ForceN);
        }
        return true;
    }

    public static string AxisName(int idx) => idx switch
    {
        0 => "+X", 1 => "-X", 2 => "+Y", 3 => "-Y", 4 => "+Z", _ => "-Z",
    };

    #endregion
}

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
    /// then zeroes every pulse and nothing would ever terminate). Only
    /// counts while firing is eligible: align slews and the pre-ignition
    /// coast rebase the clock.</summary>
    private const double NoProgressTimeoutSec = 15.0;

    /// <summary>Continuous-slew bound for Align: past this the vehicle is
    /// judged unable to settle into the burn attitude and the burn cancels
    /// with an attitude message (never a thruster-coverage one). Generous
    /// on purpose - ingame slews estimate up to ~35 seconds.</summary>
    private const double AlignTimeoutSec = 120.0;

    /// <summary>The Align tracker is commanded only this close to the burn:
    /// the slew duration estimate scaled by the factor, plus a fixed settle
    /// margin. Any earlier just pays the attitude hold's deadband limit
    /// cycle through the whole coast (a warped multi-hour wait can burn
    /// tonnes on thirsty layouts); any later merely shifts the burn a bit
    /// past its impulse center, which the closed loop absorbs. Internal so
    /// the harness single-sources the window formula.</summary>
    internal const double AlignLeadFactor = 2.0;
    internal const double AlignLeadMarginSec = 15.0;

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
            // stock engine autopilot: on failure the click is swallowed
            // with BurnMode still Manual, the mod-side state cleared, and
            // the navball frame restored. An attitude hold Activate already
            // engaged stays engaged - whether the user wanted stabilization
            // beforehand is unknowable here, and holding is the safe side.
            try
            {
                Activate(vehicle);
            }
            catch (Exception ex)
            {
                // On-screen like the refusal paths inside Activate; a
                // log-only failure would read as the click doing nothing.
                Alert($"RCS burn could not engage on '{vehicle.Id}' (internal error, see log).");
                DefaultCategory.Log.Warning(
                    $"[AFC] RCS activation failed for vehicle='{vehicle.Id}': {ex}");
                if (RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? failed))
                    failed.ClearActive();
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

        // The LP solves before the sufficiency check so the warning uses
        // the pattern's real cost; a zero-torque pattern can need
        // considerably more than the axis-group estimate.
        exec.ResolvedAllocator = options.Allocator;
        if (exec.ResolvedAllocator == RcsAllocator.Lp)
            EnsureLpSolution(vehicle, fc, exec, ImpulseBodyFromTogo(vehicle, fc, fc.Burn),
                Universe.GetElapsedSimTime().Seconds());

        // Warn-only sufficiency check: the user may refill mid-burn or
        // accept a partial burn, so an underfueled activation proceeds.
        double neededKg = exec.LpSecondsPerImpulse != null
            ? exec.LpCostPerImpulse * fc.TotalMassPropsBody.Mass * dvMs
            : exec.Estimates.RequiredPropellantKg(strategy);
        double availableKg = RcsPropellant.AvailableKg(vehicle);
        if (neededKg > availableKg)
            Alert($"RCS burn may run out of propellant: needs ~{neededKg:F0} kg, {availableKg:F0} kg available.");

        exec.ActiveBurn = burn;
        exec.ActiveBurnTimeSec = timeSec;
        exec.ActiveBurnDvMs = dvMs;
        exec.ResolvedStrategy = strategy;
        exec.ResolvedAxis = axis;
        exec.StallAlerted = false;

        exec.BaselineFuel(fc, exec.CapabilityProbedAtSec);

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
        // The Align tracker is commanded only once the ignition lead window
        // opens (EnsureAlignCommanded, called here and per driver tick);
        // until then the coast keeps whatever attitude the user had, so a
        // long wait does not pay the tracking limit cycle.
        if (!EnsureAlignCommanded(fc, exec, exec.CapabilityProbedAtSec))
        {
            Alert($"RCS burn not engaged: cannot align or hold for the burn direction on '{vehicle.Id}'.");
            exec.ClearActive();
            return;
        }
        strategy = exec.ResolvedStrategy;
        axis = exec.ResolvedAxis;

        PublishCommand(vehicle, exec);
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
                $"(slew {est.AlignSlewDurationSec:F0}s), mass={fc.TotalMassPropsBody.Mass:F0}kg");
        }
    }

    public static void Cancel(Vehicle vehicle, RcsExecution exec, string reason)
    {
        RcsFuelSummary fuel = ComputeFuelSummary(vehicle.FlightComputer, exec);
        // Hand the attitude back only when Align actually commanded the
        // tracker; a deferred align cancelled during the coast never
        // touched it (and Hold never does).
        if (exec.AlignCommanded)
            vehicle.FlightComputer.SetNullRot(VehicleReferenceFrame.BurnBody);
        exec.ClearActive();
        RcsCommandChannel.Clear(vehicle.FlightComputer.BurnPlan);
        DefaultCategory.Log.Info($"[AFC] RCS burn cancelled ({reason}): vehicle='{vehicle.Id}'");
        LogFuel(vehicle, in fuel);
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

    /// <summary>Remaining delta-V as a body-frame impulse (CCI to BODY,
    /// scaled by total mass). The worker computes the same quantity from
    /// its navigation snapshot; these are the main-thread sites.</summary>
    private static float3 ImpulseBodyFromTogo(Vehicle vehicle, FlightComputer fc, BurnTarget bt)
        => float3.Pack(double3.Unpack(bt.DeltaVToGoCci).Transform(vehicle.GetBody2Cci().Inverse()))
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
            // No summary is computed on this path; drop the previous
            // execution's numbers so LastFuel reads as "no data".
            exec.LastFuel = default;
            exec.ClearActive();
            return;
        }
        exec.ActiveBurn = burn;
        // The Align tracker is not re-commanded here: the next driver tick
        // commands it once the lead window is open (immediately for a
        // resume near or past ignition), keeping the deferral for loads
        // that land in the coast.

        // Fuel telemetry restarts at the load; the completion line then
        // covers the post-load remainder of the burn.
        exec.BaselineFuel(fc, Universe.GetElapsedSimTime().Seconds());

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

        if (!EnsureAlignCommanded(fc, exec, nowSec))
        {
            Alert($"RCS burn cancelled: cannot align or hold for the burn direction on '{vehicle.Id}'.");
            Cancel(vehicle, exec, "cannot align or hold");
            return;
        }

        float3 togo = bt.DeltaVToGoCci;
        float togoMs = togo.Length();
        float3 impulseBody = ImpulseBodyFromTogo(vehicle, fc, bt);

        // The worker's firing eligibility, mirrored here: an Align burn
        // deliberately fires nothing while pitch/yaw error is outside the
        // align gate, and nothing fires before ignition. Every time-based
        // decision below must distinguish "legitimately not firing yet"
        // from "firing but going nowhere". Both sides share
        // OutsideAlignGate so they cannot disagree. Before the tracker is
        // commanded the error angles are relative to the user's own
        // attitude target, so they do not count as slewing.
        bool slewing = exec.ResolvedStrategy == RcsAttitudeStrategy.Align
            && exec.AlignCommanded
            && OutsideAlignGate(fc);
        bool firingEligible = !slewing && nowSec >= bt.IgnitionTime.Seconds();

        // Solving during the slew is waste: the direction in the body frame
        // rotates every tick (churning the simplex and its logs) and the
        // worker cannot fire the pattern anyway.
        if (exec.ResolvedAllocator == RcsAllocator.Lp && !slewing)
            EnsureLpSolution(vehicle, fc, exec, impulseBody, nowSec);

        AccumulateFuel(exec, fc, bt, impulseBody, slewing, firingEligible);

        // The completion floor must match whichever allocation the worker
        // actually runs: LP pattern floors when a solution is published,
        // the per-axis group floors otherwise (including LP fallback). It
        // is only meaningful while firing is eligible - during a slew the
        // rotating body direction can make a stale LP projection dip below
        // the floors and would falsely complete a burn that never fired.
        bool belowFloor = firingEligible
            && (exec.LpSecondsPerImpulse != null
                ? IsBelowLpFloor(impulseBody, fc, exec)
                : IsBelowImpulseFloor(impulseBody, in exec.Capability));
        if (float3.Dot(togo, bt.DeltaVTargetCci) <= 0f || belowFloor)
        {
            Complete(vehicle, fc, exec, togoMs);
            return;
        }

        // No-progress watchdog: firing was eligible but the to-go has not
        // moved. Covers directions the current layout cannot serve (all
        // pulses suppressed) without any propellant drain to trigger the
        // capability stall above. While not yet eligible the clock rebases
        // every tick, so an align slew or a pre-ignition coast never counts
        // as "no progress".
        if (!firingEligible || togoMs < exec.WatchdogTogoMs - ProgressEpsilonMs)
        {
            exec.WatchdogTogoMs = togoMs;
            exec.WatchdogAtSec = nowSec;
        }
        else if (nowSec - exec.WatchdogAtSec > NoProgressTimeoutSec)
        {
            Alert($"RCS burn stalled: no progress on '{vehicle.Id}' " +
                  $"({togoMs:F2}m/s to go). Check thruster coverage for the burn direction.");
            Cancel(vehicle, exec, "no progress");
            return;
        }

        // Terminal bound for the slew itself: a vehicle that cannot settle
        // into the deadband (marginal torque authority, oscillation) must
        // not hang armed forever, and the message must not blame thruster
        // coverage.
        if (slewing)
        {
            if (exec.SlewingSinceSec <= 0.0)
                exec.SlewingSinceSec = nowSec;
            else if (nowSec - exec.SlewingSinceSec > AlignTimeoutSec)
            {
                Alert($"RCS burn cancelled: '{vehicle.Id}' cannot reach the burn attitude " +
                      $"(still slewing after {AlignTimeoutSec:F0}s).");
                Cancel(vehicle, exec, "cannot reach burn attitude");
                return;
            }
        }
        else
        {
            exec.SlewingSinceSec = 0.0;
        }

        if (nowSec - exec.EstimatesComputedAtSec > EstimateRefreshSec)
        {
            exec.Estimates = ComputeEstimates(vehicle, bt, in exec.Capability);
            exec.EstimatesComputedAtSec = nowSec;
        }

        if (DebugConfig.RcsTranslation && !exec.FiringLogged
            && nowSec >= bt.IgnitionTime.Seconds())
        {
            exec.FiringLogged = true;
            DefaultCategory.Log.Debug(
                $"[AFC] RCS firing window entered: vehicle='{vehicle.Id}' " +
                $"togo={togoMs:F2}m/s duration est={bt.BurnDuration:F1}s " +
                $"allocator={exec.ResolvedAllocator}");
            if (exec.LpSecondsPerImpulse == null)
            {
                // Group mode fires per signed axis; the split shows which
                // groups carry the burn and at what force.
                ref readonly RcsCapabilitySnapshot cap = ref exec.Capability;
                DefaultCategory.Log.Debug(
                    $"[AFC]   axis impulse split: X={impulseBody.X / 1000f:F1}kNs " +
                    $"(F {(impulseBody.X >= 0f ? cap.Ax0.ForceN : cap.Ax1.ForceN) / 1000f:F1}kN), " +
                    $"Y={impulseBody.Y / 1000f:F1}kNs " +
                    $"(F {(impulseBody.Y >= 0f ? cap.Ax2.ForceN : cap.Ax3.ForceN) / 1000f:F1}kN), " +
                    $"Z={impulseBody.Z / 1000f:F1}kNs " +
                    $"(F {(impulseBody.Z >= 0f ? cap.Ax4.ForceN : cap.Ax5.ForceN) / 1000f:F1}kN)");
            }
            else
            {
                DefaultCategory.Log.Debug(
                    $"[AFC]   LP pattern throughput ~{exec.LpImpulseCapNs / MaxPulseSec / 1000f:F1}kN " +
                    $"({exec.LpCostPerImpulse * 1e6:F1}mg per Ns)");
            }
        }

        PublishCommand(vehicle, exec);
    }

    private static void Complete(Vehicle vehicle, FlightComputer fc, RcsExecution exec, float residualMs)
    {
        double burnTime = exec.ActiveBurnTimeSec ?? 0.0;
        double burnDv = exec.ActiveBurnDvMs ?? 0.0;
        Burn? completedBurn = exec.ActiveBurn;
        RcsFuelSummary fuel = ComputeFuelSummary(fc, exec);
        if (exec.AlignCommanded)
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
        LogFuel(vehicle, in fuel);

        // Raised last: a subscriber may remove the burn from the plan
        // (AutoRemoveFinishedBurns), which must not race our own cleanup.
        if (completedBurn != null)
            RcsBurnCompletions.Raise(vehicle, completedBurn);
    }

    /// <summary>True while the pitch/yaw error is outside the Align firing
    /// gate. Deliberately wider than the stock burn gate's plain
    /// AngleDeadband: the stock phase plane
    /// (FlightComputer.ComputeRcsTrackAxis) stops actively correcting
    /// inside its 0.5 * AngleDeadband + AngleTurnaround corridor and lets
    /// the vehicle coast through zero at a sub-deadband rate, so any gate
    /// tighter than that corridor can wait minutes on pure drift. Stock
    /// never faces this because engine burns hand pitch/yaw to TVC, which
    /// bypasses the RCS angle gate entirely. Misalignment inside the
    /// corridor is a small cosine thrust loss the closed loop absorbs.
    /// Shared by the worker's RequireAttitude gate and the driver's
    /// slewing mirror so the two cannot drift apart.</summary>
    internal static bool OutsideAlignGate(FlightComputer fc)
    {
        float gateY = Math.Max(fc.AngleDeadband, 0.5f * fc.AngleDeadband + fc.AngleTurnaround.Y);
        float gateZ = Math.Max(fc.AngleDeadband, 0.5f * fc.AngleDeadband + fc.AngleTurnaround.Z);
        return Math.Abs(fc.ErrorAngles.Y) > gateY || Math.Abs(fc.ErrorAngles.Z) > gateZ;
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

    #region Fuel telemetry

    /// <summary>Per-driver-tick fuel bookkeeping. The slew bucket takes the
    /// measured mass delta while the Align slew holds firing back, the
    /// coast bucket the delta before the firing window opens (both all
    /// attitude by construction); the translation bucket attributes the
    /// delta-V the game accounted this tick at the active allocator's model
    /// cost. An exact physical split does not exist (one thruster pulse can
    /// serve translation and attitude at once, and RocketCore.UpdateState
    /// REPLACES a re-commanded pulse's remaining time, so summing worker
    /// commands would overcount overlapped pulses); the unattributed
    /// remainder is reported as attitude.</summary>
    private static void AccumulateFuel(
        RcsExecution exec, FlightComputer fc, BurnTarget bt,
        float3 impulseBody, bool slewing, bool firingEligible)
    {
        if (exec.StartMassKg <= 0.0)
            return;
        double massNow = fc.TotalMassPropsBody.Mass;
        double burnedTick = Math.Max(0.0, exec.LastTickMassKg - massNow);
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
                        double3.Unpack(impulseBody).NormalizeOrZero());
                exec.TranslationPropellantKg += deliveredNs * costPerNs;
            }
        }
        // Advanced every tick so attitude-driven delta-V outside the firing
        // window is never attributed to translation later.
        exec.LastAccumCci = accumNow;
    }

    /// <summary>Model cost of group-allocated translation along
    /// <paramref name="uBody"/>, kg per newton-second of net impulse: each
    /// demanded signed axis contributes its direction weight times the
    /// group's massflow per force (the L1 penalty falls out of the weights
    /// summing above 1 for off-axis directions). Unusable axes are skipped,
    /// matching the worker's suppression of those components.</summary>
    private static double GroupCostPerNs(in RcsCapabilitySnapshot cap, double3 uBody)
    {
        Span<double> weight = stackalloc double[6]
        {
            Math.Max(uBody.X, 0.0), Math.Max(-uBody.X, 0.0),
            Math.Max(uBody.Y, 0.0), Math.Max(-uBody.Y, 0.0),
            Math.Max(uBody.Z, 0.0), Math.Max(-uBody.Z, 0.0),
        };
        double cost = 0.0;
        for (int i = 0; i < 6; i++)
        {
            // Negligible off-axis components, same threshold as
            // TryHoldPerformance's feasibility walk.
            if (weight[i] < 1e-4)
                continue;
            RcsAxisGroup g = cap.Get(i);
            if (g.IsUsable)
                cost += weight[i] * g.MassFlowKgS / g.ForceN;
        }
        return cost;
    }

    /// <summary>Snapshot the breakdown before ClearActive wipes the
    /// accumulators; also stores it as <see cref="RcsExecution.LastFuel"/>.
    /// Invalid (and later unlogged) when no baseline exists, e.g. an
    /// execution cancelled right after a save load reattach failure.</summary>
    private static RcsFuelSummary ComputeFuelSummary(FlightComputer fc, RcsExecution exec)
    {
        if (exec.StartMassKg <= 0.0)
        {
            // Overwrite here too, so LastFuel always reflects the most
            // recent finish and a baseline-less one reads as "no data"
            // instead of the previous execution's numbers.
            exec.LastFuel = default;
            return default;
        }
        double totalKg = exec.StartMassKg - fc.TotalMassPropsBody.Mass;

        double dvMs = 0.0;
        double veMs = 0.0;
        double angleDeg = 0.0;
        BurnTarget? bt = fc.Burn;
        // The loaded BurnTarget can already belong to a different burn on
        // the cancel paths (plan reordered/deleted); its delta-V numbers
        // would be someone else's.
        bool btMatches = bt != null && exec.ActiveBurnTimeSec.HasValue
            && Math.Abs(bt.ImpulsiveInstant.Seconds() - exec.ActiveBurnTimeSec.Value)
               <= BurnIdentityToleranceSec;
        if (btMatches)
        {
            double3 accum = double3.Unpack(bt!.DeltaVAccumCci);
            double3 target = double3.Unpack(bt.DeltaVTargetCci);
            // The ve pairs this window's delta-V with this window's
            // propellant; after a mid-burn save load both restart at the
            // baseline, while the angle stays a whole-burn statement.
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
            ElapsedSec = Universe.GetElapsedSimTime().Seconds() - exec.EngagedAtSec,
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

    #region LP allocation

    /// <summary>Keeps the LP solution fresh for the current burn direction:
    /// rebuilds the wrench table at the capability cadence or when the
    /// FlightComputer replaced its VehicleConfig (staging can swap the
    /// thruster list without changing its count, so identity is the
    /// trigger), and re-solves at the estimate cadence or when the demanded
    /// direction drifted more than ~2.5 degrees. On an infeasible
    /// constraint set the solution is dropped and the worker falls back to
    /// the axis-group path; the cadence still applies so a persistently
    /// infeasible layout is retried, not re-solved every tick.</summary>
    private static void EnsureLpSolution(
        Vehicle vehicle, FlightComputer fc, RcsExecution exec, float3 impulseBody, double nowSec)
    {
        List<ThrusterController> thrusters = fc.VehicleConfig.Thrusters;
        exec.Wrench ??= new RcsWrenchTable();
        if (nowSec - exec.WrenchBuiltAtSec > CapabilityRefreshSec
            || !exec.Wrench.Matches(thrusters))
        {
            exec.Wrench.Build(vehicle, fc);
            exec.WrenchBuiltAtSec = nowSec;
            exec.LpSolvedAtSec = double.NegativeInfinity;
        }

        float3 dir = impulseBody.NormalizeOrZero();
        if (dir.IsExactlyZero())
            return;
        // LpDirBody records the last ATTEMPTED direction (also on failure),
        // so an infeasible layout honors the cadence instead of re-running
        // the simplex every driver tick.
        bool drifted = float3.Dot(dir, exec.LpDirBody) < 0.999f;
        if (!drifted && nowSec - exec.LpSolvedAtSec <= EstimateRefreshSec)
            return;
        exec.LpSolvedAtSec = nowSec;
        exec.LpDirBody = dir;

        RcsWrenchTable w = exec.Wrench;
        if (w.UsableCount == 0)
        {
            DropLpSolution(exec, "no usable thrusters");
            return;
        }

        double[] columns = new double[w.UsableCount * 6];
        double[] cost = new double[w.UsableCount];
        int[] map = new int[w.UsableCount];
        int k = 0;
        for (int i = 0; i < w.Count; i++)
        {
            if (!w.Usable[i])
                continue;
            columns[k * 6 + 0] = w.ForceBody[i].X;
            columns[k * 6 + 1] = w.ForceBody[i].Y;
            columns[k * 6 + 2] = w.ForceBody[i].Z;
            columns[k * 6 + 3] = w.TorqueBody[i].X;
            columns[k * 6 + 4] = w.TorqueBody[i].Y;
            columns[k * 6 + 5] = w.TorqueBody[i].Z;
            cost[k] = w.MassFlow[i];
            map[k] = i;
            k++;
        }
        double[] rhs = { dir.X, dir.Y, dir.Z, 0.0, 0.0, 0.0 };

        double[]? x = RcsLpSolver.Solve(6, w.UsableCount, columns, cost, rhs);
        if (x == null)
        {
            DropLpSolution(exec,
                "the zero-torque force constraint is infeasible for this layout/direction");
            return;
        }

        float[] full = new float[thrusters.Count];
        float maxX = 0f;
        double costPerImpulse = 0.0;
        int support = 0;
        for (int j = 0; j < w.UsableCount; j++)
        {
            float xi = (float)x[j];
            if (xi <= 0f)
                continue;
            full[map[j]] = xi;
            maxX = Math.Max(maxX, xi);
            costPerImpulse += cost[j] * x[j];
            support++;
        }
        if (maxX <= 0f)
        {
            DropLpSolution(exec, "the solver returned an empty firing pattern");
            return;
        }

        exec.LpSecondsPerImpulse = full;
        exec.LpImpulseCapNs = MaxPulseSec / maxX;
        exec.LpCostPerImpulse = costPerImpulse;

        // Log the pattern only when its support set changes: cadence
        // re-solves with an unchanged set differ just in the last digits
        // and would flood the log over a long burn.
        if (DebugConfig.RcsTranslation)
        {
            int signature = support;
            for (int i = 0; i < full.Length; i++)
            {
                if (full[i] > 0f)
                    signature = signature * 31 + i;
            }
            if (signature != exec.LpLoggedSupportSignature)
            {
                exec.LpLoggedSupportSignature = signature;
                DefaultCategory.Log.Debug(
                    $"[AFC] RCS LP solved: vehicle='{vehicle.Id}' {support}/{w.UsableCount} thrusters, " +
                    $"{costPerImpulse * 1e6:F2}mg per Ns, cap {exec.LpImpulseCapNs:F0}Ns/tick");
                for (int i = 0; i < full.Length; i++)
                {
                    if (full[i] <= 0f)
                        continue;
                    float3 f = w.ForceBody[i];
                    DefaultCategory.Log.Debug(
                        $"[AFC]   LP thruster {i}: duty={full[i] / maxX * 100f:F0}% " +
                        $"F=({f.X / 1000f:F1},{f.Y / 1000f:F1},{f.Z / 1000f:F1})kN " +
                        $"|tau|={w.TorqueBody[i].Length() / 1000f:F1}kNm");
                }
            }
        }
    }

    private static void DropLpSolution(RcsExecution exec, string reason)
    {
        exec.LpSecondsPerImpulse = null;
        exec.LpCostPerImpulse = 0.0;
        if (!exec.LpFallbackLogged)
        {
            exec.LpFallbackLogged = true;
            DefaultCategory.Log.Warning(
                $"[AFC] RCS LP allocator falling back to axis groups: {reason}.");
        }
    }

    /// <summary>LP-mode completion floor: the pattern scales linearly with
    /// the demanded impulse, so once every participating thruster's pulse
    /// would fall below its own minimum-pulse floor the worker fires
    /// nothing and the burn is as done as this pattern can make it.</summary>
    private static bool IsBelowLpFloor(float3 impulseBody, FlightComputer fc, RcsExecution exec)
    {
        float[] x = exec.LpSecondsPerImpulse!;
        List<ThrusterController> thrusters = fc.VehicleConfig.Thrusters;
        if (x.Length != thrusters.Count)
            return false;
        float j = float3.Dot(impulseBody, exec.LpDirBody);
        if (j <= 0f)
            return false;
        for (int i = 0; i < x.Length; i++)
        {
            if (x[i] <= 0f)
                continue;
            if (x[i] * j >= MinImpulseSuppressionFactor * thrusters[i].MinimumPulseTime)
                return false;
        }
        return true;
    }

    #endregion

    /// <summary>Anchored on ImpulsiveInstant, not IgnitionTime: at
    /// activation (before the worker mirrors the RCS timing) IgnitionTime
    /// still carries stock UpdateBurnTarget's Manual-mode value, which for
    /// a vehicle with active engines divides the engine burn duration by
    /// the clamped manual throttle and can sit wildly early, silently
    /// defeating the deferral. ImpulsiveInstant is the finite primitive
    /// the RCS timing itself is derived from; the settle margin covers the
    /// half-duration offset to the real firing start.</summary>
    private static bool AlignCommandDue(RcsExecution exec, BurnTarget bt, double nowSec)
        => nowSec >= bt.ImpulsiveInstant.Seconds()
           - (AlignLeadFactor * exec.Estimates.AlignSlewDurationSec + AlignLeadMarginSec);

    /// <summary>Commands the Align tracker once the ignition lead window
    /// opens, and re-commands it if something else took the tracker over
    /// afterwards. On a failed command the execution degrades to Hold;
    /// returns false when Hold cannot serve the burn direction either and
    /// the caller must abort.</summary>
    private static bool EnsureAlignCommanded(FlightComputer fc, RcsExecution exec, double nowSec)
    {
        if (exec.ResolvedStrategy != RcsAttitudeStrategy.Align)
            return true;
        BurnTarget? bt = fc.Burn;
        if (!exec.AlignCommanded)
        {
            if (bt == null || !AlignCommandDue(exec, bt, nowSec))
                return true;
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
            // The LP pattern's throughput is capped by its busiest thruster
            // (a sparse vertex fires far fewer jets than the axis groups),
            // so the group-model estimate would badly understate the burn
            // time the countdown and warp mark show.
            double totalImpulse = bt.DeltaVTargetCci.Length() * fc.TotalMassPropsBody.Mass;
            duration = totalImpulse * MaxPulseSec / exec.LpImpulseCapNs;
        }
        else if (exec.Estimates.Valid)
        {
            duration = align ? exec.Estimates.AlignDurationSec : exec.Estimates.HoldDurationSec;
        }
        // Ignition stays impulse-centred and deliberately carries no slew
        // lead: past ignition the worker still waits on the attitude gate,
        // and the driver's watchdog rebases while slewing. Folding the
        // (rough) slew estimate into the warp-to-burn mark is a separate
        // UX decision, not needed for correctness.
        double ignition = bt.ImpulsiveInstant.Seconds() - 0.5 * duration;

        ref readonly RcsCapabilitySnapshot cap = ref exec.Capability;
        RcsCommandChannel.Publish(fc.BurnPlan, new RcsWorkerCommand
        {
            Active = true,
            IgnitionTimeSec = ignition,
            RequireAttitude = align,
            MaxPulseSec = MaxPulseSec,
            AxisForcePos = new float3(cap.Ax0.ForceN, cap.Ax2.ForceN, cap.Ax4.ForceN),
            AxisForceNeg = new float3(cap.Ax1.ForceN, cap.Ax3.ForceN, cap.Ax5.ForceN),
            AxisMinImpulsePos = new float3(cap.Ax0.MinImpulseNs, cap.Ax2.MinImpulseNs, cap.Ax4.MinImpulseNs),
            AxisMinImpulseNeg = new float3(cap.Ax1.MinImpulseNs, cap.Ax3.MinImpulseNs, cap.Ax5.MinImpulseNs),
            LpSecondsPerImpulse = exec.LpSecondsPerImpulse,
            LpDirBody = exec.LpDirBody,
            LpImpulseCapNs = exec.LpImpulseCapNs,
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

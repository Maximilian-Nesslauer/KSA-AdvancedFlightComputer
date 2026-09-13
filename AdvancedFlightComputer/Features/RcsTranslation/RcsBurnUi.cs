using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

// The docked burn editor uses RcsGaugePanel. Rendezvous and detached editors use this ImGui block.
internal static class RcsBurnUi
{
    internal static void DrawBlock(Burn burn, Vehicle vehicle, FlightComputer flightComputer)
    {
        if (burn.ParentDepartureBurn)
            return;
        if (vehicle.Parts.Modules.Get<ThrusterController>().Length == 0)
            return;

        double timeSec = burn.Time.Seconds();
        double dvMs = burn.DeltaVVlf.Length();
        RcsExecRegistry.TryGet(vehicle.Id, out RcsExecution? exec);
        RcsBurnOptions? options = exec?.FindOptions(timeSec, dvMs);
        bool isActiveBurn = exec != null && exec.IsActive
            && options != null
            && options.Matches(exec.ActiveBurnTimeSec!.Value, exec.ActiveBurnDvMs!.Value);

        ImGui.Separator();
        if (exec?.Faulted == true)
        {
            DrawWarning(exec.CleanupPending ? "RCS stopped. Control cleanup failed." : "RCS stopped after an internal error. See log.");
            if (exec.CleanupPending)
            {
                if (ConsoleWidgets.DangerButton("RETRY RCS CLEANUP".AsSpan()))
                    RcsExecutor.RequestCancel(exec, "retry fault cleanup");
                return;
            }
        }
        RcsExecutionMode mode = options?.Mode ?? RcsExecutionMode.Default;
        RcsExecutionMode resolved = RcsExecutor.ResolveMode(vehicle, options);

        DrawSelectors(vehicle, timeSec, dvMs, options, mode, resolved, isActiveBurn);
        if (resolved == RcsExecutionMode.Rcs || isActiveBurn)
            DrawEstimatesAndStatus(burn, vehicle, flightComputer, exec,
                options?.Attitude ?? RcsAttitudeStrategy.Auto, isActiveBurn);
    }

    private static void DrawSelectors(Vehicle vehicle, double timeSec, double dvMs,
        RcsBurnOptions? options, RcsExecutionMode mode, RcsExecutionMode resolved, bool isActiveBurn)
    {
        // Freeze selectors while the executor holds the resolved strategy.
        Span<char> id = stackalloc char[64];
        using (new ImGuiDisabledScope(isActiveBurn))
        {
            ConsoleWidgets.BeginRow("EXECUTION".AsSpan());
            string modeLabel = mode == RcsExecutionMode.Default
                ? (resolved == RcsExecutionMode.Rcs ? "DEFAULT (RCS)" : "DEFAULT (ENGINE)")
                : ModeLabel(mode);
            bool modeClicked = ConsoleWidgets.Button(
                modeLabel.AsSpan(), ControlId(id, "rcsmode", timeSec),
                new float2(ConsoleWidgets.RowControlWidth, ConsoleWidgets.ButtonHeight));
            ConsoleWidgets.EndRow();
            if (modeClicked && !isActiveBurn)
            {
                RcsExecution target = RcsExecRegistry.GetOrCreate(vehicle.Id);
                RcsBurnOptions o = target.GetOrCreateOptions(timeSec, dvMs);
                CycleMode(o);
            }

            if (resolved == RcsExecutionMode.Rcs || isActiveBurn)
            {
                RcsAttitudeStrategy attitude = options?.Attitude ?? RcsAttitudeStrategy.Auto;
                ConsoleWidgets.BeginRow("ATTITUDE".AsSpan());
                bool attClicked = ConsoleWidgets.Button(
                    AttitudeLabel(attitude).AsSpan(), ControlId(id, "rcsatt", timeSec),
                    new float2(ConsoleWidgets.RowControlWidth, ConsoleWidgets.ButtonHeight));
                ConsoleWidgets.EndRow();
                if (attClicked && !isActiveBurn)
                {
                    RcsExecution target = RcsExecRegistry.GetOrCreate(vehicle.Id);
                    RcsBurnOptions o = target.GetOrCreateOptions(timeSec, dvMs);
                    CycleAttitude(o);
                }

                RcsAllocator allocator = options?.Allocator ?? RcsAllocator.Groups;
                ConsoleWidgets.BeginRow("ALLOCATOR".AsSpan());
                bool allocClicked = ConsoleWidgets.Button(
                    AllocatorLabel(allocator).AsSpan(), ControlId(id, "rcsalloc", timeSec),
                    new float2(ConsoleWidgets.RowControlWidth, ConsoleWidgets.ButtonHeight));
                ConsoleWidgets.EndRow();
                if (allocClicked && !isActiveBurn)
                {
                    RcsExecution target = RcsExecRegistry.GetOrCreate(vehicle.Id);
                    RcsBurnOptions o = target.GetOrCreateOptions(timeSec, dvMs);
                    CycleAllocator(o);
                }
            }
        }

    }

    private static void DrawEstimatesAndStatus(
        Burn burn, Vehicle vehicle, FlightComputer flightComputer, RcsExecution? exec,
        RcsAttitudeStrategy attitude, bool isActiveBurn)
    {
        if (isActiveBurn && exec != null)
        {
            BurnTarget? bt = flightComputer.Burn;
            string phase = "waiting";
            if (bt != null)
            {
                double toIgnition = (bt.IgnitionTime - Universe.GetElapsedTime()).Seconds();
                if (toIgnition > 0.0)
                    phase = $"waiting T-{toIgnition:F0}s";
                else if (exec.ResolvedStrategy == RcsAttitudeStrategy.Align
                         && RcsExecutor.OutsideAlignGate(flightComputer))
                    phase = "aligning";
                else
                    phase = "firing";
            }
            string strategy = exec.ResolvedStrategy == RcsAttitudeStrategy.Align
                ? $"Align {RcsExecutor.AxisName(exec.ResolvedAxis)}"
                : "Hold";
            string allocator = exec.ResolvedAllocator == RcsAllocator.Lp
                ? (exec.LpSecondsPerImpulse != null ? "LP" : "LP->Groups")
                : "Groups";
            ConsoleWidgets.Readout("RCS STATUS".AsSpan(),
                $"{phase} ({strategy}, {allocator})".AsSpan());
            if (bt != null)
                ConsoleWidgets.Readout("TO GO".AsSpan(),
                    $"{bt.DeltaVToGoCci.Length():F2} m/s".AsSpan());
            if (ConsoleWidgets.DangerButton("CANCEL RCS BURN".AsSpan()))
            {
                // An explicit stop must not restore stock Auto.
                exec.ForcedBurnManual = false;
                RcsExecutor.RequestCancel(exec, "user request");
            }
            return;
        }

        // Show unavailable translation even when no estimates exist.
        RcsCapabilitySnapshot cap = RcsExecutor.ProbeCached(vehicle);
        if (!cap.HasAnyTranslation)
        {
            DrawWarning("RCS unavailable: no active thruster with propellant can translate");
            return;
        }

        if (!RcsBurnPreview.TryGetEstimates(burn, vehicle, flightComputer, exec,
                out RcsEstimates est, out bool currentVehicle))
            return;
        if (currentVehicle)
            ConsoleWidgets.Readout("ESTIMATE BASIS".AsSpan(), "Current vehicle".AsSpan());
        if (est.HoldFeasible)
            ConsoleWidgets.Readout("HOLD EST.".AsSpan(),
                $"{est.HoldPropellantKg:F1} kg, {est.HoldDurationSec:F0} s".AsSpan());
        if (est.AlignFeasible)
            ConsoleWidgets.Readout("ALIGN EST.".AsSpan(),
                ($"{est.AlignTotalPropellantKg:F1} kg, " +
                 $"{est.AlignDurationSec:F0} s ({RcsExecutor.AxisName(est.AlignAxis)})").AsSpan());

        double neededKg = est.RequiredPropellantKg(attitude);
        double availableKg = RcsExecutor.AvailablePropellantCached(vehicle);
        if (neededKg > availableKg)
            DrawWarning($"Propellant short: needs ~{neededKg:F0} kg, {availableKg:F0} kg available");
    }

    internal static bool HasEstimatesFor(double timeSec, BurnTarget? loaded, RcsExecution? exec)
        // FlightComputer loads only the first executable burn. A later editor must not show its estimates.
        => exec != null && exec.Estimates.Valid && loaded != null
            && Math.Abs(loaded.ImpulsiveInstant.Seconds() - timeSec) <= RcsExecutor.BurnIdentityToleranceSec;

    internal static void CycleMode(RcsBurnOptions options)
        => options.Mode = options.Mode == RcsExecutionMode.Rcs ? RcsExecutionMode.Default : RcsExecutionMode.Rcs;

    internal static void CycleAttitude(RcsBurnOptions options)
        => options.Attitude = options.Attitude switch
        {
            RcsAttitudeStrategy.Auto => RcsAttitudeStrategy.Hold,
            RcsAttitudeStrategy.Hold => RcsAttitudeStrategy.Align,
            _ => RcsAttitudeStrategy.Auto,
        };

    internal static void CycleAllocator(RcsBurnOptions options)
        => options.Allocator = options.Allocator == RcsAllocator.Groups ? RcsAllocator.Lp : RcsAllocator.Groups;

    internal static string ModeLabel(RcsExecutionMode mode) => mode switch
    {
        RcsExecutionMode.Default => "DEFAULT",
        RcsExecutionMode.Engine => "ENGINE",
        RcsExecutionMode.Rcs => "RCS",
        _ => mode.ToString(),
    };

    internal static string AttitudeLabel(RcsAttitudeStrategy attitude) => attitude switch
    {
        RcsAttitudeStrategy.Auto => "AUTO",
        RcsAttitudeStrategy.Hold => "HOLD",
        RcsAttitudeStrategy.Align => "ALIGN",
        _ => attitude.ToString(),
    };

    internal static string AllocatorLabel(RcsAllocator allocator) => allocator switch
    {
        RcsAllocator.Groups => "GROUPS",
        RcsAllocator.Lp => "LP",
        _ => allocator.ToString(),
    };

    private static ReadOnlySpan<char> ControlId(Span<char> buffer, string prefix, double timeSec)
    {
        prefix.AsSpan().CopyTo(buffer);
        timeSec.TryFormat(buffer[prefix.Length..], out int written, "R");
        return buffer[..(prefix.Length + written)];
    }

    private static void DrawWarning(string text) => ConsoleUi.DangerWrapped(text);
}

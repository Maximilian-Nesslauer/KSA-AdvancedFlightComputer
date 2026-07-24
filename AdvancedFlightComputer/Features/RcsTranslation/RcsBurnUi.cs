using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// The per-burn RCS block (execution/attitude/allocator selectors, strategy
/// estimates, live status + cancel), drawn inside whatever ImGui window the
/// caller has open: the stock rendezvous infobox
/// (<see cref="RcsBurnWindowUi"/>), and the detached-canvas fallback in
/// <see cref="RcsBurnCanvasUi"/> where ImGauge cannot follow the viewport.
/// The docked flight burn editor draws the gauge-styled
/// <see cref="RcsGaugePanel"/> instead.
/// </summary>
internal static class RcsBurnUi
{
    internal static void DrawBlock(Burn burn, Vehicle vehicle, FlightComputer flightComputer)
    {
        if (burn.ParentEjectBurn)
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
        RcsExecutionMode mode = options?.Mode ?? RcsExecutionMode.Default;
        RcsExecutionMode resolved = RcsExecutor.ResolveMode(vehicle, options);

        // The selectors freeze during execution; a running burn must not
        // re-resolve mid-flight.
        using (new ImGuiDisabledScope(isActiveBurn))
        {
            ImGui.Text("Execution"u8);
            ImGui.SameLine(120f);
            string modeLabel = mode == RcsExecutionMode.Default
                ? $"Default ({resolved})##rcsmode{timeSec:R}"
                : $"{mode}##rcsmode{timeSec:R}";
            if (ImGui.Button(modeLabel, (float2?)null) && !isActiveBurn)
            {
                RcsExecution target = RcsExecRegistry.GetOrCreate(vehicle.Id);
                RcsBurnOptions o = target.GetOrCreateOptions(timeSec, dvMs);
                // Default <-> RCS only. Explicit Engine is never usefully
                // different from Default: Default already picks the engine
                // when an active, fueled one exists, and forcing Engine
                // without one cannot fire. Any stale Engine value (e.g. from
                // an older save) folds back to RCS on the next click.
                o.Mode = o.Mode == RcsExecutionMode.Rcs
                    ? RcsExecutionMode.Default
                    : RcsExecutionMode.Rcs;
            }

            if (resolved == RcsExecutionMode.Rcs || isActiveBurn)
            {
                RcsAttitudeStrategy attitude = options?.Attitude ?? RcsAttitudeStrategy.Auto;
                ImGui.Text("Attitude"u8);
                ImGui.SameLine(120f);
                if (ImGui.Button($"{attitude}##rcsatt{timeSec:R}", (float2?)null)
                    && !isActiveBurn)
                {
                    RcsExecution target = RcsExecRegistry.GetOrCreate(vehicle.Id);
                    RcsBurnOptions o = target.GetOrCreateOptions(timeSec, dvMs);
                    o.Attitude = o.Attitude switch
                    {
                        RcsAttitudeStrategy.Auto => RcsAttitudeStrategy.Hold,
                        RcsAttitudeStrategy.Hold => RcsAttitudeStrategy.Align,
                        _ => RcsAttitudeStrategy.Auto,
                    };
                }

                // Per-burn allocator choice; see RcsAllocator for the tradeoff.
                RcsAllocator allocator = options?.Allocator ?? RcsAllocator.Groups;
                ImGui.Text("Allocator"u8);
                ImGui.SameLine(120f);
                if (ImGui.Button($"{allocator}##rcsalloc{timeSec:R}", (float2?)null)
                    && !isActiveBurn)
                {
                    RcsExecution target = RcsExecRegistry.GetOrCreate(vehicle.Id);
                    RcsBurnOptions o = target.GetOrCreateOptions(timeSec, dvMs);
                    o.Allocator = o.Allocator == RcsAllocator.Groups
                        ? RcsAllocator.Lp
                        : RcsAllocator.Groups;
                }
            }
        }

        if (resolved == RcsExecutionMode.Rcs || isActiveBurn)
            DrawEstimatesAndStatus(burn, vehicle, flightComputer, exec,
                options?.Attitude ?? RcsAttitudeStrategy.Auto, isActiveBurn);
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
                double toIgnition = bt.IgnitionTime.Seconds() - Universe.GetElapsedSimTime().Seconds();
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
            ImGuiHelper.DrawTextWidget("RCS status"u8, $"{phase} ({strategy}, {allocator})");
            if (bt != null)
                ImGuiHelper.DrawTextWidget("To go"u8, $"{bt.DeltaVToGoCci.Length():F2} m/s");
            if (ImGui.Button("Cancel RCS burn"u8, (float2?)null))
                RcsExecutor.Cancel(vehicle, exec, "user request");
            return;
        }

        // Guard warnings come before the estimate gating: a vehicle with no
        // usable translation has no estimates to show, and silence here is
        // exactly the confusion the warning exists to prevent.
        RcsCapabilitySnapshot cap = RcsExecutor.ProbeCached(vehicle);
        if (!cap.HasAnyTranslation)
        {
            DrawWarning("RCS unavailable: no active thruster with propellant can translate");
            return;
        }

        // The estimates are computed against the loaded first burn only
        // (stock loads BurnPlan.FirstBurn exclusively); showing them on a
        // later burn's editor would be someone else's numbers.
        if (exec == null || !exec.Estimates.Valid)
            return;
        BurnTarget? loaded = flightComputer.Burn;
        if (loaded == null
            || Math.Abs(loaded.ImpulsiveInstant.Seconds() - burn.Time.Seconds())
               > RcsExecutor.BurnIdentityToleranceSec)
            return;
        ref readonly RcsEstimates est = ref exec.Estimates;
        if (est.HoldFeasible)
            ImGuiHelper.DrawTextWidget("Hold est."u8,
                $"{est.HoldPropellantKg:F1} kg, {est.HoldDurationSec:F0} s");
        if (est.AlignFeasible)
            ImGuiHelper.DrawTextWidget("Align est."u8,
                $"{est.AlignTotalPropellantKg:F1} kg, " +
                $"{est.AlignDurationSec:F0} s ({RcsExecutor.AxisName(est.AlignAxis)})");

        double neededKg = est.RequiredPropellantKg(attitude);
        double availableKg = RcsExecutor.AvailablePropellantCached(vehicle);
        if (neededKg > availableKg)
            DrawWarning($"Propellant short: needs ~{neededKg:F0} kg, {availableKg:F0} kg available");
    }

    private static void DrawWarning(string text)
    {
        ImGui.PushStyleColor(ImGuiCol.Text, Color.Red.AsByte4);
        ImGui.Text(text);
        ImGui.PopStyleColor(1);
    }
}

using AdvancedFlightComputer.Core;
using Brutal.ImGuiApi;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.RcsTranslation;

/// <summary>
/// Injects the per-burn RCS block into the stock burn editor infobox
/// (postfix on <see cref="Burn.DrawBurnEditorWindowContent"/>): execution
/// and attitude selectors, the strategy estimates, and the live status
/// with a cancel button while an execution runs.
/// </summary>
[HarmonyPatch(typeof(Burn), nameof(Burn.DrawBurnEditorWindowContent))]
internal static class RcsBurnWindowUi
{
    static void Postfix(Burn __instance, Vehicle vehicle, FlightComputer flightComputer)
    {
        try
        {
            Draw(__instance, vehicle, flightComputer);
        }
        catch (Exception ex)
        {
            // Once per load: this runs every frame the editor is open, and a
            // persistent draw failure would otherwise flood the log.
            LogHelper.WarnOnce("rcs-burn-window",
                $"[AFC] RcsBurnWindowUi failed for vehicle='{vehicle.Id}': {ex}");
        }
    }

    private static void Draw(Burn burn, Vehicle vehicle, FlightComputer flightComputer)
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
            if (ImGui.Button(modeLabel, (Brutal.Numerics.float2?)null) && !isActiveBurn)
            {
                RcsExecution target = RcsExecRegistry.GetOrCreate(vehicle.Id);
                RcsBurnOptions o = target.GetOrCreateOptions(timeSec, dvMs);
                o.Mode = o.Mode switch
                {
                    RcsExecutionMode.Default => RcsExecutionMode.Engine,
                    RcsExecutionMode.Engine => RcsExecutionMode.Rcs,
                    _ => RcsExecutionMode.Default,
                };
            }

            if (resolved == RcsExecutionMode.Rcs || isActiveBurn)
            {
                RcsAttitudeStrategy attitude = options?.Attitude ?? RcsAttitudeStrategy.Auto;
                ImGui.Text("Attitude"u8);
                ImGui.SameLine(120f);
                if (ImGui.Button($"{attitude}##rcsatt{timeSec:R}", (Brutal.Numerics.float2?)null)
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
            }
        }

        if (resolved == RcsExecutionMode.Rcs || isActiveBurn)
            DrawEstimatesAndStatus(burn, vehicle, flightComputer, exec, isActiveBurn);
    }

    private static void DrawEstimatesAndStatus(
        Burn burn, Vehicle vehicle, FlightComputer flightComputer, RcsExecution? exec, bool isActiveBurn)
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
                         && (Math.Abs(flightComputer.ErrorAngles.Y) > flightComputer.AngleDeadband
                             || Math.Abs(flightComputer.ErrorAngles.Z) > flightComputer.AngleDeadband))
                    phase = "aligning";
                else
                    phase = "firing";
            }
            string strategy = exec.ResolvedStrategy == RcsAttitudeStrategy.Align
                ? $"Align {RcsExecutor.AxisName(exec.ResolvedAxis)}"
                : "Hold";
            ImGuiHelper.DrawTextWidget("RCS status"u8, $"{phase} ({strategy})");
            if (bt != null)
                ImGuiHelper.DrawTextWidget("To go"u8, $"{bt.DeltaVToGoCci.Length():F2} m/s");
            if (ImGui.Button("Cancel RCS burn"u8, (Brutal.Numerics.float2?)null))
                RcsExecutor.Cancel(vehicle, exec, "user request");
            return;
        }

        // The estimates are computed against the loaded first burn only
        // (stock loads BurnPlan.FirstBurn exclusively); showing them on a
        // later burn's editor would be someone else's numbers.
        if (exec == null || !exec.Estimates.Valid)
            return;
        BurnTarget? loaded = flightComputer.Burn;
        if (loaded == null
            || Math.Abs(loaded.ImpulsiveInstant.Seconds() - burn.Time.Seconds()) > 0.5)
            return;
        ref readonly RcsEstimates est = ref exec.Estimates;
        if (est.HoldFeasible)
            ImGuiHelper.DrawTextWidget("Hold est."u8,
                $"{est.HoldPropellantKg:F1} kg, {est.HoldDurationSec:F0} s");
        if (est.AlignFeasible)
            ImGuiHelper.DrawTextWidget("Align est."u8,
                $"{est.AlignPropellantKg + est.AlignSlewPropellantKg:F1} kg, " +
                $"{est.AlignDurationSec:F0} s ({RcsExecutor.AxisName(est.AlignAxis)})");
    }
}

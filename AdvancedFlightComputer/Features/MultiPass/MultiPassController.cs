using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.ImGuiApi;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Orchestration entry points called from the plan-window UI when the
/// user commits or cancels a multi-pass execution. Splitting these out
/// keeps Patch_DrawPlanWindow focused on Harmony glue and ImGui layout
/// while this file owns the lifecycle.
/// </summary>
internal static class MultiPassController
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private static readonly ImColor8 StatusGrey = new(120, 120, 120, 255);

    /// <summary>
    /// Commits pass 0 as a real Burn and registers the execution so
    /// PassCompletionPatch picks up the rest. SaveId comes from
    /// <see cref="SaveLoadObserver.CurrentSaveId"/>; an empty value
    /// makes the entry transient (in-memory only) until the user
    /// first saves the game, at which point SaveLoadObserver rekeys
    /// it to that save.
    /// </summary>
    public static void Start(Vehicle source, string typeKey)
    {
        if (source.Orbit?.Parent == null) return;

        IManeuverIntent? intent = BuildIntent(source, typeKey);
        if (intent == null) return;

        var exec = new MultiPassExecution
        {
            SaveId = SaveLoadObserver.CurrentSaveId,
            VehicleId = source.Id,
            Intent = intent,
            Mode = MultiPassUI.CurrentSplitMode,
            PassCountTotal = MultiPassUI.PassCount,
            PassIndex = 0,
        };

        if (DebugConfig.MultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPassController.Start: vehicle='{source.Id}' kind='{intent.Kind}' " +
                $"passes={MultiPassUI.PassCount} mode={MultiPassUI.CurrentSplitMode} " +
                $"saveId='{exec.SaveId}'");

        string? failure = MultiPassCommitter.TryCommitNext(source, exec);
        if (failure != null)
        {
            DefaultCategory.Log.Warning(
                $"[AFC] MultiPassController.Start: vehicle={source.Id} pass 0 could not be committed ({failure}); aborting.");
            TimedAlert.Create($"Multi-pass failed: {failure}", Color.Red, 4.0);
            return;
        }

        MultiPassRegistry.Add(exec);
    }

    /// <summary>
    /// Renders the "pass i of N" status line plus the Cancel button
    /// when an execution is active for <paramref name="source"/>.
    /// </summary>
    public static void DrawStatus(Vehicle source)
    {
        if (!MultiPassRegistry.TryGet(source.Id, out var exec))
            return;

        ImGui.PushStyleColor(ImGuiCol.Text, StatusGrey);
        ImGui.Text(string.Format(Inv,
            "Multi-pass active: pass {0} of {1}",
            exec.PassIndex + 1, exec.PassCountTotal));
        ImGui.PopStyleColor();

        ImGui.Spacing();
        if (ImGuiHelper.DrawButton("Cancel remaining passes"u8,
                KSAColor.DarkGrey, KSAColor.Xkcd.DustyBlue, Color.Red))
            CancelExecution(source, exec);
    }

    private static void CancelExecution(Vehicle source, MultiPassExecution exec)
    {
        // Already-fired passes stay applied to the trajectory; only
        // the still-queued burn (if any) gets removed.
        Burn? pending = exec.TryResolveCurrentBurn(source.FlightComputer.BurnPlan);
        if (pending != null)
        {
            InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
            {
                Burn = pending,
                FlightComputer = source.FlightComputer,
                DeleteBurn = true,
            });
        }
        DefaultCategory.Log.Info(
            $"[AFC] MultiPass: vehicle={source.Id} user cancelled at pass " +
            $"{exec.PassIndex + 1}/{exec.PassCountTotal}.");

        PassCompletionPatch.OnRegistryRemovedExternally(source.Id);
        MultiPassRegistry.Remove(source.Id);
    }

    private static IManeuverIntent? BuildIntent(Vehicle source, string typeKey)
    {
        if (source.Orbit?.Parent == null) return null;
        string parentId = source.Orbit.Parent.Id;

        if (typeKey == ManeuverTools.ManeuverTools.KeySetApoapsis
            || typeKey == ManeuverTools.ManeuverTools.KeySetPeriapsis)
        {
            bool isSetApoapsis = typeKey == ManeuverTools.ManeuverTools.KeySetApoapsis;
            double parentRadius = source.Orbit.Parent.MeanRadius;
            return new ApseIntent
            {
                IsSetApoapsis = isSetApoapsis,
                TargetRadiusMeters = ManeuverToolsWindow.TargetAltitude + parentRadius,
                ParentId = parentId,
            };
        }

        if (typeKey == ManeuverTools.ManeuverTools.KeyMatchInclination)
        {
            IOrbiter? target = ManeuverToolsWindow.GetSelectedTargetOrbiter();
            if (target == null) return null;
            return new MatchInclinationIntent
            {
                TargetId = target.Id,
                UseDescendingNode = ManeuverToolsWindow.UseDescendingNode,
                ParentId = parentId,
            };
        }

        if (typeKey == ManeuverTools.ManeuverTools.KeySetInclination)
        {
            return new SetInclinationIntent
            {
                TargetInclinationRad = ManeuverToolsWindow.TargetInclinationRad,
                Reference = ManeuverToolsWindow.InclinationRef,
                UseDescendingNode = ManeuverToolsWindow.UseDescendingNode,
                ParentId = parentId,
            };
        }

        if (typeKey == ManeuverTools.ManeuverTools.KeyStockCircularizeApoapsis
            || typeKey == ManeuverTools.ManeuverTools.KeyStockCircularizePeriapsis)
        {
            bool isAtApoapsis = typeKey == ManeuverTools.ManeuverTools.KeyStockCircularizeApoapsis;
            return new CircularizeIntent
            {
                IsAtApoapsis = isAtApoapsis,
                ParentId = parentId,
            };
        }

        return null;
    }
}

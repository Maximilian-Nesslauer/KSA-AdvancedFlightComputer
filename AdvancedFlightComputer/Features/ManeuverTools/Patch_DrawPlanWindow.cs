using System;
using System.Collections.Generic;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow), new[] { typeof(IGameViewport) })]
internal static partial class Patch_DrawPlanWindow
{
    // Share the stock window identity and dimensions so changing plan type preserves its placement.
    private const string WindowId = "transfer-planning";
    private const string WindowTitle = "TRANSFER PLANNING";
    private const string WindowSignature = "KSA-TRJ";

    private static float MainWindowOffsetXPx => 440f * ImGuiHelper.InterfaceScale;
    private static float MainWindowOffsetYPx => 50f * ImGuiHelper.InterfaceScale;
    private static float MainWindowWidthPx => 400f * ImGuiHelper.InterfaceScale;
    private static float MainWindowHeightPx => 1050f * ImGuiHelper.InterfaceScale;

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private static readonly List<TransferObject> _sourceListBuffer = new();

    private static Burn? _ourBurn;
    private static OrbitalTransfers.PorkChopEntry? _lastEntry;
    private static Vehicle? _lastSource;
    private static bool _showFlightPlanPreview;
    private static bool _showOrbitPreview;

    static bool Prefix(IGameViewport inViewport)
    {
        TransferType transferType;
        try
        {
            if (StockPlanner.TransferType is not TransferType current)
                return true;
            transferType = current;
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("maneuvertools-type-lookup:" + ex.GetType().Name,
                $"[AFC] ManeuverTools Prefix (type lookup): {ex}");
            return true;
        }

        if (!ManeuverTools.IsHandledType(transferType.GetKey()))
        {
            DropPlanState();
            return true;
        }

#if DEBUG
        using var _perf = new PerfTracker.Scope("Patch_DrawPlanWindow.Prefix");
#endif

        try
        {
            DrawWindow(inViewport, transferType);
        }
        catch (Exception ex)
        {
            // Separate plan types must not silence each other after a draw failure.
            string typeKey = transferType.GetKey();
            LogHelper.WarnOnce($"maneuvertools-draw:{typeKey}:{ex.GetType().Name}",
                $"[AFC] ManeuverTools Prefix (plan type '{typeKey}', "
                + $"source '{_lastSource?.Id ?? "none"}'): {ex}");
            // Do not let stock begin the same window after a partial AFC draw.
        }
        return false;
    }

    [HarmonyTranspiler]
    static IEnumerable<CodeInstruction> Transpiler(
        IEnumerable<CodeInstruction> instructions)
        => PlanWindowPatchPipeline.Rewrite(instructions);

    [HarmonyPostfix]
    static void Postfix(IGameViewport inViewport)
        => Patch_TransferPlanner_DrawPlanWindow_HohmannMarkers.Draw(inViewport);

    private static void DrawWindow(IViewport inViewport, TransferType transferType)
    {
        ImGui.SetNextWindowPos(
            inViewport.Position + new float2(inViewport.Size.X - MainWindowOffsetXPx, MainWindowOffsetYPx),
            ImGuiCond.Appearing, (float2?)null);

        bool open = StockPlanner.ShowPlanWindow;
        // ConsoleStyle.BeginWindow closes the ImGui window itself when it returns false.
        if (!ConsoleStyle.BeginWindow(WindowId, WindowTitle, WindowSignature, ref open,
                new float2(MainWindowWidthPx, MainWindowHeightPx), ImGuiWindowFlags.NoScrollbar))
            return;

        Commit commit = default;
        try
        {
            if (!open)
            {
                HandleWindowClose();
                return;
            }

            StockPlanner.ShowPlanWindow = true;

            ConsoleStyle.BeginBody();
            ConsoleStyle.PushWidgetStyle();
            try
            {
                commit = DrawBody(transferType);
            }
            finally
            {
                ConsoleStyle.PopWidgetStyle();
                ConsoleStyle.EndBody();
            }

            ConsoleStyle.BeginFooter();
            bool create = DrawFooter(commit.State);
            ConsoleStyle.EndFooter();

            // Commit after EndFooter, as TransferPlanner.DrawPlanWindow does. Ready implies a resolved source.
            if (create)
                CreateSingleOrMultiPass(commit);
        }
        finally
        {
            ConsoleStyle.EndWindow();
        }

        if (_showOrbitPreview && _lastSource != null)
        {
            if (MultiPassUI.HasMultiPassPreview)
                MultiPassUI.RenderMarkers(inViewport, _lastSource);
            else if (_lastEntry != null)
                DrawOrbitMarkers(inViewport);
        }

        if (_showFlightPlanPreview)
            DrawFlightPlanWindow(inViewport);
    }

    private static Commit DrawBody(TransferType transferType)
    {
        if (!DrawPlanTypeDropdown(ref transferType))
            return default;

        Vehicle? source = DrawSourceDropdown();
        if (source?.Orbit == null)
            return default;
        _lastSource = source;
        CleanupStaleBurn(source);

        ImGui.Separator();

        // The controls, calculation and preview must use the same planning trajectory.
        PlanningBasis basis = PlanningBasis.For(source);
        string typeKey = transferType.GetKey();

        ManeuverToolsWindow.DrawInline(typeKey, source, basis);

        var result = ComputeManeuver(typeKey, source, basis);
        if (result == null)
        {
            _lastEntry = null;
            return default;
        }

        _lastEntry = BuildTransferEntry(source, result.Value, basis);

        ImGui.Separator();
        DrawManeuverInfo(result.Value);

        MultiPassUI.Draw(source, result.Value, typeKey);

        CommitState state = ResolveCommitState(source, typeKey);
        if (state == CommitState.MultiPassRunning)
        {
            ImGui.Spacing();
            MultiPassController.DrawStatus(source);
        }

        // Stock renders committed single burns, while execution across several passes keeps its preview controls.
        if (_ourBurn == null)
        {
            ConsoleWidgets.Rule();
            ConsoleUi.CheckboxRow("PREVIEW ORBIT".AsSpan(), "AfcMtPreviewOrbit".AsSpan(),
                ref _showOrbitPreview);
            ConsoleUi.CheckboxRow("PREVIEW FLIGHT PLAN".AsSpan(), "AfcMtPreviewPlan".AsSpan(),
                ref _showFlightPlanPreview);
        }

        return new Commit(state, source, result.Value, typeKey, basis);
    }

    #region Dropdowns

    private static bool DrawPlanTypeDropdown(ref TransferType transferType)
    {
        TransferType prev = transferType;
        if (ConsoleUi.ComboRow("PLAN TYPE".AsSpan(), "PlanType".AsSpan(), ref transferType,
                TransferPlanner.TransferTypes)
            && transferType.GetKey() != prev.GetKey())
        {
            StockPlanner.TransferType = transferType;
            StockPlanner.TransferCalculated = false;
            ManeuverToolsWindow.OnTypeChanged();
            OnManeuverContextChanged();

            if (!ManeuverTools.IsHandledType(transferType.GetKey()))
            {
                GameReflection.TransferPlanner_SetTransferInfo!.Invoke(null, null);
                return false;
            }
        }
        return true;
    }

    private static Vehicle? DrawSourceDropdown()
    {
        TransferObject sourceBody = StockPlanner.SourceBody;

        // Rebuild from Program.VehiclesInFrame because lookup indices can move after deregistration.
        ReadOnlySpan<Vehicle> vehiclesInFrame = Program.VehiclesInFrame;
        _sourceListBuffer.Clear();
        foreach (Vehicle vehicle in vehiclesInFrame)
            _sourceListBuffer.Add(new TransferObject(vehicle));
        List<TransferObject> list = _sourceListBuffer;

        if (ImGui.IsWindowAppearing() || sourceBody.GetKey() == "N/A")
        {
            // Use a negative lookup index for no selection because default(TransferObject) names index zero.
            sourceBody = new TransferObject(-1);
            if (Program.ControlledVehicle != null)
            {
                for (int i = 0; i < list.Count; i++)
                {
                    if (list[i].GetKey() == Program.ControlledVehicle.Id)
                    {
                        sourceBody = list[i];
                        break;
                    }
                }
            }
            if (sourceBody.GetKey() == "N/A" && list.Count > 0)
                sourceBody = list[0];

            StockPlanner.SourceBody = sourceBody;
        }

        TransferObject prev = sourceBody;
        if (ConsoleUi.ComboRow("SOURCE".AsSpan(), "Source".AsSpan(), ref sourceBody, _sourceListBuffer)
            && sourceBody.GetKey() != prev.GetKey())
        {
            StockPlanner.SourceBody = sourceBody;
            ManeuverToolsWindow.OnSourceChanged();
        }

        return sourceBody.Body as Vehicle;
    }

    #endregion

    #region Maneuver Info + Create

    private static void DrawManeuverInfo(OrbitManeuvers.ManeuverResult maneuver)
    {
        double dvMag = maneuver.DvCci.Length();
        double timeToNode = (maneuver.BurnTime - Universe.GetElapsedTime()).Seconds();

        ConsoleWidgets.Readout("REQUIRED DELTA V".AsSpan(),
            string.Format(Inv, "{0:F1} m/s", dvMag).AsSpan());

        if (timeToNode > 0)
        {
            ConsoleWidgets.Readout("TIME TO BURN".AsSpan(),
                FormatHelper.FormatDuration(timeToNode).AsSpan());
        }
    }

    #endregion

    #region Helpers

    private static OrbitManeuvers.ManeuverResult? ComputeManeuver(
        string key, Vehicle source, PlanningBasis basis)
    {
        Orbit orbit = basis.Orbit;
        double parentRadius = source.Parent?.MeanRadius ?? 0.0;
        // Search after the preceding burn when chaining maneuvers.
        UniverseTime now = basis.Earliest;

        if (key == ManeuverTools.KeySetPeriapsis)
            return OrbitManeuvers.ComputeSetPeriapsis(
                orbit, ManeuverToolsWindow.TargetAltitude, parentRadius, now);

        if (key == ManeuverTools.KeySetApoapsis)
            return OrbitManeuvers.ComputeSetApoapsis(
                orbit, ManeuverToolsWindow.TargetAltitude, parentRadius, now);

        if (key == ManeuverTools.KeyMatchInclination)
        {
            Orbit? targetOrbit = ManeuverToolsWindow.GetSelectedTargetOrbit();
            if (targetOrbit == null) return null;
            return OrbitManeuvers.ComputeMatchInclination(
                orbit, targetOrbit, ManeuverToolsWindow.UseDescendingNode, now);
        }

        if (key == ManeuverTools.KeySetInclination)
        {
            return OrbitManeuvers.ComputeSetInclination(
                orbit, ManeuverToolsWindow.TargetInclinationRad,
                ManeuverToolsWindow.UseDescendingNode, now,
                ManeuverToolsWindow.InclinationRef);
        }

        if (key == ManeuverTools.KeyStockCircularizeApoapsis)
            return OrbitManeuvers.ComputeCircularize(orbit, useApoapsis: true, now);

        if (key == ManeuverTools.KeyStockCircularizePeriapsis)
            return OrbitManeuvers.ComputeCircularize(orbit, useApoapsis: false, now);

        return null;
    }

    // Burn.Equals compares time and delta V, so plan membership alone does not prove that the burn belongs to the selected vehicle.
    private static void CleanupStaleBurn(Vehicle source)
    {
        if (_ourBurn == null) return;
        if (_ourBurn.Vehicle.Id != source.Id
            || !source.FlightComputer.BurnPlan.TryGetBurn(_ourBurn))
            _ourBurn = null;
    }

    // The keybind and View menu close the window without drawing its body.
    internal static void TickWindowState()
    {
        if (!TransferPlanner.ShowPlanWindow)
        {
            DropPlanState();
            HohmannFlybyUI.ClearPreview();
            HohmannMultiPassUI.ClearFlybyPreview();
        }
    }

    private static void DropPlanState()
    {
        _ourBurn = null;
        _lastEntry = null;
        _lastSource = null;
    }

    private static void HandleWindowClose()
    {
        // The public setter also clears stock transfer state.
        TransferPlanner.ShowPlanWindow = false;
        DropPlanState();
    }

    internal static void Reset()
    {
        DropPlanState();
        _showFlightPlanPreview = false;
        _showOrbitPreview = false;
    }

    // Keep the preview toggles when execution is cancelled and disable them when it completes successfully.
    internal static void OnMultiPassCompleted()
    {
        _showOrbitPreview = false;
        _showFlightPlanPreview = false;
    }

    #endregion
}

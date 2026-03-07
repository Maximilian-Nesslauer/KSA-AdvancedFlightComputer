using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.OberthMultiPass;
using Brutal.ImGuiApi;
using Brutal.Logging;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance.Buffers;
using HarmonyLib;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// Prefix on TransferPlanner.DrawPlanWindow that takes over the entire window
/// when one of our plan types is selected. Draws the Plan Type dropdown, Source
/// dropdown, type-specific controls, and Create button all inside the stock
/// "Transfer Planning" window.
///
/// Returns false (skip original) for our types, true for stock types.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.DrawPlanWindow))]
static class Patch_DrawPlanWindow
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private static Burn? _ourBurn;
    private static OrbitalTransfers.PorkChopEntry? _lastEntry;
    private static Vehicle? _lastSource;
    private static bool _showFlightPlanPreview;
    private static bool _showOrbitPreview;

    static bool Prefix(Viewport inViewport)
    {
        try
        {
            var transferType = (TransferType)GameReflection.TransferPlanner_transferType!.GetValue(null)!;
            if (!ManeuverTools.IsOurType(transferType.GetKey()))
            {
                _ourBurn = null;
                _lastEntry = null;
                _lastSource = null;
                return true;
            }

            DrawWindow(inViewport, transferType);
            return false;
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] ManeuverTools Prefix: {ex.Message}");
            return true;
        }
    }

    private static void DrawWindow(Viewport inViewport, TransferType transferType)
    {
        CleanupStaleBurn();

        ImGui.SetNextWindowPos(
            inViewport.Position + new float2(inViewport.Size.X - 440, 50f),
            ImGuiCond.Appearing, (float2?)null);
        ImGui.SetNextWindowSize(new float2(400f, 600f), ImGuiCond.Appearing);

        bool pOpen = (bool)GameReflection.TransferPlanner_showPlanWindow!.GetValue(null)!;
        if (!ImGui.Begin("Transfer Planning"u8, ref pOpen,
                ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoCollapse
                | ImGuiWindowFlags.NoSavedSettings))
        {
            if (!pOpen) HandleWindowClose();
            ImGui.End();
            return;
        }
        if (!pOpen)
        {
            HandleWindowClose();
            ImGui.End();
            return;
        }
        GameReflection.TransferPlanner_showPlanWindow!.SetValue(null, true);

        if (!DrawPlanTypeDropdown(ref transferType))
        {
            ImGui.End();
            return;
        }

        ImGui.Text(""u8);

        Vehicle? source = DrawSourceDropdown();
        if (source?.Orbit == null)
        {
            ImGui.End();
            return;
        }
        _lastSource = source;

        ImGui.Separator();

        ManeuverToolsWindow.DrawInline(transferType.GetKey(), source);

        var result = ComputeManeuver(transferType.GetKey(), source);
        if (result != null)
        {
            var (entry, _) = OrbitManeuvers.BuildTransferEntry(source, result.Value);
            _lastEntry = entry;

            ImGui.Separator();
            DrawManeuverInfo(result.Value);

            OberthManeuverIntegration.DrawOberthSection(source, result.Value, transferType.GetKey());

            ImGui.Spacing();
            DrawCreateButton(source, result.Value, transferType.GetKey());

            if (_ourBurn == null && !MultiPassState.HasActiveSplit && !MultiPassState.HasPreview)
            {
                ImGui.Separator();
                ImGuiHelper.BeginColumns(2, new float[] { 0.9f });
                ImGuiHelper.DrawCheckbox("Preview Orbit"u8, ref _showOrbitPreview, isChanged: false);
                ImGuiHelper.DrawCheckbox("Preview Flight Plan"u8, ref _showFlightPlanPreview,
                    isChanged: false);
                ImGuiHelper.EndColumns();
            }
        }
        else
        {
            _lastEntry = null;
            if (MultiPassState.HasPreview)
                MultiPassState.ClearPreview();
        }

        ImGui.End();

        if (_showOrbitPreview && _ourBurn == null && _lastEntry != null
                && _lastSource != null && !MultiPassState.HasPreview
                && !MultiPassState.HasActiveSplit)
        {
            var uiCtx = new Astronomical.UiContext(
                inViewport, _lastSource, Color.Green,
                TrueAnomaly.Zero, new TrueAnomaly(Math.PI * 2.0), null);
            _lastEntry.FlightPlan.DrawUi(inViewport, uiCtx);
        }

        if (MultiPassState.HasPreview || MultiPassState.HasActiveSplit)
            MultiPassRenderer.RenderPreviewMarkers(inViewport);

        if (_showFlightPlanPreview && _lastEntry != null)
            DrawFlightPlanWindow(inViewport);
    }

    #region Dropdowns

    private static bool DrawPlanTypeDropdown(ref TransferType transferType)
    {
        TransferType prev = transferType;
        if (ImGuiHelper.DrawCombo("Plan Type:"u8, ref transferType, TransferPlanner.TransferTypes)
            && transferType.GetKey() != prev.GetKey())
        {
            GameReflection.TransferPlanner_transferType!.SetValue(null, transferType);
            GameReflection.TransferPlanner_transferCalculated!.SetValue(null, false);
            ManeuverToolsWindow.OnTypeChanged();
            OberthUI.OnTypeChanged();

            if (!ManeuverTools.IsOurType(transferType.GetKey()))
            {
                GameReflection.TransferPlanner_SetTransferInfo!.Invoke(null, null);
                return false;
            }
        }
        return true;
    }

    private static Vehicle? DrawSourceDropdown()
    {
        var sourceBody = (TransferObject)GameReflection.TransferPlanner_sourceBody!.GetValue(null)!;
        var vehicleList = new List<TransferObject>();
        foreach (Vehicle v in Universe.CurrentSystem!.Vehicles.GetList())
            vehicleList.Add(new TransferObject(v));

        if (ImGui.IsWindowAppearing() || sourceBody.GetKey() == "N/A")
        {
            if (Program.ControlledVehicle != null)
            {
                int idx = vehicleList.FindIndex(v => v.GetKey() == Program.ControlledVehicle.Id);
                if (idx > -1)
                    sourceBody = vehicleList[idx];
            }
            else if (vehicleList.Count > 0)
                sourceBody = vehicleList[0];
            else
                sourceBody = default;

            GameReflection.TransferPlanner_sourceBody!.SetValue(null, sourceBody);
        }

        if (MultiPassState.HasActiveSplit)
            ImGui.BeginDisabled();

        TransferObject prev = sourceBody;
        if (ImGuiHelper.DrawCombo("Source:"u8, ref sourceBody, vehicleList)
            && sourceBody.GetKey() != prev.GetKey())
        {
            GameReflection.TransferPlanner_sourceBody!.SetValue(null, sourceBody);
            ManeuverToolsWindow.OnSourceChanged();
        }

        if (MultiPassState.HasActiveSplit)
            ImGui.EndDisabled();

        return sourceBody.Body as Vehicle;
    }

    #endregion

    #region Maneuver Info + Create

    private static void DrawManeuverInfo(OrbitManeuvers.ManeuverResult maneuver)
    {
        double dvMag = maneuver.DvCci.Length();
        double timeToNode = maneuver.BurnTime.Seconds() - Universe.GetElapsedSimTime().Seconds();

        ImGuiHelper.DrawTextWidget("Required Delta V:"u8,
            string.Format(Inv, "{0:F1} m/s", dvMag));

        if (timeToNode > 0)
        {
            ImGuiHelper.DrawTextWidget("Time to Burn:"u8,
                ManeuverToolsWindow.FormatTimeSpan(timeToNode));
        }
    }

    private static void DrawFlightPlanWindow(Viewport inViewport)
    {
        ImGui.SetNextWindowPos(
            inViewport.Position + new float2(620f, 40f),
            ImGuiCond.Appearing, (float2?)null);
        ImGui.SetNextWindowSize(new float2(460f, 620f), ImGuiCond.Appearing);

        if (ImGui.Begin("Maneuver Flight Plan"u8, ref _showFlightPlanPreview,
                ImGuiWindowFlags.NoSavedSettings | ImGuiWindowFlags.NoFocusOnAppearing))
        {
            _lastEntry!.FlightPlan.DrawPatchInfo();
        }
        ImGui.End();
    }

    private static void DrawCreateButton(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, string typeKey)
    {
        // When an active multi-pass split exists, the pass list + Back to Single
        // are already shown in DrawOberthSection. Just suppress the Create button.
        if (MultiPassState.HasActiveSplit)
            return;

        if (_ourBurn != null)
        {
            if (_ourBurn.Time < Universe.GetElapsedSimTime())
            {
                _ourBurn = null;
            }
            else
            {
                ImGui.PushStyleColor(ImGuiCol.Text, new ImColor8(120, 120, 120, 255));
                ImGui.Text("Maneuver node created."u8);
                ImGui.PopStyleColor();
                return;
            }
        }

        if (ImGuiHelper.DrawButton("Create"u8, KSAColor.DarkGrey,
                KSAColor.Xkcd.DustyBlue, Color.Green))
        {
            if (OberthUI.CurrentPassCount > 1)
            {
                OberthManeuverIntegration.CreateMultiPassBurns(source, maneuver, typeKey);
            }
            else
            {
                PatchedConic? patch = source.FlightPlan.TryFindPatch(maneuver.BurnTime);
                if (patch != null)
                {
                    OrbitPointCce point = patch.Orbit.GetPointAt(maneuver.BurnTime);
                    _ourBurn = Burn.Create(point, maneuver.BurnTime.Seconds(),
                        maneuver.DvVlf, patch, source);
                    source.FlightComputer.AddBurn(_ourBurn);
                    _ourBurn.IsGizmoActive = false;
                }
            }
        }
    }

    #endregion

    #region Visual Orbit Preview

    /// <summary>
    /// Renders the post-burn orbit in the 3D view. Called from
    /// Patch_OnPreRender when our type is active and preview is enabled.
    /// Follows the same pattern as stock DrawSelectedTransfer.
    /// </summary>
    internal static void RenderOrbitPreview(Viewport inViewport)
    {
        if (!_showOrbitPreview || _ourBurn != null || _lastEntry == null || _lastSource == null)
            return;
        if (MultiPassState.HasPreview || MultiPassState.HasActiveSplit)
            return;

        FlightPlan fp = _lastEntry.FlightPlan;
        if (fp.Patches.Count == 0)
            return;

        if (fp.Patches[0].Orbit.IsMissingPoints())
        {
            foreach (PatchedConic patch in fp.Patches)
            {
                patch.HidePatch = false;
                MemoryOwner<OrbitPointCce> points = UpdateTaskUtils.GenerateSpacedPoints(patch);
                patch.Orbit.UpdateCachedPoints(points);
            }
        }

        fp.AddLineInstances(inViewport, _lastSource, isActive: true,
            drawVehiclePosition: false, TrueAnomaly.NaN, TrueAnomaly.NaN);
    }

    #endregion

    #region Helpers

    private static OrbitManeuvers.ManeuverResult? ComputeManeuver(string key, Vehicle source)
    {
        Orbit orbit = source.Orbit;
        double parentRadius = source.Parent?.MeanRadius ?? 0.0;
        SimTime now = Universe.GetElapsedSimTime();

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
                ManeuverToolsWindow.UseDescendingNode, now);
        }

        return null;
    }

    private static void CleanupStaleBurn()
    {
        if (_ourBurn == null) return;

        Vehicle? source = (_ourBurn.Vehicle != null
            && _ourBurn.Vehicle.FlightComputer.BurnPlan.TryGetBurn(_ourBurn))
            ? _ourBurn.Vehicle : null;

        if (source == null)
            _ourBurn = null;
    }

    private static void HandleWindowClose()
    {
        GameReflection.TransferPlanner_showPlanWindow!.SetValue(null, false);
        GameReflection.TransferPlanner_transferCalculated!.SetValue(null, false);
        GameReflection.TransferPlanner_selectedEntry!.SetValue(null, null);
        GameReflection.TransferPlanner_transferBurn!.SetValue(null, null);
        _ourBurn = null;
        _lastEntry = null;
        _lastSource = null;
        OberthUI.OnTypeChanged();
        // Invalidate so reopening with the same inputs forces a fresh preview recompute.
        OberthManeuverIntegration.InvalidatePreviewHash();
    }

    internal static void Reset()
    {
        _ourBurn = null;
        _lastEntry = null;
        _lastSource = null;
        _showFlightPlanPreview = false;
        _showOrbitPreview = false;
        OberthManeuverIntegration.Reset();
        MultiPassState.Reset();
        OberthUI.Reset();
    }

    #endregion
}

/// <summary>
/// Postfix on TransferPlanner.OnPreRender to render the visual orbit preview
/// in the 3D view when one of our plan types is active.
/// </summary>
[HarmonyPatch(typeof(TransferPlanner), nameof(TransferPlanner.OnPreRender))]
static class Patch_OnPreRender
{
    static void Postfix(Viewport inViewport)
    {
        try
        {
            Patch_DrawPlanWindow.RenderOrbitPreview(inViewport);
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] ManeuverTools OnPreRender: {ex.Message}");
        }
    }
}

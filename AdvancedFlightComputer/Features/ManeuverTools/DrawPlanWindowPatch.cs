using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
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
internal static class Patch_DrawPlanWindow
{
    // Window placement, kept to match stock TransferPlanner.cs:181-182, 1012-1013.
    private const float MainWindowOffsetX = 440f;
    private const float MainWindowOffsetY = 50f;
    private const float MainWindowWidth = 400f;
    private const float MainWindowHeight = 600f;
    private const float FlightPlanWindowOffsetX = 620f;
    private const float FlightPlanWindowOffsetY = 40f;
    private const float FlightPlanWindowWidth = 460f;
    private const float FlightPlanWindowHeight = 620f;

    private static Burn? _ourBurn;
    private static OrbitalTransfers.PorkChopEntry? _lastEntry;
    private static Vehicle? _lastSource;
    private static bool _showFlightPlanPreview;
    private static bool _showOrbitPreview;

    private static List<TransferObject>? _vehicleList;
    private static int _lastVehicleCount;

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
            inViewport.Position + new float2(inViewport.Size.X - MainWindowOffsetX, MainWindowOffsetY),
            ImGuiCond.Appearing, (float2?)null);
        ImGui.SetNextWindowSize(new float2(MainWindowWidth, MainWindowHeight), ImGuiCond.Appearing);

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

            ImGui.Spacing();
            DrawCreateButton(source, result.Value);

            if (_ourBurn == null)
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
        }

        ImGui.End();

        if (_showOrbitPreview && _lastEntry != null && _lastSource != null)
            DrawOrbitMarkers(inViewport);

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
        int currentCount = Universe.CurrentSystem!.CountOf<Vehicle>();
        if (_vehicleList == null || currentCount != _lastVehicleCount)
        {
            _vehicleList = new List<TransferObject>();
            foreach (Vehicle v in Universe.CurrentSystem.All.OfType<Vehicle>())
                _vehicleList.Add(new TransferObject(v));
            _lastVehicleCount = currentCount;
        }
        var vehicleList = _vehicleList;

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

        TransferObject prev = sourceBody;
        if (ImGuiHelper.DrawCombo("Source:"u8, ref sourceBody, vehicleList)
            && sourceBody.GetKey() != prev.GetKey())
        {
            GameReflection.TransferPlanner_sourceBody!.SetValue(null, sourceBody);
            ManeuverToolsWindow.OnSourceChanged();
        }

        return sourceBody.Body as Vehicle;
    }

    #endregion

    #region Maneuver Info + Create

    private static void DrawManeuverInfo(OrbitManeuvers.ManeuverResult maneuver)
    {
        double dvMag = maneuver.DvCci.Length();
        double timeToNode = maneuver.BurnTime.Seconds() - Universe.GetElapsedSimTime().Seconds();

        ImGuiHelper.DrawTextWidget("Required Delta V:"u8,
            string.Format(System.Globalization.CultureInfo.InvariantCulture,
                "{0:F1} m/s", dvMag));

        if (timeToNode > 0)
        {
            ImGuiHelper.DrawTextWidget("Time to Burn:"u8,
                ManeuverToolsWindow.FormatTimeSpan(timeToNode));
        }
    }

    private static void DrawFlightPlanWindow(Viewport inViewport)
    {
        ImGui.SetNextWindowPos(
            inViewport.Position + new float2(FlightPlanWindowOffsetX, FlightPlanWindowOffsetY),
            ImGuiCond.Appearing, (float2?)null);
        ImGui.SetNextWindowSize(new float2(FlightPlanWindowWidth, FlightPlanWindowHeight),
            ImGuiCond.Appearing);

        if (ImGui.Begin("Maneuver Flight Plan"u8, ref _showFlightPlanPreview,
                ImGuiWindowFlags.NoSavedSettings | ImGuiWindowFlags.NoFocusOnAppearing))
        {
            _lastEntry!.FlightPlan.DrawPatchInfo();
        }
        ImGui.End();
    }

    private static void DrawCreateButton(Vehicle source, OrbitManeuvers.ManeuverResult maneuver)
    {
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
            PatchedConic? patch = source.FlightPlan.TryFindPatch(maneuver.BurnTime);
            if (patch != null)
            {
                OrbitPointCce point = patch.Orbit.GetPointAt(maneuver.BurnTime);
                _ourBurn = Burn.Create(point, maneuver.BurnTime.Seconds(),
                    maneuver.DvVlf, patch, source);
                _ourBurn.IsGizmoActive = false;

                // Stock pattern (TransferPlanner.cs:438-444): the actual
                // BurnPlan mutation runs at the next frame boundary so it
                // is sequenced with deletes/updates.
                InputEvents.BurnUpdateBuffer.Add(new InputEvents.BurnUpdateData
                {
                    Burn = _ourBurn,
                    FlightComputer = source.FlightComputer,
                    AddBurn = true,
                });
            }
        }
    }

    #endregion

    #region Visual Orbit Preview

    /// <summary>
    /// Renders encounter, escape, impact, closest approach, and Ap/Pe markers
    /// on the preview orbit. Called from the ImGui pass (after the main window)
    /// using the background draw list.
    /// </summary>
    private static void DrawOrbitMarkers(Viewport inViewport)
    {
        var uiContext = new Astronomical.UiContext(
            inViewport, _lastSource!, Color.Green,
            TrueAnomaly.Zero, new TrueAnomaly(Math.PI * 2.0),
            ManeuverToolsWindow.GetSelectedTargetOrbiter());
        _lastEntry!.FlightPlan.DrawUi(inViewport, uiContext);
    }

    /// <summary>
    /// Renders the post-burn orbit in the 3D view. Called from
    /// Patch_OnPreRender when our type is active and preview is enabled.
    /// Follows the same pattern as stock DrawSelectedTransfer.
    /// </summary>
    internal static void RenderOrbitPreview(Viewport inViewport)
    {
        if (!_showOrbitPreview || _ourBurn != null || _lastEntry == null || _lastSource == null)
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
                ManeuverToolsWindow.UseDescendingNode, now,
                ManeuverToolsWindow.InclinationRef);
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
        // Use the public setter so stock state (_transferBurn, _correctionBurn,
        // _selectedEntry, _lambertPatch, _transferCalculated) is cleared too;
        // setting only _showPlanWindow via reflection would leak that state.
        TransferPlanner.ShowPlanWindow = false;
        _ourBurn = null;
        _lastEntry = null;
        _lastSource = null;
    }

    internal static void Reset()
    {
        _ourBurn = null;
        _lastEntry = null;
        _lastSource = null;
        _showFlightPlanPreview = false;
        _showOrbitPreview = false;
        _vehicleList = null;
        _lastVehicleCount = 0;
    }

    #endregion
}

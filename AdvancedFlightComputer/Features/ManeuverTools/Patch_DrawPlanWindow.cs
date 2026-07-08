using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
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
    // Window placement, kept to match stock TransferPlanner.DrawPlanWindow and
    // DrawSelectedTransferFlightPlan window-position constants.
    private const float MainWindowOffsetX = 440f;
    private const float MainWindowOffsetY = 50f;
    private const float MainWindowWidth = 400f;
    private const float MainWindowHeight = 600f;
    private const float FlightPlanWindowOffsetX = 620f;
    private const float FlightPlanWindowOffsetY = 40f;
    private const float FlightPlanWindowWidth = 460f;
    private const float FlightPlanWindowHeight = 620f;

    private static readonly ImColor8 StatusGrey = new(120, 120, 120, 255);
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private static Burn? _ourBurn;
    private static OrbitalTransfers.PorkChopEntry? _lastEntry;
    private static Vehicle? _lastSource;
    private static bool _showFlightPlanPreview;
    private static bool _showOrbitPreview;

    static bool Prefix(Viewport inViewport)
    {
        TransferType transferType;
        try
        {
            transferType = (TransferType)GameReflection.TransferPlanner_transferType!.GetValue(null)!;
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] ManeuverTools Prefix (type lookup): {ex}");
            return true;
        }

        if (!ManeuverTools.IsHandledType(transferType.GetKey()))
        {
            _ourBurn = null;
            _lastEntry = null;
            _lastSource = null;
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
            DefaultCategory.Log.Warning($"[AFC] ManeuverTools Prefix: {ex}");
            // ImGui state may be partially set; do not let stock also Begin the
            // same window this frame, that produces ID-stack conflicts.
        }
        return false;
    }

    private static void DrawWindow(Viewport inViewport, TransferType transferType)
    {
        CleanupStaleBurn();

        ImGui.SetNextWindowPos(
            inViewport.Position + new float2(inViewport.Size.X - MainWindowOffsetX, MainWindowOffsetY),
            ImGuiCond.Appearing, (float2?)null);
        ImGui.SetNextWindowSize(new float2(MainWindowWidth, MainWindowHeight), ImGuiCond.Appearing);

        bool pOpen = (bool)GameReflection.TransferPlanner_showPlanWindow!.GetValue(null)!;
        bool windowOpen = ImGui.Begin("Transfer Planning"u8, ref pOpen,
            ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoCollapse | ImGuiWindowFlags.NoSavedSettings);
        try
        {
            if (!pOpen)
            {
                HandleWindowClose();
                return;
            }
            if (!windowOpen)
                return;

            GameReflection.TransferPlanner_showPlanWindow!.SetValue(null, true);

            if (!DrawPlanTypeDropdown(ref transferType))
                return;

            ImGui.Text(""u8);

            Vehicle? source = DrawSourceDropdown();
            if (source?.Orbit == null)
                return;
            _lastSource = source;

            ImGui.Separator();

            ManeuverToolsWindow.DrawInline(transferType.GetKey(), source);

            var result = ComputeManeuver(transferType.GetKey(), source);
            if (result != null)
            {
                _lastEntry = BuildTransferEntry(source, result.Value);

                ImGui.Separator();
                DrawManeuverInfo(result.Value);

                MultiPassUI.Draw(source, result.Value, transferType.GetKey());

                ImGui.Spacing();
                DrawCreateButton(source, result.Value, transferType.GetKey());

                // Hidden only when a single-burn maneuver node has been
                // created (stock then owns the rendering); during active
                // multi-pass execution the checkboxes stay visible so
                // the user can toggle the future-passes overlay.
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
        }
        finally
        {
            // ImGui requires End() for every Begin(), regardless of return value.
            ImGui.End();
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
        var sourceBody = (TransferObject)GameReflection.TransferPlanner_sourceBody!.GetValue(null)!;

        // Rebuild every frame from VehiclesInFrame to match stock and avoid
        // TransferObject._index aliasing after a vehicle is destroyed.
        ReadOnlySpan<Vehicle> vehiclesInFrame = Program.VehiclesInFrame;
        Span<TransferObject> list = stackalloc TransferObject[vehiclesInFrame.Length];
        for (int i = 0; i < vehiclesInFrame.Length; i++)
            list[i] = new TransferObject(vehiclesInFrame[i]);

        if (ImGui.IsWindowAppearing() || sourceBody.GetKey() == "N/A")
        {
            sourceBody = default;
            if (Program.ControlledVehicle != null)
            {
                for (int i = 0; i < list.Length; i++)
                {
                    if (list[i].GetKey() == Program.ControlledVehicle.Id)
                    {
                        sourceBody = list[i];
                        break;
                    }
                }
            }
            if (sourceBody.GetKey() == "N/A" && list.Length > 0)
                sourceBody = list[0];

            GameReflection.TransferPlanner_sourceBody!.SetValue(null, sourceBody);
        }

        TransferObject prev = sourceBody;
        if (ImGuiHelper.DrawCombo("Source:"u8, ref sourceBody, list)
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
            string.Format(Inv, "{0:F1} m/s", dvMag));

        if (timeToNode > 0)
        {
            ImGuiHelper.DrawTextWidget("Time to Burn:"u8,
                FormatHelper.FormatDuration(timeToNode));
        }
    }

    private static void DrawFlightPlanWindow(Viewport inViewport)
    {
        // Multi-pass: show the final-pass trajectory.
        FlightPlan? fp = MultiPassUI.HasMultiPassPreview
            ? MultiPassUI.LastPassFlightPlan
            : _lastEntry?.FlightPlan;
        if (fp == null) return;

        ImGui.SetNextWindowPos(
            inViewport.Position + new float2(FlightPlanWindowOffsetX, FlightPlanWindowOffsetY),
            ImGuiCond.Appearing, (float2?)null);
        ImGui.SetNextWindowSize(new float2(FlightPlanWindowWidth, FlightPlanWindowHeight),
            ImGuiCond.Appearing);

        if (ImGui.Begin("Maneuver Flight Plan"u8, ref _showFlightPlanPreview,
                ImGuiWindowFlags.NoSavedSettings | ImGuiWindowFlags.NoFocusOnAppearing))
        {
            fp.DrawPatchInfo();
        }
        ImGui.End();
    }

    private static void DrawCreateButton(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, string typeKey)
    {
        if (MultiPassRegistry.Has(source.Id))
        {
            MultiPassController.DrawStatus(source);
            return;
        }

        if (_ourBurn != null)
        {
            if (_ourBurn.Time < Universe.GetElapsedSimTime())
            {
                _ourBurn = null;
            }
            else
            {
                ImGui.PushStyleColor(ImGuiCol.Text, StatusGrey);
                ImGui.Text("Maneuver node created."u8);
                ImGui.PopStyleColor();
                return;
            }
        }

        // Multi-pass selected but preview failed: disable Create rather
        // than silently fall back to a full-dV single burn.
        bool blockedByFailedPreview = MultiPassUI.WantsMultiPassButCannot(typeKey);
        if (blockedByFailedPreview)
            ImGui.BeginDisabled();

        if (ImGuiHelper.DrawButton("Create"u8, KSAColor.DarkGrey,
                KSAColor.Xkcd.DustyBlue, Color.Green))
        {
            if (MultiPassUI.IsArmed(typeKey))
                MultiPassController.Start(source, typeKey);
            else
                CreateSingleBurn(source, maneuver);
        }

        if (blockedByFailedPreview)
            ImGui.EndDisabled();
    }

    private static void CreateSingleBurn(Vehicle source, OrbitManeuvers.ManeuverResult maneuver)
    {
        // Routed through MultiPassCommitter so single-burn and the first
        // multi-pass pass take the same Burn.Create -> buffer-Add path.
        Burn? burn = MultiPassCommitter.QueueAddBurn(source, maneuver.BurnTime, maneuver.DvVlf);
        if (burn == null) return;

        _ourBurn = burn;
    }

    /// <summary>
    /// Builds a PorkChopEntry for <paramref name="maneuver"/> so the
    /// flight-plan preview / orbit-marker rendering pipelines (which
    /// expect a PorkChopEntry like the stock Hohmann path produces)
    /// can run unchanged.
    /// </summary>
    private static OrbitalTransfers.PorkChopEntry BuildTransferEntry(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver)
    {
        var transferData = new OrbitalTransfers.TransferData
        {
            Start = maneuver.BurnTime,
            Point = source.Orbit.GetPointAt(maneuver.BurnTime),
            DeltaVelocityCci = maneuver.DvCci,
            TransferDvVlf = maneuver.DvVlf
        };

        FlightPlan flightPlan = FlightPlan.CreateUninitialized(source.Hash);
        // Committed burns get this margin stamped by Burn.Create, so the preview's
        // impact test agrees with what the created burn's plan will compute.
        flightPlan.ImpactClearanceMargin = source.BoundingSphereRadiusBody;
        var info = new OrbitalTransfers.TransferInfo(source, source, source, usePorkChopData: false);
        // BuildFlightPlan forwards info.Target as encounterFilter, which restricts
        // SOI-encounter detection to that one body. Apse / inclination maneuvers
        // have no target, so null it to detect all high-SOI siblings.
        info.Target = null!;
        OrbitalTransfers.BuildFlightPlan(
            ref flightPlan, info, transferData.Start, transferData.TransferDvVlf,
            out _, out _);
        // BuildFlightPlan leaves the terrain-impact search incremental and nothing
        // ever advances a detached preview plan's frontier, so finish the search here
        // or the preview can omit an impact the created burn's plan will find. 5/8
        // match the limits BuildFlightPlan itself computed the plan with.
        if (flightPlan.ImpactSearchUnresolved)
            flightPlan.ComputeCompleteTrajectory(out _, 5, 8, null,
                resolveImpactsCompletely: true);

        return new OrbitalTransfers.PorkChopEntry(transferData, flightPlan);
    }

    #endregion

    #region Visual Orbit Preview

    /// <summary>Background-drawlist markers (encounter, escape, impact,
    /// closest approach, Ap/Pe) for the preview orbit.</summary>
    private static void DrawOrbitMarkers(Viewport inViewport)
    {
        var uiContext = new Astronomical.UiContext(
            inViewport, _lastSource!, Color.Green,
            TrueAnomaly.Zero, new TrueAnomaly(Math.PI * 2.0),
            ManeuverToolsWindow.GetSelectedTargetOrbiter());
        _lastEntry!.FlightPlan.DrawUi(inViewport, uiContext);
    }

    /// <summary>3D-view post-burn orbit (single-burn or multi-pass), drawn
    /// from Patch_OnPreRender when "Preview Orbit" is on.</summary>
    internal static void RenderOrbitPreview(Viewport inViewport)
    {
        if (_ourBurn != null || _lastSource == null) return;
        if (!_showOrbitPreview) return;

        if (MultiPassUI.HasMultiPassPreview)
        {
            MultiPassUI.Render(inViewport, _lastSource);
            return;
        }

        if (_lastEntry == null) return;

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

        if (key == ManeuverTools.KeyStockCircularizeApoapsis)
            return OrbitManeuvers.ComputeCircularize(orbit, useApoapsis: true, now);

        if (key == ManeuverTools.KeyStockCircularizePeriapsis)
            return OrbitManeuvers.ComputeCircularize(orbit, useApoapsis: false, now);

        return null;
    }

    private static void CleanupStaleBurn()
    {
        if (_ourBurn == null) return;
        if (!_ourBurn.Vehicle.FlightComputer.BurnPlan.TryGetBurn(_ourBurn))
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
    }

    /// <summary>Called from PassCompletionPatch when a multi-pass execution
    /// finishes all passes cleanly (not on user cancel). Auto-disables the
    /// preview toggles since the goal orbit has been reached and the
    /// overlay is no longer informative.</summary>
    internal static void OnMultiPassCompleted()
    {
        _showOrbitPreview = false;
        _showFlightPlanPreview = false;
    }

    #endregion
}

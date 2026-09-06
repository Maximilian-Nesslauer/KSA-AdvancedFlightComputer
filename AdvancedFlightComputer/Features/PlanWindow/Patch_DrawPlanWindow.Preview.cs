using System;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance.Buffers;
using KSA;

namespace AdvancedFlightComputer.Features.PlanWindow;

internal static partial class Patch_DrawPlanWindow
{
    private const string FlightPlanWindowId = "afc-maneuver-flightplan";
    private const string FlightPlanWindowTitle = "MANEUVER FLIGHT PLAN";

    private static float FlightPlanWindowOffsetXPx => 620f * ImGuiHelper.InterfaceScale;
    private static float FlightPlanWindowOffsetYPx => 40f * ImGuiHelper.InterfaceScale;
    private static float FlightPlanWindowWidthPx => 460f * ImGuiHelper.InterfaceScale;
    private static float FlightPlanWindowHeightPx => 620f * ImGuiHelper.InterfaceScale;

    private static void DrawFlightPlanWindow(IViewport inViewport)
    {
        FlightPlan? flightPlan = MultiPassUI.HasMultiPassPreview
            ? MultiPassUI.LastPassFlightPlan
            : _lastEntry?.FlightPlan;
        if (flightPlan == null) return;

        ImGui.SetNextWindowPos(
            inViewport.Position + new float2(FlightPlanWindowOffsetXPx, FlightPlanWindowOffsetYPx),
            ImGuiCond.Appearing, (float2?)null);

        if (!ConsoleStyle.BeginWindow(FlightPlanWindowId, FlightPlanWindowTitle, WindowSignature,
                ref _showFlightPlanPreview,
                new float2(FlightPlanWindowWidthPx, FlightPlanWindowHeightPx),
                ImGuiWindowFlags.NoFocusOnAppearing))
            return;

        try
        {
            ConsoleStyle.BeginBody();
            ConsoleStyle.PushWidgetStyle();
            try
            {
                flightPlan.DrawPatchInfo();
            }
            finally
            {
                ConsoleStyle.PopWidgetStyle();
                ConsoleStyle.EndBody();
            }
            ConsoleStyle.BeginFooter();
            ConsoleStyle.EndFooter();
        }
        finally
        {
            ConsoleStyle.EndWindow();
        }
    }

    private static OrbitalTransfers.PorkChopEntry BuildTransferEntry(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, PlanningBasis basis)
    {
        var transferData = new OrbitalTransfers.TransferData
        {
            Start = maneuver.BurnTime,
            Point = basis.Orbit.GetPointAt(maneuver.BurnTime),
            DeltaVelocityCci = maneuver.DvCci,
            TransferDvVlf = maneuver.DvVlf
        };

        if (basis.IsChained && basis.Patch != null)
        {
            var (chainedPlan, _) = MultiPassForwardChainPlanner.BuildPassFlightPlan(
                source, basis.Patch, maneuver.BurnTime, maneuver.DvVlf);
            return new OrbitalTransfers.PorkChopEntry(transferData, chainedPlan);
        }

        FlightPlan flightPlan = FlightPlan.CreateUninitialized(source.Hash);
        flightPlan.ImpactClearanceMargin = source.BoundingSphereRadiusBody;
        var info = new OrbitalTransfers.TransferInfo(source, source, source, usePorkChopData: false)
        {
            Target = null!
        };
        OrbitalTransfers.BuildFlightPlan(
            ref flightPlan, info, transferData.Start, transferData.TransferDvVlf,
            out _, out _);
        if (flightPlan.ImpactSearchUnresolved)
            flightPlan.ComputeCompleteTrajectory(out _, 5, 8, null,
                resolveImpactsCompletely: true);

        return new OrbitalTransfers.PorkChopEntry(transferData, flightPlan);
    }

    private static void DrawOrbitMarkers(IViewport inViewport)
    {
        var uiContext = new Astronomical.UiContext(
            inViewport, _lastSource!, Color.Green,
            TrueAnomaly.Zero, new TrueAnomaly(Math.PI * 2.0),
            ManeuverToolsWindow.GetSelectedTargetOrbiter());
        _lastEntry!.FlightPlan.DrawUi(inViewport, uiContext, tintDanger: true);
    }

    internal static void RenderOrbitPreview(IViewport inViewport)
    {
        if (!TransferPlanner.ShowPlanWindow)
            return;

        if (_ourBurn != null || _lastSource == null || !_showOrbitPreview)
            return;

        if (MultiPassUI.HasMultiPassPreview)
        {
            MultiPassUI.Render(inViewport, _lastSource);
            return;
        }

        if (_lastEntry == null || _lastEntry.FlightPlan.Patches.Count == 0)
            return;

        FlightPlan flightPlan = _lastEntry.FlightPlan;
        if (flightPlan.Patches[0].Orbit.IsMissingPoints())
        {
            foreach (PatchedConic patch in flightPlan.Patches)
            {
                patch.HidePatch = false;
                MemoryOwner<OrbitPointCce> points = UpdateTaskUtils.GenerateSpacedPoints(patch);
                patch.Orbit.UpdateCachedPoints(points);
            }
        }

        flightPlan.AddLineInstances(inViewport, _lastSource, isActive: true,
            drawVehiclePosition: false, TrueAnomaly.NaN, TrueAnomaly.NaN,
            isPostBurnOrbit: true);
    }
}

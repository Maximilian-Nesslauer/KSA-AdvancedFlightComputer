using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.ImGuiApi;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance.Buffers;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

internal static partial class OrbitToolsPanel
{
    /// <summary>The trajectory after the planned burn: the last pass of a multi-pass preview, or the single burn's.</summary>
    internal static FlightPlan? PreviewFlightPlan => MultiPassUI.HasMultiPassPreview
        ? MultiPassUI.LastPassFlightPlan
        : _lastEntry?.FlightPlan;

    internal static FlightPlan? PlannedFlightPlan => PreviewFlightPlan ?? _ourBurn?.FlightPlan;

    /// <summary>The trajectory that TUNE > DIAG > FLIGHT PLAN draws (FlightPlan.DrawPatchInfo), or null. Nothing refreshes the plan while that view is on screen, so a burn that left the stock plan or whose time has passed no longer counts.</summary>
    internal static FlightPlan? KeptFlightPlan(UniverseTime now)
    {
        if (_ourBurn != null && (_ourBurn.Vehicle.IsDisposed || !_ourBurn.Vehicle.FlightComputer.BurnPlan.TryGetBurn(_ourBurn)))
            _ourBurn = null;
        if (PreviewBurnTime is UniverseTime previewTime && previewTime > now)
            return PreviewFlightPlan;
        return _ourBurn != null && _ourBurn.Time > now ? _ourBurn.FlightPlan : null;
    }

    private static UniverseTime? PreviewBurnTime
    {
        get
        {
            if (!MultiPassUI.HasMultiPassPreview)
                return _lastEntry?.TransferData.Start;
            PassPreview[] passes = MultiPassPreviewCache.PreviewPasses;
            return passes.Length > 0 ? passes[0].BurnTime : null;
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

        // With engine thrust the preview shows the burn arc and markers that stock draws once the burn exists.
        PatchedConic? prePatch = source.FlightPlan.TryFindPatch(maneuver.BurnTime);
        if (prePatch != null && FiniteBurns.HasThrust(FiniteBurns.FullThrottlePhases(source)))
        {
            var (finitePlan, _) = MultiPassForwardChainPlanner.BuildPassFlightPlan(
                source, prePatch, maneuver.BurnTime, maneuver.DvVlf);
            return new OrbitalTransfers.PorkChopEntry(transferData, finitePlan);
        }

        FlightPlan flightPlan = FlightPlan.CreateUninitialized(source.Hash);
        flightPlan.ImpactClearanceMargin = source.BoundingSphereRadiusBody;
        var info = new OrbitalTransfers.TransferInfo(source, source, source, usePorkChopData: false)
        {
            Target = null
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
            OrbitToolForm.GetSelectedTargetOrbiter());
        _lastEntry!.FlightPlan.DrawUi(inViewport, uiContext, tintDanger: true);
    }

    /// <summary>The MAP lines of the planned trajectory, from the CelestialSystem.OnPreRender postfix.</summary>
    internal static void RenderOrbitPreview(IViewport inViewport)
    {
        if (!PreviewVisible)
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
            drawVehiclePosition: false, nextBurnTime: null,
            isPostBurnOrbit: true);
    }
}

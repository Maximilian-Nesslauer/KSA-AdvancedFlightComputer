using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using AdvancedFlightComputer.Features.Transfer;
using Brutal.ImGuiApi;
using Brutal.Logging;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance.Buffers;
using KSA;

namespace AdvancedFlightComputer.Features.Flyby;

// The controls select a request before MultiPass selects the departure time. The readout then uses that selected departure and its propagated trajectory.
internal static class HohmannFlybyUI
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private static readonly string[] ReferenceLabels = { "SURFACE", "CENTER", "ATMOSPHERE" };
    // Same order as FlybySide.
    private static readonly string[] SideLabels =
    {
        "INNER (TOWARD PARENT)", "OUTER (AWAY FROM PARENT)", "NORTH", "SOUTH",
    };

    /// <summary>The FLYBY SIDE label of the side the flyby aims at, for the flyby inset.</summary>
    internal static string SideLabel => SideLabels[(int)_side];

    public static bool Enabled { get; set; }

    private static bool _flybyOn;
    private static FlybyReference _reference = FlybyReference.Surface;
    private static double _inputValueKm = 100.0;
    private static FlybySide _side = FlybySide.Inner;

    private static FlybyTargeting.DepartureSolution? _displayedDeparture;
    private static FlybyPrediction _prediction;
    private static bool _displayingMultiPass;
    private static bool _belowFloor;
    // Readouts are formatted once per cache update, not per frame.
    // The departure dV line is the exception, because stock's refine worker rewrites the selected entry's dV in place after the click, so its baseline is read again every frame and the line is formatted again only when that value moves.
    private static string _approachSpeedText = string.Empty;
    private static string _impactParameterText = string.Empty;
    private static string _departureDvText = string.Empty;
    private static string? _predictedPeText;
    private static double _cachedFlybyDv = double.NaN;
    private static double _cachedStockDv = double.NaN;
    // Propagated periapsis from the target's center, and the floor it is judged against.
    // The requested radius is not enough, because the achieved periapsis comes out of the patched conic propagation and can land below the body even when the input was above it.
    private static double _minFlybyRadius = double.NaN;
    private static FlightPlan? _previewPlan;
    private static bool _previewHadCoveringPatch;
    private static UniverseTime _previewPatchStart;
    private static UniverseTime _previewPatchEnd;
    private static double _previewClearance;
    private static int _previewPhaseSignature;
    private static double _displayTargetRadius;
    private static string? _lastSourceId;

    private static FlybyTargeting.FlybyResult? CachedResult => _displayedDeparture?.Outcome.Result;

    /// <summary>The propagated flyby would hit the body or its atmosphere, so the
    /// departure must not be armed however sane the requested altitude looked.</summary>
    private static bool PredictedFlybyBelowFloor =>
        _prediction.PeriapsisRadius is double radius
        && double.IsFinite(_minFlybyRadius) && radius < _minFlybyRadius;

    internal static void DrawControls(
        Vehicle source, OrbitalTransfers.TransferInfo info)
    {
        if (!Enabled) return;

        if (_lastSourceId != source.Id)
        {
            _lastSourceId = source.Id;
            InvalidateCache();
        }

        try
        {
            DrawBody(source, info);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("flyby-draw-controls:" + ex.GetType().Name,
                $"[AFC] HohmannFlybyUI.DrawControls: vehicle='{source.Id}' " +
                $"target='{(info.Target as Astronomical)?.Id ?? "?"}': {ex}");
        }
    }

    private static void DrawBody(
        Vehicle source, OrbitalTransfers.TransferInfo info)
    {
        bool prevOn = _flybyOn;
        ConsoleUi.CheckboxRow("TARGET FLYBY PERIAPSIS".AsSpan(), "AfcFlybyOn".AsSpan(), ref _flybyOn,
            "Aims the transfer past the destination at the periapsis set below, instead of at the centre of the body. Off: the stock aim at the centre.".AsSpan());
        if (_flybyOn != prevOn)
        {
            InvalidateCache();
            if (DebugConfig.Flyby)
                DefaultCategory.Log.Debug(string.Format(Inv,
                    "[AFC] HohmannFlybyUI: flyby toggled {0} for vehicle='{1}' target='{2}'.",
                    _flybyOn ? "ON" : "OFF", source.Id,
                    (info.Target as Astronomical)?.Id ?? "?"));
        }

        if (!_flybyOn) return;

        if (info.Target is not IParentBody target)
        {
            ConsoleUi.WarningWrapped("Flyby targeting needs a celestial body (not a vehicle).");
            return;
        }

        ClampReference(target);
        DrawValueInput();

        double peRadius = FlybyTargeting.ResolvePeriapsisRadius(target, _inputValueKm * 1000.0, _reference);
        double minRadius = FlybyTargeting.MinFlybyRadius(target);
        _belowFloor = peRadius < minRadius;

        ConsoleUi.Readout("FLYBY PERIAPSIS (FROM CENTER)".AsSpan(),
            OrbitToolForm.FormatDistance(peRadius).AsSpan());

        if (_belowFloor)
        {
            // The floor is the terrain ceiling on an airless body and the top of the atmosphere on one with an atmosphere, so it is not "the surface".
            ConsoleUi.WarningWrapped(string.Format(Inv,
                "Periapsis is below the safe flyby floor ({0} from center). Raise the altitude.",
                OrbitToolForm.FormatDistance(minRadius)));
            return;
        }
    }

    internal static int ChangedInFold
        => FlybyRequested ? (_reference != FlybyReference.Surface ? 1 : 0) + (_side != FlybySide.Inner ? 1 : 0) : 0;

    internal static bool FoldRowsShown => FlybyRequested && TransferSession.Info?.Target is IParentBody;

    internal static void DrawFoldRows()
    {
        if (TransferSession.Info?.Target is not IParentBody target)
            return;
        DrawReferenceDropdown(target);
        DrawSidePicker();
    }

    // A body without an atmosphere offers no atmosphere reference, so a choice made for another body falls back to the surface.
    private static void ClampReference(IParentBody target)
    {
        if (FlybyTargeting.HasAtmosphere(target) || _reference != FlybyReference.Atmosphere)
            return;
        _reference = FlybyReference.Surface;
        InvalidateCache();
    }

    private static void DrawReferenceDropdown(IParentBody target)
    {
        bool hasAtmo = FlybyTargeting.HasAtmosphere(target);
        ClampReference(target);

        int refIdx = (int)_reference;
        ConsoleUi.BeginRow("REFERENCE".AsSpan());
        if (ConsoleUi.BeginCombo("AfcFlybyRef".AsSpan(), ReferenceLabels[refIdx]))
        {
            for (int i = 0; i < ReferenceLabels.Length; i++)
            {
                if (i == (int)FlybyReference.Atmosphere && !hasAtmo) continue;
                bool selected = i == refIdx;
                if (ImGui.Selectable(ReferenceLabels[i], selected))
                {
                    var newRef = (FlybyReference)i;
                    if (newRef != _reference) { _reference = newRef; InvalidateCache(); }
                }
            }
            ConsoleWidgets.EndComboControl();
        }
        ConsoleUi.EndRow();
    }

    private static void DrawValueInput()
    {
        double prev = _inputValueKm;
        ConsoleUi.InputDoubleRow(
            _reference == FlybyReference.Center ? "RADIUS".AsSpan() : "ALTITUDE".AsSpan(),
            "##AfcFlybyAlt"u8, ref _inputValueKm, 10.0, 100.0, "%.1f"u8, unit: "km");
        if (_inputValueKm < 0.0) _inputValueKm = 0.0;
        if (Math.Abs(_inputValueKm - prev) > 1e-9) InvalidateCache();
    }

    /// <summary>A side whose axis is nearly parallel to the approach cannot be
    /// reached, because no perpendicular offset puts the periapsis there, so it is
    /// drawn disabled rather than silently aimed at.</summary>
    private static void DrawSidePicker()
    {
        ConsoleUi.BeginRow("FLYBY SIDE".AsSpan());
        if (ConsoleUi.BeginCombo("AfcFlybySide".AsSpan(), SideLabels[(int)_side]))
        {
            for (int i = 0; i < SideLabels.Length; i++)
            {
                var candidate = (FlybySide)i;
                bool reachable = _displayedDeparture == null || _displayedDeparture.Outcome.CanReach(candidate);
                if (!reachable) ImGui.BeginDisabled();
                if (ImGui.Selectable(SideLabels[i], candidate == _side) && candidate != _side)
                {
                    _side = candidate;
                    InvalidateCache();
                }
                if (!reachable) ImGui.EndDisabled();
            }
            ConsoleWidgets.EndComboControl();
        }
        ConsoleUi.EndRow();
    }

    private static void DrawResult()
    {
        if (CachedResult == null)
        {
            ConsoleUi.WarningWrapped("Flyby retarget failed for this geometry; the stock burn would still impact. Try a different transfer window or side.");
            return;
        }

        ConsoleUi.Readout("APPROACH SPEED".AsSpan(), _approachSpeedText.AsSpan());
        ConsoleUi.Readout("IMPACT PARAMETER".AsSpan(), _impactParameterText.AsSpan());
        ConsoleUi.Readout("DEPARTURE DELTA-V".AsSpan(), _departureDvText.AsSpan());
        if (_predictedPeText != null)
            ConsoleUi.Readout("PREDICTED PERIAPSIS".AsSpan(), _predictedPeText.AsSpan());

        if (IsCacheExpired())
        {
            ConsoleUi.WarningWrapped("The departure time has passed. CALCULATE again to pick a new window.");
            return;
        }

        if (PredictedFlybyBelowFloor)
        {
            ConsoleUi.WarningWrapped(string.Format(Inv,
                "Propagated periapsis is below the safe floor ({0} from center): " +
                "this trajectory impacts. Raise the altitude or try the other side.",
                OrbitToolForm.FormatDistance(_minFlybyRadius)));
            return;
        }

        if (_prediction.Status != FlybyPredictionStatus.Available)
        {
            // Advisory rather than a block, because the propagation is best effort and has been seen to miss an encounter that a later recompute resolves, so refusing here could strand a valid plan.
            ConsoleUi.WarningWrapped(_prediction.Reason);
            return;
        }

        ConsoleUi.Muted((_displayingMultiPass
            ? "The passes use this flyby departure."
            : "CREATE FLYBY DEPARTURE plans this departure.").AsSpan());
    }

    #region Cache

    internal static void ShowSingleDeparture(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry, OrbitalTransfers.TransferInfo info)
    {
        if (!FlybyRequested || _belowFloor || info.Target is not IParentBody target) return;
        if (!TryGetRequest(target, out double radius, out FlybySide side)) return;
        var key = FlybyTargeting.CaptureDepartureKey(
            source, info.Target, entry.TransferData.Start, entry.TransferData.Transit, radius, side);
        var solution = FlybyTargeting.GetDeparture(key);
        double floor = FlybyTargeting.MinFlybyRadius(target);
        PatchedConic? patch = solution.Outcome.Result is { } result
            ? source.FlightPlan.TryFindPatch(result.BurnTime) : null;
        if (ReferenceEquals(solution, _displayedDeparture) && !_displayingMultiPass
            && _previewClearance == source.BoundingSphereRadiusBody
            && _previewPhaseSignature == FiniteBurns.PhaseSignature(source)
            && _displayTargetRadius == target.MeanRadius && _minFlybyRadius == floor
            && (patch != null) == _previewHadCoveringPatch
            && (patch == null || (patch.StartTime == _previewPatchStart && patch.EndTime == _previewPatchEnd))) return;
        _displayedDeparture = solution;
        _displayingMultiPass = false;
        _previewHadCoveringPatch = patch != null;
        _previewPatchStart = patch?.StartTime ?? default;
        _previewPatchEnd = patch?.EndTime ?? default;
        _previewClearance = source.BoundingSphereRadiusBody;
        _previewPhaseSignature = FiniteBurns.PhaseSignature(source);
        _displayTargetRadius = target.MeanRadius;
        _minFlybyRadius = floor;
        _prediction = BuildPreview(source, info.Target, target, solution.Outcome.Result, out _previewPlan);
        FormatReadouts(entry);
    }

    internal static void ShowMultiPassDeparture(
        FlybyTargeting.DepartureSolution? solution, FlightPlan? plan,
        IParentBody target, OrbitalTransfers.PorkChopEntry entry)
    {
        double floor = FlybyTargeting.MinFlybyRadius(target);
        if (ReferenceEquals(solution, _displayedDeparture)
            && ReferenceEquals(plan, _previewPlan) && _displayingMultiPass
            && _displayTargetRadius == target.MeanRadius && _minFlybyRadius == floor) return;
        _displayedDeparture = solution;
        _displayingMultiPass = true;
        _previewPlan = plan;
        _displayTargetRadius = target.MeanRadius;
        _minFlybyRadius = floor;
        _prediction = solution?.Outcome.Result == null
            ? new(FlybyPredictionStatus.NoDeparture)
            : plan == null ? new(FlybyPredictionStatus.PropagationFailed)
            : FlybyPrediction.FromPlan(plan, target);
        FormatReadouts(entry);
    }

    internal static void DrawSelectedResult(OrbitalTransfers.PorkChopEntry entry)
    {
        if (!FlybyRequested || _belowFloor) return;
        RefreshDepartureDvText(entry);
        DrawResult();
    }

    private static void FormatReadouts(OrbitalTransfers.PorkChopEntry entry)
    {
        _predictedPeText = null;
        _cachedFlybyDv = double.NaN;
        _cachedStockDv = double.NaN;
        if (CachedResult is not FlybyTargeting.FlybyResult r) return;
        _approachSpeedText = string.Format(Inv, "{0:F1} m/s", r.VInfMs);
        _impactParameterText = OrbitToolForm.FormatDistance(r.ImpactParameterMeters);
        _predictedPeText = _prediction.PeriapsisRadius is double radius
            && _displayedDeparture?.Key.Target is IParentBody target
            ? OrbitToolForm.FormatDistance(radius - target.MeanRadius)
            : "No prediction";
        _cachedFlybyDv = r.DvVlf.Length();
        RefreshDepartureDvText(entry);
    }

    private static void RefreshDepartureDvText(OrbitalTransfers.PorkChopEntry entry)
    {
        if (double.IsNaN(_cachedFlybyDv)) return;
        double stockDv = entry.TransferData.TransferDvVlf.Length();
        if (stockDv == _cachedStockDv) return;
        _cachedStockDv = stockDv;
        _departureDvText = string.Format(Inv,
            "{0:F1} m/s ({1:+0.0;-0.0} vs impact)", _cachedFlybyDv, _cachedFlybyDv - stockDv);
    }

    // FlightPlan.TryAddFiniteBurn and ComputeCompleteTrajectory provide the same propagation used by stock previews. Without engine thrust the burn stays impulsive through FlightPlan.CalculateBurnPatch.
    internal static FlybyPrediction BuildPreview(
        Vehicle source, IOrbiter targetOrbiter, IParentBody target,
        FlybyTargeting.FlybyResult? result, out FlightPlan? preview)
    {
        preview = null;
        if (result is not FlybyTargeting.FlybyResult departure)
            return new(FlybyPredictionStatus.NoDeparture);

        PatchedConic? prePatch = source.FlightPlan.TryFindPatch(departure.BurnTime);
        if (prePatch == null)
            return new(FlybyPredictionStatus.NoCoveringPatch);

        try
        {
            FlightPlan plan = FlightPlan.CreateUninitialized(source.Hash);
            plan.ImpactClearanceMargin = source.BoundingSphereRadiusBody;
            double3 dvVlf = RefineDeparture(source, prePatch, departure);
            bool errors = false;
            if (FiniteBurns.TryAddFiniteBurn(plan, prePatch, departure.BurnTime, dvVlf,
                    FiniteBurns.FullThrottlePhases(source), out bool impacted) == null)
            {
                UniverseTime timeSincePe = prePatch.Orbit.GetTimeSincePeriapsisThisOrbit(departure.BurnTime);
                plan.Patches.Add(plan.CalculateBurnPatch(
                    prePatch, timeSincePe, dvVlf, departure.BurnTime));
                plan.ComputeCompleteTrajectory(out errors, 8, 8, targetOrbiter, resolveImpactsCompletely: true);
            }
            else if (!impacted)
            {
                // The patch limit counts every patch, so a burn split into several pieces keeps the same number of coast patches after it.
                plan.ComputeCompleteTrajectory(out errors, plan.Patches.Count + 7, 8, targetOrbiter,
                    resolveImpactsCompletely: true);
            }
            if (errors) return new(FlybyPredictionStatus.PropagationFailed);
            preview = plan;
            return FlybyPrediction.FromPlan(plan, target);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("flyby-preview:" + ex.GetType().Name,
                $"[AFC] Flyby preview: source='{source.Id}' target='{target.Id}' burn={departure.BurnTime.Seconds():F1}s: {ex}");
            return new(FlybyPredictionStatus.PropagationFailed);
        }
    }

    /// <summary>The flyby departure corrected for the finite burn, as stock refines
    /// its own transfer burn with FiniteBurnRefiner.RefineTransfer. The preview and
    /// the burn that Create queues both use it, so they show the same trajectory.</summary>
    internal static double3 RefineDeparture(
        Vehicle source, PatchedConic prePatch, FlybyTargeting.FlybyResult departure)
        => FiniteBurns.Refine(prePatch.Orbit, departure.BurnTime, departure.DvVlf,
            FiniteBurns.FullThrottlePhases(source), FiniteBurnGoal.Transfer(departure.Arrival)).DvVlf;

    /// <summary>True when the single burn flyby overlay should draw, which needs a
    /// flyby armed with a propagated plan, a result of the transfer session for the
    /// cached vehicle, and SHOW IN WORLD on. The multi pass overlay is checked by the caller
    /// and takes precedence, because with N &gt; 1 its own preview already contains
    /// the retargeted departure.</summary>
    public static bool ShouldRenderPreview(out Vehicle? source)
    {
        source = null;
        if (!Enabled || !_flybyOn || _belowFloor || _displayingMultiPass) return false;
        if (CachedResult == null || _previewPlan == null) return false;
        // An elapsed departure cannot be flown, so it is not drawn.
        if (IsCacheExpired()) return false;
        if (!TransferSession.ShowMap) return false;
        // The session clears its result on a source or destination change, so a cached plan of an earlier destination never paints over the live trajectory.
        if (!TransferSession.Calculated) return false;
        // Like the selected transfer, an uncommitted flyby departure draws only while TRANSFER is on screen.
        if (!TransferSession.PageVisible) return false;

        source = TransferSession.Source;
        return source != null && ReferenceEquals(source, _displayedDeparture?.Key.Source);
    }

    /// <summary>True when the path aimed at the center should not be drawn,
    /// because the trajectory CREATE flies is drawn instead: the flyby departure,
    /// or the passes of <see cref="HohmannMultiPassUI"/>, which never fly the
    /// selected transfer as one burn.</summary>
    public static bool ReplacesCenterAimedPreview()
    {
        if (HohmannMultiPassUI.IsDrawingPassOverlay()) return true;
        if (!FlybyRequested) return false;
        if (HohmannMultiPassUI.HasMultiPassPreview) return true;
        return ShouldRenderPreview(out _);
    }

    /// <summary>The predicted flyby periapsis from the target's center, m, or NaN.</summary>
    internal static double PredictedPeriapsisRadius => _prediction.PeriapsisRadius ?? double.NaN;

    /// <summary>Draws the retargeted flyby trajectory in the 3D view in place of
    /// the path aimed at the center, which would contradict what CREATE flies.</summary>
    public static void RenderPreview(IViewport viewport, Vehicle source)
    {
        FlightPlan? fp = _previewPlan;
        if (fp == null || fp.Patches.Count == 0) return;

        if (fp.Patches[0].Orbit.IsMissingPoints())
        {
            foreach (PatchedConic patch in fp.Patches)
            {
                patch.HidePatch = false;
                MemoryOwner<OrbitPointCce> points = UpdateTaskUtils.GenerateSpacedPoints(patch);
                patch.Orbit.UpdateCachedPoints(points);
            }
        }

        fp.AddLineInstances(viewport, source, isActive: true,
            drawVehiclePosition: false, nextBurnTime: null,
            isPostBurnOrbit: true);
    }

    /// <summary>True when the cached departure can no longer be flown because its
    /// burn time is in the past. Consumers refuse it, and it does not trigger a
    /// recompute.</summary>
    private static bool IsCacheExpired()
    {
        if (CachedResult is not FlybyTargeting.FlybyResult r) return false;
        return r.BurnTime.Seconds() <= Universe.GetElapsedTime().Seconds();
    }

    private static void InvalidateCache()
    {
        _displayedDeparture = null;
        _displayingMultiPass = false;
        _prediction = default;
        _belowFloor = false;
        _minFlybyRadius = double.NaN;
        _previewPlan = null;
        _previewHadCoveringPatch = false;
        _predictedPeText = null;
        _cachedFlybyDv = double.NaN;
        _cachedStockDv = double.NaN;
    }

    #endregion

    #region Interceptor and multi pass handoff

    /// <summary>True when the user has the flyby option on.</summary>
    public static bool FlybyRequested => Enabled && _flybyOn;

    /// <summary>The retargeted single flyby departure for <paramref name="vehicle"/>
    /// when the flyby is on, valid, above the floor and matches the cached source.
    /// Used by the create interceptor for the single burn path.</summary>
    public static bool TryGetArmed(
        Vehicle vehicle, out FlybyTargeting.FlybyResult result)
    {
        if (_displayingMultiPass) return TryGetSingleDeparture(vehicle, out result);
        result = default;
        if (!FlybyRequested || _belowFloor) return false;
        if (CachedResult is not FlybyTargeting.FlybyResult cached) return false;
        if (!ReferenceEquals(_displayedDeparture?.Key.Source, vehicle)) return false;
        // A past burn time has no patch for Burn.Create, and the interceptor would fall back to the stock burn aimed at the center.
        if (IsCacheExpired()) return false;
        // The propagated trajectory, not the typed altitude, decides whether this is a flyby at all.
        // A preview without an encounter is only advisory, see DrawResult.
        if (PredictedFlybyBelowFloor) return false;
        result = cached;
        return true;
    }

    // A failed split can fall back only to the original single departure, never to its shifted multi-pass input.
    internal static bool TryGetSingleDeparture(Vehicle vehicle, out FlybyTargeting.FlybyResult result)
    {
        result = default;
        if (!FlybyRequested || !TransferSession.Calculated) return false;
        if (!ReferenceEquals(vehicle, TransferSession.Source)) return false;
        var entry = TransferSession.SelectedEntry;
        var info = TransferSession.Info;
        if (entry == null || info?.Target is not IParentBody target) return false;
        if (!TryGetRequest(target, out double radius, out FlybySide side)) return false;
        var key = FlybyTargeting.CaptureDepartureKey(
            vehicle, info.Target, entry.TransferData.Start, entry.TransferData.Transit, radius, side);
        var solution = FlybyTargeting.GetDeparture(key);
        if (solution.Outcome.Result is not { } departure
            || departure.BurnTime <= Universe.GetElapsedTime()) return false;
        FlybyPrediction prediction = BuildPreview(vehicle, info.Target, target, departure, out _);
        if (prediction.PeriapsisRadius is double predicted && predicted < FlybyTargeting.MinFlybyRadius(target))
            return false;
        result = departure;
        return true;
    }

    /// <summary>The current flyby request for <paramref name="target"/>, which is
    /// the periapsis radius from the center and the side, or false when the flyby
    /// is off or invalid. Used by <see cref="HohmannMultiPassUI"/> to bake the flyby
    /// into a multi pass plan input.</summary>
    public static bool TryGetRequest(IParentBody target, out double peRadius, out FlybySide side)
    {
        peRadius = 0.0;
        side = _side;
        if (!FlybyRequested) return false;
        peRadius = FlybyTargeting.ResolvePeriapsisRadius(target, _inputValueKm * 1000.0, _reference);
        if (peRadius < FlybyTargeting.MinFlybyRadius(target)) return false;
        return peRadius > 0.0;
    }

    #endregion

    public static void Reset()
    {
        _flybyOn = false;
        _reference = FlybyReference.Surface;
        _inputValueKm = 100.0;
        _side = FlybySide.Inner;
        _lastSourceId = null;
        ClearPreview();
    }

    internal static void ClearPreview()
    {
        InvalidateCache();
        FlybyTargeting.ResetDepartureCache();
    }
}

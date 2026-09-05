using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.MultiPass;
using Brutal.ImGuiApi;
using Brutal.Logging;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance.Buffers;
using KSA;

namespace AdvancedFlightComputer.Features.Flyby;

/// <summary>
/// Inline "Target flyby periapsis" section inside the stock Transfer Planning
/// window, above the multi pass controls. The retarget math lives in
/// <see cref="FlybyTargeting"/>. This holds the UI state and a result cache keyed
/// on the porkchop entry and the flyby inputs. <see cref="HohmannMultiPassUI"/>
/// reads the live request through <see cref="TryGetRequest"/> to bake the flyby
/// into a multi pass plan, and <see cref="HohmannCreateInterceptor"/> reads the
/// cached solve through <see cref="TryGetArmed"/> to fire a single flyby burn.
/// </summary>
internal static class HohmannFlybyUI
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private static readonly string[] ReferenceLabels = { "Surface", "Center", "Atmosphere" };
    // Same order as FlybySide.
    private static readonly string[] SideLabels =
    {
        "Inner (toward parent)", "Outer (away from parent)", "North", "South",
    };

    public static bool Enabled { get; set; }

    private static bool _flybyOn;
    private static FlybyReference _reference = FlybyReference.Surface;
    private static double _inputValueKm = 100.0;
    private static FlybySide _side = FlybySide.Inner;

    // What the retarget consumes, split into the inputs a user or a porkchop click
    // changes and the departure orbit, which drifts on its own during a burn. A
    // changed input always recomputes, while orbit drift is frozen under thrust,
    // see UpdateCacheIfStale. Vehicle mass is not a key, because the retarget is
    // orbital mechanics only and keying on mass would rebuild the solve per gram
    // burned.
    private readonly record struct FlybyInputs(
        string SourceId,
        string TargetId,
        long StartBucketSec,
        long TransitBucketSec,
        FlybyReference Reference,
        long ValueBucketM,
        FlybySide Side);

    // Periapsis and eccentricity rather than the semi major axis, because on the
    // nearly parabolic departure ellipse this feature creates da/dv is of order
    // 1e6 m per m/s, so integrator jitter would move the SMA by kilometres and bust
    // the cache every frame. Periapsis and eccentricity hold while coasting and
    // still jump once a burn runs.
    private readonly record struct DepartureOrbit(long PeriapsisBucketKm, long EccentricityBucket);

    private static FlybyInputs _cachedInputs;
    private static DepartureOrbit _cachedOrbit;
    private static bool _hasCached;
    private static FlybyTargeting.FlybyOutcome _cachedOutcome;
    private static bool _belowFloor;
    // Readouts are formatted once per cache update, not per frame. The departure
    // dV line is the exception, because stock's refine worker rewrites the selected
    // entry's dV in place after the click, so its baseline is read again every
    // frame and the line is formatted again only when that value moves.
    private static string _approachSpeedText = string.Empty;
    private static string _impactParameterText = string.Empty;
    private static string _departureDvText = string.Empty;
    private static string? _predictedPeText;
    private static double _cachedFlybyDv = double.NaN;
    private static double _cachedStockDv = double.NaN;
    private static double _predictedPeAlt = double.NaN;
    // Propagated periapsis from the target's center, and the floor it is judged
    // against. The requested radius is not enough, because the achieved periapsis
    // comes out of the patched conic propagation and can land below the body even
    // when the input was above it.
    private static double _predictedPeRadius = double.NaN;
    private static double _minFlybyRadius = double.NaN;
    private static bool _previewHasEncounter;
    private static FlightPlan? _previewPlan;
    private static string? _lastSourceId;

    private static FlybyTargeting.FlybyResult? CachedResult => _hasCached ? _cachedOutcome.Result : null;

    /// <summary>The propagated flyby would hit the body or its atmosphere, so the
    /// departure must not be armed however sane the requested altitude looked.</summary>
    private static bool PredictedFlybyBelowFloor =>
        _previewHasEncounter
        && double.IsFinite(_predictedPeRadius) && double.IsFinite(_minFlybyRadius)
        && _predictedPeRadius < _minFlybyRadius;

    /// <summary>Draws the flyby section. <paramref name="entry"/> and
    /// <paramref name="info"/> are the stock selected porkchop entry, resolved by
    /// the calling <see cref="HohmannMultiPassUI"/>.</summary>
    public static void DrawInline(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry,
        OrbitalTransfers.TransferInfo info)
    {
        if (!Enabled) return;

        if (_lastSourceId != source.Id)
        {
            _lastSourceId = source.Id;
            InvalidateCache();
        }

        try
        {
            DrawBody(source, entry, info);
        }
        catch (Exception ex)
        {
            LogHelper.WarnOnce("flyby-draw-inline:" + ex.GetType().Name,
                $"[AFC] HohmannFlybyUI.DrawInline: vehicle='{source.Id}' " +
                $"target='{(info.Target as Astronomical)?.Id ?? "?"}': {ex}");
        }
    }

    private static void DrawBody(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry,
        OrbitalTransfers.TransferInfo info)
    {
        bool prevOn = _flybyOn;
        ConsoleUi.CheckboxRow("TARGET FLYBY PERIAPSIS".AsSpan(), "AfcFlybyOn".AsSpan(), ref _flybyOn);
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

        DrawReferenceDropdown(target);
        DrawValueInput();
        DrawSidePicker();

        double peRadius = FlybyTargeting.ResolvePeriapsisRadius(target, _inputValueKm * 1000.0, _reference);
        double minRadius = FlybyTargeting.MinFlybyRadius(target);
        _belowFloor = peRadius < minRadius;

        ConsoleWidgets.Readout("FLYBY PERIAPSIS (FROM CENTER)".AsSpan(),
            ManeuverToolsWindow.FormatDistance(peRadius).AsSpan());

        if (_belowFloor)
        {
            // The floor is the terrain ceiling on an airless body and the top of
            // the atmosphere on one with an atmosphere, so it is not "the surface".
            ConsoleUi.WarningWrapped(string.Format(Inv,
                "Periapsis is below the safe flyby floor ({0} from center). Raise the altitude.",
                ManeuverToolsWindow.FormatDistance(minRadius)));
            return;
        }

        UpdateCacheIfStale(source, entry, info, target, peRadius);
        RefreshDepartureDvText(entry);
        DrawResult();
    }

    private static void DrawReferenceDropdown(IParentBody target)
    {
        bool hasAtmo = FlybyTargeting.HasAtmosphere(target);
        if (!hasAtmo && _reference == FlybyReference.Atmosphere)
        {
            _reference = FlybyReference.Surface;
            InvalidateCache();
        }

        int refIdx = (int)_reference;
        ConsoleWidgets.BeginRow("REFERENCE".AsSpan());
        if (ConsoleWidgets.BeginComboControl("AfcFlybyRef".AsSpan(),
                ReferenceLabels[refIdx].AsSpan(), pending: false))
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
        ConsoleWidgets.EndRow();
    }

    private static void DrawValueInput()
    {
        double prev = _inputValueKm;
        ConsoleUi.InputDoubleRow(
            _reference == FlybyReference.Center ? "RADIUS (KM)".AsSpan() : "ALTITUDE (KM)".AsSpan(),
            "##AfcFlybyAlt"u8, ref _inputValueKm, 10.0, 100.0, "%.1f"u8);
        if (_inputValueKm < 0.0) _inputValueKm = 0.0;
        if (Math.Abs(_inputValueKm - prev) > 1e-9) InvalidateCache();
    }

    /// <summary>A side whose axis is nearly parallel to the approach cannot be
    /// reached, because no perpendicular offset puts the periapsis there, so it is
    /// drawn disabled rather than silently aimed at.</summary>
    private static void DrawSidePicker()
    {
        ConsoleWidgets.BeginRow("FLYBY SIDE".AsSpan());
        if (ConsoleWidgets.BeginComboControl("AfcFlybySide".AsSpan(),
                SideLabels[(int)_side].AsSpan(), pending: false))
        {
            for (int i = 0; i < SideLabels.Length; i++)
            {
                var candidate = (FlybySide)i;
                bool reachable = !_hasCached || _cachedOutcome.CanReach(candidate);
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
        ConsoleWidgets.EndRow();
    }

    private static void DrawResult()
    {
        if (CachedResult == null)
        {
            ConsoleUi.WarningWrapped("Flyby retarget failed for this geometry; the stock burn would still impact. Try a different transfer window or side.");
            return;
        }

        ConsoleWidgets.Readout("APPROACH SPEED".AsSpan(), _approachSpeedText.AsSpan());
        ConsoleWidgets.Readout("IMPACT PARAMETER".AsSpan(), _impactParameterText.AsSpan());
        ConsoleWidgets.Readout("DEPARTURE DV".AsSpan(), _departureDvText.AsSpan());
        if (_predictedPeText != null)
            ConsoleWidgets.Readout("PREDICTED PERIAPSIS".AsSpan(), _predictedPeText.AsSpan());

        if (IsCacheExpired())
        {
            ConsoleUi.WarningWrapped("Departure time has passed - Re-Calculate to pick a new window.");
            return;
        }

        if (PredictedFlybyBelowFloor)
        {
            ConsoleUi.WarningWrapped(string.Format(Inv,
                "Propagated periapsis is below the safe floor ({0} from center): " +
                "this trajectory impacts. Raise the altitude or try the other side.",
                ManeuverToolsWindow.FormatDistance(_minFlybyRadius)));
            return;
        }

        if (!_previewHasEncounter)
        {
            // Advisory rather than a block, because the propagation is best effort
            // and has been seen to miss an encounter that a later recompute
            // resolves, so refusing here could strand a valid plan.
            ConsoleUi.WarningWrapped("No encounter resolved in the preview; the flyby periapsis could not be confirmed.");
            return;
        }

        ConsoleUi.Positive("Create fires this flyby departure directly.".AsSpan());
    }

    #region Cache

    private static void UpdateCacheIfStale(
        Vehicle source, OrbitalTransfers.PorkChopEntry entry,
        OrbitalTransfers.TransferInfo info, IParentBody target, double peRadius)
    {
        if (info.Target is not IOrbiter targetOrbiter) return;

        var inputs = new FlybyInputs(
            SourceId: source.Id,
            TargetId: (target as Astronomical)?.Id ?? string.Empty,
            StartBucketSec: (long)entry.TransferData.Start.Seconds(),
            TransitBucketSec: (long)entry.TransferData.Transit.Seconds(),
            Reference: _reference,
            ValueBucketM: (long)peRadius,
            Side: _side);
        var orbit = new DepartureOrbit(
            PeriapsisBucketKm: (long)(source.Orbit.Periapsis / 1000.0),
            EccentricityBucket: (long)(source.Orbit.Eccentricity * 10000.0));

        if (_hasCached && inputs == _cachedInputs)
        {
            if (orbit == _cachedOrbit) return;
            // The departure orbit changes every tick under thrust, and a recompute
            // is three Lambert solves plus a preview FlightPlan, so drift is frozen
            // for the burn and refreshed once thrust stops, and the readouts go
            // slightly stale meanwhile. An expired result is not recomputed either,
            // because the same inputs give the same past burn time. TryGetArmed
            // refuses it and DrawResult says so.
            if (IsThrusting(source)) return;
        }

        _cachedOutcome = FlybyTargeting.ComputeFlybyDeparture(
            source, targetOrbiter, entry.TransferData.Start, entry.TransferData.Transit,
            peRadius, _side);
        _cachedInputs = inputs;
        _cachedOrbit = orbit;
        _hasCached = true;
        BuildPreview(source, targetOrbiter, target, _cachedOutcome.Result);
        FormatReadouts(entry);

        if (DebugConfig.Flyby)
            DefaultCategory.Log.Debug(string.Format(Inv,
                "[AFC] HohmannFlybyUI.UpdateCacheIfStale: vehicle='{0}' target='{1}' " +
                "rp={2:F0}m side={3} -> {4} predictedPeAlt={5:F0}m",
                source.Id, inputs.TargetId, peRadius, _side,
                _cachedOutcome.Result == null ? "FAILED" : "ok",
                _predictedPeAlt));
    }

    private static void FormatReadouts(OrbitalTransfers.PorkChopEntry entry)
    {
        _predictedPeText = null;
        _cachedFlybyDv = double.NaN;
        _cachedStockDv = double.NaN;
        if (CachedResult is not FlybyTargeting.FlybyResult r) return;
        _approachSpeedText = string.Format(Inv, "{0:F1} m/s", r.VInfMs);
        _impactParameterText = ManeuverToolsWindow.FormatDistance(r.ImpactParameterMeters);
        _predictedPeText = double.IsNaN(_predictedPeAlt)
            ? null
            : ManeuverToolsWindow.FormatDistance(_predictedPeAlt);
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

    /// <summary>Propagates the flyby departure through the target SOI and keeps the
    /// plan for the predicted periapsis readout and the 3D preview. Leaves the plan
    /// null and the altitude NaN when the departure could not be propagated, and
    /// both consumers degrade quietly. Both NaN branches log unconditionally,
    /// because the readout has been seen to come back NaN on the first compute and
    /// resolve on the next, and which branch was taken is what a log from the game
    /// has to show.</summary>
    private static void BuildPreview(
        Vehicle source, IOrbiter targetOrbiter, IParentBody target,
        FlybyTargeting.FlybyResult? result)
    {
        _previewPlan = null;
        _predictedPeAlt = double.NaN;
        _predictedPeRadius = double.NaN;
        _previewHasEncounter = false;
        _minFlybyRadius = FlybyTargeting.MinFlybyRadius(target);
        if (result == null) return;
        try
        {
            FlybyTargeting.FlybyResult r = result.Value;
            PatchedConic? prePatch = source.FlightPlan.TryFindPatch(r.BurnTime);
            if (prePatch == null)
            {
                DefaultCategory.Log.Warning(string.Format(Inv,
                    "[AFC] HohmannFlybyUI.BuildPreview: no flight-plan patch covers burn time " +
                    "{0:F1}s for vehicle='{1}' (live plan has {2} patches).",
                    r.BurnTime.Seconds(), source.Id, source.FlightPlan.Patches.Count));
                return;
            }

            // Stock FlightPlan machinery, kept local so the flyby does not couple
            // back to the multi pass planner. The target is the encounter filter so
            // its SOI patch is resolved, and 8 patches at order 8 match stock's own
            // porkchop propagation for one short departure.
            UniverseTime timeSincePe = prePatch.Orbit.GetTimeSincePeriapsisThisOrbit(r.BurnTime);
            FlightPlan fp = FlightPlan.CreateUninitialized(source.Hash);
            fp.ImpactClearanceMargin = source.BoundingSphereRadiusBody;
            PatchedConic burnPatch = fp.CalculateBurnPatch(prePatch, timeSincePe, r.DvVlf, r.BurnTime);
            fp.Patches.Add(burnPatch);
            fp.ComputeCompleteTrajectory(out _, 8, 8, targetOrbiter, resolveImpactsCompletely: true);
            _previewPlan = fp;

            // The flyby patch inside the target SOI is hyperbolic, and Orbit.Periapsis
            // still gives a(1-e), so there is no IsBound gate.
            foreach (PatchedConic patch in fp.Patches)
                if (patch.Orbit.Parent?.Id == target.Id)
                {
                    _previewHasEncounter = true;
                    _predictedPeRadius = patch.Orbit.Periapsis;
                    _predictedPeAlt = _predictedPeRadius - target.MeanRadius;
                    return;
                }

            // Bounded by the patch limit above and reached only on a cache miss, so
            // the dump of every patch cannot flood the log.
            DefaultCategory.Log.Warning(string.Format(Inv,
                "[AFC] HohmannFlybyUI.BuildPreview: no patch inside target '{0}' SOI for " +
                "vehicle='{1}' burnTime={2:F1}s; prePatch parent='{3}', preview plan has " +
                "{4} patch(es).",
                target.Id, source.Id, r.BurnTime.Seconds(),
                prePatch.Orbit.Parent?.Id ?? "?", fp.Patches.Count));
            for (int i = 0; i < fp.Patches.Count; i++)
            {
                PatchedConic p = fp.Patches[i];
                DefaultCategory.Log.Warning(string.Format(Inv,
                    "[AFC]   patch[{0}] parent='{1}' {2}->{3} e={4:F4} t={5:F1}..{6:F1}s " +
                    "encounter='{7}'",
                    i, p.Orbit.Parent?.Id ?? "?", p.StartTransition, p.EndTransition,
                    p.Orbit.Eccentricity, p.StartTime.Seconds(), p.EndTime.Seconds(),
                    p.EncounterBody?.Id ?? "-"));
            }
        }
        catch (Exception ex)
        {
            DefaultCategory.Log.Warning($"[AFC] HohmannFlybyUI.BuildPreview: {ex}");
        }
    }

    /// <summary>True when the single burn flyby overlay should draw, which needs a
    /// flyby armed with a propagated plan, stock's plan window showing a Hohmann
    /// for the cached vehicle, and "Preview Selected Transfer" on. The multi pass
    /// overlay is checked by the caller and takes precedence, because with N &gt; 1
    /// its own preview already contains the retargeted departure.</summary>
    public static bool ShouldRenderPreview(out Vehicle? source)
    {
        source = null;
        if (!Enabled || !_flybyOn || _belowFloor) return false;
        if (CachedResult == null || _previewPlan == null) return false;
        // An elapsed departure cannot be flown, so neither draw it nor keep stock's
        // preview hidden for it.
        if (IsCacheExpired()) return false;
        if (!StockPlanner.ShowPlanWindow) return false;
        if (!StockPlanner.DisplaySelectedTransfer) return false;
        // Stock clears _transferCalculated on a source or destination change and
        // once the created burn's time passes, while keeping _selectedEntry. Without
        // this gate the cached plan, which only tracks the source id, would keep
        // painting the previous target's flyby over the live trajectory.
        if (!StockPlanner.TransferCalculated) return false;

        if (StockPlanner.TransferTypeKey != ManeuverTools.ManeuverTools.KeyStockHohmann)
            return false;

        source = StockPlanner.SourceVehicle;
        return source != null && source.Id == _cachedInputs.SourceId;
    }

    /// <summary>True when stock's preview aimed at the center should be skipped
    /// because the trajectory Create will fly is drawn elsewhere. With a flyby armed
    /// that also covers multi pass, because the flyby is baked into its plan input,
    /// so stock's entry is the impact the retarget replaced there too, and
    /// <see cref="HohmannMultiPassUI"/> draws its own final pass.</summary>
    public static bool SuppressesStockTransferPreview()
    {
        if (!FlybyRequested) return false;
        if (HohmannMultiPassUI.HasMultiPassPreview) return true;
        return ShouldRenderPreview(out _);
    }

    /// <summary>Draws the retargeted flyby trajectory in the 3D view in place of
    /// stock's preview aimed at the center, which would contradict what Create flies.</summary>
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
            drawVehiclePosition: false, TrueAnomaly.NaN, TrueAnomaly.NaN,
            isPostBurnOrbit: true);
    }

    private static bool IsThrusting(Vehicle source) =>
        source.FlightComputer.BurnMode == FlightComputerBurnMode.Auto
        || source.GetManualThrottle() > 0f;

    /// <summary>True when the cached departure can no longer be flown because its
    /// burn time is in the past. Consumers refuse it, and it does not trigger a
    /// recompute, see <see cref="UpdateCacheIfStale"/>.</summary>
    private static bool IsCacheExpired()
    {
        if (CachedResult is not FlybyTargeting.FlybyResult r) return false;
        return r.BurnTime.Seconds() <= Universe.GetElapsedTime().Seconds();
    }

    private static void InvalidateCache()
    {
        _hasCached = false;
        _cachedOutcome = default;
        _cachedInputs = default;
        _cachedOrbit = default;
        _belowFloor = false;
        _predictedPeAlt = double.NaN;
        _predictedPeRadius = double.NaN;
        _minFlybyRadius = double.NaN;
        _previewHasEncounter = false;
        _previewPlan = null;
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
        result = default;
        if (!FlybyRequested || _belowFloor) return false;
        if (CachedResult is not FlybyTargeting.FlybyResult cached) return false;
        if (_cachedInputs.SourceId != vehicle.Id) return false;
        // A past burn time has no patch for Burn.Create, and the interceptor would
        // fall back to the stock burn aimed at the center.
        if (IsCacheExpired()) return false;
        // The propagated trajectory, not the typed altitude, decides whether this is
        // a flyby at all. A preview without an encounter is only advisory, see
        // DrawResult.
        if (PredictedFlybyBelowFloor) return false;
        result = cached;
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
        InvalidateCache();
    }
}

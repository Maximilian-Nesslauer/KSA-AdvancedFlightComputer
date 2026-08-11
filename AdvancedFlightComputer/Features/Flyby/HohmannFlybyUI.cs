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
/// Inline "Target flyby periapsis" section drawn inside the stock Transfer
/// Planning window, above the multi-pass controls. When enabled, the Hohmann
/// departure is retargeted so the arrival is a flyby at the chosen periapsis
/// instead of a center-aimed impact, removing the manual impact-to-flyby
/// correction step. Works for moon flybys (same parent) and interplanetary
/// flybys (cross parent); the retarget math lives in <see cref="FlybyTargeting"/>.
///
/// Holds its own UI state and a small result cache keyed on the porkchop entry
/// plus the flyby inputs. <see cref="HohmannMultiPassUI"/> reads the live request
/// (periapsis radius + side) via <see cref="TryGetRequest"/> to bake the flyby
/// into a multi-pass plan; <see cref="MultiPass.HohmannCreateInterceptor"/> reads
/// the cached solve via <see cref="TryGetArmed"/> to fire a single flyby burn
/// when N == 1.
/// </summary>
internal static class HohmannFlybyUI
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private static readonly string[] ReferenceLabels = { "Surface", "Center", "Atmosphere" };
    // Index-aligned with FlybySide.
    private static readonly string[] SideLabels =
    {
        "Inner (toward parent)", "Outer (away from parent)", "North", "South",
    };
    private static readonly float[] SingleColumnWidths = { 0.9f };

    public static bool Enabled { get; set; }

    private static bool _flybyOn;
    private static FlybyReference _reference = FlybyReference.Surface;
    private static double _inputValueKm = 100.0;
    private static FlybySide _side = FlybySide.Inner;

    // Keyed on what the retarget actually consumes: the transfer window, the
    // requested periapsis / side, and the departure orbit. Vehicle mass is NOT a
    // field - the retarget is pure orbital mechanics (state vectors plus mu), so
    // keying on mass would rebuild the whole solve on every gram of propellant
    // drained.
    //
    // The orbit-changed signal is periapsis plus eccentricity, NOT the semi-major
    // axis: on the near-parabolic departure ellipse this feature creates, da/dv is
    // of order 1e6 m per m/s, so millimetre-per-second integrator jitter would move
    // the SMA by kilometres and bust the cache every frame. Periapsis and
    // eccentricity stay put while coasting and still jump hard once a burn runs.
    private readonly record struct FlybyKey(
        string SourceId,
        string TargetId,
        long StartBucketSec,
        long TransitBucketSec,
        FlybyReference Reference,
        long ValueBucketM,
        FlybySide Side,
        long PeriapsisBucketKm,
        long EccentricityBucket);

    private static FlybyKey _cachedKey;
    private static bool _hasCached;
    private static FlybyTargeting.FlybyOutcome _cachedOutcome;
    private static FlybyTargeting.FlybyResult? _cachedResult;
    private static bool _belowFloor;
    private static double _predictedPeAlt = double.NaN;
    // Propagated periapsis measured from the target's center, plus the floor it is
    // judged against. The requested radius alone is not enough: the achieved
    // periapsis comes out of the patched-conic propagation and can land below the
    // body even when the input was above it.
    private static double _predictedPeRadius = double.NaN;
    private static double _minFlybyRadius = double.NaN;
    private static bool _previewHasEncounter;
    private static FlightPlan? _previewPlan;
    private static string? _lastSourceId;

    /// <summary>The propagated flyby would hit the body (or its atmosphere), so the
    /// departure must not be armed however sane the requested altitude looked.</summary>
    private static bool PredictedFlybyBelowFloor =>
        _previewHasEncounter
        && double.IsFinite(_predictedPeRadius) && double.IsFinite(_minFlybyRadius)
        && _predictedPeRadius < _minFlybyRadius;

    /// <summary>Draws the flyby section. <paramref name="entry"/> and
    /// <paramref name="info"/> come from the stock selected porkchop entry, so no
    /// reflection here. Called from <see cref="HohmannMultiPassUI"/> which already
    /// resolved them.</summary>
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
            DefaultCategory.Log.Warning($"[AFC] HohmannFlybyUI.DrawInline: {ex}");
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
        DrawValueInput(target);
        DrawSidePicker();

        double peRadius = FlybyTargeting.ResolvePeriapsisRadius(target, _inputValueKm * 1000.0, _reference);
        double minRadius = FlybyTargeting.MinFlybyRadius(target);
        _belowFloor = peRadius < minRadius;

        ConsoleWidgets.Readout("FLYBY PERIAPSIS (FROM CENTER)".AsSpan(),
            ManeuverToolsWindow.FormatDistance(peRadius).AsSpan());

        if (_belowFloor)
        {
            // The floor is the terrain ceiling on an airless body and the top of the
            // atmosphere on one with an atmosphere, so it is not always "the surface".
            ConsoleUi.WarningWrapped(string.Format(Inv,
                "Periapsis is below the safe flyby floor ({0} from center). Raise the altitude.",
                ManeuverToolsWindow.FormatDistance(minRadius)));
            return;
        }

        UpdateCacheIfStale(source, entry, info, target, peRadius);
        DrawResult(source, entry);
    }

    private static void DrawReferenceDropdown(IParentBody target)
    {
        bool hasAtmo = FlybyTargeting.HasAtmosphere(target);
        // Drop the Atmosphere option for airless bodies (e.g. Luna).
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

    private static void DrawValueInput(IParentBody target)
    {
        // The Reference dropdown directly above states what the value is
        // measured from, so the label only has to name the quantity.
        double prev = _inputValueKm;
        ConsoleUi.InputDoubleRow(
            _reference == FlybyReference.Center ? "RADIUS (KM)".AsSpan() : "ALTITUDE (KM)".AsSpan(),
            "##AfcFlybyAlt"u8, ref _inputValueKm, 10.0, 100.0, "%.1f"u8);
        if (_inputValueKm < 0.0) _inputValueKm = 0.0;
        if (Math.Abs(_inputValueKm - prev) > 1e-9) InvalidateCache();
    }

    /// <summary>Side picker in the target's orbital frame. A side whose axis is
    /// near-parallel to the approach is unreachable (no perpendicular offset puts
    /// the periapsis there), so it is drawn disabled rather than silently aimed at.</summary>
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

    private static void DrawResult(Vehicle source, OrbitalTransfers.PorkChopEntry entry)
    {
        if (!_hasCached || _cachedResult == null)
        {
            ConsoleUi.WarningWrapped("Flyby retarget failed for this geometry; the stock burn would still impact. Try a different transfer window or side.");
            return;
        }

        FlybyTargeting.FlybyResult r = _cachedResult.Value;
        double stockDv = entry.TransferData.TransferDvVlf.Length();
        double flybyDv = r.DvVlf.Length();

        ConsoleWidgets.Readout("APPROACH SPEED".AsSpan(),
            string.Format(Inv, "{0:F1} m/s", r.VInfMs).AsSpan());
        ConsoleWidgets.Readout("IMPACT PARAMETER".AsSpan(),
            ManeuverToolsWindow.FormatDistance(r.ImpactParameterMeters).AsSpan());
        ConsoleWidgets.Readout("DEPARTURE DV".AsSpan(),
            string.Format(Inv, "{0:F1} m/s ({1:+0.0;-0.0} vs impact)", flybyDv, flybyDv - stockDv).AsSpan());
        if (!double.IsNaN(_predictedPeAlt))
            ConsoleWidgets.Readout("PREDICTED PERIAPSIS".AsSpan(),
                ManeuverToolsWindow.FormatDistance(_predictedPeAlt).AsSpan());

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
            // Advisory rather than a block: the propagation is best-effort and has
            // been seen to miss an encounter that a later recompute resolves, so
            // refusing here could strand a valid plan.
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

        var key = new FlybyKey(
            SourceId: source.Id,
            TargetId: (target as Astronomical)?.Id ?? string.Empty,
            StartBucketSec: (long)entry.TransferData.Start.Seconds(),
            TransitBucketSec: (long)entry.TransferData.Transit.Seconds(),
            Reference: _reference,
            ValueBucketM: (long)peRadius,
            Side: _side,
            PeriapsisBucketKm: (long)(source.Orbit.Periapsis / 1000.0),
            EccentricityBucket: (long)(source.Orbit.Eccentricity * 10000.0));

        if (_hasCached && key == _cachedKey) return;

        // Freeze while thrusting: the departure orbit changes every tick, and each
        // recompute is three Lambert solves plus a full preview FlightPlan, so an
        // unfrozen cache rebuilds all of that per frame for the whole burn. Covers
        // both an Auto burn and a manual throttle. The displayed numbers go
        // slightly stale for the duration and refresh once thrust stops.
        //
        // An expired result deliberately does NOT force a recompute here: the
        // recompute would return the same past burn time and loop every frame.
        // TryGetArmed refuses expired results instead, and DrawResult says so.
        if (_hasCached && IsThrusting(source))
            return;

        _cachedOutcome = FlybyTargeting.ComputeFlybyDeparture(
            source, targetOrbiter, entry.TransferData.Start, entry.TransferData.Transit,
            peRadius, _side);
        _cachedResult = _cachedOutcome.Result;
        _cachedKey = key;
        _hasCached = true;
        BuildPreview(source, targetOrbiter, target, _cachedResult);

        if (DebugConfig.Flyby)
            DefaultCategory.Log.Debug(string.Format(Inv,
                "[AFC] HohmannFlybyUI.UpdateCacheIfStale: vehicle='{0}' target='{1}' " +
                "rp={2:F0}m side={3} -> {4} predictedPeAlt={5:F0}m",
                source.Id, key.TargetId, peRadius, _side,
                _cachedResult == null ? "FAILED" : "ok",
                _predictedPeAlt));
    }

    /// <summary>Propagates the flyby departure through the target SOI and stores
    /// the resulting plan for both the predicted-periapsis readout and the 3D
    /// preview overlay. Leaves the plan null / the altitude NaN when the departure
    /// could not be propagated; both consumers degrade quietly.</summary>
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
                // One of the two ways the predicted-periapsis readout comes back NaN.
                // Logged unconditionally, like the catch below: the readout has been
                // seen to come back NaN on the first compute for an input and resolve
                // on the next one, so which branch was taken is the thing worth
                // knowing after an in-game run.
                DefaultCategory.Log.Warning(string.Format(Inv,
                    "[AFC] HohmannFlybyUI.BuildPreview: no flight-plan patch covers burn time " +
                    "{0:F1}s for vehicle='{1}' (live plan has {2} patches).",
                    r.BurnTime.Seconds(), source.Id, source.FlightPlan.Patches.Count));
                return;
            }

            // Propagate with stock FlightPlan machinery (kept local so the flyby
            // feature does not couple back to the multi-pass planner). The target
            // is the encounter filter so the SOI patch is resolved; 8/8 are ample
            // patch / precision limits for one short departure.
            UniverseTime timeSincePe = prePatch.Orbit.GetTimeSincePeriapsisThisOrbit(r.BurnTime);
            FlightPlan fp = FlightPlan.CreateUninitialized(source.Hash);
            fp.ImpactClearanceMargin = source.BoundingSphereRadiusBody;
            PatchedConic burnPatch = fp.CalculateBurnPatch(prePatch, timeSincePe, r.DvVlf, r.BurnTime);
            fp.Patches.Add(burnPatch);
            fp.ComputeCompleteTrajectory(out _, 8, 8, targetOrbiter, resolveImpactsCompletely: true);
            _previewPlan = fp;

            // The flyby patch inside the target SOI is hyperbolic; Orbit.Periapsis
            // still gives a(1-e) = the periapsis radius, so no IsBound gate.
            foreach (PatchedConic patch in fp.Patches)
                if (patch.Orbit.Parent?.Id == target.Id)
                {
                    _previewHasEncounter = true;
                    _predictedPeRadius = patch.Orbit.Periapsis;
                    _predictedPeAlt = _predictedPeRadius - target.MeanRadius;
                    return;
                }

            // The other NaN branch: the propagation resolved no patch inside the
            // target's SOI, so the readout is omitted and the arming guard falls back
            // to a caution. Dump what the propagation did produce rather than guessing
            // at it. Bounded by the 8-patch limit above, and only reached on a cache
            // miss, so this cannot flood the log.
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
            // Display-only, so a failure is non-fatal, but log unconditionally
            // (not gated on the debug flag) so a post-update break is diagnosable.
            DefaultCategory.Log.Warning($"[AFC] HohmannFlybyUI.BuildPreview: {ex}");
        }
    }

    /// <summary>True when the single-burn flyby overlay should draw: a flyby is
    /// armed with a propagated plan, stock's plan window is showing a Hohmann for
    /// the cached vehicle, and the user has "Preview Selected Transfer" on. The
    /// multi-pass overlay is checked by the caller and takes precedence, because
    /// with N &gt; 1 its own preview already contains the flyby-retargeted
    /// departure.</summary>
    public static bool ShouldRenderPreview(out Vehicle? source)
    {
        source = null;
        if (!Enabled || !_flybyOn || _belowFloor) return false;
        if (!_hasCached || _cachedResult == null || _previewPlan == null) return false;
        // An elapsed departure cannot be flown, so neither draw it nor keep stock's
        // preview hidden for it.
        if (IsCacheExpired()) return false;
        if (!StockPlanner.ShowPlanWindow) return false;
        if (!StockPlanner.DisplaySelectedTransfer) return false;
        // Stock clears _transferCalculated on a source / destination change and once
        // the created burn's time passes, while keeping _selectedEntry. Without this
        // the cached plan (which only tracks the source id) would keep painting the
        // previous target's flyby over the live trajectory.
        if (!StockPlanner.TransferCalculated) return false;

        if (StockPlanner.TransferTypeKey != ManeuverTools.ManeuverTools.KeyStockHohmann)
            return false;

        source = StockPlanner.SourceVehicle;
        return source != null && source.Id == _cachedKey.SourceId;
    }

    /// <summary>True when stock's center-aimed preview should be skipped because the
    /// trajectory Create will fly is drawn elsewhere. With a flyby armed that also
    /// covers multi-pass: the flyby is baked into its plan input, so stock's entry is
    /// the impact the retarget replaced there too, and
    /// <see cref="HohmannMultiPassUI"/> draws its own final pass instead of
    /// delegating it to stock.</summary>
    public static bool SuppressesStockTransferPreview()
    {
        if (!FlybyRequested) return false;
        if (HohmannMultiPassUI.HasMultiPassPreview) return true;
        return ShouldRenderPreview(out _);
    }

    /// <summary>Draws the retargeted flyby trajectory in the 3D view. Stock's own
    /// "Preview Selected Transfer" overlay shows the center-aimed (impact) plan it
    /// selected from the porkchop, so without this the preview contradicts what
    /// Create would actually fly.</summary>
    public static void RenderPreview(Viewport viewport, Vehicle source)
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
    /// burn time is in the past (the window elapsed, or the departure already
    /// fired). Consumers refuse it; it deliberately does not trigger a recompute,
    /// see <see cref="UpdateCacheIfStale"/>.</summary>
    private static bool IsCacheExpired()
    {
        if (!_hasCached || _cachedResult == null) return false;
        return _cachedResult.Value.BurnTime.Seconds()
               <= Universe.GetElapsedTime().Seconds();
    }

    private static void InvalidateCache()
    {
        _hasCached = false;
        _cachedResult = null;
        _cachedOutcome = default;
        _cachedKey = default;
        _belowFloor = false;
        _predictedPeAlt = double.NaN;
        _predictedPeRadius = double.NaN;
        _minFlybyRadius = double.NaN;
        _previewHasEncounter = false;
        _previewPlan = null;
    }

    #endregion

    #region Interceptor / coupling handoff

    /// <summary>True when the user has the flyby option on. Multi-pass and the
    /// create interceptor gate the flyby retarget on this.</summary>
    public static bool FlybyRequested => Enabled && _flybyOn;

    /// <summary>Returns the retargeted single flyby departure for
    /// <paramref name="vehicle"/> when the flyby is on, valid, above the surface
    /// and matches the cached source. Used by the create interceptor for the
    /// N == 1 path.</summary>
    public static bool TryGetArmed(
        Vehicle vehicle, out FlybyTargeting.FlybyResult result)
    {
        result = default;
        if (!FlybyRequested || _belowFloor) return false;
        if (!_hasCached || _cachedResult == null) return false;
        if (_cachedKey.SourceId != vehicle.Id) return false;
        // Never hand a past burn time to Burn.Create: no patch covers it, and the
        // interceptor would fall back to the stock impact-aimed burn.
        if (IsCacheExpired()) return false;
        // The propagated trajectory, not the typed altitude, decides whether this is
        // a flyby at all. A no-encounter preview is only advisory (see DrawResult).
        if (PredictedFlybyBelowFloor) return false;
        result = _cachedResult.Value;
        return true;
    }

    /// <summary>The current flyby request (periapsis radius from center + side)
    /// for <paramref name="target"/>, or false when the flyby is off / invalid.
    /// Used by <see cref="HohmannMultiPassUI"/> to bake the flyby into a
    /// multi-pass plan input.</summary>
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

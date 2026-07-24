using System;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using KSA;
using static AdvancedFlightComputer.Features.ManeuverTools.ManeuverTools;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>Per-frame cache for the two expensive UI computations:
/// <see cref="SequenceBurnState.Analyze"/> and
/// <see cref="ApseBurnPlanner.Plan"/>. Continuous inputs are quantized
/// in the cache keys so per-frame drift does not invalidate.</summary>
internal static class MultiPassPreviewCache
{
    // Rocket equation is logarithmic in mass; 100 kg is fine resolution.
    private const double MassQuantumKg = 100.0;
    private const double SmaQuantumM = 100.0;

    #region SequenceBurnState cache

    private readonly record struct StateKey(
        string VehicleId,
        long MassBucket,
        int ActiveEngineSignature);

    private static SequenceBurnState? _cachedState;
    private static StateKey _cachedStateKey;

    public static SequenceBurnState GetSequenceState(Vehicle source)
    {
        var key = new StateKey(
            source.Id,
            (long)(source.TotalMass / MassQuantumKg),
            ComputeActiveEngineSignature(source));

        if (_cachedState != null && key == _cachedStateKey)
            return _cachedState;

        // Cache miss path: measured. Steady-state hits are unmeasured.
#if DEBUG
        using var _perf = new PerfTracker.Scope("SequenceBurnState.Analyze");
#endif

        _cachedState = SequenceBurnState.Analyze(source);
        _cachedStateKey = key;
        return _cachedState;
    }

    // Catches state changes that leave total mass unchanged: engine.IsActive
    // toggles, sequence.Activated flips, engine reassignments, per-core
    // FlowRule changes, and tank PropellantUseEnabled toggles - each moves
    // the SequenceBurnState result without moving MassBucket.
    private static int ComputeActiveEngineSignature(Vehicle source)
    {
        if (source.Parts == null) return 0;
        var hc = new HashCode();

        ReadOnlySpan<Sequence> sequences = source.Parts.SequenceList.Sequences;
        for (int i = 0; i < sequences.Length; i++)
        {
            hc.Add(sequences[i].Number);
            hc.Add(sequences[i].Activated);
        }

        ReadOnlySpan<Part> parts = source.Parts.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            Span<EngineController> engines = parts[i].Modules.Get<EngineController>();
            for (int e = 0; e < engines.Length; e++)
            {
                hc.Add(parts[i].InstanceId);
                hc.Add(engines[e].IsActive);
                // Only a Combustor carries a player-selected FlowRule; a
                // SolidMotor has none, so it folds in as a constant.
                foreach (RocketCore core in engines[e].Cores)
                    hc.Add(core is Combustor c && c.ResourceManager != null ? (int)c.ResourceManager.FlowRule : -1);
            }
        }

        // ComputeSequenceFuel skips propellant-disabled tanks, so a toggle
        // changes burnable fuel while the vehicle's total mass is unchanged.
        Span<Tank> tanks = source.Parts.Tanks.Modules;
        for (int i = 0; i < tanks.Length; i++)
            hc.Add(tanks[i].PropellantUseEnabled);

        return hc.ToHashCode();
    }

    #endregion

    #region PassPreviewResult cache

    /// <summary>Cache key. Quantized in <see cref="From"/> so per-frame
    /// drift on continuous fields does not bust the cache. BurnTime is
    /// intentionally absent (advances every frame). Intent-side fields
    /// (UseDescendingNode etc.) are included because UI toggles for
    /// plane-change types can leave the dV bucket unchanged - e.g.
    /// AN vs DN on a near-circular orbit has identical speed.</summary>
    private readonly record struct PreviewKey(
        string TypeKey,
        string VehicleId,
        int PassCount,
        SplitMode Mode,
        long DvBucket,
        long SmaBucket,
        long MassBucket,
        bool UseDescendingNode,
        long TargetIncMilliRad,
        OrbitManeuvers.InclinationReference Reference,
        string TargetId)
    {
        public static PreviewKey From(
            Vehicle source, string typeKey, int passCount,
            SplitMode mode, double totalDv)
        {
            IOrbiter? target = ManeuverToolsWindow.GetSelectedTargetOrbiter();
            return new(
                typeKey,
                source.Id,
                passCount,
                mode,
                (long)totalDv,
                (long)(source.Orbit.SemiMajorAxis / SmaQuantumM),
                (long)(source.TotalMass / MassQuantumKg),
                ManeuverToolsWindow.UseDescendingNode,
                (long)(ManeuverToolsWindow.TargetInclinationRad * 1000.0),
                ManeuverToolsWindow.InclinationRef,
                target?.Id ?? string.Empty);
        }
    }

    private static PassPreviewResult? _cachedPreview;
    private static PassAllocation[]? _cachedAllocations;
    private static double _cachedAllocationsSum;
    private static PreviewKey _cachedPreviewKey;
    private static bool _hasPreviewKey;

    public static bool HasPreview =>
        _cachedPreview is { Passes.Length: > 0 } && _hasPreviewKey;

    public static string? PreviewSourceId =>
        _hasPreviewKey ? _cachedPreviewKey.VehicleId : null;

    public static PassPreview[] PreviewPasses =>
        _cachedPreview?.Passes ?? Array.Empty<PassPreview>();

    public static bool LastPreviewFailed => _cachedPreview?.Failed ?? false;
    public static string? LastPreviewFailureReason => _cachedPreview?.FailureReason;

    /// <summary>Sum of DvCapacityMs from the most recent allocation;
    /// NaN if nothing has been cached yet.</summary>
    public static double CachedAllocationsSum =>
        _cachedAllocations != null ? _cachedAllocationsSum : double.NaN;

    /// <summary>Recomputes preview when the cache key changes.</summary>
    public static void UpdatePreviewIfStale(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, string typeKey,
        int passCount, SplitMode splitMode, SequenceBurnState state, double totalDv)
    {
        var key = PreviewKey.From(source, typeKey, passCount, splitMode, totalDv);

        if (_hasPreviewKey && _cachedPreview != null && key == _cachedPreviewKey)
            return;

        // Mid-burn state is intrinsically unstable (orbit + mass drift
        // every physics tick). Freeze the cache while engines fire.
        // Pass completion feeds a lower passCount (remaining = total -
        // PassIndex), busting the key; mode transitions alone do not
        // change the key. Allow the initial build so a user who opens
        // the window mid-burn still sees a (slightly stale) preview.
        if (_hasPreviewKey && _cachedPreview != null
            && source.FlightComputer.BurnMode == FlightComputerBurnMode.Auto)
            return;

        // Cache miss path: times Splitter + per-type planner together.
#if DEBUG
        using var _perf = new PerfTracker.Scope("MultiPassPreviewCache.Plan");
#endif

        PassAllocation[] allocations = Splitter.Allocate(totalDv, passCount, splitMode, state);
        PassPreviewResult result = PlanForType(
            source, maneuver, typeKey, allocations, Universe.GetElapsedSimTime());

        _cachedPreview = result;
        _cachedAllocations = allocations;
        _cachedAllocationsSum = Splitter.SumDvCapacityMs(allocations);
        _cachedPreviewKey = key;
        _hasPreviewKey = true;
    }

    /// <summary>Per-type planner dispatch for the preview chain. Apse
    /// types feed ApseBurnPlanner; inclination types route to the
    /// PlaneChangeBurnPlanner so the per-pass node, rotation axis and
    /// dV->angle math match what execution will actually do. Without
    /// this dispatch, plane-change previews would walk through
    /// ApseBurnPlanner and re-apply the full single-burn dV direction
    /// at each apoapsis - dropping SMA every pass because the original
    /// vector carries a retrograde component.</summary>
    private static PassPreviewResult PlanForType(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, string typeKey,
        PassAllocation[] allocations, SimTime now)
    {
        if (typeKey == KeySetApoapsis)
            return ApseBurnPlanner.Plan(source, maneuver.DvVlf, TrueAnomaly.Zero, allocations, now);
        if (typeKey == KeySetPeriapsis)
            return ApseBurnPlanner.Plan(source, maneuver.DvVlf, new TrueAnomaly(Math.PI), allocations, now);
        if (typeKey == KeyMatchInclination)
        {
            Orbit? target = ManeuverToolsWindow.GetSelectedTargetOrbit();
            if (target == null)
                return new PassPreviewResult(System.Array.Empty<PassPreview>(), Failed: true,
                    "no target selected");
            return PlaneChangeBurnPlanner.PlanForMatch(
                source, target, ManeuverToolsWindow.UseDescendingNode, allocations, now);
        }
        if (typeKey == KeySetInclination)
        {
            return PlaneChangeBurnPlanner.PlanForSet(
                source, ManeuverToolsWindow.TargetInclinationRad,
                ManeuverToolsWindow.InclinationRef,
                ManeuverToolsWindow.UseDescendingNode, allocations, now);
        }
        // Circularize at the chosen apse is mechanically an apse burn: a
        // tangential kick that leaves the burn-radius apse invariant and
        // moves the opposite apse toward it. burnTa matches the burn point.
        if (typeKey == KeyStockCircularizeApoapsis)
            return ApseBurnPlanner.Plan(source, maneuver.DvVlf, new TrueAnomaly(Math.PI), allocations, now);
        if (typeKey == KeyStockCircularizePeriapsis)
            return ApseBurnPlanner.Plan(source, maneuver.DvVlf, TrueAnomaly.Zero, allocations, now);
        return new PassPreviewResult(System.Array.Empty<PassPreview>(), Failed: true,
            $"no planner for typeKey '{typeKey}'");
    }

    public static void ClearPreview()
    {
        _cachedPreview = null;
        _cachedAllocations = null;
        _cachedAllocationsSum = 0.0;
        _cachedPreviewKey = default;
        _hasPreviewKey = false;
    }

    public static void Invalidate() => _hasPreviewKey = false;

    #endregion

    public static void Reset()
    {
        ClearPreview();
        _cachedState = null;
        _cachedStateKey = default;
    }
}

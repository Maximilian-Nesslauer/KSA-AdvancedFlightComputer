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

    // Catches state changes that don't move total mass: engine.IsActive
    // toggles, sequence.Activated flips, engine reassignments.
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
            }
        }
        return hc.ToHashCode();
    }

    #endregion

    #region PassPreviewResult cache

    /// <summary>Cache key. Quantized in <see cref="From"/> so per-frame
    /// drift on continuous fields does not bust the cache. BurnTime is
    /// intentionally absent (advances every frame).</summary>
    private readonly record struct PreviewKey(
        string TypeKey,
        string VehicleId,
        int PassCount,
        SplitMode Mode,
        long DvBucket,
        long SmaBucket,
        long MassBucket)
    {
        public static PreviewKey From(
            Vehicle source, string typeKey, int passCount,
            SplitMode mode, double totalDv) => new(
                typeKey,
                source.Id,
                passCount,
                mode,
                (long)totalDv,
                (long)(source.Orbit.SemiMajorAxis / SmaQuantumM),
                (long)(source.TotalMass / MassQuantumKg));
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

        // Cache miss path: times Splitter + ApseBurnPlanner together.
#if DEBUG
        using var _perf = new PerfTracker.Scope("MultiPassPreviewCache.Plan");
#endif

        TrueAnomaly burnTa = typeKey == KeySetApoapsis
            ? TrueAnomaly.Zero
            : new TrueAnomaly(Math.PI);

        PassAllocation[] allocations = Splitter.Allocate(totalDv, passCount, splitMode, state);
        PassPreviewResult result = ApseBurnPlanner.Plan(
            source, maneuver.DvVlf, burnTa, allocations,
            Universe.GetElapsedSimTime());

        _cachedPreview = result;
        _cachedAllocations = allocations;
        _cachedAllocationsSum = Splitter.SumDvCapacityMs(allocations);
        _cachedPreviewKey = key;
        _hasPreviewKey = true;
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

using System;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.RcsTranslation;
using KSA;
using static AdvancedFlightComputer.Features.ManeuverTools.ManeuverTools;

namespace AdvancedFlightComputer.Features.MultiPass;

// Cache sequence performance and planner previews using quantized inputs to limit recalculation during small changes in the orbit.
internal static class MultiPassPreviewCache
{
    // Mass is grouped in 100 kg intervals because the rocket equation varies logarithmically with mass.
    private const double MassQuantumKg = 100.0;
    private const double SmaQuantumM = 100.0;

    #region SequenceBurnState cache

    private readonly record struct StateKey(
        Vehicle Source,
        long MassBucket,
        int ActiveEngineSignature);

    private static SequenceBurnState? _cachedState;
    private static StateKey _cachedStateKey;

    public static SequenceBurnState GetSequenceState(Vehicle source)
    {
        var key = new StateKey(
            source,
            (long)(source.TotalMass / MassQuantumKg),
            ComputeActiveEngineSignature(source));

        if (_cachedState != null && key == _cachedStateKey)
            return _cachedState;

        _cachedState = SequenceBurnState.Analyze(source);
        _cachedStateKey = key;
        return _cachedState;
    }

    // Sequence assignments, decoupler settings, engine settings, and tank permissions can change performance without changing total mass.
    // Include these settings even when stock ignores one in a particular sequence.
    private static int ComputeActiveEngineSignature(Vehicle source)
    {
        if (source.Parts == null) return 0;
        var hc = new HashCode();
        hc.Add(source.Parts);
        hc.Add(source.Parts.SequenceList.ActiveSequence);

        ReadOnlySpan<Sequence> sequences = source.Parts.SequenceList.Sequences;
        for (int i = 0; i < sequences.Length; i++)
        {
            hc.Add(sequences[i].Number);
            hc.Add(sequences[i].Activated);
            hc.Add((int)sequences[i].Environment);
        }

        ReadOnlySpan<Part> parts = source.Parts.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            hc.Add(parts[i].InstanceId);
            foreach (ISequenced module in parts[i].GetSubtreeSequencedModules())
            {
                hc.Add(module.Sequence);
                if (module is Decoupler decoupler) hc.Add(decoupler.IsEnabled);
            }
            Span<EngineController> engines = parts[i].Modules.Get<EngineController>();
            for (int e = 0; e < engines.Length; e++)
            {
                hc.Add(parts[i].InstanceId);
                hc.Add(engines[e].IsActive);
                // Only a liquid combustor has a FlowRule that the player can change.
                foreach (RocketCore core in engines[e].Cores)
                    hc.Add(core is Combustor c && c.ResourceManager != null ? (int)c.ResourceManager.FlowRule : -1);
            }
        }

        // Stock excludes tanks with PropellantUseEnabled disabled even though their fuel still contributes to vehicle mass.
        Span<Tank> tanks = source.Parts.Tanks.Modules;
        for (int i = 0; i < tanks.Length; i++)
            hc.Add(tanks[i].PropellantUseEnabled);

        return hc.ToHashCode();
    }

    #endregion

    #region PassPreviewResult cache

    // BurnTime is omitted because it advances each frame.
    // Keep node and inclination inputs in the key because they can change the maneuver without changing its delta v magnitude.
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
        string TargetId,
        int SequenceSignature)
    {
        public static PreviewKey From(
            Vehicle source, string typeKey, int passCount,
            SplitMode mode, double totalDv, SequenceBurnState state)
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
                target?.Id ?? string.Empty,
                ComputeSequenceSignature(state));
        }

        // Exclude orbit, mass, delta v, and sequence performance changes when comparing intent during thrust.
        public PreviewKey WithoutDrift() =>
            this with { DvBucket = 0, SmaBucket = 0, MassBucket = 0, SequenceSignature = 0 };
    }

    private static int ComputeSequenceSignature(SequenceBurnState state)
    {
        var hash = new HashCode();
        hash.Add(state.HasUsableEngines);
        for (int i = 0; i < state.Sequences.Count; i++) hash.Add(state.Sequences[i]);
        return hash.ToHashCode();
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

    // Return NaN until an allocation has been cached.
    public static double CachedAllocationsSum =>
        _cachedAllocations != null ? _cachedAllocationsSum : double.NaN;

    public static void UpdatePreviewIfStale(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, string typeKey,
        int passCount, SplitMode splitMode, SequenceBurnState state, double totalDv)
    {
        var key = PreviewKey.From(source, typeKey, passCount, splitMode, totalDv, state);

        if (_hasPreviewKey && _cachedPreview != null && key == _cachedPreviewKey)
            return;

        if (_hasPreviewKey && _cachedPreview != null
            && ShouldFreezeForThrust(source, key.WithoutDrift() == _cachedPreviewKey.WithoutDrift()))
            return;

        PassAllocation[] allocations = Splitter.Allocate(totalDv, passCount, splitMode, state);
        PassPreviewResult result = PlanForType(
            source, maneuver, typeKey, allocations, Universe.GetElapsedTime());

        _cachedPreview = result;
        _cachedAllocations = allocations;
        _cachedAllocationsSum = Splitter.SumDvCapacityMs(allocations);
        _cachedPreviewKey = key;
        _hasPreviewKey = true;
    }

    // Use this only when a preview exists and the comparison excludes inputs that drift during thrust.
    // A failed RCS driver can leave IsActive set until cancellation.
    // Manual engine thrust does not freeze the preview.
    // Target altitude is represented only through drift inputs, so altitude changes also stay frozen during thrust.
    internal static bool ShouldFreezeForThrust(Vehicle source, bool sameIntent) =>
        (source.FlightComputer.BurnMode == FlightComputerBurnMode.Auto || RcsExecutor.IsActive(source))
        && sameIntent;

    // Use the same planner for preview and execution.
    // Plane changes need their own planner because repeating the original burn vector would also change orbital energy.
    private static PassPreviewResult PlanForType(
        Vehicle source, OrbitManeuvers.ManeuverResult maneuver, string typeKey,
        PassAllocation[] allocations, UniverseTime now)
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
        // Circularization is a tangential burn that preserves the chosen apsis and moves the opposite apsis toward it.
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

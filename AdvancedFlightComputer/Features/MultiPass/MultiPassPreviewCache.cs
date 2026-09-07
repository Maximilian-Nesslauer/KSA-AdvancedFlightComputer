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
        int PerformanceInputSignature);

    private static SequenceBurnState? _cachedState;
    private static StateKey _cachedStateKey;

    public static SequenceBurnState GetSequenceState(Vehicle source)
    {
        var key = new StateKey(
            source,
            (long)(source.TotalMass / MassQuantumKg),
            ComputePerformanceInputSignature(source));

        if (_cachedState != null && key == _cachedStateKey)
            return _cachedState;

        _cachedState = SequenceBurnState.Analyze(source);
        _cachedStateKey = key;
        return _cachedState;
    }

    // SequencePerformanceList.Recompute reads live store contents and engine design data in addition to the vehicle mass.
    // Keep the same mass quantum for each store so ordinary propellant drain does not rebuild the snapshot every frame.
    private static int ComputePerformanceInputSignature(Vehicle source)
    {
        if (source.Parts == null) return 0;
        var hc = new HashCode();
        hc.Add(source.Parts);
        hc.Add(source.Parts.SequenceList.ActiveSequence);

        AddSequenceInputs(source.Parts, ref hc);
        AddPartInputs(source.Parts, ref hc);
        AddMoleInputs(source.Parts, ref hc);
        AddTankInputs(source.Parts, ref hc);
        return hc.ToHashCode();
    }

    private static void AddSequenceInputs(PartTree tree, ref HashCode hc)
    {
        ReadOnlySpan<Sequence> sequences = tree.SequenceList.Sequences;
        hc.Add(sequences.Length);
        for (int i = 0; i < sequences.Length; i++)
        {
            hc.Add(sequences[i].Number);
            hc.Add(sequences[i].Activated);
            hc.Add((int)sequences[i].Environment);
        }
    }

    private static void AddPartInputs(PartTree tree, ref HashCode hc)
    {
        ReadOnlySpan<Part> parts = tree.Parts;
        hc.Add(parts.Length);
        for (int i = 0; i < parts.Length; i++)
        {
            Part part = parts[i];
            hc.Add(part.InstanceId);
            hc.Add(part.SequenceOrder);
            hc.Add(part.InertMass?.MassPropertiesAsmb.Props.Mass ?? 0f);

            ReadOnlySpan<Part> subParts = part.SubParts;
            for (int s = 0; s < subParts.Length; s++)
            {
                hc.Add(subParts[s].InstanceId);
                hc.Add(subParts[s].InertMass?.MassPropertiesAsmb.Props.Mass ?? 0f);
            }

            foreach (ISequenced module in part.GetSubtreeSequencedModules())
            {
                hc.Add(module);
                hc.Add(module.Sequence);
                if (module is Decoupler decoupler)
                {
                    hc.Add(decoupler.IsEnabled);
                    hc.Add(decoupler.Connector.Connection);
                }
            }

            Span<EngineController> engines = part.Modules.Get<EngineController>();
            for (int e = 0; e < engines.Length; e++)
                AddEngineInputs(tree, engines[e], ref hc);
        }
    }

    private static void AddMoleInputs(PartTree tree, ref HashCode hc)
    {
        Span<Mole> moles = tree.Moles.Modules;
        ReadOnlySpan<MoleState> states = tree.Moles.States;
        hc.Add(moles.Length);
        for (int i = 0; i < moles.Length; i++)
        {
            Mole mole = moles[i];
            hc.Add(mole);
            hc.Add(mole.SubstancePhase);
            hc.Add(mole.ContainerVolume);
            hc.Add((long)(states[mole.StatesIdx].Mass / MassQuantumKg));
        }
    }

    private static void AddTankInputs(PartTree tree, ref HashCode hc)
    {
        Span<Tank> tanks = tree.Tanks.Modules;
        hc.Add(tanks.Length);
        for (int i = 0; i < tanks.Length; i++)
        {
            hc.Add(tanks[i]);
            hc.Add(tanks[i].PropellantUseEnabled);
            hc.Add(tanks[i].Moles.Count);
            foreach (Mole mole in tanks[i].Moles)
                hc.Add(mole);
        }
    }

    private static void AddEngineInputs(PartTree tree, EngineController engine, ref HashCode hc)
    {
        hc.Add(engine);
        hc.Add(engine.IsActive);
        hc.Add(engine.Cores.Length);
        foreach (RocketCore core in engine.Cores)
        {
            hc.Add(core);
            hc.Add(core.Reaction);
            hc.Add(core.MinimumThrottle);
            hc.Add(core.MinimumPulseTime);
            AddCoreInputs(core, ref hc);
            AddNozzleInputs(core, ref hc);
        }
    }

    private static void AddCoreInputs(RocketCore core, ref HashCode hc)
    {
        if (core is Combustor combustor)
        {
            hc.Add(combustor.Config.Lut);
            hc.Add(combustor.Config.CombustionPressureMax);
            hc.Add(combustor.Config.ThermalEfficiency);
            AddReactants(combustor.DesiredMix, ref hc);
            AddConsumptionOrder(combustor.ResourceManager, ref hc);
            return;
        }

        if (core is not SolidMotor solid) return;
        hc.Add(solid.Lut);
        hc.Add(solid.ThermalEfficiency);
        hc.Add(solid.BurnRate.CoefficientMPerS);
        hc.Add(solid.BurnRate.Exponent);
        hc.Add(solid.Propellant);
        hc.Add(solid.AuthoredChamberPressure);
        hc.Add(solid.ManualAreaRatio);
        hc.Add(solid.AreaRatio);
        hc.Add(solid.PeakChamberPressure);
        hc.Add(solid.Stack);
        hc.Add(solid.Stack.IsValid);
        hc.Add(solid.Stack.Segments.Length);
        foreach (SolidGrainSegment segment in solid.Stack.Segments)
        {
            hc.Add(segment);
            hc.Add(segment.Geometry);
            hc.Add(segment.CasingInnerRadius);
            hc.Add(segment.Length);
            hc.Add(segment.GrainVolume);
            hc.Add(segment.UnburnableGrainMass);
            hc.Add(segment.Grain);
        }
    }

    private static void AddReactants(ReactantMix mix, ref HashCode hc)
    {
        ReadOnlySpan<Reactant> reactants = mix.Reactants;
        hc.Add(reactants.Length);
        for (int i = 0; i < reactants.Length; i++)
        {
            hc.Add(reactants[i].SubstancePhase);
            hc.Add(reactants[i].MassFraction);
        }
    }

    private static void AddConsumptionOrder(ResourceManager? manager, ref HashCode hc)
    {
        hc.Add(manager);
        if (manager == null) return;
        hc.Add((int)manager.FlowRule);
        Tank[][]? levels = manager.ConsumptionOrder;
        hc.Add(levels?.Length ?? -1);
        if (levels == null) return;
        for (int i = 0; i < levels.Length; i++)
        {
            Tank[]? level = levels[i];
            hc.Add(level?.Length ?? -1);
            if (level == null) continue;
            for (int j = 0; j < level.Length; j++)
                hc.Add(level[j]);
        }
    }

    // The active sequence reads live nozzle performance and direction, but physics rewrites both values each solver step.
    // Sample that state only when another key input changes. The per-mole mass buckets set the ordinary drain cadence.
    private static void AddNozzleInputs(RocketCore core, ref HashCode hc)
    {
        RocketNozzle[] nozzles = core.Rocket.Nozzles;
        hc.Add(nozzles.Length);
        foreach (RocketNozzle nozzle in nozzles)
        {
            hc.Add(nozzle);
            hc.Add(nozzle.GetType());
            hc.Add(nozzle.ThroatArea);

            if (nozzle is DeLavalNozzle liquid)
                AddNozzleConfig(liquid.Config, ref hc);
            else if (nozzle is SolidMotorNozzle solid)
            {
                AddNozzleConfig(solid.Config, ref hc);
                hc.Add(solid.AreaRatioMultiplier);
                hc.Add(solid.TwoPhaseEfficiency);
            }
        }
    }

    private static void AddNozzleConfig(DeLavalNozzleConfig config, ref HashCode hc)
    {
        hc.Add(config.ThroatArea);
        hc.Add(config.ExitArea);
        hc.Add(config.FlowEfficiency);
        hc.Add(config.ExpansionEfficiency);
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

    // Clear only cache entries that can keep this vehicle and its part graph reachable.
    public static void OnVehicleDisposed(Vehicle vehicle)
    {
        if (ReferenceEquals(_cachedStateKey.Source, vehicle))
        {
            _cachedState = null;
            _cachedStateKey = default;
        }
        if (_hasPreviewKey && _cachedPreviewKey.VehicleId == vehicle.Id)
            ClearPreview();
    }
}

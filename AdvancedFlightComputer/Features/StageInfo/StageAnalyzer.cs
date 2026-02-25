using System;
using System.Collections.Generic;
using Brutal.Logging;
using CommunityToolkit.HighPerformance.Buffers;
using KSA;

namespace AdvancedFlightComputer.Features.StageInfo;

public record struct StageBurnInfo
{
    public int StageNumber;
    public bool IsActiveStage;
    public float DeltaV;
    public float BurnTime;
    public float Thrust;
    public float ExhaustVelocity;
    public float Isp;
    public float StartMass;
    public float EndMass;
    public float FuelMass;
    public float MaxFuelMass;
    public float FuelFraction;
    public float MassFlowRate;
    public float Twr;
    public float JettisonedMass;
    public int EngineCount;
}

public record struct VehicleBurnAnalysis
{
    public List<StageBurnInfo> Stages;
    public float TotalDeltaV;
    public float TotalBurnTime;
}

public record struct BurnStageAllocation
{
    public int StageNumber;
    public float AllocatedDv;
    public float StageTotalDv;
}

public record struct BurnAnalysis
{
    public float RequiredDv;
    public float AvailableDv;
    public float TotalBurnTime;
    public bool IsSufficient;
    public List<BurnStageAllocation> StageAllocations;
}

public static class StageAnalyzer
{
    private const float MinMassFlowRate = 1e-6f;
    private const float MinDryMass = 1f;

    #region Pooled Collections

    private static readonly List<StageBurnInfo> _pooledStages = new();
    private static readonly HashSet<ulong> _pooledJettisonedPartIds = new();
    private static readonly HashSet<ulong> _pooledFuelClaimedTankIds = new();
    private static readonly List<Stage> _pooledSortedStages = new();
    private static readonly List<EngineController> _pooledEngines = new();
    private static readonly List<BurnStageAllocation> _pooledAllocations = new();

    private static readonly Comparison<Stage> StageDescending =
        static (a, b) => b.StageNumber.CompareTo(a.StageNumber);

    public static void ResetPools()
    {
        _pooledStages.Clear();
        _pooledJettisonedPartIds.Clear();
        _pooledFuelClaimedTankIds.Clear();
        _pooledSortedStages.Clear();
        _pooledEngines.Clear();
        _pooledAllocations.Clear();
    }

    #endregion

    /// <summary>
    /// Analyzes all stages of a vehicle and computes per-stage dV, burn time,
    /// TWR, and fuel mass. Includes the currently active (burning) stage.
    /// Walks stages from highest number (first to fire) to lowest.
    ///
    /// Fuel attribution uses each engine's ResourceManager SameStage tank list.
    /// The game pre-computes this list by filtering to tanks whose Part.Stage
    /// matches the engine's Part.Stage. This naturally scopes fuel to the
    /// correct vehicle segment (same side of decoupler boundaries), because
    /// parts on opposite sides of a decoupler are assigned different stage
    /// numbers by the player.
    ///
    /// Since we read the live MoleState.Mass values, the display automatically
    /// reflects cross-stage fuel consumption: if an engine's FlowRule is set
    /// to a non-SameStage variant, it drains tanks in other stages, and those
    /// stages' dV will decrease in real-time even though they aren't firing.
    ///
    /// Uses pooled static collections to avoid GC pressure when called every
    /// frame. Safe because all callers run on the main thread.
    /// </summary>
    public static VehicleBurnAnalysis Analyze(Vehicle vehicle, bool log = false)
    {
        _pooledStages.Clear();
        _pooledJettisonedPartIds.Clear();
        _pooledFuelClaimedTankIds.Clear();

        var result = new VehicleBurnAnalysis
        {
            Stages = _pooledStages,
            TotalDeltaV = 0f,
            TotalBurnTime = 0f
        };

        ReadOnlySpan<Stage> stages = vehicle.Parts.StageList.Stages;
        ReadOnlySpan<MoleState> moleStates = vehicle.Parts.Moles.States;
        float currentMass = vehicle.TotalMass;

        double parentMass = vehicle.Parent?.Mass ?? 0.0;
        double parentRadius = vehicle.Parent?.MeanRadius ?? 1.0;
        float surfaceGravity = (float)(Constants.GRAVITATIONAL_CONSTANT * parentMass / (parentRadius * parentRadius));

        if (log)
        {
            DefaultCategory.Log.Debug(
                $"[AFC] StageAnalyzer: vehicle={vehicle.Id}, totalMass={currentMass:F1} kg, " +
                $"inertMass={vehicle.InertMass:F1} kg, propellant={vehicle.PropellantMass:F1} kg, " +
                $"stages={stages.Length}, surfaceG={surfaceGravity:F3} m/s^2");
        }

        SortStagesDescending(stages);

        for (int si = 0; si < _pooledSortedStages.Count; si++)
        {
            Stage stage = _pooledSortedStages[si];
            bool isActiveStage = false;

            if (stage.Activated)
            {
                if (!HasActiveEngines(stage))
                {
                    if (log)
                        DefaultCategory.Log.Debug(
                            $"[AFC]   Stage {stage.StageNumber}: activated, no active engines, skipping");
                    continue;
                }
                isActiveStage = true;
            }

            // Step 1: Jettison mass from this stage's decouplers.
            // Tanks in fuelClaimedTankIds are treated as empty because an earlier
            // stage's engines will have consumed their propellant by now.
            float jettisonedMass = ComputeJettisonedMass(
                stage, moleStates, _pooledJettisonedPartIds, _pooledFuelClaimedTankIds, log);
            currentMass -= jettisonedMass;

            if (log && jettisonedMass > 0f)
            {
                DefaultCategory.Log.Debug(
                    $"[AFC]   Stage {stage.StageNumber}: jettisoned {jettisonedMass:F1} kg, " +
                    $"mass after jettison={currentMass:F1} kg");
            }

            // Step 2: Find engines
            CollectEngines(stage, isActiveStage, log);

            if (_pooledEngines.Count == 0)
            {
                if (log)
                    DefaultCategory.Log.Debug(
                        $"[AFC]   Stage {stage.StageNumber}: no engines (decoupler-only stage)");
                continue;
            }

            // Step 3: Aggregate thrust and mass flow rate
            float totalThrust = 0f;
            float totalFlowRate = 0f;
            foreach (EngineController engine in _pooledEngines)
            {
                totalThrust += engine.VacuumData.ThrustMax.Length();
                totalFlowRate += engine.VacuumData.MassFlowRateMax;
            }

            if (totalFlowRate < MinMassFlowRate)
            {
                if (log)
                    DefaultCategory.Log.Debug(
                        $"[AFC]   Stage {stage.StageNumber}: zero mass flow rate, skipping");
                continue;
            }

            float ve = totalThrust / totalFlowRate;
            float isp = (float)(ve / Constants.STANDARD_GRAVITY);

            // Step 4: Compute fuel from each engine's SameStage tank list.
            // Reads live MoleState.Mass, so values reflect any cross-stage
            // consumption by currently burning engines.
            // Also computes max fuel mass (tank capacity) for fuel fraction display.
            var (fuelMass, maxFuelMass) = ComputeStageFuel(
                _pooledEngines, _pooledFuelClaimedTankIds, moleStates, log);

            float maxFuel = currentMass - MinDryMass;
            if (fuelMass > maxFuel)
            {
                if (log)
                    DefaultCategory.Log.Warning(
                        $"[AFC]   Stage {stage.StageNumber}: fuel ({fuelMass:F1} kg) clamped " +
                        $"to max burnable mass ({maxFuel:F1} kg)");
                fuelMass = Math.Max(0f, maxFuel);
            }

            // Step 5: Tsiolkovsky rocket equation
            float startMass = currentMass;
            float endMass = currentMass - fuelMass;
            float dv = (endMass > 0f && fuelMass > 0f)
                ? ve * MathF.Log(startMass / endMass)
                : 0f;
            float burnTime = fuelMass / totalFlowRate;
            float twr = (surfaceGravity > 0f)
                ? totalThrust / (startMass * surfaceGravity)
                : 0f;

            float fuelFraction = maxFuelMass > 0f ? fuelMass / maxFuelMass : 0f;

            var info = new StageBurnInfo
            {
                StageNumber = stage.StageNumber,
                IsActiveStage = isActiveStage,
                DeltaV = dv,
                BurnTime = burnTime,
                Thrust = totalThrust,
                ExhaustVelocity = ve,
                Isp = isp,
                StartMass = startMass,
                EndMass = endMass,
                FuelMass = fuelMass,
                MaxFuelMass = maxFuelMass,
                FuelFraction = fuelFraction,
                MassFlowRate = totalFlowRate,
                Twr = twr,
                JettisonedMass = jettisonedMass,
                EngineCount = _pooledEngines.Count
            };

            result.Stages.Add(info);
            result.TotalDeltaV += dv;
            result.TotalBurnTime += burnTime;

            if (log)
            {
                string activeTag = isActiveStage ? " [ACTIVE]" : "";
                DefaultCategory.Log.Debug(
                    $"[AFC]   Stage {stage.StageNumber}{activeTag}: " +
                    $"dV={dv:F1} m/s, burn={burnTime:F1} s, TWR={twr:F2}, " +
                    $"thrust={totalThrust:F0} N, Ve={ve:F1} m/s, Isp={isp:F1} s, " +
                    $"fuel={fuelMass:F1} kg, mass={startMass:F1}->{endMass:F1} kg, " +
                    $"engines={_pooledEngines.Count}");
            }

            currentMass = endMass;
        }

        if (log)
        {
            DefaultCategory.Log.Info(
                $"[AFC] StageAnalyzer result: {result.Stages.Count} burn stages, " +
                $"total dV={result.TotalDeltaV:F1} m/s, total burn={result.TotalBurnTime:F1} s");
        }

        return result;
    }

    private static void SortStagesDescending(ReadOnlySpan<Stage> stages)
    {
        _pooledSortedStages.Clear();
        for (int i = 0; i < stages.Length; i++)
            _pooledSortedStages.Add(stages[i]);
        _pooledSortedStages.Sort(StageDescending);
    }

    /// <summary>
    /// Checks whether an activated stage still has engines with IsActive=true
    /// in the vehicle. Returns false if the stage's parts have been jettisoned
    /// or if the stage has no engines.
    /// </summary>
    private static bool HasActiveEngines(Stage stage)
    {
        ReadOnlySpan<Part> parts = stage.Parts;
        for (int pi = 0; pi < parts.Length; pi++)
        {
            Span<EngineController> engines = parts[pi].Modules.Get<EngineController>();
            for (int ei = 0; ei < engines.Length; ei++)
            {
                if (engines[ei].IsActive)
                    return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Populates _pooledEngines with engine controllers for a stage.
    /// For the active stage: only engines with IsActive=true.
    /// For future stages: all engines (they will activate when the stage fires).
    /// </summary>
    private static void CollectEngines(Stage stage, bool isActiveStage, bool log)
    {
        _pooledEngines.Clear();
        ReadOnlySpan<Part> parts = stage.Parts;

        for (int pi = 0; pi < parts.Length; pi++)
        {
            Span<EngineController> engines = parts[pi].Modules.Get<EngineController>();
            for (int ei = 0; ei < engines.Length; ei++)
            {
                EngineController engine = engines[ei];
                if (isActiveStage && !engine.IsActive)
                    continue;
                _pooledEngines.Add(engine);

                if (log)
                {
                    string activeTag = engine.IsActive ? " [active]" : "";
                    DefaultCategory.Log.Debug(
                        $"[AFC]     Engine '{engine.TemplateId}'{activeTag}: " +
                        $"thrust={engine.VacuumData.ThrustMax.Length():F0} N, " +
                        $"flowRate={engine.VacuumData.MassFlowRateMax:F4} kg/s, " +
                        $"cores={engine.Cores.Length}");
                }
            }
        }
    }

    #region Burn Analysis

    /// <summary>
    /// Allocates a required dV across stages in firing order and computes the
    /// multi-stage burn time. Pure function operating on pre-computed StageBurnInfo
    /// data - no vehicle/part access needed.
    ///
    /// Stages are walked in firing order (highest stage number first, matching
    /// the order in VehicleBurnAnalysis.Stages). For each stage, either the full
    /// stage dV is consumed or just the portion needed to reach the required dV.
    ///
    /// For partially consumed stages, the burn time is computed via inverse
    /// Tsiolkovsky: the fuel mass needed for the partial dV determines the time.
    /// </summary>
    public static BurnAnalysis AnalyzeBurn(VehicleBurnAnalysis analysis, float requiredDv)
    {
        _pooledAllocations.Clear();

        var result = new BurnAnalysis
        {
            RequiredDv = requiredDv,
            AvailableDv = analysis.TotalDeltaV,
            TotalBurnTime = 0f,
            IsSufficient = analysis.TotalDeltaV >= requiredDv,
            StageAllocations = _pooledAllocations
        };

        float dvRemaining = requiredDv;

        foreach (StageBurnInfo stage in analysis.Stages)
        {
            if (dvRemaining <= 0f)
                break;

            if (stage.DeltaV <= 0f || stage.EngineCount == 0)
                continue;

            bool fullyConsumed = dvRemaining >= stage.DeltaV;
            float allocatedDv = fullyConsumed ? stage.DeltaV : dvRemaining;

            float burnTime;
            if (fullyConsumed)
            {
                burnTime = stage.BurnTime;
            }
            else
            {
                // Inverse Tsiolkovsky for partial burn:
                // allocatedDv = Ve * ln(startMass / endMass)
                // endMass = startMass * exp(-allocatedDv / Ve)
                // fuelNeeded = startMass - endMass
                // burnTime = fuelNeeded / flowRate
                float endMass = stage.StartMass * MathF.Exp(-allocatedDv / stage.ExhaustVelocity);
                float fuelNeeded = stage.StartMass - endMass;
                burnTime = (stage.MassFlowRate > MinMassFlowRate)
                    ? fuelNeeded / stage.MassFlowRate
                    : 0f;
            }

            result.StageAllocations.Add(new BurnStageAllocation
            {
                StageNumber = stage.StageNumber,
                AllocatedDv = allocatedDv,
                StageTotalDv = stage.DeltaV
            });

            result.TotalBurnTime += burnTime;
            dvRemaining -= allocatedDv;
        }

        return result;
    }

    #endregion

    #region Fuel Calculation

    /// <summary>
    /// Computes the total propellant available to a stage's engines by walking
    /// each RocketCore's SameStage tank list. This list is pre-computed by the
    /// game's ResourceManager and contains only tanks whose Part.Stage matches
    /// the engine's Part.Stage.
    ///
    /// Returns (currentFuel, maxFuel) where maxFuel is the mass when all tanks
    /// are full, derived from each tank's FilledFraction. Used for the fuel
    /// fraction progress bar in the staging panel.
    ///
    /// Tank IDs are added to fuelClaimedTankIds so jettison mass calculations
    /// can treat them as empty (fuel consumed before decoupler fires).
    /// The set also prevents double-counting if multiple engines share tanks.
    /// </summary>
    private static (float current, float max) ComputeStageFuel(
        List<EngineController> engines,
        HashSet<ulong> fuelClaimedTankIds,
        ReadOnlySpan<MoleState> moleStates,
        bool log)
    {
        float totalCurrent = 0f;
        float totalMax = 0f;

        foreach (EngineController engine in engines)
        {
            foreach (RocketCore core in engine.Cores)
            {
                if (core.ResourceManager == null)
                {
                    if (log)
                        DefaultCategory.Log.Warning(
                            $"[AFC]     Core '{core.TemplateId}' has no ResourceManager");
                    continue;
                }

                var (current, max) = WalkSameStage(
                    core.ResourceManager, fuelClaimedTankIds, moleStates, log);
                totalCurrent += current;
                totalMax += max;
            }
        }

        return (totalCurrent, totalMax);
    }

    /// <summary>
    /// Walks a ResourceManager's FurtherestToNearestNodeSameStage tank list
    /// and sums the current and maximum propellant mass. Only counts each tank
    /// once (dedup via fuelClaimedTankIds).
    ///
    /// Max mass is derived from current mass and FilledFraction:
    ///   maxMass = currentMass / filledFraction
    /// This correctly accounts for substance density without needing to look
    /// up individual mole properties.
    /// </summary>
    private static (float current, float max) WalkSameStage(
        ResourceManager resourceManager,
        HashSet<ulong> fuelClaimedTankIds,
        ReadOnlySpan<MoleState> moleStates,
        bool log)
    {
        float current = 0f;
        float max = 0f;
        MemoryOwner<ArrayPoolResult<Tank>> nodes =
            resourceManager.FurtherestToNearestNodeSameStage;

        if (nodes.Length == 0)
            return (0f, 0f);

        Span<ArrayPoolResult<Tank>> nodeSpan = nodes.Span;
        for (int i = 0; i < nodeSpan.Length; i++)
        {
            Span<Tank> tanks = nodeSpan[i].AsSpan();
            for (int j = 0; j < tanks.Length; j++)
            {
                Tank tank = tanks[j];

                if (!fuelClaimedTankIds.Add(tank.InstanceId))
                    continue;

                float mass = tank.ComputeSubstanceMass(moleStates);
                float filledFraction = tank.FilledFraction(moleStates);
                float maxMass = filledFraction > 0.001f ? mass / filledFraction : 0f;

                current += mass;
                max += maxMass;

                if (log && mass > 0.01f)
                {
                    DefaultCategory.Log.Debug(
                        $"[AFC]       Tank '{tank.InstanceId}' on " +
                        $"'{tank.Parent.FullPart.DisplayName}' " +
                        $"(stage {tank.Parent.FullPart.Stage}): " +
                        $"{mass:F2}/{maxMass:F2} kg ({filledFraction:P0})");
                }
            }
        }

        return (current, max);
    }

    #endregion

    #region Jettisoned Mass Calculation

    /// <summary>
    /// Computes the mass that will be jettisoned when a stage's decouplers fire.
    /// For each decoupler, walks the Part subtree that will separate.
    ///
    /// Tanks in fuelClaimedTankIds are treated as empty: their propellant was
    /// attributed to an earlier stage's engines and will be consumed before
    /// the decoupler fires.
    /// </summary>
    private static float ComputeJettisonedMass(
        Stage stage,
        ReadOnlySpan<MoleState> moleStates,
        HashSet<ulong> jettisonedPartIds,
        HashSet<ulong> fuelClaimedTankIds,
        bool log)
    {
        float totalJettisoned = 0f;

        ReadOnlySpan<Part> parts = stage.Parts;
        for (int pi = 0; pi < parts.Length; pi++)
        {
            Part part = parts[pi];
            if (!part.Modules.HasAny<Decoupler>())
                continue;

            float subtreeMass = CollectSubtreeMass(
                part, moleStates, jettisonedPartIds, fuelClaimedTankIds);
            totalJettisoned += subtreeMass;

            if (log)
            {
                DefaultCategory.Log.Debug(
                    $"[AFC]     Decoupler on '{part.DisplayName}' (id={part.InstanceId}): " +
                    $"jettisons {subtreeMass:F1} kg");
            }
        }

        return totalJettisoned;
    }

    /// <summary>
    /// Recursively collects the mass of a part subtree.
    /// Claimed tanks contribute 0 propellant (fuel already accounted for).
    /// </summary>
    private static float CollectSubtreeMass(
        Part part,
        ReadOnlySpan<MoleState> moleStates,
        HashSet<ulong> jettisonedPartIds,
        HashSet<ulong> fuelClaimedTankIds)
    {
        if (!jettisonedPartIds.Add(part.InstanceId))
            return 0f;

        float mass = ComputePartMass(part, moleStates, fuelClaimedTankIds);

        List<Part> children = part.TreeChildren;
        for (int i = 0; i < children.Count; i++)
        {
            mass += CollectSubtreeMass(
                children[i], moleStates, jettisonedPartIds, fuelClaimedTankIds);
        }

        return mass;
    }

    /// <summary>
    /// Computes the mass of a part and its SubParts.
    /// Inert mass is always counted. Tank propellant is only counted if the
    /// tank has not been claimed by an engine stage.
    /// </summary>
    private static float ComputePartMass(
        Part part,
        ReadOnlySpan<MoleState> moleStates,
        HashSet<ulong> fuelClaimedTankIds)
    {
        float mass = SumComponentMass(part.Modules, moleStates, fuelClaimedTankIds);

        ReadOnlySpan<Part> subParts = part.SubParts;
        for (int i = 0; i < subParts.Length; i++)
            mass += SumComponentMass(subParts[i].Modules, moleStates, fuelClaimedTankIds);

        return mass;
    }

    private static float SumComponentMass(
        ModuleList components,
        ReadOnlySpan<MoleState> moleStates,
        HashSet<ulong> fuelClaimedTankIds)
    {
        float mass = 0f;

        Span<InertMass> inertMasses = components.Get<InertMass>();
        for (int i = 0; i < inertMasses.Length; i++)
            mass += inertMasses[i].MassPropertiesAsmb.Props.Mass;

        Span<Tank> tanks = components.Get<Tank>();
        for (int i = 0; i < tanks.Length; i++)
        {
            if (!fuelClaimedTankIds.Contains(tanks[i].InstanceId))
                mass += tanks[i].ComputeSubstanceMass(moleStates);
        }

        return mass;
    }

    #endregion
}

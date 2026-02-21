using System;
using System.Collections.Generic;
using Brutal.Logging;
using CommunityToolkit.HighPerformance.Buffers;
using KSA;

namespace AdvancedFlightComputer.Staging;

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

public static class StageAnalyzer
{
    private const float MinMassFlowRate = 1e-6f;
    private const float MinDryMass = 1f;

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
    /// </summary>
    public static VehicleBurnAnalysis Analyze(Vehicle vehicle, bool log = false)
    {
        var result = new VehicleBurnAnalysis
        {
            Stages = new List<StageBurnInfo>(),
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

        // Part IDs of parts that will be physically separated by decouplers
        // in earlier (higher-numbered) stages. Used to prevent double-counting
        // in jettison subtree walks.
        var jettisonedPartIds = new HashSet<ulong>();

        // Tank IDs whose propellant has been attributed to an earlier stage's
        // engines. When computing jettison mass, claimed tanks are treated as
        // empty (their fuel was consumed before the decoupler fires).
        var fuelClaimedTankIds = new HashSet<ulong>();

        var stagesByNumber = SortStagesDescending(stages);

        for (int si = 0; si < stagesByNumber.Count; si++)
        {
            Stage stage = stagesByNumber[si];
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
                stage, moleStates, jettisonedPartIds, fuelClaimedTankIds, log);
            currentMass -= jettisonedMass;

            if (log && jettisonedMass > 0f)
            {
                DefaultCategory.Log.Debug(
                    $"[AFC]   Stage {stage.StageNumber}: jettisoned {jettisonedMass:F1} kg, " +
                    $"mass after jettison={currentMass:F1} kg");
            }

            // Step 2: Find engines
            var stageEngines = CollectEngines(stage, isActiveStage, log);

            if (stageEngines.Count == 0)
            {
                if (log)
                    DefaultCategory.Log.Debug(
                        $"[AFC]   Stage {stage.StageNumber}: no engines (decoupler-only stage)");
                continue;
            }

            // Step 3: Aggregate thrust and mass flow rate
            float totalThrust = 0f;
            float totalFlowRate = 0f;
            foreach (EngineController engine in stageEngines)
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
            float fuelMass = ComputeStageFuel(
                stageEngines, fuelClaimedTankIds, moleStates, log);

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
                MassFlowRate = totalFlowRate,
                Twr = twr,
                JettisonedMass = jettisonedMass,
                EngineCount = stageEngines.Count
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
                    $"engines={stageEngines.Count}");
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

    private static List<Stage> SortStagesDescending(ReadOnlySpan<Stage> stages)
    {
        var list = new List<Stage>(stages.Length);
        for (int i = 0; i < stages.Length; i++)
            list.Add(stages[i]);
        list.Sort((a, b) => b.StageNumber.CompareTo(a.StageNumber));
        return list;
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
            Span<EngineController> engines = parts[pi].Components.Get<EngineController>();
            for (int ei = 0; ei < engines.Length; ei++)
            {
                if (engines[ei].IsActive)
                    return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Collects engine controllers for a stage.
    /// For the active stage: only engines with IsActive=true.
    /// For future stages: all engines (they will activate when the stage fires).
    /// </summary>
    private static List<EngineController> CollectEngines(Stage stage, bool isActiveStage, bool log)
    {
        var result = new List<EngineController>();
        ReadOnlySpan<Part> parts = stage.Parts;

        for (int pi = 0; pi < parts.Length; pi++)
        {
            Span<EngineController> engines = parts[pi].Components.Get<EngineController>();
            for (int ei = 0; ei < engines.Length; ei++)
            {
                EngineController engine = engines[ei];
                if (isActiveStage && !engine.IsActive)
                    continue;
                result.Add(engine);

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

        return result;
    }

    #region Fuel Calculation

    /// <summary>
    /// Computes the total propellant available to a stage's engines by walking
    /// each RocketCore's SameStage tank list. This list is pre-computed by the
    /// game's ResourceManager and contains only tanks whose Part.Stage matches
    /// the engine's Part.Stage.
    ///
    /// Tank IDs are added to fuelClaimedTankIds so jettison mass calculations
    /// can treat them as empty (fuel consumed before decoupler fires).
    /// The set also prevents double-counting if multiple engines share tanks.
    /// </summary>
    private static float ComputeStageFuel(
        List<EngineController> engines,
        HashSet<ulong> fuelClaimedTankIds,
        ReadOnlySpan<MoleState> moleStates,
        bool log)
    {
        float totalFuel = 0f;

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

                totalFuel += WalkSameStage(
                    core.ResourceManager, fuelClaimedTankIds, moleStates, log);
            }
        }

        return totalFuel;
    }

    /// <summary>
    /// Walks a ResourceManager's FurtherestToNearestNodeSameStage tank list
    /// and sums the current propellant mass. Only counts each tank once
    /// (dedup via fuelClaimedTankIds).
    /// </summary>
    private static float WalkSameStage(
        ResourceManager resourceManager,
        HashSet<ulong> fuelClaimedTankIds,
        ReadOnlySpan<MoleState> moleStates,
        bool log)
    {
        float fuel = 0f;
        MemoryOwner<ArrayPoolResult<Tank>> nodes =
            resourceManager.FurtherestToNearestNodeSameStage;

        if (nodes.Length == 0)
            return 0f;

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
                fuel += mass;

                if (log && mass > 0.01f)
                {
                    DefaultCategory.Log.Debug(
                        $"[AFC]       Tank '{tank.InstanceId}' on " +
                        $"'{tank.Parent.FullPart.DisplayName}' " +
                        $"(stage {tank.Parent.FullPart.Stage}): {mass:F2} kg");
                }
            }
        }

        return fuel;
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
            if (!part.Components.HasAny<Decoupler>())
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
        float mass = SumComponentMass(part.Components, moleStates, fuelClaimedTankIds);

        ReadOnlySpan<Part> subParts = part.SubParts;
        for (int i = 0; i < subParts.Length; i++)
            mass += SumComponentMass(subParts[i].Components, moleStates, fuelClaimedTankIds);

        return mass;
    }

    private static float SumComponentMass(
        PartComponentList components,
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

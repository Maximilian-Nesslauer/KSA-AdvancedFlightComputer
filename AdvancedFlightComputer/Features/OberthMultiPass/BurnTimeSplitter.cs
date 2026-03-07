using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.StageInfo;
using Brutal.Logging;

namespace AdvancedFlightComputer.Features.OberthMultiPass;

enum SplitMode { BurnTime, EqualDv }

record struct PassAllocation(double DvCapacity, double EstimatedBurnTime);

/// <summary>
/// Converts a total dV magnitude into per-pass capacity allocations.
///
/// BurnTime mode: budget per pass is time (totalBurnTime / passCount).
/// Each stage advances by the time budget; dV per pass varies naturally.
/// Keeps burn arcs uniform across all passes (optimal for Oberth).
///
/// EqualDv mode: budget per pass is dV (totalDv / passCount). Burn time
/// per pass is computed accurately via inverse Tsiolkovsky with accumulated
/// fuel consumption, so later passes show shorter times (vehicle is lighter
/// after earlier passes burned fuel).
///
/// Both modes share SimulatePass, which accepts a typed budget and returns
/// (dvDelivered, timeUsed) from the same mutable stage state arrays.
/// </summary>
static class BurnTimeSplitter
{
    public static List<PassAllocation> ComputeAllocations(
        double totalDv, int passCount, VehicleBurnAnalysis analysis,
        SplitMode mode = SplitMode.BurnTime)
    {
        passCount = Math.Clamp(passCount, 1, 10);
        var result = new List<PassAllocation>(passCount);

        var stages = analysis.Stages;
        BurnAnalysis maneuverBurn = StageAnalyzer.AnalyzeBurn(analysis, (float)totalDv);
        double totalBurnTime = maneuverBurn.TotalBurnTime;

        if (!maneuverBurn.IsSufficient && DebugConfig.OberthMultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] BurnTimeSplitter: vehicle cannot deliver {totalDv:F1} m/s " +
                $"(available: {analysis.TotalDeltaV:F1} m/s), normalization will inflate allocations.");

        int eligibleCount = 0;
        for (int i = 0; i < stages.Count; i++)
        {
            if (stages[i].EngineCount > 0 && stages[i].DeltaV > 0f)
                eligibleCount++;
        }

        // No usable engine data: simple equal split, burn time unknown.
        if (totalBurnTime <= 0.0 || eligibleCount == 0)
        {
            double dvPerPass = totalDv / passCount;
            for (int i = 0; i < passCount; i++)
                result.Add(new PassAllocation(dvPerPass, 0.0));
            return result;
        }

        // Mutable stage state shared by both modes.
        double[] stageStartMass = new double[eligibleCount];
        double[] stageBurnTimeRemaining = new double[eligibleCount];
        double[] stageMassFlowRate = new double[eligibleCount];
        double[] stageExhaustVelocity = new double[eligibleCount];

        int idx = 0;
        for (int i = 0; i < stages.Count; i++)
        {
            StageBurnInfo s = stages[i];
            if (s.EngineCount <= 0 || s.DeltaV <= 0f)
                continue;
            stageStartMass[idx] = s.StartMass;
            stageBurnTimeRemaining[idx] = s.BurnTime;
            stageMassFlowRate[idx] = s.MassFlowRate;
            stageExhaustVelocity[idx] = s.ExhaustVelocity;
            idx++;
        }

        if (mode == SplitMode.EqualDv)
        {
            double dvPerPass = totalDv / passCount;
            for (int pass = 0; pass < passCount; pass++)
            {
                var (_, passTime) = SimulatePass(
                    stageStartMass, stageBurnTimeRemaining,
                    stageMassFlowRate, stageExhaustVelocity,
                    eligibleCount, SplitMode.EqualDv, dvPerPass);
                result.Add(new PassAllocation(dvPerPass, passTime));
            }
            // No normalization: dV is exactly dvPerPass per pass by construction.
            return result;
        }

        // BurnTime mode: equal time budget per pass, dV per pass varies.
        double targetTimePerPass = totalBurnTime / passCount;
        for (int pass = 0; pass < passCount; pass++)
        {
            // Last pass absorbs floating-point remainder.
            double timeBudget = (pass == passCount - 1)
                ? totalBurnTime - targetTimePerPass * (passCount - 1)
                : targetTimePerPass;

            var (passCapacity, actualBurnTime) = SimulatePass(
                stageStartMass, stageBurnTimeRemaining,
                stageMassFlowRate, stageExhaustVelocity,
                eligibleCount, SplitMode.BurnTime, timeBudget);
            result.Add(new PassAllocation(passCapacity, actualBurnTime));
        }

        // Normalize: raw simulation may not sum to exactly totalDv due to
        // accumulated floating-point error. Scale proportionally.
        double totalCap = 0.0;
        for (int i = 0; i < result.Count; i++)
            totalCap += result[i].DvCapacity;

        if (totalCap > 0.001)
        {
            double scale = totalDv / totalCap;
            for (int i = 0; i < result.Count; i++)
                result[i] = new PassAllocation(result[i].DvCapacity * scale, result[i].EstimatedBurnTime);
        }

        return result;
    }

    /// <summary>
    /// Simulates one pass of burning using the shared mutable stage state.
    ///
    /// BurnTime mode: budget is seconds. Forward Tsiolkovsky (time -> dV).
    /// EqualDv mode:  budget is m/s.  Inverse Tsiolkovsky (dV -> time).
    ///
    /// Returns (dvDelivered, timeUsed). Mutates stageStartMass and
    /// stageBurnTimeRemaining so the next pass starts from the correct
    /// post-burn mass state.
    /// </summary>
    private static (double dvDelivered, double timeUsed) SimulatePass(
        double[] stageStartMass, double[] stageBurnTimeRemaining,
        double[] stageMassFlowRate, double[] stageExhaustVelocity,
        int eligibleCount, SplitMode mode, double budget)
    {
        double dvDelivered = 0.0;
        double timeUsed = 0.0;
        double remainingBudget = budget;

        for (int s = 0; s < eligibleCount; s++)
        {
            if (stageBurnTimeRemaining[s] <= 0.0 || remainingBudget <= 0.0)
                continue;

            double startM = stageStartMass[s];
            double endM;
            double fuelBurned;

            if (mode == SplitMode.BurnTime)
            {
                // Time-constrained: forward Tsiolkovsky.
                double burnableTime = Math.Min(stageBurnTimeRemaining[s], remainingBudget);
                fuelBurned = stageMassFlowRate[s] * burnableTime;
                endM = startM - fuelBurned;
            }
            else
            {
                // dV-constrained: inverse Tsiolkovsky.
                // Max dV available from this stage at its current mass state.
                double minEndM = startM - stageMassFlowRate[s] * stageBurnTimeRemaining[s];
                if (minEndM <= 0.0) { stageBurnTimeRemaining[s] = 0.0; continue; }
                double stageMaxDv = stageExhaustVelocity[s] * Math.Log(startM / minEndM);
                double dvFromStage = Math.Min(remainingBudget, stageMaxDv);
                endM = startM * Math.Exp(-dvFromStage / stageExhaustVelocity[s]);
                fuelBurned = startM - endM;
            }

            if (endM <= 0.0)
            {
                // Stage data inconsistent (fuelBurned > startMass). Skip to avoid NaN from Log.
                stageBurnTimeRemaining[s] = 0.0;
                continue;
            }

            double dv = stageExhaustVelocity[s] * Math.Log(startM / endM);
            double burnTime = fuelBurned / stageMassFlowRate[s];

            dvDelivered += dv;
            timeUsed += burnTime;
            remainingBudget -= mode == SplitMode.BurnTime ? burnTime : dv;
            stageStartMass[s] = endM;
            stageBurnTimeRemaining[s] -= burnTime;
        }

        return (dvDelivered, timeUsed);
    }
}

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
/// BurnTime mode: splits totalDv so each pass burns for an equal fraction of
/// total engine firing time, accounting for multi-stage rockets. High-TWR
/// stages contribute more dV per second, so early passes may carry more dV.
/// Keeps burn arcs uniform across all passes (optimal for Oberth).
///
/// EqualDv mode: simple equal split (totalDv / passCount per pass).
/// </summary>
static class BurnTimeSplitter
{
    public static List<PassAllocation> ComputeAllocations(
        double totalDv, int passCount, VehicleBurnAnalysis analysis,
        SplitMode mode = SplitMode.BurnTime)
    {
        passCount = Math.Clamp(passCount, 1, 10);
        var result = new List<PassAllocation>(passCount);

        // Compute burn time for the REQUESTED dV only.

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

        // EqualDv mode or no usable engine data: simple equal split
        if (mode == SplitMode.EqualDv || totalBurnTime <= 0.0 || eligibleCount == 0)
        {
            double dvPerPass = totalDv / passCount;
            float burnTimePerPass = 0.0f;
            if (mode == SplitMode.EqualDv && eligibleCount > 0)
                burnTimePerPass = StageAnalyzer.AnalyzeBurn(analysis, (float)dvPerPass).TotalBurnTime;
            for (int i = 0; i < passCount; i++)
                result.Add(new PassAllocation(dvPerPass, burnTimePerPass));
            return result;
        }

        double targetTimePerPass = totalBurnTime / passCount;

        // Inline mutable stage state (StartMass and BurnTimeRemaining per eligible stage)
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

        for (int pass = 0; pass < passCount; pass++)
        {
            // Last pass absorbs floating-point remainder
            double remainingPassTime = (pass == passCount - 1)
                ? totalBurnTime - targetTimePerPass * (passCount - 1)
                : targetTimePerPass;

            double passCapacity = 0.0;

            for (int s = 0; s < eligibleCount; s++)
            {
                if (stageBurnTimeRemaining[s] <= 0.0)
                    continue;

                double burnableTime = Math.Min(stageBurnTimeRemaining[s], remainingPassTime);
                double fuelBurned = stageMassFlowRate[s] * burnableTime;
                double startM = stageStartMass[s];
                double endM = startM - fuelBurned;

                if (endM <= 0.0)
                {
                    // Stage data inconsistent (fuelBurned > startMass). Skip to avoid NaN from Log.
                    remainingPassTime -= burnableTime;
                    stageBurnTimeRemaining[s] = 0.0;
                    continue;
                }

                passCapacity += stageExhaustVelocity[s] * Math.Log(startM / endM);

                remainingPassTime -= burnableTime;
                stageStartMass[s] -= fuelBurned;
                stageBurnTimeRemaining[s] -= burnableTime;

                if (remainingPassTime <= 0.0)
                    break;
            }

            double actualBurnTime = (pass == passCount - 1)
                ? totalBurnTime - targetTimePerPass * (passCount - 1)
                : targetTimePerPass;
            actualBurnTime -= remainingPassTime;

            result.Add(new PassAllocation(passCapacity, actualBurnTime));
        }
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
}

using System;
using System.Collections.Generic;
using AdvancedFlightComputer.Features.StageInfo;

namespace AdvancedFlightComputer.Features.OberthMultiPass;

record struct PassAllocation(double DvCapacity, double EstimatedBurnTime);

/// <summary>
/// Converts a total dV magnitude into per-pass capacity allocations based on
/// equal burn time, with full multi-stage simulation inline.
///
/// Splits totalDv across passCount passes so each pass burns for an equal
/// fraction of total engine firing time. High-TWR stages contribute more dV
/// per second than low-TWR stages, so early passes may carry more dV. This
/// keeps burn arcs uniform across all passes (optimal for Oberth).
/// </summary>
static class BurnTimeSplitter
{
    public static List<PassAllocation> ComputeAllocations(
        double totalDv, int passCount, VehicleBurnAnalysis analysis)
    {
        passCount = Math.Clamp(passCount, 1, 10);
        var result = new List<PassAllocation>(passCount);

        // Collect eligible stages and sum total burn time
        var stages = analysis.Stages;
        int eligibleCount = 0;
        double totalBurnTime = 0.0;
        for (int i = 0; i < stages.Count; i++)
        {
            if (stages[i].EngineCount > 0 && stages[i].DeltaV > 0f)
            {
                totalBurnTime += stages[i].BurnTime;
                eligibleCount++;
            }
        }

        // If no usable engine data, fall back to equal dV split with zero time
        if (totalBurnTime <= 0.0 || eligibleCount == 0)
        {
            double dvPerPass = totalDv / passCount;
            for (int i = 0; i < passCount; i++)
                result.Add(new PassAllocation(dvPerPass, 0.0));
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

                if (endM > 0.0)
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

        return result;
    }
}

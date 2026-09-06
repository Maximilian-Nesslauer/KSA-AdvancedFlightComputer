using System;

namespace AdvancedFlightComputer.Features.MultiPass;

internal static class Splitter
{
    public const int MaxPasses = 10;

    public static PassAllocation[] Allocate(
        double totalDv, int passCount, SplitMode mode, SequenceBurnState state)
    {
        passCount = Math.Clamp(passCount, 1, MaxPasses);
        if (totalDv <= 0.0) return EqualDvFallback(0.0, passCount);
        if (!state.HasUsableEngines) return EqualDvFallback(totalDv, passCount);

        var drain = new SequenceDrain(state);
        return mode == SplitMode.EqualBurnTime
            ? AllocateByTime(totalDv, passCount, state, drain)
            : AllocateByDv(totalDv, passCount, drain);
    }

    private static PassAllocation[] AllocateByTime(
        double totalDv, int passCount, SequenceBurnState state, SequenceDrain drain)
    {
        double totalBurnTime = drain.ByDv(totalDv).timeUsed;
        double tPerPass = totalBurnTime / passCount;
        drain.Reset(state);
        var result = new PassAllocation[passCount];
        double dvSum = 0.0;
        for (int p = 0; p < passCount; p++)
        {
            // The final pass absorbs any remainder from floating point rounding.
            double tBudget = p == passCount - 1
                ? Math.Max(0.0, totalBurnTime - tPerPass * (passCount - 1))
                : tPerPass;
            var (dvDelivered, timeUsed) = drain.ByTime(tBudget);
            result[p] = new PassAllocation(dvDelivered, timeUsed);
            dvSum += dvDelivered;
        }
        // Do not scale an allocation up to the requested delta v when fuel is insufficient.
        if (dvSum > 1e-9)
        {
            double scale = Math.Min(1.0, totalDv / dvSum);
            for (int p = 0; p < passCount; p++)
                result[p] = result[p] with { DvCapacityMs = result[p].DvCapacityMs * scale };
        }
        return result;
    }

    private static PassAllocation[] AllocateByDv(double totalDv, int passCount, SequenceDrain drain)
    {
        var result = new PassAllocation[passCount];
        double dvPerPass = totalDv / passCount;
        for (int p = 0; p < passCount; p++)
        {
            double dvBudget = p == passCount - 1
                ? Math.Max(0.0, totalDv - dvPerPass * (passCount - 1))
                : dvPerPass;
            var (dvDelivered, timeUsed) = drain.ByDv(dvBudget);
            result[p] = new PassAllocation(dvDelivered, timeUsed);
        }
        return result;
    }

    public static double SumDvCapacityMs(PassAllocation[] allocations)
    {
        double sum = 0.0;
        for (int i = 0; i < allocations.Length; i++) sum += allocations[i].DvCapacityMs;
        return sum;
    }

    // Each call drains the remaining fuel.
    // Later sequence masses already account for jettison, so keep them separate from the current sequence end mass.
    internal sealed class SequenceDrain
    {
        private readonly double[] startMass;
        private readonly double[] fuelRemaining;
        private readonly double[] mDot;
        private readonly double[] vExhaust;

        public SequenceDrain(SequenceBurnState state)
        {
            int count = state.Sequences.Count;
            startMass = new double[count];
            fuelRemaining = new double[count];
            mDot = new double[count];
            vExhaust = new double[count];
            for (int i = 0; i < count; i++)
            {
                mDot[i] = state.Sequences[i].MassFlowKgPerSec;
                vExhaust[i] = state.Sequences[i].ExhaustVelocityMs;
            }
            Reset(state);
        }

        public void Reset(SequenceBurnState state)
        {
            for (int i = 0; i < startMass.Length; i++)
            {
                startMass[i] = state.Sequences[i].StartMassKg;
                fuelRemaining[i] = state.Sequences[i].FuelMassKg;
            }
        }

        public (double dvDelivered, double timeUsed) ByDv(double dvBudget)
        {
            double dvLeft = dvBudget;
            double timeUsed = 0.0;
            double dvDelivered = 0.0;

            for (int s = 0; s < startMass.Length && dvLeft > 0.0; s++)
            {
                if (mDot[s] <= 0.0 || fuelRemaining[s] <= 0.0) continue;

                double m0 = startMass[s];
                double mMin = Math.Max(0.0, m0 - fuelRemaining[s]);
                if (mMin <= 0.0 || m0 <= 0.0) continue;

                double dvStageMax = vExhaust[s] * Math.Log(m0 / mMin);
                double dvFromStage = Math.Min(dvLeft, dvStageMax);
                double mEnd = m0 * Math.Exp(-dvFromStage / vExhaust[s]);
                double fuelBurned = m0 - mEnd;
                double tBurn = fuelBurned / mDot[s];

                startMass[s] = mEnd;
                fuelRemaining[s] -= fuelBurned;
                timeUsed += tBurn;
                dvDelivered += dvFromStage;
                dvLeft -= dvFromStage;
            }

            return (dvDelivered, timeUsed);
        }
        public (double dvDelivered, double timeUsed) ByTime(double tBudget)
        {
            double tLeft = tBudget;
            double timeUsed = 0.0;
            double dvDelivered = 0.0;

            for (int s = 0; s < startMass.Length && tLeft > 0.0; s++)
            {
                if (mDot[s] <= 0.0 || fuelRemaining[s] <= 0.0) continue;

                double m0 = startMass[s];
                double tStageMax = fuelRemaining[s] / mDot[s];
                double tFromStage = Math.Min(tLeft, tStageMax);
                double fuelBurned = mDot[s] * tFromStage;
                double mEnd = m0 - fuelBurned;
                if (mEnd <= 0.0) continue;

                double dv = vExhaust[s] * Math.Log(m0 / mEnd);

                startMass[s] = mEnd;
                fuelRemaining[s] -= fuelBurned;
                timeUsed += tFromStage;
                dvDelivered += dv;
                tLeft -= tFromStage;
            }

            return (dvDelivered, timeUsed);
        }
    }

    private static PassAllocation[] EqualDvFallback(double totalDv, int passCount)
    {
        var result = new PassAllocation[passCount];
        double per = totalDv / passCount;
        for (int i = 0; i < passCount; i++) result[i] = new PassAllocation(per, 0.0);
        return result;
    }
}

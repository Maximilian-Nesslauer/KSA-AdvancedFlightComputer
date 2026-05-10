using System;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Distributes total dV across N passes via per-sequence Tsiolkovsky
/// in firing order. EqualBurnTime equalises burn arc (Oberth-optimal),
/// EqualDv splits dV evenly. SequenceBurnState pre-accounts for
/// decoupler jettison; the Splitter does not re-model it.
/// </summary>
internal static class Splitter
{
    public const int MaxPasses = 10;

    /// <summary>One PassAllocation per pass; passCount clamped to
    /// [1, <see cref="MaxPasses"/>]. Sum of DvCapacityMs equals
    /// totalDv unless the vehicle runs out of fuel.</summary>
    public static PassAllocation[] Allocate(
        double totalDv, int passCount, SplitMode mode, SequenceBurnState state)
    {
        passCount = Math.Clamp(passCount, 1, MaxPasses);

        if (totalDv <= 0.0)
            return EqualDvFallback(0.0, passCount);

        if (!state.HasUsableEngines)
            return EqualDvFallback(totalDv, passCount);

        // Scratch state mutated by passes so each one continues from
        // the previous pass's drained-mass.
        int n = state.Sequences.Count;
        var startMass = new double[n];
        var fuelRemaining = new double[n];
        var mDot = new double[n];
        var vExhaust = new double[n];
        for (int i = 0; i < n; i++)
        {
            var s = state.Sequences[i];
            startMass[i] = s.StartMassKg;
            fuelRemaining[i] = s.FuelMassKg;
            mDot[i] = s.MassFlowKgPerSec;
            vExhaust[i] = s.ExhaustVelocityMs;
        }

        var result = new PassAllocation[passCount];

        if (mode == SplitMode.EqualBurnTime)
        {
            double totalBurnTime = ComputeTimeForDv(totalDv, startMass, fuelRemaining, mDot, vExhaust);
            double tPerPass = totalBurnTime / passCount;

            // Reset: ComputeTimeForDv mutated the scratch arrays.
            for (int i = 0; i < n; i++)
            {
                var s = state.Sequences[i];
                startMass[i] = s.StartMassKg;
                fuelRemaining[i] = s.FuelMassKg;
            }

            double dvSum = 0.0;
            for (int p = 0; p < passCount; p++)
            {
                // Last pass absorbs floating-point remainder.
                double tBudget = (p == passCount - 1)
                    ? Math.Max(0.0, totalBurnTime - tPerPass * (passCount - 1))
                    : tPerPass;

                var (dvDelivered, timeUsed) = SimulatePassByTime(
                    tBudget, startMass, fuelRemaining, mDot, vExhaust);
                result[p] = new PassAllocation(dvDelivered, timeUsed);
                dvSum += dvDelivered;
            }

            // Drift correction clamped to <= 1.0: when fuel is short
            // we report the truth, not an inflated allocation.
            if (dvSum > 1e-9)
            {
                double scale = Math.Min(1.0, totalDv / dvSum);
                for (int p = 0; p < passCount; p++)
                {
                    result[p] = new PassAllocation(
                        result[p].DvCapacityMs * scale,
                        result[p].EstimatedBurnTimeSec);
                }
            }
            return result;
        }

        // EqualDv: store delivered dV (not the budget) so a fuel-short
        // pass shows up in the sum.
        double dvPerPass = totalDv / passCount;
        for (int p = 0; p < passCount; p++)
        {
            double dvBudget = (p == passCount - 1)
                ? Math.Max(0.0, totalDv - dvPerPass * (passCount - 1))
                : dvPerPass;

            var (dvDelivered, timeUsed) = SimulatePassByDv(
                dvBudget, startMass, fuelRemaining, mDot, vExhaust);
            result[p] = new PassAllocation(dvDelivered, timeUsed);
        }
        return result;
    }

    /// <summary>Sum of DvCapacityMs across allocations. Below the
    /// requested totalDv means the vehicle is fuel-short.</summary>
    public static double SumDvCapacityMs(PassAllocation[] allocations)
    {
        double sum = 0.0;
        for (int i = 0; i < allocations.Length; i++)
            sum += allocations[i].DvCapacityMs;
        return sum;
    }

    /// <summary>Inverse Tsiolkovsky walk: how long to deliver
    /// dvBudget. Mutates startMass / fuelRemaining; dvDelivered may
    /// be less than dvBudget if fuel runs out.</summary>
    private static (double dvDelivered, double timeUsed) SimulatePassByDv(
        double dvBudget,
        double[] startMass, double[] fuelRemaining,
        double[] mDot, double[] vExhaust)
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

            // Do NOT propagate mEnd into startMass[j>s]: Analyze
            // already pre-subtracted the intervening decoupler
            // jettison. Overwriting later-sequence start masses with
            // the current mass would re-add jettisoned mass.
        }

        return (dvDelivered, timeUsed);
    }

    /// <summary>Forward Tsiolkovsky walk: dV delivered in tBudget
    /// seconds. Same mutation contract as SimulatePassByDv.</summary>
    private static (double dvDelivered, double timeUsed) SimulatePassByTime(
        double tBudget,
        double[] startMass, double[] fuelRemaining,
        double[] mDot, double[] vExhaust)
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

            // See SimulatePassByDv re: not propagating mEnd to later sequences.
        }

        return (dvDelivered, timeUsed);
    }

    /// <summary>Time to deliver <paramref name="dv"/>. Mutates the
    /// scratch arrays; caller resets them.</summary>
    private static double ComputeTimeForDv(
        double dv,
        double[] startMass, double[] fuelRemaining,
        double[] mDot, double[] vExhaust)
    {
        var (_, t) = SimulatePassByDv(dv, startMass, fuelRemaining, mDot, vExhaust);
        return t;
    }

    private static PassAllocation[] EqualDvFallback(double totalDv, int passCount)
    {
        var r = new PassAllocation[passCount];
        double per = passCount > 0 ? totalDv / passCount : 0.0;
        for (int i = 0; i < passCount; i++)
            r[i] = new PassAllocation(per, 0.0);
        return r;
    }
}

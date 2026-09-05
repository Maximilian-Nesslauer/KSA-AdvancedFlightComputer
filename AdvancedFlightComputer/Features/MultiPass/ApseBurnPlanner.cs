using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

// A tangential kick at an apsis preserves the line of apsides.
internal static class ApseBurnPlanner
{
    public static PassPreviewResult Plan(
        Vehicle source,
        double3 totalDvVlf,
        TrueAnomaly burnTa,
        PassAllocation[] allocations,
        UniverseTime now) =>
        Plan(source, totalDvVlf, burnTa, allocations, now, false);

    public static PassPreviewResult Plan(
        Vehicle source,
        double3 totalDvVlf,
        TrueAnomaly burnTa,
        PassAllocation[] allocations,
        UniverseTime now, bool execution)
    {
        double3 dvDir = totalDvVlf.NormalizeOrZero();
        if (dvDir.LengthSquared() < 0.5)
            return new PassPreviewResult(System.Array.Empty<PassPreview>(), Failed: true,
                FailureReason: "zero dV direction");

        return MultiPassForwardChainPlanner.PlanForwardChain(source, allocations, now,
            (orbit, dvCap, earliestTime) =>
            {
                if (orbit.TimeOfTrueAnomaly(burnTa, earliestTime) is not UniverseTime burnTime)
                    return null;
                return new PassStep(burnTime, dvDir * dvCap);
            }, execution);
    }
}

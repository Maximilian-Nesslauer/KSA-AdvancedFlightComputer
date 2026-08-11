using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Forward-chains N apse-burn flight plans (burnTa = Zero for Set
/// Apoapsis, Pi for Set Periapsis). All passes share one dV unit
/// vector: a prograde-at-apsis kick preserves the line of apsides.
/// The chaining and SOI / unbound-orbit checks live in
/// <see cref="MultiPassForwardChainPlanner"/>; this class just supplies
/// the apse-specific (burnTime, dvVlf) factory.
/// </summary>
internal static class ApseBurnPlanner
{
    public static PassPreviewResult Plan(
        Vehicle source,
        double3 totalDvVlf,
        TrueAnomaly burnTa,
        PassAllocation[] allocations,
        UniverseTime now)
    {
        double3 dvDir = totalDvVlf.NormalizeOrZero();
        if (dvDir.LengthSquared() < 0.5)
            return new PassPreviewResult(System.Array.Empty<PassPreview>(), Failed: true,
                FailureReason: "zero dV direction");

        return MultiPassForwardChainPlanner.PlanForwardChain(source, allocations, now,
            (orbit, dvCap, earliestTime) =>
            {
                // Caller must guard Eccentricity < 1 so this returns >= earliestTime.
                if (orbit.TimeOfTrueAnomaly(burnTa, earliestTime) is not UniverseTime burnTime)
                    return null;
                return new PassStep(burnTime, dvDir * dvCap);
            });
    }
}

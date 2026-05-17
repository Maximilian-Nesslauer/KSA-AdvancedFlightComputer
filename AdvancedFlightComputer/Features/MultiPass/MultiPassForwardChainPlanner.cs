using System.Collections.Generic;
using System.Globalization;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>Per-pass output of a <see cref="PassStepFactory"/>: when
/// the pass fires and the VLF dV vector to apply.</summary>
internal readonly record struct PassStep(SimTime BurnTime, double3 DvVlf);

/// <summary>Maneuver-type-specific step computer invoked once per pass
/// by <see cref="MultiPassForwardChainPlanner.PlanForwardChain"/>. The
/// factory sees the pre-burn orbit at the start of this pass plus the
/// splitter's dV budget, and answers "when to burn and by how much".
/// Return null to abort the chain (e.g. target unreachable from the
/// current orbit).
/// </summary>
/// <param name="currentOrbit"><see cref="Vehicle.Orbit"/> on the first
/// pass; the previous burn's post-burn orbit on subsequent passes.</param>
/// <param name="dvCapacityMs">Per-pass dV budget from the splitter.</param>
/// <param name="earliestTime">The earliest sim time this pass's burn
/// may occur. Equals <c>now</c> for the first pass; the previous
/// pass's burn time plus a 1 s epsilon for subsequent passes so that
/// chained <c>TimeOfTrueAnomaly</c> calls return the next occurrence
/// rather than the current one.</param>
internal delegate PassStep? PassStepFactory(
    Orbit currentOrbit, double dvCapacityMs, SimTime earliestTime);

/// <summary>
/// Forward-chains N flight plans for a multi-pass execution. Generic
/// over the maneuver type: caller supplies a <see cref="PassStepFactory"/>
/// that knows how to compute <c>(burnTime, dvVlf)</c> for one pass; the
/// helper handles patch chaining, unbound-orbit detection and SOI
/// Escape / Encounter checks between successive passes.
/// </summary>
internal static class MultiPassForwardChainPlanner
{
    // Slack so chained TimeOfTrueAnomaly returns the NEXT occurrence,
    // not "now" when the vehicle is currently at the target anomaly.
    private const double NextOccurrenceEpsilonSec = 1.0;

    // Stock live BurnPlan uses ~100 patches; our previews settle in
    // 1-2 even with an SOI transition, 8 is generous headroom.
    private const int FlightPlanPatchLimit = 8;

    // Per-segment encounter precision; lower than stock live (16) is
    // fine for a preview that only needs an indicative trajectory.
    private const int FlightPlanPolynomialOrder = 8;

    public static PassPreviewResult PlanForwardChain(
        Vehicle source,
        PassAllocation[] allocations,
        SimTime now,
        PassStepFactory stepFactory)
    {
        var results = new List<PassPreview>(allocations.Length);

        Orbit currentOrbit = source.Orbit;
        SimTime earliestTime = now;
        PatchedConic? prePatch = null;
        FlightPlan? lastFp = null;
        PatchedConic? lastBurnPatch = null;

        for (int i = 0; i < allocations.Length; i++)
        {
            PassStep? step = stepFactory(currentOrbit, allocations[i].DvCapacityMs, earliestTime);
            if (step == null)
                return new PassPreviewResult(results.ToArray(), Failed: true,
                    $"step factory returned no step for pass {i}");

            // SOI Escape / Encounter on the previous pass's flight plan
            // before this pass's burn time would invalidate the chained
            // TimeOfTrueAnomaly: the reference frame has flipped.
            if (i > 0 && lastFp != null)
            {
                foreach (PatchedConic p in lastFp.Patches)
                {
                    if (p.EndTime >= step.Value.BurnTime) continue;
                    if (p.EndTransition == PatchTransition.Escape)
                        return new PassPreviewResult(results.ToArray(), Failed: true,
                            $"pass {i - 1} escapes SOI before next pass");
                    if (p.EndTransition == PatchTransition.Encounter)
                        return new PassPreviewResult(results.ToArray(), Failed: true,
                            $"pass {i - 1} encounters another body before next pass");
                }
            }

            if (i == 0)
            {
                prePatch = source.FlightPlan.TryFindPatch(step.Value.BurnTime);
                if (prePatch == null || prePatch.PrimaryBody == null)
                    return new PassPreviewResult(results.ToArray(), Failed: true,
                        $"no parking patch at t={step.Value.BurnTime.Seconds():F0}s");
            }
            else
            {
                prePatch = lastBurnPatch;
            }

            var (fp, burnPatch) = BuildPassFlightPlan(
                source, prePatch!, step.Value.BurnTime, step.Value.DvVlf);
            results.Add(new PassPreview(
                BurnTime: step.Value.BurnTime,
                DvVlf: step.Value.DvVlf,
                EstimatedBurnTimeSec: allocations[i].EstimatedBurnTimeSec,
                FlightPlan: fp));

            // Pre / post orbit diagnostic: plane-change should leave SMA
            // and eccentricity nearly unchanged; apse burns should swing
            // one apsis substantially. Useful to spot dv-direction bugs.
            if (DebugConfig.MultiPass)
            {
                Orbit pre = currentOrbit;
                Orbit post = burnPatch.Orbit;
                DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                    "[AFC] ForwardChain pass {0}: burnTime={1:F1}s |dvVlf|={2:F3}m/s " +
                    "pre[SMA={3:F0} e={4:F6} Pe={5:F0} Ap={6:F0}] -> " +
                    "post[SMA={7:F0} e={8:F6} Pe={9:F0} Ap={10:F0}] " +
                    "delta[SMA={11:+0;-0;0}m e={12:+0.000000;-0.000000;0}]",
                    i, step.Value.BurnTime.Seconds(), step.Value.DvVlf.Length(),
                    pre.SemiMajorAxis, pre.Eccentricity, pre.Periapsis, pre.Apoapsis,
                    post.SemiMajorAxis, post.Eccentricity, post.Periapsis, post.Apoapsis,
                    post.SemiMajorAxis - pre.SemiMajorAxis,
                    post.Eccentricity - pre.Eccentricity));
            }

            if (!burnPatch.Orbit.IsBound())
                return new PassPreviewResult(results.ToArray(), Failed: true,
                    $"pass {i} produced an unbound orbit");

            if (i == allocations.Length - 1)
                break;

            currentOrbit = burnPatch.Orbit;
            earliestTime = new SimTime(step.Value.BurnTime.Seconds() + NextOccurrenceEpsilonSec);
            lastFp = fp;
            lastBurnPatch = burnPatch;
        }

        return new PassPreviewResult(results.ToArray(), Failed: false, FailureReason: null);
    }

    /// <summary>One pass's flight plan: a burn patch plus SOI
    /// propagation. Caller chains pass i+1 off the returned
    /// <paramref name="burnPatch"/>.</summary>
    private static (FlightPlan fp, PatchedConic burnPatch) BuildPassFlightPlan(
        Vehicle source, PatchedConic prePatch, SimTime burnTime, double3 dvVlf)
    {
        SimTime timeSincePe = prePatch.Orbit.GetTimeSincePeriapsisThisOrbit(burnTime);
        FlightPlan fp = FlightPlan.CreateUninitialized(source.Hash);
        PatchedConic burnPatch = fp.CalculateBurnPatch(prePatch, timeSincePe, dvVlf, burnTime);
        fp.Patches.Add(burnPatch);
        fp.ComputeCompleteTrajectory(FlightPlanPatchLimit, FlightPlanPolynomialOrder);
        if (source.Target != null)
            fp.CalculateTargetNodes(source.Target);
        return (fp, burnPatch);
    }
}

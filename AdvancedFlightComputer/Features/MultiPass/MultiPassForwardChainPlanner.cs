using System.Collections.Generic;
using System.Globalization;
using AdvancedFlightComputer.Core;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

internal readonly record struct PassStep(UniverseTime BurnTime, double3 DvVlf);

// The factory receives the orbit before the burn and returns null when the goal is unreachable.
internal delegate PassStep? PassStepFactory(
    Orbit currentOrbit, double dvCapacityMs, UniverseTime earliestTime);

internal static class MultiPassForwardChainPlanner
{
    // Move past the current anomaly so Orbit.TimeOfTrueAnomaly selects the next orbit.
    private const double NextOccurrenceEpsilonSec = 1.0;
    private const int FlightPlanPatchLimit = 8;
    private const int FlightPlanPolynomialOrder = 8;
    private const double PassDiagnosticMinIntervalSec = 2.0;

    public static PassPreviewResult PlanForwardChain(
        Vehicle source,
        PassAllocation[] allocations,
        UniverseTime now,
        PassStepFactory stepFactory) =>
        PlanForwardChain(source, allocations, now, stepFactory, false);

    public static PassPreviewResult PlanForwardChain(
        Vehicle source,
        PassAllocation[] allocations,
        UniverseTime now,
        PassStepFactory stepFactory, bool execution)
    {
        var results = new List<PassPreview>(allocations.Length);
        bool? logPasses = null;

        Orbit currentOrbit = source.Orbit;
        UniverseTime earliestTime = now;
        PatchedConic? prePatch = null;
        FlightPlan? lastFp = null;
        PatchedConic? lastBurnPatch = null;

        for (int i = 0; i < allocations.Length; i++)
        {
            PassStep? step = stepFactory(currentOrbit, allocations[i].DvCapacityMs, earliestTime);
            if (step == null)
                return new PassPreviewResult(results.ToArray(), Failed: true,
                    $"step factory returned no step for pass {i}");
            string? failure = CheckPreviousPass(lastFp, step.Value.BurnTime, i);
            if (failure != null)
                return new PassPreviewResult(results.ToArray(), Failed: true, failure);
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
            if (MultiPassDebug.Enabled)
                logPasses ??= execution || LogHelper.ThrottleAllows(
                    "forward-chain-passes", PassDiagnosticMinIntervalSec);
            if (logPasses == true)
                LogPass(source, currentOrbit, burnPatch.Orbit, step.Value, i, now, execution);
            if (!burnPatch.Orbit.IsBound())
                return new PassPreviewResult(results.ToArray(), Failed: true,
                    $"pass {i} produced an unbound orbit");

            if (i == allocations.Length - 1)
                break;

            currentOrbit = burnPatch.Orbit;
            earliestTime = step.Value.BurnTime + NextOccurrenceEpsilonSec;
            lastFp = fp;
            lastBurnPatch = burnPatch;
        }

        return new PassPreviewResult(results.ToArray(), Failed: false, FailureReason: null);
    }

    private static string? CheckPreviousPass(FlightPlan? plan, UniverseTime nextTime, int passIndex)
    {
        if (plan == null) return null;
        foreach (PatchedConic patch in plan.Patches)
        {
            if (patch.EndTime >= nextTime) continue;
            string? eventName = patch.EndTransition switch
            {
                PatchTransition.Escape => "escapes SOI",
                PatchTransition.Encounter => "encounters another body",
                PatchTransition.Impact => "impacts the parent body",
                _ => null,
            };
            if (eventName != null) return $"pass {passIndex - 1} {eventName} before next pass";
        }
        return null;
    }

    private static void LogPass(Vehicle source, Orbit pre, Orbit post, PassStep step,
        int i, UniverseTime chainTime, bool execution)
    {
        DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
            "[AFC] ForwardChain pass {0}: burnTime={1:F1}s |dvVlf|={2:F3}m/s " +
            "pre[SMA={3:F0} e={4:F6} Pe={5:F0} Ap={6:F0}] -> " +
            "post[SMA={7:F0} e={8:F6} Pe={9:F0} Ap={10:F0}] " +
            "delta[SMA={11:+0;-0;0}m e={12:+0.000000;-0.000000;0}] " +
            "vehicle='{13}' context={14} chainTime={15:R}s",
            i, step.BurnTime.Seconds(), step.DvVlf.Length(),
            pre.SemiMajorAxis, pre.Eccentricity, pre.Periapsis, pre.Apoapsis,
            post.SemiMajorAxis, post.Eccentricity, post.Periapsis, post.Apoapsis,
            post.SemiMajorAxis - pre.SemiMajorAxis,
            post.Eccentricity - pre.Eccentricity, source.Id,
            execution ? "execution" : "preview", chainTime.Seconds()));
    }

    // The returned burn patch is the next pass's starting orbit.
    internal static (FlightPlan fp, PatchedConic burnPatch) BuildPassFlightPlan(
        Vehicle source, PatchedConic prePatch, UniverseTime burnTime, double3 dvVlf,
        IOrbiter? encounterFilter = null)
    {
        UniverseTime timeSincePe = prePatch.Orbit.GetTimeSincePeriapsisThisOrbit(burnTime);
        FlightPlan fp = FlightPlan.CreateUninitialized(source.Hash);
        // Use the same terrain clearance as plans installed on the vehicle.
        fp.ImpactClearanceMargin = source.BoundingSphereRadiusBody;
        PatchedConic burnPatch = fp.CalculateBurnPatch(prePatch, timeSincePe, dvVlf, burnTime);
        fp.Patches.Add(burnPatch);
        // Detached plans have no worker to finish the incremental terrain search.
        fp.ComputeCompleteTrajectory(out _, FlightPlanPatchLimit, FlightPlanPolynomialOrder,
            encounterFilter, resolveImpactsCompletely: true);
        // Markers use closest approaches from every pass, including intermediate passes.
        if (source.Target != null)
            fp.CalculateTargetNodes(source.Target);
        return (fp, burnPatch);
    }
}

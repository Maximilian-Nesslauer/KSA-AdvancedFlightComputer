using System;
using System.Collections.Generic;
using System.Diagnostics;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using AdvancedFlightComputer.Features.StageInfo;
using Brutal.Logging;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance.Buffers;
using KSA;

namespace AdvancedFlightComputer.Features.OberthMultiPass;

/// <summary>
/// Captures the MultiPassState fields that CreateBurns will overwrite.
/// Restoring them after CreateBurns keeps BackToSingle coherent across
/// multiple correction cycles.
/// </summary>
readonly struct OriginalBurnSnapshot
{
    readonly GoalFunc? _corrGoal;
    readonly double3 _dvVlf;
    readonly SimTime _burnTime;
    readonly int _passCount;
    readonly double[]? _dvCaps;  // preserved for Phase 2.5 per-pass dV reset across correction cycles

    private OriginalBurnSnapshot(
        GoalFunc? corrGoal, double3 dvVlf, SimTime burnTime, int passCount, double[]? dvCaps)
    {
        _corrGoal  = corrGoal;
        _dvVlf     = dvVlf;
        _burnTime  = burnTime;
        _passCount = passCount;
        _dvCaps    = dvCaps;
    }

    public static OriginalBurnSnapshot Capture() => new OriginalBurnSnapshot(
        MultiPassState.CorrectionGoal,
        MultiPassState.OriginalDvVlf,
        MultiPassState.OriginalBurnTime,
        MultiPassState.OriginalPassCount,
        MultiPassState.OriginalDvCapacities);

    public void Restore()
    {
        MultiPassState.CorrectionGoal       = _corrGoal;
        MultiPassState.OriginalDvVlf        = _dvVlf;
        MultiPassState.OriginalBurnTime     = _burnTime;
        MultiPassState.OriginalPassCount    = _passCount;
        MultiPassState.OriginalDvCapacities = _dvCaps;
    }
}

/// <summary>
/// Core orchestration for multi-pass burn planning, preview, creation, and
/// correction. All methods are static and stateless; state is stored in
/// MultiPassState. Rendering is handled by MultiPassRenderer.
/// </summary>
static class MultiPassPlanner
{
    #region Preview

    /// <summary>
    /// Computes a preview for apse burns (Set Ap/Pe): equal VLF split at a
    /// fixed True Anomaly, chained across passes via FlightPlan orbit propagation.
    /// Stores results in MultiPassState.
    /// </summary>
    public static bool ComputeApseBurnPreview(
        Vehicle source, List<PassAllocation> allocations,
        TrueAnomaly burnTa, double3 dvDirection)
    {
        SimTime now = Universe.GetElapsedSimTime();
        // Add 1s so TimeOfTrueAnomaly always returns the NEXT occurrence of burnTa,
        // not the current one. Without this, if the vehicle is exactly at burnTa
        // (e.g. right after a burn at apoapsis), the method returns ~now and the
        // next pass is scheduled immediately instead of one orbit later.
        SimTime burnTime = source.Orbit.TimeOfTrueAnomaly(burnTa, new SimTime(now.Seconds() + 1.0));

        bool escapeWarning = false;
        var results = new List<PassResult>(allocations.Count);
        PatchedConic? prevBurnPatch = null;

        for (int i = 0; i < allocations.Count; i++)
        {
            if (allocations[i].DvCapacity < 0.1)
                continue;

            // Pass 0 uses the vehicle's actual flight plan; subsequent passes chain
            // directly from the previous iteration's burn patch to avoid TryFindPatch
            // boundary issues (burnPatch.EndTime == nextBurnTime can fail floating-point <=).
            PatchedConic? prePatch = (i == 0)
                ? source.FlightPlan.TryFindPatch(burnTime)
                : prevBurnPatch;

            if (prePatch == null || prePatch.PrimaryBody == null)
            {
                if (DebugConfig.OberthMultiPass)
                    DefaultCategory.Log.Debug(
                        $"[AFC] MultiPassPlanner: apse pass {i} - patch not found at {burnTime.Seconds():F0}s.");
                escapeWarning = true;
                break;
            }

            double3 dvVlf = dvDirection * allocations[i].DvCapacity;
            var (fp, burnPatch) = CreatePreviewFlightPlan(source, prePatch, burnTime, dvVlf);
            MultiPassRenderer.SetPassColors(fp, i, allocations.Count);

            if (!burnPatch.Orbit.IsBound())
            {
                if (DebugConfig.OberthMultiPass)
                    DefaultCategory.Log.Debug($"[AFC] MultiPassPlanner: apse pass {i} - post-burn orbit unbound.");
                results.Add(new PassResult(burnTime, dvVlf, allocations[i].EstimatedBurnTime, fp));
                escapeWarning = true;
                break;
            }

            results.Add(new PassResult(burnTime, dvVlf, allocations[i].EstimatedBurnTime, fp));

            // Compute next burn time and check SOI escape for intermediate passes.
            if (i < allocations.Count - 1)
            {
                SimTime afterBurn = new SimTime(burnTime.Seconds() + 1.0);
                SimTime nextBurnTime = burnPatch.Orbit.TimeOfTrueAnomaly(burnTa, afterBurn);

                bool soiEscape = false;
                foreach (PatchedConic p in fp.Patches)
                {
                    if (p.EndTransition == PatchTransition.Escape && p.EndTime < nextBurnTime)
                    { soiEscape = true; break; }
                }
                if (soiEscape)
                {
                    if (DebugConfig.OberthMultiPass)
                        DefaultCategory.Log.Debug($"[AFC] MultiPassPlanner: apse pass {i} - SOI escape before next pass.");
                    escapeWarning = true;
                    break;
                }

                prevBurnPatch = burnPatch;
                burnTime = nextBurnTime;
            }
        }

        if (results.Count < 2)
        {
            if (DebugConfig.OberthMultiPass)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPassPlanner: apse preview failed - only {results.Count} passes. Escape: {escapeWarning}");
            MultiPassState.PreviewFailed = true;
            return false;
        }

        MultiPassState.PreviewPasses = results;
        MultiPassState.PreviewSource = source;
        MultiPassState.ShowOrbitPreview = true;
        MultiPassState.PreviewFailed = false;
        MultiPassState.PreviewDvDirection = dvDirection;
        MultiPassState.PreviewGoal = null;

        if (DebugConfig.OberthMultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPassPlanner: apse preview computed {results.Count} passes. Escape warning: {escapeWarning}");

        return true;
    }

    /// <summary>
    /// Computes a preview for plane-change burns (Match/Set Inclination).
    /// Stores results in MultiPassState.
    /// </summary>
    public static bool ComputeGoalPreview(
        Vehicle source, List<PassAllocation> allocations,
        GoalFunc previewGoal, SimTime now,
        Func<Orbit, double>? totalAngleFunc = null)
    {
        Orbit currentOrbit = source.Orbit;
        var results = new List<PassResult>(allocations.Count);
        PatchedConic? prevBurnPatch = null;

        for (int i = 0; i < allocations.Count; i++)
        {
            if (allocations[i].DvCapacity < 0.1)
                continue;

            // Get the full remaining rotation to compute the arcsin fraction.
            OrbitManeuvers.ManeuverResult? fullResult = previewGoal(currentOrbit, now, 1.0);
            if (fullResult == null || fullResult.Value.DvCci.Length() < 0.5)
            {
                if (DebugConfig.OberthMultiPass)
                    DefaultCategory.Log.Debug($"[AFC] MultiPassPlanner: goal preview pass {i} - goal achieved.");
                break;
            }

            SimTime nodeTime = fullResult.Value.BurnTime;
            double v = currentOrbit.GetStateVectorsAt(nodeTime).VelocityCci.Length();
            double dvFull = fullResult.Value.DvCci.Length();

            if (v < 1.0)
            {
                if (DebugConfig.OberthMultiPass)
                    DefaultCategory.Log.Debug($"[AFC] MultiPassPlanner: goal preview pass {i} - orbital speed < 1 m/s.");
                break;
            }

            // Prefer the direct geometric angle from the orbit when available. It is
            // exact by definition and avoids the dV-inversion approximation. Fall back to
            // inverting the plane-change formula dV = 2v*sin(theta/2) when no angle
            // function is provided (e.g. for generic goal-based burns).
            double totalRemainingAngle = totalAngleFunc != null
                ? totalAngleFunc(currentOrbit)
                : 2.0 * Math.Asin(Math.Clamp(dvFull / (2.0 * v), 0.0, 0.9999));

            // Derive the effective perpendicular velocity from the full GoalFunc result.
            // The plane change formula dV = 2*v*sin(theta/2) only holds when all of v is
            // perpendicular to the rotation axis. For eccentric orbits, the velocity at the
            // AN/DN has a radial component along the line of nodes (rotation axis) that is
            // unchanged by the rotation. Using total v overestimates the fraction denominator,
            // causing each pass to under-rotate and waste dV capacity.
            // Since dvFull = 2*v_perp*sin(totalAngle/2), we can invert to get v_perp.
            double halfAngleSin = Math.Sin(totalRemainingAngle / 2.0);
            double vEffective = (halfAngleSin > 0.001)
                ? dvFull / (2.0 * halfAngleSin)
                : v;

            double dvCapacity = allocations[i].DvCapacity;
            double fraction;
            if (dvCapacity >= 2.0 * vEffective * 0.9999)
            {
                fraction = 1.0;
            }
            else
            {
                double dvCapped = Math.Min(dvCapacity, 2.0 * vEffective * 0.9999);
                double theta = 2.0 * Math.Asin(dvCapped / (2.0 * vEffective));
                fraction = Math.Clamp(theta / totalRemainingAngle, 0.0, 1.0);
            }

            if (DebugConfig.OberthMultiPass)
                DefaultCategory.Log.Debug(
                    $"[AFC] GoalPreview pass {i}: inc={currentOrbit.Inclination * (180.0 / Math.PI):F2}deg, " +
                    $"remainAngle={totalRemainingAngle * (180.0 / Math.PI):F2}deg, " +
                    $"dvCap={dvCapacity:F1} v={v:F1} vEff={vEffective:F1} fraction={fraction:F4}");

            OrbitManeuvers.ManeuverResult? result = previewGoal(currentOrbit, now, fraction);
            if (result == null)
            {
                if (DebugConfig.OberthMultiPass)
                    DefaultCategory.Log.Debug(
                        $"[AFC] MultiPassPlanner: goal preview pass {i} - null result at fraction {fraction:F3}.");
                break;
            }

            SimTime burnTime = result.Value.BurnTime;

            // Pass 0 uses the vehicle's actual flight plan; subsequent passes chain
            // directly from the previous iteration's burn patch.
            PatchedConic? prePatch = (i == 0)
                ? source.FlightPlan.TryFindPatch(burnTime)
                : prevBurnPatch;

            if (prePatch == null || prePatch.PrimaryBody == null)
            {
                if (DebugConfig.OberthMultiPass)
                    DefaultCategory.Log.Debug($"[AFC] MultiPassPlanner: goal preview pass {i} - patch not found.");
                break;
            }

            var (fp, burnPatch) = CreatePreviewFlightPlan(source, prePatch, burnTime, result.Value.DvVlf);
            MultiPassRenderer.SetPassColors(fp, i, allocations.Count);

            if (!burnPatch.Orbit.IsBound())
            {
                if (DebugConfig.OberthMultiPass)
                    DefaultCategory.Log.Debug($"[AFC] MultiPassPlanner: goal preview pass {i} - post-burn orbit unbound.");
                results.Add(new PassResult(burnTime, result.Value.DvVlf, allocations[i].EstimatedBurnTime, fp));
                break;
            }

            if (DebugConfig.OberthMultiPass)
                DefaultCategory.Log.Debug(
                    $"[AFC] GoalPreview pass {i} post-burn: inc={burnPatch.Orbit.Inclination * (180.0 / Math.PI):F2}deg, " +
                    $"dv={result.Value.DvCci.Length():F1}");

            results.Add(new PassResult(burnTime, result.Value.DvVlf, allocations[i].EstimatedBurnTime, fp));

            // Post-burn orbit is the starting orbit for the next pass.
            prevBurnPatch = burnPatch;
            currentOrbit = burnPatch.Orbit;
            now = new SimTime(burnTime.Seconds() + 1.0);

            // SOI escape check: call the goal func with the
            // updated orbit to get the next pass's burn time, then verify no patch in the
            // current flight plan escapes SOI before that time.
            if (i < allocations.Count - 1)
            {
                OrbitManeuvers.ManeuverResult? nextFull = previewGoal(currentOrbit, now, 1.0);
                if (nextFull != null)
                {
                    bool soiEscape = false;
                    foreach (PatchedConic p in fp.Patches)
                    {
                        if (p.EndTransition == PatchTransition.Escape && p.EndTime < nextFull.Value.BurnTime)
                        { soiEscape = true; break; }
                    }
                    if (soiEscape)
                    {
                        if (DebugConfig.OberthMultiPass)
                            DefaultCategory.Log.Debug(
                                $"[AFC] MultiPassPlanner: goal preview pass {i} - SOI escape before next pass.");
                        break;
                    }
                }
            }
        }

        if (results.Count < 2)
        {
            if (DebugConfig.OberthMultiPass)
                DefaultCategory.Log.Debug(
                    $"[AFC] MultiPassPlanner: goal preview failed - only {results.Count} passes computed.");
            MultiPassState.PreviewFailed = true;
            return false;
        }

        MultiPassState.PreviewPasses = results;
        MultiPassState.PreviewSource = source;
        MultiPassState.ShowOrbitPreview = true;
        MultiPassState.PreviewFailed = false;
        MultiPassState.PreviewGoal = previewGoal;
        MultiPassState.PreviewDvDirection = default;

        if (DebugConfig.OberthMultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPassPlanner: goal preview computed {results.Count} passes.");

        return true;
    }

    /// <summary>
    /// Creates a preview FlightPlan for a single pass. Computes the post-burn
    /// orbit via CalculateBurnPatch and runs SOI propagation.
    /// </summary>
    private static (FlightPlan fp, PatchedConic burnPatch) CreatePreviewFlightPlan(
        Vehicle source, PatchedConic prePatch, SimTime burnTime, double3 dvVlf)
    {
        SimTime timeSincePe = prePatch.Orbit.GetTimeSincePeriapsisThisOrbit(burnTime);
        FlightPlan fp = FlightPlan.CreateUninitialized(source.Hash);
        PatchedConic burnPatch = fp.CalculateBurnPatch(prePatch, timeSincePe, dvVlf, burnTime);
        fp.Patches.Add(burnPatch);
        fp.ComputeCompleteTrajectory(5, 8);
        return (fp, burnPatch);
    }

    #endregion

    #region Create / Remove Burns

    /// <summary>
    /// Creates real Burn objects in the BurnPlan from the current preview.
    /// Follows the exact DeserializeSave pattern: one burn at a time, recalculate
    /// flight plans after each AddBurn. Rolls back on any failure.
    /// </summary>
    public static bool CreateBurns(
        Vehicle source,
        double3 originalDvVlf,
        SimTime originalBurnTime,
        GoalFunc? correctionGoal,
        double[]? originalDvCapacities,
        TrueAnomaly activeBurnTa)
    {
        if (MultiPassState.PreviewPasses == null)
        {
            DefaultCategory.Log.Warning("[AFC] MultiPassPlanner.CreateBurns: no preview exists.");
            return false;
        }

        var fc = source.FlightComputer;

        // Remove the burn left behind by a previous RemoveAllBurns (BackToSingle).
        // If it is still in the BurnPlan, CalculateNewFlightPlans chains each pass
        // flight plan from the post-singleBurn orbit instead of the vehicle's actual
        // orbit, producing wrong dV on every subsequent Create+BackToSingle cycle.
        if (MultiPassState.RestoredSingleBurn != null)
        {
            if (fc.BurnPlan.TryGetBurn(MultiPassState.RestoredSingleBurn))
            {
                if (DebugConfig.OberthMultiPass)
                    DefaultCategory.Log.Debug(
                        "[AFC] MultiPassPlanner.CreateBurns: removing stale restored single burn.");
                fc.RemoveBurn(MultiPassState.RestoredSingleBurn);
            }
            MultiPassState.RestoredSingleBurn = null;
        }

        var passes = MultiPassState.PreviewPasses;
        var createdBurns = new List<Burn>(passes.Count);

        for (int i = 0; i < passes.Count; i++)
        {
            PassResult pass = passes[i];

            PatchedConic? patch = (i == 0)
                ? source.FlightPlan.TryFindPatch(pass.BurnTime)
                : createdBurns[i - 1].FlightPlan.TryFindPatch(pass.BurnTime);

            if (patch == null)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] MultiPassPlanner.CreateBurns: patch not found for pass {i}. Rolling back {createdBurns.Count} burns.");
                for (int r = createdBurns.Count - 1; r >= 0; r--)
                    fc.RemoveBurn(createdBurns[r]);
                return false;
            }

            OrbitPointCce point = patch.Orbit.GetPointAt(pass.BurnTime);
            Burn burn = Burn.Create(point, pass.BurnTime.Seconds(), pass.DvVlf, patch, source);
            burn.IsGizmoActive = false;
            fc.AddBurn(burn);

            List<FlightPlan> fps = fc.CalculateNewFlightPlans(source.FlightPlan, source.Hash);
            foreach (FlightPlan fp in fps)
            {
                fp.CalculateTargetNodes(source.Target);
                foreach (PatchedConic p in fp.Patches)
                    p.Orbit.UpdateCachedPoints(UpdateTaskUtils.GenerateSpacedPoints(p));
            }
            fc.BurnPlan.UpdateBurnPlan(source, fps);
            createdBurns.Add(burn);
        }

        // Store EstimatedBurnTime values from the preview before ClearPreview wipes them.
        // These are used in OberthUI to show accurate per-pass burn times that account for
        // multi-stage fuel consumption across passes. The local `passes` ref stays valid.
        var plannedTimes = new double[passes.Count];
        for (int i = 0; i < passes.Count; i++)
            plannedTimes[i] = passes[i].EstimatedBurnTime;

        MultiPassState.Vehicle = source;
        MultiPassState.PassBurns = createdBurns;
        MultiPassState.OriginalPassCount = createdBurns.Count;
        MultiPassState.OriginalDvVlf = originalDvVlf;
        MultiPassState.OriginalBurnTime = originalBurnTime;
        MultiPassState.OriginalDvCapacities = originalDvCapacities;
        MultiPassState.PlannedBurnTimes = plannedTimes;
        MultiPassState.CorrectionGoal = correctionGoal;
        MultiPassState.SelectedPassIndex = 0;
        // Save ActiveDvDirection and ActiveBurnTa BEFORE ClearPreview(),
        // which wipes PreviewDvDirection.
        MultiPassState.ActiveDvDirection = MultiPassState.PreviewDvDirection;
        MultiPassState.ActiveBurnTa = activeBurnTa;
        // IsApseBurn: true when burnTa is a real TA (Set Ap/Pe), false for plane changes
        // where burnTa is NaN and the GoalFunc path is used instead.
        MultiPassState.IsApseBurn = activeBurnTa != TrueAnomaly.NaN;
        MultiPassState.ClearPreview();

        // Activate gizmo on the first burn so the user sees it immediately.
        if (createdBurns.Count > 0)
            createdBurns[0].IsGizmoActive = true;

        if (DebugConfig.OberthMultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPassPlanner.CreateBurns: created {createdBurns.Count} burns successfully. " +
                $"BurnMode={source.FlightComputer.BurnMode} (enable Auto to execute).");

        return true;
    }

    /// <summary>
    /// Removes all pass burns and recreates the original single burn.
    /// Only safe to call when !HasExecutedPasses.
    /// </summary>
    public static bool RemoveAllBurns(Vehicle vehicle)
    {
        if (!MultiPassState.HasActiveSplit || MultiPassState.PassBurns == null)
            return false;

        var fc = vehicle.FlightComputer;

        foreach (Burn b in MultiPassState.PassBurns)
            fc.RemoveBurn(b);

        // Recreate the original single burn.
        PatchedConic? patch = vehicle.FlightPlan.TryFindPatch(MultiPassState.OriginalBurnTime);
        if (patch == null)
        {
            DefaultCategory.Log.Warning(
                "[AFC] MultiPassPlanner.RemoveAllBurns: original burn patch not found.");
            MultiPassState.Reset();
            return false;
        }

        OrbitPointCce point = patch.Orbit.GetPointAt(MultiPassState.OriginalBurnTime);
        Burn burn = Burn.Create(point, MultiPassState.OriginalBurnTime.Seconds(),
            MultiPassState.OriginalDvVlf, patch, vehicle);
        burn.IsGizmoActive = false;
        fc.AddBurn(burn);
        RebuildFlightPlans(vehicle, fc);

        MultiPassState.Reset();
        // Track so CreateBurns can remove it before adding pass burns.
        // Without this, CalculateNewFlightPlans chains pass flight plans from the
        // post-singleBurn orbit instead of the vehicle's actual orbit.
        MultiPassState.RestoredSingleBurn = burn;
        // Invalidate the preview hash so the next frame recomputes a fresh preview
        // against the restored single-burn orbit state.
        OberthManeuverIntegration.InvalidatePreviewHash();

        if (DebugConfig.OberthMultiPass)
            DefaultCategory.Log.Debug("[AFC] MultiPassPlanner.RemoveAllBurns: restored single burn.");

        return true;
    }

    #endregion

    #region Execution Correction

    /// <summary>
    /// Called by AutoRemoveBurn after a pass completes. Recomputes remaining
    /// passes using the CorrectionGoal from the actual post-burn orbit.
    /// </summary>
    public static void HandlePassCompletion(Vehicle vehicle)
    {
        MultiPassState.ValidateState();

        if (!MultiPassState.HasActiveSplit || MultiPassState.PassBurns == null)
            return;

        var fc = vehicle.FlightComputer;
        if (!fc.BurnPlan.HasActiveBurns)
            return;

        int remainingPasses = MultiPassState.PassBurns.Count;
        SimTime now = Universe.GetElapsedSimTime();
        // Add 1s so apse-type CorrectionGoals (Set Ap/Pe) find the NEXT apse
        // occurrence rather than returning ~now. Without this, TimeOfTrueAnomaly
        // returns 0 when the vehicle is still at the burn apse right after execution,
        // scheduling the corrected burn immediately instead of one orbit later.
        SimTime nowPlus = new SimTime(now.Seconds() + 1.0);

        if (DebugConfig.OberthMultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPassPlanner.HandlePassCompletion: {remainingPasses} passes remaining.");

        if (MultiPassState.CorrectionGoal == null)
        {
            if (DebugConfig.OberthMultiPass)
                DefaultCategory.Log.Debug(
                    "[AFC] MultiPassPlanner.HandlePassCompletion: no CorrectionGoal, skipping correction.");
            return;
        }

        OrbitManeuvers.ManeuverResult? remaining =
            MultiPassState.CorrectionGoal(vehicle.Orbit, nowPlus, 1.0);

        if (remaining == null || remaining.Value.DvCci.Length() < 0.5)
        {
            if (DebugConfig.OberthMultiPass)
                DefaultCategory.Log.Debug(
                    "[AFC] MultiPassPlanner.HandlePassCompletion: goal achieved, removing remaining burns.");
            foreach (Burn b in MultiPassState.PassBurns)
                fc.RemoveBurn(b);
            MultiPassState.Reset();
            return;
        }

        // Single remaining pass: correct the burn directly without the multi-pass
        // preview pipeline (which requires >= 2 passes).
        if (remainingPasses == 1)
        {
            CorrectLastPass(vehicle, fc, remaining.Value);
            return;
        }

        VehicleBurnAnalysis analysis = StageAnalysisCache.Analysis ?? StageAnalysisCache.Empty;
        List<PassAllocation> allocations = BurnTimeSplitter.ComputeAllocations(
            remaining.Value.DvCci.Length(), remainingPasses, analysis);

        // Preserve state before CreateBurns can overwrite it.
        GoalFunc corrGoal = MultiPassState.CorrectionGoal;
        TrueAnomaly burnTa = MultiPassState.ActiveBurnTa;
        var snapshot = OriginalBurnSnapshot.Capture();

        // 1. Compute corrected preview first, before touching the BurnPlan.
        // For apse burns, derive the current correction direction from the CorrectionGoal
        // result rather than the stored ActiveDvDirection. After an imprecise burn, the
        // direction can flip (e.g., over-burned Set Apoapsis now needs retrograde correction).
        bool previewOk;
        if (MultiPassState.IsApseBurn)
        {
            double3 correctedDir = remaining.Value.DvVlf.NormalizeOrZero();
            previewOk = ComputeApseBurnPreview(vehicle, allocations, burnTa, correctedDir);
        }
        else
        {
            previewOk = ComputeGoalPreview(vehicle, allocations, corrGoal, nowPlus);
        }

        // 2. If preview failed, leave remaining burns in BurnPlan unchanged so they
        //    execute as originally planned.
        if (!previewOk)
        {
            DefaultCategory.Log.Warning(
                "[AFC] MultiPassPlanner.HandlePassCompletion: correction preview failed, burns unchanged.");
            return;
        }

        // 3. Preview succeeded: remove old burns, then create corrected ones.
        //    Copy PassBurns before removal because CreateBurns overwrites MultiPassState.PassBurns.
        var burnsToRemove = new List<Burn>(MultiPassState.PassBurns);
        foreach (Burn b in burnsToRemove)
            fc.RemoveBurn(b);

        RebuildFlightPlans(vehicle, fc);

        // 4. Extract dV capacities from the corrected preview.
        double[]? freshCaps = MultiPassState.ExtractPreviewDvCapacities();

        if (!CreateBurns(vehicle, remaining.Value.DvVlf, remaining.Value.BurnTime, corrGoal, freshCaps, burnTa))
        {
            // Old burns were already removed.
            DefaultCategory.Log.Warning(
                "[AFC] MultiPassPlanner.HandlePassCompletion: failed to create corrected burns, resetting state.");
            MultiPassState.Reset();
            return;
        }

        snapshot.Restore();

        if (DebugConfig.OberthMultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPassPlanner.HandlePassCompletion: recreated {remainingPasses} corrected burns.");
    }

    /// <summary>
    /// Corrects the single remaining pass by removing the old burn and creating
    /// a new one with the corrected dV from the CorrectionGoal. Bypasses the
    /// multi-pass preview pipeline which requires >= 2 passes.
    /// </summary>
    private static void CorrectLastPass(
        Vehicle vehicle, FlightComputer fc, OrbitManeuvers.ManeuverResult remaining)
    {
        var burnsToRemove = new List<Burn>(MultiPassState.PassBurns!);
        // Capture original state before modifying PassBurns and related fields.
        // ActiveDvDirection and ActiveBurnTa are intentionally NOT in the snapshot:
        // they retain the original values from CreateBurns and remain valid for
        // determining apse-vs-goal path in any subsequent HandlePassCompletion calls.
        var snapshot = OriginalBurnSnapshot.Capture();

        foreach (Burn b in burnsToRemove)
            fc.RemoveBurn(b);

        RebuildFlightPlans(vehicle, fc);

        PatchedConic? patch = vehicle.FlightPlan.TryFindPatch(remaining.BurnTime);
        if (patch == null)
        {
            DefaultCategory.Log.Warning(
                "[AFC] MultiPassPlanner.CorrectLastPass: patch not found for corrected burn.");
            MultiPassState.Reset();
            return;
        }

        OrbitPointCce point = patch.Orbit.GetPointAt(remaining.BurnTime);
        Burn burn = Burn.Create(point, remaining.BurnTime.Seconds(), remaining.DvVlf, patch, vehicle);
        burn.IsGizmoActive = false;
        fc.AddBurn(burn);
        RebuildFlightPlans(vehicle, fc);

        MultiPassState.PassBurns = new List<Burn> { burn };
        MultiPassState.SelectedPassIndex = 0;
        MultiPassState.PlannedBurnTimes = null; // single corrected pass, estimate not available
        snapshot.Restore();

        if (DebugConfig.OberthMultiPass)
            DefaultCategory.Log.Debug(
                $"[AFC] MultiPassPlanner.CorrectLastPass: corrected last burn, dV={remaining.DvCci.Length():F1} m/s.");
    }

    /// <summary>
    /// Rebuilds flight plans after removing burns, so subsequent CreateBurns
    /// or burn creation sees consistent state.
    /// </summary>
    private static void RebuildFlightPlans(Vehicle vehicle, FlightComputer fc)
    {
        if (!fc.BurnPlan.HasActiveBurns)
            return;

        List<FlightPlan> rebuildFps = fc.CalculateNewFlightPlans(vehicle.FlightPlan, vehicle.Hash);
        foreach (FlightPlan rfp in rebuildFps)
        {
            rfp.CalculateTargetNodes(vehicle.Target);
            foreach (PatchedConic rp in rfp.Patches)
                rp.Orbit.UpdateCachedPoints(UpdateTaskUtils.GenerateSpacedPoints(rp));
        }
        fc.BurnPlan.UpdateBurnPlan(vehicle, rebuildFps);
    }

    #endregion
}

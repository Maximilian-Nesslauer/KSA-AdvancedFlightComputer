using System;
using System.Threading;
using System.Threading.Tasks;
using AdvancedFlightComputer.Features.HyperbolicTargets;
using Brutal.Logging;
using Brutal.Numerics;
using Core;
using KSA;

namespace AdvancedFlightComputer.Features.Transfer;

/// <summary>
/// Cancellable background intercept refinement that operates solely on main-thread snapshots of the
/// departure patch and porkchop entry so worker threads never race live Vehicle or UI state.
/// </summary>
internal sealed class TransferRefineTask
{
    private readonly CancellationTokenSource _cts = new();
    private int _done;
    private int _disposed;
    private volatile bool _cancelled;
    private volatile bool _succeeded;

    internal OrbitalTransfers.PorkChopEntry Result { get; }
    internal bool IsDone => Volatile.Read(ref _done) != 0;
    internal bool Succeeded => _succeeded;
    internal bool IsCancellationRequested => _cancelled;

    internal TransferRefineTask(
        OrbitalTransfers.PorkChopEntry workEntry,
        PatchedConic departurePatchSnapshot,
        OrbitPointCce departurePoint,
        KeyHash vehicleHash,
        IOrbiter target)
        : this(workEntry, (stop, entry) => RunRefine(stop, entry, departurePatchSnapshot, departurePoint, vehicleHash, target))
    {
    }

    internal TransferRefineTask(
        OrbitalTransfers.PorkChopEntry workEntry,
        Func<CancellationToken, OrbitalTransfers.PorkChopEntry, bool> runner)
    {
        Result = workEntry;
        CancellationToken token = _cts.Token;
        _ = Task.Run(() =>
        {
            try
            {
                if (!token.IsCancellationRequested)
                    _succeeded = runner(token, Result);
            }
            catch (OperationCanceledException) when (token.IsCancellationRequested)
            {
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] Transfer intercept refinement failed: {ex.GetBaseException().Message}");
            }
            finally
            {
                Volatile.Write(ref _done, 1);
            }
        });
    }

    internal void Cancel()
    {
        _cancelled = true;
        if (Interlocked.Exchange(ref _disposed, 1) == 0)
        {
            _cts.Cancel();
            _cts.Dispose();
        }
    }

    internal void Dispose()
    {
        if (Interlocked.Exchange(ref _disposed, 1) == 0)
            _cts.Dispose();
    }

    private static bool RunRefine(
        CancellationToken stop,
        OrbitalTransfers.PorkChopEntry entry,
        PatchedConic departurePatch,
        OrbitPointCce departurePoint,
        KeyHash vehicleHash,
        IOrbiter target)
    {
        if (target.Orbit != null && !target.Orbit.IsBound())
        {
            if (stop.IsCancellationRequested)
                return false;
            FlightPlan unboundPlan = FlightPlan.CreateUninitialized(vehicleHash);
            if (!BuildSnapshotFlightPlan(unboundPlan, departurePatch, entry.TransferData.Start, entry.TransferData.TransferDvVlf, target))
                return false;
            if (stop.IsCancellationRequested)
                return false;
            entry.FlightPlan = unboundPlan;
            entry.TransferData.Point = departurePoint;
            entry.TransferData.ClosestApproachDistance = Patch_TryFindIntercept.FindPatchedConicClosestApproach(
                unboundPlan, target, entry.TransferData.Transit.Seconds());
            return true;
        }

        if (target is not IParentBody parentBody)
            return false;

        double delta = entry.TransferData.TransferDvVlf.X * RefineBurnTask.INTERCEPT_REFINEMENT_PERCENTAGE;
        double low = entry.TransferData.TransferDvVlf.X - delta;
        double high = entry.TransferData.TransferDvVlf.X + delta;
        if (low > high)
            (low, high) = (high, low);

        double maxSteps = delta * 2.0 - 1.0;
        int step = 0;
        for (double dvX = low; dvX < high && step < maxSteps; dvX += 1.0)
        {
            if (stop.IsCancellationRequested)
                return false;
            double3 trialDv = new(dvX, entry.TransferData.TransferDvVlf.Y, entry.TransferData.TransferDvVlf.Z);
            FlightPlan flightPlan = FlightPlan.CreateUninitialized(vehicleHash);
            if (BuildSnapshotFlightPlan(flightPlan, departurePatch, entry.TransferData.Start, trialDv, target)
                && InterceptsBody(target, flightPlan, parentBody.SphereOfInfluence, out double closestApproach))
            {
                entry.FlightPlan = flightPlan;
                entry.TransferData.TransferDvVlf = trialDv;
                entry.TransferData.Point = departurePoint;
                entry.TransferData.ClosestApproachDistance = closestApproach;
                return true;
            }
            step++;
        }
        return false;
    }

    internal static bool BuildSnapshotFlightPlan(
        FlightPlan flightPlan, PatchedConic departurePatch, UniverseTime startTime, double3 dvVlf, IOrbiter? target)
    {
        var prePatch = new PatchedConic(departurePatch);
        prePatch.EndTime = startTime;
        UniverseTime fromPe = prePatch.Orbit.GetTimeSincePeriapsisThisOrbit(startTime);
        prePatch.EndTrueAnomaly = prePatch.Orbit.GetTrueAnomaly(fromPe);
        PatchedConic burnPatch = flightPlan.CalculateBurnPatch(prePatch, fromPe, dvVlf, startTime);
        flightPlan.Patches.Add(burnPatch);
        flightPlan.ComputeCompleteTrajectory(out bool errorsEncountered, 5, 8, target);
        return !errorsEncountered;
    }

    private static bool InterceptsBody(IOrbiter target, FlightPlan flightPlan, double soi, out double closestApproach)
    {
        closestApproach = double.MaxValue;
        foreach (PatchedConic patch in flightPlan.Patches)
        {
            Encounter? encounter = patch.TryFindClosestEncounter(target);
            if (encounter.HasValue && encounter.Value.ClosestDistance < soi)
            {
                closestApproach = encounter.Value.ClosestDistance;
                return true;
            }
        }
        return false;
    }
}

/// <summary>
/// Cancellable background course-correction solver that runs on deep-copied transfer data and flight
/// plan snapshots and validates finiteness after computing the VLF burn vector.
/// </summary>
internal sealed class TransferCorrectionTask
{
    private readonly CancellationTokenSource _cts = new();
    private int _done;
    private int _disposed;
    private volatile bool _cancelled;
    private volatile bool _succeeded;

    internal OrbitalTransfers.TransferData TransferData { get; }
    internal bool IsDone => Volatile.Read(ref _done) != 0;
    internal bool Succeeded => _succeeded;
    internal bool IsCancellationRequested => _cancelled;

    internal TransferCorrectionTask(
        IOrbiter target,
        double requiredPeAltitude,
        OrbitalTransfers.TransferData dataSnapshot,
        FlightPlan planSnapshot)
        : this(dataSnapshot, (stop, data) => CalculateCorrection(target, requiredPeAltitude, data, planSnapshot, stop))
    {
    }

    internal TransferCorrectionTask(
        OrbitalTransfers.TransferData dataSnapshot,
        Func<CancellationToken, OrbitalTransfers.TransferData, bool> runner)
    {
        TransferData = dataSnapshot;
        CancellationToken token = _cts.Token;
        _ = Task.Run(() =>
        {
            try
            {
                if (!token.IsCancellationRequested)
                    _succeeded = runner(token, TransferData);
            }
            catch (OperationCanceledException) when (token.IsCancellationRequested)
            {
            }
            catch (Exception ex)
            {
                DefaultCategory.Log.Warning(
                    $"[AFC] Transfer correction calculation failed: {ex.GetBaseException().Message}");
            }
            finally
            {
                Volatile.Write(ref _done, 1);
            }
        });
    }

    internal void Cancel()
    {
        _cancelled = true;
        if (Interlocked.Exchange(ref _disposed, 1) == 0)
        {
            _cts.Cancel();
            _cts.Dispose();
        }
    }

    internal void Dispose()
    {
        if (Interlocked.Exchange(ref _disposed, 1) == 0)
            _cts.Dispose();
    }

    private static bool CalculateCorrection(
        IOrbiter target,
        double requiredPeAltitude,
        OrbitalTransfers.TransferData transferData,
        FlightPlan flightPlan,
        CancellationToken stop)
    {
        PatchedConic? transferPatch = null;
        foreach (PatchedConic patch in flightPlan.Patches)
        {
            if (patch.Orbit.Parent.Id == target.Parent.Id)
            {
                transferPatch = patch;
                break;
            }
        }
        if (transferPatch == null)
        {
            DefaultCategory.Log.Warning("[AFC] Transfer correction failed: transfer orbit not found.");
            return false;
        }
        if (target is not IParentBody targetBody)
            return false;

        Orbit orbit = transferPatch.Orbit;
        double3 dvCci = CourseCorrectCci(
            orbit, transferPatch.StartTime, target, targetBody.Mu, target.MeanRadius + requiredPeAltitude, out UniverseTime burnTime, stop);
        if (stop.IsCancellationRequested || !IsFinite(dvCci) || !double.IsFinite(burnTime.Seconds()))
            return false;

        transferData.CorrectionStart = burnTime;
        StateVectors state = orbit.GetStateVectorsAt(burnTime);
        transferData.CorrectionPoint = orbit.GetPointAt(burnTime);
        doubleQuat rotation = state.GetVlf2ParentCci().OrIdentity().Inverse();
        double3 dvVlf = dvCci.Transform(rotation);
        if (!IsFinite(dvVlf))
            return false;
        transferData.CorrectionDvVlf = dvVlf;
        return true;
    }

    // Mirrors stock CorrectionBurnTask.CourseCorrectCci on the snapshotted orbit while checking cancellation.
    private static double3 CourseCorrectCci(
        Orbit orbit, UniverseTime gameTime, IOrbiter target, double targetMu, double targetPe, out UniverseTime burnTime, CancellationToken stop)
    {
        double3 coarseDv = CourseCorrectCci(orbit, gameTime, target.Orbit, out burnTime, stop);
        stop.ThrowIfCancellationRequested();
        StateVectors stateAtBurn = orbit.GetStateVectorsAt(burnTime);
        Orbit corrected = Orbit.CreateFromStateCci(
            orbit.Parent, burnTime, stateAtBurn.PositionCci, stateAtBurn.VelocityCci + coarseDv, orbit.OrbitLineColor);
        UniverseTime encounterTime = NextClosestApproach(corrected, target.Orbit, burnTime, stop);
        stop.ThrowIfCancellationRequested();

        StateVectors shipAtEnc = corrected.GetStateVectorsAt(encounterTime);
        StateVectors targetAtEnc = target.Orbit.GetStateVectorsAt(encounterTime);
        double3 relVelEnc = shipAtEnc.VelocityCci - targetAtEnc.VelocityCci;
        UniverseTime soiEntryTime = encounterTime - target.SphereOfInfluence / relVelEnc.Length();
        StateVectors shipAtSoi = corrected.GetStateVectorsAt(soiEntryTime);
        StateVectors targetAtSoi = target.Orbit.GetStateVectorsAt(soiEntryTime);
        double3 relVelSoi = shipAtSoi.VelocityCci - targetAtSoi.VelocityCci;
        double impactParameter = OrbitalTransfers.ImpactParameterAtSoiEntry(
            targetMu, target.SphereOfInfluence, relVelSoi.Length(), targetPe);
        double3 offsetDir = double3.Cross(relVelEnc, orbit.GetOrbitNormalCci()).NormalizeOrZero();
        double3 aimPos = targetAtEnc.PositionCci + impactParameter * offsetDir;

        StateVectors burnState = orbit.GetStateVectorsAt(burnTime);
        UniverseTime tof = encounterTime - burnTime;
        OrbitalTransfers.SuperiorLambert(orbit.Mu, burnState.PositionCci, aimPos, tof, out double3 departureVel, out _);
        return departureVel - burnState.VelocityCci;
    }

    private static double3 CourseCorrectCci(
        Orbit orbit, UniverseTime gameTime, Orbit target, out UniverseTime burnTime, CancellationToken stop)
    {
        UniverseTime closestTime = NextClosestApproach(orbit, target, gameTime + 1.0, stop);
        UniverseTime low = gameTime + (closestTime - gameTime) / 40L;
        UniverseTime high = gameTime + (closestTime - gameTime) * 39L / 40L;
        burnTime = new UniverseTime(MathEx.BrentMin(BurnFunc, low.Seconds(), high.Seconds(), 1e-6));
        return DeltaVToInterceptAt(orbit, burnTime, target, closestTime - burnTime);

        double BurnFunc(double testTime)
        {
            stop.ThrowIfCancellationRequested();
            return DeltaVToInterceptAt(orbit, new UniverseTime(testTime), target, closestTime - testTime).Length();
        }
    }

    private static double3 DeltaVToInterceptAt(Orbit orbit, UniverseTime startTime, Orbit target, UniverseTime deltaTime)
    {
        StateVectors ship = orbit.GetStateVectorsAt(startTime);
        StateVectors dest = target.GetStateVectorsAt(startTime + deltaTime);
        OrbitalTransfers.SuperiorLambert(orbit.Mu, ship.PositionCci, dest.PositionCci, deltaTime, out double3 departureVel, out _);
        return departureVel - ship.VelocityCci;
    }

    private static UniverseTime NextClosestApproach(Orbit a, Orbit b, UniverseTime gameTime, CancellationToken stop)
    {
        double span = a.Eccentricity > 1.0 ? 110.0 / a.GetMeanMotion() : a.Period;
        UniverseTime end = gameTime + span;
        return new UniverseTime(MathEx.BrentMin(DistFunc, gameTime.Seconds(), end.Seconds(), 1e-6));

        double DistFunc(double testTime)
        {
            stop.ThrowIfCancellationRequested();
            UniverseTime t = new(testTime);
            return (a.GetStateVectorsAt(t).PositionCci - b.GetStateVectorsAt(t).PositionCci).Length();
        }
    }

    private static bool IsFinite(double3 v)
        => double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z);
}

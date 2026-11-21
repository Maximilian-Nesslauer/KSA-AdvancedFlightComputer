using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.Transfer;

/// <summary>Scores the transfer grid without propagating a trajectory for each point.</summary>
internal sealed class PorkchopScores
{
    /// <param name="DepartureImpact">The transfer arc passes its periapsis below the surface of the departure body before it arrives, so the propagated trajectory impacts.</param>
    internal readonly record struct Cell(UniverseTime Start, UniverseTime Transit, double DepartureDv, double ArrivalSpeed,
                                         bool DepartureImpact)
    {
        internal bool Valid => Transit > UniverseTime.Zero && double.IsFinite(DepartureDv) && double.IsFinite(ArrivalSpeed);
    }

    private readonly OrbitalTransfers.TransferInfo _snapshot;
    private readonly Orbit _sourceOrbit;
    private readonly Orbit _targetOrbit;
    private readonly UniverseTime _start;
    private readonly UniverseTime _minTransit;
    private readonly UniverseTime _startStep;
    private readonly UniverseTime _transitStep;
    private int _rowsComplete;

    internal Cell[,] Data { get; }
    internal Task Completion { get; }
    internal string? Failure { get; private set; }
    internal float Progress => (float)Volatile.Read(ref _rowsComplete) / Data.GetLength(0);

    internal PorkchopScores(OrbitalTransfers.TransferInfo info, int size, UniverseTime start,
                            UniverseTime minTransit, UniverseTime startStep, UniverseTime transitStep, CancellationToken stop)
    {
        _snapshot = new OrbitalTransfers.TransferInfo(info, usePorkChopData: false);
        _snapshot.Starting = info.Starting;
        _sourceOrbit = info.Source.Orbit;
        _targetOrbit = info.Target!.Orbit;
        _start = start;
        _minTransit = minTransit;
        _startStep = startStep;
        _transitStep = transitStep;
        Data = new Cell[size, size];
        Completion = Task.Run(() => Score(stop));
    }

    private void Score(CancellationToken stop)
    {
        try
        {
            Parallel.For(0, Data.GetLength(0), new ParallelOptions
            {
                CancellationToken = stop,
                MaxDegreeOfParallelism = Math.Clamp(TransferTask.WorkerCount, 1, Data.GetLength(0)),
            }, row =>
            {
                var transfer = new OrbitalTransfers.TransferData();
                for (int column = 0; column < Data.GetLength(1) && !stop.IsCancellationRequested; column++)
                {
                    transfer.Start = _start + row * _startStep;
                    transfer.Transit = _minTransit + column * _transitStep;
                    if (Solve(ref transfer))
                        Data[row, column] = new Cell(transfer.Start, transfer.Transit,
                            transfer.TransferDvVlf.Length(), transfer.RelativeVelocityCci.Length(), transfer.Impacts);
                }
                Interlocked.Increment(ref _rowsComplete);
            });
        }
        catch (OperationCanceledException) when (stop.IsCancellationRequested)
        {
        }
        catch (Exception error)
        {
            Failure = error.GetBaseException().Message;
        }
    }

    internal OrbitalTransfers.TransferData? SolveForSelection(int row, int column)
    {
        var transfer = new OrbitalTransfers.TransferData
        {
            Start = _start + row * _startStep,
            Transit = _minTransit + column * _transitStep,
            ClosestApproachDistance = double.MaxValue,
        };
        return Solve(ref transfer) ? transfer : null;
    }

    internal bool StillCurrent(UniverseTime now)
        => !_snapshot.Vehicle.IsDisposed && SameOrbit(_snapshot.Starting, _snapshot.Vehicle.Orbit, now)
            && SameOrbit(_sourceOrbit, _snapshot.Source.Orbit, now)
            && SameOrbit(_targetOrbit, _snapshot.Target?.Orbit, now);

    internal bool MatchesDeparture(Orbit? orbit, UniverseTime time)
        => SameOrbit(_snapshot.Starting, orbit, time);

    private static bool SameOrbit(Orbit original, Orbit? current, UniverseTime now)
    {
        if (ReferenceEquals(original, current))
            return true;
        if (current == null || original.Parent != current.Parent || original.IsBound() != current.IsBound())
            return false;
        StateVectors a = original.GetStateVectorsAt(now), b = current.GetStateVectorsAt(now);
        return (a.PositionCci - b.PositionCci).Length() <= 10.0
            && (a.VelocityCci - b.VelocityCci).Length() <= 0.05;
    }

    private bool Solve(ref OrbitalTransfers.TransferData transfer)
    {
        transfer.Impacts = false;
        if (!ReferenceEquals(_snapshot.Source, _snapshot.Vehicle))
            return OrbitalTransfers.SolveLambert(_snapshot, ref transfer);

        // A same-sphere transfer uses the captured orbits because the live vehicle's orbit can change during scoring.
        StateVectors from = _sourceOrbit.GetStateVectorsAt(transfer.Start);
        StateVectors to = _targetOrbit.GetStateVectorsAt(transfer.Arrival);
        double mu = _targetOrbit.Parent.Mass * 6.6743e-11;
        OrbitalTransfers.SuperiorLambert(mu, from.PositionCci, to.PositionCci, transfer.Transit,
            out double3 departure, out double3 arrival);
        double3 excess = departure - from.VelocityCci;
        double speed = excess.Length();
        if (!double.IsFinite(speed))
            return false;
        doubleQuat vlf = from.GetVlf2ParentCci().OrIdentity();
        transfer.DepartureVelocityCci = excess;
        transfer.DeltaVelocityCci = excess;
        transfer.SourcePositionCci = from.PositionCci;
        transfer.SourceTransferVelocityCci = departure;
        transfer.CaptureVelocityCci = arrival;
        transfer.RelativeVelocityCci = arrival - to.VelocityCci;
        transfer.TransferDvVlf = excess.Transform(vlf.Inverse());
        transfer.VesselSpeed = Math.Sqrt(mu / from.PositionCci.Length());
        transfer.DepartureInclination = speed > 0.0 ? Math.Asin(Math.Clamp(excess.Z / speed, -1.0, 1.0)) : 0.0;
        transfer.Impacts = PassesPeriapsisBelow(mu, from.PositionCci, departure, to.PositionCci, _targetOrbit.Parent.MeanRadius);
        return double.IsFinite(transfer.RelativeVelocityCci.Length());
    }

    /// <summary>
    /// Whether the conic through <paramref name="position"/> with <paramref name="velocity"/> reaches its
    /// periapsis before <paramref name="arrival"/> and that periapsis is below <paramref name="radius"/>.
    /// Stock's FlightPlan then ends the departure patch with an impact (PatchedConic.SearchForParentCollision),
    /// and the trajectory check marks the transfer Impacts. A single-revolution arc moves forward in true
    /// anomaly, so it passes the periapsis when the true anomaly at arrival is below the one at departure.
    /// </summary>
    internal static bool PassesPeriapsisBelow(double mu, double3 position, double3 velocity, double3 arrival, double radius)
    {
        double3 h = double3.Cross(position, velocity);
        double3 e = double3.Cross(velocity, h) / mu - position.Normalized();
        double eccentricity = e.Length();
        double periapsis = double3.Dot(h, h) / mu / (1.0 + eccentricity);
        if (!(periapsis < radius) || eccentricity < 1e-9)
            return false;
        return TrueAnomaly(h, e, arrival) < TrueAnomaly(h, e, position);
    }

    private static double TrueAnomaly(double3 h, double3 e, double3 position)
    {
        double angle = Math.Atan2(double3.Dot(h.Normalized(), double3.Cross(e, position)), double3.Dot(e, position));
        return angle < 0.0 ? angle + 2.0 * Math.PI : angle;
    }
}

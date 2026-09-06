using Brutal.Numerics;
using AdvancedFlightComputer.Core;
using KSA;

namespace AdvancedFlightComputer.Features.Flyby;

internal static partial class FlybyTargeting
{
    internal readonly record struct OrbitGeometry(
        IParentBody Parent, double Mu, double SemiMajorAxis, double Eccentricity,
        double Inclination, double Longitude, double Argument, UniverseTime PeriapsisTime)
    {
        internal static OrbitGeometry Capture(Orbit orbit) => new(
            orbit.Parent, orbit.Mu, orbit.SemiMajorAxis, orbit.Eccentricity,
            orbit.Inclination, orbit.LongitudeOfAscendingNode, orbit.ArgumentOfPeriapsis,
            orbit.TimeAtPeriapsis);
    }

    // State at the requested departure covers orbital phase and orientation without the unstable semi-major axis of a near-parabolic parking orbit.
    // Kilometres and tenths of a metre per second suppress small integration changes during coast.
    internal readonly record struct ParkingGeometry(
        long X, long Y, long Z, long Vx, long Vy, long Vz, bool IsBound)
    {
        internal static ParkingGeometry Capture(Orbit orbit, UniverseTime start)
        {
            StateVectors state = orbit.GetStateVectorsAt(start);
            return new(Bucket(state.PositionCci.X, 1000), Bucket(state.PositionCci.Y, 1000),
                Bucket(state.PositionCci.Z, 1000), Bucket(state.VelocityCci.X, 0.1),
                Bucket(state.VelocityCci.Y, 0.1), Bucket(state.VelocityCci.Z, 0.1), orbit.IsBound());
        }

        private static long Bucket(double value, double size) => (long)System.Math.Round(value / size);
    }

    internal readonly record struct DepartureKey(
        Vehicle Source, IOrbiter Target, IParentBody Parent, double ParentMu,
        UniverseTime Start, UniverseTime Transit, double Radius, FlybySide Side,
        double TargetMu, double TargetSoi, OrbitGeometry? TargetOrbit,
        OrbitGeometry? ParentOrbit, doubleQuat ParentToCci, doubleQuat LambertToCce,
        ParkingGeometry Parking)
    {
        internal DepartureKey WithoutParkingDrift() => this with { Parking = default };
    }

    internal sealed record DepartureSolution(DepartureKey Key, FlybyOutcome Outcome);

    private static DepartureSolution? _departure;

    internal static DepartureKey CaptureDepartureKey(
        Vehicle source, IOrbiter target, UniverseTime start, UniverseTime transit,
        double radius, FlybySide side)
    {
        var body = (IParentBody)target;
        IParentBody parent = source.Orbit.Parent;
        bool cross = IsCrossParentTransfer(source, target);
        return new(source, target, parent, parent.Mu, start, transit, radius, side,
            body.Mu, body.SphereOfInfluence, target.Orbit?.Parent != null ? OrbitGeometry.Capture(target.Orbit) : null,
            cross && parent is Celestial celestial && celestial.Orbit?.Parent != null
                ? OrbitGeometry.Capture(celestial.Orbit) : null,
            parent.GetCce2Cci(), target.Orbit?.Parent?.GetCci2Cce() ?? default,
            ParkingGeometry.Capture(source.Orbit, start));
    }

    internal static DepartureSolution GetDeparture(DepartureKey key)
    {
        if (_departure is { } cached && cached.Key.WithoutParkingDrift() == key.WithoutParkingDrift())
        {
            if (cached.Key.Parking == key.Parking || IsDepartureThrusting(key.Source))
                return cached;
        }

        FlybyOutcome outcome;
        try
        {
            outcome = ComputeFlybyDeparture(
                key.Source, key.Target, key.Start, key.Transit, key.Radius, key.Side);
        }
        catch (System.Exception ex)
        {
            LogHelper.WarnOnce("flyby-departure:" + ex.GetType().Name,
                $"[AFC] Flyby departure: source='{key.Source.Id}' target='{(key.Target as Astronomical)?.Id}' " +
                $"start={key.Start.Seconds():F1}s transit={key.Transit.Seconds():F1}s radius={key.Radius:F0}m side={key.Side}: {ex}");
            outcome = FlybyOutcome.Unavailable;
        }
        return _departure = new(key, outcome);
    }

    // Explicit request changes always bypass this freeze. Only the parking state may stay fixed during thrust.
    private static bool IsDepartureThrusting(Vehicle source) =>
        source.FlightComputer.BurnMode == FlightComputerBurnMode.Auto || source.GetManualThrottle() > 0f;

    internal static void ResetDepartureCache() => _departure = null;
}

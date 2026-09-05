using System;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

internal static class OrbitManeuvers
{
    public record struct ManeuverResult(double3 DvCci, double3 DvVlf, UniverseTime BurnTime);

    public enum InclinationReference { Ecliptic, Equatorial }

    // CCI +Z is the ecliptic normal. The equatorial normal is CCE +Z transformed to CCI.
    public static double3 GetReferenceNormalCci(Orbit orbit, InclinationReference reference)
    {
        if (reference == InclinationReference.Ecliptic)
            return double3.UnitZ;
        return double3.UnitZ.Transform(orbit.Parent.GetCce2Cci());
    }

    public static double GetInclinationAgainst(Orbit orbit, InclinationReference reference)
    {
        double3 referenceNormal = GetReferenceNormalCci(orbit, reference);
        double3 orbitNormal = orbit.GetOrbitNormalCci();
        return MathEx.SafeAcos(double3.Dot(referenceNormal, orbitNormal));
    }

    public static ManeuverResult? ComputeSetPeriapsis(
        Orbit orbit, double targetAltitudeMeters, double parentRadius, UniverseTime now)
    {
        if (orbit.Eccentricity >= 1.0)
            return null;

        double currentApRadius = orbit.Apoapsis;
        double newPeRadius = targetAltitudeMeters + parentRadius;

        if (newPeRadius <= 0.0 || newPeRadius >= currentApRadius)
            return null;

        if (orbit.GetNextApoapsisTime(now) is not UniverseTime burnTime)
            return null;
        return ComputeApseBurn(orbit, burnTime, currentApRadius, newPeRadius);
    }

    public static ManeuverResult? ComputeSetApoapsis(
        Orbit orbit, double targetAltitudeMeters, double parentRadius, UniverseTime now)
    {
        if (orbit.Eccentricity >= 1.0)
            return null;

        double currentPeRadius = orbit.Periapsis;
        double newApRadius = targetAltitudeMeters + parentRadius;

        if (newApRadius <= currentPeRadius)
            return null;

        if (orbit.GetNextPeriapsisTime(now) is not UniverseTime burnTime)
            return null;
        return ComputeApseBurn(orbit, burnTime, currentPeRadius, newApRadius);
    }

    // The 0.001 eccentricity tolerance matches CircularizeIntent.IsSatisfied and the UI.
    public static ManeuverResult? ComputeCircularize(
        Orbit orbit, bool useApoapsis, UniverseTime now)
    {
        if (orbit.Eccentricity >= 1.0)
            return null;
        if (orbit.Eccentricity < 0.001)
            return null;

        UniverseTime? apsisTime = useApoapsis
            ? orbit.GetNextApoapsisTime(now)
            : orbit.GetNextPeriapsisTime(now);
        if (apsisTime is not UniverseTime burnTime)
            return null;

        double3 dvCci = OrbitalTransfers.DvCciToCircularize(orbit, burnTime);
        if (dvCci.LengthSquared() < 1e-12)
            return null;

        double3 dvVlf = CciToVlf(dvCci, orbit, burnTime);
        return new ManeuverResult(dvCci, dvVlf, burnTime);
    }

    // Preserve speed and rotate velocity into the target plane. The fraction scales the rotation angle for partial plane changes.
    public static ManeuverResult? ComputeMatchInclination(
        Orbit vehicleOrbit, Orbit targetOrbit, bool useDescendingNode, UniverseTime now,
        double fraction = 1.0)
    {
        // Orbit.TimeOfTrueAnomaly does not advance past node times for an unbound orbit.
        if (vehicleOrbit.Eccentricity >= 1.0)
            return null;

        double relInc = vehicleOrbit.GetRelativeInclination(targetOrbit).Value();
        if (relInc < 0.001)
            return null;

        TrueAnomaly nodeTa = useDescendingNode
            ? vehicleOrbit.GetDescendingNode(targetOrbit)
            : vehicleOrbit.GetAscendingNode(targetOrbit);

        if (vehicleOrbit.TimeOfTrueAnomaly(nodeTa, now) is not UniverseTime nodeTime)
            return null;
        StateVectors sv = vehicleOrbit.GetStateVectorsAt(nodeTime);

        double3 vehicleNormal = vehicleOrbit.GetOrbitNormalCci();
        double3 targetNormal = targetOrbit.GetOrbitNormalCci();
        double3 rotAxis = double3.Cross(vehicleNormal, targetNormal).NormalizeOrZero();

        if (rotAxis.LengthSquared() < 1e-12)
            return null;

        doubleQuat planeChange = QuaternionEx.AngleAxis(relInc * fraction, rotAxis);
        double3 targetVel = sv.VelocityCci.Transform(planeChange);
        double3 dvCci = targetVel - sv.VelocityCci;

        double3 dvVlf = CciToVlf(dvCci, vehicleOrbit, nodeTime);
        return new ManeuverResult(dvCci, dvVlf, nodeTime);
    }

    // Preserve speed and the node line. The fraction scales the rotation angle for partial plane changes.
    public static ManeuverResult? ComputeSetInclination(
        Orbit orbit, double targetInclinationRad, bool useDescendingNode, UniverseTime now,
        InclinationReference reference, double fraction = 1.0)
    {
        if (orbit.Eccentricity >= 1.0)
            return null;

        targetInclinationRad = Math.Clamp(targetInclinationRad, 0.0, Math.PI);

        double currentInc = GetInclinationAgainst(orbit, reference);
        double incDiff = Math.Abs(targetInclinationRad - currentInc);
        if (incDiff < 0.001)
            return null;

        double3 vehicleNormal = orbit.GetOrbitNormalCci();
        double3 referenceNormal = GetReferenceNormalCci(orbit, reference);

        double3 nodeDir = double3.Cross(referenceNormal, vehicleNormal).NormalizeOrZero();
        if (nodeDir.LengthSquared() < 1e-12)
            // Use CCI +X when the orbit is coplanar because its node line is undefined.
            nodeDir = new double3(1, 0, 0);

        TrueAnomaly anTa = orbit.GetTrueAnomaly(nodeDir);
        TrueAnomaly nodeTa = useDescendingNode
            ? new TrueAnomaly((anTa.Value() + Math.PI) % (Math.PI * 2.0))
            : anTa;

        if (orbit.TimeOfTrueAnomaly(nodeTa, now) is not UniverseTime nodeTime)
            return null;
        StateVectors sv = orbit.GetStateVectorsAt(nodeTime);

        // Rotate about the node line to preserve it while setting the target inclination.
        doubleQuat tilt = QuaternionEx.AngleAxis(targetInclinationRad, nodeDir);
        double3 targetNormal = referenceNormal.Transform(tilt);

        double3 rotAxis = double3.Cross(vehicleNormal, targetNormal).NormalizeOrZero();
        if (rotAxis.LengthSquared() < 1e-12)
            return null;

        double rotAngle = MathEx.Angle(vehicleNormal, targetNormal).Value();
        doubleQuat planeChange = QuaternionEx.AngleAxis(rotAngle * fraction, rotAxis);
        double3 targetVel = sv.VelocityCci.Transform(planeChange);
        double3 dvCci = targetVel - sv.VelocityCci;

        double3 dvVlf = CciToVlf(dvCci, orbit, nodeTime);
        return new ManeuverResult(dvCci, dvVlf, nodeTime);
    }

    // An unreachable node has no time.
    public static (TrueAnomaly anTa, TrueAnomaly dnTa, UniverseTime? anTime, UniverseTime? dnTime)
        GetReferenceNodes(Orbit orbit, UniverseTime now, InclinationReference reference)
    {
        double3 vehicleNormal = orbit.GetOrbitNormalCci();
        double3 referenceNormal = GetReferenceNormalCci(orbit, reference);
        double3 nodeDir = double3.Cross(referenceNormal, vehicleNormal).NormalizeOrZero();

        if (nodeDir.LengthSquared() < 1e-12)
            // Use the same coplanar node convention as ComputeSetInclination.
            nodeDir = new double3(1, 0, 0);

        TrueAnomaly anTa = orbit.GetTrueAnomaly(nodeDir);
        TrueAnomaly dnTa = new TrueAnomaly((anTa.Value() + Math.PI) % (Math.PI * 2.0));
        UniverseTime? anTime = orbit.TimeOfTrueAnomaly(anTa, now);
        UniverseTime? dnTime = orbit.TimeOfTrueAnomaly(dnTa, now);

        return (anTa, dnTa, anTime, dnTime);
    }

    #region Helpers

    // Place the burn at the apse with burnRadius so the radius in the vis viva equation matches the position radius.
    private static ManeuverResult? ComputeApseBurn(
        Orbit orbit, UniverseTime burnTime, double burnRadius, double oppositeRadius)
    {
        double newSma = (burnRadius + oppositeRadius) / 2.0;
        if (newSma <= 0.0)
            return null;

        StateVectors sv = orbit.GetStateVectorsAt(burnTime);
        double vNew = Math.Sqrt(orbit.Mu * (2.0 / burnRadius - 1.0 / newSma));
        double3 vDir = sv.VelocityCci.NormalizeOrZero();
        double3 dvCci = vDir * vNew - sv.VelocityCci;

        double3 dvVlf = CciToVlf(dvCci, orbit, burnTime);
        return new ManeuverResult(dvCci, dvVlf, burnTime);
    }

    private static double3 CciToVlf(double3 dvCci, Orbit orbit, UniverseTime time)
    {
        doubleQuat parentCci2Vlf = orbit.GetStateVectorsAt(time).GetVlf2ParentCci().OrIdentity().Inverse();
        return dvCci.Transform(parentCci2Vlf);
    }

    #endregion
}

using System;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.ManeuverTools;

/// <summary>
/// Pure dV calculations for maneuver planning. All methods are stateless
/// and return nullable results to signal invalid inputs (e.g. hyperbolic
/// orbit for apse maneuvers, coplanar orbits for inclination matching).
/// </summary>
static class OrbitManeuvers
{
    public record struct ManeuverResult(double3 DvCci, double3 DvVlf, SimTime BurnTime);

    /// <summary>
    /// Computes a prograde/retrograde burn at the next apoapsis to set periapsis
    /// to a target altitude above the parent body's surface.
    /// </summary>
    public static ManeuverResult? ComputeSetPeriapsis(
        Orbit orbit, double targetAltitudeMeters, double parentRadius, SimTime now)
    {
        if (orbit.Eccentricity >= 1.0)
            return null;

        double currentApRadius = orbit.Apoapsis;
        double newPeRadius = targetAltitudeMeters + parentRadius;

        if (newPeRadius <= 0.0 || newPeRadius >= currentApRadius)
            return null;

        SimTime burnTime = orbit.GetNextApoapsisTime(now);
        return ComputeApseBurn(orbit, burnTime, currentApRadius, newPeRadius);
    }

    /// <summary>
    /// Computes a prograde/retrograde burn at the next periapsis to set apoapsis
    /// to a target altitude above the parent body's surface.
    /// </summary>
    public static ManeuverResult? ComputeSetApoapsis(
        Orbit orbit, double targetAltitudeMeters, double parentRadius, SimTime now)
    {
        if (orbit.Eccentricity >= 1.0)
            return null;

        double currentPeRadius = orbit.Periapsis;
        double newApRadius = targetAltitudeMeters + parentRadius;

        if (newApRadius <= currentPeRadius)
            return null;

        SimTime burnTime = orbit.GetNextPeriapsisTime(now);
        return ComputeApseBurn(orbit, burnTime, currentPeRadius, newApRadius);
    }

    /// <summary>
    /// Computes a plane-change burn at the ascending or descending node to match
    /// a target orbit's inclination. Preserves orbital speed, only rotates the
    /// velocity vector into the target's orbital plane.
    /// </summary>
    public static ManeuverResult? ComputeMatchInclination(
        Orbit vehicleOrbit, Orbit targetOrbit, bool useDescendingNode, SimTime now)
    {
        double relInc = vehicleOrbit.GetRelativeInclination(targetOrbit).Value();
        if (relInc < 0.001)
            return null;

        TrueAnomaly nodeTa = useDescendingNode
            ? vehicleOrbit.GetDescendingNode(targetOrbit)
            : vehicleOrbit.GetAscendingNode(targetOrbit);

        if (nodeTa == TrueAnomaly.NaN)
            return null;

        SimTime nodeTime = vehicleOrbit.TimeOfTrueAnomaly(nodeTa, now);
        StateVectors sv = vehicleOrbit.GetStateVectorsAt(nodeTime);

        double3 vehicleNormal = vehicleOrbit.GetOrbitNormalCci();
        double3 targetNormal = targetOrbit.GetOrbitNormalCci();
        double3 rotAxis = double3.Cross(vehicleNormal, targetNormal).NormalizeOrZero();

        if (rotAxis.LengthSquared() < 1e-12)
            return null;

        doubleQuat planeChange = QuaternionEx.AngleAxis(relInc, rotAxis);
        double3 targetVel = sv.VelocityCci.Transform(planeChange);
        double3 dvCci = targetVel - sv.VelocityCci;

        double3 dvVlf = CciToVlf(dvCci, vehicleOrbit, nodeTime);
        return new ManeuverResult(dvCci, dvVlf, nodeTime);
    }

    /// <summary>
    /// Builds a PorkChopEntry + TransferInfo for use with the stock Create button.
    /// Follows the exact same pattern as stock Circularize (TransferPlanner.cs:389-412).
    /// </summary>
    public static (OrbitalTransfers.PorkChopEntry entry, OrbitalTransfers.TransferInfo info)
        BuildTransferEntry(Vehicle source, ManeuverResult maneuver)
    {
        var transferData = new OrbitalTransfers.TransferData
        {
            Start = maneuver.BurnTime,
            Point = source.Orbit.GetPointAt(maneuver.BurnTime),
            DeltaVelocityCci = maneuver.DvCci,
            TransferDvVlf = maneuver.DvVlf
        };

        var info = new OrbitalTransfers.TransferInfo(source, source, source, usePorkChopData: false);
        info.PorkChopData = new OrbitalTransfers.PorkChopEntry[1, 1];

        FlightPlan flightPlan = FlightPlan.CreateUninitialized(source.Hash);
        OrbitalTransfers.BuildFlightPlan(
            ref flightPlan, info, transferData.Start, transferData.TransferDvVlf,
            out _, out _);

        var entry = new OrbitalTransfers.PorkChopEntry(transferData, flightPlan);
        info.PorkChopData[0, 0] = entry;

        return (entry, info);
    }

    /// <summary>
    /// Computes a plane-change burn at the ascending or descending node (relative
    /// to the parent body's equatorial plane) to set the orbit's inclination to
    /// a specific angle. Preserves orbital speed and LAN.
    /// </summary>
    public static ManeuverResult? ComputeSetInclination(
        Orbit orbit, double targetInclinationRad, bool useDescendingNode, SimTime now)
    {
        targetInclinationRad = Math.Clamp(targetInclinationRad, 0.0, Math.PI);

        double currentInc = orbit.Inclination;
        double incDiff = Math.Abs(targetInclinationRad - currentInc);
        if (incDiff < 0.001)
            return null;

        double3 vehicleNormal = orbit.GetOrbitNormalCci();
        double3 equatorialNormal = new double3(0, 0, 1);

        // AN/DN relative to equatorial plane
        double3 nodeDir = double3.Cross(equatorialNormal, vehicleNormal).NormalizeOrZero();
        if (nodeDir.LengthSquared() < 1e-12)
            nodeDir = new double3(1, 0, 0); // fallback for equatorial orbits

        TrueAnomaly anTa = orbit.GetTrueAnomaly(nodeDir);
        TrueAnomaly nodeTa = useDescendingNode
            ? new TrueAnomaly((anTa.Value() + Math.PI) % (Math.PI * 2.0))
            : anTa;

        SimTime nodeTime = orbit.TimeOfTrueAnomaly(nodeTa, now);
        StateVectors sv = orbit.GetStateVectorsAt(nodeTime);

        // Target normal: same LAN, new inclination
        double lan = orbit.LongitudeOfAscendingNode;
        double3 targetNormal = new double3(
            Math.Sin(targetInclinationRad) * Math.Sin(lan),
            -Math.Sin(targetInclinationRad) * Math.Cos(lan),
            Math.Cos(targetInclinationRad));

        double3 rotAxis = double3.Cross(vehicleNormal, targetNormal).NormalizeOrZero();
        if (rotAxis.LengthSquared() < 1e-12)
            return null;

        double rotAngle = MathEx.Angle(vehicleNormal, targetNormal).Value();
        doubleQuat planeChange = QuaternionEx.AngleAxis(rotAngle, rotAxis);
        double3 targetVel = sv.VelocityCci.Transform(planeChange);
        double3 dvCci = targetVel - sv.VelocityCci;

        double3 dvVlf = CciToVlf(dvCci, orbit, nodeTime);
        return new ManeuverResult(dvCci, dvVlf, nodeTime);
    }

    /// <summary>
    /// Computes AN/DN true anomalies and times relative to the equatorial plane.
    /// Used by the UI to display both node options for Set Inclination.
    /// </summary>
    public static (TrueAnomaly anTa, TrueAnomaly dnTa, SimTime anTime, SimTime dnTime)
        GetEquatorialNodes(Orbit orbit, SimTime now)
    {
        double3 vehicleNormal = orbit.GetOrbitNormalCci();
        double3 equatorialNormal = new double3(0, 0, 1);
        double3 nodeDir = double3.Cross(equatorialNormal, vehicleNormal).NormalizeOrZero();

        if (nodeDir.LengthSquared() < 1e-12)
            nodeDir = new double3(1, 0, 0);

        TrueAnomaly anTa = orbit.GetTrueAnomaly(nodeDir);
        TrueAnomaly dnTa = new TrueAnomaly((anTa.Value() + Math.PI) % (Math.PI * 2.0));
        SimTime anTime = orbit.TimeOfTrueAnomaly(anTa, now);
        SimTime dnTime = orbit.TimeOfTrueAnomaly(dnTa, now);

        return (anTa, dnTa, anTime, dnTime);
    }

    #region Helpers

    /// <summary>
    /// Generic apse burn: given the burn radius and the opposite apse radius,
    /// compute the dV needed to create an orbit passing through both.
    /// </summary>
    private static ManeuverResult? ComputeApseBurn(
        Orbit orbit, SimTime burnTime, double burnRadius, double oppositeRadius)
    {
        StateVectors sv = orbit.GetStateVectorsAt(burnTime);
        double r = sv.PositionCci.Length();
        double newSma = (burnRadius + oppositeRadius) / 2.0;

        if (newSma <= 0.0)
            return null;

        double vNew = Math.Sqrt(orbit.Mu * (2.0 / r - 1.0 / newSma));
        double3 vDir = sv.VelocityCci.NormalizeOrZero();
        double3 dvCci = vDir * vNew - sv.VelocityCci;

        double3 dvVlf = CciToVlf(dvCci, orbit, burnTime);
        return new ManeuverResult(dvCci, dvVlf, burnTime);
    }

    /// <summary>
    /// Converts a dV vector from CCI frame to VLF frame, using the same
    /// transformation as stock Circularize (TransferPlanner.cs:394-396).
    /// </summary>
    private static double3 CciToVlf(double3 dvCci, Orbit orbit, SimTime time)
    {
        doubleQuat vlf2Cci = orbit.GetStateVectorsAt(time).GetVlf2ParentCci().OrIdentity();
        return dvCci.Transform(vlf2Cci.Inverse());
    }

    #endregion
}

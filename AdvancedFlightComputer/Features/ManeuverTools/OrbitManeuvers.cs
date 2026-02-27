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

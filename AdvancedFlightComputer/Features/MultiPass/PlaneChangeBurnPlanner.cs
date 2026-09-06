using System;
using System.Globalization;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

internal static class PlaneChangeBurnPlanner
{
    public static PassPreviewResult PlanForMatch(
        Vehicle source, Orbit targetOrbit, bool useDescendingNode,
        PassAllocation[] allocations, UniverseTime now) =>
        PlanForMatch(source, targetOrbit, useDescendingNode, allocations, now, false);

    public static PassPreviewResult PlanForMatch(
        Vehicle source, Orbit targetOrbit, bool useDescendingNode,
        PassAllocation[] allocations, UniverseTime now, bool execution)
    {
        return MultiPassForwardChainPlanner.PlanForwardChain(source, allocations, now,
            (orbit, dvCap, earliestTime) =>
            {
                if (orbit.Eccentricity >= 1.0) return null;
                double relInc = orbit.GetRelativeInclination(targetOrbit).Value();
                if (relInc < 0.001) return null;

                TrueAnomaly nodeTa = useDescendingNode
                    ? orbit.GetDescendingNode(targetOrbit)
                    : orbit.GetAscendingNode(targetOrbit);
                if (orbit.TimeOfTrueAnomaly(nodeTa, earliestTime) is not UniverseTime nodeTime)
                    return null;

                double3 rotAxis = double3.Cross(
                    orbit.GetOrbitNormalCci(),
                    targetOrbit.GetOrbitNormalCci()).NormalizeOrZero();
                if (rotAxis.LengthSquared() < 1e-12) return null;

                return BuildStep(orbit, nodeTime, rotAxis, relInc, dvCap,
                    f => OrbitManeuvers.ComputeMatchInclination(
                        orbit, targetOrbit, useDescendingNode, earliestTime, f),
                    "Match");
            }, execution);
    }

    public static PassPreviewResult PlanForSet(
        Vehicle source, double targetInclinationRad,
        OrbitManeuvers.InclinationReference reference, bool useDescendingNode,
        PassAllocation[] allocations, UniverseTime now) =>
        PlanForSet(source, targetInclinationRad, reference, useDescendingNode, allocations, now, false);

    public static PassPreviewResult PlanForSet(
        Vehicle source, double targetInclinationRad,
        OrbitManeuvers.InclinationReference reference, bool useDescendingNode,
        PassAllocation[] allocations, UniverseTime now, bool execution)
    {
        return MultiPassForwardChainPlanner.PlanForwardChain(source, allocations, now,
            (orbit, dvCap, earliestTime) =>
            {
                if (orbit.Eccentricity >= 1.0) return null;

                double clampedTargetInc = Math.Clamp(targetInclinationRad, 0.0, Math.PI);
                double currentInc = OrbitManeuvers.GetInclinationAgainst(orbit, reference);
                if (Math.Abs(clampedTargetInc - currentInc) < 0.001) return null;

                double3 vehicleNormal = orbit.GetOrbitNormalCci();
                double3 referenceNormal = OrbitManeuvers.GetReferenceNormalCci(orbit, reference);
                double3 nodeDir = double3.Cross(referenceNormal, vehicleNormal).NormalizeOrZero();
                if (nodeDir.LengthSquared() < 1e-12)
                    nodeDir = new double3(1, 0, 0);

                TrueAnomaly anTa = orbit.GetTrueAnomaly(nodeDir);
                TrueAnomaly nodeTa = useDescendingNode
                    ? new TrueAnomaly((anTa.Value() + Math.PI) % (Math.PI * 2.0))
                    : anTa;
                if (orbit.TimeOfTrueAnomaly(nodeTa, earliestTime) is not UniverseTime nodeTime)
                    return null;
                doubleQuat tilt = QuaternionEx.AngleAxis(clampedTargetInc, nodeDir);
                double3 targetNormal = referenceNormal.Transform(tilt);
                double3 rotAxis = double3.Cross(vehicleNormal, targetNormal).NormalizeOrZero();
                if (rotAxis.LengthSquared() < 1e-12) return null;
                double fullAngle = MathEx.Angle(vehicleNormal, targetNormal).Value();

                return BuildStep(orbit, nodeTime, rotAxis, fullAngle, dvCap,
                    f => OrbitManeuvers.ComputeSetInclination(
                        orbit, clampedTargetInc, useDescendingNode, earliestTime, reference, f),
                    "Set");
            }, execution);
    }

    private static PassStep? BuildStep(
        Orbit orbit, UniverseTime nodeTime, double3 rotAxis, double fullAngle,
        double dvCapMs,
        Func<double, OrbitManeuvers.ManeuverResult?> computeWithFraction,
        string logLabel)
    {
        StateVectors sv = orbit.GetStateVectorsAt(nodeTime);
        double3 vPerpVec = sv.VelocityCci - double3.Dot(sv.VelocityCci, rotAxis) * rotAxis;
        double vPerp = vPerpVec.Length();
        // Use theta = 2 * asin(deltaV / (2 * vPerp)) because velocity along the node axis does not rotate.
        double sinHalfTheta = vPerp >= 1e-3
            ? Math.Min(dvCapMs / (2.0 * vPerp), 0.9999)
            : 0.0;
        double theta = 2.0 * Math.Asin(sinHalfTheta);
        double fraction = fullAngle > 1e-9 ? Math.Min(theta / fullAngle, 1.0) : 0.0;

        OrbitManeuvers.ManeuverResult? m = computeWithFraction(fraction);

        if (MultiPassDebug.Enabled)
        {
            DefaultCategory.Log.Debug(string.Format(CultureInfo.InvariantCulture,
                "[AFC] PlaneChange.{0}: " +
                "preOrbit[SMA={1:F0} e={2:F6} Pe={3:F0} Ap={4:F0}] " +
                "geom[fullAngle={5:F4}deg nodeTime={6:F1}s rotAxis=({7:F4},{8:F4},{9:F4})] " +
                "v[|v|={10:F3} |vPerp|={11:F3}] " +
                "budget[dvCap={12:F3} theta={13:F4}deg fraction={14:F4}] " +
                "dv[{15}]",
                logLabel,
                orbit.SemiMajorAxis, orbit.Eccentricity, orbit.Periapsis, orbit.Apoapsis,
                fullAngle * 180.0 / Math.PI, nodeTime.Seconds(),
                rotAxis.X, rotAxis.Y, rotAxis.Z,
                sv.VelocityCci.Length(), vPerp,
                dvCapMs, theta * 180.0 / Math.PI, fraction,
                m != null
                    ? string.Format(CultureInfo.InvariantCulture,
                        "|dvCci|={0:F3} |dvVlf|={1:F3} burnTime={2:F1}s",
                        m.Value.DvCci.Length(), m.Value.DvVlf.Length(), m.Value.BurnTime.Seconds())
                    : "null"));
        }

        if (m == null) return null;
        return new PassStep(m.Value.BurnTime, m.Value.DvVlf);
    }
}

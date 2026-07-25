using System;
using System.Globalization;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Features.ManeuverTools;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace AdvancedFlightComputer.Features.MultiPass;

/// <summary>
/// Forward-chains N plane-change burns via
/// <see cref="MultiPassForwardChainPlanner"/>. Two entry points:
/// PlanForMatch (target = another orbit's plane) and PlanForSet
/// (target = absolute inclination against Ecliptic/Equatorial). Both
/// share the same dV -> rotation-fraction conversion: the splitter
/// hands the planner a per-pass dV budget; the planner converts that
/// to a fraction of the remaining angle using the velocity component
/// perpendicular to the rotation axis at the chosen node.
/// </summary>
internal static class PlaneChangeBurnPlanner
{
    public static PassPreviewResult PlanForMatch(
        Vehicle source, Orbit targetOrbit, bool useDescendingNode,
        PassAllocation[] allocations, SimTime now)
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
                SimTime nodeTime = orbit.TimeOfTrueAnomaly(nodeTa, earliestTime);

                double3 rotAxis = double3.Cross(
                    orbit.GetOrbitNormalCci(),
                    targetOrbit.GetOrbitNormalCci()).NormalizeOrZero();
                if (rotAxis.LengthSquared() < 1e-12) return null;

                return BuildStep(orbit, nodeTime, rotAxis, relInc, dvCap,
                    f => OrbitManeuvers.ComputeMatchInclination(
                        orbit, targetOrbit, useDescendingNode, earliestTime, f),
                    "Match");
            });
    }

    public static PassPreviewResult PlanForSet(
        Vehicle source, double targetInclinationRad,
        OrbitManeuvers.InclinationReference reference, bool useDescendingNode,
        PassAllocation[] allocations, SimTime now)
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
                SimTime nodeTime = orbit.TimeOfTrueAnomaly(nodeTa, earliestTime);

                // Reconstruct the same target normal that ComputeSetInclination
                // will compute so rotAxis / fullAngle here match what the actual
                // maneuver will rotate by.
                doubleQuat tilt = QuaternionEx.AngleAxis(clampedTargetInc, nodeDir);
                double3 targetNormal = referenceNormal.Transform(tilt);
                double3 rotAxis = double3.Cross(vehicleNormal, targetNormal).NormalizeOrZero();
                if (rotAxis.LengthSquared() < 1e-12) return null;
                double fullAngle = MathEx.Angle(vehicleNormal, targetNormal).Value();

                return BuildStep(orbit, nodeTime, rotAxis, fullAngle, dvCap,
                    f => OrbitManeuvers.ComputeSetInclination(
                        orbit, clampedTargetInc, useDescendingNode, earliestTime, reference, f),
                    "Set");
            });
    }

    /// <summary>Shared dV -> fraction conversion + maneuver invocation.
    /// The caller supplies how to derive the rotation geometry; this
    /// helper does the v_perp calc and the fraction clamp.
    /// <paramref name="logLabel"/> is included in the per-pass debug log
    /// to distinguish Match vs Set entries.</summary>
    private static PassStep? BuildStep(
        Orbit orbit, SimTime nodeTime, double3 rotAxis, double fullAngle,
        double dvCapMs,
        Func<double, OrbitManeuvers.ManeuverResult?> computeWithFraction,
        string logLabel)
    {
        StateVectors sv = orbit.GetStateVectorsAt(nodeTime);
        double3 vPerpVec = sv.VelocityCci - double3.Dot(sv.VelocityCci, rotAxis) * rotAxis;
        double vPerp = vPerpVec.Length();

        // theta = 2 * asin(dv / (2 * v_perp)), where v_perp is the velocity
        // component perpendicular to the
        // rotation axis at the node (NOT the full |v|; the radial-velocity
        // component lies along the node line and does not rotate).
        double sinHalfTheta = vPerp >= 1e-3
            ? Math.Min(dvCapMs / (2.0 * vPerp), 0.9999)
            : 0.0;
        double theta = 2.0 * Math.Asin(sinHalfTheta);
        double fraction = fullAngle > 1e-9 ? Math.Min(theta / fullAngle, 1.0) : 0.0;

        OrbitManeuvers.ManeuverResult? m = computeWithFraction(fraction);

        if (DebugConfig.MultiPass)
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

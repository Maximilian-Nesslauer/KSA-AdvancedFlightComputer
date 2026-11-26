using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>
/// Pure decision guards for guidance mode transitions, vacuum climb attitude, aero error retries, and emergency braking.
/// </summary>
public static class GuidanceModeGuards
{
    /// <summary>Dynamic pressure ceiling below which a climbing glide holds a descending-entry attitude instead of pointing engines upward, Pa.</summary>
    public const double VacuumClimbHoldMaxQPa = 10.0;

    /// <summary>Stopping-distance margin factor for the entry emergency braking backstop.</summary>
    public const double EmergencyBrakeMargin = 1.35;

    /// <summary>Reaction and spool lead time added to the emergency braking trigger altitude, s.</summary>
    public const double EmergencyBrakeLeadS = 0.5;

    /// <summary>Minimum downward speed required before the emergency braking backstop can trigger, m/s.</summary>
    public const double EmergencyBrakeMinSinkMs = 5.0;

    /// <summary>Target gentle sink rate for hoverable craft during emergency braking near zero vertical speed, m/s.</summary>
    public const double EmergencyBrakeTargetSinkMs = 0.5;

    /// <summary>
    /// Decides whether touchdown guidance cuts a lit engine on a positive vertical speed.
    /// Non-hoverable vehicles cut as soon as descent stops. Hoverable vehicles only cut when no live plan is active.
    /// </summary>
    public static bool ShouldCutSixDofOnClimb(double verticalSpeedMs, double minTwr, bool hasLivePlan,
                                              double climbCutMs, double hoverMaxMinTwr)
    {
        if (minTwr > hoverMaxMinTwr && verticalSpeedMs >= 0.0)
            return true;
        return !hasLivePlan && verticalSpeedMs > climbCutMs;
    }

    /// <summary>
    /// Returns true when dynamic pressure is negligible and the booster is still climbing, so pointing along -vAir would flip engines upward.
    /// </summary>
    public static bool ShouldHoldVacuumClimbAttitude(double dynamicPressurePa, double verticalSpeedMs)
        => double.IsFinite(dynamicPressurePa) && dynamicPressurePa < VacuumClimbHoldMaxQPa
           && double.IsFinite(verticalSpeedMs) && verticalSpeedMs > 0.0;

    /// <summary>
    /// Computes the unit thrust-axis direction for a climbing booster above the atmosphere.
    /// Mirrors the surface-relative velocity across the local horizon so engines stay downward and join retrograde continuously at apogee.
    /// </summary>
    public static bool VacuumClimbHoldDirection(double vx, double vy, double vz,
                                                double rx, double ry, double rz,
                                                out double ux, out double uy, out double uz)
    {
        ux = uy = uz = 0.0;
        double rNorm = Math.Sqrt(rx * rx + ry * ry + rz * rz);
        if (!(rNorm > 1e-9))
            return false;
        double hx = rx / rNorm, hy = ry / rNorm, hz = rz / rNorm;
        double vUp = vx * hx + vy * hy + vz * hz;
        // Reflect v across the local horizontal plane: vMirror = v - 2 * vUp * rHat, then point thrust axis along -vMirror.
        double dx = -(vx - 2.0 * vUp * hx);
        double dy = -(vy - 2.0 * vUp * hy);
        double dz = -(vz - 2.0 * vUp * hz);
        double dNorm = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        if (!(dNorm > 1e-9))
            return false;
        ux = dx / dNorm;
        uy = dy / dNorm;
        uz = dz / dNorm;
        return true;
    }

    /// <summary>
    /// Decides whether the boostback aero resampler starts a sweep.
    /// Retries after retryAt when a previous error is present on a non-stale table, unless the mesh is actively rebuilding.
    /// </summary>
    public static bool ShouldStartAeroSweep(bool force, double now, double retryAt,
                                            bool hasError, bool meshActivelyRebuilding,
                                            bool wrongKind, bool isStale)
    {
        if (force)
            return true;
        if (now < retryAt)
            return false;
        if (wrongKind || isStale)
            return true;
        return hasError && !meshActivelyRebuilding;
    }

    /// <summary>
    /// Returns true when an unpowered entry glide without a live plan has descended to the emergency braking altitude.
    /// </summary>
    public static bool ShouldTriggerEmergencyBrake(double legsAltM, double verticalSpeedMs,
                                                   double maxThrustN, double massKg, double gravityMs2)
    {
        if (!double.IsFinite(legsAltM) || !double.IsFinite(verticalSpeedMs)
            || !double.IsFinite(maxThrustN) || !double.IsFinite(massKg) || !double.IsFinite(gravityMs2))
            return false;
        if (!(massKg > 0.0) || !(maxThrustN > 0.0) || !(gravityMs2 >= 0.0))
            return false;
        if (!(verticalSpeedMs < -EmergencyBrakeMinSinkMs))
            return false;
        double sink = -verticalSpeedMs;
        double aNet = maxThrustN / massKg - gravityMs2;
        if (aNet <= 0.5)
            return legsAltM <= sink * 10.0;
        double hStop = sink * sink / (2.0 * aNet);
        double hTrigger = EmergencyBrakeMargin * hStop + sink * EmergencyBrakeLeadS;
        return legsAltM <= hTrigger;
    }

    /// <summary>
    /// Computes the thrust magnitude required by the emergency braking backstop, N.
    /// Returns zero to cut a non-hoverable engine once descent stops, or modulates hoverable craft down to a gentle sink.
    /// </summary>
    public static double ComputeEmergencyBrakeDemandN(double legsAltM, double verticalSpeedMs,
                                                      double massKg, double gravityMs2,
                                                      double verticalCos, double minTwr, double hoverMaxMinTwr)
    {
        if (!(massKg > 0.0) || !(gravityMs2 >= 0.0))
            return 0.0;
        double cos = Math.Max(verticalCos, 0.5);
        if (verticalSpeedMs >= 0.0)
        {
            if (minTwr > hoverMaxMinTwr)
                return 0.0;
            // Hoverable craft: throttle slightly below weight to resume gentle descent.
            return Math.Max(0.0, massKg * Math.Max(gravityMs2 - 1.0, 0.0) / cos);
        }
        double legs = Math.Max(legsAltM, 0.5);
        double sink = -verticalSpeedMs;
        if (sink <= EmergencyBrakeTargetSinkMs && !(minTwr > hoverMaxMinTwr))
            return massKg * gravityMs2 / cos;
        double decel = sink * sink / (2.0 * legs);
        return massKg * (decel + gravityMs2) / cos;
    }
}

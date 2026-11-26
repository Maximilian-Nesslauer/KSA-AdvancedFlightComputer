namespace AdvancedFlightComputer.Features.Guidance;

/// <summary>
/// Pure math helpers for <see cref="AttitudeController"/>: rate filtering, per-axis conditional anti-windup,
/// joint Fin+RCS backup pricing, and 1-step rate-limited gimbal slew prediction.
/// </summary>
internal static class AttitudeControlMath
{
    internal const double MinSaturationTorqueNm = 1.0e-3;
    internal const double RateCapPerNaturalFrequency = 1.0 / (5.0 * Math.E);

    /// <summary>
    /// Applies an optional first-order backward-Euler lag to the rate error. When <paramref name="rateFilterS"/>
    /// is not positive, returns the raw rate error with zero phase lag.
    /// </summary>
    internal static void FilterRateError(
        double rawX, double rawY, double rawZ,
        double prevX, double prevY, double prevZ,
        bool rateKnown, double rateFilterS, double dt,
        out double outX, out double outY, out double outZ)
    {
        if (!rateKnown || !(rateFilterS > 0.0))
        {
            outX = rawX;
            outY = rawY;
            outZ = rawZ;
            return;
        }
        if (!(dt > 0.0))
        {
            outX = prevX;
            outY = prevY;
            outZ = prevZ;
            return;
        }
        double alpha = dt / (rateFilterS + dt);
        outX = prevX + alpha * (rawX - prevX);
        outY = prevY + alpha * (rawY - prevY);
        outZ = prevZ + alpha * (rawZ - prevZ);
    }

    /// <summary>
    /// Conditional anti-windup on one model axis: freezes integration when the downstream actuator shortfall
    /// exceeds the saturation threshold in the same direction as the attitude error, while allowing unwinding
    /// when the error reverses sign.
    /// </summary>
    internal static double StepIntegralAxis(
        double integral, double thetaRad, double dt,
        double lastDemandNm, double lastShortfallNm, double saturatedShare)
    {
        if (!(dt > 0.0) || thetaRad == 0.0)
            return integral;
        double threshold = Math.Max(saturatedShare * Math.Abs(lastDemandNm), MinSaturationTorqueNm);
        if (Math.Abs(lastShortfallNm) > threshold && lastShortfallNm * thetaRad > 0.0)
            return integral;
        return integral + dt * thetaRad;
    }

    /// <summary>
    /// Updates the 3-axis attitude integral in model coordinates with per-axis conditional anti-windup,
    /// pointing/roll capture windows, and norm clamping.
    /// </summary>
    internal static void UpdateIntegral(
        double intX, double intY, double intZ,
        double thetaPointX, double thetaPointY, double thetaRollZ,
        bool rollFree, double withinRad, double capRadS, double dt,
        double lastDemandX, double lastDemandY, double lastDemandZ,
        double lastShortfallX, double lastShortfallY, double lastShortfallZ,
        double saturatedShare,
        out double outX, out double outY, out double outZ)
    {
        outX = intX;
        outY = intY;
        outZ = intZ;
        if (!(dt > 0.0))
            return;

        double pointNorm = Math.Sqrt(thetaPointX * thetaPointX + thetaPointY * thetaPointY);
        if (pointNorm < withinRad)
        {
            outX = StepIntegralAxis(outX, thetaPointX, dt, lastDemandX, lastShortfallX, saturatedShare);
            outY = StepIntegralAxis(outY, thetaPointY, dt, lastDemandY, lastShortfallY, saturatedShare);
        }
        if (!rollFree && Math.Abs(thetaRollZ) < withinRad)
            outZ = StepIntegralAxis(outZ, thetaRollZ, dt, lastDemandZ, lastShortfallZ, saturatedShare);

        double norm = Math.Sqrt(outX * outX + outY * outY + outZ * outZ);
        if (capRadS >= 0.0 && norm > capRadS && norm > 0.0)
        {
            double s = capRadS / norm;
            outX *= s;
            outY *= s;
            outZ *= s;
        }
    }

    /// <summary>
    /// Computes the propellant-flow cost multiplier for joint Fin+RCS allocation so that even the most efficient
    /// RCS thruster costs more than fin deflection (preserving fin priority) while even the least efficient
    /// thruster costs less than the lowest shortfall priority (so RCS fills residuals instead of taking shortfall).
    /// </summary>
    internal static double BackupPricePerKgS(
        ReadOnlySpan<double> torques, ReadOnlySpan<double> flows, ReadOnlySpan<double> priority,
        double nominalPerNm, double minPerNm, double shortfallCapShare)
    {
        int m = flows.Length;
        if (m == 0 || torques.Length < 3 * m)
            return double.NaN;
        double best = double.PositiveInfinity;
        double worst = 0.0;
        for (int i = 0; i < m; i++)
        {
            double tx = torques[3 * i], ty = torques[3 * i + 1], tz = torques[3 * i + 2];
            double t = Math.Sqrt(tx * tx + ty * ty + tz * tz);
            double flow = flows[i];
            if (!(flow > 0.0) || !double.IsFinite(flow) || !(t > 0.0) || !double.IsFinite(t))
                continue;
            double ratio = flow / t;
            best = Math.Min(best, ratio);
            worst = Math.Max(worst, ratio);
        }
        if (!double.IsFinite(best) || !(worst > 0.0))
            return double.NaN;

        double minPriority = double.PositiveInfinity;
        for (int r = 0; r < 3; r++)
        {
            double w = priority.Length > r && priority[r] > 0.0 && double.IsFinite(priority[r]) ? priority[r] : 1.0;
            minPriority = Math.Min(minPriority, w);
        }
        double nominal = nominalPerNm / best;
        double cap = shortfallCapShare * minPriority / worst;
        double floor = minPerNm / best;
        return Math.Max(floor, Math.Min(nominal, cap));
    }

    /// <summary>
    /// Advances one gimbal axis by <paramref name="deltaTime"/> under KSA's rate-capped critically damped
    /// second-order actuator model (mirroring KSA.GimbalAxis.UpdateState).
    /// </summary>
    internal static void StepGimbalAxis(
        ref float angle, ref float angleRate, float angleCommand,
        float maxAngle, float naturalFrequency, float deltaTime)
    {
        if (!(maxAngle > 0f))
        {
            angle = 0f;
            angleRate = 0f;
            return;
        }
        float rateCap = (float)RateCapPerNaturalFrequency * naturalFrequency * maxAngle;
        float brakeDist = 2f * rateCap / naturalFrequency;
        float rem = deltaTime;
        for (int seg = 0; seg < 4 && rem > 0f; seg++)
        {
            float err = angleCommand - angle;
            if (MathF.Abs(angleRate) >= rateCap && angleRate * err > 0f && MathF.Abs(err) > brakeDist)
            {
                float dtSeg = MathF.Min(rem, (MathF.Abs(err) - brakeDist) / rateCap);
                angleRate = MathF.CopySign(rateCap, angleRate);
                angle += angleRate * dtSeg;
                rem -= dtSeg;
                continue;
            }
            float step = rem;
            float offset = angle - angleCommand;
            EvaluateFilter(offset, angleRate, naturalFrequency, step, out float offsetAt, out float rateAt);
            if (seg + 1 < 4)
            {
                float c1 = angleRate + naturalFrequency * offset;
                float tPeak = c1 != 0f ? (c1 + angleRate) / (naturalFrequency * c1) : 0f;
                if (tPeak > 0f && tPeak < step)
                {
                    EvaluateFilter(offset, angleRate, naturalFrequency, tPeak, out float offPeak, out float ratePeak);
                    if (MathF.Abs(ratePeak) > 1.0001f * rateCap)
                    {
                        step = tPeak;
                        offsetAt = offPeak;
                        rateAt = ratePeak;
                    }
                }
            }
            if (MathF.Abs(rateAt) > 1.0001f * rateCap && seg + 1 < 4)
            {
                float lo = 0f;
                for (int b = 0; b < 12; b++)
                {
                    float mid = 0.5f * (lo + step);
                    EvaluateFilter(offset, angleRate, naturalFrequency, mid, out float offMid, out float rateMid);
                    if (MathF.Abs(rateMid) > rateCap)
                    {
                        step = mid;
                        offsetAt = offMid;
                        rateAt = rateMid;
                    }
                    else
                        lo = mid;
                }
            }
            angle = angleCommand + offsetAt;
            angleRate = Math.Clamp(rateAt, -rateCap, rateCap);
            rem -= step;
        }
        if (rem > 0f)
        {
            EvaluateFilter(angle - angleCommand, angleRate, naturalFrequency, rem, out float offEnd, out float rateEnd);
            angle = angleCommand + offEnd;
            angleRate = Math.Clamp(rateEnd, -rateCap, rateCap);
        }
        if (MathF.Abs(angle - angleCommand) <= 1e-6f && MathF.Abs(angleRate) <= 1e-6f * naturalFrequency)
        {
            angle = angleCommand;
            angleRate = 0f;
        }
        if (angle > maxAngle)
        {
            angle = maxAngle;
            angleRate = MathF.Min(angleRate, 0f);
        }
        else if (angle < -maxAngle)
        {
            angle = -maxAngle;
            angleRate = MathF.Max(angleRate, 0f);
        }
    }

    /// <summary>
    /// Advances a 2-axis gimbal by one step and projects onto the circular deflection cone when enabled,
    /// returning normalized command coordinates in [-1, 1].
    /// </summary>
    internal static void StepGimbalNormalized(
        float angleY, float rateY, float angleZ, float rateZ,
        double cmdY, double cmdZ, float maxY, float maxZ,
        float naturalFrequency, float deltaTime, bool constrainToCircle,
        out double nextCmdY, out double nextCmdZ)
    {
        float targetY = (float)(Math.Clamp(cmdY, -1.0, 1.0) * maxY);
        float targetZ = (float)(Math.Clamp(cmdZ, -1.0, 1.0) * maxZ);
        if (naturalFrequency > 0f && float.IsFinite(naturalFrequency) && deltaTime > 0f)
        {
            StepGimbalAxis(ref angleY, ref rateY, targetY, maxY, naturalFrequency, deltaTime);
            StepGimbalAxis(ref angleZ, ref rateZ, targetZ, maxZ, naturalFrequency, deltaTime);
            if (constrainToCircle && maxY > 0f && maxZ > 0f)
            {
                float ny = angleY / maxY, nz = angleZ / maxZ;
                float n2 = ny * ny + nz * nz;
                if (n2 > 1.000001f)
                {
                    float inv = 1f / MathF.Sqrt(n2);
                    angleY = ny * inv * maxY;
                    angleZ = nz * inv * maxZ;
                }
            }
            nextCmdY = maxY > 0f ? angleY / maxY : 0.0;
            nextCmdZ = maxZ > 0f ? angleZ / maxZ : 0.0;
        }
        else
        {
            nextCmdY = maxY > 0f ? Math.Clamp(cmdY, -1.0, 1.0) : 0.0;
            nextCmdZ = maxZ > 0f ? Math.Clamp(cmdZ, -1.0, 1.0) : 0.0;
        }
    }

    private static void EvaluateFilter(float offset, float rate, float wn, float time, out float offsetAt, out float rateAt)
    {
        float c1 = rate + wn * offset;
        float e = MathF.Exp(-wn * time);
        offsetAt = (offset + c1 * time) * e;
        rateAt = (rate - wn * c1 * time) * e;
    }
}

namespace AdvancedFlightComputer.Features.Guidance.Upfg;

internal static class AscentTerminalCutoff
{
    internal const double EnergyGateSeconds = 3.0;

    // Computes the terminal-phase throttle command for a physics step of length dtStep.
    // Inside the final seconds of AscentPhase.Terminal, tracks live specific orbital energy and
    // predicts one step of lagged-gravity VelocityVerlet drift (KSA.PhysicsStates.IntegrateVelocityVerlet)
    // plus thrust energy gain, scaling the final step proportionally to avoid discrete warp overrun.
    internal static double Evaluate(double remainingSeconds, double dtStep, double minThrottle,
                                    double baseThrottle, bool hasOrbit, double mu, double bodyRadius,
                                    double peKm, double apKm, double rMag, double vMag,
                                    double vDotU, double fullVacuumAccel, out bool cutNow)
    {
        cutNow = false;
        if (!double.IsFinite(remainingSeconds) || remainingSeconds <= 0.0)
        {
            cutNow = true;
            return 0.0;
        }
        if (!double.IsFinite(baseThrottle) || baseThrottle <= 0.0)
            return 0.0;

        double dt = double.IsFinite(dtStep) ? Math.Clamp(dtStep, 1.0 / 120.0, 1.0) : 1.0 / 60.0;
        double minThr = double.IsFinite(minThrottle) ? Math.Clamp(minThrottle, 0.01, 1.0) : 0.01;
        // KSA.Vehicle clamps EngineThrottle to [GetMinThrottle(), 1f], so a lit step delivers at least minThr.
        double effectiveBase = Math.Clamp(baseThrottle, minThr, 1.0);

        if (remainingSeconds <= EnergyGateSeconds && hasOrbit && mu > 0.0 && bodyRadius > 0.0)
        {
            double targetA = bodyRadius + 500.0 * (peKm + apKm);
            double targetE = -0.5 * mu / Math.Max(targetA, bodyRadius);
            if (double.IsFinite(targetE) && targetE < 0.0
                && double.IsFinite(rMag) && rMag > 0.0
                && double.IsFinite(vMag) && vMag > 1.0)
            {
                double curE = 0.5 * vMag * vMag - mu / rMag;
                if (curE >= targetE)
                {
                    cutNow = true;
                    return 0.0;
                }

                double aBurn = double.IsFinite(fullVacuumAccel) && fullVacuumAccel > 0.0
                    ? fullVacuumAccel * effectiveBase
                    : 0.0;
                double vProj = double.IsFinite(vDotU) ? Math.Max(vDotU, 0.1 * vMag) : vMag;
                double aVerlet = 0.5 * dt * (mu / (rMag * rMag * rMag)) * vMag;
                double dEVerlet = dt * aVerlet * vMag;
                double dEThrust = dt * aBurn * vProj;
                double needE = targetE - curE - dEVerlet;
                if (needE <= 0.0)
                {
                    cutNow = true;
                    return 0.0;
                }
                if (dEThrust > 0.0 && needE < dEThrust)
                {
                    double desiredThr = effectiveBase * (needE / dEThrust);
                    if (desiredThr < 0.5 * minThr)
                    {
                        cutNow = true;
                        return 0.0;
                    }
                    return Math.Clamp(desiredThr, minThr, effectiveBase);
                }
            }
        }

        if (remainingSeconds < dt)
        {
            double desiredThr = effectiveBase * (remainingSeconds / dt);
            if (desiredThr < 0.5 * minThr)
            {
                cutNow = true;
                return 0.0;
            }
            return Math.Clamp(desiredThr, minThr, effectiveBase);
        }

        return effectiveBase;
    }
}

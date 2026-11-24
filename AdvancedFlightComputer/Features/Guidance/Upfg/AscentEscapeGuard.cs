namespace AdvancedFlightComputer.Features.Guidance.Upfg;

internal static class AscentEscapeGuard
{
    internal const double LowEnergyCutoffRadiusFactor = 0.05;
    internal const double LowEnergyOverrunFraction = 0.25;

    // Evaluates whether a bound-orbit ascent has crossed into hyperbolic escape or severe over-energy runaway.
    // High-apogee targets whose specific energy is already close to zero only cut at curE >= 0 so legitimate
    // high-energy transfers are not cut early.
    internal static bool ShouldCutForEscape(double bodyRadius, double peKm, double apKm,
                                            double mu, double rMag, double vSq,
                                            out double targetE, out double curE)
    {
        targetE = double.NaN;
        curE = double.NaN;
        if (!(mu > 0.0) || !(bodyRadius > 0.0) || !(rMag > 0.0) || !double.IsFinite(vSq) || vSq < 0.0)
            return false;

        double targetA = bodyRadius + 500.0 * (peKm + apKm);
        targetE = -0.5 * mu / Math.Max(targetA, bodyRadius);
        curE = 0.5 * vSq - mu / rMag;
        if (!double.IsFinite(targetE) || !double.IsFinite(curE) || targetE >= 0.0)
            return false;

        if (curE >= 0.0)
            return true;

        return targetE < -LowEnergyCutoffRadiusFactor * mu / rMag
            && curE > LowEnergyOverrunFraction * targetE;
    }
}

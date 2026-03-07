using System;
using AdvancedFlightComputer.Features.StageInfo;
using KSA;

namespace AdvancedFlightComputer.Features.OberthMultiPass;

record struct OberthAssessmentResult(
    double BurnDuration,
    double OrbitalPeriod,
    double BurnRatio,
    bool ExceedsThreshold,
    int SuggestedPasses,
    double EstimatedSavings);

/// <summary>
/// Evaluates whether a burn should be split into multiple passes to exploit
/// the Oberth effect. Provides a suggested pass count and a heuristic dV
/// savings estimate (cosine loss approximation).
/// </summary>
static class OberthAssessment
{
    public const double DefaultThreshold = 0.15;
    public const double DebugThreshold   = 0.01;
    public const int    MinPasses        = 1;
    public const int    MaxPasses        = 10;

    /// <summary>
    /// Assesses a burn and returns advisory data: burn ratio, whether it exceeds
    /// the threshold, suggested pass count, and estimated dV savings.
    /// </summary>
    public static OberthAssessmentResult Assess(
        Orbit orbit, double burnDvMagnitude, VehicleBurnAnalysis vehicleAnalysis,
        bool debugThreshold = false)
    {
        BurnAnalysis ba = StageAnalyzer.AnalyzeBurn(vehicleAnalysis, (float)burnDvMagnitude);
        double burnDuration = ba.TotalBurnTime;
        double period = orbit.Period;

        if (period <= 0.0)
        {
            return new OberthAssessmentResult(
                burnDuration, period, 0.0, false, MinPasses, 0.0);
        }

        double burnRatio = burnDuration / period;
        double threshold = debugThreshold ? DebugThreshold : DefaultThreshold;
        bool exceeds = burnRatio > threshold;
        int suggested = exceeds ? SuggestPassCount(burnRatio, threshold) : MinPasses;
        double savings = EstimateSavings(orbit, burnDvMagnitude, suggested, burnDuration);

        return new OberthAssessmentResult(burnDuration, period, burnRatio, exceeds, suggested, savings);
    }

    /// <summary>
    /// Heuristic dV savings estimate using cosine loss approximation.
    /// singlePassLoss = totalDv * arcFraction^2 * 0.5
    /// multiPassLoss  = singlePassLoss / passCount^2
    /// </summary>
    public static double EstimateSavings(
        Orbit orbit, double totalDv, int passCount, double burnDuration)
    {
        if (orbit.Period <= 0.0 || passCount <= 1)
            return 0.0;

        double arcFraction = burnDuration / orbit.Period;
        double singlePassLoss = totalDv * arcFraction * arcFraction * 0.5;
        double multiPassLoss  = singlePassLoss / ((double)passCount * passCount);
        return Math.Max(0.0, singlePassLoss - multiPassLoss);
    }

    /// <summary>
    /// Suggests a pass count based on how much the burn ratio exceeds the threshold.
    /// ceil(burnRatio / threshold), clamped to [2, MaxPasses].
    /// </summary>
    public static int SuggestPassCount(double burnRatio, double threshold = DefaultThreshold)
    {
        int passes = (int)Math.Ceiling(burnRatio / threshold);
        return Math.Clamp(passes, 2, MaxPasses);
    }
}

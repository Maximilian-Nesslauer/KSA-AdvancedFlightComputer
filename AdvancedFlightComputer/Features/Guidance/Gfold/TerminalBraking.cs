namespace AdvancedFlightComputer.Guidance.Gfold;

// Vertical coast and braking control for an engine whose minimum thrust exceeds the weight.
// Inputs use metres, seconds and upward velocity. Accelerations include thrust only.
public static class TerminalBraking
{
    public const double TouchdownSpeed = 0.5;
    // The landing legs take this sink at contact.
    public const double ContactSinkLimit = 2;

    public readonly record struct Command(double Acceleration, double IgnitionHeight, double BurnSeconds)
    {
        public bool EngineOn => Acceleration > 0;
    }

    public static Command Evaluate(double height, double velocityUp, double gravity,
        double minimumAcceleration, double maximumAcceleration, double stepSeconds,
        double minimumPulseSeconds, bool burning)
    {
        if (!double.IsFinite(height + velocityUp + gravity + minimumAcceleration + maximumAcceleration
            + stepSeconds + minimumPulseSeconds) || gravity <= 0 || minimumAcceleration < 0
            || maximumAcceleration <= gravity || maximumAcceleration < minimumAcceleration
            || stepSeconds <= 0 || minimumPulseSeconds < 0)
            throw new ArgumentOutOfRangeException(nameof(maximumAcceleration), "Terminal braking needs finite state and upward thrust authority.");

        double sink = Math.Max(0, -velocityUp);
        // Leave throttle authority on both sides of the planned burn, including as mass falls.
        double nominal = Math.Max(gravity * 1.5, minimumAcceleration + 0.35 * (maximumAcceleration - minimumAcceleration));
        nominal = Math.Min(nominal, maximumAcceleration);
        double net = nominal - gravity;
        double lead = stepSeconds + 0.15;
        double futureSink = sink + gravity * lead;
        double ignitionHeight = sink * lead + 0.5 * gravity * lead * lead
            + Math.Max(0, futureSink * futureSink - TouchdownSpeed * TouchdownSpeed) / (2 * net);
        double burnSeconds = Math.Max(0, sink - TouchdownSpeed) / net;

        if (velocityUp >= -TouchdownSpeed || (!burning && height > ignitionHeight))
            return new(0, ignitionHeight, burnSeconds);

        double pulse = Math.Max(stepSeconds, minimumPulseSeconds);
        // One step or minimum pulse must not reverse the descent, so it may remove at most the sink above the touchdown speed.
        double noReversal = gravity + (sink - TouchdownSpeed) / pulse;
        bool pulseReverses = minimumAcceleration > noReversal;

        // Below the estimated touchdown plane the real ground distance is unknown, so minimum thrust holds the sink between the touchdown speed and the contact limit until contact.
        if (height <= 0)
            return new(!pulseReverses && (burning || sink > ContactSinkLimit) ? minimumAcceleration : 0, ignitionHeight, burnSeconds);

        double required = gravity + (sink * sink - TouchdownSpeed * TouchdownSpeed) / (2 * Math.Max(height, 0.05));
        // Coast again if the burn ended above the ground.
        if (required < minimumAcceleration || pulseReverses)
            return new(0, ignitionHeight, burnSeconds);

        return new(Math.Clamp(required, minimumAcceleration, Math.Min(maximumAcceleration, noReversal)), ignitionHeight, burnSeconds);
    }
}

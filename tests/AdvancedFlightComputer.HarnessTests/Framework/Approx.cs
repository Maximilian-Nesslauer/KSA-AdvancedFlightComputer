namespace AdvancedFlightComputer.HarnessTests.Framework;

// Tolerance bounds, all inclusive.
public static class Approx
{
    // floor puts a lower bound under |expected|, so an expectation of zero does not silently demand
    // bit-exactness.
    public static bool Rel(double actual, double expected, double relTol, double floor = 0.0)
        => Math.Abs(actual - expected) <= relTol * Math.Max(Math.Abs(expected), floor);

    public static bool Abs(double actual, double expected, double absTol)
        => Math.Abs(actual - expected) <= absTol;

    public static bool Mixed(double actual, double expected, double absTol, double relTol)
        => Math.Abs(actual - expected) <= absTol + relTol * Math.Abs(expected);
}

using AdvancedFlightComputer.HarnessTests.Framework;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Shared orbit-level assertion for the RCS flight tests: the achieved
// orbit must land near the impulsive prediction, with tolerances relative
// to the CHANGE the burn was supposed to make (a burn that moved the
// semi-major axis 90% of the way but in the wrong direction should fail
// even though every absolute number is within a fraction of a percent of
// the original orbit). The remaining slack absorbs finite-burn arc loss
// and the attitude hold's cross-thrust.
internal static class RcsOrbitCheck
{
    private const double SignalTolerance = 0.2;
    private const double SlackM = 1.0;

    public static bool Assert(
        TestContext t, string label, Orbit achieved, Orbit predicted,
        double initialSma, double initialEcc)
    {
        // Both element errors are measured against ONE scale: the burn's
        // total orbital effect in meters, max(|dSMA|, a*|decc|). A per-
        // element signal would fail on noise for a near-radial burn, whose
        // SMA change is mm-scale second order while the real effect lives
        // in the eccentricity (and vice versa for a tangential burn).
        double smaSignal = Math.Abs(predicted.SemiMajorAxis - initialSma);
        double eccSignalM = initialSma * Math.Abs(predicted.Eccentricity - initialEcc);
        double scaleM = Math.Max(smaSignal, eccSignalM);
        double toleranceM = SignalTolerance * scaleM + SlackM;

        double smaErrorM = Math.Abs(achieved.SemiMajorAxis - predicted.SemiMajorAxis);
        double eccErrorM = initialSma * Math.Abs(achieved.Eccentricity - predicted.Eccentricity);

        return t.Check(label, smaErrorM <= toleranceM && eccErrorM <= toleranceM,
            $"SMA {initialSma:F0} -> {achieved.SemiMajorAxis:F0}m " +
            $"(predicted {predicted.SemiMajorAxis:F0}m, error {smaErrorM:F1}m), " +
            $"ecc {initialEcc:F6} -> {achieved.Eccentricity:F6} " +
            $"(predicted {predicted.Eccentricity:F6}, error {eccErrorM:F1}m), " +
            $"effect scale {scaleM:F1}m, tol {toleranceM:F1}m");
    }
}

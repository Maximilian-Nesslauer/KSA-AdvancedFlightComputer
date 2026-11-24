using AdvancedFlightComputer.Features.Guidance.Upfg;

internal static partial class Program
{
    private static void AscentTerminalCutoffRules()
    {
        // Unit edge cases: timer expiry, energy already reached, non-throttleable g-limit clamp, degenerate orbit fallback.
        double thr = AscentTerminalCutoff.Evaluate(0.0, 1.0 / 60.0, 0.1, 1.0, false, 0, 0, 0, 0, 0, 0, 0, 0, out bool cut);
        Check("rem <= 0 cuts immediately", cut && thr == 0.0);

        thr = AscentTerminalCutoff.Evaluate(5.0, 1.0 / 60.0, 0.1, 0.8, false, 0, 0, 0, 0, 0, 0, 0, 0, out cut);
        Check("outside energy gate and timer step holds base throttle", !cut && Math.Abs(thr - 0.8) < 1e-12);

        // Non-throttleable engine (minThrottle = 1.0) with baseThrottle < 1.0 must clamp safely without throwing.
        thr = AscentTerminalCutoff.Evaluate(0.08, 0.10, 1.0, 0.4, false, 0, 0, 0, 0, 0, 0, 0, 0, out cut);
        Check("non-throttleable engine above half step burns at full throttle", !cut && Math.Abs(thr - 1.0) < 1e-12);

        thr = AscentTerminalCutoff.Evaluate(0.04, 0.10, 1.0, 0.4, false, 0, 0, 0, 0, 0, 0, 0, 0, out cut);
        Check("non-throttleable engine below half step cuts early", cut && thr == 0.0);

        // Degenerate orbit falls back to proportional timer cutoff on the last physics step.
        thr = AscentTerminalCutoff.Evaluate(0.05, 0.10, 0.1, 1.0, false, 0, 0, 0, 0, 0, 0, 0, 0, out cut);
        Check("degenerate orbit uses fractional timer throttle", !cut && Math.Abs(thr - 0.5) < 1e-12);

        // Closed-loop 1x / 4x / 10x warp terminal cutoff comparisons against binary timer cutoff,
        // including the lagged-gravity VelocityVerlet energy drift model.
        VerifyWarpCutoff("Earth 200x200 km", bodyR: 6_371_000.0, mu: 3.986004418e14,
            peKm: 200.0, apKm: 200.0, fullAccel: 30.0, minThr: 0.05);
        VerifyWarpCutoff("Luna 30x50 km", bodyR: 1_737_400.0, mu: 4.9048695e12,
            peKm: 30.0, apKm: 50.0, fullAccel: 15.0, minThr: 0.05);

        // High minimum throttle (0.40) at 10x warp must stay bounded by half of one min-throttle step.
        double hiMinErr = SimulateTerminalBurn(
            bodyR: 6_371_000.0, mu: 3.986004418e14, peKm: 200.0, apKm: 200.0,
            fullAccel: 30.0, minThr: 0.40, dt: 10.0 / 60.0, proportional: true);
        double hiMinBinary = SimulateTerminalBurn(
            bodyR: 6_371_000.0, mu: 3.986004418e14, peKm: 200.0, apKm: 200.0,
            fullAccel: 30.0, minThr: 0.40, dt: 10.0 / 60.0, proportional: false);
        Check("Earth 200x200 km 10x warp with minThrottle 0.40 beats binary cutoff",
            Math.Abs(hiMinErr) < Math.Abs(hiMinBinary) * 0.5,
            $"proportional {hiMinErr:F3} km vs binary {hiMinBinary:F3} km");
    }

    private static void VerifyWarpCutoff(string label, double bodyR, double mu,
                                         double peKm, double apKm, double fullAccel, double minThr)
    {
        foreach ((int warp, double maxSmaErrKm) in new[] { (1, 0.05), (4, 0.05), (10, 0.15) })
        {
            double dt = warp / 60.0;
            double binErr = SimulateTerminalBurn(bodyR, mu, peKm, apKm, fullAccel, minThr, dt, proportional: false);
            double propErr = SimulateTerminalBurn(bodyR, mu, peKm, apKm, fullAccel, minThr, dt, proportional: true);
            Check($"{label} at {warp}x warp reduces SMA overrun below {maxSmaErrKm:F2} km",
                Math.Abs(propErr) < maxSmaErrKm && Math.Abs(propErr) < Math.Abs(binErr) * 0.1,
                $"proportional {propErr:+0.0000;-0.0000} km vs binary {binErr:+0.0000;-0.0000} km");
        }
    }

    // Simulates the final 2.5 s of a prograde terminal burn with discrete step dt and lagged-gravity
    // VelocityVerlet drift, returning the final semi-major-axis error in km.
    private static double SimulateTerminalBurn(double bodyR, double mu, double peKm, double apKm,
                                               double fullAccel, double minThr, double dt, bool proportional)
    {
        double targetA = bodyR + 500.0 * (peKm + apKm);
        double targetE = -0.5 * mu / targetA;
        double r = bodyR + peKm * 1000.0;
        double vTarget = Math.Sqrt(2.0 * (targetE + mu / r));

        // Choose a start speed so the exact cutoff lands at 0.15 * dt past a step boundary,
        // forcing nearly a full step of quantization overrun under binary timer cutoff.
        const double burnTime = 2.5;
        double rem = burnTime + 0.15 * dt;
        double v = vTarget - fullAccel * rem;
        double t = 0.0;
        double cutoffTime = rem;

        for (int step = 0; step < 500; step++)
        {
            double throttle;
            bool cutNow;
            if (proportional)
            {
                throttle = AscentTerminalCutoff.Evaluate(
                    cutoffTime - t, dt, minThr, 1.0, true, mu, bodyR, peKm, apKm,
                    r, v, v, fullAccel, out cutNow);
            }
            else
            {
                cutNow = t >= cutoffTime;
                throttle = cutNow ? 0.0 : 1.0;
            }

            if (cutNow)
                break;

            double aVerlet = 0.5 * dt * (mu / (r * r * r)) * v;
            v += (fullAccel * throttle + aVerlet) * dt;
            t += dt;
        }

        double finalE = 0.5 * v * v - mu / r;
        double finalA = -0.5 * mu / finalE;
        return (finalA - targetA) / 1000.0;
    }
}

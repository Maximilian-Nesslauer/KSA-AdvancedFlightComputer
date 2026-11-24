using AdvancedFlightComputer.Features.Guidance.Upfg;

internal static partial class Program
{
    private static void AscentEscapeGuardRules()
    {
        const double earthR = 6_371_000.0;
        const double earthMu = 3.986004418e14;
        double rLeo = earthR + 200_000.0;
        double vCircLeo = Math.Sqrt(earthMu / rLeo);
        double vEscLeo = Math.Sqrt(2.0 * earthMu / rLeo);

        Check("nominal Earth LEO ascent does not trigger escape guard",
            !AscentEscapeGuard.ShouldCutForEscape(earthR, 200.0, 200.0, earthMu, rLeo, vCircLeo * vCircLeo, out _, out _));

        // Mild overshoot below the 0.25 * targetE runaway bound stays active for normal cutoff.
        double vMild = vCircLeo * 1.05;
        Check("mild LEO velocity overshoot does not trigger early escape guard",
            !AscentEscapeGuard.ShouldCutForEscape(earthR, 200.0, 200.0, earthMu, rLeo, vMild * vMild, out _, out _));

        // Severe over-energy runaway (curE > 0.25 * targetE) on a LEO target triggers safe cutoff.
        double vRunaway = Math.Sqrt(2.0 * (0.20 * (-0.5 * earthMu / rLeo) + earthMu / rLeo));
        Check("severe LEO over-energy runaway triggers escape guard",
            AscentEscapeGuard.ShouldCutForEscape(earthR, 200.0, 200.0, earthMu, rLeo, vRunaway * vRunaway, out _, out _));

        // True hyperbolic escape triggers for any bound target.
        Check("hyperbolic escape velocity triggers escape guard on LEO target",
            AscentEscapeGuard.ShouldCutForEscape(earthR, 200.0, 200.0, earthMu, rLeo, (vEscLeo + 5.0) * (vEscLeo + 5.0), out _, out _));

        // High-apogee transfer (600 x 1,000,000 km from 500 km LEO) sits within 0.05 * mu/r of escape:
        // approaching target energy below 0 must NOT trigger early, while crossing curE >= 0 MUST cut.
        double r500 = earthR + 500_000.0;
        double vNearEsc = Math.Sqrt(2.0 * earthMu / r500) - 25.0;
        double vPastEsc = Math.Sqrt(2.0 * earthMu / r500) + 1.0;
        Check("legitimate high-apogee transfer below escape does not trigger early cutoff",
            !AscentEscapeGuard.ShouldCutForEscape(earthR, 600.0, 1_000_000.0, earthMu, r500, vNearEsc * vNearEsc, out _, out _));
        Check("high-apogee transfer crossing curE >= 0 triggers escape cutoff",
            AscentEscapeGuard.ShouldCutForEscape(earthR, 600.0, 1_000_000.0, earthMu, r500, vPastEsc * vPastEsc, out _, out _));

        // Lunar 150 x 150 km ascent: nominal stays active, positive-energy runaway cuts.
        const double lunaR = 1_737_400.0;
        const double lunaMu = 4.9048695e12;
        double rLuna = lunaR + 150_000.0;
        double vLunaCirc = Math.Sqrt(lunaMu / rLuna);
        double vLunaRunaway = Math.Sqrt(2.0 * (266_418.0 + lunaMu / rLuna));
        Check("nominal lunar 150 km ascent does not trigger escape guard",
            !AscentEscapeGuard.ShouldCutForEscape(lunaR, 150.0, 150.0, lunaMu, rLuna, vLunaCirc * vLunaCirc, out _, out _));
        Check("lunar positive-energy runaway triggers escape guard",
            AscentEscapeGuard.ShouldCutForEscape(lunaR, 150.0, 150.0, lunaMu, rLuna, vLunaRunaway * vLunaRunaway, out _, out _));

        // Invalid inputs fail safe (no trigger, no exception).
        Check("invalid radius or mu returns false safely",
            !AscentEscapeGuard.ShouldCutForEscape(earthR, 200.0, 200.0, 0.0, rLeo, vEscLeo * vEscLeo, out _, out _)
            && !AscentEscapeGuard.ShouldCutForEscape(earthR, 200.0, 200.0, earthMu, 0.0, vEscLeo * vEscLeo, out _, out _)
            && !AscentEscapeGuard.ShouldCutForEscape(earthR, 200.0, 200.0, earthMu, rLeo, double.NaN, out _, out _));
    }
}

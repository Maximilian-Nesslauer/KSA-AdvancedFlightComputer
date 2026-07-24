using AdvancedFlightComputer.Features.Flyby;
using Brutal.Numerics;
using HeadlessHarness.Core;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates the flyby B-plane closed forms and the reference-radius resolution.
// The oracle for the impact-parameter relation is the game's own hyperbolic orbit
// elements: for a hyperbola built at periapsis rp with excess speed v_inf about a
// body of mu, the game's SemiMajorAxis / Eccentricity must give the same impact
// parameter b = |a| * sqrt(e^2 - 1) that FlybyTargeting.ImpactParameterForPeriapsis
// computes, and the inverse must round-trip back to rp.
public sealed class FlybyTargetingTest : IHarnessTest
{
    private const double RelTol = 1e-6;

    // (v_inf m/s, periapsis altitude above the body surface m).
    private static readonly (double VInf, double Alt)[] Cases =
    {
        (500.0, 100_000.0),
        (850.0, 200_000.0),
        (1500.0, 50_000.0),
        (3000.0, 500_000.0),
    };

    public string Name => "afc-flyby-targeting";

    public int Run(HeadlessSession session)
    {
        if (!ManeuverTestSupport.RequireHome(Name, session, out IParentBody home))
            return 1;

        bool ok = true;
        ok &= CheckImpactParameter(home);
        ok &= CheckReference(home);

        IParentBody? moon = FindMoon(home);
        if (moon != null)
            ok &= CheckAirlessMoon(moon);
        else
            HarnessLog.Line($"[{Name}] no moon found under home body; airless-reference subcase skipped.");

        HarnessLog.Line($"[{Name}] {TestSupport.Verdict(ok)}");
        return ok ? 0 : 1;
    }

    private bool CheckImpactParameter(IParentBody body)
    {
        SimTime now = Universe.GetElapsedSimTime();
        double mu = body.Mu;
        bool ok = true;

        foreach ((double vInf, double alt) in Cases)
        {
            double rp = body.MeanRadius + alt;
            double b = FlybyTargeting.ImpactParameterForPeriapsis(vInf, rp, mu);

            // Build the incoming hyperbola at its periapsis and let the game
            // compute the elements; b_game = |a| * sqrt(e^2 - 1).
            double vP = Math.Sqrt(vInf * vInf + 2.0 * mu / rp);
            Orbit hyp = Orbit.CreateFromStateCci(
                body, now, new double3(rp, 0.0, 0.0), new double3(0.0, vP, 0.0),
                VehicleSpawner.OrbitLineColor);
            double a = hyp.SemiMajorAxis;
            double e = hyp.Eccentricity;
            double bGame = Math.Abs(a) * Math.Sqrt(Math.Max(0.0, e * e - 1.0));

            double rpBack = FlybyTargeting.PeriapsisForImpactParameter(vInf, b, mu);

            bool hyperbolic = e > 1.0;
            bool peOk = ManeuverTestSupport.NearRel(hyp.Periapsis, rp, RelTol);
            bool bOk = ManeuverTestSupport.NearRel(b, bGame, 1e-4);
            bool inverseOk = ManeuverTestSupport.NearRel(rpBack, rp, 1e-4);
            bool caseOk = hyperbolic && peOk && bOk && inverseOk;
            ok &= caseOk;

            HarnessLog.Line($"[{Name}] b-param vInf={vInf:F0} rp={rp:E6}: " +
                $"b={b:E6} bGame={bGame:E6} e={e:F4} rpBack={rpBack:E6} => {TestSupport.Verdict(caseOk)}");
        }
        return ok;
    }

    private bool CheckReference(IParentBody body)
    {
        const double alt = 100_000.0;
        bool ok = true;

        double surface = FlybyTargeting.ResolvePeriapsisRadius(body, alt, FlybyReference.Surface);
        ok &= Expect("Surface", surface, body.MeanRadius + alt);

        double center = FlybyTargeting.ResolvePeriapsisRadius(body, body.MeanRadius + alt, FlybyReference.Center);
        ok &= Expect("Center", center, body.MeanRadius + alt);

        double minR = FlybyTargeting.MinFlybyRadius(body);
        ok &= Expect("MinFlybyRadius", minR, body.GetNearSurfaceRadius());

        if (FlybyTargeting.HasAtmosphere(body))
        {
            double atmo = FlybyTargeting.ResolvePeriapsisRadius(body, alt, FlybyReference.Atmosphere);
            ok &= Expect("Atmosphere", atmo, body.GetAtmosphereRadius() + alt);
            bool posAtmo = body.GetAtmosphereRadius() > 0.0;
            ok &= posAtmo;
            HarnessLog.Line($"[{Name}] atmosphere radius {body.GetAtmosphereRadius():E6} => {TestSupport.Verdict(posAtmo)}");
        }
        else
        {
            HarnessLog.Line($"[{Name}] home body has no atmosphere; Atmosphere reference subcase skipped.");
        }
        return ok;
    }

    private bool CheckAirlessMoon(IParentBody moon)
    {
        // A moon without an atmosphere must report GetAtmosphereRadius() == 0 and
        // HasAtmosphere() == false, so the UI drops the Atmosphere option.
        double atmoR = moon.GetAtmosphereRadius();
        if (atmoR > 0.0)
        {
            HarnessLog.Line($"[{Name}] moon has an atmosphere ({atmoR:E6}); airless subcase not applicable.");
            return true;
        }
        bool ok = !FlybyTargeting.HasAtmosphere(moon);
        HarnessLog.Line($"[{Name}] airless moon: HasAtmosphere={FlybyTargeting.HasAtmosphere(moon)} => {TestSupport.Verdict(ok)}");
        return ok;
    }

    private bool Expect(string label, double actual, double expected)
    {
        bool ok = ManeuverTestSupport.NearRel(actual, expected, RelTol) || Math.Abs(actual - expected) < 1.0;
        HarnessLog.Line($"[{Name}] ref {label}: {actual:E6} (expected {expected:E6}) => {TestSupport.Verdict(ok)}");
        return ok;
    }

    private static IParentBody? FindMoon(IParentBody home)
    {
        foreach (IOrbiter child in home.Children)
            if (child is Celestial && child is IParentBody moon
                && moon.SphereOfInfluence > 0.0 && !double.IsNaN(moon.SphereOfInfluence))
                return moon;
        return null;
    }
}

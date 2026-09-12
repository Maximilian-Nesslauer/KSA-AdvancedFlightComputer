using AdvancedFlightComputer.Features.Flyby;
using AdvancedFlightComputer.HarnessTests.Fixtures;
using AdvancedFlightComputer.HarnessTests.Framework;
using Brutal.Numerics;
using HeadlessHarness.Harness;
using KSA;

namespace AdvancedFlightComputer.HarnessTests;

// Validates the flyby B plane closed forms and the reference radius resolution.
// The oracle for the impact parameter relation is the game's own hyperbolic orbit.
// For a hyperbola built at periapsis rp with excess speed v_inf about a body of mu,
// the game's SemiMajorAxis and Eccentricity must give the same impact parameter
// b = |a| * sqrt(e^2 - 1) that FlybyTargeting.ImpactParameterForPeriapsis computes,
// and the inverse must come back to rp. The form with the SOI correction the
// retarget flies with is checked against the game's propagated speed at the SOI
// radius.
public sealed class FlybyTargetingTest : AfcTest
{
    private const double RelTol = 1e-6;

    // Excess speed in m/s and periapsis altitude above the body surface in m.
    private static readonly (double VInf, double Alt)[] Cases =
    {
        (500.0, 100_000.0),
        (850.0, 200_000.0),
        (1500.0, 50_000.0),
        (3000.0, 500_000.0),
    };

    public override string Name => "afc-flyby-targeting";

    protected override void Execute(TestContext t)
    {
        if (!TestWorld.RequireHome(t, out IParentBody home))
            return;

        CheckImpactParameter(t, home);
        CheckSoiCorrectedForm(t, home);
        CheckReference(t, home);

        if (TestWorld.FindMoon(home) is IParentBody moon)
            CheckAirlessMoon(t, moon);
        else
            t.Skip("no moon found under home body; airless-reference subcase not applicable.");
    }

    private static void CheckImpactParameter(TestContext t, IParentBody body)
    {
        UniverseTime now = Universe.GetElapsedTime();
        double mu = body.Mu;

        foreach ((double vInf, double alt) in Cases)
        {
            double rp = body.MeanRadius + alt;
            double b = FlybyTargeting.ImpactParameterForPeriapsis(vInf, rp, mu);

            // Build the incoming hyperbola at its periapsis and let the game compute
            // the elements, which give b_game = |a| * sqrt(e^2 - 1).
            double vP = Math.Sqrt(vInf * vInf + 2.0 * mu / rp);
            Orbit hyp = Orbit.CreateFromStateCci(
                body, now, new double3(rp, 0.0, 0.0), new double3(0.0, vP, 0.0),
                VehicleSpawner.OrbitLineColor);
            double a = hyp.SemiMajorAxis;
            double e = hyp.Eccentricity;
            double bGame = Math.Abs(a) * Math.Sqrt(Math.Max(0.0, e * e - 1.0));

            double rpBack = FlybyTargeting.PeriapsisForImpactParameter(vInf, b, mu);

            bool hyperbolic = e > 1.0;
            bool peOk = Approx.Rel(hyp.Periapsis, rp, RelTol);
            bool bOk = Approx.Rel(b, bGame, 1e-4);
            bool inverseOk = Approx.Rel(rpBack, rp, 1e-4);
            t.Check($"b-param vInf={vInf:F0}", hyperbolic && peOk && bOk && inverseOk,
                $"rp={rp:E6} b={b:E6} bGame={bGame:E6} e={e:F4} rpBack={rpBack:E6}");
        }
    }

    // The retarget measures the relative speed at the SOI boundary, not at
    // infinity. For an orbit built at periapsis with the speed the corrected form
    // implies, the game's own propagated speed at the SOI radius must come back as
    // the v_soi that went in, and the inverse must come back to rp. Below the
    // escape speed at the SOI boundary, about 930 m/s for Earth, that orbit is an
    // ellipse whose apoapsis still lies beyond the SOI, which is what makes the
    // slower cases cover the branch of the inverse below escape.
    private static void CheckSoiCorrectedForm(TestContext t, IParentBody body)
    {
        double soi = body.SphereOfInfluence;
        if (!(soi > 0.0) || !double.IsFinite(soi))
        {
            t.Skip($"home body SOI is {soi}; SOI-corrected subcase not applicable.");
            return;
        }

        UniverseTime now = Universe.GetElapsedTime();
        double mu = body.Mu;

        foreach ((double vSoi, double alt) in Cases)
        {
            double rp = body.MeanRadius + alt;
            double b = FlybyTargeting.ImpactParameterForPeriapsis(vSoi, rp, mu, soi);
            if (!(b > 0.0))
            {
                t.Fail($"soi-form vSoi={vSoi:F0}", $"closed form rejected a valid input (b={b})");
                continue;
            }

            // b = rp * vP / vSoi, so the periapsis speed the form implies is bv / rp.
            double vP = b * vSoi / rp;
            Orbit hyp = Orbit.CreateFromStateCci(
                body, now, new double3(rp, 0.0, 0.0), new double3(0.0, vP, 0.0),
                VehicleSpawner.OrbitLineColor);
            double3? vAtSoi = hyp.GetVelocityCciAtRadius(new Radius(soi));
            double rpBack = FlybyTargeting.PeriapsisForImpactParameter(vSoi, b, mu, soi);

            bool reachesSoi = vAtSoi.HasValue;
            bool speedOk = reachesSoi && Approx.Rel(vAtSoi!.Value.Length(), vSoi, 1e-4);
            bool peOk = Approx.Rel(hyp.Periapsis, rp, RelTol);
            bool inverseOk = Approx.Rel(rpBack, rp, 1e-4);
            t.Check($"soi-form vSoi={vSoi:F0}", reachesSoi && speedOk && peOk && inverseOk,
                $"rp={rp:E6} b={b:E6} vAtSoi={(reachesSoi ? vAtSoi!.Value.Length() : double.NaN):F3} " +
                $"e={hyp.Eccentricity:F4} rpBack={rpBack:E6}");
        }
    }

    private static void CheckReference(TestContext t, IParentBody body)
    {
        const double alt = 100_000.0;

        CheckRadius(t, "Surface",
            FlybyTargeting.ResolvePeriapsisRadius(body, alt, FlybyReference.Surface),
            body.MeanRadius + alt);
        CheckRadius(t, "Center",
            FlybyTargeting.ResolvePeriapsisRadius(body, body.MeanRadius + alt, FlybyReference.Center),
            body.MeanRadius + alt);
        CheckRadius(t, "MinFlybyRadius",
            FlybyTargeting.MinFlybyRadius(body), body.GetNearSurfaceRadius());

        if (!FlybyTargeting.HasAtmosphere(body))
        {
            t.Skip("home body has no atmosphere; Atmosphere reference subcase not applicable.");
            return;
        }
        CheckRadius(t, "Atmosphere",
            FlybyTargeting.ResolvePeriapsisRadius(body, alt, FlybyReference.Atmosphere),
            body.GetAtmosphereRadius() + alt);
        t.Check("atmosphere radius is positive", body.GetAtmosphereRadius() > 0.0,
            $"{body.GetAtmosphereRadius():E6}");
    }

    // A moon without an atmosphere must report GetAtmosphereRadius() == 0 and
    // HasAtmosphere() == false, so the UI drops the Atmosphere option.
    private static void CheckAirlessMoon(TestContext t, IParentBody moon)
    {
        double atmoR = moon.GetAtmosphereRadius();
        if (atmoR > 0.0)
        {
            t.Skip($"moon has an atmosphere ({atmoR:E6}); airless subcase not applicable.");
            return;
        }
        t.Check("airless moon offers no atmosphere reference", !FlybyTargeting.HasAtmosphere(moon));
    }

    // A metre of slack under the relative bound, because these radii are sums of
    // two large doubles.
    private static void CheckRadius(TestContext t, string label, double actual, double expected)
        => t.Check($"ref {label}",
            Approx.Rel(actual, expected, RelTol) || Approx.Abs(actual, expected, 1.0),
            $"{actual:E6} (expected {expected:E6})");
}

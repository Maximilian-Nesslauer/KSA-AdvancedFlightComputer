using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>
/// WOULD THE TOUCHDOWN TIP THE CRAFT OVER? At contact the craft pivots about the edge of what it stands on - a skirt's rim, the feet - that it is moving or leaning towards: R out from the axis, with the centre of mass H above the contact plane when upright. Its angular momentum about that edge is the sideways momentum's moment plus its own spin, <c>L = m v H + I w</c>; after the contact it turns about the edge with <c>I_e = I + m (R^2 + H^2)</c>, carrying <c>L^2 / (2 I_e)</c>. To go over, the centre of mass has to climb from where the tilt puts it to straight above the edge: <c>m g d (1 - cos(alpha - theta))</c>, with <c>d = sqrt(R^2 + H^2)</c>, <c>alpha = atan(R / H)</c> the tilt at which it balances on the edge, and theta the tilt towards it. Past alpha there is nothing left to climb.
///  WORST CASE: the sideways speed, the turn and the tilt all taken towards the same edge, and the descent's own moment about the edge (which turns the craft back) left out.
///  The user's line (2026-11-27): a touchdown carrying 80 % of what tips it or less is fine (<see cref="AcceptShare"/>); more is not.
///  Pure: no game types, so UiLogic tests it.
/// </summary>
public static class TipOver
{
    /// <summary>The share of the energy that tips the craft over a touchdown may carry.</summary>
    public const double AcceptShare = 0.8;

    /// <summary>The tilt at which the centre of mass stands straight over the edge, rad: atan(R / H).</summary>
    public static double BalanceTiltRad(double radiusM, double heightM)
        => Math.Atan2(Math.Max(radiusM, 0.0), Math.Max(heightM, 1e-6));

    /// <summary>The energy that tips the craft over from <paramref name="tiltRad"/> towards the edge, J; 0 at or past the balance tilt.</summary>
    public static double BarrierJ(double massKg, double radiusM, double heightM, double gravity, double tiltRad)
    {
        double d = Math.Sqrt(radiusM * radiusM + heightM * heightM);
        double left = BalanceTiltRad(radiusM, heightM) - Math.Abs(tiltRad);
        return left > 0.0 ? massKg * gravity * d * (1.0 - Math.Cos(left)) : 0.0;
    }

    /// <summary>The energy the touchdown carries about the edge, J: sideways speed <paramref name="sideMs"/> and pitch-yaw rate <paramref name="rateRadS"/>, both taken towards the edge.</summary>
    public static double ImpactJ(double massKg, double inertiaKgM2, double radiusM, double heightM, double sideMs, double rateRadS)
    {
        double l = massKg * Math.Abs(sideMs) * heightM + inertiaKgM2 * Math.Abs(rateRadS);
        double ie = inertiaKgM2 + massKg * (radiusM * radiusM + heightM * heightM);
        return ie > 0.0 ? l * l / (2.0 * ie) : 0.0;
    }

    /// <summary>The touchdown's energy as a share of what tips the craft: 1 tips it, <see cref="AcceptShare"/> is the line; infinity at or past the balance tilt.</summary>
    public static double Share(double massKg, double inertiaKgM2, double radiusM, double heightM, double gravity,
                               double sideMs, double rateRadS, double tiltRad)
    {
        double barrier = BarrierJ(massKg, radiusM, heightM, gravity, tiltRad);
        double impact = ImpactJ(massKg, inertiaKgM2, radiusM, heightM, sideMs, rateRadS);
        return barrier > 0.0 ? impact / barrier : double.PositiveInfinity;
    }

    /// <summary>
    /// The plan's linear form of the line (Scvx6DofConfig.TipLimitEnabled): for tilts well short of the balance tilt the barrier is m g d (alpha - theta)^2 / 2, so the share stays at or under <paramref name="share"/> while
    /// <c>(m H v + I w) / K + theta &lt;= alpha</c>, with <c>K = sqrt(share I_e m g d)</c>. Returns K, N m s per rad; NaN for a craft with no width to stand on.
    /// </summary>
    public static double LimitGain(double massKg, double inertiaKgM2, double radiusM, double heightM, double gravity, double share = AcceptShare)
    {
        double d = Math.Sqrt(radiusM * radiusM + heightM * heightM);
        double ie = inertiaKgM2 + massKg * d * d;
        double k = Math.Sqrt(Math.Max(share, 0.0) * ie * massKg * gravity * d);
        return radiusM > 0.0 && double.IsFinite(k) && k > 0.0 ? k : double.NaN;
    }
}

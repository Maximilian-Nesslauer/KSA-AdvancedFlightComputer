using System;

namespace AdvancedFlightComputer.Features.Ui.Logic;

/// <summary>
/// THE CONTACT HEIGHT AS A FUNCTION OF ATTITUDE (#339). How far below the centre of mass the craft's lowest point sits, for any attitude, from the colliders measured once upright (KsaVehicleHeight.Footprint): each collider's centre off the thrust axis (X, Y, model axes), the half-widths of its bounds (HalfX, HalfY) and the depth of its lowest point under the centre of mass when upright (Depth).
///  The colliders' bottoms are taken as ellipses with those half-widths, which is exact for a round skirt or footpad. With the up direction <c>u</c> in model axes (z along the thrust axis, up the vehicle), a collider's lowest point is <c>Depth uz - X ux - Y uy + sqrt((HalfX ux)^2 + (HalfY uy)^2)</c> below the centre of mass, and the craft's is the largest over the colliders. Upright, u = (0, 0, 1), that is the deepest Depth: the measured touchdown height.
///  WHY. The touchdown height was measured along the thrust axis with the craft upright, which suits splayed legs. A Starship lands on its skirt, about 4.5 m out: tilted 2.5 deg its low edge is about 0.2 m nearer the ground than that, at 10 deg about 0.45 m, and on 2026-11-27 contact came with the legs' height still reading 0.2-0.5 m.
///  Pure: no game types, so UiLogic tests it. The geometry only changes when it is measured again (the vehicle height refresh, as the centre of mass moves or legs deploy); each step just reads it at the attitude.
/// </summary>
public static class ContactDepth
{
    /// <summary>The depth of the lowest point below the centre of mass, m, at the up direction (<paramref name="ux"/>, <paramref name="uy"/>, <paramref name="uz"/>) in model axes; NaN for no colliders.</summary>
    public static double DepthM(ReadOnlySpan<double> x, ReadOnlySpan<double> y, ReadOnlySpan<double> halfX, ReadOnlySpan<double> halfY,
                                ReadOnlySpan<double> depth, double ux, double uy, double uz)
    {
        int n = Math.Min(Math.Min(x.Length, y.Length), Math.Min(Math.Min(halfX.Length, halfY.Length), depth.Length));
        double norm = Math.Sqrt(ux * ux + uy * uy + uz * uz);
        if (n == 0 || !(norm > 0.0))
            return double.NaN;
        ux /= norm;
        uy /= norm;
        uz /= norm;
        double worst = double.NegativeInfinity;
        for (int i = 0; i < n; i++)
        {
            double reach = Math.Sqrt(halfX[i] * ux * (halfX[i] * ux) + halfY[i] * uy * (halfY[i] * uy));
            double d = depth[i] * uz - x[i] * ux - y[i] * uy + reach;
            if (double.IsFinite(d) && d > worst)
                worst = d;
        }
        return double.IsFinite(worst) ? worst : double.NaN;
    }

    /// <summary>
    /// THE NARROWEST STANCE, m: how far the craft's support reaches from the thrust axis in the direction it reaches least - the minimum over directions of the support function of the colliders whose lowest point is within <paramref name="bandM"/> of the deepest (bottoms as ellipses). A tip goes over the nearest edge in its direction, so a stance that is wide one way and narrow another stands only on its narrow side when it leans that way.
    ///  WHY (2026-11-27). The Starship's aft flaps reach about 3 m beyond its 4.5 m skirt, port and starboard only, with their trailing edges at the base. Taken as the widest reach, the stance was about 8 m all round: two touchdowns tilted 5-6 deg and sliding 2-3 m/s logged 39 and 50 % of what tips them, and both went over - on the skirt alone that was about four times.
    /// </summary>
    public static double NarrowestStanceM(ReadOnlySpan<double> x, ReadOnlySpan<double> y, ReadOnlySpan<double> halfX, ReadOnlySpan<double> halfY,
                                          ReadOnlySpan<double> depth, double bandM = 0.5)
    {
        int n = Math.Min(Math.Min(x.Length, y.Length), Math.Min(Math.Min(halfX.Length, halfY.Length), depth.Length));
        if (n == 0)
            return double.NaN;
        double deepest = double.NegativeInfinity;
        for (int i = 0; i < n; i++)
            deepest = Math.Max(deepest, depth[i]);
        double narrowest = double.PositiveInfinity;
        for (int a = 0; a < 360; a++)
        {
            double c = Math.Cos(a * Math.PI / 180.0), s = Math.Sin(a * Math.PI / 180.0), reach = double.NegativeInfinity;
            for (int i = 0; i < n; i++)
                if (depth[i] >= deepest - bandM)
                    reach = Math.Max(reach, x[i] * c + y[i] * s + Math.Sqrt(halfX[i] * c * (halfX[i] * c) + halfY[i] * s * (halfY[i] * s)));
            narrowest = Math.Min(narrowest, reach);
        }
        return double.IsFinite(narrowest) && narrowest > 0.0 ? narrowest : double.NaN;
    }

    /// <summary>How much deeper the lowest point sits at this attitude than upright, m: what the legs' height measured upright overstates the clearance by. Zero for no colliders; negative where tilting lifts the lowest point (one collider on the axis).</summary>
    public static double ExtraDepthM(ReadOnlySpan<double> x, ReadOnlySpan<double> y, ReadOnlySpan<double> halfX, ReadOnlySpan<double> halfY,
                                     ReadOnlySpan<double> depth, double ux, double uy, double uz)
    {
        double now = DepthM(x, y, halfX, halfY, depth, ux, uy, uz);
        double upright = DepthM(x, y, halfX, halfY, depth, 0.0, 0.0, 1.0);
        return double.IsFinite(now) && double.IsFinite(upright) ? now - upright : 0.0;
    }
}

#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.Bellyflop;

/// <summary>
/// Measures the live craft, body and belly-first sweep into bellyflop's model (BellyflopModel), in the 3-DOF's landing-site frame, and reads the craft's attitude back as bellyflop's angles.
/// </summary>
internal static class KsaBellyflopSetup
{
    /// <summary>
    /// The model for a craft over its site. <paramref name="siteRadius"/> is the site's distance from the body's centre (mean radius plus terrain): the model's datum, so altitude zero is the site's ground. <paramref name="bearingLocal"/> is the yaw reference, horizontal in the site frame. Null, with a reason, where the sweep has no belly-first body table.
    /// </summary>
    public static BellyflopModel Build(Vehicle vehicle, IParentBody parent, KsaFrameBridge.SiteFrame frame, double siteRadius,
                                       KsaAeroSweep.Result aero, double3 bearingLocal, double forceScale, out string why)
    {
        why = "";
        BodyAeroTable table = aero?.BellyBodyAero;
        AtmosphereModel atm = aero?.Atmosphere;
        if (table == null || atm == null)
        {
            why = "no belly-first body table";
            return null;
        }
        double3 omega = frame.VecToLocal(parent.GetAngularVelocityCci());
        double terrain = siteRadius - parent.MeanRadius;
        double3 bearing = new double3(bearingLocal.X, bearingLocal.Y, 0).NormalizeOrZero();
        if (bearing.Length() < 0.5)
            bearing = new double3(1, 0, 0);
        return new BellyflopModel
        {
            Mu = parent.Mu,
            BodyRadius = siteRadius,
            OmegaX = omega.X, OmegaY = omega.Y, OmegaZ = omega.Z,
            Density = h => atm.Density(h + terrain),
            SpeedOfSound = atm.SpeedOfSound,
            SpeedOfSoundAt = h => atm.SpeedOfSoundAt(h + terrain),
            Mass = Math.Max(vehicle.TotalMass, 1.0),
            ReferenceArea = aero.ReferenceArea,
            Coefficients = Coefficients(table),
            BearingX = bearing.X, BearingY = bearing.Y,
            ForceScale = forceScale,
        };
    }

    /// <summary>
    /// The table's coefficients in bellyflop's belly axes. The table's frame leads along the belly (Lead), takes alpha towards the nose (Alpha, body +x) and beta towards its remaining axis, whose sign depends on the side; bellyflop's span is belly x nose. So the angles go through the table's own Angles from the airflow in KSA body axes, and the body-axis coefficients are projected back onto nose, span and belly: no sign assumed.
    /// </summary>
    public static BellyCoefficients Coefficients(BodyAeroTable table)
    {
        BodyAeroFrame f = table.Frame;
        var nose = new double3(f.AX, f.AY, f.AZ);
        var belly = new double3(f.LX, f.LY, f.LZ);
        double3 span = double3.Cross(belly, nose);
        return (double mach, double alpha, double beta, out double cn, out double cs, out double cb) =>
        {
            double ta = Math.Tan(alpha), tb = Math.Tan(beta);
            double inv = 1.0 / Math.Sqrt(ta * ta + tb * tb + 1);
            double3 u = (ta * inv) * nose + (tb * inv) * span + inv * belly;
            BodyAeroTable.Angles(f, u.X, u.Y, u.Z, out double tAlpha, out double tBeta);
            table.Coefficients(Math.Clamp(mach, table.MachMin, table.MachMax), tAlpha, tBeta, out double cx, out double cy, out double cz);
            var c = new double3(cx, cy, cz);
            cn = double3.Dot(c, nose);
            cs = double3.Dot(c, span);
            cb = double3.Dot(c, belly);
        };
    }

    /// <summary>The craft's belly normal in KSA body axes: the table's leading axis.</summary>
    public static double3 BellyAxisBody(BodyAeroTable table) => new(table.Frame.LX, table.Frame.LY, table.Frame.LZ);

    /// <summary>
    /// The craft's attitude now as bellyflop's angles at the state <paramref name="x"/> (site frame): the airflow's angles on the belly from its real nose and belly, and the yaw of its nose about the airflow from the model's reference.
    /// </summary>
    public static (double Alpha, double Beta, double Psi) AnglesNow(Vehicle vehicle, KsaFrameBridge.SiteFrame frame, BellyflopModel model,
                                                                    double3 bellyBody, ReadOnlySpan<double> x)
    {
        doubleQuat body2Cci = KsaFrameBridge.BodyToCci(vehicle);
        double3 noseL = frame.VecToLocal(double3.UnitX.Transform(body2Cci)).NormalizeOrZero();
        double3 bellyL = frame.VecToLocal(bellyBody.Transform(body2Cci)).NormalizeOrZero();
        double3 spanL = double3.Cross(bellyL, noseL);
        var v = new V3(x[3], x[4], x[5]);
        var (a, b) = BellyflopModel.AnglesOf(v, ToV3(noseL), ToV3(spanL), ToV3(bellyL));
        double psi = 0.0;
        if (model.Axes(x, 0, 0, 0, out V3 n0, out _, out _, out double speed))
        {
            V3 f = v / speed;
            V3 np = ToV3(noseL) - f * V3.Dot(ToV3(noseL), f);
            if (np.Length > 1e-6)
                psi = Math.Atan2(V3.Dot(V3.Cross(n0, np), f), V3.Dot(n0, np));
        }
        return (a, b, psi);
    }

    public static V3 ToV3(double3 v) => new(v.X, v.Y, v.Z);
    public static double3 ToDouble3(V3 v) => new(v.X, v.Y, v.Z);
}

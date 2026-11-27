using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// A synthetic returning booster over a real-size spinning Earth, for the headless 3-DOF checks: the frame, the body and the vehicle the KSA model is built from in flight, with plausible numbers instead of the game's.
///
/// The site sits at 28.5 degrees north on the CCI x-z plane at t = 0, and the site frame is built exactly as KsaFrameBridge.BuildSiteFrame builds it, so a check that converts between the two exercises the same geometry the guidance does.
/// </summary>
internal static class ThreeDofScenario
{
    public const double Mu = 3.986004418e14;
    public const double Radius = 6.371e6;
    public const double Omega = 7.2921159e-5;
    public const double SiteLatDeg = 28.5;

    // A Falcon 9 first stage on one engine, near the end of its propellant.
    public const double DryMass = 25_600;
    public const double WetMass = 31_000;
    public const double VacuumThrust = 914e3;
    public const double ExitArea = 0.91;
    public const double IspVac = 311;
    public const double Area = 10.8;
    public const double ThrottleFloor = 0.40;

    public static double[] SiteDir => [Math.Cos(SiteLatDeg * Math.PI / 180), 0, Math.Sin(SiteLatDeg * Math.PI / 180)];

    /// <summary>Site frame axes in CCI at t = 0: east, north, up - KsaFrameBridge's construction.</summary>
    public static void Axes(out double[] ex, out double[] ey, out double[] ez)
    {
        ez = SiteDir;
        double[] reference = Math.Abs(ez[2]) < 0.99 ? [0, 0, 1] : [1, 0, 0];
        ex = Normalise(Cross(reference, ez));
        ey = Cross(ez, ex);
    }

    /// <summary>
    /// A slender booster's lift, flying engine first: attached-flow lift falling away towards broadside plus crossflow lift growing with the square of the angle, rising a little through the transonic region. Positive is towards the engine end's tilt, as KsaAeroSweep measures it.
    /// </summary>
    public static double[] LiftTable(double[] mach, double[] alphaDeg)
    {
        var cl = new double[mach.Length * alphaDeg.Length];
        for (int i = 0; i < mach.Length; i++)
        {
            double machFactor = 1.0 + 0.3 * Math.Exp(-Math.Pow((mach[i] - 1.1) / 0.4, 2));
            for (int j = 0; j < alphaDeg.Length; j++)
            {
                double a = alphaDeg[j] * Math.PI / 180;
                cl[i * alphaDeg.Length + j] = machFactor * (1.1 * Math.Sin(a) * Math.Cos(a) + 0.9 * Math.Sin(a) * Math.Abs(Math.Sin(a)));
            }
        }
        return cl;
    }

    public static KsaPointMassModel Model(bool lift = true)
    {
        Axes(out double[] ex, out double[] ey, out double[] ez);
        double[] w = [0, 0, Omega];
        double[] mach = AeroTable.DefaultMachBreakpoints, alpha = AeroTable.DefaultAlphaBreakpointsDeg;
        return new KsaPointMassModel
        {
            Mu = Mu,
            MeanRadius = Radius,
            CentreZ = -Radius,
            OmegaX = Dot(w, ex),
            OmegaY = Dot(w, ey),
            OmegaZ = Dot(w, ez),
            Atmosphere = ExponentialAtmosphere.Earth,
            Drag = new AeroTable(),
            LiftSlope = lift ? KsaPointMassModel.LiftSlopeTable(mach, alpha, LiftTable(mach, alpha)) : null,
            ReferenceArea = Area,
            VacuumThrust = VacuumThrust,
            ExitArea = ExitArea,
            MassFlow = VacuumThrust / (IspVac * 9.80665),
        };
    }

    /// <summary>
    /// A body-axis table (BodyAeroTable) built from the scenario's own axisymmetric drag and lift - drag against the airflow, lift along the engine end's tilt, both of the total angle only - plus a fixed force on body +y (<paramref name="side"/>) and +z (<paramref name="up"/>) at every node, as an asymmetric fin set makes, in coefficients.
    /// </summary>
    public static BodyAeroTable BodyTable(double side = 0.0, double up = 0.0)
    {
        double[] mach = AeroTable.DefaultMachBreakpoints, alphaGrid = AeroTable.DefaultAlphaBreakpointsDeg;
        var drag = new AeroTable();
        var lift = new AeroTable(mach, alphaGrid, LiftTable(mach, alphaGrid));
        double[] angles = { -20, -16, -12, -8, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 8, 12, 16, 20 };
        var c = new double[mach.Length * angles.Length * angles.Length * 3];
        for (int i = 0; i < mach.Length; i++)
            for (int ja = 0; ja < angles.Length; ja++)
                for (int jb = 0; jb < angles.Length; jb++)
                {
                    (double ux, double uy, double uz) = BodyAeroTable.AirDirection(angles[ja] * Math.PI / 180, angles[jb] * Math.PI / 180);
                    double total = Math.Acos(Math.Clamp(-ux, -1, 1));
                    double cd = drag.Cd(mach[i], total), cl = lift.Cd(mach[i], total);
                    double px = 1 - ux * ux, py = -ux * uy, pz = -ux * uz;
                    double pn = Math.Sqrt(px * px + py * py + pz * pz);
                    double lx = pn > 1e-12 ? -px / pn : 0, ly = pn > 1e-12 ? -py / pn : 0, lz = pn > 1e-12 ? -pz / pn : 0;
                    int n = ((i * angles.Length + ja) * angles.Length + jb) * 3;
                    c[n] = -cd * ux + cl * lx;
                    c[n + 1] = -cd * uy + cl * ly + side;
                    c[n + 2] = -cd * uz + cl * lz + up;
                }
        return new BodyAeroTable(mach, angles, angles, c);
    }

    /// <summary>The flight config: the script's problem plus the blocks a landing needs.</summary>
    /// <param name="accelMax">Thrust acceleration limit, m/s^2, or 0 for none; see Scvx3DofConfig.AccelMax.</param>
    public static Scvx3DofConfig Config(int nodes = 30, int glideIntervals = 12, double alphaMaxDeg = 15.0, bool anchor = false,
                                        double accelMax = 0.0, double turnAccel = 0.0, double vcVelocity = 0.0,
                                        double burnTiltPerSDeg = 0.0, bool useAugmentedLagrangian = true)
    {
        var min = Enumerable.Repeat(double.NegativeInfinity, 7).ToArray();
        var max = Enumerable.Repeat(double.PositiveInfinity, 7).ToArray();
        min[PointMass3Dof.IR + 2] = 0.0;
        min[PointMass3Dof.IM] = DryMass;
        return new Scvx3DofConfig
        {
            Nodes = nodes,
            GlideIntervals = glideIntervals,
            ThrottleFloor = ThrottleFloor,
            AccelMax = accelMax,
            AlphaMaxDeg = alphaMaxDeg,
            StateMin = min,
            StateMax = max,
            TerminalAttitude = [0, 0, 1],
            AttitudeRateMax = 10.0 * Math.PI / 180,
            AttitudeAccelMax = turnAccel,
            BurnTiltPerS = burnTiltPerSDeg * Math.PI / 180,
            VirtualControlScale = vcVelocity > 0.0 ? [0, 0, 0, vcVelocity, vcVelocity, vcVelocity, 0] : null,
            UseAugmentedLagrangian = useAugmentedLagrangian,
            PathSlackWeight = 1e3,
            AttitudeAnchor = anchor,
            // Soft terminal, the 6-DOF's weights: miss the pad before faking the dynamics, and miss the speed last of all.
            TerminalMissWeight = 1e3,
            TerminalSpeedWeight = 1e4,
            GlideSigmaMax = 200,
            BurnSigmaMin = 2,
            BurnSigmaMax = 60,
            SigmaScale = 30,
            XScale = [2000, 2000, 5000, 300, 300, 300, WetMass],
        };
    }

    public static double Dot(double[] a, double[] b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    public static double[] Cross(double[] a, double[] b) =>
        [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    public static double[] Normalise(double[] v)
    {
        double l = Math.Sqrt(Dot(v, v));
        return [v[0] / l, v[1] / l, v[2] / l];
    }
}

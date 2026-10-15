#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Text;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Guidance.Numerics.Flight;

/// <summary>
/// Measures the aerodynamic force the game applies to the current vehicle onto a Cd(Mach, alpha) grid, and pairs it with an atmosphere that mirrors the game's.
///  THIS IS A MEASUREMENT, NOT A MODEL. Every Cd below is read off PhysicsStates.ComputeDerivatives - the force function the integrator itself calls every substep - run on a copy of the live vehicle's state with the airspeed set per sample. Whatever drag model is active is what gets measured: stock's box, or a mod that replaces it, with no knowledge here of which. A KSA update or an aero mod that changes the forces changes these numbers with it.
///  The force along the airspeed goes into the drag table, because that is all the solvers model. The force across it is measured too: its size is reported in <see cref="Result.LiftToDragMax"/> so it is visible rather than silently dropped, and its signed component goes into <see cref="Result.Lift"/>, which the boostback glide steers with.
/// </summary>
public static class KsaAeroSweep
{
    /// <summary>Roll azimuths averaged over per alpha. 15 degree steps, which land on both the 0 and 45 degree extremes of a box model, so the spread is exact and the mean within half a percent.</summary>
    private const int RollSamples = 24;

    /// <summary>The Mach 0 row is a limit, not a speed anything can be measured at. It is sampled here instead.</summary>
    private const double MinProbeMach = 0.05;

    /// <summary>Airspeed of the baseline probe, m/s. Low enough that its drag is a part in 10^7 of the slowest real sample, and not zero, so the game still takes its atmospheric branch and the baseline carries the same buoyancy the samples do.</summary>
    private const double BaselineSpeed = 0.01;

    /// <summary>Alpha the pointing sensitivity is read at, degrees from tail-first.</summary>
    private const double PointingAlphaDeg = 5.0;

    /// <summary>
    /// Mach breakpoints past the default grid's last at 5, out to orbital entry.
    ///  A booster coming back from a high apogee enters at Mach 7 or 8, and beyond the last breakpoint the fit extrapolates along its end slope. On a measured lift curve that slope pointed down, and by Mach 7.8 the extrapolated lift had changed sign - the glide read it as a body that lifts the other way and tilted the booster against its own correction for the whole of its upper entry. Coefficients level off at high Mach, so a handful of coarse breakpoints is all the hypersonic end needs.
    /// </summary>
    private static readonly double[] HypersonicMachBreakpoints = { 6.0, 7.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0 };

    private static double[] SweepMachBreakpoints()
    {
        double[] grid = AeroTable.DefaultMachBreakpoints;
        var all = new System.Collections.Generic.List<double>(grid);
        foreach (double m in HypersonicMachBreakpoints)
            if (m > grid[^1])
                all.Add(m);
        return all.ToArray();
    }

    /// <summary>
    /// One sweep's worth of results: the fitted surrogate, the atmosphere it goes with, and everything needed to judge whether either is trustworthy.
    ///  Immutable by construction and built entirely from copies. Once this exists it shares nothing with the game, which is what makes it safe to hand to a solver on another thread - the same argument Ksa6DofInputs makes for bias and inertia.
    /// </summary>
    public sealed class Result
    {
        /// <summary>The fitted Cd(Mach, alpha) surrogate.</summary>
        public AeroTable Table;

        /// <summary>
        /// The fitted lift coefficient C_L(Mach, alpha) - read it through <see cref="LiftAt"/>, since AeroTable names its lookup Cd. Same grid, reference area and retrograde-first alpha as <see cref="Table"/>.
        ///  SIGNED BY WHERE THE ENGINE END POINTS: positive is lift towards the side the engine end is tilted off the airflow, negative away from it. A slender body leading with its engine lifts the positive way, from crossflow on its flanks; a blunt one leading with a heat shield lifts the negative way, from its tilted axial force; stock's box makes none.
        /// </summary>
        public AeroTable Lift;
        public double[] LiftTable;

        /// <summary>Signed C_L at a Mach number and retrograde-first alpha (radians). Zero when there is no lift table.</summary>
        public double LiftAt(double mach, double alphaRad) => Lift?.Cd(mach, alphaRad) ?? 0.0;

        /// <summary>The game's atmosphere, mirrored. Null if the body has none.</summary>
        public ExponentialAtmosphere Atmosphere;

        public double[] MachGrid;
        public double[] AlphaGridDeg;
        public double[] CdTable;

        /// <summary>Frontal area the Cd values are referenced to, m^2: pi/4 * dy * dz off the bounding box. Only a normalisation - the solvers multiply it back in - kept as the nose face so the numbers read like Cd on the cross-section.</summary>
        public double ReferenceArea;

        /// <summary>Bounding-box extents along the assembly axes (x = long axis), m.</summary>
        public double3 BoxExtents;

        /// <summary>Vehicle mass at the moment of sampling, kg. Not used by the table;
        /// recorded so a stale sweep is recognisable.</summary>
        public double Mass;

        /// <summary>Headline Cd values at the lowest Mach row, referenced to <see cref="ReferenceArea"/>.</summary>
        public double CdTailFirst, CdBroadside, CdNoseFirst;

        /// <summary>
        /// How much Cd rises from tail-first to 5 degrees off it, as a fraction, at the lowest Mach row. Small means a few degrees of pointing error near the boostback attitude costs almost nothing, which is load-bearing for guidance.
        /// </summary>
        public double PointingSensitivity;

        /// <summary>Cd(broadside) / Cd(tail-first): how much the drag varies across the whole attitude range.</summary>
        public double AttitudeSensitivity;

        /// <summary>
        /// How much Cd varies with ROLL at fixed alpha, as a fraction of its roll mean, at its worst alpha. The table has no roll input, so it stores the azimuthal mean; this says how much that averaging threw away.
        ///  It goes to zero only if the active model is axisymmetric. Stock's is a box, so a square-section booster rolled 45 degrees presents sqrt(2) times the flank it presents at 0, which is roughly 25% for a slender stack.
        /// </summary>
        public double RollSpread;

        /// <summary>The largest fractional change in Cd across the Mach axis at any one alpha. Zero means the active model has no compressibility.</summary>
        public double MachSpread;

        /// <summary>The largest lift-to-drag ratio anywhere on the grid. The table holds drag only, so this is how much force the solvers leave out.</summary>
        public double LiftToDragMax;

        /// <summary>Largest relative disagreement between our mirrored density and
        /// KSA's own GetAtmosphericDensityAtAltitude, sampled across the atmosphere.
        /// Should be at the level of float/double rounding.</summary>
        public double AtmosphereMirrorError;

        /// <summary>Body the atmosphere was read from, for the readout.</summary>
        public string BodyName = "";

        /// <summary>Sim time the sweep was taken at.</summary>
        public double SampledAt;

        /// <summary>Bounding box the sweep was taken from, to spot a stale table.</summary>
        public double3 SampledExtents;

        public int MachCount => MachGrid.Length;
        public int AlphaCount => AlphaGridDeg.Length;

        /// <summary>Cd at a grid node, for the readout.</summary>
        public double CdAt(int machIndex, int alphaIndex)
            => CdTable[machIndex * AlphaGridDeg.Length + alphaIndex];
    }

    /// <summary>
    /// Measure the focused vehicle's aerodynamics and fit the surrogate.
    ///  Main thread only: it reads Vehicle.Props, which the sim thread owns and rewrites. That is the same access the rest of the panel makes (TotalMass and friends), and it is why the result is a snapshot of plain arrays rather than anything that reaches back into the game.
    /// </summary>
    /// <returns>False with a reason in <paramref name="error"/> if the vehicle has no
    /// usable geometry, or the game's force function gives no drag. A body with no atmosphere is NOT an error - the probe flies through a nominal one, the table is still meaningful, and Atmosphere comes back null.</returns>
    public static bool TryBuild(Vehicle vehicle, IParentBody parent, double simTime,
                                out Result result, out string error)
    {
        result = null;
        error = "";

        if (vehicle == null)
        {
            error = "no vehicle";
            return false;
        }

        // Extents along the ASSEMBLY axes. x is the long axis for any sane rocket - the one the thrust axis lies along.
        //  Read through Vehicle's own accessor rather than off Props.BoundingBoxAsmb directly: that field is a BepuPhysics.Box, and touching it would drag a BepuPhysics reference into the mod for three floats. This is the same three floats, and it is what the staleness check reads too, so the two cannot disagree about which box the table was built from.
        float3 half = vehicle.BoundingBoxHalfExtentsAsmb;
        double dx = half.X * 2.0;
        double dy = half.Y * 2.0;
        double dz = half.Z * 2.0;
        double refArea = Math.PI / 4.0 * dy * dz;

        if (!(refArea > 0.0) || !double.IsFinite(refArea))
        {
            error = "vehicle has no bounding box yet";
            return false;
        }

        var res = new Result
        {
            ReferenceArea = refArea,
            BoxExtents = new double3(dx, dy, dz),
            SampledExtents = new double3(dx, dy, dz),
            Mass = vehicle.TotalMass,
            SampledAt = simTime,
        };
        BuildAtmosphere(parent, res);

        // The air the probe flies through. Sea level of this body's own atmosphere, so a model whose Cd depends on Mach through sqrt(gamma p / rho) sees the same Mach the solvers compute - KSA's air is isothermal, so p / rho, and with it the speed of sound, is the same at every altitude.
        ExponentialAtmosphere air = res.Atmosphere ?? ExponentialAtmosphere.Earth;
        if (!ForceProbe.TryCreate(vehicle, air.SeaLevelDensity, air.SeaLevelPressure, out ForceProbe probe, out error))
            return false;

        double[] machGrid = SweepMachBreakpoints();
        double[] alphaDeg = AeroTable.DefaultAlphaBreakpointsDeg;
        int nm = machGrid.Length;
        int na = alphaDeg.Length;

        // Every direction the grid uses, in body axes, with the force the game applies at a crawl in that direction. Subtracting it removes everything that does not depend on airspeed - buoyancy - and leaves the aerodynamics.
        //  RETROGRADE-FIRST: alpha = 0 means the wind comes at the TAIL, so the velocity in body axes points along -x. This is the sign that carries the whole convention - see AeroTable.AngleOfAttack.
        //  Beside each, the direction the ENGINE END (-x) is tilted off the airflow: -x less its component along u, which works out to -(sin a, cos a cos phi, cos a sin phi). Zero where it is undefined, at alpha 0 and 180, where there is no lift to sign anyway.
        var dirs = new double3[na, RollSamples];
        var tilts = new double3[na, RollSamples];
        var baseline = new double3[na, RollSamples];
        for (int j = 0; j < na; j++)
        {
            double alpha = alphaDeg[j] * Math.PI / 180.0;
            double sa = Math.Sin(alpha), ca = Math.Cos(alpha);
            for (int k = 0; k < RollSamples; k++)
            {
                double phi = 2.0 * Math.PI * k / RollSamples;
                dirs[j, k] = new double3(-ca, sa * Math.Cos(phi), sa * Math.Sin(phi));
                tilts[j, k] = sa > 1e-9
                    ? new double3(-sa, -ca * Math.Cos(phi), -ca * Math.Sin(phi))
                    : double3.Zero;
                baseline[j, k] = probe.ForceBody(BaselineSpeed * dirs[j, k]);
            }
        }

        var cdTable = new double[nm * na];
        var clTable = new double[nm * na];
        double rollSpread = 0.0, liftToDrag = 0.0;
        for (int i = 0; i < nm; i++)
        {
            double speed = Math.Max(machGrid[i], MinProbeMach) * air.SpeedOfSound;
            double qa = 0.5 * air.SeaLevelDensity * speed * speed * refArea;

            for (int j = 0; j < na; j++)
            {
                double dragSum = 0.0, liftSum = 0.0, signedLiftSum = 0.0, lo = double.MaxValue, hi = double.MinValue;
                for (int k = 0; k < RollSamples; k++)
                {
                    double3 u = dirs[j, k];
                    double3 force = probe.ForceBody(speed * u) - baseline[j, k];
                    double drag = -double3.Dot(force, u);
                    double3 across = force + drag * u;
                    dragSum += drag;
                    liftSum += across.Length();
                    signedLiftSum += double3.Dot(across, tilts[j, k]);
                    lo = Math.Min(lo, drag);
                    hi = Math.Max(hi, drag);
                }

                double cd = dragSum / RollSamples / qa;
                double cl = signedLiftSum / RollSamples / qa;
                if (!double.IsFinite(cd) || !double.IsFinite(cl))
                {
                    error = $"the game's force function gave a non-finite force at Mach {machGrid[i]:0.##}, alpha {alphaDeg[j]:0}";
                    return false;
                }
                cdTable[i * na + j] = cd;
                clTable[i * na + j] = cl;

                if (dragSum > 0.0)
                {
                    liftToDrag = Math.Max(liftToDrag, liftSum / dragSum);
                    if (i == 0)
                        rollSpread = Math.Max(rollSpread, (hi - lo) / (dragSum / RollSamples));
                }
            }
        }

        // The one thing every drag model has: drag. A probe that measured none was not exercising the game's atmosphere branch, and a table of zeros would plan through vacuum without saying so.
        double[] lowRow = new double[na];
        Array.Copy(cdTable, 0, lowRow, 0, na);
        if (!(lowRow[0] > 0.0) || !(lowRow[na - 1] > 0.0))
        {
            error = "the game applied no drag to the probe";
            return false;
        }

        double machSpread = 0.0;
        for (int j = 0; j < na; j++)
        {
            double mn = double.MaxValue, mx = double.MinValue;
            for (int i = 0; i < nm; i++)
            {
                mn = Math.Min(mn, cdTable[i * na + j]);
                mx = Math.Max(mx, cdTable[i * na + j]);
            }
            if (mn > 0.0)
                machSpread = Math.Max(machSpread, (mx - mn) / mn);
        }

        res.MachGrid = machGrid;
        res.AlphaGridDeg = alphaDeg;
        res.CdTable = cdTable;
        res.CdTailFirst = lowRow[0];
        res.CdNoseFirst = lowRow[na - 1];
        res.CdBroadside = InterpolateAt(alphaDeg, lowRow, 90.0);
        res.AttitudeSensitivity = res.CdBroadside / res.CdTailFirst;
        res.PointingSensitivity = InterpolateAt(alphaDeg, lowRow, PointingAlphaDeg) / res.CdTailFirst - 1.0;
        res.RollSpread = rollSpread;
        res.MachSpread = machSpread;
        res.LiftToDragMax = liftToDrag;
        res.LiftTable = clTable;

        try
        {
            res.Table = new AeroTable(machGrid, alphaDeg, cdTable);
            res.Lift = new AeroTable(machGrid, alphaDeg, clTable);
        }
        catch (Exception ex)
        {
            error = "table fit failed: " + ex.Message;
            return false;
        }

        result = res;
        return true;
    }

    /// <summary>
    /// A copy of one vehicle's physics state, with the air set per probe. The force read back is the game's own, from the same ComputeDerivatives the integrator calls, so any mod that patches the forces there is measured with them.
    /// </summary>
    private sealed class ForceProbe
    {
        private BubbleOrigin _origin;
        private KinematicStates _kinematic;
        private VehicleProperties _props;
        private PhysicsEnvironment _environment;

        /// <summary>The airspeed at zero physics velocity - the bubble's own motion against the turning air - in physics axes. Airspeed is this plus the physics velocity.</summary>
        private double3 _frameAirPhys;

        public static bool TryCreate(Vehicle vehicle, double density, double pressure, out ForceProbe probe, out string error)
        {
            probe = new ForceProbe
            {
                _origin = vehicle.BubbleOrigin,
                _kinematic = vehicle.KinematicStates,
                _props = vehicle.Props,
                _environment = vehicle.PhysicsEnvironment,
            };

            // Air, no water, no spin: a probe in free air at this body's sea level. Anything this leaves set, like gravity, lands in AccelPhys or in the buoyancy the baseline removes.
            probe._environment.InPhysicsRadius = true;
            probe._environment.AtmosphericDensity = (float)density;
            probe._environment.AtmosphericPressure = (float)pressure;
            probe._environment.OceanVolume = 0.0;
            probe._environment.OceanSurfaceArea = 0.0;
            probe._kinematic.AngularVelocityPhys = double3.Zero;

            // The game's own definition of airspeed, asked at zero physics velocity, gives the frame's share; every probe velocity is set relative to it.
            probe._kinematic.VelocityPhys = double3.Zero;
            float3 frameAir = PhysicsStates.ComputeAirVelocityBody(in probe._origin, in probe._kinematic, in probe._environment);
            probe._frameAirPhys = double3.Unpack(in frameAir).Transform(probe._kinematic.Body2Phys);

            // Check the airspeed lands where it was put, against the same game function. A frame this gets wrong would measure the drag of the wrong direction.
            var check = new double3(-100.0, 30.0, 10.0);
            probe._kinematic.VelocityPhys = check.Transform(probe._kinematic.Body2Phys) - probe._frameAirPhys;
            float3 got = PhysicsStates.ComputeAirVelocityBody(in probe._origin, in probe._kinematic, in probe._environment);
            double miss = (double3.Unpack(in got) - check).Length();
            if (!(miss < 0.01))
            {
                error = $"could not set the probe's airspeed (missed by {miss:G3} m/s)";
                return false;
            }

            error = "";
            return true;
        }

        /// <summary>The body-axis force the game applies at this air velocity (body axes), buoyancy included.</summary>
        public double3 ForceBody(double3 airVelocityBody)
        {
            _kinematic.VelocityPhys = airVelocityBody.Transform(_kinematic.Body2Phys) - _frameAirPhys;

            // dt = 0 turns off the semi-implicit limiter stock applies to its drag, which is a property of the integrator rather than of the air. No thrusters, no chutes.
            Disturbances d = PhysicsStates.ComputeDerivatives(in _origin, in _kinematic, in _props, in _environment,
                0.0, 0.0, _origin.PositionBub, _origin.VelocityBub, default, default);
            return d.ForceBody;
        }
    }

    /// <summary>
    /// Mirror the parent body's atmosphere, and CHECK the mirror against the game rather than assuming it.
    ///  The check is the point. Three numbers copied across and an exponential written out again is exactly the kind of thing that is right until someone changes a unit, and the failure mode - a solver planning through slightly the wrong air - produces plans that are plausible and wrong rather than plans that break. So every resample re-verifies it against KSA's own GetAtmosphericDensityAtAltitude and records the worst disagreement.
    /// </summary>
    private static void BuildAtmosphere(IParentBody parent, Result res)
    {
        AtmosphereReference reference = parent?.GetAtmosphereReference();
        if (reference == null)
            return;

        PhysicalAtmosphereReference phys = reference.Physical;
        double rho0 = phys.SeaLevelDensity;
        double p0 = phys.SeaLevelPressure;
        double h = phys.ScaleHeight.InMeters();

        if (!(rho0 > 0.0) || !(p0 > 0.0) || !(h > 0.0))
            return;

        res.Atmosphere = new ExponentialAtmosphere(rho0, p0, h);
        // IParentBody does not carry a name; Astronomical does, and every body a vehicle can orbit is one.
        res.BodyName = (parent as Astronomical)?.Id ?? "";

        // Sample the whole column, including above the cutoff, so the boundary height is checked too and not just the exponential.
        double worst = 0.0;
        double top = res.Atmosphere.TopAltitude;
        double gameTop = phys.Height;
        for (int i = 0; i <= 200; i++)
        {
            double alt = top * 1.05 * i / 200.0;
            double ours = res.Atmosphere.Density(alt);
            double theirs = phys.GetAtmosphericDensityAtAltitude(alt);

            // KSA zeroes density outside the boundary in PhysicsEnvironment rather than inside GetAtmosphericDensityAtAltitude, which keeps returning the exponential. Compare against the game's EFFECTIVE density, which is what a vehicle experiences.
            if (alt >= gameTop)
                theirs = 0.0;

            double scale = Math.Max(Math.Abs(theirs), 1e-12);
            worst = Math.Max(worst, Math.Abs(ours - theirs) / scale);
        }
        res.AtmosphereMirrorError = worst;
    }

    /// <summary>Linear interpolation on the alpha grid, for the headline readouts.</summary>
    private static double InterpolateAt(double[] alphaDeg, double[] values, double atDeg)
    {
        if (atDeg <= alphaDeg[0]) return values[0];
        if (atDeg >= alphaDeg[^1]) return values[^1];
        for (int i = 1; i < alphaDeg.Length; i++)
        {
            if (atDeg > alphaDeg[i]) continue;
            double t = (atDeg - alphaDeg[i - 1]) / (alphaDeg[i] - alphaDeg[i - 1]);
            return values[i - 1] + t * (values[i] - values[i - 1]);
        }
        return values[^1];
    }

    /// <summary>
    /// The sampled table as CSV, for pasting into a plot or a regression test. Mach down the rows, alpha across - the same order the flat array is stored in.
    /// </summary>
    public static string ToCsv(Result r)
    {
        if (r == null) return "";
        var sb = new StringBuilder();
        sb.Append("mach\\alpha_deg");
        for (int j = 0; j < r.AlphaCount; j++)
            sb.Append(',').Append(r.AlphaGridDeg[j].ToString("0.##"));
        sb.AppendLine();
        for (int i = 0; i < r.MachCount; i++)
        {
            sb.Append(r.MachGrid[i].ToString("0.##"));
            for (int j = 0; j < r.AlphaCount; j++)
                sb.Append(',').Append(r.CdAt(i, j).ToString("0.####"));
            sb.AppendLine();
        }
        return sb.ToString();
    }
}

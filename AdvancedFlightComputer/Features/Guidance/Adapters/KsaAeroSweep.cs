#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Diagnostics;
using System.Text;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Core;
using AdvancedFlightComputer.Guidance.Numerics.Flight;

/// <summary>
/// Measures the aerodynamic force the game applies to the current vehicle onto a Cd(Mach, alpha) grid, and pairs it with an atmosphere that mirrors the game's.
///  THIS IS A MEASUREMENT, NOT A MODEL. Every Cd below is read off PhysicsStates.ComputeDerivatives - the force function the integrator itself calls every substep - run on a copy of the live vehicle's state, with its part tree, and with the airspeed set per sample. Whatever aero model is active is what gets measured: the game's pressure and skin friction over the craft's aero mesh, or a mod that replaces it, with no knowledge here of which. A KSA update or an aero mod that changes the forces changes these numbers with it.
///  The force along the airspeed goes into the drag table, because that is all the solvers model. The force across it is measured too: its size is reported in <see cref="Result.LiftToDragMax"/> so it is visible rather than silently dropped, and its signed component goes into <see cref="Result.Lift"/>, which the boostback glide steers with.
/// </summary>
public static class KsaAeroSweep
{
    /// <summary>Roll azimuths averaged over per alpha, in 15 degree steps. They land on the 0 and 45 degree extremes of a square section and on the 0, 30 and 60 degree marks of three side boosters.</summary>
    private const int RollSamples = 24;

    /// <summary>The Mach 0 row is a limit, not a speed anything can be measured at. It is sampled here instead.</summary>
    private const double MinProbeMach = 0.05;

    /// <summary>Airspeed of the baseline probe, m/s. Low enough that its drag is a part in 10^7 of the slowest real sample, and not zero, so the game still takes its atmospheric branch and the baseline carries the same buoyancy the samples do.</summary>
    private const double BaselineSpeed = 0.01;

    /// <summary>Alpha the pointing sensitivity is read at, degrees from tail-first.</summary>
    private const double PointingAlphaDeg = 5.0;

    /// <summary>Uniform intervals the atmosphere column is sampled in, before the points of the temperature profile are added. 2048 aims for a chord error of ln(rho) near 1e-5; the mirror check in the boostback gauge measures the real error per body.</summary>
    private const int AtmosphereIntervals = 2048;

    /// <summary>The largest relative density error of the sampled atmosphere that the readout shows as good.</summary>
    public const double MirrorTolerance = 1e-4;

    /// <summary>The error of a sweep asked for while the game rebuilds the craft's aero mesh. The sweep would wait for the build on the main thread, so it returns this instead, and the caller keeps its table and asks again on a later frame.</summary>
    public const string RebuildingError = "aero mesh rebuilding";

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
        ///  SIGNED BY WHERE THE ENGINE END POINTS: positive is lift towards the side the engine end is tilted off the airflow, negative away from it. A slender body leading with its engine lifts the positive way, from crossflow on its flanks; a blunt one leading with a heat shield lifts the negative way, from its tilted axial force.
        /// </summary>
        public AeroTable Lift;
        public double[] LiftTable;

        /// <summary>Signed C_L at a Mach number and retrograde-first alpha (radians). Zero when there is no lift table.</summary>
        public double LiftAt(double mach, double alphaRad) => Lift?.Cd(mach, alphaRad) ?? 0.0;

        /// <summary>The game's atmosphere, sampled. Null if the body has none, and then <see cref="Table"/> and <see cref="Lift"/> are all zero, because the game applies no aerodynamic force there.</summary>
        public AtmosphereModel Atmosphere;

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
        ///  It goes to zero only for an airframe that is axisymmetric about its long axis. Fins, side boosters and a square section all spread it.
        /// </summary>
        public double RollSpread;

        /// <summary>The largest fractional change in Cd across the Mach axis at any one alpha. Zero means the active model has no compressibility.</summary>
        public double MachSpread;

        /// <summary>The largest lift-to-drag ratio anywhere on the grid. The table holds drag only, so this is how much force the solvers leave out.</summary>
        public double LiftToDragMax;

        /// <summary>Largest relative disagreement between the sampled density and KSA's own PhysicalAtmosphere.ComputeDensityAtAltitude, checked between the samples across the whole column. Below <see cref="MirrorTolerance"/> when the samples are dense enough.</summary>
        public double AtmosphereMirrorError;

        /// <summary>Body the atmosphere was read from, for the readout.</summary>
        public string BodyName = "";

        /// <summary>Sim time the sweep was taken at.</summary>
        public double SampledAt;

        /// <summary>Bounding box the sweep was taken from, to spot a stale table.</summary>
        public double3 SampledExtents;

        /// <summary>AeroMeshSource.Hash of the aero mesh the sweep measured, or 0 when the craft had none. The game rebuilds the mesh when this hash changes, so a different live hash means a stale table.</summary>
        public ulong MeshHash;

        /// <summary>Wall time the sweep took, ms.</summary>
        public double SweepMillis;

        public int MachCount => MachGrid.Length;
        public int AlphaCount => AlphaGridDeg.Length;

        /// <summary>Cd at a grid node, for the readout.</summary>
        public double CdAt(int machIndex, int alphaIndex)
            => CdTable[machIndex * AlphaGridDeg.Length + alphaIndex];
    }

    /// <summary>
    /// Measure the focused vehicle's aerodynamics and fit the surrogate.
    ///  Main thread only: it reads Vehicle.Props, which the sim thread owns and rewrites. That is the same access the rest of the console makes (TotalMass and friends), and it is why the result is a snapshot of plain arrays rather than anything that reaches back into the game.
    /// </summary>
    /// <returns>False with a reason in <paramref name="error"/> if the vehicle has no
    /// usable geometry, the game's force function gives no drag, or the game rebuilds the aero mesh (<see cref="RebuildingError"/>). A body with no atmosphere is NOT an error: the game applies no aerodynamic force there, so nothing is measured, Atmosphere comes back null and the tables are zero.</returns>
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

        double[] machGrid = SweepMachBreakpoints();
        double[] alphaDeg = AeroTable.DefaultAlphaBreakpointsDeg;
        int nm = machGrid.Length;
        int na = alphaDeg.Length;

        // PhysicsStates.ComputeAerodynamicLoads applies no force without the parent's PhysicalAtmosphere, so on an airless body there is nothing to measure. Zero tables keep every reader that takes the table as given working, and with no atmosphere they never meet any air.
        PhysicalAtmosphere physical = parent?.GetPhysicalAtmosphere();
        if (physical == null || !(physical.Height > 0.0))
        {
            res.MachGrid = machGrid;
            res.AlphaGridDeg = alphaDeg;
            res.CdTable = new double[nm * na];
            res.LiftTable = new double[nm * na];
            res.Table = new AeroTable(machGrid, alphaDeg, res.CdTable);
            res.Lift = new AeroTable(machGrid, alphaDeg, res.LiftTable);
            result = res;
            return true;
        }

        // PartTree.AwaitAerodynamics blocks while a mesh build runs, which would stall the main thread. The game flies the old mesh until the build ends, and the caller keeps its old table until then too.
        PartTree parts = vehicle.Parts;
        if (parts != null && parts.IsRebuildingAerodynamics)
        {
            error = RebuildingError;
            return false;
        }
        if (parts != null && parts.HasAerodynamicsBuildFailed)
            LogHelper.WarnOnce("afc-aero-build-failed:" + vehicle.Id,
                $"[AFC] The game could not build the aero mesh of '{vehicle.Id}'. The aero sweep measures the drag the game applies without it.");
        res.MeshHash = parts?.Aerodynamics?.Source?.Hash ?? 0UL;

        // The air the probe flies through: sea level of this body's own atmosphere. The game reads the Mach number from the speed of sound it is given and the gas properties from the temperature, so both come from the same sea level temperature. The Mach axis is airspeed over the sea level speed of sound, the same scaling the solvers use.
        double seaTemperature = physical.ComputeTemperatureAtAltitude(0.0);
        AtmosphericConditions sea = physical.ComputeConditionsAtAltitude(0.0);
        double speedOfSound = physical.Mixture.ComputeSpeedOfSound(seaTemperature);
        if (!ForceProbe.TryCreate(vehicle, parts, sea.Density, sea.Pressure, seaTemperature, speedOfSound, out ForceProbe probe, out error))
            return false;

        // Every direction the grid uses, in body axes, with the force the game applies at a crawl in that direction. Subtracting it removes everything that does not depend on airspeed - buoyancy - and leaves the aerodynamics.
        //  RETROGRADE-FIRST: alpha = 0 means the wind comes at the TAIL, so the velocity in body axes points along -x. This is the sign that carries the whole convention - see AeroTable.AngleOfAttack.
        //  Beside each, the direction the ENGINE END (-x) is tilted off the airflow: -x less its component along u, which works out to -(sin a, cos a cos phi, cos a sin phi). Zero where it is undefined, at alpha 0 and 180, where there is no lift to sign anyway.
        //  The direction is the outer loop and the Mach rows the inner one, because AeroSurfaceFlow.Update recomputes the shielding of the whole mesh each time the flow direction moves, and a sample at the same direction reuses it.
        var speeds = new double[nm];
        for (int i = 0; i < nm; i++)
            speeds[i] = Math.Max(machGrid[i], MinProbeMach) * speedOfSound;
        var dragSums = new double[nm * na];
        var liftSums = new double[nm * na];
        var signedLiftSums = new double[nm * na];
        var lowRowLo = new double[na];
        var lowRowHi = new double[na];
        var clock = Stopwatch.StartNew();
        for (int j = 0; j < na; j++)
        {
            double alpha = alphaDeg[j] * Math.PI / 180.0;
            double sa = Math.Sin(alpha), ca = Math.Cos(alpha);
            lowRowLo[j] = double.MaxValue;
            lowRowHi[j] = double.MinValue;
            for (int k = 0; k < RollSamples; k++)
            {
                double phi = 2.0 * Math.PI * k / RollSamples;
                var u = new double3(-ca, sa * Math.Cos(phi), sa * Math.Sin(phi));
                double3 tilt = sa > 1e-9
                    ? new double3(-sa, -ca * Math.Cos(phi), -ca * Math.Sin(phi))
                    : double3.Zero;
                double3 baseline = probe.ForceBody(BaselineSpeed * u);
                for (int i = 0; i < nm; i++)
                {
                    double3 force = probe.ForceBody(speeds[i] * u) - baseline;
                    double drag = -double3.Dot(force, u);
                    double3 across = force + drag * u;
                    int cell = i * na + j;
                    dragSums[cell] += drag;
                    liftSums[cell] += across.Length();
                    signedLiftSums[cell] += double3.Dot(across, tilt);
                    if (i == 0)
                    {
                        lowRowLo[j] = Math.Min(lowRowLo[j], drag);
                        lowRowHi[j] = Math.Max(lowRowHi[j], drag);
                    }
                }
            }
        }
        res.SweepMillis = clock.Elapsed.TotalMilliseconds;
        DefaultCategory.Log.Info(FormattableString.Invariant(
            $"[AFC] Aero sweep of '{vehicle.Id}' over {(parent as Astronomical)?.Id}: {na * RollSamples} directions, {na * RollSamples * (nm + 1)} samples, {res.SweepMillis:F0} ms."));

        var cdTable = new double[nm * na];
        var clTable = new double[nm * na];
        double rollSpread = 0.0, liftToDrag = 0.0;
        for (int i = 0; i < nm; i++)
        {
            double qa = 0.5 * sea.Density * speeds[i] * speeds[i] * refArea;
            for (int j = 0; j < na; j++)
            {
                int cell = i * na + j;
                double dragSum = dragSums[cell];
                double cd = dragSum / RollSamples / qa;
                double cl = signedLiftSums[cell] / RollSamples / qa;
                if (!double.IsFinite(cd) || !double.IsFinite(cl))
                {
                    error = $"the game's force function gave a non-finite force at Mach {machGrid[i]:0.##}, alpha {alphaDeg[j]:0}";
                    return false;
                }
                cdTable[cell] = cd;
                clTable[cell] = cl;

                if (dragSum > 0.0)
                {
                    liftToDrag = Math.Max(liftToDrag, liftSums[cell] / dragSum);
                    if (i == 0)
                        rollSpread = Math.Max(rollSpread, (lowRowHi[j] - lowRowLo[j]) / (dragSum / RollSamples));
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
    ///  The part tree is the live one, because ComputeDerivatives reads the aero mesh from it. Its AeroSurfaceFlow keeps the flow of the last direction in buffers that the vehicle's physics step also writes, so the probe runs only where no worker steps the vehicle, as in the Vehicle.PrepareWorker prefix.
    /// </summary>
    private sealed class ForceProbe
    {
        private BubbleOrigin _origin;
        private KinematicStates _kinematic;
        private VehicleProperties _props;
        private PhysicsEnvironment _environment;
        private PartTree _parts;

        /// <summary>The airspeed at zero physics velocity - the bubble's own motion against the turning air - in physics axes. Airspeed is this plus the physics velocity.</summary>
        private double3 _frameAirPhys;

        public static bool TryCreate(Vehicle vehicle, PartTree parts, double density, double pressure, double temperature, double speedOfSound, out ForceProbe probe, out string error)
        {
            probe = new ForceProbe
            {
                _origin = vehicle.BubbleOrigin,
                _kinematic = vehicle.KinematicStates,
                _props = vehicle.Props,
                _environment = vehicle.PhysicsEnvironment,
                _parts = parts,
            };

            // Air, no water, no spin: a probe in free air at this body's sea level. Anything this leaves set, like gravity, lands in AccelPhys or in the buoyancy the baseline removes.
            probe._environment.InPhysicsRadius = true;
            probe._environment.AtmosphericDensity = (float)density;
            probe._environment.AtmosphericPressure = (float)pressure;
            probe._environment.AtmosphericTemperature = (float)temperature;
            probe._environment.SpeedOfSound = (float)speedOfSound;
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
                0.0, 0.0, _origin.PositionBub, _origin.VelocityBub, default, default, _parts);
            return d.ForceBody;
        }
    }

    /// <summary>
    /// Sample the parent body's atmosphere from the game's own functions, and check the samples against the game between them.
    ///  The game's air is layered: PhysicalAtmosphere integrates the hydrostatic equation through a temperature profile, so there is no single scale height to copy across. The column from sea level to PhysicalAtmosphere.Height is sampled with ComputeDensityAtAltitude and ComputePressureAtAltitude on a uniform grid plus every point of the body's temperature profile, where the slope of ln(rho) has its corners. The check then asks the game again halfway between the samples, where the interpolation is worst, and records the largest relative error.
    /// </summary>
    private static void BuildAtmosphere(IParentBody parent, Result res)
    {
        PhysicalAtmosphere physical = parent?.GetPhysicalAtmosphere();
        if (physical == null || !(physical.Height > 0.0))
            return;

        double top = physical.Height;
        var altitudes = new SortedSet<double>();
        for (int i = 0; i <= AtmosphereIntervals; i++)
            altitudes.Add(top * i / AtmosphereIntervals);
        List<AtmosphereTemperaturePoint> profile = parent.GetAtmosphereReference()?.Physical?.TemperatureProfile;
        if (profile != null)
        {
            foreach (AtmosphereTemperaturePoint point in profile)
            {
                double at = point.InMeters();
                if (at > 0.0 && at < top)
                    altitudes.Add(at);
            }
        }

        double[] alt = altitudes.ToArray();
        var density = new double[alt.Length];
        var pressure = new double[alt.Length];
        for (int i = 0; i < alt.Length; i++)
        {
            AtmosphericConditions air = physical.ComputeConditionsAtAltitude(alt[i]);
            density[i] = air.Density;
            pressure[i] = air.Pressure;
        }

        // The game derives the speed of sound from the mixture and the local temperature (PhysicsEnvironment.RecomputePositionalValues). The Mach axis takes its sea level value as one fixed scale, see AtmosphereModel.
        double speedOfSound = physical.Mixture.ComputeSpeedOfSound(physical.ComputeTemperatureAtAltitude(0.0));

        try
        {
            res.Atmosphere = new SampledAtmosphere(alt, density, pressure, top, speedOfSound);
        }
        catch (ArgumentException ex)
        {
            DefaultCategory.Log.Warning($"[AFC] The atmosphere of '{(parent as Astronomical)?.Id}' could not be sampled: {ex.Message}");
            return;
        }
        // IParentBody does not carry a name; Astronomical does, and every body a vehicle can orbit is one.
        res.BodyName = (parent as Astronomical)?.Id ?? "";

        // PhysicsEnvironment.RecomputePositionalValues zeroes the air at and above Height, so the check compares against that effective density, a little past the top too.
        double worst = 0.0;
        for (int i = 0; i + 1 < alt.Length; i++)
            worst = Math.Max(worst, MirrorErrorAt(physical, res.Atmosphere, 0.5 * (alt[i] + alt[i + 1]), top));
        for (int i = 0; i <= 20; i++)
            worst = Math.Max(worst, MirrorErrorAt(physical, res.Atmosphere, top * (1.0 + 0.0025 * i), top));
        res.AtmosphereMirrorError = worst;
    }

    private static double MirrorErrorAt(PhysicalAtmosphere physical, AtmosphereModel model, double altitude, double top)
    {
        double theirs = altitude < top ? physical.ComputeDensityAtAltitude(altitude) : 0.0;
        double ours = model.Density(altitude);
        return Math.Abs(ours - theirs) / Math.Max(Math.Abs(theirs), 1e-12);
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

    /// <summary>AeroMeshSource.Hash of the vehicle's current aero mesh, or 0 while it has none, as during a rebuild.</summary>
    public static ulong LiveMeshHash(Vehicle vehicle) => vehicle?.Parts?.Aerodynamics?.Source?.Hash ?? 0UL;

    /// <summary>True while the game rebuilds the vehicle's aero mesh, when <see cref="TryBuild"/> refuses with <see cref="RebuildingError"/>.</summary>
    public static bool MeshRebuilding(Vehicle vehicle) => vehicle?.Parts?.IsRebuildingAerodynamics ?? false;

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

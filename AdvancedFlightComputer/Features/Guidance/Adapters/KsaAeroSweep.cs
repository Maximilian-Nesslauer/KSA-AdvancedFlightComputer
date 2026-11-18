#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Diagnostics;
using System.Numerics;
using System.Text;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;
using KSA.Aero;
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

    /// <summary>Alpha the centre of pressure is located at, degrees from tail-first: on the alpha grid, small enough to be the balance a glide flies with and large enough for a force across the axis to measure.</summary>
    public const double BalanceAlphaDeg = 5.0;

    /// <summary>Highest Mach the balance verdict is taken over: about the fastest a returning booster glides. The log still lists every row.</summary>
    public const double BalanceMaxMach = 2.0;

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

    /// <summary>
    /// Alpha breakpoints added near zero, degrees, where a booster glides. The default grid goes 0, 2, 5, 8; a booster gliding at 1-2 deg sat between two of them, and the game's lift there is not a straight line - the blunt engine end lifts one way in proportion to sin(alpha), the sides the other way in proportion to sin^2(alpha) - so a spline through 0, 2 and 5 cannot show it.
    /// </summary>
    private static readonly double[] NearZeroAlphaBreakpointsDeg = { 1.0, 3.0, 4.0 };

    private static double[] SweepAlphaBreakpoints()
    {
        var all = new SortedSet<double>(AeroTable.DefaultAlphaBreakpointsDeg);
        foreach (double a in NearZeroAlphaBreakpointsDeg)
            all.Add(a);
        return all.ToArray();
    }

    /// <summary>
    /// Alpha and beta breakpoints of the body-axis table, degrees: every degree out to 5, where a booster glides and lands, then wider to 20, which covers a glide's tilt and the burn tracker's on top of it. Past 20 the fit extends linearly.
    /// </summary>
    private static readonly double[] BodyAngleBreakpointsDeg = { -20, -16, -12, -8, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 8, 12, 16, 20 };

    /// <summary>Highest Mach row of the body-axis table: above a returning booster's fastest, with the boostback's coast in thin air above it read at this row.</summary>
    private const double BodyAeroMaxMach = 8.0;

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

        /// <summary>
        /// The fitted pitching-moment coefficient C_m(Mach, alpha) about the centre of mass - read it through <see cref="MomentAt"/>. Referenced to <see cref="ReferenceArea"/> times <see cref="ReferenceLength"/>, same grid and retrograde-first alpha as <see cref="Table"/>, roll-averaged.
        ///  SIGNED BY WHAT IT DOES TO ALPHA: positive turns the craft further off the airflow (statically unstable flying engine first), negative turns it back (stable). The moment needed to hold an alpha is this, whichever its sign.
        ///  Null where nothing was measured: on an airless body, and from a sweep taken before the moment was recorded.
        /// </summary>
        public AeroTable Moment;
        public double[] MomentTable;

        /// <summary>Signed C_m at a Mach number and retrograde-first alpha (radians), positive away from the airflow. Zero when there is no moment table.</summary>
        public double MomentAt(double mach, double alphaRad) => Moment?.Cd(mach, alphaRad) ?? 0.0;

        /// <summary>Length the moment coefficient is referenced to, m: the bounding box's long extent.</summary>
        public double ReferenceLength;

        /// <summary>
        /// Where the aerodynamic force acts along the long axis, against the centre of mass, per Mach row at <see cref="BalanceAlphaDeg"/>, m. POSITIVE TOWARDS THE NOSE (+x), the trailing end when the craft flies engine first, which is the stable side: the air then turns the engine end back into the wind. NaN where there was no force across the axis to locate.
        /// </summary>
        public double[] CentreOfPressureM;

        /// <summary>The smallest of <see cref="CentreOfPressureM"/>, m - the least stable Mach row - and the Mach it is at. NaN when nothing was measured.</summary>
        public double BalanceWorstM = double.NaN, BalanceWorstMach = double.NaN;

        public bool HasMoment => Moment != null && CentreOfPressureM != null && double.IsFinite(BalanceWorstM);

        /// <summary>
        /// The body-axis force near retrograde, over Mach, alpha and beta, with no averaging over roll (BodyAeroTable): what a craft held at a fixed roll flies with, fins and all. Null on an airless body.
        /// </summary>
        public BodyAeroTable BodyAero;

        /// <summary>
        /// The body-axis MOMENT over the same Mach, alpha and beta: C_l, C_m, C_n = torque / (q S L) about the x, y and z body axes, through the centre of mass the sweep saw (<see cref="BodyMomentComAsmb"/>), with L the <see cref="ReferenceLength"/>. Fins set at an angle, an asymmetric airframe or a centre of mass off the axis all show here as a moment at alpha and beta 0 or one that is not the mirror of its opposite. Read through <see cref="PredictBodyMoment"/>, which moves it to the centre of mass the craft has now. Null on an airless body.
        /// </summary>
        public BodyAeroTable BodyMoment;

        /// <summary>The centre of mass <see cref="BodyMoment"/> is taken about, assembly axes, m.</summary>
        public double3 BodyMomentComAsmb;

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

        /// <summary>The body the sweep was taken over. The atmosphere, and on an airless body the zero tables, belong to it.</summary>
        public IParentBody Parent;

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
        //  Read through Vehicle's own accessor: the same box Props carries, and what the staleness check reads too, so the two cannot disagree about which box the table was built from.
        float3 half = vehicle.BoundingBoxHalfExtentsAsmb;
        return TryBuildFrom(vehicle, vehicle.Props, new double3(half.X * 2.0, half.Y * 2.0, half.Z * 2.0), vehicle.TotalMass,
                            vehicle.Parts, parent, simTime, out result, out error);
    }

    /// <summary>
    /// The sweep of a booster still attached to <paramref name="vehicle"/>: the subtree rooted at <paramref name="root"/>, as the vehicle it becomes when it separates (KsaBoosterGeometry). Its own properties go to the game's force function, so its table is what that vehicle would measure after the split, under whichever aero model is in.
    /// </summary>
    public static bool TryBuildBooster(Vehicle vehicle, Part root, IParentBody parent, double simTime,
                                       out Result result, out string error)
        => TryBuildBooster(vehicle, root, KsaBoosterGeometry.Subtree(root), parent, simTime, out result, out error);

    /// <summary>The same for the parts in <paramref name="parts"/>: the booster's subtree short of what separates before it or beside it (GuidanceWindow.PartsLeavingWith).</summary>
    internal static bool TryBuildBooster(Vehicle vehicle, Part root, System.Collections.Generic.HashSet<Part> parts, IParentBody parent, double simTime,
                                         out Result result, out string error)
    {
        result = null;
        if (!KsaBoosterGeometry.TryProperties(vehicle, root, parts, out VehicleProperties props, out double3 extents, out error))
            return false;
        var clock = Stopwatch.StartNew();
        PartTree aeroParts = null;
        ulong geometryHash = 0;
        try
        {
            if (parent?.GetPhysicalAtmosphere() is { Height: > 0.0 }
                && !KsaBoosterAeroMesh.TryBuild(root, parts, out aeroParts, out geometryHash, out error))
                return false;
            if (!TryBuildFrom(vehicle, props, extents, props.TotalMass, aeroParts, parent, simTime, out result, out error))
                return false;
            if (result.MeshHash != geometryHash)
            {
                error = "the booster sweep did not use its selected aero mesh";
                result = null;
                return false;
            }
            result.SweepMillis = clock.Elapsed.TotalMilliseconds;
            DefaultCategory.Log.Info(FormattableString.Invariant(
                $"[AFC] Booster aero of '{root.DisplayName}': mesh and sweep {result.SweepMillis:F0} ms."));
            return true;
        }
        finally
        {
            aeroParts?.Dispose();
        }
    }

    /// <summary>The sweep of one set of vehicle properties, flown through <paramref name="vehicle"/>'s own physics state: <paramref name="extents"/> is the box those properties were built on, along the assembly axes.</summary>
    private static bool TryBuildFrom(Vehicle vehicle, VehicleProperties props, double3 extents, double mass, PartTree parts, IParentBody parent, double simTime,
                                     out Result result, out string error)
    {
        result = null;
        error = "";
        double dx = extents.X;
        double dy = extents.Y;
        double dz = extents.Z;
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
            Mass = mass,
            SampledAt = simTime,
        };
        BuildAtmosphere(parent, res);

        double[] machGrid = SweepMachBreakpoints();
        double[] alphaDeg = SweepAlphaBreakpoints();
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
            res.MeshHash = parts?.Aerodynamics?.Source?.Hash ?? 0UL;
            res.Parent = parent;
            result = res;
            return true;
        }

        // PartTree.RefreshAerodynamics takes the mesh away when a build starts, and PartTree.AwaitAerodynamics blocks until the build ends. The vehicle's own step waits for the new mesh on its worker, and the sweep refuses instead of stalling the main thread, so the caller keeps its old table until then.
        if (parts != null && parts.IsRebuildingAerodynamics)
        {
            error = RebuildingError;
            return false;
        }
        if (parts != null && parts.HasAerodynamicsBuildFailed)
            LogHelper.WarnOnce("afc-aero-build-failed:" + vehicle.Id,
                $"[AFC] The game could not build the aero mesh of '{vehicle.Id}'. The aero sweep measures the drag the game applies without it.");

        // The air the probe flies through: sea level of this body's own atmosphere. The game reads the Mach number from the speed of sound it is given and the gas properties from the temperature, so both come from the same sea level temperature. The Mach axis is airspeed over the sea level speed of sound, the same scaling the solvers use.
        double seaTemperature = physical.ComputeTemperatureAtAltitude(0.0);
        AtmosphericConditions sea = physical.ComputeConditionsAtAltitude(0.0);
        double speedOfSound = physical.Mixture.ComputeSpeedOfSound(seaTemperature);
        if (!ForceProbe.TryCreate(vehicle, props, parts, parent, sea.Density, sea.Pressure, seaTemperature, speedOfSound, out ForceProbe probe, out error))
            return false;
        using var flow = new SamplingFlow(vehicle, parts);

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
        // THE MOMENT, from the same calls: ComputeDerivatives returns the torque about the centre of mass beside the force. Resolved about the axis that turns the engine end further off the airflow, so positive is destabilising, and beside it the moment the force across the axis would make at one metre, which locates the centre of pressure.
        var momentSums = new double[nm * na];
        var armSums = new double[nm * na];
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
                // The axis a positive rotation about which moves the engine end (-x) towards its tilt, further off the airflow: (-x) x tilt.
                double3 offAxis = sa > 1e-9 ? double3.Cross(new double3(-1.0, 0.0, 0.0), tilt).NormalizeOrZero() : double3.Zero;
                probe.Wrench(BaselineSpeed * u, out double3 baseline, out double3 baselineTorque);
                flow.Prepare(u);
                for (int i = 0; i < nm; i++)
                {
                    probe.Wrench(speeds[i] * u, out double3 rawForce, out double3 rawTorque);
                    double3 force = rawForce - baseline;
                    double3 torque = rawTorque - baselineTorque;
                    double drag = -double3.Dot(force, u);
                    double3 across = force + drag * u;
                    int cell = i * na + j;
                    dragSums[cell] += drag;
                    liftSums[cell] += across.Length();
                    signedLiftSums[cell] += double3.Dot(across, tilt);
                    momentSums[cell] += double3.Dot(torque, offAxis);
                    // The same force across the long axis, applied one metre along +x: x x F_normal, about the same axis.
                    var normal = new double3(0.0, force.Y, force.Z);
                    armSums[cell] += double3.Dot(double3.Cross(new double3(1.0, 0.0, 0.0), normal), offAxis);
                    if (i == 0)
                    {
                        lowRowLo[j] = Math.Min(lowRowLo[j], drag);
                        lowRowHi[j] = Math.Max(lowRowHi[j], drag);
                    }
                }
            }
        }
        // THE BODY-AXIS TABLE near retrograde (BodyAeroTable): for each alpha and beta, the air direction in body axes, and per Mach row the force the game applies, kept whole - axial, and across the body in y and z - with no averaging over roll. The same calls as above, in the same order: the baseline at a crawl, then the flow prepared for the direction, then the rows.
        double[] bodyMach = machGrid.Where(m => m <= BodyAeroMaxMach + 1e-9).ToArray();
        int nbm = bodyMach.Length, nab = BodyAngleBreakpointsDeg.Length;
        var bodyCoefficients = new double[nbm * nab * nab * 3];
        var bodyMoments = new double[nbm * nab * nab * 3];
        double momentLength = dx;
        for (int ja = 0; ja < nab; ja++)
        {
            for (int jb = 0; jb < nab; jb++)
            {
                (double ux, double uy, double uz) = BodyAeroTable.AirDirection(BodyAngleBreakpointsDeg[ja] * Math.PI / 180.0, BodyAngleBreakpointsDeg[jb] * Math.PI / 180.0);
                var u = new double3(ux, uy, uz);
                probe.Wrench(BaselineSpeed * u, out double3 baseline, out double3 baselineTorque);
                flow.Prepare(u);
                for (int i = 0; i < nbm; i++)
                {
                    double speed = Math.Max(bodyMach[i], MinProbeMach) * speedOfSound;
                    probe.Wrench(speed * u, out double3 rawForce, out double3 rawTorque);
                    double3 force = rawForce - baseline;
                    double3 torque = rawTorque - baselineTorque;
                    double qa = 0.5 * sea.Density * speed * speed * refArea;
                    int node = ((i * nab + ja) * nab + jb) * 3;
                    bodyCoefficients[node] = force.X / qa;
                    bodyCoefficients[node + 1] = force.Y / qa;
                    bodyCoefficients[node + 2] = force.Z / qa;
                    if (momentLength > 0.0)
                    {
                        // A torque the game did not report reads as none, as in the roll-averaged table.
                        bodyMoments[node] = double.IsFinite(torque.X) ? torque.X / (qa * momentLength) : 0.0;
                        bodyMoments[node + 1] = double.IsFinite(torque.Y) ? torque.Y / (qa * momentLength) : 0.0;
                        bodyMoments[node + 2] = double.IsFinite(torque.Z) ? torque.Z / (qa * momentLength) : 0.0;
                    }
                }
            }
        }
        res.SweepMillis = clock.Elapsed.TotalMilliseconds;
        // Read after the probe, because its first PartTree.AwaitAerodynamics publishes a mesh whose build has ended but was not taken yet.
        res.MeshHash = parts?.Aerodynamics?.Source?.Hash ?? 0UL;
        res.Parent = parent;
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

        // The moment coefficient, and the centre of pressure at the balance alpha per Mach row. A torque the game did not report (an aero mod that applies forces without one) leaves the coefficients at zero and every centre of pressure at the centre of mass, which reads as neutral rather than as stable.
        double refLength = dx;
        var cmTable = new double[nm * na];
        var centreOfPressure = new double[nm];
        int balanceColumn = Array.IndexOf(alphaDeg, BalanceAlphaDeg);
        double worstCp = double.NaN, worstMach = double.NaN;
        for (int i = 0; i < nm; i++)
        {
            double qal = 0.5 * sea.Density * speeds[i] * speeds[i] * refArea * refLength;
            for (int j = 0; j < na; j++)
            {
                int cell = i * na + j;
                double cm = momentSums[cell] / RollSamples / qal;
                cmTable[cell] = double.IsFinite(cm) ? cm : 0.0;
            }
            centreOfPressure[i] = double.NaN;
            if (balanceColumn >= 0)
            {
                int cell = i * na + balanceColumn;
                if (Math.Abs(armSums[cell]) > 1e-9 * Math.Max(dragSums[cell], 1e-9))
                    centreOfPressure[i] = momentSums[cell] / armSums[cell];
            }
            // The verdict is taken over the Mach range a returning booster flies: the hypersonic rows are there for entries from high, and a far-forward centre of pressure at Mach 25 says nothing about the glide.
            if (machGrid[i] <= BalanceMaxMach && double.IsFinite(centreOfPressure[i]) && !(centreOfPressure[i] >= worstCp))
            {
                worstCp = centreOfPressure[i];
                worstMach = machGrid[i];
            }
        }
        res.ReferenceLength = refLength;
        res.MomentTable = cmTable;
        res.CentreOfPressureM = centreOfPressure;
        res.BalanceWorstM = worstCp;
        res.BalanceWorstMach = worstMach;

        try
        {
            res.Table = new AeroTable(machGrid, alphaDeg, cdTable);
            res.Lift = new AeroTable(machGrid, alphaDeg, clTable);
            res.Moment = refLength > 0.0 ? new AeroTable(machGrid, alphaDeg, cmTable) : null;
            res.BodyAero = new BodyAeroTable(bodyMach, BodyAngleBreakpointsDeg, BodyAngleBreakpointsDeg, bodyCoefficients);
            res.BodyMoment = momentLength > 0.0 ? new BodyAeroTable(bodyMach, BodyAngleBreakpointsDeg, BodyAngleBreakpointsDeg, bodyMoments) : null;
            res.BodyMomentComAsmb = new double3(props.CenterOfMassAsmb.X, props.CenterOfMassAsmb.Y, props.CenterOfMassAsmb.Z);
        }
        catch (Exception ex)
        {
            error = "table fit failed: " + ex.Message;
            return false;
        }

        // The measured lift near alpha 0, the angles a booster glides at, so the shape of the curve there can be read off the game log.
        DefaultCategory.Log.Info(FormattableString.Invariant($"[AFC] Lift near alpha 0 of '{vehicle.Id}' (C_L, roll-averaged, + towards the engine end's tilt): {NearZeroLiftText(res)}."));
        if (res.BodyAero != null)
            DefaultCategory.Log.Info(FormattableString.Invariant($"[AFC] Body-axis force of '{vehicle.Id}' near retrograde at Mach 0.4 (C_x, C_y, C_z): {BodyAeroText(res.BodyAero, 0.4)}."));
        if (res.BodyMoment != null)
            DefaultCategory.Log.Info(FormattableString.Invariant($"[AFC] Body-axis moment of '{vehicle.Id}' near retrograde at Mach 0.4 (C_l, C_m, C_n about the centre of mass, L {res.ReferenceLength:F1} m): {BodyAeroText(res.BodyMoment, 0.4)}."));
        // Logged once the moment table exists, which BalanceText reads.
        if (double.IsFinite(worstCp))
            DefaultCategory.Log.Info(FormattableString.Invariant(
                $"[AFC] Aero balance of '{vehicle.Id}': {BalanceText(res)}; centre of pressure at {BalanceAlphaDeg:0} deg by Mach: {CentreOfPressureText(res)}."));
        result = res;
        return true;
    }

    private sealed class SamplingFlow : IDisposable
    {
        private readonly PartTree _parts;
        private readonly Aerodynamics _product;
        private readonly AeroSurfaceFlow _flow;
        private readonly Vector3? _restoreWind;

        public SamplingFlow(Vehicle vehicle, PartTree parts)
        {
            _parts = parts;
            _product = parts?.AwaitAerodynamics();
            _flow = _product?.GetSurfaceFlow();
            if (!ReferenceEquals(parts, vehicle.Parts))
                return;
            BubbleOrigin origin = vehicle.BubbleOrigin;
            KinematicStates kinematic = vehicle.KinematicStates;
            PhysicsEnvironment environment = vehicle.PhysicsEnvironment;
            float3 air = PhysicsStates.ComputeAirVelocityBody(in origin, in kinematic, in environment);
            var wind = new Vector3(-air.X, -air.Y, -air.Z);
            float speedSquared = wind.LengthSquared();
            if (speedSquared > 1e-6f && float.IsFinite(speedSquared))
                _restoreWind = Vector3.Normalize(wind);
        }

        public void Prepare(double3 airDirection)
        {
            // Stock reuses geometry within one degree. An opposite wind forces fresh geometry for the next sample.
            _flow?.Update(airDirection.ToBepu());
        }

        public void Dispose()
        {
            if (!_restoreWind.HasValue || _flow == null || _parts.IsRebuildingAerodynamics
                || !ReferenceEquals(_parts.Aerodynamics, _product))
                return;
            Vector3 wind = _restoreWind.Value;
            _flow.Update(-wind);
            _flow.Update(wind);
        }
    }

    /// <summary>
    /// A copy of one vehicle's physics state, with the air set per probe. The force read back is the game's own, from the same ComputeDerivatives the integrator calls, so any mod that patches the forces there is measured with them.
    /// The tree supplies the aero mesh, either the live craft or a private booster mesh holder. A live tree's AeroSurfaceFlow shares buffers with the physics worker, so the probe runs at the Vehicle.PrepareWorker boundary.
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

        public static bool TryCreate(Vehicle vehicle, VehicleProperties props, PartTree parts, IParentBody parent, double density, double pressure, double temperature, double speedOfSound, out ForceProbe probe, out string error)
        {
            probe = new ForceProbe
            {
                _origin = vehicle.BubbleOrigin,
                _kinematic = vehicle.KinematicStates,
                _props = props,
                _environment = vehicle.PhysicsEnvironment,
                _parts = parts,
            };

            // Air, no water, no spin: a probe in free air at this body's sea level. Anything this leaves set, like gravity, lands in AccelPhys or in the buoyancy the baseline removes.
            probe._environment.InPhysicsRadius = true;
            probe._environment.ClosestParent = parent;
            probe._environment.AtmosphericDensity = (float)density;
            probe._environment.AtmosphericPressure = (float)pressure;
            probe._environment.AtmosphericTemperature = (float)temperature;
            probe._environment.SpeedOfSound = (float)speedOfSound;
            probe._environment.OceanVolume = 0.0;
            probe._environment.OceanSurfaceArea = 0.0;
            var state = new PhysicsStates
            {
                Origin = ref probe._origin,
                Kinematic = ref probe._kinematic,
                Props = ref probe._props,
                Environment = ref probe._environment,
            };
            state.SetBodyRates(double3.Zero);

            // The game's own definition of airspeed, asked at zero physics velocity, gives the frame's share; every probe velocity is set relative to it.
            probe._kinematic.VelocityPhys = double3.Zero;
            probe._frameAirPhys = PhysicsStates.ComputeSurfaceVelocityBub(in probe._origin, in probe._kinematic);

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
            Wrench(airVelocityBody, out double3 force, out _);
            return force;
        }

        /// <summary>The body-axis force and the torque about the centre of mass the game applies at this air velocity (body axes), from one call.</summary>
        public void Wrench(double3 airVelocityBody, out double3 force, out double3 torque)
        {
            _kinematic.VelocityPhys = airVelocityBody.Transform(_kinematic.Body2Phys) - _frameAirPhys;

            // dt = 0 turns off the semi-implicit limiter stock applies to its drag, which is a property of the integrator rather than of the air. No thrusters, no chutes.
            Disturbances d = PhysicsStates.ComputeDerivatives(in _origin, in _kinematic, in _props, in _environment,
                0.0, 0.0, _origin.PositionBub, _origin.VelocityBub, default, default, _parts);
            force = d.ForceBody;
            torque = d.TorqueBody;
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

    /// <summary>Below this distance between the centre of pressure and the centre of mass, m, the craft reads as neutral rather than stable or unstable.</summary>
    public const double NeutralBalanceM = 0.05;

    /// <summary>
    /// The craft's static balance flying engine first, at its least stable Mach row: "STABLE ENGINE FIRST, CENTRE OF PRESSURE 2.1 m TOWARDS THE NOSE" and the like. Empty when the sweep measured no moment.
    /// </summary>
    public static string BalanceText(Result r)
    {
        if (r == null || !r.HasMoment)
            return "";
        double cp = r.BalanceWorstM;
        string where = FormattableString.Invariant($"CENTRE OF PRESSURE {Math.Abs(cp):F1} m TOWARDS THE {(cp >= 0.0 ? "NOSE" : "ENGINE")} FROM THE CoM, AT MACH {r.BalanceWorstMach:0.##}");
        return Math.Abs(cp) < NeutralBalanceM ? "NEUTRAL ENGINE FIRST, " + where
            : cp > 0.0 ? "STABLE ENGINE FIRST, " + where
            : "UNSTABLE ENGINE FIRST, " + where;
    }

    /// <summary>The body table at a Mach number, at alpha and beta of 0, +-2 and +-5 deg one at a time, for the game log.</summary>
    private static string BodyAeroText(BodyAeroTable table, double mach)
    {
        var sb = new StringBuilder();
        (double a, double b)[] points = { (0, 0), (2, 0), (-2, 0), (5, 0), (-5, 0), (0, 2), (0, -2), (0, 5), (0, -5) };
        foreach ((double a, double b) in points)
        {
            table.Coefficients(mach, a * Math.PI / 180.0, b * Math.PI / 180.0, out double cx, out double cy, out double cz);
            if (sb.Length > 0)
                sb.Append("; ");
            sb.Append(FormattableString.Invariant($"a{a:+0;-0;0} b{b:+0;-0;0}: {cx:+0.000;-0.000} {cy:+0.0000;-0.0000} {cz:+0.0000;-0.0000}"));
        }
        return sb.ToString();
    }

    /// <summary>Mach numbers the near-zero lift is logged at: the nearest row to each.</summary>
    private static readonly double[] LiftLogMach = { 0.0, 0.4, 0.8, 1.2, 2.0, 4.0 };

    /// <summary>Measured C_L up to 8 deg at a few Mach rows, "M0.4: 0 +0.0000, 1 +0.0012, ..." for the game log.</summary>
    private static string NearZeroLiftText(Result r)
    {
        var sb = new StringBuilder();
        int last = -1;
        foreach (double target in LiftLogMach)
        {
            int row = 0;
            for (int i = 1; i < r.MachCount; i++)
                if (Math.Abs(r.MachGrid[i] - target) < Math.Abs(r.MachGrid[row] - target))
                    row = i;
            if (row == last)
                continue;
            last = row;
            if (sb.Length > 0)
                sb.Append("; ");
            sb.Append(FormattableString.Invariant($"M{r.MachGrid[row]:0.##}:"));
            for (int j = 0; j < r.AlphaCount && r.AlphaGridDeg[j] <= 8.0 + 1e-9; j++)
                sb.Append(FormattableString.Invariant($" {r.AlphaGridDeg[j]:0} {r.LiftTable[row * r.AlphaCount + j]:+0.0000;-0.0000}"));
        }
        return sb.ToString();
    }

    /// <summary>The centre of pressure per Mach row, "M0.05 +2.1, M1 +1.8, ...", for the game log.</summary>
    private static string CentreOfPressureText(Result r)
    {
        var sb = new StringBuilder();
        for (int i = 0; i < r.MachCount; i++)
        {
            if (!double.IsFinite(r.CentreOfPressureM[i]))
                continue;
            if (sb.Length > 0)
                sb.Append(", ");
            sb.Append(FormattableString.Invariant($"M{r.MachGrid[i]:0.##} {r.CentreOfPressureM[i]:+0.00;-0.00} m"));
        }
        return sb.ToString();
    }

    /// <summary>
    /// The aerodynamic force and moment the game applies to the vehicle as it is NOW: its own state, air, roll and body rates, through the same ComputeDerivatives the integrator calls, less the same call at a crawl in the same direction, which takes out buoyancy. Body axes, N and N m about the centre of mass.
    ///  What the tables cannot be: they average over roll, sample at sea-level air and a still body, and leave out any force the craft makes at zero angle of attack. Against this the guidance can tell a model error from a force that is not aerodynamic at all.
    ///  Main thread only, as the sweep: guidance calls it from the Vehicle.PrepareWorker prefix, where no worker steps the vehicle. The flow direction is the live one, so the mesh's shielding is what the game's own step left. False while the aero mesh rebuilds.
    /// </summary>
    public static bool TryLiveAero(Vehicle vehicle, out double3 forceBody, out double3 torqueBody)
    {
        forceBody = torqueBody = double3.Zero;
        PartTree parts = vehicle?.Parts;
        if (parts == null || parts.IsRebuildingAerodynamics)
            return false;
        BubbleOrigin origin = vehicle.BubbleOrigin;
        KinematicStates live = vehicle.KinematicStates;
        VehicleProperties props = vehicle.Props;
        PhysicsEnvironment environment = vehicle.PhysicsEnvironment;
        Disturbances d = PhysicsStates.ComputeDerivatives(in origin, in live, in props, in environment,
            0.0, 0.0, origin.PositionBub, origin.VelocityBub, default, default, parts);

        // The crawl: the airspeed scaled down to BaselineSpeed along the same direction, the body rates kept.
        float3 airBody = PhysicsStates.ComputeAirVelocityBody(in origin, in live, in environment);
        double3 air = double3.Unpack(in airBody);
        double airspeed = air.Length();
        if (!(airspeed > BaselineSpeed))
            return false;
        KinematicStates crawl = live;
        crawl.VelocityPhys = live.VelocityPhys - (air * (1.0 - BaselineSpeed / airspeed)).Transform(live.Body2Phys);
        Disturbances d0 = PhysicsStates.ComputeDerivatives(in origin, in crawl, in props, in environment,
            0.0, 0.0, origin.PositionBub, origin.VelocityBub, default, default, parts);
        forceBody = d.ForceBody - d0.ForceBody;
        torqueBody = d.TorqueBody - d0.TorqueBody;
        return double.IsFinite(forceBody.X) && double.IsFinite(forceBody.Y) && double.IsFinite(forceBody.Z);
    }

    /// <summary>
    /// The body-axis aerodynamic moment the tables predict, N m about the centre of mass the craft has now (assembly axes, as the torque the game reports), for an air velocity in body axes and the air's density: the moment table at that Mach, alpha and beta, moved from the sweep's centre of mass by M_now = M_sweep + (r_sweep - r_now) x F, with F the force table's. False outside the tables' 30 deg cone round engine first, or without a moment table.
    ///  The assembly and body axes are taken as one rotation, as the probe that measured both does.
    /// </summary>
    public static bool PredictBodyMoment(Result res, double3 airBody, double density, double3 comAsmbNow, out double3 moment)
    {
        moment = default;
        if (res?.BodyMoment == null || res.BodyAero == null || res.Atmosphere == null)
            return false;
        double speed = airBody.Length();
        if (!(speed > 1.0))
            return false;
        double3 u = airBody / speed;
        if (!(-u.X > Math.Cos(30.0 * Math.PI / 180.0)))
            return false;
        double alpha = Math.Atan2(u.Z, -u.X), beta = Math.Atan2(u.Y, -u.X);
        double mach = res.Atmosphere.Mach(speed);
        double machF = Math.Clamp(mach, res.BodyAero.MachMin, res.BodyAero.MachMax);
        double machM = Math.Clamp(mach, res.BodyMoment.MachMin, res.BodyMoment.MachMax);
        double qs = 0.5 * density * speed * speed * res.ReferenceArea;
        res.BodyAero.Coefficients(machF, alpha, beta, out double cx, out double cy, out double cz);
        res.BodyMoment.Coefficients(machM, alpha, beta, out double cl, out double cm, out double cn);
        var force = new double3(cx, cy, cz) * qs;
        double3 sweepMoment = new double3(cl, cm, cn) * (qs * res.ReferenceLength);
        moment = sweepMoment + double3.Cross(res.BodyMomentComAsmb - comAsmbNow, force);
        return double.IsFinite(moment.X) && double.IsFinite(moment.Y) && double.IsFinite(moment.Z);
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

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
using AdvancedFlightComputer.Features.AeroControl;
using AdvancedFlightComputer.Guidance.Numerics.Flight;

/// <summary>
/// Measures the aerodynamic force the game applies to the current vehicle onto a Cd(Mach, alpha) grid, and pairs it with an atmosphere that mirrors the game's.
///  THIS IS A MEASUREMENT, NOT A MODEL. Every Cd below is read off PhysicsStates.ComputeDerivatives - the force function the integrator itself calls every substep - run on a copy of the live vehicle's state, with its part tree, and with the airspeed set per sample. Whatever aero model is active is what gets measured: the game's pressure and skin friction over the craft's aero mesh, or a mod that replaces it, with no knowledge here of which. A KSA update or an aero mod that changes the forces changes these numbers with it.
///  The force along the airspeed goes into the drag table, because that is all the solvers model. The force across it is measured too: its size is reported in <see cref="Result.LiftToDragMax"/> so it is visible rather than silently dropped, and its signed component goes into <see cref="Result.Lift"/>, which the boostback glide steers with.
/// </summary>
public static partial class KsaAeroSweep
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

    /// <summary>The largest alpha and beta the body table and the surface tables reach, degrees.</summary>
    internal static double BodyAngleMaxDeg => BodyAngleBreakpointsDeg[^1];

    /// <summary>
    /// Alpha and beta breakpoints of the NOSE-FIRST body-axis table (Result.NoseBodyAero, #216), degrees about the nose: the convex ascent's. Every degree to 2 and wider to 20: the ascent stays within a few degrees of the airflow where q is large (the q-alpha limit) and pitches further off it only as the air thins - to 20 deg near 1 kPa on 2stage_new. Coarser than the retrograde table's, so the extra sweep stays about half of it.
    /// </summary>
    private static readonly double[] NoseAngleBreakpointsDeg = { -20, -15, -10, -6, -4, -2, -1, 0, 1, 2, 4, 6, 10, 15, 20 };

    /// <summary>The largest alpha and beta the nose-first tables reach, degrees.</summary>
    internal static double NoseAngleMaxDeg => NoseAngleBreakpointsDeg[^1];

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

        /// <summary>The roll-averaged normal force's moment arm per metre, per Mach row and alpha, as MomentTable lays them out: when the centre of mass moves s metres towards the nose, MomentTable's coefficient changes by -s times this over ReferenceLength (KsaAeroSweepCache moves it). Null in a sweep without moments.</summary>
        public double[] ArmTable;

        /// <summary>A copy sharing every table, for a view of the same sweep with some of them replaced (KsaAeroSweepCache).</summary>
        internal Result ShallowCopy() => (Result)MemberwiseClone();

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

        /// <summary>
        /// The body-axis force and moment NOSE FIRST (BodyAeroTable.NoseFirst), over Mach and the nose-first alpha and beta (NoseAngleBreakpointsDeg), in the same units and about the same centre of mass as <see cref="BodyAero"/> and <see cref="BodyMoment"/>: what an ascent flies with (#216), lift and side force included, and the moment AFC's attitude controller holds the ascent against. Swept only when asked for (TryStart's noseFirst), null otherwise and on an airless body.
        /// </summary>
        public BodyAeroTable NoseBodyAero;
        public BodyAeroTable NoseBodyMoment;

        /// <summary>
        /// The belly-first sweep for an orbital entry, null unless asked for (TryStart's bellyFirst): for each candidate belly axis, drag, lift and the pitch moment over Mach and the angle of attack from nose-first (0) through belly-first (90) to tail-first (180), at no sideslip. See <see cref="BellySweep"/>.
        /// </summary>
        public BellySweep[] Belly;

        /// <summary>
        /// The body-axis force and moment BELLY FIRST (BodyAeroFrame.Belly) on the side <see cref="BellySide"/> names, over Mach and the belly-first alpha and beta (BellyAlphaTableDeg, BellyBetaTableDeg), in the same units and about the same centre of mass as <see cref="BodyAero"/>: what an orbital entry flies with, and the moment AFC's attitude controller holds it against. The fins' and flaps' own belly-first tables are in SurfaceTable.BellyForce. Swept only when asked for (TryStart's bellySide), null otherwise.
        /// </summary>
        public BodyAeroTable BellyBodyAero;
        public BodyAeroTable BellyBodyMoment;

        /// <summary>Which side the belly-first tables lead with: 0 -Z, 1 +Z, 2 -Y, 3 +Y (BellyAxes), -1 for none.</summary>
        public int BellySide = -1;

        /// <summary>
        /// The body-axis force and moment over EVERY air direction (BodyAeroTable.Sphere), in the same units and about the same centre of mass as <see cref="BodyAero"/>: what touchdown guidance flies a belly-flop's flip with, from belly first round to engine first. Its frame leads engine first with the longitude towards the belly (SphereFrame), so the flip runs along its equator, 15 deg a node (SphereLambdaDeg, SphereMuDeg). The control surfaces' own are coarser (SurfaceTable.SphereForce). Swept with the belly-first tables (TryStart's bellySide), null otherwise.
        /// </summary>
        public BodyAeroTable SphereBodyAero;
        public BodyAeroTable SphereBodyMoment;

        /// <summary>The same on the flip band's finer grid (BandLambdaDeg, BandMuDeg; the whole sphere's frame): what touchdown guidance flies a belly-flop's flip with between its engine-first and belly-first tables. Swept with them, null otherwise.</summary>
        public BodyAeroTable BandBodyAero;
        public BodyAeroTable BandBodyMoment;

        /// <summary>The body-axis force and moment tables of a set (TableSet).</summary>
        public BodyAeroTable BodyForceFor(TableSet set) => set switch { TableSet.Nose => NoseBodyAero, TableSet.Belly => BellyBodyAero, TableSet.Sphere => SphereBodyAero, _ => BodyAero };
        public BodyAeroTable BodyMomentFor(TableSet set) => set switch { TableSet.Nose => NoseBodyMoment, TableSet.Belly => BellyBodyMoment, TableSet.Sphere => SphereBodyMoment, _ => BodyMoment };

        /// <summary>
        /// Each control surface's effect (ControlSurfaces), swept like the body table and on its grid: the change its deflection makes to the body-axis force and moment, per deflection swept. Empty for a craft with none, or one swept on an airless body.
        /// </summary>
        public SurfaceTable[] Surfaces = [];

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
    /// Measure the vehicle's aerodynamics and fit the surrogate, on this thread: what <see cref="TryStart"/> prepares, run inline. For callers that need the table before going on (the harness); flight starts a <see cref="Job"/> instead, so the second or so of samples never holds the main thread.
    ///  Main thread only, as TryStart.
    /// </summary>
    /// <returns>False with a reason in <paramref name="error"/> if the vehicle has no
    /// usable geometry, the game's force function gives no drag, or the game rebuilds the aero mesh (<see cref="RebuildingError"/>). A body with no atmosphere is NOT an error: the game applies no aerodynamic force there, so nothing is measured, Atmosphere comes back null and the tables are zero.</returns>
    public static bool TryBuild(Vehicle vehicle, IParentBody parent, double simTime,
                                out Result result, out string error, bool noseFirst = false, bool bellyFirst = false)
    {
        result = null;
        error = "";

        if (vehicle == null)
        {
            error = "no vehicle";
            return false;
        }

        if (!TryPrepareCraft(vehicle, parent, simTime, out Prepared prepared, out error))
            return false;
        prepared.NoseFirst = noseFirst;
        prepared.BellyFirst = bellyFirst;
        return Run(prepared, Log, out result, out error);
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
        return TryPrepareBooster(vehicle, root, parts, parent, simTime, out Prepared prepared, out error)
            && Run(prepared, Log, out result, out error);
    }

    private static void Log(string line) => DefaultCategory.Log.Info(line);

    /// <summary>The log lines of sweeps on their own threads, written by the main thread (WriteLog): a surface pass ends after its result was taken.</summary>
    private static readonly System.Collections.Concurrent.ConcurrentQueue<string> PendingLog = new();

    /// <summary>Writes what sweeps on their own threads have logged. Main thread; Job.TryTake calls it, and so does the craft's aero refresh each step, for a surface pass that ends later.</summary>
    internal static void WriteLog()
    {
        while (PendingLog.TryDequeue(out string line))
            Log(line);
    }

    /// <summary>The deflection step a control surface is swept at, degrees (SurfaceDeflections).</summary>
    public const double SurfaceStepDeg = 15.0;

    /// <summary>
    /// One control surface's effect on the body-axis tables (BodyAeroTable), over the same Mach, alpha and beta: at each deflection swept, the change it makes to the force coefficients (per q S) and to the moment coefficients (per q S L, about BodyMomentComAsmb). Zero at rest. A deflection between two swept is read linearly between them.
    ///  WHY SWEPT WITH THE BODY ANGLES. A surface on stock aero is panels of the craft's mesh like any other, shielded by the body and the other surfaces as the flow turns, so what it does depends on alpha and beta, not only on its own angle - it cannot be a fixed increment on top of the body table. Each surface is swept alone and the effects added in flight (AeroTorqueAllocator); the interference between two deflected surfaces is left out.
    /// </summary>
    public sealed class SurfaceTable
    {
        public Part Part;
        public string Name = "";
        public double MinDeg, MaxDeg;
        /// <summary>The deflections swept, degrees, ascending, with 0 among them.</summary>
        public double[] DeflectionDeg = [];
        public BodyAeroTable[] Force = [];
        public BodyAeroTable[] Moment = [];
        /// <summary>The same nose first (Result.NoseBodyAero's grid), per deflection; empty where the sweep had no nose-first pass.</summary>
        public BodyAeroTable[] NoseForce = [];
        public BodyAeroTable[] NoseMoment = [];
        /// <summary>The same belly first (Result.BellyBodyAero's grid and side), per deflection; empty where the sweep had no belly-first pass.</summary>
        public BodyAeroTable[] BellyForce = [];
        public BodyAeroTable[] BellyMoment = [];
        /// <summary>The same over every air direction (Result.SphereBodyAero's frame), on the coarser FinSphereLambdaDeg and FinSphereMuDeg grid, per deflection; empty where the sweep had no whole-sphere pass.</summary>
        public BodyAeroTable[] SphereForce = [];
        public BodyAeroTable[] SphereMoment = [];
        /// <summary>The same on the flip band (Result.BandBodyAero's grid), per deflection; empty where the sweep had no band.</summary>
        public BodyAeroTable[] BandForce = [];
        public BodyAeroTable[] BandMoment = [];
        public double SweepMillis;

        /// <summary>The surface's part as the sweep found it - template and place on the craft (KsaAeroSweepCache.PartKey) - so a reloaded craft's new part takes its table.</summary>
        public string PartKey = "";

        internal SurfaceTable ShallowCopy() => (SurfaceTable)MemberwiseClone();

        /// <summary>The per-deflection force tables of a set (TableSet).</summary>
        public BodyAeroTable[] ForceTables(TableSet set) => set switch { TableSet.Nose => NoseForce, TableSet.Belly => BellyForce, TableSet.Sphere => SphereForce, _ => Force };
        public BodyAeroTable[] MomentTables(TableSet set) => set switch { TableSet.Nose => NoseMoment, TableSet.Belly => BellyMoment, TableSet.Sphere => SphereMoment, _ => Moment };
    }

    /// <summary>The Mach numbers the control surfaces' tables are swept at: the body's rows nearest these, through the transonic dip and out to its top. Own choice: ten rows against the body's 27, a third of the time; a surface's effect is read linearly between them by its table's own fit.</summary>
    private static readonly double[] SurfaceMachTargets = { 0.0, 0.6, 0.8, 0.95, 1.05, 1.2, 1.6, 2.5, 4.0, 8.0 };

    internal static double[] SurfaceMachRows(double[] bodyMach)
    {
        var rows = new SortedSet<double>();
        foreach (double target in SurfaceMachTargets)
        {
            double best = bodyMach[0];
            foreach (double m in bodyMach)
                if (Math.Abs(m - target) < Math.Abs(best - target))
                    best = m;
            rows.Add(best);
        }
        return rows.ToArray();
    }

    /// <summary>The deflections a surface is swept at: every SurfaceStepDeg out from rest inside its range, its two ends, and rest.</summary>
    internal static double[] SurfaceDeflections(double minDeg, double maxDeg)
    {
        var at = new SortedSet<double> { 0.0 };
        if (minDeg < 0.0) at.Add(minDeg);
        if (maxDeg > 0.0) at.Add(maxDeg);
        for (double d = SurfaceStepDeg; d < maxDeg - 1e-6; d += SurfaceStepDeg)
            at.Add(d);
        for (double d = -SurfaceStepDeg; d > minDeg + 1e-6; d -= SurfaceStepDeg)
            at.Add(d);
        return at.ToArray();
    }

    // ------------------------------------------------------------------ off the main thread

    /// <summary>
    /// Everything one sweep reads of the game, taken on the main thread (TryPrepareCraft, TryPrepareBooster): plain copies of the physics state and the vehicle properties - all value types - and a private aero mesh source gathered from the parts. Run reads nothing else, so it can run on any thread.
    /// </summary>
    /// <summary>
    /// One belly axis's belly-first sweep (Result.Belly), as flat arrays indexed [mach * alphas + alpha]. Cd and Cl on <see cref="ReferenceArea"/>; Cm on area times <see cref="ReferenceLength"/>, about the centre of mass at the sweep, positive when it raises the angle of attack.
    /// </summary>
    public sealed class BellySweep
    {
        public double3 Axis;
        public double[] MachGrid;
        public double[] AlphaDeg;
        public double[] Cd, Cl, Cm;
        public double ReferenceArea, ReferenceLength;

        public string AxisName => Axis.Z < -0.5 ? "-Z" : Axis.Z > 0.5 ? "+Z" : Axis.Y < -0.5 ? "-Y" : "+Y";
    }

    /// <summary>The body-axis tables a reader can fly with: engine first (BodyAero), nose first (NoseBodyAero), belly first (BellyBodyAero), or every air direction (SphereBodyAero, its angles the sphere's longitude and latitude, BodyAeroTable.SphereAngles).</summary>
    public enum TableSet { Tail, Nose, Belly, Sphere }

    /// <summary>
    /// THE ONE RULE for which body-axis tables describe the air as it is now: belly first where the sweep has those tables and the air is inside them, else nose first likewise, else engine first likewise, else the whole sphere where the sweep has it (a belly-flop's: every air direction, coarser). Without the sphere a Starship banked past the belly-first tables' 45 deg had no air moment and no flaps, and tumbled (2026-11-24, twice). With <paramref name="surfacesToo"/>, a set counts only where every swept surface has it too. False where the air is inside none. <paramref name="u"/> is the craft's velocity through the air in body axes, unit; the angles come back about the chosen set's frame, radians.
    /// </summary>
    public static bool TryPickTables(Result aero, double3 u, bool surfacesToo, out TableSet set, out double alphaRad, out double betaRad)
    {
        set = TableSet.Tail;
        alphaRad = betaRad = 0.0;
        if (aero == null)
            return false;
        bool SurfacesHave(TableSet s)
        {
            if (!surfacesToo)
                return true;
            if (aero.Surfaces == null || aero.Surfaces.Length == 0)
                return false;
            foreach (SurfaceTable t in aero.Surfaces)
                if (t.ForceTables(s).Length != t.DeflectionDeg.Length || t.MomentTables(s).Length != t.DeflectionDeg.Length)
                    return false;
            return true;
        }
        if (aero.BellyBodyAero != null && aero.BellyBodyMoment != null && SurfacesHave(TableSet.Belly))
        {
            BodyAeroFrame f = aero.BellyBodyAero.Frame;
            if (f.LeadOf(u.X, u.Y, u.Z) > 0.0)
            {
                BodyAeroTable.Angles(in f, u.X, u.Y, u.Z, out double a, out double b);
                double aDeg = a * 180.0 / Math.PI, bDeg = b * 180.0 / Math.PI;
                if (aDeg >= BellyAlphaTableMinDeg && aDeg <= BellyAlphaTableMaxDeg && Math.Abs(bDeg) <= BellyBetaTableMaxDeg)
                {
                    set = TableSet.Belly;
                    alphaRad = a;
                    betaRad = b;
                    return true;
                }
            }
        }
        if (u.X > 0.0 && aero.NoseBodyAero != null && aero.NoseBodyMoment != null && SurfacesHave(TableSet.Nose))
        {
            BodyAeroTable.Angles(BodyAeroFrame.NoseFirst, u.X, u.Y, u.Z, out double a, out double b);
            double limit = NoseAngleMaxDeg * Math.PI / 180.0;
            if (Math.Abs(a) <= limit && Math.Abs(b) <= limit)
            {
                set = TableSet.Nose;
                alphaRad = a;
                betaRad = b;
                return true;
            }
        }
        if (u.X < 0.0 && aero.BodyAero != null && SurfacesHave(TableSet.Tail))
        {
            BodyAeroTable.Angles(BodyAeroFrame.TailFirst, u.X, u.Y, u.Z, out double a, out double b);
            double limit = BodyAngleMaxDeg * Math.PI / 180.0;
            if (Math.Abs(a) <= limit && Math.Abs(b) <= limit)
            {
                set = TableSet.Tail;
                alphaRad = a;
                betaRad = b;
                return true;
            }
        }
        if (aero.SphereBodyAero != null && aero.SphereBodyMoment != null && SurfacesHave(TableSet.Sphere))
        {
            BodyAeroTable.SphereAngles(aero.SphereBodyAero.Frame, u.X, u.Y, u.Z, out double lambda, out double mu);
            set = TableSet.Sphere;
            alphaRad = lambda;
            betaRad = mu;
            return true;
        }
        return false;
    }

    /// <summary>The four sides a belly can be, body axes, in Result.Belly's and BellySide's order: -Z, +Z, -Y, +Y.</summary>
    public static readonly double3[] BellyAxes = { new(0, 0, -1), new(0, 0, 1), new(0, -1, 0), new(0, 1, 0) };

    /// <summary>
    /// The whole-sphere tables' grids, degrees (BodyAeroTable.Sphere): the longitude every 15 deg about engine first, run 30 deg past +-180 with the values wrapped so the fit is continuous round the back, and the latitude every 15 deg pole to pole: 24 x 11 directions and the two poles, 266 a Mach row against the engine-first table's 361. The control surfaces' every 30 deg (12 x 5 and the poles, 62): their share of the force is small, and each deflection costs a sweep of its own.
    /// </summary>
    public static readonly double[] SphereLambdaDeg = Grid(-210.0, 210.0, 15.0);
    public static readonly double[] SphereMuDeg = Grid(-90.0, 90.0, 15.0);
    public static readonly double[] FinSphereLambdaDeg = Grid(-240.0, 240.0, 30.0);
    public static readonly double[] FinSphereMuDeg = Grid(-90.0, 90.0, 30.0);

    /// <summary>
    /// THE FLIP BAND's grids, degrees, about the whole sphere's frame (SphereFrame: 0 engine first, 90 belly first): the longitude every 5 deg from 15 past engine first to 10 past belly first, the latitude every 7.5 deg to 45 either side, for the body and each control surface alike (Result.BandBodyAero, SurfaceTable.BandForce): 24 x 13 directions a Mach row. Between the engine-first table's 20 deg and the belly-first table's 20 deg towards the engine (70 here) a belly-flopper's flip had only the whole sphere, 15 deg a node for the body and 30 for the flaps; mid-flip the Starship's flaps gave a third of what those said, and it fell 21 deg behind its plan (2026-11-24).
    /// </summary>
    public static readonly double[] BandLambdaDeg = Grid(-15.0, 100.0, 5.0);
    public static readonly double[] BandMuDeg = Grid(-45.0, 45.0, 7.5);

    private static double[] Grid(double from, double to, double step)
    {
        int n = (int)Math.Round((to - from) / step) + 1;
        var g = new double[n];
        for (int i = 0; i < n; i++) g[i] = from + i * step;
        return g;
    }

    /// <summary>The whole-sphere tables' frame for a belly side (BellyAxes index): engine first leading (-x), the longitude towards the belly, so 90 is belly first and the flip runs along the equator; the latitude towards the remaining axis.</summary>
    public static BodyAeroFrame SphereFrame(int side)
    {
        double3 l = new(-1, 0, 0), a = BellyAxes[side], b = double3.Cross(l, a);
        return new BodyAeroFrame(l.X, l.Y, l.Z, a.X, a.Y, a.Z, b.X, b.Y, b.Z);
    }

    /// <summary>The belly-first body table's frame for a side (BellyAxes index).</summary>
    public static BodyAeroFrame BellyFrame(int side) => BodyAeroFrame.Belly(BellyAxes[side].Y, BellyAxes[side].Z);

    /// <summary>
    /// Alpha and beta breakpoints of the belly-first body-axis table (Result.BellyBodyAero), degrees about the belly. Alpha, towards the nose, is 90 deg less the angle of attack from nose-first, so this covers 10 to 110 deg of it: a belly-flop's trim, and far enough towards nose-first that a craft the air has turned there keeps its flaps and its air moment to turn back with (2026-11-21: at 6 to 17 deg it had neither). Beta is the sideslip a bank and a reversal see. As coarse as the nose-first table's, and on one side only, the one the trim chose.
    /// </summary>
    public static readonly double[] BellyAlphaTableDeg = [-20, -10, 0, 10, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80];
    public static readonly double[] BellyBetaTableDeg = [-45, -32, -20, -12, -6, -3, 0, 3, 6, 12, 20, 32, 45];

    /// <summary>How far the belly-first table reaches, deg: past it a reader goes back to the other tables, or to none.</summary>
    public const double BellyAlphaTableMinDeg = -20.0, BellyAlphaTableMaxDeg = 80.0, BellyBetaTableMaxDeg = 45.0;

    /// <summary>The belly-first sweep's angles of attack, deg: dense through the belly-first range, sparse towards nose- and tail-first.</summary>
    public static readonly double[] BellyAlphaBreakpointsDeg =
        [0, 10, 20, 30, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 100, 110, 120, 135, 150, 165, 180];

    internal sealed class Prepared
    {
        public string Name;
        public VehicleProperties Props;
        public double3 Extents;
        public double Mass;
        public IParentBody Parent;
        public double SimTime;
        public BubbleOrigin Origin;
        public KinematicStates Kinematic;
        public PhysicsEnvironment Environment;
        /// <summary>The mesh to build and fly, private to this sweep; null where there is no air to measure.</summary>
        public AeroMeshSource Source;
        /// <summary>The mesh hash the result carries (Result.MeshHash): the live craft's for a whole-craft sweep, so the staleness check compares like with like; the booster geometry's for a booster.</summary>
        public ulong MeshHash;
        /// <summary>A booster sweep logs its whole time under this name; empty for the whole craft.</summary>
        public string BoosterName = "";

        /// <summary>Sweep the nose-first tables too (Result.NoseBodyAero): an ascent's. Whole-craft sweeps only.</summary>
        public bool NoseFirst;

        /// <summary>Sweep the belly-first trim rows too (Result.Belly): an orbital entry's. Whole-craft sweeps only.</summary>
        public bool BellyFirst;

        /// <summary>Sweep the belly-first body and surface tables on this side (BellyAxes index), -1 for none: an orbital entry's, once its trim has chosen the side.</summary>
        public int BellySide = -1;

        /// <summary>The control surfaces to sweep (ControlSurfaces), each with the binder that finds it in the private mesh. Whole-craft sweeps only.</summary>
        public List<(ControlSurface Surface, Func<Aerodynamics, Action<double>> Binder)> Surfaces = new();

        /// <summary>Each surface's PartKey, in the order of <see cref="Surfaces"/>: read on the main thread.</summary>
        public List<string> SurfaceKeys = new();

        /// <summary>The file the finished sweep is saved to (KsaAeroSweepStore), null for none.</summary>
        public string StorePath;

        /// <summary>What the sweep is doing, for TUNE > DIAG > AERO (KsaAeroSweepStatus); null for one run on the caller's thread.</summary>
        public SweepProgress Progress;
    }

    private static bool HasAir(IParentBody parent) => parent?.GetPhysicalAtmosphere() is { Height: > 0.0 };

    private static void CopyPhysics(Vehicle vehicle, Prepared p)
    {
        p.Origin = vehicle.BubbleOrigin;
        p.Kinematic = vehicle.KinematicStates;
        p.Environment = vehicle.PhysicsEnvironment;
    }

    /// <summary>
    /// The whole craft's sweep, prepared on the main thread: its properties and physics state copied, and its aero mesh gathered afresh from its parts (AeroMeshSource.Gather, as the game gathers its own), so the sweep flies a mesh of its own. The live mesh cannot be flown off the main thread: the vehicle's physics worker steps its surface flow. False while the game rebuilds the mesh (<see cref="RebuildingError"/>): the parts have not settled.
    /// </summary>
    private static bool TryPrepareCraft(Vehicle vehicle, IParentBody parent, double simTime, out Prepared prepared, out string error)
    {
        prepared = null;
        error = "";
        if (vehicle == null)
        {
            error = "no vehicle";
            return false;
        }
        PartTree live = vehicle.Parts;
        if (live != null && live.IsRebuildingAerodynamics)
        {
            error = RebuildingError;
            return false;
        }
        bool meshFailed = live != null && live.HasAerodynamicsBuildFailed;
        if (meshFailed)
            LogHelper.WarnOnce("afc-aero-build-failed:" + vehicle.Id,
                $"[AFC] The game could not build the aero mesh of '{vehicle.Id}'. The aero sweep measures the drag the game applies without it.");

        // Extents along the ASSEMBLY axes. x is the long axis for any sane rocket - the one the thrust axis lies along.
        //  Read through Vehicle's own accessor: the same box Props carries, and what the staleness check reads too, so the two cannot disagree about which box the table was built from.
        float3 half = vehicle.BoundingBoxHalfExtentsAsmb;
        prepared = new Prepared
        {
            Name = vehicle.Id,
            Props = vehicle.Props,
            Extents = new double3(half.X * 2.0, half.Y * 2.0, half.Z * 2.0),
            Mass = vehicle.TotalMass,
            Parent = parent,
            SimTime = simTime,
            MeshHash = LiveMeshHash(vehicle),
        };
        CopyPhysics(vehicle, prepared);
        if (HasAir(parent) && live != null && !meshFailed)
        {
            try
            {
                prepared.Source = AeroMeshSource.Gather(live);
            }
            catch (Exception ex)
            {
                error = $"could not gather the aero mesh ({ex.Message})";
                return false;
            }
            // Its control surfaces, each with what finds it in the private mesh.
            if (ControlSurfaces.Any)
            {
                var surfaces = new List<ControlSurface>();
                ControlSurfaces.Collect(vehicle, surfaces);
                foreach (ControlSurface surface in surfaces)
                {
                    Func<Aerodynamics, Action<double>> binder = ControlSurfaces.Poser(vehicle, surface);
                    if (binder != null)
                    {
                        prepared.Surfaces.Add((surface, binder));
                        prepared.SurfaceKeys.Add(PartKey(surface.Part));
                    }
                }
            }
        }
        return true;
    }

    /// <summary>The booster's sweep, prepared on the main thread: its properties as the vehicle it becomes (KsaBoosterGeometry) and its mesh source (KsaBoosterAeroMesh.TryGather).</summary>
    private static bool TryPrepareBooster(Vehicle vehicle, Part root, System.Collections.Generic.HashSet<Part> parts, IParentBody parent, double simTime,
                                          out Prepared prepared, out string error)
    {
        prepared = null;
        if (!KsaBoosterGeometry.TryProperties(vehicle, root, parts, out VehicleProperties props, out double3 extents, out error))
            return false;
        prepared = new Prepared
        {
            Name = vehicle.Id,
            Props = props,
            Extents = extents,
            Mass = props.TotalMass,
            Parent = parent,
            SimTime = simTime,
            BoosterName = root.DisplayName,
        };
        CopyPhysics(vehicle, prepared);
        if (HasAir(parent))
        {
            if (!KsaBoosterAeroMesh.TryGather(root, parts, out AeroMeshSource source, out ulong hash, out error))
                return false;
            prepared.Source = source;
            prepared.MeshHash = hash;
        }
        return true;
    }

    /// <summary>
    /// A prepared sweep run to its result: the private mesh built (KsaBoosterAeroMesh.TryBuildHolder), flown, and disposed. Reads nothing of the game's but what <paramref name="p"/> holds and the body's atmosphere, which the game's own physics workers read too. <paramref name="log"/> takes the log lines, so a worker can hand them to the main thread.
    /// </summary>
    /// <param name="bodyReady">Called with the result as soon as its body table is fitted, before the surfaces are swept (two passes, see Sweep); null runs both before returning.</param>
    private static bool Run(Prepared p, Action<string> log, out Result result, out string error, Action<Result> bodyReady = null)
    {
        result = null;
        var clock = Stopwatch.StartNew();
        PartTree holder = null;
        try
        {
            if (p.Source != null && !KsaBoosterAeroMesh.TryBuildHolder(p.Source, out holder, out error))
                return false;
            if (!Sweep(p, holder, log, out result, out error, bodyReady))
                return false;
            if (p.BoosterName.Length > 0)
            {
                result.SweepMillis = clock.Elapsed.TotalMilliseconds;
                log(FormattableString.Invariant($"[AFC] Booster aero of '{p.BoosterName}': mesh and sweep {result.SweepMillis:F0} ms."));
            }
            return true;
        }
        catch (Exception ex)
        {
            error = $"aero sweep failed ({ex.GetType().Name}: {ex.Message})";
            result = null;
            return false;
        }
        finally
        {
            holder?.Dispose();
        }
    }

    /// <summary>
    /// One sweep running on its own background thread (#223: each sweep held the main thread for about a second, at SOLVE and at every staging). Started on the main thread (<see cref="TryStart"/>, <see cref="TryStartBooster"/>), which prepares it; taken there too (<see cref="TryTake"/>), which writes its log lines. Its own thread rather than the shared pool, whose queue held entry guidance's solves for seconds (afc-entry-solve).
    /// </summary>
    public sealed class Job
    {
        private volatile bool _done;
        private Result _result;
        private string _error = "";
        // Set once the whole sweep has ended, its surfaces too (Done comes earlier, with the body table).
        private readonly System.Threading.ManualResetEventSlim _finished = new(false);

        private Job()
        {
        }

        private Job(Prepared prepared)
        {
            SweepProgress progress = prepared.Progress = Track(prepared);
            var thread = new System.Threading.Thread(() =>
            {
                try
                {
                    // Taken as soon as the body table is fitted; the surfaces follow into the same result.
                    bool ok = Run(prepared, PendingLog.Enqueue, out Result result, out string error, bodyReady: body =>
                    {
                        _result = body;
                        _done = true;
                    });
                    if (!_done)
                    {
                        if (ok)
                            _result = result;
                        else
                            _error = error;
                    }
                    if (ok)
                    {
                        // Saved with its surfaces, for the next time this craft is asked for (KsaAeroSweepStore).
                        SaveSweep(prepared.StorePath, result, prepared.BoosterName.Length > 0 ? prepared.BoosterName : prepared.Name, PendingLog.Enqueue);
                        progress.Enter(SweepProgress.Stage.Done);
                    }
                    else
                        progress.Fail(error);
                }
                catch (Exception ex)
                {
                    _error = $"aero sweep failed ({ex.GetType().Name}: {ex.Message})";
                    progress.Fail(_error);
                }
                finally
                {
                    _done = true;
                    if (progress.Running)
                        progress.Fail("ended without a result");
                    _finished.Set();
                }
            })
            {
                IsBackground = true,
                Name = "afc-aero-sweep",
                Priority = System.Threading.ThreadPriority.BelowNormal,
            };
            thread.Start();
        }

        internal static Job StartFor(Prepared prepared) => new(prepared);

        /// <summary>A job already done with a result in hand: a cache hit (KsaAeroSweepCache).</summary>
        internal static Job Completed(Result result)
        {
            var job = new Job { _result = result, _done = true };
            job._finished.Set();
            return job;
        }

        /// <summary>A job whose result is worked out on its own thread from another's: a cached sweep re-linked to a reloaded craft or moved to its centre of mass (KsaAeroSweepCache).</summary>
        internal static Job Derived(Func<(Result Result, string Error)> work, SweepProgress progress)
        {
            var job = new Job();
            var thread = new System.Threading.Thread(() =>
            {
                try
                {
                    (Result result, string error) = work();
                    job._result = result;
                    job._error = error ?? "";
                    if (result != null)
                        progress?.Enter(SweepProgress.Stage.Done);
                    else
                        progress?.Fail(job._error);
                }
                catch (Exception ex)
                {
                    job._error = $"cached aero tables could not be taken ({ex.GetType().Name}: {ex.Message})";
                    progress?.Fail(job._error);
                }
                finally
                {
                    job._done = true;
                    job._finished.Set();
                }
            })
            {
                IsBackground = true,
                Name = "afc-aero-cache",
                Priority = System.Threading.ThreadPriority.BelowNormal,
            };
            thread.Start();
            return job;
        }

        /// <summary>Whether the whole sweep has ended, control surfaces too.</summary>
        internal bool Finished => _finished.IsSet;

        /// <summary>Blocks until the whole sweep has ended. Worker threads only.</summary>
        internal void WaitFinished() => _finished.Wait();

        /// <summary>The result so far (the body table, with surfaces filling in), or null while there is none or the sweep failed. Any thread; writes no log.</summary>
        internal Result Peek => _done ? _result : null;

        /// <summary>Whether the sweep has its body table, or has failed: what a caller waits on. The surfaces may still be being swept (Result.Surfaces fills in when they are).</summary>
        public bool Done => _done;

        /// <summary>
        /// The finished sweep: true with its result, or with the reason it failed in <paramref name="error"/> and a null result. False while it still runs. Main thread: writes the sweep's log.
        /// </summary>
        public bool TryTake(out Result result, out string error)
        {
            result = null;
            error = "";
            if (!_done)
                return false;
            WriteLog();
            result = _result;
            error = _result == null ? (_error.Length > 0 ? _error : "aero sweep gave no table") : "";
            return true;
        }
    }

    /// <summary>The whole craft's sweep, started on its own thread. False, with the reason, where it cannot be prepared - <see cref="RebuildingError"/> while the game rebuilds the mesh. Main thread.</summary>
    /// <param name="noseFirst">Sweep the nose-first tables too (Result.NoseBodyAero), for an ascent.</param>
    /// <param name="bellyFirst">Sweep the belly-first tables too (Result.Belly), for an orbital entry.</param>
    /// <param name="bellySide">Sweep the belly-first body and surface tables on this side (BellyAxes index), for an orbital entry; -1 for none.</param>
    /// <param name="fresh">Sweep again whatever the cache holds: DIAG's RESAMPLE.</param>
    public static bool TryStart(Vehicle vehicle, IParentBody parent, double simTime, out Job job, out string error, bool noseFirst = false, bool bellyFirst = false, int bellySide = -1, bool fresh = false)
    {
        job = null;
        error = "";
        // EVERY TABLE, every time (2026-11-25, the user's call): tail, nose, belly trim rows and the belly side, on the craft's own belly (BELLY AXIS), whichever mode asks. Asked for piecemeal, each mode's set missed the last one's - three sweeps of a Starship in a minute, and one on a side guessed from the attitude in orbit. About twice the body pass (5 to 10 s on a worker thread); the surfaces cost the same.
        noseFirst = true;
        bellyFirst = true;
        if (bellySide < 0 && vehicle != null)
            bellySide = VehicleAutopilotState.For(vehicle).BellySideWanted();
        // The same craft swept before, this game session: its tables again, moved to the craft as it is now (KsaAeroSweepCache).
        ulong hash = LiveMeshHash(vehicle);
        double3 com = vehicle?.Props is { } props ? new double3(props.CenterOfMassAsmb.X, props.CenterOfMassAsmb.Y, props.CenterOfMassAsmb.Z) : default;
        if (vehicle != null && !fresh && TryCached(vehicle, parent, hash, "", noseFirst, bellyFirst, bellySide, com, out job))
            return true;
        if (!TryPrepareCraft(vehicle, parent, simTime, out Prepared prepared, out error))
            return false;
        // What was swept before on this geometry is swept again with it, so one result serves every mode.
        CacheUnion(hash, parent, "", ref noseFirst, ref bellyFirst, ref bellySide);
        prepared.NoseFirst = noseFirst;
        prepared.BellyFirst = bellyFirst;
        prepared.BellySide = bellySide;
        prepared.StorePath = StorePathFor(vehicle, parent, prepared.MeshHash, "", bellySide);
        job = Job.StartFor(prepared);
        CacheAdd(vehicle, parent, prepared.MeshHash, "", noseFirst, bellyFirst, bellySide, job);
        return true;
    }

    /// <summary>A booster's sweep (TryBuildBooster), started on its own thread. Main thread.</summary>
    internal static bool TryStartBooster(Vehicle vehicle, Part root, System.Collections.Generic.HashSet<Part> parts, IParentBody parent, double simTime,
                                         out Job job, out string error)
    {
        job = null;
        if (!TryPrepareBooster(vehicle, root, parts, parent, simTime, out Prepared prepared, out error))
            return false;
        // A booster is keyed by its own geometry's hash, and moved to its own centre of mass.
        string booster = "BOOSTER " + prepared.BoosterName;
        double3 com = new(prepared.Props.CenterOfMassAsmb.X, prepared.Props.CenterOfMassAsmb.Y, prepared.Props.CenterOfMassAsmb.Z);
        if (TryCached(vehicle, parent, prepared.MeshHash, booster, false, false, -1, com, out job))
            return true;
        prepared.StorePath = StorePathFor(vehicle, parent, prepared.MeshHash, booster, -1);
        job = Job.StartFor(prepared);
        CacheAdd(vehicle, parent, prepared.MeshHash, booster, false, false, -1, job);
        return true;
    }

    /// <summary>The sweep of one prepared craft or booster, flown through its private mesh <paramref name="parts"/> (null where there is no air): the box its properties were built on is p.Extents, along the assembly axes.</summary>
    private static bool Sweep(Prepared p, PartTree parts, Action<string> log, out Result result, out string error,
                              Action<Result> bodyReady = null)
    {
        result = null;
        error = "";
        p.Progress?.Enter(SweepProgress.Stage.Body);
        VehicleProperties props = p.Props;
        double3 extents = p.Extents;
        double mass = p.Mass;
        IParentBody parent = p.Parent;
        double simTime = p.SimTime;
        string name = p.Name;
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
            res.MeshHash = p.MeshHash;
            res.Parent = parent;
            result = res;
            return true;
        }

        // The air the probe flies through: sea level of this body's own atmosphere. The game reads the Mach number from the speed of sound it is given and the gas properties from the temperature, so both come from the same sea level temperature. The Mach axis is airspeed over the sea level speed of sound, the same scaling the solvers use.
        double seaTemperature = physical.ComputeTemperatureAtAltitude(0.0);
        AtmosphericConditions sea = physical.ComputeConditionsAtAltitude(0.0);
        double speedOfSound = physical.Mixture.ComputeSpeedOfSound(seaTemperature);
        if (!ForceProbe.TryCreate(p.Origin, p.Kinematic, p.Environment, props, parts, parent, sea.Density, sea.Pressure, seaTemperature, speedOfSound, out ForceProbe probe, out error))
            return false;
        var flow = new SamplingFlow(parts);

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
        int nbm = bodyMach.Length, nab = BodyAngleBreakpointsDeg.Length, nan = NoseAngleBreakpointsDeg.Length;
        // THE CONTROL SURFACES' MACH ROWS (SurfaceMachRows): fewer than the body's - a surface's change to the loads varies slowly with Mach, and each row is a call of the game's force (2026-11-25: four flaps on all 27 rows took 40 s of the sweep's 50). Their rest values are the body's own rows there.
        double[] surfaceMach = SurfaceMachRows(bodyMach);
        int nsm = surfaceMach.Length;
        int[] surfaceRow = surfaceMach.Select(m => Array.IndexOf(bodyMach, m)).ToArray();
        double[] SurfaceRowsOf(double[] full, int perRow)
        {
            var rowsOut = new double[nsm * perRow];
            for (int i = 0; i < nsm; i++)
                Array.Copy(full, surfaceRow[i] * perRow, rowsOut, i * perRow, perRow);
            return rowsOut;
        }
        double momentLength = dx;
        // One grid of directions flown: per alpha and beta of the grid (about the nose for a nose-first one), the coefficients of the force and the moment, as BodyAeroTable lays them out.
        void FlyGrid(double[] anglesDeg, bool nose, double[] coefficients, double[] moments)
            => FlyFrame(anglesDeg, anglesDeg, nose ? BodyAeroFrame.NoseFirst : BodyAeroFrame.TailFirst, coefficients, moments);
        // The same about any frame, with its own alpha and beta grids.
        void FlyFrame(double[] alphaDeg, double[] betaDeg, BodyAeroFrame frame, double[] coefficients, double[] moments, double[] machs = null)
        {
            double[] rows = machs ?? bodyMach;
            int nr = rows.Length;
            int na2 = alphaDeg.Length, nb2 = betaDeg.Length;
            for (int ja = 0; ja < na2; ja++)
            {
                for (int jb = 0; jb < nb2; jb++)
                {
                    (double ux, double uy, double uz) = BodyAeroTable.AirDirection(alphaDeg[ja] * Math.PI / 180.0, betaDeg[jb] * Math.PI / 180.0, frame);
                    var u = new double3(ux, uy, uz);
                    probe.Wrench(BaselineSpeed * u, out double3 baseline, out double3 baselineTorque);
                    flow.Prepare(u);
                    for (int i = 0; i < nr; i++)
                    {
                        double speed = Math.Max(rows[i], MinProbeMach) * speedOfSound;
                        probe.Wrench(speed * u, out double3 rawForce, out double3 rawTorque);
                        double3 force = rawForce - baseline;
                        double3 torque = rawTorque - baselineTorque;
                        double qa = 0.5 * sea.Density * speed * speed * refArea;
                        int node = ((i * na2 + ja) * nb2 + jb) * 3;
                        coefficients[node] = force.X / qa;
                        coefficients[node + 1] = force.Y / qa;
                        coefficients[node + 2] = force.Z / qa;
                        if (momentLength > 0.0)
                        {
                            // A torque the game did not report reads as none, as in the roll-averaged table.
                            moments[node] = double.IsFinite(torque.X) ? torque.X / (qa * momentLength) : 0.0;
                            moments[node + 1] = double.IsFinite(torque.Y) ? torque.Y / (qa * momentLength) : 0.0;
                            moments[node + 2] = double.IsFinite(torque.Z) ? torque.Z / (qa * momentLength) : 0.0;
                        }
                    }
                }
            }
        }
        // The same over the whole sphere, on a longitude and latitude grid about a frame: each distinct direction flown once (the wrapped longitudes and the poles repeat), its rows copied to every node that points there.
        void FlySphere(double[] lambdaDeg, double[] muDeg, BodyAeroFrame frame, double[] coefficients, double[] moments, double[] machs = null)
        {
            double[] rows = machs ?? bodyMach;
            int nr = rows.Length;
            int nl = lambdaDeg.Length, nmu = muDeg.Length;
            var seen = new Dictionary<(int, int), (int L, int M)>();
            for (int jl = 0; jl < nl; jl++)
            {
                for (int jm = 0; jm < nmu; jm++)
                {
                    bool pole = Math.Abs(Math.Abs(muDeg[jm]) - 90.0) < 1e-9;
                    int lon = pole ? 0 : (int)Math.Round(((lambdaDeg[jl] % 360.0) + 540.0) % 360.0 - 180.0);
                    var key = (lon, (int)Math.Round(muDeg[jm]));
                    if (seen.TryGetValue(key, out var from))
                    {
                        for (int i = 0; i < nr; i++)
                        {
                            int src = ((i * nl + from.L) * nmu + from.M) * 3, dst = ((i * nl + jl) * nmu + jm) * 3;
                            for (int c = 0; c < 3; c++) { coefficients[dst + c] = coefficients[src + c]; moments[dst + c] = moments[src + c]; }
                        }
                        continue;
                    }
                    seen[key] = (jl, jm);
                    (double ux, double uy, double uz) = BodyAeroTable.SphereDirection(lambdaDeg[jl] * Math.PI / 180.0, muDeg[jm] * Math.PI / 180.0, frame);
                    var u = new double3(ux, uy, uz);
                    probe.Wrench(BaselineSpeed * u, out double3 baseline, out double3 baselineTorque);
                    flow.Prepare(u);
                    for (int i = 0; i < nr; i++)
                    {
                        double speed = Math.Max(rows[i], MinProbeMach) * speedOfSound;
                        probe.Wrench(speed * u, out double3 rawForce, out double3 rawTorque);
                        double3 force = rawForce - baseline;
                        double3 torque = rawTorque - baselineTorque;
                        double qa = 0.5 * sea.Density * speed * speed * refArea;
                        int node = ((i * nl + jl) * nmu + jm) * 3;
                        coefficients[node] = force.X / qa;
                        coefficients[node + 1] = force.Y / qa;
                        coefficients[node + 2] = force.Z / qa;
                        if (momentLength > 0.0)
                        {
                            moments[node] = double.IsFinite(torque.X) ? torque.X / (qa * momentLength) : 0.0;
                            moments[node + 1] = double.IsFinite(torque.Y) ? torque.Y / (qa * momentLength) : 0.0;
                            moments[node + 2] = double.IsFinite(torque.Z) ? torque.Z / (qa * momentLength) : 0.0;
                        }
                    }
                }
            }
        }
        var bodyCoefficients = new double[nbm * nab * nab * 3];
        var bodyMoments = new double[nbm * nab * nab * 3];
        FlyGrid(BodyAngleBreakpointsDeg, false, bodyCoefficients, bodyMoments);
        // NOSE FIRST, for an ascent (#216): the same, about the nose.
        double[] noseCoefficients = null, noseMoments = null;
        if (p.NoseFirst)
        {
            noseCoefficients = new double[nbm * nan * nan * 3];
            noseMoments = new double[nbm * nan * nan * 3];
            FlyGrid(NoseAngleBreakpointsDeg, true, noseCoefficients, noseMoments);
        }
        // BELLY FIRST body tables, for an orbital entry on the side its trim chose: the same calls again, about the belly.
        double[] bellyCoefficients = null, bellyMoments = null;
        int bellySide = p.BellySide >= 0 && p.BellySide < BellyAxes.Length ? p.BellySide : -1;
        BodyAeroFrame bellyFrame = bellySide >= 0 ? BellyFrame(bellySide) : default;
        int nbelly = BellyAlphaTableDeg.Length * BellyBetaTableDeg.Length;
        if (bellySide >= 0)
        {
            bellyCoefficients = new double[nbm * nbelly * 3];
            bellyMoments = new double[nbm * nbelly * 3];
            FlyFrame(BellyAlphaTableDeg, BellyBetaTableDeg, bellyFrame, bellyCoefficients, bellyMoments);
        }
        // THE WHOLE SPHERE with them, for touchdown's flip (Result.SphereBodyAero); the fins' coarser one is flown with the surfaces.
        double[] sphereCoefficients = null, sphereMoments = null, finSphereRest = null, finSphereRestMoments = null;
        BodyAeroFrame sphereFrame = bellySide >= 0 ? SphereFrame(bellySide) : default;
        if (bellySide >= 0)
        {
            sphereCoefficients = new double[nbm * SphereLambdaDeg.Length * SphereMuDeg.Length * 3];
            sphereMoments = new double[sphereCoefficients.Length];
            FlySphere(SphereLambdaDeg, SphereMuDeg, sphereFrame, sphereCoefficients, sphereMoments);
        }
        // THE FLIP BAND, finer, between the engine-first and belly-first tables (BandLambdaDeg).
        double[] bandCoefficients = null, bandMoments = null, bandRest = null, bandRestMoments = null;
        if (bellySide >= 0)
        {
            bandCoefficients = new double[nbm * BandLambdaDeg.Length * BandMuDeg.Length * 3];
            bandMoments = new double[bandCoefficients.Length];
            FlySphere(BandLambdaDeg, BandMuDeg, sphereFrame, bandCoefficients, bandMoments);
        }
        // BELLY FIRST, for an orbital entry: per candidate belly axis b (body axes, square to x), the air at angle of attack a comes from cos(a) x + sin(a) b, so 0 is nose-first and 90 is belly-first. Drag is along that direction, lift along sin(a) x - cos(a) b, and the pitch moment about b x x, positive when it raises a.
        if (p.BellyFirst)
        {
            double[] bellyAlpha = BellyAlphaBreakpointsDeg;
            int nba = bellyAlpha.Length;
            double3[] axes = BellyAxes;
            var sweeps = new BellySweep[axes.Length];
            for (int ax = 0; ax < axes.Length; ax++)
            {
                double3 b = axes[ax];
                var cd = new double[nm * nba];
                var cl = new double[nm * nba];
                var cm = new double[nm * nba];
                double3 pitchAxis = double3.Cross(b, new double3(1, 0, 0));
                for (int j = 0; j < nba; j++)
                {
                    double a = bellyAlpha[j] * Math.PI / 180.0;
                    double3 u = new double3(Math.Cos(a), 0, 0) + Math.Sin(a) * b;
                    double3 lift = new double3(Math.Sin(a), 0, 0) - Math.Cos(a) * b;
                    probe.Wrench(BaselineSpeed * u, out double3 baseline, out double3 baselineTorque);
                    flow.Prepare(u);
                    for (int i = 0; i < nm; i++)
                    {
                        probe.Wrench(speeds[i] * u, out double3 rawForce, out double3 rawTorque);
                        double3 force = rawForce - baseline;
                        double3 torque = rawTorque - baselineTorque;
                        double qa = 0.5 * sea.Density * speeds[i] * speeds[i] * refArea;
                        int cell = i * nba + j;
                        cd[cell] = -double3.Dot(force, u) / qa;
                        cl[cell] = double3.Dot(force, lift) / qa;
                        cm[cell] = momentLength > 0.0 && double.IsFinite(torque.X)
                            ? double3.Dot(torque, pitchAxis) / (qa * momentLength) : 0.0;
                    }
                }
                sweeps[ax] = new BellySweep
                {
                    Axis = b, MachGrid = machGrid, AlphaDeg = bellyAlpha, Cd = cd, Cl = cl, Cm = cm,
                    ReferenceArea = refArea, ReferenceLength = momentLength,
                };
            }
            res.Belly = sweeps;
        }
        res.SweepMillis = clock.Elapsed.TotalMilliseconds;
        res.MeshHash = p.MeshHash;
        res.Parent = parent;
        log(FormattableString.Invariant(
            $"[AFC] Aero sweep of '{name}' over {(parent as Astronomical)?.Id}: {na * RollSamples} directions, {na * RollSamples * (nm + 1)} samples{(p.NoseFirst ? FormattableString.Invariant($", nose first {nan * nan} directions") : "")}, {res.SweepMillis:F0} ms."));

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
        var armTable = new double[nm * na];
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
                double arm = armSums[cell] / RollSamples / (qal / refLength);
                armTable[cell] = double.IsFinite(arm) ? arm : 0.0;
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
        res.ArmTable = refLength > 0.0 ? armTable : null;
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
            if (noseCoefficients != null)
            {
                res.NoseBodyAero = new BodyAeroTable(bodyMach, NoseAngleBreakpointsDeg, NoseAngleBreakpointsDeg, noseCoefficients, noseFirst: true);
                res.NoseBodyMoment = momentLength > 0.0 ? new BodyAeroTable(bodyMach, NoseAngleBreakpointsDeg, NoseAngleBreakpointsDeg, noseMoments, noseFirst: true) : null;
            }
            if (bellyCoefficients != null)
            {
                res.BellyBodyAero = new BodyAeroTable(bodyMach, BellyAlphaTableDeg, BellyBetaTableDeg, bellyCoefficients, bellyFrame);
                res.BellyBodyMoment = momentLength > 0.0 ? new BodyAeroTable(bodyMach, BellyAlphaTableDeg, BellyBetaTableDeg, bellyMoments, bellyFrame) : null;
                res.BellySide = bellySide;
            }
            if (sphereCoefficients != null)
            {
                res.SphereBodyAero = new BodyAeroTable(bodyMach, SphereLambdaDeg, SphereMuDeg, sphereCoefficients, sphereFrame, sphere: true);
                res.SphereBodyMoment = momentLength > 0.0 ? new BodyAeroTable(bodyMach, SphereLambdaDeg, SphereMuDeg, sphereMoments, sphereFrame, sphere: true) : null;
            }
            if (bandCoefficients != null)
            {
                res.BandBodyAero = new BodyAeroTable(bodyMach, BandLambdaDeg, BandMuDeg, bandCoefficients, sphereFrame, sphere: true);
                res.BandBodyMoment = momentLength > 0.0 ? new BodyAeroTable(bodyMach, BandLambdaDeg, BandMuDeg, bandMoments, sphereFrame, sphere: true) : null;
            }
        }
        catch (Exception ex)
        {
            error = "table fit failed: " + ex.Message;
            return false;
        }

        // The measured lift near alpha 0, the angles a booster glides at, so the shape of the curve there can be read off the game log.
        log(FormattableString.Invariant($"[AFC] Lift near alpha 0 of '{name}' (C_L, roll-averaged, + towards the engine end's tilt): {NearZeroLiftText(res)}."));
        if (res.BodyAero != null)
            log(FormattableString.Invariant($"[AFC] Body-axis force of '{name}' near retrograde at Mach 0.4 (C_x, C_y, C_z): {BodyAeroText(res.BodyAero, 0.4)}."));
        if (res.BodyMoment != null)
            log(FormattableString.Invariant($"[AFC] Body-axis moment of '{name}' near retrograde at Mach 0.4 (C_l, C_m, C_n about the centre of mass, L {res.ReferenceLength:F1} m): {BodyAeroText(res.BodyMoment, 0.4)}."));
        if (res.NoseBodyAero != null)
            log(FormattableString.Invariant($"[AFC] Body-axis force of '{name}' nose first at Mach 0.4 (C_x, C_y, C_z): {BodyAeroText(res.NoseBodyAero, 0.4)}."));
        if (res.NoseBodyMoment != null)
            log(FormattableString.Invariant($"[AFC] Body-axis moment of '{name}' nose first at Mach 0.4 (C_l, C_m, C_n): {BodyAeroText(res.NoseBodyMoment, 0.4)}."));
        // Logged once the moment table exists, which BalanceText reads.
        if (double.IsFinite(worstCp))
            log(FormattableString.Invariant(
                $"[AFC] Aero balance of '{name}': {BalanceText(res)}; centre of pressure at {BalanceAlphaDeg:0} deg by Mach: {CentreOfPressureText(res)}."));
        result = res;

        // TWO PASSES. The body table is ready here and handed over (bodyReady): a sweep on its own thread publishes it now, and whoever waits on the craft's aero - a boostback engaged at separation, a SOLVE - goes on with it. The surfaces are swept after, on the same private mesh, and their tables attached when they are all done (Result.Surfaces, written once). Four fins took 5.2 s against the body's 2 s (2026-11-20).
        bodyReady?.Invoke(res);
        if (p.Surfaces.Count == 0)
            return true;
        p.Progress?.Enter(SweepProgress.Stage.Surfaces);
        try
        {
            // THE CONTROL SURFACES (SurfaceTable): each turned alone to every deflection of its grid, and the body table's directions and rows flown again, the same calls in the same order. Turned back to rest whatever happens, so the next one is swept against the rest of the craft as built.
            var surfaceTables = new List<SurfaceTable>();
            int surfaceIndex = -1;
            Aerodynamics surfaceMesh = p.Surfaces.Count > 0 && momentLength > 0.0 ? parts?.AwaitAerodynamics() : null;
            foreach ((ControlSurface surface, Func<Aerodynamics, Action<double>> binder) in p.Surfaces)
            {
                surfaceIndex++;
                if (surfaceMesh == null)
                    break;
                var surfaceClock = Stopwatch.StartNew();
                p.Progress?.SurfaceStarted(surface.Name);
                Action<double> pose = binder(surfaceMesh);
                if (pose == null)
                {
                    log($"[AFC] Control surface '{surface.Name}' of '{name}' is not in its aero mesh; not swept.");
                    p.Progress?.SurfaceEnded(missing: true);
                    continue;
                }
                double[] grid = SurfaceDeflections(surface.MinDeg, surface.MaxDeg);
                var forceTables = new BodyAeroTable[grid.Length];
                var momentTables = new BodyAeroTable[grid.Length];
                var noseForceTables = p.NoseFirst ? new BodyAeroTable[grid.Length] : [];
                var noseMomentTables = p.NoseFirst ? new BodyAeroTable[grid.Length] : [];
                var bellyForceTables = bellySide >= 0 ? new BodyAeroTable[grid.Length] : [];
                var bellyMomentTables = bellySide >= 0 ? new BodyAeroTable[grid.Length] : [];
                var sphereForceTables = bellySide >= 0 ? new BodyAeroTable[grid.Length] : [];
                var sphereMomentTables = bellySide >= 0 ? new BodyAeroTable[grid.Length] : [];
                var bandForceTables = bellySide >= 0 ? new BodyAeroTable[grid.Length] : [];
                var bandMomentTables = bellySide >= 0 ? new BodyAeroTable[grid.Length] : [];
                // The body at rest on the band's grid, once for all the surfaces, before any is turned.
                if (bellySide >= 0 && bandRest == null)
                {
                    bandRest = new double[nsm * BandLambdaDeg.Length * BandMuDeg.Length * 3];
                    bandRestMoments = new double[bandRest.Length];
                    FlySphere(BandLambdaDeg, BandMuDeg, sphereFrame, bandRest, bandRestMoments, surfaceMach);
                }
                void DeltaBand(int d)
                {
                    int n3 = bandRest.Length;
                    var df = new double[n3];
                    var dm = new double[n3];
                    if (grid[d] != 0.0)
                    {
                        FlySphere(BandLambdaDeg, BandMuDeg, sphereFrame, df, dm, surfaceMach);
                        for (int k = 0; k < n3; k++)
                        {
                            df[k] -= bandRest[k];
                            dm[k] -= bandRestMoments[k];
                        }
                    }
                    bandForceTables[d] = new BodyAeroTable(surfaceMach, BandLambdaDeg, BandMuDeg, df, sphereFrame, sphere: true);
                    bandMomentTables[d] = new BodyAeroTable(surfaceMach, BandLambdaDeg, BandMuDeg, dm, sphereFrame, sphere: true);
                }
                // The body at rest on the fins' coarser sphere, once for all the surfaces, before any is turned.
                if (bellySide >= 0 && finSphereRest == null)
                {
                    finSphereRest = new double[nsm * FinSphereLambdaDeg.Length * FinSphereMuDeg.Length * 3];
                    finSphereRestMoments = new double[finSphereRest.Length];
                    FlySphere(FinSphereLambdaDeg, FinSphereMuDeg, sphereFrame, finSphereRest, finSphereRestMoments, surfaceMach);
                }
                void DeltaSphere(int d)
                {
                    int n3 = finSphereRest.Length;
                    var df = new double[n3];
                    var dm = new double[n3];
                    if (grid[d] != 0.0)
                    {
                        FlySphere(FinSphereLambdaDeg, FinSphereMuDeg, sphereFrame, df, dm, surfaceMach);
                        for (int k = 0; k < n3; k++)
                        {
                            df[k] -= finSphereRest[k];
                            dm[k] -= finSphereRestMoments[k];
                        }
                    }
                    sphereForceTables[d] = new BodyAeroTable(surfaceMach, FinSphereLambdaDeg, FinSphereMuDeg, df, sphereFrame, sphere: true);
                    sphereMomentTables[d] = new BodyAeroTable(surfaceMach, FinSphereLambdaDeg, FinSphereMuDeg, dm, sphereFrame, sphere: true);
                }
                // The change from the body's own: the grid flown with the surface turned, less the body table at rest.
                void Delta(double[] anglesDeg, bool nose, double[] restForce, double[] restMoment, int d, BodyAeroTable[] forceOut, BodyAeroTable[] momentOut)
                    => DeltaFrame(anglesDeg, anglesDeg, nose ? BodyAeroFrame.NoseFirst : BodyAeroFrame.TailFirst, restForce, restMoment, d, forceOut, momentOut);
                void DeltaFrame(double[] alphaDeg, double[] betaDeg, BodyAeroFrame frame, double[] restForce, double[] restMoment, int d, BodyAeroTable[] forceOut, BodyAeroTable[] momentOut)
                {
                    int perRow = alphaDeg.Length * betaDeg.Length * 3;
                    int n3 = nsm * perRow;
                    var df = new double[n3];
                    var dm = new double[n3];
                    if (grid[d] != 0.0)
                    {
                        FlyFrame(alphaDeg, betaDeg, frame, df, dm, surfaceMach);
                        double[] restF = SurfaceRowsOf(restForce, perRow), restM = SurfaceRowsOf(restMoment, perRow);
                        for (int k = 0; k < n3; k++)
                        {
                            df[k] -= restF[k];
                            dm[k] -= restM[k];
                        }
                    }
                    forceOut[d] = new BodyAeroTable(surfaceMach, alphaDeg, betaDeg, df, frame);
                    momentOut[d] = new BodyAeroTable(surfaceMach, alphaDeg, betaDeg, dm, frame);
                }
                try
                {
                    for (int d = 0; d < grid.Length; d++)
                    {
                        if (grid[d] != 0.0)
                            pose(grid[d]);
                        Delta(BodyAngleBreakpointsDeg, false, bodyCoefficients, bodyMoments, d, forceTables, momentTables);
                        if (p.NoseFirst)
                            Delta(NoseAngleBreakpointsDeg, true, noseCoefficients, noseMoments, d, noseForceTables, noseMomentTables);
                        if (bellySide >= 0)
                        {
                            DeltaFrame(BellyAlphaTableDeg, BellyBetaTableDeg, bellyFrame, bellyCoefficients, bellyMoments, d, bellyForceTables, bellyMomentTables);
                            DeltaSphere(d);
                            DeltaBand(d);
                        }
                    }
                }
                finally
                {
                    pose(0.0);
                }
                var table = new SurfaceTable
                {
                    Part = surface.Part, Name = surface.Name, MinDeg = surface.MinDeg, MaxDeg = surface.MaxDeg,
                    DeflectionDeg = grid, Force = forceTables, Moment = momentTables,
                    NoseForce = noseForceTables, NoseMoment = noseMomentTables,
                    BellyForce = bellyForceTables, BellyMoment = bellyMomentTables,
                    SphereForce = sphereForceTables, SphereMoment = sphereMomentTables,
                    BandForce = bandForceTables, BandMoment = bandMomentTables,
                    SweepMillis = surfaceClock.Elapsed.TotalMilliseconds,
                    PartKey = surfaceIndex < p.SurfaceKeys.Count ? p.SurfaceKeys[surfaceIndex] : "",
                };
                surfaceTables.Add(table);
                p.Progress?.SurfaceEnded(missing: false);
                log(FormattableString.Invariant($"[AFC] Control surface '{surface.Name}' of '{name}': {grid.Length - 1} deflections from {grid[0]:0} to {grid[^1]:0} deg, {table.SweepMillis:F0} ms; {SurfaceText(table, 0.4)}."));
            }
            System.Threading.Volatile.Write(ref res.Surfaces, surfaceTables.ToArray());
        }
        catch (Exception ex)
        {
            // The body table stands; only the surfaces are missing.
            log($"[AFC] Control surfaces of '{name}' could not be swept ({ex.GetType().Name}: {ex.Message}); the craft's body table stands.");
            p.Progress?.Note($"surfaces not swept ({ex.GetType().Name}: {ex.Message})");
        }
        return true;
    }

    /// <summary>The surface flow of a sweep's private mesh, turned to each sample direction before its rows. Nothing else steps it.</summary>
    private sealed class SamplingFlow
    {
        private readonly AeroSurfaceFlow _flow;

        public SamplingFlow(PartTree parts)
        {
            _flow = parts?.AwaitAerodynamics()?.GetSurfaceFlow();
        }

        public void Prepare(double3 airDirection)
        {
            // Stock reuses geometry within one degree. An opposite wind forces fresh geometry for the next sample.
            _flow?.Update(airDirection.ToBepu());
        }
    }

    /// <summary>
    /// A copy of one vehicle's physics state, with the air set per probe. The force read back is the game's own, from the same ComputeDerivatives the integrator calls, so any mod that patches the forces there is measured with them.
    /// The tree supplies the aero mesh: a private holder built for the sweep (KsaBoosterAeroMesh.TryBuildHolder), never the live craft's, whose AeroSurfaceFlow the vehicle's physics worker steps.
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

        public static bool TryCreate(BubbleOrigin origin, KinematicStates kinematic, PhysicsEnvironment environment, VehicleProperties props, PartTree parts, IParentBody parent, double density, double pressure, double temperature, double speedOfSound, out ForceProbe probe, out string error)
        {
            probe = new ForceProbe
            {
                _origin = origin,
                _kinematic = kinematic,
                _props = props,
                _environment = environment,
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
    /// <summary>A surface's moment coefficients at its largest deflection each way, retrograde at <paramref name="mach"/>, for the log.</summary>
    private static string SurfaceText(SurfaceTable t, double mach)
    {
        var sb = new StringBuilder();
        foreach (int d in new[] { 0, t.DeflectionDeg.Length - 1 })
        {
            if (t.DeflectionDeg[d] == 0.0)
                continue;
            t.Moment[d].Coefficients(mach, 0.0, 0.0, out double cl, out double cm, out double cn);
            if (sb.Length > 0) sb.Append("; ");
            sb.Append(FormattableString.Invariant($"at {t.DeflectionDeg[d]:0} deg dC_l/m/n {cl:0.0000}/{cm:0.0000}/{cn:0.0000} at Mach {mach:0.0}"));
        }
        return sb.ToString();
    }

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
    /// <param name="dtS">The physics step the game's limiters are worked for, s. 0 (the default) asks for the loads with no limiter. The game's own integrator calls the same function with its substep (KinematicMeasurements.DeltaTime), and ComputeAerodynamicLoads divides the moment by 1 + invInertiaBound dt q (0.5 dt stiffnessBound + dampingBound / V) - so at 0 this is not what the integrator applies.</param>
    public static bool TryLiveAero(Vehicle vehicle, out double3 forceBody, out double3 torqueBody, double dtS = 0.0)
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
            0.0, dtS, origin.PositionBub, origin.VelocityBub, default, default, parts);

        // The crawl: the airspeed scaled down to BaselineSpeed along the same direction, the body rates kept.
        float3 airBody = PhysicsStates.ComputeAirVelocityBody(in origin, in live, in environment);
        double3 air = double3.Unpack(in airBody);
        double airspeed = air.Length();
        if (!(airspeed > BaselineSpeed))
            return false;
        KinematicStates crawl = live;
        crawl.VelocityPhys = live.VelocityPhys - (air * (1.0 - BaselineSpeed / airspeed)).Transform(live.Body2Phys);
        Disturbances d0 = PhysicsStates.ComputeDerivatives(in origin, in crawl, in props, in environment,
            0.0, dtS, origin.PositionBub, origin.VelocityBub, default, default, parts);
        forceBody = d.ForceBody - d0.ForceBody;
        torqueBody = d.TorqueBody - d0.TorqueBody;
        return double.IsFinite(forceBody.X) && double.IsFinite(forceBody.Y) && double.IsFinite(forceBody.Z);
    }

    /// <summary>
    /// The game's moment limiter on the craft as it is now, per pascal (BodyAero6Dof.MomentLimiterPerPa): the air's moment with no limiter against the one the integrator applies at its own substep (KinematicMeasurements.DeltaTime), k = (M0 / Mdt - 1) / q. False where the air is too thin or the moment too small to read, or the ratio is not one a limiter could give.
    /// </summary>
    public static bool TryMomentLimiter(Vehicle vehicle, out double perPa, out double divisor)
    {
        perPa = divisor = double.NaN;
        double dt = vehicle?.KinematicMeasurements.DeltaTime ?? 0.0;
        if (!(dt > 0.0) || !TryLiveAero(vehicle, out _, out double3 m0) || !TryLiveAero(vehicle, out _, out double3 mdt, dt))
            return false;
        double mm = double3.Dot(mdt, mdt);
        if (!(mm > 1e4))   // 100 N m
            return false;
        double ratio = double3.Dot(m0, mdt) / mm;
        float3 air = PhysicsStates.ComputeAirVelocityBody(vehicle.BubbleOrigin, vehicle.KinematicStates, vehicle.PhysicsEnvironment);
        double v2 = double3.Unpack(in air).LengthSquared();
        double q = 0.5 * vehicle.PhysicsEnvironment.AtmosphericDensity * v2;
        if (!(q > 500.0) || !(ratio >= 1.0 && ratio < 100.0))
            return false;
        divisor = ratio;
        perPa = (ratio - 1.0) / q;
        return double.IsFinite(perPa);
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

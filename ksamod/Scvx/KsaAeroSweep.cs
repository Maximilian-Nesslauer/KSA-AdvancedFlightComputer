using System;
using System.Text;
using Brutal.Numerics;
using KSA;
using Navbox.Flight;

/// <summary>
/// Samples KSA's own aerodynamics onto a Cd(Mach, alpha) grid for the current
/// vehicle, and pairs it with an atmosphere that mirrors the game's.
///
/// THIS IS A MEASUREMENT, NOT A MODEL. Every Cd below comes out of the game's own
/// BoundingBoxCdA.ComputeCdA, called on the live vehicle's own AerodynamicCdABody -
/// so the surrogate cannot drift away from what the vehicle will actually fly
/// through, and a KSA update that changes the aero changes these numbers with it. We
/// re-derive nothing. The one thing added on top is the skin term, because it lives
/// in PhysicsStates.ComputeDrag rather than in ComputeCdA and there is no way to ask
/// the game for the two together.
///
/// WHAT THE GAME ACTUALLY HAS, since it shapes everything here:
///
///   F = (CdA(v_hat_body) + 0.1*S) * q,   q = 1/2 rho |v|^2,   applied at the CoM
///
///   CdA is a six-face box model - sum over axes of |v_hat_i| * Cd_i * A_i - with
///   Cd = 0.3 on the nose, 1.0 on the tail, 1.2 on each flank. It is a cosine blend
///   across faces, not an aerodynamic angle-of-attack law, and it is the ONLY
///   direction dependence there is.
///
///   There is no Mach number, no compressibility, no lift, no pitching moment (drag
///   acts through the centre of mass), no control surfaces and no wind.
///
/// So the Mach axis of this sweep comes back FLAT, by construction, and the code
/// below does not pretend otherwise - it samples one alpha profile and copies it
/// across every Mach row. The axis is kept because the surrogate is parameterised on
/// Mach for the solver's sake: the grid is already shaped for a transonic rise, so
/// the day KSA grows one, only this file changes.
///
/// THE SKIN TERM DOMINATES, and it is the single most surprising thing about KSA
/// aerodynamics. S is the vehicle's bounding-box SURFACE area, and 0.1*S is added to
/// CdA isotropically. For a 70 m x 3.7 m stack that is 105.8 m^2 against a nose-on
/// form CdA of 3.2 - thirty-three times larger. KSA drag is therefore very nearly
/// isotropic and proportional to bounding-box area, so slender-body intuition does
/// not apply and the Cd values here are much larger than an aerodynamicist would
/// expect. They are correct for this game.
/// </summary>
public static class KsaAeroSweep
{
    /// <summary>
    /// KSA's skin-drag coefficient: PhysicsStates.SkinDragCoefficient, multiplying
    /// the bounding-box surface area into an isotropic CdA increment.
    /// </summary>
    public const double SkinDragCoefficient = 0.1;

    /// <summary>Roll azimuths averaged over per alpha. 5 degree steps; the integrand
    /// is |cos| + |sin| weighted, so this is far finer than it needs to be and costs
    /// nothing at 72 evaluations per alpha.</summary>
    private const int RollSamples = 72;

    /// <summary>
    /// One sweep's worth of results: the fitted surrogate, the atmosphere it goes
    /// with, and everything needed to judge whether either is trustworthy.
    ///
    /// Immutable by construction and built entirely from copies. Once this exists it
    /// shares nothing with the game, which is what makes it safe to hand to a solver
    /// on another thread - the same argument Ksa6DofInputs makes for bias and inertia.
    /// </summary>
    public sealed class Result
    {
        /// <summary>The fitted Cd(Mach, alpha) surrogate.</summary>
        public AeroTable Table;

        /// <summary>The game's atmosphere, mirrored. Null if the body has none.</summary>
        public ExponentialAtmosphere Atmosphere;

        public double[] MachGrid;
        public double[] AlphaGridDeg;
        public double[] CdTable;

        /// <summary>Frontal area the Cd values are referenced to, m^2. This is KSA's
        /// own nose-face area, pi/4 * dy * dz - the same number the game uses for the
        /// X faces of its box model, so the reference is the game's rather than ours.</summary>
        public double ReferenceArea;

        /// <summary>Bounding-box surface area S, m^2 - the skin term's multiplier.</summary>
        public double SkinArea;

        /// <summary>Bounding-box extents along the assembly axes (x = long axis), m.</summary>
        public double3 BoxExtents;

        /// <summary>Vehicle mass at the moment of sampling, kg. Not used by the table;
        /// recorded so a stale sweep is recognisable.</summary>
        public double Mass;

        /// <summary>Headline Cd values, referenced to <see cref="ReferenceArea"/>.</summary>
        public double CdTailFirst, CdBroadside, CdNoseFirst;

        /// <summary>The pure form contribution AT ALPHA = 0, before the skin term, as
        /// a fraction of the total there. Small means that in the boostback attitude
        /// the drag is essentially all of KSA's isotropic skin term, so a few degrees
        /// of pointing error costs almost nothing.
        ///
        /// It says nothing about the rest of the range: broadside form drag is
        /// enormous whatever this is, because the flank area of a slender stack dwarfs
        /// its nose area. <see cref="AttitudeSensitivity"/> is the number for that.</summary>
        public double FormFraction;

        /// <summary>Cd(broadside) / Cd(tail-first): how much the drag actually varies
        /// across the whole attitude range. Around 4 for a slender booster, so the
        /// surrogate's alpha axis is carrying real information even though the drag is
        /// nearly attitude-independent close to alpha = 0.</summary>
        public double AttitudeSensitivity;

        /// <summary>
        /// How much Cd varies with ROLL at fixed alpha, as a fraction of the total, at
        /// its worst alpha. The table has no roll input, so it stores the azimuthal
        /// mean; this says how much that averaging threw away.
        ///
        /// IT DOES NOT GO TO ZERO FOR AN AXISYMMETRIC VEHICLE, which is the surprising
        /// part and worth knowing before reading the number. KSA's model is a BOX, not
        /// a body of revolution: the cross-flow term is |v_y|*A_y + |v_z|*A_z, so even
        /// with A_y == A_z a square-section booster rolled 45 degrees presents
        /// sqrt(2) times the area it presents at 0. For a slender stack that works out
        /// at roughly 25% and it is inherent, not a property of the airframe. Above
        /// about 35% the cross-section is genuinely not square as well.
        /// </summary>
        public double RollSpread;

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
    /// Sample the focused vehicle's aerodynamics and fit the surrogate.
    ///
    /// Main thread only: it reads Vehicle.Props, which the sim thread owns and
    /// rewrites. That is the same access the rest of the panel makes (TotalMass and
    /// friends), and it is why the result is a snapshot of plain arrays rather than
    /// anything that reaches back into the game.
    /// </summary>
    /// <returns>False with a reason in <paramref name="error"/> if the vehicle has no
    /// usable geometry. A body with no atmosphere is NOT an error - the aero table is
    /// still meaningful, and Atmosphere comes back null.</returns>
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

        ref readonly VehicleProperties props = ref vehicle.Props;

        // Extents along the ASSEMBLY axes. x is the long axis for any sane rocket -
        // it is the one KSA gives the streamlined 0.3/1.0 pair and the elliptical
        // cross-section, and the one the thrust axis lies along.
        //
        // Read through Vehicle's own accessor rather than off Props.BoundingBoxAsmb
        // directly: that field is a BepuPhysics.Box, and touching it would drag a
        // BepuPhysics reference into the mod for three floats. This is the same three
        // floats, and it is what the staleness check reads too, so the two cannot
        // disagree about which box the table was built from.
        float3 half = vehicle.BoundingBoxHalfExtentsAsmb;
        double dx = half.X * 2.0;
        double dy = half.Y * 2.0;
        double dz = half.Z * 2.0;

        // Reference area: KSA's own nose face. Not a convention we picked - it is
        // literally the A_x the game multiplies its 0.3 and 1.0 by.
        double refArea = Math.PI / 4.0 * dy * dz;
        double skinArea = props.TotalSurfaceArea;

        if (!(refArea > 0.0) || !double.IsFinite(refArea))
        {
            error = "vehicle has no bounding box yet";
            return false;
        }

        double[] machGrid = AeroTable.DefaultMachBreakpoints;
        double[] alphaDeg = AeroTable.DefaultAlphaBreakpointsDeg;
        int na = alphaDeg.Length;

        // --- the alpha profile, sampled once ---------------------------------
        // One profile, not one per Mach row: the game has no Mach dependence, so
        // sampling it na*nm times would be nm identical answers and a slower tab.
        var cdAlpha = new double[na];
        double rollSpread = 0.0;
        double formAtZero = 0.0;

        for (int j = 0; j < na; j++)
        {
            double alpha = alphaDeg[j] * Math.PI / 180.0;
            double sa = Math.Sin(alpha), ca = Math.Cos(alpha);

            // Average the form term over roll azimuth. KSA's two flank faces carry
            // the same Cd but different areas, so at fixed alpha the answer still
            // depends on which flank is into the wind; the surrogate has no roll
            // input, so the mean is what it can represent. RollSpread records what
            // that costs.
            double sum = 0.0, lo = double.MaxValue, hi = double.MinValue;
            for (int k = 0; k < RollSamples; k++)
            {
                double phi = 2.0 * Math.PI * k / RollSamples;

                // RETROGRADE-FIRST: alpha = 0 means the wind comes at the TAIL, so
                // the velocity in body axes points along -x. This is the sign that
                // carries the whole convention - see AeroTable.AngleOfAttack.
                var dir = new double3(-ca, sa * Math.Cos(phi), sa * Math.Sin(phi));

                // The game's own function, on the game's own coefficients.
                double cdA = props.AerodynamicCdABody.ComputeCdA(dir);
                sum += cdA;
                if (cdA < lo) lo = cdA;
                if (cdA > hi) hi = cdA;
            }

            double formCdA = sum / RollSamples;
            double totalCdA = formCdA + SkinDragCoefficient * skinArea;
            cdAlpha[j] = totalCdA / refArea;

            if (j == 0)
                formAtZero = formCdA / totalCdA;

            // Spread is judged against the TOTAL, since that is what the vehicle
            // feels - a big spread in a term that is 3% of the force is not a big
            // spread in the force.
            if (totalCdA > 0.0)
                rollSpread = Math.Max(rollSpread, (hi - lo) / totalCdA);
        }

        // --- broadcast across the (flat) Mach axis ----------------------------
        var cdTable = new double[machGrid.Length * na];
        for (int i = 0; i < machGrid.Length; i++)
            Array.Copy(cdAlpha, 0, cdTable, i * na, na);

        var res = new Result
        {
            MachGrid = machGrid,
            AlphaGridDeg = alphaDeg,
            CdTable = cdTable,
            ReferenceArea = refArea,
            SkinArea = skinArea,
            BoxExtents = new double3(dx, dy, dz),
            SampledExtents = new double3(dx, dy, dz),
            Mass = vehicle.TotalMass,
            SampledAt = simTime,
            FormFraction = formAtZero,
            RollSpread = rollSpread,
            CdTailFirst = cdAlpha[0],
            CdNoseFirst = cdAlpha[na - 1],
            CdBroadside = InterpolateAt(alphaDeg, cdAlpha, 90.0),
        };
        res.AttitudeSensitivity = res.CdTailFirst > 0.0
            ? res.CdBroadside / res.CdTailFirst
            : 0.0;

        try
        {
            res.Table = new AeroTable(machGrid, alphaDeg, cdTable);
        }
        catch (Exception ex)
        {
            error = "table fit failed: " + ex.Message;
            return false;
        }

        BuildAtmosphere(parent, res);

        result = res;
        return true;
    }

    /// <summary>
    /// Mirror the parent body's atmosphere, and CHECK the mirror against the game
    /// rather than assuming it.
    ///
    /// The check is the point. Three numbers copied across and an exponential written
    /// out again is exactly the kind of thing that is right until someone changes a
    /// unit, and the failure mode - a solver planning through slightly the wrong air -
    /// produces plans that are plausible and wrong rather than plans that break. So
    /// every resample re-verifies it against KSA's own
    /// GetAtmosphericDensityAtAltitude and records the worst disagreement.
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
        // IParentBody does not carry a name; Astronomical does, and every body
        // a vehicle can orbit is one.
        res.BodyName = (parent as Astronomical)?.Id ?? "";

        // Sample the whole column, including above the cutoff, so the boundary height
        // is checked too and not just the exponential.
        double worst = 0.0;
        double top = res.Atmosphere.TopAltitude;
        double gameTop = phys.Height;
        for (int i = 0; i <= 200; i++)
        {
            double alt = top * 1.05 * i / 200.0;
            double ours = res.Atmosphere.Density(alt);
            double theirs = phys.GetAtmosphericDensityAtAltitude(alt);

            // KSA zeroes density outside the boundary in PhysicsEnvironment rather
            // than inside GetAtmosphericDensityAtAltitude, which keeps returning the
            // exponential. Compare against the game's EFFECTIVE density, which is what
            // a vehicle experiences.
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
    /// The sampled table as CSV, for pasting into a plot or a regression test. Mach
    /// down the rows, alpha across - the same order the flat array is stored in.
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

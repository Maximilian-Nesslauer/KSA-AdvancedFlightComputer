using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;

namespace AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// The air an ascent flies through and the drag law it flies by, in SI, as <see cref="Dual"/>s so the dynamics can differentiate through them.
///
/// Abstract because two models use it. The game supplies KSA's own isothermal atmosphere and its sampled drag table (<see cref="KsaAscentAtmosphere"/>); the offline check supplies the atmosphere launch3dof.py uses, so the port can be compared against that script number for number.
/// </summary>
public abstract class AscentAtmosphere
{
    /// <summary>Density at a geometric altitude above the mean radius, kg/m^3.</summary>
    public abstract Dual Density(Dual altitude);

    /// <summary>Speed of sound at an altitude, m/s. Only the drag coefficient's Mach axis reads it.</summary>
    public abstract Dual SpeedOfSound(Dual altitude);

    /// <summary>Ambient pressure at an altitude, Pa: the engines' back pressure.</summary>
    public abstract Dual Pressure(Dual altitude);

    /// <summary>Drag coefficient at a Mach number, referenced to each stage's <see cref="AscentStage.DragArea"/>.</summary>
    public abstract Dual DragCoefficient(Dual mach);
}

/// <summary>
/// No air at all: an airless body. Density and pressure are zero everywhere, so drag, dynamic pressure and the q-alpha limit all drop out and every engine runs at its vacuum thrust.
/// </summary>
public sealed class VacuumAscentAtmosphere : AscentAtmosphere
{
    public static readonly VacuumAscentAtmosphere Instance = new();

    public override Dual Density(Dual altitude) => new(0.0);
    public override Dual SpeedOfSound(Dual altitude) => new(1.0);
    public override Dual Pressure(Dual altitude) => new(0.0);
    public override Dual DragCoefficient(Dual mach) => new(0.0);
}

/// <summary>
/// KSA's air: the game's isothermal exponential atmosphere and the drag table sampled off the vehicle.
///
/// NOSE-FIRST DRAG, whatever the attitude. launch3dof.py models drag as acting straight against the relative wind with a coefficient that depends on Mach alone, and this keeps that form: the coefficient is the table's value at alpha = 180 deg, which is nose-first in the table's retrograde-first convention. KSA drag hardly depends on attitude near there anyway - its skin term, which is isotropic, dominates (see KsaAeroSweep) - so a few degrees of angle of attack changes it very little.
///
/// The table is flat in Mach because KSA models no compressibility, so the transonic rise launch3dof.py has is absent here. The Mach axis is still read, so a table that grows one is picked up without any change.
/// </summary>
public sealed class KsaAscentAtmosphere : AscentAtmosphere
{
    private readonly ExponentialAtmosphere _atmosphere;
    private readonly AeroTable? _table;
    private readonly double _cd;

    /// <param name="atmosphere">The body's atmosphere, as KsaAeroSweep mirrors it.</param>
    /// <param name="table">Sampled drag table, or null to use <paramref name="constantCd"/>.</param>
    /// <param name="constantCd">Drag coefficient when there is no table.</param>
    public KsaAscentAtmosphere(ExponentialAtmosphere atmosphere, AeroTable? table, double constantCd = 0.0)
    {
        _atmosphere = atmosphere ?? throw new ArgumentNullException(nameof(atmosphere));
        _table = table;
        _cd = constantCd;
    }

    public ExponentialAtmosphere Atmosphere => _atmosphere;

    public override Dual Density(Dual altitude) => _atmosphere.Density(altitude);
    public override Dual SpeedOfSound(Dual altitude) => new(_atmosphere.SpeedOfSound);
    public override Dual Pressure(Dual altitude) => _atmosphere.Pressure(altitude);

    public override Dual DragCoefficient(Dual mach)
        => _table != null ? _table.Cd(mach, new Dual(_table.AlphaMaxRad)) : new Dual(_cd);
}

/// <summary>
/// One burn of the ascent, as launch3dof.py describes a stage: what it burns, what it drops when it is empty, and how hard it pushes.
///
/// THRUST CAN DEPEND ON BACK PRESSURE, which is the one place this departs from the script, where thrust is a constant per stage. KSA's engines lose thrust at sea level (and its nozzle model includes flow separation, so the loss is not linear in pressure), so a stage built from the game carries its thrust at a few pressures, and a spline through them gives the thrust at any altitude. The MASS FLOW does not depend on back pressure (DeLavalNozzleConfig.ComputePerformance), so the burn time to depletion is the same either way. A stage with no pressure table has constant thrust, which is exactly the script's model.
/// </summary>
public sealed class AscentStage
{
    private readonly CubicBSplineNd? _thrustVsPressure;

    /// <param name="thrust">Full-throttle thrust, N. Used when there is no pressure table, and as the reference thrust otherwise.</param>
    /// <param name="massFlow">Full-throttle propellant mass flow, kg/s.</param>
    /// <param name="propellantMass">Propellant this stage burns, kg.</param>
    /// <param name="jettisonMass">Structure dropped when this stage separates, kg. Zero for the last stage.</param>
    /// <param name="dragArea">Area the drag coefficient is referenced to, m^2.</param>
    /// <param name="pressureGrid">Strictly increasing back pressures, Pa, or null for constant thrust.</param>
    /// <param name="thrustAtPressure">Full-throttle thrust at each grid pressure, N.</param>
    /// <param name="throttleMin">This stage's throttle floor, a fraction of full thrust; NaN takes <see cref="AscentSettings.ThrottleMin"/>. A stage that cannot throttle - a solid motor - keeps the script's 99 % whatever the vehicle-wide floor is.</param>
    public AscentStage(double thrust, double massFlow, double propellantMass, double jettisonMass,
                       double dragArea, double[]? pressureGrid = null, double[]? thrustAtPressure = null,
                       double throttleMin = double.NaN)
    {
        if (!(thrust > 0.0) || !double.IsFinite(thrust))
            throw new ArgumentOutOfRangeException(nameof(thrust), thrust, "Thrust must be finite and positive.");
        if (!(massFlow > 0.0) || !double.IsFinite(massFlow))
            throw new ArgumentOutOfRangeException(nameof(massFlow), massFlow, "Mass flow must be finite and positive.");
        if (!(propellantMass > 0.0) || !double.IsFinite(propellantMass))
            throw new ArgumentOutOfRangeException(nameof(propellantMass), propellantMass, "Propellant must be finite and positive.");
        if (!(jettisonMass >= 0.0) || !double.IsFinite(jettisonMass))
            throw new ArgumentOutOfRangeException(nameof(jettisonMass), jettisonMass, "Jettisoned mass must be finite and non-negative.");
        if (!(dragArea >= 0.0) || !double.IsFinite(dragArea))
            throw new ArgumentOutOfRangeException(nameof(dragArea), dragArea, "Drag area must be finite and non-negative.");

        Thrust = thrust;
        MassFlow = massFlow;
        PropellantMass = propellantMass;
        JettisonMass = jettisonMass;
        DragArea = dragArea;
        if (!double.IsNaN(throttleMin) && !(throttleMin > 0.0 && throttleMin <= 1.0))
            throw new ArgumentOutOfRangeException(nameof(throttleMin), throttleMin, "The throttle floor must be in (0, 1].");
        ThrottleMin = throttleMin;

        if (pressureGrid != null && thrustAtPressure != null && pressureGrid.Length >= 2)
        {
            if (thrustAtPressure.Length != pressureGrid.Length)
                throw new ArgumentException("One thrust per grid pressure.", nameof(thrustAtPressure));
            foreach (double t in thrustAtPressure)
                if (!(t > 0.0) || !double.IsFinite(t))
                    throw new ArgumentException("Every tabulated thrust must be finite and positive.", nameof(thrustAtPressure));
            PressureGrid = (double[])pressureGrid.Clone();
            ThrustAtPressure = (double[])thrustAtPressure.Clone();
            // Linear edges: a pressure past the table (a pad below the mean radius, say) extrapolates along the slope rather than flattening, so the derivative stays continuous.
            _thrustVsPressure = CubicBSplineNd.Fit(PressureGrid, ThrustAtPressure, EdgeMode.Linear);
        }
    }

    public double Thrust { get; }
    public double MassFlow { get; }
    public double PropellantMass { get; }
    public double JettisonMass { get; }
    public double DragArea { get; }
    /// <summary>This stage's own throttle floor, or NaN for the problem's.</summary>
    public double ThrottleMin { get; }
    public double[]? PressureGrid { get; }
    public double[]? ThrustAtPressure { get; }

    /// <summary>Exhaust velocity at the reference thrust, m/s - for the dV readout only.</summary>
    public double ExhaustVelocity => Thrust / MassFlow;

    /// <summary>Full-throttle burn time, s.</summary>
    public double FullBurnTime => PropellantMass / MassFlow;

    /// <summary>Full-throttle thrust at a back pressure, N.</summary>
    public Dual MaxThrust(Dual pressure)
    {
        if (_thrustVsPressure == null)
            return new Dual(Thrust);
        Span<double> p = stackalloc double[1] { pressure.V };
        Span<double> value = stackalloc double[1];
        Span<double> grad = stackalloc double[1];
        _thrustVsPressure.EvaluateWithGradient(p, value, grad);
        return new Dual(value[0], grad[0] * pressure.D);
    }
}

/// <summary>
/// Tuning of the ascent SCvx, with launch3dof.py's values as the defaults. Scales and tolerances are in the script's canonical units (length = body radius, mu = 1, mass = lift-off mass) except where a name says seconds or metres.
///
/// THE STATE TRUST SCALES ARE THE SCRIPT'S, CARRIED TO OTHER BODIES BY THE CANONICAL UNITS. The script sets 500 km and 1000 m/s for Earth; here they are that fraction of the body radius and of the canonical speed, which is the same thing on Earth and shrinks sensibly on a small moon, where a 500 km box would never bind.
/// </summary>
public sealed class AscentSettings
{
    public int NodesPerStage = 60;

    /// <summary>
    /// Throttle floor as a fraction of full thrust, for every stage that does not set its own. The script's 99 %.
    ///
    /// NOT 100 %. The floor is imposed as the halfspace tangent to it in the reference thrust direction, and together with the ceiling cone that leaves a spherical cap: at 99 % the thrust can turn up to arccos(0.99) = 8.1 deg per iteration. At 100 % the cap closes to a point and the thrust direction could never move from the seed's.
    /// </summary>
    public double ThrottleMin = 0.99;

    /// <summary>Fraction of the q and q-alpha limits the linearised constraints enforce.</summary>
    public double PathBackoff = 0.98;

    /// <summary>q and q-alpha are imposed only where the reference q exceeds this, Pa.</summary>
    public double QActive = 100.0;

    // Trust region and defect scales (launch3dof.py XS_POS, XS_VEL, Dscale, SIG_SCALE).
    public double PositionTrust = 500e3 / 6378137.0;
    public double VelocityTrust = 1000.0 / Math.Sqrt(3.986004418e14 / 6378137.0);
    public double PositionDefectScale = 0.05;
    public double SigmaTrustSeconds = 300.0;
    public double SigmaMinSeconds = 20.0;
    public double SigmaMaxSeconds = 700.0;

    // Weights.
    public double RhoVirtualControl = 1e5;
    public double RhoTerminal = 1e5;
    public double RhoPath = 1.0;
    public double SmoothingWeight = 4e-3;

    // Convergence.
    public double PredictedTolerance = 1e-5;
    public double DefectTolerance = 1e-4;
    public double TerminalTolerance = 2e-5;
    public double PathTolerance = 2e-3;
    public int MaxIterations = 350;

    // Trust-region schedule.
    public double TrustInitial = 0.10;
    public double TrustMin = 2e-3;
    public double TrustMax = 0.30;
    public double RhoAccept = 0.0;
    public double RhoShrink = 0.25;
    public double RhoGrow = 0.7;
    public double Shrink = 0.5;
    public double Grow = 1.5;
    public int StallMax = 6;

    // Seed: a flown gravity turn with a searched kick angle.
    public double PitchOverSpeed = 130.0;       // m/s, air-relative
    public double FinalBurnFraction = 0.8;

    /// <summary>
    /// End the seed's last burn when the orbital energy reaches the target's, rather than after FinalBurnFraction of its load (the script). See AscentSeed.Build. Off reproduces the script's seed exactly.
    /// </summary>
    public bool SeedEnergyCutoff = true;

    /// <summary>Seeds whose path violation is within this fraction of the least any seed manages (plus as much again) tie on it; see AscentSeed.Search. Zero is the script's cost.</summary>
    public double SeedPathBand = 0.1;

    /// <summary>launch3dof.py's own seed: the last stage burns 80 % of its load, and the kick is scanned linearly over 1-4 deg. It reproduces the script's run iteration for iteration; the default converges to the same optimum from a better start.</summary>
    public AscentSettings UseScriptSeed()
    {
        SeedEnergyCutoff = false;
        SeedPathBand = 0.0;
        KickMinDeg = 1.0;
        KickMaxDeg = 4.0;
        KickScan = 13;
        KickScanLog = false;
        return this;
    }
    public int SeedSubsteps = 20;
    /// <summary>
    /// The kick scan. The script scans 1 to 4 deg linearly, which brackets a Saturn V; a KSA stack at twice its thrust-to-weight needs ten times the kick to turn over at all before its first stage burns out, so the default scans 0.25 to 30 deg on a log scale, fine where small kicks live and coarse where large ones do. <see cref="UseScriptSeed"/> restores the script's.
    /// </summary>
    public double KickMinDeg = 0.25;
    public double KickMaxDeg = 30.0;
    public int KickScan = 25;
    public bool KickScanLog = true;
    public double KickToleranceDeg = 1e-3;
    public double BadSeedCost = 1e3;

    /// <summary>
    /// How many times the kick scan may slide its window when the best sample lands on an edge. The script only warns there; sliding keeps the scan usable for a vehicle whose best kick is outside 1-4 deg, and never runs for one whose best kick is inside it, so the script's own case is unchanged.
    /// </summary>
    public int KickWindowSlides = 4;

    /// <summary>Clarabel tolerance and iteration cap per subproblem. CVXPY's defaults for Clarabel, which are Clarabel's own.</summary>
    public double SubproblemEps = 1e-8;
    public int SubproblemMaxIterations = 200;
}

/// <summary>
/// The minimum-propellant ascent, in SI, in a body-centred inertial frame whose +z is the spin axis - KSA's CCI, and the script's ECI.
///
/// Five insertion conditions: the radius, the speed, r . v, and the two plane conditions. The script targets a circular orbit, where r . v is zero; <see cref="TargetRadialRate"/> generalises it to any point on an ellipse, so an elliptical target orbit is inserted into at a point with its own flight-path angle. Where along the orbit the vehicle arrives is left free, exactly as in the script.
/// </summary>
public sealed class AscentProblem
{
    public required double Mu { get; init; }
    /// <summary>Mean radius: the canonical length unit and the atmosphere's datum.</summary>
    public required double BodyRadius { get; init; }
    /// <summary>Spin rate about +z, rad/s. The atmosphere co-rotates.</summary>
    public required double Omega { get; init; }

    /// <summary>Lift-off position and velocity, inertial, m and m/s.</summary>
    public required double[] R0 { get; init; }
    public required double[] V0 { get; init; }
    /// <summary>Lift-off mass, kg: every stage's propellant, every jettison, and what reaches orbit.</summary>
    public required double M0 { get; init; }

    public required AscentStage[] Stages { get; init; }
    public required AscentAtmosphere Atmosphere { get; init; }

    /// <summary>Radius the trajectory must stay above, m.</summary>
    public required double GroundRadius { get; init; }

    public required double TargetRadius { get; init; }
    public required double TargetSpeed { get; init; }
    /// <summary>r . v at insertion, m^2/s: r v sin(flight-path angle). Zero for a circular orbit or at an apsis.</summary>
    public double TargetRadialRate { get; init; }
    /// <summary>Unit normal of the target plane, along r x v.</summary>
    public required double[] PlaneNormal { get; init; }

    public double QMax { get; init; } = 35000.0;
    public double QAlphaMax { get; init; } = 3500.0;

    /// <summary>
    /// Straight up at lift-off: the first thrust is pinned parallel to this. Null takes the lift-off air-relative velocity, as the script does.
    /// </summary>
    public double[]? LiftoffDirection { get; init; }

    public AscentSettings Settings { get; init; } = new();

    /// <summary>What is left when the last stage is empty, kg: the mass floor.</summary>
    public double FinalMassFloor
    {
        get
        {
            double m = M0;
            for (int i = 0; i < Stages.Length; i++)
            {
                m -= Stages[i].PropellantMass;
                if (i < Stages.Length - 1)
                    m -= Stages[i].JettisonMass;
            }
            return m;
        }
    }

    /// <summary>Why this problem cannot be solved, or empty if it can.</summary>
    public string Validate()
    {
        if (!(Mu > 0.0) || !(BodyRadius > 0.0) || !double.IsFinite(Omega))
            return "the body is not valid";
        if (R0 is not { Length: 3 } || V0 is not { Length: 3 } || PlaneNormal is not { Length: 3 })
            return "the lift-off state is not valid";
        foreach (double x in R0.Concat(V0).Concat(PlaneNormal))
            if (!double.IsFinite(x))
                return "the lift-off state is not finite";
        if (Stages == null || Stages.Length == 0)
            return "there are no stages";
        if (!(M0 > 0.0))
            return "the lift-off mass is not positive";
        if (!(FinalMassFloor > 0.0))
            return $"the stages burn and drop more than the vehicle weighs ({FinalMassFloor:F0} kg left)";
        if (!(TargetRadius > BodyRadius) || !(TargetSpeed > 0.0))
            return "the target orbit is not above the surface";
        if (!(QMax > 0.0) || !(QAlphaMax > 0.0))
            return "the structural limits must be positive";
        return "";
    }
}

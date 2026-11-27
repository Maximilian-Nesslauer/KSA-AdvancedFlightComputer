#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using System.Collections.Generic;
using BepuUtilities;
using Brutal.Numerics;
using KSA;
using AdvancedFlightComputer.Features.AeroControl;
using AdvancedFlightComputer.Features.RcsTorque;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// Builds the 6-DOF solver's configuration from the LIVE vehicle and body, instead of the 6dof.py mirror values Scvx6DofConfig defaults to.
///  Everything here is measured, not assumed - the defaults describe a Super Heavy-class booster landing on a 9.81 m/s^2 world, which is not what is being flown. Getting these wrong does not error: the solver happily plans a perfectly feasible trajectory for the wrong vehicle.
/// </summary>
public static class Ksa6DofSetup
{
    /// <summary>
    /// What the last <see cref="TryBuild"/> assumed about the air, for the UI.
    ///  These are diagnostics, not inputs - nothing reads them back. They exist because the vacuum-thrust bug was invisible: the solver plans a perfectly feasible trajectory for a vehicle with 25% more thrust than it has, and the only symptom is that the vehicle slowly loses the altitude fight. Showing the ratio makes the assumption checkable at a glance instead.
    /// </summary>
    public static double LastAmbientPressurePa { get; private set; }

    /// <summary>
    /// The scale 6-DOF prices a velocity jump on, m/s, in place of the speed at engage (Scvx6DofConfig.VirtualControlScale). The 3-DOF keeps XScale; see Ksa3DofSetup.Config.
    ///  Below this size per interval, faking braking with virtual control is cheaper than arriving that much faster: RhoVc (v/D)^2 against the terminal speed's TerminalSpeedWeight v / XScale, so up to TerminalSpeedWeight D^2 / (RhoVc XScale) per interval. Priced on a ~95 m/s engage speed that was about 9 m/s per interval, and on the 2026-10-09 7 t booster a plan braked 0.18 m/s an interval on nothing; at 5 m/s it is about 0.03.
    ///  Not tighter, because at convergence the virtual control is the defect, collocation error included: on the same booster the trapezoid's own error at 0.6 s spacing is a few tenths of a m/s an interval, and a scale near the flight gate's 0.5 m/s would price that hundreds of times above the fuel term and have the optimiser shape trajectories around the mesh.
    /// </summary>
    public const double VirtualControlVelocityScaleMs = 5.0;

    /// <summary>
    /// The scale position's virtual control is priced on once a plan is refined, m (the pricing stage ramps it from XScale, as velocity's): 5 m, Maxi's measurement in #265. Left on XScale, about 2 km on a Starship's 15 s coast, a metre of phantom displacement cost next to nothing and its re-solves carried 1-2 m of it into the intervals the 1 m gate holds, refused 15 times in a row (2026-11-24). Offline in #265 the same change took 10-15 s landings' misses from 2-4 m to 0.2-0.8 m at about the same fuel; tight from a cold solve's first iteration it cost 1.3 t, which the pricing stage's loose start keeps out.
    /// </summary>
    public const double VirtualControlPositionScaleM = 5.0;

    /// <summary>The progressive glideslope's first height, at least, m: a craft shorter than this still comes down its last 20 m in the narrowest part.</summary>
    internal const double GlideFunnelMinNearM = 20.0;

    /// <summary>The progressive glideslope's two upper heights, m (the user's, 2026-11-27): its half-angles from vertical are set at one craft length, here, and here.</summary>
    internal const double GlideFunnelMidM = 300.0, GlideFunnelTopM = 1000.0;

    /// <summary>The progressive glideslope's heights and half-angles the last build used. For the log and the drawings.</summary>
    public static double[] LastGlideFunnelHeightsM { get; private set; }
    public static double[] LastGlideFunnelHalfAngleDeg { get; private set; }

    /// <summary>
    /// THE PROGRESSIVE GLIDESLOPE'S HEIGHTS (Scvx6DofConfig.GlideFunnelHeightsM): one craft length over the target - its bounding box's longest side, at least GlideFunnelMinNearM - then GlideFunnelMidM and GlideFunnelTopM, with the half-angles from vertical the craft's settings give for each. A length at or above the next height drops that height. Null when the first half-angle is 0 (off).
    /// </summary>
    internal static void GlideFunnelKnots(Vehicle vehicle, double nearDeg, double midDeg, double topDeg, out double[] heightsM, out double[] halfAngleDeg)
    {
        heightsM = null;
        halfAngleDeg = null;
        if (!(nearDeg > 0.0))
            return;
        float3 half = vehicle.BoundingBoxHalfExtentsAsmb;
        double length = 2.0 * Math.Max(half.X, Math.Max(half.Y, half.Z));
        double near = Math.Max(GlideFunnelMinNearM, double.IsFinite(length) ? length : 0.0);
        var h = new List<double> { near };
        var a = new List<double> { nearDeg };
        if (midDeg > 0.0 && GlideFunnelMidM > near) { h.Add(GlideFunnelMidM); a.Add(midDeg); }
        if (topDeg > 0.0 && GlideFunnelTopM > h[^1]) { h.Add(GlideFunnelTopM); a.Add(topDeg); }
        heightsM = h.ToArray();
        halfAngleDeg = a.ToArray();
    }

    /// <summary>
    /// The roll acceleration at full thrust, rad/s^2, under which the gimbals count as giving no roll torque and the plan leaves the roll free (Scvx6DofConfig.TerminalRollFree). 0.002 rad/s^2 takes 40 s at full thrust to remove a roll rate of 0.08 rad/s, so a craft under it cannot stop the roll it enters with during a touchdown burn. One gimballed engine on the thrust line gives a thousandth of this or less, engines on a ring a hundred times more.
    /// </summary>
    public const double RollAuthorityFloorRadS2 = 0.002;

    // The smallest roll torque the model is given, N m. The solver scales the roll torque by Scvx6DofConfig.TauRollMax, so the figure must stay above zero for a craft whose gimbals give none.
    internal const double ModelRollTorqueFloorNm = 1.0e-3;

    /// <summary>
    /// The roll part of the model for gimbals that give <paramref name="tauRoll"/> N m at full thrust, and RCS that gives <paramref name="rcsRollNm"/> N m either way, about a roll inertia of <paramref name="izz"/>: whether the plan leaves the roll free, and the gimbals' roll torque bound. The roll is held when the two together can stop it; the RCS's roll is its own control (Scvx6DofConfig.RcsMaxPosNm), not part of the bound. A choice made for an earlier plan of the same landing comes in as <paramref name="keptRollFree"/> and stands, because the roll inertia falls with the propellant and a craft near the floor would otherwise change the end condition between two plans.
    /// </summary>
    internal static void RollModel(double tauRoll, double izz, bool? keptRollFree, out bool rollFree, out double modelTauRoll, double rcsRollNm = 0.0)
    {
        rollFree = keptRollFree ?? tauRoll + Math.Max(rcsRollNm, 0.0) < RollAuthorityFloorRadS2 * izz;
        modelTauRoll = Math.Max(tauRoll, ModelRollTorqueFloorNm);
    }

    /// <summary>Whether the RCS turns the craft both ways about pitch and yaw (model x and y), so a plan can point its thrust with it alone.</summary>
    internal static bool RcsSteers(Dynamics6Dof.RcsModel rcs)
    {
        if (rcs?.MaxPosNm is not { Length: >= 2 } pos || rcs.MaxNegNm is not { Length: >= 2 } neg)
            return false;
        return pos[0] > 0.0 && neg[0] > 0.0 && pos[1] > 0.0 && neg[1] > 0.0;
    }

    /// <summary>The RCS's roll torque the weaker way, N m; 0 without RCS roll.</summary>
    internal static double RcsRollNm(Dynamics6Dof.RcsModel rcs)
        => rcs?.MaxPosNm is { Length: >= 3 } pos && rcs.MaxNegNm is { Length: >= 3 } neg ? Math.Max(Math.Min(pos[2], neg[2]), 0.0) : 0.0;
    public static double LastVacuumThrustN { get; private set; }
    public static double LastPressureThrustN { get; private set; }

    /// <summary>
    /// Inertia about the model's body axes, in kg m^2.
    ///  KSA's Vehicle.TotalMassPropsBody.Inertia is a FULL symmetric tensor about the centre of mass and is LIVE - it tracks propellant drain, so this must be re-read per plan rather than captured once. The model wants a diagonal, so we project onto the model body axes and drop the off-diagonal terms. That is defensible rather than lazy: KSA's own UpdateTvcParams uses only the diagonal too, so the game's control model already ignores those terms.
    /// </summary>
    public static void Inertia(Vehicle vehicle, out double ixx, out double iyy, out double izz)
    {
        Inertia(vehicle, out ixx, out iyy, out izz, out _, out _);
    }

    /// <summary>
    /// As above, plus the two numbers that say whether the diagonal approximation is actually valid for this vehicle.
    ///  The model wants a DIAGONAL inertia; KSA supplies a full symmetric tensor. For a genuinely axisymmetric booster - thrust axis along the symmetry axis, centre of mass on it - the tensor in the model's body frame is exactly diag(It, It, Ia), so dropping the off-diagonals costs NOTHING and the transverse terms are equal.
    /// That is a property of the vehicle, not an assumption we are entitled to make, so both are measured:
    ///  offDiagonalRatio  max|off-diagonal| / max|diagonal|. Near 0 means the model frame IS a principal-axis frame and the approximation is exact. Large means real coupling is being discarded.
    ///   transverseAsymmetry |Ixx-Iyy| / max(Ixx,Iyy). Near 0 confirms axisymmetry - and if it holds, the arbitrary roll reference that BodyAxes picks is harmless, because the transverse inertia is degenerate and ANY perpendicular pair are principal axes.
    /// </summary>
    public static void Inertia(Vehicle vehicle, out double ixx, out double iyy, out double izz,
                               out double offDiagonalRatio, out double transverseAsymmetry)
    {
        Symmetric3x3 t = vehicle.TotalMassPropsBody.Inertia;
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);

        // a' I b, with Symmetric3x3's YX/ZX/ZY being Ixy/Ixz/Iyz.
        double Bilinear(double3 a, double3 b) =>
            t.XX * a.X * b.X + t.YY * a.Y * b.Y + t.ZZ * a.Z * b.Z +
            t.YX * (a.X * b.Y + a.Y * b.X) +
            t.ZX * (a.X * b.Z + a.Z * b.X) +
            t.ZY * (a.Y * b.Z + a.Z * b.Y);

        ixx = Bilinear(mx, mx);
        iyy = Bilinear(my, my);
        izz = Bilinear(mz, mz);

        double maxDiag = Math.Max(Math.Abs(ixx), Math.Max(Math.Abs(iyy), Math.Abs(izz)));
        double maxOff = Math.Max(Math.Abs(Bilinear(mx, my)),
                        Math.Max(Math.Abs(Bilinear(mx, mz)), Math.Abs(Bilinear(my, mz))));
        offDiagonalRatio = maxDiag > 0.0 ? maxOff / maxDiag : 0.0;

        double maxTransverse = Math.Max(Math.Abs(ixx), Math.Abs(iyy));
        transverseAsymmetry = maxTransverse > 0.0 ? Math.Abs(ixx - iyy) / maxTransverse : 0.0;
    }

    /// <summary>
    /// How far off vertical the vehicle is RIGHT NOW, in degrees, from the model quaternion - the same r22 the subproblem's tilt cone is written against, so the two cannot disagree about what "tilt" means.
    /// </summary>
    public static double EntryTiltDeg(double[] x0)
    {
        if (x0 == null || x0.Length < 14) return 0.0;
        double qx = x0[Dynamics6Dof.IQ + 1], qy = x0[Dynamics6Dof.IQ + 2];
        double r22 = Math.Clamp(1.0 - 2.0 * (qx * qx + qy * qy), -1.0, 1.0);
        return Math.Acos(r22) * 180.0 / Math.PI;
    }

    /// <summary>
    /// Distance from the centre of mass to the main engine's thrust point, measured ALONG the thrust axis. This is the model's L_arm: it sets how much pitch/yaw torque a given gimbal deflection produces (tau = r_T x T_body).
    ///  Taken from the highest-thrust gimbal that KsaGimbalControl.CountsForPlanning accepts, the same one KsaFrameBridge derives the body axes from, so the two cannot disagree about which engine they mean. An unstaged upper engine sits on the far side of the centre of mass and would give a meaningless arm.
    /// </summary>
    public static double EngineArm(Vehicle vehicle)
    {
        KsaFrameBridge.BodyAxes(vehicle, out _, out _, out double3 mz);
        // Vehicle's own CoM, not the FlightComputer's copy: the FC field is only refreshed by ReadMeasurements, so at engage time it can still be zero - which would silently measure the engine arm from the assembly origin instead of the centre of mass and hand the solver a wrong LArm.
        float3 com = vehicle.CenterOfMassAsmbF;

        double best = 0.0, arm = 0.0;
        foreach (GimbalController gc in KsaGimbalControl.PlanningGimbals(vehicle))
        {
            if (gc.Data.MaximumThrust <= best)
                continue;
            best = gc.Data.MaximumThrust;
            // To the gimbal's PIVOT, which a deflected nozzle swings about: the model's torque is LArm times the side force, and the side force acts on the pivot's arm, not the thrust point's (see KsaTvcAllocator.FillColumns).
            double3 d = KsaTvcAllocator.GimbalPivot(gc) - new double3(com.X, com.Y, com.Z);
            // Negative because the engine sits at -Z in the model's body frame; the model wants the magnitude of that offset.
            arm = Math.Abs(double3.Dot(d, mz));
        }
        return arm;
    }

    /// <summary>
    /// The moment the engines the plan runs make about the centre of mass with the gimbals at rest, per newton of their thrust, MODEL body axes (KsaFrameBridge.BodyAxes), N m per N: Dynamics6Dof.Params.ThrustMoment*. The engines are the activated ones, or every one when none is, as KsaGimbalControl.CountsForPlanning counts the gimbals.
    ///  WHY. A landing on part of a stage's engines (LandingEngineSelection) can put the thrust off the centre of mass: one engine of a ring 0.8 m out on a 10 m arm needs about 4.6 deg of a 6 deg gimbal just to hold it. Without this the plan took the whole cone as free and the gimbals had nothing left for its turns. The flight side needs no change: the allocator works from rest, so the torque it is asked for adds to this moment as the model has it.
    /// </summary>
    public static double3 ThrustMomentPerNewton(Vehicle vehicle)
    {
        Span<EngineController> all = vehicle.Parts.Modules.Get<EngineController>();
        bool anyActive = false;
        foreach (EngineController engine in all)
            anyActive |= engine.IsActive;
        var engines = new List<EngineController>();
        foreach (EngineController engine in all)
            if (!anyActive || engine.IsActive)
                engines.Add(engine);
        (double3 moment, double3 force) = KsaThrustMoment.Rest(vehicle, engines);
        double thrust = force.Length();
        if (!(thrust > 0.0))
            return double3.Zero;
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);
        var perNewton = new double3(double3.Dot(moment, mx), double3.Dot(moment, my), double3.Dot(moment, mz)) / thrust;
        return double.IsFinite(perNewton.X) && double.IsFinite(perNewton.Y) && double.IsFinite(perNewton.Z) ? perNewton : double3.Zero;
    }

    /// <summary>
    /// The thrust the capability figures below are evaluated at: every planning gimbal at its Data.MaximumThrust. KSA computes that at full throttle and in vacuum (GimbalController.RecomputeStaticData), more than the engines deliver in air or throttled down, so a capability is only meaningful per newton of this - the gimbals deliver torque in proportion to the thrust actually through them.
    /// </summary>
    public static double PlanningThrust(Vehicle vehicle)
    {
        double total = 0.0;
        foreach (GimbalController gc in KsaGimbalControl.PlanningGimbals(vehicle))
            total += gc.Data.MaximumThrust;
        return total;
    }

    /// <summary>
    /// How fast the dominant planning gimbal slews, as full deflections a second (Scvx6DofConfig.GimbalSlewPerS): Gimbal's own rate cap, RATE_CAP_PER_NATURAL_FREQUENCY of its range per unit of natural frequency. 0 where there is no gimbal to read.
    /// </summary>
    internal static double GimbalSlewPerS(Vehicle vehicle)
    {
        double thrust = 0.0, frequency = 0.0;
        foreach (GimbalController controller in KsaGimbalControl.PlanningGimbals(vehicle))
        {
            Gimbal gimbal = controller.Gimbal;
            if (!(controller.Data.MaximumThrust > thrust)
                || Math.Max(gimbal.AxisY.MaxAngle, gimbal.AxisZ.MaxAngle) <= 0.0)
                continue;
            thrust = controller.Data.MaximumThrust;
            frequency = gimbal.NaturalFrequency;
        }
        return double.IsFinite(frequency) && frequency > 0.0 ? frequency * Gimbal.RATE_CAP_PER_NATURAL_FREQUENCY : 0.0;
    }

    internal static double LateralTorqueLead(Vehicle vehicle)
    {
        double thrust = 0.0;
        double frequency = 0.0;
        foreach (GimbalController controller in KsaGimbalControl.PlanningGimbals(vehicle))
        {
            Gimbal gimbal = controller.Gimbal;
            if (!(controller.Data.MaximumThrust > thrust)
                || Math.Max(gimbal.AxisY.MaxAngle, gimbal.AxisZ.MaxAngle) <= 0.0)
                continue;
            thrust = controller.Data.MaximumThrust;
            frequency = gimbal.NaturalFrequency;
        }

        // Use the ramp delay of the dominant gimbal's critically damped response.
        return double.IsFinite(frequency) && frequency > 0.0 ? Math.Min(2.0 / frequency, 0.2) : 0.0;
    }

    /// <summary>
    /// The thrust of every engine the plan runs, at the conditions PlanningThrust is taken at: the planning gimbals' Data.MaximumThrust plus the engines that sit on no gimbal, through the same nozzle evaluation (GimbalController.RecomputeStaticData, full throttle in vacuum). The model's Tmax is the thrust of all activated engines, and only the part of it that goes through a gimbal makes torque, so the capability per newton of the model's thrust is the allocator's capability over this figure. Over PlanningThrust alone, a craft with one gimballed engine among seven planned with the gimbal range of that engine on the thrust of all seven, about 3.7 times the torque its gimbal gives.
    /// An engine without a gimbal counts when it is activated, or when no gimballed engine is activated, as KsaGimbalControl.CountsForPlanning counts the gimbals. RCS thrusters are rockets too and are left out by their controller type.
    /// </summary>
    public static double PlanningEngineThrust(Vehicle vehicle)
    {
        Span<GimbalController> all = vehicle.Parts.Modules.Get<GimbalController>();
        bool anyActive = KsaGimbalControl.AnyEngineActive(all);
        var onGimbal = new HashSet<RocketNozzle>();
        double total = 0.0;
        foreach (GimbalController gc in all)
        {
            if (!KsaGimbalControl.CountsForPlanning(gc, anyActive))
                continue;
            total += gc.Data.MaximumThrust;
            foreach (RocketNozzle nozzle in gc.Data.Nozzles)
                onGimbal.Add(nozzle);
        }

        foreach (RocketNozzle nozzle in vehicle.Parts.Modules.Get<RocketNozzle>())
        {
            Rocket rocket = nozzle.Rocket;
            if (rocket?.Controller is not EngineController engine || nozzle.Core == null || onGimbal.Contains(nozzle))
                continue;
            if (anyActive && !engine.IsActive)
                continue;
            RocketCoreConditions combustion = nozzle.Core.ComputeConditions(1f);
            double thrust = nozzle.ComputePerformance(in combustion, 0f).GetTotalThrust();
            if (double.IsFinite(thrust) && thrust > 0.0)
                total += thrust;
        }
        return total;
    }

    /// <summary>
    /// What the craft's RCS can give touchdown guidance, in MODEL body axes (KsaFrameBridge.BodyAxes), measured by the allocator that will fly it (KsaRcsTorqueControl.TryCharacterise): per one-sided control the largest torque, the propellant per newton metre and the side force per newton metre. Null for a craft with no usable thruster.
    /// The side force is the point on a booster: its thrusters sit at the top, far from the centre of mass, so every newton metre they give comes with a push of roughly 1/arm newtons sideways, which the plan must carry or it will drift off what it planned.
    /// </summary>
    public static Dynamics6Dof.RcsModel RcsModel(Vehicle vehicle)
    {
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);
        // Six measurements, each axis each way (KsaRcsTorqueControl.TryCharacterise), two per RCS control.
        Span<RcsTorqueAllocator.AxisAuthority> six = stackalloc RcsTorqueAllocator.AxisAuthority[2 * Dynamics6Dof.NRCS];
        if (!KsaRcsTorqueControl.TryCharacterise(vehicle, mx, my, mz, six))
            return null;
        return RcsModelFrom(six);
    }

    /// <summary>The plan's RCS model from the allocator's six one-way measurements (+x, -x, +y, -y, +z, -z): each axis's two bounds, and the side force per newton metre as the average of what the two ways bring - force odd in the torque, exact for a symmetric layout. Null when no direction has any authority.</summary>
    internal static Dynamics6Dof.RcsModel RcsModelFrom(ReadOnlySpan<RcsTorqueAllocator.AxisAuthority> six)
    {
        static bool Good(RcsTorqueAllocator.AxisAuthority a)
            => a.MaxTorqueNm > 0.0 && double.IsFinite(a.MaxTorqueNm) && double.IsFinite(a.FlowPerNm)
               && double.IsFinite(a.ForcePerNmX) && double.IsFinite(a.ForcePerNmY) && double.IsFinite(a.ForcePerNmZ);
        var model = new Dynamics6Dof.RcsModel();
        for (int axis = 0; axis < Dynamics6Dof.NRCS; axis++)
        {
            RcsTorqueAllocator.AxisAuthority plus = six[2 * axis], minus = six[2 * axis + 1];
            bool p = Good(plus), m = Good(minus);
            if (!p && !m)
                continue;
            model.MaxPosNm[axis] = p ? plus.MaxTorqueNm : 0.0;
            model.MaxNegNm[axis] = m ? minus.MaxTorqueNm : 0.0;
            // Force for a torque tau: f+ tau one way, f- |tau| the other; the odd line through both is (f+ - f-) / 2, or the one side measured.
            double w = p && m ? 0.5 : 1.0;
            model.ForcePerNm[3 * axis] = w * ((p ? plus.ForcePerNmX : 0.0) - (m ? minus.ForcePerNmX : 0.0));
            model.ForcePerNm[3 * axis + 1] = w * ((p ? plus.ForcePerNmY : 0.0) - (m ? minus.ForcePerNmY : 0.0));
            model.ForcePerNm[3 * axis + 2] = w * ((p ? plus.ForcePerNmZ : 0.0) - (m ? minus.ForcePerNmZ : 0.0));
            model.FlowPerNm[axis] = w * ((p ? Math.Max(plus.FlowPerNm, 0.0) : 0.0) + (m ? Math.Max(minus.FlowPerNm, 0.0) : 0.0));
        }
        return model.Any ? model : null;
    }

    /// <summary>
    /// Peak roll torque the vehicle can actually produce, in N*m, at PlanningThrust - the model's TauRollMax before it is scaled to the plan's thrust. See TorqueAlong.
    /// </summary>
    public static double RollTorqueLimit(Vehicle vehicle)
    {
        KsaFrameBridge.BodyAxes(vehicle, out _, out _, out double3 mz);
        return TorqueAlong(vehicle, mz);
    }

    /// <summary>
    /// Peak LATERAL (pitch/yaw) torque the vehicle can actually produce, N*m, at PlanningThrust: the smaller of the two model lateral axes. See TorqueAlong.
    ///  This matters more than it looks. The model bounds lateral torque as LArm * T * tan(gimbal), and if that overstates the truth the solver plans a trajectory needing torque the allocator cannot deliver, the allocator saturates, and the vehicle does not fly its own plan - with nothing in the numbers to say why.
    /// </summary>
    /// <summary>A trim under this tangent (0.06 deg) is a balanced cluster: its cone keeps the zero-roll width.</summary>
    private const double OffAxisTrimTan = 1e-3;

    public static double LateralTorqueLimit(Vehicle vehicle)
    {
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out _);
        return Math.Min(TorqueAlong(vehicle, mx), TorqueAlong(vehicle, my));
    }

    /// <summary>
    /// The torque the allocator actually delivers along one model axis (KSA body coordinates), N*m, with every planning gimbal at its MaximumThrust: it is asked for far more than it has, and what it hands back along that axis is the capability.
    ///  THAT, AND NOT TvcAllocationResult.MaxTorque. MaxTorque is a per-KSA-axis figure that every gimbal could reach on its own, and it came out twice what the allocator delivers in any one direction: an over-asked allocator keeps the demand's direction and scales it down until the first gimbal saturates, so the deliverable torque is where that happens. Planned against MaxTorque, the 2026-10-08 booster's torques were scaled to about half by the allocator at every step and it never flew the attitude its plan turned on (measured live: 0.135 m of pitch torque per newton of thrust against MaxTorque's 0.268, 0.041 of roll against 0.088). The shared deflection is not the binding thing: 70 % of the pitch capability with 70 % of the roll fits.
    /// </summary>
    /// <summary>
    /// The lateral torque the allocator delivers along each model axis, N*m, the smaller of the two, when the roll a deflection of thrust acting off the axis brings is asked for with it (Dynamics6Dof.Params.GimbalRollPerTdx/Tdy, from <paramref name="thrustMomentPerN"/>): the authority the plan has once its dynamics carry that roll. For one engine off the stack's axis it is the main gimbal's; at zero roll (LateralTorqueLimit) it is what the verniers can take out.
    /// </summary>
    public static double CoupledLateralTorqueLimit(Vehicle vehicle, double3 thrustMomentPerN, double arm)
    {
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);
        if (!(arm > 0.0))
            return LateralTorqueLimit(vehicle);
        // A unit lateral torque about x is tdy = 1/L, about y tdx = -1/L; the roll is e_x tdy - e_y tdx with e = (-m_y, m_x).
        double rollPerX = -thrustMomentPerN.Y / arm, rollPerY = thrustMomentPerN.X / arm;
        return Math.Min(TorqueAlong(vehicle, mx, rollPerX, mz), TorqueAlong(vehicle, my, rollPerY, mz));
    }

    public static double TorqueAlong(Vehicle vehicle, double3 axis) => TorqueAlong(vehicle, axis, 0.0, default);

    /// <param name="rollPerUnit">The roll, about <paramref name="rollAxis"/>, that comes with each newton metre along <paramref name="axis"/>; asked for with it as the allocator's coupled torque, and allocated with redistribution, since nothing it brings is unmodelled. Zero for the zero-roll figure.</param>
    public static double TorqueAlong(Vehicle vehicle, double3 axis, double rollPerUnit, double3 rollAxis)
    {
        GimbalController[] gimbals = KsaGimbalControl.PlanningGimbals(vehicle);
        double length = axis.Length();
        if (gimbals.Length == 0 || !(length > 0.0))
            return 0.0;

        var thrusts = new double[gimbals.Length];
        double total = 0.0;
        for (int i = 0; i < gimbals.Length; i++)
        {
            thrusts[i] = gimbals[i].Data.MaximumThrust;
            total += thrusts[i];
        }

        // Vehicle's own CoM, not the FlightComputer's copy: the FC field is only refreshed by ReadMeasurements, so at engage time it can still be zero - which would silently measure the arm from the assembly origin instead of the centre of mass.
        double3 unit = axis / length;
        var commands = new double[2 * gimbals.Length];
        // Far beyond anything the gimbals can give - thrust times a kilometre - so the result is the saturation point.
        //  WITHOUT REDISTRIBUTION: the min-norm solution scaled to its first gimbal at a stop, which keeps the roll at zero. Past that point the free gimbals can bring roll with them (KsaTvcAllocator.Redistribute), which the plan's lateral cone does not model; the flight's allocation has that margin on top, for the attitude feedback.
        double size = Math.Max(total, 1.0) * 1000.0;
        bool coupled = rollPerUnit != 0.0;
        double3 roll = coupled ? rollAxis * (rollPerUnit * size) : default;
        TvcAllocationResult a = KsaTvcAllocator.Solve(gimbals, thrusts, vehicle.CenterOfMassAsmbF,
                                                      unit * size + roll, commands, redistribute: coupled, coupled: roll);
        return Math.Max(double3.Dot(a.AchievedTorque, unit), 0.0);
    }

    /// <summary>
    /// The vehicle's actual minimum throttle, from KSA's own aggregate (PartTree.EngineThrottleMin = min of MinimumThrottle over every engine).
    ///  The model's throttle floor was defaulting to 0.40 - the PYTHON TEST CASE's value, chosen there so min thrust sits just under hover. On a vehicle that can actually throttle to 0.10 that overstates Tmin four-fold, which is precisely the over-powered condition that forces the solver to tilt to shed thrust and produces looping, spiralling descents.
    /// </summary>
    public static double VehicleThrottleFloor(Vehicle vehicle)
    {
        PartTree tree = vehicle?.Parts;
        if (tree == null)
            return 1.0;
        double f = tree.EngineThrottleMin;
        return f is > 0.0 and <= 1.0 ? f : 1.0;
    }

    /// <summary>Smallest gimbal deflection limit on the vehicle, in degrees - the binding one.</summary>
    public static double GimbalLimitDeg(Vehicle vehicle)
    {
        double minRad = double.PositiveInfinity;
        foreach (GimbalController gc in KsaGimbalControl.PlanningGimbals(vehicle))
        {
            Gimbal g = gc.Gimbal;
            double m = Math.Min(g.AxisY.MaxAngle, g.AxisZ.MaxAngle);
            if (m > 0.0)
                minRad = Math.Min(minRad, m);
        }
        return double.IsPositiveInfinity(minRad) ? 0.0 : minRad * 180.0 / Math.PI;
    }

    /// <summary>
    /// Full solver configuration for this vehicle at this site. Returns false if the vehicle cannot be planned for (no engine, no gimbal) - better a refusal than a plan built on zeros. A craft with no roll torque is planned with the roll left free, see RollModel.
    /// </summary>
    /// <summary>
    /// Thrust-to-weight at the MINIMUM throttle, and the tilt that implies.
    ///  THE SINGLE MOST IMPORTANT NUMBER FOR WHETHER A PLAN EXISTS. The model has a convex throttle box Tmin &lt;= T &lt;= Tmax, so the vehicle can never thrust below Tmin while the engine is lit. If Tmin exceeds weight the vehicle CANNOT hold altitude pointing up - the only way to shed the excess is to tilt until the vertical component drops to 1 g, needing acos(1/TWRmin) of tilt. Past the tilt limit no descent exists at all and the solver reports infeasible; just inside it, the solver tilts to burn off thrust and the path curves away sideways, which is where the "spiral" trajectories come from.
    ///  The Python reference is tuned to TWRmin ~ 0.98 (min thrust just under hover), which is why it never shows this. A real vehicle rarely obliges - especially when Tmax sums every engine on the craft rather than the ones that will be lit.
    /// </summary>
    public static void ThrottleMargin(double thrust, double throttleFloor, double mass, double g,
                                      out double twrMin, out double requiredTiltDeg)
    {
        double weight = Math.Max(mass * g, 1e-9);
        twrMin = throttleFloor * thrust / weight;
        requiredTiltDeg = twrMin <= 1.0
            ? 0.0
            : Math.Acos(Math.Clamp(1.0 / twrMin, -1.0, 1.0)) * 180.0 / Math.PI;
    }

    /// <param name="glideSlopeDeg">
    /// Approach corridor, degrees above the horizontal at the target. 0 disables it.
    /// </param>
    /// <param name="vzMaxMs">Largest allowed climb rate, m/s. Negative disables it.</param>
    /// <param name="x0">Initial model state - needed to SIZE THE PROBLEM, see XScale below.</param>
    /// <param name="xf">Terminal model state, same reason.</param>
    /// <param name="accelMaxMs2">Largest thrust acceleration the plan may use, m/s^2, or 0 for none; see Scvx6DofConfig.AccelMax.</param>
    /// <param name="keptRollFree">The Scvx6DofConfig.TerminalRollFree of the plan this one replaces, or null for the first plan of a landing; see RollModel.</param>
    /// <param name="bodyAero">Plan with the body-axis force and moment (BodyAeroFor), or only the roll-averaged force on the thrust axis.</param>
    /// <param name="gimbalRateLimit">Hold the plan's lateral thrust to the gimbal's slew rate (GimbalSlewPerS).</param>
    /// <summary>
    /// The body-axis force and moment for the plan's full attitude (BodyAero6Dof), from the sweep's tables, or null where it measured none: the model's axes in the game's body axes (KsaFrameBridge.BodyAxes), and the moment moved from the centre of mass the sweep saw to the one the craft has now.
    /// </summary>
    /// <summary>
    /// The coast's rate price (Scvx6DofConfig.CoastRateWeight), as the fuel of this much burn, m/s, for turning at CoastTurnRefRadS about one axis through the whole coast. Small: it is there to pick one glide among those that cost the same fuel, not to stop a turn that saves some. The 2026-11-20 coasts turned 25 deg in about 7 s for the burn, about 1 m/s's worth at this price.
    /// </summary>
    private const double CoastTurnPriceMs = 1.0;
    private const double CoastTurnRefRadS = 0.05;

    /// <summary>
    /// The craft's control surfaces as the plan's fins (BodyAero6Dof.Fins): every surface the registry lists (ControlSurfaces.Collect) that the sweep has a table for, in the registry's order - the order the flight commands them in (TouchdownFinSurfaces) - with its range and rate. None where the sweep measured none.
    /// </summary>
    internal static BodyAero6Dof.FinTable[] FinsFor(Vehicle vehicle, KsaAeroSweep.Result aero, List<ControlSurface> surfaces = null)
    {
        surfaces?.Clear();
        if (vehicle == null || aero?.Surfaces == null || aero.Surfaces.Length == 0 || !ControlSurfaces.Any)
            return [];
        var live = new List<ControlSurface>();
        ControlSurfaces.Collect(vehicle, live);
        var fins = new List<BodyAero6Dof.FinTable>();
        foreach (ControlSurface surface in live)
            foreach (KsaAeroSweep.SurfaceTable table in aero.Surfaces)
                if (ReferenceEquals(table.Part, surface.Part) && table.DeflectionDeg.Length > 1)
                {
                    fins.Add(new BodyAero6Dof.FinTable
                    {
                        DeflectionDeg = table.DeflectionDeg,
                        Force = table.Force,
                        Moment = table.Moment,
                        // Over every air direction too, where the sweep had a whole-sphere pass and the body has one to match.
                        SphereForce = aero.SphereBodyAero != null && table.SphereForce.Length == table.DeflectionDeg.Length ? table.SphereForce : [],
                        SphereMoment = aero.SphereBodyAero != null && table.SphereMoment.Length == table.DeflectionDeg.Length ? table.SphereMoment : [],
                        // Belly first and on the flip band too, where the body has them to match (BodyAero6Dof.BellyForce, BandForce).
                        BellyForce = aero.BellyBodyAero != null && table.BellyForce.Length == table.DeflectionDeg.Length ? table.BellyForce : [],
                        BellyMoment = aero.BellyBodyAero != null && table.BellyMoment.Length == table.DeflectionDeg.Length ? table.BellyMoment : [],
                        BandForce = aero.BandBodyAero != null && table.BandForce.Length == table.DeflectionDeg.Length ? table.BandForce : [],
                        BandMoment = aero.BandBodyAero != null && table.BandMoment.Length == table.DeflectionDeg.Length ? table.BandMoment : [],
                        MinDeg = surface.MinDeg,
                        MaxDeg = surface.MaxDeg,
                        RateDegS = surface.RateDegS,
                    });
                    surfaces?.Add(surface);
                    break;
                }
        return fins.ToArray();
    }

    /// <summary>
    /// The share of the RCS's torque and of the control surfaces' travel touchdown's plans may use (Scvx6DofConfig.PlanRcsShare, FinMinDeg/FinMaxDeg), under the gimbals' (PlanTorqueShare): the rest is for flying the plan, the RCS's through the attitude feedback (AttitudeFeedback6Dof), and a turn the air, the flaps and the thrusters cannot do with room to spare is planned on the gimbals with the engine lit. The body's own air moment is physics and is not scaled. A Starship's plan flipped it in the coast on the flaps and the RCS at their limits; the air held the flip back harder than the model said (1.6-1.7 MN m against 0.5-1.1 between 75 and 45 deg of tilt), the craft fell 26 deg behind, and it lit at 60 m (2026-11-24).
    /// </summary>
    internal const double PlanSurfaceShare = 0.75;   // the default; PLAN RCS AND FLAPS sets it (TryBuild's surfaceShare)

    private static BodyAero6Dof BodyAeroFor(Vehicle vehicle, KsaAeroSweep.Result aero, double momentLimiterPerPa,
                                            double momentLimiterStiffnessPerPa, double momentLimiterDampingPerPaMs, bool fins = true)
    {
        if (aero?.BodyAero == null || aero.Atmosphere == null || !(aero.ReferenceArea > 0.0))
            return null;
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);
        float3 com = vehicle.CenterOfMassAsmbF;
        double3 shift = aero.BodyMoment != null ? aero.BodyMomentComAsmb - new double3(com.X, com.Y, com.Z) : double3.Zero;
        return new BodyAero6Dof
        {
            Atmosphere = aero.Atmosphere,
            Force = aero.BodyAero,
            Moment = aero.BodyMoment,
            // Every air direction past the engine-first table's fade, where the sweep flew the whole sphere (a belly-flop's tables): the flip.
            SphereForce = aero.SphereBodyAero,
            SphereMoment = aero.SphereBodyMoment,
            // A belly-flopper's belly-first tables and the flip band between them and engine first, read before the whole sphere.
            BellyForce = aero.BellyBodyAero,
            BellyMoment = aero.BellyBodyMoment,
            BandForce = aero.BandBodyAero,
            BandMoment = aero.BandBodyMoment,
            ReferenceArea = aero.ReferenceArea,
            ReferenceLength = aero.ReferenceLength,
            ModelToTable = [mx.X, my.X, mz.X, mx.Y, my.Y, mz.Y, mx.Z, my.Z, mz.Z],
            ComShiftX = shift.X, ComShiftY = shift.Y, ComShiftZ = shift.Z,
            RotDampX = double.IsFinite(aero.RotDampCoeff.X) && aero.RotDampCoeff.X > 0.0 ? aero.RotDampCoeff.X : 0.0,
            RotDampY = double.IsFinite(aero.RotDampCoeff.Y) && aero.RotDampCoeff.Y > 0.0 ? aero.RotDampCoeff.Y : 0.0,
            RotDampZ = double.IsFinite(aero.RotDampCoeff.Z) && aero.RotDampCoeff.Z > 0.0 ? aero.RotDampCoeff.Z : 0.0,
            MomentLimiterPerPa = double.IsFinite(momentLimiterPerPa) && momentLimiterPerPa > 0.0 ? momentLimiterPerPa : 0.0,
            MomentLimiterStiffnessPerPa = double.IsFinite(momentLimiterStiffnessPerPa) && momentLimiterStiffnessPerPa > 0.0 ? momentLimiterStiffnessPerPa : 0.0,
            MomentLimiterDampingPerPaMs = double.IsFinite(momentLimiterDampingPerPaMs) && momentLimiterDampingPerPaMs > 0.0 ? momentLimiterDampingPerPaMs : 0.0,
            // The control surfaces, one fin control each (Dynamics6Dof.IFIN on).
            Fins = fins ? FinsFor(vehicle, aero) : [],
        };
    }

    /// <summary>The fixed length and speed the terminal miss is priced on, m and m/s (Scvx6DofConfig.TerminalMissScaleM): the scales of the hand-over the terminal weights were tuned at.</summary>
    internal const double TerminalMissPriceM = 250.0, TerminalSpeedPriceMs = 100.0;

    /// <summary>
    /// The body-rate scale, rad/s. The trust region is a share of each scale, so at 1 a warm re-solve may move a body rate 0.05 rad/s in one step against a 0.01 rad/s defect gate, and the refusals at ignition on 2026-11-20 were rate defects at the first interval, 4-8 times the gate.
    ///  KEPT AT 1. Tried at 0.2, about the rates a landing flies (--6dof-coast --rate-scale 0.2, 2026-11-20): the two-phase landings came out the same (3.1 and 2.8 m off against 2.9 and 3.2 m), but the burn-only re-solves went from 650-1,825 SCS iterations to 5,750-9,000, at the cap. The virtual control and the rate limit's slack are priced on this scale, so a tighter one makes every subproblem harder.
    /// </summary>
    internal static double RateScaleRadS = 1.0;

    public static bool TryBuild(Vehicle vehicle, IParentBody parent, double3 siteCci,
                                int nodes, double tiltMaxDeg, double throttleFloor,
                                double sigmaSeed,
                                double rateDampShare, double controlSmoothWeight,
                                double proximalWeight,
                                double glideSlopeDeg, double vzMaxMs,
                                double[] x0, double[] xf,
                                out Scvx6DofConfig cfg, out Dynamics6Dof.Params dyn,
                                out string error, double accelMaxMs2 = 0.0, double floorMargin = 0.0,
                                KsaAeroSweep.Result aero = null, bool? keptRollFree = null, bool bodyAero = true,
                                bool gimbalRateLimit = true, Dynamics6Dof.RcsModel rcs = null, double rcsWeight = 0.0,
                                double rateMaxDegS = 0.0, int coastIntervals = 0, double[] scaleFrom = null,
                                double momentLimiterPerPa = 0.0, double torqueMargin = 0.0, bool fins = true, double surfaceShare = PlanSurfaceShare,
                                double momentLimiterStiffnessPerPa = 0.0, double momentLimiterDampingPerPaMs = 0.0,
                                double[] glideFunnelDeg = null)
    {
        cfg = null!;
        dyn = null!;
        error = "";

        // Thrust AT THE PRESSURE THE VEHICLE WILL ACTUALLY SEE, not in vacuum.
        //  Evaluated at the TARGET altitude rather than the current one, deliberately.
        // Thrust falls as a lander descends into thicker air, so the current (high, thin) altitude is the most optimistic point on the whole trajectory - using it reintroduces a smaller copy of the same error. The target is the densest air the vehicle will meet, so this under-promises slightly for the rest of the descent, and under-promising is the safe direction: the plan is conservative rather than one the vehicle cannot fly.
        double siteAltAsl = siteCci.Length() - parent.MeanRadius;
        double targetAltAsl = siteAltAsl + xf[2];
        LastAmbientPressurePa = KsaEnginePerf.AmbientPressureAt(parent, targetAltAsl);

        (double thrust, double massFlow) = KsaEnginePerf.AtPressure(vehicle, LastAmbientPressurePa);
        if (thrust <= 0.0 || massFlow <= 0.0)
        {
            error = "no engine thrust to plan with";
            return false;
        }

        LastVacuumThrustN = KsaEnginePerf.VacuumThrust(vehicle);
        LastPressureThrustN = thrust;

        // THE FLOOR AS THE THRUST IT GIVES, not as a fraction of Tmax. The nozzle's back-pressure loss is the same newtons at any throttle, so the minimum throttle gives less than that fraction of full thrust. On the 2026-10-08 boosters, floor x Tmax took 13.3 % throttle to deliver against a real 10 % floor, putting the model's minimum thrust above the weight of a booster that could in fact hover - and its plans climbed off the pad. The model's floor is the real thrust at the minimum throttle over Tmax, both at the same pressure.
        (double floorThrust, double floorFlow) = KsaEnginePerf.AtThrottle(vehicle, throttleFloor, LastAmbientPressurePa);
        // THE PLAN'S FLOOR sits above the engines' by floorMargin of the thrust range, so the tracker has thrust to take off, and under the hover line when the engines can go under it (KsaEnginePerf.LandingPlanFloorThrottle); the real floor stays what the flow is fitted through and what the over-powered check falls back to.
        //  The weight is the one at the end of the burn, where hovering matters: no lighter than dry, and no heavier than now less the burn's seed time at the floor's flow.
        double siteGravity = parent.Mu / siteCci.LengthSquared();
        double endMass = Math.Max(vehicle.TotalMass - vehicle.PropellantMass, vehicle.TotalMass - Math.Max(sigmaSeed, 0.0) * Math.Max(floorFlow, 0.0));
        double planFloorThrust = floorMargin > 0.0
            ? KsaEnginePerf.ThrustAtThrottle(vehicle,
                KsaEnginePerf.LandingPlanFloorThrottle(vehicle, throttleFloor, floorMargin, LastAmbientPressurePa, endMass * siteGravity),
                LastAmbientPressurePa)
            : floorThrust;
        double realFloorFraction = floorThrust > 0.0 && floorThrust < thrust ? floorThrust / thrust : throttleFloor;
        if (planFloorThrust > 0.0 && planFloorThrust < thrust)
            throttleFloor = planFloorThrust / thrust;
        else
            throttleFloor = realFloorFraction;

        // MASS FLOW AS THE ENGINES BURN IT. The flow follows the throttle, but under back pressure the thrust does not, so the flow is not proportional to the thrust: one Isp read at full thrust had the 2026-10-08 booster burning 62 kg/s at its 10 % floor against a real 91.7, and every re-solve from the standby refused on the mass channel. The line through the game's two ends of the range - the floor and full thrust, at the same pressure - is exact at both and within 5 % between them on that booster.
        double alpha = massFlow / thrust, flowOffset = 0.0;
        if (floorThrust > 0.0 && floorFlow > 0.0 && thrust > 1.01 * floorThrust)
        {
            double slope = (massFlow - floorFlow) / (thrust - floorThrust);
            double offset = floorFlow - slope * floorThrust;
            if (slope > 0.0 && double.IsFinite(offset))
            {
                alpha = slope;
                flowOffset = offset;
            }
        }

        // Tmax is the thrust of every engine the craft runs, because no engine is shut down for the landing. The throttle command is T / Tmax of the model, so a plan built against a smaller Tmax would command full throttle for part of the thrust and overshoot every plan.
        // An over-powered vehicle is a real situation, not something to model around. The feasibility readout reports it and names the throttle floor that would work.

        // NO GIMBAL: the plan points the thrust with the RCS alone, where it turns the craft both ways about pitch and yaw (RcsSteers). The gimbal's controls are then pinned at zero by their own cone (a zero angle), and its arm and roll play no part.
        double gimbalDeg = GimbalLimitDeg(vehicle);
        bool noGimbal = gimbalDeg <= 0.0;
        if (noGimbal && !RcsSteers(rcs))
        {
            error = "no gimballed engine and no RCS torque on pitch and yaw - nothing to point the thrust with";
            return false;
        }

        // TORQUE AUTHORITY PER NEWTON. Both capabilities below are the allocator's at PlanningThrust, and the gimbals deliver torque in proportion to the thrust through them. Divided by full thrust at the target's pressure instead, the effective gimbal came out 1.3x too wide on the 2026-10-08 booster (3.17 deg against 2.40), and the roll bound was a fixed figure at that same vacuum thrust: the plan asked for torque the gimbals could not give at a third of full thrust, and the allocator saturated from the first step.
        // Per newton of every engine's thrust, because the model's thrust is that of all of them; see PlanningEngineThrust.
        double planningThrust = PlanningEngineThrust(vehicle);
        double tauRoll = RollTorqueLimit(vehicle);
        // The roll capability at the model's Tmax; the rows scale it with the thrust (RollTorqueFollowsThrust).
        if (planningThrust > 0.0)
            tauRoll *= thrust / planningThrust;

        double arm = noGimbal ? 0.0 : EngineArm(vehicle);
        if (!noGimbal && arm <= 0.0)
        {
            error = "engine thrust point coincides with the centre of mass";
            return false;
        }

        // Replace the vehicle's nominal gimbal limit with the EFFECTIVE one implied by the torque the allocator can really deliver:
        //     LArm * Tmax * tan(effective) = measured lateral capability so the model's own gimbal cone bounds the plan to flyable torque. Capped at the physical limit, since the allocator can never beat the hardware, and floored so a measurement glitch cannot collapse the cone to nothing.
        double latCapability = LateralTorqueLimit(vehicle);
        if (latCapability > 0.0)
        {
            double perN = arm * (planningThrust > 0.0 ? planningThrust : thrust);
            double effectiveTan = latCapability / perN;
            // THRUST OFF THE AXIS (an off-centre engine choice): the plan's dynamics carry the roll a deflection brings (Dynamics6Dof.Params.GimbalRollPerTdx/Tdy) and its cone is centred on the trim (GimbalTrimX/Y), so its width is the authority with that roll AROUND the trim: the coupled capability less the trim, never under the zero-roll width and never past the hardware's travel beyond the trim. On one 10 deg engine of the triple plate the zero-roll width was 1.54 deg, the verniers' roll authority, and every plan sat at its edge (2026-11-21 10:58). A balanced cluster keeps the zero-roll width it had.
            double3 thrustMoment = noGimbal ? double3.Zero : ThrustMomentPerNewton(vehicle);
            double trimTan = Math.Sqrt(thrustMoment.X * thrustMoment.X + thrustMoment.Y * thrustMoment.Y) / arm;
            if (trimTan > OffAxisTrimTan)
            {
                double coupledTan = CoupledLateralTorqueLimit(vehicle, thrustMoment, arm) / perN;
                double hardwareTan = Math.Tan(gimbalDeg * Math.PI / 180.0);
                double aroundTrim = Math.Min(coupledTan, hardwareTan) - trimTan;
                if (aroundTrim > effectiveTan)
                    effectiveTan = aroundTrim;
            }
            double effectiveDeg = Math.Atan(effectiveTan) * 180.0 / Math.PI;
            gimbalDeg = Math.Clamp(effectiveDeg, 0.1, gimbalDeg);
        }

        Inertia(vehicle, out double ixx, out double iyy, out double izz);
        if (ixx <= 0.0 || iyy <= 0.0 || izz <= 0.0)
        {
            error = "vehicle inertia is degenerate";
            return false;
        }
        RollModel(tauRoll, izz, keptRollFree, out bool rollFree, out tauRoll, RcsRollNm(rcs));

        // THE TORQUE MARGIN (Scvx6DofConfig.PlanTorqueShare): the plan gets this share of the gimbals' torque, their roll and the RCS's, and the attitude feedback the rest (AttitudeFeedback6Dof). The cone is shrunk in torque, through its tangent; the gimbals' slew, in full deflections a second, is widened by as much so it stays the hardware's rate in degrees. After the roll model, so a margin cannot by itself decide the roll is left free.
        double torqueShare = Math.Clamp(1.0 - (double.IsFinite(torqueMargin) ? torqueMargin : 0.0), 0.5, 1.0);
        gimbalDeg = Math.Atan(Math.Tan(gimbalDeg * Math.PI / 180.0) * torqueShare) * 180.0 / Math.PI;
        tauRoll *= torqueShare;
        // THE RCS AND THE CONTROL SURFACES AT PlanSurfaceShare, under the gimbals' share: a turn the air, flaps and thrusters cannot do with room to spare is planned on the gimbals with the engine lit.
        surfaceShare = Math.Min(torqueShare, Math.Clamp(double.IsFinite(surfaceShare) ? surfaceShare : PlanSurfaceShare, 0.05, 1.0));
        double[] rcsPos = rcs?.MaxPosNm is { } pos ? Array.ConvertAll(pos, v => v * surfaceShare) : null;
        double[] rcsNeg = rcs?.MaxNegNm is { } neg ? Array.ConvertAll(neg, v => v * surfaceShare) : null;
        // A plan with a coast keeps the heading and the roll-rate limit like any other. Freeing them was tried on 2026-11-20 while the air's roll moment read 40 kN m against 22 of RCS; that was the sweep's moment without the game's limiter (BodyAero6Dof.MomentLimiterPerPa), and the booster felt about 11.
        bool coastPlan = false;

        // Refuse an over-powered configuration up front, with the number needed to fix it. Reporting "infeasible" after a 500 ms solve tells the user nothing; this says exactly which knob is wrong and what value would work.
        {
            double gSite = parent.Mu / (siteCci.Length() * siteCci.Length());
            ThrottleMargin(thrust, throttleFloor, vehicle.TotalMass, gSite,
                           out double twrMin, out double needTiltDeg);
            // A margin that leaves no plan is dropped rather than refusing the craft: planning on the real floor is what it did before.
            if (needTiltDeg >= tiltMaxDeg && throttleFloor > realFloorFraction)
            {
                throttleFloor = realFloorFraction;
                ThrottleMargin(thrust, throttleFloor, vehicle.TotalMass, gSite, out twrMin, out needTiltDeg);
            }
            if (needTiltDeg >= tiltMaxDeg)
            {
                double feasibleFloor = Math.Cos(tiltMaxDeg * Math.PI / 180.0) *
                                       vehicle.TotalMass * gSite / thrust;
                error =
                    $"over-powered: min throttle gives TWR {twrMin:F2}, which needs " +
                    $"{needTiltDeg:F0} deg of tilt just to stop climbing (limit {tiltMaxDeg:F0} deg). " +
                    $"Lower the throttle floor below {feasibleFloor:F2} or raise the tilt limit.";
                return false;
            }
        }

        // Gravity at the SITE, from the body actually being landed on - not 9.81.
        // Constant over the descent, which is the model's assumption; see the fidelity notes for when that stops being true.
        double r = siteCci.Length();
        double g = parent.Mu / (r * r);

        const double G0 = 9.80665;
        dyn = new Dynamics6Dof.Params
        {
            Gx = 0.0, Gy = 0.0, Gz = -g,
            G0 = G0,
            Isp = 1.0 / (alpha * G0),
            MassFlowOffset = flowOffset,
            Engine = LastAmbientPressurePa > 0.0 && aero?.Atmosphere != null && aero.Atmosphere.SeaLevelPressure > 0.0
                ? Ksa3DofSetup.EngineTable(vehicle, aero.Atmosphere.SeaLevelPressure)
                : null,
            EngineRefPressurePa = LastAmbientPressurePa,
            // THE AERODYNAMICS, from the same sweep-measured tables the 3-DOF flies (Ksa3DofSetup.BuildModel), read at the site's altitude plus the model's height. Without them the model had none, and the bias estimate stood in: a constant acceleration measured at the hand-over's ~90 m/s - several m/s^2 of drag and lift - that the plan leaned on to brake, and that fell away as the booster slowed (2026-10-08: it crossed the site still moving 13 m/s sideways).
            Aero = aero?.Table != null
                ? new KsaPointMassModel
                {
                    Atmosphere = aero.Atmosphere,
                    Drag = aero.Table,
                    LiftSlope = aero.LiftTable != null
                        ? KsaPointMassModel.LiftSlopeTable(aero.MachGrid, aero.AlphaGridDeg, aero.LiftTable)
                        : null,
                    ReferenceArea = aero.ReferenceArea,
                }
                : null,
            // THE BODY-AXIS FORCE AND MOMENT for the plan's own attitude, where the sweep measured them: the gimbal then holds against the air's moment from the plan's start rather than finding it in flight (BodyAero6Dof). The tables above carry the share of the force these fade out of.
            BodyAero = bodyAero && aero?.Table != null
                ? BodyAeroFor(vehicle, aero, momentLimiterPerPa, momentLimiterStiffnessPerPa, momentLimiterDampingPerPaMs, fins)
                : null,
            AeroAltitudeOffset = siteAltAsl,
            LArm = arm,
            Ixx = ixx, Iyy = iyy, Izz = izz,
            // RCS torque and the side force and propellant that come with it (RcsModel), or none.
            Rcs = rcs,
        };
        // THE ENGINES' OWN MOMENT per newton (ThrustMomentPerNewton), on a gimballed craft: from the centre of mass now, so each re-solve has it as the propellant burns down.
        if (!noGimbal)
        {
            double3 thrustMoment = ThrustMomentPerNewton(vehicle);
            dyn.ThrustMomentX = thrustMoment.X;
            dyn.ThrustMomentY = thrustMoment.Y;
            dyn.ThrustMomentZ = thrustMoment.Z;
        }

        // NON-DIMENSIONALISE AGAINST THIS PROBLEM, not against the Python test case.
        //  This is the difference between converging and "cold solve failed". The solver works on x~ = x/scale, and THE TRUST REGION IS IN THOSE UNITS, capped at TrustRegionMax = 0.1. So the movement permitted per iteration is 0.1*scale in physical terms. With the scale hard-coded to the reference case's 300 m altitude, that is ~30 m per iteration REGARDLESS of the actual descent: engage from 3 km and the problem is ten times larger while the step stays the same, so the iteration budget can no longer traverse it. The problem is perfectly feasible; the solver simply cannot walk there. It also explains why relaxing constraints does not help - this was never a constraint failure.
        //  Gfold learned this already (GfoldPlanner: "length scale ~ the problem size, time scale such that gravity is ~1... an interior-point solver breaks down on the raw SI problem"). Same lesson, applied here.
        //  One length scale for all three position axes rather than per-axis: it is the trajectory's own extent, and it keeps the scaling isotropic so a mostly horizontal approach conditions as well as a vertical one. Velocity is scaled by the natural free-fall speed over that length, floored by the actual initial speed so a fast entry is not under-scaled.
        //  Quaternion and body-rate scales stay at 1: both are already dimensionless or O(1), and they are the values the reference converges with - no reason to perturb a validated part of the setup.
        double mass = vehicle.TotalMass;
        double gScale = parent.Mu / (siteCci.Length() * siteCci.Length());
        // A plan with a coast before ignition is scaled on the burn, from the state at ignition (scaleFrom), not on the glide from the craft now: the terminal miss is priced per length scale, so scaled on a hand-over several hundred metres up it cost a third as much per metre, and offline the landings missed by 10 m against 3 m (--6dof-coast, 2026-11-20).
        double[] sx = scaleFrom != null && scaleFrom.Length >= 6 ? scaleFrom : x0;
        double lengthScale = Math.Max(
            Math.Sqrt((sx[0] - xf[0]) * (sx[0] - xf[0]) +
                      (sx[1] - xf[1]) * (sx[1] - xf[1]) +
                      (sx[2] - xf[2]) * (sx[2] - xf[2])), 1.0);
        double speed0 = Math.Sqrt(sx[3] * sx[3] + sx[4] * sx[4] + sx[5] * sx[5]);
        double velocityScale = Math.Max(Math.Max(speed0, Math.Sqrt(lengthScale * gScale)), 1.0);
        // Burn-time bounds must BRACKET the seed. Scvx6DofConfig's defaults (5..25 s, scale 12) describe the Python test case; leaving them while seeding sigma elsewhere gives the solver a starting point outside its own feasible range and a badly conditioned scale. Both are set from the seed instead.
        double sigma = Math.Max(sigmaSeed, 1.0);
        // REGULARISER WEIGHTS SIZED FOR THIS PROBLEM, not the Python test case. 6dof.py uses W_DU = 0.2, W_W = 1.0, and at its converged solution those contribute 41% and 80% of the fuel term respectively - 121% combined.
        // They are not regularising the objective, they ARE the objective.
        // That matters because BOTH GET CHEAPER AS SIGMA GROWS: stretch the same manoeuvre over a longer burn and the node-to-node control deltas shrink, and the rotation rates shrink.
        // Only the fuel term pushes back.
        // W_W scales as N*3*omega^2, so at a perfectly ordinary 0.1 rad/s it is ~10x fuel and at 0.2 rad/s ~40x - at which point the optimiser is simply minimising rotation rate, and the cheapest way to rotate slowly is to take as long as possible.
        // Sigma pins to its upper bound and the vehicle loops.
        // Fix: size W_W so the rate penalty is a SMALL FRACTION of fuel at the rate this manoeuvre actually needs - roughly the tilt range swept over half the burn.
        // Scvx6DofConfig's defaults stay at the Python values so the reference validation and the constants drift guard remain valid; only flight overrides them. rateDampShare is the fraction of a typical (~0.1) fuel term the rate penalty should cost at the manoeuvre's characteristic rate.
        // Turned right down after flight testing: these terms only need to discourage chatter and spin, and anything large enough to show up against fuel is steering the trajectory instead.
        // There IS a floor - at zero the control can go bang-bang between nodes and the attitude can oscillate - so the objective readout and the plan overlay are the check when reducing further.
        double omegaScale = Math.Max(tiltMaxDeg * Math.PI / 180.0 / Math.Max(sigma * 0.5, 1.0), 1e-3);
        double wW = Math.Max(rateDampShare, 0.0) / (nodes * 3.0 * omegaScale * omegaScale);

        double[] funnelHeightsM = null, funnelHalfAngleDeg = null;
        if (glideSlopeDeg > 0.0 && glideFunnelDeg is { Length: 3 })
            GlideFunnelKnots(vehicle, glideFunnelDeg[0], glideFunnelDeg[1], glideFunnelDeg[2], out funnelHeightsM, out funnelHalfAngleDeg);
        LastGlideFunnelHeightsM = funnelHeightsM;
        LastGlideFunnelHalfAngleDeg = funnelHalfAngleDeg;

        BodyAero6Dof.FinTable[] finTables = dyn.BodyAero?.Fins ?? [];
        cfg = new Scvx6DofConfig
        {
            Nodes = nodes,
            // The coast before ignition (Scvx6DofConfig.CoastIntervals), or none.
            CoastIntervals = coastIntervals > 0 && coastIntervals <= nodes - 3 ? coastIntervals : 0,
            CoastSigmaMax = 60.0,
            Tmax = thrust,
            ThrottleFloor = throttleFloor,
            GimbalMaxDeg = gimbalDeg,
            // The cone centred on the deflection that holds the engines' own moment (Scvx6DofConfig.GimbalTrimX/Y): L tdy + m_x T = 0, -L tdx + m_y T = 0.
            GimbalTrimX = !noGimbal && arm > 0.0 ? dyn.ThrustMomentY / arm : 0.0,
            GimbalTrimY = !noGimbal && arm > 0.0 ? -dyn.ThrustMomentX / arm : 0.0,
            // The RCS controls are scaled on each axis's whole control torque (Scvx6DofConfig.ResolvedUScale).
            GimbalArm = arm,
            GimbalSlewPerS = gimbalRateLimit ? GimbalSlewPerS(vehicle) / torqueShare : 0.0,
            PlanTorqueShare = torqueShare,
            PlanRcsShare = surfaceShare,
            TauRollMax = tauRoll,
            RollTorqueFollowsThrust = true,
            RcsMaxPosNm = rcsPos,
            RcsMaxNegNm = rcsNeg,
            RcsWeight = rcsWeight,
            // THE FINS (Dynamics6Dof.IFIN on): each one's range and rate, the plan's share of them (PlanTorqueShare), the rest kept for the feedback as for the gimbals and the RCS.
            FinMinDeg = finTables.Length > 0 ? Array.ConvertAll(finTables, f => f.MinDeg * surfaceShare) : null,
            FinMaxDeg = finTables.Length > 0 ? Array.ConvertAll(finTables, f => f.MaxDeg * surfaceShare) : null,
            FinRateDegS = finTables.Length > 0 ? Array.ConvertAll(finTables, f => f.RateDegS * torqueShare) : null,
            // The turn-rate limit (Scvx6DofConfig.RateMaxRadS).
            RateMaxRadS = Math.Max(rateMaxDegS, 0.0) * Math.PI / 180.0,
            RateLimitIncludesRoll = !coastPlan,
            TerminalRollFree = rollFree,
            // WIDENED TO ADMIT THE CURRENT ATTITUDE. The tilt cone is applied at every node INCLUDING node 0, and node 0 is pinned by an equality to the measured state - so a cap below the vehicle's present tilt asks for a quaternion that is simultaneously exactly the measured one and inside a cone the measured one is outside. That is infeasible by construction, not merely expensive: measured offline from the the recorded run entry at 92.1 degrees with a 60-degree cap, NO node count solves at all - infinite defect at 15, 30 and 50 nodes, never reaching even the loose cold gate. Raise the cap above the entry attitude and the same problem solves at every one of them.
            //  This is the same failure the trust-region box had before it was moved off node 0, and the reason the glideslope and climb-rate constraints start at node 1. Widening rather than restructuring the rows, because the row layout is what the Python reference comparison checks.
            //  A GUARD, NOT A DIAGNOSIS. The flight this was measured against had already been flown with the cap raised to 120, so it was feasible throughout - the trouble there was node spacing, not this. What this prevents is the case where someone leaves the cap at a value that sounds cautious and the vehicle simply cannot be planned for, with nothing in the failure text to say why.
            //  The margin is what makes it a starting point rather than a knife edge: a cap exactly at the current tilt leaves the first node with no slack at all.
            TiltMaxDeg = Math.Min(Math.Max(tiltMaxDeg, EntryTiltDeg(x0) + 5.0), 175.0),
            GroundFloor = -1.0,
            // Never under what the minimum throttle already gives at this mass: a limit below it could only ever be met through its slack.
            AccelMax = accelMaxMs2 > 0.0 ? Math.Max(accelMaxMs2, 1.05 * throttleFloor * thrust / vehicle.TotalMass) : 0.0,
            // Velocity's virtual control priced on its own scale, the rest on XScale; see VirtualControlVelocityScaleMs.
            VirtualControlScale =
            [
                VirtualControlPositionScaleM, VirtualControlPositionScaleM, VirtualControlPositionScaleM,
                VirtualControlVelocityScaleMs, VirtualControlVelocityScaleMs, VirtualControlVelocityScaleMs,
                0, 0, 0, 0,
                0, 0, 0,
                0,
            ],

            // Generous bounds. These only need to BRACKET the answer - sigma is a free variable and the solver picks it - but a bound that binds silently dictates the trajectory instead of the physics: sigma pinned at the max forces the vehicle to spend that long in the air, and if it cannot hover it loops to burn the time. Widening costs nothing, because conditioning comes from SigmaScale (the seed), not from the bounds.
            WW = wW,
            // The coast's rate price (Scvx6DofConfig.CoastRateWeight): turning at CoastTurnRefRadS about one axis through the whole coast costs what CoastTurnPriceMs of burn does in fuel, whose term is -m_final/m_init, so a metre per second is alpha (1 / Isp g0) of it.
            CoastRateWeight = coastIntervals > 1 && coastIntervals <= nodes - 3
                ? CoastTurnPriceMs * alpha / ((coastIntervals - 1) * CoastTurnRefRadS * CoastTurnRefRadS)
                : 0.0,
            WDu = Math.Max(controlSmoothWeight, 0.0),
            // Restores the conditioning the regularisers previously provide, without the bias. Measured on the reference problem at low weights: 91650 ADMM iterations / 4907 ms at 0, versus 23675 / 1295 ms at 0.05 - 3.8x faster for a burn time within 1.2% and a merit within 0.03%. Do NOT raise this chasing more speed: at 0.2 it is 7x faster but shifts burn time 6% and worsens merit 1%, because SCvx stops at a tolerance rather than exactly at X = Xbar, so a large proximal term leaves a residual pull and starts acting as a second trust region.
            ProximalWeight = Math.Max(proximalWeight, 0.0),
            GlideSlopeDeg = glideSlopeDeg,
            // The progressive glideslope: half-angles from vertical at one craft length, 300 m and 1000 m (GlideFunnelKnots).
            GlideFunnelHeightsM = funnelHeightsM,
            GlideFunnelHalfAngleDeg = funnelHalfAngleDeg,
            VzMax = vzMaxMs,
            // Terminal position and speed are SOFT, heavily penalised. A hard terminal says "arrive exactly there, exactly at rest", and when that is not achievable the honest answer is a plan that misses rather than no plan at all - a descending booster cannot decline to land. The weights are high enough that both slacks sit at zero whenever the target is genuinely reachable, so this changes nothing on a normal descent.
            //  POSITION ABOVE SPEED. Both are normalised by the problem's scales (here ~250 m and ~100 m/s at a hand-over), so the weights alone do not say what trades for what. With speed at 1e5 and the miss at 1e3, 1 m/s cost as much as ~270 m of miss, and a squeezed plan stopped at rest 20-30 m over the pad - where a booster that cannot throttle below its weight floats and then climbs (2026-10-08, 17:42 and 17:53). At 1e5 and 1e4, 1 m of miss costs about as much as 4 m/s: a squeezed plan arrives a little fast rather than high, as Falcon 9 lands at ~2 m/s rather than stopping short.
            TerminalMissWeight = 1.0e5,
            TerminalSpeedWeight = 1.0e4,
            // Priced per fixed metre and m/s - the ~250 m and ~100 m/s scales of the hand-over these weights were tuned at - so a metre of miss costs the same however far away the plan starts (Scvx6DofConfig.TerminalMissScaleM).
            TerminalMissScaleM = TerminalMissPriceM,
            TerminalSpeedScaleMs = TerminalSpeedPriceMs,
            SigmaMin = sigma * 0.15,
            SigmaMax = sigma * 4.0,
            SigmaScale = sigma,
            XScale =
            [
                lengthScale, lengthScale, lengthScale,
                velocityScale, velocityScale, velocityScale,
                1, 1, 1, 1,
                // Body rates on the rates a landing flies (RateScaleRadS), not 1 rad/s.
                RateScaleRadS, RateScaleRadS, RateScaleRadS,
                Math.Max(mass, 1.0),
            ],
        };
        // Validate every derived value before handing the problem to the native solver because one non-finite value can make the complete matrix invalid.
        //  The realistic path is not exotic: `gScale = Mu / |siteCci|^2` is Infinity if the site has not resolved yet, and `Math.Max(NaN, x)` RETURNS NaN, so a single bad intermediate propagates silently through the scale chain.
        (string Name, double Value)[] mustBeFinitePositive =
        [
            ("Tmax", cfg.Tmax), ("Tmin", cfg.Tmin), ("gimbal limit", noGimbal ? 1.0 : cfg.GimbalMaxDeg),
            ("tau_roll max", cfg.TauRollMax), ("tilt limit", cfg.TiltMaxDeg),
            ("sigma scale", cfg.SigmaScale), ("sigma min", cfg.SigmaMin), ("sigma max", cfg.SigmaMax),
            ("length scale", lengthScale), ("velocity scale", velocityScale),
            ("gravity", -dyn.Gz), ("Isp", dyn.Isp), ("L_arm", noGimbal ? 1.0 : dyn.LArm),
            ("Ixx", dyn.Ixx), ("Iyy", dyn.Iyy), ("Izz", dyn.Izz),
            ("mass", mass),
        ];
        foreach ((string name, double value) in mustBeFinitePositive)
        {
            if (!double.IsFinite(value) || value <= 0.0)
            {
                error = $"{name} is {value} - not a usable number. The solver would be " +
                        "handed a poisoned problem.";
                return false;
            }
        }
        for (int i = 0; i < cfg.XScale.Length; i++)
        {
            if (!double.IsFinite(cfg.XScale[i]) || cfg.XScale[i] <= 0.0)
            {
                error = $"state scale [{i}] is {cfg.XScale[i]} - not usable.";
                return false;
            }
        }
        if (cfg.SigmaMin >= cfg.SigmaMax)
        {
            error = $"burn-time bounds are inverted ({cfg.SigmaMin:F2} .. {cfg.SigmaMax:F2}).";
            return false;
        }

        return true;
    }
}

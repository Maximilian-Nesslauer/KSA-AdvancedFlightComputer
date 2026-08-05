using System;
using BepuUtilities;
using Brutal.Numerics;
using KSA;
using Scvx;

/// <summary>
/// Builds the 6-DOF solver's configuration from the LIVE vehicle and body, instead
/// of the 6dof.py mirror values Scvx6DofConfig defaults to.
///
/// Everything here is measured, not assumed — the defaults describe a Super
/// Heavy-class booster landing on a 9.81 m/s^2 world, which is not what is being
/// flown. Getting these wrong does not error: the solver happily plans a perfectly
/// feasible trajectory for the wrong vehicle.
/// </summary>
public static class Ksa6DofSetup
{
    /// <summary>
    /// Inertia about the model's body axes, in kg m^2.
    ///
    /// KSA's Vehicle.TotalMassPropsBody.Inertia is a FULL symmetric tensor about
    /// the centre of mass and is LIVE — it tracks propellant drain, so this must be
    /// re-read per plan rather than captured once. The model wants a diagonal, so we
    /// project onto the model body axes and drop the off-diagonal terms. That is
    /// defensible rather than lazy: KSA's own UpdateTvcParams uses only the diagonal
    /// too, so the game's control model already ignores those terms.
    /// </summary>
    public static void Inertia(Vehicle vehicle, out double ixx, out double iyy, out double izz)
    {
        Symmetric3x3 t = vehicle.TotalMassPropsBody.Inertia;
        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out double3 mz);

        double Quadratic(double3 a) =>
            t.XX * a.X * a.X + t.YY * a.Y * a.Y + t.ZZ * a.Z * a.Z +
            2.0 * (t.YX * a.X * a.Y + t.ZX * a.X * a.Z + t.ZY * a.Y * a.Z);

        ixx = Quadratic(mx);
        iyy = Quadratic(my);
        izz = Quadratic(mz);
    }

    /// <summary>
    /// Distance from the centre of mass to the main engine's thrust point, measured
    /// ALONG the thrust axis. This is the model's L_arm: it sets how much pitch/yaw
    /// torque a given gimbal deflection produces (tau = r_T x T_body).
    ///
    /// Taken from the highest-thrust gimbal, which is the main engine on any sane
    /// layout — the same one KsaFrameBridge derives the body axes from, so the two
    /// cannot disagree about which engine they mean.
    /// </summary>
    public static double EngineArm(Vehicle vehicle)
    {
        KsaFrameBridge.BodyAxes(vehicle, out _, out _, out double3 mz);
        // Vehicle's own CoM, not the FlightComputer's copy: the FC field is only
        // refreshed by ReadMeasurements, so at engage time it can still be zero —
        // which would silently measure the engine arm from the assembly origin
        // instead of the centre of mass and hand the solver a wrong LArm.
        float3 com = vehicle.CenterOfMassAsmbF;

        double best = 0.0, arm = 0.0;
        foreach (GimbalController gc in vehicle.Parts.Modules.Get<GimbalController>())
        {
            if (gc.Data.MaximumThrust <= best)
                continue;
            best = gc.Data.MaximumThrust;
            float3 d = gc.Data.ThrustPosVehicleAsmb - com;
            // Negative because the engine sits at -Z in the model's body frame; the
            // model wants the magnitude of that offset.
            arm = Math.Abs(double3.Dot(new double3(d.X, d.Y, d.Z), mz));
        }
        return arm;
    }

    /// <summary>
    /// Peak roll torque the vehicle can actually produce, in N·m — the model's
    /// TauRollMax. Comes from the allocator's honest capability figure (torque at
    /// the point some gimbal saturates), NOT from KSA's TvcTorqueAuthority, which
    /// sums absolute values and so assumes every gimbal serves roll maximally at
    /// once. Planning against an optimistic bound would produce a trajectory the
    /// vehicle cannot fly.
    /// </summary>
    public static double RollTorqueLimit(Vehicle vehicle)
    {
        Span<GimbalController> gimbals = vehicle.Parts.Modules.Get<GimbalController>();
        if (gimbals.Length == 0)
            return 0.0;

        var thrusts = new double[gimbals.Length];
        for (int i = 0; i < gimbals.Length; i++)
            thrusts[i] = gimbals[i].Data.MaximumThrust;

        // Vehicle's own CoM, not the FlightComputer's copy: the FC field is only
        // refreshed by ReadMeasurements, so at engage time it can still be zero —
        // which would silently measure the engine arm from the assembly origin
        // instead of the centre of mass and hand the solver a wrong LArm.
        float3 com = vehicle.CenterOfMassAsmbF;
        var commands = new double[2 * gimbals.Length];
        TvcAllocationResult a = KsaTvcAllocator.Solve(gimbals, thrusts, com, default, commands);

        // MaxTorque is in KSA body axes; roll is about the model's +Z.
        KsaFrameBridge.BodyAxes(vehicle, out _, out _, out double3 mz);
        return Math.Abs(a.MaxTorque.X * mz.X + a.MaxTorque.Y * mz.Y + a.MaxTorque.Z * mz.Z);
    }

    /// <summary>
    /// Peak LATERAL (pitch/yaw) torque the vehicle can actually produce, N·m.
    ///
    /// This matters more than it looks. The model bounds lateral torque implicitly as
    /// LArm * Tmax * tan(gimbal) — a nominal geometry product that assumes ALL the
    /// thrust gimbals through the full angle on the full arm. A real vehicle rarely
    /// obliges: some engines do not gimbal at all, and the binding gimbal limit may
    /// belong to a small vernier. If that product overstates the truth, the solver
    /// plans a trajectory needing torque the allocator simply cannot deliver, the
    /// allocator saturates, and the vehicle does not fly its own plan — with nothing
    /// in the numbers to say why.
    ///
    /// Approximate: MaxTorque is a per-KSA-axis figure rather than a tensor, so each
    /// model lateral axis is scored by how much it projects onto each KSA axis. Erring
    /// toward the conservative (min of the two lateral axes) is the right direction —
    /// planning inside real capability.
    /// </summary>
    public static double LateralTorqueLimit(Vehicle vehicle)
    {
        Span<GimbalController> gimbals = vehicle.Parts.Modules.Get<GimbalController>();
        if (gimbals.Length == 0)
            return 0.0;

        var thrusts = new double[gimbals.Length];
        for (int i = 0; i < gimbals.Length; i++)
            thrusts[i] = gimbals[i].Data.MaximumThrust;

        var commands = new double[2 * gimbals.Length];
        TvcAllocationResult a = KsaTvcAllocator.Solve(
            gimbals, thrusts, vehicle.CenterOfMassAsmbF, default, commands);

        KsaFrameBridge.BodyAxes(vehicle, out double3 mx, out double3 my, out _);
        double Along(double3 axis) =>
            Math.Abs(axis.X) * Math.Abs(a.MaxTorque.X) +
            Math.Abs(axis.Y) * Math.Abs(a.MaxTorque.Y) +
            Math.Abs(axis.Z) * Math.Abs(a.MaxTorque.Z);

        return Math.Min(Along(mx), Along(my));
    }

    /// <summary>Smallest gimbal deflection limit on the vehicle, in degrees — the binding one.</summary>
    public static double GimbalLimitDeg(Vehicle vehicle)
    {
        double minRad = double.PositiveInfinity;
        foreach (GimbalController gc in vehicle.Parts.Modules.Get<GimbalController>())
        {
            Gimbal g = gc.Gimbal;
            double m = Math.Min(g.AxisY.MaxAngle, g.AxisZ.MaxAngle);
            if (m > 0.0)
                minRad = Math.Min(minRad, m);
        }
        return double.IsPositiveInfinity(minRad) ? 0.0 : minRad * 180.0 / Math.PI;
    }

    /// <summary>
    /// Full solver configuration for this vehicle at this site. Returns false if the
    /// vehicle cannot be planned for (no engine, no gimbal, no roll authority) —
    /// better a refusal than a plan built on zeros.
    /// </summary>
    /// <summary>
    /// Thrust-to-weight at the MINIMUM throttle, and the tilt that implies.
    ///
    /// THE SINGLE MOST IMPORTANT NUMBER FOR WHETHER A PLAN EXISTS. The model has a
    /// convex throttle box Tmin &lt;= T &lt;= Tmax, so the vehicle can never thrust below
    /// Tmin while the engine is lit. If Tmin exceeds weight the vehicle CANNOT hold
    /// altitude pointing up — the only way to shed the excess is to tilt until the
    /// vertical component drops to 1 g, needing acos(1/TWRmin) of tilt. Past the tilt
    /// limit no descent exists at all and the solver reports infeasible; just inside
    /// it, the solver tilts to burn off thrust and the path curves away sideways,
    /// which is where the "spiral" trajectories come from.
    ///
    /// The Python reference is tuned to TWRmin ~ 0.98 (min thrust just under hover),
    /// which is why it never shows this. A real vehicle rarely obliges — especially
    /// when Tmax sums every engine on the craft rather than the ones that will be lit.
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

    /// <param name="thrustFraction">
    /// Share of the vehicle's total thrust the landing burn will actually use. An
    /// over-powered booster lands on a SUBSET of its engines; the model has no way to
    /// express "shut three of five down", so scaling Tmax is how that intent is
    /// communicated. This is usually the difference between a feasible plan and a
    /// spiral.
    /// </param>
    /// <param name="x0">Initial model state — needed to SIZE THE PROBLEM, see XScale below.</param>
    /// <param name="xf">Terminal model state, same reason.</param>
    public static bool TryBuild(Vehicle vehicle, IParentBody parent, double3 siteCci,
                                int nodes, double tiltMaxDeg, double throttleFloor,
                                double sigmaSeed, double thrustFraction,
                                double[] x0, double[] xf,
                                out Scvx6DofConfig cfg, out Dynamics6Dof.Params dyn,
                                out string error)
    {
        cfg = null!;
        dyn = null!;
        error = "";

        (double thrust, double massFlow) = KsaEnginePerf.Vacuum(vehicle);
        if (thrust <= 0.0 || massFlow <= 0.0)
        {
            error = "no engine thrust to plan with";
            return false;
        }

        thrust *= Math.Clamp(thrustFraction, 0.01, 1.0);

        double gimbalDeg = GimbalLimitDeg(vehicle);
        if (gimbalDeg <= 0.0)
        {
            error = "no gimballed engine - 6-DOF needs thrust vectoring";
            return false;
        }

        double tauRoll = RollTorqueLimit(vehicle);
        if (tauRoll <= 0.0)
        {
            error = "no roll authority from TVC (no off-axis gimbals)";
            return false;
        }

        double arm = EngineArm(vehicle);
        if (arm <= 0.0)
        {
            error = "engine thrust point coincides with the centre of mass";
            return false;
        }

        // Replace the vehicle's nominal gimbal limit with the EFFECTIVE one implied by
        // the torque the allocator can really deliver:
        //     LArm * Tmax * tan(effective) = measured lateral capability
        // so the model's own gimbal cone bounds the plan to flyable torque. Capped at
        // the physical limit, since the allocator can never beat the hardware, and
        // floored so a measurement glitch cannot collapse the cone to nothing.
        double latCapability = LateralTorqueLimit(vehicle);
        if (latCapability > 0.0)
        {
            double effectiveTan = latCapability / (arm * thrust);
            double effectiveDeg = Math.Atan(effectiveTan) * 180.0 / Math.PI;
            gimbalDeg = Math.Clamp(effectiveDeg, 0.1, gimbalDeg);
        }

        Inertia(vehicle, out double ixx, out double iyy, out double izz);
        if (ixx <= 0.0 || iyy <= 0.0 || izz <= 0.0)
        {
            error = "vehicle inertia is degenerate";
            return false;
        }

        // Refuse an over-powered configuration up front, with the number needed to fix
        // it. Reporting "infeasible" after a 500 ms solve tells the user nothing; this
        // says exactly which knob is wrong and what value would work.
        {
            double gSite = parent.Mu / (siteCci.Length() * siteCci.Length());
            ThrottleMargin(thrust, throttleFloor, vehicle.TotalMass, gSite,
                           out double twrMin, out double needTiltDeg);
            if (needTiltDeg >= tiltMaxDeg)
            {
                double feasibleFloor = Math.Cos(tiltMaxDeg * Math.PI / 180.0) *
                                       vehicle.TotalMass * gSite / thrust;
                error =
                    $"over-powered: min throttle gives TWR {twrMin:F2}, which needs " +
                    $"{needTiltDeg:F0} deg of tilt just to stop climbing (limit {tiltMaxDeg:F0} deg). " +
                    $"Lower the throttle floor below {feasibleFloor:F2}, raise the tilt limit, " +
                    $"or set the thrust fraction to ~{feasibleFloor / Math.Max(throttleFloor, 1e-6):F2} " +
                    "to plan on fewer engines.";
                return false;
            }
        }

        // Gravity at the SITE, from the body actually being landed on — not 9.81.
        // Constant over the descent, which is the model's assumption; see the
        // fidelity notes for when that stops being true.
        double r = siteCci.Length();
        double g = parent.Mu / (r * r);

        const double G0 = 9.80665;
        dyn = new Dynamics6Dof.Params
        {
            Gx = 0.0, Gy = 0.0, Gz = -g,
            G0 = G0,
            Isp = thrust / massFlow / G0,
            LArm = arm,
            Ixx = ixx, Iyy = iyy, Izz = izz,
        };

        // NON-DIMENSIONALISE AGAINST THIS PROBLEM, not against the Python test case.
        //
        // This is the difference between converging and "cold solve failed". The
        // solver works on x~ = x/scale, and THE TRUST REGION IS IN THOSE UNITS,
        // capped at TrustRegionMax = 0.1. So the movement permitted per iteration is
        // 0.1*scale in physical terms. With the scale hard-coded to the reference
        // case's 300 m altitude, that is ~30 m per iteration REGARDLESS of the actual
        // descent: engage from 3 km and the problem is ten times larger while the step
        // stays the same, so the iteration budget can no longer traverse it. The
        // problem is perfectly feasible; the solver simply cannot walk there. It also
        // explains why relaxing constraints does not help — this was never a
        // constraint failure.
        //
        // Gfold learned this already (GfoldPlanner: "length scale ~ the problem size,
        // time scale such that gravity is ~1... ECOS breaks down on the raw SI
        // problem"). Same lesson, applied here.
        //
        // One length scale for all three position axes rather than per-axis: it is
        // the trajectory's own extent, and it keeps the scaling isotropic so a mostly
        // horizontal approach conditions as well as a vertical one. Velocity is
        // scaled by the natural free-fall speed over that length, floored by the
        // actual initial speed so a fast entry is not under-scaled.
        //
        // Quaternion and body-rate scales stay at 1: both are already dimensionless
        // or O(1), and they are the values the reference converges with — no reason
        // to perturb a validated part of the setup.
        double mass = vehicle.TotalMass;
        double gScale = parent.Mu / (siteCci.Length() * siteCci.Length());
        double lengthScale = Math.Max(
            Math.Sqrt((x0[0] - xf[0]) * (x0[0] - xf[0]) +
                      (x0[1] - xf[1]) * (x0[1] - xf[1]) +
                      (x0[2] - xf[2]) * (x0[2] - xf[2])), 1.0);
        double speed0 = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
        double velocityScale = Math.Max(Math.Max(speed0, Math.Sqrt(lengthScale * gScale)), 1.0);
        // Burn-time bounds must BRACKET the seed. Scvx6DofConfig's defaults (5..25 s,
        // scale 12) describe the Python test case; leaving them while seeding sigma
        // elsewhere gives the solver a starting point outside its own feasible range
        // and a badly conditioned scale. Both are set from the seed instead.
        double sigma = Math.Max(sigmaSeed, 1.0);
        cfg = new Scvx6DofConfig
        {
            Nodes = nodes,
            Tmax = thrust,
            ThrottleFloor = throttleFloor,
            GimbalMaxDeg = gimbalDeg,
            TauRollMax = tauRoll,
            TiltMaxDeg = tiltMaxDeg,
            GroundFloor = -1.0,
            SigmaMin = sigma * 0.25,
            SigmaMax = sigma * 2.5,
            SigmaScale = sigma,
            XScale =
            [
                lengthScale, lengthScale, lengthScale,
                velocityScale, velocityScale, velocityScale,
                1, 1, 1, 1,
                1, 1, 1,
                Math.Max(mass, 1.0),
            ],
        };
        return true;
    }
}

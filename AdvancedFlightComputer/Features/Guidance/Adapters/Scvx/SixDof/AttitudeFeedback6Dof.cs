#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// Holds a 6-DOF plan's attitude and body rates between re-solves with a torque on top of the plan's, from the actuators' TORQUE MARGIN: the share of the gimbals' and the RCS's authority the plans leave unused (Scvx6DofConfig.PlanTorqueShare).
///
/// WHY. The plan's torques are flown open loop, and a plan is free to use every newton metre the craft has. On 2026-11-20 at 14:16 the plans after ignition held the gimbals at 94-100 % of their limit for the rest of the burn: the craft followed them, but anything it had drifted by could only wait for the next re-solve, and each re-solve then needed more turn than the last. The thrust has had the same arrangement since 2026-10-08 - plans keep throttle in hand above the floor (LandingPlanFloorMargin) and Tracker6Dof spends it. This is the torque's.
///
/// THE LAW. A PD on the attitude error and the rate error against the plan at the same instant, per body axis, scaled by the inertia: tau = -I (wn^2 theta + 2 zeta wn (w - w_plan)). A plan starts on the measured state, so the error is zero at every re-solve and grows only between them. The bandwidth sits well under the gimbals' slew (a full deflection in half a second).
///
/// THE SPLIT. Each actuator takes the correction in proportion to the authority it has left over the plan's own use, on that axis and in that direction: the gimbals' cone around the plan's lateral torque (pitch and yaw together), their roll range, and the RCS's box. Near the plan's limits that is the margin of each, so the split follows their sizes; an actuator the plan is not using takes more; with the engine off the gimbals have none and the RCS takes it all. A correction larger than both together is cut to fit, so it can never ask for more than the craft has - what the attitude loop of 2026-10-08, which commanded several times the gimbals' torque and spun a booster up in roll, did.
/// </summary>
public sealed record AttitudeFeedback6Dof
{
    /// <summary>Natural frequency of the attitude loop, rad/s, and its damping ratio.</summary>
    public double BandwidthRadS { get; init; } = 1.5;
    public double Damping { get; init; } = 0.8;

    /// <summary>
    /// THE INTEGRAL, or null for none: the attitude error against the plan, integrated over the burn, per body axis, rad s. The PD alone holds a steady moment the plan does not know - what is left of an off-centre engine choice's after the plan's own trim (Dynamics6Dof.Params.ThrustMoment*), the air's misfit - with a standing error of M / (I wn^2), and each re-solve starts on that error. The integral takes it out and, unlike the error, is not reset by a re-solve: the moment it found is still there.
    ///  Its gain is <see cref="IntegralGain"/> wn^3 (s^3 + kd s^2 + kp s + ki stays stable while ki is under kd kp = 2 zeta wn^3), and its torque is capped at <see cref="IntegralCapRad"/> of attitude error's worth of the proportional term. It integrates only while the engine burns, and not on a step whose correction was cut to fit, so it cannot wind up against a saturated actuator.
    /// </summary>
    public Integrator Integral { get; init; }
    public double IntegralGain { get; init; } = 0.25;
    public double IntegralCapRad { get; init; } = 0.1;

    /// <summary>The integral's state, kept for the life of one touchdown engagement (VehicleAutopilotState.SixDofAttitudeIntegral).</summary>
    public sealed class Integrator
    {
        public double3 AngleRadS;
        public double LastTime = double.NaN;
        public bool LastCut;

        public void Reset()
        {
            AngleRadS = default;
            LastTime = double.NaN;
            LastCut = false;
        }
    }

    /// <summary>A gap in the commands longer than this restarts the integral: a new engagement, or a step the feedback did not run.</summary>
    private const double IntegralGapS = 1.0;

    /// <summary>The last correction: attitude and rate error against the plan (deg, deg/s), the torque the law wanted, what went to the gimbals and to the RCS (N m, model body axes), what did not fit (N m), and the integral's part of what the law wanted (N m).</summary>
    public readonly record struct Result(double AttitudeErrorDeg, double RateErrorDegS, double3 Wanted, double3 Gimbal, double3 Rcs, double ShortNm,
                                         double3 IntegralNm = default);

    private const int NX = Dynamics6Dof.NX;

    /// <summary>
    /// The correction at node k + f of the plan for the measured state x (model frame, 14 long).
    /// </summary>
    /// <param name="gimbalPlan">The plan's gimbal torque now, model body axes (pitch, yaw, roll), N m.</param>
    /// <param name="rcsPlan">The plan's RCS torque now, model body axes, N m.</param>
    /// <param name="gimbalLateralNm">The gimbals' whole lateral torque at the thrust now: the radius of their cone in torque, N m. 0 with the engine off.</param>
    /// <param name="gimbalRollNm">The gimbals' whole roll torque at the thrust now, N m.</param>
    /// <param name="rcsPos">The RCS's whole torque per axis, positive direction, N m; null for no RCS.</param>
    /// <param name="rcsNeg">The same, negative direction, as a magnitude.</param>
    /// <param name="now">Simulation time, s, for the integral; NaN leaves it out.</param>
    /// <param name="gimbalCentre">The centre of the gimbals' cone in torque, model body axes, N m: the deflection that holds the engines' own moment (Scvx6DofConfig.GimbalTrimX/Y); zero for a balanced cluster.</param>
    public Result Correct(Ksa6DofPlan plan, int k, double f, double[] x, double ixx, double iyy, double izz,
                          double3 gimbalPlan, double3 rcsPlan, double gimbalLateralNm, double gimbalRollNm,
                          double[] rcsPos, double[] rcsNeg, double now = double.NaN, double3 gimbalCentre = default)
    {
        int k1 = Math.Min(k + 1, plan.Nodes - 1);
        double L(int i) => plan.X[k * NX + i] * (1.0 - f) + plan.X[k1 * NX + i] * f;

        // The plan's attitude here, normalised between its nodes (they are a fraction of a degree apart), and its rates.
        int q = Dynamics6Dof.IQ;
        double sign = plan.X[k * NX + q] * plan.X[k1 * NX + q] + plan.X[k * NX + q + 1] * plan.X[k1 * NX + q + 1]
                    + plan.X[k * NX + q + 2] * plan.X[k1 * NX + q + 2] + plan.X[k * NX + q + 3] * plan.X[k1 * NX + q + 3] < 0.0 ? -1.0 : 1.0;
        double pw = plan.X[k * NX + q] * (1.0 - f) + sign * plan.X[k1 * NX + q] * f;
        double px = plan.X[k * NX + q + 1] * (1.0 - f) + sign * plan.X[k1 * NX + q + 1] * f;
        double py = plan.X[k * NX + q + 2] * (1.0 - f) + sign * plan.X[k1 * NX + q + 2] * f;
        double pz = plan.X[k * NX + q + 3] * (1.0 - f) + sign * plan.X[k1 * NX + q + 3] * f;
        double pn = Math.Sqrt(pw * pw + px * px + py * py + pz * pz);
        if (!(pn > 1e-9))
            return default;
        pw /= pn; px /= pn; py /= pn; pz /= pn;

        // The measured attitude against the plan's, in the plan's body axes: q_m = q_p (x) dq, so dq = conj(q_p) (x) q_m.
        double mw = x[q], mx = x[q + 1], my = x[q + 2], mz = x[q + 3];
        double dw = pw * mw + px * mx + py * my + pz * mz;
        double dx = pw * mx - mw * px - (py * mz - pz * my);
        double dy = pw * my - mw * py - (pz * mx - px * mz);
        double dz = pw * mz - mw * pz - (px * my - py * mx);
        if (dw < 0.0) { dw = -dw; dx = -dx; dy = -dy; dz = -dz; }
        double dv = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        double angle = 2.0 * Math.Atan2(dv, dw);
        double3 theta = dv > 1e-12 ? new double3(dx, dy, dz) * (angle / dv) : default;

        int w = Dynamics6Dof.IW;
        var rateError = new double3(x[w] - L(w), x[w + 1] - L(w + 1), x[w + 2] - L(w + 2));
        double kp = BandwidthRadS * BandwidthRadS, kd = 2.0 * Damping * BandwidthRadS;
        double3 law = theta * kp + rateError * kd;

        // THE INTEGRAL (see Integral): while the engine burns, from the last step's time, held where it is on a step after a cut.
        double3 integralLaw = default;
        bool burning = gimbalLateralNm > 0.0;
        if (Integral != null && double.IsFinite(now) && BandwidthRadS > 0.0 && IntegralGain > 0.0)
        {
            double dt = now - Integral.LastTime;
            if (!(dt >= 0.0 && dt <= IntegralGapS))
            {
                Integral.Reset();
                dt = 0.0;
            }
            Integral.LastTime = now;
            double ki = IntegralGain * BandwidthRadS * BandwidthRadS * BandwidthRadS;
            if (burning)
            {
                if (!Integral.LastCut)
                {
                    double cap = kp * Math.Max(IntegralCapRad, 0.0) / ki;
                    double3 a = Integral.AngleRadS + theta * dt;
                    Integral.AngleRadS = new double3(Math.Clamp(a.X, -cap, cap), Math.Clamp(a.Y, -cap, cap), Math.Clamp(a.Z, -cap, cap));
                }
                integralLaw = Integral.AngleRadS * ki;
                law += integralLaw;
            }
        }

        var wanted = new double3(-ixx * law.X, -iyy * law.Y, -izz * law.Z);
        if (!double.IsFinite(wanted.X) || !double.IsFinite(wanted.Y) || !double.IsFinite(wanted.Z))
            return default;

        // PITCH AND YAW together: along the wanted direction, the gimbals' room is to the edge of their cone around the plan's torque, the RCS's to the nearest wall of its box.
        double3 gimbal = default, rcs = default;
        double shortNm = 0.0;
        double mag = Math.Sqrt(wanted.X * wanted.X + wanted.Y * wanted.Y);
        if (mag > 1e-6)
        {
            double ux = wanted.X / mag, uy = wanted.Y / mag;
            double gRoom = ConeRoom(gimbalPlan.X - gimbalCentre.X, gimbalPlan.Y - gimbalCentre.Y, ux, uy, gimbalLateralNm);
            double rRoom = double.PositiveInfinity;
            if (rcsPos != null && rcsNeg != null)
            {
                rRoom = Math.Min(rRoom, BoxRoom(rcsPlan.X, ux, Bound(rcsPos, 0), Bound(rcsNeg, 0)));
                rRoom = Math.Min(rRoom, BoxRoom(rcsPlan.Y, uy, Bound(rcsPos, 1), Bound(rcsNeg, 1)));
            }
            if (!double.IsFinite(rRoom)) rRoom = 0.0;
            Split(mag, gRoom, rRoom, out double toGimbal, out double toRcs, out double cut);
            gimbal = new double3(toGimbal * ux, toGimbal * uy, 0.0);
            rcs = new double3(toRcs * ux, toRcs * uy, 0.0);
            shortNm += cut;
        }

        // ROLL: the gimbals' roll range and the RCS's roll, each around the plan's own.
        if (Math.Abs(wanted.Z) > 1e-6)
        {
            double u = Math.Sign(wanted.Z);
            double gRoom = BoxRoom(gimbalPlan.Z, u, gimbalRollNm, gimbalRollNm);
            double rRoom = rcsPos != null && rcsNeg != null ? BoxRoom(rcsPlan.Z, u, Bound(rcsPos, 2), Bound(rcsNeg, 2)) : 0.0;
            Split(Math.Abs(wanted.Z), gRoom, rRoom, out double toGimbal, out double toRcs, out double cut);
            gimbal = new double3(gimbal.X, gimbal.Y, toGimbal * u);
            rcs = new double3(rcs.X, rcs.Y, toRcs * u);
            shortNm += cut;
        }

        if (Integral != null)
            Integral.LastCut = shortNm > 1e-3 * (Math.Sqrt(double3.Dot(wanted, wanted)) + 1.0);
        double rad = 180.0 / Math.PI;
        return new Result(angle * rad, Math.Sqrt(double3.Dot(rateError, rateError)) * rad, wanted, gimbal, rcs, shortNm,
                          new double3(-ixx * integralLaw.X, -iyy * integralLaw.Y, -izz * integralLaw.Z));
    }

    /// <summary>The correction shared in proportion to each actuator's room, cut to the room there is.</summary>
    private static void Split(double want, double gRoom, double rRoom, out double toGimbal, out double toRcs, out double cut)
    {
        gRoom = Math.Max(gRoom, 0.0);
        rRoom = Math.Max(rRoom, 0.0);
        double room = gRoom + rRoom;
        toGimbal = toRcs = 0.0;
        cut = want;
        if (!(room > 0.0))
            return;
        double given = Math.Min(want, room);
        toGimbal = given * gRoom / room;
        toRcs = given * rRoom / room;
        cut = want - given;
    }

    /// <summary>How far from (px, py) along the unit (ux, uy) stays inside the circle of the given radius; 0 outside it.</summary>
    private static double ConeRoom(double px, double py, double ux, double uy, double radius)
    {
        if (!(radius > 0.0))
            return 0.0;
        double b = px * ux + py * uy;
        double c = px * px + py * py - radius * radius;
        double disc = b * b - c;
        return disc > 0.0 ? Math.Max(-b + Math.Sqrt(disc), 0.0) : 0.0;
    }

    /// <summary>How far one axis can move from the plan's value along the unit component u before it reaches [-neg, pos]; infinite for an axis the direction does not move.</summary>
    private static double BoxRoom(double plan, double u, double pos, double neg)
    {
        if (Math.Abs(u) < 1e-9)
            return double.PositiveInfinity;
        double room = u > 0.0 ? pos - plan : neg + plan;
        return Math.Max(room, 0.0) / Math.Abs(u);
    }

    private static double Bound(double[] v, int a) => v != null && a < v.Length && double.IsFinite(v[a]) ? Math.Max(v[a], 0.0) : 0.0;
}

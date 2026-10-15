#nullable disable

namespace AdvancedFlightComputer.Features.Guidance;

using System;
using Brutal.Numerics;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;

/// <summary>
/// Flies a 6-DOF plan between re-solves with feedback on the THRUST alone: the plan's axial thrust, scaled to the measured mass, plus PD feedback on the plan's position and velocity taken along the engine's measured axis. The plan's gimbal and roll torques pass through untouched.
///
/// WHY THRUST ONLY. The plan is already acceleration-level - its controls are thrust and torques, and its attitude is a state those torques steer - so a correction that needs a different attitude can only be had by re-planning. An attitude loop layered on top (the first version of this tracker, 2026-10-08) commanded several times the torque the gimbals could deliver, spun a booster up in roll, and took it so far off the plan that every re-solve was refused. A change of thrust along the axis the engine is pointing changes the acceleration at once, with no attitude dynamics in between: it is what corrects a burn that is slowing more or less than planned, which is what the plan's floor margin (LandingPlanFloorMargin) keeps throttle in hand for. Sideways errors are the re-solves' to correct.
///
/// The reference is read at the current time, not a node ahead: a plan starts at the measured state, so reading ahead would feed a step of expected motion back as error.
/// </summary>
public sealed record Tracker6Dof
{
    /// <summary>Position gain (1/s^2) and velocity gain (1/s): the 3-DOF burn tracker's, 0.5 rad/s at a damping ratio of 0.8.</summary>
    public double Kp { get; init; } = 0.25;
    public double Kd { get; init; } = 0.8;

    /// <summary>Largest feedback acceleration, m/s^2, so a plan gone stale through refusals cannot ask for an absurd correction.</summary>
    public double MaxFeedbackAccel { get; init; } = 5.0;

    /// <summary>What the last correction was, for the log and the panel: position and velocity off the plan (m, m/s), and the thrust it added (N, negative for taken off).</summary>
    public readonly record struct Error(double PositionM, double VelocityMs, double ThrustN);

    /// <summary>
    /// The plan's axial thrust at node k + f, corrected for the measured state x (model frame, 14 long), N.
    /// </summary>
    public double Correct(Ksa6DofPlan plan, int k, double f, double[] x, double thrust, out Error error)
    {
        const int NX = 14;
        int k1 = Math.Min(k + 1, plan.Nodes - 1);
        double L(int i) => plan.X[k * NX + i] * (1.0 - f) + plan.X[k1 * NX + i] * f;

        double massPlan = L(Dynamics6Dof.IM);
        double massNow = x[Dynamics6Dof.IM];
        error = default;
        if (!(massPlan > 0.0) || !(massNow > 0.0))
            return thrust;

        // Position and velocity on the plan's cubic segment, as MeasureDrift and the warm seed sample it: linear blending puts a braking plan's midpoint off its own arc (10 m at 20 m/s^2 over a 2 s interval), which the gains then fly as error.
        Span<double> planR = stackalloc double[3];
        Span<double> planV = stackalloc double[3];
        double dt = plan.Nodes > 1 ? plan.Sigma / (plan.Nodes - 1) : 0.0;
        PlanStateInterpolation.SamplePositionVelocity(plan.X, k, k1, f, dt, planR, planV);

        double[] fb = new double[3];
        double posErr2 = 0.0, velErr2 = 0.0;
        for (int i = 0; i < 3; i++)
        {
            double dr = planR[i] - x[Dynamics6Dof.IR + i];
            double dv = planV[i] - x[Dynamics6Dof.IV + i];
            posErr2 += dr * dr;
            velErr2 += dv * dv;
            fb[i] = Kp * dr + Kd * dv;
        }
        double fl = Math.Sqrt(fb[0] * fb[0] + fb[1] * fb[1] + fb[2] * fb[2]);
        if (fl > MaxFeedbackAccel)
            for (int i = 0; i < 3; i++)
                fb[i] *= MaxFeedbackAccel / fl;

        // The engine's measured axis, body +z, in the model frame: the third column of R(q).
        double qn = Math.Sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
        if (!(qn > 0.0))
            return thrust;
        double qw = x[6] / qn, qx = x[7] / qn, qy = x[8] / qn, qz = x[9] / qn;
        double bx = 2.0 * (qx * qz + qw * qy);
        double by = 2.0 * (qy * qz - qw * qx);
        double bz = 1.0 - 2.0 * (qx * qx + qy * qy);

        // What the plan's thrust meant as an acceleration, delivered at the measured mass, plus the feedback along the axis.
        double axial = thrust * massNow / massPlan + massNow * (fb[0] * bx + fb[1] * by + fb[2] * bz);
        axial = Math.Max(axial, 0.0);
        error = new Error(Math.Sqrt(posErr2), Math.Sqrt(velErr2), axial - thrust);
        return axial;
    }
}

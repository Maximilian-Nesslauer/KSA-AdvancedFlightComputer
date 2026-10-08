namespace AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// Flies a burn plan between re-solves: the plan's thrust acceleration as feed-forward, plus PD feedback on the plan's position and velocity, as G-FOLD's tracker does.
///
/// WHY IT IS NEEDED. Each re-solve starts from the measured state, so a fresh plan has no error to correct; but between re-solves, and through a run of refused ones, the plan alone is open loop. A min-fuel burn also rides its throttle ceiling, so a burn that falls behind - lit late, an engine weaker than learned, more drag than the table - has nothing in hand unless the ceiling the PLAN sees is below what the ENGINE can give. The guidance plans to <see cref="Guidance3Dof.Settings.BurnThrottleCeiling"/>; this feedback is what may use the rest.
///
/// ALONG THE AXIS, THROTTLE; ACROSS IT, A SMALL TILT. The correction's component along the plan's body axis changes the thrust and nothing else. Its component across the axis can only be had by tilting, and in this model the body axis is also the attitude the aerodynamics were planned around: at Mach 2 the air does a large share of the braking, so a large tilt would buy the correction with lift and drag the plan never saw. The tilt is held inside <see cref="MaxTiltRad"/> of the plan's axis, and the thrust is what the corrected acceleration has along the direction actually commanded, so a clamped lateral demand never turns into extra thrust along the axis.
///
/// FEED-FORWARD IS WHAT THE PLAN MEANT. The plan's throttle by the thrust law of the model it was solved with (<see cref="Plan3Dof.Model"/>), over the plan's mass, is the acceleration it means. Not the guidance's current model: that moves on with every thrust estimate, refused re-solves included, and read through it a stale plan would ask for less than it was flying. The newtons that acceleration costs come from the measured mass, so a mass the plan has slightly wrong does not show up as a tracking error either.
///
/// The reference is read at the current time, not a node ahead: a plan starts at the measured state, so reading ahead would feed a step of expected motion back as error.
/// </summary>
public sealed record BurnTracker3Dof
{
    /// <summary>
    /// Position gain, 1/s^2, and velocity gain, 1/s: 0.5 rad/s at a damping ratio of 0.8. G-FOLD's 0.08 and 0.3 left a 30 % lift deficit 7 m off target at 2.5 m/s offline; these land it inside a metre, and every --3dof-mpc case passed against the 20 m, 2 m/s aim point of the time. Swept offline from 0.08/0.3 to 0.3/1.2 (--track-kp, --track-kd): the velocity gain is what matters, and above about 1 the combined case's cross-range gets worse again.
    /// </summary>
    public double Kp { get; init; } = 0.25;
    public double Kd { get; init; } = 0.8;

    /// <summary>Largest feedback acceleration, m/s^2, so a plan gone stale through refusals cannot ask for an absurd correction.</summary>
    public double MaxFeedbackAccel { get; init; } = 5.0;

    /// <summary>Largest tilt of the command off the plan's body axis, rad.</summary>
    public double MaxTiltRad { get; init; } = 10.0 * Math.PI / 180.0;

    /// <summary>
    /// The thrust to fly now, N, and its direction, model frame, for a vehicle at <paramref name="x"/> on <paramref name="plan"/>. The newtons are what the vehicle should actually deliver; turning them into a throttle is the caller's, by what it has learned of the engine. Zero thrust when the plan has no direction or no model.
    /// </summary>
    public double Command(Plan3Dof plan, double time, double[] x, out double[] axis)
    {
        var xr = new double[PointMass3Dof.NX];
        var ur = new double[PointMass3Dof.NU];
        plan.Sample(time, xr, ur);
        double bl = Math.Sqrt(ur[0] * ur[0] + ur[1] * ur[1] + ur[2] * ur[2]);
        KsaPointMassModel? model = plan.Model;
        if (!(bl > 0.5) || model == null)
        {
            axis = bl > 0.5 ? [ur[0] / bl, ur[1] / bl, ur[2] / bl] : [0, 0, 1];
            return 0.0;
        }
        double[] b = [ur[0] / bl, ur[1] / bl, ur[2] / bl];

        double massRef = xr[PointMass3Dof.IM];
        double feedForward = massRef > 0.0 ? model.Thrust(ur[PointMass3Dof.ITH], model.Altitude(xr)) / massRef : 0.0;

        double[] fb = new double[3];
        for (int i = 0; i < 3; i++)
            fb[i] = Kp * (xr[PointMass3Dof.IR + i] - x[PointMass3Dof.IR + i]) + Kd * (xr[PointMass3Dof.IV + i] - x[PointMass3Dof.IV + i]);
        double fl = Math.Sqrt(fb[0] * fb[0] + fb[1] * fb[1] + fb[2] * fb[2]);
        if (fl > MaxFeedbackAccel)
            for (int i = 0; i < 3; i++)
                fb[i] *= MaxFeedbackAccel / fl;

        double[] a = [feedForward * b[0] + fb[0], feedForward * b[1] + fb[1], feedForward * b[2] + fb[2]];
        double al = Math.Sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        // A correction that points the acceleration behind the axis asks for no thrust at all, and has no tilt worth taking; Slew has no axis to turn about at 180 degrees.
        if (!(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] > 0.0))
        {
            axis = b;
            return 0.0;
        }
        axis = Guidance3Dof.Slew(b, [a[0] / al, a[1] / al, a[2] / al], MaxTiltRad);
        double along = a[0] * axis[0] + a[1] * axis[1] + a[2] * axis[2];
        return Math.Max(along, 0.0) * x[PointMass3Dof.IM];
    }
}

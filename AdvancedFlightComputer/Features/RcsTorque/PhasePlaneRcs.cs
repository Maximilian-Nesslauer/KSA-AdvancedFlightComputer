namespace AdvancedFlightComputer.Features.RcsTorque;

/// <summary>
/// One axis of a phase-plane jet controller, the stock flight computer's (KSA FlightComputer.ComputeRcsTrackAxis and its switching lines, 2026.11.16.5901), restated over plain numbers so any controller can use it and it can be checked offline.
///
/// THE PHASE PLANE. The state is the error angle (where the craft points less where it should, rad) and its rate (rad/s). Two switching lines bound a region the jets leave alone: inside it the craft coasts. They are the parabolas the jets can brake along, rate = -sqrt(2 a |angle - edge|), from either edge of the angle band widened by a turnaround margin, flattening near the band to a small rate margin (1.5 times the rate band). Crossing one, the axis goes Above or Below and fires: each call it asks for the rate change that puts the craft on the target line, T(angle) - rate, which inside the band is a drift of half a rate bit towards the centre and outside it the braking parabola onto the band's edge, capped at the rate limit. It stops once the rate reaches the target line, or once it is back inside the band and heading in. So it fires once, as one impulse sized to the drift it wants, and coasts the band from edge to edge, instead of pushing in proportion to the error every step.
///
/// THE SIZES, as stock sizes them: the rate bit is the least rate change a pulse gives; the rate band that plus the target's own turning rate; the angle band no narrower than two control steps' drift at the rate bit; the rate limit no lower than the rate band; the turnaround the stopping distance at full authority from the rate band, and at least 10 s of drift at it.
/// </summary>
internal static class PhasePlaneRcs
{
    internal enum State : byte { Inside, Above, Below }

    /// <param name="AuthorityRadS2">The jets' angular acceleration about the axis, rad/s^2.</param>
    /// <param name="RateBitRadS">The least rate change one pulse gives, rad/s.</param>
    /// <param name="TargetRateRadS">How fast the target itself turns, rad/s: it widens the rate band, as stock's does.</param>
    /// <param name="AngleBandRad">The angle band's whole width, rad (stock's AngleDeadband: Balanced 1 deg).</param>
    /// <param name="RateLimitRadS">The fastest the target line closes an error, rad/s (stock's RateLimit: Balanced 5 deg/s).</param>
    /// <param name="StepS">The time between commands the jets can act on, s.</param>
    /// <param name="RateBandFloorRadS">The narrowest the rate band may be, rad/s, beyond stock (0: stock's, the rate bit alone). Stock's jets are the only thing turning their axis; where air and other actuators turn it too, a rate band of one rate bit is crossed by every disturbance and the jets pulse without end.</param>
    internal readonly record struct Axis(double AuthorityRadS2, double RateBitRadS, double TargetRateRadS, double AngleBandRad, double RateLimitRadS, double StepS,
                                         double RateBandFloorRadS = 0.0)
    {
        public double RateBand => Math.Max(RateBitRadS + Math.Abs(TargetRateRadS), RateBandFloorRadS);
        public double Band => Math.Max(AngleBandRad, 2.0 * StepS * RateBitRadS);
        public double RateLimit => Math.Max(RateLimitRadS, RateBand);
        public double Turnaround => Math.Max(RateBand * RateBand / AuthorityRadS2, 10.0 * RateBand);
    }

    /// <summary>
    /// One call on one axis: the rate change to command now, rad/s (0 while it coasts), with <paramref name="state"/> carried from call to call. <paramref name="angle"/> is the error (the craft less the target), rad; <paramref name="rate"/> its rate, rad/s. No authority: nothing, and the state reset.
    /// </summary>
    internal static double Step(in Axis p, double angle, double rate, ref State state)
    {
        double a = p.AuthorityRadS2;
        if (!(a > 0.0) || !double.IsFinite(angle) || !double.IsFinite(rate))
        {
            state = State.Inside;
            return 0.0;
        }
        double rd = p.RateBand, band = p.Band, limit = p.RateLimit, turn = p.Turnaround;
        double upper = UpperLine(angle, turn, rd, a, band, limit);
        double lower = LowerLine(angle, turn, rd, a, band, limit);
        double target = TargetLine(angle, turn, p.RateBitRadS, a, band, limit);
        bool near = Math.Abs(angle) <= 0.5 * band + turn;
        if (rate > upper)
            state = State.Above;
        else if (rate < lower)
            state = State.Below;
        switch (state)
        {
            case State.Above:
                if (rate <= target || (near && rate < 0.0))
                    state = State.Inside;
                break;
            case State.Below:
                if (rate >= target || (near && rate > 0.0))
                    state = State.Inside;
                break;
        }
        if (state == State.Inside)
            return 0.0;
        // Never less than one pulse: a request under the rate bit is carried by the thrusters to the next period and lands doubled (offline, a strong RCS fired 3 times the impulse it needed that way).
        double dw = target - rate;
        return Math.Abs(dw) < p.RateBitRadS ? Math.CopySign(p.RateBitRadS, dw) : dw;
    }

    /// <summary>
    /// The jets as a backup on an axis something else flies (the surfaces), as stock never needs one: they fire only while the error is past <paramref name="halfBandRad"/> and still growing, for the rate change that stops it growing, and stop once it is back inside or no longer growing. They never hurry the other actuator's own return, and leave alone every rate it makes inside the band: the phase plane's switching lines, run there, fired on the flaps' every move (2026-11-22, the RCS against the flaps 70 % of the time from 0.5 kPa). Requests are at least one rate bit, as in Step.
    /// </summary>
    internal static double Backup(double halfBandRad, double rateBitRadS, double angle, double rate, ref State state)
    {
        if (!double.IsFinite(angle) || !double.IsFinite(rate))
        {
            state = State.Inside;
            return 0.0;
        }
        bool outside = Math.Abs(angle) > halfBandRad, growing = angle * rate > 0.0;
        if (state == State.Inside)
        {
            if (outside && growing)
                state = angle > 0.0 ? State.Above : State.Below;
        }
        else if (!outside || !growing)
            state = State.Inside;
        if (state == State.Inside)
            return 0.0;
        double dw = -rate;
        return Math.Abs(dw) < rateBitRadS ? Math.CopySign(rateBitRadS, dw) : dw;
    }

    /// <summary>The braking parabola through <paramref name="offset"/>: the rate from which full authority stops there.</summary>
    internal static double Parabola(double angle, double a, double offset)
    {
        double n = angle - offset;
        return -Math.CopySign(Math.Sqrt(2.0 * Math.Abs(n) * a), n);
    }

    /// <summary>The angle at which the parabola through <paramref name="offset"/> has <paramref name="rate"/>.</summary>
    internal static double ParabolaInverse(double rate, double a, double offset) => -rate * Math.Abs(rate) / (2.0 * a) + offset;

    internal static double TargetLine(double angle, double turn, double rateBit, double a, double band, double limit)
    {
        double edge = 0.5 * band + turn;
        if (angle <= ParabolaInverse(limit, a, -edge))
            return limit;
        if (angle >= ParabolaInverse(-limit, a, edge))
            return -limit;
        double drift = 0.5 * rateBit;
        if (angle <= ParabolaInverse(drift, a, -edge))
            return Parabola(angle, a, -edge);
        if (angle >= ParabolaInverse(-drift, a, edge))
            return Parabola(angle, a, edge);
        return angle >= 0.0 ? -drift : drift;
    }

    internal static double LowerLine(double angle, double turn, double rd, double a, double band, double limit)
    {
        double cap = limit - rd, inner = -0.5 * band, offset = inner - turn;
        if (angle <= ParabolaInverse(cap, a, offset))
            return cap;
        if (angle <= ParabolaInverse(0.0, a, offset))
            return Parabola(angle, a, offset);
        if (angle < inner)
            return 0.0;
        double margin = -1.5 * rd;
        if (angle < ParabolaInverse(margin, a, 0.0))
            return margin;
        return Parabola(angle, a, 0.0);
    }

    internal static double UpperLine(double angle, double turn, double rd, double a, double band, double limit)
    {
        double cap = -limit + rd, inner = 0.5 * band, offset = inner + turn;
        if (angle >= ParabolaInverse(cap, a, offset))
            return cap;
        if (angle >= ParabolaInverse(0.0, a, offset))
            return Parabola(angle, a, offset);
        if (angle > inner)
            return 0.0;
        double margin = 1.5 * rd;
        if (angle > ParabolaInverse(margin, a, 0.0))
            return margin;
        return Parabola(angle, a, 0.0);
    }
}

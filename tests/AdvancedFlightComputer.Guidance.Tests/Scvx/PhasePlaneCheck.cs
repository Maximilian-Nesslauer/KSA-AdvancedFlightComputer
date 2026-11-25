using AdvancedFlightComputer.Features.RcsTorque;

/// <summary>
/// --phase-plane: one axis held by the RCS, the phase-plane law (PhasePlaneRcs) against the attitude controller's continuous law with and without the 1 deg dead band of 2026-11-22, on a rigid body fired as KsaRcsTorqueControl fires it: the torque asked for at the start of each 0.1 s period, as a duty of full authority, pulses under the minimum pulse carried over. Not in CI: a tool for judging the law before it flies.
/// </summary>
internal static class PhasePlaneCheck
{
    const double Deg = Math.PI / 180.0;
    const double Period = 0.1, ControlDt = 1.0 / 90.0, MinPulse = 0.04;

    enum Law { PhasePlane, Continuous, DeadBand }

    sealed record Case(string Name, double AuthorityDegS2, double DisturbShare, double StartDeg, double StartRateDegS, double StepDeg = 0, double RampDegS = 0, double RampS = 0);

    public static int Run()
    {
        var cases = new[]
        {
            new Case("hold, calm", 2, 0.0, 0.3, 0.05),
            new Case("hold, air torque 2 %", 2, 0.02, 0.0, 0.0),
            new Case("hold, air torque 20 %", 2, 0.20, 0.0, 0.0),
            new Case("step 10 deg", 2, 0.0, 0.0, 0.0, StepDeg: 10),
            new Case("turn 5 deg/s for 5 s", 2, 0.0, 0.0, 0.0, RampDegS: 5, RampS: 5),
            new Case("hold, strong RCS, air 5 %", 20, 0.05, 0.0, 0.0),
            new Case("turn 5 deg/s, strong RCS", 20, 0.0, 0.0, 0.0, RampDegS: 5, RampS: 5),
        };
        Console.WriteLine("  300 s each; impulse = total rate change fired (propellant), deg/s; pulses = periods that fired; error after the first 20 s");
        Console.WriteLine($"  {"case",-28} {"law",-12} {"impulse",8} {"pulses",7} {"|err| p50",9} {"max",6}");
        foreach (var c in cases)
        {
            foreach (Law law in new[] { Law.PhasePlane, Law.Continuous, Law.DeadBand })
            {
                var (impulse, pulses, p50, max) = Fly(c, law, 1);
                Console.WriteLine(FormattableString.Invariant($"  {c.Name,-28} {law,-12} {impulse,8:F1} {pulses,7} {p50,9:F2} {max,6:F2}"));
            }
            foreach (int warp in new[] { 10, 30 })
            {
                var (impulse, pulses, p50, max) = Fly(c, Law.PhasePlane, warp);
                Console.WriteLine(FormattableString.Invariant($"  {c.Name,-28} {"PP warp " + warp + "x",-12} {impulse,8:F1} {pulses,7} {p50,9:F2} {max,6:F2}"));
            }
            Console.WriteLine();
        }
        EntryAxis();
        return 0;
    }

    /// <summary>
    /// One axis of an entry: flaps (a 30 deg/s rate limit over 90 deg of travel) whose authority rises from nothing to 4 deg/s^2 over 600 s as the air thickens, the belly flop's RCS (4.4 deg/s^2, a rate bit of 0.007 deg/s, 5 ms pulses), random air torque, and a 20 deg turn at 5 deg/s every minute. The flaps fly the controller's law (1 rad/s, damping 0.8, capture capped at 10 deg/s) on their axis. Old: the axis goes to the flaps at a tenth of the RCS's authority, the RCS backs them on the phase plane with a band three times wider, and its rate band is the rate bit (the 23:47 flight). New: the flaps take it at what the law needs to hold the band (or the RCS's authority), the RCS backs them only past 1.5 deg while the error grows, and its own rate band is floored at the band over 20 s.
    /// </summary>
    static void EntryAxis()
    {
        Console.WriteLine("  entry axis, 600 s, flap authority 0 -> 4 deg/s2; per band of flap authority: RCS impulse deg/s | RCS pulses/min | |err| p50/p90 deg | flap travel deg/min | RCS owned %");
        foreach (bool fresh in new[] { false, true })
        {
            var bins = new double[4, 7];
            double a = 4.4 * Deg, rb = 0.007 * Deg, pulse = 0.005, band = 1.0 * Deg;
            double theta = 0, w = 0, tgt = 0, tgtRate = 0, u = 0, d = 0, turned = 0, turnDir = 1;
            var state = PhasePlaneRcs.State.Inside;
            bool rcsOwns = true;
            double torque = 0, fireUntil = 0, fireSign = 0, carry = 0, periodStart = -1;
            var rng = new Random(5);
            var errs = new List<double>[4];
            for (int k = 0; k < 4; k++) errs[k] = new List<double>();
            for (double t = 0; t < 600; t += ControlDt)
            {
                double af = 4.0 * Deg * t / 600.0;
                double phase = t % 60.0;
                // The craft's spare acceleration as the controller sees it: old, flaps and RCS summed on every axis; new, the axis's owner alone.
                double spare = fresh ? (rcsOwns ? a : af) : af + a;
                // A 20 deg turn each minute, slewed as the entry slews its bank: up to 5 deg/s, accelerating at half the spare.
                if (phase < ControlDt) { turned = 0; turnDir = -turnDir; }
                double left = phase >= 30 ? 20 * Deg - turned : 0.0;
                double slewAcc = 0.5 * spare;
                double slewWant = left > 0 ? Math.Min(5 * Deg, Math.Sqrt(2 * slewAcc * left)) : 0.0;
                double slew = Math.Abs(tgtRate);
                slew += Math.Clamp(slewWant - slew, -slewAcc * ControlDt, slewAcc * ControlDt);
                tgtRate = turnDir * slew;
                turned += slew * ControlDt;
                tgt += tgtRate * ControlDt;
                d += (-d / 2.0) * ControlDt + 0.03 * Deg * Math.Sqrt(ControlDt) * (2 * rng.NextDouble() - 1) * 3;
                double angle = theta - tgt, rate = w - tgtRate;
                // Owner.
                if (fresh)
                {
                    double need = Math.Min(1.0 * 1.0 * band, a);
                    rcsOwns = rcsOwns ? !(af >= need) : af < 0.7 * need;
                }
                else
                    rcsOwns = rcsOwns ? af < 0.1 * a : af < 0.07 * a;
                // Flaps: the law on their axis, nothing on the RCS's.
                double uCmd = 0;
                if (!rcsOwns && af > 0)
                {
                    // The controller's square-root capture, at half the spare acceleration.
                    double lin = 1.0 / 1.6, capA = 0.5 * spare;
                    double mag = Math.Abs(angle) * lin;
                    double thetaLin = capA / (lin * lin);
                    if (Math.Abs(angle) > thetaLin) mag = Math.Sqrt(2 * capA * (Math.Abs(angle) - 0.5 * thetaLin));
                    double want = -Math.Sign(angle) * Math.Min(mag, 10 * Deg);
                    uCmd = Math.Clamp(1.6 * (want - rate) / af, -1, 1);
                }
                u += Math.Clamp(uCmd - u, -ControlDt / 3.0, ControlDt / 3.0);
                // RCS.
                double dw;
                if (rcsOwns)
                {
                    var p = new PhasePlaneRcs.Axis(a, rb, 0, band, 5 * Deg, Period, fresh ? band / 20.0 : 0.0);
                    dw = PhasePlaneRcs.Step(p, angle, rate, ref state);
                }
                else if (fresh)
                    dw = PhasePlaneRcs.Backup(1.5 * band, rb, angle, rate, ref state);
                else
                {
                    var p = new PhasePlaneRcs.Axis(a, rb, 0, 3 * band, 5 * Deg, Period);
                    dw = PhasePlaneRcs.Step(p, angle, rate, ref state);
                }
                torque = Math.Clamp(dw / Period, -a, a);
                bool fired = false;
                if (periodStart < 0 || t - periodStart >= 0.999 * Period)
                {
                    periodStart = t;
                    double want = Math.Abs(torque) / a * Period + carry;
                    if (want > 0 && want >= pulse) { double on = Math.Min(want, Period); fireUntil = t + on; carry = Math.Min(want - on, Period); fireSign = Math.Sign(torque); fired = true; }
                    else { fireUntil = t; carry = want; }
                }
                double on2 = Math.Clamp(fireUntil - t, 0.0, ControlDt);
                double acc = d + af * u + fireSign * a * on2 / ControlDt;
                w += acc * ControlDt;
                theta += w * ControlDt;
                int bin = Math.Min((int)(af / Deg), 3);
                bins[bin, 0] += a * on2 / Deg;
                bins[bin, 1] += fired ? 1 : 0;
                bins[bin, 2] += ControlDt;
                bins[bin, 3] += Math.Abs(Math.Clamp(uCmd - u, -ControlDt / 3.0, ControlDt / 3.0)) * 90.0;
                bins[bin, 4] += rcsOwns ? ControlDt : 0;
                if (t > 5) errs[bin].Add(Math.Abs(angle) / Deg);
            }
            for (int k = 0; k < 4; k++)
            {
                var e = errs[k]; e.Sort();
                double T = bins[k, 2];
                Console.WriteLine(FormattableString.Invariant($"    {(fresh ? "new" : "old")} flaps {k}-{k + 1} deg/s2: {bins[k, 0],7:F1} | {bins[k, 1] / T * 60,6:F1} | {e[e.Count / 2],5:F2}/{e[(int)(0.9 * (e.Count - 1))],5:F2} | {bins[k, 3] / T * 60,6:F0} | {100 * bins[k, 4] / T,3:F0}"));
            }
        }
    }

    static (double Impulse, int Pulses, double P50, double Max) Fly(Case c, Law law, int warp)
    {
        double a = c.AuthorityDegS2 * Deg, d = c.DisturbShare * a;
        double theta = c.StartDeg * Deg, w = c.StartRateDegS * Deg;
        double tgt = c.StepDeg * Deg, tgtRate = 0.0;
        var state = PhasePlaneRcs.State.Inside;
        bool outside = false;
        double torque = 0.0, fireUntil = 0.0, fireSign = 0.0, carry = 0.0, periodStart = -1.0;
        double impulse = 0.0;
        int pulses = 0;
        var errs = new List<double>();
        double max = 0.0;
        // Under warp the law runs once a frame, warp frames of game time apart, while the RCS fires period after period on its last request; the physics steps at ControlDt throughout.
        double hold = warp * ControlDt;
        long step = 0;
        for (double t = 0.0; t < 300.0; t += ControlDt, step++)
        {
            tgtRate = c.RampS > 0 && t < c.RampS ? c.RampDegS * Deg : 0.0;
            double angle = theta - tgt, rate = w - tgtRate;
            if (step % warp == 0)
            switch (law)
            {
                case Law.PhasePlane:
                {
                    var p = new PhasePlaneRcs.Axis(a, a * MinPulse, 0.0, 1.0 * Deg, 5.0 * Deg, Math.Max(hold, Period));
                    double dw = PhasePlaneRcs.Step(p, angle, rate, ref state);
                    torque = Math.Clamp(dw / Math.Max(hold, Period), -a, a);
                    break;
                }
                case Law.Continuous:
                case Law.DeadBand:
                {
                    // The controller's law at 1 rad/s, damping 0.8, its capture capped at 10 deg/s; the dead band on at 1 deg or 1 deg/s, off under half.
                    double want = Math.Clamp(-angle * 1.0 / 1.6, -10 * Deg, 10 * Deg);
                    double cmd = 1.6 * (want - rate) - d; // with the air moment fed forward, as the controller flies
                    if (law == Law.DeadBand)
                    {
                        double e = Math.Abs(angle) / Deg, r = Math.Abs(rate) / Deg;
                        outside = outside ? !(e < 0.5 && r < 0.5) : (e > 1.0 || r > 1.0);
                        if (!outside) cmd = 0.0;
                    }
                    torque = Math.Clamp(cmd, -a, a);
                    break;
                }
            }
            // The RCS: the torque asked for at each period's start, as a duty, the minimum pulse carried.
            if (periodStart < 0 || t - periodStart >= 0.999 * Period)
            {
                periodStart = t;
                double want = Math.Abs(torque) / a * Period + carry;
                if (want > 0 && want >= MinPulse)
                {
                    double on = Math.Min(want, Period);
                    fireUntil = t + on;
                    carry = Math.Min(want - on, Period);
                    fireSign = Math.Sign(torque);
                    pulses++;
                }
                else
                {
                    fireUntil = t;
                    carry = want; // as KsaRcsTorqueControl's worker: a short pulse waits for the next
                }
            }
            double on2 = Math.Clamp(fireUntil - t, 0.0, ControlDt);
            double acc = d + fireSign * a * on2 / ControlDt;
            impulse += a * on2;
            w += acc * ControlDt;
            theta += w * ControlDt;
            tgt += tgtRate * ControlDt;
            if (t > 20.0)
            {
                double e = Math.Abs(theta - tgt) / Deg;
                errs.Add(e);
                max = Math.Max(max, e);
            }
        }
        errs.Sort();
        return (impulse / Deg, pulses, errs[errs.Count / 2], max);
    }
}

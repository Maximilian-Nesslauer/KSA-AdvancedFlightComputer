using System;
using System.Collections.Generic;
using Brutal.Numerics;

namespace PoweredGuidance.Upfg;

// Unified Powered Flight Guidance — the closed-loop ascent algorithm flown by the
// Space Shuttle. Each call to Step() refines a thrust-direction estimate that, if
// followed, places the vehicle on the target orbit at engine cutoff.
//
// This is an explicit standalone port of navbox's Upfg.cs, converted to double
// precision over Brutal's double3. It takes the vehicle state as plain inertial
// (CCI) inputs — no Simulator, no external dependencies — so it runs inside KSA.
//
// Guidance modes (the original's `mode` field):
//   1 — standard ascent: insert at target radius/velocity/FPA in the target plane,
//       cutoff position free.
//   2 — predictive landing: soft target (vd = current v) so the solution converges
//       on *where the braking burn would end* (Rd) — run synchronously to
//       convergence to measure the burn's downrange before committing.
//   3 — precision landing: drive the cutoff to target.Rdes at target.Velocity,
//       with a throttle command (K, exposed as Throttle) stretching the burn to
//       null the downrange error. Unlike the original (which propagated a CSE
//       reference trajectory with a hard-coded t_ref), the reference here is
//       simply the desired state at the landing point — equivalent for the
//       zero-speed-at-site target and free of the absolute-time dependence.
//
// The vehicle model must carry *current* data: callers rebuild the stage list from
// the live vehicle every step, so Stages[0].MassTotal is the present mass and its
// burn time is inherently time-remaining. (Original navbox instead kept a static
// config and subtracted tb, the time burned on the current stage; with a live
// model that correction would double-count, so this port does not track tb.)
public sealed class UpfgGuidance
{
    private const double G0 = 9.80665;

    private sealed class State
    {
        public CseState Cser;
        public double3 Rbias;
        public double3 Rd;
        public double3 Rgrav;
        public double Tgo;
        public double3 V;
        public double3 Vgo;
        public double K;
    }

    private State _prev = new State();
    private bool _setup;
    private int _mode = 1;

    public int Mode => _mode;

    public bool Converged { get; private set; }
    public double3 Steering { get; private set; }    // unit thrust direction, CCI
    public double Tgo => _prev.Tgo;
    public double VgoMag { get; private set; }        // remaining delta-v magnitude (m/s)
    public double3 Rd => _prev.Rd;                    // predicted cutoff position, CCI
    public double3 Vgo { get; private set; }          // velocity-to-go vector, CCI
    public double3 Rgo { get; private set; }          // position-to-go vector, CCI
    public double Throttle { get; private set; } = 1.0; // commanded throttle (g-limit)

    public void Reset() { _setup = false; Converged = false; Throttle = 1.0; Vgo = default; Rgo = default; _prev = new State(); }

    // r, v: inertial (CCI) state in metres and m/s. vehicle: the staged model,
    // rebuilt by the caller from live data so stage 0 reflects the current mass.
    // mode is latched at setup (after Reset); pass 2/3 for the landing modes.
    public void Step(double3 r, double3 v, double mass, double mu,
                     UpfgTarget target, UpfgVehicle vehicle, int mode = 1)
    {
        if (!_setup)
        {
            _mode = mode;
            Setup(r, v, mu, target);
            _setup = true;
        }
        else
        {
            Run(r, v, mass, mu, target, vehicle);
        }
    }

    private void Setup(double3 r, double3 v, double mu, UpfgTarget target)
    {
        double3 desR;
        double3 tgoV;
        if (_mode == 3)
        {
            // Precision landing: aim straight at the desired landing vector and
            // start from "cancel all current velocity".
            desR = target.Rdes;
            tgoV = -v;
        }
        else if (_mode == 2)
        {
            // Predictive landing: shape rd ahead on the target sphere; vgo seeds
            // as the full braking dv.
            double3 unit2 = RodriguesRotation(r, target.Normal, DegToRad(15));
            desR = unit2 * (target.Radius / unit2.Length());
            tgoV = -v;
        }
        else
        {
            double3 unit = RodriguesRotation(r, target.Normal, DegToRad(15));
            desR = unit * (target.Radius / unit.Length());
            double3 temp = double3.Cross(target.Normal, desR);
            tgoV = target.Velocity * (temp * (1.0 / temp.Length())) - v;
        }

        _prev = new State
        {
            Cser = CseState.Zero,
            Rbias = new double3(0, 0, 0),
            Rd = desR,
            Rgrav = 0.5 * GravVector(mu, r),
            Tgo = 100,
            V = v,
            Vgo = tgoV,
            K = 1,
        };
    }

    private void Run(double3 r, double3 v, double mass, double mu,
                     UpfgTarget target, UpfgVehicle vehicle)
    {
        double prevTgoForConvergence = _prev.Tgo;

        double gamma = target.Fpa;
        double3 iy = -target.Normal;
        double rdval = target.Radius;
        double vdval = target.Velocity;

        CseState cser = _prev.Cser;
        double3 rbias = _prev.Rbias;
        double3 rd = _prev.Rd;
        double3 rgrav = _prev.Rgrav;
        double3 vprev = _prev.V;
        double3 vgo = _prev.Vgo;
        double K = _prev.K;

        // 1 - Stage parameters
        int n = vehicle.Stages.Count;
        var stageModes = new List<int>();
        var accelLimits = new List<double>();
        var massFlows = new List<double>();
        var exhaustVel = new List<double>();
        var thrusts = new List<double>();
        var thrustAccel = new List<double>();
        var charTimes = new List<double>();
        var burnTimes = new List<double>();
        for (int i = 0; i < n; i++)
        {
            UpfgStage s = vehicle.Stages[i];
            double massflow = s.Thrust / (s.Isp * G0);
            stageModes.Add(s.Mode);
            accelLimits.Add(s.GLim * G0);
            thrusts.Add(s.Thrust * K);
            massFlows.Add(massflow);
            exhaustVel.Add(s.Isp * G0);
            thrustAccel.Add(s.Thrust / s.MassTotal);
            charTimes.Add(exhaustVel[i] / thrustAccel[i]);
            // Constant-acceleration stages burn dv = ve·ln(m0/m1) at a fixed accel,
            // so their burn time is exact rather than the full-throttle estimate.
            if (s.Mode == 2)
                burnTimes.Add(exhaustVel[i] * Math.Log(s.MassTotal / s.MassDry) / accelLimits[i]);
            else
                burnTimes.Add((s.MassTotal - s.MassDry) / massflow);
        }

        // 2 - Accelerations (subtract sensed dv). No tb correction: the stage list
        // is rebuilt from live data each step, so burnTimes[0] is already remaining.
        double3 dvsensed = v - vprev;
        vgo -= dvsensed;

        // Update first stage with current mass
        if (stageModes[0] == 1)
        {
            double minMass = vehicle.Stages[0].MassDry + 1e-6;
            if (mass > minMass && thrusts[0] > 0)
            {
                thrustAccel[0] = thrusts[0] / mass;
                if (thrustAccel[0] > 0)
                {
                    charTimes[0] = exhaustVel[0] / thrustAccel[0];
                    if (charTimes[0] <= burnTimes[0])
                        charTimes[0] = burnTimes[0] + 1e-3;
                }
            }
        }
        else if (mass > vehicle.Stages[0].MassDry + 1e-6)
        {
            burnTimes[0] = exhaustVel[0] * Math.Log(mass / vehicle.Stages[0].MassDry) / accelLimits[0];
        }

        // Throttle command: while the current stage is acceleration-limited, scale
        // thrust so full-throttle acceleration never exceeds the limit. Computed
        // directly from the live mass each step (no feedback through K, which the
        // original used and which is unstable).
        Throttle = 1.0;
        if (stageModes[0] == 2 && mass > 0)
        {
            double fullAccel = vehicle.Stages[0].Thrust / mass;
            if (fullAccel > accelLimits[0])
                Throttle = Math.Max(accelLimits[0] / fullAccel, 0.05);
        }

        // 3 - Burn times
        ComputeBurnTimes(stageModes, accelLimits, exhaustVel, charTimes, burnTimes, vgo,
            out List<double> Li, out double L, out List<double> tgoi, out double tgo);

        // More dv aboard than vgo needs: the last stage won't be burned at all, so
        // drop it and re-solve (same as original navbox's L > vgo trim) — otherwise
        // its burn time goes negative and corrupts tgo.
        if (L > vgo.Length() && vehicle.Stages.Count > 1)
        {
            var trimmed = new UpfgVehicle();
            for (int i = 0; i < vehicle.Stages.Count - 1; i++)
                trimmed.Stages.Add(vehicle.Stages[i]);
            Run(r, v, mass, mu, target, trimmed);
            return;
        }

        // 4 - Thrust integrals
        ComputeThrustIntegrals(stageModes, Li, tgoi, charTimes, exhaustVel, burnTimes, accelLimits,
            out double J, out double S, out double Q, out double H, out double P, out L);

        // 5 - Guidance vectors
        ComputeGuidanceVectors(vgo, rd, r, v, tgo, _prev.Tgo, rgrav, iy, S, L, J, Q, H, P, rbias,
            out double3 lambda, out double3 rgo, out double3 iF, out double3 vthrust,
            out double3 rthrust, out double3 vbias);
        rbias = rgo - rthrust;
        Rgo = rgo;   // latched for the panel schematic; not used by the solver

        // 7 - External (conic) estimation of gravity over the burn
        double3 rc1 = r - 0.1 * rthrust - (tgo / 30.0) * vthrust;
        double3 vc1 = v + 1.2 * rthrust / tgo - 0.1 * vthrust;
        (double3 rend, double3 vend, cser) = CseRoutine.Run(rc1, vc1, tgo, mu, cser);
        rgrav = rend - rc1 - vc1 * tgo;

        // 8 - Update target vectors (per guidance mode)
        double3 rp = r + v * tgo + rgrav + rthrust;
        double3 vgrav = vend - vc1;
        double3 vd;
        if (_mode == 2)
        {
            // Predictive: pin the cutoff to the target sphere in the plane, keep
            // current velocity as the soft target — rd converges on where the
            // braking burn actually ends.
            rp -= double3.Dot(rp, iy) * iy;
            rd = rdval * rp * (1.0 / rp.Length());
            vd = v;
            K = 1;
        }
        else if (_mode == 3)
        {
            // Precision: drive the cutoff to the desired landing vector at the
            // desired (zero) speed, and stretch/relax the burn via throttle K to
            // null the downrange miss: dtgo = -2·drz/vgoz, K <- K·tb/(tb+dtgo).
            double3 ix3 = double3.Normalize(target.Rdes);
            double3 iz3 = double3.Cross(ix3, iy);
            // Forward speed along track plus the commanded sink rate (down = -ix).
            vd = vdval * iz3 - target.DescentRate * ix3;
            double drz = double3.Dot(iz3, target.Rdes - rp);
            double vgoz = double3.Dot(iz3, vgo);
            if (Math.Abs(vgoz) > 1e-6 && burnTimes.Count > 0)
            {
                double dtgo = -2.0 * drz / vgoz;
                if (burnTimes[0] + dtgo > 1e-3)
                    K *= burnTimes[0] / (burnTimes[0] + dtgo);
            }
            K = Math.Clamp(K, 0.01, 1.0);
            rd = target.Rdes;
        }
        else
        {
            rp -= double3.Dot(rp, iy) * iy;
            rd = rdval * rp * (1.0 / rp.Length());
            double3 ix = double3.Normalize(rd);
            double3 iz = double3.Cross(ix, iy);
            // Velocity target: desired speed at flight-path angle gamma in the plane.
            double3 vop = new double3(Math.Sin(gamma), 0, Math.Cos(gamma));
            vd = new double3(
                ix.X * vop.X + iy.X * vop.Y + iz.X * vop.Z,
                ix.Y * vop.X + iy.Y * vop.Y + iz.Y * vop.Z,
                ix.Z * vop.X + iy.Z * vop.Y + iz.Z * vop.Z) * vdval;
            K = 1;
        }
        vgo = vd - v - vgrav + vbias;

        // A transient bad input (e.g. the zero-thrust frame mid-staging) can drive the
        // solution non-finite. Committing it would poison the persistent state (vgo,
        // cser, rd) and corrupt every later step — so discard it and re-seed from the
        // live vehicle state on the next call instead.
        if (!IsFinite(tgo) || !IsFinite(rd) || !IsFinite(vgo) || !IsFinite(iF) || !IsFinite(rgrav))
        {
            Reset();
            return;
        }

        // Convergence: tgo settled between iterations
        EvaluateConvergence(prevTgoForConvergence, tgo);

        _prev = new State
        {
            Cser = cser,
            Rbias = rbias,
            Rd = rd,
            Rgrav = rgrav,
            Tgo = tgo,
            V = v,
            Vgo = vgo,
            K = K,
        };

        Steering = double3.Normalize(iF);
        VgoMag = vgo.Length();
        Vgo = vgo;

        // In precision-landing mode the throttle command is K itself (the g-limit
        // block above only applies to constant-acceleration stages).
        if (_mode == 3)
            Throttle = K;
    }

    private static void ComputeBurnTimes(
        List<int> stageModes, List<double> accelLimits, List<double> exhaustVel,
        List<double> charTimes, List<double> burnTimes, double3 vgo,
        out List<double> Li, out double L, out List<double> tgoi, out double tgo)
    {
        int n = stageModes.Count;
        Li = new List<double>();
        L = 0;
        for (int i = 0; i < n - 1; i++)
        {
            if (stageModes[i] == 1)
            {
                double denom = Math.Max(charTimes[i] - burnTimes[i], 1e-6);
                double ratio = charTimes[i] / denom;
                if (ratio <= 0) ratio = 1e-6;
                Li.Add(exhaustVel[i] * Math.Log(ratio));
            }
            else
            {
                Li.Add(accelLimits[i] * burnTimes[i]);
            }
            L += Li[i];
        }

        Li.Add(vgo.Length() - L);
        tgoi = new List<double>();
        for (int i = 0; i < n; i++)
        {
            if (stageModes[i] == 1)
            {
                double exvel = Math.Max(exhaustVel[i], 1e-6);
                burnTimes[i] = charTimes[i] * (1 - Math.Exp(-Li[i] / exvel));
            }
            else
            {
                burnTimes[i] = Li[i] / accelLimits[i];
            }
            tgoi.Add(i == 0 ? burnTimes[i] : tgoi[i - 1] + burnTimes[i]);
        }
        tgo = tgoi[n - 1];
    }

    private static void ComputeThrustIntegrals(
        List<int> stageModes, List<double> Li, List<double> tgoi, List<double> charTimes,
        List<double> exhaustVel, List<double> burnTimes, List<double> accelLimits,
        out double J, out double S, out double Q, out double H, out double P, out double L)
    {
        int n = stageModes.Count;
        L = 0; J = 0; S = 0; Q = 0; H = 0; P = 0;
        for (int i = 0; i < n; i++)
        {
            double tgoi1 = (i == 0) ? 0 : tgoi[i - 1];
            double Ji, Si, Qi, Pi;
            if (stageModes[i] == 1)
            {
                Ji = charTimes[i] * Li[i] - exhaustVel[i] * burnTimes[i];
                Si = -Ji + Li[i] * burnTimes[i];
                Qi = Si * (charTimes[i] + tgoi1) - 0.5 * exhaustVel[i] * burnTimes[i] * burnTimes[i];
                Pi = Qi * (charTimes[i] + tgoi1) - 0.5 * exhaustVel[i] * burnTimes[i] * burnTimes[i] * (burnTimes[i] / 3 + tgoi1);
            }
            else
            {
                Ji = 0.5 * Li[i] * burnTimes[i];
                Si = Ji;
                Qi = Si * (burnTimes[i] / 3 + tgoi1);
                Pi = (1.0 / 6.0) * Si * (tgoi[i] * tgoi[i] + 2 * tgoi[i] * tgoi1 + 3 * tgoi1 * tgoi1);
            }
            Ji += Li[i] * tgoi1;
            Si += L * burnTimes[i];
            Qi += J * burnTimes[i];
            Pi += H * burnTimes[i];
            L += Li[i];
            J += Ji;
            S += Si;
            Q += Qi;
            P += Pi;
            H = J * tgoi[i] - Q;
        }
    }

    private void ComputeGuidanceVectors(
        double3 vgo, double3 rd, double3 r, double3 v, double tgo, double prevTgo, double3 rgrav,
        double3 iy, double S, double L, double J, double Q, double H, double P, double3 rbias,
        out double3 lambda, out double3 rgo, out double3 iF, out double3 vthrust,
        out double3 rthrust, out double3 vbias)
    {
        lambda = double3.Normalize(vgo);
        if (prevTgo > 0)
            rgrav = Math.Pow(tgo / prevTgo, 2) * rgrav;

        rgo = rd - (r + v * tgo + rgrav);
        double3 iz = double3.Normalize(double3.Cross(rd, iy));
        double3 rgoxy = rgo - double3.Dot(iz, rgo) * iz;
        double rgoz = (S - double3.Dot(lambda, rgoxy)) / double3.Dot(lambda, iz);
        rgo = rgoxy + rgoz * iz + rbias;

        double lambdade = Q - S * J / L;
        double3 lambdadot = (rgo - S * lambda) * (1.0 / lambdade);
        iF = lambda - lambdadot * (J / L);
        iF = double3.Normalize(iF);

        double phi = Math.Acos(Math.Clamp(double3.Dot(iF, lambda) / (iF.Length() * lambda.Length()), -1.0, 1.0));
        double phidot = -phi * L / J;
        vthrust = (L - 0.5 * L * phi * phi - J * phi * phidot - 0.5 * H * phidot * phidot) * lambda;
        double rthrustMag = S - 0.5 * S * phi * phi - Q * phi * phidot - 0.5 * P * phidot * phidot;
        rthrust = rthrustMag * lambda - (S * phi + Q * phidot) * (lambdadot * (1.0 / lambdadot.Length()));
        vbias = vgo - vthrust;
    }

    private void EvaluateConvergence(double prevTgo, double curTgo)
    {
        if (prevTgo <= 0) return;
        double tgodiff = (curTgo - prevTgo) / prevTgo;
        if (Math.Abs(tgodiff) < 0.01)
            Converged = true;
    }

    private static bool IsFinite(double d) => !double.IsNaN(d) && !double.IsInfinity(d);

    private static bool IsFinite(double3 v) => IsFinite(v.X) && IsFinite(v.Y) && IsFinite(v.Z);

    private static double3 RodriguesRotation(double3 v, double3 axis, double angle)
    {
        double c = Math.Cos(angle);
        double s = Math.Sin(angle);
        return v * c + double3.Cross(axis, v) * s + axis * (double3.Dot(axis, v) * (1 - c));
    }

    private static double3 GravVector(double mu, double3 r) => mu * (-r) * (1.0 / Math.Pow(r.Length(), 3));

    private static double DegToRad(double d) => d * Math.PI / 180.0;
}

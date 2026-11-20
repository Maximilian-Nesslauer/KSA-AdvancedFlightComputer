using System.Globalization;
using AdvancedFlightComputer.Features.RcsTorque;
using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.SixDof;
using AdvancedFlightComputer.Guidance.Scvx.ThreeDof;

/// <summary>
/// --6dof-rcs: touchdown guidance planning with the craft's RCS as well as its gimbals (issue #222).
///
/// 1. THE ALLOCATOR on a booster's RCS: eight thrusters at the top, pushing either way round the body at four points. The largest pure torque about each axis, and what comes with it: pitch and yaw from the top push the booster sideways by 1/H newtons per newton metre (H the thrusters' height over the centre of mass); roll comes from pairs whose pushes cancel. Then a demand inside the authority (met in full, at the least propellant) and one beyond it (the shortfall reported, nothing else disturbed).
///
/// 2. THE LANDINGS that stalled on 2026-11-18. The air turns the booster back towards its flight path at 0.04 rad/s^2 per degree at 100 m/s (growing with dynamic pressure); the gimbals give about 0.15. At 23:38 the hand-over came at 103 m/s, 7.3 deg off the airflow, where the air's turn (0.31) is twice what the gimbals can hold, and touchdown guidance stalled at 50 nodes; at 23:41, 68 m/s and 4.4 deg, it converged and landed. Each is planned and flown with the gimbals alone and with the gimbals and RCS (about 0.33 rad/s^2): a cold solve at 50 nodes, then the closed loop at 15 nodes, re-solving every 0.1 s as the game does, on a plant whose moment is 10 % stronger than the model's. The plant flies the plan's RCS torque exactly, with its side force: the allocator's pulses are not modelled.
///
/// Fails only if the allocator's measurements are wrong; the landings are reported.
/// </summary>
internal static class SixDofRcsCheck
{
    private const int NX = Dynamics6Dof.NX;
    private const int NU = Dynamics6Dof.NU;
    private const double WarmTrustRegion = 0.05;   // Ksa6DofGuidance.WarmTrustRegion

    // The logged booster (SixDofLiftCheck's): 2171 kN, 10 % floor, 2 deg gimbals at 2 deflections a second.
    internal static double CoastTurnMs;
    private const double Tmax = 2171e3, Floor = 0.10, Isp = 290.0, Gravity = 9.81;
    private const double Arm = 10.0, Ipitch = 1.14e6, Iroll = 2.7e4, Length = 40.0;
    internal const double TargetZ = 13.0, AccelMax = 31.0, GimbalDeg = 2.0, SlewPerS = 2.0;
    private const double Area = ThreeDofScenario.Area;

    // Its RCS: eight 6.7 kN thrusters, Isp 220 s, at the top, 28 m over the centre of mass, on a 1.85 m radius.
    private const double RcsHeight = 28.0, RcsRadius = 1.85, RcsThrust = 6.7e3, RcsIsp = 220.0;

    // The air's turn on the logged booster, rad/s^2 per degree off the airflow at 100 m/s.
    internal const double AirTurnPerDeg = 0.04, PlantMomentFactor = 1.1;

    // The model's +Z (thrust axis) along the game's +x (nose); see SixDofAeroCheck.GameAxes.
    private static readonly double[] GameAxes = [0, 0, 1, 1, 0, 0, 0, 1, 0];

    // --rcs-weight w: Scvx6DofConfig.RcsWeight for the plans, 0 for none.
    internal static double RcsWeight;

    // --abs-terminal: the terminal miss priced on fixed scales; --rate-scale s: the body-rate scale, rad/s (1 before 2026-11-20).
    internal static bool AbsTerminal;

    // The 2026-11-20 12:20 booster's roll, for the coast check: --izz (kg m^2, 64,780 flown), --roll-cl (the swept C_l, -0.002 to -0.005 flown), --rcs-roll (N m of RCS roll each way, ~22,000 flown).
    internal static double IrollUsed = Iroll, RollCl, RcsRollNm;
    internal static bool RollPinned;
    internal static double RateScale = 1.0;

    // Probes of the SCS slowdown (2026-11-20), harness only: --eps e (SCS tolerance), --anderson n (acceleration memory; 0 off), --uscale-rcs f (RCS controls' column scale times f), --wdu w, --prox w.
    internal static double ProbeEps = 3e-4, ProbeUScaleRcs = 1.0, ProbeWdu = 0.05, ProbeProx = 0.05, RateMaxDegS;

    // Diagnostics of the SCS slowdown with RCS (2026-11-19): --diag runs only the fast case's closed loop and prints the first warm re-solves' SCS outcome; with it, --rcs-scale f scales the RCS authority, --rcs-noforce drops its side force, --gimbal flies gimbals alone, --nowarm turns SCS's warm start off.
    private static double RcsScale = 1.0;
    private static bool Diag;

    internal static bool NoGimbal => Environment.GetCommandLineArgs().Contains("--no-gimbal");

    internal static int Run(bool verbose)
    {
        string[] args = Environment.GetCommandLineArgs();
        int wi = Array.IndexOf(args, "--rcs-weight");
        RcsWeight = wi >= 0 && wi + 1 < args.Length ? double.Parse(args[wi + 1], CultureInfo.InvariantCulture) : 0.0;
        int si = Array.IndexOf(args, "--rcs-scale");
        RcsScale = si >= 0 && si + 1 < args.Length ? double.Parse(args[si + 1], CultureInfo.InvariantCulture) : 1.0;
        Diag = args.Contains("--diag");
        double Num(string flag, double dflt) { int i = Array.IndexOf(args, flag); return i >= 0 && i + 1 < args.Length ? double.Parse(args[i + 1], CultureInfo.InvariantCulture) : dflt; }
        ProbeEps = Num("--eps", 3e-4);
        ProbeUScaleRcs = Num("--uscale-rcs", 1.0);
        ProbeWdu = Num("--wdu", 0.05);
        ProbeProx = Num("--prox", 0.05);
        RateMaxDegS = Num("--rate-max", 0.0);
        int ai = Array.IndexOf(args, "--anderson");
        if (ai >= 0 && ai + 1 < args.Length) AdvancedFlightComputer.Guidance.Conic.ScsWorkspace.AccelerationLookback = int.Parse(args[ai + 1], CultureInfo.InvariantCulture);
        Console.WriteLine($"RCS weight {RcsWeight}, authority x{RcsScale}, eps {ProbeEps}, uscale-rcs x{ProbeUScaleRcs}, WDu {ProbeWdu}, prox {ProbeProx}, anderson {AdvancedFlightComputer.Guidance.Conic.ScsWorkspace.AccelerationLookback?.ToString() ?? "default"}");
        if (args.Contains("--rcs-fd"))
            return RcsColumnsAgainstDifferences();
        if (Diag && args.Contains("--gimbal"))
        {
            var st0 = States[0];
            double[] x00 = State(st0.h, st0.r, st0.v, st0.tiltDeg, 16_400.0);
            Console.WriteLine("    " + Loop(Params(MomentPerDeg(1.0), null), Params(MomentPerDeg(PlantMomentFactor), null), x00, st0.sigma, false));
            return 0;
        }
        if (Diag)
        {
            Dynamics6Dof.RcsModel m = RcsModel();
            for (int a = 0; a < 3; a++) { m.MaxPosNm[a] *= RcsScale; m.MaxNegNm[a] *= RcsScale; }
            if (args.Contains("--rcs-noforce")) Array.Clear(m.ForcePerNm);
            int sti = Array.IndexOf(args, "--state");
            var st = States[sti >= 0 && sti + 1 < args.Length ? int.Parse(args[sti + 1], CultureInfo.InvariantCulture) : 0];
            double[] x0 = State(st.h, st.r, st.v, st.tiltDeg, 16_400.0);
            if (args.Contains("--demand"))
            {
                (Dynamics6Dof.Params ddyn, Dynamics6Dof.Params dplant, Mixer mixer) = DemandSetup(m);
                Demand = true;
                mixer.Frozen = args.Contains("--freeze");
                mixer.Reset(ddyn);
                Console.WriteLine("    " + Loop(ddyn, dplant, x0, st.sigma, false, mixer.Mix, u => mixer.UpdateEffective(ddyn, u))
                                  + $"; RCS gave up to {mixer.PeakRcs / 1e3:F0} kN m, gimbals {mixer.PeakGimbal / 1e3:F0}");
                return 0;
            }
            Console.WriteLine("    " + Loop(Params(MomentPerDeg(1.0), m), Params(MomentPerDeg(PlantMomentFactor), m), x0, st.sigma, false));
            return 0;
        }
        Console.WriteLine("TOUCHDOWN GUIDANCE WITH RCS TORQUE");
        if (NoGimbal) Console.WriteLine("  --no-gimbal: no gimbal on the craft. \"gimbals alone\" has no torque at all, \"gimbals and RCS\" is the RCS alone.");
        bool ok = Allocator();
        Landings(verbose);
        Console.WriteLine();
        Console.WriteLine(ok ? "PASS - the allocator measures the layout as built (landings reported above)" : "FAIL - see above");
        return ok ? 0 : 1;
    }

    // ------------------------------------------------------------------- 1. the allocator

    /// <summary>The booster's RCS in model axes: at four points round the top, a thruster pushing each way round the body.</summary>
    private static RcsTorqueAllocator Layout()
    {
        var a = new RcsTorqueAllocator(8);
        double flow = RcsThrust / (RcsIsp * 9.80665);
        int i = 0;
        for (int p = 0; p < 4; p++)
        {
            double phi = p * Math.PI / 2.0;
            double rx = RcsRadius * Math.Cos(phi), ry = RcsRadius * Math.Sin(phi), rz = RcsHeight;
            foreach (int s in new[] { 1, -1 })
            {
                double fx = -s * RcsThrust * Math.Sin(phi), fy = s * RcsThrust * Math.Cos(phi), fz = 0.0;
                a.Force[3 * i] = fx; a.Force[3 * i + 1] = fy; a.Force[3 * i + 2] = fz;
                a.Torque[3 * i] = ry * fz - rz * fy;
                a.Torque[3 * i + 1] = rz * fx - rx * fz;
                a.Torque[3 * i + 2] = rx * fy - ry * fx;
                a.Flow[i] = flow;
                i++;
            }
        }
        return a;
    }

    private static bool Allocator()
    {
        Console.WriteLine();
        Console.WriteLine("1. the allocator: eight thrusters round the top of a booster, 28 m over its centre of mass");
        RcsTorqueAllocator a = Layout();
        double flow = RcsThrust / (RcsIsp * 9.80665);
        bool ok = true;

        string[] names = ["+pitch(x)", "-pitch(x)", "+yaw(y)", "-yaw(y)", "+roll(z)", "-roll(z)"];
        for (int j = 0; j < 6; j++)
        {
            int axis = j / 2, sign = j % 2 == 0 ? 1 : -1;
            RcsTorqueAllocator.AxisAuthority auth = a.Characterise(axis, sign);
            double force = Math.Sqrt(auth.ForcePerNmX * auth.ForcePerNmX + auth.ForcePerNmY * auth.ForcePerNmY + auth.ForcePerNmZ * auth.ForcePerNmZ);
            double wantMax = axis < 2 ? 2.0 * RcsHeight * RcsThrust : 4.0 * RcsRadius * RcsThrust;
            double wantForce = axis < 2 ? 1.0 / RcsHeight : 0.0;
            double wantFlow = axis < 2 ? flow / (RcsHeight * RcsThrust) : flow / (RcsRadius * RcsThrust);
            bool good = Math.Abs(auth.MaxTorqueNm - wantMax) < 1e-6 * wantMax
                        && Math.Abs(force - wantForce) < 1e-6 / RcsHeight
                        && Math.Abs(auth.FlowPerNm - wantFlow) < 1e-6 * wantFlow;
            ok &= good;
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"  {names[j],-10} max {auth.MaxTorqueNm / 1e3,7:F1} kN m ({auth.MaxTorqueNm / (axis < 2 ? Ipitch : Iroll),5:F2} rad/s^2)  side force {force * 1e3,6:F2} N per kN m  propellant {auth.FlowPerNm * 1e6,6:F2} g/s per kN m  {(good ? "ok" : "WRONG")}"));
        }

        Span<double> duty = stackalloc double[a.Count];
        double[] priority = [1.0, 1.0, 0.3];
        double pitchMax = 2.0 * RcsHeight * RcsThrust;
        RcsTorqueAllocator.Result inside = a.Allocate(0.4 * pitchMax, -0.3 * pitchMax, 0.2 * 4.0 * RcsRadius * RcsThrust, priority, duty);
        bool insideOk = inside.Solved && inside.ShortfallNm < 1e-6 * pitchMax;
        ok &= insideOk;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  inside its reach (40 % pitch, -30 % yaw, 20 % roll): short {inside.ShortfallNm:F3} N m, {inside.Firing} firing, {inside.FlowKgS:F2} kg/s, side force {Math.Sqrt(inside.ForceX * inside.ForceX + inside.ForceY * inside.ForceY):F0} N  {(insideOk ? "ok" : "WRONG")}"));
        RcsTorqueAllocator.Result beyond = a.Allocate(2.0 * pitchMax, 0.0, 0.0, priority, duty);
        bool beyondOk = beyond.Solved && Math.Abs(beyond.TorqueX - pitchMax) < 1e-6 * pitchMax
                        && Math.Abs(beyond.TorqueY) < 1e-6 * pitchMax && Math.Abs(beyond.TorqueZ) < 1e-6 * pitchMax;
        ok &= beyondOk;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  twice its pitch: gives {beyond.TorqueX / 1e3:F1} kN m, short {beyond.ShortfallNm / 1e3:F1} kN m, yaw {beyond.TorqueY:F3} and roll {beyond.TorqueZ:F3} N m left alone  {(beyondOk ? "ok" : "WRONG")}"));
        return ok;
    }

    internal static Dynamics6Dof.RcsModel RcsModel()
    {
        // As Ksa6DofSetup.RcsModelFrom: each axis's two bounds, and the side force per newton metre odd in the torque.
        RcsTorqueAllocator a = Layout();
        var m = new Dynamics6Dof.RcsModel();
        for (int axis = 0; axis < Dynamics6Dof.NRCS; axis++)
        {
            RcsTorqueAllocator.AxisAuthority plus = a.Characterise(axis, 1), minus = a.Characterise(axis, -1);
            m.MaxPosNm[axis] = plus.MaxTorqueNm;
            m.MaxNegNm[axis] = minus.MaxTorqueNm;
            m.ForcePerNm[3 * axis] = 0.5 * (plus.ForcePerNmX - minus.ForcePerNmX);
            m.ForcePerNm[3 * axis + 1] = 0.5 * (plus.ForcePerNmY - minus.ForcePerNmY);
            m.ForcePerNm[3 * axis + 2] = 0.5 * (plus.ForcePerNmZ - minus.ForcePerNmZ);
            m.FlowPerNm[axis] = 0.5 * (plus.FlowPerNm + minus.FlowPerNm);
        }
        if (RcsRollNm > 0.0)
            m.MaxPosNm[2] = m.MaxNegNm[2] = RcsRollNm;
        return m;
    }

    // ------------------------------------------------------------------- 2. the landings

    private static readonly (string name, double h, double[] r, double[] v, double tiltDeg, double sigma)[] States =
    [
        ("23:38-like: 257 m, 103 m/s, 7.3 deg off the air", 257.0, [25.0, -10.0], [-12.0, -8.0, -102.0], 7.3, 6.0),
        ("23:41-like: 121 m, 68 m/s, 4.4 deg off the air", 121.0, [12.0, -5.0], [-8.0, -5.0, -67.3], 4.4, 4.5),
    ];

    private static void Landings(bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine("2. the logged fast and slow hand-overs, against the air's turn of 0.04 rad/s^2 per degree at 100 m/s");
        Dynamics6Dof.RcsModel rcs = RcsModel();
        foreach ((string name, double h, double[] r, double[] v, double tilt, double sigma) in States)
        {
            double[] x0 = State(h, r, v, tilt, 16_400.0);
            Console.WriteLine();
            Console.WriteLine($"  {name}");
            foreach ((string label, Dynamics6Dof.RcsModel? model) in new[] { ("gimbals alone", (Dynamics6Dof.RcsModel?)null), ("gimbals and RCS", rcs) })
            {
                Dynamics6Dof.Params dyn = Params(MomentPerDeg(1.0), model);
                Dynamics6Dof.Params plant = Params(MomentPerDeg(PlantMomentFactor), model);
                string cold = Cold(dyn, x0, sigma, verbose);
                string loop = Loop(dyn, plant, x0, sigma, verbose);
                Console.WriteLine($"    {label,-16} cold: {cold}");
                Console.WriteLine($"    {"",-16} loop: {loop}");
            }
            {
                (Dynamics6Dof.Params dyn, Dynamics6Dof.Params plant, Mixer mixer) = DemandSetup(rcs);
                Demand = true;
                mixer.Reset(dyn);
                string cold = Cold(dyn, x0, sigma, verbose);
                mixer.Reset(dyn);
                string loop = Loop(dyn, plant, x0, sigma, verbose, mixer.Mix, u => mixer.UpdateEffective(dyn, u));
                Demand = false;
                Console.WriteLine($"    {"torque demand",-16} cold: {cold}");
                Console.WriteLine($"    {"",-16} loop: {loop}; RCS gave up to {mixer.PeakRcs / 1e3:F0} kN m, gimbals {mixer.PeakGimbal / 1e3:F0}");
            }
        }
    }

    internal static Dynamics6Dof.Params Params(BodyAeroTable moment, Dynamics6Dof.RcsModel? rcs, double areaScale = 1.0) => new()
    {
        Gx = 0, Gy = 0, Gz = -Gravity, G0 = 9.80665, Isp = Isp,
        LArm = Arm, Ixx = Ipitch, Iyy = Ipitch, Izz = IrollUsed,
        Aero = ThreeDofScenario.Model(),
        BodyAero = new BodyAero6Dof
        {
            Atmosphere = ThreeDofScenario.Model().Atmosphere!,
            Force = ThreeDofScenario.BodyTable(),
            Moment = moment,
            ReferenceArea = Area * areaScale,
            ReferenceLength = Length,
            ModelToTable = GameAxes,
        },
        Rcs = rcs,
    };

    private static BodyAeroTable? _unitMoment;
    private static double _unitScale;

    /// <summary>
    /// A moment table that turns the booster back towards its flight path at <see cref="AirTurnPerDeg"/> rad/s^2 per degree at 100 m/s, times <paramref name="factor"/>: C_m and C_n linear in alpha and beta. Its sign and size are found by flying it: whichever signs turn a tilted booster back, scaled to the logged turn at a 5 degree tilt.
    /// </summary>
    internal static BodyAeroTable MomentPerDeg(double factor)
    {
        if (_unitMoment == null)
        {
            double best = 0.0;
            foreach (int sy in new[] { 1, -1 })
                foreach (int sz in new[] { 1, -1 })
                {
                    BodyAeroTable t = Linear(1e-3 * sy, 1e-3 * sz);
                    double a = Turn(t, about: 0), b = Turn(t, about: 1);
                    // Restoring in both planes, and the larger of the two readings sets the scale.
                    if (a > 0.0 && b > 0.0 && (_unitMoment == null || Math.Min(a, b) > best))
                    {
                        best = Math.Min(a, b);
                        _unitMoment = t;
                        _unitScale = 1e-3 * (AirTurnPerDeg * 5.0) / best;
                        _signs = (sy, sz);
                    }
                }
            if (_unitMoment == null)
                throw new InvalidOperationException("no sign of the moment table turns the booster back");
        }
        return Linear(_signs.sy * _unitScale * factor, _signs.sz * _unitScale * factor);
    }

    private static (int sy, int sz) _signs;

    private static BodyAeroTable Linear(double perDegM, double perDegN)
    {
        double[] mach = AeroTable.DefaultMachBreakpoints;
        double[] angles = { -20, -16, -12, -8, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 8, 12, 16, 20 };
        var c = new double[mach.Length * angles.Length * angles.Length * 3];
        int i = 0;
        for (int m = 0; m < mach.Length; m++)
            for (int ja = 0; ja < angles.Length; ja++)
                for (int jb = 0; jb < angles.Length; jb++)
                {
                    c[i++] = RollCl;
                    c[i++] = perDegM * angles[ja];
                    c[i++] = perDegN * angles[jb];
                }
        return new BodyAeroTable(mach, angles, angles, c);
    }

    /// <summary>How fast a moment table turns a booster tilted 5 degrees off the air at 100 m/s back towards it, rad/s^2: positive when it turns it back. The tilt is about the site's x axis (about = 0) or y axis (1).</summary>
    private static double Turn(BodyAeroTable moment, int about)
    {
        Dynamics6Dof.Params p = Params(moment, null);
        double[] x = State(200.0, [0.0, 0.0], [0.0, 0.0, -100.0], 5.0, 16_400.0, about);
        var f = new double[NX];
        Dynamics6Dof.Eval(x, new double[NU], p, f);
        // Turn the attitude a little by the angular acceleration and see whether the tilt off the air shrinks.
        double[] y = (double[])x.Clone();
        const double dt = 0.01;
        double wx = f[10] * dt, wy = f[11] * dt, wz = f[12] * dt;
        double qw = y[6], qx = y[7], qy = y[8], qz = y[9];
        y[6] = qw - 0.5 * dt * (qx * wx + qy * wy + qz * wz);
        y[7] = qx + 0.5 * dt * (qw * wx + qy * wz - qz * wy);
        y[8] = qy + 0.5 * dt * (qw * wy - qx * wz + qz * wx);
        y[9] = qz + 0.5 * dt * (qw * wz + qx * wy - qy * wx);
        double before = OffAir(x), after = OffAir(y);
        double turn = Math.Sqrt(f[10] * f[10] + f[11] * f[11]);
        return after < before ? turn : -turn;
    }

    /// <summary>The thrust axis's angle off retrograde, radians.</summary>
    internal static double OffAir(double[] x)
    {
        double n = Math.Sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
        double qw = x[6] / n, qx = x[7] / n, qy = x[8] / n, qz = x[9] / n;
        double bx = 2.0 * (qx * qz + qw * qy), by = 2.0 * (qy * qz - qw * qx), bz = 1.0 - 2.0 * (qx * qx + qy * qy);
        double sp = Math.Sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
        double c = -(bx * x[3] + by * x[4] + bz * x[5]) / sp;
        return Math.Acos(Math.Clamp(c, -1.0, 1.0));
    }

    /// <summary>A state at height h, position r, velocity v, the thrust axis tilted off retrograde about the site's x (about = 0) or y (1) axis.</summary>
    internal static double[] State(double h, double[] r, double[] v, double tiltDeg, double mass, int about = 0)
    {
        double sp = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        double[] retro = [-v[0] / sp, -v[1] / sp, -v[2] / sp];
        double a = tiltDeg * Math.PI / 180;
        double[] axis = about == 0
            ? [retro[0], retro[1] * Math.Cos(a) - retro[2] * Math.Sin(a), retro[1] * Math.Sin(a) + retro[2] * Math.Cos(a)]
            : [retro[0] * Math.Cos(a) + retro[2] * Math.Sin(a), retro[1], -retro[0] * Math.Sin(a) + retro[2] * Math.Cos(a)];
        double[] q = Pointing(axis);
        return [r[0], r[1], h, v[0], v[1], v[2], q[0], q[1], q[2], q[3], 0, 0, 0, mass];
    }

    private static double[] Pointing(double[] t)
    {
        double[] z = [0, 0, 1];
        double[] u = ThreeDofScenario.Normalise(t);
        double c = Math.Clamp(ThreeDofScenario.Dot(z, u), -1, 1);
        double[] axis = ThreeDofScenario.Cross(z, u);
        double s = Math.Sqrt(ThreeDofScenario.Dot(axis, axis));
        if (s < 1e-12) return c > 0 ? [1, 0, 0, 0] : [0, 1, 0, 0];
        double half = 0.5 * Math.Acos(c);
        return [Math.Cos(half), axis[0] / s * Math.Sin(half), axis[1] / s * Math.Sin(half), axis[2] / s * Math.Sin(half)];
    }

    internal static double[] Target(double mass)
    {
        var xf = new double[NX];
        xf[2] = TargetZ;
        xf[6] = 1.0;
        xf[13] = mass;
        return xf;
    }

    internal static Scvx6DofConfig Config(int n, double sigma, double[] x0, double[] xf, Dynamics6Dof.Params dyn, int coastIntervals = 0)
    {
        double length = Math.Sqrt(x0[0] * x0[0] + x0[1] * x0[1] + (x0[2] - xf[2]) * (x0[2] - xf[2]));
        double speed = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
        double velocity = Math.Max(Math.Max(speed, Math.Sqrt(length * Gravity)), 1.0);
        double omegaScale = 60.0 * Math.PI / 180.0 / (sigma * 0.5);
        return new Scvx6DofConfig
        {
            Nodes = n,
            CoastIntervals = coastIntervals,
            Tmax = Tmax,
            ThrottleFloor = Floor,
            // --no-gimbal: a craft that points its thrust with the RCS alone, as Ksa6DofSetup plans one (a zero cone and no arm).
            GimbalMaxDeg = NoGimbal ? 0.0 : GimbalDeg,
            GimbalArm = Environment.GetCommandLineArgs().Contains("--old-scale") || NoGimbal ? 0.0 : Arm,
            GimbalSlewPerS = Environment.GetCommandLineArgs().Contains("--noslew") ? 0.0 : SlewPerS,
            // --roll-pinned (coast check): as the 12:20-12:53 boosters flew - their engine gives 88.8 kN m of roll at full thrust, so the plan holds the heading to the end (Ksa6DofSetup.RollModel).
            TauRollMax = RollPinned ? 88_833.0 : 1.0e4,
            RollTorqueFollowsThrust = RollPinned,
            TerminalRollFree = !RollPinned || coastIntervals > 0,
            // As flight: a coast plan neither pins the heading nor limits the roll rate (Ksa6DofSetup).
            RateLimitIncludesRoll = coastIntervals == 0,
            TiltMaxDeg = 60.0,
            GroundFloor = -1.0,
            AccelMax = AccelMax,
            WW = 0.002 / (n * 3.0 * omegaScale * omegaScale),
            // --coast-turn m/s: the coast's rate price as Ksa6DofSetup sizes it, turning at 0.05 rad/s through the coast priced as that much burn.
            CoastRateWeight = coastIntervals > 1 && CoastTurnMs > 0.0 ? CoastTurnMs / (Isp * 9.80665) / ((coastIntervals - 1) * 0.05 * 0.05) : 0.0,
            WDu = ProbeWdu,
            ProximalWeight = ProbeProx,
            UScale = ProbeUScale(dyn),
            GlideSlopeDeg = 0.0,
            VzMax = 0.0,
            TerminalMissWeight = 1.0e5,
            TerminalSpeedWeight = 1.0e4,
            // --abs-terminal: priced per fixed metre and m/s, as in flight (Ksa6DofSetup.TerminalMissPriceM).
            TerminalMissScaleM = AbsTerminal ? 250.0 : 0.0,
            TerminalSpeedScaleMs = AbsTerminal ? 100.0 : 0.0,
            SigmaMin = sigma * 0.15,
            SigmaMax = sigma * 4.0,
            SigmaScale = sigma,
            RcsMaxPosNm = dyn.Rcs?.MaxPosNm,
            RcsMaxNegNm = dyn.Rcs?.MaxNegNm,
            RcsWeight = RcsWeight,
            RateMaxRadS = RateMaxDegS * Math.PI / 180.0,
            TorqueDemand = Demand,
            DemandTorquePerN = Demand ? GimbalTorquePerN : 0.0,
            DemandRollPerN = Demand ? RollTorquePerN : 0.0,
            VirtualControlScale = [0, 0, 0, 5, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0],
            XScale = [length, length, length, velocity, velocity, velocity, 1, 1, 1, 1, RateScale, RateScale, RateScale, x0[13]],
        };
    }

    /// <summary>The control scales as Scvx6DofConfig.ResolvedUScale would make them, with the RCS controls' times --uscale-rcs; null (the default) when that is 1.</summary>
    private static double[]? ProbeUScale(Dynamics6Dof.Params dyn)
    {
        if (ProbeUScaleRcs == 1.0 || dyn.Rcs == null || Demand)
            return null;
        double tan = Math.Tan(GimbalDeg * Math.PI / 180.0);
        var us = new double[NU];
        us[0] = us[1] = Tmax * tan;
        us[2] = Tmax;
        us[3] = 1.0e4;
        for (int a = 0; a < Dynamics6Dof.NRCS; a++)
            us[Dynamics6Dof.IRCS + a] = ProbeUScaleRcs * Math.Max(dyn.Rcs.MaxPosNm[a], dyn.Rcs.MaxNegNm[a]);
        return us;
    }

    internal static Scvx6DofSolver Solver(Scvx6DofConfig cfg, Dynamics6Dof.Params dyn)
    {
        var s = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = ProbeEps, WarmStart = !Environment.GetCommandLineArgs().Contains("--nowarm") };
        s.MaxSubproblemIterations = 9000;
        s.EscalatedSubproblemIterations = 9000;
        return s;
    }

    internal static double[] Tolerance(double mass)
    {
        var tol = new double[NX];
        for (int i = 0; i < 3; i++) { tol[i] = 1.0; tol[3 + i] = 0.5; tol[10 + i] = 0.01; }
        for (int i = 0; i < 4; i++) tol[6 + i] = Math.Sin(0.5 * Math.PI / 180.0);
        tol[13] = mass * 1e-3;
        return tol;
    }

    private static string Cold(Dynamics6Dof.Params dyn, double[] x0, double sigma, bool verbose)
    {
        const int n = 50;
        double[] xf = Target(x0[13]);
        Scvx6DofConfig cfg = Config(n, sigma, x0, xf, dyn);
        Scvx6DofSolver s = Solver(cfg, dyn);
        ColdSeed6Dof.Build(x0, xf, sigma, n, cfg, dyn, out double[] xs, out double[] us);
        s.Initialize(x0, xf, xs, us, sigma);
        double[] tol = Tolerance(x0[13]);
        int toGate = -1, toFine = -1;
        double best = double.PositiveInfinity;
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var admm = new List<int>();
        for (int i = 1; i <= 40; i++)
        {
            ScvxIteration it = s.Iterate();
            admm.Add(it.SolverIterations);
            double ratio = it.Accepted ? s.WeightedDefect(tol, n - 2, 1.0, out _, out _, out _) : double.PositiveInfinity;
            best = Math.Min(best, ratio);
            if (verbose)
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"        {i,2}: accepted {it.Accepted,-5} trust {it.TrustRegion,6:G3} sigma {it.Sigma,5:F2} gate {ratio,7:F3}x"));
            if (toGate < 0 && ratio <= 1.0) toGate = i;
            if (ratio <= 0.05) { toFine = i; break; }
        }
        double peak = PeakRcs(s.ReferenceU, n, out double share);
        return string.Create(CultureInfo.InvariantCulture,
            $"gate in {(toGate < 0 ? "-" : toGate.ToString())}, 5 % of it in {(toFine < 0 ? "- (40 tried)" : toFine.ToString())}, best {best:F3}x, burn {s.Sigma:F1} s, {clock.Elapsed.TotalMilliseconds / 1e3:F1} s, SCS {Median(admm)}/iteration; peak RCS {peak / 1e3:F0} kN m ({share:P0} of reach)");
    }

    /// <summary>The largest RCS torque a plan asks for on one axis, N m, and its share of that axis's reach.</summary>
    internal static double PeakRcs(double[] u, int n, out double share)
    {
        double peak = 0.0;
        share = 0.0;
        double reach = 2.0 * RcsHeight * RcsThrust;
        for (int k = 0; k < n; k++)
            for (int a = 0; a < Dynamics6Dof.NRCS; a++)
                peak = Math.Max(peak, Math.Abs(u[k * NU + Dynamics6Dof.IRCS + a]));
        share = peak / reach;
        return peak;
    }

    private static string Loop(Dynamics6Dof.Params dyn, Dynamics6Dof.Params plant, double[] x0, double sigma, bool verbose,
                               Func<double[], double[]>? mix = null, Action<double[]>? onPlan = null)
    {
        const int n = 15;
        const double cycle = 0.1, dt = 0.01;
        double[] xf = Target(x0[13]);
        Scvx6DofConfig cfg = Config(n, sigma, x0, xf, dyn);
        Scvx6DofSolver s = Solver(cfg, dyn);
        ColdSeed6Dof.Build(x0, xf, sigma, n, cfg, dyn, out double[] xs, out double[] us);
        s.Initialize(x0, xf, xs, us, sigma);
        double[] tol = Tolerance(x0[13]);
        bool planned = false;
        for (int i = 0; i < 40; i++)
        {
            ScvxIteration it = s.Iterate();
            if (it.Accepted && s.WeightedDefect(tol, n - 2, 1.0, out _, out _, out _) <= 1.0)
                planned = true;
            if (it.Accepted && s.WeightedDefect(tol, n - 2, 1.0, out _, out _, out _) <= 0.05)
                break;
        }

        double[] fx = (double[])s.ReferenceX.Clone(), fu = (double[])s.ReferenceU.Clone();
        onPlan?.Invoke(Sample(fu, NU, s.Sigma, n, 0.0));
        double fs = s.Sigma, f0 = 0.0;
        double[] x = (double[])x0.Clone();
        int refused = 0, kept = 0;
        var gates = new List<double>();
        var loopAdmm = new List<int>();
        double t = 0.0, maxOff = 0.0, mass0 = x0[13], peakRate = 0.0;
        while (t < 20.0)
        {
            double left = fs - (t - f0);
            if (left > 0.4 && t > 0.0)
            {
                Shift(fx, fu, fs, t - f0, n, out double[] sx, out double[] su, out double ss);
                // As the game re-seeds a warm solve (Ksa6DofGuidance): the whole reference moved with the measured state, not node 0 alone, and the trust region opened to its warm start rather than left where the last solve shrank it.
                ColdReferenceShift.Apply(sx, n, ss, x);
                s.Reseed(x, sx, su, ss, trustRegion: Math.Clamp(s.TrustRegion, WarmTrustRegion, s.TrustRegionMax));
                double ratio = double.PositiveInfinity;
                for (int i = 0; i < 3; i++)
                {
                    ScvxIteration it = s.Iterate();
                    loopAdmm.Add(it.SolverIterations);
                    if (Diag && gates.Count < 6)
                        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"      t {t:F1} it {i}: solved {it.Solved} accepted {it.Accepted} scs {it.SolverIterations} trust {it.TrustRegion:G3} {(it.Solved ? "" : s.LastFailureReason)}"));
                    ratio = it.Accepted ? s.WeightedDefect(tol, n - 2, 1.0, out _, out _, out _) : double.PositiveInfinity;
                    if (ratio <= 0.05) break;
                }
                gates.Add(ratio);
                if (ratio <= 1.0)
                {
                    fx = (double[])s.ReferenceX.Clone();
                    fu = (double[])s.ReferenceU.Clone();
                    onPlan?.Invoke(Sample(fu, NU, s.Sigma, n, 0.0));
                    fs = s.Sigma;
                    f0 = t;
                    kept++;
                }
                else
                    refused++;
                if (verbose)
                    Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                        $"        t {t,5:F1} alt {x[2],6:F1} off ({x[0],6:F1},{x[1],6:F1}) v ({x[3],6:F1},{x[4],6:F1},{x[5],6:F1}) gate {ratio,7:F3}x {(ratio <= 1.0 ? "kept" : "REFUSED")}"));
            }
            bool landed = false;
            for (double h = 0; h < cycle - 1e-9; h += dt)
            {
                double[] u = Sample(fu, NU, fs, n, t - f0);
                if (mix != null) u = mix(u);
                Step(x, u, plant, dt);
                maxOff = Math.Max(maxOff, OffAir(x) * 180.0 / Math.PI);
                peakRate = Math.Max(peakRate, Math.Sqrt(x[10] * x[10] + x[11] * x[11] + x[12] * x[12]) * 180.0 / Math.PI);
                t += dt;
                if (x[2] <= TargetZ || (x[5] > 0.0 && x[2] < TargetZ + 20.0)) { landed = true; break; }
            }
            if (landed) break;
        }
        gates.Sort();
        double med = gates.Count > 0 ? gates[gates.Count / 2] : double.NaN;
        double miss = Math.Sqrt(x[0] * x[0] + x[1] * x[1]);
        double side = Math.Sqrt(x[3] * x[3] + x[4] * x[4]);
        return string.Create(CultureInfo.InvariantCulture,
            $"first plan {(planned ? "inside" : "OUTSIDE")} the gate; {kept} kept, {refused} refused, median gate {med:F2}x, SCS {Median(loopAdmm)}/iteration; ends {x[2]:F1} m up, {miss:F1} m off, {-x[5]:F1} m/s down, {side:F1} m/s sideways; {mass0 - x[13]:F0} kg burned; peak rate {peakRate:F1} deg/s");
    }

    internal static int Median(List<int> v)
    {
        if (v.Count == 0) return 0;
        var c = new List<int>(v);
        c.Sort();
        return c[c.Count / 2];
    }

    // ------------------------------------------------------------------- the RCS columns of B

    /// <summary>--rcs-fd: Dynamics6Dof.Jacobian writes the RCS columns of B by hand (RcsColumns); they must equal central differences of Eval in the RCS controls, at tilted, spinning states with RCS in use.</summary>
    private static int RcsColumnsAgainstDifferences()
    {
        Dynamics6Dof.Params p = Params(MomentPerDeg(1.0), RcsModel());
        double worst = 0.0;
        foreach (double tilt in new[] { 3.0, 12.0, 25.0 })
        {
            double[] x = State(200.0, [10.0, -5.0], [-8.0, 4.0, -80.0], tilt, 16_400.0);
            x[10] = 0.05; x[11] = -0.08; x[12] = 0.2;
            double[] u = [12e3, -9e3, 600e3, 2e3, 150e3, -90e3, 20e3];
            var A = new double[NX * NX];
            var B = new double[NX * NU];
            Dynamics6Dof.Jacobian(x, u, p, new double[NX], A, B);
            for (int j = Dynamics6Dof.IRCS; j < NU; j++)
            {
                double h = 10.0;
                double[] up = (double[])u.Clone(), um = (double[])u.Clone();
                up[j] += h; um[j] -= h;
                var fp = new double[NX];
                var fm = new double[NX];
                Dynamics6Dof.Eval(x, up, p, fp);
                Dynamics6Dof.Eval(x, um, p, fm);
                for (int r = 0; r < NX; r++)
                {
                    double fd = (fp[r] - fm[r]) / (2.0 * h);
                    double scale = Math.Max(Math.Abs(fd), 1e-9);
                    double err = Math.Abs(B[r * NU + j] - fd) / Math.Max(scale, 1e-6);
                    if (Math.Abs(fd) > 1e-12 || Math.Abs(B[r * NU + j]) > 1e-12)
                        worst = Math.Max(worst, err);
                    if (err > 1e-6 && (Math.Abs(fd) > 1e-12 || Math.Abs(B[r * NU + j]) > 1e-12))
                        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  tilt {tilt}: B[{r},{j}] = {B[r * NU + j]:E4}, differences {fd:E4}"));
                }
            }
        }
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"RCS columns of B against central differences: worst relative error {worst:E2}"));
        return worst < 1e-6 ? 0 : 1;
    }

    // ------------------------------------------------------------------- torque demand

    // The plan's torque-demand envelope, as the mixer reports it: the gimbals' pitch/yaw and roll torque per newton of axial thrust.
    private static double GimbalTorquePerN => Arm * Math.Tan(GimbalDeg * Math.PI / 180.0);
    private const double RollTorquePerN = 1.0e4 / Tmax;   // Config's TauRollMax at Tmax
    private static bool Demand;

    /// <summary>The plan for a torque demand: three torque controls carrying the whole control torque, bounded by the gimbals' share at the thrust plus the RCS's; the side force they bring, an effective factor the mixer refreshes. The plant flies the real split.</summary>
    private static (Dynamics6Dof.Params dyn, Dynamics6Dof.Params plant, Mixer mixer) DemandSetup(Dynamics6Dof.RcsModel rcs)
    {
        var envelope = new Dynamics6Dof.RcsModel
        {
            MaxPosNm = (double[])rcs.MaxPosNm.Clone(),
            MaxNegNm = (double[])rcs.MaxNegNm.Clone(),
            ForcePerNm = new double[9],
            FlowPerNm = (double[])rcs.FlowPerNm.Clone(),
        };
        return (Params(MomentPerDeg(1.0), envelope), Params(MomentPerDeg(PlantMomentFactor), rcs), new Mixer(rcs));
    }

    /// <summary>
    /// The mixer, gimbals first: each step it splits the plan's torque, the gimbals taking what they can at the thrust and the RCS the rest, and turns the gimbals' share into lateral thrust and roll torque for the plant. Between re-solves it gives the plan the side force per newton metre its split brings (UpdateEffective): the gimbals' share pushes the bottom of the booster one way (1/arm), the RCS's the top the other (its own factor).
    /// </summary>
    private sealed class Mixer(Dynamics6Dof.RcsModel rcs)
    {
        public double PeakRcs, PeakGimbal;

        /// <summary>Hold the side-force factor at the first plan's split instead of refreshing it with every plan kept, so consecutive plans share one model.</summary>
        public bool Frozen;
        private bool _set;

        private (double[] gimbal, double[] rest) Split(double[] u)
        {
            double t = Math.Max(u[Dynamics6Dof.IT], 0.0);
            double tx = u[Dynamics6Dof.IRCS], ty = u[Dynamics6Dof.IRCS + 1], tz = u[Dynamics6Dof.IRCS + 2];
            double cap = GimbalTorquePerN * t, mag = Math.Sqrt(tx * tx + ty * ty);
            double k = mag > cap && mag > 0.0 ? cap / mag : 1.0;
            double rollCap = RollTorquePerN * t;
            var g = new[] { tx * k, ty * k, Math.Clamp(tz, -rollCap, rollCap) };
            var r = new double[3];
            double[] want = { tx - g[0], ty - g[1], tz - g[2] };
            for (int a = 0; a < 3; a++)
                r[a] = Math.Clamp(want[a], -rcs.MaxNegNm[a], rcs.MaxPosNm[a]);
            return (g, r);
        }

        public double[] Mix(double[] u)
        {
            (double[] g, double[] r) = Split(u);
            var p = new double[NU];
            p[Dynamics6Dof.IT] = u[Dynamics6Dof.IT];
            // tau_x = L tdy, tau_y = -L tdx (Dynamics6Dof).
            p[Dynamics6Dof.ITDX] = -g[1] / Arm;
            p[Dynamics6Dof.ITDY] = g[0] / Arm;
            p[Dynamics6Dof.ITAU] = g[2];
            for (int a = 0; a < 3; a++)
                p[Dynamics6Dof.IRCS + a] = r[a];
            PeakRcs = Math.Max(PeakRcs, Math.Sqrt(r[0] * r[0] + r[1] * r[1]));
            PeakGimbal = Math.Max(PeakGimbal, Math.Sqrt(g[0] * g[0] + g[1] * g[1]));
            return p;
        }

        /// <summary>The plan's side force per newton metre on each axis from the split of torque <paramref name="u"/>: the gimbals' share at their factor, the rest at the RCS's.</summary>
        public void UpdateEffective(Dynamics6Dof.Params plan, double[] u)
        {
            if (Frozen && _set)
                return;
            _set = true;
            (double[] g, double[] r) = Split(u);
            double[][] gimbalK = { new[] { 0.0, 1.0 / Arm, 0.0 }, new[] { -1.0 / Arm, 0.0, 0.0 }, new[] { 0.0, 0.0, 0.0 } };
            for (int a = 0; a < 3; a++)
            {
                double total = Math.Abs(g[a]) + Math.Abs(r[a]);
                double share = total > 1.0 ? Math.Abs(g[a]) / total : 1.0;
                for (int i = 0; i < 3; i++)
                    plan.Rcs!.ForcePerNm[3 * a + i] = share * gimbalK[a][i] + (1.0 - share) * rcs.ForcePerNm[3 * a + i];
            }
        }

        public void Reset(Dynamics6Dof.Params plan)
        {
            PeakRcs = PeakGimbal = 0.0;
            _set = false;
            UpdateEffective(plan, new double[NU]);
            _set = false;
        }
    }

    private static void Shift(double[] X, double[] U, double sigma, double elapsed, int n, out double[] sx, out double[] su, out double ss)
    {
        ss = Math.Max(sigma - elapsed, 0.3);
        sx = new double[n * NX];
        su = new double[n * NU];
        for (int k = 0; k < n; k++)
        {
            double tk = elapsed + k * ss / (n - 1);
            Array.Copy(Sample(X, NX, sigma, n, tk), 0, sx, k * NX, NX);
            Array.Copy(Sample(U, NU, sigma, n, tk), 0, su, k * NU, NU);
        }
    }

    private static double[] Sample(double[] V, int width, double sigma, int n, double t)
    {
        double sIdx = Math.Clamp(t / (sigma / (n - 1)), 0.0, n - 1.0);
        int a = Math.Min((int)sIdx, n - 2);
        double fr = sIdx - a;
        var r = new double[width];
        for (int i = 0; i < width; i++)
            r[i] = V[a * width + i] * (1 - fr) + V[(a + 1) * width + i] * fr;
        return r;
    }

    internal static void Step(double[] x, double[] u, Dynamics6Dof.Params p, double h)
    {
        Span<double> k1 = stackalloc double[NX], k2 = stackalloc double[NX], k3 = stackalloc double[NX], k4 = stackalloc double[NX], tmp = stackalloc double[NX];
        Dynamics6Dof.Eval(x, u, p, k1);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k1[i];
        Dynamics6Dof.Eval(tmp, u, p, k2);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + 0.5 * h * k2[i];
        Dynamics6Dof.Eval(tmp, u, p, k3);
        for (int i = 0; i < NX; i++) tmp[i] = x[i] + h * k3[i];
        Dynamics6Dof.Eval(tmp, u, p, k4);
        for (int i = 0; i < NX; i++) x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        double norm = Math.Sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
        for (int i = 6; i < 10; i++) x[i] /= norm;
    }
}

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
    private const double Tmax = 2171e3, Floor = 0.10, Isp = 290.0, Gravity = 9.81;
    private const double Arm = 10.0, Ipitch = 1.14e6, Iroll = 2.7e4, Length = 40.0;
    private const double TargetZ = 13.0, AccelMax = 31.0, GimbalDeg = 2.0, SlewPerS = 2.0;
    private const double Area = ThreeDofScenario.Area;

    // Its RCS: eight 6.7 kN thrusters, Isp 220 s, at the top, 28 m over the centre of mass, on a 1.85 m radius.
    private const double RcsHeight = 28.0, RcsRadius = 1.85, RcsThrust = 6.7e3, RcsIsp = 220.0;

    // The air's turn on the logged booster, rad/s^2 per degree off the airflow at 100 m/s.
    private const double AirTurnPerDeg = 0.04, PlantMomentFactor = 1.1;

    // The model's +Z (thrust axis) along the game's +x (nose); see SixDofAeroCheck.GameAxes.
    private static readonly double[] GameAxes = [0, 0, 1, 1, 0, 0, 0, 1, 0];

    // --rcs-weight w: Scvx6DofConfig.RcsWeight for the plans, 0 for none.
    private static double RcsWeight;

    // Diagnostics of the SCS slowdown with RCS (2026-11-19): --diag runs only the fast case's closed loop and prints the first warm re-solves' SCS outcome; with it, --rcs-scale f scales the RCS authority, --rcs-noforce drops its side force, --gimbal flies gimbals alone, --nowarm turns SCS's warm start off.
    private static double RcsScale = 1.0;
    private static bool Diag;

    internal static int Run(bool verbose)
    {
        string[] args = Environment.GetCommandLineArgs();
        int wi = Array.IndexOf(args, "--rcs-weight");
        RcsWeight = wi >= 0 && wi + 1 < args.Length ? double.Parse(args[wi + 1], CultureInfo.InvariantCulture) : 0.0;
        int si = Array.IndexOf(args, "--rcs-scale");
        RcsScale = si >= 0 && si + 1 < args.Length ? double.Parse(args[si + 1], CultureInfo.InvariantCulture) : 1.0;
        Diag = args.Contains("--diag");
        Console.WriteLine($"RCS weight {RcsWeight}, authority x{RcsScale}");
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
            var st = States[0];
            double[] x0 = State(st.h, st.r, st.v, st.tiltDeg, 16_400.0);
            Console.WriteLine("    " + Loop(Params(MomentPerDeg(1.0), m), Params(MomentPerDeg(PlantMomentFactor), m), x0, st.sigma, false));
            return 0;
        }
        Console.WriteLine("TOUCHDOWN GUIDANCE WITH RCS TORQUE");
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

    private static Dynamics6Dof.RcsModel RcsModel()
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
        }
    }

    private static Dynamics6Dof.Params Params(BodyAeroTable moment, Dynamics6Dof.RcsModel? rcs) => new()
    {
        Gx = 0, Gy = 0, Gz = -Gravity, G0 = 9.80665, Isp = Isp,
        LArm = Arm, Ixx = Ipitch, Iyy = Ipitch, Izz = Iroll,
        Aero = ThreeDofScenario.Model(),
        BodyAero = new BodyAero6Dof
        {
            Atmosphere = ThreeDofScenario.Model().Atmosphere!,
            Force = ThreeDofScenario.BodyTable(),
            Moment = moment,
            ReferenceArea = Area,
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
    private static BodyAeroTable MomentPerDeg(double factor)
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
                    c[i++] = 0.0;
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
    private static double OffAir(double[] x)
    {
        double n = Math.Sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
        double qw = x[6] / n, qx = x[7] / n, qy = x[8] / n, qz = x[9] / n;
        double bx = 2.0 * (qx * qz + qw * qy), by = 2.0 * (qy * qz - qw * qx), bz = 1.0 - 2.0 * (qx * qx + qy * qy);
        double sp = Math.Sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
        double c = -(bx * x[3] + by * x[4] + bz * x[5]) / sp;
        return Math.Acos(Math.Clamp(c, -1.0, 1.0));
    }

    /// <summary>A state at height h, position r, velocity v, the thrust axis tilted off retrograde about the site's x (about = 0) or y (1) axis.</summary>
    private static double[] State(double h, double[] r, double[] v, double tiltDeg, double mass, int about = 0)
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

    private static double[] Target(double mass)
    {
        var xf = new double[NX];
        xf[2] = TargetZ;
        xf[6] = 1.0;
        xf[13] = mass;
        return xf;
    }

    private static Scvx6DofConfig Config(int n, double sigma, double[] x0, double[] xf, Dynamics6Dof.Params dyn)
    {
        double length = Math.Sqrt(x0[0] * x0[0] + x0[1] * x0[1] + (x0[2] - xf[2]) * (x0[2] - xf[2]));
        double speed = Math.Sqrt(x0[3] * x0[3] + x0[4] * x0[4] + x0[5] * x0[5]);
        double velocity = Math.Max(Math.Max(speed, Math.Sqrt(length * Gravity)), 1.0);
        double omegaScale = 60.0 * Math.PI / 180.0 / (sigma * 0.5);
        return new Scvx6DofConfig
        {
            Nodes = n,
            Tmax = Tmax,
            ThrottleFloor = Floor,
            GimbalMaxDeg = GimbalDeg,
            GimbalSlewPerS = SlewPerS,
            TauRollMax = 1.0e4,
            TerminalRollFree = true,
            TiltMaxDeg = 60.0,
            GroundFloor = -1.0,
            AccelMax = AccelMax,
            WW = 0.002 / (n * 3.0 * omegaScale * omegaScale),
            WDu = 0.05,
            ProximalWeight = 0.05,
            GlideSlopeDeg = 0.0,
            VzMax = 0.0,
            TerminalMissWeight = 1.0e5,
            TerminalSpeedWeight = 1.0e4,
            SigmaMin = sigma * 0.15,
            SigmaMax = sigma * 4.0,
            SigmaScale = sigma,
            RcsMaxPosNm = dyn.Rcs?.MaxPosNm,
            RcsMaxNegNm = dyn.Rcs?.MaxNegNm,
            RcsWeight = RcsWeight,
            VirtualControlScale = [0, 0, 0, 5, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0],
            XScale = [length, length, length, velocity, velocity, velocity, 1, 1, 1, 1, 1, 1, 1, x0[13]],
        };
    }

    private static Scvx6DofSolver Solver(Scvx6DofConfig cfg, Dynamics6Dof.Params dyn)
    {
        var s = new Scvx6DofSolver(cfg, dyn) { SubproblemEps = 3e-4, WarmStart = !Environment.GetCommandLineArgs().Contains("--nowarm") };
        s.MaxSubproblemIterations = 9000;
        s.EscalatedSubproblemIterations = 9000;
        return s;
    }

    private static double[] Tolerance(double mass)
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
    private static double PeakRcs(double[] u, int n, out double share)
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

    private static string Loop(Dynamics6Dof.Params dyn, Dynamics6Dof.Params plant, double[] x0, double sigma, bool verbose)
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
        double fs = s.Sigma, f0 = 0.0;
        double[] x = (double[])x0.Clone();
        int refused = 0, kept = 0;
        var gates = new List<double>();
        var loopAdmm = new List<int>();
        double t = 0.0, maxOff = 0.0, mass0 = x0[13];
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
                Step(x, u, plant, dt);
                maxOff = Math.Max(maxOff, OffAir(x) * 180.0 / Math.PI);
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
            $"first plan {(planned ? "inside" : "OUTSIDE")} the gate; {kept} kept, {refused} refused, median gate {med:F2}x, SCS {Median(loopAdmm)}/iteration; ends {x[2]:F1} m up, {miss:F1} m off, {-x[5]:F1} m/s down, {side:F1} m/s sideways; {mass0 - x[13]:F0} kg burned");
    }

    private static int Median(List<int> v)
    {
        if (v.Count == 0) return 0;
        var c = new List<int>(v);
        c.Sort();
        return c[c.Count / 2];
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

    private static void Step(double[] x, double[] u, Dynamics6Dof.Params p, double h)
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

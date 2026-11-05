using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// Does the C# ascent SCvx solve launch3dof.py's problem to launch3dof.py's answer?
///
/// The Saturn V scenario from "The Saturn V Ascent Problem", with the script's own atmosphere (exponential density, ISA-troposphere speed of sound, Gaussian transonic drag rise) and its own vehicle, so every difference that remains is the port and not the model. The reference numbers are the script's output on 2026-09-24 (46 iterations, 148.5 t to orbit, kick 2.4755 deg).
///
/// On 2026-09-24 the port reproduced the script's log iteration for iteration - the same 46 iterations, 37 accepted, the same merit to five figures at every step - in 3.4 s against the script's 180. The iteration count is still not asserted: CVXPY reformulates the problem before Clarabel sees it, so the two interior-point paths can part company in the last digits and a near-tie in the ratio test would then change the count. The OPTIMUM is asserted - the mass it delivers, the burn times that deliver it, and which structural limit shapes it - because that is the part both must agree on.
/// </summary>
internal static class AscentCheck
{
    private const double Mu = 3.986004418e14;
    private const double Re = 6378137.0;
    private const double OmegaE = 7.2921159e-5;
    private const double G0 = 9.80665;

    /// <summary>launch3dof.py's air.</summary>
    private sealed class ScriptAtmosphere : AscentAtmosphere
    {
        public override Dual Density(Dual h) => 1.225 * Dual.Exp(new Dual(-h.V / 8500.0, -h.D / 8500.0));

        public override Dual SpeedOfSound(Dual h)
        {
            Dual t = 288.15 - 0.0065 * h;
            if (t.V < 216.65) t = new Dual(216.65);
            return Math.Sqrt(1.4 * 287.0) * Dual.Sqrt(t);
        }

        public override Dual Pressure(Dual h) => new(0.0);

        public override Dual DragCoefficient(Dual mach)
        {
            Dual z = (mach - 1.1) / 0.35;
            return 0.4 * (1.0 + 0.75 * Dual.Exp(-(z * z)));
        }
    }

    /// <summary>launch3dof.py's Saturn V. <paramref name="offPlaneDeg"/> tilts the target plane away from the pad by that angle about the plane's line through it, which the script's own case (inclination = latitude, the pad at the plane's northernmost point) has at zero. <paramref name="incDeg"/> changes the inclination, with the LAN of the plane whose ascending pass is over the pad; at the script's 28.5 deg that is its own 270 deg.</summary>
    internal static AscentProblem SaturnV(double offPlaneDeg = 0.0, double incDeg = 28.5)
    {
        double[] isp = [290.0, 421.0, 421.0];
        double[] tmax = [3.45e7, 5.00e6, 1.00e6];
        double[] prop = [2.15e6, 0.45e6, 0.109e6];
        double[] dry = [0.13e6, 0.04e6, 0.013e6];
        var stages = new AscentStage[3];
        for (int i = 0; i < 3; i++)
            stages[i] = new AscentStage(tmax[i], tmax[i] / (isp[i] * G0), prop[i],
                                        i < 2 ? dry[i] : 0.0, 80.0);

        double lat = 28.5 * Math.PI / 180.0;
        double[] site = [Math.Cos(lat), 0.0, Math.Sin(lat)];
        double r = Re + 100.0;
        double[] r0 = [r * site[0], r * site[1], r * site[2]];
        // v0 = omega x r0 + 10 m/s straight up.
        double[] v0 = [-OmegaE * r0[1] + 10.0 * site[0], OmegaE * r0[0] + 10.0 * site[1], 10.0 * site[2]];

        // The ascending node is asin(tan lat / tan inc) of right ascension behind the pad, which is at right ascension zero.
        double inc = incDeg * Math.PI / 180.0;
        double lan = incDeg == 28.5 ? 270.0 * Math.PI / 180.0
            : -Math.Asin(Math.Clamp(Math.Tan(lat) / Math.Tan(inc), -1.0, 1.0));
        double rt = Re + 200e3;
        return new AscentProblem
        {
            Mu = Mu,
            BodyRadius = Re,
            Omega = OmegaE,
            R0 = r0,
            V0 = v0,
            M0 = 2.97e6,
            Stages = stages,
            Atmosphere = new ScriptAtmosphere(),
            GroundRadius = Re,
            TargetRadius = rt,
            TargetSpeed = Math.Sqrt(Mu / rt),
            TargetRadialRate = 0.0,
            PlaneNormal = OffPlane([Math.Sin(inc) * Math.Sin(lan), -Math.Sin(inc) * Math.Cos(lan), Math.Cos(inc)], site, offPlaneDeg),
            QMax = 35000.0,
            QAlphaMax = 3500.0,
        };
    }

    /// <summary>The plane normal n turned toward the pad's radial by the angle, so the pad sits that far off the plane: n stays a unit vector because the radial is perpendicular to it.</summary>
    private static double[] OffPlane(double[] n, double[] site, double deg)
    {
        double a = deg * Math.PI / 180.0, c = Math.Cos(a), s = Math.Sin(a);
        return [c * n[0] + s * site[0], c * n[1] + s * site[1], c * n[2] + s * site[2]];
    }

    /// <summary>Solve a problem the game wrote out (see AscentProblemFile), optionally with fewer stages, printing the whole trace.</summary>
    internal static int Replay(string path, string[] args)
    {
        if (!File.Exists(path))
        {
            Console.WriteLine($"no such problem file: '{path}'");
            return 2;
        }
        AscentProblem p = AscentProblemFile.FromJson(File.ReadAllText(path));
        int si = Array.IndexOf(args, "--stages");
        if (si >= 0 && si + 1 < args.Length && int.TryParse(args[si + 1], out int k) && k >= 1 && k < p.Stages.Length)
        {
            var stages = p.Stages.Take(k).ToArray();
            AscentStage last = stages[^1];
            stages[^1] = new AscentStage(last.Thrust, last.MassFlow, last.PropellantMass, 0.0, last.DragArea,
                                         last.PressureGrid, last.ThrustAtPressure);
            p = new AscentProblem
            {
                Mu = p.Mu, BodyRadius = p.BodyRadius, Omega = p.Omega, R0 = p.R0, V0 = p.V0, M0 = p.M0,
                Stages = stages, Atmosphere = p.Atmosphere, GroundRadius = p.GroundRadius,
                TargetRadius = p.TargetRadius, TargetSpeed = p.TargetSpeed, TargetRadialRate = p.TargetRadialRate,
                PlaneNormal = p.PlaneNormal, QMax = p.QMax, QAlphaMax = p.QAlphaMax,
                PlaneTurnMin = p.PlaneTurnMin, PlaneTurnMax = p.PlaneTurnMax,
                Recovery = p.Recovery is { } rc && rc.Stage < stages.Length - 1 ? rc : null,
            };
        }

        double Arg(string name, double fallback)
        {
            int i = Array.IndexOf(args, name);
            return i >= 0 && i + 1 < args.Length && double.TryParse(args[i + 1], System.Globalization.CultureInfo.InvariantCulture, out double v) ? v : fallback;
        }
        double qMax = Arg("--qmax", p.QMax / 1000.0) * 1000.0, qaMax = Arg("--qamax", p.QAlphaMax);
        if (qMax != p.QMax || qaMax != p.QAlphaMax)
            p = new AscentProblem
            {
                Mu = p.Mu, BodyRadius = p.BodyRadius, Omega = p.Omega, R0 = p.R0, V0 = p.V0, M0 = p.M0,
                Stages = p.Stages, Atmosphere = p.Atmosphere, GroundRadius = p.GroundRadius,
                TargetRadius = p.TargetRadius, TargetSpeed = p.TargetSpeed, TargetRadialRate = p.TargetRadialRate,
                PlaneNormal = p.PlaneNormal, QMax = qMax, QAlphaMax = qaMax,
                PlaneTurnMin = p.PlaneTurnMin, PlaneTurnMax = p.PlaneTurnMax,
                Recovery = p.Recovery,
            };
        // --turn S: free the launch time within S seconds either way, as a plane turn of Omega S (AscentProblem.PlaneTurnMin).
        double turnS = Arg("--turn", double.NaN);
        if (double.IsFinite(turnS))
            p = p.WithPlaneTurn(-Math.Abs(p.Omega) * turnS, Math.Abs(p.Omega) * turnS);
        p.Settings.ThrottleMin = Arg("--floor", p.Settings.ThrottleMin);
        // A stage's own floor wins over the vehicle-wide one, and the game writes one per stage, so --floor sets those too.
        if (args.Contains("--floor"))
        {
            double fl = p.Settings.ThrottleMin;
            AscentStage[] st = p.Stages.Select(x => double.IsNaN(x.ThrottleMin) || x.ThrottleMin < 0.99 ? x.WithThrottleMin(fl) : x).ToArray();   // a stage held at full thrust (solids, 0.99) stays held
            p = new AscentProblem
            {
                Mu = p.Mu, BodyRadius = p.BodyRadius, Omega = p.Omega, R0 = p.R0, V0 = p.V0, M0 = p.M0, Stages = st,
                Atmosphere = p.Atmosphere, GroundRadius = p.GroundRadius, TargetRadius = p.TargetRadius, TargetSpeed = p.TargetSpeed,
                TargetRadialRate = p.TargetRadialRate, PlaneNormal = p.PlaneNormal, QMax = p.QMax, QAlphaMax = p.QAlphaMax,
                PlaneTurnMin = p.PlaneTurnMin, PlaneTurnMax = p.PlaneTurnMax, LiftoffDirection = p.LiftoffDirection, Settings = p.Settings,
                Recovery = p.Recovery,
            };
        }
        // --set Name=value (repeatable): any numeric or boolean AscentSettings field, to try a tuning against a dumped problem.
        for (int i = 0; i + 1 < args.Length; i++)
        {
            if (args[i] != "--set") continue;
            string[] kv = args[i + 1].Split('=', 2);
            System.Reflection.FieldInfo? fi = kv.Length == 2 ? typeof(AscentSettings).GetField(kv[0]) : null;
            if (fi == null)
            {
                Console.WriteLine($"--set: no AscentSettings field '{args[i + 1]}'");
                return 2;
            }
            fi.SetValue(p.Settings, Convert.ChangeType(kv[1], fi.FieldType, System.Globalization.CultureInfo.InvariantCulture));
            Console.WriteLine($"setting {fi.Name} = {fi.GetValue(p.Settings)}");
        }
        double r0 =Math.Sqrt(p.R0.Sum(x => x * x));
        Console.WriteLine($"body R {p.BodyRadius / 1000.0:F1} km, mu {p.Mu:E4}, omega {p.Omega:E4}; lift-off {(r0 - p.BodyRadius):F0} m up, {p.M0 / 1000.0:F2} t");
        Console.WriteLine($"target {(p.TargetRadius - p.BodyRadius) / 1000.0:F1} km at {p.TargetSpeed:F1} m/s, r.v {p.TargetRadialRate:E3}; q {p.QMax / 1000.0:F1} kPa, q-alpha {p.QAlphaMax:F0}; floor {p.FinalMassFloor / 1000.0:F2} t");
        for (int i = 0; i < p.Stages.Length; i++)
        {
            AscentStage s = p.Stages[i];
            string tp = s.PressureGrid == null ? "constant"
                : $"{s.ThrustAtPressure![0] / 1000.0:F0} kN vac .. {s.ThrustAtPressure[^1] / 1000.0:F0} kN at {s.PressureGrid[^1]:F0} Pa";
            Console.WriteLine($"  stage {i + 1}: {s.Thrust / 1000.0:F0} kN ({tp}), {s.MassFlow:F1} kg/s, ve {s.ExhaustVelocity:F0} m/s, "
                            + $"prop {s.PropellantMass / 1000.0:F2} t ({s.FullBurnTime:F1} s), drop {s.JettisonMass / 1000.0:F2} t, drag area {s.DragArea:F1} m^2");
        }

        AscentSolution sol = AscentScvx.Solve(p, it =>
            Console.WriteLine($"iter {it.Index}: rho={it.Rho:+0.00;-0.00} {it.Status,-7} tr={it.TrustRegion:F3}  "
                + $"sig={string.Join("/", it.SigmaSeconds.Select(x => x.ToString("F0")))}s  pred={it.Predicted:E1}  "
                + $"defect_n={it.DefectNorm:E2}  pviol={it.PathViolation:E1}  term={it.TerminalViolation:E1}  "
                + $"dv={it.DeltaV:F0}  J={it.Cost:E5}  clarabel={it.SolverIterations} {it.SolveMs:F0} ms"),
            phase => Console.WriteLine("-- " + phase));
        Console.WriteLine($"{sol.Message}: kick {sol.KickDeg:F3} deg, seed {sol.SeedSeconds:F1} s, total {sol.SolveSeconds:F1} s, "
            + $"seeds' least max q {sol.SeedLeastMaxQ / 1000.0:F1} kPa");
        if (sol.Nodes > 0)
            Console.WriteLine($"to orbit {sol.FinalMass / 1000.0:F2} t, burns {string.Join(" / ", sol.BurnTime.Select(b => b.ToString("F1")))} s, "
                + $"max q {sol.DynamicPressure.Max() / 1000.0:F1} kPa, max q-alpha {sol.QAlpha.Max():F0}, miss {sol.TerminalResidual[0]:F0} m / {sol.TerminalResidual[1]:F2} m/s");
        if (double.IsFinite(sol.RecoveryReserveKg))
            Console.WriteLine($"booster keeps {sol.RecoveryReserveKg:F0} kg against a need of {sol.RecoveryRequiredKg:F0} kg: {sol.BoostbackDv:F0} m/s boostback, {sol.LandingDv:F0} m/s landing, separates at {sol.SeparationTime:F1} s");
        if (p.PlaneTurnMin != p.PlaneTurnMax)
            Console.WriteLine($"plane turned {sol.PlaneTurn * 180.0 / Math.PI:F3} deg: a lift-off {(p.Omega != 0.0 ? -sol.PlaneTurn / p.Omega : double.NaN):F1} s later, or that much more LAN");
        Console.WriteLine($"{(sol.Usable ? "usable" : "not usable")}: {sol.Status}, gates {(sol.GatesMet ? "met" : "not met")}, last predicted gain {sol.RemainingGainKg:F2} kg");
        // --throttle: the plan's throttle node by node, where it is below full - what UPFG would have to inherit after the hand-over.
        if (args.Contains("--throttle"))
            for (int n = 0; n < sol.Nodes; n++)
            {
                double u = Math.Sqrt(sol.Throttle[3 * n] * sol.Throttle[3 * n] + sol.Throttle[3 * n + 1] * sol.Throttle[3 * n + 1] + sol.Throttle[3 * n + 2] * sol.Throttle[3 * n + 2]);
                double rr = Math.Sqrt(sol.Position[3 * n] * sol.Position[3 * n] + sol.Position[3 * n + 1] * sol.Position[3 * n + 1] + sol.Position[3 * n + 2] * sol.Position[3 * n + 2]);
                Console.WriteLine($"  node {n,3} stage {sol.NodeStage[n] + 1} T+{sol.Time[n],6:F1} s {(rr - p.BodyRadius) / 1000.0,6:F1} km throttle {u * 100.0,5:F1} % accel {u * p.Stages[sol.NodeStage[n]].Thrust / sol.Mass[n] / 9.80665,5:F2} g");
            }
        return sol.Usable ? 0 : 1;
    }

    internal static int Run(bool verbose)
    {
        int fails = 0;
        void Check(string name, bool ok, string detail = "")
        {
            Console.WriteLine($"  {(ok ? "PASS" : "FAIL")}  {name,-44} {detail}");
            if (!ok) fails++;
        }

        Console.WriteLine("ASCENT SCVX: launch3dof.py's Saturn V, against the script's own result");

        // Twice: with the script's own seed, which reproduces its run; and with the energy cut-off the mod seeds with by default, which must reach the same optimum.
        foreach (bool scriptSeed in new[] { true, false })
        {
            Console.WriteLine();
            Console.WriteLine(scriptSeed ? "the script's seed (last stage burns 80 % of its load)"
                                         : "the default seed (last stage burns to the target energy, kick scanned 0.25-30 deg)");
            AscentProblem p = SaturnV();
            if (scriptSeed)
                p.Settings.UseScriptSeed();
            AscentSolution s = AscentScvx.Solve(p, it =>
            {
                if (!verbose) return;
                Console.WriteLine($"iter {it.Index}: rho={it.Rho:+0.00;-0.00} {it.Status,-7} tr={it.TrustRegion:F3}  "
                    + $"sig={string.Join("/", it.SigmaSeconds.Select(x => x.ToString("F0")))}s  pred={it.Predicted:E1}  "
                    + $"defect_n={it.DefectNorm:E2}  pviol={it.PathViolation:E1}  term={it.TerminalViolation:E1}  "
                    + $"dv={it.DeltaV:F0}  J={it.Cost:E5}  clarabel={it.SolverIterations} {it.SolveMs:F0} ms");
            });

            Console.WriteLine($"  {s.Message}: {s.Iterations} iterations ({s.Accepted} accepted), seed {s.SeedSeconds:F1} s, "
                            + $"total {s.SolveSeconds:F1} s, kick {s.KickDeg:F4} deg");
            if (s.Nodes == 0)
            {
                Check("solved at all", false, s.Message);
                continue;
            }

            double qMax = s.DynamicPressure.Max(), qaMax = s.QAlpha.Max();
            Console.WriteLine($"  to orbit {s.FinalMass / 1000.0:F2} t, burns {string.Join(" / ", s.BurnTime.Select(b => b.ToString("F1")))} s, "
                            + $"dv {s.TotalDeltaV:F0} m/s, max q {qMax / 1000.0:F1} kPa, max q.alpha {qaMax:F0} Pa.rad");
            Console.WriteLine($"  defects {s.MaxDefectPosition:F2} m, {s.MaxDefectVelocity:F4} m/s, {s.MaxDefectMass:F1} kg; "
                            + $"insertion {s.TerminalResidual[0]:+0.0;-0.0} m, {s.TerminalResidual[1]:+0.000;-0.000} m/s, "
                            + $"n.r {s.TerminalResidual[3]:+0.0;-0.0} m");

            // The script: kick 2.4755 deg; 177.2 / 371.8 / 215.7 s; 148.5 t; max q 29.8 kPa; q.alpha held at 3430.
            if (scriptSeed)
                Check("kick angle matches the script's search", Math.Abs(s.KickDeg - 2.4755) < 0.005, $"{s.KickDeg:F4} deg");
            Check("converged", s.Converged, s.Message);
            Check("mass to orbit", Math.Abs(s.FinalMass - 148.5e3) < 0.5e3, $"{s.FinalMass / 1000.0:F2} t (script 148.5)");
            Check("S-IC burns its load", Math.Abs(s.BurnTime[0] - 177.2) < 1.0, $"{s.BurnTime[0]:F1} s (script 177.2)");
            Check("S-II burns its load", Math.Abs(s.BurnTime[1] - 371.8) < 1.5, $"{s.BurnTime[1]:F1} s (script 371.8)");
            Check("S-IVB burn", Math.Abs(s.BurnTime[2] - 215.7) < 3.0, $"{s.BurnTime[2]:F1} s (script 215.7)");
            Check("q.alpha binds at the back-off, not past it", qaMax < 3500.0 * 1.002 && qaMax > 3400.0, $"{qaMax:F0} Pa.rad");
            Check("max q is not the binding limit", qMax < 35000.0 && Math.Abs(qMax - 29.8e3) < 1.0e3, $"{qMax / 1000.0:F1} kPa (script 29.8)");
            Check("insertion radius", Math.Abs(s.TerminalResidual[0]) < 150.0, $"{s.TerminalResidual[0]:F1} m");
            Check("insertion speed", Math.Abs(s.TerminalResidual[1]) < 0.2, $"{s.TerminalResidual[1]:F3} m/s");
            Check("in the plane", Math.Abs(s.TerminalResidual[3]) < 150.0 && Math.Abs(s.TerminalResidual[4]) < 0.2,
                  $"{s.TerminalResidual[3]:F1} m, {s.TerminalResidual[4]:F3} m/s");
            Check("dynamically feasible", s.MaxDefectPosition < 35.0 && s.MaxDefectVelocity < 0.8,
                  $"{s.MaxDefectPosition:F2} m, {s.MaxDefectVelocity:F4} m/s");
        }

        // OFF THE PLANE (#90). The seed's thrust is projected into the target plane, so off it the seed's lift-off thrust leans away from the straight-up one the problem pins. The first subproblem must still be solvable, the plan must still reach the plane, and leaving the plane must cost propellant. Before the fix every case from 7 deg on failed its first subproblem and every retry after it; now 5 and 10 deg converge, and 20 deg, past what the S-IVB can turn, has only to get past the first subproblem.
        Console.WriteLine();
        Console.WriteLine("the pad off the target plane (#90)");
        foreach (double off in new[] { 5.0, 10.0, 20.0 })
        {
            AscentSolution s = AscentScvx.Solve(SaturnV(off));
            bool firstSolved = s.Trace.Count > 0 && s.Trace[0].Solved;
            Console.WriteLine($"  {off:F0} deg: {s.Message}, {s.Iterations} iterations"
                            + (s.Nodes > 0 ? $", to orbit {s.FinalMass / 1000.0:F2} t, n.r {s.TerminalResidual[3]:F1} m" : ""));
            Check($"{off:F0} deg off: the first subproblem solves", firstSolved, s.Trace.Count > 0 ? s.Trace[0].Status : "no iterations");
            Check($"{off:F0} deg off: no subproblem fails", s.Trace.All(t => t.Solved),
                  $"{s.Trace.Count(t => !t.Solved)} of {s.Trace.Count} failed");
            if (off <= 10.0)
            {
                Check($"{off:F0} deg off: converged", s.Converged, s.Message);
                Check($"{off:F0} deg off: in the plane", s.Nodes > 0 && Math.Abs(s.TerminalResidual[3]) < 150.0 && Math.Abs(s.TerminalResidual[4]) < 0.2,
                      s.Nodes > 0 ? $"{s.TerminalResidual[3]:F1} m, {s.TerminalResidual[4]:F3} m/s" : "");
                Check($"{off:F0} deg off: costs propellant", s.Nodes > 0 && s.FinalMass < 148.5e3 - 1.0e3,
                      s.Nodes > 0 ? $"{s.FinalMass / 1000.0:F2} t against 148.5 in the plane" : "");
            }
        }

        // A FREE LAUNCH TIME. The Saturn V into 51.6 deg, whose window was ten minutes ago: lifting off now is late, the pad has turned east of the plane. Not the script's 28.5 deg: there the pad is at the plane's northernmost point, where ten minutes of spin hardly moves it off the plane, and every launch time puts the same 148.5 t into orbit. Free to lift off up to twenty minutes either way, the plan must find the launch time a sweep of fixed launch times finds best, and put that much more into orbit than lifting off now. A launch delay t is the plane turned by -Omega t (AscentProblem.PlaneTurnMin), so the sweep turns the plane and the free solve reports its turn as a delay.
        Console.WriteLine();
        Console.WriteLine("a free launch time, the window ten minutes ago");
        {
            const double lateS = 600.0, freeS = 1200.0;
            // The pad passed under the plane lateS ago, so the problem now is the unturned one lifting off lateS later: the plane turned by -Omega lateS.
            AscentProblem Late() => SpinTurned(SaturnV(0.0, 51.6), -OmegaE * lateS);
            AscentSolution now = AscentScvx.Solve(Late());
            double bestDelay = double.NaN, bestMass = 0.0;
            foreach (double delay in new[] { -1400.0, -1300.0, -1200.0, -1100.0, -1000.0, -900.0, -800.0, -700.0, -600.0 })
            {
                AscentSolution d = AscentScvx.Solve(SpinTurned(Late(), -OmegaE * delay));
                Console.WriteLine($"  fixed, lifting off {delay:+0;-0} s from now: {d.Message}, {(d.Nodes > 0 ? d.FinalMass / 1000.0 : double.NaN):F2} t");
                if (d.Converged && d.FinalMass > bestMass)
                {
                    bestMass = d.FinalMass;
                    bestDelay = delay;
                }
            }
            AscentSolution free = AscentScvx.Solve(Late().WithPlaneTurn(-OmegaE * freeS, OmegaE * freeS));
            double freeDelay = -free.PlaneTurn / OmegaE;
            Console.WriteLine($"  now: {now.Message}, {(now.Nodes > 0 ? now.FinalMass / 1000.0 : double.NaN):F2} t; "
                            + $"free: {free.Message}, {(free.Nodes > 0 ? free.FinalMass / 1000.0 : double.NaN):F2} t, lifting off {freeDelay:+0;-0} s from now; "
                            + $"the sweep's best {bestMass / 1000.0:F2} t at {bestDelay:+0;-0} s");
            Check("free launch time: converged", free.Converged, free.Message);
            Check("free launch time: as much as the sweep's best", free.Nodes > 0 && free.FinalMass > bestMass - 0.3e3,
                  $"{free.FinalMass / 1000.0:F2} t against {bestMass / 1000.0:F2} t");
            Check("free launch time: near the sweep's best time", Math.Abs(freeDelay - bestDelay) < 150.0,
                  $"{freeDelay:+0;-0} s against {bestDelay:+0;-0} s");
            Check("free launch time: beats lifting off now", now.Nodes > 0 && free.FinalMass > now.FinalMass + 1.0e3,
                  $"{free.FinalMass / 1000.0:F2} t against {now.FinalMass / 1000.0:F2} t");
            Check("free launch time: in the turned plane", Math.Abs(free.TerminalResidual[3]) < 150.0 && Math.Abs(free.TerminalResidual[4]) < 0.2,
                  $"{free.TerminalResidual[3]:F1} m, {free.TerminalResidual[4]:F3} m/s");
            Check("free launch time: within its bounds", Math.Abs(freeDelay) <= freeS + 1e-6, $"{freeDelay:F1} s");

            // Turned back by the spin over the delay, the plan is the same flight from where the pad will be: in the problem's own plane, and with the turn used up.
            AscentSolution flown = free.RotatedAboutSpin(OmegaE * freeDelay);
            double[] n0 = Late().PlaneNormal;
            int f = (flown.Nodes - 1) * 3;
            double nr = n0[0] * flown.Position[f] + n0[1] * flown.Position[f + 1] + n0[2] * flown.Position[f + 2];
            Check("free launch time: turned by the spin, in the plane itself", Math.Abs(nr) < 150.0 && Math.Abs(flown.PlaneTurn) < 1e-12,
                  $"{nr:F1} m, turn {flown.PlaneTurn:E1} rad");

            // Pinned, the variable must change nothing: the script's own run, iteration for iteration.
            AscentSolution pinned = AscentScvx.Solve(SaturnV().WithPlaneTurn(0.0, 0.0));
            AscentSolution plain = AscentScvx.Solve(SaturnV());
            Check("pinned turn: the same run", pinned.Iterations == plain.Iterations && Math.Abs(pinned.FinalMass - plain.FinalMass) < 1e-6,
                  $"{pinned.Iterations} against {plain.Iterations} iterations, {pinned.FinalMass - plain.FinalMass:E1} kg");
        }

        Console.WriteLine();
        Console.WriteLine(fails == 0 ? "ALL PASS" : $"{fails} FAILED");
        return fails == 0 ? 0 : 1;
    }

    /// <summary>The problem with its target plane turned about the spin axis by <paramref name="angle"/>, radians: lifting off -angle / Omega later.</summary>
    private static AscentProblem SpinTurned(AscentProblem p, double angle)
    {
        double c = Math.Cos(angle), s = Math.Sin(angle);
        double[] n = p.PlaneNormal;
        return new AscentProblem
        {
            Mu = p.Mu, BodyRadius = p.BodyRadius, Omega = p.Omega, R0 = p.R0, V0 = p.V0, M0 = p.M0, Stages = p.Stages,
            Atmosphere = p.Atmosphere, GroundRadius = p.GroundRadius, TargetRadius = p.TargetRadius, TargetSpeed = p.TargetSpeed,
            TargetRadialRate = p.TargetRadialRate, PlaneNormal = [c * n[0] - s * n[1], s * n[0] + c * n[1], n[2]],
            QMax = p.QMax, QAlphaMax = p.QAlphaMax, PlaneTurnMin = p.PlaneTurnMin, PlaneTurnMax = p.PlaneTurnMax,
            LiftoffDirection = p.LiftoffDirection, Settings = p.Settings,
        };
    }
}

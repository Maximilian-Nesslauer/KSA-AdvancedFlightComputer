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

    internal static AscentProblem SaturnV()
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

        double inc = 28.5 * Math.PI / 180.0, lan = 270.0 * Math.PI / 180.0;
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
            PlaneNormal = [Math.Sin(inc) * Math.Sin(lan), -Math.Sin(inc) * Math.Cos(lan), Math.Cos(inc)],
            QMax = 35000.0,
            QAlphaMax = 3500.0,
        };
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
            };
        p.Settings.ThrottleMin = Arg("--floor", p.Settings.ThrottleMin);
        if (args.Contains("--diag"))
            p.Settings.Diagnostics = line => Console.WriteLine("   " + line);
        double r0 = Math.Sqrt(p.R0.Sum(x => x * x));
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
        Console.WriteLine($"{sol.Message}: kick {sol.KickDeg:F3} deg, seed {sol.SeedSeconds:F1} s, total {sol.SolveSeconds:F1} s");
        if (sol.Nodes > 0)
            Console.WriteLine($"to orbit {sol.FinalMass / 1000.0:F2} t, burns {string.Join(" / ", sol.BurnTime.Select(b => b.ToString("F1")))} s, "
                + $"max q {sol.DynamicPressure.Max() / 1000.0:F1} kPa, max q-alpha {sol.QAlpha.Max():F0}, miss {sol.TerminalResidual[0]:F0} m / {sol.TerminalResidual[1]:F2} m/s");
        return sol.Converged ? 0 : 1;
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

        Console.WriteLine();
        Console.WriteLine(fails == 0 ? "ALL PASS" : $"{fails} FAILED");
        return fails == 0 ? 0 : 1;
    }
}

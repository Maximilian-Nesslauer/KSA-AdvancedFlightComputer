using AdvancedFlightComputer.Guidance.Numerics;
using AdvancedFlightComputer.Guidance.Numerics.Flight;
using AdvancedFlightComputer.Guidance.Scvx.Ascent;

/// <summary>
/// Validates high-TWR conditioning, s_g-only backoff merit, variable-duration ballistic coasts
/// (AscentStage.Coast), persistent ClarabelWorkspace parity and re-equilibration, and
/// BodyForce rotated PlaneNormal(phi) handling.
/// </summary>
internal static class AscentHighTwrCheck
{
    private const double G0 = 9.80665;
    private const double EarthMu = 398601877170000.0;
    private const double EarthR = 6371000.0;
    private const double EarthOmega = 7.29211585454431e-5;
    private const double MoonMu = 4.9028e12;
    private const double MoonR = 1737400.0;
    private const double MoonOmega = 2.6617e-6;

    private static int _fails;

    private static void Check(string name, bool ok, string detail = "")
    {
        Console.WriteLine($"  {(ok ? "PASS" : "FAIL")}  {name,-60} {detail}");
        if (!ok) _fails++;
    }

    private sealed class ExponentialEarthAtmosphere : AscentAtmosphere
    {
        public override Dual Density(Dual h) => 1.225 * Dual.Exp(new Dual(-h.V / 8500.0, -h.D / 8500.0));

        public override Dual SpeedOfSound(Dual h)
        {
            Dual t = 288.15 - 0.0065 * h;
            if (t.V < 216.65) t = new Dual(216.65);
            return Math.Sqrt(1.4 * 287.0) * Dual.Sqrt(t);
        }

        public override Dual Pressure(Dual h) => 101325.0 * Dual.Exp(new Dual(-h.V / 8500.0, -h.D / 8500.0));

        public override Dual DragCoefficient(Dual mach)
        {
            Dual z = (mach - 1.1) / 0.35;
            return 0.4 * (1.0 + 0.75 * Dual.Exp(-(z * z)));
        }
    }

    private static double PeakG(AscentSolution s)
    {
        double peak = 0.0;
        for (int k = 0; k < s.Nodes; k++)
            peak = Math.Max(peak, s.Thrust[k] / s.Mass[k] / G0);
        return peak;
    }

    private static AscentSolution Solve(string label, AscentProblem p, bool verbose)
    {
        AscentSolution s = AscentScvx.Solve(p, it =>
        {
            if (verbose)
                Console.WriteLine($"    iter {it.Index}: rho={it.Rho:+0.00;-0.00} {it.Status,-7} tr={it.TrustRegion:F4} "
                    + $"sig={string.Join("/", it.SigmaSeconds.Select(x => x.ToString("F1")))}s pred={it.Predicted:E1} "
                    + $"defect={it.DefectNorm:E2} path={it.PathViolation:E1} term={it.TerminalViolation:E1} J={it.Cost:E5}");
        });
        Console.WriteLine($"  {label}: {s.Message}, {s.Iterations} it ({s.Accepted} acc), {s.SolveSeconds:F2} s"
            + (s.Nodes > 0
                ? $"; {s.FinalMass / 1000.0:F2} t, stages [{string.Join(" / ", s.BurnTime.Select(b => b.ToString("F1")))}] s, "
                  + $"peak {PeakG(s):F2} g, max q {s.DynamicPressure.Max() / 1000.0:F1} kPa, "
                  + $"miss {s.TerminalResidual[0]:F1} m / {s.TerminalResidual[1]:F3} m/s"
                : ""));
        return s;
    }

    private static AscentProblem Escaton(bool withCoast)
    {
        double[] pGrid = [0.0, 50662.5, 101325.0];
        double[] tGrid = [6440000.0, 5829769.16, 5537044.20];
        const double mdot = 1520.13250268;
        const double gLimit = 16.7 * G0;
        const double s1Prop = 89843.8506;
        const double s1Dry = 5665.7630;
        const double s2Prop = 44921.9253;

        AscentStage[] stages = withCoast
            ? [
                new AscentStage(5537044.20, mdot, s1Prop, s1Dry, 8.0, pGrid, tGrid, throttleMin: 0.8, accelerationLimit: gLimit),
                new AscentStage(5537044.20, mdot, 0.4 * s2Prop, 0.0, 8.0, pGrid, tGrid, throttleMin: 0.8, liquidCarriesOver: true, accelerationLimit: gLimit),
                AscentStage.Coast(8.0, minSeconds: 0.0, maxSeconds: 1800.0, liquidCarriesOver: true),
                new AscentStage(5537044.20, mdot, 0.6 * s2Prop, 0.0, 8.0, pGrid, tGrid, throttleMin: 0.8, accelerationLimit: gLimit),
            ]
            : [
                new AscentStage(5537044.20, mdot, s1Prop, s1Dry, 8.0, pGrid, tGrid, throttleMin: 0.8, accelerationLimit: gLimit),
                new AscentStage(5537044.20, mdot, s2Prop, 0.0, 8.0, pGrid, tGrid, throttleMin: 0.8, accelerationLimit: gLimit),
            ];

        double lat = 28.5 * Math.PI / 180.0;
        double[] site = [Math.Cos(lat), 0.0, Math.Sin(lat)];
        double r0m = EarthR + 100.0;
        double[] r0 = [r0m * site[0], r0m * site[1], r0m * site[2]];
        double[] v0 = [-EarthOmega * r0[1] + 10.0 * site[0], EarthOmega * r0[0] + 10.0 * site[1], 10.0 * site[2]];
        double inc = lat, lan = 270.0 * Math.PI / 180.0;
        double rt = EarthR + 250e3;
        return new AscentProblem
        {
            Mu = EarthMu, BodyRadius = EarthR, Omega = EarthOmega,
            R0 = r0, V0 = v0, M0 = 146000.0,
            Stages = stages,
            Atmosphere = new ExponentialEarthAtmosphere(),
            GroundRadius = EarthR,
            TargetRadius = rt,
            TargetSpeed = Math.Sqrt(EarthMu / rt),
            PlaneNormal = [Math.Sin(inc) * Math.Sin(lan), -Math.Sin(inc) * Math.Cos(lan), Math.Cos(inc)],
            QMax = 195000.0,
            QAlphaMax = 3500.0,
        };
    }

    private static AscentProblem Luna2(bool withCoast, double coastSeedSeconds = double.NaN)
    {
        const double thrust = 952e3;
        const double isp = 436.0;
        double mdot = thrust / (isp * G0);
        const double prop = 60000.0;

        AscentStage[] stages = withCoast
            ? [
                new AscentStage(thrust, mdot, 0.8 * prop, 0.0, 0.0, throttleMin: 0.8, liquidCarriesOver: true),
                AscentStage.Coast(0.0, minSeconds: 0.0, maxSeconds: 1800.0, seedSeconds: coastSeedSeconds, liquidCarriesOver: true),
                new AscentStage(thrust, mdot, 0.2 * prop, 0.0, 0.0, throttleMin: 0.8),
            ]
            : [
                new AscentStage(thrust, mdot, prop, 0.0, 0.0, throttleMin: 0.8),
            ];

        double[] site = [1.0, 0.0, 0.0];
        double r0m = MoonR + 100.0;
        double[] r0 = [r0m * site[0], 0.0, 0.0];
        double[] v0 = [10.0, MoonOmega * r0m, 0.0];
        double inc = 60.0 * Math.PI / 180.0;
        double rt = MoonR + 150e3;
        return new AscentProblem
        {
            Mu = MoonMu, BodyRadius = MoonR, Omega = MoonOmega,
            R0 = r0, V0 = v0, M0 = 87100.0,
            Stages = stages,
            Atmosphere = VacuumAscentAtmosphere.Instance,
            GroundRadius = MoonR,
            TargetRadius = rt,
            TargetSpeed = Math.Sqrt(MoonMu / rt),
            PlaneNormal = [0.0, -Math.Sin(inc), Math.Cos(inc)],
            QMax = 195000.0,
            QAlphaMax = 3500.0,
        };
    }

    private static AscentProblem InterstellarMk1Coast()
    {
        const double thrust = 37e6;
        const double ve = 461000.0;
        double mdot = thrust / ve;
        const double prop = 12000.0;
        const double gLimit = 6.0 * G0;

        AscentStage[] stages =
        [
            new AscentStage(thrust, mdot, 0.8 * prop, 0.0, 6.0, throttleMin: 0.02, liquidCarriesOver: true, accelerationLimit: gLimit),
            AscentStage.Coast(6.0, minSeconds: 0.0, maxSeconds: 1800.0, liquidCarriesOver: true),
            new AscentStage(thrust, mdot, 0.2 * prop, 0.0, 6.0, throttleMin: 0.02, accelerationLimit: gLimit),
        ];

        double lat = 28.5 * Math.PI / 180.0;
        double[] site = [Math.Cos(lat), 0.0, Math.Sin(lat)];
        double r0m = EarthR + 100.0;
        double[] r0 = [r0m * site[0], r0m * site[1], r0m * site[2]];
        double[] v0 = [-EarthOmega * r0[1] + 10.0 * site[0], EarthOmega * r0[0] + 10.0 * site[1], 10.0 * site[2]];
        double inc = lat, lan = 270.0 * Math.PI / 180.0;
        double rt = EarthR + 200e3;
        return new AscentProblem
        {
            Mu = EarthMu, BodyRadius = EarthR, Omega = EarthOmega,
            R0 = r0, V0 = v0, M0 = 34660.0,
            Stages = stages,
            Atmosphere = new ExponentialEarthAtmosphere(),
            GroundRadius = EarthR,
            TargetRadius = rt,
            TargetSpeed = Math.Sqrt(EarthMu / rt),
            PlaneNormal = [Math.Sin(inc) * Math.Sin(lan), -Math.Sin(inc) * Math.Cos(lan), Math.Cos(inc)],
            QMax = 140000.0,
            QAlphaMax = 3500.0,
        };
    }

    internal static int Run(bool verbose)
    {
        _fails = 0;
        Console.WriteLine("ASCENT SCVX: high-TWR trust scaling, s_g backoff merit, coast stages, persistent Clarabel, turned BodyForce");

        TestEscaton(verbose);
        TestLuna2(verbose);
        TestInterstellar(verbose);
        TestCoastSerializationAndValidation();
        TestPersistentWorkspaceParity(verbose);
        TestTurnedBodyForcePlaneNormal(verbose);

        Console.WriteLine();
        Console.WriteLine(_fails == 0 ? "ALL PASS" : $"{_fails} FAILED");
        return _fails == 0 ? 0 : 1;
    }

    private static void TestEscaton(bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine("1. High-TWR two-stage stack (#232 Escaton, S2 TWR 11..118, 16.7 g cap -> 250 km)");
        AscentSolution cont = Solve("Escaton continuous", Escaton(withCoast: false), verbose);
        Check("Escaton continuous converges", cont.Converged, cont.Message);
        if (cont.Nodes > 0)
        {
            Check("Escaton continuous holds 16.7 g cap", PeakG(cont) <= 16.7 * 1.003, $"peak {PeakG(cont):F2} g");
            Check("Escaton continuous inserts cleanly",
                Math.Abs(cont.TerminalResidual[0]) < 150.0 && Math.Abs(cont.TerminalResidual[1]) < 0.2,
                $"dr {cont.TerminalResidual[0]:F1} m, dv {cont.TerminalResidual[1]:F3} m/s");
        }

        AscentSolution coast = Solve("Escaton burn-coast-burn", Escaton(withCoast: true), verbose);
        Check("Escaton coast converges", coast.Converged, coast.Message);
        if (coast.Nodes > 0)
        {
            Check("Escaton coast holds 16.7 g cap", PeakG(coast) <= 16.7 * 1.003, $"peak {PeakG(coast):F2} g");
            Check("Escaton coast has positive coast duration", coast.BurnTime[2] > 50.0, $"{coast.BurnTime[2]:F1} s");
            Check("Escaton coast inserts cleanly",
                Math.Abs(coast.TerminalResidual[0]) < 150.0 && Math.Abs(coast.TerminalResidual[1]) < 0.2,
                $"dr {coast.TerminalResidual[0]:F1} m, dv {coast.TerminalResidual[1]:F3} m/s");
            Check("Escaton coast delivers > 3 t more mass than continuous",
                cont.Nodes > 0 && coast.FinalMass > cont.FinalMass + 3000.0,
                $"{coast.FinalMass / 1000.0:F2} t vs {cont.FinalMass / 1000.0:F2} t (+{(coast.FinalMass - cont.FinalMass) / 1000.0:F2} t)");
        }
    }

    private static void TestLuna2(bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine("2. Airless burn-coast-burn (#231 Luna 2, 150 km 60 deg)");
        AscentSolution cont = Solve("Luna 2 continuous", Luna2(withCoast: false), verbose);
        Check("Luna 2 continuous converges", cont.Converged, cont.Message);

        AscentSolution coast = Solve("Luna 2 burn-coast-burn", Luna2(withCoast: true), verbose);
        Check("Luna 2 coast converges", coast.Converged, coast.Message);
        if (coast.Nodes > 0)
        {
            Check("Luna 2 coast inserts cleanly",
                Math.Abs(coast.TerminalResidual[0]) < 150.0 && Math.Abs(coast.TerminalResidual[1]) < 0.2,
                $"dr {coast.TerminalResidual[0]:F1} m, dv {coast.TerminalResidual[1]:F3} m/s");
            Check("Luna 2 coast duration > 300 s", coast.BurnTime[1] > 300.0, $"{coast.BurnTime[1]:F1} s");
            Check("Luna 2 coast gains > 10 t payload over continuous",
                cont.Nodes > 0 && coast.FinalMass > cont.FinalMass + 10000.0,
                $"{coast.FinalMass / 1000.0:F2} t vs {cont.FinalMass / 1000.0:F2} t (+{(coast.FinalMass - cont.FinalMass) / 1000.0:F2} t)");
        }
    }

    private static void TestInterstellar(bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine("3. Ultra-high-TWR atmospheric coast (#120 interstellar mk1, TWR 108, 6 g cap, q < 140 kPa)");
        AscentSolution s = Solve("interstellar mk1 coast", InterstellarMk1Coast(), verbose);
        Check("interstellar mk1 coast converges", s.Converged, s.Message);
        if (s.Nodes > 0)
        {
            Check("interstellar mk1 holds 6 g cap", PeakG(s) <= 6.0 * 1.003, $"peak {PeakG(s):F2} g");
            Check("interstellar mk1 stays under 140 kPa", s.DynamicPressure.Max() <= 140000.0 * 1.005,
                $"max q {s.DynamicPressure.Max() / 1000.0:F1} kPa");
            Check("interstellar mk1 inserts cleanly",
                Math.Abs(s.TerminalResidual[0]) < 150.0 && Math.Abs(s.TerminalResidual[1]) < 0.2,
                $"dr {s.TerminalResidual[0]:F1} m, dv {s.TerminalResidual[1]:F3} m/s");
        }
    }

    private static void TestCoastSerializationAndValidation()
    {
        Console.WriteLine();
        Console.WriteLine("4. AscentStage.Coast JSON round-trip and validation");
        AscentProblem orig = Luna2(withCoast: true, coastSeedSeconds: 120.0);
        string json = AscentProblemFile.ToJson(orig);
        AscentProblem back = AscentProblemFile.FromJson(json);
        AscentStage c0 = orig.Stages[1], c1 = back.Stages[1];
        Check("coast stage fields survive JSON round-trip",
            c1.IsCoast && c1.CoastMinSeconds == c0.CoastMinSeconds && c1.CoastMaxSeconds == c0.CoastMaxSeconds
            && c1.CoastSeedSeconds == c0.CoastSeedSeconds && c1.LiquidCarriesOver == c0.LiquidCarriesOver,
            $"min={c1.CoastMinSeconds}, max={c1.CoastMaxSeconds}, seed={c1.CoastSeedSeconds}, carry={c1.LiquidCarriesOver}");

        AscentProblem coastFirst = new()
        {
            Mu = orig.Mu, BodyRadius = orig.BodyRadius, Omega = orig.Omega,
            R0 = orig.R0, V0 = orig.V0, M0 = orig.M0,
            Stages = [AscentStage.Coast(0.0), orig.Stages[2]],
            Atmosphere = orig.Atmosphere, GroundRadius = orig.GroundRadius,
            TargetRadius = orig.TargetRadius, TargetSpeed = orig.TargetSpeed,
            PlaneNormal = orig.PlaneNormal,
        };
        Check("Validate rejects coast as first stage", coastFirst.Validate().Length > 0, coastFirst.Validate());

        AscentProblem orphanCarry = new()
        {
            Mu = orig.Mu, BodyRadius = orig.BodyRadius, Omega = orig.Omega,
            R0 = orig.R0, V0 = orig.V0, M0 = orig.M0,
            Stages = [orig.Stages[2], AscentStage.Coast(0.0, liquidCarriesOver: true), orig.Stages[2]],
            Atmosphere = orig.Atmosphere, GroundRadius = orig.GroundRadius,
            TargetRadius = orig.TargetRadius, TargetSpeed = orig.TargetSpeed,
            PlaneNormal = orig.PlaneNormal,
        };
        Check("Validate rejects coast carry-over without preceding carry-over", orphanCarry.Validate().Length > 0, orphanCarry.Validate());
    }

    private static void TestPersistentWorkspaceParity(bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine("5. Persistent ClarabelWorkspace parity and >= 2x stage-duration re-equilibration");
        AscentProblem satPersist = AscentCheck.SaturnV();
        satPersist.Settings.UseClarabelPersistent = true;
        AscentProblem satFresh = AscentCheck.SaturnV();
        satFresh.Settings.UseClarabelPersistent = false;
        AscentSolution sp = Solve("Saturn V persistent", satPersist, verbose);
        AscentSolution sf = Solve("Saturn V fresh", satFresh, verbose);
        Check("Saturn V persistent vs fresh parity",
            sp.Converged && sf.Converged && Math.Abs(sp.FinalMass - sf.FinalMass) < 50.0,
            $"delta mass {sp.FinalMass - sf.FinalMass:+0.0;-0.0} kg ({sp.SolveSeconds:F2} s vs {sf.SolveSeconds:F2} s)");

        // Seed coast short (300 s -> ~915 s, > 3x expansion) to force >= 2x equilibration resets.
        AscentProblem lunaPersist = Luna2(withCoast: true, coastSeedSeconds: 300.0);
        lunaPersist.Settings.UseClarabelPersistent = true;
        lunaPersist.Settings.MaxIterations = 500;
        AscentProblem lunaFresh = Luna2(withCoast: true, coastSeedSeconds: 300.0);
        lunaFresh.Settings.UseClarabelPersistent = false;
        lunaFresh.Settings.MaxIterations = 500;
        AscentSolution lp = Solve("Luna 2 short-seed persistent", lunaPersist, verbose);
        AscentSolution lf = Solve("Luna 2 short-seed fresh", lunaFresh, verbose);
        Check("Luna 2 > 2.5x coast expansion converges with persistent reset guard",
            lp.Converged && lf.Converged && lp.BurnTime[1] > 800.0 && Math.Abs(lp.FinalMass - lf.FinalMass) < 50.0,
            $"delta mass {lp.FinalMass - lf.FinalMass:+0.0;-0.0} kg, coast {lp.BurnTime[1]:F1} s ({lp.SolveSeconds:F2} s vs {lf.SolveSeconds:F2} s)");
    }

    private static void TestTurnedBodyForcePlaneNormal(bool verbose)
    {
        Console.WriteLine();
        Console.WriteLine("6. AscentDynamics.BodyForce rotated PlaneNormal(phi) and free-PlaneTurn convergence");
        BodyAeroTable liftTable = AscentLiftCheck.Table(normalPerRad: 1000.0, sideM2: 5.0);
        AscentProblem baseP = AscentAoaCheck.Problem(AscentAoaCheck.Table(attitude: true));
        var air = (KsaAscentAtmosphere)baseP.Atmosphere;
        AscentProblem p = new()
        {
            Mu = baseP.Mu, BodyRadius = baseP.BodyRadius, Omega = baseP.Omega,
            R0 = baseP.R0, V0 = baseP.V0, M0 = baseP.M0, Stages = baseP.Stages,
            Atmosphere = new KsaAscentAtmosphere(air.Atmosphere, air.Table, bodyForce: liftTable),
            GroundRadius = baseP.GroundRadius, TargetRadius = baseP.TargetRadius, TargetSpeed = baseP.TargetSpeed,
            PlaneNormal = baseP.PlaneNormal, QMax = baseP.QMax, QAlphaMax = baseP.QAlphaMax, RollAngleRad = 0.4,
        };

        const double phi = 0.35;
        double cPhi = Math.Cos(phi), sPhi = Math.Sin(phi);
        double[] n0 = p.PlaneNormal;
        double[] nRot = [cPhi * n0[0] - sPhi * n0[1], sPhi * n0[0] + cPhi * n0[1], n0[2]];
        AscentProblem pRot = new()
        {
            Mu = p.Mu, BodyRadius = p.BodyRadius, Omega = p.Omega,
            R0 = p.R0, V0 = p.V0, M0 = p.M0, Stages = p.Stages,
            Atmosphere = p.Atmosphere, GroundRadius = p.GroundRadius,
            TargetRadius = p.TargetRadius, TargetSpeed = p.TargetSpeed,
            PlaneNormal = nRot, QMax = p.QMax, QAlphaMax = p.QAlphaMax, RollAngleRad = p.RollAngleRad,
        };

        double rMag = Math.Sqrt(p.R0[0] * p.R0[0] + p.R0[1] * p.R0[1] + p.R0[2] * p.R0[2]);
        double scale = (p.BodyRadius + 10000.0) / rMag;
        double[] r = [p.R0[0] * scale, p.R0[1] * scale, p.R0[2] * scale];
        double[] v = [-p.Omega * r[1] + 250.0, p.Omega * r[0] + 350.0, 200.0];
        double[] u = [0.6, 0.5, 0.6];
        double[] aPhi = AscentAeroLaw.Acceleration(p, 0, r, v, 120000.0, u, phi);
        double[] aRot = AscentAeroLaw.Acceleration(pRot, 0, r, v, 120000.0, u, 0.0);
        double[] aZero = AscentAeroLaw.Acceleration(p, 0, r, v, 120000.0, u, 0.0);
        double maxErr = 0.0, maxDiffFromZero = 0.0;
        for (int i = 0; i < 3; i++)
        {
            maxErr = Math.Max(maxErr, Math.Abs(aPhi[i] - aRot[i]));
            maxDiffFromZero = Math.Max(maxDiffFromZero, Math.Abs(aPhi[i] - aZero[i]));
        }
        Check("BodyForce at phi matches manually rotated PlaneNormal",
            maxErr < 1e-11 && maxDiffFromZero > 1e-3,
            $"err {maxErr:E2} m/s^2, shift vs phi=0 {maxDiffFromZero:E2} m/s^2");

        // Free PlaneTurn solve with active BodyForce exercises dF/d(phi) in collocation.
        AscentSolution freeTurn = Solve("2stage_new BodyForce free PlaneTurn", p.WithPlaneTurn(-0.04, 0.04), verbose);
        Check("free-PlaneTurn BodyForce solve converges", freeTurn.Converged, freeTurn.Message);
    }
}
